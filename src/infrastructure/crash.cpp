// Crash reporter (Windows). Kept free of raylib headers: windows.h clashes with raylib names.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define PSAPI_VERSION 2
#include <windows.h>
#include <psapi.h>
#include <algorithm>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fstream>
#include <sstream>
#include "crash.hpp"

namespace crash {
namespace {

char g_dir[MAX_PATH] = {0};
char g_version[64] = "?";
char g_state[600] = "";
const int NCRUMB = 64, NLOG = 40;
char g_crumbs[NCRUMB][180];
char g_logs[NLOG][220];
volatile LONG g_crumbIdx = 0, g_logIdx = 0;
volatile LONG g_inHandler = 0;
char g_buf[96 * 1024];
size_t g_pos = 0;

void put(const char* fmt, ...) {
    if (g_pos >= sizeof(g_buf) - 1) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(g_buf + g_pos, sizeof(g_buf) - g_pos, fmt, ap);
    va_end(ap);
    if (n > 0) g_pos = std::min(sizeof(g_buf) - 1, g_pos + (size_t)n);
}

const char* exceptionName(DWORD code) {
    switch (code) {
    case EXCEPTION_ACCESS_VIOLATION: return "ACCESS_VIOLATION";
    case EXCEPTION_ARRAY_BOUNDS_EXCEEDED: return "ARRAY_BOUNDS_EXCEEDED";
    case EXCEPTION_DATATYPE_MISALIGNMENT: return "DATATYPE_MISALIGNMENT";
    case EXCEPTION_FLT_DIVIDE_BY_ZERO: return "FLT_DIVIDE_BY_ZERO";
    case EXCEPTION_FLT_INVALID_OPERATION: return "FLT_INVALID_OPERATION";
    case EXCEPTION_ILLEGAL_INSTRUCTION: return "ILLEGAL_INSTRUCTION";
    case EXCEPTION_IN_PAGE_ERROR: return "IN_PAGE_ERROR";
    case EXCEPTION_INT_DIVIDE_BY_ZERO: return "INT_DIVIDE_BY_ZERO";
    case EXCEPTION_PRIV_INSTRUCTION: return "PRIV_INSTRUCTION";
    case EXCEPTION_STACK_OVERFLOW: return "STACK_OVERFLOW";
    case 0xC0000409: return "STACK_BUFFER_OVERRUN / FAST_FAIL";
    case 0xE06D7363: return "C++ EXCEPTION";
    default: return "UNKNOWN";
    }
}

void describeAddress(DWORD64 addr) {
    HMODULE mod = nullptr;
    char name[MAX_PATH] = "?";
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)addr, &mod) && mod) {
        GetModuleFileNameA(mod, name, MAX_PATH);
        const char* base = strrchr(name, '\\');
        put("0x%016llx  %s+0x%llx\n", (unsigned long long)addr, base ? base + 1 : name, (unsigned long long)(addr - (DWORD64)mod));
    } else {
        put("0x%016llx  ?\n", (unsigned long long)addr);
    }
}

void stackTrace(CONTEXT ctx) {
    put("\n--- Stack trace (module + offset; resolve with build\\Astraum.map) ---\n");
#if defined(_M_X64) || defined(__x86_64__)
    for (int i = 0; i < 48 && ctx.Rip; i++) {
        put("#%02d ", i);
        describeAddress(ctx.Rip);
        DWORD64 imageBase = 0;
        PRUNTIME_FUNCTION fn = RtlLookupFunctionEntry(ctx.Rip, &imageBase, nullptr);
        if (fn) {
            PVOID handlerData = nullptr;
            DWORD64 establisher = 0;
            RtlVirtualUnwind(UNW_FLAG_NHANDLER, imageBase, ctx.Rip, fn, &ctx, &handlerData, &establisher, nullptr);
        } else {
            // leaf function: return address is on top of the stack
            if (IsBadReadPtr((void*)ctx.Rsp, sizeof(DWORD64))) break;
            ctx.Rip = *(DWORD64*)ctx.Rsp;
            ctx.Rsp += 8;
        }
    }
#else
    put("(stack walk only implemented for x64)\n");
#endif
}

void systemInfo() {
    MEMORYSTATUSEX ms{};
    ms.dwLength = sizeof ms;
    if (GlobalMemoryStatusEx(&ms))
        put("System memory: %llu MB total, %llu MB free (%lu%% in use)\n", (unsigned long long)(ms.ullTotalPhys >> 20),
            (unsigned long long)(ms.ullAvailPhys >> 20), (unsigned long)ms.dwMemoryLoad);
    PROCESS_MEMORY_COUNTERS_EX pmc{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof pmc))
        put("Process memory: %llu MB private, %llu MB working set, peak %llu MB\n", (unsigned long long)(pmc.PrivateUsage >> 20),
            (unsigned long long)(pmc.WorkingSetSize >> 20), (unsigned long long)(pmc.PeakWorkingSetSize >> 20));
    typedef LONG(WINAPI * RtlGetVersionFn)(OSVERSIONINFOW*);
    HMODULE nt = GetModuleHandleA("ntdll.dll");
    if (nt) {
        auto fn = (RtlGetVersionFn)(void*)GetProcAddress(nt, "RtlGetVersion");
        OSVERSIONINFOW v{};
        v.dwOSVersionInfoSize = sizeof v;
        if (fn && fn(&v) == 0) put("Windows %lu.%lu build %lu\n", v.dwMajorVersion, v.dwMinorVersion, v.dwBuildNumber);
    }
}

void writeReport(const char* reason, const CONTEXT* ctx, const EXCEPTION_RECORD* er) {
    if (InterlockedExchange(&g_inHandler, 1)) return; // already reporting
    g_pos = 0;
    SYSTEMTIME st;
    GetLocalTime(&st);
    put("Astraum crash report\n=========================\n");
    put("Version: %s\nTime: %04d-%02d-%02d %02d:%02d:%02d\nReason: %s\n", g_version, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute,
        st.wSecond, reason);
    if (er) {
        put("Exception: 0x%08lx %s\nAddress: ", (unsigned long)er->ExceptionCode, exceptionName(er->ExceptionCode));
        describeAddress((DWORD64)er->ExceptionAddress);
        if (er->ExceptionCode == EXCEPTION_ACCESS_VIOLATION && er->NumberParameters >= 2)
            put("Access: %s at 0x%016llx\n", er->ExceptionInformation[0] == 0 ? "read" : er->ExceptionInformation[0] == 1 ? "write" : "execute",
                (unsigned long long)er->ExceptionInformation[1]);
    }
    CONTEXT c{};
    if (ctx) c = *ctx;
    else {
        RtlCaptureContext(&c);
    }
#if defined(_M_X64) || defined(__x86_64__)
    put("\nRegisters: RIP=%016llx RSP=%016llx RBP=%016llx\n           RAX=%016llx RBX=%016llx RCX=%016llx RDX=%016llx\n",
        (unsigned long long)c.Rip, (unsigned long long)c.Rsp, (unsigned long long)c.Rbp, (unsigned long long)c.Rax,
        (unsigned long long)c.Rbx, (unsigned long long)c.Rcx, (unsigned long long)c.Rdx);
#endif
    stackTrace(c);
    put("\n--- Application state ---\n%s\n", g_state);
    put("\n--- System ---\n");
    systemInfo();
    put("\n--- Recent events (oldest first) ---\n");
    LONG ci = g_crumbIdx;
    for (int k = 0; k < NCRUMB; k++) {
        const char* s = g_crumbs[(ci + k) % NCRUMB];
        if (s[0]) put("%s\n", s);
    }
    put("\n--- Recent log ---\n");
    LONG li = g_logIdx;
    for (int k = 0; k < NLOG; k++) {
        const char* s = g_logs[(li + k) % NLOG];
        if (s[0]) put("%s\n", s);
    }
    char file[MAX_PATH], path[MAX_PATH], marker[MAX_PATH];
    snprintf(file, sizeof file, "crash-%04d%02d%02d-%02d%02d%02d.txt", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
    snprintf(path, sizeof path, "%s\\%s", g_dir, file);
    snprintf(marker, sizeof marker, "%s\\pending.txt", g_dir);
    HANDLE h = CreateFileA(path, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h != INVALID_HANDLE_VALUE) {
        DWORD wr;
        WriteFile(h, g_buf, (DWORD)g_pos, &wr, nullptr);
        CloseHandle(h);
        HANDLE m = CreateFileA(marker, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (m != INVALID_HANDLE_VALUE) {
            WriteFile(m, file, (DWORD)strlen(file), &wr, nullptr);
            CloseHandle(m);
        }
    }
}

LONG WINAPI unhandledFilter(EXCEPTION_POINTERS* ep) {
    writeReport("Unhandled exception", ep ? ep->ContextRecord : nullptr, ep ? ep->ExceptionRecord : nullptr);
    return EXCEPTION_EXECUTE_HANDLER;
}

void onTerminate() {
    char reason[300] = "std::terminate (unhandled C++ exception)";
    if (auto ex = std::current_exception()) {
        try {
            std::rethrow_exception(ex);
        } catch (const std::exception& e) {
            snprintf(reason, sizeof reason, "Unhandled C++ exception: %s", e.what());
        } catch (...) {
        }
    }
    writeReport(reason, nullptr, nullptr);
    TerminateProcess(GetCurrentProcess(), 3);
}

void onSignal(int sig) {
    writeReport(sig == SIGABRT ? "abort()" : sig == SIGFPE ? "SIGFPE" : sig == SIGILL ? "SIGILL" : "signal", nullptr, nullptr);
    TerminateProcess(GetCurrentProcess(), 3);
}

void onInvalidParameter(const wchar_t*, const wchar_t*, const wchar_t*, unsigned, uintptr_t) {
    writeReport("Invalid parameter passed to a C runtime function", nullptr, nullptr);
    TerminateProcess(GetCurrentProcess(), 3);
}

void onPureCall() {
    writeReport("Pure virtual function call", nullptr, nullptr);
    TerminateProcess(GetCurrentProcess(), 3);
}

void stamp(char* dst, size_t n, const char* fmt, va_list ap) {
    SYSTEMTIME st;
    GetLocalTime(&st);
    int k = snprintf(dst, n, "%02d:%02d:%02d.%03d  ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
    if (k > 0 && (size_t)k < n) vsnprintf(dst + k, n - k, fmt, ap);
}
} // namespace

void init(const std::string& dir, const char* version) {
    snprintf(g_dir, sizeof g_dir, "%s", dir.c_str());
    snprintf(g_version, sizeof g_version, "%s", version);
    CreateDirectoryA(g_dir, nullptr);
    ULONG guarantee = 64 * 1024; // leave stack for the handler after a stack overflow
    SetThreadStackGuarantee(&guarantee);
    SetUnhandledExceptionFilter(unhandledFilter);
    std::set_terminate(onTerminate);
    std::signal(SIGABRT, onSignal);
    std::signal(SIGFPE, onSignal);
    std::signal(SIGILL, onSignal);
    _set_invalid_parameter_handler(onInvalidParameter);
    _set_purecall_handler(onPureCall);
}

void breadcrumb(const char* fmt, ...) {
    LONG i = (InterlockedIncrement(&g_crumbIdx) - 1) % NCRUMB;
    va_list ap;
    va_start(ap, fmt);
    stamp(g_crumbs[i], sizeof g_crumbs[i], fmt, ap);
    va_end(ap);
}

void logLine(const char* line) {
    LONG i = (InterlockedIncrement(&g_logIdx) - 1) % NLOG;
    snprintf(g_logs[i], sizeof g_logs[i], "%s", line);
}

void setState(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_state, sizeof g_state, fmt, ap);
    va_end(ap);
}

bool pendingReport(std::string& text, std::string& path) {
    std::string marker = std::string(g_dir) + "\\pending.txt";
    std::ifstream m(marker);
    if (!m) return false;
    std::string file;
    std::getline(m, file);
    // only plain file names produced by this module are accepted
    if (file.size() < 10 || file.size() > 60 || file.find_first_of("\\/:") != std::string::npos || file.rfind("crash-", 0) != 0) return false;
    path = std::string(g_dir) + "\\" + file;
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::ostringstream os;
    os << f.rdbuf();
    text = os.str();
    if (text.size() > 256 * 1024) text.resize(256 * 1024);
    return true;
}

void markSeen() {
    std::string marker = std::string(g_dir) + "\\pending.txt";
    DeleteFileA(marker.c_str());
}

void selfTest() {
    breadcrumb("crash self-test requested");
    volatile int* p = nullptr;
    *p = 42;
    for (;;) {
    }
}

size_t processMemoryBytes() {
    PROCESS_MEMORY_COUNTERS_EX pmc{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof pmc)) return (size_t)pmc.PrivateUsage;
    return 0;
}

} // namespace crash
