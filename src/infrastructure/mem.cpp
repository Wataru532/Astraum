#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define PSAPI_VERSION 2
#include <windows.h>
#include <psapi.h>
#include <malloc.h>
#include "mem.hpp"

namespace mem {
namespace {
Status g_st;
double g_lastSample = -1e9;
} // namespace

Pressure update(double now) {
    if (now - g_lastSample < 0.5) return Normal;
    g_lastSample = now;
    PROCESS_MEMORY_COUNTERS_EX pmc{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof pmc)) g_st.process = pmc.PrivateUsage;
    MEMORYSTATUSEX ms{};
    ms.dwLength = sizeof ms;
    if (GlobalMemoryStatusEx(&ms)) {
        g_st.sysAvail = (size_t)ms.ullAvailPhys;
        g_st.sysTotal = (size_t)ms.ullTotalPhys;
    }
    // the system running out of RAM causes paging just like exceeding our own budget
    Pressure p = Normal;
    if (g_st.process > SOFT || (g_st.sysAvail && g_st.sysAvail < 1536 * MB)) p = Soft;
    if (g_st.process > BUDGET - 256 * MB || (g_st.sysAvail && g_st.sysAvail < 640 * MB)) p = Hard;
    g_st.pressure = p;
    if (p == Normal) return Normal;
    if (now - g_st.lastTrim < (p == Hard ? 2.0 : 8.0)) return Normal;
    g_st.lastTrim = now;
    g_st.trims++;
    return p;
}

const Status& status() { return g_st; }

void compactHeap() {
    _heapmin();
    HeapCompact(GetProcessHeap(), 0);
}
} // namespace mem
