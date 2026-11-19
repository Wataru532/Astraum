// Kept free of raylib headers: windows.h clashes with raylib names.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <pdh.h>
#include <pdhmsg.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>
#include "gpumon.hpp"

namespace gpumon {
namespace {
std::atomic<float> g_util{-1.0f};
std::thread g_thread;
std::mutex g_mx;
std::condition_variable g_cv;
bool g_stop = false;
std::mutex g_tmx;
std::vector<std::pair<std::string, float>> g_temps;

// thermal zones: "High Precision Temperature" is in tenths of a kelvin, "Temperature" in kelvin
struct Thermal {
    PDH_HQUERY q = nullptr;
    PDH_HCOUNTER c = nullptr;
    double scale = 0.1;
    bool tried = false;
    void open() {
        tried = true;
        if (PdhOpenQueryW(nullptr, 0, &q) != ERROR_SUCCESS) { q = nullptr; return; }
        if (PdhAddEnglishCounterW(q, L"\\Thermal Zone Information(*)\\High Precision Temperature", 0, &c) == ERROR_SUCCESS) scale = 0.1;
        else if (PdhAddEnglishCounterW(q, L"\\Thermal Zone Information(*)\\Temperature", 0, &c) == ERROR_SUCCESS) scale = 1.0;
        else {
            PdhCloseQuery(q);
            q = nullptr;
        }
    }
    void sample(std::vector<unsigned char>& buf) {
        if (!q && !tried) open();
        if (!q || PdhCollectQueryData(q) != ERROR_SUCCESS) return;
        DWORD size = 0, count = 0;
        if (PdhGetFormattedCounterArrayW(c, PDH_FMT_DOUBLE, &size, &count, nullptr) != (PDH_STATUS)PDH_MORE_DATA || size == 0) return;
        buf.resize(size);
        auto* items = (PDH_FMT_COUNTERVALUE_ITEM_W*)buf.data();
        if (PdhGetFormattedCounterArrayW(c, PDH_FMT_DOUBLE, &size, &count, items) != ERROR_SUCCESS) return;
        std::vector<std::pair<std::string, float>> out;
        for (DWORD i = 0; i < count; i++) {
            if (items[i].FmtValue.CStatus != PDH_CSTATUS_VALID_DATA && items[i].FmtValue.CStatus != PDH_CSTATUS_NEW_DATA) continue;
            double k = items[i].FmtValue.doubleValue * scale;
            if (k < 274.5 || k > 420) continue; // no sensor behind the zone (placeholders report 0 C)
            std::string name;
            for (const wchar_t* p = items[i].szName; p && *p && name.size() < 40; p++) name += (*p < 128) ? (char)*p : '?';
            out.push_back({name, (float)(k - 273.15)});
        }
        std::lock_guard<std::mutex> lk(g_tmx);
        g_temps = std::move(out);
    }
    ~Thermal() {
        if (q) PdhCloseQuery(q);
    }
};

void run() {
    wchar_t path[160];
    std::swprintf(path, 160, L"\\GPU Engine(pid_%lu_*engtype_3D)\\Utilization Percentage", GetCurrentProcessId());
    PDH_HQUERY q = nullptr;
    PDH_HCOUNTER c = nullptr;
    int age = 0;
    std::vector<unsigned char> buf, tbuf;
    Thermal thermal;
    for (;;) {
        // the wildcard is expanded when the counter is added: rebuild now and then so engines created
        // after start-up are included
        if (!q || age++ >= 15) {
            if (q) PdhCloseQuery(q);
            q = nullptr;
            age = 0;
            if (PdhOpenQueryW(nullptr, 0, &q) != ERROR_SUCCESS || PdhAddEnglishCounterW(q, path, 0, &c) != ERROR_SUCCESS) {
                if (q) PdhCloseQuery(q);
                q = nullptr;
            } else PdhCollectQueryData(q); // rate counters need a first sample
        }
        {
            std::unique_lock<std::mutex> lk(g_mx);
            if (g_cv.wait_for(lk, std::chrono::milliseconds(1000), [] { return g_stop; })) break;
        }
        thermal.sample(tbuf);
        if (!q || PdhCollectQueryData(q) != ERROR_SUCCESS) {
            g_util = -1.0f;
            continue;
        }
        DWORD size = 0, count = 0;
        PDH_STATUS st = PdhGetFormattedCounterArrayW(c, PDH_FMT_DOUBLE | PDH_FMT_NOCAP100, &size, &count, nullptr);
        if (st != (PDH_STATUS)PDH_MORE_DATA || size == 0) {
            g_util = -1.0f;
            continue;
        }
        buf.resize(size);
        auto* items = (PDH_FMT_COUNTERVALUE_ITEM_W*)buf.data();
        if (PdhGetFormattedCounterArrayW(c, PDH_FMT_DOUBLE | PDH_FMT_NOCAP100, &size, &count, items) != ERROR_SUCCESS) {
            g_util = -1.0f;
            continue;
        }
        double sum = 0;
        for (DWORD i = 0; i < count; i++)
            if (items[i].FmtValue.CStatus == PDH_CSTATUS_VALID_DATA || items[i].FmtValue.CStatus == PDH_CSTATUS_NEW_DATA)
                sum += items[i].FmtValue.doubleValue;
        g_util = (float)std::min(100.0, std::max(0.0, sum));
    }
    if (q) PdhCloseQuery(q);
}
} // namespace

void start() {
    if (g_thread.joinable()) return;
    g_stop = false;
    g_thread = std::thread(run);
}

void stop() {
    {
        std::lock_guard<std::mutex> lk(g_mx);
        g_stop = true;
    }
    g_cv.notify_all();
    if (g_thread.joinable()) g_thread.join();
}

float utilization() { return g_util; }

std::vector<std::pair<std::string, float>> temperatures() {
    std::lock_guard<std::mutex> lk(g_tmx);
    return g_temps;
}
} // namespace gpumon
