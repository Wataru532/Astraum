#include "screenshot.hpp"
#include "brand.hpp"
#include "storage.hpp"
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <ctime>
#include <deque>
#include <fstream>
#include <mutex>
#include <set>
#include <thread>
#include "rl.hpp"
#include "winshell.hpp"

namespace screenshot {
namespace {
struct ShotJob {
    Image img;
    std::filesystem::path dir;
    std::filesystem::path path;
};
std::mutex g_mx;
std::condition_variable g_cv;
std::deque<ShotJob> g_jobs;
std::set<std::wstring> g_reserved;
std::thread g_worker;
bool g_stop = false;
int g_inFlight = 0;
std::atomic<int> g_result{0};
std::filesystem::path g_override;
constexpr int MAX_PENDING = 3; // each pending capture holds one RGBA frame (about 20 MB at 2880x1800)

void workerRun() {
    for (;;) {
        ShotJob job;
        {
            std::unique_lock<std::mutex> lk(g_mx);
            g_cv.wait(lk, [] { return g_stop || !g_jobs.empty(); });
            if (g_jobs.empty()) return;
            job = g_jobs.front();
            g_jobs.pop_front();
        }
        bool ok = false;
        std::error_code e2;
        std::filesystem::create_directories(job.dir, e2);
        int size = 0;
        unsigned char* png = ExportImageToMemory(job.img, ".png", &size);
        UnloadImage(job.img);
        if (png && size > 0) {
            // written through a wide path, so a non-ASCII user / OneDrive folder works
            std::ofstream f(job.path, std::ios::binary | std::ios::trunc);
            if (f) {
                f.write((const char*)png, size);
                ok = (bool)f;
            }
        }
        if (png) MemFree(png);
        g_result = ok ? 1 : -1;
        {
            std::lock_guard<std::mutex> lk(g_mx);
            g_reserved.erase(job.path.wstring());
            g_inFlight--;
        }
        g_cv.notify_all();
    }
}

struct StopGuard {
    ~StopGuard() { finish(); }
} g_guard;
} // namespace

std::filesystem::path folder(const std::string& dataDir) {
    if (!g_override.empty()) return g_override;
    std::filesystem::path pics = winshell::picturesDir();
    std::error_code ec;
    if (!pics.empty() && std::filesystem::is_directory(pics, ec)) return pics / brand::DATA_FOLDER;
    return pathFromUtf8(dataDir) / "screenshots";
}

std::string capture(const std::string& dataDir) {
    {
        std::lock_guard<std::mutex> lk(g_mx);
        if (g_inFlight >= MAX_PENDING) return "";
    }
    rlDrawRenderBatchActive(); // flush pending 2D drawing before reading back
    Image img = LoadImageFromScreen();
    if (!img.data) return "";
    int rw = GetRenderWidth(), rh = GetRenderHeight();
    if (img.width > rw || img.height > rh) ImageCrop(&img, {0, (float)(img.height - rh), (float)rw, (float)rh});

    std::filesystem::path dir = folder(dataDir);
    char stamp[32];
    std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &now);
    std::strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", &tm);
    std::string base = std::string(brand::NAME) + "_" + stamp;
    std::string name;
    std::filesystem::path path;
    {
        std::lock_guard<std::mutex> lk(g_mx);
        if (g_inFlight >= MAX_PENDING) {
            UnloadImage(img);
            return "";
        }
        std::error_code ec;
        for (int i = 1; i <= 999; i++) {
            std::string cand = i == 1 ? (base + ".png") : (base + "_" + std::to_string(i) + ".png");
            std::filesystem::path p = dir / cand;
            if (!g_reserved.count(p.wstring()) && !std::filesystem::exists(p, ec)) {
                name = cand;
                path = p;
                break;
            }
        }
        if (name.empty()) {
            UnloadImage(img);
            return "";
        }
        g_reserved.insert(path.wstring());
        g_inFlight++;
        g_jobs.push_back({img, dir, path});
        if (!g_worker.joinable()) {
            g_stop = false;
            g_worker = std::thread(workerRun);
        }
    }
    g_cv.notify_one();
    return name;
}

void finish() {
    std::thread th;
    {
        std::lock_guard<std::mutex> lk(g_mx);
        g_stop = true;
        th = std::move(g_worker);
    }
    g_cv.notify_all();
    if (th.joinable()) th.join();
}

int takeResult() { return g_result.exchange(0); }
void setFolderOverride(const std::filesystem::path& dir) { g_override = dir; }

} // namespace screenshot
