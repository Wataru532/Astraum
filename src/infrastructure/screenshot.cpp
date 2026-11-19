#include "screenshot.hpp"
#include "brand.hpp"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <thread>
#include "rl.hpp"
#include "winshell.hpp"

namespace screenshot {
namespace {
std::atomic<int> g_pending{0};
std::atomic<int> g_result{0};
std::filesystem::path g_override;
constexpr int MAX_PENDING = 3; // each pending capture holds one RGBA frame (about 20 MB at 2880x1800)
} // namespace

std::filesystem::path folder(const std::string& dataDir) {
    if (!g_override.empty()) return g_override;
    std::filesystem::path pics = winshell::picturesDir();
    std::error_code ec;
    if (!pics.empty() && std::filesystem::is_directory(pics, ec)) return pics / brand::DATA_FOLDER;
    return std::filesystem::path(dataDir) / "screenshots";
}

std::string capture(const std::string& dataDir) {
    if (g_pending.load() >= MAX_PENDING) return "";
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
    std::string name = base + ".png";
    std::error_code ec;
    for (int i = 2; std::filesystem::exists(dir / name, ec) && i < 100; i++) name = base + "_" + std::to_string(i) + ".png";
    std::filesystem::path path = dir / name;

    g_pending++;
    std::thread([img, dir, path]() mutable {
        bool ok = false;
        std::error_code e2;
        std::filesystem::create_directories(dir, e2);
        int size = 0;
        unsigned char* png = ExportImageToMemory(img, ".png", &size);
        UnloadImage(img);
        if (png && size > 0) {
            // written through a wide path, so a non-ASCII user / OneDrive folder works
            std::ofstream f(path, std::ios::binary | std::ios::trunc);
            if (f) {
                f.write((const char*)png, size);
                ok = (bool)f;
            }
        }
        if (png) MemFree(png);
        g_result = ok ? 1 : -1;
        g_pending--;
    }).detach();
    return name;
}

void finish() {
    for (int i = 0; i < 500 && g_pending.load() > 0; i++) std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

int takeResult() { return g_result.exchange(0); }
void setFolderOverride(const std::filesystem::path& dir) { g_override = dir; }

} // namespace screenshot
