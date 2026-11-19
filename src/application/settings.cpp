// Preferences (prefs.txt) and the data folders. The key bindings are loaded by the presentation (keys.cpp) from
// keysFile(); the system language comes through the platform port.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include "app.hpp"
#include "brand.hpp"
#include "i18n.hpp"
#include "ports.hpp"

// Text sizes in points (1 pt = 96/72 logical px). UI body text is authored at 14 px,
// so "Standard" = 11 pt = 14.67 px; the other steps keep the same ratios.
const float TEXT_POINTS[4] = {9.0f, 11.0f, 13.0f, 15.0f};
const float TEXT_SCALES[4] = {9.0f * 96 / 72 / 14, 11.0f * 96 / 72 / 14, 13.0f * 96 / 72 / 14, 15.0f * 96 / 72 / 14};
const char* TEXT_SCALE_NAMES[4] = {"Small", "Standard", "Large", "Extra large"};
const float VOLUME_STEPS[5] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
const float UI_SIZES[6] = {0.75f, 0.9f, 1.0f, 1.1f, 1.25f, 1.5f};
const int FPS_LIMITS[4] = {30, 60, 120, 0};

void loadPrefs(App& app) {
    // default: follow the system's display language
    app.lang = ports::systemLanguage();
    app.textSize = 1;
    std::ifstream f(app.dataDir + "\\prefs.txt");
    std::string key;
    int v, version = 1, textSize = 1;
    int oldVol = -1, oldMusicVol = -1, sfxPct = -1, musicPct = -1;
    while (f >> key >> v) {
        if (key == "lang" && v >= 0 && v < i18n::COUNT) app.lang = v;
        if (key == "textsize" && v >= 0 && v < 4) textSize = v;
        if (key == "version") version = v;
        if (key == "reversezoom") app.reverseZoom = v != 0;
        if (key == "volume" && v >= 0 && v < 5) oldVol = v; // (five steps, before the sliders)
        if (key == "musicvolume" && v >= 0 && v < 5) oldMusicVol = v;
        if (key == "sfxpct" && v >= 0 && v <= 100) sfxPct = v;
        if (key == "musicpct" && v >= 0 && v <= 100) musicPct = v;
        if (key == "music" && v >= 0) app.music = v != 0;
        if (key == "terrainq" && v >= 0 && v < 4) app.terrainQuality = v;
        if (key == "detailrange" && v >= 50 && v <= 400) app.detailRange = v;
        if (key == "pbr") app.physicalShading = v != 0;
        if (key == "autosave" && (v == 0 || v == 1 || v == 3 || v == 5 || v == 10)) app.autosaveMin = v;
        if (key == "fpslimit" && v >= 0 && v < 4) app.fpsLimit = v;
        if (key == "showfps") app.showFps = v != 0;
        if (key == "adaptive") app.adaptiveDetail = v != 0;
        if (key == "belts") app.showBelts = v != 0;
        if (key == "fullscreen") app.fullscreen = v != 0;
        if (key == "uisize" && v >= 0 && v < 6) app.uiSize = v;
        if (key == "tutorials2" && v >= 0) app.tutorialsDone = (unsigned)v & 0xFFFFu; // (order of 2026-11-12; older records dropped)
    }
    // volumes in per cent; a file from before the sliders keeps its step
    if (sfxPct >= 0) app.volumePct = sfxPct;
    else if (oldVol >= 0) app.volumePct = (int)std::lround(VOLUME_STEPS[oldVol] * 100);
    if (musicPct >= 0) app.musicPct = musicPct;
    else if (oldMusicVol >= 0) app.musicPct = (int)std::lround(VOLUME_STEPS[oldMusicVol] * 100);
    // version 2 redefined the sizes in points (Standard = 11 pt): older choices restart from Standard
    app.textSize = version >= 2 ? textSize : 1;
}

std::string keysFile(const App& app) { return app.dataDir + "\\keys.txt"; }

bool g_prefsReadOnly = false;

void savePrefs(const App& app) {
    if (g_prefsReadOnly) return;
    std::ofstream f(app.dataDir + "\\prefs.txt");
    f << "version 2\nlang " << app.lang << "\ntextsize " << app.textSize << "\nreversezoom " << (app.reverseZoom ? 1 : 0) << "\nsfxpct "
      << std::clamp(app.volumePct, 0, 100) << "\nmusic " << (app.music ? 1 : 0) << "\nmusicpct " << std::clamp(app.musicPct, 0, 100) << "\nterrainq " << app.terrainQuality << "\ndetailrange " << std::clamp(app.detailRange, 50, 400) << "\npbr " << (app.physicalShading ? 1 : 0) << "\nautosave " << app.autosaveMin << "\nfpslimit " << app.fpsLimit << "\nshowfps " << (app.showFps ? 1 : 0) << "\nadaptive " << (app.adaptiveDetail ? 1 : 0) << "\nbelts " << (app.showBelts ? 1 : 0) << "\nfullscreen " << (app.fullscreen ? 1 : 0) << "\nuisize " << app.uiSize
      << "\ntutorials2 " << (app.tutorialsDone & 0xFFFFu) << "\n";
}

void ensureDirs(App& app) {
    namespace fs = std::filesystem;
    const char* ad = std::getenv("APPDATA");
    fs::path base = ad ? fs::path(ad) / brand::DATA_FOLDER : fs::current_path() / (std::string(brand::DATA_FOLDER) + "Data");
    std::error_code ec;
    fs::create_directories(base / "designs", ec);
    app.dataDir = base.string();
    app.designDir = (base / "designs").string();
    app.saveFile = (base / "quicksave.sfs").string();
    app.quickFile = app.saveFile;
    app.mapsDir = (base / "maps").string();
    app.lastDesignFile = (base / "last_design.sfd").string();
}
