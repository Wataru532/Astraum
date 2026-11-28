// Use cases of a game session: maps (one saved game each, named by the player), autosave bookkeeping and the
// factory's last design. The dialogs around them are in presentation/map_dialogs.cpp.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <atomic>
#include <thread>
#include <map>
#include "session.hpp"
#include "brand.hpp"
#include "i18n.hpp"
#include "storage.hpp"
#include "ports.hpp"

namespace fs = std::filesystem;

static long long fileTime(const std::string& path) {
    std::error_code ec;
    fs::path p = pathFromUtf8(path);
    if (p.empty()) return 0;
    auto ft = fs::last_write_time(p, ec);
    if (ec) return 0;
    auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        ft - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
    return (long long)std::chrono::system_clock::to_time_t(sctp);
}

static bool isOwnedMapPath(const App& app, const std::string& path) {
    if (path.empty() || app.mapsDir.empty()) return false;
    std::error_code ec1, ec2;
    fs::path p = fs::weakly_canonical(pathFromUtf8(path), ec1);
    fs::path dir = fs::weakly_canonical(pathFromUtf8(app.mapsDir), ec2);
    if (ec1 || ec2 || p.empty() || dir.empty() || p.parent_path() != dir) return false;
    std::string fn = pathToUtf8(p.filename());
    return fn.rfind("map_", 0) == 0 && pathToUtf8(p.extension()) == ".sfs";
}

std::string localTimeText(long long t) {
    if (t <= 0) return "-";
    std::time_t tt = (std::time_t)t;
    std::tm lt{};
    localtime_s(&lt, &tt);
    char b[40];
    std::strftime(b, sizeof b, "%Y-%m-%d %H:%M", &lt);
    return b;
}

std::string trimmed(const std::string& s) {
    size_t a = s.find_first_not_of(" \t"), e = s.find_last_not_of(" \t");
    return a == std::string::npos ? std::string() : s.substr(a, e - a + 1);
}

// the name and the vessel count, read without loading the game
MapInfo readInfo(const std::string& path) {
    MapInfo m;
    m.path = path;
    m.savedAt = fileTime(path);
    std::ifstream f(pathFromUtf8(path), std::ios::binary);
    std::string line;
    int n = 0;
    while (std::getline(f, line) && n++ < 2000000) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.rfind("mapname ", 0) == 0) m.name = trimmed(line.substr(8));
        else if (line == "vessel") m.vessels++; // (a vessel block starts with this line alone)
    }
    if (m.name.empty()) m.name = tr("Saved game");
    return m;
}

void notice(App& app, const std::string& s, bool bad) {
    app.notice = s;
    app.noticeBad = bad;
    app.noticeTime = ports::now();
}

// the file is written on a worker (the world is turned into text on the main thread first): no hitch
static std::thread g_writer;
static std::atomic<bool> g_writeFailed{false};

std::string thumbPath(const std::string& mapPath) {
    fs::path p = pathFromUtf8(mapPath);
    p.replace_extension(".png");
    return pathToUtf8(p);
}

bool renameMap(App& app, const std::string& path, const std::string& name0) {
    std::string name = cleanLineText(trimmed(name0), 120);
    if (name.empty() || !app.persist || !isOwnedMapPath(app, path)) return false;
    finishSaves();
    if (path == app.mapPath && app.gameStarted) {
        // the map being played: its world carries the name
        app.world.mapName = name;
        return saveMap(app, false, true);
    }
    // another map: only its name line changes (bounded read)
    std::string text;
    if (!readFileLimited(path, text, 256u << 20)) return false;
    size_t eol = text.find('\n');
    if (!brand::hasTag(text.substr(0, eol == std::string::npos ? 0 : eol), "SAVE") || eol == std::string::npos) return false;
    size_t a = text.find("\nmapname ");
    size_t firstVessel = text.find("\nvessel");
    if (a != std::string::npos && (firstVessel == std::string::npos || a < firstVessel)) {
        size_t e = text.find('\n', a + 1);
        text.replace(a + 1, (e == std::string::npos ? text.size() : e) - (a + 1), "mapname " + name);
    } else text.insert(eol + 1, "mapname " + name + "\n");
    return writeFileAtomic(path, text);
}

std::vector<MapInfo> listMaps(const App& app) {
    finishSaves(); // avoid Windows sharing-violation against an in-flight background rename
    std::vector<MapInfo> out;
    std::error_code ec;
    fs::path dir = pathFromUtf8(app.mapsDir);
    if (dir.empty() || !fs::is_directory(dir, ec)) return out;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        std::error_code ec2;
        if (!it->is_regular_file(ec2) || pathToUtf8(it->path().extension()) != ".sfs") continue;
        std::string fn = pathToUtf8(it->path().filename());
        if (fn.rfind("map_", 0) != 0) continue;
        std::string p = pathToUtf8(it->path());
        if (p.empty()) continue;
        out.push_back(readInfo(p));
        if (out.size() >= 500) break;
    }
    std::sort(out.begin(), out.end(), [](const MapInfo& a, const MapInfo& b) { return a.savedAt > b.savedAt; });
    return out;
}

void markMapSaved(App& app) {
    const World& W = app.world;
    app.savedT = W.t;
    app.savedN = W.vessels.size();
    app.savedBases = W.bases.size();
    app.savedActive = W.active;
    app.savedNextId = W.nextId;
    app.savedNextBaseId = W.nextBaseId;
    app.savedLaunchSite = W.settings.launchSite;
    app.savedLaunchBase = W.settings.launchBase;
    app.savedInfFuel = W.settings.infiniteFuel;
    app.savedHyperspace = W.settings.hyperspace;
    app.savedMapName = W.mapName;
    app.saveFailed = false;
    app.lastSaveReal = ports::now();
}

bool mapDirty(const App& app) {
    if (app.mapPath.empty() || !app.gameStarted) return false;
    if (app.saveFailed) return true;
    const World& W = app.world;
    return W.t != app.savedT || W.vessels.size() != app.savedN || W.bases.size() != app.savedBases ||
           W.active != app.savedActive || W.nextId != app.savedNextId || W.nextBaseId != app.savedNextBaseId ||
           W.settings.launchSite != app.savedLaunchSite || W.settings.launchBase != app.savedLaunchBase ||
           W.settings.infiniteFuel != app.savedInfFuel || W.settings.hyperspace != app.savedHyperspace ||
           W.mapName != app.savedMapName;
}

void finishSaves() {
    if (g_writer.joinable()) g_writer.join();
}

bool saveMap(App& app, bool autosave, bool wait) {
    if (app.mapPath.empty() || !app.persist) return false;
    double t0 = ports::now();
    std::string text = app.world.serialize();
    finishSaves(); // (one write at a time)
    bool ok = true;
    if (wait) ok = writeFileAtomic(app.mapPath, text);
    else g_writer = std::thread([path = app.mapPath, t = std::move(text)] {
        if (!writeFileAtomic(path, t)) g_writeFailed = true;
    });
    if (ok) {
        markMapSaved(app);
        ports::presentation().mapSaved(app.mapPath); // (its picture: the view shown now)
        notice(app, autosave ? trf("Autosaved \"{0}\"", app.world.mapName) : trf("Saved \"{0}\"", app.world.mapName));
        ports::log("map saved in %.1f ms (%s)", (ports::now() - t0) * 1000, autosave ? "autosave" : "save");
    } else {
        app.saveFailed = true;
        notice(app, tr("Could not save the map"), true);
        app.lastSaveReal = ports::now(); // (not again at once)
    }
    return ok;
}

bool createMap(App& app, const std::string& name0) {
    std::string name = cleanLineText(trimmed(name0), 120);
    if (name.empty() || !app.persist) return false;
    if (mapDirty(app)) saveMap(app, false); // the game being left
    std::error_code ec;
    fs::path dir = pathFromUtf8(app.mapsDir);
    fs::create_directories(dir, ec);
    // a generated ASCII file name: the player's name may hold any character
    std::string path;
    for (int i = 1; i < 100000 && path.empty(); i++) {
        char b[32];
        std::snprintf(b, sizeof b, "map_%04d.sfs", i);
        fs::path fp = dir / b;
        if (!fs::exists(fp, ec)) path = pathToUtf8(fp);
    }
    if (path.empty()) return false;
    Settings s = app.world.settings;
    app.world.newGame(s);
    app.world.missionLog.clear();
    app.world.mapName = name;
    ports::presentation().stopTutorial();
    ports::presentation().worldReplaced();
    app.mapPath = path;
    app.saveFile = path;
    app.gameStarted = true;
    restoreDesign(app);
    if (!saveWorld(app.world, path)) {
        app.mapPath.clear();
        app.saveFile = app.quickFile;
        notice(app, tr("Could not save the map"), true);
        return false;
    }
    markMapSaved(app);
    app.screen = Screen::Factory;
    return true;
}

bool loadMap(App& app, const std::string& path) {
    if (mapDirty(app) && path != app.mapPath) saveMap(app, false);
    finishSaves();
    if (!loadWorld(app.world, path)) {
        notice(app, tr("Could not load the map"), true);
        return false;
    }
    if (app.world.mapName.empty()) app.world.mapName = readInfo(path).name;
    ports::presentation().stopTutorial();
    ports::presentation().worldReplaced();
    app.mapPath = path;
    app.saveFile = path;
    app.gameStarted = true;
    restoreDesign(app);
    markMapSaved(app);
    return true;
}

bool deleteMap(App& app, const std::string& path) {
    finishSaves();
    std::error_code ec;
    if (!isOwnedMapPath(app, path)) return false; // (only our own files)
    bool ok = fs::remove(pathFromUtf8(path), ec);
    fs::remove(pathFromUtf8(path + ".tmp"), ec);
    fs::remove(pathFromUtf8(thumbPath(path)), ec); // (its picture)
    if (ok && path == app.mapPath) {
        // the map being played is gone: back to an empty game
        ports::presentation().stopTutorial();
        ports::presentation().worldReplaced();
        app.mapPath.clear();
        app.saveFile = app.quickFile;
        app.gameStarted = false;
        Settings s = app.world.settings;
        app.world.newGame(s);
    }
    return ok;
}

void leaveMap(App& app) {
    if (app.mapPath.empty()) return;
    if (mapDirty(app)) saveMap(app, false);
    app.mapPath.clear();
    app.saveFile = app.quickFile;
}

void serviceSaves(App& app) {
    if (g_writeFailed.exchange(false)) {
        app.saveFailed = true;
        notice(app, tr("Could not save the map"), true);
    }
    ports::presentation().serviceThumbs();
}

bool quickSave(App& app) {
    if (!app.mapPath.empty()) return saveMap(app, false);
    return saveWorld(app.world, app.saveFile);
}

void persistDesign(App& app, bool force) {
    if (!app.persist || ports::presentation().tutorialActive() || app.lastDesignFile.empty()) return;
    double now = ports::now();
    if (!force && now - app.designCheck < 2.0) return;
    app.designCheck = now;
    if (app.design.stack.empty()) return;
    std::string t = designToText(app.design);
    if (t == app.designText) return;
    if (writeFileAtomic(app.lastDesignFile, t)) app.designText = t;
}

void restoreDesign(App& app) {
    Design d;
    if (!app.designText.empty() && designFromText(d, app.designText) && !d.stack.empty()) app.design = d;
    else if (app.design.stack.empty() || ports::presentation().tutorialActive()) app.design = builtinTemplates()[0];
}

void loadLastDesign(App& app) {
    Design d;
    if (!app.lastDesignFile.empty() && loadDesign(d, app.lastDesignFile) && !d.stack.empty()) {
        app.design = d;
        app.designText = designToText(d);
    }
}

void migrateQuicksave(App& app) {
    // the quicksave of earlier versions becomes the first map (the old file stays where it was)
    std::error_code ec;
    fs::path dir = pathFromUtf8(app.mapsDir), qf = pathFromUtf8(app.quickFile);
    if (dir.empty() || fs::exists(dir, ec)) return;
    fs::create_directories(dir, ec);
    if (qf.empty() || !fs::exists(qf, ec)) return;
    fs::copy_file(qf, dir / "map_0001.sfs", ec);
}

