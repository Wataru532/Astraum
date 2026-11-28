// The application state: the world being played, the designs being edited, the paths of the player's data,
// the preferences, and the view flags the screens share (which dialog is open). Pure C++: the presentation
// reads and changes it, the use cases (session.hpp, settings) keep it saved.
#pragma once
#include <string>
#include <vector>
#include "content.hpp"
#include "ports.hpp"
#include "sim.hpp"

enum class Screen { Menu, Factory, Flight, CAD };

struct App {
    Screen screen = Screen::Menu;
    World world;
    bool gameStarted = false;
    Design design;          // design currently in the factory
    Craft lastLaunched;
    int lastLaunchBody = -2; // -2: launch pad, -3: in orbit (next to lastLaunchNear); otherwise the aircraft start (body index, -1 = Earth's runway)
    int lastLaunchNear = -1; // the vessel an orbital start was placed next to (id)
    Craft craft;            // craft currently in the CAD designer
    std::string dataDir, designDir, saveFile;
    // ---- maps (one saved game each) and the factory's last design
    std::string quickFile;      // quicksave outside a map (tutorials, test flights)
    std::string mapsDir;        // <data>\maps\map_NNNN.sfs
    std::string lastDesignFile; // the design last used in the factory (kept across sessions)
    std::string mapPath;        // the current map's save ("" = no map: tutorials, flights without a map)
    bool persist = true;        // write maps and the last design (off in automated tests)
    int autosaveMin = 5;        // autosave interval [min], 0 = off
    double lastSaveReal = -1;   // GetTime() of the last save of the map
    double savedT = 0;          // the world as last saved / loaded (changes since -> unsaved)
    size_t savedN = 0;
    size_t savedBases = 0;
    int savedActive = -1, savedNextId = 0, savedNextBaseId = 0;
    int savedLaunchSite = 0, savedLaunchBase = 0;
    bool savedInfFuel = false, savedHyperspace = false;
    std::string savedMapName;
    bool saveFailed = false;
    std::string designText;     // the last design as written to lastDesignFile
    double designCheck = 0;     // GetTime() of the last look at the design
    bool closeAsk = false;      // "quit with unsaved changes?" dialog open
    bool newMapOpen = false, loadMapOpen = false; // menu dialogs
    bool settingsOpen = false;
    int lang = 0;           // i18n::Lang
    int textSize = 1;       // index into TEXT_SCALES
    int uiSize = 2;         // index into UI_SIZES (GUI size, 100 %)
    bool reverseZoom = false;
    int volumePct = 75;     // effects volume [%] (0 = off)
    bool music = true;      // background music now and then
    int musicPct = 50;      // music volume [%]
    int terrainQuality = 2; // near-ground terrain detail: 0 Low .. 3 Ultra
    int detailRange = 100;  // how far the detailed terrain reaches [% of the standard range, 50..400]
    bool physicalShading = false; // physically based shading (GGX materials, scattering sky): heavy, off by default
    bool realShadows = false;     // physically based mode: shadow maps with soft penumbrae (heavier still)
    int shadowQuality = 1;        // shadow map 0: 2048 px, 1: 4096 px
    int fpsLimit = 1;       // index into FPS_LIMITS (0 = match the display)
    bool showFps = false;   // compact frame-rate overlay (Settings)
    bool debugOpen = false; // F3 debug screen (this session)
    bool hudHidden = false; // flight HUD hidden (F1, this session)
    double hudToggleTime = -10;
    bool keysOpen = false;  // Settings -> key bindings
    bool creditsOpen = false; // Settings -> credits and licences
    std::string notice;     // short global notice (screenshot saved), drawn after the capture
    double noticeTime = -10;
    bool noticeBad = false;
    bool adaptiveDetail = true; // lower planet detail when the frame rate drops below the limit
    bool showBelts = true;      // the asteroid belt, Jupiter's Trojans and the Kuiper belt on the map (Settings)
    bool fullscreen = false;    // borderless full screen (F11, Settings); remembered for the next start
    unsigned tutorialsDone = 0; // bit i: tutorial i completed
    bool quit = false;
    std::string crashReport, crashPath; // report from a previous crash, shown once on the menu
    std::string status;     // transient status line
    double statusTime = -10;
    void setStatus(const std::string& s) { status = s; statusTime = ports::now(); }
};

extern const float TEXT_POINTS[4];
extern const float TEXT_SCALES[4];
extern const char* TEXT_SCALE_NAMES[4];
extern const float UI_SIZES[6];          // GUI size factors (Settings)
extern const float VOLUME_STEPS[5]; // (the old five volume steps: prefs written before the sliders)
extern const int FPS_LIMITS[4];

void ensureDirs(App& app);
void loadPrefs(App& app);
void savePrefs(const App& app);
extern bool g_prefsReadOnly; // automated test runs leave the player's settings alone
std::string sanitizeFileName(const std::string& s);   // a design's file name (stem): the name itself where Windows allows it
std::string legacyFileName(const std::string& s);     // the file name versions up to 1.1 gave (ASCII only, a hash for the rest)
std::string savedDesignName(const std::string& path); // the name written in a saved design / craft file ("" if none)
struct SavedFile {
    std::string path, name; // (UTF-8; name: as saved, "" if the file holds none)
};
// the saved files with this extension, sorted by name; a file an older version named is renamed to its name
std::vector<SavedFile> listSavedDesigns(const std::string& dir, const std::string& ext, size_t maxN);

std::string keysFile(const App& app); // the key bindings' file (the presentation reads / writes it)
