// Configurable key / mouse bindings (Settings -> Key bindings). Every action has two slots; a slot is a
// keyboard key or a mouse button with optional Ctrl / Shift / Alt. Saved to %APPDATA%\Astraum\keys.txt.
#pragma once
#include <cstdint>
#include <string>

namespace keys {

enum class A {
    // flight
    ThrottleUp, ThrottleDown, ThrottleFull, ThrottleCut,
    PitchDown, PitchUp, YawLeft, YawRight, RollLeft, RollRight,
    TransFwd, TransBack, TransUp, TransDown, TransLeft, TransRight,
    Stage, SAS, RCS, Legs, Chute, Sail, Solar, Hyperspace, MissionLog, NightVision, Flaps, BayDoors, PayloadRelease, PayloadCapture,
    MapView, WarpFaster, WarpSlower, WarpStop, FocusVessel, QuickSave, QuickLoad, ToggleHUD,
    // editors (factory and CAD)
    Undo, Redo, Delete, Duplicate, SelectAll, SaveDesign, ToolSelect, ToolMove, ToolRotate,
    // everywhere
    Screenshot, DebugScreen, FitView, Fullscreen,
    COUNT
};
constexpr int N = (int)A::COUNT;
constexpr int SLOTS = 2;

enum Ctx : uint8_t { Flight = 1, Editor = 2, Global = 3 };
enum Mod : uint8_t { Ctrl = 1, Shift = 2, Alt = 4 };
constexpr int MOUSE_BASE = 1000; // code = MOUSE_BASE + raylib mouse button

struct Binding {
    int code = 0;     // 0 = unbound; raylib KeyboardKey, or MOUSE_BASE + MouseButton
    uint8_t mods = 0; // required modifiers; 0 = modifiers are ignored (so Shift-throttle + Space still stages)
    bool operator==(const Binding& o) const { return code == o.code && mods == o.mods; }
};

struct Info {
    const char* id;    // file key
    const char* name;  // English UI name (translated)
    Ctx ctx;
    Binding def[SLOTS];
};
const Info& info(A a);
Binding& slot(A a, int i);

// input queries; keyboard bindings are ignored while a text field has focus or a binding is being captured
bool pressed(A a);
bool down(A a);
float axis(A plus, A minus); // down(plus) - down(minus)

std::string name(const Binding& b);  // "Ctrl+Shift+Z", "Mouse 4 (Back)", localised
std::string label(A a);              // first bound slot, "" when unbound
std::string tip(const char* en, A a); // "Map view (M)", localised

void resetDefaults();
bool load(const std::string& path);   // missing file = defaults
bool save(const std::string& path);
// another action in an overlapping context already uses binding b (returns the action or COUNT)
A conflict(A self, const Binding& b);

// capture: start, then call every frame; returns true when a binding was recorded into out
// (Esc cancels -> capturing() becomes false without a result)
void beginCapture();
bool capturing();
void cancelCapture();
bool pollCapture(Binding& out);

// call once at the start of every frame (activates an injected press for exactly that frame)
void newFrame();
void setSuspended(bool s); // no binding reacts (an application-wide dialog is open)
// automated tests: report b as pressed during the next frame
void inject(const Binding& b);

} // namespace keys
