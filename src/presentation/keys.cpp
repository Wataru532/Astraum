#include "keys.hpp"
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include "rl.hpp"
#include "storage.hpp"
#include "ui.hpp"

namespace keys {
namespace {

constexpr Binding K(int code, uint8_t mods = 0) { return Binding{code, mods}; }
constexpr Binding M(int button) { return Binding{MOUSE_BASE + button, 0}; }
constexpr Binding NONE{};

// clang-format off
const Info INFO[N] = {
    {"throttle_up",    "Throttle up",           Flight, {K(KEY_LEFT_SHIFT), NONE}},
    {"throttle_down",  "Throttle down",         Flight, {K(KEY_LEFT_CONTROL), NONE}},
    {"throttle_full",  "Full throttle",         Flight, {K(KEY_Z), NONE}},
    {"throttle_cut",   "Cut throttle",          Flight, {K(KEY_X), NONE}},
    {"pitch_down",     "Pitch down",            Flight, {K(KEY_W), NONE}},
    {"pitch_up",       "Pitch up",              Flight, {K(KEY_S), NONE}},
    {"yaw_left",       "Yaw left",              Flight, {K(KEY_A), NONE}},
    {"yaw_right",      "Yaw right",             Flight, {K(KEY_D), NONE}},
    {"roll_left",      "Roll left",             Flight, {K(KEY_Q), NONE}},
    {"roll_right",     "Roll right",            Flight, {K(KEY_E), NONE}},
    {"trans_fwd",      "Translate forward",     Flight, {K(KEY_H), NONE}},
    {"trans_back",     "Translate back",        Flight, {K(KEY_N), NONE}},
    {"trans_up",       "Translate up",          Flight, {K(KEY_I), NONE}},
    {"trans_down",     "Translate down",        Flight, {K(KEY_K), NONE}},
    {"trans_left",     "Translate left",        Flight, {K(KEY_J), NONE}},
    {"trans_right",    "Translate right",       Flight, {K(KEY_L), NONE}},
    {"stage",          "Activate next stage",   Flight, {K(KEY_SPACE), NONE}},
    {"sas",            "SAS on / off",          Flight, {K(KEY_T), NONE}},
    {"rcs",            "RCS on / off",          Flight, {K(KEY_R), NONE}},
    {"legs",           "Landing legs",          Flight, {K(KEY_G), NONE}},
    {"chute",          "Parachute",             Flight, {K(KEY_P), NONE}},
    {"sail",           "Solar sail",            Flight, {K(KEY_B), NONE}},
    {"solar",          "Solar panels",          Flight, {K(KEY_U), NONE}},
    {"hyperspace",     "Hyperspace",            Flight, {K(KEY_Y), NONE}},
    {"mission_log",    "Mission log",           Flight, {K(KEY_O), NONE}},
    {"night_vision",   "Night vision",          Flight, {K(KEY_V), NONE}},
    {"flaps",          "Flaps (next setting)",  Flight, {K(KEY_F), NONE}},
    {"bay_doors",      "Payload bay doors",     Flight, {K(KEY_C), NONE}},
    {"payload_release","Release payload",       Flight, {NONE, NONE}},
    {"payload_capture","Capture with the arm",  Flight, {NONE, NONE}},
    {"map",            "Map view",              Flight, {K(KEY_M), NONE}},
    {"warp_faster",    "Time warp faster",      Flight, {K(KEY_PERIOD), NONE}},
    {"warp_slower",    "Time warp slower",      Flight, {K(KEY_COMMA), NONE}},
    {"warp_stop",      "Real time",             Flight, {K(KEY_SLASH), NONE}},
    {"focus_vessel",   "Map: focus vessel",     Flight, {K(KEY_TAB), NONE}},
    {"quicksave",      "Quicksave",             Flight, {K(KEY_F5), NONE}},
    {"quickload",      "Quickload",             Flight, {K(KEY_F9), NONE}},
    {"toggle_hud",     "Show / hide HUD",       Flight, {K(KEY_F1), NONE}},
    {"undo",           "Undo",                  Editor, {K(KEY_Z, Ctrl), M(MOUSE_BUTTON_SIDE)}},
    {"redo",           "Redo",                  Editor, {K(KEY_Z, Ctrl | Shift), M(MOUSE_BUTTON_EXTRA)}},
    {"delete",         "Delete selection",      Editor, {K(KEY_DELETE), K(KEY_BACKSPACE)}},
    {"duplicate",      "Duplicate selection",   Editor, {K(KEY_D, Ctrl), NONE}},
    {"select_all",     "Select all",            Editor, {K(KEY_A, Ctrl), NONE}},
    {"save_design",    "Save design",           Editor, {K(KEY_S, Ctrl), NONE}},
    {"tool_select",    "Select tool",           Editor, {K(KEY_Q), NONE}},
    {"tool_move",      "Move tool",             Editor, {K(KEY_M), NONE}},
    {"tool_rotate",    "Rotate tool",           Editor, {K(KEY_R), NONE}},
    {"screenshot",     "Take screenshot",       Global, {K(KEY_F2), NONE}},
    {"debug",          "Debug screen",          Global, {K(KEY_F3), NONE}},
    {"fit_view",       "Fit view",              Global, {K(KEY_F6), NONE}},
    {"fullscreen",     "Full screen",           Global, {K(KEY_F11), NONE}},
};
// clang-format on

Binding g_bind[N][SLOTS];
bool g_init = false;
bool g_capturing = false;
int g_pendingMod = 0; // modifier key pressed alone during capture
Binding g_fakeNext, g_fake; // injected press (tests): queued, active this frame

void ensure() {
    if (g_init) return;
    g_init = true;
    resetDefaults();
}

bool isModifier(int k) {
    return k == KEY_LEFT_SHIFT || k == KEY_RIGHT_SHIFT || k == KEY_LEFT_CONTROL || k == KEY_RIGHT_CONTROL || k == KEY_LEFT_ALT ||
           k == KEY_RIGHT_ALT || k == KEY_LEFT_SUPER || k == KEY_RIGHT_SUPER;
}
uint8_t modOfKey(int k) {
    if (k == KEY_LEFT_CONTROL || k == KEY_RIGHT_CONTROL) return Ctrl;
    if (k == KEY_LEFT_SHIFT || k == KEY_RIGHT_SHIFT) return Shift;
    if (k == KEY_LEFT_ALT || k == KEY_RIGHT_ALT) return Alt;
    return 0;
}
uint8_t heldMods() {
    uint8_t m = 0;
    if (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) m |= Ctrl;
    if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) m |= Shift;
    if (IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT)) m |= Alt;
    return m;
}
bool isMouse(int code) { return code >= MOUSE_BASE; }
bool validCode(int code) {
    return code == 0 || (code >= 32 && code <= KEY_KB_MENU) || (code >= MOUSE_BASE + MOUSE_BUTTON_MIDDLE && code <= MOUSE_BASE + MOUSE_BUTTON_BACK);
}
bool g_suspended = false; // an application-wide dialog is open
bool blockedNow(int code) {
    if (g_capturing || g_suspended) return true;
    return !isMouse(code) && ui::keyboardCaptured(); // typing into a text field
}
bool overlap(Ctx a, Ctx b) { return (a & b) != 0; }

// key names are looked up as "key:<name>" so they cannot collide with UI strings ("Delete" -> "Entf")
std::string keyText(const char* n) {
    std::string k = std::string("key:") + n;
    std::string t = tr(k);
    return t == k ? std::string(n) : t;
}

// a binding without modifiers is shadowed by a binding of the same key that names exactly the
// modifiers held now (Ctrl+S saves instead of also pitching up)
bool shadowed(Ctx ctx, int code, uint8_t cur) {
    if (cur == 0) return false;
    for (int a = 0; a < N; a++) {
        if (!overlap(INFO[a].ctx, ctx)) continue;
        for (int s = 0; s < SLOTS; s++) {
            const Binding& o = g_bind[a][s];
            if (o.code == code && o.mods != 0 && o.mods == cur) return true;
        }
    }
    return false;
}
bool modsMatchPressed(const Binding& b, Ctx ctx) {
    uint8_t cur = heldMods() & (uint8_t)~modOfKey(b.code);
    if (b.mods) return cur == b.mods;
    return !shadowed(ctx, b.code, cur);
}

const char* keyName(int k) {
    switch (k) {
    case KEY_SPACE: return "Space";
    case KEY_APOSTROPHE: return "'";
    case KEY_COMMA: return ",";
    case KEY_MINUS: return "-";
    case KEY_PERIOD: return ".";
    case KEY_SLASH: return "/";
    case KEY_SEMICOLON: return ";";
    case KEY_EQUAL: return "=";
    case KEY_LEFT_BRACKET: return "[";
    case KEY_BACKSLASH: return "\\";
    case KEY_RIGHT_BRACKET: return "]";
    case KEY_GRAVE: return "`";
    case KEY_ESCAPE: return "Esc";
    case KEY_ENTER: return "Enter";
    case KEY_TAB: return "Tab";
    case KEY_BACKSPACE: return "Backspace";
    case KEY_INSERT: return "Insert";
    case KEY_DELETE: return "Delete";
    case KEY_RIGHT: return "\xE2\x86\x92";
    case KEY_LEFT: return "\xE2\x86\x90";
    case KEY_DOWN: return "\xE2\x86\x93";
    case KEY_UP: return "\xE2\x86\x91";
    case KEY_PAGE_UP: return "Page Up";
    case KEY_PAGE_DOWN: return "Page Down";
    case KEY_HOME: return "Home";
    case KEY_END: return "End";
    case KEY_CAPS_LOCK: return "Caps Lock";
    case KEY_SCROLL_LOCK: return "Scroll Lock";
    case KEY_NUM_LOCK: return "Num Lock";
    case KEY_PRINT_SCREEN: return "Print Screen";
    case KEY_PAUSE: return "Pause";
    case KEY_LEFT_SHIFT: return "Left Shift";
    case KEY_LEFT_CONTROL: return "Left Ctrl";
    case KEY_LEFT_ALT: return "Left Alt";
    case KEY_LEFT_SUPER: return "Left Win";
    case KEY_RIGHT_SHIFT: return "Right Shift";
    case KEY_RIGHT_CONTROL: return "Right Ctrl";
    case KEY_RIGHT_ALT: return "Right Alt";
    case KEY_RIGHT_SUPER: return "Right Win";
    case KEY_KB_MENU: return "Menu";
    case KEY_KP_DECIMAL: return "Num .";
    case KEY_KP_DIVIDE: return "Num /";
    case KEY_KP_MULTIPLY: return "Num *";
    case KEY_KP_SUBTRACT: return "Num -";
    case KEY_KP_ADD: return "Num +";
    case KEY_KP_ENTER: return "Num Enter";
    case KEY_KP_EQUAL: return "Num =";
    default: return nullptr;
    }
}

} // namespace

const Info& info(A a) { return INFO[std::clamp((int)a, 0, N - 1)]; }
Binding& slot(A a, int i) {
    ensure();
    return g_bind[std::clamp((int)a, 0, N - 1)][std::clamp(i, 0, SLOTS - 1)];
}

bool pressed(A a) {
    ensure();
    int ai = (int)a;
    if (ai < 0 || ai >= N) return false;
    for (int s = 0; s < SLOTS; s++) {
        const Binding& b = g_bind[ai][s];
        if (b.code == 0 || blockedNow(b.code)) continue;
        if (g_fake.code && g_fake == b) return true;
        bool hit = isMouse(b.code) ? IsMouseButtonPressed(b.code - MOUSE_BASE) : IsKeyPressed(b.code);
        if (hit && modsMatchPressed(b, INFO[ai].ctx)) return true;
    }
    return false;
}

bool down(A a) {
    ensure();
    int ai = (int)a;
    if (ai < 0 || ai >= N) return false;
    for (int s = 0; s < SLOTS; s++) {
        const Binding& b = g_bind[ai][s];
        if (b.code == 0 || blockedNow(b.code)) continue;
        bool hit = isMouse(b.code) ? IsMouseButtonDown(b.code - MOUSE_BASE) : IsKeyDown(b.code);
        if (hit && (heldMods() & b.mods) == b.mods) return true; // holds tolerate extra modifiers
    }
    return false;
}

float axis(A plus, A minus) { return (down(plus) ? 1.f : 0.f) - (down(minus) ? 1.f : 0.f); }

std::string name(const Binding& b) {
    if (b.code == 0) return tr("Unbound");
    std::string s;
    if (b.mods & Ctrl) s += keyText("Ctrl") + "+";
    if (b.mods & Shift) s += keyText("Shift") + "+";
    if (b.mods & Alt) s += "Alt+";
    if (isMouse(b.code)) {
        switch (b.code - MOUSE_BASE) {
        case MOUSE_BUTTON_MIDDLE: s += tr("Mouse 3 (Middle)"); break;
        case MOUSE_BUTTON_SIDE: s += tr("Mouse 4 (Back)"); break;
        case MOUSE_BUTTON_EXTRA: s += tr("Mouse 5 (Forward)"); break;
        default: s += trf("Mouse {0}", std::to_string(b.code - MOUSE_BASE + 1)); break;
        }
        return s;
    }
    int k = b.code;
    if ((k >= KEY_A && k <= KEY_Z) || (k >= KEY_ZERO && k <= KEY_NINE)) s += (char)k;
    else if (k >= KEY_F1 && k <= KEY_F12) s += "F" + std::to_string(k - KEY_F1 + 1);
    else if (k >= KEY_KP_0 && k <= KEY_KP_9) s += "Num " + std::to_string(k - KEY_KP_0);
    else if (const char* n = keyName(k)) s += keyText(n);
    else s += trf("Key {0}", std::to_string(k));
    return s;
}

std::string label(A a) {
    for (int s = 0; s < SLOTS; s++)
        if (slot(a, s).code) return name(slot(a, s));
    return "";
}

std::string tip(const char* en, A a) {
    std::string l = label(a);
    return l.empty() ? tr(en) : tr(en) + " (" + l + ")";
}

void resetDefaults() {
    g_init = true;
    for (int a = 0; a < N; a++)
        for (int s = 0; s < SLOTS; s++) g_bind[a][s] = INFO[a].def[s];
}

bool load(const std::string& path) {
    resetDefaults();
    std::filesystem::path fp = pathFromUtf8(path);
    if (fp.empty()) return false;
    std::ifstream f(fp, std::ios::binary | std::ios::ate);
    if (!f) return false;
    if (f.tellg() > 64 * 1024) return false; // not a bindings file: keep the defaults
    f.seekg(0);
    std::string line;
    int lines = 0;
    while (std::getline(f, line) && ++lines < 1000) {
        if (line.size() > 200 || line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string id;
        ss >> id;
        int a = -1;
        for (int i = 0; i < N; i++)
            if (id == INFO[i].id) a = i;
        if (a < 0) continue; // unknown or removed action
        Binding nb[SLOTS];
        bool ok = true;
        for (int s = 0; s < SLOTS && ok; s++) {
            int code = 0, mods = 0;
            if (!(ss >> code >> mods)) ok = false;
            else if (!validCode(code) || mods < 0 || mods > 7) ok = false;
            else nb[s] = Binding{code, (uint8_t)(code ? mods : 0)};
        }
        if (ok)
            for (int s = 0; s < SLOTS; s++) g_bind[a][s] = nb[s];
    }
    return true;
}

bool save(const std::string& path) {
    ensure();
    std::ostringstream f;
    f << "# Astraum key bindings: action  code mods  code mods  (mods: 1 Ctrl, 2 Shift, 4 Alt; code 1000+ = mouse button)\n";
    for (int a = 0; a < N; a++) {
        f << INFO[a].id;
        for (int s = 0; s < SLOTS; s++) f << ' ' << g_bind[a][s].code << ' ' << (int)g_bind[a][s].mods;
        f << '\n';
    }
    return writeFileAtomic(path, f.str());
}

A conflict(A self, const Binding& b) {
    ensure();
    if (b.code == 0) return A::COUNT;
    for (int a = 0; a < N; a++) {
        if (a == (int)self || !overlap(INFO[a].ctx, info(self).ctx)) continue;
        for (int s = 0; s < SLOTS; s++)
            if (g_bind[a][s] == b) return (A)a;
    }
    return A::COUNT;
}

void beginCapture() {
    g_capturing = true;
    g_pendingMod = 0;
    while (GetKeyPressed() != 0) {} // drop keys queued before the click
}
bool capturing() { return g_capturing; }
void cancelCapture() { g_capturing = false; }

bool pollCapture(Binding& out) {
    if (!g_capturing) return false;
    uint8_t held = heldMods();
    for (int k = GetKeyPressed(); k != 0; k = GetKeyPressed()) {
        if (k == KEY_ESCAPE) { g_capturing = false; return false; }
        if (isModifier(k)) {
            // a modifier alone becomes the binding when it is released without another key
            g_pendingMod = k;
            continue;
        }
        if (!validCode(k)) continue;
        out = Binding{k, held};
        g_capturing = false;
        return true;
    }
    for (int mb = MOUSE_BUTTON_MIDDLE; mb <= MOUSE_BUTTON_BACK; mb++)
        if (IsMouseButtonPressed(mb)) {
            out = Binding{MOUSE_BASE + mb, held};
            g_capturing = false;
            return true;
        }
    if (g_pendingMod && IsKeyReleased(g_pendingMod)) {
        out = Binding{g_pendingMod, 0};
        g_pendingMod = 0;
        g_capturing = false;
        return true;
    }
    return false;
}

void setSuspended(bool s) { g_suspended = s; }

void newFrame() {
    g_fake = g_fakeNext;
    g_fakeNext = {};
}
void inject(const Binding& b) { g_fakeNext = b; }

} // namespace keys
