// Minimal immediate-mode UI with a modern dark theme and vector icons (no emoji)
#pragma once
#include <string>
#include <functional>
#include <utility>
#include <vector>
#include "i18n.hpp"
#include "rl.hpp"

namespace ui {

enum class Icon {
    None, Rocket, Play, Pause, Forward, Backward, Map, Factory, Gear, Save, Folder, Trash, Up, Down, Plus, Minus,
    Hyper, Close, Check, Target, Planet, Star, Bolt, Info, Exit, Stage, Ship, Copy, Eye, Flame, Sail,
    NavOrbit, NavPan, NavZoom, NavFit, Home, Cube, Move, Rotate, Undo, Redo, Mirror, Wire, Xray, Book, Keyboard, Pencil, Trophy, Trajectory, EyeOff,
    Base // a surface base: a dome on the ground with a mast
};

namespace col {
inline const Color bg{11, 14, 20, 255};
inline const Color panel{18, 23, 32, 228};
inline const Color panel2{26, 32, 44, 240};
inline const Color line{48, 58, 76, 255};
inline const Color text{226, 232, 240, 255};
inline const Color dim{140, 152, 172, 255};
inline const Color faint{90, 102, 122, 255};
inline const Color accent{56, 189, 248, 255};
inline const Color accent2{129, 140, 248, 255};
inline const Color good{74, 222, 128, 255};
inline const Color warn{251, 191, 36, 255};
inline const Color bad{248, 113, 113, 255};
inline const Color hover{38, 47, 64, 255};
} // namespace col

void init(int lang);
// Per-frame UI scaling: call right after BeginDrawing(). Computes the scale from the
// monitor DPI (fitted so the layout's minimum logical size still fits the window).
void beginFrameScale();
// the player's GUI size (Settings): a factor on the DPI scale, 0.75 .. 1.5. A larger GUI is limited by the window
// (the layout keeps at least 1100 x 680 logical pixels; at 100 % and below 1280 x 740 as before)
void setUserScale(float k);
float userScale();
float scale();               // logical -> physical
float SW();                  // logical screen width
float SH();                  // logical screen height
void beginScissor(Rectangle r); // logical-rect scissor
void endScissor();
void shutdown();
void setLanguage(int lang);           // switches translation and reloads fonts
void setTextScale(float s);
float textScale();
void textNative(const std::string& s, Rectangle r, float size, Color c); // CJK-capable, untranslated
// word-wrapped (CJK aware) translated text; returns y after the last line
float wrap(const std::string& s, float x, float y, float width, float size, Color c, float maxY = 1e9f);
void beginFrame();
bool mouseOverUI();           // true if the mouse was over a UI element last frame
void setBlockAll(bool b); // an application-wide dialog: every widget ignores input until cleared
bool blockAll();
void ensureGlyphs(const std::string& s); // text outside the language's character set (names typed by the player)
bool keyboardCaptured();      // a text field has focus
bool editing();               // a text / number field holds uncommitted input
void cancelEdit();            // drop focus and any uncommitted input
// widgets drawn while blocked ignore the mouse (everything behind a modal dialog); reset every frame
void setInputBlocked(bool b);
void setClickHook(void (*fn)()); // called whenever a widget is clicked (UI sound)
bool inputBlocked();
void blockRect(Rectangle r);  // mark area as UI (consumes mouse)
// overlapping panels (the flight HUD): each is drawn as a layer; a click raises the layer under the mouse to the top,
// and widgets of a layer ignore the mouse where a higher layer covered it last frame. Layers are drawn in z-order by
// layered() (bottom first); panel() records the layer's area.
void layered(const std::vector<std::pair<int, std::function<void()>>>& layers);
void layerRaise(int id);   // to the top (a panel just opened)
bool layerOccluded();      // the mouse is over a higher layer than the one being drawn

void text(const std::string& s, float x, float y, float size, Color c, bool bold = false);
void textRaw(const std::string& s, float x, float y, float size, Color c, bool bold = false); // drawn as given, never translated
void textC(const std::string& s, Rectangle r, float size, Color c, bool bold = false); // centred in rect
void textR(const std::string& s, float xRight, float y, float size, Color c, bool bold = false);
Vector2 measure(const std::string& s, float size, bool bold = false);
std::string fit(const std::string& s, float maxW, float size, bool bold = false);

void panel(Rectangle r, Color c = col::panel, float round = 10);
void divider(float x, float y, float w);
bool button(Rectangle r, const std::string& label, bool active = false, bool enabled = true, Icon ic = Icon::None,
            Color accentCol = col::accent);
bool primaryButton(Rectangle r, const std::string& label, Icon ic = Icon::None, bool enabled = true, Color c = col::accent);
bool iconButton(Rectangle r, Icon ic, bool active = false, const char* tip = nullptr, bool enabled = true);
bool toggle(Rectangle r, const std::string& label, bool& v);
bool chip(Rectangle r, const std::string& label, bool active);
// horizontal slider over [lo, hi] in steps of `step`: drag (keeps tracking outside the bar until released),
// click, mouse wheel or the arrow keys while hovered. Returns true when the value changed; `committed` is set
// when a change is final (released, wheel, key) - save then, not on every frame of a drag
bool slider(Rectangle r, double& v, double lo, double hi, double step, int id, bool* committed = nullptr, bool enabled = true);
bool rowItem(Rectangle r, bool selected); // clickable list row background
void bar(Rectangle r, float frac, Color c, Color back = Color{36, 44, 60, 255});
bool textInput(Rectangle r, std::string& s, int maxLen, int id); // caret, selection, Ctrl+A / C / X / V, IME
void focusText(int id, const std::string& s); // give text field id the focus with all of s selected (a dialog opens)
void testTextInput(int focus, const std::vector<int>& chars, const std::vector<int>& keys, bool ctrl, bool shift); // (tests: input for the next field draw)
int testCaret();
// numeric entry: click to type a value, Enter / click elsewhere to apply, Esc to cancel; returns true when changed
// dragging across it sideways changes the value (left: down, right: up; Shift: finer): dragStep per pixel (0: from the
// value and the precision), snapped to multiples of snap (0: none)
bool numberField(Rectangle r, double& v, int prec, const char* unit, int id, double lo, double hi, double dragStep = 0, double snap = 0);
// an icon button that repeats while it is held down (after 0.4 s, faster the longer it is held)
bool repeatButton(Rectangle r, Icon ic, bool enabled = true);
// a value with - / + buttons (held: repeat) around a number field (type a value, or drag sideways)
bool spinField(Rectangle r, double& v, double step, double lo, double hi, int prec, const char* unit, int id);
void tooltip(const std::string& s);
void icon(Icon ic, Rectangle r, Color c);

// scroll areas
float beginScroll(int id, Rectangle r, float contentH);
void endScroll();
void setScroll(int id, float off); // (tests: clamped on the next draw)
bool inClip(Vector2 p);

// modal dimmer
void dim(float a = 0.55f);
void drawTooltip(); // call once at the end of the frame

} // namespace ui
