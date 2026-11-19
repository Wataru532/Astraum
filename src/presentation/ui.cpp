#include "ui.hpp"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include "i18n.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <vector>
#include "rl.hpp"
#include "render.hpp"

namespace ui {
namespace {
Font fR16, fR32, fB16, fB32, fNative;
bool fontsOk = false, nativeOk = false;
float g_scale = 1.0f;      // text-size setting (em size relative to the authored 14 px = 11 pt)
// raylib sizes fonts by line height (ascent - descent), not by em. These ratios convert an
// em-based size (what "pt" means) into raylib's size for the loaded fonts.
float g_lineRatio = 1.33f, g_nativeRatio = 1.10f;
float g_ui = 1.0f;         // logical -> physical
float g_fontPx = 0;        // physical scale the font atlases were built for
int g_lang = 0;
std::set<int> g_extraCps;     // characters typed by the player beyond the language's set
std::vector<int> g_haveCps;   // the characters in the current atlases (sorted)
bool g_glyphRebuild = false;
int atlasSmall = 22, atlasLarge = 44;
bool overNow = false, overPrev = false;
int focusId = -1;
int g_editId = -1;          // numberField being edited
bool g_focusSeen = false;   // the focused widget was drawn this frame
bool g_ewReq = false, g_ewShown = false; // the sideways-arrows cursor asked for this frame / shown
bool g_inputBlocked = false; // widgets behind an open modal dialog ignore the mouse
bool g_blockAll = false;     // an application-wide dialog (quit with unsaved changes) is open: every screen is blocked
std::map<int, float> scrolls;
std::vector<Rectangle> clipStack;
std::string tipText;
bool anyClickConsumed = false;

Font pick(float size, bool bold) {
    if (!fontsOk) return GetFontDefault();
    bool small = size * g_ui <= atlasSmall * 1.15f; // choose by physical pixel size
    if (bold) return small ? fB16 : fB32;
    return small ? fR16 : fR32;
}

} // namespace
// ---- layers (overlapping HUD panels)
std::vector<int> g_zOrder;                            // back = top
std::map<int, std::vector<Rectangle>> g_layerPrev, g_layerNow;
int g_curLayer = -1;
int zIndex(int id) {
    for (int i = 0; i < (int)g_zOrder.size(); i++)
        if (g_zOrder[i] == id) return i;
    return -1;
}
bool layerOccluded() {
    if (g_curLayer < 0) return false;
    int z = zIndex(g_curLayer);
    Vector2 m = GetMousePosition();
    for (int i = z + 1; i < (int)g_zOrder.size(); i++) {
        auto it = g_layerPrev.find(g_zOrder[i]);
        if (it == g_layerPrev.end()) continue;
        for (const Rectangle& r : it->second)
            if (CheckCollisionPointRec(m, r)) return true;
    }
    return false;
}
void layerRaise(int id) {
    int z = zIndex(id);
    if (z >= 0) g_zOrder.erase(g_zOrder.begin() + z);
    g_zOrder.push_back(id);
}
// at the start of a frame: last frame's areas become the ones that hide; a click raises the top layer under the mouse
void layerFrame() {
    g_layerPrev.swap(g_layerNow);
    g_layerNow.clear();
    g_curLayer = -1;
    if (g_inputBlocked) return;
    if (!IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) return;
    Vector2 m = GetMousePosition();
    for (int i = (int)g_zOrder.size() - 1; i >= 0; i--) {
        auto it = g_layerPrev.find(g_zOrder[i]);
        if (it == g_layerPrev.end()) continue;
        bool in = false;
        for (const Rectangle& r : it->second) in = in || CheckCollisionPointRec(m, r);
        if (in) {
            if (i != (int)g_zOrder.size() - 1) layerRaise(g_zOrder[i]);
            return;
        }
    }
}
void layered(const std::vector<std::pair<int, std::function<void()>>>& layers) {
    std::vector<int> fresh; // (new layers go under the known ones, in list order: the last listed highest)
    for (auto& l : layers)
        if (zIndex(l.first) < 0) fresh.push_back(l.first);
    g_zOrder.insert(g_zOrder.begin(), fresh.begin(), fresh.end());
    std::vector<int> order;
    for (int id : g_zOrder)
        for (auto& l : layers)
            if (l.first == id) order.push_back(id);
    for (int id : order)
        for (auto& l : layers)
            if (l.first == id) {
                g_curLayer = id;
                g_layerNow[id]; // (present even when it draws nothing this frame)
                l.second();
                g_curLayer = -1;
            }
}
namespace {

bool hovered(Rectangle r) {
    if (g_inputBlocked) return false;
    Vector2 m = GetMousePosition();
    return CheckCollisionPointRec(m, r) && inClip(m) && !layerOccluded();
}

// Extract the first face of a TrueType Collection (.ttc) into a standalone TTF image.
// raylib/stb_truetype only reads plain TTF/OTF files. All offsets are bounds-checked.
bool ttcToTtf(const unsigned char* d, size_t n, std::vector<unsigned char>& out) {
    auto u32 = [&](size_t o) -> uint32_t { return (uint32_t)d[o] << 24 | (uint32_t)d[o + 1] << 16 | (uint32_t)d[o + 2] << 8 | d[o + 3]; };
    auto u16 = [&](size_t o) -> uint32_t { return (uint32_t)d[o] << 8 | d[o + 1]; };
    if (n < 16 || std::memcmp(d, "ttcf", 4) != 0) return false;
    if (u32(8) == 0) return false;
    size_t off = u32(12);
    if (off + 12 > n) return false;
    size_t numTables = u16(off + 4);
    if (numTables == 0 || numTables > 64 || off + 12 + numTables * 16 > n) return false;
    size_t headerSize = 12 + numTables * 16;
    out.assign(d + off, d + off + headerSize);
    for (size_t i = 0; i < numTables; i++) {
        size_t rec = off + 12 + i * 16;
        size_t tOff = u32(rec + 8), tLen = u32(rec + 12);
        if (tOff > n || tLen > n - tOff) return false;
        while (out.size() % 4) out.push_back(0);
        uint32_t newOff = (uint32_t)out.size();
        out.insert(out.end(), d + tOff, d + tOff + tLen);
        unsigned char* w = out.data() + 12 + i * 16 + 8;
        w[0] = (unsigned char)(newOff >> 24); w[1] = (unsigned char)(newOff >> 16); w[2] = (unsigned char)(newOff >> 8); w[3] = (unsigned char)newOff;
    }
    return true;
}

// (hhea ascender - descender) / unitsPerEm of the first face in a TTF/OTF/TTC file; 0 on failure
float fontLineRatio(const std::string& path) {
    int n = 0;
    unsigned char* d = LoadFileData(path.c_str(), &n);
    if (!d) return 0;
    float ratio = 0;
    size_t N = (size_t)n;
    auto u16 = [&](size_t o) -> uint32_t { return o + 2 <= N ? (uint32_t)d[o] << 8 | d[o + 1] : 0; };
    auto u32 = [&](size_t o) -> uint32_t { return o + 4 <= N ? (uint32_t)d[o] << 24 | (uint32_t)d[o + 1] << 16 | (uint32_t)d[o + 2] << 8 | d[o + 3] : 0; };
    size_t off = (N >= 16 && std::memcmp(d, "ttcf", 4) == 0) ? u32(12) : 0;
    size_t numTables = u16(off + 4);
    size_t head = 0, hhea = 0;
    for (size_t i = 0; i < numTables && i < 64; i++) {
        size_t rec = off + 12 + i * 16;
        if (rec + 16 > N) break;
        if (std::memcmp(d + rec, "head", 4) == 0) head = u32(rec + 8);
        if (std::memcmp(d + rec, "hhea", 4) == 0) hhea = u32(rec + 8);
    }
    if (head && hhea && head + 20 <= N && hhea + 8 <= N) {
        float upem = (float)u16(head + 18);
        float asc = (float)(int16_t)u16(hhea + 4), desc = (float)(int16_t)u16(hhea + 6);
        if (upem > 0 && asc - desc > 0) ratio = (asc - desc) / upem;
    }
    UnloadFileData(d);
    return (ratio > 0.8f && ratio < 2.0f) ? ratio : 0;
}

Font loadFont(const std::string& path, int size, std::vector<int> cps) {
    Font f{};
    if (IsFileExtension(path.c_str(), ".ttc")) {
        int n = 0;
        unsigned char* data = LoadFileData(path.c_str(), &n);
        std::vector<unsigned char> ttf;
        if (data && n > 0 && ttcToTtf(data, (size_t)n, ttf))
            f = LoadFontFromMemory(".ttf", ttf.data(), (int)ttf.size(), size, cps.data(), (int)cps.size());
        if (data) UnloadFileData(data);
    } else {
        f = LoadFontEx(path.c_str(), size, cps.data(), (int)cps.size());
    }
    if (f.texture.id != 0) {
        GenTextureMipmaps(&f.texture);
        SetTextureFilter(f.texture, TEXTURE_FILTER_TRILINEAR);
    }
    return f;
}

// the glyphs of one font file (TTC: its first face) without an atlas; nullptr on failure
GlyphInfo* fontGlyphs(const std::string& path, int size, std::vector<int>& cps) {
    int n = 0;
    unsigned char* data = LoadFileData(path.c_str(), &n);
    if (!data || n <= 0) return nullptr;
    GlyphInfo* g = nullptr;
    if (IsFileExtension(path.c_str(), ".ttc")) {
        std::vector<unsigned char> ttf;
        if (ttcToTtf(data, (size_t)n, ttf)) g = LoadFontData(ttf.data(), (int)ttf.size(), size, cps.data(), (int)cps.size(), FONT_DEFAULT);
    } else g = LoadFontData(data, n, size, cps.data(), (int)cps.size(), FONT_DEFAULT);
    UnloadFileData(data);
    return g;
}

// a Latin font with the CJK characters taken from a Japanese font (player-typed names shown in English or
// German): one atlas, as raylib draws a string with a single font
Font loadMerged(const std::string& path, const std::string& cjkPath, int size, std::vector<int> cps, std::vector<int> cjk) {
    Font f{};
    GlyphInfo* a = fontGlyphs(path, size, cps);
    GlyphInfo* b = a ? fontGlyphs(cjkPath, size, cjk) : nullptr;
    if (!a || !b) {
        if (a) UnloadFontData(a, (int)cps.size());
        if (b) UnloadFontData(b, (int)cjk.size());
        return loadFont(path, size, cps);
    }
    int count = (int)(cps.size() + cjk.size());
    GlyphInfo* all = (GlyphInfo*)MemAlloc(count * sizeof(GlyphInfo));
    std::memcpy(all, a, cps.size() * sizeof(GlyphInfo));
    std::memcpy(all + cps.size(), b, cjk.size() * sizeof(GlyphInfo));
    MemFree(a); // (the glyph images now belong to `all`)
    MemFree(b);
    f.baseSize = size;
    f.glyphCount = count;
    f.glyphPadding = 4;
    f.glyphs = all;
    Image atlas = GenImageFontAtlas(f.glyphs, &f.recs, count, size, 4, 0);
    f.texture = LoadTextureFromImage(atlas);
    UnloadImage(atlas);
    if (f.texture.id != 0) {
        GenTextureMipmaps(&f.texture);
        SetTextureFilter(f.texture, TEXTURE_FILTER_TRILINEAR);
    }
    return f;
}

std::string fontDir() {
    const char* wd = std::getenv("WINDIR");
    return std::string(wd ? wd : "C:\\Windows") + "\\Fonts\\";
}

std::string firstExisting(std::initializer_list<const char*> names) {
    std::string dir = fontDir();
    for (const char* n : names)
        if (FileExists((dir + n).c_str())) return dir + n;
    return "";
}

void unloadFonts() {
    if (fontsOk) {
        UnloadFont(fR16);
        UnloadFont(fR32);
        UnloadFont(fB16);
        UnloadFont(fB32);
    }
    fontsOk = false;
}

// UTF-8 decode helper: returns codepoint and advances i
int nextCp(const std::string& s, size_t& i) {
    int bytes = 0;
    int cp = GetCodepointNext(s.c_str() + i, &bytes);
    i += bytes > 0 ? bytes : 1;
    return cp;
}
} // namespace

void setLanguage(int lang) {
    i18n::setLang(lang);
    rlDrawRenderBatchActive(); // pending text still references the old font textures
    unloadFonts();
    std::string reg, bold;
    if (lang == i18n::JA) {
        reg = firstExisting({"YuGothM.ttc", "meiryo.ttc", "BIZ-UDGothicR.ttc", "msgothic.ttc"});
        bold = firstExisting({"YuGothB.ttc", "meiryob.ttc", "BIZ-UDGothicB.ttc", "msgothic.ttc"});
    } else {
        reg = firstExisting({"segoeui.ttf", "arial.ttf"});
        bold = firstExisting({"segoeuisb.ttf", "segoeuib.ttf", "arialbd.ttf"});
    }
    if (reg.empty() || bold.empty()) return;
    auto cps = i18n::codepoints(lang);
    // characters the player typed (map names): added on demand, the atlas is rebuilt with them
    for (int c : g_extraCps)
        if (!std::binary_search(cps.begin(), cps.end(), c)) cps.push_back(c);
    std::sort(cps.begin(), cps.end());
    g_haveCps.assign(cps.begin(), cps.end());
    float lr = fontLineRatio(reg);
    g_lineRatio = lr > 0 ? lr : 1.25f;
    // atlases sized for the physical pixel size of body text and headings
    float k = g_ui * g_scale * g_lineRatio;
    atlasSmall = std::clamp((int)std::lround(16 * k), 14, 64);
    atlasLarge = std::clamp((int)std::lround(34 * k), 28, 112);
    g_fontPx = k;
    g_lang = lang;
    // English / German with Japanese characters typed by the player: those come from a Japanese font
    std::vector<int> cjk;
    std::string cjkR, cjkB;
    if (lang != i18n::JA) {
        std::vector<int> latin;
        for (int c : cps) (c >= 0x2E80 && g_extraCps.count(c) ? cjk : latin).push_back(c); // (only typed ones: no cost otherwise)
        cjkR = firstExisting({"YuGothM.ttc", "meiryo.ttc", "BIZ-UDGothicR.ttc", "msgothic.ttc"});
        cjkB = firstExisting({"YuGothB.ttc", "meiryob.ttc", "BIZ-UDGothicB.ttc", "msgothic.ttc"});
        if (!cjk.empty() && !cjkR.empty() && !cjkB.empty()) cps = latin;
        else cjk.clear();
    }
    auto load = [&](const std::string& path, const std::string& cjkPath, int size) {
        return cjk.empty() ? loadFont(path, size, cps) : loadMerged(path, cjkPath, size, cps, cjk);
    };
    fR16 = load(reg, cjkR, atlasSmall);
    fR32 = load(reg, cjkR, atlasLarge);
    fB16 = load(bold, cjkB, atlasSmall);
    fB32 = load(bold, cjkB, atlasLarge);
    fontsOk = fR16.texture.id && fR32.texture.id && fB16.texture.id && fB32.texture.id;
}

void init(int lang) {
    // small CJK-capable font so the language picker can always show native names
    std::string cjk = firstExisting({"YuGothM.ttc", "meiryo.ttc", "BIZ-UDGothicR.ttc", "msgothic.ttc"});
    if (!cjk.empty()) {
        int n = 0;
        int* raw = LoadCodepoints("日本語DeutschEnglish ", &n);
        std::vector<int> cps(raw, raw + n);
        UnloadCodepoints(raw);
        fNative = loadFont(cjk, 32, cps);
        float nr = fontLineRatio(cjk);
        if (nr > 0) g_nativeRatio = nr;
        nativeOk = fNative.texture.id != 0;
    }
    setLanguage(lang);
}

void shutdown() {
    unloadFonts();
    if (nativeOk) UnloadFont(fNative);
}

void setTextScale(float s) {
    g_scale = std::clamp(s, 0.7f, 1.6f);
    if (g_fontPx > 0 && std::fabs(g_ui * g_scale * g_lineRatio - g_fontPx) > 0.01f) setLanguage(g_lang); // rebuild atlases
}

float g_userScale = 1.0f;
void setUserScale(float k) { g_userScale = std::clamp(k, 0.5f, 2.0f); }
float userScale() { return g_userScale; }

void beginFrameScale() {
    float dpi = GetWindowScaleDPI().x;
    if (!(dpi > 0.5f)) dpi = 1.0f;
    float w = (float)GetScreenWidth(), h = (float)GetScreenHeight();
    // the layout needs at least ~1280 x 740 logical pixels (a GUI enlarged by the player: 1100 x 680, the
    // window's minimum size)
    float fit = g_userScale > 1.0f ? std::min(w / 1100.0f, h / 680.0f) : std::min(w / 1280.0f, h / 740.0f);
    float s = std::min(dpi * g_userScale, fit);
    s = std::clamp(s, 0.5f, 4.0f);
    if (std::fabs(s - g_ui) > 0.001f) {
        g_ui = s;
        if (g_fontPx > 0 && std::fabs(g_ui * g_scale * g_lineRatio - g_fontPx) / g_fontPx > 0.08f) setLanguage(g_lang);
    }
    rlScalef(g_ui, g_ui, 1.0f);
    SetMouseScale(1.0f / g_ui, 1.0f / g_ui);
    gfx::setUiScale(g_ui);
}
float scale() { return g_ui; }
float SW() { return GetScreenWidth() / g_ui; }
float SH() { return GetScreenHeight() / g_ui; }
void beginScissor(Rectangle r) {
    rlDrawRenderBatchActive();
    BeginScissorMode((int)std::floor(r.x * g_ui), (int)std::floor(r.y * g_ui), (int)std::ceil(r.width * g_ui), (int)std::ceil(r.height * g_ui));
}
void endScissor() {
    rlDrawRenderBatchActive();
    EndScissorMode();
}
float textScale() { return g_scale; }

void textNative(const std::string& s, Rectangle r, float size, Color c) {
    if (!nativeOk) { textC(s, r, size, c); return; }
    size *= g_scale * g_nativeRatio;
    Vector2 m = MeasureTextEx(fNative, s.c_str(), size, 0);
    DrawTextEx(fNative, s.c_str(), {std::round(r.x + (r.width - m.x) / 2), std::round(r.y + (r.height - m.y) / 2)}, size, 0, c);
}

float wrap(const std::string& src, float x, float y, float width, float size, Color c, float maxY) {
    std::string s = tr(src);
    // tokens: words separated by spaces, CJK characters are individual tokens
    std::vector<std::string> toks;
    std::string cur;
    for (size_t i = 0; i < s.size();) {
        size_t start = i;
        int cp = nextCp(s, i);
        std::string ch = s.substr(start, i - start);
        if (cp == ' ') {
            if (!cur.empty()) toks.push_back(cur);
            toks.push_back(" ");
            cur.clear();
        } else if (cp >= 0x2E80) {
            if (!cur.empty()) toks.push_back(cur);
            cur.clear();
            toks.push_back(ch);
        } else cur += ch;
    }
    if (!cur.empty()) toks.push_back(cur);
    float rsz = size * g_scale * g_lineRatio;
    float lineH = rsz * 1.12f;
    std::string line;
    auto flush = [&]() {
        while (!line.empty() && line.back() == ' ') line.pop_back();
        if (!line.empty() && y + lineH <= maxY + 1) DrawTextEx(pick(rsz, false), line.c_str(), {std::round(x), std::round(y)}, rsz, 0, c);
        y += lineH;
        line.clear();
    };
    for (auto& t : toks) {
        if (line.empty() && t == " ") continue;
        std::string trial = line + t;
        if (MeasureTextEx(pick(rsz, false), trial.c_str(), rsz, 0).x > width && !line.empty()) {
            flush();
            if (t == " ") continue;
            line = t;
        } else line = trial;
    }
    if (!line.empty()) flush();
    return y;
}

void ensureGlyphs(const std::string& s) {
    for (size_t i = 0; i < s.size();) {
        int cp = nextCp(s, i);
        if (cp < 32 || cp > 0x2FFFF) continue;
        if (std::binary_search(g_haveCps.begin(), g_haveCps.end(), cp)) continue;
        if (g_extraCps.size() < 6000 && g_extraCps.insert(cp).second) g_glyphRebuild = true;
    }
}

void beginFrame() {
    if (g_glyphRebuild) {
        g_glyphRebuild = false;
        setLanguage(g_lang); // (new characters in the atlas)
    }
    // a focused text / number field that was not drawn last frame has gone away: drop its focus
    if (focusId >= 0 && !g_focusSeen) {
        focusId = -1;
        g_editId = -1;
    }
    g_focusSeen = false;
    if (g_ewReq != g_ewShown) {
        SetMouseCursor(g_ewReq ? MOUSE_CURSOR_RESIZE_EW : MOUSE_CURSOR_DEFAULT);
        g_ewShown = g_ewReq;
    }
    g_ewReq = false;
    g_inputBlocked = g_blockAll;
    layerFrame();
    overPrev = overNow;
    overNow = false;
    tipText.clear();
    anyClickConsumed = false;
}

bool mouseOverUI() { return overPrev || overNow; }
bool keyboardCaptured() { return focusId >= 0; }
bool editing() { return focusId >= 0 || g_editId >= 0; }
void setInputBlocked(bool b) { g_inputBlocked = b || g_blockAll; }
void setBlockAll(bool b) {
    g_blockAll = b;
    g_inputBlocked = b;
}
bool blockAll() { return g_blockAll; }
bool inputBlocked() { return g_inputBlocked; }
void cancelEdit() {
    focusId = -1;
    g_editId = -1;
}
void blockRect(Rectangle r) {
    if (CheckCollisionPointRec(GetMousePosition(), r)) overNow = true;
}

void text(const std::string& s, float x, float y, float size, Color c, bool bold) {
    textRaw(tr(s), x, y, size, c, bold);
}
void textRaw(const std::string& s, float x, float y, float size, Color c, bool bold) {
    size *= g_scale * g_lineRatio;
    Font f = pick(size, bold);
    DrawTextEx(f, s.c_str(), {std::round(x), std::round(y)}, size, 0.0f, c);
}
Vector2 measure(const std::string& s, float size, bool bold) {
    std::string t = tr(s);
    size *= g_scale * g_lineRatio;
    return MeasureTextEx(pick(size, bold), t.c_str(), size, 0.0f);
}
void textC(const std::string& s, Rectangle r, float size, Color c, bool bold) {
    Vector2 m = measure(s, size, bold);
    text(s, r.x + (r.width - m.x) * 0.5f, r.y + (r.height - m.y) * 0.5f, size, c, bold);
}
void textR(const std::string& s, float xr, float y, float size, Color c, bool bold) {
    Vector2 m = measure(s, size, bold);
    text(s, xr - m.x, y, size, c, bold);
}
std::string fit(const std::string& s, float maxW, float size, bool bold) {
    std::string t = tr(s);
    if (measure(t, size, bold).x <= maxW) return t;
    // trim whole UTF-8 codepoints
    std::vector<size_t> starts;
    for (size_t i = 0; i < t.size();) {
        starts.push_back(i);
        nextCp(t, i);
    }
    while (!starts.empty()) {
        std::string o = t.substr(0, starts.back()) + "...";
        starts.pop_back();
        if (measure(o, size, bold).x <= maxW) return o;
    }
    return "...";
}

void panel(Rectangle r, Color c, float round) {
    blockRect(r);
    if (g_curLayer >= 0) g_layerNow[g_curLayer].push_back(r);
    float rr = std::min(1.0f, round * 2 / std::max(1.0f, std::min(r.width, r.height)));
    DrawRectangleRounded(r, rr, 8, c);
    DrawRectangleRoundedLinesEx(r, rr, 8, 1.0f, Color{60, 72, 94, 120});
}

void divider(float x, float y, float w) { DrawRectangle((int)x, (int)y, (int)w, 1, col::line); }

static void (*g_clickHook)() = nullptr;
void setClickHook(void (*fn)()) { g_clickHook = fn; }

static bool clicked(Rectangle r) {
    if (!hovered(r)) return false;
    overNow = true;
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !anyClickConsumed) {
        anyClickConsumed = true;
        if (g_clickHook) g_clickHook();
        return true;
    }
    return false;
}

bool button(Rectangle r, const std::string& label0, bool active, bool enabled, Icon ic, Color ac) {
    bool h = hovered(r) && enabled;
    Color bgc = active ? Color{(unsigned char)(ac.r * 0.25f), (unsigned char)(ac.g * 0.25f), (unsigned char)(ac.b * 0.25f), 255}
                       : (h ? col::hover : col::panel2);
    float rr = std::min(1.0f, 12.0f / std::max(1.0f, std::min(r.width, r.height)));
    DrawRectangleRounded(r, rr, 6, bgc);
    DrawRectangleRoundedLinesEx(r, rr, 6, 1.0f, active ? ac : (h ? Color{80, 96, 124, 255} : col::line));
    Color tc = enabled ? (active ? ac : col::text) : col::faint;
    float fs = std::min(16.0f, r.height * 0.5f);
    float isz = ic != Icon::None ? std::min(r.height * 0.55f, 18.0f) + 8 : 0;
    std::string label = fit(label0, r.width - 14 - isz, fs);
    if (ic != Icon::None) {
        float is = std::min(r.height * 0.55f, 18.0f);
        if (label.empty()) {
            icon(ic, {r.x + (r.width - is) / 2, r.y + (r.height - is) / 2, is, is}, tc);
        } else {
            Vector2 m = measure(label, fs);
            float total = is + 8 + m.x;
            float x = r.x + (r.width - total) / 2;
            icon(ic, {x, r.y + (r.height - is) / 2, is, is}, tc);
            text(label, x + is + 8, r.y + (r.height - m.y) / 2, fs, tc);
        }
    } else textC(label, r, fs, tc);
    if (!enabled) { if (hovered(r)) overNow = true; return false; }
    return clicked(r);
}

bool primaryButton(Rectangle r, const std::string& label0, Icon ic, bool enabled, Color c) {
    bool h = hovered(r) && enabled;
    Color bgc = enabled ? (h ? Color{(unsigned char)std::min(255, c.r + 25), (unsigned char)std::min(255, c.g + 25), (unsigned char)std::min(255, c.b + 25), 255} : c)
                        : Color{50, 58, 72, 255};
    float rr = std::min(1.0f, 12.0f / std::max(1.0f, std::min(r.width, r.height)));
    DrawRectangleRounded(r, rr, 6, bgc);
    Color tc = enabled ? Color{8, 14, 22, 255} : col::faint;
    float fs = std::min(17.0f, r.height * 0.5f);
    std::string label = fit(label0, r.width - 14 - (ic != Icon::None ? 26.f : 0.f), fs, true);
    if (ic != Icon::None) {
        float is = std::min(r.height * 0.5f, 18.0f);
        Vector2 m = measure(label, fs, true);
        float total = is + 8 + m.x;
        float x = r.x + (r.width - total) / 2;
        icon(ic, {x, r.y + (r.height - is) / 2, is, is}, tc);
        text(label, x + is + 8, r.y + (r.height - m.y) / 2, fs, tc, true);
    } else textC(label, r, fs, tc, true);
    if (!enabled) return false;
    return clicked(r);
}

bool iconButton(Rectangle r, Icon ic, bool active, const char* tip, bool enabled) {
    bool h = hovered(r) && enabled;
    float rr = std::min(1.0f, 12.0f / std::max(1.0f, std::min(r.width, r.height)));
    DrawRectangleRounded(r, rr, 6, active ? Color{14, 60, 82, 255} : (h ? col::hover : col::panel2));
    DrawRectangleRoundedLinesEx(r, rr, 6, 1.0f, active ? col::accent : col::line);
    float is = std::min(r.width, r.height) * 0.56f;
    icon(ic, {r.x + (r.width - is) / 2, r.y + (r.height - is) / 2, is, is}, enabled ? (active ? col::accent : col::text) : col::faint);
    if (h && tip) tooltip(tip);
    if (!enabled) return false;
    return clicked(r);
}

bool toggle(Rectangle r, const std::string& label, bool& v) {
    bool h = hovered(r);
    Rectangle sw{r.x + r.width - 44, r.y + (r.height - 22) / 2, 44, 22};
    if (!label.empty()) text(fit(label, std::max(r.width - 56, 0.0f), 16, false), r.x, r.y + (r.height - 18) / 2, 16, h ? col::text : Color{200, 208, 222, 255}); // (never under the switch)
    DrawRectangleRounded(sw, 1.0f, 12, v ? col::accent : Color{52, 62, 80, 255});
    float kx = v ? sw.x + sw.width - 11 : sw.x + 11;
    DrawCircle((int)kx, (int)(sw.y + 11), 8, WHITE);
    if (clicked(r)) {
        v = !v;
        return true;
    }
    return false;
}

bool chip(Rectangle r, const std::string& label, bool active) {
    bool h = hovered(r);
    DrawRectangleRounded(r, 1.0f, 10, active ? Color{14, 60, 82, 255} : (h ? col::hover : Color{24, 30, 42, 255}));
    if (active) DrawRectangleRoundedLinesEx(r, 1.0f, 10, 1.0f, col::accent);
    textC(label, r, 14, active ? col::accent : col::dim, active);
    return clicked(r);
}

bool slider(Rectangle r, double& v, double lo, double hi, double step, int id, bool* committed, bool enabled) {
    static int dragId = -1;
    if (committed) *committed = false;
    if (hi <= lo) return false;
    auto snap = [&](double x) {
        x = std::clamp(x, lo, hi);
        return step > 0 ? std::clamp(lo + std::round((x - lo) / step) * step, lo, hi) : x;
    };
    double old = v;
    bool h = enabled && hovered(r);
    if (h) overNow = true;
    float mx = GetMousePosition().x;
    if (enabled && h && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !anyClickConsumed) {
        anyClickConsumed = true;
        dragId = id;
    }
    bool dragging = dragId == id;
    if (dragging) {
        overNow = true;
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && enabled && !g_inputBlocked)
            v = snap(lo + (mx - r.x) / std::max(r.width, 1.0f) * (hi - lo));
        else {
            dragId = -1;
            dragging = false;
            if (committed) *committed = true;
        }
    } else if (h) {
        // fine steps: the wheel and the arrow keys
        float wh = GetMouseWheelMove();
        int dir = wh > 0 ? 1 : wh < 0 ? -1 : 0;
        if (IsKeyPressed(KEY_RIGHT) || IsKeyPressedRepeat(KEY_RIGHT) || IsKeyPressed(KEY_UP) || IsKeyPressedRepeat(KEY_UP)) dir = 1;
        if (IsKeyPressed(KEY_LEFT) || IsKeyPressedRepeat(KEY_LEFT) || IsKeyPressed(KEY_DOWN) || IsKeyPressedRepeat(KEY_DOWN)) dir = -1;
        if (dir != 0) {
            v = snap(v + dir * (step > 0 ? step : (hi - lo) / 100));
            if (committed) *committed = v != old;
        }
    }
    // the bar, the filled part and the knob
    float f = (float)std::clamp((v - lo) / (hi - lo), 0.0, 1.0);
    float cy = r.y + r.height / 2;
    Rectangle track{r.x, cy - 3, r.width, 6};
    Color fill = enabled ? col::accent : Color{70, 82, 104, 255};
    DrawRectangleRounded(track, 1.0f, 6, Color{40, 48, 64, 255});
    if (f > 0.001f) DrawRectangleRounded({track.x, track.y, std::max(6.0f, track.width * f), track.height}, 1.0f, 6, fill);
    float kx = r.x + r.width * f;
    float kr = dragging ? 9.5f : (h ? 9.0f : 8.0f);
    if (dragging || h) DrawCircleV({kx, cy}, kr + 5, Color{fill.r, fill.g, fill.b, 48});
    DrawCircleV({kx, cy}, kr, enabled ? WHITE : Color{150, 158, 172, 255});
    return v != old;
}

bool rowItem(Rectangle r, bool selected) {
    bool h = hovered(r);
    if (selected) {
        DrawRectangleRounded(r, 0.25f, 6, Color{14, 52, 72, 255});
        DrawRectangle((int)r.x, (int)r.y + 4, 3, (int)r.height - 8, col::accent);
    } else if (h) DrawRectangleRounded(r, 0.25f, 6, col::hover);
    return clicked(r);
}

void bar(Rectangle r, float frac, Color c, Color back) {
    frac = std::clamp(frac, 0.0f, 1.0f);
    DrawRectangleRounded(r, 1.0f, 6, back);
    if (frac > 0.001f) DrawRectangleRounded({r.x, r.y, std::max(r.height, r.width * frac), r.height}, 1.0f, 6, c);
}

// ---- text fields: caret, selection (drag, double-click = word, triple-click / Ctrl+A = all, Shift + arrows),
// clipboard (Ctrl+C / X / V), UTF-8 throughout (the IME delivers Japanese). One field has the focus at a
// time, so its editing state lives here.
namespace {
int g_caret = 0, g_anchor = 0, g_tClickId = -1, g_tClicks = 0;
std::vector<int> g_tiChars, g_tiKeys; // (tests: typed characters and keys for the focused field, this frame)
bool g_tiCtrl = false, g_tiShift = false;
bool g_tDrag = false;
float g_tScroll = 0;
double g_tClickT = -10;
float tSize() { return 16 * g_scale * g_lineRatio; }
// width of the first n bytes (the text as it is: no translation of what the player typed)
float rawW(const std::string& s, size_t n) {
    if (n == 0) return 0;
    std::string t = s.substr(0, n);
    return MeasureTextEx(pick(tSize(), false), t.c_str(), tSize(), 0).x;
}
size_t prevCpI(const std::string& s, size_t i) {
    if (i == 0) return 0;
    i--;
    while (i > 0 && ((unsigned char)s[i] & 0xC0) == 0x80) i--;
    return i;
}
size_t nextCpI(const std::string& s, size_t i) {
    if (i >= s.size()) return s.size();
    i++;
    while (i < s.size() && ((unsigned char)s[i] & 0xC0) == 0x80) i++;
    return i;
}
bool isSpace(const std::string& s, size_t i) { return i < s.size() && (s[i] == ' ' || s[i] == '\t'); }
} // namespace

void testTextInput(int focus, const std::vector<int>& chars, const std::vector<int>& keys, bool ctrl, bool shift) {
    if (focus >= 0 && focusId != focus) {
        focusId = focus;
        g_caret = g_anchor = 0;
        g_tScroll = 0;
    }
    g_focusSeen = true; // (kept through the next frame start)
    g_tiChars = chars;
    g_tiKeys = keys;
    g_tiCtrl = ctrl;
    g_tiShift = shift;
}
int testCaret() { return g_caret; }
void focusText(int id, const std::string& s) {
    focusId = id;
    g_anchor = 0;
    g_caret = (int)s.size();
    g_tScroll = 0;
    g_focusSeen = true; // (kept through the next frame start)
}

bool textInput(Rectangle r, std::string& s, int maxLen, int id) {
    bool h = hovered(r);
    if (h) overNow = true;
    const float pad = 10, innerW = r.width - 2 * pad;
    bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL) || g_tiCtrl;
    bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT) || g_tiShift;
    auto IsKeyPressed = [](int k) { return ::IsKeyPressed(k) || std::find(g_tiKeys.begin(), g_tiKeys.end(), k) != g_tiKeys.end(); };
    // the byte index (a character boundary) nearest to the mouse
    auto idxAt = [&](float mx) {
        float x = mx - (r.x + pad) + g_tScroll;
        size_t best = 0;
        float bd = 1e9f;
        for (size_t i = 0;; i = nextCpI(s, i)) {
            float d = std::fabs(rawW(s, i) - x);
            if (d < bd) { bd = d; best = i; }
            if (i >= s.size()) break;
        }
        return (int)best;
    };
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        if (h) {
            bool fresh = focusId != id;
            if (fresh) {
                focusId = id;
                g_tScroll = 0;
            }
            int at = idxAt((float)GetMouseX());
            double now = GetTime();
            g_tClicks = (g_tClickId == id && now - g_tClickT < 0.45) ? g_tClicks + 1 : 1;
            g_tClickT = now;
            g_tClickId = id;
            g_tDrag = false;
            if (g_tClicks == 2) {
                // double click: the word under the mouse
                size_t lo = (size_t)at, hi = (size_t)at;
                while (lo > 0 && !isSpace(s, prevCpI(s, lo))) lo = prevCpI(s, lo);
                while (hi < s.size() && !isSpace(s, hi)) hi = nextCpI(s, hi);
                if (lo == hi) { lo = 0; hi = s.size(); }
                g_anchor = (int)lo;
                g_caret = (int)hi;
            } else if (g_tClicks >= 3) {
                g_anchor = 0;
                g_caret = (int)s.size();
            } else {
                g_caret = at;
                if (!shift || fresh) g_anchor = at;
                g_tDrag = true;
            }
        } else if (focusId == id) focusId = -1;
    }
    bool focused = focusId == id;
    if (focused) g_focusSeen = true;
    DrawRectangleRounded(r, 0.3f, 6, focused ? Color{20, 28, 40, 255} : col::panel2);
    DrawRectangleRoundedLinesEx(r, 0.3f, 6, 1.0f, focused ? col::accent : col::line);
    bool changed = false;
    if (focused) {
        g_caret = std::clamp(g_caret, 0, (int)s.size());
        g_anchor = std::clamp(g_anchor, 0, (int)s.size());
        if (g_tDrag) {
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) g_caret = idxAt((float)GetMouseX()); // (drag: select)
            else g_tDrag = false;
        }
        auto lo = [&] { return std::min(g_caret, g_anchor); };
        auto hi = [&] { return std::max(g_caret, g_anchor); };
        auto delSel = [&] {
            if (g_caret == g_anchor) return false;
            int a = lo(), n = hi() - a;
            s.erase((size_t)a, (size_t)n);
            g_caret = g_anchor = a;
            changed = true;
            return true;
        };
        auto insertText = [&](std::string t) {
            delSel();
            // only whole characters that fit
            size_t room = maxLen > (int)s.size() ? (size_t)(maxLen - (int)s.size()) : 0, cut = 0;
            for (size_t i = 0; i < t.size(); i = nextCpI(t, i))
                if (nextCpI(t, i) <= room) cut = nextCpI(t, i);
            t.resize(cut);
            if (t.empty()) return;
            s.insert((size_t)g_caret, t);
            g_caret += (int)t.size();
            g_anchor = g_caret;
            changed = true;
        };
        if (ctrl && IsKeyPressed(KEY_A)) { // select all
            g_anchor = 0;
            g_caret = (int)s.size();
        }
        if (ctrl && (IsKeyPressed(KEY_C) || IsKeyPressed(KEY_X)) && g_caret != g_anchor) {
            SetClipboardText(s.substr((size_t)lo(), (size_t)(hi() - lo())).c_str());
            if (IsKeyPressed(KEY_X)) delSel();
        }
        if (ctrl && IsKeyPressed(KEY_V)) {
            const char* c = GetClipboardText();
            if (c) {
                std::string t;
                for (const char* q = c; *q && t.size() < 400; q++) t.push_back((unsigned char)*q < 32 ? ' ' : *q); // (one line)
                insertText(t);
            }
        }
        int ch;
        size_t tiNext = 0;
        while ((ch = GetCharPressed()) > 0 || (tiNext < g_tiChars.size() && (ch = g_tiChars[tiNext++]) > 0)) {
            // any printable character (also Japanese from the IME), UTF-8, maxLen in bytes
            if (ch >= 32 && ch != 127 && ch <= 0x10FFFF && !(ch >= 0xD800 && ch <= 0xDFFF)) {
                int n = 0;
                const char* u = CodepointToUTF8(ch, &n);
                if (n > 0) insertText(std::string(u, (size_t)n));
            }
        }
        auto rep = [&](int k) { return IsKeyPressed(k) || IsKeyPressedRepeat(k); };
        if (rep(KEY_BACKSPACE) && !delSel() && g_caret > 0) {
            int p0 = (int)prevCpI(s, (size_t)g_caret);
            s.erase((size_t)p0, (size_t)(g_caret - p0));
            g_caret = g_anchor = p0;
            changed = true;
        }
        if (rep(KEY_DELETE) && !delSel() && g_caret < (int)s.size()) {
            int n = (int)nextCpI(s, (size_t)g_caret);
            s.erase((size_t)g_caret, (size_t)(n - g_caret));
            g_anchor = g_caret;
            changed = true;
        }
        if (rep(KEY_LEFT)) {
            if (!shift && g_caret != g_anchor) g_caret = lo();
            else g_caret = (int)prevCpI(s, (size_t)g_caret);
            if (!shift) g_anchor = g_caret;
        }
        if (rep(KEY_RIGHT)) {
            if (!shift && g_caret != g_anchor) g_caret = hi();
            else g_caret = (int)nextCpI(s, (size_t)g_caret);
            if (!shift) g_anchor = g_caret;
        }
        if (IsKeyPressed(KEY_HOME)) { g_caret = 0; if (!shift) g_anchor = 0; }
        if (IsKeyPressed(KEY_END)) { g_caret = (int)s.size(); if (!shift) g_anchor = g_caret; }
        if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER) || IsKeyPressed(KEY_ESCAPE)) focusId = -1;
        g_tiChars.clear();
        g_tiKeys.clear();
        g_tiCtrl = g_tiShift = false;
    }
    ensureGlyphs(s);
    float ty = r.y + (r.height - 18) / 2;
    if (!focused) {
        std::string shown = fit(s, r.width - 20, 16);
        text(shown, r.x + pad, ty, 16, col::text);
        return changed;
    }
    // ---- the focused field: the text as it is, scrolled so that the caret stays in view
    float total = rawW(s, s.size()), cx = rawW(s, (size_t)g_caret);
    if (total <= innerW) g_tScroll = 0;
    else {
        if (cx - g_tScroll > innerW) g_tScroll = cx - innerW;
        if (cx - g_tScroll < 0) g_tScroll = cx;
        g_tScroll = std::clamp(g_tScroll, 0.0f, total - innerW);
    }
    // the visible characters
    size_t i0 = 0, i1 = s.size();
    while (i0 < s.size() && rawW(s, nextCpI(s, i0)) <= g_tScroll + 0.5f) i0 = nextCpI(s, i0);
    while (i1 > i0 && rawW(s, i1) - g_tScroll > innerW + 0.5f) i1 = prevCpI(s, i1);
    float x0 = r.x + pad - g_tScroll;
    if (g_caret != g_anchor) {
        float a = std::clamp(rawW(s, (size_t)std::min(g_caret, g_anchor)) - g_tScroll, 0.0f, innerW);
        float b = std::clamp(rawW(s, (size_t)std::max(g_caret, g_anchor)) - g_tScroll, 0.0f, innerW);
        DrawRectangleRec({r.x + pad + a, r.y + 6, b - a, r.height - 12}, Color{56, 189, 248, 90});
    }
    std::string vis = s.substr(i0, i1 - i0);
    DrawTextEx(pick(tSize(), false), vis.c_str(), {std::round(x0 + rawW(s, i0)), std::round(ty)}, tSize(), 0, col::text);
    if ((int)(GetTime() * 2) % 2 == 0 || g_tDrag) {
        float x = r.x + pad + cx - g_tScroll;
        DrawRectangle((int)x, (int)(r.y + 7), 2, (int)(r.height - 14), col::accent);
    }
    return changed;
}

// a press on a number field: a drag sideways changes the value, a click (no drag) starts typing
int g_numPressId = -1;
bool g_numDragging = false;
float g_numPressX = 0;
double g_numV0 = 0;

bool numberField(Rectangle r, double& v, int prec, const char* unit, int id, double lo, double hi, double dragStep, double snap) {
    int& editId = g_editId;
    static std::string buf;
    bool h = hovered(r);
    if (h) overNow = true;
    bool changed = false;
    if (g_numPressId == id) {
        g_focusSeen = true; // (keeps the press alive while the mouse is off the field)
        float dx = GetMousePosition().x - g_numPressX;
        if (!g_numDragging && std::fabs(dx) > 4) g_numDragging = true;
        if (g_numDragging) {
            g_ewReq = true;
            double per = dragStep > 0 ? dragStep : std::max(std::pow(10.0, -prec), 0.005 * std::max(std::fabs(g_numV0), std::pow(10.0, 1 - prec)));
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) per *= 0.2;
            double nv = g_numV0 + (dx - (dx > 0 ? 4 : -4)) * per;
            if (snap > 0) nv = g_numV0 + std::round((nv - g_numV0) / snap) * snap;
            else {
                double q = std::pow(10.0, prec);
                nv = std::round(nv * q) / q;
            }
            nv = std::clamp(nv, lo, hi);
            if (nv != v) { v = nv; changed = true; }
        }
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            bool click = !g_numDragging;
            g_numPressId = -1;
            g_numDragging = false;
            if (click) {
                // a click: type a value
                editId = id;
                focusId = id;
                char b[48];
                std::snprintf(b, sizeof b, "%.*f", prec, v);
                buf = b;
            }
        }
    }
    auto apply = [&]() {
        char* end = nullptr;
        double nv = std::strtod(buf.c_str(), &end);
        if (end != buf.c_str() && std::isfinite(nv)) {
            nv = std::clamp(nv, lo, hi);
            if (nv != v) { v = nv; changed = true; }
        }
        editId = -1;
        if (focusId == id) focusId = -1;
    };
    if (editId == id) {
        int ch;
        while ((ch = GetCharPressed()) > 0)
            if ((ch >= '0' && ch <= '9') || ch == '.' || ch == '-' || ch == '+' || ch == 'e' || ch == 'E')
                if (buf.size() < 24) buf.push_back((char)ch);
        if ((IsKeyPressed(KEY_BACKSPACE) || IsKeyPressedRepeat(KEY_BACKSPACE)) && !buf.empty()) buf.pop_back();
        if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER) || IsKeyPressed(KEY_TAB)) apply();
        else if (IsKeyPressed(KEY_ESCAPE)) { editId = -1; if (focusId == id) focusId = -1; }
        else if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !h) apply();
    } else if (h && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !anyClickConsumed && g_numPressId < 0) {
        anyClickConsumed = true;
        g_numPressId = id;
        g_numDragging = false;
        g_numPressX = GetMousePosition().x;
        g_numV0 = v;
    }
    if (h && !(editId == id) && g_numPressId < 0) g_ewReq = true; // (the sideways arrows: it can be dragged)
    bool editing = editId == id;
    if (editing) g_focusSeen = true;
    bool dragging = g_numPressId == id && g_numDragging;
    DrawRectangleRounded(r, 0.3f, 6, editing ? Color{20, 28, 40, 255} : (h || dragging ? col::hover : Color{20, 26, 36, 255}));
    DrawRectangleRoundedLinesEx(r, 0.3f, 6, 1.0f, editing || dragging ? col::accent : col::line);
    std::string shown;
    if (editing) shown = buf;
    else {
        char b[64];
        std::snprintf(b, sizeof b, "%.*f%s", prec, v, unit ? unit : "");
        shown = b;
    }
    Vector2 m = measure(shown, 14);
    text(shown, r.x + r.width - m.x - 8, r.y + (r.height - m.y) / 2, 14, col::text);
    if (editing && ((int)(GetTime() * 2) % 2 == 0))
        DrawRectangle((int)(r.x + r.width - 6), (int)(r.y + 6), 2, (int)(r.height - 12), col::accent);
    return changed;
}

// held buttons: the rectangle of the one held, when it fires next
Rectangle g_repRect{};
bool g_repOn = false;
double g_repT0 = 0, g_repNext = 0;
bool repeatButton(Rectangle r, Icon ic, bool enabled) {
    bool held = g_repOn && g_repRect.x == r.x && g_repRect.y == r.y && g_repRect.width == r.width && g_repRect.height == r.height;
    if (held && !IsMouseButtonDown(MOUSE_BUTTON_LEFT)) g_repOn = held = false;
    bool fire = iconButton(r, ic, held, nullptr, enabled);
    double now = GetTime();
    if (fire) {
        g_repOn = true;
        g_repRect = r;
        g_repT0 = now;
        g_repNext = now + 0.4;
        return true;
    }
    if (held && enabled && hovered(r) && now >= g_repNext) {
        double since = now - g_repT0;
        g_repNext = now + (since > 2.0 ? 0.025 : since > 1.0 ? 0.05 : 0.09); // (faster the longer it is held)
        return true;
    }
    return false;
}

bool spinField(Rectangle r, double& v, double step, double lo, double hi, int prec, const char* unit, int id) {
    bool ch = false;
    double k = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT) ? 5 : 1;
    if (repeatButton({r.x, r.y, 30, r.height}, Icon::Minus, v > lo)) { v = std::max(lo, v - step * k); ch = true; }
    if (numberField({r.x + 34, r.y, r.width - 68, r.height}, v, prec, unit, id, lo, hi, step / 8, step)) ch = true;
    if (repeatButton({r.x + r.width - 30, r.y, 30, r.height}, Icon::Plus, v < hi)) { v = std::min(hi, v + step * k); ch = true; }
    if (ch) {
        double q = std::pow(10.0, prec);
        v = std::clamp(std::round(v * q) / q, lo, hi);
    }
    return ch;
}

void tooltip(const std::string& s) {
    if (!layerOccluded()) tipText = s; // (not through a panel above)
}

float beginScroll(int id, Rectangle r, float contentH) {
    float& off = scrolls[id];
    float maxOff = std::max(0.0f, contentH - r.height);
    if (!g_inputBlocked && CheckCollisionPointRec(GetMousePosition(), r) && !layerOccluded()) {
        overNow = true;
        off -= GetMouseWheelMove() * 48.0f;
    }
    off = std::clamp(off, 0.0f, maxOff);
    beginScissor(r);
    clipStack.push_back(r);
    if (maxOff > 0) {
        float th = r.height * r.height / contentH;
        float ty = r.y + (r.height - th) * (off / maxOff);
        DrawRectangleRounded({r.x + r.width - 4, ty, 3, th}, 1.0f, 4, Color{80, 92, 115, 200});
    }
    return off;
}

void setScroll(int id, float off) { scrolls[id] = off; }

void endScroll() {
    endScissor();
    if (!clipStack.empty()) clipStack.pop_back();
    if (!clipStack.empty()) beginScissor(clipStack.back());
}

bool inClip(Vector2 p) { return clipStack.empty() || CheckCollisionPointRec(p, clipStack.back()); }

void dim(float a) {
    DrawRectangle(0, 0, (int)SW() + 1, (int)SH() + 1, Color{4, 6, 10, (unsigned char)(a * 255)});
    overNow = true;
}

// draws pending tooltip (call at the very end of the frame)
void drawTooltip();
void drawTooltip() {
    if (tipText.empty()) return;
    Vector2 m = GetMousePosition();
    Vector2 sz = measure(tipText, 15);
    Rectangle r{m.x + 16, m.y + 18, sz.x + 20, sz.y + 12};
    if (r.x + r.width > SW()) r.x = SW() - r.width - 4;
    if (r.y + r.height > SH()) r.y = m.y - r.height - 6;
    DrawRectangleRounded(r, 0.3f, 6, Color{30, 38, 52, 245});
    DrawRectangleRoundedLinesEx(r, 0.3f, 6, 1.0f, col::line);
    text(tipText, r.x + 10, r.y + 6, 15, col::text);
}

// ------------------------------------------------------------------ vector icons
void icon(Icon ic, Rectangle r, Color c) {
    float x = r.x, y = r.y, w = r.width, h = r.height;
    float cx = x + w / 2, cy = y + h / 2;
    float t = std::max(1.5f, w * 0.1f);
    auto L = [&](float x1, float y1, float x2, float y2) { DrawLineEx({x + x1 * w, y + y1 * h}, {x + x2 * w, y + y2 * h}, t, c); };
    auto T = [&](float x1, float y1, float x2, float y2, float x3, float y3) {
        Vector2 a{x + x1 * w, y + y1 * h}, b{x + x2 * w, y + y2 * h}, d{x + x3 * w, y + y3 * h};
        DrawTriangle(a, b, d, c);
        DrawTriangle(a, d, b, c);
    };
    switch (ic) {
    case Icon::Rocket:
        T(0.5f, 0.02f, 0.32f, 0.35f, 0.68f, 0.35f);
        DrawRectangleV({x + 0.34f * w, y + 0.33f * h}, {0.32f * w, 0.4f * h}, c);
        T(0.34f, 0.5f, 0.12f, 0.85f, 0.34f, 0.73f);
        T(0.66f, 0.5f, 0.88f, 0.85f, 0.66f, 0.73f);
        T(0.4f, 0.75f, 0.6f, 0.75f, 0.5f, 0.98f);
        break;
    case Icon::Play: T(0.25f, 0.15f, 0.25f, 0.85f, 0.85f, 0.5f); break;
    case Icon::Pause:
        DrawRectangleV({x + 0.22f * w, y + 0.15f * h}, {0.18f * w, 0.7f * h}, c);
        DrawRectangleV({x + 0.6f * w, y + 0.15f * h}, {0.18f * w, 0.7f * h}, c);
        break;
    case Icon::Forward:
        T(0.08f, 0.2f, 0.08f, 0.8f, 0.5f, 0.5f);
        T(0.5f, 0.2f, 0.5f, 0.8f, 0.92f, 0.5f);
        break;
    case Icon::Backward:
        T(0.92f, 0.2f, 0.92f, 0.8f, 0.5f, 0.5f);
        T(0.5f, 0.2f, 0.5f, 0.8f, 0.08f, 0.5f);
        break;
    case Icon::Map:
        DrawCircleLinesV({cx, cy}, w * 0.12f, c);
        DrawRing({cx, cy}, w * 0.40f - t * 0.5f, w * 0.40f + t * 0.1f, 0, 360, 36, c);
        DrawCircleV({cx + w * 0.28f, cy - h * 0.28f}, w * 0.08f, c);
        DrawCircleV({cx, cy}, w * 0.14f, c);
        break;
    case Icon::Factory:
        DrawRectangleV({x + 0.1f * w, y + 0.5f * h}, {0.8f * w, 0.4f * h}, c);
        T(0.1f, 0.5f, 0.1f, 0.25f, 0.35f, 0.5f);
        T(0.35f, 0.5f, 0.35f, 0.25f, 0.6f, 0.5f);
        DrawRectangleV({x + 0.68f * w, y + 0.1f * h}, {0.14f * w, 0.42f * h}, c);
        break;
    case Icon::Gear:
        DrawRing({cx, cy}, w * 0.18f, w * 0.34f, 0, 360, 24, c);
        for (int i = 0; i < 8; i++) {
            float a = i * PI / 4;
            DrawLineEx({cx + cosf(a) * w * 0.3f, cy + sinf(a) * h * 0.3f}, {cx + cosf(a) * w * 0.47f, cy + sinf(a) * h * 0.47f}, w * 0.14f, c);
        }
        break;
    case Icon::Save:
        DrawRectangleLinesEx({x + 0.12f * w, y + 0.12f * h, 0.76f * w, 0.76f * h}, t, c);
        DrawRectangleV({x + 0.3f * w, y + 0.12f * h}, {0.4f * w, 0.25f * h}, c);
        DrawRectangleV({x + 0.28f * w, y + 0.55f * h}, {0.44f * w, 0.2f * h}, c);
        break;
    case Icon::Folder:
        DrawRectangleV({x + 0.08f * w, y + 0.32f * h}, {0.84f * w, 0.52f * h}, c);
        DrawRectangleV({x + 0.08f * w, y + 0.2f * h}, {0.35f * w, 0.14f * h}, c);
        break;
    case Icon::Trash:
        DrawRectangleV({x + 0.12f * w, y + 0.18f * h}, {0.76f * w, 0.1f * h}, c);
        DrawRectangleV({x + 0.38f * w, y + 0.08f * h}, {0.24f * w, 0.1f * h}, c);
        T(0.2f, 0.32f, 0.8f, 0.32f, 0.72f, 0.92f);
        T(0.2f, 0.32f, 0.72f, 0.92f, 0.28f, 0.92f);
        break;
    case Icon::Up: L(0.5f, 0.85f, 0.5f, 0.18f); L(0.2f, 0.45f, 0.5f, 0.15f); L(0.8f, 0.45f, 0.5f, 0.15f); break;
    case Icon::Down: L(0.5f, 0.15f, 0.5f, 0.82f); L(0.2f, 0.55f, 0.5f, 0.85f); L(0.8f, 0.55f, 0.5f, 0.85f); break;
    case Icon::Plus: L(0.5f, 0.15f, 0.5f, 0.85f); L(0.15f, 0.5f, 0.85f, 0.5f); break;
    case Icon::Minus: L(0.15f, 0.5f, 0.85f, 0.5f); break;
    case Icon::Close: L(0.2f, 0.2f, 0.8f, 0.8f); L(0.8f, 0.2f, 0.2f, 0.8f); break;
    case Icon::Check: L(0.15f, 0.55f, 0.4f, 0.8f); L(0.4f, 0.8f, 0.88f, 0.22f); break;
    case Icon::Hyper:
        for (int i = 0; i < 8; i++) {
            float a = i * PI / 4 + 0.2f;
            DrawLineEx({cx + cosf(a) * w * 0.14f, cy + sinf(a) * h * 0.14f}, {cx + cosf(a) * w * 0.48f, cy + sinf(a) * h * 0.48f}, t, c);
        }
        DrawCircleV({cx, cy}, w * 0.1f, c);
        break;
    case Icon::Target:
        DrawRing({cx, cy}, w * 0.34f, w * 0.34f + t, 0, 360, 32, c);
        DrawCircleV({cx, cy}, w * 0.1f, c);
        L(0.5f, 0.0f, 0.5f, 0.2f); L(0.5f, 0.8f, 0.5f, 1.0f); L(0.0f, 0.5f, 0.2f, 0.5f); L(0.8f, 0.5f, 1.0f, 0.5f);
        break;
    case Icon::Planet:
        DrawCircleV({cx, cy}, w * 0.28f, c);
        DrawEllipseLines((int)cx, (int)cy, w * 0.48f, h * 0.14f, c);
        break;
    case Icon::Star:
        for (int i = 0; i < 5; i++) {
            float a0 = -PI / 2 + i * 2 * PI / 5, a1 = a0 + PI / 5, a2 = a0 - PI / 5;
            Vector2 o{cx + cosf(a0) * w * 0.48f, cy + sinf(a0) * h * 0.48f};
            Vector2 p1{cx + cosf(a1) * w * 0.2f, cy + sinf(a1) * h * 0.2f};
            Vector2 p2{cx + cosf(a2) * w * 0.2f, cy + sinf(a2) * h * 0.2f};
            DrawTriangle(o, p2, p1, c);
            DrawTriangle(o, p1, p2, c);
        }
        DrawCircleV({cx, cy}, w * 0.21f, c);
        break;
    case Icon::Bolt:
        T(0.6f, 0.02f, 0.18f, 0.58f, 0.5f, 0.55f);
        T(0.4f, 0.98f, 0.82f, 0.42f, 0.5f, 0.45f);
        break;
    case Icon::Info:
        DrawRing({cx, cy}, w * 0.42f, w * 0.42f + t, 0, 360, 32, c);
        DrawCircleV({cx, y + 0.3f * h}, w * 0.07f, c);
        L(0.5f, 0.45f, 0.5f, 0.75f);
        break;
    case Icon::Exit:
        L(0.55f, 0.12f, 0.15f, 0.12f); L(0.15f, 0.12f, 0.15f, 0.88f); L(0.15f, 0.88f, 0.55f, 0.88f);
        L(0.4f, 0.5f, 0.92f, 0.5f); L(0.72f, 0.3f, 0.92f, 0.5f); L(0.72f, 0.7f, 0.92f, 0.5f);
        break;
    case Icon::Stage:
        DrawRectangleV({x + 0.3f * w, y + 0.05f * h}, {0.4f * w, 0.35f * h}, c);
        DrawRectangleV({x + 0.3f * w, y + 0.55f * h}, {0.4f * w, 0.4f * h}, c);
        L(0.1f, 0.47f, 0.9f, 0.47f);
        break;
    case Icon::Ship:
        T(0.5f, 0.05f, 0.2f, 0.9f, 0.8f, 0.9f);
        break;
    case Icon::Copy:
        DrawRectangleLinesEx({x + 0.1f * w, y + 0.28f * h, 0.55f * w, 0.62f * h}, t, c);
        DrawRectangleLinesEx({x + 0.35f * w, y + 0.08f * h, 0.55f * w, 0.62f * h}, t, c);
        break;
    case Icon::Pencil: {
        // a pencil drawn diagonally: body, the sharpened tip and the eraser end
        Vector2 a{x + 0.78f * w, y + 0.12f * h}, b{x + 0.25f * w, y + 0.65f * h};
        DrawLineEx(a, b, w * 0.2f, c);
        DrawTriangle({x + 0.12f * w, y + 0.88f * h}, {x + 0.18f * w, y + 0.56f * h}, {x + 0.44f * w, y + 0.82f * h}, c);
        DrawTriangle({x + 0.12f * w, y + 0.88f * h}, {x + 0.44f * w, y + 0.82f * h}, {x + 0.18f * w, y + 0.56f * h}, c);
        DrawLineEx({x + 0.68f * w, y + 0.02f * h}, {x + 0.92f * w, y + 0.26f * h}, t, c);
        break;
    }
    case Icon::Eye:
        DrawEllipseLines((int)cx, (int)cy, w * 0.45f, h * 0.26f, c);
        DrawCircleV({cx, cy}, w * 0.14f, c);
        break;
    case Icon::EyeOff:
        DrawEllipseLines((int)cx, (int)cy, w * 0.45f, h * 0.26f, c);
        DrawCircleV({cx, cy}, w * 0.14f, c);
        L(0.12f, 0.88f, 0.88f, 0.12f);
        break;
    case Icon::Trophy: {
        // cup with handles on a stem and a base
        DrawCircleSector({cx, y + 0.18f * h}, w * 0.3f, 0, 180, 16, c);
        DrawRectangleV({x + 0.2f * w, y + 0.08f * h}, {0.6f * w, 0.1f * h}, c);
        DrawRing({x + 0.2f * w, y + 0.3f * h}, w * 0.1f, w * 0.1f + t, 90, 270, 12, c);
        DrawRing({x + 0.8f * w, y + 0.3f * h}, w * 0.1f, w * 0.1f + t, -90, 90, 12, c);
        L(0.5f, 0.48f, 0.5f, 0.78f);
        DrawRectangleV({x + 0.28f * w, y + 0.78f * h}, {0.44f * w, 0.12f * h}, c);
        break;
    }
    case Icon::Trajectory: {
        // a small planet low on the left, an orbit arc over it and the vessel on the arc
        Vector2 pc{x + 0.3f * w, y + 0.72f * h};
        DrawCircleV(pc, w * 0.16f, c);
        DrawRing(pc, w * 0.56f - t * 0.5f, w * 0.56f + t * 0.5f, -95, 5, 24, c);
        DrawCircleV({pc.x + w * 0.56f * cosf(-50 * DEG2RAD), pc.y + w * 0.56f * sinf(-50 * DEG2RAD)}, w * 0.1f, c);
        break;
    }
    case Icon::Flame:
        T(0.5f, 0.02f, 0.2f, 0.65f, 0.8f, 0.65f);
        DrawCircleV({cx, y + 0.68f * h}, w * 0.3f, c);
        break;
    case Icon::Sail:
        T(0.12f, 0.12f, 0.88f, 0.12f, 0.88f, 0.88f);
        L(0.12f, 0.12f, 0.12f, 0.95f);
        break;
    case Icon::NavOrbit:
        DrawRing({cx, cy}, w * 0.36f - t * 0.5f, w * 0.36f + t * 0.5f, 200, 520, 36, c);
        T(0.80f, 0.30f, 0.98f, 0.52f, 0.66f, 0.56f);
        DrawCircleV({cx, cy}, w * 0.08f, c);
        break;
    case Icon::NavPan:
        L(0.5f, 0.08f, 0.5f, 0.92f); L(0.08f, 0.5f, 0.92f, 0.5f);
        T(0.5f, 0.02f, 0.38f, 0.18f, 0.62f, 0.18f); T(0.5f, 0.98f, 0.38f, 0.82f, 0.62f, 0.82f);
        T(0.02f, 0.5f, 0.18f, 0.38f, 0.18f, 0.62f); T(0.98f, 0.5f, 0.82f, 0.38f, 0.82f, 0.62f);
        break;
    case Icon::NavZoom:
        DrawRing({x + 0.42f * w, y + 0.42f * h}, w * 0.26f - t * 0.5f, w * 0.26f + t * 0.5f, 0, 360, 32, c);
        L(0.62f, 0.62f, 0.92f, 0.92f);
        L(0.30f, 0.42f, 0.54f, 0.42f); L(0.42f, 0.30f, 0.42f, 0.54f);
        break;
    case Icon::NavFit:
        L(0.08f, 0.08f, 0.34f, 0.08f); L(0.08f, 0.08f, 0.08f, 0.34f);
        L(0.92f, 0.08f, 0.66f, 0.08f); L(0.92f, 0.08f, 0.92f, 0.34f);
        L(0.08f, 0.92f, 0.34f, 0.92f); L(0.08f, 0.92f, 0.08f, 0.66f);
        L(0.92f, 0.92f, 0.66f, 0.92f); L(0.92f, 0.92f, 0.92f, 0.66f);
        DrawRectangleV({x + 0.36f * w, y + 0.36f * h}, {0.28f * w, 0.28f * h}, c);
        break;
    case Icon::Home:
        T(0.5f, 0.08f, 0.06f, 0.5f, 0.94f, 0.5f);
        DrawRectangleV({x + 0.18f * w, y + 0.48f * h}, {0.64f * w, 0.44f * h}, c);
        break;
    case Icon::Base: {
        // a habitat dome on the ground line, a small module beside it and a mast with a dish
        Vector2 ctr{x + 0.40f * w, y + 0.80f * h};
        DrawRing(ctr, 0.30f * w - t, 0.30f * w, 180, 360, 24, c);
        DrawLineEx({x + 0.04f * w, y + 0.80f * h}, {x + 0.96f * w, y + 0.80f * h}, t, c);
        DrawRectangleV({x + 0.30f * w, y + 0.62f * h}, {0.20f * w, 0.18f * h}, c); // (the door)
        DrawRectangleLinesEx({x + 0.70f * w, y + 0.60f * h, 0.20f * w, 0.20f * h}, t * 0.8f, c);
        DrawLineEx({x + 0.80f * w, y + 0.60f * h}, {x + 0.80f * w, y + 0.18f * h}, t * 0.8f, c);
        DrawRing({x + 0.80f * w, y + 0.18f * h}, 0.06f * w, 0.13f * w, 200, 340, 10, c);
        break;
    }
    case Icon::Cube: {
        Vector2 a{x + 0.5f * w, y + 0.08f * h}, b{x + 0.9f * w, y + 0.28f * h}, cc{x + 0.5f * w, y + 0.48f * h}, d{x + 0.1f * w, y + 0.28f * h};
        Vector2 b2{x + 0.9f * w, y + 0.72f * h}, c2{x + 0.5f * w, y + 0.92f * h}, d2{x + 0.1f * w, y + 0.72f * h};
        DrawLineEx(a, b, t, c); DrawLineEx(b, cc, t, c); DrawLineEx(cc, d, t, c); DrawLineEx(d, a, t, c);
        DrawLineEx(b, b2, t, c); DrawLineEx(d, d2, t, c); DrawLineEx(cc, c2, t, c); DrawLineEx(b2, c2, t, c); DrawLineEx(d2, c2, t, c);
        break;
    }
    case Icon::Move:
        L(0.5f, 0.1f, 0.5f, 0.9f); L(0.1f, 0.5f, 0.9f, 0.5f);
        T(0.5f, 0.02f, 0.36f, 0.2f, 0.64f, 0.2f); T(0.98f, 0.5f, 0.8f, 0.36f, 0.8f, 0.64f);
        T(0.5f, 0.98f, 0.36f, 0.8f, 0.64f, 0.8f); T(0.02f, 0.5f, 0.2f, 0.36f, 0.2f, 0.64f);
        break;
    case Icon::Rotate:
        DrawRing({cx, cy}, w * 0.34f - t * 0.5f, w * 0.34f + t * 0.5f, 30, 300, 32, c);
        T(0.78f, 0.62f, 0.98f, 0.70f, 0.82f, 0.88f);
        break;
    case Icon::Undo:
        DrawRing({x + 0.55f * w, y + 0.58f * h}, w * 0.28f - t * 0.5f, w * 0.28f + t * 0.5f, 180, 450, 24, c);
        T(0.08f, 0.40f, 0.42f, 0.24f, 0.36f, 0.58f);
        break;
    case Icon::Redo:
        DrawRing({x + 0.45f * w, y + 0.58f * h}, w * 0.28f - t * 0.5f, w * 0.28f + t * 0.5f, 90, 360, 24, c);
        T(0.92f, 0.40f, 0.58f, 0.24f, 0.64f, 0.58f);
        break;
    case Icon::Mirror:
        L(0.5f, 0.05f, 0.5f, 0.95f);
        T(0.42f, 0.2f, 0.42f, 0.8f, 0.08f, 0.8f);
        DrawTriangleLines({x + 0.58f * w, y + 0.2f * h}, {x + 0.92f * w, y + 0.8f * h}, {x + 0.58f * w, y + 0.8f * h}, c);
        break;
    case Icon::Wire:
        DrawRectangleLinesEx({x + 0.1f * w, y + 0.1f * h, 0.8f * w, 0.8f * h}, t, c);
        L(0.1f, 0.1f, 0.9f, 0.9f); L(0.5f, 0.1f, 0.5f, 0.9f); L(0.1f, 0.5f, 0.9f, 0.5f);
        break;
    case Icon::Xray:
        DrawRing({x + 0.4f * w, y + 0.5f * h}, w * 0.28f - t * 0.5f, w * 0.28f + t * 0.5f, 0, 360, 24, c);
        DrawCircleV({x + 0.62f * w, y + 0.5f * h}, w * 0.28f, Color{c.r, c.g, c.b, (unsigned char)(c.a / 2)});
        break;
    case Icon::Book: // open book
        L(0.5f, 0.26f, 0.5f, 0.86f);
        L(0.08f, 0.18f, 0.5f, 0.26f); L(0.08f, 0.18f, 0.08f, 0.78f); L(0.08f, 0.78f, 0.5f, 0.86f);
        L(0.92f, 0.18f, 0.5f, 0.26f); L(0.92f, 0.18f, 0.92f, 0.78f); L(0.92f, 0.78f, 0.5f, 0.86f);
        L(0.18f, 0.38f, 0.4f, 0.42f); L(0.18f, 0.54f, 0.4f, 0.58f);
        L(0.82f, 0.38f, 0.6f, 0.42f); L(0.82f, 0.54f, 0.6f, 0.58f);
        break;
    case Icon::Keyboard: { // keyboard outline, two rows of keys and a space bar
        DrawRectangleRoundedLinesEx({x + w * 0.04f, y + h * 0.22f, w * 0.92f, h * 0.58f}, 0.25f, 6, t * 0.8f, c);
        float k = std::max(1.5f, w * 0.09f);
        for (int i = 0; i < 5; i++) {
            DrawRectangle((int)(x + w * (0.16f + i * 0.155f)), (int)(y + h * 0.34f), (int)k, (int)k, c);
            if (i < 4) DrawRectangle((int)(x + w * (0.235f + i * 0.155f)), (int)(y + h * 0.48f), (int)k, (int)k, c);
        }
        L(0.3f, 0.67f, 0.7f, 0.67f);
        break;
    }
    default: break;
    }
}

} // namespace ui
