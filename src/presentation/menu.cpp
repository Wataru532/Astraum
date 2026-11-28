// The start screen (title, new / load map, tutorials, settings), the settings with their sub-dialogs (key
// bindings, credits and licences), the crash report dialog and the short notice at the top of the screen
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include "app.hpp"
#include "audio.hpp"
#include "brand.hpp"
#include "keys.hpp"
#include "mem.hpp"
#include "nav.hpp"
#include "render.hpp"
#include "rl.hpp"
#include "screens.hpp"
#include "screenshot.hpp"
#include "session.hpp"
#include "terrain.hpp"
#include "tutorial.hpp"
#include "ui.hpp"
#include "winshell.hpp"
#include "crash.hpp"

using namespace ui;

void keyBindingsModal(App& app) {
    using keys::A;
    static int capA = -1, capS = 0; // slot being captured
    if (!keys::capturing()) capA = -1;
    dim();
    float SW = ui::SW(), SH = ui::SH();
    float h = std::min(820.0f, SH - 40);
    Rectangle r{(SW - 920) / 2, (SH - h) / 2, 920, h};
    panel(r, Color{20, 26, 36, 252}, 14);
    icon(Icon::Keyboard, {r.x + 24, r.y + 20, 26, 26}, col::accent);
    text("Key bindings", r.x + 60, r.y + 18, 24, col::text, true);
    bool wasCapturing = keys::capturing();
    if (wasCapturing) {
        keys::Binding b;
        if (keys::pollCapture(b) && capA >= 0) {
            A a = (A)capA;
            for (int s = 0; s < keys::SLOTS; s++) // the same input twice on one action is pointless
                if (s != capS && keys::slot(a, s) == b) keys::slot(a, s) = {};
            keys::slot(a, capS) = b;
            keys::save(keysFile(app));
            capA = -1;
        }
    }
    bool closeNow = iconButton({r.x + r.width - 48, r.y + 16, 32, 32}, Icon::Close) || (!wasCapturing && IsKeyPressed(KEY_ESCAPE));
    float y = wrap("Click a slot, then press a key or a mouse button (Ctrl / Shift / Alt can be held). Esc cancels, right-click clears a slot.",
                   r.x + 24, r.y + 58, r.width - 48, 13, col::dim) + 8;
    // ---- list
    Rectangle lr{r.x + 16, y, r.width - 32, r.y + r.height - 72 - y};
    struct Group { const char* title; keys::Ctx ctx; };
    const Group groups[] = {{"FLIGHT", keys::Flight}, {"FACTORY AND CAD", keys::Editor}, {"GENERAL", keys::Global}};
    const float rowH = 36, headH = 34;
    float contentH = 8;
    for (const Group& g : groups) {
        contentH += headH;
        for (int a = 0; a < keys::N; a++)
            if (keys::info((A)a).ctx == g.ctx) contentH += rowH;
    }
    float off = beginScroll(9101, lr, contentH);
    float ly = lr.y - off + 4;
    const float slotW = 214, nameX = lr.x + 12;
    Vector2 mouse = GetMousePosition();
    for (const Group& g : groups) {
        text(g.title, nameX, ly + 12, 13, col::dim, true);
        ly += headH;
        for (int a = 0; a < keys::N; a++) {
            const keys::Info& in = keys::info((A)a);
            if (in.ctx != g.ctx) continue;
            if (ly + rowH >= lr.y && ly <= lr.y + lr.height) {
                text(fit(in.name, lr.width - 2 * slotW - 60, 15), nameX, ly + 9, 15, col::text);
                for (int s = 0; s < keys::SLOTS; s++) {
                    Rectangle sr{lr.x + lr.width - 16 - (keys::SLOTS - s) * (slotW + 8), ly + 3, slotW, rowH - 6};
                    keys::Binding b = keys::slot((A)a, s);
                    bool cap = capA == a && capS == s && keys::capturing();
                    A other = keys::conflict((A)a, b);
                    std::string lbl = cap ? tr("Press a key...") : (b.code ? keys::name(b) : std::string("-"));
                    bool hov = !wasCapturing && CheckCollisionPointRec(mouse, sr) && inClip(mouse) && !inputBlocked();
                    if (button(sr, lbl, cap, !wasCapturing || cap) && !wasCapturing) {
                        capA = a;
                        capS = s;
                        keys::beginCapture();
                    }
                    if (other != A::COUNT && !cap) {
                        DrawRectangleRoundedLinesEx(sr, 0.35f, 6, 1.5f, col::warn);
                        if (hov) tooltip(trf("Also used by: {0}", tr(keys::info(other).name)));
                    } else if (hov) tooltip(tr("Click to change, right-click to clear"));
                    if (hov && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && b.code) {
                        keys::slot((A)a, s) = {};
                        keys::save(keysFile(app));
                    }
                }
            }
            ly += rowH;
        }
    }
    endScroll();
    // ---- footer
    float by = r.y + r.height - 58;
    if (button({r.x + 24, by, 240, 40}, "Reset all to defaults", false, !wasCapturing, Icon::Backward)) {
        keys::resetDefaults();
        keys::save(keysFile(app));
    }
    if (primaryButton({r.x + r.width - 184, by, 160, 40}, "Done", Icon::Check, !wasCapturing) || closeNow) {
        keys::cancelCapture();
        capA = -1;
        app.keysOpen = false;
    }
}

// Settings -> Credits and licences: THIRD_PARTY_NOTICES.txt (tools/make_notices.py), embedded as app.rc 401
void creditsModal(App& app) {
    static std::string notes;
    static std::vector<std::string> lines;
    if (lines.empty()) {
        const void* data = nullptr;
        size_t size = 0;
        if (winshell::resource(401, data, size) && data && size) notes.assign((const char*)data, size);
        else notes = "THIRD_PARTY_NOTICES.txt is missing from this build.";
        size_t pos = 0;
        while (pos <= notes.size()) {
            size_t e = notes.find('\n', pos);
            if (e == std::string::npos) e = notes.size();
            std::string ln = notes.substr(pos, e - pos);
            if (!ln.empty() && ln.back() == '\r') ln.pop_back();
            lines.push_back(std::move(ln));
            pos = e + 1;
        }
    }
    dim();
    float SW = ui::SW(), SH = ui::SH();
    float W = std::min(940.0f, SW - 24), H = std::min(860.0f, SH - 24);
    Rectangle r{(SW - W) / 2, (SH - H) / 2, W, H};
    panel(r, Color{20, 26, 36, 252}, 14);
    icon(Icon::Book, {r.x + 24, r.y + 20, 24, 24}, col::accent);
    text("Credits and licences", r.x + 58, r.y + 18, 24, col::text, true);
    bool closeNow = iconButton({r.x + r.width - 48, r.y + 16, 32, 32}, Icon::Close) || IsKeyPressed(KEY_ESCAPE);
    float y = wrap("Astraum uses pictures and data from NASA, the USGS, ESA, DLR and other institutions, and is built with open-source libraries. "
                   "Their credits and licence texts follow (in English, as published). This text is also in THIRD_PARTY_NOTICES.txt.",
                   r.x + 24, r.y + 58, r.width - 48, 14, col::dim) + 8;
    Rectangle box{r.x + 24, y, r.width - 48, r.y + r.height - 76 - y};
    DrawRectangleRounded(box, 0.02f, 6, Color{10, 13, 19, 255});
    const float lh = 18;
    Rectangle inner{box.x + 12, box.y + 8, box.width - 24, box.height - 16};
    float off = beginScroll(9002, inner, lines.size() * lh + 8);
    for (size_t i = 0; i < lines.size(); i++) {
        float ly = inner.y + i * lh - off;
        if (ly <= inner.y - lh || ly >= inner.y + inner.height) continue;
        const std::string& ln = lines[i];
        bool head = i + 1 < lines.size() && i > 0 && lines[i - 1].rfind("=====", 0) == 0 && lines[i + 1].rfind("=====", 0) == 0;
        if (ln.rfind("=====", 0) == 0) {
            DrawRectangle((int)inner.x, (int)(ly + lh / 2), (int)(inner.width - 12), 1, Color{48, 58, 76, 255});
            continue;
        }
        // licence texts are shown word for word: never through the translation table (it would turn "Moon" into "月")
        textRaw(ln, inner.x, ly, 12.5f, head ? col::accent : Color{190, 200, 215, 255}, head);
    }
    endScroll();
    float by = r.y + r.height - 58;
    if (button({r.x + 24, by, 200, 40}, "Copy text", false, true, Icon::Copy)) {
        SetClipboardText(notes.c_str());
        app.setStatus(tr("Credits and licences copied to the clipboard"));
    }
    if (primaryButton({r.x + r.width - 184, by, 160, 40}, "Done", Icon::Check) || closeNow) app.creditsOpen = false;
}

// borderless full screen on the window's monitor (F11 / Settings); the window comes back to its size and place
void setFullscreen(App& app, bool on) {
    bool now = IsWindowState(FLAG_BORDERLESS_WINDOWED_MODE);
    if (on != now) ToggleBorderlessWindowed();
    app.fullscreen = IsWindowState(FLAG_BORDERLESS_WINDOWED_MODE);
    savePrefs(app);
}

void settingsModal(App& app) {
    if (app.keysOpen) {
        keyBindingsModal(app);
        return;
    }
    if (app.creditsOpen) {
        creditsModal(app);
        return;
    }
    Settings& s = app.world.settings;
    dim();
    float SW = ui::SW(), SH = ui::SH();
    float PW = std::min(940.0f, SW - 24), PH = std::min(960.0f, SH - 24);
    Rectangle r{(SW - PW) / 2, (SH - PH) / 2, PW, PH};
    panel(r, Color{20, 26, 36, 252}, 14);
    icon(Icon::Gear, {r.x + 24, r.y + 20, 24, 24}, col::accent);
    text("Settings", r.x + 58, r.y + 18, 24, col::text, true);
    if (iconButton({r.x + r.width - 48, r.y + 16, 32, 32}, Icon::Close) || (IsKeyPressed(KEY_ESCAPE) && !ui::editing())) {
        app.settingsOpen = false;
        savePrefs(app);
        return;
    }
    float colW = (r.width - 72) / 2;
    float lx = r.x + 24, rx = r.x + 48 + colW;
    // returns the chip clicked (-1: none) and moves y below the row
    auto chipRow = [&](float x, float& y, const char* label, const std::vector<std::string>& names, int selected) {
        const float pad = 20, gap = 6, h = 30;
        float total = -gap;
        for (const std::string& n : names) total += measure(n, 14, true).x + pad + gap;
        float labelW = measure(tr(label), 15, false).x;
        text(label, x, y + 6, 15, col::text);
        if (labelW + 16 + total > colW) y += 30; // (no room beside the label)
        float cx = x + colW - total;
        int hit = -1;
        for (int i = 0; i < (int)names.size(); i++) {
            float w = measure(names[i], 14, true).x + pad;
            if (chip({cx, y, w, h}, names[i], i == selected) && i != selected) hit = i;
            cx += w + gap;
        }
        y += 38;
        return hit;
    };
    // the content scrolls when it does not fit (large text sizes, small windows); its height from the last frame
    static float contentH = 0;
    Rectangle area{r.x + 8, r.y + 60, r.width - 16, r.height - 68};
    float scrollOff = beginScroll(7200, area, contentH);
    float top = r.y + 72 - scrollOff;
    // ---- left column: language, text size, fun modes
    float y = top;
    text("LANGUAGE", lx, y, 13, col::dim, true);
    y += 24;
    {
        float cx = lx;
        for (int l = 0; l < i18n::COUNT; l++) {
            Rectangle c{cx, y, 118, 34};
            if (chip(c, "", app.lang == l) && app.lang != l) {
                app.lang = l;
                setLanguage(l);
                savePrefs(app);
            }
            textNative(i18n::langName(l), c, 15, app.lang == l ? col::accent : col::text);
            cx += 126;
        }
    }
    y += 52;
    text("TEXT SIZE", lx, y, 13, col::dim, true);
    y += 24;
    {
        float cx = lx;
        for (int i = 0; i < 4; i++) {
            char pt[32];
            std::snprintf(pt, sizeof pt, " (%.0f pt)", TEXT_POINTS[i]);
            std::string label = tr(TEXT_SCALE_NAMES[i]) + pt;
            float w = measure(label, 14, true).x + 22;
            if (cx + w > lx + colW) { cx = lx; y += 38; }
            if (chip({cx, y, w, 32}, label, app.textSize == i)) {
                app.textSize = i;
                setTextScale(TEXT_SCALES[i]);
                savePrefs(app);
            }
            cx += w + 8;
        }
    }
    y += 50;
    text("GUI SIZE", lx, y, 13, col::dim, true);
    y += 24;
    {
        float cx = lx;
        for (int i = 0; i < 6; i++) {
            std::string label = std::to_string((int)std::lround(UI_SIZES[i] * 100)) + " %";
            float w = measure(label, 14, true).x + 22;
            if (cx + w > lx + colW) { cx = lx; y += 38; }
            if (chip({cx, y, w, 32}, label, app.uiSize == i)) {
                app.uiSize = i;
                ui::setUserScale(UI_SIZES[i]);
                savePrefs(app);
            }
            cx += w + 8;
        }
    }
    y = wrap("The size of the whole interface (panels, buttons, text) on top of the display scaling. A large GUI is limited by the window size.",
             lx, y + 38, colW, 13, col::faint) + 12;
    if (toggle({lx, y, colW, 32}, "Reverse zoom direction", app.reverseZoom)) {
        nav::setReverseZoom(app.reverseZoom);
        savePrefs(app);
    }
    y += 44;
    text("FUN MODES", lx, y, 13, col::dim, true);
    y += 24;
    y = wrap("Achievements are not earned by vessels flown with these modes.", lx, y, colW, 13, col::faint) + 8;
    toggle({lx, y, colW, 34}, "Infinite fuel and electricity", s.infiniteFuel);
    y = wrap("Propellant and batteries are never consumed. Physics is otherwise unchanged.", lx, y + 38, colW, 13, col::faint) + 10;
    toggle({lx, y, colW, 34}, "Hyperspace travel", s.hyperspace);
    y = wrap("Vessels with a Hyperspace Drive can jump to any body (key Y). Fictional.", lx, y + 38, colW, 13, col::faint) + 12;
    text("Hyperspace speed", lx, y, 15, col::text);
    y += 28;
    {
        float cx = lx;
        for (int i = 0; i < HYPER_SPEED_COUNT; i++) {
            float w = measure(HYPER_SPEED_NAMES[i], 14, true).x + 26;
            if (chip({cx, y, w, 30}, HYPER_SPEED_NAMES[i], s.hyperSpeed == i)) s.hyperSpeed = i;
            cx += w + 8;
        }
    }
    y += 48;
    // ---- display: frame-rate limit, overlay, adaptive detail
    text("DISPLAY", lx, y, 13, col::dim, true);
    y += 24;
    {
        std::vector<std::string> names;
        for (int i = 0; i < 4; i++) names.push_back(FPS_LIMITS[i] ? std::to_string(FPS_LIMITS[i]) : tr("Display"));
        int hit = chipRow(lx, y, "Frame rate limit", names, app.fpsLimit);
        if (hit >= 0) {
            app.fpsLimit = hit;
            savePrefs(app);
        }
        y += 2;
    }
    if (toggle({lx, y, colW, 32}, "Show frame rate and GPU load", app.showFps)) savePrefs(app); // full debug screen: key binding
    y += 38;
    {
        // near-ground terrain: mesh density and the finest relief (Ultra: about a metre)
        std::vector<std::string> names{tr("Low"), tr("Medium"), tr("High"), tr("Ultra")};
        int hit = chipRow(lx, y, "Terrain detail", names, app.terrainQuality);
        if (hit >= 0) {
            app.terrainQuality = hit;
            gfx::setTerrainQuality(hit);
            savePrefs(app);
        }
        y += 2;
    }
    {
        // how far the detailed ground reaches: 50 % .. 400 % of the standard range (farther costs frame time)
        text("Detail range", lx, y + 6, 15, col::text);
        const float fieldW = 66;
        double val = app.detailRange;
        bool done = false;
        if (slider({lx + 150, y, colW - 150 - fieldW - 14, 30}, val, 50, 400, 5, 7111, &done)) {
            app.detailRange = (int)std::lround(val);
            gfx::setDetailRange(app.detailRange / 100.0f);
        }
        double typed = app.detailRange;
        if (numberField({lx + colW - fieldW, y + 1, fieldW, 28}, typed, 0, "%", 7112, 50, 400)) {
            app.detailRange = (int)std::lround(typed);
            gfx::setDetailRange(app.detailRange / 100.0f);
            done = true;
        }
        if (done) savePrefs(app);
        y += 38;
    }
    if (toggle({lx, y, colW, 32}, "Adaptive detail (keeps the frame rate)", app.adaptiveDetail)) {
        if (!app.adaptiveDetail) gfx::setPlanetDetail(16);
        savePrefs(app);
    }
    float leftEnd = wrap("Lowers the finest planet surface detail only while the GPU cannot reach the frame-rate limit.", lx, y + 36, colW, 13, col::faint);
    if (toggle({lx, leftEnd + 10, colW, 32}, "Asteroid belt and Kuiper belt on the map", app.showBelts)) savePrefs(app);
    leftEnd += 46;
    {
        bool fs = app.fullscreen;
        if (toggle({lx, leftEnd + 10, colW, 32}, trf("Full screen ({0})", keys::label(keys::A::Fullscreen)), fs)) setFullscreen(app, fs);
        leftEnd += 46;
    }
    // ---- right column: launch site
    y = top;
    text("LAUNCH SITE", rx, y, 13, col::dim, true);
    y += 24;
    for (int i = 0; i < LAUNCH_SITE_COUNT; i++) {
        Rectangle row{rx - 6, y, colW + 6, 30};
        if (rowItem(row, s.launchSite == i && s.launchBase <= 0)) {
            s.launchSite = i;
            s.launchBase = 0;
        }
        char b[96];
        std::snprintf(b, sizeof b, "%.2f°, %.2f°", LAUNCH_SITES[i].latDeg, LAUNCH_SITES[i].lonDeg);
        text(fit(LAUNCH_SITES[i].name, colW - 130, 15), row.x + 14, row.y + 6, 15, col::text);
        textR(b, row.x + row.width - 12, row.y + 7, 13, col::dim);
        y += 32;
    }
    // the surface bases of this map that have a launch pad
    for (int id : app.world.launchBaseIds()) {
        const SurfaceBase& sb = app.world.bases[findBase(app.world, id)];
        Rectangle row{rx - 6, y, colW + 6, 30};
        if (rowItem(row, s.launchBase == id)) s.launchBase = id;
        icon(Icon::Base, {row.x + 12, row.y + 6, 18, 18}, col::accent);
        text(fit(sb.name, colW - 150, 15), row.x + 38, row.y + 6, 15, col::text);
        textR(tr(app.world.U.bodies[sb.body].name), row.x + row.width - 12, row.y + 7, 13, col::dim);
        y += 32;
    }
    y = wrap("Lower latitudes gain more of Earth's 465 m/s equatorial rotation speed.", rx, y + 10, colW, 13, col::faint) + 22;
    // ---- sound: effects, and the music (its own switch and volume)
    text("SOUND", rx, y, 13, col::dim, true);
    y += 24;
    {
        // a slider in 1 % steps (drag, wheel or arrow keys) and the exact value to type in; the music has its
        // own on / off switch
        const float fieldW = 66, swW = 44, labelW = 118;
        auto volumeRow = [&](const char* label, int& pct, bool* on, int id) {
            text(label, rx, y + 6, 15, col::text);
            float sx = rx + labelW, ex = rx + colW - fieldW - 10;
            if (on) {
                bool v = *on;
                Rectangle sw{sx, y + 4, swW, 22};
                // the switch alone (toggle() draws its label to the left of the knob)
                if (toggle(sw, "", v)) {
                    *on = v;
                    sfx::setMusic(app.music, app.musicPct / 100.0f);
                    savePrefs(app);
                }
                sx += swW + 12;
            }
            double val = pct;
            bool done = false;
            bool en = !on || *on;
            if (slider({sx + 6, y, ex - sx - 12, 30}, val, 0, 100, 1, id, &done, en)) {
                pct = (int)std::lround(val);
                if (on) sfx::setMusic(app.music, pct / 100.0f);
                else sfx::setVolume(pct / 100.0f);
            }
            double typed = pct;
            if (numberField({rx + colW - fieldW, y + 1, fieldW, 28}, typed, 0, "%", id + 1, 0, 100)) {
                pct = (int)std::lround(typed);
                if (on) sfx::setMusic(app.music, pct / 100.0f);
                else sfx::setVolume(pct / 100.0f);
                done = true;
            }
            if (done) savePrefs(app);
            y += 38;
        };
        volumeRow("Sound effects", app.volumePct, nullptr, 7101);
        volumeRow("Music", app.musicPct, &app.music, 7103);
        y = wrap("Music plays now and then, with quiet pauses between the tracks.", rx, y, colW, 13, col::faint);
    }
    y += 20;
    // ---- memory
    text("MEMORY", rx, y, 13, col::dim, true);
    y += 24;
    const mem::Status& ms = mem::status();
    float frac = (float)((double)ms.process / mem::BUDGET);
    bar({rx, y, colW, 8}, std::min(1.0f, frac), ms.pressure == mem::Hard ? col::bad : ms.pressure == mem::Soft ? col::warn : col::good);
    y += 16;
    text(trf("{0} MB used of {1} MB", std::to_string(ms.process / mem::MB), std::to_string(mem::BUDGET / mem::MB)), rx, y, 14, col::text);
    y += 22;
    text(trf("Aerodynamics cache: {0} MB  ·  system free: {1} MB", std::to_string(aeroCacheBytes() / mem::MB), std::to_string(ms.sysAvail / mem::MB)),
         rx, y, 13, col::dim);
    y += 20;
    text(trf("Planet surface cache (GPU): {0} MB", std::to_string(gfx::bakedFieldBytes() / mem::MB)), rx, y, 13, col::dim);
    y += 20;
    y = wrap("Caches are released automatically before the limit is reached, so the game does not slow down from paging.", rx, y, colW, 13, col::faint) + 22;
    // ---- controls
    text("CONTROLS", rx, y, 13, col::dim, true);
    y += 24;
    if (button({rx, y, colW, 38}, "Key bindings", false, true, Icon::Keyboard)) app.keysOpen = true;
    y += 46;
    if (button({rx, y, colW, 38}, "Open screenshot folder", false, true, Icon::Folder)) {
        std::error_code ec;
        std::filesystem::path d = screenshot::folder(app.dataDir);
        std::filesystem::create_directories(d, ec);
        winshell::openFolder(d);
    }
    y += 44;
    y = wrap(trf("Screenshots ({0}) are saved as PNG files.", keys::label(keys::A::Screenshot)), rx, y, colW, 13, col::faint) + 20;
    // ---- saving: the autosave interval of the map being played
    text("SAVING", rx, y, 13, col::dim, true);
    y += 24;
    {
        static const int MIN[] = {0, 1, 3, 5, 10};
        std::vector<std::string> names{tr("Off")};
        int sel = 0;
        for (int i = 1; i < 5; i++) names.push_back(trf("{0} min", std::to_string(MIN[i])));
        for (int i = 0; i < 5; i++)
            if (MIN[i] == app.autosaveMin) sel = i;
        int hit = chipRow(rx, y, "Autosave", names, sel);
        if (hit >= 0) {
            app.autosaveMin = MIN[hit];
            savePrefs(app);
        }
        y = wrap("The map is also saved when you go to the home screen or the factory, and you are asked before quitting with unsaved changes.",
                 rx, y - 4, colW, 13, col::faint) + 20;
    }
    // ---- rendering: physically based shading, off by default (heavy)
    text("RENDERING", rx, y, 13, col::dim, true);
    y += 24;
    {
        bool on = app.physicalShading;
        if (toggle({rx, y, colW, 32}, "Physically based shading (heavy)", on)) {
            app.physicalShading = gfx::setPhysicalShading(on) && on;
            if (on && !app.physicalShading) app.setStatus(tr("Physically based shading is not available on this graphics driver"));
            savePrefs(app);
        }
        y = wrap("GGX materials with Fresnel reflection, a sky computed from Rayleigh / Mie scattering, aerial perspective and sunlight reddened by the air. Needs a strong GPU.",
                 rx, y + 36, colW, 13, col::faint);
        // detail of the physically based mode: realistic shadows (shadow maps)
        if (app.physicalShading) {
            y += 10;
            bool sh = app.realShadows;
            if (toggle({rx, y, colW, 32}, "Realistic shadows (heavy)", sh)) {
                app.realShadows = gfx::setShadows(sh, app.shadowQuality) && sh;
                if (sh && !app.realShadows) app.setStatus(tr("Realistic shadows are not available on this graphics driver"));
                savePrefs(app);
            }
            y = wrap("Shadows of the vessels, the pad and the bases cast by the sun, on the ground and on each other. Their edges are as soft as the sun's disc makes them: sharp where a part touches the ground, blurred far from it.",
                     rx, y + 36, colW, 13, col::faint) + 8;
            if (app.realShadows) {
                int hit = chipRow(rx, y, "Shadow resolution", {"2048", "4096"}, app.shadowQuality);
                if (hit >= 0) {
                    app.shadowQuality = hit;
                    gfx::setShadows(app.realShadows, hit);
                    savePrefs(app);
                }
            }
        }
    }
    // ---- about: version, data credits and the licences of the libraries
    y += 22;
    text("ABOUT", rx, y, 13, col::dim, true);
    y += 24;
    text(trf("Version {0}", APP_VERSION), rx, y, 14, col::text);
    y += 26;
    if (button({rx, y, colW, 38}, "Credits and licences", false, true, Icon::Book)) app.creditsOpen = true;
    y = wrap("Surface maps and star data from NASA, the USGS, ESA and others, and the open-source libraries Astraum is built with.",
             rx, y + 46, colW, 13, col::faint);
    contentH = std::max(leftEnd, y) + 16 - (area.y - scrollOff);
    endScroll();
}


namespace {

// short notice at the top centre (screenshot saved); fades out after 3 s
} // namespace

void drawNotice(const App& app) {
    double age = GetTime() - app.noticeTime;
    if (app.notice.empty() || age < 0 || age > 3.0) return;
    float a = (float)std::clamp((3.0 - age) / 0.4, 0.0, 1.0);
    Vector2 m = measure(app.notice, 15);
    float w = m.x + 58, SW = ui::SW();
    Rectangle r{(SW - w) / 2, 70, w, 38};
    DrawRectangleRounded(r, 1.0f, 12, Fade(Color{18, 24, 34, 240}, a));
    Color c = app.noticeBad ? col::warn : col::good;
    DrawRectangleRoundedLinesEx(r, 1.0f, 12, 1.0f, Fade(c, a * 0.7f));
    icon(app.noticeBad ? Icon::Info : Icon::Check, {r.x + 14, r.y + 10, 18, 18}, Fade(c, a));
    text(app.notice, r.x + 42, r.y + (r.height - m.y) / 2, 15, Fade(col::text, a));
}

void crashModal(App& app) {
    dim(0.7f);
    float SW = ui::SW(), SH = ui::SH();
    Rectangle r{(SW - 900) / 2, (SH - 640) / 2, 900, 640};
    panel(r, Color{20, 26, 36, 252}, 14);
    icon(Icon::Info, {r.x + 24, r.y + 20, 24, 24}, col::warn);
    text("Astraum closed unexpectedly last time", r.x + 58, r.y + 18, 22, col::text, true);
    float y = wrap("A crash report was saved. Your saved game and designs are not affected. You can copy the report to send it to the developer.",
                   r.x + 24, r.y + 58, r.width - 48, 14, col::dim) + 6;
    text(fit(app.crashPath, r.width - 48, 12), r.x + 24, y, 12, col::faint);
    y += 24;
    Rectangle box{r.x + 24, y, r.width - 48, r.y + r.height - 76 - y};
    DrawRectangleRounded(box, 0.02f, 6, Color{10, 13, 19, 255});
    // the report is plain ASCII; draw only the visible lines
    const float lh = 17;
    int lines = 1;
    for (char ch : app.crashReport) lines += ch == '\n';
    Rectangle inner{box.x + 10, box.y + 8, box.width - 20, box.height - 16};
    float off = beginScroll(9001, inner, lines * lh);
    size_t pos = 0;
    for (int i = 0; i < lines && pos <= app.crashReport.size(); i++) {
        size_t e = app.crashReport.find('\n', pos);
        if (e == std::string::npos) e = app.crashReport.size();
        float ly = inner.y + i * lh - off;
        if (ly > inner.y - lh && ly < inner.y + inner.height) {
            std::string ln = app.crashReport.substr(pos, e - pos);
            if (!ln.empty() && ln.back() == '\r') ln.pop_back();
            textRaw(fitRaw(ln, inner.width - 8, 12), inner.x, ly, 12, Color{190, 200, 215, 255});
        }
        pos = e + 1;
    }
    endScroll();
    float by = r.y + r.height - 60;
    if (button({r.x + 24, by, 200, 40}, "Copy report", false, true, Icon::Copy)) {
        SetClipboardText(app.crashReport.c_str());
        app.setStatus("Crash report copied to the clipboard");
    }
    if (primaryButton({r.x + r.width - 184, by, 160, 40}, "Close", Icon::Close) || IsKeyPressed(KEY_ESCAPE) || IsKeyPressed(KEY_ENTER)) {
        crash::markSeen();
        app.crashReport.clear();
    }
}

namespace {

void drawMenuBackground(App& app) {
    gfx::setNightLift(0.f); // (do not carry flight night-vision brightness back onto the title screen)
    const Universe& U = app.world.U;
    ClearBackground(Color{3, 5, 9, 255});
    gfx::Cam cam;
    cam.fwd = {0, 1, 0};
    cam.up = {0, 0, 1};
    cam.fovy = 40;
    gfx::begin3D(cam, 0.1, 1000);
    gfx::drawStarfield(0.9f);
    gfx::end3D();
    int earth = U.find("Earth"), moon = U.find("Moon");
    double t = GetTime();
    Vec3d sunDir = norm(Vec3d{-0.8, -0.35, 0.35});
    gfx::clearDepth();
    gfx::begin3D(cam, 0.05, 1000);
    Mat3d rotM = rotZ(t * 0.02) * rotX(0.5);
    gfx::drawBodySphere(U, moon, {-3.2, 14, 1.4}, 0.36, rotM, sunDir, 1.0f, false, 8);
    gfx::end3D();
    gfx::clearDepth();
    gfx::begin3D(cam, 0.05, 1000);
    Mat3d rot = rotX(0.41) * rotZ(t * 0.03 + 2.0);
    gfx::drawBodySphere(U, earth, {2.3, 6.4, -0.5}, 1.55, rot, sunDir, 1.0f, true, 10);
    gfx::end3D();
}

void menuFrame_(App& app) {
    drawMenuBackground(app);
    ui::setInputBlocked(app.settingsOpen || !app.crashReport.empty() || tut::menuOpen || app.newMapOpen || app.loadMapOpen);
    float SW = ui::SW(), SH = ui::SH();
    float x = SW * 0.08f, y = SH * 0.22f;
    icon(Icon::Rocket, {x, y + 6, 42, 42}, col::accent);
    text("ASTRAUM", x + 58, y, 52, col::text, true);
    wrap("Design spacecraft. Fly them with real orbital mechanics. Explore the Solar System and the nearest stars.",
         x + 2, y + 70, std::min(SW * 0.6f, 900.0f), 17, col::dim);
    y += 130;
    // the maps on disk (looked up again now and then: cheap, and new saves appear)
    static double mapsChecked = -10;
    static bool hasMaps = false;
    if (GetTime() - mapsChecked > 2.0) {
        mapsChecked = GetTime();
        hasMaps = !listMaps(app).empty();
    }
    if (app.gameStarted) {
        if (primaryButton({x, y, 280, 50}, "Resume", Icon::Play)) {
            app.screen = app.world.activeVessel() ? Screen::Flight : Screen::Factory;
            if (app.screen == Screen::Flight) flightOnEnter(app);
        }
        // which game this is
        std::string mn = !app.mapPath.empty() ? app.world.mapName : (tut::active() ? tr("Tutorial") : std::string());
        if (!mn.empty()) {
            ensureGlyphs(mn);
            text(fit(trf("Map: {0}", mn), 360, 14), x + 296, y + 16, 14, col::dim);
        }
        y += 62;
    }
    if (app.gameStarted ? button({x, y, 280, 50}, "New game", false, true, Icon::Plus)
                        : primaryButton({x, y, 280, 50}, "New game", Icon::Plus))
        app.newMapOpen = true; // (named first: every new game is a map of its own)
    y += 62;
    if (button({x, y, 280, 50}, "Tutorials", false, true, Icon::Book)) tut::menuOpen = true;
    y += 62;
    if (button({x, y, 280, 50}, "Load saved game", false, hasMaps, Icon::Folder)) app.loadMapOpen = true;
    y += 62;
    if (button({x, y, 280, 50}, "CAD Designer", false, true, Icon::Cube)) {
        if (app.craft.comps.empty()) app.craft = builtinCrafts()[0];
        app.screen = Screen::CAD;
    }
    y += 62;
    if (button({x, y, 280, 50}, "Settings", false, true, Icon::Gear)) app.settingsOpen = true;
    y += 62;
    if (button({x, y, 280, 50}, "Quit", false, true, Icon::Exit)) app.quit = true;

    // feature card
    Rectangle card{x, SH - 190, 700, 150};
    panel(card, Color{14, 18, 26, 200}, 12);
    text("SIMULATION", card.x + 18, card.y + 14, 13, col::dim, true);
    const char* lines[] = {
        "N-body gravity on Keplerian rails  ·  sphere-of-influence transitions  ·  time warp to x100,000,000",
        "Rotating atmospheres, drag and entry heating  ·  pressure-dependent engine Isp  ·  staging",
        "Batteries, solar panels (1/r²), RTGs, fission reactors, ion thrusters and solar sails",
        "Solar System (30 bodies) + Alpha Centauri A/B, Proxima Centauri and Barnard's Star",
    };
    for (int i = 0; i < 4; i++) text(fit(lines[i], card.width - 36, 14), card.x + 18, card.y + 40 + i * 24, 14, Color{200, 210, 225, 255});
    if (GetTime() - app.statusTime < 4.0 && !app.status.empty()) text(app.status, x, y + 64, 15, col::warn);
    std::string df = trf("Data folder: {0}", app.dataDir);
    text(df, SW - measure(df, 12).x - 16, SH - 24, 12, col::faint);
    ui::setInputBlocked(false);
    if (!app.crashReport.empty()) crashModal(app);
    else if (app.settingsOpen) settingsModal(app);
    else if (tut::menuOpen) tut::menuModal(app);
    else if (app.newMapOpen) newMapModal(app);
    else if (app.loadMapOpen) loadMapModal(app);
    mapDialogsClosed(app);
}

} // namespace

void menuFrame(App& app) { menuFrame_(app); }
