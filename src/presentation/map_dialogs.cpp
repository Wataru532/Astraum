// The dialogs around the maps (new map, load map with the pictures, quit with unsaved changes) and the map's
// picture: a copy of the flight view without the HUD, written as a small PNG next to the save
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include "app.hpp"
#include "rl.hpp"
#include "screens.hpp"
#include "session.hpp"
#include "storage.hpp"
#include "tutorial.hpp"
#include "ui.hpp"

using namespace ui;

int g_testCloseChoice = -1;

namespace {
std::string g_newName;     // the name typed in the new-map dialog
int g_loadSel = -1;        // the selected row in the load dialog
bool g_deleteArmed = false; // the delete button was clicked once
std::vector<MapInfo> g_list;
bool g_saveFailed = false; // the quit dialog: the save did not work
// rename in the load dialog
bool g_renaming = false;
std::string g_renameBuf;
// the map's picture: a copy of the flight view without the HUD (taken every half second, a GPU-side blit),
// made into a small PNG next to the save whenever the map is saved (it replaces the previous one)
RenderTexture2D g_grab{};
bool g_haveGrab = false;
double g_grabT = -10;
std::string g_thumbPending;
std::map<std::string, Texture2D> g_thumbTex; // the load dialog's pictures (while it is open)
void unloadThumbs() {
    for (auto& e : g_thumbTex)
        if (e.second.id) UnloadTexture(e.second);
    g_thumbTex.clear();
}
// the grabbed view -> a 400 px wide PNG (main thread, outside the frame's drawing)
void writeThumb(const std::string& mapPath) {
    if (!g_haveGrab || g_grab.id == 0) return;
    int w = g_grab.texture.width, h = g_grab.texture.height;
    int tw = 400, th = std::max(1, (int)std::lround(400.0 * h / std::max(w, 1)));
    RenderTexture2D small = LoadRenderTexture(tw, th);
    if (small.id == 0) return;
    GenTextureMipmaps(&g_grab.texture); // (a smooth reduction: the mip chain of the full view)
    SetTextureFilter(g_grab.texture, TEXTURE_FILTER_TRILINEAR);
    BeginTextureMode(small);
    ClearBackground(BLACK);
    DrawTexturePro(g_grab.texture, {0, 0, (float)w, (float)-h}, {0, 0, (float)tw, (float)th}, {0, 0}, 0, WHITE);
    EndTextureMode();
    Image img = LoadImageFromTexture(small.texture);
    UnloadRenderTexture(small);
    ImageFlipVertical(&img);
    int size = 0;
    unsigned char* png = ExportImageToMemory(img, ".png", &size);
    UnloadImage(img);
    if (png && size > 0) writeFileAtomic(thumbPath(mapPath), std::string((const char*)png, (size_t)size)); // (replaces the old picture)
    if (png) MemFree(png);
}

} // namespace

void grabWorldView() {
    double now = GetTime();
    if (now - g_grabT < 0.5) return;
    g_grabT = now;
    int w = GetRenderWidth(), h = GetRenderHeight();
    if (w <= 0 || h <= 0) return;
    if (g_grab.id == 0 || g_grab.texture.width != w || g_grab.texture.height != h) {
        if (g_grab.id) UnloadRenderTexture(g_grab);
        g_grab = LoadRenderTexture(w, h);
        if (g_grab.id == 0) return;
    }
    rlDrawRenderBatchActive();
    rlBindFramebuffer(RL_READ_FRAMEBUFFER, 0); // the screen (multisampled: the blit resolves it)
    rlBindFramebuffer(RL_DRAW_FRAMEBUFFER, g_grab.id);
    rlBlitFramebuffer(0, 0, w, h, 0, 0, w, h, 0x00004000); // (colour)
    rlBindFramebuffer(RL_READ_FRAMEBUFFER, 0);
    rlBindFramebuffer(RL_DRAW_FRAMEBUFFER, 0);
    g_haveGrab = true;
}

void forgetWorldView() { g_haveGrab = false; }

void finishThumbs() {
    if (!g_thumbPending.empty()) writeThumb(g_thumbPending);
    g_thumbPending.clear();
    unloadThumbs();
    if (g_grab.id) UnloadRenderTexture(g_grab);
    g_grab = RenderTexture2D{};
    g_haveGrab = false;
}

// the session's view of the pictures and the tutorials (ports::Presentation)
void mapSavedPicture(const std::string& mapPath) {
    if (g_haveGrab) g_thumbPending = mapPath;
}
void serviceMapPictures() {
    if (!g_thumbPending.empty()) {
        writeThumb(g_thumbPending);
        g_thumbPending.clear();
    }
}

// ------------------------------------------------------------------ dialogs

bool g_newWasOpen = false, g_loadWasOpen = false; // (the dialogs' first frame: fresh contents)
void mapDialogsClosed(const App& app) {
    if (!app.newMapOpen) g_newWasOpen = false;
    if (!app.loadMapOpen && g_loadWasOpen) {
        g_loadWasOpen = false;
        g_renaming = false;
        unloadThumbs();
    }
}

void newMapModal(App& app) {
    bool& wasOpen = g_newWasOpen;
    if (!wasOpen) g_newName = trf("Map {0}", std::to_string(listMaps(app).size() + 1));
    wasOpen = app.newMapOpen;
    dim();
    float SW = ui::SW(), SH = ui::SH();
    Rectangle r{(SW - 560) / 2, (SH - 250) / 2, 560, 250};
    panel(r, Color{20, 26, 36, 252}, 14);
    icon(Icon::Plus, {r.x + 24, r.y + 20, 24, 24}, col::accent);
    text("New map", r.x + 58, r.y + 18, 22, col::text, true);
    wrap("A new game is saved as its own map. It is saved automatically from then on.", r.x + 24, r.y + 58, r.width - 48, 13, col::dim);
    text("Map name", r.x + 24, r.y + 100, 13, col::dim, true);
    textInput({r.x + 24, r.y + 122, r.width - 48, 40}, g_newName, 90, 7301);
    bool ok = !trimmed(g_newName).empty();
    bool create = primaryButton({r.x + r.width - 184, r.y + r.height - 60, 160, 40}, "Create", Icon::Check, ok) ||
                  (ok && (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)));
    bool cancel = button({r.x + r.width - 344, r.y + r.height - 60, 148, 40}, "Cancel") ||
                  iconButton({r.x + r.width - 48, r.y + 16, 32, 32}, Icon::Close) || (IsKeyPressed(KEY_ESCAPE) && !ui::keyboardCaptured());
    if (create) {
        cancelEdit();
        app.newMapOpen = false;
        wasOpen = false;
        if (!createMap(app, g_newName)) app.setStatus(tr("Could not save the map"));
    } else if (cancel) {
        cancelEdit();
        app.newMapOpen = false;
        wasOpen = false;
    }
}

void loadMapModal(App& app) {
    bool& wasOpen = g_loadWasOpen;
    if (!wasOpen) {
        g_list = listMaps(app);
        g_loadSel = g_list.empty() ? -1 : 0;
        g_deleteArmed = false;
        g_renaming = false;
        unloadThumbs();
        for (int i = 0; i < (int)g_list.size(); i++) {
            ensureGlyphs(g_list[i].name);
            if (g_list[i].path == app.mapPath) g_loadSel = i;
            std::string tp = thumbPath(g_list[i].path);
            if (FileExists(tp.c_str())) {
                Texture2D t = LoadTexture(tp.c_str());
                if (t.id) {
                    SetTextureFilter(t, TEXTURE_FILTER_BILINEAR);
                    g_thumbTex[g_list[i].path] = t;
                }
            }
        }
    }
    wasOpen = app.loadMapOpen;
    auto closeDialogNow = [&] {
        app.loadMapOpen = false;
        wasOpen = false;
        g_renaming = false;
        unloadThumbs();
    };
    dim();
    float SW = ui::SW(), SH = ui::SH();
    float h = std::min(640.0f, SH - 40);
    Rectangle r{(SW - 760) / 2, (SH - h) / 2, 760, h};
    panel(r, Color{20, 26, 36, 252}, 14);
    icon(Icon::Folder, {r.x + 24, r.y + 20, 24, 24}, col::accent);
    text("Load map", r.x + 58, r.y + 18, 22, col::text, true);
    bool close = iconButton({r.x + r.width - 48, r.y + 16, 32, 32}, Icon::Close) || (IsKeyPressed(KEY_ESCAPE) && !g_renaming);
    Rectangle lr{r.x + 16, r.y + 62, r.width - 32, r.height - 62 - 76};
    const float rowH = 84, thW = 128, thH = 72;
    bool load = false;
    if (g_list.empty()) {
        wrap("No maps yet. Start a new game to create one.", lr.x + 8, lr.y + 12, lr.width - 16, 15, col::dim);
    } else {
        float off = beginScroll(7302, lr, g_list.size() * rowH + 4);
        for (int i = 0; i < (int)g_list.size(); i++) {
            Rectangle row{lr.x, lr.y + i * rowH - off, lr.width - 10, rowH - 4};
            if (row.y + row.height < lr.y || row.y > lr.y + lr.height) continue;
            if (rowItem(row, i == g_loadSel) && !(g_renaming && i == g_loadSel)) {
                static double lastClick = -10;
                static int lastRow = -1;
                if (lastRow == i && GetTime() - lastClick < 0.45) load = true; // double click
                lastClick = GetTime();
                lastRow = i;
                if (g_loadSel != i) { g_deleteArmed = false; g_renaming = false; }
                g_loadSel = i;
            }
            const MapInfo& m = g_list[i];
            // the picture of the last save (left of the name), or an empty frame
            Rectangle tr0{row.x + 8, row.y + (row.height - thH) / 2, thW, thH};
            auto it = g_thumbTex.find(m.path);
            if (it != g_thumbTex.end()) {
                const Texture2D& t = it->second;
                // cover the frame, centred (pictures of another aspect are cropped, not stretched)
                float k = std::max(thW / t.width, thH / t.height), sw = thW / k, sh = thH / k;
                DrawTexturePro(t, {(t.width - sw) / 2, (t.height - sh) / 2, sw, sh}, tr0, {0, 0}, 0, WHITE);
            } else {
                DrawRectangleRec(tr0, Color{12, 16, 24, 255});
                icon(Icon::Planet, {tr0.x + thW / 2 - 14, tr0.y + thH / 2 - 14, 28, 28}, col::faint);
            }
            DrawRectangleLinesEx(tr0, 1, col::line);
            float tx = tr0.x + thW + 14, tw = row.width - (tx - row.x) - 90;
            if (g_renaming && i == g_loadSel) {
                textInput({tx, row.y + 10, tw, 32}, g_renameBuf, 90, 7303);
                if (IsKeyPressed(KEY_ESCAPE)) g_renaming = false;
            } else text(fit(m.name, tw, 16, true), tx, row.y + 16, 16, col::text, true);
            text(trf("Saved {0}  ·  {1} vessels", localTimeText(m.savedAt), std::to_string(m.vessels)), tx, row.y + 48, 13, col::dim);
            if (m.path == app.mapPath && app.gameStarted) textR(tr("current"), row.x + row.width - 14, row.y + 33, 13, col::accent, true);
        }
        endScroll();
    }
    float by = r.y + r.height - 60;
    bool sel = g_loadSel >= 0 && g_loadSel < (int)g_list.size();
    if (button({r.x + 24, by, 220, 40}, g_deleteArmed ? "Click again to delete" : "Delete", g_deleteArmed, sel, Icon::Trash)) {
        if (!g_deleteArmed) g_deleteArmed = true;
        else {
            deleteMap(app, g_list[g_loadSel].path);
            g_list = listMaps(app);
            g_loadSel = g_list.empty() ? -1 : 0;
            g_deleteArmed = false;
        }
    }
    // rename: the selected map's name becomes a text field; Enter or the button again applies it
    bool doRename = false;
    if (button({r.x + 252, by, 150, 40}, g_renaming ? "Apply" : "Rename", g_renaming, sel, g_renaming ? Icon::Check : Icon::Pencil)) {
        if (!g_renaming) {
            g_renaming = true;
            g_renameBuf = g_list[g_loadSel].name;
            ui::testTextInput(7303, {}, {}, false, false); // (the field takes the focus at once)
        } else doRename = true;
    }
    if (g_renaming && sel && (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER))) doRename = true;
    if (doRename && sel) {
        if (!trimmed(g_renameBuf).empty() && renameMap(app, g_list[g_loadSel].path, g_renameBuf)) {
            std::string path = g_list[g_loadSel].path;
            g_list[g_loadSel].name = trimmed(g_renameBuf).substr(0, 120);
            ensureGlyphs(g_list[g_loadSel].name);
        }
        g_renaming = false;
        cancelEdit();
    }
    if (button({r.x + r.width - 344, by, 148, 40}, "Cancel")) close = true;
    if (primaryButton({r.x + r.width - 184, by, 160, 40}, "Load", Icon::Folder, sel && !g_renaming) || (sel && load && !g_renaming)) {
        std::string path = g_list[g_loadSel].path;
        closeDialogNow();
        if (loadMap(app, path)) {
            app.screen = app.world.activeVessel() ? Screen::Flight : Screen::Factory;
            if (app.screen == Screen::Flight) flightOnEnter(app);
        }
        return;
    }
    if (close) closeDialogNow();
}

bool debugRenameMap(App& app, const std::string& name) {
    // (tests: the dialog's rename of the selected row)
    if (g_loadSel < 0 || g_loadSel >= (int)g_list.size()) return false;
    g_renaming = true;
    g_renameBuf = name;
    return true;
}

std::string debugNewMapName() { return g_newName; }

int closeDialog(App& app) {
    dim(0.6f);
    float SW = ui::SW(), SH = ui::SH();
    // the buttons as wide as their labels (German is long), the dialog as wide as the buttons
    float wC = std::max(130.0f, measure(tr("Cancel"), 15).x + 40), wQ = std::max(190.0f, measure(tr("Quit without saving"), 15).x + 64),
          wS = std::max(180.0f, measure(tr("Save and quit"), 15, true).x + 64);
    float W = std::clamp(24 + wC + 12 + wQ + 12 + wS + 24, 620.0f, SW - 40);
    Rectangle r{(SW - W) / 2, (SH - 240) / 2, W, 240};
    panel(r, Color{20, 26, 36, 252}, 14);
    icon(Icon::Save, {r.x + 24, r.y + 20, 24, 24}, col::warn);
    text("Unsaved changes", r.x + 58, r.y + 18, 22, col::text, true);
    ensureGlyphs(app.world.mapName);
    float y = wrap(trf("The map \"{0}\" has changes that have not been saved yet. Save them before quitting?", app.world.mapName), r.x + 24,
                   r.y + 62, r.width - 48, 15, col::text) + 8;
    if (g_saveFailed) wrap("Could not save the map", r.x + 24, y, r.width - 48, 14, col::bad);
    float by = r.y + r.height - 60;
    int choice = -1;
    if (button({r.x + 24, by, wC, 40}, "Cancel") || IsKeyPressed(KEY_ESCAPE)) choice = 2;
    if (button({r.x + r.width - 24 - wS - 12 - wQ, by, wQ, 40}, "Quit without saving", false, true, Icon::Exit)) choice = 1;
    if (primaryButton({r.x + r.width - 24 - wS, by, wS, 40}, "Save and quit", Icon::Save)) choice = 0;
    if (g_testCloseChoice >= 0) {
        choice = g_testCloseChoice;
        g_testCloseChoice = -1;
    }
    if (choice == 0 && !saveMap(app, false, true)) { // (written before the window closes)
        g_saveFailed = true;
        return -1;
    }
    if (choice >= 0) g_saveFailed = false;
    return choice;
}
