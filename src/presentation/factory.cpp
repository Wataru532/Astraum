// Engineering factory: browse parts, assemble a stack, inspect stats, launch
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include "app.hpp"
#include "screens.hpp"
#include "session.hpp"
#include "storage.hpp"
#include "rl.hpp"
#include "render.hpp"
#include "nav.hpp"
#include "ui.hpp"
#include "keys.hpp"
#include "tutorial.hpp"

using namespace ui;

namespace {
int selected = -1;
std::vector<int> multiSel;   // additional selected stack parts (box / Ctrl / Shift selection)
bool fBoxing = false;
Vector2 fBoxStart{};
int fPress = -1;
bool isMulti(int i) { return std::find(multiSel.begin(), multiSel.end(), i) != multiSel.end(); }
int catSel = 0; // 0 = all, otherwise Cat+1
nav::OrbitCam fcam;
bool fcamInit = false;
bool loadOpen = false;
int hoverPart = -1;
std::vector<SavedFile> savedFiles;
bool confirmNew = false;
void refreshSaved(App& app);
// saving under a name that a saved design already has: ask before replacing it
bool confirmOverwrite = false;
std::string overwritePath, overwriteName; // (the name the saved design has)
int g_testOverwrite = -1; // (tests: the choice to make in the dialog)
// Save asks for the name first (the overwrite question follows when a saved design has it)
bool saveAsOpen = false;
std::string saveAsName;
void openSaveAs(const std::string& name) {
    saveAsOpen = true;
    saveAsName = name;
    ui::focusText(1002, saveAsName);
}

std::string designPath(App& app, const std::string& name) { return app.designDir + "\\" + sanitizeFileName(name) + ".sfd"; }
void saveDesignTo(App& app, const Design& D, const std::string& path) {
    app.setStatus(saveDesign(D, path) ? trf("Saved design to {0}", path) : tr("Could not save design"));
    refreshSaved(app);
}
// Save: straight away for a new name, through the overwrite dialog when that design file exists already
void requestSave(App& app) {
    std::string path = designPath(app, app.design.name);
    std::error_code ec;
    if (std::filesystem::exists(path, ec)) {
        confirmOverwrite = true;
        overwritePath = path;
        overwriteName = savedDesignName(path);
        if (overwriteName.empty()) overwriteName = app.design.name;
    } else saveDesignTo(app, app.design, path);
}

std::string num(double v, int prec = 2) {
    char b[48];
    std::snprintf(b, sizeof b, "%.*f", prec, v);
    return b;
}
std::string thousands(double v) {
    char b[48];
    long long x = (long long)std::llround(v);
    std::snprintf(b, sizeof b, "%lld", x);
    std::string s = b, o;
    int n = 0;
    for (int i = (int)s.size() - 1; i >= 0; i--) {
        o.insert(o.begin(), s[i]);
        if (++n % 3 == 0 && i > 0 && s[i - 1] != '-') o.insert(o.begin(), ',');
    }
    return o;
}
std::string massStr(double kg) { return kg >= 10000 ? num(kg / 1000.0, 1) + " t" : thousands(kg) + " kg"; }
std::string powerStr(double w) {
    if (w >= 1e6) return num(w / 1e6, 2) + " MW";
    if (w >= 1e3) return num(w / 1e3, 1) + " kW";
    return num(w, 0) + " W";
}

std::string partSubtitle(const PartDef& d) {
    char b[160];
    switch (d.kind) {
    case PK::Engine:
        if (d.powerUse > 0)
            std::snprintf(b, sizeof b, "%.0f mN  ·  Isp %.0f s  ·  %s", d.thrustVac * 1000, d.ispVac, powerStr(d.powerUse).c_str());
        else std::snprintf(b, sizeof b, "%.0f kN  ·  Isp %.0f / %.0f s  ·  %.0f kg", d.thrustVac / 1000, d.ispSL, d.ispVac, d.dryMass);
        return b;
    case PK::SRB:
        std::snprintf(b, sizeof b, "%.0f kN  ·  Isp %.0f / %.0f s  ·  ", d.thrustVac / 1000, d.ispSL, d.ispVac);
        return b + trf("{0} propellant", massStr(d.srbProp));
    case PK::Tank: return tr(propName(d.prop)) + "  ·  " + tr("size adjustable");
    case PK::Capsule:
    case PK::Probe:
        if (d.diamAdj) return tr("Avionics  ·  diameter adjustable");
        return trf("{0}  ·  {1} crew  ·  {2} N m", massStr(d.dryMass), std::to_string(d.crew), num(d.torque, 0));
    case PK::Battery: return trf("{0} kWh  ·  radial  ·  {1}", num(d.batteryJ / 3.6e6, 0), massStr(d.dryMass));
    case PK::Solar: return trf("{0} @ 1 AU  ·  radial  ·  {1}", powerStr(d.solarArea * d.solarEff * 1361), massStr(d.dryMass));
    case PK::RTG: return trf("{0} constant  ·  radial  ·  {1}", powerStr(d.genW), massStr(d.dryMass));
    case PK::Reactor: return trf("{0} electric  ·  {1}", powerStr(d.genW), massStr(d.dryMass));
    case PK::CMG: return num(d.torque, 0) + " N m  ·  " + powerStr(d.drawW) + "  ·  " + massStr(d.dryMass);
    case PK::Sail: return thousands(d.sailArea) + " m²  ·  " + massStr(d.dryMass);
    case PK::Legs: return trf("radial  ·  {0} each", massStr(d.dryMass));
    case PK::RCS: return trf("4 × {0} N  ·  radial  ·  {1} each", num(d.thrustVac, 0), massStr(d.dryMass));
    case PK::Parachute: return "Cd·A " + num(d.chuteCdA, 0) + " m²  ·  " + massStr(d.dryMass);
    case PK::Hyperdrive: return trf("Fictional FTL  ·  {0}", massStr(d.dryMass));
    default: return d.diamAdj ? tr("Diameter adjustable") : massStr(d.dryMass);
    }
}

std::string sizeTag(const PartInst& p) {
    const PartDef& d = CATALOG.defs[p.def];
    std::string s;
    if (d.kind == PK::Adapter) return "Ø" + num(p.diam, 2) + " → " + num(p.diam2, 2);
    if (d.diamAdj) s = "Ø" + num(p.diam, 2);
    if (d.lenAdj) s += " × " + num(p.length, 1) + " m";
    if (d.clusterAdj && p.cluster > 1) s += "  ×" + std::to_string(p.cluster);
    return s;
}

int defaultRadialCount(const PartDef& d) {
    switch (d.kind) {
    case PK::Legs: return 4;
    case PK::Solar: return 2;
    case PK::SRB: return 2;
    default: return 1;
    }
}

bool placeSide = false; // parts list: add to the side of the selected part instead of below it

void addPart(App& app, int def) {
    const PartDef& d = CATALOG.defs[def];
    Design& D = app.design;
    if (placeSide && !sideMountable(d)) {
        app.setStatus(trf("{0} can only be stacked", tr(d.name)));
        return;
    }
    if (d.radial || (placeSide && d.kind == PK::SRB)) {
        if (selected < 0 || selected >= (int)D.stack.size()) {
            app.setStatus(trf("Select a stack part first to attach {0}", tr(d.name)));
            return;
        }
        auto& rl = D.stack[selected].radial;
        for (auto& r : rl)
            if (r.def == def) { r.count = std::min(d.kind == PK::SRB ? 8 : 12, r.count + defaultRadialCount(d)); return; }
        if (rl.size() >= 8) { app.setStatus("Too many radial attachment groups on this part"); return; }
        rl.push_back({def, defaultRadialCount(d)});
        return;
    }
    if (D.stack.size() >= 60) { app.setStatus("Part limit reached (60 stack parts)"); return; }
    PartInst p = makeInst(def);
    // inherit the diameter of the neighbour above
    if (d.diamAdj && selected >= 0 && selected < (int)D.stack.size()) {
        const PartInst& above = D.stack[selected];
        PartGeom g = partGeom(above);
        double dia = std::max(0.5, g.rBot * 2);
        p.diam = p.diam2 = std::round(dia * 100) / 100;
        if (d.kind == PK::Engine) p.diam = std::max(p.diam, d.exitDiam + 0.1);
    }
    int at = (selected >= 0 && selected < (int)D.stack.size()) ? selected + 1 : (int)D.stack.size();
    D.stack.insert(D.stack.begin() + at, p);
    selected = at;
}

void refreshSaved(App& app) {
    savedFiles = listSavedDesigns(app.designDir, ".sfd", 200);
    for (auto& s : savedFiles) ensureGlyphs(s.name); // (names in another script than the interface language)
}

// a value with - / + (held: repeats; Shift: x5) around a field: click it to type a value, or drag across it sideways
bool stepper(float x, float y, float w, const std::string& label, double& v, double step, double lo, double hi, int prec, const char* unit) {
    text(fit(label, w - 160, 15), x, y + 6, 15, col::dim);
    // (the id from the label and the row: a field keeps its typing focus while it stays where it is)
    unsigned hsh = 2166136261u;
    for (char c : label) hsh = (hsh ^ (unsigned char)c) * 16777619u;
    int id = 60000 + (int)((hsh ^ (unsigned)(int)(y * 7 + x)) % 30000u);
    return spinField({x + w - 150, y, 150, 28}, v, step, lo, hi, prec, unit, id);
}

void drawScene(App& app, Rectangle view, const DesignStats& st) {
    double h = std::max(st.height, 2.0);
    double legs = 0;
    for (auto& p : app.design.stack)
        for (auto& r : p.radial)
            if (CATALOG.defs[r.def].kind == PK::Legs) legs = 1.7;
    double platform = 1.0;
    double baseZ = platform + legs;
    Vec3d centre{0, 0, baseZ + h * 0.45};
    double radius = std::max(h * 0.6, st.maxDiam);
    if (!fcamInit) {
        fcamInit = true;
        fcam.fovy = 45;
        fcam.minDist = 2;
        fcam.maxDist = 3000;
        fcam.homeYaw = -PI / 2 + 0.7;
        fcam.homePitch = 0.25;
        fcam.yaw = fcam.homeYaw;
        fcam.pitch = fcam.homePitch;
        fcam.target = centre;
        fcam.dist = std::max(h * 1.5, 14.0);
    }
    fcam.update(GetFrameTime());
    Vector2 m = GetMousePosition();
    bool modal = loadOpen || app.settingsOpen || confirmNew || confirmOverwrite || saveAsOpen;
    bool inView = CheckCollisionPointRec(m, view) && !mouseOverUI() && !modal;
    // ray picking of stack parts (also used as the Fusion orbit pivot)
    auto pickPart = [&](const Vec3d& o, const Vec3d& d, int& part, Vec3d& hit) {
        Ray ray{toF(o), toF(d)};
        double z = baseZ;
        part = -1;
        float bestD = 1e30f;
        for (int i = (int)app.design.stack.size() - 1; i >= 0; i--) {
            PartGeom g = partGeom(app.design.stack[i]);
            float r = (float)std::max(g.rTop, g.rBot);
            if (app.design.stack[i].hidden) { // (hidden: seen through, never picked)
                z += g.height;
                continue;
            }
            BoundingBox bb{{-r, -r, (float)z}, {r, r, (float)(z + g.height)}};
            RayCollision rc = GetRayCollisionBox(ray, bb);
            if (rc.hit && rc.distance < bestD) { bestD = rc.distance; part = i; hit = o + d * rc.distance; }
            z += g.height;
        }
        return part >= 0;
    };
    auto pivotPick = [&](const Vec3d& o, const Vec3d& d, Vec3d& hit) { int pi; return pickPart(o, d, pi, hit); };
    auto fitFn = [&]() { fcam.fit(centre, radius); };
    auto homeFn = [&]() { fcam.home(centre, radius); };
    bool cubeHover = !modal && CheckCollisionPointRec(m, nav::viewCubeRect(view));
    bool navigating = nav::handleMouse(fcam, view, inView && !cubeHover, pivotPick, fitFn);

    // backdrop gradient
    DrawRectangleGradientV((int)view.x, (int)view.y, (int)view.width, (int)view.height, Color{24, 32, 46, 255}, Color{9, 11, 16, 255});
    nav::beginViewport3D(fcam, view, 0.1, 3000);
    gfx::resetEnvironment(); // neutral studio light (the flight view leaves its sky, sunlight tint and flood lights)
    gfx::setSpotLights(0, nullptr, nullptr, {}, 1);

    // floor & hangar
    Color floorC{30, 34, 42, 255};
    DrawCylinderEx({0, 0, -0.05f}, {0, 0, 0}, 80, 80, 64, floorC);
    for (int i = -40; i <= 40; i += 4) {
        DrawLine3D({(float)i, -40, 0.01f}, {(float)i, 40, 0.01f}, Color{48, 56, 70, 255});
        DrawLine3D({-40, (float)i, 0.01f}, {40, (float)i, 0.01f}, Color{48, 56, 70, 255});
    }
    float pr = (float)std::max(st.maxDiam * 0.9, 3.0);
    DrawCylinderEx({0, 0, 0}, {0, 0, (float)platform}, pr, pr, 48, Color{58, 62, 72, 255});
    DrawCylinderWiresEx({0, 0, (float)platform}, {0, 0, (float)platform + 0.02f}, pr, pr, 48, Color{250, 190, 60, 255});
    float tx = pr + 3.0f;
    float th = (float)(h + baseZ + 4);
    DrawCubeWires({tx, 0, th / 2}, 2.2f, 2.2f, th, Color{180, 70, 50, 255});
    for (float z = 2; z < th; z += 2.2f) DrawCube({tx, 0, z}, 2.2f, 2.2f, 0.12f, Color{150, 60, 45, 255});
    rlDrawRenderBatchActive();

    gfx::VesselVisual vv;
    vv.design = &app.design;
    vv.highlight = selected;
    std::vector<char> mask(app.design.stack.size(), 0);
    for (int i : multiSel)
        if (i >= 0 && i < (int)mask.size()) mask[i] = 1;
    vv.selMask = &mask;
    vv.ghostHidden = true;
    vv.legs = true;
    int segN = 0;
    auto seg = segmentOfParts(app.design, segN);
    vv.seg = &seg;
    vv.bottomSeg = -1;
    gfx::drawVessel(vv, Mat3d{}, {0, 0, baseZ}, 0, norm(Vec3d{0.45, -0.55, 0.7}), 1.0f, nav::renderEye(fcam), 0.28f);
    nav::endViewport3D();
    if (!modal) nav::viewCube(fcam, view, homeFn);

    // selection: click, Ctrl/Shift+click to add, left drag = box selection (window / crossing like Fusion)
    bool addMod = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT) || IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
    if (inView && !cubeHover && !navigating && fcam.tool == 0 && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vec3d o, d, hit;
        nav::viewRay(fcam, view, m, o, d);
        int part;
        fPress = pickPart(o, d, part, hit) ? part : -1;
        fBoxing = true;
        fBoxStart = m;
    }
    if (fBoxing) {
        Rectangle br{std::min(fBoxStart.x, m.x), std::min(fBoxStart.y, m.y), std::fabs(m.x - fBoxStart.x), std::fabs(m.y - fBoxStart.y)};
        bool crossing = m.x < fBoxStart.x;
        if (br.width >= 4 || br.height >= 4) {
            Color bc = crossing ? Color{74, 222, 128, 255} : Color{56, 189, 248, 255};
            DrawRectangleRec(br, Color{bc.r, bc.g, bc.b, 40});
            DrawRectangleLinesEx(br, 1.5f, bc);
        }
        ui::blockRect(view);
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            fBoxing = false;
            auto addSel = [&](int i) {
                if (i < 0) return;
                if (selected < 0) selected = i;
                else if (i != selected && !isMulti(i)) multiSel.push_back(i);
            };
            if (br.width < 4 && br.height < 4) {
                if (addMod) {
                    if (fPress >= 0 && isMulti(fPress)) multiSel.erase(std::find(multiSel.begin(), multiSel.end(), fPress));
                    else if (fPress >= 0 && fPress == selected) {
                        selected = multiSel.empty() ? -1 : multiSel.back();
                        if (!multiSel.empty()) multiSel.pop_back();
                    } else addSel(fPress);
                } else {
                    selected = fPress;
                    multiSel.clear();
                }
            } else {
                if (!addMod) { selected = -1; multiSel.clear(); }
                double z = baseZ;
                for (int i = (int)app.design.stack.size() - 1; i >= 0; i--) {
                    PartGeom g = partGeom(app.design.stack[i]);
                    double r = std::max(g.rTop, g.rBot);
                    int inside = 0, total = 0;
                    for (int k = 0; k < 8; k++) {
                        Vec3d c{(k & 1) ? r : -r, (k & 2) ? r : -r, z + ((k & 4) ? g.height : 0)};
                        Vector2 sp;
                        total++;
                        if (nav::project(fcam, view, c, sp) && CheckCollisionPointRec(sp, br)) inside++;
                    }
                    if ((crossing ? inside > 0 : inside == total) && !app.design.stack[i].hidden) addSel(i); // (hidden parts: not selectable in the view)
                    z += g.height;
                }
            }
        }
    }
    if (!modal) nav::navBar(fcam, {view.x + view.width / 2, view.y + view.height - 10}, fitFn, homeFn, true);
    text(fit("Right drag: orbit   Middle drag: pan   Wheel: zoom   Click / left drag: select parts", view.width / 2 - 150, 13), view.x + 16,
         view.y + view.height - 28, 13, col::faint);
}

void loadModal(App& app) {
    if (!loadOpen) return;
    dim();
    float W = 760, H = 560;
    Rectangle r{(ui::SW() - W) / 2, (ui::SH() - H) / 2, W, H};
    panel(r, Color{20, 26, 36, 252}, 14);
    text("Open design", r.x + 24, r.y + 18, 24, col::text, true);
    if (iconButton({r.x + W - 48, r.y + 16, 32, 32}, Icon::Close) || IsKeyPressed(KEY_ESCAPE)) loadOpen = false;
    float colW = (W - 72) / 2;
    text("TEMPLATES", r.x + 24, r.y + 64, 13, col::dim, true);
    text("SAVED DESIGNS", r.x + 48 + colW, r.y + 64, 13, col::dim, true);
    auto tpl = builtinTemplates();
    float y = r.y + 88;
    for (auto& d : tpl) {
        Rectangle row{r.x + 24, y, colW, 54};
        if (rowItem(row, false)) {
            app.design = d;
            selected = -1;
            loadOpen = false;
            app.setStatus(trf("Loaded template: {0}", d.name));
        }
        DesignStats st = analyzeDesign(d);
        icon(Icon::Rocket, {row.x + 10, row.y + 15, 22, 22}, col::accent);
        text(d.name, row.x + 42, row.y + 8, 16, col::text, true);
        text(massStr(st.wetMass) + "  ·  Δv " + thousands(st.totalDvVac) + " m/s  ·  " + trf("{0} stages", std::to_string(st.stages.size())),
             row.x + 42, row.y + 30, 13, col::dim);
        y += 58;
    }
    Rectangle listR{r.x + 48 + colW, r.y + 88, colW, H - 112};
    float off = beginScroll(77, listR, savedFiles.size() * 46.0f);
    y = listR.y - off;
    std::string toDelete;
    for (auto& sf : savedFiles) {
        const std::string& f = sf.path;
        Rectangle row{listR.x, y, colW - 46, 42};
        std::string nm = sf.name.empty() ? std::filesystem::path(f).stem().string() : sf.name; // (the name as saved)
        if (rowItem(row, false)) {
            Design d;
            if (loadDesign(d, f)) {
                ensureGlyphs(d.name); // (a name in another script than the interface language)
                app.design = d;
                selected = -1;
                loadOpen = false;
                app.setStatus(trf("Loaded {0}", d.name));
            } else app.setStatus(trf("Could not read {0}", nm));
        }
        text(fit(nm, colW - 70, 15), row.x + 12, row.y + 12, 15, col::text);
        static std::string deleteArmed; // first click arms, second click deletes
        bool armed = deleteArmed == f;
        if (armed) textR("Click again to delete", listR.x + colW - 48, y + 14, 12, col::bad);
        if (iconButton({listR.x + colW - 40, y + 5, 32, 32}, armed ? Icon::Check : Icon::Trash, armed, armed ? "Click again to delete" : "Delete")) {
            if (armed) { toDelete = f; deleteArmed.clear(); }
            else deleteArmed = f;
        }
        y += 46;
    }
    endScroll();
    if (savedFiles.empty()) text("No saved designs yet", listR.x + 12, listR.y + 12, 15, col::faint);
    if (!toDelete.empty()) {
        std::error_code ec;
        // only delete files inside the design directory
        auto p = std::filesystem::weakly_canonical(toDelete, ec);
        auto base = std::filesystem::weakly_canonical(app.designDir, ec);
        if (!ec && p.parent_path() == base && p.extension() == ".sfd") std::filesystem::remove(p, ec);
        refreshSaved(app);
    }
}

} // namespace

// design history: an edit is committed once it is finished (mouse released, no field being typed in),
// so a drag or a typed number is one undo step
std::vector<Design> fUndo, fRedo;
Design fLast;
std::string fLastText;
bool fHistInit = false;

void factoryHistory(App& app) {
    Design& D = app.design;
    std::string cur = designToText(D);
    if (!fHistInit) {
        fLast = D;
        fLastText = cur;
        fHistInit = true;
        return;
    }
    if (cur == fLastText || IsMouseButtonDown(MOUSE_BUTTON_LEFT) || ui::editing()) return;
    fUndo.push_back(fLast);
    if (fUndo.size() > 100) fUndo.erase(fUndo.begin());
    fRedo.clear();
    fLast = D;
    fLastText = cur;
}

void factoryUndoRedo(App& app, bool redo) {
    auto& from = redo ? fRedo : fUndo;
    auto& to = redo ? fUndo : fRedo;
    if (from.empty()) return;
    to.push_back(app.design);
    app.design = from.back();
    from.pop_back();
    fLast = app.design;
    fLastText = designToText(app.design);
    selected = std::min(selected, (int)app.design.stack.size() - 1);
    multiSel.clear();
    app.setStatus(redo ? "Redo" : "Undo");
}

void factoryDebugUndo(App& app, bool redo) { factoryUndoRedo(app, redo); }
// (autotest) the Save button, whether the overwrite dialog is up, and the choice to make in it (0 cancel, 1 new
// name, 2 overwrite) on the next frame
void factoryDebugSave(App& app) { requestSave(app); }
void factoryDebugSaveAs(const std::string& name) { openSaveAs(name); }
bool factoryDebugSaveAsOpen() { return saveAsOpen; }
bool factoryDebugOverwriteOpen() { return confirmOverwrite; }
void factoryDebugOpenList(App& app, bool on) {
    refreshSaved(app);
    loadOpen = on;
}
std::vector<std::string> factoryDebugSavedNames(App& app) {
    refreshSaved(app);
    std::vector<std::string> v;
    for (auto& s : savedFiles) v.push_back(s.name);
    return v;
}
void factoryDebugOverwriteChoice(int c) { g_testOverwrite = c; }

// where the show / hide buttons of the stack list were drawn (tests)
Rectangle g_eyeRect[64]{};
bool factoryDebugEye(int i, Rectangle& r) {
    if (i < 0 || i >= 64 || g_eyeRect[i].width <= 0) return false;
    r = g_eyeRect[i];
    return true;
}

void factoryFrame(App& app) {
    Design& D = app.design;
    factoryHistory(app);
    if (selected >= (int)D.stack.size()) selected = (int)D.stack.size() - 1;
    multiSel.erase(std::remove_if(multiSel.begin(), multiSel.end(), [&](int i) { return i < 0 || i >= (int)D.stack.size() || i == selected; }), multiSel.end());
    DesignStats st = analyzeDesign(D);
    float W = ui::SW(), H = ui::SH();
    bool modal = loadOpen || app.settingsOpen || confirmNew || confirmOverwrite || saveAsOpen;
    ui::setInputBlocked(modal);

    ClearBackground(col::bg);
    Rectangle view{344, 64, W - 344 - 384, H - 64};
    drawScene(app, view, st);

    // ---------------- keyboard
    if (!modal && !keyboardCaptured()) {
        // bindings: Ctrl+Z / mouse back = undo, Ctrl+Shift+Z / mouse forward = redo (Ctrl+Y kept as a fixed alias)
        bool ctrlOnly = (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && !IsKeyDown(KEY_LEFT_SHIFT) && !IsKeyDown(KEY_RIGHT_SHIFT);
        if (keys::pressed(keys::A::Undo)) factoryUndoRedo(app, false);
        else if (keys::pressed(keys::A::Redo) || (ctrlOnly && IsKeyPressed(KEY_Y))) factoryUndoRedo(app, true);
        // Ctrl+X (cut) also removes the selection, as in other programs
        bool cutKey = ctrlOnly && IsKeyPressed(KEY_X);
        if ((keys::pressed(keys::A::Delete) || cutKey) && selected >= 0) {
            std::vector<int> del = multiSel;
            del.push_back(selected);
            std::sort(del.rbegin(), del.rend());
            del.erase(std::unique(del.begin(), del.end()), del.end());
            for (int i : del)
                if (i >= 0 && i < (int)D.stack.size()) D.stack.erase(D.stack.begin() + i);
            multiSel.clear();
            selected = std::min(del.back(), (int)D.stack.size() - 1);
        }
        if (IsKeyPressed(KEY_UP) && selected > 0) selected--;
        if (IsKeyPressed(KEY_DOWN) && selected + 1 < (int)D.stack.size()) selected++;
    }

    // ---------------- top bar
    DrawRectangle(0, 0, (int)W, 56, Color{14, 18, 26, 255});
    DrawRectangle(0, 56, (int)W, 1, col::line);
    icon(Icon::Rocket, {18, 14, 28, 28}, col::accent);
    text("ASTRAUM", 56, 10, 20, col::text, true);
    text("Engineering Factory  ·  Earth", 56, 32, 14, col::dim);
    float bx = W - 16;
    auto topBtn = [&](float w, const std::string& label, Icon ic) {
        bx -= w;
        Rectangle r{bx, 10, w, 36};
        bx -= 8;
        return r;
    };
    Rectangle rLaunch = topBtn(150, "", Icon::Rocket);
    if (primaryButton(rLaunch, "LAUNCH", Icon::Rocket, !D.stack.empty() && !modal)) {
        app.lastLaunched = craftFromDesign(D);
        app.lastLaunchBody = -2;
        app.world.launch(app.lastLaunched);
        flightOnEnter(app);
        app.screen = Screen::Flight;
        return;
    }
    Vessel* av = app.world.activeVessel();
    Rectangle rFlight = topBtn(190, "", Icon::Play);
    if (button(rFlight, "Return to flight", false, av != nullptr && !modal, Icon::Play)) {
        flightOnEnter(app);
        app.screen = Screen::Flight;
        return;
    }
    Rectangle rSet = topBtn(40, "", Icon::Gear);
    if (iconButton(rSet, Icon::Gear, false, "Settings", !modal)) app.settingsOpen = true;
    Rectangle rHome = topBtn(40, "", Icon::Home);
    if (iconButton(rHome, Icon::Home, false, "Home screen", !modal)) {
        app.screen = Screen::Menu;
        return;
    }
    Rectangle rSave = topBtn(96, "", Icon::Save);
    if (button(rSave, "Save", false, !D.stack.empty() && !modal, Icon::Save)) openSaveAs(D.name);
    Rectangle rLoad = topBtn(96, "", Icon::Folder);
    if (button(rLoad, "Open", false, !modal, Icon::Folder)) {
        refreshSaved(app);
        loadOpen = true;
    }
    Rectangle rCad = topBtn(150, "", Icon::Cube);
    if (button(rCad, "Edit in CAD", false, !D.stack.empty() && !modal, Icon::Cube)) {
        cadSetCraft(app, craftFromDesign(D));
        app.screen = Screen::CAD;
        return;
    }
    Rectangle rNew = topBtn(86, "", Icon::Plus);
    if (button(rNew, "New", false, !modal, Icon::Plus)) confirmNew = true;
    // launch site
    // launch site: the Earth sites, then the surface bases with a launch pad (click: next, right-click: previous)
    Rectangle rSite{bx - 280, 10, 280, 36};
    bool atBase = app.world.padSite().baseId > 0;
    std::string siteLbl = fit(trf("Site: {0}", tr(app.world.launchSiteName())), 230, 16);
    if (button(rSite, siteLbl, atBase, !modal, atBase ? Icon::Base : Icon::Target)) app.world.cycleLaunchSite(1);
    if (CheckCollisionPointRec(GetMousePosition(), rSite) && !modal) {
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) app.world.cycleLaunchSite(-1);
        tooltip(app.world.launchBaseIds().empty()
                    ? tr("Launch site (click: next, right-click: previous). Surface bases with a launch pad are listed here too.")
                    : tr("Launch site (click: next, right-click: previous): the Earth sites and your surface bases with a launch pad."));
    }

    // ---------------- left: parts catalogue
    Rectangle L{12, 68, 320, H - 80};
    panel(L);
    text("PARTS", L.x + 16, L.y + 14, 13, col::dim, true);
    const char* cats[] = {"All", "Command", "Tanks", "Engines", "Structure", "Power", "Control", "Utility", "Fun", "Base"};
    // (chip i shows category catOf[i]; 0 = all)
    static const int catOf[] = {-1, (int)Cat::Command, (int)Cat::Propellant, (int)Cat::Engines, (int)Cat::Structure, (int)Cat::Power,
                                (int)Cat::Control, (int)Cat::Utility, (int)Cat::Fun, (int)Cat::Base};
    float cx = L.x + 14, cy = L.y + 38;
    for (int i = 0; i < 10; i++) {
        float w = measure(cats[i], 14, true).x + 22;
        if (cx + w > L.x + L.width - 12) { cx = L.x + 14; cy += 32; }
        if (chip({cx, cy, w, 26}, cats[i], catSel == i) && !modal) catSel = i;
        cx += w + 6;
    }
    cy += 34;
    {
        // placement of the next part: stacked below the selected part, or on its side
        text("Placement", L.x + 16, cy + 5, 13, col::dim);
        float px = L.x + 16 + measure("Placement", 13).x + 12;
        float w1 = measure("Stack", 13, true).x + 22, w2 = measure("Side", 13, true).x + 22;
        if (chip({px, cy, w1, 26}, "Stack", !placeSide) && !modal) placeSide = false;
        if (chip({px + w1 + 6, cy, w2, 26}, "Side", placeSide) && !modal) placeSide = true;
    }
    float listTop = cy + 38;
    float descH = 120;
    Rectangle listR{L.x + 8, listTop, L.width - 16, L.y + L.height - listTop - descH - 8};
    std::vector<int> shown;
    for (int i = 0; i < (int)CATALOG.defs.size(); i++) {
        if (CATALOG.defs[i].cat == Cat::Aircraft) continue; // aircraft are built in the CAD designer
        if (catSel == 0 || (int)CATALOG.defs[i].cat == catOf[std::clamp(catSel, 0, 9)]) shown.push_back(i);
    }
    float off = beginScroll(1, listR, shown.size() * 58.0f);
    float y = listR.y - off;
    hoverPart = -1;
    for (int idx : shown) {
        const PartDef& d = CATALOG.defs[idx];
        Rectangle row{listR.x, y, listR.width - 8, 54};
        if (y + 54 >= listR.y && y <= listR.y + listR.height) {
            if (CheckCollisionPointRec(GetMousePosition(), row) && inClip(GetMousePosition())) hoverPart = idx;
            if (rowItem(row, false) && !modal) addPart(app, idx);
            Icon ic = Icon::Planet;
            switch (d.cat) {
            case Cat::Command: ic = Icon::Ship; break;
            case Cat::Propellant: ic = Icon::Stage; break;
            case Cat::Engines: ic = Icon::Flame; break;
            case Cat::Power: ic = Icon::Bolt; break;
            case Cat::Control: ic = Icon::Target; break;
            case Cat::Utility: ic = d.kind == PK::Sail ? Icon::Sail : Icon::Info; break;
            case Cat::Fun: ic = Icon::Hyper; break;
            default: ic = Icon::Copy; break;
            }
            DrawRectangleRounded({row.x + 8, row.y + 9, 36, 36}, 0.3f, 6, Color{30, 38, 52, 255});
            icon(ic, {row.x + 16, row.y + 17, 20, 20}, d.cat == Cat::Fun ? col::accent2 : col::accent);
            text(fit(d.name, row.width - 64, 15, true), row.x + 54, row.y + 8, 15, col::text, true);
            text(fit(partSubtitle(d), row.width - 64, 13), row.x + 54, row.y + 30, 13, col::dim);
        }
        y += 58;
    }
    endScroll();
    Rectangle descR{L.x + 12, L.y + L.height - descH, L.width - 24, descH - 12};
    DrawRectangleRounded(descR, 0.12f, 6, Color{14, 18, 26, 255});
    if (hoverPart >= 0) {
        const PartDef& d = CATALOG.defs[hoverPart];
        text(fit(d.name, descR.width - 20, 15, true), descR.x + 10, descR.y + 8, 15, col::accent, true);
        wrap(partDescription(d), descR.x + 10, descR.y + 32, descR.width - 20, 14, col::text, descR.y + descR.height - 4);
    } else {
        wrap(placeSide ? "Side placement: click a solid booster or a radial part to attach it around the selected part. Height and angle are set below the part list."
                       : "Click a part to add it below the selected part. Radial parts attach to the selected part's sides; switch Placement to Side for boosters.",
             descR.x + 10,
             descR.y + 12, descR.width - 20, 14, col::dim, descR.y + descR.height - 4);
    }

    // ---------------- right: assembly
    Rectangle R{W - 372, 68, 360, H - 80};
    panel(R);
    text("VESSEL", R.x + 16, R.y + 14, 13, col::dim, true);
    textInput({R.x + 16, R.y + 36, R.width - 32, 34}, D.name, 40, 1001);
    int segN = 0;
    auto seg = segmentOfParts(D, segN);
    float listH = std::max(160.0f, (R.height - 120) * 0.38f);
    Rectangle sl{R.x + 8, R.y + 82, R.width - 16, listH};
    DrawRectangleRounded(sl, 0.05f, 6, Color{14, 18, 26, 255});
    float contentH = 0;
    for (size_t i = 0; i < D.stack.size(); i++) {
        if (i == 0 || seg[i] != seg[i - 1]) contentH += 26;
        contentH += 32 + D.stack[i].radial.size() * 22.0f;
    }
    off = beginScroll(2, sl, contentH + 8);
    y = sl.y + 4 - off;
    int moveReq = 0, removeRadial = -1;
    for (size_t i = 0; i < D.stack.size(); i++) {
        if (i == 0 || seg[i] != seg[i - 1]) {
            std::string hdr = trf(seg[i] == 0 ? "STAGE {0}  ·  fires first" : "STAGE {0}", std::to_string(seg[i] + 1));
            text(hdr, sl.x + 10, y + 6, 12, seg[i] == 0 ? col::warn : col::faint, true);
            y += 26;
        }
        const PartInst& p = D.stack[i];
        const PartDef& d = CATALOG.defs[p.def];
        Rectangle row{sl.x + 4, y, sl.width - 14, 30};
        // show / hide in the 3D view (the eye; hidden parts are drawn faintly and cannot be clicked there)
        Rectangle eye{row.x + row.width - 30, row.y + 3, 26, 24};
        // the row's own click area stops short of the eye: a click there must reach the eye button (the row took it
        // first and the eye never saw it - the button did nothing)
        Rectangle rowHit{row.x, row.y, row.width - 34, row.height};
        if (i < 64) g_eyeRect[i] = eye;
        if (rowItem(rowHit, (int)i == selected || isMulti((int)i)) && !modal) {
            bool sh = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            bool ct = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
            if (sh && selected >= 0) {
                int a = std::min(selected, (int)i), b = std::max(selected, (int)i);
                for (int j = a; j <= b; j++)
                    if (j != selected && !isMulti(j)) multiSel.push_back(j);
            } else if (ct && selected >= 0 && (int)i != selected) {
                if (isMulti((int)i)) multiSel.erase(std::find(multiSel.begin(), multiSel.end(), (int)i));
                else multiSel.push_back((int)i);
            } else {
                selected = (int)i;
                multiSel.clear();
            }
        }
        if (iconButton(eye, Icon::Eye, !D.stack[i].hidden, D.stack[i].hidden ? "Show this part" : "Hide this part (it stays on the vessel)", !modal)) D.stack[i].hidden = !D.stack[i].hidden;
        Color nameC = p.hidden ? col::faint : col::text;
        text(fit(d.name, row.width * 0.52f, 14, (int)i == selected), row.x + 12, row.y + 7, 14, nameC, (int)i == selected);
        textR(sizeTag(p), row.x + row.width - 38, row.y + 8, 13, p.hidden ? col::faint : col::dim);
        y += 32;
        for (auto& ra : p.radial) {
            bool booster = CATALOG.defs[ra.def].kind == PK::SRB;
            text(fit("+ " + std::to_string(ra.count) + "×  " + tr(CATALOG.defs[ra.def].name) + (booster ? "  ·  " + tr("side boosters, separate first") : ""),
                     row.width - 32, 13),
                 row.x + 28, y + 2, 13, booster ? col::warn : col::dim);
            y += 22;
        }
    }
    endScroll();
    if (D.stack.empty()) {
        textC("Empty stack - add a command part first", sl, 15, col::faint);
    }

    // selected part editor
    float ey = sl.y + sl.height + 10;
    if (selected >= 0 && selected < (int)D.stack.size()) {
        PartInst& p = D.stack[selected];
        const PartDef& d = CATALOG.defs[p.def];
        text(fit(d.name, R.width - 180, 16, true), R.x + 16, ey + 6, 16, col::accent, true);
        float ix = R.x + R.width - 16 - 4 * 36;
        if (iconButton({ix, ey, 32, 32}, Icon::Up, false, "Move up", selected > 0 && !modal)) moveReq = -1;
        if (iconButton({ix + 36, ey, 32, 32}, Icon::Down, false, "Move down", selected + 1 < (int)D.stack.size() && !modal)) moveReq = 1;
        if (iconButton({ix + 72, ey, 32, 32}, Icon::Copy, false, "Duplicate", !modal)) {
            PartInst cpy = p;
            D.stack.insert(D.stack.begin() + selected + 1, cpy);
            selected++;
        } else if (iconButton({ix + 108, ey, 32, 32}, Icon::Trash, false, keys::tip("Remove", keys::A::Delete).c_str(), !modal)) {
            D.stack.erase(D.stack.begin() + selected);
            selected = std::min(selected, (int)D.stack.size() - 1);
            ey = -1;
        }
        if (ey >= 0 && selected >= 0) {
            PartInst& q = D.stack[selected];
            ey += 42;
            if (d.diamAdj) { stepper(R.x + 16, ey, R.width - 32, d.kind == PK::Adapter ? "Top diameter" : "Diameter", q.diam, 0.05, 0.5, 12, 2, " m"); ey += 34; }
            if (d.kind == PK::Adapter) { stepper(R.x + 16, ey, R.width - 32, "Bottom diameter", q.diam2, 0.05, 0.5, 12, 2, " m"); ey += 34; }
            if (d.lenAdj) { stepper(R.x + 16, ey, R.width - 32, "Length", q.length, 0.5, 0.5, 80, 1, " m"); ey += 34; }
            if (d.clusterAdj) {
                double c = q.cluster;
                if (stepper(R.x + 16, ey, R.width - 32, "Engine count", c, 1, 1, 19, 0, "")) q.cluster = (int)c;
                ey += 34;
            }
            if (d.kind == PK::Tank) {
                text(trf("Propellant {0}   ·   Dry {1}", massStr(tankCapacity(q)), massStr(partDryMass(q))), R.x + 16, ey + 4, 13, col::dim);
                ey += 24;
            }
            for (size_t k = 0; k < q.radial.size(); k++) {
                auto& ra = q.radial[k];
                bool booster = CATALOG.defs[ra.def].kind == PK::SRB;
                double c = ra.count;
                if (stepper(R.x + 16, ey, R.width - 70, fit(CATALOG.defs[ra.def].name, 150, 15), c, 1, booster ? 2 : 1, booster ? 8 : 12, 0, ""))
                    ra.count = (int)c;
                if (iconButton({R.x + R.width - 50, ey, 30, 28}, Icon::Close, false, "Detach")) removeRadial = (int)k;
                ey += 32;
                // free placement around the part: height along it and angle about the vehicle axis
                stepper(R.x + 36, ey, R.width - 90, booster ? "Base height" : "Height", ra.offset, 0.25, -60, 120, 2, " m");
                ey += 32;
                double a = ra.angle;
                if (stepper(R.x + 36, ey, R.width - 90, "Angle", a, 5, -5, 365, 0, "Â°")) ra.angle = std::fmod(a + 360.0, 360.0);
                ey += 36;
            }
            if (removeRadial >= 0) q.radial.erase(q.radial.begin() + removeRadial);
            if (moveReq != 0) {
                int j = selected + moveReq;
                if (j >= 0 && j < (int)D.stack.size()) { std::swap(D.stack[selected], D.stack[j]); selected = j; }
            }
        }
    } else {
        text("Select a part to edit it", R.x + 16, ey + 8, 14, col::faint);
        ey += 30;
    }

    // stats
    float sy = std::max(ey + 8, sl.y + sl.height + 60);
    divider(R.x + 16, sy, R.width - 32);
    sy += 10;
    Rectangle statR{R.x + 8, sy, R.width - 16, R.y + R.height - sy - 8};
    float statH = 150 + st.stages.size() * 22 + st.warnings.size() * 20 + 40;
    off = beginScroll(3, statR, statH);
    float y2 = statR.y - off;
    float lx = statR.x + 8, rx = statR.x + statR.width - 12;
    auto kv = [&](const std::string& k, const std::string& v, Color c = col::text) {
        text(k, lx, y2, 14, col::dim);
        textR(v, rx, y2, 14, c, true);
        y2 += 21;
    };
    text("PERFORMANCE", lx, y2, 13, col::dim, true);
    y2 += 22;
    kv("Total mass", massStr(st.wetMass));
    kv("Height  ·  max diameter", num(st.height, 1) + " m  ·  " + num(st.maxDiam, 2) + " m");
    kv("Total Δv (vacuum)", thousands(st.totalDvVac) + " m/s", col::accent);
    y2 += 4;
    text("STAGE", lx, y2, 12, col::faint, true);
    text("Δv vac", lx + 60, y2, 12, col::faint, true);
    text("Δv SL", lx + 140, y2, 12, col::faint, true);
    text("TWR", lx + 218, y2, 12, col::faint, true);
    text("burn", lx + 268, y2, 12, col::faint, true);
    y2 += 18;
    for (size_t s = 0; s < st.stages.size(); s++) {
        const StageStats& ss = st.stages[s];
        char b[128];
        if (ss.hasEngines)
            std::snprintf(b, sizeof b, "%zu", s + 1);
        else std::snprintf(b, sizeof b, "%zu", s + 1);
        text(b, lx + 6, y2, 14, s == 0 ? col::warn : col::text, true);
        if (ss.hasEngines) {
            text(thousands(ss.dvVac), lx + 60, y2, 14, col::text);
            text(thousands(ss.dvSL), lx + 140, y2, 14, col::dim);
            text(num(ss.twrSL, 2), lx + 218, y2, 14, ss.twrSL >= 1 ? col::good : col::dim);
            text(formatTime(ss.burnTime).substr(0, 12), lx + 268, y2, 13, col::dim);
        } else text("no engines", lx + 60, y2, 14, col::faint);
        y2 += 22;
    }
    y2 += 6;
    kv("Battery", num(st.batteryJ / 3.6e6, 2) + " kWh");
    kv("Generation  ·  solar @1 AU", powerStr(st.genW) + "  ·  " + powerStr(st.solarW1AU));
    kv("Continuous draw", powerStr(st.drawW));
    kv("Control torque", num(st.torque, 0) + " N m");
    if (st.crew) kv("Crew", std::to_string(st.crew));
    for (auto& w : st.warnings) {
        icon(Icon::Info, {lx, y2 + 1, 14, 14}, col::warn);
        text(fit(w, statR.width - 40, 13), lx + 20, y2, 13, col::warn);
        y2 += 20;
    }
    endScroll();

    // status line
    if (GetTime() - app.statusTime < 4.0 && !app.status.empty()) {
        Vector2 m = measure(app.status, 15);
        Rectangle sr{view.x + (view.width - m.x - 32) / 2, 76, m.x + 32, 34};
        DrawRectangleRounded(sr, 0.5f, 8, Color{20, 30, 44, 235});
        textC(app.status, sr, 15, col::text);
    }

    tut::frame(app);
    ui::setInputBlocked(app.settingsOpen);
    loadModal(app);
    if (confirmNew) {
        dim();
        Rectangle r{(W - 420) / 2, (H - 170) / 2, 420, 170};
        panel(r, Color{20, 26, 36, 252}, 14);
        text("Start a new design?", r.x + 24, r.y + 22, 20, col::text, true);
        text("The current unsaved design will be cleared.", r.x + 24, r.y + 58, 15, col::dim);
        if (button({r.x + 24, r.y + 110, 170, 40}, "Cancel") || IsKeyPressed(KEY_ESCAPE)) confirmNew = false;
        else if (primaryButton({r.x + 226, r.y + 110, 170, 40}, "Clear")) {
            D = Design{};
            selected = -1;
            confirmNew = false;
        }
    }
    if (confirmOverwrite) {
        dim(0.6f);
        // the buttons as wide as their labels (German is long), the dialog as wide as the buttons
        float wC = std::max(130.0f, measure(tr("Cancel"), 15).x + 40), wN = std::max(190.0f, measure(tr("Save under a new name"), 15).x + 64),
              wO = std::max(160.0f, measure(tr("Overwrite"), 15, true).x + 64);
        float dw = std::clamp(24 + wC + 12 + wN + 12 + wO + 24, 600.0f, W - 40);
        Rectangle r{(W - dw) / 2, (H - 230) / 2, dw, 230};
        panel(r, Color{20, 26, 36, 252}, 14);
        icon(Icon::Save, {r.x + 24, r.y + 20, 24, 24}, col::warn);
        text("Overwrite the saved design?", r.x + 58, r.y + 18, 22, col::text, true);
        ensureGlyphs(overwriteName);
        wrap(trf("A saved design named \"{0}\" exists already. Replace it with this design?", overwriteName), r.x + 24, r.y + 62, r.width - 48, 15, col::text);
        float by = r.y + r.height - 60;
        int choice = -1;
        if (button({r.x + 24, by, wC, 40}, "Cancel") || IsKeyPressed(KEY_ESCAPE)) choice = 0;
        if (button({r.x + r.width - 24 - wO - 12 - wN, by, wN, 40}, "Save under a new name", false, true, Icon::Copy)) choice = 1;
        if (primaryButton({r.x + r.width - 24 - wO, by, wO, 40}, "Overwrite", Icon::Save)) choice = 2;
        bool fromTest = false;
        if (g_testOverwrite >= 0) {
            choice = g_testOverwrite;
            g_testOverwrite = -1;
            fromTest = true;
        }
        if (choice == 1 && !fromTest) {
            // back to the name: the first free "name (2)", "name (3)", ... offered, to keep or to change
            std::error_code ec;
            std::string sug = D.name;
            for (int n = 2; n < 1000; n++) {
                std::string nm = cleanLineText(cleanLineText(D.name, 32) + " (" + std::to_string(n) + ")", 48);
                if (!std::filesystem::exists(designPath(app, nm), ec)) { sug = nm; break; }
            }
            openSaveAs(sug);
        } else if (choice == 1) {
            // the first free "name (2)", "name (3)", ... (the design takes that name)
            std::error_code ec;
            for (int n = 2; n < 1000; n++) {
                // (cut at a character boundary first: the name checked is exactly the name saved)
                std::string nm = cleanLineText(cleanLineText(D.name, 32) + " (" + std::to_string(n) + ")", 48);
                std::string p = designPath(app, nm);
                if (!std::filesystem::exists(p, ec)) {
                    D.name = nm;
                    saveDesignTo(app, D, p);
                    break;
                }
            }
        } else if (choice == 2) saveDesignTo(app, D, overwritePath);
        if (choice >= 0) {
            confirmOverwrite = false;
            overwritePath.clear();
        }
    }
    if (saveAsOpen && !confirmOverwrite) {
        dim(0.6f);
        float dw = std::min(560.0f, W - 40);
        Rectangle r{(W - dw) / 2, (H - 210) / 2, dw, 210};
        panel(r, Color{20, 26, 36, 252}, 14);
        icon(Icon::Save, {r.x + 24, r.y + 20, 24, 24}, col::accent);
        text("Save design", r.x + 58, r.y + 18, 22, col::text, true);
        text("Name", r.x + 24, r.y + 66, 14, col::dim);
        textInput({r.x + 24, r.y + 88, r.width - 48, 38}, saveAsName, 48, 1002);
        std::string nm = cleanLineText(saveAsName, 48);
        bool ok = nm.find_first_not_of(' ') != std::string::npos;
        float by = r.y + r.height - 60;
        if (button({r.x + 24, by, 140, 40}, "Cancel") || IsKeyPressed(KEY_ESCAPE)) {
            saveAsOpen = false;
            ui::cancelEdit();
        } else if (primaryButton({r.x + r.width - 24 - 160, by, 160, 40}, "Save", Icon::Save, ok) ||
                   (ok && (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)))) {
            saveAsOpen = false;
            ui::cancelEdit();
            D.name = nm;
            requestSave(app); // (asks first when a saved design has this name)
        }
    }
    ui::setInputBlocked(false);
    if (app.settingsOpen) settingsModal(app);
}
