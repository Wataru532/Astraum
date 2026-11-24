// CAD designer: free-form craft modelling with live mass-property and aerodynamic analysis
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include "app.hpp"
#include "screens.hpp"
#include "session.hpp"
#include "storage.hpp"
#include "nav.hpp"
#include "render.hpp"
#include "rl.hpp"
#include "ui.hpp"
#include "keys.hpp"

using namespace ui;

namespace {

nav::OrbitCam cam;
bool camInit = false;
int sel = -1;                // primary selection (properties panel, gizmo anchor)
std::vector<int> selSet;     // all selected components
bool boxing = false;         // rubber-band selection in progress
Vector2 boxStart{};
int pressComp = -1;
std::vector<Vec3d> gStartPosAll, gStartRotAll;
Vec3d gPivot0;

void selClear() { selSet.clear(); sel = -1; }
void selOnly(int i) {
    selSet.clear();
    if (i >= 0) selSet.push_back(i);
    sel = i;
}
bool isSel(int i) { return std::find(selSet.begin(), selSet.end(), i) != selSet.end(); }
void selToggle(int i) {
    auto it = std::find(selSet.begin(), selSet.end(), i);
    if (it != selSet.end()) {
        selSet.erase(it);
        sel = selSet.empty() ? -1 : selSet.back();
    } else {
        selSet.push_back(i);
        sel = i;
    }
}
void selValidate(int n) {
    selSet.erase(std::remove_if(selSet.begin(), selSet.end(), [n](int i) { return i < 0 || i >= n; }), selSet.end());
    std::sort(selSet.begin(), selSet.end());
    selSet.erase(std::unique(selSet.begin(), selSet.end()), selSet.end());
    if (sel >= n || (sel >= 0 && !isSel(sel))) sel = selSet.empty() ? -1 : selSet.back();
}
int tool = 1;               // 0 select, 1 move, 2 rotate
int gridIdx = 3, angleIdx = 2;
const double GRID[] = {0, 0.01, 0.05, 0.1, 0.25, 0.5, 1.0};
const double ANGLES[] = {0, 5, 15, 45};
bool xray = false, wire = false, showCoM = true, showCoP = true, showThrust = true;
int rightTab = 0;           // 0 properties, 1 analysis
int g_craftGen = 0;         // bumped whenever the whole craft is replaced (undo, redo, new, open)
int paletteCat = 0;
std::vector<Craft> undoStack, redoStack;
} // namespace

void cadTrimMemory(bool hard) {
    size_t keep = hard ? 10 : 40;
    if (undoStack.size() > keep) undoStack.erase(undoStack.begin(), undoStack.end() - keep);
    if (hard) redoStack.clear();
    undoStack.shrink_to_fit();
    redoStack.shrink_to_fit();
}

namespace {
bool openModal = false;
int openMode = 0;           // open dialog: 0 opens a design (replaces the current one), 1 adds it to the current one
std::vector<SavedFile> files;
// analysis cache
std::string analysedText;
CraftStats stats;
std::shared_ptr<AeroTable> aero;
double lastChange = 0;
bool importCraft(App& app, const Craft& add);
bool dirty = true;
// gizmo drag
int gAxis = -1;             // 0..2 dragging; hover in gHover
int gHover = -1;
Vec3d gStartPos, gStartRot;
double gStartParam = 0, gStartAngle = 0;

std::string num(double v, int prec = 2) {
    char b[48];
    std::snprintf(b, sizeof b, "%.*f", prec, v);
    return b;
}
std::string massStr(double kg) { return kg >= 10000 ? num(kg / 1000.0, 2) + " t" : num(kg, 0) + " kg"; }

void pushUndo(const Craft& c) {
    undoStack.push_back(c);
    if (undoStack.size() > 200) undoStack.erase(undoStack.begin());
    redoStack.clear();
}

Vec3d eulerFromMat(const Mat3d& R);

void deleteSelection(Craft& c) {
    if (selSet.empty()) return;
    pushUndo(c);
    std::vector<int> idx = selSet;
    std::sort(idx.rbegin(), idx.rend());
    for (int i : idx)
        if (i >= 0 && i < (int)c.comps.size()) c.comps.erase(c.comps.begin() + i);
    selClear();
    dirty = true;
}

// what a copy of a component starts with: the same part, not docked to anything and from no other module
Component freshCopy(const Component& k) {
    Component d = k;
    d.dockTag = d.dockedTo = d.origin = 0;
    d.stage0 = -1;
    return d;
}

// Duplicate: a copy of the selection beside it (offset in X by the width of the selection), exactly the same
// shape as the original - a part with radial symmetry becomes its separate copies (the ring turns about the craft
// axis, so the moved ring would be a different one), wing panels keep their root end and gear legs their fold
// (both follow from the side of the craft axis a part sits on unless fixed). Mirror X: the mirror image in the
// craft's YZ plane. Either all of it fits under the component limit or nothing is added.
void duplicateSelection(App& app, Craft& c, bool mirror) {
    if (selSet.empty()) return;
    std::vector<Component> src;
    for (int i : selSet) {
        if (i < 0 || i >= (int)c.comps.size()) continue;
        const Component& k = c.comps[i];
        if (mirror || k.symmetry <= 1) src.push_back(k);
        else {
            Craft one;
            one.comps.push_back(k);
            for (const Component& e : expandCraft(one)) src.push_back(e);
        }
    }
    if (src.empty()) return;
    if (c.comps.size() + src.size() > 400) {
        app.setStatus("Component limit reached (400)");
        return;
    }
    pushUndo(c);
    double lo = 1e30, hi = -1e30;
    std::vector<Vec3d> pts;
    for (const Component& k : src) {
        pts.clear();
        outlinePoints(k, pts, 12);
        for (const Vec3d& p : pts) {
            lo = std::min(lo, p.x);
            hi = std::max(hi, p.x);
        }
    }
    double shift = std::max(1.0, (hi - lo) * 1.1);
    std::vector<int> added;
    for (const Component& k : src) {
        Component d = freshCopy(k);
        if (mirror) d = mirroredX(d);
        else {
            freezePlacement(d);
            d.pos.x = clampd(d.pos.x + shift, -2000, 2000);
        }
        c.comps.push_back(d);
        added.push_back((int)c.comps.size() - 1);
    }
    selSet = added;
    sel = added.back();
    dirty = true;
    lastChange = GetTime();
}

// another design added to the current one (not replacing it): placed on top of it along the craft axis, so that its
// radial symmetry and everything else it derives from the axis stay exactly as designed; stage groups as they were
// in that design. The added parts are selected (to be moved into place).
// a design loaded into the payload bay: turned so that it fits (its longest side along the bay), its middle on the
// bay's centre, every part of it a payload (released from the bay in space)
bool importIntoBay(App& app, const Craft& add) {
    Craft& C = app.craft;
    int bi = -1;
    for (int i = 0; i < (int)C.comps.size() && bi < 0; i++)
        if (C.comps[i].func == Func::CargoBay) bi = i;
    if (bi < 0 || add.comps.empty()) return false;
    std::vector<Component> parts = expandCraft(add);
    // the payload already in the bay (and its cradle: support parts standing inside the bay) makes way for this one
    const Component bay = C.comps[bi];
    auto inBay = [&](const Component& k) {
        if (k.func == Func::CargoBay) return false;
        if (k.cargo != 0) return true;
        if (k.func != Func::Structure) return false;
        std::vector<Vec3d> op;
        outlinePoints(k, op, 12);
        Vec3d he = bay.dims * 0.5 + Vec3d{0.05, 0.05, 0.05};
        for (const Vec3d& p : op) {
            Vec3d l = bay.rot().conj().rotate(p - bay.pos);
            if (std::fabs(l.x) > he.x || std::fabs(l.y) > he.y || std::fabs(l.z) > he.z) return false; // (wholly inside only)
        }
        return !op.empty();
    };
    int old = 0;
    for (const Component& k : C.comps) old += inBay(k);
    if (C.comps.size() - old + parts.size() > 400) {
        app.setStatus("Component limit reached (400)");
        return false;
    }
    Vec3d lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    std::vector<Vec3d> pts;
    for (const Component& k : parts) {
        pts.clear();
        outlinePoints(k, pts, 12);
        for (const Vec3d& p : pts) {
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
            hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        }
    }
    Vec3d size = hi - lo, he = bay.dims * 0.5;
    const Mat3d cand[6] = {Mat3d{},
                           Mat3d::fromCols({0, 1, 0}, {1, 0, 0}, {0, 0, -1}),
                           Mat3d::fromCols({1, 0, 0}, {0, 0, 1}, {0, -1, 0}),
                           Mat3d::fromCols({0, 0, 1}, {0, 1, 0}, {-1, 0, 0}),
                           Mat3d::fromCols({0, 1, 0}, {0, 0, 1}, {1, 0, 0}),
                           Mat3d::fromCols({0, 0, 1}, {1, 0, 0}, {0, 1, 0})};
    int pick = -1;
    for (int j = 0; j < 6 && pick < 0; j++) {
        Vec3d s = cand[j] * size;
        if (std::fabs(s.x) <= 2 * he.x * 0.97 && std::fabs(s.y) <= 2 * he.y * 0.97 && std::fabs(s.z) <= 2 * he.z * 0.97) pick = j;
    }
    if (pick < 0) {
        app.setStatus(trf("Too large for the payload bay ({0})", num(size.x, 1) + " x " + num(size.y, 1) + " x " + num(size.z, 1) + " m"));
        return false;
    }
    Mat3d RP = bay.rot().toMat() * cand[pick];
    Vec3d mid = (lo + hi) * 0.5;
    // set down on the bay's floor (centred across and along it)
    Vec3d sz = cand[pick] * size;
    Vec3d place = bay.pos + bay.rot().toMat() * Vec3d{0, -he.y + 0.05 + std::fabs(sz.y) / 2, 0};
    pushUndo(C);
    selClear();
    C.comps.erase(std::remove_if(C.comps.begin(), C.comps.end(), inBay), C.comps.end());
    for (Component d : parts) {
        d = freshCopy(d);
        freezePlacement(d);
        d.symmetry = 1;
        d.pos = place + RP * (d.pos - mid);
        d.rotDeg = eulerDegOf(RP * d.rot().toMat());
        d.cargo = 1;
        d.stage = bay.stage;
        C.comps.push_back(d);
        selSet.push_back((int)C.comps.size() - 1);
    }
    sel = selSet.back();
    g_craftGen++;
    dirty = true;
    lastChange = GetTime();
    ensureGlyphs(add.name);
    for (const Component& k : add.comps) ensureGlyphs(k.name);
    app.setStatus(old > 0 ? trf("Loaded {0} into the payload bay (the previous payload taken out)", add.name)
                          : trf("Loaded {0} into the payload bay", add.name));
    return true;
}

bool importCraft(App& app, const Craft& add) {
    Craft& C = app.craft;
    if (add.comps.empty()) return false;
    if (C.comps.size() + add.comps.size() > 400) {
        app.setStatus("Component limit reached (400)");
        return false;
    }
    double dz = 0;
    if (!C.comps.empty()) {
        double top = -1e30, bottom = 1e30;
        std::vector<Vec3d> pts;
        for (const Component& k : expandCraft(C)) {
            pts.clear();
            outlinePoints(k, pts, 12);
            for (const Vec3d& p : pts) top = std::max(top, p.z);
        }
        for (const Component& k : expandCraft(add)) {
            pts.clear();
            outlinePoints(k, pts, 12);
            for (const Vec3d& p : pts) bottom = std::min(bottom, p.z);
        }
        if (top > -1e29 && bottom < 1e29) dz = top - bottom;
    }
    pushUndo(C);
    selClear();
    for (const Component& k : add.comps) {
        Component d = freshCopy(k);
        d.pos.z = clampd(d.pos.z + dz, -2000, 2000);
        C.comps.push_back(d);
        selSet.push_back((int)C.comps.size() - 1);
    }
    sel = selSet.back();
    g_craftGen++;
    dirty = true;
    lastChange = GetTime();
    ensureGlyphs(add.name);
    for (const Component& k : add.comps) ensureGlyphs(k.name);
    app.setStatus(trf("Added {0} to the design", add.name));
    return true;
}

double snapLen(double v) { double g = GRID[gridIdx]; return g > 0 ? std::round(v / g) * g : v; }
double snapAng(double v) { double g = ANGLES[angleIdx]; return g > 0 ? std::round(v / g) * g : v; }

// Euler (XYZ extrinsic, degrees) from a rotation matrix
Vec3d eulerFromMat(const Mat3d& R) {
    double b = std::asin(clampd(-R.m[2][0], -1, 1));
    double a, c;
    if (std::fabs(std::cos(b)) > 1e-6) {
        a = std::atan2(R.m[2][1], R.m[2][2]);
        c = std::atan2(R.m[1][0], R.m[0][0]);
    } else {
        a = std::atan2(-R.m[1][2], R.m[1][1]);
        c = 0;
    }
    return {a / DEG, b / DEG, c / DEG};
}

void craftBounds(const Craft& c, Vec3d& centre, double& radius) {
    Vec3d lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    auto comps = expandCraft(c);
    for (auto& k : comps) {
        Vec3d he = shapeHalfExtent(k.shape, k.dims);
        double r = len(he);
        lo = {std::min(lo.x, k.pos.x - r), std::min(lo.y, k.pos.y - r), std::min(lo.z, k.pos.z - r)};
        hi = {std::max(hi.x, k.pos.x + r), std::max(hi.y, k.pos.y + r), std::max(hi.z, k.pos.z + r)};
    }
    if (comps.empty()) { centre = {0, 0, 1}; radius = 3; return; }
    centre = (lo + hi) * 0.5;
    radius = std::max(1.0, len(hi - lo) * 0.5);
}

// pick the (unexpanded) component under a ray: returns component index and distance
int pickComponent(const Craft& c, const Vec3d& o, const Vec3d& d, double& tBest) {
    int best = -1;
    tBest = 1e30;
    for (int i = 0; i < (int)c.comps.size(); i++) {
        Craft one;
        one.comps.push_back(c.comps[i]);
        for (auto& k : expandCraft(one)) {
            Mat3d R = k.rot().toMat();
            double t;
            if (shapeRay(k.shape, k.dims, R.T() * (o - k.pos), R.T() * d, t) && t < tBest) {
                tBest = t;
                best = i;
            }
        }
    }
    return best;
}

Vec3d axisVec(int a) { return a == 0 ? Vec3d{1, 0, 0} : a == 1 ? Vec3d{0, 1, 0} : Vec3d{0, 0, 1}; }
Color axisCol(int a, bool hot) {
    Color c = a == 0 ? Color{239, 68, 68, 255} : a == 1 ? Color{74, 222, 128, 255} : Color{59, 130, 246, 255};
    if (hot) c = Color{250, 204, 21, 255};
    return c;
}

// parameter along the axis line (through P, direction A) closest to the ray (o, d)
double axisParam(const Vec3d& P, const Vec3d& A, const Vec3d& o, const Vec3d& d) {
    Vec3d w0 = P - o;
    double a = dot(A, A), b = dot(A, d), c = dot(d, d), dd = dot(A, w0), e = dot(d, w0);
    double den = a * c - b * b;
    if (std::fabs(den) < 1e-12) return 0;
    return (b * e - c * dd) / den;
}

double distToSeg(Vector2 p, Vector2 a, Vector2 b) {
    double vx = b.x - a.x, vy = b.y - a.y, wx = p.x - a.x, wy = p.y - a.y;
    double l2 = vx * vx + vy * vy;
    double t = l2 > 0 ? clampd((wx * vx + wy * vy) / l2, 0, 1) : 0;
    double dx = a.x + t * vx - p.x, dy = a.y + t * vy - p.y;
    return std::sqrt(dx * dx + dy * dy);
}

void refreshFiles(App& app) {
    files = listSavedDesigns(app.designDir, ".sfc", 300);
    for (auto& s : files) ensureGlyphs(s.name); // (names in another script than the interface language)
}

Vec3d placeAbove(const Craft& c) {
    double top = 0;
    for (auto& k : expandCraft(c)) top = std::max(top, k.pos.z + shapeHalfExtent(k.shape, k.dims).z);
    return {0, 0, top};
}

void addComponent(App& app, Component k) {
    Craft& c = app.craft;
    if (c.comps.size() >= 400) { app.setStatus("Component limit reached (400)"); return; }
    pushUndo(c);
    double hz = shapeHalfExtent(k.shape, k.dims).z;
    if (sel >= 0 && sel < (int)c.comps.size()) {
        const Component& s = c.comps[sel];
        k.pos = s.pos + Vec3d{0, 0, shapeHalfExtent(s.shape, s.dims).z + hz};
        k.stage = s.stage;
        if (k.func == Func::Solar || k.func == Func::Battery || k.func == Func::RTG || k.func == Func::Wing || k.func == Func::Legs) {
            double r = std::max(s.dims.x, s.dims.y);
            k.pos = s.pos + Vec3d{r + shapeHalfExtent(k.shape, k.dims).x, 0, 0};
            if (k.func == Func::Wing) k.rotDeg = {90, 0, 0};
            k.symmetry = k.func == Func::Legs ? 4 : 2;
        }
    } else {
        k.pos = placeAbove(c) + Vec3d{0, 0, hz};
    }
    c.comps.push_back(k);
    selOnly((int)c.comps.size() - 1);
    dirty = true;
    lastChange = GetTime();
}

// ------------------------------------------------------------------ palette (left)
struct PaletteItem {
    std::string label, sub;
    std::function<Component()> make;
    Icon ic;
};

std::vector<PaletteItem> paletteItems(int cat) {
    std::vector<PaletteItem> v;
    if (cat == 0) {
        auto shape = [](Shape s, Vec3d d, const char* nm) {
            return [=]() {
                Component k = makeComponent(Func::Structure);
                k.shape = s;
                k.dims = d;
                k.name = nm;
                return k;
            };
        };
        v.push_back({"Box", "Structure", shape(Shape::Box, {1, 1, 1}, "Box"), Icon::Cube});
        v.push_back({"Cylinder", "Structure", shape(Shape::Cylinder, {0.6, 0.6, 1.5}, "Cylinder"), Icon::Stage});
        v.push_back({"Cone", "Structure", shape(Shape::Cylinder, {0.6, 0.0, 1.5}, "Cone"), Icon::Rocket});
        v.push_back({"Ellipsoid", "Structure", shape(Shape::Ellipsoid, {0.6, 0.6, 0.6}, "Ellipsoid"), Icon::Planet});
        v.push_back({"Wedge", "Structure", shape(Shape::Wedge, {1.2, 0.2, 1.0}, "Wedge"), Icon::Sail});
        v.push_back({"Truss", "Lattice beam for stations and booms: light, stiff, open (aluminium tubes)", [] {
                         Component k = makeComponent(Func::Structure);
                         k.shape = Shape::Truss;
                         k.dims = {1.2, 1.2, 8.0};
                         k.wall = 0.004;
                         k.material = 0;
                         k.hollow = false;
                         k.name = "Truss";
                         k.color = Color{200, 204, 210, 255};
                         return k;
                     }, Icon::Wire});
        v.push_back({"Fuselage", "Streamlined body: fuselage, pod, nacelle", shape(Shape::Loft, {1.2, 1.3, 6.0}, "Fuselage"), Icon::Rocket});
        v.push_back({"Interstage skirt", "Thin shell over an upper-stage engine", [] {
                         Component k = makeComponent(Func::Structure);
                         k.shape = Shape::Cylinder;
                         k.dims = {1.25, 1.25, 2.0};
                         k.hollow = true;
                         k.wall = 0.004;
                         k.material = 0;
                         k.color = Color{34, 36, 40, 255};
                         k.name = "Interstage";
                         return k;
                     },
                     Icon::Stage});
        v.push_back({"Wing / fin", "Thin-airfoil lift", [] { return makeComponent(Func::Wing); }, Icon::Sail});
        v.push_back({"Heat shield", "PICA ablator", [] { return makeComponent(Func::HeatShield); }, Icon::Flame});
        v.push_back({"Window", "A pane set into the hull below it (place it on the surface, local Z outward)", [] { return makeComponent(Func::Window); }, Icon::Cube});
        v.push_back({"Payload bay", "Box of the bay inside a fuselage (local Y: the doors); payloads in it are released in space", [] {
                         return makeComponent(Func::CargoBay); }, Icon::Cube});
        for (Prop p : {Prop::Kerolox, Prop::Hydrolox, Prop::Methalox, Prop::Hypergolic, Prop::LH2, Prop::Xenon, Prop::Monoprop, Prop::JetFuel}) {
            v.push_back({std::string(propName(p)), "Propellant tank", [p] {
                             Component k = makeComponent(Func::Tank);
                             k.prop = p;
                             k.color = propColor(p);
                             k.name = std::string(propName(p));
                             return k;
                         },
                         Icon::Stage});
        }
        return v;
    }
    if (cat == 4) {
        // aircraft: the airframe pieces first (wing, fuselage, fuel), then the rotors, jets, rocket-plane engine
        // and landing gear of the catalog
        v.push_back({"Wing / fin", "Tapered, swept airfoil panel (lift, control surfaces)", [] { return makeComponent(Func::Wing); }, Icon::Sail});
        v.push_back({"Window", "A pane set into the hull below it (place it on the surface, local Z outward)", [] { return makeComponent(Func::Window); }, Icon::Cube});
        v.push_back({"Fuselage", "Streamlined body: fuselage, pod, nacelle", [] {
                         Component k = makeComponent(Func::Structure);
                         k.shape = Shape::Loft;
                         k.dims = {1.2, 1.3, 6.0};
                         k.name = "Fuselage";
                         return k;
                     },
                     Icon::Rocket});
        for (Prop p : {Prop::JetFuel, Prop::LH2}) {
            v.push_back({std::string(propName(p)), p == Prop::JetFuel ? "Fuel tank for jet engines" : "Fuel of the combined-cycle engine's air-breathing mode", [p] {
                             Component k = makeComponent(Func::Tank);
                             k.prop = p;
                             k.color = propColor(p);
                             k.name = std::string(propName(p));
                             return k;
                         },
                         Icon::Stage});
        }
    }
    for (int i = 0; i < (int)CATALOG.defs.size(); i++) {
        const PartDef& d = CATALOG.defs[i];
        Func f;
        switch (d.kind) {
        case PK::Capsule:
        case PK::Probe: f = Func::Command; break;
        case PK::Engine:
        case PK::SRB: f = Func::Engine; break;
        case PK::Battery: f = Func::Battery; break;
        case PK::Solar: f = Func::Solar; break;
        case PK::RTG: f = Func::RTG; break;
        case PK::Reactor: f = Func::Reactor; break;
        case PK::CMG: f = Func::CMG; break;
        case PK::Parachute: f = Func::Parachute; break;
        case PK::Legs: f = Func::Legs; break;
        case PK::Sail: f = Func::Sail; break;
        case PK::Hyperdrive: f = Func::Hyperdrive; break;
        case PK::RCS: f = Func::RCS; break;
        case PK::Rotor: f = Func::Rotor; break;
        case PK::Jet: f = Func::Jet; break;
        case PK::Wheel: f = Func::Wheel; break;
        case PK::Docking: f = Func::Docking; break;
        default: continue;
        }
        int group = (f == Func::Engine || f == Func::Rotor || f == Func::Jet) ? 2 : (f == Func::Command || f == Func::CMG || f == Func::RCS) ? 1 : 3;
        if (d.cat == Cat::Aircraft) group = 4; // rotors, jets, the rocket-plane engine, gear and rover wheels
        if (group != cat) continue;
        v.push_back({d.name, funcName(f), [f, i] { return makeComponent(f, i); },
                     f == Func::Engine ? Icon::Flame : f == Func::Command ? Icon::Ship : Icon::Bolt});
    }
    return v;
}

// ------------------------------------------------------------------ properties (right)
float propRow(float x, float y, float w, const std::string& label) {
    text(fit(label, w * 0.42f, 14), x, y + 6, 14, col::dim);
    return x + w * 0.44f;
}

bool cycleRow(float x, float y, float w, const std::string& label, const std::string& value, int& delta) {
    float fx = propRow(x, y, w, label);
    float fw = x + w - fx;
    delta = 0;
    if (iconButton({fx, y, 28, 28}, Icon::Backward)) delta = -1;
    DrawRectangleRounded({fx + 32, y, fw - 64, 28}, 0.3f, 6, Color{20, 26, 36, 255});
    textC(fit(value, fw - 72, 13), {fx + 32, y, fw - 64, 28}, 13, col::text);
    if (iconButton({x + w - 28, y, 28, 28}, Icon::Forward)) delta = 1;
    return delta != 0;
}

// Fairing: the selected parts (the payload) enclosed in a payload fairing on the craft axis - a cylinder as wide as
// the payload with a clearance, a tangent ogive nose above it and, where the stage below is narrower, a boat-tail
// down onto it; two clamshell halves that come off by themselves above the thick air (as the factory builds them).
// A fairing already around the payload is replaced. Stage group: that of the part it stands on.
bool encloseInFairing(App& app, Craft& c) {
    std::vector<int> pick = selSet;
    if (pick.empty() && sel >= 0) pick.push_back(sel);
    Craft pay;
    for (int i : pick)
        if (i >= 0 && i < (int)c.comps.size() && c.comps[i].func != Func::Fairing) pay.comps.push_back(c.comps[i]);
    if (pay.comps.empty()) return false;
    std::vector<Vec3d> pts;
    double R = 0, z0 = 1e30, z1 = -1e30;
    // the payload as it rides inside: solar panels folded, landing legs and gear up (the fairing wraps that, not
    // the deployed span). The folding depends on the parts around, so it is worked out on the whole craft.
    std::vector<Component> all = expandCraft(c), stowed = stowedGeometry(all);
    for (Component& k : stowed) {
        if (k.func == Func::Legs) k = stowedLeg(k);
        else if (retractableWheel(k)) k = stowedWheel(k);
    }
    for (const Component& k : expandCraft(pay)) {
        const Component* use = &k;
        for (size_t j = 0; j < all.size(); j++)
            if (all[j].func == k.func && len(all[j].pos - k.pos) < 1e-6 && len(all[j].dims - k.dims) < 1e-6 && len(all[j].rotDeg - k.rotDeg) < 1e-6) {
                use = &stowed[j];
                break;
            }
        outlinePoints(*use, pts, 16);
    }
    for (const Vec3d& p : pts) {
        R = std::max(R, std::hypot(p.x, p.y));
        z0 = std::min(z0, p.z);
        z1 = std::max(z1, p.z);
    }
    if (pts.empty()) return false;
    const double clear = std::max(0.1, 0.06 * R);
    R += clear;
    // what it stands on: the other parts reaching up to the payload's bottom (its radius there, its stage)
    Craft rest;
    for (int i = 0; i < (int)c.comps.size(); i++)
        if (std::find(pick.begin(), pick.end(), i) == pick.end() && c.comps[i].func != Func::Fairing) rest.comps.push_back(c.comps[i]);
    double rBelow = 0, topBelow = -1e30;
    int stage = pay.comps.front().stage;
    for (const Component& k : rest.comps) {
        pts.clear();
        Craft one;
        one.comps.push_back(k);
        bool under = false;
        for (const Component& e : expandCraft(one)) outlinePoints(e, pts, 16);
        for (const Vec3d& p : pts)
            if (p.z <= z0 + 0.05 && p.z >= z0 - 0.4 && std::hypot(p.x, p.y) <= R) {
                rBelow = std::max(rBelow, std::hypot(p.x, p.y));
                under = true;
            }
        double top = -1e30;
        for (const Vec3d& p : pts) top = std::max(top, p.z);
        if (under && top > topBelow) { topBelow = top; stage = k.stage; }
    }
    const double zBot = z0 - 0.02, L = (z1 + clear) - zBot, Ln = fairingNoseLength(2 * R);
    struct Seg { double z0, h, rb, rt; };
    std::vector<Seg> segs;
    double bt = rBelow > 0.05 && rBelow < R - 0.05 ? (R - rBelow) * 1.4 : 0.0;
    if (bt > 0) segs.push_back({zBot - bt, bt, rBelow, R});
    segs.push_back({zBot, L, R, R});
    double rho = (R * R + Ln * Ln) / (2 * R);
    auto rAt = [&](double x) { return std::max(0.0, std::sqrt(std::max(0.0, rho * rho - x * x)) + R - rho); };
    const double xs[] = {0.0, 0.30, 0.52, 0.70, 0.84, 0.94, 1.0};
    for (int k = 0; k < 6; k++) {
        double a = xs[k] * Ln, b = xs[k + 1] * Ln;
        segs.push_back({zBot + L + a, b - a, rAt(a), k == 5 ? R * 0.04 : rAt(b)});
    }
    int nNew = (int)segs.size() * 2;
    // the fairing already there around this payload goes
    std::vector<int> old;
    for (int i = 0; i < (int)c.comps.size(); i++) {
        const Component& k = c.comps[i];
        if (k.func == Func::Fairing && k.pos.z + k.dims.z / 2 > z0 && k.pos.z - k.dims.z / 2 < z1 + Ln) old.push_back(i);
    }
    if ((int)c.comps.size() - (int)old.size() + nNew > 400) {
        app.setStatus("Component limit reached (400)");
        return false;
    }
    pushUndo(c);
    for (int j = (int)old.size() - 1; j >= 0; j--) c.comps.erase(c.comps.begin() + old[j]);
    int def = CATALOG.find("fairing");
    selClear();
    for (int side = 0; side < 2; side++)
        for (const Seg& sg : segs) {
            Component f = makeComponent(Func::Fairing);
            f.name = "Payload fairing";
            f.def = def;
            f.stage = stage;
            f.shape = Shape::Cylinder;
            f.dims = {sg.rb, sg.rt, sg.h};
            f.pos = {0, 0, sg.z0 + sg.h / 2};
            f.rotDeg = {0, 0, side ? 180.0 : 0.0};
            if (def >= 0) f.color = CATALOG.defs[def].color;
            f.hollow = true;
            f.wall = 0.004;
            double area = PI * (sg.rb + sg.rt) * std::sqrt(sg.h * sg.h + (sg.rb - sg.rt) * (sg.rb - sg.rt));
            f.massOverride = std::max(1.0, 8.0 * area * 0.5); // (composite shell ~8 kg/m2, one half)
            c.comps.push_back(f);
            selSet.push_back((int)c.comps.size() - 1);
        }
    sel = selSet.back();
    g_craftGen++;
    dirty = true;
    lastChange = GetTime();
    app.setStatus(trf("Payload fairing: {0} m wide, {1} m tall", num(2 * R, 2), num(bt + L + Ln, 1)));
    return true;
}

void multiPanel(App& app, Rectangle R) {
    Craft& c = app.craft;
    float x = R.x + 14, w = R.width - 28, y = R.y + 12;
    text(trf("{0} components selected", std::to_string(selSet.size())), x, y, 18, col::accent, true);
    y += 34;
    wrap("Shared settings are applied to every selected component.", x, y, w, 13, col::dim);
    y += 40;
    Craft before = c;
    bool changed = false;
    // stage group
    {
        int st = c.comps[selSet.front()].stage;
        float fx = propRow(x, y, w, "Stage group");
        bool mixed = false;
        for (int i : selSet) mixed |= c.comps[i].stage != st;
        if (iconButton({fx, y, 28, 28}, Icon::Minus)) { for (int i : selSet) c.comps[i].stage = std::max(0, c.comps[i].stage - 1); changed = true; }
        textC(mixed ? std::string("--") : trf("Stage {0}", std::to_string(st + 1)), {fx + 32, y, x + w - fx - 64, 28}, 14, col::text, true);
        if (iconButton({x + w - 28, y, 28, 28}, Icon::Plus)) { for (int i : selSet) c.comps[i].stage = std::min(31, c.comps[i].stage + 1); changed = true; }
        y += 36;
    }
    // material
    {
        int dlt;
        int m0 = c.comps[selSet.front()].material;
        if (cycleRow(x, y, w, "Material", tr(MATERIALS[m0].name), dlt)) {
            int nm = (m0 + dlt + MATERIAL_COUNT) % MATERIAL_COUNT;
            for (int i : selSet) {
                Component& k = c.comps[i];
                if (k.func == Func::Structure || k.func == Func::Tank || k.func == Func::Wing || k.func == Func::HeatShield) {
                    k.material = nm;
                    k.color = MATERIALS[nm].color;
                }
            }
            changed = true;
        }
        y += 40;
    }
    // colour
    {
        const Color sw[] = {{236, 236, 240, 255}, {205, 208, 214, 255}, {52, 55, 62, 255}, {200, 150, 40, 255},
                            {220, 80, 60, 255},   {60, 120, 220, 255},  {80, 170, 110, 255}, {130, 90, 200, 255}};
        text("Colour", x, y + 6, 14, col::dim);
        float cx = x + w * 0.44f;
        for (const Color& s2 : sw) {
            Rectangle b{cx, y + 4, 20, 20};
            DrawRectangleRounded(b, 0.4f, 4, s2);
            if (rowItem(b, false)) {
                for (int i : selSet) c.comps[i].color = s2;
                changed = true;
            }
            cx += 24;
        }
        y += 40;
    }
    double mass = 0;
    for (int i : selSet) {
        Craft one;
        one.comps.push_back(c.comps[i]);
        for (auto& e : expandCraft(one)) {
            CompPhys p;
            computeCompPhys(e, p);
            mass += p.dryMass;
        }
    }
    text(trf("Dry mass {0}", massStr(mass)), x, y, 13, col::dim);
    y += 30;
    if (changed) {
        pushUndo(before);
        dirty = true;
        lastChange = GetTime();
    }
    if (button({x, y, (w - 12) / 3, 32}, "Duplicate", false, true, Icon::Copy)) duplicateSelection(app, c, false);
    else if (button({x + (w - 12) / 3 + 6, y, (w - 12) / 3, 32}, "Mirror X", false, true, Icon::Mirror)) duplicateSelection(app, c, true);
    else if (button({x + 2 * ((w - 12) / 3 + 6), y, (w - 12) / 3, 32}, "Delete", false, true, Icon::Trash, col::bad)) deleteSelection(c);
    y += 40;
    Rectangle fb{x, y, w, 32};
    if (button(fb, "Enclose in a payload fairing", false, true, Icon::Cube)) encloseInFairing(app, c);
    if (CheckCollisionPointRec(GetMousePosition(), fb))
        tooltip(tr("A fairing sized to the selected parts, on the part below them; it comes off by itself above the thick air"));
}

void propertiesPanel(App& app, Rectangle R) {
    Craft& c = app.craft;
    if (selSet.size() > 1) {
        multiPanel(app, R);
        return;
    }
    if (sel < 0 || sel >= (int)c.comps.size()) {
        wrap("Select a component in the 3D view or the component list. Add parts from the palette on the left.", R.x + 16, R.y + 16,
             R.width - 32, 14, col::faint);
        return;
    }
    Component& k = c.comps[sel];
    Component before = k;
    float x = R.x + 14, w = R.width - 28;
    Rectangle sr{R.x + 4, R.y, R.width - 8, R.height};
    float off = beginScroll(310, sr, 1420);
    float y = R.y + 10 - off;
    static std::string nameBuf;
    static int nameFor = -1;
    if (nameFor != sel || !ui::keyboardCaptured()) { nameBuf = k.name; nameFor = sel; }
    if (textInput({x, y, w, 32}, nameBuf, 40, 3101)) k.name = nameBuf.empty() ? "Component" : nameBuf;
    y += 40;
    int dlt;
    // function & part model
    if (cycleRow(x, y, w, "Function", tr(funcName(k.func)), dlt)) {
        int f = ((int)k.func + dlt + (int)Func::COUNT) % (int)Func::COUNT;
        Component nk = makeComponent((Func)f);
        nk.pos = k.pos;
        nk.rotDeg = k.rotDeg;
        nk.stage = k.stage;
        nk.symmetry = k.symmetry;
        if (nk.def < 0) {
            // pick the first catalog part matching the new function
            for (int i = 0; i < (int)CATALOG.defs.size(); i++) {
                PK kd = CATALOG.defs[i].kind;
                bool ok = ((Func)f == Func::Engine && (kd == PK::Engine || kd == PK::SRB)) || ((Func)f == Func::Command && (kd == PK::Probe || kd == PK::Capsule)) ||
                          ((Func)f == Func::Battery && kd == PK::Battery) || ((Func)f == Func::Solar && kd == PK::Solar) || ((Func)f == Func::RTG && kd == PK::RTG) ||
                          ((Func)f == Func::Reactor && kd == PK::Reactor) || ((Func)f == Func::CMG && kd == PK::CMG) || ((Func)f == Func::Parachute && kd == PK::Parachute) ||
                          ((Func)f == Func::Legs && kd == PK::Legs) || ((Func)f == Func::Sail && kd == PK::Sail) || ((Func)f == Func::Hyperdrive && kd == PK::Hyperdrive) ||
                          ((Func)f == Func::RCS && kd == PK::RCS) || ((Func)f == Func::Rotor && kd == PK::Rotor) ||
                          ((Func)f == Func::Jet && kd == PK::Jet) || ((Func)f == Func::Wheel && kd == PK::Wheel) ||
                          ((Func)f == Func::Docking && kd == PK::Docking);
                if (ok) {
                    nk = makeComponent((Func)f, i);
                    nk.pos = k.pos;
                    nk.rotDeg = k.rotDeg;
                    nk.stage = k.stage;
                    nk.symmetry = k.symmetry;
                    break;
                }
            }
        }
        k = nk;
        nameFor = -1;
    }
    y += 34;
    if (k.def >= 0) {
        if (cycleRow(x, y, w, "Part model", tr(CATALOG.defs[k.def].name), dlt)) {
            PK kind = CATALOG.defs[k.def].kind;
            int n = (int)CATALOG.defs.size();
            for (int s = 1; s < n; s++) {
                int i = ((k.def + dlt * s) % n + n) % n;
                PK kd = CATALOG.defs[i].kind;
                bool same = kd == kind || (kind == PK::Engine && kd == PK::SRB) || (kind == PK::SRB && kd == PK::Engine) ||
                            (kind == PK::Probe && kd == PK::Capsule) || (kind == PK::Capsule && kd == PK::Probe);
                if (same) {
                    Component nk = makeComponent(k.func, i);
                    nk.pos = k.pos;
                    nk.rotDeg = k.rotDeg;
                    nk.stage = k.stage;
                    nk.symmetry = k.symmetry;
                    k = nk;
                    nameFor = -1;
                    break;
                }
            }
        }
        y += 34;
    }
    // shape
    text("SHAPE", x, y + 6, 12, col::faint, true);
    y += 26;
    {
        const char* sn[] = {"Box", "Cylinder / cone", "Ellipsoid", "Wedge", "Airfoil", "Fuselage"};
        float cx = x;
        for (int s = 0; s < 6; s++) {
            float cw = measure(sn[s], 13, true).x + 20;
            if (cx + cw > x + w) { cx = x; y += 32; }
            if (chip({cx, y, cw, 26}, sn[s], (int)k.shape == s)) k.shape = (Shape)s;
            cx += cw + 6;
        }
        y += 36;
    }
    // (one row per shape: a shape added to the enum without its row read past the table - the truss crashed)
    const char* dl[][3] = {{"Size X", "Size Y", "Size Z"}, {"Bottom radius", "Top radius", "Height"}, {"Radius X", "Radius Y", "Radius Z"},
                           {"Length X", "Width Y", "Height Z"}, {"Span X", "Mean chord Y", "Thickness Z"}, {"Width X", "Height Y", "Length Z"},
                           {"Size X", "Size Y", "Size Z"}};
    static_assert(sizeof dl / sizeof dl[0] == (size_t)Shape::COUNT, "a label row for every shape");
    double* dv[3] = {&k.dims.x, &k.dims.y, &k.dims.z};
    for (int a = 0; a < 3; a++) {
        float fx = propRow(x, y, w, dl[(int)k.shape][a]);
        numberField({fx, y, x + w - fx, 28}, *dv[a], 3, " m", 3200 + a, a < 2 && k.shape == Shape::Cylinder ? 0.0 : 0.01, 500);
        y += 32;
    }
    if (k.shape == Shape::Airfoil) {
        float fx = propRow(x, y, w, "Taper (tip / root)");
        numberField({fx, y, x + w - fx, 28}, k.taper, 2, "", 3205, 0.1, 1.0);
        y += 32;
        fx = propRow(x, y, w, "Sweep");
        numberField({fx, y, x + w - fx, 28}, k.sweep, 1, "°", 3206, -80, 80);
        y += 32;
        // the root end: the end toward the craft axis unless fixed (a copy placed beside keeps the one it had)
        std::string rv = k.rootSide == 0 ? trf("Auto ({0})", airfoilRoot(k) > 0 ? "+X" : "-X") : tr(k.rootSide > 0 ? "Local +X" : "Local -X");
        int dr;
        if (cycleRow(x, y, w, "Root end", rv, dr)) k.rootSide = ((k.rootSide + 1 + dr) % 3 + 3) % 3 - 1; // (0 -> 1 -> -1 -> 0)
        y += 34;
        if (k.func == Func::Wing) {
            // flaps: the inboard trailing edge; F steps through 9 settings (0 = up) up to this deflection
            fx = propRow(x, y, w, "Flap (full deflection)");
            numberField({fx, y, x + w - fx, 28}, k.flap, 1, "°", 3207, 0, 60);
            y += 32;
        }
    }
    if (k.func == Func::Window) {
        float fx = propRow(x, y, w, "Top width (of the bottom)");
        numberField({fx, y, x + w - fx, 28}, k.taper, 2, "", 3250, 0.1, 1.0);
        y += 32;
        fx = propRow(x, y, w, "Frame width");
        numberField({fx, y, x + w - fx, 28}, k.rim, 3, " m", 3251, 0, 0.5);
        y += 32;
    }
    if (k.func == Func::Wheel && retractableWheel(k)) {
        const char* fn[4] = {"", "Toward -X", "Toward +X", "Aft"};
        int g = gearFold(k);
        std::string fv = k.foldDir == 0 ? trf("Auto ({0})", tr(fn[g])) : tr(fn[std::clamp(k.foldDir, 1, 3)]);
        int dr;
        if (cycleRow(x, y, w, "Retracts", fv, dr)) k.foldDir = ((k.foldDir + dr) % 4 + 4) % 4;
        y += 34;
    }
    if (k.shape == Shape::Loft) {
        struct F { const char* label; double* v; double lo, hi; };
        const F f[6] = {{"Nose length", &k.noseLen, 0.02, 0.6}, {"Tail length", &k.tailLen, 0.02, 0.7}, {"Tail end size", &k.tailR, 0, 1},
                        {"Tail upsweep", &k.upsweep, -1, 1},  {"Nose bluntness", &k.bluff, 0, 1},  {"Squareness", &k.boxy, 0, 1}};
        for (int i = 0; i < 6; i++) {
            float fx = propRow(x, y, w, f[i].label);
            numberField({fx, y, x + w - fx, 28}, *f[i].v, 2, "", 3240 + i, f[i].lo, f[i].hi);
            y += 32;
        }
    }
    if (k.func != Func::CargoBay) {
        // a payload: carried in the payload bay and released from it in space as a vessel of its own
        bool cg = k.cargo != 0;
        if (toggle({x, y, w, 30}, "Payload (released from the bay)", cg)) k.cargo = cg ? 1 : 0;
        y += 34;
    }
    if (k.func != Func::Fairing && k.func != Func::CargoBay) {
        Rectangle fb{x, y, w, 32};
        if (button(fb, "Enclose in a payload fairing", false, true, Icon::Cube)) {
            encloseInFairing(app, c);
            endScroll();
            return; // (the selection changed)
        }
        if (CheckCollisionPointRec(GetMousePosition(), fb))
            tooltip(tr("A fairing sized to the selected parts, on the part below them; it comes off by itself above the thick air"));
        y += 40;
    }
    text("PLACEMENT", x, y + 6, 12, col::faint, true);
    y += 26;
    const char* pl[3] = {"Position X", "Position Y", "Position Z"};
    double* pv[3] = {&k.pos.x, &k.pos.y, &k.pos.z};
    for (int a = 0; a < 3; a++) {
        float fx = propRow(x, y, w, pl[a]);
        numberField({fx, y, x + w - fx, 28}, *pv[a], 3, " m", 3210 + a, -2000, 2000);
        y += 32;
    }
    const char* rl[3] = {"Rotation X", "Rotation Y", "Rotation Z"};
    double* rv[3] = {&k.rotDeg.x, &k.rotDeg.y, &k.rotDeg.z};
    for (int a = 0; a < 3; a++) {
        float fx = propRow(x, y, w, rl[a]);
        numberField({fx, y, x + w - fx, 28}, *rv[a], 1, "°", 3220 + a, -360, 360);
        y += 32;
    }
    {
        double sym = k.symmetry;
        float fx = propRow(x, y, w, "Radial symmetry");
        if (spinField({fx, y, x + w - fx, 28}, sym, 1, 1, 12, 0, " ×", 3290)) k.symmetry = (int)std::lround(sym);
        y += 32;
        double stg = k.stage + 1; // (shown from 1)
        fx = propRow(x, y, w, "Stage group");
        if (spinField({fx, y, x + w - fx, 28}, stg, 1, 1, 32, 0, "", 3291)) k.stage = (int)std::lround(stg) - 1;
        y += 36;
    }
    text("MATERIAL & MASS", x, y + 6, 12, col::faint, true);
    y += 26;
    bool matUsed = k.func == Func::Structure || k.func == Func::Tank || k.func == Func::Wing || k.func == Func::HeatShield;
    if (matUsed) {
        if (cycleRow(x, y, w, "Material", tr(MATERIALS[k.material].name), dlt)) {
            k.material = (k.material + dlt + MATERIAL_COUNT) % MATERIAL_COUNT;
            k.color = MATERIALS[k.material].color;
        }
        y += 34;
        if (k.func != Func::Tank) {
            bool hol = k.hollow;
            if (toggle({x, y, w, 30}, "Hollow shell", hol)) k.hollow = hol;
            y += 34;
        }
        if (k.hollow || k.func == Func::Tank) {
            double mm = k.wall * 1000;
            float fx = propRow(x, y, w, "Wall thickness");
            if (numberField({fx, y, x + w - fx, 28}, mm, 2, " mm", 3230, 0.1, 500)) k.wall = mm / 1000;
            y += 32;
        }
    }
    if (k.func == Func::Tank) {
        if (cycleRow(x, y, w, "Propellant", tr(propName(k.prop)), dlt)) {
            // liquid propellants only (solid propellant belongs to solid motors)
            int p = (int)k.prop;
            do {
                p += dlt;
                if (p < 1) p = PROP_N - 1;
                if (p > PROP_N - 1) p = 1;
            } while ((Prop)p == Prop::Solid);
            k.prop = (Prop)p;
            k.color = propColor(k.prop);
        }
        y += 34;
        double fp = k.fill * 100;
        float fx = propRow(x, y, w, "Fill level");
        if (numberField({fx, y, x + w - fx, 28}, fp, 1, " %", 3231, 0, 100)) k.fill = fp / 100;
        y += 32;
    }
    if (k.func == Func::Engine && k.def >= 0 && CATALOG.defs[k.def].kind == PK::Engine) {
        double cl = k.cluster;
        float fx = propRow(x, y, w, "Engine count");
        if (spinField({fx, y, x + w - fx, 28}, cl, 1, 1, 37, 0, " ×", 3292)) k.cluster = (int)std::lround(cl);
        y += 32;
    }
    if (k.func == Func::Engine) {
        // ignition: with its own stage group, or already from an earlier one (a core stage in group 2 lit at lift-off
        // beside side boosters in group 1: Space then drops the boosters only and the core burns on)
        if (k.fireFrom >= k.stage) k.fireFrom = -1;
        std::string iv = k.fireFrom < 0 ? trf("With its group (Stage {0})", std::to_string(k.stage + 1))
                                        : trf("From Stage {0} (beside it)", std::to_string(k.fireFrom + 1));
        int dr;
        Rectangle rr{x, y, w, 30};
        if (cycleRow(x, y, w, "Ignition", iv, dr) && k.stage > 0) {
            // -1, 0, 1 .. stage-1
            int n = k.stage + 1, i = k.fireFrom + 1;
            i = ((i + dr) % n + n) % n;
            k.fireFrom = i - 1;
        }
        if (CheckCollisionPointRec(GetMousePosition(), rr))
            tooltip(k.stage == 0 ? tr("Put the side boosters in Stage 1 and this engine in Stage 2 to light it at lift-off beside them")
                                 : tr("An engine in a later group can already burn from an earlier one: Space then drops the earlier group only"));
        y += 34;
    }
    {
        float fx = propRow(x, y, w, "Mass override (0 = auto)");
        numberField({fx, y, x + w - fx, 28}, k.massOverride, 1, " kg", 3232, 0, 1e7);
        y += 32;
    }
    // colour swatches
    {
        const Color sw[] = {{236, 236, 240, 255}, {205, 208, 214, 255}, {52, 55, 62, 255}, {200, 150, 40, 255},
                            {220, 80, 60, 255},   {60, 120, 220, 255},  {80, 170, 110, 255}, {130, 90, 200, 255}};
        text("Colour", x, y + 6, 14, col::dim);
        float cx = x + w * 0.44f;
        for (const Color& s : sw) {
            Rectangle b{cx, y + 4, 20, 20};
            DrawRectangleRounded(b, 0.4f, 4, s);
            if (k.color.r == s.r && k.color.g == s.g && k.color.b == s.b) DrawRectangleRoundedLinesEx(b, 0.4f, 4, 2, col::accent);
            if (rowItem(b, false)) k.color = s;
            cx += 24;
        }
        y += 36;
    }
    // computed info for this component
    {
        Craft one;
        one.comps.push_back(k);
        auto ex = expandCraft(one);
        double dry = 0, cap = 0;
        for (auto& e : ex) {
            CompPhys p;
            computeCompPhys(e, p);
            dry += p.dryMass;
            cap += p.propCap;
        }
        text(trf("Dry mass {0}", massStr(dry)) + (cap > 0 ? "   ·   " + trf("Propellant {0}", massStr(cap)) : ""), x, y, 13, col::dim);
        y += 20;
        text(trf("Volume {0}", num(shapeVolume(k.shape, k.dims) * ex.size(), 3) + " m³"), x, y, 13, col::dim);
        y += 26;
    }
    if (button({x, y, (w - 12) / 3, 32}, "Duplicate", false, true, Icon::Copy)) {
        duplicateSelection(app, c, false);
        endScroll();
        return;
    } else if (button({x + (w - 12) / 3 + 6, y, (w - 12) / 3, 32}, "Mirror X", false, true, Icon::Mirror)) {
        duplicateSelection(app, c, true);
        endScroll();
        return;
    } else if (button({x + 2 * ((w - 12) / 3 + 6), y, (w - 12) / 3, 32}, "Delete", false, true, Icon::Trash, col::bad)) {
        deleteSelection(c);
        endScroll();
        return;
    }
    endScroll();
    // record edits made through the property fields
    if (sel >= 0 && sel < (int)c.comps.size()) {
        Component& now = c.comps[sel];
        bool changed = craftToText(Craft{"", {before}}) != craftToText(Craft{"", {now}});
        if (changed) {
            Craft prev = c;
            prev.comps[sel] = before;
            pushUndo(prev);
            dirty = true;
            lastChange = GetTime();
        }
    }
}

void analysisPanel(App& app, Rectangle R) {
    Rectangle sr{R.x + 4, R.y, R.width - 8, R.height};
    float off = beginScroll(311, sr, 760 + stats.stages.size() * 22 + stats.warnings.size() * 36);
    float x = R.x + 14, w = R.width - 28;
    float y = R.y + 10 - off;
    auto kv = [&](const std::string& k, const std::string& v, Color c = col::text) {
        text(k, x, y, 14, col::dim);
        textR(v, x + w, y, 14, c, true);
        y += 21;
    };
    text("MASS PROPERTIES", x, y, 12, col::faint, true);
    y += 22;
    kv("Wet mass", massStr(stats.wet.mass));
    kv("Dry mass", massStr(stats.dry.mass));
    kv("Centre of mass", "(" + num(stats.wet.com.x) + ", " + num(stats.wet.com.y) + ", " + num(stats.wet.com.z) + ") m", col::warn);
    kv("Ixx  ·  Iyy  ·  Izz", num(stats.wet.inertia.m[0][0] / 1000, 1) + " · " + num(stats.wet.inertia.m[1][1] / 1000, 1) + " · " +
                                   num(stats.wet.inertia.m[2][2] / 1000, 1) + " t m²");
    kv("Ixy  ·  Ixz  ·  Iyz", num(stats.wet.inertia.m[0][1] / 1000, 2) + " · " + num(stats.wet.inertia.m[0][2] / 1000, 2) + " · " +
                                   num(stats.wet.inertia.m[1][2] / 1000, 2) + " t m²");
    kv("Principal moments", num(stats.wet.principal.x / 1000, 1) + " / " + num(stats.wet.principal.y / 1000, 1) + " / " +
                                num(stats.wet.principal.z / 1000, 1) + " t m²");
    y += 6;
    text("AERODYNAMICS", x, y, 12, col::faint, true);
    y += 22;
    kv("Frontal area (ascent)", num(stats.frontalArea, 2) + " m²");
    kv("Centre of pressure (Z)", num(stats.copAscent.z, 2) + " m", Color{96, 165, 250, 255});
    kv("Static margin", num(stats.staticMargin, 2) + " cal", stats.staticMargin >= 0.5 ? col::good : (stats.staticMargin >= 0 ? col::warn : col::bad));
    y += 6;
    text("PROPULSION", x, y, 12, col::faint, true);
    y += 22;
    kv("Total Δv (vacuum)", num(stats.totalDvVac, 0) + " m/s", col::accent);
    text("STAGE", x, y, 12, col::faint, true);
    text("Δv vac", x + 54, y, 12, col::faint, true);
    text("TWR", x + 132, y, 12, col::faint, true);
    text("burn", x + 180, y, 12, col::faint, true);
    text("|τ| thrust", x + 250, y, 12, col::faint, true);
    y += 18;
    for (auto& s : stats.stages) {
        text(std::to_string(s.stage + 1), x + 6, y, 14, col::text, true);
        if (s.hasEngines) {
            text(num(s.dvVac, 0), x + 54, y, 14, col::text);
            text(num(s.twrSL, 2), x + 132, y, 14, s.twrSL >= 1 ? col::good : col::dim);
            text(formatTime(s.burnTime), x + 180, y, 13, col::dim);
            double tq = len(s.thrustTorqueAtStart);
            text(tq < 1 ? "0" : num(tq / 1000, 1) + " kN m", x + 250, y, 13, tq > 0.02 * s.thrustVac ? col::warn : col::dim);
        } else text("no engines", x + 54, y, 14, col::faint);
        y += 22;
    }
    y += 6;
    text("POWER", x, y, 12, col::faint, true);
    y += 22;
    kv("Battery", num(stats.batteryJ / 3.6e6, 2) + " kWh");
    kv("Generation  ·  solar @1 AU", num(stats.genW, 0) + " W  ·  " + num(stats.solarW1AU, 0) + " W");
    kv("Continuous draw", num(stats.drawW, 0) + " W");
    kv("Control torque", num(stats.torque, 0) + " N m");
    y += 8;
    for (auto& wmsg : stats.warnings) {
        icon(Icon::Info, {x, y + 1, 14, 14}, col::warn);
        y = wrap(wmsg, x + 20, y, w - 20, 13, col::warn) + 6;
    }
    (void)app;
    endScroll();
}

} // namespace

// ================================================================== frame
void cadFrame(App& app) {
    Craft& C = app.craft;
    float W = ui::SW(), H = ui::SH();
    bool modal = openModal || app.settingsOpen;
    ui::setInputBlocked(modal);
    selValidate((int)C.comps.size());
    // selection / tab / craft as the properties panel last showed them (pending field edits belong to these)
    const int panelSel = sel, panelTab = rightTab, panelGen = g_craftGen;
    const std::vector<int> panelSet = selSet;
    Vec3d centre;
    double radius;
    craftBounds(C, centre, radius);
    if (!camInit) {
        camInit = true;
        cam.fovy = 40;
        cam.minDist = 0.3;
        cam.maxDist = 5000;
        cam.homeYaw = -PI / 2 + 0.75;
        cam.homePitch = 0.4;
        cam.yaw = cam.homeYaw;
        cam.pitch = cam.homePitch;
        cam.target = centre;
        cam.dist = radius / std::sin(cam.fovy * 0.5 * DEG) * 1.15;
    }
    cam.update(GetFrameTime());

    // ---- background analysis (debounced)
    // names do not change the physics: leave them out so typing a name does not re-run the analysis
    std::string txt;
    {
        std::string full = craftToText(C);
        txt.reserve(full.size());
        size_t p = 0;
        while (p < full.size()) {
            size_t e = full.find('\n', p);
            if (e == std::string::npos) e = full.size();
            if (full.compare(p, 5, "name ") != 0 && full.compare(p, 6, "cname ") != 0) txt.append(full, p, e - p + 1);
            p = e + 1;
        }
    }
    if (txt != analysedText && GetTime() - lastChange > 0.25 && gAxis < 0) {
        analysedText = txt;
        auto comps = expandCraft(C);
        aero = comps.empty() ? nullptr : aeroTableCached(comps);
        stats = analyzeCraft(C, aero.get());
    }

    // ---- keyboard
    bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
    if (!modal && !keyboardCaptured()) {
        // bindings: Ctrl+Z / mouse back = undo, Ctrl+Shift+Z / mouse forward = redo (Ctrl+Y kept as a fixed alias)
        using keys::A;
        bool shiftK = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
        bool undoK = keys::pressed(A::Undo), redoK = !undoK && (keys::pressed(A::Redo) || (ctrl && !shiftK && IsKeyPressed(KEY_Y)));
        if (undoK && !undoStack.empty()) {
            redoStack.push_back(C);
            C = undoStack.back();
            g_craftGen++;
            undoStack.pop_back();
            selClear();
        }
        if (redoK && !redoStack.empty()) {
            undoStack.push_back(C);
            C = redoStack.back();
            g_craftGen++;
            redoStack.pop_back();
            selClear();
        }
        bool cutKey = (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && !IsKeyDown(KEY_LEFT_SHIFT) && !IsKeyDown(KEY_RIGHT_SHIFT) && IsKeyPressed(KEY_X);
        if (!selSet.empty() && (keys::pressed(A::Delete) || cutKey)) deleteSelection(C); // (Ctrl+X: cut = remove)
        if (!selSet.empty() && keys::pressed(A::Duplicate)) duplicateSelection(app, C, false);
        if (keys::pressed(A::SelectAll)) {
            selSet.clear();
            for (int i = 0; i < (int)C.comps.size(); i++) selSet.push_back(i);
            sel = selSet.empty() ? -1 : selSet.back();
        }
        if (keys::pressed(A::ToolMove)) tool = 1;
        if (keys::pressed(A::ToolRotate)) tool = 2;
        if (keys::pressed(A::ToolSelect)) tool = 0;
        if (IsKeyPressed(KEY_ESCAPE) && cam.tool == 0) selClear();
        if (keys::pressed(A::SaveDesign)) {
            std::string path = app.designDir + "\\" + sanitizeFileName(C.name) + ".sfc";
            app.setStatus(saveCraft(C, path) ? trf("Saved design to {0}", path) : tr("Could not save design"));
        }
    }

    ClearBackground(col::bg);
    Rectangle view{344, 64, W - 344 - 404, H - 64};

    // ---- 3D view input
    Vector2 m = GetMousePosition();
    bool inView = CheckCollisionPointRec(m, view) && !mouseOverUI() && !modal;
    auto fitFn = [&]() { cam.fit(centre, radius); };
    auto homeFn = [&]() { cam.home(centre, radius); };
    auto pivotPick = [&](const Vec3d& o, const Vec3d& d, Vec3d& hit) {
        double t;
        if (pickComponent(C, o, d, t) >= 0) { hit = o + d * t; return true; }
        return false;
    };
    bool cubeHover = !modal && CheckCollisionPointRec(m, nav::viewCubeRect(view));
    // gizmo hover / drag
    double gl = cam.dist * 0.18;
    Vec3d gP{};
    for (int i : selSet) gP += C.comps[i].pos;
    if (!selSet.empty()) gP = gP / (double)selSet.size();
    gHover = -1;
    if (!selSet.empty() && tool != 0 && gAxis < 0 && inView && !cubeHover) {
        double best = 9;
        for (int a = 0; a < 3; a++) {
            if (tool == 1) {
                Vector2 s0, s1;
                if (nav::project(cam, view, gP, s0) && nav::project(cam, view, gP + axisVec(a) * gl, s1)) {
                    double d = distToSeg(m, s0, s1);
                    if (d < best) { best = d; gHover = a; }
                }
            } else {
                Vec3d u = anyPerp(axisVec(a)), v2 = cross(axisVec(a), u);
                Vector2 prev{};
                bool havePrev = false;
                for (int k = 0; k <= 48; k++) {
                    double t = TAU * k / 48;
                    Vector2 s;
                    if (!nav::project(cam, view, gP + (u * std::cos(t) + v2 * std::sin(t)) * gl, s)) { havePrev = false; continue; }
                    if (havePrev) {
                        double d = distToSeg(m, prev, s);
                        if (d < best) { best = d; gHover = a; }
                    }
                    prev = s;
                    havePrev = true;
                }
            }
        }
    }
    if (gHover >= 0 && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && cam.tool == 0) {
        gAxis = gHover;
        pushUndo(C);
        gStartPos = gP;
        gPivot0 = gP;
        gStartPosAll.clear();
        gStartRotAll.clear();
        for (int i : selSet) {
            gStartPosAll.push_back(C.comps[i].pos);
            gStartRotAll.push_back(C.comps[i].rotDeg);
        }
        Vec3d o, d;
        nav::viewRay(cam, view, m, o, d);
        gStartParam = axisParam(gP, axisVec(gAxis), o, d);
        Vector2 sp;
        nav::project(cam, view, gP, sp);
        gStartAngle = std::atan2(m.y - sp.y, m.x - sp.x);
    }
    if (gAxis >= 0) {
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT) || selSet.empty() || gStartPosAll.size() != selSet.size()) {
            gAxis = -1;
            dirty = true;
            lastChange = GetTime();
        } else {
            Vec3d A = axisVec(gAxis);
            if (tool == 1) {
                Vec3d o, d;
                nav::viewRay(cam, view, m, o, d);
                double s = axisParam(gStartPos, A, o, d) - gStartParam;
                // snap the moved coordinate of the primary (or the pivot for groups)
                Vec3d ref = selSet.size() == 1 ? gStartPosAll[0] : gPivot0;
                double* rc = gAxis == 0 ? &ref.x : gAxis == 1 ? &ref.y : &ref.z;
                double moved = snapLen(*rc + s) - *rc;
                for (size_t j = 0; j < selSet.size(); j++) C.comps[selSet[j]].pos = gStartPosAll[j] + A * moved;
            } else {
                Vector2 sp;
                nav::project(cam, view, gStartPos, sp);
                double ang = std::atan2(m.y - sp.y, m.x - sp.x) - gStartAngle;
                Vec3d fwd, right, up;
                cam.basis(fwd, right, up);
                if (dot(A, -fwd) < 0) ang = -ang;
                double deg = snapAng(-ang / DEG);
                Mat3d Rg = Quatd::axisAngle(A, deg * DEG).toMat();
                for (size_t j = 0; j < selSet.size(); j++) {
                    Component& k = C.comps[selSet[j]];
                    Component tmp = k;
                    tmp.rotDeg = gStartRotAll[j];
                    k.rotDeg = eulerFromMat(Rg * tmp.rot().toMat());
                    k.pos = gPivot0 + Rg * (gStartPosAll[j] - gPivot0);
                }
            }
            ui::blockRect(view);
        }
    }
    bool navigating = nav::handleMouse(cam, view, inView && !cubeHover && gAxis < 0 && gHover < 0, pivotPick, fitFn);
    bool addMod = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT) || ctrl;
    if (inView && !cubeHover && !navigating && gAxis < 0 && gHover < 0 && cam.tool == 0 && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vec3d o, d;
        nav::viewRay(cam, view, m, o, d);
        double t;
        pressComp = pickComponent(C, o, d, t);
        boxing = true;
        boxStart = m;
    }
    Rectangle boxR{};
    bool boxCrossing = false;
    if (boxing) {
        boxR = {std::min(boxStart.x, m.x), std::min(boxStart.y, m.y), std::fabs(m.x - boxStart.x), std::fabs(m.y - boxStart.y)};
        boxCrossing = m.x < boxStart.x; // Fusion: right-to-left = crossing, left-to-right = window
        ui::blockRect(view);
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            boxing = false;
            if (boxR.width < 4 && boxR.height < 4) {
                // plain click
                if (addMod) { if (pressComp >= 0) selToggle(pressComp); }
                else selOnly(pressComp);
            } else {
                if (!addMod) selClear();
                for (int i = 0; i < (int)C.comps.size(); i++) {
                    Craft one;
                    one.comps.push_back(C.comps[i]);
                    int inside = 0, total = 0;
                    for (auto& e : expandCraft(one)) {
                        std::vector<Vec3d> pts;
                        shapeContactPoints(e.shape, e.dims, pts);
                        pts.push_back({});
                        Mat3d Rr = e.rot().toMat();
                        for (auto& q : pts) {
                            Vector2 sp;
                            total++;
                            if (nav::project(cam, view, e.pos + Rr * q, sp) && CheckCollisionPointRec(sp, boxR)) inside++;
                        }
                    }
                    bool hit = boxCrossing ? inside > 0 : (total > 0 && inside == total);
                    if (hit && !isSel(i)) {
                        selSet.push_back(i);
                        sel = i;
                    }
                }
            }
        }
    }

    // ---- 3D rendering
    DrawRectangleGradientV((int)view.x, (int)view.y, (int)view.width, (int)view.height, Color{30, 38, 52, 255}, Color{12, 15, 21, 255});
    nav::beginViewport3D(cam, view, 0.05, 20000);
    gfx::resetEnvironment(); // neutral studio light (the flight view leaves its sky, sunlight tint and flood lights)
    gfx::setSpotLights(0, nullptr, nullptr, {}, 1);
    // grid on the z = 0 plane
    {
        double step = GRID[gridIdx] > 0 ? std::max(GRID[gridIdx], 0.1) : 0.5;
        while (cam.dist / step > 120) step *= 5;
        int n = 60;
        Vec3d gc{std::round(cam.target.x / step) * step, std::round(cam.target.y / step) * step, 0};
        for (int i = -n; i <= n; i++) {
            bool major = (int)std::lround((gc.x / step) + i) % 5 == 0;
            Color lc = major ? Color{70, 82, 104, 255} : Color{44, 52, 68, 255};
            DrawLine3D(toF(gc + Vec3d{i * step, -n * step, 0}), toF(gc + Vec3d{i * step, n * step, 0}), lc);
            bool major2 = (int)std::lround((gc.y / step) + i) % 5 == 0;
            Color lc2 = major2 ? Color{70, 82, 104, 255} : Color{44, 52, 68, 255};
            DrawLine3D(toF(gc + Vec3d{-n * step, i * step, 0}), toF(gc + Vec3d{n * step, i * step, 0}), lc2);
        }
        double al = std::max(1.0, cam.dist * 0.08);
        DrawLine3D({0, 0, 0}, {(float)al, 0, 0}, Color{239, 68, 68, 255});
        DrawLine3D({0, 0, 0}, {0, (float)al, 0}, Color{74, 222, 128, 255});
        DrawLine3D({0, 0, 0}, {0, 0, (float)al}, Color{59, 130, 246, 255});
    }
    rlDrawRenderBatchActive();
    auto comps = expandCraft(C);
    // map the selection to the expanded list (all symmetry copies highlighted)
    std::vector<int> expIndex;
    for (int i = 0; i < (int)C.comps.size(); i++)
        for (int k = 0; k < std::clamp(C.comps[i].symmetry, 1, 12); k++) expIndex.push_back(i);
    Vec3d eye = nav::renderEye(cam);
    for (int pass = 0; pass < 2; pass++) {
        // pass 0: everything except the selection, pass 1: the selected component's copies
        std::vector<Component> part;
        for (size_t i = 0; i < comps.size(); i++)
            if (isSel(expIndex[i]) == (pass == 1)) part.push_back(comps[i]);
        if (part.empty()) continue;
        gfx::CraftVisual cv;
        cv.comps = &part;
        cv.highlight = -1;
        cv.xray = xray;
        cv.wire = wire;
        cv.legs = true;
        cv.editor = true;
        cv.bayT = 1; // (the payload bay shown open: its payload can be seen and placed)
        if (pass == 1) {
            // emissive highlight for all copies
            for (size_t i = 0; i < part.size(); i++) {
                cv.highlight = (int)i;
                std::vector<Component> one{part[i]};
                cv.comps = &one;
                cv.highlight = 0;
                gfx::drawCraft(cv, Mat3d{}, {}, {}, norm(Vec3d{0.45, -0.55, 0.7}), 1.0f, eye, 0.3f);
            }
        } else gfx::drawCraft(cv, Mat3d{}, {}, {}, norm(Vec3d{0.45, -0.55, 0.7}), 1.0f, eye, 0.3f);
    }
    // overlays
    rlDisableDepthTest();
    double mk = std::max(0.08, cam.dist * 0.012);
    if (showCoM && stats.wet.mass > 0) {
        DrawSphere(toF(stats.wet.com), (float)mk, Color{250, 204, 21, 255});
        DrawSphereWires(toF(stats.wet.com), (float)mk * 1.6f, 6, 8, Color{20, 20, 20, 255});
    }
    if (showCoP && stats.frontalArea > 0) DrawSphere(toF(stats.copAscent), (float)mk, Color{96, 165, 250, 255});
    if (showThrust) {
        for (size_t i = 0; i < comps.size(); i++) {
            if (comps[i].func != Func::Engine || comps[i].stage != (stats.stages.empty() ? 0 : stats.stages.front().stage)) continue;
            CompPhys p;
            computeCompPhys(comps[i], p);
            if (p.thrustVac <= 0) continue;
            Vec3d a = p.thrustPoint, b = a - p.thrustDir * std::max(1.5, cam.dist * 0.08);
            DrawLine3D(toF(a), toF(b), Color{251, 146, 60, 255});
            DrawSphere(toF(b), (float)mk * 0.6f, Color{251, 146, 60, 255});
        }
    }
    // gizmo
    if (!selSet.empty() && tool != 0) {
        for (int a = 0; a < 3; a++) {
            bool hot = a == gHover || a == gAxis;
            Color c = axisCol(a, hot);
            Vec3d A = axisVec(a);
            if (tool == 1) {
                DrawCylinderEx(toF(gP), toF(gP + A * (gl * 0.85)), (float)(gl * 0.012), (float)(gl * 0.012), 8, c);
                DrawCylinderEx(toF(gP + A * (gl * 0.85)), toF(gP + A * gl), (float)(gl * 0.05), 0.0f, 12, c);
            } else {
                Vec3d u = anyPerp(A), v2 = cross(A, u);
                for (int k = 0; k < 64; k++) {
                    double t0 = TAU * k / 64, t1 = TAU * (k + 1) / 64;
                    DrawLine3D(toF(gP + (u * std::cos(t0) + v2 * std::sin(t0)) * gl), toF(gP + (u * std::cos(t1) + v2 * std::sin(t1)) * gl), c);
                }
            }
        }
    }
    rlEnableDepthTest();
    nav::endViewport3D();
    if (!modal) nav::viewCube(cam, view, homeFn);
    if (boxing && (boxR.width >= 4 || boxR.height >= 4)) {
        Color bc = boxCrossing ? Color{74, 222, 128, 255} : Color{56, 189, 248, 255};
        DrawRectangleRec(boxR, Color{bc.r, bc.g, bc.b, 40});
        DrawRectangleLinesEx(boxR, 1.5f, bc);
    }
    // overlay labels
    {
        Vector2 sp;
        if (showCoM && stats.wet.mass > 0 && nav::project(cam, view, stats.wet.com, sp)) text("CoM", sp.x + 10, sp.y - 8, 13, Color{250, 204, 21, 255}, true);
        if (showCoP && stats.frontalArea > 0 && nav::project(cam, view, stats.copAscent, sp)) text("CoP", sp.x + 10, sp.y + 4, 13, Color{96, 165, 250, 255}, true);
    }

    // ---- view toolbar (top of the 3D view)
    {
        // one row when it fits beside the ViewCube, otherwise the snap buttons go to a second row
        char gb[32], ab[32];
        if (GRID[gridIdx] > 0) std::snprintf(gb, sizeof gb, "%.2f m", GRID[gridIdx]);
        else std::snprintf(gb, sizeof gb, "%s", tr("off").c_str());
        if (ANGLES[angleIdx] > 0) std::snprintf(ab, sizeof ab, "%.0f°", ANGLES[angleIdx]);
        else std::snprintf(ab, sizeof ab, "%s", tr("off").c_str());
        std::string gl = tr("Grid snap") + ": " + gb, al = tr("Angle snap") + ": " + ab;
        // fixed widths sized for the longest value so the toolbar does not jump while cycling
        float gw = std::max(150.0f, measure(tr("Grid snap") + ": 0.25 m", 16).x + 20);
        float aw = std::max(150.0f, measure(tr("Angle snap") + ": 45°", 16).x + 20);
        const float toolsW = 8 * 40 + 8, snapsW = gw + aw + 22;
        bool twoRows = toolsW + snapsW > view.width - 170;
        Rectangle tb{view.x + 10, view.y + 10, twoRows ? std::max(toolsW, snapsW - 8) : toolsW + snapsW, twoRows ? 86.0f : 46.0f};
        panel(tb, Color{18, 23, 32, 220}, 10);
        float bx = tb.x + 6;
        auto tbtn = [&](Icon ic, bool on, const char* tip) {
            bool r = iconButton({bx, tb.y + 6, 34, 34}, ic, on, tip);
            bx += 38;
            return r;
        };
        if (tbtn(Icon::Target, tool == 0, keys::tip("Select", keys::A::ToolSelect).c_str())) tool = 0;
        if (tbtn(Icon::Move, tool == 1, keys::tip("Move", keys::A::ToolMove).c_str())) tool = 1;
        if (tbtn(Icon::Rotate, tool == 2, keys::tip("Rotate", keys::A::ToolRotate).c_str())) tool = 2;
        bx += 6;
        if (tbtn(Icon::Xray, xray, "X-ray")) xray = !xray;
        if (tbtn(Icon::Wire, wire, "Wireframe")) wire = !wire;
        if (tbtn(Icon::Planet, showCoM, "Centre of mass")) showCoM = !showCoM;
        if (tbtn(Icon::Sail, showCoP, "Centre of pressure")) showCoP = !showCoP;
        if (tbtn(Icon::Flame, showThrust, "Thrust vectors")) showThrust = !showThrust;
        float sy = tb.y + 6;
        if (twoRows) {
            bx = tb.x + 6;
            sy = tb.y + 46;
        } else bx += 8;
        if (button({bx, sy, gw, 34}, gl)) gridIdx = (gridIdx + 1) % 7;
        bx += gw + 6;
        if (button({bx, sy, aw, 34}, al)) angleIdx = (angleIdx + 1) % 4;
    }
    if (!modal) nav::navBar(cam, {view.x + view.width / 2, view.y + view.height - 10}, fitFn, homeFn, true);
    text(fit("Right drag: orbit   Middle drag: pan   Wheel: zoom   Left drag: box select   Gizmo: edit", view.width / 2 - 150, 12),
         view.x + 16, view.y + view.height - 26, 12, col::faint);

    // ---- top bar
    DrawRectangle(0, 0, (int)W + 1, 56, Color{14, 18, 26, 255});
    DrawRectangle(0, 56, (int)W + 1, 1, col::line);
    icon(Icon::Cube, {18, 14, 28, 28}, col::accent);
    text("CAD Designer", 56, 10, 20, col::text, true);
    text(fit(trf("{0} components", std::to_string(C.comps.size())), 200, 14), 56, 32, 14, col::dim);
    static std::string nameBuf;
    static bool nameInit = false;
    if (!nameInit || (!keyboardCaptured() && nameBuf != C.name)) { nameBuf = C.name; nameInit = true; }
    float bx = W - 16;
    auto top = [&](float w) { bx -= w; Rectangle r{bx, 10, w, 36}; bx -= 8; return r; };
    // aircraft (rotors, jets or wheels and no rocket engine) start on a runway or at a landing site of
    // another world with an atmosphere
    bool aircraft = false, rocket = false, wheels = false;
    for (const Component& k : C.comps) {
        aircraft |= k.func == Func::Rotor || k.func == Func::Jet || k.func == Func::Wheel;
        wheels |= k.func == Func::Wheel;
        rocket |= k.func == Func::Engine || (k.func == Func::Jet && k.def >= 0 && worksWithoutAir(CATALOG.defs[k.def]));
    }
    aircraft &= !rocket;
    bool spaceplane = rocket && wheels; // winged rocket / combined-cycle craft: pad or runway
    bool hasPort = false;
    for (const Component& k : C.comps) hasPort |= k.func == Func::Docking;
    static int planeStart = 0, orbitStart = 0;
    if (!hasPort || aircraft || spaceplane) orbitStart = 0;
    // an orbital start next to the vessel flown last (a module delivered for assembly)
    const Vessel* nearV = nullptr;
    if (app.gameStarted)
        if (const Vessel* av = app.world.activeVessel())
            if (!av->destroyed && !av->landed) nearV = av;
    // where a rover or an aircraft starts: Earth, or a world with a surface base (beside the base)
    static int airBody = -1;
    std::vector<int> deploy = app.world.deployBodies();
    if (std::find(deploy.begin(), deploy.end(), airBody) == deploy.end()) airBody = deploy.front(); // (no base there any more: Earth)
    if (primaryButton(top(140), "LAUNCH", aircraft ? Icon::Play : Icon::Rocket, !C.comps.empty() && !modal)) {
        if (!app.gameStarted) {
            app.world.newGame(app.world.settings);
            app.gameStarted = true;
        }
        app.lastLaunched = C;
        if (orbitStart == 1) {
            app.lastLaunchBody = -3;
            app.lastLaunchNear = nearV ? nearV->id : -1;
            app.world.launchInOrbit(C, app.lastLaunchNear, 120);
        } else if (aircraft || (spaceplane && planeStart == 1)) {
            int body = aircraft ? airBody : app.world.U.find("Earth");
            app.lastLaunchBody = body;
            app.world.launchAircraft(C, body);
        } else {
            app.lastLaunchBody = -2;
            app.world.launch(C);
        }
        flightOnEnter(app);
        app.screen = Screen::Flight;
        return;
    }
    if (hasPort && !aircraft && !spaceplane) {
        Rectangle sr = top(230);
        if (button(sr, orbitStart == 0 ? tr("Start: launch pad") : tr("Start: in orbit"), false, !modal, orbitStart == 0 ? Icon::Rocket : Icon::Planet))
            orbitStart = 1 - orbitStart;
        if (CheckCollisionPointRec(GetMousePosition(), sr))
            tooltip(nearV ? trf("In orbit: 120 m behind {0} in the same orbit, with it as the target (a module delivered for assembly). Click to change.", nearV->name)
                          : tr("In orbit: a 400 km orbit (fly a vessel first to start next to it). Click to change."));
    }
    if (spaceplane) {
        Rectangle sr = top(200);
        if (button(sr, planeStart == 0 ? tr("Start: launch pad") : tr("Start: runway"), false, !modal, planeStart == 0 ? Icon::Rocket : Icon::Play))
            planeStart = 1 - planeStart;
        if (CheckCollisionPointRec(GetMousePosition(), sr))
            tooltip(tr("Launch pad: stands upright on the pad (a winged orbiter on its rocket). Runway: takes off horizontally (a single-stage spaceplane). Click to change."));
    }
    if (aircraft) {
        Rectangle sr = top(170);
        auto at = std::find(deploy.begin(), deploy.end(), airBody);
        int k = at == deploy.end() ? 0 : (int)(at - deploy.begin());
        if (button(sr, trf("Start on: {0}", tr(app.world.U.bodies[airBody].name)), false, !modal && deploy.size() > 1, Icon::Planet))
            airBody = deploy[(k + 1) % deploy.size()];
        if (CheckCollisionPointRec(GetMousePosition(), sr))
            tooltip(deploy.size() > 1 ? tr("Earth: the runway beside the launch complex. Other worlds: beside your surface base there. Click to change.")
                                      : tr("Earth: the runway beside the launch complex. To start on another world, build a surface base there first."));
    }
    if (!aircraft && orbitStart == 0 && !(spaceplane && planeStart == 1)) {
        // the launch site (the same setting as the factory's): click for the next, right-click for the previous
        World& LW = app.world;
        std::string siteName = tr(LW.launchSiteName());
        bool atBase = LW.padSite().baseId > 0;
        std::string lb = trf("Site: {0}", siteName);
        // (as wide as the bar allows: the buttons still to come and a name box of 150 to its left)
        float rest = 150 + 8 + 40 * 4 + 96 * 2 + 86 + 8 * 8 + 150 + 8, room = bx - 250 - rest;
        float want = std::clamp(measure(lb, 16).x + 56, 170.0f, 300.0f);
        if (room < want) lb = fit(siteName, std::max(room, 44.0f) - 50, 16);
        Rectangle sr = top(std::clamp(std::min(want, room), 44.0f, 300.0f));
        if (button(sr, lb, atBase, !modal, atBase ? Icon::Base : Icon::Target)) LW.cycleLaunchSite(1);
        if (CheckCollisionPointRec(GetMousePosition(), sr) && !modal) {
            if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) LW.cycleLaunchSite(-1);
            std::string all;
            for (int i = 0; i < LAUNCH_SITE_COUNT; i++) all += (i ? "  ·  " : "") + tr(LAUNCH_SITES[i].name);
            for (int id : LW.launchBaseIds()) all += "  ·  " + LW.bases[findBase(LW, id)].name;
            tooltip(tr("Launch site (click: next, right-click: previous)") + ":  " + all);
        }
    }
    if (button(top(150), "Back to factory", false, !modal, Icon::Factory)) { app.screen = Screen::Factory; return; }
    if (iconButton(top(40), Icon::Gear, false, "Settings", !modal)) app.settingsOpen = true;
    if (iconButton(top(40), Icon::Home, false, "Home screen", !modal)) {
        app.screen = Screen::Menu;
        return;
    }
    if (iconButton(top(40), Icon::Redo, false, keys::tip("Redo", keys::A::Redo).c_str(), !redoStack.empty() && !modal)) {
        undoStack.push_back(C);
        C = redoStack.back();
        g_craftGen++;
        redoStack.pop_back();
        selClear();
    }
    if (iconButton(top(40), Icon::Undo, false, keys::tip("Undo", keys::A::Undo).c_str(), !undoStack.empty() && !modal)) {
        redoStack.push_back(C);
        C = undoStack.back();
        g_craftGen++;
        undoStack.pop_back();
        selClear();
    }
    if (button(top(96), "Save", false, !C.comps.empty() && !modal, Icon::Save)) {
        std::string path = app.designDir + "\\" + sanitizeFileName(C.name) + ".sfc";
        app.setStatus(saveCraft(C, path) ? trf("Saved design to {0}", path) : tr("Could not save design"));
    }
    if (button(top(96), "Open", false, !modal, Icon::Folder)) {
        refreshFiles(app);
        openModal = true;
        openMode = 0;
    }
    if (CheckCollisionPointRec(GetMousePosition(), Rectangle{bx + 8, 10, 96, 36}) && !modal)
        tooltip(tr("Open a design, or add one to the current design"));
    if (button(top(86), "New", false, !modal, Icon::Plus)) {
        pushUndo(C);
        C = Craft{};
        g_craftGen++;
        selClear();
    }
    // the design's name in the room the buttons leave (a narrow window: shorter, never under them)
    if (textInput({250, 11, std::clamp(bx - 250, 100.0f, 240.0f), 34}, nameBuf, 40, 3001)) C.name = nameBuf;

    // ---- left: palette + component list
    Rectangle L{12, 68, 320, H - 80};
    panel(L);
    text("ADD", L.x + 16, L.y + 14, 13, col::dim, true);
    const char* cats[] = {"Shapes", "Command", "Engines", "Systems", "Aircraft"};
    float cx = L.x + 14, cy = L.y + 36;
    for (int i = 0; i < 5; i++) {
        float cw = measure(tr(cats[i]), 14, true).x + 22;
        if (cx + cw > L.x + L.width - 12) { cx = L.x + 14; cy += 32; } // (a second row when the names are long)
        if (chip({cx, cy, cw, 26}, tr(cats[i]), paletteCat == i) && !modal) paletteCat = i;
        cx += cw + 6;
    }
    auto items = paletteItems(paletteCat);
    float listTop = cy + 34;
    float listH = (L.height - 80) * 0.52f - (listTop - (L.y + 70));
    Rectangle pr{L.x + 8, listTop, L.width - 16, listH};
    float off = beginScroll(320, pr, items.size() * 46.0f);
    float y = pr.y - off;
    for (auto& it : items) {
        Rectangle row{pr.x, y, pr.width - 8, 42};
        if (y + 42 >= pr.y && y <= pr.y + pr.height) {
            if (rowItem(row, false) && !modal) addComponent(app, it.make());
            icon(it.ic, {row.x + 10, row.y + 11, 20, 20}, col::accent);
            text(fit(it.label, row.width - 50, 14, true), row.x + 40, row.y + 4, 14, col::text, true);
            text(fit(it.sub, row.width - 50, 12), row.x + 40, row.y + 23, 12, col::dim);
        }
        y += 46;
    }
    endScroll();
    float ly = pr.y + pr.height + 10;
    divider(L.x + 14, ly, L.width - 28);
    text("COMPONENTS", L.x + 16, ly + 8, 13, col::dim, true);
    textR(fit(tr("Check boxes / Ctrl / Shift: select several"), L.width - 150, 11), L.x + L.width - 14, ly + 10, 11, col::faint);
    Rectangle cr{L.x + 8, ly + 32, L.width - 16, L.y + L.height - ly - 40};
    off = beginScroll(321, cr, C.comps.size() * 30.0f);
    y = cr.y - off;
    for (int i = 0; i < (int)C.comps.size(); i++) {
        const Component& k = C.comps[i];
        Rectangle row{cr.x, y, cr.width - 8, 28};
        if (y + 28 >= cr.y && y <= cr.y + cr.height) {
            // the check box at the row's start adds / removes the part without a modifier key
            Rectangle box{row.x + 6, row.y + 6, 16, 16};
            if (rowItem(row, isSel(i)) && !modal) {
                bool boxHit = GetMousePosition().x < row.x + 28;
                bool sh = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
                if (boxHit) selToggle(i);
                else if (sh && sel >= 0) {
                    int a = std::min(sel, i), b = std::max(sel, i);
                    for (int j = a; j <= b; j++)
                        if (!isSel(j)) selSet.push_back(j);
                    sel = i;
                } else if (ctrl) selToggle(i);
                else selOnly(i);
            }
            DrawRectangleRoundedLinesEx(box, 0.3f, 4, 1.5f, isSel(i) ? col::accent : col::dim);
            if (isSel(i)) {
                DrawRectangleRounded({box.x + 3, box.y + 3, box.width - 6, box.height - 6}, 0.3f, 4, col::accent);
            }
            DrawRectangleRounded({row.x + 28, row.y + 9, 10, 10}, 0.4f, 4, k.color);
            text(fit(tr(k.name), row.width - 128, 13), row.x + 44, row.y + 6, 13, col::text, isSel(i));
            std::string tag = "S" + std::to_string(k.stage + 1) + (k.symmetry > 1 ? "  ×" + std::to_string(k.symmetry) : "");
            textR(tag, row.x + row.width - 8, row.y + 7, 12, col::dim);
        }
        y += 30;
    }
    endScroll();
    if (C.comps.empty()) wrap("Empty craft: add parts from the palette above.", cr.x + 8, cr.y + 8, cr.width - 16, 13, col::faint);

    // ---- right: properties / analysis
    Rectangle Rr{W - 392, 68, 380, H - 80};
    panel(Rr);
    int newTab = rightTab;
    if (chip({Rr.x + 14, Rr.y + 12, 130, 28}, "Properties", rightTab == 0)) newTab = 0;
    if (chip({Rr.x + 150, Rr.y + 12, 130, 28}, "Analysis", rightTab == 1)) newTab = 1;
    Rectangle body{Rr.x, Rr.y + 48, Rr.width, Rr.height - 52};
    if (ui::editing() && (sel != panelSel || selSet != panelSet || newTab != panelTab || g_craftGen != panelGen)) {
        // the click that changed the selection also ends the edit: run the old panel once (invisibly)
        // so the field commits to the component it was editing, never to the newly selected one
        if (g_craftGen == panelGen && panelTab == 0 && panelSel >= 0 && panelSel < (int)C.comps.size()) {
            int s = sel;
            std::vector<int> ss = selSet;
            sel = panelSel;
            selSet = panelSet;
            beginScroll(9002, {0, 0, 0, 0}, 0); // zero-size clip: nothing drawn, nothing hovered
            propertiesPanel(app, body);
            endScroll();
            sel = s;
            selSet = ss;
        }
        ui::cancelEdit();
    }
    rightTab = newTab;
    if (rightTab == 0) propertiesPanel(app, body);
    else analysisPanel(app, body);

    // status
    if (GetTime() - app.statusTime < 4.0 && !app.status.empty()) {
        Vector2 sz = measure(app.status, 15);
        Rectangle s{view.x + (view.width - sz.x - 32) / 2, view.y + 66, sz.x + 32, 34};
        DrawRectangleRounded(s, 0.5f, 8, Color{20, 30, 44, 235});
        textC(app.status, s, 15, col::text);
    }

    // ---- open modal
    ui::setInputBlocked(app.settingsOpen);
    if (openModal) {
        dim();
        float mw = 760, mh = 560;
        Rectangle r{(W - mw) / 2, (H - mh) / 2, mw, mh};
        panel(r, Color{20, 26, 36, 252}, 14);
        text(openMode == 0 ? "Open design" : openMode == 2 ? "Load a payload" : "Add a design", r.x + 24, r.y + 18, 24, col::text, true);
        if (iconButton({r.x + mw - 48, r.y + 16, 32, 32}, Icon::Close) || IsKeyPressed(KEY_ESCAPE)) openModal = false;
        // what a click on a design does: open it in place of the current one, or add its parts to the current one
        {
            float cx = r.x + 24, cy = r.y + 60;
            const char* ml[3] = {"Open (replace)", "Add to current design", "Load into the payload bay"};
            bool hasBay = false;
            for (const Component& k : C.comps) hasBay |= k.func == Func::CargoBay;
            if (!hasBay && openMode == 2) openMode = 1;
            for (int i = 0; i < (hasBay ? 3 : 2); i++) {
                float cw = measure(tr(ml[i]), 13, true).x + 24;
                if (chip({cx, cy, cw, 28}, ml[i], openMode == i)) openMode = i;
                cx += cw + 8;
            }
            if (openMode == 1)
                text(fit(tr("Placed on top of the current design along its axis, and selected"), r.x + mw - 24 - cx, 12), cx + 6, cy + 7, 12, col::faint);
            if (openMode == 2) text(fit(tr("Set down on the bay's floor as its payload"), r.x + mw - 24 - cx, 12), cx + 6, cy + 7, 12, col::faint);
        }
        float colW = (mw - 72) / 2;
        float listTop = r.y + 100;
        text("TEMPLATES", r.x + 24, listTop, 13, col::dim, true);
        text("SAVED DESIGNS", r.x + 48 + colW, listTop, 13, col::dim, true);
        // a chosen design: replaces the current one or is added to it
        auto take = [&](const Craft& t, const std::string& loadedMsg) {
            if (openMode == 2) {
                if (!importIntoBay(app, t)) return;
            } else if (openMode == 1) {
                if (!importCraft(app, t)) return;
            } else {
                pushUndo(C);
                C = t;
                g_craftGen++;
                selClear();
                ensureGlyphs(C.name);
                for (const Component& k : C.comps) ensureGlyphs(k.name);
                app.setStatus(loadedMsg);
            }
            openModal = false;
            Vec3d cc;
            double rr;
            craftBounds(C, cc, rr);
            cam.home(cc, rr);
        };
        std::vector<Craft> tpl = builtinCrafts();
        for (auto& d : builtinTemplates()) tpl.push_back(craftFromDesign(d));
        Rectangle tr0{r.x + 24, listTop + 24, colW, r.y + mh - 24 - (listTop + 24)};
        float o2 = beginScroll(330, tr0, tpl.size() * 46.0f);
        float yy = tr0.y - o2;
        for (auto& t : tpl) {
            Rectangle row{tr0.x, yy, colW - 8, 42};
            if (rowItem(row, false)) take(t, trf("Loaded template: {0}", t.name));
            icon(Icon::Cube, {row.x + 10, row.y + 11, 20, 20}, col::accent);
            text(fit(t.name, colW - 60, 15), row.x + 40, row.y + 11, 15, col::text);
            yy += 46;
        }
        endScroll();
        Rectangle lr{r.x + 48 + colW, listTop + 24, colW, r.y + mh - 24 - (listTop + 24)};
        o2 = beginScroll(331, lr, files.size() * 46.0f);
        yy = lr.y - o2;
        std::string toDelete;
        for (auto& sf : files) {
            const std::string& f = sf.path;
            Rectangle row{lr.x, yy, colW - 46, 42};
            std::string nm = sf.name.empty() ? std::filesystem::path(f).stem().string() : sf.name; // (the name as saved)
            if (rowItem(row, false)) {
                Craft cc;
                if (loadCraft(cc, f)) take(cc, trf("Loaded {0}", cc.name));
                else app.setStatus(trf("Could not read {0}", nm));
            }
            text(fit(nm, colW - 70, 15), row.x + 12, row.y + 12, 15, col::text);
            static std::string deleteArmed; // first click arms, second click deletes
            bool armed = deleteArmed == f;
            if (armed) textR("Click again to delete", lr.x + colW - 48, yy + 14, 12, col::bad);
            if (iconButton({lr.x + colW - 40, yy + 5, 32, 32}, armed ? Icon::Check : Icon::Trash, armed, armed ? "Click again to delete" : "Delete")) {
                if (armed) { toDelete = f; deleteArmed.clear(); }
                else deleteArmed = f;
            }
            yy += 46;
        }
        endScroll();
        if (files.empty()) text("No saved designs yet", lr.x + 12, lr.y + 12, 15, col::faint);
        if (!toDelete.empty()) {
            std::error_code ec;
            auto p = std::filesystem::weakly_canonical(toDelete, ec);
            auto base = std::filesystem::weakly_canonical(app.designDir, ec);
            if (!ec && p.parent_path() == base && p.extension() == ".sfc") std::filesystem::remove(p, ec);
            refreshFiles(app);
        }
    }
    ui::setInputBlocked(false);
    if (app.settingsOpen) settingsModal(app);
}

// test hook: select a component and right-panel tab
void cadDebugSet(int s, int tab) {
    selOnly(s);
    rightTab = tab;
}

void cadDebugPalette(int cat) { paletteCat = std::clamp(cat, 0, 4); } // (autotest)
// (autotest) the CAD's own operations on the current craft: select all / these components, Duplicate, Mirror X,
// add a design, the open dialog in a mode
void cadDebugSelect(const std::vector<int>& s) {
    selSet = s;
    sel = s.empty() ? -1 : s.back();
}
void cadDebugDuplicate(App& app, bool mirror) { duplicateSelection(app, app.craft, mirror); }
bool cadDebugImport(App& app, const Craft& c) { return importCraft(app, c); }
bool cadDebugFairing(App& app) { return encloseInFairing(app, app.craft); }
bool cadDebugIntoBay(App& app, const Craft& c) { return importIntoBay(app, c); }
std::vector<int> cadDebugSelection() { return selSet; }
void cadDebugHome(App& app) {
    Vec3d cc;
    double rr;
    craftBounds(app.craft, cc, rr);
    cam.home(cc, rr);
}
void cadDebugOpenDialog(App& app, int mode) {
    refreshFiles(app);
    openModal = mode >= 0;
    openMode = std::max(mode, 0);
}
void cadSetCraft(App& app, const Craft& c) {
    if (!app.craft.comps.empty()) {
        pushUndo(app.craft); // the previous CAD craft stays recoverable with Undo
        app.setStatus("The previous CAD craft can be restored with Undo (Ctrl+Z)");
    }
    app.craft = c;
    g_craftGen++;
    selClear();
    ui::cancelEdit();
}
