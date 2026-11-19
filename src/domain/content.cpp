// Built-in content: the template designs of the factory and the example crafts of the CAD designer (game data)
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "content.hpp"
#include "i18n.hpp"

namespace {
struct B {
    Design d;
    B& add(const char* id, double diam = 0, double len = 0, int cluster = 1, double diam2 = 0) {
        int def = CATALOG.find(id);
        if (def < 0) return *this;
        PartInst p = makeInst(def);
        if (diam > 0) { p.diam = diam; p.diam2 = diam; }
        if (diam2 > 0) p.diam2 = diam2;
        if (len > 0) p.length = len;
        p.cluster = cluster;
        d.stack.push_back(p);
        return *this;
    }
    // payload fairing: open() at its stack position, close() after the last part it covers (the skirt
    // length becomes the height of those parts)
    int fairingAt = -1;
    B& fairing(double diam) {
        add("fairing", diam);
        fairingAt = (int)d.stack.size() - 1;
        return *this;
    }
    B& endFairing() {
        if (fairingAt < 0) return *this;
        double h = 0;
        for (size_t i = fairingAt + 1; i < d.stack.size(); i++) h += partGeom(d.stack[i]).height;
        d.stack[fairingAt].length = h;
        fairingAt = -1;
        return *this;
    }
    B& rad(const char* id, int count) {
        int def = CATALOG.find(id);
        if (def >= 0 && !d.stack.empty()) d.stack.back().radial.push_back({def, count});
        return *this;
    }
};
} // namespace

std::vector<Design> builtinTemplates() {
    std::vector<Design> out;
    {
        B b;
        b.d.name = "Kestrel Orbiter";
        // proportions after Falcon 9: 3.7 m core, nine engines on a 36 m first stage, a 5.2 m fairing and a
        // second stage of 6 m + its engine (the real tank is ~9 m; shorter here so a stage flown prograde, as the
        // tutorial teaches, keeps a thrust-to-weight of 1.5 instead of a steeply lofted, lossy climb)
        b.fairing(5.2).add("probe_ring", 3.7).rad("solar_s", 2).rad("bat_s", 1).endFairing();
        b.add("tank_kerolox", 3.7, 6).add("eng_kv981", 3.7);
        b.add("decoupler", 3.7).add("tank_kerolox", 3.7, 36).add("eng_kb845", 3.7, 0, 9);
        out.push_back(b.d);
    }
    {
        B b;
        b.d.name = "Aurora Crew Mission";
        b.fairing(5.2).add("chute_cluster").add("capsule3").add("shield", 3.9);
        b.add("decoupler", 3.9).add("tank_hyper", 3.9, 0.8).rad("solar_m", 2).add("eng_ho27", 3.9, 0, 2).endFairing();
        b.add("decoupler", 3.9).add("tank_kerolox", 3.9, 10).add("eng_kv981", 3.9);
        b.add("decoupler", 3.9).add("tank_kerolox", 3.9, 40).rad("srb_2", 2).add("eng_kb845", 3.9, 0, 9);
        out.push_back(b.d);
    }
    {
        B b;
        b.d.name = "Selene Lander";
        b.fairing(4.4).add("probe_ring", 2.5).rad("solar_s", 3).rad("bat_s", 2);
        b.add("tank_hyper", 2.5, 1.8).rad("legs", 4).add("eng_hl45", 2.5);
        b.add("decoupler", 2.5).add("adapter", 2.5, 0, 1, 3.7).endFairing();
        b.add("tank_hydrolox", 3.7, 10).add("eng_hu110", 3.7, 0, 2); // like the S-IVB: orbit insertion and the burn to the Moon
        b.add("decoupler", 3.7).add("tank_kerolox", 3.7, 34).add("eng_kb845", 3.7, 0, 9);
        out.push_back(b.d);
    }
    {
        B b;
        b.d.name = "Ion Explorer";
        b.fairing(4.0).add("probe_bus").rad("solar_m", 2).rad("bat_s", 2);
        b.add("tank_xenon", 1.25, 1.2).add("eng_ht5", 1.25, 0, 4);
        b.add("decoupler", 1.25).add("adapter", 1.25, 0, 1, 3.7).endFairing();
        b.add("tank_hydrolox", 3.7, 12).add("eng_hu110", 3.7, 0, 2);
        b.add("decoupler", 3.7).add("tank_kerolox", 3.7, 30).add("eng_kb845", 3.7, 0, 7);
        out.push_back(b.d);
    }
    {
        B b;
        b.d.name = "Lightsail Pathfinder";
        b.fairing(2.8).add("sail_m").add("probe_s").rad("solar_s", 2).rad("bat_s", 1);
        b.add("decoupler", 0.8).add("adapter", 0.8, 0, 1, 2.5).endFairing();
        b.add("tank_hydrolox", 2.5, 6).add("eng_hu110", 2.5);
        b.add("decoupler", 2.5).add("tank_kerolox", 2.5, 18).add("eng_kb845", 2.5, 0, 2);
        out.push_back(b.d);
    }
    {
        B b;
        b.d.name = "Nuclear Voyager";
        b.fairing(5.4).add("probe_ring", 3.7).rad("solar_m", 2).rad("rtg", 2);
        b.add("tank_lh2", 3.7, 14).add("eng_nt333", 3.7).endFairing();
        b.add("decoupler", 3.7).add("tank_methalox", 3.7, 16).add("eng_mv2700", 3.7);
        b.add("decoupler", 3.7).add("tank_methalox", 3.7, 42).add("eng_mh2600", 3.7, 0, 4);
        out.push_back(b.d);
    }
    {
        B b;
        b.d.name = "Hyperspace Explorer HX";
        b.fairing(4.6).add("probe_bus").rad("solar_m", 2).rad("rtg", 2).rad("bat_l", 1);
        b.add("hyperdrive").add("tank_hyper", 2.5, 1.5).rad("legs", 4).add("eng_hl45", 2.5, 0, 2);
        b.add("decoupler", 2.5).add("adapter", 2.5, 0, 1, 3.7).endFairing();
        b.add("tank_kerolox", 3.7, 8).add("eng_kv981", 3.7);
        b.add("decoupler", 3.7).add("tank_kerolox", 3.7, 36).add("eng_kb845", 3.7, 0, 9);
        out.push_back(b.d);
    }
    return out;
}

namespace {
// a point of a fuselage's (Shape::Loft) surface and the outward normal there (craft frame, the fuselage unrotated):
// zl = position along it from its centre [m], th = angle around the section (0: +X, 90 degrees: the top)
void loftSurface(const Component& L, double zl, double thDeg, Vec3d& p, Vec3d& n) {
    auto at = [&](double z, double th) {
        double nose = std::clamp(L.noseLen, 0.02, 0.6), tail = std::clamp(L.tailLen, 0.02, 0.7);
        if (nose + tail > 0.98) { double k = 0.98 / (nose + tail); nose *= k; tail *= k; }
        double expo = 2.0 / (2.0 + 2.5 * std::clamp(L.boxy, 0.0, 1.0));
        double s = std::clamp(z / L.dims.z + 0.5, 0.0, 1.0), yc = 0, f = 1;
        if (s > 1 - nose) {
            double t = (1 - s) / nose, e = 1.0 - 0.5 * std::clamp(L.bluff, 0.0, 1.0);
            f = std::pow(std::max(0.0, 1 - (1 - t) * (1 - t)), e);
        } else if (s < tail) {
            double t = s / tail, sm = t * t * (3 - 2 * t);
            yc = L.upsweep * 0.5 * (1 - sm) * (1 - std::clamp(L.tailR, 0.0, 1.0) * 0.5);
            f = L.tailR + (1 - L.tailR) * std::pow(sm, 0.75);
        }
        double c = std::cos(th * DEG), sn = std::sin(th * DEG);
        double x = 0.5 * f * (c < 0 ? -1 : 1) * std::pow(std::fabs(c), expo);
        double y = 0.5 * f * (sn < 0 ? -1 : 1) * std::pow(std::fabs(sn), expo) * (1 - std::fabs(yc)) + yc * 0.5;
        return Vec3d{x * L.dims.x, y * L.dims.y, z} + L.pos;
    };
    p = at(zl, thDeg);
    Vec3d dz = at(zl + 0.02, thDeg) - at(zl - 0.02, thDeg), dt = at(zl, thDeg + 0.5) - at(zl, thDeg - 0.5);
    n = norm(cross(dt, dz));
    if (dot(n, p - Vec3d{L.pos.x, L.pos.y, p.z}) < 0) n = n * -1.0;
}
// seat a wing panel's root in the parts already placed: extend the panel inward along its leading and trailing
// edges (its tip stays where it is) or, extend = false, move it inward whole, until its whole root chord lies
// 3 cm inside them (no slit between the wing and the fuselage it is joined to)
Component seatRoot(Component k, const std::vector<Component>& body, bool extend) {
    if (k.shape != Shape::Airfoil) return k;
    k.rootSide = airfoilRoot(k);
    const double rs = k.rootSide > 0 ? 1.0 : -1.0, sw = std::clamp(k.sweep, -80.0, 80.0) * DEG;
    auto yShift = [&](double s, double crA, double ctA) {
        double num = 0, den = 0;
        for (int i = 0; i < 32; i++) {
            double u = (i + 0.5) / 32, c = crA + (ctA - crA) * u;
            num += c * (u * std::tan(sw) * s + c / 2);
            den += c;
        }
        return -num / den;
    };
    for (int it = 0; it < 80; it++) {
        std::vector<Vec3d> pts;
        airfoilRootEdge(k, pts, 13);
        bool seated = true;
        for (const Vec3d& p : pts) {
            bool in = false;
            for (const Component& o : body) in = in || pointInShape(o, p, -0.03);
            seated = seated && in;
        }
        if (seated) break;
        const double dl = 0.05;
        Mat3d R = k.rot().toMat();
        if (!extend) {
            k.pos = k.pos + R * Vec3d{rs * dl, 0, 0};
            continue;
        }
        double s = k.dims.x, tp = std::clamp(k.taper, 0.1, 1.0);
        double crA = 2 / (1 + tp) * k.dims.y, ctA = tp * crA;
        double s2 = s + dl, crA2 = crA + dl * (crA - ctA) / s, tp2 = ctA / crA2;
        double y0 = yShift(s, crA, ctA), y1 = yShift(s2, crA2, ctA);
        k.pos = k.pos + R * Vec3d{rs * dl / 2, y0 - std::tan(sw) * dl - y1, 0};
        k.dims.x = s2;
        k.dims.y = crA2 * (1 + tp2) / 2;
        k.taper = tp2;
    }
    return k;
}
// a window lying on the surface at p (normal n): its up edge toward the craft's top (+Y), or forward on a roof
Component windowAt(const char* name, Vec3d p, Vec3d n, double w, double h, double taper, double rim, Rgba8 frame, double mass) {
    Component k = makeComponent(Func::Window);
    k.name = name;
    k.dims = {w, h, 0.02};
    k.taper = taper;
    k.rim = rim;
    k.color = frame;
    k.massOverride = mass;
    Vec3d up = std::fabs(n.y) > 0.9 ? Vec3d{0, 0, 1} : Vec3d{0, 1, 0};
    Vec3d y = norm(up - n * dot(up, n)), x = cross(y, n);
    k.pos = p;
    k.rotDeg = eulerDegOf(Mat3d::fromCols(x, y, n));
    return k;
}
} // namespace

std::vector<Craft> builtinCrafts() {
    std::vector<Craft> out;
    {
        Craft c;
        c.name = "Finned Sounding Rocket";
        auto add = [&](Component k) { c.comps.push_back(k); };
        Component srb = makeComponent(Func::Engine, CATALOG.find("srb_1"));
        srb.dims = {0.65, 0.65, 12};
        srb.pos = {0, 0, 6};
        srb.stage = 0;
        add(srb);
        Component fin = makeComponent(Func::Wing);
        fin.name = "Tail fin";
        fin.dims = {1.4, 2.2, 0.06};
        fin.material = 1;   // titanium: tolerates the Mach 4 heating of the leading edges
        fin.hollow = true;
        fin.wall = 0.003;
        fin.color = MATERIALS[1].color;
        fin.pos = {0.65 + 0.7, 0, 1.4};
        fin.rotDeg = {90, 0, 0};
        fin.symmetry = 4;
        fin.stage = 0;
        add(fin);
        Component dec = makeComponent(Func::Structure);
        dec.name = "Separation ring";
        dec.shape = Shape::Cylinder;
        dec.dims = {0.65, 0.65, 0.25};
        dec.pos = {0, 0, 12.125};
        dec.color = {200, 150, 40, 255};
        dec.massOverride = 40;
        dec.stage = 0;
        add(dec);
        // shallow spherical-cap shield: its centre of curvature lies behind the CoM, so the payload
        // re-enters stably shield-first (a flat disk gives no restoring torque)
        Component shield = makeComponent(Func::HeatShield);
        shield.shape = Shape::Ellipsoid;
        shield.dims = {0.95, 0.95, 0.22};
        shield.pos = {0, 0, 12.4};
        shield.massOverride = 260; // ablator + backshell structure keeps the CoM low
        shield.stage = 1;
        add(shield);
        Component cmd = makeComponent(Func::Command, CATALOG.find("probe_ring"));
        cmd.dims = {0.65, 0.65, 0.5};
        cmd.pos = {0, 0, 12.8};
        cmd.stage = 1;
        add(cmd);
        Component nose = makeComponent(Func::Structure);
        nose.name = "Nose cone";
        nose.shape = Shape::Cylinder;
        nose.dims = {0.65, 0.3, 1.5};
        nose.pos = {0, 0, 13.8};
        nose.material = 3;
        nose.color = {236, 236, 240, 255};
        nose.stage = 1;
        add(nose);
        Component chute = makeComponent(Func::Parachute, CATALOG.find("chute"));
        chute.dims = {0.3, 0.25, 0.5};
        chute.pos = {0, 0, 14.8};
        chute.stage = 1;
        add(chute);
        out.push_back(c);
    }
    // ---------------- aircraft (CAD): +Z up for rotorcraft, +Z forward / +Y up for aeroplanes
    auto part = [](Func f, const char* id) { return makeComponent(f, id ? CATALOG.find(id) : -1); };
    auto box = [](const char* name, Vec3d dims, Vec3d pos, double mass, Rgba8 col) {
        Component k = makeComponent(Func::Structure);
        k.name = name;
        k.dims = dims;
        k.pos = pos;
        k.massOverride = mass;
        k.color = col;
        return k;
    };
    // shape helpers for the aircraft and rovers below
    auto cyl = [](const char* name, double rb, double rt, double L, Vec3d pos, Vec3d rot, double mass, Rgba8 col) {
        Component k = makeComponent(Func::Structure);
        k.name = name;
        k.shape = Shape::Cylinder;
        k.dims = {rb, rt, L};
        k.pos = pos;
        k.rotDeg = rot;
        k.massOverride = mass;
        k.color = col;
        return k;
    };
    auto ell = [](const char* name, Vec3d radii, Vec3d pos, Vec3d rot, double mass, Rgba8 col) {
        Component k = makeComponent(Func::Structure);
        k.name = name;
        k.shape = Shape::Ellipsoid;
        k.dims = radii;
        k.pos = pos;
        k.rotDeg = rot;
        k.massOverride = mass;
        k.color = col;
        return k;
    };
    // a streamlined body (Shape::Loft): width, height, length; nose / tail length, tail end, upsweep, bluntness, squareness
    auto loft = [](const char* name, Vec3d dims, Vec3d pos, Vec3d rot, double nose, double tail, double tailR, double up, double bluff, double boxy,
                   double mass, Rgba8 col) {
        Component k = makeComponent(Func::Structure);
        k.name = name;
        k.shape = Shape::Loft;
        k.dims = dims;
        k.pos = pos;
        k.rotDeg = rot;
        k.noseLen = nose;
        k.tailLen = tail;
        k.tailR = tailR;
        k.upsweep = up;
        k.bluff = bluff;
        k.boxy = boxy;
        k.massOverride = mass;
        k.color = col;
        return k;
    };
    const Vec3d FWD_Y{90, 0, 180}; // a loft lying along +Y (the nose of a rotorcraft), its height up (+Z)
    // a wing panel (Shape::Airfoil): span, mean chord, thickness, taper, leading-edge sweep
    auto panel = [](const char* name, Vec3d dims, double taper, double sweep, Vec3d pos, Vec3d rot, double mass, Rgba8 col) {
        Component k = makeComponent(Func::Wing);
        k.name = name;
        k.shape = Shape::Airfoil;
        k.dims = dims;
        k.taper = taper;
        k.sweep = sweep;
        k.pos = pos;
        k.rotDeg = rot;
        k.massOverride = mass;
        k.color = col;
        k.material = 0; // (painted aluminium: the aircraft's panels; winged spacecraft get their tiles below)
        return k;
    };
    const Rgba8 glass{46, 70, 98, 255}, dark{30, 32, 36, 255}, metal{150, 154, 160, 255};
    {
        // electric coaxial helicopter after the Kamov Ka-226: a rounded, fully glazed cabin pod, coaxial
        // rotors on a short mast (no tail rotor), twin tail booms with a stabiliser and two fins, fixed
        // four-wheel gear. 1.6 t (hover needs about 310 kW of the 420 kW). +Z up, nose toward +Y
        Craft c;
        c.name = "Kestrel Helicopter";
        const Rgba8 body{214, 120, 36, 255}, light{236, 238, 240, 255};
        c.comps.push_back(loft("Cabin", {1.7, 1.75, 3.5}, {0, 0.35, 1.25}, FWD_Y, 0.42, 0.28, 0.6, 0.12, 0.85, 0.4, 330, body));
        {
            Component gl = loft("Glazed nose", {1.6, 1.55, 1.7}, {0, 1.33, 1.33}, FWD_Y, 0.6, 0.05, 1.0, 0, 0.9, 0.3, 30, glass);
            gl.material = MATERIAL_COUNT - 1; // (glass: see-through, reflecting)
            c.comps.push_back(gl);
        }
        c.comps.push_back(loft("Engine and gearbox fairing", {1.05, 0.7, 2.7}, {0, -0.3, 2.05}, FWD_Y, 0.38, 0.42, 0.35, 0.1, 0.7, 0.5, 110, light));
        for (int side = -1; side <= 1; side += 2) {
            c.comps.push_back(loft(side < 0 ? "Left tail boom" : "Right tail boom", {0.24, 0.27, 3.9}, {side * 0.5, -2.75, 1.78}, {90, 0, 180 + side * 4.0},
                                   0.06, 0.55, 0.55, 0.2, 1.0, 0.3, 30, body));
            Component fin = panel(side < 0 ? "Left fin" : "Right fin", {1.05, 0.72, 0.07}, 0.7, -18, {side * 0.64, -4.62, 2.28}, {0, -90, 0}, 6, body);
            c.comps.push_back(fin);
        }
        c.comps.push_back(panel("Stabiliser", {1.9, 0.62, 0.07}, 1.0, 0, {0, -4.6, 1.78}, {}, 12, light));
        Component cock = part(Func::Command, "capsule1");
        cock.name = "Flight computer";
        cock.dims = {0.4, 0.4, 0.5};
        cock.pos = {0, 0.6, 1.0};
        cock.massOverride = 60;
        c.comps.push_back(cock);
        Component bat = part(Func::Battery, "bat_l");
        for (int k = 0; k < 4; k++) {
            Component b = bat;
            b.pos = {k < 2 ? -0.45 : 0.45, k % 2 ? -0.6 : 0.2, 0.75};
            c.comps.push_back(b);
        }
        Component rot = part(Func::Rotor, "rotor_coax");
        rot.pos = {0, 0, 2.05 + rot.dims.z / 2};
        c.comps.push_back(rot);
        // quadricycle gear (fixed on the real one; here it can also fold): struts straight down
        Component g = part(Func::Wheel, "wheel");
        g.dims = {0.05, 0.05, 0.42};
        g.massOverride = 24;
        const Vec3d wheels[4] = {{-0.62, 1.15, 0.0}, {0.62, 1.15, 0.0}, {-0.95, -0.55, 0.0}, {0.95, -0.55, 0.0}};
        for (const Vec3d& w : wheels) {
            Component k = g;
            k.pos = {w.x, w.y, 0.35 + 0.21}; // the tyre (0.35 m radius) on the ground
            k.name = w.y > 0 ? "Front wheel" : "Main wheel";
            c.comps.push_back(k);
        }
        for (int side = -1; side <= 1; side += 2) // (the main wheels stand outside the cabin: their sponsons)
            c.comps.push_back(box("Main gear sponson", {0.5, 0.16, 0.09}, {side * 0.74, -0.55, 0.76}, 6, body));
        out.push_back(c);
    }
    {
        // Mars helicopter after Ingenuity, 1.8 kg: a 14 x 16 x 20 cm fuselage hung below coaxial 1.2 m rotors,
        // the solar panel on top of the mast, four carbon legs 38 cm long. About 120 W to hover at Jezero
        Craft c;
        c.name = "Mars Helicopter";
        const Rgba8 foil{196, 160, 92, 255};
        c.comps.push_back(box("Fuselage", {0.136, 0.163, 0.195}, {0, 0, 0.2}, 0.5, foil));
        Component cpu = part(Func::Command, "probe_s");
        cpu.name = "Flight computer";
        cpu.dims = {0.05, 0.05, 0.05};
        cpu.pos = {0, 0, 0.2};
        cpu.massOverride = 0.2;
        c.comps.push_back(cpu);
        c.comps.push_back(cyl("Mast", 0.012, 0.01, 0.36, {0, 0, 0.47}, {}, 0.05, dark));
        Component sol = part(Func::Solar, "solar_s");
        sol.name = "Solar panel";
        sol.dims = {0.18, 0.01, 0.10};
        sol.rotDeg = {90, 0, 0};
        sol.pos = {0, 0, 0.66};
        sol.massOverride = 0.05;
        c.comps.push_back(sol);
        Component rot = part(Func::Rotor, "rotor_mars");
        rot.pos = {0, 0, 0.3 + rot.dims.z / 2};
        c.comps.push_back(rot);
        Component leg = part(Func::Legs, "legs");
        leg.dims = {0.008, 0.008, 0.38};
        leg.pos = {0.148, 0, 0.12};
        leg.rotDeg = {0, -25, 0}; // (from the fuselage's lower edge out and down)
        leg.symmetry = 4;
        leg.massOverride = 0.05;
        leg.color = dark;
        c.comps.push_back(leg);
        out.push_back(c);
    }
    {
        // Titan rotorcraft after Dragonfly, about 0.5 t: an elongated lander body on two skids, four arms
        // with coaxial rotor pairs (eight rotors), the RTG at the back, a dish antenna on the top deck
        Craft c;
        c.name = "Titan Rotorcraft";
        const Rgba8 body{222, 216, 202, 255};
        c.comps.push_back(loft("Lander body", {1.35, 0.78, 2.4}, {0, 0, 0.95}, FWD_Y, 0.12, 0.12, 0.75, 0, 1.0, 0.85, 210, body));
        c.comps.push_back(box("Top deck", {1.5, 2.45, 0.06}, {0, 0, 1.35}, 15, metal));
        Component cpu = part(Func::Command, "probe_ring");
        cpu.name = "Flight computer";
        cpu.dims = {0.4, 0.4, 0.2};
        cpu.pos = {0, 0.4, 1.0};
        cpu.massOverride = 25;
        c.comps.push_back(cpu);
        Component rtg = part(Func::RTG, "rtg");
        rtg.pos = {0, -1.35, 1.05};
        rtg.rotDeg = {60, 0, 0};
        c.comps.push_back(rtg);
        Component bat = part(Func::Battery, "bat_l");
        bat.pos = {0, 0.2, 0.9};
        c.comps.push_back(bat);
        c.comps.push_back(cyl("Dish antenna", 0.05, 0.42, 0.1, {0, 0.35, 1.45}, {-20, 0, 0}, 6, metal));
        Component arm = makeComponent(Func::Structure);
        arm.name = "Rotor arm";
        arm.shape = Shape::Cylinder;
        arm.dims = {0.06, 0.05, 1.3};
        arm.rotDeg = {0, 90, 45};
        arm.pos = {0.75 * 0.7071, 0.75 * 0.7071, 1.3};
        arm.symmetry = 4;
        arm.massOverride = 3;
        arm.color = metal;
        c.comps.push_back(arm);
        Component rot = part(Func::Rotor, "rotor_titan");
        rot.pos = {1.3 * 0.7071, 1.3 * 0.7071, 1.3 + rot.dims.z / 2};
        rot.symmetry = 4;
        c.comps.push_back(rot);
        // two long skids under the body (legs that never fold)
        for (int side = -1; side <= 1; side += 2) {
            c.comps.push_back(cyl(side < 0 ? "Left skid" : "Right skid", 0.045, 0.045, 2.6, {side * 0.6, 0, 0.05}, {90, 0, 0}, 8, metal));
            for (int e = -1; e <= 1; e += 2) {
                Component s2 = part(Func::Legs, "legs");
                s2.name = "Skid strut";
                s2.dims = {0.04, 0.04, 0.62};
                s2.pos = {side * 0.6, e * 0.85, 0.34};
                s2.massOverride = 3;
                s2.color = metal;
                c.comps.push_back(s2);
            }
        }
        out.push_back(c);
    }
    {
        // advanced jet trainer after the BAE Hawk T.1: 11.6 m long, 9.4 m span (16.7 m2 swept, tapered, low
        // wing with 2 degrees of dihedral), a swept all-moving tailplane with 10 degrees of dihedral, one swept
        // fin, tandem canopy, intakes at the wing roots and a 27 kN turbofan in the tail. The tricycle gear
        // folds (G): the main wheels inward into the wing roots, the nose wheel forward
        Craft c;
        c.name = "Swift Jet Trainer";
        const Rgba8 red{196, 24, 46, 255}, white{236, 238, 242, 255};
        c.comps.push_back(loft("Fuselage", {1.24, 1.5, 11.5}, {0, 0.02, 0.25}, {}, 0.34, 0.42, 0.4, 0.22, 0.12, 0.3, 1210, red));
        c.comps.push_back(loft("Radome", {0.5, 0.52, 1.1}, {0, 0.1, 5.47}, {}, 0.6, 0.04, 1.0, 0, 0.1, 0, 40, white));
        {
            Component cn = loft("Canopy", {0.74, 0.66, 3.3}, {0, 0.62, 2.95}, {}, 0.46, 0.44, 0.08, -0.2, 0.55, 0.1, 30, glass);
            cn.material = MATERIAL_COUNT - 1; // (acrylic: the crew is seen through it)
            c.comps.push_back(cn);
            // tandem crew under the canopy: ejection seats, helmets, the instrument coamings (the rear seat raised)
            const Rgba8 seatC{46, 50, 56, 255}, helmet{232, 232, 226, 255}, visor{30, 30, 34, 255};
            // (inside the canopy: a token mass, the cockpit's 320 kg holds them)
            for (int s = 0; s < 2; s++) {
                double z = s ? 2.35 : 3.3, yb = s ? 0.05 : 0.0;
                c.comps.push_back(box("Ejection seat headrest", {0.26, 0.26, 0.08}, {0, 0.68 + yb, z - 0.27}, 0.1, seatC));
                c.comps.push_back(ell("Pilot's shoulders", {0.17, 0.07, 0.11}, {0, 0.66 + yb, z - 0.14}, {}, 0.1, Rgba8{70, 82, 64, 255}));
                c.comps.push_back(ell("Helmet", {0.11, 0.12, 0.13}, {0, 0.76 + yb, z - 0.13}, {}, 0.1, helmet));
                c.comps.push_back(ell("Visor", {0.09, 0.045, 0.045}, {0, 0.75 + yb, z - 0.02}, {}, 0.1, visor));
                c.comps.push_back(box("Instrument coaming", {0.3, 0.07, 0.14}, {0, 0.66 + yb, z + 0.36}, 0.1, seatC));
            }
        }
        c.comps.push_back(loft("Dorsal spine", {0.4, 0.4, 3.6}, {0, 0.6, -0.4}, {}, 0.25, 0.5, 0.2, -0.3, 0.6, 0.2, 0, red));
        for (int side = -1; side <= 1; side += 2) {
            c.comps.push_back(loft(side < 0 ? "Left intake" : "Right intake", {0.52, 0.78, 2.4}, {side * 0.64, 0.02, 1.05}, {}, 0.04, 0.6, 0.25, 0, 1.0, 0.45, 25, red));
            c.comps.push_back(loft("Intake face", {0.42, 0.64, 0.06}, {side * 0.64, 0.02, 2.25}, {}, 0.3, 0.3, 1.0, 0, 1.0, 0.45, 1, dark));
            {
                // double-slotted flaps inboard: 50 degrees at full (the Hawk's land setting; "quarter" = 12.5 degrees)
                Component wg = panel(side < 0 ? "Left wing" : "Right wing", {4.7, 1.78, 0.18}, 0.36, 26, {side * 2.35, -0.38, -0.12},
                                     {-93, 0, side * 2.0}, 300, red);
                wg.flap = 50;
                c.comps.push_back(wg);
            }
            c.comps.push_back(panel(side < 0 ? "Left tailplane" : "Right tailplane", {2.2, 0.95, 0.08}, 0.45, 30, {side * 1.1, 0.08, -4.75},
                                    {-90, 0, side * 10.0}, 40, white));
        }
        c.comps.push_back(seatRoot(panel("Fin and rudder", {2.0, 1.45, 0.1}, 0.45, 40, {0, 1.55, -4.5}, {-90, 0, -90}, 50, red), c.comps, true));
        Component cock = part(Func::Command, "capsule1");
        cock.name = "Cockpit";
        cock.dims = {0.55, 0.45, 1.6};
        cock.pos = {0, 0.1, 3.0};
        cock.massOverride = 320;
        cock.color = Rgba8{52, 56, 62, 255}; // (the cockpit tub, seen through the canopy)
        c.comps.push_back(cock);
        Component jet = part(Func::Jet, "jet_tf27");
        jet.pos = {0, 0, -3.6};
        c.comps.push_back(jet);
        Component tank = makeComponent(Func::Tank);
        tank.prop = Prop::JetFuel;
        tank.name = "Fuel tank";
        tank.color = propColor(Prop::JetFuel);
        tank.dims = {0.55, 0.55, 2.2};
        tank.pos = {0, 0, 0.6};
        c.comps.push_back(tank);
        Component bat = part(Func::Battery, "bat_s");
        bat.pos = {0, -0.2, 2.0};
        c.comps.push_back(bat);
        // tricycle gear: short struts down (-Y) from the fuselage (nose) and from the underside of the wing
        // (main wheels, just behind the centre of mass); the jet stands level
        Component g = part(Func::Wheel, "wheel");
        g.rotDeg = {-90, 0, 0};
        g.dims.z = 0.77;
        g.pos = {0, -0.55 - g.dims.z / 2, 3.6};
        g.name = "Nose gear";
        c.comps.push_back(g);
        for (int side = -1; side <= 1; side += 2) {
            Component m = g;
            m.name = "Main gear";
            m.dims.z = 0.85;
            m.pos = {side * 1.55, -0.47 - m.dims.z / 2, -0.9};
            c.comps.push_back(m);
        }
        out.push_back(c);
    }
    {
        // lunar rover after the Apollo Lunar Roving Vehicle: 3.0 m long, 2.3 m wheelbase, 1.8 m track, four
        // 81 cm wire-mesh wheels each with a 190 W hub motor (13 km/h at the most), two seats, the dish
        // antenna on its mast, the TV camera in front. Starts at Hadley-Apennine (Apollo 15) - "Start on: Moon"
        Craft c;
        c.name = "Lunar Rover";
        const Rgba8 alu{196, 198, 202, 255}, seat{88, 96, 110, 255}, fender{70, 76, 86, 255};
        c.comps.push_back(box("Chassis", {1.25, 0.12, 3.0}, {0, 0.55, 0}, 60, alu));
        c.comps.push_back(box("Floor panel", {1.15, 0.03, 1.4}, {0, 0.62, -0.2}, 8, alu));
        for (int side = -1; side <= 1; side += 2) {
            c.comps.push_back(box("Seat", {0.5, 0.06, 0.55}, {side * 0.3, 0.78, -0.35}, 4, seat));
            c.comps.push_back(box("Seat frame", {0.44, 0.13, 0.5}, {side * 0.3, 0.69, -0.35}, 1, alu));
            c.comps.push_back(box("Seat back", {0.5, 0.55, 0.05}, {side * 0.3, 1.05, -0.65}, 3, seat));
            for (int e = -1; e <= 1; e += 2) {
                c.comps.push_back(box("Fender", {0.42, 0.04, 0.95}, {side * 0.865, 0.98, e * 1.15}, 1, fender));
                c.comps.push_back(box("Fender bracket", {0.05, 0.38, 0.3}, {side * 0.645, 0.795, e * 1.15}, 0.5, alu));
                c.comps.push_back(box("Suspension arm", {0.34, 0.06, 0.1}, {side * 0.77, 0.58, e * 1.15}, 2, alu));
            }
        }
        c.comps.push_back(box("Control console", {0.22, 0.42, 0.14}, {0, 0.98, 0.25}, 6, metal));
        c.comps.push_back(box("Console post", {0.08, 0.15, 0.08}, {0, 0.7, 0.25}, 0.5, alu));
        Component cpu = part(Func::Command, "probe_s");
        cpu.name = "Drive control electronics";
        cpu.dims = {0.4, 0.15, 0.3};
        cpu.pos = {0, 0.7, 0.95};
        cpu.massOverride = 12;
        c.comps.push_back(cpu);
        Component bat = part(Func::Battery, "bat_s");
        for (int k = 0; k < 2; k++) {
            Component b = bat;
            b.name = "Silver-zinc battery";
            b.pos = {k ? 0.3 : -0.3, 0.72, 1.25};
            b.massOverride = 27;
            c.comps.push_back(b);
        }
        c.comps.push_back(cyl("Antenna mast", 0.015, 0.015, 0.85, {0.42, 1.05, 1.05}, {-90, 0, 0}, 2, metal));
        c.comps.push_back(cyl("High-gain dish", 0.04, 0.45, 0.22, {0.42, 1.55, 1.05}, {-90, 0, 0}, 4, alu));
        c.comps.push_back(cyl("Low-gain antenna", 0.025, 0.025, 0.6, {-0.42, 1.0, 1.05}, {-90, 0, 0}, 1, metal));
        c.comps.push_back(box("TV camera", {0.16, 0.14, 0.26}, {-0.25, 1.05, 1.38}, 6, dark));
        c.comps.push_back(cyl("Camera post", 0.02, 0.02, 0.39, {-0.25, 0.79, 1.38}, {-90, 0, 0}, 0.5, metal));
        Component w = part(Func::Wheel, "wheel_rover");
        w.rotDeg = {-90, 0, 0}; // strut down
        for (int side = -1; side <= 1; side += 2)
            for (int e = -1; e <= 1; e += 2) {
                Component k = w;
                k.name = "Wheel";
                k.pos = {side * 0.92, 0.405 + 0.25, e * 1.15};
                c.comps.push_back(k);
            }
        out.push_back(c);
    }
    {
        // Mars rover after Curiosity: 2.9 m long, 2.7 m wide, 2.2 m to the top of the mast, six 50 cm
        // wheels on a rocker-bogie, the MMRTG (110 W) at the back, the mast with its cameras, the robotic arm
        // folded in front, the high-gain antenna. 4 cm/s at the most. "Start on: Mars"
        Craft c;
        c.name = "Mars Rover";
        const Rgba8 white{226, 226, 222, 255}, deck{180, 176, 168, 255};
        c.comps.push_back(box("Body (warm electronics box)", {1.6, 0.55, 1.9}, {0, 0.95, 0}, 380, white));
        c.comps.push_back(box("Top deck", {1.9, 0.05, 2.3}, {0, 1.25, -0.1}, 50, deck));
        Component rtg = part(Func::RTG, "rtg");
        rtg.name = "MMRTG";
        rtg.pos = {0, 1.3, -1.4};
        rtg.rotDeg = {60, 0, 0};
        c.comps.push_back(rtg);
        Component cpu = part(Func::Command, "probe_s");
        cpu.name = "Rover compute element";
        cpu.dims = {0.4, 0.15, 0.3};
        cpu.pos = {0, 0.95, 0.2};
        cpu.massOverride = 15;
        c.comps.push_back(cpu);
        Component bat = part(Func::Battery, "bat_s");
        bat.name = "Battery";
        bat.pos = {0.4, 0.95, -0.4};
        bat.massOverride = 26;
        c.comps.push_back(bat);
        c.comps.push_back(cyl("Remote sensing mast", 0.07, 0.06, 1.05, {0.55, 1.8, 0.75}, {-90, 0, 0}, 15, white));
        c.comps.push_back(box("Mast head (cameras, laser)", {0.46, 0.24, 0.26}, {0.55, 2.4, 0.8}, 12, white));
        c.comps.push_back(cyl("High-gain antenna", 0.3, 0.28, 0.05, {-0.55, 1.42, -0.55}, {-60, 0, 0}, 8, deck));
        c.comps.push_back(cyl("Robotic arm", 0.05, 0.05, 1.1, {0.0, 0.72, 1.0}, {0, 90, 0}, 22, white));
        c.comps.push_back(box("Arm turret (drill, tools)", {0.3, 0.3, 0.26}, {0.6, 0.7, 1.12}, 12, metal));
        // rocker-bogie: a rocker from the middle wheel to the front, a bogie to the middle and rear wheels
        for (int side = -1; side <= 1; side += 2) {
            c.comps.push_back(box("Rocker", {0.06, 0.06, 1.3}, {side * 1.05, 0.72, 0.6}, 12, metal));
            c.comps.push_back(box("Bogie", {0.06, 0.06, 1.25}, {side * 1.05, 0.62, -0.6}, 10, metal));
            // the rocker turns on the body's differential axle, the bogie on the rocker's rear pivot; each wheel on a fork
            c.comps.push_back(cyl("Differential pivot", 0.05, 0.05, 0.27, {side * 0.92, 0.72, 0.0}, {0, 90, 0}, 2, metal));
            c.comps.push_back(box("Bogie pivot", {0.06, 0.16, 0.06}, {side * 1.05, 0.67, 0.0}, 1, metal));
            for (int q = 0; q < 3; q++)
                c.comps.push_back(box("Wheel fork", {0.15, 0.06, 0.08}, {side * 1.11, q < 2 ? 0.72 : 0.62, q == 0 ? 1.2 : q == 1 ? 0.0 : -1.2}, 1, metal));
        }
        Component w = part(Func::Wheel, "wheel_mars");
        w.rotDeg = {-90, 0, 0};
        const double zs[3] = {1.2, 0.0, -1.2};
        for (int side = -1; side <= 1; side += 2)
            for (double z : zs) {
                Component k = w;
                k.name = "Wheel";
                k.pos = {side * 1.15, 0.25 + 0.275, z};
                c.comps.push_back(k);
            }
        out.push_back(c);
    }
    const int MAT_TPS = 7; // Ceramic TPS (craft.cpp MATERIALS)
    auto rcsQuad = [&](const char* id, Vec3d pos, Vec3d rot) {
        Component r = part(Func::RCS, id);
        r.pos = pos;
        r.rotDeg = rot;
        return r;
    };
    // ---------------- space station modules (ISS-style assembly in orbit): +Z along the module. Start them with
    // "Start: in orbit" next to the vessel you fly; dock port to port (Docking Port DP-1, slower than 0.5 m/s)
    auto port = [&](Vec3d pos, Vec3d rot) {
        Component p = part(Func::Docking, "dock_port");
        p.pos = pos;
        p.rotDeg = rot;
        return p;
    };
    auto avionics = [&](Craft& c, Vec3d pos, double torqueKind) {
        Component cpu = part(Func::Command, "probe_ring");
        cpu.name = "Flight computer";
        cpu.dims = {0.6, 0.6, 0.3};
        cpu.pos = pos;
        cpu.massOverride = 300;
        c.comps.push_back(cpu);
        Component cmg = part(Func::CMG, torqueKind > 0 ? "cmg_l" : "cmg_s");
        cmg.pos = pos + Vec3d{0, 0, -1.2};
        c.comps.push_back(cmg);
        Component bat = part(Func::Battery, "bat_l");
        bat.pos = pos + Vec3d{0.8, 0, -0.6};
        c.comps.push_back(bat);
    };
    auto rcsSet = [&](Craft& c, double r, double zf, double za, const char* id) {
        Component mono = makeComponent(Func::Tank);
        mono.name = "RCS propellant";
        mono.prop = Prop::Monoprop;
        mono.dims = {0.45, 0.45, 1.4};
        mono.pos = {0, 0.9, (zf + za) * 0.5};
        mono.massOverride = 120;
        c.comps.push_back(mono);
        // four quads around each end (a quad's nozzles point along its local +-Y and +-Z: turned by 90 degrees
        // around the hull they cover every direction of translation and rotation)
        for (double z : {zf, za})
            for (int k = 0; k < 4; k++) {
                Component q = part(Func::RCS, id);
                double ang = k * 90.0;
                q.pos = {(r + 0.1) * std::cos(ang * DEG), (r + 0.1) * std::sin(ang * DEG), z};
                q.rotDeg = {0, 0, ang};
                c.comps.push_back(q);
            }
    };
    const Rgba8 stw{232, 233, 230, 255}, stg{196, 198, 200, 255};
    {
        // core module after Zarya (FGB): 12.6 m x 4.1 m, ~19 t; ports fore, aft and nadir, two solar wings
        Craft c;
        c.name = "Station Core Module";
        c.comps.push_back(cyl("Pressurised hull", 2.05, 2.05, 9.0, {0, 0, 0}, {}, 9500, stw));
        c.comps.push_back(cyl("Forward compartment", 2.05, 1.2, 2.2, {0, 0, 5.6}, {}, 1200, stw));
        c.comps.push_back(cyl("Aft skirt", 1.95, 2.05, 0.6, {0, 0, -4.8}, {}, 400, stg));
        c.comps.push_back(port({0, 0, 6.95}, {}));                // forward, facing +Z
        c.comps.push_back(port({0, 0, -5.35}, {180, 0, 0}));     // aft, facing -Z
        c.comps.push_back(port({0, -2.3, 2.5}, {90, 0, 0}));     // nadir, facing -Y
        avionics(c, {0, 0.6, 1.5}, 1);
        Component hy = makeComponent(Func::Tank);
        hy.name = "Propellant (reboost)";
        hy.prop = Prop::Hypergolic;
        hy.dims = {0.8, 0.8, 3.0};
        hy.pos = {0, -0.7, -1.8};
        hy.massOverride = 500;
        c.comps.push_back(hy);
        rcsSet(c, 2.05, 3.5, -3.8, "rcs_440");
        for (int side = -1; side <= 1; side += 2) {
            Component s = part(Func::Solar, "solar_m");
            s.name = "Solar wing";
            s.dims = {10.7, 0.03, 3.3};
            s.pos = {side * (2.05 + 5.45), 0, -2.0};
            if (side < 0) s.rotDeg = {0, 180, 0}; // (mirrored: hinged at the hull on both sides)
            c.comps.push_back(s);
        }
        out.push_back(c);
    }
    {
        // node after Unity (Node 1): 5.5 m x 4.6 m, ~11.6 t, six ports (both ends and four around the middle)
        Craft c;
        c.name = "Station Node Module";
        c.comps.push_back(cyl("Node hull", 2.28, 2.28, 4.6, {0, 0, 0}, {}, 7200, stw));
        c.comps.push_back(port({0, 0, 2.55}, {}));
        c.comps.push_back(port({0, 0, -2.55}, {180, 0, 0}));
        c.comps.push_back(port({2.53, 0, 0}, {0, 90, 0}));       // +X
        c.comps.push_back(port({-2.53, 0, 0}, {0, -90, 0}));     // -X
        c.comps.push_back(port({0, 2.53, 0}, {-90, 0, 0}));      // +Y
        c.comps.push_back(port({0, -2.53, 0}, {90, 0, 0}));      // -Y
        avionics(c, {0, 0.8, 0.8}, 0);
        rcsSet(c, 2.28, 1.6, -1.6, "rcs_440");
        out.push_back(c);
    }
    {
        // laboratory after Destiny: 8.5 m x 4.3 m, ~14.5 t; its berthing port forward, a second one aft
        Craft c;
        c.name = "Station Laboratory Module";
        c.comps.push_back(cyl("Laboratory hull", 2.15, 2.15, 7.4, {0, 0, 0}, {}, 9800, stw));
        c.comps.push_back(cyl("Forward cone", 2.15, 1.3, 0.8, {0, 0, 4.1}, {}, 500, stw));
        c.comps.push_back(cyl("Aft cone", 1.3, 2.15, 0.8, {0, 0, -4.1}, {}, 500, stw));
        c.comps.push_back(port({0, 0, 4.75}, {}));
        c.comps.push_back(port({0, 0, -4.75}, {180, 0, 0}));
        {
            Component w = makeComponent(Func::Window);
            w.name = "Window (optical port)";
            w.dims = {0.51, 0.51, 0.02}; // (Destiny's 51 cm science window)
            w.pos = {0, -2.15, -1.0};
            w.rotDeg = {90, 0, 0};       // facing -Y (the Earth)
            w.rim = 0.08;
            w.color = Rgba8{150, 154, 160, 255};
            w.massOverride = 60;
            c.comps.push_back(w);
        }
        avionics(c, {0, 0.8, 1.5}, 0);
        rcsSet(c, 2.15, 3.0, -3.0, "rcs_440");
        out.push_back(c);
    }
    {
        // truss with solar arrays and radiators (after the ISS integrated truss): a 32 m beam, four 30 x 11 m
        // arrays, radiators; the port in its middle faces -Y (docks onto a node's +Y port)
        Craft c;
        c.name = "Station Truss and Solar Arrays";
        Component tr = box("Truss", {32.0, 1.6, 1.6}, {0, 0, 0}, 9000, stg);
        tr.shape = Shape::Truss;
        c.comps.push_back(tr);
        c.comps.push_back(port({0, -1.05, 0}, {90, 0, 0}));
        avionics(c, {0, 0.2, 0}, 0);
        for (int side = -1; side <= 1; side += 2) {
            for (int k = -1; k <= 1; k += 2) {
                Component s = part(Func::Solar, "solar_l");
                s.name = "Solar array";
                s.dims = {11.0, 0.03, 30.0};
                s.pos = {side * 13.0, 0, k * 15.8}; // (the blanket box on the truss)
                c.comps.push_back(s);
            }
            c.comps.push_back(box("Radiator", {0.1, 6.0, 12.0}, {side * 6.0, -3.8, 0}, 600, stw));
        }
        for (int side = -1; side <= 1; side += 2)
            for (int e = -1; e <= 1; e += 2) {
                Component q = part(Func::RCS, "rcs_440");
                q.pos = {side * 15.5, 0.9, e * 0.6};
                c.comps.push_back(q);
            }
        Component mono = makeComponent(Func::Tank);
        mono.name = "RCS propellant";
        mono.prop = Prop::Monoprop;
        mono.dims = {0.45, 0.45, 1.4};
        mono.pos = {2.0, 0, 0};
        mono.rotDeg = {0, 90, 0};
        mono.massOverride = 120;
        c.comps.push_back(mono);
        out.push_back(c);
    }
    {
        // crew ferry after Crew Dragon: a 4 m capsule, the docking port in its nose, the trunk with solar cells;
        // RCS and an orbital engine for the rendezvous
        Craft c;
        c.name = "Crew Ferry";
        Component cap = part(Func::Command, "capsule3");
        cap.name = "Crew capsule";
        cap.pos = {0, 0, 0};
        c.comps.push_back(cap);
        c.comps.push_back(port({0, 0, cap.dims.z / 2 + 0.25}, {})); // (the port's 0.5 m collar on the capsule's top)
        Component hs = makeComponent(Func::HeatShield);
        hs.dims = {1.95, 1.95, 0.3};
        hs.pos = {0, 0, -cap.dims.z / 2 - 0.15};
        c.comps.push_back(hs);
        Component hy = makeComponent(Func::Tank);
        hy.name = "Propellant";
        hy.prop = Prop::Hypergolic;
        hy.dims = {1.2, 1.2, 1.6};
        hy.pos = {0, 0, -cap.dims.z / 2 - 1.2};
        hy.massOverride = 250;
        c.comps.push_back(hy);
        c.comps.push_back(cyl("Trunk", 1.85, 1.85, 2.8, {0, 0, -cap.dims.z / 2 - 1.7}, {}, 800, stw)); // (right under the heat shield)
        Component eng = part(Func::Engine, "eng_ho27");
        eng.pos = {0, 0, -cap.dims.z / 2 - 3.6};
        c.comps.push_back(eng);
        Component mono = makeComponent(Func::Tank);
        mono.name = "RCS propellant";
        mono.prop = Prop::Monoprop;
        mono.dims = {0.4, 0.4, 1.0};
        mono.pos = {0.9, 0, -cap.dims.z / 2 - 1.2};
        mono.massOverride = 60;
        c.comps.push_back(mono);
        for (int k = 0; k < 4; k++)
            for (double z : {cap.dims.z * 0.1, -cap.dims.z / 2 - 2.8}) {
                Component q = part(Func::RCS, "rcs_440");
                double ang = k * 90.0;
                // on the capsule's cone (its radius at that height), or on the trunk
                double rr = z > 0 ? cap.dims.x + (cap.dims.y - cap.dims.x) * (z + cap.dims.z / 2) / cap.dims.z : 1.85;
                rr += 0.06;
                q.pos = {rr * std::cos(ang * DEG), rr * std::sin(ang * DEG), z};
                q.rotDeg = {0, 0, ang};
                c.comps.push_back(q);
            }
        Component sol = part(Func::Solar, "solar_s");
        sol.name = "Trunk solar cells";
        // lengthwise along the trunk (lying close to its curved skin), between the RCS quads at 0 and 90 degrees
        sol.pos = {1.865 * std::cos(45 * DEG), 1.865 * std::sin(45 * DEG), -cap.dims.z / 2 - 1.7};
        sol.rotDeg = {0, 90, -45};
        c.comps.push_back(sol);
        out.push_back(c);
    }
    {
        // Space Shuttle type stack (STS): the winged orbiter (37 m, 23.8 m span, 3 x RS-25 class HM-2279
        // main engines, 2 x OMS, about 100 t at landing) on the side of the external tank (46.9 x 8.4 m,
        // 735 t of LOX / LH2 that feeds the orbiter's main engines) with two solid boosters (45 m, 500 t of
        // propellant, 12.5 MN at lift-off each). Stages: boosters (0), external tank (1), orbiter (2).
        // Stack frame: +Z up (the orbiter's nose), +Y the orbiter's top; it rides on the -Y side of nothing:
        // the tank is below its belly. Start: launch pad. The orbiter glides back to a runway
        Craft c;
        c.name = "Albatross Space Shuttle";
        const Rgba8 white{236, 236, 232, 255}, black{34, 34, 36, 255}, foam{196, 108, 50, 255};
        auto stg = [](Component k, int s) { k.stage = s; return k; };
        // --- external tank (stage 1): hydrolox tank, ogive nose, the attachment struts to the orbiter
        Component et = makeComponent(Func::Tank);
        et.name = "External tank";
        et.shape = Shape::Cylinder;
        et.prop = Prop::Hydrolox;
        et.dims = {4.2, 4.2, 37.0};
        et.pos = {0, 0, 18.5};
        et.massOverride = 23500;
        et.color = foam;
        et.stage = 1;
        c.comps.push_back(et);
        c.comps.push_back(stg(cyl("External tank nose (LOX tank top)", 4.2, 0.35, 9.4, {0, 0, 41.7}, {}, 3000, foam), 1));
        c.comps.push_back(stg(box("Forward bipod", {1.2, 1.0, 0.5}, {0, 4.7, 33.0}, 150, metal), 1));
        c.comps.push_back(stg(box("Aft attach struts", {4.0, 1.0, 0.6}, {0, 4.7, 8.0}, 400, metal), 1));
        // --- solid rocket boosters (stage 0) beside the tank: 45 m with the nose cap, the aft skirt on the pad
        for (int side = -1; side <= 1; side += 2) {
            Component srb = part(Func::Engine, "srb_3");
            srb.name = "Solid rocket booster";
            srb.pos = {side * 6.45, 0, 21.0};
            srb.stage = 0;
            c.comps.push_back(srb);
            // the attachments to the external tank (they go with the boosters): the forward attach at the tank's
            // intertank and the aft attach struts at its lower ring
            double rS = std::max(srb.dims.x, srb.dims.y), gap = 6.45 - rS - 4.2;
            for (double z : {33.0, 6.0}) {
                Component at = box(z > 20 ? "Booster forward attach" : "Booster aft attach struts", {gap + 0.3, z > 20 ? 0.5 : 0.9, z > 20 ? 0.6 : 0.35},
                                   {side * (4.2 + gap / 2), 0, z}, 60, metal);
                at.stage = 0;
                c.comps.push_back(at);
            }
        }
        // --- the orbiter (stage 2)
        const double y0 = 7.9; // its axis: 4.2 m tank radius + 0.9 m struts + 2.8 m half height
        auto orb = [&](Component k) { k.stage = 2; k.material = (k.func == Func::Structure || k.func == Func::Wing) ? MAT_TPS : k.material; return k; };
        // (the payload bay's doors and liner, the drag chute and the windows come out of the fuselage's 40 t)
        Component fus = loft("Orbiter fuselage", {5.2, 5.6, 33.0}, {0, y0, 22.6}, {}, 0.2, 0.02, 0.95, 0, 0.85, 0.7, 40000 - 1200 - 260 - 3 - 200, white);
        c.comps.push_back(orb(fus));
        c.comps.push_back(orb(box("Belly tiles (HRSI)", {4.9, 0.1, 26.5}, {0, y0 - 2.78, 20.5}, 900, black)));
        {
            // eleven windows (as the orbiter's): six windshield panes, two overhead, two aft into the payload bay, one
            // in the side hatch; black frames (the HRSI tiles around them)
            const Rgba8 frame{18, 18, 20, 255};
            Vec3d p, n;
            const double ws[3][4] = {{10.5, 0.92, 0.64, 0.86}, {31.0, 0.88, 0.62, 0.82}, {50.5, 0.78, 0.56, 0.8}}; // angle, width, height, taper
            for (int s = -1; s <= 1; s += 2)
                for (auto& wv : ws) {
                    loftSurface(fus, 12.35, 90 + s * wv[0], p, n);
                    c.comps.push_back(orb(windowAt("Windshield", p, n, wv[1], wv[2], wv[3], 0.09, frame, 7)));
                }
            for (int s = -1; s <= 1; s += 2) {
                loftSurface(fus, 10.3, 90 + s * 9.0, p, n);
                c.comps.push_back(orb(windowAt("Overhead window", p, n, 0.46, 0.44, 1.0, 0.07, frame, 7)));
            }
            loftSurface(fus, 8.9, -8.0, p, n);
            c.comps.push_back(orb(windowAt("Side hatch window", p, n, 0.26, 0.26, 1.0, 0.05, frame, 7)));
        }
        // payload bay: 4.6 m wide, 4 m deep, between the crew module and the OMS pods (the doors open in orbit)
        Component bay = makeComponent(Func::CargoBay);
        bay.name = "Payload bay";
        bay.dims = {4.6, 4.0, 15.7};
        bay.pos = {0, y0 + 0.15, 21.75};
        bay.massOverride = 1200;
        c.comps.push_back(orb(bay));
        for (int s = -1; s <= 1; s += 2) {
            Vec3d n{0, 0, -1};
            c.comps.push_back(orb(windowAt("Aft flight deck window", {s * 0.45, y0 + 1.55, 21.75 + 7.85 - 0.04}, n, 0.42, 0.3, 1.0, 0.05,
                                           Rgba8{18, 18, 20, 255}, 7)));
        }
        {
            // the payload (3 t at the old cargo's centre of mass): a relay satellite with its apogee motor, folded
            // solar wings and an antenna; released in orbit from the open bay
            auto cg = [](Component k) { k.stage = 2; k.cargo = 1; return k; };
            const double py = y0 + 0.6;
            Component bus = part(Func::Command, "probe_s");
            bus.name = "Relay satellite";
            bus.shape = Shape::Box;
            bus.dims = {2.0, 2.0, 2.2};
            bus.pos = {0, py, 23.9};
            bus.massOverride = 1500;
            bus.color = Rgba8{214, 170, 70, 255};
            c.comps.push_back(cg(bus));
            Component dish = makeComponent(Func::Structure);
            dish.name = "Antenna";
            dish.shape = Shape::Cylinder;
            dish.dims = {0.9, 0.05, 0.35};
            dish.pos = {0, py, 25.2};
            dish.massOverride = 40;
            dish.color = Rgba8{236, 236, 232, 255};
            c.comps.push_back(cg(dish));
            Component tk = makeComponent(Func::Tank);
            tk.name = "Apogee motor propellant";
            tk.prop = Prop::Hypergolic;
            tk.dims = {0.6, 0.6, 1.4};
            tk.pos = {0, py, 22.1};
            tk.fill = 0.65;
            tk.massOverride = 120;
            tk.color = Rgba8{200, 200, 196, 255};
            c.comps.push_back(cg(tk));
            Component am = part(Func::Engine, "eng_ho27");
            am.name = "Apogee motor";
            am.pos = {0, py, 20.3};
            c.comps.push_back(cg(am));
            for (int s = -1; s <= 1; s += 2) {
                Component sp = part(Func::Solar, "solar_s");
                sp.name = "Solar wing";
                sp.pos = {s * (1.0 + sp.dims.x / 2), py, 23.9};
                if (s < 0) sp.rotDeg = {0, 180, 0}; // (mirrored: hinged at the bus on both sides)
                c.comps.push_back(cg(sp));
            }
            // its cradle on the bay floor (airborne support equipment: stays in the orbiter)
            const double floorY = y0 + 0.15 - 2.0;
            Component cr1 = box("Payload cradle", {2.2, (py - 1.0) - floorY, 0.5}, {0, (floorY + py - 1.0) / 2, 23.9}, 120, metal);
            Component cr2 = box("Payload cradle", {1.4, (py - 0.6) - floorY, 0.4}, {0, (floorY + py - 0.6) / 2, 22.1}, 80, metal);
            cr1.stage = cr2.stage = 2;
            c.comps.push_back(cr1);
            c.comps.push_back(cr2);
        }
        {
            // drag chute in its compartment at the base of the fin, facing aft (deployed on the runway: P)
            Component dc = part(Func::Parachute, "chute_drag");
            dc.name = "Drag chute";
            dc.pos = {0, y0 + 3.0, 6.4};
            dc.rotDeg = {180, 0, 0};
            c.comps.push_back(orb(dc));
        }
        for (int side = -1; side <= 1; side += 2) {
            // double delta: the fixed glove (strake, 78 degrees) and the main wing (45 degrees) with its elevons
            // (both seated in the fuselage's lower sides: the main wing extended inward, its tip at the real span;
            // the 78-degree glove moved in whole, so that it still starts behind the crew cabin)
            const std::vector<Component> body{fus};
            c.comps.push_back(orb(seatRoot(panel(side < 0 ? "Left wing" : "Right wing", {9.4, 11.0, 0.7}, 0.22, 45, {side * (2.55 + 4.7), y0 - 2.1, 17.6},
                                                 {-91, 0, side * 3.5}, 5200, white), body, true)));
            Component glove = panel("Wing glove", {2.2, 12.0, 0.9}, 0.25, 78, {side * (2.55 + 1.1), y0 - 2.1, 25.8}, {-91, 0, side * 3.5}, 600, white);
            glove.func = Func::Structure; // (its lift is part of the wing's)
            c.comps.push_back(orb(seatRoot(glove, body, false)));
            // OMS pods with their hypergolic tanks and engines
            c.comps.push_back(orb(loft("OMS pod", {2.0, 1.7, 8.0}, {side * 1.75, y0 + 2.45, 9.8}, {}, 0.35, 0.08, 0.6, 0, 0.6, 0.3, 1300, white)));
            Component ot = makeComponent(Func::Tank);
            ot.name = "OMS propellant";
            ot.prop = Prop::Hypergolic;
            ot.dims = {0.62, 0.62, 3.9};
            ot.pos = {side * 1.75, y0 + 2.45, 10.0};
            ot.massOverride = 300;
            ot.color = white;
            c.comps.push_back(orb(ot));
            Component oms = part(Func::Engine, "eng_ho27");
            oms.name = "OMS engine";
            oms.pos = {side * 1.75, y0 + 2.45, 5.0};
            c.comps.push_back(orb(oms));
        }
        c.comps.push_back(orb(panel("Vertical stabilizer and rudder", {7.6, 6.4, 0.7}, 0.42, 45, {0, y0 + 2.8 + 3.8, 9.6}, {-90, 0, -90}, 1500, white)));
        // body flap: a moving surface under the engines that trims the pitch at the high angle of attack of the
        // entry (the belly's lift acts ahead of the centre of mass there) and shields the main engines from below
        for (int side = -1; side <= 1; side += 2)
            c.comps.push_back(orb(panel(side < 0 ? "Body flap (left)" : "Body flap (right)", {3.0, 3.6, 0.45}, 1.0, 0, {side * 1.5, y0 - 2.35, 4.55},
                                        {-90, 0, 0}, 250, black)));
        // three main engines fed from the external tank, canted toward the stack's centre of mass (lit with
        // the boosters: they fire from group 0)
        const Vec3d ssme[3] = {{0, y0 + 1.25, 4.4}, {-1.35, y0 - 1.05, 4.4}, {1.35, y0 - 1.05, 4.4}};
        for (const Vec3d& p : ssme) {
            Component e = part(Func::Engine, "eng_hm2279");
            e.name = "Main engine";
            e.pos = p;
            e.rotDeg = {15.5, 0, 0};
            e.fireFrom = 0;
            c.comps.push_back(orb(e));
        }
        Component cab = part(Func::Command, "capsule3");
        cab.name = "Crew module (flight deck)";
        cab.dims = {1.3, 1.3, 2.6}; // (inside the fuselage's forward section, behind the windows)
        cab.pos = {0, y0 + 0.4, 31.0};
        cab.massOverride = 7000;
        c.comps.push_back(orb(cab));
        Component bat = part(Func::Battery, "bat_l");
        bat.name = "Fuel cells";
        bat.pos = {0, y0 - 1.0, 30.0};
        c.comps.push_back(orb(bat));
        Component mono = makeComponent(Func::Tank);
        mono.name = "RCS propellant";
        mono.prop = Prop::Monoprop;
        mono.dims = {0.7, 0.7, 2.2}; // (3.4 t, as the orbiter's RCS propellant)
        // in the nose ahead of the crew module (the forward RCS module's place): the centre of mass forward
        // enough for the elevons and the body flap to trim the 40-degree entry with room to spare
        mono.pos = {0, y0 + 1.0, 33.5};
        mono.massOverride = 300;
        c.comps.push_back(orb(mono));
        for (int side = -1; side <= 1; side += 2) {
            // forward RCS module in the nose (on its skin), aft RCS in the OMS pods
            {
                Vec3d p, n;
                loftSurface(fus, 36.5 - fus.pos.z, side > 0 ? 28.0 : 152.0, p, n);
                c.comps.push_back(orb(rcsQuad("rcs_3870", p, {})));
            }
            c.comps.push_back(orb(rcsQuad("rcs_3870", {side * 2.75, y0 + 2.45, 7.0}, {})));
        }
        // tricycle landing gear (stowed for the launch)
        Component g = part(Func::Wheel, "wheel");
        g.rotDeg = {-90, 0, 0};
        g.dims.z = 2.3;
        g.name = "Nose gear";
        g.pos = {0, y0 - 2.6 - g.dims.z / 2, 33.0};
        g.massOverride = 400;
        c.comps.push_back(orb(g));
        for (int side = -1; side <= 1; side += 2) {
            Component m = g;
            m.name = "Main gear";
            m.dims.z = 2.6;
            m.pos = {side * 3.45, y0 - 2.4 - m.dims.z / 2, 13.8};
            m.massOverride = 1300;
            c.comps.push_back(orb(m));
        }
        out.push_back(c);
    }
    {
        // single-stage-to-orbit spaceplane after Skylon (Reaction Engines): 83 m, 26.8 m span, 6.3 m fuselage,
        // 325 t at take-off, two precooled combined-cycle engines in wing-tip nacelles (air-breathing on LH2
        // to Mach 5.4 / ~28 km, then rocket mode on LOX / LH2), canards, a ceramic skin. Takes off from the
        // runway ("Start: runway"), flies to orbit on one stage and glides back
        Craft c;
        c.name = "Skylark SSTO Spaceplane";
        const Rgba8 skin{62, 58, 54, 255}, nac{78, 74, 70, 255};
        auto sk = [&](Component k) { if (k.func == Func::Structure || k.func == Func::Wing) k.material = MAT_TPS; return k; };
        c.comps.push_back(sk(loft("Fuselage", {6.3, 6.3, 83.0}, {0, 0, 0}, {}, 0.24, 0.22, 0.3, 0.55, 0.25, 0.05, 7500, skin)));
        // propellant: LOX / LH2 for the rocket mode fore and aft of the centre of mass (2 x 113 t), the air-breathing
        // LH2 in the middle (30 t: Mach 5.4 takes ~25 t; the balance stays put as they drain)
        auto tank = [&](const char* name, Prop p, double L, double z, double mass) {
            Component t = makeComponent(Func::Tank);
            t.name = name;
            t.prop = p;
            t.dims = {3.0, 3.0, L};
            t.pos = {0, 0, z};
            t.massOverride = mass;
            t.color = propColor(p);
            return t;
        };
        c.comps.push_back(tank("Forward LOX / LH2 tank", Prop::Hydrolox, 11.0, 15.9, 6000));
        c.comps.push_back(tank("Aft LOX / LH2 tank", Prop::Hydrolox, 11.0, -16.5, 6000));
        c.comps.push_back(tank("Air-breathing LH2 tank", Prop::LH2, 15.0, -0.3, 3500));
        for (int side = -1; side <= 1; side += 2) {
            const std::vector<Component> body{c.comps[0]}; // (the fuselage)
            c.comps.push_back(sk(seatRoot(panel(side < 0 ? "Left wing" : "Right wing", {9.9, 16.0, 0.6}, 0.35, 35, {side * (3.15 + 4.95), -0.6, -2.4},
                                                {-91, 0, 0}, 3000, skin), body, true)));
            c.comps.push_back(sk(seatRoot(panel(side < 0 ? "Left canard" : "Right canard", {4.0, 3.2, 0.3}, 0.5, 30, {side * (2.45 + 2.0), 0.6, 30.5},
                                                {-90, 0, 0}, 500, skin), body, true)));
            Component e = part(Func::Jet, "jet_pce2000");
            e.name = "Combined-cycle engine";
            e.pos = {side * 13.2, -0.6, -3.6};
            e.massOverride = 8000;
            e.color = nac;
            c.comps.push_back(e);
        }
        c.comps.push_back(sk(seatRoot(panel("Fin and rudder", {7.0, 6.5, 0.5}, 0.4, 40, {0, 2.3 + 3.5, -35.0}, {-90, 0, -90}, 1200, skin),
                                      {c.comps[0]}, true)));
        Component cpu = part(Func::Command, "probe_ring");
        cpu.name = "Flight computer";
        cpu.dims = {0.8, 0.8, 0.4};
        cpu.pos = {0, 0.5, 34.0};
        cpu.massOverride = 500;
        c.comps.push_back(cpu);
        Component cmg = part(Func::CMG, "cmg_l");
        cmg.pos = {0, -1.0, 24.0};
        c.comps.push_back(cmg);
        Component bat = part(Func::Battery, "bat_l");
        bat.pos = {0, 1.0, 24.0};
        c.comps.push_back(bat);
        Component mono = makeComponent(Func::Tank);
        mono.name = "RCS propellant";
        mono.prop = Prop::Monoprop;
        mono.dims = {0.55, 0.55, 2.0};
        mono.pos = {0, 1.2, 28.0};
        mono.massOverride = 120;
        c.comps.push_back(mono);
        for (int side = -1; side <= 1; side += 2) {
            for (double z : {30.0, -30.0}) {
                Vec3d p, n;
                loftSurface(c.comps[0], z, side > 0 ? 15.0 : 165.0, p, n);
                c.comps.push_back(rcsQuad("rcs_440", p, {}));
            }
        }
        Component g = part(Func::Wheel, "wheel");
        g.rotDeg = {-90, 0, 0};
        g.dims.z = 3.4;
        g.name = "Nose gear";
        // (the struts reach up into the hull; the wheels stand where they did)
        {
            Vec3d p, n;
            loftSurface(c.comps[0], 28.0, 270.0, p, n);
            double bottom = -2.95 - 3.4, top = p.y + 0.06;
            g.dims.z = top - bottom;
            g.pos = {0, (top + bottom) / 2, 28.0};
        }
        g.massOverride = 900;
        c.comps.push_back(g);
        for (int side = -1; side <= 1; side += 2) {
            Component m = g;
            m.name = "Main gear";
            Vec3d p, n;
            double x = 2.4;
            loftSurface(c.comps[0], -4.2, side > 0 ? 270.0 + 40.0 : 270.0 - 40.0, p, n);
            x = std::fabs(p.x);
            double bottom = -2.95 - 3.6, top = p.y + 0.06;
            m.dims.z = top - bottom;
            m.pos = {side * x, (top + bottom) / 2, -4.2};
            m.massOverride = 1800;
            c.comps.push_back(m);
        }
        out.push_back(c);
    }
    {
        // a truss segment to extend a station: 12 m of lattice with a docking port at each end and the means to
        // fly to the station on its own (flight computer, battery, RCS around both ends)
        Craft c;
        c.name = "Station Truss Segment";
        Component tr = makeComponent(Func::Structure);
        tr.name = "Truss";
        tr.shape = Shape::Truss;
        tr.dims = {1.6, 1.6, 12.0};
        tr.wall = 0.004;
        tr.material = 0;
        tr.hollow = false;
        tr.color = Rgba8{196, 200, 206, 255};
        c.comps.push_back(tr);
        c.comps.push_back(port({0, 0, 6.25}, {}));
        c.comps.push_back(port({0, 0, -6.25}, {180, 0, 0}));
        avionics(c, {0, 0, 2.0}, 0);
        rcsSet(c, 0.8, 5.2, -5.2, "rcs_440");
        out.push_back(c);
    }
    return out;
}

