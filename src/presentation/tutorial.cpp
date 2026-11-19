// Tutorial missions. Each tutorial sets up a starting situation (factory, launch pad or an orbit) and
// walks through a list of steps; a step is finished when its condition holds in the simulation (or
// when the player clicks Next on an explanation step).
#include "tutorial.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>
#include "app.hpp"
#include "screens.hpp"
#include "session.hpp"
#include "storage.hpp"
#include "audio.hpp"
#include "i18n.hpp"
#include "keys.hpp"
#include "orbit.hpp"
#include "ui.hpp"

bool flightMapMode();
bool flightHyperOpen();

namespace tut {
bool menuOpen = false;

namespace {
using namespace ui;

struct State {
    int tut = -1;
    int step = 0;
    double doneAt = -1;   // real time the current step was completed (short "done" flash)
    bool finished = false;
    bool collapsed = false;
    int stage0 = 0;       // stage index when the step started
    double acc = 0;       // per-step accumulator (seconds of RCS translation)
    bool flag = false;    // per-step flag (warp-to-point seen)
    int vesselId = -1;    // the tutorial vessel
    double best = 1e9;    // per-step minimum (smallest tilt seen during a plane change)
    double mark = 0;      // simulation time planned by the setup (launch window)
    double target = 0;    // apoapsis radius of the burn for the Moon (the Moon's distance on arrival)
    double tliEnd = 0;    // simulation time the burn for the Moon ended
};
State S;
Guide G;                  // guidance of the current step, rebuilt every frame

struct Ctx {
    App& app;
    World& W;
    Vessel* v;
    float dt;
};

struct Step {
    const char* text;
    std::function<bool(Ctx&)> done; // null: explanation step finished with "Next"
    std::function<std::string(Ctx&)> hint;
};

struct Tutorial {
    const char* title;
    const char* summary;
    std::function<bool(App&)> setup;
    std::vector<Step> steps;
    const char* doneText;
};

// ---------------------------------------------------------------- vessel helpers
std::string fmtd(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
}
double altOf(const World& W, const Vessel& v) { return len(v.r) - W.U.bodies[v.ref].radius; }
const Body& bodyOf(const World& W, const Vessel& v) { return W.U.bodies[v.ref]; }
Conic conicOf(const World& W, const Vessel& v) { return stateToConic(v.r, v.v, bodyOf(W, v).GM); }
double apAlt(const World& W, const Vessel& v) {
    if (v.landed) return 0;
    Conic c = conicOf(W, v);
    return c.e < 1 ? c.ap - bodyOf(W, v).radius : 1e12;
}
double peAlt(const World& W, const Vessel& v) {
    if (v.landed) return -1e9;
    return conicOf(W, v).pe - bodyOf(W, v).radius;
}
// attitude against the local horizon (as the navball shows it)
void horizonAxes(const World& W, const Vessel& v, Vec3d& up, Vec3d& north, Vec3d& east) {
    const Body& b = bodyOf(W, v);
    up = norm(v.r);
    Vec3d pole = b.poleFrame.col(2);
    north = pole - up * dot(pole, up);
    if (len(north) < 1e-9) north = anyPerp(up);
    north = norm(north);
    east = cross(north, up);
}
double pitchDeg(const World& W, const Vessel& v) {
    Vec3d up, n, e;
    horizonAxes(W, v, up, n, e);
    return std::asin(clampd(dot(v.q.rotate({0, 0, 1}), up), -1, 1)) / DEG;
}
double headingDeg(const World& W, const Vessel& v) {
    Vec3d up, n, e;
    horizonAxes(W, v, up, n, e);
    Vec3d az = v.q.rotate({0, 0, 1});
    return wrapAngle(std::atan2(dot(az, e), dot(az, n))) / DEG;
}
// the pitch programme of the ascent guidance: vertical to 800 m, then the angle from the vertical grows
// with the square root of the altitude (80 degrees at a third of the atmosphere)
double targetPitchDeg(const World& W, const Vessel& v) {
    const Body& b = bodyOf(W, v);
    double alt = altOf(W, v);
    if (alt < 800) return 90;
    double th = std::min(80.0, 90.0 * std::sqrt((alt - 800) / (b.atmTop * 0.34)));
    return 90 - th;
}

// angle between the nose and the retrograde direction (surface velocity low in the atmosphere)
double retroErrDeg(const World& W, const Vessel& v) {
    const Body& b = bodyOf(W, v);
    bool surf = altOf(W, v) < (b.atmTop > 0 ? b.atmTop * 0.5 : 0.02 * b.radius);
    Vec3d vel = surf ? W.surfaceVel(v) : v.v;
    if (len(vel) < 1) return 0;
    return std::acos(clampd(dot(v.q.rotate({0, 0, 1}), -norm(vel)), -1, 1)) / DEG;
}

// ---------------------------------------------------------------- orbit planes, nodes, burns
Vec3d poleDir(const Body& b) { return b.poleFrame.col(2) * (b.rotRate < 0 ? -1.0 : 1.0); }
// tilt of an orbit plane against the equator of its body: 0 for an equatorial orbit in either direction
double tiltOf(const Conic& c, const Body& b) {
    double inc = std::acos(clampd(dot(c.H, poleDir(b)), -1, 1)) / DEG;
    return std::min(inc, 180 - inc);
}
double tiltDeg(const World& W, const Vessel& v) { return v.landed ? 90 : tiltOf(conicOf(W, v), bodyOf(W, v)); }

// a crossing of the equatorial plane ahead on the orbit
struct Node {
    bool ok = false;
    double t = 0, radius = 0, speed = 0;
    bool northbound = false;
};
// the next crossing, or with `high` the one farther from the body (the cheaper place to turn the plane)
Node nodeAhead(const World& W, const Vessel& v, bool high) {
    Node best;
    if (v.landed) return best;
    const Body& b = bodyOf(W, v);
    Conic c = conicOf(W, v);
    Vec3d n = cross(poleDir(b), c.H); // towards the northbound crossing
    if (!c.valid || len(n) < 1e-4) return best;
    n = norm(n);
    double nuN = std::atan2(dot(n, c.Q), dot(n, c.P));
    for (int k = 0; k < 2; k++) {
        double nu = nuN + k * PI, den = 1 + c.e * std::cos(nu);
        if (den < 1e-6) continue;
        double t = timeToTrueAnomaly(c, nu);
        if (t < 0) continue;
        Node o{true, t, c.p / den, 0, k == 0};
        o.speed = std::sqrt(std::max(0.0, b.GM * (2 / o.radius - 1 / c.a)));
        bool take = !best.ok;
        if (best.ok && high) take = o.radius > best.radius * 1.02 || (o.radius > best.radius / 1.02 && o.t < best.t);
        if (best.ok && !high) take = o.t < best.t;
        if (take) best = o;
    }
    return best;
}
// SAS mode whose thrust flattens the orbit at a crossing (pushes against the north / south motion)
SAS flattenMode(const World& W, const Vessel& v, bool northbound) {
    bool prograde = dot(conicOf(W, v).H, poleDir(bodyOf(W, v))) > 0;
    return northbound == prograde ? SAS::AntiNormal : SAS::Normal;
}
// duration of a burn of dv with the engines of the active stage at full throttle
double burnTime(const World& W, const Vessel& v, double dv) {
    double amt[PROP_N], cap[PROP_N], sa, sc, F, mdot;
    stageResources(v, amt, cap, sa, sc, F, mdot);
    if (F <= 0 || mdot <= 0 || dv <= 0) return 0;
    return W.vesselMass(v) * (1 - std::exp(-dv * mdot / F)) / mdot;
}
// time to deliver dv when the burn runs on through the following stages (a stage that runs dry is
// dropped and the next one lit): liquid engines of each group with that group's tanks
double burnTimeStages(const World& W, const Vessel& v, double dv) {
    double m = W.vesselMass(v), t = 0, left = dv;
    std::vector<int> groups = vesselStages(v);
    for (int g : groups) {
        if (g < v.stage) continue;
        double F = 0, mdot = 0, prop = 0, mass = 0;
        for (size_t i = 0; i < v.comps.size(); i++) {
            const Component& c = v.comps[i];
            if (c.stage != g) continue;
            mass += v.phys[i].dryMass + v.prop[i];
            if (c.func == Func::Tank && c.prop != Prop::Monoprop) prop += v.prop[i];
            if (c.func == Func::Engine && v.phys[i].thrustVac > 0 && v.phys[i].propCap <= 0) {
                F += v.phys[i].thrustVac;
                mdot += v.phys[i].thrustVac / (v.phys[i].ispVac * G0);
            }
        }
        if (F > 0 && mdot > 0 && prop > 0) {
            double ve = F / mdot, dvStage = ve * std::log(m / std::max(m - prop, 1.0));
            if (left <= dvStage) return t + m * (1 - std::exp(-left / ve)) / mdot;
            t += prop / mdot + 3; // and a few seconds for the staging
            left -= dvStage;
        }
        m -= mass;
        if (m <= 1) break;
    }
    return t;
}

// seconds until a burn centred on an event `tEvent` seconds ahead has to start. A start missed by more
// than a minute waits for the next pass.
double igniteIn(double tEvent, double burn, double period) {
    double t = tEvent - burn / 2;
    if (t < -60 && period > 0) t += period;
    return t;
}
// speed on an orbit through radius r with the opposite apsis at r2
double apsisSpeed(double GM, double r, double r2) { return std::sqrt(GM * (2 / r - 2 / (r + r2))); }
double geoRadius(const Body& b) { return std::cbrt(b.GM / (b.rotRate * b.rotRate)); }

// ---------------------------------------------------------------- the Moon
// predicted pass of the Moon (or the orbit around it once the vessel is there): fills G.enc*
double lunarEncounter(const World& W, const Vessel* v) {
    G.enc = false;
    if (!v || v->landed) return 0;
    const Universe& U = W.U;
    int moon = U.find("Moon");
    if (moon < 0) return 0;
    const Body& mb = U.bodies[moon];
    Conic k;
    double tEntry = 0;
    if (v->ref == moon) k = stateToConic(v->r, v->v, mb.GM);
    else {
        Encounter e;
        if (!predictEncounter(U, v->ref, v->r, v->v, W.t, e) || e.body != moon) return 0;
        k = stateToConic(e.r, e.v, mb.GM);
        tEntry = e.tEntry;
    }
    G.enc = k.valid;
    G.encPe = k.pe - mb.radius;
    G.encTilt = tiltOf(k, mb);
    return tEntry;
}
// Trans-lunar injection from the parking orbit: the burn is centred on the point opposite to where the
// Moon will be when the transfer ellipse reaches its distance.
struct Tli {
    bool ok = false;
    double tBurn = 0; // seconds until the vessel is at that point
    double dv = 0;
    double moonDist = 0;
};
Tli planTli(const World& W, const Vessel& v) {
    Tli out;
    const Universe& U = W.U;
    int moon = U.find("Moon");
    const Body& e = bodyOf(W, v);
    Conic c = conicOf(W, v);
    if (moon < 0 || U.bodies[moon].parent != v.ref || !c.valid || c.e >= 0.5) return out;
    std::vector<BodyState> S2;
    double tWait = 0, tTransfer = 5 * DAY, ra = U.bodies[moon].a, rb = c.a;
    for (int it = 0; it < 6; it++) {
        U.computeStatesFor(W.t + tWait + tTransfer, U.relevant[v.ref], S2);
        Vec3d m = U.relPos(S2, moon, v.ref);
        ra = len(m);
        Vec3d inPlane = m - c.H * dot(m, c.H);
        if (len(inPlane) < 1) return out;
        Vec3d p = -norm(inPlane);
        double nu = std::atan2(dot(p, c.Q), dot(p, c.P));
        rb = c.p / (1 + c.e * std::cos(nu));
        tWait = timeToTrueAnomaly(c, nu);
        if (tWait < 0) return out;
        double a = 0.5 * (rb + ra);
        tTransfer = PI * std::sqrt(a * a * a / e.GM);
    }
    out.ok = true;
    out.tBurn = tWait;
    out.moonDist = ra;
    out.dv = apsisSpeed(e.GM, rb, ra) - std::sqrt(e.GM * (2 / rb - 1 / c.a));
    return out;
}
// Lift-off time for the Moon: a launch due east puts the orbit plane through the launch site with its
// normal along the local north. The window is the moment that plane contains the point where the Moon
// will be on arrival. Of the windows of the next two days the one with the Sun highest is taken.
double lunarLaunchWindow(const World& W, double tFrom) {
    const Universe& U = W.U;
    int earth = U.find("Earth"), moon = U.find("Moon"), sun = U.find("Sun");
    if (earth < 0 || moon < 0) return tFrom;
    const LaunchSite& site = LAUNCH_SITES[std::clamp(W.settings.launchSite, 0, LAUNCH_SITE_COUNT - 1)];
    double lat = site.latDeg * DEG, lon = site.lonDeg * DEG;
    Vec3d upF{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
    Vec3d pole = U.bodies[earth].poleFrame.col(2);
    const double arrive = 4.82 * DAY; // ascent and parking orbit (about 2 h) plus the transfer (4.7 days)
    std::vector<BodyState> S2;
    auto upAt = [&](double t) { return U.bodyRot(earth, t) * upF; };
    auto miss = [&](double t) {
        Vec3d up = upAt(t);
        Vec3d north = norm(pole - up * dot(pole, up));
        U.computeStatesFor(t + arrive, U.relevant[earth], S2);
        return dot(north, norm(U.relPos(S2, moon, earth)));
    };
    double bestT = tFrom, bestEl = -1e9;
    const double step = 120;
    double f0 = miss(tFrom);
    for (double t = tFrom; t < tFrom + 2 * DAY; t += step) {
        double f1 = miss(t + step);
        if ((f0 < 0) != (f1 < 0)) {
            double lo = t, hi = t + step, flo = f0;
            for (int it = 0; it < 30; it++) {
                double mid = 0.5 * (lo + hi), fm = miss(mid);
                if ((fm < 0) == (flo < 0)) { lo = mid; flo = fm; }
                else hi = mid;
            }
            double tw = 0.5 * (lo + hi), el = 0;
            if (sun >= 0) {
                U.computeStatesFor(tw, U.relevant[earth], S2);
                el = dot(upAt(tw), norm(U.relPos(S2, sun, earth)));
            }
            if (el > bestEl) { bestEl = el; bestT = tw; }
        }
        f0 = f1;
    }
    return bestT;
}

// predicted closest approach to the Moon after a velocity change dV: the periapsis of the encounter, or
// (no encounter) the closest distance of the coasting path to the Moon's centre minus its radius
double lunarMiss(const World& W, const Vessel& v, const Vec3d& dV) {
    const Universe& U = W.U;
    int moon = U.find("Moon");
    const Body& mb = U.bodies[moon];
    Encounter e;
    if (predictEncounter(U, v.ref, v.r, v.v + dV, W.t, e) && e.body == moon) return stateToConic(e.r, e.v, mb.GM).pe - mb.radius;
    const Body& rb = U.bodies[v.ref];
    Vec3d r = v.r, vv = v.v + dV;
    std::vector<BodyState> S2;
    double best = 1e30;
    for (int k = 1; k <= 300; k++) {
        keplerPropagate(r, vv, rb.GM, 0.025 * DAY);
        U.computeStatesFor(W.t + k * 0.025 * DAY, U.relevant[v.ref], S2);
        best = std::min(best, len(r - U.relPos(S2, moon, v.ref)));
    }
    return best - mb.radius;
}
// the RCS axis (craft frame, 0 +x .. 5 -z) whose push brings the predicted periapsis closest to 1,500 km
int bestTrimAxis(const World& W, const Vessel& v) {
    if (v.ref != W.U.find("Earth")) return -1;
    auto cost = [](double miss) { return miss < 0 ? 1500e3 - miss + 1e7 : std::fabs(miss - 1500e3); };
    double base = cost(lunarMiss(W, v, {}));
    int best = -1;
    static const Vec3d AX[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (int a = 0; a < 6; a++) {
        double c = cost(lunarMiss(W, v, v.q.rotate(AX[a]) * 1.0)); // a 1 m/s push along the axis
        if (c < base) { base = c; best = a; }
    }
    return best;
}

// "{key:stage}" in a step text becomes the key currently bound to that action
std::string expandKeys(std::string s) {
    for (size_t at = s.find("{key:"); at != std::string::npos; at = s.find("{key:", at)) {
        size_t end = s.find('}', at);
        if (end == std::string::npos) break;
        std::string id = s.substr(at + 5, end - at - 5), label = "?";
        for (int a = 0; a < keys::N; a++)
            if (id == keys::info((keys::A)a).id) {
                label = keys::label((keys::A)a);
                if (label.empty()) label = tr("Unbound");
            }
        s.replace(at, end - at + 1, label);
        at += label.size();
    }
    return s;
}
std::string keyOf(keys::A a) {
    std::string l = keys::label(a);
    return l.empty() ? tr("Unbound") : l;
}

// ---------------------------------------------------------------- setups
Design templateNamed(const char* name) {
    for (const Design& d : builtinTemplates())
        if (d.name == name) return d;
    return builtinTemplates()[0];
}
// the upper part of a stack: everything above its (keep + 1)-th decoupler from the top
Design upperStages(Design d, int keep) {
    int dec = CATALOG.find("decoupler"), seen = 0;
    for (size_t i = 0; i < d.stack.size(); i++)
        if (d.stack[i].def == dec && ++seen > keep) {
            d.stack.resize(i);
            break;
        }
    return d;
}
void freshGame(App& app, bool hyperspace, int launchSite = -1) {
    Settings s = app.world.settings;
    if (hyperspace) s.hyperspace = true;
    if (launchSite >= 0) s.launchSite = launchSite; // tutorials that depend on the latitude of the pad
    app.world.newGame(s);
    app.world.recordsOn = false; // (a tutorial is no map: it earns no achievements)
    app.world.missionLog.clear();
    app.gameStarted = true;
}
bool launchDesign(App& app, const Design& d, double orbitAlt) {
    app.design = d;
    app.lastLaunchBody = -2; // "Revert to launch" goes back to the pad (not to an earlier aircraft start)
    app.lastLaunched = craftFromDesign(d);
    World& W = app.world;
    int idx = W.launch(app.lastLaunched);
    if (idx < 0 || idx >= (int)W.vessels.size()) return false;
    Vessel& v = W.vessels[idx];
    if (orbitAlt > 0) W.placeInOrbit(v, orbitAlt);
    S.vesselId = v.id;
    app.screen = Screen::Flight;
    flightOnEnter(app);
    return true;
}

// ---------------------------------------------------------------- steps shared by the launch tutorials
// the ascent is flown by the SAS "Ascent guidance" (the manual way is taught in "Reaching orbit")
Step stepAscentGuided() {
    return {"The ascent guidance flies the gravity turn, coasts to the apoapsis and burns into a 200 km parking orbit; it works the throttle itself. Your job: when a stage runs dry, drop it ({key:stage}). Above the atmosphere you may speed up the coast with time warp ({key:warp_faster}).",
            [](Ctx& c) { return c.v && !c.v->landed && c.v->sas != SAS::Ascent && peAlt(c.W, *c.v) > 150e3 && c.v->throttle < 0.01; },
            [](Ctx& c) {
                if (!c.v) return std::string();
                if (c.v->sas != SAS::Ascent && peAlt(c.W, *c.v) < 150e3) return tr("Click Ascent guidance in the SAS panel again");
                static const char* phase[] = {"Boost", "Coast to the apoapsis", "Orbit insertion"};
                return trf("{0}  ·  apoapsis {1}  ·  periapsis {2}", tr(phase[std::clamp(c.v->ascentPhase, 0, 2)]), formatDist(apAlt(c.W, *c.v)),
                           formatDist(std::max(-999e3, peAlt(c.W, *c.v))));
            }};
}
// wait for the equator crossing where the plane is turned (the higher one), nose along the SAS mode
// that flattens the orbit there
Step stepPlaneWait(const char* text) {
    return {text,
            [](Ctx& c) {
                if (!c.v) return false;
                Node n = nodeAhead(c.W, *c.v, true);
                if (!n.ok) return false;
                double tilt = tiltDeg(c.W, *c.v);
                SAS mode = flattenMode(c.W, *c.v, n.northbound);
                double tIgn = igniteIn(n.t, burnTime(c.W, *c.v, 2 * n.speed * std::sin(tilt * DEG / 2)), conicOf(c.W, *c.v).period);
                G.tIgnite = tIgn;
                G.sas = (int)mode;
                G.tilt = tilt;
                if (tIgn > 0) c.W.warpHold = c.W.t + tIgn;
                return c.v->sas == mode && tIgn < 10;
            },
            [](Ctx& c) {
                if (!c.v || G.sas == 0) return std::string();
                if (G.tIgnite <= 0) return trf("Click {0}  ·  ignite now", tr(sasName((SAS)G.sas)));
                return trf("Click {0}  ·  ignite in {1}", tr(sasName((SAS)G.sas)), formatTime(G.tIgnite));
            }};
}
// the plane change itself: burn until the tilt is below 1 degree
Step stepPlaneBurn(const char* text) {
    return {text,
            [](Ctx& c) {
                if (!c.v) return false;
                double tilt = tiltDeg(c.W, *c.v);
                G.tilt = tilt;
                G.tIgnite = 0;
                S.best = std::min(S.best, tilt);
                return tilt < 2.0 && c.v->throttle < 0.01 && peAlt(c.W, *c.v) > 15e3; // a person needs a window of a few seconds
            },
            [](Ctx& c) {
                if (!c.v) return std::string();
                if (G.tilt > S.best + 0.4 && c.v->throttle > 0) return tr("The tilt is rising again: cut the engine");
                return trf("Tilt against the equator now {0}", fmtd("%.2f°", G.tilt));
            }};
}

std::vector<Tutorial> build() {
    std::vector<Tutorial> T;
    // ---------------------------------------------------------- 1: first launch
    T.push_back({"First launch", "Launch a rocket from the factory and reach space (100 km).",
                 [](App& app) {
                     freshGame(app, false);
                     app.design = templateNamed("Kestrel Orbiter");
                     app.screen = Screen::Factory;
                     S.vesselId = -1;
                     return true;
                 },
                 {
                     {"This is the Engineering Factory. The Kestrel Orbiter is assembled on the stand: a two-stage rocket with a probe core and solar panels. Click LAUNCH at the top right to roll it out to the pad.",
                      [](Ctx& c) {
                          if (c.app.screen != Screen::Flight || !c.v) return false;
                          S.vesselId = c.v->id;
                          return true;
                      }, nullptr},
                     {"You are on the launch pad. Right-drag to orbit the camera around the rocket and turn the mouse wheel to zoom. Click Next when you are ready.",
                      nullptr, nullptr},
                     {"Set the throttle to full: press Z. (Shift and Ctrl raise and lower it gradually, X cuts it.)",
                      [](Ctx& c) { return c.v && c.v->throttle > 0.99; }, nullptr},
                     {"Ignite the engines and lift off: press Space. The clamps release once the thrust exceeds the weight.",
                      [](Ctx& c) { return c.v && c.v->ignited && !c.v->landed; }, nullptr},
                     {"SAS Stability is on and holds the rocket upright. Watch the altitude in the top-right panel and climb to 10 km.",
                      [](Ctx& c) { return c.v && altOf(c.W, *c.v) > 10e3; }, nullptr},
                     {"Open the map view with M to see your trajectory around Earth.",
                      [](Ctx&) { return flightMapMode(); }, nullptr},
                     {"Press M again to return to the flight view.",
                      [](Ctx&) { return !flightMapMode(); }, nullptr},
                     {"Keep climbing until you cross the Karman line at 100 km, the edge of space. If the first stage runs dry, press Space to drop it and Space again to light the next engine.",
                      [](Ctx& c) { return c.v && altOf(c.W, *c.v) >= 100e3; }, nullptr},
                 },
                 "You reached space! Next, try the tutorial \"Reaching orbit\"."});

    // ---------------------------------------------------------- 2: reaching orbit
    T.push_back({"Reaching orbit", "Gravity turn, staging and circularising into a low Earth orbit.",
                 [](App& app) {
                     freshGame(app, false);
                     return launchDesign(app, templateNamed("Kestrel Orbiter"), 0);
                 },
                 {
                     {"To orbit Earth you need about 7.8 km/s of horizontal speed above the atmosphere (140 km here). You will climb, tilt east into a gravity turn and circularise at apoapsis.",
                      nullptr, nullptr},
                     {"Click Ascent guidance below the SAS buttons, then full throttle ({key:throttle_full}) and lift off ({key:stage}).",
                      [](Ctx& c) { return c.v && c.v->sas == SAS::Ascent && c.v->ignited && !c.v->landed; }, nullptr},
                     {"Watch the navball: the guidance flies straight up, then tilts the nose east and lets gravity bend the path over (the gravity turn). Above the thick air it follows the velocity, so the rocket is never pushed sideways through the air. It cuts the engine when the apoapsis reaches 200 km and burns again near it, until the periapsis is in space as well.",
                      nullptr, nullptr},
                     stepAscentGuided(),
                     {"Open the map view with M to see your trajectory around Earth.",
                      [](Ctx&) { return flightMapMode(); }, nullptr},
                 },
                 "You are in orbit! Next: \"Manual ascent\": the same flight by hand."});

    // ---------------------------------------------------------- 3: operating in space
    T.push_back({"Operating in space", "Solar panels, SAS modes, RCS, targets and warping along the orbit.",
                 [](App& app) {
                     freshGame(app, false);
                     Design d = upperStages(templateNamed("Kestrel Orbiter"), 0);
                     d.name = "Kestrel Probe";
                     int rcs = CATALOG.find("rcs_22"), mono = CATALOG.find("tank_mono");
                     if (d.stack.size() >= 2 && rcs >= 0) d.stack[1].radial.push_back({rcs, 4});
                     if (mono >= 0 && d.stack.size() >= 2) {
                         PartInst m = makeInst(mono);
                         m.diam = m.diam2 = 3.7;
                         m.length = 0.6;
                         d.stack.insert(d.stack.begin() + 2, m);
                     }
                     return launchDesign(app, d, 250e3);
                 },
                 {
                     {"You are in a 250 km orbit. This probe has solar panels, RCS thrusters and a hydrazine tank. You will practise attitude control and the map.",
                      nullptr, nullptr},
                     {"Deploy the solar panels: press U (or the Solar button left of the navball). They charge the battery in sunlight.",
                      [](Ctx& c) { return c.v && c.v->solarDeployed; }, nullptr},
                     {"Point the probe backwards along its orbit: click Retrograde in the SAS panel and wait until the nose settles.",
                      [](Ctx& c) { return c.v && c.v->sas == SAS::Retrograde && retroErrDeg(c.W, *c.v) < 10; },
                      [](Ctx& c) { return c.v && c.v->sas == SAS::Retrograde ? trf("Off by {0}", fmtd("%.0f deg", retroErrDeg(c.W, *c.v))) : std::string(); }},
                     {"Turn the RCS thrusters on with R.",
                      [](Ctx& c) { return c.v && c.v->rcs; }, nullptr},
                     {"Translate with RCS: hold H / N (forward / back), I / K (up / down) or J / L (left / right). The puffs show which thrusters fire.",
                      [](Ctx& c) {
                          if (!c.v || !c.v->rcs) return false;
                          const ControlInput& in = c.W.input;
                          if (std::fabs(in.tx) + std::fabs(in.ty) + std::fabs(in.tz) > 0) S.acc += c.dt;
                          return S.acc > 1.5;
                      }, nullptr},
                     {"Open the map with M.",
                      [](Ctx&) { return flightMapMode(); }, nullptr},
                     {"Set the Moon as your target: right-click it in the map (or select it in the body list and click Target). A target marker appears on the navball.",
                      [](Ctx& c) { return c.v && c.v->target == c.W.U.find("Moon"); }, nullptr},
                     {"Hover your orbit line and click a point on it: time warps there and returns to x1 on arrival.",
                      [](Ctx& c) {
                          if (c.W.warpTo > 0) S.flag = true;
                          return S.flag && c.W.warpTo <= 0;
                      }, nullptr},
                     {"Press M to return to the flight view. The mission log (O) lists every notification.",
                      [](Ctx&) { return !flightMapMode(); }, nullptr},
                 },
                 "Well done! Next: \"Return and landing\"."});

    // ---------------------------------------------------------- 4: return and landing
    T.push_back({"Return and landing", "Deorbit burn, atmospheric entry and a parachute landing.",
                 [](App& app) {
                     freshGame(app, false);
                     Design d = upperStages(templateNamed("Aurora Crew Mission"), 1);
                     d.name = "Aurora Capsule";
                     return launchDesign(app, d, 220e3);
                 },
                 {
                     {"The Aurora capsule is in orbit with its service module. To come home, lower the periapsis into the atmosphere, drop the service module and let the heat shield and the parachute do the rest.",
                      nullptr, nullptr},
                     {"Click Retrograde in the SAS panel so that the engines push against the direction of travel.",
                      [](Ctx& c) { return c.v && c.v->sas == SAS::Retrograde && retroErrDeg(c.W, *c.v) < 10; },
                      [](Ctx& c) { return c.v && c.v->sas == SAS::Retrograde ? trf("Off by {0}", fmtd("%.0f deg", retroErrDeg(c.W, *c.v))) : std::string(); }},
                     {"Press Z for full throttle, then Space to ignite the service module engines.",
                      [](Ctx& c) { return c.v && c.v->ignited && c.v->throttle > 0.5; }, nullptr},
                     {"Burn until Periapsis drops below 50 km, then cut the engines with X.",
                      [](Ctx& c) { return c.v && peAlt(c.W, *c.v) < 50e3 && c.v->throttle < 0.01; },
                      [](Ctx& c) { return c.v ? trf("Periapsis now {0}", formatDist(peAlt(c.W, *c.v))) : std::string(); }},
                     {"Separate the service module: press Space. Only the capsule with its heat shield enters the atmosphere.",
                      [](Ctx& c) { return c.v && c.v->stage != S.stage0; }, nullptr},
                     {"Keep SAS on Retrograde so that the heat shield faces the airflow. Use time warp (.) to reach the atmosphere at 140 km.",
                      [](Ctx& c) { return c.v && altOf(c.W, *c.v) < bodyOf(c.W, *c.v).atmTop; }, nullptr},
                     {"Ride out the entry heating. Open the parachute (P) once the dynamic pressure has dropped below 20 kPa; the hint below tells you when.",
                      [](Ctx& c) { return c.v && c.v->chuteDeployed; },
                      [](Ctx& c) {
                          if (!c.v) return std::string();
                          double q = c.v->tDynP;
                          if (altOf(c.W, *c.v) > 40e3) return tr("Still too high: wait for the lower atmosphere");
                          return q > 20000 ? trf("Too fast for the parachute: dynamic pressure {0} (it tears above 25 kPa)", fmtd("%.1f kPa", q / 1000))
                                           : tr("Slow enough: press P");
                      }},
                     {"Descend under the parachute until touchdown.",
                      [](Ctx& c) { return c.v && c.v->landed; }, nullptr},
                 },
                 "Welcome home! Next: \"Equatorial orbit\"."});

    // ---------------------------------------------------------- 5: hyperspace
    T.push_back({"Hyperspace", "Jump to another world with the Hyperspace Drive.",
                 [](App& app) {
                     freshGame(app, true);
                     Design d = upperStages(templateNamed("Hyperspace Explorer HX"), 0);
                     d.name = "HX Probe";
                     return launchDesign(app, d, 300e3);
                 },
                 {
                     {"This probe carries a Hyperspace Drive. It jumps between worlds and star systems, but only works in space and needs at least 20% battery charge.",
                      nullptr, nullptr},
                     {"Open the hyperspace menu: press Y (or the hyperspace button in the bottom-left toolbar).",
                      [](Ctx&) { return flightHyperOpen(); }, nullptr},
                     {"Choose a destination, for example Mars, and click ENGAGE.",
                      [](Ctx& c) { return c.W.hyper.active; }, nullptr},
                     {"Hold on: the drive turns the probe toward the destination and jumps.",
                      [](Ctx& c) { return !c.W.hyper.active && c.v && c.v->ref != c.W.U.find("Earth"); }, nullptr},
                 },
                 "You arrived at a new world and completed every tutorial. Try designing your own craft in the factory next."});

    // ---------------------------------------------------------- 6: equatorial orbit
    // The way real geostationary launches from Tanegashima or Cape Canaveral go: parking orbit at the
    // latitude of the pad, transfer burn over the equator, plane change far out where the vessel is slow.
    T.push_back({"Equatorial orbit", "From a launch site at 30° north to an orbit over the equator: transfer orbit and plane change.",
                 [](App& app) {
                     freshGame(app, false, 0);
                     return launchDesign(app, templateNamed("Kestrel Orbiter"), 0);
                 },
                 {
                     {"Tanegashima lies at 30.4° north, so a launch due east leads to an orbit tilted 30.4° against the equator. Turning the plane in a low orbit would cost about 4 km/s. Geostationary launches turn it far out instead, where the vessel is slow: first raise the apoapsis over the equator, then turn the plane there.",
                      nullptr, nullptr},
                     {"Click Ascent guidance below the SAS buttons, then full throttle ({key:throttle_full}) and lift off ({key:stage}).",
                      [](Ctx& c) { return c.v && c.v->sas == SAS::Ascent && c.v->ignited && !c.v->landed; }, nullptr},
                     stepAscentGuided(),
                     {"Inclination in the orbit panel shows about 30°. The next burn has to be centred on an equator crossing, so that the new apoapsis lies over the equator as well. Click Prograde in the SAS panel and use time warp ({key:warp_faster}): the hint counts down and the warp slows down by itself.",
                      [](Ctx& c) {
                          if (!c.v || c.v->landed) return false;
                          Node n = nodeAhead(c.W, *c.v, false);
                          if (!n.ok) return false;
                          const Body& b = bodyOf(c.W, *c.v);
                          double dv = apsisSpeed(b.GM, n.radius, geoRadius(b)) - n.speed;
                          double tIgn = igniteIn(n.t, burnTime(c.W, *c.v, dv), conicOf(c.W, *c.v).period / 2);
                          G.tIgnite = tIgn;
                          G.sas = (int)SAS::Prograde;
                          if (tIgn > 0) c.W.warpHold = c.W.t + tIgn;
                          return c.v->sas == SAS::Prograde && tIgn < 10;
                      },
                      [](Ctx& c) {
                          if (!c.v || G.sas == 0) return std::string();
                          return G.tIgnite <= 0 ? tr("Over the equator: ignite now") : trf("Equator crossing: ignite in {0}", formatTime(G.tIgnite));
                      }},
                     {"Ignite now: full throttle ({key:throttle_full}; {key:stage} if the engine is off). Burn until Apoapsis reaches 35,786 km, the altitude of the geostationary orbit, then cut the engine ({key:throttle_cut}).",
                      [](Ctx& c) {
                          if (!c.v) return false;
                          G.tIgnite = 0;
                          G.sas = (int)SAS::Prograde;
                          return apAlt(c.W, *c.v) >= 35.0e6 && c.v->throttle < 0.01;
                      },
                      [](Ctx& c) { return c.v ? trf("Apoapsis now {0} of {1}", formatDist(apAlt(c.W, *c.v)), formatDist(35786e3)) : std::string(); }},
                     stepPlaneWait("Coast up to the equator crossing near the apoapsis, about five hours away. Click the SAS mode named in the hint: it turns the nose at right angles to the orbit plane. Then use time warp. Solar panels ({key:solar}) keep the battery charged on the way."),
                     stepPlaneBurn("Turn the plane: full throttle ({key:throttle_full}) and watch Inclination in the orbit panel fall. Lower the throttle ({key:throttle_down}) as it approaches 0° and cut the engine ({key:throttle_cut}) below 2°."),
                 },
                 "You are in an equatorial orbit. A geostationary satellite would now circularise at the apoapsis and stay above one point of the equator. Next: \"To the Moon\"."});

    // ---------------------------------------------------------- 7: to the Moon
    // Launch window -> parking orbit -> trans-lunar injection -> midcourse correction -> lunar orbit
    // insertion -> plane change into the lunar equator
    T.push_back({"To the Moon", "Launch window, trans-lunar injection, course correction and an orbit over the Moon's equator.",
                 [](App& app) {
                     freshGame(app, false, 0);
                     // RCS quads and hydrazine on the lander for the course corrections
                     Design d = templateNamed("Selene Lander");
                     int rcs = CATALOG.find("rcs_440"), mono = CATALOG.find("tank_mono");
                     if (d.stack.size() >= 3 && rcs >= 0 && mono >= 0) {
                         d.stack[1].radial.push_back({rcs, 4});
                         PartInst m = makeInst(mono);
                         m.diam = m.diam2 = 2.5;
                         m.length = 0.4;
                         d.stack.insert(d.stack.begin() + 2, m);
                         d.stack[0].length += partGeom(m).height; // the fairing covers it too
                     }
                     World& W = app.world;
                     S.mark = lunarLaunchWindow(W, W.t + 600);
                     W.t = S.mark - 150; // the countdown starts 2.5 minutes before the window
                     W.refreshStates();
                     return launchDesign(app, d, 0);
                 },
                 {
                     {"The Moon is 384,400 km away and travels 1 km/s along its orbit, so you aim at the point where it will be in five days. The lift-off is timed (launch window) so that your orbit plane passes through that point. Then follow the burn for the Moon, a course correction, braking into lunar orbit and turning that orbit over the lunar equator.",
                      nullptr,
                      [](Ctx& c) { return c.W.t < S.mark ? trf("Launch window in {0}", formatTime(S.mark - c.W.t)) : tr("Launch window open: lift off now"); }},
                     {"Click Ascent guidance below the SAS buttons. Lift off when the countdown in the hint reaches zero: full throttle ({key:throttle_full}), then {key:stage}. A minute early or late does not matter.",
                      [](Ctx& c) {
                          G.tIgnite = S.mark - c.W.t;
                          if (G.tIgnite > 0) c.W.warpHold = S.mark;
                          return c.v && c.v->sas == SAS::Ascent && c.v->ignited && !c.v->landed;
                      },
                      [](Ctx& c) {
                          double late = c.W.t - S.mark;
                          if (late < 0) return trf("Launch window in {0}", formatTime(-late));
                          return late < 180 ? tr("Launch window open: lift off now") : trf("Launch window missed by {0}: restart for a cheaper flight", formatTime(late));
                      }},
                     stepAscentGuided(),
                     {"The burn for the Moon (trans-lunar injection) has to happen on the side of Earth opposite the point where the Moon will be. Click Prograde in the SAS panel and use time warp ({key:warp_faster}): the hint counts down to the ignition and the warp slows down by itself.",
                      [](Ctx& c) {
                          if (!c.v || c.v->landed) return false;
                          Tli p = planTli(c.W, *c.v);
                          if (!p.ok) return false;
                          S.acc = burnTimeStages(c.W, *c.v, p.dv);
                          S.target = p.moonDist;
                          // centred on the impulse: half of the velocity change is delivered at the planned point
                          double tIgn = igniteIn(p.tBurn, 2 * burnTimeStages(c.W, *c.v, p.dv / 2), conicOf(c.W, *c.v).period);
                          G.tIgnite = tIgn;
                          G.sas = (int)SAS::Prograde;
                          if (tIgn > 0) c.W.warpHold = c.W.t + tIgn;
                          return c.v->sas == SAS::Prograde && tIgn < 10;
                      },
                      [](Ctx& c) {
                          if (!c.v || G.sas == 0) return std::string();
                          return G.tIgnite <= 0 ? tr("Ignite now") : trf("Ignite in {0}  ·  the burn takes about {1}", formatTime(G.tIgnite), formatTime(S.acc));
                      }},
                     {"Ignite now: full throttle ({key:throttle_full}; {key:stage} if the engine is off). If the stage runs dry, drop it and continue with the next one. Watch Apoapsis: the hint shows the target, the distance of the Moon on arrival. Lower the throttle ({key:throttle_down}) as it comes close and cut the engine ({key:throttle_cut}) when it is reached.",
                      [](Ctx& c) {
                          if (!c.v) return false;
                          G.tIgnite = 0;
                          G.sas = (int)SAS::Prograde;
                          lunarEncounter(c.W, c.v);
                          G.target = S.target;
                          Conic k = conicOf(c.W, *c.v);
                          bool done = S.target > 0 && k.valid && k.e < 1 && k.ap >= S.target * 0.995 && c.v->throttle < 0.01;
                          if (done && S.tliEnd <= 0) S.tliEnd = c.W.t;
                          return done;
                      },
                      [](Ctx& c) {
                          if (!c.v) return std::string();
                          if (S.target > 0) {
                              Conic k = conicOf(c.W, *c.v);
                              const double R = bodyOf(c.W, *c.v).radius;
                              if (k.valid && k.e < 1 && k.ap >= S.target * 0.995) return trf("Apoapsis reached: cut the engine ({0})", keyOf(keys::A::ThrottleCut));
                              return trf("Apoapsis {0} of {1}", formatDist(apAlt(c.W, *c.v)), formatDist(S.target - R));
                          }
                          int moon = c.W.U.find("Moon");
                          double md = len(c.W.U.relPos(c.W.states, moon, c.v->ref)), ap = apAlt(c.W, *c.v);
                          if (ap > md * 1.2) return tr("Too fast: burn Retrograde until an encounter appears");
                          return trf("Apoapsis {0}  ·  Moon at {1}", formatDist(ap), formatDist(md));
                      }},
                     {"You are on your way. Deploy the solar panels ({key:solar}): the trip takes about five days and the avionics need power.",
                      [](Ctx& c) { return c.v && c.v->solarDeployed; }, nullptr},
                     {"Coast for a day ({key:warp_faster}). Far from Earth a small push moves the arrival point a long way, so the course is corrected there.",
                      [](Ctx& c) {
                          if (S.tliEnd <= 0) S.tliEnd = c.W.t; // the coast starts right after the burn
                          double due = S.tliEnd + DAY;
                          G.tIgnite = S.tliEnd > 0 ? due - c.W.t : 1e18;
                          if (S.tliEnd > 0 && c.W.t < due) c.W.warpHold = due;
                          return S.tliEnd > 0 && c.W.t >= due;
                      },
                      [](Ctx& c) { return S.tliEnd > 0 ? trf("Course correction in {0}", formatTime(S.tliEnd + DAY - c.W.t)) : std::string(); }},
                     {"Mid-course correction: turn the RCS on ({key:rcs}) and click Prograde in the SAS panel. The hint names the RCS key to hold; let go when the predicted lunar periapsis is between 100 and 5,000 km. The tilt of the pass does not matter yet: the orbit is turned over the lunar equator at the end.",
                      [](Ctx& c) {
                          if (!c.v) return false;
                          lunarEncounter(c.W, c.v);
                          const ControlInput& in = c.W.input;
                          bool idle = std::fabs(in.tx) + std::fabs(in.ty) + std::fabs(in.tz) < 0.01 && c.v->throttle < 0.01;
                          bool good = G.enc && G.encPe >= 80e3 && G.encPe <= 5000e3;
                          S.acc = good && idle ? S.acc + c.dt : 0; // settled for a moment
                          G.trimAxis = good ? -1 : bestTrimAxis(c.W, *c.v);
                          return S.acc > 1.5;
                      },
                      [](Ctx& c) {
                          if (!c.v) return std::string();
                          std::string now = G.enc ? (G.encPe < 0 ? tr("impact") : formatDist(G.encPe)) : tr("no encounter");
                          if (G.trimAxis < 0) return trf("Let go of the keys  ·  predicted periapsis {0}", now);
                          static const keys::A AX[6] = {keys::A::TransLeft, keys::A::TransRight, keys::A::TransUp, keys::A::TransDown, keys::A::TransFwd, keys::A::TransBack};
                          return trf("Hold {0}  ·  predicted periapsis {1}", keyOf(AX[G.trimAxis]), now);
                      }},
                     {"Coast on until the Moon's gravity takes over, 66,000 km from the Moon (its sphere of influence). The orbit panel then switches to the Moon.",
                      [](Ctx& c) {
                          if (!c.v) return false;
                          double tEntry = lunarEncounter(c.W, c.v);
                          S.acc = tEntry;
                          if (bodyOf(c.W, *c.v).name == "Moon") return true;
                          if (G.enc && tEntry > 0) c.W.warpHold = c.W.t + tEntry;
                          return false;
                      },
                      [](Ctx& c) {
                          if (!c.v) return std::string();
                          return G.enc ? trf("Sphere of influence in {0}", formatTime(S.acc)) : tr("No encounter: SAS Prograde, then translate forward or back");
                      }},
                     {"You are falling toward the Moon. If Periapsis in the orbit panel is below 50 km, select SAS Radial out and push with {key:trans_fwd} to raise it. Then click Retrograde and use time warp: the hint counts down to the braking burn at the periapsis.",
                      [](Ctx& c) {
                          if (!c.v || c.v->landed) return false;
                          const Body& b = bodyOf(c.W, *c.v);
                          Conic k = conicOf(c.W, *c.v);
                          double tPe = timeToTrueAnomaly(k, 0);
                          if (b.name != "Moon" || !k.valid) return false;
                          double vPe = std::sqrt(std::max(0.0, b.GM * (2 / k.pe - 1 / k.a)));
                          double dv = vPe - apsisSpeed(b.GM, k.pe, std::max(b.radius + 10000e3, k.pe * 1.5));
                          S.acc = k.pe - b.radius;
                          double tIgn = tPe < 0 ? -1 : tPe - burnTime(c.W, *c.v, dv) / 2;
                          G.tIgnite = tIgn;
                          G.sas = (int)SAS::Retrograde;
                          bool safe = S.acc > 30e3;
                          if (tIgn > 0 && safe) c.W.warpHold = c.W.t + tIgn;
                          return safe && c.v->sas == SAS::Retrograde && tIgn < 10;
                      },
                      [](Ctx& c) {
                          if (!c.v || G.sas == 0) return std::string();
                          if (S.acc <= 30e3) return trf("Periapsis {0}: too low, raise it first", formatDist(S.acc));
                          return G.tIgnite <= 0 ? tr("Ignite now") : trf("Periapsis {0}  ·  ignite in {1}", formatDist(S.acc), formatTime(G.tIgnite));
                      }},
                     {"Brake into lunar orbit: full throttle ({key:throttle_full}; {key:stage} if the engine is off). Burn until Apoapsis has come down below 10,000 km, then cut the engine ({key:throttle_cut}).",
                      [](Ctx& c) {
                          if (!c.v) return false;
                          const Body& b = bodyOf(c.W, *c.v);
                          Conic k = conicOf(c.W, *c.v);
                          G.tIgnite = 0;
                          G.sas = (int)SAS::Retrograde;
                          S.flag = b.name == "Moon" && k.valid && k.e < 1 && k.ap < b.soi;
                          return S.flag && k.ap - b.radius < 10000e3 && k.pe - b.radius > 15e3 && c.v->throttle < 0.01;
                      },
                      [](Ctx& c) {
                          if (!c.v) return std::string();
                          return S.flag ? trf("Apoapsis now {0}", formatDist(apAlt(c.W, *c.v))) : tr("Not captured yet: keep burning");
                      }},
                     stepPlaneWait("You are in lunar orbit. Its tilt against the Moon's equator is removed where the orbit crosses the equator. Click the SAS mode named in the hint and use time warp to the crossing."),
                     stepPlaneBurn("Turn the plane: full throttle ({key:throttle_full}) until the tilt in the hint is below 2°, then cut the engine ({key:throttle_cut}). Lower the throttle ({key:throttle_down}) near the end. An inclination of 0° and of 180° are both orbits over the equator."),
                 },
                 "You are in an orbit over the Moon's equator, 384,400 km from home. One more, just for fun: \"Hyperspace\"."});

    // ---------------------------------------------------------- 8: manual ascent (no ascent guidance)
    T.push_back({"Manual ascent", "Fly to orbit by hand, without the ascent guidance: pitch programme, gravity turn and circularisation.",
                 [](App& app) {
                     freshGame(app, false);
                     return launchDesign(app, templateNamed("Kestrel Orbiter"), 0);
                 },
                 {
                     {"This time you fly the ascent yourself. A rocket has to turn from vertical to horizontal while it climbs: too steep and it wastes fuel lifting itself, too flat and the air burns it up. The hint shows the right pitch for your altitude (the pitch programme) and counts down to each burn.", nullptr, nullptr},
                     {"Full throttle ({key:throttle_full}) and lift off ({key:stage}). SAS Hold keeps the rocket pointed where you leave it.",
                      [](Ctx& c) { return c.v && c.v->ignited && !c.v->landed; }, nullptr},
                     {"Above 800 m tilt the nose east (HDG 090) with {key:pitch_down} and keep PITCH on the target in the hint: it falls from 90° to about 15° at 35 km. Small taps are enough; SAS Hold keeps the new attitude. When a stage runs dry, drop it ({key:stage}).",
                      [](Ctx& c) { return c.v && !c.v->landed && altOf(c.W, *c.v) >= 35e3; },
                      [](Ctx& c) {
                          if (!c.v) return std::string();
                          double tp = targetPitchDeg(c.W, *c.v), p = pitchDeg(c.W, *c.v);
                          std::string s = trf("Target PITCH {0}  \u00b7  now {1}  \u00b7  HDG {2}", fmtd("%.0f", tp), fmtd("%.0f", p), fmtd("%03.0f", headingDeg(c.W, *c.v)));
                          if (p > tp + 8) return s + "  \u00b7  " + tr("too steep");
                          if (p < tp - 8) return s + "  \u00b7  " + tr("too flat");
                          return s;
                      }},
                     {"Above the thick air, click Prograde in the SAS panel: the rocket now follows its velocity. Cut the engine ({key:throttle_cut}) when Apoapsis reaches 200 km.",
                      [](Ctx& c) { return c.v && c.v->sas == SAS::Prograde && apAlt(c.W, *c.v) >= 195e3 && c.v->throttle < 0.01; },
                      [](Ctx& c) { return c.v ? trf("Apoapsis now {0}", formatDist(apAlt(c.W, *c.v))) : std::string(); }},
                     {"Coast to the apoapsis (time warp {key:warp_faster}; it slows down by itself). When the hint says so, burn at full throttle ({key:throttle_full}) with SAS on Prograde until Periapsis is above 170 km, then cut the engine ({key:throttle_cut}).",
                      [](Ctx& c) {
                          if (!c.v) return false;
                          const Body& b = bodyOf(c.W, *c.v);
                          Conic k = conicOf(c.W, *c.v);
                          if (k.valid && k.e < 1 && c.v->throttle < 0.01 && peAlt(c.W, *c.v) < 170e3) {
                              double vAp = std::sqrt(std::max(0.0, b.GM * (2 / k.ap - 1 / k.a)));
                              double dv = std::sqrt(b.GM / k.ap) - vAp;
                              double tAp = timeToTrueAnomaly(k, PI);
                              double tIgn = tAp - burnTimeStages(c.W, *c.v, dv) / 2;
                              if (tIgn > k.period * 0.6) tIgn -= k.period; // just past the apoapsis: ignite now
                              G.tIgnite = tIgn;
                              G.sas = (int)SAS::Prograde;
                              if (tIgn > 0) c.W.warpHold = c.W.t + tIgn;
                          } else G.tIgnite = 0;
                          return peAlt(c.W, *c.v) > 170e3 && c.v->throttle < 0.01;
                      },
                      [](Ctx& c) {
                          if (!c.v) return std::string();
                          if (c.W.warpIndex > 0 && c.v->throttle > 0) return tr("Engines do not burn during time warp: press / for x1.");
                          if (c.v->throttle > 0 || G.tIgnite <= 0) return trf("Periapsis now {0}", formatDist(peAlt(c.W, *c.v)));
                          return trf("Ignite in {0}", formatTime(G.tIgnite));
                      }},
                 },
                 "You flew into orbit by hand. The ascent guidance does exactly this; now you know what it does for you. Next: \"Operating in space\"."});
    // the natural order: first flight, orbit (guided, then by hand), in space, home, further out, the Moon,
    // and the fictional hyperspace jump last
    std::vector<Tutorial> R;
    for (int i : {0, 1, 7, 2, 3, 5, 6, 4}) R.push_back(std::move(T[i]));
    return R;
}

const std::vector<Tutorial>& all() {
    static const std::vector<Tutorial> T = build();
    return T;
}

void enterStep(App& app) {
    S.doneAt = -1;
    S.acc = 0;
    S.flag = false;
    S.best = 1e9;
    G.target = 0;
    Vessel* v = app.world.activeVessel();
    S.stage0 = v ? v->stage : 0;
}

void complete(App& app) {
    S.finished = true;
    sfx::play(sfx::Ev::Milestone);
    app.tutorialsDone |= 1u << S.tut;
    savePrefs(app);
}

void advance(App& app) {
    const Tutorial& t = all()[S.tut];
    if (S.step + 1 >= (int)t.steps.size()) {
        complete(app);
        return;
    }
    S.step++;
    enterStep(app);
}
} // namespace

int count() { return (int)all().size(); }
const char* title(int i) { return all()[i].title; }
const char* summary(int i) { return all()[i].summary; }
bool active() { return S.tut >= 0; }
int stepIndex() { return S.step; }
const Guide& guide() { return G; }
bool stepDone() { return S.tut >= 0 && !S.finished && S.doneAt >= 0; }
void debugNext() {
    if (S.tut >= 0 && !S.finished && !all()[S.tut].steps[S.step].done) S.doneAt = GetTime() - 1;
}
bool finished() { return S.finished; }
void stop() { S = State{}; }

bool start(App& app, int i) {
    if (i < 0 || i >= count()) return false;
    leaveMap(app); // (the map being played is saved first: a tutorial is a game of its own)
    stop();
    S.tut = i;
    if (!all()[i].setup(app)) {
        stop();
        return false;
    }
    S.step = 0;
    enterStep(app);
    return true;
}

void frame(App& app) {
    app.world.warpHold = -1; // set again below by a step that waits for a moment
    G = Guide{};
    if (S.tut < 0) return;
    World& W = app.world;
    const Tutorial& t = all()[S.tut];
    Vessel* v = W.activeVessel();
    if (v && S.vesselId >= 0 && v->id != S.vesselId) {
        // the player switched vessels: follow the tutorial vessel if it still exists
        Vessel* tv = nullptr;
        for (auto& o : W.vessels)
            if (o.id == S.vesselId) tv = &o;
        v = tv;
    }
    Ctx c{app, W, v, GetFrameTime()};
    double now = GetTime();
    if (!S.finished) {
        const Step& st = t.steps[S.step];
        if (S.doneAt < 0 && st.done && st.done(c)) {
            S.doneAt = now;
            sfx::play(sfx::Ev::Toggle);
            if (W.warpHold > 0 && W.warpIndex > 0) W.setWarp(0); // the moment the step waited for has come
        }
        if (S.doneAt >= 0 && now - S.doneAt > 0.9) advance(app);
    }
    if (W.hyper.active && app.screen == Screen::Flight) return; // keep the hyperspace view clear
    if (app.hudHidden && app.screen == Screen::Flight) return;  // HUD hidden (F1): steps are still checked

    // ---------------- guide card (as tall as its text needs)
    float SW = ui::SW(), SH = ui::SH();
    const float w = 384;
    bool flash = !S.finished && S.doneAt >= 0;
    std::string body = expandKeys(tr(S.finished ? t.doneText : t.steps[S.step].text));
    // hints: losses first, then the step's own live hint, then generic ones
    std::string hint;
    Color hc = col::warn;
    if (!S.finished) {
        const Step& st = t.steps[S.step];
        if (app.screen == Screen::Flight && c.v && c.v->destroyed) {
            hint = tr("The vessel was lost: use Back 30 s in the dialog, or restart the tutorial.");
            hc = col::bad;
        } else if (st.hint) hint = st.hint(c);
        if (hint.empty() && c.v && c.v->tFlameout && c.v->throttle > 0 && !W.settings.infiniteFuel && app.screen == Screen::Flight)
            hint = tr("This stage is empty: press Space to separate it.");
    }
    bool mark = S.finished || flash; // check mark left of the text
    float bodyW = w - (mark ? 62 : 32);
    float bodyH = wrap(body, 0, 0, bodyW, 15, col::text, -1) + 4; // measured only (nothing fits above maxY)
    float hintH = hint.empty() ? 0 : wrap(hint, 0, 0, w - 32, 13, hc, -1) + 8;
    float h = S.collapsed ? 44 : std::max(232.0f, 57 + bodyH + hintH + 12 + 48);
    Rectangle r = app.screen == Screen::Factory ? Rectangle{356, SH - h - 84, w, h} : Rectangle{324, 128, w, h};
    if (r.x + r.width > SW - 360) r.x = std::max(12.0f, SW - 360 - r.width);
    panel(r, Color{16, 22, 34, 238}, 12);
    DrawRectangleRounded({r.x, r.y, 4, r.height}, 1.0f, 4, col::accent);
    icon(Icon::Book, {r.x + 14, r.y + 12, 20, 20}, col::accent);
    std::string head = tr("TUTORIAL") + "  ·  " + tr(t.title);
    text(fit(head, w - 128, 13, true), r.x + 42, r.y + 14, 13, col::accent, true);
    int nSteps = (int)t.steps.size();
    std::string prog = S.finished ? tr("Complete") : std::to_string(S.step + 1) + " / " + std::to_string(nSteps);
    textR(prog, r.x + w - 50, r.y + 14, 13, col::dim, true);
    if (iconButton({r.x + w - 40, r.y + 8, 28, 28}, S.collapsed ? Icon::Plus : Icon::Minus, false, S.collapsed ? "Show" : "Hide"))
        S.collapsed = !S.collapsed;
    if (S.collapsed) return;
    float frac = S.finished ? 1.f : (float)(S.step + (S.doneAt >= 0 ? 1 : 0)) / nSteps;
    bar({r.x + 16, r.y + 40, w - 32, 5}, frac, col::accent);

    float ty = r.y + 56;
    if (mark) icon(Icon::Check, {r.x + 16, ty, 22, 22}, col::good);
    wrap(body, r.x + (mark ? 46 : 16), ty + 1, bodyW, 15, flash ? col::good : col::text);
    if (!hint.empty()) wrap(hint, r.x + 16, r.y + h - 48 - 12 - hintH + 6, w - 32, 13, hc);
    // buttons
    float by = r.y + h - 48;
    if (S.finished) {
        bool hasNext = S.tut + 1 < count();
        if (hasNext && primaryButton({r.x + w - 176, by, 160, 34}, "Next tutorial", Icon::Forward)) {
            start(app, S.tut + 1);
            return;
        }
        if (button({r.x + 16, by, 120, 34}, "Close", false, true, Icon::Close)) {
            stop();
            return;
        }
        return;
    }
    const Step& st = t.steps[S.step];
    if (!st.done) {
        if (primaryButton({r.x + w - 126, by, 110, 34}, "Next", Icon::Forward) && S.doneAt < 0) advance(app);
    } else if (button({r.x + w - 126, by, 110, 34}, "Skip step", false, S.doneAt < 0, Icon::Forward)) advance(app);
    if (button({r.x + 16, by, 112, 34}, "Restart", false, true, Icon::Backward)) {
        start(app, S.tut);
        return;
    }
    if (button({r.x + 136, by, 96, 34}, "Exit", false, true, Icon::Close)) stop();
}

void menuModal(App& app) {
    if (!menuOpen) return;
    dim();
    float SW = ui::SW(), SH = ui::SH();
    int n = count();
    float H = 150 + n * 76.0f;
    Rectangle r{(SW - 640) / 2, (SH - H) / 2, 640, H};
    panel(r, Color{20, 24, 36, 252}, 14);
    icon(Icon::Book, {r.x + 22, r.y + 20, 26, 26}, col::accent);
    text("Tutorials", r.x + 58, r.y + 18, 24, col::text, true);
    if (iconButton({r.x + r.width - 48, r.y + 16, 32, 32}, Icon::Close) || IsKeyPressed(KEY_ESCAPE)) {
        menuOpen = false;
        return;
    }
    std::string note = app.gameStarted ? tr("Starting a tutorial replaces the current game. Quicksave (F5) first to keep it.")
                                       : tr("Guided missions that teach the controls step by step.");
    text(fit(note, r.width - 48, 13), r.x + 24, r.y + 58, 13, app.gameStarted ? col::warn : col::dim);
    float y = r.y + 88;
    for (int i = 0; i < n; i++) {
        Rectangle row{r.x + 16, y, r.width - 32, 68};
        DrawRectangleRounded(row, 0.15f, 6, Color{26, 32, 46, 255});
        bool done = (app.tutorialsDone >> i) & 1u;
        Rectangle badge{row.x + 12, row.y + 18, 32, 32};
        DrawCircleV({badge.x + 16, badge.y + 16}, 16, done ? Color{30, 70, 48, 255} : Color{36, 44, 62, 255});
        if (done) icon(Icon::Check, {badge.x + 7, badge.y + 7, 18, 18}, col::good);
        else textC(std::to_string(i + 1), badge, 15, col::text, true);
        text(fit(tr(title(i)), row.width - 220, 17, true), row.x + 58, row.y + 12, 17, col::text, true);
        text(fit(tr(summary(i)), row.width - 220, 13), row.x + 58, row.y + 38, 13, col::dim);
        if (primaryButton({row.x + row.width - 132, row.y + 16, 120, 36}, done ? "Replay" : "Start", Icon::Play)) {
            menuOpen = false;
            if (!start(app, i)) app.setStatus("Could not start the tutorial");
            return;
        }
        y += 76;
    }
    text("Progress is saved. The tutorial guide appears in the flight view and can be hidden.", r.x + 24, r.y + r.height - 38, 12, col::faint);
}
} // namespace tut
