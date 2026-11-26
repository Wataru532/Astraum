// Scripted self-test: launches a template, flies a gravity turn to orbit, checks physics, captures screenshots.
// Run: Astraum.exe --autotest <output-directory>
#include <cstdio>
#include <algorithm>
#include <cctype>
#include <functional>
#include <map>
#include <set>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include "app.hpp"
#include "screens.hpp"
#include "session.hpp"
#include "storage.hpp"
#include "navguide.hpp"
#include "mission.hpp"
#include <chrono>
#include "achieve.hpp"
#include "tutorial.hpp"
#include "crash.hpp"
#include "gpumon.hpp"
#include "gputime.hpp"
#include "mem.hpp"
#include "audio.hpp"
#include "features.hpp"
#include "render.hpp"
#include "rl.hpp"
#include "ui.hpp"
#include "keys.hpp"
#include "terrain.hpp"
#include "screenshot.hpp"
#include "winshell.hpp"
#include <cstring>

void flightDebugSetMap(bool on, double dist);
void flightDebugPick(int mode);
bool flightDebugPickSeen();
void cadDebugSet(int sel, int tab);
void flightDebugInput(const ControlInput* in);
void cadDebugPalette(int cat);
void flightDebugLog(bool on);
void flightDebugNightVision(bool on);
void flightDebugView(double yaw, double pitch, double dist, Vec3d target);
void flightDebugBuildDialog(App& app, bool on);
void flightDebugBases(bool on);
bool factoryDebugEye(int i, Rectangle& r);
void flightDebugHorizon(const World& W, const Vessel& v, Vec3d& up, Vec3d& north, Vec3d& east);
extern Vec3d g_rotDbg[6]; // (sim.cpp: the active vessel's last attitude step)
int flightDebugDeleteDebris(World& W);
void flightDebugRename(World& W, Vessel& v, const std::string& name);
int flightDebugRecoverAsk();
void flightDebugRecover();
void flightDebugMapFocus(int body, double dist, double yaw, double pitch);
Vec3d debugAeroTorque(const World& W, const Vessel& v);
void flightDebugPause(bool on);
void flightDebugVessels(bool on);
void flightDebugMenu(int id, int item);
bool flightDebugMenuOpen();
void flightDebugAchievements(bool on);
void flightDebugEngines(bool on, int hoverBell);
void flightDebugNav(bool open);
void flightDebugGetView(double& yaw, double& pitch, double& dist, Vec3d& target);
double flightDebugFovy();
void factoryDebugUndo(App& app, bool redo);
void cadDebugSelect(const std::vector<int>& s);
void cadDebugDuplicate(App& app, bool mirror);
bool cadDebugImport(App& app, const Craft& c);
std::vector<int> cadDebugSelection();
void cadDebugOpenDialog(App& app, int mode);
void cadDebugHome(App& app);
bool cadDebugFairing(App& app);
bool cadDebugIntoBay(App& app, const Craft& c);
void factoryDebugSaveAs(const std::string& name);
bool factoryDebugSaveAsOpen();
void flightDebugNightMode(int m);
void factoryDebugSave(App& app);
bool factoryDebugOverwriteOpen();
std::vector<std::string> factoryDebugSavedNames(App& app);
void factoryDebugOpenList(App& app, bool on);
void factoryDebugOverwriteChoice(int c);

namespace {
struct AT {
    bool on = false;
    std::string dir;
    int phase = 0;
    int frames = 0;
    FILE* log = nullptr;
    double lastLog = -1e9;
    double e0 = 0;
    bool shotAscent = false;
    double peakFlux = 0;
    bool shotEntry = false;
    bool rewound = false;
    double vYaw = 0, vPitch = 0, vDist = 0, tSave = 0;
    Vec3d vTarget;
    std::string undoT0, undoT1;
    bool mouseUndoOk = false, mouseRedoOk = false;
    // --tutflight: the advanced tutorials flown from lift-off to the end by a pilot that follows the hints
    int tf = -1, tfLast = -1, tfStep = -2, tfSub = 0, tfDir = 1, tfRound = 0;
    double tfStepT = 0, tfRef = 0, tfMass0 = 0, tfTurn = 0;
    int tfRef2 = 0;
    bool tfFail = false, tfBurnDone = false;
    Vec3d trimDir;
    double warpAp = 1500e3, warpPe = 50e3;
    std::string warpBody; // --warpbody: the body (default Earth)
    bool tfKeys = false; // --tfkeys: the manual-ascent pilot flies with the keys (rate command), not by setting the attitude
    int warpIdx = -1;    // --warptest: the level asked for (-1: the highest)
    double kickDeg = 10; // tutorial 2 pilot: tilt at 1 km (the card says PITCH 80 = 10 degrees)
    // --rewindtest
    std::vector<std::pair<double, double>> rwFuel; // (time, propellant on board)
    double rwCrash = -1, rwLaunch = 0;
    Vec3d rcsV0, rcsR0;
    double rcsM0 = 0, rcsMono0 = 0, rcsT0 = 0;
    // --missiontest
    double t0 = 0, helioFrame = 0;
    float sc0 = 1;
} T;

// key bindings: defaults asked for (F1 HUD, F2 screenshot, mouse back / forward undo / redo), file
// round trip, rejection of malformed files; everything is written to the test directory only
void keyBindingChecks() {
    using keys::A;
    keys::resetDefaults();
    auto is = [](A a, int s, int code, int mods) { return keys::slot(a, s).code == code && keys::slot(a, s).mods == mods; };
    bool defs = is(A::ToggleHUD, 0, KEY_F1, 0) && is(A::Screenshot, 0, KEY_F2, 0) && is(A::DebugScreen, 0, KEY_F3, 0) &&
                is(A::Undo, 0, KEY_Z, keys::Ctrl) && is(A::Undo, 1, keys::MOUSE_BASE + MOUSE_BUTTON_SIDE, 0) &&
                is(A::Redo, 0, KEY_Z, keys::Ctrl | keys::Shift) && is(A::Redo, 1, keys::MOUSE_BASE + MOUSE_BUTTON_EXTRA, 0);
    // conflicts in overlapping contexts only: flight M (map) vs editor M (move tool) is fine
    bool conflictsOk = true;
    for (int a = 0; a < keys::N; a++)
        for (int s = 0; s < keys::SLOTS; s++)
            if (keys::conflict((A)a, keys::slot((A)a, s)) != A::COUNT) {
                conflictsOk = false;
                std::fprintf(T.log, "    default binding conflict: %s slot %d\n", keys::info((A)a).id, s);
            }
    std::string path = T.dir + "\\keys_test.txt";
    keys::slot(A::Stage, 1) = {KEY_ENTER, keys::Alt};
    keys::slot(A::ToggleHUD, 0) = {keys::MOUSE_BASE + MOUSE_BUTTON_MIDDLE, keys::Shift};
    keys::slot(A::Undo, 1) = {};
    bool saved = keys::save(path);
    keys::resetDefaults();
    keys::load(path);
    bool round = is(A::Stage, 1, KEY_ENTER, keys::Alt) && is(A::ToggleHUD, 0, keys::MOUSE_BASE + MOUSE_BUTTON_MIDDLE, keys::Shift) &&
                 is(A::Undo, 1, 0, 0) && is(A::Stage, 0, KEY_SPACE, 0);
    // malformed: out-of-range codes / modifiers, missing fields, unknown ids, overlong lines
    {
        std::ofstream f(path, std::ios::binary | std::ios::trunc);
        f << "stage 99999 0 0 0\nsas 84 9 0 0\nlegs 71\nnope 65 0 0 0\nmap -5 0 0 0\n" << std::string(5000, 'x') << "\nrcs 70 1 0 0\n";
    }
    keys::load(path);
    bool robust = is(A::Stage, 0, KEY_SPACE, 0) && is(A::SAS, 0, KEY_T, 0) && is(A::Legs, 0, KEY_G, 0) && is(A::MapView, 0, KEY_M, 0) &&
                  is(A::RCS, 0, KEY_F, keys::Ctrl);
    std::remove(path.c_str());
    keys::resetDefaults();
    std::fprintf(T.log, "key bindings: defaults %s, no default conflicts %s, save %s, round trip %s, malformed file %s (%s / %s / %s)\n",
                 defs ? "ok" : "FAIL", conflictsOk ? "ok" : "FAIL", saved ? "ok" : "FAIL", round ? "ok" : "FAIL", robust ? "ok" : "FAIL",
                 keys::name(keys::slot(A::Redo, 0)).c_str(), keys::name(keys::slot(A::Undo, 1)).c_str(), keys::name(keys::slot(A::ThrottleUp, 0)).c_str());
}

void shot(const char* name) {
    rlDrawRenderBatchActive(); // flush pending 2D UI before reading back
    Image img = LoadImageFromScreen();
    int rw = GetRenderWidth(), rh = GetRenderHeight();
    if (img.width > rw || img.height > rh) ImageCrop(&img, {0, (float)(img.height - rh), (float)rw, (float)rh});
    ExportImage(img, (T.dir + "\\" + name).c_str());
    UnloadImage(img);
    if (T.log) std::fprintf(T.log, "# screenshot %s\n", name);
}

double monoAmount(const Vessel& v) {
    double a = 0;
    for (size_t i = 0; i < v.comps.size(); i++)
        if (v.comps[i].func == Func::Tank && v.comps[i].prop == Prop::Monoprop) a += v.prop[i];
    return a;
}

void logState(App& app, const char* tag) {
    World& W = app.world;
    Vessel* v = W.activeVessel();
    if (!v || !T.log) return;
    const Body& b = W.U.bodies[v->ref];
    Conic c = stateToConic(v->r, v->v, b.GM);
    std::fprintf(T.log, "[fps %d gpu %.0f%% detail %d] ", GetFPS(), gpumon::utilization(), gfx::planetDetail());
    std::fprintf(T.log, "%s t=%.1f ref=%s alt=%.0f v=%.1f ap=%.0f pe=%.0f e=%.4f mass=%.0f thr=%.2f q=%.0f heat=%.3f landed=%d destroyed=%d %s\n", tag,
                 W.t - v->launchTime, b.name.c_str(), len(v->r) - b.radius, len(v->v), c.ap - b.radius, c.pe - b.radius, c.e,
                 W.vesselMass(*v), v->throttle, v->tDynP, v->tHeatMax, v->landed, v->destroyed, v->fate.c_str());
    std::fflush(T.log);
}

void pointAlong(Vessel& v, const Vec3d& dir) {
    v.q = Quatd::lookRot(dir, v.q.rotate({0, 1, 0}));
    v.w = {};
}

// ------------------------------------------------------------------ tutorial flights (--tutflight)
// remaining delta-v of the active stage (vacuum)
double stageDv(const World& W, const Vessel& v) {
    double amt[PROP_N], cap[PROP_N], sa, sc, F, mdot;
    stageResources(v, amt, cap, sa, sc, F, mdot);
    double prop = sa;
    for (int p = 1; p < PROP_N; p++)
        if (p != (int)Prop::Monoprop) prop += amt[p];
    double m = W.vesselMass(v);
    if (F <= 0 || mdot <= 0 || prop <= 0 || prop >= m) return 0;
    return F / mdot * std::log(m / (m - prop));
}
double tiltNow(const World& W, const Vessel& v) {
    const Body& b = W.U.bodies[v.ref];
    Conic c = stateToConic(v.r, v.v, b.GM);
    double inc = std::acos(clampd(dot(c.H, b.poleFrame.col(2) * (b.rotRate < 0 ? -1.0 : 1.0)), -1, 1)) / DEG;
    return std::min(inc, 180 - inc);
}
void tfLog(App& app, const char* what) {
    World& W = app.world;
    Vessel* v = W.activeVessel();
    if (!v) return;
    const Body& b = W.U.bodies[v->ref];
    Conic c = stateToConic(v->r, v->v, b.GM);
    std::fprintf(T.log, "tutflight %d %s: step %d MET %s ref %s alt %.0f km ap %.0f km pe %.0f km tilt %.2f deg mass %.0f kg stage %d dv(stage) %.0f m/s mono %.0f kg charge %.0f%%\n",
                 T.tf + 1, what, tut::stepIndex(), formatTime(W.t - v->launchTime).c_str(), b.name.c_str(), (len(v->r) - b.radius) / 1000,
                 (c.ap - b.radius) / 1000, (c.pe - b.radius) / 1000, v->landed ? 0.0 : tiltNow(W, *v), W.vesselMass(*v), v->stage, stageDv(W, *v),
                 v->monoAmt, v->chargeCap > 0 ? 100 * v->charge / v->chargeCap : 0.0);
    std::fflush(T.log);
}
// drop an empty stage and light the next one, as the hint of the tutorial tells the player
void tfStaging(App& app, Vessel*& v) {
    World& W = app.world;
    if (v->tFlameout && v->throttle > 0 && vesselStages(*v).size() > 1) {
        W.stage(*v);
        v = W.activeVessel();
        if (!v->ignited) W.stage(*v); // side boosters gone: the core keeps burning, nothing to ignite
        v = W.activeVessel();
        tfLog(app, "staged");
    }
}
bool tfAligned(const World& W, const Vessel& v) {
    Vec3d d = W.targetDirection(v, v.sas);
    return len(d) > 0.5 && dot(v.q.rotate({0, 0, 1}), d) > 0.9995;
}
// SAS turns the vessel with its own torque; true once the nose points along the selected mode. A turn
// that takes more than 10 minutes is reported (and finished by hand so that the flight goes on).
bool tfTurn(App& app, Vessel& v, SAS mode) {
    World& W = app.world;
    if (v.sas != mode) {
        v.sas = mode;
        T.tfTurn = W.t;
    }
    if (tfAligned(W, v)) return true;
    if (W.warpIndex > 0) W.setWarp(0); // turning: back to x1 (time stands still for a turn under warp otherwise)
    if (W.t - T.tfTurn > 600) {
        std::fprintf(T.log, "tutflight %d: SAS %s not reached after 10 min -> FAIL (slow attitude control)\n", T.tf + 1, sasName(mode));
        T.tfFail = true;
        pointAlong(v, W.targetDirection(v, mode));
        return true;
    }
    if (W.warpIndex == 0)
        for (int k = 0; k < 6; k++) W.update(0.05);
    return false;
}
// delta-v left in all stages (vacuum, stages burnt one after the other)
double totalDv(const World& W, const Vessel& v) {
    double m = W.vesselMass(v), dv = 0;
    for (int g : vesselStages(v)) {
        double prop = 0, F = 0, mdot = 0, gm = 0;
        for (size_t i = 0; i < v.comps.size(); i++) {
            const Component& c = v.comps[i];
            if (c.stage != g) continue;
            gm += v.phys[i].dryMass + v.prop[i];
            if (c.func == Func::Tank && c.prop != Prop::Monoprop) prop += v.prop[i];
            if (c.func == Func::Engine && v.phys[i].thrustVac > 0) {
                F += v.phys[i].thrustVac;
                mdot += v.phys[i].thrustVac / (v.phys[i].ispVac * G0);
                prop += v.prop[i]; // solid motors
            }
        }
        if (F > 0 && mdot > 0 && prop > 0 && prop < m) dv += F / mdot * std::log(m / (m - prop));
        m -= gm;
        if (m <= 1) break;
    }
    return dv;
}

// ------------------------------------------------------------------ every template to orbit (--orbitall)
// The player's part only: SAS "Ascent guidance", full throttle, lift off, drop empty stages, time warp
// through the coast above the air. The guidance steers, works the throttle and cuts the engine in orbit.
bool orbitAllStep(App& app) {
    World& W = app.world;
    auto tpl = builtinTemplates();
    if (T.tf >= (int)tpl.size()) return false;
    if (T.frames == 2) {
        W.newGame(Settings{});
        app.gameStarted = true;
        app.design = tpl[T.tf];
        app.lastLaunched = craftFromDesign(app.design);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        T.tfSub = 0;
        T.tfRound = 0;
    }
    if (T.frames < 6) return true;
    Vessel* v = W.activeVessel();
    auto next = [&](const char* verdict) {
        if (v) {
            const Body& b = W.U.bodies[v->ref];
            Conic c = stateToConic(v->r, v->v, b.GM);
            double thr = 0, md = 0, amt[PROP_N], cap[PROP_N], sa, sc;
            stageResources(*v, amt, cap, sa, sc, thr, md);
            std::fprintf(T.log, "orbit check %-24s %s  MET %s  ap %.0f km  pe %.0f km  mass %.0f kg  stage %d (thrust/weight %.2f)  dv left %.0f m/s  %s\n",
                         tpl[T.tf].name.c_str(), verdict, formatTime(W.t - v->launchTime).c_str(), (c.ap - b.radius) / 1000, (c.pe - b.radius) / 1000,
                         W.vesselMass(*v), v->stage, thr / (W.vesselMass(*v) * 9.81), totalDv(W, *v), v->destroyed ? v->fate.c_str() : "");
            std::fflush(T.log);
        }
        W.warpIndex = 0;
        T.tf++;
        T.frames = 0;
        return T.tf < (int)tpl.size();
    };
    if (!v) return next("FAIL (no vessel)");
    const Body& b = W.U.bodies[v->ref];
    if (v->destroyed) return next("FAIL");
    if (W.t - v->launchTime > 6000) return next("FAIL (no orbit after 100 minutes)");
    if (T.tfSub == 0) {
        v->sas = SAS::Ascent;
        v->throttle = 1;
        if (!v->ignited) W.stage(*v);
        T.tfSub = 1;
        std::fprintf(T.log, "orbit check %-24s lift-off: mass %.0f kg, dv %.0f m/s\n", tpl[T.tf].name.c_str(), W.vesselMass(*v), totalDv(W, *v));
        return true;
    }
    if (v->ascentPhase == 1 && len(v->r) - b.radius > b.atmTop) W.warpIndex = std::min(5, W.maxWarpIndex());
    else if (W.warpIndex > 0) W.warpIndex = 0;
    for (int k = 0; k < 8 && !v->destroyed && v->sas == SAS::Ascent; k++) {
        W.update(0.05);
        tfStaging(app, v);
        if (v->ascentPhase != T.tfRound) {
            T.tfRound = v->ascentPhase;
            Conic k2 = stateToConic(v->r, v->v, b.GM);
            std::fprintf(T.log, "orbit check %-24s phase %d: MET %s alt %.0f km speed %.0f m/s ap %.0f km pe %.0f km stage %d\n", tpl[T.tf].name.c_str(),
                         v->ascentPhase, formatTime(W.t - v->launchTime).c_str(), (len(v->r) - b.radius) / 1000, len(v->v), (k2.ap - b.radius) / 1000,
                         (k2.pe - b.radius) / 1000, v->stage);
        }
    }
    if (v->destroyed) return next("FAIL");
    if (v->sas != SAS::Ascent) {
        Conic c = stateToConic(v->r, v->v, b.GM);
        return next(c.valid && c.e < 1 && c.pe - b.radius > 150e3 ? "ok  " : "FAIL (cut-off without orbit)");
    }
    return true;
}
// a burn along the SAS mode the step asks for; `stop` says when to cut
template <class F>
void tfBurn(App& app, Vessel*& v, SAS mode, double throttle, int steps, double h, F stop) {
    World& W = app.world;
    if (T.tfBurnDone) return; // cut once: wait for the step to be accepted
    if (v->throttle < 0.01 && !tfTurn(app, *v, mode)) return;
    if (W.warpIndex > 0) W.setWarp(0);
    if (!v->ignited) W.stage(*v);
    v->throttle = throttle;
    for (int k = 0; k < steps && !v->destroyed; k++) {
        W.update(h);
        tfStaging(app, v);
        if (stop()) {
            v->throttle = 0;
            T.tfBurnDone = true;
            break;
        }
    }
}
void tfWarp(World& W) {
    if (W.warpIndex < W.maxWarpIndex()) W.warpIndex = W.maxWarpIndex(); // the tutorial's warp hold slows it down in time
    // once the moment the step waits for has come (the hold expires), stop: the tutorial must see it
    // before the warp is raised again (a pass through a sphere of influence takes less than a frame at x1e8)
    bool held = W.warpHold > 0;
    for (int k = 0; k < 8 && !(held && W.warpHold <= 0); k++) W.update(0.05);
    if (held && W.warpHold <= 0) W.setWarp(0);
}
// plane change at the crossing: full throttle, lower near the end, cut when the tilt stops falling
void tfPlaneBurn(App& app, Vessel*& v) {
    World& W = app.world;
    double tilt = tiltNow(W, *v);
    if (T.tfSub == 0) {
        T.tfRef = tilt;
        T.tfSub = 1;
    }
    bool fine = tilt < 4;
    if (fine && T.tfSub == 1) { // coarse part done: the fine part is a new burn
        T.tfSub = 2;
        T.tfBurnDone = false;
    }
    tfBurn(app, v, v->sas, fine ? 0.4 : 1.0, fine ? 10 : 6, fine ? 0.02 : 0.05, [&] {
        double t = tiltNow(W, *v);
        bool done = t < 0.4 || t > T.tfRef + 0.02;
        T.tfRef = std::min(T.tfRef, t);
        return done || (!fine && t < 4);
    });
}
// one frame of the pilot; false when the flight is over
bool tutFlightStep(App& app) {
    World& W = app.world;
    if (T.frames == 2) {
        bool ok = tut::start(app, T.tf);
        T.tfStep = -2;
        T.tfFail = !ok;
        std::fprintf(T.log, "tutflight %d: start %s\n", T.tf + 1, ok ? "ok" : "FAIL");
    }
    if (T.frames < 6) return true;
    Vessel* v = W.activeVessel();
    auto finish = [&](const char* verdict) {
        tfLog(app, verdict);
        char name[64];
        std::snprintf(name, sizeof name, "22_tutflight_%d_end.png", T.tf + 1);
        shot(name);
        flightDebugInput(nullptr);
        tut::stop();
        if (T.tf < T.tfLast) {
            T.tf++;
            T.frames = 0;
            return true;
        }
        return false;
    };
    if (!v || v->destroyed) {
        std::fprintf(T.log, "tutflight %d: vessel lost (%s) -> FAIL\n", T.tf + 1, v ? v->fate.c_str() : "none");
        return finish("FAIL");
    }
    int step = tut::stepIndex();
    if (step != T.tfStep) {
        tfLog(app, "step");
        T.tfStep = step;
        T.tfStepT = W.t;
        T.tfSub = 0;
        T.tfRound = 0;
        T.tfBurnDone = false;
        if (step == 3 || step == 6) {
            char name[64];
            std::snprintf(name, sizeof name, "22_tutflight_%d_step%d.png", T.tf + 1, step + 1);
            shot(name);
        }
    }
    if (tut::finished()) return finish(T.tfFail ? "completed with problems -> FAIL" : "completed -> ok");
    if (W.t - T.tfStepT > (T.tf == 1 ? 7200 : 12 * DAY) || T.frames > 400000) {
        std::fprintf(T.log, "tutflight %d: step %d does not finish -> FAIL\n", T.tf + 1, step);
        return finish("FAIL");
    }
    if (tut::stepDone()) { // the check mark shows: hands off (and no time warp) until the next step
        if (W.warpIndex > 0) W.setWarp(0);
        flightDebugInput(nullptr);
        return true;
    }
    const tut::Guide& g = tut::guide();
    const Body& b = W.U.bodies[v->ref];
    Conic c = stateToConic(v->r, v->v, b.GM);
    bool moonFlight = T.tf == 6;
    // tutorial 8 "Manual ascent": the pitch programme of the hint held by hand, prograde, cut at 200 km,
    // the circularisation burn when the hint says so
    if (T.tf == 2) {
        const Body& eb = W.U.bodies[v->ref];
        Vec3d up = norm(v->r), east = norm(cross(eb.poleFrame.col(2), up));
        double alt = len(v->r) - eb.radius;
        switch (step) {
        case 0: tut::debugNext(); return true;
        case 1:
            v->throttle = 1;
            if (!v->ignited) W.stage(*v);
            return true;
        case 2: // hold the target pitch of the hint (heading east)
            for (int k = 0; k < 6 && !v->destroyed; k++) {
                up = norm(v->r);
                east = norm(cross(eb.poleFrame.col(2), up));
                alt = len(v->r) - eb.radius;
                double tp = alt < 800 ? 90.0 : 90.0 - std::min(80.0, 90.0 * std::sqrt((alt - 800) / (eb.atmTop * 0.34)));
                Vec3d dirT = up * std::sin(tp * DEG) + east * std::cos(tp * DEG);
                if (T.tfKeys) {
                    // as a player: pitch / yaw keys toward the hint's target (the input asks for a turn rate)
                    Vec3d nose = v->q.rotate({0, 0, 1});
                    Vec3d e = v->q.conj().rotate(cross(nose, dirT));
                    ControlInput in;
                    in.pitch = std::clamp(e.x * 12.0, -1.0, 1.0);
                    in.yaw = std::clamp(e.y * 12.0, -1.0, 1.0);
                    in.roll = std::clamp(-v->w.z * 2.0, -1.0, 1.0);
                    v->sas = SAS::Off;
                    W.input = in;
                    flightDebugInput(&in);
                } else pointAlong(*v, dirT);
                W.update(0.05);
                tfStaging(app, v);
            }
            return true;
        case 3: // prograde, cut at an apoapsis of 200 km
            if (T.tfKeys) flightDebugInput(nullptr);
            v->sas = SAS::Prograde;
            for (int k = 0; k < 6 && !v->destroyed && v->throttle > 0; k++) {
                W.update(0.05);
                tfStaging(app, v);
                Conic k2 = stateToConic(v->r, v->v, eb.GM);
                if (k2.valid && k2.e < 1 && k2.ap - eb.radius >= 200e3) v->throttle = 0;
            }
            if (v->throttle <= 0) for (int k = 0; k < 4; k++) W.update(0.05);
            return true;
        case 4: // coast with warp; at the hint's ignition: full throttle prograde until the periapsis is in space
            v->sas = SAS::Prograde;
            if (v->throttle <= 0 && g.tIgnite > 2) {
                if (alt > eb.atmTop) tfWarp(W);
                else for (int k = 0; k < 6; k++) W.update(0.05);
                return true;
            }
            if (W.warpIndex > 0) W.setWarp(0);
            if (!T.tfBurnDone) {
                v->throttle = 1;
                if (!v->ignited) W.stage(*v);
                for (int k = 0; k < 6 && !v->destroyed; k++) {
                    W.update(0.05);
                    tfStaging(app, v);
                    Conic k2 = stateToConic(v->r, v->v, eb.GM);
                    if (k2.valid && k2.pe - eb.radius > 172e3) { v->throttle = 0; T.tfBurnDone = true; break; }
                }
            }
            return true;
        }
        return true;
    }
    // tutorial 2 "Reaching orbit": 0 briefing, 1 lift-off with the guidance, 2 explanation, 3 ascent, 4 map
    if (T.tf == 1) {
        const Body& eb = W.U.bodies[v->ref];
        switch (step) {
        case 0:
        case 2: tut::debugNext(); return true;
        case 1:
            v->sas = SAS::Ascent;
            v->throttle = 1;
            if (!v->ignited) W.stage(*v);
            return true;
        case 3:
            if (v->ascentPhase == 1 && len(v->r) - eb.radius > eb.atmTop) W.warpIndex = std::min(5, W.maxWarpIndex());
            else if (W.warpIndex > 0) W.warpIndex = 0;
            for (int k = 0; k < 8 && !v->destroyed && v->sas == SAS::Ascent; k++) {
                W.update(0.05);
                tfStaging(app, v);
            }
            return true;
        case 4: flightDebugSetMap(true, 4 * eb.radius); return true;
        }
        return true;
    }
    // steps shared by both flights: 0 briefing, 1 lift-off with the ascent guidance, 2 ascent to the parking orbit
    switch (step) {
    case 0: tut::debugNext(); return true;
    case 1:
        v->sas = SAS::Ascent;
        if (moonFlight && g.tIgnite > 0 && g.tIgnite < 1e17) { tfWarp(W); return true; }
        if (moonFlight && g.tIgnite > 1e17) return true;
        v->throttle = 1;
        if (!v->ignited) W.stage(*v);
        return true;
    case 2:
        if (v->ascentPhase == 1 && len(v->r) - b.radius > b.atmTop) W.warpIndex = std::min(5, W.maxWarpIndex());
        else if (W.warpIndex > 0) W.warpIndex = 0;
        for (int k = 0; k < 8 && !v->destroyed && v->sas == SAS::Ascent; k++) {
            W.update(0.05);
            tfStaging(app, v);
        }
        return true;
    }
    if (!moonFlight) {
        switch (step) {
        case 3: // wait for the equator crossing, nose prograde
        case 5: // wait for the crossing near the apoapsis, nose along the mode of the hint
            if (g.sas == 0) return true;
            if (step == 5 && !v->solarDeployed) W.setSolarDeployed(*v, true);
            if (tfTurn(app, *v, (SAS)g.sas) && g.tIgnite > 0) tfWarp(W);
            return true;
        case 4: tfBurn(app, v, SAS::Prograde, 1, 6, 0.05, [&] { return stateToConic(v->r, v->v, b.GM).ap - b.radius > 35.9e6; }); return true;
        case 6: tfPlaneBurn(app, v); return true;
        }
        return true;
    }
    switch (step) {
    case 3: // wait for the injection point
        if (g.sas == 0) return true;
        if (T.tfSub == 0) {
            // how well did the launch window work? angle between the Moon on arrival and the orbit plane
            std::vector<BodyState> S2;
            W.U.computeStatesFor(W.t + 4.75 * DAY, W.U.relevant[v->ref], S2); // about the arrival
            double off = std::asin(clampd(dot(c.H, norm(W.U.relPos(S2, W.U.find("Moon"), v->ref))), -1, 1)) / DEG;
            std::fprintf(T.log, "tutflight 7: Moon (in 5 days) lies %.2f deg out of the parking orbit plane\n", off);
            T.tfSub = 1;
        }
        if (tfTurn(app, *v, SAS::Prograde) && g.tIgnite > 0) tfWarp(W);
        return true;
    case 4: // injection: cut as soon as the hint reports the encounter
    {
        // the apoapsis target in the hint: full throttle, the lowest throttle for the last 1.5 %, cut on it
        Conic k = stateToConic(v->r, v->v, b.GM);
        double ratio = g.target > 0 && k.valid && k.e < 1 ? k.ap / g.target : 0;
        if (g.target > 0 && (ratio >= 0.996 || !k.valid || k.e >= 1)) { v->throttle = 0; T.tfBurnDone = true; return true; }
        if (g.tIgnite > 1e17) return true;
        bool fine = ratio > 0.985;
        tfBurn(app, v, SAS::Prograde, fine ? 0.1 : 1.0, fine ? 1 : 4, 0.05, [&] {
            Conic k2 = stateToConic(v->r, v->v, b.GM);
            return g.target > 0 && k2.valid && k2.e < 1 && k2.ap >= g.target * 0.996;
        });
    }
        return true;
    case 5: W.setSolarDeployed(*v, true); return true;
    case 6: // a day of coasting
        if (T.frames % 10 == 0) {
            std::fprintf(T.log, "  coastday: left %.0f s  warp %d  hold %.0f s  ref %s\n", g.tIgnite, W.warpIndex, W.warpHold > 0 ? W.warpHold - W.t : -1.0, b.name.c_str());
            std::fflush(T.log);
        }
        // the step sets its warp hold on its first check: until then a frame at full warp would skip days
        if (W.warpHold <= 0 || g.tIgnite > 1e17) return true;
        tfWarp(W);
        return true;
    case 7: { // mid-course correction: hold the RCS key the hint names
        v->rcs = true;
        if (!tfTurn(app, *v, SAS::Prograde)) return true;
        ControlInput in;
        static const double AX[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
        if (g.trimAxis >= 0) {
            in.tx = AX[g.trimAxis][0];
            in.ty = AX[g.trimAxis][1];
            in.tz = AX[g.trimAxis][2];
        }
        flightDebugInput(&in);
        W.input = in;
        if (g.trimAxis >= 0) for (int k = 0; k < 2; k++) W.update(0.05);
        if (T.frames % 25 == 0) {
            std::fprintf(T.log, "  trim: axis %d enc %d pe %.0f km mono %.0f kg\n", g.trimAxis, (int)g.enc, g.encPe / 1000, v->monoAmt);
            std::fflush(T.log);
        }
        return true;
    }
    case 8:
        if (T.frames % 20 == 0) {
            int mn = W.U.find("Moon");
            std::fprintf(T.log, "  coast: ref %s enc %d pe %.0f km  warp %d hold %.0f s  moon dist %.0f km  t %.0f s\n", b.name.c_str(), (int)g.enc,
                         g.encPe / 1000, W.warpIndex, W.warpHold > 0 ? W.warpHold - W.t : -1.0,
                         len(v->r - W.U.relPos(W.states, mn, v->ref)) / 1000, W.t - T.tfStepT);
            std::fflush(T.log);
        }
        tfWarp(W);
        return true;
    case 9: // falling toward the Moon: raise a low periapsis, then nose retrograde and wait
        flightDebugInput(nullptr);
        if (b.name != "Moon") return true;
        if (c.pe - b.radius < 60e3) {
            v->rcs = true;
            if (!tfTurn(app, *v, SAS::RadialOut)) return true;
            ControlInput in;
            in.tz = 1;
            W.input = in;
            for (int k = 0; k < 6; k++) W.update(0.05);
            W.input = {};
            return true;
        }
        if (g.sas == 0) return true;
        if (tfTurn(app, *v, SAS::Retrograde) && g.tIgnite > 0) tfWarp(W);
        return true;
    case 10:
        tfBurn(app, v, SAS::Retrograde, 1, 6, 0.05, [&] {
            Conic k = stateToConic(v->r, v->v, b.GM);
            return k.e < 1 && k.ap - b.radius < 9500e3;
        });
        return true;
    case 11:
        if (g.sas == 0) return true;
        if (tfTurn(app, *v, (SAS)g.sas) && g.tIgnite > 0) tfWarp(W);
        return true;
    case 12: tfPlaneBurn(app, v); return true;
    }
    return true;
}

// ------------------------------------------------------------------ rewind after a loss (--rewindtest)
double propOnBoard(const Vessel& v) {
    double s = 0;
    for (double p : v.prop) s += p;
    return s;
}
double fuelAt(double t) {
    const auto& f = T.rwFuel;
    if (f.empty()) return -1;
    if (t <= f.front().first) return f.front().second;
    for (size_t i = 1; i < f.size(); i++)
        if (f[i].first >= t) {
            double u = (t - f[i - 1].first) / std::max(1e-9, f[i].first - f[i - 1].first);
            return f[i - 1].second + (f[i].second - f[i - 1].second) * u;
        }
    return f.back().second;
}
bool rewindTestStep(App& app) {
    World& W = app.world;
    if (T.frames == 2) {
        W.newGame(Settings{});
        app.gameStarted = true;
        app.design = builtinTemplates()[0];
        app.lastLaunched = craftFromDesign(app.design);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        Vessel* v = W.activeVessel();
        v->throttle = 1;
        v->sas = SAS::Stability;
        W.stage(*v);
        T.rwLaunch = W.t;
        T.rwFuel.clear();
        T.rwCrash = -1;
        return true;
    }
    if (T.frames < 3) return true;
    Vessel* v = W.activeVessel();
    if (!v) return false;
    if (T.rwCrash < 0) {
        for (int k = 0; k < 20 && !v->destroyed; k++) {
            if (W.t - T.rwLaunch > 40) pointAlong(*v, -norm(v->r)); // dive back into the ground under power
            W.update(0.05);
            v = W.activeVessel();
            T.rwFuel.push_back({W.t, propOnBoard(*v)});
        }
        if (v->destroyed) {
            T.rwCrash = W.t;
            std::fprintf(T.log, "rewind test: lost at MET %.0f s (%s), history %zu states\n", W.t - T.rwLaunch, v->fate.c_str(), W.history.size());
        }
        if (W.t - T.rwLaunch > 900) {
            std::fprintf(T.log, "rewind test: the vessel did not crash -> FAIL\n");
            return false;
        }
        return true;
    }
    if (W.t - T.rwCrash < 45) { // the player looks at the dialog for a while; the world goes on
        for (int k = 0; k < 20; k++) W.update(0.05);
        return true;
    }

    // one press of "Back 30 s"
    bool ok1 = W.rewind(30);
    v = W.activeVessel();
    double fuelWant = fuelAt(W.t), fuelNow = propOnBoard(*v);
    bool alive = !v->destroyed;
    double back = T.rwCrash - W.t;
    bool fuelOk = std::fabs(fuelNow - fuelWant) <= 0.03 * std::max(fuelWant, 1.0) + 50;
    std::fprintf(T.log, "rewind test: Back 30 s (one press, 45 s after the loss): %s, vessel %s, %.1f s before the loss, propellant %.0f kg (flight record %.0f kg) -> %s\n",
                 ok1 ? "done" : "refused", alive ? "intact" : "still lost", back, fuelNow, fuelWant,
                 ok1 && alive && back > 26 && back < 34 && fuelOk ? "ok" : "FAIL");
    // then "Back 3 min" while flying: the flight is shorter, so it goes to the lift-off
    bool ok2 = W.rewind(180);
    v = W.activeVessel();
    double full = T.rwFuel.empty() ? 0 : T.rwFuel.front().second;
    std::fprintf(T.log, "rewind test: Back 3 min: %s, MET %.1f s, landed %d, propellant %.0f kg (full %.0f kg) -> %s\n", ok2 ? "done" : "refused",
                 W.t - T.rwLaunch, (int)v->landed, propOnBoard(*v), full,
                 ok2 && W.t - T.rwLaunch < 3 && propOnBoard(*v) >= full * 0.98 ? "ok" : "FAIL");
    return false;
}

// ------------------------------------------------------------------ real terrain (--terraintest)
Vec3d siteDir(double latDeg, double lonDeg) {
    double la = latDeg * DEG, lo = lonDeg * DEG;
    return {std::cos(la) * std::cos(lo), std::cos(la) * std::sin(lo), std::sin(la)};
}
// move the clock so that the Sun stands `elev` degrees above the site (morning side)
void sunAt(World& W, int body, const Vec3d& f, double elev) {
    int star = W.U.hostStar(body);
    W.refreshStates();
    Vec3d s = norm(W.U.relPos(W.states, star, body));
    const Body& b = W.U.bodies[body];
    double day = std::fabs(TAU / b.rotRate);
    double best = 1e9, bestT = W.t;
    for (int k = 0; k < 720; k++) {
        double t = W.t + day * k / 720.0;
        Vec3d up = W.U.bodyRot(body, t) * f;
        Vec3d upNext = W.U.bodyRot(body, t + day / 720.0) * f;
        double el = std::asin(clampd(dot(up, s), -1, 1)) / DEG;
        bool rising = dot(upNext, s) > dot(up, s);
        double err = std::fabs(el - elev) + (rising ? 0 : 1000);
        if (err < best) { best = err; bestT = t; }
    }
    W.t = bestT;
    W.refreshStates();
}
// put the active vessel `alt` metres above the ground at a site, at rest relative to the surface
void placeAt(World& W, Vessel& v, int body, const Vec3d& f, double alt) {
    const Body& b = W.U.bodies[body];
    double t0 = GetTime();
    terrain::focus(body, f, alt); // (a jump: the detail tiles there first, as the simulation does before a step)
    if (T.log) std::fprintf(T.log, "terrain: detail tiles at the new place decoded in %.0f ms (%zu MB of elevation tiles)\n", (GetTime() - t0) * 1000,
                            terrain::detailBytes() >> 20);
    double h = terrain::height(body, f);
    Mat3d R = W.U.bodyRot(body, W.t);
    v.ref = body;
    v.landed = false;
    v.r = R * f * (b.radius + h + alt);
    v.v = cross(W.U.angVel(body), v.r);
    Vec3d up = norm(v.r);
    v.q = Quatd::lookRot(up, anyPerp(up));
    v.w = {};
    v.throttle = 0;
    v.ignited = false;
    v.sas = SAS::Stability;
    v.prevAlt = -1;
    W.warpIndex = 0;
    W.history.clear();
    W.recordHistory(true);
}
bool terrainTestStep(App& app) {
    World& W = app.world;
    int moon = W.U.find("Moon"), mars = W.U.find("Mars"), earth = W.U.find("Earth");
    auto view = [&](double pitch, double dist) { flightDebugView(0.3, pitch, dist, Vec3d{}); };
    struct Shot { const char* name; int body; double lat, lon, alt, sun, pitch, dist; };
    static const Shot SHOTS[] = {
        {"23a_mars_olympus_orbit.png", 0, 18.6, -133.8, 1400e3, 18, 1.48, 40},
        {"23b_mars_valles_orbit.png", 0, -9.0, -70.0, 1800e3, 22, 1.48, 40},
        {"23c_mars_olympus_low.png", 0, 16.0, -128.0, 35e3, 12, 0.32, 60},
        {"23d_moon_nearside.png", 1, 10.0, -10.0, 2500e3, 30, 1.48, 40},
        {"23e_moon_low.png", 1, 30.0, -12.0, 20e3, 8, 0.25, 60},
        {"23f_earth_himalaya.png", 2, 30.0, 86.0, 900e3, 25, 1.45, 40},
        {"23i_mercury_generated.png", 3, 10.0, 40.0, 1500e3, 30, 1.45, 40},
        {"23j_callisto_low.png", 4, 5.0, 20.0, 8e3, 10, 0.25, 60},
        {"23o_earth_window_3500km.png", 2, 25.0, 60.0, 3500e3, 40, 1.45, 40}, // the border of the detail window (8k tiles -> global map)
    };
    const int nShots = sizeof SHOTS / sizeof SHOTS[0];
    if (T.frames == 2) {
        W.newGame(Settings{});
        app.gameStarted = true;
        // lander: the upper part of the Selene Lander (above its first decoupler), fairing dropped
        Design d = builtinTemplates()[2];
        int dec = CATALOG.find("decoupler");
        for (size_t i = 0; i < d.stack.size(); i++)
            if (d.stack[i].def == dec) { d.stack.resize(i); break; }
        app.design = d;
        app.lastLaunched = craftFromDesign(d);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        Vessel* v = W.activeVessel();
        W.jettisonFairing(*v, false);
        v = W.activeVessel();
        v->legsDeployed = true;
        {
            double legLow = 1e9, bodyLow = 1e9;
            for (size_t k = 0; k < v->contacts.size(); k++) (v->contactLeg[k] ? legLow : bodyLow) = std::min(v->contactLeg[k] ? legLow : bodyLow, v->contacts[k].z);
            std::fprintf(T.log, "terrain: lander feet at z %.2f m, lowest other point %.2f m -> %s\n", legLow, bodyLow, legLow < bodyLow ? "feet first" : "FAIL (the engine touches first)");
        }
        // the Moon keeps its near side (longitude 0) towards the Earth
        Vec3d e = W.U.bodyRot(moon, W.t).T() * W.U.relPos(W.states, earth, moon);
        double lon = std::atan2(e.y, e.x) / DEG, lat = std::asin(e.z / len(e)) / DEG;
        std::fprintf(T.log, "terrain: maps %zu MB; Earth seen from the Moon at lat %.1f lon %.1f -> %s\n", terrain::bytes() >> 20, lat, lon,
                     std::fabs(lon) < 8 && std::fabs(lat) < 8 ? "ok" : "FAIL");
        const Vec3d olympus = siteDir(18.65, -133.8), valles = siteDir(-12, -65), imbrium = siteDir(33, -16);
        std::fprintf(T.log, "terrain: Olympus Mons %.0f m, Valles Marineris (Coprates) %.0f m, Hellas %.0f m, Mare Imbrium %.0f m, Everest region %.0f m, pad %.0f m\n",
                     terrain::height(mars, olympus), terrain::height(mars, valles), terrain::height(mars, siteDir(-42, 70)), terrain::height(moon, imbrium),
                     terrain::height(earth, siteDir(27.99, 86.93)), terrain::height(earth, siteDir(LAUNCH_SITES[0].latDeg, LAUNCH_SITES[0].lonDeg)));
        T.tfSub = 0;
        return true;
    }
    if (T.frames < 4) return true;
    Vessel* v = W.activeVessel();
    if (!v) return false;
    // shots: place, wait for the bakes, shoot
    if (T.tfSub < nShots) {
        const Shot& s = SHOTS[T.tfSub];
        int body = s.body == 0 ? mars : s.body == 1 ? moon : s.body == 2 ? earth : W.U.find(s.body == 3 ? "Mercury" : "Callisto");
        int k = (T.frames - 4) % 14;
        if (k == 0) {
            Vec3d f = siteDir(s.lat, s.lon);
            sunAt(W, body, f, s.sun);
            placeAt(W, *v, body, f, s.alt);
            view(s.pitch, s.dist);
        }
        if (k == 13) {
            shot(s.name);
            T.tfSub++;
        }
        return true;
    }
    // landings: on the floor of Mare Imbrium (-2.3 km), on the summit of Olympus Mons (+21 km), in the Hellas
    // basin (-7 km, below the datum) and on the Tibetan plateau (+5 km); after each, the ground is looked at
    // from several camera positions (along the horizon, from below the vessel, from far away): it must
    // never be cut away or seen through
    struct Landing { const char* name; const char* tag; int body; double lat, lon, alt; };
    static const Landing LANDINGS[] = {{"Mare Imbrium", "mare", 1, 33, -16, 12},
                                       {"Olympus Mons summit", "olympus", 0, 18.4, -133.5, 6},
                                       {"Hellas basin", "hellas", 0, -42, 70, 8},
                                       {"Tibetan plateau", "tibet", 2, 33, 88, 8},
                                       {"Moon south pole", "spole", 1, -89.3, 40, 8},
                                       {"Elysium (date line)", "dateline", 0, 12, 179.6, 8}};
    const int nLand = sizeof LANDINGS / sizeof LANDINGS[0];
    int li = T.tfSub - nShots;
    if (li >= nLand) {
        mem::update(GetTime());
        std::fprintf(T.log, "terrain: process memory %zu MB (global maps %zu MB, elevation tiles %zu MB)\n", mem::status().process >> 20, terrain::bytes() >> 20,
                     terrain::detailBytes() >> 20);
        return false;
    }
    const Landing& l = LANDINGS[li];
    int body = l.body == 0 ? mars : l.body == 1 ? moon : earth;
    static int post = -1; // frames after the landing (views)
    if (T.tfRound != li + 1) {
        T.tfRound = li + 1;
        if (li > 0) { // a fresh lander for each landing
            W.launch(app.lastLaunched);
            v = W.activeVessel();
            W.jettisonFairing(*v, false);
            v = W.activeVessel();
        }
        Vec3d f = siteDir(l.lat, l.lon);
        sunAt(W, body, f, 25);
        T.tfRef = W.t; // (after moving the clock)
        placeAt(W, *v, body, f, l.alt);
        v->legsDeployed = true;
        view(0.25, 30);
        post = -1;
        return true;
    }
    if (post < 0) {
        for (int k = 0; k < 10 && !v->landed && !v->destroyed; k++) W.update(0.05);
        if (!(v->landed || v->destroyed || W.t - T.tfRef > 60)) return true;
        const Body& b = W.U.bodies[body];
        Vec3d fb = W.U.bodyRot(body, W.t).T() * v->r;
        double ground = terrain::height(body, fb);
        double alt = len(v->r) - b.radius;
        std::fprintf(T.log, "terrain landing on %s: %s, centre of mass %.0f m above the datum, ground %.0f m, %.1f m above the ground -> %s\n", l.name,
                     v->landed ? "landed" : v->destroyed ? v->fate.c_str() : "still falling", alt, ground, alt - ground,
                     v->landed && alt - ground > 0 && alt - ground < 15 ? "ok" : "FAIL");
        post = 0;
    }
    struct V { double pitch, dist; const char* tag; };
    static const V VIEWS[] = {{0.25, 30, "a"}, {0.03, 40, "b_horizon"}, {-0.4, 18, "c_below"}, {0.15, 900, "d_far"}};
    int vi = post / 8, k = post % 8;
    if (vi >= 4) {
        post = -1;
        T.tfSub++;
        return true;
    }
    if (k == 0) view(VIEWS[vi].pitch, VIEWS[vi].dist);
    if (k == 7) {
        char name[64];
        std::snprintf(name, sizeof name, "23%c_landed_%s_%s.png", 'k' + li, l.tag, VIEWS[vi].tag);
        shot(name);
    }
    post++;
    return true;
}

// ------------------------------------------------------------------ time warp over an orbit that dips into the air (--warptest)
// a probe on an orbit with its periapsis inside the atmosphere, started near the apoapsis: the player asks
// for the highest warp every frame (as with the key held). Logged: the level, the rate reached, the reason
// shown, the frame time; the run ends after the vessel entered the atmosphere (or after 1500 frames)
bool warpTestStep(App& app) {
    World& W = app.world;
    if (T.frames == 2) {
        W.newGame(Settings{});
        app.gameStarted = true;
        app.design = builtinTemplates()[2];
        app.lastLaunched = craftFromDesign(app.design);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        Vessel* v = W.activeVessel();
        if (!T.warpBody.empty()) {
            int wb = W.U.find(T.warpBody.c_str());
            if (wb >= 0) {
                v->ref = wb;
                v->landed = false;
                v->r = W.U.bodyRot(wb, W.t) * Vec3d{1, 0, 0} * (W.U.bodies[wb].radius + 400e3);
            }
        }
        W.placeInOrbit(*v, 400e3);
        const Body& b = W.U.bodies[v->ref];
        double ra = b.radius + T.warpAp, rp = b.radius + T.warpPe;
        v->r = norm(v->r) * ra;
        v->v = norm(v->v) * std::sqrt(b.GM * (2 / ra - 2 / (ra + rp)));
        v->q = Quatd::lookRot(norm(v->v), norm(v->r));
        W.history.clear();
        W.recordHistory(true);
        std::fprintf(T.log, "warp test: orbit pe %.0f km ap %.0f km, start at the apoapsis (%s, %s)\n", T.warpPe / 1000, T.warpAp / 1000, v->name.c_str(), b.name.c_str());
        return true;
    }
    Vessel* v = W.activeVessel();
    if (T.frames < 4) {
        if (v) std::fprintf(T.log, "  f %d alt %.0f km speed %.0f m/s heat flux %.0f destroyed %d warp %d t %.2f rewound history %zu\n", T.frames, (len(v->r) - W.U.bodies[v->ref].radius) / 1000, len(v->v), v->tHeatFlux, (int)v->destroyed, W.warpIndex, W.t, W.history.size());
        return true;
    }
    if (!v || v->destroyed) {
        std::fprintf(T.log, "warp test: vessel lost (%s, active %d of %zu) t %.2f alt %.0f km\n", v ? v->fate.c_str() : "none", W.active, W.vessels.size(), W.t,
                     v ? (len(v->r) - W.U.bodies[v->ref].radius) / 1000 : 0.0);
        return false;
    }
    const Body& b = W.U.bodies[v->ref];
    double alt = len(v->r) - b.radius;
    bool inAtm = alt < b.atmTop;
    // the player keeps asking for the level (as with the key held): the game clamps it
    if (W.warpIndex != (T.warpIdx >= 0 ? T.warpIdx : WARP_COUNT - 1)) W.setWarp(T.warpIdx >= 0 ? T.warpIdx : WARP_COUNT - 1);
    static double tPrev = 0, wallPrev = 0;
    static int enteredAt = -1, passes = 0;
    static bool wasIn = false;
    if (T.frames == 4) { tPrev = W.t; wallPrev = GetTime(); enteredAt = -1; passes = 0; wasIn = false; }
    if (inAtm != wasIn) {
        if (inAtm) passes++;
        if (inAtm && enteredAt < 0) enteredAt = T.frames;
        Conic k = stateToConic(v->r, v->v, b.GM);
        std::fprintf(T.log, "warp test: %s the atmosphere at frame %d (pass %d), orbit %.0f x %.0f km, warp x%g\n", inAtm ? "entered" : "left", T.frames, passes,
                     (k.pe - b.radius) / 1000, (k.ap - b.radius) / 1000, W.warpFactor());
        wasIn = inAtm;
    }
    if (T.frames % 5 == 0) {
        double wall = GetTime();
        std::string toast = W.toasts.empty() ? "" : W.toasts.back().text;
        Vec3d va = v->v - cross(W.U.angVel(v->ref), v->r);
        double q = 0.5 * W.U.density(v->ref, alt) * dot(va, va);
        std::fprintf(T.log, "  f %4d alt %7.1f km  q %8.2f Pa  heat %.0f kW/m2  warp idx %2d (x%g)  effective x%.0f  reason '%s'  frame %.1f ms  toast '%s'\n",
                     T.frames, alt / 1000, q, v->tHeatFlux / 1000, W.warpIndex, W.warpFactor(), W.effectiveWarp, W.warpLimitReason.c_str(),
                     GetFrameTime() * 1000, toast.c_str());
        std::fflush(T.log);
        tPrev = W.t;
        wallPrev = wall;
    }
    if (T.frames == 60) shot("26_warp_high.png");
    if (T.frames >= 1500) std::fprintf(T.log, "warp test: end, %d passes through the atmosphere, vessel %s\n", passes, v->destroyed ? "lost" : "intact");
    return T.frames < 1500;
}

// ------------------------------------------------------------------ click a vessel to fly it (--picktest)
// two vessels in orbit 60 m apart: in the flight view the other one is hovered and clicked, then the same
// on the map (back to the first one)
bool pickTestStep(App& app) {
    World& W = app.world;
    if (T.frames == 2) {
        W.newGame(Settings{});
        app.gameStarted = true;
        auto tpl = builtinTemplates();
        auto launchTpl = [&](const char* name) {
            for (const Design& d : tpl)
                if (d.name == name) {
                    app.design = d;
                    app.lastLaunched = craftFromDesign(d);
                    W.launch(app.lastLaunched);
                    return;
                }
        };
        launchTpl("Kestrel Orbiter");
        app.screen = Screen::Flight;
        flightOnEnter(app);
        Vessel* a = W.activeVessel();
        W.placeInOrbit(*a, 300e3);
        Vec3d ra = a->r, va = a->v;
        Quatd qa = a->q;
        launchTpl("Aurora Crew Mission");
        Vessel* b = W.activeVessel();
        W.placeInOrbit(*b, 300e3);
        b = W.activeVessel();
        Vec3d side = norm(cross(va, ra));
        b->r = ra + side * 60.0 + norm(va) * 25.0;
        b->v = va;
        b->q = qa;
        flightOnEnter(app);
        std::fprintf(T.log, "pick test: %zu vessels, active %d (%s)\n", W.vessels.size(), W.active, W.activeVessel()->name.c_str());
        return true;
    }
    if (T.frames == 4) flightDebugView(PI * 0.5, 0.15, 140, Vec3d{});
    if (T.frames == 12) flightDebugPick(1);
    if (T.frames == 14) {
        shot("27a_pick_hover.png");
        std::fprintf(T.log, "pick test: flight view, hover %s\n", flightDebugPickSeen() ? "shown" : "not shown -> FAIL");
        flightDebugPick(2);
    }
    if (T.frames == 18) {
        flightDebugPick(0);
        Vessel* v = W.activeVessel();
        std::fprintf(T.log, "pick test: flight view click -> active %d (%s) -> %s\n", W.active, v ? v->name.c_str() : "-",
                     W.active == 0 ? "ok" : "FAIL");
        shot("27b_switched.png");
        flightDebugSetMap(true, 400);
    }
    if (T.frames == 26) flightDebugPick(1);
    if (T.frames == 28) {
        shot("27c_map_hover.png");
        std::fprintf(T.log, "pick test: map, hover %s\n", flightDebugPickSeen() ? "shown" : "not shown -> FAIL");
        flightDebugPick(2);
    }
    if (T.frames == 32) {
        flightDebugPick(0);
        Vessel* v = W.activeVessel();
        std::fprintf(T.log, "pick test: map click -> active %d (%s) -> %s\n", W.active, v ? v->name.c_str() : "-", W.active == 1 ? "ok" : "FAIL");
        flightDebugSetMap(false, 0);
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ rendering cost (--perftest [--nomaps])
// the same views with and without the surface maps: average frame time over 240 frames each (uncapped)
bool perfTestStep(App& app) {
    World& W = app.world;
    struct View { const char* name; const char* body; double lat, lon, alt, pitch, dist; };
    static const View V[] = {{"Earth orbit 400 km", "Earth", 20, 100, 400e3, 0.15, 40},
                             {"Earth low 3 km", "Earth", 35, 138, 3e3, 0.12, 60},
                             {"Moon low 5 km", "Moon", 30, -12, 5e3, 0.15, 60},
                             {"Mars orbit 1500 km", "Mars", 18, -130, 1500e3, 1.48, 40},
                             {"Pacific 300 m", "Earth", 25, -150, 300, 0.06, 60}};
    const int per = 240, warm = 30;
    int vi = T.frames / (per + warm), k = T.frames % (per + warm);
    if (T.frames == 1) {
        ClearWindowState(FLAG_VSYNC_HINT); // (uncapped: a view faster than the display must show its real cost)
        W.newGame(Settings{});
        app.gameStarted = true;
        app.design = builtinTemplates()[2];
        app.lastLaunched = craftFromDesign(app.design);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        T.tfRef = 0;
        T.tfMass0 = 0;
    }
    if (vi >= 5) return false;
    Vessel* v = W.activeVessel();
    if (!v) return false;
    const View& s = V[vi];
    int body = W.U.find(s.body);
    if (k == 0) {
        Vec3d f = siteDir(s.lat, s.lon);
        sunAt(W, body, f, 35);
        placeAt(W, *v, body, f, s.alt);
        flightDebugView(0.3, s.pitch, s.dist, Vec3d{});
        T.tfRef = 0;
    }
    // hold the vessel in place (no physics drift between frames)
    if (k >= 1) {
        Vec3d f = siteDir(s.lat, s.lon);
        placeAt(W, *v, body, f, s.alt);
    }
    if (k == warm) gputime::reset();
    if (k >= warm) T.tfRef += GetFrameTime();
    if (k == per + warm - 1) {
        std::fprintf(T.log, "perf %-20s %s: %.2f ms per frame (%.0f fps), GPU %.0f%%\n", s.name, terrain::map(W.U.find("Earth")) ? "maps   " : "no maps",
                     1000 * T.tfRef / per, per / std::max(T.tfRef, 1e-6), gpumon::utilization());
        for (const gputime::Part& p : gputime::averages())
            std::fprintf(T.log, "     %-10s GPU %6.2f ms   CPU %6.2f ms\n", p.name.c_str(), p.gpuMs, p.cpuMs);
        char name[64];
        std::snprintf(name, sizeof name, "24_perf_%d_%s.png", vi, terrain::map(W.U.find("Earth")) ? "maps" : "nomaps");
        shot(name);
    }
    return true;
}


Craft craftNamed(const char* name);
// ---- structural gaps: parts of a craft that touch nothing (floating) or stand a little apart from the rest (a
// gap). Two parts touch when a point of one's surface lies within tol of the other (both shapes inflated by tol).
static bool insideInflated(const Component& k, const Vec3d& pc, double tol) { return pointInShape(k, pc, tol); }
// panels whose root edge is not held along its whole chord by another part (within tol): the fraction held
static std::vector<std::pair<int, double>> rootsLoose(const std::vector<Component>& comps, double tol) {
    std::vector<std::pair<int, double>> out;
    for (int i = 0; i < (int)comps.size(); i++) {
        if (comps[i].shape != Shape::Airfoil) continue;
        {
            const Component& k = comps[i];
            Vec3d ex = k.rot().rotate(Vec3d{1, 0, 0}), r{k.pos.x, k.pos.y, 0};
            if (k.rootSide == 0 && std::fabs(dot(ex, r)) <= 1e-6 * (1 + len(r))) continue; // (across the axis: held mid-span)
        }
        std::vector<Vec3d> pts;
        airfoilRootEdge(comps[i], pts, 13);
        int held = 0;
        for (const Vec3d& p : pts) {
            bool ok = false;
            for (int j = 0; j < (int)comps.size() && !ok; j++)
                if (j != i && insideInflated(comps[j], p, tol)) ok = true;
            held += ok;
        }
        if (held < (int)pts.size()) out.push_back({i, held / (double)pts.size()});
    }
    return out;
}
static void gapGroups(const std::vector<Component>& comps, double tol, std::vector<int>& group) {
    int n = (int)comps.size();
    std::vector<std::vector<Vec3d>> pts(n);
    for (int i = 0; i < n; i++) {
        const Component& k = comps[i];
        Mat3d R = k.rot().toMat();
        const Vec3d& d = k.dims;
        std::vector<Vec3d> loc;
        if (k.shape == Shape::Loft) {
            // points of the drawn surface (as render loftMesh)
            double nose = std::clamp(k.noseLen, 0.02, 0.6), tail = std::clamp(k.tailLen, 0.02, 0.7);
            if (nose + tail > 0.98) { double q = 0.98 / (nose + tail); nose *= q; tail *= q; }
            double expo = 2.0 / (2.0 + 2.5 * std::clamp(k.boxy, 0.0, 1.0));
            for (int a = 0; a <= 40; a++) {
                double s = 0.5 - 0.5 * std::cos(PI * a / 40), yc = 0, f = 1;
                if (s > 1 - nose) {
                    double t = (1 - s) / nose, e = 1.0 - 0.5 * std::clamp(k.bluff, 0.0, 1.0);
                    f = std::pow(std::max(0.0, 1 - (1 - t) * (1 - t)), e);
                } else if (s < tail) {
                    double t = s / tail, sm = t * t * (3 - 2 * t);
                    yc = k.upsweep * 0.5 * (1 - sm) * (1 - std::clamp(k.tailR, 0.0, 1.0) * 0.5);
                    f = k.tailR + (1 - k.tailR) * std::pow(sm, 0.75);
                }
                for (int b = 0; b < 32; b++) {
                    double th = b * TAU / 32, c = std::cos(th), sn = std::sin(th);
                    double x = 0.5 * f * (c < 0 ? -1 : 1) * std::pow(std::fabs(c), expo);
                    double y = 0.5 * f * (sn < 0 ? -1 : 1) * std::pow(std::fabs(sn), expo) * (1 - std::fabs(yc)) + yc * 0.5;
                    loc.push_back({x * d.x, y * d.y, (s - 0.5) * d.z});
                }
            }
        } else if (k.shape == Shape::Airfoil) {
            double taper = std::clamp(k.taper, 0.1, 1.0), aspect = d.x / std::max(d.y, 1e-4);
            double cr = 2 / (1 + taper), ct = taper * cr, tl = std::tan(std::clamp(k.sweep, -80.0, 80.0) * DEG) * aspect;
            double num = 0, den = 0;
            for (int q = 0; q < 32; q++) {
                double u = (q + 0.5) / 32, c = cr + (ct - cr) * u;
                num += c * (u * tl + c / 2);
                den += c;
            }
            double ys = -num / den;
            int root = airfoilRoot(k);
            for (int a = 0; a <= 12; a++)
                for (int b = 0; b <= 10; b++)
                    for (int sd = -1; sd <= 1; sd += 2) {
                        double u = a / 12.0, xc = b / 10.0, c = cr + (ct - cr) * u;
                        double th = 5 * (0.2969 * std::sqrt(xc) - 0.1260 * xc - 0.3516 * xc * xc + 0.2843 * xc * xc * xc - 0.1036 * xc * xc * xc * xc);
                        double sx = root > 0 ? 0.5 - u : u - 0.5;
                        loc.push_back({sx * d.x, (u * tl + xc * c + ys) * d.y, sd * th * c * d.z});
                    }
        } else {
            std::vector<SurfSample> ss;
            shapeSurface(k.shape == Shape::Truss ? Shape::Box : k.shape, d, 7, ss);
            for (auto& s : ss) loc.push_back(s.p);
        }
        for (const Vec3d& p : loc) pts[i].push_back(k.pos + R * p);
        pts[i].push_back(k.pos);
    }
    group.assign(n, -1);
    std::vector<int> par(n);
    for (int i = 0; i < n; i++) par[i] = i;
    std::function<int(int)> find = [&](int x) { return par[x] == x ? x : par[x] = find(par[x]); };
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++) {
            if (find(i) == find(j)) continue;
            Vec3d hi = shapeHalfExtent(comps[i].shape, comps[i].dims), hj = shapeHalfExtent(comps[j].shape, comps[j].dims);
            if (len(comps[i].pos - comps[j].pos) > len(hi) + len(hj) + 2 * tol) continue;
            bool touch = false;
            for (const Vec3d& p : pts[i]) if (insideInflated(comps[j], p, tol)) { touch = true; break; }
            if (!touch)
                for (const Vec3d& p : pts[j]) if (insideInflated(comps[i], p, tol)) { touch = true; break; }
            if (touch) par[find(i)] = find(j);
        }
    for (int i = 0; i < n; i++) group[i] = find(i);
}
// the parts outside the largest group at tolerance tol
static std::vector<int> gapLoose(const std::vector<Component>& comps, double tol) {
    std::vector<int> g;
    gapGroups(comps, tol, g);
    std::map<int, int> cnt;
    for (int x : g) cnt[x]++;
    int big = -1, bn = -1;
    for (auto& e : cnt)
        if (e.second > bn) { bn = e.second; big = e.first; }
    std::vector<int> out;
    for (size_t i = 0; i < g.size(); i++)
        if (g[i] != big) out.push_back((int)i);
    return out;
}

// ------------------------------------------------------------------ realism (--realtest)
// Flaps (the 9 settings, their lift / drag / stall, the F key), the orbiter's drag chute (rollout with and without
// it, reefing, the 60 kt jettison, a deployment too fast), the payload bay in orbit (doors, release, capture with
// the arm), and pictures of the windows, the bay, the chute, the flaps.
bool realTestStep(App& app) {
    World& W = app.world;
    const int f = T.frames;
    static int bad = 0, S = 0, S0 = 0;
    static double dist[3] = {0, 0, 0}, tJett = -1, vJett = -1, reefCdA = 0;
    static Vec3d start;
    static int payloadId = -1;
    auto check = [&](bool ok, const std::string& what) {
        std::fprintf(T.log, "real: %s -> %s\n", what.c_str(), ok ? "ok" : "FAIL");
        std::fflush(T.log);
        if (!ok) bad++;
    };
    auto orbiterOnly = []() {
        Craft c = craftNamed("Albatross Space Shuttle"), o = c;
        o.comps.clear();
        for (const Component& k : c.comps)
            if (k.stage == 2) {
                Component k2 = k;
                k2.stage = 0;
                k2.fireFrom = -1;
                o.comps.push_back(k2);
            }
        o.name = "Albatross Orbiter";
        return o;
    };
    auto run = [&](int steps) {
        for (int k = 0; k < steps; k++) W.update(0.05);
    };
    if (f == 1) {
        // ---- 1. the flap settings: 0, then a geometric sequence from 1 degree to the full deflection
        double a[FLAP_STAGES + 1];
        std::string s;
        for (int i = 0; i <= FLAP_STAGES; i++) {
            a[i] = flapSettingDeg(i, 50);
            char b[24];
            std::snprintf(b, sizeof b, " %.2f", a[i]);
            s += b;
        }
        double q = a[2] / a[1];
        bool geo = a[0] == 0 && std::fabs(a[1] - 1) < 1e-9 && std::fabs(a[8] - 50) < 1e-9;
        for (int i = 1; i < FLAP_STAGES; i++) geo = geo && std::fabs(a[i + 1] / a[i] - q) < 1e-9;
        check(geo, "flap settings (deg):" + s + "  ratio " + std::to_string(q));
        bool inv = true;
        for (double p = 0; p <= 8; p += 0.37) inv = inv && std::fabs(flapPosOf(flapAngleAt(p, 50), 50) - p) < 1e-6;
        check(inv, "flap position <-> angle round trip");
        // ---- 2. a flapped wing (the Swift's): more lift at the same incidence, more maximum lift, more drag
        Craft sw = craftNamed("Swift Jet Trainer");
        const Component* wing = nullptr;
        for (const Component& k : sw.comps)
            if (k.func == Func::Wing && k.flap > 0) { wing = &k; break; }
        check(wing && wing->flap == 50, "the Swift's wing panels have flaps (50 degrees)");
        if (wing) {
            std::vector<CompPhys> ph(sw.comps.size());
            for (size_t i = 0; i < sw.comps.size(); i++) computeCompPhys(sw.comps[i], ph[i]);
            size_t wi = wing - &sw.comps[0];
            const CompPhys& p = ph[wi];
            Vec3d n = p.axisZ, chord = norm(wing->rot().rotate(Vec3d{0, 1, 0}) * -1.0); // (flight direction: toward the leading edge)
            double fsign = n.y > 0 ? 1 : -1;
            auto force = [&](double alphaDeg, double flapDeg) {
                double al = alphaDeg * DEG;
                Vec3d vAir = (chord * std::cos(al) - n * fsign * std::sin(al)) * 70.0; // the wing moving at 70 m/s
                return wingForce(*wing, p, vAir, 1.225, 70.0 / 340, 0.0, fsign * flapDeg * DEG);
            };
            auto liftOf = [&](const Vec3d& F) { return dot(F, n) * fsign; };
            auto dragOf = [&](const Vec3d& F) { return -dot(F, chord); };
            double L0 = liftOf(force(5, 0)), L50 = liftOf(force(5, 50)), D0 = dragOf(force(5, 0)), D50 = dragOf(force(5, 50));
            double M0 = 0, M50 = 0, aM0 = 0, aM50 = 0;
            for (double al = 0; al <= 30; al += 0.25) {
                double l0 = liftOf(force(al, 0)), l5 = liftOf(force(al, 50));
                if (l0 > M0) { M0 = l0; aM0 = al; }
                if (l5 > M50) { M50 = l5; aM50 = al; }
            }
            char b[300];
            std::snprintf(b, sizeof b, "flaps 50: lift at 5 deg %.1f -> %.1f kN, drag %.2f -> %.2f kN, max lift %.1f kN at %.1f deg -> %.1f kN at %.1f deg",
                          L0 / 1000, L50 / 1000, D0 / 1000, D50 / 1000, M0 / 1000, aM0, M50 / 1000, aM50);
            check(L50 > L0 * 1.5 && D50 > D0 * 2 && M50 > M0 * 1.15 && aM50 < aM0, b);
            // stall speed of the whole aeroplane (two panels, its mass): clean and with full flap
            double m = 0;
            for (auto& c : ph) m += c.dryMass;
            m += 1700; // (fuel)
            double vs0 = 70 * std::sqrt(m * 9.81 / (2 * M0)), vs50 = 70 * std::sqrt(m * 9.81 / (2 * M50));
            std::snprintf(b, sizeof b, "stall speed %.1f t: clean %.0f kt, full flap %.0f kt", m / 1000, vs0 / 0.5144, vs50 / 0.5144);
            check(vs50 < vs0 * 0.93, b);
        }
        {
            // diagnostic: the shuttle stack's aerodynamic force with and without the new parts (windows, bay, chute)
            Craft st = craftNamed("Albatross Space Shuttle");
            std::vector<Component> all = expandCraft(st), less;
            for (const Component& k : all)
                if (k.func != Func::Window && k.func != Func::CargoBay && !(k.func == Func::Parachute)) less.push_back(k);
            auto A = buildAeroTable(all), B = buildAeroTable(less);
            for (double aoa : {0.0, 5.0, -5.0}) {
                Vec3d u = norm(Vec3d{0, std::sin(aoa * DEG), -std::cos(aoa * DEG)}); // (air from ahead)
                auto sa = A->lookup(u, nullptr), sb = B->lookup(u, nullptr);
                std::fprintf(T.log, "real: stack aero at %.0f deg: all F_p (%.1f %.1f %.1f) F_f (%.1f %.1f %.1f) | without new parts F_p (%.1f %.1f %.1f) F_f (%.1f %.1f %.1f)\n",
                             aoa, sa.F_p.x, sa.F_p.y, sa.F_p.z, sa.F_f.x, sa.F_f.y, sa.F_f.z, sb.F_p.x, sb.F_p.y, sb.F_p.z, sb.F_f.x, sb.F_f.y, sb.F_f.z);
            }
        }
        {
            // structural gaps in every built-in craft and template: floating parts (touching nothing within 30 cm)
            // and parts standing apart by 2..30 cm
            std::vector<Craft> all = builtinCrafts();
            for (auto& d : builtinTemplates()) all.push_back(craftFromDesign(d));
            int floating = 0, gaps = 0, roots = 0;
            for (const Craft& c0 : all) {
                std::vector<Component> comps = expandCraft(c0);
                if (std::getenv("SF_GAPONLY"))
                    for (const Component& k : comps)
                        if (k.shape == Shape::Airfoil)
                            std::fprintf(T.log, "  panel %s / %s: span %.2f chord %.2f taper %.3f at %.2f %.2f %.2f\n", c0.name.c_str(), k.name.c_str(),
                                         k.dims.x, k.dims.y, k.taper, k.pos.x, k.pos.y, k.pos.z);
                for (auto& [i, fr] : rootsLoose(comps, 0.02)) {
                    std::fprintf(T.log, "real:   %s: \"%s\" root edge held over %.0f%% of its chord only, at %.2f %.2f %.2f\n", c0.name.c_str(),
                                 comps[i].name.c_str(), fr * 100, comps[i].pos.x, comps[i].pos.y, comps[i].pos.z);
                    roots++;
                }
                std::vector<int> far = gapLoose(comps, 0.30), near = gapLoose(comps, 0.02);
                for (int i : far) {
                    std::fprintf(T.log, "real:   %s: \"%s\" floats (nothing within 30 cm) at %.2f %.2f %.2f\n", c0.name.c_str(), comps[i].name.c_str(),
                                 comps[i].pos.x, comps[i].pos.y, comps[i].pos.z);
                    floating++;
                }
                for (int i : near)
                    if (std::find(far.begin(), far.end(), i) == far.end()) {
                        std::fprintf(T.log, "real:   %s: \"%s\" stands 2-30 cm apart from the rest at %.2f %.2f %.2f\n", c0.name.c_str(),
                                     comps[i].name.c_str(), comps[i].pos.x, comps[i].pos.y, comps[i].pos.z);
                        gaps++;
                    }
            }
            check(floating == 0 && gaps == 0 && roots == 0, "structural gaps in " + std::to_string(all.size()) + " crafts: " + std::to_string(floating) +
                                                  " floating parts, " + std::to_string(gaps) + " parts with a gap, " + std::to_string(roots) +
                                                  " panels with a loose root edge");
            // solar panels open from their root on the craft: while the deploy animation runs, the panel keeps
            // an end at the hinge, and the hinge sits on another part (in the craft's frame and, for a payload,
            // in its own frame once released: the stowing is worked out about the vessel's axis)
            int nPanels = 0, loosePanels = 0;
            for (const Craft& c0 : all) {
                std::vector<Component> comps = expandCraft(c0);
                std::vector<std::vector<Component>> frames{comps};
                {
                    std::vector<Component> cargo;
                    Vec3d axis{};
                    bool haveAxis = false;
                    for (const Component& k : comps)
                        if (k.cargo != 0 && k.func != Func::CargoBay) {
                            cargo.push_back(k);
                            if (k.func == Func::Command && !haveAxis) { axis = {k.pos.x, k.pos.y, 0}; haveAxis = true; }
                        }
                    if (!cargo.empty()) {
                        for (Component& k : cargo) k.pos = k.pos - axis;
                        frames.push_back(cargo);
                    }
                }
                for (size_t fi = 0; fi < frames.size(); fi++) {
                    const std::vector<Component>& fc = frames[fi];
                    std::vector<Component> st = stowedGeometry(fc);
                    for (size_t i = 0; i < fc.size(); i++) {
                        const Component& k = fc[i];
                        if (k.func != Func::Solar || k.shape != Shape::Box) continue;
                        nPanels++;
                        Mat3d R = k.rot().toMat();
                        bool alongX = k.dims.x >= k.dims.z;
                        double L = alongX ? k.dims.x : k.dims.z;
                        Vec3d e = R * (alongX ? Vec3d{1, 0, 0} : Vec3d{0, 0, 1});
                        Vec3d a = k.pos + e * (L / 2), b = k.pos - e * (L / 2);
                        Vec3d hinge = len(a - st[i].pos) < len(b - st[i].pos) ? a : b;
                        std::string why;
                        bool onPart = false;
                        for (size_t j = 0; j < fc.size(); j++)
                            if (j != i && fc[j].func != Func::Solar && pointInShape(fc[j], hinge, 0.15)) onPart = true;
                        if (!onPart) why = "its root touches no other part";
                        for (size_t j = 0; j < fc.size() && why.empty(); j++)
                            if (j != i && fc[j].func != Func::Solar && pointInShape(k, fc[j].pos, 0.05))
                                why = "\"" + fc[j].name + "\" stands in it";
                        for (float t : {0.6f, 0.9f}) {
                            Mat3d Rs;
                            Vec3d ps, ds;
                            gfx::solarPose(k, st[i], t, Rs, ps, ds);
                            Vec3d es = Rs * (ds.x >= ds.z ? Vec3d{1, 0, 0} : Vec3d{0, 0, 1});
                            double Ls = std::max(ds.x, ds.z);
                            double gap = std::min(len(ps + es * (Ls / 2) - hinge), len(ps - es * (Ls / 2) - hinge));
                            if (gap > 0.1 && why.empty()) {
                                char buf[96];
                                std::snprintf(buf, sizeof buf, "at %.0f%% open its nearest end is %.2f m from the root", t * 100, gap);
                                why = buf;
                            }
                        }
                        if (!why.empty()) {
                            std::fprintf(T.log, "real:   %s%s: \"%s\" at %.2f %.2f %.2f: %s\n", c0.name.c_str(), fi ? " (payload)" : "", k.name.c_str(),
                                         k.pos.x, k.pos.y, k.pos.z, why.c_str());
                            loosePanels++;
                        }
                    }
                }
            }
            check(nPanels > 0 && loosePanels == 0, "solar panels open from their root on the craft: " + std::to_string(nPanels - loosePanels) + " / " +
                                                       std::to_string(nPanels));
            if (std::getenv("SF_GAPONLY")) {
                std::fprintf(T.log, "realtest: %s (%d failed)\n", bad ? "FAIL" : "all ok", bad);
                return false;
            }
        }
        // ---- 3. the Swift on the runway: F steps through the settings (8 returns to 0)
        W.newGame(Settings{});
        app.gameStarted = true;
        W.recordsOn = false;
        app.lastLaunched = sw;
        {
            int earth = W.U.find("Earth");
            const LaunchSite& site = LAUNCH_SITES[std::clamp(W.settings.launchSite, 0, LAUNCH_SITE_COUNT - 1)];
            sunAt(W, earth, siteDir(site.latDeg, site.lonDeg), 35);
        }
        W.launchAircraft(sw, W.U.find("Earth"));
        app.screen = Screen::Flight;
        flightOnEnter(app);
        S = 0;
        S0 = 0;
    }
    Vessel* v = W.activeVessel();
    if (!v) return false;
    if (f >= 2 && f <= 19) {
        // a press every other frame: 9 presses
        if (f % 2 == 0) keys::inject(keys::slot(keys::A::Flaps, 0));
        else {
            S0++;
            std::fprintf(T.log, "real: F pressed %d time(s) -> setting %d\n", S0, v->flapStage);
            if (S0 <= 9) {
                int want = S0 % 9;
                if (v->flapStage != want) check(false, "F press " + std::to_string(S0) + ": setting " + std::to_string(v->flapStage));
            }
        }
        return true;
    }
    if (f == 22) {
        check(v->flapStage == 0 && v->hasFlaps, "9 presses of F: 1, 2, ... 8, then back to 0");
        while (v->flapStage != 8) W.cycleFlaps(*v);
        run(300); // 15 s: the flaps travel down
        char b[120];
        std::snprintf(b, sizeof b, "setting 8 reached after 15 s: position %.2f (%.1f deg)", v->flapPos, flapAngleAt(v->flapPos, v->flapMax));
        check(std::fabs(v->flapPos - 8) < 1e-6, b);
        app.hudHidden = true;
        flightDebugView(2.4, 0.35, 18, Vec3d{});
    }
    if (f == 28) shot("30_swift_flaps.png");
    if (f == 29) flightDebugView(0.6, 0.35, 9, Vec3d{});
    if (f == 34) shot("30_swift_canopy.png");
    // ---- 4. the orbiter's rollout from 100 m/s: brakes only, with the drag chute, chute deployed too fast
    if (f == 36 || f == 120 || f == 200) {
        S = f == 36 ? 0 : f == 120 ? 1 : 2;
        W.newGame(Settings{});
        app.gameStarted = true;
        W.recordsOn = false;
        Craft o = orbiterOnly();
        app.lastLaunched = o;
        {
            int earth = W.U.find("Earth");
            const LaunchSite& site = LAUNCH_SITES[std::clamp(W.settings.launchSite, 0, LAUNCH_SITE_COUNT - 1)];
            sunAt(W, earth, siteDir(site.latDeg, site.lonDeg), 35);
        }
        W.launchAircraft(o, W.U.find("Earth"));
        app.screen = Screen::Flight;
        flightOnEnter(app);
        v = W.activeVessel();
        run(2);
        Vec3d up = norm(v->r), fwd = v->q.rotate({0, 0, 1});
        fwd = norm(fwd - up * dot(fwd, up));
        v->landed = false;
        v->rolling = true;
        v->legsDeployed = true;
        v->throttle = 0;
        double sp0 = S == 2 ? 130.0 : 100.0;
        v->v = cross(W.U.angVel(v->ref), v->r) + fwd * sp0;
        start = W.U.bodyRot(v->ref, W.t).T() * v->r; // (body-fixed: the ground turns meanwhile)
        tJett = vJett = -1;
        reefCdA = 0;
        if (S >= 1) {
            v->chuteDeployed = true;
            v->chuteCut = false;
        }
        app.hudHidden = true;
        flightDebugView(3.0, 0.22, 90, Vec3d{});
        return true;
    }
    if ((f > 36 && f < 118) || (f > 120 && f < 198) || (f > 200 && f < 230)) {
        for (int k = 0; k < 10; k++) {
            run(1);
            if (v->chuteCut && tJett < 0 && S == 1) { tJett = W.t; vJett = len(W.surfaceVel(*v)); }
            if (S == 1 && v->chuteT > 0.5 && v->chuteT < 3.0 && reefCdA == 0) reefCdA = v->chuteT;
        }
        double sp = len(W.surfaceVel(*v));
        if (S == 1 && f == 124) shot("30_drag_chute_reefed.png");
        if (S == 1 && f == 140) shot("30_drag_chute.png");
        if (S == 2 && f > 203) {
            check(v->chuteCut, "drag chute deployed at 130 m/s (253 kt): torn off (limit 230 kt)");
            T.frames = 229;
        }
        if (sp < 0.5 && S < 2) {
            dist[S] = len(W.U.bodyRot(v->ref, W.t).T() * v->r - start);
            std::fprintf(T.log, "real: rollout from 100 m/s %s: %.0f m\n", S ? "with the drag chute" : "on the brakes", dist[S]);
            if (S == 1) {
                char b[200];
                std::snprintf(b, sizeof b, "drag chute: rollout %.0f m -> %.0f m (%.0f m shorter), cast off at %.1f m/s (%.0f kt)", dist[0], dist[1], dist[0] - dist[1],
                              vJett, vJett / 0.5144);
                check(dist[1] < dist[0] - 150 && vJett > 0 && vJett < 31 && vJett > 25, b);
            }
            T.frames = S == 0 ? 119 : 199;
        }
        return true;
    }
    // ---- 5. the payload bay in orbit
    if (f == 232) {
        W.newGame(Settings{});
        app.gameStarted = true;
        W.recordsOn = false;
        Craft o = orbiterOnly();
        app.lastLaunched = o;
        W.launchInOrbit(o, -1, 0);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        v = W.activeVessel();
        v->sas = SAS::Stability;
        {
            // onto the day side: the orbit turned so that the sun stands 50 degrees high over the vessel
            int earth = W.U.find("Earth"), star = W.U.hostStar(earth);
            W.refreshStates();
            Vec3d sd = norm(W.U.relPos(W.states, star, earth)), rr = norm(v->r), ax = cross(rr, sd);
            double sn = len(ax);
            if (sn > 1e-9) {
                Quatd qr = Quatd::axisAngle(ax / sn, std::atan2(sn, dot(rr, sd)) - 40 * DEG);
                v->r = qr.rotate(v->r);
                v->v = qr.rotate(v->v);
                v->q = (qr * v->q).normalized();
            }
        }
        int cargo = 0;
        for (auto& c : v->comps) cargo += c.cargo;
        check(v->bayComp >= 0 && cargo >= 5, "orbiter in orbit with its payload bay and " + std::to_string(cargo) + " payload parts");
        // the payload's motor and tank are not the orbiter's
        double hyp = 0;
        for (size_t i = 0; i < v->comps.size(); i++)
            if (v->comps[i].cargo && v->comps[i].func == Func::Tank) hyp += v->prop[i];
        check(hyp > 1000, "the payload's propellant stays aboard it");
        W.releasePayload(*v);
        check(W.vessels.size() == 1 || W.activeVessel()->comps.size() == v->comps.size(), "release refused while the doors are closed");
        app.hudHidden = true;
        flightDebugView(0.8, 0.45, 55, Vec3d{});
        return true;
    }
    if (f == 236) shot("30_orbiter_windows.png");
    if (f == 237) flightDebugView(0.0, 0.35, 34, Vec3d{});
    if (f == 239) shot("30_orbiter_view_a.png");
    if (f == 240) flightDebugView(1.57, 0.35, 34, Vec3d{});
    if (f == 241) shot("30_orbiter_view_b.png");
    if (f == 242) {
        shot("30_orbiter_windshield.png");
        flightDebugView(3.14, 0.35, 34, Vec3d{});
    }
    if (f == 243) shot("30_orbiter_view_c.png");
    if (f == 244) {
        flightDebugView(4.71, 0.35, 34, Vec3d{});
        W.setBayDoors(*v, true);
        run(400); // 20 s: half open
        flightDebugView(0.9, 0.7, 60, Vec3d{});
    }
    if (f == 246) shot("30_orbiter_view_d.png");
    if (f == 247) {
        flightDebugView(0.9, 0.7, 60, Vec3d{});
        shot("30_bay_opening.png");
        run(1000); // 50 s more: open, the radiators out
        check(v->bayT >= 0.999, "payload bay doors open after about a minute");
    }
    if (f == 252) shot("30_bay_open.png");
    if (f == 253) {
        double m0 = W.vesselMass(*v);
        size_t n0 = W.vessels.size();
        Vec3d v0 = v->v;
        bool ok = W.releasePayload(*v);
        v = W.activeVessel();
        Vessel* p = ok && W.vessels.size() == n0 + 1 ? &W.vessels.back() : nullptr;
        if (p) payloadId = p->id;
        char b[220];
        std::snprintf(b, sizeof b, "released \"%s\": orbiter %.1f t -> %.1f t, payload %.2f t, separation %.2f m/s", p ? p->name.c_str() : "-", m0 / 1000,
                      W.vesselMass(*v) / 1000, p ? W.vesselMass(*p) / 1000 : 0.0, p ? len(p->v - v->v) : 0.0);
        check(p && std::fabs(m0 - W.vesselMass(*v) - W.vesselMass(*p)) < 1.0 && std::fabs(W.vesselMass(*p) - 3000) < 60 && len(p->v - v->v) > 0.25 &&
                  len(p->v - v->v) < 0.35 && p->hasControl,
              b);
        (void)v0;
        run(400); // 20 s: it rises out of the bay
        // its solar wings opening (the shot catches them part way: both from the bus)
        for (auto& o : W.vessels)
            if (o.id == payloadId && !o.destroyed) W.setSolarDeployed(o, true);
        run(50);
        v = W.activeVessel();
        flightDebugView(0.9, 0.55, 70, Vec3d{});
    }
    if (f == 258) shot("30_payload_released.png");
    if (f == 259 || f == 261) {
        // close up on the payload: its solar wings part way open (the camera's frame: east, north, up at the orbiter)
        for (auto& o : W.vessels)
            if (o.id == payloadId && !o.destroyed) {
                Vec3d up, north, east, d = o.r - v->r;
                flightDebugHorizon(W, *v, up, north, east);
                flightDebugView(f == 259 ? 0.9 : 2.6, 0.35, 13, Vec3d{dot(d, east), dot(d, north), dot(d, up)});
            }
    }
    if (f == 260) shot("30_payload_wings_a.png");
    if (f == 262) shot("30_payload_wings_b.png");
    if (f == 263) {
        // the navball panel with the orbiter's second column (bay doors, release, capture) inside its frame
        app.hudHidden = false;
        flightDebugView(0.9, 0.55, 70, Vec3d{});
    }
    if (f == 264) shot("30_orbiter_hud.png");
    if (f == 265) app.hudHidden = true;
    if (f == 266) {
        Vessel* p = nullptr;
        for (auto& o : W.vessels)
            if (o.id == payloadId && !o.destroyed) p = &o;
        std::string why;
        int cand = W.captureCandidate(*v, &why);
        check(cand < 0, "capture refused while it drifts away at 0.3 m/s: " + why);
        if (p) {
            // stop it 8 m above the open bay, matching the orbiter's motion
            p->v = v->v;
            p->w = v->w;
            p->r = v->r + v->q.rotate(Vec3d{0, 9, 1});
            W.setSolarDeployed(*p, false); // (its wings folded again: open they do not fit in the bay)
            size_t nc = v->comps.size();
            bool ok = W.capturePayload(*v);
            v = W.activeVessel();
            int cargo = 0;
            for (auto& c : v->comps) cargo += c.cargo;
            check(ok && p->destroyed && v->comps.size() > nc && cargo >= 5 && std::fabs(W.vesselMass(*v) - 0) > 0,
                  "captured with the arm: " + std::to_string(v->comps.size() - nc) + " parts back in the bay");
            // and out again
            check(W.releasePayload(*v), "released again");
        } else check(false, "the released payload is gone");
        W.setBayDoors(*v, false);
        run(1300);
        check(v->bayT <= 0.001, "doors closed for the entry");
        // ---- the SAS pointing modes keep the roll: belly-down, prograde -> retrograde -> prograde stays belly-down
        // (a yaw round, not a pitch over the top), and a craft rolled 90 degrees stays rolled 90 degrees
        {
            auto rollNow = [&](Vessel& o) {
                Vec3d nose = o.q.rotate({0, 0, 1}), upB = o.q.rotate({0, 1, 0}), vert = norm(o.r);
                Vec3d rp = norm(vert - nose * dot(vert, nose));
                return std::atan2(dot(cross(rp, upB), nose), dot(rp, upB)) / DEG;
            };
            auto turnTo = [&](SAS m, double wantRoll, const char* what) {
                v = W.activeVessel();
                v->sas = m;
                v->rcs = true;
                double tS = -1, worst = 0;
                for (int k = 0; k < 6000 && tS < 0; k++) {
                    W.update(0.05);
                    Vec3d t = W.targetDirection(*v, m), nose = v->q.rotate({0, 0, 1});
                    double dr = std::fabs(std::remainder(rollNow(*v) - wantRoll, 360.0));
                    if (std::fabs(dot(nose, norm(v->r))) < 0.95) worst = std::max(worst, dr);
                    if (dot(t, nose) > 0.9995 && len(v->w) < 0.003) tS = (k + 1) * 0.05;
                }
                double dr = std::fabs(std::remainder(rollNow(*v) - wantRoll, 360.0));
                char b[240];
                std::snprintf(b, sizeof b, "SAS %s: on the target in %.0f s, roll %.1f deg (want %.0f), worst roll error on the way %.1f deg", what, tS,
                              rollNow(*v), wantRoll, worst);
                check(tS > 0 && dr < 3 && worst < 20, b);
            };
            v = W.activeVessel();
            v->q = Quatd::lookRot(W.targetDirection(*v, SAS::Prograde), norm(v->r));
            v->w = {};
            v->sas = SAS::Prograde;
            run(40);
            turnTo(SAS::Retrograde, 0, "prograde -> retrograde, belly-down");
            turnTo(SAS::Prograde, 0, "retrograde -> prograde, belly-down");
            v->q = (v->q * Quatd::axisAngle({0, 0, 1}, 90 * DEG)).normalized();
            v->w = {};
            v->sas = SAS::Stability;
            run(40);
            turnTo(SAS::Retrograde, rollNow(*v), "prograde -> retrograde, rolled 90 deg");
            turnTo(SAS::Normal, rollNow(*v), "retrograde -> normal, rolled");
            v->sas = SAS::Off;
        }
        std::fprintf(T.log, "realtest: %s (%d failed)\n", bad ? "FAIL" : "all ok", bad);
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ the sea (--seatest)
// views of the open Pacific: just above the water, at 300 m and 10 km, from orbit, with the sun high, low and
// set, looking toward the sun (glitter) and away from it
bool realTestStep(App& app);
bool seaTestStep(App& app) {
    World& W = app.world;
    struct View { const char* name; double lat, lon, alt, sunEl, yaw, pitch, dist; };
    static const View V[] = {{"sea_20m_sun", 25, -150, 20, 25, 0.0, 0.02, 40},      {"sea_20m_sun_b", 25, -150, 20, 25, 1.57, 0.02, 40},
                             {"sea_20m_sun_c", 25, -150, 20, 25, 3.14, 0.02, 40},  {"sea_20m_sun_d", 25, -150, 20, 25, 4.71, 0.02, 40},
                             {"sea_300m", 25, -150, 300, 35, 0.3, 0.25, 60},       {"sea_10km", 25, -150, 10e3, 30, 2.55, 0.35, 80},
                             {"sea_orbit", 10, -160, 400e3, 50, 2.55, 0.6, 60},     {"sea_low_sun", 25, -150, 200, 4, 2.55, 0.05, 60},
                             {"sea_night", 25, -150, 300, -20, 0.3, 0.25, 60},     {"sea_down", 25, -150, 60, 40, 0.3, 1.2, 30}};
    const int N = sizeof V / sizeof V[0], per = 24;
    int vi = (T.frames - 1) / per, k = (T.frames - 1) % per;
    if (T.frames == 1) {
        W.newGame(Settings{});
        app.gameStarted = true;
        app.lastLaunched = craftFromDesign(builtinTemplates()[0]);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        app.hudHidden = true;
        W.recordsOn = false; // (no achievement banners in the pictures)
    }
    if (vi >= N) return false;
    Vessel* v = W.activeVessel();
    if (!v) return false;
    const View& s = V[vi];
    int body = W.U.find("Earth");
    Vec3d f = siteDir(s.lat, s.lon);
    if (k == 0) {
        sunAt(W, body, f, s.sunEl);
        flightDebugView(s.yaw, s.pitch, s.dist, Vec3d{});
    }
    placeAt(W, *v, body, f, s.alt); // (held in place)
    if (k == per - 1) {
        char name[64];
        std::snprintf(name, sizeof name, "30_%s.png", s.name);
        shot(name);
    }
    return true;
}

// ------------------------------------------------------------------ truss structure (--trusstest)
// the lattice in the CAD view and in orbit next to a station module; its mass from the members' tubes
Craft craftNamed(const char* name);
bool trussTestStep(App& app) {
    World& W = app.world;
    const int f = T.frames;
    if (f == 2) {
        Craft c = craftNamed("Station Truss Segment");
        cadSetCraft(app, c);
        app.screen = Screen::CAD;
        // the truss alone: the members' mass (aluminium tubes)
        Vessel tmp;
        Component tr;
        for (const Component& k : c.comps)
            if (k.shape == Shape::Truss) tr = k;
        tmp.comps = {tr};
        setupVessel(tmp, false);
        std::vector<std::pair<Vec3d, Vec3d>> mem;
        trussMembers(tr.dims, mem);
        double m = tmp.mp.mass;
        std::fprintf(T.log, "truss: %zu members, tube radius %.3f m, %.0f kg for %.1f x %.1f x %.1f m\n", mem.size(), trussTubeRadius(tr.dims), m,
                     tr.dims.x, tr.dims.y, tr.dims.z);
        bool ok = m > 300 && m < 2000 && mem.size() > 40;
        std::fprintf(T.log, "truss: lattice mass -> %s\n", ok ? "ok" : "FAIL");
    }
    if (f == 30) shot("60_truss_cad.png");
    if (f == 31) {
        W.newGame(Settings{});
        app.gameStarted = true;
        int earth = W.U.find("Earth");
        sunAt(W, earth, siteDir(0, 0), 30);
        W.launchInOrbit(craftNamed("Station Core Module"), -1, 0);
        int core = W.activeVessel()->id;
        W.launchInOrbit(craftNamed("Station Truss Segment"), core, 25);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        flightDebugView(0.8, 0.3, 40, Vec3d{});
    }
    if (f == 60) shot("61_truss_orbit.png");
    if (f == 61) {
        std::fprintf(T.log, "truss: in orbit -> ok\n");
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ every body seen from close by (--bodytest)
// the vessel on the day side of each body of the Solar System at two heights (30 % and 3 % of its radius), the camera
// looking down at the ground ahead: the surface (map, generated ground, clouds, gas-giant bands) as it should look
bool bodyTestStep(App& app) {
    World& W = app.world;
    static std::vector<int> list;
    static int f = -1;
    f++;
    const int per = 30;
    if (f == 0) {
        W.newGame(Settings{});
        app.gameStarted = true;
        W.recordsOn = false;
        app.lastLaunched = craftFromDesign(builtinTemplates()[0]);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        app.hudHidden = true;
        for (int i = 0; i < W.U.count(); i++) {
            const Body& b = W.U.bodies[i];
            if (b.isStar || b.system != "Solar System" || b.comet) continue;
            if (const char* only = std::getenv("SF_BODIES"))
                if (std::string(only).find(b.name) == std::string::npos) continue;
            list.push_back(i);
        }
    }
    int vi = f / per, k = f % per;
    if (vi >= (int)list.size() * 2) {
        std::fprintf(T.log, "bodytest: done\n");
        return false;
    }
    int bi = list[vi / 2];
    bool low = vi % 2 == 1;
    const Body& b = W.U.bodies[bi];
    Vessel* v = W.activeVessel();
    if (!v) return false;
    if (k == 0) {
        W.refreshStates();
        Vec3d sun = norm(W.U.relPos(W.states, W.U.hostStar(bi), bi));
        // a place with the Sun 50 degrees up, at the body's rotation now
        Vec3d e = norm(cross(sun, Vec3d{0, 0, 1}));
        Vec3d u = norm(sun * std::sin(50 * DEG) + e * std::cos(50 * DEG));
        Vec3d fb = W.U.bodyRot(bi, W.t).T() * u;
        double km = std::getenv("SF_BODYALT") ? std::atof(std::getenv("SF_BODYALT")) : 1.0; // (diagnostics: other heights)
        placeAt(W, *v, bi, fb, b.radius * (low ? 0.03 : 0.3) * km);
        v->q = Quatd::lookRot(norm(v->r), anyPerp(norm(v->r)));
        flightDebugView(0.3, low ? 0.35 : 0.55, std::max(40.0, v->length * 3), Vec3d{});
    }
    if (k == per - 1) {
        char name[96];
        std::snprintf(name, sizeof name, "94_body_%02d_%s_%s.png", bi, b.name.c_str(), low ? "low" : "high");
        for (char* c = name; *c; c++)
            if (*c == ' ' || *c == '\'') *c = '_';
        shot(name);
        std::fprintf(T.log, "body: %s %s alt %.0f km (ref %s, soi %.1f km)\n", b.name.c_str(), low ? "low" : "high",
                     (len(v->r) - W.U.bodies[v->ref].radius) / 1000, W.U.bodies[v->ref].name.c_str(), b.soi / 1000);
    }
    return true;
}

// ------------------------------------------------------------------ the night sky (--startest)
// the vessel in a 400 km orbit on the night side, the camera on well-known parts of the sky: the star catalogue
// and the Milky Way where they belong
bool starTestStep(App& app) {
    World& W = app.world;
    struct View { const char* name; double ra, dec, fovDist; };
    static const View V[] = {{"sagittarius", 266.4, -28.9, 1}, {"orion", 84.0, -1.0, 1}, {"crux", 187.0, -60.0, 1},
                             {"ursa_major", 165.0, 56.0, 1}, {"cygnus", 305.0, 40.0, 1}};
    const int N = (int)(sizeof V / sizeof V[0]), per = 20;
    static int f = -1;
    f++;
    int vi = f / per, k = f % per;
    if (f == 0) {
        W.newGame(Settings{});
        app.gameStarted = true;
        W.recordsOn = false;
        app.design = builtinTemplates()[0];
        app.lastLaunched = craftFromDesign(app.design);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        app.hudHidden = true;
    }
    if (vi >= N) {
        // the map: the asteroid belt and Jupiter's Trojans, the Kuiper belt, a comet with its tails
        static const struct { const char* body; double distAU, pitch; const char* name; } M[] = {
            {"Sun", 9.0, 0.9, "92_map_asteroid_belt"}, {"Sun", 110.0, 0.7, "92_map_kuiper_belt"}, {"Encke's Comet", 0.12, 0.5, "92_map_encke"},
            {"Sun", 6.0, 0.25, "92_map_inner_side"},
            // the bodies with spacecraft maps (a negative distance: in body radii)
            {"Mercury", -3.2, 0.3, "93_body_mercury"}, {"Io", -3.2, 0.3, "93_body_io"}, {"Europa", -3.2, 0.3, "93_body_europa"},
            {"Ganymede", -3.2, 0.3, "93_body_ganymede"}, {"Callisto", -3.2, 0.3, "93_body_callisto"}, {"Jupiter", -3.2, 0.3, "93_body_jupiter"},
            {"Enceladus", -3.2, 0.3, "93_body_enceladus"}, {"Triton", -3.2, -0.6, "93_body_triton"}, {"Pluto", -3.2, 0.3, "93_body_pluto"},
            {"Charon", -3.2, 0.3, "93_body_charon"}, {"Ceres", -3.2, 0.3, "93_body_ceres"}, {"Phobos", -3.2, 0.3, "93_body_phobos"}};
        const int NM = (int)(sizeof M / sizeof M[0]);
        int mi = (f - N * per) / 25, mk = (f - N * per) % 25;
        if (mi >= NM) {
            std::fprintf(T.log, "startest: done\n");
            return false;
        }
        if (mk == 0) {
            app.hudHidden = false;
            int bi = W.U.find(M[mi].body);
            double dist = M[mi].distAU > 0 ? M[mi].distAU * AU : -M[mi].distAU * W.U.bodies[bi].radius;
            // (the camera on the lit side: yaw toward the Sun as seen from the body)
            Vec3d sd = norm(W.U.relPos(W.states, W.U.hostStar(bi), bi));
            double yaw = M[mi].distAU > 0 ? 0.6 : std::atan2(sd.y, sd.x) + 0.5;
            flightDebugMapFocus(bi, dist, yaw, M[mi].pitch);
            app.hudHidden = M[mi].distAU < 0;
        }
        if (mk == 24) shot((std::string(M[mi].name) + ".png").c_str());
        return true;
    }
    Vessel* v = W.activeVessel();
    if (!v) return false;
    const View& s = V[vi];
    int earth = W.U.find("Earth");
    W.refreshStates();
    Vec3d sun = norm(W.U.relPos(W.states, W.U.hostStar(earth), earth));
    Vec3d t = raDecToEcl(s.ra, s.dec);
    // a place under that part of the sky, on the night side
    Vec3d u = t;
    if (dot(t, sun) > -0.6) u = norm(t + sun * (-0.6 - dot(t, sun)));
    Vec3d fb = W.U.bodyRot(earth, W.t).T() * u;
    placeAt(W, *v, earth, fb, 400e3);
    v->q = Quatd::lookRot(norm(v->v), norm(v->r));
    Vec3d up, north, east;
    flightDebugHorizon(W, *v, up, north, east);
    Vec3d l{dot(t, east), dot(t, north), dot(t, up)};
    Vec3d d = -l; // (the camera sits on the far side of its target from where it looks)
    flightDebugView(std::atan2(d.y, d.x), std::asin(clampd(d.z, -1, 1)), 1.0, l * 3000.0);
    if (k == per - 1) {
        std::fprintf(T.log, "star: %s at elevation %.0f deg, sun elevation %.0f deg\n", s.name, std::asin(clampd(l.z, -1, 1)) / DEG,
                     std::asin(clampd(dot(sun, up), -1, 1)) / DEG);
        shot((std::string("91_sky_") + s.name + ".png").c_str());
    }
    return true;
}

// ------------------------------------------------------------------ solar panels deploying (--solartest)
// every craft with solar panels (and the Shuttle's payload on its own) in orbit, nose east, belly down; pictures at
// 0, 25, 55 and 100 % of the deploy animation, the camera on the panels
bool solarTestStep(App& app) {
    World& W = app.world;
    static std::vector<Craft> crafts;
    static int ci = -1, k = 0;
    static const float TS[] = {0.0f, 0.25f, 0.55f, 1.0f};
    const int PER = 4, STEP = 12; // frames per picture
    if (ci < 0) {
        std::vector<Craft> all = builtinCrafts();
        for (auto& d : builtinTemplates()) all.push_back(craftFromDesign(d));
        for (const Craft& c0 : all) {
            std::vector<Component> comps = expandCraft(c0);
            bool any = false, cargoSolar = false;
            for (const Component& q : comps)
                if (q.func == Func::Solar && q.shape == Shape::Box) {
                    any = true;
                    if (q.cargo) cargoSolar = true;
                }
            if (!any) continue;
            if (!cargoSolar) crafts.push_back(c0);
            else {
                // the payload on its own, as it flies once released (about its own axis)
                Craft cc;
                cc.name = c0.name + " payload";
                Vec3d axis{};
                bool haveAxis = false;
                for (const Component& q : comps)
                    if (q.cargo != 0 && q.func != Func::CargoBay && q.func == Func::Command && !haveAxis) {
                        axis = {q.pos.x, q.pos.y, 0};
                        haveAxis = true;
                    }
                for (Component q : comps)
                    if (q.cargo != 0 && q.func != Func::CargoBay) {
                        q.pos = q.pos - axis;
                        q.cargo = 0;
                        q.stage = 0;
                        q.symmetry = 1;
                        cc.comps.push_back(q);
                    }
                crafts.push_back(cc);
            }
        }
        std::fprintf(T.log, "solar: %d crafts with solar panels\n", (int)crafts.size());
        ci = 0;
        k = 0;
    }
    if (ci >= (int)crafts.size()) {
        std::fprintf(T.log, "solartest: done\n");
        return false;
    }
    static int f = -1;
    f++;
    if (f % (PER * STEP + 4) == 0) {
        // the next craft
        const Craft& c = crafts[ci];
        W.newGame(Settings{});
        app.gameStarted = true;
        W.recordsOn = false;
        app.lastLaunched = c;
        W.launch(c);
        Vessel* v = W.activeVessel();
        if (!v) { ci++; return true; }
        W.placeInOrbit(*v, 400e3);
        v->sas = SAS::Off;
        v->w = {};
        app.screen = Screen::Flight;
        flightOnEnter(app);
        app.hudHidden = true;
        int earth = W.U.find("Earth");
        sunAt(W, earth, norm(v->r), 50);
        k = 0;
    }
    Vessel* v = W.activeVessel();
    if (!v) { ci++; return true; }
    int local = f % (PER * STEP + 4);
    int pic = std::min(PER - 1, local / STEP);
    // nose east, belly down; the panels' deploy state held
    Vec3d up, north, east;
    flightDebugHorizon(W, *v, up, north, east);
    v->q = Quatd::lookRot(east, up);
    v->w = {};
    v->solarDeployed = true;
    v->solarT = TS[pic];
    // the camera on the panels: their middle and their spread
    Vec3d lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    for (size_t i = 0; i < v->comps.size(); i++)
        if (v->comps[i].func == Func::Solar) {
            Vec3d p = v->phys[i].centre;
            double r = std::max(v->comps[i].dims.x, v->comps[i].dims.z) * 0.6;
            lo = {std::min(lo.x, p.x - r), std::min(lo.y, p.y - r), std::min(lo.z, p.z - r)};
            hi = {std::max(hi.x, p.x + r), std::max(hi.y, p.y + r), std::max(hi.z, p.z + r)};
        }
    Vec3d mid = (lo + hi) * 0.5 - v->mp.com, wmid = v->q.rotate(mid);
    double spread = std::max(3.0, len(hi - lo));
    flightDebugView(0.75, 0.45, spread * 1.5, Vec3d{dot(wmid, east), dot(wmid, north), dot(wmid, up)});
    if (local % STEP == STEP - 1 && local < PER * STEP) {
        char name[128];
        std::snprintf(name, sizeof name, "90_solar_%02d_%d.png", ci, (int)(TS[pic] * 100));
        shot(name);
        if (pic == 0) {
            std::fprintf(T.log, "solar: %02d %s\n", ci, crafts[ci].name.c_str());
            std::vector<Component> st = stowedGeometry(v->comps);
            for (size_t i = 0; i < v->comps.size(); i++) {
                const Component& q = v->comps[i];
                if (q.func != Func::Solar) continue;
                std::fprintf(T.log, "  panel %-22s radial %d dims %.2f %.2f %.2f rot %.0f %.0f %.0f pos %.2f %.2f %.2f -> stowed dims %.2f %.2f %.2f pos %.2f %.2f %.2f\n",
                             q.name.c_str(), (int)solarRadialMount(q), q.dims.x, q.dims.y, q.dims.z, q.rotDeg.x, q.rotDeg.y, q.rotDeg.z, q.pos.x, q.pos.y, q.pos.z,
                             st[i].dims.x, st[i].dims.y, st[i].dims.z, st[i].pos.x, st[i].pos.y, st[i].pos.z);
            }
        }
    }
    if (local == PER * STEP + 3) ci++;
    return true;
}

// ------------------------------------------------------------------ CAD / HUD / factory features (--uitest)
// a truss selected in the CAD (its property rows: crashed before), a payload fairing made from a selection, cargo
// loaded into the Shuttle's bay (the relay satellite and its cradle go), the launch site from the CAD, overlapping
// HUD panels in z-order, the night vision modes, the factory's save dialog
bool uiTestStep(App& app) {
    World& W = app.world;
    const int f = T.frames;
    static int bad = 0;
    auto check = [&](bool ok, const std::string& what) {
        std::fprintf(T.log, "ui: %s -> %s\n", what.c_str(), ok ? "ok" : "FAIL");
        if (!ok) bad++;
    };
    static Craft fc; // the fairing craft (launched later)
    if (f == 2) {
        Craft c = craftNamed("Station Truss Segment");
        cadSetCraft(app, c);
        app.screen = Screen::CAD;
        int ti = -1;
        for (int i = 0; i < (int)c.comps.size(); i++)
            if (c.comps[i].shape == Shape::Truss) ti = i;
        cadDebugSet(ti, 0);
        cadDebugHome(app);
    }
    if (f == 20) {
        shot("80_ui_truss_props.png");
        check(true, "a truss selected in the CAD: its properties drawn");
    }
    if (f == 21) {
        // a stage (3 m tank) and a payload box on it: the payload selected, enclosed
        Craft c;
        c.name = "Fairing test";
        Component tk = makeComponent(Func::Tank);
        tk.dims = {1.5, 1.5, 5.0};
        tk.pos = {0, 0, -2.5};
        c.comps.push_back(tk);
        Component pb = makeComponent(Func::Structure);
        pb.shape = Shape::Box;
        pb.dims = {2.0, 2.0, 3.0};
        pb.pos = {0, 0, 1.5};
        c.comps.push_back(pb);
        cadSetCraft(app, c);
        cadDebugSelect({1});
        bool ok = cadDebugFairing(app);
        int nf = 0;
        double rMin = 1e30, zTop = -1e30, zBot = 1e30;
        for (const Component& k : app.craft.comps)
            if (k.func == Func::Fairing) {
                nf++;
                if (k.dims.x == k.dims.y) rMin = std::min(rMin, k.dims.x);
                zTop = std::max(zTop, k.pos.z + k.dims.z / 2);
                zBot = std::min(zBot, k.pos.z - k.dims.z / 2);
            }
        char b[200];
        std::snprintf(b, sizeof b, "fairing from the selection: %d pieces, radius %.2f m (payload corner 1.41 m), %.2f .. %.2f m (payload 0 .. 3 m)", nf,
                      rMin, zBot, zTop);
        check(ok && nf >= 14 && nf % 2 == 0 && rMin > 1.42 && rMin < 1.7 && zBot <= 0.01 && zTop > 3.5, b);
        // again: the old fairing is replaced, not doubled
        cadDebugSelect({1});
        cadDebugFairing(app);
        int nf2 = 0;
        for (const Component& k : app.craft.comps) nf2 += k.func == Func::Fairing;
        check(nf2 == nf, "enclosing again replaces the fairing (" + std::to_string(nf2) + " pieces)");
        fc = app.craft;
        cadDebugSelect({});
        cadDebugHome(app);
    }
    if (f == 40) shot("81_ui_fairing.png");
    if (f == 41) {
        Craft sh = craftNamed("Albatross Space Shuttle");
        cadSetCraft(app, sh);
        bool ok = cadDebugIntoBay(app, craftNamed("Station Truss Segment"));
        int relay = 0, cradle = 0, cargo = 0;
        for (const Component& k : app.craft.comps) {
            relay += k.name == "Relay satellite" || k.name == "Solar wing" || k.name == "Apogee motor";
            cradle += k.name == "Payload cradle";
            cargo += k.cargo != 0 && k.func != Func::CargoBay;
        }
        check(ok && relay == 0 && cradle == 0 && cargo > 0, "cargo loaded into the Shuttle's bay replaces the relay satellite: " + std::to_string(relay) +
                                                                " satellite parts, " + std::to_string(cradle) + " cradles, " + std::to_string(cargo) +
                                                                " cargo parts");
        cadDebugSelect({});
        cadDebugHome(app);
    }
    if (f == 60) shot("82_ui_bay_cargo.png");
    if (f == 61) {
        // the launch site chosen for a CAD design: launched there
        W.newGame(Settings{});
        app.gameStarted = true;
        W.recordsOn = false;
        W.settings.launchSite = 4; // (the equatorial platform)
        W.launch(fc);
        Vessel* v = W.activeVessel();
        int earth = W.U.find("Earth");
        Vec3d f0 = norm(W.U.bodyRot(earth, W.t).T() * v->r);
        double lat = std::asin(f0.z) / DEG, lon = std::atan2(f0.y, f0.x) / DEG;
        check(v && std::fabs(lat - LAUNCH_SITES[4].latDeg) < 0.1 && std::fabs(lon - LAUNCH_SITES[4].lonDeg) < 0.1 && v->hasFairing,
              "the CAD design launched from the chosen site with its fairing (" + std::to_string(lat) + ", " + std::to_string(lon) + ")");
        // overlapping panels: the engine selection over the navball, then the navball raised over it
        W.settings.launchSite = 0;
        app.lastLaunched = craftFromDesign(builtinTemplates()[1]); // (side boosters: a long engine list)
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        app.hudHidden = false;
        flightDebugEngines(true, -1);
        sunAt(W, earth, siteDir(LAUNCH_SITES[0].latDeg, LAUNCH_SITES[0].lonDeg), 35);
    }
    if (f == 80) shot("83_ui_engines_front.png");
    if (f == 81) ui::layerRaise(106); // (the navball cluster: L_NAVBALL)
    if (f == 84) shot("83_ui_navball_front.png");
    if (f == 85) {
        flightDebugEngines(false, -1);
        int earth = W.U.find("Earth");
        sunAt(W, earth, siteDir(LAUNCH_SITES[0].latDeg, LAUNCH_SITES[0].lonDeg), -25); // night
        app.hudHidden = true;
        flightDebugView(0.6, 0.15, 80, Vec3d{});
        flightDebugNightMode(0);
        gfx::setPhysicalShading(true); // (the sunlight tint at night: black, and left behind for the factory before)
    }
    if (f == 100) shot("84_ui_night_off.png");
    if (f == 101) flightDebugNightMode(1);
    if (f == 116) shot("84_ui_night_intensifier.png");
    if (f == 117) flightDebugNightMode(2);
    if (f == 132) shot("84_ui_night_bright.png");
    if (f == 133) {
        // back to the factory and the CAD with the night vision still on: their views as usual
        app.hudHidden = false;
        app.design = builtinTemplates()[0];
        app.screen = Screen::Factory;
    }
    if (f == 150) {
        shot("86_ui_factory_after_nv.png");
        // the rocket's white body in the middle of the 3D view: lit by the studio light, not the night's
        Image im = LoadImageFromScreen();
        int rw = GetRenderWidth(), rh = GetRenderHeight();
        if (im.width > rw || im.height > rh) ImageCrop(&im, {0, (float)(im.height - rh), (float)rw, (float)rh}); // (as shot())
        double sum = 0;
        int n = 0;
        for (int y = (int)(im.height * 0.45); y < (int)(im.height * 0.75); y += 4)
            for (int x = (int)(im.width * 0.481); x < (int)(im.width * 0.493); x++) {
                Color c = GetImageColor(im, x, y);
                sum += 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b;
                n++;
            }
        UnloadImage(im);
        double mean = n ? sum / n : 0;
        check(!gfx::physicalShading() || mean > 120, "the factory after a night flight with night vision: the rocket lit (mean " + std::to_string((int)mean) + ")");
    }
    if (f == 151) {
        cadSetCraft(app, craftNamed("Station Truss Segment"));
        app.screen = Screen::CAD;
        cadDebugHome(app);
    }
    if (f == 170) shot("86_ui_cad_after_nv.png");
    if (f == 171) {
        flightDebugNightMode(0);
        app.screen = Screen::Factory;
        factoryDebugSaveAs("My rocket");
    }
    if (f == 190) {
        shot("85_ui_factory_save.png");
        check(factoryDebugSaveAsOpen(), "the factory asks for the name before saving");
    }
    if (f == 191) {
        // a payload with solar wings: the fairing wraps the folded panels, not their deployed span
        Craft c;
        c.name = "Fairing panels test";
        Component tk = makeComponent(Func::Tank);
        tk.dims = {1.5, 1.5, 5.0};
        tk.pos = {0, 0, -2.5};
        c.comps.push_back(tk);
        Component pb = makeComponent(Func::Structure);
        pb.shape = Shape::Box;
        pb.dims = {1.2, 1.2, 2.0};
        pb.pos = {0, 0, 1.0};
        c.comps.push_back(pb);
        for (int side = -1; side <= 1; side += 2) {
            Component sp = makeComponent(Func::Solar);
            sp.shape = Shape::Box;
            sp.dims = {5.0, 0.03, 1.0};
            sp.pos = {side * (0.6 + 2.5), 0, 1.0};
            sp.rotDeg = {0, 0, side < 0 ? 180.0 : 0.0};
            c.comps.push_back(sp);
        }
        cadSetCraft(app, c);
        cadDebugSelect({1, 2, 3});
        bool ok = cadDebugFairing(app);
        double rMax = 0;
        for (const Component& k : app.craft.comps)
            if (k.func == Func::Fairing) rMax = std::max(rMax, std::max(k.dims.x, k.dims.y));
        char b2[160];
        std::snprintf(b2, sizeof b2, "fairing over folded solar wings: radius %.2f m (deployed span 5.6 m)", rMax);
        check(ok && rMax > 0.8 && rMax < 2.0, b2);
        cadDebugSelect({});
    }
    if (f == 192) {
        // debris: a two-stage rocket staged on the pad leaves its spent stage; "Delete debris" takes it, not the
        // vessel flown
        W.newGame(Settings{});
        app.gameStarted = true;
        app.lastLaunched = craftFromDesign(builtinTemplates()[0]);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        Vessel* v = W.activeVessel();
        {
            // two spent pieces near it: a stage without a command part and a fairing half
            for (int k = 0; k < 2; k++) {
                Vessel d = *v;
                d.id = 900000 + k;
                d.name = k ? "Fairing half" : "Spent stage";
                std::vector<Component> one;
                for (const Component& c : d.comps)
                    if (c.func == Func::Tank && one.empty()) one.push_back(c);
                d.comps = one;
                d.prop.clear();
                d.heat.clear();
                d.debris = k == 1;
                d.r = d.r + norm(d.r) * (200.0 + 50 * k);
                d.landed = false;
                setupVessel(d, false);
                W.vessels.push_back(d);
            }
            v = W.activeVessel();
        }
        for (const Vessel& o : W.vessels)
            std::fprintf(T.log, "  vessel %s: destroyed %d debris %d control %d comps %d\n", o.name.c_str(), (int)o.destroyed, (int)o.debris, (int)o.hasControl, (int)o.comps.size());
        int nDeb = 0;
        for (int i = 0; i < (int)W.vessels.size(); i++) nDeb += !W.vessels[i].destroyed && i != W.active && (W.vessels[i].debris || !W.vessels[i].hasControl);
        int act = v ? v->id : -1;
        int n = flightDebugDeleteDebris(W);
        int left = 0;
        for (int i = 0; i < (int)W.vessels.size(); i++) left += !W.vessels[i].destroyed && i != W.active && (W.vessels[i].debris || !W.vessels[i].hasControl);
        check(nDeb > 0 && n == nDeb && left == 0 && W.activeVessel() && W.activeVessel()->id == act,
              "delete debris: " + std::to_string(n) + " of " + std::to_string(nDeb) + " gone, the vessel flown kept");
        // a new name from the vessel menu
        flightDebugRename(W, *W.activeVessel(), "Test Flight 1");
        check(W.activeVessel()->name == "Test Flight 1", "rename: " + W.activeVessel()->name);
    }
    if (f == 194) {
        // recovery: lifted off the pad and down again on Earth - the question; "Recover" removes it, the factory is shown
        W.newGame(Settings{});
        app.gameStarted = true;
        W.recordsOn = true;
        app.lastLaunched = craftFromDesign(builtinTemplates()[2]);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        W.activeVessel()->landed = false; // (in the air: the next flight frame takes note)
    }
    if (f == 197) W.activeVessel()->landed = true; // (down again)
    if (f == 230) {
        Vessel* v = W.activeVessel();
        if (v) std::fprintf(T.log, "  recover: landed %d rolling %d ref %d alt %.2f\n", (int)v->landed, (int)v->rolling, v->ref, len(v->r) - W.U.bodies[v->ref].radius);
        check(v && flightDebugRecoverAsk() == v->id, "back on Earth after a hop: the recovery question");
        shot("87_ui_recover.png");
    }
    if (f == 231) flightDebugRecover();
    if (f == 234) check(app.screen == Screen::Factory && (!W.activeVessel() || W.activeVessel()->name != app.lastLaunched.name),
                        "recovered: the vessel removed, the factory shown");
    if (f == 235) {
        std::fprintf(T.log, "uitest: %s (%d failed)\n", bad ? "FAIL" : "all ok", bad);
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ orbit lines on the map (--orbitlinetest)
// a short suborbital arc (a few degrees of the orbit above the ground), a low orbit seen up close and the
// overview: the lines must stay smooth at any zoom
bool orbitLineTestStep(App& app) {
    World& W = app.world;
    const int f = T.frames;
    int earth = W.U.find("Earth");
    const Body& eb = W.U.bodies[earth];
    auto setState = [&](double alt, double vh, double vv) {
        Vessel* v = W.activeVessel();
        Vec3d up = W.U.bodyRot(earth, W.t) * siteDir(20, 40);
        Vec3d east = norm(cross(Vec3d{0, 0, 1}, up));
        v->ref = earth;
        v->landed = false;
        v->r = up * (eb.radius + alt);
        v->v = east * vh + up * vv;
        v->q = Quatd::lookRot(norm(v->v), up);
        v->w = {};
        v->throttle = 0;
        W.warpIndex = 0;
    };
    if (f == 1) {
        W.newGame(Settings{});
        app.gameStarted = true;
        sunAt(W, earth, siteDir(20, 40), 40);
        app.lastLaunched = craftFromDesign(builtinTemplates()[0]);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        setState(40e3, 1900, 1400);
        flightDebugSetMap(true, 1.6e6);
    }
    if (f >= 1 && f < 20) setState(40e3, 1900, 1400);
    if (f == 20) shot("70_orbit_suborbital.png");
    if (f == 21) { setState(400e3, std::sqrt(eb.GM / (eb.radius + 400e3)), 0); flightDebugSetMap(true, 2.0e5); }
    if (f == 40) shot("71_orbit_leo_close.png");
    if (f == 41) { setState(300e3, std::sqrt(eb.GM / (eb.radius + 300e3)) * 1.25, 0); flightDebugSetMap(true, 6e7); }
    if (f == 60) shot("72_orbit_overview.png");
    if (f == 61) { setState(200e3, 7600, 0); flightDebugSetMap(true, 4e5); }
    if (f == 80) shot("73_orbit_pe_close.png");
    if (f == 81) {
        flightDebugSetMap(false, 0);
        std::fprintf(T.log, "orbit lines: pictures taken -> ok\n");
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ the physically based sky (--skytest)
// high-altitude and twilight views toward the horizon (where a coarse integral shows steps and flat bands)
bool skyTestStep(App& app) {
    World& W = app.world;
    struct View { double altKm, sunEl, yaw, pitch; };
    static const View V[] = {{20, -3, 0.0, -0.15}, {20, -3, 3.1, -0.15}, {30, 2, 1.6, -0.2}, {35, -6, 0.0, -0.12},
                             {50, 0, 3.1, -0.1}, {15, 5, 0.0, -0.25}, {90, 3, 3.1, 0.05}, {2, -5, 0.0, -0.1}};
    const int N = (int)(sizeof V / sizeof V[0]), per = 25;
    int vi = (T.frames - 1) / per, k = (T.frames - 1) % per;
    if (T.frames == 1) {
        if (!gfx::physicalShading()) gfx::setPhysicalShading(true);
        W.newGame(Settings{});
        app.gameStarted = true;
        app.design = builtinTemplates()[2];
        app.lastLaunched = craftFromDesign(app.design);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
    }
    if (vi >= N) return false;
    Vessel* v = W.activeVessel();
    if (!v) return false;
    const View& s = V[vi];
    int earth = W.U.find("Earth");
    Vec3d f = siteDir(10, 60);
    if (k == 0) {
        sunAt(W, earth, f, s.sunEl);
        flightDebugView(s.yaw, s.pitch, 60, Vec3d{});
    }
    placeAt(W, *v, earth, f, s.altKm * 1000);
    if (k == per - 1) {
        char name[64];
        std::snprintf(name, sizeof name, "50_sky_%d_%dkm_sun%+d.png", vi, (int)s.altKm, (int)s.sunEl);
        shot(name);
    }
    return true;
}

// ------------------------------------------------------------------ aircraft (--airtest)
Craft craftNamed(const char* name);
bool dockTestStep(App& app);
bool spaceplaneTestStep(App& app);
// hover margin from momentum theory, then real flights: helicopters on Earth, Mars and Titan climb to
// 60 m and land again; the jet trainer takes off from the runway and climbs
Craft craftNamed(const char* name) {
    for (const Craft& c : builtinCrafts())
        if (c.name == name) return c;
    return builtinCrafts()[0];
}
bool airTestStep(App& app) {
    World& W = app.world;
    struct Flight { const char* craft; const char* body; bool plane; };
    static const Flight F[] = {{"Kestrel Helicopter", "Earth", false}, {"Mars Helicopter", "Mars", false},
                               {"Titan Rotorcraft", "Titan", false}, {"Swift Jet Trainer", "Earth", true}};
    if (T.frames == 2 && T.tf < 0) {
        // ---- analytic hover check: full-power thrust at the landing site against the weight
        for (const Flight& f : F) {
            if (f.plane) continue;
            Craft c = craftNamed(f.craft);
            int body = W.U.find(f.body);
            const Body& b = W.U.bodies[body];
            auto comps = expandCraft(c);
            double mass = 0, T0 = 0, P = 0;
            Vessel tmp;
            tmp.comps = comps;
            setupVessel(tmp, false);
            mass = tmp.mp.mass;
            double alt = f.body == std::string("Mars") ? -2600 : 0; // Jezero lies 2.6 km below the datum
            double rho = W.U.density(body, std::max(alt, 0.0)) * (alt < 0 ? std::exp(-alt / std::max(b.atmH, 1.0)) : 1.0);
            for (size_t i = 0; i < comps.size(); i++) {
                if (comps[i].func != Func::Rotor) continue;
                const PartDef& d = CATALOG.defs[comps[i].def];
                double A = PI * 0.25 * d.diam * d.diam * (d.canopies > 1 ? 1.25 : 1.0);
                double vtip = std::min(230.0, 0.8 * b.soundSpeed);
                T0 += rotorThrust(d.powerUse * 0.92, rho, A, 0, 0.016 * rho * PI * 0.25 * d.diam * d.diam * vtip * vtip * (d.canopies > 1 ? 1.6 : 1.0));
                P += d.powerUse;
            }
            double g = surfaceGravity(b), Wt = mass * g;
            std::fprintf(T.log, "air: %-20s on %-6s mass %.1f kg, air %.4f kg/m3, g %.2f: full-power thrust %.0f N vs weight %.0f N (x%.2f), shaft %.0f W -> %s\n",
                         f.craft, f.body, mass, rho, g, T0, Wt, T0 / std::max(Wt, 1e-9), P, T0 > Wt * 1.15 ? "ok" : "FAIL");
        }
        // the turbofan breathes oxygen: it gives thrust on Earth, none on Mars
        std::fprintf(T.log, "air: turbofan at sea level %.1f kN, at 10 km %.1f kN, Mach 0.8 at 10 km %.1f kN\n", jetThrust(27e3, 1.225, 0) / 1000,
                     jetThrust(27e3, 0.414, 0) / 1000, jetThrust(27e3, 0.414, 0.8) / 1000);
        T.tf = 0;
        T.frames = 0;
        return true;
    }
    if (T.tf < 0) return true;
    if (T.tf >= 4) {
        // the user's path: the jet trainer in the CAD designer with the "Fly on" selector beside LAUNCH
        if (T.frames == 2) {
            cadSetCraft(app, craftNamed("Swift Jet Trainer"));
            cadDebugPalette(4); // the Aircraft tab of the parts palette
            app.screen = Screen::CAD;
        }
        if (T.frames == 12) {
            shot("25_air_cad.png");
            return false;
        }
        return true;
    }
    const Flight& f = F[T.tf];
    if (T.frames == 2) {
        W.newGame(Settings{});
        app.gameStarted = true;
        Craft c = craftNamed(f.craft);
        app.lastLaunched = c;
        int body = W.U.find(f.body);
        {
            // daylight at the start site (the shots show the ground)
            const LaunchSite& site = LAUNCH_SITES[std::clamp(W.settings.launchSite, 0, LAUNCH_SITE_COUNT - 1)];
            double la = f.body == std::string("Mars") ? 18.4 : f.body == std::string("Titan") ? -10.3 : site.latDeg;
            double lo = f.body == std::string("Mars") ? 77.5 : f.body == std::string("Titan") ? 167.6 : site.lonDeg;
            sunAt(W, body, siteDir(la, lo), 35);
        }
        W.launchAircraft(c, body);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        Vessel* v = W.activeVessel();
        v->throttle = f.plane ? 1.0 : 0.0; // (the motors of an aircraft are armed at the start)
        T.tfSub = 0;
        T.tfRef = W.t;
        T.tfMass0 = len(v->r);
        T.tfTurn = 0;
        flightDebugView(f.plane ? 2.6 : 0.6, 0.25, f.plane ? 28 : (f.body == std::string("Mars") ? 2.5 : 22), Vec3d{});
        std::fprintf(T.log, "air: %s on %s: start %s, mass %.1f kg, aircraft %d, wheels %d\n", f.craft, f.body, v->landed ? "landed" : "airborne",
                     W.vesselMass(*v), (int)v->isAircraft, (int)v->hasWheels);
        return true;
    }
    if (T.frames < 4) return true;
    Vessel* v = W.activeVessel();
    auto next = [&](const char* verdict) {
        std::fprintf(T.log, "air: %s on %s: %s\n", f.craft, f.body, verdict);
        char name[64];
        std::snprintf(name, sizeof name, "25_air_%d_end.png", T.tf);
        shot(name);
        flightDebugInput(nullptr);
        T.tf++;
        T.frames = 0;
        return true;
    };
    if (!v) return next("FAIL (no vessel)");
    if (v->destroyed) return next((std::string("FAIL (") + v->fate + ")").c_str());
    const Body& b = W.U.bodies[v->ref];
    Vec3d up = norm(v->r);
    Mat3d RT = W.U.bodyRot(v->ref, W.t).T();
    double agl = len(v->r) - b.radius - terrain::height(v->ref, RT * v->r);
    Vec3d vs = W.surfaceVel(*v);
    if (!f.plane) {
        // helicopter pilot: climb at 3 m/s to 60 m, hover, descend at 1 m/s and land
        double vzT = T.tfSub == 0 ? (agl < 60 ? 3.0 : 0.0) : -1.0;
        for (int k = 0; k < 8 && !v->destroyed; k++) {
            double vzNow = dot(W.surfaceVel(*v), norm(v->r));
            v->throttle = std::clamp(v->throttle + (vzT - vzNow) * 0.012, 0.0, 1.0);
            if (v->landed && T.tfSub == 1) break;
            W.update(0.05);
        }
        if (T.tfSub == 0 && agl > 59) {
            T.tfSub = 1;
            T.tfTurn = v->throttle;
            std::fprintf(T.log, "air: %s on %s: hovering at %.0f m above the ground, throttle %.2f (%.0f W), battery %.0f%%, %.0f s after the start\n", f.craft,
                         f.body, agl, v->throttle, v->tPowerUse, 100 * v->charge / std::max(v->chargeCap, 1e-9), W.t - T.tfRef);
            char name[64];
            std::snprintf(name, sizeof name, "25_air_%d_hover.png", T.tf);
            shot(name);
        }
        if (T.tfSub == 1 && v->landed) {
            v->throttle = 0;
            return next("climbed to 60 m and landed -> ok");
        }
        if (W.t - T.tfRef > 400) return next("FAIL (no climb / landing within 400 s)");
        return true;
    }
    // jet pilot: full throttle down the runway, rotate at 62 m/s, climb at 8 degrees to 600 m
    double sp = len(vs);
    Vec3d nose = v->q.rotate({0, 0, 1});
    double pitch = std::asin(std::clamp(dot(nose, up), -1.0, 1.0)) / DEG;
    ControlInput in;
    double want = sp > 62 ? (agl < 600 ? 8.0 : 2.0) : 0.0;
    if (T.tfSub == 2) {
        // approach: about 75 m/s, sinking 4 m/s, flare below 15 m to 1 m/s
        double vz = dot(vs, up), vzT = agl > 30 ? -3.5 : -0.8;
        want = std::clamp(3.0 + (vzT - vz) * 1.5, -6.0, 12.0);
        v->throttle = std::clamp(0.45 + (75 - sp) * 0.04, 0.05, 1.0);
    }
    if (T.tfSub == 3) {
        want = pitch; // roll on the wheels, throttle closed (brakes)
        v->throttle = 0;
    }
    // the stick moves every physics step, as a pilot's hand would (a command held for 0.4 s made the pitch
    // oscillate): nose up = negative pitch, damped by the pitch rate (w.x > 0: nose going down); wings level
    v->sas = SAS::Off;
    for (int k = 0; k < 8 && !v->destroyed; k++) {
        nose = v->q.rotate({0, 0, 1});
        up = norm(v->r);
        double pk = std::asin(std::clamp(dot(nose, up), -1.0, 1.0)) / DEG;
        double bank = std::asin(std::clamp(dot(v->q.rotate({1, 0, 0}), up), -1.0, 1.0)) / DEG;
        if (T.tfSub == 3) want = pk;
        in.pitch = std::clamp((pk - want) * 0.08 - v->w.x * 1.2, -1.0, 1.0);
        in.roll = std::clamp(-bank * 0.05 - v->w.z * 0.5, -1.0, 1.0) * (v->rolling ? 0.0 : 1.0);
        flightDebugInput(&in);
        W.input = in;
        W.update(0.05);
    }
    if (T.frames % 4 == 0 && agl < 700 && (T.tfSub <= 1 || agl < 60))
        std::fprintf(T.log, "  jet: t %5.1f speed %5.1f agl %6.1f pitch %5.1f want %4.1f in %5.2f surf %5.2f %5.2f %5.2f vz %5.1f rolling %d wx %.3f ctrl %.2f bank %.1f\n", W.t - T.tfRef, sp, agl,
                     pitch, want, in.pitch, v->surf.x, v->surf.y, v->surf.z, dot(vs, up), (int)v->rolling, v->w.x, W.ctrl.pitch,
                     std::asin(std::clamp(dot(v->q.rotate({1, 0, 0}), up), -1.0, 1.0)) / DEG);
    if (T.tfSub == 0 && !v->rolling && !v->landed && agl > 3) {
        T.tfSub = 1;
        std::fprintf(T.log, "air: %s: lift-off at %.1f m/s after %.0f m of runway, %.0f s\n", f.craft, sp, len(v->r - up * dot(v->r, up)) * 0 +
                     std::acos(std::clamp(dot(norm(v->r), norm(W.U.bodyRot(v->ref, W.t) * norm(v->landPos))), -1.0, 1.0)) * b.radius, W.t - T.tfRef);
        shot("25_air_3_liftoff.png");
    }
    if (T.tfSub == 0 && W.t - T.tfRef > 90) return next("FAIL (no lift-off within 90 s)");
    if (T.tfSub == 1 && agl > 500) {
        std::fprintf(T.log, "air: %s: climbing through %.0f m at %.0f m/s, pitch %.1f deg, fuel %.0f kg -> ok\n", f.craft, agl, sp, pitch,
                     propOnBoard(*v));
        T.tfSub = 2; // now down again: approach, flare, touchdown, roll-out, stop
        T.tfRef = W.t;
    }
    if (T.tfSub == 1 && W.t - T.tfRef > 400) return next("FAIL (no climb to 500 m)");
    if (T.tfSub == 2 && v->rolling) {
        std::fprintf(T.log, "air: %s: touchdown at %.1f m/s, sink %.1f m/s, pitch %.1f deg\n", f.craft, sp, -dot(vs, up), pitch);
        T.tfSub = 3;
    }
    if (T.tfSub == 3 && v->landed) {
        std::fprintf(T.log, "air: %s: stopped on the ground, parked -> ok\n", f.craft);
        return next("took off, climbed, landed and stopped -> ok");
    }
    if (T.tfSub >= 2 && W.t - T.tfRef > 600) return next("FAIL (no landing within 600 s)");
    return true;
}

// ------------------------------------------------------------------ manual attitude control feel (--ctrltest)
// a key held for 1 s on pitch (and roll), then released with SAS "hold attitude": the rate reached, the
// angle turned, the time to stop. Rockets in orbit and the jet trainer in level flight at 3 km.
// ------------------------------------------------------------------ spaceplanes (--spaceplanetest)
// 0: the air-breathing engines over their envelopes; 1: the Shuttle-type stack to orbit (boosters and
// external tank dropped), de-orbit and entry at 40 degrees angle of attack; 2: the orbiter's glide to the
// runway and landing on its gear (flown with the stick: rate command, elevons, rudder); 3: the SSTO from
// the runway: air-breathing climb to Mach 5, rocket mode, orbit. --spsub n runs only part n
int g_spOnly = -1;
double g_spAltKm = 300;
static void holdAttitude(Vessel& v, const Vec3d& nose, const Vec3d& upHint) {
    v.q = Quatd::lookRot(nose, upHint);
    v.w = {};
}
static void logDebris(World& W, const char* when) {
    const Vessel* a = W.activeVessel();
    for (const Vessel& d : W.vessels) {
        if (!d.debris || !a || d.ref != a->ref) continue;
        const Body& b = W.U.bodies[d.ref];
        std::fprintf(T.log, "  debris (%s): %s, %d parts, %.1f t, %.0f m from the vessel, alt %.1f km, %s\n", when, d.name.c_str(), (int)d.comps.size(),
                     W.vesselMass(d) / 1000, len(d.r - a->r), (len(d.r) - b.radius) / 1000, d.destroyed ? d.fate.c_str() : "flying");
    }
}
bool spaceplaneTestStep(App& app) {
    World& W = app.world;
    static double peakHeat = 0, peakG = 0, peakFlux = 0, mark = 0, vMax = 0;
    static int sub = 0;
    auto finish = [&](const char* verdict) {
        std::fprintf(T.log, "spaceplane %d: %s\n", T.tf, verdict);
        char name[64];
        std::snprintf(name, sizeof name, "27_sp_%d_end.png", T.tf);
        shot(name);
        flightDebugInput(nullptr);
        W.warpIndex = 0;
        T.tf = g_spOnly >= 0 ? 99 : T.tf + 1;
        T.frames = 0;
        return T.tf <= 5;
    };
    if (T.tf < 0) T.tf = g_spOnly >= 0 ? g_spOnly : 0;
    if (T.tf > 5) return false;
    if (T.tf == 0) {
        // ---- the engines: thrust and specific impulse over their envelopes (Earth's air)
        struct Pt { const char* what; double alt, mach; };
        static const Pt P[] = {{"sea level, static", 0, 0}, {"11 km, Mach 0.9", 11e3, 0.9}, {"11 km, Mach 2.0", 11e3, 2.0},
                               {"24 km, Mach 3.2", 24e3, 3.2}, {"30 km, Mach 6", 30e3, 6.0}, {"26 km, Mach 5.0", 26e3, 5.0},
                               {"100 km, Mach 20", 100e3, 20}};
        int earth = W.U.find("Earth");
        const Body& b = W.U.bodies[earth];
        bool ok = true;
        double F[8][8] = {};
        const char* ids[] = {"jet_tf27", "jet_tf131ab", "jet_trj151", "jet_sj60", "jet_pce2000"};
        for (int e = 0; e < 5; e++) {
            const PartDef& d = CATALOG.defs[CATALOG.find(ids[e])];
            for (int k = 0; k < 7; k++) {
                double rho = P[k].alt < b.atmTop ? W.U.density(earth, P[k].alt) : 0.0;
                double a = b.soundSpeed, v = P[k].mach * a, q = 0.5 * rho * v * v;
                double patm = rho / 1.225;
                double isp;
                int mode;
                F[e][k] = jetPerformance(d, rho, P[k].mach, q, patm, isp, mode);
                std::fprintf(T.log, "spaceplane 0: %-34s %-18s thrust %8.1f kN  Isp %6.0f s  mode %d\n", tr(d.name).c_str(), P[k].what, F[e][k] / 1000, isp, mode);
            }
        }
        // expectations: the afterburner adds thrust; the turbofans give nothing at 100 km; the turboramjet works
        // at Mach 3.2; the scramjet needs Mach 4+ and works at Mach 6; the combined cycle works at 100 km
        ok &= F[1][0] > 120e3 && F[1][2] > F[1][1] && F[0][6] == 0 && F[1][6] == 0 && F[2][3] > 40e3 && F[3][0] == 0 && F[3][2] == 0 && F[3][4] > 20e3 &&
              F[4][6] > 1.9e6 && F[4][5] > 0.4e6;
        // and the rocket-plane engine is a rocket (works in the vacuum)
        const PartDef& rp = CATALOG.defs[CATALOG.find("eng_rp257")];
        ok &= rp.kind == PK::Engine && worksWithoutAir(rp);
        return finish(ok ? "engine envelopes -> ok" : "engine envelopes -> FAIL");
    }
    Vessel* v = W.activeVessel();
    if (T.frames == 2) {
        W.newGame(Settings{});
        app.gameStarted = true;
        peakHeat = peakG = peakFlux = vMax = 0;
        T.shotEntry = false;
        sub = 0;
        mark = 0;
        Craft c = craftNamed(T.tf == 3 ? "Skylark SSTO Spaceplane" : "Albatross Space Shuttle");
        if (const char* sv = std::getenv("SF_SPSTRIP")) {
            // (diagnostics: the shuttle without some of its new parts) 1 drag chute, 2 windows, 4 bay, 8 cargo flags
            int m = std::atoi(sv);
            std::vector<Component> kept;
            for (Component k : c.comps) {
                if ((m & 1) && k.func == Func::Parachute) continue;
                if ((m & 2) && k.func == Func::Window) continue;
                if ((m & 4) && k.func == Func::CargoBay) continue;
                if (m & 8) k.cargo = 0;
                kept.push_back(k);
            }
            c.comps = kept;
            std::fprintf(T.log, "spaceplane: stripped %d\n", m);
        }
        if (T.tf == 2 || T.tf >= 4) {
            // the orbiter alone (stage 2 of the stack)
            Craft o = c;
            o.comps.clear();
            for (const Component& k : c.comps)
                if (k.stage == 2) {
                    Component k2 = k;
                    k2.stage = 0;
                    k2.fireFrom = -1;
                    o.comps.push_back(k2);
                }
            o.name = "Albatross Orbiter";
            if (T.tf == 5) // the control case: no thermal protection - an aluminium skin
                for (Component& k : o.comps)
                    if (k.material == 7) k.material = 0;
            c = o;
        }
        app.lastLaunched = c;
        int earth = W.U.find("Earth");
        const LaunchSite& site = LAUNCH_SITES[std::clamp(W.settings.launchSite, 0, LAUNCH_SITE_COUNT - 1)];
        sunAt(W, earth, siteDir(site.latDeg, site.lonDeg), 35);
        if (T.tf == 1) W.launch(c);
        else W.launchAircraft(c, earth);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        v = W.activeVessel();
        T.tfRef = W.t;
        T.tfSub = 0;
        std::fprintf(T.log, "spaceplane %d: %s: mass %.1f t, aircraft %d, wheels %d, tailless %d, ctrlAuth %.0f / %.0f / %.0f N m/Pa, centre of mass z %.2f m\n", T.tf,
                     c.name.c_str(), W.vesselMass(*v) / 1000, (int)v->isAircraft, (int)v->hasWheels, (int)v->tailless, v->ctrlAuth.x, v->ctrlAuth.y,
                     v->ctrlAuth.z, v->mp.com.z);
        if (T.tf >= 4) {
            // in a 300 km orbit, ready for the de-orbit burn
            W.placeInOrbit(*v, g_spAltKm * 1000); // (--spalt km: another orbit; 206 = a low, nearly circular one: the shallowest entry)
            v->sas = SAS::Stability;
            T.tfSub = 2;
            flightDebugView(2.0, 0.15, 90, Vec3d{});
        }
        if (T.tf == 2) {
            // the approach: 8 km before the runway, 1,600 m up, 150 m/s along the runway, sinking at 12 degrees
            const Body& b = W.U.bodies[v->ref];
            Mat3d Rb = W.U.bodyRot(v->ref, W.t);
            Vec3d upI = norm(Rb * v->landPos), fwdI = norm(Rb * v->landQ.rotate({0, 0, 1}));
            fwdI = norm(fwdI - upI * dot(fwdI, upI));
            double ground = len(v->landPos);
            v->landed = false;
            v->rolling = false;
            v->r = upI * (ground + 1600) - fwdI * 8000;
            Vec3d up2 = norm(v->r);
            Vec3d f2 = norm(fwdI - up2 * dot(fwdI, up2));
            double gam = -12 * DEG;
            Vec3d vAir = (f2 * std::cos(gam) + up2 * std::sin(gam)) * 150.0;
            v->v = vAir + cross(W.U.angVel(v->ref), v->r);
            holdAttitude(*v, f2 * std::cos(gam + 7 * DEG) + up2 * std::sin(gam + 7 * DEG), up2);
            v->legsDeployed = true;
            v->sas = SAS::Stability;
            v->throttle = 0;
            (void)b;
            flightDebugView(2.6, 0.2, 90, Vec3d{});
        } else if (T.tf == 3) {
            v->throttle = 1;
            v->sas = SAS::Stability;
            flightDebugView(2.4, 0.2, 140, Vec3d{});
        } else flightDebugView(2.0, 0.12, 80, Vec3d{});
        return true;
    }
    if (T.frames < 4) return true;
    if (!v) return finish("FAIL (no vessel)");
    if (v->destroyed) {
        // the control case burns up as it should; seen here when the last step of a frame was the one (under load)
        if (T.tf == 5 && v->fate.rfind(tr("Burned up during atmospheric entry"), 0) == 0)
            return finish((std::string("without tiles: ") + v->fate + " -> ok (the thermal protection is needed)").c_str());
        return finish((std::string("FAIL (") + v->fate + ")").c_str());
    }
    const Body& b = W.U.bodies[v->ref];
    Conic c = stateToConic(v->r, v->v, b.GM);
    double alt = len(v->r) - b.radius;
    Vec3d up = norm(v->r);
    Vec3d vAir = W.surfaceVel(*v);
    double sp = len(vAir);
    peakHeat = std::max(peakHeat, v->tHeatMax);
    peakFlux = std::max(peakFlux, v->tHeatFlux);
    peakG = std::max(peakG, std::max(v->tAccel, std::hypot(v->tDragAcc, v->tLiftAcc)) / G0); // (thrust or air)
    vMax = std::max(vMax, sp);
    auto status = [&](const char* tag) {
        std::fprintf(T.log, "spaceplane %d %s: MET %s alt %.1f km speed %.0f m/s Mach %.2f q %.1f kPa ap %.0f km pe %.0f km mass %.1f t stage %d heat %.2f mode %d\n", T.tf, tag,
                     formatTime(W.t - v->launchTime).c_str(), alt / 1000, sp, v->tMach, v->tDynP / 1000, (c.ap - b.radius) / 1000, (c.pe - b.radius) / 1000,
                     W.vesselMass(*v) / 1000, v->stage, v->tHeatMax, v->ccMode);
        std::fflush(T.log);
    };
    if (W.t - T.tfRef > 3 * 3600) return finish("FAIL (time out)");
    if (T.tf == 1 || T.tf >= 4) {
        // ---------------- Shuttle: ascent guidance with the boosters and the external tank
        if (T.tfSub == 0) {
            v->sas = SAS::Ascent;
            v->throttle = 1;
            if (!v->ignited) W.stage(*v);
            T.tfSub = 1;
            status("lift-off");
            shot("27_sp_1_liftoff.png");
            return true;
        }
        if (T.tfSub == 1) {
            if (v->ascentPhase == 1 && alt > b.atmTop) W.warpIndex = std::min(5, W.maxWarpIndex());
            else if (W.warpIndex > 0) W.warpIndex = 0;
            for (int k = 0; k < 8 && !v->destroyed && v->sas == SAS::Ascent; k++) {
                W.update(0.05);
                // the boosters burned out: drop them (the main engines burn on from the external tank)
                bool srbs = false, srbLeft = false;
                for (size_t i = 0; i < v->comps.size(); i++)
                    if (v->comps[i].stage == v->stage && v->comps[i].def >= 0 && CATALOG.defs[v->comps[i].def].kind == PK::SRB) {
                        srbs = true;
                        if (v->prop[i] > 0) srbLeft = true;
                    }
                if (srbs && !srbLeft) {
                    status("boosters burned out");
                    W.stage(*v);
                    v = W.activeVessel();
                    status("boosters separated");
                    logDebris(W, "at the separation");
                    shot("27_sp_1_srbsep.png");
                }
                if (v->tFlameout && v->throttle > 0) {
                    status("flameout");
                    break;
                }
            }
            if (T.frames % 50 == 0) {
                Vec3d tgt = W.targetDirection(*v, SAS::Ascent);
                double err = len(tgt) > 0.5 ? std::acos(std::clamp(dot(norm(tgt), v->q.rotate({0, 0, 1})), -1.0, 1.0)) / DEG : -1;
                std::fprintf(T.log, "  att: error %.1f deg, w %.2f deg/s, target elevation %.1f deg, nose elevation %.1f deg, thrust %.0f kN\n", err,
                             len(v->w) / DEG, std::asin(std::clamp(dot(norm(tgt), up), -1.0, 1.0)) / DEG,
                             std::asin(std::clamp(dot(v->q.rotate({0, 0, 1}), up), -1.0, 1.0)) / DEG, v->tThrust / 1000);
                status("ascent");
            }
            if (v->destroyed) return finish((std::string("FAIL (") + v->fate + ")").c_str());
            if (v->sas != SAS::Ascent || v->tFlameout) {
                c = stateToConic(v->r, v->v, b.GM);
                status("guidance done");
                if (!(c.valid && c.e < 1 && c.pe - b.radius > 120e3)) {
                    // main-engine cut-off just below orbit (as on the real Shuttle): drop the tank, the OMS
                    // completes the insertion at the apoapsis
                    if (!(c.valid && c.e < 1 && c.ap - b.radius > 80e3)) return finish("FAIL (no orbit)");
                    v->throttle = 0;
                    W.stage(*v);
                    v = W.activeVessel();
                    std::fprintf(T.log, "spaceplane 1: cut-off below orbit, tank dropped; OMS delta-v %.0f m/s for the insertion\n", stageDv(W, *v));
                    T.tfSub = 5;
                    return true;
                }
                // drop the external tank: the orbiter alone (with its OMS)
                v->throttle = 0;
                W.stage(*v);
                v = W.activeVessel();
                status("external tank separated - in orbit");
                logDebris(W, "in orbit");
                std::fprintf(T.log, "spaceplane 1: orbiter in orbit: %s, OMS delta-v %.0f m/s\n", v->name.c_str(), stageDv(W, *v));
                shot("27_sp_1_orbit.png");
                T.tfSub = 2;
            }
            return true;
        }
        if (T.tfSub == 5) {
            // OMS insertion: coast to the apoapsis, then prograde until the periapsis is above 110 km
            double tAp = c.valid && c.e < 1 ? timeToTrueAnomaly(c, PI) : 0;
            holdAttitude(*v, norm(v->v), up);
            if (tAp > 30 && v->throttle <= 0) {
                for (int k = 0; k < 8; k++) W.update(0.05);
                return true;
            }
            v->throttle = 1;
            if (!v->ignited) W.stage(*v);
            for (int k = 0; k < 8 && !v->destroyed; k++) {
                holdAttitude(*v, norm(v->v), norm(v->r));
                W.update(0.05);
                c = stateToConic(v->r, v->v, b.GM);
                if (c.valid && c.pe - b.radius > 110e3) break;
            }
            if (c.valid && c.pe - b.radius > 110e3) {
                v->throttle = 0;
                status("in orbit after the OMS insertion");
                std::fprintf(T.log, "spaceplane 1: orbiter in orbit, OMS delta-v left %.0f m/s\n", stageDv(W, *v));
                shot("27_sp_1_orbit.png");
                T.tfSub = 2;
            }
            if (v->tFlameout) return finish("FAIL (OMS propellant out during the insertion)");
            return true;
        }
        if (T.tfSub == 2) {
            // de-orbit: at the apoapsis (where it lowers the periapsis cheapest), retrograde with the OMS until
            // the periapsis is 50 km
            holdAttitude(*v, -norm(v->v), up);
            double tAp = c.valid && c.e < 1 ? timeToTrueAnomaly(c, PI) : 0;
            if (v->throttle <= 0 && c.e > 0.003 && tAp > 40 && tAp < c.period - 40) { // (a circular orbit: any point)
                W.warpIndex = tAp > 300 ? std::min(5, W.maxWarpIndex()) : 0;
                for (int k = 0; k < 8; k++) W.update(0.05);
                return true;
            }
            if (W.warpIndex > 0) W.warpIndex = 0;
            v->throttle = 1;
            if (!v->ignited) W.stage(*v);
            for (int k = 0; k < 8; k++) {
                W.update(0.05);
                holdAttitude(*v, -norm(v->v), norm(v->r));
                c = stateToConic(v->r, v->v, b.GM);
                if (c.pe - b.radius < 50e3) break;
            }
            if (c.pe - b.radius < 50e3) {
                v->throttle = 0;
                status("de-orbit burn done");
                if (std::getenv("SF_SPDIAG")) {
                    // (diagnostics: the centre of mass against the orbiter's own parts, and what each part holds)
                    int ref = -1;
                    for (int i = 0; i < (int)v->comps.size(); i++)
                        if (v->comps[i].func == Func::Wing && ref < 0) ref = i;
                    Vec3d d = ref >= 0 ? v->mp.com - v->phys[ref].centre : Vec3d{};
                    std::fprintf(T.log, "  diag: com - wing0 centre %.3f %.3f %.3f, comps %d, bayComp %d, flapStage %d, gear %d, bayOpen %d\n", d.x, d.y, d.z,
                                 (int)v->comps.size(), v->bayComp, v->flapStage, (int)v->legsDeployed, (int)v->bayOpen);
                    std::fprintf(T.log, "  diag: flapPos %.3f bayT %.3f legsT %.3f chute %d/%d solar %d/%.2f surf %.2f %.2f %.2f shown %d stowed %d\n", v->flapPos, v->bayT,
                                 v->legsT, (int)v->chuteDeployed, (int)v->chuteCut, (int)v->solarDeployed, v->solarT, v->surf.x, v->surf.y, v->surf.z,
                                 (int)v->shown.size(), (int)v->stowedGeom.size());
                    {
                        const std::vector<Component>& G = v->geometry();
                        for (size_t i = 0; i < G.size(); i++) {
                            const Component& k = G[i];
                            Vec3d pr = k.pos - v->comps[ref].pos;
                            std::fprintf(T.log, "  geo: %-28s f%d s%d dims %.3f %.3f %.3f rot %.2f %.2f %.2f pos %.3f %.3f %.3f fl %.1f sw %.1f tp %.2f\n", k.name.c_str(),
                                         (int)k.func, (int)k.shape, k.dims.x, k.dims.y, k.dims.z, k.rotDeg.x, k.rotDeg.y, k.rotDeg.z, pr.x, pr.y, pr.z, k.flap,
                                         k.sweep, k.taper);
                        }
                    }
                    for (int i = 0; i < (int)v->comps.size(); i++)
                        if (v->prop[i] > 0 || v->comps[i].cargo)
                            std::fprintf(T.log, "  diag: %2d %-28s stage %d cargo %d prop %.0f kg at %.2f %.2f %.2f\n", i, v->comps[i].name.c_str(), v->comps[i].stage,
                                         (int)v->comps[i].cargo, v->prop[i], v->phys[i].centre.x - (ref >= 0 ? v->phys[ref].centre.x : 0),
                                         v->phys[i].centre.y - (ref >= 0 ? v->phys[ref].centre.y : 0), v->phys[i].centre.z - (ref >= 0 ? v->phys[ref].centre.z : 0));
                }
                peakHeat = peakFlux = peakG = 0; // (the entry's own peaks, not the ascent's)
                T.tfSub = 3;
            }
            if (v->tFlameout) return finish("FAIL (OMS propellant out before the de-orbit)");
            return true;
        }
        // entry, flown with the stick as the Shuttle's autopilot does (rate command: the RCS, then the elevons and
        // the rudder as the air thickens): belly first at 40 degrees angle of attack down to ~4 km/s, then less;
        // the lift vector banked so that the deceleration follows ~1.3 g (S-turns of the real orbiter)
        double aoaDeg = sp > 4000 ? 40.0 : sp > 900 ? 15.0 + 25.0 * (sp - 900) / 3100 : 15.0;
        double bankCmd = 0, D = 0;
        auto pilot = [&](double h) {
            Vec3d va = W.surfaceVel(*v), u = norm(v->r);
            double s2 = len(va);
            Vec3d d = norm(va), n = norm(u - d * dot(u, d)), e = cross(d, n);
            double aoa = aoaDeg * DEG;
            // drag from the measured acceleration (L/D of the attitude), the lift needed for a level path
            D = v->tDragAcc;
            double Lacc = v->tLiftAcc;
            double r = len(v->r), gl = b.GM / (r * r);
            Vec3d vin = v->v;
            double vh = len(vin - u * dot(vin, u));
            double need = gl - vh * vh / r; // (negative above orbital speed)
            double DT = 1.3 * G0, hdot = dot(va, u);
            double hdotT = clampd(-60.0 * (1.0 - D / DT), -150.0, 40.0);
            double cb = Lacc > 0.3 ? clampd((need + 0.05 * (hdotT - hdot)) / Lacc, -0.2, 1.0) : 1.0;
            bankCmd = std::acos(cb);
            Vec3d lift = n * std::cos(bankCmd) + e * std::sin(bankCmd);
            Vec3d noseT = d * std::cos(aoa) + lift * std::sin(aoa);
            Vec3d upT = lift * std::cos(aoa) - d * std::sin(aoa);
            // stick: pitch / yaw toward the nose target, roll toward the bank
            Vec3d nose = v->q.rotate({0, 0, 1}), upB = v->q.rotate({0, 1, 0});
            Vec3d er = v->q.conj().rotate(cross(nose, noseT));
            Vec3d upP = norm(upT - nose * dot(upT, nose));
            double th = std::atan2(dot(cross(upB, upP), nose), dot(upB, upP));
            ControlInput in;
            in.pitch = clampd(er.x * 12.0, -1, 1);
            in.yaw = clampd(er.y * 12.0, -1, 1);
            in.roll = clampd(th * 4.0, -1, 1);
            v->sas = SAS::Stability;
            v->rcs = true;
            flightDebugInput(&in);
            W.input = in;
            W.update(h);
            (void)s2;
        };
        if (alt > b.atmTop + 5e3) {
            // the coast down from the de-orbit burn: warp, then turn to the entry attitude in time
            if (alt > b.atmTop + 60e3) {
                W.warpIndex = std::min(6, W.maxWarpIndex());
                holdAttitude(*v, norm(vAir), up); // (on rails at warp)
                for (int k = 0; k < 8; k++) W.update(0.05);
            } else {
                if (W.warpIndex > 0) W.warpIndex = 0;
                for (int k = 0; k < 8; k++) pilot(0.05);
            }
            return true;
        }
        if (W.warpIndex > 0) W.warpIndex = 0;
        for (int k = 0; k < 8 && !v->destroyed; k++) pilot(0.05);
        if (T.frames % 60 == 0) {
            Vec3d nose = v->q.rotate({0, 0, 1});
            double aoaNow = std::acos(clampd(dot(nose, norm(W.surfaceVel(*v))), -1, 1)) / DEG;
            Vec3d u = norm(v->r), d = norm(W.surfaceVel(*v)), n = norm(u - d * dot(u, d));
            Vec3d upB = v->q.rotate({0, 1, 0}), upP = norm(upB - d * dot(upB, d));
            double bankNow = std::acos(clampd(dot(upP, n), -1, 1)) / DEG;
            std::fprintf(T.log, "  entry: alt %6.1f km speed %5.0f m/s Mach %5.2f decel %.2f g sink %6.1f m/s AoA %5.1f (want %4.1f) bank %5.1f (want %5.1f) flux %4.0f kW/m2 heat %.2f RCS %.0f kg surf %+.2f %+.2f %+.2f q %.1f kPa\n",
                         alt / 1000, sp, v->tMach, D / G0, -dot(W.surfaceVel(*v), up), aoaNow, aoaDeg, bankNow, bankCmd / DEG, v->tHeatFlux / 1000,
                         v->tHeatMax, v->monoAmt, v->surf.x, v->surf.y, v->surf.z, v->tDynP / 1000);
            if (alt < 80e3) mark = std::max(mark, std::fabs(aoaNow - aoaDeg)); // (the attitude held?)
            if (std::getenv("SF_SPDIAG")) {
                const Vec3d* g = g_rotDbg;
                std::fprintf(T.log, "  rot: tauExt %+.0f %+.0f %+.0f  I*a %+.0f %+.0f %+.0f  gyro %+.0f %+.0f %+.0f  surfQ %.0f %.0f %.0f  surfNow %+.0f  I %.3g %.3g %.3g  com %.2f %.2f %.2f  aero %p n %d\n",
                             g[0].x, g[0].y, g[0].z, g[1].x, g[1].y, g[1].z, g[2].x, g[2].y, g[2].z, g[3].x, g[3].y, g[3].z, g[4].x, g[5].x, g[5].y, g[5].z,
                             v->mp.com.x, v->mp.com.y, v->mp.com.z, (void*)v->aero.get(), v->aero ? v->aero->ncomp : -1);
                Vec3d s0 = v->surf;
                v->surf = {0, s0.y, s0.z};
                Vec3d t0 = debugAeroTorque(W, *v);
                v->surf = {1, s0.y, s0.z};
                Vec3d t1 = debugAeroTorque(W, *v);
                v->surf = {-1, s0.y, s0.z};
                Vec3d tm = debugAeroTorque(W, *v);
                v->surf = s0;
                std::fprintf(T.log, "  aero pitch: surf 0 %+.0f, +1 %+.0f, -1 %+.0f (model authority %.0f, Mach %.1f)\n", t0.x, t1.x, tm.x, g[3].x, v->tMach);
            }
        }
        if (v->destroyed) {
            if (T.tf == 5) return finish((std::string("without tiles: ") + v->fate + " -> ok (the thermal protection is needed)").c_str());
            return finish((std::string("FAIL (") + v->fate + ")").c_str());
        }
        if (sp < 900 || alt < 25e3) {
            status("entry done");
            if (T.tf == 5) return finish("FAIL (an aluminium orbiter survived the entry: the heating is too weak)");
            char b2[320];
            std::snprintf(b2, sizeof b2,
                          "entry flown with the controls: peak heat %.2f of the tile limit, peak flux %.0f kW/m2, peak load %.1f g, worst angle-of-attack error %.1f deg, RCS left %.0f kg -> %s",
                          peakHeat, peakFlux / 1000, peakG, mark, v->monoAmt, peakHeat < 1 && peakG < 3.0 && mark < 15 ? "ok" : "FAIL");
            return finish(b2);
        }
        if (v->tHeatFlux > 3e5 && !T.shotEntry) {
            T.shotEntry = true;
            shot("27_sp_1_entry.png");
        }
        return true;
    }
    Mat3d RT = W.U.bodyRot(v->ref, W.t).T();
    double agl = alt - terrain::height(v->ref, RT * v->r);
    Vec3d nose = v->q.rotate({0, 0, 1});
    double pitch = std::asin(std::clamp(dot(nose, up), -1.0, 1.0)) / DEG;
    double bank = std::asin(std::clamp(dot(v->q.rotate({1, 0, 0}), up), -1.0, 1.0)) / DEG;
    double vz = dot(vAir, up);
    if (T.tf == 2) {
        // ---------------- the orbiter's glide and landing, flown with the stick (rate command)
        ControlInput in;
        double want;
        if (T.tfSub == 0) {
            if (agl > 450) want = std::clamp(-6.0 + 0.5 * (sp - 170), -25.0, 12.0); // hold 170 m/s on the glide
            else {
                // pre-flare to a shallow path, then the final flare: the sink rate falls with the height
                double vzT = -std::max(3.0, agl * 0.1);
                want = std::clamp(pitch + (vzT - vz) * 0.25, -12.0, 12.0); // (tail clearance)
            }
            if (v->rolling || v->landed) {
                T.tfSub = 1;
                char b2[160];
                std::snprintf(b2, sizeof b2, "spaceplane 2: touchdown at %.1f m/s, sink %.2f m/s, pitch %.1f deg\n", sp, -vz, pitch);
                std::fputs(b2, T.log);
                shot("27_sp_2_touchdown.png");
            }
        } else want = pitch;
        v->sas = SAS::Stability;
        for (int k = 0; k < 8 && !v->destroyed; k++) {
            nose = v->q.rotate({0, 0, 1});
            Vec3d u = norm(v->r);
            double pk = std::asin(std::clamp(dot(nose, u), -1.0, 1.0)) / DEG;
            double bk = std::asin(std::clamp(dot(v->q.rotate({1, 0, 0}), u), -1.0, 1.0)) / DEG;
            in.pitch = T.tfSub == 0 ? std::clamp((pk - want) * 0.25, -1.0, 1.0) : 0.0;
            in.roll = T.tfSub == 0 ? std::clamp(-bk * 0.08, -1.0, 1.0) : 0.0;
            flightDebugInput(&in);
            W.input = in;
            W.update(0.05);
        }
        if (T.frames % (agl < 400 ? 3 : 20) == 0)
            std::fprintf(T.log, "  glide: agl %7.1f speed %6.1f vz %6.1f pitch %5.1f want %5.1f bank %5.1f q %.1f kPa\n", agl, sp, vz, pitch, want, bank, v->tDynP / 1000);
        if (v->destroyed) return finish((std::string("FAIL (") + v->fate + ")").c_str());
        if (T.tfSub == 1 && sp < 1.0) return finish("glided, landed on the gear and stopped -> ok");
        if (W.t - T.tfRef > 400) return finish("FAIL (no landing within 400 s)");
        return true;
    }
    // ---------------- SSTO: take-off run and rotation with the stick, then the climb flown by attitude
    if (T.tfSub == 0) {
        ControlInput in;
        v->sas = SAS::Stability;
        for (int k = 0; k < 8 && !v->destroyed; k++) {
            nose = v->q.rotate({0, 0, 1});
            double pk = std::asin(std::clamp(dot(nose, norm(v->r)), -1.0, 1.0)) / DEG;
            double s2 = len(W.surfaceVel(*v));
            in.pitch = s2 > 130 ? std::clamp((pk - 7.5) * 0.25, -1.0, 1.0) : 0.0;
            flightDebugInput(&in);
            W.input = in;
            W.update(0.05);
        }
        if (T.frames % 20 == 0)
            std::fprintf(T.log, "  take-off: agl %6.1f speed %6.1f pitch %5.1f vz %5.1f rolling %d landed %d\n", agl, sp, pitch, vz, (int)v->rolling, (int)v->landed);
        if (agl > 150 && !v->rolling && !v->landed) {
            flightDebugInput(nullptr);
            status("lift-off");
            shot("27_sp_3_takeoff.png");
            T.shotAscent = false;
            v->legsDeployed = false; // gear up
            T.tfSub = 1;
        }
        if (W.t - T.tfRef > 120) return finish("FAIL (no take-off within 120 s)");
        return true;
    }
    // flight path: climb, then ride a dynamic-pressure corridor (~45 kPa) to Mach 5+ on the air-breathing
    // mode; in rocket mode pitch up, cut at an apoapsis of 200 km, coast, circularise
    v->sas = SAS::Stability;
    if (T.tfSub == 1 || T.tfSub == 2) {
        for (int k = 0; k < 8 && !v->destroyed; k++) {
            Vec3d u = norm(v->r), va = W.surfaceVel(*v), d = norm(va);
            Vec3d h = norm(d - u * dot(d, u));
            double gam = std::asin(std::clamp(dot(d, u), -1.0, 1.0));
            double gT;
            double a2 = len(v->r) - b.radius;
            if (T.tfSub == 1) {
                double q = v->tDynP;
                gT = a2 < 6000 ? 12 * DEG : std::clamp(gam + (q - 30e3) / 30e3 * 0.10, -3 * DEG, 15 * DEG);
                if (v->ccMode == 3) {
                    T.tfSub = 2;
                    status("rocket mode");
                    shot("27_sp_3_rocketmode.png");
                }
            } else {
                Conic k2 = stateToConic(v->r, v->v, b.GM);
                gT = k2.ap - b.radius < 110e3 ? 15 * DEG : gam;
                if (k2.valid && k2.e < 1 && k2.ap - b.radius > 200e3) {
                    v->throttle = 0;
                    T.tfSub = 3;
                    status("engines off, coasting to apoapsis");
                    break;
                }
            }
            double aoa = T.tfSub == 1 ? std::clamp(3 * DEG + 2.0 * (gT - gam), -2 * DEG, 12 * DEG) : 0.0;
            holdAttitude(*v, h * std::cos(gT + aoa) + u * std::sin(gT + aoa), u);
            W.update(0.05);
        }
        if (T.frames % 100 == 0) status(T.tfSub == 1 ? "air-breathing climb" : "rocket climb");
        // the launch site seen from 14 km above just after the lift-off: the island's real outline, not a disc
        if (T.tfSub == 1 && !T.shotAscent) {
            T.shotAscent = true;
            flightDebugView(2.4, 1.35, 14000, Vec3d{});
            T.tfRef2 = T.frames;
        }
        if (T.tfRef2 > 0 && T.frames == T.tfRef2 + 6) {
            shot("27_sp_3_island.png");
            flightDebugView(2.4, 0.2, 140, Vec3d{});
            T.tfRef2 = 0;
        }
        if (v->tFlameout) return finish("FAIL (propellant out before orbit)");
        return true;
    }
    if (T.tfSub == 3) {
        // coast to the apoapsis (warp above the air), then prograde until the periapsis is above 150 km
        double tAp = c.valid && c.e < 1 ? timeToTrueAnomaly(c, PI) : 0;
        holdAttitude(*v, norm(v->v), up);
        if (alt > b.atmTop && tAp > 60) {
            W.warpIndex = std::min(5, W.maxWarpIndex());
            for (int k = 0; k < 8; k++) W.update(0.05);
            return true;
        }
        if (W.warpIndex > 0) W.warpIndex = 0;
        if (tAp > 25) {
            for (int k = 0; k < 8; k++) W.update(0.05);
            return true;
        }
        v->throttle = 1;
        for (int k = 0; k < 8 && !v->destroyed; k++) {
            holdAttitude(*v, norm(v->v), norm(v->r));
            W.update(0.05);
            Conic k2 = stateToConic(v->r, v->v, b.GM);
            if (k2.valid && k2.pe - b.radius > 150e3) {
                v->throttle = 0;
                status("in orbit");
                double prop = 0;
                for (size_t i = 0; i < v->comps.size(); i++)
                    if (v->comps[i].func == Func::Tank) prop += v->prop[i];
                char b2[200];
                std::snprintf(b2, sizeof b2, "runway to orbit on one stage: %.1f t of propellant left, %.1f t in orbit, peak heat %.2f -> ok", prop / 1000,
                              W.vesselMass(*v) / 1000, peakHeat);
                shot("27_sp_3_orbit.png");
                return finish(b2);
            }
        }
        if (v->tFlameout) return finish("FAIL (propellant out during the circularisation)");
    }
    return true;
}

// ------------------------------------------------------------------ docking (--docktest)
// two modules started in orbit 60 m apart: the drift between them (x1 and under warp), the approach flown with
// RCS translation to the docking, undocking, docking again and a third module: a small station
static const Vessel* vesselById(World& W, int id) {
    for (const Vessel& v : W.vessels)
        if (v.id == id && !v.destroyed) return &v;
    return nullptr;
}
// RCS translation toward the nearest pair of free ports facing each other; returns their gap [m] (-1: none)
static double g_apLat = 0, g_apAng = 0, g_apAlong = 0;
static double approachStep(World& W, Vessel& a, const Vessel& b, double& closing) {
    int ia = -1, ib = -1;
    double best = 1e30;
    Vec3d pa, da, pb, db;
    for (int i = 0; i < (int)a.comps.size(); i++) {
        if (a.comps[i].func != Func::Docking || a.comps[i].dockedTo) continue;
        Vec3d p1, d1;
        W.portFrame(a, i, p1, d1);
        for (int j = 0; j < (int)b.comps.size(); j++) {
            if (b.comps[j].func != Func::Docking || b.comps[j].dockedTo) continue;
            Vec3d p2, d2;
            W.portFrame(b, j, p2, d2);
            if (dot(d1, d2) > -0.9) continue;
            double g = len(p2 - p1);
            if (g < best) { best = g; ia = i; ib = j; pa = p1; da = d1; pb = p2; db = d2; }
        }
    }
    if (ia < 0) return -1;
    Vec3d rel = a.q.conj().rotate(pb - pa);
    Vec3d vrel = a.q.conj().rotate(a.v - b.v);
    double dist = len(rel);
    // straight in along the port axis: come onto the axis first, then close at ~0.1 m/s near contact
    Vec3d ax = a.q.conj().rotate(da);
    double along = dot(rel, ax);
    Vec3d lat = rel - ax * along;
    // closing speed with the distance (0.05 m/s at contact); hold (or back off past the port) until the
    // ports are on one axis: within 0.1 m + 5 % of the distance
    double vAlong = along > 0 ? std::clamp(0.02 * along + 0.05, 0.05, 0.6) : std::clamp(0.05 * along - 0.05, -0.3, -0.05);
    if (along > 0 && len(lat) > 0.1 + 0.05 * along) vAlong = std::min(vAlong, 0.0);
    Vec3d vDes = ax * vAlong + lat * 0.2;
    Vec3d e = (vDes - vrel) * 6.0;
    ControlInput in;
    in.tx = std::clamp(e.x, -1.0, 1.0);
    in.ty = std::clamp(e.y, -1.0, 1.0);
    in.tz = std::clamp(e.z, -1.0, 1.0);
    // attitude with the stick: turn the own port to face the target port (rate command, RCS and wheels)
    {
        Vec3d er = a.q.conj().rotate(cross(da, -db));
        in.pitch = std::clamp(er.x * 12.0, -1.0, 1.0);
        in.yaw = std::clamp(er.y * 12.0, -1.0, 1.0);
        in.roll = std::clamp(er.z * 12.0, -1.0, 1.0);
    }
    a.rcs = true;
    a.sas = SAS::Stability;
    flightDebugInput(&in);
    W.input = in;
    closing = -dot(a.q.rotate(vrel), norm(pb - pa));
    g_apLat = len(lat);
    g_apAng = std::acos(std::clamp(-dot(da, db), -1.0, 1.0)) / DEG;
    g_apAlong = along;
    (void)db;
    return dist;
}
// ------------------------------------------------------------------ maps and saving (--savetest)
// a named map, autosave, reload, save on going home, the unsaved-changes dialog (cancel, then save and
// quit) and the factory's last design across a restart; everything written below the test folder
bool saveTestStep(App& app) {
    World& W = app.world;
    static int bad = 0, A = 0;
    static double tSaved = 0;
    static size_t nSaved = 0;
    const int f = T.frames;
    auto check = [&](bool ok, const char* what) {
        std::fprintf(T.log, "save: %s -> %s\n", what, ok ? "ok" : "FAIL");
        if (!ok) bad++;
    };
    if (f > 4000) { // (something did not happen: end without the quit dialog)
        check(false, "timeout");
        app.persist = false;
        return false;
    }
    if (f == 2) {
        app.persist = true;
        app.mapsDir = T.dir + "\\maps";
        app.lastDesignFile = T.dir + "\\last_design.sfd";
        app.quickFile = app.saveFile = T.dir + "\\quick.sfs";
        app.designText.clear();
        app.screen = Screen::Menu;
        app.newMapOpen = true;
    }
    // the name field: Ctrl+A selects all and typing replaces it; Shift + arrows select a part; Backspace /
    // Delete remove the selection or one character (also Japanese)
    auto ti = [](std::vector<int> ch, std::vector<int> k, bool ctrl, bool shift) { ui::testTextInput(7301, ch, k, ctrl, shift); };
    if (f == 4) ti({}, {KEY_A}, true, false);
    if (f == 5) shot("40a_name_selected.png");
    if (f == 6) ti({'B', 'a', 's', 'e'}, {}, false, false);
    if (f == 7) {
        check(debugNewMapName() == "Base", "name field: Ctrl+A then typing replaces the whole name");
        ti({}, {KEY_HOME}, false, false);
    }
    if (f == 8) ti({}, {KEY_RIGHT}, false, true);
    if (f == 9) ti({}, {KEY_RIGHT}, false, true);
    if (f == 10) shot("40b_name_part.png");
    if (f == 11) ti({'X'}, {}, false, false);
    if (f == 12) {
        check(debugNewMapName() == "Xse" && ui::testCaret() == 1, "name field: Shift+Right selects a part, typing replaces it");
        ti({}, {KEY_END}, false, false);
    }
    if (f == 13) ti({}, {KEY_LEFT}, false, true);
    if (f == 14) ti({}, {KEY_BACKSPACE}, false, false);
    if (f == 15) {
        check(debugNewMapName() == "Xs", "name field: Backspace removes the selection");
        ti({0x57FA, 0x5730}, {}, false, false); // (Japanese, as the IME delivers it)
    }
    if (f == 16) ti({}, {KEY_HOME}, false, false);
    if (f == 17) ti({}, {KEY_DELETE}, false, false);
    if (f == 18) {
        check(debugNewMapName() == "s基地", "name field: Japanese input and Delete");
        shot("40_new_map.png");
    }
    if (f == 19) {
        app.newMapOpen = false;
        ui::cancelEdit();
        bool ok = createMap(app, "月面基地 テスト");
        auto maps = listMaps(app);
        check(ok && maps.size() == 1 && maps[0].name == "月面基地 テスト" && app.screen == Screen::Factory, "new map created with its name");
        app.design = builtinTemplates()[1];
        app.lastLaunched = craftFromDesign(app.design);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
    }
    if (f == 40) {
        check(mapDirty(app), "flying makes the map unsaved");
        app.lastSaveReal = -1e9; // (the autosave interval has passed)
    }
    if (f == 44) {
        // the map's picture: written next to the save, replaced at every save (one file)
        std::string png = app.mapPath.substr(0, app.mapPath.size() - 4) + ".png";
        Image im = LoadImage(png.c_str());
        double sum = 0;
        if (im.data) {
            Color* px = LoadImageColors(im);
            for (int k = 0; k < im.width * im.height; k++) sum += px[k].r + px[k].g + px[k].b;
            UnloadImageColors(px);
            sum /= 3.0 * im.width * im.height;
        }
        int pngs = 0;
        for (auto& e : std::filesystem::directory_iterator(app.mapsDir)) pngs += e.path().extension() == ".png";
        check(im.data && im.width == 400 && sum > 8 && pngs == 1, "the save wrote one picture of the view (no HUD)");
        std::fprintf(T.log, "save: picture %dx%d, mean brightness %.1f\n", im.width, im.height, sum);
        if (im.data) UnloadImage(im);
    }
    if (f == 42) // (the flight goes on after the save: the time of the save is what tells)
        check(GetTime() - app.lastSaveReal < 1.0 && app.notice.find("テスト") != std::string::npos && listMaps(app)[0].savedAt > 0, "autosave wrote the map");
    if (f == 50) {
        tSaved = W.t;
        nSaved = W.vessels.size();
        double t0 = GetTime();
        quickSave(app);
        std::fprintf(T.log, "save: writing the map took %.1f ms (%zu vessels)\n", (GetTime() - t0) * 1000, nSaved);
        bool ok = loadMap(app, app.mapPath);
        check(ok && W.vessels.size() == nSaved && std::fabs(W.t - tSaved) < 1e-6 && W.mapName == "月面基地 テスト" && !mapDirty(app),
              "the map loads back (time, vessels, name)");
        flightOnEnter(app);
    }
    if (f == 70) {
        check(mapDirty(app), "unsaved again after flying on");
        app.screen = Screen::Menu; // (home screen)
    }
    if (f == 72) check(!mapDirty(app), "going to the home screen saved the map");
    if (f == 74) app.loadMapOpen = true;
    if (f == 76) {
        bool ok = renameMap(app, app.mapPath, "改名テスト");
        auto maps = listMaps(app);
        check(ok && W.mapName == "改名テスト" && !maps.empty() && maps[0].name == "改名テスト", "renamed the map");
        debugRenameMap(app, "改名テスト");
    }
    if (f == 80) shot("41_load_map.png");
    if (f == 81) {
        app.loadMapOpen = false;
        app.screen = Screen::Flight;
        flightOnEnter(app);
    }
    if (f == 86) flightDebugPause(true);
    if (f == 90) {
        shot("44_pause_menu.png");
        flightDebugPause(false);
        flightDebugVessels(true);
    }
    if (f == 94) {
        shot("45_vessel_list.png");
        flightDebugVessels(false);
    }
    if (f == 100) {
        check(mapDirty(app), "unsaved before quitting");
        app.quit = true; // (the window's close button)
        A = f;
    }
    if (A && f == A + 4) {
        check(app.closeAsk, "quitting with unsaved changes asks first");
        shot("42_unsaved_dialog.png");
        g_testCloseChoice = 2; // Cancel
    }
    if (A && f == A + 5) {
        check(!app.closeAsk && mapDirty(app), "cancel keeps playing (nothing saved)");
        // the factory's design: written when leaving the factory, read back by a fresh start
        app.design = builtinTemplates()[3];
        if (app.design.stack.size() > 2) app.design.stack[1].hidden = true; // (a part hidden in the view)
        app.screen = Screen::Factory;
    }
    if (A && f == A + 7) shot("43_factory_hidden_part.png");
    if (A && f == A + 8) app.screen = Screen::Menu;
    if (A && f == A + 10) {
        App fresh;
        fresh.lastDesignFile = app.lastDesignFile;
        loadLastDesign(fresh);
        check(designToText(fresh.design) == designToText(builtinTemplates()[3]), "the last design is there after a restart");
        app.screen = Screen::Flight;
        flightOnEnter(app);
    }
    if (A && f == A + 30) {
        check(mapDirty(app), "unsaved before the second quit");
        app.quit = true;
    }
    if (A && f == A + 32) g_testCloseChoice = 0; // Save and quit
    if (A && f == A + 33) {
        check(!app.closeAsk && !mapDirty(app), "save and quit wrote the map");
        std::fprintf(T.log, "save: %s\n", bad == 0 ? "maps, autosave, reload, home-screen save, quit dialog, last design -> ok" : "FAIL");
        logState(app, "end");
        std::fclose(T.log);
        T.log = nullptr;
        T.phase = -1; // (the main loop ends this frame)
    }
    return true;
}

// ------------------------------------------------------------------ vessel menu + navigation (--navtest)
// The menu of a clicked vessel (hide, list, remove, navigate), then a pilot that flies a Crew Ferry from a
// lower orbit, a few hundred km behind, to a docking with the station core using only what the navigation
// panel says: the rendezvous burn along the burn marker (SAS Burn), the coast, the RCS amounts of the
// approach with SAS Dock. The test passes when the ferry docks.
bool navTestStep(App& app) {
    World& W = app.world;
    const int f = T.frames;
    static int coreId = -1, nodeId = -1, ferryId = -1, bad = 0;
    static double tStart = 0, dvUsed = 0, lastLog = -1e9, maxClose = 0, tNear = -1;
    static navg::Mode lastMode = navg::Mode::None;
    static bool shotT = false, shotC = false, shotP = false, shotP2 = false;
    static int shotsDone = 0;
    auto check = [&](bool ok, const char* what) {
        std::fprintf(T.log, "nav: %s -> %s\n", what, ok ? "ok" : "FAIL");
        std::fflush(T.log);
        if (!ok) bad++;
    };
    auto fin = [&](const char* verdict) {
        flightDebugInput(nullptr);
        W.setWarp(0);
        std::fprintf(T.log, "nav: %s\n", verdict);
        return false;
    };
    if (f == 2) {
        bad = 0;
        W.newGame(Settings{});
        app.gameStarted = true;
        int earth = W.U.find("Earth");
        sunAt(W, earth, siteDir(0, 0), 30);
        W.launchInOrbit(craftNamed("Station Core Module"), -1, 0);
        coreId = W.activeVessel()->id;
        W.launchInOrbit(craftNamed("Station Node Module"), coreId, 150);
        nodeId = W.activeVessel()->id;
        W.launchInOrbit(craftNamed("Crew Ferry"), coreId, 60);
        ferryId = W.activeVessel()->id;
        app.screen = Screen::Flight;
        flightOnEnter(app);
        flightDebugView(2.6, 0.3, 60, Vec3d{});
        W.activeVessel()->targetVessel = -1;
    }
    if (f == 6) flightDebugMenu(nodeId, -1);
    if (f == 9) shot("80_vessel_menu.png");
    if (f == 10) flightDebugMenu(nodeId, 3); // Hide vessel
    if (f == 13) {
        const Vessel* n = vesselById(W, nodeId);
        check(n && n->hidden && !flightDebugMenuOpen(), "menu: hide vessel");
        flightDebugVessels(true);
    }
    if (f == 16) shot("81_vessel_list.png");
    if (f == 17) {
        flightDebugVessels(false);
        flightDebugMenu(nodeId, 4); // Remove
    }
    if (f == 20) {
        check(vesselById(W, nodeId) == nullptr && W.activeVessel() && W.activeVessel()->id == ferryId, "menu: remove vessel (the vessel flown stays)");
        flightDebugMenu(coreId, 1); // Navigate
    }
    if (f == 23) {
        Vessel* a = W.activeVessel();
        check(a && a->targetVessel == coreId, "menu: navigate sets the target");
        // the ferry into a lower orbit (35 km below), 6 degrees behind and 0.15 degrees out of the plane
        Vessel* c = const_cast<Vessel*>(vesselById(W, coreId));
        const Body& b = W.U.bodies[c->ref];
        auto rot = [](const Vec3d& v, const Vec3d& k, double ang) {
            return v * std::cos(ang) + cross(k, v) * std::sin(ang) + k * (dot(k, v) * (1 - std::cos(ang)));
        };
        Vec3d h = norm(cross(c->r, c->v));
        Vec3d rdir = rot(norm(c->r), h, -6.0 * DEG);
        Vec3d hh = rot(h, rdir, 0.15 * DEG);
        double R = len(c->r) - 35e3;
        a->r = rdir * R;
        a->v = norm(cross(hh, rdir)) * std::sqrt(b.GM / R);
        a->q = Quatd::lookRot(norm(a->v), rdir);
        a->w = {};
        a->landed = false;
        a->sas = SAS::Stability;
        a->rcs = false;
        W.history.clear();
        W.recordHistory(true);
        tStart = W.t;
        dvUsed = 0;
        maxClose = 0;
        tNear = -1;
        lastMode = navg::Mode::None;
        shotT = shotC = shotP = shotP2 = false;
        shotsDone = 0;
        std::fprintf(T.log, "nav: ferry %.0f km below the station, %.0f km away\n", 35.0, len(c->r - a->r) / 1000);
        double t0 = GetTime();
        navg::Guidance g0;
        for (int k = 0; k < 400 && (g0 = navg::compute(W, *a, *c, 0, 1e9)).planning; k++) WaitTime(0.002);
        std::fprintf(T.log, "nav: the first plan arrived after %.1f ms (worker thread); frame cost of compute() %s\n", (GetTime() - t0) * 1000,
                     g0.mode == navg::Mode::Transfer ? "-> ok" : "FAIL");
        t0 = GetTime();
        for (int k = 0; k < 100; k++) navg::compute(W, *a, *c, 0, 1e9);
        std::fprintf(T.log, "nav: compute() per frame with a plan: %.3f ms\n", (GetTime() - t0) * 1000 / 100);
        std::fflush(T.log);
    }
    if (f < 24) return true;
    Vessel* a = W.activeVessel();
    if (!a || a->destroyed) return fin("FAIL (the ferry was lost)");
    // docked? (the station absorbed into the ferry or the other way round: one vessel with a docked port)
    bool docked = false;
    for (const Component& c : a->comps) docked |= c.func == Func::Docking && c.dockedTo != 0;
    if (docked) {
        flightDebugInput(nullptr);
        if (!shotP2) {
            shotP2 = true;
            flightDebugView(2.4, 0.25, 60, Vec3d{});
            return true;
        }
        shot("85_nav_docked.png");
        std::fprintf(T.log, "nav: docked after %s, rendezvous delta-v %.1f m/s, peak closing speed below 100 m %.2f m/s\n", formatTime(W.t - tStart).c_str(),
                     dvUsed, maxClose);
        check(true, "the ferry docked following the navigation panel");
        return fin(bad == 0 ? "menu, navigation and docking -> ok" : "FAIL");
    }
    const Vessel* t = vesselById(W, coreId);
    if (!t) return fin("FAIL (the station is gone)");
    if (W.t - tStart > 12 * 3600 || f > 60000) return fin("FAIL (no docking within 12 h)");
    if (f % 200 == 0) {
        std::fprintf(T.log, "  [frame %d] t+%s range %.0f m, warp idx %d x%.0f, warpTo in %.0f s, limit '%s', sas %d thr %.2f\n", f,
                     formatTime(W.t - tStart).c_str(), len(t->r - a->r), W.warpIndex, W.effectiveWarp, W.warpTo > 0 ? W.warpTo - W.t : -1.0,
                     W.warpLimitReason.c_str(), (int)a->sas, a->throttle);
        std::fflush(T.log);
    }
    double Fvac = 0, mdot = 0, srbA, srbC, amt[PROP_N], cap[PROP_N];
    stageResources(*a, amt, cap, srbA, srbC, Fvac, mdot);
    double accMax = Fvac / std::max(W.vesselMass(*a), 1.0);
    for (int step = 0; step < 12; step++) {
        navg::Guidance g = navg::compute(W, *a, *t, f * 0.05, 1.0); // (a plan search about every 20 test frames, as ~1 s in the game)
        a->navBurn = g.burn;
        if (g.mode != lastMode) {
            std::fprintf(T.log, "nav: mode %d at %s, range %.0f m\n", (int)g.mode, formatTime(W.t - tStart).c_str(), g.range);
            std::fflush(T.log);
            lastMode = g.mode;
        }
        if (g.planning) { // (the first plan comes from a worker thread a moment later)
            W.update(0.05);
            continue;
        }
        if (g.mode == navg::Mode::Message) return fin(("FAIL (" + g.message + ")").c_str());
        ControlInput in;
        a->throttle = 0;
        if (g.mode == navg::Mode::Transfer) {
            double dt = g.tBurn - W.t;
            if (!shotT && dt > 0) {
                shotT = true;
                W.setWarp(0);
                return true; // (the next frame draws the panel)
            }
            if (shotT && shotsDone < 1) { shot("82_nav_transfer.png"); shotsDone = 1; }
            if (dt > 90) {
                if (W.warpTo < 0) W.startWarpTo(g.tBurn - 60);
                W.update(0.05);
                continue;
            }
            W.setWarp(0);
            a->sas = SAS::Burn;
            if (!a->ignited) W.stage(*a); // (the panel says: engines off, press Space)
            Vec3d nose = a->q.rotate({0, 0, 1});
            double ang = std::acos(clampd(dot(nose, norm(g.burn)), -1, 1)) / DEG;
            double burnT = g.dvTotal / std::max(accMax, 1e-3);
            if (ang < 3 && dt <= burnT * 0.5 + 0.05) a->throttle = clampd(g.dvTotal / (accMax * 1.5), 0.05, 1.0);
        } else if (g.mode == navg::Mode::Coast) {
            a->sas = SAS::Stability;
            if (!shotC) {
                shotC = true;
                W.setWarp(0);
                return true;
            }
            if (shotsDone < 2) { shot("83_nav_coast.png"); shotsDone = 2; }
            if (g.tArrive - W.t > 300 && W.warpTo < 0) W.startWarpTo(g.tArrive - 240);
            if (W.warpTo > 0) { W.update(0.05); continue; }
        } else if (g.mode == navg::Mode::Proximity) {
            W.setWarp(0);
            if (tNear < 0) tNear = W.t;
            a->rcs = true;
            double dv = len(g.dvCraft);
            if (g.range < 100) maxClose = std::max(maxClose, g.closing);
            if (dv > 3) {
                // a large velocity change: the main engine along the burn marker
                a->sas = SAS::Burn;
                if (!a->ignited) W.stage(*a);
                Vec3d nose = a->q.rotate({0, 0, 1});
                double ang = std::acos(clampd(dot(nose, norm(g.burn)), -1, 1)) / DEG;
                if (ang < 4) a->throttle = clampd(dv / (accMax * 1.5), 0.05, 1.0);
            } else {
                a->sas = g.alignDeg >= 0 ? SAS::Dock : SAS::Stability;
                // hold the RCS keys the panel names (each axis on while its amount is above 2 cm/s)
                auto key = [](double v) { return std::fabs(v) < 0.02 ? 0.0 : (v > 0 ? 1.0 : -1.0); };
                in.tz = key(g.dvCraft.z);
                in.ty = key(g.dvCraft.y);
                in.tx = key(g.dvCraft.x);
            }
            if (!shotP && g.range < 150 && !g.viaWaypoint) {
                shotP = true;
                flightDebugInput(&in);
                W.input = in;
                return true;
            }
            if (shotP && shotsDone < 3) { shot("84_nav_approach.png"); shotsDone = 3; }
            if (W.t - lastLog > 120) {
                lastLog = W.t;
                std::fprintf(T.log, "  near: range %.1f m along %.1f off %.2f align %.1f deg dv %.2f m/s (via %d)\n", g.range, g.along, g.lateral, g.alignDeg,
                             dv, (int)g.viaWaypoint);
            }
        }
        flightDebugInput(&in);
        W.input = in;
        W.update(0.05);
        a = W.activeVessel();
        if (!a) break;
        if (a->throttle > 0) dvUsed += accMax * a->throttle * 0.05;
        bool dk = false;
        for (const Component& c : a->comps) dk |= c.func == Func::Docking && c.dockedTo != 0;
        if (dk) break;
    }
    return true;
}


// ------------------------------------------------------------------ copies, saving and adding designs (--duptest)
// Duplicate in the CAD must give exactly the same craft beside the original (every built-in craft, all parts
// selected): the same drawn components (expanded symmetry, wing root ends, gear folds) moved by one offset, the
// same mass properties. Mirror X twice gives the original back; the mirror keeps the shape's asymmetric axes.
// Saving and reading a craft or a factory design gives every field back exactly (Japanese names included).
// Adding a design puts it on top of the current one with its parts selected. The factory asks before
// overwriting a saved design (cancel / new name / overwrite).
bool dupTestStep(App& app) {
    const int f = T.frames;
    static int bad = 0;
    auto check = [&](bool ok, const std::string& what) {
        std::fprintf(T.log, "dup: %s -> %s\n", what.c_str(), ok ? "ok" : "FAIL");
        std::fflush(T.log);
        if (!ok) bad++;
    };
    auto matEq = [](const Mat3d& a, const Mat3d& b, double eps) {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                if (std::fabs(a.m[i][j] - b.m[i][j]) > eps) return false;
        return true;
    };
    auto vecEq = [](const Vec3d& a, const Vec3d& b, double eps) { return len(a - b) <= eps; };
    // every field of a component (exact)
    auto sameComp = [](const Component& a, const Component& b) {
        return a.name == b.name && a.func == b.func && a.shape == b.shape && a.dims.x == b.dims.x && a.dims.y == b.dims.y && a.dims.z == b.dims.z &&
               a.pos.x == b.pos.x && a.pos.y == b.pos.y && a.pos.z == b.pos.z && a.rotDeg.x == b.rotDeg.x && a.rotDeg.y == b.rotDeg.y &&
               a.rotDeg.z == b.rotDeg.z && a.material == b.material && a.hollow == b.hollow && a.wall == b.wall && a.def == b.def &&
               a.cluster == b.cluster && a.prop == b.prop && a.fill == b.fill && a.stage == b.stage && a.fireFrom == b.fireFrom &&
               a.symmetry == b.symmetry && a.color.r == b.color.r && a.color.g == b.color.g && a.color.b == b.color.b && a.color.a == b.color.a &&
               a.massOverride == b.massOverride && a.taper == b.taper && a.sweep == b.sweep && a.noseLen == b.noseLen && a.tailLen == b.tailLen &&
               a.tailR == b.tailR && a.upsweep == b.upsweep && a.bluff == b.bluff && a.boxy == b.boxy && a.dockTag == b.dockTag &&
               a.dockedTo == b.dockedTo && a.origin == b.origin && a.stage0 == b.stage0 && a.engOff == b.engOff && a.rootSide == b.rootSide &&
               a.foldDir == b.foldDir && a.flap == b.flap && a.cargo == b.cargo && a.rim == b.rim;
    };
    std::vector<Craft> all = builtinCrafts();
    for (auto& d : builtinTemplates()) all.push_back(craftFromDesign(d));
    if (f == 2) {
        bad = 0;
        // ---- 1. Duplicate: every built-in craft, everything selected
        int nOk = 0, nCraft = 0;
        for (const Craft& c0 : all) {
            if (c0.comps.empty() || c0.comps.size() > 200) continue;
            nCraft++;
            cadSetCraft(app, c0);
            std::vector<int> s;
            for (int i = 0; i < (int)c0.comps.size(); i++) s.push_back(i);
            cadDebugSelect(s);
            cadDebugDuplicate(app, false);
            const Craft& c1 = app.craft;
            Craft orig, copy;
            orig.comps.assign(c1.comps.begin(), c1.comps.begin() + c0.comps.size());
            copy.comps.assign(c1.comps.begin() + c0.comps.size(), c1.comps.end());
            std::vector<Component> e0 = expandCraft(orig), e1 = expandCraft(copy);
            std::string why;
            if (e0.size() != e1.size()) why = "count " + std::to_string(e0.size()) + " / " + std::to_string(e1.size());
            Vec3d shift = e1.empty() ? Vec3d{} : e1[0].pos - e0[0].pos;
            if (why.empty() && (std::fabs(shift.y) > 0 || std::fabs(shift.z) > 0 || shift.x < 0.99)) why = "offset";
            for (size_t i = 0; why.empty() && i < e0.size(); i++) {
                const Component &a = e0[i], &b = e1[i];
                if (!vecEq(b.pos - a.pos, shift, 1e-9)) why = "position of " + a.name;
                else if (!matEq(a.rot().toMat(), b.rot().toMat(), 1e-12)) why = "rotation of " + a.name;
                else if (a.shape == Shape::Airfoil && airfoilRoot(a) != airfoilRoot(b)) why = "wing root end of " + a.name;
                else if (retractableWheel(a)) {
                    Component sa = stowedWheel(a), sb = stowedWheel(b);
                    if (!vecEq(sb.pos - sa.pos, shift, 1e-9) || !matEq(sa.rot().toMat(), sb.rot().toMat(), 1e-12)) why = "gear fold of " + a.name;
                }
                if (why.empty()) {
                    Component x = b;
                    x.pos = a.pos;
                    x.rotDeg = a.rotDeg;
                    x.symmetry = a.symmetry;
                    x.rootSide = a.rootSide;
                    x.foldDir = a.foldDir;
                    if (!sameComp(a, x)) why = "fields of " + a.name;
                }
            }
            if (why.empty()) {
                CraftStats s0 = analyzeCraft(orig, nullptr), s1 = analyzeCraft(copy, nullptr);
                if (std::fabs(s0.wet.mass - s1.wet.mass) > 1e-9 * s0.wet.mass || !vecEq(s1.wet.com - s0.wet.com, shift, 1e-9) ||
                    !matEq(s0.wet.inertia, s1.wet.inertia, 1e-7 * std::max(1.0, s0.wet.principal.z)))
                    why = "mass properties";
            }
            // (the launch packing of radial solar panels is worked out about the craft axis: only logged)
            std::vector<Component> st0 = stowedGeometry(e0), st1 = stowedGeometry(e1);
            int stowDiff = 0;
            for (size_t i = 0; i < st0.size() && i < st1.size(); i++)
                if (!vecEq(st1[i].pos - st0[i].pos, shift, 1e-6)) stowDiff++;
            if (stowDiff) std::fprintf(T.log, "dup:   (info) %s: %d stowed solar panel(s) packed differently beside the axis\n", c0.name.c_str(), stowDiff);
            if (!why.empty()) std::fprintf(T.log, "dup:   %s: %s\n", c0.name.c_str(), why.c_str());
            else nOk++;
        }
        check(nOk == nCraft, "duplicate of all parts = the same craft moved by one offset (" + std::to_string(nOk) + " / " + std::to_string(nCraft) + " crafts)");
        // the user's case: a jet's wing pair (left and right panels) duplicated with the rest of the aircraft
        {
            Craft jet = craftNamed("Swift Jet Trainer");
            int wings = 0;
            for (const Component& k : expandCraft(jet)) wings += k.shape == Shape::Airfoil;
            std::fprintf(T.log, "dup: Swift Jet Trainer has %d wing panels (expanded)\n", wings);
        }
        // a component limit: a copy that does not fit is refused as a whole
        {
            Craft big;
            Component k = makeComponent(Func::Structure);
            for (int i = 0; i < 300; i++) {
                k.pos.z = i;
                big.comps.push_back(k);
            }
            cadSetCraft(app, big);
            std::vector<int> s;
            for (int i = 0; i < 300; i++) s.push_back(i);
            cadDebugSelect(s);
            cadDebugDuplicate(app, false);
            check(app.craft.comps.size() == 300, "a copy over the 400 limit is refused whole (" + std::to_string(app.craft.comps.size()) + " components)");
        }
        // ---- 2. Mirror X: twice = the original; the mirror keeps the shape's asymmetric local axes
        {
            int n = 0, ok2 = 0, ok1 = 0;
            for (const Craft& c0 : all)
                for (const Component& k : c0.comps) {
                    n++;
                    Component m1 = mirroredX(k), m2 = mirroredX(m1);
                    if (m2.pos.x == k.pos.x && m2.pos.y == k.pos.y && m2.pos.z == k.pos.z && matEq(m2.rot().toMat(), k.rot().toMat(), 1e-12) &&
                        m2.foldDir == k.foldDir && m2.rootSide == k.rootSide)
                        ok2++;
                    // the reflected axis: z for a wing (section), x for a fuselage, y otherwise; the other two map as S * R
                    int ax = k.shape == Shape::Airfoil ? 2 : (k.shape == Shape::Loft || k.func == Func::Window) ? 0 : 1;
                    Mat3d R = k.rot().toMat(), Rm = m1.rot().toMat();
                    bool good = true;
                    for (int a = 0; a < 3; a++) {
                        if (a == ax) continue;
                        Vec3d e{a == 0 ? 1.0 : 0.0, a == 1 ? 1.0 : 0.0, a == 2 ? 1.0 : 0.0};
                        Vec3d want = R * e;
                        want.x = -want.x;
                        good &= vecEq(Rm * e, want, 1e-12);
                    }
                    if (k.shape == Shape::Airfoil) good &= airfoilRoot(m1) == airfoilRoot(k); // (the same local end; the mirrored point)
                    ok1 += good;
                    if (!good) {
                        Mat3d A = R, B = Rm;
                        std::fprintf(T.log, "dup:   mirror axes of %s (%s, shape %d, rot %.6f %.6f %.6f -> %.6f %.6f %.6f, pos %.3f %.3f, root %d -> %d)\n", k.name.c_str(),
                                     c0.name.c_str(), (int)k.shape, k.rotDeg.x, k.rotDeg.y, k.rotDeg.z, m1.rotDeg.x, m1.rotDeg.y, m1.rotDeg.z, k.pos.x, k.pos.y,
                                     airfoilRoot(k), airfoilRoot(m1));
                        for (int i = 0; i < 3; i++)
                            std::fprintf(T.log, "dup:     %8.5f %8.5f %8.5f   %8.5f %8.5f %8.5f\n", A.m[i][0], A.m[i][1], A.m[i][2], B.m[i][0], B.m[i][1], B.m[i][2]);
                    }
                }
            check(ok2 == n, "mirror twice = the original (" + std::to_string(ok2) + " / " + std::to_string(n) + " components)");
            check(ok1 == n, "mirror keeps the sweep / upsweep / nose direction mirrored (" + std::to_string(ok1) + " / " + std::to_string(n) + ")");
        }
        // ---- 3. saving and reading: crafts and factory designs give every field back exactly
        {
            int n = 0, ok = 0;
            std::vector<Craft> set = all;
            // components with every field set to unusual values (within what the editor allows)
            Craft r;
            r.name = "試験機 Ünïcode 7";
            unsigned s = 12345;
            auto rnd = [&]() { s = s * 1664525u + 1013904223u; return (s >> 8) / 16777216.0; };
            for (int i = 0; i < 60; i++) {
                Component k = makeComponent((Func)(i % (int)Func::COUNT));
                k.name = i % 3 == 0 ? "主翼 " + std::to_string(i) : "Part " + std::to_string(i);
                k.shape = (Shape)(i % (int)Shape::COUNT);
                k.dims = {0.01 + rnd() * 30, 0.01 + rnd() * 30, 0.01 + rnd() * 30};
                k.pos = {(rnd() - 0.5) * 3000, (rnd() - 0.5) * 3000, (rnd() - 0.5) * 3000};
                k.rotDeg = {(rnd() - 0.5) * 720, (rnd() - 0.5) * 720, (rnd() - 0.5) * 720};
                k.material = i % MATERIAL_COUNT;
                k.hollow = i % 2;
                k.wall = 0.0001 + rnd() * 0.5;
                k.cluster = 1 + i % 37;
                k.fill = rnd();
                k.stage = i % 32;
                k.fireFrom = i % 33 - 1;
                k.symmetry = 1 + i % 12;
                k.color = Color{(unsigned char)(rnd() * 255), (unsigned char)(rnd() * 255), (unsigned char)(rnd() * 255), 255};
                k.massOverride = i % 4 == 0 ? rnd() * 1e5 : 0;
                k.taper = 0.1 + rnd() * 0.9;
                k.sweep = (rnd() - 0.5) * 140;
                k.noseLen = 0.02 + rnd() * 0.58;
                k.tailLen = 0.02 + rnd() * 0.68;
                k.tailR = rnd();
                k.upsweep = rnd() * 2 - 1;
                k.bluff = rnd();
                k.boxy = rnd();
                k.dockTag = i * 7;
                k.dockedTo = i * 3;
                k.origin = i;
                k.stage0 = i % 33 - 1;
                k.engOff = (uint64_t)(rnd() * 1e9) & ((1ull << 37) - 1);
                k.rootSide = i % 3 - 1;
                k.foldDir = i % 4;
                r.comps.push_back(k);
            }
            set.push_back(r);
            for (const Craft& c0 : set) {
                std::string t1 = craftToText(c0);
                Craft c1;
                bool read = craftFromText(c1, t1);
                bool same = read && c1.name == c0.name && c1.comps.size() == c0.comps.size();
                for (size_t i = 0; same && i < c0.comps.size(); i++) same = sameComp(c0.comps[i], c1.comps[i]);
                same = same && craftToText(c1) == t1;
                n++;
                ok += same;
                if (!same) {
                    std::fprintf(T.log, "dup:   save + read differs: %s\n", c0.name.c_str());
                    for (size_t i = 0; read && i < c0.comps.size() && i < c1.comps.size(); i++)
                        if (!sameComp(c0.comps[i], c1.comps[i])) {
                            Craft a, b;
                            a.comps = {c0.comps[i]};
                            b.comps = {c1.comps[i]};
                            std::string ta = craftToText(a), tb = craftToText(b);
                            std::fprintf(T.log, "dup:     before %s     after  %s", ta.substr(ta.find("comp")).c_str(), tb.substr(tb.find("comp")).c_str());
                            break;
                        }
                }
            }
            check(ok == n, "craft save + read gives every field back (" + std::to_string(ok) + " / " + std::to_string(n) + " crafts, Japanese names)");
            int dn = 0, dok = 0;
            std::vector<Design> ds = builtinTemplates();
            Design jd = ds[0];
            jd.name = "ロケット 改 (2)";
            jd.stack[0].diam = 1.2345678901234;
            ds.push_back(jd);
            for (const Design& d0 : ds) {
                std::string t1 = designToText(d0);
                Design d1;
                bool same = designFromText(d1, t1) && d1.name == d0.name && d1.stack.size() == d0.stack.size();
                for (size_t i = 0; same && i < d0.stack.size(); i++) {
                    const PartInst &a = d0.stack[i], &b = d1.stack[i];
                    same = a.def == b.def && a.diam == b.diam && a.diam2 == b.diam2 && a.length == b.length && a.cluster == b.cluster &&
                           a.radial.size() == b.radial.size();
                    for (size_t j = 0; same && j < a.radial.size(); j++)
                        same = a.radial[j].def == b.radial[j].def && a.radial[j].count == b.radial[j].count && a.radial[j].offset == b.radial[j].offset &&
                               a.radial[j].angle == b.radial[j].angle;
                }
                dn++;
                dok += same;
                if (!same) std::fprintf(T.log, "dup:   design save + read differs: %s\n", d0.name.c_str());
            }
            check(dok == dn, "design save + read gives every field back (" + std::to_string(dok) + " / " + std::to_string(dn) + " designs)");
            check(cleanLineText("a\x01" "b\xff\xfe" "c\xe3\x81" "d\xc0\xaf" "e\xe2\x80\xa8" "f") == "abcdef", "names: control and broken UTF-8 bytes dropped");
            check(cleanLineText(std::string(30, 'x') + "あいうえおか", 48) == std::string(30, 'x') + "あいうえおか" &&
                      cleanLineText(std::string(47, 'x') + "あ", 48) == std::string(47, 'x'), "names: cut at a character boundary");
        }
        // ---- 4. adding a design: on top of the current one, symmetry kept, the added parts selected
        {
            Craft base = craftNamed("Swift Jet Trainer"), add = all[0];
            for (const Craft& c : all)
                for (const Component& k : c.comps)
                    if (k.symmetry > 1) { add = c; goto found; }
        found:
            cadSetCraft(app, base);
            bool okImp = cadDebugImport(app, add);
            const Craft& c = app.craft;
            bool cnt = c.comps.size() == base.comps.size() + add.comps.size();
            double top = -1e30, bottom = 1e30, dz = 0;
            std::vector<Vec3d> pts;
            for (const Component& k : expandCraft(base)) { pts.clear(); outlinePoints(k, pts, 12); for (auto& p : pts) top = std::max(top, p.z); }
            Craft added;
            added.comps.assign(c.comps.begin() + base.comps.size(), c.comps.end());
            for (const Component& k : expandCraft(added)) { pts.clear(); outlinePoints(k, pts, 12); for (auto& p : pts) bottom = std::min(bottom, p.z); }
            bool same = cnt;
            for (size_t i = 0; same && i < add.comps.size(); i++) {
                const Component &a = add.comps[i], &b = added.comps[i];
                if (i == 0) dz = b.pos.z - a.pos.z;
                Component x = b;
                x.pos = a.pos;
                same = sameComp(a, x) && b.pos.x == a.pos.x && b.pos.y == a.pos.y && std::fabs(b.pos.z - a.pos.z - dz) < 1e-9;
            }
            std::vector<int> sel = cadDebugSelection();
            bool selOk = (int)sel.size() == (int)add.comps.size() && !sel.empty() && sel.front() == (int)base.comps.size();
            check(okImp && same && std::fabs(top - bottom) < 1e-6 && selOk,
                  "add " + add.name + " to " + base.name + ": parts unchanged but moved " + std::to_string(dz) + " m up, flush on top (gap " +
                      std::to_string(bottom - top) + " m), selected");
            Craft big;
            for (int i = 0; i < 390; i++) big.comps.push_back(makeComponent(Func::Structure));
            size_t before = app.craft.comps.size();
            check(!cadDebugImport(app, big) && app.craft.comps.size() == before, "a design that does not fit under the limit is refused");
            cadSetCraft(app, base);
            cadDebugImport(app, add);
            app.screen = Screen::CAD;
            cadDebugHome(app);
        }
        return true;
    }
    if (f == 12) shot("29_dup_import.png");
    if (f == 14) {
        // the user's path: the jet, all selected, Duplicate
        Craft jet = craftNamed("Swift Jet Trainer");
        cadSetCraft(app, jet);
        std::vector<int> s;
        for (int i = 0; i < (int)jet.comps.size(); i++) s.push_back(i);
        cadDebugSelect(s);
        cadDebugDuplicate(app, false);
        cadDebugHome(app);
    }
    if (f == 24) shot("29_dup_jet.png");
    if (f == 26) {
        // a wing panel's properties: the root end row
        int w = -1;
        for (int i = 0; i < (int)app.craft.comps.size(); i++)
            if (app.craft.comps[i].shape == Shape::Airfoil) { w = i; break; }
        cadDebugSet(w, 0);
    }
    if (f == 34) shot("29_dup_wingprops.png");
    if (f == 36) cadDebugOpenDialog(app, 1);
    if (f == 44) {
        shot("29_dup_adddialog.png");
        cadDebugOpenDialog(app, -1);
    }
    // ---- 5. the factory: saving over a saved design asks first (designs written under the test directory)
    static std::string path, dir;
    if (f == 46) {
        dir = T.dir + "\\designs";
        std::filesystem::create_directories(dir);
        app.designDir = dir;
        app.design = builtinTemplates()[0];
        app.design.name = "Overwrite test";
        path = dir + "\\" + sanitizeFileName(app.design.name) + ".sfd";
        std::filesystem::remove(path);
        app.screen = Screen::Factory;
        factoryDebugSave(app);
        check(!factoryDebugOverwriteOpen() && std::filesystem::exists(path), "first save: no question, file written");
        app.design.stack.pop_back();
        factoryDebugSave(app);
        check(factoryDebugOverwriteOpen(), "save again under that name: asks");
    }
    if (f == 54) shot("29_dup_overwrite.png");
    if (f == 56) factoryDebugOverwriteChoice(0);
    if (f == 58) {
        Design d;
        check(!factoryDebugOverwriteOpen() && loadDesign(d, path) && d.stack.size() == app.design.stack.size() + 1, "cancel: the saved design is unchanged");
        factoryDebugSave(app);
        factoryDebugOverwriteChoice(2);
    }
    if (f == 60) {
        Design d;
        check(!factoryDebugOverwriteOpen() && loadDesign(d, path) && d.stack.size() == app.design.stack.size(), "overwrite: the saved design replaced");
        factoryDebugSave(app);
        factoryDebugOverwriteChoice(1);
    }
    if (f == 62) {
        Design d;
        std::string p2 = dir + "\\" + sanitizeFileName("Overwrite test (2)") + ".sfd";
        check(!factoryDebugOverwriteOpen() && app.design.name == "Overwrite test (2)" && loadDesign(d, p2) && d.name == "Overwrite test (2)",
              "new name: saved as \"" + app.design.name + "\" beside it");

        // ---- 6. a Japanese name: the file has that name, the open list shows the names as saved
        namespace fs = std::filesystem;
        auto has = [](const std::vector<std::string>& v, const std::string& s) { return std::find(v.begin(), v.end(), s) != v.end(); };
        const std::string jp = "\xE8\xA9\xA6\xE9\xA8\x93\xE6\xA9\x9F \xE3\x81\x82"; // (shiken-ki a)
        std::string pj = dir + "\\" + jp + ".sfd";
        fs::remove(fs::path(pj));
        app.design = builtinTemplates()[1];
        app.design.name = jp;
        factoryDebugSave(app);
        check(sanitizeFileName(jp) == jp && !factoryDebugOverwriteOpen() && fs::exists(fs::path(dir) / fs::path(L"\u8A66\u9A13\u6A5F \u3042.sfd")),
              "a Japanese name: the file is named in Japanese");
        Design dj;
        check(loadDesign(dj, pj) && dj.name == jp && designToText(dj) == designToText(app.design), "the Japanese-named file reads back");
        factoryDebugSave(app);
        check(factoryDebugOverwriteOpen(), "the Japanese name saved again: asks");
        factoryDebugOverwriteChoice(0);
        // a file an older version wrote (ASCII + hash) is renamed to the name it holds
        const std::string jp2 = "\xE6\x97\xA7\xE5\x90\x8D\xE5\x89\x8D"; // (kyuu namae)
        std::string legacy = dir + "\\" + legacyFileName(jp2) + ".sfd", pj2 = dir + "\\" + jp2 + ".sfd";
        fs::remove(fs::path(pj2));
        Design d2 = builtinTemplates()[2];
        d2.name = jp2;
        saveDesign(d2, legacy);
        std::vector<std::string> names = factoryDebugSavedNames(app);
        std::string all;
        for (auto& n : names) all += "[" + n + "]";
        std::fprintf(T.log, "dup: saved designs listed: %s\n", all.c_str());
        check(has(names, jp) && has(names, jp2) && has(names, "Overwrite test") && has(names, "Overwrite test (2)") && !has(names, "Overwrite_test"),
              "the open list shows the names as saved");
        Design dl;
        check(!fs::exists(fs::path(legacy)) && loadDesign(dl, pj2) && designToText(dl) == designToText(d2), "an older file is renamed to its name");
        // a hand-named file keeps its file name (only files named the old way are renamed)
        std::string hand = dir + "\\my file.sfd";
        saveDesign(d2, hand);
        names = factoryDebugSavedNames(app);
        check(fs::exists(fs::path(hand)), "a file named by hand is left as it is");
        // CAD: the same for a craft
        Craft cj;
        cj.name = jp2;
        std::string pc = dir + "\\" + sanitizeFileName(cj.name) + ".sfc";
        check(saveCraft(cj, pc) && fs::exists(fs::path(dir) / fs::path(L"\u65E7\u540D\u524D.sfc")), "CAD: a Japanese name is the file name");
        bool listed = false;
        for (auto& s : listSavedDesigns(dir, ".sfc", 300)) listed = listed || (s.name == jp2 && s.path == pc);
        check(listed, "CAD: the open list shows the name as saved");
        // names Windows cannot take as a file name: kept apart by a hash, never a device or a path
        std::set<std::string> outs;
        const char* odd[] = {"CON", "nul", "Com1", "LPT\xC2\xB9", "a/b", "a\\b", "x:y", "a*b?", "<>|\"", "..", "  ", "", "Rocket.", " Rocket",
                             "\xE2\x80\xAE" "evil", "a\xE2\x80\x8B" "b"};
        bool safe = true;
        for (const char* s : odd) {
            std::string n = sanitizeFileName(s);
            std::string u = n;
            for (char& c : u) c = (char)std::toupper((unsigned char)c);
            bool ok = !n.empty() && n.find_first_of("<>:\"/\\|?*.") == std::string::npos && n.front() != ' ' && n.back() != ' ' &&
                      n.find("\xE2\x80\xAE") == std::string::npos && n.find("\xE2\x80\x8B") == std::string::npos &&
                      u != "CON" && u != "NUL" && u != "COM1" && n != "LPT\xC2\xB9";
            if (!ok) std::fprintf(T.log, "dup: unsafe file name for \"%s\": \"%s\"\n", s, n.c_str());
            safe = safe && ok;
            outs.insert(n);
        }
        check(safe, "names Windows cannot take: a safe file name");
        check(outs.size() == sizeof odd / sizeof odd[0], "different names: different files");
        factoryDebugOpenList(app, true);
    }
    if (f == 68) shot("29_dup_openlist.png");
    if (f == 70) {
        factoryDebugOpenList(app, false);
        std::fprintf(T.log, "duptest: %s (%d failed)\n", bad ? "FAIL" : "all ok", bad);
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ achievements + engine selection (--achtest)
// A vessel launched from the pad is moved through the milestones (lift-off, space, low Earth orbit, the Moon:
// orbit, landing, back on Earth): the achievements must come from the state alone. A vessel flown with
// infinite fuel and a rover started on Mars must earn nothing. Save + load keeps the map's achievements and
// the vessels' records. Then the engine selection: half the nozzles of the Kestrel's first stage off halves
// its thrust.
bool achTestStep(App& app) {
    World& W = app.world;
    const int f = T.frames;
    static int bad = 0;
    auto check = [&](bool ok, const char* what) {
        std::fprintf(T.log, "ach: %s -> %s\n", what, ok ? "ok" : "FAIL");
        std::fflush(T.log);
        if (!ok) bad++;
    };
    auto has = [&](const char* id) { return W.achievements.count(id) > 0; };
    int earth = W.U.find("Earth"), moon = W.U.find("Moon"), mars = W.U.find("Mars");
    auto orbitAt = [&](Vessel& v, int body, double alt) {
        const Body& b = W.U.bodies[body];
        v.ref = body;
        v.landed = false;
        v.r = Vec3d{0.3, 0.9, 0.2};
        v.r = norm(v.r) * (b.radius + alt);
        Vec3d e = norm(cross(Vec3d{0, 0, 1}, v.r));
        v.v = e * std::sqrt(b.GM / len(v.r));
        v.q = Quatd::lookRot(norm(v.v), norm(v.r));
        v.w = {};
        v.throttle = 0;
        for (int k = 0; k < 4; k++) W.update(0.05);
    };
    auto landOn = [&](Vessel& v, int body, Vec3d dir) {
        placeAt(W, v, body, dir, 0.4);
        for (int k = 0; k < 400 && !v.landed && !v.destroyed; k++) W.update(0.02);
        for (int k = 0; k < 4; k++) W.update(0.05);
    };
    if (f == 2) {
        bad = 0;
        W.newGame(Settings{});
        app.gameStarted = true;
        W.mapName = "Achievement test";
        sunAt(W, earth, siteDir(0, 0), 30);
        app.lastLaunched = craftFromDesign(builtinTemplates()[0]);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        for (int k = 0; k < 10; k++) W.update(0.05);
        check(W.achievements.empty(), "nothing on the pad");
        Vessel& v = *W.activeVessel();
        check(v.earthLaunch && !v.cheated && v.countsForRecords(), "a pad launch counts");
        placeAt(W, v, earth, siteDir(28, -80), 2000);
        for (int k = 0; k < 4; k++) W.update(0.05);
        check(has("liftoff") && !has("space"), "lift-off at 2 km");
        orbitAt(v, earth, 300e3);
        check(has("space") && has("low:Earth"), "space and low Earth orbit");
        orbitAt(v, moon, 100e3);
        check(has("visit:Moon") && has("low:Moon"), "the Moon: reached, low orbit");
        landOn(v, moon, siteDir(0.7, 23.5));
        check(v.landed && !v.destroyed && has("land:Moon"), "landing on the Moon");
        check(!has("return:Moon"), "no return yet");
        landOn(v, earth, siteDir(30, -40));
        check(v.landed && !v.destroyed && has("return:Moon") && has("return:space"), "back on Earth from the Moon");
        size_t n0 = W.achievements.size();
        // fun mode: infinite fuel taints what flies while it is on
        W.settings.infiniteFuel = true;
        W.update(0.05);
        W.settings.infiniteFuel = false;
        check(v.cheated, "infinite fuel marks the vessel");
        orbitAt(v, mars, 400e3);
        check(!has("visit:Mars") && !has("low:Mars"), "a vessel flown with infinite fuel earns nothing");
        // a rover started on Mars
        W.launchAircraft(craftNamed("Mars Rover"), mars);
        for (int k = 0; k < 10; k++) W.update(0.05);
        check(!W.activeVessel()->earthLaunch && !has("land:Mars") && !has("visit:Mars"), "a rover started on Mars earns nothing");
        // a start in orbit
        W.launchInOrbit(craftNamed("Crew Ferry"), -1, 0);
        for (int k = 0; k < 10; k++) W.update(0.05);
        check(!W.activeVessel()->earthLaunch && W.achievements.size() == n0, "a start in orbit earns nothing");
        // save + load
        std::string path = T.dir + "\\ach_test.sfs";
        W.vessels[0].hidden = false;
        W.vessels[0].showOrbit = false;
        bool saved = writeFileAtomic(path, W.serialize());
        std::map<std::string, World::Achieved> before = W.achievements;
        uint64_t lo = W.vessels[0].landedOn;
        bool loaded = loadWorld(W, path);
        bool same = loaded && W.achievements.size() == before.size();
        for (auto& e : before) same = same && W.achievements.count(e.first) && W.achievements[e.first].vessel == e.second.vessel;
        check(saved && same, "save + load keeps the achievements");
        check(loaded && !W.vessels.empty() && W.vessels[0].landedOn == lo && W.vessels[0].earthLaunch && !W.vessels[0].showOrbit,
              "save + load keeps the vessel's record and view flags");
        std::fprintf(T.log, "ach: %zu of %zu earned:", W.achievements.size(), ach::list(W.U).size());
        for (auto& e : W.achievements) std::fprintf(T.log, " %s", e.first.c_str());
        std::fprintf(T.log, "\n");
        flightOnEnter(app);
        W.newAchievements = {"land:Moon"};
    }
    if (f == 12) shot("90_ach_banner.png");
    if (f == 13) flightDebugAchievements(true);
    if (f == 16) shot("91_ach_list.png");
    if (f == 17) {
        flightDebugAchievements(false);
        // engine selection: the Kestrel on the pad, full throttle, all nine engines and then four of them off
        W.newGame(Settings{});
        app.lastLaunched = craftFromDesign(builtinTemplates()[0]);
        W.launch(app.lastLaunched);
        flightOnEnter(app);
        Vessel& v = *W.activeVessel();
        W.stage(v);
        v.throttle = 1;
        for (int k = 0; k < 6; k++) W.update(0.02);
        double F9 = W.activeVessel()->tThrust;
        Vessel& v2 = *W.activeVessel();
        int eng = -1;
        for (int i = 0; i < (int)v2.comps.size(); i++)
            if (v2.comps[i].func == Func::Engine && engineCount(v2.comps[i]) == 9) eng = i;
        if (eng >= 0) v2.comps[eng].engOff = 0x1E; // nozzles 1-4 off (the ring), the centre and four others run
        for (int k = 0; k < 6; k++) W.update(0.02);
        double F5 = W.activeVessel()->tThrust;
        std::fprintf(T.log, "ach: engines: thrust with 9 engines %.0f kN, with 5 of 9 %.0f kN (ratio %.3f)\n", F9 / 1000, F5 / 1000, F9 > 0 ? F5 / F9 : 0);
        check(eng >= 0 && F9 > 0 && std::fabs(F5 / F9 - 5.0 / 9.0) < 0.03, "four of nine engines off: 5/9 of the thrust");
        check(enginesOn(W.activeVessel()->comps[eng]) == 5, "5 engines counted as running");
        v2.throttle = 0.6;
        flightDebugEngines(true, -1);
    }
    if (f == 21) shot("92_engines_panel.png");
    if (f == 22) {
        // the selection is saved with the vessel
        Vessel& v = *W.activeVessel();
        int eng = -1;
        for (int i = 0; i < (int)v.comps.size(); i++)
            if (v.comps[i].func == Func::Engine && engineCount(v.comps[i]) == 9) eng = i;
        std::string path = T.dir + "\\eng_test.sfs";
        bool ok = writeFileAtomic(path, W.serialize()) && loadWorld(W, path);
        int eng2 = -1;
        if (ok && W.activeVessel())
            for (int i = 0; i < (int)W.activeVessel()->comps.size(); i++)
                if (W.activeVessel()->comps[i].func == Func::Engine && engineCount(W.activeVessel()->comps[i]) == 9) eng2 = i;
        check(ok && eng >= 0 && eng2 >= 0 && W.activeVessel()->comps[eng2].engOff == 0x1E, "the engine selection is saved");
        flightDebugEngines(false, -1);
        flightOnEnter(app);
        std::fprintf(T.log, "ach: %s\n", bad == 0 ? "achievements and engine selection -> ok" : "FAIL");
        return false;
    }
    return true;
}

bool dockTestStep(App& app) {
    World& W = app.world;
    static int coreId = -1, nodeId = -1, labId = -1, docks = 0;
    static double d0 = 0, tRef = 0, mom = 0;
    auto fin = [&](const char* verdict) {
        std::fprintf(T.log, "dock: %s\n", verdict);
        shot("28_dock_end.png");
        flightDebugInput(nullptr);
        return false;
    };
    if (T.frames == 2) {
        W.newGame(Settings{});
        app.gameStarted = true;
        int earth = W.U.find("Earth");
        sunAt(W, earth, siteDir(0, 0), 30);
        W.launchInOrbit(craftNamed("Station Core Module"), -1, 0);
        coreId = W.activeVessel()->id;
        W.launchInOrbit(craftNamed("Station Node Module"), coreId, 60);
        nodeId = W.activeVessel()->id;
        app.screen = Screen::Flight;
        flightOnEnter(app);
        flightDebugView(2.2, 0.35, 45, Vec3d{});
        const Vessel* c = vesselById(W, coreId);
        const Vessel* n = vesselById(W, nodeId);
        d0 = len(c->r - n->r);
        tRef = W.t;
        T.tfSub = 0;
        docks = 0;
        std::fprintf(T.log, "dock: core %.1f t, node %.1f t, %.1f m apart at %.0f km\n", W.vesselMass(*c) / 1000, W.vesselMass(*n) / 1000, d0,
                     (len(c->r) - W.U.bodies[c->ref].radius) / 1000);
        return true;
    }
    if (T.frames < 4) return true;
    Vessel* a = W.activeVessel();
    if (!a) return fin("FAIL (no vessel)");
    if (T.tfSub == 0) {
        // drift of two free vessels side by side: 300 s at x1, then 600 s at x10
        for (int k = 0; k < 20; k++) W.update(0.05);
        if (W.t - tRef > 300) {
            const Vessel* c = vesselById(W, coreId);
            double d1 = len(c->r - a->r);
            std::fprintf(T.log, "dock: after 300 s at x1 the modules are %.2f m apart (drift %.3f m)\n", d1, d1 - d0);
            d0 = d1;
            tRef = W.t;
            W.setWarp(3);
            T.tfSub = 1;
        }
        return true;
    }
    if (T.tfSub == 1) {
        for (int k = 0; k < 8; k++) W.update(0.05);
        if (W.t - tRef > 600) {
            W.setWarp(0);
            const Vessel* c = vesselById(W, coreId);
            double d1 = len(c->r - a->r);
            std::fprintf(T.log, "dock: after 600 s of warp (x%.0f) %.2f m apart (drift %.3f m)\n", W.effectiveWarp, d1, d1 - d0);
            T.tfSub = 2;
            tRef = W.t;
            shot("28_dock_approach.png");
        }
        return true;
    }
    if (T.tfSub == 2 || T.tfSub == 5 || T.tfSub == 7) {
        // approach and dock: the node to the core (2), again after undocking (5), the laboratory to the station (7)
        int tgt = T.tfSub == 7 ? nodeId : coreId;
        const Vessel* b = nullptr;
        for (const Vessel& v : W.vessels)
            if (!v.destroyed && v.id != a->id && (v.id == tgt || T.tfSub == 7)) b = &v;
        static int lastParts = 0;
        if (!b && lastParts > 0 && (int)a->comps.size() > lastParts) {
            // (it latched between two test frames, in the flight screen's own update)
            docks++;
            std::fprintf(T.log, "dock: docked (%d): %d parts, %.1f t, %s\n", docks, (int)a->comps.size(), W.vesselMass(*a) / 1000, a->name.c_str());
            lastParts = 0;
            flightDebugInput(nullptr);
            W.input = {};
            tRef = W.t;
            T.tfSub = T.tfSub == 2 ? 3 : T.tfSub == 5 ? 6 : 8;
            return true;
        }
        if (!b) {
            for (const Vessel& v : W.vessels)
                std::fprintf(T.log, "  vessel %d %s: %d parts, destroyed %d (%s), debris %d, active %d\n", v.id, v.name.c_str(), (int)v.comps.size(),
                             (int)v.destroyed, v.fate.c_str(), (int)v.debris, (int)(&v == a));
            return fin("FAIL (no target)");
        }
        double cl = 0, gap = 0;
        int before = (int)a->comps.size();
        double mBefore = W.vesselMass(*a) + W.vesselMass(*b);
        Vec3d pBefore;
        for (int k = 0; k < 8; k++) {
            gap = approachStep(W, *a, *b, cl);
            // momentum of the two before this step, plus what gravity adds over it (the same pull on both)
            double rr = len(a->r);
            Vec3d g = -a->r * (W.U.bodies[a->ref].GM / (rr * rr * rr));
            pBefore = a->v * W.vesselMass(*a) + b->v * W.vesselMass(*b) + g * (mBefore * 0.05);
            W.update(0.05);
            a = W.activeVessel();
            if ((int)a->comps.size() > before) break;
        }
        if (T.frames % 40 == 0) std::fprintf(T.log, "  approach: port gap %.2f m (along %.2f, off-axis %.2f, angle %.1f deg), closing %.3f m/s, RCS %.1f kg\n", gap,
                                             g_apAlong, g_apLat, g_apAng, cl, a->monoAmt);
        if ((int)a->comps.size() > before) {
            docks++;
            double m = W.vesselMass(*a);
            Vec3d p = a->v * m;
            std::fprintf(T.log, "dock: docked (%d): %d parts, %.1f t (before %.1f t), momentum change %.3f m/s x mass, %s\n", docks, (int)a->comps.size(),
                         m / 1000, mBefore / 1000, len(p - pBefore) / std::max(m, 1.0), a->name.c_str());
            char nm[64];
            std::snprintf(nm, sizeof nm, "28_dock_docked_%d.png", docks);
            shot(nm);
            flightDebugInput(nullptr);
            W.input = {};
            tRef = W.t;
            T.tfSub = T.tfSub == 2 ? 3 : T.tfSub == 5 ? 6 : 8;
            return true;
        }
        if (W.t - tRef > 1800) return fin("FAIL (no docking within 30 minutes)");
        lastParts = (int)a->comps.size();
        return true;
    }
    if (T.tfSub == 3) {
        // undock: the lighter part (the node) leaves and is flown on
        for (int k = 0; k < 20; k++) W.update(0.05);
        if (W.t - tRef < 10) return true;
        int port = -1;
        for (int i = 0; i < (int)a->comps.size(); i++)
            if (a->comps[i].func == Func::Docking && a->comps[i].dockedTo) port = i;
        if (port < 0 || !W.undock(*a, port)) return fin("FAIL (undocking refused)");
        a = W.activeVessel();
        nodeId = a->id;
        for (const Vessel& v : W.vessels)
            if (!v.destroyed && v.id != a->id && !v.debris) coreId = v.id;
        tRef = W.t;
        T.tfSub = 4;
        std::fprintf(T.log, "dock: undocked; now flying %s (%d parts)\n", a->name.c_str(), (int)a->comps.size());
        for (const Vessel& v : W.vessels)
            std::fprintf(T.log, "  vessel %d %s: %d parts, destroyed %d (%s), debris %d, %.1f m from the active\n", v.id, v.name.c_str(), (int)v.comps.size(),
                         (int)v.destroyed, v.fate.c_str(), (int)v.debris, len(v.r - a->r));
        return true;
    }
    if (T.tfSub == 4) {
        for (int k = 0; k < 20; k++) W.update(0.05);
        if (W.t - tRef > 30) {
            const Vessel* c = vesselById(W, coreId);
            if (!c) {
                for (const Vessel& v : W.vessels)
                    std::fprintf(T.log, "  vessel %d %s: %d parts, destroyed %d (%s), debris %d\n", v.id, v.name.c_str(), (int)v.comps.size(), (int)v.destroyed,
                                 v.fate.c_str(), (int)v.debris);
                return fin("FAIL (the other part is gone)");
            }
            Vec3d pa, da, pb, db;
            double sep = len(c->r - a->r);
            std::fprintf(T.log, "dock: 30 s after undocking: %.2f m between the centres, relative speed %.3f m/s\n", sep, len(c->v - a->v));
            // back off 8 m along the port axis with the RCS, then dock again
            tRef = W.t;
            T.tfSub = 5;
            (void)pa; (void)da; (void)pb; (void)db;
        }
        return true;
    }
    if (T.tfSub == 6) {
        // the laboratory, delivered behind the station
        for (int k = 0; k < 20; k++) W.update(0.05);
        if (W.t - tRef < 10) return true;
        int stationId = a->id;
        W.launchInOrbit(craftNamed("Station Laboratory Module"), stationId, 60);
        a = W.activeVessel();
        labId = a->id;
        nodeId = stationId;
        tRef = W.t;
        T.tfSub = 7;
        return true;
    }
    if (T.tfSub == 8) {
        for (int k = 0; k < 20; k++) W.update(0.05);
        if (W.t - tRef < 10) return true;
        int ports = 0, used = 0;
        for (const Component& k : a->comps)
            if (k.func == Func::Docking) { ports++; used += k.dockedTo != 0; }
        flightDebugView(2.0, 0.45, 70, Vec3d{});
        T.tfSub = 9;
        tRef = W.t;
        std::fprintf(T.log, "dock: station of %d parts, %.1f t, %d of %d ports joined\n", (int)a->comps.size(), W.vesselMass(*a) / 1000, used, ports);
        return true;
    }
    if (T.tfSub == 9) {
        W.update(0.05);
        if (W.t - tRef > 2) {
            shot("28_dock_station.png");
            // save / load round trip of the station, then the laboratory leaves again with its own name
            std::string path = T.dir + "\\dockstation.sfs";
            bool rt = saveWorld(W, path) && loadWorld(W, path);
            Vessel* st = W.activeVessel();
            bool named = false;
            if (rt && st) {
                std::string host = st->name;
                int port = -1;
                for (int i = 0; i < (int)st->comps.size(); i++)
                    if (st->comps[i].func == Func::Docking && st->comps[i].dockedTo != 0 && st->comps[i].origin == labId) port = i;
                size_t nBefore = W.vessels.size();
                bool ud = port >= 0 && W.undock(*st, port);
                std::string leftName = ud && W.vessels.size() > nBefore ? W.vessels.back().name : "";
                named = ud && host == "Station Core Module" && leftName == "Station Laboratory Module";
                std::fprintf(T.log, "dock: after save/load the station is \"%s\"; undocked \"%s\" -> %s\n", host.c_str(), leftName.c_str(),
                             named ? "ok" : "FAIL");
            } else std::fprintf(T.log, "dock: save/load of the station -> FAIL\n");
            return fin(docks == 3 && named ? "docked, undocked, docked again and added a third module -> ok" : "FAIL");
        }
        return true;
    }
    (void)labId;
    (void)mom;
    return true;
}

bool ctrlTestStep(App& app) {
    World& W = app.world;
    // mode 0: in orbit with the last stage only, 1: climbing under power through 3 km, 2: aeroplane in level
    // flight, 3: in orbit with the RCS only (no wheels), 4: helicopter hovering at 60 m
    struct Case { const char* name; int mode; };
    static const Case C[] = {{"Ion Explorer", 0}, {"Kestrel Orbiter", 0}, {"Aurora Crew Mission", 0}, {"Kestrel Orbiter", 1},
                             {"Aurora Crew Mission", 1}, {"Swift Jet Trainer", 2}, {"Kestrel Probe", 3}, {"Kestrel Helicopter", 4}};
    const int n = sizeof C / sizeof C[0];
    int ci = T.frames / 10;
    if (ci >= n) return false;
    if (T.frames % 10 != 2) return true;
    const Case& c = C[ci];
    W.newGame(Settings{});
    app.gameStarted = true;
    bool craft = c.mode == 2 || c.mode == 4;
    if (craft) {
        Craft k = craftNamed(c.name);
        app.lastLaunched = k;
        W.launchAircraft(k, W.U.find("Earth"));
    } else if (c.mode == 3) {
        tut::start(app, 3); // the "Operating in space" probe: RCS thrusters and a hydrazine tank, in orbit
        tut::stop();
    } else {
        for (const Design& d : builtinTemplates())
            if (d.name == c.name) {
                app.design = d;
                app.lastLaunched = craftFromDesign(d);
                W.launch(app.lastLaunched);
            }
    }
    app.screen = Screen::Flight;
    flightOnEnter(app);
    Vessel* v = W.activeVessel();
    if (!v) return false;
    const Body& b = W.U.bodies[v->ref];
    if (craft) {
        // aeroplane: level flight at 3 km, 150 m/s along the nose; helicopter: hovering at 60 m (rotor up)
        Vec3d up = norm(v->r), nose = v->q.rotate({0, 0, 1});
        v->landed = false;
        v->rolling = false;
        v->r = up * (len(v->r) + (c.mode == 2 ? 3000 : 60));
        v->v = cross(W.U.angVel(v->ref), v->r) + (c.mode == 2 ? nose * 150.0 : Vec3d{});
        v->throttle = c.mode == 2 ? 0.7 : 0.56;
        v->spool = v->throttle;
        v->sas = SAS::Stability;
        W.input = {};
        for (int k = 0; k < 180; k++) W.update(1 / 60.0);
    } else if (c.mode == 1) {
        // lift-off and a vertical climb under full power to 3 km
        v->throttle = 1;
        v->sas = SAS::Stability;
        W.stage(*v);
        for (int k = 0; k < 60 * 120 && len(v->r) - b.radius < 3000 && !v->destroyed; k++) W.update(1 / 60.0);
    } else if (c.mode == 3) {
        v->torque = 0; // (no reaction wheels: the RCS alone)
        v->rcs = true;
        v->ignited = false;
        v->throttle = 0;
    } else {
        W.placeInOrbit(*v, 300e3);
        v = W.activeVessel();
        for (int k = 0; k < 12 && vesselStages(*v).size() > 1; k++) W.stage(*v); // down to the last stage
        v->ignited = false;
        v->throttle = 0;
    }
    if (c.mode == 3 && v->rcsNoz.empty()) {
        std::fprintf(T.log, "ctrl: %s has no RCS for the RCS-only case -> FAIL\n", c.name);
        return true;
    }
    double I = v->mp.inertia.m[0][0];
    const char* axName[3] = {"pitch", "yaw", "roll"};
    for (int axis = 0; axis < (craft ? 3 : 1); axis++) {
        v->w = {};
        v->sas = c.mode == 0 || c.mode == 3 ? SAS::Off : SAS::Stability;
        Quatd q0 = v->q;
        double r25 = 0, r50 = 0, r100 = 0, rMax = 0;
        for (int k = 1; k <= 60; k++) {
            ControlInput in;
            (axis == 0 ? in.pitch : axis == 1 ? in.yaw : in.roll) = 1;
            W.input = in;
            W.update(1 / 60.0);
            int bx = c.mode == 4 && axis > 0 ? 3 - axis : axis; // (a rotorcraft: yaw about its Z axis, roll about Y)
            double rate = (bx == 0 ? v->w.x : bx == 1 ? v->w.y : v->w.z) / DEG; // signed: + is the commanded way
            if (k == 15) r25 = rate;
            if (k == 30) r50 = rate;
            if (k == 60) r100 = rate;
            rMax = std::max(rMax, std::fabs(rate));
        }
        double turned = 2 * std::acos(std::clamp(std::fabs((q0.conj() * v->q).w), 0.0, 1.0)) / DEG;
        W.input = {};
        v->sas = SAS::Stability;
        double tStop = -1;
        for (int k = 1; k <= 600; k++) {
            W.update(1 / 60.0);
            if (len(v->w) / DEG < 0.5) { tStop = k / 60.0; break; }
        }
        double turnedAll = 2 * std::acos(std::clamp(std::fabs((q0.conj() * v->q).w), 0.0, 1.0)) / DEG;
        // (an aeroplane's pitch has a short-period swing: its rate passes through zero around 1 s, so it is
        // judged by the turn it made and its peak rate; the others hold a commanded rate)
        bool ok = (c.mode == 2 ? turned > 0.3 && rMax < 120 : r100 > 0.3 && r100 < 120) && tStop > 0 && !v->destroyed;
        static const char* MODE[] = {"orbit", "ascent", "flight", "rcs", "hover"};
        std::fprintf(T.log, "ctrl: %-20s %-7s %-5s mass %6.1f t  Ixx %.3g kg m2  wheels %.0f N m  rate after 0.25 / 0.5 / 1 s: %5.1f / %5.1f / %5.1f deg/s  "
                     "turned %5.1f deg in 1 s, %5.1f deg until stopped after %.2f s -> %s\n",
                     c.name, MODE[c.mode], axName[axis], v->mp.mass / 1000, I, v->torque, r25, r50, r100, turned, turnedAll, tStop, ok ? "ok" : "FAIL");
    }
    std::fflush(T.log);
    return true;
}

// ------------------------------------------------------------------ the aircraft and rover designs (--designtest)
// each built-in aircraft / rover at its start site in daylight, seen from three sides; the jet also in
// flight with the gear up (the gear folding)
bool designTestStep(App& app) {
    World& W = app.world;
    struct D { const char* craft; const char* body; double lat, lon; };
    static const D L[] = {{"Kestrel Helicopter", "Earth", 0, 0},   {"Mars Helicopter", "Mars", 18.4, 77.5}, {"Titan Rotorcraft", "Titan", -10.3, 167.6},
                          {"Swift Jet Trainer", "Earth", 0, 0},    {"Lunar Rover", "Moon", 26.13, 3.63},   {"Mars Rover", "Mars", 18.4, 77.5}};
    const int n = sizeof L / sizeof L[0];
    const int per = 40;
    int ci = T.frames / per, k = T.frames % per;
    static double tWait = 0;
    if (ci < n) {
        const D& d = L[ci];
        if (k == 1) {
            W.newGame(Settings{});
            app.gameStarted = true;
            Craft c = craftNamed(d.craft);
            app.lastLaunched = c;
            int body = W.U.find(d.body);
            double la = d.lat, lo = d.lon;
            if (std::string(d.body) == "Earth") {
                const LaunchSite& site = LAUNCH_SITES[std::clamp(W.settings.launchSite, 0, LAUNCH_SITE_COUNT - 1)];
                la = site.latDeg;
                lo = site.lonDeg;
            }
            sunAt(W, body, siteDir(la, lo), 35);
            W.launchAircraft(c, body);
            app.screen = Screen::Flight;
            flightOnEnter(app);
            Vessel* v = W.activeVessel();
            std::fprintf(T.log, "design: %-20s on %-5s mass %7.1f kg  length %.2f m  centre of mass (%.2f, %.2f, %.2f)  aircraft %d rotorcraft %d wheels %d gear %d drive %.0f W\n",
                         d.craft, d.body, W.vesselMass(*v), v->length, v->mp.com.x, v->mp.com.y, v->mp.com.z, (int)v->isAircraft, (int)v->rotorcraft,
                         (int)v->hasWheels, (int)v->gearRetract, v->drivePower);
        }
        Vessel* v = W.activeVessel();
        double size = v ? std::max(v->length, 2.0 * v->maxR) : 5;
        if (std::string(L[ci].craft) == "Mars Helicopter") size = 0.8;
        const double yaws[3] = {0.7, 2.3, 4.1};
        for (int vw = 0; vw < 3; vw++) {
            if (k == 6 + vw * 10) flightDebugView(yaws[vw], vw == 1 ? 0.08 : 0.3, size * 1.9 + 1.5, Vec3d{});
            if (k == 14 + vw * 10) {
                char name[64];
                std::snprintf(name, sizeof name, "28_%d_%c.png", ci, 'a' + vw);
                shot(name);
            }
        }
        return true;
    }
    // the jet in flight: gear up
    int j = T.frames - n * per;
    Vessel* v = W.activeVessel();
    if (j == 1) {
        W.newGame(Settings{});
        app.gameStarted = true;
        Craft c = craftNamed("Swift Jet Trainer");
        app.lastLaunched = c;
        int earth = W.U.find("Earth");
        const LaunchSite& site = LAUNCH_SITES[std::clamp(W.settings.launchSite, 0, LAUNCH_SITE_COUNT - 1)];
        sunAt(W, earth, siteDir(site.latDeg, site.lonDeg), 40);
        W.launchAircraft(c, earth);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        v = W.activeVessel();
        Vec3d up = norm(v->r), nose = v->q.rotate({0, 0, 1});
        v->landed = false;
        v->rolling = false;
        v->r = up * (len(v->r) + 600);
        v->v = cross(W.U.angVel(v->ref), v->r) + nose * 140.0;
        v->throttle = 0.7;
        v->spool = 0.7;
        v->sas = SAS::Stability;
        W.toggleLegs(*v);
        std::fprintf(T.log, "design: jet in flight, gear %s\n", v->legsDeployed ? "down (FAIL: did not fold)" : "up");
        tWait = GetTime();
        flightDebugView(2.0, -0.25, 16, Vec3d{});
        return true;
    }
    if (j > 1 && j < 100000 && GetTime() - tWait < 3.2) return true; // the folding takes 2.5 s
    if (j > 1 && T.tfSub < 77) {
        shot("28_9_jet_gear_up.png");
        flightDebugView(0.6, 0.15, 16, Vec3d{});
        T.tfSub = 77;
        return true;
    }
    if (T.tfSub == 77) {
        T.tfSub++;
        return true;
    }
    if (T.tfSub == 78) {
        shot("28_9_jet_gear_up_front.png");
        return false;
    }
    return true;
}


// ------------------------------------------------------------------ surface bases (--basetest)
// a lander with a habitat, an ISRU plant and surface power set down at Jezero builds a base; the rules (no base
// on Earth, none without a habitat), the launch complex, a rocket launched from the base's pad, save / load
Design moduleLander(std::initializer_list<const char*> parts) {
    Design d;
    d.name = "Outpost Lander";
    PartInst core = makeInst(CATALOG.find("probe_s"));
    d.stack.push_back(core);
    for (const char* id : parts) d.stack.push_back(makeInst(CATALOG.find(id)));
    PartInst& bottom = d.stack.back();
    RadialAttach legs;
    legs.def = CATALOG.find("legs");
    legs.count = 4;
    bottom.radial.push_back(legs);
    return d;
}
bool baseTestStep(App& app) {
    World& W = app.world;
    int mars = W.U.find("Mars");
    static int bad = 0, baseIdx = -1;
    auto check = [&](bool ok, const char* what) {
        std::fprintf(T.log, "basetest: %s -> %s\n", what, ok ? "ok" : "FAIL");
        bad += !ok;
    };
    if (T.frames == 2) {
        bad = 0;
        W.newGame(Settings{});
        app.gameStarted = true;
        // without a base, rovers and aircraft can start on Earth only (v1.1.1)
        check(W.deployBodies() == std::vector<int>{W.U.find("Earth")}, "no base yet: craft start on Earth only");
        // the rules first: nothing on Earth, no new base without a habitat
        {
            Craft c = craftFromDesign(moduleLander({"isru_plant"}));
            int i = W.launchAircraft(c, mars);
            BasePlan pl = planBase(W, i);
            check(!pl.ok && pl.why == "A new base needs a habitat module", "no base without a habitat");
            std::string why;
            check(!baseAllowed(W.U, W.U.find("Earth"), &why) && !baseAllowed(W.U, W.U.find("Jupiter"), &why), "no base on Earth or a gas giant");
            W.vessels.clear();
            W.active = -1;
        }
        Craft c = craftFromDesign(moduleLander({"hab_module", "greenhouse", "isru_plant", "surface_power"}));
        app.lastLaunched = c;
        sunAt(W, mars, siteDir(18.4, 77.5), 28);
        W.launchAircraft(c, mars);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        flightDebugView(2.2, 0.32, 60, Vec3d{});
        return true;
    }
    if (T.frames == 4) {
        flightDebugBuildDialog(app, true);
        return true;
    }
    if (T.frames == 5) {
        shot("49_build_dialog.png");
        flightDebugBuildDialog(app, false);
        return true;
    }
    if (T.frames == 6) {
        Vessel* v = W.activeVessel();
        for (int k = 0; k < 40 && v; k++) W.update(0.05); // (settles on its legs)
        BasePlan pl = planBase(W, W.active);
        std::fprintf(T.log, "basetest: plan ok %d (%s), %d modules, %d components\n", (int)pl.ok, pl.why.c_str(), (int)pl.modules.size(),
                     (int)pl.comps.size());
        check(pl.ok && pl.joins < 0 && pl.modules.size() == 4, "lander plans a new base of 4 modules");
        int n0 = (int)W.vessels[W.active].comps.size();
        baseIdx = buildBase(W, W.active, "Jezero Station");
        check(baseIdx == 0 && W.bases.size() == 1, "base founded");
        if (baseIdx >= 0) {
            BaseStatus st = baseStatus(W.bases[baseIdx]);
            std::fprintf(T.log, "basetest: %d modules, crew %d, power %.0f kW, demand %.0f kW, pad %d\n", (int)W.bases[baseIdx].modules.size(),
                         st.crew, st.powerKw, st.demandKw, (int)st.hasPad);
            check(st.requirementsMet() && st.hasPad, "habitat + ISRU + 40 kW: launch complex built");
        }
        Vessel* av = W.activeVessel();
        check(av && (int)av->comps.size() < n0 && av->landed, "the lander keeps what it did not unload");
        if (av) {
            for (const Component& k : av->comps)
                std::fprintf(T.log, "basetest:   kept %-28s z %.2f  pos %.2f %.2f  dims %.2f %.2f %.2f\n", k.name.c_str(), k.pos.z, k.pos.x, k.pos.y,
                             k.dims.x, k.dims.y, k.dims.z);
            Vec3d up = norm(av->landPos);
            double g = W.U.bodies[av->ref].radius + terrain::height(av->ref, up);
            std::fprintf(T.log, "basetest:   CoM %.2f m above the ground, %d contacts\n", len(av->landPos) - g, (int)av->contacts.size());
        }
        check(!W.launchBaseIds().empty(), "the base is a launch site");
        flightDebugView(2.2, 0.42, 320, Vec3d{-40, 50, 0});
        return true;
    }
    if (T.frames == 10) {
        shot("50_base_overview.png");
        flightDebugView(0.6, 0.20, 70, Vec3d{-40, 50, 5});
        return true;
    }
    if (T.frames == 11) {
        flightDebugBases(true);
        return true;
    }
    if (T.frames == 12) {
        shot("50b_bases_list.png");
        flightDebugBases(false);
        return true;
    }
    if (T.frames == 13) {
        shot("51_base_habitats.png");
        flightDebugView(-0.9, 0.26, 140, Vec3d{-70, 75, 4});
        return true;
    }
    if (T.frames == 16) {
        shot("52_base_greenhouse_isru.png");
        // night: the windows and the grow lights
        sunAt(W, mars, siteDir(18.4, 77.5), -12);
        flightDebugView(2.4, 0.22, 110, Vec3d{-40, 50, 4});
        return true;
    }
    if (T.frames == 20) {
        shot("53_base_night.png");
        sunAt(W, mars, siteDir(18.4, 77.5), 30);
        // a rocket from the base's pad
        W.settings.launchBase = W.bases[baseIdx].id;
        World::PadSite ps = W.padSite();
        check(ps.body == mars && ps.baseId == W.bases[baseIdx].id, "pad site is the base");
        Design d = builtinTemplates()[0];
        app.design = d;
        Craft c = craftFromDesign(d);
        app.lastLaunched = c;
        int vi = W.launch(c);
        Vessel& v = W.vessels[vi];
        double off = len(norm(v.landPos) - ps.up) * W.U.bodies[mars].radius;
        std::fprintf(T.log, "basetest: launched %s from %s: ref %s, %.1f m from the pad axis, landed %d\n", v.name.c_str(), W.launchSiteName().c_str(),
                     W.U.bodies[v.ref].name.c_str(), off, (int)v.landed);
        check(v.ref == mars && v.landed && off < 2.0, "rocket stands on the base pad");
        flightOnEnter(app);
        flightDebugView(1.9, 0.18, 70, Vec3d{0, 0, 15});
        return true;
    }
    if (T.frames == 24) {
        shot("54_base_rocket_on_pad.png");
        Vessel* v = W.activeVessel();
        double r0 = len(v->r);
        W.stage(*v);
        v = W.activeVessel();
        v->throttle = 1;
        for (int k = 0; k < 400 && v && !v->destroyed; k++) {
            W.update(0.05);
            v = W.activeVessel();
        }
        double climb = v ? len(v->r) - r0 : 0;
        std::fprintf(T.log, "basetest: after 20 s of thrust: %.0f m higher, destroyed %d\n", climb, v ? (int)v->destroyed : 1);
        check(v && !v->destroyed && climb > 500, "the rocket lifts off from Mars");
        flightDebugView(1.9, 0.10, 120, Vec3d{});
        return true;
    }
    if (T.frames == 28) {
        shot("55_base_liftoff.png");
        // with the base, Mars is a start world: a rover stands beside the base
        {
            std::vector<int> d = W.deployBodies();
            check(std::find(d.begin(), d.end(), mars) != d.end(), "with a base on Mars, Mars is offered as a start world");
            int ri = W.launchAircraft(craftNamed("Mars Rover"), mars);
            const SurfaceBase& sb = W.bases[baseIdx];
            double R = W.U.bodies[mars].radius;
            double dist = std::acos(std::clamp(dot(norm(W.vessels[ri].landPos), baseCentreDir(sb)), -1.0, 1.0)) * R;
            std::fprintf(T.log, "basetest: rover started %.0f m from the base centre\n", dist);
            check(W.vessels[ri].ref == mars && dist > 150 && dist < 400, "the rover starts beside the base");
        }
        // save and load: the base comes back with all its modules, the launch site with it
        std::string txt = W.serialize();
        std::string path = T.dir + "\\basetest.sfs";
        writeFileAtomic(path, txt);
        size_t nMod = W.bases[baseIdx].modules.size();
        int launchBase = W.settings.launchBase;
        World w2;
        bool ok = loadWorld(w2, path);
        check(ok && w2.bases.size() == 1 && w2.bases[0].modules.size() == nMod && w2.bases[0].name == "Jezero Station" &&
                  w2.settings.launchBase == launchBase,
              "save / load keeps the base");
        // a save from before the rename (its first line still the old tag) loads as well
        {
            std::string old = txt;
            size_t eol = old.find('\n');
            old = std::string("STELLAR") + "FORGE_SAVE 2" + old.substr(eol);
            std::string p2 = T.dir + "\\legacy.sfs";
            writeFileAtomic(p2, old);
            World w3;
            check(txt.rfind("ASTRAUM_SAVE", 0) == 0 && loadWorld(w3, p2) && w3.bases.size() == 1, "saves of the earlier name load");
        }
        // F11: borderless full screen and back
        keys::inject({KEY_F11, 0});
        return true;
    }
    if (T.frames == 31) {
        check(IsWindowState(FLAG_BORDERLESS_WINDOWED_MODE) && app.fullscreen, "F11 switches to full screen");
        shot("56_fullscreen.png");
        keys::inject({KEY_F11, 0});
        return true;
    }
    if (T.frames == 34) {
        check(!IsWindowState(FLAG_BORDERLESS_WINDOWED_MODE) && !app.fullscreen, "F11 again: back to the window");
        std::fprintf(T.log, "basetest: %s (%d failed)\n", bad ? "FAIL" : "all ok", bad);
        return false;
    }
    return true;
}


// ------------------------------------------------------------------ mission guidance, exploration, map speed (--missiontest)
// the plan toward the Moon from the pad to a lunar orbit, each burn carried out on rails and checked by the
// prediction; an interplanetary plan to Mars (window, departure, correction, encounter); the exploration log
// (a rover started on the Moon is no landing, old saves repaired); the cost of the encounter prediction on a
// heliocentric orbit; the GUI size
bool missionTestStep(App& app) {
    World& W = app.world;
    const Universe& U = W.U;
    static int bad = 0;
    auto check = [&](bool ok, const std::string& what) {
        std::fprintf(T.log, "missiontest: %s -> %s\n", what.c_str(), ok ? "ok" : "FAIL");
        bad += !ok;
    };
    int earth = U.find("Earth"), moon = U.find("Moon"), mars = U.find("Mars"), sun = U.find("Sun");
    auto plan = [&](Vessel& v, int target) {
        mission::invalidate();
        return mission::compute(W, v, target);
    };
    // carry the vessel on rails to simulation time t (no other vessel matters here)
    auto coastTo = [&](Vessel& v, double t) {
        keplerPropagate(v.r, v.v, U.bodies[v.ref].GM, t - W.t);
        W.t = t;
        W.refreshStates();
    };
    auto names = [&](mission::Phase ph) {
        static const char* N[] = {"None", "Message", "Prelaunch", "Ascent", "Departure", "Transfer", "Arrival", "InOrbit"};
        return N[(int)ph];
    };
    if (T.frames == 2) {
        bad = 0;
        W.newGame(Settings{});
        app.gameStarted = true;
        Design d = builtinTemplates()[0];
        app.design = d;
        app.lastLaunched = craftFromDesign(d);
        int vi = W.launch(app.lastLaunched);
        Vessel& v = W.vessels[vi];
        v.target = moon;
        mission::Plan p = plan(v, moon);
        std::fprintf(T.log, "missiontest: on the pad, target Moon: %s, window in %.0f s, parking %.0f km\n", names(p.phase), p.tLaunch - W.t, p.parkAlt / 1e3);
        check(p.phase == mission::Phase::Prelaunch && p.tLaunch > W.t && p.tLaunch < W.t + 1.3 * DAY, "Moon: a launch window within a day");
        app.screen = Screen::Flight;
        flightOnEnter(app);
        flightDebugView(2.0, 0.15, 90, Vec3d{0, 0, 20});
        return true;
    }
    if (T.frames == 6) {
        shot("60_mission_prelaunch.png");
        Vessel& v = *W.activeVessel();
        // as a pilot would: lift off at the launch window (the ascent itself is flown in the other tests)
        {
            mission::Plan pl = plan(v, moon);
            if (pl.tLaunch > W.t) {
                W.t = pl.tLaunch;
                W.refreshStates();
                v.r = W.U.bodyRot(v.ref, W.t) * v.landPos;
            }
        }
        W.placeInOrbit(v, ascentTargetAlt(U.bodies[earth]));
        mission::Plan p = plan(v, moon);
        std::fprintf(T.log, "missiontest: parking orbit: %s, burn in %.0f s, dv %.0f m/s, arrival in %.1f d\n", names(p.phase), p.tBurn - W.t, p.dv,
                     (p.tArrive - W.t) / DAY);
        check(p.phase == mission::Phase::Departure && p.dv > 2900 && p.dv < 3400 && p.tBurn >= W.t, "Moon: departure burn of ~3.1 km/s");
        flightDebugView(2.0, 0.25, 40, Vec3d{});
        return true;
    }
    if (T.frames == 9) {
        shot("61_mission_departure.png");
        Vessel& v = *W.activeVessel();
        mission::Plan p = plan(v, moon);
        coastTo(v, p.tBurn);
        mission::Plan pNow = plan(v, moon); // (at the burn point: still "burn now", not the next pass)
        std::fprintf(T.log, "missiontest: at the burn point: burn in %.0f s, dv %.0f m/s\n", pNow.tBurn - W.t, pNow.dv);
        check(pNow.tBurn - W.t < 5 && std::fabs(pNow.dv - p.dv) < 120, "Moon: at the burn point the plan says burn now");
        v.v += pNow.burn;
        mission::Plan q = plan(v, moon);
        std::fprintf(T.log, "missiontest: after the departure burn: %s, encounter %d in %.1f d, periapsis %.0f km\n", names(q.phase), (int)q.encounter,
                     (q.tEncounter - W.t) / DAY, q.encPe / 1e3);
        check(q.phase == mission::Phase::Transfer && q.encounter && q.encPe > 5e3 && q.encPe < 300e3, "Moon: the burn brings an encounter with a low periapsis");
        // into the Moon's sphere of influence, then the capture
        Encounter e;
        if (q.encounter && predictEncounter(U, v.ref, v.r, v.v, W.t, e) && e.body == moon) {
            W.t += e.tEntry;
            W.refreshStates();
            v.ref = moon;
            v.r = e.r;
            v.v = e.v;
            mission::Plan a = plan(v, moon);
            std::fprintf(T.log, "missiontest: at the Moon: %s, capture in %.0f s, dv %.0f m/s, periapsis %.0f km\n", names(a.phase), a.tBurn - W.t, a.dv, a.pe / 1e3);
            check(a.phase == mission::Phase::Arrival && a.dv > 300 && a.dv < 1500, "Moon: a capture burn at periapsis");
            coastTo(v, a.tBurn);
            a = plan(v, moon);
            v.v += a.burn;
            mission::Plan o = plan(v, moon);
            std::fprintf(T.log, "missiontest: after the capture: %s, %.0f x %.0f km\n", names(o.phase), o.pe / 1e3, o.ap / 1e3);
            check(o.phase == mission::Phase::InOrbit, "Moon: in orbit after the capture");
        } else check(false, "Moon: the encounter state");
        return true;
    }
    if (T.frames == 12) {
        // Mars from a parking orbit around Earth
        W.newGame(Settings{});
        int vi = W.launch(app.lastLaunched);
        Vessel& v = W.vessels[vi];
        W.placeInOrbit(v, ascentTargetAlt(U.bodies[earth]));
        v.target = mars;
        mission::Plan p = plan(v, mars);
        std::fprintf(T.log, "missiontest: to Mars: %s, window in %.0f d, burn in %.1f d, dv %.0f m/s, flight %.0f d\n", names(p.phase),
                     (p.tWindow - W.t) / DAY, (p.tBurn - W.t) / DAY, p.dv, (p.tArrive - p.tBurn) / DAY);
        check(p.phase == mission::Phase::Departure && p.dv > 3300 && p.dv < 6000 && p.tWindow < W.t + 800 * DAY, "Mars: a transfer window and an ejection burn");
        coastTo(v, p.tBurn);
        p = plan(v, mars);
        v.v += p.burn;
        // out of Earth's sphere of influence: on the Sun's patched conic
        Conic c = stateToConic(v.r, v.v, U.bodies[earth].GM);
        double cosExit = (c.p / U.bodies[earth].soi - 1) / c.e;
        double tExit = std::fabs(cosExit) <= 1 ? timeToTrueAnomaly(c, std::acos(cosExit)) : -1;
        check(c.e > 1 && tExit > 0, "Mars: the ejection escapes Earth");
        {
            mission::Plan out = plan(v, mars);
            check(out.phase == mission::Phase::Transfer, "Mars: just after the ejection the plan is the way out (no capture burn)");
        }
        if (tExit > 0) {
            coastTo(v, W.t + tExit + 60);
            Vec3d er, ev;
            er = U.relPos(W.states, earth, sun);
            ev = U.relVel(W.states, earth, sun);
            v.ref = sun;
            v.r += er;
            v.v += ev;
            mission::Plan t = plan(v, mars);
            std::fprintf(T.log, "missiontest: around the Sun: %s, encounter %d, correction %.0f m/s\n", names(t.phase), (int)t.encounter, t.dv);
            if (!t.encounter && t.tBurn >= 0) {
                coastTo(v, std::max(W.t, t.tBurn));
                t = plan(v, mars);
                v.v += t.burn;
                t = plan(v, mars);
            }
            std::fprintf(T.log, "missiontest: after the correction: encounter %d in %.0f d, periapsis %.0f km\n", (int)t.encounter, (t.tEncounter - W.t) / DAY,
                         t.encPe / 1e3);
            check(t.encounter, "Mars: an encounter after the course correction");
            // the encounter prediction on a heliocentric orbit (the map called it every frame)
            auto t0 = std::chrono::steady_clock::now();
            Encounter e;
            for (int k = 0; k < 20; k++) predictEncounter(U, v.ref, v.r, v.v, W.t, e);
            double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / 20;
            std::fprintf(T.log, "missiontest: encounter prediction around the Sun: %.2f ms\n", ms);
            check(ms < 8, "the heliocentric encounter prediction is cheap");
        }
        app.screen = Screen::Flight;
        flightOnEnter(app);
        flightDebugSetMap(true, 4e11);
        return true;
    }
    if (T.frames == 14) T.t0 = GetTime();
    if (T.frames == 44) {
        double frame = (GetTime() - T.t0) / 30 * 1000;
        std::fprintf(T.log, "missiontest: map view with a heliocentric vessel: %.1f ms per frame\n", frame);
        T.helioFrame = frame; // (judged against the reference below: the machine load changes both alike)
        shot("62_mission_map_heliocentric.png");
        // the same map with a vessel in a low Earth orbit (the reference)
        W.newGame(Settings{});
        int vi = W.launch(app.lastLaunched);
        W.placeInOrbit(W.vessels[vi], 200e3);
        flightOnEnter(app);
        flightDebugSetMap(true, 4e11);
        return true;
    }
    if (T.frames == 48) T.t0 = GetTime();
    if (T.frames == 78) {
        double ref = (GetTime() - T.t0) / 30 * 1000;
        std::fprintf(T.log, "missiontest: map view with a vessel in low Earth orbit (reference): %.1f ms per frame\n", ref);
        check(T.helioFrame < ref * 1.5 + 5, "the map with a heliocentric vessel is as fast as with one in Earth orbit");
        flightDebugSetMap(false, 0);
        // exploration: a rover started on the Moon is no landing there; an old save's false mark is repaired
        W.newGame(Settings{});
        Craft rover = craftNamed("Lunar Rover");
        W.launchAircraft(rover, moon);
        for (int k = 0; k < 20; k++) W.update(0.05);
        check(!(W.explored[moon] & 4), "a rover started on the Moon does not count as a landing");
        W.explored[moon] |= 4 | 1;
        World w2;
        bool ok = w2.deserialize(W.serialize());
        check(ok && !(w2.explored[moon] & 4) && (w2.explored[earth] & 4), "an old false landing mark is cleared on load");
        // the GUI size
        app.screen = Screen::Menu;
        app.settingsOpen = true;
        T.sc0 = ui::scale();
        ui::setUserScale(1.25f);
        return true;
    }
    if (T.frames == 81) {
        float s1 = ui::scale();
        std::fprintf(T.log, "missiontest: GUI scale %.3f -> %.3f at 125 %%\n", T.sc0, s1);
        check(s1 > T.sc0 * 1.02f || s1 >= std::min(GetScreenWidth() / 1100.0f, GetScreenHeight() / 680.0f) - 0.01f, "the GUI size setting enlarges the interface");
        shot("63_gui_125.png");
        ui::setUserScale(0.75f);
        return true;
    }
    if (T.frames == 84) {
        float s2 = ui::scale();
        check(s2 < T.sc0 * 0.98f, "the GUI size setting shrinks the interface");
        shot("64_gui_75.png");
        ui::setUserScale(1.0f);
        app.settingsOpen = false;
        // the factory's show / hide button (v1.1.1: the row took its click)
        app.design = builtinTemplates()[0];
        app.screen = Screen::Factory;
        return true;
    }
    if (T.frames == 88 || T.frames == 92) {
        Rectangle e;
        if (factoryDebugEye(0, e)) ui::testClick({e.x + e.width / 2, e.y + e.height / 2});
        else check(false, "the factory shows the eye button of the first part");
        return true;
    }
    if (T.frames == 90) {
        check(app.design.stack.size() > 0 && app.design.stack[0].hidden, "factory: the eye button hides the part");
        shot("65_factory_part_hidden.png");
        return true;
    }
    if (T.frames == 94) {
        check(app.design.stack.size() > 0 && !app.design.stack[0].hidden, "factory: a second click shows it again");
        std::fprintf(T.log, "missiontest: %s (%d failed)\n", bad ? "FAIL" : "all ok", bad);
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ leaving Earth on rails (--escapetest)
// an escape from a low Earth orbit flown at each time warp, the warp asked for again whenever it stops (as a
// player holding the warp key): the vessel must follow the heliocentric orbit the map showed before the escape
bool escapeTestStep(App& app) {
    World& W = app.world;
    const Universe& U = W.U;
    if (T.frames < 2) return true;
    int bad = 0;
    int earth = U.find("Earth"), sun = U.find("Sun");
    struct Case { double vinf; double ang; int warp; };
    std::vector<Case> cases;
    for (double vinf : {300.0, 1500.0, 4000.0})
        for (double ang : {0.0, 2.0})
            for (int wi : {8, 10, 12}) cases.push_back({vinf, ang, wi});
    for (const Case& cs : cases) {
        W.newGame(Settings{});
        app.gameStarted = true;
        int vi = W.launch(craftFromDesign(builtinTemplates()[0]));
        W.placeInOrbit(W.vessels[vi], 200e3);
        for (int k = 0; k < 3; k++) W.update(0.02);
        Vessel* v = W.activeVessel();
        // rotate the burn point along the orbit (a different exit direction)
        keplerPropagate(v->r, v->v, U.bodies[earth].GM, cs.ang * 1000);
        double GM = U.bodies[earth].GM, rr = len(v->r);
        v->v = norm(v->v) * std::sqrt(cs.vinf * cs.vinf + 2 * GM / rr);
        // the prediction the map draws: the exit point and the Sun's conic from there
        Conic c = stateToConic(v->r, v->v, GM);
        double soi = U.bodies[earth].soi;
        double cosExit = (c.p / soi - 1) / c.e;
        double tExit = std::fabs(cosExit) <= 1 ? timeToTrueAnomaly(c, std::acos(cosExit)) : -1;
        if (tExit <= 0) {
            std::fprintf(T.log, "escapetest: vinf %.0f: no exit predicted -> FAIL\n", cs.vinf);
            bad++;
            continue;
        }
        Vec3d r = v->r, vv = v->v;
        keplerPropagate(r, vv, GM, tExit);
        std::vector<BodyState> S2;
        U.computeStates(W.t + tExit, S2);
        Vec3d rp = r + U.relPos(S2, earth, sun), vp = vv + U.relVel(S2, earth, sun);
        double tPred = W.t + tExit, tStart = W.t;
        double tSwitch = -1, tEnd = tPred + 120 * DAY;
        int frames = 0, stops = 0;
        double worst = 0, worstT = 0;
        while (frames++ < 200000 && W.t < tEnd) {
            v = W.activeVessel();
            if (!v || v->destroyed) break;
            if (W.warpIndex < cs.warp) {
                if (W.warpIndex == 0 && frames > 1) stops++;
                W.setWarp(cs.warp);
            }
            W.update(0.05);
            v = W.activeVessel();
            if (!v) break;
            if (v->ref == sun && tSwitch < 0) tSwitch = W.t;
            if (v->ref == sun) {
                Vec3d a = rp, b = vp;
                keplerPropagate(a, b, U.bodies[sun].GM, W.t - tPred);
                double d = len(v->r - a);
                if (d > worst) { worst = d; worstT = W.t; }
            }
        }
        v = W.activeVessel();
        double dEarth = v ? len(v->r - U.relPos(W.states, earth, v->ref)) : -1;
        bool ok = v && v->ref == sun && tSwitch > 0 && std::fabs(tSwitch - tPred) < 0.02 * tExit + 3600 && worst < 1e-3 * len(rp);
        std::fprintf(T.log,
                     "escapetest: vinf %5.0f m/s, burn +%4.0f s, warp x%-6g: exit predicted in %6.2f d, flown %6.2f d, %d stops, %d frames; "
                     "largest gap to the predicted Sun orbit %.0f km (at +%.1f d), %.2f AU from Earth after %.0f d -> %s\n",
                     cs.vinf, cs.ang * 1000, WARP_LEVELS[cs.warp], tExit / DAY, tSwitch > 0 ? (tSwitch - tStart) / DAY : -1, stops, frames, worst / 1e3,
                     (worstT - tStart) / DAY, dEarth / AU, (W.t - tStart) / DAY, ok ? "ok" : "FAIL");
        bad += !ok;
        std::fflush(T.log);
    }
    std::fprintf(T.log, "escapetest: %s (%d failed)\n", bad ? "FAIL" : "all ok", bad);
    std::fflush(T.log);
    return false;
}

// ------------------------------------------------------------------ a target launched by the player (--rdvtest)
// the first rocket flown to orbit by the ascent guidance (as a player would), then a second one launched from
// the pad toward it: the first one's orbit must not change while the second flies (ascent, coast, warp)
bool rdvTestStep(App& app) {
    World& W = app.world;
    const Universe& U = W.U;
    static int stage = 0, tgtId = -1, bad = 0;
    static double a0 = 0, e0 = 0, worstDa = 0, worstDe = 0, tLaunch2 = 0;
    static int tplIdx = 0;
    auto tpl = builtinTemplates();
    int earth = U.find("Earth");
    const Body& eb = U.bodies[earth];
    auto check = [&](bool ok, const std::string& what) {
        std::fprintf(T.log, "rdvtest: %s -> %s\n", what.c_str(), ok ? "ok" : "FAIL");
        std::fflush(T.log);
        bad += !ok;
    };
    auto target = [&]() -> Vessel* {
        for (Vessel& o : W.vessels)
            if (o.id == tgtId && !o.destroyed) return &o;
        return nullptr;
    };
    auto watch = [&](const char* when) {
        Vessel* t = target();
        if (!t) return;
        Conic c = stateToConic(t->r, t->v, U.bodies[t->ref].GM);
        double da = std::fabs(c.a - a0), de = std::fabs(c.e - e0);
        if (da > worstDa + 50 || de > worstDe + 1e-4) {
            Vessel* a = W.activeVessel();
            std::fprintf(T.log, "rdvtest: [%s] target a %.3f km (%+.3f km) e %.6f ref %s ignited %d thr %.2f sas %d stage %d rcs %d; active alt %.0f km warp %d\n", when, c.a / 1e3,
                         (c.a - a0) / 1e3, c.e, U.bodies[t->ref].name.c_str(), (int)t->ignited, t->throttle, (int)t->sas, t->stage, (int)t->rcs,
                         a ? (len(a->r) - U.bodies[a->ref].radius) / 1e3 : -1.0, W.warpIndex);
            std::fflush(T.log);
        }
        worstDa = std::max(worstDa, da);
        worstDe = std::max(worstDe, de);
    };
    if (T.frames == 2) {
        bad = 0;
        stage = 0;
        tplIdx = 0;
        for (int i = 0; i < (int)tpl.size(); i++)
            if (tpl[i].name.find("Orbiter") != std::string::npos || tpl[i].name.find("orbit") != std::string::npos) { tplIdx = i; break; }
        W.newGame(Settings{});
        app.gameStarted = true;
        app.design = tpl[tplIdx];
        app.lastLaunched = craftFromDesign(app.design);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        std::fprintf(T.log, "rdvtest: rocket '%s'\n", tpl[tplIdx].name.c_str());
        return true;
    }
    if (T.frames < 6) return true;
    Vessel* v = W.activeVessel();
    if (!v || v->destroyed) {
        check(false, "the active rocket flies");
        return false;
    }
    if (stage == 0 || stage == 2) {
        if (v->sas != SAS::Ascent && v->throttle < 0.5 && W.t - v->launchTime < 1) {
            v->sas = SAS::Ascent;
            v->throttle = 1;
            if (!v->ignited) W.stage(*v);
            return true;
        }
        if (v->ascentPhase == 1 && len(v->r) - eb.radius > eb.atmTop) W.warpIndex = std::min(5, W.maxWarpIndex());
        else if (W.warpIndex > 0) W.warpIndex = 0;
        for (int k = 0; k < 8 && !v->destroyed && v->sas == SAS::Ascent; k++) {
            W.update(0.05);
            tfStaging(app, v);
            if (stage == 2) watch("second ascent");
        }
        if (W.t - v->launchTime > 6000) {
            check(false, "the ascent reaches an orbit");
            return false;
        }
        if (v->sas == SAS::Ascent) return true;
        Conic c = stateToConic(v->r, v->v, U.bodies[v->ref].GM);
        std::fprintf(T.log, "rdvtest: ascent %d done: %.0f x %.0f km, ignited %d throttle %.2f sas %d stage %d\n", stage / 2 + 1, (c.pe - eb.radius) / 1e3,
                     (c.ap - eb.radius) / 1e3, (int)v->ignited, v->throttle, (int)v->sas, v->stage);
        if (stage == 0) {
            check(c.valid && c.e < 1 && c.pe - eb.radius > 150e3, "the first rocket reaches orbit");
            tgtId = v->id;
            a0 = c.a;
            e0 = c.e;
            worstDa = worstDe = 0;
            // some time in orbit, as a player would spend it
            for (int k = 0; k < 40; k++) W.update(0.05);
            // the second rocket from the pad, the first one's throttle left up (v1.1.3: it went on burning unseen)
            {
                double fuel = 0;
                for (size_t i = 0; i < v->comps.size(); i++)
                    if (v->comps[i].func == Func::Tank && v->comps[i].stage == v->stage) fuel += v->prop[i];
                std::fprintf(T.log, "rdvtest: the first rocket keeps %.0f kg of propellant in its stage; throttle left at 100 %%\n", fuel);
            }
            v->throttle = 1;
            int vi = W.launch(app.lastLaunched);
            W.vessels[vi].targetVessel = tgtId;
            Vessel* t = target();
            check(t != nullptr, "the first rocket stays in orbit after the second launch");
            if (t) {
                Conic ct = stateToConic(t->r, t->v, U.bodies[t->ref].GM);
                a0 = ct.a;
                e0 = ct.e;
            }
            tLaunch2 = W.t;
            stage = 2;
            return true;
        }
        stage = 3;
        return true;
    }
    if (stage == 3) {
        // coasting near each other at every warp level, then back
        for (int wi : {4, 6, 8, 10, 12, 6, 0}) {
            for (int k = 0; k < 60; k++) {
                W.setWarp(wi);
                W.update(0.05);
                watch(("warp " + std::to_string(wi)).c_str());
            }
        }
        std::fprintf(T.log, "rdvtest: target orbit drift over %.0f min: a %.3f km, e %.6f\n", (W.t - tLaunch2) / 60, worstDa / 1e3, worstDe);
        check(worstDa < 200 && worstDe < 1e-4, "the target's orbit stays the same while the second rocket flies");
        // a vessel left with its throttle up and its tanks dry (an older save) counts as able to thrust: under
        // x10,000,000 it must keep up with the world clock (its steps fell to 1 s, the frame ended early and it was left
        // behind: a target that jumps; at x1e8 each frame stalled for seconds)
        {
            W.newGame(Settings{});
            W.launchInOrbit(craftNamed("Crew Ferry"), -1, 0);
            tgtId = W.activeVessel()->id;
            W.launchInOrbit(craftNamed("Station Core Module"), tgtId, 2000);
            Vessel* t = target();
            if (!t->ignited) W.stage(*t);
            t = target();
            t->throttle = 1;
            for (size_t i = 0; i < t->comps.size(); i++)
                if (t->comps[i].func == Func::Tank) t->prop[i] = 0;
            {
                int fires = 0;
                for (size_t i = 0; i < t->comps.size(); i++)
                    if (t->comps[i].func == Func::Engine && engineFiresNow(*t, t->comps[i]) && t->phys[i].thrustVac > 0) fires++;
                std::fprintf(T.log, "rdvtest: dry vessel '%s': ignited %d stage %d, engines of the stage %d\n", t->name.c_str(), (int)t->ignited, t->stage, fires);
            }
            Vec3d r0 = t->r, v0 = t->v;
            double tA = W.t, GM = U.bodies[t->ref].GM;
            W.setWarp(11);
            double w0 = GetTime();
            W.update(0.05);
            std::fprintf(T.log, "rdvtest: that frame took %.0f ms\n", (GetTime() - w0) * 1000);
            t = target();
            if (t) {
                keplerPropagate(r0, v0, GM, W.t - tA);
                double d = len(t->r - r0);
                std::fprintf(T.log, "rdvtest: dry vessel with its throttle up after %.1f d at x1e7 (one frame): %.1f km from its orbit position\n", (W.t - tA) / DAY,
                             d / 1e3);
                check(d < 1000e3, "a vessel with dry tanks and its throttle up keeps pace with the world clock");
            }
        }
        std::fprintf(T.log, "rdvtest: %s (%d failed)\n", bad ? "FAIL" : "all ok", bad);
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ the target's orbit near the active vessel (--rdvsweep)
// every SAS mode / RCS / engine state of the target, the active vessel idle, translating on RCS or burning
// its engine close by, at x1 and under time warp: the target (coasting, nobody flies it) keeps its orbit
bool rdvSweepStep(App& app) {
    World& W = app.world;
    const Universe& U = W.U;
    if (T.frames < 2) return true;
    int bad = 0, n = 0;
    const SAS modes[] = {SAS::Off, SAS::Stability, SAS::Prograde, SAS::Target, SAS::Dock, SAS::Burn, SAS::Ascent};
    for (SAS md : modes)
        for (int rcs = 0; rcs < 2; rcs++)
            for (int ign = 0; ign < 2; ign++)
                for (int act = 0; act < 3; act++) {
                        // (a 400 km orbit: one through the air loses height to its drag)
                        W.newGame(Settings{});
                        app.gameStarted = true;
                        W.launchInOrbit(craftNamed("Station Core Module"), -1, 0);
                        Vessel* t0 = W.activeVessel();
                        int tid = t0->id;
                        // the target's state as its pilot left it (ignited: with the throttle up as well)
                        t0->sas = md;
                        t0->rcs = rcs;
                        if (ign && !t0->ignited) W.stage(*t0);
                        if (ign) W.activeVessel()->throttle = 1;
                        W.launchInOrbit(craftNamed("Crew Ferry"), tid, 60);
                        Vessel* a = W.activeVessel();
                        a->targetVessel = tid;
                        Vessel* t = nullptr;
                        for (Vessel& o : W.vessels)
                            if (o.id == tid) t = &o;
                        t->targetVessel = a->id;
                        Conic c0 = stateToConic(t->r, t->v, U.bodies[t->ref].GM);
                        double worst = 0, worstE = 0;
                        std::string when;
                        auto watch = [&](const char* w) {
                            Vessel* tt = nullptr;
                            for (Vessel& o : W.vessels)
                                if (o.id == tid && !o.destroyed) tt = &o;
                            if (!tt) { worst = 1e30; when = std::string(w) + " (gone)"; return; }
                            Conic c = stateToConic(tt->r, tt->v, U.bodies[tt->ref].GM);
                            double d = std::fabs(c.a - c0.a), de = std::fabs(c.e - c0.e);
                            if (d > worst) { worst = d; when = w; }
                            worstE = std::max(worstE, de);
                        };
                        for (int k = 0; k < 400; k++) {
                            a = W.activeVessel();
                            if (!a) break;
                            ControlInput in;
                            if (act == 1) {
                                a->rcs = true;
                                in.tz = (k / 50) % 2 ? -1 : 1;
                                in.tx = 0.5;
                            }
                            if (act == 2) {
                                if (!a->ignited) W.stage(*a);
                                a->sas = SAS::Prograde;
                                a->throttle = k < 200 ? 1 : 0;
                            }
                            W.input = in;
                            int wi = k < 250 ? 0 : k < 300 ? 4 : k < 350 ? 7 : 10;
                            if (act == 2 && k < 200) wi = 0;
                            W.setWarp(wi);
                            W.update(0.05);
                            watch(("frame " + std::to_string(k) + " warp " + std::to_string(W.warpIndex)).c_str());
                        }
                        W.input = {};
                        bool ok = worst < 50.0 && worstE < 1e-5;
                        n++;
                        if (!ok)
                            std::fprintf(T.log, "rdvsweep: target sas %-10s rcs %d ignited %d, active %s: target a changed %.3f km e %.2e (worst at %s) -> FAIL\n",
                                         sasName(md), rcs, ign, act == 0 ? "idle" : act == 1 ? "RCS" : "engine", worst / 1e3, worstE, when.c_str());
                        bad += !ok;
                        std::fflush(T.log);
                    }
    std::fprintf(T.log, "rdvsweep: %d cases, %s (%d failed)\n", n, bad ? "FAIL" : "all ok", bad);
    return false;
}

// ------------------------------------------------------------------ a saved game's vessels over time (--savecheck <file>)
// (diagnostics) every vessel of a save: its orbit and state; then the game runs at x1 and under time warp and
// every change of a coasting vessel's orbit is reported
std::string g_saveCheck;
bool saveCheckStep(App& app) {
    World& W = app.world;
    const Universe& U = W.U;
    if (T.frames < 2) return true;
    std::ifstream in(g_saveCheck, std::ios::binary);
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    bool ok = W.deserialize(text);
    std::fprintf(T.log, "savecheck: loaded %d (%zu vessels), t %.0f, active %d\n", (int)ok, W.vessels.size(), W.t, W.active);
    if (!ok) return false;
    app.gameStarted = true;
    Vessel* av = W.activeVessel();
    auto info = [&](const Vessel& v, const char* tag) {
        const Body& b = U.bodies[v.ref];
        Conic c = stateToConic(v.r, v.v, b.GM);
        std::fprintf(T.log, "  %s id %d '%s' ref %s landed %d debris %d destroyed %d: a %.3f km e %.6f pe %.1f km ap %.1f km | ignited %d thr %.2f sas %s rcs %d stage %d "
                            "sail %d target %d\n",
                     tag, v.id, v.name.c_str(), b.name.c_str(), (int)v.landed, (int)v.debris, (int)v.destroyed, c.a / 1e3, c.e, (c.pe - b.radius) / 1e3,
                     c.e < 1 ? (c.ap - b.radius) / 1e3 : -1.0, (int)v.ignited, v.throttle, sasName(v.sas), (int)v.rcs, v.stage, (int)v.sailDeployed, v.targetVessel);
    };
    for (const Vessel& v : W.vessels) info(v, &v == av ? "ACTIVE" : "      ");
    std::fflush(T.log);
    struct Ref { int id; double a, e; double worst; };
    std::vector<Ref> refs;
    for (const Vessel& v : W.vessels)
        if (!v.destroyed && !v.landed) {
            Conic c = stateToConic(v.r, v.v, U.bodies[v.ref].GM);
            refs.push_back({v.id, c.a, c.e, 0});
        }
    auto watch = [&](const std::string& when) {
        for (Ref& r : refs)
            for (const Vessel& v : W.vessels) {
                if (v.id != r.id || v.destroyed) continue;
                Conic c = stateToConic(v.r, v.v, U.bodies[v.ref].GM);
                double d = std::fabs(c.a - r.a);
                if (d > r.worst * 2 + 100) {
                    std::fprintf(T.log, "savecheck: [%s] id %d '%s' a %.3f km -> %.3f km (e %.6f -> %.6f) ref %s alt %.1f km ignited %d thr %.2f sas %s active %d\n",
                                 when.c_str(), v.id, v.name.c_str(), r.a / 1e3, c.a / 1e3, r.e, c.e, U.bodies[v.ref].name.c_str(),
                                 (len(v.r) - U.bodies[v.ref].radius) / 1e3, (int)v.ignited, v.throttle, sasName(v.sas), (int)(&v == W.activeVessel()));
                    std::fflush(T.log);
                    r.worst = d;
                }
            }
    };
    double t0 = W.t;
    for (int wi : {0, 2, 4, 5, 6, 7, 8, 9, 10}) {
        for (int k = 0; k < (wi == 0 ? 400 : 120); k++) {
            W.setWarp(wi);
            W.update(0.05);
            watch("warp " + std::to_string(wi) + " (asked) " + std::to_string(W.warpIndex) + " (got) t+" + std::to_string((int)(W.t - t0)));
        }
    }
    std::fprintf(T.log, "savecheck: ran %.0f s\n", W.t - t0);
    for (const Vessel& v : W.vessels) info(v, &v == W.activeVessel() ? "ACTIVE" : "      ");
    return false;
}

// ------------------------------------------------------------------ launch window toward a vessel, night vision on the map (--launchwintest)
// From Tanegashima (30.4 N) toward targets in three planes: inclined 51.6 degrees (crossed twice a day), 30.4 (just
// reached, once a day) and 0 (never over the site: nearest once a day). The ISS-like one flown: warp to the window
// on the pad, SAS Ascent into orbit - the two planes must agree; a launch off the window for comparison. Then the
// map with night vision (image intensifier: a green picture) and the bright mode.
bool launchWinTestStep(App& app) {
    World& W = app.world;
    const Universe& U = W.U;
    static int bad = 0, stage = 0, tgtId = -1;
    static double relOff = -1, tLift = 0;
    int earth = U.find("Earth");
    const Body& eb = U.bodies[earth];
    auto check = [&](bool ok, const std::string& what) {
        std::fprintf(T.log, "launchwin: %s -> %s\n", what.c_str(), ok ? "ok" : "FAIL");
        std::fflush(T.log);
        bad += !ok;
    };
    auto target = [&]() -> Vessel* {
        for (Vessel& o : W.vessels)
            if (o.id == tgtId && !o.destroyed) return &o;
        return nullptr;
    };
    // a target in a circular 420 km orbit of inclination incDeg, its node at lanDeg (Earth's equatorial frame)
    auto placeTarget = [&](double incDeg, double lanDeg) {
        Settings st;
        st.launchSite = 0; // Tanegashima
        W.newGame(st);
        app.gameStarted = true;
        W.launchInOrbit(craftNamed("Station Core Module"), -1, 0);
        Vessel* t = W.activeVessel();
        tgtId = t->id;
        Vec3d ex = eb.poleFrame.col(0), ey = eb.poleFrame.col(1), ez = eb.poleFrame.col(2);
        double O = lanDeg * DEG, i = incDeg * DEG;
        Vec3d node = ex * std::cos(O) + ey * std::sin(O);
        Vec3d h = ez * std::cos(i) + cross(node, ez) * std::sin(i);
        double R = eb.radius + 420e3;
        t->r = node * R;
        t->v = norm(cross(h, node)) * std::sqrt(eb.GM / R);
        t->landed = false;
        int vi = W.launch(craftFromDesign(builtinTemplates()[0]));
        W.vessels[vi].targetVessel = tgtId;
        return &W.vessels[vi];
    };
    auto relInc = [&](const Vessel& a, const Vessel& b) {
        return std::acos(std::clamp(dot(norm(cross(a.r, a.v)), norm(cross(b.r, b.v))), -1.0, 1.0)) / DEG;
    };
    if (T.frames == 2) {
        bad = 0;
        struct C { double inc; const char* what; bool crosses; };
        for (C c : {C{51.6, "ISS-like 51.6 deg", true}, C{30.4009, "30.4 deg (the site's latitude)", true}, C{0.0, "equatorial", false}}) {
            Vessel* a = placeTarget(c.inc, 40);
            Vessel* t = target();
            navg::Guidance g = navg::compute(W, *a, *t, 0, 1.0);
            std::fprintf(T.log, "launchwin: %s: mode %d, window in %.2f h, heading %.1f deg, site %.2f deg from the plane now, %.2f deg at the window\n",
                         c.what, (int)g.mode, (g.tLaunch - W.t) / 3600, g.launchAzDeg, g.offNowDeg, g.planeOffDeg);
            bool ok = g.mode == navg::Mode::Launch && g.tLaunch >= W.t && g.tLaunch - W.t < 1.05 * DAY;
            if (c.crosses) ok = ok && g.planeOffDeg < 0.3;
            else ok = ok && std::fabs(g.planeOffDeg - 30.4) < 0.3 && std::fabs(g.launchAzDeg - 90) < 1;
            if (c.inc > 40) ok = ok && (std::fabs(g.launchAzDeg - 43.4) < 4 || std::fabs(g.launchAzDeg - 136.6) < 4); // (sin az = cos i / cos lat)
            check(ok, std::string("a launch window toward a target ") + c.what);
            // the window found is the nearest: the site's distance from the plane there is a minimum
            if (g.mode == navg::Mode::Launch) {
                Vec3d h = norm(cross(t->r, t->v));
                auto off = [&](double tt) { return std::fabs(dot(norm(U.bodyRot(earth, tt) * a->landPos), h)); };
                // (the lift-off leads the nearest pass by 3 minutes when the plane crosses the site)
                double tMin = g.tLaunch + (g.planeOffDeg < 0.5 && g.tLaunch > W.t + 1 ? 180.0 : 0.0);
                double best = 1e9;
                for (double tt = W.t; tt < tMin - 60; tt += 30) best = std::min(best, off(tt));
                check(best >= std::sin(g.planeOffDeg * DEG) - 1e-4, std::string("no closer pass before the window ") + c.what);
            }
        }
        // the ISS-like target: warp to the window on the pad, then fly SAS Ascent
        Vessel* a = placeTarget(51.6, 40);
        app.lastLaunched = craftFromDesign(builtinTemplates()[0]);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        flightDebugNav(true);
        navg::Guidance g = navg::compute(W, *a, *target(), 0, 1.0);
        W.startWarpTo(g.tLaunch - 30);
        stage = 1;
        return true;
    }
    if (T.frames < 4) return true;
    Vessel* v = W.activeVessel();
    if (!v || v->destroyed) {
        check(false, "the rocket flies");
        return false;
    }
    if (stage == 1) {
        if (T.frames == 6) shot("90_launch_window_pad.png");
        if (W.warpTo > 0) {
            for (int k = 0; k < 4; k++) W.update(0.05);
            return true;
        }
        navg::Guidance g = navg::compute(W, *v, *target(), 0, 1.0);
        std::fprintf(T.log, "launchwin: the warp ended %.0f s before the window; site %.2f deg from the plane\n", g.tLaunch - W.t, g.offNowDeg);
        check(g.offNowDeg < 1.0, "the warp to the window ends with the site under the target's plane");
        shot("91_launch_window_open.png");
        // lift off at the window
        for (int k = 0; k < 400 && W.t < g.tLaunch - 1; k++) W.update(0.05);
        v = W.activeVessel();
        v->sas = SAS::Ascent;
        v->throttle = 1;
        if (!v->ignited) W.stage(*v);
        tLift = W.t;
        stage = 2;
        return true;
    }
    if (stage == 2 || stage == 4) {
        if (v->ascentPhase == 1 && len(v->r) - eb.radius > eb.atmTop) W.warpIndex = std::min(5, W.maxWarpIndex());
        else if (W.warpIndex > 0) W.warpIndex = 0;
        for (int k = 0; k < 8 && !v->destroyed && v->sas == SAS::Ascent; k++) {
            W.update(0.05);
            tfStaging(app, v);
        }
        if (W.t - tLift > 6000) { // (from the lift-off: the rocket waited on the pad for the window)
            check(false, "the ascent reaches an orbit");
            return false;
        }
        if (v->sas == SAS::Ascent) return true;
        Vessel* t = target();
        Conic c = stateToConic(v->r, v->v, eb.GM);
        double ri = t ? relInc(*v, *t) : 99;
        std::fprintf(T.log, "launchwin: %s: %.0f x %.0f km, inclination to the equator %.2f deg, %.2f deg between the planes\n",
                     stage == 2 ? "SAS Ascent at the window" : "SAS Ascent off the window (6 h later)", (c.pe - eb.radius) / 1e3, (c.ap - eb.radius) / 1e3,
                     std::acos(std::clamp(dot(norm(cross(v->r, v->v)), eb.poleFrame.col(2)), -1.0, 1.0)) / DEG, ri);
        if (stage == 2) {
            check(c.valid && c.e < 1 && c.pe - eb.radius > 150e3, "the ascent at the window reaches orbit");
            check(ri < 0.6, "the ascent at the window puts the rocket into the target's plane");
            relOff = ri;
            // the same off the window: the old due-east ascent
            Vessel* a = placeTarget(51.6, 40);
            navg::Guidance g = navg::compute(W, *a, *target(), 0, 1.0);
            W.t = g.tLaunch + 6 * 3600;
            W.refreshStates();
            a = W.activeVessel();
            a->sas = SAS::Ascent;
            a->throttle = 1;
            if (!a->ignited) W.stage(*a);
            tLift = W.t;
            flightOnEnter(app);
            stage = 4;
            return true;
        }
        check(ri > 10, "off the window the ascent flies due east as before (a large plane difference)");
        // night vision on the map
        flightDebugSetMap(true, 3.0e7);
        flightDebugNightMode(1);
        stage = 5;
        T.t0 = T.frames;
        return true;
    }
    if (stage == 5) {
        if (T.frames < T.t0 + 4) return true;
        Image im = LoadImageFromScreen();
        double r = 0, gsum = 0, bl = 0;
        int n = 0;
        Color* px = LoadImageColors(im);
        // (the Earth in the middle of the picture: the panels around are drawn after the night vision)
        for (int y = im.height * 45 / 100; y < im.height * 70 / 100; y += 5)
            for (int x = im.width * 27 / 100; x < im.width * 48 / 100; x += 5) {
                Color c = px[y * im.width + x];
                r += c.r;
                gsum += c.g;
                bl += c.b;
                n++;
            }
        UnloadImageColors(px);
        UnloadImage(im);
        r /= n; gsum /= n; bl /= n;
        std::fprintf(T.log, "launchwin: the map with the image intensifier: mean colour %.1f %.1f %.1f\n", r, gsum, bl);
        check(gsum > 1.25 * r + 2 && gsum > 1.25 * bl + 2, "night vision works on the map (a green picture)");
        shot("92_map_night_vision.png");
        flightDebugNightMode(2);
        stage = 6;
        T.t0 = T.frames;
        return true;
    }
    if (stage == 6) {
        if (T.frames < T.t0 + 4) return true;
        shot("93_map_night_bright.png");
        flightDebugNightMode(0);
        flightDebugSetMap(false, 0);
        std::fprintf(T.log, "launchwin: %s (%d failed)\n", bad ? "FAIL" : "all ok", bad);
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ realistic shadows (--shadowtest)
// The physically based mode with shadow maps: a rocket on the Tanegashima pad in the afternoon sun, a rover on the
// Moon, two vessels side by side in orbit - each with the shadows on and off (the blob shadows). The map must be
// available; with it the ground under the scene is darker where the shadows fall; the frame cost is logged.
bool shadowTestStep(App& app) {
    World& W = app.world;
    const Universe& U = W.U;
    static int bad = 0, scene = 0;
    static double msOn = 0;
    static std::vector<float> lumOn;
    auto check = [&](bool ok, const std::string& what) {
        std::fprintf(T.log, "shadowtest: %s -> %s\n", what.c_str(), ok ? "ok" : "FAIL");
        std::fflush(T.log);
        bad += !ok;
    };
    // the luminance of the view on a grid (the HUD hidden)
    auto lum = [&]() {
        std::vector<float> out;
        Image im = LoadImageFromScreen();
        Color* px = LoadImageColors(im);
        for (int y = 0; y < im.height; y += 4)
            for (int x = 0; x < im.width; x += 4) {
                Color c = px[y * im.width + x];
                out.push_back(0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b);
            }
        UnloadImageColors(px);
        UnloadImage(im);
        return out;
    };
    int earth = U.find("Earth"), moon = U.find("Moon");
    const char* names[] = {"pad", "moon", "orbit"};
    auto setup = [&](int sc) {
        W.newGame(Settings{});
        app.gameStarted = true;
        if (sc == 0) {
            W.launch(craftFromDesign(builtinTemplates()[0]));
            sunAt(W, earth, siteDir(30.4009, 130.97), 30);
            app.screen = Screen::Flight;
            flightOnEnter(app);
            flightDebugView(2.3, 0.35, 95, Vec3d{0, 0, 10});
        } else if (sc == 1) {
            Craft rover = craftNamed("Lunar Rover");
            sunAt(W, moon, siteDir(26.13, 3.63), 20);
            W.launchAircraft(rover, moon);
            app.screen = Screen::Flight;
            flightOnEnter(app);
            flightDebugView(1.2, 0.45, 14, Vec3d{});
        } else {
            W.launchInOrbit(craftNamed("Station Core Module"), -1, 0);
            int core = W.activeVessel()->id;
            W.launchInOrbit(craftNamed("Crew Ferry"), core, 25);
            // on the day side: both coast on until the sun shines on them
            for (int k = 0; k < 60; k++) {
                bool sh = false;
                Vec3d dir;
                W.solarFluxAt(W.activeVessel()->ref, W.activeVessel()->r, &sh, &dir);
                if (!sh) break;
                for (Vessel& v : W.vessels) keplerPropagate(v.r, v.v, U.bodies[v.ref].GM, 120);
                W.t += 120;
                W.refreshStates();
            }
            app.screen = Screen::Flight;
            flightOnEnter(app);
            flightDebugView(0.8, 0.2, 60, Vec3d{});
        }
        app.hudHidden = true;
    };
    int f = T.frames;
    if (f == 2) {
        bad = 0;
        scene = 0;
        app.physicalShading = gfx::setPhysicalShading(true);
        app.realShadows = gfx::setShadows(true, 1);
        check(app.physicalShading && gfx::shadowsActive(), "physically based shading with realistic shadows is on");
        setup(0);
        return true;
    }
    // each scene: frames +0 setup .. +6 shadows on (shot, luminance, cost) .. +12 off (the same)
    int base = 2 + scene * 16, k = f - base;
    if (k == 6) T.t0 = GetTime();
    if (k == 8) {
        msOn = (GetTime() - T.t0) * 1000 / 2;
        lumOn = lum();
        shot(("94_shadows_" + std::string(names[scene]) + "_on.png").c_str());
        gfx::setShadows(false, 1);
        return true;
    }
    if (k == 12) T.t0 = GetTime();
    if (k == 14) {
        double ms = (GetTime() - T.t0) * 1000 / 2;
        std::vector<float> l = lum();
        shot(("94_shadows_" + std::string(names[scene]) + "_off.png").c_str());
        // how much of the picture the shadows change (darker by more than 8 of 255)
        double darker = 0;
        size_t n = std::min(l.size(), lumOn.size());
        for (size_t q = 0; q < n; q++) darker += lumOn[q] < l[q] - 8 ? 1 : 0;
        darker /= std::max<size_t>(n, 1);
        std::fprintf(T.log, "shadowtest: %s: %.1f %% of the picture darker with the shadows; ~%.1f ms per frame with them, ~%.1f ms without\n",
                     names[scene], darker * 100, msOn, ms);
        if (scene < 2) check(darker > 0.003, std::string("the shadows show in the ") + names[scene] + " scene");
        gfx::setShadows(true, 1);
        if (++scene >= 3) {
            // the shadow pass inside the night-vision target (it must draw back into that target afterwards)
            setup(0);
            flightDebugNightMode(2);
            return true;
        }
        setup(scene);
        return true;
    }
    if (scene == 3 && k == 6) {
        std::vector<float> l = lum();
        double mean = 0;
        for (float x : l) mean += x;
        mean /= std::max<size_t>(l.size(), 1);
        shot("94_shadows_pad_nightvision.png");
        std::fprintf(T.log, "shadowtest: the pad with night vision (bright) and the shadows: mean luminance %.1f\n", mean);
        check(mean > 20, "the shadows work with night vision (the view is drawn)");
        flightDebugNightMode(0);
        app.hudHidden = false;
        std::fprintf(T.log, "shadowtest: %s (%d failed)\n", bad ? "FAIL" : "all ok", bad);
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ rovers (--rovertest)
// the lunar rover at Hadley and the Mars rover at Jezero: drive (W), steer (D), stop and park (keys
// released), drive off again, reverse (S); speed, heading, battery and the warp allowed while rolling
bool roverTestStep(App& app) {
    World& W = app.world;
    struct R { const char* craft; const char* body; double lat, lon, driveS, vmax; };
    static const R L[] = {{"Lunar Rover", "Moon", 26.13, 3.63, 25, 3.6}, {"Mars Rover", "Mars", 18.4, 77.5, 90, 0.04}};
    int ri = T.tfRound;
    if (ri >= 2) return false;
    const R& r = L[ri];
    if (T.frames == 2) {
        W.newGame(Settings{});
        app.gameStarted = true;
        Craft c = craftNamed(r.craft);
        app.lastLaunched = c;
        int body = W.U.find(r.body);
        sunAt(W, body, siteDir(r.lat, r.lon), 40);
        W.launchAircraft(c, body);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        flightDebugView(0.8, 0.3, 9, Vec3d{});
        return true;
    }
    if (T.frames != 6) return true;
    Vessel* v = W.activeVessel();
    const Body& b = W.U.bodies[v->ref];
    auto heading = [&]() {
        Vec3d up = norm(v->r), n = v->q.rotate({0, 0, 1});
        Vec3d pole = b.poleFrame.col(2);
        Vec3d east = norm(cross(pole, up)), north = cross(up, east);
        return std::atan2(dot(n, east), dot(n, north)) / DEG;
    };
    auto drive = [&](double pitch, double yaw, double secs) {
        ControlInput in;
        in.pitch = pitch;
        in.yaw = yaw;
        for (int k = 0; k < (int)(secs / 0.05) && !v->destroyed; k++) {
            W.input = in;
            flightDebugInput(&in);
            W.update(0.05);
            v = W.activeVessel();
        }
    };
    auto speed = [&]() { return dot(W.surfaceVel(*v), v->q.rotate({0, 0, 1})); };
    Vec3d p0 = W.U.bodyRot(v->ref, W.t).T() * v->r;
    double ch0 = v->charge, h0 = heading();
    std::fprintf(T.log, "rover: %s on %s, %.0f kg, drive %.0f W, top speed %.2f m/s, landed %d, battery %.2f kWh\n", r.craft, r.body, W.vesselMass(*v),
                 v->drivePower, v->driveVmax, (int)v->landed, v->charge / 3.6e6);
    drive(1, 0, r.driveS); // W: forward
    double s1 = speed();
    Vec3d p1 = W.U.bodyRot(v->ref, W.t).T() * v->r;
    W.setWarp(WARP_COUNT - 1);
    int warpRolling = W.warpIndex;
    W.setWarp(0);
    std::fprintf(T.log, "rover: after %.0f s on W: speed %.3f m/s (top %.2f), %.1f m driven, rolling %d, warp allowed x%g -> %s\n", r.driveS, s1, r.vmax, len(p1 - p0),
                 (int)v->rolling, WARP_LEVELS[warpRolling], s1 > r.vmax * 0.8 && s1 < r.vmax * 1.05 && v->rolling ? "ok" : "FAIL");
    char name[64];
    std::snprintf(name, sizeof name, "29_%d_driving.png", ri);
    shot(name);
    drive(1, -1, r.vmax > 1 ? 6 : 60); // W + D (yaw -1, see flight.cpp): a right turn
    double h1 = heading();
    double dh = h1 - h0;
    while (dh > 180) dh -= 360;
    while (dh < -180) dh += 360;
    std::fprintf(T.log, "rover: steering (D) turned the heading by %.1f deg (to the right: +) -> %s\n", dh, dh > 3 ? "ok" : "FAIL");
    drive(0, 0, r.vmax > 1 ? 15 : 10); // released: brakes, stop, park
    std::fprintf(T.log, "rover: keys released: speed %.3f m/s, parked (landed) %d -> %s\n", speed(), (int)v->landed, v->landed ? "ok" : "FAIL");
    drive(-1, 0, r.vmax > 1 ? 6 : 30); // S: reverse from the parked state
    double s2 = speed();
    std::fprintf(T.log, "rover: S from parked: speed %.3f m/s, rolling %d -> %s\n", s2, (int)v->rolling, s2 < -0.01 && v->rolling ? "ok" : "FAIL");
    drive(0, 0, 10);
    std::fprintf(T.log, "rover: battery %.2f -> %.2f kWh, destroyed %d -> %s\n", ch0 / 3.6e6, v->charge / 3.6e6, (int)v->destroyed,
                 v->charge < ch0 && !v->destroyed ? "ok" : "FAIL");
    flightDebugInput(nullptr);
    T.tfRound++;
    T.frames = 0;
    return true;
}

// ------------------------------------------------------------------ surface feature labels (--labeltest)
// every named feature checked against the relief (a mountain higher than its surroundings, a basin, sea or
// crater lower), then a few views with the labels on
bool labelTestStep(App& app) {
    World& W = app.world;
    if (T.frames == 2) {
        W.newGame(Settings{});
        app.gameStarted = true;
        Design d = builtinTemplates()[2];
        app.design = d;
        app.lastLaunched = craftFromDesign(d);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        int nf = 0;
        const features::Feature* F = features::all(nf);
        int bad = 0, checked = 0;
        for (int k = 0; k < nf; k++) {
            const features::Feature& f = F[k];
            int body = W.U.find(f.body);
            if (body < 0 || !terrain::ensure(body)) continue; // (no relief: Venus, Titan)
            const Body& b = W.U.bodies[body];
            bool high = f.kind == features::Mons || f.kind == features::Range || f.kind == features::Plateau;
            bool low = f.kind == features::Basin || f.kind == features::Mare || f.kind == features::Crater || f.kind == features::Planitia ||
                       f.kind == features::Valley;
            if (!high && !low) continue;
            Vec3d c = siteDir(f.lat, f.lon);
            Vec3d e = norm(cross(Vec3d{0, 0, 1}, c) + Vec3d{1e-9, 0, 0}), n = cross(c, e);
            double r = f.radiusKm * 1000 / b.radius, hc = 0, hr = 0;
            int nc = 0;
            for (int i = 0; i < 9; i++) { // the centre region (inner third)
                double a = i * TAU / 9, q = i ? r * 0.25 : 0;
                hc += terrain::height(body, norm(c + (e * std::cos(a) + n * std::sin(a)) * q));
                nc++;
            }
            hc /= nc;
            for (int i = 0; i < 24; i++) {
                double a = i * TAU / 24;
                hr += terrain::height(body, norm(c + (e * std::cos(a) + n * std::sin(a)) * (r * 1.6)));
            }
            hr /= 24;
            bool ok = high ? hc > hr : hc < hr;
            checked++;
            if (!ok) bad++;
            std::fprintf(T.log, "label: %-8s %-24s %7.2f %8.2f  centre %7.0f m, around %7.0f m -> %s\n", f.body, f.name, f.lat, f.lon, hc, hr,
                         ok ? "ok" : (f.kind == features::Valley || f.kind == features::Mare ? "check (lowlands around)" : "FAIL"));
        }
        std::fprintf(T.log, "label: %d features checked against the relief, %d not as expected\n", checked, bad);
        return true;
    }
    struct V { const char* name; const char* body; double lat, lon, alt, pitch; };
    static const V VIEWS[] = {{"30a_labels_earth.png", "Earth", 30, 80, 2500e3, 1.45}, {"30b_labels_moon.png", "Moon", 15, 0, 2600e3, 1.45},
                              {"30c_labels_mars.png", "Mars", 5, -100, 3000e3, 1.45},  {"30d_labels_mercury.png", "Mercury", 25, 165, 3000e3, 1.45},
                              {"30e_labels_pluto.png", "Pluto", 20, 175, 2500e3, 1.45}, {"30f_labels_callisto.png", "Callisto", 15, -56, 4000e3, 1.45},
                              {"30g_labels_mars_low.png", "Mars", 16, -128, 400e3, 0.35}};
    const int nv = sizeof VIEWS / sizeof VIEWS[0];
    int vi = (T.frames - 4) / 16, k = (T.frames - 4) % 16;
    if (T.frames < 4) return true;
    if (vi >= nv) return false;
    const V& vw = VIEWS[vi];
    Vessel* v = W.activeVessel();
    if (!v) return false;
    int body = W.U.find(vw.body);
    if (k == 0) {
        Vec3d f = siteDir(vw.lat, vw.lon);
        sunAt(W, body, f, 40);
        placeAt(W, *v, body, f, vw.alt);
        flightDebugView(0.3, vw.pitch, 40, Vec3d{});
    }
    if (k == 15) shot(vw.name);
    return true;
}

// ------------------------------------------------------------------ music (--bgmtest)
// the tracks and effect files load; a track starts after the pause, ends, a pause of random length
// follows, the next track is another one (played at an inaudible volume)
bool bgmTestStep(App& app) {
    (void)app;
    static int round = 0, prev = -1, repeats = 0;
    static double t0 = 0;
    if (T.frames == 2) {
        int n = sfx::musicTracks();
        std::fprintf(T.log, "bgm: %d tracks", n);
        for (int k = 0; k < n; k++) std::fprintf(T.log, "  [%d] %.0f s", k, sfx::musicLength(k));
        std::fprintf(T.log, " -> %s\n", n == 3 ? "ok" : "FAIL");
        std::fprintf(T.log, "bgm: %d of %d effects from src/audio/fx -> %s\n", sfx::effectsFromFiles(), (int)sfx::Ev::COUNT,
                     sfx::effectsFromFiles() == (int)sfx::Ev::COUNT ? "ok" : "FAIL");
        sfx::setMusic(true, 1e-4f);
        std::fprintf(T.log, "bgm: first pause %.1f s\n", sfx::musicPause());
        sfx::musicDebugSkipPause();
        t0 = GetTime();
        return true;
    }
    if (T.frames < 3) return true;
    int k = sfx::musicPlaying();
    if (k >= 0 && GetTime() - t0 > 1.5) {
        std::fprintf(T.log, "bgm: round %d: track %d playing%s\n", round, k, k == prev ? " (the same as before -> FAIL)" : "");
        if (k == prev) repeats++;
        prev = k;
        sfx::musicDebugSeekEnd();
        t0 = GetTime() + 1e9;
        return true;
    }
    if (k < 0 && t0 > GetTime() + 1e8) {
        float pz = sfx::musicPause();
        std::fprintf(T.log, "bgm: round %d: the track ended, pause %.0f s before the next -> %s\n", round, pz, pz >= 60 && pz <= 300 ? "ok" : "FAIL");
        round++;
        if (round >= 6) {
            std::fprintf(T.log, "bgm: %d rounds, %d immediate repeats -> %s\n", round, repeats, repeats == 0 ? "ok" : "FAIL");
            sfx::setMusic(false, 0);
            return false;
        }
        sfx::musicDebugSkipPause();
        t0 = GetTime();
    }
    if (T.frames > 20000) {
        std::fprintf(T.log, "bgm: no progress -> FAIL\n");
        return false;
    }
    return true;
}

// ------------------------------------------------------------------ TLI cut-off experiment (--tliexp)
// a parking orbit in the Moon's plane; burn prograde from the point opposite the Moon's future place
// and log how the predicted lunar periapsis evolves with the apoapsis
bool tliExpStep(App& app) {
    World& W = app.world;
    int earth = W.U.find("Earth"), moon = W.U.find("Moon");
    if (T.frames == 2) {
        W.newGame(Settings{});
        app.gameStarted = true;
        Design d = builtinTemplates()[2]; // Selene: everything above the booster
        int dec = CATALOG.find("decoupler"), seen = 0;
        for (size_t i = 0; i < d.stack.size(); i++)
            if (d.stack[i].def == dec && ++seen > 1) { d.stack.resize(i); break; }
        app.lastLaunched = craftFromDesign(d);
        W.launch(app.lastLaunched);
        app.screen = Screen::Flight;
        flightOnEnter(app);
        Vessel* v = W.activeVessel();
        W.jettisonFairing(*v, false);
        v = W.activeVessel();
        // circular 200 km orbit in the Moon's orbit plane
        Vec3d m = W.U.relPos(W.states, moon, earth), mv = W.U.relVel(W.states, moon, earth);
        Vec3d n = norm(cross(m, mv));
        Vec3d rh = norm(cross(n, m));
        double r = W.U.bodies[earth].radius + 200e3;
        v->landed = false;
        v->r = rh * r;
        v->v = norm(cross(n, rh)) * std::sqrt(W.U.bodies[earth].GM / r);
        v->q = Quatd::lookRot(norm(v->v), rh);
        v->w = {};
        v->sas = SAS::Prograde;
        v->ignited = false;
        v->throttle = 0;
        W.history.clear();
        T.tfSub = 0;
        T.tfRef = 0;
        return true;
    }
    if (T.frames < 4) return true;
    Vessel* v = W.activeVessel();
    if (!v || v->destroyed) return false;
    const Body& eb = W.U.bodies[earth];
    Conic c = stateToConic(v->r, v->v, eb.GM);
    std::vector<BodyState> S2;
    W.U.computeStatesFor(W.t + 5 * DAY, W.U.relevant[earth], S2);
    Vec3d target = -norm(W.U.relPos(S2, moon, earth));
    double ang = std::acos(std::clamp(dot(norm(v->r), target), -1.0, 1.0)) / DEG;
    bool before = dot(cross(norm(v->r), target), c.H) > 0;
    if (T.tfSub == 0) {
        // coast (warp) until 4 degrees before that point
        if (before && ang < 4.0) {
            W.warpIndex = 0;
            T.tfSub = 1;
            v->throttle = 1;
            W.stage(*v);
            std::fprintf(T.log, "tliexp: ignition %.1f deg before the antipode of the Moon in 5 days\n", ang);
        } else {
            W.warpIndex = ang > 20 ? std::min(6, W.maxWarpIndex()) : 2;
            for (int k = 0; k < 4; k++) W.update(0.05);
        }
        return true;
    }
    for (int k = 0; k < 2 && !v->destroyed; k++) {
        pointAlong(*v, norm(v->v));
        W.update(0.05);
        tfStaging(app, v);
    }
    c = stateToConic(v->r, v->v, eb.GM);
    double md = len(W.U.relPos(W.states, moon, earth));
    Encounter e;
    bool enc = predictEncounter(W.U, v->ref, v->r, v->v, W.t, e) && e.body == moon;
    double pe = 0, tilt = 0;
    if (enc) {
        Conic k = stateToConic(e.r, e.v, W.U.bodies[moon].GM);
        pe = k.pe - W.U.bodies[moon].radius;
        double inc = std::acos(std::clamp(dot(k.H, W.U.bodies[moon].poleFrame.col(2)), -1.0, 1.0)) / DEG;
        tilt = std::min(inc, 180 - inc);
    }
    if (T.frames % 3 == 0 || enc != (T.tfRef > 0))
        std::fprintf(T.log, "tliexp: apo %.0f km (Moon %.0f km, ratio %.4f) speed %.1f m/s enc %d pe %.0f km tilt %.1f\n", (c.ap - eb.radius) / 1000, md / 1000,
                     c.ap / md, len(v->v), (int)enc, pe / 1000, tilt);
    T.tfRef = enc ? 1 : 0;
    // the cut-off of the tutorial: apoapsis = the Moon's distance on arrival (Hohmann transfer time)
    {
        double rr = len(v->r), aT = 0.5 * (rr + md);
        double tT = PI * std::sqrt(aT * aT * aT / eb.GM);
        W.U.computeStatesFor(W.t + tT, W.U.relevant[earth], S2);
        double target = len(W.U.relPos(S2, moon, earth));
        if (c.valid && c.e < 1 && c.ap >= target * 0.996) {
            v->throttle = 0;
            std::fprintf(T.log, "tliexp: cut at apo %.0f km (target %.0f km), encounter %d pe %.0f km\n", (c.ap - eb.radius) / 1000, (target - eb.radius) / 1000,
                         (int)enc, pe / 1000);
            const Body& mb = W.U.bodies[moon];
            Vec3d pro = norm(v->v), nrm = norm(cross(v->r, v->v));
            double t0 = GetTime();
            for (int axis = 0; axis < 2; axis++)
                for (double d = -60; d <= 60; d += 5) {
                    Vec3d dV = (axis == 0 ? pro : nrm) * d;
                    Encounter e2;
                    double p2 = -1;
                    bool ok = predictEncounter(W.U, v->ref, v->r, v->v + dV, W.t, e2) && e2.body == moon;
                    if (ok) p2 = (stateToConic(e2.r, e2.v, mb.GM).pe - mb.radius) / 1000;
                    std::fprintf(T.log, "tliexp: %s %+5.0f m/s -> %s pe %.0f km\n", axis == 0 ? "prograde" : "normal  ", d, ok ? "enc" : "no enc", p2);
                }
            std::fprintf(T.log, "tliexp: %.1f ms per prediction\n", (GetTime() - t0) * 1000 / 50);
            return false;
        }
    }
    if (!c.valid || c.e >= 1 || c.ap > 1.4 * md || T.frames > 20000) return false;
    return true;
}
} // namespace

bool autotestInit(App& app, int argc, char** argv) {
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == "--autotest") {
            T.on = true;
            T.dir = argv[i + 1];
        }
    if (T.on) {
        // the player's graphics settings do not apply: every run starts from the defaults (the flags below
        // choose others)
        app.terrainQuality = 2;
        app.detailRange = 100;
        app.physicalShading = false;
        gfx::setTerrainQuality(2);
        gfx::setDetailRange(1.0f);
        gfx::setPhysicalShading(false);
    }
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == "--lang") {
            std::string l = argv[i + 1];
            app.lang = l == "ja" ? i18n::JA : (l == "de" ? i18n::DE : i18n::EN);
            ui::setLanguage(app.lang);
        }
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == "--textsize") {
            app.textSize = std::clamp(std::atoi(argv[i + 1]), 0, 3);
            ui::setTextScale(TEXT_SCALES[app.textSize]);
        }
    if (!T.on) return false;
    std::error_code ec;
    std::filesystem::create_directories(T.dir, ec);
    T.log = std::fopen((T.dir + "\\autotest.log").c_str(), "w");
    if (!T.log) T.log = stderr;
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--cadonly") T.phase = 100;
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--rewindtest") T.phase = 220;
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--terraintest") T.phase = 230;
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--perftest") T.phase = 240;
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--airtest") { T.phase = 250; T.tf = -1; }
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--tliexp") T.phase = 260;
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--warptest") T.phase = 270;
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--tfkeys") T.tfKeys = true;
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--picktest") T.phase = 280;
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--ctrltest") T.phase = 290;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--spaceplanetest") { T.phase = 340; T.tf = -1; }
        if (std::string(argv[i]) == "--docktest") T.phase = 350;
        if (std::string(argv[i]) == "--savetest") T.phase = 360;
        if (std::string(argv[i]) == "--skytest") T.phase = 370;
        if (std::string(argv[i]) == "--trusstest") T.phase = 380;
        if (std::string(argv[i]) == "--orbitlinetest") T.phase = 390;
        if (std::string(argv[i]) == "--navtest") T.phase = 400;
        if (std::string(argv[i]) == "--achtest") T.phase = 410;
        if (std::string(argv[i]) == "--duptest") T.phase = 420;
        if (std::string(argv[i]) == "--seatest") T.phase = 430;
        if (std::string(argv[i]) == "--realtest") T.phase = 440;
        if (std::string(argv[i]) == "--uitest") T.phase = 450;
        if (std::string(argv[i]) == "--basetest") T.phase = 490;
        if (std::string(argv[i]) == "--missiontest") T.phase = 500;
        if (std::string(argv[i]) == "--escapetest") T.phase = 510;
        if (std::string(argv[i]) == "--rdvtest") T.phase = 520;
        if (std::string(argv[i]) == "--rdvsweep") T.phase = 530;
        if (std::string(argv[i]) == "--launchwintest") T.phase = 550;
        if (std::string(argv[i]) == "--shadowtest") T.phase = 560;
        if (std::string(argv[i]) == "--savecheck" && i + 1 < argc) { T.phase = 540; g_saveCheck = argv[i + 1]; }
        if (std::string(argv[i]) == "--solartest") T.phase = 460;
        if (std::string(argv[i]) == "--startest") T.phase = 470;
        if (std::string(argv[i]) == "--bodytest") T.phase = 480;
        if (std::string(argv[i]) == "--spsub" && i + 1 < argc) g_spOnly = std::atoi(argv[i + 1]);
        if (std::string(argv[i]) == "--spalt" && i + 1 < argc) g_spAltKm = std::clamp(std::atof(argv[i + 1]), 150.0, 2000.0);
    }
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--designtest") T.phase = 300;
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--rovertest") { T.phase = 310; T.tfRound = 0; }
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--labeltest") T.phase = 320;
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--bgmtest") T.phase = 330;
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == "--terrainq") gfx::setTerrainQuality(std::atoi(argv[i + 1]));
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == "--detailrange") gfx::setDetailRange((float)std::atof(argv[i + 1]) / 100.0f);
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--pbr" && !gfx::setPhysicalShading(true)) std::fprintf(T.log ? T.log : stderr, "pbr: the shaders did not compile -> FAIL\n");
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--orbitall") { // every template to a parking orbit
            T.phase = 210;
            T.tf = 0;
        }
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == "--tutflight") { // 2, 6, 7 or 0 = 6 and 7
            int n = std::atoi(argv[i + 1]);
            T.phase = 200;
            // tutorial numbers as in the menu: 2 orbit (guided), 3 manual ascent, 6 equatorial, 7 Moon; 0 = 6 and 7
            T.tf = n == 3 ? 2 : n == 7 ? 6 : n == 2 ? 1 : 5;
            T.tfLast = n == 3 ? 2 : n == 6 ? 5 : n == 2 ? 1 : 6;
        }
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == "--kick") T.kickDeg = std::atof(argv[i + 1]);
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == "--warpidx") T.warpIdx = std::atoi(argv[i + 1]);
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == "--warpap") T.warpAp = std::atof(argv[i + 1]) * 1000;
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == "--warppe") T.warpPe = std::atof(argv[i + 1]) * 1000;
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == "--warpbody") T.warpBody = argv[i + 1];
    keys::resetDefaults(); // tests run with the default bindings (the user's keys.txt is not written)
    (void)app;
    return true;
}

// called every frame after drawing; returns false when the test is complete
static void physicsChecks() {
    auto check = [&](const char* name, double got, double want) {
        double err = want != 0 ? (got - want) / want : got;
        std::fprintf(T.log, "check %-34s got=%.6g want=%.6g err=%+.3f%%\n", name, got, want, err * 100);
    };
    auto phys = [](Component c) {
        CompPhys p;
        computeCompPhys(c, p);
        return toMassProps(p.dryM);
    };
    double rho = MATERIALS[0].density;
    {
        Component c = makeComponent(Func::Structure);
        c.shape = Shape::Box; c.dims = {2, 3, 4}; c.hollow = false; c.material = 0;
        MassProps m = phys(c);
        double M = rho * 24;
        check("box mass", m.mass, M);
        check("box Ixx", m.inertia.m[0][0], M * (9 + 16) / 12);
        check("box Izz", m.inertia.m[2][2], M * (4 + 9) / 12);
    }
    {
        Component c = makeComponent(Func::Structure);
        c.shape = Shape::Cylinder; c.dims = {1, 1, 4}; c.hollow = false; c.material = 0;
        MassProps m = phys(c);
        double M = rho * PI * 4;
        check("cylinder mass", m.mass, M);
        check("cylinder Izz", m.inertia.m[2][2], M / 2);
        check("cylinder Ixx", m.inertia.m[0][0], M * (3 + 16) / 12);
    }
    {
        Component c = makeComponent(Func::Structure);
        c.shape = Shape::Cylinder; c.dims = {1, 0, 3}; c.hollow = false; c.material = 0;
        MassProps m = phys(c);
        double M = rho * PI * 3 / 3;
        check("cone mass", m.mass, M);
        check("cone Izz", m.inertia.m[2][2], 0.3 * M);
        check("cone Ixx (about CoM)", m.inertia.m[0][0], M * (3.0 / 20 + 3.0 / 80 * 9));
        check("cone CoM z (from base -1.5)", m.com.z, -1.5 + 0.75);
    }
    {
        Component c = makeComponent(Func::Structure);
        c.shape = Shape::Ellipsoid; c.dims = {1, 2, 3}; c.hollow = false; c.material = 0;
        MassProps m = phys(c);
        double M = rho * 4.0 / 3 * PI * 6;
        check("ellipsoid mass", m.mass, M);
        check("ellipsoid Ixx", m.inertia.m[0][0], M * (4 + 9) / 5);
        check("ellipsoid Izz", m.inertia.m[2][2], M * (1 + 4) / 5);
    }
    {
        Component c = makeComponent(Func::Structure);
        c.shape = Shape::Cylinder; c.dims = {1, 1, 4}; c.hollow = true; c.wall = 0.001; c.material = 0;
        MassProps m = phys(c);
        double ms = 2 * PI * 4 * 0.001 * rho, mc = PI * 0.001 * rho;
        check("thin shell mass", m.mass, ms + 2 * mc);
        check("thin shell Izz", m.inertia.m[2][2], ms + 2 * mc * 0.5);
    }
    {
        // rotated + translated box: parallel-axis theorem
        Component c = makeComponent(Func::Structure);
        c.shape = Shape::Box; c.dims = {1, 1, 1}; c.hollow = false; c.material = 0; c.pos = {3, 0, 0}; c.rotDeg = {0, 0, 45};
        Craft cr;
        cr.comps.push_back(c);
        Component c2 = c;
        c2.pos = {-3, 0, 0};
        cr.comps.push_back(c2);
        CraftStats st = analyzeCraft(cr, nullptr);
        double M = rho;
        check("two boxes Izz (parallel axis)", st.wet.inertia.m[2][2], 2 * (M / 6 + M * 9));
    }
    {
        Component c = makeComponent(Func::Structure);
        c.shape = Shape::Ellipsoid; c.dims = {1, 1, 1};
        auto tab = buildAeroTable({c});
        auto smp = tab->lookup({0, 0, -1}, nullptr);
        check("sphere Newtonian Cd", -smp.F_p.z / PI, 1.0);
        check("sphere frontal area", smp.aproj, PI);
    }
    {
        Component c = makeComponent(Func::Structure);
        c.shape = Shape::Box; c.dims = {2, 2, 0.01};
        auto tab = buildAeroTable({c});
        auto smp = tab->lookup({0, 0, -1}, nullptr);
        check("flat plate Newtonian Cd", -smp.F_p.z / 4.0, 2.0);
    }
    {
        Craft rk = builtinCrafts()[0];
        auto comps = expandCraft(rk);
        auto tab = buildAeroTable(comps);
        CraftStats st = analyzeCraft(rk, tab.get());
        std::fprintf(T.log, "finned rocket: mass=%.0f CoM z=%.2f CoP z=%.2f margin=%.2f cal stages=%zu dv=%.0f\n", st.wet.mass, st.wet.com.z,
                     st.copAscent.z, st.staticMargin, st.stages.size(), st.totalDvVac);
        {
            // payload alone, re-entering shield-first: pitching moment about its CoM vs angle of attack
            Craft pl;
            for (auto& k : rk.comps)
                if (k.stage == 1) pl.comps.push_back(k);
            auto pc = expandCraft(pl);
            auto pt = buildAeroTable(pc);
            CraftStats ps = analyzeCraft(pl, pt.get());
            std::fprintf(T.log, "payload: mass=%.0f CoM z=%.3f\n", ps.wet.mass, ps.wet.com.z);
            for (double a : {5.0, 15.0, 30.0}) {
                Vec3d u{std::sin(a * DEG), 0, std::cos(a * DEG)}; // air flowing up: shield first
                auto smp = pt->lookup(u, nullptr);
                Vec3d F = smp.F_p * 0.92 + smp.F_f, To = smp.T_p * 0.92 + smp.T_f;
                Vec3d Tc = To - cross(ps.wet.com, F);
                std::fprintf(T.log, "  shield-first aoa=%4.0f: F=(%.3f, %.3f, %.3f) My=%.4f (per unit q; restoring if > 0 for shield-first flow)\n", a, F.x, F.y, F.z, Tc.y);
            }
        }
        {
            // worst-case cost: 300 components
            std::vector<Component> big;
            for (int i = 0; i < 300; i++) {
                Component b = makeComponent(Func::Structure);
                b.pos = {(i % 10) * 1.2, ((i / 10) % 10) * 1.2, (i / 100) * 1.2};
                big.push_back(b);
            }
            double t0 = GetTime();
            auto bt = buildAeroTable(big);
            std::fprintf(T.log, "aero table for 300 components: %.2f s\n", GetTime() - t0);
        }
        rk.comps[1].symmetry = 1; // single fin -> asymmetric
        CraftStats st2 = analyzeCraft(rk, nullptr);
        std::fprintf(T.log, "asymmetric fin: CoM=(%.3f, %.3f, %.2f)\n", st2.wet.com.x, st2.wet.com.y, st2.wet.com.z);
    }
    std::fflush(T.log);
}

static void logEvents(World& W) {
    Vessel* v = W.activeVessel();
    if (v)
        for (SimEvent e : W.events) {
            double alt = len(v->r) - W.U.bodies[v->ref].radius;
            if (e == SimEvent::Karman) std::fprintf(T.log, "event: karman line at alt %.0f m\n", alt);
            if (e == SimEvent::OrbitReached) std::fprintf(T.log, "event: orbit class %d around %s (alt %.0f m)\n", (int)v->orbitClass, W.U.bodies[v->ref].name.c_str(), alt);
            if (e == SimEvent::Landed) std::fprintf(T.log, "event: landed on %s\n", W.U.bodies[v->ref].name.c_str());
            if (e == SimEvent::WarpArrived) std::fprintf(T.log, "event: warp arrived\n");
            if (e == SimEvent::Gravity) std::fprintf(T.log, "event: gravity (%s)\n", W.toasts.empty() ? "" : W.toasts.back().text.c_str());
        }
    W.events.clear();
}

bool autotestStep(App& app) {
    if (!T.on) return true;
    World& W = app.world;
    T.frames++;
    Vessel* v = W.activeVessel();
    logEvents(W);
    if (v && v->tHeatFlux > 6e5 && !T.shotEntry) {
        shot("16b_entry_plasma.png");
        T.shotEntry = true;
    }
    switch (T.phase) {
    case 0:
        if (T.frames == 2) physicsChecks();
        if (T.frames == 3) {
            keyBindingChecks();
            screenshot::setFolderOverride(std::filesystem::path(T.dir) / "user_shots");
            keys::inject({KEY_F2, 0}); // the screenshot key, handled by the main loop next frame
        }
        if (T.frames == 9) {
            screenshot::finish();
            namespace fs = std::filesystem;
            int n = 0;
            bool png = false;
            std::error_code ec;
            for (auto& e : fs::directory_iterator(fs::path(T.dir) / "user_shots", ec)) {
                n++;
                std::ifstream f(e.path(), std::ios::binary);
                char sig[8] = {};
                f.read(sig, 8);
                png = png || (std::memcmp(sig, "\x89PNG\r\n\x1a\n", 8) == 0 && fs::file_size(e.path(), ec) > 10000);
            }
            std::fprintf(T.log, "screenshot key: %d file(s), PNG %s, notice \"%s\"\n", n, n == 1 && png ? "ok" : "FAIL", app.notice.c_str());
        }
        if (T.frames == 6) app.settingsOpen = true;
        if (T.frames == 7) app.creditsOpen = true; // Settings -> Credits and licences
        if (T.frames == 8) {
            shot("00e_credits.png");
            ui::setScroll(9002, 1e6f);
        }
        if (T.frames == 9) {
            shot("00f_credits_end.png");
            ui::setScroll(9002, 0);
            const void* data = nullptr;
            size_t size = 0;
            bool ok = winshell::resource(401, data, size) && size > 20000;
            std::fprintf(T.log, "credits and licences: notices %zu bytes, %s\n", size, ok ? "ok" : "FAIL");
            app.creditsOpen = false;
        }
        if (T.frames == 10) app.keysOpen = true;
        if (T.frames == 13) {
            shot("00c_key_bindings.png");
            app.keysOpen = false;
        }
        if (T.frames == 14) {
            shot("00_settings.png");
            ui::setScroll(7200, 1e6f); // (the end of the settings, when they scroll)
        }
        if (T.frames == 15) {
            shot("00d_settings_end.png");
            ui::setScroll(7200, 0);
            app.settingsOpen = false;
            // show the crash dialog with the previous report (if any) without marking it seen
            std::string text, path;
            if (crash::pendingReport(text, path)) { app.crashReport = text; app.crashPath = path; }
        }
        if (T.frames == 19 && !app.crashReport.empty()) {
            shot("00b_crash_dialog.png");
            app.crashReport.clear();
        }
        if (T.frames == 20) {
            shot("01_menu.png");
            W.newGame(Settings{});
            app.gameStarted = true;
            app.design = builtinTemplates()[0];
            app.screen = Screen::Factory;
            T.phase = 1;
            T.frames = 0;
        }
        break;
    case 1:
        // factory undo / redo: an edit becomes one history step once it is finished
        if (T.frames == 5) {
            T.undoT0 = designToText(app.design);
            if (!app.design.stack.empty()) app.design.stack.pop_back(); // an edit
        }
        if (T.frames == 8) {
            T.undoT1 = designToText(app.design);
            factoryDebugUndo(app, false);
            bool undoOk = designToText(app.design) == T.undoT0;
            factoryDebugUndo(app, true);
            bool redoOk = designToText(app.design) == T.undoT1;
            factoryDebugUndo(app, false);
            std::fprintf(T.log, "factory undo: %s, redo: %s, restored: %s\n", undoOk ? "ok" : "FAIL", redoOk ? "ok" : "FAIL",
                         designToText(app.design) == T.undoT0 ? "ok" : "FAIL");
        }
        // the same through the bindings: mouse back (button 4) = undo, mouse forward (button 5) = redo,
        // injected as real presses that the factory screen reads in the next frame
        if (T.frames == 9) {
            T.undoT0 = designToText(app.design);
            if (!app.design.stack.empty()) app.design.stack.pop_back();
        }
        if (T.frames == 12) {
            T.undoT1 = designToText(app.design);
            keys::inject({keys::MOUSE_BASE + MOUSE_BUTTON_SIDE, 0});
        }
        if (T.frames == 13) {
            T.mouseUndoOk = designToText(app.design) == T.undoT0;
            keys::inject({keys::MOUSE_BASE + MOUSE_BUTTON_EXTRA, 0});
        }
        if (T.frames == 14) {
            T.mouseRedoOk = designToText(app.design) == T.undoT1;
            keys::inject({KEY_Z, keys::Ctrl}); // Ctrl+Z
        }
        if (T.frames == 15) {
            bool ctrlZ = designToText(app.design) == T.undoT0;
            std::fprintf(T.log, "bindings undo/redo: mouse back undo %s, mouse forward redo %s, Ctrl+Z undo %s\n", T.mouseUndoOk ? "ok" : "FAIL",
                         T.mouseRedoOk ? "ok" : "FAIL", ctrlZ ? "ok" : "FAIL");
        }
        if (T.frames == 20) {
            shot("02_factory.png");
            for (auto& d : builtinTemplates()) {
                DesignStats st = analyzeDesign(d);
                std::fprintf(T.log, "template %-24s mass=%.0f dv=%.0f stage1TWR=%.2f warnings=%zu\n", d.name.c_str(), st.wetMass, st.totalDvVac,
                             st.stages.empty() ? 0 : st.stages[0].twrSL, st.warnings.size());
                for (auto& w : st.warnings) std::fprintf(T.log, "    warning: %s\n", w.c_str());
                // does the stowed payload fit inside the fairing? (inner skin 1.5 % inside the outer)
                Craft cr = craftFromDesign(d);
                std::vector<Component> geo = stowedGeometry(cr.comps);
                struct FS { double z0, z1, rb, rt; };
                std::vector<FS> fs;
                double fz0 = 1e30, fz1 = -1e30;
                for (auto& k : geo)
                    if (k.func == Func::Fairing && std::cos(k.rotDeg.z * DEG) > 0) {
                        fs.push_back({k.pos.z - k.dims.z / 2, k.pos.z + k.dims.z / 2, k.dims.x, k.dims.y});
                        fz0 = std::min(fz0, k.pos.z - k.dims.z / 2);
                        fz1 = std::max(fz1, k.pos.z + k.dims.z / 2);
                    }
                if (fs.empty()) continue;
                auto innerR = [&](double zz) {
                    double r = -1;
                    for (auto& f : fs)
                        if (zz >= f.z0 - 1e-6 && zz <= f.z1 + 1e-6) r = std::max(r, (f.rb + (f.rt - f.rb) * (zz - f.z0) / std::max(f.z1 - f.z0, 1e-9)) * 0.985);
                    return r;
                };
                double worst = -1e9;
                std::string worstName;
                std::vector<Vec3d> pts;
                for (auto& k0 : geo) {
                    if (k0.func == Func::Fairing || k0.pos.z <= fz0 + 1e-3) continue;
                    Component k = k0.func == Func::Legs ? stowedLeg(k0) : k0;
                    pts.clear();
                    outlinePoints(k, pts, 36);
                    double worstK = -1e9;
                    for (auto& p : pts) {
                        if (p.z < fz0 + 1e-3) continue;
                        double rr = std::hypot(p.x, p.y);
                        double ri = p.z > fz1 ? -1 : innerR(p.z);
                        worstK = std::max(worstK, ri < 0 ? rr + (p.z - fz1) + 0.01 : rr - ri);
                    }
                    if (worstK > worst) { worst = worstK; worstName = k.name; }
                }
                std::fprintf(T.log, "    fairing fit: %s (tightest: %s, %+.3f m %s)\n", worst <= 0 ? "ok" : "FAIL", worstName.c_str(), worst,
                             worst <= 0 ? "clearance" : "outside");
            }
            app.lastLaunched = craftFromDesign(app.design);
            W.launch(app.lastLaunched);
            flightOnEnter(app);
            app.screen = Screen::Flight;
            T.phase = 2;
            T.frames = 0;
        }
        break;
    case 2:
        if (T.frames == 12) W.t += 43200; // half a day later: the other side of the day / night line
        if (T.frames == 24) {
            const Body& eb = W.U.bodies[v->ref];
            double el = dot(norm(v->r), W.sunDirFrom(v->ref, v->r));
            std::fprintf(T.log, "pad +12 h: sun elevation %.1f deg\n", std::asin(std::clamp(el, -1.0, 1.0)) / DEG);
            (void)eb;
            shot("03d_pad_half_day_later.png");
            flightDebugNightVision(true);
        }
        if (T.frames == 25) {
            shot("03i_pad_night_vision.png");
            flightDebugNightVision(false);
            W.t -= 43200;
        }
        // the ground in daylight (sun ~40 degrees high): the pad view and a low view towards the horizon
        if (T.frames == 26) {
            const Body& eb = W.U.bodies[v->ref];
            int earth = v->ref;
            Vec3d s = W.sunDirFrom(earth, v->r);
            double best = -1e9, bestDt = 0;
            for (int k = 0; k < 48; k++) {
                double dt = k * 1800.0;
                Vec3d up = W.U.bodyRot(earth, W.t + dt) * norm(v->landPos);
                double el = std::asin(std::clamp(dot(up, s), -1.0, 1.0)) / DEG;
                double score = -std::fabs(el - 40.0);
                if (score > best) { best = score; bestDt = dt; }
            }
            (void)eb;
            T.tSave = W.t;
            W.t += bestDt;
            flightDebugGetView(T.vYaw, T.vPitch, T.vDist, T.vTarget);
        }
        if (T.frames == 27) {
            Vec3d up = norm(v->r), east = norm(cross(Vec3d{0, 0, 1}, up)), north = cross(up, east);
            Vec3d s = W.sunDirFrom(v->ref, v->r);
            std::fprintf(T.log, "daylight ground: sun elevation %.1f deg\n", std::asin(std::clamp(dot(up, s), -1.0, 1.0)) / DEG);
            shot("03e_pad_day.png");
            double sy = std::atan2(dot(s, north), dot(s, east));
            flightDebugView(sy + 0.7, 0.03, 75, Vec3d{0, 0, -26});
        }
        if (T.frames == 29) {
            shot("03f_ground_day_low.png");
            flightDebugView(T.vYaw, 1.35, 170, Vec3d{0, 0, -20}); // from above: the vehicle's shadow
        }
        if (T.frames == 31) {
            shot("03g_pad_day_top.png");
            flightDebugView(T.vYaw, 0.35, 3500, Vec3d{}); // the surroundings from a few km
        }
        if (T.frames == 33) shot("03h_pad_day_far.png");
        if (T.frames == 34 && T.tSave != 0) {
            W.t = T.tSave;
            flightDebugView(T.vYaw, T.vPitch, T.vDist, T.vTarget);
        }
        if (T.frames == 35) {
            shot("03_pad.png");
            app.hudHidden = true; // F1: the same view without the HUD
            app.hudToggleTime = -10;
        }
        if (T.frames == 36) {
            shot("03j_pad_hud_hidden.png");
            app.hudHidden = false;
        }
        if (T.frames == 36) {
            logState(app, "pad");
            W.stage(*v);
            v->throttle = 0.4; // TWR < 1: the vehicle stays on the pad with the engines at low thrust
            v->sas = SAS::Off;
        }
        if (T.frames == 56) {
            float lv = 0;
            for (float f : v->engLevel) lv = std::max(lv, f);
            std::fprintf(T.log, "low thrust on the pad: level %.2f landed %d\n", lv, (int)v->landed);
            shot("03b_pad_low_thrust.png");
            v->throttle = 1;
        }
        if (T.frames == 58) {
            float lv = 0;
            for (float f : v->engLevel) lv = std::max(lv, f);
            std::fprintf(T.log, "full thrust: level %.2f\n", lv);
            shot("03c_full_thrust.png");
            T.phase = 3;
            T.frames = 0;
        }
        break;
    case 3: { // gravity-turn ascent (attitude set directly by the test pilot)
        if (!v) return false;
        if (!T.rewound && W.t - v->launchTime > 60) {
            // rewind 30 s during the ascent, then fly on from there
            double t0 = W.t, alt0 = len(v->r) - W.U.bodies[v->ref].radius, m0 = W.vesselMass(*v);
            bool ok = W.rewind(30);
            v = W.activeVessel();
            std::fprintf(T.log, "rewind 30 s: %d, t %.1f -> %.1f (back %.1f s), alt %.0f -> %.0f m, mass %.0f -> %.0f kg, reach now %.1f s\n", ok,
                         t0 - v->launchTime, W.t - v->launchTime, t0 - W.t, alt0, len(v->r) - W.U.bodies[v->ref].radius, m0, W.vesselMass(*v),
                         W.rewindReach());
            T.rewound = true;
        }

        for (int k = 0; k < 6 && !v->destroyed; k++) {
            const Body& b = W.U.bodies[v->ref];
            double alt = len(v->r) - b.radius;
            Vec3d up = norm(v->r);
            Vec3d pole = b.poleFrame.col(2);
            Vec3d east = norm(cross(pole, up));
            double th = alt < 800 ? 0 : std::min(80.0, 90.0 * std::sqrt((alt - 800) / 48000.0)) * DEG;
            Vec3d vs = W.surfaceVel(*v);
            Vec3d dir = up * std::cos(th) + east * std::sin(th);
            if (alt > 25000 && len(vs) > 300) dir = norm(vs); // gravity turn: follow surface prograde
            pointAlong(*v, dir);
            W.update(0.05);
            logEvents(W);
            if (v->tFlameout && vesselStages(*v).size() > 1) {
                W.stage(*v); // separate: the next stage must be off
                std::fprintf(T.log, "after separation: ignited %d, thrust %.0f N\n", (int)v->ignited, v->tThrust);
                W.stage(*v); // ignite it
                logState(app, "staged");
            }
            Conic c = stateToConic(v->r, v->v, b.GM);
            if (c.ap - b.radius > 210000) {
                v->throttle = 0;
                logState(app, "meco");
                T.phase = 4;
                break;
            }
        }
        if (W.t - T.lastLog > 10) { T.lastLog = W.t; logState(app, "ascent"); }
        double alt = len(v->r) - W.U.bodies[v->ref].radius;
        if (!T.shotAscent && alt > 25000) { shot("04_ascent.png"); T.shotAscent = true; }
        if (v->destroyed || T.frames > 20000) { logState(app, "FAIL"); T.phase = 9; }
        break;
    }
    case 4: { // coast to apoapsis
        const Body& b = W.U.bodies[v->ref];
        Conic c = stateToConic(v->r, v->v, b.GM);
        double tAp = timeToTrueAnomaly(c, PI);
        for (int k = 0; k < 20 && tAp > 45; k++) {
            pointAlong(*v, norm(v->v));
            W.update(0.05);
            logEvents(W);
            c = stateToConic(v->r, v->v, b.GM);
            tAp = timeToTrueAnomaly(c, PI);
        }
        if (tAp <= 45 || tAp > c.period * 0.9) {
            logState(app, "at-apoapsis");
            v->throttle = 1;
            T.phase = 5;
        }
        if (v->destroyed) T.phase = 9;
        break;
    }
    case 5: { // circularise
        const Body& b = W.U.bodies[v->ref];
        for (int k = 0; k < 6; k++) {
            // closed-loop circularisation: horizontal prograde, correcting vertical speed
            Vec3d up = norm(v->r);
            Vec3d hor = norm(v->v - up * dot(v->v, up));
            double vv = dot(v->v, up);
            pointAlong(*v, norm(hor + up * clampd(-vv / 150.0, -0.6, 0.6)));
            W.update(0.05);
            logEvents(W);
            if (v->tFlameout && vesselStages(*v).size() > 1) {
                W.stage(*v); // separate (next stage off) ...
                std::fprintf(T.log, "after separation: ignited %d\n", (int)v->ignited);
                W.stage(*v); // ... and ignite
                logState(app, "staged");
            }
            Conic c = stateToConic(v->r, v->v, b.GM);
            if (W.t - T.lastLog > 5) {
                T.lastLog = W.t;
                Vec3d f = v->q.rotate({0, 0, 1});
                std::fprintf(T.log, "  circ: fwd.v=%.3f fwd.up=%.3f thrust=%.0f accel=%.2f\n", dot(f, norm(v->v)), dot(f, norm(v->r)), v->tThrust, v->tAccel);
                logState(app, "circ");
            }
            if (c.pe - b.radius > 150000 || (c.e < 0.003)) {
                v->throttle = 0;
                logState(app, "orbit");
                T.e0 = len2(v->v) / 2 - b.GM / len(v->r);
                T.phase = 64;
                T.frames = 0;
                break;
            }
        }
        if (v->destroyed) T.phase = 9;
        break;
    }
    case 64: { // apparent size of the Sun from Earth orbit
        if (!v) return false;
        int sun = W.U.find("Sun");
        if (T.frames == 2) {
            {
                // behind the Earth: in shadow, and the light direction stays on the (hidden) Sun
                Vec3d sunD = norm(W.U.relPos(W.states, sun, v->ref));
                Vec3d rs = -sunD * (W.U.bodies[v->ref].radius + 200e3);
                bool sh2 = false;
                Vec3d ld2;
                double f2 = W.solarFluxAt(v->ref, rs, &sh2, &ld2);
                std::fprintf(T.log, "earth shadow: shadow %d, flux %.3g W/m2, light dir . sun %.3f -> %s\n", (int)sh2, f2, dot(ld2, sunD),
                             sh2 && dot(ld2, sunD) > 0.99 ? "ok" : "FAIL");
            }
            T.tSave = 0;
            bool sh = false;
            Vec3d ld;
            W.solarFluxAt(v->ref, v->r, &sh, &ld);
            if (sh) { // night side: half a year later the Sun is on the other side
                T.tSave = W.t;
                W.t += 0.5 * 365.25 * DAY;
                W.refreshStates();
            }
            flightDebugGetView(T.vYaw, T.vPitch, T.vDist, T.vTarget);
        }
        if (T.frames == 3) {
            Vec3d up = norm(v->r), east = norm(cross(Vec3d{0, 0, 1}, up)), north = cross(up, east);
            // horizon frame used by the flight camera
            Vec3d s = norm(W.U.relPos(W.states, sun, v->ref) - v->r);
            Vec3d l{dot(s, east), dot(s, north), dot(s, up)};
            flightDebugView(std::atan2(-l.y, -l.x), std::asin(std::clamp(-l.z, -1.0, 1.0)), 10, l * 2000.0);
        }
        if (T.frames == 6) {
            double d = len(W.U.relPos(W.states, sun, v->ref) - v->r);
            double Rs = W.U.bodies[sun].radius;
            double ang = 2 * std::asin(Rs / d) / DEG;
            double H = GetRenderHeight();
            double f = H * 0.5 / std::tan(flightDebugFovy() * 0.5 * DEG);
            double px = 2 * std::tan(std::asin(Rs / d)) * f;
            std::fprintf(T.log, "sun from orbit: distance %.4f AU, angular diameter %.4f deg, expected disc %.1f px of %d (fov %.0f deg)\n",
                         d / AU, ang, px, (int)H, flightDebugFovy());
            shot("05s_sun_from_orbit.png");
        }
        if (T.frames == 7) {
            // the upper-stage engine from behind and a little to the side: open bell, sooty inside
            Vec3d up = norm(v->r), east = norm(cross(Vec3d{0, 0, 1}, up)), north = cross(up, east);
            Vec3d a = v->q.rotate({0, 0, -1});
            Vec3d l{dot(a, east), dot(a, north), dot(a, up)};
            double Lv = std::max(4.0, v->length);
            flightDebugView(std::atan2(l.y, l.x) + 0.18, std::asin(std::clamp(l.z, -1.0, 1.0)) + 0.12, Lv * 0.6, l * (Lv * 0.4));
            int debris = 0;
            for (auto& o : W.vessels) debris += o.debris && !o.destroyed;
            std::fprintf(T.log, "fairing: still attached %d, fairing halves flying %d\n", (int)v->hasFairing, debris);
        }
        if (T.frames == 10) {
            shot("05e_engine_closeup.png");
            app.debugOpen = true; // F3 debug screen over the same view
        }
        if (T.frames == 13) {
            shot("05f_debug_screen.png");
            app.debugOpen = false;
        }
        if (T.frames == 14) {
            if (T.tSave != 0) {
                W.t = T.tSave;
                W.refreshStates();
            }
            flightDebugView(T.vYaw, T.vPitch, T.vDist, T.vTarget);
            T.phase = 6;
            T.frames = 0;
        }
        break;
    }
    case 6:
        if (T.frames == 2) {
            v->sas = SAS::Prograde;
            // interstage: the lower stage carried a skirt over the upper engine
            int skirts = 0;
            for (auto& c : app.lastLaunched.comps) skirts += c.name == "Interstage";
            std::fprintf(T.log, "interstage components in launched craft: %d\n", skirts);
            // solar panels: stowed for launch, now deployed
            double before = v->solarArea;
            W.setSolarDeployed(*v, true);
            std::fprintf(T.log, "solar: stowed area %.2f -> deployed %.2f (max %.2f), folded geometry %d\n", before, v->solarArea,
                         v->solarAreaMax, (int)!v->shown.empty());
        }
        if (T.frames == 3) {
            // fit four RCS quads and a hydrazine tank to the orbiter for the RCS checks
            double zc = v->mp.com.z + 1.0, R = std::max(0.5, v->maxR);
            for (int k = 0; k < 4; k++) {
                Component c = makeComponent(Func::RCS, CATALOG.find("rcs_22"));
                c.stage = v->stage; // part of the current stage
                double a = k * PI / 2;
                c.pos = {std::cos(a) * (R + 0.1), std::sin(a) * (R + 0.1), zc};
                c.rotDeg = {0, 0, a / DEG};
                v->comps.push_back(c);
                v->prop.push_back(0);
                v->heat.push_back(0);
            }
            Component tk = makeComponent(Func::Tank);
            tk.stage = v->stage;
            tk.prop = Prop::Monoprop;
            tk.dims = {0.4, 0.4, 0.6};
            tk.pos = {0, 0, v->mp.com.z};
            v->comps.push_back(tk);
            v->prop.push_back(0);
            v->heat.push_back(0);
            setupVessel(*v, true);
            v->prop.back() = v->phys.back().propCap;
            updateMassProps(*v);
            v->rcs = true;
            v->sas = SAS::Off;
            v->w = {};
            T.rcsV0 = v->v;
            T.rcsR0 = v->r;
            T.rcsM0 = W.vesselMass(*v);
            T.rcsT0 = W.t;
            T.rcsMono0 = v->prop.back();
            std::fprintf(T.log, "rcs: nozzles %zu, authority %.1f N m, monoprop %.1f kg, mass %.1f kg\n", v->rcsNoz.size(), v->rcsAuth,
                         T.rcsMono0, T.rcsM0);
            ControlInput in;
            in.tz = 1; // translate forward
            flightDebugInput(&in);
        }
        if (T.frames == 20) shot("05b_rcs.png");
        if (T.frames == 123) {
            flightDebugInput(nullptr);
            Vec3d gr = T.rcsR0, gv = T.rcsV0; // ghost coasting on the same orbit
            keplerPropagate(gr, gv, W.U.bodies[v->ref].GM, W.t - T.rcsT0);
            Vec3d dv = v->v - gv;
            Vec3d fwd = v->q.rotate({0, 0, 1});
            double used = T.rcsMono0 - v->prop.back();
            double m1 = W.vesselMass(*v);
            // forward nozzles: 4 x 22 N; impulse check against the rocket equation
            double expect = 229 * G0 * std::log(T.rcsM0 / m1);
            std::fprintf(T.log, "rcs translate: %.1f s, dv fwd %.4f m/s (rocket eq %.4f), side %.4f m/s, used %.3f kg, w=%.5f rad/s\n",
                         W.t - T.rcsT0, dot(dv, fwd), expect, len(dv - fwd * dot(dv, fwd)), used, len(v->w));
            ControlInput in;
            in.pitch = 1;
            flightDebugInput(&in);
            T.rcsT0 = W.t;
        }
        if (T.frames == 160) {
            flightDebugInput(nullptr);
            std::fprintf(T.log, "rcs + wheels pitch for %.1f s: w=(%.4f, %.4f, %.4f) rad/s\n", W.t - T.rcsT0, v->w.x, v->w.y, v->w.z);
            v->sas = SAS::Stability; // wheels first, RCS for the remainder
        }
        if (T.frames == 230) {
            std::fprintf(T.log, "rcs + SAS hold: |w|=%.5f rad/s, monoprop left %.2f kg\n", len(v->w), v->prop.back());
            v->rcs = false;
            v->sas = SAS::Prograde;
            // memory pressure: release everything regenerable and keep flying
            W.trimMemory(true);
            trimAeroCache(0);
            gfx::trimCaches(true);
            cadTrimMemory(true);
            mem::compactHeap();
            v = W.activeVessel(); // trimMemory rebuilds the vessel list
            if (!v) return false;
            std::fprintf(T.log, "memory trim: vessels %zu, active aero %s, cache %zu MB, process %zu MB\n", W.vessels.size(),
                         v->aero ? "kept" : "released", aeroCacheBytes() / mem::MB, mem::status().process / mem::MB);
            T.phase = 60;
            T.frames = 0;
        }
        break;
    case 60: { // map "warp here": a quarter orbit ahead, then back to x1
        v = W.activeVessel();
        if (!v) return false;
        const Body& b = W.U.bodies[v->ref];
        static double nuT = 0, tStart = 0, tTarget = 0;
        static double fuel0 = 0, wheelT = 0, tAcs0 = 0, e0 = 0, mono0 = 0;
        static Vec3d gr0, gv0, vSave;
        auto stageFuel = [&]() {
            double f = 0;
            for (size_t i = 0; i < v->comps.size(); i++)
                if (v->comps[i].stage == v->stage && v->comps[i].func == Func::Tank) f += v->prop[i];
            return f;
        };
        if (T.frames == 2) {
            // attitude thrusters with the engines off: refill a little stage propellant, pitch by hand
            for (size_t i = 0; i < v->comps.size(); i++)
                if (v->comps[i].stage == v->stage && v->comps[i].func == Func::Tank && v->comps[i].prop == Prop::Kerolox) {
                    v->prop[i] = 500;
                    break;
                }
            updateMassProps(*v);
            v->rcs = false;
            v->sas = SAS::Off;
            v->w = {};
            v->throttle = 0;
            fuel0 = stageFuel();
            wheelT = v->torque;
            tAcs0 = W.t;
            ControlInput in;
            in.pitch = 1;
            flightDebugInput(&in);
            std::fprintf(T.log, "attitude thrusters: %.0f N per pair, Isp %.0f s, propellant %s, engines ignited %d, wheels %.0f N m\n", v->acsF,
                         v->acsIsp, propName(v->acsProp), (int)v->ignited, wheelT);
        }
        if (T.frames == 62) {
            flightDebugInput(nullptr);
            double used = fuel0 - stageFuel();
            double lever = std::max(0.5, v->length * 0.45), dt = W.t - tAcs0;
            double expectW = (v->acsF * lever + wheelT) / v->mp.inertia.m[0][0] * dt;
            std::fprintf(T.log, "attitude thrusters, engines off, pitch input %.2f s: w = %.4f rad/s (theory %.4f, Ixx %.0f kg m2), stage propellant used %.3f kg (theory %.3f)\n",
                         dt, len(v->w), expectW, v->mp.inertia.m[0][0], used, v->acsF / (v->acsIsp * G0) * dt);
            v->sas = SAS::Stability; // the thrusters now brake the rotation (and keep using propellant)
            fuel0 = stageFuel();
        }
        if (T.frames == 140) {
            std::fprintf(T.log, "attitude thrusters, SAS hold: w = %.5f rad/s, propellant used %.3f kg\n", len(v->w), fuel0 - stageFuel());
            v->sas = SAS::Prograde;
            for (size_t i = 0; i < v->comps.size(); i++) // empty the stage again (the flame-out check below needs it dry)
                if (v->comps[i].stage == v->stage && v->comps[i].func == Func::Tank && v->comps[i].prop == Prop::Kerolox) v->prop[i] = 0;
            updateMassProps(*v);
        }
        if (T.frames == 150) {
            // electric RCS: battery powered translation (10 % thrust, Isp 800 s)
            v->rcs = true;
            v->rcsSource = RcsSource::Battery;
            v->sas = SAS::Off;
            v->w = {};
            e0 = v->charge;
            mono0 = monoAmount(*v);
            gr0 = v->r;
            gv0 = v->v;
            tAcs0 = W.t;
            ControlInput in;
            in.tz = 1;
            flightDebugInput(&in);
        }
        if (T.frames == 210) {
            flightDebugInput(nullptr);
            double dt = W.t - tAcs0;
            Vec3d gr = gr0, gv = gv0;
            keplerPropagate(gr, gv, b.GM, dt);
            double dv = dot(v->v - gv, v->q.rotate({0, 0, 1}));
            double F = 0;
            for (auto& z : v->rcsNoz)
                if (dot(z.dir, {0, 0, 1}) > 0.9) F += z.F * RCS_ELEC_THRUST;
            double expectE = F * RCS_ELEC_ISP * G0 / (2 * RCS_ELEC_EFF) * dt;
            std::fprintf(T.log, "electric RCS %.2f s: dv %.5f m/s (theory %.5f), battery used %.0f J (theory %.0f, power %.1f kW), monoprop used %.3f kg\n", dt, dv,
                         F * dt / W.vesselMass(*v), e0 - v->charge, expectE, v->tRcsPowerW / 1000, mono0 - monoAmount(*v));
            v->rcs = false;
            v->rcsSource = RcsSource::Auto;
        }
        if (T.frames == 214) {
            vSave = v->v; // escape trajectory, then back into orbit: gravity notifications
            v->v = v->v * 1.45;
        }
        if (T.frames == 222) v->v = vSave;
        if (T.frames == 262) {
            Conic c = stateToConic(v->r, v->v, b.GM);
            nuT = c.nu + PI / 2;
            double dt = timeToTrueAnomaly(c, nuT);
            tStart = W.t;
            tTarget = W.t + dt;
            bool ok = W.startWarpTo(tTarget);
            std::fprintf(T.log, "warp to point: start=%d dt=%.1f s\n", ok, dt);
        }
        if (T.frames > 262 && W.warpTo < 0) {
            Conic c0 = stateToConic(v->r, v->v, b.GM);
            Vec3d expect = conicPoint(c0, nuT);
            double ang = std::acos(std::clamp(dot(norm(v->r), norm(expect)), -1.0, 1.0)) / DEG;
            std::fprintf(T.log, "warp to point: arrived after %d frames, time error %.3f s, position error %.4f deg, warp x%g\n", T.frames,
                         W.t - tTarget, ang, W.warpFactor());
            (void)tStart;
            T.phase = 61;
            T.frames = 0;
        }
        if (T.frames > 3400) {
            std::fprintf(T.log, "warp to point: FAIL (did not arrive)\n");
            W.cancelWarpTo(nullptr);
            T.phase = 61;
            T.frames = 0;
        }
        break;
    }
    case 61:
        v = W.activeVessel();
        if (T.frames == 5) {
            // save / load round trip (RCS parts, deployed panels, expanded components)
            std::string path = T.dir + "\\roundtrip.sfs";
            size_t nc = v->comps.size();
            int id = v->id;
            bool solar = v->solarDeployed;
            bool ok = saveWorld(W, path);
            bool ld = ok && loadWorld(W, path);
            v = W.activeVessel();
            std::fprintf(T.log, "save/load: saved=%d loaded=%d comps %zu -> %zu, active id %d -> %d, solar %d -> %d, nozzles %zu\n", ok, ld, nc,
                         v ? v->comps.size() : 0, id, v ? v->id : -1, solar, v ? v->solarDeployed : -1, v ? v->rcsNoz.size() : 0);
            if (!v) { T.phase = 9; break; }
            // a truncated file must be rejected without touching the current game
            {
                std::ifstream in(path, std::ios::binary);
                std::string all((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
                std::ofstream out(path + ".cut", std::ios::binary);
                out << all.substr(0, all.size() / 2);
            }
            std::fprintf(T.log, "truncated save rejected: %d\n", !loadWorld(W, path + ".cut"));
        }
        if (T.frames == 12) {
            v->ignited = true;
            v->throttle = 1; // the upper stage is empty: the engine must not light (no thrust, no flame)
        }
        if (T.frames == 20) {
            double amt[PROP_N], cap[PROP_N], srbA, srbC, fv, md;
            stageResources(*v, amt, cap, srbA, srbC, fv, md);
            float lv = 0;
            for (float f : v->engLevel) lv += f;
            std::fprintf(T.log, "flameout: stage kerolox %.1f kg, thrust %.1f N, engine level sum %.3f, flameout flag %d\n",
                         amt[(int)Prop::Kerolox], v->tThrust, lv, (int)v->tFlameout);
            v->throttle = 0;
        }
        if (T.frames == 30) shot("05_orbit.png");
        if (T.frames == 31) W.setWarp(9); // x100000 on rails
        if (T.frames == 100 || T.frames == 199 || T.frames == 230) {
            const Body& bb = W.U.bodies[v->ref];
            std::fprintf(T.log, "sas diag f%d: warp x%g eff x%g, target . nose %.3f, landed %d, thrusting %d, alt %.0f, atmTop %.0f, reason '%s'\n", T.frames,
                         W.warpFactor(), W.effectiveWarp, dot(W.targetDirection(*v, v->sas), v->q.rotate({0, 0, 1})), (int)v->landed,
                         (int)(v->ignited && v->throttle > 0), len(v->r) - bb.radius, bb.atmTop, W.warpLimitReason.c_str());
        }
        if (T.frames == 200) {
            W.setWarp(0);
            const Body& b = W.U.bodies[v->ref];
            double e1 = len2(v->v) / 2 - b.GM / len(v->r);
            std::fprintf(T.log, "energy drift after warp: %.3e (relative)\n", (e1 - T.e0) / std::fabs(T.e0));
            logState(app, "after-warp");
            flightDebugSetMap(true, 4e7);
        }
        if (T.frames == 240) {
            shot("06_map.png");
            std::fprintf(T.log, "map icon: nose . prograde = %.3f (sas %d, control %d, power out %d, charge %.0f J, target . nose %.3f)\n",
                         dot(v->q.rotate({0, 0, 1}), norm(v->v)), (int)v->sas, (int)v->hasControl, (int)v->tPowerOut, v->charge,
                         dot(W.targetDirection(*v, SAS::Prograde), v->q.rotate({0, 0, 1})));
            flightDebugSetMap(true, 2e12);
        }
        if (T.frames == 270) {
            shot("07_map_solar.png");
            flightDebugSetMap(false, 0);
            // orbital re-entry: put the orbiter at 70 km, 7.6 km/s, nose first (entry plasma check)
            const Body& b = W.U.bodies[v->ref];
            Vec3d up = norm(v->r);
            Vec3d hz = norm(v->v - up * dot(v->v, up));
            v->r = up * (b.radius + 70000.0);
            v->v = hz * 7600.0 - up * 120.0 + cross(W.U.angVel(v->ref), v->r) * 0.0;
            v->q = Quatd::lookRot(norm(v->v), up);
            v->w = {};
            v->sas = SAS::Off;
            v->landed = false;
            W.setWarp(0);
            T.phase = 62;
            T.frames = 0;
        }
        break;
    case 62:
        v = W.activeVessel();
        if (!v) return false;
        if (T.frames == 20) flightDebugLog(true);
        if (T.frames == 23) {
            shot("07c_mission_log.png");
            flightDebugLog(false);
        }
        if (T.frames == 25) {
            std::fprintf(T.log, "entry: heat flux %.0f kW/m2, heat max %.3f, destroyed %d\n", v->tHeatFlux / 1000, v->tHeatMax, (int)v->destroyed);
            shot("07b_entry_plasma.png");
        }
        if (T.frames == 26) {
            // hyperspace test with the HX template
            W.settings.hyperspace = true;
            auto tpl = builtinTemplates();
            app.lastLaunched = craftFromDesign(tpl.back());
            W.launch(app.lastLaunched);
            flightOnEnter(app);
            std::string err;
            int prox = W.U.find("Proxima Centauri b");
            // refused on the pad and inside the atmosphere, accepted in orbit
            bool okPad = W.startHyperspace(prox, err);
            std::fprintf(T.log, "hyperspace from the pad: %s (%s)\n", okPad ? "STARTED (FAIL)" : "refused (ok)", err.c_str());
            bool okAtm = true;
            if (Vessel* a = W.activeVessel()) {
                const Body& hb = W.U.bodies[a->ref];
                Vec3d rh = norm(a->r);
                Vec3d east = norm(cross(Vec3d{0, 0, 1}, rh));
                auto place = [&](double alt) {
                    double rr = hb.radius + alt;
                    a->landed = false;
                    a->r = rh * rr;
                    a->v = east * std::sqrt(hb.GM / rr);
                    a->w = {};
                    a->q = Quatd::lookRot(east, rh);
                };
                place(hb.atmTop * 0.5);
                okAtm = W.startHyperspace(prox, err);
                std::fprintf(T.log, "hyperspace in the atmosphere: %s (%s)\n", okAtm ? "STARTED (FAIL)" : "refused (ok)", err.c_str());
                if (okAtm) W.hyper = {};
                place(hb.atmTop + 200e3);
                a->q = Quatd::lookRot(rh, east); // nose up: the drive has to turn it onto the destination
            }
            bool ok = W.startHyperspace(prox, err);
            std::fprintf(T.log, "hyperspace start in orbit: %d %s\n", ok, err.c_str());
            T.phase = 7;
            T.frames = 0;
        }
        break;
    case 7:
        if (T.frames == 40) shot("08a_hyperspace_turning.png");
        if (T.frames == 120) shot("08_hyperspace.png");
        if (!W.hyper.active && T.frames > 60) {
            logState(app, "arrived");
            T.phase = 8;
            T.frames = 0;
        }
        break;
    case 8:
        if (T.frames == 40) {
            shot("09_proxima_b.png");
            // Saturn rings view
            int sat = W.U.find("Saturn");
            std::string err;
            W.startHyperspace(sat, err);
            T.phase = 10;
            T.frames = 0;
        }
        break;
    case 10:
        if (!W.hyper.active && T.frames > 60) {
            if (Vessel* a = W.activeVessel()) {
                // look back at Saturn from a bit farther away
                a->q = Quatd::lookRot(-norm(a->r), {0, 0, 1});
            }
            T.frames = 0;
            T.phase = 11;
        }
        break;
    case 11:
        if (T.frames == 30) {
            shot("10_saturn.png");
            flightDebugSetMap(true, 2e9);
        }
        if (T.frames == 60) {
            shot("11_saturn_map.png");
            flightDebugSetMap(false, 0);
            T.phase = 70;
            T.frames = 0;
        }
        break;
    case 70: { // tutorials: each one starts in its situation; the first is played through its early steps
        auto logTut = [&](const char* what) {
            Vessel* a = W.activeVessel();
            std::fprintf(T.log, "tutorial %s: active %d step %d finished %d screen %d", what, (int)tut::active(), tut::stepIndex(),
                         (int)tut::finished(), (int)app.screen);
            if (a) {
                const Body& b = W.U.bodies[a->ref];
                Conic c = stateToConic(a->r, a->v, b.GM);
                std::fprintf(T.log, "  vessel %s ref %s alt %.0f km pe %.0f km ap %.0f km landed %d rcs %zu chute %d hyper %d where '%s'",
                             a->name.c_str(), b.name.c_str(), (len(a->r) - b.radius) / 1000, (c.pe - b.radius) / 1000,
                             (c.ap - b.radius) / 1000, (int)a->landed, a->rcsNoz.size(), (int)(a->chuteCdA > 0), (int)a->hasHyperdrive,
                             W.hyperspaceLocationBlock(*a).c_str());
            }
            std::fprintf(T.log, "\n");
        };
        if (T.frames == 2) {
            tut::start(app, 0);
            logTut("1 start");
        }
        if (T.frames == 20) {
            shot("17a_tutorial_factory.png");
            // the player clicks LAUNCH
            app.lastLaunched = craftFromDesign(app.design);
            W.launch(app.lastLaunched);
            app.screen = Screen::Flight;
            flightOnEnter(app);
        }
        if (T.frames == 120) {
            logTut("1 after launch");
            shot("17b_tutorial_pad.png");
            if (Vessel* a = W.activeVessel()) a->throttle = 0; // waits on Next (explanation step)
        }
        // later steps: throttle (step 2) after the explanation is skipped by the test
        if (T.frames == 122) tut::start(app, 1), logTut("2 start");
        if (T.frames == 180) shot("17c_tutorial_orbit_intro.png");
        if (T.frames == 182) tut::start(app, 3), logTut("4 start (space)");
        if (T.frames == 240) shot("17d_tutorial_space.png");
        if (T.frames == 242) tut::start(app, 4), logTut("5 start (return)");
        if (T.frames == 300) shot("17e_tutorial_return.png");
        if (T.frames == 302) tut::start(app, 7), logTut("8 start (hyperspace)");
        if (T.frames == 360) {
            shot("17f_tutorial_hyper.png");
            tut::stop();
            app.screen = Screen::Menu; // the tutorial list in its order
            tut::menuOpen = true;
        }
        if (T.frames == 366) {
            shot("17g_tutorial_menu.png");
            tut::menuOpen = false;
            app.screen = Screen::Flight;
            T.phase = 73;
            T.frames = 0;
        }
        break;
    }
    case 73: { // payloads on the pad: with the fairing, and without it (folded panels and legs)
        auto launchTpl = [&](const char* name) {
            for (const Design& t : builtinTemplates())
                if (t.name == name) {
                    app.design = t;
                    app.lastLaunched = craftFromDesign(t);
                    W.launch(app.lastLaunched);
                    flightOnEnter(app);
                }
        };
        auto topView = [&](double yaw) {
            Vessel* a = W.activeVessel();
            if (!a) return;
            double ztop = -1e30;
            for (auto& c : a->comps) ztop = std::max(ztop, c.pos.z + c.dims.z / 2);
            double zf = 1e30;
            for (auto& c : a->comps)
                if (c.func == Func::Fairing) zf = std::min(zf, c.pos.z - c.dims.z / 2);
            if (zf > 1e29) zf = ztop - 6;
            flightDebugView(yaw, 0.12, (ztop - zf) * 1.6 + 6, Vec3d{0, 0, (ztop + zf) / 2 - a->mp.com.z});
        };
        if (T.frames == 2) launchTpl("Aurora Crew Mission");
        if (T.frames == 4) topView(-PI / 2 + 0.5);
        if (T.frames == 8) shot("20_aurora_fairing.png");
        if (T.frames == 10) {
            launchTpl("Hyperspace Explorer HX");
            if (Vessel* a = W.activeVessel()) W.jettisonFairing(*a, false);
        }
        if (T.frames == 12) topView(-PI / 2 + 0.9);
        if (T.frames == 16) {
            shot("20b_hx_payload_stowed.png");
            if (Vessel* a = W.activeVessel()) {
                W.setSolarDeployed(*a, true);
                W.toggleLegs(*a);
                std::fprintf(T.log, "deploy animation: solar %d legs %d (anim %.2f %.2f)\n", (int)a->solarDeployed, (int)a->legsDeployed, a->solarT,
                             a->legsT);
            }
        }
        if (T.frames > 16 && T.frames < 100) {
            // deploy animation: a few poses on the way out
            static int shotN = 0;
            if (T.frames == 17) shotN = 0;
            Vessel* a = W.activeVessel();
            const float marks[] = {0.18f, 0.5f, 0.8f, 1.0f};
            const char* names[] = {"20d_deploy_swing.png", "20e_deploy_unfold.png", "20f_deploy_late.png", "20g_deployed.png"};
            if (a && shotN < 4 && a->solarT >= marks[shotN]) {
                std::fprintf(T.log, "deploy animation: solar %.2f legs %.2f -> %s\n", a->solarT, a->legsT, names[shotN]);
                shot(names[shotN]);
                shotN++;
            }
            static int waitN = 0;
            if (T.frames == 17) waitN = 0;
            if (shotN >= 4 || ++waitN > 3000) T.frames = 100;
            else T.frames = 50; // hold here (real time passes) until the animation is through
        }
        if (T.frames == 118) {
            launchTpl("Aurora Crew Mission");
            if (Vessel* a = W.activeVessel()) W.jettisonFairing(*a, false);
        }
        if (T.frames == 120) topView(-PI / 2 + 0.2);
        if (T.frames == 124) {
            shot("20c_aurora_payload_stowed.png");
            T.phase = 74;
            T.frames = 0;
        }
        break;
    }
    case 74: { // side boosters: lit together with the core stage, separated first, core keeps burning
        static double t0 = 0;
        auto levels = [&](const Vessel& a, float& boost, float& core, int& nb) {
            boost = core = 0;
            nb = 0;
            for (size_t i = 0; i < a.comps.size(); i++) {
                const Component& c = a.comps[i];
                if (c.func != Func::Engine || c.def < 0) continue;
                float lv = i < a.engLevel.size() ? a.engLevel[i] : 0.f;
                bool srb = CATALOG.defs[c.def].kind == PK::SRB;
                if (srb) { boost = std::max(boost, lv); nb++; }
                else if (c.stage == a.stage || c.fireFrom >= 0) core = std::max(core, lv);
            }
        };
        if (T.frames == 2) {
            for (const Design& t : builtinTemplates())
                if (t.name == "Aurora Crew Mission") {
                    DesignStats st = analyzeDesign(t);
                    std::fprintf(T.log, "boosters: Aurora liftoff TWR %.2f, stage 1 dv %.0f m/s\n", st.stages.empty() ? 0 : st.stages[0].twrSL,
                                 st.stages.empty() ? 0 : st.stages[0].dvVac);
                    {
                        // booster nozzle exits level with the core stage's engines
                        Craft cr = craftFromDesign(t);
                        double zb = 1e9, ze = 1e9;
                        for (auto& c : cr.comps) {
                            if (c.func != Func::Engine || c.def < 0) continue;
                            double bot = c.pos.z - c.dims.z / 2;
                            if (CATALOG.defs[c.def].kind == PK::SRB) zb = std::min(zb, bot);
                            else if (c.fireFrom >= 0) ze = std::min(ze, bot);
                        }
                        std::fprintf(T.log, "boosters: nozzle exits booster %.2f m, core %.2f m -> %s\n", zb, ze, std::fabs(zb - ze) < 0.01 ? "ok" : "FAIL");
                    }
                    app.design = t;
                    app.screen = Screen::Factory; // the factory view of this design for one frame
                }
        }
        if (T.frames == 3) {
            shot("21a_factory_side_boosters.png");
            app.lastLaunched = craftFromDesign(app.design);
            W.launch(app.lastLaunched);
            app.screen = Screen::Flight;
            flightOnEnter(app);
        }
        if (T.frames == 4) {
            if (v) {
                std::fprintf(T.log, "boosters: stages %zu, active %d\n", vesselStages(*v).size(), v->stage);
                v->throttle = 1;
                W.stage(*v); // ignite
                t0 = W.t;
            }
        }
        if (T.frames == 6) flightDebugView(-PI / 2 + 0.6, 0.12, 150, Vec3d{0, 0, -12});
        if (T.frames > 4 && v && T.frames < 400) {
            if (T.frames == 40) {
                float b, c;
                int nb;
                levels(*v, b, c, nb);
                std::fprintf(T.log, "boosters: after %.1f s alt %.0f m: booster level %.2f (%d motors), core level %.2f\n", W.t - t0,
                             len(v->r) - W.U.bodies[v->ref].radius, b, nb, c);
                shot("21_side_boosters.png");
                W.stage(*v); // drop the boosters early
            }
            if (T.frames == 46) {
                float b, c;
                int nb;
                levels(*v, b, c, nb);
                std::fprintf(T.log, "boosters: after separation: %d motors left, ignited %d, core level %.2f -> %s\n", nb, (int)v->ignited, c,
                             nb == 0 && v->ignited && c > 0.05f ? "ok" : "FAIL");
                shot("21b_boosters_separated.png");
                T.phase = 72;
                T.frames = 0;
            }
        }
        break;
    }
    case 72: { // crew capsule entering the atmosphere heat shield first: shape of the plasma
        if (T.frames == 2) {
            Design d;
            for (const Design& t : builtinTemplates())
                if (t.name == "Aurora Crew Mission") d = t;
            int dec = CATALOG.find("decoupler");
            for (size_t i = 0; i < d.stack.size(); i++)
                if (d.stack[i].def == dec) { d.stack.resize(i); break; }
            d.name = "Aurora Capsule";
            app.lastLaunched = craftFromDesign(d);
            int idx = W.launch(app.lastLaunched);
            Vessel& a = W.vessels[idx];
            W.jettisonFairing(a, false);
            const Body& eb = W.U.bodies[a.ref];
            Vec3d up = norm(a.r), east = norm(cross(Vec3d{0, 0, 1}, up));
            a.landed = false;
            a.r = up * (eb.radius + 68000);
            a.v = east * 7300.0 - up * 150.0;
            a.q = Quatd::lookRot(-norm(a.v), up); // nose back: the heat shield faces the flow
            a.w = {};
            a.sas = SAS::Retrograde;
            flightOnEnter(app);
        }
        if (T.frames == 6) flightDebugView(-PI / 2 + 1.2, 0.15, 30, Vec3d{});
        if (T.frames == 12) {
            if (v) std::fprintf(T.log, "capsule entry: heat flux %.0f kW/m2, plasma shot\n", v->tHeatFlux / 1000);
            shot("19_capsule_entry.png");
            flightDebugView(-PI / 2 + 2.6, 0.1, 30, Vec3d{});
        }
        if (T.frames == 16) {
            shot("19b_capsule_entry_rear.png");
            T.phase = 71;
            T.frames = 0;
        }
        break;
    }
    case 71: { // crew capsule under its parachute cluster: descent rate and touchdown
        if (T.frames == 2) {
            Design d;
            for (const Design& t : builtinTemplates())
                if (t.name == "Aurora Crew Mission") d = t;
            int dec = CATALOG.find("decoupler");
            for (size_t i = 0; i < d.stack.size(); i++)
                if (d.stack[i].def == dec) { d.stack.resize(i); break; }
            d.name = "Aurora Capsule";
            app.lastLaunched = craftFromDesign(d);
            int idx = W.launch(app.lastLaunched);
            Vessel& a = W.vessels[idx];
            W.jettisonFairing(a, false);
            const Body& eb = W.U.bodies[a.ref];
            Vec3d up = norm(a.r);
            a.landed = false;
            a.r = up * (eb.radius + 2500);
            a.v = W.surfaceVel(a) * 0.0; // set below in the rotating frame
            Vec3d omega = norm(eb.poleFrame.col(2)) * eb.rotRate;
            a.v = cross(omega, a.r) - up * 90.0; // falling at 90 m/s relative to the ground
            a.q = Quatd::lookRot(up, anyPerp(up));
            a.w = {};
            a.chuteDeployed = true;
            a.sas = SAS::Stability;
            flightOnEnter(app);
            T.peakFlux = 0;
            T.shotEntry = false;
            double cda = 0;
            for (auto& c : a.comps)
                if (c.func == Func::Parachute) cda += CATALOG.defs[c.def].chuteCdA;
            double m = W.vesselMass(a);
            std::fprintf(T.log, "parachute: mass %.0f kg, CdA %.0f m2, expected sea-level descent %.1f m/s (PR-25 alone: %.1f m/s)\n", m, cda,
                         std::sqrt(2 * m * 9.81 / (1.225 * cda)), std::sqrt(2 * m * 9.81 / (1.225 * 420)));
        }
        if (T.frames > 2 && v) {
            for (int k = 0; k < 30 && !v->landed && !v->destroyed; k++) {
                W.update(0.05);
                logEvents(W);
                v = W.activeVessel();
                if (!v) break;
                double alt = len(v->r) - W.U.bodies[v->ref].radius;
                if (!T.shotEntry && alt < 300) {
                    std::fprintf(T.log, "parachute: descent rate at %.0f m: %.1f m/s (open %.2f)\n", alt, len(W.surfaceVel(*v)), v->chuteOpen);
                    T.shotEntry = true;
                }
            }
            if (T.frames == 12) {
                Vessel* a = W.activeVessel();
                if (a) flightDebugView(-PI / 2 + 0.6, 0.25, 95, Vec3d{0, 0, 28});
            }
            if (T.frames == 16) shot("18_parachute.png");
            if (!v || v->landed || v->destroyed || T.frames > 900) {
                if (v) std::fprintf(T.log, "parachute: landed %d destroyed %d (%s)\n", (int)v->landed, (int)v->destroyed, v->fate.c_str());
                app.craft = builtinCrafts()[0];
                app.screen = Screen::CAD;
                T.phase = 12;
                T.frames = 0;
            }
        }
        break;
    }
    case 100: // CAD-only run
        if (T.frames == 2) {
            physicsChecks();
            W.newGame(Settings{});
            app.gameStarted = true;
            app.craft = builtinCrafts()[0];
            app.screen = Screen::CAD;
            T.phase = 12;
            T.frames = 0;
        }
        break;
    case 12: // CAD designer
        if (T.frames == 20) shot("12_cad.png");
        if (T.frames == 22) cadDebugSet(1, 0);
        if (T.frames == 40) shot("13_cad_selected.png");
        if (T.frames == 42) cadDebugSet(-1, 1);
        if (T.frames == 60) {
            shot("14_cad_analysis.png");
            app.lastLaunched = app.craft;
            W.launch(app.craft);
            flightOnEnter(app);
            app.screen = Screen::Flight;
            Vessel* r = W.activeVessel();
            r->sas = SAS::Off; // passive aerodynamic stability only
            W.stage(*r);       // light the solid motor
            T.lastLog = -1e9;
            T.shotAscent = false;
            T.phase = 13;
            T.frames = 0;
        }
        break;
    case 13: { // finned sounding rocket: passive stability, separation, parachute landing
        if (!v) { T.phase = 9; break; }
        for (int k = 0; k < 10 && !v->destroyed && !v->landed; k++) {
            W.update(0.05);
            logEvents(W);
            const Body& b = W.U.bodies[v->ref];
            double alt = len(v->r) - b.radius;
            Vec3d vs = W.surfaceVel(*v);
            double vz = dot(vs, norm(v->r));
            if (v->stage == 0 && v->ignited && v->prop[0] <= 0 && vz < 0) {
                logState(app, "separate");
                W.stage(*v);
            }
            if (v->stage == 1 && vz < 0 && alt < 3000 && len(vs) < 160 && !v->chuteDeployed) {
                v->chuteDeployed = true;
                logState(app, "chute");
            }
        }
        if (!v->destroyed && W.t - T.lastLog > 5) {
            T.lastLog = W.t;
            Vec3d up = norm(v->r), nose = v->q.rotate({0, 0, 1});
            std::fprintf(T.log, "  rocket: tilt=%.1fdeg aoa=%.1fdeg mach=%.2f |w|=%.3f stage=%d ", std::acos(clampd(dot(up, nose), -1, 1)) / DEG,
                         v->tAoA / DEG, v->tMach, len(v->w), v->stage);
            if (v->tDynP > 50 && v->aero) {
                Vec3d va = v->v - cross(W.U.angVel(v->ref), v->r);
                Vec3d u = v->q.conj().rotate(-norm(va));
                std::vector<float> ex;
                v->aero->lookup(u, &ex);
                std::fprintf(T.log, "u=(%.2f,%.2f,%.2f) flux=%.0f ", u.x, u.y, u.z, v->tHeatFlux);
                for (size_t i = 0; i < ex.size(); i++) std::fprintf(T.log, "[%s e=%.2f h=%.2f] ", v->comps[i].name.c_str(), ex[i], v->heat[i]);
            }
            logState(app, "fly");
        }
        double alt = len(v->r) - W.U.bodies[v->ref].radius;
        if (!T.shotAscent && alt > 3000) { shot("15_rocket_ascent.png"); T.shotAscent = true; }
        if (v->landed || v->destroyed || T.frames > 6000) {
            logState(app, v->landed ? "rocket-landed" : "rocket-end");
            shot("16_rocket_end.png");
            T.phase = 9;
        }
        break;
    }
    case 200:
        if (!tutFlightStep(app)) T.phase = 9;
        break;
    case 210:
        if (!orbitAllStep(app)) T.phase = 9;
        break;
    case 220:
        if (!rewindTestStep(app)) T.phase = 9;
        break;
    case 230:
        if (!terrainTestStep(app)) T.phase = 9;
        break;
    case 240:
        if (!perfTestStep(app)) T.phase = 9;
        break;
    case 250:
        if (!airTestStep(app)) T.phase = 9;
        break;
    case 260:
        if (!tliExpStep(app)) T.phase = 9;
        break;
    case 270:
        if (!warpTestStep(app)) T.phase = 9;
        break;
    case 280:
        if (!pickTestStep(app)) T.phase = 9;
        break;
    case 290:
        if (!ctrlTestStep(app)) T.phase = 9;
        break;
    case 300:
        if (!designTestStep(app)) T.phase = 9;
        break;
    case 310:
        if (!roverTestStep(app)) T.phase = 9;
        break;
    case 320:
        if (!labelTestStep(app)) T.phase = 9;
        break;
    case 330:
        if (!bgmTestStep(app)) T.phase = 9;
        break;
    case 340:
        if (!spaceplaneTestStep(app)) T.phase = 9;
        break;
    case 350:
        if (!dockTestStep(app)) T.phase = 9;
        break;
    case 360:
        if (!saveTestStep(app)) T.phase = 9;
        break;
    case 370:
        if (!skyTestStep(app)) T.phase = 9;
        break;
    case 380:
        if (!trussTestStep(app)) T.phase = 9;
        break;
    case 390:
        if (!orbitLineTestStep(app)) T.phase = 9;
        break;
    case 400:
        if (!navTestStep(app)) T.phase = 9;
        break;
    case 410:
        if (!achTestStep(app)) T.phase = 9;
        break;
    case 420:
        if (!dupTestStep(app)) T.phase = 9;
        break;
    case 430:
        if (!seaTestStep(app)) T.phase = 9;
        break;
    case 440:
        if (!realTestStep(app)) T.phase = 9;
        break;
    case 450:
        if (!uiTestStep(app)) T.phase = 9;
        break;
    case 460:
        if (!solarTestStep(app)) T.phase = 9;
        break;
    case 470:
        if (!starTestStep(app)) T.phase = 9;
        break;
    case 480:
        if (!bodyTestStep(app)) T.phase = 9;
        break;
    case 490:
        if (!baseTestStep(app)) T.phase = 9;
        break;
    case 500:
        if (!missionTestStep(app)) T.phase = 9;
        break;
    case 510:
        if (!escapeTestStep(app)) T.phase = 9;
        break;
    case 520:
        if (!rdvTestStep(app)) T.phase = 9;
        break;
    case 530:
        if (!rdvSweepStep(app)) T.phase = 9;
        break;
    case 550:
        if (!launchWinTestStep(app)) T.phase = 9;
        break;
    case 560:
        if (!shadowTestStep(app)) T.phase = 9;
        break;
    case 540:
        if (!saveCheckStep(app)) T.phase = 9;
        break;
    case 9:
        logState(app, "end");
        if (T.log) std::fclose(T.log);
        T.log = nullptr;
        return false;
    }
    return true;
}
