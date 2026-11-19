#include "sim.hpp"
#include "brand.hpp"
#include "i18n.hpp"
#include "diag.hpp"
#include "ground.hpp"
#include "navguide.hpp"
#include "base.hpp"
#include <algorithm>
#include <functional>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <initializer_list>
#include <sstream>

const double WARP_LEVELS[] = {1, 2, 3, 4, 10, 50, 100, 1000, 1e4, 1e5, 1e6, 1e7, 1e8};
const int WARP_COUNT = sizeof(WARP_LEVELS) / sizeof(WARP_LEVELS[0]);
static const int PHYSICS_WARP_MAX = 3; // index of 4x
static const int THIN_AIR_WARP = 8;    // index of 10,000x: the highest rate inside an atmosphere (its thin top)
// airless bodies on a course into the ground: the warp drops this far above the highest summit (2 % of
// the radius, 2-20 km)
static double surfaceApproach(const Body& b) { return std::clamp(0.02 * b.radius, 2e3, 20e3); }
// a rover's drive command (-1..1): W / S (the pitch keys) drive and reverse, else the throttle cruises forward;
// only for the active vessel with driven wheels on the ground
double roverDrive(const World& W, const Vessel& v) {
    bool active = W.active >= 0 && W.active < (int)W.vessels.size() && &v == &W.vessels[W.active];
    if (v.drivePower <= 0 || !(v.landed || v.rolling) || !active || v.destroyed) return 0;
    if (std::fabs(W.ctrl.pitch) > 0.01) return std::clamp(W.ctrl.pitch, -1.0, 1.0);
    return v.throttle > 0.01 ? v.throttle : 0.0;
}

// manual attitude control: the pilot's input asks for a turn rate (rate command, as the hand controllers of
// crewed spacecraft); the wheels, gimbals, RCS and fins deliver the torque for it
static const double RATE_PY = 6 * DEG, RATE_ROLL = 10 * DEG; // at full input (less for a vessel with little authority)
static const double INPUT_RAMP_IN = 2.5, INPUT_RAMP_OUT = 6.0;  // keys: full input after 0.4 s, back to 0 in 0.17 s
// control surfaces: effectiveness (deflection -> change of incidence, ~30 % chord flap) and actuator speed
static const double SURF_TAU = 0.45, SURF_RATE = 2.0; // full travel in 0.5 s
// a surface's role follows from what its lift turns the vessel about (g = lever x normal): an aileron
// (roll) is the outer ~35 % of a wing's span with 10 degrees of travel, a rudder (yaw) 20 degrees, a
// stabilator (pitch) 15 degrees of the whole surface. Returned: the incidence change at full deflection [rad]
static double surfGain(const Vec3d& g) {
    double ax = std::fabs(g.x), ay = std::fabs(g.y), az = std::fabs(g.z);
    if (az >= ax && az >= ay) return SURF_TAU * 10 * DEG * 0.35;
    if (ax >= ay) return 15 * DEG; // pitch: an all-moving tailplane (as on jets), the whole surface turns
    return SURF_TAU * 20 * DEG;
}

// where a surface's deflection acts: a wing panel's flap at its trailing edge (local +Y is aft along the chord),
// the centre of other fins (all-moving)
static Vec3d surfPoint(const Component& c, const CompPhys& p) {
    if (c.shape != Shape::Airfoil) return p.centre;
    return p.centre + c.rot().rotate(Vec3d{0, 0.4 * c.dims.y, 0});
}
// incidence change for the commanded surface positions (pitch, yaw, roll) of a surface whose lift turns the
// vessel by g. A tailless craft (no pitch surface: a delta orbiter) flies its wings' flaps as elevons: pitch with
// the flap's full 20 degrees, roll as ailerons
static const double ELEVON_GAIN = SURF_TAU * 20 * DEG;
static double surfDeflect(const Vec3d& g, const Vec3d& cmd, bool elevons) {
    double gl = len(g);
    if (gl < 1e-6) return 0;
    Vec3d u = g / gl;
    bool rollRole = std::fabs(g.z) >= std::fabs(g.x) && std::fabs(g.z) >= std::fabs(g.y);
    if (elevons && rollRole) // (full-span elevons: the whole trailing edge rolls as well as pitches)
        return ELEVON_GAIN * clampd(u.y * cmd.y + u.z * cmd.z, -1, 1) + ELEVON_GAIN * clampd(u.x * cmd.x, -1, 1);
    return surfGain(g) * clampd(dot(u, cmd), -1, 1);
}

const LaunchSite LAUNCH_SITES[] = {
    {"Tanegashima, Japan", 30.4009, 130.9700},
    {"Cape Canaveral, USA", 28.5729, -80.6490},
    {"Kourou, French Guiana", 5.2390, -52.7680},
    {"Baikonur, Kazakhstan", 45.9650, 63.3050},
    {"Equatorial Platform", 0.0, -154.0},
};
const int LAUNCH_SITE_COUNT = sizeof(LAUNCH_SITES) / sizeof(LAUNCH_SITES[0]);

const double HYPER_SPEEDS[] = {100 * C_LIGHT, 1000 * C_LIGHT, 100000 * C_LIGHT, 1e300};
const char* HYPER_SPEED_NAMES[] = {"100 c", "1,000 c", "100,000 c", "Instant"};
const int HYPER_SPEED_COUNT = 4;

const char* sasName(SAS s) {
    switch (s) {
    case SAS::Off: return "Off";
    case SAS::Stability: return "Hold";
    case SAS::Prograde: return "Prograde";
    case SAS::Retrograde: return "Retrograde";
    case SAS::Normal: return "Normal";
    case SAS::AntiNormal: return "Anti-normal";
    case SAS::RadialOut: return "Radial out";
    case SAS::RadialIn: return "Radial in";
    case SAS::Target: return "Target";
    case SAS::Burn: return "Burn";
    case SAS::Dock: return "Dock";
    case SAS::AntiTarget: return "Anti-target";
    case SAS::Sun: return "Star";
    case SAS::Ascent: return "Ascent";
    default: return "";
    }
}

// ------------------------------------------------------------------ formatting
std::string formatDist(double m) {
    char b[64];
    double a = std::fabs(m);
    if (a < 1e4) std::snprintf(b, sizeof b, "%.0f m", m);
    else if (a < 1e7) std::snprintf(b, sizeof b, "%.1f km", m / 1e3);
    else if (a < 1e10) std::snprintf(b, sizeof b, "%.0f km", m / 1e3);
    else if (a < 0.05 * LY) std::snprintf(b, sizeof b, "%.3f AU", m / AU);
    else std::snprintf(b, sizeof b, "%.3f ly", m / LY);
    return b;
}
std::string formatSpeed(double ms) {
    char b[64];
    if (std::fabs(ms) < 1e4) std::snprintf(b, sizeof b, "%.1f m/s", ms);
    else if (std::fabs(ms) < 0.01 * C_LIGHT) std::snprintf(b, sizeof b, "%.2f km/s", ms / 1e3);
    else std::snprintf(b, sizeof b, "%.4f c", ms / C_LIGHT);
    return b;
}
std::string formatTime(double s) {
    char b[64];
    bool neg = s < 0;
    s = std::fabs(s);
    if (!std::isfinite(s)) return "--";
    long long tot = (long long)s;
    long long y = tot / (long long)YEAR;
    tot -= (long long)(y * YEAR);
    long long d = tot / 86400;
    tot %= 86400;
    int h = (int)(tot / 3600), mi = (int)(tot % 3600 / 60), se = (int)(tot % 60);
    if (y > 0) std::snprintf(b, sizeof b, "%s%lldy %lldd %02d:%02d:%02d", neg ? "-" : "", y, d, h, mi, se);
    else if (d > 0) std::snprintf(b, sizeof b, "%s%lldd %02d:%02d:%02d", neg ? "-" : "", d, h, mi, se);
    else std::snprintf(b, sizeof b, "%s%02d:%02d:%02d", neg ? "-" : "", h, mi, se);
    return b;
}
std::string formatDate(double t) {
    double jd = 2451545.0 + t / DAY + 0.5;
    double Z = std::floor(jd), F = jd - Z;
    double A = Z;
    if (Z >= 2299161) {
        double al = std::floor((Z - 1867216.25) / 36524.25);
        A = Z + 1 + al - std::floor(al / 4);
    }
    double B = A + 1524, C = std::floor((B - 122.1) / 365.25), D = std::floor(365.25 * C), E = std::floor((B - D) / 30.6001);
    double day = B - D - std::floor(30.6001 * E) + F;
    int month = (int)(E < 14 ? E - 1 : E - 13);
    long long year = (long long)(month > 2 ? C - 4716 : C - 4715);
    int dd = (int)day;
    double fr = day - dd;
    int hh = (int)(fr * 24), mm = (int)((fr * 24 - hh) * 60);
    char b[64];
    std::snprintf(b, sizeof b, "%04lld-%02d-%02d %02d:%02d UTC", year, month, dd, hh, mm);
    return b;
}

// ------------------------------------------------------------------ world basics
void World::toast(const std::string& s, Rgba8 c) {
    toasts.push_back({s, realTime, c});
    while (toasts.size() > 6) toasts.pop_front();
    missionLog.push_back({t, s, c});
    while (missionLog.size() > 300) missionLog.pop_front();
}

static void followCoM(World& W, Vessel& v, const Vec3d& oldCom, double t); // (keeps a vessel in place when its CoM moves)
namespace { void pinLanded(World& W, Vessel& v, double t); } // (defined below)

void World::newGame(const Settings& s) {
    U.init();
    warpHold = -1;
    settings = s;
    // start at the real current date (seconds since J2000.0 = 2000-01-01 12:00 UTC)
    t = (double)std::time(nullptr) - 946728000.0 + g_epochOffset;
    vessels.clear();
    active = -1;
    nextId = 1;
    warpIndex = 0;
    moduleNames.clear();
    mapName.clear();
    bases.clear();
    nextBaseId = 1;
    achievements.clear();
    newAchievements.clear();
    recordsOn = true;
    dockHoldUntil = -1;
    explored.assign(U.count(), 0);
    explored[U.find("Earth")] = 1 | 2 | 4;
    toasts.clear();
    hyper = {};
    refreshStates();
    syncBaseTerrain();
}

// ---------------------------------------------------------------- launch sites and surface bases
World::PadSite World::padSite() const {
    PadSite ps;
    if (settings.launchBase > 0) {
        int bi = findBase(*this, settings.launchBase);
        double x = 0, y = 0;
        if (bi >= 0 && padOffset(bases[bi], x, y)) {
            const SurfaceBase& b = bases[bi];
            ps.body = b.body;
            ps.up = baseDir(b, U.bodies[b.body].radius, x, y);
            ps.deckH = BASE_PAD_DECK_H;
            ps.baseId = b.id;
            return ps;
        }
    }
    const LaunchSite& site = LAUNCH_SITES[std::clamp(settings.launchSite, 0, LAUNCH_SITE_COUNT - 1)];
    double lat = site.latDeg * DEG, lon = site.lonDeg * DEG;
    ps.body = U.find("Earth");
    ps.up = {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
    return ps;
}

std::vector<int> World::launchBaseIds() const {
    std::vector<int> ids;
    for (const SurfaceBase& b : bases)
        if (baseStatus(b).hasPad) ids.push_back(b.id);
    return ids;
}

std::string World::launchSiteName() const {
    PadSite ps = padSite();
    if (ps.baseId > 0) return bases[findBase(*this, ps.baseId)].name;
    return LAUNCH_SITES[std::clamp(settings.launchSite, 0, LAUNCH_SITE_COUNT - 1)].name;
}

int World::launchSiteSlots() const { return LAUNCH_SITE_COUNT + (int)launchBaseIds().size(); }

int World::launchSiteSlot() const {
    std::vector<int> ids = launchBaseIds();
    if (settings.launchBase > 0)
        for (int i = 0; i < (int)ids.size(); i++)
            if (ids[i] == settings.launchBase) return LAUNCH_SITE_COUNT + i;
    return std::clamp(settings.launchSite, 0, LAUNCH_SITE_COUNT - 1);
}

void World::cycleLaunchSite(int step) {
    std::vector<int> ids = launchBaseIds();
    int n = LAUNCH_SITE_COUNT + (int)ids.size();
    int cur = ((launchSiteSlot() + step) % n + n) % n;
    if (cur < LAUNCH_SITE_COUNT) {
        settings.launchSite = cur;
        settings.launchBase = 0;
    } else settings.launchBase = ids[cur - LAUNCH_SITE_COUNT];
}

void World::unloadComponents(int vi, std::vector<int> idx) {
    if (vi < 0 || vi >= (int)vessels.size()) return;
    Vessel& v = vessels[vi];
    std::sort(idx.begin(), idx.end());
    std::vector<Component> nc;
    std::vector<double> np, nh;
    for (int i = 0; i < (int)v.comps.size(); i++) {
        if (std::binary_search(idx.begin(), idx.end(), i)) continue;
        nc.push_back(v.comps[i]);
        np.push_back(i < (int)v.prop.size() ? v.prop[i] : 0.0);
        nh.push_back(i < (int)v.heat.size() ? v.heat[i] : 0.0);
    }
    if (nc.empty()) {
        // nothing left of the lander: it is gone (other vessels stop pointing at it)
        int id = v.id;
        vessels.erase(vessels.begin() + vi);
        for (Vessel& o : vessels)
            if (o.targetVessel == id) o.targetVessel = -1;
        if (active == vi) active = -1;
        else if (active > vi) active--;
        return;
    }
    // what stood above an unloaded stack part comes down by its height (no gap in the lander's stack)
    {
        struct Gap { double z0, z1; };
        std::vector<Gap> gaps;
        auto extent = [](const Component& c, double& z0, double& z1) {
            double h = c.shape == Shape::Ellipsoid ? 2 * c.dims.z : c.dims.z;
            z0 = c.pos.z - h / 2;
            z1 = c.pos.z + h / 2;
        };
        for (int i : idx) {
            if (i < 0 || i >= (int)v.comps.size()) continue;
            const Component& c = v.comps[i];
            if (std::hypot(c.pos.x, c.pos.y) > 0.2 || c.rotDeg.x != 0 || c.rotDeg.y != 0) continue; // (stack parts only)
            Gap g;
            extent(c, g.z0, g.z1);
            gaps.push_back(g);
        }
        for (Component& k : nc) {
            double drop = 0;
            for (const Gap& g : gaps)
                if (k.pos.z > g.z1 - 1e-6) drop += g.z1 - g.z0;
            k.pos.z -= drop;
        }
    }
    Vec3d oldCom = v.mp.com;
    v.comps = nc;
    v.prop = np;
    v.heat = nh;
    setupVessel(v, true);
    followCoM(*this, v, oldCom, t);
    if (v.landed && !v.contacts.empty() && v.ref >= 0) {
        // seated again on its lowest contact (feet, wheels)
        Vec3d up = norm(v.landPos);
        double low = 0;
        for (const Vec3d& k : v.contacts) low = std::min(low, dot(v.landQ.rotate(k - v.mp.com), up));
        double ground = U.bodies[v.ref].radius + ground::height(v.ref, up);
        v.landPos = up * (ground - low + 0.02);
        pinLanded(*this, v, t);
    }
}

double World::groundHeight(int body, const Vec3d& p) const { return ground::height(body, p); }

void World::moveLanded(Vessel& v, const Vec3d& delta) {
    v.landPos += delta;
    pinLanded(*this, v, t);
}

void World::syncBaseTerrain() const {
    for (int body = 0; body < U.count(); body++) {
        std::vector<Vec3d> c;
        for (const SurfaceBase& b : bases)
            if (b.body == body) c.push_back(baseCentreDir(b));
        ground::setFlatSpots(body, c, BASE_FLAT_RADIUS);
    }
}

double g_epochOffset = 0; // tests: start the game this many seconds after the real date (--epoch <days>)

bool predictEncounter(const Universe& U, int ref, Vec3d r, Vec3d v, double t0, Encounter& out) {
    const Body& rb = U.bodies[ref];
    if (rb.children.empty()) return false;
    Conic c = stateToConic(r, v, rb.GM);
    if (!c.valid) return false;
    double soi = rb.parent >= 0 ? rb.soi : 1e300;
    if (soi > 1e299) soi = std::max(len(r) * 50, 1e13);
    double horizon = c.e < 1 && c.ap < soi ? c.period : 0;
    if (horizon == 0) {
        double cosExit = (c.p / soi - 1) / std::max(c.e, 1e-9);
        double nuExit = std::fabs(cosExit) <= 1 ? std::acos(cosExit) : PI * 0.99;
        horizon = timeToTrueAnomaly(c, nuExit);
        if (horizon <= 0) horizon = 30 * DAY;
    }
    horizon = std::min(horizon, 2 * YEAR);
    const int N = 400;
    const double dt = horizon / N;
    std::vector<BodyState> S;
    auto inside = [&](int ch, const Vec3d& p, double tRel) {
        U.computeStatesFor(t0 + tRel, U.relevant[ref], S);
        return len(p - U.relPos(S, ch, ref)) < U.bodies[ch].soi;
    };
    for (int k = 1; k <= N; k++) {
        Vec3d r0 = r, v0 = v;
        keplerPropagate(r, v, rb.GM, dt);
        for (int ch : rb.children) {
            if (!inside(ch, r, k * dt)) continue;
            // the crossing lies inside this step
            double lo = 0, hi = dt;
            Vec3d rr = r, vv = v;
            for (int it = 0; it < 22; it++) {
                double mid = 0.5 * (lo + hi);
                rr = r0;
                vv = v0;
                keplerPropagate(rr, vv, rb.GM, mid);
                (inside(ch, rr, (k - 1) * dt + mid) ? hi : lo) = mid;
            }
            rr = r0;
            vv = v0;
            keplerPropagate(rr, vv, rb.GM, hi);
            U.computeStatesFor(t0 + (k - 1) * dt + hi, U.relevant[ref], S);
            out.body = ch;
            out.tEntry = (k - 1) * dt + hi;
            out.r = rr - U.relPos(S, ch, ref);
            out.v = vv - U.relVel(S, ch, ref);
            return true;
        }
    }
    return false;
}

double World::warpFactor() const { return WARP_LEVELS[std::clamp(warpIndex, 0, WARP_COUNT - 1)]; }

Vec3d g_rotDbg[6]; // (diagnostics for the tests: the active vessel's last attitude step)
namespace { // defined with the engine model below
bool thrustPossible(const World& W, const Vessel& v);
} // namespace

// dynamic pressure of the air on a vessel [Pa] (0 outside an atmosphere)
static double airDynP(const World& W, const Vessel& v) {
    const Body& b = W.U.bodies[v.ref];
    double alt = len(v.r) - b.radius;
    if (b.atmTop <= 0 || alt >= b.atmTop) return 0;
    Vec3d va = v.v - cross(W.U.angVel(v.ref), v.r);
    return 0.5 * W.U.density(v.ref, alt) * dot(va, va);
}

// the highest warp a vessel inside an atmosphere allows: real-time physics rates (x4) in dense air, under
// power, under a parachute and for aircraft; faster through the thin top layers where the drag hardly acts
// (the translation is still integrated with the drag, only the attitude is held as on rails)
static int airWarpIndex(const World& W, const Vessel& v) {
    if (v.drivePower > 0 && (v.rolling || v.landed)) return WARP_COUNT - 1; // a rover on the ground: the rule for rolling vessels
    if (thrustPossible(W, v) || (v.chuteDeployed && !v.chuteCut) || v.isAircraft) return PHYSICS_WARP_MAX;
    double q = airDynP(W, v);
    if (q < 0.5) return THIN_AIR_WARP;
    if (q < 5) return 7;  // x1000
    if (q < 50) return 5; // x50
    return PHYSICS_WARP_MAX;
}

int World::maxWarpIndex() const {
    int mx = WARP_COUNT - 1;
    for (const auto& v : vessels) {
        if (v.destroyed || v.landed || v.debris) continue; // falling debris never holds the warp back
        if (v.rolling) {
            // on its wheels: physics steps (x100 while crawling, as a rover; x10 faster)
            Vec3d vs = v.v - cross(U.angVel(v.ref), v.r);
            mx = std::min(mx, len(vs) < 0.5 ? 6 : 4);
        }
        const Body& b = U.bodies[v.ref];
        double alt = len(v.r) - b.radius;
        if (b.atmTop > 0 && alt < b.atmTop) mx = std::min(mx, airWarpIndex(*this, v));
        // close above an airless surface on a course into it: no more than x10
        if (b.atmTop <= 0 && !b.isStar && !b.gasGiant && alt < ground::maxHeight(v.ref) + surfaceApproach(b)) {
            Conic k = stateToConic(v.r, v.v, b.GM);
            if (k.valid && k.pe < b.radius + ground::maxHeight(v.ref)) mx = std::min(mx, 4);
        }
    }
    return mx;
}

// time until a coasting vessel comes down through the radius rf (-1: it does not, or it is below it now)
static double timeToRadiusInbound(const Vec3d& r, const Vec3d& v, double GM, double rf) {
    if (len(r) <= rf) return -1;
    Conic c = stateToConic(r, v, GM);
    if (!c.valid || c.pe >= rf || c.e < 1e-9) return -1;
    double cosNu = (c.p / rf - 1) / c.e;
    if (std::fabs(cosNu) > 1) return -1;
    return timeToTrueAnomaly(c, -std::acos(cosNu));
}

void World::setWarp(int idx) {
    if (warpTo > 0) cancelWarpTo(nullptr); // a manual warp change takes over
    int mx = maxWarpIndex();
    if (idx > mx) {
        bool inAir = false;
        for (const auto& v : vessels)
            if (!v.destroyed && !v.landed && !v.debris && U.bodies[v.ref].atmTop > 0 && len(v.r) - U.bodies[v.ref].radius < U.bodies[v.ref].atmTop) inAir = true;
        toast(!inAir ? "Time warp limited: a vessel is close above the ground on a course into it"
              : mx >= THIN_AIR_WARP ? "Time warp limited: a vessel is inside an atmosphere"
                                    : "Time warp limited: a vessel is in dense air or under power",
              {255, 200, 120, 255});
        idx = mx;
    }
    warpIndex = std::clamp(idx, 0, WARP_COUNT - 1);
}

// ------------------------------------------------------------------ light / environment
// Total stellar flux at a point (W/m^2), shadowing by the reference body chain
static double starFlux(const World& W, const std::vector<BodyState>& S, int ref, const Vec3d& r, bool* shadow,
                       Vec3d* brightestDir, std::vector<std::pair<Vec3d, double>>* each) {
    const Universe& U = W.U;
    double total = 0, best = -1;
    bool shadowed = false;
    for (int s : U.relevant[ref]) {
        const Body& sb = U.bodies[s];
        if (!sb.isStar) continue;
        Vec3d d = U.relPos(S, s, ref) - r;
        double dl = len(d);
        if (dl < 1) continue;
        Vec3d dh = d / dl;
        double f = sb.luminosity / (4 * PI * dl * dl);
        // occlusion by non-star bodies in the reference chain (ref + parents)
        bool occ = false;
        int o = ref;
        while (o >= 0 && !U.bodies[o].isStar) {
            Vec3d c = U.relPos(S, o, ref) - r;
            double tt = dot(c, dh);
            if (tt > 0 && tt < dl) {
                double miss = len(c - dh * tt);
                if (miss < U.bodies[o].radius) { occ = true; break; }
            }
            o = U.bodies[o].parent;
        }
        // the brightest star decides light and shadow even when it is hidden: a faint star elsewhere
        // must not take over as the light source while the Sun is behind the planet
        if (f > best) {
            best = f;
            shadowed = occ;
            if (brightestDir) *brightestDir = dh;
        }
        if (occ) {
            if (each) each->push_back({dh, 0});
            continue;
        }
        total += f;
        if (each) each->push_back({dh, f});
    }
    if (shadow) *shadow = shadowed;
    return total;
}

double World::solarFluxAt(int ref, const Vec3d& r, bool* shadow, Vec3d* dirToBrightest) const {
    return starFlux(*this, states, ref, r, shadow, dirToBrightest, nullptr);
}

Vec3d World::sunDirFrom(int ref, const Vec3d& r) const {
    Vec3d best{1, 0, 0};
    double bf = -1;
    for (int s : U.relevant[ref]) {
        const Body& sb = U.bodies[s];
        if (!sb.isStar) continue;
        Vec3d d = U.relPos(states, s, ref) - r;
        double dl = len(d);
        if (dl < 1) continue;
        double f = sb.luminosity / (dl * dl);
        if (f > bf) { bf = f; best = d / dl; }
    }
    return best;
}

Vec3d World::surfaceVel(const Vessel& v) const { return v.v - cross(U.angVel(v.ref), v.r); }

static bool engineFires(const Vessel& v, const Component& c);
// craft-frame shift of an engine cluster's thrust point when the pilot switched some of its nozzles off
static Vec3d engineThrustOffset(const Component& c) {
    int n = engineCount(c);
    if (!c.engOff || n <= 1) return {};
    double bellR;
    auto pos = clusterLayout(n, std::max(c.dims.y, 0.1), bellR, 1.0);
    Vec3d s;
    int on = 0;
    for (int i = 0; i < n && i < (int)pos.size(); i++)
        if (!((c.engOff >> i) & 1)) { s = s + pos[i]; on++; }
    return on > 0 ? c.rot().rotate(s / (double)on) : Vec3d{};
}
namespace {
bool groupHasEngine(const Vessel& v, int g);
} // namespace
double ascentTargetAlt(const Body& b) { return b.atmTop > 0 ? b.atmTop + 60e3 : std::max(30e3, 0.02 * b.radius); }

// Ascent guidance, attitude part (the phases are switched in World::update):
//  0 boost: straight up, a pitch programme through the dense air (angle from the vertical grows with
//    altitude), then thrust along the velocity, which raises the apoapsis at zero angle of attack; a weak
//    stage that comes close to its apoapsis too early gets its nose raised to carry its weight,
//  1 coast: nose along the velocity, turning to the insertion attitude shortly before the burn,
//  2 orbit insertion: thrust near the horizon, steered to arrive at the target altitude with no vertical
//    speed and to stay there (the nose carries the weight that the speed does not carry yet).
static Vec3d ascentDirection(const World& W, const Vessel& v) {
    const Body& b = W.U.bodies[v.ref];
    Vec3d up = norm(v.r);
    if (v.landed) return up;
    Vec3d pole = b.poleFrame.col(2);
    double r = len(v.r), alt = r - b.radius;
    Vec3d hv = v.v - up * dot(v.v, up);
    Vec3d hdir = len(hv) > 1 ? norm(hv) : cross(pole, up);
    if (len(hdir) < 1e-6) hdir = anyPerp(up);
    hdir = norm(hdir);
    const double turnStart = b.atmTop > 0 ? 800.0 : 150.0;
    if (alt < turnStart) return up;
    // a stack whose liquid core can hardly carry itself once its solid boosters are gone (the Space Shuttle:
    // thrust-to-weight ~1 after booster separation) flies a lofted programme: it turns later and higher
    double loft = 1;
    {
        double Fl = 0, mSrb = 0;
        bool srb = false;
        for (size_t i = 0; i < v.comps.size(); i++) {
            const Component& c = v.comps[i];
            if (c.func != Func::Engine || c.def < 0 || !engineFires(v, c)) continue;
            if (CATALOG.defs[c.def].kind == PK::SRB) {
                srb = true;
                mSrb += v.phys[i].dryMass + (i < v.prop.size() ? v.prop[i] : 0.0);
            } else Fl += v.phys[i].thrustVac * engineShare(c);
        }
        if (srb && Fl > 0) {
            double twr = Fl / (std::max(v.mp.mass - mSrb, 1.0) * b.GM / (r * r));
            loft = clampd(1.5 / twr, 1.0, 2.5);
        }
    }
    const double hTurn = (b.atmTop > 0 ? b.atmTop * 0.34 : std::max(3000.0, 0.004 * b.radius)) * loft;
    const double hProg = (b.atmTop > 0 ? b.atmTop * 0.25 : hTurn * 0.5 / loft) * loft; // the programme ends here
    // a core on a drop tank after its boosters (thrust-to-weight ~1) leaves the pitch programme: it is steered by
    // its climb rate below (an open-loop programme lays such a stage flat in the air)
    bool dropCore = false;
    if (!groupHasEngine(v, v.stage))
        for (const Component& c : v.comps)
            if (c.func == Func::Engine && c.stage > v.stage && engineFires(v, c)) dropCore = true;
    if (v.ascentPhase == 0 && alt < hProg && !dropCore) {
        double th = std::min(80.0, 90.0 * std::sqrt((alt - turnStart) / hTurn)) * DEG;
        return up * std::cos(th) + hdir * std::sin(th);
    }
    const double hT = ascentTargetAlt(b);
    double vz = dot(v.v, up), vh = len(hv), sp = std::max(len(v.v), 1.0);
    double gE = std::max(0.0, b.GM / (r * r) - vh * vh / r); // weight not yet carried by the speed [m/s^2]
    double thrust = v.tThrust;
    if (thrust <= 0) { // coasting: the acceleration the stage will have
        double amt[PROP_N], cap[PROP_N], sa, sc, md;
        stageResources(v, amt, cap, sa, sc, thrust, md);
    }
    double aT = std::max(thrust / std::max(v.mp.mass, 1.0), 0.5);
    Conic c = stateToConic(v.r, v.v, b.GM);
    bool open = !c.valid || c.e >= 1;
    double tAp = vz > 0 && !open ? timeToTrueAnomaly(c, PI) : 0;
    double sinPro = clampd(vz / sp, -1, 1), sinP;
    const double tau = 8.0;
    if (v.ascentPhase == 0) {
        double w = clampd((50 - tAp) / 20, 0, 1);
        double vzHold = clampd((hT - alt) / 120, 0, 150);
        // a core burning from a drop tank with a thrust-to-weight near 1 (the Shuttle after its boosters) lets
        // its climb rate decay and builds speed instead of holding it up (gravity losses)
        if (!groupHasEngine(v, v.stage)) {
            bool drop = false;
            for (const Component& c : v.comps)
                if (c.func == Func::Engine && c.stage > v.stage && engineFires(v, c)) drop = true;
            // (in the air the climb rate is always held: flown prograde, such a stage sinks back into it)
            if (drop && b.atmTop > 0 && alt < 0.75 * b.atmTop) w = 1;
            if (drop) {
                vzHold *= clampd((aT / std::max(gE, 0.1) - 1.0) / 0.6, 0.15, 1.0);
                // but it keeps climbing out of the air that would heat it up as it gains speed (the Shuttle on its
                // external tank climbs past 100 km before it levels off): to 3/4 of the top of the atmosphere
                if (b.atmTop > 0) vzHold = std::max(vzHold, clampd((0.75 * b.atmTop - alt) / 40.0, 0.0, 180.0));
            }
        }
        sinP = std::max(sinPro, w * (gE + (vzHold - vz) / tau) / aT);
    } else {
        double vzBall = std::sqrt(2 * std::max(gE, 0.02) * std::max(hT - alt, 0.0));
        double ratio = vzBall > 1 ? clampd(vz / vzBall, 0, 1) : 0;
        sinP = (gE * (1 - ratio) + (vzBall - vz) / tau) / aT;
        if (v.ascentPhase == 1) sinP = tAp > 40 ? sinPro : sinPro + (sinP - sinPro) * clampd((40 - tAp) / 20, 0, 1);
    }
    sinP = clampd(sinP, -0.3, 0.9);
    return up * sinP + hdir * std::sqrt(1 - sinP * sinP);
}

Vec3d World::targetDirection(const Vessel& v, SAS mode) const {
    const Body& b = U.bodies[v.ref];
    double alt = len(v.r) - b.radius;
    bool surf = alt < (b.atmTop > 0 ? b.atmTop * 0.5 : 0.02 * b.radius);
    Vec3d vel = surf ? surfaceVel(v) : v.v;
    Vec3d nrm = cross(v.r, v.v);
    switch (mode) {
    case SAS::Prograde: return len(vel) > 0.05 ? norm(vel) : Vec3d{};
    case SAS::Retrograde: return len(vel) > 0.05 ? -norm(vel) : Vec3d{};
    case SAS::Normal: return len(nrm) > 1e-6 ? norm(nrm) : Vec3d{};
    case SAS::AntiNormal: return len(nrm) > 1e-6 ? -norm(nrm) : Vec3d{};
    case SAS::RadialOut: return len(nrm) > 1e-6 ? norm(cross(v.v, nrm)) : norm(v.r);
    case SAS::RadialIn: return len(nrm) > 1e-6 ? -norm(cross(v.v, nrm)) : -norm(v.r);
    case SAS::Target:
    case SAS::AntiTarget: {
        if (v.targetVessel >= 0) // a vessel: toward it (rendezvous, docking)
            for (const Vessel& o : vessels)
                if (o.id == v.targetVessel && !o.destroyed && o.ref == v.ref) {
                    Vec3d d = o.r - v.r;
                    if (len(d) < 0.1) return {};
                    return mode == SAS::Target ? norm(d) : -norm(d);
                }
        if (v.target < 0) return {};
        Vec3d d = U.relPos(states, v.target, v.ref) - v.r;
        if (len(d) < 1) return {};
        return mode == SAS::Target ? norm(d) : -norm(d);
    }
    case SAS::Sun: return sunDirFrom(v.ref, v.r);
    case SAS::Ascent: return ascentDirection(*this, v);
    case SAS::Burn: return len(v.navBurn) > 1e-3 ? norm(v.navBurn) : Vec3d{};
    case SAS::Dock: {
        // turn the vessel so that its docking port faces the target's port: the nose is turned by the
        // rotation that brings the port axis onto the wanted one (re-evaluated every step)
        for (const Vessel& o : vessels) {
            if (o.id != v.targetVessel || o.destroyed || o.ref != v.ref) continue;
            int ia, it;
            if (!navg::nearestPorts(*this, v, o, ia, it)) {
                Vec3d d = o.r - v.r;
                return len(d) > 0.1 ? norm(d) : Vec3d{};
            }
            Vec3d pa, da, pt, nt;
            portFrame(v, ia, pa, da);
            portFrame(o, it, pt, nt);
            Vec3d want = -nt, nose = v.q.rotate({0, 0, 1});
            Vec3d ax = cross(da, want);
            double s = len(ax), c = dot(da, want);
            if (s < 1e-9) {
                if (c > 0) return nose;
                ax = anyPerp(da);
                s = 1;
            }
            Vec3d k = ax / s;
            double ang = std::atan2(s, c);
            // Rodrigues rotation of the nose about k by ang
            Vec3d r = nose * std::cos(ang) + cross(k, nose) * std::sin(ang) + k * (dot(k, nose) * (1 - std::cos(ang)));
            return norm(r);
        }
        return {};
    }
    default: return {};
    }
}

// ------------------------------------------------------------------ vessel structure
static Mat3d invert3(const Mat3d& A) {
    const auto& m = A.m;
    double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                 m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    Mat3d r;
    if (std::fabs(det) < 1e-30) {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) r.m[i][j] = i == j ? 1.0 / std::max(m[i][i], 1e-6) : 0;
        return r;
    }
    double id = 1.0 / det;
    r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * id;
    r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * id;
    r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * id;
    r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * id;
    r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * id;
    r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * id;
    r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * id;
    r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * id;
    r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * id;
    return r;
}

void updateMassProps(Vessel& v) {
    Moments M;
    for (size_t i = 0; i < v.comps.size(); i++) {
        M.add(v.phys[i].dryM);
        if (v.prop[i] > 0) {
            Moments p = v.phys[i].unitPropM;
            p.m *= v.prop[i];
            p.s *= v.prop[i];
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++) p.q[a][b] *= v.prop[i];
            M.add(p);
        }
    }
    v.mp = toMassProps(M);
    if (v.mp.mass < 1) {
        v.mp.mass = 1;
        v.mp.inertia = Mat3d{};
    }
    // keep the tensor well conditioned for tiny / degenerate crafts
    for (int k = 0; k < 3; k++) v.mp.inertia.m[k][k] = std::max(v.mp.inertia.m[k][k], 1e-3 * v.mp.mass + 1e-3);
    v.invI = invert3(v.mp.inertia);
}

void setupVessel(Vessel& v, bool rebuildAero) {
    int n = (int)v.comps.size();
    v.phys.assign(n, {});
    for (int i = 0; i < n; i++) computeCompPhys(v.comps[i], v.phys[i]);
    if ((int)v.prop.size() != n) {
        v.prop.assign(n, 0);
        for (int i = 0; i < n; i++)
            v.prop[i] = v.phys[i].propCap * (v.comps[i].func == Func::Tank ? v.comps[i].fill : 1.0);
    }
    if ((int)v.heat.size() != n) v.heat.assign(n, 0);
    v.stage = 1 << 30;
    for (auto& c : v.comps) v.stage = std::min(v.stage, c.stage);
    if (n == 0) v.stage = 0;
    v.genW = v.drawW = v.solarArea = v.sailArea = v.chuteCdA = v.torque = 0;
    v.crew = 0;
    v.hasControl = v.hasLegs = v.hasHyperdrive = v.hasFairing = v.hasWheels = v.isAircraft = v.gearRetract = v.rotorcraft = false;
    v.hasFlaps = false;
    v.flapMax = 0;
    v.bayComp = -1;
    v.chuteQMax = 25000;
    v.chuteJettV = 0;
    v.chuteReefK = 1;
    v.chuteReefT = 0;
    for (int i = 0; i < n; i++) {
        const Component& c = v.comps[i];
        if (c.func == Func::Wing && c.shape == Shape::Airfoil && c.flap > 0) {
            v.hasFlaps = true;
            v.flapMax = std::max(v.flapMax, c.flap);
        }
        if (c.func == Func::CargoBay && v.bayComp < 0) v.bayComp = i;
        if (c.func == Func::Parachute && c.def >= 0 && c.def < (int)CATALOG.defs.size()) {
            const PartDef& d = CATALOG.defs[c.def];
            v.chuteQMax = std::min(v.chuteQMax, d.chuteQMax);
            v.chuteJettV = std::max(v.chuteJettV, d.chuteJettisonV);
            if (d.chuteReefT > 0) {
                v.chuteReefK = d.chuteReefK;
                v.chuteReefT = d.chuteReefT;
            }
        }
    }
    if (!v.hasFlaps) v.flapStage = 0, v.flapPos = 0;
    if (v.bayComp < 0) v.bayOpen = false, v.bayT = 0;
    v.drivePower = v.driveVmax = 0;
    double battery = 0;
    v.contacts.clear();
    v.contactLeg.clear();
    double zmin = 1e30, zmax = -1e30, rmax = 0.3;
    for (int i = 0; i < n; i++) {
        const Component& c = v.comps[i];
        const CompPhys& p = v.phys[i];
        battery += p.batteryJ;
        v.genW += p.genW;
        v.drawW += p.drawW;
        v.solarArea += p.solarArea;
        v.sailArea += p.sailArea;
        v.chuteCdA += p.chuteCdA;
        v.torque += p.torque;
        v.crew += p.crew;
        if (c.func == Func::Command) v.hasControl = true;
        if (c.func == Func::Legs) v.hasLegs = true;
        // a lift rotor (not a propeller) makes a rotorcraft, whether it stands on skids or on wheels
        if (c.func == Func::Rotor && c.def >= 0 && CATALOG.defs[c.def].id.rfind("rotor", 0) == 0) v.rotorcraft = true;
        if (c.func == Func::Hyperdrive) v.hasHyperdrive = true;
        if (c.func == Func::Fairing) v.hasFairing = true;
        Mat3d R = c.rot().toMat();
        if (c.func == Func::Legs) {
            v.contacts.push_back(c.pos + R * Vec3d{0, 0, -c.dims.z / 2});
            v.contactLeg.push_back(1);
        } else if (c.func == Func::Wheel) {
            double tyre = c.def >= 0 ? CATALOG.defs[c.def].diam * 0.5 : 0.35;
            v.contacts.push_back(c.pos + R * Vec3d{0, 0, -c.dims.z / 2 - tyre});
            bool retract = retractableWheel(c);
            v.contactLeg.push_back(retract ? 2 : 3); // 2: landing gear (folds), 3: rover wheel (always down)
            v.hasWheels = true;
            if (retract) {
                v.gearRetract = true;
                v.hasLegs = true; // (the legs command raises and lowers the gear)
            } else if (c.def >= 0) {
                const PartDef& d = CATALOG.defs[c.def];
                v.drivePower += d.powerUse;
                v.driveVmax = v.driveVmax > 0 ? std::min(v.driveVmax, d.maxSpeed) : d.maxSpeed;
            }
        } else {
            std::vector<Vec3d> pts;
            shapeContactPoints(c.shape, c.dims, pts);
            for (auto& q : pts) {
                v.contacts.push_back(c.pos + R * q);
                v.contactLeg.push_back(0);
            }
        }
    }
    for (auto& c : v.contacts) {
        zmin = std::min(zmin, c.z);
        zmax = std::max(zmax, c.z);
        rmax = std::max(rmax, std::sqrt(c.x * c.x + c.y * c.y));
    }
    v.length = n ? std::max(0.5, zmax - zmin) : 1;
    v.maxR = rmax;
    v.chargeCap = battery;
    v.charge = std::min(v.charge, battery);
    v.solarAreaMax = v.solarArea;
    if (!v.solarDeployed) v.solarArea = 0;
    v.shown.clear();
    v.stowedGeom.clear();
    if (v.solarAreaMax > 0) {
        v.stowedGeom = stowedGeometry(v.comps);
        if (!v.solarDeployed) v.shown = v.stowedGeom;
    }
    // landing gear up: the airflow sees the folded wheels (less drag)
    if (!v.legsDeployed) {
        bool any = false;
        for (const Component& c : v.comps) any |= retractableWheel(c);
        if (any) {
            if (v.shown.empty()) v.shown = v.comps;
            for (Component& c : v.shown) c = stowedWheel(c);
        }
    }
    v.rcsNoz.clear();
    v.rcsAuth = 0;
    v.monoCap = 0;
    for (int i = 0; i < n; i++) {
        if (v.comps[i].func == Func::Tank && v.comps[i].prop == Prop::Monoprop) v.monoCap += v.phys[i].propCap;
        if (v.comps[i].func != Func::RCS || v.phys[i].rcsF <= 0) continue;
        Vec3d p[4], d[4];
        rcsNozzles(v.comps[i], p, d);
        for (int k = 0; k < 4; k++) v.rcsNoz.push_back({p[k], d[k], v.phys[i].rcsF, v.phys[i].rcsIsp});
    }
    v.rcsFire.assign(v.rcsNoz.size(), 0.f);
    v.engLevel.assign(n, 0.f);
    // attitude-control thrusters of the active stage: sized from its engines, fed from its tanks
    {
        double F = 0, isp = 0;
        Prop pr = Prop::None;
        for (int i = 0; i < n; i++) {
            const Component& c = v.comps[i];
            if (c.stage != v.stage || c.func != Func::Engine || v.phys[i].thrustVac <= 0 || v.phys[i].propCap > 0) continue; // liquid only
            F += v.phys[i].thrustVac;
            if (pr == Prop::None && c.def >= 0) {
                pr = CATALOG.defs[c.def].prop;
                isp = v.phys[i].ispVac;
            }
        }
        bool tank = false;
        for (int i = 0; i < n && pr != Prop::None; i++)
            tank |= v.comps[i].stage == v.stage && v.comps[i].func == Func::Tank && v.comps[i].prop == pr && !(v.comps[i].cargo && v.bayComp >= 0);
        if (!tank) { // no engine propellant in the stage: use any liquid tank of the stage (e.g. a capsule's hypergolics)
            pr = Prop::None;
            for (int i = 0; i < n; i++)
                if (v.comps[i].stage == v.stage && v.comps[i].func == Func::Tank && v.comps[i].prop != Prop::Xenon && !(v.comps[i].cargo && v.bayComp >= 0)) {
                    pr = v.comps[i].prop;
                    break;
                }
            isp = 230;
        }
        v.acsProp = pr;
        v.acsF = pr == Prop::None ? 0 : std::clamp(0.0025 * F, 150.0, 25000.0);
        v.acsIsp = std::max(150.0, (isp > 0 ? isp : 230) * 0.8);
    }
    if (rebuildAero || !v.aero) v.aero = aeroTableCached(v.geometry());
    else if (v.aero->key != geometryKey(v.geometry()) || v.aero->ncomp != (int)v.geometry().size())
        v.aero.reset(); // the shape changed out of the air (gear, panels, parts gone): rebuilt when it meets air again
    updateMassProps(v);
    v.rcsAuthAxis = {};
    for (auto& nz : v.rcsNoz) {
        Vec3d t = cross(nz.pos - v.mp.com, nz.dir) * (nz.F * 0.5);
        v.rcsAuth += len(t);
        v.rcsAuthAxis += Vec3d{std::fabs(t.x), std::fabs(t.y), std::fabs(t.z)};
    }
    // control surfaces: every wing / fin has a moving part (elevator, aileron, rudder). A surface moves by how
    // much its lift turns the vessel about the commanded axis (see surfAlpha); its torque at full deflection
    // per pascal of dynamic pressure: lift slope x area x effectiveness x travel, times the lever
    v.ctrlAuth = {};
    bool rocket = false, air = false;
    for (int i = 0; i < n; i++) {
        const Component& c = v.comps[i];
        if (c.func == Func::Engine || (c.func == Func::Jet && c.def >= 0 && worksWithoutAir(CATALOG.defs[c.def]))) rocket = true;
        if (c.func == Func::Rotor || c.func == Func::Jet || c.func == Func::Wheel) air = true;
    }
    // tailless: no lifting surface of some size turns the craft mostly in pitch (a small trim surface such as an
    // orbiter's body flap does not count: the wings' flaps stay elevons)
    v.tailless = true;
    bool anyWing = false;
    double Sall = 0;
    for (int i = 0; i < n; i++)
        if (v.comps[i].func == Func::Wing) Sall += std::max(0.05, v.comps[i].dims.x) * std::max(0.05, v.comps[i].dims.y);
    for (int i = 0; i < n; i++) {
        const Component& c = v.comps[i];
        if (c.func != Func::Wing) continue;
        anyWing = true;
        Vec3d g0 = cross(surfPoint(c, v.phys[i]) - v.mp.com, v.phys[i].axisZ);
        double S = std::max(0.05, c.dims.x) * std::max(0.05, c.dims.y);
        if (std::fabs(g0.x) > std::fabs(g0.y) && std::fabs(g0.x) > std::fabs(g0.z) && S > 0.07 * Sall) v.tailless = false;
    }
    if (!anyWing) v.tailless = false;
    {
        double Sw = 0;
        for (const Component& c : v.comps)
            if (c.func == Func::Wing && c.shape == Shape::Airfoil) Sw += std::max(0.0, c.dims.x) * std::max(0.0, c.dims.y);
        v.lifting = Sw > 8.0;
    }
    // torque per pascal at full deflection, per axis: lift change x its lever (at the flap)
    for (int i = 0; i < n; i++) {
        const Component& c = v.comps[i];
        if (c.func != Func::Wing) continue;
        double S = std::max(0.05, c.dims.x) * std::max(0.05, c.dims.y);
        Vec3d g0 = cross(surfPoint(c, v.phys[i]) - v.mp.com, v.phys[i].axisZ);
        double k = S * wingLiftSlope(c);
        v.ctrlAuth += Vec3d{std::fabs(k * surfDeflect(g0, {1, 0, 0}, v.tailless) * g0.x), std::fabs(k * surfDeflect(g0, {0, 1, 0}, v.tailless) * g0.y),
                            std::fabs(k * surfDeflect(g0, {0, 0, 1}, v.tailless) * g0.z)};
    }
    v.isAircraft = air && !rocket;
}

// ------------------------------------------------------------------ RCS
static double monopropAmount(const Vessel& v) {
    double a = 0;
    for (size_t i = 0; i < v.comps.size(); i++)
        if (v.comps[i].func == Func::Tank && v.comps[i].prop == Prop::Monoprop) a += v.prop[i];
    return a;
}

namespace { // defined with the engine model below
double stageTankAmount(const Vessel& v, Prop p);
void stageTankConsume(Vessel& v, Prop p, double amount);
} // namespace

static void rcsBegin(const World& W, Vessel& v) {
    v.rcsUse = 0;
    v.rcsImpulse = {};
    v.rcsFireTime = 0;
    std::fill(v.rcsFire.begin(), v.rcsFire.end(), 0.f);
    v.monoAmt = W.settings.infiniteFuel ? v.monoCap : monopropAmount(v);
    // energy source of this step
    bool monoOk = W.settings.infiniteFuel || v.monoAmt > 1e-9;
    bool elecOk = W.settings.infiniteFuel || v.charge > 1.0;
    v.rcsElectric = v.rcsSource == RcsSource::Battery ? true : v.rcsSource == RcsSource::Monoprop ? false : (!monoOk && elecOk);
    v.rcsElecJ = 0;
    v.rcsChargeAvail = W.settings.infiniteFuel ? 1e30 : v.charge;
    v.acsUse = 0;
    v.tAcs = 0;
    v.acsAmt = v.acsF <= 0 ? 0 : (W.settings.infiniteFuel ? 1e30 : stageTankAmount(v, v.acsProp));
}

// torque authority of the attitude thrusters per axis (pitch / yaw act over half the length, roll over the radius)
static Vec3d acsAuthority(const Vessel& v) {
    if (v.acsF <= 0 || v.acsAmt - v.acsUse <= 1e-9) return {};
    double lever = std::max(0.5, v.length * 0.45), roll = std::max(0.3, v.maxR);
    return {v.acsF * lever, v.acsF * lever, v.acsF * roll};
}

static bool rcsReady(const World& W, const Vessel& v) {
    if (!v.rcs || v.rcsNoz.empty()) return false;
    if (W.settings.infiniteFuel) return true;
    if (v.rcsElectric) return v.rcsChargeAvail - v.rcsElecJ > 1.0;
    return v.monoAmt - v.rcsUse > 1e-9;
}
static double rcsThrustScale(const Vessel& v) { return v.rcsElectric ? RCS_ELEC_THRUST : 1.0; }

// Thrust allocation by projection: every nozzle fires in proportion to how well its torque (and force)
// lines up with the request. fullTorque: pilot input, the best aligned nozzles fire at full thrust.
// Returns the RCS torque (craft axes); accumulates impulse, propellant use and visuals.
static Vec3d rcsAllocate(Vessel& v, const Vec3d& tauWant, bool fullTorque, const Vec3d& fWant, double hs) {
    size_t n = v.rcsNoz.size();
    static thread_local std::vector<double> u;
    u.assign(n, 0.0);
    double tl = len(tauWant);
    if (tl > 1e-9) {
        Vec3d g = tauWant / tl;
        double s2 = 0, cmax = 0;
        for (size_t j = 0; j < n; j++) {
            const RcsNozzle& z = v.rcsNoz[j];
            double c = std::max(0.0, dot(cross(z.pos - v.mp.com, z.dir) * (z.F * rcsThrustScale(v)), g));
            u[j] = c;
            s2 += c * c;
            cmax = std::max(cmax, c);
        }
        if (s2 > 0) {
            double k = fullTorque ? 1.0 / cmax : tl / s2;
            for (size_t j = 0; j < n; j++) u[j] *= k;
        }
        if (!fullTorque && s2 > 0) {
            // the projection leaves torque about other axes where the nozzles stand unevenly (a yaw command that
            // also rolls the orbiter: its nose thrusters sit high on the sides), and more when the request is
            // beyond them and they are cut to full thrust; a few projected-gradient passes take the residual
            // out, measured as angular acceleration (a long craft rolls far more easily than it yaws, so a little
            // roll torque matters as much as a lot of yaw). (step 1.5 / sum |a_j|^2 < 2 / the largest eigenvalue)
            static thread_local std::vector<Vec3d> tq;
            tq.resize(n);
            double L = 0;
            for (size_t j = 0; j < n; j++) {
                const RcsNozzle& z = v.rcsNoz[j];
                tq[j] = v.invI * (cross(z.pos - v.mp.com, z.dir) * (z.F * rcsThrustScale(v)));
                L += len2(tq[j]);
            }
            Vec3d aWant = v.invI * tauWant;
            double al = len(aWant);
            for (int it = 0; it < 24 && L > 0; it++) {
                Vec3d got{};
                for (size_t j = 0; j < n; j++) got += tq[j] * std::clamp(u[j], 0.0, 1.0);
                Vec3d r = aWant - got;
                if (len(r) < 0.03 * al) break;
                for (size_t j = 0; j < n; j++) u[j] = std::clamp(u[j] + 1.5 * dot(tq[j], r) / L, 0.0, 1.0);
            }
        }
    }
    double fl = len(fWant);
    if (fl > 1e-6) {
        Vec3d fh = fWant / fl;
        for (size_t j = 0; j < n; j++) {
            double e = dot(v.rcsNoz[j].dir, fh);
            if (e > 0.3) u[j] += e * std::min(1.0, fl);
        }
    }
    Vec3d tau, F;
    double use = 0, power = 0;
    const double ks = rcsThrustScale(v);
    for (size_t j = 0; j < n; j++) {
        double uj = std::clamp(u[j], 0.0, 1.0);
        if (uj <= 0) continue;
        const RcsNozzle& z = v.rcsNoz[j];
        double Fj = z.F * ks * uj;
        Vec3d f = z.dir * Fj;
        F += f;
        tau += cross(z.pos - v.mp.com, f);
        if (v.rcsElectric) power += Fj * RCS_ELEC_ISP * G0 / (2 * RCS_ELEC_EFF); // jet power / efficiency
        else use += Fj / (z.isp * G0);
        v.rcsFire[j] += (float)(uj * hs);
    }
    if (v.rcsElectric && v.rcsElecJ + power * hs > v.rcsChargeAvail) {
        // the battery runs dry within this sub-step: only the part it can still supply
        double k = std::max(0.0, (v.rcsChargeAvail - v.rcsElecJ) / (power * hs));
        tau *= k;
        F *= k;
        power *= k;
    }
    v.rcsElecJ += power * hs;
    v.rcsUse += use * hs;
    v.rcsImpulse += F * hs;
    return tau;
}

static void acsEnd(World& W, Vessel& v) {
    if (v.acsUse <= 0 || W.settings.infiniteFuel) return;
    stageTankConsume(v, v.acsProp, v.acsUse);
}

static void rcsEnd(World& W, Vessel& v, double h) {
    acsEnd(W, v);
    v.rcsFireTime += h;
    if (h > 0)
        for (auto& f : v.rcsFire) f = std::min(1.f, (float)(f / h));
    v.tRcsPowerW = h > 0 ? v.rcsElecJ / h : 0;
    if (v.rcsElecJ > 0 && !W.settings.infiniteFuel) v.charge = std::max(0.0, v.charge - v.rcsElecJ);
    if (W.settings.infiniteFuel || v.rcsUse <= 0) return;
    // draw from every monopropellant tank in proportion to its contents
    double have = monopropAmount(v);
    if (have <= 0) return;
    double k = std::min(1.0, v.rcsUse / have);
    for (size_t i = 0; i < v.comps.size(); i++)
        if (v.comps[i].func == Func::Tank && v.comps[i].prop == Prop::Monoprop) v.prop[i] *= 1 - k;
    v.monoAmt = have * (1 - k);
}

static bool rcsTranslating(const World& W, const Vessel& v) {
    if (!v.rcs || v.rcsNoz.empty() || W.active < 0 || W.active >= (int)W.vessels.size() || &W.vessels[W.active] != &v) return false;
    return std::fabs(W.input.tx) + std::fabs(W.input.ty) + std::fabs(W.input.tz) > 0.01;
}

// an engine fires at its own group, and engines of a core stage also while side boosters (an earlier
// group) are still attached
static bool engineFires(const Vessel& v, const Component& c) {
    if (c.cargo && v.bayComp >= 0) return false; // (a payload's motor stays cold in the bay)
    return c.stage == v.stage || (c.fireFrom >= 0 && c.fireFrom <= v.stage && v.stage < c.stage);
}

bool engineFiresNow(const Vessel& v, const Component& c) { return engineFires(v, c); }

std::vector<int> vesselStages(const Vessel& v) {
    std::vector<int> s;
    for (auto& c : v.comps)
        if (std::find(s.begin(), s.end(), c.stage) == s.end()) s.push_back(c.stage);
    std::sort(s.begin(), s.end());
    return s;
}

std::string stageEngineSummary(const Vessel& v, int stage) {
    std::vector<std::pair<std::string, int>> names;
    for (size_t i = 0; i < v.comps.size(); i++) {
        const Component& c = v.comps[i];
        if (c.stage != stage || c.func != Func::Engine || c.def < 0) continue;
        std::string nm = tr(CATALOG.defs[c.def].name);
        int cnt = CATALOG.defs[c.def].kind == PK::Engine ? std::max(1, c.cluster) : 1;
        bool found = false;
        for (auto& p : names)
            if (p.first == nm) { p.second += cnt; found = true; }
        if (!found) names.push_back({nm, cnt});
    }
    std::string s;
    for (auto& p : names) {
        if (!s.empty()) s += ", ";
        s += p.first;
        if (p.second > 1) s += " ×" + std::to_string(p.second);
    }
    return s;
}

namespace {
bool groupHasEngine(const Vessel& v, int g);
} // namespace

void stageResources(const Vessel& v, double amt[PROP_N], double cap[PROP_N], double& srbAmt, double& srbCap, double& Fvac, double& mdot) {
    for (int p = 0; p < PROP_N; p++) amt[p] = cap[p] = 0;
    srbAmt = srbCap = Fvac = mdot = 0;
    // tanks of the active group and of every group with an engine firing now
    std::vector<int> groups{v.stage};
    for (const Component& c : v.comps)
        if (c.func == Func::Engine && engineFires(v, c)) {
            if (std::find(groups.begin(), groups.end(), c.stage) == groups.end()) groups.push_back(c.stage);
            for (int g = v.stage + 1; g < c.stage; g++) // drop tanks it draws from
                if (!groupHasEngine(v, g) && std::find(groups.begin(), groups.end(), g) == groups.end()) groups.push_back(g);
        }
    for (size_t i = 0; i < v.comps.size(); i++) {
        const Component& c = v.comps[i];
        bool inGroup = std::find(groups.begin(), groups.end(), c.stage) != groups.end();
        if (!inGroup) continue;
        const CompPhys& p = v.phys[i];
        if (c.func == Func::Tank) { amt[(int)c.prop] += v.prop[i]; cap[(int)c.prop] += p.propCap; }
        if (c.func == Func::Engine && p.thrustVac > 0 && engineFires(v, c) && enginesOn(c) > 0) {
            double share = engineShare(c); // (nozzles the pilot switched off do not count)
            Fvac += p.thrustVac * share;
            mdot += p.thrustVac * share / (p.ispVac * G0);
            if (p.propCap > 0) { srbAmt += v.prop[i]; srbCap += p.propCap; }
        }
    }
}

// ------------------------------------------------------------------ physics internals
namespace {

struct EngineOut {
    Vec3d Fbody;          // total thrust (craft axes) [N]
    Vec3d Tbody;          // thrust torque about the CoM (craft axes) [N m]
    double F = 0;
    Vec3d gimbalAuth;     // control torque available from gimbals, per axis [N m]
    bool any = false, flameout = false;
    double electricW = 0;
};

struct JetRun {
    double F = 0, mdot = 0;
    Prop prop = Prop::None;
    int mode = 0;
};
JetRun jetRun(const PartDef& d, double rho, double M, double q, bool oxygen, double setting, double throttle, double patm, bool haveLH2);
double feedAmount(const Vessel& v, const Component& c, Prop p);
void feedConsume(Vessel& v, const Component& c, Prop p, double amount);

double groupTankAmount(const Vessel& v, int group, Prop p) {
    double s = 0;
    for (size_t i = 0; i < v.comps.size(); i++)
        if (v.comps[i].stage == group && v.comps[i].func == Func::Tank && v.comps[i].prop == p && !(v.comps[i].cargo && v.bayComp >= 0)) s += v.prop[i];
    return s;
}

void groupTankConsume(Vessel& v, int group, Prop p, double amount) {
    double tot = groupTankAmount(v, group, p);
    if (tot <= 0) return;
    double f = std::min(1.0, amount / tot);
    for (size_t i = 0; i < v.comps.size(); i++)
        if (v.comps[i].stage == group && v.comps[i].func == Func::Tank && v.comps[i].prop == p && !(v.comps[i].cargo && v.bayComp >= 0))
            v.prop[i] = std::max(0.0, v.prop[i] * (1 - f));
}
double stageTankAmount(const Vessel& v, Prop p) { return groupTankAmount(v, v.stage, p); }
void stageTankConsume(Vessel& v, Prop p, double amount) { groupTankConsume(v, v.stage, p, amount); }

} // namespace

// Rotor / propeller thrust from momentum theory: shaft power P [W], air density, disc area A, axial
// speed vc (> 0: moving along the thrust). Solves P * FM = T (vc / 2 + sqrt(vc^2 / 4 + T / (2 rho A))),
// limited by the blade loading the tips can carry.
double rotorThrust(double P, double rho, double A, double vc, double tipT) {
    if (P <= 0 || rho <= 0 || A <= 0) return 0;
    const double FM = 0.72; // figure of merit of a good rotor
    double Pe = P * FM;
    vc = vc > 0 ? vc : vc * 0.3; // descending into the own wake gains little (vortex ring)
    auto need = [&](double T) { return T * (0.5 * vc + std::sqrt(std::max(0.0, 0.25 * vc * vc + T / (2 * rho * A)))); };
    double lo = 0, hi = std::max(1.0, std::cbrt(Pe * Pe * 2 * rho * A) * 2);
    for (int k = 0; k < 60 && need(hi) < Pe; k++) hi *= 2;
    for (int k = 0; k < 40; k++) {
        double m = 0.5 * (lo + hi);
        (need(m) < Pe ? lo : hi) = m;
    }
    return std::min(lo, tipT);
}
// turbofan: static thrust scaled by the density (lapse ~ rho^0.7) and a ram-effect curve over Mach
double jetThrust(double Tsl, double rho, double M) {
    if (rho <= 0 || M > 2.4) return 0;
    double ram = std::clamp(1.0 - 0.25 * M + 0.35 * M * M, 0.0, 1.35) * (M > 1.8 ? std::max(0.0, (2.4 - M) / 0.6) : 1.0);
    return Tsl * std::pow(rho / 1.225, 0.7) * ram;
}

namespace {
double smooth01(double a, double b, double x) {
    double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3 - 2 * t);
}

// one air-breathing engine at the present flight state: thrust [N], propellant flow [kg/s] and its kind, mode
// (1 dry, 2 afterburner, 3 rocket). setting: the spooled-up power (0..1), throttle: the lever (fuel control
// of ramjets / rockets acts at once). haveLH2: liquid hydrogen within reach (combined cycle, air mode)
JetRun jetRun(const PartDef& d, double rho, double M, double q, bool oxygen, double setting, double throttle, double patm, bool haveLH2) {
    JetRun o;
    bool air = oxygen && rho > 0;
    switch (d.cycle) {
    case JetCycle::Turbofan:
        if (!air) return o;
        o.F = jetThrust(d.thrustVac, rho, M) * setting;
        o.mdot = o.F / (std::max(d.ispVac, 1.0) * G0);
        o.prop = d.prop;
        o.mode = o.F > 0 ? 1 : 0;
        return o;
    case JetCycle::Afterburner:
    case JetCycle::Turboramjet: {
        if (!air || M > d.machMax + 0.3) return o;
        // ram rise over Mach and the inlet's limit; the afterburner adds the rest above 85 % power
        double ram = std::max(0.0, 1.0 - 0.15 * M + d.ramK * M * M) * (1.0 - smooth01(d.machMax - 0.2, d.machMax + 0.3, M));
        double k = std::pow(rho / 1.225, d.lapse) * ram;
        double dry = d.dryThrust * k * std::min(setting, 0.85) / 0.85;
        double wet = setting > 0.85 ? (d.thrustVac - d.dryThrust) * k * (setting - 0.85) / 0.15 : 0.0;
        o.F = dry + wet;
        o.mdot = dry / (std::max(d.ispVac, 1.0) * G0) + wet / (std::max(d.ispWet, 1.0) * G0);
        o.prop = d.prop;
        o.mode = wet > 0 ? 2 : (o.F > 0 ? 1 : 0);
        return o;
    }
    case JetCycle::Scramjet: {
        if (!air || M < 4.0) return o; // the inlet does not start below Mach 4
        double f = smooth01(4.0, 5.5, M) * (1.0 - smooth01(8.5, 10.5, M));
        o.F = d.thrustVac * std::min(q / 50e3, 2.0) * f * throttle;
        o.mdot = o.F / (std::max(d.ispVac, 1.0) * G0);
        o.prop = d.prop;
        o.mode = o.F > 0 ? 1 : 0;
        return o;
    }
    case JetCycle::CombinedCycle: {
        // air-breathing while there is enough air below the inlet's limit, on liquid hydrogen
        if (air && haveLH2 && M < d.machMax && rho >= 0.012) {
            double ram = std::max(0.0, 1.0 - 0.1 * M + d.ramK * M * M);
            o.F = d.thrustVac * std::pow(rho / 1.225, d.lapse) * ram * setting;
            o.mdot = o.F / (std::max(d.ispVac, 1.0) * G0);
            o.prop = d.prop;
            o.mode = 1;
            return o;
        }
        // closed cycle: a rocket on board oxidiser
        double thr = throttle > 0 ? std::max(throttle, d.minThrottle) : 0.0;
        o.mdot = d.rocketThrust * thr / (std::max(d.rocketIspVac, 1.0) * G0);
        double isp = std::max(0.0, d.rocketIspVac - (d.rocketIspVac - d.rocketIspSL) * std::clamp(patm, 0.0, 1.0));
        o.F = o.mdot * isp * G0;
        o.prop = d.rocketProp;
        o.mode = 3;
        return o;
    }
    }
    return o;
}

// the tanks an engine draws from: drop tanks first - groups between the vessel's active group and the engine's
// own that have no engines of their own (the external tank of a winged orbiter) - then its own group
bool groupHasEngine(const Vessel& v, int g) {
    for (const Component& c : v.comps)
        if (c.stage == g && (c.func == Func::Engine || c.func == Func::Jet || c.func == Func::Rotor)) return true;
    return false;
}
double feedAmount(const Vessel& v, const Component& c, Prop p) {
    double s = groupTankAmount(v, c.stage, p);
    for (int g = v.stage; g < c.stage; g++)
        if (!groupHasEngine(v, g)) s += groupTankAmount(v, g, p);
    return s;
}
void feedConsume(Vessel& v, const Component& c, Prop p, double amount) {
    for (int g = v.stage; g < c.stage && amount > 0; g++) {
        if (groupHasEngine(v, g)) continue;
        double have = groupTankAmount(v, g, p);
        double take = std::min(have, amount);
        if (take > 0) groupTankConsume(v, g, p, take);
        amount -= take;
    }
    if (amount > 0) groupTankConsume(v, c.stage, p, amount);
}
} // namespace

double jetPerformance(const PartDef& d, double rho, double mach, double q, double patm, double& isp, int& mode) {
    JetRun jr = jetRun(d, rho, mach, q, true, 1.0, 1.0, patm, true);
    isp = jr.mdot > 0 ? jr.F / (jr.mdot * G0) : 0.0;
    mode = jr.mode;
    return jr.F;
}

namespace {
EngineOut runEngines(World& W, Vessel& v, double pressure, double dt, double powerFactor) {
    EngineOut o;
    if (v.engLevel.size() != v.comps.size()) v.engLevel.assign(v.comps.size(), 0.f);
    std::fill(v.engLevel.begin(), v.engLevel.end(), 0.f);
    if (v.jetMode.size() != v.comps.size()) v.jetMode.assign(v.comps.size(), 0);
    std::fill(v.jetMode.begin(), v.jetMode.end(), 0);
    if (!v.ignited) return o;
    bool inf = W.settings.infiniteFuel;
    double patm = pressure / P_ATM;
    // the air at the vessel (rotors and jets)
    const Body& bd = W.U.bodies[v.ref];
    double altA = len(v.r) - bd.radius;
    double rho = bd.atmTop > 0 && altA < bd.atmTop ? W.U.density(v.ref, altA) : 0.0;
    Vec3d vAirB = v.q.conj().rotate(v.v - cross(W.U.angVel(v.ref), v.r)); // craft-frame velocity through the air
    double mach = len(vAirB) / std::max(bd.soundSpeed, 1.0);
    bool oxygen = bd.name == "Earth";
    int ccMode = 0;
    // a turbofan spools up in ~4 s from idle to full thrust and down in ~2.5 s: the thrust follows the
    // throttle with that lag
    if (dt > 0) v.spool += std::clamp(v.throttle - v.spool, -0.4 * dt, 0.25 * dt);
    for (size_t i = 0; i < v.comps.size(); i++) {
        const Component& c = v.comps[i];
        const CompPhys& p = v.phys[i];
        if ((c.func == Func::Rotor || c.func == Func::Jet) && c.def >= 0 && engineFires(v, c) && v.throttle > 0) {
            if (enginesOn(c) <= 0) continue; // switched off by the pilot
            const PartDef& d = CATALOG.defs[c.def];
            double F = 0;
            if (c.func == Func::Rotor) {
                double P = p.powerUse * v.throttle * powerFactor;
                o.electricW += p.powerUse * v.throttle;
                double A = PI * 0.25 * d.diam * d.diam;
                double vtip = std::min(230.0, 0.8 * bd.soundSpeed);
                double tipT = 0.016 * rho * A * vtip * vtip * (d.canopies > 1 ? 1.6 : 1.0);
                F = rotorThrust(P * 0.92, rho, A * (d.canopies > 1 ? 1.25 : 1.0), dot(vAirB, p.thrustDir), tipT); // 92 % motor
                // cyclic (rotors) / slipstream over the tail (propellers): control torque
                double Rr = 0.5 * d.diam;
                Vec3d lever = p.thrustPoint - v.mp.com;
                o.gimbalAuth += Vec3d{0.12 * F * Rr + 0.1 * F * std::fabs(lever.z), 0.12 * F * Rr + 0.1 * F * std::fabs(lever.z), 0.05 * F * Rr};
            } else {
                // air-breathing engines (and the closed mode of a combined-cycle engine)
                double qd = 0.5 * rho * dot(vAirB, vAirB);
                bool lh2 = inf || feedAmount(v, c, Prop::LH2) > 0;
                JetRun jr = jetRun(d, rho, mach, qd, oxygen, std::clamp(v.spool, 0.0, 1.0), v.throttle, patm, lh2);
                if (jr.mode == 0 || jr.F <= 0) continue; // no oxygen in the air / out of its speed range
                F = jr.F;
                if (!inf) {
                    double have = feedAmount(v, c, jr.prop);
                    if (have <= 0) { o.flameout = true; continue; }
                    double use = jr.mdot * dt;
                    if (dt > 0) {
                        if (use > have) F *= have / use;
                        feedConsume(v, c, jr.prop, std::min(use, have));
                    }
                }
                v.jetMode[i] = (uint8_t)jr.mode;
                if (d.cycle == JetCycle::CombinedCycle) ccMode = jr.mode;
            }
            if (F <= 0) continue;
            v.engLevel[i] = (float)std::clamp(v.throttle, 0.05, 1.0);
            Vec3d f = p.thrustDir * F;
            o.Fbody += f;
            o.Tbody += cross(p.thrustPoint - v.mp.com, f);
            o.F += F;
            o.any = true;
            continue;
        }
        if (!engineFires(v, c) || c.func != Func::Engine || p.thrustVac <= 0 || c.def < 0) continue;
        // the pilot may run only some engines (nozzles of a cluster): thrust and flow scale with their share,
        // the thrust acts at the centre of the nozzles still running
        double share = engineShare(c);
        if (share <= 0) continue;
        const PartDef& d = CATALOG.defs[c.def];
        double isp = std::max(0.0, p.ispVac - (p.ispVac - p.ispSL) * patm);
        double mdot, frac = 1;
        if (d.kind == PK::SRB) {
            if (v.prop[i] <= 0 && !inf) continue;
            mdot = p.thrustVac / (p.ispVac * G0) * share;
            double use = mdot * dt;
            if (!inf && dt > 0) {
                if (use > v.prop[i]) frac = v.prop[i] / use;
                v.prop[i] = std::max(0.0, v.prop[i] - use);
            }
        } else {
            if (v.throttle <= 0) continue;
            double thr = std::max(v.throttle, p.minThrottle);
            double pf = p.powerUse > 0 ? powerFactor : 1.0;
            mdot = p.thrustVac / (p.ispVac * G0) * thr * pf * share;
            if (p.powerUse > 0) o.electricW += p.powerUse * thr * share;
            if (!inf) {
                double have = feedAmount(v, c, d.prop); // its own stage's tanks (after any drop tanks)
                if (have <= 0) { o.flameout = true; continue; }
                double use = mdot * dt;
                if (dt > 0) {
                    if (use > have) frac = have / use;
                    feedConsume(v, c, d.prop, std::min(use, have));
                }
            }
        }
        double F = mdot * isp * G0 * frac;
        if (F <= 0) continue;
        v.engLevel[i] = (float)std::min(1.0, F / std::max(p.thrustVac * share, 1e-9)); // (level of the nozzles running)
        Vec3d f = p.thrustDir * F;
        Vec3d lever = p.thrustPoint + engineThrustOffset(c) - v.mp.com;
        o.Fbody += f;
        o.Tbody += cross(lever, f);
        o.F += F;
        double gs = std::sin(p.gimbalDeg * DEG) * F;
        o.gimbalAuth += Vec3d{gs * std::fabs(lever.z), gs * std::fabs(lever.z), gs * std::sqrt(lever.x * lever.x + lever.y * lever.y)};
        o.any = true;
    }
    // a combined-cycle engine changed its mode: say so (the active vessel)
    if (ccMode != 0 && ccMode != v.ccMode && dt > 0) {
        if (v.ccMode != 0 && &v == W.activeVessel())
            W.toast(ccMode == 3 ? tr("Combined-cycle engines: rocket mode (on-board oxygen)") : tr("Combined-cycle engines: air-breathing mode"),
                    {255, 190, 110, 255});
        v.ccMode = ccMode;
    }
    return o;
}

bool thrustPossible(const World& W, const Vessel& v) {
    if (v.sailDeployed && v.sailArea > 0) return true;
    if (roverDrive(W, v) != 0) return true;
    if (rcsTranslating(W, v)) return true;
    if (!v.ignited) return false;
    for (size_t i = 0; i < v.comps.size(); i++) {
        const Component& c = v.comps[i];
        if ((c.func == Func::Rotor || c.func == Func::Jet) && engineFires(v, c) && v.throttle > 0 && enginesOn(c) > 0) {
            if (c.func == Func::Rotor) return true;
            const Body& b = W.U.bodies[v.ref];
            bool air = b.name == "Earth" && b.atmTop > 0 && len(v.r) - b.radius < b.atmTop;
            if (air || (c.def >= 0 && worksWithoutAir(CATALOG.defs[c.def]))) return true;
            continue;
        }
        if (!engineFires(v, c) || c.func != Func::Engine || v.phys[i].thrustVac <= 0 || enginesOn(c) <= 0) continue;
        if (v.phys[i].propCap > 0 && (v.prop[i] > 0 || W.settings.infiniteFuel)) return true; // solid motor burning
        if (v.phys[i].propCap <= 0 && v.throttle > 0) return true;
    }
    return false;
}

Vec3d gravity(const World& W, const std::vector<BodyState>& S, int ref, const Vec3d& r) {
    const Universe& U = W.U;
    Vec3d a = -S[ref].acc;
    for (int j : U.relevant[ref]) {
        Vec3d d = (j == ref) ? r : r - U.relPos(S, j, ref);
        double d2 = len2(d);
        if (d2 < 1) continue;
        a -= d * (U.bodies[j].GM / (d2 * std::sqrt(d2)));
    }
    return a;
}

void pinLanded(World& W, Vessel& v, double t) {
    Mat3d R = W.U.bodyRot(v.ref, t);
    v.r = R * v.landPos;
    v.v = cross(W.U.angVel(v.ref), v.r);
    v.q = (Quatd::fromMat(R) * v.landQ).normalized();
    v.w = {};
}

double machFactor(double M) {
    if (M < 0.8) return 0.55;
    if (M < 1.2) return lerpd(0.55, 1.0, (M - 0.8) / 0.4);
    if (M < 5.0) return lerpd(1.0, 0.92, (M - 1.2) / 3.8);
    return 0.92;
}

struct AeroOut {
    Vec3d Fbody, Tbody;    // force and torque about the CoM (craft axes)
    double q = 0, mach = 0, rho = 0, speed = 0, aproj = 0, heatFlux = 0, aoa = 0;
    Vec3d flowBody;        // air-flow direction (craft axes)
};

// Aerodynamic force/torque from the precomputed table, thin-airfoil wings, rate damping and parachutes.
AeroOut aerodynamics(const World& W, const Vessel& v, const Vec3d& r, const Vec3d& vel, const Quatd& q, const Vec3d& w,
                     std::vector<float>* exposure) {
    AeroOut o;
    const Universe& U = W.U;
    const Body& b = U.bodies[v.ref];
    if (b.atmTop <= 0 || !v.aero) return o;
    double alt = len(r) - b.radius;
    double rho = U.density(v.ref, alt);
    if (rho <= 0) return o;
    Vec3d va = vel - cross(U.angVel(v.ref), r); // craft velocity relative to the air
    double sp = len(va);
    if (sp < 1e-3) return o;
    Vec3d uW = -va / sp;
    Vec3d u = q.conj().rotate(uW);
    double qd = 0.5 * rho * sp * sp;
    double M = sp / std::max(50.0, b.soundSpeed);
    double kM = machFactor(M);
    auto s = v.aero->lookup(u, exposure);
    o.Fbody = (s.F_p * kM + s.F_f) * qd;
    Vec3d To = (s.T_p * kM + s.T_f) * qd;
    o.Tbody = To - cross(v.mp.com, o.Fbody);
    o.aproj = s.aproj;
    // lifting surfaces see the local flow including the rotation of the craft
    Vec3d vaB = q.conj().rotate(va);
    for (size_t i = 0; i < v.comps.size(); i++) {
        if (v.comps[i].func != Func::Wing) continue;
        const CompPhys& p = v.phys[i];
        Vec3d rel = p.centre - v.mp.com;
        // the deflected surface: its incidence changes by how much it can turn the vessel about the commanded
        // axes (elevators pitch, ailerons roll, rudders yaw, elevons both, rocket fins all three); the extra lift
        // of the deflection acts at the flap
        double dA = 0;
        Vec3d relF = surfPoint(v.comps[i], p) - v.mp.com;
        Vec3d vLoc = vaB + cross(w, rel);
        if (len2(v.surf) > 1e-8) {
            dA = surfDeflect(cross(relF, p.axisZ), v.surf, v.tailless);
            // a flap on a wing at a high angle of attack in hypersonic flow (an orbiter's entry) is not a
            // thin-airfoil flap: turned into the flow it takes the full impact pressure (Newtonian, Cp = 2 sin^2 of
            // its own angle), ~1.45 x what the low-speed flap effectiveness gives
            double vl = len(vLoc);
            if (vl > 1e-3 && M > 2) {
                double inc = std::asin(clampd(std::fabs(dot(vLoc / vl, p.axisZ)), 0.0, 1.0));
                double stalled = clampd((inc - 15 * DEG) / (10 * DEG), 0.0, 1.0), hyper = clampd((M - 2) / 3, 0.0, 1.0);
                dA *= 1 + 0.45 * stalled * hyper;
            }
        }
        Vec3d F = wingForce(v.comps[i], p, vLoc, rho, M, 0.0);
        o.Fbody += F;
        o.Tbody += cross(rel, F);
        // flaps: down toward the craft's belly (more lift on a wing whose +Z is the craft's top); their lift
        // acts at the flap (aft, inboard): the nose-down moment of the flaps
        double fl = 0;
        const Component& wc = v.comps[i];
        if (wc.flap > 0 && wc.shape == Shape::Airfoil && v.flapPos > 1e-6 && std::fabs(p.axisZ.y) > 0.5)
            fl = (p.axisZ.y > 0 ? 1.0 : -1.0) * flapAngleAt(v.flapPos, wc.flap) * DEG;
        if (fl != 0) {
            Vec3d Ffl = wingForce(wc, p, vLoc, rho, M, 0.0, fl) - F;
            Vec3d fp = p.centre + wc.rot().rotate(Vec3d{airfoilRoot(wc) * (0.5 - FLAP_SPAN / 2) * wc.dims.x, (0.5 - FLAP_CHORD / 2) * wc.dims.y, 0});
            o.Fbody += Ffl;
            o.Tbody += cross(fp - v.mp.com, Ffl);
            F += Ffl;
        }
        if (dA != 0) {
            Vec3d dF = wingForce(v.comps[i], p, vLoc, rho, M, dA, fl) - F;
            o.Fbody += dF;
            o.Tbody += cross(relF, dF);
        }
    }
    // parachutes pull along the flow at their attachment point; the canopy inflates over time
    if (v.chuteDeployed && !v.chuteCut && v.chuteOpen > 0) {
        double open2 = v.chuteOpen * v.chuteOpen * (v.chuteT < v.chuteReefT ? v.chuteReefK : 1.0); // (reefed at first)
        for (size_t i = 0; i < v.comps.size(); i++) {
            if (v.comps[i].func != Func::Parachute) continue;
            Vec3d F = u * (qd * v.phys[i].chuteCdA * open2);
            o.Fbody += F;
            o.Tbody += cross(v.phys[i].centre - v.mp.com, F);
        }
    }
    // rotational damping of the body (moving surfaces see a changed local flow)
    double L = v.aero->length;
    double kd = 0.5 * rho * sp * std::max(s.aproj, 0.1) * L * L / 8.0;
    o.Tbody -= Vec3d{w.x * kd, w.y * kd, w.z * kd * 0.1};
    o.q = qd;
    o.mach = M;
    o.rho = rho;
    o.speed = sp;
    o.flowBody = u;
    o.aoa = std::acos(clampd(dot(Vec3d{0, 0, 1}, -u), -1, 1));
    // effective nose radius for the stagnation heating: a bluff body's size, but no more than a few metres (a
    // winged orbiter's nose and leading edges heat as small radii, however large its belly)
    // (a capsule's blunt shield: up to 4 m; a winged orbiter: its nose cap and wing leading edges, ~1 m - the
    // Space Shuttle's nose cap radius is ~0.7 m)
    double rn = clampd(std::sqrt(std::max(s.aproj, 0.01) / PI), 0.3, v.lifting ? 1.0 : 4.0);
    o.heatFlux = 1.7415e-4 * std::sqrt(rho / rn) * sp * sp * sp;
    return o;
}

// solar sails: radiation pressure on each sail membrane (normal = component +Z)
void sailForces(World& W, const Vessel& v, const std::vector<BodyState>& S, const Vec3d& r, const Quatd& q, Vec3d& Fworld, Vec3d& Tbody,
                double& mag) {
    Fworld = {};
    Tbody = {};
    mag = 0;
    if (!v.sailDeployed || v.sailArea <= 0) return;
    std::vector<std::pair<Vec3d, double>> each;
    starFlux(W, S, v.ref, r, nullptr, nullptr, &each);
    for (size_t i = 0; i < v.comps.size(); i++) {
        if (v.comps[i].func != Func::Sail) continue;
        Vec3d nW = q.rotate(v.phys[i].axisZ);
        Vec3d F{};
        for (auto& e : each) {
            if (e.second <= 0) continue;
            Vec3d away = -e.first;
            double c = dot(nW, away);
            Vec3d n = c >= 0 ? nW : -nW;
            F += n * (2.0 * e.second / C_LIGHT * v.phys[i].sailArea * c * c);
        }
        Fworld += F;
        Tbody += cross(v.phys[i].centre - v.mp.com, q.conj().rotate(F));
    }
    mag = len(Fworld);
}
} // namespace

// (diagnostics for the tests: the aerodynamic torque on a vessel as it flies now)
Vec3d debugAeroTorque(const World& W, const Vessel& v) { return aerodynamics(W, v, v.r, v.v, v.q, v.w, nullptr).Tbody; }

// ------------------------------------------------------------------ attitude dynamics
// One rigid-body rotation step: Euler's equations with control (wheels, gimbals) and external torque.
// ---- the roll kept by the pointing SAS modes (prograde, retrograde, normal, target, ...): a mode turns the nose and
// holds the roll it found relative to the local vertical, not only the nose's direction (a shortest turn of the nose
// alone flips an orbiter from belly-down to belly-up when it turns from retrograde to prograde)
static bool sasKeepsRoll(SAS m) { return m != SAS::Off && m != SAS::Stability && m != SAS::Ascent && m != SAS::Dock && m != SAS::COUNT; }

// the roll angle of the craft's +Y about the nose from the vertical (false: the nose within ~10 degrees of it,
// where a roll about the vertical has no meaning)
static bool rollFromVertical(const Vec3d& nose, const Vec3d& upB, const Vec3d& vert, double& phi) {
    Vec3d rp = vert - nose * dot(vert, nose);
    if (len(rp) < 0.17) return false;
    rp = norm(rp);
    phi = std::atan2(dot(cross(rp, upB), nose), dot(rp, upB));
    return true;
}

// the roll reference: taken anew when the mode changes, while the pilot rolls, or once it can be (the nose
// off the vertical)
static void sasRollTrack(Vessel& v, bool manualRoll) {
    if (v.sas != v.sasRollMode) {
        v.sasRollMode = v.sas;
        v.sasRollSet = false;
    }
    if (!sasKeepsRoll(v.sas) || (v.sasRollSet && !manualRoll) || len2(v.r) < 1) return;
    double phi;
    if (rollFromVertical(v.q.rotate({0, 0, 1}), v.q.rotate({0, 1, 0}), norm(v.r), phi)) {
        v.sasRoll = phi;
        v.sasRollSet = true;
    }
}

// the craft's +Y wanted with the nose along t (unit), and how much it counts 0..1 (it fades out as t nears the
// vertical)
static Vec3d sasWantedUp(const Vessel& v, const Vec3d& t, double& wgt) {
    wgt = 0;
    if (!v.sasRollSet || !sasKeepsRoll(v.sas) || len2(v.r) < 1) return {};
    Vec3d vert = norm(v.r);
    Vec3d rt = vert - t * dot(vert, t);
    double s = len(rt);
    wgt = clampd((s - 0.08) / 0.15, 0.0, 1.0);
    if (s < 1e-6) return {};
    rt = rt / s;
    return rt * std::cos(v.sasRoll) + cross(t, rt) * std::sin(v.sasRoll);
}

// the attitude set straight away at a high time warp (on rails): the nose on the target, the roll kept
static Quatd sasRailAttitude(Vessel& v, const Vec3d& target) {
    sasRollTrack(v, false);
    Vec3d t = norm(target);
    double wgt;
    Vec3d upW = sasWantedUp(v, t, wgt);
    return Quatd::lookRot(t, wgt > 0.5 ? upW : v.q.rotate({0, 1, 0}));
}

static void rotStep(World& W, Vessel& v, double h, const Vec3d& tauExt, const Vec3d& gimbalAuth, bool isActive, bool control) {
    const Mat3d& I = v.mp.inertia;
    Vec3d Iw = I * v.w;
    Vec3d gyro = cross(v.w, Iw);
    // direct torque: reaction wheels (an aircraft has none) and engine gimbals / rotor cyclic
    double Tw = control && !v.isAircraft ? v.torque : 0;
    Vec3d auth{Tw + (control ? gimbalAuth.x : 0), Tw + (control ? gimbalAuth.y : 0), Tw + (control ? gimbalAuth.z : 0)};
    // control surfaces: their torque at full deflection now, and what they give at their present position
    Vec3d surfQ = control ? Vec3d{v.ctrlAuth.x * v.ctrlEff.x, v.ctrlAuth.y * v.ctrlEff.y, v.ctrlAuth.z * v.ctrlEff.z} * v.tDynP : Vec3d{};
    Vec3d surfNow{surfQ.x * v.surf.x, surfQ.y * v.surf.y, surfQ.z * v.surf.z};
    Vec3d acsA = control ? acsAuthority(v) : Vec3d{};
    Vec3d authAll = auth + surfQ + acsA; // what the SAS can count on
    if (control && rcsReady(W, v)) authAll += Vec3d{1, 1, 1} * (v.rcsAuth * rcsThrustScale(v));
    ControlInput in = isActive ? W.ctrl : ControlInput{};
    // a rotorcraft stands upright (+Z up): its heading turns about the craft's Z axis, so the yaw keys act
    // there and the roll keys tilt it sideways
    bool rotorcraft = v.isAircraft && v.rotorcraft;
    if (v.drivePower > 0 && (v.landed || v.rolling)) in.pitch = in.yaw = in.roll = 0; // a rover: W/S drive, A/D steer
    const double inA[3] = {in.pitch, rotorcraft ? in.roll : in.yaw, rotorcraft ? in.yaw : in.roll};
    bool man[3] = {std::fabs(inA[0]) > 0.01, std::fabs(inA[1]) > 0.01, std::fabs(inA[2]) > 0.01};
    bool manual = man[0] || man[1] || man[2];
    // an aeroplane: the pilot moves the control surfaces themselves (stick and pedals), nothing else turns it
    // (a rotorcraft flies on its rotor's cyclic; its fins only help)
    bool planeSurf = v.isAircraft && !v.rotorcraft && v.hasWheels && len2(v.ctrlAuth) > 0;
    double Ix = std::max(I.m[0][0], 1e-6), Iy = std::max(I.m[1][1], 1e-6), Iz = std::max(I.m[2][2], 1e-6);
    Vec3d amax{authAll.x / Ix, authAll.y / Iy, authAll.z / Iz};
    const double am[3] = {amax.x, amax.y, amax.z};
    bool sasOn = control && v.sas != SAS::Off;
    double wd[3] = {0, 0, 0};
    if (control) sasRollTrack(v, man[2]);
    if (sasOn && v.sas != SAS::Stability) {
        Vec3d target = W.targetDirection(v, v.sas);
        if (len(target) > 0.5) {
            Vec3d tl = v.q.conj().rotate(target);
            Vec3d axis{-tl.y, tl.x, 0};
            double sl = len(axis);
            double ang = std::atan2(sl, tl.z);
            double at = std::max(1e-6, std::min(amax.x, amax.y));
            Vec3d wdes{};
            if (sl > 1e-9) wdes = axis / sl * std::min(1.0, std::sqrt(2 * at * ang) * 0.75);
            else if (tl.z < 0) wdes = {std::min(1.0, std::sqrt(2 * at * PI) * 0.75), 0, 0};
            // with the roll kept: the turn from the attitude now to the wanted one (nose on the target, the
            // recorded roll from the vertical), its rate limited by the weakest axis it uses. (A craft that
            // can hardly roll keeps the nose-only turn.)
            double wgt = 0;
            Vec3d upW = sasWantedUp(v, norm(target), wgt);
            if (wgt > 0 && amax.z > 0.05 * std::min(amax.x, amax.y)) {
                Quatd qe = v.q.conj() * Quatd::lookRot(target, upW); // (body frame: now -> wanted)
                if (qe.w < 0) qe = {-qe.w, -qe.x, -qe.y, -qe.z};
                Vec3d ax{qe.x, qe.y, qe.z};
                double sh = len(ax);
                Vec3d wf{};
                if (sh > 1e-9) {
                    Vec3d k = ax / sh;
                    double angF = 2 * std::atan2(sh, qe.w);
                    double s = 1e30;
                    const double kk[3] = {k.x, k.y, k.z};
                    for (int a = 0; a < 3; a++)
                        if (std::fabs(kk[a]) > 1e-6) s = std::min(s, std::max(am[a], 1e-6) / std::fabs(kk[a]));
                    wf = k * std::min(1.0, std::sqrt(2 * s * angF) * 0.75);
                }
                wdes = wdes * (1 - wgt) + wf * wgt;
            }
            wd[0] = wdes.x;
            wd[1] = wdes.y;
            wd[2] = wdes.z;
        }
    }
    // which axes are flown: a pilot input asks for a turn rate on its axis (not for an aeroplane: its
    // surfaces follow the stick), the SAS takes the others
    bool drive[3] = {false, false, false};
    if (control)
        for (int a = 0; a < 3; a++) {
            if (man[a] && !planeSurf) {
                double cap = a == 2 ? RATE_ROLL : RATE_PY;
                cap = std::min(cap, std::sqrt(2 * std::max(am[a], 1e-9) * 6 * DEG)); // a weak vessel turns slower, so it can still stop
                wd[a] = inA[a] * cap;
                drive[a] = true;
            } else if (sasOn && !man[a]) drive[a] = true;
        }
    const double wv[3] = {v.w.x, v.w.y, v.w.z};
    const double Iv[3] = {Ix, Iy, Iz};
    Vec3d alphaDes{drive[0] ? (wd[0] - wv[0]) / 0.25 : 0, drive[1] ? (wd[1] - wv[1]) / 0.25 : 0, drive[2] ? (wd[2] - wv[2]) / 0.25 : 0};
    (void)Iv;
    // torque needed (beyond the external torque without the surfaces' share)
    // an angular acceleration beyond what the craft can give is scaled down as a whole (its direction kept):
    // otherwise the axis that can follow gets its full share of the cross-coupling torque (the products of
    // inertia) while the saturated one cannot, and the craft turns about the wrong axis (an orbiter rolls
    // as it yaws round)
    {
        Vec3d IA = I * alphaDes, rest = gyro - (tauExt - surfNow);
        Vec3d cap = auth + surfQ + acsA;
        if (control && rcsReady(W, v)) cap += v.rcsAuthAxis * rcsThrustScale(v);
        const double ia[3] = {IA.x, IA.y, IA.z}, rs[3] = {rest.x, rest.y, rest.z}, cp[3] = {cap.x, cap.y, cap.z};
        double f = 1;
        for (int a = 0; a < 3; a++) {
            if (!drive[a] || std::fabs(ia[a]) < 1e-12 || std::fabs(rs[a]) >= cp[a]) continue; // (saturated anyway)
            // |f ia + rs| <= cp
            double lim = ((ia[a] > 0 ? cp[a] : -cp[a]) - rs[a]) / ia[a];
            f = std::min(f, std::max(lim, 0.0));
        }
        if (f < 1) alphaDes = alphaDes * std::max(f, 0.02);
    }
    Vec3d needV = I * alphaDes + gyro - (tauExt - surfNow);
    if (isActive) {
        g_rotDbg[0] = tauExt;
        g_rotDbg[1] = I * alphaDes;
        g_rotDbg[2] = gyro;
        g_rotDbg[3] = surfQ;
        g_rotDbg[4] = surfNow;
        g_rotDbg[5] = Vec3d{Ix, Iy, Iz};
    }
    double need[3] = {drive[0] ? needV.x : 0, drive[1] ? needV.y : 0, drive[2] ? needV.z : 0};
    const double au[3] = {auth.x, auth.y, auth.z}, sq[3] = {surfQ.x, surfQ.y, surfQ.z};
    double tc[3] = {0, 0, 0}, sw[3] = {0, 0, 0};
    double cur[3] = {v.surf.x, v.surf.y, v.surf.z};
    for (int a = 0; a < 3; a++) {
        if (!control) continue;
        if (planeSurf && man[a]) sw[a] = std::clamp(inA[a], -1.0, 1.0);
        else if (drive[a]) {
            // the surfaces first where they bite (they cost nothing), the wheels / gimbals for the rest
            if (sq[a] > 1e-6) sw[a] = std::clamp(need[a] / sq[a], -1.0, 1.0);
            tc[a] = std::clamp(need[a] - sw[a] * sq[a], -au[a], au[a]);
        }
    }
    // the surface actuators move at a limited speed
    for (int a = 0; a < 3; a++) cur[a] += std::clamp(sw[a] - cur[a], -SURF_RATE * h, SURF_RATE * h);
    v.surf = {cur[0], cur[1], cur[2]};
    Vec3d tauC{tc[0], tc[1], tc[2]};
    Vec3d rem{need[0] - tc[0] - sw[0] * sq[0], need[1] - tc[1] - sw[1] * sq[1], need[2] - tc[2] - sw[2] * sq[2]};
    bool anyDrive = drive[0] || drive[1] || drive[2];
    if (control && rcsReady(W, v)) {
        Vec3d fWant = isActive ? Vec3d{in.tx, in.ty, in.tz} : Vec3d{};
        Vec3d tauWant{};
        if (anyDrive) {
            tauWant = rem;
            // deadband while holding: the wheels hold, RCS saves propellant
            if (!manual && len(tauWant) < 0.02 * v.rcsAuth * rcsThrustScale(v)) tauWant = {};
        }
        if (len(tauWant) > 0 || len(fWant) > 0) {
            Vec3d tr = rcsAllocate(v, tauWant, false, fWant, h);
            tauC += tr;
            rem -= tr;
        }
    }
    if (control && anyDrive && (acsA.x > 0 || acsA.z > 0)) {
        // attitude thrusters of the stage: what is still missing (deadband so a settled attitude does not keep
        // burning propellant)
        Vec3d want{clampd(rem.x, -acsA.x, acsA.x), clampd(rem.y, -acsA.y, acsA.y), clampd(rem.z, -acsA.z, acsA.z)};
        if (std::fabs(want.x) < 0.01 * acsA.x) want.x = 0;
        if (std::fabs(want.y) < 0.01 * acsA.y) want.y = 0;
        if (std::fabs(want.z) < 0.01 * acsA.z) want.z = 0;
        double lever = std::max(0.5, v.length * 0.45), roll = std::max(0.3, v.maxR);
        double force = std::fabs(want.x) / lever + std::fabs(want.y) / lever + std::fabs(want.z) / roll; // total thrust [N]
        if (force > 0) {
            tauC += want;
            v.acsUse += force / (v.acsIsp * G0) * h;
            v.tAcs = std::max(v.tAcs, std::min(1.0, force / (3 * v.acsF)));
        }
    }
    Vec3d wdot = v.invI * (tauExt + tauC - gyro);
    v.w += wdot * h;
    double wl = len(v.w);
    if (wl > 30.0) { v.w *= 30.0 / wl; wl = 30.0; }
    if (wl > 1e-12) v.q = (v.q * Quatd::axisAngle(v.w, wl * h)).normalized();
}

static bool vesselControllable(const World& W, const Vessel& v) {
    return v.hasControl && (!v.tPowerOut || W.settings.infiniteFuel || v.crew > 0);
}

// frame-level attitude update for vessels coasting on rails (no external torque)
static void attitudeOnRails(World& W, Vessel& v, double dt, bool highWarp, bool isActive) {
    if (v.landed || v.destroyed) { v.w = {}; return; }
    bool control = vesselControllable(W, v);
    if (highWarp) {
        v.w = {};
        Vec3d target = (v.sas != SAS::Off && v.sas != SAS::Stability) ? W.targetDirection(v, v.sas) : Vec3d{};
        if (control && len(target) > 0.5) v.q = sasRailAttitude(v, target);
        return;
    }
    int steps = std::max(1, (int)std::ceil(dt / 0.01));
    double h = dt / steps;
    rcsBegin(W, v);
    for (int k = 0; k < steps; k++) rotStep(W, v, h, {}, {}, isActive, control);
    rcsEnd(W, v, dt);
    if (v.rcsUse > 0 || v.acsUse > 0) updateMassProps(v);
}

// ------------------------------------------------------------------ main vessel advance
static void handleSOI(World& W, Vessel& v, double t, std::vector<BodyState>& S) {
    const Universe& U = W.U;
    for (int guard = 0; guard < 4; guard++) {
        const Body& b = U.bodies[v.ref];
        int newRef = -1;
        if (b.parent >= 0 && len(v.r) > b.soi) newRef = b.parent;
        else {
            for (int c : b.children) {
                Vec3d d = v.r - U.relPos(S, c, v.ref);
                if (len(d) < U.bodies[c].soi) { newRef = c; break; }
            }
        }
        if (newRef < 0) return;
        int oldRef = v.ref;
        U.computeStates(t, S);
        v.r = v.r + U.relPos(S, v.ref, newRef);
        v.v = v.v + U.relVel(S, v.ref, newRef);
        v.ref = newRef;
        U.computeStatesFor(t, U.relevant[newRef], S);
        if (&v == W.activeVessel()) {
            const Body& nb = U.bodies[newRef];
            Conic c = stateToConic(v.r, v.v, nb.GM);
            bool closed = c.valid && c.e < 1 && (nb.parent < 0 || c.ap < nb.soi);
            if (newRef == U.bodies[oldRef].parent)
                W.toast(trf("Escaped the gravity of {0} - now in the sphere of influence of {1}", tr(U.bodies[oldRef].name), tr(nb.name)),
                        {255, 214, 140, 255});
            else if (closed)
                W.toast(trf("Captured by the gravity of {0}", tr(nb.name)), {150, 220, 255, 255});
            else
                W.toast(trf("Entered the sphere of influence of {0} (flyby trajectory)", tr(nb.name)), {150, 200, 255, 255});
            W.event(v, SimEvent::Gravity);
            diag::breadcrumb("SOI %s -> %s", U.bodies[oldRef].name.c_str(), nb.name.c_str());
        }
        v.bound = -1; // re-evaluated silently in the new sphere of influence
    }
}

static void checkRootDominance(World& W, Vessel& v, double t) {
    const Universe& U = W.U;
    if (len(v.r) < 1e14 && U.bodies[v.ref].parent >= 0) return;
    std::vector<BodyState> S;
    U.computeStates(t, S);
    int best = -1;
    double bestA = 0;
    for (int rt : U.roots) {
        Vec3d d = U.relPos(S, rt, v.ref) - v.r;
        double a = U.bodies[rt].GM / len2(d);
        if (a > bestA) { bestA = a; best = rt; }
    }
    int curRoot = U.bodies[v.ref].root;
    if (best >= 0 && best != curRoot) {
        Vec3d dCur = U.relPos(S, curRoot, v.ref) - v.r;
        double aCur = U.bodies[curRoot].GM / len2(dCur);
        if (bestA > aCur * 1.2) {
            v.r = v.r + U.relPos(S, v.ref, best);
            v.v = v.v + U.relVel(S, v.ref, best);
            v.ref = best;
            if (&v == W.activeVessel()) W.toast(trf("Now dominated by {0}", tr(U.bodies[best].name)), {255, 220, 150, 255});
        }
    }
}

static void destroyVessel(World& W, Vessel& v, const std::string& why) {
    if (v.destroyed) return;
    W.event(v, SimEvent::Explosion);
    v.destroyed = true;
    v.fate = why;
    diag::breadcrumb("vessel lost: %s - %s", v.name.c_str(), why.c_str());
    v.throttle = 0;
    if (!v.debris) W.toast(v.name + ": " + why, {255, 110, 100, 255});
    if (!v.debris && &v != W.activeVessel() && W.blasts.size() < 16) W.blasts.push_back({v.ref, v.r});
}

// shift the tracked CoM position after the mass distribution changed (fuel burn, staging)
static void followCoM(World& W, Vessel& v, const Vec3d& oldCom, double t) {
    Vec3d d = v.mp.com - oldCom;
    if (len2(d) < 1e-18) return;
    Vec3d dw = v.q.rotate(d);
    if (v.landed) {
        Mat3d R = W.U.bodyRot(v.ref, t);
        v.landPos = v.landPos + R.T() * dw;
    }
    v.r += dw;
}

static void checkGround(World& W, Vessel& v, double t) {
    if (v.landed || v.destroyed) return;
    const Universe& U = W.U;
    const Body& b = U.bodies[v.ref];
    double rl = len(v.r);
    if (rl - b.radius > 0.06 * b.radius + v.length + v.maxR + 60) return; // far above any relief
    if (rl - b.radius - (ground::hasRelief(v.ref) ? ground::maxHeight(v.ref) : 0.0) > v.length + v.maxR + 60) return;
    Vec3d up = v.r / rl;
    // the ground: the reference sphere plus the real elevation (Earth, Moon, Mars), the same function
    // the renderer raises the ground with
    const bool relief = ground::hasRelief(v.ref);
    Mat3d RT = relief ? U.bodyRot(v.ref, t).T() : Mat3d{};
    if (relief && rl - b.radius - ground::height(v.ref, RT * v.r) > v.length + v.maxR + 60) return;
    double minAlt = 1e30;
    int lowest = -1;
    for (size_t k = 0; k < v.contacts.size(); k++) {
        if ((v.contactLeg[k] == 1 || v.contactLeg[k] == 2) && !v.legsDeployed) continue; // (folded legs / gear)
        Vec3d p = v.r + v.q.rotate(v.contacts[k] - v.mp.com);
        double a = len(p) - b.radius - (relief ? ground::height(v.ref, RT * p) : 0.0);
        if (a < minAlt) { minAlt = a; lowest = (int)k; }
    }
    if (lowest >= 0 && v.contactLeg[lowest] >= 2 && !b.isStar && !b.gasGiant && minAlt <= 0.05) {
        // ---- on the wheels: the gear carries the vessel and it rolls; a hard touchdown breaks the gear
        Vec3d vs0 = W.surfaceVel(v);
        double vz = dot(vs0, up);
        if (vz < -6.0) {
            char buf[64];
            std::snprintf(buf, sizeof buf, "%.1f", -vz);
            destroyVessel(W, v, trf("The landing gear collapsed on {0} ({1} m/s sink rate)", tr(b.name), buf));
            return;
        }
        if (!v.rolling && &v == W.activeVessel()) {
            char buf[64];
            std::snprintf(buf, sizeof buf, "%.1f", -vz);
            W.toast(trf("Wheels on the ground  \u00b7  sink rate {0} m/s", buf), {140, 255, 170, 255});
            W.event(v, SimEvent::Touchdown);
        }
        v.rolling = true;
        if (minAlt < 0) v.r = v.r + up * (-minAlt);
        if (vz < 0) v.v = v.v - up * vz;
        return;
    }
    if (minAlt > 0 || lowest < 0) {
        if (v.rolling && (lowest < 0 || minAlt > 0.6)) v.rolling = false; // airborne
        return;
    }
    if (b.isStar) { destroyVessel(W, v, trf("Vaporised in the photosphere of {0}", tr(b.name))); return; }
    if (b.gasGiant) { destroyVessel(W, v, trf("Crushed in the depths of {0}", tr(b.name))); return; }
    Vec3d vs = W.surfaceVel(v);
    double sp = len(vs);
    Vec3d fwd = v.q.rotate({0, 0, 1});
    bool upright = dot(fwd, up) > 0.85;
    double tol = 5.0;
    if (v.contactLeg[lowest]) tol = 10.0;
    else {
        // a heat shield (or other blunt PICA body) at the bottom tolerates harder landings
        for (size_t i = 0; i < v.comps.size(); i++)
            if (v.comps[i].func == Func::HeatShield && upright) tol = std::max(tol, 9.0);
    }
    if (sp > tol) {
        char buf[64];
        std::snprintf(buf, sizeof buf, "%.1f", sp);
        destroyVessel(W, v, trf("Crashed into {0} at {1} m/s", tr(b.name), buf));
        return;
    }
    v.landed = true;
    W.event(v, SimEvent::Touchdown);
    if (&v == W.activeVessel() && v.ref < (int)W.explored.size() && (W.explored[v.ref] & 4)) {
        char sb[32];
        std::snprintf(sb, sizeof sb, "%.1f", sp);
        W.toast(trf("Landed on {0}  ·  {1} m/s", tr(b.name), sb), {134, 239, 172, 255});
        W.event(v, SimEvent::Landed);
    }
    v.r = v.r + up * (-minAlt);
    Mat3d R = U.bodyRot(v.ref, t);
    v.landPos = R.T() * v.r;
    v.landQ = (Quatd::fromMat(R).conj() * v.q).normalized();
    v.chuteDeployed = false;
    pinLanded(W, v, t);
    if (&v == W.activeVessel()) {
        char buf[64];
        std::snprintf(buf, sizeof buf, "%.1f", sp);
        W.toast(trf("Touchdown on {0} at {1} m/s", tr(b.name), buf), {140, 255, 170, 255});
    }
}

// Wheels rolling on the ground: rolling resistance, brakes when the throttle is closed, the tyres stop
// any sideways slip, and the vessel parks (pinned to the ground) once it has stopped.
static void groundRoll(World& W, Vessel& v, double h, double t) {
    if (!v.rolling || v.landed || v.destroyed) return;
    const Body& b = W.U.bodies[v.ref];
    Vec3d up = norm(v.r);
    double g = b.GM / len2(v.r);
    Vec3d vs = W.surfaceVel(v);
    double vz = dot(vs, up);
    Vec3d vh = vs - up * vz;
    double sp = len(vh);
    // the heading of the wheels: the nose (+Z) projected onto the ground
    Vec3d fwd = v.q.rotate({0, 0, 1});
    fwd = fwd - up * dot(fwd, up);
    if (len(fwd) < 1e-3) fwd = v.q.rotate({0, 1, 0}) - up * dot(v.q.rotate({0, 1, 0}), up);
    fwd = norm(fwd);
    double along = dot(vh, fwd);
    Vec3d side = vh - fwd * along;
    double drive = roverDrive(W, v);
    double brake = v.throttle < 0.01 && drive == 0 ? 0.35 : 0.02; // brakes, or rolling resistance
    double dv = brake * g * h;
    double na = std::fabs(along) > dv ? along - (along > 0 ? dv : -dv) : 0.0;
    // a rover's drive motors: the push is limited by their power, by the grip of the wheels, and the gearing
    // gives a top speed
    if (drive != 0 && v.drivePower > 0 && (v.charge > 0 || W.settings.infiniteFuel)) {
        double m = std::max(W.vesselMass(v), 1.0);
        double P = v.drivePower * std::fabs(drive);
        double F = std::min(P / std::max(std::fabs(na), 0.05), 0.6 * m * g);
        double target = drive * v.driveVmax;
        double dvd = F / m * h * (drive > 0 ? 1 : -1);
        if (drive > 0) na = std::min(std::max(na, na + dvd), std::max(na, target));
        else na = std::max(std::min(na, na + dvd), std::min(na, target));
        if (!W.settings.infiniteFuel) v.charge = std::max(0.0, v.charge - P * h);
        v.tPowerUse = std::max(v.tPowerUse, P);
    }
    Vec3d nside = side * std::exp(-h * 6.0); // tyres grip sideways
    Vec3d dV = fwd * (na - along) + (nside - side);
    v.v = v.v + dV;
    // the gear carries the attitude: no wheel sinks into the ground. A sinking wheel tips the vessel about
    // its centre of mass (then the whole vessel is lifted), and a rotation that would push a wheel on the
    // ground further down is stopped: the nose rests on the nose wheel until the elevator lifts it, the
    // vessel then pivots on the main wheels
    {
        const bool relief = ground::hasRelief(v.ref);
        Mat3d RT = relief ? W.U.bodyRot(v.ref, t).T() : Mat3d{};
        auto altOf = [&](const Vec3d& pw) { return len(pw) - b.radius - (relief ? ground::height(v.ref, RT * pw) : 0.0); };
        for (int it = 0; it < 3; it++) {
            Vec3d dth{};
            double wsum = 0, lowest = 1e30;
            for (size_t k = 0; k < v.contacts.size(); k++) {
                if (v.contactLeg[k] < 2 || (v.contactLeg[k] == 2 && !v.legsDeployed)) continue;
                Vec3d rk = v.q.rotate(v.contacts[k] - v.mp.com);
                double a = altOf(v.r + rk);
                lowest = std::min(lowest, a);
                Vec3d rh = rk - up * dot(rk, up);
                if (a < 0 && len2(rh) > 1e-6) {
                    dth += cross(rk, up) * (-a);
                    wsum += len2(rh);
                }
            }
            if (wsum > 0) {
                dth = dth * (1.0 / wsum);
                double ang = len(dth);
                if (ang > 1e-9) v.q = (Quatd::axisAngle(dth / ang, std::min(ang, 0.05)) * v.q).normalized();
            }
            // then the lowest wheel back onto the ground
            lowest = 1e30;
            for (size_t k = 0; k < v.contacts.size(); k++)
                if (v.contactLeg[k] == 3 || (v.contactLeg[k] == 2 && v.legsDeployed)) lowest = std::min(lowest, altOf(v.r + v.q.rotate(v.contacts[k] - v.mp.com)));
            if (lowest < 0 && lowest > -5) v.r = v.r + up * (-lowest);
        }
        // a wheel on the ground that moves down is stopped by an impulse at its contact, on the motion and the
        // rotation alike: when the elevator lifts the nose, the vessel pivots on its main wheels
        double m = std::max(v.mp.mass, 1.0);
        Vec3d upB = v.q.conj().rotate(up);
        for (int it = 0; it < 4; it++)
            for (size_t k = 0; k < v.contacts.size(); k++) {
                if (v.contactLeg[k] < 2 || (v.contactLeg[k] == 2 && !v.legsDeployed)) continue;
                Vec3d rkB = v.contacts[k] - v.mp.com;
                Vec3d rk = v.q.rotate(rkB);
                if (altOf(v.r + rk) > 0.03) continue;
                double vk = dot(v.v, up) + dot(cross(v.q.rotate(v.w), rk), up);
                if (vk >= 0) continue;
                Vec3d bI = v.invI * cross(rkB, upB);
                double kk = 1 / m + dot(cross(bI, rkB), upB);
                if (kk <= 1e-12) continue;
                double J = -vk / kk;
                v.v = v.v + up * (J / m);
                v.w = v.w + bI * J;
            }
    }
    // the gear keeps the wings level and the vessel upright on the ground
    v.w.z *= std::exp(-h * 4.0);
    // nose-wheel steering with the yaw input: the turn follows the rolling speed; less wheel angle when fast
    // a rover's wheels steer at the corners: it also turns slowly on the spot (at its drive speed), as
    // Curiosity and Perseverance do
    bool rover = v.drivePower > 0;
    if (&v == W.activeVessel() && std::fabs(W.ctrl.yaw) > 0.01 && (std::fabs(along) > 0.2 || rover)) {
        double steer = W.ctrl.yaw * std::clamp(30.0 - std::fabs(along) * 0.45, 3.0, 30.0) * DEG;
        double base = std::max(rover ? 1.0 : 2.0, v.length * 0.45);
        double vs2 = rover ? (std::fabs(along) > v.driveVmax * 0.5 ? along : v.driveVmax) : along;
        double rate = vs2 * std::tan(steer) / base;
        v.q = (v.q * Quatd::axisAngle(Vec3d{0, 1, 0}, rate * h)).normalized();
    }
    if (sp < 0.3 && v.throttle < 0.01 && drive == 0) {
        // stopped: park
        v.rolling = false;
        v.landed = true;
        Mat3d R = W.U.bodyRot(v.ref, t);
        v.landPos = R.T() * v.r;
        v.landQ = (Quatd::fromMat(R).conj() * v.q).normalized();
        pinLanded(W, v, t);
    }
}

static double numericDt(const World& W, const Vessel& v) {
    const Body& b = W.U.bodies[v.ref];
    double rl = std::max(len(v.r), b.radius);
    double T = TAU * std::sqrt(rl * rl * rl / b.GM);
    double dt = T / 900.0;
    double alt = rl - b.radius;
    if (b.atmTop > 0 && alt < b.atmTop) {
        // coasting through thin air the drag changes slowly: longer steps (dense air, power, parachute: 40 ms)
        double da = 0.04;
        if (v.tAccel <= 0.02 && !v.landed && !(v.chuteDeployed && !v.chuteCut)) {
            double q = airDynP(W, v);
            da = q < 0.5 ? 1.0 : q < 5 ? 0.4 : q < 50 ? 0.12 : 0.04;
        }
        dt = std::min(dt, da);
    }
    if (v.tAccel > 0.02) dt = std::min(dt, v.tAccel > 2 ? 0.1 : 0.5);
    if (v.landed || v.rolling) dt = std::min(dt, 0.05);
    return std::max(dt, 1e-3);
}

static double railsDt(const World& W, const Vessel& v, const std::vector<BodyState>& S) {
    const Universe& U = W.U;
    const Body& b = U.bodies[v.ref];
    double rl = len(v.r), sp = std::max(len(v.v), 1e-3);
    double dt = 1e30;
    Conic c = stateToConic(v.r, v.v, b.GM);
    if (c.valid && c.e < 1) dt = c.period / 16.0;
    for (int ch : b.children) {
        const Body& cb = U.bodies[ch];
        Vec3d d = v.r - U.relPos(S, ch, v.ref);
        Vec3d rv = v.v - U.relVel(S, ch, v.ref);
        double gap = len(d) - cb.soi;
        // the crossing is caught within 0.5 % of the SOI radius, so the flown pass matches the predicted one
        dt = std::min(dt, std::max(gap, cb.soi * 0.01) / (len(rv) + 1.0) * 0.5);
    }
    if (b.parent >= 0) dt = std::min(dt, std::max((b.soi - rl) * 0.5, b.soi * 0.02) / sp);
    double floor = b.radius + std::max(b.atmTop, ground::maxHeight(v.ref));
    if (c.valid && c.pe < floor) {
        double gap = rl - floor;
        dt = std::min(dt, std::max(0.5, 0.3 * gap / sp));
    }
    if (rl > 1e14) dt = std::min(dt, 0.02 * rl / sp);
    return std::max(dt, 0.05);
}

static void numericStep(World& W, Vessel& v, double t0, double h, bool isActive, bool highWarp, std::vector<BodyState>& S0,
                        std::vector<BodyState>& S1) {
    const Universe& U = W.U;
    const Body& b = U.bodies[v.ref];
    double alt = len(v.r) - b.radius;
    double pres = U.pressure(v.ref, alt);
    bool inAtm = b.atmTop > 0 && alt < b.atmTop;

    // electrical balance (uses previous-step demand for electric engines)
    bool shadow = false;
    double flux = starFlux(W, S0, v.ref, v.r, &shadow, nullptr, nullptr);
    double gen = v.genW + v.solarArea * flux;
    double powerFactor = 1;
    if (!W.settings.infiniteFuel && v.charge <= 1e-6) {
        double dem = v.tPowerUse;
        powerFactor = dem > 0 ? clampd((gen - v.drawW) / std::max(1.0, dem - v.drawW), 0, 1) : 1;
    }
    double m0 = W.vesselMass(v);
    EngineOut eo = (isActive || v.ignited) ? runEngines(W, v, pres, h, powerFactor) : EngineOut{};
    Vec3d oldCom = v.mp.com;
    updateMassProps(v);
    followCoM(W, v, oldCom, t0);
    double demand = v.drawW + eo.electricW;
    if (W.settings.infiniteFuel) v.charge = v.chargeCap;
    else v.charge = clampd(v.charge + (gen - demand) * h, 0, v.chargeCap);
    v.tPowerGen = gen;
    v.tPowerUse = demand;
    v.tSolarFlux = flux;
    v.tInShadow = shadow;
    v.tPowerOut = !W.settings.infiniteFuel && v.charge <= 1e-6 && gen < v.drawW;
    v.tFlameout = eo.flameout && !eo.any; // (a dry engine beside burning ones is no flameout)
    v.tThrust = eo.F;
    double m1 = W.vesselMass(v), mAvg = 0.5 * (m0 + m1);
    bool control = vesselControllable(W, v);

    if (v.landed) {
        Vec3d up = norm(v.r);
        double g = b.GM / len2(v.r);
        Vec3d thrustW = v.q.rotate(eo.Fbody);
        Vec3d thrustH = thrustW - up * dot(thrustW, up);
        if (dot(thrustW, up) / mAvg > g * 1.0005) {
            v.landed = false;
        } else if ((v.hasWheels && len(thrustH) / mAvg > 0.03 * g) || roverDrive(W, v) != 0) {
            v.landed = false; // released from the parking brake: the take-off run (or a rover drives off)
            v.rolling = true;
        } else {
            pinLanded(W, v, t0 + h);
            v.tAccel = len(thrustW) / mAvg;
            v.tAlt = len(v.r) - b.radius;
            v.tDynP = 0;
            // the caller swaps S0/S1 after every step: S1 must hold the body states at t0 + h
            U.computeStatesFor(t0 + h, U.relevant[v.ref], S1);
            return;
        }
    }

    // tables of inactive vessels may have been released under memory pressure: rebuild on demand
    if (inAtm && !v.aero && !v.comps.empty()) v.aero = aeroTableCached(v.geometry());

    // ---- rotation (sub-stepped: aerodynamic torques can be stiff)
    Vec3d sailF, sailT;
    double sailMag = 0;
    if (highWarp) {
        v.w = {};
        if (control && v.sas != SAS::Off && v.sas != SAS::Stability) {
            Vec3d tg = W.targetDirection(v, v.sas);
            if (len(tg) > 0.5) v.q = sasRailAttitude(v, tg);
        }
    } else {
        double hr = inAtm ? 0.004 : 0.02;
        int n = std::clamp((int)std::ceil(h / hr), 1, 400);
        double hs = h / n;
        // the surfaces' real authority (the SAS counts on it): the torque a deflection gives now, by the same
        // aerodynamics, against the low-speed estimate; measured a few times a second
        if (inAtm && control && len2(v.ctrlAuth) > 0 && v.tDynP > 20 && std::fabs(t0 - v.ctrlEffT) > 0.2) {
            v.ctrlEffT = t0;
            Vec3d s0 = v.surf;
            double ax[3] = {1, 1, 1};
            const double au[3] = {v.ctrlAuth.x, v.ctrlAuth.y, v.ctrlAuth.z};
            for (int a = 0; a < 3; a++) {
                if (au[a] * v.tDynP < 1e-6) continue;
                Vec3d sp = s0, sm = s0;
                double c0 = a == 0 ? s0.x : a == 1 ? s0.y : s0.z;
                double hi = std::min(1.0, c0 + 0.25), lo = std::max(-1.0, c0 - 0.25);
                (a == 0 ? sp.x : a == 1 ? sp.y : sp.z) = hi;
                (a == 0 ? sm.x : a == 1 ? sm.y : sm.z) = lo;
                v.surf = sp;
                Vec3d tp = aerodynamics(W, v, v.r, v.v, v.q, v.w, nullptr).Tbody;
                v.surf = sm;
                Vec3d tm = aerodynamics(W, v, v.r, v.v, v.q, v.w, nullptr).Tbody;
                double d = a == 0 ? tp.x - tm.x : a == 1 ? tp.y - tm.y : tp.z - tm.z;
                ax[a] = clampd(std::fabs(d) / ((hi - lo) * au[a] * v.tDynP), 0.05, 1.5);
            }
            v.surf = s0;
            v.ctrlEff = v.ctrlEff * 0.5 + Vec3d{ax[0], ax[1], ax[2]} * 0.5;
        }
        rcsBegin(W, v);
        for (int k = 0; k < n; k++) {
            Vec3d tau = eo.Tbody;
            if (inAtm) tau += aerodynamics(W, v, v.r, v.v, v.q, v.w, nullptr).Tbody;
            if (v.sailDeployed) {
                sailForces(W, v, S0, v.r, v.q, sailF, sailT, sailMag);
                tau += sailT;
            }
            rotStep(W, v, hs, tau, eo.gimbalAuth, isActive, control);
        }
        rcsEnd(W, v, h);
    }
    Vec3d rcsF = h > 0 ? v.rcsImpulse / h : Vec3d{};
    if (highWarp) rcsF = {};

    // ---- translation (velocity Verlet)
    U.computeStatesFor(t0 + h, U.relevant[v.ref], S1);
    static thread_local std::vector<float> exposure; // reused: no allocation per physics step
    exposure.clear();
    auto accel = [&](const std::vector<BodyState>& S, const Vec3d& r, const Vec3d& vel, AeroOut* aoOut, std::vector<float>* ex) {
        Vec3d a = gravity(W, S, v.ref, r) + v.q.rotate(eo.Fbody + rcsF) / mAvg;
        if (inAtm) {
            AeroOut ao = aerodynamics(W, v, r, vel, v.q, v.w, ex);
            a += v.q.rotate(ao.Fbody) / mAvg;
            if (aoOut) *aoOut = ao;
        }
        if (v.sailDeployed) {
            Vec3d F, T;
            double mg;
            sailForces(W, v, S, r, v.q, F, T, mg);
            a += F / mAvg;
            sailMag = mg;
        }
        return a;
    };
    AeroOut ao;
    Vec3d a0 = accel(S0, v.r, v.v, nullptr, nullptr);
    Vec3d vh = v.v + a0 * (h * 0.5);
    Vec3d r1 = v.r + vh * h;
    Vec3d a1 = accel(S1, r1, vh, &ao, &exposure);
    v.r = r1;
    v.v = vh + a1 * (h * 0.5);
    v.tAccel = (eo.F + sailMag) / mAvg;
    v.tDynP = ao.q;
    {
        double fd = dot(ao.Fbody, ao.flowBody);
        v.tDragAcc = std::fabs(fd) / mAvg;
        v.tLiftAcc = len(ao.Fbody - ao.flowBody * fd) / mAvg;
    }
    if (v.solarDeployed && v.solarAreaMax > 0 && ao.q > 3000) {
        Vec3d oldCom2 = v.mp.com;
        std::vector<Component> nc;
        std::vector<double> np, nh;
        for (size_t i = 0; i < v.comps.size(); i++) {
            if (v.comps[i].func == Func::Solar) continue;
            nc.push_back(v.comps[i]);
            np.push_back(v.prop[i]);
            nh.push_back(v.heat[i]);
        }
        v.comps = nc;
        v.prop = np;
        v.heat = nh;
        setupVessel(v, true);
        followCoM(W, v, oldCom2, t0 + h);
        diag::breadcrumb("solar panels torn off: %s", v.name.c_str());
        W.event(v, SimEvent::PanelsTorn);
        if (&v == W.activeVessel()) W.toast("Deployed solar panels were torn off by aerodynamic pressure", {255, 110, 100, 255});
        return;
    }
    v.tHeatFlux = ao.heatFlux;
    v.tSailForce = sailMag;
    v.tMach = ao.mach;
    v.tAoA = ao.aoa;
    v.tAlt = len(v.r) - b.radius;
    v.tTorque = eo.Tbody + ao.Tbody;

    // ---- heating, per component
    v.tHeatMax = 0;
    v.tHottest = -1;
    for (size_t i = 0; i < v.comps.size(); i++) {
        double e = (inAtm && i < exposure.size()) ? exposure[i] : 0.0;
        if (!v.legsDeployed && retractableWheel(v.comps[i])) e = 0; // (up in its bay behind the TPS-covered doors)
        double flux_i = ao.heatFlux * e;
        double ratio = flux_i / v.phys[i].heatLimit;
        if (ratio > 0.5) v.heat[i] += (ratio - 0.5) * h * 0.4;
        else v.heat[i] = std::max(0.0, v.heat[i] - 0.08 * h);
        if (v.heat[i] > v.tHeatMax) { v.tHeatMax = v.heat[i]; v.tHottest = (int)i; }
    }
    if (v.tHeatMax >= 1.0) {
        destroyVessel(W, v, tr("Burned up during atmospheric entry") + " (" + tr(v.comps[v.tHottest].name) + ")");
        return;
    }
    if (v.chuteDeployed && !v.chuteCut) v.chuteT += h;
    else v.chuteT = 0;
    // a drag chute is cast off on the runway once the vessel has slowed to its jettison speed
    if (v.chuteDeployed && !v.chuteCut && v.chuteJettV > 0 && (v.landed || v.rolling) && v.chuteOpen > 0.3 && ao.speed < v.chuteJettV) {
        v.chuteCut = true;
        if (isActive) W.toast("Drag chute jettisoned", {150, 200, 255, 255});
    }
    // canopy inflation: fully open after travelling ~8 canopy diameters
    if (v.chuteDeployed && !v.chuteCut && v.chuteCdA > 0) {
        int canopies = 0;
        for (const Component& c : v.comps)
            if (c.func == Func::Parachute) canopies += (c.def >= 0 && c.def < (int)CATALOG.defs.size()) ? std::max(1, CATALOG.defs[c.def].canopies) : 1;
        double D = std::sqrt(4 * v.chuteCdA / (PI * 0.8 * std::max(1, canopies)));
        v.chuteOpen = std::min(1.0, v.chuteOpen + h * std::max(ao.speed, 5.0) / (8 * D));
    } else v.chuteOpen = 0;
    if (v.chuteDeployed && !v.chuteCut && ao.q > v.chuteQMax) {
        v.chuteCut = true;
        if (isActive) W.toast("Parachute torn off: deployed too fast", {255, 170, 90, 255});
    }
    // structural limit: extreme aerodynamic loads tear the vessel apart
    if (ao.q > 0 && len(ao.Fbody) / mAvg > 40 * G0) destroyVessel(W, v, tr("Torn apart by aerodynamic forces"));
}

// returns the time flown: less than dt when the active vessel used up its physics steps (the frame ends
// there instead of taking one huge step)
static double advanceVessel(World& W, Vessel& v, double t0, double dt, bool isActive, int maxNumericSteps) {
    if (v.destroyed) return dt;
    const Universe& U = W.U;
    double tt = t0, end = t0 + dt;
    bool highWarp = W.warpFactor() > 4;
    std::vector<BodyState> S0, S1;
    U.computeStatesFor(tt, U.relevant[v.ref], S0);
    int numericSteps = 0, guard = 0;
    while (tt < end - 1e-9 && guard++ < 400000) {
        if (v.destroyed) return dt;
        double rem = end - tt;
        bool thrusting = thrustPossible(W, v) && (isActive || v.ignited);
        if (v.landed && !thrusting) {
            pinLanded(W, v, end);
            tt = end;
            break;
        }
        const Body& b = U.bodies[v.ref];
        double alt = len(v.r) - b.radius;
        bool inAtm = b.atmTop > 0 && alt < b.atmTop;
        int refBefore = v.ref;
        if (thrusting || inAtm || v.landed || v.rolling) {
            double h = std::min(rem, numericDt(W, v));
            if (numericSteps >= maxNumericSteps) {
                if (isActive) break;   // the world clock stops with the active vessel
                h = std::min(rem, 1.0); // others: coarse but bounded steps
            }
            numericStep(W, v, tt, h, isActive, highWarp, S0, S1);
            numericSteps++;
            tt += h;
            std::swap(S0, S1);
        } else {
            double h = std::min(rem, railsDt(W, v, S0));
            keplerPropagate(v.r, v.v, b.GM, h);
            tt += h;
            v.tThrust = 0;
            std::fill(v.engLevel.begin(), v.engLevel.end(), 0.f); // coasting on rails: nothing burns
            v.tAccel = 0;
            v.tDynP = 0;
            v.tHeatFlux = 0;
            v.tMach = 0;
            v.tTorque = {};
            v.tAlt = len(v.r) - b.radius;
            for (auto& hh : v.heat) hh = std::max(0.0, hh - 0.08 * h);
            if (!W.settings.infiniteFuel) {
                bool shadow = false;
                double flux = starFlux(W, S0, v.ref, v.r, &shadow, nullptr, nullptr);
                double gen = v.genW + v.solarArea * flux;
                v.charge = clampd(v.charge + (gen - v.drawW) * h, 0, v.chargeCap);
                v.tPowerGen = gen;
                v.tPowerUse = v.drawW;
                v.tSolarFlux = flux;
                v.tInShadow = shadow;
                v.tPowerOut = v.charge <= 1e-6 && gen < v.drawW;
            } else v.charge = v.chargeCap;
            U.computeStatesFor(tt, U.relevant[v.ref], S0);
        }
        handleSOI(W, v, tt, S0);
        if (v.ref != refBefore) U.computeStatesFor(tt, U.relevant[v.ref], S0);
        checkGround(W, v, tt);
        if (v.rolling) groundRoll(W, v, std::min(rem, numericDt(W, v)), tt);
        // a steep entry: within one frame the air gets thicker than the warp allows - the frame ends here
        if (isActive && !v.destroyed && W.warpIndex > PHYSICS_WARP_MAX) {
            const Body& b2 = U.bodies[v.ref];
            if (b2.atmTop > 0 && len(v.r) - b2.radius < b2.atmTop) {
                int lim = airWarpIndex(W, v);
                if (lim < W.warpIndex) {
                    W.warpIndex = lim;
                    break;
                }
            }
        }
    }
    if (v.destroyed) return dt;
    checkRootDominance(W, v, tt);
    return isActive ? tt - t0 : dt;
}

// ------------------------------------------------------------------ public update
OrbitBands orbitBands(const Body& b) {
    OrbitBands o;
    o.lowMin = b.atmTop > 0 ? b.atmTop : 0.02 * b.radius;
    o.lowMax = b.name == "Earth" ? 2.0e6 : 0.3 * b.radius;
    o.lowMax = std::max(o.lowMax, o.lowMin * 1.5);
    if (b.parent >= 0) o.lowMax = std::min(o.lowMax, b.soi - b.radius);
    if (b.rotRate != 0) {
        double T = TAU / std::fabs(b.rotRate);
        double a = std::cbrt(b.GM * T * T / (4 * PI * PI));
        double alt = a - b.radius;
        if (alt > o.lowMin && (b.parent < 0 || a < b.soi)) o.syncAlt = alt;
    }
    return o;
}

// classification of the current (coasting) orbit against the body's standards
static OrbitClass classifyOrbit(const World& W, const Vessel& v, Conic& c);
OrbitClass orbitClassOf(const World& W, const Vessel& v, Conic& c) { return classifyOrbit(W, v, c); }
static OrbitClass classifyOrbit(const World& W, const Vessel& v, Conic& c) {
    const Body& b = W.U.bodies[v.ref];
    if (v.landed) return OrbitClass::None;
    c = stateToConic(v.r, v.v, b.GM);
    double soi = b.parent >= 0 ? b.soi : 1e300;
    if (!c.valid || c.e >= 1 || c.ap >= soi) return OrbitClass::None;
    OrbitBands ob = orbitBands(b);
    double peAlt = c.pe - b.radius, apAlt = c.ap - b.radius;
    if (peAlt < ob.lowMin) return OrbitClass::None; // suborbital or decaying
    if (ob.syncAlt > 0 && b.rotRate != 0) {
        double T = TAU / std::fabs(b.rotRate);
        if (std::fabs(c.period - T) / T < 0.01 && c.e < 0.05) return OrbitClass::Synchronous;
    }
    if (apAlt <= ob.lowMax) return OrbitClass::Low;
    if (peAlt >= ob.lowMax) return OrbitClass::High;
    return OrbitClass::Spanning;
}

void World::recordHistory(bool force) {
    if (hyper.active || vessels.empty()) return;
    // nothing is recorded after the active vessel was lost: "back 30 s" then means 30 s before the loss
    // (recording the wreck made the first press land on another wreck state)
    if (const Vessel* a = activeVessel(); a && a->destroyed && !force) return;
    while (!history.empty() && history.back().t > t + 1e-6) history.pop_back(); // time went back (rewind / load)
    if (!force && !history.empty() && (t - history.back().t < 2.0 || realTime - historyReal < 0.25)) return;
    WorldSnapshot s;
    s.t = t;
    s.vessels = vessels;
    s.active = active;
    s.nextId = nextId;
    s.explored = explored;
    s.bases = bases;
    s.nextBaseId = nextBaseId;
    history.push_back(std::move(s));
    historyReal = realTime;
    // keep 10 minutes of game time (always at least the launch state), a bounded count and at most
    // ~256 MB (a very large craft is several MB per state)
    while (history.size() > 2 && history[1].t < t - 600.0) history.pop_front();
    size_t per = sizeof(WorldSnapshot);
    for (const auto& v : vessels)
        per += sizeof(Vessel) + v.comps.size() * (sizeof(Component) + sizeof(CompPhys) + 2 * sizeof(double) + sizeof(float)) +
               (v.shown.size() + v.stowedGeom.size()) * sizeof(Component) + v.rcsNoz.size() * (sizeof(RcsNozzle) + sizeof(float));
    size_t maxCount = std::clamp<size_t>((256u << 20) / std::max<size_t>(per, 1), 3, 400);
    while (history.size() > maxCount) history.erase(history.begin() + 1);
}

double World::rewindReach() const { return history.empty() ? 0 : std::max(0.0, t - history.front().t); }
// (after a loss the reach counts from now, so the buttons stay enabled; rewind() measures from the loss)

bool World::rewind(double seconds) {
    if (history.empty()) return false;
    double base = t;
    if (const Vessel* a = activeVessel(); a && a->destroyed) base = std::min(t, history.back().t); // the last state before the loss
    double target = base - seconds;
    size_t pick = 0; // the newest snapshot not later than the target (the oldest if none is)
    for (size_t i = 0; i < history.size(); i++)
        if (history[i].t <= target + 1e-6) pick = i;
    const WorldSnapshot& s = history[pick];
    double back = t - s.t;
    if (back < 0.5) return false;
    t = s.t;
    vessels = s.vessels;
    vessels.reserve(vessels.size() + 64); // room for fairing halves and debris (see jettisonFairing / releaseDebris)
    active = s.active;
    nextId = s.nextId;
    explored = s.explored;
    if (bases.size() != s.bases.size()) { bases = s.bases; nextBaseId = s.nextBaseId; syncBaseTerrain(); }
    else { bases = s.bases; nextBaseId = s.nextBaseId; }
    history.erase(history.begin() + pick + 1, history.end());
    warpIndex = 0;
    warpTo = -1;
    warpToRef = -1;
    events.clear();
    hyper = HyperJump{};
    refreshStates();
    diag::breadcrumb("rewind %.1f s", back);
    toast(trf("Rewound {0}", formatTime(back)), {150, 200, 255, 255});
    return true;
}

bool World::startWarpTo(double tTarget) {
    Vessel* v = activeVessel();
    if (!v || v->destroyed || v->landed || !(tTarget > t + 1.0) || !std::isfinite(tTarget)) return false;
    if (thrustPossible(*this, *v)) {
        toast("Stop the engines before warping to a point", {255, 200, 120, 255});
        return false;
    }
    warpTo = tTarget;
    warpToRef = v->ref;
    toast(trf("Warping to the selected point ({0})", formatTime(tTarget - t)), {150, 200, 255, 255});
    return true;
}

void World::cancelWarpTo(const char* why) {
    warpTo = -1;
    warpToRef = -1;
    if (why) toast(why, {255, 200, 120, 255});
}

// the ground under a landed vessel changed (detail tiles of its body were loaded or dropped): it stands on
// the new ground again. Not at the launch complexes: the ground there is flat in every level and the
// vehicle stands on the deck of the mount.
static void reseatLanded(World& W, Vessel& v) {
    const Body& b = W.U.bodies[v.ref];
    if (!ground::reliefReady(v.ref) || len(v.landPos) < 1) return;
    Vec3d up = norm(v.landPos);
    if (b.name == "Earth")
        for (int s = 0; s < LAUNCH_SITE_COUNT; s++) {
            double la = LAUNCH_SITES[s].latDeg * DEG, lo = LAUNCH_SITES[s].lonDeg * DEG;
            Vec3d c{std::cos(la) * std::cos(lo), std::cos(la) * std::sin(lo), std::sin(la)};
            if (dot(up, c) > std::cos(0.25 * DEG)) return;
        }
    double minAlt = 1e30;
    for (size_t k = 0; k < v.contacts.size(); k++) {
        if ((v.contactLeg[k] == 1 || v.contactLeg[k] == 2) && !v.legsDeployed) continue; // (folded legs / gear)
        Vec3d p = v.landPos + v.landQ.rotate(v.contacts[k] - v.mp.com);
        minAlt = std::min(minAlt, len(p) - b.radius - ground::height(v.ref, p));
    }
    if (minAlt > 1e29 || std::fabs(minAlt) < 0.3) return;
    v.landPos = v.landPos + up * (0.02 - minAlt);
    pinLanded(W, v, W.t);
}

void World::update(double realDt) {
    realTime += realDt;
    // the ground: landed vessels follow a change of it, then the detail tiles follow the active vessel (the
    // tiles right below it are decoded now, before the physics)
    for (Vessel& v : vessels) {
        if (v.destroyed || !v.landed) continue;
        unsigned r = ground::revision(v.ref);
        if (r != v.groundRev) {
            v.groundRev = r;
            reseatLanded(*this, v);
        }
    }
    if (Vessel* a = activeVessel(); a && !a->destroyed) {
        const Body& b = U.bodies[a->ref];
        if (!b.isStar && !b.gasGiant) ground::focus(a->ref, U.bodyRot(a->ref, t).T() * a->r, len(a->r) - b.radius);
        else ground::focus(-1, {}, 0);
    }
    {
        // the pilot's rotation input ramps in (keys are all or nothing) and back out a little faster
        auto ramp = [&](double cur, double want) {
            bool growing = std::fabs(want) > std::fabs(cur) && want * cur >= 0;
            double r = (growing ? INPUT_RAMP_IN : INPUT_RAMP_OUT) * std::min(realDt, 0.05);
            return cur + std::clamp(want - cur, -r, r);
        };
        ctrl.pitch = ramp(ctrl.pitch, input.pitch);
        ctrl.yaw = ramp(ctrl.yaw, input.yaw);
        ctrl.roll = ramp(ctrl.roll, input.roll);
        ctrl.tx = input.tx;
        ctrl.ty = input.ty;
        ctrl.tz = input.tz;
    }
    while (!toasts.empty() && realTime - toasts.front().born > 6.0) toasts.pop_front();
    if (hyper.active) {
        updateHyperspace(realDt);
        refreshStates();
        return;
    }
    realDt = std::min(realDt, 0.05);
    if (warpTo > 0) {
        // warp to a point: the fastest level that still takes ~2.5 s to cover the rest (it slows down
        // on the approach), never above what the situation allows
        Vessel* a = activeVessel();
        if (!a || a->destroyed || a->landed) cancelWarpTo("Warp to point cancelled");
        else if (a->ref != warpToRef) cancelWarpTo("Warp to point cancelled: the vessel left the sphere of influence");
        else if (thrustPossible(*this, *a)) cancelWarpTo("Warp to point cancelled: engines are firing");
        else {
            double remain = warpTo - t;
            int mx = maxWarpIndex();
            int best = std::min(4, mx); // at least x10 on rails: the final frame is clamped onto the point
            for (int i = best + 1; i <= mx; i++)
                if (WARP_LEVELS[i] * 0.6 <= remain) best = i;
            warpIndex = best;
        }
    }
    if (warpIndex > maxWarpIndex()) warpIndex = maxWarpIndex();
    bool held = false;
    if (warpHold > 0) {
        // an event is coming up: at least 1.5 s of real time must remain at the current rate
        double left = warpHold - t;
        if (left <= 0) warpHold = -1;
        else
            while (warpIndex > 0 && WARP_LEVELS[warpIndex] * 1.5 > left) {
                warpIndex--;
                held = true;
            }
    }
    double warp = warpFactor();
    double dtFrame = realDt * warp;
    if (warpTo > 0) dtFrame = std::min(dtFrame, std::max(0.0, warpTo - t)); // stop exactly on the point
    warpLimitReason.clear();
    if (held) warpLimitReason = "next tutorial step";
    // a vessel coming down into an atmosphere (or toward an airless surface) within this frame: the frame
    // ends there and the warp drops to what the air allows, so the pass is flown instead of skipped (at
    // x100,000 and more a whole orbit fits into one frame)
    if (dtFrame > 1.0) {
        double te = 1e300;
        int lim = warpIndex;
        bool air = true;
        for (const Vessel& v : vessels) {
            if (v.destroyed || v.landed || v.debris) continue;
            const Body& b = U.bodies[v.ref];
            double rf;
            int l;
            if (b.atmTop > 0) {
                rf = b.radius + b.atmTop;
                l = THIN_AIR_WARP;
            } else if (!b.isStar && !b.gasGiant) {
                // only on a course into the ground (a low orbit above the summits keeps its warp)
                Conic k = stateToConic(v.r, v.v, b.GM);
                if (!k.valid || k.pe >= b.radius + ground::maxHeight(v.ref)) continue;
                rf = b.radius + ground::maxHeight(v.ref) + surfaceApproach(b);
                l = 4; // x10
            } else continue;
            if (warpIndex <= l) continue;
            double tx = timeToRadiusInbound(v.r, v.v, b.GM, rf);
            if (tx >= 0 && tx < dtFrame && tx < te) {
                te = tx;
                lim = l;
                air = b.atmTop > 0;
            }
        }
        if (te < dtFrame) {
            dtFrame = std::min(dtFrame, te + 0.5);
            warpIndex = lim;
            if (!air && warpTo > 0) {
                warpTo = -1;
                warpToRef = -1;
            }
            toast(air ? "Time warp reduced: entering the atmosphere" : "Time warp reduced: approaching the surface", {255, 200, 120, 255});
        }
    }

    Vessel* av = activeVessel();
    const int MAX_SUB = 3000;
    if (av && !av->destroyed) {
        bool thrusting = thrustPossible(*this, *av);
        const Body& b = U.bodies[av->ref];
        bool inAtm = b.atmTop > 0 && len(av->r) - b.radius < b.atmTop;
        if (thrusting || inAtm || av->landed || av->rolling) {
            double h = numericDt(*this, *av);
            if (dtFrame / h > MAX_SUB) {
                dtFrame = MAX_SUB * h;
                warpLimitReason = "physics load";
            }
        } else {
            // fast warp: a frame can span weeks. Stop at the next change of the sphere of influence (an
            // encounter or the escape) instead of flying through it within one frame
            if (dtFrame > 600 && !av->landed) {
                double te = 1e300;
                Encounter e;
                if (predictEncounter(U, av->ref, av->r, av->v, t, e)) te = e.tEntry;
                if (b.parent >= 0) {
                    Conic c = stateToConic(av->r, av->v, b.GM);
                    if (c.valid && !(c.e < 1 && c.ap < b.soi)) {
                        double cosExit = (c.p / b.soi - 1) / std::max(c.e, 1e-9);
                        if (std::fabs(cosExit) <= 1) {
                            double tx = timeToTrueAnomaly(c, std::acos(cosExit));
                            if (tx > 0) te = std::min(te, tx);
                        }
                    }
                }
                if (te < dtFrame) {
                    dtFrame = te + 1.0;
                    warpIndex = 0;
                    warpTo = -1;
                    warpToRef = -1;
                    toast("Time warp stopped: change of the sphere of influence", {150, 200, 255, 255});
                }
            }
            attitudeOnRails(*this, *av, dtFrame, warp > 4, true);
        }
    }
    effectiveWarp = realDt > 0 ? dtFrame / realDt : warp;

    // the active vessel first: the frame may end early when its physics steps run out
    double flown = dtFrame;
    if (active >= 0 && active < (int)vessels.size() && !vessels[active].destroyed)
        flown = std::clamp(advanceVessel(*this, vessels[active], t, dtFrame, true, MAX_SUB * 2), 0.0, dtFrame);
    if (flown < dtFrame * 0.999 && warpLimitReason.empty()) warpLimitReason = "physics load";
    effectiveWarp = realDt > 0 ? flown / realDt : warp;
    for (int i = 0; i < (int)vessels.size(); i++) {
        if (i == active) continue;
        Vessel& v = vessels[i];
        if (!v.destroyed) {
            const Body& b = U.bodies[v.ref];
            bool inAtm = b.atmTop > 0 && len(v.r) - b.radius < b.atmTop;
            if (v.debris && inAtm && !v.landed && warpIndex > PHYSICS_WARP_MAX) {
                // debris in the air at rails warp: a ballistic fall (no drag at this rate) to its impact
                Vec3d r0 = v.r, v0 = v.v;
                keplerPropagate(v.r, v.v, b.GM, flown);
                Mat3d RT = U.bodyRot(v.ref, t + flown).T();
                double ground = b.radius + (ground::hasRelief(v.ref) ? ground::height(v.ref, RT * norm(v.r)) : 0.0);
                bool hit = len(v.r) < ground || (dot(v.r, v0) < 0 && len(r0) > ground && dot(v.r - r0, norm(r0)) < 0 && len(v.r) < ground + 50);
                if (hit) {
                    v.r = norm(v.r) * ground;
                    v.destroyed = true;
                    v.fate = "Crashed";
                }
                continue;
            }
            if (!inAtm && !v.landed) attitudeOnRails(*this, v, flown, warp > 4, false);
        }
        advanceVessel(*this, v, t, flown, false, MAX_SUB * 2);
    }
    t += flown;
    refreshStates();
    checkDocking(); // (ports that met this frame latch: the vessels become one)
    recordHistory(false);
    if (warp > 4) {
        // at high warp the orbit moves a long way within one frame: point SAS-held vessels again at the
        // target direction of where they are now (the snap above used the start of the frame)
        for (int i = 0; i < (int)vessels.size(); i++) {
            Vessel& v = vessels[i];
            if (v.destroyed || v.landed) continue;
            const Body& b = U.bodies[v.ref];
            if (b.atmTop > 0 && len(v.r) - b.radius < b.atmTop) continue;
            attitudeOnRails(*this, v, 0, true, i == active);
        }
    }
    if (warpTo > 0 && t >= warpTo - 1e-6) {
        warpTo = -1;
        warpToRef = -1;
        warpIndex = 0;
        toast("Arrived at the selected point - time warp x1", {150, 200, 255, 255});
        if (activeVessel()) event(*activeVessel(), SimEvent::WarpArrived);
    }

    // Ascent guidance: phases and throttle (boost -> coast -> orbit insertion -> cut-off)
    if (Vessel* a = activeVessel(); a && !a->destroyed) {
        if (a->sas != SAS::Ascent || a->landed) {
            a->ascentPhase = 0;
            if (a->landed) a->ascentLoft = 0;
        }
        else {
            const Body& b = U.bodies[a->ref];
            Conic c = stateToConic(a->r, a->v, b.GM);
            const double hT = ascentTargetAlt(b);
            double alt = len(a->r) - b.radius, vz = dot(a->v, norm(a->r));
            bool closed = c.valid && c.e < 1;
            double apAlt = closed ? c.ap - b.radius : 1e12;
            bool inAir = b.atmTop > 0 && alt < b.atmTop;
            const Rgba8 note{150, 220, 255, 255};
            switch (a->ascentPhase) {
            case 0:
                if (apAlt >= hT + 3e3 && alt > (b.atmTop > 0 ? b.atmTop * 0.25 : 1000.0)) {
                    a->ascentPhase = 1;
                    a->throttle = 0;
                    toast("Ascent guidance: apoapsis reached - engine off, coasting", note);
                }
                break;
            case 1: {
                double tAp = closed && vz > 0 ? timeToTrueAnomaly(c, PI) : 0;
                if (inAir && vz > 0 && tAp > 60 && apAlt < hT - 4e3) { // the air took some of it back
                    a->ascentPhase = 0;
                    a->throttle = 1;
                    break;
                }
                double amt[PROP_N], cap[PROP_N], sa, sc, F, mdot;
                stageResources(*a, amt, cap, sa, sc, F, mdot);
                double dv = closed ? std::sqrt(b.GM / c.ap) - std::sqrt(std::max(0.0, b.GM * (2 / c.ap - 1 / c.a))) : 0;
                double burn = F > 0 && mdot > 0 ? vesselMass(*a) * (1 - std::exp(-dv * mdot / F)) / mdot : 0;
                if (vz <= 0 || tAp <= std::max(45.0, burn / 2)) {
                    a->ascentPhase = 2;
                    a->throttle = 1;
                    if (!a->ignited) stage(*a); // a stage dropped during the coast: light the next one
                    toast("Ascent guidance: orbit insertion burn", note);
                }
                break;
            }
            default: {
                // the periapsis only has to be safely above the air: the hold sags a little below the target
                double peCut = b.atmTop > 0 ? std::max(b.atmTop + 15e3, hT - 30e3) : 0.7 * hT;
                if (closed && c.pe - b.radius >= peCut) {
                    a->throttle = 0;
                    a->sas = SAS::Prograde;
                    a->ascentPhase = 0;
                    toast(trf("Orbit insertion complete: engine cut-off ({0} x {1})", formatDist(c.pe - b.radius), formatDist(c.ap - b.radius)), {150, 255, 170, 255});
                }
                break;
            }
            }
        }
    }

    // Karman line (100 km above Earth's surface): the conventional edge of space
    if (Vessel* a = activeVessel()) {
        const Body& b = U.bodies[a->ref];
        double alt = len(a->r) - b.radius;
        if (!a->destroyed && b.name == "Earth" && a->prevAlt >= 0 && a->prevAlt < 100000.0 && alt >= 100000.0) {
            toast("Crossed the Karman line (100 km) - welcome to space!", {255, 214, 102, 255});
            event(*a, SimEvent::Karman);
            diag::breadcrumb("karman line: %s", a->name.c_str());
        }
        a->prevAlt = a->destroyed ? -1 : alt;

        // captured into / escaping from the current body (hysteresis around e = 1)
        if (!a->destroyed && !a->landed) {
            const Body& gb = U.bodies[a->ref];
            Conic gc = stateToConic(a->r, a->v, gb.GM);
            double soi = gb.parent >= 0 ? gb.soi : 1e300;
            int now = -1;
            if (gc.valid) {
                if (gc.e < 0.999 && gc.ap < soi * 0.999) now = 1;
                else if (gc.e > 1.001 || gc.ap > soi * 1.001 || gc.ap < 0) now = 0;
            }
            if (now >= 0) {
                if (a->bound >= 0 && now != a->bound) {
                    if (now == 1) toast(trf("Captured into orbit around {0}", tr(gb.name)), {150, 220, 255, 255});
                    else toast(trf("Escape trajectory: leaving the gravity of {0}", tr(gb.name)), {255, 214, 140, 255});
                    event(*a, SimEvent::Gravity);
                }
                a->bound = now;
            }
        }

        // orbit standards: announce a new class once the orbit has been stable (engines off) for a moment
        Conic c;
        OrbitClass cls = (a->destroyed || thrustPossible(*this, *a)) ? OrbitClass::Unknown : classifyOrbit(*this, *a, c);
        if (cls == OrbitClass::Unknown) a->pendingT = 0;
        else if (a->orbitClass == OrbitClass::Unknown) {
            a->orbitClass = cls; // first look (new or loaded vessel): no announcement
            a->orbitClassRef = a->ref;
        } else if (cls != a->orbitClass || a->ref != a->orbitClassRef) {
            if (cls != a->pendingClass) {
                a->pendingClass = cls;
                a->pendingT = 0;
            }
            a->pendingT += realDt;
            if (a->pendingT > 1.5) {
                a->orbitClass = cls;
                a->orbitClassRef = a->ref;
                const Body& ob = U.bodies[a->ref];
                bool earth = ob.name == "Earth";
                std::string what;
                if (cls == OrbitClass::Low) what = earth ? tr("Low Earth orbit (LEO) reached") : trf("Low orbit around {0} reached", tr(ob.name));
                else if (cls == OrbitClass::High) {
                    OrbitBands bb = orbitBands(ob);
                    if (earth) what = tr(c.ap - ob.radius < bb.syncAlt ? "Medium Earth orbit (MEO) reached" : "High Earth orbit (HEO) reached");
                    else what = trf("High orbit around {0} reached", tr(ob.name));
                } else if (cls == OrbitClass::Synchronous) {
                    Vec3d spin = norm(U.angVel(a->ref));
                    double incEq = std::acos(std::clamp(std::fabs(dot(c.H, spin)), 0.0, 1.0));
                    if (earth) what = tr(incEq < 1.0 * DEG && c.e < 0.01 ? "Geostationary orbit (GEO) reached" : "Geosynchronous orbit reached");
                    else what = trf("Synchronous orbit around {0} reached", tr(ob.name));
                }
                if (!what.empty()) {
                    toast(what + "  ·  " + trf("Pe {0}", formatDist(c.pe - ob.radius)) + "  ·  " + trf("Ap {0}", formatDist(c.ap - ob.radius)),
                          {125, 211, 252, 255});
                    event(*a, SimEvent::OrbitReached);
                    diag::breadcrumb("orbit class %d around %s", (int)cls, ob.name.c_str());
                }
            }
        } else a->pendingT = 0;
    }

    // flaps, payload bay doors and radiators (~1 minute to open or close): game time (they finish during a time warp)
    {
        double gdt = animT < 0 ? 0.0 : clampd(t - animT, 0, 3600);
        animT = t;
        for (auto& bv : vessels) {
            // flaps move at the actuators' rate (full travel in ~12 s, at least 3 degrees per second)
            if (bv.hasFlaps && !bv.destroyed) {
                double a = flapAngleAt(bv.flapPos, bv.flapMax), tg = flapSettingDeg(bv.flapStage, bv.flapMax);
                double st = std::max(3.0, bv.flapMax / 12.0) * gdt;
                a = a < tg ? std::min(tg, a + st) : std::max(tg, a - st);
                bv.flapPos = flapPosOf(a, bv.flapMax);
            }
            if (bv.bayComp < 0 || bv.destroyed) continue;
            double tg = bv.bayOpen ? 1.0 : 0.0, st = gdt / 60.0;
            bv.bayT = bv.bayT < tg ? std::min(tg, bv.bayT + st) : std::max(tg, bv.bayT - st);
        }
    }
    // deploy animations (real time: they play at the same pace at any time warp)
    for (auto& av2 : vessels) {
        auto anim = [&](float& x, bool on, float secs) {
            float tg = on ? 1.f : 0.f;
            if (x < 0) { x = tg; return; }
            float st = (float)(realDt / secs);
            x = x < tg ? std::min(tg, x + st) : std::max(tg, x - st);
        };
        anim(av2.solarT, av2.solarDeployed, 6.0f);
        anim(av2.legsT, av2.legsDeployed, 2.5f);
        anim(av2.sailT, av2.sailDeployed, 10.0f);
    }

    // payload fairings come off once the air is thin (~110 km on Earth); the halves become debris
    for (size_t i = 0; i < vessels.size(); i++) {
        Vessel& fv = vessels[i];
        if (!fv.hasFairing || fv.destroyed || fv.landed || fv.debris) continue;
        const Body& fb = U.bodies[fv.ref];
        double alt = len(fv.r) - fb.radius;
        if (fb.atmTop > 0 && alt < 0.8 * fb.atmTop) continue;
        jettisonFairing(fv, true);
    }

    // achievements of this map (before the milestones: one notification, not two)
    bool earned = checkAchievements();
    // exploration log
    for (auto& v : vessels) {
        if (v.destroyed) continue;
        int b = v.ref;
        unsigned before = explored[b];
        explored[b] |= 1;
        const Body& bd = U.bodies[b];
        if (v.landed) explored[b] |= 4;
        else {
            Conic c = stateToConic(v.r, v.v, bd.GM);
            if (c.valid && c.e < 1 && c.pe > bd.radius + std::max(0.0, bd.atmTop) && c.ap < bd.soi) explored[b] |= 2;
        }
        unsigned gained = explored[b] & ~before;
        if (gained && !earned) {
            const char* what = (gained & 4) ? "Milestone - Landed on {0}" : (gained & 2) ? "Milestone - Achieved orbit around {0}" : "Milestone - First visit: {0}";
            toast(trf(what, tr(bd.name)), {255, 214, 102, 255});
        }
    }
}

void World::trimMemory(bool hard) {
    // release aero tables nobody needs right now (rebuilt lazily when the vessel enters an atmosphere)
    for (size_t i = 0; i < vessels.size(); i++) {
        Vessel& v = vessels[i];
        if ((int)i == active && !hard) continue;
        const Body& b = U.bodies[v.ref];
        bool inAtm = b.atmTop > 0 && len(v.r) - b.radius < b.atmTop * 1.2;
        if (!inAtm || v.destroyed || v.landed) v.aero.reset();
    }
    // forget long-gone debris
    if (hard) {
        int act = active;
        std::vector<Vessel> keep;
        for (size_t i = 0; i < vessels.size(); i++) {
            if ((int)i == act) act = (int)keep.size();
            else if (vessels[i].destroyed) continue;
            keep.push_back(std::move(vessels[i]));
        }
        vessels = std::move(keep); // (every vessel was moved into keep, removed or not)
        vessels.reserve(vessels.size() + 64);
        active = act;
    }
    for (Vessel& v : vessels) {
        v.comps.shrink_to_fit();
        v.phys.shrink_to_fit();
    }
    toasts.shrink_to_fit();
    if (history.size() > 2) {
        // hard: keep only the launch state and the newest; soft: drop every second state in between
        std::deque<WorldSnapshot> kept;
        for (size_t i = 0; i < history.size(); i++)
            if (i == 0 || i + 1 == history.size() || (!hard && i % 2 == 0)) kept.push_back(std::move(history[i]));
        history = std::move(kept);
    }
}

void World::setSolarDeployed(Vessel& v, bool on) {
    if (on && v.hasFairing) {
        if (&v == activeVessel()) toast("The payload fairing is still on: panels deploy after it is jettisoned", {255, 200, 120, 255});
        return;
    }
    if (v.solarAreaMax <= 0 || v.solarDeployed == on || v.destroyed) return;
    v.solarDeployed = on;
    const Body& b = U.bodies[v.ref];
    bool inAtm = b.atmTop > 0 && len(v.r) - b.radius < b.atmTop;
    if (!inAtm) v.aero.reset(); // rebuilt lazily when the vessel enters an atmosphere
    setupVessel(v, inAtm);
    toast(on ? "Solar panels deployed" : "Solar panels retracted", {150, 200, 255, 255});
}

void World::stage(Vessel& v) {
    if (v.comps.empty() || v.destroyed) return;
    diag::breadcrumb("stage: %s (%d comps)", v.name.c_str(), (int)v.comps.size());
    bool hasEngines = false;
    for (size_t i = 0; i < v.comps.size(); i++)
        if (engineFires(v, v.comps[i]) && ((v.comps[i].func == Func::Engine && v.phys[i].thrustVac > 0) || v.comps[i].func == Func::Rotor || v.comps[i].func == Func::Jet))
            hasEngines = true;
    if (!v.ignited && hasEngines) {
        v.ignited = true;
        event(v, SimEvent::Ignite);
        toast("Stage: engines ignited", {255, 190, 110, 255});
        return;
    }
    auto stages = vesselStages(v);
    if (stages.size() <= 1) {
        toast("No more stages", {200, 200, 210, 255});
        return;
    }
    // jettison the active stage group (it flies on as debris: spent stages, side boosters)
    int oldStage = v.stage;
    Vec3d oldCom = v.mp.com;
    std::vector<Component> nc, dc;
    std::vector<double> np, nh, dp, dh;
    for (size_t i = 0; i < v.comps.size(); i++) {
        if (v.comps[i].stage == v.stage) {
            dc.push_back(v.comps[i]);
            dp.push_back(i < v.prop.size() ? v.prop[i] : 0.0);
            dh.push_back(i < v.heat.size() ? v.heat[i] : 0.0);
            continue;
        }
        nc.push_back(v.comps[i]);
        np.push_back(v.prop[i]);
        nh.push_back(v.heat[i]);
    }
    Vessel before = v; // (state at the separation, for the debris)
    v.comps = nc;
    v.prop = np;
    v.heat = nh;
    setupVessel(v, true);
    followCoM(*this, v, oldCom, t);
    if (!before.landed && !dc.empty()) {
        char lbl[48];
        std::snprintf(lbl, sizeof lbl, "%d", oldStage + 1);
        releaseDebris(before, dc, dp, dh, oldCom, lbl);
    }
    bool newHas = false;
    for (size_t i = 0; i < v.comps.size(); i++)
        if (v.comps[i].stage == v.stage && v.comps[i].func == Func::Engine && v.phys[i].thrustVac > 0) newHas = true;
    // the next stage starts with its engines off (lit by the next staging command), unless they were
    // already burning beside the side boosters that just separated
    bool burning = false;
    if (v.ignited)
        for (const Component& c : v.comps)
            if (c.func == Func::Engine && engineFires(v, c) && c.fireFrom >= 0 && c.fireFrom <= oldStage) burning = true;
    v.ignited = burning;
    if (!burning) std::fill(v.engLevel.begin(), v.engLevel.end(), 0.f);
    event(v, SimEvent::Stage);
    if (burning) toast("Side boosters separated - core stage still burning", {255, 190, 110, 255});
    else toast(newHas ? "Stage separation - next stage engines off (Space to ignite)" : "Stage separation", {255, 190, 110, 255});
}

int World::launch(const Craft& craft) {
    diag::breadcrumb("launch: %s (%d comps)", craft.name.c_str(), (int)craft.comps.size());
    // the pad: an Earth launch site, or the launch complex of a surface base
    PadSite pad = padSite();
    int padBody = pad.body;
    Vec3d upF = pad.up;
    for (int i = (int)vessels.size() - 1; i >= 0; i--) {
        Vessel& o = vessels[i];
        if (o.destroyed || (o.landed && o.ref == padBody && len(o.landPos - upF * len(o.landPos)) < 500)) {
            vessels.erase(vessels.begin() + i);
            if (active == i) active = -1;
            else if (active > i) active--;
        }
    }
    Vessel v;
    v.id = nextId++;
    v.name = craft.name;
    v.comps = expandCraft(craft);
    setupVessel(v, true);
    v.charge = v.chargeCap;
    v.ref = padBody;
    const Body& e = U.bodies[padBody];
    double zmin = 0;
    for (size_t k = 0; k < v.contacts.size(); k++) zmin = std::min(zmin, v.contacts[k].z - v.mp.com.z);
    double h = -zmin + 0.05;
    // on the launch mount (the Earth sites stand on levelled ground at the datum; a base's pad on its own ground)
    double ground = pad.baseId > 0 ? ground::height(padBody, upF) : 0.0;
    v.landPos = upF * (e.radius + ground + pad.deckH + h);
    padRocketLen = std::max(5.0, v.length);
    padRocketR = std::max(0.5, v.maxR);
    Vec3d east = norm(cross(Vec3d{0, 0, 1}, upF));
    v.landQ = Quatd::lookRot(upF, -east);
    v.landed = true;
    v.legsDeployed = true;
    v.launchTime = t;
    v.sas = SAS::Stability;
    v.earthLaunch = true;                   // (a real launch, from Earth or a base: counts for achievements; other starts clear it)
    v.cheated = settings.infiniteFuel;
    pinLanded(*this, v, t);
    if (v.hasFairing) v.legsDeployed = false; // landing legs of a payload are folded inside the fairing
    if (v.gearRetract) v.legsDeployed = false; // the landing gear of a winged orbiter rides up in its bays
    pruneDebris(40); // (the oldest debris of earlier flights go: a bounded number stays in the sky)
    vessels.reserve(vessels.size() + 64);  // fairing halves and spent stages are appended without reallocation
    vessels.push_back(std::move(v));
    active = (int)vessels.size() - 1;
    warpIndex = 0;
    history.clear(); // "rewind" never goes back before this launch
    recordHistory(true);
    return active;
}

int World::launchAircraft(const Craft& craft, int body) {
    // where: the runway south of the launch complex (Earth), or a flat landing site elsewhere
    struct Site { const char* body; double lat, lon; };
    static const Site SITES[] = {{"Moon", 26.13, 3.63},    // Hadley-Apennine (Apollo 15, the first lunar rover)
                                 {"Mars", 18.4, 77.5},     // Jezero crater (Perseverance / Ingenuity)
                                 {"Titan", -10.3, 167.6},  // Huygens landing site
                                 {"Venus", -7.0, -26.0}};  // (a cloud-top balloon world: 92 bar, 460 C)
    int earth = U.find("Earth");
    if (body < 0) body = earth;
    int idx = launch(craft); // builds the vessel (and clears old pad vessels)
    if (idx < 0 || idx >= (int)vessels.size()) return idx;
    Vessel& v = vessels[idx];
    const Body& b = U.bodies[body];
    v.earthLaunch = body == earth; // a rover on the Moon, a helicopter on Mars: started there, no achievements
    Vec3d upF, east, north;
    if (body == earth) {
        const LaunchSite& site = LAUNCH_SITES[std::clamp(settings.launchSite, 0, LAUNCH_SITE_COUNT - 1)];
        double lat = site.latDeg * DEG, lon = site.lonDeg * DEG;
        Vec3d pad{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
        east = norm(cross(Vec3d{0, 0, 1}, pad));
        north = cross(pad, east);
        // runway: 600 m south of the pad, the start 1,300 m west of it
        upF = norm(pad * b.radius + east * RUNWAY_START_X + north * RUNWAY_Y);
    } else {
        double lat = 0, lon = 0;
        for (const Site& s : SITES)
            if (b.name == s.body) { lat = s.lat * DEG; lon = s.lon * DEG; }
        upF = {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
        east = norm(cross(Vec3d{0, 0, 1}, upF));
        north = cross(upF, east);
    }
    v.ref = body;
    double ground = b.radius + ground::height(body, upF);
    Quatd q;
    double low = 0;
    if (v.hasWheels && !v.rotorcraft) {
        // nose (+Z) east along the runway, top (+Y) up; the lowest wheel on the ground
        q = Quatd::lookRot(east, upF);
        for (size_t k = 0; k < v.contacts.size(); k++) low = std::min(low, dot(q.rotate(v.contacts[k] - v.mp.com), upF));
    } else {
        // rotorcraft stand upright (+Z up)
        q = Quatd::lookRot(upF, -east);
        for (size_t k = 0; k < v.contacts.size(); k++) low = std::min(low, dot(q.rotate(v.contacts[k] - v.mp.com), upF));
    }
    v.landPos = upF * (ground - low + 0.02);
    v.landQ = q;
    v.landed = true;
    v.rolling = false;
    v.legsDeployed = true;
    v.sas = SAS::Stability;
    v.throttle = 0;
    // aircraft: the motors are armed, the throttle alone flies them (not when a solid motor would light at once)
    v.ignited = true;
    for (const Component& c : v.comps)
        if (engineFires(v, c) && c.func == Func::Engine && c.def >= 0 && CATALOG.defs[c.def].kind == PK::SRB) v.ignited = false;
    pinLanded(*this, v, t);
    history.clear();
    recordHistory(true);
    return idx;
}

void World::jettisonFairing(Vessel& v, bool debris) {
    std::vector<Component> keep, half[2];
    std::vector<double> kp, kh;
    for (size_t i = 0; i < v.comps.size(); i++) {
        const Component& c = v.comps[i];
        if (c.func == Func::Fairing) half[std::cos(c.rotDeg.z * DEG) >= 0 ? 0 : 1].push_back(c);
        else {
            keep.push_back(c);
            kp.push_back(i < v.prop.size() ? v.prop[i] : 0.0);
            kh.push_back(i < v.heat.size() ? v.heat[i] : 0.0);
        }
    }
    if (half[0].empty() && half[1].empty()) {
        v.hasFairing = false;
        return;
    }
    Vec3d oldCom = v.mp.com, r0 = v.r, v0 = v.v, w0 = v.w;
    Quatd q0 = v.q;
    int ref = v.ref, vid = v.id;
    std::string name = v.name;
    v.comps = keep;
    v.prop = kp;
    v.heat = kh;
    setupVessel(v, true);
    followCoM(*this, v, oldCom, t);
    bool isActive = activeVessel() && activeVessel()->id == vid;
    if (isActive && debris) toast("Payload fairing jettisoned", {200, 220, 255, 255}); // (not for starts in orbit)
    if (!debris) return;
    // the halves: pushed apart sideways, tumbling outward. No reallocation of 'vessels' here, so
    // references held by callers stay valid (without room the halves are simply dropped)
    std::vector<Vessel> out;
    for (int s = 0; s < 2; s++) {
        if (half[s].empty()) continue;
        Vessel d;
        d.name = name + " " + tr("fairing");
        d.debris = true;
        d.comps = half[s];
        setupVessel(d, true);
        d.ref = ref;
        d.q = q0;
        d.r = r0 + q0.rotate(d.mp.com - oldCom);
        double sgn = s == 0 ? 1.0 : -1.0;
        d.v = v0 + q0.rotate(Vec3d{sgn * 3.5, 0, 0});
        d.w = w0 + Vec3d{0, sgn * 0.35, 0};
        d.landed = false;
        d.throttle = 0;
        d.sas = SAS::Off;
        d.launchTime = t;
        d.charge = 0;
        out.push_back(std::move(d));
    }
    if (vessels.size() + out.size() <= vessels.capacity())
        for (auto& d : out) {
            d.id = nextId++;
            vessels.push_back(std::move(d));
        }
}

void World::releaseDebris(const Vessel& parent, const std::vector<Component>& comps, const std::vector<double>& prop,
                          const std::vector<double>& heat, const Vec3d& oldCom, const std::string& label) {
    int n = (int)comps.size();
    // pieces: components joined by touching bounding boxes (a craft-axes box around each, 0.3 m of slack)
    std::vector<Vec3d> lo(n), hi(n);
    for (int i = 0; i < n; i++) {
        Vec3d he = shapeHalfExtent(comps[i].shape, comps[i].dims);
        Mat3d R = comps[i].rot().toMat();
        Vec3d e{0, 0, 0};
        for (int a = 0; a < 3; a++) {
            Vec3d col{R.m[0][a], R.m[1][a], R.m[2][a]};
            double h = a == 0 ? he.x : a == 1 ? he.y : he.z;
            e.x += std::fabs(col.x) * h;
            e.y += std::fabs(col.y) * h;
            e.z += std::fabs(col.z) * h;
        }
        Vec3d c = comps[i].pos;
        lo[i] = c - e - Vec3d{0.3, 0.3, 0.3};
        hi[i] = c + e + Vec3d{0.3, 0.3, 0.3};
    }
    std::vector<int> grp(n);
    for (int i = 0; i < n; i++) grp[i] = i;
    std::function<int(int)> root = [&](int i) { return grp[i] == i ? i : grp[i] = root(grp[i]); };
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
            if (lo[i].x <= hi[j].x && lo[j].x <= hi[i].x && lo[i].y <= hi[j].y && lo[j].y <= hi[i].y && lo[i].z <= hi[j].z && lo[j].z <= hi[i].z)
                grp[root(i)] = root(j);
    std::vector<int> roots;
    for (int i = 0; i < n; i++)
        if (std::find(roots.begin(), roots.end(), root(i)) == roots.end()) roots.push_back(root(i));
    if (vessels.size() + roots.size() > vessels.capacity()) return; // (no room without moving the vessels)
    for (size_t k = 0; k < roots.size(); k++) {
        Vessel d;
        d.debris = true;
        d.name = parent.name + " " + (roots.size() > 1 ? trf("stage {0} piece {1}", label, std::to_string(k + 1)) : trf("stage {0}", label));
        for (int i = 0; i < n; i++)
            if (root(i) == roots[k]) {
                d.comps.push_back(comps[i]);
                d.prop.push_back(i < (int)prop.size() ? prop[i] : 0.0);
                d.heat.push_back(i < (int)heat.size() ? heat[i] : 0.0);
            }
        setupVessel(d, true);
        // (setupVessel may refill the tanks: restore what was left in them)
        {
            size_t m = 0;
            std::vector<double> keepP;
            for (int i = 0; i < n; i++)
                if (root(i) == roots[k]) keepP.push_back(i < (int)prop.size() ? prop[i] : 0.0);
            for (; m < keepP.size() && m < d.prop.size(); m++) d.prop[m] = keepP[m];
            updateMassProps(d);
        }
        d.ref = parent.ref;
        d.q = parent.q;
        Vec3d off = d.mp.com - oldCom;
        d.r = parent.r + parent.q.rotate(off);
        // pushed away from the vessel (separation springs / booster separation motors), keeping the spin
        Vec3d away = len(off) > 0.05 ? norm(off) : Vec3d{0, 0, -1};
        double push = std::fabs(away.z) > 0.7 ? 1.2 : 3.0; // (a stage below: slowly; side boosters: outward)
        d.v = parent.v + parent.q.rotate(away * push);
        d.w = parent.w + Vec3d{away.y, -away.x, 0} * 0.08;
        d.landed = false;
        d.throttle = 0;
        d.sas = SAS::Off;
        d.launchTime = t;
        d.charge = 0;
        // a solid booster that still holds propellant goes on burning (it cannot be shut down)
        for (size_t i = 0; i < d.comps.size(); i++)
            if (d.comps[i].func == Func::Engine && d.comps[i].def >= 0 && CATALOG.defs[d.comps[i].def].kind == PK::SRB && d.prop[i] > 0) d.ignited = true;
        d.id = nextId++;
        vessels.push_back(std::move(d));
    }
}

// ------------------------------------------------------------------ docking
void World::portFrame(const Vessel& v, int i, Vec3d& pos, Vec3d& dir) const {
    const Component& c = v.comps[i];
    Mat3d R = c.rot().toMat();
    Vec3d face = c.pos + R * Vec3d{0, 0, c.dims.z / 2};
    pos = v.r + v.q.rotate(face - v.mp.com);
    dir = v.q.rotate(R * Vec3d{0, 0, 1});
}

void World::checkDocking() {
    if (t < dockHoldUntil) return;
    Vessel* a = activeVessel();
    if (!a || a->destroyed || a->landed) return;
    for (int ia = 0; ia < (int)a->comps.size(); ia++) {
        if (a->comps[ia].func != Func::Docking || a->comps[ia].dockedTo != 0) continue;
        Vec3d pa, da;
        portFrame(*a, ia, pa, da);
        for (Vessel& b : vessels) {
            if (&b == a || b.destroyed || b.debris || b.landed || b.ref != a->ref) continue;
            if (len2(b.r - a->r) > 400.0 * 400.0) continue;
            for (int ib = 0; ib < (int)b.comps.size(); ib++) {
                if (b.comps[ib].func != Func::Docking || b.comps[ib].dockedTo != 0) continue;
                Vec3d pb, db;
                portFrame(b, ib, pb, db);
                double gap = len(pb - pa);
                if (gap > 0.35 || dot(da, db) > -std::cos(12 * DEG)) continue;
                // the ports' own velocities (translation and rotation)
                Vec3d va = a->v + a->q.rotate(cross(a->w, a->q.conj().rotate(pa - a->r)));
                Vec3d vb = b.v + b.q.rotate(cross(b.w, b.q.conj().rotate(pb - b.r)));
                if (len(va - vb) > 0.5) continue;
                // ---- latch: b becomes part of a, aligned face to face, momentum kept
                Vessel& A = *a;
                Quatd fix = Quatd{};
                {
                    Vec3d from = db, to = -da;
                    Vec3d ax = cross(from, to);
                    double s = len(ax), cth = clampd(dot(from, to), -1, 1);
                    if (s > 1e-9) fix = Quatd::axisAngle(ax / s, std::atan2(s, cth));
                }
                Quatd qB = (fix * b.q).normalized();
                Vec3d faceB = b.comps[ib].pos + b.comps[ib].rot().toMat() * Vec3d{0, 0, b.comps[ib].dims.z / 2};
                double mA = A.mp.mass, mB = b.mp.mass;
                Vec3d rC = (A.r * mA + b.r * mB) / (mA + mB), vC = (A.v * mA + b.v * mB) / (mA + mB);
                // angular momentum about the joint centre of mass (world axes)
                Vec3d LA = A.q.rotate(A.mp.inertia * A.w) + cross(A.r - rC, (A.v - vC) * mA);
                Vec3d LB = b.q.rotate(b.mp.inertia * b.w) + cross(b.r - rC, (b.v - vC) * mB);
                // port tags: unique per port (64-bit arithmetic: no overflow for any id)
                auto tagOf = [](int id, int k) { return (int)(((long long)std::max(id, 0) * 4096 + k) % 2000000000LL) + 1; };
                if (A.comps[ia].dockTag == 0) A.comps[ia].dockTag = tagOf(A.id, ia);
                if (b.comps[ib].dockTag == 0) b.comps[ib].dockTag = tagOf(b.id, ib + 2048);
                A.comps[ia].dockedTo = b.comps[ib].dockTag;
                b.comps[ib].dockedTo = A.comps[ia].dockTag;
                int top = 0;
                for (Component& c : A.comps) {
                    top = std::max(top, c.stage);
                    if (c.origin == 0) c.origin = A.id;
                }
                Vec3d oldCom = A.mp.com;
                Quatd toA = (A.q.conj() * qB).normalized();
                for (size_t k = 0; k < b.comps.size(); k++) {
                    Component c = b.comps[k];
                    Vec3d world = pa + qB.rotate(c.pos - faceB);
                    c.pos = oldCom + A.q.conj().rotate(world - A.r);
                    c.rotDeg = eulerDegOf((toA * c.rot()).toMat());
                    if (c.origin == 0) c.origin = b.id;
                    if (c.stage0 < 0) c.stage0 = c.stage;
                    c.stage = top; // (with the host's last group: staging never drops a docked module)
                    A.comps.push_back(c);
                    A.prop.push_back(k < b.prop.size() ? b.prop[k] : 0.0);
                    A.heat.push_back(k < b.heat.size() ? b.heat[k] : 0.0);
                }
                const Body& bd = U.bodies[A.ref];
                setupVessel(A, bd.atmTop > 0 && len(A.r) - bd.radius < bd.atmTop);
                A.r = A.r + A.q.rotate(A.mp.com - oldCom);
                A.v = vC;
                A.w = A.invI * A.q.conj().rotate(LA + LB);
                awardDocking(A, b);
                // the joint vessel: counts only if both did, and has done what either did
                A.earthLaunch = A.earthLaunch && b.earthLaunch;
                A.cheated = A.cheated || b.cheated;
                A.reachedSpace = A.reachedSpace || b.reachedSpace;
                A.landedOn |= b.landedOn;
                A.orbited |= b.orbited;
                b.destroyed = true;
                b.debris = true; // (no crash report, not drawn)
                b.fate = "Docked";
                if (A.targetVessel == b.id) A.targetVessel = -1;
                moduleNames[b.id] = b.name;
                if (!moduleNames.count(A.id)) moduleNames[A.id] = A.name;
                {
                    // the joint vessel is named after its heaviest module (a ferry docking to a station)
                    std::map<int, double> mo;
                    for (size_t i = 0; i < A.comps.size() && i < A.phys.size(); i++) mo[A.comps[i].origin] += A.phys[i].dryMass;
                    int best = 0;
                    double bm = -1;
                    for (auto& e : mo)
                        if (e.second > bm) { bm = e.second; best = e.first; }
                    auto itn = moduleNames.find(best);
                    if (itn != moduleNames.end()) A.name = itn->second;
                }
                event(A, SimEvent::Stage);
                toast(trf("Docked with {0}", b.name), {140, 255, 170, 255});
                return; // one docking per frame
            }
        }
    }
}

bool World::undock(Vessel& v, int port) {
    if (port < 0 || port >= (int)v.comps.size() || v.comps[port].func != Func::Docking || v.comps[port].dockedTo == 0) return false;
    if (vessels.size() + 1 > vessels.capacity()) {
        toast("Cannot undock now", {255, 170, 90, 255});
        return false;
    }
    int n = (int)v.comps.size();
    int partner = -1;
    for (int i = 0; i < n; i++)
        if (v.comps[i].func == Func::Docking && v.comps[i].dockTag == v.comps[port].dockedTo) partner = i;
    // the modules joined by the docked port pairs, without this one
    auto originOf = [&](int i) { return v.comps[i].origin; };
    std::vector<std::pair<int, int>> edges;
    for (int i = 0; i < n; i++) {
        if (v.comps[i].func != Func::Docking || v.comps[i].dockedTo == 0 || i == port || i == partner) continue;
        for (int j = i + 1; j < n; j++)
            if (v.comps[j].func == Func::Docking && v.comps[j].dockTag == v.comps[i].dockedTo) edges.push_back({originOf(i), originOf(j)});
    }
    // the side of the partner port
    std::vector<int> side{partner >= 0 ? originOf(partner) : -12345};
    for (bool grew = true; grew;) {
        grew = false;
        for (auto& e : edges) {
            bool a = std::find(side.begin(), side.end(), e.first) != side.end(), b = std::find(side.begin(), side.end(), e.second) != side.end();
            if (a != b) {
                side.push_back(a ? e.second : e.first);
                grew = true;
            }
        }
    }
    if (std::find(side.begin(), side.end(), originOf(port)) != side.end()) return false; // (a ring of docked modules)
    std::vector<int> inB;
    double mSide = 0, mAll = 0;
    for (int i = 0; i < n; i++) {
        mAll += v.phys[i].dryMass + (i < (int)v.prop.size() ? v.prop[i] : 0.0);
        if (std::find(side.begin(), side.end(), originOf(i)) != side.end()) {
            inB.push_back(i);
            mSide += v.phys[i].dryMass + (i < (int)v.prop.size() ? v.prop[i] : 0.0);
        }
    }
    if (inB.empty() || (int)inB.size() == n) return false;
    bool lighterLeaves = mSide <= mAll - mSide;
    // the part that leaves: the lighter one (a ferry leaves the station)
    std::vector<bool> leaves(n, false);
    for (int i : inB) leaves[i] = lighterLeaves;
    if (!lighterLeaves)
        for (int i = 0; i < n; i++) leaves[i] = std::find(inB.begin(), inB.end(), i) == inB.end();
    v.comps[port].dockedTo = 0;
    if (partner >= 0) v.comps[partner].dockedTo = 0;
    Vessel nv;
    std::vector<Component> keep;
    std::vector<double> kp, kh;
    int leaveOrigin = 0;
    for (int i = 0; i < n; i++) {
        Component c = v.comps[i];
        double pr = i < (int)v.prop.size() ? v.prop[i] : 0.0, ht = i < (int)v.heat.size() ? v.heat[i] : 0.0;
        if (leaves[i]) {
            if (c.stage0 >= 0) { c.stage = c.stage0; c.stage0 = -1; }
            if (!leaveOrigin) leaveOrigin = c.origin;
            nv.comps.push_back(c);
            nv.prop.push_back(pr);
            nv.heat.push_back(ht);
        } else {
            keep.push_back(c);
            kp.push_back(pr);
            kh.push_back(ht);
        }
    }
    Vec3d sepDir = leaves[port] ? -v.q.rotate(v.comps[port].rot().toMat() * Vec3d{0, 0, 1}) : v.q.rotate(v.comps[port].rot().toMat() * Vec3d{0, 0, 1});
    if (partner >= 0 && leaves[partner]) sepDir = v.q.rotate(v.comps[port].rot().toMat() * Vec3d{0, 0, 1});
    Vec3d oldCom = v.mp.com;
    Vessel host = v;
    setupVessel(nv, false);
    nv.ref = v.ref;
    nv.q = v.q;
    Vec3d off = nv.mp.com - oldCom;
    nv.r = v.r + v.q.rotate(off);
    nv.v = v.v + v.q.rotate(cross(v.w, off)) + sepDir * 0.12; // (the latches' springs)
    nv.w = v.w;
    nv.charge = v.charge * std::min(1.0, nv.chargeCap / std::max(v.chargeCap, 1e-9));
    nv.ignited = false;
    nv.sas = SAS::Stability;
    nv.rcs = v.rcs;
    nv.launchTime = t;
    nv.earthLaunch = v.earthLaunch; // (both parts keep what the joint vessel did)
    nv.cheated = v.cheated;
    nv.reachedSpace = v.reachedSpace;
    nv.landedOn = v.landedOn;
    nv.orbited = v.orbited;
    auto it = moduleNames.find(leaveOrigin);
    nv.name = it != moduleNames.end() ? it->second : v.name + " " + tr("module");
    nv.id = nextId++;
    v.comps = keep;
    v.prop = kp;
    v.heat = kh;
    setupVessel(v, false);
    followCoM(*this, v, oldCom, t);
    v.v = v.v - sepDir * 0.12 * nv.mp.mass / std::max(v.mp.mass, 1.0);
    {
        // the part that stays is named after its heaviest module
        std::map<int, double> mo;
        for (size_t i = 0; i < v.comps.size(); i++) mo[v.comps[i].origin] += v.phys[i].dryMass;
        int best = 0;
        double bm = -1;
        for (auto& e : mo)
            if (e.second > bm) { bm = e.second; best = e.first; }
        auto itn = moduleNames.find(best);
        if (itn != moduleNames.end()) v.name = itn->second;
    }
    bool flyNew = &v == activeVessel() && nv.hasControl && !lighterLeaves ? false : (&v == activeVessel() && nv.hasControl);
    std::string nm = nv.name;
    vessels.push_back(std::move(nv));
    if (flyNew) active = (int)vessels.size() - 1;
    dockHoldUntil = t + 10; // (the springs push the two apart first)
    toast(trf("Undocked: {0}", nm), {150, 200, 255, 255});
    return true;
}

void World::cycleFlaps(Vessel& v) {
    if (!v.hasFlaps || v.destroyed) return;
    v.flapStage = (v.flapStage + 1) % (FLAP_STAGES + 1);
    if (&v == activeVessel()) {
        char b[32];
        std::snprintf(b, sizeof b, "%.1f", flapSettingDeg(v.flapStage, v.flapMax));
        toast(trf("Flaps {0} ({1} deg)", std::to_string(v.flapStage), b), {150, 200, 255, 255});
    }
}

// the bay's frame in craft axes: centre, rotation (local +Y: the doors), half sizes
static bool bayFrame(const Vessel& v, Vec3d& c, Mat3d& R, Vec3d& he) {
    if (v.bayComp < 0 || v.bayComp >= (int)v.comps.size()) return false;
    const Component& b = v.comps[v.bayComp];
    c = b.pos;
    R = b.rot().toMat();
    he = b.dims * 0.5;
    return true;
}
static bool inSpace(const World& W, const Vessel& v) {
    const Body& b = W.U.bodies[v.ref];
    return !v.landed && !v.rolling && (b.atmTop <= 0 || len(v.r) - b.radius > b.atmTop);
}

void World::setBayDoors(Vessel& v, bool open) {
    if (v.bayComp < 0 || v.destroyed) return;
    bool act = &v == activeVessel();
    if (open && !inSpace(*this, v)) {
        if (act) toast("The payload bay doors are opened in space only", {255, 200, 120, 255});
        return;
    }
    v.bayOpen = open;
    if (act) toast(open ? "Payload bay doors opening" : "Payload bay doors closing", {150, 200, 255, 255});
}

bool World::releasePayload(Vessel& v) {
    bool act = &v == activeVessel();
    auto say = [&](const char* s) {
        if (act) toast(s, {255, 200, 120, 255});
        return false;
    };
    if (v.bayComp < 0 || v.destroyed) return false;
    if (!inSpace(*this, v)) return say("The payload is released in space only");
    if (v.bayT < 0.999) return say("Open the payload bay doors first");
    if (vessels.size() + 1 > vessels.capacity()) return say("Cannot release now");
    int n = (int)v.comps.size();
    std::vector<bool> leaves(n, false);
    int cnt = 0;
    for (int i = 0; i < n; i++)
        if (v.comps[i].cargo && v.comps[i].func != Func::CargoBay) { leaves[i] = true; cnt++; }
    if (cnt == 0) return say("There is no payload in the bay");
    if (cnt == n) return false;
    Vec3d bc, bhe;
    Mat3d bR;
    bayFrame(v, bc, bR, bhe);
    Vessel nv;
    std::vector<Component> keep;
    std::vector<double> kp, kh;
    std::map<int, double> om; // (the payload is named after its heaviest module)
    for (int i = 0; i < n; i++) {
        Component c = v.comps[i];
        double pr = i < (int)v.prop.size() ? v.prop[i] : 0.0, ht = i < (int)v.heat.size() ? v.heat[i] : 0.0;
        if (leaves[i]) {
            if (c.stage0 >= 0) { c.stage = c.stage0; c.stage0 = -1; }
            om[c.origin] += v.phys[i].dryMass;
            nv.comps.push_back(c);
            nv.prop.push_back(pr);
            nv.heat.push_back(ht);
        } else {
            keep.push_back(c);
            kp.push_back(pr);
            kh.push_back(ht);
        }
    }
    {
        // the payload's own stage groups start at 0
        int s0 = 1 << 30;
        for (auto& c : nv.comps) s0 = std::min(s0, c.stage);
        for (auto& c : nv.comps) c.stage -= s0;
    }
    int bestO = 0;
    double bm = -1;
    for (auto& e : om)
        if (e.second > bm) { bm = e.second; bestO = e.first; }
    std::string nm;
    if (auto it = moduleNames.find(bestO); bestO != 0 && it != moduleNames.end()) nm = it->second;
    if (nm.empty()) {
        // the heaviest payload component's name
        double mm = -1;
        for (int i = 0; i < n; i++)
            if (leaves[i] && v.phys[i].dryMass > mm) { mm = v.phys[i].dryMass; nm = v.comps[i].name; }
    }
    Vec3d sepDir = v.q.rotate(bR * Vec3d{0, 1, 0}); // out of the bay (the springs of the cradle: 0.3 m/s)
    Vec3d oldCom = v.mp.com;
    setupVessel(nv, false);
    nv.ref = v.ref;
    nv.q = v.q;
    Vec3d off = nv.mp.com - oldCom;
    nv.r = v.r + v.q.rotate(off);
    nv.v = v.v + v.q.rotate(cross(v.w, off)) + sepDir * 0.3;
    nv.w = v.w;
    nv.charge = nv.chargeCap;
    nv.ignited = false;
    nv.sas = nv.hasControl ? SAS::Stability : SAS::Off;
    nv.launchTime = t;
    nv.earthLaunch = v.earthLaunch;
    nv.cheated = v.cheated;
    nv.reachedSpace = v.reachedSpace;
    nv.landedOn = v.landedOn;
    nv.orbited = v.orbited;
    nv.name = nm.empty() ? v.name + " " + tr("payload") : nm;
    nv.id = nextId++;
    for (auto& c : nv.comps) if (c.origin == 0) c.origin = nv.id;
    moduleNames[nv.id] = nv.name;
    v.comps = keep;
    v.prop = kp;
    v.heat = kh;
    setupVessel(v, false);
    followCoM(*this, v, oldCom, t);
    v.v = v.v - sepDir * 0.3 * nv.mp.mass / std::max(v.mp.mass, 1.0);
    std::string pn = nv.name;
    vessels.push_back(std::move(nv));
    event(v, SimEvent::Stage);
    toast(trf("Payload released: {0}", pn), {150, 200, 255, 255});
    return true;
}

// the payload's extent in its own craft axes (outline of every component)
static void craftBounds(const std::vector<Component>& comps, Vec3d& lo, Vec3d& hi) {
    lo = {1e30, 1e30, 1e30};
    hi = {-1e30, -1e30, -1e30};
    std::vector<Vec3d> pts;
    for (const Component& c : comps) {
        pts.clear();
        outlinePoints(c, pts, 12);
        for (const Vec3d& p : pts) {
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
            hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        }
    }
}
// a turn of the payload's axes that fits its extent into the bay (half sizes he, 3 % to spare): identity first,
// then the other axis orders (its longest side along the bay)
static bool fitInBay(const Vec3d& size, const Vec3d& he, Mat3d& P) {
    const Mat3d cand[6] = {Mat3d{},
                           Mat3d::fromCols({0, 1, 0}, {1, 0, 0}, {0, 0, -1}),
                           Mat3d::fromCols({1, 0, 0}, {0, 0, 1}, {0, -1, 0}),
                           Mat3d::fromCols({0, 0, 1}, {0, 1, 0}, {-1, 0, 0}),
                           Mat3d::fromCols({0, 1, 0}, {0, 0, 1}, {1, 0, 0}),
                           Mat3d::fromCols({0, 0, 1}, {1, 0, 0}, {0, 1, 0})};
    for (const Mat3d& m : cand) {
        Vec3d s = m * size;
        if (std::fabs(s.x) <= 2 * he.x * 0.97 && std::fabs(s.y) <= 2 * he.y * 0.97 && std::fabs(s.z) <= 2 * he.z * 0.97) {
            P = m;
            return true;
        }
    }
    return false;
}

// the remote manipulator arm: 15.2 m long; it reaches a vessel whose centre of mass is within ~15 m of the bay
// opening, moving slower than 0.3 m/s and turning slower than 2 degrees per second relative to the orbiter
static const double ARM_REACH = 15.2, ARM_DV = 0.3, ARM_DW = 2 * DEG;
int World::captureCandidate(const Vessel& v, std::string* why) const {
    Vec3d bc, bhe;
    Mat3d bR;
    if (!bayFrame(v, bc, bR, bhe) || v.destroyed) return -1;
    auto fail = [&](const char* s) {
        if (why) *why = s;
        return -1;
    };
    if (!inSpace(*this, v)) return fail("The arm works in space only");
    if (v.bayT < 0.999) return fail("Open the payload bay doors first");
    Vec3d mouth = v.r + v.q.rotate(bc + bR * Vec3d{0, bhe.y, 0} - v.mp.com);
    int best = -1;
    double bd = 1e30;
    for (int i = 0; i < (int)vessels.size(); i++) {
        const Vessel& b = vessels[i];
        if (&b == &v || b.destroyed || b.debris || b.landed || b.ref != v.ref) continue;
        double d = len(b.r - mouth);
        if (d < bd) { bd = d; best = i; }
    }
    if (best < 0 || bd > ARM_REACH) return fail("No vessel within the arm's reach (15 m)");
    const Vessel& b = vessels[best];
    if (len(b.v - v.v) > ARM_DV) return fail("Too fast: match the velocity to within 0.3 m/s");
    if (len(b.q.rotate(b.w) - v.q.rotate(v.w)) > ARM_DW) return fail("It turns too fast to be grappled");
    Vec3d lo, hi;
    craftBounds(b.geometry(), lo, hi); // (as it flies now: solar wings folded)
    Mat3d P;
    if (!fitInBay(hi - lo, bhe, P)) return fail("Too large for the payload bay (fold its solar panels)");
    if (why) why->clear();
    return best;
}

bool World::capturePayload(Vessel& v) {
    std::string why;
    int bi = captureCandidate(v, &why);
    if (bi < 0) {
        if (&v == activeVessel() && !why.empty()) toast(why, {255, 200, 120, 255});
        return false;
    }
    Vessel& A = v;
    Vessel& b = vessels[bi];
    Vec3d bc, bhe;
    Mat3d bR;
    bayFrame(A, bc, bR, bhe);
    Vec3d lo, hi;
    craftBounds(b.geometry(), lo, hi);
    Mat3d P;
    fitInBay(hi - lo, bhe, P);
    Vec3d mid = (lo + hi) * 0.5;
    // a payload this bay released comes back to where it was (its parts still hold their places in the bay);
    // another one is set down on the bay's floor
    Vec3d place = bc + bR * Vec3d{0, -bhe.y + 0.05, 0};
    {
        bool asIs = true;
        Mat3d bRT = bR.T();
        for (const Component& c : b.comps) {
            std::vector<Vec3d> pts;
            outlinePoints(c, pts, 12);
            for (const Vec3d& p : pts) {
                Vec3d l = bRT * (p - bc);
                asIs = asIs && std::fabs(l.x) <= bhe.x && std::fabs(l.y) <= bhe.y && std::fabs(l.z) <= bhe.z;
            }
        }
        if (asIs) {
            P = bR.T(); // (RP = identity: positions as they are)
            mid = bc;
            place = bc;
        } else {
            Vec3d s = P * (hi - lo);
            place = place + bR * Vec3d{0, std::fabs(s.y) / 2, 0};
        }
    }
    double mA = A.mp.mass, mB = b.mp.mass;
    Vec3d vC = (A.v * mA + b.v * mB) / (mA + mB);
    int top = 0;
    for (Component& c : A.comps) {
        top = std::max(top, c.stage);
        if (c.origin == 0) c.origin = A.id;
    }
    Vec3d oldCom = A.mp.com;
    Mat3d RP = bR * P;
    for (size_t k = 0; k < b.comps.size(); k++) {
        Component c = b.comps[k];
        // berthed by the arm in the bay
        c.pos = place + RP * (c.pos - mid);
        c.rotDeg = eulerDegOf(RP * c.rot().toMat());
        if (c.origin == 0) c.origin = b.id;
        if (c.stage0 < 0) c.stage0 = c.stage;
        c.stage = top;
        c.cargo = 1;
        A.comps.push_back(c);
        A.prop.push_back(k < b.prop.size() ? b.prop[k] : 0.0);
        A.heat.push_back(k < b.heat.size() ? b.heat[k] : 0.0);
    }
    if (!moduleNames.count(b.id)) moduleNames[b.id] = b.name;
    if (!moduleNames.count(A.id)) moduleNames[A.id] = A.name;
    const Body& bd = U.bodies[A.ref];
    setupVessel(A, bd.atmTop > 0 && len(A.r) - bd.radius < bd.atmTop);
    A.r = A.r + A.q.rotate(A.mp.com - oldCom);
    A.v = vC;
    if (A.targetVessel == b.id) A.targetVessel = -1;
    b.destroyed = true;
    b.debris = true; // (no crash report, not drawn)
    b.fate = "Stowed in the payload bay";
    std::string nm = b.name;
    event(A, SimEvent::Stage);
    toast(trf("Captured into the payload bay: {0}", nm), {140, 255, 170, 255});
    return true;
}

int World::launchInOrbit(const Craft& craft, int nearId, double dist) {
    int idx = launch(craft);
    if (idx < 0 || idx >= (int)vessels.size()) return idx;
    Vessel& v = vessels[idx];
    v.earthLaunch = false; // (placed in orbit, not flown there: no achievements)
    const Vessel* near = nullptr;
    for (const Vessel& o : vessels)
        if (o.id == nearId && !o.destroyed && !o.landed) near = &o;
    if (near) {
        // in the same orbit, 'dist' behind it, facing it
        v.ref = near->ref;
        // where it was dist / v seconds ago on its own orbit: truly co-orbital (the same velocity vector at a
        // point behind it would be another orbit and drift by metres within minutes)
        Vec3d r0 = near->r, v0 = near->v;
        keplerPropagate(r0, v0, U.bodies[near->ref].GM, -dist / std::max(len(near->v), 1.0));
        Vec3d dir = len(near->r - r0) > 1e-3 ? norm(near->r - r0) : (len(near->v) > 1 ? norm(near->v) : Vec3d{1, 0, 0});
        v.landed = false;
        v.r = r0;
        v.v = v0;
        v.w = {};
        v.q = Quatd::lookRot(dir, norm(r0));
        if (v.legsDeployed && v.gearRetract) {
            v.legsDeployed = false;
            setupVessel(v, false); // (the gear's shape out of the airflow)
        }
        v.legsDeployed = false;
        v.ignited = false;
        v.throttle = 0;
        v.targetVessel = near->id;
        v.launchTime = t;
        if (v.hasFairing) jettisonFairing(v, false);
        history.clear();
        recordHistory(true);
    } else {
        placeInOrbit(v, 400e3);
    }
    v.sas = SAS::Stability;
    return idx;
}

void World::pruneDebris(int keep) {
    int count = 0;
    for (const Vessel& v : vessels) count += v.debris && !v.destroyed;
    if (count <= keep) return;
    // the oldest (lowest id) debris go
    std::vector<int> ids;
    for (const Vessel& v : vessels)
        if (v.debris && !v.destroyed) ids.push_back(v.id);
    std::sort(ids.begin(), ids.end());
    ids.resize(count - keep);
    int actId = activeVessel() ? activeVessel()->id : -1;
    std::vector<Vessel> kept;
    kept.reserve(vessels.size() + 64);
    for (Vessel& v : vessels)
        if (!(v.debris && std::find(ids.begin(), ids.end(), v.id) != ids.end()) || v.id == actId) kept.push_back(std::move(v));
    vessels = std::move(kept);
    active = -1;
    for (size_t i = 0; i < vessels.size(); i++)
        if (vessels[i].id == actId) active = (int)i;
}

void World::toggleLegs(Vessel& v) {
    if (!v.hasLegs || v.destroyed) return;
    if (v.gearRetract && v.legsDeployed && (v.landed || v.rolling)) {
        if (&v == activeVessel()) toast("The landing gear cannot be raised on the ground", {255, 200, 120, 255});
        return;
    }
    if (!v.legsDeployed && v.hasFairing) {
        double z0 = 1e30, z1 = -1e30;
        for (const Component& c : v.comps)
            if (c.func == Func::Fairing) {
                z0 = std::min(z0, c.pos.z - c.dims.z / 2);
                z1 = std::max(z1, c.pos.z + c.dims.z / 2);
            }
        for (const Component& c : v.comps)
            if (c.func == Func::Legs && c.pos.z > z0 && c.pos.z < z1) {
                if (&v == activeVessel()) toast("The landing legs are folded inside the payload fairing", {255, 200, 120, 255});
                return;
            }
    }
    v.legsDeployed = !v.legsDeployed;
    if (v.gearRetract) {
        const Body& b = U.bodies[v.ref];
        setupVessel(v, b.atmTop > 0 && len(v.r) - b.radius < b.atmTop); // (drag of the gear in or out)
        if (&v == activeVessel()) toast(v.legsDeployed ? "Landing gear down" : "Landing gear up", {150, 200, 255, 255});
    }
}

void World::placeInOrbit(Vessel& v, double alt) {
    jettisonFairing(v, false); // a start in orbit: the fairing is long gone
    v.earthLaunch = false;
    const Body& b = U.bodies[v.ref];
    Vec3d rh = norm(v.r);
    Vec3d pole = b.poleFrame.col(2);
    if (b.rotRate < 0) pole = -pole;
    Vec3d dir = cross(pole, rh);
    if (len(dir) < 1e-6) dir = anyPerp(rh);
    double rr = b.radius + alt;
    v.landed = false;
    v.r = rh * rr;
    v.v = norm(dir) * std::sqrt(b.GM / rr);
    v.w = {};
    v.q = Quatd::lookRot(norm(v.v), rh);
    v.throttle = 0;
    v.ignited = false;
    if (v.legsDeployed && v.gearRetract) {
        v.legsDeployed = false;
        setupVessel(v, false); // (the gear's shape out of the airflow)
    }
    v.legsDeployed = false;
    v.prevAlt = -1;                     // no milestone toasts for the jump
    v.orbitClass = OrbitClass::Unknown; // classified silently
    v.bound = -1;
    v.launchTime = t;
    history.clear();
    recordHistory(true);
}

// ------------------------------------------------------------------ hyperspace
std::string World::hyperspaceLocationBlock(const Vessel& v) const {
    const Body& b = U.bodies[v.ref];
    if (v.landed) return tr("Hyperspace jumps are only possible in space: the vessel is on the ground.");
    double alt = len(v.r) - b.radius;
    if (b.atmTop > 0 && alt < b.atmTop)
        return trf("Hyperspace jumps are only possible in space: climb above the atmosphere of {0} ({1}).", tr(b.name), formatDist(b.atmTop));
    return {};
}

bool World::startHyperspace(int target, std::string& err) {
    diag::breadcrumb("hyperspace -> body %d", target);
    Vessel* v = activeVessel();
    if (!v || v->destroyed) { err = "No active vessel"; return false; }
    if (!settings.hyperspace) { err = "Hyperspace travel is disabled in Settings"; return false; }
    if (!v->hasHyperdrive) { err = "This vessel has no Hyperspace Drive part"; return false; }
    err = hyperspaceLocationBlock(*v);
    if (!err.empty()) return false;
    if (target < 0 || target >= U.count()) { err = "Choose a destination"; return false; }
    if (!settings.infiniteFuel && v->charge < v->chargeCap * 0.2 && v->chargeCap > 0) {
        err = "Hyperspace Drive needs at least 20% battery charge";
        return false;
    }
    hyper = {};
    hyper.active = true;
    v->cheated = true; // (a jump does not count for achievements)
    event(*v, SimEvent::HyperIn);
    hyper.target = target;
    Vec3d d = U.relPos(states, target, v->ref) - v->r;
    hyper.distance = len(d);
    hyper.dir = hyper.distance > 1e-9 ? d / hyper.distance : v->q.rotate({0, 0, 1});
    double sp = HYPER_SPEEDS[std::clamp(settings.hyperSpeed, 0, HYPER_SPEED_COUNT - 1)];
    hyper.travelTime = sp > 1e200 ? 0 : hyper.distance / sp;
    warpIndex = 0;
    v->throttle = 0;
    return true;
}

void World::updateHyperspace(double realDt) {
    hyper.phase += realDt;
    const double total = 5.0;
    if (hyper.phase < total) return;
    if (activeVessel()) event(*activeVessel(), SimEvent::HyperOut);
    hyper.active = false;
    Vessel* v = activeVessel();
    if (!v) return;
    // advance the rest of the universe by the travel time
    double tt = hyper.travelTime;
    for (int i = 0; i < (int)vessels.size(); i++)
        if (i != active) advanceVessel(*this, vessels[i], t, tt, false, 200000);
    t += tt;
    refreshStates();
    int tg = hyper.target;
    const Body& b = U.bodies[tg];
    double alt = std::max({b.atmTop + 60e3, 0.3 * b.radius, 30e3});
    if (b.isStar) alt = 15 * b.radius;
    double rOrb = b.radius + alt;
    if (b.parent >= 0 && rOrb > 0.7 * b.soi) rOrb = std::max(b.radius * 1.15, 0.7 * b.soi);
    Vec3d pole = b.poleFrame.col(2);
    if (b.rotRate < 0) pole = -pole;
    // place on the day side
    Vec3d toStar{1, 0, 0};
    if (!b.isStar) {
        int hs = U.hostStar(tg);
        toStar = norm(U.relPos(states, hs, tg));
    }
    Vec3d rh = toStar - pole * dot(toStar, pole);
    if (len(rh) < 1e-6) rh = anyPerp(pole);
    rh = norm(rh);
    v->ref = tg;
    v->landed = false;
    v->r = rh * rOrb;
    v->v = norm(cross(pole, rh)) * std::sqrt(b.GM / rOrb);
    v->w = {};
    v->q = Quatd::lookRot(norm(v->v), rh);
    std::fill(v->heat.begin(), v->heat.end(), 0.0);
    v->chuteDeployed = false;
    if (!settings.infiniteFuel) v->charge *= 0.5;
    toast(trf("Hyperspace exit: orbiting {0} ({1} travelled in {2})", tr(b.name), formatDist(hyper.distance), formatTime(tt)), {190, 160, 255, 255});
    explored[tg] |= 1;
}

// ------------------------------------------------------------------ save / load
// writes a temporary file and replaces the old one only once it is complete (also from a worker thread)
std::string World::serialize() const {
    std::ostringstream f;
    int activeOut = -1, activeId = -1, kept = 0;
    for (size_t i = 0; i < vessels.size(); i++) {
        if (vessels[i].destroyed) continue; // destroyed vessels are not written: renumber the active one
        if ((int)i == active) { activeOut = kept; activeId = vessels[i].id; }
        kept++;
    }
    f.precision(17);
    f << brand::tag("SAVE") << " 3\n";
    if (!mapName.empty()) {
        std::string nm = mapName.substr(0, 120);
        for (char& ch : nm)
            if (ch == '\n' || ch == '\r') ch = ' ';
        f << "mapname " << nm << "\n";
    }
    f << "time " << t << "\n";
    f << "settings " << settings.infiniteFuel << " " << settings.hyperspace << " " << settings.hyperSpeed << " " << settings.launchSite << " "
      << settings.launchBase << "\n";
    f << "active " << activeOut << "\n";
    f << "activeid " << activeId << "\n";
    f << "nextid " << nextId << "\n";
    f << "explored " << explored.size();
    for (unsigned e : explored) f << " " << e;
    f << "\n";
    for (const auto& a : achievements) {
        std::string nm = a.second.vessel.substr(0, 80);
        for (char& ch : nm)
            if (ch == '\n' || ch == '\r') ch = ' ';
        f << "achieved " << a.first << " " << a.second.t << " " << nm << "\n";
    }
    for (const auto& m : moduleNames) {
        std::string nm = m.second;
        for (char& ch : nm)
            if (ch == '\n' || ch == '\r') ch = ' ';
        f << "modname " << m.first << " " << nm << "\n";
    }
    // surface bases: one "base" line, then its modules ("bmod" lines belong to the base above them)
    f << "nextbase " << nextBaseId << "\n";
    for (const SurfaceBase& b : bases) {
        std::string nm = b.name.substr(0, 48);
        for (char& ch : nm)
            if (ch == '\n' || ch == '\r') ch = ' ';
        f << "base " << b.id << " " << b.body << " " << b.lat << " " << b.lon << " " << b.founded << " " << nm << "\n";
        for (const BaseModule& m : b.modules)
            f << "bmod " << (int)m.kind << " " << m.x << " " << m.y << " " << m.heading << " " << m.kw << " " << m.variant << "\n";
    }
    for (const auto& v : vessels) {
        if (v.destroyed) continue;
        f << "vessel\n";
        Craft c;
        c.name = v.name;
        c.comps = v.comps;
        std::istringstream ds(craftToText(c));
        std::string line;
        while (std::getline(ds, line)) f << "C " << line << "\n";
        f << "state " << v.id << " " << v.ref << " " << v.r.x << " " << v.r.y << " " << v.r.z << " " << v.v.x << " " << v.v.y << " " << v.v.z
          << " " << v.q.w << " " << v.q.x << " " << v.q.y << " " << v.q.z << " " << v.w.x << " " << v.w.y << " " << v.w.z << "\n";
        f << "flags " << v.landed << " " << v.ignited << " " << v.chuteDeployed << " " << v.chuteCut << " " << v.sailDeployed << " "
          << v.legsDeployed << " " << (int)v.sas << " " << v.target << " " << v.throttle << " " << v.charge << " " << v.launchTime << " "
          << v.solarDeployed << " " << v.rcs << " " << v.chuteOpen << " " << (int)v.rcsSource << " " << (int)v.debris << " " << v.targetVessel << "\n";
        f << "land " << v.landPos.x << " " << v.landPos.y << " " << v.landPos.z << " " << v.landQ.w << " " << v.landQ.x << " " << v.landQ.y
          << " " << v.landQ.z << "\n";
        f << "view " << v.hidden << " " << v.showOrbit << "\n";
        f << "mech " << v.flapStage << " " << v.flapPos << " " << v.bayOpen << " " << v.bayT << "\n";
        f << "record " << v.earthLaunch << " " << v.cheated << " " << v.reachedSpace << " " << v.landedOn << " " << v.orbited << "\n";
        f << "props " << v.prop.size();
        for (double p : v.prop) f << " " << p;
        f << "\nheats " << v.heat.size();
        for (double h : v.heat) f << " " << h;
        f << "\nendvessel\n";
    }
    f << "end\n";
    return f.str();
}

static bool finiteAll(std::initializer_list<double> l) {
    for (double d : l)
        if (!std::isfinite(d)) return false;
    return true;
}

bool World::deserialize(const std::string& text) {
    diag::breadcrumb("load game");
    std::istringstream f(text);
    std::string line;
    if (!std::getline(f, line) || !brand::hasTag(line, "SAVE")) return false;
    World w;
    w.U.init();
    w.explored.assign(w.U.count(), 0);
    Vessel cur;
    bool inVessel = false;
    std::string ctext, dtext;
    std::vector<double> props, heats;
    int lines = 0, activeId = -1;
    bool complete = false, baseOpen = false;
    while (std::getline(f, line) && lines++ < 400000) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::istringstream ls(line);
        std::string key;
        ls >> key;
        if (key == "time") { ls >> w.t; if (!std::isfinite(w.t)) return false; }
        else if (key == "settings") {
            int a = 0, b = 1, c = 1, d = 0, e = 0;
            ls >> a >> b >> c >> d;
            if (!(ls >> e)) e = 0; // (saves before the surface bases)
            w.settings.infiniteFuel = a != 0;
            w.settings.hyperspace = b != 0;
            w.settings.hyperSpeed = std::clamp(c, 0, HYPER_SPEED_COUNT - 1);
            w.settings.launchSite = std::clamp(d, 0, LAUNCH_SITE_COUNT - 1);
            w.settings.launchBase = std::max(0, e);
        } else if (key == "active") ls >> w.active;
        else if (key == "activeid") ls >> activeId;
        else if (key == "nextid") ls >> w.nextId;
        else if (key == "mapname") {
            std::string nm;
            std::getline(ls, nm);
            size_t a = nm.find_first_not_of(' ');
            w.mapName = a == std::string::npos ? std::string() : nm.substr(a, 120);
        }
        else if (key == "modname") {
            // the names of docked modules (given back on undocking)
            int id = -1;
            std::string nm;
            if (ls >> id && id >= 0 && w.moduleNames.size() < 4096) {
                std::getline(ls, nm);
                size_t a = nm.find_first_not_of(' ');
                nm = a == std::string::npos ? std::string() : nm.substr(a, 160);
                if (!nm.empty()) w.moduleNames[id] = nm;
            }
        }
        else if (key == "nextbase") {
            int n = 1;
            if (ls >> n) w.nextBaseId = std::clamp(n, 1, 1 << 24);
        } else if (key == "base") {
            // a surface base: id, body, centre [rad], founding time, name (bounded and checked)
            SurfaceBase b;
            if (ls >> b.id >> b.body >> b.lat >> b.lon >> b.founded && b.id > 0 && b.body >= 0 && b.body < w.U.count() &&
                finiteAll({b.lat, b.lon, b.founded}) && std::fabs(b.lat) <= PI / 2 + 1e-9 && std::fabs(b.lon) <= PI + 1e-9 &&
                (int)w.bases.size() < BASE_MAX && baseAllowed(w.U, b.body, nullptr) && findBase(w, b.id) < 0) {
                std::string nm;
                std::getline(ls, nm);
                size_t a = nm.find_first_not_of(' ');
                b.name = a == std::string::npos ? defaultBaseName(w, b.body) : nm.substr(a, 48);
                w.bases.push_back(b);
                baseOpen = true;
            } else baseOpen = false; // (its modules are skipped too)
        } else if (key == "bmod") {
            // a module of the base above
            int kind = -1, variant = 0;
            BaseModule m;
            if (baseOpen && !w.bases.empty() && ls >> kind >> m.x >> m.y >> m.heading >> m.kw >> variant && kind >= 0 &&
                kind < (int)ModuleKind::COUNT && finiteAll({m.x, m.y, m.heading, m.kw}) && std::fabs(m.x) < 20000 && std::fabs(m.y) < 20000 &&
                m.kw >= 0 && m.kw < 1e6 && (int)w.bases.back().modules.size() < BASE_MAX_MODULES) {
                m.kind = (ModuleKind)kind;
                m.variant = std::clamp(variant, 0, 7);
                w.bases.back().modules.push_back(m);
            }
        }
        else if (key == "achieved") {
            // an achievement of this map: id, time, vessel name (ids are plain words; bounded)
            std::string id, nm;
            double at = 0;
            if (ls >> id >> at && id.size() <= 48 && std::isfinite(at) && w.achievements.size() < 512) {
                bool okId = true;
                for (char ch : id) okId &= (std::isalnum((unsigned char)ch) || ch == '_' || ch == ':' || ch == '-');
                std::getline(ls, nm);
                size_t a = nm.find_first_not_of(' ');
                nm = a == std::string::npos ? std::string() : nm.substr(a, 80);
                if (okId) w.achievements[id] = {at, nm};
            }
        }
        else if (key == "view" && inVessel) {
            int h = 0, o = 1;
            ls >> h >> o;
            cur.hidden = h != 0;
            cur.showOrbit = o != 0;
        } else if (key == "record" && inVessel) {
            int el = 0, ch = 1, rs = 0;
            unsigned long long lo = 0, ob = 0;
            if (ls >> el >> ch >> rs >> lo >> ob) {
                cur.earthLaunch = el != 0;
                cur.cheated = ch != 0;
                cur.reachedSpace = rs != 0;
                cur.landedOn = lo;
                cur.orbited = ob;
            }
        }
        else if (key == "explored") {
            size_t n = 0;
            ls >> n;
            for (size_t i = 0; i < n && i < w.explored.size(); i++) ls >> w.explored[i];
        } else if (key == "vessel") {
            inVessel = true;
            cur = Vessel{};
            ctext.clear();
            dtext.clear();
            props.clear();
            heats.clear();
        } else if ((key == "C" || key == "D") && inVessel) {
            std::string& dst = key == "C" ? ctext : dtext;
            if (dst.size() < (2u << 20)) dst += line.substr(std::min<size_t>(2, line.size())) + "\n";
        } else if (key == "state" && inVessel) {
            ls >> cur.id >> cur.ref >> cur.r.x >> cur.r.y >> cur.r.z >> cur.v.x >> cur.v.y >> cur.v.z >> cur.q.w >> cur.q.x >> cur.q.y >> cur.q.z;
            if (!(ls >> cur.w.x >> cur.w.y >> cur.w.z)) cur.w = {};
        } else if (key == "flags" && inVessel) {
            int sas = 0;
            ls >> cur.landed >> cur.ignited >> cur.chuteDeployed >> cur.chuteCut >> cur.sailDeployed >> cur.legsDeployed >> sas >> cur.target >>
                cur.throttle >> cur.charge >> cur.launchTime;
            if (!(ls >> cur.solarDeployed)) cur.solarDeployed = true; // older saves: panels were always open
            if (!(ls >> cur.rcs)) cur.rcs = false;
            if (!(ls >> cur.chuteOpen) || !std::isfinite(cur.chuteOpen)) cur.chuteOpen = cur.chuteDeployed && !cur.chuteCut ? 1 : 0;
            cur.chuteOpen = clampd(cur.chuteOpen, 0, 1);
            int rs = 0;
            if (ls >> rs) cur.rcsSource = (RcsSource)std::clamp(rs, 0, 2);
            int db = 0;
            if (ls >> db) cur.debris = db != 0;
            int tv = -1;
            if (ls >> tv) cur.targetVessel = tv;
            cur.sas = (SAS)std::clamp(sas, 0, (int)SAS::COUNT - 1);
        } else if (key == "mech" && inVessel) {
            int fs = 0, bo = 0;
            double fp = 0, bt = 0;
            if (ls >> fs >> fp >> bo >> bt) {
                cur.flapStage = std::clamp(fs, 0, FLAP_STAGES);
                cur.flapPos = std::isfinite(fp) ? clampd(fp, 0, FLAP_STAGES) : 0;
                cur.bayOpen = bo != 0;
                cur.bayT = std::isfinite(bt) ? clampd(bt, 0, 1) : 0;
            }
        } else if (key == "land" && inVessel) {
            ls >> cur.landPos.x >> cur.landPos.y >> cur.landPos.z >> cur.landQ.w >> cur.landQ.x >> cur.landQ.y >> cur.landQ.z;
        } else if ((key == "props" || key == "heats") && inVessel) {
            size_t n = 0;
            ls >> n;
            n = std::min<size_t>(n, 4800);
            auto& dst = key == "props" ? props : heats;
            dst.assign(n, 0);
            for (size_t i = 0; i < n; i++) ls >> dst[i];
        } else if (key == "endvessel" && inVessel) {
            inVessel = false;
            Craft c;
            if (!ctext.empty()) {
                if (!craftFromText(c, ctext, 400 * 12)) continue;
            } else {
                Design d; // version 1 saves stored stacked designs
                if (!designFromText(d, dtext)) continue;
                c = craftFromDesign(d);
                props.clear(); // stack-indexed propellant cannot be mapped reliably: start full
            }
            if (cur.ref < 0 || cur.ref >= w.U.count()) continue;
            if (!finiteAll({cur.r.x, cur.r.y, cur.r.z, cur.v.x, cur.v.y, cur.v.z, cur.q.w, cur.q.x, cur.q.y, cur.q.z, cur.w.x, cur.w.y, cur.w.z,
                            cur.charge, cur.throttle, cur.launchTime, cur.landPos.x, cur.landPos.y, cur.landPos.z, cur.landQ.w, cur.landQ.x,
                            cur.landQ.y, cur.landQ.z}))
                continue;
            cur.name = c.name;
            cur.comps = expandCraft(c);
            if (cur.comps.empty()) continue;
            setupVessel(cur, true); // fills full propellant
            if (props.size() == cur.comps.size())
                for (size_t i = 0; i < props.size(); i++)
                    cur.prop[i] = clampd(std::isfinite(props[i]) ? props[i] : 0, 0, cur.phys[i].propCap);
            if (heats.size() == cur.comps.size())
                for (size_t i = 0; i < heats.size(); i++) cur.heat[i] = clampd(std::isfinite(heats[i]) ? heats[i] : 0, 0, 0.99);
            updateMassProps(cur);
            if (cur.target < -1 || cur.target >= w.U.count()) cur.target = -1;
            cur.q = cur.q.normalized();
            cur.landQ = cur.landQ.normalized();
            cur.throttle = clampd(cur.throttle, 0, 1);
            cur.w = {clampd(cur.w.x, -30, 30), clampd(cur.w.y, -30, 30), clampd(cur.w.z, -30, 30)};
            cur.charge = clampd(cur.charge, 0, cur.chargeCap);
            if (w.vessels.size() < 256) w.vessels.push_back(cur);
        } else if (key == "end") {
            complete = true;
            break;
        }
    }
    if (!complete) return false; // truncated or damaged file: keep the current game
    if (activeId >= 0)
        for (size_t i = 0; i < w.vessels.size(); i++)
            if (w.vessels[i].id == activeId) w.active = (int)i;
    if (w.active >= (int)w.vessels.size()) w.active = w.vessels.empty() ? -1 : 0;
    if (w.active < -1) w.active = -1;
    w.nextId = std::clamp(w.nextId, (int)w.vessels.size() + 1, 1 << 24);
    for (const SurfaceBase& b : w.bases) w.nextBaseId = std::max(w.nextBaseId, b.id + 1);
    w.vessels.reserve(w.vessels.size() + 64);
    w.warpIndex = 0;
    w.refreshStates();
    w.missionLog = std::move(missionLog); // the mission log belongs to the session, not to the save
    *this = std::move(w);
    syncBaseTerrain();
    return true;
}
