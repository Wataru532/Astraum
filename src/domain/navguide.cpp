#include "navguide.hpp"
#include "i18n.hpp"
#include "orbit.hpp"
#include "sim.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>

using i18n::tr;
using i18n::trf;

namespace navg {

// ------------------------------------------------------------------ Lambert's problem
// universal variables (Bate, Mueller & White; Curtis algorithm 5.2): solve F(z) = 0 for the single-revolution
// conic through r1 and r2 with time of flight dt. F rises with z; the first sign change is bracketed on a
// coarse grid and refined by bisection.
static double stC(double z) {
    if (z > 1e-6) return (1 - std::cos(std::sqrt(z))) / z;
    if (z < -1e-6) return (std::cosh(std::sqrt(-z)) - 1) / (-z);
    return 0.5 - z / 24 + z * z / 720;
}
static double stS(double z) {
    if (z > 1e-6) {
        double s = std::sqrt(z);
        return (s - std::sin(s)) / (s * s * s);
    }
    if (z < -1e-6) {
        double s = std::sqrt(-z);
        return (std::sinh(s) - s) / (s * s * s);
    }
    return 1.0 / 6 - z / 120 + z * z / 5040;
}

bool lambert(const Vec3d& r1, const Vec3d& r2, double dt, double mu, const Vec3d& h, Vec3d& v1, Vec3d& v2) {
    double R1 = len(r1), R2 = len(r2);
    if (dt <= 0 || R1 <= 0 || R2 <= 0 || mu <= 0) return false;
    double cth = clampd(dot(r1, r2) / (R1 * R2), -1, 1);
    double th = std::acos(cth);
    if (dot(cross(r1, r2), h) < 0) th = TAU - th;
    double sth = std::sin(th);
    if (std::fabs(sth) < 1e-5 || 1 - cth < 1e-10) return false; // (0 or 180 degrees: the plane is undefined)
    double A = sth * std::sqrt(R1 * R2 / (1 - cth));
    double smu = std::sqrt(mu);
    auto yOf = [&](double z) { return R1 + R2 + A * (z * stS(z) - 1) / std::sqrt(stC(z)); };
    auto F = [&](double z) {
        double y = yOf(z);
        if (!(y > 0)) return -1e300;
        return std::pow(y / stC(z), 1.5) * stS(z) + A * std::sqrt(y) - smu * dt;
    };
    const double zHi = TAU * TAU - 1e-6;
    double lo = -4 * TAU * TAU, flo = F(lo);
    for (int k = 0; k < 12 && flo > 0; k++) {
        lo *= 3;
        flo = F(lo);
    }
    if (flo > 0) return false;
    // first crossing from below on a grid (F can be -inf near zHi when A < 0)
    const int N = 16;
    double a = lo, b = -1;
    for (int i = 1; i <= N; i++) {
        double z = lo + (zHi - lo) * i / N;
        if (F(z) >= 0) { b = z; break; }
        a = z;
    }
    if (b < a) return false;
    for (int it = 0; it < 44; it++) {
        double m = 0.5 * (a + b);
        if (F(m) < 0) a = m;
        else b = m;
    }
    double z = 0.5 * (a + b), y = yOf(z);
    if (!(y > 0)) return false;
    double f = 1 - y / R1, g = A * std::sqrt(y / mu), gd = 1 - y / R2;
    if (std::fabs(g) < 1e-12) return false;
    v1 = (r2 - r1 * f) / g;
    v2 = (r2 * gd - r1) / g;
    return std::isfinite(v1.x + v1.y + v1.z + v2.x + v2.y + v2.z);
}

bool nearestPorts(const World& W, const Vessel& a, const Vessel& t, int& ia, int& it) {
    double best = 1e300;
    ia = it = -1;
    for (int i = 0; i < (int)a.comps.size(); i++) {
        if (a.comps[i].func != Func::Docking || a.comps[i].dockedTo) continue;
        Vec3d pa, da;
        W.portFrame(a, i, pa, da);
        for (int j = 0; j < (int)t.comps.size(); j++) {
            if (t.comps[j].func != Func::Docking || t.comps[j].dockedTo) continue;
            Vec3d pb, db;
            W.portFrame(t, j, pb, db);
            // the nearest pair, preferring ports that can face each other (the target port toward the chaser)
            double g = len(pb - pa) * (dot(db, pa - pb) < 0 ? 3.0 : 1.0);
            if (g < best) { best = g; ia = i; it = j; }
        }
    }
    return ia >= 0;
}

// ------------------------------------------------------------------ the transfer plan
// The grid search (~1,000 Lambert solutions, ~20 ms) runs on a worker thread on a snapshot of the two
// orbits; the frame only re-evaluates the chosen plan (departure / arrival times) from the current state.
namespace {
struct Snap {
    double GM = 0, R = 0, atm = 0, now = 0; // body, time of the snapshot
    Vec3d ra, va, rt, vt;                   // chaser, target
    double Tc = 0, Tt = 0;                  // their orbit periods
};
struct Found { bool ok = false; double tDep = 0, tArr = 0; };

struct PlanState {
    int aId = -1, tId = -1, ref = -1;
    double tDep = 0, tArr = 0;
    bool valid = false;
    double lastSearch = -1e9;
    std::future<Found> job;
    bool running = false;
    int jobA = -1, jobT = -1, jobRef = -1;
};
PlanState P;

struct Eval {
    bool ok = false;
    Vec3d dv1, dv2;   // departure burn, arrival burn (world)
    Vec3d rDep, vDep; // the chaser at the departure, before the burn
    double cost = 1e300;
};

// the transfer leaving at tDep (now if past) and meeting the target at tArr
Eval evaluate(const Snap& S, double tDep, double tArr) {
    Eval e;
    double now = S.now;
    tDep = std::max(tDep, now);
    if (tArr <= tDep + 1) return e;
    Vec3d r1 = S.ra, v1 = S.va;
    keplerPropagate(r1, v1, S.GM, tDep - now);
    Vec3d r2 = S.rt, v2 = S.vt;
    keplerPropagate(r2, v2, S.GM, tArr - now);
    Vec3d h = cross(S.ra, S.va);
    Vec3d l1, l2;
    if (!lambert(r1, r2, tArr - tDep, S.GM, h, l1, l2)) return e;
    // the transfer must not dip into the atmosphere / the ground on its way
    Conic c = stateToConic(r1, l1, S.GM);
    double safe = S.R + std::max(S.atm > 0 ? S.atm : 0.0, 0.005 * S.R);
    if (c.valid && c.pe < safe) {
        double nu1 = std::atan2(std::sin(c.nu), std::cos(c.nu)); // (-pi, pi]
        double cth = clampd(dot(r1, r2) / (len(r1) * len(r2)), -1, 1), th = std::acos(cth);
        if (dot(cross(r1, r2), h) < 0) th = TAU - th;
        bool passesPe = (nu1 < 0 && nu1 + th > 0) || nu1 + th > TAU;
        if (passesPe) return e;
    }
    e.ok = true;
    e.dv1 = l1 - v1;
    e.dv2 = v2 - l2;
    e.rDep = r1;
    e.vDep = v1;
    e.cost = len(e.dv1) + len(e.dv2);
    return e;
}

double period(const Vessel& v, double mu) {
    Conic c = stateToConic(v.r, v.v, mu);
    return c.valid && c.e < 1 && c.period > 0 ? c.period : 0;
}

// grid search: departure in [now, now + span], flight time over a range of the orbit period. `cur`: the plan
// in use, kept unless the new one is clearly better (the instructions should not jump around)
Found search(Snap S, Found cur) {
    double now = S.now, Tc = S.Tc, Tt = S.Tt;
    double Tm = 0.5 * (Tc + Tt);
    double syn = std::fabs(Tc - Tt) > 1e-6 ? Tc * Tt / std::fabs(Tc - Tt) : 1e30;
    double span = clampd(syn, Tc, 8 * Tc);
    const int NW = 56, NF = 16;
    double best = 1e300, bd = 0, ba = 0;
    for (int i = 0; i < NW; i++) {
        double wait = span * i / NW;
        for (int j = 0; j < NF; j++) {
            double tof = Tm * (0.12 + 0.9 * j / (NF - 1));
            Eval e = evaluate(S, now + wait, now + wait + tof);
            if (!e.ok) continue;
            double cost = e.cost + 0.5 * wait / Tc; // (sooner is better when it costs about the same)
            if (cost < best) { best = cost; bd = now + wait; ba = now + wait + tof; }
        }
    }
    if (best > 1e299) return cur.ok ? cur : Found{};
    // refine around the best point
    double sw = span / NW, sf = Tm * 0.9 / (NF - 1);
    for (int pass = 0; pass < 2; pass++) {
        double cd = bd, ca = ba;
        for (int i = -3; i <= 3; i++)
            for (int j = -3; j <= 3; j++) {
                double d = cd + sw * i / 3, ar = ca + (sw * i + sf * j) / 3;
                if (d < now) continue;
                Eval e = evaluate(S, d, ar);
                if (!e.ok) continue;
                double cost = e.cost + 0.5 * (d - now) / Tc;
                if (cost < best) { best = cost; bd = d; ba = ar; }
            }
        sw /= 3;
        sf /= 3;
    }
    if (cur.ok) {
        Eval ce = evaluate(S, cur.tDep, cur.tArr);
        if (ce.ok && ce.cost + 0.5 * std::max(0.0, cur.tDep - now) / Tc <= best * 1.05 + 0.2) return cur;
    }
    return {true, bd, ba};
}
} // namespace

// ------------------------------------------------------------------ launch window from the ground
// s(t) = (site direction at t) . h: the sine of the site's distance from the plane. Over one turn of the body
// it runs like sin(lat) cos(i) + cos(lat) sin(i) cos(theta): |s| has its first local minimum at the next crossing
// (zero) or, when the plane never comes over the site, where the two are nearest. Sampled over a little more than
// one turn, refined by golden-section search.
bool launchWindow(const World& W, int body, const Vec3d& siteFixed, const Vec3d& h, double now, LaunchWindow& out) {
    const Universe& U = W.U;
    const Body& b = U.bodies[body];
    if (std::fabs(b.rotRate) < 1e-12 || len(h) < 1e-12) return false;
    Vec3d site = norm(siteFixed), hn = norm(h);
    auto s = [&](double tt) { return std::fabs(dot(U.bodyRot(body, tt) * site, hn)); };
    const double P = TAU / std::fabs(b.rotRate);
    const int N = 720;
    const double step = P * 1.05 / N;
    double prev2 = s(now), prev = s(now + step);
    double lo = -1, hi = -1;
    {
        // a plane that keeps the same angle to the site all day (an equatorial orbit): every time is as good
        double mn = 1e9, mx = -1e9;
        for (int k = 0; k <= 24; k++) {
            double x = s(now + P * k / 24.0);
            mn = std::min(mn, x);
            mx = std::max(mx, x);
        }
        if (mx - mn < std::sin(0.2 * DEG)) { lo = now; hi = now; }
    }
    // in the plane right now (just past the crossing): the window is now
    if (prev2 <= prev && prev2 < std::sin(0.3 * DEG)) { lo = now; hi = now + step; }
    for (int k = 2; k <= N && lo < 0; k++) {
        double cur = s(now + k * step);
        if (prev <= prev2 && prev <= cur) { lo = now + (k - 2) * step; hi = now + k * step; }
        prev2 = prev;
        prev = cur;
    }
    if (lo < 0) return false;
    lo = std::max(lo, now);
    const double gr = 0.6180339887498949;
    double x1 = hi - gr * (hi - lo), x2 = lo + gr * (hi - lo), f1 = s(x1), f2 = s(x2);
    for (int it = 0; it < 40 && hi - lo > 0.5; it++) {
        if (f1 < f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - gr * (hi - lo); f1 = s(x1); }
        else { lo = x1; x1 = x2; f1 = f2; x2 = lo + gr * (hi - lo); f2 = s(x2); }
    }
    double tMin = 0.5 * (lo + hi);
    out.offDeg = std::asin(clampd(s(tMin), 0, 1)) / DEG;
    // lift-off a little before the crossing: the rocket keeps the site's speed across the plane until the ascent
    // has taken it out (~1/2 of the way it drifts while that takes; measured with SAS Ascent: ~3 minutes)
    out.t = out.offDeg < 0.5 ? std::max(now, tMin - 180.0) : tMin;
    // the heading along the plane at the site then (the direction the plane moves through it)
    Vec3d up = U.bodyRot(body, tMin) * site;
    Vec3d pole = b.poleFrame.col(2);
    Vec3d east = cross(pole, up);
    if (len(east) < 1e-9) east = anyPerp(up);
    east = norm(east);
    Vec3d north = cross(up, east);
    Vec3d d = cross(hn, up);
    double az = len(d) > 1e-9 ? std::atan2(dot(d, east), dot(d, north)) / DEG : 90;
    out.azDeg = az < 0 ? az + 360 : az;
    return true;
}

Guidance compute(const World& W, const Vessel& a, const Vessel& t, double realNow, double searchEvery) {
    Guidance g;
    const Universe& U = W.U;
    if (a.landed) {
        g.mode = Mode::Message;
        g.message = trf("Reach orbit first: the rendezvous with {0} is planned from there.", t.name);
        if (t.ref == a.ref && !t.landed) {
            Vec3d h = cross(t.r, t.v);
            LaunchWindow lw;
            if (len(h) > 0 && launchWindow(W, a.ref, a.landPos, h, W.t, lw)) {
                g.mode = Mode::Launch;
                g.tLaunch = lw.t;
                g.launchAzDeg = lw.azDeg;
                g.planeOffDeg = lw.offDeg;
                g.offNowDeg = std::asin(clampd(std::fabs(dot(norm(U.bodyRot(a.ref, W.t) * a.landPos), norm(h))), 0, 1)) / DEG;
                g.message = lw.offDeg < 0.5
                                ? trf("Lift off at the window with SAS Ascent: it flies along the orbital plane of {0}. The rendezvous is planned in orbit.", tr(t.name))
                                : trf("The orbital plane of {0} never passes over the site: at the window it comes nearest. The rest is a plane change in orbit.", tr(t.name));
            }
        }
        return g;
    }
    if (t.landed) {
        g.mode = Mode::Message;
        g.message = trf("{0} is on the ground: docking needs it in flight.", t.name);
        return g;
    }
    if (t.ref != a.ref) {
        g.mode = Mode::Message;
        g.message = trf("{0} is near {1}: travel there first (set {1} as target on the map).", t.name, tr(U.bodies[t.ref].name));
        return g;
    }
    const Body& b = U.bodies[a.ref];
    g.range = len(t.r - a.r);
    Vec3d u = a.v - t.v;
    g.closing = g.range > 1e-6 ? dot(u, (t.r - a.r) / g.range) : 0; // (positive: approaching)
    {
        Vec3d ha = cross(a.r, a.v), ht = cross(t.r, t.v);
        if (len(ha) > 0 && len(ht) > 0) g.relIncDeg = std::acos(clampd(dot(norm(ha), norm(ht)), -1, 1)) / DEG;
    }

    // ---------------- near: proximity operations along the target port's axis
    if (g.range < 2500) {
        P.valid = false;
        g.mode = Mode::Proximity;
        int ia, it;
        bool ports = nearestPorts(W, a, t, ia, it);
        Vec3d PA, dA, PT, n;
        if (ports) {
            W.portFrame(a, ia, PA, dA);
            W.portFrame(t, it, PT, n);
            g.alignDeg = std::acos(clampd(-dot(dA, n), -1, 1)) / DEG;
        } else {
            PA = a.r;
            PT = t.r;
            n = g.range > 1e-3 ? (a.r - t.r) / g.range : Vec3d{0, 0, 1};
        }
        Vec3d d = PA - PT;
        double along = dot(d, n);
        Vec3d lat = d - n * along;
        double latL = len(lat);
        g.along = along;
        g.lateral = latL;
        double Rt = std::max(t.maxR, t.length * 0.5) + 5;
        double S = std::max(30.0, Rt * 1.5);              // stand-off point on the axis
        double corridor = 0.3 + 0.12 * std::max(along, 0.0); // approach cone around the axis
        auto speedFor = [](double dist) { return clampd(0.1 * std::sqrt(std::max(dist, 0.0)), 0.06, 4.0); };
        Vec3d vdes;
        if (!ports) {
            // no free port pair: hold at the stand-off distance
            Vec3d goal = PT + n * S;
            Vec3d to = goal - PA;
            double dist = len(to);
            vdes = dist > 0.5 ? to / dist * std::min(speedFor(dist), dist * 0.2) : Vec3d{};
            g.viaWaypoint = true;
        } else if (latL > corridor || along < 0) {
            // off the axis (beside or behind the target): first onto the axis, out at the stand-off point
            g.viaWaypoint = true;
            Vec3d latDir = latL > 1e-3 ? lat / latL : anyPerp(n);
            Vec3d goal = along >= S * 0.5 ? PT + n * along : PT + n * S + latDir * std::max(latL, Rt);
            Vec3d to = goal - PA;
            double dist = len(to);
            vdes = dist > 1e-3 ? to / dist * std::min(speedFor(dist), dist * 0.2) : Vec3d{};
        } else {
            // final approach down the axis, slowing to ~0.1 m/s at the contact; the drift off the axis is damped
            double sp = clampd(0.1 * std::sqrt(std::max(along, 0.0)), 0.05, 2.5);
            Vec3d side = lat * -0.08;
            if (len(side) > 0.3) side = norm(side) * 0.3;
            vdes = -n * sp + side;
        }
        Vec3d dv = vdes - u;
        g.dvCraft = a.q.conj().rotate(dv);
        g.burn = dv;
        return g;
    }

    // ---------------- far: rendezvous transfer
    double Tc = period(a, b.GM), Tt = period(t, b.GM);
    if (Tc <= 0 || Tt <= 0) {
        P.valid = false;
        g.mode = Mode::Message;
        g.message = tr("Both vessels must be in closed orbits for a rendezvous plan.");
        return g;
    }
    if (P.aId != a.id || P.tId != t.id || P.ref != a.ref) {
        P.aId = a.id;
        P.tId = t.id;
        P.ref = a.ref;
        P.valid = false;
        P.lastSearch = -1e9;
    }
    Snap S;
    S.GM = b.GM;
    S.R = b.radius;
    S.atm = b.atmTop;
    S.now = W.t;
    S.ra = a.r;
    S.va = a.v;
    S.rt = t.r;
    S.vt = t.v;
    S.Tc = Tc;
    S.Tt = Tt;
    // a finished search: adopt its plan (if it was for these two vessels)
    if (P.running && P.job.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        Found fd = P.job.get();
        P.running = false;
        if (P.jobA == a.id && P.jobT == t.id && P.jobRef == a.ref) {
            P.valid = fd.ok;
            if (fd.ok) { P.tDep = fd.tDep; P.tArr = fd.tArr; }
        }
    }
    Eval e;
    if (P.valid) e = evaluate(S, P.tDep, P.tArr);
    bool stale = !P.valid || !e.ok || P.tArr - W.t < 0.03 * Tc;
    // (a plan that went stale is replaced at once; without any plan the search is retried at the normal pace)
    if (!P.running && ((stale && P.valid) || realNow - P.lastSearch >= searchEvery)) {
        P.lastSearch = realNow;
        P.jobA = a.id;
        P.jobT = t.id;
        P.jobRef = a.ref;
        Found cur{P.valid && e.ok, P.tDep, P.tArr};
        P.job = std::async(std::launch::async, search, S, cur);
        P.running = true;
    }
    if (stale && P.running) {
        g.mode = Mode::Message;
        g.planning = true;
        g.message = tr("Planning the rendezvous...");
        return g;
    }
    if (!P.valid || !e.ok) {
        g.mode = Mode::Message;
        g.message = tr("No rendezvous transfer found from this orbit yet: match the target's orbit plane and altitude more closely.");
        return g;
    }
    g.tBurn = std::max(P.tDep, W.t);
    g.tArrive = P.tArr;
    g.dvTotal = len(e.dv1);
    g.dvMatch = len(e.dv2);
    // the burn in the orbital frame at the departure point
    Vec3d pro = norm(e.vDep), nrm = norm(cross(e.rDep, e.vDep)), rad = norm(cross(e.vDep, nrm));
    g.dvPro = dot(e.dv1, pro);
    g.dvNrm = dot(e.dv1, nrm);
    g.dvRad = dot(e.dv1, rad);
    // the same direction now (the frame turns with the orbit until the burn): what the navball shows
    Vec3d proN = norm(a.v), nrmN = norm(cross(a.r, a.v)), radN = norm(cross(a.v, nrmN));
    g.burn = proN * g.dvPro + nrmN * g.dvNrm + radN * g.dvRad;
    g.mode = (g.tBurn - W.t < 1.0 && g.dvTotal < 0.3) ? Mode::Coast : Mode::Transfer;
    if (g.mode == Mode::Coast) g.burn = {};
    return g;
}

void invalidate() {
    P.aId = P.tId = P.ref = -1;
    P.jobA = P.jobT = P.jobRef = -1;
    P.valid = false;
    P.lastSearch = -1e9;
}

void shutdown() {
    if (P.running && P.job.valid()) P.job.wait();
    P.running = false;
    invalidate();
}

} // namespace navg
