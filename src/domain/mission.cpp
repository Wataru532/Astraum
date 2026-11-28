#include "mission.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include "navguide.hpp"
#include "orbit.hpp"
#include "sim.hpp"

namespace mission {
namespace {

// the child of `B` whose family holds `T` (T itself, or the planet of a moon T); -1: none
int childToward(const Universe& U, int B, int T) {
    for (int x = T; x >= 0; x = U.bodies[x].parent)
        if (U.bodies[x].parent == B) return x;
    return -1;
}

// state of body i relative to body j at time t (only the bodies on the two chains are computed)
void relState(const Universe& U, double t, int i, int j, Vec3d& r, Vec3d& v) {
    std::vector<int> need;
    for (int x = i; x >= 0; x = U.bodies[x].parent) need.insert(need.begin(), x);
    std::vector<int> cj;
    for (int x = j; x >= 0; x = U.bodies[x].parent) cj.insert(cj.begin(), x);
    need.insert(need.end(), cj.begin(), cj.end());
    std::vector<BodyState> S;
    U.computeStatesFor(t, need, S);
    r = U.relPos(S, i, j);
    v = U.relVel(S, i, j);
}

double hohmannTime(double mu, double r1, double r2) {
    double a = 0.5 * (r1 + r2);
    return PI * std::sqrt(a * a * a / mu);
}

// the height above the surface where the air (or the relief) is no longer a concern
double safeAlt(const Body& b) { return b.atmTop > 0 ? b.atmTop : std::max(5e3, 0.005 * b.radius); }

struct Xfer {
    bool ok = false;
    double tDep = 0, tof = 0, cost = 1e300;
    Vec3d r1, v1, vDep;  // departure point, the transfer's velocity there, the velocity it starts from
    Vec3d v2, vGoal;     // arrival: the transfer's velocity, the goal's
};

// the cheapest Lambert transfer (departure + arrival velocity change) to body G around P, departing over
// [t0, t0 + span] from the state dep(t), with flight times in [tofLo, tofHi]
// (earliest: of the departures within `slack` of the cheapest, the first - the window that is open now or comes
// next, not a slightly cheaper one years later)
template <class Dep>
Xfer bestTransfer(const Universe& U, int P, int G, double t0, double span, double tofLo, double tofHi, Dep dep, int nDep, int nTof,
                  double slack = 1.0) {
    const double mu = U.bodies[P].GM;
    Xfer best;
    auto solve = [&](double td, double tof, Xfer& x) {
        if (tof <= 0) return false;
        Vec3d r1, vd, r2, vg, v1, v2;
        dep(td, r1, vd);
        relState(U, td + tof, G, P, r2, vg);
        if (!navg::lambert(r1, r2, tof, mu, cross(r1, vd), v1, v2)) return false;
        double c = len(v1 - vd) + len(v2 - vg);
        if (!std::isfinite(c)) return false;
        x = {true, td, tof, c, r1, v1, vd, v2, vg};
        return true;
    };
    auto tryOne = [&](double td, double tof) {
        Xfer x;
        if (solve(td, tof, x) && x.cost < best.cost) best = x;
    };
    std::vector<Xfer> perDep(std::max(nDep, 1));
    for (int i = 0; i < nDep; i++)
        for (int j = 0; j < nTof; j++) {
            Xfer x;
            if (solve(t0 + span * i / std::max(nDep - 1, 1), tofLo + (tofHi - tofLo) * j / std::max(nTof - 1, 1), x) && x.cost < perDep[i].cost)
                perDep[i] = x;
        }
    double minCost = 1e300;
    for (const Xfer& x : perDep)
        if (x.ok) minCost = std::min(minCost, x.cost);
    for (int i = 0; i < nDep && !best.ok; i++) {
        // the first local minimum that is good enough
        const Xfer& x = perDep[i];
        if (!x.ok || x.cost > minCost * slack) continue;
        bool localMin = (i == 0 || !perDep[i - 1].ok || perDep[i - 1].cost >= x.cost) && (i + 1 >= nDep || !perDep[i + 1].ok || perDep[i + 1].cost >= x.cost);
        if (localMin || slack <= 1.0) best = x;
    }
    if (!best.ok)
        for (const Xfer& x : perDep)
            if (x.ok && x.cost < best.cost) best = x;
    if (!best.ok) return best;
    // a finer look around the best cell
    double sd = span / std::max(nDep - 1, 1), st = (tofHi - tofLo) / std::max(nTof - 1, 1);
    double td0 = best.tDep, tf0 = best.tof;
    for (int i = -4; i <= 4; i++)
        for (int j = -4; j <= 4; j++) tryOne(std::max(t0, td0 + sd * i / 4), tf0 + st * j / 4);
    return best;
}

// ejection from a parking orbit (conic c around a body of gravity mu) onto the hyperbola that leaves with
// the excess velocity vInf: where to burn (the time after now on the orbit) and how much
struct Ejection {
    bool ok = false;
    double tWait = 0, dv = 0;
    Vec3d dir; // prograde at the burn point (world axes)
};
Ejection ejection(const Conic& c, const Vec3d& r, const Vec3d& v, double mu, const Vec3d& vInf) {
    Ejection out;
    double vi = len(vInf);
    if (!c.valid || c.e >= 1 || vi < 1) return out;
    Vec3d s = vInf / vi;
    Vec3d sp = s - c.H * dot(s, c.H); // the asymptote's direction in the orbit's plane
    if (len(sp) < 1e-6) return out;
    sp = norm(sp);
    double rp = 0.5 * (c.pe + c.ap);
    double eh = 1 + rp * vi * vi / mu;
    double nuInf = std::acos(std::clamp(-1.0 / eh, -1.0, 1.0));
    // the periapsis of the hyperbola (the burn point) lies nuInf before the asymptote, along the motion
    Vec3d p = sp * std::cos(nuInf) - cross(c.H, sp) * std::sin(nuInf);
    double nu = std::atan2(dot(p, c.Q), dot(p, c.P));
    double tw = timeToTrueAnomaly(c, nu);
    if (tw < 0) return out;
    if (c.period > 0 && tw > 0.97 * c.period) tw = 0; // (just passed during the burn: still burning now)
    Vec3d rr = r, vv = v;
    keplerPropagate(rr, vv, mu, tw);
    out.ok = true;
    out.tWait = tw;
    out.dir = norm(vv);
    out.dv = std::sqrt(vi * vi + 2 * mu / len(rr)) - len(vv);
    return out;
}

// lift-off time for a moon G of body B: a launch due east puts the orbit plane through the site with its
// normal along the local north; the window is the moment that plane holds the point where G will be on arrival
double moonLaunchWindow(const World& W, const Vessel& v, int B, int G, double arriveAfter) {
    const Universe& U = W.U;
    const Body& b = U.bodies[B];
    Mat3d R = U.bodyRot(B, W.t);
    Vec3d upF = norm(R.T() * v.r); // body-fixed direction of the pad
    Vec3d pole = b.poleFrame.col(2);
    auto miss = [&](double t) {
        Vec3d up = U.bodyRot(B, t) * upF;
        Vec3d north = pole - up * dot(pole, up);
        if (len(north) < 1e-9) return 1.0;
        Vec3d g, gv;
        relState(U, t + arriveAfter, G, B, g, gv);
        return dot(norm(north), norm(g));
    };
    double day = b.rotRate != 0 ? std::fabs(TAU / b.rotRate) : DAY;
    double span = std::min(1.2 * day, 30 * DAY), step = span / 720;
    double t0 = W.t + 120, f0 = miss(t0);
    for (double t = t0; t < t0 + span; t += step) {
        double f1 = miss(t + step);
        if ((f0 < 0) != (f1 < 0)) {
            double lo = t, hi = t + step, flo = f0;
            for (int it = 0; it < 30; it++) {
                double mid = 0.5 * (lo + hi), fm = miss(mid);
                if ((fm < 0) == (flo < 0)) {
                    lo = mid;
                    flo = fm;
                } else hi = mid;
            }
            return 0.5 * (lo + hi);
        }
        f0 = f1;
    }
    return -1;
}

// the interplanetary transfer from planet B to its sibling G around P: the next window
Xfer planetTransfer(const World& W, int B, int G) {
    const Universe& U = W.U;
    int P = U.bodies[B].parent;
    const Body &bb = U.bodies[B], &gb = U.bodies[G];
    double mu = U.bodies[P].GM;
    double nB = std::sqrt(mu / std::pow(bb.a, 3)), nG = std::sqrt(mu / std::pow(gb.a, 3));
    double syn = std::fabs(nB - nG) > 1e-15 ? TAU / std::fabs(nB - nG) : 3 * YEAR;
    double span = std::clamp(syn, 30 * DAY, 3 * YEAR);
    double tH = hohmannTime(mu, bb.a, gb.a);
    auto dep = [&](double t, Vec3d& r, Vec3d& v) { relState(U, t, B, P, r, v); };
    return bestTransfer(U, P, G, W.t, span, 0.55 * tH, 1.45 * tH, dep, 120, 28, 1.15);
}

// cache of the heavy searches (the flight screen asks every frame)
struct Cache {
    int vessel = -1, target = -1, ref = -1, phase = -1;
    double real = -1e300;
    Plan plan;
};
Cache g_cache;
// the departure chosen last (vessel, goal, time): kept when it is close, so that "burn now" stays now
struct LastDep {
    int vessel = -1, goal = -1;
    double t = 0;
} g_lastDep;

Plan computeNow(const World& W, const Vessel& v, int target) {
    const Universe& U = W.U;
    Plan p;
    p.target = target;
    if (target < 0 || target >= U.count()) return p;
    int B = v.ref;
    const Body& b = U.bodies[B];
    const Body& tb = U.bodies[target];
    auto msg = [&](const char* m) {
        p.phase = Phase::Message;
        p.message = m;
        return p;
    };
    if (v.destroyed) return p;
    if (tb.isStar) return msg("A star cannot be orbited closely: target a planet or a moon");
    if (U.bodies[B].root != tb.root) return msg("The target is in another star system: travel there with the hyperspace drive");
    p.parkAlt = ascentTargetAlt(b);
    Conic c = stateToConic(v.r, v.v, b.GM);
    double soi = b.parent >= 0 ? b.soi : 1e300;
    bool closed = c.valid && c.e < 1 && c.ap < soi;
    p.escaping = !closed;
    p.ap = closed ? c.ap - b.radius : -1;
    p.pe = c.valid ? c.pe - b.radius : 0;
    bool inOrbit = closed && c.pe > b.radius + safeAlt(b);

    // ---- where to go next: a child of the current body, a sibling, or the parent
    enum Kind { None, Child, Sibling, Parent } kind = None;
    int G = -1;
    if (target != B) {
        if ((G = childToward(U, B, target)) >= 0) kind = Child;
        else if (b.parent >= 0) {
            int s = childToward(U, b.parent, target);
            if (s >= 0 && s != B) {
                G = s;
                kind = Sibling;
            } else {
                G = b.parent;
                kind = Parent;
            }
        }
    }
    p.goal = target == B ? B : G;

    // ---- on the ground
    if (v.landed) {
        p.phase = Phase::Prelaunch;
        if (target == B) return msg("The vessel is on the target already");
        if (kind == Child) {
            double tH = hohmannTime(b.GM, b.radius + p.parkAlt, U.bodies[G].a);
            p.tLaunch = moonLaunchWindow(W, v, B, G, 3600 + tH);
        } else if (kind == Sibling) {
            Xfer x = planetTransfer(W, B, G);
            if (x.ok) {
                p.transferWindow = true;
                p.tLaunch = std::max(W.t, x.tDep - 0.25 * DAY); // (lift off some hours ahead: the parking orbit first)
                p.tWindow = x.tDep;
                p.tArrive = x.tDep + x.tof;
            }
        }
        return p; // (Parent: a return home, launch any time)
    }

    // ---- arriving at the current body from outside: the capture at periapsis
    // (only where the trip is meant to stop - the target, or the planet of a target moon: just after a departure burn
    // the vessel is on a hyperbola past its periapsis too, and that is no arrival)
    bool stopHere = target == B || childToward(U, B, target) >= 0;
    bool inbound = dot(v.r, v.v) < 0 || (c.valid && c.nu >= 0 && c.nu < 0.3); // (or just past the periapsis: the capture burn under way)
    if (!closed && inbound && c.valid && stopHere) {
        p.phase = Phase::Arrival;
        p.impact = c.pe < b.radius + (b.atmTop > 0 ? 0.5 * b.atmTop : 0);
        double tPe = timeToTrueAnomaly(c, 0);
        if (tPe < 0 && c.nu > 0 && c.nu < 0.3) tPe = 0; // (just past the periapsis, during the burn: still now)
        if (tPe >= 0) {
            Vec3d rr = v.r, vv = v.v;
            keplerPropagate(rr, vv, b.GM, tPe);
            double rp = len(rr);
            p.tBurn = W.t + tPe;
            p.dv = len(vv) - std::sqrt(b.GM / rp); // to a circular orbit at the periapsis
            p.burn = -norm(vv) * p.dv;
        }
        if (p.impact) p.message = b.atmTop > 0 ? "The periapsis is deep in the atmosphere: raise it with a small radial burn, unless you mean to land"
                                               : "Collision course: raise the periapsis with a small radial burn before the capture";
        return p;
    }

    // ---- climbing from the surface (at the target itself: an orbit that dips too low)
    if (closed && !inOrbit) {
        if (target == B) {
            p.phase = Phase::Arrival;
            p.impact = true;
            p.message = "The periapsis is below the safe height: raise it with a prograde burn at the apoapsis, unless you mean to land";
            return p;
        }
        p.phase = Phase::Ascent;
        return p;
    }

    // ---- in orbit around the target: done
    if (target == B) {
        if (inOrbit) p.phase = Phase::InOrbit;
        else p.phase = Phase::Transfer; // (leaving it on an escape path)
        return p;
    }
    if (kind == None) return msg("No route to the target from here");

    // ---- in orbit (or escaping) around the current body: the transfer to the goal
    const Body& gb = U.bodies[G];
    if (kind == Child) {
        Vec3d rg, vg;
        relState(U, W.t, G, B, rg, vg);
        double rG = len(rg);
        bool reaches = !closed || (c.ap >= rG * 0.85 && c.pe <= rG * 1.15);
        Encounter e;
        bool enc = predictEncounter(U, B, v.r, v.v, W.t, e);
        if (enc && e.body == G) {
            p.phase = Phase::Transfer;
            p.encounter = true;
            p.tEncounter = W.t + e.tEntry;
            Conic ce = stateToConic(e.r, e.v, gb.GM);
            p.encPe = ce.pe - gb.radius;
            p.impact = p.encPe < (gb.atmTop > 0 ? 0.5 * gb.atmTop : 0);
            if (p.impact) p.message = "The path hits the target: a small radial or normal burn raises the periapsis";
            return p;
        }
        // the transfer: Lambert from the vessel's own path, aimed beside the goal (the periapsis of the flyby at
        // the goal's parking orbit; passing on the side that gives a prograde orbit)
        double tH = hohmannTime(b.GM, len(v.r), rG);
        auto dep = [&](double t, Vec3d& r, Vec3d& vv) {
            r = v.r;
            vv = v.v;
            keplerPropagate(r, vv, b.GM, t - W.t);
        };
        double span = reaches ? 0 : (closed ? std::min(c.period, 2.0 * YEAR) : 0);
        Xfer x;
        if (reaches) x = bestTransfer(U, B, G, W.t, 0, 0.3 * tH, 2.0 * tH, dep, 1, 60);
        else if (g_lastDep.vessel == v.id && g_lastDep.goal == G && closed && g_lastDep.t - W.t < 0.08 * c.period && W.t - g_lastDep.t < 0.05 * c.period) {
            // the burn is due (or under way): depart now
            x = bestTransfer(U, B, G, W.t, 0, 0.6 * tH, 1.4 * tH, dep, 1, 48);
        } else x = bestTransfer(U, B, G, W.t, span, 0.6 * tH, 1.4 * tH, dep, 72, 24);
        if (!reaches && x.ok) g_lastDep = {v.id, G, x.tDep};
        if (!x.ok) return msg("No transfer found: raise or lower the orbit and try again");
        {
            Vec3d vInf = x.v2 - x.vGoal;
            double vi = std::max(len(vInf), 1.0), rp = gb.radius + ascentTargetAlt(gb);
            double bImpact = rp * std::sqrt(1 + 2 * gb.GM / (rp * vi * vi));
            Vec3d n = cross(vInf, cross(x.r1, x.v1));
            if (len(n) > 1e-9) {
                n = norm(n);
                Vec3d r2c, vg2;
                relState(U, x.tDep + x.tof, G, B, r2c, vg2);
                // the aim offset: from the impact parameter of the flyby hyperbola, then refined on the patched-conic
                // prediction itself (the closed form ignores the pull of the goal before its sphere and of the parent
                // inside it): the secant method on periapsis(offset) - target
                // (an aim that costs much more than the transfer itself is no answer: the iteration keeps to cheap ones)
                const double dvMax = len(x.v1 - x.vDep) * 1.10 + 50;
                auto peFor = [&](double off, Vec3d& v1out) {
                    Vec3d v1, v2;
                    if (!navg::lambert(x.r1, r2c + n * off, x.tof, b.GM, cross(x.r1, x.vDep), v1, v2)) return -1e30;
                    if (len(v1 - x.vDep) > dvMax) return -1e30;
                    v1out = v1;
                    Encounter e;
                    if (!predictEncounter(U, B, x.r1, v1, x.tDep, e) || e.body != G) return -1e30;
                    return stateToConic(e.r, e.v, gb.GM).pe;
                };
                double o0 = bImpact, o1 = 0.6 * bImpact;
                const double tol = std::max(1e3, 0.15 * (rp - gb.radius)); // (5 km for a 35 km lunar orbit)
                Vec3d v0, v1;
                double f0 = peFor(o0, v0) - rp, f1 = peFor(o1, v1) - rp;
                Vec3d vBest = f0 > -1e29 ? v0 : x.v1;
                double fBest = f0 > -1e29 ? std::fabs(f0) : 1e300;
                if (f1 > -1e29 && std::fabs(f1) < fBest) { vBest = v1; fBest = std::fabs(f1); }
                for (int it = 0; it < 6 && f0 > -1e29 && f1 > -1e29 && std::fabs(f1 - f0) > 1e-3 && fBest > tol; it++) {
                    double o2 = std::max(0.0, o1 - f1 * (o1 - o0) / (f1 - f0));
                    Vec3d v2;
                    double f2 = peFor(o2, v2) - rp;
                    if (f2 < -1e29) break;
                    if (std::fabs(f2) < fBest) { vBest = v2; fBest = std::fabs(f2); }
                    o0 = o1; f0 = f1; o1 = o2; f1 = f2;
                }
                x.v1 = vBest;
            }
        }
        Vec3d dvv = x.v1 - x.vDep;
        p.phase = reaches ? Phase::Transfer : Phase::Departure;
        p.correction = reaches;
        p.tBurn = x.tDep;
        p.dv = len(dvv);
        p.burn = dvv;
        p.tArrive = x.tDep + x.tof;
        if (reaches && p.dv < 0.5) {
            p.tBurn = -1;
            p.burn = {};
            p.message = "Coasting: no encounter predicted yet";
        }
        return p;
    }

    // ---- Sibling (to another planet) or Parent (home from a moon): an ejection from the parking orbit
    if (!closed) {
        // already escaping: on the way out of the sphere of influence
        p.phase = Phase::Transfer;
        p.message = "Leaving the sphere of influence: the course toward the goal follows outside it";
        return p;
    }
    Vec3d vInf;
    p.phase = Phase::Departure;
    if (kind == Sibling) {
        Xfer x = planetTransfer(W, B, G);
        if (!x.ok) return msg("No transfer window found");
        vInf = x.v1 - x.vDep;
        p.tWindow = x.tDep;
        p.tArrive = x.tDep + x.tof;
    } else {
        // home: down from the moon's orbit to a low orbit (or the air) of the parent
        const Body& P = U.bodies[G];
        Vec3d rb, vb;
        relState(U, W.t, B, G, rb, vb);
        double rB = len(rb), rLow = P.radius + (P.atmTop > 0 ? 0.6 * P.atmTop : ascentTargetAlt(P));
        double vApo = std::sqrt(P.GM * 2 * rLow / (rB * (rB + rLow)));
        vInf = norm(vb) * (vApo - len(vb));
        p.tWindow = W.t;
        p.tArrive = W.t + hohmannTime(P.GM, rB, rLow);
    }
    // the burn: at the next pass of the ejection point once the window is open
    Vec3d r = v.r, vv = v.v;
    double tStart = std::max(W.t, p.tWindow - 0.5 * c.period);
    keplerPropagate(r, vv, b.GM, tStart - W.t);
    Conic cs = stateToConic(r, vv, b.GM);
    Ejection ej = ejection(cs, r, vv, b.GM, vInf);
    if (!ej.ok) return msg("No departure point found on this orbit");
    p.tBurn = tStart + ej.tWait;
    p.dv = ej.dv;
    p.burn = ej.dir * ej.dv;
    return p;
}

} // namespace

void invalidate() {
    g_cache = Cache{};
    g_lastDep = LastDep{};
}

Plan compute(const World& W, const Vessel& v, int target) {
    // the searches take a few milliseconds: the plan is kept for half a second of real time (its times are
    // absolute, so the countdowns stay right), longer on the ground; a change of situation renews it at once
    int phaseKey = v.landed ? 0 : 1;
    double keep = v.landed ? 1.0 : 0.5;
    if (g_cache.vessel == v.id && g_cache.target == target && g_cache.ref == v.ref && g_cache.phase == phaseKey &&
        W.realTime - g_cache.real >= 0 && W.realTime - g_cache.real < keep)
        return g_cache.plan;
    Plan p = computeNow(W, v, target);
    g_cache = {v.id, target, v.ref, phaseKey, W.realTime, p};
    return p;
}

} // namespace mission
