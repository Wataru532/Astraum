#include "orbit.hpp"
#include <algorithm>

Conic stateToConic(const Vec3d& r, const Vec3d& v, double mu) {
    Conic c;
    c.mu = mu;
    double rl = len(r);
    if (rl < 1e-6 || mu <= 0) return c;
    Vec3d h = cross(r, v);
    double hl = len(h);
    c.energy = len2(v) * 0.5 - mu / rl;
    Vec3d ev = cross(v, h) / mu - r / rl;
    c.e = len(ev);
    c.p = hl * hl / mu;
    if (hl < 1e-9) {
        // radial trajectory: degenerate, approximate
        c.H = anyPerp(r);
        c.P = norm(r);
        c.Q = cross(c.H, c.P);
        c.e = 1.0;
        c.a = std::fabs(c.energy) > 1e-12 ? -mu / (2 * c.energy) : 1e300;
        c.pe = 0;
        c.ap = c.energy < -1e-12 && std::isfinite(c.a) ? 2 * c.a : -1;
        c.nu = 0;
        c.valid = true;
        return c;
    }
    c.H = h / hl;
    c.inc = std::acos(clampd(c.H.z, -1, 1));
    if (c.e > 1e-9) c.P = ev / c.e;
    else c.P = norm(r);
    c.Q = cross(c.H, c.P);
    c.nu = std::atan2(dot(r, c.Q), dot(r, c.P));
    if (std::fabs(c.e - 1.0) > 1e-9) c.a = c.p / (1 - c.e * c.e);
    else c.a = 1e300;
    c.pe = c.p / (1 + c.e);
    if (c.e < 1 && c.a > 0 && c.a < 1e100) {
        c.ap = c.p / (1 - c.e);
        double T = TAU * std::sqrt(c.a / mu) * c.a;
        c.period = std::isfinite(T) ? T : 0;
    } else {
        c.ap = -1;
        c.period = 0;
    }
    c.valid = true;
    return c;
}

Vec3d conicPoint(const Conic& c, double nu) {
    double r = c.p / (1 + c.e * std::cos(nu));
    return c.P * (r * std::cos(nu)) + c.Q * (r * std::sin(nu));
}

static double meanAnomalyFromTrue(const Conic& c, double nu) {
    if (c.e < 1) {
        double E = 2 * std::atan2(std::sqrt(1 - c.e) * std::sin(nu / 2), std::sqrt(1 + c.e) * std::cos(nu / 2));
        return E - c.e * std::sin(E);
    }
    double arg = std::sqrt((c.e - 1) / (c.e + 1)) * std::tan(nu / 2);
    if (!std::isfinite(arg) || std::fabs(arg) >= 1.0) return 0;
    double F = 2 * std::atanh(arg);
    return c.e * std::sinh(F) - F;
}

double timeToTrueAnomaly(const Conic& c, double nuTarget) {
    if (!c.valid || c.mu <= 0) return -1;
    if (c.e < 1 - 1e-9 && c.a > 0 && c.a < 1e100) {
        double n = std::sqrt(c.mu / c.a) / c.a;
        if (!(n > 0) || !std::isfinite(n)) return -1;
        double M0 = meanAnomalyFromTrue(c, c.nu), M1 = meanAnomalyFromTrue(c, nuTarget);
        double dM = wrapAngle(M1 - M0);
        double dt = dM / n;
        return std::isfinite(dt) ? dt : -1;
    }
    if (std::fabs(c.e - 1.0) <= 1e-9) {
        // parabolic (Barker's equation): only if non-radial (p > 0) and within (-pi, pi)
        if (c.p <= 1e-9 || std::fabs(nuTarget) >= PI - 1e-6 || std::fabs(c.nu) >= PI - 1e-6) return -1;
        double D0 = std::tan(c.nu * 0.5), D1 = std::tan(nuTarget * 0.5);
        double dt = 0.5 * std::sqrt(c.p * c.p * c.p / c.mu) * ((D1 + D1 * D1 * D1 / 3.0) - (D0 + D0 * D0 * D0 / 3.0));
        return (std::isfinite(dt) && dt >= 0) ? dt : -1;
    }
    // hyperbolic: only reachable if within asymptote limits
    if (!(c.a < 0) || !std::isfinite(c.a) || c.e <= 1.0) return -1;
    double nuInf = std::acos(clampd(-1.0 / c.e, -1.0, 1.0));
    if (std::fabs(nuTarget) >= nuInf - 1e-9 || std::fabs(c.nu) >= nuInf - 1e-9) return -1;
    double ma = -c.a;
    double n = std::sqrt(c.mu / ma) / ma;
    if (!(n > 0) || !std::isfinite(n)) return -1;
    double M0 = meanAnomalyFromTrue(c, c.nu), M1 = meanAnomalyFromTrue(c, nuTarget);
    double dt = (M1 - M0) / n;
    return (std::isfinite(dt) && dt >= 0) ? dt : -1;
}

static void stumpff(double z, double& C, double& S) {
    if (z > 1e-6) {
        double sz = std::sqrt(z);
        C = (1 - std::cos(sz)) / z;
        S = (sz - std::sin(sz)) / (sz * sz * sz);
    } else if (z < -1e-6) {
        double sz = std::sqrt(-z);
        C = (std::cosh(sz) - 1) / (-z);
        S = (std::sinh(sz) - sz) / (sz * sz * sz);
    } else {
        C = 0.5 - z / 24.0 + z * z / 720.0;
        S = 1.0 / 6.0 - z / 120.0 + z * z / 5040.0;
    }
}

void keplerPropagate(Vec3d& R0, Vec3d& V0, double mu, double dt) {
    if (dt == 0 || mu <= 0) return;
    double r0 = len(R0), v0 = len(V0);
    if (r0 < 1e-3) return;
    double alpha = 2.0 / r0 - v0 * v0 / mu;
    // reduce elliptical propagation to less than one period
    if (alpha > 1e-15) {
        double a = 1.0 / alpha;
        double T = TAU * std::sqrt(a * a * a / mu);
        if (std::fabs(dt) > T) dt = std::fmod(dt, T);
    }
    double smu = std::sqrt(mu);
    double vr0 = dot(R0, V0) / r0;
    double sgn = dt >= 0 ? 1.0 : -1.0;
    // The universal Kepler equation F(chi) = 0 rises monotonically (dF/dchi = r > 0), so the root is
    // bracketed and Newton steps that leave the bracket fall back to bisection. (Plain Newton from the
    // initial guess ran away on a hyperbola far out from the planet - one step of a fast warp threw a vessel
    // leaving Earth to 1e32 m - and stalled on a wrong value in short steps there: the escape came late.)
    // Bracket: r >= the periapsis on the whole path, so |chi| <= sqrt(mu)|dt| / pe; an ellipse reduced to less
    // than one period also has |chi| < 2 pi sqrt(a).
    Vec3d hv = cross(R0, V0);
    double pe = r0;
    {
        Vec3d ev = cross(V0, hv) / mu - R0 / r0;
        double e = len(ev), p = len2(hv) / mu;
        if (p > 0) pe = std::min(r0, p / (1 + e));
    }
    double bound = smu * std::fabs(dt) / std::max(pe, 1e-3);
    if (alpha > 1e-15) bound = std::min(bound, TAU / std::sqrt(alpha) * 1.000001);
    double lo = sgn > 0 ? 0 : -bound, hi = sgn > 0 ? bound : 0;
    double chi;
    if (alpha > 1e-15) chi = smu * alpha * dt;
    else if (alpha < -1e-15) {
        double a = 1.0 / alpha;
        double arg = (-2 * mu * alpha * dt) / (dot(R0, V0) + sgn * std::sqrt(-mu * a) * (1 - r0 * alpha));
        chi = sgn * std::sqrt(-a) * std::log(std::max(arg, 1e-12));
    } else chi = smu * dt / r0;
    if (!std::isfinite(chi) || chi <= lo || chi >= hi) chi = 0.5 * (lo + hi);
    double C = 0, S = 0;
    for (int it = 0; it < 200; it++) {
        double z = alpha * chi * chi;
        stumpff(z, C, S);
        double F = r0 * vr0 / smu * chi * chi * C + (1 - alpha * r0) * chi * chi * chi * S + r0 * chi - smu * dt;
        double dF = r0 * vr0 / smu * chi * (1 - alpha * chi * chi * S) + (1 - alpha * r0) * chi * chi * C + r0;
        if (!std::isfinite(F)) F = chi > 0 ? 1e300 : -1e300; // (cosh overflow: far beyond the root)
        if (F == 0) break;
        (F < 0 ? lo : hi) = chi;
        double next = chi - F / dF;
        if (!std::isfinite(next) || next <= lo || next >= hi) next = 0.5 * (lo + hi);
        double step = next - chi;
        chi = next;
        if (std::fabs(step) < 1e-13 * std::max(1.0, std::fabs(chi)) || hi - lo < 1e-15 * std::max(1.0, std::fabs(chi))) break;
    }
    double z = alpha * chi * chi;
    stumpff(z, C, S);
    double f = 1 - chi * chi / r0 * C;
    double g = dt - chi * chi * chi * S / smu;
    Vec3d R = R0 * f + V0 * g;
    double r = len(R);
    if (!(r > 1e-6) || !std::isfinite(r)) return;
    double fdot = smu / (r * r0) * (alpha * chi * chi * chi * S - chi);
    double gdot = 1 - chi * chi / r * C;
    Vec3d V = R0 * fdot + V0 * gdot;
    auto fin3 = [](const Vec3d& u) { return std::isfinite(u.x) && std::isfinite(u.y) && std::isfinite(u.z); };
    if (fin3(R) && fin3(V)) { R0 = R; V0 = V; }
}
