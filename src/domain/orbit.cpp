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
        c.a = -mu / (2 * c.energy);
        c.pe = 0;
        c.ap = c.energy < 0 ? 2 * c.a : -1;
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
    if (c.e < 1) {
        c.ap = c.p / (1 - c.e);
        c.period = TAU * std::sqrt(c.a * c.a * c.a / mu);
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
    double F = 2 * std::atanh(std::sqrt((c.e - 1) / (c.e + 1)) * std::tan(nu / 2));
    return c.e * std::sinh(F) - F;
}

double timeToTrueAnomaly(const Conic& c, double nuTarget) {
    if (!c.valid) return -1;
    if (c.e < 1) {
        double n = std::sqrt(c.mu / (c.a * c.a * c.a));
        double M0 = meanAnomalyFromTrue(c, c.nu), M1 = meanAnomalyFromTrue(c, nuTarget);
        double dM = wrapAngle(M1 - M0);
        return dM / n;
    }
    // hyperbolic: only reachable if within asymptote limits
    double nuInf = std::acos(-1.0 / c.e);
    if (std::fabs(nuTarget) >= nuInf) return -1;
    double n = std::sqrt(c.mu / (-c.a * -c.a * -c.a));
    double M0 = meanAnomalyFromTrue(c, c.nu), M1 = meanAnomalyFromTrue(c, nuTarget);
    double dt = (M1 - M0) / n;
    return dt >= 0 ? dt : -1;
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
    double chi;
    if (alpha > 1e-15) chi = smu * alpha * dt;
    else if (alpha < -1e-15) {
        double a = 1.0 / alpha;
        double sgn = dt >= 0 ? 1.0 : -1.0;
        double arg = (-2 * mu * alpha * dt) / (dot(R0, V0) + sgn * std::sqrt(-mu * a) * (1 - r0 * alpha));
        chi = sgn * std::sqrt(-a) * std::log(std::max(arg, 1e-12));
        if (!std::isfinite(chi)) chi = smu * std::fabs(alpha) * dt;
    } else chi = smu * dt / r0;
    double C = 0, S = 0;
    for (int it = 0; it < 60; it++) {
        double z = alpha * chi * chi;
        stumpff(z, C, S);
        double F = r0 * vr0 / smu * chi * chi * C + (1 - alpha * r0) * chi * chi * chi * S + r0 * chi - smu * dt;
        double dF = r0 * vr0 / smu * chi * (1 - alpha * chi * chi * S) + (1 - alpha * r0) * chi * chi * C + r0;
        if (std::fabs(dF) < 1e-300) break;
        double step = F / dF;
        chi -= step;
        if (std::fabs(step) < 1e-9 * std::max(1.0, std::fabs(chi))) break;
    }
    double z = alpha * chi * chi;
    stumpff(z, C, S);
    double f = 1 - chi * chi / r0 * C;
    double g = dt - chi * chi * chi * S / smu;
    Vec3d R = R0 * f + V0 * g;
    double r = len(R);
    double fdot = smu / (r * r0) * (alpha * chi * chi * chi * S - chi);
    double gdot = 1 - chi * chi / r * C;
    Vec3d V = R0 * fdot + V0 * gdot;
    if (std::isfinite(R.x) && std::isfinite(V.x)) { R0 = R; V0 = V; }
}
