#include "craft.hpp"
#include "brand.hpp"
#include "i18n.hpp"
#include <algorithm>
#include <atomic>
#include <charconv>
#include <cstdio>
#include <list>
#include <mutex>
#include <sstream>
#include <thread>

const Substance MATERIALS[] = {
    {"Aluminium-lithium 2195", 2700, 2.5e5, {205, 208, 214, 255}},
    {"Titanium Ti-6Al-4V", 4430, 4.5e5, {168, 170, 178, 255}},
    {"Stainless steel 304L", 7900, 6.0e5, {222, 224, 228, 255}},
    {"Carbon fibre composite", 1600, 1.5e5, {52, 55, 62, 255}},
    {"Inconel 718", 8190, 8.0e5, {158, 150, 140, 255}},
    {"Reinforced carbon-carbon", 1900, 2.5e6, {66, 66, 70, 255}},
    {"PICA ablator", 270, 1.5e7, {84, 60, 42, 255}},
    // silica tiles / ceramic-matrix composite skin of winged re-entry craft (Space Shuttle HRSI ~1,260 C,
    // Skylon CMC): reusable, no ablation (appended: materials are saved by index)
    {"Ceramic TPS (tiles / CMC)", 450, 1.6e6, {38, 38, 40, 255}},
    // window panes: fused silica / aluminosilicate glass (the orbiter's outer panes take the entry heating of the
    // forward fuselage); also acrylic canopies (appended)
    {"Glass (fused silica)", 2200, 1.0e6, {40, 52, 62, 255}},
};
const int MATERIAL_COUNT = sizeof(MATERIALS) / sizeof(MATERIALS[0]);
static const int MAT_PICA = 6;

const char* shapeName(Shape s) {
    switch (s) {
    case Shape::Box: return "Box";
    case Shape::Cylinder: return "Cylinder / cone";
    case Shape::Ellipsoid: return "Ellipsoid";
    case Shape::Wedge: return "Wedge";
    case Shape::Airfoil: return "Airfoil";
    case Shape::Loft: return "Fuselage";
    case Shape::Truss: return "Truss";
    default: return "";
    }
}
const char* funcName(Func f) {
    switch (f) {
    case Func::Structure: return "Structure";
    case Func::Tank: return "Propellant tank";
    case Func::Engine: return "Engine";
    case Func::Command: return "Command";
    case Func::Battery: return "Battery";
    case Func::Solar: return "Solar panel";
    case Func::RTG: return "RTG";
    case Func::Reactor: return "Reactor";
    case Func::CMG: return "Gyroscope";
    case Func::Parachute: return "Parachute";
    case Func::Legs: return "Landing leg";
    case Func::Sail: return "Solar sail";
    case Func::Hyperdrive: return "Hyperspace drive";
    case Func::HeatShield: return "Heat shield";
    case Func::Wing: return "Wing / fin";
    case Func::RCS: return "RCS thrusters";
    case Func::Fairing: return "Payload fairing";
    case Func::Rotor: return "Rotor / propeller";
    case Func::Jet: return "Jet engine";
    case Func::Docking: return "Docking port";
    case Func::Window: return "Window";
    case Func::CargoBay: return "Payload bay";
    case Func::Wheel: return "Landing gear";
    default: return "";
    }
}

double trussTubeRadius(const Vec3d& d) {
    // the section: the two sides other than the longest
    double mx = std::max({d.x, d.y, d.z});
    double a = d.x == mx ? std::max(d.y, d.z) : (d.y == mx ? std::max(d.x, d.z) : std::max(d.x, d.y));
    return std::clamp(0.035 * a, 0.012, 0.15);
}

void trussMembers(const Vec3d& d, std::vector<std::pair<Vec3d, Vec3d>>& out) {
    out.clear();
    // the long axis and the two across it
    int la = d.x >= d.y && d.x >= d.z ? 0 : (d.y >= d.z ? 1 : 2);
    Vec3d ax[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double dd[3] = {d.x, d.y, d.z};
    Vec3d u = ax[la], v = ax[(la + 1) % 3], w = ax[(la + 2) % 3];
    double L = dd[la], r = trussTubeRadius(d);
    double hv = std::max(dd[(la + 1) % 3] / 2 - r, 0.0), hw = std::max(dd[(la + 2) % 3] / 2 - r, 0.0);
    double sec = std::max(dd[(la + 1) % 3], dd[(la + 2) % 3]);
    int n = std::clamp((int)std::lround(L / std::max(sec, 0.05)), 1, 300);
    auto corner = [&](int j, int k) {
        static const int sv[4] = {1, -1, -1, 1}, sw[4] = {1, 1, -1, -1};
        double s = -L / 2 + L * k / n;
        return u * s + v * (sv[j & 3] * hv) + w * (sw[j & 3] * hw);
    };
    for (int k = 0; k <= n; k++)
        for (int j = 0; j < 4; j++) {
            out.push_back({corner(j, k), corner(j + 1, k)}); // frame
            if (k == n) continue;
            out.push_back({corner(j, k), corner(j, k + 1)}); // longeron
            // diagonal on face j, alternating direction from bay to bay (a Warren-like pattern)
            if (k % 2 == 0) out.push_back({corner(j, k), corner(j + 1, k + 1)});
            else out.push_back({corner(j + 1, k), corner(j, k + 1)});
        }
}

std::vector<Vec3d> clusterLayout(int n, double plateR, double& bellR, double exitR) {
    std::vector<Vec3d> pos;
    bellR = exitR;
    if (n <= 1) { pos.push_back({0, 0, 0}); bellR = std::min(exitR, plateR * 0.95); return pos; }
    int ring = n <= 7 ? n : n - 1;
    double rr = plateR * (n <= 7 ? 0.55 : 0.66);
    if (n > 7) pos.push_back({0, 0, 0});
    for (int i = 0; i < ring; i++) {
        double a = (double)i / ring * TAU;
        pos.push_back({std::cos(a) * rr, std::sin(a) * rr, 0});
    }
    double maxR = std::min(rr * std::sin(PI / ring) * 0.98, plateR - rr);
    if (n > 7) maxR = std::min(maxR, rr * 0.5);
    bellR = std::min(exitR, std::max(maxR, 0.1));
    return pos;
}

int engineCount(const Component& c) {
    if (c.func == Func::Engine && c.def >= 0 && CATALOG.defs[c.def].kind == PK::Engine) return std::clamp(c.cluster, 1, 37);
    return 1;
}
int enginesOn(const Component& c) {
    int n = engineCount(c), on = 0;
    for (int i = 0; i < n; i++) on += !((c.engOff >> i) & 1);
    return on;
}

Quatd Component::rot() const {
    Quatd qx = Quatd::axisAngle({1, 0, 0}, rotDeg.x * DEG);
    Quatd qy = Quatd::axisAngle({0, 1, 0}, rotDeg.y * DEG);
    Quatd qz = Quatd::axisAngle({0, 0, 1}, rotDeg.z * DEG);
    return (qz * qy * qx).normalized();
}

// ===================================================================== geometry
double shapeVolume(Shape s, const Vec3d& d) {
    if (s == Shape::Loft) return shapeVolume(Shape::Ellipsoid, d * 0.5);
    switch (s) {
    case Shape::Box:
    case Shape::Truss:
    case Shape::Airfoil: return d.x * d.y * d.z;
    case Shape::Cylinder: return PI * d.z / 3.0 * (d.x * d.x + d.x * d.y + d.y * d.y);
    case Shape::Ellipsoid: return 4.0 / 3.0 * PI * d.x * d.y * d.z;
    case Shape::Wedge: return 0.5 * d.x * d.y * d.z;
    default: return 0;
    }
}
double shapeArea(Shape s, const Vec3d& d) {
    if (s == Shape::Loft) return shapeArea(Shape::Ellipsoid, d * 0.5);
    switch (s) {
    case Shape::Box:
    case Shape::Truss:
    case Shape::Airfoil: return 2 * (d.x * d.y + d.y * d.z + d.z * d.x);
    case Shape::Cylinder: {
        double sl = std::sqrt((d.x - d.y) * (d.x - d.y) + d.z * d.z);
        return PI * (d.x + d.y) * sl + PI * (d.x * d.x + d.y * d.y);
    }
    case Shape::Ellipsoid: {
        double p = 1.6075, a = d.x, b = d.y, c = d.z;
        return 4 * PI * std::pow((std::pow(a * b, p) + std::pow(a * c, p) + std::pow(b * c, p)) / 3.0, 1.0 / p);
    }
    case Shape::Wedge: return d.x * d.z + d.y * (d.x + d.z + std::sqrt(d.x * d.x + d.z * d.z));
    default: return 0;
    }
}
Vec3d shapeHalfExtent(Shape s, const Vec3d& d) {
    if (s == Shape::Loft) return d * 0.5;
    switch (s) {
    case Shape::Box:
    case Shape::Truss:
    case Shape::Airfoil:
    case Shape::Wedge: return d * 0.5;
    case Shape::Cylinder: { double r = std::max(d.x, d.y); return {r, r, d.z * 0.5}; }
    case Shape::Ellipsoid: return d;
    default: return d;
    }
}
Vec3d shapeCentroid(Shape s, const Vec3d& d) {
    if (s == Shape::Wedge) return {-d.x / 6.0, 0, -d.z / 6.0};
    if (s == Shape::Cylinder) {
        double r1 = d.x, r2 = d.y, h = d.z;
        double den = r1 * r1 + r1 * r2 + r2 * r2;
        if (den < 1e-12) return {};
        double zc = h * (r1 * r1 + 2 * r1 * r2 + 3 * r2 * r2) / (4 * den); // from the bottom
        return {0, 0, zc - h / 2};
    }
    return {};
}

namespace {
struct Plane { Vec3d n; double d; };
void polyPlanes(Shape s, const Vec3d& dm, std::vector<Plane>& P) {
    P.clear();
    Vec3d h = dm * 0.5;
    if (s == Shape::Box || s == Shape::Airfoil || s == Shape::Truss) {
        P.push_back({{1, 0, 0}, h.x}); P.push_back({{-1, 0, 0}, h.x});
        P.push_back({{0, 1, 0}, h.y}); P.push_back({{0, -1, 0}, h.y});
        P.push_back({{0, 0, 1}, h.z}); P.push_back({{0, 0, -1}, h.z});
    } else { // wedge
        P.push_back({{-1, 0, 0}, h.x}); P.push_back({{0, 0, -1}, h.z});
        P.push_back({{0, 1, 0}, h.y}); P.push_back({{0, -1, 0}, h.y});
        Vec3d n = norm(Vec3d{dm.z, 0, dm.x});
        // plane through (h.x, 0, -h.z)
        P.push_back({n, dot(n, Vec3d{h.x, 0, -h.z})});
    }
}
bool rayPoly(const std::vector<Plane>& P, const Vec3d& o, const Vec3d& dir, double& t) {
    double tmin = -1e300, tmax = 1e300;
    for (const auto& p : P) {
        double den = dot(p.n, dir), num = p.d - dot(p.n, o);
        if (std::fabs(den) < 1e-14) {
            if (num < 0) return false;
            continue;
        }
        double tt = num / den;
        if (den < 0) tmin = std::max(tmin, tt);
        else tmax = std::min(tmax, tt);
        if (tmin > tmax) return false;
    }
    if (tmax < 1e-9) return false;
    t = std::max(tmin, 0.0);
    return true;
}
} // namespace

bool shapeRay(Shape s, const Vec3d& d, const Vec3d& o, const Vec3d& dir, double& tHit) {
    if (s == Shape::Loft) return shapeRay(Shape::Ellipsoid, d * 0.5, o, dir, tHit);
    switch (s) {
    case Shape::Box:
    case Shape::Truss:
    case Shape::Airfoil:
    case Shape::Wedge: {
        std::vector<Plane> P;
        polyPlanes(s, d, P);
        return rayPoly(P, o, dir, tHit);
    }
    case Shape::Ellipsoid: {
        Vec3d so{o.x / d.x, o.y / d.y, o.z / d.z}, sd{dir.x / d.x, dir.y / d.y, dir.z / d.z};
        double a = dot(sd, sd), b = 2 * dot(so, sd), c = dot(so, so) - 1;
        double disc = b * b - 4 * a * c;
        if (disc < 0 || a < 1e-300) return false;
        double sq = std::sqrt(disc);
        double t0 = (-b - sq) / (2 * a), t1 = (-b + sq) / (2 * a);
        if (t1 < 1e-9) return false;
        tHit = t0 >= 0 ? t0 : 0;
        return true;
    }
    case Shape::Cylinder: {
        double rB = d.x, rT = d.y, h = d.z;
        double bb = (rT - rB) / h, aa = rB + bb * h / 2;
        double best = 1e300;
        bool inside = false;
        { // origin inside?
            double r = aa + bb * o.z;
            if (std::fabs(o.z) <= h / 2 && o.x * o.x + o.y * o.y <= r * r) inside = true;
        }
        if (inside) { tHit = 0; return true; }
        double A = dir.x * dir.x + dir.y * dir.y - bb * bb * dir.z * dir.z;
        double B = 2 * (o.x * dir.x + o.y * dir.y - bb * dir.z * (aa + bb * o.z));
        double C = o.x * o.x + o.y * o.y - (aa + bb * o.z) * (aa + bb * o.z);
        auto tryT = [&](double t) {
            if (t < 1e-9 || t >= best) return;
            double z = o.z + t * dir.z;
            if (std::fabs(z) > h / 2) return;
            if (aa + bb * z < 0) return;
            best = t;
        };
        if (std::fabs(A) > 1e-14) {
            double disc = B * B - 4 * A * C;
            if (disc >= 0) {
                double sq = std::sqrt(disc);
                tryT((-B - sq) / (2 * A));
                tryT((-B + sq) / (2 * A));
            }
        } else if (std::fabs(B) > 1e-14) tryT(-C / B);
        for (int cap = 0; cap < 2; cap++) {
            double zc = cap ? h / 2 : -h / 2, rc = cap ? rT : rB;
            if (std::fabs(dir.z) < 1e-14) continue;
            double t = (zc - o.z) / dir.z;
            if (t < 1e-9 || t >= best) continue;
            double x = o.x + t * dir.x, y = o.y + t * dir.y;
            if (x * x + y * y <= rc * rc) best = t;
        }
        if (best < 1e299) { tHit = best; return true; }
        return false;
    }
    default: return false;
    }
}

static bool shapeInside(Shape s, const Vec3d& d, const Vec3d& p) {
    if (s == Shape::Loft) {
        // inside a fuselage: its sections keep their size over most of the length and close at the ends (closer to
        // the drawn body than an ellipsoid - parts carried inside it, such as tanks, are shielded from the flow)
        Vec3d h = d * 0.5;
        double t = p.z / std::max(h.z, 1e-9);
        if (std::fabs(t) > 1) return false;
        double f2 = std::max(0.0, 1 - t * t * t * t);
        return (p.x * p.x) / (h.x * h.x) + (p.y * p.y) / (h.y * h.y) <= f2;
    }
    switch (s) {
    case Shape::Box:
    case Shape::Airfoil: return std::fabs(p.x) <= d.x / 2 && std::fabs(p.y) <= d.y / 2 && std::fabs(p.z) <= d.z / 2;
    case Shape::Truss: return false; // (open frame: what is inside stays in the flow)
    case Shape::Ellipsoid: return (p.x * p.x) / (d.x * d.x) + (p.y * p.y) / (d.y * d.y) + (p.z * p.z) / (d.z * d.z) <= 1;
    case Shape::Cylinder: {
        if (std::fabs(p.z) > d.z / 2) return false;
        double r = d.x + (d.y - d.x) * (p.z + d.z / 2) / d.z;
        return p.x * p.x + p.y * p.y <= r * r;
    }
    case Shape::Wedge: {
        if (std::fabs(p.y) > d.y / 2 || p.x < -d.x / 2 || p.z < -d.z / 2) return false;
        return (p.x + d.x / 2) / d.x + (p.z + d.z / 2) / d.z <= 1;
    }
    default: return false;
    }
}

void shapeSurface(Shape s, const Vec3d& d, int res, std::vector<SurfSample>& out) {
    if (s == Shape::Loft) { shapeSurface(Shape::Ellipsoid, d * 0.5, res, out); return; }
    out.clear();
    res = std::max(res, 2);
    auto rectFace = [&](const Vec3d& c, const Vec3d& u, const Vec3d& v, double lu, double lv, const Vec3d& n) {
        double mx = std::max(lu, lv);
        int nu = std::max(1, (int)std::lround(res * lu / mx)), nv = std::max(1, (int)std::lround(res * lv / mx));
        double a = lu * lv / (nu * nv);
        for (int i = 0; i < nu; i++)
            for (int j = 0; j < nv; j++)
                out.push_back({c + u * (lu * ((i + 0.5) / nu - 0.5)) + v * (lv * ((j + 0.5) / nv - 0.5)), n, a});
    };
    auto triFace = [&](const Vec3d& A, const Vec3d& B, const Vec3d& C, const Vec3d& n) {
        int k = res;
        double area = 0.5 * len(cross(B - A, C - A)) / (k * k);
        for (int i = 0; i < k; i++)
            for (int j = 0; j < k - i; j++) {
                // upright small triangle
                Vec3d e1 = (B - A) / k, e2 = (C - A) / k;
                Vec3d p0 = A + e1 * (double)i + e2 * (double)j;
                out.push_back({p0 + (e1 + e2) / 3.0, n, area});                     // upright cell
                if (j < k - i - 1) out.push_back({p0 + (e1 + e2) * (2.0 / 3.0), n, area}); // inverted cell
            }
    };
    switch (s) {
    case Shape::Truss:
    case Shape::Box:
    case Shape::Airfoil: {
        Vec3d h = d * 0.5;
        size_t first = out.size();
        rectFace({h.x, 0, 0}, {0, 1, 0}, {0, 0, 1}, d.y, d.z, {1, 0, 0});
        rectFace({-h.x, 0, 0}, {0, 1, 0}, {0, 0, 1}, d.y, d.z, {-1, 0, 0});
        rectFace({0, h.y, 0}, {1, 0, 0}, {0, 0, 1}, d.x, d.z, {0, 1, 0});
        rectFace({0, -h.y, 0}, {1, 0, 0}, {0, 0, 1}, d.x, d.z, {0, -1, 0});
        rectFace({0, 0, h.z}, {1, 0, 0}, {0, 1, 0}, d.x, d.y, {0, 0, 1});
        rectFace({0, 0, -h.z}, {1, 0, 0}, {0, 1, 0}, d.x, d.y, {0, 0, -1});
        if (s == Shape::Truss)
            for (size_t i = first; i < out.size(); i++) out[i].area *= 0.22; // (the members cover about a fifth of a face)
        break;
    }
    case Shape::Wedge: {
        Vec3d h = d * 0.5;
        rectFace({0, 0, -h.z}, {1, 0, 0}, {0, 1, 0}, d.x, d.y, {0, 0, -1});
        rectFace({-h.x, 0, 0}, {0, 0, 1}, {0, 1, 0}, d.z, d.y, {-1, 0, 0});
        Vec3d n = norm(Vec3d{d.z, 0, d.x});
        Vec3d along = norm(Vec3d{-d.x, 0, d.z});
        rectFace({0, 0, 0}, along, {0, 1, 0}, std::sqrt(d.x * d.x + d.z * d.z), d.y, n);
        Vec3d A{-h.x, 0, -h.z}, B{h.x, 0, -h.z}, C{-h.x, 0, h.z};
        triFace(A + Vec3d{0, h.y, 0}, B + Vec3d{0, h.y, 0}, C + Vec3d{0, h.y, 0}, {0, 1, 0});
        triFace(A - Vec3d{0, h.y, 0}, B - Vec3d{0, h.y, 0}, C - Vec3d{0, h.y, 0}, {0, -1, 0});
        break;
    }
    case Shape::Cylinder: {
        double rB = d.x, rT = d.y, h = d.z;
        int nth = std::max(8, res * 2);
        double sl = std::sqrt((rB - rT) * (rB - rT) + h * h);
        int nz = std::max(1, (int)std::lround(res * sl / std::max(PI * (rB + rT), 1e-6) * 2));
        nz = std::min(nz, res * 4);
        for (int j = 0; j < nz; j++) {
            double f = (j + 0.5) / nz;
            double z = -h / 2 + f * h, r = rB + (rT - rB) * f;
            double dA = 2 * PI * r / nth * sl / nz;
            if (r <= 1e-9) continue;
            for (int i = 0; i < nth; i++) {
                double a = (i + 0.5) / nth * TAU;
                Vec3d n = norm(Vec3d{std::cos(a) * h, std::sin(a) * h, rB - rT});
                out.push_back({{r * std::cos(a), r * std::sin(a), z}, n, dA});
            }
        }
        for (int cap = 0; cap < 2; cap++) {
            double r = cap ? rT : rB, z = cap ? h / 2 : -h / 2;
            if (r < 1e-6) continue;
            int nr = std::max(1, res / 2);
            for (int k = 0; k < nr; k++) {
                double r0 = r * k / nr, r1 = r * (k + 1) / nr;
                int segs = std::max(4, (int)std::lround(nth * (k + 0.5) / nr));
                double dA = PI * (r1 * r1 - r0 * r0) / segs, rm = 0.5 * (r0 + r1);
                for (int i = 0; i < segs; i++) {
                    double a = (i + 0.5) / segs * TAU;
                    out.push_back({{rm * std::cos(a), rm * std::sin(a), z}, {0, 0, cap ? 1.0 : -1.0}, dA});
                }
            }
        }
        break;
    }
    case Shape::Ellipsoid: {
        int nt = std::max(4, res), np = std::max(8, res * 2);
        double dt = PI / nt, dp = TAU / np;
        for (int i = 0; i < nt; i++) {
            double th = (i + 0.5) * dt;
            for (int j = 0; j < np; j++) {
                double ph = (j + 0.5) * dp;
                Vec3d p{d.x * std::sin(th) * std::cos(ph), d.y * std::sin(th) * std::sin(ph), d.z * std::cos(th)};
                Vec3d dth{d.x * std::cos(th) * std::cos(ph), d.y * std::cos(th) * std::sin(ph), -d.z * std::sin(th)};
                Vec3d dph{-d.x * std::sin(th) * std::sin(ph), d.y * std::sin(th) * std::cos(ph), 0};
                Vec3d n = norm(Vec3d{p.x / (d.x * d.x), p.y / (d.y * d.y), p.z / (d.z * d.z)});
                out.push_back({p, n, len(cross(dth, dph)) * dt * dp});
            }
        }
        break;
    }
    default: break;
    }
}

void shapeContactPoints(Shape s, const Vec3d& d, std::vector<Vec3d>& out) {
    if (s == Shape::Loft) { shapeContactPoints(Shape::Ellipsoid, d * 0.5, out); return; }
    out.clear();
    Vec3d h = d * 0.5;
    switch (s) {
    case Shape::Box:
    case Shape::Truss:
    case Shape::Airfoil:
        for (int i = 0; i < 8; i++) out.push_back({(i & 1) ? h.x : -h.x, (i & 2) ? h.y : -h.y, (i & 4) ? h.z : -h.z});
        break;
    case Shape::Wedge:
        for (double y : {-h.y, h.y}) {
            out.push_back({-h.x, y, -h.z});
            out.push_back({h.x, y, -h.z});
            out.push_back({-h.x, y, h.z});
        }
        break;
    case Shape::Cylinder:
        for (int i = 0; i < 16; i++) {
            double a = i * TAU / 16;
            out.push_back({d.x * std::cos(a), d.x * std::sin(a), -d.z / 2});
            out.push_back({d.y * std::cos(a), d.y * std::sin(a), d.z / 2});
        }
        break;
    case Shape::Ellipsoid:
        for (int i = 0; i < 26; i++) {
            int a = i % 3 - 1, b = (i / 3) % 3 - 1, c = i / 9 - 1;
            if (a == 0 && b == 0 && c == 0) continue;
            Vec3d u = norm(Vec3d{(double)a, (double)b, (double)c});
            Vec3d p{u.x * d.x, u.y * d.y, u.z * d.z};
            double k = std::sqrt(p.x * p.x / (d.x * d.x) + p.y * p.y / (d.y * d.y) + p.z * p.z / (d.z * d.z));
            out.push_back(p / k);
        }
        break;
    default: break;
    }
}

// Solid raw moments (unit density), local frame, by slicing along z
static Moments solidMoments(Shape s, const Vec3d& d) {
    if (s == Shape::Loft) return solidMoments(Shape::Ellipsoid, d * 0.5);
    Moments M;
    double z0, z1;
    if (s == Shape::Ellipsoid) { z0 = -d.z; z1 = d.z; }
    else { z0 = -d.z / 2; z1 = d.z / 2; }
    const int N = 400;
    double dz = (z1 - z0) / N;
    for (int k = 0; k < N; k++) {
        double z = z0 + (k + 0.5) * dz;
        double A = 0, cx = 0, cy = 0, jxx = 0, jyy = 0;
        switch (s) {
        case Shape::Box:
        case Shape::Truss:
        case Shape::Airfoil: A = d.x * d.y; jxx = d.x * d.x / 12; jyy = d.y * d.y / 12; break;
        case Shape::Cylinder: {
            double r = d.x + (d.y - d.x) * (z - z0) / d.z;
            A = PI * r * r; jxx = jyy = r * r / 4;
            break;
        }
        case Shape::Ellipsoid: {
            double f = 1 - (z / d.z) * (z / d.z);
            if (f <= 0) continue;
            double a = d.x * std::sqrt(f), b = d.y * std::sqrt(f);
            A = PI * a * b; jxx = a * a / 4; jyy = b * b / 4;
            break;
        }
        case Shape::Wedge: {
            double t = (z - z0) / d.z;
            double w = d.x * (1 - t);
            A = w * d.y; cx = -d.x / 2 + w / 2; jxx = w * w / 12; jyy = d.y * d.y / 12;
            break;
        }
        default: break;
        }
        double m = A * dz;
        M.m += m;
        M.s += Vec3d{cx, cy, z} * m;
        M.q[0][0] += m * (cx * cx + jxx);
        M.q[1][1] += m * (cy * cy + jyy);
        M.q[2][2] += m * z * z;
        M.q[0][1] += m * cx * cy;
        M.q[0][2] += m * cx * z;
        M.q[1][2] += m * cy * z;
    }
    M.q[1][0] = M.q[0][1];
    M.q[2][0] = M.q[0][2];
    M.q[2][1] = M.q[1][2];
    return M;
}

void Moments::add(const Moments& o) {
    m += o.m;
    s += o.s;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) q[i][j] += o.q[i][j];
}
void Moments::addPoint(double mass, const Vec3d& r) {
    m += mass;
    s += r * mass;
    double v[3] = {r.x, r.y, r.z};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) q[i][j] += mass * v[i] * v[j];
}

static Moments scaleMoments(const Moments& M, double k) {
    Moments r = M;
    r.m *= k;
    r.s *= k;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.q[i][j] *= k;
    return r;
}

// rotate by R and translate by p
static Moments transformMoments(const Moments& M, const Mat3d& R, const Vec3d& p) {
    Moments o;
    o.m = M.m;
    Vec3d rs = R * M.s;
    o.s = rs + p * M.m;
    Mat3d Q;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) Q.m[i][j] = M.q[i][j];
    Mat3d RQ = R * Q * R.T();
    double pv[3] = {p.x, p.y, p.z}, sv[3] = {rs.x, rs.y, rs.z};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) o.q[i][j] = RQ.m[i][j] + sv[i] * pv[j] + pv[i] * sv[j] + M.m * pv[i] * pv[j];
    return o;
}

MassProps toMassProps(const Moments& M) {
    MassProps mp;
    mp.mass = M.m;
    if (M.m <= 0) return mp;
    mp.com = M.s / M.m;
    double c[3] = {mp.com.x, mp.com.y, mp.com.z};
    double J[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) J[i][j] = M.q[i][j] - M.m * c[i] * c[j]; // central second moment
    double tr = J[0][0] + J[1][1] + J[2][2];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) mp.inertia.m[i][j] = (i == j ? tr : 0) - J[i][j];
    // principal moments via Jacobi iteration
    double A[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) A[i][j] = mp.inertia.m[i][j];
    for (int sweep = 0; sweep < 30; sweep++) {
        double off = std::fabs(A[0][1]) + std::fabs(A[0][2]) + std::fabs(A[1][2]);
        if (off < 1e-12 * (std::fabs(A[0][0]) + std::fabs(A[1][1]) + std::fabs(A[2][2]) + 1e-30)) break;
        for (int p = 0; p < 2; p++)
            for (int q = p + 1; q < 3; q++) {
                if (std::fabs(A[p][q]) < 1e-300) continue;
                double th = 0.5 * std::atan2(2 * A[p][q], A[q][q] - A[p][p]);
                double cs = std::cos(th), sn = std::sin(th);
                for (int k = 0; k < 3; k++) {
                    double akp = A[k][p], akq = A[k][q];
                    A[k][p] = cs * akp - sn * akq;
                    A[k][q] = sn * akp + cs * akq;
                }
                for (int k = 0; k < 3; k++) {
                    double apk = A[p][k], aqk = A[q][k];
                    A[p][k] = cs * apk - sn * aqk;
                    A[q][k] = sn * apk + cs * aqk;
                }
            }
    }
    double ev[3] = {A[0][0], A[1][1], A[2][2]};
    std::sort(ev, ev + 3);
    mp.principal = {ev[0], ev[1], ev[2]};
    return mp;
}

Vec3d eulerDegOf(const Mat3d& R) {
    // (b from atan2: exact near +-90 degrees as well; the general case down to a vanishing cos b)
    double cb = std::hypot(R.m[0][0], R.m[1][0]);
    double b = std::atan2(-R.m[2][0], cb);
    double aa, cc;
    if (cb > 1e-12) {
        aa = std::atan2(R.m[2][1], R.m[2][2]);
        cc = std::atan2(R.m[1][0], R.m[0][0]);
    } else {
        aa = std::atan2(-R.m[1][2], R.m[1][1]);
        cc = 0;
    }
    return {aa / DEG, b / DEG, cc / DEG};
}

std::vector<Component> expandCraft(const Craft& c) {
    std::vector<Component> out;
    for (const auto& comp : c.comps) {
        int n = std::clamp(comp.symmetry, 1, 12);
        for (int k = 0; k < n; k++) {
            Component e = comp;
            e.symmetry = 1;
            if (n > 1) {
                double a = TAU * k / n;
                Mat3d Rz = rotZ(a);
                e.pos = Rz * comp.pos;
                // fold the symmetry rotation into the Euler angles via a matrix decomposition
                Mat3d R = Rz * comp.rot().toMat();
                // XYZ extrinsic decomposition: R = Rz(c) Ry(b) Rx(a)
                double b = std::asin(clampd(-R.m[2][0], -1, 1));
                double aa, cc;
                if (std::fabs(std::cos(b)) > 1e-6) {
                    aa = std::atan2(R.m[2][1], R.m[2][2]);
                    cc = std::atan2(R.m[1][0], R.m[0][0]);
                } else {
                    aa = std::atan2(-R.m[1][2], R.m[1][1]);
                    cc = 0;
                }
                e.rotDeg = {aa / DEG, b / DEG, cc / DEG};
            }
            out.push_back(e);
        }
    }
    return out;
}

int airfoilRoot(const Component& c) {
    if (c.rootSide != 0) return c.rootSide > 0 ? 1 : -1;
    // the end toward the craft axis: local +x pointing outward -> root at -x. A span square to the radial
    // direction (a fin standing on a boom, a panel across the axis) has no end toward the axis: local -x then
    // (rounding error must not pick an end: the two fins of a pair would get different ones)
    Vec3d ex = c.rot().rotate(Vec3d{1, 0, 0}), r{c.pos.x, c.pos.y, 0};
    double d = dot(ex, r);
    if (std::fabs(d) <= 1e-9 * (1 + len(r))) return -1;
    return d > 0 ? -1 : 1;
}
// is the point pc (craft frame) inside the part as drawn, grown by tol (negative: shrunk)?
bool pointInShape(const Component& k, const Vec3d& pc, double tol) {
    Vec3d p = k.rot().toMat().T() * (pc - k.pos);
    const Vec3d& d = k.dims;
    switch (k.shape) {
    case Shape::Airfoil: {
        // the drawn planform (render airfoilMesh): chord and leading edge along the span, the section's thickness
        if (std::fabs(p.x) > d.x / 2 + tol) return false;
        double taper = std::clamp(k.taper, 0.1, 1.0), aspect = d.x / std::max(d.y, 1e-4);
        double cr = 2 / (1 + taper), ct = taper * cr, tl = std::tan(std::clamp(k.sweep, -80.0, 80.0) * DEG) * aspect;
        double num = 0, den = 0;
        for (int i = 0; i < 32; i++) {
            double u = (i + 0.5) / 32, c = cr + (ct - cr) * u;
            num += c * (u * tl + c / 2);
            den += c;
        }
        double ys = -num / den;
        double u = std::clamp(airfoilRoot(k) > 0 ? 0.5 - p.x / d.x : p.x / d.x + 0.5, 0.0, 1.0);
        double c = cr + (ct - cr) * u, le = u * tl + ys;
        double y = p.y / d.y;
        if (y < le - tol / d.y || y > le + c + tol / d.y) return false;
        return std::fabs(p.z) <= 0.5 * c * d.z + tol;
    }
    case Shape::Box:
    case Shape::Truss:
    case Shape::Wedge: return std::fabs(p.x) <= d.x / 2 + tol && std::fabs(p.y) <= d.y / 2 + tol && std::fabs(p.z) <= d.z / 2 + tol;
    case Shape::Ellipsoid: {
        double a = d.x + tol, b = d.y + tol, c = d.z + tol;
        return p.x * p.x / (a * a) + p.y * p.y / (b * b) + p.z * p.z / (c * c) <= 1;
    }
    case Shape::Cylinder: {
        if (std::fabs(p.z) > d.z / 2 + tol) return false;
        double t = std::clamp((p.z + d.z / 2) / std::max(d.z, 1e-9), 0.0, 1.0);
        double r = d.x + (d.y - d.x) * t + tol;
        return p.x * p.x + p.y * p.y <= r * r;
    }
    case Shape::Loft: {
        // the drawn body (render loftMesh): superellipse sections, the nose and the tail cone closing, the tail
        // centre line raised by the upsweep
        if (std::fabs(p.z) > d.z / 2 + tol) return false;
        double nose = std::clamp(k.noseLen, 0.02, 0.6), tail = std::clamp(k.tailLen, 0.02, 0.7);
        if (nose + tail > 0.98) { double q = 0.98 / (nose + tail); nose *= q; tail *= q; }
        double s = std::clamp(p.z / d.z + 0.5, 0.0, 1.0), yc = 0, f = 1;
        if (s > 1 - nose) {
            double t = (1 - s) / nose, e = 1.0 - 0.5 * std::clamp(k.bluff, 0.0, 1.0);
            f = std::pow(std::max(0.0, 1 - (1 - t) * (1 - t)), e);
        } else if (s < tail) {
            double t = s / tail, sm = t * t * (3 - 2 * t);
            yc = k.upsweep * 0.5 * (1 - sm) * (1 - std::clamp(k.tailR, 0.0, 1.0) * 0.5);
            f = k.tailR + (1 - k.tailR) * std::pow(sm, 0.75);
        }
        double n = 2.0 + 2.5 * std::clamp(k.boxy, 0.0, 1.0);
        double ax = 0.5 * f * d.x + tol, ay = 0.5 * f * d.y * (1 - std::fabs(yc)) + tol;
        double y = p.y - yc * 0.5 * d.y;
        return std::pow(std::fabs(p.x) / ax, n) + std::pow(std::fabs(y) / ay, n) <= 1;
    }
    default: return false;
    }
}
// the root edge of a panel (Shape::Airfoil): points along its root chord on the camber line, craft frame
void airfoilRootEdge(const Component& k, std::vector<Vec3d>& out, int n) {
    const Vec3d& d = k.dims;
    double taper = std::clamp(k.taper, 0.1, 1.0), aspect = d.x / std::max(d.y, 1e-4);
    double cr = 2 / (1 + taper), ct = taper * cr, tl = std::tan(std::clamp(k.sweep, -80.0, 80.0) * DEG) * aspect;
    double num = 0, den = 0;
    for (int i = 0; i < 32; i++) {
        double u = (i + 0.5) / 32, c = cr + (ct - cr) * u;
        num += c * (u * tl + c / 2);
        den += c;
    }
    double ys = -num / den;
    double xr = (airfoilRoot(k) > 0 ? 0.5 : -0.5) * d.x;
    Mat3d R = k.rot().toMat();
    for (int i = 0; i < n; i++) {
        double xc = 0.03 + 0.94 * i / std::max(1, n - 1);
        out.push_back(k.pos + R * Vec3d{xr, (ys + xc * cr) * d.y, 0});
    }
}
int gearFold(const Component& c) {
    if (c.foldDir >= 1 && c.foldDir <= 3) return c.foldDir;
    return std::fabs(c.pos.x) > 0.3 ? (c.pos.x > 0 ? 1 : 2) : 3;
}
void freezePlacement(Component& c) {
    if (c.shape == Shape::Airfoil) c.rootSide = airfoilRoot(c);
    if (c.func == Func::Wheel) c.foldDir = gearFold(c);
}
Component mirroredX(const Component& c) {
    Component d = c;
    d.pos.x = -c.pos.x;
    // the mirror image is S*R*F applied to the part reflected in a local axis its shape is symmetric about (S: the
    // craft's x -> -x): S*R*F is a proper rotation. Boxes, cylinders, ellipsoids, wedges (right angle at local -x / -z)
    // and trusses are symmetric in local y; a wing panel is swept along y, its section symmetric in z; a fuselage
    // sweeps its tail up along y, symmetric in x.
    Mat3d S, F;
    S.m[0][0] = -1;
    // (a window: its outline is symmetric across, in local x)
    int ax = c.shape == Shape::Airfoil ? 2 : (c.shape == Shape::Loft || c.func == Func::Window) ? 0 : 1;
    F.m[ax][ax] = -1;
    d.rotDeg = eulerDegOf(S * c.rot().toMat() * F);
    // the root end stays at the same local end (the reflection keeps local x for a wing); a gear folding sideways
    // folds to the other side
    if (c.foldDir == 1) d.foldDir = 2;
    else if (c.foldDir == 2) d.foldDir = 1;
    return d;
}

std::string cleanLineText(const std::string& s, size_t maxBytes) {
    std::string o;
    size_t i = 0;
    while (i < s.size()) {
        unsigned char u = (unsigned char)s[i];
        size_t n = u < 0x80 ? 1 : (u & 0xE0) == 0xC0 ? 2 : (u & 0xF0) == 0xE0 ? 3 : (u & 0xF8) == 0xF0 ? 4 : 0;
        bool ok = n > 0 && i + n <= s.size();
        uint32_t cp = 0;
        if (ok && n == 1) {
            cp = u;
            ok = u >= 32 && u < 127;
        } else if (ok) {
            cp = u & (0x7F >> n);
            for (size_t j = 1; j < n && ok; j++) {
                unsigned char t = (unsigned char)s[i + j];
                ok = (t & 0xC0) == 0x80;
                cp = cp << 6 | (t & 0x3F);
            }
            static const uint32_t minCp[5] = {0, 0, 0x80, 0x800, 0x10000};
            // no overlong forms, surrogates, code points past U+10FFFF, C1 controls or line / paragraph separators
            ok = ok && cp >= minCp[n] && cp <= 0x10FFFF && !(cp >= 0xD800 && cp <= 0xDFFF) && cp >= 0xA0 && cp != 0x2028 && cp != 0x2029;
        }
        if (!ok) {
            i++;
            continue;
        }
        if (o.size() + n > maxBytes) break;
        o.append(s, i, n);
        i += n;
    }
    return o.empty() ? "Unnamed" : o;
}

// ===================================================================== component physics
static double defaultHeatLimit(Func f, int mat) {
    switch (f) {
    case Func::Structure:
    case Func::Tank:
    case Func::Wing:
    case Func::HeatShield: return MATERIALS[std::clamp(mat, 0, MATERIAL_COUNT - 1)].heatLimit;
    // rocket engines: nozzles run hot by design (radiatively cooled niobium / carbon-carbon extensions,
    // ~1,300 C) - an orbiter's main engines ride through the entry in its base
    case Func::Engine:
    case Func::RCS: return 1.0e6; // (thrusters as well: hot-running hardware, set in TPS-covered modules)
    default: return 4.0e5;
    }
}

void computeCompPhys(const Component& c, CompPhys& o) {
    o = CompPhys{};
    Mat3d R = c.rot().toMat();
    const Substance& mat = MATERIALS[std::clamp(c.material, 0, MATERIAL_COUNT - 1)];
    double V = std::max(shapeVolume(c.shape, c.dims), 1e-9);
    Moments solid = solidMoments(c.shape, c.dims); // unit density, local
    Moments solidUnit = scaleMoments(solid, 1.0 / std::max(solid.m, 1e-12)); // 1 kg uniformly distributed
    auto shellMoments = [&](double thickness, double density) {
        std::vector<SurfSample> ss;
        shapeSurface(c.shape, c.dims, 24, ss);
        Moments M;
        for (auto& s : ss) M.addPoint(s.area * thickness * density, s.p);
        return M;
    };
    Moments dry;
    const PartDef* def = (c.def >= 0 && c.def < (int)CATALOG.defs.size()) ? &CATALOG.defs[c.def] : nullptr;
    switch (c.func) {
    case Func::Structure:
    case Func::Wing:
    case Func::HeatShield:
        if (c.shape == Shape::Truss) {
            // the members: tubes of the truss radius with the component's wall, as point masses along them
            std::vector<std::pair<Vec3d, Vec3d>> mem;
            trussMembers(c.dims, mem);
            double r = trussTubeRadius(c.dims), w = std::clamp(c.wall, 0.0005, r);
            double area = PI * (r * r - (r - w) * (r - w));
            for (auto& e : mem) {
                double L = len(e.second - e.first), m = mat.density * area * L / 4;
                for (int k = 0; k < 4; k++) dry.addPoint(m, e.first + (e.second - e.first) * ((k + 0.5) / 4));
            }
        } else if (c.hollow) dry = shellMoments(c.wall, mat.density);
        else dry = scaleMoments(solid, mat.density);
        break;
    case Func::Tank: {
        dry = shellMoments(c.wall, mat.density);
        double inner = std::max(0.0, V - shapeArea(c.shape, c.dims) * c.wall);
        o.propCap = inner * propDensity(c.prop);
        o.unitPropM = solidUnit;
        break;
    }
    default: {
        double m = 50;
        if (def) {
            m = def->dryMass * (c.func == Func::Engine && def->kind == PK::Engine ? std::max(1, c.cluster) : 1);
            if (c.func == Func::Engine && def->kind == PK::SRB) {
                o.propCap = def->srbProp;
                o.unitPropM = solidUnit;
            }
        }
        dry = scaleMoments(solidUnit, m);
        break;
    }
    }
    if (c.massOverride > 0 && dry.m > 0) dry = scaleMoments(dry, c.massOverride / dry.m);
    else if (c.massOverride > 0) dry = scaleMoments(solidUnit, c.massOverride);
    o.dryM = transformMoments(dry, R, c.pos);
    o.unitPropM = transformMoments(o.unitPropM, R, c.pos);
    o.dryMass = o.dryM.m;
    o.centre = c.pos + R * shapeCentroid(c.shape, c.dims);
    o.axisZ = R * Vec3d{0, 0, 1};
    o.radius = len(shapeHalfExtent(c.shape, c.dims));
    o.heatLimit = defaultHeatLimit(c.func, c.material);
    // a window is flush with the hull: it is heated as the hull around it is, not as a small body of its own (the
    // orbiter's panes take the entry of the forward fuselage)
    if (c.func == Func::Window) o.heatLimit = 1e9;
    if (def) {
        o.batteryJ = def->batteryJ;
        o.genW = def->genW;
        o.drawW = def->drawW;
        o.solarArea = def->solarArea * def->solarEff;
        o.torque = def->torque;
        o.sailArea = def->sailArea;
        o.chuteCdA = def->chuteCdA;
        o.crew = def->crew;
        if (c.func == Func::Rotor || c.func == Func::Jet) {
            o.thrustVac = def->thrustVac; // jets: static sea-level thrust; rotors: from the shaft power
            o.ispVac = def->ispVac;
            o.ispSL = def->ispSL;
            o.powerUse = c.func == Func::Rotor ? def->powerUse : 0;
            o.thrustDir = o.axisZ;
            o.thrustPoint = c.pos;
        }
        if (c.func == Func::Engine) {
            int n = def->kind == PK::Engine ? std::max(1, c.cluster) : 1;
            o.thrustVac = def->thrustVac * n;
            o.ispVac = def->ispVac;
            o.ispSL = def->ispSL;
            o.gimbalDeg = def->gimbalDeg;
            o.minThrottle = def->minThrottle;
            o.powerUse = def->powerUse * n;
            o.thrustDir = o.axisZ;
            o.thrustPoint = c.pos + R * Vec3d{0, 0, -c.dims.z / 2};
        }
    }
    // only parts that really are batteries / panels keep those values
    if (c.func != Func::Command && c.func != Func::Battery) o.batteryJ = 0;
    if (c.func != Func::Solar) o.solarArea = 0;
    if (c.func != Func::RTG && c.func != Func::Reactor) o.genW = 0;
    if (c.func != Func::Command && c.func != Func::CMG) { o.torque = 0; o.drawW = c.func == Func::Command ? o.drawW : 0; }
    if (c.func == Func::Command && def) o.drawW = def->drawW;
    if (c.func == Func::CMG && def) o.drawW = def->drawW;
    if (c.func != Func::Sail) o.sailArea = 0;
    if (c.func != Func::Parachute) o.chuteCdA = 0;
    if (c.func != Func::Command) o.crew = 0;
    if (c.func == Func::RCS) {
        o.rcsF = def && def->kind == PK::RCS ? def->thrustVac : 22.0;
        o.rcsIsp = def && def->kind == PK::RCS ? def->ispVac : 229.0;
    }
}

Component foldedPanel(const Component& c, int hingeEnd) {
    if (c.func != Func::Solar || c.shape != Shape::Box) return c;
    Component f = c;
    bool alongX = c.dims.x >= c.dims.z; // fold along the longer in-plane axis
    double L = alongX ? c.dims.x : c.dims.z;
    double Lf = std::max(0.08, L * 0.12);
    Mat3d R = c.rot().toMat();
    Vec3d e = R * (alongX ? Vec3d{1, 0, 0} : Vec3d{0, 0, 1});
    Vec3d a = c.pos + e * (L / 2), b = c.pos - e * (L / 2);
    auto radial = [](const Vec3d& p) { return std::sqrt(p.x * p.x + p.y * p.y); };
    // the hinge: the end given, or the end nearer the craft axis (panels along the axis fold towards their lower end)
    Vec3d hinge = hingeEnd > 0 ? a : hingeEnd < 0 ? b
                : std::fabs(radial(a) - radial(b)) > 1e-3 ? (radial(a) < radial(b) ? a : b) : (a.z < b.z ? a : b);
    f.pos = hinge + norm(c.pos - hinge) * (Lf / 2);
    if (alongX) f.dims.x = Lf;
    else f.dims.z = Lf;
    f.dims.y = std::min(c.dims.y * 4, 0.25); // stacked segments
    return f;
}

void outlinePoints(const Component& k, std::vector<Vec3d>& out, int ring) {
    Mat3d R = k.rot().toMat();
    if (k.shape == Shape::Cylinder) {
        int levels = std::clamp((int)std::ceil(k.dims.z / 0.25) + 1, 2, 200);
        for (int l = 0; l < levels; l++) {
            double f = (double)l / (levels - 1);
            double z = -k.dims.z / 2 + f * k.dims.z, r = k.dims.x + (k.dims.y - k.dims.x) * f;
            for (int a = 0; a < ring; a++) out.push_back(k.pos + R * Vec3d{r * std::cos(a * TAU / ring), r * std::sin(a * TAU / ring), z});
        }
    } else if (k.shape == Shape::Ellipsoid || k.shape == Shape::Loft) {
        for (int j = 0; j <= 6; j++)
            for (int a = 0; a < ring; a++) {
                double th = PI * j / 6, ph = TAU * a / ring;
                out.push_back(k.pos + R * Vec3d{k.dims.x * std::sin(th) * std::cos(ph), k.dims.y * std::sin(th) * std::sin(ph), k.dims.z * std::cos(th)});
            }
    } else {
        Vec3d he = shapeHalfExtent(k.shape, k.dims);
        for (int sx = -1; sx <= 1; sx++)
            for (int sy = -1; sy <= 1; sy++)
                for (int sz = -1; sz <= 1; sz++)
                    if (sx || sy || sz) out.push_back(k.pos + R * Vec3d{sx * he.x, sy * he.y, sz * he.z});
    }
}

Component stowedLeg(const Component& c) {
    if (c.func != Func::Legs) return c;
    Component f = c;
    Mat3d R = c.rot().toMat();
    Vec3d top = c.pos + R * Vec3d{0, 0, c.dims.z / 2}; // the hinge on the mount
    Vec3d u{top.x, top.y, 0};
    double rr = len(u);
    u = rr > 1e-6 ? u / rr : Vec3d{1, 0, 0};
    double Ls = c.dims.z * 0.8, r = std::max(c.dims.x, 0.03);
    f.pos = top + u * (r + 0.04) - Vec3d{0, 0, Ls / 2};
    f.dims.z = Ls;
    f.rotDeg = {0, 0, std::atan2(u.y, u.x) / DEG}; // vertical, along the side
    return f;
}

bool retractableWheel(const Component& c) {
    return c.func == Func::Wheel && !(c.def >= 0 && c.def < (int)CATALOG.defs.size() && CATALOG.defs[c.def].powerUse > 0);
}
Component stowedWheel(const Component& c) {
    if (!retractableWheel(c)) return c;
    Component f = c;
    Quatd q = c.rot();
    Vec3d hinge = c.pos + q.rotate(Vec3d{0, 0, c.dims.z / 2});
    Vec3d dD = q.rotate(Vec3d{0, 0, -1}); // hinge -> wheel, deployed (down)
    // main gear (off the centre line) folds inward, the nose gear aft (-Z): into the wing roots and the
    // fuselage, the wheel lifted into its bay by its own radius (gearFold)
    int fd = gearFold(c);
    Vec3d dS = fd == 1 ? Vec3d{-1, 0, 0} : fd == 2 ? Vec3d{1, 0, 0} : Vec3d{0, 0, -1};
    Vec3d ax = cross(dD, dS);
    double s = len(ax), cth = clampd(dot(dD, dS), -1, 1);
    Quatd turn = s > 1e-9 ? Quatd::axisAngle(ax / s, std::atan2(s, cth)) : Quatd{};
    Quatd qs = (turn * q).normalized();
    double tyre = c.def >= 0 && c.def < (int)CATALOG.defs.size() ? CATALOG.defs[c.def].diam * 0.5 : 0.35;
    f.pos = hinge - qs.rotate(Vec3d{0, 0, c.dims.z / 2}) - dD * (tyre + 0.05);
    // back to Euler angles (as the CAD stores them): through the matrix
    Mat3d R = qs.toMat();
    double ry = std::asin(clampd(-R.m[2][0], -1, 1));
    double rx = std::atan2(R.m[2][1], R.m[2][2]), rz = std::atan2(R.m[1][0], R.m[0][0]);
    f.rotDeg = {rx / DEG, ry / DEG, rz / DEG};
    return f;
}

Component wheelAt(const Component& c, double t) {
    if (!retractableWheel(c) || t >= 0.999) return c;
    Component s = stowedWheel(c);
    if (t <= 0.001) return s;
    Quatd qa = s.rot(), qb = c.rot();
    if (qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z < 0) qb = Quatd{-qb.w, -qb.x, -qb.y, -qb.z};
    double k = t * t * (3 - 2 * t);
    Quatd q = Quatd{qa.w + (qb.w - qa.w) * k, qa.x + (qb.x - qa.x) * k, qa.y + (qb.y - qa.y) * k, qa.z + (qb.z - qa.z) * k}.normalized();
    Vec3d hinge = c.pos + c.rot().rotate(Vec3d{0, 0, c.dims.z / 2});
    Component f = c;
    f.pos = hinge - q.rotate(Vec3d{0, 0, c.dims.z / 2});
    // (the lift into the bay grows over the last part of the travel)
    Vec3d lift = s.pos - (hinge - s.rot().rotate(Vec3d{0, 0, c.dims.z / 2}));
    f.pos = f.pos + lift * std::clamp((1 - t) * 2 - 1, 0.0, 1.0);
    Mat3d R = q.toMat();
    f.rotDeg = {std::atan2(R.m[2][1], R.m[2][2]) / DEG, std::asin(clampd(-R.m[2][0], -1, 1)) / DEG, std::atan2(R.m[1][0], R.m[0][0]) / DEG};
    return f;
}

bool solarRadialMount(const Component& c) {
    if (c.func != Func::Solar || c.shape != Shape::Box) return false;
    Vec3d e = c.rot().rotate(Vec3d{1, 0, 0});
    Vec3d hinge = c.pos - e * (c.dims.x / 2);
    double rh = std::hypot(hinge.x, hinge.y);
    Vec3d u{e.x, e.y, 0};
    return c.dims.x >= c.dims.z && std::fabs(e.z) < 0.3 && len(u) > 0.5 && rh > 0.1 && dot(norm(u), Vec3d{hinge.x, hinge.y, 0} / rh) > 0.9;
}

std::vector<Component> stowedGeometry(const std::vector<Component>& comps) {
    std::vector<Component> out = comps;
    std::vector<Vec3d> pts;
    for (size_t i = 0; i < comps.size(); i++) {
        const Component& c = comps[i];
        if (c.func != Func::Solar || c.shape != Shape::Box) continue;
        if (!solarRadialMount(c)) {
            // folded towards the end that is mounted on the rest of the craft, when only one of them is
            // (an array on a truss either side of its blanket box; a wing on a payload off the craft axis)
            bool alongX = c.dims.x >= c.dims.z;
            double L = alongX ? c.dims.x : c.dims.z;
            Vec3d e = c.rot().rotate(alongX ? Vec3d{1, 0, 0} : Vec3d{0, 0, 1});
            auto mounted = [&](const Vec3d& p) {
                for (size_t j = 0; j < comps.size(); j++)
                    if (j != i && comps[j].func != Func::Solar && pointInShape(comps[j], p, 0.15)) return true;
                return false;
            };
            bool ma = mounted(c.pos + e * (L / 2)), mb = mounted(c.pos - e * (L / 2));
            // both ends on the craft: cells laid on its skin (a capsule's trunk, a probe's body) - fixed, nothing
            // to unfold (before, they slid along the hull as if opening)
            if (ma && mb && mounted(c.pos)) continue;
            out[i] = foldedPanel(c, ma == mb ? 0 : ma ? 1 : -1);
            continue;
        }
        Mat3d R = c.rot().toMat();
        Vec3d e = R * Vec3d{1, 0, 0};
        Vec3d hinge = c.pos - e * (c.dims.x / 2);
        double rh = std::hypot(hinge.x, hinge.y);
        Vec3d u = norm(Vec3d{e.x, e.y, 0});
        double L = c.dims.x, W = c.dims.z;
        double Ws = std::min(W, 2.4), zc = c.pos.z;
        std::vector<Vec3d> others; // everything near the stack's height band
        for (size_t j = 0; j < comps.size(); j++) {
            if (j == i || comps[j].func == Func::Fairing || comps[j].func == Func::Solar) continue;
            pts.clear();
            outlinePoints(comps[j].func == Func::Legs ? stowedLeg(comps[j]) : comps[j], pts);
            for (const Vec3d& p : pts)
                if (p.z >= zc - Ws / 2 - 0.02 && p.z <= zc + Ws / 2 + 0.02) others.push_back(p);
        }
        double bestR = 1e30, bestD = 0, bestT = 0, bestLs = 0;
        for (double Lc : {2.0, 1.6, 1.2, 0.9}) {
            double Ls = std::min(L, Lc), half = Ls / 2;
            int n = (int)std::ceil(L / Ls - 1e-9) * (int)std::ceil(W / Ws - 1e-9);
            double thick = n * 0.025 + 0.02; // ~25 mm per folded panel
            // radial position of the stack: just outside everything in front of its face
            double d = std::max(0.05, rh - 0.17);
            for (const Vec3d& p : others) {
                double along = p.x * u.x + p.y * u.y, side = -p.x * u.y + p.y * u.x;
                if (along <= 0 || std::fabs(side) > half + 0.05) continue;
                d = std::max(d, along + 0.03);
            }
            double outer = std::hypot(d + thick, half);
            if (outer < bestR - 1e-6) { bestR = outer; bestD = d; bestT = thick; bestLs = Ls; }
            if (Ls >= L) break;
        }
        Component f = c;
        f.dims = {bestT, bestLs, Ws};                       // local x radial (stack), y tangential, z axial
        f.pos = u * (bestD + bestT / 2) + Vec3d{0, 0, zc};
        out[i] = f;
    }
    return out;
}

void rcsNozzles(const Component& c, Vec3d pos[4], Vec3d forceDir[4]) {
    Mat3d R = c.rot().toMat();
    Vec3d h = shapeHalfExtent(c.shape, c.dims);
    const Vec3d e[4] = {{0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    const double ext[4] = {h.y, h.y, h.z, h.z};
    for (int k = 0; k < 4; k++) {
        pos[k] = c.pos + R * (e[k] * ext[k]);
        forceDir[k] = -(R * e[k]); // exhaust leaves along e: the reaction pushes the other way
    }
}

// ===================================================================== aerodynamics
namespace {
struct AeroComp {
    Shape shape;
    Vec3d dims, pos;
    Mat3d R, RT;
    Vec3d centre;
    double radius;
    bool wing;
};

bool occluded(const std::vector<AeroComp>& C, int self, const Vec3d& p, const Vec3d& dir) {
    for (int j = 0; j < (int)C.size(); j++) {
        if (j == self) continue;
        const AeroComp& c = C[j];
        // bounding-sphere rejection
        Vec3d oc = c.centre - p;
        double t = dot(oc, dir);
        double d2 = len2(oc) - t * t;
        if (d2 > c.radius * c.radius) continue;
        if (t < -c.radius) continue;
        double th;
        if (shapeRay(c.shape, c.dims, c.RT * (p - c.pos), c.RT * dir, th)) return true;
    }
    return false;
}
bool buriedIn(const std::vector<AeroComp>& C, int self, const Vec3d& p) {
    for (int j = 0; j < (int)C.size(); j++) {
        if (j == self) continue;
        const AeroComp& c = C[j];
        if (len2(p - c.centre) > c.radius * c.radius * 1.01) continue;
        if (shapeInside(c.shape, c.dims, c.RT * (p - c.pos))) return true;
    }
    return false;
}
} // namespace

namespace {
struct AeroCacheEntry {
    uint64_t key;
    size_t ncomp, bytes;
    std::shared_ptr<AeroTable> tab;
};
std::list<AeroCacheEntry> g_aeroCache; // front = most recently used
size_t g_aeroCacheBytes = 0;
std::mutex g_aeroMutex;

} // namespace
uint64_t geometryKey(const std::vector<Component>& comps) {
    uint64_t h = 1469598103934665603ull;
    auto mix = [&](const void* p, size_t n) {
        const unsigned char* b = (const unsigned char*)p;
        for (size_t i = 0; i < n; i++) h = (h ^ b[i]) * 1099511628211ull;
    };
    for (const Component& c : comps) {
        int iv[4] = {(int)c.func, (int)c.shape, c.hollow ? 1 : 0, c.symmetry};
        double dv[10] = {c.dims.x, c.dims.y, c.dims.z, c.pos.x, c.pos.y, c.pos.z, c.rotDeg.x, c.rotDeg.y, c.rotDeg.z, c.wall};
        mix(iv, sizeof iv);
        mix(dv, sizeof dv);
    }
    return h;
}

size_t aeroTableBytes(const AeroTable& t) {
    return sizeof(AeroTable) + (t.Fp.capacity() + t.Tp.capacity() + t.Ff.capacity() + t.Tf.capacity()) * sizeof(Vec3d) +
           (t.aproj.capacity() + t.exposure.capacity()) * sizeof(float);
}

size_t aeroCacheBytes() {
    std::lock_guard<std::mutex> lk(g_aeroMutex);
    return g_aeroCacheBytes;
}

void trimAeroCache(size_t keepBytes) {
    std::lock_guard<std::mutex> lk(g_aeroMutex);
    while (g_aeroCacheBytes > keepBytes && !g_aeroCache.empty()) {
        g_aeroCacheBytes -= g_aeroCache.back().bytes;
        g_aeroCache.pop_back(); // vessels still holding the table keep it alive
    }
}

std::shared_ptr<AeroTable> aeroTableCached(const std::vector<Component>& comps) {
    if (comps.empty()) return buildAeroTable(comps);
    uint64_t key = geometryKey(comps);
    {
        std::lock_guard<std::mutex> lk(g_aeroMutex);
        for (auto it = g_aeroCache.begin(); it != g_aeroCache.end(); ++it)
            if (it->key == key && it->ncomp == comps.size()) {
                g_aeroCache.splice(g_aeroCache.begin(), g_aeroCache, it);
                return it->tab;
            }
    }
    auto tab = buildAeroTable(comps);
    size_t bytes = aeroTableBytes(*tab);
    std::lock_guard<std::mutex> lk(g_aeroMutex);
    for (auto it = g_aeroCache.begin(); it != g_aeroCache.end(); ++it)
        if (it->key == key && it->ncomp == comps.size()) {
            g_aeroCache.splice(g_aeroCache.begin(), g_aeroCache, it);
            return it->tab;
        }
    g_aeroCache.push_front({key, comps.size(), bytes, tab});
    g_aeroCacheBytes += bytes;
    while (g_aeroCacheBytes > AERO_CACHE_BUDGET && g_aeroCache.size() > 1) {
        g_aeroCacheBytes -= g_aeroCache.back().bytes;
        g_aeroCache.pop_back();
    }
    return tab;
}

std::shared_ptr<AeroTable> buildAeroTable(const std::vector<Component>& comps) {
    auto tab = std::make_shared<AeroTable>();
    int n = (int)comps.size();
    tab->ncomp = n;
    tab->key = geometryKey(comps);
    tab->nt = 19;
    tab->np = 36;
    std::vector<AeroComp> C(n);
    struct S { Vec3d p, n; double a; int comp; bool active; };
    std::vector<S> samples;
    bool fairingA = false; // the first clamshell half carries the samples of the shell
    for (const Component& c : comps)
        if (c.func == Func::Fairing && std::cos(c.rotDeg.z * DEG) > 0) fairingA = true;
    Vec3d lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    for (int i = 0; i < n; i++) {
        const Component& c = comps[i];
        AeroComp& a = C[i];
        a.shape = c.shape;
        a.dims = c.dims;
        a.pos = c.pos;
        a.R = c.rot().toMat();
        a.RT = a.R.T();
        Vec3d he = shapeHalfExtent(c.shape, c.dims);
        a.radius = len(he) * 1.001 + 1e-3;
        a.centre = c.pos;
        a.wing = c.func == Func::Wing;
        double mx = std::max({he.x, he.y, he.z});
        // keep the table cost bounded for very large crafts (samples x directions x occluders)
        int resMax = n <= 40 ? 12 : n <= 120 ? 7 : 4;
        int res = std::clamp((int)std::lround(8 + std::log2(1 + mx)), std::min(8, resMax), resMax);
        std::vector<SurfSample> ss;
        if (fairingA && c.func == Func::Fairing && std::cos(c.rotDeg.z * DEG) < 0) continue; // twin half
        shapeSurface(c.shape, c.dims, res, ss);
        for (auto& s : ss) {
            // a jet engine's intake swallows the air (its ram drag is part of the engine's net thrust): its
            // front face is no wall
            if (c.func == Func::Jet && s.n.z > 0.9) continue;
            Vec3d p = a.R * s.p + c.pos;
            samples.push_back({p, a.R * s.n, s.area, i, true});
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
            hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        }
    }
    tab->length = samples.empty() ? 0.5 : std::max(0.5, len(hi - lo));
    // samples covered by another component do not see the flow
    for (auto& s : samples) {
        double eps = 1e-3 * (1 + C[s.comp].radius);
        if (buriedIn(C, s.comp, s.p + s.n * eps)) s.active = false;
    }
    int ND = tab->nt * tab->np;
    tab->Fp.assign(ND, {});
    tab->Tp.assign(ND, {});
    tab->Ff.assign(ND, {});
    tab->Tf.assign(ND, {});
    tab->aproj.assign(ND, 0);
    tab->exposure.assign((size_t)ND * std::max(n, 1), 0);
    const double CF = 0.004;
    std::atomic<int> next{0};
    auto worker = [&]() {
        for (;;) {
            int d = next++;
            if (d >= ND) break;
            int it = d / tab->np, ip = d % tab->np;
            double th = PI * it / (tab->nt - 1), ph = TAU * ip / tab->np;
            Vec3d u{std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)}; // air flow direction
            Vec3d Fp, Tp, Ff, Tf;
            double ap = 0;
            float* ex = tab->exposure.data() + (size_t)d * n;
            for (const auto& s : samples) {
                if (!s.active) continue;
                double c = dot(s.n, u);
                double eps = 1e-3 * (1 + C[s.comp].radius);
                bool wind = c < -1e-9;
                if (wind && occluded(C, s.comp, s.p + s.n * eps, -u)) continue;
                if (C[s.comp].wing) {
                    if (wind) ex[s.comp] = std::max(ex[s.comp], (float)-c);
                    continue; // wings use the thin-airfoil model at run time
                }
                if (wind) {
                    Vec3d f = s.n * (-2.0 * c * c * s.a); // Newtonian, Cp = 2 cos^2 (scaled at run time)
                    Fp += f;
                    Tp += cross(s.p, f);
                    ap += -c * s.a;
                    ex[s.comp] = std::max(ex[s.comp], (float)-c);
                }
                Vec3d tng = u - s.n * c;
                double tl = len(tng);
                if (tl > 1e-6 && c < 0.2) { // wetted surface not in the wake
                    Vec3d f = tng * (CF * s.a);
                    Ff += f;
                    Tf += cross(s.p, f);
                }
            }
            tab->Fp[d] = Fp;
            tab->Tp[d] = Tp;
            tab->Ff[d] = Ff;
            tab->Tf[d] = Tf;
            tab->aproj[d] = (float)ap;
        }
    };
    unsigned nth = std::clamp(std::thread::hardware_concurrency(), 1u, 16u);
    std::vector<std::thread> th;
    for (unsigned k = 0; k + 1 < nth; k++) th.emplace_back(worker);
    worker();
    for (auto& t : th) t.join();
    return tab;
}

double wingLiftSlope(const Component& c) {
    double span = std::max(0.05, c.dims.x), chord = std::max(0.05, c.dims.y);
    double AR = span / chord * (c.shape == Shape::Airfoil ? 2.0 : 1.0); // (a panel: its root on the body or the other half)
    return TAU * AR / (2 + std::sqrt(AR * AR + 4)); // Helmbold
}

double flapSettingDeg(int s, double maxDeg) {
    if (s <= 0 || maxDeg <= 0) return 0;
    s = std::min(s, FLAP_STAGES);
    double first = std::min(1.0, maxDeg);
    return first * std::pow(maxDeg / first, (s - 1) / double(FLAP_STAGES - 1));
}
double flapAngleAt(double pos, double maxDeg) {
    if (pos <= 0 || maxDeg <= 0) return 0;
    double first = std::min(1.0, maxDeg);
    if (pos <= 1) return first * pos;
    return first * std::pow(maxDeg / first, std::min(pos - 1, double(FLAP_STAGES - 1)) / double(FLAP_STAGES - 1));
}
double flapPosOf(double deg, double maxDeg) {
    if (deg <= 0 || maxDeg <= 0) return 0;
    double first = std::min(1.0, maxDeg);
    if (deg <= first) return deg / first;
    if (maxDeg <= first * 1.0000001) return 1;
    return std::min(double(FLAP_STAGES), 1 + (FLAP_STAGES - 1) * std::log(deg / first) / std::log(maxDeg / first));
}

Vec3d wingForce(const Component& c, const CompPhys& p, const Vec3d& vAir, double rho, double M, double dAlpha, double flapRad) {
    double vls = len(vAir);
    if (vls < 1e-3 || rho <= 0) return {};
    Vec3d uw = -vAir / vls; // air-flow direction at the wing
    Vec3d n = p.axisZ;
    double span = std::max(0.05, c.dims.x), chord = std::max(0.05, c.dims.y);
    double S = span * chord, AR = span * span / S * (c.shape == Shape::Airfoil ? 2.0 : 1.0); // (a panel: half a wing or a fin on a body)
    double ql = 0.5 * rho * vls * vls;
    double un = clampd(dot(uw, n), -1, 1);
    double alphaS = std::asin(un) + dAlpha; // (the incidence that stalls)
    // a slotted flap over FLAP_SPAN x FLAP_CHORD of the panel: as an incidence change (thin-airfoil flap
    // effectiveness ~0.55 for a 28 % chord flap, falling off past 15 degrees as the flow separates from it); the
    // wing stalls at a smaller incidence but at more lift (only 40 % of the flap's angle counts for the stall),
    // and the flap adds profile drag (Raymer: 1.7 (cf/c)^1.38 (Sflap/S) sin^2 delta)
    double dFl = 0, cdFl = 0;
    if (flapRad != 0) {
        double d = std::min(std::fabs(flapRad), 60 * DEG);
        double eff = 0.55 * (d < 15 * DEG ? 1.0 : std::max(0.45, 1 - 0.55 * (d - 15 * DEG) / (35 * DEG)));
        dFl = (flapRad > 0 ? 1.0 : -1.0) * eff * d * FLAP_SPAN;
        cdFl = 1.7 * std::pow(FLAP_CHORD, 1.38) * FLAP_SPAN * std::sin(d) * std::sin(d);
    }
    double alpha = alphaS + dFl;
    auto sub = [&](double m) {
        double b = std::sqrt(std::max(1 - m * m, 0.05));
        return TAU * AR / (2 + std::sqrt(AR * AR * b * b + 4)); // Helmbold / DATCOM
    };
    auto sup = [&](double m) {
        double B = std::sqrt(m * m - 1);
        return 4.0 / B * std::max(0.35, 1 - 1 / (2 * AR * B)); // Ackeret with tip loss
    };
    double CLa;
    if (M < 0.85) CLa = sub(M);
    else if (M > 1.2) CLa = sup(M);
    else CLa = lerpd(sub(0.85), sup(1.2), (M - 0.85) / 0.35);
    // a highly swept (delta) panel at low speed: the leading-edge vortex adds lift (Polhamus, Kv ~ pi) and the
    // stall comes later (about 30 degrees on a 60-degree delta)
    double dlt = c.shape == Shape::Airfoil ? clampd((std::fabs(c.sweep) - 35) / 25, 0, 1) * clampd((1.6 - M) / 0.6, 0, 1) : 0.0;
    double aStall = (15 + 15 * dlt) * DEG;
    double t = clampd((std::fabs(alphaS + 0.4 * dFl) - aStall) / (8 * DEG), 0, 1);
    double sa = std::sin(std::fabs(alpha));
    double CL = (CLa * alpha + (alpha >= 0 ? 1.0 : -1.0) * PI * dlt * sa * sa * std::cos(alpha)) * (1 - t);
    double CD = 0.01 + cdFl + CL * CL / (PI * 0.8 * AR);
    Vec3d F = uw * (ql * S * CD);
    Vec3d Lhat = n - uw * un;
    double ll = len(Lhat);
    if (ll > 1e-6) F += Lhat / ll * (ql * S * CL);
    double CN = 1.2 * std::sin(std::fabs(alpha)) * t; // post-stall flat plate
    F += n * (ql * S * CN * (alpha >= 0 ? 1.0 : -1.0));
    return F;
}

AeroTable::Sample AeroTable::lookup(const Vec3d& u, std::vector<float>* exposureOut) const {
    Sample s{};
    if (nt < 2 || np < 1 || !std::isfinite(u.x) || !std::isfinite(u.y) || !std::isfinite(u.z)) return s;
    double th = std::acos(clampd(u.z, -1, 1));
    double ph = wrapAngle(std::atan2(u.y, u.x));
    double ft = th / PI * (nt - 1), fp = ph / TAU * np;
    if (!std::isfinite(ft) || !std::isfinite(fp)) return s;
    int i0 = std::clamp((int)ft, 0, nt - 2);
    int j0 = ((int)std::floor(fp) % np + np) % np;
    double a = clampd(ft - i0, 0, 1), b = clampd(fp - std::floor(fp), 0, 1);
    int j1 = (j0 + 1) % np;
    int idx[4] = {i0 * np + j0, i0 * np + j1, (i0 + 1) * np + j0, (i0 + 1) * np + j1};
    double w[4] = {(1 - a) * (1 - b), (1 - a) * b, a * (1 - b), a * b};
    for (int k = 0; k < 4; k++) {
        s.F_p += Fp[idx[k]] * w[k];
        s.T_p += Tp[idx[k]] * w[k];
        s.F_f += Ff[idx[k]] * w[k];
        s.T_f += Tf[idx[k]] * w[k];
        s.aproj += aproj[idx[k]] * w[k];
    }
    if (exposureOut) {
        exposureOut->assign(ncomp, 0.f);
        for (int c = 0; c < ncomp; c++) {
            float v = 0;
            for (int k = 0; k < 4; k++) v += exposure[(size_t)idx[k] * ncomp + c] * (float)w[k];
            (*exposureOut)[c] = v;
        }
    }
    return s;
}

// ===================================================================== analysis
CraftStats analyzeCraft(const Craft& craft, const AeroTable* aero) {
    CraftStats st;
    auto comps = expandCraft(craft);
    int n = (int)comps.size();
    std::vector<CompPhys> ph(n);
    for (int i = 0; i < n; i++) computeCompPhys(comps[i], ph[i]);
    Moments dryAll, wetAll;
    int maxStage = 0;
    for (int i = 0; i < n; i++) {
        dryAll.add(ph[i].dryM);
        wetAll.add(ph[i].dryM);
        if (ph[i].propCap > 0) wetAll.add(scaleMoments(ph[i].unitPropM, ph[i].propCap * (comps[i].func == Func::Tank ? comps[i].fill : 1.0)));
        maxStage = std::max(maxStage, comps[i].stage);
        st.batteryJ += ph[i].batteryJ;
        st.genW += ph[i].genW;
        st.solarW1AU += ph[i].solarArea * 1361.0;
        st.drawW += ph[i].drawW;
        st.torque += ph[i].torque;
        st.crew += ph[i].crew;
        if (comps[i].func == Func::Command) st.hasControl = true;
    }
    st.dry = toMassProps(dryAll);
    st.wet = toMassProps(wetAll);
    // stage-by-stage burn (stage k engines burn stage k tanks; stage k parts drop afterwards)
    std::vector<double> prop(n);
    for (int i = 0; i < n; i++) prop[i] = ph[i].propCap * (comps[i].func == Func::Tank ? comps[i].fill : 1.0);
    for (int sidx = 0; sidx <= maxStage; sidx++) {
        CraftStage cs;
        cs.stage = sidx;
        Moments M;
        for (int i = 0; i < n; i++) {
            if (comps[i].stage < sidx) continue;
            M.add(ph[i].dryM);
            if (prop[i] > 0) M.add(scaleMoments(ph[i].unitPropM, prop[i]));
        }
        MassProps mp = toMassProps(M);
        cs.massStart = mp.mass;
        double tank[PROP_N] = {0};
        bool hasTank[PROP_N] = {false};
        struct E { int i; Prop p; double F, mdot, ispSL, ispVac; bool srb; };
        std::vector<E> engs;
        for (int i = 0; i < n; i++) {
            if (comps[i].stage != sidx) continue;
            if (comps[i].func == Func::Tank) { tank[(int)comps[i].prop] += prop[i]; hasTank[(int)comps[i].prop] = true; }
            if (comps[i].func == Func::Engine && ph[i].thrustVac > 0) {
                const PartDef& d = CATALOG.defs[comps[i].def];
                E e{i, d.prop, ph[i].thrustVac, ph[i].thrustVac / (d.ispVac * G0), d.ispSL, d.ispVac, d.kind == PK::SRB};
                engs.push_back(e);
                cs.hasEngines = true;
                cs.thrustVac += e.F;
                cs.thrustTorqueAtStart += cross(ph[i].thrustPoint - mp.com, ph[i].thrustDir * e.F);
                if (!e.srb && !hasTank[(int)d.prop]) {
                    // checked after the loop
                }
            }
        }
        for (auto& e : engs)
            if (!e.srb && !hasTank[(int)e.p])
                st.warnings.push_back(trf("Stage {0}: no {1} tank for its engine", std::to_string(sidx + 1), tr(propName(e.p))));
        double FSL = 0;
        for (auto& e : engs) FSL += e.F * std::max(0.0, e.ispSL) / e.ispVac;
        cs.twrSL = cs.massStart > 0 ? FSL / (cs.massStart * G0) : 0;
        double m = cs.massStart;
        std::vector<bool> on(engs.size(), true);
        for (int iter = 0; iter < 32; iter++) {
            double mdotP[PROP_N] = {0};
            double Ft = 0, Fsl = 0, md = 0;
            for (size_t k = 0; k < engs.size(); k++) {
                if (!on[k]) continue;
                auto& e = engs[k];
                if (!e.srb) {
                    if (tank[(int)e.p] <= 1e-9) { on[k] = false; continue; }
                    mdotP[(int)e.p] += e.mdot;
                } else if (prop[e.i] <= 1e-9) { on[k] = false; continue; }
                Ft += e.F;
                Fsl += e.F * std::max(0.0, e.ispSL) / e.ispVac;
                md += e.mdot;
            }
            if (md <= 0) break;
            double tph = 1e300;
            for (int p = 0; p < PROP_N; p++)
                if (mdotP[p] > 0) tph = std::min(tph, tank[p] / mdotP[p]);
            for (size_t k = 0; k < engs.size(); k++)
                if (on[k] && engs[k].srb) tph = std::min(tph, prop[engs[k].i] / engs[k].mdot);
            if (!(tph < 1e300) || tph <= 0) break;
            double m1 = m - md * tph;
            if (m1 <= 0) break;
            cs.dvVac += Ft / md * std::log(m / m1);
            cs.dvSL += Fsl / md * std::log(m / m1);
            cs.burnTime += tph;
            for (int p = 0; p < PROP_N; p++) tank[p] -= mdotP[p] * tph;
            for (size_t k = 0; k < engs.size(); k++)
                if (on[k] && engs[k].srb) prop[engs[k].i] -= engs[k].mdot * tph;
            m = m1;
        }
        cs.massEnd = m;
        // tanks of this stage are now (partly) empty (snapshot per-propellant totals before updating)
        double totP[PROP_N] = {0};
        for (int k = 0; k < n; k++)
            if (comps[k].stage == sidx && comps[k].func == Func::Tank) totP[(int)comps[k].prop] += prop[k];
        for (int i = 0; i < n; i++)
            if (comps[i].stage == sidx && comps[i].func == Func::Tank) {
                int pi = (int)comps[i].prop;
                double left = std::max(0.0, tank[pi]);
                prop[i] = totP[pi] > 0 ? prop[i] * left / totP[pi] : 0;
            }
        st.totalDvVac += cs.dvVac;
        st.stages.push_back(cs);
        double lever = std::max(1.0, len(st.wet.com));
        if (cs.hasEngines && len(cs.thrustTorqueAtStart) > 0.02 * cs.thrustVac * lever)
            st.warnings.push_back(trf("Stage {0}: thrust line misses the centre of mass", std::to_string(sidx + 1)));
    }
    if (!st.hasControl) st.warnings.push_back("No command capsule or probe core: vessel cannot be controlled");
    if (aero && aero->nt > 0) {
        // centre of pressure for a small angle of attack during ascent (flow towards -Z)
        double a = 4 * DEG, M = 0.5, kM = 0.55;
        Vec3d u1{std::sin(a), 0, -std::cos(a)}, u0{0, 0, -1};
        auto s1 = aero->lookup(u1, nullptr);
        auto s0 = aero->lookup(u0, nullptr);
        Vec3d F1 = s1.F_p * kM + s1.F_f, F0 = s0.F_p * kM + s0.F_f;
        Vec3d T1 = s1.T_p * kM + s1.T_f, T0 = s0.T_p * kM + s0.T_f;
        // lifting surfaces (unit dynamic pressure: rho = 2, |v| = 1)
        for (int i = 0; i < n; i++) {
            if (comps[i].func != Func::Wing) continue;
            Vec3d f1 = wingForce(comps[i], ph[i], -u1, 2.0, M), f0 = wingForce(comps[i], ph[i], -u0, 2.0, M);
            F1 += f1;
            F0 += f0;
            T1 += cross(ph[i].centre, f1);
            T0 += cross(ph[i].centre, f0);
        }
        double dFx = F1.x - F0.x;
        double dTy = T1.y - T0.y;
        st.frontalArea = s0.aproj;
        if (std::fabs(dFx) > 1e-9) {
            double zc = dTy / dFx;
            st.copAscent = {0, 0, zc};
            double D = 2 * std::sqrt(std::max(st.frontalArea, 0.01) / PI);
            st.staticMargin = (st.wet.com.z - zc) / D;
            if (st.staticMargin < 0) st.warnings.push_back("Aerodynamically unstable during ascent (centre of pressure ahead of centre of mass)");
        }
    }
    return st;
}

// ===================================================================== defaults / conversion
Component makeComponent(Func f, int def) {
    Component c;
    c.func = f;
    c.def = def;
    c.name = funcName(f);
    const PartDef* d = (def >= 0 && def < (int)CATALOG.defs.size()) ? &CATALOG.defs[def] : nullptr;
    if (d) c.name = d->name;
    switch (f) {
    case Func::Structure: c.shape = Shape::Box; c.dims = {1, 1, 1}; c.wall = 0.005; break;
    case Func::Tank: c.shape = Shape::Cylinder; c.dims = {1.25, 1.25, 4}; c.wall = 0.004; c.prop = Prop::Kerolox; c.color = propColor(c.prop); break;
    case Func::Wing: c.shape = Shape::Airfoil; c.dims = {4, 1.5, 0.12}; c.material = 3; c.hollow = false; c.color = MATERIALS[3].color; break;
    case Func::HeatShield: c.shape = Shape::Cylinder; c.dims = {1.95, 1.95, 0.45}; c.material = MAT_PICA; c.hollow = false; c.color = MATERIALS[MAT_PICA].color; break;
    // a window: a thin pane lying on the hull (local z: its outward normal); it is drawn into the surfaces around it
    case Func::Window: c.shape = Shape::Box; c.dims = {0.6, 0.45, 0.02}; c.material = MATERIAL_COUNT - 1; c.hollow = false; c.color = Rgba8{28, 30, 34, 255}; c.rim = 0.04; break;
    // a payload bay: the box of its inside (local +Y: the doors), opened in space to release or take in payloads
    case Func::CargoBay: c.shape = Shape::Box; c.dims = {4.6, 4.0, 18.3}; c.wall = 0.003; c.hollow = true; c.color = Rgba8{236, 236, 232, 255}; break;
    default:
        if (d) {
            PartInst pi = makeInst(def);
            PartGeom g = partGeom(pi);
            c.shape = Shape::Cylinder;
            c.dims = {std::max(g.rBot, 0.05), std::max(g.rTop, 0.0), std::max(g.height, 0.1)};
            c.color = d->color;
            if (f == Func::Engine && d->kind == PK::Engine) c.dims = {d->diam / 2, d->diam / 2, d->length + 0.35};
            if (f == Func::Solar) { c.shape = Shape::Box; double L = std::sqrt(d->solarArea * 3.0); c.dims = {L, 0.03, d->solarArea / L}; }
            if (f == Func::Battery) { c.shape = Shape::Box; double s = d->batteryJ > 1e7 ? 0.5 : 0.25; c.dims = {s, s * 1.4, s * 2}; }
            if (f == Func::RTG) c.dims = {0.2, 0.2, 1.0};
            if (f == Func::Legs) c.dims = {0.08, 0.08, 2.6};
            if (f == Func::RCS) { c.shape = Shape::Box; double s = d->thrustVac > 100 ? 0.34 : 0.2; c.dims = {s, s, s}; }
            // rotor: the mast and hub (the blades are drawn, their disc is not a solid body)
            if (f == Func::Rotor) { double hr = std::max(0.03, d->diam * 0.035); c.dims = {hr, hr, d->length}; }
            if (f == Func::Jet) c.dims = {d->diam / 2, d->diam / 2, d->length};
            if (f == Func::Wheel) c.dims = {0.07, 0.07, d->length}; // strut along local z, the wheel at its -z end
            if (f == Func::Docking) c.dims = {d->diam / 2, d->diam / 2, d->length}; // the port faces local +Z
        }
        break;
    }
    return c;
}

void fairingPieces(const Design& d, int i, const std::vector<double>& zb, int stage, std::vector<Component>& out) {
    const PartInst& p = d.stack[i];
    const PartDef& def = CATALOG.defs[p.def];
    double R = std::max(p.diam * 0.5, 0.2), L = std::max(p.length, 0.0), Ln = fairingNoseLength(p.diam);
    double zBase = zb[i], zBot = zBase - L;
    struct Seg { double z0, h, rb, rt; };
    std::vector<Seg> segs;
    // base: when the fairing is wider than the stage it sits on (hammerhead), a boat-tail below the
    // payload envelope closes down onto the top of that stage
    double rBelow = R;
    for (int j = i + 1; j < (int)d.stack.size(); j++) {
        PartGeom gj = partGeom(d.stack[j]);
        if (zb[j] < zBot - 1e-6 && zb[j] + gj.height >= zBot - 1e-3) { rBelow = gj.rTop; break; }
    }
    double bt = rBelow < R - 0.05 ? (R - rBelow) * 1.4 : 0.0;
    if (bt > 0) segs.push_back({zBot - bt, bt, std::max(rBelow, 0.05), R});
    if (L > 0.01) segs.push_back({zBot, L, R, R});
    // tangent ogive nose, finer toward the tip
    double rho = (R * R + Ln * Ln) / (2 * R);
    auto rAt = [&](double x) { return std::max(0.0, std::sqrt(std::max(0.0, rho * rho - x * x)) + R - rho); };
    const double xs[] = {0.0, 0.30, 0.52, 0.70, 0.84, 0.94, 1.0};
    for (int k = 0; k < 6; k++) {
        double x0 = xs[k] * Ln, x1 = xs[k + 1] * Ln;
        segs.push_back({zBase + x0, x1 - x0, rAt(x0), k == 5 ? R * 0.04 : rAt(x1)});
    }
    double areaTot = 0;
    for (auto& s : segs) areaTot += PI * (s.rb + s.rt) * std::sqrt(s.h * s.h + (s.rb - s.rt) * (s.rb - s.rt));
    double mass = partDryMass(p);
    for (int side = 0; side < 2; side++)
        for (auto& s : segs) {
            Component f;
            f.name = def.name;
            f.def = p.def;
            f.func = Func::Fairing;
            f.stage = stage;
            f.shape = Shape::Cylinder;
            f.dims = {s.rb, s.rt, s.h};
            f.pos = {0, 0, s.z0 + s.h / 2};
            f.rotDeg = {0, 0, side ? 180.0 : 0.0};
            f.color = def.color;
            f.hollow = true;
            f.wall = 0.004;
            double a = PI * (s.rb + s.rt) * std::sqrt(s.h * s.h + (s.rb - s.rt) * (s.rb - s.rt));
            f.massOverride = std::max(1.0, mass * a / std::max(areaTot, 1e-6) * 0.5);
            out.push_back(f);
        }
}

Craft craftFromDesign(const Design& d) {
    Craft cr;
    cr.name = d.name;
    int segN = 0;
    auto seg0 = segmentOfParts(d, segN);
    int n = (int)d.stack.size();
    // stage groups in firing order: a segment carrying side boosters gets a booster group just before it
    // (jettisoned first); the segment's own engines fire from that group on, in parallel with them
    std::vector<int> boostStage(std::max(segN, 1), -1), coreStage(std::max(segN, 1), 0);
    {
        std::vector<bool> hasB(std::max(segN, 1), false);
        for (int i = 0; i < n; i++)
            for (const auto& ra : d.stack[i].radial)
                if (CATALOG.defs[ra.def].kind == PK::SRB) hasB[seg0[i]] = true;
        int next = 0;
        for (int s = 0; s < segN; s++) {
            if (hasB[s]) boostStage[s] = next++;
            coreStage[s] = next++;
        }
    }
    std::vector<int> seg(n);
    for (int i = 0; i < n; i++) seg[i] = coreStage[seg0[i]];
    std::vector<double> zb(n);
    double z = 0;
    for (int i = n - 1; i >= 0; i--) {
        zb[i] = z;
        z += partGeom(d.stack[i]).height;
    }
    for (int i = 0; i < n; i++) {
        const PartInst& p = d.stack[i];
        const PartDef& def = CATALOG.defs[p.def];
        PartGeom g = partGeom(p);
        Component c;
        c.name = def.name;
        c.def = p.def;
        c.stage = seg[i];
        c.shape = Shape::Cylinder;
        c.dims = {std::max(g.rBot, 0.0), std::max(g.rTop, 0.0), g.height};
        c.pos = {0, 0, zb[i] + g.height / 2};
        c.color = def.color;
        c.massOverride = 0;
        if (def.kind == PK::Fairing) {
            fairingPieces(d, i, zb, seg[i], cr.comps);
            continue;
        }
        switch (def.kind) {
        case PK::Tank: {
            c.func = Func::Tank;
            c.prop = def.prop;
            double dry = partDryMass(PartInst{p.def, p.diam, p.diam2, p.length, 1, {}});
            double area = shapeArea(Shape::Cylinder, c.dims);
            c.wall = clampd(dry / (area * MATERIALS[0].density), 0.0005, 0.05);
            // match the factory's propellant capacity exactly
            double V = shapeVolume(Shape::Cylinder, c.dims);
            double inner = std::max(V - area * c.wall, 1e-9);
            c.fill = clampd(tankCapacity(p) / (inner * propDensity(c.prop)), 0, 1);
            c.massOverride = 0;
            break;
        }
        case PK::Engine:
        case PK::SRB:
            c.func = Func::Engine;
            c.cluster = p.cluster;
            c.massOverride = partDryMass(PartInst{p.def, p.diam, p.diam2, p.length, p.cluster, {}});
            break;
        case PK::Capsule:
        case PK::Probe:
            c.func = Func::Command;
            c.massOverride = partDryMass(PartInst{p.def, p.diam, p.diam2, p.length, 1, {}});
            break;
        case PK::HeatShield:
            c.func = Func::HeatShield;
            c.material = MAT_PICA;
            c.hollow = false;
            c.massOverride = partDryMass(PartInst{p.def, p.diam, p.diam2, p.length, 1, {}});
            break;
        case PK::Parachute: c.func = Func::Parachute; break;
        case PK::CMG: c.func = Func::CMG; break;
        case PK::Reactor: c.func = Func::Reactor; break;
        case PK::Sail: c.func = Func::Sail; break;
        case PK::Hyperdrive: c.func = Func::Hyperdrive; break;
        case PK::Docking: c.func = Func::Docking; break;
        default:
            c.func = Func::Structure;
            c.hollow = true;
            c.massOverride = partDryMass(PartInst{p.def, p.diam, p.diam2, p.length, 1, {}});
            break;
        }
        if (c.func != Func::Structure && c.func != Func::Tank && c.func != Func::HeatShield && c.func != Func::Engine && c.func != Func::Command)
            c.massOverride = 0;
        if (c.func == Func::Engine && boostStage[seg0[i]] >= 0) c.fireFrom = boostStage[seg0[i]];
        cr.comps.push_back(c);
        if (def.kind == PK::Decoupler) {
            Interstage is = interstageFor(d, i);
            if (is.on) {
                Component k;
                k.name = "Interstage";
                k.func = Func::Structure;
                k.shape = Shape::Cylinder;
                k.dims = {is.rBot, is.rTop, is.height};
                k.pos = {0, 0, zb[i] + g.height + is.height / 2};
                k.material = 0;
                k.hollow = true;
                k.wall = is.wall;
                k.stage = seg[i];
                k.color = Rgba8{34, 36, 40, 255}; // black thermal-protection coat
                k.massOverride = is.mass;
                cr.comps.push_back(k);
            }
        }
        // radial attachments
        double rMid = (g.rBot + g.rTop) * 0.5;
        for (size_t k = 0; k < p.radial.size(); k++) {
            const RadialAttach& ra = p.radial[k];
            const PartDef& rd = CATALOG.defs[ra.def];
            Component r;
            r.name = rd.name;
            r.def = ra.def;
            r.stage = seg[i];
            r.symmetry = ra.count;
            r.color = rd.color;
            double a0 = k * 0.4 + ra.angle * DEG;
            double zc = zb[i] + g.height * 0.5 + ra.offset;
            auto at = [&](double radius, double zz) { return rotZ(a0) * Vec3d{radius, 0, zz}; };
            switch (rd.kind) {
            case PK::Battery: {
                r.func = Func::Battery;
                r.shape = Shape::Box;
                double s = rd.batteryJ > 1e7 ? 0.5 : 0.25;
                r.dims = {s, s * 1.4, s * 2};
                r.pos = at(rMid + s * 0.5, zc);
                r.rotDeg = {0, 0, a0 / DEG};
                break;
            }
            case PK::Solar: {
                r.func = Func::Solar;
                r.shape = Shape::Box;
                double L = std::sqrt(rd.solarArea * 3.0), W = rd.solarArea / L;
                r.dims = {L, 0.03, W};
                r.pos = at(rMid + 0.2 + L * 0.5, zc);
                r.rotDeg = {0, 0, a0 / DEG};
                break;
            }
            case PK::RCS: {
                r.func = Func::RCS;
                r.shape = Shape::Box;
                double s = rd.thrustVac > 100 ? 0.34 : 0.2;
                r.dims = {s, s, s};
                r.pos = at(rMid + s * 0.5, zb[i] + g.height * 0.8 + ra.offset);
                r.rotDeg = {0, 0, a0 / DEG};
                break;
            }
            case PK::RTG:
                r.func = Func::RTG;
                r.dims = {0.18, 0.18, 1.0};
                r.pos = at(rMid + 0.45, zc);
                break;
            case PK::Legs: {
                r.func = Func::Legs;
                // the feet reach 0.5 m below the nozzle exits of this stage, so the legs and not the
                // engine bell meet the ground (they used to end 1.6 m below the part: above a long bell)
                double footZ = std::min(zb[i] - 1.6, stageBottomZ(d, i, zb) - 0.5);
                Vec3d top{rMid, 0, zb[i] + g.height * 0.6 + ra.offset}, foot{rMid + 1.3, 0, footZ + ra.offset};
                Vec3d dd = top - foot;
                double L = len(dd);
                r.dims = {0.08, 0.08, L};
                Vec3d mid = (top + foot) * 0.5;
                r.pos = rotZ(a0) * mid;
                double beta = std::atan2(dd.x, dd.z);
                r.rotDeg = {0, beta / DEG, a0 / DEG};
                break;
            }
            case PK::SRB: {
                // side booster: its own (earlier) stage group, with two attach struts to the core
                double rb = rd.diam * 0.5, Lb = rd.length, dist = sideBoosterDistance(rMid, rd);
                double zBot = stageBottomZ(d, i, zb) + ra.offset; // nozzle exits level with the main engines
                r.func = Func::Engine;
                r.shape = Shape::Cylinder;
                r.dims = {rb, rb, Lb};
                r.pos = at(dist, zBot + Lb / 2);
                r.rotDeg = {0, 0, a0 / DEG};
                r.stage = boostStage[seg0[i]] >= 0 ? boostStage[seg0[i]] : seg[i];
                r.massOverride = rd.dryMass;
                cr.comps.push_back(r);
                for (double fz : {0.12, 0.85}) {
                    Component s = r;
                    s.name = "Booster attach strut";
                    s.def = -1;
                    s.func = Func::Structure;
                    s.shape = Shape::Box;
                    double gap = dist - rb - rMid + 0.1;
                    s.dims = {gap, 0.22, 0.22};
                    s.pos = at(rMid + gap / 2 - 0.05, zBot + Lb * fz);
                    s.hollow = false;
                    s.color = Rgba8{120, 122, 128, 255};
                    s.massOverride = 20;
                    cr.comps.push_back(s);
                }
                continue;
            }
            default: continue;
            }
            cr.comps.push_back(r);
        }
    }
    return cr;
}

// ===================================================================== serialization
static std::string cleanName(const std::string& s) { return cleanLineText(s, 48); }

// a number written so that reading it back gives exactly the same double (shortest round-trip form)
struct Exact {
    double v;
};
static std::ostream& operator<<(std::ostream& os, Exact e) {
    char b[32];
    auto r = std::to_chars(b, b + sizeof b, e.v);
    return os.write(b, r.ec == std::errc() ? r.ptr - b : 0);
}

std::string craftToText(const Craft& c) {
    std::ostringstream os;
    os << brand::tag("CRAFT") << " 1\n";
    os << "name " << cleanName(c.name) << "\n";
    for (const auto& k : c.comps) {
        os << "comp " << (int)k.func << " " << (int)k.shape << " " << Exact{k.dims.x} << " " << Exact{k.dims.y} << " " << Exact{k.dims.z} << " "
           << Exact{k.pos.x} << " " << Exact{k.pos.y} << " " << Exact{k.pos.z} << " " << Exact{k.rotDeg.x} << " " << Exact{k.rotDeg.y} << " "
           << Exact{k.rotDeg.z} << " " << k.material << " " << k.hollow << " " << Exact{k.wall} << " "
           << (k.def >= 0 ? CATALOG.defs[k.def].id : std::string("-")) << " " << k.cluster << " " << (int)k.prop << " " << Exact{k.fill} << " "
           << k.stage << " " << k.symmetry << " " << (int)k.color.r << " " << (int)k.color.g << " " << (int)k.color.b << " " << Exact{k.massOverride}
           << " " << k.fireFrom << " " << Exact{k.taper} << " " << Exact{k.sweep} << " " << Exact{k.noseLen} << " " << Exact{k.tailLen} << " "
           << Exact{k.tailR} << " " << Exact{k.upsweep} << " " << Exact{k.bluff} << " " << Exact{k.boxy} << " " << k.dockTag << " " << k.dockedTo
           << " " << k.origin << " " << k.stage0 << " " << k.engOff << " " << k.rootSide << " " << k.foldDir << " " << Exact{k.flap} << " "
           << k.cargo << " " << Exact{k.rim} << "\n";
        os << "cname " << cleanName(k.name) << "\n";
    }
    os << "end\n";
    return os.str();
}

bool craftFromText(Craft& out, const std::string& text, size_t maxComps) {
    std::istringstream is(text);
    std::string line;
    if (!std::getline(is, line) || !brand::hasTag(line, "CRAFT")) return false;
    Craft c;
    bool lastKept = false;
    while (std::getline(is, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::istringstream ls(line);
        std::string key;
        ls >> key;
        if (key == "name") {
            std::string rest;
            std::getline(ls, rest);
            if (!rest.empty() && rest[0] == ' ') rest.erase(0, 1);
            c.name = cleanName(rest);
        } else if (key == "comp") {
            lastKept = false;
            Component k;
            int f, s, prop, r, g, b;
            std::string defId;
            ls >> f >> s >> k.dims.x >> k.dims.y >> k.dims.z >> k.pos.x >> k.pos.y >> k.pos.z >> k.rotDeg.x >> k.rotDeg.y >> k.rotDeg.z >>
                k.material >> k.hollow >> k.wall >> defId >> k.cluster >> prop >> k.fill >> k.stage >> k.symmetry >> r >> g >> b >> k.massOverride;
            if (ls.fail()) continue;
            {
                int ff = -1;
                if (ls >> ff) k.fireFrom = std::clamp(ff, -1, 31); // (absent in older files)
                double tp = 1, sw = 0;
                if (ls >> tp >> sw && std::isfinite(tp) && std::isfinite(sw)) {
                    k.taper = clampd(tp, 0.1, 1.0);
                    k.sweep = clampd(sw, -80, 80);
                }
                double lf[6];
                if (ls >> lf[0] >> lf[1] >> lf[2] >> lf[3] >> lf[4] >> lf[5]) {
                    bool fin = true;
                    for (double x : lf) fin &= std::isfinite(x);
                    if (fin) {
                        k.noseLen = clampd(lf[0], 0.02, 0.6);
                        k.tailLen = clampd(lf[1], 0.02, 0.7);
                        k.tailR = clampd(lf[2], 0.0, 1.0);
                        k.upsweep = clampd(lf[3], -1.0, 1.0);
                        k.bluff = clampd(lf[4], 0.0, 1.0);
                        k.boxy = clampd(lf[5], 0.0, 1.0);
                    }
                    int dt = 0, dto = 0, og = 0, s0 = -1;
                    if (ls >> dt >> dto >> og >> s0) {
                        k.dockTag = std::max(0, dt);
                        k.dockedTo = std::max(0, dto);
                        k.origin = std::max(0, og);
                        k.stage0 = std::clamp(s0, -1, 31);
                        unsigned long long eo = 0;
                        if (ls >> eo) {
                            k.engOff = eo & ((1ull << 37) - 1);
                            int rs = 0, fd = 0;
                            if (ls >> rs >> fd) {
                                k.rootSide = std::clamp(rs, -1, 1);
                                k.foldDir = std::clamp(fd, 0, 3);
                                double fl = 0, rm = 0.04;
                                int cg = 0;
                                if (ls >> fl >> cg >> rm) {
                                    k.flap = std::isfinite(fl) ? clampd(fl, 0, 60) : 0;
                                    k.cargo = cg != 0 ? 1 : 0;
                                    k.rim = std::isfinite(rm) ? clampd(rm, 0, 2) : 0.04;
                                }
                            }
                        }
                    }
                }
            }
            if (f < 0 || f >= (int)Func::COUNT || s < 0 || s >= (int)Shape::COUNT || prop < 0 || prop >= PROP_N) continue;
            bool finite = true;
            for (double v : {k.dims.x, k.dims.y, k.dims.z, k.pos.x, k.pos.y, k.pos.z, k.rotDeg.x, k.rotDeg.y, k.rotDeg.z, k.wall, k.fill, k.massOverride})
                if (!std::isfinite(v)) finite = false;
            if (!finite) continue;
            k.func = (Func)f;
            k.shape = (Shape)s;
            k.prop = (Prop)prop;
            double minXY = (k.shape == Shape::Cylinder) ? 0.0 : 0.001;
            k.dims = {clampd(k.dims.x, minXY, 500), clampd(k.dims.y, minXY, 500), clampd(k.dims.z, 0.001, 500)};
            if (k.shape == Shape::Cylinder && std::max(k.dims.x, k.dims.y) < 0.001) k.dims.x = 0.001;
            k.pos = {clampd(k.pos.x, -2000, 2000), clampd(k.pos.y, -2000, 2000), clampd(k.pos.z, -2000, 2000)};
            k.material = std::clamp(k.material, 0, MATERIAL_COUNT - 1);
            k.wall = clampd(k.wall, 0.0001, 1.0);
            k.def = defId == "-" ? -1 : CATALOG.find(defId);
            k.cluster = std::clamp(k.cluster, 1, 37);
            k.fill = clampd(k.fill, 0, 1);
            k.stage = std::clamp(k.stage, 0, 31);
            k.symmetry = std::clamp(k.symmetry, 1, 12);
            k.massOverride = clampd(k.massOverride, 0, 1e7);
            k.color = Rgba8{(unsigned char)std::clamp(r, 0, 255), (unsigned char)std::clamp(g, 0, 255), (unsigned char)std::clamp(b, 0, 255), 255};
            if (c.comps.size() < maxComps) {
                c.comps.push_back(k);
                lastKept = true;
            }
        } else if (key == "cname" && lastKept) {
            // the name belongs to the comp line just before it (skipped if that line was rejected)
            std::string rest;
            std::getline(ls, rest);
            if (!rest.empty() && rest[0] == ' ') rest.erase(0, 1);
            c.comps.back().name = cleanName(rest);
            lastKept = false;
        } else if (key == "end") break;
    }
    out = c;
    return true;
}

