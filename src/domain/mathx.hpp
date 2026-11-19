// Double precision vector / quaternion / matrix helpers
#pragma once
#include <cmath>
#ifdef PI
#undef PI
#endif

constexpr double PI = 3.14159265358979323846;
constexpr double TAU = 2.0 * PI;
constexpr double DEG = PI / 180.0;
constexpr double G0 = 9.80665;          // standard gravity [m/s^2]
constexpr double C_LIGHT = 299792458.0; // [m/s]
constexpr double AU = 1.495978707e11;   // [m]
constexpr double LY = 9.4607304725808e15; // [m]
constexpr double DAY = 86400.0;
constexpr double YEAR = 365.25 * DAY;
constexpr double L_SUN = 3.828e26;      // [W]
constexpr double GM_SUN = 1.32712440018e20;
constexpr double R_SUN = 6.957e8;
constexpr double GM_EARTH = 3.986004418e14;
constexpr double R_EARTH = 6.371e6;
constexpr double P_ATM = 101325.0;

struct Vec3d {
    double x = 0, y = 0, z = 0;
    constexpr Vec3d() = default;
    constexpr Vec3d(double X, double Y, double Z) : x(X), y(Y), z(Z) {}
    Vec3d operator+(const Vec3d& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3d operator-(const Vec3d& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3d operator-() const { return {-x, -y, -z}; }
    Vec3d operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3d operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3d& operator+=(const Vec3d& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3d& operator-=(const Vec3d& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3d& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
};
inline Vec3d operator*(double s, const Vec3d& v) { return v * s; }
inline double dot(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3d cross(const Vec3d& a, const Vec3d& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double len(const Vec3d& a) { return std::sqrt(dot(a, a)); }
inline double len2(const Vec3d& a) { return dot(a, a); }
inline Vec3d norm(const Vec3d& a) { double l = len(a); return l > 1e-300 ? a / l : Vec3d{0, 0, 1}; }
inline Vec3d anyPerp(const Vec3d& v) {
    Vec3d a = std::fabs(v.z) < 0.9 ? Vec3d{0, 0, 1} : Vec3d{1, 0, 0};
    return norm(cross(v, a));
}

// Row-major 3x3 matrix
struct Mat3d {
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    static Mat3d fromCols(const Vec3d& a, const Vec3d& b, const Vec3d& c) {
        Mat3d r;
        r.m[0][0] = a.x; r.m[1][0] = a.y; r.m[2][0] = a.z;
        r.m[0][1] = b.x; r.m[1][1] = b.y; r.m[2][1] = b.z;
        r.m[0][2] = c.x; r.m[1][2] = c.y; r.m[2][2] = c.z;
        return r;
    }
    Vec3d operator*(const Vec3d& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    Mat3d operator*(const Mat3d& o) const {
        Mat3d r;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
        return r;
    }
    Mat3d T() const {
        Mat3d r;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) r.m[i][j] = m[j][i];
        return r;
    }
    Vec3d col(int c) const { return {m[0][c], m[1][c], m[2][c]}; }
};
inline Mat3d rotX(double a) { Mat3d r; double c = std::cos(a), s = std::sin(a); r.m[1][1] = c; r.m[1][2] = -s; r.m[2][1] = s; r.m[2][2] = c; return r; }
inline Mat3d rotY(double a) { Mat3d r; double c = std::cos(a), s = std::sin(a); r.m[0][0] = c; r.m[0][2] = s; r.m[2][0] = -s; r.m[2][2] = c; return r; }
inline Mat3d rotZ(double a) { Mat3d r; double c = std::cos(a), s = std::sin(a); r.m[0][0] = c; r.m[0][1] = -s; r.m[1][0] = s; r.m[1][1] = c; return r; }

struct Quatd {
    double w = 1, x = 0, y = 0, z = 0;
    static Quatd axisAngle(const Vec3d& axis, double ang) {
        Vec3d a = norm(axis);
        double s = std::sin(ang * 0.5);
        return {std::cos(ang * 0.5), a.x * s, a.y * s, a.z * s};
    }
    Quatd operator*(const Quatd& q) const {
        return {w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w};
    }
    Quatd conj() const { return {w, -x, -y, -z}; }
    Quatd normalized() const {
        double l = std::sqrt(w * w + x * x + y * y + z * z);
        if (l < 1e-300) return {};
        return {w / l, x / l, y / l, z / l};
    }
    Vec3d rotate(const Vec3d& v) const {
        Vec3d u{x, y, z};
        Vec3d t = cross(u, v) * 2.0;
        return v + t * w + cross(u, t);
    }
    Mat3d toMat() const {
        return Mat3d::fromCols(rotate({1, 0, 0}), rotate({0, 1, 0}), rotate({0, 0, 1}));
    }
    static Quatd fromMat(const Mat3d& M) {
        Quatd q;
        double tr = M.m[0][0] + M.m[1][1] + M.m[2][2];
        if (tr > 0) {
            double s = std::sqrt(tr + 1.0) * 2;
            q.w = 0.25 * s; q.x = (M.m[2][1] - M.m[1][2]) / s; q.y = (M.m[0][2] - M.m[2][0]) / s; q.z = (M.m[1][0] - M.m[0][1]) / s;
        } else if (M.m[0][0] > M.m[1][1] && M.m[0][0] > M.m[2][2]) {
            double s = std::sqrt(1.0 + M.m[0][0] - M.m[1][1] - M.m[2][2]) * 2;
            q.w = (M.m[2][1] - M.m[1][2]) / s; q.x = 0.25 * s; q.y = (M.m[0][1] + M.m[1][0]) / s; q.z = (M.m[0][2] + M.m[2][0]) / s;
        } else if (M.m[1][1] > M.m[2][2]) {
            double s = std::sqrt(1.0 + M.m[1][1] - M.m[0][0] - M.m[2][2]) * 2;
            q.w = (M.m[0][2] - M.m[2][0]) / s; q.x = (M.m[0][1] + M.m[1][0]) / s; q.y = 0.25 * s; q.z = (M.m[1][2] + M.m[2][1]) / s;
        } else {
            double s = std::sqrt(1.0 + M.m[2][2] - M.m[0][0] - M.m[1][1]) * 2;
            q.w = (M.m[1][0] - M.m[0][1]) / s; q.x = (M.m[0][2] + M.m[2][0]) / s; q.y = (M.m[1][2] + M.m[2][1]) / s; q.z = 0.25 * s;
        }
        return q.normalized();
    }
    // Orientation whose local +Z points to fwd and local +Y is as close as possible to upHint
    static Quatd lookRot(const Vec3d& fwd, const Vec3d& upHint) {
        Vec3d f = norm(fwd);
        Vec3d r = cross(upHint, f);
        if (len(r) < 1e-9) r = anyPerp(f);
        r = norm(r);
        Vec3d u = cross(f, r);
        return fromMat(Mat3d::fromCols(r, u, f));
    }
};

inline double clampd(double v, double a, double b) { return v < a ? a : (v > b ? b : v); }
inline double lerpd(double a, double b, double t) { return a + (b - a) * t; }
inline double wrapAngle(double a) { a = std::fmod(a, TAU); if (a < 0) a += TAU; return a; }

// Equatorial (ICRF) -> ecliptic J2000
inline Vec3d eqToEcl(const Vec3d& v) {
    const double eps = 23.43928 * DEG;
    double c = std::cos(eps), s = std::sin(eps);
    return {v.x, c * v.y + s * v.z, -s * v.y + c * v.z};
}
inline Vec3d raDecToEcl(double raDeg, double decDeg) {
    double ra = raDeg * DEG, dec = decDeg * DEG;
    return eqToEcl({std::cos(dec) * std::cos(ra), std::cos(dec) * std::sin(ra), std::sin(dec)});
}
