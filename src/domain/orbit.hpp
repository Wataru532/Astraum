// Two-body orbital mechanics: conic elements and universal-variable propagation
#pragma once
#include "mathx.hpp"

struct Conic {
    double mu = 0;
    double a = 0;          // semi-major axis (negative for hyperbola)
    double e = 0;
    double inc = 0;        // inclination [rad]
    double p = 0;          // semi-latus rectum
    double nu = 0;         // true anomaly [rad]
    double pe = 0, ap = 0; // periapsis / apoapsis radius (ap < 0 => escape)
    double period = 0;     // 0 for open orbits
    double energy = 0;
    Vec3d P, Q, H;         // perifocal unit axes and angular momentum unit vector
    bool valid = false;
};

Conic stateToConic(const Vec3d& r, const Vec3d& v, double mu);
Vec3d conicPoint(const Conic& c, double nu);
// Time from now until the orbit reaches true anomaly nuTarget (forward); negative if unreachable
double timeToTrueAnomaly(const Conic& c, double nuTarget);
// Universal variable Kepler propagation (any conic)
void keplerPropagate(Vec3d& r, Vec3d& v, double mu, double dt);
