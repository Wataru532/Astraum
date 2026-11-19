// Celestial bodies: Solar System + nearby stellar systems, on Keplerian rails
#pragma once
#include <array>
#include <string>
#include <vector>
#include "mathx.hpp"
#include "types.hpp"

enum class Surf { Rocky = 0, Earth = 1, Gas = 2, Star = 3, Icy = 4, Cloudy = 5, Desert = 6 };

struct Body {
    std::string name, system, desc;
    int parent = -1;        // -1 => root (a star fixed in the galactic frame)
    int root = 0;
    bool isStar = false;
    bool gasGiant = false;  // no solid surface
    bool comet = false;     // an icy nucleus: grows a coma and tails near the Sun
    double GM = 0, radius = 0;
    // orbit (relative to parent)
    double a = 0, e = 0, M0 = 0, n = 0, muOrbit = 0;
    Mat3d orbitFrame;       // perifocal -> ecliptic inertial
    Vec3d absPos;           // roots only (relative to the Sun)
    // rotation
    Mat3d poleFrame;        // body equatorial frame (W = 0) -> inertial
    double rotRate = 0, W0 = 0;
    // atmosphere
    double atmRho0 = 0, atmH = 1, atmP0 = 0, atmTop = 0;
    Vec3f atmColor{0, 0, 0};
    double soundSpeed = 300;  // representative speed of sound in the lower atmosphere [m/s]
    std::vector<std::array<double, 3>> atmTable; // optional {alt m, density kg/m^3, pressure Pa}
    // light
    double luminosity = 0;
    // appearance
    Surf surf = Surf::Rocky;
    Vec3f colA{0.5f, 0.5f, 0.5f}, colB{0.3f, 0.3f, 0.3f}, colC{1, 1, 1};
    float seed = 0;
    double ringInner = 0, ringOuter = 0;
    double soi = 1e300;
    std::vector<int> children;
};

struct BodyState {
    Vec3d pos, vel, acc; // relative to the body's root star (inertial, ecliptic J2000 axes)
};

struct Universe {
    std::vector<Body> bodies;
    std::vector<int> roots;
    std::vector<std::vector<int>> relevant; // per reference body: bodies whose gravity matters

    void init();
    int find(const std::string& name) const;
    int count() const { return (int)bodies.size(); }

    void computeStates(double t, std::vector<BodyState>& out) const;
    void computeStatesFor(double t, const std::vector<int>& which, std::vector<BodyState>& out) const;
    // position / velocity of body i relative to body j
    Vec3d relPos(const std::vector<BodyState>& s, int i, int j) const;
    Vec3d relVel(const std::vector<BodyState>& s, int i, int j) const;
    Mat3d bodyRot(int i, double t) const; // body-fixed -> inertial
    Vec3d angVel(int i) const;            // inertial rotation vector

    double density(int i, double alt) const;
    double pressure(int i, double alt) const;
    bool hasAtmosphere(int i) const { return bodies[i].atmTop > 0; }
    int hostStar(int i) const; // nearest star in hierarchy (the body itself if it is a star)
    std::string fullTitle(int i) const;
};

double surfaceGravity(const Body& b);
