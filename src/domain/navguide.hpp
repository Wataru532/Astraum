// Navigation to a target vessel: what the pilot has to do to dock with it.
//  far:  a rendezvous transfer (Lambert's problem over a grid of departure times and flight times, the
//        cheapest total of departure + arrival burn): when to burn and how much along prograde / normal /
//        radial, then the speed to match at the intercept
//  near: proximity operations along the target port's axis: the velocity change still needed, in the
//        vessel's own axes (RCS translation keys), and how far the ports are from facing each other
#pragma once
#include <string>
#include "mathx.hpp"

struct World;
struct Vessel;

namespace navg {

enum class Mode { None, Message, Transfer, Coast, Proximity, Docked, Launch };

struct Guidance {
    Mode mode = Mode::None;
    std::string message;        // Message: why there is no plan (and what to do)
    bool planning = false;      // Message: the first plan is being computed (a moment)
    // ---- Transfer / Coast
    double tBurn = 0;           // simulation time of the departure burn
    double dvPro = 0, dvNrm = 0, dvRad = 0; // the burn in the orbital frame at tBurn [m/s]
    double dvTotal = 0;         // |departure burn|
    double tArrive = 0;         // simulation time of the intercept
    double dvMatch = 0;         // relative speed to cancel at the intercept [m/s]
    double relIncDeg = 0;       // inclination between the two orbits
    // ---- Proximity
    Vec3d dvCraft;              // velocity change still needed, craft axes (+x left, +y up, +z forward) [m/s]
    double range = 0, closing = 0;
    double along = 0, lateral = 0; // the chaser port relative to the target port axis [m]
    double alignDeg = -1;       // angle between the port axes (-1: no free port pair)
    bool viaWaypoint = false;   // moving to the approach corridor first (beside / behind the target)
    // the burn direction (world axes, length = m/s): navball marker and SAS "Burn"
    Vec3d burn;
    // ---- Launch (on the ground, the target in orbit around the same body)
    double tLaunch = 0;         // simulation time of the next launch window (the site passes under the target's plane)
    double launchAzDeg = 0;     // heading along the target's plane there (from north, clockwise) [deg]
    double planeOffDeg = 0;     // the site's distance from the plane at the window (0: in it; > 0: it never gets closer)
    double offNowDeg = 0;       // ... now
};

// The next launch window from a site toward a target's orbit: the first time from `now` at which the site comes
// closest to the target's orbital plane (it is in the plane twice a day when the inclination exceeds the site's
// latitude, else nearest once a day). siteFixed: the site's direction in the body-fixed frame; h: the plane's
// normal (inertial). Returns false for a body that does not turn.
struct LaunchWindow {
    double t = 0;        // simulation time
    double azDeg = 0;    // the heading along the plane there (from north, clockwise)
    double offDeg = 0;   // the site's distance from the plane then [deg]
};
bool launchWindow(const World& W, int body, const Vec3d& siteFixed, const Vec3d& h, double now, LaunchWindow& out);

// compute the guidance for vessel a toward vessel t (call every frame; the transfer search runs at most
// every `searchEvery` real seconds and keeps its plan while it stays good)
Guidance compute(const World& W, const Vessel& a, const Vessel& t, double realNow, double searchEvery = 1.0);

// Lambert's problem (universal variables, single revolution): velocities on the conic from r1 to r2 in dt
// seconds, the short way when prograde about `h`. False when there is no solution
bool lambert(const Vec3d& r1, const Vec3d& r2, double dt, double mu, const Vec3d& h, Vec3d& v1, Vec3d& v2);

// the nearest pair of free docking ports (component indices); false when either has none
bool nearestPorts(const World& W, const Vessel& a, const Vessel& t, int& ia, int& it);

} // namespace navg
