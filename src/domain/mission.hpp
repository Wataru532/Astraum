// Mission guidance toward a target body (the "Target" of the body panel): from the launch pad to an orbit
// around it. The phase follows from where the vessel is:
//   Prelaunch  on the ground: when to lift off (the launch site turned under the plane toward a moon, or the
//              interplanetary transfer window), the parking orbit to climb to
//   Ascent     climbing: the parking orbit (SAS Ascent flies it)
//   Departure  in orbit: the transfer burn - when, how much (Lambert's problem over departure times and flight
//              times; an interplanetary ejection from the parking orbit along the transfer's asymptote)
//   Transfer   coasting: the predicted encounter, or the course correction that brings one
//   Arrival    inbound at the body: the capture burn at periapsis
//   InOrbit    done: in orbit around the target
// Pure computation over the domain (no drawing): the flight screen shows it and SAS "Burn" follows `burn`.
#pragma once
#include <string>
#include "mathx.hpp"

struct World;
struct Vessel;

namespace mission {

enum class Phase { None, Message, Prelaunch, Ascent, Departure, Transfer, Arrival, InOrbit };

struct Plan {
    Phase phase = Phase::None;
    std::string message;        // English (tr): Message, and warnings of the other phases
    int target = -1;            // the target body
    int goal = -1;              // the body flown to now: the target, or the body on the way to it
    // ---- Prelaunch
    double tLaunch = -1;        // simulation time to lift off (-1: any time)
    bool transferWindow = false; // tLaunch is an interplanetary transfer window (days ahead, not minutes)
    double parkAlt = 0;         // the parking orbit to climb to [m above the surface]
    // ---- the vessel's orbit now (Ascent, Departure, Arrival, InOrbit) [m above the surface]
    double ap = 0, pe = 0;
    bool escaping = false;      // no apoapsis inside the sphere of influence
    // ---- the next burn (Departure, Transfer correction, Arrival)
    double tBurn = -1;          // simulation time (-1: none)
    double dv = 0;              // [m/s]
    Vec3d burn;                 // world axes, m/s (navball marker, SAS "Burn")
    double tWindow = -1;        // Departure: start of the transfer window (simulation time, -1: now)
    double tArrive = -1;        // planned arrival (simulation time)
    // ---- Transfer
    bool encounter = false;
    double tEncounter = 0;      // simulation time of the entry into the goal's sphere of influence
    double encPe = 0;           // periapsis there above the surface [m] (negative: impact)
    bool correction = false;    // the burn is a course correction (no encounter yet)
    bool impact = false;        // Transfer / Arrival: the path hits the surface (or dives deep into the air)
};

// the heavy searches are cached and refreshed every half second of real time (World::realTime)
Plan compute(const World& W, const Vessel& v, int target);
void invalidate(); // forget the cached plan (a jump in time, the tests)

} // namespace mission
