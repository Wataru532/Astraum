// Port: the ground of the bodies as the simulation needs it (elevation for contacts and landings, the levelled
// ground of the surface bases, the detail window that follows the active vessel). The infrastructure's terrain
// module implements it from the embedded maps; without one installed every body is a smooth sphere.
#pragma once
#include <vector>
#include "mathx.hpp"

class GroundPort {
public:
    virtual ~GroundPort() = default;
    virtual bool hasRelief(int body) = 0;                         // a relief exists (generated on first use)
    virtual bool reliefReady(int body) = 0;                       // the relief is there already (nothing generated)
    virtual double height(int body, const Vec3d& dirBodyFixed) = 0; // [m] above the reference radius
    virtual double maxHeight(int body) = 0;                       // highest point [m]
    virtual unsigned revision(int body) = 0;                      // changes whenever the ground of the body changed
    virtual void focus(int body, const Vec3d& dirBodyFixed, double alt) = 0; // detail near the active vessel (-1: none)
    virtual void setFlatSpots(int body, const std::vector<Vec3d>& centres, double radius) = 0;
};

namespace ground {
void install(GroundPort* p); // nullptr: smooth spheres
GroundPort& port();
inline bool hasRelief(int body) { return port().hasRelief(body); }
inline bool reliefReady(int body) { return port().reliefReady(body); }
inline double height(int body, const Vec3d& d) { return port().height(body, d); }
inline double maxHeight(int body) { return port().maxHeight(body); }
inline unsigned revision(int body) { return port().revision(body); }
inline void focus(int body, const Vec3d& d, double alt) { port().focus(body, d, alt); }
inline void setFlatSpots(int body, const std::vector<Vec3d>& c, double r) { port().setFlatSpots(body, c, r); }
} // namespace ground
