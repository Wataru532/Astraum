// Fusion-style 3D view navigation shared by every 3D view:
//   middle drag = pan, Shift + middle drag = orbit (around the point under the cursor),
//   wheel = zoom at cursor, Ctrl + Shift + middle drag = zoom, double middle click = fit,
//   ViewCube (faces / edges / corners / home / drag) and a navigation bar (orbit, pan, zoom tools, fit, home).
// Every view works in its own Z-up coordinate system; callers convert to their world frame.
#pragma once
#include "rl.hpp"
#include <functional>
#include "mathx.hpp"

namespace nav {

struct OrbitCam {
    Vec3d target;                 // look-at point / zoom & orbit centre
    double yaw = -PI / 2, pitch = 0.35, dist = 20;  // camera direction from target (yaw around +Z from +X)
    double fovy = 45;             // degrees (perspective)
    bool ortho = false;
    double minDist = 0.5, maxDist = 1e7;
    double homeYaw = -PI / 2 + 0.6, homePitch = 0.35;
    // smooth view transitions
    double animT = -1;
    double aYaw0 = 0, aPitch0 = 0, aYaw1 = 0, aPitch1 = 0, aDist0 = 0, aDist1 = 0;
    Vec3d aT0, aT1;
    int tool = 0;                 // navigation-bar tool for the left button: 0 none, 1 orbit, 2 pan, 3 zoom

    Vec3d dir() const;            // unit vector target -> camera
    Vec3d eye() const { return target + dir() * dist; }
    void basis(Vec3d& fwd, Vec3d& right, Vec3d& up) const;
    void animateTo(double yaw, double pitch, double dist, const Vec3d& target);
    void home(const Vec3d& centre, double radius);
    void fit(const Vec3d& centre, double radius);
    void update(double dt);       // advance animations
};

// project a point into logical screen coordinates of the view (false if behind the camera)
bool project(const OrbitCam& c, Rectangle view, const Vec3d& p, Vector2& out);
// ray through a logical screen point of the view
void viewRay(const OrbitCam& c, Rectangle view, Vector2 p, Vec3d& o, Vec3d& d);

using PickFn = std::function<bool(const Vec3d& o, const Vec3d& d, Vec3d& hit)>;
using FitFn = std::function<void()>;

// Mouse navigation inside 'view'. Returns true while a navigation drag is in progress.
bool handleMouse(OrbitCam& c, Rectangle view, bool hoverAllowed, const PickFn& pick, const FitFn& fit);
// ViewCube in the top-right corner of 'area'; returns true if the mouse is over it
bool viewCube(OrbitCam& c, Rectangle area, const FitFn& home);
Rectangle viewCubeRect(Rectangle area); // screen area occupied by the cube (for hover tests before drawing)
// Navigation bar centred at the bottom of 'area' (or at the given position)
void navBar(OrbitCam& c, Vector2 centreBottom, const FitFn& fit, const FitFn& home, bool allowOrtho);

// Render helpers for a 3D sub-viewport driven by an OrbitCam (perspective or orthographic)
void beginViewport3D(const OrbitCam& c, Rectangle view, double nearP, double farP);
void endViewport3D();
Vec3d renderEye(const OrbitCam& c); // camera position actually used for rendering

void setReverseZoom(bool r);
bool reverseZoom();
bool takeRightClick();       // right button released without dragging inside a navigated view (consumes it)
bool middleDragging();        // a middle-button navigation drag is active (e.g. to ignore Shift for throttle)

} // namespace nav
