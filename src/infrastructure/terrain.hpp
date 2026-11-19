// Surface relief. Earth, Moon and Mars use real maps (public-domain NASA / USGS data embedded in the
// executable: colour and elevation). The other solid bodies get a generated elevation map (craters and
// hills scaled to the body), made once on first use. Equirectangular layout of the planet shader (u:
// longitude from 180 W, v: colatitude). The same elevation feeds the ground contact of the physics and
// the rendered relief, so a vessel stands on the mountain it is drawn on.
#pragma once
#include <vector>
#include "ground.hpp"
#include "mathx.hpp"

struct Universe;

namespace terrain {

struct Map {
    int w = 0, h = 0;
    std::vector<float> height; // metres above the reference radius, row 0 = north pole
    float hMin = 0, hMax = 0;
    std::vector<unsigned char> colorFile; // encoded colour map (JPEG), decoded by the renderer; empty: generated relief
    bool generated = false;
};

// load the embedded maps and bind them to the bodies of U (call once after the universe exists)
void init(const Universe& U);
const Map* map(int body);   // nullptr: no relief (yet)
bool realColor(int body);   // a spacecraft colour map waits for the generated relief of this body
// the map of a body, generating the relief of a solid body without a real map now (~0.1 s, once)
const Map* ensure(int body);
// elevation at a body-fixed direction (bilinear, same sampling as the GPU) [m]; 0 without a map
// (generates the relief of the body if needed)
double height(int body, const Vec3d& dirBodyFixed);
// the same ground, with the fine relief only down to the wavelength minWl [m] (the drawn ground far from
// the camera: what would be smaller than its mesh spacing is left out)
double heightLod(int body, const Vec3d& dirBodyFixed, double minWl);
// the fine relief alone [m]: fractal hills and (airless bodies) small craters below the resolution of the
// maps, from ~2 texels down to about a metre; flat at the launch sites and on the sea
double microRelief(int body, const Vec3d& dirBodyFixed, double minWl);
// level the fine relief within radius [m] of these body-fixed directions (surface bases stand there); replaces the
// body's earlier list and changes revision() when it differs. Main thread; readers on other threads stay safe.
void setFlatSpots(int body, const std::vector<Vec3d>& centres, double radius);
double maxHeight(int body); // highest point (0 without a map)
// the domain's port (ground.hpp) served by these maps
GroundPort& groundPort();
double minHeight(int body); // lowest point (0 without a map; negative below the reference radius)
size_t bytes();             // CPU memory of the loaded maps

// ---- high-resolution tiles (assets/maps/*_hi.pak: 16 x 8 tiles per map, colour 8192 px around, elevation
// 5760-8192 px). Near a body the 4 x 4 tiles (90 x 90 degrees) below the active vessel are loaded - a worker
// thread decodes them - and used instead of the global map, blended into it at the window's border (for
// the physics and the drawing alike). Higher up, or at another body, they are dropped again.
constexpr int DET_GW = 16, DET_GH = 8, DET_WIN = 4;
struct Detail {
    int tx0 = 0, ty0 = 0; // window origin in tiles (x wraps around)
    int ctw = 0, cth = 0; // colour tile size [px] (0: no colour tiles)
    int htw = 0, hth = 0; // elevation tile size [px] (0: no elevation tiles)
};
// called by the simulation before each step with the active vessel: picks / moves / drops the window, takes
// the decoded tiles, and decodes the tiles right around the vessel at once (so the ground under it is the
// detailed one before the physics touches it)
void focus(int body, const Vec3d& dirBodyFixed, double alt);
// the altitude below which the detailed map tiles are used, as a multiple of the standard (0.6 body radii)
void setDetailRange(double k);
const Detail* detail(int body); // nullptr: no window on this body now
unsigned revision(int body);    // changes whenever the ground of the body changed (tiles loaded / dropped)
// the renderer: decoded tiles to upload (colour tiles are not kept on the CPU afterwards)
bool takeColorTile(int body, int& tx, int& ty, std::vector<unsigned char>& rgb);
bool takeHeightTile(int body, int& tx, int& ty, const float*& h);
void resendTiles(int body); // the renderer (re)made its textures: deliver the window again
// the patch of land painted under each launch complex (Earth colour; the same in both levels)
void paintLaunchSites(unsigned char* rgb, int w, int h, int ox, int oy, int fullW, int fullH);
size_t detailBytes(); // CPU memory of the loaded tiles

} // namespace terrain
