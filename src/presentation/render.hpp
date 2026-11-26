// 3D rendering: shaders, planets, starfield, vessel models, navball
#pragma once
#include "rl.hpp"
#include "base.hpp"
#include <cstdint>
#include <vector>
#include "craft.hpp"
#include "parts.hpp"
#include "universe.hpp"

namespace gfx {
// near-ground terrain detail: 0 Low, 1 Medium, 2 High, 3 Ultra (mesh density, the finest relief drawn)
void setTerrainQuality(int q);
// how far the detailed ground reaches (1 = standard): the fine relief shading and detail near the camera
void setDetailRange(float k);
// physically based shading (off: the standard look). Returns false when its shaders are not available
bool setPhysicalShading(bool on);
bool physicalShading();
// realistic shadows (physically based mode): shadow maps from the sun with soft penumbrae as wide as the sun's
// disc makes them (contact-hardening, PCSS). quality 0: 2048 px, 1: 4096 px. Returns false when not available
bool setShadows(bool on, int quality);
bool shadowsActive(); // on, the physically based mode on and the shadow map available
// the shadow pass: the casters (pad, bases, vessels) drawn between begin and end into a map seen from the sun.
// center: the middle of the region (camera-relative), toSun: unit vector, radius: half its width [m], back / ahead:
// how far it reaches toward the sun and away from it [m]. prevFbo / w / h: the target drawn to before (0: the screen)
bool beginShadowPass(const Vec3d& center, const Vec3d& toSun, double radius, double back, double ahead);
void endShadowPass(unsigned prevFbo, int w, int h, double sunAngDiam);
// the planet ground and the vessel surfaces drawn from now on receive the shadows of the last pass (off: none)
void shadowReceive(bool on);
bool shadowPassActive();
// physically based mode: the sky (Rayleigh / Mie scattering) as the background of the flight view, and the
// colour of the sunlight that reaches a point through the air (white in space, red at a sunset)
void drawPhysicalSky(const Universe& U, int body, const Vec3d& camRel, const Vec3d& fwd, const Vec3d& up, float fovyDeg, const Vec3d& sunDir);
Vector3 sunTintAt(const Universe& U, int body, const Vec3d& posRel, const Vec3d& sunDir);
void setSunTint(Vector3 c);
int terrainQuality();

struct Cam {
    Vec3d pos{};            // camera position in render space (usually origin)
    Vec3d fwd{1, 0, 0}, up{0, 0, 1};
    float fovy = 60;
};

void init();
void trimCaches(bool hard);   // release regenerable GPU/CPU caches under memory pressure
// bake at most one requested planet field texture; call once per frame outside any 3D pass
void serviceBakes();
size_t bakedFieldBytes();     // GPU memory held by baked planet fields
void setBakingEnabled(bool on); // false: fully procedural planets (comparison / fallback)
// finest noise octave drawn on planets (16 = full detail); the adaptive frame-rate control lowers it
void setPlanetDetail(int maxOctaves);
int planetDetail();
// sky / ground colours seen in reflections and hemisphere ambient (world "up" of the current scene)
void setEnvironment(const Vec3d& up, Vector3 sky, Vector3 ground);
void setSeaTime(double t); // clock of the sea's waves [s] (the caller stops it while paused)
void resetEnvironment();      // neutral studio lighting, +Z up (factory / CAD)

// Launch complex: hardstand with flame trench and deflector, launch mount with hold-down clamps,
// umbilical tower with swing arms, lightning mast, flood-light masts and a water tower.
struct PadSpec {
    double deckH = 7.5;       // top of the launch mount above the ground
    double rocketLen = 50;    // height of the vehicle on the pad
    double rocketR = 1.85;    // vehicle radius
    double armRetract = 0;    // 0 = swing arms connected, 1 = fully retracted
    float night = 0;          // 0 day .. 1 night: flood lights on
};
// hardstand (raised deck) edge length of the pad [m]
double padHardstandSize(const PadSpec& ps);
// soft shadow of a vessel on the ground: the shadow of its axis runs from a to b (camera-relative,
// on the ground plane with normal up), halfW wide, extended by ext at both ends
void drawGroundShadow(const Vec3d& a, const Vec3d& b, const Vec3d& up, double halfW, double ext, float alpha);
// flood-light heads and beam directions in the pad frame
int padFloodlights(const PadSpec& ps, Vec3d pos[4], Vec3d dir[4]);
// spotlights for vessels and pad structures (world, camera-relative); n = 0 switches them off
void setSpotLights(int n, const Vec3d* pos, const Vec3d* dir, Vector3 colour, float range);
// rot / origin: pad frame (x east, y north, z up, origin on the ground under the vehicle axis)
void drawLaunchPad(const PadSpec& ps, const Mat3d& rot, const Vec3d& origin, const Vec3d& lightDir, float light, const Vec3d& camPos,
                   float ambient);
// a surface base: its buildings in the base frame (x east, y north, z up; origin on the ground at the centre)
struct BaseModuleDraw {
    ModuleKind kind = ModuleKind::Habitat;
    int variant = 0;
    Vec3d pos;          // ground point of the module in the base frame [m]
    double heading = 0; // [rad]
};
struct BaseDrawSpec {
    std::vector<BaseModuleDraw> mods;
    Color soil{150, 120, 100, 255}; // the colour of the local ground (regolith shells, berms)
    float night = 0;                // windows, grow lights and beacons glow
    double rocketLen = 0, rocketR = 0;
    bool rocketOnPad = false;
};
void drawBase(const BaseDrawSpec& s, const Mat3d& rot, const Vec3d& origin, const Vec3d& lightDir, float light, const Vec3d& camPos,
              float ambient);
void shutdown();

// UI scale (logical px -> physical px). 2D drawing is done in logical units.
void setUiScale(float s);
// night vision, bright mode: light added to the ground of the planets (0 = off)
void setNightLift(float k);
// a solar panel's drawn pose while it opens: dep = deployed, sto = stowed (stowedGeometry), t 0..1
void solarPose(const Component& dep, const Component& sto, float t, Mat3d& R, Vec3d& pos, Vec3d& dims);
float uiScale();
// world -> logical screen coordinates for the current 3D camera
Vector2 project(Vector3 p, Camera3D cam);

void begin3D(const Cam& c, double nearP, double farP);
void end3D();
void clearDepth();
Camera3D current();

void drawStarfield(float brightness);
// Planet/star rendered as a sphere at 'center' (render units) with radius 'radiusUnits'
void drawBodySphere(const Universe& U, int body, const Vec3d& center, double radiusUnits, const Mat3d& rot,
                    const Vec3d& sunDir, float lightScale, bool halo, int octaves, bool surface = true);
// Saturn-style rings
void drawRings(const Universe& U, int body, const Vec3d& center, double unitsPerMeter, const Mat3d& rot, const Vec3d& sunDir);
// Near-surface patch of a body up to the horizon. camRel = camera position relative to the body centre [m]
void drawBodyCap(const Universe& U, int body, const Vec3d& camRel, const Mat3d& rot, const Vec3d& sunDir,
                 Vector3 fogColor, float fogK, float lightScale);
void drawGlow(Vector3 pos, float size, Color c);

struct VesselVisual {
    const Design* design = nullptr;
    int bottomSeg = 0;              // which segment is the active (bottom) one
    const std::vector<int>* seg = nullptr;
    const std::vector<double>* prop = nullptr;
    double throttle = 0;
    bool ignited = false;
    bool chute = false, sail = false, legs = true;
    int highlight = -1;
    const std::vector<char>* selMask = nullptr; // additional highlighted stack parts
    bool ghostHidden = false;                   // parts marked hidden are drawn faintly (factory)
    double pressureAtm = 0;
    bool infinite = false;
    bool hyperGlow = false;
};
// Draws a vessel with stack bottom at local z = 0, shifted by zOffset, rotated by rot and placed at origin.
void drawVessel(const VesselVisual& vv, const Mat3d& rot, const Vec3d& origin, double zOffset, const Vec3d& lightDir,
                float light, const Vec3d& camPos, float ambient);

struct RcsPuff {
    Vec3d pos, dir;   // craft frame: nozzle exit and exhaust direction
    float level = 0;  // 0..1
    float size = 1;   // scales with thrust
    bool electric = false; // battery-powered: faint blue plasma instead of a white gas puff
};

struct CraftVisual {
    const std::vector<Component>* comps = nullptr;
    const std::vector<double>* prop = nullptr;     // optional
    const std::vector<double>* heat = nullptr;     // optional (glow)
    int activeStage = 0;
    double throttle = 0;
    bool ignited = false;
    bool chute = false, sail = false, legs = true;
    double pressureAtm = 0;
    bool infinite = false, hyperGlow = false;
    int highlight = -1;                            // component index
    bool xray = false, wire = false;
    const std::vector<RcsPuff>* puffs = nullptr;  // firing RCS nozzles
    const std::vector<float>* engine = nullptr;   // per component thrust level from the simulation (flight)
    const std::vector<uint8_t>* jetMode = nullptr; // air-breathing engines: 1 dry, 2 afterburner, 3 rocket mode
    // atmospheric entry: plasma intensity 0..1 (from the stagnation heat flux) and the direction of
    // motion through the air in craft axes (the hot side faces it)
    float plasma = 0;
    Vec3d motionDir{0, 0, 1};
    // parachutes: inflation 0..1 and the direction from the craft to the canopy (craft axes: upstream
    // of the motion through the air)
    float chuteOpen = 1;
    Vec3d chuteDir{0, 0, 1};
    // deploy animations 0..1 (-1: from the legs / sail flags); stowed: the folded launch geometry of
    // comps (same order) that solar panels move from / to
    float solarT = 1, legsT = -1, sailT = -1;
    const std::vector<Component>* stowed = nullptr;
    float flapPos = 0;      // flap setting position 0..8 (between settings while they move)
    float bayT = 0;         // payload bay doors 0 closed .. 1 open (radiators deployed)
    float chuteReef = 1;    // parachute canopy diameter share while reefed
    bool editor = false;    // CAD: windows drawn as panes too (they may lie on no surface yet)
};
// Draws a craft; craft-frame point 'pivot' is placed at 'origin' and the craft is rotated by 'rot'.
void drawCraft(const CraftVisual& cv, const Mat3d& rot, const Vec3d& origin, const Vec3d& pivot, const Vec3d& lightDir, float light,
               const Vec3d& camPos, float ambient);

void drawNavball(Rectangle r, const Mat3d& localToHorizon);
Texture2D white();

} // namespace gfx
