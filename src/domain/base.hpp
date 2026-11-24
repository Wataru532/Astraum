// Surface bases: outposts on other worlds, built from the modules that landers bring down (a habitat, a
// greenhouse, an ISRU propellant plant, surface power, solar arrays). A base that houses a crew, makes its own
// propellant and has the power for it gets a launch pad: rockets can then be built and launched there.
// Pure domain logic: no drawing, no files, no platform code.
#pragma once
#include <string>
#include <vector>
#include "mathx.hpp"

struct Universe;
struct World;
struct Vessel;

// the buildings of a base (saved as ints: append only)
enum class ModuleKind { Habitat = 0, Greenhouse, Isru, Fission, Solar, Storage, Comms, LaunchPad, COUNT };
const char* moduleKindName(ModuleKind k); // English, for tr()

struct BaseModule {
    ModuleKind kind = ModuleKind::Habitat;
    double x = 0, y = 0;  // east / north of the base centre [m]
    double heading = 0;   // turn about the local vertical [rad]
    double kw = 0;        // electric power it supplies [kW] (fission plants, solar arrays)
    int variant = 0;      // one of several designs of its kind
};

struct SurfaceBase {
    int id = 0;
    std::string name;
    int body = -1;
    double lat = 0, lon = 0; // centre, body-fixed [rad]
    double founded = 0;      // simulation time
    std::vector<BaseModule> modules;
};

constexpr double BASE_LAUNCH_KW = 40.0;    // power a base needs before it may launch rockets [kW]
constexpr double BASE_REACH = 3000.0;      // a lander this close to a base adds its modules to it [m]
constexpr double BASE_FLAT_RADIUS = 900.0; // the fine relief is levelled within this distance of the centre [m]
constexpr double BASE_PAD_DECK_H = 2.0;    // height of the base's launch mount above the ground [m]
constexpr int BASE_MAX = 64, BASE_MAX_MODULES = 160;
// where rovers and aircraft started at a base stand: north of the buildings (east, north of the centre [m])
constexpr double DEPLOY_X = -40, DEPLOY_Y = 230;

struct BaseStatus {
    int habitats = 0, greenhouses = 0, isru = 0, crew = 0;
    double powerKw = 0, demandKw = 0;
    bool hasPad = false;
    // the launch complex is built once these are met: a crew can live there, propellant is made, enough power
    bool requirementsMet() const { return habitats > 0 && isru > 0 && powerKw >= BASE_LAUNCH_KW; }
};
BaseStatus baseStatus(const SurfaceBase& b);

// can a base stand on this body? (why not: English text for tr())
bool baseAllowed(const Universe& U, int body, std::string* why);
// body-fixed unit direction of a point of the base: x east, y north of the centre [m]
Vec3d baseDir(const SurfaceBase& b, double bodyRadius, double x, double y);
Vec3d baseCentreDir(const SurfaceBase& b);
// local axes of the base (body-fixed): east, north, up
void baseFrame(const SurfaceBase& b, Vec3d& east, Vec3d& north, Vec3d& up);
// latitude / longitude of a body-fixed direction [rad]
void dirToLatLon(const Vec3d& d, double& lat, double& lon);

// place a module in the base's layout (deterministic: by its kind and how many of that kind stand already)
void placeModule(SurfaceBase& b, BaseModule m);
// the launch complex (pad and propellant storage) once the requirements are met; true when it was added now
bool completeLaunchComplex(SurfaceBase& b);
// centre of the base's launch pad (x east, y north [m]); false without one
bool padOffset(const SurfaceBase& b, double& x, double& y);

// what a landed vessel would build: the modules from its catalog parts, fission reactors and solar panels
struct BasePlan {
    bool ok = false;
    std::string why;            // English (for tr()) when it cannot
    int joins = -1;             // index of the base it adds to (-1: founds a new one)
    std::vector<BaseModule> modules;
    std::vector<int> comps;     // the vessel's components that are unloaded into the base
};
BasePlan planBase(const World& W, int vesselIndex);
// carries out the plan: founds the base or extends the one nearby, unloads the components from the lander
// (the vessel is removed when nothing else is left of it). Returns the index of the base, -1 when it cannot.
int buildBase(World& W, int vesselIndex, const std::string& name);
// a default name for a new base on this body ("Mars Base 2")
std::string defaultBaseName(const World& W, int body);
int findBase(const World& W, int id);              // index by id (-1: none)
int baseNear(const World& W, int body, const Vec3d& dirBodyFixed, double maxDist); // index of the closest within reach
