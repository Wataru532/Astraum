// Free-form craft model (CAD): components with geometry, materials and functions,
// rigid-body mass properties and a precomputed aerodynamic coefficient table.
#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include "parts.hpp"

// Airfoil: a wing panel - physically the box of its span, mean chord and thickness; drawn with its taper,
// sweep and a symmetric airfoil section (root at the end toward the craft axis)
// Loft: a streamlined body (fuselage, pod, nacelle, boom): width x height x length, a nose and a tail that
// narrow smoothly, the tail swept up; physically the ellipsoid of the same size
enum class Shape { Box = 0, Cylinder, Ellipsoid, Wedge, Airfoil, Loft, Truss, COUNT }; // (appended: saved as ints)
enum class Func {
    Structure = 0, Tank, Engine, Command, Battery, Solar, RTG, Reactor, CMG, Parachute, Legs, Sail, Hyperdrive,
    HeatShield, Wing, RCS, Fairing, Rotor, Jet, Wheel, Docking, Window, CargoBay, COUNT // (appended: saved as ints)
};
const char* shapeName(Shape s);
const char* funcName(Func f);

struct Substance {
    const char* name;
    double density;   // kg/m^3
    double heatLimit; // sustainable stagnation-point heat flux [W/m^2]
    Rgba8 color;
};
extern const Substance MATERIALS[];
extern const int MATERIAL_COUNT;

struct Component {
    std::string name = "Component";
    Func func = Func::Structure;
    Shape shape = Shape::Cylinder;
    // Box: size x,y,z | Cylinder: bottom radius, top radius, height | Ellipsoid: radii | Wedge: size x,y,z
    Vec3d dims{1, 1, 1};
    Vec3d pos;          // centre, craft frame (+Z = nose)
    Vec3d rotDeg;       // XYZ Euler angles [deg]
    int material = 0;
    bool hollow = true;
    double wall = 0.004; // shell thickness [m]
    int def = -1;        // catalog part for functional components
    int cluster = 1;     // engines
    Prop prop = Prop::Kerolox;
    double fill = 1.0;   // tanks
    int stage = 0;       // burn / jettison group (0 = first)
    int fireFrom = -1;   // engines: also fire while the vessel is at this earlier group (core stage beside side boosters)
    int symmetry = 1;    // copies rotated around the craft Z axis
    Rgba8 color{200, 204, 212, 255};
    double massOverride = 0; // >0: use this dry mass [kg] instead of the computed one
    double taper = 1.0;      // Airfoil: tip chord / root chord (0.1-1)
    double sweep = 0.0;      // Airfoil: sweep of the leading edge [deg] (the tip moves toward local +Y), -80..80
    // Loft: nose and tail length (fractions of the length), size of the tail end (fraction of the section),
    // tail upsweep (fraction of the half height), nose bluntness (0 pointed .. 1 round), squareness of the
    // section (0 elliptic .. 1 boxy)
    double noseLen = 0.3, tailLen = 0.35, tailR = 0.3, upsweep = 0.0, bluff = 0.5, boxy = 0.0;
    // docking: a port's own tag and the tag of the port it is docked to (0: none); the module a component came
    // from (the id of the vessel it belonged to when that one docked) and its stage before the docking
    int dockTag = 0, dockedTo = 0, origin = 0, stage0 = -1;
    // flight: engines (nozzles of a cluster, jets) the pilot switched off - bit i = nozzle i of clusterLayout
    uint64_t engOff = 0;
    // wing panel (Airfoil): its root end - 0: the end toward the craft axis (from where the panel sits), -1 / +1:
    // local -x / +x
    int rootSide = 0;
    // retractable gear: where it folds - 0: from where it sits (main gear inward, nose gear aft), 1: toward the
    // craft's -X, 2: toward +X, 3: aft (-Z)
    int foldDir = 0;
    // wing panel (Airfoil): the flap's largest deflection [deg], 0: no flap. The flap is the inboard part of the
    // trailing edge (FLAP_SPAN of the span from the root, FLAP_CHORD of the chord)
    double flap = 0;
    // payload: carried in a payload bay (Func::CargoBay) and released from it in space
    int cargo = 0;
    // window (Func::Window): the width of its frame [m]; the frame and the surround have the component's colour
    double rim = 0.04;

    Quatd rot() const;
};

// flaps: settings 0 (up) .. FLAP_STAGES (full); setting 0 is 0 degrees, settings 1..8 a geometric sequence from
// 1 degree to the flap's full deflection (the same spread as an airliner's detents 1, 2, 5, 10, 15, 25, 30, 40)
constexpr double FLAP_SPAN = 0.6, FLAP_CHORD = 0.28;
constexpr int FLAP_STAGES = 8;
double flapSettingDeg(int setting, double maxDeg);  // the angle of a setting
double flapAngleAt(double pos, double maxDeg);      // continuous: pos 0..8 (between settings while the flaps move)
double flapPosOf(double deg, double maxDeg);        // the inverse

// what a component derives from where it sits relative to the craft axis, resolved
int airfoilRoot(const Component& c); // -1 / +1: the root end of a wing panel at local -x / +x
bool pointInShape(const Component& k, const Vec3d& pc, double tol); // inside the part as drawn (grown by tol)?
void airfoilRootEdge(const Component& k, std::vector<Vec3d>& out, int n); // points along a panel's root chord
int gearFold(const Component& c);    // 1 / 2 / 3: a retractable gear folds toward craft -X / +X / aft
// fix those (root end of a wing panel, fold of a gear) where they are now, so that the component keeps its shape
// when it is moved rigidly (a copy placed beside the original, a design added to another one)
void freezePlacement(Component& c);
// the mirror image of a component in the craft's YZ plane (x -> -x), reflected about an axis its shape is
// symmetric about (a wing panel in its thickness, a fuselage in its width, other shapes in local y)
Component mirroredX(const Component& c);
// a name for a line of a text file: printable ASCII and whole UTF-8 characters (no control characters), cut at a
// character boundary to at most maxBytes; "Unnamed" when nothing is left
std::string cleanLineText(const std::string& s, size_t maxBytes = 48);

// nozzles of an engine cluster on its mounting plate (component frame, z = 0); bellR: the bell radius that fits
std::vector<Vec3d> clusterLayout(int n, double plateR, double& bellR, double exitR);
int engineCount(const Component& c);    // nozzles of an engine component (cluster size; 1 for other engines)
int enginesOn(const Component& c);      // of them not switched off by the pilot
inline double engineShare(const Component& c) { int n = engineCount(c); return n > 0 ? (double)enginesOn(c) / n : 0.0; }

// the clamshell pieces of the payload fairing at stack index i (two halves, rotDeg.z 0 and 180)
void fairingPieces(const struct Design& d, int i, const std::vector<double>& zb, int stage, std::vector<Component>& out);

struct Craft {
    std::string name = "New Craft";
    std::vector<Component> comps;
};

// XYZ Euler angles [deg] of a rotation matrix (the convention of Component::rot)
Vec3d eulerDegOf(const Mat3d& R);

// --- geometry -----------------------------------------------------------------------------
double shapeVolume(Shape s, const Vec3d& d);
double shapeArea(Shape s, const Vec3d& d);
Vec3d shapeHalfExtent(Shape s, const Vec3d& d);       // local bounding half-size
bool shapeRay(Shape s, const Vec3d& d, const Vec3d& o, const Vec3d& dir, double& tHit); // local-frame ray hit
struct SurfSample { Vec3d p, n; double area; };
void shapeSurface(Shape s, const Vec3d& d, int res, std::vector<SurfSample>& out); // local frame
void shapeContactPoints(Shape s, const Vec3d& d, std::vector<Vec3d>& out);       // local frame
Vec3d shapeCentroid(Shape s, const Vec3d& d);

// expand symmetry copies (the returned components all have symmetry = 1)
std::vector<Component> expandCraft(const Craft& c);

// --- mass properties --------------------------------------------------------------------------
struct Moments {           // raw moments about the craft origin
    double m = 0;
    Vec3d s;               // first moment  (sum m*r)
    double q[3][3] = {{0}}; // second moment (sum m*r r^T)
    void add(const Moments& o);
    void addPoint(double mass, const Vec3d& r);
};
struct MassProps {
    double mass = 0;
    Vec3d com;             // craft frame
    Mat3d inertia;         // about CoM, craft axes [kg m^2]
    Vec3d principal;       // principal moments (sorted)
};
MassProps toMassProps(const Moments& M);

// Per-component cached physical data (craft frame, after expansion)
struct CompPhys {
    double dryMass = 0;    // structure / hardware
    double propCap = 0;    // propellant capacity (tanks, solid motors)
    Moments dryM;          // moments of the dry mass
    Moments unitPropM;     // moments of 1 kg of propellant
    Vec3d thrustPoint, thrustDir; // engines (craft frame)
    double thrustVac = 0, ispVac = 0, ispSL = 0, gimbalDeg = 0, minThrottle = 0, powerUse = 0;
    double batteryJ = 0, genW = 0, drawW = 0, solarArea = 0, torque = 0, sailArea = 0, chuteCdA = 0;
    double rcsF = 0, rcsIsp = 0; // RCS: thrust per nozzle [N], vacuum Isp [s]
    int crew = 0;
    double heatLimit = 4e5;
    Vec3d centre, axisZ;   // craft frame
    double radius = 0.5;   // bounding sphere
};
void computeCompPhys(const Component& c, CompPhys& out);
// Solar panel folded against its mount (stowed for launch). Other components are returned unchanged.
// hingeEnd: +1 / -1 the end along +/- its long axis, 0 the end nearer the craft axis
Component foldedPanel(const Component& c, int hingeEnd = 0);
// Launch configuration of a craft: radially mounted solar panels folded accordion-style into a stack
// lying flat against the side of the vehicle (about 2.0 x 2.4 m faces), clear of every other part in
// that height band and sector; other panels as foldedPanel
std::vector<Component> stowedGeometry(const std::vector<Component>& comps);
// a solar panel stowed as such a stack (hinged at its local -x end, pointing away from the craft axis)?
bool solarRadialMount(const Component& c);
// landing leg folded up along the side of its mount
Component stowedLeg(const Component& c);
// outline sample points of a component in the craft frame (cylinder rims at several heights, box
// corners and edge midpoints, ellipsoid surface)
void outlinePoints(const Component& k, std::vector<Vec3d>& out, int ring = 24);
// RCS block: nozzle positions / force directions in the craft frame (4 nozzles: local +y, -y, +z, -z)
void rcsNozzles(const Component& c, Vec3d pos[4], Vec3d forceDir[4]);

// --- aerodynamics -------------------------------------------------------------------------
// Coefficients per unit dynamic pressure, tabulated over the flow direction in the body frame.
struct AeroTable {
    int nt = 0, np = 0;
    std::vector<Vec3d> Fp, Tp, Ff, Tf;  // pressure / friction force and torque about the craft origin [m^2, m^3]
    std::vector<float> aproj;           // projected (frontal) area [m^2]
    std::vector<float> exposure;        // [dir * ncomp + comp] max windward cosine (heating)
    int ncomp = 0;
    uint64_t key = 0;                   // geometryKey of the shape it was built for
    double length = 1;                  // characteristic length for rate damping
    struct Sample { Vec3d F_p, T_p, F_f, T_f; double aproj; };
    Sample lookup(const Vec3d& flowDirBody, std::vector<float>* exposureOut) const;
};
std::shared_ptr<AeroTable> buildAeroTable(const std::vector<Component>& comps);
uint64_t geometryKey(const std::vector<Component>& comps); // a hash of the shapes and their placement
// the aero tables kept for reuse at most [bytes] (rebuild cost ~1 s per large craft)
constexpr size_t AERO_CACHE_BUDGET = (size_t)768 * 1024 * 1024;
// Same, but reuses a table built earlier for identical geometry (LRU cache bounded by AERO_CACHE_BUDGET).
std::shared_ptr<AeroTable> aeroTableCached(const std::vector<Component>& comps);
size_t aeroTableBytes(const AeroTable& t);
size_t aeroCacheBytes();
void trimAeroCache(size_t keepBytes); // drop least recently used tables until the cache holds <= keepBytes
// Lifting-surface model (Helmbold subsonic, Ackeret + tip loss supersonic, stall to a flat plate).
// vAir: velocity of the wing centre relative to the air (craft axes). Returns the force (craft axes).
// dAlpha: extra incidence of a deflected control surface (elevator / aileron / rudder part of the wing)
// a retractable landing-gear wheel folded up for flight (main gear inward, nose gear forward); the strut
// turns about its top (the hinge)
Component stowedWheel(const Component& c);
bool retractableWheel(const Component& c); // landing gear (not a driven rover wheel)
Component wheelAt(const Component& c, double deployed); // the gear on its way: 0 folded, 1 down
// flapRad: the flap deflection toward more lift along the panel's +Z (radians, signed)
Vec3d wingForce(const Component& c, const CompPhys& p, const Vec3d& vAir, double rho, double mach, double dAlpha = 0, double flapRad = 0);
double wingLiftSlope(const Component& c); // dCL / d alpha at low speed (per radian)

// --- whole-craft analysis (CAD / factory) -------------------------------------------------
struct CraftStage {
    int stage = 0;
    double massStart = 0, massEnd = 0, dvVac = 0, dvSL = 0, twrSL = 0, burnTime = 0, thrustVac = 0;
    bool hasEngines = false;
    Vec3d thrustTorqueAtStart; // disturbance torque from thrust misalignment [N m]
};
struct CraftStats {
    MassProps wet, dry;
    double totalDvVac = 0;
    double batteryJ = 0, genW = 0, solarW1AU = 0, drawW = 0, torque = 0;
    int crew = 0;
    bool hasControl = false;
    std::vector<CraftStage> stages;   // firing order
    Vec3d copAscent;                  // centre of pressure for flow along -Z (ascent), craft frame
    double staticMargin = 0;          // (CoM - CoP) along Z, in calibres (positive = stable)
    double frontalArea = 0;
    std::vector<std::string> warnings;
};
CraftStats analyzeCraft(const Craft& c, const AeroTable* aero);

// --- conversion & I/O -----------------------------------------------------------------------
Craft craftFromDesign(const Design& d);
std::string craftToText(const Craft& c);
// maxComps: 400 source components for .sfc designs; saved vessels hold expanded components (up to 400 x 12)
bool craftFromText(Craft& c, const std::string& text, size_t maxComps = 400);
Component makeComponent(Func f, int def = -1);

// Truss (Shape::Truss): a lattice beam in the box dims - four longerons along the longest side, a frame and a
// diagonal on every face each bay (bays about as long as the section is wide). The members as segments in the
// component's local frame, and the radius of their tubes.
void trussMembers(const Vec3d& dims, std::vector<std::pair<Vec3d, Vec3d>>& out);
double trussTubeRadius(const Vec3d& dims);
