// Simulation world: vessels (free-form rigid bodies), physics integration, time warp, hyperspace, saving
#pragma once
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "base.hpp"
#include "craft.hpp"
#include "orbit.hpp"
#include "universe.hpp"

// Burn: along the velocity change the navigation asks for (Vessel::navBurn); Dock: the docking port toward the
// target vessel's port (appended: saved as ints)
enum class SAS { Off = 0, Stability, Prograde, Retrograde, Normal, AntiNormal, RadialOut, RadialIn, Target, AntiTarget, Sun, Ascent, Burn, Dock, COUNT };
const char* sasName(SAS s);
// SAS Ascent: guidance from the pad into a parking orbit at this altitude. Boost through a gravity
// turn until the apoapsis is there, coast, orbit insertion burn around the apoapsis, cut-off. It
// steers and works the throttle; staging stays with the pilot.
double ascentTargetAlt(const struct Body& b);

struct LaunchSite {
    const char* name;
    double latDeg, lonDeg;
};
extern const LaunchSite LAUNCH_SITES[];
extern const int LAUNCH_SITE_COUNT;

struct Settings {
    bool infiniteFuel = false;
    bool hyperspace = true;
    int hyperSpeed = 1;   // index into hyperspace speed table
    int launchSite = 0;
    int launchBase = 0;   // >0: rockets start from the launch pad of this surface base (id) instead of an Earth site
};
extern const double HYPER_SPEEDS[];
extern const char* HYPER_SPEED_NAMES[];
extern const int HYPER_SPEED_COUNT;

// Orbit standards of a body (altitudes above the surface). Low orbit: periapsis above the atmosphere
// (2 % of the radius for airless bodies) and apoapsis below the low-orbit ceiling (2,000 km for Earth,
// 30 % of the radius elsewhere). High orbit: the whole orbit above that ceiling. Synchronous: the
// period equals the rotation period (Earth: geostationary when also circular and equatorial).
struct OrbitBands {
    double lowMin = 0, lowMax = 0;
    double syncAlt = -1;   // circular synchronous altitude (-1: none inside the SOI)
};
OrbitBands orbitBands(const Body& b);
enum class OrbitClass { Unknown = -1, None = 0, Low, High, Synchronous, Spanning };
struct World;
struct Vessel;
// the class of a vessel's orbit against the body's standards (None: suborbital, escaping or on the ground)
OrbitClass orbitClassOf(const World& W, const Vessel& v, struct Conic& c);

// RCS energy source: hydrazine (catalytic monopropellant) or battery (electric mode: resistojet /
// pulsed-plasma operation of the same blocks at a tenth of the thrust, Isp 800 s, 50 % efficiency;
// its tiny reaction mass comes from the thrusters' own solid propellant bars)
constexpr double RCS_ELEC_THRUST = 0.10, RCS_ELEC_ISP = 800.0, RCS_ELEC_EFF = 0.5;
enum class RcsSource { Auto = 0, Monoprop, Battery };

struct RcsNozzle {
    Vec3d pos, dir;    // craft frame: nozzle exit, direction of the force it produces
    double F = 0, isp = 0;
};

struct Vessel {
    int id = 0;
    std::string name;
    // ---- structure (expanded components of the remaining craft)
    std::vector<Component> comps;
    std::vector<CompPhys> phys;
    std::vector<double> prop;      // per component: propellant left (tanks / solid motors)
    std::vector<double> heat;      // per component: accumulated thermal load (1 = failure)
    int stage = 0;                 // active (lowest remaining) stage group
    bool ignited = false;          // engines of the active stage lit
    std::shared_ptr<AeroTable> aero;
    MassProps mp;                  // current mass properties (craft frame)
    Mat3d invI;                    // inverse inertia tensor
    std::vector<Vec3d> contacts;   // craft frame contact points
    std::vector<char> contactLeg;  // contact belongs to a landing leg foot
    double length = 1, maxR = 0.5;
    double charge = 0, chargeCap = 0;

    // ---- state
    int ref = 0;                   // reference (SOI) body
    Vec3d r, v;                    // centre of mass relative to ref, inertial axes
    Quatd q;                       // craft frame -> inertial (craft +Z = nose)
    Vec3d w;                       // angular velocity (craft axes) [rad/s]
    bool landed = false;
    Vec3d landPos;                 // body-fixed CoM position when landed
    Quatd landQ;                   // body-fixed orientation when landed

    double throttle = 0;
    SAS sas = SAS::Off;
    // the roll a pointing SAS mode keeps: the angle of the craft's +Y about the nose from the local vertical,
    // taken when the mode was chosen (and while the pilot rolls), so the belly that faced the ground still does
    SAS sasRollMode = SAS::COUNT;
    bool sasRollSet = false;
    double sasRoll = 0;
    int target = -1;
    int targetVessel = -1;         // id of the vessel targeted for a rendezvous (-1: none)
    bool chuteDeployed = false, chuteCut = false, sailDeployed = false, legsDeployed = true;
    double chuteOpen = 0;          // canopy inflation 0..1
    bool destroyed = false;
    std::string fate;
    double launchTime = 0;
    bool solarDeployed = false;    // panels are stowed for launch
    bool rcs = false;              // RCS enabled
    int ascentPhase = 0;           // SAS Ascent: 0 boost, 1 coast to the apoapsis, 2 orbit insertion
    mutable double ascentLoft = 0; // SAS Ascent: the loft of the programme, fixed while side boosters fire (0: none yet)
    bool rolling = false;          // on its wheels on the ground, free to roll (take-off / landing run)
    // flaps: the setting selected (0..8) and where they are now (0..8, moving toward it at the actuator's rate)
    int flapStage = 0;
    double flapPos = 0;
    // payload bay doors: commanded open, how far open (0..1, both doors and the radiators, in game time)
    bool bayOpen = false;
    double bayT = 0;
    double chuteT = 0;             // time since the parachute was deployed (reefing) [s]

    // ---- derived geometry / RCS
    std::vector<Component> shown;  // geometry as flown (stowed panels folded); empty = comps
    std::vector<Component> stowedGeom; // launch configuration (folded panels), for the deploy animation
    // deploy animations, 0 stowed .. 1 deployed (visual; the physics switches at once). -1: not yet set
    float solarT = -1, legsT = -1, sailT = -1;
    const std::vector<Component>& geometry() const { return shown.empty() ? comps : shown; }
    std::vector<RcsNozzle> rcsNoz;
    std::vector<float> rcsFire;    // per nozzle firing level of the last step (visuals)
    std::vector<float> engLevel;   // per component: thrust actually produced / vacuum rating (0 = not burning)
    std::vector<uint8_t> jetMode;  // per component (air-breathing engines): 0 off, 1 dry, 2 afterburner, 3 rocket mode
    int ccMode = 0;                // combined-cycle engines: last mode (1 air, 3 rocket) - the switch is announced
    double rcsAuth = 0;            // rough maximum RCS torque [N m]
    Vec3d rcsAuthAxis;             // the same per craft axis (pitch, yaw, roll) [N m]
    double monoAmt = 0, monoCap = 0; // monopropellant on board [kg]
    double rcsUse = 0;             // monopropellant used in the current step [kg]
    Vec3d rcsImpulse;              // craft-frame impulse of the current step [N s]
    double rcsFireTime = 0;
    RcsSource rcsSource = RcsSource::Auto;
    bool rcsElectric = false;      // this step runs on battery power
    double rcsElecJ = 0;           // battery energy used this step [J]
    double rcsChargeAvail = 0;     // battery energy available at the start of the step [J]
    double tRcsPowerW = 0;         // telemetry: electric RCS power [W]
    // attitude-control thrusters of the active stage, fed from its main propellant (work with the
    // engines off; reaction wheels are used first, then RCS, then these)
    double acsF = 0, acsIsp = 0;   // thrust per thruster pair [N], Isp [s] (0: stage has none)
    Prop acsProp = Prop::None;
    double acsAmt = 0;             // propellant available to them at the start of the step [kg]
    double acsUse = 0;             // propellant they used in the current step [kg]
    double tAcs = 0;               // telemetry: fraction of their authority in use (0..1)
    double prevAlt = -1;           // altitude of the previous frame (milestone checks; -1 = unknown)
    OrbitClass orbitClass = OrbitClass::Unknown; // last announced orbit class (Unknown: set silently first)
    int orbitClassRef = -1;
    OrbitClass pendingClass = OrbitClass::Unknown;
    double pendingT = 0;           // real seconds the pending class has been stable
    int bound = -1;                // orbit bound to the current body: 1 yes, 0 escaping, -1 unknown

    // ---- cached totals
    double genW = 0, drawW = 0, solarArea = 0, sailArea = 0, chuteCdA = 0, torque = 0;
    double solarAreaMax = 0;       // all panels deployed
    int crew = 0;
    bool hasControl = false, hasLegs = false, hasHyperdrive = false, hasFairing = false;
    bool hasWheels = false, isAircraft = false; // aircraft: wheels, rotors or jets and no rocket engine
    bool hasFlaps = false;         // wing panels with flaps
    double flapMax = 0;            // the largest flap deflection of them [deg]
    int bayComp = -1;              // its payload bay (component index; -1: none)
    // its parachutes' limits (from the part): tear-off dynamic pressure, jettison ground speed (drag chute), reefing
    double chuteQMax = 25000, chuteJettV = 0, chuteReefK = 1, chuteReefT = 0;
    bool gearRetract = false;      // retractable landing gear (folds with the legs command)
    bool rotorcraft = false;       // lifted by a rotor (helicopter, multicopter): stands upright (+Z up), flies on its cyclic
    double drivePower = 0, driveVmax = 0; // rover: drive motors of the wheels [W], their top speed [m/s]
    Vec3d ctrlAuth;                // control-surface torque per pascal of dynamic pressure at full deflection (craft axes) [N m / Pa]
    Vec3d ctrlEff{1, 1, 1};        // what the surfaces really give against ctrlAuth now (measured in flight: a wing past its stall, at
                                   // hypersonic speed, gives far less than its low-speed lift slope promises)
    double ctrlEffT = 0;           // (game time of the last measurement)
    bool tailless = false;         // no pitch surface (a delta orbiter): the wings' trailing-edge flaps are elevons
    bool lifting = false;          // a winged vehicle (airfoil wings of some size): its stagnation heating sees small radii
    Vec3d surf;                    // control-surface position now (-1..1 per axis: pitch, yaw, roll), moved by the actuators
    double spool = 0;              // turbofans: the share of full thrust the engine has spooled up to (follows the throttle in seconds)
    unsigned groundRev = 0;        // terrain::revision of the ground this landed vessel was last seated on
    bool debris = false;           // jettisoned hardware (fairing halves, spent stages, boosters): no notifications
    // ---- view (the player's choice, saved)
    bool hidden = false;           // not drawn in the flight view or on the map (still simulated, can dock)
    Vec3d navBurn;                 // navigation: the velocity change still to make (world axes, m/s; set by the flight screen)
    bool showOrbit = true;         // its trajectory is drawn on the map
    // ---- achievements: only a vessel launched from the surface of Earth and flown without fun modes counts
    bool earthLaunch = false;      // launched from a pad or runway on Earth (not started elsewhere / in orbit)
    bool cheated = false;          // flown with infinite fuel or through hyperspace (sticky)
    bool reachedSpace = false;     // has been above the Karman line / out of Earth's sphere of influence
    uint64_t landedOn = 0, orbited = 0; // bodies (bit = body index) it has landed on / orbited
    bool countsForRecords() const { return earthLaunch && !cheated && !debris; }

    // ---- telemetry (last step)
    double tThrust = 0, tAccel = 0, tDynP = 0, tHeatFlux = 0, tPowerGen = 0, tPowerUse = 0, tSolarFlux = 0;
    double tDragAcc = 0, tLiftAcc = 0; // aerodynamic deceleration along the flow and lift across it [m/s^2]
    double tSailForce = 0, tMach = 0, tAlt = 0, tAoA = 0, tHeatMax = 0;
    int tHottest = -1;
    bool tInShadow = false, tFlameout = false, tPowerOut = false;
    Vec3d tTorque;                 // last external torque (craft axes)
};

struct Toast {
    std::string text;
    double born;
    Rgba8 color;
};


// flight history for "rewind": the dynamic part of the world at one moment
struct WorldSnapshot {
    double t = 0;
    std::vector<Vessel> vessels;
    int active = -1, nextId = 1;
    std::vector<unsigned> explored;
    std::vector<SurfaceBase> bases; // (a rewind also takes back a base built since: its lander comes back)
    int nextBaseId = 1;
};

// mission log: every notification, kept for the session (newest last)
struct LogEntry {
    double t;          // simulation time
    std::string text;
    Rgba8 color;
};

struct HyperJump {
    bool active = false;
    int target = -1;
    double phase = 0;      // real seconds elapsed
    double travelTime = 0; // game seconds
    double distance = 0;
    Vec3d dir{0, 0, 1};    // unit direction of travel (towards the destination, reference-body frame)
};

struct ControlInput {
    double pitch = 0, yaw = 0, roll = 0; // -1..1
    double tx = 0, ty = 0, tz = 0;       // RCS translation (craft axes: +x right, +y up, +z forward)
};

// things the active vessel did this frame that the presentation layer may react to (sound)
enum class SimEvent { Stage, Ignite, Explosion, Touchdown, HyperIn, HyperOut, PanelsTorn, Karman, WarpArrived, OrbitReached, Landed, Gravity };


#include "site.hpp" // launch pad deck height, runway

struct World {
    Universe U;
    Settings settings;
    double t = 0;                       // seconds since J2000
    std::vector<Vessel> vessels;
    int active = -1;
    int nextId = 1;
    int warpIndex = 0;
    double effectiveWarp = 1;
    std::string warpLimitReason;
    std::vector<unsigned> explored;     // per body bitmask: 1 visited SOI, 2 orbited, 4 landed
    std::deque<Toast> toasts;
    std::deque<LogEntry> missionLog;
    std::deque<WorldSnapshot> history;  // since the last launch, every ~2 s of game time (10 min kept)
    double historyReal = -1e9;
    void recordHistory(bool force);
    double rewindReach() const;         // how far back the history goes [s] (0: nothing to rewind)
    bool rewind(double seconds);        // back to the newest state at least `seconds` ago
    HyperJump hyper;
    double realTime = 0;
    ControlInput input;
    ControlInput ctrl; // the pilot's rotation input as the vessel sees it: keys ramp in over ~0.4 s (translation unchanged)
    double padRocketLen = 50, padRocketR = 1.85; // vehicle last placed on the pad (sizes the umbilical tower)
    std::vector<SimEvent> events;       // drained by the flight screen every frame
    struct Blast { int ref; Vec3d r; };
    std::map<int, std::string> moduleNames; // docked modules: their names (to give them back on undocking)
    std::string mapName;                    // the name the player gave this game (map), saved with it
    // surface bases on other worlds (base.hpp): founded and extended by landers that unload their modules
    std::vector<SurfaceBase> bases;
    int nextBaseId = 1;
    // where rockets start: an Earth launch site, or the pad of a surface base with a launch complex
    struct PadSite {
        int body = -1;
        Vec3d up{0, 0, 1};      // body-fixed unit direction of the pad
        double deckH = PAD_DECK_H;
        int baseId = 0;         // 0: an Earth launch site
    };
    PadSite padSite() const;
    std::vector<int> launchBaseIds() const; // bases whose launch pad is ready (ids)
    std::string launchSiteName() const;     // English name of the site rockets start from (a base: its name)
    // the Earth sites, then the bases with a launch pad: step through them (+1 next, -1 previous); the index of the
    // current one and how many there are
    void cycleLaunchSite(int step);
    int launchSiteSlot() const;
    int launchSiteSlots() const;
    // remove these components from a vessel (unloaded into a base); the vessel goes when nothing is left of it
    void unloadComponents(int vesselIndex, std::vector<int> comps);
    // tell the terrain where the bases level the ground (after bases were added, loaded or removed)
    void syncBaseTerrain() const;
    // the ground [m above the reference radius] under a body-fixed point (the domain's view of the terrain)
    double groundHeight(int body, const Vec3d& bodyFixed) const;
    // shift a landed vessel (body-fixed offset) and pin it there
    void moveLanded(Vessel& v, const Vec3d& delta);
    // achievements of this game (map): id -> when, and the vessel that earned it
    struct Achieved { double t = 0; std::string vessel; };
    std::map<std::string, Achieved> achievements;
    bool recordsOn = true;                  // false in tutorials (their worlds are not maps)
    std::vector<std::string> newAchievements; // earned this frame (drained by the flight screen: banner + sound)
    bool checkAchievements();               // (achieve.cpp) true when something new was earned
    void awardDocking(const Vessel& a, const Vessel& b);
    double dockHoldUntil = -1;              // no docking until then (the ports of an undocking still touch)
    double animT = -1;                      // game time of the last animation update (payload bay doors)
    std::vector<Blast> blasts;          // other (non-debris) vessels destroyed this frame, for the sound
    void event(const Vessel& v, SimEvent e) {
        if (active >= 0 && active < (int)vessels.size() && &vessels[active] == &v && events.size() < 64) events.push_back(e);
    }

    std::vector<BodyState> states;      // all bodies at time t (for rendering)

    void newGame(const Settings& s);
    void refreshStates() { U.computeStates(t, states); }
    Vessel* activeVessel() { return (active >= 0 && active < (int)vessels.size()) ? &vessels[active] : nullptr; }

    int launch(const Craft& c);         // returns vessel index
    // aircraft start: on the runway beside the launch complex (Earth) or at a landing site of another
    // body with an atmosphere (body >= 0); returns the vessel index
    int launchAircraft(const Craft& c, int body);
    // circular orbit at 'alt' above the vessel's current position (tutorial starts); clears the rewind history
    void placeInOrbit(Vessel& v, double alt);
    // drop the payload fairing of v; with 'debris' the two halves fly off as separate objects
    void jettisonFairing(Vessel& v, bool debris);
    // the jettisoned group of a staging as free objects: one per piece in contact (each side booster on its own),
    // pushed away from the vessel; appended only within the reserved capacity (callers hold Vessel pointers)
    // docking: called once per frame after the motion (a merge leaves the vessel array as it is: the docked one is
    // marked gone). undock(): split the vessel at the docked port pair of component 'port'; the lighter part becomes
    // a new vessel (appended within the reserved capacity). Returns false when it cannot
    void checkDocking();
    bool undock(Vessel& v, int port);
    // flaps: the next setting (8 -> 0)
    void cycleFlaps(Vessel& v);
    // payload bay: doors (open only in space), release of the payload (cargo components) in space, and taking a
    // vessel in with the arm (the closest one in reach, slow, small enough for the bay)
    void setBayDoors(Vessel& v, bool open);
    bool releasePayload(Vessel& v);
    int captureCandidate(const Vessel& v, std::string* why = nullptr) const; // vessel index or -1
    bool capturePayload(Vessel& v);
    // a port's face in world (body-relative) coordinates and its facing direction
    void portFrame(const Vessel& v, int i, Vec3d& pos, Vec3d& dir) const;
    // start a craft in orbit 'dist' metres behind the vessel with id 'nearId' (or in a 400 km orbit without one)
    int launchInOrbit(const Craft& c, int nearId, double dist);
    void releaseDebris(const Vessel& parent, const std::vector<Component>& comps, const std::vector<double>& prop,
                       const std::vector<double>& heat, const Vec3d& oldCom, const std::string& label);
    // keep the number of debris objects bounded (the oldest go first); call only where no Vessel pointers are held
    void pruneDebris(int keep);
    // toggle the landing legs (refused while legs are folded inside the payload fairing)
    void toggleLegs(Vessel& v);
    void update(double realDt);         // advance simulation
    void stage(Vessel& v);
    void trimMemory(bool hard);       // release caches under memory pressure
    void setSolarDeployed(Vessel& v, bool on);
    // "warp here": time warp along the current orbit until simulation time tTarget, then back to x1
    double warpTo = -1;
    int warpToRef = -1;
    bool startWarpTo(double tTarget);
    // guidance (tutorials): an event is due at this simulation time; time warp slows down ahead of it so
    // that it cannot be skipped (-1: none). Cleared once the time has passed.
    double warpHold = -1;
    void cancelWarpTo(const char* why);
    void toast(const std::string& s, Rgba8 c = {230, 236, 245, 255});

    double warpFactor() const;
    void setWarp(int idx);
    int maxWarpIndex() const;

    bool startHyperspace(int target, std::string& err);
    // why the vessel cannot jump from where it is (empty = it is in space: not landed, above the atmosphere)
    std::string hyperspaceLocationBlock(const Vessel& v) const;
    void updateHyperspace(double realDt);

    // the saved game as text, and back (a damaged or truncated text leaves the world as it was: false). Reading and
    // writing the files is the application's business (storage.hpp)
    std::string serialize() const;
    bool deserialize(const std::string& text);

    // helpers
    Vec3d sunDirFrom(int ref, const Vec3d& r) const;
    double solarFluxAt(int ref, const Vec3d& r, bool* shadow, Vec3d* dirToBrightest) const;
    Vec3d targetDirection(const Vessel& v, SAS mode) const;
    Vec3d surfaceVel(const Vessel& v) const;
    double vesselMass(const Vessel& v) const { return std::max(v.mp.mass, 1.0); }
};

// Next entry into the sphere of influence of a child of body `ref`, flying the conic from (r, v) at
// time t0 (two-body prediction, the crossing time refined by bisection)
// aircraft propulsion (sim.cpp): rotor / propeller thrust from momentum theory, turbofan thrust
double rotorThrust(double shaftW, double rho, double discArea, double axialSpeed, double tipLimitN);
double jetThrust(double staticSeaLevelN, double rho, double mach);
// an air-breathing engine (PK::Jet) at full power in the air of Earth: thrust [N], specific impulse [s] and
// mode (0 off, 1 dry / air-breathing, 2 afterburner, 3 rocket mode); q: dynamic pressure [Pa], patm [atm]
double jetPerformance(const PartDef& d, double rho, double mach, double q, double patm, double& isp, int& mode);

extern double g_epochOffset; // tests: shift of the start date [s] (--epoch <days>)

struct Encounter {
    int body = -1;
    double tEntry = 0; // seconds after t0
    Vec3d r, v;        // state relative to `body` at the crossing
};
bool predictEncounter(const Universe& U, int ref, Vec3d r, Vec3d v, double t0, Encounter& out);

extern const double WARP_LEVELS[];
extern const int WARP_COUNT;

// (re)build all derived data of a vessel from its components; keeps the CoM continuous
void setupVessel(Vessel& v, bool rebuildAero);
void updateMassProps(Vessel& v);
// stage helpers for the HUD
std::vector<int> vesselStages(const Vessel& v);
// an engine belongs to what burns now (its stage group, or a core beside boosters); the pilot's on/off choice aside
bool engineFiresNow(const Vessel& v, const Component& c);
double roverDrive(const World& W, const Vessel& v); // drive command of a rover (-1..1), 0 when not driving           // remaining stage groups, firing order
std::string stageEngineSummary(const Vessel& v, int stage);
void stageResources(const Vessel& v, double amt[PROP_N], double cap[PROP_N], double& srbAmt, double& srbCap, double& FvacSum,
                    double& mdotSum);

std::string formatDist(double m);
std::string formatTime(double s);
std::string formatDate(double t);
std::string formatSpeed(double ms);

