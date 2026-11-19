// Part catalog and vessel designs
#pragma once
#include <string>
#include <vector>
#include "mathx.hpp"
#include "types.hpp"

// (values are stored in save files: append new propellants at the end)
enum class Prop { None = 0, Kerolox, Hydrolox, Methalox, Hypergolic, LH2, Xenon, Solid, Monoprop, JetFuel, COUNT };
constexpr int PROP_N = (int)Prop::COUNT;
const char* propName(Prop p);
double propDensity(Prop p);   // bulk density of the propellant mix [kg/m^3]
double tankDryFraction(Prop p);
Rgba8 propColor(Prop p);

enum class PK {
    Capsule, Probe, Tank, Engine, SRB, Decoupler, NoseCone, Adapter, HeatShield, Parachute,
    CMG, Battery, Solar, RTG, Reactor, Sail, Legs, Hyperdrive, RCS, Fairing,
    Rotor, Jet, Wheel, // aircraft (CAD designer)
    Docking,          // (appended)
    Module            // surface base module (habitat, greenhouse, ISRU plant, surface power): built into a base on landing
};
enum class Cat { Command = 0, Propellant, Engines, Structure, Power, Control, Utility, Fun, Aircraft, Base, COUNT };
// air-breathing engine cycles (PK::Jet)
enum class JetCycle { Turbofan = 0, Afterburner, Turboramjet, Scramjet, CombinedCycle };
const char* catName(Cat c);

struct PartDef {
    std::string id, name, desc;
    Cat cat = Cat::Structure;
    PK kind = PK::Tank;
    bool radial = false;
    bool diamAdj = false, lenAdj = false, clusterAdj = false;
    double diam = 1.25, length = 1.0; // fixed / default geometry [m]
    double dryMass = 0;               // fixed-size parts [kg]
    // engines
    Prop prop = Prop::None;
    double thrustVac = 0, ispVac = 0, ispSL = 0, gimbalDeg = 0, minThrottle = 0;
    double exitDiam = 1.0;            // nozzle exit diameter
    double powerUse = 0;              // electric engines [W at full throttle]; rover wheels: the drive motor
    double maxSpeed = 0;              // rover wheels: top speed of the geared drive [m/s]
    // solid boosters
    double srbProp = 0;
    // air-breathing engines (PK::Jet): thrustVac = sea-level static thrust at full power (with the afterburner
    // where there is one; the scramjet: thrust at 50 kPa dynamic pressure, Mach 6), ispVac = specific impulse of
    // the dry engine on its fuel
    JetCycle cycle = JetCycle::Turbofan;
    double dryThrust = 0;             // afterburning engines: the thrust without it [N]
    double ispWet = 0;                // afterburning engines: specific impulse of the afterburner [s]
    double machMax = 0;               // the inlet's limit (thrust fades out above it)
    double ramK = 0;                  // ram rise of the thrust with the Mach number (1 - 0.15 M + ramK M^2)
    double lapse = 0.7;               // thrust ~ (air density / sea level)^lapse
    // combined cycle: the closed (rocket) mode on board propellant - works at any height and in space
    double rocketThrust = 0, rocketIspVac = 0, rocketIspSL = 0;
    Prop rocketProp = Prop::None;
    // electrical
    double batteryJ = 0, genW = 0, drawW = 0, solarArea = 0, solarEff = 0;
    // control
    double torque = 0;                // [N m]
    double sailArea = 0;              // [m^2]
    double chuteCdA = 0;              // [m^2]
    int canopies = 1;                 // parachutes: canopies sharing chuteCdA
    double chuteQMax = 25000;         // parachutes: torn off above this dynamic pressure [Pa]
    double chuteJettisonV = 0;        // drag chutes: cast off below this ground speed on the runway [m/s] (0: never)
    double chuteReefK = 1, chuteReefT = 0; // reefing: drag area share and how long the canopy stays reefed [s]
    double chuteRiser = 0;            // riser from the attachment to the suspension lines [m] (0: short)
    double pilotChuteD = 0;           // the pilot chute that pulls the canopy out, drawn ahead of it [m]
    int crew = 0;
    // surface base modules (PK::Module): the building it becomes (ModuleKind, base.hpp) and the electric power it
    // supplies there [kW] (surface fission power)
    int baseModule = -1;
    double moduleKw = 0;
    Rgba8 color{200, 200, 205, 255};
};

// does the part produce thrust without air (a rocket engine, a solid motor, a combined-cycle engine)?
bool worksWithoutAir(const PartDef& d);

struct PartCatalog {
    std::vector<PartDef> defs;
    void init();
    int find(const std::string& id) const;
};
extern PartCatalog CATALOG;

struct RadialAttach {
    int def = 0;
    int count = 1;
    double offset = 0; // vertical shift [m] (side boosters: base above the bottom of the part they hang on)
    double angle = 0;  // rotation about the vehicle axis [deg]
};
// parts that can be attached to the side of a stack part (radial parts, and solid boosters)
bool sideMountable(const struct PartDef& d);
// side booster geometry next to a stack part of radius rParent: distance of the booster axis
double sideBoosterDistance(double rParent, const struct PartDef& booster);

struct PartInst {
    int def = 0;
    double diam = 1.25, diam2 = 1.25, length = 1.0;
    int cluster = 1;
    std::vector<RadialAttach> radial;
    bool hidden = false; // factory view only: drawn faintly, not picked (not saved, no effect on the vessel)
};

struct PartGeom {
    double height = 1, rTop = 0.5, rBot = 0.5;
};

PartGeom partGeom(const PartInst& p);
double partDryMass(const PartInst& p);       // includes radial attachments
double tankCapacity(const PartInst& p);      // propellant mass when full [kg]
double instThrustVac(const PartInst& p);     // total (cluster)
double instPowerUse(const PartInst& p);
PartInst makeInst(int def);
std::string partDescription(const PartDef& d); // localised

struct Design {
    std::string name = "New Vessel";
    std::vector<PartInst> stack; // top -> bottom
};
// z of the bottom of the stage that stack part i belongs to (the main engines' nozzle exits): side
// boosters stand on it, their offset is measured from there
double stageBottomZ(const Design& d, int i, const std::vector<double>& zb);

// Segments: bottom segment = index 0. A decoupler belongs to the segment below it.
std::vector<int> segmentOfParts(const Design& d, int& segCount);
// Interstage skirt: a decoupler with an engine directly above it carries a thin shell that covers
// the upper-stage engine (aerodynamic fairing and load path). It belongs to the lower stage and
// is dropped with it. z0 is measured from the top of the decoupler.
struct Interstage {
    bool on = false;
    double rBot = 0, rTop = 0, height = 0, wall = 0, mass = 0;
};
Interstage interstageFor(const Design& d, int decouplerIndex);
// Payload fairing (PK::Fairing): an ogive nose of 1.25 diameters on top of its stack position and a
// cylindrical skirt of 'length' that hangs down over the parts below it (the payload)
double fairingNoseLength(double diam);
// stack z of the bottom of every part (top part first, z grows upward from the bottom of the stack)
std::vector<double> stackBottoms(const Design& d);
// is part idx (fully) inside the skirt of a fairing above it?
bool coveredByFairing(const Design& d, int idx);

struct StageStats {
    double massStart = 0, massEnd = 0, dvVac = 0, dvSL = 0, twrSL = 0, burnTime = 0, thrustVac = 0;
    bool hasEngines = false;
};
struct DesignStats {
    double wetMass = 0, dryMass = 0, height = 0, maxDiam = 0;
    double totalDvVac = 0;
    double batteryJ = 0, genW = 0, solarW1AU = 0, drawW = 0, torque = 0;
    int crew = 0;
    bool hasControl = false, hasHyperdrive = false, hasSail = false, hasChute = false, hasLegs = false;
    std::vector<StageStats> stages; // index 0 = first to fire (bottom)
    std::vector<std::string> warnings;
};
DesignStats analyzeDesign(const Design& d);

std::string designToText(const Design& d);
bool designFromText(Design& d, const std::string& text);
