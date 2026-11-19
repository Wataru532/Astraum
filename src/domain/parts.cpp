#include "parts.hpp"
#include "brand.hpp"
#include "craft.hpp"
#include "i18n.hpp"
#include <algorithm>
#include <cstdio>
#include <sstream>
#include <tuple>

PartCatalog CATALOG;

const char* propName(Prop p) {
    switch (p) {
    case Prop::Kerolox: return "Kerolox (LOX/RP-1)";
    case Prop::Hydrolox: return "Hydrolox (LOX/LH2)";
    case Prop::Methalox: return "Methalox (LOX/CH4)";
    case Prop::Hypergolic: return "Hypergolic (NTO/MMH)";
    case Prop::LH2: return "Liquid Hydrogen";
    case Prop::Xenon: return "Xenon";
    case Prop::Solid: return "Solid (APCP)";
    case Prop::Monoprop: return "Monopropellant (N2H4)";
    case Prop::JetFuel: return "Jet fuel (Jet-A)";
    default: return "None";
    }
}
double propDensity(Prop p) {
    switch (p) {
    case Prop::Kerolox: return 1030;    // O/F 2.56
    case Prop::Hydrolox: return 362;    // O/F 6.0
    case Prop::Methalox: return 830;    // O/F 3.6
    case Prop::Hypergolic: return 1160; // O/F 1.65
    case Prop::LH2: return 71;
    case Prop::Xenon: return 1500;      // supercritical storage
    case Prop::Solid: return 1750;
    case Prop::Monoprop: return 1004;   // hydrazine
    case Prop::JetFuel: return 804;     // kerosene: the oxidiser is the air
    default: return 0;
    }
}
double tankDryFraction(Prop p) {
    switch (p) {
    case Prop::Kerolox: return 0.05;
    case Prop::Hydrolox: return 0.09;
    case Prop::Methalox: return 0.05;
    case Prop::Hypergolic: return 0.06;
    case Prop::LH2: return 0.15;
    case Prop::Xenon: return 0.12;
    case Prop::Monoprop: return 0.10;   // blowdown tank with diaphragm
    case Prop::JetFuel: return 0.06;    // integral wing / fuselage tank
    default: return 0.1;
    }
}
Rgba8 propColor(Prop p) {
    switch (p) {
    case Prop::Kerolox: return {238, 239, 241, 255};   // white-painted aluminium-lithium
    case Prop::Hydrolox: return {205, 116, 52, 255};   // UV-tanned spray-on foam
    case Prop::Methalox: return {192, 196, 202, 255};  // stainless steel
    case Prop::Hypergolic: return {230, 230, 224, 255};
    case Prop::LH2: return {205, 116, 52, 255};
    case Prop::Xenon: return {52, 54, 60, 255};        // carbon overwrap
    case Prop::Monoprop: return {170, 172, 178, 255};  // titanium
    case Prop::JetFuel: return {196, 200, 206, 255};   // aluminium
    default: return {200, 200, 200, 255};
    }
}
const char* catName(Cat c) {
    switch (c) {
    case Cat::Command: return "Command";
    case Cat::Propellant: return "Tanks";
    case Cat::Engines: return "Engines";
    case Cat::Structure: return "Structure";
    case Cat::Power: return "Power";
    case Cat::Control: return "Control";
    case Cat::Utility: return "Utility";
    case Cat::Fun: return "Fun";
    case Cat::Aircraft: return "Aircraft";
    case Cat::Base: return "Base";
    default: return "";
    }
}

static PartDef mk(const char* id, const char* name, Cat cat, PK kind) {
    PartDef d;
    d.id = id;
    d.name = name;
    d.cat = cat;
    d.kind = kind;
    return d;
}

static PartDef engine(const char* id, const char* name, Prop prop, double kN, double ispSL, double ispVac,
                      double mass, double gimbal, double exitD, double len, double minThr, const char* desc) {
    PartDef d = mk(id, name, Cat::Engines, PK::Engine);
    d.prop = prop;
    d.thrustVac = kN * 1000.0;
    d.ispSL = ispSL;
    d.ispVac = ispVac;
    d.dryMass = mass;
    d.gimbalDeg = gimbal;
    d.exitDiam = exitD;
    d.length = len;
    d.minThrottle = minThr;
    d.diamAdj = true;
    d.clusterAdj = true;
    d.diam = std::max(1.25, exitD + 0.2);
    d.desc = desc;
    d.color = {70, 72, 78, 255};
    return d;
}

void PartCatalog::init() {
    defs.clear();
    // ---------------- Command ----------------
    {
        PartDef d = mk("capsule3", "Crew Capsule CC-3", Cat::Command, PK::Capsule);
        d.diam = 3.9; d.length = 3.3; d.dryMass = 8500; d.crew = 3;
        d.batteryJ = 30.0 * 3.6e6; d.drawW = 900; d.torque = 4000;
        d.desc = "3-seat capsule with life support, RCS and 30 kWh batteries. Base 3.9 m.";
        d.color = {228, 228, 232, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("capsule1", "Crew Capsule CC-1", Cat::Command, PK::Capsule);
        d.diam = 2.3; d.length = 2.3; d.dryMass = 2900; d.crew = 1;
        d.batteryJ = 8.0 * 3.6e6; d.drawW = 350; d.torque = 900;
        d.desc = "Compact single-seat capsule. Base 2.3 m.";
        d.color = {228, 228, 232, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("probe_s", "Probe Core PC-1", Cat::Command, PK::Probe);
        d.diam = 0.8; d.length = 0.35; d.dryMass = 30;
        d.batteryJ = 0.1 * 3.6e6; d.drawW = 15; d.torque = 3;
        d.desc = "Tiny flight computer. 100 Wh battery, small reaction wheels.";
        d.color = {60, 64, 72, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("probe_ring", "Avionics Ring PC-2", Cat::Command, PK::Probe);
        d.diam = 2.5; d.length = 0.6; d.dryMass = 160; d.diamAdj = true;
        d.batteryJ = 1.0 * 3.6e6; d.drawW = 80; d.torque = 40;
        d.desc = "Launch-vehicle avionics ring. Fits any diameter.";
        d.color = {52, 56, 64, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("probe_bus", "Deep Space Probe Bus DS-1", Cat::Command, PK::Probe);
        d.diam = 1.8; d.length = 1.6; d.dryMass = 420;
        d.batteryJ = 2.0 * 3.6e6; d.drawW = 120; d.torque = 8; d.genW = 0;
        d.desc = "Interplanetary probe body with high-gain antenna and 2 kWh battery.";
        d.color = {206, 168, 64, 255};
        defs.push_back(d);
    }
    // ---------------- Tanks ----------------
    struct T { const char* id; const char* name; Prop p; double d, l; const char* desc; };
    for (T t : {T{"tank_kerolox", "Kerolox Tank", Prop::Kerolox, 3.7, 12, "LOX / RP-1. Dense, simple, good for boosters."},
                T{"tank_hydrolox", "Hydrolox Tank", Prop::Hydrolox, 5.0, 15, "LOX / LH2. Highest chemical Isp, low density."},
                T{"tank_methalox", "Methalox Tank", Prop::Methalox, 9.0, 20, "LOX / methane. Good Isp and density."},
                T{"tank_hyper", "Hypergolic Tank", Prop::Hypergolic, 1.25, 1.5, "NTO / MMH. Storable for years, ignites on contact."},
                T{"tank_lh2", "Liquid Hydrogen Tank", Prop::LH2, 3.7, 10, "Pure LH2 for nuclear thermal engines."},
                T{"tank_xenon", "Xenon Tank", Prop::Xenon, 0.8, 0.8, "High-pressure xenon for ion and Hall thrusters."},
                T{"tank_mono", "Monopropellant Tank", Prop::Monoprop, 1.0, 0.8, "Hydrazine for RCS thrusters. Feeds every RCS block on the vessel."}}) {
        PartDef d = mk(t.id, t.name, Cat::Propellant, PK::Tank);
        d.prop = t.p; d.diam = t.d; d.length = t.l; d.diamAdj = true; d.lenAdj = true;
        d.desc = t.desc;
        d.color = propColor(t.p);
        defs.push_back(d);
    }
    // ---------------- Engines ----------------
    defs.push_back(engine("eng_kb845", "Kerolox Booster KB-845", Prop::Kerolox, 914, 282, 311, 470, 5, 0.92, 2.6, 0.40,
                          "Workhorse sea-level gas-generator engine. Cluster it under boosters."));
    defs.push_back(engine("eng_kv981", "Kerolox Vacuum KV-981", Prop::Kerolox, 981, 100, 348, 490, 3, 3.0, 4.2, 0.40,
                          "Large-expansion upper-stage engine. Useless at sea level."));
    defs.push_back(engine("eng_hm2279", "Hydrolox Main HM-2279", Prop::Hydrolox, 2279, 366, 452, 3527, 10.5, 2.4, 4.3, 0.67,
                          "Staged-combustion hydrolox main engine."));
    defs.push_back(engine("eng_hu110", "Hydrolox Upper HU-110", Prop::Hydrolox, 110, 200, 465, 301, 4, 1.5, 2.3, 0.10,
                          "Efficient expander-cycle upper stage engine."));
    defs.push_back(engine("eng_mh2600", "Methalox Booster MH-2600", Prop::Methalox, 2560, 327, 350, 1600, 15, 1.3, 3.1, 0.40,
                          "Full-flow staged combustion. High thrust-to-weight."));
    defs.push_back(engine("eng_mv2700", "Methalox Vacuum MV-2700", Prop::Methalox, 2700, 120, 380, 2000, 2, 2.3, 4.0, 0.40,
                          "Vacuum-optimised methalox engine."));
    defs.push_back(engine("eng_ho27", "Hypergolic Orbital HO-27", Prop::Hypergolic, 27, 150, 316, 118, 6, 1.0, 1.9, 0.5,
                          "Reliable restartable spacecraft engine."));
    defs.push_back(engine("eng_hl45", "Hypergolic Lander HL-45", Prop::Hypergolic, 45, 230, 311, 180, 6, 1.5, 2.1, 0.10,
                          "Deep-throttling descent engine for landers."));
    defs.push_back(engine("eng_nt333", "Nuclear Thermal NT-333", Prop::LH2, 333, 380, 850, 6800, 3, 1.4, 6.0, 0.30,
                          "Solid-core fission rocket. Heats hydrogen to 2,500 K. Isp 850 s."));
    {
        PartDef d = engine("eng_ix1", "Gridded Ion Thruster IX-1", Prop::Xenon, 0.000092, 0, 3100, 50, 0, 0.3, 0.4, 0.10,
                           "92 mN at 2.3 kW. Isp 3100 s. Needs electric power.");
        d.powerUse = 2300; d.diam = 0.8; d.color = {120, 130, 150, 255};
        defs.push_back(d);
    }
    {
        PartDef d = engine("eng_ht5", "Hall Thruster HT-5", Prop::Xenon, 0.00029, 0, 1950, 30, 0, 0.25, 0.3, 0.10,
                           "290 mN at 4.5 kW. Isp 1950 s. Needs electric power.");
        d.powerUse = 4500; d.diam = 0.8; d.color = {120, 130, 150, 255};
        defs.push_back(d);
    }
    {
        PartDef d = engine("eng_ht100", "Nested Hall Thruster HT-100", Prop::Xenon, 0.0054, 0, 2600, 230, 0, 0.9, 0.6, 0.10,
                           "5.4 N at 100 kW. Pair with a fission reactor.");
        d.powerUse = 100000; d.diam = 1.25; d.color = {120, 130, 150, 255};
        defs.push_back(d);
    }
    for (auto s : {std::make_tuple("srb_1", "Solid Booster SB-1", 1.3, 12.0, 12000.0, 1400.0, 600.0, 250.0, 275.0),
                   std::make_tuple("srb_2", "Solid Booster SB-2", 2.5, 24.0, 120000.0, 12000.0, 3500.0, 245.0, 270.0),
                   std::make_tuple("srb_3", "Solid Booster SB-3", 3.7, 45.0, 500000.0, 91000.0, 12500.0, 242.0, 268.0)}) {
        PartDef d = mk(std::get<0>(s), std::get<1>(s), Cat::Engines, PK::SRB);
        d.diam = std::get<2>(s); d.length = std::get<3>(s); d.srbProp = std::get<4>(s); d.dryMass = std::get<5>(s);
        d.thrustVac = std::get<6>(s) * 1000.0; d.ispSL = std::get<7>(s); d.ispVac = std::get<8>(s);
        d.prop = Prop::Solid; d.exitDiam = d.diam * 0.7; d.gimbalDeg = 4;
        d.desc = "Solid rocket motor. Cannot be throttled or shut down once lit.";
        d.color = {236, 236, 236, 255};
        defs.push_back(d);
    }
    // ---------------- Structure ----------------
    {
        PartDef d = mk("decoupler", "Stage Decoupler", Cat::Structure, PK::Decoupler);
        d.diamAdj = true; d.diam = 2.5; d.length = 0.3;
        d.desc = "Separates stages. Everything below it is dropped when staged. With an engine directly above, it carries an interstage skirt that covers that engine and is dropped with the lower stage.";
        d.color = {176, 178, 184, 255}; // aluminium separation ring
        defs.push_back(d);
    }
    {
        PartDef d = mk("nose", "Aerodynamic Nose Cone", Cat::Structure, PK::NoseCone);
        d.diamAdj = true; d.diam = 2.5;
        d.desc = "Reduces drag on the top of the stack.";
        d.color = {236, 236, 240, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("fairing", "Payload Fairing PF-1", Cat::Structure, PK::Fairing);
        d.diamAdj = true; d.lenAdj = true; d.diam = 4.0; d.length = 4.0;
        d.desc = "Two-piece composite shell protecting the payload below it from airflow and heating during ascent. Length is the cylindrical section that covers the payload. Jettisoned automatically above about 110 km.";
        d.color = {238, 238, 240, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("adapter", "Diameter Adapter", Cat::Structure, PK::Adapter);
        d.diamAdj = true; d.diam = 2.5;
        d.desc = "Joins parts of different diameters. Top / bottom sizes adjustable.";
        d.color = {180, 184, 192, 255};
        defs.push_back(d);
    }
    // ---------------- Power ----------------
    {
        PartDef d = mk("bat_s", "Battery Pack 1 kWh", Cat::Power, PK::Battery);
        d.radial = true; d.dryMass = 7; d.batteryJ = 1.0 * 3.6e6;
        d.desc = "Lithium-ion battery, 1 kWh (radial).";
        d.color = {60, 62, 70, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("bat_l", "Battery Bank 20 kWh", Cat::Power, PK::Battery);
        d.radial = true; d.dryMass = 130; d.batteryJ = 20.0 * 3.6e6;
        d.desc = "Large lithium-ion bank, 20 kWh (radial).";
        d.color = {60, 62, 70, 255};
        defs.push_back(d);
    }
    for (auto s : {std::make_tuple("solar_s", "Solar Panel SP-2", 2.0, 9.0),
                   std::make_tuple("solar_m", "Solar Array SA-40", 40.0, 120.0),
                   std::make_tuple("solar_l", "Roll-Out Array RA-150", 150.0, 330.0)}) {
        PartDef d = mk(std::get<0>(s), std::get<1>(s), Cat::Power, PK::Solar);
        d.radial = true; d.solarArea = std::get<2>(s); d.dryMass = std::get<3>(s); d.solarEff = 0.30;
        d.desc = ""; // generated (localised) by partDescription()
        d.color = {40, 60, 120, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("rtg", "Radioisotope Generator RTG-110", Cat::Power, PK::RTG);
        d.radial = true; d.dryMass = 45; d.genW = 110;
        d.desc = "Pu-238 thermoelectric generator. 110 W anywhere, day or night.";
        d.color = {90, 92, 96, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("reactor_s", "Fission Reactor FR-10", Cat::Power, PK::Reactor);
        d.diam = 1.25; d.length = 3.2; d.dryMass = 1500; d.genW = 10000;
        d.desc = "Compact uranium reactor with Stirling converters. 10 kWe.";
        d.color = {150, 150, 155, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("reactor_l", "Fission Reactor FR-1000", Cat::Power, PK::Reactor);
        d.diam = 3.7; d.length = 9.0; d.dryMass = 24000; d.genW = 1.0e6;
        d.desc = "Megawatt-class space reactor with radiators. 1 MWe.";
        d.color = {150, 150, 155, 255};
        defs.push_back(d);
    }
    // ---------------- Control ----------------
    {
        PartDef d = mk("cmg_s", "Control Moment Gyro CMG-260", Cat::Control, PK::CMG);
        d.diam = 1.25; d.length = 0.7; d.dryMass = 300; d.torque = 260; d.drawW = 150;
        d.desc = "260 N m of torque. Draws 150 W.";
        d.color = {180, 180, 190, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("cmg_l", "Control Moment Gyro CMG-2000", Cat::Control, PK::CMG);
        d.diam = 2.5; d.length = 1.1; d.dryMass = 1200; d.torque = 2000; d.drawW = 500;
        d.desc = "2,000 N m of torque. Draws 500 W.";
        d.color = {180, 180, 190, 255};
        defs.push_back(d);
    }
    for (auto s : {std::make_tuple("rcs_22", "RCS Quad RQ-22", 22.0, 4.0, 229.0),
                   std::make_tuple("rcs_440", "RCS Quad RQ-440", 440.0, 22.0, 235.0),
                   std::make_tuple("rcs_3870", "RCS Pod RP-3870", 3870.0, 120.0, 240.0)}) { // (Space Shuttle primary RCS class)
        PartDef d = mk(std::get<0>(s), std::get<1>(s), Cat::Control, PK::RCS);
        d.radial = true; d.thrustVac = std::get<2>(s); d.dryMass = std::get<3>(s); d.ispVac = std::get<4>(s); d.ispSL = std::get<4>(s) - 45;
        d.prop = Prop::Monoprop;
        d.desc = ""; // generated (localised) by partDescription()
        d.color = {70, 72, 78, 255};
        defs.push_back(d);
    }
    {
        // androgynous docking port after the NASA Docking System / IDSS: 1.6 m ring, 0.8 m passage, ~320 kg
        PartDef d = mk("dock_port", "Docking Port DP-1", Cat::Utility, PK::Docking);
        d.diam = 1.6;
        d.length = 0.5;
        d.dryMass = 320;
        d.desc = "Androgynous docking port (IDSS class). Two ports that meet face to face slower than 0.5 m/s and aligned within 12 degrees latch together: the vessels become one. Undock from the flight screen.";
        d.color = {200, 202, 206, 255};
        defs.push_back(d);
    }
    // ---------------- Surface base modules ----------------
    // Landed on another world they are unloaded and become the buildings of a base (base.hpp). A base with a
    // habitat, an ISRU propellant plant and 40 kW of power gets a launch pad: rockets can be launched from there.
    {
        // after inflatable surface habitats (NASA / Bigelow studies): packed for the trip, 4 crew once inflated
        PartDef d = mk("hab_module", "Surface Habitat SH-4", Cat::Base, PK::Module);
        d.diam = 4.2; d.length = 6.5; d.dryMass = 9500; d.crew = 4;
        d.baseModule = 0;
        d.desc = "Inflatable surface habitat, packed for the trip. Landed on another world it becomes the living quarters of a base for four (Build base). A base needs one to be founded.";
        d.color = {226, 224, 214, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("greenhouse", "Greenhouse Module GH-2", Cat::Base, PK::Module);
        d.diam = 3.5; d.length = 5.0; d.dryMass = 4200;
        d.baseModule = 1;
        d.desc = "Pressurised greenhouse with LED grow lights and hydroponic racks. Grows part of the crew's food at a base.";
        d.color = {196, 214, 200, 255};
        defs.push_back(d);
    }
    {
        // after the Mars ISRU studies (MOXIE scaled up, Sabatier reactor, water from ice): methane and oxygen
        PartDef d = mk("isru_plant", "ISRU Propellant Plant IP-5", Cat::Base, PK::Module);
        d.diam = 3.5; d.length = 4.5; d.dryMass = 6800;
        d.baseModule = 2;
        d.desc = "Makes rocket propellant from local resources (water ice, carbon dioxide): electrolysis and a Sabatier reactor. Rockets launched from the base are fuelled by it. Needs about 25 kW.";
        d.color = {214, 206, 188, 255};
        defs.push_back(d);
    }
    {
        // after NASA's Fission Surface Power project: 40 kWe, about 6 t, ten years of operation
        PartDef d = mk("surface_power", "Surface Fission Power FSP-40", Cat::Base, PK::Module);
        d.diam = 2.6; d.length = 4.2; d.dryMass = 5800;
        d.baseModule = 3; d.moduleKw = 40;
        d.desc = "Fission power plant for a surface base: 40 kWe day and night, set up a safe distance from the habitats. Fission reactors and solar panels carried by a lander count too.";
        d.color = {178, 180, 186, 255};
        defs.push_back(d);
    }
    // ---------------- Utility ----------------
    {
        PartDef d = mk("shield", "Ablative Heat Shield", Cat::Utility, PK::HeatShield);
        d.diamAdj = true; d.diam = 3.9;
        d.desc = "Lets the vessel survive atmospheric entry when facing the airflow.";
        d.color = {90, 60, 40, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("chute", "Parachute PR-25", Cat::Utility, PK::Parachute);
        d.diam = 0.9; d.length = 0.6; d.dryMass = 140; d.chuteCdA = 420;
        d.desc = "25 m main parachute. Deploy below ~25 kPa dynamic pressure. Key: P";
        d.color = {220, 120, 40, 255};
        defs.push_back(d);
    }
    {
        // three 35 m ringsail mains, as on Orion (about 8 m/s with a 10 t capsule)
        // (four mains: three 35 m canopies left a 10 t capsule at ~8 m/s, close to what its heat shield takes)
        PartDef d = mk("chute_cluster", "Parachute Cluster PR-4x38", Cat::Utility, PK::Parachute);
        d.diam = 1.6; d.length = 0.9; d.dryMass = 900; d.chuteCdA = 3850; d.canopies = 4;
        d.desc = "Four 38 m main parachutes for crew capsules: about 6.5 m/s touchdown with 10 t. Deploy below ~25 kPa dynamic pressure. Key: P";
        d.color = {220, 120, 40, 255};
        defs.push_back(d);
    }
    {
        // Space Shuttle orbiter drag chute (in service from STS-49, 1992): a 12.2 m (40 ft) ring-sail canopy
        // pulled out by a 2.7 m pilot chute from a compartment at the base of the fin, reefed to 40 % of its
        // diameter for 3.5 s, trailing 27 m behind on a 12.6 m riser. Deployed on the runway between main- and
        // nose-gear touchdown (195 kt preferred, no more than ~230 kt), cast off at 60 kt ground speed so it
        // cannot fall on the main engine bells. Ring sail Cd0 ~0.55 on its 117 m2 nominal area
        PartDef d = mk("chute_drag", "Drag Chute DC-12", Cat::Aircraft, PK::Parachute);
        d.diam = 0.9; d.length = 1.4; d.dryMass = 260; d.chuteCdA = 64;
        d.chuteQMax = 0.5 * 1.225 * 118.3 * 118.3; // 230 knots at sea level
        d.chuteJettisonV = 30.9;                    // 60 knots
        d.chuteReefK = 0.16; d.chuteReefT = 3.5;    // 40 % of the diameter
        d.chuteRiser = 12.6; d.pilotChuteD = 2.7;
        d.desc = "Landing drag chute (12 m ring sail) for winged vehicles: deploy on the runway below 230 kt. Cast off by itself at 60 kt. Key: P";
        d.color = {200, 200, 196, 255};
        defs.push_back(d);
    }
    {
        PartDef d = mk("legs", "Landing Leg", Cat::Utility, PK::Legs);
        d.radial = true; d.dryMass = 80;
        d.desc = "Absorbs touchdowns up to 8 m/s. Key: G to toggle.";
        d.color = {170, 170, 176, 255};
        defs.push_back(d);
    }
    for (auto s : {std::make_tuple("sail_s", "Solar Sail SS-100", 100.0, 5.0),
                   std::make_tuple("sail_m", "Solar Sail SS-10K", 10000.0, 120.0),
                   std::make_tuple("sail_l", "Solar Sail SS-1M", 1.0e6, 3500.0)}) {
        PartDef d = mk(std::get<0>(s), std::get<1>(s), Cat::Utility, PK::Sail);
        d.diam = 0.7; d.length = 0.5; d.sailArea = std::get<2>(s); d.dryMass = std::get<3>(s);
        d.desc = ""; // generated (localised) by partDescription()
        d.color = {200, 200, 210, 255};
        defs.push_back(d);
    }
    // ---------------- Fun ----------------
    {
        PartDef d = mk("hyperdrive", "Hyperspace Drive HX-1", Cat::Fun, PK::Hyperdrive);
        d.diam = 2.5; d.length = 2.2; d.dryMass = 3000;
        d.desc = "Fictional. Opens a hyperspace corridor to any body. Requires 'Hyperspace' in settings. Key: Y";
        d.color = {90, 60, 160, 255};
        defs.push_back(d);
    }
    // ---------------- Aircraft (CAD designer): rotors, propellers, jets, wheels ----------------
    // rotors and propellers: electric, any atmosphere (thrust from momentum theory with the local air
    // density); diameter = rotor disc, powerUse = shaft power at full throttle
    struct Rot { const char* id; const char* name; double D, P, mass, hubL; bool coax; const char* desc; };
    for (Rot r : {Rot{"rotor_coax", "Coaxial Rotor CR-10", 10.0, 420e3, 340, 1.0, true,
                      "Two counter-rotating 10 m rotors on one mast (no tail rotor needed). Electric, 420 kW. Throttle = collective."},
                  Rot{"rotor_mars", "Mars Rotor MR-1.2", 1.21, 350, 0.55, 0.25, true,
                      "Coaxial 1.2 m carbon rotors at 2,500 rpm for the thin Martian air (Ingenuity class). 350 W."},
                  Rot{"rotor_titan", "Titan Rotor TR-1.4", 1.35, 1500, 9, 0.3, true,
                      "Coaxial 1.35 m rotor for Titan's dense, cold air (Dragonfly class). 1.5 kW."},
                  Rot{"prop_e", "Electric Propeller EP-2.6", 2.6, 260e3, 140, 1.2, false,
                      "2.6 m variable-pitch propeller with a 260 kW motor. Pulls along its axis."}}) {
        PartDef d = mk(r.id, r.name, Cat::Aircraft, PK::Rotor);
        d.diam = r.D;
        d.length = r.hubL;
        d.powerUse = r.P;
        d.dryMass = r.mass;
        d.canopies = r.coax ? 2 : 1; // blade sets
        d.desc = r.desc;
        d.color = {58, 60, 66, 255};
        defs.push_back(d);
    }
    {
        // turbofan: needs oxygen in the air (Earth); thrust falls with the air density
        PartDef d = mk("jet_tf27", "Turbofan TF-27", Cat::Aircraft, PK::Jet);
        d.prop = Prop::JetFuel;
        d.thrustVac = 27e3; // static sea-level thrust [N]
        d.ispVac = 5500;    // fuel-specific impulse (TSFC 0.065 kg/(N h))
        d.ispSL = 5500;
        d.dryMass = 520;
        d.diam = 0.95;
        d.length = 2.9;
        d.desc = "27 kN turbofan. Breathes the air: works only where the air holds oxygen (Earth). Burns jet fuel.";
        d.color = {150, 154, 160, 255};
        defs.push_back(d);
    }
    // ---- more air-breathing engines (figures after real engines; names are the game's own)
    {
        // after the GE F110-GE-129 (F-16, F-15K): 76.3 kN dry, 131 kN with afterburner, 1,800 kg,
        // SFC 0.067 / 0.186 kg/(N h)
        PartDef d = mk("jet_tf131ab", "Afterburning Turbofan TF-131", Cat::Aircraft, PK::Jet);
        d.prop = Prop::JetFuel;
        d.cycle = JetCycle::Afterburner;
        d.thrustVac = 131e3;
        d.dryThrust = 76.3e3;
        d.ispVac = d.ispSL = 5480; // dry
        d.ispWet = 1970;
        d.machMax = 2.2;
        d.ramK = 0.30;
        d.dryMass = 1800;
        d.diam = 1.18;
        d.length = 4.6;
        d.desc = "Fighter engine: 76 kN dry, 131 kN with the afterburner (throttle above 85 %, three times the fuel flow). Up to Mach 2.2. Needs oxygen in the air (Earth).";
        d.color = {120, 122, 128, 255};
        defs.push_back(d);
    }
    {
        // after the Pratt & Whitney J58 (SR-71): 111 kN dry, 151 kN wet, 2,700 kg, 1.45 x 5.7 m; bleed bypass
        // makes it a ramjet at high Mach - cruise at Mach 3.2 at 24 km
        PartDef d = mk("jet_trj151", "Turboramjet TRJ-151", Cat::Aircraft, PK::Jet);
        d.prop = Prop::JetFuel;
        d.cycle = JetCycle::Turboramjet;
        d.thrustVac = 151e3;
        d.dryThrust = 111e3;
        d.ispVac = d.ispSL = 3890;
        d.ispWet = 1720;
        d.machMax = 3.4;
        d.ramK = 0.42;
        d.lapse = 0.7;
        d.dryMass = 2700;
        d.diam = 1.45;
        d.length = 5.7;
        d.desc = "Turbojet that turns into a ramjet at high speed: 151 kN with the afterburner, cruise at Mach 3.2 near 24 km, limit Mach 3.4. Needs oxygen in the air (Earth).";
        d.color = {74, 70, 66, 255};
        defs.push_back(d);
    }
    {
        // supersonic-combustion ramjet after the X-43A / X-51A: starts above Mach 4, best Mach 5.5-8.5;
        // no static thrust (needs a booster or another engine to get there)
        PartDef d = mk("jet_sj60", "Scramjet SJ-60", Cat::Aircraft, PK::Jet);
        d.prop = Prop::JetFuel;
        d.cycle = JetCycle::Scramjet;
        d.thrustVac = 60e3; // at 50 kPa dynamic pressure, Mach 6
        d.ispVac = d.ispSL = 1200;
        d.machMax = 10;
        d.dryMass = 900;
        d.diam = 1.2;
        d.length = 4.0;
        d.desc = "Supersonic-combustion ramjet: no thrust below Mach 4, 60 kN at Mach 6 and 50 kPa dynamic pressure, works to about Mach 10. Needs oxygen in the air (Earth).";
        d.color = {96, 90, 84, 255};
        defs.push_back(d);
    }
    {
        // precooled combined cycle after the SABRE 4 of the Skylon spaceplane: air-breathing on liquid
        // hydrogen to about Mach 5.4 / 28 km (Isp 3,600 s), then a closed-cycle rocket on LOX / LH2 (Isp 460 s)
        // that works at 100 km and in orbit; 2,000 kN, thrust-to-weight ~14
        PartDef d = mk("jet_pce2000", "Combined-Cycle Engine PCE-2000", Cat::Aircraft, PK::Jet);
        d.prop = Prop::LH2; // air-breathing mode
        d.cycle = JetCycle::CombinedCycle;
        d.thrustVac = 1960e3;
        d.ispVac = d.ispSL = 3600;
        d.machMax = 5.4;
        d.ramK = 0.12;
        d.lapse = 0.5;
        d.rocketThrust = 2000e3;
        d.rocketIspVac = 460;
        d.rocketIspSL = 360;
        d.rocketProp = Prop::Hydrolox;
        d.minThrottle = 0.3;
        d.dryMass = 14000;
        d.diam = 2.8;
        d.length = 12.0;
        d.desc = "Precooled air-breathing rocket: on liquid hydrogen (LH2 tank) it breathes the air up to Mach 5.4 / about 28 km, then switches by itself to a closed-cycle rocket on hydrolox that works at 100 km and in space. 2,000 kN.";
        d.color = {180, 176, 168, 255};
        defs.push_back(d);
    }
    {
        // rocket engine for rocket planes after the XLR99 of the X-15 (107 km in 1963): 220 kN at sea level,
        // 257 kN in the vacuum, Isp 239 / 279 s, throttle 30-100 %, 415 kg (the real one burned LOX / ammonia)
        PartDef d = engine("eng_rp257", "Rocket-Plane Engine RP-257", Prop::Kerolox, 257, 239, 279, 415, 0, 1.0, 2.08, 0.30,
                           "Throttleable rocket engine for rocket planes (X-15 class): reaches 100 km and beyond. Steer with the control surfaces and RCS.");
        d.cat = Cat::Aircraft;
        d.diamAdj = false;
        d.clusterAdj = false;
        d.diam = 1.0;
        defs.push_back(d);
    }
    {
        PartDef d = mk("tank_jet", "Jet Fuel Tank", Cat::Aircraft, PK::Tank);
        d.prop = Prop::JetFuel;
        d.diam = 1.2;
        d.length = 3.0;
        d.diamAdj = d.lenAdj = true;
        d.desc = "Kerosene for turbofans.";
        d.color = propColor(Prop::JetFuel);
        defs.push_back(d);
    }
    {
        PartDef d = mk("wheel", "Landing Gear LG-1", Cat::Aircraft, PK::Wheel);
        d.diam = 0.7; // tyre
        d.length = 1.4;
        d.dryMass = 75;
        d.desc = "Retractable strut with a braked wheel (G / the gear button folds it up in flight). Rolls on the ground: take-off and landing runs. Brakes when the throttle is 0.";
        d.color = {40, 42, 46, 255};
        defs.push_back(d);
    }
    {
        // Apollo Lunar Roving Vehicle wheel: 81 cm wire-mesh tyre, a 190 W (0.25 hp) brushed DC motor with an
        // 80:1 harmonic drive in the hub, about 13 km/h at the most
        PartDef d = mk("wheel_rover", "Rover Wheel RW-81 (driven)", Cat::Aircraft, PK::Wheel);
        d.diam = 0.81;
        d.length = 0.5;
        d.dryMass = 12;
        d.powerUse = 190;
        d.maxSpeed = 3.6;
        d.desc = "Wire-mesh rover wheel with its own electric drive motor (as on the Apollo Lunar Roving Vehicle). W / S drive, A / D steer; power from the batteries.";
        d.color = {150, 152, 156, 255};
        defs.push_back(d);
    }
    {
        // Mars rover wheel (Curiosity class): 50 cm machined aluminium wheel on a rocker-bogie, slow and sure
        PartDef d = mk("wheel_mars", "Rover Wheel MW-50 (driven)", Cat::Aircraft, PK::Wheel);
        d.diam = 0.5;
        d.length = 0.55;
        d.dryMass = 20;
        d.powerUse = 25;
        d.maxSpeed = 0.04; // 4 cm/s, as Curiosity / Perseverance on flat ground
        d.desc = "Aluminium rover wheel with a geared drive motor (Curiosity class): 4 cm/s at the most. W / S drive, A / D steer.";
        d.color = {170, 172, 176, 255};
        defs.push_back(d);
    }
}

int PartCatalog::find(const std::string& id) const {
    for (int i = 0; i < (int)defs.size(); i++)
        if (defs[i].id == id) return i;
    return -1;
}

PartInst makeInst(int def) {
    const PartDef& d = CATALOG.defs[def];
    PartInst p;
    p.def = def;
    p.diam = d.diam;
    p.diam2 = d.diam;
    p.length = d.length;
    p.cluster = 1;
    return p;
}

PartGeom partGeom(const PartInst& p) {
    const PartDef& d = CATALOG.defs[p.def];
    PartGeom g;
    double r = p.diam * 0.5;
    switch (d.kind) {
    case PK::Capsule: g.height = d.length; g.rTop = d.diam * 0.18; g.rBot = d.diam * 0.5; break;
    case PK::Tank: g.height = p.length; g.rTop = g.rBot = r; break;
    case PK::Engine: {
        g.height = d.length + 0.35;
        g.rTop = r;
        g.rBot = r;
        break;
    }
    case PK::Decoupler: g.height = 0.3; g.rTop = g.rBot = r; break;
    case PK::NoseCone: g.height = 1.5 * p.diam; g.rTop = 0; g.rBot = r; break;
    case PK::Fairing: g.height = fairingNoseLength(p.diam); g.rTop = 0; g.rBot = r; break;
    case PK::Adapter: g.height = 0.5 + 0.7 * std::fabs(p.diam - p.diam2); g.rTop = r; g.rBot = p.diam2 * 0.5; break;
    case PK::HeatShield: g.height = 0.15 + 0.08 * p.diam; g.rTop = g.rBot = r; break;
    case PK::Probe:
        g.height = d.length;
        g.rTop = g.rBot = r;
        break;
    default: g.height = d.length; g.rTop = g.rBot = d.diam * 0.5; break;
    }
    if (!d.diamAdj && d.kind != PK::Capsule && d.kind != PK::NoseCone) { g.rTop = g.rBot = d.diam * 0.5; }
    return g;
}

bool worksWithoutAir(const PartDef& d) {
    return d.kind == PK::Engine || d.kind == PK::SRB || (d.kind == PK::Jet && d.cycle == JetCycle::CombinedCycle);
}

double tankCapacity(const PartInst& p) {
    const PartDef& d = CATALOG.defs[p.def];
    if (d.kind == PK::SRB) return d.srbProp;
    if (d.kind != PK::Tank) return 0;
    double r = p.diam * 0.5;
    double vol = PI * r * r * p.length * 0.92;
    return vol * propDensity(d.prop);
}

double partDryMass(const PartInst& p) {
    const PartDef& d = CATALOG.defs[p.def];
    double m = 0;
    double D = p.diam;
    switch (d.kind) {
    case PK::Tank: m = tankCapacity(p) * tankDryFraction(d.prop); break;
    case PK::Engine: m = d.dryMass * p.cluster + 25 * D * D; break;
    case PK::Decoupler: m = 10 + 28 * D * D; break;
    case PK::NoseCone: m = 35 * D * D; break;
    case PK::Fairing: {
        // carbon / aluminium-honeycomb shell ~8 kg/m^2 plus the separation system (Falcon 9: 1.9 t for 5.2 m x 13 m)
        double R = D * 0.5, Ln = fairingNoseLength(D);
        m = 8.0 * (PI * D * std::max(p.length, 0.0) + PI * R * std::sqrt(R * R + Ln * Ln) * 1.1) + 25 * D;
        break;
    }
    case PK::Adapter: m = 30 * (D * D + p.diam2 * p.diam2) * 0.5 + 10; break;
    case PK::HeatShield: m = 150 * PI * D * D * 0.25; break;
    case PK::Probe: m = d.diamAdj ? d.dryMass * (D * D) / (2.5 * 2.5) + 40 : d.dryMass; break;
    default: m = d.dryMass; break;
    }
    for (const auto& ra : p.radial) {
        const PartDef& rd = CATALOG.defs[ra.def];
        m += rd.dryMass * ra.count;
        if (rd.kind == PK::SRB) m += 40.0 * ra.count; // attach struts and separation thrusters
    }
    return m;
}

double instThrustVac(const PartInst& p) {
    const PartDef& d = CATALOG.defs[p.def];
    if (d.kind == PK::Engine) return d.thrustVac * p.cluster;
    if (d.kind == PK::SRB) return d.thrustVac;
    return 0;
}
double instPowerUse(const PartInst& p) {
    const PartDef& d = CATALOG.defs[p.def];
    return d.kind == PK::Engine ? d.powerUse * p.cluster : 0;
}

bool sideMountable(const PartDef& d) { return d.radial || d.kind == PK::SRB; }
double sideBoosterDistance(double rParent, const PartDef& b) { return rParent + b.diam * 0.5 + 0.08; }

double stageBottomZ(const Design& d, int i, const std::vector<double>& zb) {
    int segN = 0;
    auto seg = segmentOfParts(d, segN);
    int n = (int)d.stack.size(), last = i;
    for (int j = i + 1; j < n && seg[j] == seg[i]; j++) {
        if (CATALOG.defs[d.stack[j].def].kind == PK::Decoupler) break; // the stage ends above its decoupler
        last = j;
    }
    return zb[last];
}

std::vector<int> segmentOfParts(const Design& d, int& segCount) {
    int n = (int)d.stack.size();
    std::vector<int> seg(n, 0);
    int s = 0;
    for (int i = n - 1; i >= 0; i--) {
        seg[i] = s;
        if (CATALOG.defs[d.stack[i].def].kind == PK::Decoupler && i > 0) s++;
    }
    segCount = n ? s + 1 : 0;
    return seg;
}

double fairingNoseLength(double diam) { return 1.25 * std::max(diam, 0.2); }

std::vector<double> stackBottoms(const Design& d) {
    int n = (int)d.stack.size();
    std::vector<double> zb(n);
    double z = 0;
    for (int i = n - 1; i >= 0; i--) {
        zb[i] = z;
        z += partGeom(d.stack[i]).height;
    }
    return zb;
}

bool coveredByFairing(const Design& d, int idx) {
    if (idx <= 0 || idx >= (int)d.stack.size()) return false;
    auto zb = stackBottoms(d);
    for (int f = 0; f < idx; f++) {
        const PartInst& p = d.stack[f];
        if (CATALOG.defs[p.def].kind != PK::Fairing) continue;
        if (zb[idx] >= zb[f] - p.length - 1e-6) return true;
    }
    return false;
}

Interstage interstageFor(const Design& d, int i) {
    Interstage s;
    int n = (int)d.stack.size();
    if (i <= 0 || i >= n || CATALOG.defs[d.stack[i].def].kind != PK::Decoupler) return s;
    if (coveredByFairing(d, i - 1)) return s; // the fairing already encloses the engine above
    const PartInst& eng = d.stack[i - 1];
    PK k = CATALOG.defs[eng.def].kind;
    if (k != PK::Engine) return s; // solid boosters are their own casing
    PartGeom dg = partGeom(d.stack[i]), eg = partGeom(eng);
    double rEng = std::max(eg.rTop, eg.rBot);
    // the part the engine is mounted to sets the upper diameter
    double rAbove = i >= 2 ? partGeom(d.stack[i - 2]).rBot : dg.rTop;
    s.rBot = std::max(dg.rTop, rEng * 1.04);
    s.rTop = std::max(rAbove > 0 ? rAbove : dg.rTop, rEng * 1.04);
    s.height = eg.height;
    if (s.height <= 0.05 || s.rBot <= 0.05) return s;
    double D = (s.rBot + s.rTop);
    s.wall = std::clamp(0.0015 + 0.0008 * D, 0.0015, 0.008);
    double slant = std::sqrt(s.height * s.height + (s.rBot - s.rTop) * (s.rBot - s.rTop));
    double area = 3.141592653589793 * (s.rBot + s.rTop) * slant;
    s.mass = area * s.wall * 2700.0 * 1.3; // Al-Li shell + ring frames
    s.on = true;
    return s;
}

DesignStats analyzeDesign(const Design& d) {
    DesignStats st;
    int segN = 0;
    auto seg = segmentOfParts(d, segN);
    int n = (int)d.stack.size();
    std::vector<double> segDry(segN, 0), segWet(segN, 0);
    for (int i = 0; i < n; i++) {
        const PartInst& p = d.stack[i];
        const PartDef& def = CATALOG.defs[p.def];
        double dry = partDryMass(p);
        double cap = tankCapacity(p);
        for (const auto& ra : p.radial)
            if (CATALOG.defs[ra.def].kind == PK::SRB) cap += CATALOG.defs[ra.def].srbProp * ra.count;
        if (def.kind == PK::Decoupler) dry += interstageFor(d, i).mass;
        segDry[seg[i]] += dry;
        segWet[seg[i]] += dry + cap;
        st.dryMass += dry;
        st.wetMass += dry + cap;
        PartGeom g = partGeom(p);
        st.height += g.height;
        st.maxDiam = std::max(st.maxDiam, std::max(g.rTop, g.rBot) * 2);
        st.batteryJ += def.batteryJ;
        st.genW += def.genW;
        st.drawW += def.drawW;
        st.torque += def.torque;
        st.crew += def.crew;
        if (def.kind == PK::Capsule || def.kind == PK::Probe) st.hasControl = true;
        if (def.kind == PK::Hyperdrive) st.hasHyperdrive = true;
        if (def.kind == PK::Sail) st.hasSail = true;
        if (def.kind == PK::Parachute) st.hasChute = true;
        if (def.kind == PK::Fairing) {
            // the skirt must reach past the payload: warn about parts that stick out of it or below it
            auto zb = stackBottoms(d);
            double skirtBot = zb[i] - p.length, R = p.diam * 0.5;
            for (int j = i + 1; j < n; j++) {
                PartGeom gj = partGeom(d.stack[j]);
                if (zb[j] + gj.height <= skirtBot + 1e-6) break;
                if (CATALOG.defs[d.stack[j].def].kind == PK::Decoupler && zb[j] < skirtBot - 1e-6) {
                    st.warnings.push_back("Fairing is shorter than the payload above the decoupler");
                    break;
                }
                if (std::max(gj.rTop, gj.rBot) > R + 1e-3) {
                    st.warnings.push_back("Fairing diameter is smaller than the payload inside it");
                    break;
                }
            }
        }
        for (const auto& ra : p.radial) {
            const PartDef& rd = CATALOG.defs[ra.def];
            st.batteryJ += rd.batteryJ * ra.count;
            st.genW += rd.genW * ra.count;
            st.solarW1AU += rd.solarArea * rd.solarEff * 1361.0 * ra.count;
            if (rd.kind == PK::Legs) st.hasLegs = true;
        }
    }
    // Burn simulation, stage by stage (bottom first)
    double massAbove = st.wetMass;
    for (int s = 0; s < segN; s++) {
        StageStats ss;
        ss.massStart = massAbove;
        double tank[PROP_N] = {0};
        struct Eng { Prop p; double F, mdot, ispSL, ispVac; double srbLeft; double drop; };
        std::vector<Eng> engs;
        bool hasTank[PROP_N] = {false};
        for (int i = 0; i < n; i++) {
            if (seg[i] != s) continue;
            const PartInst& p = d.stack[i];
            const PartDef& def = CATALOG.defs[p.def];
            if (def.kind == PK::Tank) { tank[(int)def.prop] += tankCapacity(p); hasTank[(int)def.prop] = true; }
            if (def.kind == PK::Engine || def.kind == PK::SRB) {
                double F = instThrustVac(p);
                Eng e{def.prop, F, F / (def.ispVac * G0), def.ispSL, def.ispVac, def.kind == PK::SRB ? def.srbProp : -1, 0};
                engs.push_back(e);
                ss.hasEngines = true;
                ss.thrustVac += F;
            }
            for (const auto& ra : p.radial) {
                const PartDef& rd = CATALOG.defs[ra.def];
                if (rd.kind != PK::SRB) continue;
                // side boosters: lit with the stage, dropped (casing and struts) once burnt out
                double F = rd.thrustVac * ra.count;
                engs.push_back({Prop::Solid, F, F / (rd.ispVac * G0), rd.ispSL, rd.ispVac, rd.srbProp * ra.count, (rd.dryMass + 40.0) * ra.count});
                ss.hasEngines = true;
                ss.thrustVac += F;
            }
        }
        for (auto& e : engs)
            if (e.srbLeft < 0 && !hasTank[(int)e.p]) {
                st.warnings.push_back(trf("Stage {0}: no {1} tank for its engine", std::to_string(s + 1), tr(propName(e.p))));
            }
        // TWR at Earth sea level
        double FSL = 0;
        for (auto& e : engs) FSL += e.F * std::max(0.0, e.ispSL) / e.ispVac;
        ss.twrSL = FSL / (ss.massStart * G0);
        // phased burn
        double m = massAbove;
        std::vector<bool> on(engs.size(), true);
        for (int iter = 0; iter < 32; iter++) {
            double mdotProp[PROP_N] = {0};
            double Ftot = 0, mdotTot = 0, FslTot = 0;
            for (size_t k = 0; k < engs.size(); k++) {
                if (!on[k]) continue;
                auto& e = engs[k];
                if (e.srbLeft < 0) {
                    if (tank[(int)e.p] <= 1e-9) { on[k] = false; continue; }
                    mdotProp[(int)e.p] += e.mdot;
                }
                Ftot += e.F;
                FslTot += e.F * std::max(0.0, e.ispSL) / e.ispVac;
                mdotTot += e.mdot;
            }
            if (mdotTot <= 0) break;
            double tphase = 1e300;
            for (int p = 0; p < PROP_N; p++)
                if (mdotProp[p] > 0) tphase = std::min(tphase, tank[p] / mdotProp[p]);
            for (size_t k = 0; k < engs.size(); k++)
                if (on[k] && engs[k].srbLeft >= 0) tphase = std::min(tphase, engs[k].srbLeft / engs[k].mdot);
            if (!(tphase < 1e300) || tphase <= 0) break;
            double dm = mdotTot * tphase;
            double m1 = m - dm;
            if (m1 <= 0) break;
            double ispEff = Ftot / (mdotTot * G0);
            double ispEffSL = FslTot / (mdotTot * G0);
            ss.dvVac += ispEff * G0 * std::log(m / m1);
            ss.dvSL += ispEffSL * G0 * std::log(m / m1);
            ss.burnTime += tphase;
            for (int p = 0; p < PROP_N; p++) tank[p] -= mdotProp[p] * tphase;
            for (size_t k = 0; k < engs.size(); k++) {
                if (!on[k]) continue;
                if (engs[k].srbLeft >= 0) {
                    engs[k].srbLeft -= engs[k].mdot * tphase;
                    if (engs[k].srbLeft <= 1e-6) {
                        on[k] = false;
                        m1 -= engs[k].drop; // side boosters separate
                    }
                }
            }
            m = m1;
        }
        ss.massEnd = m;
        st.totalDvVac += ss.dvVac;
        st.stages.push_back(ss);
        massAbove -= segWet[s];
    }
    if (!st.hasControl) st.warnings.push_back("No command capsule or probe core: vessel cannot be controlled");
    double elecDemand = 0;
    for (const auto& p : d.stack) elecDemand += instPowerUse(p);
    if (elecDemand > 0 && st.genW + st.solarW1AU < elecDemand * 0.1)
        st.warnings.push_back("Electric thrusters need much more power generation");
    if (!st.stages.empty() && st.stages[0].hasEngines && st.stages[0].twrSL < 1.0)
        st.warnings.push_back("First stage TWR < 1: cannot lift off from Earth");
    if (!d.stack.empty() && CATALOG.defs[d.stack.back().def].kind == PK::Decoupler)
        st.warnings.push_back("Decoupler at the bottom does nothing");
    return st;
}

// ---------------- serialization ----------------
// (whole UTF-8 characters: a Japanese name is kept)
static std::string sanitizeName(const std::string& s) { return cleanLineText(s, 48); }

std::string designToText(const Design& d) {
    std::ostringstream os;
    os.precision(17); // (every value reads back as the same double)
    os << brand::tag("DESIGN") << " 1\n";
    os << "name " << sanitizeName(d.name) << "\n";
    for (const auto& p : d.stack) {
        os << "part " << CATALOG.defs[p.def].id << " " << p.diam << " " << p.diam2 << " " << p.length << " " << p.cluster << "\n";
        for (const auto& r : p.radial)
            os << "radial " << CATALOG.defs[r.def].id << " " << r.count << " " << r.offset << " " << r.angle << "\n";
    }
    os << "end\n";
    return os.str();
}

bool designFromText(Design& d, const std::string& text) {
    std::istringstream is(text);
    std::string line;
    if (!std::getline(is, line) || !brand::hasTag(line, "DESIGN")) return false;
    Design out;
    while (std::getline(is, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::istringstream ls(line);
        std::string key;
        ls >> key;
        if (key == "name") {
            std::string rest;
            std::getline(ls, rest);
            if (!rest.empty() && rest[0] == ' ') rest.erase(0, 1);
            out.name = sanitizeName(rest);
        } else if (key == "part") {
            std::string id;
            PartInst p;
            ls >> id >> p.diam >> p.diam2 >> p.length >> p.cluster;
            int def = CATALOG.find(id);
            if (def < 0 || ls.fail()) continue;
            p.def = def;
            p.diam = clampd(p.diam, 0.3, 12.0);
            p.diam2 = clampd(p.diam2, 0.3, 12.0);
            p.length = clampd(p.length, 0.25, 80.0);
            p.cluster = std::clamp(p.cluster, 1, 37);
            if (CATALOG.defs[def].radial) continue;
            if (out.stack.size() >= 64) break;
            out.stack.push_back(p);
        } else if (key == "radial") {
            std::string id;
            RadialAttach r;
            ls >> id >> r.count;
            int def = CATALOG.find(id);
            if (def < 0 || ls.fail() || out.stack.empty() || !sideMountable(CATALOG.defs[def])) continue;
            double off = 0, ang = 0;
            if (ls >> off >> ang) {
                if (std::isfinite(off)) r.offset = clampd(off, -60.0, 120.0);
                if (std::isfinite(ang)) r.angle = std::fmod(clampd(ang, -720.0, 720.0) + 720.0, 360.0);
            }
            r.def = def;
            r.count = std::clamp(r.count, 1, 12);
            if (out.stack.back().radial.size() < 16) out.stack.back().radial.push_back(r);
        } else if (key == "end") break;
    }
    d = out;
    return true;
}

std::string partDescription(const PartDef& d) {
    char a[64], b[64];
    if (d.kind == PK::Solar) {
        std::snprintf(a, sizeof a, "%.0f", d.solarArea);
        std::snprintf(b, sizeof b, "%.1f", d.solarArea * d.solarEff * 1361 / 1000);
        return trf("{0} m² triple-junction cells, 30% efficiency. {1} kW at 1 AU. Falls off with 1/r².", a, b);
    }
    if (d.kind == PK::RCS) {
        std::snprintf(a, sizeof a, "%.0f", d.thrustVac);
        std::snprintf(b, sizeof b, "%.0f", d.ispVac);
        return trf("Four {0} N hydrazine thrusters (Isp {1} s) for attitude control and translation. Uses monopropellant from any tank, or battery power in electric mode (10% thrust, Isp 800 s). Key: R", a, b);
    }
    if (d.kind == PK::Sail) {
        std::snprintf(a, sizeof a, "%.0f", d.sailArea);
        std::snprintf(b, sizeof b, "%.3g", 2 * 1361.0 / C_LIGHT * d.sailArea);
        return trf("{0} m² reflective membrane. {1} N at 1 AU when facing the star. Propellant-free. Key: B", a, b);
    }
    return tr(d.desc);
}
