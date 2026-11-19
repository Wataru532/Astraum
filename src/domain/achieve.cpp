#include "achieve.hpp"
#include "i18n.hpp"
#include "sim.hpp"
#include <algorithm>
#include <cmath>

using i18n::tr;
using i18n::trf;

namespace ach {

bool solidBody(const Universe& U, int body) {
    if (body < 0 || body >= U.count()) return false;
    const Body& b = U.bodies[body];
    return !b.isStar && !b.gasGiant;
}

static bool inSolarSystem(const Universe& U, int body) {
    const Body& b = U.bodies[body];
    return !b.isStar && b.system == "Solar System";
}

const std::vector<Def>& list(const Universe& U) {
    static std::vector<Def> L;
    static int builtFor = -1;
    if (builtFor == U.count() && !L.empty()) return L;
    L.clear();
    builtFor = U.count();
    int earth = U.find("Earth");
    // general firsts (Earth and beyond)
    L.push_back({"liftoff", Kind::Liftoff, earth});
    L.push_back({"space", Kind::Space, earth});
    L.push_back({"low:Earth", Kind::LowOrbit, earth});
    L.push_back({"high:Earth", Kind::HighOrbit, earth});
    L.push_back({"geo", Kind::Geostationary, earth});
    L.push_back({"return:space", Kind::SafeReturn, earth});
    L.push_back({"dock", Kind::Docking, earth});
    L.push_back({"mach1", Kind::Supersonic, earth});
    L.push_back({"mach5", Kind::Hypersonic, earth});
    L.push_back({"interplanetary", Kind::Interplanetary, earth});
    L.push_back({"sundiver", Kind::SunDiver, earth});
    L.push_back({"solarescape", Kind::SolarEscape, earth});
    // every other body of the Solar System: reach it, orbit it low and high, land, come back
    for (int i = 0; i < U.count(); i++) {
        if (i == earth || !inSolarSystem(U, i)) continue;
        const std::string& n = U.bodies[i].name;
        L.push_back({"visit:" + n, Kind::Visit, i});
        L.push_back({"low:" + n, Kind::LowOrbit, i});
        if (orbitBands(U.bodies[i]).lowMax < U.bodies[i].soi - U.bodies[i].radius * 1.02) // (room for a high orbit)
            L.push_back({"high:" + n, Kind::HighOrbit, i});
        if (solidBody(U, i)) L.push_back({"land:" + n, Kind::Landing, i});
        L.push_back({"return:" + n, Kind::ReturnFrom, i});
    }
    return L;
}

const Def* find(const Universe& U, const std::string& id) {
    for (const Def& d : list(U))
        if (d.id == id) return &d;
    return nullptr;
}

std::string title(const Universe& U, const Def& d) {
    std::string bn = tr(U.bodies[d.body].name);
    switch (d.kind) {
    case Kind::Liftoff: return tr("Liftoff");
    case Kind::Space: return tr("Into space");
    case Kind::Supersonic: return tr("Sound barrier");
    case Kind::Hypersonic: return tr("Hypersonic");
    case Kind::SafeReturn: return tr("Safe return");
    case Kind::Docking: return tr("Docking");
    case Kind::Geostationary: return tr("Geostationary");
    case Kind::Interplanetary: return tr("Interplanetary");
    case Kind::SunDiver: return tr("Sun diver");
    case Kind::SolarEscape: return tr("Beyond the Sun");
    case Kind::Visit: return trf("Reach {0}", bn);
    case Kind::LowOrbit: return trf("Low orbit: {0}", bn);
    case Kind::HighOrbit: return trf("High orbit: {0}", bn);
    case Kind::Landing: return trf("Landing: {0}", bn);
    case Kind::ReturnFrom: return trf("Back from {0}", bn);
    default: return d.id;
    }
}

std::string description(const Universe& U, const Def& d) {
    const Body& b = U.bodies[d.body];
    std::string bn = tr(b.name);
    switch (d.kind) {
    case Kind::Liftoff: return tr("Climb 1 km above the ground of Earth.");
    case Kind::Space: return tr("Cross the Karman line (100 km).");
    case Kind::Supersonic: return tr("Fly faster than sound (Mach 1) on air-breathing engines.");
    case Kind::Hypersonic: return tr("Reach Mach 5 on air-breathing engines.");
    case Kind::SafeReturn: return tr("Land on Earth with a vessel that has been to space.");
    case Kind::Docking: return tr("Dock two vessels with each other.");
    case Kind::Geostationary: return tr("Reach a geostationary orbit (circular, equatorial, 35,786 km).");
    case Kind::Interplanetary: return tr("Leave Earth's sphere of influence into an orbit around the Sun.");
    case Kind::SunDiver: return tr("Come within 0.25 AU of the Sun.");
    case Kind::SolarEscape: return tr("Reach a trajectory that escapes the Solar System.");
    case Kind::Visit: return trf("Enter the sphere of influence of {0}.", bn);
    case Kind::LowOrbit: return trf("Reach a low orbit around {0} (up to {1}).", bn, formatDist(orbitBands(b).lowMax));
    case Kind::HighOrbit: return trf("Reach an orbit around {0} entirely above {1}.", bn, formatDist(orbitBands(b).lowMax));
    case Kind::Landing: return trf("Land safely on {0}.", bn);
    case Kind::ReturnFrom:
        return solidBody(U, d.body) ? trf("Land on {0}, then return and land on Earth with the same vessel.", bn)
                                    : trf("Orbit {0}, then return and land on Earth with the same vessel.", bn);
    default: return {};
    }
}

} // namespace ach

// ------------------------------------------------------------------ awarding (once per frame, every vessel)
bool World::checkAchievements() {
    if (!recordsOn) return false;
    bool any = false;
    int earth = U.find("Earth"), sun = U.find("Sun");
    auto award = [&](const std::string& id, const Vessel& v) {
        if (achievements.count(id) || !ach::find(U, id)) return;
        achievements[id] = {t, v.name};
        newAchievements.push_back(id);
        if (newAchievements.size() > 32) newAchievements.erase(newAchievements.begin());
        missionLog.push_back({t, trf("Achievement: {0}", ach::title(U, *ach::find(U, id))) + "  ·  " + v.name, {250, 204, 21, 255}});
        any = true;
    };
    for (Vessel& v : vessels) {
        if (v.destroyed) continue;
        // the fun modes taint whatever flies while they are on (for good)
        if (settings.infiniteFuel) v.cheated = true;
        if (!v.countsForRecords()) continue;
        const Body& b = U.bodies[v.ref];
        double alt = len(v.r) - b.radius;
        // what this vessel has done (kept through docking and undocking)
        if (v.ref != earth || alt > 100e3) v.reachedSpace = true;
        Conic c;
        OrbitClass oc = orbitClassOf(*this, v, c);
        bool orbiting = oc == OrbitClass::Low || oc == OrbitClass::High || oc == OrbitClass::Synchronous || oc == OrbitClass::Spanning;
        if (v.ref < 64) {
            if (v.landed) v.landedOn |= 1ull << v.ref;
            if (orbiting) v.orbited |= 1ull << v.ref;
        }
        bool solar = b.system == "Solar System" && !b.isStar;
        // general
        if (v.ref == earth && !v.landed && alt > 1000) award("liftoff", v);
        if (v.reachedSpace) award("space", v);
        if (v.ref == earth && v.landed && v.reachedSpace) award("return:space", v);
        if (v.ref == earth && b.atmTop > 0 && alt < b.atmTop && !v.landed) {
            bool air = false;
            for (uint8_t m : v.jetMode) air |= m == 1 || m == 2;
            if (air && v.tMach >= 1.0) award("mach1", v);
            if (air && v.tMach >= 5.0) award("mach5", v);
        }
        if (v.ref == earth && oc == OrbitClass::Synchronous) {
            Vec3d spin = norm(U.angVel(earth));
            double incEq = std::acos(std::clamp(std::fabs(dot(c.H, spin)), 0.0, 1.0));
            if (incEq < 1.0 * DEG && c.e < 0.01) award("geo", v);
        }
        if (v.ref == sun) {
            award("interplanetary", v);
            if (len(v.r) < 0.25 * 1.495978707e11) award("sundiver", v);
            Conic sc = stateToConic(v.r, v.v, b.GM);
            if (sc.valid && (sc.e >= 1 || sc.energy >= 0)) award("solarescape", v);
        }
        // the body it is at
        if (solar) {
            const std::string& n = b.name;
            if (v.ref != earth) award("visit:" + n, v);
            if (oc == OrbitClass::Low) award("low:" + n, v);
            if (oc == OrbitClass::High || oc == OrbitClass::Synchronous) award("high:" + n, v);
            if (v.landed && v.ref != earth && ach::solidBody(U, v.ref)) award("land:" + n, v);
        }
        // home again: every body it landed on (or, for gas giants, orbited) before
        if (v.ref == earth && v.landed) {
            for (int i = 0; i < U.count() && i < 64; i++) {
                if (i == earth || U.bodies[i].isStar || U.bodies[i].system != "Solar System") continue;
                uint64_t bit = 1ull << i;
                if (ach::solidBody(U, i) ? (v.landedOn & bit) : (v.orbited & bit)) award("return:" + U.bodies[i].name, v);
            }
        }
    }
    return any;
}

void World::awardDocking(const Vessel& a, const Vessel& b) {
    if (!recordsOn || !a.countsForRecords() || !b.countsForRecords() || achievements.count("dock")) return;
    achievements["dock"] = {t, a.name};
    newAchievements.push_back("dock");
    missionLog.push_back({t, trf("Achievement: {0}", tr("Docking")) + "  ·  " + a.name, {250, 204, 21, 255}});
}
