// Achievements of a map (saved game): firsts that only count for vessels launched from the surface of Earth
// and flown without the fun modes (infinite fuel, hyperspace). World::checkAchievements awards them.
#pragma once
#include <string>
#include <vector>

struct Universe;

namespace ach {

enum class Kind {
    Liftoff, Space, Supersonic, Hypersonic, SafeReturn, Docking, Geostationary, Interplanetary, SunDiver, SolarEscape,
    Visit, LowOrbit, HighOrbit, Landing, ReturnFrom, COUNT
};

struct Def {
    std::string id; // stable key in the save file ("land:Moon")
    Kind kind;
    int body;       // the body it is about (Earth for the general ones)
};

// every achievement, in display order: the general ones, then per body of the Solar System
const std::vector<Def>& list(const Universe& U);
const Def* find(const Universe& U, const std::string& id);
std::string title(const Universe& U, const Def& d);       // translated
std::string description(const Universe& U, const Def& d); // translated
bool solidBody(const Universe& U, int body);               // has a surface to land on

} // namespace ach
