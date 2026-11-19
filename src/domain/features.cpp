#include "features.hpp"

namespace features {
namespace {
// clang-format off
const Feature F[] = {
    // ---- Earth
    {"Earth", "Mount Everest",          27.99,   86.93,   30, Mons, 0},
    {"Earth", "Himalayas",              28.60,   83.90,  800, Range, 0},
    {"Earth", "Tibetan Plateau",        33.00,   88.00,  900, Plateau, 0},
    {"Earth", "Andes",                 -20.00,  -67.50, 2000, Range, 0},
    {"Earth", "Aconcagua",             -32.65,  -70.01,   30, Mons, 0},
    {"Earth", "Rocky Mountains",        43.00, -110.00, 1500, Range, 0},
    {"Earth", "Alps",                   46.30,    9.50,  400, Range, 0},
    {"Earth", "Mont Blanc",             45.83,    6.86,   20, Mons, 0},
    {"Earth", "Kilimanjaro",            -3.07,   37.35,   30, Mons, 0},
    {"Earth", "Mount Fuji",             35.36,  138.73,   20, Mons, 0},
    {"Earth", "Mauna Kea",              19.82, -155.47,   30, Mons, 0},
    {"Earth", "Denali",                 63.07, -151.01,   30, Mons, 0},
    {"Earth", "Grand Canyon",           36.10, -112.10,  150, Valley, 0},
    {"Earth", "East African Rift",       1.50,   36.50, 1500, Valley, 0},
    {"Earth", "Sahara",                 23.40,   12.00, 2500, Region, 0},
    {"Earth", "Amazon Basin",           -4.00,  -62.00, 1500, Region, 0},
    {"Earth", "Ural Mountains",         60.00,   59.50, 1000, Range, 0},
    {"Earth", "Greenland Ice Sheet",    72.00,  -40.00, 1000, Ice, 0},
    {"Earth", "Antarctic Ice Sheet",   -82.00,   45.00, 2500, Ice, 0},
    // ---- Moon
    {"Moon", "Mare Imbrium",            32.80,  -15.60,  573, Mare, 0},
    {"Moon", "Mare Serenitatis",        28.00,   17.50,  337, Mare, 0},
    {"Moon", "Mare Tranquillitatis",     8.50,   31.40,  437, Mare, 0},
    {"Moon", "Oceanus Procellarum",     18.40,  -57.40, 1300, Mare, 0},
    {"Moon", "Mare Crisium",            17.00,   59.10,  278, Mare, 0},
    {"Moon", "Mare Nubium",            -21.30,  -16.60,  357, Mare, 0},
    {"Moon", "Mare Fecunditatis",       -7.80,   51.30,  455, Mare, 0},
    {"Moon", "Mare Frigoris",           56.00,    1.40,  800, Mare, 0},
    {"Moon", "Mare Orientale",         -19.40,  -92.80,  465, Basin, 0},
    {"Moon", "South Pole-Aitken Basin", -53.00, -169.00, 1250, Basin, 0},
    {"Moon", "Montes Apenninus",        18.90,   -3.70,  300, Range, 0},
    {"Moon", "Tycho",                  -43.31,  -11.36,   43, Crater, 0},
    {"Moon", "Copernicus",               9.62,  -20.08,   47, Crater, 0},
    {"Moon", "Kepler",                   8.12,  -38.01,   16, Crater, 0},
    {"Moon", "Aristarchus",             23.73,  -47.49,   20, Crater, 0},
    {"Moon", "Plato",                   51.62,   -9.38,   51, Crater, 0},
    {"Moon", "Shackleton",             -89.67,  129.78,   10, Crater, 0},
    {"Moon", "Apollo 11 landing site",   0.674,  23.473,   2, Site, 0},
    {"Moon", "Apollo 15 landing site",  26.13,    3.63,    2, Site, 0},
    {"Moon", "Apollo 17 landing site",  20.19,   30.77,    2, Site, 0},
    // ---- Mars
    {"Mars", "Olympus Mons",            18.65, -133.80,  300, Mons, 0},
    {"Mars", "Valles Marineris",       -13.90,  -59.20, 2000, Valley, 0},
    {"Mars", "Hellas Planitia",        -42.40,   70.50, 1150, Basin, 0},
    {"Mars", "Argyre Planitia",        -49.70,  -43.40,  900, Basin, 0},
    {"Mars", "Arsia Mons",              -8.26, -120.09,  220, Mons, 0},
    {"Mars", "Pavonis Mons",             1.48, -112.96,  190, Mons, 0},
    {"Mars", "Ascraeus Mons",           11.92, -104.08,  230, Mons, 0},
    {"Mars", "Alba Mons",               40.47, -109.60,  500, Mons, 0},
    {"Mars", "Elysium Mons",            24.80,  146.90,  200, Mons, 0},
    {"Mars", "Utopia Planitia",         49.70,  118.00, 1650, Planitia, 0},
    {"Mars", "Syrtis Major Planum",      8.40,   69.50,  750, Plateau, 0},
    {"Mars", "Chryse Planitia",         28.40,  -40.00,  800, Planitia, 0},
    {"Mars", "Noctis Labyrinthus",      -6.90, -101.90,  595, Valley, 0},
    {"Mars", "Gale",                    -5.40,  137.80,   77, Crater, 0},
    {"Mars", "Jezero",                  18.38,   77.58,   25, Crater, 0},
    {"Mars", "Planum Boreum",           88.00,   15.00,  500, Ice, 0},
    {"Mars", "Planum Australe",        -84.00,  160.00,  500, Ice, 0},
    // ---- Venus (under the clouds: no relief is drawn there)
    {"Venus", "Maxwell Montes",         65.20,    3.30,  400, Range, 0},
    {"Venus", "Ishtar Terra",           70.40,   27.50, 2800, Region, 0},
    {"Venus", "Aphrodite Terra",        -5.80,  104.80, 5000, Region, 0},
    // ---- bodies with generated relief: carved
    {"Mercury", "Caloris Planitia",     30.50,  170.20,  775, Basin, 2000},
    {"Mercury", "Rachmaninoff",         27.60,   57.60,  145, Crater, 2000},
    {"Ceres", "Occator",                19.82, -120.67,   46, Crater, 4000},
    {"Ceres", "Kerwan",                -10.77,  123.99,  140, Crater, 2000},
    {"Ceres", "Ahuna Mons",            -10.48,  -43.80,   10, Mons, 4000},
    {"Phobos", "Stickney",               1.00,  -49.00,  4.5, Crater, 1000},
    {"Deimos", "Voltaire",              22.00,   -3.50, 0.95, Crater, 200},
    {"Io", "Loki Patera",               12.97,   51.20,  114, Crater, 500},
    {"Europa", "Pwyll",                -25.20,   88.60,   22, Crater, 600},
    {"Ganymede", "Gilgamesh",          -62.84, -124.83,   76, Crater, 1500},
    {"Callisto", "Valhalla",            14.70,  -56.00, 1500, Basin, 300},
    {"Titan", "Menrva",                 20.10,  -87.20,  196, Crater, 0},
    {"Titania", "Gertrude",            -15.80,  -72.90,  163, Crater, 2000},
    {"Pluto", "Sputnik Planitia",       19.51,  178.69,  746, Basin, 3000},
};
// clang-format on
} // namespace

const Feature* all(int& count) {
    count = (int)(sizeof F / sizeof F[0]);
    return F;
}

} // namespace features
