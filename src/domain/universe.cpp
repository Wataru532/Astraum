#include "universe.hpp"
#include <algorithm>
#include <set>

namespace {

// IAU-style body frame from north pole RA/Dec (ICRF, degrees) -> ecliptic
Mat3d poleFromRaDec(double raDeg, double decDeg) {
    double ra = raDeg * DEG, dec = decDeg * DEG;
    Vec3d p{std::cos(dec) * std::cos(ra), std::cos(dec) * std::sin(ra), std::sin(dec)};
    Vec3d Q{-std::sin(ra), std::cos(ra), 0};
    Vec3d Y = cross(p, Q);
    return Mat3d::fromCols(eqToEcl(Q), eqToEcl(Y), eqToEcl(p));
}

Mat3d orbitRot(double Om, double inc, double om) { return rotZ(Om) * rotX(inc) * rotZ(om); }

Vec3f rgb(double r, double g, double b) { return {(float)r, (float)g, (float)b}; }

struct Builder {
    Universe& U;
    int add(Body b) {
        U.bodies.push_back(b);
        return (int)U.bodies.size() - 1;
    }
    // Planet / dwarf planet from JPL-style heliocentric ecliptic elements at J2000
    int planet(const char* name, int parent, double aAU, double e, double I, double L, double varpi, double Om,
               double GM, double Rkm, double poleRA, double poleDec, double rotHours, double W0) {
        Body b;
        b.name = name;
        b.parent = parent;
        b.GM = GM;
        b.radius = Rkm * 1000.0;
        b.a = aAU * AU;
        b.e = e;
        double om = varpi - Om;
        b.M0 = (L - varpi) * DEG;
        b.orbitFrame = orbitRot(Om * DEG, I * DEG, om * DEG);
        b.poleFrame = poleFromRaDec(poleRA, poleDec);
        b.rotRate = TAU / (rotHours * 3600.0);
        b.W0 = W0 * DEG;
        return add(b);
    }
    // Moon: elements in parent's equatorial frame (refEq) or ecliptic. km / degrees.
    int moon(const char* name, int parent, double aKm, double e, double inc, double Om, double om, double M0,
             bool refEq, double GM, double Rkm, bool synchronous, double poleRA = 0, double poleDec = 90, double rotHours = 24) {
        Body b;
        b.name = name;
        b.parent = parent;
        b.GM = GM;
        b.radius = Rkm * 1000.0;
        b.a = aKm * 1000.0;
        b.e = e;
        b.M0 = M0 * DEG;
        Mat3d R = orbitRot(Om * DEG, inc * DEG, om * DEG);
        b.orbitFrame = refEq ? U.bodies[parent].poleFrame * R : R;
        if (synchronous) {
            b.poleFrame = b.orbitFrame;
            b.W0 = b.M0 + PI;
            b.rotRate = -1; // filled once mean motion is known
        } else {
            b.poleFrame = poleFromRaDec(poleRA, poleDec);
            b.rotRate = TAU / (rotHours * 3600.0);
        }
        return add(b);
    }
    int star(const char* name, const char* system, double raDeg, double decDeg, double distLy, double massSun,
             double radSun, double lumSun, double rotDays, Vec3f col) {
        Body b;
        b.name = name;
        b.system = system;
        b.isStar = true;
        b.parent = -1;
        b.GM = GM_SUN * massSun;
        b.radius = R_SUN * radSun;
        b.luminosity = L_SUN * lumSun;
        b.absPos = raDecToEcl(raDeg, decDeg) * (distLy * LY);
        b.poleFrame = poleFromRaDec(raDeg + 90, 40);
        b.rotRate = TAU / (rotDays * DAY);
        b.surf = Surf::Star;
        b.colA = col;
        return add(b);
    }
    // comet from heliocentric ecliptic J2000 elements (a, e, i, node, argument of perihelion, mean anomaly at J2000)
    // and its nucleus (diameter; mass from a density of 0.5 g/cm3 unless known)
    int comet(const char* name, double aAU, double e, double I, double Om, double w, double MJ2000, double diamKm, double rotHours,
              double GM = 0) {
        Body b;
        b.name = name;
        b.parent = 0; // (the Sun)
        b.radius = diamKm * 500.0;
        b.GM = GM > 0 ? GM : 6.6743e-11 * 500.0 * 4.0 / 3.0 * PI * b.radius * b.radius * b.radius;
        b.a = aAU * AU;
        b.e = e;
        b.M0 = MJ2000 * DEG;
        b.orbitFrame = orbitRot(Om * DEG, I * DEG, w * DEG);
        b.poleFrame = b.orbitFrame;
        b.rotRate = TAU / (rotHours * 3600.0);
        b.comet = true;
        b.surf = Surf::Rocky;
        b.system = "Solar System";
        return add(b);
    }
    // generic planet around another star (elements w.r.t. ecliptic; orientation unknown -> chosen)
    int exo(const char* name, int parent, double aAU, double e, double inc, double Om, double om, double M0,
            double massEarth, double radEarth) {
        Body b;
        b.name = name;
        b.parent = parent;
        b.GM = GM_EARTH * massEarth;
        b.radius = R_EARTH * radEarth;
        b.a = aAU * AU;
        b.e = e;
        b.M0 = M0 * DEG;
        b.orbitFrame = orbitRot(Om * DEG, inc * DEG, om * DEG);
        b.poleFrame = b.orbitFrame;
        b.rotRate = -1; // assume tidally locked
        b.W0 = b.M0 + PI;
        return add(b);
    }
};

void atmo(Body& b, double rho0, double Hkm, double P0, double topKm, Vec3f col) {
    b.atmRho0 = rho0;
    b.atmH = Hkm * 1000.0;
    b.atmP0 = P0;
    b.atmTop = topKm * 1000.0;
    b.atmColor = col;
}
void look(Body& b, Surf s, Vec3f a, Vec3f c2, Vec3f c3, float seed) {
    b.surf = s;
    b.colA = a;
    b.colB = c2;
    b.colC = c3;
    b.seed = seed;
}

} // namespace

void Universe::init() {
    bodies.clear();
    Builder B{*this};
    const double GMS = GM_SUN;

    // ---------------- Solar System ----------------
    int sun = B.star("Sun", "Solar System", 0, 0, 0, 1.0, 1.0, 1.0, 25.38, rgb(1.0, 0.86, 0.58));
    bodies[sun].poleFrame = poleFromRaDec(286.13, 63.87);
    bodies[sun].W0 = 84.176 * DEG;
    bodies[sun].desc = "G2V main-sequence star. 99.86% of the Solar System's mass.";

    int mercury = B.planet("Mercury", sun, 0.38709927, 0.20563593, 7.00497902, 252.25032350, 77.45779628, 48.33076593,
                           2.2032e13, 2439.7, 281.0103, 61.4155, 1407.6, 329.5988);
    int venus = B.planet("Venus", sun, 0.72333566, 0.00677672, 3.39467605, 181.97909950, 131.60246718, 76.67984255,
                         3.24859e14, 6051.8, 272.76, 67.16, -5832.6, 160.20);
    int earth = B.planet("Earth", sun, 1.00000261, 0.01671123, -0.00001531, 100.46457166, 102.93768193, 0.0,
                         GM_EARTH, 6371.0, 0.0, 90.0, 23.9344696, 190.147);
    int moon = B.moon("Moon", earth, 384400, 0.0549, 5.145, 125.08, 318.15, 135.27, false, 4.9048695e12, 1737.4, true);
    int mars = B.planet("Mars", sun, 1.52371034, 0.09339410, 1.84969142, -4.55343205, -23.94362959, 49.55953891,
                        4.282837e13, 3389.5, 317.681, 52.887, 24.6229, 176.630);
    int phobos = B.moon("Phobos", mars, 9376, 0.0151, 1.093, 16.9, 150.0, 92.0, true, 7.087e5, 11.27, true);
    int deimos = B.moon("Deimos", mars, 23463.2, 0.00033, 0.93, 47.0, 260.7, 296.2, true, 9.6e4, 6.2, true);
    int ceres = B.planet("Ceres", sun, 2.7675, 0.0758, 10.593, 230.0, 153.9, 80.305,
                         6.26325e10, 469.7, 291.418, 66.764, 9.074, 170.65);
    int jupiter = B.planet("Jupiter", sun, 5.20288700, 0.04838624, 1.30439695, 34.39644051, 14.72847983, 100.47390909,
                           1.26686534e17, 69911, 268.056595, 64.495303, 9.9250, 284.95);
    int io = B.moon("Io", jupiter, 421700, 0.0041, 0.05, 43.9, 84.1, 342.0, true, 5.959916e12, 1821.6, true);
    int europa = B.moon("Europa", jupiter, 671034, 0.009, 0.47, 219.1, 88.97, 171.0, true, 3.202739e12, 1560.8, true);
    int ganymede = B.moon("Ganymede", jupiter, 1070412, 0.0013, 0.2, 63.5, 192.4, 317.5, true, 9.887834e12, 2634.1, true);
    int callisto = B.moon("Callisto", jupiter, 1882709, 0.0074, 0.192, 298.8, 52.6, 181.4, true, 7.179289e12, 2410.3, true);
    int saturn = B.planet("Saturn", sun, 9.53667594, 0.05386179, 2.48599187, 49.95424423, 92.59887831, 113.66242448,
                          3.7931187e16, 58232, 40.589, 83.537, 10.656, 38.90);
    int enceladus = B.moon("Enceladus", saturn, 237948, 0.0047, 0.019, 0, 115, 200, true, 7.211e9, 252.1, true);
    int titan = B.moon("Titan", saturn, 1221870, 0.0288, 0.348, 28.1, 180.5, 163.3, true, 8.978138e12, 2574.7, true);
    int uranus = B.planet("Uranus", sun, 19.18916464, 0.04725744, 0.77263783, 313.23810451, 170.95427630, 74.01692503,
                          5.793939e15, 25362, 257.311, -15.175, -17.24, 203.81);
    int titania = B.moon("Titania", uranus, 435910, 0.0011, 180 - 0.34, 99.8, 284.4, 24.6, true, 2.27e11, 788.9, true);
    int neptune = B.planet("Neptune", sun, 30.06992276, 0.00859048, 1.77004347, -55.12002969, 44.96476227, 131.78422574,
                           6.836529e15, 24622, 299.36, 43.46, 16.11, 253.18);
    int triton = B.moon("Triton", neptune, 354759, 0.000016, 156.885, 177.6, 66.1, 352.3, true, 1.427598e12, 1353.4, true);
    int pluto = B.planet("Pluto", sun, 39.48211675, 0.24882730, 17.14001206, 238.92903833, 224.06891629, 110.30393684,
                         8.71e11, 1188.3, 132.993, -6.163, -153.2928, 302.695);
    int charon = B.moon("Charon", pluto, 19591, 0.0002, 180.0, 223.0, 0, 147.8, true, 1.058e11, 606.0, true);

    // ---------------- Alpha Centauri ----------------
    int acA = B.star("Alpha Centauri A", "Alpha Centauri", 219.90206, -60.83399, 4.344, 1.0788, 1.2175, 1.5059, 22.0, rgb(1.0, 0.93, 0.80));
    int acB = (int)bodies.size();
    {
        Body b;
        b.name = "Alpha Centauri B";
        b.parent = acA;
        b.isStar = true;
        b.GM = GM_SUN * 0.9092;
        b.radius = R_SUN * 0.8591;
        b.luminosity = L_SUN * 0.4981;
        b.a = 23.52 * AU;
        b.e = 0.5179;
        b.M0 = 210 * DEG;
        b.orbitFrame = orbitRot(205 * DEG, 79.32 * DEG, 232.3 * DEG);
        b.poleFrame = b.orbitFrame;
        b.rotRate = TAU / (36 * DAY);
        b.surf = Surf::Star;
        b.colA = rgb(1.0, 0.78, 0.52);
        b.desc = "K1V orange dwarf orbiting Alpha Centauri A every 79.9 years.";
        B.add(b);
    }
    int prox = B.star("Proxima Centauri", "Alpha Centauri", 217.42895, -62.67948, 4.2465, 0.1221, 0.1542, 0.00155, 83.0, rgb(1.0, 0.45, 0.25));
    int proxb = B.exo("Proxima Centauri b", prox, 0.04857, 0.02, 0, 0, 0, 40, 1.07, 1.03);
    int proxd = B.exo("Proxima Centauri d", prox, 0.02885, 0.04, 0, 0, 90, 200, 0.26, 0.81);
    // ---------------- Barnard's Star ----------------
    int barn = B.star("Barnard's Star", "Barnard's Star", 269.452, 4.6934, 5.963, 0.162, 0.187, 0.0034, 145.0, rgb(1.0, 0.5, 0.32));
    int barnb = B.exo("Barnard's Star b", barn, 0.0188, 0.03, 0, 0, 0, 120, 0.30, 0.71);

    // ---------------- comets (appended: the bodies before keep their indices in saved games) ----------------
    // JPL Small-Body Database elements (heliocentric ecliptic J2000), mean anomaly carried to J2000
    const int firstComet = (int)bodies.size();
    B.comet("Halley's Comet", 17.928635, 0.967936, 162.1905, 59.0989, 112.2414, 65.8903, 11.0, 52.8);
    B.comet("Encke's Comet", 2.218293, 0.847018, 11.3405, 334.0135, 187.2823, 286.1353, 4.8, 11.083);
    B.comet("Churyumov-Gerasimenko", 3.462249, 0.640908, 7.0403, 50.1356, 12.7982, 207.5540, 3.4, 12.76129, 662.2);
    B.comet("Tempel 1", 3.146134, 0.509703, 10.4734, 68.7536, 179.1973, 10.0274, 6.0, 40.7);
    B.comet("Hale-Bopp", 177.433384, 0.994981, 89.2876, 282.7334, 130.4147, 0.4203, 60.0, 11.3);
    B.comet("Swift-Tuttle", 26.092069, 0.963226, 113.4538, 139.3812, 152.9822, 19.0534, 26.0, 24.0);
    B.comet("Giacobini-Zinner", 3.500303, 0.710470, 32.0026, 195.4045, 172.8125, 52.5501, 2.0, 9.5);

    // ---------------- descriptions / systems ----------------
    for (auto& b : bodies) {
        if (b.system.empty()) b.system = "Solar System";
    }
    for (int i : {acB, proxb, proxd}) bodies[i].system = "Alpha Centauri";
    bodies[barnb].system = "Barnard's Star";

    // appearance + atmospheres
    look(bodies[mercury], Surf::Rocky, rgb(0.58, 0.55, 0.52), rgb(0.36, 0.34, 0.33), rgb(1, 1, 1), 1.3f);
    look(bodies[venus], Surf::Cloudy, rgb(0.95, 0.87, 0.65), rgb(0.82, 0.70, 0.45), rgb(1, 1, 1), 2.1f);
    atmo(bodies[venus], 65.0, 15.9, 9.2e6, 200, rgb(0.95, 0.80, 0.50));
    look(bodies[earth], Surf::Earth, rgb(0.05, 0.16, 0.42), rgb(0.22, 0.42, 0.16), rgb(0.62, 0.55, 0.40), 3.7f);
    atmo(bodies[earth], 1.225, 8.5, 101325, 140, rgb(0.35, 0.58, 1.0));
    // U.S. Standard Atmosphere 1976 (altitude km, density kg/m^3, pressure Pa)
    for (auto row : std::vector<std::array<double, 3>>{
             {0, 1.225, 101325}, {5, 0.7364, 54048}, {10, 0.4135, 26500}, {15, 0.1948, 12111}, {20, 0.08891, 5529},
             {25, 0.04008, 2549}, {30, 0.01841, 1197}, {40, 0.003996, 287.1}, {50, 0.001027, 79.78}, {60, 3.097e-4, 21.96},
             {70, 8.283e-5, 5.221}, {80, 1.846e-5, 1.052}, {90, 3.416e-6, 0.1836}, {100, 5.604e-7, 0.03201},
             {110, 9.708e-8, 0.007104}, {120, 2.222e-8, 0.002538}, {130, 8.152e-9, 0.001251}, {140, 3.831e-9, 7.203e-4}})
        bodies[earth].atmTable.push_back({row[0] * 1000.0, row[1], row[2]});
    look(bodies[moon], Surf::Rocky, rgb(0.66, 0.65, 0.63), rgb(0.36, 0.36, 0.37), rgb(1, 1, 1), 4.4f);
    look(bodies[mars], Surf::Desert, rgb(0.78, 0.42, 0.22), rgb(0.48, 0.24, 0.14), rgb(0.95, 0.93, 0.9), 5.9f);
    atmo(bodies[mars], 0.020, 11.1, 636, 125, rgb(0.85, 0.58, 0.42));
    look(bodies[phobos], Surf::Rocky, rgb(0.45, 0.40, 0.36), rgb(0.28, 0.25, 0.23), rgb(1, 1, 1), 6.2f);
    look(bodies[deimos], Surf::Rocky, rgb(0.52, 0.46, 0.40), rgb(0.33, 0.30, 0.27), rgb(1, 1, 1), 7.7f);
    look(bodies[ceres], Surf::Rocky, rgb(0.48, 0.47, 0.46), rgb(0.30, 0.30, 0.30), rgb(1, 1, 1), 8.1f);
    look(bodies[jupiter], Surf::Gas, rgb(0.86, 0.76, 0.62), rgb(0.66, 0.46, 0.32), rgb(0.96, 0.94, 0.88), 9.3f);
    atmo(bodies[jupiter], 0.16, 27, 1e5, 1000, rgb(0.85, 0.75, 0.60));
    look(bodies[io], Surf::Desert, rgb(0.92, 0.86, 0.42), rgb(0.72, 0.48, 0.18), rgb(0.95, 0.95, 0.85), 10.5f);
    look(bodies[europa], Surf::Icy, rgb(0.92, 0.88, 0.80), rgb(0.62, 0.45, 0.30), rgb(1, 1, 1), 11.2f);
    look(bodies[ganymede], Surf::Rocky, rgb(0.62, 0.58, 0.53), rgb(0.40, 0.37, 0.33), rgb(1, 1, 1), 12.6f);
    look(bodies[callisto], Surf::Rocky, rgb(0.46, 0.41, 0.36), rgb(0.24, 0.21, 0.19), rgb(1, 1, 1), 13.1f);
    look(bodies[saturn], Surf::Gas, rgb(0.93, 0.85, 0.64), rgb(0.80, 0.68, 0.48), rgb(0.97, 0.93, 0.80), 14.4f);
    atmo(bodies[saturn], 0.19, 59.5, 1e5, 1500, rgb(0.90, 0.82, 0.62));
    bodies[saturn].ringInner = 74500e3;
    bodies[saturn].ringOuter = 140220e3;
    look(bodies[enceladus], Surf::Icy, rgb(0.97, 0.97, 0.98), rgb(0.70, 0.78, 0.85), rgb(1, 1, 1), 15.8f);
    look(bodies[titan], Surf::Cloudy, rgb(0.88, 0.62, 0.30), rgb(0.70, 0.45, 0.20), rgb(1, 1, 1), 16.6f);
    atmo(bodies[titan], 5.3, 21, 146700, 600, rgb(0.90, 0.62, 0.28));
    look(bodies[uranus], Surf::Gas, rgb(0.64, 0.86, 0.89), rgb(0.55, 0.78, 0.84), rgb(0.72, 0.90, 0.92), 17.2f);
    atmo(bodies[uranus], 0.42, 27.7, 1e5, 800, rgb(0.60, 0.85, 0.90));
    look(bodies[titania], Surf::Rocky, rgb(0.58, 0.55, 0.53), rgb(0.38, 0.36, 0.35), rgb(1, 1, 1), 18.4f);
    look(bodies[neptune], Surf::Gas, rgb(0.30, 0.46, 0.92), rgb(0.22, 0.32, 0.75), rgb(0.70, 0.80, 1.00), 19.9f);
    atmo(bodies[neptune], 0.45, 19.7, 1e5, 800, rgb(0.35, 0.50, 0.95));
    look(bodies[triton], Surf::Icy, rgb(0.86, 0.80, 0.76), rgb(0.66, 0.55, 0.52), rgb(1, 1, 1), 20.3f);
    look(bodies[pluto], Surf::Icy, rgb(0.88, 0.80, 0.66), rgb(0.55, 0.38, 0.28), rgb(1, 1, 1), 21.7f);
    look(bodies[charon], Surf::Rocky, rgb(0.60, 0.58, 0.57), rgb(0.42, 0.38, 0.36), rgb(1, 1, 1), 22.5f);
    look(bodies[proxb], Surf::Desert, rgb(0.62, 0.42, 0.36), rgb(0.35, 0.24, 0.22), rgb(0.9, 0.9, 0.95), 23.1f);
    look(bodies[proxd], Surf::Rocky, rgb(0.55, 0.47, 0.44), rgb(0.32, 0.28, 0.27), rgb(1, 1, 1), 24.8f);
    look(bodies[barnb], Surf::Rocky, rgb(0.60, 0.50, 0.44), rgb(0.34, 0.29, 0.26), rgb(1, 1, 1), 25.2f);
    // comet nuclei: among the darkest surfaces known (albedo ~0.04), dust over ice
    for (int i = firstComet; i < (int)bodies.size(); i++)
        look(bodies[i], Surf::Rocky, rgb(0.20, 0.19, 0.18), rgb(0.10, 0.10, 0.10), rgb(0.9, 0.92, 0.95), 26.0f + (i - firstComet) * 1.3f);
    for (int i : {jupiter, saturn, uranus, neptune}) bodies[i].gasGiant = true;
    bodies[earth].soundSpeed = 320;
    bodies[venus].soundSpeed = 410;
    bodies[mars].soundSpeed = 240;
    bodies[titan].soundSpeed = 195;
    bodies[jupiter].soundSpeed = 1000;
    bodies[saturn].soundSpeed = 800;
    bodies[uranus].soundSpeed = 650;
    bodies[neptune].soundSpeed = 650;

    bodies[mercury].desc = "Smallest planet. No atmosphere, extreme temperature swings.";
    bodies[venus].desc = "Crushing 92-bar CO2 atmosphere. Rotates backwards.";
    bodies[earth].desc = "Home. Your factory and launch sites are here.";
    bodies[moon].desc = "Earth's only natural satellite. Tidally locked.";
    bodies[mars].desc = "Thin CO2 atmosphere (0.6 kPa). Polar ice caps.";
    bodies[phobos].desc = "Inner moon of Mars, slowly spiralling inward.";
    bodies[deimos].desc = "Tiny outer moon of Mars.";
    bodies[ceres].desc = "Dwarf planet, largest object in the asteroid belt.";
    bodies[jupiter].desc = "Gas giant. No solid surface - descending means destruction.";
    bodies[io].desc = "Most volcanically active body in the Solar System.";
    bodies[europa].desc = "Ice shell over a global subsurface ocean.";
    bodies[ganymede].desc = "Largest moon in the Solar System.";
    bodies[callisto].desc = "Heavily cratered, outside Jupiter's harsh radiation belts.";
    bodies[saturn].desc = "Ringed gas giant, less dense than water.";
    bodies[enceladus].desc = "Icy moon venting water plumes from its south pole.";
    bodies[titan].desc = "Thick nitrogen atmosphere (1.5 bar) and methane lakes.";
    bodies[uranus].desc = "Ice giant tilted 98 degrees on its side.";
    bodies[titania].desc = "Largest moon of Uranus.";
    bodies[neptune].desc = "Windiest planet in the Solar System.";
    bodies[triton].desc = "Captured Kuiper-belt object on a retrograde orbit.";
    bodies[pluto].desc = "Dwarf planet with nitrogen-ice plains.";
    bodies[charon].desc = "Half Pluto's size; the pair are mutually tidally locked.";
    bodies[acA].desc = "G2V star, slightly larger and brighter than the Sun.";
    bodies[prox].desc = "M5.5V red dwarf, the closest star to the Sun (4.25 ly).";
    bodies[proxb].desc = "Planet in the habitable zone (radius estimated).";
    bodies[proxd].desc = "Small inner planet (radius estimated).";
    bodies[barn].desc = "M4V red dwarf, 5.96 ly away, fastest proper motion of any star.";
    bodies[barnb].desc = "Sub-Earth on a 3.15-day orbit (radius estimated).";
    bodies[firstComet + 0].desc = "1P/Halley. Returns every 76 years; next perihelion in 2061.";
    bodies[firstComet + 1].desc = "2P/Encke. The shortest-period bright comet: 3.3 years.";
    bodies[firstComet + 2].desc = "67P. Visited by Rosetta and the Philae lander (2014).";
    bodies[firstComet + 3].desc = "9P/Tempel 1. Struck by the Deep Impact probe in 2005.";
    bodies[firstComet + 4].desc = "C/1995 O1. The Great Comet of 1997, with a 60 km nucleus.";
    bodies[firstComet + 5].desc = "109P. Parent of the Perseid meteor shower.";
    bodies[firstComet + 6].desc = "21P. Parent of the Draconid meteor shower.";

    // ---------------- derived quantities ----------------
    for (int i = 0; i < count(); i++) {
        Body& b = bodies[i];
        if (b.parent >= 0) {
            Body& p = bodies[b.parent];
            b.muOrbit = p.GM + b.GM;
            b.n = std::sqrt(b.muOrbit / (b.a * b.a * b.a));
            if (b.rotRate < 0) b.rotRate = b.n; // synchronous rotation
            b.soi = b.a * std::pow(b.GM / p.GM, 0.4);
            // a tiny body close to its planet (Phobos: 7 km against an 11 km radius) would have its sphere of
            // influence inside itself - nothing could orbit or land on it. There its Hill sphere counts (where its
            // own gravity still holds things: ~17 km for Phobos), and never less than 1.5 radii
            if (b.soi < 3 * b.radius) {
                double hill = b.a * (1 - b.e) * std::cbrt(b.GM / (3 * p.GM));
                b.soi = std::max({b.soi, hill, 1.5 * b.radius});
            }
            p.children.push_back(i);
        } else {
            b.soi = 1e300;
        }
    }
    (void)GMS;
    roots.clear();
    for (int i = 0; i < count(); i++) {
        int r = i;
        while (bodies[r].parent >= 0) r = bodies[r].parent;
        bodies[i].root = r;
        if (bodies[i].parent < 0) roots.push_back(i);
    }
    // gravitationally relevant bodies for each reference body
    relevant.assign(count(), {});
    for (int r = 0; r < count(); r++) {
        std::set<int> s(roots.begin(), roots.end());
        s.insert(r);
        for (int c : bodies[r].children) s.insert(c);
        int a = r;
        while (a >= 0) {
            s.insert(a);
            for (int c : bodies[a].children) s.insert(c);
            a = bodies[a].parent;
        }
        relevant[r].assign(s.begin(), s.end());
    }
}

int Universe::find(const std::string& name) const {
    for (int i = 0; i < count(); i++)
        if (bodies[i].name == name) return i;
    return -1;
}

static void keplerRel(const Body& b, double t, Vec3d& pos, Vec3d& vel) {
    double M = wrapAngle(b.M0 + b.n * t);
    double e = b.e;
    double E = e < 0.8 ? M : PI;
    for (int k = 0; k < 30; k++) {
        double f = E - e * std::sin(E) - M;
        double d = 1 - e * std::cos(E);
        double dE = f / d;
        E -= dE;
        if (std::fabs(dE) < 1e-13) break;
    }
    double cE = std::cos(E), sE = std::sin(E);
    double sq = std::sqrt(1 - e * e);
    double r = b.a * (1 - e * cE);
    Vec3d pp{b.a * (cE - e), b.a * sq * sE, 0};
    double k = std::sqrt(b.muOrbit * b.a) / r;
    Vec3d vv{-k * sE, k * sq * cE, 0};
    pos = b.orbitFrame * pp;
    vel = b.orbitFrame * vv;
}

static void stateOne(const Universe& U, double t, int i, std::vector<BodyState>& out) {
    const Body& b = U.bodies[i];
    if (b.parent < 0) {
        out[i] = BodyState{};
        return;
    }
    Vec3d p, v;
    keplerRel(b, t, p, v);
    const BodyState& ps = out[b.parent];
    double r = len(p);
    out[i].pos = ps.pos + p;
    out[i].vel = ps.vel + v;
    out[i].acc = ps.acc - p * (b.muOrbit / (r * r * r));
}

void Universe::computeStates(double t, std::vector<BodyState>& out) const {
    out.resize(bodies.size());
    for (int i = 0; i < count(); i++) stateOne(*this, t, i, out);
}

void Universe::computeStatesFor(double t, const std::vector<int>& which, std::vector<BodyState>& out) const {
    out.resize(bodies.size());
    for (int i : which) stateOne(*this, t, i, out);
}

Vec3d Universe::relPos(const std::vector<BodyState>& s, int i, int j) const {
    int ri = bodies[i].root, rj = bodies[j].root;
    if (ri == rj) return s[i].pos - s[j].pos;
    return (bodies[ri].absPos - bodies[rj].absPos) + (s[i].pos - s[j].pos);
}

Vec3d Universe::relVel(const std::vector<BodyState>& s, int i, int j) const { return s[i].vel - s[j].vel; }

Mat3d Universe::bodyRot(int i, double t) const {
    const Body& b = bodies[i];
    double W = b.W0 + b.rotRate * t;
    W = std::fmod(W, TAU);
    return b.poleFrame * rotZ(W);
}

Vec3d Universe::angVel(int i) const { return bodies[i].poleFrame.col(2) * bodies[i].rotRate; }

static double tableLookup(const std::vector<std::array<double, 3>>& T, double alt, int col) {
    if (alt <= T.front()[0]) return T.front()[col];
    for (size_t k = 1; k < T.size(); k++) {
        if (alt <= T[k][0]) {
            double f = (alt - T[k - 1][0]) / (T[k][0] - T[k - 1][0]);
            return std::exp(std::log(T[k - 1][col]) * (1 - f) + std::log(T[k][col]) * f); // log-linear
        }
    }
    return T.back()[col];
}

double Universe::density(int i, double alt) const {
    const Body& b = bodies[i];
    if (b.atmTop <= 0 || alt >= b.atmTop) return 0;
    if (!b.atmTable.empty()) return tableLookup(b.atmTable, alt, 1);
    return b.atmRho0 * std::exp(-std::max(alt, -5 * b.atmH) / b.atmH);
}
double Universe::pressure(int i, double alt) const {
    const Body& b = bodies[i];
    if (b.atmTop <= 0 || alt >= b.atmTop) return 0;
    if (!b.atmTable.empty()) return tableLookup(b.atmTable, alt, 2);
    return b.atmP0 * std::exp(-std::max(alt, -5 * b.atmH) / b.atmH);
}

int Universe::hostStar(int i) const {
    while (i >= 0 && !bodies[i].isStar) i = bodies[i].parent;
    return i < 0 ? 0 : i;
}

std::string Universe::fullTitle(int i) const { return bodies[i].name + "  (" + bodies[i].system + ")"; }

double surfaceGravity(const Body& b) { return b.GM / (b.radius * b.radius); }
