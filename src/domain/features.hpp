// Named surface features of the bodies (mountains, basins, craters, plains...), with their centres and
// sizes. Coordinates: planetocentric latitude, east longitude (-180..180) as the surface maps use them.
// Earth, Moon and Mars: from the real maps (the labels mark relief that is there). Bodies with generated
// relief: the feature is carved into the relief (terrain.cpp), so the label marks what you see.
// Sources: IAU / USGS Gazetteer of Planetary Nomenclature (planetarynames.wr.usgs.gov), west longitudes
// converted to east.
#pragma once

namespace features {

enum Kind { Mons = 0, Range, Crater, Basin, Mare, Planitia, Valley, Plateau, Region, Site, Ice };
struct Feature {
    const char* body;
    const char* name; // English (translated by the i18n table)
    double lat, lon;  // [deg]
    double radiusKm;  // half the size across
    Kind kind;
    double carve;     // generated relief only: depth (craters, basins) or height (mons) to carve [m]; 0: none
};

const Feature* all(int& count);

} // namespace features
