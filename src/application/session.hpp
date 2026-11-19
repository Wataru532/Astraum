// Use cases of a game session: maps (one saved game each, named by the player), saving and autosaving, the
// factory's last design. The dialogs around them are in the presentation (map_dialogs.cpp).
#pragma once
#include <string>
#include <vector>
#include "app.hpp"

struct MapInfo {
    std::string path, name;
    long long savedAt = 0; // seconds since 1970 (file time)
    int vessels = 0;
};
std::vector<MapInfo> listMaps(const App& app);
MapInfo readInfo(const std::string& path);          // the name and the vessel count, without loading the game
bool createMap(App& app, const std::string& name); // a new game saved as a new map
bool loadMap(App& app, const std::string& path);
bool deleteMap(App& app, const std::string& path);
bool saveMap(App& app, bool autosave, bool wait = false); // the current map (written on a worker unless `wait`)
void finishSaves();                                 // wait for a write in progress (before loading, at exit)
bool renameMap(App& app, const std::string& path, const std::string& name);
void serviceSaves(App& app);                        // report a failed background write, write pending pictures
bool mapDirty(const App& app);                      // the current map has changes that are not saved
void markMapSaved(App& app);
void leaveMap(App& app);                            // save and leave the current map (a tutorial starts)
bool quickSave(App& app);                           // F5 / the save button: the map, or the quicksave file
void persistDesign(App& app, bool force);           // write the factory design when it changed
void restoreDesign(App& app);                       // the player's last design back into the factory
void loadLastDesign(App& app);                      // at start-up
void migrateQuicksave(App& app);                    // an old quicksave becomes the first map
void notice(App& app, const std::string& s, bool bad = false); // the short notice at the top of the screen
// helpers shared with the dialogs
std::string localTimeText(long long t);
std::string trimmed(const std::string& s);
std::string thumbPath(const std::string& mapPath); // the map's picture next to it (.png)
