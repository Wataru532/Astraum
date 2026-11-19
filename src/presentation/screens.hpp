// The screens and the dialogs of the presentation layer, called from the main loop (main.cpp)
#pragma once
#include <string>
#include "app.hpp"

void menuFrame(App& app);
void factoryFrame(App& app);
void flightFrame(App& app);
void cadFrame(App& app);
void cadTrimMemory(bool hard);
void cadSetCraft(App& app, const Craft& c); // replace the CAD craft (old one kept on the undo stack)
void flightOnEnter(App& app);
bool flightPaused();          // pause menu open in the flight view
void settingsModal(App& app);
void keyBindingsModal(App& app);
void creditsModal(App& app);
void crashModal(App& app);
void drawNotice(const App& app); // the short notice at the top (screenshot saved, map saved)
void setFullscreen(App& app, bool on); // borderless full screen (F11)

// ---- maps: the dialogs and the map's picture (map_dialogs.cpp)
void newMapModal(App& app);
void mapDialogsClosed(const App& app);              // (every menu frame: dialogs closed from outside start fresh)
void loadMapModal(App& app);
int closeDialog(App& app);                          // -1 open, 0 save and quit, 1 quit without saving, 2 cancel
extern int g_testCloseChoice;                       // tests: the answer the dialog gives
std::string debugNewMapName();                      // tests: the text in the new-map dialog
bool debugRenameMap(App& app, const std::string& name);
void grabWorldView();                               // flight: a copy of the 3D view (no HUD) for the map's picture
void forgetWorldView();
void finishThumbs();                                // at exit: a pending picture, then free the GPU copies
void mapSavedPicture(const std::string& mapPath);   // (ports::Presentation)
void serviceMapPictures();
