// Tutorial missions: guided first flights with step-by-step goals checked against the simulation
#pragma once
struct App;

namespace tut {
int count();
const char* title(int i);
const char* summary(int i);
bool start(App& app, int i); // replaces the current game with the tutorial's starting situation
void stop();
bool active();
int stepIndex();   // current step (tests)
bool finished();
// evaluate the current step and draw the guide card (call from the flight and factory screens)
void frame(App& app);
// Live guidance of the current step: the numbers its hint shows to the player. The automated test
// pilot flies by exactly these, which proves that a player following the hints arrives.
struct Guide {
    double tIgnite = 1e18; // seconds until the burn the step asks for should start (1e18: none planned)
    int sas = 0;           // SAS mode the step asks for (0: none)
    bool enc = false;      // a lunar encounter is predicted (or the vessel is already at the Moon)
    double encPe = 0;      // its periapsis altitude [m]
    double encTilt = 0;    // its tilt against the lunar equator [deg]
    double tilt = 0;       // tilt of the current orbit against the equator of its body [deg]
    double target = 0;     // an apoapsis the step asks for (radius) [m] (0: none)
    int trimAxis = -1;     // course correction: RCS axis to push (0 +x right, 1 -x, 2 +y up, 3 -y, 4 +z forward, 5 -z)
};
const Guide& guide();
void debugNext();          // tests: click "Next" on an explanation step
bool stepDone();           // the current step is complete (its check mark shows before the next one)
// the tutorial list opened from the main menu
extern bool menuOpen;
void menuModal(App& app);
} // namespace tut
