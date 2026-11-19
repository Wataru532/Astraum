// Sound effects and music. One-shot events play the WAV files of src/audio/fx (embedded in the
// executable; synthesised as before if one is missing); the continuous mix (engine rumble, aerodynamic
// wind, RCS hiss) follows the flight and is generated on the audio thread. The music (src/audio/bgm,
// embedded) plays now and then: a randomly chosen track, then a quiet pause of random length.
#pragma once

namespace sfx {
enum class Ev { Click, Toggle, Stage, Ignite, Explosion, Chute, Touchdown, HyperIn, HyperOut, Tear, Milestone, ExplosionFar, COUNT };

// continuous sounds of the vessel being watched (all 0..1; zero = silence)
struct Ambient {
    float engine = 0;  // combined engine loudness
    float vacuum = 0;  // 1 = no air: the roar becomes a muffled, structure-borne rumble
    float crackle = 0; // solid / kerosene engines crackle, hydrogen engines do not
    float wind = 0;    // aerodynamic noise from dynamic pressure
    float speed = 0;   // airspeed / 3000 m/s (pitch of the wind)
    float rcs = 0;     // RCS thrusters firing
};

bool init();                 // false if no audio device is available (the game then stays silent)
void shutdown();
void setVolume(float v);     // master volume 0..1
void play(Ev e, float gain = 1.0f);
void setAmbient(const Ambient& a);
// music: on / off and its own volume (0..1, apart from the effects); call update once per frame
void setMusic(bool on, float volume);
void update(float dt);            // streams the music, schedules the next track
void pauseMusic(bool paused);     // (the window is minimised)
int musicTracks();                // tracks found (0: no music)
int musicPlaying();               // the track playing now, -1 none
float musicPause();               // seconds until the next track while none plays
void musicDebugSkipPause();       // test hook: start the next track now
void musicDebugSeekEnd();         // test hook: jump to 1 s before the end of the track playing
float musicLength(int track);     // [s]
int effectsFromFiles();           // effects loaded from their files (the others synthesised)
// development aid: write every event and a few continuous mixes as WAV files into dir (no audio device needed)
bool dump(const char* dir);
} // namespace sfx
