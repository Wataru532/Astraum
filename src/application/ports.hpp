// Ports of the application layer: what the use cases need from the outer layers, plugged in by main.cpp.
// Without a plug the defaults keep things working (std::chrono clock, English, no pictures, no tutorial).
#pragma once
#include <string>

namespace ports {

// the platform: a monotonic clock in seconds, the system's display language (i18n::Lang), Roaming AppData (UTF-8), a log line
struct Platform {
    double (*now)() = nullptr;
    int (*systemLanguage)() = nullptr;
    std::string (*appDataDir)() = nullptr;
    void (*log)(const char* line) = nullptr;
};
// the presentation: the map's picture (a copy of the view), the guided tutorials
struct Presentation {
    void (*mapSaved)(const std::string& mapPath) = nullptr; // the map was saved: a picture of the view may go next to it
    void (*worldReplaced)() = nullptr;                      // another world was loaded: the view copy is stale
    void (*serviceThumbs)() = nullptr;                      // write pending pictures (main thread)
    bool (*tutorialActive)() = nullptr;
    void (*stopTutorial)() = nullptr;
};

void install(const Platform& p);
void install(const Presentation& p);

double now();
int systemLanguage();
std::string appDataDir();
void log(const char* fmt, ...);

// the presentation port with no-op defaults for the parts not plugged in
struct PresentationCalls {
    void mapSaved(const std::string& path) const;
    void worldReplaced() const;
    void serviceThumbs() const;
    bool tutorialActive() const;
    void stopTutorial() const;
};
PresentationCalls presentation();

} // namespace ports
