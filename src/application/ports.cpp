#include "ports.hpp"
#include <chrono>
#include <cstdarg>
#include <cstdio>

namespace ports {
namespace {
Platform g_platform;
Presentation g_presentation;
} // namespace

void install(const Platform& p) { g_platform = p; }
void install(const Presentation& p) { g_presentation = p; }

double now() {
    if (g_platform.now) return g_platform.now();
    using namespace std::chrono;
    static const steady_clock::time_point t0 = steady_clock::now();
    return duration<double>(steady_clock::now() - t0).count();
}
int systemLanguage() { return g_platform.systemLanguage ? g_platform.systemLanguage() : 0; }
void log(const char* fmt, ...) {
    if (!g_platform.log) return;
    char b[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    g_platform.log(b);
}

void PresentationCalls::mapSaved(const std::string& path) const {
    if (g_presentation.mapSaved) g_presentation.mapSaved(path);
}
void PresentationCalls::worldReplaced() const {
    if (g_presentation.worldReplaced) g_presentation.worldReplaced();
}
void PresentationCalls::serviceThumbs() const {
    if (g_presentation.serviceThumbs) g_presentation.serviceThumbs();
}
bool PresentationCalls::tutorialActive() const { return g_presentation.tutorialActive && g_presentation.tutorialActive(); }
void PresentationCalls::stopTutorial() const {
    if (g_presentation.stopTutorial) g_presentation.stopTutorial();
}
PresentationCalls presentation() { return {}; }

} // namespace ports
