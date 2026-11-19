// The domain's ports with their defaults (a smooth ground, no diagnostics sink)
#include <cstdarg>
#include <cstdio>
#include "diag.hpp"
#include "ground.hpp"

namespace {
class SmoothGround : public GroundPort {
public:
    bool hasRelief(int) override { return false; }
    bool reliefReady(int) override { return false; }
    double height(int, const Vec3d&) override { return 0; }
    double maxHeight(int) override { return 0; }
    unsigned revision(int) override { return 0; }
    void focus(int, const Vec3d&, double) override {}
    void setFlatSpots(int, const std::vector<Vec3d>&, double) override {}
};
SmoothGround g_smooth;
GroundPort* g_ground = &g_smooth;
diag::Sink g_sink = nullptr;
} // namespace

namespace ground {
void install(GroundPort* p) { g_ground = p ? p : &g_smooth; }
GroundPort& port() { return *g_ground; }
} // namespace ground

namespace diag {
void setSink(Sink s) { g_sink = s; }
void breadcrumb(const char* fmt, ...) {
    if (!g_sink) return;
    char b[256];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    g_sink(b);
}
} // namespace diag
