// F3 debug screen. Two columns drawn over everything: on the left the program (frame rate and frame
// time graph, GPU, display, memory and caches, hardware temperatures), on the right the simulation and
// the active vessel (time, warp, position, orbit, attitude, aerodynamics, heating, power, deployables,
// camera). Plain technical labels; nothing here changes any state.
#include "brand.hpp"
#include "debugview.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>
#include "app.hpp"
#include "screens.hpp"
#include "session.hpp"
#include "storage.hpp"
#include "gpumon.hpp"
#include "gputime.hpp"
#include "i18n.hpp"
#include "mem.hpp"
#include "orbit.hpp"
#include "render.hpp"
#include "tutorial.hpp"
#include "ui.hpp"

void flightDebugGetView(double& yaw, double& pitch, double& dist, Vec3d& target);
double flightDebugFovy();
bool flightMapMode();
int frameLimit(const App& app, bool testing);

namespace dbg {
using namespace ui;
namespace {
std::string g_vendor, g_renderer, g_version, g_glsl;
constexpr int HIST = 240;
float g_ft[HIST] = {};
int g_head = 0, g_count = 0;

std::string fmtS(const char* f, ...) {
    char b[512];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(b, sizeof b, f, ap);
    va_end(ap);
    return b;
}

struct Col {
    float x, y;
    bool right;
    std::vector<std::pair<std::string, Color>> lines;
    void add(const std::string& s, Color c = Color{225, 230, 238, 255}) { lines.push_back({s, c}); }
    void head(const std::string& s) { lines.push_back({s, Color{120, 200, 255, 255}}); }
    void gap() { lines.push_back({"", BLANK}); }
    float draw(float size) {
        float lh = size + 4, yy = y;
        float SW = ui::SW();
        // one backing plate per column (covers the HUD underneath completely)
        float wMax = 0, hTot = 0;
        for (auto& l : lines) {
            if (!l.first.empty()) wMax = std::max(wMax, measure(l.first, size).x);
            hTot += l.first.empty() ? lh * 0.5f : lh;
        }
        float px = right ? SW - x - wMax - 12 : x - 6;
        DrawRectangleRounded({px, y - 6, wMax + 18, hTot + 10}, 0.02f, 4, Color{6, 9, 14, 246});
        for (auto& l : lines) {
            if (!l.first.empty()) {
                Vector2 m = measure(l.first, size);
                float xx = right ? SW - x - m.x - 6 : x;
                text(l.first, xx, yy, size, l.second);
            }
            yy += l.first.empty() ? lh * 0.5f : lh;
        }
        return yy;
    }
};

Color tempColor(float c) { return c >= 90 ? col::bad : (c >= 75 ? col::warn : Color{225, 230, 238, 255}); }

std::string zoneName(const std::string& raw) {
    std::string s = raw;
    for (auto& ch : s) ch = (char)std::tolower((unsigned char)ch);
    auto has = [&](const char* k) { return s.find(k) != std::string::npos; };
    if (has("cpu")) return "CPU";
    if (has("gfx") || has("gpu")) return "GPU";
    if (has("bat")) return "Battery";
    if (has("chg")) return "Charger";
    if (has("pch")) return "Chipset";
    if (has("sk1")) return "Skin 1";
    if (has("sk2")) return "Skin 2";
    if (has("ext")) return "External";
    if (has("loc")) return "Local";
    size_t p = raw.find_last_of('.');
    return p != std::string::npos ? raw.substr(p + 1) : raw;
}

void horizon(const World& W, const Vessel& v, Vec3d& up, Vec3d& north, Vec3d& east) {
    const Body& b = W.U.bodies[v.ref];
    up = norm(v.r);
    Vec3d pole = b.poleFrame.col(2);
    north = pole - up * dot(pole, up);
    if (len(north) < 1e-9) north = anyPerp(up);
    north = norm(north);
    east = cross(north, up);
}
} // namespace

void noteLog(const char* line) {
    if (!line) return;
    auto grab = [&](const char* key, std::string& out) {
        const char* p = std::strstr(line, key);
        if (!p) return false;
        p += std::strlen(key);
        while (*p == ' ' || *p == '\t') p++;
        out = p;
        return true;
    };
    if (grab("> Vendor:", g_vendor) || grab("> Renderer:", g_renderer) || grab("> Version:", g_version) || grab("> GLSL:", g_glsl)) return;
}

void frame(float dt) {
    g_ft[g_head] = dt;
    g_head = (g_head + 1) % HIST;
    g_count = std::min(g_count + 1, HIST);
}

std::vector<gputime::Part> g_parts;
void draw(App& app) {
    World& W = app.world;
    const float size = 12;
    // ---------------- frame statistics
    float sum = 0, mx = 0;
    std::vector<float> sorted;
    for (int i = 0; i < g_count; i++) {
        float f = g_ft[(g_head - 1 - i + HIST) % HIST];
        sum += f;
        mx = std::max(mx, f);
        sorted.push_back(f);
    }
    float avg = g_count ? sum / g_count : 0;
    std::sort(sorted.begin(), sorted.end());
    float p99 = sorted.empty() ? 0 : sorted[std::min(sorted.size() - 1, (size_t)(sorted.size() * 0.99))];

    Col L{10, 10, false, {}};
    L.head(fmtS("Astraum %s  -  debug (F3)", APP_VERSION));
    int fps = GetFPS(), limit = frameLimit(app, false);
    L.add(fmtS("%d fps  (limit %d)   frame %.2f ms   avg %.2f   max %.2f   1%% low %.0f fps", fps, limit, GetFrameTime() * 1000, avg * 1000,
               mx * 1000, p99 > 0 ? 1.0 / p99 : 0.0),
          fps >= limit * 0.92f ? col::good : (fps >= 30 ? col::warn : col::bad));
    float gpu = gpumon::utilization();
    L.add(gpu >= 0 ? fmtS("GPU load %.0f %%   planet detail %d/16 (%s)", gpu, gfx::planetDetail(), app.adaptiveDetail ? "adaptive" : "fixed")
                   : fmtS("GPU load n/a   planet detail %d/16", gfx::planetDetail()));
    if (!g_renderer.empty()) L.add("GL " + g_renderer);
    if (!g_version.empty()) L.add("OpenGL " + g_version + (g_glsl.empty() ? "" : "   GLSL " + g_glsl));
    int mon = GetCurrentMonitor();
    L.add(fmtS("Window %dx%d px   render %dx%d   UI %.0fx%.0f (x%.2f)   monitor %d Hz", GetScreenWidth(), GetScreenHeight(), GetRenderWidth(),
               GetRenderHeight(), ui::SW(), ui::SH(), ui::scale(), GetMonitorRefreshRate(mon)));
    L.add(fmtS("CPU threads %u   screen %s   language %d   text size %d", std::thread::hardware_concurrency(),
               app.screen == Screen::Flight ? "Flight" : app.screen == Screen::Factory ? "Factory" : app.screen == Screen::CAD ? "CAD" : "Menu", app.lang,
               app.textSize));
    L.gap();
    L.head("Memory");
    const mem::Status& ms = mem::status();
    L.add(fmtS("Process %zu MB of %zu MB budget   system free %zu / %zu MB   trims %d", ms.process / mem::MB, mem::BUDGET / mem::MB,
               ms.sysAvail / mem::MB, ms.sysTotal / mem::MB, ms.trims),
          ms.process > mem::SOFT ? col::warn : Color{225, 230, 238, 255});
    L.add(fmtS("Aero tables %.1f MB   planet fields %.1f MB   rewind states %zu", aeroCacheBytes() / (double)mem::MB,
               gfx::bakedFieldBytes() / (double)mem::MB, W.history.size()));
    L.gap();
    {
        // GPU time per part of the frame (timestamp queries; averaged since the screen was opened)
        static double resetAt = 0;
        if (GetTime() - resetAt > 3.0) {
            resetAt = GetTime();
            g_parts = gputime::averages();
            gputime::reset();
        }
        L.head("GPU time per part (ms, last 3 s)");
        std::string line;
        int n = 0;
        for (const gputime::Part& p : g_parts) {
            if (p.name == "start") continue;
            line += fmtS("%s %.2f   ", p.name.c_str(), p.gpuMs);
            if (++n % 5 == 0) {
                L.add(line);
                line.clear();
            }
        }
        if (!line.empty()) L.add(line);
        if (g_parts.empty()) L.add(gputime::available() ? "measuring..." : "timer queries not available", col::dim);
    }
    L.gap();
    L.head("Temperatures");
    auto temps = gpumon::temperatures();
    if (temps.empty()) L.add("no thermal sensors exposed by the system", col::dim);
    else {
        std::string line;
        int n = 0;
        // CPU and GPU first
        std::stable_sort(temps.begin(), temps.end(), [](const auto& a, const auto& b) {
            auto rank = [](const std::string& s) { std::string z = zoneName(s); return z == "CPU" ? 0 : z == "GPU" ? 1 : 2; };
            return rank(a.first) < rank(b.first);
        });
        float hot = -1e9;
        for (auto& t : temps) {
            line += fmtS("%s %.1f \xC2\xB0" "C   ", zoneName(t.first).c_str(), t.second);
            hot = std::max(hot, t.second);
            if (++n % 4 == 0) {
                L.add(line, tempColor(hot));
                line.clear();
                hot = -1e9;
            }
        }
        if (!line.empty()) L.add(line, tempColor(hot));
    }
    if (tut::active()) {
        L.gap();
        L.add(fmtS("Tutorial step %d%s", tut::stepIndex() + 1, tut::finished() ? " (complete)" : ""));
    }
    float graphY = L.draw(size) + 6;

    // frame time graph (newest on the right), 1 px per frame, lines at 60 and 30 fps
    {
        float gx = 10, gw = HIST * 2.0f, gh = 70, scaleMs = 50;
        DrawRectangle((int)gx - 6, (int)graphY - 3, (int)gw + 60, (int)gh + 6, Color{6, 9, 14, 246});
        for (int i = 0; i < g_count; i++) {
            float f = g_ft[(g_head - g_count + i + HIST) % HIST] * 1000;
            float hgt = std::min(gh, f / scaleMs * gh);
            Color c = f <= 17.5f ? col::good : (f <= 34 ? col::warn : col::bad);
            DrawRectangle((int)(gx + (HIST - g_count + i) * 2), (int)(graphY + gh - hgt), 2, (int)std::max(1.0f, hgt), c);
        }
        for (float ms2 : {16.7f, 33.3f}) {
            float yy = graphY + gh - ms2 / scaleMs * gh;
            DrawLineEx({gx, yy}, {gx + gw, yy}, 1, Color{255, 255, 255, 70});
            text(fmtS("%.0f ms", ms2), gx + gw + 6, yy - 7, 11, col::dim);
        }
    }

    // ---------------- simulation / vessel
    Col R{10, 10, true, {}};
    const Vessel* v = W.activeVessel();
    R.head("Simulation");
    R.add(fmtS("t = %.2f s after J2000   %s", W.t, formatDate(W.t).c_str()));
    R.add(fmtS("Warp x%g (level %d, effective x%.3g)%s%s", W.warpFactor(), W.warpIndex, W.effectiveWarp,
               W.warpLimitReason.empty() ? "" : ("   limited: " + W.warpLimitReason).c_str(),
               W.warpTo > 0 ? ("   to point in " + formatTime(W.warpTo - W.t)).c_str() : ""));
    int debris = 0, dead = 0;
    for (auto& o : W.vessels) {
        debris += o.debris && !o.destroyed;
        dead += o.destroyed;
    }
    R.add(fmtS("Vessels %zu (debris %d, destroyed %d)   hyperspace %s", W.vessels.size(), debris, dead, W.hyper.active ? "active" : "off"));
    if (v && app.gameStarted) {
        const Body& b = W.U.bodies[v->ref];
        double alt = len(v->r) - b.radius;
        Vec3d vs = W.surfaceVel(*v);
        Vec3d up, north, east;
        horizon(W, *v, up, north, east);
        R.gap();
        R.head(fmtS("Vessel #%d  %s", v->id, v->name.c_str()));
        R.add(fmtS("%zu components   stage %d   mass %.3f t   length %.1f m   radius %.2f m", v->comps.size(), v->stage, W.vesselMass(*v) / 1000,
                   v->length, v->maxR));
        R.add(fmtS("CoM (%.2f, %.2f, %.2f) m   %s%s", v->mp.com.x, v->mp.com.y, v->mp.com.z, v->landed ? "landed" : "flying", v->destroyed ? "  DESTROYED" : ""));
        Mat3d Rb = W.U.bodyRot(v->ref, W.t);
        Vec3d f = norm(Rb.T() * v->r);
        R.add(fmtS("Ref %s   alt %.1f m   lat %.4f\xC2\xB0  lon %.4f\xC2\xB0", tr(b.name).c_str(), alt, std::asin(clampd(f.z, -1, 1)) / DEG,
                   std::atan2(f.y, f.x) / DEG));
        R.add(fmtS("r (%.3f, %.3f, %.3f) km   |r| %.3f km", v->r.x / 1000, v->r.y / 1000, v->r.z / 1000, len(v->r) / 1000));
        R.add(fmtS("v (%.2f, %.2f, %.2f) m/s   orbital %.2f   surface %.2f   vertical %+.2f", v->v.x, v->v.y, v->v.z, len(v->v), len(vs), dot(vs, up)));
        if (!v->landed) {
            Conic c = stateToConic(v->r, v->v, b.GM);
            if (c.valid)
                R.add(fmtS("a %.1f km   e %.5f   i %.3f\xC2\xB0   Pe %.1f km   Ap %s   T %s", c.a / 1000, c.e, c.inc / DEG, (c.pe - b.radius) / 1000,
                           c.e < 1 ? fmtS("%.1f km", (c.ap - b.radius) / 1000).c_str() : "escape", c.period > 0 ? formatTime(c.period).c_str() : "-"));
        }
        R.add(fmtS("SOI %s   bound %d   orbit class %d", b.soi > 1e299 ? "-" : formatDist(b.soi).c_str(), v->bound, (int)v->orbitClass));
        Vec3d az = v->q.rotate({0, 0, 1}), ay = v->q.rotate({0, 1, 0});
        double pitch = std::asin(clampd(dot(az, up), -1, 1)) / DEG, hdg = wrapAngle(std::atan2(dot(az, east), dot(az, north))) / DEG;
        Vec3d hr = norm(cross(az, up));
        double roll = len(hr) > 1e-6 ? std::atan2(dot(ay, cross(hr, az)), dot(ay, hr)) / DEG : 0;
        R.add(fmtS("Pitch %.2f\xC2\xB0  heading %.2f\xC2\xB0  roll %.2f\xC2\xB0   w (%.3f, %.3f, %.3f) \xC2\xB0/s", pitch, hdg, roll, v->w.x / DEG, v->w.y / DEG,
                   v->w.z / DEG));
        R.add(fmtS("q (%.4f, %.4f, %.4f, %.4f)   SAS %s   RCS %s (%s%s)   ACS %.0f %%", v->q.w, v->q.x, v->q.y, v->q.z, tr(sasName(v->sas)).c_str(),
                   v->rcs ? "on" : "off", v->rcsSource == RcsSource::Auto ? "auto" : v->rcsSource == RcsSource::Monoprop ? "N2H4" : "battery",
                   v->rcsElectric ? ", electric" : "", v->tAcs * 100));
        R.gap();
        R.head("Air and heating");
        double rho = W.U.density(v->ref, alt), p = W.U.pressure(v->ref, alt);
        std::string air = fmtS("q %.3f kPa   Mach %.2f   AoA %.1f\xC2\xB0   density %.4g kg/m\xC2\xB3   pressure %.4g kPa", v->tDynP / 1000, v->tMach,
                               v->tAoA / DEG, rho, p / 1000);
        if (b.name == "Earth" && rho > 1e-9) air += fmtS("   air %.1f \xC2\xB0" "C", p / (rho * 287.05) - 273.15);
        R.add(air);
        if (v->tHeatFlux > 1) {
            // stagnation-point wall in radiative equilibrium (emissivity 0.85): q = e sigma T^4
            double Tw = std::pow(v->tHeatFlux / (0.85 * 5.670374e-8), 0.25);
            R.add(fmtS("Heat flux %.1f kW/m\xC2\xB2   wall ~%.0f \xC2\xB0" "C (stagnation point, radiative equilibrium)", v->tHeatFlux / 1000, Tw - 273.15),
                  Tw > 1600 ? col::warn : Color{225, 230, 238, 255});
        }
        std::string hot = v->tHottest >= 0 && v->tHottest < (int)v->comps.size() ? tr(v->comps[v->tHottest].name) : "-";
        R.add(fmtS("Hottest part %s   %.0f %% of its limit", hot.c_str(), v->tHeatMax * 100), v->tHeatMax > 0.5 ? col::bad : Color{225, 230, 238, 255});
        R.gap();
        R.head("Propulsion and power");
        double g = b.GM / len2(v->r);
        R.add(fmtS("Throttle %.0f %%   ignited %d   thrust %.1f kN   TWR %.2f   accel %.2f m/s\xC2\xB2%s", v->throttle * 100, (int)v->ignited, v->tThrust / 1000,
                   v->tThrust / (W.vesselMass(*v) * g), v->tAccel, v->tFlameout ? "   FLAMEOUT" : ""));
        R.add(fmtS("Charge %.2f / %.2f kWh   generation %.0f W   use %.0f W   star flux %.1f W/m\xC2\xB2%s", v->charge / 3.6e6, v->chargeCap / 3.6e6, v->tPowerGen,
                   v->tPowerUse, v->tSolarFlux, v->tInShadow ? " (shadow)" : ""));
        R.add(fmtS("Deploy: solar %.2f  legs %.2f  sail %.2f  chute %.2f   fairing %s", std::max(0.f, v->solarT), std::max(0.f, v->legsT),
                   std::max(0.f, v->sailT), v->chuteOpen, v->hasFairing ? "on" : "off"));
        if (app.screen == Screen::Flight) {
            double yaw, cp, dist;
            Vec3d tg;
            flightDebugGetView(yaw, cp, dist, tg);
            R.gap();
            R.add(fmtS("Camera yaw %.1f\xC2\xB0  pitch %.1f\xC2\xB0  distance %.1f m  fov %.0f\xC2\xB0   %s", yaw / DEG, cp / DEG, dist, flightDebugFovy(),
                       flightMapMode() ? "map view" : "flight view"));
        }
    }
    R.draw(size);
}
} // namespace dbg
