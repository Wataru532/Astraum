// Flight view, map view and flight HUD
#include <algorithm>
#include <functional>
#include <cstdio>
#include <random>
#include "app.hpp"
#include "screens.hpp"
#include "session.hpp"
#include "storage.hpp"
#include "audio.hpp"
#include "rl.hpp"
#include "render.hpp"
#include "gputime.hpp"
#include "nav.hpp"
#include "ui.hpp"
#include "tutorial.hpp"
#include "keys.hpp"
#include "terrain.hpp"
#include "features.hpp"
#include "achieve.hpp"
#include "navguide.hpp"
#include "mission.hpp"
#include <deque>
#include <set>

using namespace ui;

namespace {

// Fusion-style cameras. Flight: local east/north/up frame centred on the vessel. Map: ecliptic frame centred on the focus.
nav::OrbitCam fcam, mcam;
bool camInit = false;
bool mapMode = false;
int mapFocus = -1; // -1 = active vessel, otherwise body index
void setMapDist(double d) {
    mcam.animT = -1;
    mcam.target = {};
    mcam.dist = d;
}
void initCams() {
    if (camInit) return;
    camInit = true;
    fcam.fovy = 55;
    fcam.minDist = 2;
    fcam.maxDist = 2e6;
    fcam.homeYaw = -PI / 2 + 0.55;
    fcam.homePitch = 0.30;
    mcam.fovy = 50;
    mcam.minDist = 20;
    mcam.maxDist = 3e18;
    mcam.yaw = -1.2;
    mcam.pitch = 0.6;
    mcam.dist = 2e7;
    mcam.homeYaw = -1.2;
    mcam.homePitch = 0.6;
}
int selBody = -1;
bool hyperOpen = false, vesselsOpen = false, pauseOpen = false, showLabels = true, bodyListOpen = true, logOpen = false;
bool basesOpen = false, buildOpen = false; // the surface bases list, the build-a-base dialog
int nightVision = 0;      // V cycles: 0 off, 1 image intensifier (green), 2 bright (natural colours, the dark lifted)
bool achOpen = false;     // the achievements of this map
bool navOpen = true;      // the navigation panel (rendezvous / docking guidance to the target vessel)
float g_navBottom = 0;    // its lower edge this frame (the engine panel goes below it)
float g_navPanelRight = 0; // the right edge of the navball panel this frame (the help text starts beyond it)
int hyperSel = -1;
void vesselPickHover(int i, const Vessel& v, Vector2 p, float rad); // (switching vessels, below)
void openVesselMenu(const Vessel& v, Vector2 p);
extern int g_dbgMenuItem;

// ---- names of the surface features (with the labels on): every body seen large enough; a feature is shown
// when it is on the side facing the camera and big enough on the screen, the larger ones first, none on top
// of another. `project` maps a position (frame of body `frameBody`) to the screen
void featureLabels(World& W, int frameBody, const Vec3d& camPos, double pxPerRad, const std::function<bool(const Vec3d&, Vector2&)>& project) {
    const Universe& U = W.U;
    const auto& S = W.states;
    int nf = 0;
    const features::Feature* F = features::all(nf);
    struct L { Vector2 p; float size; int k; };
    std::vector<L> cand;
    int lastBody = -1;
    double R = 0, bodyPx = 0;
    Vec3d bc;
    Mat3d rot;
    for (int k = 0; k < nf; k++) {
        const features::Feature& f = F[k];
        int body = U.find(f.body);
        if (body < 0) continue;
        if (body != lastBody) {
            lastBody = body;
            R = U.bodies[body].radius;
            bc = U.relPos(S, body, frameBody);
            rot = U.bodyRot(body, W.t);
            double d = std::max(len(bc - camPos), R * 1.0001);
            bodyPx = R / d * pxPerRad;
        }
        if (bodyPx < 90) continue; // (the body is small on the screen)
        double la = f.lat * DEG, lo = f.lon * DEG;
        Vec3d dirB{std::cos(la) * std::cos(lo), std::cos(la) * std::sin(lo), std::sin(la)};
        Vec3d nW = rot * dirB;
        double h = terrain::map(body) ? terrain::height(body, dirB) : 0.0;
        Vec3d pos = bc + nW * (R + h), rel = pos - camPos;
        double dist = len(rel);
        if (dist < 1500 || dot(nW, rel) > -0.02 * dist) continue; // under the camera / on the far side
        if (body != frameBody) {
            // hidden behind the body the camera is at (Mars' names seen through Phobos)
            const Body& fb = U.bodies[frameBody];
            Vec3d dvec = rel / dist, oc = -camPos;
            double tt = dot(oc, dvec);
            if (!fb.isStar && tt > 0 && tt < dist && len(oc - dvec * tt) < fb.radius) continue;
        }
        float size = (float)(f.radiusKm * 1000 / dist * pxPerRad);
        if (size < 7 || f.radiusKm * 1000 < dist * 0.0008) continue; // too small to point at
        Vector2 sp;
        if (!project(pos, sp)) continue;
        cand.push_back({sp, size, k});
    }
    std::sort(cand.begin(), cand.end(), [](const L& a, const L& b) { return a.size > b.size; });
    std::vector<Vector2> placed;
    for (const L& c : cand) {
        bool clash = false;
        for (const Vector2& q : placed)
            if (std::fabs(q.x - c.p.x) < 90 && std::fabs(q.y - c.p.y) < 20) clash = true;
        if (clash || placed.size() >= 24) continue;
        placed.push_back(c.p);
        const features::Feature& f = F[c.k];
        Color col = f.kind == features::Mons || f.kind == features::Range || f.kind == features::Plateau ? Color{255, 214, 160, 230}
                    : f.kind == features::Site                                                          ? Color{150, 240, 180, 230}
                    : f.kind == features::Ice                                                           ? Color{220, 240, 255, 230}
                                                                                                        : Color{170, 210, 255, 230};
        if (f.kind == features::Mons || f.kind == features::Range) DrawTriangleLines({c.p.x, c.p.y - 5}, {c.p.x - 5, c.p.y + 4}, {c.p.x + 5, c.p.y + 4}, col);
        else if (f.kind == features::Site) DrawCircleV(c.p, 3, col);
        else DrawCircleLinesV(c.p, 5, col);
        std::string name = tr(f.name);
        Vector2 m = measure(name, 13);
        DrawRectangleRounded({c.p.x + 8, c.p.y - 9, m.x + 10, 18}, 0.4f, 6, Color{8, 12, 18, 150});
        text(name, c.p.x + 13, c.p.y - 8, 13, col);
    }
}
// surface bases: their names on the ground (flight view and map), a dome icon beside each
void baseLabels(World& W, int frameBody, const Vec3d& camPos, double pxPerRad, const std::function<bool(const Vec3d&, Vector2&)>& project) {
    const Universe& U = W.U;
    for (const SurfaceBase& b : W.bases) {
        const Body& body = U.bodies[b.body];
        Vec3d bc = U.relPos(W.states, b.body, frameBody);
        Mat3d rot = U.bodyRot(b.body, W.t);
        double d0 = std::max(len(bc - camPos), body.radius * 1.0001);
        if (body.radius / d0 * pxPerRad < 40) continue; // (the body is small on the screen)
        Vec3d dirB = baseCentreDir(b), nW = rot * dirB;
        Vec3d pos = bc + nW * (body.radius + terrain::height(b.body, dirB) + 40), rel = pos - camPos;
        double dist = len(rel);
        if (dist > 150 && dot(nW, rel) > -0.02 * dist) continue; // (on the far side)
        Vector2 sp;
        if (!project(pos, sp)) continue;
        bool pad = baseStatus(b).hasPad;
        Color col = pad ? Color{150, 240, 180, 240} : Color{220, 226, 236, 230};
        Vector2 m = measure(b.name, 13, true);
        DrawRectangleRounded({sp.x - 4, sp.y - 11, m.x + 34, 22}, 0.4f, 6, Color{8, 12, 18, 170});
        icon(Icon::Base, {sp.x, sp.y - 8, 16, 16}, col);
        text(b.name, sp.x + 22, sp.y - 9, 13, col, true);
    }
}
// test hook (autotest): the vessel pick sees the mouse on the first candidate and, if asked, a click
int g_dbgPick = 0; // 0 off, 1 hover the first candidate, 2 hover and click
bool g_dbgPickSeen = false;
Vector2 pickMouse(Vector2 candidate) { return g_dbgPick ? candidate : GetMousePosition(); }
bool pickClick() { return g_dbgPick == 2 || IsMouseButtonPressed(MOUSE_BUTTON_LEFT); }
double lastClickTime = 0;
int lastClickBody = -2;
std::string encounterText;

std::string fmt(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
}
std::string thousands(double v) {
    char b[48];
    std::snprintf(b, sizeof b, "%lld", (long long)std::llround(v));
    std::string s = b, o;
    int n = 0;
    for (int i = (int)s.size() - 1; i >= 0; i--) {
        o.insert(o.begin(), s[i]);
        if (++n % 3 == 0 && i > 0 && s[i - 1] != '-') o.insert(o.begin(), ',');
    }
    return o;
}
std::string powerStr(double w) {
    if (std::fabs(w) >= 1e6) return fmt("%.2f MW", w / 1e6);
    if (std::fabs(w) >= 1e3) return fmt("%.1f kW", w / 1e3);
    return fmt("%.0f W", w);
}
std::string massStr(double kg) { return kg >= 10000 ? fmt("%.1f t", kg / 1000.0) : thousands(kg) + " kg"; }

float smooth(float a, float b, float x) {
    float t = std::clamp((x - a) / (b - a), 0.f, 1.f);
    return t * t * (3 - 2 * t);
}

Vec3d poleOf(const Body& b) { return b.poleFrame.col(2) * (b.rotRate < 0 ? -1.0 : 1.0); }

void horizonFrame(const World& W, const Vessel& v, Vec3d& up, Vec3d& north, Vec3d& east) {
    up = norm(v.r);
    Vec3d pole = W.U.bodies[v.ref].poleFrame.col(2);
    north = pole - up * dot(pole, up);
    if (len(north) < 1e-6) north = anyPerp(up);
    north = norm(north);
    east = cross(north, up);
}

// hyperspace: while the drive spools up the vessel swings its nose (+Z) onto the direction of travel
// (shortest turn, no roll), and the camera comes round behind it
double hyperTurn(const World& W) {
    double s = clampd(W.hyper.phase / 1.4, 0, 1);
    return s * s * (3 - 2 * s);
}
Quatd hyperAttitude(const World& W, const Vessel& v) {
    Vec3d n0 = norm(v.q.rotate({0, 0, 1}));
    Vec3d d = norm(W.hyper.dir);
    double c = clampd(dot(n0, d), -1, 1);
    Vec3d ax = cross(n0, d);
    if (len(ax) < 1e-9) {
        if (c > 0) return v.q;
        ax = v.q.rotate({1, 0, 0}); // reversed: turn about the pitch axis
    }
    return (Quatd::axisAngle(ax, std::acos(c) * hyperTurn(W)) * v.q).normalized();
}
// direction of travel in the vessel's horizon frame (the flight camera's frame)
Vec3d hyperDirLocal(const World& W, const Vessel& v) {
    Vec3d up, north, east;
    horizonFrame(W, v, up, north, east);
    return {dot(W.hyper.dir, east), dot(W.hyper.dir, north), dot(W.hyper.dir, up)};
}

std::string situation(World& W, const Vessel& v) {
    const Body& b = W.U.bodies[v.ref];
    if (v.destroyed) return tr("Destroyed");
    if (v.landed) return trf("Landed on {0}", tr(b.name));
    double alt = len(v.r) - b.radius;
    if (b.atmTop > 0 && alt < b.atmTop) return trf("Flying in the atmosphere of {0}", tr(b.name));
    Conic c = stateToConic(v.r, v.v, b.GM);
    if (c.e >= 1 || (b.parent >= 0 && c.ap > b.soi)) return trf("Escaping {0}", tr(b.name));
    if (c.pe < b.radius) return trf("Sub-orbital over {0}", tr(b.name));
    return trf("Orbiting {0}", tr(b.name));
}


// ------------------------------------------------------------------ night vision
// The 3D view is rendered into an offscreen target and shown through an image-intensifier model:
// photocathode response weighted to red / near infrared, automatic brightness control from the mean
// scene luminance (top mip level), halo around bright sources, green P43 phosphor, shot noise that
// grows in the dark, and the darker edge of the tube.
const char* NV_FS = R"(#version 330
in vec2 fragTexCoord;
in vec4 fragColor;
uniform sampler2D texture0;
uniform vec2 res;
uniform float time;
out vec4 finalColor;
float hash(vec2 p) {
    p = fract(p * vec2(443.897, 441.423));
    p += dot(p, p.yx + 19.19);
    return fract((p.x + p.y) * p.x);
}
float lum(vec3 c) { return dot(c, vec3(0.50, 0.38, 0.12)); }
void main() {
    vec2 uv = fragTexCoord;
    float L = lum(texture(texture0, uv).rgb);
    // automatic brightness control: mean of the whole frame from the smallest mip level
    float mean = lum(textureLod(texture0, vec2(0.5), 12.0).rgb);
    float gain = clamp(0.20 / (mean + 0.006), 1.0, 16.0);
    // halo of bright points (the intensifier blooms around lights)
    vec2 px = 1.0 / res;
    float halo = 0.0;
    for (int i = 0; i < 8; i++) {
        float a = 6.2831853 * float(i) / 8.0;
        vec2 o = vec2(cos(a), sin(a));
        halo += lum(texture(texture0, uv + o * px * 3.0).rgb) + lum(texture(texture0, uv + o * px * 8.0).rgb) * 0.6;
    }
    halo /= 12.8;
    float a = L * gain;
    float v = a / (1.0 + 0.85 * a) * 1.2; // keeps gradation in the bright parts
    v += 0.35 * max(halo * gain - 1.1, 0.0);
    // shot noise: grainy where little light arrives, a little scintillation everywhere
    float n = hash(floor(uv * res / 1.5) + floor(time * 60.0) * vec2(17.0, 31.0)) - 0.5;
    v += n * (0.05 + 0.20 * (1.0 - clamp(v, 0.0, 1.0)));
    // tube edge
    vec2 d = (uv - 0.5) * vec2(res.x / res.y, 1.0);
    v *= mix(1.0, 0.55, smoothstep(0.45, 1.0, length(d)));
    v = clamp(v, 0.0, 1.2);
    vec3 phosphor = vec3(0.30, 1.00, 0.42);
    vec3 c = phosphor * v + vec3(0.6, 1.0, 0.65) * max(v - 0.85, 0.0); // saturated highlights turn whitish
    finalColor = vec4(pow(c, vec3(0.92)), 1.0);
}
)";
// bright mode (like a game's night-vision potion): everything seen in its own colours as if lit. The ground and the
// vessels get ambient light in their shaders (setNightLift, amb); this pass lifts what is still dark - exposure from
// the frame's mean, a curve that opens the shadows, the hue kept. Rendered to a half-float target: no banding.
const char* NVB_FS = R"(#version 330
in vec2 fragTexCoord;
in vec4 fragColor;
uniform sampler2D texture0;
uniform vec2 res;
uniform float time;
out vec4 finalColor;
float lum(vec3 c) { return dot(c, vec3(0.2126, 0.7152, 0.0722)); }
void main() {
    vec3 c = max(texture(texture0, fragTexCoord).rgb, vec3(0.0));
    float L = lum(c);
    float mean = lum(textureLod(texture0, vec2(0.5), 12.0).rgb);
    float gain = clamp(0.32 / (mean + 0.004), 1.0, 10.0);
    float a = L * gain;
    float Lo = 1.0 - exp(-2.4 * pow(a, 0.7)); // opens the shadows, rolls off the highlights
    vec3 o = L > 1e-6 ? c * (Lo / L) : vec3(0.0);
    o /= max(1.0, max(o.r, max(o.g, o.b))); // (the hue kept where a channel would clip)
    finalColor = vec4(o, 1.0);
}
)";
RenderTexture2D nvTarget{};
Shader nvShader{}, nvbShader{};
int nvRes = -1, nvTime = -1, nvbRes = -1, nvbTime = -1;
// a render target with a half-float colour buffer (dark tones keep their gradation when lifted) and a depth buffer
RenderTexture2D loadHalfFloatTarget(int w, int h) {
    RenderTexture2D t{};
    t.id = rlLoadFramebuffer();
    if (t.id == 0) return LoadRenderTexture(w, h);
    rlEnableFramebuffer(t.id);
    t.texture.id = rlLoadTexture(nullptr, w, h, PIXELFORMAT_UNCOMPRESSED_R16G16B16A16, 1);
    t.texture.width = w;
    t.texture.height = h;
    t.texture.format = PIXELFORMAT_UNCOMPRESSED_R16G16B16A16;
    t.texture.mipmaps = 1;
    t.depth.id = rlLoadTextureDepth(w, h, true);
    t.depth.width = w;
    t.depth.height = h;
    t.depth.format = 19;
    t.depth.mipmaps = 1;
    rlFramebufferAttach(t.id, t.texture.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
    rlFramebufferAttach(t.id, t.depth.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_RENDERBUFFER, 0);
    bool ok = rlFramebufferComplete(t.id);
    rlDisableFramebuffer();
    if (!ok || t.texture.id == 0) {
        UnloadRenderTexture(t);
        return LoadRenderTexture(w, h);
    }
    return t;
}

// V / the eye button: off -> image intensifier -> bright -> off
void cycleNightVision(World& W) {
    nightVision = (nightVision + 1) % 3;
    W.toast(nightVision == 0 ? "Night vision off" : nightVision == 1 ? "Night vision: image intensifier" : "Night vision: bright (natural colours)",
            Color{150, 255, 170, 255});
}
void nvBegin() {
    int w = GetRenderWidth(), h = GetRenderHeight();
    if (nvTarget.id == 0 || nvTarget.texture.width != w || nvTarget.texture.height != h) {
        if (nvTarget.id) UnloadRenderTexture(nvTarget);
        nvTarget = loadHalfFloatTarget(w, h);
        SetTextureFilter(nvTarget.texture, TEXTURE_FILTER_TRILINEAR);
    }
    if (nvShader.id == 0) {
        nvShader = LoadShaderFromMemory(nullptr, NV_FS);
        nvRes = GetShaderLocation(nvShader, "res");
        nvTime = GetShaderLocation(nvShader, "time");
    }
    if (nvbShader.id == 0) {
        nvbShader = LoadShaderFromMemory(nullptr, NVB_FS);
        nvbRes = GetShaderLocation(nvbShader, "res");
        nvbTime = GetShaderLocation(nvbShader, "time");
    }
    BeginTextureMode(nvTarget);
    rlScalef(ui::scale(), ui::scale(), 1.0f); // the 2D labels inside the 3D view are in UI units
}

void nvEnd() {
    EndTextureMode();
    ui::beginFrameScale(); // EndTextureMode reset the UI scale
    GenTextureMipmaps(&nvTarget.texture);
    SetTextureFilter(nvTarget.texture, TEXTURE_FILTER_TRILINEAR);
    Vector2 r{(float)nvTarget.texture.width, (float)nvTarget.texture.height};
    float tm = (float)std::fmod(GetTime(), 1000.0);
    Shader& sh = nightVision == 2 ? nvbShader : nvShader;
    SetShaderValue(sh, nightVision == 2 ? nvbRes : nvRes, &r, SHADER_UNIFORM_VEC2);
    SetShaderValue(sh, nightVision == 2 ? nvbTime : nvTime, &tm, SHADER_UNIFORM_FLOAT);
    // a shader that failed to compile falls back to raylib's default one (without its locations):
    // then the plain picture is shown rather than risking the draw
    bool ok = sh.id != 0 && sh.id != rlGetShaderIdDefault() && sh.locs != nullptr;
    if (ok) BeginShaderMode(sh);
    DrawTexturePro(nvTarget.texture, {0, 0, r.x, -r.y}, {0, 0, ui::SW(), ui::SH()}, {0, 0}, 0, WHITE);
    if (ok) EndShaderMode();
}

// ------------------------------------------------------------------ comets and the small-body belts
// a comet's look now: within ~5 AU of the Sun the ices sublimate - a coma and two tails, the blue ion tail
// straight away from the Sun, the yellowish dust tail curving back along the orbit; longer and brighter as
// it closes in (Halley in 1986: ~0.1 AU of tail at 0.6 AU from the Sun; Hale-Bopp with its 60 km nucleus more)
struct CometLook { double bright = 0, coma = 0, lenIon = 0, lenDust = 0; Vec3d anti, back; };
bool cometLook(const World& W, int i, CometLook& c) {
    const Body& b = W.U.bodies[i];
    if (!b.comet) return false;
    int star = W.U.hostStar(i);
    Vec3d r = W.U.relPos(W.states, i, star);
    double rAU = len(r) / AU;
    double act = std::clamp((5.0 - rAU) / 4.5, 0.0, 1.0);
    if (act <= 0) return false;
    act *= act;
    double g = 1.0 / std::max(rAU, 0.25), size = std::sqrt(std::max(b.radius, 500.0) / 5000.0);
    c.bright = std::clamp(act * (0.45 + 0.75 * std::min(g, 3.0)) * std::sqrt(size), 0.08, 1.0);
    c.coma = 6e7 * std::sqrt(act) * g * size;
    c.lenIon = AU * 0.10 * act * std::min(g, 4.0) * size;
    c.lenDust = 0.65 * c.lenIon;
    c.anti = norm(r);
    Vec3d v = W.U.relVel(W.states, i, star);
    c.back = len(v) > 0 ? -norm(v) : Vec3d{};
    return true;
}
// draws the coma and the tails; place(P, worldSize, pos, size) turns a camera-relative point [m] and a size [m]
// into the drawing space of the view (false: not drawn)
template <class F> void drawComet(const CometLook& c, const Vec3d& rel, F place) {
    Vector3 p;
    float s;
    auto glow = [&](const Vec3d& P, double w, Color col) {
        if (place(P, w, p, s)) gfx::drawGlow(p, s, col);
    };
    unsigned char a0 = (unsigned char)(220 * c.bright);
    glow(rel, c.coma * 2.5, Color{190, 225, 255, a0});
    glow(rel, c.coma * 0.6, Color{235, 245, 255, a0});
    // the tails: overlapping glows, their width growing along the tail (a few % of its length: a continuous band)
    const int N = 72;
    for (int k = 1; k <= N; k++) {
        double f = (double)k / N;
        // ion tail: straight, narrow, blue
        double wi = std::max(c.coma * 0.8, c.lenIon * (0.012 + 0.05 * f));
        glow(rel + c.anti * (c.lenIon * f), wi, Color{110, 160, 255, (unsigned char)(110 * c.bright * std::pow(1 - f, 1.1))});
        // dust tail: wider, curving back along the orbit
        Vec3d d = c.anti * f + (c.back - c.anti * dot(c.back, c.anti)) * (0.55 * f * f);
        double wd = std::max(c.coma * 1.2, c.lenDust * (0.02 + 0.16 * f));
        glow(rel + d * c.lenDust, wd, Color{255, 236, 200, (unsigned char)(90 * c.bright * std::pow(1 - f, 1.5))});
    }
}

// the main asteroid belt (2.1-3.3 AU, Kirkwood gaps at Jupiter's resonances), Jupiter's Trojans at L4 / L5 and the
// Kuiper belt (plutinos at 39.4 AU, the classical belt 42-48 AU, the scattered disc): random members with the
// distributions of the real populations, on Kepler orbits
struct BeltObj { double a, e, n, M0; Mat3d R; unsigned char kind; };
std::vector<BeltObj> g_belt;
void buildBelts(const Universe& U) {
    if (!g_belt.empty()) return;
    std::mt19937 rng(1801);
    std::uniform_real_distribution<double> U01(0, 1);
    std::normal_distribution<double> G(0, 1);
    auto rayleigh = [&](double s) { return s * std::sqrt(-2 * std::log(std::max(1e-12, 1 - U01(rng)))); };
    auto add = [&](double aAU, double e, double incDeg, double Om, double w, double M0, unsigned char kind) {
        BeltObj o;
        o.a = aAU * AU;
        o.e = std::clamp(e, 0.0, 0.95);
        o.n = std::sqrt(GM_SUN / (o.a * o.a * o.a));
        o.M0 = M0;
        o.R = rotZ(Om) * rotX(incDeg * DEG) * rotZ(w);
        o.kind = kind;
        g_belt.push_back(o);
    };
    const double gaps[] = {2.502, 2.825, 2.958, 3.279};
    while ((int)g_belt.size() < 9000) {
        double a = 2.1 + 1.2 * std::pow(U01(rng), 0.85);
        bool inGap = false;
        for (double gp : gaps) inGap |= std::fabs(a - gp) < 0.025;
        if (inGap) continue;
        add(a, rayleigh(0.09), rayleigh(7.5), U01(rng) * TAU, U01(rng) * TAU, U01(rng) * TAU, 0);
    }
    int jup = U.find("Jupiter");
    if (jup >= 0) {
        const Body& J = U.bodies[jup];
        // the mean longitude of Jupiter at J2000 (its orbit frame's node + argument of perihelion + M0)
        Vec3d px = J.orbitFrame * Vec3d{1, 0, 0};
        double varpi = std::atan2(px.y, px.x), L0 = varpi + J.M0;
        for (int k = 0; k < 1600; k++) {
            double side = k % 2 ? 1 : -1, Om = U01(rng) * TAU, w = U01(rng) * TAU;
            double lam = L0 + side * PI / 3 + G(rng) * 0.22;
            add(J.a / AU * (1 + G(rng) * 0.01), rayleigh(0.05), rayleigh(11), Om, w, lam - Om - w, 1);
        }
    }
    for (int k = 0; k < 6000; k++) {
        double r = U01(rng);
        double a, e, inc;
        if (r < 0.22) { a = 39.4 + G(rng) * 0.2; e = 0.1 + 0.2 * U01(rng); inc = rayleigh(8); }              // plutinos (3:2)
        else if (r < 0.62) { a = 42.0 + 5.5 * U01(rng); e = 0.1 * U01(rng); inc = rayleigh(2.2); }          // cold classical
        else if (r < 0.85) { a = 40.0 + 8.0 * U01(rng); e = 0.2 * U01(rng); inc = rayleigh(12); }           // hot classical
        else { a = 50 + 70 * std::pow(U01(rng), 2); e = 0.3 + 0.4 * U01(rng); inc = rayleigh(15); }        // scattered disc
        add(a, e, inc, U01(rng) * TAU, U01(rng) * TAU, U01(rng) * TAU, 2);
    }
}
Vec3d beltPos(const BeltObj& o, double t) {
    double M = std::fmod(o.M0 + o.n * t, TAU), E = M;
    for (int k = 0; k < 6; k++) E -= (E - o.e * std::sin(E) - M) / (1 - o.e * std::cos(E));
    return o.R * Vec3d{o.a * (std::cos(E) - o.e), o.a * std::sqrt(1 - o.e * o.e) * std::sin(E), 0};
}

// ------------------------------------------------------------------ flight rendering
void renderFlight(App& app, Vessel& av) {
    {
        // the sea's waves move in real time (not with the time warp), and stand still while paused
        static double seaT = 0;
        if (!pauseOpen) seaT += std::min((double)GetFrameTime(), 0.1);
        gfx::setSeaTime(seaT);
    }
    gputime::mark("pre");
    World& W = app.world;
    const Universe& U = W.U;
    const auto& S = W.states;
    int ref = av.ref;
    const Body& rb = U.bodies[ref];
    Vec3d up, north, east;
    horizonFrame(W, av, up, north, east);
    auto toW = [&](const Vec3d& l) { return east * l.x + north * l.y + up * l.z; };
    Vec3d lf, lr, lu;
    fcam.basis(lf, lr, lu);
    Vec3d camRel = av.r + toW(fcam.eye());
    // the ground below the camera (real or generated relief): the camera never goes under it, and the
    // near clipping plane is set from the height above it (not above the reference radius: on a 400 m
    // plateau a near plane from the datum altitude cut the ground right below the camera away)
    double groundUnder = 0;
    if (!rb.isStar && !rb.gasGiant && len(camRel) < rb.radius * 1.5 && terrain::ensure(ref))
        groundUnder = terrain::height(ref, U.bodyRot(ref, W.t).T() * norm(camRel));
    if (!rb.isStar && len(camRel) < rb.radius + groundUnder + 1.5) camRel = norm(camRel) * (rb.radius + groundUnder + 1.5);
    gfx::Cam cam;
    cam.fwd = norm(toW(lf));
    cam.up = norm(toW(lu));
    cam.fovy = (float)fcam.fovy;
    double camAlt = len(camRel) - rb.radius;
    double camAboveGround = std::max(0.0, camAlt - groundUnder);

    // sky
    Vec3d sunCam = W.sunDirFrom(ref, camRel);
    float day = 1, skyAmt = 0;
    Vector3 skyC{0, 0, 0};
    float fogK = 0;
    Vector3 fogC{0, 0, 0};
    if (rb.atmTop > 0 && camAlt < rb.atmTop) {
        double e = dot(norm(camRel), sunCam);
        day = smooth(-0.14f, 0.22f, (float)e);
        double dens = U.density(ref, camAlt) / rb.atmRho0;
        skyAmt = (float)std::clamp(std::pow(dens, 0.35), 0.0, 1.0);
        float k = skyAmt * day * 0.9f;
        skyC = {rb.atmColor.x * k, rb.atmColor.y * k, rb.atmColor.z * k};
        float thick = (float)std::clamp(rb.atmRho0 / 1.225, 0.05, 3.0);
        fogK = (float)(dens * thick / 20000.0); // aerial perspective: ~20 km e-folding at sea level
        fogC = {(rb.atmColor.x * 0.6f + 0.4f) * day * 0.92f, (rb.atmColor.y * 0.6f + 0.4f) * day * 0.92f, (rb.atmColor.z * 0.6f + 0.4f) * day * 0.92f};
    }
    ClearBackground(Color{(unsigned char)(std::min(1.f, skyC.x) * 255), (unsigned char)(std::min(1.f, skyC.y) * 255),
                          (unsigned char)(std::min(1.f, skyC.z) * 255), 255});
    // physically based mode: the sky from the scattering of sunlight in the air (also the limb seen from orbit)
    bool pbrSky = gfx::physicalShading() && rb.atmTop > 0 && !rb.isStar && camAlt < rb.radius * 3;
    if (pbrSky) gfx::drawPhysicalSky(U, ref, camRel, cam.fwd, cam.up, cam.fovy, sunCam);
    float starA = 1.0f - std::clamp(skyAmt * day * 3.0f, 0.f, 1.f);
    gfx::begin3D(cam, 0.1, 1000);
    gfx::drawStarfield(starA);
    gfx::end3D();
    gputime::mark("clear+stars");
    {
        // comets: their comae and tails on the sky (behind the planets drawn next)
        gfx::begin3D(cam, 1, 1e5);
        for (int i = 0; i < U.count(); i++) {
            CometLook cl;
            if (!cometLook(W, i, cl)) continue;
            Vec3d rel = U.relPos(S, i, ref) - camRel;
            drawComet(cl, rel, [&](const Vec3d& P, double w, Vector3& pos, float& sz) {
                double d = len(P);
                if (d < 1) return false;
                pos = toF(P / d * 1000.0);
                sz = (float)(w * 1000.0 / d);
                return sz > 0.02f;
            });
        }
        gfx::end3D();
    }
    if (!pbrSky && skyAmt * day > 0.02f && !rb.isStar) {
        // the sky brightens toward the horizon into the colour of the distant haze (the camera has no
        // roll against the local horizon, so the horizon is a horizontal line on screen)
        double Hs = ui::SH(), Ws = ui::SW();
        double f = Hs * 0.5 / std::tan(cam.fovy * 0.5 * DEG);
        double dip = std::acos(clampd((rb.radius + std::min(groundUnder, 0.0)) / len(camRel), -1, 1));
        double el = std::asin(clampd(dot(cam.fwd, norm(camRel)), -1, 1));
        double a = -dip - el;
        if (std::fabs(a) < 75 * DEG) {
            float yh = (float)(Hs * 0.5 - std::tan(a) * f);
            float band = (float)(Hs * 0.30);
            float k = skyAmt * day;
            Color hc{(unsigned char)(std::min(1.f, fogC.x) * 255), (unsigned char)(std::min(1.f, fogC.y) * 255),
                     (unsigned char)(std::min(1.f, fogC.z) * 255), (unsigned char)(215 * k)};
            Color top{hc.r, hc.g, hc.b, 0};
            DrawRectangleGradientV(0, (int)(yh - band), (int)Ws + 1, (int)band + 1, top, hc);
            DrawRectangle(0, (int)yh, (int)Ws + 1, (int)std::max(0.0, Hs - yh) + 1, hc);
        }
    }

    // bodies, far to near
    struct Item { int i; Vec3d rel; double d; };
    std::vector<Item> items;
    for (int i = 0; i < U.count(); i++) {
        Vec3d rel = U.relPos(S, i, ref) - camRel;
        items.push_back({i, rel, len(rel)});
    }
    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.d > b.d; });
    double H = ui::SH();
    double tanHalf = std::tan(cam.fovy * 0.5 * DEG);
    double pxPerRad = H * 0.5 / tanHalf;
    double worldPerPx = 2 * 1000 * tanHalf / H;
    // glare of a bright star: the disc keeps its true angular size, the glare (scattering in the eye /
    // lens) has a fixed angular extent and grows with the flux received here (Sun at 1 AU = 1)
    auto starGlare = [&](const Body& b, const Vec3d& dir, double d) {
        double flux = b.luminosity / (4 * PI * d * d);
        if (flux < 0.5) return;
        float g = (float)std::clamp(1.0 + 0.3 * std::log10(flux / 1361.0), 0.15, 1.25);
        auto angSize = [](double deg) { return (float)(2 * 1000 * std::tan(deg * 0.5 * DEG)); };
        Color c1{255, (unsigned char)(200 + b.colA.y * 55), (unsigned char)(150 + b.colA.z * 100), (unsigned char)(150 * g)};
        Color c2{(unsigned char)(b.colA.x * 255), (unsigned char)(b.colA.y * 235), (unsigned char)(b.colA.z * 210), (unsigned char)(60 * g)};
        Vector3 p = toF(dir * 1000.0);
        gfx::drawGlow(p, angSize(2.4 * g), c1);  // bright halo, ~4x the disc
        gfx::drawGlow(p, angSize(16.0 * g), c2); // wide glare
    };
    gputime::mark("sky");
    for (const Item& it : items) {
        const Body& b = U.bodies[it.i];
        Mat3d rot = U.bodyRot(it.i, W.t);
        Vec3d sunDir = b.isStar ? Vec3d{0, 0, 1} : norm(U.relPos(S, U.hostStar(it.i), it.i));
        bool isCap = it.i == ref && camAlt < 4 * b.radius;
        if (isCap) {
            gfx::clearDepth();
            double d = len(camRel);
            double hz = std::sqrt(std::max(d * d - b.radius * b.radius, 1.0));
            double Rh = b.radius * (1.0 + std::max(b.atmTop / b.radius * 0.55, 0.012));
            double farP = hz * 1.3 + 2 * std::sqrt(Rh * Rh - b.radius * b.radius) + 2000;
            double nearP = std::max(0.5, camAboveGround * 0.3);
            gfx::begin3D(cam, nearP, farP);
            gputime::mark("bodies");
            gfx::drawBodyCap(U, it.i, camRel, rot, sunDir, fogC, fogK, 1.0f);
            gputime::mark("cap");
            if (b.atmTop > 0 && camAlt > b.atmTop * 0.6)
                gfx::drawBodySphere(U, it.i, it.rel, b.radius, rot, sunDir, 1.0f, true, 8, false);
            if (b.ringOuter > 0) gfx::drawRings(U, it.i, it.rel, 1.0, rot, sunDir);
            gfx::end3D();
            gputime::mark("halo");
            continue;
        }
        double ang = b.radius / it.d;
        double px = ang * pxPerRad;
        Vec3d dir = it.rel / it.d;
        if (dot(dir, cam.fwd) < -0.2 && px < 50) continue;
        if (px < 1.5) {
            // point source
            float alpha;
            if (b.isStar) {
                double flux = b.luminosity / (4 * PI * it.d * it.d);
                alpha = (float)std::clamp((std::log10(flux) + 10.5) / 4.0, 0.12, 1.0);
            } else {
                alpha = (float)std::clamp((std::log10(ang) + 6.0) / 2.2, 0.0, 1.0) * starA;
            }
            if (alpha < 0.02f) continue;
            gfx::begin3D(cam, 1, 1e5);
            float size = (float)(worldPerPx * (b.isStar ? 9 + 8 * alpha : 6));
            Color c{(unsigned char)(b.colA.x * 255), (unsigned char)(b.colA.y * 255), (unsigned char)(b.colA.z * 255), (unsigned char)(alpha * 255)};
            if (b.isStar) c = Color{(unsigned char)(155 + b.colA.x * 100), (unsigned char)(155 + b.colA.y * 100), (unsigned char)(155 + b.colA.z * 100), c.a};
            gfx::drawGlow(toF(dir * 1000.0), size, c);
            if (b.isStar) starGlare(b, dir, it.d);
            gfx::end3D();
            continue;
        }
        gfx::clearDepth();
        double s = 1000.0 / it.d;
        gfx::begin3D(cam, 0.5, 1e5);
        gfx::drawBodySphere(U, it.i, it.rel * s, b.radius * s, rot, sunDir, 1.0f, true, 9);
        if (b.ringOuter > 0) gfx::drawRings(U, it.i, it.rel * s, s, rot, sunDir);
        if (b.isStar) {
            float g = (float)(b.radius * s * 7);
            gfx::drawGlow(toF(it.rel * s), g, Color{(unsigned char)(b.colA.x * 255), (unsigned char)(b.colA.y * 255), (unsigned char)(b.colA.z * 255), 200});
            starGlare(b, dir, it.d);
        }
        gfx::end3D();
    }

    gputime::mark("planets");
    // vessels
    gfx::clearDepth();
    gfx::begin3D(cam, 0.25, 80000);
    {
        // reflections / hemisphere light: sky above the local horizon, ground or space below
        Vec3d upL = norm(camRel);
        float sk = skyAmt * day;
        Vector3 sky{0.04f + rb.atmColor.x * 0.75f * sk, 0.04f + rb.atmColor.y * 0.75f * sk, 0.05f + rb.atmColor.z * 0.85f * sk};
        bool nearGround = !rb.isStar && camAlt < rb.radius * 0.5;
        Vector3 gnd = nearGround ? Vector3{0.10f + 0.14f * day, 0.10f + 0.13f * day, 0.09f + 0.11f * day} : Vector3{0.02f, 0.02f, 0.025f};
        gfx::setEnvironment(upL, sky, gnd);
        // the sunlight on the vessels, coloured by its way through the air (physically based mode)
        if (gfx::physicalShading()) gfx::setSunTint(gfx::sunTintAt(U, ref, camRel, sunCam));
    }
    // ground under the vessels: the pad deck / apron near the launch complex, the body's surface elsewhere
    bool padDrawn = false;
    Vec3d padC, padE, padN, padU;
    double padS = 0;
    {
        // launch complex at the selected site (body-fixed)
        int earth = U.find("Earth");
        if (ref == earth) {
            const LaunchSite& site = LAUNCH_SITES[std::clamp(W.settings.launchSite, 0, LAUNCH_SITE_COUNT - 1)];
            double lat = site.latDeg * DEG, lon = site.lonDeg * DEG;
            Vec3d upF{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
            Vec3d eastF = norm(cross(Vec3d{0, 0, 1}, upF)), northF = cross(upF, eastF);
            Mat3d R = U.bodyRot(earth, W.t);
            Vec3d padRel = R * (upF * rb.radius) - camRel;
            if (len(padRel) < 30000) {
                // swing arms retract once the vehicle on the pad is ignited
                static double armT = 0;
                bool onPad = av.landed && len(av.landPos - upF * len(av.landPos)) < 200;
                double target = onPad && !av.ignited ? 0.0 : 1.0;
                double rate = GetFrameTime() * 0.45;
                armT = target > armT ? std::min(target, armT + rate) : std::max(target, armT - rate * 3);
                gfx::PadSpec ps;
                ps.deckH = PAD_DECK_H;
                ps.rocketLen = W.padRocketLen;
                ps.rocketR = W.padRocketR;
                ps.armRetract = armT;
                Vec3d padLight{1, 0, 0};
                bool padShadow = false;
                W.solarFluxAt(ref, padRel + camRel, &padShadow, &padLight);
                // night at the pad (sun below the local horizon, with twilight): flood lights on; they also
                // light the vessels drawn after the pad in this pass
                double sunEl = dot(norm(padRel + camRel), W.sunDirFrom(ref, padRel + camRel));
                ps.night = 1.0f - smooth(-0.10f, 0.06f, (float)sunEl);
                Mat3d padRot = R * Mat3d::fromCols(eastF, northF, upF);
                if (ps.night > 0.01f) {
                    Vec3d lp[4], ld[4];
                    int n = gfx::padFloodlights(ps, lp, ld);
                    for (int q = 0; q < n; q++) {
                        lp[q] = padRel + padRot * lp[q];
                        ld[q] = padRot * ld[q];
                    }
                    float I = 2.4f * ps.night;
                    gfx::setSpotLights(n, lp, ld, Vector3{1.0f * I, 0.92f * I, 0.78f * I}, 95.0f);
                }
                float amb = (padShadow ? 0.05f : 0.14f) + 0.25f * skyAmt * day;
                if (nightVision == 2) amb = std::max(amb, 0.7f);
                gfx::drawLaunchPad(ps, padRot, padRel, padLight, padShadow ? 0.04f : 1.0f, {0, 0, 0}, amb);
                padDrawn = true;
                padC = padRel;
                padE = padRot.col(0);
                padN = padRot.col(1);
                padU = padRot.col(2);
                padS = gfx::padHardstandSize(ps);
            }
        }
    }
    double padDeck = PAD_DECK_H; // (the deck height of the pad drawn: an Earth site or a base's launch mount)
    // surface bases on this body near the camera: their buildings, and the launch pad rockets start from
    for (const SurfaceBase& sb : W.bases) {
        if (sb.body != ref || rb.isStar) continue;
        Vec3d eB, nB, uB;
        baseFrame(sb, eB, nB, uB);
        double g0 = terrain::height(ref, uB);
        Mat3d Rb = U.bodyRot(ref, W.t);
        Vec3d C = uB * (rb.radius + g0);
        Vec3d cRel = Rb * C - camRel;
        if (len(cRel) > 40000) continue;
        gfx::BaseDrawSpec spec;
        for (const BaseModule& m : sb.modules) {
            Vec3d d = baseDir(sb, rb.radius, m.x, m.y);
            Vec3d P = d * (rb.radius + terrain::height(ref, d)) - C;
            spec.mods.push_back({m.kind, m.variant, {dot(P, eB), dot(P, nB), dot(P, uB) - 0.25}, m.heading});
        }
        {
            // the local regolith: the body's colour, toned down toward a dusty grey (shells, berms, roads)
            auto ch = [](float c) { return (unsigned char)std::clamp((c * 0.55f + 0.42f * 0.45f) * 0.78f * 255.f, 25.f, 220.f); };
            spec.soil = Color{ch(rb.colA.x), ch(rb.colA.y), ch(rb.colA.z), 255};
        }
        Vec3d bLight{1, 0, 0};
        bool bShadow = false;
        W.solarFluxAt(ref, cRel + camRel, &bShadow, &bLight);
        double sunEl = dot(norm(cRel + camRel), W.sunDirFrom(ref, cRel + camRel));
        spec.night = 1.0f - smooth(-0.10f, 0.06f, (float)sunEl);
        double px = 0, py = 0;
        bool hasPad = padOffset(sb, px, py);
        Vec3d padDirF = hasPad ? baseDir(sb, rb.radius, px, py) : uB;
        bool onPad = hasPad && av.landed && av.ref == ref && len(av.landPos - padDirF * len(av.landPos)) < 200;
        spec.rocketOnPad = onPad && !av.ignited;
        spec.rocketLen = W.padRocketLen;
        spec.rocketR = W.padRocketR;
        Mat3d bRot = Rb * Mat3d::fromCols(eB, nB, uB);
        float amb = (bShadow ? 0.05f : 0.14f) + 0.25f * skyAmt * day;
        if (nightVision == 2) amb = std::max(amb, 0.7f);
        gfx::drawBase(spec, bRot, cRel, bLight, bShadow ? 0.04f : 1.0f, {0, 0, 0}, amb);
        if (hasPad && !padDrawn) {
            // vessel shadows on the launch mount of the base
            Vec3d Pp = padDirF * (rb.radius + terrain::height(ref, padDirF));
            padDrawn = true;
            padC = Rb * Pp - camRel;
            Vec3d eP = norm(cross(Vec3d{0, 0, 1}, padDirF) + Vec3d{1e-9, 0, 0}), nP = cross(padDirF, eP);
            padE = Rb * eP;
            padN = Rb * nP;
            padU = Rb * padDirF;
            padS = 2 * (std::max(0.5, W.padRocketR) * 1.25 + 1.6);
            padDeck = BASE_PAD_DECK_H;
        }
    }
    bool shadow = false;
    Vec3d lightDir{1, 0, 0};
    W.solarFluxAt(ref, av.r, &shadow, &lightDir);
    // vessel shadows on the ground: the strongest cue for the height above it
    if (!rb.isStar && !rb.gasGiant && !shadow) {
        for (auto& v : W.vessels) {
            if (v.destroyed || v.ref != ref || (v.hidden && &v != &av)) continue;
            Vec3d rel = v.r - camRel;
            if (len(rel) > 20000) continue;
            Vec3d u = norm(v.r);
            double sunUp = dot(lightDir, u);
            if (sunUp < 0.03) continue; // the sun is at or below the horizon
            // ground plane: the body's surface, or the pad deck / apron
            Vec3d g0 = u * rb.radius - camRel;
            double hAbove = len(v.r) - rb.radius;
            if (padDrawn) {
                Vec3d q = rel - padC;
                double x = dot(q, padE), y = dot(q, padN);
                double gh = (std::fabs(x) < padS / 2 && std::fabs(y) < padS / 2) ? padDeck : (std::hypot(x, y) < padS * 0.8 ? 0.4 : 0.0);
                if (std::hypot(x, y) < padS * 1.5) {
                    g0 = padC + padU * gh;
                    u = padU;
                    sunUp = dot(lightDir, u);
                    hAbove = dot(rel - g0, u);
                }
            }
            double Lv = std::max(1.0, v.length);
            if (hAbove > Lv * 2 + 1500 || hAbove < -Lv) continue;
            // the axis of the vessel from its lowest contact point to its top, projected along the sunlight
            Vec3d az = v.q.rotate({0, 0, 1});
            double zmin = -Lv * 0.5;
            if (!v.contacts.empty()) {
                zmin = 1e9;
                for (const Vec3d& c : v.contacts) zmin = std::min(zmin, c.z - v.mp.com.z);
            }
            auto onGround = [&](const Vec3d& p) { return p - lightDir * (dot(p - g0, u) / sunUp); };
            Vec3d a = onGround(rel + az * zmin), b = onGround(rel + az * (zmin + Lv));
            double blur = std::max(0.0, hAbove) * 0.0047; // penumbra of the solar disc
            float alpha = 0.55f * (1.0f - smooth((float)(Lv * 2 + 60), (float)(Lv * 2 + 1500), (float)hAbove)) *
                          smooth(0.03f, 0.18f, (float)sunUp) * std::min(1.0f, day + 0.3f);
            gfx::drawGroundShadow(a, b, u, std::max(0.4, v.maxR) * 1.05 + blur, std::max(0.3, v.maxR * 0.4) + blur, alpha);
        }
    }
    for (auto& v : W.vessels) {
        if (v.destroyed || v.ref != ref || (v.hidden && &v != &av)) continue;
        Vec3d rel = v.r - camRel;
        if (len(rel) > 60000) continue;
        gfx::CraftVisual cv;
        // deployed geometry plus the folded one: panels are drawn in between while they move
        cv.comps = &v.comps; // (v.shown is for the airflow: folded panels / gear are animated by the renderer)
        if (!v.stowedGeom.empty()) {
            cv.stowed = &v.stowedGeom;
            cv.solarT = v.solarT < 0 ? (v.solarDeployed ? 1.f : 0.f) : v.solarT;
        }
        cv.legsT = v.legsT < 0 ? (v.legsDeployed ? 1.f : 0.f) : v.legsT;
        cv.sailT = v.sailT < 0 ? (v.sailDeployed ? 1.f : 0.f) : v.sailT;
        std::vector<gfx::RcsPuff> puffs;
        for (size_t j = 0; j < v.rcsNoz.size() && j < v.rcsFire.size(); j++)
            if (v.rcsFire[j] > 0.03f)
                puffs.push_back({v.rcsNoz[j].pos, -v.rcsNoz[j].dir, v.rcsFire[j],
                                 (float)std::sqrt(v.rcsNoz[j].F * (v.rcsElectric ? RCS_ELEC_THRUST : 1.0) / 22.0), v.rcsElectric});
        cv.puffs = &puffs;
        cv.engine = &v.engLevel;
        cv.jetMode = &v.jetMode;
        // entry plasma: visible from ~50 kW/m² of stagnation heating, fully developed at ~2 MW/m²
        if (v.tHeatFlux > 5e4 && !v.landed) {
            cv.plasma = (float)std::clamp(std::log10(v.tHeatFlux / 5e4) / std::log10(40.0), 0.0, 1.0);
            Vec3d va = W.surfaceVel(v);
            if (len(va) > 1) cv.motionDir = v.q.conj().rotate(norm(va));
        }
        cv.prop = &v.prop;
        cv.heat = &v.heat;
        cv.activeStage = v.stage;
        cv.throttle = v.throttle;
        cv.ignited = v.ignited;
        cv.chute = v.chuteDeployed && !v.chuteCut && (!v.landed || v.chuteJettV > 0); // (a drag chute trails on the runway)
        cv.chuteOpen = (float)v.chuteOpen;
        cv.chuteReef = v.chuteT < v.chuteReefT ? (float)std::sqrt(v.chuteReefK) : 1.0f;
        cv.flapPos = (float)v.flapPos;
        cv.bayT = (float)v.bayT;
        if (cv.chute) {
            Vec3d va = W.surfaceVel(v);
            cv.chuteDir = len(va) > 0.5 ? v.q.conj().rotate(-norm(va)) : v.q.conj().rotate(norm(v.r));
        }
        cv.sail = v.sailDeployed;
        cv.legs = v.legsDeployed;
        cv.pressureAtm = U.pressure(ref, len(v.r) - rb.radius) / P_ATM;
        cv.infinite = W.settings.infiniteFuel;
        cv.hyperGlow = W.hyper.active && &v == &av;
        float amb = (shadow ? 0.05f : 0.14f) + 0.25f * skyAmt * day; // in the planet's shadow only faint sky / planet light
        if (nightVision == 2) amb = std::max(amb, 0.7f);
        Quatd vq = cv.hyperGlow ? hyperAttitude(W, v) : v.q;
        gfx::drawCraft(cv, vq.toMat(), rel, v.mp.com, lightDir, shadow ? 0.04f : 1.0f, {0, 0, 0}, amb);
    }
    gfx::end3D();
    gputime::mark("vessels");
    // the map's picture: the view as it is now, before labels and the HUD (not the night-vision image)
    if (!nightVision) grabWorldView();
    gfx::resetEnvironment();
    gfx::setSpotLights(0, nullptr, nullptr, {}, 1); // the flood lights belong to this pass only

    // other vessels nearby: names with the labels on; under the mouse a ring and a click switches to them
    if (!W.hyper.active && !(pauseOpen || hyperOpen || vesselsOpen || basesOpen || buildOpen || achOpen || app.settingsOpen)) {
        gfx::begin3D(cam, 0.25, 80000);
        Camera3D cv = gfx::current();
        gfx::end3D();
        Vector2 mouse = GetMousePosition();
        int best = -1;
        float bestD = 1e9f, bestR = 0;
        Vector2 bestP{};
        struct Lbl { int i; Vector2 p; float rad; };
        std::vector<Lbl> labels;
        for (int i = 0; i < (int)W.vessels.size(); i++) {
            const Vessel& v = W.vessels[i];
            if (i == W.active || v.destroyed || v.debris || v.hidden || v.ref != ref) continue;
            Vec3d rel = v.r - camRel;
            double d = len(rel);
            if (d < 1 || d > 60000 || dot(rel / d, cam.fwd) < 0.1) continue;
            Vector2 p = gfx::project(toF(rel), cv);
            float rad = (float)std::clamp(std::max(v.length * 0.5, v.maxR) / d * pxPerRad, 12.0, 400.0);
            if (showLabels) labels.push_back({i, p, std::min(rad, 46.f)});
            if (g_dbgPick && best < 0) mouse = pickMouse(p);
            float dm = Vector2Distance(mouse, p);
            if (dm < rad && dm < bestD) {
                best = i;
                bestD = dm;
                bestR = rad;
                bestP = p;
            }
        }
        bool hov = best >= 0 && (g_dbgPick || !mouseOverUI());
        for (const Lbl& l : labels)
            if (!(hov && l.i == best)) text(W.vessels[l.i].name, l.p.x + l.rad + 6, l.p.y - 8, 13, Color{200, 210, 225, 190});
        if (hov) vesselPickHover(best, W.vessels[best], bestP, bestR);
    }

    // labels
    if (showLabels) {
        gfx::begin3D(cam, 1, 1e5);
        Camera3D c3 = gfx::current();
        gfx::end3D();
        for (const Item& it : items) {
            const Body& b = U.bodies[it.i];
            if (it.i == ref && camAlt < 4 * b.radius) continue;
            Vec3d dir = it.rel / it.d;
            if (dot(dir, cam.fwd) < 0.2) continue;
            if (b.root != rb.root && !b.isStar) continue;
            // hidden behind the body we are near?
            Vec3d rc = -camRel;
            double tt = dot(rc, dir);
            if (it.i != ref && tt > 0 && tt < it.d && len(rc - dir * tt) < rb.radius) continue;
            Vector2 p = gfx::project(toF(dir * 1000.0), c3);
            // moons crowding their parent planet
            if (b.parent >= 0 && !U.bodies[b.parent].isStar && b.parent != ref) {
                Vec3d pd = norm(U.relPos(S, b.parent, ref) - camRel);
                if (std::acos(clampd(dot(pd, dir), -1, 1)) * pxPerRad < 24) continue;
            }
            double px = b.radius / it.d * pxPerRad;
            Color lc = b.isStar ? Color{255, 220, 150, 200} : Color{180, 200, 230, 170};
            double ringR = px > 2 ? px * 1.5 + 8 : std::max(6.0, px + 4);
            DrawCircleLinesV(p, (float)ringR, Color{lc.r, lc.g, lc.b, 90});
            text(b.name, p.x + (float)(ringR + 3), p.y - 8, 13, lc);
        }
        auto proj = [&](const Vec3d& pos, Vector2& out) {
            Vec3d rel = pos - camRel;
            double l = len(rel);
            if (l <= 0 || dot(rel / l, cam.fwd) < 0.15) return false;
            out = gfx::project(toF(rel / l * 1000.0), c3);
            return true;
        };
        featureLabels(W, ref, camRel, pxPerRad, proj);
        baseLabels(W, ref, camRel, pxPerRad, proj);
    }
    gputime::mark("labels");
}

// ------------------------------------------------------------------ switching vessels
// one way for all: the vessel list, a click on a vessel in the flight view or on the map. The switch is
// applied at the start of the next frame (the frame that saw the click keeps drawing the old vessel).
int pendingSwitch = -1;
int pendingUndock = -1; // a docked port of the active vessel to release (at the start of the next frame)
void switchToVessel(App& app, int i) {
    World& W = app.world;
    if (i < 0 || i >= (int)W.vessels.size() || i == W.active || W.vessels[i].destroyed) return;
    if (Vessel* cur = W.activeVessel()) cur->throttle = 0;
    if (W.warpTo > 0) W.setWarp(W.warpIndex); // a warp to a point of the old vessel's orbit ends
    W.active = i;
    W.vessels[i].hidden = false; // (the vessel flown is always shown)
    W.input = {};
    W.ctrl = {};
    vesselsOpen = false;
    mapFocus = -1;
    fcam.target = {};
    fcam.dist = std::max(W.vessels[i].length * 2.2, 14.0);
    W.toast(trf("Now controlling {0}", W.vessels[i].name), {150, 200, 255, 255});
    sfx::play(sfx::Ev::Click);
}
// a vessel under the mouse: ring, name and the hint; a left click switches to it
void vesselPickHover(int i, const Vessel& v, Vector2 p, float rad) {
    Color c = col::accent;
    rad = std::min(rad, 46.f); // (a long vessel close by: a ring of the marker size, not around all of it)
    DrawRing(p, rad + 3, rad + 5.5f, 0, 360, 48, c);
    DrawCircleLinesV(p, rad + 8, Color{c.r, c.g, c.b, 110});
    std::string hint = tr("Click for options");
    Vector2 n = measure(v.name, 14, true), h = measure(hint, 12);
    float w = std::max(n.x, h.x) + 20;
    Rectangle box{p.x + rad + 10, p.y - 22, w, 42};
    DrawRectangleRounded(box, 0.3f, 6, Color{10, 14, 20, 215});
    text(v.name, box.x + 10, box.y + 5, 14, col::text, true);
    text(hint, box.x + 10, box.y + 24, 12, col::dim);
    g_dbgPickSeen = true;
    if (pickClick()) {
        openVesselMenu(v, p);
        if (g_dbgPick == 2) {
            g_dbgPick = 0;
            g_dbgMenuItem = 0; // (the pick test goes on with "Switch")
        }
    }
}

// a new name for a vessel (also the name its modules get back when they undock)
void renameVessel(World& W, Vessel& v, const std::string& name) {
    std::string nm = cleanLineText(name, 48);
    if (nm.find_first_not_of(' ') == std::string::npos || nm == v.name) return;
    std::string old = v.name;
    v.name = nm;
    if (W.moduleNames.count(v.id)) W.moduleNames[v.id] = nm;
    W.toast(trf("{0} renamed to {1}", old, nm), col::good);
}
// debris: what nobody can fly any more - spent stages, fairing halves, anything without a command part
bool isDebris(const World& W, const Vessel& v, int i) { return !v.destroyed && i != W.active && (v.debris || !v.hasControl); }
// removes all debris; the vessel flown stays the active one. Returns how many went.
int deleteDebris(World& W) {
    std::vector<int> ids;
    for (int i = 0; i < (int)W.vessels.size(); i++)
        if (isDebris(W, W.vessels[i], i)) ids.push_back(W.vessels[i].id);
    if (ids.empty()) return 0;
    int actId = W.activeVessel() ? W.activeVessel()->id : -1;
    for (Vessel& o : W.vessels)
        if (std::find(ids.begin(), ids.end(), o.targetVessel) != ids.end()) o.targetVessel = -1;
    W.vessels.erase(std::remove_if(W.vessels.begin(), W.vessels.end(),
                                   [&](const Vessel& o) { return std::find(ids.begin(), ids.end(), o.id) != ids.end(); }),
                    W.vessels.end());
    W.active = -1;
    for (int i = 0; i < (int)W.vessels.size(); i++)
        if (W.vessels[i].id == actId) W.active = i;
    if (W.active < 0 && !W.vessels.empty()) W.active = 0;
    return (int)ids.size();
}

// ------------------------------------------------------------------ vessel menu (a click on a vessel)
// Switch to it, navigate to it (rendezvous and docking guidance), hide it, remove it. The menu is keyed by
// the vessel id; removals are applied at the start of the next frame (the frame holds Vessel references).
int menuVessel = -1;      // id of the vessel whose menu is open (-1: none)
Vector2 menuAt{};
int vmenuFrame = 0;        // frames since it opened (the opening click must not close it again)
bool menuConfirm = false; // "Remove" pressed once: the second press removes
bool menuRenaming = false; // "Rename": the menu shows a name field
std::string menuName;      // (the name being typed)
int pendingRemove = -1;   // id of a vessel to remove at the start of the next frame
// recovery: a vessel that flew and came down on Earth is offered for recovery (removed from the flight)
std::set<int> g_airborne; // ids of vessels that left the ground since they last stood on it
int recoverAsk = -1;      // the vessel the recovery question is about (-1: none)
int pendingRecover = -1;  // "Recover" pressed: removed at the start of the next frame
int g_dbgMenuItem = -1;   // tests: the menu item to "click" on the next draw
void openVesselMenu(const Vessel& v, Vector2 p) {
    menuVessel = v.id;
    menuAt = p;
    vmenuFrame = 0;
    menuConfirm = false;
    if (menuRenaming) ui::cancelEdit();
    menuRenaming = false;
}
void vesselMenu(App& app) {
    if (menuVessel < 0) {
        if (menuRenaming) { // (closed while typing: the field lets go of the keyboard)
            menuRenaming = false;
            ui::cancelEdit();
        }
        return;
    }
    World& W = app.world;
    int idx = -1;
    for (int i = 0; i < (int)W.vessels.size(); i++)
        if (W.vessels[i].id == menuVessel && !W.vessels[i].destroyed) idx = i;
    if (idx < 0 || idx == W.active || pauseOpen || hyperOpen || vesselsOpen || basesOpen || buildOpen || achOpen || app.settingsOpen || W.hyper.active) {
        menuVessel = -1;
        return;
    }
    Vessel& v = W.vessels[idx];
    Vessel* av = W.activeVessel();
    float w = 260, ih = 36;
    Rectangle r{menuAt.x + 16, menuAt.y - 24, w, 60 + 5 * (ih + 4) + 6};
    r.x = std::clamp(r.x, 8.f, ui::SW() - w - 8);
    r.y = std::clamp(r.y, 8.f, ui::SH() - r.height - 8);
    panel(r, Color{16, 21, 30, 248}, 10);
    text(fit(v.name, w - 24, 15, true), r.x + 12, r.y + 10, 15, col::text, true);
    text(fit(situation(W, v), w - 24, 12), r.x + 12, r.y + 33, 12, col::dim);
    float y = r.y + 58;
    int k = 0;
    auto item = [&](const std::string& label, Icon ic, bool active, bool enabled, Color ac = col::accent) {
        bool b = button({r.x + 8, y, w - 16, ih}, label, active, enabled, ic, ac) || (g_dbgMenuItem == k && enabled);
        y += ih + 4;
        k++;
        return b;
    };
    bool tg = av && av->targetVessel == v.id;
    if (menuRenaming) {
        // the name field in place of the items: Enter keeps it, Esc / Cancel leaves the old one
        text("New name", r.x + 12, y, 12, col::dim);
        textInput({r.x + 8, y + 18, w - 16, 36}, menuName, 48, 1110);
        std::string nm = cleanLineText(menuName, 48);
        bool ok = nm.find_first_not_of(' ') != std::string::npos;
        float by = y + 62;
        if (button({r.x + 8, by, (w - 24) / 2, ih}, "Cancel")) {
            menuRenaming = false;
            ui::cancelEdit();
        } else if (primaryButton({r.x + 16 + (w - 24) / 2, by, (w - 24) / 2, ih}, "Rename", Icon::Check, ok) ||
                   (ok && (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)))) {
            renameVessel(W, v, nm);
            menuRenaming = false;
            ui::cancelEdit();
        }
    } else if (item("Switch", Icon::Play, false, true)) {
        pendingSwitch = idx;
        menuVessel = -1;
    } else if (item("Navigate", Icon::Target, tg, av && !av->destroyed)) {
        av->targetVessel = v.id;
        navOpen = true;
        W.toast(trf("Navigating to {0}", v.name), {236, 120, 180, 255});
        menuVessel = -1;
    } else if (item("Rename", Icon::Pencil, false, true)) {
        menuRenaming = true;
        menuName = v.name;
        ui::focusText(1110, menuName);
    } else if (item("Hide vessel", Icon::EyeOff, false, true)) {
        v.hidden = true;
        W.toast(trf("{0} hidden  ·  show it again in the vessel list", v.name), col::dim);
        menuVessel = -1;
    } else if (item(menuConfirm ? "Click again to remove" : "Remove", Icon::Trash, menuConfirm, true, col::bad)) {
        if (menuConfirm || g_dbgMenuItem == 4) {
            pendingRemove = v.id;
            menuVessel = -1;
        } else menuConfirm = true;
    }
    g_dbgMenuItem = -1;
    // a click elsewhere closes it (Esc too: see the input handling)
    Vector2 m = GetMousePosition();
    if (menuVessel >= 0 && vmenuFrame > 0 && (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) &&
        !CheckCollisionPointRec(m, r))
        menuVessel = -1;
    vmenuFrame++;
}

// ------------------------------------------------------------------ achievements: banner and list
struct Banner { std::string id; double born; };
std::deque<Banner> banners;
void achievementBanners(App& app) {
    World& W = app.world;
    for (const std::string& id : W.newAchievements) {
        if (banners.size() < 8) banners.push_back({id, -1});
        sfx::play(sfx::Ev::Milestone);
    }
    W.newAchievements.clear();
    if (banners.empty()) return;
    Banner& b = banners.front();
    if (b.born < 0) b.born = GetTime();
    double age = GetTime() - b.born;
    if (age > 5.5) { banners.pop_front(); return; }
    const ach::Def* d = ach::find(W.U, b.id);
    if (!d) { banners.pop_front(); return; }
    float a = (float)std::clamp(std::min(age * 3, (5.5 - age) * 2), 0.0, 1.0);
    std::string t = ach::title(W.U, *d), ds = ach::description(W.U, *d);
    float w = std::max({measure(t, 18, true).x, measure(ds, 13).x, 260.f}) + 100;
    w = std::min(w, ui::SW() - 40);
    Rectangle r{(ui::SW() - w) / 2, ui::SH() * 0.2f, w, 78};
    DrawRectangleRounded(r, 0.3f, 8, Fade(Color{24, 20, 8, 240}, a));
    DrawRectangleRoundedLinesEx(r, 0.3f, 8, 1.5f, Fade(Color{250, 204, 21, 255}, a));
    icon(Icon::Trophy, {r.x + 18, r.y + 18, 42, 42}, Fade(Color{250, 204, 21, 255}, a));
    text("ACHIEVEMENT UNLOCKED", r.x + 76, r.y + 10, 11, Fade(Color{250, 204, 21, 255}, a), true);
    text(t, r.x + 76, r.y + 26, 18, Fade(col::text, a), true);
    text(fit(ds, r.width - 92, 13), r.x + 76, r.y + 52, 13, Fade(col::dim, a));
}

void achievementsModal(App& app) {
    if (!achOpen) return;
    World& W = app.world;
    const auto& L = ach::list(W.U);
    dim();
    float SW = ui::SW(), SH = ui::SH();
    Rectangle r{(SW - 940) / 2, (SH - 700) / 2, 940, 700};
    panel(r, Color{20, 26, 36, 252}, 14);
    icon(Icon::Trophy, {r.x + 24, r.y + 20, 26, 26}, Color{250, 204, 21, 255});
    text("Achievements", r.x + 60, r.y + 18, 24, col::text, true);
    int got = 0;
    for (const ach::Def& d : L) got += W.achievements.count(d.id) ? 1 : 0;
    std::string sub = trf("{0} of {1} earned", std::to_string(got), std::to_string((int)L.size()));
    if (!W.mapName.empty()) sub = W.mapName + "  ·  " + sub;
    text(sub, r.x + 60, r.y + 50, 14, col::dim);
    if (iconButton({r.x + r.width - 48, r.y + 16, 32, 32}, Icon::Close) || IsKeyPressed(KEY_ESCAPE)) { achOpen = false; return; }
    float ny = wrap(W.recordsOn ? "Only vessels launched from Earth count: not those started on another body or in orbit, and not those flown with infinite fuel or through hyperspace."
                                : "Tutorials earn no achievements: play a map (New game).",
                    r.x + 24, r.y + 78, r.width - 48, 13, col::faint);
    Rectangle lr{r.x + 16, ny + 10, r.width - 32, r.y + r.height - ny - 26};
    // rows: a header per body, then two cards per row
    struct Row { int header; int a, b; };
    std::vector<Row> rows;
    int lastBody = -2;
    std::vector<int> pend;
    auto flush = [&]() {
        for (size_t i = 0; i < pend.size(); i += 2) rows.push_back({-1, pend[i], i + 1 < pend.size() ? pend[i + 1] : -1});
        pend.clear();
    };
    int earth = W.U.find("Earth");
    for (int i = 0; i < (int)L.size(); i++) {
        int grp = L[i].kind == ach::Kind::Visit || L[i].kind == ach::Kind::LowOrbit || L[i].kind == ach::Kind::HighOrbit ||
                          L[i].kind == ach::Kind::Landing || L[i].kind == ach::Kind::ReturnFrom
                      ? L[i].body
                      : -1;
        if (L[i].body == earth) grp = -1;
        if (grp != lastBody) {
            flush();
            rows.push_back({grp, -1, -1});
            lastBody = grp;
        }
        pend.push_back(i);
    }
    flush();
    float contentH = 0;
    for (const Row& rw : rows) contentH += rw.a < 0 ? 34 : 72;
    float off = beginScroll(7400, lr, contentH);
    float y = lr.y - off;
    float cw = (lr.width - 8 - 10) / 2;
    Vector2 mouse = GetMousePosition();
    auto card = [&](int i, float x) {
        const ach::Def& d = L[i];
        auto it = W.achievements.find(d.id);
        bool have = it != W.achievements.end();
        Rectangle c{x, y, cw, 64};
        if (c.y + c.height < lr.y || c.y > lr.y + lr.height) return;
        DrawRectangleRounded(c, 0.15f, 6, have ? Color{40, 34, 14, 255} : Color{26, 32, 44, 255});
        if (have) DrawRectangleRoundedLinesEx(c, 0.15f, 6, 1, Color{150, 120, 30, 255});
        icon(Icon::Trophy, {c.x + 12, c.y + 16, 30, 30}, have ? Color{250, 204, 21, 255} : Color{60, 68, 84, 255});
        std::string t = ach::title(W.U, d), ds = ach::description(W.U, d);
        text(fit(t, c.width - 70, 15, true), c.x + 54, c.y + 7, 15, have ? col::text : col::dim, true);
        text(fit(ds, c.width - 66, 12), c.x + 54, c.y + 28, 12, have ? col::dim : col::faint);
        if (have) text(fit(formatDate(it->second.t) + "  ·  " + it->second.vessel, c.width - 66, 11), c.x + 54, c.y + 45, 11, Color{230, 190, 90, 255});
        if (inClip(mouse) && CheckCollisionPointRec(mouse, c)) tooltip(ds);
    };
    for (const Row& rw : rows) {
        if (rw.a < 0) {
            std::string h = rw.header < 0 ? i18n::tr("General") : i18n::tr(W.U.bodies[rw.header].name);
            int n = 0, g = 0;
            for (const ach::Def& d : L) {
                bool inGroup = (rw.header < 0) ? (d.body == earth || !(d.kind == ach::Kind::Visit || d.kind == ach::Kind::LowOrbit || d.kind == ach::Kind::HighOrbit ||
                                                                       d.kind == ach::Kind::Landing || d.kind == ach::Kind::ReturnFrom))
                                               : (d.body == rw.header && d.body != earth);
                if (inGroup) { n++; g += W.achievements.count(d.id) ? 1 : 0; }
            }
            text(h, lr.x + 6, y + 10, 14, col::accent, true);
            textR(std::to_string(g) + " / " + std::to_string(n), lr.x + lr.width - 18, y + 10, 13, col::dim);
            y += 34;
            continue;
        }
        card(rw.a, lr.x);
        if (rw.b >= 0) card(rw.b, lr.x + cw + 10);
        y += 72;
    }
    endScroll();
}

// ------------------------------------------------------------------ map
struct MapCtx {
    int anchor;
    Vec3d camRel;
    double k;
    gfx::Cam cam;
    Camera3D c3;
};

bool mapProject(const MapCtx& m, const Vec3d& relAnchor, Vector2& out) {
    Vec3d rel = relAnchor - m.camRel;
    double l = len(rel);
    if (l <= 0 || dot(rel / l, m.cam.fwd) < 0.05) return false;
    out = gfx::project(toF(rel / l * 1000.0), m.c3);
    return true;
}

// vessel marker on the map: a triangle whose tip follows the nose (pitch / yaw) as seen from the map
// camera. It shortens when the nose points towards or away from the viewer; a ring marks "nose towards
// the camera", a dot "nose away".
void vesselIcon(const MapCtx& m, const Vessel& v, const Vec3d& posRel, Vector2 p, Color c) {
    Vec3d nose = v.q.rotate({0, 0, 1}), side = v.q.rotate({1, 0, 0}), top = v.q.rotate({0, 1, 0});
    double eps = std::max(1.0, len(posRel - m.camRel) * 0.01);
    auto screenDelta = [&](const Vec3d& dir) {
        Vector2 q;
        if (!mapProject(m, posRel + dir * eps, q)) return Vector2{0, 0};
        return Vector2{q.x - p.x, q.y - p.y};
    };
    Vector2 dn = screenDelta(nose), ds = screenDelta(side), dt = screenDelta(top);
    float ln = std::hypot(dn.x, dn.y);
    float full = std::max({ln, std::hypot(ds.x, ds.y), std::hypot(dt.x, dt.y), 1e-4f});
    float f = ln / full; // 1 = nose across the screen, 0 = straight at / away from the camera
    Vector2 u = ln > 1e-5f ? Vector2{dn.x / ln, dn.y / ln} : Vector2{0, -1};
    Vector2 w{-u.y, u.x};
    float L = 6.0f + 6.0f * std::max(f, 0.25f), W = 5.5f;
    Vector2 tip{p.x + u.x * L * 0.62f, p.y + u.y * L * 0.62f};
    Vector2 b0{p.x - u.x * L * 0.38f + w.x * W, p.y - u.y * L * 0.38f + w.y * W};
    Vector2 b1{p.x - u.x * L * 0.38f - w.x * W, p.y - u.y * L * 0.38f - w.y * W};
    // raylib culls the other winding: keep the order of the original marker (tip, left, right on screen)
    if ((b0.x - tip.x) * (b1.y - tip.y) - (b0.y - tip.y) * (b1.x - tip.x) > 0) std::swap(b0, b1);
    DrawTriangle(tip, b0, b1, c);
    if (f < 0.5f) {
        Vec3d toCam = norm(m.camRel - posRel);
        if (dot(nose, toCam) > 0) DrawCircleLinesV(p, 9.5f, c);
        else DrawCircleV(p, 2.0f, Color{10, 14, 20, 255});
    }
}

// closest point of the drawn orbit (inside the sphere of influence) to the mouse, in true anomaly
bool pickOrbitPoint(const MapCtx& m, const Conic& c, const Vec3d& refRel, double soi, Vector2 mouse, double& nuOut, Vector2& pOut) {
    if (!c.valid || c.p <= 0) return false;
    double lo = -PI, hi = PI;
    bool closed = c.e < 1 && c.ap < soi;
    if (!closed) {
        // open or leaving the SOI: the arc between the two SOI crossings (and inside the asymptotes)
        double cs = (c.p / std::min(soi, 1e15) - 1) / std::max(c.e, 1e-9);
        double lim = std::acos(std::clamp(cs, -1.0, 1.0));
        if (c.e >= 1) lim = std::min(lim, std::acos(-1.0 / c.e) - 1e-3);
        lo = -lim;
        hi = lim;
    }
    auto dist2 = [&](double nu, Vector2& sp) {
        if (!mapProject(m, refRel + conicPoint(c, nu), sp)) return 1e30;
        double dx = sp.x - mouse.x, dy = sp.y - mouse.y;
        return dx * dx + dy * dy;
    };
    const int N = 720;
    double best = 1e30, bestNu = 0;
    Vector2 sp;
    for (int i = 0; i <= N; i++) {
        double nu = lo + (hi - lo) * i / N;
        double d = dist2(nu, sp);
        if (d < best) { best = d; bestNu = nu; }
    }
    // refine between the neighbouring samples
    double step = (hi - lo) / N, a = bestNu - step, b = bestNu + step;
    for (int k = 0; k < 30; k++) {
        double m1 = a + (b - a) / 3, m2 = b - (b - a) / 3;
        if (dist2(m1, sp) < dist2(m2, sp)) b = m2;
        else a = m1;
    }
    nuOut = (a + b) / 2;
    if (dist2(nuOut, pOut) > 9.0 * 9.0) return false;
    return true;
}

void lineStrip(const MapCtx& m, const std::vector<Vec3d>& pts, Color c) {
    rlBegin(RL_LINES);
    rlColor4ub(c.r, c.g, c.b, c.a);
    for (size_t i = 1; i < pts.size(); i++) {
        Vec3d a = (pts[i - 1] - m.camRel) * m.k, b = (pts[i] - m.camRel) * m.k;
        rlVertex3f((float)a.x, (float)a.y, (float)a.z);
        rlVertex3f((float)b.x, (float)b.y, (float)b.z);
    }
    rlEnd();
}

// Points of a curve f(u), u0..u1, as fine as the screen needs: every segment is split until its middle
// lies within ~0.35 px of the straight line (a short suborbital arc near the camera gets as many points
// as a whole orbit seen from afar). Parts behind the camera or far off the screen are not refined.
void adaptiveCurve(const MapCtx& m, const std::function<Vec3d(double)>& f, double u0, double u1, int n0, std::vector<Vec3d>& out) {
    struct P { double u; Vec3d p; Vector2 s; bool vis; };
    float SW = ui::SW() * ui::scale(), SH = ui::SH() * ui::scale();
    (void)SW; (void)SH;
    float W = ui::SW(), H = ui::SH(), mg = 60;
    auto mk = [&](double u) {
        P q;
        q.u = u;
        q.p = f(u);
        q.vis = mapProject(m, q.p, q.s);
        return q;
    };
    const size_t cap = 24000;
    std::function<void(const P&, const P&, int)> refine = [&](const P& a, const P& b, int d) {
        if (d >= 12 || out.size() >= cap) return;
        P c = mk((a.u + b.u) * 0.5);
        if (!a.vis && !b.vis && !c.vis) return;
        bool need = true;
        if (a.vis && b.vis && c.vis) {
            float minx = std::min({a.s.x, b.s.x, c.s.x}), maxx = std::max({a.s.x, b.s.x, c.s.x});
            float miny = std::min({a.s.y, b.s.y, c.s.y}), maxy = std::max({a.s.y, b.s.y, c.s.y});
            float ext = std::max(maxx - minx, maxy - miny);
            if ((maxx < -mg || minx > W + mg || maxy < -mg || miny > H + mg) && ext < std::max(W, H)) return; // off the screen
            float dx = c.s.x - (a.s.x + b.s.x) * 0.5f, dy = c.s.y - (a.s.y + b.s.y) * 0.5f;
            need = dx * dx + dy * dy > 0.35f * 0.35f || ext > 400;
        }
        if (!need) return;
        refine(a, c, d + 1);
        out.push_back(c.p);
        refine(c, b, d + 1);
    };
    P a = mk(u0);
    out.push_back(a.p);
    for (int i = 1; i <= n0; i++) {
        P b = mk(u0 + (u1 - u0) * i / n0);
        refine(a, b, 0);
        out.push_back(b.p);
        a = b;
    }
}

std::vector<Vec3d> conicPoints(const MapCtx& m, const Conic& c, double soi, int n, const Vec3d& offset) {
    std::vector<Vec3d> pts;
    if (!c.valid) return pts;
    double nu0 = c.nu, nu1;
    bool closed = c.e < 1 && c.ap < soi;
    if (closed) {
        nu1 = nu0 + TAU;
    } else {
        double cosExit = (c.p / soi - 1) / c.e;
        double nuExit = cosExit >= -1 && cosExit <= 1 ? std::acos(cosExit) : (c.e >= 1 ? std::acos(-1.0 / c.e) * 0.98 : PI);
        if (c.e >= 1) nuExit = std::min(nuExit, std::acos(-1.0 / c.e) * 0.995);
        nu1 = nuExit;
        if (nu0 > nu1) nu0 = nu0 - TAU;
        if (nu0 < -nuExit) nu0 = -nuExit;
    }
    adaptiveCurve(m, [&](double nu) { return offset + conicPoint(c, nu); }, nu0, nu1, n, pts);
    return pts;
}

void renderMap(App& app, Vessel* av) {
    World& W = app.world;
    const Universe& U = W.U;
    const auto& S = W.states;
    int anchor;
    Vec3d focusRel;
    if (mapFocus < 0 && av) {
        anchor = av->ref;
        focusRel = av->r;
    } else {
        anchor = mapFocus < 0 ? U.find("Earth") : mapFocus;
        focusRel = {};
    }
    Vec3d mf, mr, mu;
    mcam.basis(mf, mr, mu);
    MapCtx m;
    m.anchor = anchor;
    m.camRel = focusRel + mcam.eye();
    m.k = 100.0 / mcam.dist;
    m.cam.fwd = mf;
    m.cam.up = mu;
    m.cam.fovy = 50;
    ClearBackground(Color{5, 7, 12, 255});
    gfx::begin3D(m.cam, 0.1, 1000);
    gfx::drawStarfield(0.8f);
    gfx::end3D();

    gfx::begin3D(m.cam, 0.01, 1e8);
    m.c3 = gfx::current();
    double camDU = 100;
    (void)camDU;
    // orbits of bodies
    for (int i = 0; i < U.count(); i++) {
        const Body& b = U.bodies[i];
        if (b.parent < 0) continue;
        double aU = b.a * m.k;
        if (aU < 0.6) continue;
        Vec3d prel = U.relPos(S, b.parent, anchor);
        if (len(prel - m.camRel) * m.k > 5e6) continue;
        std::vector<Vec3d> pts;
        double bb = b.a * std::sqrt(1 - b.e * b.e);
        adaptiveCurve(m, [&](double E) { return prel + b.orbitFrame * Vec3d{b.a * (std::cos(E) - b.e), bb * std::sin(E), 0}; }, 0, TAU, 96, pts);
        Color c{(unsigned char)(80 + b.colA.x * 140), (unsigned char)(80 + b.colA.y * 140), (unsigned char)(90 + b.colA.z * 140), (unsigned char)(i == selBody ? 230 : 110)};
        lineStrip(m, pts, c);
    }
    rlDrawRenderBatchActive();
    // bodies
    for (int i = 0; i < U.count(); i++) {
        const Body& b = U.bodies[i];
        Vec3d rel = U.relPos(S, i, anchor) - m.camRel;
        double du = len(rel) * m.k;
        double ru = b.radius * m.k;
        if (ru / du < 0.0015 && !b.isStar) continue;
        if (du > 9e7) continue;
        Mat3d rot = U.bodyRot(i, W.t);
        Vec3d sunDir = b.isStar ? Vec3d{0, 0, 1} : norm(U.relPos(S, U.hostStar(i), i));
        if (ru / du >= 0.0015) {
            gfx::drawBodySphere(U, i, rel * m.k, ru, rot, sunDir, 1.0f, true, 7);
            if (b.ringOuter > 0) gfx::drawRings(U, i, rel * m.k, m.k, rot, sunDir);
        }
        if (b.isStar) {
            float g = (float)std::max(ru * 6, du * 0.025);
            gfx::drawGlow(toF(rel * m.k), g, Color{(unsigned char)(b.colA.x * 255), (unsigned char)(b.colA.y * 255), (unsigned char)(b.colA.z * 255), 230});
        }
    }
    {
        // the small-body belts around the Sun: fine dust of points, shown once the view takes in their scale
        buildBelts(U);
        int sun = U.find("Sun");
        Vec3d sunRel = U.relPos(S, sun, anchor) - m.camRel;
        double pxW = 2 * std::tan(m.cam.fovy * 0.5 * DEG) / ui::SH(); // (world size of a pixel per unit of distance)
        // (the inner belt fades out again when the whole Kuiper belt is in view: its points would pile up into a blot)
        float kMain = smooth((float)(0.08 * AU), (float)(0.6 * AU), (float)mcam.dist) * (1 - smooth((float)(25 * AU), (float)(70 * AU), (float)mcam.dist)), kKuiper = smooth((float)(1.5 * AU), (float)(8 * AU), (float)mcam.dist);
        if (app.showBelts && (kMain > 0.01f || kKuiper > 0.01f)) {
            BeginBlendMode(BLEND_ADDITIVE);
            rlDisableDepthMask();
            rlBegin(RL_QUADS);
            for (const BeltObj& o : g_belt) {
                float k = o.kind == 2 ? kKuiper : kMain;
                if (k < 0.01f) continue;
                Vec3d P = (sunRel + beltPos(o, W.t)) * m.k;
                double d = len(P);
                if (d < 1e-6 || dot(P / d, m.cam.fwd) < 0.05) continue;
                double h = d * pxW * 0.9;
                Vec3d a = mr * h, bu = mu * h;
                Color c = o.kind == 0 ? Color{190, 175, 150, (unsigned char)(150 * k)} : o.kind == 1 ? Color{200, 170, 140, (unsigned char)(160 * k)}
                                                                                   : Color{150, 175, 215, (unsigned char)(150 * k)};
                rlColor4ub(c.r, c.g, c.b, c.a);
                Vec3d q[4] = {P - a - bu, P + a - bu, P + a + bu, P - a + bu};
                for (const Vec3d& v : q) rlVertex3f((float)v.x, (float)v.y, (float)v.z);
            }
            rlEnd();
            rlDrawRenderBatchActive();
            rlEnableDepthMask();
            EndBlendMode();
        }
        // comets: coma and tails (at least a few pixels across, so that they can be found on the map)
        for (int i = 0; i < U.count(); i++) {
            CometLook cl;
            if (!cometLook(W, i, cl)) continue;
            Vec3d rel = U.relPos(S, i, anchor) - m.camRel;
            drawComet(cl, rel, [&](const Vec3d& P, double w, Vector3& pos, float& sz) {
                pos = toF(P * m.k);
                double d = len(P) * m.k;
                sz = (float)std::max(w * m.k, d * pxW * 3.0);
                return true;
            });
        }
    }
    // vessel trajectories
    encounterText.clear();
    for (int vi = 0; vi < (int)W.vessels.size(); vi++) {
        Vessel& v = W.vessels[vi];
        if (v.destroyed || v.landed || !v.showOrbit || (v.hidden && &v != av)) continue;
        const Body& rb = U.bodies[v.ref];
        Conic c = stateToConic(v.r, v.v, rb.GM);
        Vec3d refRel = U.relPos(S, v.ref, anchor);
        bool isAct = &v == av;
        double soi = rb.parent >= 0 ? rb.soi : 1e300;
        if (soi > 1e299) soi = std::max(len(v.r) * 50, 1e13);
        auto pts = conicPoints(m, c, soi, 128, refRel);
        lineStrip(m, pts, isAct ? Color{56, 189, 248, 255} : Color{150, 160, 180, 160});
        if (!isAct) continue;
        // encounters with moons / planets of the current SOI
        if (Encounter en; predictEncounter(U, v.ref, v.r, v.v, W.t, en)) {
            const Body& hb = U.bodies[en.body];
            Conic c2 = stateToConic(en.r, en.v, hb.GM);
            Vec3d hitNow = U.relPos(S, en.body, anchor);
            auto p2 = conicPoints(m, c2, hb.soi, 128, hitNow);
            lineStrip(m, p2, Color{251, 146, 60, 255});
            encounterText = trf("Encounter: {0} in {1}  ·  Pe {2}", tr(hb.name), formatTime(en.tEntry),
                                c2.pe < hb.radius ? tr("IMPACT") : formatDist(c2.pe - hb.radius));
        }
        // next patched conic after leaving the SOI
        if (rb.parent >= 0 && c.valid && !(c.e < 1 && c.ap < soi)) {
            double cosExit = (c.p / soi - 1) / c.e;
            if (std::fabs(cosExit) <= 1) {
                double tExit = timeToTrueAnomaly(c, std::acos(cosExit));
                if (tExit > 0) {
                    Vec3d r = v.r, vv = v.v;
                    keplerPropagate(r, vv, rb.GM, tExit);
                    std::vector<BodyState> S2;
                    U.computeStates(W.t + tExit, S2);
                    int par = rb.parent;
                    Vec3d rp = r + U.relPos(S2, v.ref, par), vp = vv + U.relVel(S2, v.ref, par);
                    const Body& pb = U.bodies[par];
                    Conic c2 = stateToConic(rp, vp, pb.GM);
                    double soi2 = pb.parent >= 0 ? pb.soi : std::max(len(rp) * 50, 1e13);
                    auto p2 = conicPoints(m, c2, soi2, 128, U.relPos(S, par, anchor));
                    lineStrip(m, p2, Color{167, 139, 250, 200});
                }
            }
        }
    }
    rlDrawRenderBatchActive();
    gfx::end3D();

    // 2D markers
    Vector2 mouse = GetMousePosition();
    int hoverBody = -1;
    for (int i = 0; i < U.count(); i++) {
        const Body& b = U.bodies[i];
        Vec3d rel = U.relPos(S, i, anchor);
        Vector2 p;
        if (!mapProject(m, rel, p)) continue;
        // hide labels of moons that crowd their parent
        if (b.parent >= 0) {
            Vector2 pp;
            if (mapProject(m, U.relPos(S, b.parent, anchor), pp) && Vector2Distance(p, pp) < 22 && i != selBody) continue;
        }
        bool hov = Vector2Distance(mouse, p) < 12 && !mouseOverUI();
        if (hov) hoverBody = i;
        Color c = b.isStar ? Color{255, 214, 140, 255} : Color{170, 196, 235, 255};
        if (i == selBody) c = col::accent;
        DrawCircleLinesV(p, hov ? 9.f : 6.f, c);
        if (av && av->target == i) DrawCircleLinesV(p, 12, Color{236, 72, 153, 255});
        text(b.name, p.x + 11, p.y - 9, 14, Color{c.r, c.g, c.b, (unsigned char)(hov || i == selBody ? 255 : 190)}, hov);
    }
    if (showLabels) {
        double pxm = ui::SH() * 0.5 / std::tan(m.cam.fovy * 0.5 * DEG);
        featureLabels(W, anchor, m.camRel, pxm, [&](const Vec3d& pos, Vector2& out) { return mapProject(m, pos, out); });
        baseLabels(W, anchor, m.camRel, pxm, [&](const Vec3d& pos, Vector2& out) { return mapProject(m, pos, out); });
    }
    // vessels
    int hoverVessel = -1;
    for (auto& v : W.vessels) {
        if (v.destroyed || (v.hidden && &v != av)) continue;
        Vector2 p;
        Vec3d posRel = U.relPos(S, v.ref, anchor) + v.r;
        if (!mapProject(m, posRel, p)) continue;
        bool isAct = &v == av;
        int vi = (int)(&v - W.vessels.data());
        if (!isAct && !v.debris && g_dbgPick && hoverVessel < 0) mouse = pickMouse(p);
        bool hov = false;
        if (!isAct && !v.debris && hoverVessel < 0 && Vector2Distance(mouse, p) < 12 && (g_dbgPick || !mouseOverUI()) && !W.hyper.active) {
            hoverVessel = vi;
            hov = true;
        }
        Color c = isAct ? col::accent : Color{200, 200, 210, 220};
        vesselIcon(m, v, posRel, p, c);
        if (hov) vesselPickHover(vi, v, p, 8); // (its box carries the name)
        else text(v.name, p.x + 10, p.y + 2, 13, c);
        if (isAct && !v.landed && v.showOrbit) {
            const Body& rb = U.bodies[v.ref];
            Conic c2 = stateToConic(v.r, v.v, rb.GM);
            Vec3d refRel = U.relPos(S, v.ref, anchor);
            Vector2 q;
            if (c2.valid && mapProject(m, refRel + conicPoint(c2, 0), q)) {
                DrawPoly(q, 4, 5, 45, Color{250, 204, 21, 255});
                text(trf("Pe {0}", formatDist(c2.pe - rb.radius)), q.x + 8, q.y + 2, 13, Color{250, 204, 21, 255});
            }
            if (c2.valid && c2.e < 1 && c2.ap < (rb.parent >= 0 ? rb.soi : 1e300) && mapProject(m, refRel + conicPoint(c2, PI), q)) {
                DrawPoly(q, 4, 5, 45, Color{96, 165, 250, 255});
                text(trf("Ap {0}", formatDist(c2.ap - rb.radius)), q.x + 8, q.y + 2, 13, Color{96, 165, 250, 255});
            }
        }
    }
    if (hoverVessel >= 0) hoverBody = -1; // the vessel marker is on top
    // "warp here": hover the active vessel's orbit, click to time-warp to that point
    if (av && !av->landed && !av->destroyed && av->showOrbit && hoverVessel < 0) {
        const Body& rb = U.bodies[av->ref];
        Conic c2 = stateToConic(av->r, av->v, rb.GM);
        Vec3d refRel = U.relPos(S, av->ref, anchor);
        double soi = rb.parent >= 0 ? rb.soi : 1e300;
        Color wc{125, 211, 252, 255};
        if (W.warpTo > 0 && c2.valid) {
            // target marker: where the vessel will be when the warp ends
            Vec3d r = av->r, vv = av->v;
            keplerPropagate(r, vv, rb.GM, W.warpTo - W.t);
            Vector2 q;
            if (mapProject(m, refRel + r, q)) {
                DrawCircleLinesV(q, 8, wc);
                DrawCircleV(q, 3, wc);
                text(trf("Warp target  ·  {0}", formatTime(W.warpTo - W.t)), q.x + 12, q.y - 20, 13, wc, true);
            }
        }
        double nu;
        Vector2 q;
        if (hoverBody < 0 && !mouseOverUI() && !nav::middleDragging() && pickOrbitPoint(m, c2, refRel, soi, mouse, nu, q)) {
            double dt = timeToTrueAnomaly(c2, nu);
            if (dt > 1.0) {
                DrawCircleLinesV(q, 7, wc);
                std::string label = trf("Warp here  ·  {0}", formatTime(dt));
                Vector2 lm = measure(label, 13, true);
                DrawRectangleRounded({q.x + 10, q.y + 8, lm.x + 14, 22}, 0.5f, 6, Color{10, 14, 20, 210});
                text(label, q.x + 17, q.y + 11, 13, wc, true);
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) W.startWarpTo(W.t + dt);
            }
        }
    }
    // picking
    if (hoverBody >= 0 && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !mouseOverUI()) {
        if (lastClickBody == hoverBody && GetTime() - lastClickTime < 0.4) {
            mapFocus = hoverBody;
            setMapDist(U.bodies[hoverBody].radius * (U.bodies[hoverBody].isStar ? 30 : 6));
        }
        selBody = hoverBody;
        lastClickBody = hoverBody;
        lastClickTime = GetTime();
    }
    if (hoverBody >= 0 && nav::takeRightClick() && av) {
        av->target = hoverBody;
        W.toast(trf("Target set: {0}", tr(U.bodies[hoverBody].name)), {236, 120, 180, 255});
    }
}

// ------------------------------------------------------------------ HUD pieces
void navMarker(Vector2 c, float rad, const Quatd& q, const Vec3d& dirW, int kind, Color col) {
    if (len(dirW) < 0.5) return;
    Vec3d l = q.conj().rotate(dirW);
    if (l.z <= 0) return;
    Vector2 p{c.x + (float)(-l.x) * rad, c.y - (float)l.y * rad};
    float s = 9;
    switch (kind) {
    case 0: // prograde
        DrawRing(p, s - 2, s, 0, 360, 24, col);
        DrawCircleV(p, 2, col);
        DrawLineEx({p.x, p.y - s}, {p.x, p.y - s - 6}, 2, col);
        DrawLineEx({p.x - s, p.y}, {p.x - s - 6, p.y}, 2, col);
        DrawLineEx({p.x + s, p.y}, {p.x + s + 6, p.y}, 2, col);
        break;
    case 1: // retrograde
        DrawRing(p, s - 2, s, 0, 360, 24, col);
        DrawLineEx({p.x - 5, p.y - 5}, {p.x + 5, p.y + 5}, 2, col);
        DrawLineEx({p.x + 5, p.y - 5}, {p.x - 5, p.y + 5}, 2, col);
        break;
    case 2: // normal
        DrawTriangleLines({p.x, p.y - s}, {p.x - s, p.y + s * 0.7f}, {p.x + s, p.y + s * 0.7f}, col);
        DrawCircleV(p, 2, col);
        break;
    case 3: // radial
        DrawRing(p, s - 2, s, 0, 360, 24, col);
        for (int k = 0; k < 4; k++) {
            float a = k * PI / 2 + PI / 4;
            DrawLineEx({p.x + cosf(a) * 3, p.y + sinf(a) * 3}, {p.x + cosf(a) * (s - 1), p.y + sinf(a) * (s - 1)}, 2, col);
        }
        break;
    case 4: // target
        DrawRectangleLinesEx({p.x - s, p.y - s, 2 * s, 2 * s}, 2, col);
        DrawCircleV(p, 2, col);
        break;
    case 5: // navigation burn: a diamond with a centre dot
        DrawLineEx({p.x, p.y - s - 2}, {p.x + s + 2, p.y}, 2, col);
        DrawLineEx({p.x + s + 2, p.y}, {p.x, p.y + s + 2}, 2, col);
        DrawLineEx({p.x, p.y + s + 2}, {p.x - s - 2, p.y}, 2, col);
        DrawLineEx({p.x - s - 2, p.y}, {p.x, p.y - s - 2}, 2, col);
        DrawCircleV(p, 2.5f, col);
        break;
    default: DrawCircleV(p, 4, col); break;
    }
}

// ------------------------------------------------------------------ engine selection
// The engines of what burns now, nozzle by nozzle: the pilot may run only some of them (a lighter vehicle
// on one of five, an engine out for the thrust line). A rear view of the nozzles (click one to switch it)
// and a list of the engine parts (a switch for all nozzles of a part).
bool enginesOpen = false;
struct Bell { int comp, bell; Vec3d p; double r; };
std::vector<Bell> engineBells(const Vessel& v) {
    std::vector<Bell> out;
    for (int i = 0; i < (int)v.comps.size(); i++) {
        const Component& k = v.comps[i];
        if (k.def < 0 || !(k.func == Func::Engine || k.func == Func::Jet) || !engineFiresNow(v, k)) continue;
        const PartDef& d = CATALOG.defs[k.def];
        Quatd R = k.rot();
        if (k.func == Func::Engine && d.kind == PK::Engine) {
            double bellR;
            auto pos = clusterLayout(std::max(1, k.cluster), std::max(k.dims.y, 0.1), bellR, std::min(d.exitDiam * 0.5, std::max(k.dims.x, 0.05)));
            for (int b = 0; b < (int)pos.size(); b++) out.push_back({i, b, k.pos + R.rotate(pos[b]), bellR});
        } else {
            Vec3d he = shapeHalfExtent(k.shape, k.dims);
            out.push_back({i, 0, k.pos, std::max(0.15, std::min(he.x, he.y))});
        }
    }
    return out;
}
// a lit solid motor cannot be shut down
bool engineLocked(const Vessel& v, int i) {
    const Component& k = v.comps[i];
    return k.def >= 0 && CATALOG.defs[k.def].kind == PK::SRB && v.ignited;
}
void setBell(Vessel& v, int i, int b, bool on) {
    if (engineLocked(v, i)) return;
    uint64_t bit = 1ull << b;
    v.comps[i].engOff = on ? (v.comps[i].engOff & ~bit) : (v.comps[i].engOff | bit);
}
void setAllBells(Vessel& v, int i, bool on) {
    if (engineLocked(v, i)) return;
    int n = engineCount(v.comps[i]);
    v.comps[i].engOff = on ? 0 : (n >= 64 ? ~0ull : ((1ull << n) - 1));
}
// test hook: switch nozzle `bell` of the n-th engine part of the burning stage (-1: all of its nozzles)
int g_dbgEngineHover = -1;
void enginesPanel(App& app, Vessel& av, Rectangle sr) {
    if (!enginesOpen) return;
    std::vector<Bell> bells = engineBells(av);
    std::vector<int> parts;
    for (const Bell& b : bells)
        if (parts.empty() || parts.back() != b.comp) parts.push_back(b.comp);
    float rowsH = std::min((float)parts.size(), 6.f) * 36;
    Rectangle r{sr.x - 352, g_navBottom > 0 ? std::max(sr.y, g_navBottom + 10) : sr.y, 340, 52 + 186 + 30 + rowsH + 56};
    if (parts.empty()) r.height = 120;
    panel(r);
    text("ENGINES", r.x + 16, r.y + 14, 13, col::dim, true);
    if (iconButton({r.x + r.width - 40, r.y + 8, 30, 28}, Icon::Close)) { enginesOpen = false; return; }
    if (parts.empty()) {
        wrap("No engines in the stage that burns now.", r.x + 16, r.y + 50, r.width - 32, 13, col::faint);
        return;
    }
    // rear view: craft -X to the right (looking forward from behind the nozzles), +Y up
    Rectangle dv{r.x + 12, r.y + 44, r.width - 24, 186};
    DrawRectangleRounded(dv, 0.06f, 6, Color{12, 16, 24, 255});
    text("View from the rear", dv.x + 8, dv.y + 6, 11, col::faint);
    double x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
    for (const Bell& b : bells) {
        x0 = std::min(x0, -b.p.x - b.r); x1 = std::max(x1, -b.p.x + b.r);
        y0 = std::min(y0, b.p.y - b.r); y1 = std::max(y1, b.p.y + b.r);
    }
    double span = std::max({x1 - x0, y1 - y0, 0.5});
    double s = std::min((dv.width - 40) / std::max(x1 - x0, span * 0.3), (dv.height - 40) / std::max(y1 - y0, span * 0.3));
    double cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    Vector2 mouse = GetMousePosition();
    int hover = -1;
    for (int k = 0; k < (int)bells.size(); k++) {
        const Bell& b = bells[k];
        Vector2 p{dv.x + dv.width / 2 + (float)((-b.p.x - cx) * s), dv.y + dv.height / 2 + 8 - (float)((b.p.y - cy) * s)};
        float rad = std::max(4.f, (float)(b.r * s * 0.92));
        bool on = !((av.comps[b.comp].engOff >> b.bell) & 1);
        bool burning = on && b.comp < (int)av.engLevel.size() && av.engLevel[b.comp] > 1e-3f;
        bool hov = (g_dbgEngineHover == k) || (!inputBlocked() && !ui::layerOccluded() && CheckCollisionPointCircle(mouse, p, rad + 2));
        if (hov) hover = k;
        Color fill = burning ? Color{251, 146, 60, 255} : on ? Color{56, 74, 100, 255} : Color{26, 30, 38, 255};
        DrawCircleV(p, rad, fill);
        DrawRing(p, rad - 1.5f, rad, 0, 360, 32, hov ? col::accent : on ? Color{150, 170, 200, 255} : Color{70, 76, 90, 255});
        if (!on) {
            float q = rad * 0.55f;
            DrawLineEx({p.x - q, p.y - q}, {p.x + q, p.y + q}, 2, Color{120, 90, 90, 255});
            DrawLineEx({p.x - q, p.y + q}, {p.x + q, p.y - q}, 2, Color{120, 90, 90, 255});
        }
    }
    if (hover >= 0) {
        const Bell& b = bells[hover];
        bool on = !((av.comps[b.comp].engOff >> b.bell) & 1);
        std::string nm = tr(CATALOG.defs[av.comps[b.comp].def].name);
        if (engineLocked(av, b.comp)) tooltip(nm + "  ·  " + tr("A burning solid motor cannot be shut down."));
        else tooltip(nm + "  ·  " + tr(on ? "Click to switch this engine off" : "Click to switch this engine on"));
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && g_dbgEngineHover < 0) {
            setBell(av, b.comp, b.bell, !on);
            sfx::play(sfx::Ev::Click);
        }
    }
    // the engine parts
    float y = dv.y + dv.height + 10;
    double Fall = 0, Fon = 0;
    for (int i : parts) {
        Fall += av.phys[i].thrustVac;
        Fon += av.phys[i].thrustVac * engineShare(av.comps[i]);
    }
    text(trf("Thrust (vac)  {0} of {1}", fmt("%.0f kN", Fon / 1000), fmt("%.0f kN", Fall / 1000)), r.x + 16, y, 13,
         Fon < Fall ? col::warn : col::dim);
    y += 24;
    Rectangle lr{r.x + 8, y, r.width - 16, rowsH};
    float off = beginScroll(7300, lr, parts.size() * 36.0f);
    float yy = lr.y - off;
    for (int i : parts) {
        const Component& k = av.comps[i];
        int n = engineCount(k), on = enginesOn(k);
        std::string nm = tr(CATALOG.defs[k.def].name);
        if (n > 1) nm += "  " + std::to_string(on) + "/" + std::to_string(n);
        bool any = on > 0;
        bool locked = engineLocked(av, i);
        Rectangle row{lr.x + 8, yy + 2, lr.width - 16, 32};
        if (locked) {
            text(fit(nm, row.width - 70, 14), row.x, row.y + 7, 14, col::faint);
            textR("lit", row.x + row.width, row.y + 8, 12, col::faint);
        } else if (toggle(row, fit(nm, row.width - 70, 14), any)) {
            setAllBells(av, i, any);
            sfx::play(sfx::Ev::Click);
        }
        yy += 36;
    }
    endScroll();
    if (button({r.x + 16, r.y + r.height - 46, r.width - 32, 34}, "All engines on", false, true, Icon::Flame)) {
        for (int i : parts) setAllBells(av, i, true);
    }
}

// ------------------------------------------------------------------ navigation panel (rendezvous and docking guidance)
navg::Guidance g_nav;
mission::Plan g_mission;    // the guidance toward the target body (mission panel)
int g_missionTarget = -1;   // the target body it was made for (a new target opens the panel again)
bool missionOpen = true;
const Vessel* navTargetOf(const World& W, const Vessel& a) {
    for (const Vessel& o : W.vessels)
        if (o.id == a.targetVessel && !o.destroyed) return &o;
    return nullptr;
}
// once per frame before the simulation step: SAS "Burn" steers along what the guidance asks for
void updateNav(App& app) {
    World& W = app.world;
    g_nav = {};
    Vessel* a = W.activeVessel();
    if (!a) return;
    const Vessel* t = navTargetOf(W, *a);
    if (!t || a->destroyed || W.hyper.active) {
        a->navBurn = {};
        // a target body: the mission guidance from the pad to an orbit around it (SAS "Burn" follows its burns)
        g_mission = {};
        if (!t && !a->destroyed && !W.hyper.active && a->target >= 0) {
            if (a->target != g_missionTarget) {
                g_missionTarget = a->target;
                missionOpen = true;
            }
            g_mission = mission::compute(W, *a, a->target);
            a->navBurn = g_mission.burn;
        } else g_missionTarget = -1;
        return;
    }
    g_nav = navg::compute(W, *a, *t, GetTime(), 1.0);
    a->navBurn = g_nav.burn;
}
void navPanel(App& app, Vessel& av) {
    g_navBottom = 0;
    World& W = app.world;
    const Vessel* t = navTargetOf(W, av);
    if (!navOpen || !t || g_nav.mode == navg::Mode::None) return;
    using navg::Mode;
    const navg::Guidance& g = g_nav;
    float SW = ui::SW();
    Rectangle r{SW - 704, 282, 340, 0}; // (beside the stage panel: clear of the time warp and the notifications)
    float w = r.width - 32;
    float hintH = 0;
    std::string hint;
    if (g.mode == Mode::Message) hint = g.message;
    else if (g.mode == Mode::Transfer) hint = tr("Turn along the orange burn marker (SAS Burn) and burn until the remaining velocity change is zero.");
    else if (g.mode == Mode::Coast) hint = tr("Coast to the intercept. Close by, the panel switches to the docking approach.");
    else hint = tr("Translate with RCS by the amounts shown; keep the closing speed low near the port.");
    {
        float lw = measure(hint, 13).x;
        hintH = std::ceil(std::max(lw, 1.f) / w * 1.08f) * 18 + 6;
    }
    // a burn is asked for but the engines are not lit (a start in orbit, after staging): say how to light them
    bool needIgnite = !av.ignited && (g.mode == Mode::Transfer || (g.mode == Mode::Proximity && len(g.dvCraft) > 3));
    float bodyH = g.mode == Mode::Message ? 0 : g.mode == Mode::Transfer ? 6 * 21 + 26 + 44 : g.mode == Mode::Coast ? 4 * 21 + 26 + 44 : 0;
    // proximity: the RCS lines
    struct Ax { double v; keys::A plus, minus; const char* pn; const char* mn; };
    Ax axes[3] = {{g.dvCraft.z, keys::A::TransFwd, keys::A::TransBack, "Forward", "Back"},
                  {g.dvCraft.y, keys::A::TransUp, keys::A::TransDown, "Up", "Down"},
                  {g.dvCraft.x, keys::A::TransLeft, keys::A::TransRight, "Left", "Right"}};
    int rcsLines = 0;
    double dvLen = len(g.dvCraft);
    if (g.mode == Mode::Proximity) {
        for (const Ax& a : axes) rcsLines += std::fabs(a.v) >= 0.02 ? 1 : 0;
        bodyH = 26 + 2 * 21 + std::max(rcsLines, 1) * 21 + (dvLen > 3 ? 21 : 0) + (g.alignDeg >= 0 ? 21 : 0) + 44;
    }
    if (needIgnite) bodyH += 21;
    r.height = 36 + bodyH + hintH + 8;
    panel(r);
    float y = r.y + 12;
    text("NAVIGATION", r.x + 16, y, 13, col::dim, true);
    textR(fit(t->name, r.width - 170, 13, true), r.x + r.width - 50, y, 13, col::accent, true);
    if (iconButton({r.x + r.width - 42, r.y + 6, 30, 28}, Icon::Close)) { navOpen = false; return; }
    y += 26;
    auto kv = [&](const std::string& k, const std::string& v, Color c = col::text) {
        text(k, r.x + 16, y, 13, col::dim);
        textR(v, r.x + r.width - 16, y, 13, c, true);
        y += 21;
    };
    auto sasBtn = [&](Rectangle b, SAS m, const char* label) {
        if (button(b, label, av.sas == m, true)) av.sas = av.sas == m ? SAS::Stability : m;
    };
    Color burnC{251, 146, 60, 255};
    if (g.mode == Mode::Transfer) {
        double dt = g.tBurn - W.t;
        if (dt > 1) text(trf("Rendezvous burn in {0}", formatTime(dt)), r.x + 16, y, 15, col::text, true);
        else text(trf("Burn now  ·  {0} left", fmt("%.1f m/s", g.dvTotal)), r.x + 16, y, 15, burnC, true);
        y += 26;
        kv("Prograde", fmt("%+.1f m/s", g.dvPro), burnC);
        kv("Normal", fmt("%+.1f m/s", g.dvNrm), burnC);
        kv("Radial out", fmt("%+.1f m/s", g.dvRad), burnC);
        kv("Total", fmt("%.1f m/s", g.dvTotal));
        kv("Intercept  ·  match speed", trf("in {0}", formatTime(g.tArrive - W.t)) + "  ·  " + fmt("%.1f m/s", g.dvMatch));
        kv("Relative inclination", fmt("%.2f°", g.relIncDeg));
        float bw = (r.width - 40) / 2;
        sasBtn({r.x + 16, y + 4, bw, 32}, SAS::Burn, "SAS Burn");
        if (button({r.x + 24 + bw, y + 4, bw, 32}, "Warp to burn", false, dt > 90 && W.warpTo < 0, Icon::Forward)) W.startWarpTo(g.tBurn - 60);
        y += 44;
    } else if (g.mode == Mode::Coast) {
        text("Coasting to the intercept", r.x + 16, y, 15, col::text, true);
        y += 26;
        kv("Intercept", trf("in {0}", formatTime(g.tArrive - W.t)));
        kv("Speed to match there", fmt("%.1f m/s", g.dvMatch));
        kv("Distance", formatDist(g.range));
        kv("Relative inclination", fmt("%.2f°", g.relIncDeg));
        if (button({r.x + 16, y + 4, r.width - 32, 32}, "Warp to the intercept", false, g.tArrive - W.t > 300 && W.warpTo < 0, Icon::Forward))
            W.startWarpTo(g.tArrive - 240);
        y += 44;
    } else if (g.mode == Mode::Proximity) {
        text(g.viaWaypoint ? "To the approach corridor" : "Docking approach", r.x + 16, y, 15, col::text, true);
        y += 26;
        kv("Distance  ·  closing", (g.range < 10000 ? fmt("%.1f m", g.range) : formatDist(g.range)) + "  ·  " + fmt("%+.2f m/s", g.closing),
           g.closing > 1.0 && g.range < 200 ? col::warn : col::text);
        kv("Along the port axis  ·  off it", fmt("%.1f m", g.along) + "  ·  " + fmt("%.1f m", g.lateral));
        if (rcsLines == 0) kv("RCS", tr("hold (no translation needed)"), col::good);
        for (const Ax& a : axes) {
            if (std::fabs(a.v) < 0.02) continue;
            bool pos = a.v > 0;
            std::string key = keys::label(pos ? a.plus : a.minus);
            kv(trf("RCS {0}", tr(pos ? a.pn : a.mn)) + (key.empty() ? std::string() : "  (" + key + ")"), fmt("%.2f m/s", std::fabs(a.v)), burnC);
        }
        if (dvLen > 3) kv("Large: main engine (SAS Burn)", fmt("%.1f m/s", dvLen), col::warn);
        if (g.alignDeg >= 0) kv("Port alignment", fmt("%.1f°", g.alignDeg), g.alignDeg < 12 ? col::good : col::warn);
        float bw = (r.width - 40) / 2;
        sasBtn({r.x + 16, y + 4, bw, 32}, SAS::Dock, "SAS Dock");
        sasBtn({r.x + 24 + bw, y + 4, bw, 32}, SAS::Burn, "SAS Burn");
        y += 44;
    }
    if (needIgnite) {
        std::string k = keys::label(keys::A::Stage);
        text(fit(trf("Engines are off: press {0} to ignite", k.empty() ? std::string("-") : k), w, 13, true), r.x + 16, y, 13, col::warn, true);
        y += 21;
    }
    wrap(hint, r.x + 16, y, w, 13, g.mode == Mode::Message ? col::warn : col::faint);
    g_navBottom = r.y + r.height;
}


// the mission panel: the way to an orbit around the target body, one step at a time (mission.hpp)
void missionPanel(App& app, Vessel& av) {
    World& W = app.world;
    const Universe& U = W.U;
    const mission::Plan& m = g_mission;
    if (!missionOpen || av.target < 0 || m.phase == mission::Phase::None) return;
    using mission::Phase;
    float SW = ui::SW();
    Rectangle r{SW - 704, 282, 340, 0};
    float w = r.width - 32;
    const Body& tb = U.bodies[av.target];
    std::string goal = m.goal >= 0 && m.goal != av.target ? U.bodies[m.goal].name : std::string();
    Color burnC{251, 146, 60, 255};
    // the lines of this phase: (key, value, colour); the title; the hint; the buttons
    struct Row { std::string k, v; Color c; };
    std::vector<Row> rows;
    std::string title, hint;
    Color titleC = col::text;
    auto kv = [&](const std::string& k, const std::string& v, Color c = col::text) { rows.push_back({k, v, c}); };
    double now = W.t;
    auto burnTitle = [&](const char* what) {
        double dt = m.tBurn - now;
        if (dt > 1) title = trf(what, formatTime(dt));
        else {
            title = trf("Burn now  ·  {0} left", fmt("%.0f m/s", m.dv));
            titleC = burnC;
        }
    };
    enum Btn { NONE = 0, B_WARP_LAUNCH = 1, B_ASCENT = 2, B_BURN = 4, B_WARP_BURN = 8, B_WARP_ENC = 16 };
    int btns = NONE;
    switch (m.phase) {
    case Phase::Message:
        title = tr("No plan");
        hint = tr(m.message);
        break;
    case Phase::Prelaunch:
        if (m.tLaunch < 0) title = tr("Launch any time");
        else if (m.tLaunch - now > 30) title = trf(m.transferWindow ? "Transfer window in {0}" : "Launch window in {0}", formatTime(m.tLaunch - now));
        else {
            title = tr("Launch now");
            titleC = col::good;
        }
        if (m.tLaunch >= 0) kv(m.transferWindow ? "Window opens" : "Lift-off", formatDate(m.tLaunch));
        kv("Parking orbit", formatDist(m.parkAlt));
        kv("Heading", tr("90° (due east)"));
        if (m.tArrive > 0) kv("Arrival (planned)", formatDate(m.tArrive));
        hint = m.transferWindow ? tr("Launch on the day of the window into the parking orbit; the panel then gives the departure burn.")
                                : tr("At the window, lift off with SAS Ascent: the orbit plane then passes under the target.");
        if (m.tLaunch - now > 60) btns |= B_WARP_LAUNCH;
        btns |= B_ASCENT;
        break;
    case Phase::Ascent:
        title = trf("Climbing to a {0} parking orbit", formatDist(m.parkAlt));
        kv("Apoapsis", m.ap >= 0 ? formatDist(m.ap) : tr("escape"));
        kv("Periapsis", formatDist(m.pe), m.pe < 0 ? col::warn : col::text);
        hint = tr("SAS Ascent flies the gravity turn, coasts to the apoapsis and makes the orbit insertion burn. Staging is yours.");
        btns |= B_ASCENT;
        break;
    case Phase::Departure:
        burnTitle(m.correction ? "Course correction in {0}" : "Departure burn in {0}");
        kv("Velocity change", fmt("%.0f m/s", m.dv), burnC);
        if (m.tWindow > now + 600) kv("Transfer window", trf("in {0}", formatTime(m.tWindow - now)));
        if (m.tArrive > 0) kv("Flight time", formatTime(m.tArrive - std::max(now, m.tBurn)));
        kv("Orbit", formatDist(m.pe) + "  ×  " + (m.ap >= 0 ? formatDist(m.ap) : tr("escape")));
        hint = tr("Turn along the orange burn marker (SAS Burn), light the engines at the time and burn until the velocity change left is zero.");
        btns |= B_BURN | B_WARP_BURN;
        break;
    case Phase::Transfer:
        if (m.encounter) {
            title = trf("Encounter with {0} in {1}", tr(U.bodies[m.goal].name), formatTime(m.tEncounter - now));
            kv("Periapsis there", m.encPe < 0 ? tr("IMPACT") : formatDist(m.encPe), m.impact ? col::warn : col::good);
            btns |= B_WARP_ENC;
        } else if (m.tBurn >= 0) {
            burnTitle("Course correction in {0}");
            kv("Velocity change", fmt("%.1f m/s", m.dv), burnC);
            if (m.tArrive > 0) kv("Arrival (planned)", formatDate(m.tArrive));
            btns |= B_BURN | B_WARP_BURN;
        } else title = tr("Coasting");
        hint = m.message.empty() ? tr("Coast on (time warp is safe); the panel gives the capture burn on arrival.") : tr(m.message);
        break;
    case Phase::Arrival:
        if (m.tBurn >= 0) {
            burnTitle("Capture burn in {0}");
            kv("Velocity change (retrograde)", fmt("%.0f m/s", m.dv), burnC);
        } else title = tr("Arriving");
        kv("Periapsis", formatDist(m.pe), m.impact ? col::warn : col::text);
        hint = m.message.empty() ? tr("Burn retrograde at the periapsis (SAS Burn) to stay in orbit.") : tr(m.message);
        if (m.tBurn >= 0) btns |= B_BURN | B_WARP_BURN;
        break;
    case Phase::InOrbit:
        title = trf("In orbit around {0}", tr(tb.name));
        titleC = col::good;
        kv("Apoapsis", formatDist(m.ap));
        kv("Periapsis", formatDist(m.pe));
        hint = tr("Mission complete: the orbit is above the air and the relief.");
        break;
    default: break;
    }
    if (m.phase != Phase::Message && !m.message.empty() && m.phase != Phase::Transfer && m.phase != Phase::Arrival) hint = tr(m.message) + "  " + hint;
    bool needIgnite = !av.ignited && (btns & B_BURN) && m.tBurn - now < 30;
    float lw = measure(hint, 13).x;
    float hintH = hint.empty() ? 0 : std::ceil(std::max(lw, 1.f) / w * 1.08f) * 18 + 6;
    int nBtnRows = (btns & (B_WARP_LAUNCH | B_ASCENT | B_BURN | B_WARP_BURN | B_WARP_ENC)) ? 1 : 0;
    r.height = 38 + 26 + (goal.empty() ? 0 : 20) + rows.size() * 21 + nBtnRows * 44 + (needIgnite ? 21 : 0) + hintH + 8;
    panel(r);
    float y = r.y + 12;
    text("MISSION", r.x + 16, y, 13, col::dim, true);
    textR(fit(tr(tb.name), r.width - 170, 13, true), r.x + r.width - 50, y, 13, col::accent, true);
    if (iconButton({r.x + r.width - 42, r.y + 6, 30, 28}, Icon::Close)) {
        missionOpen = false;
        return;
    }
    y += 26;
    text(fit(title, w, 15, true), r.x + 16, y, 15, titleC, true);
    y += 26;
    if (!goal.empty()) {
        text(fit(trf("On the way: {0} first", tr(goal)), w, 13), r.x + 16, y, 13, col::dim);
        y += 20;
    }
    for (const Row& rw : rows) {
        text(rw.k, r.x + 16, y, 13, col::dim);
        textR(rw.v, r.x + r.width - 16, y, 13, rw.c, true);
        y += 21;
    }
    if (nBtnRows) {
        float bw = (r.width - 40) / 2, bx = r.x + 16;
        auto slot = [&]() {
            Rectangle b{bx, y + 4, bw, 32};
            bx += bw + 8;
            return b;
        };
        if (btns & B_ASCENT) {
            if (button(slot(), "SAS Ascent", av.sas == SAS::Ascent, true, Icon::Rocket)) av.sas = av.sas == SAS::Ascent ? SAS::Stability : SAS::Ascent;
        }
        if (btns & B_WARP_LAUNCH) {
            if (button(slot(), "Warp to window", false, W.warpTo < 0, Icon::Forward)) W.startWarpTo(m.tLaunch - 20);
        }
        if (btns & B_BURN) {
            if (button(slot(), "SAS Burn", av.sas == SAS::Burn, true)) av.sas = av.sas == SAS::Burn ? SAS::Stability : SAS::Burn;
        }
        if (btns & B_WARP_BURN) {
            if (button(slot(), "Warp to burn", false, m.tBurn - now > 90 && W.warpTo < 0, Icon::Forward)) W.startWarpTo(m.tBurn - 60);
        }
        if (btns & B_WARP_ENC) {
            if (button({r.x + 16, y + 4, r.width - 32, 32}, "Warp to the encounter", false, m.tEncounter - now > 120 && W.warpTo < 0, Icon::Forward))
                W.startWarpTo(m.tEncounter + 30);
        }
        y += 44;
    }
    if (needIgnite) {
        std::string k = keys::label(keys::A::Stage);
        text(fit(trf("Engines are off: press {0} to ignite", k.empty() ? std::string("-") : k), w, 13, true), r.x + 16, y, 13, col::warn, true);
        y += 21;
    }
    if (!hint.empty()) wrap(hint, r.x + 16, y, w, 13, m.phase == Phase::Message || m.impact ? col::warn : col::faint);
    g_navBottom = r.y + r.height;
}

void drawHUD(App& app, Vessel& av) {
    World& W = app.world;
    const Universe& U = W.U;
    const Body& rb = U.bodies[av.ref];
    float SW = ui::SW(), SH = ui::SH();
    Vec3d up, north, east;
    horizonFrame(W, av, up, north, east);
    double alt = len(av.r) - rb.radius;
    Vec3d vs = W.surfaceVel(av);
    bool surfMode = alt < (rb.atmTop > 0 ? rb.atmTop * 0.5 : 0.02 * rb.radius);
    double mass = W.vesselMass(av);

    // every panel is a layer: a click brings the one under the mouse to the front (the engine selection over the
    // navball, the navigation panel over the stages...), and what it covers ignores the mouse
    enum { L_VESSEL = 100, L_WARP, L_ORBIT, L_RESOURCES, L_STAGES, L_TARGET, L_NAVBALL, L_NAV, L_ENGINES };
    // ---------- right: staging
    Rectangle sr{SW - 352, 282, 340, 0};
    std::vector<int> stagesLeft = vesselStages(av);
    std::vector<std::string> segInfo;
    for (int sg : stagesLeft) segInfo.push_back(stageEngineSummary(av, sg));
    int shownSegs = std::min((int)stagesLeft.size(), 5);
    sr.height = 46 + shownSegs * 46;
    if (mapMode && selBody >= 0) sr.x = -10000; // body info panel uses this space
    auto layVessel = [&]() {
        // ---------- top-left: vessel
        Rectangle tl{12, 12, 340, 104};
        panel(tl);
        text(fit(av.name, 300, 18, true), tl.x + 16, tl.y + 10, 18, col::text, true);
        text(fit(situation(W, av), 310, 14), tl.x + 16, tl.y + 36, 14, col::accent);
        text(trf("MET  {0}", formatTime(W.t - av.launchTime)), tl.x + 16, tl.y + 58, 14, col::dim);
        text(formatDate(W.t), tl.x + 16, tl.y + 78, 14, col::dim);
    };
    auto layWarp = [&]() {
        // ---------- top-centre: time warp
        Rectangle tw{SW / 2 - 170, 12, 340, 52};
        panel(tw);
        if (iconButton({tw.x + 10, tw.y + 10, 36, 32}, Icon::Backward, false, "Slower  ( , )")) W.setWarp(W.warpIndex - 1);
        if (iconButton({tw.x + 50, tw.y + 10, 36, 32}, Icon::Pause, W.warpIndex == 0, "Real time  ( / )")) W.setWarp(0);
        if (iconButton({tw.x + tw.width - 46, tw.y + 10, 36, 32}, Icon::Forward, false, "Faster  ( . )")) W.setWarp(W.warpIndex + 1);
        std::string wl = "x" + thousands(W.warpFactor());
        textC(wl, {tw.x + 90, tw.y + 6, tw.width - 140, 24}, 18, W.warpIndex > 0 ? col::warn : col::text, true);
        std::string sub = W.warpIndex > 3 ? "on rails / patched conics" : (W.warpIndex > 0 ? "physics warp" : "real time");
        if (!W.warpLimitReason.empty()) sub = trf("limited to x{0} ({1})", thousands(W.effectiveWarp), tr(W.warpLimitReason));
        if (W.warpTo > 0) sub = trf("to the selected point  ·  {0}", formatTime(W.warpTo - W.t));
        textC(sub, {tw.x + 90, tw.y + 28, tw.width - 140, 18}, 12, col::dim);
    };
    auto layOrbit = [&]() {
        // ---------- top-right: orbit
        Rectangle tr{SW - 352, 12, 340, 258};
        panel(tr);
        float y = tr.y + 12;
        auto kv = [&](const std::string& k, const std::string& v, Color c = col::text) {
            text(k, tr.x + 16, y, 14, col::dim);
            textR(v, tr.x + tr.width - 16, y, 14, c, true);
            y += 21;
        };
        text(surfMode ? "SURFACE" : "ORBIT", tr.x + 16, y, 13, col::dim, true);
        textR(rb.name, tr.x + tr.width - 16, y, 13, col::accent, true);
        y += 22;
        if (terrain::map(av.ref) && !av.landed && alt < 80e3) {
            // real relief below (Earth, Moon, Mars): the height above the ground matters for landing
            double gnd = terrain::height(av.ref, U.bodyRot(av.ref, W.t).T() * av.r);
            kv("Altitude  ·  above ground", formatDist(alt) + "  ·  " + formatDist(alt - gnd), alt - gnd < 1000 ? col::warn : col::text);
        } else kv("Altitude", formatDist(alt));
        kv("Orbital speed", formatSpeed(len(av.v)));
        kv("Surface speed", formatSpeed(len(vs)));
        kv("Vertical speed", formatSpeed(dot(vs, up)), dot(vs, up) < -5 && alt < 5000 ? col::bad : col::text);
        if (av.tDynP > 0) kv("Angle of attack  ·  Mach", fmt("%.1f°", av.tAoA / DEG) + "  ·  " + fmt("%.2f", av.tMach));
        if (!av.landed) {
            Conic c = stateToConic(av.r, av.v, rb.GM);
            double soi = rb.parent >= 0 ? rb.soi : 1e300;
            bool closed = c.e < 1 && c.ap < soi;
            kv("Apoapsis", closed ? trf("{0}  in {1}", formatDist(c.ap - rb.radius), formatTime(timeToTrueAnomaly(c, PI))) : i18n::tr("escape"));
            kv("Periapsis", timeToTrueAnomaly(c, 0) > 0 ? trf("{0}  in {1}", formatDist(c.pe - rb.radius), formatTime(timeToTrueAnomaly(c, 0))) : formatDist(c.pe - rb.radius),
               c.pe < rb.radius ? col::warn : col::text);
            double inc = std::acos(clampd(dot(c.H, poleOf(rb)), -1, 1)) / DEG;
            kv("Inclination  ·  ecc.", fmt("%.2f°", inc) + "  ·  " + fmt("%.4f", c.e));
            kv("Period", closed ? formatTime(c.period) : "--");
        } else {
            Mat3d R = U.bodyRot(av.ref, W.t);
            Vec3d f = norm(R.T() * av.r);
            kv("Latitude", fmt("%.3f°", std::asin(f.z) / DEG));
            kv("Longitude", fmt("%.3f°", std::atan2(f.y, f.x) / DEG));
            kv("Surface gravity", fmt("%.2f m/s²", surfaceGravity(rb)));
        }
        if (!encounterText.empty() && mapMode) {
            text(fit(encounterText, 320, 13), tr.x + 16, tr.y + tr.height - 18, 13, Color{251, 146, 60, 255});
        }
    };
    auto layResources = [&]() {
        // ---------- left: resources (the body list takes this place in map view)
        Rectangle lr{12, 128, 300, 0};
        if (mapMode) lr.x = -10000;
        // gather propellant in the bottom segment
        double amt[PROP_N], cap[PROP_N];
        double srbAmt, srbCap, Fvac, mdot;
        stageResources(av, amt, cap, srbAmt, srbCap, Fvac, mdot);
        int rows = 0;
        cap[(int)Prop::Monoprop] = amt[(int)Prop::Monoprop] = 0; // RCS propellant is shown for the whole vessel below
        for (int p = 1; p < PROP_N; p++)
            if (cap[p] > 0) rows++;
        if (srbCap > 0) rows++;
        if (av.monoCap > 0) rows++;
        bool hasRcs = !av.rcsNoz.empty();
        lr.height = 250 + (hasRcs ? 34 : 0) + rows * 38 + (av.tHeatFlux > 0 || av.tHeatMax > 0.01 ? 30 : 0) + (av.sailArea > 0 ? 22 : 0);
        panel(lr);
        float y = lr.y + 12;
        text("STAGE RESOURCES", lr.x + 16, y, 13, col::dim, true);
        y += 24;
        for (int p = 1; p < PROP_N; p++) {
            if (cap[p] <= 0) continue;
            text(propName((Prop)p), lr.x + 16, y, 13, col::text);
            textR(W.settings.infiniteFuel ? "infinite" : massStr(amt[p]), lr.x + lr.width - 16, y, 13, col::dim);
            bar({lr.x + 16, y + 19, lr.width - 32, 7}, W.settings.infiniteFuel ? 1.f : (float)(amt[p] / cap[p]), col::accent);
            y += 38;
        }
        if (srbCap > 0) {
            text("Solid propellant", lr.x + 16, y, 13, col::text);
            textR(massStr(srbAmt), lr.x + lr.width - 16, y, 13, col::dim);
            bar({lr.x + 16, y + 19, lr.width - 32, 7}, (float)(srbAmt / srbCap), col::warn);
            y += 38;
        }
        if (av.monoCap > 0) {
            text("Monopropellant (RCS)", lr.x + 16, y, 13, col::text);
            textR(W.settings.infiniteFuel ? "infinite" : massStr(av.monoAmt), lr.x + lr.width - 16, y, 13, col::dim);
            bar({lr.x + 16, y + 19, lr.width - 32, 7}, W.settings.infiniteFuel ? 1.f : (float)(av.monoAmt / av.monoCap), Color{214, 200, 150, 255});
            y += 38;
        }
        if (hasRcs) {
            // RCS energy source: hydrazine, battery (electric mode) or automatic
            text("RCS source", lr.x + 16, y + 6, 13, col::text);
            const char* names[3] = {"Auto", "N2H4", "Battery"};
            float cx = lr.x + lr.width - 16;
            for (int i = 2; i >= 0; i--) {
                float w = measure(names[i], 12, true).x + 16;
                cx -= w;
                if (chip({cx, y, w, 26}, names[i], (int)av.rcsSource == i)) av.rcsSource = (RcsSource)i;
                cx -= 4;
            }
            y += 34;
        }
        text("Electric charge", lr.x + 16, y, 13, col::text);
        textR(fmt("%.2f kWh", av.charge / 3.6e6), lr.x + lr.width - 16, y, 13, av.tPowerOut ? col::bad : col::dim);
        bar({lr.x + 16, y + 19, lr.width - 32, 7}, av.chargeCap > 0 ? (float)(av.charge / av.chargeCap) : 0.f, col::good);
        y += 34;
        text(av.tRcsPowerW > 1 ? trf("Gen {0}   Use {1}   RCS {2}", powerStr(av.tPowerGen), powerStr(av.tPowerUse), powerStr(av.tRcsPowerW))
                               : trf("Gen {0}   Use {1}", powerStr(av.tPowerGen), powerStr(av.tPowerUse)),
             lr.x + 16, y, 13, av.tRcsPowerW > 1 ? col::warn : col::dim);
        y += 20;
        text(trf(av.tInShadow ? "Stellar flux {0}  (shadow)" : "Stellar flux {0}", fmt("%.1f W/m²", av.tSolarFlux)), lr.x + 16, y, 13, col::dim);
        y += 24;
        double g = rb.GM / len2(av.r);
        double dvStage = 0;
        if (mdot > 0) {
            double usable = 0;
            for (int p = 1; p < PROP_N; p++) usable += amt[p];
            usable += srbAmt;
            double ispv = Fvac / (mdot * G0);
            if (W.settings.infiniteFuel) dvStage = INFINITY;
            else if (usable > 0 && mass > usable) dvStage = ispv * G0 * std::log(mass / (mass - usable));
        }
        text("Mass", lr.x + 16, y, 13, col::dim);
        textR(massStr(mass), lr.x + lr.width - 16, y, 13, col::text, true);
        y += 20;
        text("Thrust  ·  TWR", lr.x + 16, y, 13, col::dim);
        textR(fmt("%.1f kN", av.tThrust / 1000) + "  ·  " + fmt("%.2f", av.tThrust / (mass * g)), lr.x + lr.width - 16, y, 13, col::text, true);
        y += 20;
        text("Stage Δv (vac)", lr.x + 16, y, 13, col::dim);
        textR(std::isinf(dvStage) ? i18n::tr("infinite") : thousands(dvStage) + " m/s", lr.x + lr.width - 16, y, 13, col::accent, true);
        y += 20;
        text("Dynamic pressure", lr.x + 16, y, 13, col::dim);
        textR(fmt("%.2f kPa", av.tDynP / 1000), lr.x + lr.width - 16, y, 13, col::text, true);
        y += 20;
        if (av.sailArea > 0) {
            text("Sail force", lr.x + 16, y, 13, col::dim);
            textR(av.sailDeployed ? fmt("%.4f N", av.tSailForce) : "stowed (B)", lr.x + lr.width - 16, y, 13, col::text, true);
            y += 22;
        }
        if (av.tHeatFlux > 0 || av.tHeatMax > 0.01) {
            std::string hot = av.tHottest >= 0 && av.tHottest < (int)av.comps.size() ? "  ·  " + i18n::tr(av.comps[av.tHottest].name) : "";
            text(fit(trf("Heating {0}", fmt("%.0f kW/m²", av.tHeatFlux / 1000)) + hot, lr.width - 32, 13), lr.x + 16, y, 13,
                 av.tHeatMax > 0.5 ? col::bad : col::warn);
            bar({lr.x + 16, y + 18, lr.width - 32, 7}, (float)av.tHeatMax, col::bad);
            y += 30;
        }
    };
    auto layStages = [&]() {
        panel(sr);
        text("STAGES", sr.x + 16, sr.y + 12, 13, col::dim, true);
        textR("SPACE to stage", sr.x + sr.width - 16, sr.y + 12, 13, col::faint);
        for (int s = 0; s < shownSegs; s++) {
            Rectangle row{sr.x + 10, sr.y + 38 + s * 46, sr.width - 20, 42};
            bool act = s == 0;
            DrawRectangleRounded(row, 0.2f, 6, act ? Color{40, 32, 14, 255} : Color{24, 30, 42, 255});
            if (act) DrawRectangleRoundedLinesEx(row, 0.2f, 6, 1, av.ignited ? Color{251, 146, 60, 255} : col::warn);
            icon(segInfo[s].empty() ? Icon::Stage : Icon::Flame, {row.x + 10, row.y + 11, 20, 20}, act ? col::warn : col::dim);
            text(trf(act ? (av.ignited ? "Stage {0}  ·  burning" : "Stage {0}  ·  next") : "Stage {0}", std::to_string(s + 1)), row.x + 40, row.y + 4, 14, col::text, act);
            float infoW = row.width - 56;
            if (act) {
                // engine selection for what burns now (shows "on / all" once the pilot switched some off)
                int all = 0, on = 0;
                for (const Component& k : av.comps)
                    if (k.def >= 0 && (k.func == Func::Engine || k.func == Func::Jet) && engineFiresNow(av, k)) {
                        all += engineCount(k);
                        on += enginesOn(k);
                    }
                if (all > 0) {
                    std::string lb = on < all ? std::to_string(on) + "/" + std::to_string(all) : i18n::tr("Engines");
                    float bw = std::max(72.f, measure(lb, 13).x + 50);
                    if (button({row.x + row.width - bw - 6, row.y + 7, bw, 28}, lb, enginesOpen, true, Icon::Flame, on < all ? col::warn : col::accent)) {
                        enginesOpen = !enginesOpen;
                        if (enginesOpen) ui::layerRaise(L_ENGINES);
                    }
                    infoW -= bw + 8;
                }
            }
            text(fit(segInfo[s].empty() ? "payload / separation" : segInfo[s], infoW, 12), row.x + 40, row.y + 23, 12, col::dim);
        }
    };
    auto layTarget = [&]() {
        // ---------- right: rendezvous target and docked modules
        {
            const Vessel* tv = nullptr;
            for (const Vessel& o : W.vessels)
                if (o.id == av.targetVessel && !o.destroyed && o.ref == av.ref) tv = &o;
            std::vector<int> docked;
            for (int i = 0; i < (int)av.comps.size(); i++)
                if (av.comps[i].func == Func::Docking && av.comps[i].dockedTo != 0) docked.push_back(i);
            int nd = std::min((int)docked.size(), 4);
            if ((tv || nd > 0) && !(mapMode && selBody >= 0)) {
                Rectangle tp{sr.x, sr.y + sr.height + 10, sr.width, 0};
                tp.height = 14 + (tv ? 26 + 4 * 20 : 0) + (nd > 0 ? 26 + nd * 38 : 0);
                panel(tp);
                float y = tp.y + 12;
                if (tv) {
                    text("TARGET", tp.x + 16, y, 13, col::dim, true);
                    textR(fit(tv->name, tp.width - 140, 13, true), tp.x + tp.width - 52, y, 13, col::accent, true);
                    if (iconButton({tp.x + tp.width - 44, y - 6, 32, 26}, Icon::Target, navOpen, "Navigation")) {
                        navOpen = !navOpen;
                        if (navOpen) ui::layerRaise(L_NAV);
                    }
                    y += 26;
                    Vec3d d = tv->r - av.r, rv = tv->v - av.v;
                    double dist = len(d);
                    text("Distance", tp.x + 16, y, 13, col::dim);
                    textR(dist < 10000 ? fmt("%.1f m", dist) : formatDist(dist), tp.x + tp.width - 16, y, 13, col::text, true);
                    y += 20;
                    text("Relative speed", tp.x + 16, y, 13, col::dim);
                    textR(fmt("%.2f m/s", len(rv)), tp.x + tp.width - 16, y, 13, col::text, true);
                    y += 20;
                    text("Closing rate", tp.x + 16, y, 13, col::dim);
                    double cl = dist > 1e-3 ? -dot(rv, d / dist) : 0.0;
                    textR(fmt("%+.2f m/s", cl), tp.x + tp.width - 16, y, 13, cl > 1.0 && dist < 200 ? col::warn : col::text, true);
                    y += 20;
                    // the nearest pair of free ports: their gap and how far they are from facing each other
                    double best = 1e30, ang = 0;
                    for (int i = 0; i < (int)av.comps.size(); i++) {
                        if (av.comps[i].func != Func::Docking || av.comps[i].dockedTo) continue;
                        Vec3d pa, da;
                        W.portFrame(av, i, pa, da);
                        for (int j = 0; j < (int)tv->comps.size(); j++) {
                            if (tv->comps[j].func != Func::Docking || tv->comps[j].dockedTo) continue;
                            Vec3d pb, db;
                            W.portFrame(*tv, j, pb, db);
                            double g = len(pb - pa);
                            if (g < best) {
                                best = g;
                                ang = std::acos(clampd(-dot(da, db), -1, 1)) / DEG;
                            }
                        }
                    }
                    text("Docking ports", tp.x + 16, y, 13, col::dim);
                    textR(best < 1e29 ? (best < 1000 ? fmt("%.2f m", best) : formatDist(best)) + "  ·  " + fmt("%.0f°", ang) : std::string("-"), tp.x + tp.width - 16, y, 13,
                          best < 2 && ang < 12 ? col::good : col::text, true);
                    y += 26;
                }
                if (nd > 0) {
                    text("DOCKED", tp.x + 16, y, 13, col::dim, true);
                    y += 24;
                    for (int k = 0; k < nd; k++) {
                        int i = docked[k];
                        int partner = -1;
                        for (int j = 0; j < (int)av.comps.size(); j++)
                            if (av.comps[j].func == Func::Docking && av.comps[j].dockTag == av.comps[i].dockedTo) partner = j;
                        std::string nm = i18n::tr("module");
                        if (partner >= 0) {
                            auto it = W.moduleNames.find(av.comps[partner].origin);
                            auto it2 = W.moduleNames.find(av.comps[i].origin);
                            std::string a = it2 != W.moduleNames.end() ? it2->second : i18n::tr("module");
                            std::string b = it != W.moduleNames.end() ? it->second : i18n::tr("module");
                            nm = a + "  -  " + b;
                        }
                        text(fit(nm, tp.width - 130, 13), tp.x + 16, y + 8, 13, col::text);
                        if (button({tp.x + tp.width - 106, y + 2, 92, 30}, "Undock", false, true, Icon::Exit)) pendingUndock = i;
                        y += 38;
                    }
                }
            }
        }
    };
    auto layNavball = [&]() {
        // ---------- bottom centre: navball
        float nbR = 96;
        // a craft with flaps or a payload bay gets a second column of toggles left of the first: the panel grows to
        // hold it, and the whole cluster moves right (as far as the view cube allows) to stay clear of the toolbar
        const bool col2 = av.hasFlaps || av.bayComp >= 0;
        const float leftW = col2 ? 278.0f : 170.0f, rightW = 260.0f;
        Vector2 nc{SW / 2, SH - nbR - 22};
        {
            const float toolbarRight = 12 + 10 * 46 + 16 + 10;   // the bottom-left toolbar and a gap
            const float cubeLeft = SW - 250;                     // the view cube, bottom right
            float need = toolbarRight - (nc.x - nbR - leftW), room = cubeLeft - (nc.x + nbR + rightW);
            if (need > 0 && room > 0) nc.x += std::min(need, room);
        }
        g_navPanelRight = nc.x + nbR + rightW;
        Rectangle nbPanel{nc.x - nbR - leftW, nc.y - nbR - 76, 2 * nbR + leftW + rightW, 2 * nbR + 90};
        panel(nbPanel, Color{14, 18, 26, 250}, 16); // (nearly opaque: a panel it covers does not show through)
        Vec3d ax = av.q.rotate({1, 0, 0}), ay = av.q.rotate({0, 1, 0}), az = av.q.rotate({0, 0, 1});
        Mat3d M;
        Vec3d cols[3] = {ax, ay, az};
        for (int j = 0; j < 3; j++) {
            M.m[0][j] = dot(north, cols[j]);
            M.m[1][j] = dot(east, cols[j]);
            M.m[2][j] = dot(up, cols[j]);
        }
        gfx::drawNavball({nc.x - nbR, nc.y - nbR, 2 * nbR, 2 * nbR}, M);
        DrawRing(nc, nbR, nbR + 3, 0, 360, 64, Color{60, 70, 90, 255});
        // aircraft symbol
        DrawLineEx({nc.x - 30, nc.y}, {nc.x - 10, nc.y}, 3, col::warn);
        DrawLineEx({nc.x + 10, nc.y}, {nc.x + 30, nc.y}, 3, col::warn);
        DrawLineEx({nc.x - 10, nc.y}, {nc.x, nc.y + 8}, 3, col::warn);
        DrawLineEx({nc.x + 10, nc.y}, {nc.x, nc.y + 8}, 3, col::warn);
        Vec3d vel = surfMode ? vs : av.v;
        if (len(vel) > 0.1) {
            navMarker(nc, nbR, av.q, norm(vel), 0, Color{163, 230, 53, 255});
            navMarker(nc, nbR, av.q, -norm(vel), 1, Color{163, 230, 53, 255});
        }
        Vec3d nrm = cross(av.r, av.v);
        if (len(nrm) > 1e-6) {
            navMarker(nc, nbR, av.q, norm(nrm), 2, Color{192, 132, 252, 255});
            navMarker(nc, nbR, av.q, norm(cross(av.v, nrm)), 3, Color{34, 211, 238, 255});
        }
        if (av.target >= 0) {
            Vec3d td = W.targetDirection(av, SAS::Target);
            navMarker(nc, nbR, av.q, td, 4, Color{236, 72, 153, 255});
        }
        if (len(g_nav.burn) > 0.05) navMarker(nc, nbR, av.q, norm(g_nav.burn), 5, Color{251, 146, 60, 255}); // navigation burn
        double pitch = std::asin(clampd(dot(az, up), -1, 1)) / DEG;
        double hdg = wrapAngle(std::atan2(dot(az, east), dot(az, north))) / DEG;
        textC(formatSpeed(len(vel)), {nc.x - 120, nc.y - nbR - 72, 240, 26}, 20, col::text, true);
        textC(i18n::tr(surfMode ? "SURFACE" : "ORBIT") + "   " + i18n::tr("HDG") + fmt(" %03.0f°", hdg) + "   " + i18n::tr("PITCH") + fmt(" %+.1f°", pitch),
              {nc.x - 200, nc.y - nbR - 46, 400, 20}, 13, col::dim);
        // throttle
        Rectangle th{nc.x - nbR - 44, nc.y - nbR + 6, 18, 2 * nbR - 12};
        DrawRectangleRounded(th, 1.0f, 6, Color{36, 44, 60, 255});
        float tf = (float)av.throttle;
        DrawRectangleRounded({th.x, th.y + th.height * (1 - tf), th.width, std::max(th.width, th.height * tf)}, 1.0f, 6,
                             av.ignited ? Color{251, 146, 60, 255} : Color{120, 120, 130, 255});
        textC(fmt("%.0f%%", av.throttle * 100), {th.x - 30, th.y + th.height + 2, 78, 18}, 13, col::text, true);
        text("THR", th.x - 2, th.y - 20, 12, col::dim, true);
        if (av.tFlameout && av.throttle > 0 && !W.settings.infiniteFuel) textC("FLAMEOUT", {th.x - 60, th.y - 40, 140, 18}, 13, col::bad, true);
        if (av.tPowerOut) textC("NO POWER", {nc.x - 70, nc.y + nbR - 30, 140, 18}, 14, col::bad, true);
        // SAS
        const SAS modes[] = {SAS::Stability, SAS::Prograde, SAS::Retrograde, SAS::Normal, SAS::AntiNormal,
                             SAS::RadialOut, SAS::RadialIn, SAS::Target, SAS::AntiTarget, SAS::Sun};
        float sx = nc.x + nbR + 24, sy = nc.y - nbR + 4;
        text("SAS", sx, sy - 22, 12, col::dim, true);
        for (int i = 0; i < 10; i++) {
            Rectangle b{sx + (i % 2) * 114, sy + (i / 2) * 27, 110, 24};
            if (button(b, sasName(modes[i]), av.sas == modes[i])) av.sas = av.sas == modes[i] ? SAS::Off : modes[i];
        }
        {
            // ascent guidance: flies the gravity turn into a parking orbit (attitude only)
            Rectangle b{sx, sy + 5 * 27, 224, 24};
            if (button(b, "Ascent guidance", av.sas == SAS::Ascent, true, Icon::Rocket)) av.sas = av.sas == SAS::Ascent ? SAS::Stability : SAS::Ascent;
            if (CheckCollisionPointRec(GetMousePosition(), b))
                tooltip(trf("Flies from the pad into a {0} orbit: gravity turn, coast, orbit insertion. You only drop the empty stages.",
                            formatDist(ascentTargetAlt(rb))));
        }
        // ---------- left of navball: toggles
        float tx = nc.x - nbR - 170 + 12, ty = nc.y - nbR - 2;
        const float bh = 25, bs = 28.5f;
        if (button({tx, ty, 100, bh}, "Legs", av.legsDeployed, av.hasLegs)) W.toggleLegs(av);
        if (button({tx, ty + bs, 100, bh}, "Chute", av.chuteDeployed && !av.chuteCut, av.chuteCdA > 0 && !av.chuteCut)) {
            av.chuteDeployed = !av.chuteDeployed;
            if (av.chuteDeployed) sfx::play(sfx::Ev::Chute);
        }
        if (button({tx, ty + bs * 2, 100, bh}, "Sail", av.sailDeployed, av.sailArea > 0)) av.sailDeployed = !av.sailDeployed;
        if (button({tx, ty + bs * 3, 100, bh}, "Solar", av.solarDeployed && av.solarAreaMax > 0, av.solarAreaMax > 0)) W.setSolarDeployed(av, !av.solarDeployed);
        if (button({tx, ty + bs * 4, 100, bh}, "RCS", av.rcs, !av.rcsNoz.empty())) av.rcs = !av.rcs;
        if (button({tx, ty + bs * 5, 100, bh}, "Labels", showLabels)) showLabels = !showLabels;
        if (button({tx, ty + bs * 6, 100, bh}, "Stage", false, vesselStages(av).size() > 1 || !av.ignited, Icon::None, col::warn)) W.stage(av);
        // second column (only what the vessel has): flaps, payload bay
        float tx2 = tx - 108;
        int row2 = 0;
        if (av.hasFlaps) {
            Rectangle b{tx2, ty + bs * row2, 100, bh};
            if (button(b, trf("Flaps {0}", std::to_string(av.flapStage)), av.flapStage > 0)) W.cycleFlaps(av);
            if (CheckCollisionPointRec(GetMousePosition(), b))
                tooltip(keys::tip("Flaps: next setting (8 returns to 0)", keys::A::Flaps) + "  ·  " +
                        fmt("%.1f°", flapAngleAt(av.flapPos, av.flapMax)) + " → " + fmt("%.1f°", flapSettingDeg(av.flapStage, av.flapMax)));
            row2++;
        }
        if (av.bayComp >= 0) {
            Rectangle b{tx2, ty + bs * row2, 100, bh};
            if (button(b, "Bay doors", av.bayOpen)) W.setBayDoors(av, !av.bayOpen);
            if (CheckCollisionPointRec(GetMousePosition(), b))
                tooltip(keys::tip("Payload bay doors", keys::A::BayDoors) + "  ·  " + fmt("%.0f%%", av.bayT * 100));
            row2++;
            bool hasCargo = false;
            for (const Component& c : av.comps) hasCargo |= c.cargo != 0 && c.func != Func::CargoBay;
            Rectangle r{tx2, ty + bs * row2, 100, bh};
            if (button(r, "Release", false, hasCargo && av.bayT >= 0.999)) W.releasePayload(av);
            if (CheckCollisionPointRec(GetMousePosition(), r)) tooltip(keys::tip("Release the payload from the bay (in space, doors open)", keys::A::PayloadRelease));
            row2++;
            std::string why;
            int cand = W.captureCandidate(av, &why);
            Rectangle cpt{tx2, ty + bs * row2, 100, bh};
            if (button(cpt, "Capture", false, cand >= 0)) W.capturePayload(av);
            if (CheckCollisionPointRec(GetMousePosition(), cpt))
                tooltip(cand >= 0 ? trf("Take {0} into the payload bay with the arm", W.vessels[cand].name) : (why.empty() ? i18n::tr("Capture with the arm") : i18n::tr(why)));
            row2++;
        }
    };
    auto layNav = [&]() {
        navPanel(app, av);
        if (!navTargetOf(W, av)) missionPanel(app, av);
    };
    auto layEngines = [&]() { enginesPanel(app, av, sr); };
    ui::layered({{L_VESSEL, layVessel}, {L_WARP, layWarp}, {L_ORBIT, layOrbit}, {L_RESOURCES, layResources}, {L_STAGES, layStages},
                 {L_TARGET, layTarget}, {L_NAVBALL, layNavball}, {L_NAV, layNav}, {L_ENGINES, layEngines}});
}

void bodyListPanel(App& app, Vessel* av) {
    World& W = app.world;
    const Universe& U = W.U;
    float SH = ui::SH();
    Rectangle r{12, 128, 260, SH - 128 - 80};
    if (!bodyListOpen) return;
    panel(r);
    text("CELESTIAL BODIES", r.x + 14, r.y + 12, 13, col::dim, true);
    Rectangle lr{r.x + 6, r.y + 36, r.width - 12, r.height - 44};
    float off = beginScroll(40, lr, (U.count() + 4) * 28.0f + 40);
    float y = lr.y - off;
    std::string lastSys;
    if (av) {
        if (rowItem({lr.x, y, lr.width - 6, 26}, mapFocus < 0)) { mapFocus = -1; setMapDist(std::max(len(av->r) * 3, U.bodies[av->ref].radius * 4)); }
        icon(Icon::Ship, {lr.x + 10, y + 5, 16, 16}, col::accent);
        text(fit(av->name, lr.width - 50, 14), lr.x + 34, y + 4, 14, col::accent, true);
        y += 30;
    }
    for (int i = 0; i < U.count(); i++) {
        const Body& b = U.bodies[i];
        if (b.system != lastSys) {
            lastSys = b.system;
            text(b.system, lr.x + 8, y + 8, 12, col::faint, true);
            y += 28;
        }
        int depth = 0;
        for (int p = b.parent; p >= 0 && !U.bodies[p].isStar; p = U.bodies[p].parent) depth++;
        if (!b.isStar && b.parent >= 0 && U.bodies[b.parent].isStar) depth = 0;
        Rectangle row{lr.x, y, lr.width - 6, 26};
        if (rowItem(row, mapFocus == i)) {
            mapFocus = i;
            selBody = i;
            setMapDist(b.radius * (b.isStar ? 30 : 6));
        }
        unsigned ex = W.explored.empty() ? 0 : W.explored[i];
        icon(b.isStar ? Icon::Star : Icon::Planet, {row.x + 10 + depth * 14.f, row.y + 6, 14, 14},
             b.isStar ? Color{255, 210, 120, 255} : Color{(unsigned char)(b.colA.x * 255), (unsigned char)(b.colA.y * 255), (unsigned char)(b.colA.z * 255), 255});
        text(b.name, row.x + 32 + depth * 14.f, row.y + 4, 14, col::text);
        if (ex & 4) icon(Icon::Check, {row.x + row.width - 22, row.y + 6, 14, 14}, col::good);
        else if (ex & 2) icon(Icon::Target, {row.x + row.width - 22, row.y + 6, 14, 14}, col::accent);
        else if (ex & 1) icon(Icon::Eye, {row.x + row.width - 22, row.y + 6, 14, 14}, col::dim);
        y += 28;
    }
    endScroll();
}

void bodyInfoPanel(App& app, Vessel* av) {
    if (selBody < 0) return;
    World& W = app.world;
    const Universe& U = W.U;
    const Body& b = U.bodies[selBody];
    float SW = ui::SW();
    Rectangle r{SW - 352, 260, 340, 460};
    panel(r);
    text(b.name, r.x + 16, r.y + 12, 20, col::text, true);
    if (iconButton({r.x + r.width - 40, r.y + 10, 28, 28}, Icon::Close)) { selBody = -1; return; }
    text(b.parent >= 0 ? trf("{0}  ·  orbits {1}", tr(b.system), tr(U.bodies[b.parent].name)) : tr(b.system), r.x + 16, r.y + 40, 13, col::dim);
    float y = r.y + 66;
    auto kv = [&](const std::string& k, const std::string& v) {
        text(k, r.x + 16, y, 14, col::dim);
        textR(v, r.x + r.width - 16, y, 14, col::text, true);
        y += 21;
    };
    kv("Radius", formatDist(b.radius));
    kv("Mass", fmt("%.4g kg", b.GM / 6.674e-11));
    kv("Surface gravity", fmt("%.3f m/s²", surfaceGravity(b)));
    kv("Escape velocity", formatSpeed(std::sqrt(2 * b.GM / b.radius)));
    if (b.parent >= 0) {
        kv("Semi-major axis", formatDist(b.a));
        kv("Orbital period", formatTime(TAU / b.n));
        kv("SOI radius", formatDist(b.soi));
    }
    kv("Rotation period", formatTime(std::fabs(TAU / b.rotRate)));
    if (b.atmTop > 0) kv("Surface pressure", fmt("%.3g kPa", b.atmP0 / 1000));
    if (!b.isStar) {
        OrbitBands ob = orbitBands(b);
        kv("Low orbit", formatDist(ob.lowMin) + " - " + formatDist(ob.lowMax));
        kv("High orbit", trf("above {0}", formatDist(ob.lowMax)));
        if (ob.syncAlt > 0) kv("Synchronous orbit", formatDist(ob.syncAlt));
    }
    if (b.isStar) kv("Luminosity", trf("{0} x Sun", fmt("%.4g", b.luminosity / L_SUN)));
    if (av) {
        Vec3d d = U.relPos(W.states, selBody, av->ref) - av->r;
        kv("Distance from vessel", formatDist(len(d)));
    }
    y = wrap(b.desc, r.x + 16, y + 4, r.width - 32, 13, Color{190, 200, 215, 255});
    unsigned ex = W.explored.empty() ? 0 : W.explored[selBody];
    y += 4;
    text((ex & 4) ? "Explored: landed" : (ex & 2) ? "Explored: orbited" : (ex & 1) ? "Explored: visited" : "Explored: not yet", r.x + 16, y, 13,
         ex ? col::good : col::faint);
    float by = r.y + r.height - 48;
    if (button({r.x + 16, by, 96, 34}, "Focus", false, true, Icon::Eye)) {
        mapFocus = selBody;
        setMapDist(b.radius * (b.isStar ? 30 : 6));
    }
    if (av && button({r.x + 120, by, 96, 34}, "Target", av->target == selBody, true, Icon::Target)) {
        av->target = av->target == selBody ? -1 : selBody;
        if (av->target >= 0) missionOpen = true;
    }
    if (av && button({r.x + 224, by, 100, 34}, "Jump", false, W.settings.hyperspace, Icon::Hyper, col::accent2)) {
        hyperOpen = true;
        hyperSel = selBody;
    }
}

void hyperModal(App& app, Vessel* av) {
    if (!hyperOpen || !av) return;
    World& W = app.world;
    const Universe& U = W.U;
    dim();
    float SW = ui::SW(), SH = ui::SH();
    Rectangle r{(SW - 640) / 2, (SH - 600) / 2, 640, 600};
    panel(r, Color{20, 22, 38, 252}, 14);
    icon(Icon::Hyper, {r.x + 22, r.y + 18, 28, 28}, col::accent2);
    text("Hyperspace jump", r.x + 60, r.y + 18, 24, col::text, true);
    if (iconButton({r.x + r.width - 48, r.y + 16, 32, 32}, Icon::Close) || IsKeyPressed(KEY_ESCAPE)) { hyperOpen = false; return; }
    std::string req, where = W.hyperspaceLocationBlock(*av);
    Color rc = col::good;
    if (!W.settings.hyperspace) { req = "Hyperspace travel is disabled in Settings."; rc = col::bad; }
    else if (!av->hasHyperdrive) { req = "Requires a Hyperspace Drive HX-1 part (Fun category)."; rc = col::bad; }
    else if (!where.empty()) { req = where; rc = col::warn; }
    else req = trf("Drive ready. Speed: {0}. Arrives in a parking orbit.", tr(HYPER_SPEED_NAMES[W.settings.hyperSpeed]));
    text(req, r.x + 24, r.y + 58, 14, rc);
    Rectangle lr{r.x + 16, r.y + 88, r.width - 32, r.height - 160};
    float off = beginScroll(55, lr, U.count() * 34.0f + 120);
    float y = lr.y - off;
    std::string lastSys;
    for (int i = 0; i < U.count(); i++) {
        const Body& b = U.bodies[i];
        if (b.system != lastSys) {
            lastSys = b.system;
            text(b.system, lr.x + 8, y + 10, 12, col::faint, true);
            y += 30;
        }
        Rectangle row{lr.x, y, lr.width - 8, 30};
        if (rowItem(row, hyperSel == i)) hyperSel = i;
        icon(b.isStar ? Icon::Star : Icon::Planet, {row.x + 10, row.y + 7, 16, 16}, b.isStar ? Color{255, 210, 120, 255} : col::dim);
        text(b.name, row.x + 36, row.y + 6, 15, col::text);
        Vec3d d = U.relPos(W.states, i, av->ref) - av->r;
        double sp = HYPER_SPEEDS[W.settings.hyperSpeed];
        std::string tt = sp > 1e200 ? tr("instant") : formatTime(len(d) / sp);
        textR(formatDist(len(d)) + "   ·   " + tt, row.x + row.width - 10, row.y + 7, 13, col::dim);
        y += 34;
    }
    endScroll();
    std::string err;
    bool can = hyperSel >= 0 && W.settings.hyperspace && av->hasHyperdrive && where.empty();
    if (primaryButton({r.x + r.width - 220, r.y + r.height - 56, 200, 42}, "ENGAGE", Icon::Hyper, can, col::accent2)) {
        if (W.startHyperspace(hyperSel, err)) {
            hyperOpen = false;
            mapMode = false;
        } else W.toast(err, col::bad);
    }
    if (hyperSel >= 0) text(trf("Destination: {0}", tr(U.bodies[hyperSel].name)), r.x + 24, r.y + r.height - 44, 16, col::accent2, true);
}

// the last launch again from its start (pad, runway or orbit); the vessel flown now is removed
void revertToLaunch(App& app) {
    World& W = app.world;
    if (app.lastLaunched.comps.empty()) return;
    if (W.active >= 0 && W.active < (int)W.vessels.size()) W.vessels.erase(W.vessels.begin() + W.active);
    W.active = -1;
    if (app.lastLaunchBody == -3) W.launchInOrbit(app.lastLaunched, app.lastLaunchNear, 120);
    else if (app.lastLaunchBody != -2) W.launchAircraft(app.lastLaunched, app.lastLaunchBody);
    else W.launch(app.lastLaunched);
    flightOnEnter(app);
}

int listRename = -1;          // vessels list: the id of the vessel whose name is being edited
std::string listName;
bool debrisConfirm = false;   // "Delete debris" pressed once
// "Recover the vessel?" after it came down on Earth
void recoverModal(App& app) {
    if (recoverAsk < 0) return;
    World& W = app.world;
    const Vessel* v = nullptr;
    for (const Vessel& o : W.vessels)
        if (o.id == recoverAsk && !o.destroyed) v = &o;
    if (!v || !(v->landed || v->rolling) || pauseOpen || W.hyper.active) {
        if (!v || !(v->landed || v->rolling)) recoverAsk = -1; // (it took off again / is gone)
        return;
    }
    dim(0.45f);
    float SW = ui::SW(), SH = ui::SH(), dw = std::min(520.0f, SW - 40);
    Rectangle r{(SW - dw) / 2, (SH - 220) / 2, dw, 220};
    panel(r, Color{20, 26, 36, 252}, 14);
    icon(Icon::Home, {r.x + 24, r.y + 20, 26, 26}, col::good);
    text("Recover the vessel?", r.x + 60, r.y + 20, 22, col::text, true);
    wrap(trf("{0} is back on Earth. Recovering it removes it from the flight and returns to the factory.", v->name), r.x + 24, r.y + 66,
         r.width - 48, 14, col::dim);
    float by = r.y + r.height - 60;
    if (button({r.x + 24, by, 170, 40}, "Keep it") || IsKeyPressed(KEY_ESCAPE)) recoverAsk = -1;
    else if (primaryButton({r.x + r.width - 24 - 190, by, 190, 40}, "Recover", Icon::Check) || IsKeyPressed(KEY_ENTER) ||
             IsKeyPressed(KEY_KP_ENTER)) {
        pendingRecover = recoverAsk;
        recoverAsk = -1;
    }
}

// ------------------------------------------------------------------ surface bases: build dialog and list
int buildFor = -1;             // id of the landed vessel the build dialog is about
std::string buildName;          // the name typed for a new base
int pendingBuild = -1;         // "Build" pressed: carried out at the start of the next frame (vessels change)
std::string pendingBuildName;
int baseRename = -1;           // bases list: the id of the base whose name is being edited
std::string baseRenameText;
int baseAbandon = -1;          // "Abandon" pressed once (the second press removes the base)

bool carriesModules(const Vessel& v) {
    for (const Component& c : v.comps)
        if (c.def >= 0 && c.def < (int)CATALOG.defs.size() && CATALOG.defs[c.def].kind == PK::Module) return true;
    return false;
}
std::string localBaseName(const World& W, int body) {
    int n = 1;
    for (const SurfaceBase& b : W.bases) n += b.body == body;
    return trf("{0} Base {1}", tr(W.U.bodies[body].name), std::to_string(n));
}
std::string kw0(double kw) {
    char b[32];
    std::snprintf(b, sizeof b, "%.0f", kw);
    return b;
}
void requirementRow(float x, float& y, float w, bool ok, const std::string& label, const std::string& value) {
    icon(ok ? Icon::Check : Icon::Close, {x, y + 1, 16, 16}, ok ? col::good : col::warn);
    text(fit(label, w - 130, 14), x + 24, y, 14, col::text);
    textR(value, x + w, y + 1, 13, ok ? col::dim : col::warn);
    y += 24;
}
// the three conditions for the launch complex, with where the base stands
void requirementsBlock(const BaseStatus& st, float x, float& y, float w) {
    requirementRow(x, y, w, st.habitats > 0, tr("Habitat (crew quarters)"), trf("{0} crew", std::to_string(st.crew)));
    requirementRow(x, y, w, st.isru > 0, tr("ISRU propellant plant"), std::to_string(st.isru));
    requirementRow(x, y, w, st.powerKw >= BASE_LAUNCH_KW, tr("Power for the launch complex"), trf("{0} / {1} kW", kw0(st.powerKw), kw0(BASE_LAUNCH_KW)));
}
std::string modulesSummary(const SurfaceBase& b) {
    int n[(int)ModuleKind::COUNT] = {};
    for (const BaseModule& m : b.modules) n[(int)m.kind]++;
    std::string out;
    const ModuleKind show[] = {ModuleKind::Habitat, ModuleKind::Greenhouse, ModuleKind::Isru, ModuleKind::Fission, ModuleKind::Solar, ModuleKind::Storage};
    for (ModuleKind k : show)
        if (n[(int)k]) out += (out.empty() ? "" : "  ·  ") + std::to_string(n[(int)k]) + " " + tr(moduleKindName(k));
    return out;
}

// the button over the toolbar while the vessel flown stands on the ground with base modules aboard
void buildBaseButton(App& app, Vessel& av, bool modal) {
    if (!av.landed || !carriesModules(av) || modal) return;
    World& W = app.world;
    BasePlan pl = planBase(W, W.active);
    float SH = ui::SH();
    Rectangle r{12, SH - 64 - 12 - 46 - (logOpen ? 312.f : 0.f), 340, 42};
    std::string lbl = pl.ok && pl.joins >= 0 ? trf("Add modules to {0}", W.bases[pl.joins].name) : tr("Build base");
    if (pl.ok) {
        if (primaryButton(r, fit(lbl, r.width - 56, 15), Icon::Base)) {
            buildOpen = true;
            buildFor = av.id;
            buildName = localBaseName(W, av.ref);
            if (pl.joins < 0) ui::focusText(1120, buildName);
        }
    } else {
        button(r, lbl, false, false, Icon::Base);
        wrap(tr(pl.why), r.x + r.width + 12, r.y + 4, 300, 13, col::warn);
    }
}

void buildModal(App& app) {
    if (!buildOpen) return;
    World& W = app.world;
    int idx = -1;
    for (int i = 0; i < (int)W.vessels.size(); i++)
        if (W.vessels[i].id == buildFor && !W.vessels[i].destroyed) idx = i;
    BasePlan pl = idx >= 0 ? planBase(W, idx) : BasePlan{};
    if (!pl.ok) {
        buildOpen = false;
        ui::cancelEdit();
        return;
    }
    const Vessel& v = W.vessels[idx];
    dim();
    float SW = ui::SW(), SH = ui::SH(), dw = std::min(600.0f, SW - 40), dh = std::min(640.0f, SH - 40);
    Rectangle r{(SW - dw) / 2, (SH - dh) / 2, dw, dh};
    panel(r, Color{20, 26, 36, 252}, 14);
    icon(Icon::Base, {r.x + 24, r.y + 20, 26, 26}, col::accent);
    bool founding = pl.joins < 0;
    text(founding ? tr("Build a surface base") : trf("Add modules to {0}", W.bases[pl.joins].name), r.x + 60, r.y + 20, 22, col::text, true);
    text(fit(trf("{0} on {1}", v.name, tr(W.U.bodies[v.ref].name)), r.width - 48, 13), r.x + 24, r.y + 56, 13, col::dim);
    float y = r.y + 86;
    if (founding) {
        text("Name", r.x + 24, y, 13, col::dim);
        textInput({r.x + 24, y + 18, r.width - 48, 36}, buildName, 48, 1120);
        y += 68;
    }
    text("MODULES TO SET UP", r.x + 24, y, 13, col::dim, true);
    y += 24;
    for (const BaseModule& m : pl.modules) {
        icon(m.kind == ModuleKind::Fission || m.kind == ModuleKind::Solar ? Icon::Bolt : Icon::Base, {r.x + 28, y + 1, 16, 16}, col::accent);
        text(tr(moduleKindName(m.kind)), r.x + 52, y, 14, col::text);
        if (m.kw > 0) textR(trf("{0} kW", kw0(m.kw)), r.x + r.width - 24, y + 1, 13, col::dim);
        y += 22;
    }
    y += 10;
    // the base as it will be: the launch complex once all three are there
    SurfaceBase after = founding ? SurfaceBase{} : W.bases[pl.joins];
    for (const BaseModule& m : pl.modules) after.modules.push_back(m);
    BaseStatus st = baseStatus(after);
    text("AFTER BUILDING", r.x + 24, y, 13, col::dim, true);
    y += 24;
    requirementsBlock(st, r.x + 24, y, r.width - 48);
    y += 4;
    bool padReady = st.hasPad || st.requirementsMet();
    y = wrap(padReady ? tr("The base gets a launch pad: rockets designed in the factory can be launched from it (choose it as the launch site).")
                      : trf("No launch pad yet: a base needs a habitat, an ISRU propellant plant and {0} kW of power to launch rockets.", kw0(BASE_LAUNCH_KW)),
             r.x + 24, y, r.width - 48, 13, padReady ? col::good : col::dim) + 6;
    wrap("The modules are unloaded and set up around the base; what is left of the vessel stays where it landed.", r.x + 24, y, r.width - 48, 12, col::faint);
    float by = r.y + r.height - 60;
    std::string nm = cleanLineText(buildName, 48);
    bool nameOk = !founding || nm.find_first_not_of(' ') != std::string::npos;
    if (button({r.x + 24, by, 170, 40}, "Cancel") || IsKeyPressed(KEY_ESCAPE)) {
        buildOpen = false;
        ui::cancelEdit();
    } else if (primaryButton({r.x + r.width - 24 - 200, by, 200, 40}, founding ? "Build base" : "Add modules", Icon::Check, nameOk) ||
               (nameOk && (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)))) {
        pendingBuild = buildFor;
        pendingBuildName = founding ? nm : std::string();
        buildOpen = false;
        ui::cancelEdit();
    }
}

void basesModal(App& app) {
    if (!basesOpen) {
        if (baseRename >= 0) {
            baseRename = -1;
            ui::cancelEdit();
        }
        baseAbandon = -1;
        return;
    }
    World& W = app.world;
    dim();
    float SW = ui::SW(), SH = ui::SH(), dw = std::min(880.0f, SW - 40), dh = std::min(640.0f, SH - 40);
    Rectangle r{(SW - dw) / 2, (SH - dh) / 2, dw, dh};
    panel(r, Color{20, 26, 36, 252}, 14);
    icon(Icon::Base, {r.x + 24, r.y + 20, 26, 26}, col::accent);
    text("Surface bases", r.x + 60, r.y + 18, 24, col::text, true);
    bool esc = IsKeyPressed(KEY_ESCAPE);
    if (esc && baseRename >= 0) {
        baseRename = -1;
        ui::cancelEdit();
        esc = false;
    }
    if (iconButton({r.x + r.width - 48, r.y + 16, 32, 32}, Icon::Close) || esc) {
        basesOpen = false;
        return;
    }
    Rectangle lr{r.x + 16, r.y + 64, r.width - 32, r.height - 80};
    if (W.bases.empty()) {
        wrap(trf("No bases yet. Land a vessel carrying a habitat module (factory: Base parts) on the Moon, Mars or another solid world and build one there. "
                 "With a habitat, an ISRU propellant plant and {0} kW of power the base gets a launch pad, and rockets can be launched from it.",
                 kw0(BASE_LAUNCH_KW)),
             lr.x + 8, lr.y + 8, lr.width - 16, 14, col::dim);
        return;
    }
    const float rowH = 156;
    float off = beginScroll(67, lr, W.bases.size() * rowH);
    float y = lr.y - off;
    int remove = -1;
    for (int i = 0; i < (int)W.bases.size(); i++, y += rowH) {
        SurfaceBase& b = W.bases[i];
        if (y + rowH < lr.y || y > lr.y + lr.height) continue;
        Rectangle row{lr.x, y + 4, lr.width - 12, rowH - 10};
        DrawRectangleRounded(row, 0.06f, 6, Color{26, 33, 45, 255});
        BaseStatus st = baseStatus(b);
        float x = row.x + 16, colW = row.width * 0.5f - 24;
        icon(Icon::Base, {x, row.y + 14, 22, 22}, st.hasPad ? col::good : col::accent);
        if (baseRename == b.id) {
            textInput({x + 32, row.y + 8, colW - 40, 32}, baseRenameText, 48, 1130);
            std::string nm = cleanLineText(baseRenameText, 48);
            if ((IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) && nm.find_first_not_of(' ') != std::string::npos) {
                b.name = nm;
                baseRename = -1;
                ui::cancelEdit();
            }
        } else {
            text(fit(b.name, colW - 80, 17, true), x + 32, row.y + 14, 17, col::text, true);
            if (iconButton({x + colW - 36, row.y + 10, 28, 28}, Icon::Pencil, false, "Rename")) {
                baseRename = b.id;
                baseRenameText = b.name;
                ui::focusText(1130, baseRenameText);
            }
        }
        text(fit(trf("{0}  ·  founded {1}", tr(W.U.bodies[b.body].name), formatDate(b.founded)), colW, 13), x, row.y + 46, 13, col::dim);
        wrap(modulesSummary(b), x, row.y + 68, colW, 13, col::text);
        text(trf("Power {0} kW  ·  needed {1} kW", kw0(st.powerKw), kw0(st.demandKw)), x, row.y + 108, 13, st.powerKw >= st.demandKw ? col::dim : col::warn);
        float rx = row.x + row.width * 0.5f + 8, ry = row.y + 12, rw = row.width * 0.5f - 24;
        requirementsBlock(st, rx, ry, rw);
        float by = row.y + row.height - 46;
        if (st.hasPad) {
            bool sel = W.settings.launchBase == b.id;
            if (button({rx, by, 220, 36}, sel ? "Launch site (selected)" : "Launch from here", sel, true, Icon::Rocket)) {
                W.settings.launchBase = b.id;
                W.toast(trf("Launch site: {0}", b.name), col::good);
            }
        } else text("No launch pad yet", rx, by + 10, 13, col::dim);
        bool confirm = baseAbandon == b.id;
        if (button({rx + rw - 150, by, 150, 36}, confirm ? "Click to abandon" : "Abandon", confirm, true, Icon::Trash, col::bad)) {
            if (confirm) remove = i;
            else baseAbandon = b.id;
        }
    }
    endScroll();
    if (remove >= 0) {
        std::string nm = W.bases[remove].name;
        if (W.settings.launchBase == W.bases[remove].id) W.settings.launchBase = 0;
        W.bases.erase(W.bases.begin() + remove);
        W.syncBaseTerrain();
        W.toast(trf("{0} abandoned", nm), col::dim);
        baseAbandon = -1;
    }
}

void vesselsModal(App& app) {
    if (!vesselsOpen) {
        if (listRename >= 0) {
            listRename = -1;
            ui::cancelEdit();
        }
        debrisConfirm = false;
        return;
    }
    World& W = app.world;
    dim();
    float SW = ui::SW(), SH = ui::SH();
    Rectangle r{(SW - 790) / 2, (SH - 520) / 2, 790, 520};
    panel(r, Color{20, 26, 36, 252}, 14);
    text("Vessels in flight", r.x + 24, r.y + 18, 24, col::text, true);
    bool esc = IsKeyPressed(KEY_ESCAPE);
    if (esc && listRename >= 0) { // (Esc ends the name edit first)
        listRename = -1;
        ui::cancelEdit();
        esc = false;
    }
    if (iconButton({r.x + r.width - 48, r.y + 16, 32, 32}, Icon::Close) || esc) { vesselsOpen = false; return; }
    // all debris at once: spent stages, fairing halves - whatever cannot be controlled (two clicks)
    int nDebris = 0;
    for (int i = 0; i < (int)W.vessels.size(); i++) nDebris += isDebris(W, W.vessels[i], i);
    {
        std::string lbl = debrisConfirm ? trf("Delete {0} debris?", std::to_string(nDebris)) : trf("Delete debris ({0})", std::to_string(nDebris));
        Rectangle b{r.x + r.width - 60 - 230, r.y + 16, 230, 32};
        if (button(b, lbl, debrisConfirm, nDebris > 0, Icon::Trash, col::bad)) {
            if (!debrisConfirm) debrisConfirm = true;
            else {
                int n = deleteDebris(W);
                W.toast(trf("{0} debris deleted", std::to_string(n)), col::good);
                debrisConfirm = false;
                return;
            }
        }
        if (debrisConfirm && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !CheckCollisionPointRec(GetMousePosition(), b)) debrisConfirm = false;
    }
    Rectangle lr{r.x + 16, r.y + 64, r.width - 32, r.height - 80};
    // lost vessels (crashed, burnt up, docked into another one) are not listed: only what still flies
    int shown = 0;
    for (const Vessel& v : W.vessels) shown += !v.destroyed;
    float off = beginScroll(66, lr, shown * 60.0f);
    float y = lr.y - off;
    int del = -1;
    Vessel* actv = W.activeVessel();
    for (int i = 0; i < (int)W.vessels.size(); i++) {
        Vessel& v = W.vessels[i];
        if (v.destroyed) continue;
        Rectangle row{lr.x, y, lr.width - 8, 56};
        DrawRectangleRounded(row, 0.15f, 6, i == W.active ? Color{14, 52, 72, 255} : Color{26, 32, 44, 255});
        if (listRename == v.id) {
            // the name being edited: Enter keeps it, Esc leaves the old one
            textInput({row.x + 10, row.y + 4, row.width - 470, 26}, listName, 48, 1111);
            if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) {
                renameVessel(W, v, listName);
                listRename = -1;
                ui::cancelEdit();
            }
        } else text(fit(v.name, row.width - 470, 16, true), row.x + 14, row.y + 8, 16, v.hidden ? col::dim : col::text, true);
        if (iconButton({row.x + row.width - 456, row.y + 12, 34, 32}, listRename == v.id ? Icon::Check : Icon::Pencil, listRename == v.id,
                       listRename == v.id ? "Keep the new name (Enter)" : "Rename")) {
            if (listRename == v.id) {
                renameVessel(W, v, listName);
                listRename = -1;
                ui::cancelEdit();
            } else {
                listRename = v.id;
                listName = v.name;
                ui::focusText(1111, listName);
            }
        }
        text(fit(v.hidden ? i18n::tr("hidden") + "  ·  " + situation(W, v) : situation(W, v), row.width - 470, 13), row.x + 14, row.y + 31, 13,
             v.destroyed ? col::bad : col::dim);
        // shown in the view / its orbit on the map
        if (iconButton({row.x + row.width - 418, row.y + 12, 38, 32}, v.hidden ? Icon::EyeOff : Icon::Eye, !v.hidden,
                       v.hidden ? "Hidden - click to show" : "Shown - click to hide", i != W.active))
            v.hidden = !v.hidden;
        if (iconButton({row.x + row.width - 376, row.y + 12, 38, 32}, Icon::Trajectory, v.showOrbit,
                       v.showOrbit ? "Orbit shown on the map - click to hide" : "Orbit hidden - click to show"))
            v.showOrbit = !v.showOrbit;
        if (!v.destroyed && i != W.active && button({row.x + row.width - 210, row.y + 12, 110, 32}, "Switch", false, true, Icon::Play)) pendingSwitch = i;
        // rendezvous target (SAS "Target" points at it, the target panel shows range and closing rate)
        if (!v.destroyed && actv && i != W.active && v.ref == actv->ref) {
            bool tg = actv->targetVessel == v.id;
            if (button({row.x + row.width - 332, row.y + 12, 116, 32}, tg ? "Targeted" : "Target", tg, true, Icon::Target))
                actv->targetVessel = tg ? -1 : v.id;
        }
        if (button({row.x + row.width - 92, row.y + 12, 82, 32}, "Remove", false, true, Icon::Trash, col::bad)) del = i;
        y += 60;
    }
    endScroll();
    if (shown == 0) textC("No vessels in flight", lr, 16, col::faint);
    if (del >= 0) {
        for (Vessel& o : W.vessels)
            if (o.targetVessel == W.vessels[del].id) o.targetVessel = -1;
        W.vessels.erase(W.vessels.begin() + del);
        if (W.active == del) W.active = W.vessels.empty() ? -1 : 0;
        else if (W.active > del) W.active--;
    }
}

void hyperEffect(World& W, Vector2 c) {
    if (!W.hyper.active) return;
    float p = (float)W.hyper.phase;
    float SW = ui::SW(), SH = ui::SH();
    float reach = 0;
    for (Vector2 k : {Vector2{0, 0}, Vector2{SW, 0}, Vector2{0, SH}, Vector2{SW, SH}})
        reach = std::max(reach, std::hypot(k.x - c.x, k.y - c.y));
    float intensity = p < 1.5f ? p / 1.5f : (p < 4.2f ? 1.f : std::max(0.f, 1 - (p - 4.2f) / 0.8f));
    DrawRectangle(0, 0, (int)SW, (int)SH, Color{20, 8, 50, (unsigned char)(intensity * 150)});
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> U01(0, 1);
    int n = 260;
    BeginBlendMode(BLEND_ADDITIVE);
    for (int i = 0; i < n; i++) {
        float a = U01(rng) * 2 * PI;
        float speed = 0.4f + U01(rng);
        float base = U01(rng);
        float t = std::fmod(base + p * speed * (p > 1.5f ? 1.6f : 0.3f), 1.0f);
        float r0 = t * t * reach * 1.05f;
        float l = (p > 1.5f ? 220.f : 20.f) * t * intensity;
        Vector2 a0{c.x + cosf(a) * r0, c.y + sinf(a) * r0}, a1{c.x + cosf(a) * (r0 + l), c.y + sinf(a) * (r0 + l)};
        Color col = U01(rng) < 0.5f ? Color{150, 120, 255, 255} : Color{120, 200, 255, 255};
        col.a = (unsigned char)(intensity * 220 * t);
        DrawLineEx(a0, a1, 1.5f + 2 * t, col);
    }
    EndBlendMode();
    if (p > 4.2f) DrawRectangle(0, 0, (int)SW, (int)SH, Color{255, 255, 255, (unsigned char)(std::max(0.f, 1 - (p - 4.2f) / 0.8f) * 200)});
    std::string label = p < 1.5f ? "Spooling hyperspace drive..." : "In hyperspace";
    textC(label, {0, SH * 0.2f, SW, 30}, 22, Color{220, 210, 255, 255}, true);
    if (W.hyper.target >= 0)
        textC(trf("Destination: {0}", tr(W.U.bodies[W.hyper.target].name)) + "   ·   " + formatDist(W.hyper.distance), {0, SH * 0.2f + 34, SW, 22}, 16,
              Color{190, 180, 240, 255});
}

void toasts(World& W) {
    float SW = ui::SW();
    float y = 76;
    for (auto& t : W.toasts) {
        float age = (float)(W.realTime - t.born);
        float a = std::clamp(std::min(age * 4, (6 - age) * 1.5f), 0.f, 1.f);
        Vector2 m = measure(t.text, 15);
        Rectangle r{SW / 2 - m.x / 2 - 18, y, m.x + 36, 32};
        DrawRectangleRounded(r, 0.5f, 8, Color{18, 24, 36, (unsigned char)(220 * a)});
        textC(t.text, r, 15, Color{t.color.r, t.color.g, t.color.b, (unsigned char)(255 * a)});
        y += 38;
    }
}

} // namespace

namespace {
bool g_dbgInputOn = false;
ControlInput g_dbgInput;
} // namespace
bool flightPaused() { return pauseOpen; }
bool flightMapMode() { return mapMode; }
void flightDebugNightVision(bool on) { nightVision = on ? 1 : 0; }
void flightDebugNightMode(int m) { nightVision = std::clamp(m, 0, 2); }
bool flightHyperOpen() { return hyperOpen; }
void flightDebugLog(bool on) { logOpen = on; }
void flightDebugPause(bool on) { pauseOpen = on; }
void flightDebugVessels(bool on) { vesselsOpen = on; }
void flightDebugBases(bool on) { basesOpen = on; }
void flightDebugBuildDialog(App& app, bool on) {
    buildOpen = on;
    if (on && app.world.activeVessel()) {
        buildFor = app.world.activeVessel()->id;
        buildName = localBaseName(app.world, app.world.activeVessel()->ref);
    }
}
void flightDebugAchievements(bool on) { achOpen = on; }
// open the menu of the vessel with this id (at the screen centre) and "click" item k on the next frame (-1: none)
void flightDebugMenu(int id, int item) {
    menuVessel = id;
    menuAt = {ui::SW() * 0.5f, ui::SH() * 0.4f};
    vmenuFrame = 1;
    menuConfirm = false;
    g_dbgMenuItem = item;
}
bool flightDebugMenuOpen() { return menuVessel >= 0; }
void flightDebugNav(bool open) { navOpen = open; }
void flightDebugEngines(bool on, int hoverBell) {
    enginesOpen = on;
    g_dbgEngineHover = hoverBell;
}

void flightDebugInput(const ControlInput* in) {
    g_dbgInputOn = in != nullptr;
    if (in) g_dbgInput = *in;
}

// test hooks: the flight camera (yaw / pitch / distance / look-at point in the horizon frame)
// the frame of the flight camera (east, north, up at the vessel), for the tests
void flightDebugHorizon(const World& W, const Vessel& v, Vec3d& up, Vec3d& north, Vec3d& east) { horizonFrame(W, v, up, north, east); }
void flightDebugView(double yaw, double pitch, double dist, Vec3d target) {
    fcam.animT = -1;
    fcam.yaw = yaw;
    fcam.pitch = pitch;
    fcam.dist = dist;
    fcam.target = target;
}
void flightDebugGetView(double& yaw, double& pitch, double& dist, Vec3d& target) {
    yaw = fcam.yaw;
    pitch = fcam.pitch;
    dist = fcam.dist;
    target = fcam.target;
}
double flightDebugFovy() { return fcam.fovy; }

void flightDebugPick(int mode) {
    g_dbgPick = mode;
    g_dbgPickSeen = false;
}
bool flightDebugPickSeen() { return g_dbgPickSeen; }

int flightDebugDeleteDebris(World& W) { return deleteDebris(W); }
void flightDebugRename(World& W, Vessel& v, const std::string& name) { renameVessel(W, v, name); }
int flightDebugRecoverAsk() { return recoverAsk; }
void flightDebugRecover() {
    pendingRecover = recoverAsk;
    recoverAsk = -1;
}
void flightDebugMapFocus(int body, double dist, double yaw, double pitch) {
    mapMode = true;
    mapFocus = body;
    initCams();
    mcam.animT = -1;
    mcam.yaw = yaw;
    mcam.pitch = pitch;
    mcam.target = {};
    mcam.dist = dist;
}
void flightDebugSetMap(bool on, double dist) {
    mapMode = on;
    mapFocus = -1;
    if (dist > 0) setMapDist(dist);
}

void flightOnEnter(App& app) {
    Vessel* av = app.world.activeVessel();
    mapMode = false;
    mapFocus = -1;
    initCams();
    fcam.animT = -1;
    fcam.target = {};
    fcam.dist = av ? std::max(av->length * 2.2, 14.0) : 40;
    fcam.pitch = av && !av->landed ? 0.42 : 0.12;
    fcam.yaw = -PI / 2 + 0.55;
    fcam.tool = 0;
    pauseOpen = hyperOpen = vesselsOpen = achOpen = basesOpen = buildOpen = false;
    menuVessel = -1;
}

// sounds of the watched vessel: events from the simulation and the continuous engine / wind / RCS mix
static void flightSound(App& app, Vessel& av, bool paused) {
    World& W = app.world;
    for (SimEvent e : W.events) {
        switch (e) {
        case SimEvent::Stage: sfx::play(sfx::Ev::Stage); break;
        case SimEvent::Ignite: sfx::play(sfx::Ev::Ignite); break;
        case SimEvent::Explosion: sfx::play(sfx::Ev::Explosion); break;
        case SimEvent::Touchdown: sfx::play(sfx::Ev::Touchdown); break;
        case SimEvent::HyperIn: sfx::play(sfx::Ev::HyperIn); break;
        case SimEvent::HyperOut: sfx::play(sfx::Ev::HyperOut); break;
        case SimEvent::PanelsTorn: sfx::play(sfx::Ev::Tear); break;
        case SimEvent::Karman: sfx::play(sfx::Ev::Milestone); break;
        case SimEvent::WarpArrived: sfx::play(sfx::Ev::Toggle); break;
        case SimEvent::OrbitReached: sfx::play(sfx::Ev::Milestone); break;
        case SimEvent::Landed: sfx::play(sfx::Ev::Milestone, 0.7f); break;
        case SimEvent::Gravity: sfx::play(sfx::Ev::Milestone, 0.6f); break;
        }
    }
    W.events.clear();
    {
        // explosions of other vessels: sound needs air to travel through, arrives after d / 343 s and
        // fades with distance; far away only the dull rumble is left
        struct Pending { double at; float gain; bool far; };
        static std::vector<Pending> pending;
        for (const auto& b : W.blasts) {
            if (b.ref != av.ref || av.destroyed) continue;
            double d = len(b.r - av.r);
            const Body& bb = W.U.bodies[b.ref];
            double pAtm = W.U.pressure(b.ref, len(b.r) - bb.radius) / P_ATM;
            if (pAtm < 0.01 || d > 30000) continue;
            float g = (float)(std::sqrt(std::min(1.0, pAtm)) * 300.0 / (300.0 + d));
            if (g > 0.03f) pending.push_back({GetTime() + d / 343.0, std::min(1.0f, g * 1.6f), d > 900});
        }
        W.blasts.clear();
        double now = GetTime();
        for (size_t i = 0; i < pending.size();) {
            if (pending[i].at <= now) {
                if (!paused) sfx::play(pending[i].far ? sfx::Ev::ExplosionFar : sfx::Ev::Explosion, pending[i].gain);
                pending.erase(pending.begin() + i);
            } else i++;
        }
    }
    sfx::Ambient a;
    bool live = !paused && !av.destroyed && !W.hyper.active && W.warpFactor() <= 4;
    if (live) {
        const Body& b = W.U.bodies[av.ref];
        double alt = len(av.r) - b.radius;
        double patm = W.U.pressure(av.ref, alt) / P_ATM;
        float crackle = 0;
        for (size_t i = 0; i < av.comps.size() && i < av.engLevel.size(); i++) {
            if (av.engLevel[i] <= 0 || av.comps[i].func != Func::Engine) continue;
            const PartDef* d = av.comps[i].def >= 0 ? &CATALOG.defs[av.comps[i].def] : nullptr;
            if (d && d->kind == PK::SRB) crackle = 1;
            else if (av.comps[i].prop == Prop::Kerolox || (d && d->prop == Prop::Kerolox)) crackle = std::max(crackle, 0.5f);
        }
        // loudness grows with thrust (log-like), farther cameras hear less
        float dist = mapMode ? 4000.f : (float)fcam.dist;
        float atten = std::clamp(80.0f / (dist + 40.0f), 0.12f, 1.0f) * (mapMode ? 0.5f : 1.0f);
        a.engine = (float)std::min(1.0, std::sqrt(av.tThrust / 6.0e6) + (av.tThrust > 0 ? 0.12 : 0.0)) * atten;
        a.vacuum = (float)(1.0 - std::clamp(patm, 0.0, 1.0));
        a.crackle = crackle;
        a.wind = (float)std::min(1.0, std::sqrt(av.tDynP / 40000.0)) * 0.8f * atten;
        a.speed = (float)std::min(1.0, len(W.surfaceVel(av)) / 3000.0);
        float r = 0;
        for (float f : av.rcsFire) r += f;
        a.rcs = av.rcsFire.empty() ? 0.f : std::min(1.0f, r / std::max(4.0f, av.rcsFire.size() * 0.25f)) * atten;
    }
    sfx::setAmbient(a);
}

void flightFrame(App& app) {
    World& W = app.world;
    if (pendingSwitch >= 0) {
        int i = pendingSwitch;
        pendingSwitch = -1;
        switchToVessel(app, i);
    }
    if (pendingBuild >= 0) {
        int id = pendingBuild;
        pendingBuild = -1;
        for (int i = 0; i < (int)W.vessels.size(); i++)
            if (W.vessels[i].id == id) {
                if (buildBase(W, i, pendingBuildName) >= 0) sfx::play(sfx::Ev::Milestone);
                break;
            }
    }
    if (pendingRemove >= 0) {
        // "Remove" in the vessel menu (never the vessel flown)
        int id = pendingRemove;
        pendingRemove = -1;
        for (int i = 0; i < (int)W.vessels.size(); i++)
            if (W.vessels[i].id == id && i != W.active) {
                std::string nm = W.vessels[i].name;
                W.vessels.erase(W.vessels.begin() + i);
                if (W.active > i) W.active--;
                for (Vessel& o : W.vessels)
                    if (o.targetVessel == id) o.targetVessel = -1;
                W.toast(trf("Removed: {0}", nm), col::dim);
                break;
            }
    }
    if (pendingRecover >= 0) {
        // recovered: the vessel leaves the flight; the next one (a craft that can be flown) is the active one, and
        // the factory is shown
        int id = pendingRecover;
        pendingRecover = -1;
        for (int i = 0; i < (int)W.vessels.size(); i++)
            if (W.vessels[i].id == id) {
                std::string nm = W.vessels[i].name;
                W.vessels.erase(W.vessels.begin() + i);
                for (Vessel& o : W.vessels)
                    if (o.targetVessel == id) o.targetVessel = -1;
                W.active = -1;
                for (int j = 0; j < (int)W.vessels.size() && W.active < 0; j++)
                    if (!W.vessels[j].destroyed && W.vessels[j].hasControl && !W.vessels[j].debris) W.active = j;
                W.toast(trf("Recovered: {0}", nm), col::good);
                g_airborne.erase(id);
                app.screen = Screen::Factory;
                sfx::setAmbient({});
                return;
            }
    }
    if (pendingUndock >= 0) {
        int p = pendingUndock;
        pendingUndock = -1;
        if (Vessel* a = W.activeVessel()) W.undock(*a, p);
    }
    Vessel* av = W.activeVessel();
    if (!av) {
        app.screen = Screen::Factory;
        sfx::setAmbient({});
        return;
    }
    // back on Earth after a flight (standing, or rolled to a stop on its wheels): ask about recovering it
    if (!av->landed && !av->rolling) g_airborne.insert(av->id);
    if (g_airborne.count(av->id) && !av->destroyed && av->ref == W.U.find("Earth") &&
        (av->landed || (av->rolling && len(W.surfaceVel(*av)) < 0.5))) {
        g_airborne.erase(av->id);
        if (recoverAsk < 0 && W.recordsOn) recoverAsk = av->id; // (not in the tutorials: they go on after a landing)
    }
    bool modal = pauseOpen || hyperOpen || vesselsOpen || basesOpen || buildOpen || achOpen || app.settingsOpen || app.closeAsk;
    ui::setInputBlocked(modal);
    flightSound(app, *av, pauseOpen || app.settingsOpen || app.closeAsk);
    float dt = GetFrameTime();

    // ---------------- input
    W.input = {};
    if (!modal && !keyboardCaptured() && !W.hyper.active) {
        if (!av->destroyed) {
            bool navDrag = nav::middleDragging() || IsMouseButtonDown(MOUSE_BUTTON_MIDDLE);
            using keys::A;
            if (keys::down(A::ThrottleUp) && !navDrag) av->throttle = std::min(1.0, av->throttle + dt * 0.7);
            if (keys::down(A::ThrottleDown) && !navDrag) av->throttle = std::max(0.0, av->throttle - dt * 0.7);
            if (keys::pressed(A::ThrottleFull)) av->throttle = 1;
            if (keys::pressed(A::ThrottleCut)) av->throttle = 0;
            W.input.pitch = keys::axis(A::PitchDown, A::PitchUp);
            // (the craft's +X is its left side - lookRot: x = up x forward - so the right yaw key turns the
            // nose toward -X)
            W.input.yaw = keys::axis(A::YawLeft, A::YawRight);
            W.input.roll = keys::axis(A::RollRight, A::RollLeft);
            if (keys::pressed(A::Stage)) W.stage(*av);
            if (keys::pressed(A::SAS)) av->sas = av->sas == SAS::Off ? SAS::Stability : SAS::Off;
            if (keys::pressed(A::Legs) && av->hasLegs) W.toggleLegs(*av);
            if (keys::pressed(A::Chute) && av->chuteCdA > 0 && !av->chuteCut) {
                av->chuteDeployed = !av->chuteDeployed;
                if (av->chuteDeployed) sfx::play(sfx::Ev::Chute);
            }
            if (keys::pressed(A::Sail) && av->sailArea > 0) av->sailDeployed = !av->sailDeployed;
            if (keys::pressed(A::Flaps) && av->hasFlaps) W.cycleFlaps(*av);
            if (keys::pressed(A::BayDoors) && av->bayComp >= 0) W.setBayDoors(*av, !av->bayOpen);
            if (keys::pressed(A::PayloadRelease) && av->bayComp >= 0) W.releasePayload(*av);
            if (keys::pressed(A::PayloadCapture) && av->bayComp >= 0) W.capturePayload(*av);
            if (keys::pressed(A::Solar) && av->solarAreaMax > 0) W.setSolarDeployed(*av, !av->solarDeployed);
            if (keys::pressed(A::RCS) && !av->rcsNoz.empty()) {
                av->rcs = !av->rcs;
                W.toast(av->rcs ? "RCS on" : "RCS off", {150, 200, 255, 255});
            }
            if (av->rcs) {
                W.input.tz = keys::axis(A::TransFwd, A::TransBack);
                W.input.ty = keys::axis(A::TransUp, A::TransDown);
                W.input.tx = keys::axis(A::TransLeft, A::TransRight); // (craft +X is its left side, as for yaw)
            }
            if (keys::pressed(A::Hyperspace)) { hyperOpen = true; hyperSel = av->target; }
            if (keys::pressed(A::MissionLog)) logOpen = !logOpen;
            if (keys::pressed(A::NightVision)) {
                cycleNightVision(W);
            }
        }
        using keys::A;
        if (g_dbgInputOn) W.input = g_dbgInput;
        if (keys::pressed(A::ToggleHUD)) {
            app.hudHidden = !app.hudHidden;
            app.hudToggleTime = GetTime();
        }
        if (keys::pressed(A::MapView)) {
            mapMode = !mapMode;
            if (mapMode && mapFocus < 0) setMapDist(std::max(len(av->r) * 3.0, W.U.bodies[av->ref].radius * 4));
        }
        if (keys::pressed(A::WarpFaster)) W.setWarp(W.warpIndex + 1);
        if (keys::pressed(A::WarpSlower)) W.setWarp(W.warpIndex - 1);
        if (keys::pressed(A::WarpStop)) W.setWarp(0);
        if (keys::pressed(A::FocusVessel)) mapFocus = -1;
        if (keys::pressed(A::QuickSave)) W.toast(quickSave(app) ? "Quicksaved" : "Quicksave failed", col::good);
        if (keys::pressed(A::QuickLoad)) {
            if ((finishSaves(), loadWorld(W, app.saveFile))) {
                if (!app.mapPath.empty()) markMapSaved(app); // (the world is the saved one again)
                W.toast("Quickloaded", col::good);
                flightOnEnter(app);
                return;
            } else W.toast("No quicksave found", col::bad);
        }
        if (IsKeyPressed(KEY_ESCAPE)) {
            if (fcam.tool != 0 || mcam.tool != 0) fcam.tool = mcam.tool = 0;
            else if (menuVessel >= 0) menuVessel = -1;
            else if (recoverAsk >= 0) recoverAsk = -1;
            else pauseOpen = true;
        }
    } else if (pauseOpen && IsKeyPressed(KEY_ESCAPE) && !app.settingsOpen) pauseOpen = false;

    // camera controls (Fusion-style navigation)
    initCams();
    fcam.update(dt);
    mcam.update(dt);
    {
        Rectangle full{0, 0, ui::SW(), ui::SH()};
        bool allow = !modal && !mouseOverUI();
        if (mapMode) {
            Vector2 mp = GetMousePosition();
            (void)mp;
            auto fitMap = [&]() {
                double r = mapFocus >= 0 ? W.U.bodies[mapFocus].radius * (W.U.bodies[mapFocus].isStar ? 30 : 6)
                                         : std::max(len(av->r) * 3.0, W.U.bodies[av->ref].radius * 4);
                mcam.animateTo(mcam.yaw, mcam.pitch, r, {});
            };
            nav::handleMouse(mcam, full, allow, nullptr, fitMap);
        } else {
            double vr = std::max(av->length, av->maxR * 2) * 0.6 + 1;
            auto pickVessel = [&](const Vec3d& o, const Vec3d& d, Vec3d& hit) {
                double t = -dot(o, d);
                Vec3d c = o + d * t;
                if (t > 0 && len(c) < vr) { hit = c; return true; }
                return false;
            };
            auto fitVessel = [&]() { fcam.fit({}, vr); };
            nav::handleMouse(fcam, full, allow, pickVessel, fitVessel);
        }
    }

    // ---------------- simulate
    static bool wasHyper = false;
    static double hyperYaw0 = 0, hyperPitch0 = 0;
    updateNav(app);
    if (!pauseOpen && !app.closeAsk) W.update(dt); // (the quit dialog also stops the clock)
    if (W.hyper.active && W.activeVessel()) {
        // chase view: swing round behind the vessel as it turns onto the direction of travel, slightly
        // above it so the vanishing point of the star streaks stays visible
        if (!wasHyper) {
            fcam.animT = -1;
            hyperYaw0 = fcam.yaw;
            hyperPitch0 = fcam.pitch;
        }
        Vec3d l = hyperDirLocal(W, *W.activeVessel());
        double y1 = std::atan2(-l.y, -l.x), p1 = clampd(std::asin(clampd(-l.z, -1, 1)) + 0.22, -1.40, 1.40);
        double s = hyperTurn(W);
        fcam.yaw = hyperYaw0 + std::remainder(y1 - hyperYaw0, TAU) * s;
        fcam.pitch = lerpd(hyperPitch0, p1, s);
        fcam.target = fcam.target * (1 - s);
    }
    if (wasHyper && !W.hyper.active) {
        // frame the destination after a hyperspace exit
        fcam.pitch = 0.45;
        fcam.target = {};
        fcam.dist = std::max(W.activeVessel() ? W.activeVessel()->length * 2.2 : 40.0, 14.0);
    }
    wasHyper = W.hyper.active;
    av = W.activeVessel();
    if (!av) { app.screen = Screen::Factory; return; }

    // ---------------- draw
    gfx::setNightLift(!mapMode && nightVision == 2 ? 0.45f : 0.f);
    if (mapMode) renderMap(app, av);
    else if (nightVision) {
        nvBegin();
        renderFlight(app, *av);
        nvEnd();
    } else renderFlight(app, *av);
    {
        Vector2 vp{ui::SW() / 2, ui::SH() / 2};
        if (W.hyper.active && !mapMode) {
            Vector2 sp;
            Vec3d l = hyperDirLocal(W, *av);
            if (nav::project(fcam, Rectangle{0, 0, ui::SW(), ui::SH()}, fcam.eye() + l * 1e5, sp)) vp = sp;
        }
        hyperEffect(W, vp);
    }
    gputime::mark("effects");

    const bool hud = !app.hudHidden;
    if (!hud) {
        // brief hint after hiding the HUD, so the key to bring it back is known
        double age = GetTime() - app.hudToggleTime;
        std::string k = keys::label(keys::A::ToggleHUD);
        if (age < 3.0 && !k.empty()) {
            float a = (float)std::clamp((3.0 - age) / 0.5, 0.0, 1.0);
            std::string msg = trf("HUD hidden  ·  {0} to show", k);
            Vector2 m = measure(msg, 14);
            Rectangle r{(ui::SW() - m.x - 32) / 2, 24, m.x + 32, 32};
            DrawRectangleRounded(r, 1.0f, 12, Fade(Color{14, 18, 26, 230}, a));
            text(msg, r.x + 16, r.y + (r.height - m.y) / 2, 14, Fade(col::text, a));
        }
    }
    if (!W.hyper.active && hud) {
        if (mapMode) bodyListPanel(app, av);
        drawHUD(app, *av);
        {
            nav::OrbitCam& nc = mapMode ? mcam : fcam;
            float SWl = ui::SW(), SHl = ui::SH();
            auto homeFn = [&]() {
                if (mapMode) {
                    double r = mapFocus >= 0 ? W.U.bodies[mapFocus].radius * (W.U.bodies[mapFocus].isStar ? 30 : 6)
                                             : std::max(len(av->r) * 3.0, W.U.bodies[av->ref].radius * 4);
                    mcam.animateTo(mcam.homeYaw, mcam.homePitch, r, {});
                } else fcam.home({}, std::max(av->length, av->maxR * 2) * 0.6 + 1);
            };
            auto fitFn = [&]() {
                if (mapMode) {
                    double r = mapFocus >= 0 ? W.U.bodies[mapFocus].radius * (W.U.bodies[mapFocus].isStar ? 30 : 6)
                                             : std::max(len(av->r) * 3.0, W.U.bodies[av->ref].radius * 4);
                    mcam.animateTo(mcam.yaw, mcam.pitch, r, {});
                } else fcam.fit({}, std::max(av->length, av->maxR * 2) * 0.6 + 1);
            };
            nav::viewCube(nc, {SWl - 240, SHl - 300, 200, 160}, homeFn);
            nav::navBar(nc, {SWl - 150, SHl - 84}, fitFn, homeFn, false);
        }
        if (mapMode && selBody >= 0) bodyInfoPanel(app, av);
        // bottom-left toolbar
        float SH = ui::SH();
        Rectangle tb{12, SH - 64, 11 * 46 + 16, 52};
        panel(tb);
        float bx = tb.x + 8;
        using keys::A;
        // back to the start screen (the game goes on: "Continue" there brings it back)
        if (iconButton({bx, tb.y + 8, 38, 36}, Icon::Home, false, "Home screen")) {
            if (W.warpIndex > 0) W.setWarp(0);
            pauseOpen = false;
            app.screen = Screen::Menu;
            return;
        }
        bx += 46;
        if (iconButton({bx, tb.y + 8, 38, 36}, Icon::Map, mapMode, keys::tip("Map view", A::MapView).c_str())) {
            mapMode = !mapMode;
            if (mapMode && mapFocus < 0) setMapDist(std::max(len(av->r) * 3.0, W.U.bodies[av->ref].radius * 4));
        }
        bx += 46;
        if (iconButton({bx, tb.y + 8, 38, 36}, Icon::Factory, false, "Factory")) { app.screen = Screen::Factory; return; }
        bx += 46;
        if (iconButton({bx, tb.y + 8, 38, 36}, Icon::Ship, vesselsOpen, "Vessels")) vesselsOpen = true;
        bx += 46;
        if (iconButton({bx, tb.y + 8, 38, 36}, Icon::Base, basesOpen, "Surface bases")) basesOpen = true;
        bx += 46;
        if (iconButton({bx, tb.y + 8, 38, 36}, Icon::Hyper, hyperOpen, keys::tip("Hyperspace", A::Hyperspace).c_str())) { hyperOpen = true; hyperSel = av->target; }
        bx += 46;
        if (iconButton({bx, tb.y + 8, 38, 36}, Icon::Info, logOpen, keys::tip("Mission log", A::MissionLog).c_str())) logOpen = !logOpen;
        bx += 46;
        if (iconButton({bx, tb.y + 8, 38, 36}, Icon::Trophy, achOpen, "Achievements")) achOpen = true;
        bx += 46;
        if (iconButton({bx, tb.y + 8, 38, 36}, Icon::Eye, nightVision != 0, keys::tip(nightVision == 0 ? "Night vision: off" : nightVision == 1 ? "Night vision: image intensifier" : "Night vision: bright", A::NightVision).c_str())) {
            cycleNightVision(W);
        }
        bx += 46;
        if (iconButton({bx, tb.y + 8, 38, 36}, Icon::Save, false, keys::tip("Quicksave", A::QuickSave).c_str())) W.toast(quickSave(app) ? "Quicksaved" : "Quicksave failed", col::good);
        bx += 46;
        if (iconButton({bx, tb.y + 8, 38, 36}, Icon::Gear, pauseOpen, "Menu (Esc)")) pauseOpen = true;
        {
            // help text in the free area right of the navball panel
            float SWf = ui::SW();
            float hx = std::max(SWf / 2 + 96 + 260, g_navPanelRight) + 16;
            // built from the current key bindings
            auto k = [](A a) {
                std::string l = keys::label(a);
                return l.empty() ? std::string("-") : l;
            };
            auto two = [&](A a, A b, const char* what) { return k(a) + "/" + k(b) + " " + tr(what) + "  "; };
            auto one = [&](A a, const char* what) { return k(a) + " " + tr(what) + "  "; };
            std::string help;
            if (mapMode)
                help = tr("Right-drag rotate  ·  Wheel zoom  ·  Click body: info  ·  Double-click: focus  ·  Right-click: set target") + "  ·  " +
                       k(A::FocusVessel) + ": " + tr("focus vessel");
            else
                help = two(A::PitchDown, A::PitchUp, "pitch") + two(A::YawLeft, A::YawRight, "yaw") + two(A::RollLeft, A::RollRight, "roll") +
                       two(A::ThrottleUp, A::ThrottleDown, "throttle") + two(A::ThrottleFull, A::ThrottleCut, "full/cut") + one(A::Stage, "stage") +
                       one(A::SAS, "SAS") + one(A::RCS, "RCS") + k(A::TransFwd) + "/" + k(A::TransBack) + " " + k(A::TransUp) + "/" + k(A::TransDown) +
                       " " + k(A::TransLeft) + "/" + k(A::TransRight) + " " + tr("translate") + "  " + one(A::Solar, "solar") +
                       (av->hasFlaps ? one(A::Flaps, "flaps") : std::string()) + (av->bayComp >= 0 ? one(A::BayDoors, "bay doors") : std::string()) +
                       one(A::MissionLog, "log") + one(A::NightVision, "night vision") + one(A::MapView, "map") +
                       k(A::WarpSlower) + " " + k(A::WarpFaster) + " " + tr("warp") + "  " + one(A::ToggleHUD, "HUD") + one(A::Screenshot, "screenshot");
            if (SWf - hx - 16 > 160) wrap(help, hx, SH - 74, SWf - hx - 16, 13, Color{150, 160, 180, 170});
        }
    }
    if (hud && !W.hyper.active) buildBaseButton(app, *av, modal);
    if (hud) toasts(W);
    if (logOpen && hud) {
        // mission log: every notification of the session, newest first
        float SHf = ui::SH();
        Rectangle lg{12, SHf - 64 - 12 - 300, 420, 300};
        panel(lg);
        text("MISSION LOG", lg.x + 16, lg.y + 12, 13, col::dim, true);
        if (iconButton({lg.x + lg.width - 38, lg.y + 8, 28, 28}, Icon::Close)) logOpen = false;
        Rectangle lr{lg.x + 8, lg.y + 38, lg.width - 16, lg.height - 46};
        float off = beginScroll(77, lr, W.missionLog.size() * 40.0f + 8);
        float y = lr.y - off;
        for (auto it = W.missionLog.rbegin(); it != W.missionLog.rend(); ++it) {
            if (y + 40 >= lr.y && y <= lr.y + lr.height) {
                text(formatDate(it->t), lr.x + 8, y + 2, 11, col::faint);
                text(fit(it->text, lr.width - 16, 13), lr.x + 8, y + 17, 13, it->color);
            }
            y += 40;
        }
        endScroll();
        if (W.missionLog.empty()) text("No events yet", lr.x + 8, lr.y + 6, 13, col::faint);
    }

    tut::frame(app);
    if (app.screen != Screen::Flight) return; // the tutorial restarted into the factory
    av = W.activeVessel();                     // a restart replaces the vessels
    if (!av) return;
    vesselMenu(app);
    achievementBanners(app);

    // destroyed overlay
    if (av->destroyed && !modal) {
        dim(0.45f);
        float SW = ui::SW(), SH = ui::SH();
        Rectangle r{(SW - 480) / 2, (SH - 220) / 2, 480, 220};
        panel(r, Color{30, 18, 22, 250}, 14);
        text("VESSEL LOST", r.x + 24, r.y + 20, 24, col::bad, true);
        text(fit(av->fate, r.width - 48, 15), r.x + 24, r.y + 62, 15, col::text);
        double reach = W.rewindReach();
        float bw3 = (r.width - 48 - 16) / 3;
        if (button({r.x + 24, r.y + 100, bw3, 40}, "Back 30 s", false, reach >= 0.5, Icon::Backward)) W.rewind(30);
        else if (button({r.x + 24 + bw3 + 8, r.y + 100, bw3, 40}, "Back 3 min", false, reach >= 0.5, Icon::Backward)) W.rewind(180);
        else if (button({r.x + 24, r.y + 150, 200, 44}, "Revert to launch", false, !app.lastLaunched.comps.empty(), Icon::Rocket)) {
            revertToLaunch(app);
        } else if (button({r.x + 256, r.y + 150, 200, 44}, "Back to factory", false, true, Icon::Factory)) {
            W.vessels.erase(W.vessels.begin() + W.active);
            W.active = -1;
            for (int i = 0; i < (int)W.vessels.size(); i++)
                if (!W.vessels[i].destroyed) { W.active = i; break; }
            app.screen = Screen::Factory;
            return;
        }
    }

    ui::setInputBlocked(app.settingsOpen);
    av = W.activeVessel(); // a rewind or revert above replaced the vessel list
    if (!av) return;
    hyperModal(app, av);
    vesselsModal(app);
    basesModal(app);
    buildModal(app);
    achievementsModal(app);
    recoverModal(app);
    if (pauseOpen && !app.settingsOpen) {
        dim();
        float SW = ui::SW(), SH = ui::SH();
        Rectangle r{(SW - 360) / 2, (SH - 580) / 2, 360, 580};
        panel(r, Color{20, 26, 36, 252}, 14);
        text("Paused", r.x + 24, r.y + 18, 24, col::text, true);
        float y = r.y + 70;
        {
            // rewind the flight (history since the launch)
            double reach = W.rewindReach();
            float bw = (r.width - 48 - 8) / 2;
            if (button({r.x + 24, y, bw, 42}, "Back 30 s", false, reach >= 0.5, Icon::Backward)) {
                if (W.rewind(30)) pauseOpen = false;
            }
            if (button({r.x + 24 + bw + 8, y, bw, 42}, "Back 3 min", false, reach >= 0.5, Icon::Backward)) {
                if (W.rewind(180)) pauseOpen = false;
            }
            y += 50;
        }
        auto item = [&](const char* label, Icon ic, bool enabled = true) {
            bool b = button({r.x + 24, y, r.width - 48, 42}, label, false, enabled, ic);
            y += 50;
            return b;
        };
        if (item("Resume", Icon::Play)) pauseOpen = false;
        if (item("Revert to launch", Icon::Rocket, !app.lastLaunched.comps.empty())) {
            pauseOpen = false;
            revertToLaunch(app);
            return;
        }
        if (item("Achievements", Icon::Trophy)) {
            pauseOpen = false;
            achOpen = true;
        }
        if (item("Save game", Icon::Save)) W.toast(quickSave(app) ? "Game saved" : "Save failed", col::good);
        if (item("Load game", Icon::Folder)) {
            if ((finishSaves(), loadWorld(W, app.saveFile))) {
                if (!app.mapPath.empty()) markMapSaved(app);
                W.toast("Game loaded", col::good);
                flightOnEnter(app);
                return;
            }
            W.toast("No saved game found", col::bad);
        }
        if (item("Settings", Icon::Gear)) app.settingsOpen = true;
        if (item("Return to factory", Icon::Factory)) { pauseOpen = false; app.screen = Screen::Factory; return; }
        if (item("Main menu", Icon::Exit)) { pauseOpen = false; app.screen = Screen::Menu; return; }
    }
    ui::setInputBlocked(false);
    if (app.settingsOpen) settingsModal(app);
    gputime::mark("hud");
}
