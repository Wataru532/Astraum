#include "terrain.hpp"
#include <algorithm>
#include <atomic>
#include <memory>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <random>
#include <set>
#include <string>
#include <thread>
#include "rl.hpp"
#include "sim.hpp"
#include "universe.hpp"
#include "winshell.hpp"
#include "features.hpp"

namespace terrain {
namespace {

// resource ids (app.rc): colour JPEG, elevation PNG (16 bit code in R high / G low byte)
struct Source {
    const char* body;
    int colorId, heightId;
};
const Source SOURCES[] = {{"Earth", 201, 202}, {"Moon", 203, 204}, {"Mars", 205, 206}};
// colour maps only (tools/make_body_maps.py, assets/maps/SOURCES.txt): the relief of these bodies is generated, their
// face is the spacecraft mosaic; a gas giant gets a flat map that carries the colour
const Source COLOR_ONLY[] = {{"Mercury", 231, 0}, {"Io", 232, 0}, {"Europa", 233, 0}, {"Ganymede", 234, 0}, {"Callisto", 235, 0},
                             {"Enceladus", 236, 0}, {"Triton", 237, 0}, {"Pluto", 238, 0}, {"Charon", 239, 0}, {"Ceres", 240, 0},
                             {"Phobos", 241, 0}, {"Jupiter", 242, 0}};
std::vector<std::vector<unsigned char>> g_color; // body -> colour JPEG of a body whose relief is generated
constexpr double H_OFS = -12000.0, H_STEP = 0.6; // tools/make_maps.py

std::vector<Map> g_maps;        // one per source that loaded / body generated
std::vector<int> g_byBody;      // body index -> map index (-1: none)
struct Gen {
    bool want = false;
    double radius = 0;
    float seed = 0;
    int surf = 0;
    std::string name; // (its named features are carved in)
};
std::vector<Gen> g_gen;         // bodies whose relief is generated on first use

bool decodeHeight(const void* data, size_t size, Map& m) {
    Image img = LoadImageFromMemory(".png", (const unsigned char*)data, (int)size);
    if (!img.data) return false;
    if (img.format != PIXELFORMAT_UNCOMPRESSED_R8G8B8 && img.format != PIXELFORMAT_UNCOMPRESSED_R8G8B8A8) {
        UnloadImage(img);
        return false;
    }
    int ch = img.format == PIXELFORMAT_UNCOMPRESSED_R8G8B8 ? 3 : 4;
    m.w = img.width;
    m.h = img.height;
    m.height.resize((size_t)m.w * m.h);
    const unsigned char* p = (const unsigned char*)img.data;
    m.hMin = 1e9f;
    m.hMax = -1e9f;
    for (size_t i = 0; i < m.height.size(); i++) {
        double code = p[i * ch] * 256.0 + p[i * ch + 1];
        float v = (float)(code * H_STEP + H_OFS);
        m.height[i] = v;
        m.hMin = std::min(m.hMin, v);
        m.hMax = std::max(m.hMax, v);
    }
    UnloadImage(img);
    return m.w > 1 && m.h > 1;
}

double sample(const Map& m, double u, double v) {
    // texel centres at (i + 0.5) / w, as GL_LINEAR does; longitude wraps, latitude clamps
    double x = u * m.w - 0.5, y = std::clamp(v * m.h - 0.5, 0.0, m.h - 1.0);
    double fx = std::floor(x), fy = std::floor(y);
    double ax = x - fx, ay = y - fy;
    int x0 = ((int)fx % m.w + m.w) % m.w, x1 = (x0 + 1) % m.w;
    int y0 = (int)fy, y1 = std::min(y0 + 1, m.h - 1);
    const float* r0 = &m.height[(size_t)y0 * m.w];
    const float* r1 = &m.height[(size_t)y1 * m.w];
    double a = r0[x0] + (r0[x1] - r0[x0]) * ax;
    double b = r1[x0] + (r1[x1] - r1[x0]) * ax;
    return a + (b - a) * ay;
}

// direction of the centre of pixel (x, y) of an equirectangular map of fw x fh pixels
Vec3d pixelDir(int x, int y, int fw, int fh) {
    double lon = ((x + 0.5) / fw - 0.5) * TAU, colat = (y + 0.5) / fh * PI;
    return Vec3d{std::sin(colat) * std::cos(lon), std::sin(colat) * std::sin(lon), std::cos(colat)};
}
// visit the pixels of the part (ox, oy, w, h) of a fw x fh map within `reach` [rad] of a launch site
template <class F> void nearLaunchSites(int w, int h, int ox, int oy, int fw, int fh, double reach, F f) {
    for (int s = 0; s < LAUNCH_SITE_COUNT; s++) {
        double lat = LAUNCH_SITES[s].latDeg * DEG, lon = LAUNCH_SITES[s].lonDeg * DEG;
        Vec3d c{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
        double cy = (PI / 2 - lat) / PI * fh, cx = (lon / TAU + 0.5) * fw;
        double ry = reach / PI * fh + 2, rx = std::min(fw / 2.0, reach / std::max(std::cos(lat), 0.05) / TAU * fw + 2);
        for (int y = std::max(oy, (int)(cy - ry)); y < std::min(oy + h, (int)(cy + ry) + 1); y++)
            for (int k = (int)(cx - rx); k <= (int)(cx + rx); k++) {
                int x = ((k % fw) + fw) % fw;
                if (x < ox || x >= ox + w) continue;
                double ang = std::acos(std::clamp(dot(pixelDir(x, y, fw, fh), c), -1.0, 1.0));
                if (ang < reach) f(x - ox, y - oy, ang);
            }
    }
}
// the launch complexes stand at sea level: the ground is flattened around every launch site - only the pad
// and the runway (about 7 km, blending out by 14 km), but at least 1.5 texels of the map so that the pad samples
// zero on a coarse map too (the old 30 / 78 km reach squashed the hills of whole islands, Yakushima's too)
constexpr double FLAT_IN = 0.066 * DEG, FLAT_OUT = 0.132 * DEG; // (the fine relief below the maps)
void flattenHeights(float* hgt, int w, int h, int ox, int oy, int fw, int fh) {
    double px = PI / std::max(fh, 1);
    double in = std::max(FLAT_IN, 1.5 * px), out = in + std::max(FLAT_OUT - FLAT_IN, 1.5 * px);
    nearLaunchSites(w, h, ox, oy, fw, fh, out, [&](int x, int y, double ang) {
        float& v = hgt[(size_t)y * w + x];
        v = (float)(v * std::clamp((ang - in) / (out - in), 0.0, 1.0));
    });
}
void flattenLaunchSites(Map& m) { flattenHeights(m.height.data(), m.w, m.h, 0, 0, m.w, m.h); }

// ---- generated relief: value-noise hills and a crater population, deterministic per body
uint32_t hash3(int x, int y, int z, uint32_t s) {
    uint32_t h = (uint32_t)x * 0x8da6b343u ^ (uint32_t)y * 0xd8163841u ^ (uint32_t)z * 0xcb1ab31fu ^ s * 0x9e3779b9u;
    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return h;
}
double vnoise(const Vec3d& p, uint32_t s) {
    int x = (int)std::floor(p.x), y = (int)std::floor(p.y), z = (int)std::floor(p.z);
    double fx = p.x - x, fy = p.y - y, fz = p.z - z;
    auto sm = [](double t) { return t * t * (3 - 2 * t); };
    double u = sm(fx), v = sm(fy), w = sm(fz);
    auto g = [&](int i, int j, int k) { return (hash3(x + i, y + j, z + k, s) & 0xffff) / 65535.0; };
    double a = g(0, 0, 0) + (g(1, 0, 0) - g(0, 0, 0)) * u, b = g(0, 1, 0) + (g(1, 1, 0) - g(0, 1, 0)) * u;
    double c = g(0, 0, 1) + (g(1, 0, 1) - g(0, 0, 1)) * u, d = g(0, 1, 1) + (g(1, 1, 1) - g(0, 1, 1)) * u;
    double e = a + (b - a) * v, f = c + (d - c) * v;
    return e + (f - e) * w;
}
void generate(const Gen& g, Map& m) {
    m.w = 1024;
    m.h = 512;
    m.generated = true;
    m.height.assign((size_t)m.w * m.h, 0.f);
    const bool icy = g.surf == (int)Surf::Icy, volcanic = g.surf == (int)Surf::Desert;
    // relief amplitude: small bodies are lumpy (a few % of the radius), large ones a few km
    double amp = g.radius < 400e3 ? 0.05 * g.radius : std::clamp(0.0018 * g.radius, 1500.0, 6000.0);
    if (icy) amp *= 0.45;
    uint32_t seed = (uint32_t)(g.seed * 1000.0f) + 17u;
    auto dirOf = [&](int x, int y) {
        double lon = ((x + 0.5) / m.w - 0.5) * TAU, colat = (y + 0.5) / m.h * PI;
        return Vec3d{std::sin(colat) * std::cos(lon), std::sin(colat) * std::sin(lon), std::cos(colat)};
    };
    // rolling hills / basins
    for (int y = 0; y < m.h; y++)
        for (int x = 0; x < m.w; x++) {
            Vec3d p = dirOf(x, y) * 3.0;
            double s = 0, a = 0.5, n = 0;
            for (int o = 0; o < 5; o++) {
                s += a * vnoise(p, seed + o);
                n += a;
                p = p * 2.07;
                a *= 0.5;
            }
            m.height[(size_t)y * m.w + x] = (float)((s / n - 0.5) * amp * 1.2);
        }
    // craters: power-law sizes (many small, few large); bowl, raised rim and ejecta apron
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> U01(0, 1);
    int count = icy ? 120 : volcanic ? 70 : 700;
    for (int k = 0; k < count; k++) {
        Vec3d c = norm(Vec3d{U01(rng) * 2 - 1, U01(rng) * 2 - 1, U01(rng) * 2 - 1} + Vec3d{1e-9, 0, 0});
        double r = 0.006 * std::pow(0.3 / 0.006, std::pow(U01(rng), 3.0)); // angular radius [rad]
        double D = 2 * r * g.radius;
        double depth = std::min(0.18 * D, amp * 1.4), rim = 0.22 * depth;
        double clat = std::asin(std::clamp(c.z, -1.0, 1.0)), clon = std::atan2(c.y, c.x);
        double reach = r * 2.0;
        int y0 = std::max(0, (int)(((PI / 2 - clat) - reach) / PI * m.h)), y1 = std::min(m.h - 1, (int)(((PI / 2 - clat) + reach) / PI * m.h) + 1);
        for (int y = y0; y <= y1; y++) {
            double colat = (y + 0.5) / m.h * PI;
            double sl = std::max(std::sin(colat), 1e-3);
            int span = (int)std::min((double)m.w / 2, reach / sl / TAU * m.w + 2);
            int xc = (int)((clon / TAU + 0.5) * m.w);
            for (int dx = -span; dx <= span; dx++) {
                int x = ((xc + dx) % m.w + m.w) % m.w;
                double ang = std::acos(std::clamp(dot(dirOf(x, y), c), -1.0, 1.0));
                double q = ang / r;
                if (q >= 2.0) continue;
                double dh = q < 1.0 ? depth * (q * q - 1) + rim * std::pow(q, 8) : rim * std::pow(1 - (q - 1), 3) * (1 - (q - 1) * 0.3);
                m.height[(size_t)y * m.w + x] += (float)dh;
            }
        }
    }
    // the named features of the body (features.cpp) at their real places: basins and craters with their
    // rims, mountains as broad domes; after the random ones, so the random relief stays as it was
    int nf = 0;
    const features::Feature* F = features::all(nf);
    for (int k = 0; k < nf; k++) {
        const features::Feature& f = F[k];
        if (f.carve <= 0 || g.name != f.body) continue;
        double la = f.lat * DEG, lo = f.lon * DEG;
        Vec3d c{std::cos(la) * std::cos(lo), std::cos(la) * std::sin(lo), std::sin(la)};
        double r = f.radiusKm * 1000 / g.radius; // angular radius
        bool mons = f.kind == features::Mons;
        double reach = r * (mons ? 2.5 : 2.0);
        // the level around the feature (the random relief there): the floor is cut below it
        double around = 0;
        {
            Vec3d e = norm(cross(Vec3d{0, 0, 1}, c) + Vec3d{1e-9, 0, 0}), n = cross(c, e);
            for (int i = 0; i < 32; i++) {
                double a = i * TAU / 32;
                Vec3d d = norm(c * std::cos(1.5 * r) + (e * std::cos(a) + n * std::sin(a)) * std::sin(1.5 * r));
                around += sample(m, std::atan2(d.y, d.x) / TAU + 0.5, std::acos(std::clamp(d.z, -1.0, 1.0)) / PI);
            }
            around /= 32;
        }
        int y0 = std::max(0, (int)(((PI / 2 - la) - reach) / PI * m.h)), y1 = std::min(m.h - 1, (int)(((PI / 2 - la) + reach) / PI * m.h) + 1);
        for (int y = y0; y <= y1; y++) {
            double colat = (y + 0.5) / m.h * PI;
            double sl = std::max(std::sin(colat), 1e-3);
            int span = (int)std::min((double)m.w / 2, reach / sl / TAU * m.w + 2);
            int xc = (int)((lo / TAU + 0.5) * m.w);
            for (int dx = -span; dx <= span; dx++) {
                int x = ((xc + dx) % m.w + m.w) % m.w;
                double q = std::acos(std::clamp(dot(dirOf(x, y), c), -1.0, 1.0)) / r;
                float& hv = m.height[(size_t)y * m.w + x];
                // the floor of a basin or crater is smooth plains (Caloris, Sputnik): the random hills fade out
                if (!mons && q < 1.3) {
                    double w = std::clamp((1.3 - q) / 0.5, 0.0, 1.0);
                    w = 0.85 * w * w * (3 - 2 * w);
                    hv = (float)(hv * (1 - w) + around * w);
                }
                double dh;
                if (mons) dh = f.carve * std::exp(-q * q * 2.2);
                else if (q < 1.0) {
                    // a basin keeps a flatter floor than a crater bowl
                    double bowl = f.kind == features::Basin ? std::pow(q, 4.0) - 1 : q * q - 1;
                    dh = f.carve * bowl + 0.25 * f.carve * std::pow(q, 8);
                } else if (q < 2.0) {
                    dh = 0.25 * f.carve * std::pow(1 - (q - 1), 3);
                    if (f.kind == features::Basin && f.carve < 500) dh += 0.4 * f.carve * std::cos((q - 1) * 3 * TAU) * (2 - q); // (rings)
                } else dh = 0;
                hv += (float)dh;
            }
        }
    }
    m.hMin = *std::min_element(m.height.begin(), m.height.end());
    m.hMax = *std::max_element(m.height.begin(), m.height.end());
}

// ---------------------------------------------------------------- high-resolution tiles
struct Pack {
    const unsigned char* data = nullptr;
    size_t size = 0;
    int gw = 0, gh = 0, tw = 0, th = 0, fw = 0, fh = 0;
    const unsigned char* tab = nullptr;
    bool tile(int key, const unsigned char*& p, size_t& n) const {
        if (!data || key < 0 || key >= gw * gh) return false;
        uint32_t o, sz;
        std::memcpy(&o, tab + key * 8, 4);
        std::memcpy(&sz, tab + key * 8 + 4, 4);
        if ((size_t)o + sz > size || sz == 0) return false;
        p = data + o;
        n = sz;
        return true;
    }
};
bool openPack(int id, int kind, Pack& pk) {
    const void* d = nullptr;
    size_t n = 0;
    if (!winshell::resource(id, d, n) || n < 40) return false;
    const unsigned char* b = (const unsigned char*)d;
    uint32_t hd[10];
    std::memcpy(hd, b, 40);
    if (std::memcmp(b, "SFTP", 4) != 0 || hd[1] != 1 || (int)hd[2] != kind || (int)hd[3] != DET_GW || (int)hd[4] != DET_GH) return false;
    if (hd[5] == 0 || hd[6] == 0 || hd[7] != hd[5] * DET_GW || hd[8] != hd[6] * DET_GH || 40 + (size_t)DET_GW * DET_GH * 8 > n) return false;
    pk.data = b;
    pk.size = n;
    pk.gw = DET_GW;
    pk.gh = DET_GH;
    pk.tw = (int)hd[5];
    pk.th = (int)hd[6];
    pk.fw = (int)hd[7];
    pk.fh = (int)hd[8];
    pk.tab = b + 40;
    return true;
}
struct HiSrc {
    int body = -1;
    double radius = 0;
    bool earth = false;
    Pack color, height;
};
std::vector<HiSrc> g_hi;
struct Slot {
    int key = -1; // tile in this window slot (ty * DET_GW + tx), -1 none
    std::vector<float> h;
};
struct DetState {
    bool active = false;
    int src = -1;
    Detail d;
    unsigned gen = 0, rev = 0;
    Slot slots[DET_WIN * DET_WIN]; // elevation tiles, slot = (ty % 4) * 4 + tx % 4
    std::set<int> queuedH, queuedC, deliveredC;
    std::deque<std::pair<int, std::vector<unsigned char>>> colorOut;
    std::deque<int> heightOut;
};
std::vector<DetState> g_det; // by body

int slotOf(int key) { return ((key / DET_GW) % DET_WIN) * DET_WIN + (key % DET_GW) % DET_WIN; }
bool inWindow(const Detail& d, int key) {
    int tx = key % DET_GW, ty = key / DET_GW;
    int lx = ((tx - d.tx0) % DET_GW + DET_GW) % DET_GW;
    return lx < DET_WIN && ty >= d.ty0 && ty < d.ty0 + DET_WIN;
}

bool decodeColorTile(const HiSrc& s, int key, std::vector<unsigned char>& rgb) {
    const unsigned char* p;
    size_t n;
    if (!s.color.tile(key, p, n)) return false;
    Image img = LoadImageFromMemory(".jpg", p, (int)n);
    if (!img.data) return false;
    ImageFormat(&img, PIXELFORMAT_UNCOMPRESSED_R8G8B8);
    bool ok = img.width == s.color.tw && img.height == s.color.th;
    if (ok) {
        rgb.assign((const unsigned char*)img.data, (const unsigned char*)img.data + (size_t)img.width * img.height * 3);
        if (s.earth) paintLaunchSites(rgb.data(), s.color.tw, s.color.th, (key % DET_GW) * s.color.tw, (key / DET_GW) * s.color.th, s.color.fw, s.color.fh);
    }
    UnloadImage(img);
    return ok;
}
bool decodeHeightTile(const HiSrc& s, int key, std::vector<float>& h) {
    const unsigned char* p;
    size_t n;
    if (!s.height.tile(key, p, n)) return false;
    Map m;
    if (!decodeHeight(p, n, m) || m.w != s.height.tw || m.h != s.height.th) return false;
    h = std::move(m.height);
    if (s.earth) flattenHeights(h.data(), s.height.tw, s.height.th, (key % DET_GW) * s.height.tw, (key / DET_GW) * s.height.th, s.height.fw, s.height.fh);
    return true;
}

// ---- the worker thread: decodes tiles in the background
struct Job {
    int body, key;
    bool color;
    unsigned gen;
};
struct Res {
    int body, key;
    bool color, ok;
    unsigned gen;
    std::vector<unsigned char> rgb;
    std::vector<float> h;
};
struct Worker {
    std::mutex mx;
    std::condition_variable cv;
    std::deque<Job> jobs;
    std::deque<Res> done;
    std::thread th;
    bool stop = false;
    void start() {
        if (th.joinable()) return;
        th = std::thread([this] {
            for (;;) {
                Job j;
                {
                    std::unique_lock<std::mutex> lk(mx);
                    cv.wait(lk, [this] { return stop || !jobs.empty(); });
                    if (stop) return;
                    j = jobs.front();
                    jobs.pop_front();
                }
                Res r{j.body, j.key, j.color, false, j.gen, {}, {}};
                const DetState& st = g_det[j.body];
                if (st.src >= 0) {
                    const HiSrc& s = g_hi[st.src];
                    r.ok = j.color ? decodeColorTile(s, j.key, r.rgb) : decodeHeightTile(s, j.key, r.h);
                }
                std::lock_guard<std::mutex> lk(mx);
                done.push_back(std::move(r));
            }
        });
    }
    ~Worker() {
        {
            std::lock_guard<std::mutex> lk(mx);
            stop = true;
        }
        cv.notify_all();
        if (th.joinable()) th.join();
    }
};
Worker g_worker;

void queueJob(int body, int key, bool color, unsigned gen) {
    g_worker.start();
    {
        std::lock_guard<std::mutex> lk(g_worker.mx);
        g_worker.jobs.push_back({body, key, color, gen});
    }
    g_worker.cv.notify_one();
}
void putHeight(DetState& st, int key, std::vector<float>&& h) {
    Slot& sl = st.slots[slotOf(key)];
    sl.key = key;
    sl.h = std::move(h);
    st.queuedH.erase(key);
    st.heightOut.push_back(key);
    st.rev++;
}
void pollResults() {
    std::deque<Res> got;
    {
        std::lock_guard<std::mutex> lk(g_worker.mx);
        got.swap(g_worker.done);
    }
    for (Res& r : got) {
        DetState& st = g_det[r.body];
        if (!st.active || r.gen != st.gen || !inWindow(st.d, r.key)) continue; // the window moved on
        if (r.color) {
            st.queuedC.erase(r.key);
            if (r.ok) {
                st.deliveredC.insert(r.key);
                st.colorOut.push_back({r.key, std::move(r.rgb)});
            }
        } else if (r.ok && st.slots[slotOf(r.key)].key != r.key) putHeight(st, r.key, std::move(r.h));
        else st.queuedH.erase(r.key);
    }
}
void dropWindow(DetState& st) {
    bool had = false;
    for (Slot& sl : st.slots) {
        if (sl.key >= 0) had = true;
        sl.key = -1;
        sl.h.clear();
        sl.h.shrink_to_fit();
    }
    st.active = false;
    st.gen++;
    st.queuedH.clear();
    st.queuedC.clear();
    st.deliveredC.clear();
    st.colorOut.clear();
    st.heightOut.clear();
    if (had) st.rev++;
}
// place the window so that tile position (fx, fy) lies near its middle; keep the tiles it still covers
void placeWindow(DetState& st, double fx, double fy) {
    const HiSrc& s = g_hi[st.src];
    st.d.tx0 = (((int)std::floor(fx + 0.5) - DET_WIN / 2) % DET_GW + DET_GW) % DET_GW;
    st.d.ty0 = std::clamp((int)std::floor(fy + 0.5) - DET_WIN / 2, 0, DET_GH - DET_WIN);
    st.gen++;
    for (Slot& sl : st.slots)
        if (sl.key >= 0 && !inWindow(st.d, sl.key)) {
            sl.key = -1;
            sl.h.clear();
            st.rev++;
        }
    for (auto it = st.deliveredC.begin(); it != st.deliveredC.end();) it = inWindow(st.d, *it) ? std::next(it) : st.deliveredC.erase(it);
    st.queuedH.clear();
    st.queuedC.clear();
    st.colorOut.clear();
    st.heightOut.clear();
    // the tiles nearest the middle first
    std::vector<std::pair<double, int>> order;
    for (int j = 0; j < DET_WIN; j++)
        for (int i = 0; i < DET_WIN; i++) {
            int tx = (st.d.tx0 + i) % DET_GW, ty = st.d.ty0 + j;
            double dx = std::fabs(i + 0.5 - (fx - st.d.tx0 - std::floor((fx - st.d.tx0) / DET_GW) * DET_GW)), dy = std::fabs(ty + 0.5 - fy);
            order.push_back({dx * dx + dy * dy, ty * DET_GW + tx});
        }
    std::sort(order.begin(), order.end());
    for (auto& o : order) {
        int key = o.second;
        if (s.height.data && st.slots[slotOf(key)].key != key) {
            st.queuedH.insert(key);
            queueJob((int)(&st - g_det.data()), key, false, st.gen);
        } else if (st.slots[slotOf(key)].key == key)
            st.heightOut.push_back(key); // (still there: the renderer keeps or re-uploads it)
        if (s.color.data && !st.deliveredC.count(key)) {
            st.queuedC.insert(key);
            queueJob((int)(&st - g_det.data()), key, true, st.gen);
        }
    }
    st.active = true;
}
// elevation from the tiles (bilinear as the GPU), with the blend weight toward the window border
bool detailSample(const DetState& st, double u, double v, double& out, double& fade) {
    const HiSrc& s = g_hi[st.src];
    const Pack& pk = s.height;
    if (!pk.data) return false;
    double fx = u * DET_GW, fy = v * DET_GH;
    double lx = fx - st.d.tx0;
    lx -= std::floor(lx / DET_GW) * DET_GW;
    double ly = fy - st.d.ty0;
    if (lx >= DET_WIN || ly < 0 || ly >= DET_WIN) return false;
    double ex = std::min(lx, DET_WIN - lx);
    double ey = std::min(st.d.ty0 == 0 ? 1e9 : ly, st.d.ty0 + DET_WIN == DET_GH ? 1e9 : DET_WIN - ly); // (the poles are no border)
    double e = std::min(ex, ey);
    fade = std::clamp(e / 0.5, 0.0, 1.0);
    fade = fade * fade * (3 - 2 * fade);
    double x = u * pk.fw - 0.5, y = std::clamp(v * pk.fh - 0.5, 0.0, pk.fh - 1.0);
    double fxp = std::floor(x), fyp = std::floor(y);
    double ax = x - fxp, ay = y - fyp;
    int xs[2] = {(((int)fxp % pk.fw) + pk.fw) % pk.fw, 0}, ys[2] = {(int)fyp, std::min((int)fyp + 1, pk.fh - 1)};
    xs[1] = (xs[0] + 1) % pk.fw;
    double t[2][2];
    for (int j = 0; j < 2; j++)
        for (int i = 0; i < 2; i++) {
            int key = (ys[j] / pk.th) * DET_GW + xs[i] / pk.tw;
            const Slot& sl = st.slots[slotOf(key)];
            if (sl.key != key) return false;
            t[j][i] = sl.h[(size_t)(ys[j] % pk.th) * pk.tw + xs[i] % pk.tw];
        }
    double a = t[0][0] + (t[0][1] - t[0][0]) * ax, b = t[1][0] + (t[1][1] - t[1][0]) * ax;
    out = a + (b - a) * ay;
    return true;
}
// ---------------------------------------------------------------- fine relief (below the maps)
// gradient noise on a 3D lattice (body-fixed metres / wavelength), deterministic
double gnoise(double x, double y, double z, uint32_t seed) {
    int ix = (int)std::floor(x), iy = (int)std::floor(y), iz = (int)std::floor(z);
    double fx = x - ix, fy = y - iy, fz = z - iz;
    auto fade = [](double t) { return t * t * t * (t * (t * 6 - 15) + 10); };
    double u = fade(fx), v = fade(fy), w = fade(fz);
    auto grad = [&](int i, int j, int k, double dx, double dy, double dz) {
        uint32_t h = hash3(ix + i, iy + j, iz + k, seed);
        double gx = ((h & 0xff) / 127.5) - 1, gy = (((h >> 8) & 0xff) / 127.5) - 1, gz = (((h >> 16) & 0xff) / 127.5) - 1;
        return gx * dx + gy * dy + gz * dz;
    };
    double n000 = grad(0, 0, 0, fx, fy, fz), n100 = grad(1, 0, 0, fx - 1, fy, fz), n010 = grad(0, 1, 0, fx, fy - 1, fz),
           n110 = grad(1, 1, 0, fx - 1, fy - 1, fz), n001 = grad(0, 0, 1, fx, fy, fz - 1), n101 = grad(1, 0, 1, fx - 1, fy, fz - 1),
           n011 = grad(0, 1, 1, fx, fy - 1, fz - 1), n111 = grad(1, 1, 1, fx - 1, fy - 1, fz - 1);
    double x00 = n000 + (n100 - n000) * u, x10 = n010 + (n110 - n010) * u, x01 = n001 + (n101 - n001) * u, x11 = n011 + (n111 - n011) * u;
    double y0 = x00 + (x10 - x00) * v, y1 = x01 + (x11 - x01) * v;
    return y0 + (y1 - y0) * w; // about -0.7..0.7
}
// small craters of one size class: at most one per cell of size `cell` [m] (a jittered grid on the sphere,
// in body-fixed metres), fresh bowls with a raised rim
double craterField(const Vec3d& p, double cell, uint32_t seed, double fill) {
    double gx = p.x / cell, gy = p.y / cell, gz = p.z / cell;
    int ix = (int)std::floor(gx), iy = (int)std::floor(gy), iz = (int)std::floor(gz);
    // (a crater reaches half a cell at most: the cell of the point and its neighbours on the near side)
    int sx = gx - ix < 0.5 ? -1 : 1, sy = gy - iy < 0.5 ? -1 : 1, sz = gz - iz < 0.5 ? -1 : 1;
    double h = 0;
    for (int dz = std::min(0, sz); dz <= std::max(0, sz); dz++)
        for (int dy = std::min(0, sy); dy <= std::max(0, sy); dy++)
            for (int dx = std::min(0, sx); dx <= std::max(0, sx); dx++) {
                uint32_t hh = hash3(ix + dx, iy + dy, iz + dz, seed);
                if ((hh & 1023) / 1023.0 > fill) continue;
                uint32_t h2 = hash3(ix + dx, iy + dy, iz + dz, seed ^ 0x51ed27u);
                Vec3d c{(ix + dx + ((h2 & 255) / 255.0)) * cell, (iy + dy + (((h2 >> 8) & 255) / 255.0)) * cell,
                        (iz + dz + (((h2 >> 16) & 255) / 255.0)) * cell};
                double R = cell * (0.06 + 0.19 * ((hh >> 10) & 255) / 255.0);
                double d = len(p - c) / R;
                if (d >= 2.0) continue;
                double depth = 0.2 * 2 * R;
                h += d < 1.0 ? depth * (d * d - 1) + 0.25 * depth * std::pow(d, 6) : 0.25 * depth * std::pow(2.0 - d, 3);
            }
    return h;
}

struct MicroBody {
    bool on = false, airless = false, sea = false;
    double lambda0 = 4000; // the longest wavelength of the fine relief (about 2 texels of the map) [m]
    double R = 1;
    uint32_t seed = 0;
};
std::vector<MicroBody> g_micro; // by body
// levelled ground around the surface bases: per body an immutable list, swapped atomically (the cap mesh
// threads read it); the lists replaced are kept until the next init (a few per game)
struct FlatSpot { Vec3d n; double in, out, cosOut; };
using FlatList = std::vector<FlatSpot>;
std::unique_ptr<std::atomic<const FlatList*>[]> g_flat;
int g_flatN = 0;
std::deque<std::unique_ptr<FlatList>> g_flatStore;
std::vector<unsigned> g_flatRev;
} // namespace

void setFlatSpots(int body, const std::vector<Vec3d>& centres, double radius) {
    if (body < 0 || body >= g_flatN || body >= (int)g_micro.size()) return;
    const FlatList* cur = g_flat[body].load();
    double R = std::max(g_micro[body].R, 1.0);
    double in = radius / R, out = in * 1.6 + 200.0 / R;
    auto next = std::make_unique<FlatList>();
    for (const Vec3d& c : centres) next->push_back({norm(c), in, out, std::cos(out)});
    bool same = cur ? cur->size() == next->size() : next->empty();
    if (same && cur)
        for (size_t i = 0; i < cur->size(); i++) same &= len((*cur)[i].n - (*next)[i].n) < 1e-12 && std::fabs((*cur)[i].in - in) < 1e-15;
    if (same) return;
    g_flat[body].store(next->empty() ? nullptr : next.get());
    if (!next->empty()) g_flatStore.push_back(std::move(next));
    g_flatRev[body]++;
}

void init(const Universe& U) {
    g_micro.assign(U.count(), MicroBody{});
    g_flat = std::make_unique<std::atomic<const FlatList*>[]>(U.count());
    for (int i = 0; i < U.count(); i++) g_flat[i].store(nullptr);
    g_flatN = U.count();
    g_flatStore.clear();
    g_flatRev.assign(U.count(), 0);
    for (int i = 0; i < U.count(); i++) {
        const Body& b = U.bodies[i];
        bool solid = b.surf == Surf::Rocky || b.surf == Surf::Icy || b.surf == Surf::Desert || b.surf == Surf::Earth;
        if (!solid || b.isStar || b.gasGiant || b.radius <= 0) continue;
        MicroBody& mb = g_micro[i];
        mb.on = true;
        mb.R = b.radius;
        mb.airless = b.atmTop <= 0 || b.atmRho0 < 0.05; // (Mars: thin air, craters kept by the dust but softened)
        mb.sea = b.name == "Earth";
        mb.seed = (uint32_t)(b.seed * 7919.0f) + 101u;
        // 2 texels of the detailed map (Earth 8192 px, Moon / Mars 5760), of the generated relief elsewhere (1024)
        double px = b.name == "Earth" ? 8192.0 : (b.name == "Moon" || b.name == "Mars") ? 5760.0 : 1024.0;
        mb.lambda0 = std::max(200.0, 2.0 * TAU * b.radius / px);
    }
    g_maps.clear();
    g_maps.reserve(64); // pointers handed out stay valid while relief is generated later
    g_byBody.assign(U.count(), -1);
    g_gen.assign(U.count(), Gen{});
    for (int i = 0; i < U.count(); i++) {
        const Body& b = U.bodies[i];
        bool solid = b.surf == Surf::Rocky || b.surf == Surf::Icy || b.surf == Surf::Desert;
        if (solid && !b.isStar && !b.gasGiant && b.radius > 0) g_gen[i] = Gen{true, b.radius, b.seed, (int)b.surf, b.name};
    }
    for (const Source& s : SOURCES) {
        int body = U.find(s.body);
        if (body < 0) continue;
        const void* hd = nullptr;
        const void* cd = nullptr;
        size_t hs = 0, cs = 0;
        if (!winshell::resource(s.heightId, hd, hs) || !winshell::resource(s.colorId, cd, cs)) continue;
        Map m;
        if (!decodeHeight(hd, hs, m)) continue;
        m.colorFile.assign((const unsigned char*)cd, (const unsigned char*)cd + cs);
        if (std::string(s.body) == "Earth") flattenLaunchSites(m);
        g_byBody[body] = (int)g_maps.size();
        g_maps.push_back(std::move(m));
    }
    g_color.assign(U.count(), {});
    for (const Source& s : COLOR_ONLY) {
        int body = U.find(s.body);
        const void* cd = nullptr;
        size_t cs = 0;
        if (body < 0 || g_byBody[body] >= 0 || !winshell::resource(s.colorId, cd, cs)) continue;
        if (const char* skip = std::getenv("SF_SKIPMAP"))
            if (std::string(skip).find(s.body) != std::string::npos) continue; // (diagnostics)
        if (U.bodies[body].gasGiant) {
            // no ground: a flat map whose only content is the colour
            Map m;
            m.w = 64;
            m.h = 32;
            m.height.assign((size_t)m.w * m.h, 0.f);
            m.generated = true;
            m.colorFile.assign((const unsigned char*)cd, (const unsigned char*)cd + cs);
            g_byBody[body] = (int)g_maps.size();
            g_maps.push_back(std::move(m));
        } else g_color[body].assign((const unsigned char*)cd, (const unsigned char*)cd + cs);
    }
    // the high-resolution tiles (resource ids: colour 211 + 2 i, elevation 212 + 2 i)
    g_hi.clear();
    for (auto& st : g_det) dropWindow(st);
    g_det.clear();
    g_det.resize(U.count());
    for (int i = 0; i < 3; i++) {
        int body = U.find(SOURCES[i].body);
        if (body < 0 || g_byBody[body] < 0) continue;
        HiSrc h;
        h.body = body;
        h.radius = U.bodies[body].radius;
        h.earth = std::string(SOURCES[i].body) == "Earth";
        bool c = openPack(211 + 2 * i, 0, h.color), e = openPack(212 + 2 * i, 1, h.height);
        if (!c) h.color = Pack{};
        if (!e) h.height = Pack{};
        if (!c && !e) continue;
        g_det[body].src = (int)g_hi.size();
        g_hi.push_back(h);
    }
}

bool realColor(int body) { return body >= 0 && body < (int)g_color.size() && !g_color[body].empty(); }

const Map* map(int body) {
    if (body < 0 || body >= (int)g_byBody.size() || g_byBody[body] < 0) return nullptr;
    return &g_maps[g_byBody[body]];
}

const Map* ensure(int body) {
    if (const Map* m = map(body)) return m;
    if (body < 0 || body >= (int)g_gen.size() || !g_gen[body].want || g_maps.size() >= g_maps.capacity()) return nullptr;
    Map m;
    generate(g_gen[body], m);
    if (body < (int)g_color.size()) m.colorFile = g_color[body]; // (a real face over the generated relief)
    g_gen[body].want = false;
    if (body < (int)g_det.size()) g_det[body].rev++; // (landed vessels stand on the generated ground)
    g_byBody[body] = (int)g_maps.size();
    g_maps.push_back(std::move(m));
    return &g_maps.back();
}

static double mapHeight(int body, const Vec3d& d) {
    const Map* m = ensure(body);
    if (!m) return 0;
    double l = len(d);
    if (l < 1e-12) return 0;
    double u = std::atan2(d.y, d.x) / TAU + 0.5;
    double v = std::acos(std::clamp(d.z / l, -1.0, 1.0)) / PI;
    double base = sample(*m, u, v);
    if (body < (int)g_det.size() && g_det[body].active) {
        double hd, f;
        if (detailSample(g_det[body], u, v, hd, f)) return base + (hd - base) * f;
    }
    return base;
}

double microRelief(int body, const Vec3d& d, double minWl) {
    if (body < 0 || body >= (int)g_micro.size() || !g_micro[body].on) return 0;
    const MicroBody& mb = g_micro[body];
    if (minWl >= mb.lambda0) return 0; // (all of it below the resolution asked for)
    if (!ensure(body)) return 0;
    double l = len(d);
    if (l < 1e-12) return 0;
    Vec3d n = d / l;
    // the launch complexes and the runway stand on flat ground (as the maps are flattened there)
    double flat = 1;
    if (mb.sea)
        for (int s = 0; s < LAUNCH_SITE_COUNT; s++) {
            double la = LAUNCH_SITES[s].latDeg * DEG, lo = LAUNCH_SITES[s].lonDeg * DEG;
            Vec3d c{std::cos(la) * std::cos(lo), std::cos(la) * std::sin(lo), std::sin(la)};
            double ang = std::acos(std::clamp(dot(n, c), -1.0, 1.0));
            flat = std::min(flat, std::clamp((ang - FLAT_IN) / (FLAT_OUT - FLAT_IN), 0.0, 1.0));
        }
    // and the surface bases
    if (const FlatList* fl = body < g_flatN ? g_flat[body].load() : nullptr)
        for (const FlatSpot& f : *fl) {
            double c = dot(n, f.n);
            if (c < f.cosOut) continue;
            double ang = std::acos(std::clamp(c, -1.0, 1.0));
            flat = std::min(flat, std::clamp((ang - f.in) / (f.out - f.in), 0.0, 1.0));
        }
    if (flat <= 0) return 0;
    // roughness from the map: steeper (mountains, crater walls) means rougher at the smaller scales too
    double L0 = mb.lambda0, h0 = mapHeight(body, n);
    if (mb.sea && h0 < 0.5) return 0; // the sea
    Vec3d e = norm(cross(Vec3d{0, 0, 1}, n) + Vec3d{1e-9, 0, 0}), nn = cross(n, e);
    double dAng = L0 / mb.R;
    double gE = mapHeight(body, norm(n + e * dAng)) - h0, gN = mapHeight(body, norm(n + nn * dAng)) - h0;
    double slope = std::sqrt(gE * gE + gN * gN) / L0;
    double amp0 = L0 * (mb.airless ? 0.008 : 0.005) + L0 * 0.08 * std::min(slope, 0.6);
    if (mb.sea) amp0 *= std::clamp(h0 / 300.0, 0.25, 1.0); // coastal lowlands are gentle
    Vec3d p = n * mb.R; // body-fixed metres
    double h = 0, wl = L0, amp = amp0;
    minWl = std::max(minWl, 1.0);
    uint32_t sd = mb.seed;
    for (int o = 0; o < 14 && wl >= minWl; o++) {
        double fade = std::clamp(wl / minWl - 1.0, 0.0, 1.0); // (the last octave fades in: no popping)
        double v = gnoise(p.x / wl, p.y / wl, p.z / wl, sd + o * 977u);
        if (o < 3 && !mb.airless) v = 0.6 - std::fabs(v) * 1.6; // ridges on the larger scales (eroded land)
        h += v * amp * fade;
        wl *= 0.5;
        amp *= 0.5548; // 0.5^0.85, fractal: rougher at the small scales than smooth hills
    }
    if (mb.airless) {
        // craters of all sizes below the maps (the airless bodies are saturated with them)
        double cell = L0 * 0.6;
        double soft = mb.sea ? 0 : 1;
        for (int k = 0; k < 6 && cell * 0.25 >= minWl; k++) {
            h += craterField(p, cell, sd ^ (0x9e37u + k * 131u), 0.6) * soft;
            cell *= 0.33;
        }
    }
    return h * flat;
}

double height(int body, const Vec3d& d) { return mapHeight(body, d) + microRelief(body, d, 1.0); }
double heightLod(int body, const Vec3d& d, double minWl) { return mapHeight(body, d) + microRelief(body, d, minWl); }

double maxHeight(int body) {
    const Map* m = map(body);
    if (!m) return 0.0;
    double fine = body < (int)g_micro.size() && g_micro[body].on ? 0.12 * g_micro[body].lambda0 : 0.0; // (the fine relief on top)
    return std::max(0.0f, m->hMax) + fine;
}

double minHeight(int body) {
    const Map* m = map(body);
    return m ? std::min(0.0f, m->hMin) : 0.0;
}

double g_detRange = 1.0;
void setDetailRange(double k) { g_detRange = std::clamp(k, 0.5, 4.0); }
void focus(int body, const Vec3d& d, double alt) {
    if (g_det.empty()) return;
    pollResults();
    for (int b = 0; b < (int)g_det.size(); b++)
        if (b != body && g_det[b].active) dropWindow(g_det[b]); // one body at a time
    if (body < 0 || body >= (int)g_det.size() || g_det[body].src < 0) return;
    DetState& st = g_det[body];
    const HiSrc& s = g_hi[st.src];
    st.d.ctw = s.color.data ? s.color.tw : 0;
    st.d.cth = s.color.data ? s.color.th : 0;
    st.d.htw = s.height.data ? s.height.tw : 0;
    st.d.hth = s.height.data ? s.height.th : 0;
    // near the body only (with some hysteresis): from afar the global map is all one sees
    if (alt > (st.active ? 0.8 : 0.6) * s.radius * g_detRange) {
        if (st.active) dropWindow(st);
        return;
    }
    double l = len(d);
    if (l < 1e-9) return;
    double u = std::atan2(d.y, d.x) / TAU + 0.5, v = std::acos(std::clamp(d.z / l, -1.0, 1.0)) / PI;
    double fx = u * DET_GW, fy = v * DET_GH;
    if (!st.active) placeWindow(st, fx, fy);
    else {
        double lx = fx - st.d.tx0;
        lx -= std::floor(lx / DET_GW) * DET_GW;
        double ly = fy - st.d.ty0;
        bool nearX = lx < 1.0 || lx > DET_WIN - 1.0;
        bool nearY = (ly < 1.0 && st.d.ty0 > 0) || (ly > DET_WIN - 1.0 && st.d.ty0 + DET_WIN < DET_GH);
        if (nearX || nearY) placeWindow(st, fx, fy);
    }
    // the elevation right around the vessel now (main thread): it must not change under it later
    if (s.height.data) {
        int cx = (int)std::floor(fx), cy = (int)std::floor(fy);
        for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) {
                int tx = ((cx + dx) % DET_GW + DET_GW) % DET_GW, ty = cy + dy;
                if (ty < 0 || ty >= DET_GH) continue;
                int key = ty * DET_GW + tx;
                if (!inWindow(st.d, key) || st.slots[slotOf(key)].key == key) continue;
                std::vector<float> h;
                if (decodeHeightTile(s, key, h)) putHeight(st, key, std::move(h));
            }
    }
}

const Detail* detail(int body) {
    if (body < 0 || body >= (int)g_det.size() || !g_det[body].active) return nullptr;
    return &g_det[body].d;
}
unsigned revision(int body) {
    unsigned r = body >= 0 && body < (int)g_det.size() ? g_det[body].rev : 0;
    return r + (body >= 0 && body < (int)g_flatRev.size() ? g_flatRev[body] * 7919u : 0u);
}

bool takeColorTile(int body, int& tx, int& ty, std::vector<unsigned char>& rgb) {
    if (body < 0 || body >= (int)g_det.size()) return false;
    DetState& st = g_det[body];
    if (st.colorOut.empty()) return false;
    tx = st.colorOut.front().first % DET_GW;
    ty = st.colorOut.front().first / DET_GW;
    rgb = std::move(st.colorOut.front().second);
    st.colorOut.pop_front();
    return true;
}
bool takeHeightTile(int body, int& tx, int& ty, const float*& h) {
    if (body < 0 || body >= (int)g_det.size()) return false;
    DetState& st = g_det[body];
    while (!st.heightOut.empty()) {
        int key = st.heightOut.front();
        st.heightOut.pop_front();
        const Slot& sl = st.slots[slotOf(key)];
        if (sl.key != key) continue;
        tx = key % DET_GW;
        ty = key / DET_GW;
        h = sl.h.data();
        return true;
    }
    return false;
}
void resendTiles(int body) {
    if (body < 0 || body >= (int)g_det.size()) return;
    DetState& st = g_det[body];
    if (!st.active) return;
    st.heightOut.clear();
    for (const Slot& sl : st.slots)
        if (sl.key >= 0) st.heightOut.push_back(sl.key);
    // colour: decoded again (it is not kept on the CPU)
    st.deliveredC.clear();
    st.colorOut.clear();
    const HiSrc& s = g_hi[st.src];
    if (!s.color.data) return;
    for (int j = 0; j < DET_WIN; j++)
        for (int i = 0; i < DET_WIN; i++) {
            int key = (st.d.ty0 + j) * DET_GW + (st.d.tx0 + i) % DET_GW;
            if (st.queuedC.insert(key).second) queueJob(body, key, true, st.gen);
        }
}

void paintLaunchSites(unsigned char* rgb, int w, int h, int ox, int oy, int fullW, int fullH) {
    // ground for the pad and the runway only: solid within 2.4 km, blending out by 4.4 km. The pictures show
    // the real coast (Tanegashima is 57 km long and 6-12 km wide); a disc of 27-51 km (made for the coarse
    // global map) had turned it into a large round island
    const double in = 0.022 * DEG, out = 0.040 * DEG;
    const unsigned char land[3] = {92, 104, 64};
    nearLaunchSites(w, h, ox, oy, fullW, fullH, out, [&](int x, int y, double ang) {
        double k = std::clamp((out - ang) / (out - in), 0.0, 1.0);
        unsigned char* p = rgb + ((size_t)y * w + x) * 3;
        for (int c = 0; c < 3; c++) p[c] = (unsigned char)(p[c] + (land[c] - p[c]) * k);
    });
}

size_t detailBytes() {
    size_t n = 0;
    for (const DetState& st : g_det)
        for (const Slot& sl : st.slots) n += sl.h.size() * sizeof(float);
    return n;
}

size_t bytes() {
    size_t s = 0;
    for (const Map& m : g_maps) s += m.height.size() * sizeof(float) + m.colorFile.size();
    return s;
}

} // namespace terrain

namespace terrain {
namespace {
class MapGround : public GroundPort {
public:
    bool hasRelief(int body) override { return ensure(body) != nullptr; }
    bool reliefReady(int body) override { return map(body) != nullptr; }
    double height(int body, const Vec3d& d) override { return terrain::height(body, d); }
    double maxHeight(int body) override { return terrain::maxHeight(body); }
    unsigned revision(int body) override { return terrain::revision(body); }
    void focus(int body, const Vec3d& d, double alt) override { terrain::focus(body, d, alt); }
    void setFlatSpots(int body, const std::vector<Vec3d>& c, double r) override { terrain::setFlatSpots(body, c, r); }
};
MapGround g_ground;
} // namespace
GroundPort& groundPort() { return g_ground; }
} // namespace terrain
