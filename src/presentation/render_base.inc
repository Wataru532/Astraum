// ---------------------------------------------------------------- surface bases (included by render.cpp)
// The buildings of a base, modelled loosely on real concepts: 3D-printed regolith shells (ICON / AI SpaceFactory
// studies), inflatable habitats (TransHab, B330), the EDEN ISS greenhouse, Mars ISRU plants, NASA's Fission
// Surface Power and Kilopower, VSAT vertical solar towers, sintered landing pads. Every kind has three designs,
// chosen per base and place, so no two bases look alike. Generated textures give the printed layers, the
// fabric, the crops under their grow lights and the pavers of the pad.
namespace {

Texture2D g_baseTex[4]{}; // printed regolith, inflatable fabric, crops, hex pavers
bool g_baseTexReady = false;

uint32_t thash(int x, int y, uint32_t s) {
    uint32_t h = (uint32_t)x * 374761393u + (uint32_t)y * 668265263u + s * 2246822519u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}
float tval(int x, int y, int px, int py, uint32_t s) {
    x = ((x % px) + px) % px;
    y = ((y % py) + py) % py;
    return (thash(x, y, s) & 0xFFFFFF) / 16777215.0f;
}
// tileable value noise: px x py lattice cells across the texture (u, v in 0..1)
float tnoise(float u, float v, int px, int py, uint32_t s) {
    float x = u * px, y = v * py;
    int ix = (int)std::floor(x), iy = (int)std::floor(y);
    float fx = x - ix, fy = y - iy;
    fx = fx * fx * (3 - 2 * fx);
    fy = fy * fy * (3 - 2 * fy);
    float a = tval(ix, iy, px, py, s), b = tval(ix + 1, iy, px, py, s), c = tval(ix, iy + 1, px, py, s), d = tval(ix + 1, iy + 1, px, py, s);
    return a + (b - a) * fx + (c - a) * fy + (a - b - c + d) * fx * fy;
}
float tfbm(float u, float v, int p, uint32_t s, int oct) {
    float sum = 0, amp = 0.5f, n = 0;
    for (int k = 0; k < oct; k++) {
        sum += amp * tnoise(u, v, p << k, p << k, s + k * 101u);
        n += amp;
        amp *= 0.5f;
    }
    return sum / n;
}
unsigned char u8(float v) { return (unsigned char)std::clamp(v * 255.0f, 0.0f, 255.0f); }

template <class F> Texture2D makeTex(int W, F f, bool aniso) {
    Image img = GenImageColor(W, W, WHITE);
    Color* px = (Color*)img.data;
    for (int y = 0; y < W; y++)
        for (int x = 0; x < W; x++) px[y * W + x] = f(x, y, W);
    Texture2D t = LoadTextureFromImage(img);
    UnloadImage(img);
    GenTextureMipmaps(&t);
    SetTextureFilter(t, aniso ? TEXTURE_FILTER_ANISOTROPIC_8X : TEXTURE_FILTER_TRILINEAR);
    SetTextureWrap(t, TEXTURE_WRAP_REPEAT);
    return t;
}

void makeBaseTextures() {
    if (g_baseTexReady) return;
    // 3D-printed regolith: 16 extruded layers per repeat, grooves between them, grain and pebbles
    g_baseTex[0] = makeTex(512, [](int x, int y, int W) {
        float u = (x + 0.5f) / W, v = (y + 0.5f) / W;
        float layer = v * 16, fl = layer - std::floor(layer);
        float groove = std::clamp(fl / 0.16f, 0.0f, 1.0f) * std::clamp((1 - fl) / 0.14f, 0.0f, 1.0f);
        float bead = 0.88f + 0.12f * std::sin(fl * (float)PI);
        float grain = tfbm(u, v, 16, 11u, 4);
        float pebble = tnoise(u, v, 160, 160, 23u) > 0.86f ? 0.84f : 1.0f;
        float lt = 0.93f + 0.09f * tval(0, (int)layer, 1, 16, 31u);
        float speed = 0.95f + 0.07f * tnoise(u, v, 24, 16, 41u);
        float k = (0.58f + 0.42f * groove) * bead * (0.84f + 0.3f * grain) * pebble * lt * speed;
        return Color{u8(k * 1.02f), u8(k), u8(k * 0.97f), 255};
    }, false);
    // inflatable shell: woven outer layer, restraint straps every metre (stitched), a little dust
    g_baseTex[1] = makeTex(512, [](int x, int y, int W) {
        float u = (x + 0.5f) / W, v = (y + 0.5f) / W;
        float weave = ((x / 2 + y / 2) & 1) ? 0.985f : 0.955f;
        int sx = x % 128, sy = y % 128;
        bool sv = sx < 10, shz = sy < 8;
        float r = 0.93f, g = 0.92f, b = 0.89f;
        if (sv || shz) { r = 0.86f; g = 0.82f; b = 0.72f; }
        bool stitch = (sv && (sx == 1 || sx == 8) && (y % 6) < 3) || (shz && (sy == 1 || sy == 6) && (x % 6) < 3);
        float dust = 0.93f + 0.08f * tfbm(u, v, 4, 51u, 3);
        float k = weave * dust * (stitch ? 0.72f : 1.0f);
        return Color{u8(r * k), u8(g * k), u8(b * k), 255};
    }, false);
    // crops under grow lights: four beds per repeat, leaves of two greens, soil gaps, pink LED strips between
    g_baseTex[2] = makeTex(512, [](int x, int y, int W) {
        float u = (x + 0.5f) / W, v = (y + 0.5f) / W;
        float row = v * 4, fr = row - std::floor(row);
        if (fr > 0.86f || fr < 0.06f) {
            float core = 1.0f - std::fabs((fr > 0.5f ? fr - 0.93f : fr + 0.07f)) / 0.07f;
            float c = std::clamp(core, 0.0f, 1.0f);
            return Color{u8(0.75f + 0.25f * c), u8(0.25f + 0.35f * c), u8(0.62f + 0.3f * c), 255};
        }
        if (fr > 0.78f) return Color{200, 202, 206, 255}; // (tray edge)
        float n = tfbm(u, v, 24, 61u + (unsigned)row, 4), m = tnoise(u, v, 96, 96, 71u);
        if (n < 0.36f) return Color{58, 44, 34, 255}; // (substrate between the plants)
        float t = std::clamp((n - 0.36f) / 0.4f, 0.0f, 1.0f);
        float r = 0.10f + 0.30f * t + 0.08f * m, g = 0.34f + 0.42f * t + 0.1f * m, b = 0.10f + 0.14f * t;
        return Color{u8(r), u8(g), u8(b), 255};
    }, false);
    // sintered hex pavers: 8 x 10 cells per repeat (slightly squashed to tile), each its own shade, dark joints
    g_baseTex[3] = makeTex(512, [](int x, int y, int W) {
        const int cols = 8, rows = 10;
        float cw = (float)W / cols, rh = (float)W / rows;
        float best = 1e9f, second = 1e9f;
        int bc = 0, br = 0;
        int r0 = (int)(y / rh);
        for (int r = r0 - 1; r <= r0 + 1; r++) {
            float cy = (r + 0.5f) * rh;
            float off = ((r % 2) + 2) % 2 ? 0.5f : 0.0f;
            int c0 = (int)(x / cw - off);
            for (int c = c0 - 1; c <= c0 + 1; c++) {
                float cx = (c + 0.5f + off) * cw;
                float dx = x - cx, dy = (y - cy) * 1.12f;
                float d = std::sqrt(dx * dx + dy * dy);
                if (d < best) { second = best; best = d; bc = c; br = r; }
                else if (d < second) second = d;
            }
        }
        float u = (x + 0.5f) / W, v = (y + 0.5f) / W;
        float joint = std::clamp((second - best) / 3.0f, 0.0f, 1.0f);
        float tint = 0.82f + 0.2f * tval(bc, br, cols, rows, 91u);
        float grain = 0.9f + 0.16f * tfbm(u, v, 32, 93u, 3);
        float k = (0.30f + 0.70f * joint) * tint * grain;
        return Color{u8(k * 0.98f), u8(k * 0.96f), u8(k * 0.92f), 255};
    }, true);
    g_baseTexReady = true;
}

struct TexScope {
    Texture2D prev;
    TexScope(int which, float metresPerRepeat) : prev(shipMat.maps[MATERIAL_MAP_DIFFUSE].texture) {
        shipMat.maps[MATERIAL_MAP_DIFFUSE].texture = g_baseTex[which];
        setF(shipShader, shipTexM, metresPerRepeat);
    }
    ~TexScope() { shipMat.maps[MATERIAL_MAP_DIFFUSE].texture = prev; }
};

void ell(const DrawCtx& c, const Mat3d& R, const Vec3d& p, double rx, double ry, double rz, Color col, float em = 0) {
    g_uv = {(float)(TAU * std::max(rx, ry)), (float)(PI * rz)};
    g_scale = {(float)rx, (float)ry, (float)rz};
    piece(c, sphereMesh, toMatrix(R, p, rx, ry, rz), col, em);
}
// a cylinder from a to b (radius r), with its own glow
void tube(const DrawCtx& c, const Vec3d& a, const Vec3d& b, double r, Color col, float em = 0) {
    Vec3d d = b - a;
    double l = len(d);
    if (l < 1e-6) return;
    Vec3d z = d / l, x = anyPerp(z), y = cross(z, x);
    g_uv = {(float)(TAU * r), (float)l};
    g_scale = {(float)r, (float)r, (float)l};
    piece(c, frustum(1, 1), toMatrix(Mat3d::fromCols(x, y, z), a, r, r, l), col, em);
}
// a band around the axis dir through p (radius r, width w)
void band(const DrawCtx& c, const Vec3d& p, const Vec3d& dir, double r, double w, Color col, float em = 0) {
    tube(c, p - dir * (w / 2), p + dir * (w / 2), r, col, em);
}
Color mixc(Color a, Color b, float t) {
    return {(unsigned char)(a.r + (b.r - a.r) * t), (unsigned char)(a.g + (b.g - a.g) * t), (unsigned char)(a.b + (b.b - a.b) * t), 255};
}

struct BaseLook {
    Color soil, soilLight;
    float night = 0;         // 0 day .. 1 night: windows and grow lights glow
    float winEm() const { return 0.12f + 2.4f * night; }
};
const Color B_WIN{255, 212, 158, 255}, B_PANEL{232, 230, 222, 255}, B_METAL{150, 154, 160, 255}, B_DARK{52, 54, 58, 255},
    B_YELLOW{232, 182, 40, 255}, B_RADIATOR{222, 224, 228, 255};

// window slit on a curved wall at angle a, radius r, height z (tangential width w, height h)
void wallWindow(const DrawCtx& c, double a, double r, double z, double w, double h, const BaseLook& L) {
    FinScope f(FIN_PAINT);
    box(c, rotZ(a), {std::cos(a) * r, std::sin(a) * r, z}, 0.35, w, h, B_WIN, L.winEm());
}

// ---- habitats
void habDome(const DrawCtx& c, const BaseLook& L) {
    {
        FinScope f(FIN_CONCRETE);
        cyl(c, -1.0, 1.35, 10.3, 10.1, shade(L.soil, 0.8));
    }
    {
        FinScope f(FIN_REGOLITH);
        TexScope t(0, 2.0f);
        ell(c, Mat3d{}, {0, 0, 0.2}, 9.0, 9.0, 7.0, L.soilLight);
        // buttress ribs printed with the shell
        for (int k = 0; k < 8; k++) {
            double a = k * PI / 4 + 0.39;
            Vec3d d{std::cos(a), std::sin(a), 0};
            tube(c, d * 9.6 + Vec3d{0, 0, 0.2}, d * 6.2 + Vec3d{0, 0, 5.2}, 0.55, shade(L.soilLight, 0.92));
        }
    }
    for (int k = 0; k < 6; k++) wallWindow(c, k * PI / 3 + 0.2, 8.55, 2.6, 1.5, 0.8, L);
    {
        // entrance tunnel with its airlock door, and the skylight
        FinScope f(FIN_PAINT);
        tube(c, {7.0, 0, 1.7}, {13.0, 0, 1.7}, 1.75, B_PANEL);
        band(c, {12.6, 0, 1.7}, {1, 0, 0}, 1.85, 0.4, B_METAL);
        box(c, Mat3d{}, {13.05, 0, 1.55}, 0.25, 1.6, 2.4, B_DARK);
        box(c, Mat3d{}, {13.15, 0.95, 2.9}, 0.12, 0.3, 0.2, Color{120, 255, 150, 255}, 0.4f + 2 * L.night);
        cyl(c, 6.4, 1.0, 1.7, 1.4, B_PANEL);
        cyl(c, 7.35, 0.25, 1.35, 1.1, B_WIN, L.winEm());
    }
    {
        FinScope f(FIN_METAL);
        tube(c, {-2.5, 2.0, 5.8}, {-2.5, 2.0, 9.6}, 0.08, B_METAL);
        ell(c, rotY(0.6), {-2.5, 2.0, 9.7}, 0.7, 0.7, 0.18, Color{226, 228, 232, 255});
    }
}

void habInflatable(const DrawCtx& c, const BaseLook& L) {
    const double zc = 4.4, rx = 6.6, rr = 3.6;
    {
        FinScope f(FIN_METAL);
        // four legs with pads, and the access stair
        for (int k = 0; k < 4; k++) {
            double x = (k & 1 ? 1 : -1) * 3.6, y = (k & 2 ? 1 : -1) * 2.3;
            tube(c, {x, y, 0}, {x * 0.85, y * 0.75, zc - 2.4}, 0.22, B_METAL);
            cyl(c, -0.1, 0.25, 0.7, 0.7, B_DARK, x, y);
        }
        box(c, rotY(0.55), {9.3, 0, 1.55}, 4.6, 1.5, 0.18, B_METAL);
        tube(c, {rx - 0.4, 0, zc}, {rx + 1.1, 0, zc}, 1.5, B_METAL);
        tube(c, {-rx + 0.4, 0, zc}, {-rx - 0.9, 0, zc}, 1.3, B_METAL);
        box(c, Mat3d{}, {rx + 1.15, 0, zc - 0.2}, 0.2, 1.1, 1.9, B_DARK);
    }
    {
        FinScope f(FIN_FABRIC);
        TexScope t(1, 3.0f);
        ell(c, Mat3d{}, {0, 0, zc}, rx, rr, rr, B_PANEL);
        for (double x = -4.5; x <= 4.6; x += 1.5) {
            double r = rr * std::sqrt(std::max(0.0, 1 - (x / rx) * (x / rx))) + 0.04;
            band(c, {x, 0, zc}, {1, 0, 0}, r, 0.22, Color{214, 204, 178, 255});
        }
    }
    {
        // regolith bags piled on top against radiation (a shield berm over the crew quarters)
        FinScope f(FIN_REGOLITH);
        TexScope t(0, 1.4f);
        ell(c, Mat3d{}, {0, 0, zc + 2.4}, 4.4, 2.6, 1.5, L.soil);
    }
    for (int k = 0; k < 4; k++) {
        FinScope f(FIN_PAINT);
        double x = -3.0 + k * 2.0;
        ell(c, Mat3d{}, {x, -rr * std::sqrt(1 - (x / rx) * (x / rx)) + 0.05, zc + 0.3}, 0.42, 0.12, 0.42, B_WIN, L.winEm());
    }
}

void habTower(const DrawCtx& c, const BaseLook& L) {
    const double H = 13.5, rb = 6.0, rt = 3.5;
    {
        FinScope f(FIN_REGOLITH);
        TexScope t(0, 2.0f);
        cyl(c, -0.6, H, rb, rt, L.soilLight);
        ell(c, Mat3d{}, {0, 0, H - 0.6}, rt, rt, 1.5, L.soilLight);
    }
    {
        // vertical window slits following the taper, a glazed ring under the cap
        for (int k = 0; k < 5; k++) {
            double a = k * TAU / 5 + 0.3, z = 6.2, r = rb - (rb - rt) * (z + 0.6) / H - 0.05;
            FinScope f(FIN_PAINT);
            box(c, rotZ(a) * rotY(-std::atan((rb - rt) / H)), {std::cos(a) * r, std::sin(a) * r, z}, 0.35, 0.75, 6.0, B_WIN, L.winEm());
        }
        FinScope f(FIN_PAINT);
        cyl(c, H - 1.4, 0.5, rt + 0.12, rt + 0.1, B_WIN, L.winEm());
        box(c, Mat3d{}, {rb + 1.8, 0, 1.4}, 4.0, 3.2, 2.8, B_PANEL);
        box(c, Mat3d{}, {rb + 3.85, 0, 1.25}, 0.2, 1.4, 2.3, B_DARK);
        box(c, Mat3d{}, {rb + 1.8, 0, 2.9}, 4.2, 3.4, 0.2, B_YELLOW);
    }
}

// ---- greenhouses
void greenTunnel(const DrawCtx& c, const BaseLook& L) {
    const double Lh = 9.0, R = 3.4;
    {
        FinScope f(FIN_CONCRETE);
        box(c, Mat3d{}, {0, 0, 0.1}, 2 * Lh + 0.6, 2 * R + 0.6, 0.45, shade(L.soil, 0.85));
    }
    {
        FinScope f(FIN_GROW);
        TexScope t(2, 1.2f);
        for (int r = -1; r <= 1; r++) box(c, Mat3d{}, {0, r * 2.0, 1.0}, 2 * Lh - 1.4, 1.3, 1.3, Color{255, 255, 255, 255}, 0.25f + 1.3f * L.night);
    }
    {
        FinScope f(FIN_PAINT);
        for (int s = -1; s <= 1; s += 2) box(c, Mat3d{}, {s * Lh, 0, 1.7}, 0.3, 2 * R, 3.4, B_PANEL);
        box(c, Mat3d{}, {Lh + 0.18, 0, 1.3}, 0.12, 1.3, 2.2, B_DARK);
    }
    {
        FinScope f(FIN_METAL);
        for (double x = -Lh; x <= Lh + 0.01; x += 3.0) band(c, {x, 0, 0.3}, {1, 0, 0}, R + 0.08, 0.18, B_METAL);
    }
    {
        FinScope f(FIN_GLASS);
        tube(c, {-Lh, 0, 0.3}, {Lh, 0, 0.3}, R, Color{196, 222, 232, 120});
    }
}

void greenDome(const DrawCtx& c, const BaseLook& L) {
    const double R = 8.0, Hd = 6.4;
    {
        FinScope f(FIN_GROW);
        TexScope t(2, 1.2f);
        float em = 0.25f + 1.3f * L.night;
        cyl(c, 0, 0.9, 6.8, 6.8, Color{255, 255, 255, 255}, 0, 0, em);
        cyl(c, 0.9, 0.8, 4.7, 4.7, Color{255, 255, 255, 255}, 0, 0, em);
        cyl(c, 1.7, 0.8, 2.6, 2.6, Color{255, 255, 255, 255}, 0, 0, em);
    }
    {
        FinScope f(FIN_METAL);
        // the frame: meridians and two rings
        for (int k = 0; k < 10; k++) {
            double a = k * TAU / 10;
            Vec3d prev;
            for (int j = 0; j <= 6; j++) {
                double phi = j * (PI / 2) / 6;
                Vec3d p{std::cos(phi) * std::cos(a) * (R + 0.05), std::cos(phi) * std::sin(a) * (R + 0.05), std::sin(phi) * (Hd + 0.05)};
                if (j) tube(c, prev, p, 0.1, B_METAL);
                prev = p;
            }
        }
        for (double z : {2.2, 4.4}) band(c, {0, 0, z}, {0, 0, 1}, R * std::sqrt(1 - (z / Hd) * (z / Hd)) + 0.06, 0.14, B_METAL);
        box(c, Mat3d{}, {R + 0.8, 0, 1.3}, 2.4, 2.4, 2.6, B_PANEL);
        box(c, Mat3d{}, {R + 2.02, 0, 1.2}, 0.12, 1.2, 2.1, B_DARK);
    }
    {
        FinScope f(FIN_GLASS);
        ell(c, Mat3d{}, {0, 0, 0}, R, R, Hd, Color{196, 222, 232, 110});
    }
}

void greenTower(const DrawCtx& c, const BaseLook& L) {
    const double R = 3.2, H = 12.0;
    {
        FinScope f(FIN_GROW);
        TexScope t(2, 1.0f);
        for (double z = 0.4; z < H - 1; z += 1.9) cyl(c, z, 0.7, R - 0.5, R - 0.5, Color{255, 255, 255, 255}, 0, 0, 0.25f + 1.3f * L.night);
    }
    {
        FinScope f(FIN_METAL);
        cyl(c, -0.3, 0.6, R + 0.6, R + 0.5, B_METAL);
        cyl(c, H, 0.9, R + 0.2, R - 0.6, Color{226, 226, 230, 255});
        for (int k = 0; k < 6; k++) {
            double a = k * TAU / 6;
            tube(c, {std::cos(a) * (R + 0.05), std::sin(a) * (R + 0.05), 0.2}, {std::cos(a) * (R + 0.05), std::sin(a) * (R + 0.05), H}, 0.09, B_METAL);
        }
    }
    {
        FinScope f(FIN_GLASS);
        cyl(c, 0.2, H - 0.2, R, R, Color{196, 222, 232, 110});
    }
}

// ---- ISRU propellant plants
void tankOnStand(const DrawCtx& c, const Vec3d& p, double r, Color col) {
    FinScope f(FIN_METAL);
    for (int k = 0; k < 4; k++) {
        double a = k * PI / 2 + PI / 4;
        tube(c, p + Vec3d{std::cos(a) * r * 0.8, std::sin(a) * r * 0.8, -p.z}, p + Vec3d{std::cos(a) * r * 0.55, std::sin(a) * r * 0.55, -r * 0.4}, 0.12, B_METAL);
    }
    ell(c, Mat3d{}, p, r, r, r, col);
}
void isruPlant(const DrawCtx& c, const BaseLook& L, int variant) {
    {
        FinScope f(FIN_CONCRETE);
        box(c, Mat3d{}, {0, 0, 0.15}, 17, 12, 0.45, shade(L.soil, 0.85));
    }
    if (variant == 0) {
        // ice mining and a Sabatier reactor: process hall, radiators, reactor column, oxygen and methane spheres,
        // the hopper and conveyor that feed it
        {
            FinScope f(FIN_PAINT);
            box(c, Mat3d{}, {-2, 0, 2.2}, 7, 5, 3.6, B_PANEL);
            box(c, Mat3d{}, {-2, 0, 4.05}, 7.1, 5.1, 0.25, B_YELLOW);
            box(c, Mat3d{}, {1.55, -1.2, 1.2}, 0.12, 1.2, 2.0, B_DARK);
        }
        {
            FinScope f(FIN_RADIATOR);
            for (int s = -1; s <= 1; s += 2) box(c, Mat3d{}, {-2, s * 3.9, 3.0}, 6.5, 0.12, 4.0, B_RADIATOR);
        }
        {
            FinScope f(FIN_METAL);
            cyl(c, 0.4, 7.5, 0.75, 0.7, B_METAL, 3.4, -2.4);
            cyl(c, 7.9, 0.8, 0.7, 0.25, B_METAL, 3.4, -2.4);
            tube(c, {3.4, -2.4, 2.0}, {1.5, -1.5, 2.0}, 0.18, B_METAL);
        }
        tankOnStand(c, {4.6, 2.6, 2.4}, 1.9, Color{236, 238, 242, 255});
        tankOnStand(c, {1.0, 4.0, 2.0}, 1.5, Color{184, 198, 216, 255});
        {
            FinScope f(FIN_METAL);
            tube(c, {4.6, 2.6, 2.4}, {1.5, 1.5, 2.6}, 0.15, B_METAL);
            tube(c, {1.0, 4.0, 2.0}, {-0.5, 2.5, 2.6}, 0.15, B_METAL);
            cyl(c, 2.8, 2.2, 0.6, 2.2, Color{150, 132, 108, 255}, -7.0, 3.6);
            for (int k = 0; k < 3; k++) tube(c, {-7.0 + (k - 1) * 1.2, 3.6, 0}, {-7.0 + (k - 1) * 0.6, 3.6, 2.9}, 0.12, B_METAL);
            box(c, rotY(-0.32), {-5.0, 1.8, 3.4}, 5.0, 0.9, 0.25, B_DARK);
        }
    } else if (variant == 1) {
        // electrolysis halls in a row, horizontal propellant tanks on saddles, an excavator parked beside them
        {
            FinScope f(FIN_PAINT);
            box(c, Mat3d{}, {-4.5, -2.0, 1.8}, 5, 4, 3.0, B_PANEL);
            box(c, Mat3d{}, {0.5, -2.0, 1.5}, 4, 4, 2.4, Color{214, 218, 222, 255});
            box(c, Mat3d{}, {4.6, -2.0, 2.2}, 3.6, 4, 3.8, B_PANEL);
            box(c, Mat3d{}, {0.5, -2.0, 2.75}, 4.1, 4.1, 0.15, B_YELLOW);
        }
        {
            FinScope f(FIN_RADIATOR);
            box(c, rotX(0.5), {-4.5, -4.8, 3.2}, 5.0, 0.12, 3.4, B_RADIATOR);
        }
        {
            FinScope f(FIN_FOAM);
            for (int k = 0; k < 3; k++) {
                double y = 2.0 + k * 0.0, x = -5.0 + k * 5.0;
                tube(c, {x - 2.2, y + 1.0, 1.6}, {x + 2.2, y + 1.0, 1.6}, 1.15, k == 1 ? Color{196, 210, 226, 255} : Color{236, 232, 224, 255});
            }
        }
        {
            FinScope f(FIN_METAL);
            for (int k = 0; k < 3; k++)
                for (int s = -1; s <= 1; s += 2) box(c, Mat3d{}, {-5.0 + k * 5.0 + s * 1.4, 3.0, 0.4}, 0.4, 2.4, 0.8, B_METAL);
            // excavator: body, wheels, bucket arm
            box(c, Mat3d{}, {9.5, 3.5, 1.3}, 3.2, 2.2, 1.3, B_YELLOW);
            for (int k = 0; k < 4; k++) {
                double x = 9.5 + (k & 1 ? 1.1 : -1.1), y = 3.5 + (k & 2 ? 1.25 : -1.25);
                tube(c, {x, y - 0.2, 0.55}, {x, y + 0.2, 0.55}, 0.55, B_DARK);
            }
            tube(c, {10.8, 3.5, 1.8}, {12.4, 3.5, 2.6}, 0.15, B_METAL);
            tube(c, {12.4, 3.5, 2.6}, {13.2, 3.5, 0.9}, 0.12, B_METAL);
            box(c, rotY(0.4), {13.3, 3.5, 0.7}, 0.9, 1.4, 0.6, B_METAL);
        }
    } else {
        // an array of oxygen extractors (MOXIE grown up) around a compressor, one tall product tank
        {
            FinScope f(FIN_PAINT);
            for (int k = 0; k < 6; k++) {
                double x = -6 + (k % 3) * 4.0, y = k < 3 ? -3.0 : 3.0;
                box(c, Mat3d{}, {x, y, 1.1}, 2.6, 2.0, 1.8, k % 2 ? B_PANEL : Color{214, 206, 188, 255});
                box(c, Mat3d{}, {x, y, 2.05}, 2.7, 2.1, 0.1, Color{180, 130, 60, 255});
            }
            box(c, Mat3d{}, {4.0, 0, 1.6}, 3.0, 3.0, 2.8, B_PANEL);
        }
        {
            FinScope f(FIN_RADIATOR);
            for (int k = 0; k < 3; k++) box(c, Mat3d{}, {-6 + k * 4.0, 0, 2.2}, 0.1, 3.4, 3.2, B_RADIATOR);
        }
        {
            FinScope f(FIN_STEEL);
            cyl(c, 0.2, 7.5, 1.6, 1.6, Color{220, 222, 226, 255}, 7.0, 0);
            ell(c, Mat3d{}, {7.0, 0, 7.7}, 1.6, 1.6, 0.8, Color{220, 222, 226, 255});
        }
        {
            FinScope f(FIN_METAL);
            for (int k = 0; k < 6; k++) {
                double x = -6 + (k % 3) * 4.0, y = k < 3 ? -3.0 : 3.0;
                tube(c, {x, y * 0.6, 2.2}, {3.0, 0, 2.4}, 0.1, B_METAL);
            }
            tube(c, {5.5, 0, 2.0}, {6.2, 0, 2.0}, 0.15, B_METAL);
        }
    }
}

// ---- propellant storage
void storageTank(const DrawCtx& c, const BaseLook& L, int variant, int idx) {
    Color lox{238, 240, 244, 255}, ch4{190, 204, 222, 255};
    Color col = idx % 2 ? ch4 : lox;
    {
        FinScope f(FIN_CONCRETE);
        cyl(c, -0.3, 0.6, 4.6, 4.6, shade(L.soil, 0.85));
    }
    if (variant == 0) {
        tankOnStand(c, {0, 0, 4.4}, 3.4, col);
    } else if (variant == 1) {
        FinScope f(FIN_FOAM);
        tube(c, {-4.2, 0, 2.2}, {4.2, 0, 2.2}, 1.8, col);
        ell(c, Mat3d{}, {4.2, 0, 2.2}, 0.8, 1.8, 1.8, col);
        ell(c, Mat3d{}, {-4.2, 0, 2.2}, 0.8, 1.8, 1.8, col);
        FinScope g(FIN_METAL);
        for (int s = -1; s <= 1; s += 2) box(c, Mat3d{}, {s * 2.6, 0, 0.6}, 0.6, 3.0, 1.2, B_METAL);
    } else {
        FinScope f(FIN_STEEL);
        cyl(c, 0.2, 7.0, 2.2, 2.2, col);
        ell(c, Mat3d{}, {0, 0, 7.2}, 2.2, 2.2, 1.0, col);
        FinScope g(FIN_METAL);
        tube(c, {2.3, 0, 0.3}, {2.3, 0, 8.0}, 0.08, B_METAL);
    }
    FinScope g(FIN_PAINT);
    box(c, Mat3d{}, {0, 0, 0.32}, 0.6, 0.6, 0.1, B_YELLOW);
}

// ---- power
void fissionPlant(const DrawCtx& c, const BaseLook& L, int variant, const Vec3d& toCentre) {
    {
        // the power cable to the base, laid on the ground
        FinScope f(FIN_PAINT);
        double d = len(toCentre);
        if (d > 30) {
            Vec3d u = toCentre / d;
            double a = std::atan2(u.y, u.x);
            box(c, rotZ(a), u * (d * 0.5), d - 30, 0.25, 0.12, B_DARK);
        }
    }
    if (variant == 1) {
        // Kilopower-style: reactor under a tall cone of radiators on a tripod
        FinScope f(FIN_METAL);
        for (int k = 0; k < 3; k++) {
            double a = k * TAU / 3;
            tube(c, {std::cos(a) * 3.0, std::sin(a) * 3.0, 0}, {std::cos(a) * 0.8, std::sin(a) * 0.8, 3.2}, 0.18, B_METAL);
        }
        cyl(c, 2.4, 1.4, 0.55, 0.45, Color{120, 110, 96, 255});
        cyl(c, 3.8, 0.9, 0.6, 0.6, B_PANEL);
        FinScope r(FIN_RADIATOR);
        for (int k = 0; k < 8; k++) {
            double a = k * TAU / 8;
            box(c, rotZ(a) * rotY(0.42), {std::cos(a) * 1.4, std::sin(a) * 1.4, 7.2}, 0.08, 1.15, 6.0, B_RADIATOR);
        }
        FinScope p(FIN_PAINT);
        cyl(c, 10.3, 0.3, 0.12, 0.12, Color{255, 70, 60, 255}, 0, 0, 0.6f + 2.4f * L.night);
    } else {
        // fission surface power: the reactor sunk in a regolith berm, its converters above, two radiator wings
        {
            FinScope f(FIN_REGOLITH);
            TexScope t(0, 1.6f);
            for (int k = 0; k < 14; k++) {
                double a = k * TAU / 14;
                box(c, rotZ(a) * rotY(-0.5), {std::cos(a) * 5.6, std::sin(a) * 5.6, 0.7}, 2.6, 2.7, 2.2, L.soil);
            }
        }
        {
            FinScope f(FIN_METAL);
            cyl(c, -0.5, 3.0, 1.1, 0.9, Color{120, 110, 96, 255});
            box(c, Mat3d{}, {0, 0, 3.4}, 2.6, 2.6, 1.6, B_PANEL);
            cyl(c, 4.2, 6.0, 0.25, 0.2, B_METAL);
            int wings = variant == 2 ? 4 : 2;
            for (int k = 0; k < wings; k++) {
                double a = k * TAU / wings;
                Mat3d R = rotZ(a);
                tube(c, {0, 0, 7.0}, R * Vec3d{2.0, 0, 7.0}, 0.1, B_METAL);
                FinScope r(FIN_RADIATOR);
                box(c, R, R * Vec3d{6.0, 0, 6.5}, 8.0, 0.12, 6.0, B_RADIATOR);
            }
        }
        {
            FinScope f(FIN_PAINT);
            cyl(c, 10.2, 0.3, 0.12, 0.12, Color{255, 70, 60, 255}, 0, 0, 0.6f + 2.4f * L.night);
            for (int k = 0; k < 6; k++) {
                double a = k * TAU / 6 + 0.2;
                cyl(c, 0, 1.6, 0.08, 0.08, B_YELLOW, std::cos(a) * 14, std::sin(a) * 14);
            }
        }
    }
}

void solarField(const DrawCtx& c, const BaseLook& L, int variant) {
    (void)L;
    if (variant == 1) {
        // vertical solar array towers (VSAT): a mast with a tall blanket, turned to the low sun
        for (int k = 0; k < 3; k++) {
            double x = (k - 1) * 14.0;
            FinScope f(FIN_METAL);
            cyl(c, 0, 19, 0.22, 0.16, B_METAL, x, 0);
            cyl(c, -0.2, 0.8, 1.6, 1.6, B_DARK, x, 0);
            FinScope s(FIN_SOLAR);
            box(c, rotZ(0.3 * k), {x, 0, 10.5}, 3.6, 0.06, 15.0, Color{60, 70, 110, 255});
        }
    } else if (variant == 2) {
        // roll-out blankets on the ground
        for (int k = 0; k < 6; k++) {
            FinScope s(FIN_SOLAR);
            box(c, Mat3d{}, {0, (k - 2.5) * 3.0, 0.08}, 36, 2.2, 0.04, Color{60, 70, 110, 255});
            FinScope m(FIN_METAL);
            box(c, Mat3d{}, {-18.4, (k - 2.5) * 3.0, 0.25}, 0.8, 2.4, 0.5, B_METAL);
        }
    } else {
        // tilted rows on frames
        for (int r = 0; r < 3; r++) {
            double y = (r - 1) * 9.0;
            FinScope s(FIN_SOLAR);
            box(c, rotX(-0.6), {0, y, 1.9}, 32, 4.2, 0.08, Color{60, 70, 110, 255});
            FinScope m(FIN_METAL);
            for (double x = -15; x <= 15.1; x += 5) {
                tube(c, {x, y + 1.2, 0}, {x, y + 1.0, 3.0}, 0.07, B_METAL);
                tube(c, {x, y - 1.4, 0}, {x, y - 1.2, 0.9}, 0.07, B_METAL);
            }
        }
    }
}

void commsMast(const DrawCtx& c, const BaseLook& L, int variant) {
    double H = 14.0 + variant * 3.0;
    FinScope f(FIN_METAL);
    Vec3d foot[4], top[4];
    for (int k = 0; k < 4; k++) {
        double a = k * PI / 2 + PI / 4;
        foot[k] = {std::cos(a) * 1.6, std::sin(a) * 1.6, 0};
        top[k] = {std::cos(a) * 0.5, std::sin(a) * 0.5, H};
    }
    for (int k = 0; k < 4; k++) {
        tube(c, foot[k], top[k], 0.08, B_METAL);
        for (double t = 0; t < 0.99; t += 0.25) {
            Vec3d a = foot[k] + (top[k] - foot[k]) * t, b = foot[(k + 1) % 4] + (top[(k + 1) % 4] - foot[(k + 1) % 4]) * (t + 0.25);
            tube(c, a, b, 0.04, B_METAL);
        }
    }
    ell(c, rotY(0.9) * rotZ(0.4), {0.9, 0, H - 1.0}, 1.8, 1.8, 0.35, Color{230, 232, 236, 255});
    tube(c, {0.9, 0, H - 1.0}, {1.9, 0, H - 0.3}, 0.05, B_METAL);
    FinScope p(FIN_PAINT);
    box(c, Mat3d{}, {3.2, 0, 1.0}, 2.4, 1.8, 2.0, B_PANEL);
    cyl(c, H, 0.35, 0.14, 0.14, Color{255, 70, 60, 255}, 0, 0, 0.6f + 2.4f * L.night);
}

void launchPadBase(const DrawCtx& c, const BaseLook& L, const BaseDrawSpec& s, const Vec3d& toCentre) {
    const double R = 30, deck = BASE_PAD_DECK_H, rr = std::max(0.5, s.rocketR);
    Color paver = mixc(L.soilLight, Color{170, 168, 160, 255}, 0.5f);
    {
        FinScope f(FIN_PAVER);
        TexScope t(3, 6.0f);
        cyl(c, -0.4, 0.7, R, R, paver);
        cyl(c, 0.3, 0.03, 21, 21, Color{226, 226, 220, 255});
        cyl(c, 0.32, 0.03, 19.6, 19.6, paver);
    }
    {
        // edge lights and the blast berm (gap toward the road)
        double road = std::atan2(toCentre.y, toCentre.x);
        FinScope p(FIN_PAINT);
        for (int k = 0; k < 24; k++) {
            double a = k * TAU / 24;
            box(c, Mat3d{}, {std::cos(a) * (R - 0.6), std::sin(a) * (R - 0.6), 0.4}, 0.35, 0.35, 0.18, Color{255, 214, 120, 255}, 0.3f + 2.6f * L.night);
        }
        FinScope f(FIN_REGOLITH);
        TexScope t(0, 1.8f);
        for (int k = 0; k < 22; k++) {
            double a = k * TAU / 22;
            double da = std::remainder(a - road, TAU);
            if (std::fabs(da) < 0.22) continue;
            box(c, rotZ(a) * rotY(-0.55), {std::cos(a) * (R + 13), std::sin(a) * (R + 13), 1.4}, 4.5, 2 * PI * (R + 13) / 22 * 1.05, 3.2, L.soil);
        }
    }
    {
        // the road to the base
        double d = len(toCentre);
        if (d > R + 40) {
            Vec3d u = toCentre / d;
            double a = std::atan2(u.y, u.x);
            FinScope f(FIN_CONCRETE);
            box(c, rotZ(a), u * (R + (d - R - 40) * 0.5), d - R - 40, 7, 0.2, shade(L.soil, 0.7));
        }
    }
    {
        // launch mount: a ring frame on four pillars over the flame pit
        FinScope f(FIN_METAL);
        double a = rr * 1.25 + 0.4, bw = 1.2;
        box(c, Mat3d{}, {0, a + bw / 2, deck - 0.4}, 2 * (a + bw), bw, 0.8, B_METAL);
        box(c, Mat3d{}, {0, -(a + bw / 2), deck - 0.4}, 2 * (a + bw), bw, 0.8, B_METAL);
        box(c, Mat3d{}, {a + bw / 2, 0, deck - 0.4}, bw, 2 * a, 0.8, B_METAL);
        box(c, Mat3d{}, {-(a + bw / 2), 0, deck - 0.4}, bw, 2 * a, 0.8, B_METAL);
        for (int k = 0; k < 4; k++) {
            double x = (k & 1 ? 1 : -1) * (a + bw / 2), y = (k & 2 ? 1 : -1) * (a + bw / 2);
            box(c, Mat3d{}, {x, y, (deck - 0.8) / 2 + 0.3}, 1.0, 1.0, deck - 0.8 + 0.6, Color{110, 114, 120, 255});
        }
        // service tower with its arm
        double tH = std::max(18.0, s.rocketLen + 6), tx = 0, ty = rr + 7.0, th = 2.0;
        Vec3d cols[4] = {{tx - th, ty - th, 0}, {tx + th, ty - th, 0}, {tx + th, ty + th, 0}, {tx - th, ty + th, 0}};
        Color red{178, 62, 44, 255};
        for (auto& p : cols) tube(c, p, p + Vec3d{0, 0, tH}, 0.22, red);
        for (double z = 0; z <= tH; z += 4.0)
            for (int k = 0; k < 4; k++) {
                Vec3d p0 = cols[k] + Vec3d{0, 0, z}, p1 = cols[(k + 1) % 4] + Vec3d{0, 0, z};
                tube(c, p0, p1, 0.1, red);
                if (z + 4 <= tH) tube(c, p0, p1 + Vec3d{0, 0, 4.0}, 0.07, red);
            }
        if (s.rocketOnPad) box(c, Mat3d{}, {0, (rr + ty - th) / 2, std::max(6.0, s.rocketLen * 0.7)}, 1.2, ty - th - rr - 0.3, 1.0, B_METAL);
        // four light masts
        for (int k = 0; k < 4; k++) {
            double a = k * PI / 2 + PI / 4, x = std::cos(a) * (R + 4), y = std::sin(a) * (R + 4);
            cyl(c, 0, 14, 0.25, 0.2, B_METAL, x, y);
            FinScope p(FIN_PAINT);
            box(c, rotZ(a), {x * 0.97, y * 0.97, 14.2}, 0.6, 1.6, 0.8, Color{255, 236, 200, 255}, 0.3f + 2.8f * L.night);
        }
    }
}

// a pressurised rover parked by the habitats
void parkedRover(const DrawCtx& c, const BaseLook& L) {
    FinScope f(FIN_PAINT);
    box(c, Mat3d{}, {0, 0, 1.75}, 5.8, 2.8, 2.2, B_PANEL);
    box(c, rotY(0.5), {2.75, 0, 2.1}, 1.2, 2.6, 1.6, B_PANEL);
    box(c, Mat3d{}, {3.2, 0, 2.35}, 0.2, 2.2, 0.8, B_WIN, L.winEm());
    box(c, Mat3d{}, {-2.95, 0, 1.75}, 0.15, 1.2, 1.4, B_DARK);
    FinScope m(FIN_METAL);
    for (int k = 0; k < 6; k++) {
        double x = -2.0 + (k % 3) * 2.0, y = k < 3 ? -1.6 : 1.6;
        tube(c, {x, y - 0.25, 0.62}, {x, y + 0.25, 0.62}, 0.62, B_DARK);
    }
}

} // namespace

void drawBase(const BaseDrawSpec& s, const Mat3d& rot, const Vec3d& origin, const Vec3d& lightDir, float light, const Vec3d& camPos, float ambient) {
    makeBaseTextures();
    DrawCtx c;
    c.base = toMatrix(rot, origin);
    c.lightDir = lightDir;
    c.light = light;
    c.camPos = camPos;
    c.ambient = ambient;
    setV3(shipShader, shipLight, lightDir);
    setF(shipShader, shipLightI, light);
    setF(shipShader, shipAmbient, ambient);
    setV3(shipShader, shipCamPos, camPos);
    rlDisableBackfaceCulling();
    BaseLook L;
    L.soil = s.soil;
    L.soilLight = mixc(s.soil, Color{214, 206, 194, 255}, 0.45f);
    L.night = s.night;
    // pressurised tunnels between the habitats and the greenhouses close to each other
    {
        FinScope f(FIN_FABRIC);
        TexScope t(1, 2.0f);
        for (size_t i = 0; i < s.mods.size(); i++)
            for (size_t j = i + 1; j < s.mods.size(); j++) {
                const BaseModuleDraw &a = s.mods[i], &b = s.mods[j];
                bool ai = a.kind == ModuleKind::Habitat || a.kind == ModuleKind::Greenhouse;
                bool bj = b.kind == ModuleKind::Habitat || b.kind == ModuleKind::Greenhouse;
                if (!ai || !bj || len(a.pos - b.pos) > 72) continue;
                Vec3d up{0, 0, 1.5};
                tube(c, a.pos + up, b.pos + up, 1.25, Color{226, 220, 204, 255});
            }
    }
    int storageIdx = 0;
    bool roverDone = false;
    // draw opaque base buildings first, then greenhouses (semi-transparent glass domes/tunnels) so glass never clips
    // solid modules behind it
    for (int pass = 0; pass < 2; pass++) {
        for (const BaseModuleDraw& m : s.mods) {
            bool isGreen = m.kind == ModuleKind::Greenhouse;
            if ((pass == 0) == isGreen) continue;
            DrawCtx cm = c;
            Mat3d Rm = rotZ(m.heading);
            cm.base = toMatrix(rot * Rm, origin + rot * m.pos);
            Vec3d toCentre = Rm.T() * (Vec3d{0, 0, 0} - Vec3d{m.pos.x, m.pos.y, 0});
            int v = std::clamp(m.variant, 0, 2);
            switch (m.kind) {
            case ModuleKind::Habitat:
                if (v == 0) habDome(cm, L);
                else if (v == 1) habInflatable(cm, L);
                else habTower(cm, L);
                if (!roverDone) {
                    DrawCtx cr = c;
                    cr.base = toMatrix(rot * rotZ(m.heading + 1.2), origin + rot * (m.pos + Vec3d{16, -17, 0}));
                    parkedRover(cr, L);
                    roverDone = true;
                }
                break;
            case ModuleKind::Greenhouse:
                if (v == 0) greenTunnel(cm, L);
                else if (v == 1) greenDome(cm, L);
                else greenTower(cm, L);
                break;
            case ModuleKind::Isru: isruPlant(cm, L, v); break;
            case ModuleKind::Storage: storageTank(cm, L, v, storageIdx++); break;
            case ModuleKind::Fission: fissionPlant(cm, L, v, toCentre); break;
            case ModuleKind::Solar: solarField(cm, L, v); break;
            case ModuleKind::Comms: commsMast(cm, L, v); break;
            case ModuleKind::LaunchPad: launchPadBase(cm, L, s, toCentre); break;
            default: break;
            }
        }
    }
    if (!g_shadowPass) rlEnableBackfaceCulling();
    // at night: the light of the windows, the grow lights, the pad's lamps and the beacons
    if (s.night > 0.05f) {
        struct G { Vec3d p; float size; Color c; };
        std::vector<G> glows;
        auto add = [&](const BaseModuleDraw& m, const Vec3d& local, float size, Color col) {
            glows.push_back({origin + rot * (m.pos + rotZ(m.heading) * local), size, col});
        };
        const Color warm{255, 206, 150, 255}, pink{255, 120, 210, 255}, red{255, 60, 50, 255};
        for (const BaseModuleDraw& m : s.mods) {
            int v = std::clamp(m.variant, 0, 2);
            switch (m.kind) {
            case ModuleKind::Habitat:
                if (v == 0)
                    for (int k = 0; k < 6; k++) add(m, {std::cos(k * PI / 3 + 0.2) * 8.8, std::sin(k * PI / 3 + 0.2) * 8.8, 2.6}, 3.2f, warm);
                else if (v == 1)
                    for (int k = 0; k < 4; k++) add(m, {-3.0 + k * 2.0, -3.7, 4.7}, 2.6f, warm);
                else
                    for (int k = 0; k < 5; k++) add(m, {std::cos(k * TAU / 5 + 0.3) * 4.9, std::sin(k * TAU / 5 + 0.3) * 4.9, 6.2}, 4.0f, warm);
                break;
            case ModuleKind::Greenhouse: add(m, {0, 0, 2.2}, 16.0f, pink); break;
            case ModuleKind::Isru: add(m, {-2, 0, 4.4}, 4.0f, warm); break;
            case ModuleKind::Comms: add(m, {0, 0, 14.4 + v * 3.0}, 2.4f, red); break;
            case ModuleKind::Fission: add(m, {0, 0, 10.4}, 2.4f, red); break;
            case ModuleKind::LaunchPad:
                for (int k = 0; k < 4; k++) {
                    double a = k * PI / 2 + PI / 4;
                    add(m, {std::cos(a) * 33 * 0.97, std::sin(a) * 33 * 0.97, 14.2}, 11.0f, warm);
                }
                for (int k = 0; k < 24; k++) add(m, {std::cos(k * TAU / 24) * 29.4, std::sin(k * TAU / 24) * 29.4, 0.5}, 1.6f, Color{255, 214, 120, 255});
                break;
            default: break;
            }
        }
        BeginBlendMode(BLEND_ADDITIVE);
        rlDisableDepthMask();
        for (const G& g : glows) {
            unsigned char a = (unsigned char)std::clamp(170.0f * s.night, 0.0f, 255.0f);
            facingBillboard(glowTex, toF(g.p), g.size, Color{g.c.r, g.c.g, g.c.b, a});
        }
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        EndBlendMode();
    }
}
