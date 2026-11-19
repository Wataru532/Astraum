#include "nav.hpp"
#include <algorithm>
#include <vector>
#include "render.hpp"
#include "rl.hpp"
#include "ui.hpp"
#include "keys.hpp"

namespace nav {
namespace {
int dragMode = 0;           // 0 none, 1 orbit, 2 pan, 3 zoom
int dragButton = -1;
bool havePivot = false;
Vec3d pivot;
double lastMiddleClick = -10;
bool g_reverse = false;
// right button: a drag orbits, a click (no movement) is reported to the view (e.g. map "set target")
bool rightPending = false, rightClick = false;
Vector2 rightPress{};
// view cube interaction
bool cubeDrag = false, cubeMoved = false;
Vector2 cubePress{};

Mat3d axisAngleMat(const Vec3d& axis, double a) { return Quatd::axisAngle(axis, a).toMat(); }

void setFromEye(OrbitCam& c, const Vec3d& eye) {
    Vec3d d = eye - c.target;
    double l = len(d);
    if (l < 1e-12) return;
    d = d / l;
    c.dist = l;
    c.pitch = std::asin(clampd(d.z, -1, 1));
    if (std::fabs(d.z) < 0.99999) c.yaw = std::atan2(d.y, d.x);
}

void orbitBy(OrbitCam& c, double dYaw, double dPitch, const Vec3d& P) {
    double lim = 89.5 * DEG;
    double np = clampd(c.pitch + dPitch, -lim, lim);
    dPitch = np - c.pitch;
    Vec3d fwd, right, up;
    c.basis(fwd, right, up);
    Mat3d Ry = axisAngleMat({0, 0, 1}, dYaw);
    Vec3d right2 = Ry * right;
    Mat3d Rp = axisAngleMat(right2, -dPitch);
    Mat3d R = Rp * Ry;
    Vec3d eye = c.eye();
    Vec3d newEye = P + R * (eye - P);
    c.target = P + R * (c.target - P);
    setFromEye(c, newEye);
    c.pitch = clampd(c.pitch, -lim, lim);
}

double worldPerPixel(const OrbitCam& c, Rectangle view) {
    return 2.0 * c.dist * std::tan(c.fovy * 0.5 * DEG) / std::max(1.0f, view.height);
}

void zoomAt(OrbitCam& c, Rectangle view, Vector2 at, double factor) {
    Vec3d o, d, fwd, right, up;
    viewRay(c, view, at, o, d);
    c.basis(fwd, right, up);
    Vec3d A;
    if (c.ortho) A = o + d * (c.dist + 2000.0);
    else A = o + d * (c.dist / std::max(1e-6, dot(d, fwd)));
    double nd = clampd(c.dist * factor, c.minDist, c.maxDist);
    double f = nd / c.dist;
    c.target = A + (c.target - A) * f;
    c.dist = nd;
}
} // namespace

Vec3d renderEye(const OrbitCam& c) { return c.ortho ? c.target + c.dir() * (c.dist + 2000.0) : c.eye(); }

void beginViewport3D(const OrbitCam& c, Rectangle view, double nearP, double farP) {
    rlDrawRenderBatchActive();
    ui::beginScissor(view);
    float us = ui::scale();
    rlViewport((int)(view.x * us), GetScreenHeight() - (int)((view.y + view.height) * us), (int)(view.width * us), (int)(view.height * us));
    gfx::Cam cam;
    Vec3d fwd, right, up;
    c.basis(fwd, right, up);
    cam.pos = renderEye(c);
    cam.fwd = fwd;
    cam.up = up;
    cam.fovy = (float)c.fovy;
    gfx::begin3D(cam, nearP, farP);
    rlMatrixMode(RL_PROJECTION);
    rlLoadIdentity();
    double aspect = view.width / std::max(1.0f, view.height);
    if (c.ortho) {
        double hh = c.dist * std::tan(c.fovy * 0.5 * DEG);
        rlOrtho(-hh * aspect, hh * aspect, -hh, hh, nearP, farP + c.dist + 4000.0);
    } else {
        double top = nearP * std::tan(c.fovy * 0.5 * DEG);
        rlFrustum(-top * aspect, top * aspect, -top, top, nearP, farP);
    }
    rlMatrixMode(RL_MODELVIEW);
}

void endViewport3D() {
    gfx::end3D();
    ui::endScissor();
    rlViewport(0, 0, GetRenderWidth(), GetRenderHeight());
}

void setReverseZoom(bool r) { g_reverse = r; }
bool reverseZoom() { return g_reverse; }
bool middleDragging() { return dragMode != 0 && dragButton == MOUSE_BUTTON_MIDDLE; }
bool takeRightClick() {
    bool r = rightClick;
    rightClick = false;
    return r;
}

Vec3d OrbitCam::dir() const { return {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)}; }

void OrbitCam::basis(Vec3d& fwd, Vec3d& right, Vec3d& up) const {
    fwd = -dir();
    right = {-std::sin(yaw), std::cos(yaw), 0};
    up = cross(right, fwd);
}

void OrbitCam::animateTo(double y, double p, double d, const Vec3d& t) {
    aYaw0 = yaw;
    aPitch0 = pitch;
    aDist0 = dist;
    aT0 = target;
    // shortest way round
    double dy = std::remainder(y - yaw, TAU);
    aYaw1 = yaw + dy;
    aPitch1 = clampd(p, -89.5 * DEG, 89.5 * DEG);
    aDist1 = clampd(d, minDist, maxDist);
    aT1 = t;
    animT = 0;
}

void OrbitCam::home(const Vec3d& centre, double radius) {
    double d = std::max(radius, 0.1) / std::sin(fovy * 0.5 * DEG) * 1.15;
    animateTo(homeYaw, homePitch, d, centre);
}

void OrbitCam::fit(const Vec3d& centre, double radius) {
    double d = std::max(radius, 0.1) / std::sin(fovy * 0.5 * DEG) * 1.15;
    animateTo(yaw, pitch, d, centre);
}

void OrbitCam::update(double dt) {
    if (animT < 0) return;
    animT = std::min(1.0, animT + dt / 0.35);
    double s = animT * animT * (3 - 2 * animT);
    yaw = lerpd(aYaw0, aYaw1, s);
    pitch = lerpd(aPitch0, aPitch1, s);
    dist = std::exp(lerpd(std::log(std::max(aDist0, 1e-9)), std::log(std::max(aDist1, 1e-9)), s));
    target = aT0 + (aT1 - aT0) * s;
    if (animT >= 1) animT = -1;
}

bool project(const OrbitCam& c, Rectangle view, const Vec3d& p, Vector2& out) {
    Vec3d fwd, right, up;
    c.basis(fwd, right, up);
    Vec3d e = renderEye(c);
    Vec3d d = p - e;
    double x = dot(d, right), y = dot(d, up), z = dot(d, fwd);
    double t = std::tan(c.fovy * 0.5 * DEG), aspect = view.width / std::max(1.0f, view.height);
    double nx, ny;
    if (c.ortho) {
        double hh = c.dist * t;
        nx = x / (hh * aspect);
        ny = y / hh;
    } else {
        if (z <= 1e-6) return false;
        nx = x / z / (t * aspect);
        ny = y / z / t;
    }
    out = {(float)(view.x + (nx + 1) * 0.5 * view.width), (float)(view.y + (1 - ny) * 0.5 * view.height)};
    return true;
}

void viewRay(const OrbitCam& c, Rectangle view, Vector2 p, Vec3d& o, Vec3d& d) {
    Vec3d fwd, right, up;
    c.basis(fwd, right, up);
    double nx = (p.x - view.x) / view.width * 2 - 1, ny = 1 - (p.y - view.y) / view.height * 2;
    double t = std::tan(c.fovy * 0.5 * DEG), aspect = view.width / std::max(1.0f, view.height);
    if (c.ortho) {
        double hh = c.dist * t;
        o = c.target + c.dir() * (c.dist + 2000.0) + right * (nx * hh * aspect) + up * (ny * hh);
        d = fwd;
    } else {
        o = c.eye();
        d = norm(fwd + right * (nx * t * aspect) + up * (ny * t));
    }
}

bool handleMouse(OrbitCam& c, Rectangle view, bool hoverAllowed, const PickFn& pick, const FitFn& fit) {
    Vector2 m = GetMousePosition();
    bool inside = CheckCollisionPointRec(m, view) && !ui::inputBlocked();
    Vector2 dm = GetMouseDelta();
    dm.x /= ui::scale();
    dm.y /= ui::scale();
    rightClick = false;
    // finish drags
    if (dragMode != 0 && !IsMouseButtonDown(dragButton)) { dragMode = 0; dragButton = -1; }
    // right button: decide between click and orbit drag
    if (rightPending) {
        if (!IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            rightPending = false;
            rightClick = true;
        } else if (std::fabs(m.x - rightPress.x) + std::fabs(m.y - rightPress.y) > 4 && dragMode == 0) {
            rightPending = false;
            dragButton = MOUSE_BUTTON_RIGHT;
            dragMode = 1;
            Vec3d o, d, hit;
            viewRay(c, view, rightPress, o, d);
            havePivot = pick && pick(o, d, hit);
            pivot = havePivot ? hit : c.target;
            c.animT = -1;
        }
    }
    if (dragMode == 0 && inside && hoverAllowed) {
        bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
        bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
        if (IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE)) {
            if (GetTime() - lastMiddleClick < 0.35) {
                if (fit) fit();
                lastMiddleClick = -10;
            } else {
                lastMiddleClick = GetTime();
                dragButton = MOUSE_BUTTON_MIDDLE;
                dragMode = (shift && ctrl) ? 3 : 2; // orbit is on the right button only
            }
        } else if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            rightPending = true;
            rightPress = m;
        } else if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && c.tool != 0) {
            dragButton = MOUSE_BUTTON_LEFT;
            dragMode = c.tool;
        }
        if (dragMode == 1) {
            Vec3d o, d, hit;
            viewRay(c, view, m, o, d);
            havePivot = pick && pick(o, d, hit);
            pivot = havePivot ? hit : c.target;
        }
        if (dragMode != 0) c.animT = -1;
        // wheel zoom at the cursor
        float wh = GetMouseWheelMove();
        if (wh != 0) {
            double f = std::pow(0.85, (double)wh * (g_reverse ? -1 : 1));
            c.animT = -1;
            zoomAt(c, view, m, f);
        }
        if (keys::pressed(keys::A::FitView) && fit) fit();
    }
    if (dragMode != 0 && (dm.x != 0 || dm.y != 0)) {
        switch (dragMode) {
        case 1: orbitBy(c, -dm.x * 0.008, dm.y * 0.008, pivot); break;
        case 2: {
            Vec3d fwd, right, up;
            c.basis(fwd, right, up);
            double s = worldPerPixel(c, view);
            c.target += (right * (-dm.x) + up * dm.y) * s;
            break;
        }
        case 3: c.dist = clampd(c.dist * std::exp(dm.y * 0.01), c.minDist, c.maxDist); break;
        default: break;
        }
    }
    if (dragMode != 0) ui::blockRect(view);
    return dragMode != 0;
}

// ------------------------------------------------------------------ ViewCube
Rectangle viewCubeRect(Rectangle area) {
    Vector2 C{area.x + area.width - 70, area.y + 74};
    return {C.x - 58, C.y - 62, 116, 120};
}
bool viewCube(OrbitCam& c, Rectangle area, const FitFn& home) {
    const float S = 30;                        // half size in px
    Vector2 C{area.x + area.width - 70, area.y + 74};
    Rectangle hitR{C.x - 58, C.y - 62, 116, 120};
    Vector2 m = GetMousePosition();
    bool hover = CheckCollisionPointRec(m, hitR) && !ui::inputBlocked();
    Vec3d fwd, right, up;
    c.basis(fwd, right, up);
    Vec3d back = -fwd;
    auto proj = [&](const Vec3d& v) { return Vector2{C.x + (float)dot(v, right) * S, C.y - (float)dot(v, up) * S}; };
    struct Face { Vec3d n, u, w; const char* label; };
    Face faces[6] = {{{0, 0, 1}, {1, 0, 0}, {0, 1, 0}, "TOP"},    {{0, 0, -1}, {1, 0, 0}, {0, -1, 0}, "BOTTOM"},
                     {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}, "FRONT"}, {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}, "BACK"},
                     {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, "RIGHT"},  {{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}, "LEFT"}};
    std::vector<int> order;
    for (int i = 0; i < 6; i++)
        if (dot(faces[i].n, back) > 0.01) order.push_back(i);
    std::sort(order.begin(), order.end(), [&](int a, int b) { return dot(faces[a].n, back) < dot(faces[b].n, back); });
    // hover region
    Vec3d hoverDir{};
    int hoverFace = -1;
    double ha = 0, hb = 0;
    if (hover)
        for (int k = (int)order.size() - 1; k >= 0; k--) {
            const Face& f = faces[order[k]];
            Vector2 p0 = proj(f.n), pu = proj(f.n + f.u), pw = proj(f.n + f.w);
            double ux = pu.x - p0.x, uy = pu.y - p0.y, wx = pw.x - p0.x, wy = pw.y - p0.y;
            double det = ux * wy - uy * wx;
            if (std::fabs(det) < 1e-6) continue;
            double rx = m.x - p0.x, ry = m.y - p0.y;
            double a = (rx * wy - ry * wx) / det, b = (ux * ry - uy * rx) / det;
            if (std::fabs(a) <= 1 && std::fabs(b) <= 1) {
                hoverFace = order[k];
                ha = a;
                hb = b;
                Vec3d d = f.n;
                if (std::fabs(a) > 0.55) d += f.u * (a > 0 ? 1.0 : -1.0);
                if (std::fabs(b) > 0.55) d += f.w * (b > 0 ? 1.0 : -1.0);
                hoverDir = norm(d);
                break;
            }
        }
    // draw
    DrawCircleV(C, 52, Color{14, 18, 26, 150});
    for (int idx : order) {
        const Face& f = faces[idx];
        float lit = (float)(0.55 + 0.45 * dot(f.n, back));
        Color fc{(unsigned char)(196 * lit), (unsigned char)(204 * lit), (unsigned char)(218 * lit), 235};
        Vector2 q[4] = {proj(f.n - f.u - f.w), proj(f.n + f.u - f.w), proj(f.n + f.u + f.w), proj(f.n - f.u + f.w)};
        DrawTriangle(q[0], q[1], q[2], fc);
        DrawTriangle(q[0], q[2], q[1], fc);
        DrawTriangle(q[0], q[2], q[3], fc);
        DrawTriangle(q[0], q[3], q[2], fc);
        for (int e = 0; e < 4; e++) DrawLineEx(q[e], q[(e + 1) % 4], 1.2f, Color{70, 80, 98, 255});
        if (idx == hoverFace) {
            // highlight the hovered face / edge / corner region
            double a0 = std::fabs(ha) > 0.55 ? (ha > 0 ? 0.55 : -1.0) : -0.55, a1 = std::fabs(ha) > 0.55 ? (ha > 0 ? 1.0 : -0.55) : 0.55;
            double b0 = std::fabs(hb) > 0.55 ? (hb > 0 ? 0.55 : -1.0) : -0.55, b1 = std::fabs(hb) > 0.55 ? (hb > 0 ? 1.0 : -0.55) : 0.55;
            Vector2 h[4] = {proj(f.n + f.u * a0 + f.w * b0), proj(f.n + f.u * a1 + f.w * b0), proj(f.n + f.u * a1 + f.w * b1),
                            proj(f.n + f.u * a0 + f.w * b1)};
            Color hc{56, 189, 248, 170};
            DrawTriangle(h[0], h[1], h[2], hc);
            DrawTriangle(h[0], h[2], h[1], hc);
            DrawTriangle(h[0], h[2], h[3], hc);
            DrawTriangle(h[0], h[3], h[2], hc);
        }
        if (dot(f.n, back) > 0.35) {
            Vector2 pc = proj(f.n);
            ui::textC(f.label, {pc.x - 30, pc.y - 9, 60, 18}, 10, Color{30, 36, 48, 255}, true);
        }
    }
    // home button
    Rectangle homeR{C.x - 56, C.y - 60, 22, 22};
    bool homeHover = CheckCollisionPointRec(m, homeR);
    DrawRectangleRounded(homeR, 0.3f, 4, homeHover ? Color{56, 189, 248, 220} : Color{40, 48, 64, 200});
    ui::icon(ui::Icon::Home, {homeR.x + 4, homeR.y + 4, 14, 14}, homeHover ? Color{8, 14, 22, 255} : ui::col::text);
    if (homeHover) ui::tooltip("Home view");
    if (hover) {
        ui::blockRect(hitR);
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (homeHover) {
                if (home) home();
            } else {
                cubeDrag = true;
                cubeMoved = false;
                cubePress = m;
            }
        }
    }
    if (cubeDrag) {
        Vector2 dm = GetMouseDelta();
        dm.x /= ui::scale();
        dm.y /= ui::scale();
        if (std::fabs(m.x - cubePress.x) + std::fabs(m.y - cubePress.y) > 4) cubeMoved = true;
        if (cubeMoved) orbitBy(c, -dm.x * 0.01, dm.y * 0.01, c.target);
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            if (!cubeMoved && hoverFace >= 0) {
                double p = std::asin(clampd(hoverDir.z, -1, 1));
                double y = std::fabs(hoverDir.z) > 0.999 ? c.yaw : std::atan2(hoverDir.y, hoverDir.x);
                if (std::fabs(hoverDir.z) > 0.999) y = -PI / 2; // top/bottom keep FRONT at the bottom of the screen
                c.animateTo(y, p, c.dist, c.target);
            }
            cubeDrag = false;
        }
        ui::blockRect(hitR);
    }
    return hover || cubeDrag;
}

// ------------------------------------------------------------------ navigation bar
void navBar(OrbitCam& c, Vector2 cb, const FitFn& fit, const FitFn& home, bool allowOrtho) {
    int n = allowOrtho ? 6 : 5;
    float bw = 34, gap = 4, w = n * bw + (n - 1) * gap + 16;
    Rectangle r{cb.x - w / 2, cb.y - 46, w, 42};
    ui::panel(r, Color{18, 23, 32, 215}, 10);
    float x = r.x + 8;
    auto btn = [&](ui::Icon ic, bool active, const char* tip) {
        Rectangle b{x, r.y + 4, bw, 34};
        x += bw + gap;
        return ui::iconButton(b, ic, active, tip);
    };
    if (btn(ui::Icon::NavOrbit, c.tool == 1, "Orbit (right drag)")) c.tool = c.tool == 1 ? 0 : 1;
    if (btn(ui::Icon::NavPan, c.tool == 2, "Pan (middle drag)")) c.tool = c.tool == 2 ? 0 : 2;
    if (btn(ui::Icon::NavZoom, c.tool == 3, "Zoom (wheel)")) c.tool = c.tool == 3 ? 0 : 3;
    if (btn(ui::Icon::NavFit, false, "Fit (double-click middle button)") && fit) fit();
    if (btn(ui::Icon::Home, false, "Home view") && home) home();
    if (allowOrtho && btn(ui::Icon::Cube, c.ortho, c.ortho ? "Orthographic" : "Perspective")) c.ortho = !c.ortho;
    if (c.tool != 0 && IsKeyPressed(KEY_ESCAPE)) c.tool = 0;
}

} // namespace nav
