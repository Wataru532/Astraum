#include "render.hpp"
#include <algorithm>
#include <atomic>
#include <thread>
#include <map>
#include <random>
#include "rl.hpp"
#include "site.hpp"
#include "sim.hpp" // launch sites
#include "terrain.hpp"
#include "winshell.hpp"
#include <cstring>

extern "C" __declspec(dllimport) void __stdcall glClear(unsigned int mask);
extern "C" __declspec(dllimport) void __stdcall glDrawBuffer(unsigned int mode);
extern "C" __declspec(dllimport) void __stdcall glReadBuffer(unsigned int mode);

namespace gfx {
namespace {

// ------------------------------------------------------------------ shaders
const char* PLANET_VS = R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec2 vertexTexCoord;
uniform mat4 mvp;
uniform mat4 matModel;
uniform sampler2D mapHeight; // real elevation [m] (bodies with a surface map)
uniform float dispK;         // sphere: radius fraction per metre of elevation (0 = no displacement)
out vec3 vN;
out vec3 vW;
out vec2 vUV;
in vec4 vertexTangent;
out vec3 vG;      // the near-ground cap: normal of the drawn (displaced) surface, world axes
#ifdef PBR
// the air between the camera and the ground, per vertex: it changes smoothly over the surface (the cap's
// rings are dense where the view grazes the ground), so interpolating it looks the same as computing it at
// every pixel, at a small part of the cost
uniform vec3 sunDir;
out vec3 vIns;
out vec3 vTr;
#endif
void main() {
    vN = vertexNormal;
    vUV = vertexTexCoord;
    vG = vertexTangent.xyz;
    vec3 pos = vertexPosition;
    if (dispK > 0.0) {
        vec3 n = normalize(vertexNormal);
        vec2 uv = vec2(atan(n.y, n.x) / 6.28318531 + 0.5, acos(clamp(n.z, -1.0, 1.0)) / 3.14159265);
        pos *= 1.0 + textureLod(mapHeight, uv, 0.0).r * dispK;
    }
    vW = (matModel * vec4(pos, 1.0)).xyz;
    gl_Position = mvp * vec4(pos, 1.0);
#ifdef PBR
    vIns = vec3(0.0);
    vTr = vec3(1.0);
    if (atmOn == 1) atmo(camC, normalize(vW), length(vW) * 0.001, sunDir, vIns, vTr);
#endif
}
)";

const char* PLANET_FS = R"(#version 330
in vec3 vN;
in vec3 vW;
in vec2 vUV;
uniform vec3 sunDir;
uniform mat4 bodyRot;
uniform vec3 colA;
uniform vec3 colB;
uniform vec3 colC;
uniform vec3 atmColor;
uniform float atmStrength;
uniform int kind;
uniform int dbgMode; // (diagnostics: SF_PBRDBG)
uniform float seed;
uniform int octaves;
uniform vec3 center;
uniform float radius;
uniform float haloRadius;
uniform vec3 fogColor;
uniform float fogK;
uniform float lightScale;
uniform float nightLift;   // night vision (bright mode): light everywhere, the ground in its own colours
// near-surface view (planet cap only): fine ground detail in metres around the camera, and the
// cloud layer seen from below (cloudLow 0 = under the clouds: their shadows, 1 = above: painted on)
uniform int detailOct;      // 0 = off
uniform vec3 detailOfs;     // body-fixed camera position modulo 16384 m
uniform float cloudLow;
uniform float rimK;         // limb glow seen from space (0 near the ground: aerial perspective comes from the fog)
// real surface maps (Earth, Moon, Mars): colour and elevation, equirectangular like the baked fields
uniform int hasMap;          // real colour map
uniform int hasRelief;       // elevation map (real or generated)
uniform sampler2D mapColor;
uniform sampler2D mapHeight;
uniform vec2 mapTexel;      // 1 / size of the elevation map
uniform float bodyR;        // radius [m]
uniform float reliefK;      // shading exaggeration of the relief (1 = true slopes)
// high-resolution tiles near the vessel (terrain::Detail): 4 x 4 tiles of a 16 x 8 grid, kept toroidally
// (tile (tx, ty) in slot (tx % 4, ty % 4), so the texture coordinate is simply tile position / 4)
uniform int geoN;           // 1: the cap mesh carries its surface normal (vG)
uniform float geoFar;       // distance [m] beyond which the map's relief normal takes over again
in vec3 vG;
#ifdef PBR
in vec3 vIns;
in vec3 vTr;
#endif
uniform int detOn;
uniform sampler2D detColor;
uniform sampler2D detHeight;
uniform vec2 detOrigin;     // window origin [tiles]
uniform int detMaskC;       // slots that hold the window's colour tile (bit 4 * sy + sx)
uniform int detMaskH;       // ... its elevation tile
uniform vec2 detPolar;      // 1: the window reaches the north / south pole there (no border)
uniform vec2 detTexelH;     // 1 / size of the elevation texture
out vec4 finalColor;

float hash(vec3 p) {
    // integer lattice hash (pcg3d): no periodic seams
    uvec3 q = uvec3(ivec3(p) + ivec3(1048576));
    q = q * 1664525u + 1013904223u;
    q.x += q.y * q.z; q.y += q.z * q.x; q.z += q.x * q.y;
    q ^= q >> 16u;
    q.x += q.y * q.z; q.y += q.z * q.x; q.z += q.x * q.y;
    return float(q.x & 0xFFFFFFu) / 16777215.0;
}
float vnoise(vec3 x) {
    vec3 i = floor(x);
    vec3 f = fract(x);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(mix(hash(i), hash(i + vec3(1,0,0)), f.x),
                   mix(hash(i + vec3(0,1,0)), hash(i + vec3(1,1,0)), f.x), f.y),
               mix(mix(hash(i + vec3(0,0,1)), hash(i + vec3(1,0,1)), f.x),
                   mix(hash(i + vec3(0,1,1)), hash(i + vec3(1,1,1)), f.x), f.y), f.z);
}
// value noise whose lattice repeats every 'per' cells (seamless when the offset wraps)
float vnoiseP(vec3 x, int per) {
    ivec3 i = ivec3(floor(x));
    vec3 f = fract(x);
    f = f * f * (3.0 - 2.0 * f);
    ivec3 i0 = ivec3(mod(vec3(i), float(per))), i1 = (i0 + 1) % per; // float mod: correct for negative cells
    return mix(mix(mix(hash(vec3(i0.x, i0.y, i0.z)), hash(vec3(i1.x, i0.y, i0.z)), f.x),
                   mix(hash(vec3(i0.x, i1.y, i0.z)), hash(vec3(i1.x, i1.y, i0.z)), f.x), f.y),
               mix(mix(hash(vec3(i0.x, i0.y, i1.z)), hash(vec3(i1.x, i0.y, i1.z)), f.x),
                   mix(hash(vec3(i0.x, i1.y, i1.z)), hash(vec3(i1.x, i1.y, i1.z)), f.x), f.y), f.z);
}
// ground detail from 512 m down to ~1 m wavelength (body-fixed metres): albedo variation (returned)
// and relief height in metres (hgt). Octaves finer than ~2 pixels fade out, so the texture gets
// finer with distance - the main depth cue on a smooth sphere.
float groundDetail(vec3 pb, float pxSize, int maxOct, out float hgt) {
    float a = 0.0, amp = 1.0, norm = 0.0, f = 1.0 / 512.0;
    int per = 32; // 16384 m * f
    hgt = 0.0;
    for (int k = 0; k < 10; k++) {
        if (k >= maxOct) break;
        float fade = 1.0 - smoothstep(0.12, 0.45, pxSize * f);
        if (fade <= 0.0) break;
        float v = vnoiseP(pb * f + vec3(float(k) * 17.0, float(k) * 31.0, float(k) * 5.0), per) - 0.5;
        a += v * amp * fade;
        norm += amp;
        hgt += v * fade * 0.05 / f;
        amp *= 0.78;
        f *= 2.0;
        per *= 2;
    }
    return norm > 0.0 ? a / norm * 2.2 : 0.0;
}
float fbm(vec3 p, int oct) {
    float a = 0.5, s = 0.0, norm = 0.0;
    for (int i = 0; i < 16; i++) {
        if (i >= oct) break;
        s += a * vnoise(p);
        norm += a;
        p = p * 2.03 + vec3(1.7, 9.2, 3.1);
        a *= 0.5;
    }
    return s / norm;
}

// ---- baked low octaves
// The coarse octaves of every noise field are baked once per body into an equirectangular RGBA16F
// texture (2048 x 1024, one field per channel, stored as the unnormalised partial sum of the first K
// octaves). Per pixel only the finer octaves are evaluated, so the result matches the fully
// procedural surface while most of the noise work disappears. K keeps each baked octave at least two
// texels per noise cell (lattice frequency below ~80 cells per radian).
uniform sampler2D texture0;
uniform int useBaked;
uniform int bakeMode;
uniform vec2 bakeSize;
vec4 B = vec4(0.0);
const vec3 OCT_OFS = vec3(1.7, 9.2, 3.1);

float fbmPartial(vec3 p, int K) {
    float a = 0.5, s = 0.0;
    for (int i = 0; i < 16; i++) {
        if (i >= K) break;
        s += a * vnoise(p);
        p = p * 2.03 + OCT_OFS;
        a *= 0.5;
    }
    return s;
}
// same value as fbm(p, oct): baked partial sum of octaves [0, K) plus octaves [K, oct) evaluated here
float fbmB(float baked, vec3 p, int K, int oct) {
    if (useBaked == 0 || oct < K) return fbm(p, oct);
    float s = baked, a = 0.5;
    for (int i = 0; i < 16; i++) {
        if (i >= oct) break;
        if (i >= K) s += a * vnoise(p);
        p = p * 2.03 + OCT_OFS;
        a *= 0.5;
    }
    return s / (1.0 - pow(0.5, float(oct)));
}
// per surface kind: the start point and baked octave count of each field (shared by baking and use)
void fields(vec3 n, out vec3 p0, out vec3 p1, out vec3 p2, out vec3 p3, out ivec4 K) {
    vec3 sp = n + vec3(seed);
    p0 = p1 = p2 = p3 = vec3(0.0);
    K = ivec4(0);
    if (kind == 1) {
        p0 = n * 2.2 + vec3(seed); p1 = n * 9.0 + vec3(seed * 2.0); p2 = n * 6.0; p3 = n * 4.0 + vec3(seed + 11.0, 3.0, 7.0);
        K = ivec4(6, 4, 4, 5);
    } else if (kind == 2) {
        p0 = n * vec3(3.0, 3.0, 1.5) + vec3(seed); p1 = n * vec3(6.0, 6.0, 2.0) + vec3(seed * 3.0);
        K = ivec4(5, 5, 0, 0);
    } else if (kind == 4) {
        p0 = sp * 3.0; K = ivec4(5, 0, 0, 0);
    } else if (kind == 5) {
        p0 = n * vec3(2.5, 2.5, 4.0) + vec3(seed); K = ivec4(6, 0, 0, 0);
    } else if (kind == 6) {
        p0 = sp * 2.5; p1 = sp * 14.0; K = ivec4(5, 3, 0, 0);
    } else if (kind == 0) {
        p0 = sp * 3.0; p1 = sp * 12.0; K = ivec4(5, 3, 0, 0);
    }
}
vec4 bakeFields(vec3 n) {
    vec3 p0, p1, p2, p3;
    ivec4 K;
    fields(n, p0, p1, p2, p3, K);
    return vec4(fbmPartial(p0, K.x), fbmPartial(p1, K.y), fbmPartial(p2, K.z), fbmPartial(p3, K.w));
}
// equirectangular lookup; the longitude seam takes its derivatives from a seam-free copy of u
vec4 sampleBaked(vec3 n) {
    float u = atan(n.y, n.x) / 6.28318531 + 0.5;
    float v = acos(clamp(n.z, -1.0, 1.0)) / 3.14159265;
    float u2 = fract(u + 0.5);
    vec2 dx = vec2(dFdx(u), dFdx(v)), dy = vec2(dFdy(u), dFdy(v));
    float dx2 = dFdx(u2), dy2 = dFdy(u2);
    if (abs(dx2) + abs(dy2) < abs(dx.x) + abs(dy.x)) { dx.x = dx2; dy.x = dy2; }
    return textureGrad(texture0, vec2(u, v), dx, dy);
}

// equirectangular sample with the seam-free derivative trick (as sampleBaked)
vec4 sampleEq(sampler2D s, vec3 n) {
    float u = atan(n.y, n.x) / 6.28318531 + 0.5;
    float v = acos(clamp(n.z, -1.0, 1.0)) / 3.14159265;
    float u2 = fract(u + 0.5);
    vec2 dx = vec2(dFdx(u), dFdx(v)), dy = vec2(dFdy(u), dFdy(v));
    float dx2 = dFdx(u2), dy2 = dFdy(u2);
    if (abs(dx2) + abs(dy2) < abs(dx.x) + abs(dy.x)) { dx.x = dx2; dy.x = dy2; }
    return textureGrad(s, vec2(u, v), dx, dy);
}
// the tile window at uv: tile coordinates and its weight (0 outside, at its border and on a slot that
// does not hold the window's tile yet; 1 inside)
float detWeight(vec2 uv, int mask, out vec2 tc) {
    tc = uv * vec2(16.0, 8.0);
    if (detOn == 0) return 0.0;
    float lx = mod(tc.x - detOrigin.x, 16.0), ly = tc.y - detOrigin.y;
    if (lx >= 4.0 || ly < 0.0 || ly >= 4.0) return 0.0;
    ivec2 sl = ivec2(mod(floor(tc), 4.0));
    if (((mask >> (sl.y * 4 + sl.x)) & 1) == 0) return 0.0;
    float ex = min(lx, 4.0 - lx);
    float ey = min(detPolar.x > 0.5 ? 1e9 : ly, detPolar.y > 0.5 ? 1e9 : 4.0 - ly);
    return smoothstep(0.0, 0.5, min(ex, ey));
}
// screen derivatives of the equirectangular uv without the jump at the longitude seam
void eqGrad(vec3 n, out vec2 uv, out vec2 dx, out vec2 dy) {
    float u = atan(n.y, n.x) / 6.28318531 + 0.5;
    float v = acos(clamp(n.z, -1.0, 1.0)) / 3.14159265;
    uv = vec2(u, v);
    float u2 = fract(u + 0.5);
    dx = vec2(dFdx(u), dFdx(v));
    dy = vec2(dFdy(u), dFdy(v));
    float dx2 = dFdx(u2), dy2 = dFdy(u2);
    if (abs(dx2) + abs(dy2) < abs(dx.x) + abs(dy.x)) { dx.x = dx2; dy.x = dy2; }
}
// surface normal of the elevation map (body-fixed): slopes from central differences, one texel apart
vec3 reliefNormal(vec3 n) {
    float lon = atan(n.y, n.x), colat = acos(clamp(n.z, -1.0, 1.0));
    vec2 uv = vec2(lon / 6.28318531 + 0.5, colat / 3.14159265);
    float hE = textureLod(mapHeight, uv + vec2(mapTexel.x, 0.0), 0.0).r;
    float hW = textureLod(mapHeight, uv - vec2(mapTexel.x, 0.0), 0.0).r;
    float hS = textureLod(mapHeight, uv + vec2(0.0, mapTexel.y), 0.0).r;
    float hN = textureLod(mapHeight, uv - vec2(0.0, mapTexel.y), 0.0).r;
    float sl = max(sin(colat), 0.05);
    float dE = (hE - hW) / (2.0 * mapTexel.x * 6.28318531 * bodyR * sl);
    float dN = (hN - hS) / (2.0 * mapTexel.y * 3.14159265 * bodyR);
    vec3 east = normalize(vec3(-sin(lon), cos(lon), 0.0) + vec3(1e-6));
    vec3 north = cross(n, east);
    vec3 ng = normalize(n - (dE * east + dN * north) * reliefK);
    if (detOn == 1 && detMaskH != 0) {
        vec2 tc;
        float w = detWeight(uv, detMaskH, tc);
        if (w > 0.0) {
            // the differences span at least the screen footprint (no shimmer from afar), at the mip level for it
            vec2 u0, gx, gy;
            eqGrad(n, u0, gx, gy);
            vec2 fp = (abs(gx) + abs(gy)) * vec2(4.0, 2.0); // texture units per pixel
            float st = max(detTexelH.x, fp.x);
            float lod = max(0.0, log2(st / detTexelH.x));
            float sy = st * detTexelH.y / detTexelH.x;
            vec2 t = tc / 4.0;
            float e = textureLod(detHeight, t + vec2(st, 0.0), lod).r, w2 = textureLod(detHeight, t - vec2(st, 0.0), lod).r;
            float so = textureLod(detHeight, t + vec2(0.0, sy), lod).r, no = textureLod(detHeight, t - vec2(0.0, sy), lod).r;
            float dE2 = (e - w2) / (2.0 * (st / 4.0) * 6.28318531 * bodyR * sl);
            float dN2 = (no - so) / (2.0 * (sy / 2.0) * 3.14159265 * bodyR);
            vec3 nd = normalize(n - (dE2 * east + dN2 * north) * reliefK);
            ng = normalize(mix(ng, nd, w));
        }
    }
    return ng;
}

// ---- the sea (Earth). Seen from afar the surface is a statistical glitter: its slopes spread the sun's
// reflection by the wind (Cox & Munk: mean square slope 0.003 + 0.00512 U), so the sunglint is wide where the
// wind blows and sharp in calm patches; near the camera the waves themselves are drawn - twelve trains from
// 160 m to 0.5 m wavelength (waves[]: body-fixed wave vector, phase at the camera), each fading out before it
// gets finer than a few pixels, its slope handed over to the roughness (no shimmer). Fresnel reflection of
// the sky (bright toward the horizon), whitecaps by wind speed (Monahan: 3.84e-6 U^3.41), a little colour
// variation in the water body.
uniform int waveOn;          // 1: draw the waves near the camera
uniform int waveN;           // trains drawn (terrain detail setting: 6 / 8 / 10 / 12; the rest go to the roughness)
uniform vec4 waves[12];
float gSea = 0.0;            // sea fraction of the pixel (set by surfaceColor)
float gCloud = 0.0;          // cloud cover over it
// wind over the sea [m/s] at a body-fixed direction: a field of calm patches, trade-wind belts and storms
float seaWind(vec3 n) {
    float w = vnoise(n * 4.0 + vec3(seed * 0.37, 3.1, 7.7)) * 0.5 + vnoise(n * 15.0 + vec3(1.3, seed, 5.0)) * 0.3 +
              vnoise(n * 70.0 + vec3(9.0, 2.0, seed)) * 0.2; // (down to patches of ~100 km: a mottled sunglint)
    return 1.5 + 13.0 * smoothstep(0.2, 0.8, w);
}
// the surface: shading normal (world), GGX roughness, whitecap fraction. n: body-fixed up; d: body-fixed metres
// from the camera; px: metres per pixel
void seaSurface(vec3 n, vec3 d, float px, out vec3 Nw, out float alpha, out float foam) {
    float U = seaWind(n);
    float var = 0.003 + 0.00512 * U;                      // mean square slope of the whole sea
    float W = clamp(3.84e-6 * pow(U, 3.41), 0.0, 0.08);   // whitecap cover
    vec3 grad = vec3(0.0);
    float res = 0.0, crest = 0.0, crestW = 0.0;
    float specks = W;
    if (waveOn == 1) {
        float lp = 6.2832 * 1.44 * U * U / 9.81;          // peak wavelength of a sea grown in that wind (c = 1.2 U)
        float s0 = sqrt(var / 6.0);                       // slope amplitude: 12 trains share the variance (s^2 / 2 each)
        float pm = 0.0;
        for (int i = 0; i < 12; i++) {
            if (i >= waveN) break;
            vec4 w = waves[i];
            float k = length(w.xyz), lam = 6.2832 / k;
            float fade = smoothstep(3.0 * px, 8.0 * px, lam);
            if (fade <= 0.0) break;                       // (finest last)
            float r = lam / lp;
            float sw = s0 * fade * (r > 1.0 ? exp(1.5 * (1.0 - r * r)) : 1.0); // longer than the wind sea's peak: barely there
            float s = sw + (i < 3 ? 0.022 * fade : 0.0);  // the three longest: also a swell from far away storms
            // each train rides on the longer ones (its phase bent by them): no regular lattice of crossing crests
            float th = dot(w.xyz, d) + w.w + 1.0 * pm;
            float sn = sin(th);
            float e = exp(sn - 1.0);                      // peaked crests, flat troughs
            grad += w.xyz * (s * 2.16 * cos(th) * e / k); // (2.16: the same slope variance as a sine)
            res += sw * sw * 0.5;
            crest += s * e;
            crestW += s;
            pm = sn;
        }
        grad -= n * dot(grad, n);
        if (detailOct > 0 && px < 5.0) {
            // whitecaps fixed on the water (body-fixed metres, periodic over the detail window): soft patches where
            // the crests are high, lacy inside
            vec3 pb = detailOfs + d;
            float v = vnoiseP(pb * 0.125, 2048) * 0.65 + vnoiseP(pb * 0.5 + vec3(5.0), 8192) * 0.35;
            float cr = crestW > 0.0 ? crest / crestW : 0.0;
            float t = 1.0 - 2.4 * sqrt(W) - 0.02;
            float lace = vnoiseP(pb * 2.0 + vec3(11.0), 32768);
            float blob = smoothstep(t - 0.04, t + 0.14, v + 0.25 * (cr - 0.55)) * smoothstep(2.5, 6.0, U) * (0.45 + 0.55 * lace);
            specks = mix(blob * 0.75, W, smoothstep(1.5, 5.0, px)); // (patches under a pixel: their average)
        }
    }
    alpha = sqrt(max(var - res, 0.0008));                 // the slopes not drawn: roughness
    vec3 Nb = normalize(n - grad);
    Nw = normalize(mat3(bodyRot) * Nb);
    float rf = clamp(res / var, 0.0, 1.0);
    float cn = crestW > 0.0 ? crest / crestW : 0.0;
    float thr = 0.97 - 5.0 * W;
    foam = max(specks, rf * smoothstep(thr, thr + 0.05, cn) * smoothstep(4.0, 9.0, U) * 0.8);
}
// light leaving the sea. body: colour of the water body; Ng: up (world); sunL: sunlight reaching the sea;
// skyL: light of the sky overhead; amb: ambient light on the water body and foam
vec3 seaLight(vec3 body, vec3 Ng, vec3 Ns, float alpha, float foam, vec3 V, vec3 sunL, vec3 skyL, vec3 amb) {
    // seen at a grazing angle the faces turned away hide behind the crests: the visible slopes are milder
    Ns = normalize(mix(Ns, Ng, 0.55 * (1.0 - smoothstep(0.02, 0.25, dot(Ng, V)))));
    float NV = max(dot(Ns, V), 0.02);
    float Fv = 0.02 + 0.98 * pow(1.0 - NV, 5.0);
    float sunUp = smoothstep(-0.02, 0.03, dot(Ng, sunDir));
    float NLg = max(dot(Ng, sunDir), 0.0);
    vec3 H = normalize(sunDir + V);
    float NL = max(dot(Ns, sunDir), 0.0), NH = max(dot(Ns, H), 0.0);
    float a2 = alpha * alpha, den = NH * NH * (a2 - 1.0) + 1.0;
    float D = a2 / (3.14159265 * den * den);
    float Fs = 0.02 + 0.98 * pow(1.0 - max(dot(V, H), 0.0), 5.0);
    float kk = alpha * 0.5;
    float G = NL / (NL * (1.0 - kk) + kk) * (NV / (NV * (1.0 - kk) + kk));
    // (the view's cosine over the mean surface: a facet seen edge-on does not blow the glint up)
    float NVg = max(dot(Ng, V), 0.08);
    float glint = 3.14159265 * D * Fs * G / (4.0 * max(NV, NVg)) * sunUp; // (pi: the shader's reflectance convention)
    glint = 4.0 * glint / (4.0 + glint); // (soft limit: the sun's path stays white without flooding)
    // the sky in the reflected direction: the haze at the horizon (when there is fog), deeper blue above
    vec3 R = reflect(-V, Ns);
    float up = clamp(dot(R, Ng), 0.0, 1.0);
    vec3 hz = fogK > 0.0 ? fogColor : skyL * vec3(1.9, 1.75, 1.5);
    vec3 sky = mix(hz, skyL, sqrt(up));
    vec3 c = body * (NLg * sunL * (1.0 - Fv) + amb) + sunL * glint + sky * Fv;
    return mix(c, vec3(0.80, 0.84, 0.88) * (NLg * sunL + amb + skyL * 0.6), foam);
}

vec3 surfaceColor(vec3 n, out float spec) {
    spec = 0.0;
    vec3 sp = n + vec3(seed);
    vec3 p0, p1, p2, p3;
    ivec4 K;
    fields(n, p0, p1, p2, p3, K);
    if (hasMap == 1) {
        vec3 c = sampleEq(mapColor, n).rgb;
        if (detOn == 1 && detMaskC != 0) {
            vec2 uv, gx, gy, tc;
            eqGrad(n, uv, gx, gy);
            float w = detWeight(uv, detMaskC, tc);
            if (w > 0.0) c = mix(c, textureGrad(detColor, tc / 4.0, gx * vec2(4.0, 2.0), gy * vec2(4.0, 2.0)).rgb, w);
        }
        if (kind == 1) { // Earth: the sea (sea level and blue) shines; procedural clouds on top
            float h = textureLod(mapHeight, vec2(atan(n.y, n.x) / 6.28318531 + 0.5, acos(clamp(n.z, -1.0, 1.0)) / 3.14159265), 0.0).r;
            float sea = (1.0 - smoothstep(0.5, 30.0, h)) * smoothstep(0.0, 0.06, c.b - c.r);
            spec = 0.6 * sea;
            gSea = sea;
            float cl = fbmB(B.w, p3, K.w, 6);
            float cloud = smoothstep(0.52, 0.72, cl);
            gCloud = cloud;
            c = mix(c, vec3(0.96), cloud * 0.85 * cloudLow);
            c *= 1.0 - cloud * 0.42 * (1.0 - cloudLow);
            spec *= 1.0 - cloud;
        }
        return c;
    }
    if (kind == 1) { // Earth-like
        float h = fbmB(B.x, p0, K.x, octaves);
        float lat = abs(n.z);
        vec3 c;
        if (h < 0.52) {
            c = colA * (0.55 + 0.9 * h);
            spec = 0.6;
            gSea = 1.0;
        } else {
            float m = fbmB(B.y, p1, K.y, max(octaves - 2, 2));
            vec3 green = colB * (0.8 + 0.4 * m);
            vec3 desert = colC * (0.85 + 0.3 * m);
            float dry = smoothstep(0.35, 0.65, m + (0.3 - abs(lat - 0.35)) * 0.6);
            c = mix(green, desert, dry);
            c = mix(c, vec3(0.45, 0.42, 0.38), smoothstep(0.66, 0.78, h));
        }
        float ice = smoothstep(0.80, 0.86, lat + (fbmB(B.z, p2, K.z, 4) - 0.5) * 0.12);
        c = mix(c, vec3(0.93, 0.95, 0.98), ice);
        float cl = fbmB(B.w, p3, K.w, 6);
        float cloud = smoothstep(0.52, 0.72, cl);
        c = mix(c, vec3(0.96), cloud * 0.85 * cloudLow);
        c *= 1.0 - cloud * 0.42 * (1.0 - cloudLow);
        spec *= (1.0 - cloud) * (1.0 - ice);
        gSea *= 1.0 - ice;
        gCloud = cloud;
        return c;
    }
    if (kind == 2) { // gas giant
        float t = fbmB(B.x, p0, K.x, 5);
        // belts and zones: bands of several widths, with sharper edges (smooth sine bands read as one colour)
        float b = 0.6 * sin(n.z * 22.0 + t * 4.0 + seed) + 0.4 * sin(n.z * 57.0 + t * 2.5 + seed * 1.7);
        b = clamp(b * 1.6, -1.0, 1.0);
        vec3 c = mix(colA, colB, b * 0.5 + 0.5);
        c = mix(c, colC, smoothstep(0.55, 0.8, fbmB(B.y, p1, K.y, 5)) * 0.6);
        return c;
    }
    if (kind == 4) { // icy
        float h = fbmB(B.x, p0, K.x, octaves);
        vec3 c = mix(colA, colA * 0.85, h);
        float cr = abs(vnoise(sp * 7.0) - 0.5) + abs(vnoise(sp * 17.0) - 0.5) * 0.5;
        c = mix(colB, c, smoothstep(0.02, 0.07, cr));
        spec = 0.15;
        return c;
    }
    if (kind == 5) { // thick clouds
        float t = fbmB(B.x, p0, K.x, 6);
        float b = sin(n.z * 8.0 + t * 5.0);
        return mix(colA, colB, b * 0.25 + t * 0.6);
    }
    if (kind == 6) { // desert world with caps
        float h = fbmB(B.x, p0, K.x, octaves);
        vec3 c = mix(colB, colA, smoothstep(0.3, 0.7, h));
        float cr = fbmB(B.y, p1, K.y, max(octaves - 3, 2));
        c *= 0.85 + 0.3 * cr;
        float ice = smoothstep(0.88, 0.92, abs(n.z) + (h - 0.5) * 0.1);
        return mix(c, colC, ice);
    }
    // rocky / cratered
    float h = fbmB(B.x, p0, K.x, octaves);
    vec3 c = mix(colB, colA, smoothstep(0.32, 0.68, h));
    float cr = fbmB(B.y, p1, K.y, max(octaves - 2, 2));
    float crater = smoothstep(0.62, 0.66, cr) - smoothstep(0.66, 0.74, cr) * 0.6;
    c *= 0.9 + 0.25 * crater;
    return c;
}

void main() {
    if (bakeMode == 1) {
        // texel centre -> body-fixed direction (u: longitude, v: colatitude)
        vec2 uv = gl_FragCoord.xy / bakeSize;
        float lon = (uv.x - 0.5) * 6.28318531, colat = uv.y * 3.14159265;
        finalColor = bakeFields(vec3(sin(colat) * cos(lon), sin(colat) * sin(lon), cos(colat)));
        return;
    }
    vec3 n = normalize(vN);
    vec3 V = normalize(-vW);
    float pxW = max(length(dFdx(vW)), length(dFdy(vW))); // metres per pixel (the sea's waves)
    if (useBaked == 1) B = sampleBaked(n);
    if (kind == 7) { // atmospheric halo (additive)
        vec3 dir = normalize(vW);
        float tc = dot(dir, center);
        vec3 closest = dir * tc - center;
        float b = length(closest);
        float Rh = haloRadius, R = radius;
        if (b >= Rh || tc < 0.0) discard;
        float th = (b > R) ? 2.0 * sqrt(Rh * Rh - b * b) : sqrt(Rh * Rh - b * b) - sqrt(max(R * R - b * b, 0.0));
        float tn = clamp(th / (2.0 * sqrt(Rh * Rh - R * R)), 0.0, 1.0);
        float l = smoothstep(-0.35, 0.45, dot(normalize(closest), sunDir));
        float a = tn * tn * atmStrength * l * lightScale;
        finalColor = vec4(atmColor * a, 1.0);
        return;
    }
    if (kind == 8) { // rings
        float x = vUV.x;
        float d = 0.18 * smoothstep(0.0, 0.05, x);
        d = mix(d, 0.85, smoothstep(0.25, 0.28, x));
        d = mix(d, 0.06, smoothstep(0.650, 0.660, x));
        d = mix(d, 0.65, smoothstep(0.720, 0.730, x));
        d *= 1.0 - 0.8 * (smoothstep(0.900, 0.903, x) - smoothstep(0.906, 0.909, x));
        d = mix(d, 0.0, smoothstep(0.945, 0.950, x));
        d += 0.5 * (smoothstep(0.985, 0.99, x) - smoothstep(0.993, 0.998, x));
        d *= 0.75 + 0.25 * vnoise(vec3(x * 400.0, seed, 0.0));
        vec3 col = mix(colB, colA, vnoise(vec3(x * 60.0, 1.0, seed)));
        // planet shadow
        vec3 oc = center - vW;
        float tt = dot(oc, sunDir);
        float sh = 1.0;
        if (tt > 0.0 && length(oc - sunDir * tt) < radius) sh = 0.08;
        float lit = (0.35 + 0.65 * abs(dot(mat3(bodyRot) * vec3(0.0, 0.0, 1.0), sunDir))) * sh * lightScale;
        finalColor = vec4(col * lit, clamp(d, 0.0, 1.0) * 0.9);
        return;
    }
    vec3 Nw = normalize(mat3(bodyRot) * (hasRelief == 1 ? reliefNormal(n) : n));
    float slope = 0.0;
    if (geoN == 1 && dot(vG, vG) > 0.5) {
        // near the ground: the surface of the mesh itself (map relief and the fine relief on it); far out
        // the map's normal (finer there than the mesh)
        vec3 g = normalize(vG);
        vec3 rad = normalize(mat3(bodyRot) * n);
        float w = 1.0 - smoothstep(geoFar * 0.5, geoFar, length(vW));
        Nw = normalize(mix(Nw, normalize(g + (Nw - rad) * 0.35), w));
        slope = (1.0 - dot(g, rad)) * w;
    }
    float ndl = dot(Nw, sunDir);
    if (kind == 3) { // star, emissive
        float mu = max(dot(Nw, V), 0.0);
        float g = fbm(n * 60.0 + vec3(seed), 4);
        vec3 c = colA * (0.5 + 0.5 * pow(mu, 0.45)) * (0.88 + 0.24 * g);
        finalColor = vec4(c * 1.25, 1.0);
        return;
    }
    float spec;
    vec3 albedo = surfaceColor(n, spec);
    // the sea: its own surface (waves, roughness by wind, whitecaps); weighted by the sea fraction, less where a
    // cloud deck seen from above covers it
    float wSea = kind == 1 ? gSea * (1.0 - gCloud * cloudLow) : 0.0;
    vec3 seaN = vec3(0.0);
    float seaA = 0.2, seaFoam = 0.0;
    vec3 seaBody = albedo;
    if (wSea > 0.003) {
        seaSurface(n, transpose(mat3(bodyRot)) * vW, pxW, seaN, seaA, seaFoam);
        // the water body: greener / bluer and lighter / darker in patches (plankton, currents, eddies of a few
        // to a few hundred km)
        float t = vnoise(n * 37.0 + vec3(seed)) - 0.5;
        float t2 = vnoise(n * 520.0 + vec3(3.0, seed, 1.0)) - 0.5;
        seaBody = albedo * (vec3(1.0) + t * vec3(-0.10, 0.16, 0.04) + t2 * vec3(-0.08, 0.10, 0.16)) * 0.9;
    }
    if (slope > 0.0) {
        // steep ground shows bare rock: less colour, a little darker (vegetation and dust stay on the flats)
        float r = smoothstep(0.04, 0.22, slope);
        float gray = dot(albedo, vec3(0.3, 0.5, 0.2));
        albedo = mix(albedo, vec3(gray) * vec3(1.02, 0.98, 0.94) * 0.88, r * 0.75);
    }
    if (detailOct > 0 && kind != 2 && kind != 5 && wSea < 0.999) { // (open sea: its own surface below)
        // fine detail and micro relief near the camera (vW: camera-relative metres)
        vec3 pb = detailOfs + transpose(mat3(bodyRot)) * vW;
        vec3 dpx = dFdx(vW), dpy = dFdy(vW);
        float px = max(length(dpx), length(dpy));
        float hg;
        float dv = groundDetail(pb, px, detailOct, hg);
        bool water = wSea > 0.5;
        albedo *= 1.0 + dv * 0.42 * (1.0 - wSea);
        if (!water) {
            // bump from the screen-space gradient of the relief height (surface-gradient method)
            float hx = dFdx(hg), hy = dFdy(hg);
            vec3 r1 = cross(dpy, Nw), r2 = cross(Nw, dpx);
            float det = dot(dpx, r1);
            if (abs(det) > 1e-12) {
                vec3 grad = sign(det) * (hx * r1 + hy * r2);
                Nw = normalize(abs(det) * Nw - grad);
                ndl = dot(Nw, sunDir);
            }
        }
    }
#ifdef PBR
    // the shadows of the vessels and the pad on the ground (realistic shadows)
    float vis = shadowVis(vW, normalize(mat3(bodyRot) * n), sunDir);
#else
    float vis = 1.0;
#endif
#ifdef PBR
    if (atmOn == 1 && kind != 3) {
        // the ground point [km from the centre]; the flat triangles of a coarse cap sag below the sphere between
        // their corners (km on a gas giant: the sun "set" there and only the blue air was left), so it is lifted
        // to the reference sphere at least
        vec3 P = camC + vW * 0.001;
        P = normalize(P) * max(length(P), Rkm);
        vec3 sunT = sunTransmittance(P + normalize(P) * 0.02, sunDir);
        float NL = max(ndl, 0.0);
        // skylight: blue, as much as the sun is up at that point
        float el = clamp(dot(normalize(P), sunDir) + 0.15, 0.0, 1.0);
        vec3 skyAmb = vec3(0.30, 0.45, 0.80) * 0.14 * el;
        vec3 c = albedo * NL * sunT * lightScale * vis + albedo * skyAmb * lightScale;
        if (dbgMode == 1) { finalColor = vec4(albedo, 1.0); return; }
        if (dbgMode == 2) { finalColor = vec4(sunT, 1.0); return; }
        if (dbgMode == 3) { finalColor = vec4(vTr, 1.0); return; }
        if (dbgMode == 4) { finalColor = vec4(toneIns(vIns), 1.0); return; }
        if (dbgMode == 5) { vec3 dd = sunDepth(P + normalize(P) * 0.02, sunDir); finalColor = vec4(dd.z, clamp((length(P) - Rkm) * 0.5 + 0.5, 0.0, 1.0), 0.0, 1.0); return; }
        if (dbgMode == 6) { vec3 dd = sunDepth(P + normalize(P) * 0.02, sunDir); finalColor = vec4(dd.x / (HR * 4.0), dd.y / (HM * 4.0), 0.0, 1.0); return; }
        if (wSea > 0.003) {
            vec3 Ng = normalize(mat3(bodyRot) * n);
            float shade = 1.0 - gCloud * (1.0 - cloudLow) * 0.75; // (under the cloud deck: its shadow)
            vec3 skyL = (atmColor * 0.55 + vec3(0.05)) * 0.30 * smoothstep(-0.15, 0.25, dot(Ng, sunDir)) * normalize(sunT + vec3(0.05)) * 1.7;
            vec3 cw = seaLight(seaBody, Ng, seaN, seaA, seaFoam, V, sunT * shade * vis, skyL, skyAmb) * lightScale;
            c = mix(c, cw, wSea);
        }
        c += mix(albedo, vec3(0.06, 0.16, 0.24), wSea) * nightLift;
        c = c * vTr + toneIns(vIns); // (the air in front, from the vertex shader)
        finalColor = vec4(c, 1.0);
        return;
    }
#endif
    float diff = clamp((ndl + 0.03) / 1.03, 0.0, 1.0) * vis;
    vec3 R = reflect(-sunDir, Nw);
    float s = kind == 1 ? 0.0 : pow(max(dot(R, V), 0.0), 40.0) * spec * diff; // (Earth's sea: below)
    vec3 c = albedo * (diff * lightScale + 0.03) + vec3(1.0, 0.95, 0.85) * s * 0.5 * lightScale;
    if (wSea > 0.003) {
        vec3 Ng = normalize(mat3(bodyRot) * n);
        float shade = 1.0 - gCloud * (1.0 - cloudLow) * 0.75;
        float el = dot(Ng, sunDir);
        vec3 sunL = vec3(1.0, 0.95, 0.85) * mix(vec3(1.0, 0.62, 0.42), vec3(1.0), smoothstep(0.0, 0.3, el)) * shade * vis;
        vec3 skyL = (atmColor * 0.55 + vec3(0.05)) * 0.30 * smoothstep(-0.15, 0.25, el);
        vec3 cw = seaLight(seaBody, Ng, seaN, seaA, seaFoam, V, sunL, skyL, vec3(0.03)) * lightScale;
        c = mix(c, cw, wSea);
    }
    c += mix(albedo, vec3(0.06, 0.16, 0.24), wSea) * nightLift;
    float rim = pow(1.0 - max(dot(Nw, V), 0.0), 2.5) * atmStrength * smoothstep(-0.2, 0.5, ndl);
    c += atmColor * rim * 0.9 * lightScale * rimK;
    if (fogK > 0.0) {
        float f = 1.0 - exp(-length(vW) * fogK);
        c = mix(c, fogColor, f);
    }
    finalColor = vec4(c, 1.0);
}
)";

const char* SHIP_VS = R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec2 vertexTexCoord;
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
uniform vec3 objScale;
out vec3 vN;
out vec3 vW;
out vec2 vUV;
out vec3 vP;
out vec3 vNo;
void main() {
    vN = normalize((matNormal * vec4(vertexNormal, 0.0)).xyz);
    vW = (matModel * vec4(vertexPosition, 1.0)).xyz;
    vUV = vertexTexCoord;
    vP = vertexPosition * objScale;
    vNo = vertexNormal;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)";

const char* SHIP_FS = R"(#version 330
// Procedural surface finishes modelled on real hardware (uv scaled to metres):
// 0 paint (panel seams), 1 bare aluminium / titanium (brushed), 2 spray-on foam insulation (mottled, matte),
// 3 carbon composite (weave), 4 regeneratively cooled nozzle (coolant tubes, heat tint), 5 solar cells,
// 6 multi-layer insulation foil (crinkled), 7 ablator / char, 8 stainless steel (weld rings), 9 concrete
in vec3 vN;
in vec3 vW;
in vec2 vUV;
in vec3 vP;
in vec3 vNo;
uniform vec4 colDiffuse;
uniform vec3 lightDir;
uniform float lightI;
uniform float ambient;
uniform float emissive;
uniform vec3 camPos;
uniform int finish;
uniform vec2 uvScale;
// textured finishes of the surface bases (13 printed regolith, 14 inflatable fabric, 15 crops under grow lights,
// 16 sintered hex pavers): a generated texture, mapped triplanar in metres (texM metres per repeat)
uniform sampler2D texture0;
uniform float texM;
// windows drawn into the surfaces (flush panes with their frames): world -> window frame (metres, z out of the hull),
// half width at the lower edge, half height, upper / lower width, frame width; the frame colour
uniform int nWin;
uniform mat4 winM[8];
uniform vec4 winA[8];
uniform vec3 winC[8];
// an opening cut into a hull (an open payload bay): box in the piece's own metres
uniform int cutOn;
uniform vec3 cutLo;
uniform vec3 cutHi;
uniform vec3 craftUp;       // the craft's top (+Y) in world axes: the belly of a winged craft faces away from it
uniform int shapeK;         // 1: wing panel (uv.y around the section, 0.5 = leading edge), 2: fuselage (uv.y 1 = nose)
uniform vec3 envUp;
uniform vec3 envSky;
uniform vec3 envGround;
// flood lights (launch pad at night): world positions, beam directions, warm colour x intensity
uniform int nSpots;
uniform vec3 spotPos[4];
uniform vec3 spotDir[4];
uniform vec3 spotCol;
uniform float spotRange;
#ifdef PBR
uniform vec3 sunTint;       // sunlight after its way through the air (white in space)
#endif
out vec4 finalColor;
float h21(vec2 p) { p = fract(p * vec2(123.34, 456.21)); p += dot(p, p + 45.32); return fract(p.x * p.y); }
float vnoise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(h21(i), h21(i + vec2(1, 0)), u.x), mix(h21(i + vec2(0, 1)), h21(i + vec2(1, 1)), u.x), u.y);
}
float fbm(vec2 p) {
    float s = 0.0, a = 0.5;
    for (int k = 0; k < 4; k++) { s += a * vnoise(p); p *= 2.03; a *= 0.5; }
    return s;
}
// anti-aliased line every 'period' metres ('width' metres wide), faded out where it would alias
float seam(float x, float period, float width) {
    float t = fract(x / period);
    float d = min(t, 1.0 - t) * period;
    float fw = max(fwidth(x), 1e-5);
    float l = 1.0 - smoothstep(width * 0.5, width * 0.5 + fw, d);
    return l * clamp(1.5 - fw / width * 0.25, 0.0, 1.0);
}
// fine periodic detail faded out before it turns into moire
float fine(float phase) { return clamp(1.0 - fwidth(phase) * 0.35, 0.0, 1.0); }
// signed distance [m] to a window's outline: a trapezoid with rounded corners in its plane (x across, y up)
float winDist(vec3 q, vec4 a) {
    float t = clamp(q.y / (2.0 * a.y) + 0.5, 0.0, 1.0);
    float hw = a.x * mix(1.0, a.z, t);
    float rr = min(hw, a.y) * 0.22;
    vec2 d = vec2(abs(q.x) - hw + rr, abs(q.y) - a.y + rr);
    return length(max(d, 0.0)) + min(max(d.x, d.y), 0.0) - rr;
}
// window glass: a dark cabin behind tinted panes; the sky and the ground mirrored by Fresnel's law (much more at
// a grazing view), a sharp glint of the sun, the light bloom of the horizon
vec3 glassColor(vec3 N, vec3 V, vec3 L, float li, vec3 sunC) {
    float NV = max(dot(N, V), 0.0);
    float fres = 0.04 + 0.96 * pow(1.0 - NV, 5.0);
    vec3 R = reflect(-V, N);
    float ru = dot(R, envUp);
    vec3 sky = mix(envGround * 0.85, envSky * 1.2, smoothstep(-0.12, 0.22, ru));
    sky += envSky * 0.35 * (1.0 - smoothstep(0.0, 0.35, abs(ru))); // (the bright band at the horizon)
    vec3 inside = vec3(0.016, 0.019, 0.022) * (0.7 + ambient * 3.0 + li * max(dot(N, L), 0.0) * 0.5);
    float rl = max(dot(R, L), 0.0);
    vec3 c = mix(inside, sky * vec3(0.92, 0.97, 1.0), clamp(fres + 0.06, 0.0, 1.0));
    c += sunC * li * (pow(rl, 1500.0) * 40.0 + pow(rl, 80.0) * 0.18);
    return c;
}
void main() {
    if (cutOn == 1 && all(greaterThan(vP, cutLo)) && all(lessThan(vP, cutHi))) discard;
    vec3 N = normalize(vN);
    if (!gl_FrontFacing) N = -N;
    vec3 V = normalize(camPos - vW);
    vec2 m = vUV * uvScale;
    vec3 base = colDiffuse.rgb;
    float ks = 0.22, sh = 36.0, metal = 0.0, envK = 0.06;
    if (finish == 1) {
        float ph = m.x * 60.0;
        base *= 0.9 + 0.14 * vnoise(vec2(ph, m.y * 1.5)) * fine(ph);
        ks = 0.55; sh = 60.0; metal = 0.8; envK = 0.35;
    } else if (finish == 2) {
        base *= 0.80 + 0.32 * fbm(m * 2.2) + 0.06 * vnoise(m * 17.0);
        base *= 1.0 - 0.10 * seam(m.y, 3.0, 0.06);
        ks = 0.03; sh = 6.0; envK = 0.02;
    } else if (finish == 3) {
        float px = m.x * 70.0, py = m.y * 70.0;
        float wv = sin(px) * sin(py) * fine(px) * fine(py);
        base *= 0.9 + 0.18 * wv;
        ks = 0.42; sh = 40.0; envK = 0.12;
    } else if (finish == 4) {
        float ph = vUV.x * 6.2832 * 110.0;
        float tubes = 0.5 + 0.5 * cos(ph);
        base *= 0.86 + 0.2 * tubes * fine(ph);
        base = mix(base, vec3(0.50, 0.30, 0.17), smoothstep(0.55, 1.0, vUV.y) * 0.65); // copper liner near the throat
        ks = 0.5; sh = 42.0; metal = 0.6; envK = 0.2;
    } else if (finish == 5) {
        vec2 cell = floor(m / 0.156);
        float g = max(seam(m.x, 0.156, 0.01), seam(m.y, 0.156, 0.01));
        vec3 si = vec3(0.05, 0.08, 0.22) * (0.85 + 0.3 * h21(cell));
        base = mix(si, vec3(0.72, 0.74, 0.78), g * 0.8);
        ks = 0.9; sh = 140.0; envK = 0.25;
    } else if (finish == 6) {
        float n = fbm(m * 5.0);
        vec3 pert = vec3(fbm(m * 7.0 + 3.1), fbm(m * 7.0 + 7.7), n) - 0.5;
        N = normalize(N + pert * 0.7);
        base *= 0.78 + 0.4 * n;
        ks = 1.0; sh = 30.0; metal = 1.0; envK = 0.45;
    } else if (finish == 7) {
        base *= 0.55 + 0.6 * fbm(m * 4.0);
        ks = 0.03; sh = 4.0; envK = 0.02;
    } else if (finish == 8) {
        float ph = m.x * 45.0;
        base *= 0.9 + 0.12 * vnoise(vec2(ph, m.y * 2.0)) * fine(ph);
        base *= 1.0 - 0.16 * seam(m.y, 1.83, 0.05);
        ks = 0.75; sh = 85.0; metal = 0.9; envK = 0.45;
    } else if (finish == 9) {
        // planar mapping in metres: slabs and caps get a square joint grid instead of polar uv
        vec3 an = abs(vNo);
        vec2 q = an.z > 0.5 ? vP.xy : (an.x > 0.5 ? vP.yz : vP.xz);
        base *= 0.82 + 0.26 * fbm(q * 0.9) + 0.05 * vnoise(q * 9.0);
        base *= 1.0 - 0.25 * max(seam(q.x, 6.0, 0.08), seam(q.y, 6.0, 0.08));
        ks = 0.04; sh = 8.0; envK = 0.03;
    } else if (finish == 10) {
        // reusable surface insulation of a winged entry vehicle (Space Shuttle orbiter): planar mapping in metres;
        // the belly (facing away from the craft's top) and dark skins: 15 cm black HRSI tiles with their gaps, each
        // a little different (replaced tiles, entry streaks); light skins: AFRSI quilted blankets (2.5 cm stitches,
        // ~0.9 m pieces); leading edges and the nose cap: light grey reinforced carbon-carbon
        vec3 an = abs(vNo);
        vec2 q = an.z > max(an.x, an.y) ? vP.xy : (an.x > an.y ? vP.yz : vP.xz);
        float dn = dot(N, craftUp);
        float lum = dot(base, vec3(0.299, 0.587, 0.114));
        bool rcc = (shapeK == 1 && abs(vUV.y - 0.5) < 0.06) || (shapeK == 2 && vUV.y > 0.985);
        if (rcc) {
            base = vec3(0.36, 0.36, 0.37) * (0.88 + 0.2 * fbm(q * 2.5));
            ks = 0.10; sh = 14.0; envK = 0.04;
        } else if (dn < -0.2 || lum < 0.4) {
            vec2 cell = floor(q / 0.152);
            float g = max(seam(q.x, 0.152, 0.005), seam(q.y, 0.152, 0.005));
            vec3 tb = lum < 0.4 ? base : vec3(0.085, 0.085, 0.09);
            vec3 tile = tb * (0.80 + 0.32 * h21(cell)) * (0.88 + 0.24 * fbm(q * 0.45));
            base = mix(tile, tile * 0.4, g * fine(q.x / 0.152 * 6.28));
            ks = 0.07; sh = 14.0; envK = 0.02;
        } else {
            float st = max(seam(q.x, 0.0254, 0.0012), seam(q.y, 0.0254, 0.0012));
            float pnl = max(seam(q.x, 0.91, 0.008), seam(q.y, 0.76, 0.008));
            vec2 cell = floor(q / vec2(0.91, 0.76));
            base *= (0.96 + 0.04 * h21(cell)) * (1.0 - 0.10 * st - 0.16 * pnl) * (0.96 + 0.06 * fbm(q * 1.3));
            ks = 0.05; sh = 8.0; envK = 0.03;
        }
    } else if (finish == 12) {
        // radiator panels: silver-Teflon tape over aluminium honeycomb, the coolant tubes beneath every 10 cm
        base *= (1.0 - 0.10 * seam(m.x, 0.10, 0.012)) * (0.96 + 0.06 * vnoise(m * 4.0));
        base *= 1.0 - 0.25 * seam(m.y, 4.57, 0.03);
        ks = 0.65; sh = 110.0; envK = 0.22;
    } else if (finish >= 13 && finish <= 16) {
        vec3 an = pow(abs(normalize(vNo)), vec3(4.0));
        an /= an.x + an.y + an.z + 1e-5;
        vec3 q = vP / max(texM, 0.01);
        vec3 tx = texture(texture0, q.yz).rgb * an.x + texture(texture0, q.xz).rgb * an.y + texture(texture0, q.xy).rgb * an.z;
        base *= tx;
        if (finish == 13) { ks = 0.04; sh = 6.0; envK = 0.02; }
        else if (finish == 14) { ks = 0.20; sh = 18.0; envK = 0.06; }
        else if (finish == 15) { ks = 0.10; sh = 14.0; envK = 0.03; }
        else { ks = 0.06; sh = 9.0; envK = 0.03; }
    } else if (finish != 11) {
        // paint: panel seams, rows of rivets along them (faded out with distance), a little grime
        float sy = seam(m.y, 2.4, 0.03), sx = seam(m.x, 3.1, 0.025);
        base *= 1.0 - 0.07 * sy - 0.05 * sx;
        float rv = 1.0 - smoothstep(0.0025, 0.005, length(vec2((fract(m.x / 0.05) - 0.5) * 0.05, abs((fract(m.y / 2.4 + 0.5) - 0.5) * 2.4) - 0.022)));
        base *= 1.0 - 0.10 * rv * fine(m.x * 125.0);
        base *= 0.97 + 0.05 * vnoise(m * 3.0);
        base *= 1.0 - 0.06 * smoothstep(0.55, 0.9, fbm(vec2(m.x * 0.6, m.y * 0.08)));
    }
    // windows drawn into this surface
    float gMask = finish == 11 ? 1.0 : 0.0;
    for (int i = 0; i < 8; i++) {
        if (i >= nWin) break;
        vec3 q = (winM[i] * vec4(vW, 1.0)).xyz;
        vec4 a = winA[i];
        if (abs(q.z) > max(a.x, a.y) * 0.8 + 0.15) continue;
        float sd = winDist(q, a);
        float aa = max(fwidth(sd), 1e-4);
        float fr = 1.0 - smoothstep(a.w - aa, a.w + aa, sd);
        if (fr <= 0.0) continue;
        base = mix(base, winC[i] * (0.9 + 0.1 * smoothstep(0.0, a.w, sd)), fr);
        ks = mix(ks, 0.3, fr); sh = mix(sh, 30.0, fr); metal = mix(metal, 0.0, fr); envK = mix(envK, 0.06, fr);
        gMask = max(gMask, 1.0 - smoothstep(-aa, aa, sd));
    }
    float alphaOut = colDiffuse.a;
    if (finish == 11) alphaOut *= mix(0.45, 1.0, pow(1.0 - max(dot(N, V), 0.0), 3.0)); // (a canopy: see-through face on)
#ifdef PBR
    {
        // Cook-Torrance: GGX distribution, Smith-Schlick geometry, Schlick Fresnel; energy-conserving diffuse;
        // the sky and the ground as a (roughness-blurred) environment; sunlight coloured by the air (sunTint)
        float rough = clamp(sqrt(2.0 / (sh + 2.0)) * 1.15, 0.05, 1.0);
        float a = rough * rough, a2 = a * a;
        vec3 F0 = mix(vec3(0.04), base, metal);
        float NV = max(dot(N, V), 1e-4);
        vec3 c = vec3(0.0);
        float NL = max(dot(N, lightDir), 0.0);
        float vis = NL > 0.0 ? shadowVis(vW, N, lightDir) : 1.0; // (the shadows of other parts and of the pad)
        if (NL > 0.0) {
            vec3 Hh = normalize(lightDir + V);
            float NH = max(dot(N, Hh), 0.0), VH = max(dot(V, Hh), 0.0);
            float den = NH * NH * (a2 - 1.0) + 1.0;
            float Dd = a2 / (3.14159265 * den * den);
            float k = (rough + 1.0) * (rough + 1.0) / 8.0;
            float G = (NL / (NL * (1.0 - k) + k)) * (NV / (NV * (1.0 - k) + k));
            vec3 F = F0 + (1.0 - F0) * pow(1.0 - VH, 5.0);
            vec3 spec = Dd * G * F / max(4.0 * NL * NV, 1e-4);
            vec3 kd = (1.0 - F) * (1.0 - metal);
            c += (kd * base + spec * 3.14159265) * NL * lightI * sunTint * vis;
        }
        vec3 Fa = F0 + (max(vec3(1.0 - rough), F0) - F0) * pow(1.0 - NV, 5.0);
        float upk = dot(N, envUp) * 0.5 + 0.5;
        vec3 irr = mix(envGround, envSky, upk);
        vec3 Rr = reflect(-V, N);
        vec3 envR = mix(envGround, envSky, smoothstep(-0.2 - rough, 0.3 + rough, dot(Rr, envUp)));
        envR = mix(envR, irr, rough);
        c += (1.0 - Fa) * (1.0 - metal) * base * irr * ambient * 1.4 + Fa * envR * (0.25 + ambient);
        c += base * emissive;
        for (int i = 0; i < 4; i++) {
            if (i >= nSpots) break;
            vec3 Lv = spotPos[i] - vW;
            float d = length(Lv);
            vec3 L = Lv / max(d, 1e-3);
            float cone = smoothstep(0.80, 0.93, dot(-L, spotDir[i]));
            float att = 1.0 / (1.0 + (d / spotRange) * (d / spotRange));
            float nl = max(dot(N, L), 0.0);
            c += base * (1.0 - metal) * nl * spotCol * (cone * att) + F0 * nl * spotCol * (cone * att) * 0.5;
        }
        if (gMask > 0.0) c = mix(c, glassColor(N, V, lightDir, lightI * vis, sunTint), gMask);
        finalColor = vec4(c, alphaOut);
        return;
    }
#endif
    float diff = max(dot(N, lightDir), 0.0) * lightI;
    vec3 H = normalize(lightDir + V);
    float spec = pow(max(dot(N, H), 0.0), sh) * ks * lightI * (0.5 + sh / 80.0);
    vec3 specCol = mix(vec3(1.0), base * 1.6, metal);
    // hemisphere ambient and a sky / ground reflection give the surfaces their depth
    float up = dot(N, envUp) * 0.5 + 0.5;
    vec3 hemi = mix(envGround, envSky, up);
    vec3 R = reflect(-V, N);
    vec3 env = mix(envGround, envSky, smoothstep(-0.2, 0.3, dot(R, envUp)));
    float fres = pow(1.0 - max(dot(N, V), 0.0), 5.0);
    vec3 c = base * (ambient * (0.55 + 0.9 * hemi) + diff) + specCol * spec + base * emissive;
    c += env * mix(vec3(envK), base * envK * 2.0, metal) * (0.4 + 0.6 * fres) * (0.3 + lightI);
    c += vec3(fres * 0.05) * ambient * 4.0;
    for (int i = 0; i < 4; i++) {
        if (i >= nSpots) break;
        vec3 Lv = spotPos[i] - vW;
        float d = length(Lv);
        vec3 L = Lv / max(d, 1e-3);
        float cone = smoothstep(0.80, 0.93, dot(-L, spotDir[i]));       // ~45 degree beam with a soft edge
        float att = 1.0 / (1.0 + (d / spotRange) * (d / spotRange));
        float ndl = max(dot(N, L), 0.0);
        vec3 Hs = normalize(L + V);
        float sp = pow(max(dot(N, Hs), 0.0), sh) * ks * (0.5 + sh / 80.0);
        c += (base * ndl + mix(vec3(1.0), base * 1.6, metal) * sp * ndl) * spotCol * (cone * att);
    }
    if (gMask > 0.0) c = mix(c, glassColor(N, V, lightDir, lightI, vec3(1.0)), gMask);
    finalColor = vec4(c, alphaOut);
}
)";

// ---- physically based atmosphere (PBR builds only): single scattering, Rayleigh + Mie, in km from the centre
const char* ATMO_GLSL = R"(
uniform int atmOn;          // 1: the body has air to scatter in
uniform vec3 camC;          // camera position relative to the body centre [km], world axes
uniform float Rkm;          // radius [km]
uniform float Akm;          // top of the atmosphere [km from the centre]
uniform vec3 betaR;         // Rayleigh scattering at the surface [1/km]
uniform vec3 betaM;         // Mie scattering at the surface [1/km]
uniform vec3 betaMe;        // Mie extinction (scattering + absorption) at the surface [1/km]
uniform float HR;           // scale heights [km]
uniform float HM;
uniform float mieG;
uniform float sunI;         // sunlight intensity (Earth at 1 AU ~ 22)
uniform float expo;         // exposure of the scattered light
uniform sampler2D transLut; // optical depths (Rayleigh, Mie) to the top of the air, precomputed per body
uniform int useLut;
vec2 raySphere(vec3 ro, vec3 rd, float r) {
    float b = dot(ro, rd), c = dot(ro, ro) - r * r, d = b * b - c;
    if (d < 0.0) return vec2(1e9, -1e9);
    d = sqrt(d);
    return vec2(-b - d, -b + d);
}
// the table's coordinates (Bruneton's mapping: even steps in the distance to the top of the air, so the
// rays near the horizon, where the depth changes fastest, get as many texels as the steep ones)
vec2 lutUV(float r, float mu) {
    float H = sqrt(max(Akm * Akm - Rkm * Rkm, 1e-6));
    float rho = sqrt(max(r * r - Rkm * Rkm, 0.0));
    float d = max(-r * mu + sqrt(max(r * r * (mu * mu - 1.0) + Akm * Akm, 0.0)), 0.0);
    float dmin = Akm - r, dmax = rho + H;
    float xm = (d - dmin) / max(dmax - dmin, 1e-6), xr = rho / H;
    return vec2(0.5 / 256.0 + xm * (255.0 / 256.0), 0.5 / 64.0 + xr * (63.0 / 64.0));
}
// optical depths (Rayleigh, Mie) from p toward the sun, and how much of the sun's disc is above the horizon
// there (z: 0 in the planet's shadow, 1 in full sun). The shadow edge is not a hard line: the sun's disc
// (0.53 deg) sets with a penumbra a few km wide at the height the light passes - a hard edge made a step in
// the twilight sky wherever a sample crossed it.
vec3 sunDepth(vec3 p, vec3 s) {
    float r = length(p);
    float mu = dot(p, s) / r;
    // the sun ray's lowest height; with the sun above the horizon the ray climbs, and how far it clears the
    // ground grows with the sun's elevation (r mu^2 / 2: the same curve as below the horizon, mirrored). Taking
    // r - R there put every point at sea level (Earth's oceans, a gas giant's whole cloud deck) in half shadow
    float hmin = mu < 0.0 ? r * sqrt(max(1.0 - mu * mu, 0.0)) - Rkm : r - Rkm + 0.5 * r * mu * mu;
    float pen = max(0.5, 0.0093 * sqrt(2.0 * Rkm * max(r - Rkm, 0.0) + 1.0) * 0.5);
    float vis = smoothstep(-pen, pen, hmin);
    if (vis <= 0.0) return vec3(-1.0, -1.0, 0.0);
    if (useLut == 1 && r <= Akm) return vec3(textureLod(transLut, lutUV(max(r, Rkm), mu), 0.0).rg, vis);
    vec2 tg = raySphere(p, s, Rkm);
    if (tg.x > 0.0 && tg.y > 0.0) return vec3(-1.0, -1.0, 0.0);
    float tl = raySphere(p, s, Akm).y;
    if (tl <= 0.0) return vec3(0.0, 0.0, 1.0);
    // samples spaced quadratically: dense in the thick air near the start
    float dR = 0.0, dM = 0.0;
    for (int j = 0; j < 6; j++) {
        float u0 = float(j) / 6.0, u1 = float(j + 1) / 6.0;
        float ta = tl * u0 * u0, tb = tl * u1 * u1;
        float h = length(p + s * (0.5 * (ta + tb))) - Rkm;
        dR += exp(-h / HR) * (tb - ta);
        dM += exp(-h / HM) * (tb - ta);
    }
    return vec3(dR, dM, 1.0);
}
vec3 sunTransmittance(vec3 p, vec3 s) {
    vec3 d = sunDepth(p, s);
    if (d.z <= 0.0) return vec3(0.0);
    return exp(-(betaR * d.x + betaMe * d.y)) * d.z;
}
// the light scattered toward the camera along the ray (ins) and what of the background gets through (tr)
void atmo(vec3 ro, vec3 rd, float tEnd, vec3 s, out vec3 ins, out vec3 tr) {
    ins = vec3(0.0);
    tr = vec3(1.0);
    vec2 ta = raySphere(ro, rd, Akm);
    float t0 = max(ta.x, 0.0), t1 = min(ta.y, tEnd);
    if (t1 <= t0) return;
    float odR = 0.0, odM = 0.0;
    vec3 sR = vec3(0.0), sM = vec3(0.0);
    // the ray is split at its lowest point (closest to the centre: the densest air it crosses), and the
    // samples are spaced quadratically, dense toward that point on both sides. From the ground looking up the
    // lowest point is the camera itself; from high up toward the horizon it lies far away, where a sampling
    // dense only at the camera left coarse steps.
    // (always the same number of samples on each side: a ray that just tips below the horizontal must give
    // nearly the same result as one just above it, or the sky shows a line there)
    float tc = clamp(-dot(ro, rd), t0, t1);
    const int nA = 6, nB = 12;
    for (int i = 0; i < nA + nB; i++) {
        float ta, tb;
        if (i < nA) {
            float u0 = 1.0 - float(i) / float(nA), u1 = 1.0 - float(i + 1) / float(nA);
            ta = tc - (tc - t0) * u0 * u0;
            tb = tc - (tc - t0) * u1 * u1;
        } else {
            float u0 = float(i - nA) / float(nB), u1 = float(i - nA + 1) / float(nB);
            ta = tc + (t1 - tc) * u0 * u0;
            tb = tc + (t1 - tc) * u1 * u1;
        }
        float ds = tb - ta;
        vec3 p = ro + rd * (0.5 * (ta + tb));
        float h = length(p) - Rkm;
        float dR = exp(-h / HR) * ds, dM = exp(-h / HM) * ds;
        odR += dR;
        odM += dM;
        vec3 l = sunDepth(p, s);
        if (l.z <= 0.0) continue;
        vec3 att = exp(-(betaR * (odR + l.x) + betaMe * (odM + l.y))) * l.z;
        sR += dR * att;
        sM += dM * att;
    }
    float mu = dot(rd, s), g = mieG;
    float pR = 0.0596831 * (1.0 + mu * mu);
    float pM = 0.1193662 * ((1.0 - g * g) * (1.0 + mu * mu)) / ((2.0 + g * g) * pow(max(1.0 + g * g - 2.0 * g * mu, 1e-4), 1.5));
    ins = sunI * (sR * betaR * pR + sM * betaM * pM);
    tr = exp(-(betaR * odR + betaMe * odM));
}
vec3 toneIns(vec3 x) { return vec3(1.0) - exp(-x * expo); }
)";

const char* SKY_FS_MAIN = R"(
uniform vec2 res;
uniform vec3 cFwd;
uniform vec3 cRight;
uniform vec3 cUp;
uniform float tanY;
uniform float aspect;
uniform vec3 sunDir;
out vec4 finalColor;
void main() {
    vec2 ndc = gl_FragCoord.xy / res * 2.0 - 1.0;
    vec3 rd = normalize(cFwd + ndc.x * tanY * aspect * cRight + ndc.y * tanY * cUp);
    vec2 tp = raySphere(camC, rd, Rkm);
    // (a miss gives (1e9, -1e9): the ground is hit only when the far root is ahead too - testing the near
    // root alone took every sky ray for ground and added a dim ground colour, cut off in a line)
    bool ground = tp.x > 0.0 && tp.y > 0.0;
    float tEnd = ground ? tp.x : 1e9;
    vec3 ins, tr;
    atmo(camC, rd, tEnd, sunDir, ins, tr);
    vec3 col = toneIns(ins);
    if (ground) {
        // the ground beyond the drawn terrain (toward the horizon): lit through the air, seen through it
        vec3 P = camC + rd * tp.x, n = normalize(P);
        float NL = max(dot(n, sunDir), 0.0);
        vec3 g = vec3(0.09, 0.11, 0.08) * NL * sunTransmittance(P + n * 0.02, sunDir) + vec3(0.02, 0.03, 0.05) * clamp(dot(n, sunDir) + 0.15, 0.0, 1.0);
        col = g * tr + toneIns(ins);
    }
    float mu = dot(rd, sunDir);
    if (!ground) col += tr * smoothstep(0.99996, 0.99999, mu) * 4.0; // the sun's disc through the air
    // +-half a step of 8-bit noise: smooth gradients instead of bands in the dark sky
    float dn = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
    col += (dn - 0.5) / 255.0;
    finalColor = vec4(clamp(col, 0.0, 1.0), 1.0);
}
)";

const char* PLUME_VS = R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
out float vT;
out vec3 vN;
out vec3 vW;
void main() {
    vT = -vertexPosition.z;
    vN = normalize((matNormal * vec4(vertexNormal, 0.0)).xyz);
    vW = (matModel * vec4(vertexPosition, 1.0)).xyz;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)";

const char* PLUME_FS = R"(#version 330
in float vT;
in vec3 vN;
in vec3 vW;
uniform vec4 colDiffuse;
uniform vec3 camPos;
uniform float limb;
out vec4 finalColor;
void main() {
    vec3 V = normalize(camPos - vW);
    float facing = abs(dot(normalize(vN), V));
    float shade = mix(0.25 + 0.75 * facing, 0.08 + 0.92 * pow(1.0 - facing, 1.5), limb);
    float a = pow(clamp(1.0 - vT, 0.0, 1.0), 1.7) * colDiffuse.a * shade;
    finalColor = vec4(colDiffuse.rgb * a, 1.0);
}
)";

const char* NAV_FS = R"(#version 330
in vec2 fragTexCoord;
in vec4 fragColor;
uniform mat4 orient;
out vec4 finalColor;
const float PI = 3.14159265;
void main() {
    vec2 p = fragTexCoord * 2.0 - 1.0;
    p.y = -p.y;
    float r2 = dot(p, p);
    if (r2 > 1.0) discard;
    float nz = sqrt(1.0 - r2);
    vec3 local = vec3(-p.x, p.y, nz);
    vec3 h = mat3(orient) * local;
    float el = asin(clamp(h.z, -1.0, 1.0));
    float hd = atan(h.y, h.x);
    vec3 sky = vec3(0.20, 0.46, 0.84);
    vec3 gnd = vec3(0.56, 0.36, 0.20);
    vec3 c = el > 0.0 ? mix(sky * 1.15, sky * 0.65, el / 1.5708) : mix(gnd * 1.1, gnd * 0.55, -el / 1.5708);
    float px = 0.012 / max(nz, 0.15);
    float step = PI / 6.0;
    float ge = abs(fract(el / step + 0.5) - 0.5) * step;
    float gh = abs(fract(hd / step + 0.5) - 0.5) * step * cos(el);
    float line = 1.0 - smoothstep(px * 0.5, px, min(ge, gh));
    c = mix(c, vec3(0.95), line * 0.35);
    float hz = 1.0 - smoothstep(px, px * 2.0, abs(el));
    c = mix(c, vec3(1.0), hz * 0.9);
    c *= 0.55 + 0.45 * nz;
    float rim = smoothstep(0.90, 1.0, r2);
    c = mix(c, vec3(0.05, 0.06, 0.08), rim * 0.8);
    finalColor = vec4(c, 1.0);
}
)";

float g_nightLift = 0; // night vision (bright mode): ambient light added to the ground
struct PlanetLocs {
    int atmOn = -1, camC = -1, Rkm = -1, Akm = -1, betaR = -1, betaM = -1, betaMe = -1, HR = -1, HM = -1, mieG = -1, sunI = -1, expo = -1;
    int transLut = -1, useLut = -1;
    int sunDir, bodyRot, colA, colB, colC, atmColor, atmStrength, kind, seed, octaves, center, radius, haloRadius, fogColor, fogK, lightScale;
    int nightLift = -1;
    int detailOct = -1, detailOfs = -1, cloudLow = -1, rimK = -1;
    int useBaked, bakeMode, bakeSize;
    int hasMap = -1, hasRelief = -1, mapTexel = -1, bodyR = -1, reliefK = -1, dispK = -1;
    int detOn = -1, detOrigin = -1, detMaskC = -1, detMaskH = -1, detPolar = -1, detTexelH = -1;
    int geoN = -1, geoFar = -1;
    int waveOn = -1, waves = -1, waveN = -1;
};

// ---- baked planet noise fields (see the planet shader): one RGBA16F equirectangular texture per body,
// created on first use (at most one per frame, outside any 3D pass) and kept in a small LRU set
constexpr int BAKE_W = 2048, BAKE_H = 1024, BAKE_MAX = 8; // 8 x ~21 MB with mipmaps
struct BakedField {
    unsigned fbo = 0;
    Texture2D tex{};
    double lastUse = 0;
};
std::map<int, BakedField> bakedFields;
struct BakeRequest {
    int body, kind;
    float seed;
};
std::vector<BakeRequest> bakeQueue;
int g_maxPlanetOctaves = 16; // lowered by the adaptive frame-rate control
bool bakeFailed = false;

Shader planetShader, shipShader, plumeShader, navShader, plasmaShader;
Material plasmaMat;
int plasmaThick = -1, plasmaFlow = -1, plasmaCam = -1, plasmaHot = -1, plasmaInten = -1, plasmaTime = -1;
int shipFinish = -1, shipUV = -1, shipEnvUp = -1, shipEnvSky = -1, shipEnvGround = -1;
int shipNSpots = -1, shipSpotPos = -1, shipSpotDir = -1, shipSpotCol = -1, shipSpotRange = -1;
// current surface finish / uv scale for piece(); set through FinScope
int g_fin = 0;
Vector2 g_uv{1, 1};
Vector3 g_scale{1, 1, 1};
int g_shapeK = 0; // (shapeK of the next piece)
// the shadow pass (realistic shadows): the parts only as depth seen from the sun, nothing see-through
bool g_shadowPass = false;
Material g_depthMat{};
int shipNWin = -1, shipWinM[8] = {-1, -1, -1, -1, -1, -1, -1, -1}, shipWinA = -1, shipWinC = -1, shipCutOn = -1, shipCutLo = -1, shipCutHi = -1,
    shipCraftUp = -1, shipShapeK = -1;
int shipObjScale = -1;
struct FinScope {
    int prev;
    explicit FinScope(int f) : prev(g_fin) { g_fin = f; }
    ~FinScope() { g_fin = prev; }
};
PlanetLocs PL;
int shipLight, shipLightI, shipAmbient, shipEmissive, shipCamPos, plumeCamPos, navOrient;
Material planetMat, shipMat, plumeMat, starMat;
Mesh sphereMesh, sphereLo, cubeMesh, capMesh, starMesh, plumeMesh, wedgeMesh, wakeMesh, sheathMesh;
std::map<int, Mesh> ringCache;
Shader mwShader{};
int mwLodLoc = -1, mwRowLoc[3] = {-1, -1, -1};
// the Milky Way: each pixel turns its direction (the sphere's own coordinates: the ecliptic frame) into RA / Dec and
// samples the equatorial map at a level of detail from the size of a screen pixel on the sky - isotropic, so the
// poles and the RA 12h seam look like the rest of the sky (texture coordinates on a mesh streak there)
const char* MW_VS = R"(#version 330
in vec3 vertexPosition;
uniform mat4 mvp;
out vec3 dirObj;
void main() {
    dirObj = vertexPosition;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)";
const char* MW_FS = R"(#version 330
in vec3 dirObj;
uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform float lod;
uniform vec3 eqRow0, eqRow1, eqRow2; // ecliptic -> equatorial
out vec4 finalColor;
void main() {
    vec3 e = normalize(dirObj);
    vec3 d = vec3(dot(eqRow0, e), dot(eqRow1, e), dot(eqRow2, e));
    float ra = atan(d.y, d.x), dec = asin(clamp(d.z, -1.0, 1.0));
    vec2 uv = vec2(fract(0.5 - ra / 6.2831853), 0.5 - dec / 3.14159265);
    vec3 c = textureLod(texture0, uv, lod).rgb;
    c = c * c * 0.35 + c * 0.65; // (the map is gamma-encoded: back toward linear, the dark sky between darker)
    finalColor = vec4(c * colDiffuse.rgb, 1.0);
}
)";
Texture2D mwTex{};
Material mwMat{};
bool mwReady = false;
float g_milkyWay = 0.62f; // brightness of the Milky Way layer
int plumeLimb = -1;
Texture2D whiteTex, glowTex;
// camera-facing sprite: raylib's DrawBillboard keeps world +Y as the sprite's up axis, which turns the
// sprite edge-on (a thin streak) whenever the view looks along world Y; use the camera's own up instead
void facingBillboard(Texture2D tex, Vector3 pos, float size, Color c);
Texture2D shadowTex{};
Camera3D curCam;
std::map<std::pair<int, int>, Mesh> frustumCache;
// open half-shells (fairing clamshells): outer skin, inner skin
std::map<std::pair<int, int>, std::pair<Mesh, Mesh>> halfCache;
// contoured nozzle (outer skin with the exit lip, inner wall) and parachute canopy gores (two colours)
Mesh bellOuter{}, bellInner{}, canopyMesh[2]{};
constexpr float BELL_THROAT = 0.28f;

// the near-ground mesh: denser with the terrain detail setting (Low, Medium, High, Ultra)
int CAP_RINGS = 80, CAP_SEGS = 120;
int g_terrainQ = 2, g_capBuiltQ = -1;
float g_detRange = 1.0f; // detail range (settings slider)
const int CAP_Q_RINGS[4] = {60, 80, 110, 150}, CAP_Q_SEGS[4] = {96, 128, 180, 270};
const double CAP_Q_MINWL[4] = {12.0, 5.0, 2.0, 1.0};    // the finest relief drawn [m]
const double CAP_Q_LODK[4] = {0.14, 0.09, 0.06, 0.04};   // relief smaller than this fraction of the distance is left out
const int CAP_Q_DOCT[4] = {3, 5, 8, 10};                 // shader detail octaves at the most
std::vector<float> capPos, capNrm, capTan;
struct CapCache {
    bool valid = false, relief = false;
    int body = -1, q = -1;
    unsigned rev = 0;
    float detRange = 1;
    Vec3d camBF, camRel0;
    Mat3d rot0;
    double alt = 1;
} g_cap; // the cap mesh as last built (see drawBodyCap)

Mesh allocMesh(int vcount, int tcount) {
    Mesh m{};
    m.vertexCount = vcount;
    m.triangleCount = tcount;
    m.vertices = (float*)MemAlloc(vcount * 3 * sizeof(float));
    m.normals = (float*)MemAlloc(vcount * 3 * sizeof(float));
    m.texcoords = (float*)MemAlloc(vcount * 2 * sizeof(float));
    m.indices = (unsigned short*)MemAlloc(tcount * 3 * sizeof(unsigned short));
    return m;
}

// Unit frustum: bottom radius rb at z=0, top radius rt at z=1, with caps
Mesh genFrustum(float rb, float rt, int segs) {
    int vcount = (segs + 1) * 2 + (segs + 2) * 2;
    int tcount = segs * 2 + segs * 2;
    Mesh m = allocMesh(vcount, tcount);
    int v = 0, t = 0;
    float dz = rb - rt;
    for (int i = 0; i <= segs; i++) {
        float a = (float)i / segs * 2 * PI;
        float c = cosf(a), s = sinf(a);
        Vector3 n = Vector3Normalize({c, s, dz});
        for (int k = 0; k < 2; k++) {
            float r = k == 0 ? rb : rt;
            m.vertices[v * 3] = c * r; m.vertices[v * 3 + 1] = s * r; m.vertices[v * 3 + 2] = (float)k;
            m.normals[v * 3] = n.x; m.normals[v * 3 + 1] = n.y; m.normals[v * 3 + 2] = n.z;
            m.texcoords[v * 2] = (float)i / segs; m.texcoords[v * 2 + 1] = (float)k;
            v++;
        }
    }
    for (int i = 0; i < segs; i++) {
        int b0 = i * 2, t0 = i * 2 + 1, b1 = (i + 1) * 2, t1 = (i + 1) * 2 + 1;
        unsigned short* ix = m.indices + t * 3;
        ix[0] = b0; ix[1] = b1; ix[2] = t1; ix[3] = b0; ix[4] = t1; ix[5] = t0;
        t += 2;
    }
    for (int cap = 0; cap < 2; cap++) {
        float z = (float)cap, r = cap == 0 ? rb : rt, nz = cap == 0 ? -1.f : 1.f;
        int centre = v;
        m.vertices[v * 3] = 0; m.vertices[v * 3 + 1] = 0; m.vertices[v * 3 + 2] = z;
        m.normals[v * 3] = 0; m.normals[v * 3 + 1] = 0; m.normals[v * 3 + 2] = nz;
        m.texcoords[v * 2] = 0.5f; m.texcoords[v * 2 + 1] = 0.5f;
        v++;
        for (int i = 0; i <= segs; i++) {
            float a = (float)i / segs * 2 * PI;
            m.vertices[v * 3] = cosf(a) * r; m.vertices[v * 3 + 1] = sinf(a) * r; m.vertices[v * 3 + 2] = z;
            m.normals[v * 3] = 0; m.normals[v * 3 + 1] = 0; m.normals[v * 3 + 2] = nz;
            m.texcoords[v * 2] = 0; m.texcoords[v * 2 + 1] = 0;
            v++;
        }
        for (int i = 0; i < segs; i++) {
            unsigned short* ix = m.indices + t * 3;
            if (cap == 0) { ix[0] = centre; ix[1] = centre + 2 + i; ix[2] = centre + 1 + i; }
            else { ix[0] = centre; ix[1] = centre + 1 + i; ix[2] = centre + 2 + i; }
            t++;
        }
    }
    m.vertexCount = v;
    m.triangleCount = t;
    UploadMesh(&m, false); // (raylib's DrawMesh needs mesh.indices to stay set: keep the CPU copy)
    return m;
}

const Mesh& frustum(double rb, double rt) {
    double mx = std::max(std::max(rb, rt), 1e-6);
    int kb = (int)std::lround(rb / mx * 64), kt = (int)std::lround(rt / mx * 64);
    auto key = std::make_pair(kb, kt);
    auto it = frustumCache.find(key);
    if (it != frustumCache.end()) return it->second;
    Mesh m = genFrustum(kb / 64.f, kt / 64.f, 40);
    return frustumCache.emplace(key, m).first->second;
}

// UV sphere with exact unit normals (pole along z, no degenerate pole fans)
Mesh genSphere(int rings, int segs) {
    int vcount = (rings + 1) * (segs + 1);
    Mesh m = allocMesh(vcount, rings * segs * 2);
    for (int r = 0; r <= rings; r++) {
        float th = PI * r / rings;
        for (int s = 0; s <= segs; s++) {
            float ph = 2 * PI * s / segs;
            int v = r * (segs + 1) + s;
            float x = sinf(th) * cosf(ph), y = sinf(th) * sinf(ph), z = cosf(th);
            m.vertices[v * 3] = x; m.vertices[v * 3 + 1] = y; m.vertices[v * 3 + 2] = z;
            m.normals[v * 3] = x; m.normals[v * 3 + 1] = y; m.normals[v * 3 + 2] = z;
            m.texcoords[v * 2] = (float)s / segs; m.texcoords[v * 2 + 1] = (float)r / rings;
        }
    }
    int t = 0;
    for (int r = 0; r < rings; r++)
        for (int s = 0; s < segs; s++) {
            int a = r * (segs + 1) + s, b = a + segs + 1;
            unsigned short* ix = m.indices + t * 3;
            ix[0] = a; ix[1] = b; ix[2] = b + 1; ix[3] = a; ix[4] = b + 1; ix[5] = a + 1;
            t += 2;
        }
    UploadMesh(&m, false);
    return m;
}

Mesh genRing(float inner, int segs) {
    int vcount = (segs + 1) * 2;
    Mesh m = allocMesh(vcount, segs * 2);
    for (int i = 0; i <= segs; i++) {
        float a = (float)i / segs * 2 * PI;
        for (int k = 0; k < 2; k++) {
            int v = i * 2 + k;
            float r = k == 0 ? inner : 1.0f;
            m.vertices[v * 3] = cosf(a) * r; m.vertices[v * 3 + 1] = sinf(a) * r; m.vertices[v * 3 + 2] = 0;
            m.normals[v * 3] = 0; m.normals[v * 3 + 1] = 0; m.normals[v * 3 + 2] = 1;
            m.texcoords[v * 2] = (float)k; m.texcoords[v * 2 + 1] = 0;
        }
    }
    for (int i = 0; i < segs; i++) {
        unsigned short* ix = m.indices + i * 6;
        int a = i * 2, b = i * 2 + 1, c = i * 2 + 2, d = i * 2 + 3;
        ix[0] = a; ix[1] = c; ix[2] = d; ix[3] = a; ix[4] = d; ix[5] = b;
    }
    UploadMesh(&m, false);
    return m;
}

void buildCapMesh() {
    CAP_RINGS = CAP_Q_RINGS[g_terrainQ];
    CAP_SEGS = CAP_Q_SEGS[g_terrainQ];
    g_capBuiltQ = g_terrainQ;
    int vcount = 1 + CAP_RINGS * CAP_SEGS;
    int tcount = CAP_SEGS + (CAP_RINGS - 1) * CAP_SEGS * 2;
    capMesh = allocMesh(vcount, tcount);
    std::fill(capMesh.vertices, capMesh.vertices + vcount * 3, 0.f);
    std::fill(capMesh.normals, capMesh.normals + vcount * 3, 0.f);
    std::fill(capMesh.texcoords, capMesh.texcoords + vcount * 2, 0.f);
    int t = 0;
    auto idx = [](int ring, int seg) { return 1 + ring * CAP_SEGS + (seg % CAP_SEGS); };
    for (int s = 0; s < CAP_SEGS; s++) {
        unsigned short* ix = capMesh.indices + t * 3;
        ix[0] = 0; ix[1] = idx(0, s); ix[2] = idx(0, s + 1);
        t++;
    }
    for (int r = 0; r < CAP_RINGS - 1; r++)
        for (int s = 0; s < CAP_SEGS; s++) {
            unsigned short* ix = capMesh.indices + t * 3;
            ix[0] = idx(r, s); ix[1] = idx(r + 1, s); ix[2] = idx(r + 1, s + 1);
            ix[3] = idx(r, s); ix[4] = idx(r + 1, s + 1); ix[5] = idx(r, s + 1);
            t += 2;
        }
    capMesh.tangents = (float*)MemAlloc(vcount * 4 * sizeof(float));
    std::fill(capMesh.tangents, capMesh.tangents + vcount * 4, 0.f);
    UploadMesh(&capMesh, true);
    capPos.resize(vcount * 3);
    capNrm.resize(vcount * 3);
    capTan.assign(vcount * 4, 0.f);
}

// the naked-eye sky: every star of the Yale Bright Star Catalogue (to ~6.5 mag) at its J2000 place, its size and
// brightness from its magnitude, its colour from B-V (assets/sky). Without the data: random stars as before.
Color starColour(double bv) {
    bv = std::clamp(bv, -0.4, 2.0);
    double T = 4600.0 * (1.0 / (0.92 * bv + 1.7) + 1.0 / (0.92 * bv + 0.62)); // Ballesteros (2012)
    // black body -> sRGB (Tanner Helland's fit), then mostly white: the eye sees star colours faintly
    double t = T / 100.0, r, g, b;
    r = t <= 66 ? 255 : 329.698727446 * std::pow(t - 60, -0.1332047592);
    g = t <= 66 ? 99.4708025861 * std::log(t) - 161.1195681661 : 288.1221695283 * std::pow(t - 60, -0.0755148492);
    b = t >= 66 ? 255 : (t <= 19 ? 0 : 138.5177312231 * std::log(t - 10) - 305.0447927307);
    auto mix = [](double c) { return (unsigned char)std::clamp(0.45 * std::clamp(c, 0.0, 255.0) + 0.55 * 245.0, 0.0, 255.0); };
    return Color{mix(r), mix(g), mix(b), 255};
}
void buildStarfield() {
    struct Star { Vec3d d; double mag; Color c; };
    std::vector<Star> stars;
    const void* data = nullptr;
    size_t size = 0;
    if (winshell::resource(221, data, size) && size >= 4) {
        uint32_t n = 0;
        std::memcpy(&n, data, 4);
        const unsigned char* p = (const unsigned char*)data + 4;
        n = (uint32_t)std::min<size_t>(n, (size - 4) / 16);
        for (uint32_t i = 0; i < n && stars.size() < 16000; i++) {
            float f[4];
            std::memcpy(f, p + i * 16, 16);
            stars.push_back({raDecToEcl(f[0] / DEG, f[1] / DEG), f[2], starColour(f[3])});
        }
    }
    if (stars.empty()) {
        std::mt19937 rng(20261103);
        std::uniform_real_distribution<double> U01(0, 1);
        for (int i = 0; i < 9000; i++) {
            double z = U01(rng) * 2 - 1, a = U01(rng) * TAU, rr = std::sqrt(1 - z * z);
            stars.push_back({{rr * std::cos(a), rr * std::sin(a), z}, 6.5 - 6 * std::pow(U01(rng), 3.5), Color{240, 240, 250, 255}});
        }
    }
    const int N = (int)stars.size();
    Mesh m{};
    m.vertexCount = N * 4;
    m.triangleCount = N * 2;
    m.vertices = (float*)MemAlloc(m.vertexCount * 3 * sizeof(float));
    m.texcoords = (float*)MemAlloc(m.vertexCount * 2 * sizeof(float));
    m.normals = (float*)MemAlloc(m.vertexCount * 3 * sizeof(float));
    m.colors = (unsigned char*)MemAlloc(m.vertexCount * 4);
    m.indices = (unsigned short*)MemAlloc(m.triangleCount * 3 * sizeof(unsigned short));
    for (int i = 0; i < N; i++) {
        const Star& st = stars[i];
        Vec3d d = norm(st.d);
        // 0 at the faintest (6.6 mag) .. 1 at Sirius (-1.46): a bright star is a larger, fuller glow
        double t = std::clamp((6.6 - st.mag) / 8.06, 0.0, 1.0);
        double size = 0.0019 + 0.0072 * std::pow(t, 1.4);
        float br = (float)(0.24 + 0.76 * std::pow(t, 0.85));
        Vec3d e1 = anyPerp(d), e2 = cross(d, e1);
        Vec3d corners[4] = {d - e1 * size - e2 * size, d + e1 * size - e2 * size, d + e1 * size + e2 * size, d - e1 * size + e2 * size};
        for (int k = 0; k < 4; k++) {
            int v = i * 4 + k;
            m.vertices[v * 3] = (float)corners[k].x; m.vertices[v * 3 + 1] = (float)corners[k].y; m.vertices[v * 3 + 2] = (float)corners[k].z;
            m.normals[v * 3] = 0; m.normals[v * 3 + 1] = 0; m.normals[v * 3 + 2] = 1;
            m.texcoords[v * 2] = (k == 1 || k == 2) ? 1.f : 0.f; m.texcoords[v * 2 + 1] = (k >= 2) ? 1.f : 0.f;
            m.colors[v * 4] = st.c.r; m.colors[v * 4 + 1] = st.c.g; m.colors[v * 4 + 2] = st.c.b; m.colors[v * 4 + 3] = (unsigned char)(br * 255);
        }
        unsigned short* ix = m.indices + i * 6;
        int b0 = i * 4;
        ix[0] = b0; ix[1] = b0 + 1; ix[2] = b0 + 2; ix[3] = b0; ix[4] = b0 + 2; ix[5] = b0 + 3;
    }
    UploadMesh(&m, false);
    starMesh = m;
    // the Milky Way: a sphere whose texture coordinates follow the map's equatorial grid (assets/sky/milkyway.jpg:
    // RA 12h at u = 0, RA increasing to the left, Dec +90 at v = 0); its vertices turned into the game's ecliptic frame
    if (winshell::resource(222, data, size)) {
        Image img = LoadImageFromMemory(".jpg", (const unsigned char*)data, (int)size);
        if (img.data) {
            mwTex = LoadTextureFromImage(img);
            UnloadImage(img);
            GenTextureMipmaps(&mwTex);
            SetTextureFilter(mwTex, TEXTURE_FILTER_TRILINEAR);
            SetTextureWrap(mwTex, TEXTURE_WRAP_CLAMP);
            mwShader = LoadShaderFromMemory(MW_VS, MW_FS);
            mwLodLoc = GetShaderLocation(mwShader, "lod");
            mwRowLoc[0] = GetShaderLocation(mwShader, "eqRow0");
            mwRowLoc[1] = GetShaderLocation(mwShader, "eqRow1");
            mwRowLoc[2] = GetShaderLocation(mwShader, "eqRow2");
            // ecliptic -> equatorial: the transpose of what raDecToEcl does
            Vec3d cx = raDecToEcl(0, 0), cy = raDecToEcl(90, 0), cz = raDecToEcl(0, 90);
            Vector3 r0{(float)cx.x, (float)cx.y, (float)cx.z}, r1{(float)cy.x, (float)cy.y, (float)cy.z}, r2{(float)cz.x, (float)cz.y, (float)cz.z};
            SetShaderValue(mwShader, mwRowLoc[0], &r0, SHADER_UNIFORM_VEC3);
            SetShaderValue(mwShader, mwRowLoc[1], &r1, SHADER_UNIFORM_VEC3);
            SetShaderValue(mwShader, mwRowLoc[2], &r2, SHADER_UNIFORM_VEC3);
            mwMat = LoadMaterialDefault();
            mwMat.shader = mwShader;
            mwMat.maps[MATERIAL_MAP_DIFFUSE].texture = mwTex;
            mwReady = mwShader.id != 0 && mwShader.id != rlGetShaderIdDefault();
        }
    }
}

// unit wedge: right-triangle prism, x,z in [-0.5,0.5], y in [-0.5,0.5]; hypotenuse from (+x,-z) to (-x,+z)
Mesh genWedge() {
    const float h = 0.5f;
    Vector3 A{-h, -h, -h}, B{h, -h, -h}, C{-h, -h, h}, A2{-h, h, -h}, B2{h, h, -h}, C2{-h, h, h};
    struct F { Vector3 a, b, c; };
    std::vector<F> tris = {
        {A, B, C}, {A2, C2, B2},                 // y- / y+ triangles (counter-clockwise seen from outside)
        {A, B2, B}, {A, A2, B2},                 // bottom (z-)
        {A, C2, A2}, {A, C, C2},                 // back (x-)
        {B, C2, C}, {B, B2, C2},                 // hypotenuse (+x +z)
    };
    Mesh m = allocMesh((int)tris.size() * 3, (int)tris.size());
    int v = 0;
    for (size_t t = 0; t < tris.size(); t++) {
        Vector3 n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(tris[t].b, tris[t].a), Vector3Subtract(tris[t].c, tris[t].a)));
        Vector3 pts[3] = {tris[t].a, tris[t].b, tris[t].c};
        for (int k = 0; k < 3; k++, v++) {
            m.vertices[v * 3] = pts[k].x; m.vertices[v * 3 + 1] = pts[k].y; m.vertices[v * 3 + 2] = pts[k].z;
            m.normals[v * 3] = n.x; m.normals[v * 3 + 1] = n.y; m.normals[v * 3 + 2] = n.z;
            m.texcoords[v * 2] = 0; m.texcoords[v * 2 + 1] = 0;
            m.indices[v] = (unsigned short)v;
        }
    }
    UploadMesh(&m, false);
    return m;
}

// entry plasma hugging the vehicle: every part drawn again, pushed out along its normals by the
// shock-layer stand-off, glowing where the surface faces the flow (strongest at the stagnation point,
// fading toward the shoulders) and a little at the silhouette (the layer seen edge-on)
const char* PLASMA_VS = R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
uniform float thick;
out vec3 vN;
out vec3 vW;
void main() {
    vec3 n = normalize((matNormal * vec4(vertexNormal, 0.0)).xyz);
    vec4 w = matModel * vec4(vertexPosition, 1.0);
    w.xyz += n * thick;
    vN = n;
    vW = w.xyz;
    gl_Position = mvp * (inverse(matModel) * w);
}
)";
const char* PLASMA_FS = R"(#version 330
in vec3 vN;
in vec3 vW;
uniform vec3 flowDir;  // the direction of motion (the flow comes from there)
uniform vec3 camPos;
uniform vec3 hot;
uniform float inten;
uniform float time;
out vec4 finalColor;
void main() {
    vec3 n = normalize(vN);
    float wind = dot(n, flowDir);
    float face = pow(max(wind, 0.0), 1.4);
    float side = clamp(1.0 - abs(wind), 0.0, 1.0) * step(-0.25, wind); // the shoulders, where the flow turns
    vec3 v = normalize(camPos - vW);
    float edge = pow(1.0 - abs(dot(n, v)), 2.0);
    // a slow flicker over the surface (turbulent shock layer)
    float fl = 0.93 + 0.07 * sin(time * 11.0) * sin(time * 7.3 + 1.0);
    float a = inten * fl * (face * 1.1 + side * 0.35 + edge * 0.3 * max(wind + 0.4, 0.0));
    vec3 c = mix(hot, vec3(1.0, 0.92, 0.85), face * face * 0.6);
    finalColor = vec4(c * a, a);
}
)";
// plasma wake: an open cone that widens downstream (radius r0 at z = 0 to 1 at z = -1)
Mesh genWake(int segs, int rings, float r0) {
    Mesh m = allocMesh((segs + 1) * (rings + 1), segs * rings * 2);
    for (int k = 0; k <= rings; k++) {
        float t = (float)k / rings;
        float r = r0 + (1.0f - r0) * std::sqrt(t); // blunt nose, widening quickly, then slowly
        for (int i = 0; i <= segs; i++) {
            float a = (float)i / segs * 2 * PI;
            int v = k * (segs + 1) + i;
            m.vertices[v * 3] = cosf(a) * r; m.vertices[v * 3 + 1] = sinf(a) * r; m.vertices[v * 3 + 2] = -t;
            m.normals[v * 3] = cosf(a); m.normals[v * 3 + 1] = sinf(a); m.normals[v * 3 + 2] = 0.3f;
            m.texcoords[v * 2] = 0; m.texcoords[v * 2 + 1] = t;
        }
    }
    int q = 0;
    for (int k = 0; k < rings; k++)
        for (int i = 0; i < segs; i++) {
            unsigned short* ix = m.indices + q * 6;
            int a = k * (segs + 1) + i, b = a + 1, c = a + segs + 1, d = c + 1;
            ix[0] = a; ix[1] = c; ix[2] = d; ix[3] = a; ix[4] = d; ix[5] = b;
            q++;
        }
    UploadMesh(&m, false);
    return m;
}

// bell nozzle radius along its axis: s = 0 at the exit (radius 1) .. 1 at the throat; it expands
// quickly right after the throat and straightens toward the exit (Rao-style contour)
float bellProfile(float s) { return BELL_THROAT + (1.0f - BELL_THROAT) * powf(std::max(0.0f, 1.0f - s), 0.45f); }
// open bell: exit at z = 0, throat at z = 1. inner = false: the outer skin plus the lip at the exit;
// inner = true: the inner wall (normals inward), slightly inset
Mesh genBell(bool inner, int segs, int rings) {
    const float wall = 0.965f;
    int vcount = (rings + 1) * (segs + 1) + (inner ? 0 : (segs + 1) * 2);
    int tcount = rings * segs * 2 + (inner ? 0 : segs * 2);
    Mesh m = allocMesh(vcount, tcount);
    for (int k = 0; k <= rings; k++) {
        float t = (float)k / rings;
        float sv = 1.0f - powf(1.0f - t, 1.6f); // denser toward the throat, where the contour bends
        float zs = std::min(sv, 0.999f);
        float r = bellProfile(zs) * (inner ? wall : 1.0f);
        float e = 2e-3f;
        float dr = (bellProfile(std::min(zs + e, 0.999f)) - bellProfile(std::max(zs - e, 0.0f))) / (std::min(zs + e, 0.999f) - std::max(zs - e, 0.0f));
        float nr = 1.0f, nz = -dr, nl = std::sqrt(nr * nr + nz * nz);
        nr /= nl;
        nz /= nl;
        if (inner) { nr = -nr; nz = -nz; }
        for (int i = 0; i <= segs; i++) {
            float a = (float)i / segs * 2 * PI;
            int v = k * (segs + 1) + i;
            m.vertices[v * 3] = cosf(a) * r; m.vertices[v * 3 + 1] = sinf(a) * r; m.vertices[v * 3 + 2] = sv;
            m.normals[v * 3] = cosf(a) * nr; m.normals[v * 3 + 1] = sinf(a) * nr; m.normals[v * 3 + 2] = nz;
            m.texcoords[v * 2] = (float)i / segs; m.texcoords[v * 2 + 1] = sv;
        }
    }
    int q = 0;
    for (int k = 0; k < rings; k++)
        for (int i = 0; i < segs; i++) {
            unsigned short* ix = m.indices + q * 3;
            int a0 = k * (segs + 1) + i, b0 = a0 + 1, c0 = a0 + segs + 1, d0 = c0 + 1;
            if (!inner) { ix[0] = a0; ix[1] = c0; ix[2] = d0; ix[3] = a0; ix[4] = d0; ix[5] = b0; }
            else { ix[0] = a0; ix[1] = d0; ix[2] = c0; ix[3] = a0; ix[4] = b0; ix[5] = d0; }
            q += 2;
        }
    if (!inner) {
        // exit lip: the annulus between the outer skin and the inner wall
        int base = (rings + 1) * (segs + 1);
        for (int i = 0; i <= segs; i++) {
            float a = (float)i / segs * 2 * PI;
            for (int k = 0; k < 2; k++) {
                int v = base + i * 2 + k;
                float r = k == 0 ? 1.0f : wall;
                m.vertices[v * 3] = cosf(a) * r; m.vertices[v * 3 + 1] = sinf(a) * r; m.vertices[v * 3 + 2] = 0;
                m.normals[v * 3] = 0; m.normals[v * 3 + 1] = 0; m.normals[v * 3 + 2] = -1;
                m.texcoords[v * 2] = (float)i / segs; m.texcoords[v * 2 + 1] = 0;
            }
        }
        for (int i = 0; i < segs; i++) {
            unsigned short* ix = m.indices + q * 3;
            int a0 = base + i * 2, b0 = a0 + 1, c0 = a0 + 2, d0 = a0 + 3;
            ix[0] = a0; ix[1] = b0; ix[2] = d0; ix[3] = a0; ix[4] = d0; ix[5] = c0;
            q += 2;
        }
    }
    UploadMesh(&m, false);
    return m;
}

// open half of a frustum (angles -90..90 deg, the +x side): radius rb at z = 0, rt at z = 1
Mesh genHalfShell(float rb, float rt, bool inner, int segs) {
    const float k = inner ? 0.985f : 1.0f;
    Mesh m = allocMesh((segs + 1) * 2, segs * 2);
    float dz = rb - rt;
    for (int i = 0; i <= segs; i++) {
        float a = -PI / 2 + PI * i / segs;
        float c = cosf(a), sn = sinf(a);
        Vector3 n = Vector3Normalize({c, sn, dz});
        if (inner) n = {-n.x, -n.y, -n.z};
        for (int j = 0; j < 2; j++) {
            int v = i * 2 + j;
            float r = (j == 0 ? rb : rt) * k;
            m.vertices[v * 3] = c * r; m.vertices[v * 3 + 1] = sn * r; m.vertices[v * 3 + 2] = (float)j;
            m.normals[v * 3] = n.x; m.normals[v * 3 + 1] = n.y; m.normals[v * 3 + 2] = n.z;
            m.texcoords[v * 2] = (float)i / segs; m.texcoords[v * 2 + 1] = (float)j;
        }
    }
    for (int i = 0; i < segs; i++) {
        unsigned short* ix = m.indices + i * 6;
        int b0 = i * 2, t0 = i * 2 + 1, b1 = i * 2 + 2, t1 = i * 2 + 3;
        if (!inner) { ix[0] = b0; ix[1] = b1; ix[2] = t1; ix[3] = b0; ix[4] = t1; ix[5] = t0; }
        else { ix[0] = b0; ix[1] = t1; ix[2] = b1; ix[3] = b0; ix[4] = t0; ix[5] = t1; }
    }
    UploadMesh(&m, false);
    return m;
}
const std::pair<Mesh, Mesh>& halfShell(double rb, double rt) {
    double mx = std::max(std::max(rb, rt), 1e-6);
    int kb = (int)std::lround(rb / mx * 64), kt = (int)std::lround(rt / mx * 64);
    auto key = std::make_pair(kb, kt);
    auto it = halfCache.find(key);
    if (it != halfCache.end()) return it->second;
    auto pr = std::make_pair(genHalfShell(kb / 64.f, kt / 64.f, false, 24), genHalfShell(kb / 64.f, kt / 64.f, true, 24));
    return halfCache.emplace(key, pr).first->second;
}

// parachute canopy: 16 sewn gores (bulging between the radial seams), apex vent and a scalloped hem.
// Unit size: hem radius ~1 at z ~0.18, apex at z ~0.72. parity selects every other gore (two colours);
// both faces are included (inside lit from inside).
Mesh genCanopy(int parity) {
    const int G = 16, across = 4, rings = 12;
    const float ph0 = 0.12f, ph1 = 1.32f;
    auto P = [&](float ph, float th) {
        float gt = th / (2 * PI / G);
        float t = gt - std::floor(gt);
        float bulge = 1.0f + 0.07f * sinf(PI * t);
        float hem = std::clamp((ph - (ph1 - 0.3f)) / 0.3f, 0.0f, 1.0f);
        float scallop = 0.05f * sinf(PI * t) * hem * hem;
        return Vector3{sinf(ph) * bulge * cosf(th), sinf(ph) * bulge * sinf(th), 0.72f * cosf(ph) - scallop};
    };
    int per = (rings + 1) * (across + 1);
    int gores = G / 2;
    Mesh m = allocMesh(per * gores * 2, rings * across * 2 * gores * 2);
    int v = 0, q = 0;
    for (int side = 0; side < 2; side++)
        for (int g = parity; g < G; g += 2) {
            int base = v;
            for (int k = 0; k <= rings; k++) {
                float ph = ph0 + (ph1 - ph0) * k / rings;
                for (int i = 0; i <= across; i++) {
                    float th = (g + (float)i / across) * 2 * PI / G;
                    Vector3 p = P(ph, th);
                    float e = 1e-3f;
                    Vector3 dph = Vector3Subtract(P(ph + e, th), P(ph - e, th));
                    Vector3 dth = Vector3Subtract(P(ph, th + e), P(ph, th - e));
                    Vector3 n = Vector3Normalize(Vector3CrossProduct(dth, dph));
                    if (Vector3DotProduct(n, p) < 0) n = Vector3Negate(n);
                    if (side == 1) n = Vector3Negate(n);
                    m.vertices[v * 3] = p.x; m.vertices[v * 3 + 1] = p.y; m.vertices[v * 3 + 2] = p.z;
                    m.normals[v * 3] = n.x; m.normals[v * 3 + 1] = n.y; m.normals[v * 3 + 2] = n.z;
                    m.texcoords[v * 2] = (float)i / across; m.texcoords[v * 2 + 1] = (float)k / rings;
                    v++;
                }
            }
            for (int k = 0; k < rings; k++)
                for (int i = 0; i < across; i++) {
                    unsigned short* ix = m.indices + q * 3;
                    int a0 = base + k * (across + 1) + i, b0 = a0 + 1, c0 = a0 + across + 1, d0 = c0 + 1;
                    if (side == 0) { ix[0] = a0; ix[1] = c0; ix[2] = d0; ix[3] = a0; ix[4] = d0; ix[5] = b0; }
                    else { ix[0] = a0; ix[1] = d0; ix[2] = c0; ix[3] = a0; ix[4] = b0; ix[5] = d0; }
                    q += 2;
                }
        }
    m.vertexCount = v;
    m.triangleCount = q;
    UploadMesh(&m, false);
    return m;
}

Mesh genPlume(int segs) {
    // open cone from z=0 (radius 1) to z=-1 (radius 1, scaled per draw by flare via matrix tweak)
    Mesh m = allocMesh((segs + 1) * 2, segs * 2);
    for (int i = 0; i <= segs; i++) {
        float a = (float)i / segs * 2 * PI;
        for (int k = 0; k < 2; k++) {
            int v = i * 2 + k;
            float r = k == 0 ? 1.0f : 1.0f;
            m.vertices[v * 3] = cosf(a) * r; m.vertices[v * 3 + 1] = sinf(a) * r; m.vertices[v * 3 + 2] = k == 0 ? 0.f : -1.f;
            m.normals[v * 3] = cosf(a); m.normals[v * 3 + 1] = sinf(a); m.normals[v * 3 + 2] = 0;
            m.texcoords[v * 2] = 0; m.texcoords[v * 2 + 1] = (float)k;
        }
    }
    for (int i = 0; i < segs; i++) {
        unsigned short* ix = m.indices + i * 6;
        int a = i * 2, b = i * 2 + 1, c = i * 2 + 2, d = i * 2 + 3;
        ix[0] = a; ix[1] = c; ix[2] = d; ix[3] = a; ix[4] = d; ix[5] = b;
    }
    UploadMesh(&m, false);
    return m;
}

void setV3(Shader& s, int loc, Vector3 v) { SetShaderValue(s, loc, &v, SHADER_UNIFORM_VEC3); }
void setV3(Shader& s, int loc, const Vec3d& v) { Vector3 f = toF(v); SetShaderValue(s, loc, &f, SHADER_UNIFORM_VEC3); }
void setF(Shader& s, int loc, float f) { SetShaderValue(s, loc, &f, SHADER_UNIFORM_FLOAT); }
void setI(Shader& s, int loc, int i) { SetShaderValue(s, loc, &i, SHADER_UNIFORM_INT); }

enum Finish { FIN_PAINT = 0, FIN_METAL, FIN_FOAM, FIN_CARBON, FIN_NOZZLE, FIN_SOLAR, FIN_FOIL, FIN_ABLATOR, FIN_STEEL, FIN_CONCRETE,
              FIN_TILES, FIN_GLASS, FIN_RADIATOR, FIN_REGOLITH, FIN_FABRIC, FIN_GROW, FIN_PAVER };
int finishForProp(Prop p) {
    switch (p) {
    case Prop::Hydrolox:
    case Prop::LH2: return FIN_FOAM;     // unpainted spray-on foam on cryogenic hydrogen tanks
    case Prop::Methalox: return FIN_STEEL; // stainless-steel stages
    case Prop::Xenon: return FIN_CARBON;  // carbon-overwrapped pressure vessels
    case Prop::Monoprop: return FIN_METAL; // titanium tanks
    default: return FIN_PAINT;
    }
}
int finishForMaterial(int m) {
    static const int F[] = {FIN_PAINT, FIN_METAL, FIN_STEEL, FIN_CARBON, FIN_METAL, FIN_CARBON, FIN_ABLATOR, FIN_TILES, FIN_GLASS};
    return m >= 0 && m < (int)(sizeof F / sizeof F[0]) ? F[m] : FIN_PAINT;
}

// atmosphere of a body for the scattering model [km]: Earth's Rayleigh / Mie coefficients scaled by the surface
// density, other bodies with their own dust / haze (the colour of their sky)
struct AtmP { Vector3 bR, bM, bMe; float HR, HM, g, Rkm, Akm, sunI; };
AtmP atmParams(const Body& b) {
    AtmP a{};
    double k = b.atmRho0 / 1.225;
    // a body under clouds (Venus, Titan) or a gas giant: what is seen from above is the top of the clouds / haze -
    // only the thin air above it scatters (not the 92 bar of Venus, not the hundreds of km down to Jupiter's 1 bar)
    bool cloudTop = b.gasGiant || b.surf == Surf::Cloudy;
    if (cloudTop) k = std::min(k, b.gasGiant ? 0.02 : 0.1); // (~0.1 bar at Venus' cloud tops, less above a gas giant's)
    a.Rkm = (float)(b.radius / 1000);
    a.Akm = (float)((b.radius + b.atmTop) / 1000);
    double H = std::max(b.atmH / 1000.0, 0.5);
    if (b.name == "Earth") {
        a.bR = {5.8e-3f, 13.5e-3f, 33.1e-3f};
        a.bM = {21e-3f, 21e-3f, 21e-3f};
        a.HR = 8.0f;
        a.HM = 1.2f;
        a.g = 0.76f;
        a.bMe = {a.bM.x * 1.1f, a.bM.y * 1.1f, a.bM.z * 1.1f};
    } else {
        a.bR = {(float)(5.8e-3 * k), (float)(13.5e-3 * k), (float)(33.1e-3 * k)};
        a.HR = (float)H;
        a.HM = (float)(H * 0.6);
        // aerosols: an optical depth that grows with the air (Mars' dust ~0.5, Titan's haze far more), tinted
        double tau = std::clamp(0.45 * std::sqrt(std::max(k, 1e-4)) * 6.0, 0.1, 8.0);
        if (cloudTop) tau = b.gasGiant ? 0.08 : 0.3; // (fairly clear above the cloud tops: the colours are the clouds')
        Vector3 c = b.atmColor;
        float m = std::max({c.x, c.y, c.z, 1e-3f});
        a.bM = {(float)(tau / a.HM * c.x / m), (float)(tau / a.HM * c.y / m), (float)(tau / a.HM * c.z / m)};
        // the dust / haze is coloured because it absorbs the other colours (Mars' dust takes the blue): the light
        // through it turns warm. Scattering alone in its own colour (as before) took the red out of the sunlight
        // and the ground came out blue
        auto ext = [&](float cc) { return (float)(tau / a.HM * (1.0 + 0.9 * (1.0 - cc / m))); };
        a.bMe = {ext(c.x), ext(c.y), ext(c.z)};
        a.g = 0.65f;
    }
    a.sunI = 24.0f;
    return a;
}
// transmittance table of a body's air: optical depths (Rayleigh, Mie) from radius r along the direction with
// cosine mu (to the local vertical) to the top of the atmosphere, 256 x 64 texels, 128 samples each (more
// than the 6 per point the shader used to take, so the light is also a little more exact). Built once per
// body (~0.2 MB each): the shaders look it up instead of integrating toward the sun at every sample.
std::map<std::string, Texture2D> g_transLut;
Texture2D transLutFor(const Body& b) {
    auto it = g_transLut.find(b.name);
    if (it != g_transLut.end()) return it->second;
    AtmP a = atmParams(b);
    const int NX = 256, NY = 64, NS = 128;
    double R = a.Rkm, A = a.Akm, H = std::sqrt(std::max(A * A - R * R, 1e-6));
    std::vector<float> px((size_t)NX * NY * 3, 0.f);
    for (int y = 0; y < NY; y++) {
        double xr = y / (double)(NY - 1), rho = xr * H, r = std::sqrt(rho * rho + R * R);
        for (int x = 0; x < NX; x++) {
            double xm = x / (double)(NX - 1);
            double dmin = A - r, dmax = rho + H, d = dmin + xm * (dmax - dmin);
            double mu = d <= 0 ? 1.0 : std::clamp((H * H - rho * rho - d * d) / (2.0 * r * d), -1.0, 1.0);
            // integrate along the ray from r to the top (samples spaced quadratically, dense near the start)
            double dR = 0, dM = 0;
            for (int j = 0; j < NS; j++) {
                double u0 = j / (double)NS, u1 = (j + 1) / (double)NS;
                double ta = d * u0 * u0, tb = d * u1 * u1, tm = 0.5 * (ta + tb);
                double h = std::sqrt(r * r + tm * tm + 2 * r * mu * tm) - R;
                dR += std::exp(-h / a.HR) * (tb - ta);
                dM += std::exp(-h / a.HM) * (tb - ta);
            }
            size_t k = ((size_t)y * NX + x) * 3;
            px[k] = (float)dR;
            px[k + 1] = (float)dM;
        }
    }
    Image im{px.data(), NX, NY, 1, PIXELFORMAT_UNCOMPRESSED_R32G32B32};
    Texture2D t = LoadTextureFromImage(im);
    SetTextureFilter(t, TEXTURE_FILTER_BILINEAR);
    SetTextureWrap(t, TEXTURE_WRAP_CLAMP);
    g_transLut[b.name] = t;
    return t;
}

// the atmosphere uniforms; `mat`: the material that carries the table (the sky has none: bound directly)
void setAtm(Shader& sh, const PlanetLocs& L, const Body& b, const Vec3d& camRel, Material* mat) {
    Texture2D lut = transLutFor(b);
    if (mat) mat->maps[MATERIAL_MAP_EMISSION].texture = lut;
    else if (L.transLut >= 0) SetShaderValueTexture(sh, L.transLut, lut);
    setI(sh, L.useLut, lut.id ? 1 : 0);
    AtmP a = atmParams(b);
    int on = b.atmTop > 0 ? 1 : 0;
    setI(sh, L.atmOn, on);
    setV3(sh, L.camC, camRel * 0.001);
    setF(sh, L.Rkm, a.Rkm);
    setF(sh, L.Akm, a.Akm);
    setV3(sh, L.betaR, a.bR);
    setV3(sh, L.betaM, a.bM);
    setV3(sh, L.betaMe, a.bMe);
    setF(sh, L.HR, a.HR);
    setF(sh, L.HM, a.HM);
    setF(sh, L.mieG, a.g);
    setF(sh, L.sunI, a.sunI);
    setF(sh, L.expo, 1.0f);
}

void setPlanetUniforms(const Body& b, const Mat3d& rot, const Vec3d& sunDir, int kind, int octaves, float lightScale) {
    setI(planetShader, PL.atmOn, 0); // (the cap turns the scattering on in the physically based mode)
    {
        static int dbg = std::getenv("SF_PBRDBG") ? std::atoi(std::getenv("SF_PBRDBG")) : 0;
        static int loc = -2;
        static unsigned sid = 0;
        if (loc == -2 || sid != planetShader.id) { loc = GetShaderLocation(planetShader, "dbgMode"); sid = planetShader.id; }
        if (loc >= 0) SetShaderValue(planetShader, loc, &dbg, SHADER_UNIFORM_INT);
    }
    setV3(planetShader, PL.sunDir, sunDir);
    SetShaderValueMatrix(planetShader, PL.bodyRot, toMatrix(rot));
    setV3(planetShader, PL.colA, b.colA);
    setV3(planetShader, PL.colB, b.colB);
    setV3(planetShader, PL.colC, b.colC);
    setV3(planetShader, PL.atmColor, b.atmColor);
    float atm = b.atmTop > 0 ? (b.gasGiant ? 0.35f : (float)std::clamp(b.atmP0 / 101325.0, 0.25, 1.0)) : 0.f;
    setF(planetShader, PL.atmStrength, atm);
    setI(planetShader, PL.kind, kind);
    setF(planetShader, PL.seed, b.seed);
    setI(planetShader, PL.octaves, std::min(octaves, g_maxPlanetOctaves));
    setI(planetShader, PL.useBaked, 0);
    setI(planetShader, PL.bakeMode, 0);
    setF(planetShader, PL.fogK, 0.f);
    setF(planetShader, PL.lightScale, lightScale);
    setF(planetShader, PL.nightLift, g_nightLift);
    setI(planetShader, PL.detailOct, 0);
    setF(planetShader, PL.cloudLow, 1.f);
    setF(planetShader, PL.rimK, 1.f);
    setI(planetShader, PL.hasMap, 0);
    setI(planetShader, PL.hasRelief, 0);
    setF(planetShader, PL.dispK, 0.f);
    setI(planetShader, PL.waveOn, 0);
}

// ---- the sea's waves near the camera (planet shader seaSurface): twelve trains from 160 m down to 0.5 m
// wavelength spread around one wind direction, as fixed wave vectors in the tangent plane of a 3 degree cell
// (the pattern stays put on the water while the camera moves; it is laid out afresh in the next cell), with
// their phase at the camera worked out here in double precision (the shader adds k . offset from the camera).
// Deep-water dispersion: omega = sqrt(g k).
double g_seaT = 0;
void seaWaves(const Vec3d& camBF, double R, double t, float out[48]) {
    Vec3d u = norm(camBF);
    const double cs = 3 * DEG;
    double lat = std::asin(std::clamp(u.z, -1.0, 1.0)), lon = std::atan2(u.y, u.x);
    double latc = (std::floor(lat / cs) + 0.5) * cs, lonc = (std::floor(lon / cs) + 0.5) * cs;
    Vec3d c{std::cos(latc) * std::cos(lonc), std::cos(latc) * std::sin(lonc), std::sin(latc)};
    Vec3d east{-std::sin(lonc), std::cos(lonc), 0}, north = cross(c, east);
    Vec3d O = c * R;
    static const double spread[12] = {0.0, 0.14, -0.11, 0.8, -0.7, 0.25, -1.05, 1.0, -0.55, 1.3, -1.35, 0.65};
    const double wind = 0.6; // (direction of travel, from east toward north)
    for (int i = 0; i < 12; i++) {
        double lam = 160.0 * std::pow(0.5 / 160.0, i / 11.0) * (1.0 + 0.07 * std::sin(i * 2.3)), k = TAU / lam;
        // the three longest are mostly swell from another direction
        double a = (i < 3 ? wind + 0.9 : wind) + spread[i] * (0.5 + 0.5 * i / 11.0);
        Vec3d kv = (east * std::cos(a) + north * std::sin(a)) * k;
        double ph = dot(kv, camBF - O) - std::sqrt(9.81 * k) * t;
        ph -= std::floor(ph / TAU) * TAU;
        out[i * 4] = (float)kv.x;
        out[i * 4 + 1] = (float)kv.y;
        out[i * 4 + 2] = (float)kv.z;
        out[i * 4 + 3] = (float)ph;
    }
}

// ---- the high-resolution tile window (terrain::Detail): two toroidal textures of 4 x 4 tiles for the
// body the active vessel is near; tiles are uploaded as the worker delivers them (a few per frame)
struct DetTex {
    int body = -1;
    Texture2D color{}, height{};
    int slotC[16], slotH[16];
};
DetTex detTex;
void dropDetTex() {
    if (detTex.color.id) UnloadTexture(detTex.color);
    if (detTex.height.id) UnloadTexture(detTex.height);
    detTex = DetTex{};
}
// returns the window when it can be drawn, and its masks of the slots ready for it
const terrain::Detail* serviceDetail(int body, int& maskC, int& maskH) {
    maskC = maskH = 0;
    const terrain::Detail* d = terrain::detail(body);
    if (!d) {
        if (detTex.body >= 0 && !terrain::detail(detTex.body)) dropDetTex(); // (the window is gone: free the memory)
        return nullptr;
    }
    if (detTex.body != body) {
        dropDetTex();
        detTex.body = body;
        for (int i = 0; i < 16; i++) detTex.slotC[i] = detTex.slotH[i] = -1;
        if (d->ctw > 0) {
            Image im = GenImageColor(4 * d->ctw, 4 * d->cth, BLACK);
            ImageFormat(&im, PIXELFORMAT_UNCOMPRESSED_R8G8B8);
            detTex.color = LoadTextureFromImage(im);
            UnloadImage(im);
            GenTextureMipmaps(&detTex.color);
            SetTextureFilter(detTex.color, TEXTURE_FILTER_TRILINEAR);
            SetTextureWrap(detTex.color, TEXTURE_WRAP_REPEAT);
        }
        if (d->htw > 0) {
            std::vector<float> zero((size_t)16 * d->htw * d->hth, 0.f);
            Image hi{};
            hi.data = zero.data();
            hi.width = 4 * d->htw;
            hi.height = 4 * d->hth;
            hi.mipmaps = 1;
            hi.format = PIXELFORMAT_UNCOMPRESSED_R32;
            detTex.height = LoadTextureFromImage(hi);
            GenTextureMipmaps(&detTex.height);
            SetTextureFilter(detTex.height, TEXTURE_FILTER_TRILINEAR);
            SetTextureWrap(detTex.height, TEXTURE_WRAP_REPEAT);
        }
        terrain::resendTiles(body);
    }
    int tx, ty;
    bool mipC = false, mipH = false;
    std::vector<unsigned char> rgb;
    for (int n = 0; n < 6 && detTex.color.id && terrain::takeColorTile(body, tx, ty, rgb); n++) {
        UpdateTextureRec(detTex.color, Rectangle{(float)((tx % 4) * d->ctw), (float)((ty % 4) * d->cth), (float)d->ctw, (float)d->cth}, rgb.data());
        detTex.slotC[(ty % 4) * 4 + tx % 4] = ty * terrain::DET_GW + tx;
        mipC = true;
    }
    const float* h;
    while (detTex.height.id && terrain::takeHeightTile(body, tx, ty, h)) {
        UpdateTextureRec(detTex.height, Rectangle{(float)((tx % 4) * d->htw), (float)((ty % 4) * d->hth), (float)d->htw, (float)d->hth}, h);
        detTex.slotH[(ty % 4) * 4 + tx % 4] = ty * terrain::DET_GW + tx;
        mipH = true;
    }
    if (mipC) GenTextureMipmaps(&detTex.color);
    if (mipH) GenTextureMipmaps(&detTex.height);
    for (int j = 0; j < 4; j++)
        for (int i = 0; i < 4; i++) {
            int ktx = (d->tx0 + i) % terrain::DET_GW, kty = d->ty0 + j;
            int key = kty * terrain::DET_GW + ktx, sl = (kty % 4) * 4 + ktx % 4;
            if (detTex.slotC[sl] == key) maskC |= 1 << sl;
            if (detTex.slotH[sl] == key) maskH |= 1 << sl;
        }
    return d;
}

// ---- real surface maps (terrain.*): GPU textures created on first use
struct MapTex {
    Texture2D color{}, height{};
};
std::map<int, MapTex> mapTex;
// bind the surface map of a body (colour + elevation); `displace`: the far sphere is raised by the
// true elevation in the vertex shader (the near-ground cap is displaced on the CPU instead)
bool bindMap(int body, const Body& b, bool displace, bool generate) {
    // (a body with a real colour map gets its relief made when it is first drawn: the face is what counts from afar)
    const terrain::Map* m = generate || terrain::realColor(body) ? terrain::ensure(body) : terrain::map(body);
    if (!m) return false;
    MapTex& t = mapTex[body];
    if (!t.height.id) {
        if (!m->colorFile.empty()) {
            Image ci = LoadImageFromMemory(".jpg", m->colorFile.data(), (int)m->colorFile.size());
            if (ci.data && b.name == "Earth") {
                // the launch complexes stand on land: at 20 km per pixel the coastal sites (Tanegashima,
                // Kourou) fall into the sea, so a patch of coastal land is painted under each of them
                ImageFormat(&ci, PIXELFORMAT_UNCOMPRESSED_R8G8B8);
                terrain::paintLaunchSites((unsigned char*)ci.data, ci.width, ci.height, 0, 0, ci.width, ci.height);
            }
            if (ci.data) {
                t.color = LoadTextureFromImage(ci);
                UnloadImage(ci);
                GenTextureMipmaps(&t.color);
                SetTextureFilter(t.color, TEXTURE_FILTER_TRILINEAR);
                SetTextureWrap(t.color, TEXTURE_WRAP_REPEAT);
            }
        }
        Image hi{};
        hi.data = (void*)m->height.data();
        hi.width = m->w;
        hi.height = m->h;
        hi.mipmaps = 1;
        hi.format = PIXELFORMAT_UNCOMPRESSED_R32;
        t.height = LoadTextureFromImage(hi); // uploads only; the CPU copy stays with the physics
        SetTextureFilter(t.height, TEXTURE_FILTER_BILINEAR);
        SetTextureWrap(t.height, TEXTURE_WRAP_REPEAT);
    }
    planetMat.maps[MATERIAL_MAP_METALNESS].texture = t.color.id ? t.color : t.height; // (unused without a colour map)
    planetMat.maps[MATERIAL_MAP_NORMAL].texture = t.height;
    setI(planetShader, PL.hasMap, t.color.id ? 1 : 0);
    setI(planetShader, PL.hasRelief, 1);
    Vector2 tx{1.0f / m->w, 1.0f / m->h};
    SetShaderValue(planetShader, PL.mapTexel, &tx, SHADER_UNIFORM_VEC2);
    setF(planetShader, PL.bodyR, (float)b.radius);
    // gentle exaggeration: the relief reads from orbit as in photos at low sun. Over a spacecraft mosaic the generated
    // craters are only a hint (the photo already shows the real ones)
    setF(planetShader, PL.reliefK, m->generated ? (m->colorFile.empty() ? 1.5f : 0.45f) : 2.0f);
    setF(planetShader, PL.dispK, displace ? (float)(1.0 / b.radius) : 0.f);
    int maskC = 0, maskH = 0;
    const terrain::Detail* d = serviceDetail(body, maskC, maskH);
    setI(planetShader, PL.detOn, d && (maskC || maskH) ? 1 : 0);
    if (d && (maskC || maskH)) {
        planetMat.maps[MATERIAL_MAP_ROUGHNESS].texture = detTex.color.id ? detTex.color : t.height;
        planetMat.maps[MATERIAL_MAP_OCCLUSION].texture = detTex.height.id ? detTex.height : t.height;
        Vector2 o{(float)d->tx0, (float)d->ty0};
        SetShaderValue(planetShader, PL.detOrigin, &o, SHADER_UNIFORM_VEC2);
        setI(planetShader, PL.detMaskC, maskC);
        setI(planetShader, PL.detMaskH, maskH);
        Vector2 pol{d->ty0 == 0 ? 1.f : 0.f, d->ty0 + 4 == terrain::DET_GH ? 1.f : 0.f};
        SetShaderValue(planetShader, PL.detPolar, &pol, SHADER_UNIFORM_VEC2);
        Vector2 th{d->htw ? 1.0f / (4 * d->htw) : 1.f, d->hth ? 1.0f / (4 * d->hth) : 1.f};
        SetShaderValue(planetShader, PL.detTexelH, &th, SHADER_UNIFORM_VEC2);
    } else {
        planetMat.maps[MATERIAL_MAP_ROUGHNESS].texture = t.height; // (valid textures on the units; not sampled)
        planetMat.maps[MATERIAL_MAP_OCCLUSION].texture = t.height;
    }
    return true;
}
void unbindMap() {
    planetMat.maps[MATERIAL_MAP_METALNESS].texture = Texture2D{};
    planetMat.maps[MATERIAL_MAP_NORMAL].texture = Texture2D{};
    planetMat.maps[MATERIAL_MAP_ROUGHNESS].texture = Texture2D{};
    planetMat.maps[MATERIAL_MAP_OCCLUSION].texture = Texture2D{};
    setI(planetShader, PL.detOn, 0);
    setI(planetShader, PL.hasMap, 0);
    setI(planetShader, PL.hasRelief, 0);
    setF(planetShader, PL.dispK, 0.f);
}

// use the body's baked field texture if it exists, otherwise ask for it (the surface is then fully
// procedural for a frame or two). Stars are not baked: their granulation is far too fine.
bool bindBaked(int body, const Body& b) {
    if (b.surf == Surf::Star || bakeFailed) return false;
    auto it = bakedFields.find(body);
    if (it == bakedFields.end()) {
        bool queued = false;
        for (auto& r : bakeQueue) queued |= r.body == body;
        if (!queued) bakeQueue.push_back({body, (int)b.surf, (float)b.seed});
        return false;
    }
    it->second.lastUse = GetTime();
    planetMat.maps[MATERIAL_MAP_DIFFUSE].texture = it->second.tex;
    setI(planetShader, PL.useBaked, 1);
    return true;
}
void unbindBaked() {
    planetMat.maps[MATERIAL_MAP_DIFFUSE].texture = whiteTex;
    setI(planetShader, PL.useBaked, 0);
}
void freeBaked(BakedField& f) {
    if (f.tex.id) rlUnloadTexture(f.tex.id);
    if (f.fbo) rlUnloadFramebuffer(f.fbo);
    f = BakedField{};
}
void evictBaked(size_t keep) {
    while (bakedFields.size() > keep) {
        auto oldest = bakedFields.begin();
        for (auto it = bakedFields.begin(); it != bakedFields.end(); ++it)
            if (it->second.lastUse < oldest->second.lastUse) oldest = it;
        freeBaked(oldest->second);
        bakedFields.erase(oldest);
    }
}

} // namespace

void serviceBakes() {
    if (bakeQueue.empty()) return;
    BakeRequest r = bakeQueue.front();
    bakeQueue.erase(bakeQueue.begin());
    if (bakedFields.count(r.body)) return;
    evictBaked(BAKE_MAX - 1);
    BakedField f;
    f.tex.id = rlLoadTexture(nullptr, BAKE_W, BAKE_H, PIXELFORMAT_UNCOMPRESSED_R16G16B16A16, 1);
    f.tex.width = BAKE_W;
    f.tex.height = BAKE_H;
    f.tex.mipmaps = 1;
    f.tex.format = PIXELFORMAT_UNCOMPRESSED_R16G16B16A16;
    if (!f.tex.id) return;
    f.fbo = rlLoadFramebuffer();
    rlFramebufferAttach(f.fbo, f.tex.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
    if (!rlFramebufferComplete(f.fbo)) {
        TraceLog(LOG_WARNING, "PLANET: baked field framebuffer incomplete, staying procedural");
        freeBaked(f);
        bakeFailed = true; // the GPU cannot render to half-float textures: stay fully procedural
        return;
    }
    RenderTexture2D rt{};
    rt.id = f.fbo;
    rt.texture = f.tex;
    BeginTextureMode(rt);
    rlDisableColorBlend();
    BeginShaderMode(planetShader);
    setI(planetShader, PL.bakeMode, 1);
    setI(planetShader, PL.useBaked, 0);
    setI(planetShader, PL.kind, r.kind);
    setF(planetShader, PL.seed, r.seed);
    Vector2 sz{(float)BAKE_W, (float)BAKE_H};
    SetShaderValue(planetShader, PL.bakeSize, &sz, SHADER_UNIFORM_VEC2);
    DrawRectangle(0, 0, BAKE_W, BAKE_H, WHITE);
    EndShaderMode();
    setI(planetShader, PL.bakeMode, 0);
    rlEnableColorBlend();
    EndTextureMode();
    int mips = 1;
    rlGenTextureMipmaps(f.tex.id, BAKE_W, BAKE_H, f.tex.format, &mips);
    f.tex.mipmaps = mips;
    rlTextureParameters(f.tex.id, RL_TEXTURE_MIN_FILTER, RL_TEXTURE_FILTER_MIP_LINEAR);
    rlTextureParameters(f.tex.id, RL_TEXTURE_MAG_FILTER, RL_TEXTURE_FILTER_LINEAR);
    rlTextureParameters(f.tex.id, RL_TEXTURE_WRAP_S, RL_TEXTURE_WRAP_REPEAT);
    rlTextureParameters(f.tex.id, RL_TEXTURE_WRAP_T, RL_TEXTURE_WRAP_CLAMP);
    f.lastUse = GetTime();
    bakedFields[r.body] = f;
}

void setPlanetDetail(int maxOctaves) { g_maxPlanetOctaves = std::clamp(maxOctaves, 6, 16); }
int planetDetail() { return g_maxPlanetOctaves; }

void setBakingEnabled(bool on) {
    bakeFailed = !on;
    if (!on) evictBaked(0);
}

size_t bakedFieldBytes() { return bakedFields.size() * (size_t)BAKE_W * BAKE_H * 8 * 4 / 3; }

Texture2D white() { return whiteTex; }

static float g_uiScale = 1.0f;
void setNightLift(float k) { g_nightLift = k; }
void setUiScale(float s) { g_uiScale = s; }
float uiScale() { return g_uiScale; }
Vector2 project(Vector3 p, Camera3D cam) {
    Vector2 v = GetWorldToScreen(p, cam); // physical pixels
    return {v.x / g_uiScale, v.y / g_uiScale};
}
Camera3D current() { return curCam; }

// ---- shader programs: the standard ones and the physically based variants (the same sources compiled with
// PBR defined), each with its own uniform locations; switched as a whole
void planetLocs(Shader& sh, PlanetLocs& L) {
    L.sunDir = GetShaderLocation(sh, "sunDir");
    L.bodyRot = GetShaderLocation(sh, "bodyRot");
    L.colA = GetShaderLocation(sh, "colA");
    L.colB = GetShaderLocation(sh, "colB");
    L.colC = GetShaderLocation(sh, "colC");
    L.atmColor = GetShaderLocation(sh, "atmColor");
    L.atmStrength = GetShaderLocation(sh, "atmStrength");
    L.kind = GetShaderLocation(sh, "kind");
    L.seed = GetShaderLocation(sh, "seed");
    L.octaves = GetShaderLocation(sh, "octaves");
    L.center = GetShaderLocation(sh, "center");
    L.radius = GetShaderLocation(sh, "radius");
    L.haloRadius = GetShaderLocation(sh, "haloRadius");
    L.fogColor = GetShaderLocation(sh, "fogColor");
    L.fogK = GetShaderLocation(sh, "fogK");
    L.lightScale = GetShaderLocation(sh, "lightScale");
    L.nightLift = GetShaderLocation(sh, "nightLift");
    L.detailOct = GetShaderLocation(sh, "detailOct");
    L.detailOfs = GetShaderLocation(sh, "detailOfs");
    L.cloudLow = GetShaderLocation(sh, "cloudLow");
    L.rimK = GetShaderLocation(sh, "rimK");
    L.hasMap = GetShaderLocation(sh, "hasMap");
    L.hasRelief = GetShaderLocation(sh, "hasRelief");
    L.mapTexel = GetShaderLocation(sh, "mapTexel");
    L.bodyR = GetShaderLocation(sh, "bodyR");
    L.reliefK = GetShaderLocation(sh, "reliefK");
    L.dispK = GetShaderLocation(sh, "dispK");
    L.detOn = GetShaderLocation(sh, "detOn");
    L.geoN = GetShaderLocation(sh, "geoN");
    L.geoFar = GetShaderLocation(sh, "geoFar");
    L.waveOn = GetShaderLocation(sh, "waveOn");
    L.waves = GetShaderLocation(sh, "waves");
    L.waveN = GetShaderLocation(sh, "waveN");
    L.detOrigin = GetShaderLocation(sh, "detOrigin");
    L.detMaskC = GetShaderLocation(sh, "detMaskC");
    L.detMaskH = GetShaderLocation(sh, "detMaskH");
    L.detPolar = GetShaderLocation(sh, "detPolar");
    L.detTexelH = GetShaderLocation(sh, "detTexelH");
    sh.locs[SHADER_LOC_MAP_ROUGHNESS] = GetShaderLocation(sh, "detColor");
    sh.locs[SHADER_LOC_MAP_OCCLUSION] = GetShaderLocation(sh, "detHeight");
    // surface maps on texture units 1 and 2 (raylib binds material map i to the sampler at locs[MAP_DIFFUSE + i])
    sh.locs[SHADER_LOC_MAP_METALNESS] = GetShaderLocation(sh, "mapColor");
    sh.locs[SHADER_LOC_MAP_NORMAL] = GetShaderLocation(sh, "mapHeight");
    L.useBaked = GetShaderLocation(sh, "useBaked");
    L.bakeMode = GetShaderLocation(sh, "bakeMode");
    L.bakeSize = GetShaderLocation(sh, "bakeSize");
    sh.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(sh, "matModel");
    L.atmOn = GetShaderLocation(sh, "atmOn");
    L.camC = GetShaderLocation(sh, "camC");
    L.Rkm = GetShaderLocation(sh, "Rkm");
    L.Akm = GetShaderLocation(sh, "Akm");
    L.betaR = GetShaderLocation(sh, "betaR");
    L.betaM = GetShaderLocation(sh, "betaM");
    L.betaMe = GetShaderLocation(sh, "betaMe");
    L.HR = GetShaderLocation(sh, "HR");
    L.HM = GetShaderLocation(sh, "HM");
    L.mieG = GetShaderLocation(sh, "mieG");
    L.sunI = GetShaderLocation(sh, "sunI");
    L.expo = GetShaderLocation(sh, "expo");
    L.transLut = GetShaderLocation(sh, "transLut");
    L.useLut = GetShaderLocation(sh, "useLut");
    sh.locs[SHADER_LOC_MAP_EMISSION] = L.transLut; // (the planet material's emission slot carries the table)
}
struct ShipLocs {
    int light = -1, lightI = -1, ambient = -1, emissive = -1, camPos = -1, finish = -1, objScale = -1, nSpots = -1, spotPos = -1, spotDir = -1,
        spotCol = -1, spotRange = -1, uv = -1, envUp = -1, envSky = -1, envGround = -1, sunTint = -1, model = -1, normal = -1;
    int nWin = -1, winM[8] = {-1, -1, -1, -1, -1, -1, -1, -1}, winA = -1, winC = -1, cutOn = -1, cutLo = -1, cutHi = -1, craftUp = -1, shapeK = -1;
    int texM = -1;
};
ShipLocs shipLocsOf(Shader& sh) {
    ShipLocs L;
    L.model = GetShaderLocation(sh, "matModel");
    L.normal = GetShaderLocation(sh, "matNormal");
    sh.locs[SHADER_LOC_MATRIX_MODEL] = L.model;
    sh.locs[SHADER_LOC_MATRIX_NORMAL] = L.normal;
    L.light = GetShaderLocation(sh, "lightDir");
    L.lightI = GetShaderLocation(sh, "lightI");
    L.ambient = GetShaderLocation(sh, "ambient");
    L.emissive = GetShaderLocation(sh, "emissive");
    L.camPos = GetShaderLocation(sh, "camPos");
    L.finish = GetShaderLocation(sh, "finish");
    L.objScale = GetShaderLocation(sh, "objScale");
    L.nSpots = GetShaderLocation(sh, "nSpots");
    L.spotPos = GetShaderLocation(sh, "spotPos[0]");
    L.spotDir = GetShaderLocation(sh, "spotDir[0]");
    L.spotCol = GetShaderLocation(sh, "spotCol");
    L.spotRange = GetShaderLocation(sh, "spotRange");
    L.uv = GetShaderLocation(sh, "uvScale");
    L.envUp = GetShaderLocation(sh, "envUp");
    L.envSky = GetShaderLocation(sh, "envSky");
    L.envGround = GetShaderLocation(sh, "envGround");
    L.sunTint = GetShaderLocation(sh, "sunTint");
    L.nWin = GetShaderLocation(sh, "nWin");
    for (int i = 0; i < 8; i++) L.winM[i] = GetShaderLocation(sh, TextFormat("winM[%d]", i));
    L.winA = GetShaderLocation(sh, "winA[0]");
    L.winC = GetShaderLocation(sh, "winC[0]");
    L.cutOn = GetShaderLocation(sh, "cutOn");
    L.cutLo = GetShaderLocation(sh, "cutLo");
    L.cutHi = GetShaderLocation(sh, "cutHi");
    L.craftUp = GetShaderLocation(sh, "craftUp");
    L.shapeK = GetShaderLocation(sh, "shapeK");
    L.texM = GetShaderLocation(sh, "texM");
    sh.locs[SHADER_LOC_MAP_DIFFUSE] = GetShaderLocation(sh, "texture0");
    return L;
}
int shipSunTint = -1, shipTexM = -1;
void applyShipLocs(const ShipLocs& L) {
    shipTexM = L.texM;
    shipLight = L.light; shipLightI = L.lightI; shipAmbient = L.ambient; shipEmissive = L.emissive; shipCamPos = L.camPos;
    shipFinish = L.finish; shipObjScale = L.objScale; shipNSpots = L.nSpots; shipSpotPos = L.spotPos; shipSpotDir = L.spotDir;
    shipSpotCol = L.spotCol; shipSpotRange = L.spotRange; shipUV = L.uv; shipEnvUp = L.envUp; shipEnvSky = L.envSky;
    shipEnvGround = L.envGround; shipSunTint = L.sunTint;
    shipNWin = L.nWin; shipWinA = L.winA; shipWinC = L.winC; shipCutOn = L.cutOn; shipCutLo = L.cutLo; shipCutHi = L.cutHi;
    shipCraftUp = L.craftUp; shipShapeK = L.shapeK;
    for (int i = 0; i < 8; i++) shipWinM[i] = L.winM[i];
}
Shader shipStd{}, shipPbr{}, planetStd{}, planetPbr{}, skyShader{};
ShipLocs shipLocStd, shipLocPbr;
PlanetLocs PLstd, PLpbr;
bool g_pbrLoaded = false, g_pbrFailed = false, g_pbr = false;
int skyRes = -1, skyFwd = -1, skyRight = -1, skyUp = -1, skyTanY = -1, skyAspect = -1, skySun = -1;
PlanetLocs SKL; // (the atmosphere uniforms of the sky program)
// Realistic shadows (physically based mode): a depth map seen from the sun (orthographic) and percentage-closer
// soft shadows - the blockers near a point are found first; the penumbra is as wide as the sun's disc makes it
// over the distance from them to the point (sharp where a part stands on the ground, soft far from it), and
// that much of the map is filtered (a golden-angle spiral, turned per pixel).
const char* SHADOW_GLSL = R"(
uniform int shadowOn;
uniform sampler2D shadowMap;
uniform mat4 shadowVP;
uniform vec4 shadowA;   // x: 1 / resolution, y: depth range [m], z: map uv per metre, w: one texel [m]
uniform float sunTan;   // the sun's angular diameter (tangent)
uniform int shadowN;    // filter samples
float shHash(vec2 p) { return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453); }
vec2 shSpiral(int i, int n, float rot) {
    float r = sqrt((float(i) + 0.5) / float(n));
    float a = float(i) * 2.3999632 + rot;
    return vec2(cos(a), sin(a)) * r;
}
// 1: in the sun, 0: in the shadow of a part. P: camera-relative position [m], N: surface normal, L: toward the sun
float shadowVis(vec3 P, vec3 N, vec3 L) {
    if (shadowOn == 0) return 1.0;
    float NL = clamp(dot(N, L), 0.0, 1.0);
    // off the surface along its normal by a texel and a half (more on faces turned from the sun): no self-shadow acne
    vec3 Po = P + N * shadowA.w * (1.5 + 2.5 * (1.0 - NL));
    vec4 q = shadowVP * vec4(Po, 1.0);
    vec3 s = q.xyz / q.w * 0.5 + 0.5;
    if (s.x <= 0.0 || s.y <= 0.0 || s.x >= 1.0 || s.y >= 1.0 || s.z >= 1.0 || s.z <= 0.0) return 1.0;
    float zr = s.z - shadowA.w / shadowA.y;
    float rot = shHash(gl_FragCoord.xy) * 6.2831853;
    // blockers within the widest penumbra cared for (60 m from the point)
    float searchUV = max(2.0 * shadowA.x, 60.0 * sunTan * shadowA.z);
    float sum = 0.0, nb = 0.0;
    for (int i = 0; i < 16; i++) {
        float d = texture(shadowMap, s.xy + shSpiral(i, 16, rot) * searchUV).r;
        if (d < zr) { sum += d; nb += 1.0; }
    }
    if (nb < 0.5) return 1.0;
    float dist = (zr - sum / nb) * shadowA.y; // from the blockers to the point [m]
    float filt = clamp(dist * sunTan * shadowA.z, 1.2 * shadowA.x, searchUV);
    float lit = 0.0;
    for (int i = 0; i < 64; i++) {
        if (i >= shadowN) break;
        float d = texture(shadowMap, s.xy + shSpiral(i, shadowN, rot + 1.0) * filt).r;
        lit += d < zr ? 0.0 : 1.0;
    }
    float v = lit / float(shadowN);
    vec2 e = min(s.xy, 1.0 - s.xy);
    return mix(1.0, v, smoothstep(0.0, 0.04, min(e.x, e.y))); // (faded out at the edge of the map)
}
)";

std::string withPbr(const char* src, bool fragment = false) {
    std::string s = src;
    size_t e = s.find('\n');
    return s.substr(0, e + 1) + "#define PBR 1\n" + std::string(ATMO_GLSL) + (fragment ? SHADOW_GLSL : "") + s.substr(e + 1);
}
bool shaderOk(const Shader& sh) { return sh.id != 0 && sh.id != rlGetShaderIdDefault() && sh.locs != nullptr; }

void init() {
    planetShader = LoadShaderFromMemory(PLANET_VS, PLANET_FS);
    planetLocs(planetShader, PL);
    planetStd = planetShader;
    PLstd = PL;

    shipShader = LoadShaderFromMemory(SHIP_VS, SHIP_FS);
    shipStd = shipShader;
    shipLocStd = shipLocsOf(shipShader);
    applyShipLocs(shipLocStd);
    setI(shipShader, shipNSpots, 0);
    resetEnvironment();

    plasmaShader = LoadShaderFromMemory(PLASMA_VS, PLASMA_FS);
    plasmaShader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(plasmaShader, "matModel");
    plasmaShader.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(plasmaShader, "matNormal");
    plasmaThick = GetShaderLocation(plasmaShader, "thick");
    plasmaFlow = GetShaderLocation(plasmaShader, "flowDir");
    plasmaCam = GetShaderLocation(plasmaShader, "camPos");
    plasmaHot = GetShaderLocation(plasmaShader, "hot");
    plasmaInten = GetShaderLocation(plasmaShader, "inten");
    plasmaTime = GetShaderLocation(plasmaShader, "time");
    plasmaMat = LoadMaterialDefault();
    plasmaMat.shader = plasmaShader;
    plumeShader = LoadShaderFromMemory(PLUME_VS, PLUME_FS);
    plumeShader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(plumeShader, "matModel");
    plumeShader.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(plumeShader, "matNormal");
    plumeCamPos = GetShaderLocation(plumeShader, "camPos");
    plumeLimb = GetShaderLocation(plumeShader, "limb");
    setF(plumeShader, plumeLimb, 0.f);

    navShader = LoadShaderFromMemory(nullptr, NAV_FS);
    navOrient = GetShaderLocation(navShader, "orient");

    planetMat = LoadMaterialDefault();
    planetMat.shader = planetShader;
    shipMat = LoadMaterialDefault();
    shipMat.shader = shipShader;
    plumeMat = LoadMaterialDefault();
    plumeMat.shader = plumeShader;
    starMat = LoadMaterialDefault();

    sphereMesh = genSphere(96, 128);
    sphereLo = genSphere(24, 32);
    cubeMesh = GenMeshCube(1, 1, 1);
    plumeMesh = genPlume(32);
    wakeMesh = genWake(32, 12, 0.18f);
    sheathMesh = genWake(40, 12, 0.86f); // plasma sheath leaving the heat-shield rim
    bellOuter = genBell(false, 40, 18);
    bellInner = genBell(true, 40, 18);
    canopyMesh[0] = genCanopy(0);
    canopyMesh[1] = genCanopy(1);
    wedgeMesh = genWedge();
    buildCapMesh();
    buildStarfield();

    Image wi = GenImageColor(4, 4, WHITE);
    whiteTex = LoadTextureFromImage(wi);
    UnloadImage(wi);
    {
        // soft rounded rectangle for ground shadows (alpha mask)
        Image si = GenImageColor(64, 64, BLANK);
        Color* px = (Color*)si.data;
        for (int y = 0; y < 64; y++)
            for (int x = 0; x < 64; x++) {
                auto edge = [](float t) { // 0 at the border .. 1 inside, smooth over the outer 40 %
                    float d = 1.0f - std::fabs(t * 2.0f - 1.0f);
                    float k = std::clamp(d / 0.4f, 0.0f, 1.0f);
                    return k * k * (3 - 2 * k);
                };
                float m = edge((x + 0.5f) / 64.0f) * edge((y + 0.5f) / 64.0f);
                px[y * 64 + x] = Color{255, 255, 255, (unsigned char)(m * 255)};
            }
        shadowTex = LoadTextureFromImage(si);
        UnloadImage(si);
        SetTextureFilter(shadowTex, TEXTURE_FILTER_BILINEAR);
    }
    Image gi = GenImageGradientRadial(128, 128, 0.0f, WHITE, BLANK);
    glowTex = LoadTextureFromImage(gi);
    UnloadImage(gi);
    SetTextureFilter(glowTex, TEXTURE_FILTER_BILINEAR);
    starMat.maps[MATERIAL_MAP_DIFFUSE].texture = glowTex;
}

namespace {
void trimExtraCaches();
void shutdownExtra();
} // namespace

void trimCaches(bool hard) {
    // baked planet fields are re-baked on demand (~one frame each)
    evictBaked(hard ? 0 : 3);
    // frustum meshes are regenerated on demand; only drop them when memory is really short
    if (!hard) return;
    for (auto& kv : frustumCache) UnloadMesh(kv.second);
    frustumCache.clear();
    for (auto& kv : halfCache) {
        UnloadMesh(kv.second.first);
        UnloadMesh(kv.second.second);
    }
    halfCache.clear();
    trimExtraCaches();
}

void shutdown() {
    evictBaked(0);
    for (auto& kv : mapTex) {
        if (kv.second.color.id) UnloadTexture(kv.second.color);
        if (kv.second.height.id) UnloadTexture(kv.second.height);
    }
    mapTex.clear();
    dropDetTex();
    for (auto& kv : frustumCache) UnloadMesh(kv.second);
    frustumCache.clear();
    for (auto& kv : halfCache) {
        UnloadMesh(kv.second.first);
        UnloadMesh(kv.second.second);
    }
    halfCache.clear();
    for (auto& kv : ringCache) UnloadMesh(kv.second);
    ringCache.clear();
    for (auto& kv : g_transLut)
        if (kv.second.id) UnloadTexture(kv.second);
    g_transLut.clear();
    UnloadMesh(sphereMesh);
    UnloadMesh(sphereLo);
    UnloadMesh(cubeMesh);
    UnloadMesh(capMesh);
    UnloadMesh(starMesh);
    UnloadMesh(plumeMesh);
    UnloadMesh(wakeMesh);
    UnloadMesh(sheathMesh);
    UnloadMesh(bellOuter);
    UnloadMesh(bellInner);
    UnloadMesh(canopyMesh[0]);
    UnloadMesh(canopyMesh[1]);
    UnloadMesh(wedgeMesh);
    auto unloadSh = [](Shader& sh) {
        if (sh.id && sh.id != rlGetShaderIdDefault()) UnloadShader(sh);
        sh = {};
    };
    unloadSh(planetStd);
    unloadSh(shipStd);
    unloadSh(planetPbr);
    unloadSh(shipPbr);
    unloadSh(skyShader);
    unloadSh(plumeShader);
    unloadSh(plasmaShader);
    unloadSh(navShader);
    unloadSh(mwShader);
    planetShader = {};
    shipShader = {};
    if (mwTex.id) { UnloadTexture(mwTex); mwTex = {}; }
    UnloadTexture(whiteTex);
    UnloadTexture(glowTex);
    if (shadowTex.id) UnloadTexture(shadowTex);
    shutdownExtra();
}

void begin3D(const Cam& c, double nearP, double farP) {
    rlDrawRenderBatchActive();
    rlSetClipPlanes(nearP, farP);
    Camera3D cam{};
    cam.position = toF(c.pos);
    cam.target = toF(c.pos + c.fwd);
    cam.up = toF(c.up);
    cam.fovy = c.fovy;
    cam.projection = CAMERA_PERSPECTIVE;
    curCam = cam;
    BeginMode3D(cam);
}

void end3D() {
    EndMode3D();                          // resets the modelview matrix
    rlScalef(g_uiScale, g_uiScale, 1.0f); // restore the 2D UI scale
}

void clearDepth() {
    rlDrawRenderBatchActive();
    glClear(0x00000100); // GL_DEPTH_BUFFER_BIT
}

void drawStarfield(float brightness) {
    if (brightness <= 0.01f) return;
    rlDisableDepthTest();
    rlDisableDepthMask();
    rlDisableBackfaceCulling();
    BeginBlendMode(BLEND_ADDITIVE);
    Vector3 p = curCam.position;
    if (mwReady) {
        // the Milky Way's glow behind the stars (as seen under a dark sky: faint, it fades out first in daylight)
        float k = std::clamp(brightness, 0.f, 1.f);
        unsigned char g = (unsigned char)(255 * std::clamp(k * k * g_milkyWay, 0.f, 1.f));
        mwMat.maps[MATERIAL_MAP_DIFFUSE].color = Color{g, g, g, 255};
        // the level of detail: a screen pixel's angle against the map's texel (pi / height at the equator)
        float pix = (float)(curCam.fovy * DEG2RAD / std::max(1, GetRenderHeight()));
        float lod = std::max(0.f, std::log2(pix / (float)(PI / std::max(1, mwTex.height))) + 0.6f);
        SetShaderValue(mwShader, mwLodLoc, &lod, SHADER_UNIFORM_FLOAT);
        DrawMesh(sphereLo, mwMat, MatrixMultiply(MatrixScale(49, 49, 49), MatrixTranslate(p.x, p.y, p.z)));
        rlDrawRenderBatchActive();
    }
    starMat.maps[MATERIAL_MAP_DIFFUSE].color = Color{255, 255, 255, (unsigned char)(std::clamp(brightness, 0.f, 1.f) * 255)};
    DrawMesh(starMesh, starMat, MatrixMultiply(MatrixScale(50, 50, 50), MatrixTranslate(p.x, p.y, p.z)));
    EndBlendMode();
    rlEnableBackfaceCulling();
    rlEnableDepthMask();
    rlEnableDepthTest();
}

void drawGlow(Vector3 pos, float size, Color c) {
    if (g_shadowPass) return;
    BeginBlendMode(BLEND_ADDITIVE);
    rlDisableDepthMask();
    facingBillboard(glowTex, pos, size, c);
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
    EndBlendMode();
}

void drawBodySphere(const Universe& U, int body, const Vec3d& center, double radiusUnits, const Mat3d& rot,
                    const Vec3d& sunDir, float lightScale, bool halo, int octaves, bool surface) {
    const Body& b = U.bodies[body];
    int kind = (int)b.surf;
    setPlanetUniforms(b, rot, sunDir, kind, octaves, lightScale);
    if (surface) {
        const Mesh& mesh = radiusUnits < 0.5 ? sphereLo : sphereMesh;
        bool baked = bindBaked(body, b);
        bool mapped = bindMap(body, b, true, false); // (the relief of a far body is not generated for this)
        DrawMesh(mesh, planetMat, toMatrix(rot, center, radiusUnits, radiusUnits, radiusUnits));
        if (mapped) unbindMap();
        if (baked) unbindBaked();
    }
    if (halo && b.atmTop > 0 && !b.isStar) {
        double hr = radiusUnits * (1.0 + std::max(b.atmTop / b.radius * 0.55, 0.012));
        double camDist = len(center);
        if (camDist > hr * 1.001) {
            setI(planetShader, PL.kind, 7);
            setV3(planetShader, PL.center, center);
            setF(planetShader, PL.radius, (float)radiusUnits);
            setF(planetShader, PL.haloRadius, (float)hr);
            BeginBlendMode(BLEND_ADDITIVE);
            rlDisableDepthMask();
            DrawMesh(sphereMesh, planetMat, toMatrix(rot, center, hr, hr, hr));
            rlEnableDepthMask();
            EndBlendMode();
        }
    }
}

void drawRings(const Universe& U, int body, const Vec3d& center, double unitsPerMeter, const Mat3d& rot, const Vec3d& sunDir) {
    const Body& b = U.bodies[body];
    if (b.ringOuter <= 0) return;
    auto it = ringCache.find(body);
    if (it == ringCache.end()) it = ringCache.emplace(body, genRing((float)(b.ringInner / b.ringOuter), 256)).first;
    setPlanetUniforms(b, rot, sunDir, 8, 4, 1.0f);
    setV3(planetShader, PL.center, center);
    setF(planetShader, PL.radius, (float)(b.radius * unitsPerMeter));
    double s = b.ringOuter * unitsPerMeter;
    rlDisableBackfaceCulling();
    rlDisableDepthMask();
    BeginBlendMode(BLEND_ALPHA);
    DrawMesh(it->second, planetMat, toMatrix(rot, center, s, s, s));
    EndBlendMode();
    rlEnableDepthMask();
    rlEnableBackfaceCulling();
}

void drawBodyCap(const Universe& U, int body, const Vec3d& camRel, const Mat3d& rot, const Vec3d& sunDir,
                 Vector3 fogColor, float fogK, float lightScale) {
    const Body& b = U.bodies[body];
    double R = b.radius;
    double d = len(camRel);
    Vec3d u = camRel / d;
    Mat3d RT = rot.T();
    const bool relief = terrain::ensure(body) != nullptr; // near the ground: generate the relief now
    double alt = std::max(d - R - (relief ? terrain::height(body, RT * u) : 0.0), 0.5); // above the ground below
    // the cap reaches the horizon over the lowest ground (a camera in a basin below the reference radius
    // sees farther than the datum horizon, it does not see nothing) and beyond it as far as the highest
    // summits can still rise above the horizon
    double Rlo = R + (relief ? terrain::minHeight(body) : 0.0);
    double Rhi = R + (relief ? terrain::maxHeight(body) : 0.0);
    double thH = std::acos(std::clamp(Rlo / std::max(d, Rlo * 1.0000001), -1.0, 1.0));
    double thBeyond = std::acos(std::clamp(Rlo / std::max(Rhi, Rlo * 1.0000001), -1.0, 1.0));
    double thMax = std::min(thH * 1.03 + thBeyond + 0.002, PI * 0.98);
    double thMin = std::clamp(0.15 * alt / R, 1e-9, thMax * 0.2);
    Vec3d e1 = anyPerp(u), e2 = cross(u, e1);
    if (g_capBuiltQ != g_terrainQ) {
        UnloadMesh(capMesh);
        buildCapMesh();
    }
    // the mesh is rebuilt only when the camera has moved over the ground (or up / down) by a few per cent of
    // its height, the body has turned a little, or the terrain changed; in between the same vertices are
    // drawn moved by the camera's motion and the body's rotation (no visible difference, most frames
    // skip computing ~20,000 terrain heights)
    Vec3d camBF = RT * camRel;
    bool rebuild = !g_cap.valid || g_cap.body != body || g_cap.q != g_terrainQ || g_cap.relief != relief ||
                   g_cap.rev != terrain::revision(body) || g_cap.detRange != g_detRange;
    if (!rebuild) {
        double moved = len(camBF - g_cap.camBF);
        Mat3d dR = rot * g_cap.rot0.T();
        double tr = std::clamp((dR.m[0][0] + dR.m[1][1] + dR.m[2][2] - 1) * 0.5, -1.0, 1.0);
        rebuild = moved > 0.02 * std::min(alt, g_cap.alt) || std::fabs(alt / g_cap.alt - 1) > 0.03 || std::acos(tr) > 0.03 * DEG;
    }
    if (rebuild) {
        const double minWl = CAP_Q_MINWL[g_terrainQ], lodK = CAP_Q_LODK[g_terrainQ];
        auto put = [&](int vi, double th, double ph) {
            Vec3d dir = u * std::cos(th) + (e1 * std::cos(ph) + e2 * std::sin(ph)) * std::sin(th);
            Vec3d nf = RT * dir;
            // the ground the physics stands on: the same elevation function (its finest relief only near the
            // camera: further out the mesh is too coarse to show it)
            double dist = std::hypot(th * R, alt);
            Vec3d p = dir * (R + (relief ? terrain::heightLod(body, nf, std::max(minWl, dist * lodK)) : 0.0)) - camRel;
            capPos[vi * 3] = (float)p.x; capPos[vi * 3 + 1] = (float)p.y; capPos[vi * 3 + 2] = (float)p.z;
            capNrm[vi * 3] = (float)nf.x; capNrm[vi * 3 + 1] = (float)nf.y; capNrm[vi * 3 + 2] = (float)nf.z;
        };
        put(0, 0, 0);
        double ratio = thMax / thMin;
        {
            // the rings on a few threads (heights only read the maps)
            unsigned nth = std::clamp(std::thread::hardware_concurrency(), 1u, 8u);
            std::atomic<int> next{0};
            auto work = [&] {
                for (int r; (r = next++) < CAP_RINGS;) {
                    double th = thMin * std::pow(ratio, (double)r / (CAP_RINGS - 1));
                    for (int sg = 0; sg < CAP_SEGS; sg++) put(1 + r * CAP_SEGS + sg, th, (double)sg / CAP_SEGS * TAU);
                }
            };
            std::vector<std::thread> pool;
            for (unsigned k = 1; k < nth; k++) pool.emplace_back(work);
            work();
            for (auto& t : pool) t.join();
        }
        // the normal of the drawn surface at every vertex (from its neighbours along the ring and across)
        {
            auto P = [&](int vi) { return Vec3d{capPos[vi * 3], capPos[vi * 3 + 1], capPos[vi * 3 + 2]}; };
            for (int r = 0; r < CAP_RINGS; r++)
                for (int sgi = 0; sgi < CAP_SEGS; sgi++) {
                    int vi = 1 + r * CAP_SEGS + sgi;
                    int vr0 = r > 0 ? vi - CAP_SEGS : 0, vr1 = r + 1 < CAP_RINGS ? vi + CAP_SEGS : vi;
                    int vs0 = 1 + r * CAP_SEGS + (sgi + CAP_SEGS - 1) % CAP_SEGS, vs1 = 1 + r * CAP_SEGS + (sgi + 1) % CAP_SEGS;
                    Vec3d a = P(vr1) - P(vr0), b = P(vs1) - P(vs0);
                    Vec3d nn = cross(b, a);
                    double l = len(nn);
                    Vec3d up = norm(P(vi) + camRel);
                    nn = l > 1e-12 ? nn / l : up;
                    if (dot(nn, up) < 0) nn = -nn;
                    capTan[vi * 4] = (float)nn.x; capTan[vi * 4 + 1] = (float)nn.y; capTan[vi * 4 + 2] = (float)nn.z; capTan[vi * 4 + 3] = 1;
                }
            Vec3d up0 = norm(P(0) + camRel);
            capTan[0] = (float)up0.x; capTan[1] = (float)up0.y; capTan[2] = (float)up0.z; capTan[3] = 1;
        }
        UpdateMeshBuffer(capMesh, 0, capPos.data(), (int)(capPos.size() * sizeof(float)), 0);
        UpdateMeshBuffer(capMesh, 2, capNrm.data(), (int)(capNrm.size() * sizeof(float)), 0);
        UpdateMeshBuffer(capMesh, 4, capTan.data(), (int)(capTan.size() * sizeof(float)), 0);
        g_cap.valid = true;
        g_cap.body = body;
        g_cap.q = g_terrainQ;
        g_cap.relief = relief;
        g_cap.rev = terrain::revision(body);
        g_cap.detRange = g_detRange;
        g_cap.camBF = camBF;
        g_cap.camRel0 = camRel;
        g_cap.rot0 = rot;
        g_cap.alt = alt;
    }
    // the built vertices, turned with the body since then and shifted by the camera's motion (double here,
    // the small offset goes to the GPU)
    Mat3d dR = rot * g_cap.rot0.T();
    Matrix capModel = toMatrix(dR, dR * g_cap.camRel0 - camRel);
    int kind = (int)b.surf;
    setPlanetUniforms(b, rot, sunDir, kind, 16, lightScale);
    if (g_pbr && b.atmTop > 0) setAtm(planetShader, PL, b, camRel, &planetMat);
    setV3(planetShader, PL.fogColor, fogColor);
    setF(planetShader, PL.fogK, fogK);
    // ground detail below a few km (fewer octaves when the adaptive detail lowers the planet octaves)
    int dOct = alt < 30000 * g_detRange ? std::min(std::clamp(g_maxPlanetOctaves - 6, 2, 10), CAP_Q_DOCT[g_terrainQ]) : 0;
    setI(planetShader, PL.geoN, 1);
    setF(planetShader, PL.geoFar, (float)(std::max(2000.0, 40.0 * alt) * g_detRange));
    setI(planetShader, PL.detailOct, dOct);
    if (dOct > 0) {
        Vec3d cb = RT * camRel; // body-fixed camera position
        auto wrapP = [](double v) { return (float)(v - std::floor(v / 16384.0) * 16384.0); };
        setV3(planetShader, PL.detailOfs, Vector3{wrapP(cb.x), wrapP(cb.y), wrapP(cb.z)});
    }
    // the sea's waves (Earth, below about 30 km: higher up even the longest are under a pixel)
    bool waves = kind == 1 && alt < 30000;
    setI(planetShader, PL.waveOn, waves ? 1 : 0);
    if (waves) {
        float wv[48];
        seaWaves(RT * camRel, R, g_seaT, wv);
        SetShaderValueV(planetShader, PL.waves, wv, SHADER_UNIFORM_VEC4, 12);
        setI(planetShader, PL.waveN, 6 + 2 * std::clamp(g_terrainQ, 0, 3)); // (finest trains dropped first)
    }
    // cloud deck at a few km: seen from below as shadows on the ground
    float cl = (float)std::clamp((alt - 3000.0) / 11000.0, 0.0, 1.0);
    setF(planetShader, PL.cloudLow, cl * cl * (3 - 2 * cl));
    // the limb glow belongs to the view from space; inside the lower atmosphere the fog does the haze
    float rk = b.atmTop > 0 ? (float)std::clamp((alt - 0.04 * b.atmTop) / (0.5 * b.atmTop), 0.0, 1.0) : 1.f;
    setF(planetShader, PL.rimK, rk * rk * (3 - 2 * rk));
    bool baked = bindBaked(body, b);
    bool mapped = bindMap(body, b, false, true);
    DrawMesh(capMesh, planetMat, capModel);
    if (mapped) unbindMap();
    if (baked) unbindBaked();
    planetMat.maps[MATERIAL_MAP_EMISSION].texture = Texture2D{};
    setF(planetShader, PL.fogK, 0.f);
    setI(planetShader, PL.detailOct, 0);
    setI(planetShader, PL.geoN, 0);
    setI(planetShader, PL.waveOn, 0);
    setF(planetShader, PL.cloudLow, 1.f);
    setF(planetShader, PL.rimK, 1.f);
}
void setSeaTime(double t) { g_seaT = t; }

// ------------------------------------------------------------------ vessels
namespace {

struct DrawCtx {
    Matrix base;
    Vec3d lightDir;
    float light;
    Vec3d camPos;
    float ambient;
};

void envSet(const Vec3d& up, Vector3 sky, Vector3 ground) {
    setV3(shipShader, shipEnvUp, norm(up));
    setV3(shipShader, shipEnvSky, sky);
    setV3(shipShader, shipEnvGround, ground);
}

float g_ghost = 1.0f; // opacity of the parts being drawn (hidden parts in the factory: faint)
void piece(const DrawCtx& c, const Mesh& m, const Matrix& local, Color col, float emissive = 0) {
    if (g_shadowPass) {
        g_scale = {1, 1, 1};
        g_shapeK = 0;
        if (col.a < 200 || g_ghost < 1.0f) return;
        DrawMesh(m, g_depthMat, MatrixMultiply(local, c.base));
        return;
    }
    if (g_ghost < 1.0f) col.a = (unsigned char)(col.a * g_ghost);
    shipMat.maps[MATERIAL_MAP_DIFFUSE].color = col;
    setF(shipShader, shipEmissive, emissive);
    setI(shipShader, shipFinish, g_fin);
    SetShaderValue(shipShader, shipUV, &g_uv, SHADER_UNIFORM_VEC2);
    SetShaderValue(shipShader, shipObjScale, &g_scale, SHADER_UNIFORM_VEC3);
    setI(shipShader, shipShapeK, g_shapeK);
    g_scale = {1, 1, 1};
    g_shapeK = 0;
    DrawMesh(m, shipMat, MatrixMultiply(local, c.base));
}

Matrix frustumM(double z0, double h, double rb, double rt, double ox = 0, double oy = 0) {
    double mx = std::max(rb, rt);
    return toMatrix(Mat3d{}, {ox, oy, z0}, mx, mx, h);
}

void cyl(const DrawCtx& c, double z0, double h, double rb, double rt, Color col, double ox = 0, double oy = 0, float em = 0) {
    g_uv = {(float)(TAU * std::max(rb, rt)), (float)h};
    g_scale = {(float)std::max(rb, rt), (float)std::max(rb, rt), (float)h};
    piece(c, frustum(rb, rt), frustumM(z0, h, rb, rt, ox, oy), col, em);
}

// contoured, hollow nozzle: exit (radius exitR) at z0, throat at z0 + L, placed by M. The inside is
// sooty; when firing the wall glows and the throat shows the combustion light
void bellNozzle(const DrawCtx& c, const Matrix& M, double z0, double L, double exitR, Color col, double ox, double oy, float em, float glow, int finOuter) {
    Matrix m = MatrixMultiply(toMatrix(Mat3d{}, {ox, oy, z0}, exitR, exitR, L), M);
    {
        FinScope f(finOuter);
        g_uv = {(float)(TAU * exitR), (float)L};
        g_scale = {(float)exitR, (float)exitR, (float)L};
        piece(c, bellOuter, m, col, em + glow * 0.5f);
    }
    {
        FinScope f(FIN_METAL);
        g_uv = {(float)(TAU * exitR), (float)L};
        piece(c, bellInner, m, Color{(unsigned char)std::min(255.f, col.r * 0.8f + 40), (unsigned char)std::min(255.f, col.g * 0.8f + 36), (unsigned char)std::min(255.f, col.b * 0.8f + 34), col.a}, em + glow); // sooty metal
    }
    // injector face / throat: dark when cold, incandescent when firing
    double tr = exitR * BELL_THROAT * 0.97;
    Matrix tm = MatrixMultiply(toMatrix(Mat3d{}, {ox, oy, z0 + L * 0.995}, tr, tr, std::max(L * 0.004, 1e-3)), M);
    piece(c, frustum(1, 1), tm, glow > 0 ? Color{255, 214, 160, col.a} : Color{28, 26, 25, col.a}, glow > 0 ? 1.2f + glow : em);
}

void strut(const DrawCtx& c, const Vec3d& a, const Vec3d& b, double r, Color col);
// parachute(s) hanging from 'top' (craft frame) toward 'up': riser, confluence, suspension lines to the
// 16 gore seams of each canopy. D: nominal canopy diameter, n canopies in a cluster, open 0..1
void drawCanopies(const DrawCtx& c, const Vec3d& top, const Vec3d& up0, double D, int n, double open, double riser = 0, double pilotD = 0,
                  double reef = 1) {
    Vec3d up = len(up0) > 1e-9 ? norm(up0) : Vec3d{0, 0, 1};
    open = clampd(open, 0, 1);
    double Rf = D * 0.5 * clampd(reef, 0.1, 1.0);
    double Rr = Rf * (0.12 + 0.88 * std::pow(open, 0.8)); // canopy radius while it fills
    double zs = Rf * (1.9 - 0.9 * open);                   // a long, narrow bag before it is full
    double Ls = 1.15 * D;                                  // suspension line length
    Vec3d conf = top + up * (riser > 0 ? riser : 0.2 * D);
    double lineR = std::max(0.012, D * 0.0007);
    strut(c, top, conf, std::max(0.04, D * 0.0018), Color{215, 210, 200, 255});
    Vec3d ax0 = anyPerp(up), ay0 = cross(up, ax0);
    for (int k = 0; k < std::max(1, n); k++) {
        Vec3d axis = up;
        if (n > 1) {
            double az = TAU * k / n, tilt = 15 * DEG;
            axis = norm(up * std::cos(tilt) + (ax0 * std::cos(az) + ay0 * std::sin(az)) * std::sin(tilt));
        }
        Vec3d x = norm(anyPerp(axis)), y = cross(axis, x);
        Vec3d hemC = conf + axis * std::sqrt(std::max(Ls * Ls - Rr * Rr, 0.01));
        Vec3d org = hemC - axis * (0.18 * zs);
        Matrix M = toMatrix(Mat3d::fromCols(x, y, axis), org, Rr, Rr, zs);
        {
            FinScope f(FIN_PAINT);
            piece(c, canopyMesh[0], M, Color{236, 112, 36, 255});
            piece(c, canopyMesh[1], M, Color{240, 238, 232, 255});
        }
        const float ph1 = 1.32f;
        for (int g = 0; g < 16; g++) {
            double th = g * TAU / 16;
            Vec3d hp = org + (x * (std::sin(ph1) * std::cos(th)) + y * (std::sin(ph1) * std::sin(th))) * Rr + axis * (0.72 * std::cos(ph1) * zs);
            strut(c, conf, hp, lineR, Color{225, 222, 214, 255});
        }
        if (pilotD > 0) {
            // the pilot chute still pulling ahead of the canopy's apex on its bridle
            Vec3d apex = org + axis * (0.9 * zs);
            Vec3d pc = apex + axis * (pilotD * 2.2);
            strut(c, apex, pc, lineR * 1.5, Color{225, 222, 214, 255});
            double pr = pilotD * 0.5;
            Matrix Mp = toMatrix(Mat3d::fromCols(x, y, axis), pc, pr, pr, pr * 0.9);
            FinScope f(FIN_PAINT);
            piece(c, canopyMesh[0], Mp, Color{236, 112, 36, 255});
            piece(c, canopyMesh[1], Mp, Color{240, 238, 232, 255});
        }
    }
}

void box(const DrawCtx& c, const Mat3d& R, const Vec3d& p, double sx, double sy, double sz, Color col, float em = 0) {
    g_uv = {(float)std::max(sx, sy), (float)sz};
    g_scale = {(float)sx, (float)sy, (float)sz};
    piece(c, cubeMesh, toMatrix(R, p, sx, sy, sz), col, em);
}

void strut(const DrawCtx& c, const Vec3d& a, const Vec3d& b, double r, Color col) {
    Vec3d d = b - a;
    double l = len(d);
    if (l < 1e-6) return;
    Vec3d z = d / l, x = anyPerp(z), y = cross(z, x);
    g_uv = {(float)(TAU * r), (float)l};
    piece(c, frustum(1, 1), toMatrix(Mat3d::fromCols(x, y, z), a, r, r, l), col);
}

Color shade(Color c, float f) {
    return {(unsigned char)std::clamp(c.r * f, 0.f, 255.f), (unsigned char)std::clamp(c.g * f, 0.f, 255.f),
            (unsigned char)std::clamp(c.b * f, 0.f, 255.f), c.a};
}

Vector3 flameColor(Prop p) {
    switch (p) {
    case Prop::Kerolox: return {1.0f, 0.55f, 0.22f};
    case Prop::Hydrolox: return {0.55f, 0.55f, 1.0f};
    case Prop::Methalox: return {0.35f, 0.5f, 1.0f};
    case Prop::Hypergolic: return {1.0f, 0.55f, 0.45f};
    case Prop::LH2: return {0.55f, 0.75f, 1.0f};
    case Prop::Xenon: return {0.35f, 0.55f, 1.0f};
    case Prop::Solid: return {1.0f, 0.8f, 0.5f};
    default: return {1, 1, 1};
    }
}

struct Plume {
    Vec3d pos;
    double exitR, length, flare;
    Vector3 col;
    float intensity;
};

} // namespace

void drawVessel(const VesselVisual& vv, const Mat3d& rot, const Vec3d& origin, double zOffset, const Vec3d& lightDir,
                float light, const Vec3d& camPos, float ambient) {
    if (!vv.design) return;
    const Design& d = *vv.design;
    DrawCtx c;
    c.base = toMatrix(rot, origin + rot * Vec3d{0, 0, zOffset});
    c.lightDir = lightDir;
    c.light = light;
    c.camPos = camPos;
    c.ambient = ambient;
    setV3(shipShader, shipLight, lightDir);
    setF(shipShader, shipLightI, light);
    setF(shipShader, shipAmbient, ambient);
    setV3(shipShader, shipCamPos, camPos);
    setV3(plumeShader, plumeCamPos, camPos);

    int n = (int)d.stack.size();
    std::vector<double> zb(n);
    double z = 0;
    for (int i = n - 1; i >= 0; i--) {
        zb[i] = z;
        z += partGeom(d.stack[i]).height;
    }
    std::vector<Plume> plumes;
    std::vector<std::pair<Component, bool>> shells, ghostShells; // fairing pieces, highlighted
    rlDisableBackfaceCulling();
    // hidden parts (factory) in a second, see-through pass after the others
    auto hiddenPart = [&](int i) { return vv.ghostHidden && d.stack[i].hidden; };
    bool anyHidden = false;
    for (int i = 0; i < n; i++) anyHidden |= hiddenPart(i);
    for (int pass = 0; pass < (anyHidden ? 2 : 1); pass++) {
    if (pass == 1) {
        rlDrawRenderBatchActive();
        BeginBlendMode(BLEND_ALPHA);
        rlDisableDepthMask();
        g_ghost = 0.16f;
    }
    for (int i = 0; i < n; i++) {
        if (hiddenPart(i) != (pass == 1)) continue;
        const PartInst& p = d.stack[i];
        const PartDef& def = CATALOG.defs[p.def];
        PartGeom g = partGeom(p);
        double z0 = zb[i], h = g.height;
        bool hl = (i == vv.highlight) || (vv.selMask && i < (int)vv.selMask->size() && (*vv.selMask)[i]);
        if (def.kind == PK::Fairing) {
            std::vector<Component> fp;
            fairingPieces(d, i, zb, 0, fp);
            for (auto& k : fp) (pass == 1 ? ghostShells : shells).push_back({k, hl});
            continue;
        }
        float em = hl ? 0.35f : 0.f;
        Color col = def.color;
        Color dark{45, 47, 52, 255};
        int fin = FIN_PAINT;
        switch (def.kind) {
        case PK::Tank: fin = finishForProp(def.prop); break;
        case PK::Engine:
        case PK::Decoupler:
        case PK::Reactor:
        case PK::Legs:
        case PK::RTG:
        case PK::Hyperdrive: fin = FIN_METAL; break;
        case PK::HeatShield: fin = FIN_ABLATOR; break;
        case PK::Sail: fin = FIN_FOIL; break;
        case PK::Probe: fin = p.def == CATALOG.find("probe_bus") ? FIN_FOIL : FIN_PAINT; break;
        default: break;
        }
        FinScope finScope(fin);
        switch (def.kind) {
        case PK::Tank: {
            cyl(c, z0, h, g.rBot, g.rTop, col, 0, 0, em);
            double band = std::min(0.12, h * 0.1);
            cyl(c, z0, band, g.rBot * 1.01, g.rBot * 1.01, shade(col, 0.55), 0, 0, em);
            cyl(c, z0 + h - band, band, g.rTop * 1.01, g.rTop * 1.01, shade(col, 0.55), 0, 0, em);
            if (def.prop == Prop::Hydrolox || def.prop == Prop::LH2) {
                for (double zz = z0 + h * 0.25; zz < z0 + h * 0.9; zz += std::max(h * 0.25, 1.0))
                    cyl(c, zz, 0.05, g.rBot * 1.005, g.rBot * 1.005, shade(col, 0.8), 0, 0, em);
            }
            break;
        }
        case PK::Capsule: {
            cyl(c, z0, h * 0.82, g.rBot, g.rTop * 1.3, col, 0, 0, em);
            cyl(c, z0 + h * 0.82, h * 0.18, g.rTop * 0.8, g.rTop * 0.7, shade(col, 0.75), 0, 0, em);
            cyl(c, z0 - 0.02, 0.18, g.rBot * 1.01, g.rBot * 1.01, Color{70, 50, 35, 255}, 0, 0, em);
            box(c, rotZ(0.6), {g.rBot * 0.55, 0, z0 + h * 0.45}, 0.35, 0.35, 0.35, Color{30, 40, 60, 255}, em);
            break;
        }
        case PK::Probe: {
            if (p.def == CATALOG.find("probe_bus")) {
                box(c, Mat3d{}, {0, 0, z0 + h * 0.5}, g.rBot * 1.3, g.rBot * 1.3, h * 0.95, col, em);
                double dr = g.rBot * 0.9;
                cyl(c, z0 + h * 0.98, dr * 0.35, dr * 0.08, dr, Color{235, 235, 235, 255}, 0, 0, em);
            } else {
                cyl(c, z0, h, g.rBot, g.rTop, col, 0, 0, em);
                FinScope ff(FIN_FOIL);
                cyl(c, z0 + h * 0.4, h * 0.2, g.rBot * 1.01, g.rBot * 1.01, Color{200, 165, 60, 255}, 0, 0, em);
            }
            break;
        }
        case PK::Engine: {
            double plateH = 0.35;
            double bellL = def.length;
            cyl(c, z0 + h - plateH, plateH, g.rTop * 0.9, g.rTop, dark, 0, 0, em);
            double bellR;
            auto pos = clusterLayout(p.cluster, g.rTop, bellR, def.exitDiam * 0.5);
            bool firing = vv.ignited && vv.seg && i < (int)vv.seg->size() && (*vv.seg)[i] == vv.bottomSeg && vv.throttle > 0;
            double hasProp = 1;
            if (vv.prop && vv.seg && !vv.infinite && (int)vv.seg->size() >= n && (int)vv.prop->size() >= n) {
                // check any tank of this propellant left in the segment
                hasProp = 0;
                for (int k = 0; k < n; k++)
                    if ((*vv.seg)[k] == (*vv.seg)[i] && CATALOG.defs[d.stack[k].def].kind == PK::Tank &&
                        CATALOG.defs[d.stack[k].def].prop == def.prop && (*vv.prop)[k] > 0)
                        hasProp = 1;
            }
            for (auto& o : pos) {
                double throatR = bellR * 0.28;
                double chamberL = bellL * 0.25;
                cyl(c, z0 + h - plateH - chamberL, chamberL, throatR * 1.3, throatR * 1.5, Color{90, 92, 98, 255}, o.x, o.y, em);
                double bz = z0 + h - plateH - bellL;
                Color bellCol = def.powerUse > 0 ? Color{110, 120, 140, 255} : Color{75, 60, 50, 255};
                float glow = firing && hasProp > 0 ? (float)(0.15 + 0.4 * vv.throttle) : 0.f;
                bellNozzle(c, MatrixIdentity(), bz, bellL * 0.78, bellR, bellCol, o.x, o.y, em, glow, def.powerUse > 0 ? FIN_METAL : FIN_NOZZLE);
                if (firing && hasProp > 0) {
                    double pa = std::clamp(vv.pressureAtm, 0.0, 1.0);
                    double thr = std::max(vv.throttle, def.minThrottle);
                    Plume pl;
                    pl.pos = {o.x, o.y, bz};
                    pl.exitR = bellR;
                    double elec = def.powerUse > 0 ? 0.4 : 1.0;
                    pl.length = bellR * (4 + 14 * (1 - pa)) * (0.4 + 0.6 * thr) * elec + bellR * 2;
                    pl.flare = 1.0 + 2.5 * (1 - pa);
                    pl.col = flameColor(def.prop);
                    pl.intensity = (float)(0.55 + 0.45 * thr) * (def.prop == Prop::Hydrolox || def.prop == Prop::LH2 ? 0.55f : 1.0f);
                    plumes.push_back(pl);
                }
            }
            break;
        }
        case PK::SRB: {
            double nozL = std::min(h * 0.08, 2.5);
            cyl(c, z0 + nozL, h - nozL - g.rTop * 1.2, g.rBot, g.rTop, col, 0, 0, em);
            cyl(c, z0 + h - g.rTop * 1.2, g.rTop * 1.2, g.rTop, 0.0, col, 0, 0, em);
            bellNozzle(c, MatrixIdentity(), z0, nozL, def.exitDiam * 0.5, Color{60, 55, 52, 255}, 0, 0, em, 0.f, FIN_ABLATOR); // carbon-phenolic
            bool burning = vv.ignited && vv.seg && (*vv.seg)[i] == vv.bottomSeg && vv.prop && ((*vv.prop)[i] > 0 || vv.infinite);
            if (burning) {
                double pa = std::clamp(vv.pressureAtm, 0.0, 1.0);
                Plume pl;
                pl.pos = {0, 0, z0};
                pl.exitR = def.exitDiam * 0.5;
                pl.length = pl.exitR * (8 + 10 * (1 - pa));
                pl.flare = 1.2 + 2.0 * (1 - pa);
                pl.col = flameColor(Prop::Solid);
                pl.intensity = 1.0f;
                plumes.push_back(pl);
            }
            break;
        }
        case PK::Decoupler: {
            cyl(c, z0, h, g.rBot, g.rTop, dark, 0, 0, em);
            cyl(c, z0 + h * 0.35, h * 0.3, g.rBot * 1.015, g.rBot * 1.015, col, 0, 0, em);
            Interstage is = interstageFor(d, i);
            if (is.on) {
                Color sk{34, 36, 40, 255}; // black thermal-protection coat
                {
                    FinScope pf(FIN_PAINT);
                    cyl(c, z0 + h, is.height, is.rBot, is.rTop, sk, 0, 0, em);
                }
                // stiffening rings
                for (int k = 1; k <= 2; k++) {
                    double f = k / 3.0, rr = is.rBot + (is.rTop - is.rBot) * f;
                    cyl(c, z0 + h + is.height * f - 0.03, 0.06, rr * 1.006, rr * 1.006, Color{180, 184, 192, 255}, 0, 0, em);
                }
            }
            break;
        }
        case PK::NoseCone: {
            double r = g.rBot;
            cyl(c, z0, h * 0.45, r, r * 0.8, col, 0, 0, em);
            cyl(c, z0 + h * 0.45, h * 0.35, r * 0.8, r * 0.38, col, 0, 0, em);
            cyl(c, z0 + h * 0.8, h * 0.2, r * 0.38, 0.0, col, 0, 0, em);
            break;
        }
        case PK::Adapter: cyl(c, z0, h, g.rBot, g.rTop, col, 0, 0, em); break;
        case PK::HeatShield:
            cyl(c, z0 + h * 0.3, h * 0.7, g.rBot, g.rTop, Color{60, 45, 35, 255}, 0, 0, em);
            cyl(c, z0, h * 0.3, g.rBot * 0.7, g.rBot, Color{80, 55, 38, 255}, 0, 0, em);
            break;
        case PK::Parachute:
            cyl(c, z0, h * 0.7, g.rBot, g.rTop, Color{230, 230, 230, 255}, 0, 0, em);
            cyl(c, z0 + h * 0.7, h * 0.3, g.rTop, g.rTop * 0.6, col, 0, 0, em);
            break;
        case PK::CMG:
            cyl(c, z0, h, g.rBot, g.rTop, col, 0, 0, em);
            cyl(c, z0 + h * 0.3, h * 0.4, g.rBot * 1.03, g.rBot * 1.03, shade(col, 0.6), 0, 0, em);
            break;
        case PK::Reactor: {
            double r = g.rBot;
            cyl(c, z0 + h * 0.75, h * 0.25, r * 0.5, r, Color{110, 110, 115, 255}, 0, 0, em);
            cyl(c, z0, h * 0.75, r * 0.45, r * 0.45, col, 0, 0, em);
            for (int k = 0; k < 4; k++) {
                Mat3d Rz = rotZ(k * PI / 2);
                Vec3d off = Rz * Vec3d{r * 0.45 + h * 0.2, 0, z0 + h * 0.35};
                box(c, Rz, off, h * 0.4, 0.03, h * 0.6, Color{220, 222, 228, 255}, em);
            }
            break;
        }
        case PK::Sail:
            cyl(c, z0, h, g.rBot, g.rTop, Color{210, 210, 215, 255}, 0, 0, em);
            if (vv.sail) {
                double side = std::sqrt(def.sailArea);
                box(c, rotZ(PI / 4), {0, 0, z0 + h}, side, side, 0.004, Color{235, 225, 170, 255}, 0.25f);
                box(c, Mat3d{}, {0, 0, z0 + h}, side * 0.72, 0.05 + side * 0.002, 0.05 + side * 0.002, Color{90, 90, 95, 255});
                box(c, Mat3d{}, {0, 0, z0 + h}, 0.05 + side * 0.002, side * 0.72, 0.05 + side * 0.002, Color{90, 90, 95, 255});
            }
            break;
        case PK::Hyperdrive:
            cyl(c, z0, h, g.rBot, g.rTop, col, 0, 0, em);
            cyl(c, z0 + h * 0.25, h * 0.12, g.rBot * 1.08, g.rBot * 1.08, Color{160, 120, 255, 255}, 0, 0, vv.hyperGlow ? 1.5f : 0.6f);
            cyl(c, z0 + h * 0.63, h * 0.12, g.rBot * 1.08, g.rBot * 1.08, Color{160, 120, 255, 255}, 0, 0, vv.hyperGlow ? 1.5f : 0.6f);
            break;
        default: cyl(c, z0, h, g.rBot, g.rTop, col, 0, 0, em); break;
        }
        // radial attachments
        double rMid = (g.rBot + g.rTop) * 0.5;
        for (size_t ri = 0; ri < p.radial.size(); ri++) {
            const RadialAttach& ra = p.radial[ri];
            const PartDef& rd = CATALOG.defs[ra.def];
            for (int k = 0; k < ra.count; k++) {
                double a = (double)k / ra.count * TAU + ri * 0.4 + ra.angle * DEG;
                Mat3d Rz = rotZ(a);
                Vec3d outv = Rz * Vec3d{1, 0, 0};
                double z0 = zb[i] + ra.offset; // (shifted copy of the part's base for this group)
                switch (rd.kind) {
                case PK::SRB: {
                    // side booster: casing, nose cap, nozzle and two attach struts
                    double rb = rd.diam * 0.5, Lb = rd.length, dist = sideBoosterDistance(rMid, rd);
                    Vec3d axis = outv * dist;
                    z0 = stageBottomZ(d, i, zb) + ra.offset; // nozzle exits level with the main engines
                    double nozL = std::min(Lb * 0.08, 2.5);
                    if (rb >= 1.2) {
                        // large segmented booster: aft skirt, segment joints, frustum and nose cap
                        double noseL = rb * 2.6, skirtH = std::min(Lb * 0.07, 3.2), body = Lb - nozL - noseL;
                        Color jc{(unsigned char)(rd.color.r * 0.78), (unsigned char)(rd.color.g * 0.78), (unsigned char)(rd.color.b * 0.78), 255};
                        cyl(c, z0 + nozL * 0.35, skirtH, rb * 1.38, rb, rd.color, axis.x, axis.y, em);
                        cyl(c, z0 + nozL, body, rb, rb, rd.color, axis.x, axis.y, em);
                        for (int s = 1; s <= 3; s++) cyl(c, z0 + nozL + body * s / 4.0 - 0.15, 0.3, rb * 1.015, rb * 1.015, jc, axis.x, axis.y, em);
                        cyl(c, z0 + Lb - noseL, noseL * 0.55, rb, rb * 0.55, rd.color, axis.x, axis.y, em);
                        cyl(c, z0 + Lb - noseL * 0.45, noseL * 0.4, rb * 0.55, rb * 0.12, rd.color, axis.x, axis.y, em);
                        cyl(c, z0 + Lb - noseL * 0.05, noseL * 0.05, rb * 0.12, 0.0, rd.color, axis.x, axis.y, em);
                    } else {
                        cyl(c, z0 + nozL, Lb - nozL - rb * 1.2, rb, rb, rd.color, axis.x, axis.y, em);
                        cyl(c, z0 + Lb - rb * 1.2, rb * 1.2, rb, 0.0, rd.color, axis.x, axis.y, em);
                    }
                    bellNozzle(c, MatrixIdentity(), z0, nozL, rd.exitDiam * 0.5, Color{60, 55, 52, 255}, axis.x, axis.y, em, 0.f, FIN_ABLATOR);
                    for (double fz : {0.12, 0.85})
                        strut(c, outv * rMid + Vec3d{0, 0, z0 + Lb * fz}, outv * (dist - rb) + Vec3d{0, 0, z0 + Lb * fz}, 0.11,
                              Color{120, 122, 128, 255});
                    break;
                }
                case PK::Battery: {
                    double s = rd.batteryJ > 1e7 ? 0.5 : 0.25;
                    box(c, Rz, outv * (rMid + s * 0.5) + Vec3d{0, 0, z0 + h * 0.5}, s, s * 1.4, s * 2.0, rd.color, em);
                    break;
                }
                case PK::Solar: {
                    double A = rd.solarArea;
                    double L = std::sqrt(A * 3.0), W = A / L;
                    {
                        FinScope sf(FIN_SOLAR);
                        box(c, Rz, outv * (rMid + 0.2 + L * 0.5) + Vec3d{0, 0, z0 + h * 0.5}, L, 0.03, W, rd.color, em);
                    }
                    box(c, Rz, outv * (rMid + 0.1) + Vec3d{0, 0, z0 + h * 0.5}, 0.25, 0.1, 0.1, Color{150, 150, 155, 255}, em);
                    break;
                }
                case PK::RCS: {
                    double s = rd.thrustVac > 100 ? 0.34 : 0.2;
                    Vec3d cpos = outv * (rMid + s * 0.5) + Vec3d{0, 0, z0 + h * 0.8};
                    box(c, Rz, cpos, s, s, s, rd.color, em);
                    Vec3d tang = Rz * Vec3d{0, 1, 0};
                    for (int f = 0; f < 4; f++) {
                        Vec3d e = f < 2 ? tang * (f == 0 ? 1.0 : -1.0) : Vec3d{0, 0, f == 2 ? 1.0 : -1.0};
                        box(c, Rz, cpos + e * (s * 0.5 + 0.03), s * 0.25, s * 0.25, s * 0.25, Color{40, 40, 44, 255}, em);
                    }
                    break;
                }
                case PK::RTG: {
                    Vec3d base = outv * (rMid + 0.45) + Vec3d{0, 0, z0 + h * 0.5 - 0.5};
                    piece(c, frustum(1, 1), toMatrix(Mat3d{}, base, 0.18, 0.18, 1.0), rd.color, em);
                    for (int f = 0; f < 4; f++)
                        box(c, rotZ(a + f * PI / 2), base + Vec3d{0, 0, 0.5}, 0.6, 0.02, 0.9, Color{70, 70, 74, 255}, em);
                    strut(c, outv * rMid + Vec3d{0, 0, z0 + h * 0.5}, base + Vec3d{0, 0, 0.5}, 0.04, Color{120, 120, 125, 255});
                    break;
                }
                case PK::Legs: {
                    Vec3d top = outv * rMid + Vec3d{0, 0, z0 + h * 0.6};
                    Vec3d foot = vv.legs ? outv * (rMid + 1.3) + Vec3d{0, 0, z0 - 1.6} : outv * (rMid + 0.15) + Vec3d{0, 0, z0 + 0.1};
                    strut(c, top, foot, 0.08, rd.color);
                    strut(c, outv * rMid + Vec3d{0, 0, z0 + 0.1}, foot * 0.5 + top * 0.5, 0.05, shade(rd.color, 0.8));
                    piece(c, frustum(1, 1), toMatrix(Mat3d{}, foot - Vec3d{0, 0, 0.05}, 0.35, 0.35, 0.1), shade(rd.color, 0.7), em);
                    break;
                }
                default: break;
                }
            }
        }
    }
    if (pass == 1) {
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        EndBlendMode();
        g_ghost = 1.0f;
    }
    } // (passes)
    // deployed parachute canopy
    if (vv.chute) {
        for (int i = 0; i < n; i++) {
            const PartDef& cd = CATALOG.defs[d.stack[i].def];
            if (cd.kind != PK::Parachute) continue;
            int nc = std::max(1, cd.canopies);
            drawCanopies(c, {0, 0, zb[i] + partGeom(d.stack[i]).height}, {0, 0, 1}, std::sqrt(4 * cd.chuteCdA / (PI * 0.85 * nc)), nc, 1.0);
            break;
        }
    }
    // payload fairing: translucent so that the payload inside stays visible while designing
    if (!shells.empty() || !ghostShells.empty()) {
        BeginBlendMode(BLEND_ALPHA);
        rlDisableDepthMask();
        for (auto& sh : ghostShells) {
            const Component& k = sh.first;
            double h = k.dims.z, mx = std::max({k.dims.x, k.dims.y, 1e-4});
            Matrix lm = MatrixMultiply(toMatrix(Mat3d{}, {0, 0, -h / 2}, mx, mx, h), toMatrix(k.rot().toMat(), k.pos));
            const auto& hm = halfShell(k.dims.x, k.dims.y);
            piece(c, hm.first, lm, Color{k.color.r, k.color.g, k.color.b, 24}, 0.f);
            piece(c, hm.second, lm, Color{170, 172, 176, 24}, 0.f);
        }
        for (auto& sh : shells) {
            const Component& k = sh.first;
            double h = k.dims.z, mx = std::max({k.dims.x, k.dims.y, 1e-4});
            Matrix lm = MatrixMultiply(toMatrix(Mat3d{}, {0, 0, -h / 2}, mx, mx, h), toMatrix(k.rot().toMat(), k.pos));
            const auto& hm = halfShell(k.dims.x, k.dims.y);
            unsigned char a = sh.second ? 150 : 72;
            piece(c, hm.first, lm, Color{k.color.r, k.color.g, k.color.b, a}, sh.second ? 0.3f : 0.f);
            piece(c, hm.second, lm, Color{170, 172, 176, a}, 0.f);
        }
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        EndBlendMode();
    }
    rlEnableBackfaceCulling();
    // plumes (additive)
    if (!plumes.empty()) {
        BeginBlendMode(BLEND_ADDITIVE);
        rlDisableDepthMask();
        rlDisableBackfaceCulling();
        for (const auto& pl : plumes) {
            for (int layer = 0; layer < 2; layer++) {
                double rr = pl.exitR * (layer == 0 ? 0.95 : 0.55);
                double L = pl.length * (layer == 0 ? 1.0 : 0.45);
                double fl = layer == 0 ? pl.flare : 1.0;
                // flare: scale XY grows along length; approximate with two segments
                Vector3 col = pl.col;
                float inten = pl.intensity * (layer == 0 ? 0.6f : 1.0f);
                plumeMat.maps[MATERIAL_MAP_DIFFUSE].color = Color{(unsigned char)(std::min(1.f, col.x) * 255),
                                                                  (unsigned char)(std::min(1.f, col.y) * 255),
                                                                  (unsigned char)(std::min(1.f, col.z) * 255),
                                                                  (unsigned char)(std::clamp(inten, 0.f, 1.f) * 255)};
                Matrix m = toMatrix(Mat3d{}, pl.pos, rr * (0.5 + 0.5 * fl), rr * (0.5 + 0.5 * fl), L);
                DrawMesh(plumeMesh, plumeMat, MatrixMultiply(m, c.base));
            }
        }
        rlEnableBackfaceCulling();
        rlEnableDepthMask();
        EndBlendMode();
    }
}

// ------------------------------------------------------------------ free-form crafts
namespace {
// craft-frame transform of a component (piece() applies the craft-to-world base matrix)
Matrix compMatrix(const Component& c) { return toMatrix(c.rot().toMat(), c.pos); }

// ---- wing panels (Shape::Airfoil): a tapered, swept panel with a symmetric 4-digit NACA section, in units of
// its box (span x, mean chord y, thickness z). The mean-chord box is what the physics uses; the outline is
// placed so that its area centroid along the chord lies at the box centre (where the lift acts).
struct AirfoilParams {
    double taper = 1, sweep = 0; // tip / root chord, leading-edge sweep [deg]
    int root = -1;               // root at local -x (-1) or +x (+1)
};
AirfoilParams g_airfoil;
std::map<long long, Mesh> airfoilCache;
const Mesh& airfoilMesh(double aspect, const AirfoilParams& ap) {
    double taper = std::clamp(ap.taper, 0.1, 1.0), sweep = std::clamp(ap.sweep, -80.0, 80.0);
    long long key = ((long long)std::lround(taper * 100) << 40) ^ ((long long)std::lround((sweep + 90) * 4) << 24) ^
                    ((long long)std::lround(std::min(aspect, 200.0) * 20) << 4) ^ (ap.root > 0 ? 1 : 0);
    auto it = airfoilCache.find(key);
    if (it != airfoilCache.end()) return it->second;
    const int NS = 9, NC = 14; // span stations, chord points per side
    double cr = 2 / (1 + taper), ct = taper * cr; // in units of the mean chord
    double tl = std::tan(sweep * DEG) * aspect;   // tip leading-edge shift [mean chords]
    auto chordAt = [&](double u) { return cr + (ct - cr) * u; };
    auto leAt = [&](double u) { return u * tl; };
    // chordwise centroid of the planform (leading edge of the root at 0): shift it onto the box centre
    double num = 0, den = 0;
    for (int i = 0; i < 64; i++) {
        double u = (i + 0.5) / 64, c = chordAt(u);
        num += c * (leAt(u) + c / 2);
        den += c;
    }
    double yShift = -num / den;
    auto thick = [](double x) { // half thickness of the section (x 0..1 from the leading edge), peak 0.5
        double t = 5 * (0.2969 * std::sqrt(std::max(x, 0.0)) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1036 * x * x * x * x);
        return t * (0.5 / 0.5); // (t = 1 gives a peak of ~0.5)
    };
    std::vector<Vector3> P, N;
    std::vector<Vector2> T;
    std::vector<unsigned short> I;
    // the section around: upper surface TE -> LE, lower LE -> TE (2 * NC + 1 points, cosine spacing)
    std::vector<double> xs;
    for (int k = 0; k <= NC; k++) xs.push_back(0.5 * (1 + std::cos(PI * k / NC))); // 1 .. 0
    int ring = 2 * NC + 1;
    for (int si = 0; si < NS; si++) {
        double u = (double)si / (NS - 1); // 0 root .. 1 tip
        double sx = ap.root > 0 ? 0.5 - u : u - 0.5;
        double c = chordAt(u), le = leAt(u);
        for (int k = 0; k < ring; k++) {
            double xc = k <= NC ? xs[k] : xs[2 * NC - k];
            double zt = thick(xc) * c * (k <= NC ? 1 : -1);
            P.push_back(Vector3{(float)sx, (float)(le + xc * c + yShift), (float)zt});
            N.push_back(Vector3{0, 0, 0});
            T.push_back(Vector2{(float)u, (float)k / (ring - 1)});
        }
    }
    auto tri = [&](int a, int b, int c) { I.push_back((unsigned short)a); I.push_back((unsigned short)b); I.push_back((unsigned short)c); };
    for (int si = 0; si + 1 < NS; si++)
        for (int k = 0; k + 1 < ring; k++) {
            int a = si * ring + k, b = a + 1, c = a + ring, d = c + 1;
            if (ap.root > 0) { tri(a, c, b); tri(b, c, d); }
            else { tri(a, b, c); tri(b, d, c); }
        }
    // end caps (root and tip): fans around the section centre
    for (int end = 0; end < 2; end++) {
        int si = end ? NS - 1 : 0, base = si * ring;
        Vector3 cen{0, 0, 0};
        for (int k = 0; k < ring; k++) { cen.x += P[base + k].x / ring; cen.y += P[base + k].y / ring; }
        int ci = (int)P.size();
        P.push_back(cen);
        N.push_back(Vector3{0, 0, 0});
        T.push_back(Vector2{0.5f, 0.5f});
        bool flip = (end == 1) != (ap.root > 0);
        for (int k = 0; k + 1 < ring; k++) flip ? tri(ci, base + k + 1, base + k) : tri(ci, base + k, base + k + 1);
    }
    // normals from the faces (in the unit box; the box scaling is applied by the model matrix: these are
    // corrected for it by the shader's normal matrix)
    for (size_t t = 0; t + 2 < I.size(); t += 3) {
        Vector3 a = P[I[t]], b = P[I[t + 1]], c = P[I[t + 2]];
        Vector3 n = Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a));
        for (int j = 0; j < 3; j++) N[I[t + j]] = Vector3Add(N[I[t + j]], n);
    }
    for (auto& n : N) n = Vector3Normalize(n);
    Mesh m = allocMesh((int)P.size(), (int)I.size() / 3);
    for (size_t i = 0; i < P.size(); i++) {
        m.vertices[i * 3] = P[i].x; m.vertices[i * 3 + 1] = P[i].y; m.vertices[i * 3 + 2] = P[i].z;
        m.normals[i * 3] = N[i].x; m.normals[i * 3 + 1] = N[i].y; m.normals[i * 3 + 2] = N[i].z;
        m.texcoords[i * 2] = T[i].x; m.texcoords[i * 2 + 1] = T[i].y;
    }
    for (size_t i = 0; i < I.size(); i++) m.indices[i] = I[i];
    UploadMesh(&m, false);
    if (airfoilCache.size() > 96) { // (old panels: the cache stays small)
        for (auto& kv : airfoilCache) UnloadMesh(kv.second);
        airfoilCache.clear();
    }
    return airfoilCache[key] = m;
}
// the planform of a panel (units of its box, as airfoilMesh): leading edge, chord and span position at u (0 root .. 1 tip)
struct AirfoilGeo {
    double cr, ct, tl, yShift;
    int root;
    AirfoilGeo(double aspect, const AirfoilParams& ap) {
        double taper = std::clamp(ap.taper, 0.1, 1.0), sweep = std::clamp(ap.sweep, -80.0, 80.0);
        cr = 2 / (1 + taper);
        ct = taper * cr;
        tl = std::tan(sweep * DEG) * aspect;
        root = ap.root;
        double num = 0, den = 0;
        for (int i = 0; i < 64; i++) {
            double u = (i + 0.5) / 64, c = chord(u);
            num += c * (le(u) + c / 2);
            den += c;
        }
        yShift = -num / den;
    }
    double chord(double u) const { return cr + (ct - cr) * u; }
    double le(double u) const { return u * tl; }
    double sx(double u) const { return root > 0 ? 0.5 - u : u - 0.5; }
    static double thick(double x) {
        return 5 * (0.2969 * std::sqrt(std::max(x, 0.0)) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1036 * x * x * x * x);
    }
    // a point of the section at span u, chord fraction xc, upper (+1) / lower (-1) surface or the camber line (0)
    Vector3 at(double u, double xc, int side) const {
        double c = chord(u);
        return Vector3{(float)sx(u), (float)(le(u) + xc * c + yShift), (float)(side * thick(xc) * c)};
    }
};
// a solid piece of a panel: span u0..u1 (from the root), chord fraction x0..x1, closed on every side (flat faces
// where it is cut: the wing's flap well, the flap's nose)
std::map<std::string, Mesh> airfoilPartCache;
const Mesh& airfoilPart(double aspect, const AirfoilParams& ap, double u0, double u1, double x0, double x1) {
    char kb[160];
    std::snprintf(kb, sizeof kb, "%.2f %.1f %.2f %d %.3f %.3f %.3f %.3f", std::clamp(ap.taper, 0.1, 1.0), std::clamp(ap.sweep, -80.0, 80.0),
                  std::min(aspect, 200.0), ap.root, u0, u1, x0, x1);
    auto it = airfoilPartCache.find(kb);
    if (it != airfoilPartCache.end()) return it->second;
    AirfoilGeo G(aspect, ap);
    const int NS = 7, NC = 10;
    std::vector<Vector3> P, N;
    std::vector<Vector2> T;
    std::vector<unsigned short> I;
    bool flip = ap.root > 0;
    auto tri = [&](int a, int b, int c) {
        if (flip) std::swap(b, c);
        I.push_back((unsigned short)a); I.push_back((unsigned short)b); I.push_back((unsigned short)c);
    };
    auto grid = [&](int rows, int cols, auto&& pt) { // rows along the span, a strip of cols points each
        int base = (int)P.size();
        for (int r = 0; r < rows; r++)
            for (int k = 0; k < cols; k++) {
                P.push_back(pt(r, k));
                N.push_back(Vector3{0, 0, 0});
                T.push_back(Vector2{(float)r / (rows - 1), (float)k / std::max(cols - 1, 1)});
            }
        for (int r = 0; r + 1 < rows; r++)
            for (int k = 0; k + 1 < cols; k++) {
                int a = base + r * cols + k, b = a + 1, c = a + cols, d = c + 1;
                tri(a, b, c);
                tri(b, d, c);
            }
    };
    auto us = [&](int r) { return u0 + (u1 - u0) * r / (NS - 1); };
    auto xs = [&](int k) { return x0 + (x1 - x0) * 0.5 * (1 - std::cos(PI * k / NC)); }; // x0 .. x1
    grid(NS, NC + 1, [&](int r, int k) { return G.at(us(r), xs(NC - k), 1); });   // upper: x1 -> x0
    grid(NS, NC + 1, [&](int r, int k) { return G.at(us(r), xs(k), -1); });       // lower: x0 -> x1
    if (x0 > 1e-3) grid(NS, 2, [&](int r, int k) { return G.at(us(r), x0, k ? -1 : 1); });  // flat front face
    if (x1 < 0.999) grid(NS, 2, [&](int r, int k) { return G.at(us(r), x1, k ? 1 : -1); }); // flat rear face
    // the two ends: fans around the section centre
    for (int end = 0; end < 2; end++) {
        double u = end ? u1 : u0;
        int ci = (int)P.size();
        Vector3 cen = G.at(u, (x0 + x1) / 2, 0);
        P.push_back(cen);
        N.push_back(Vector3{0, 0, 0});
        T.push_back(Vector2{0.5f, 0.5f});
        std::vector<Vector3> ring;
        for (int k = 0; k <= NC; k++) ring.push_back(G.at(u, xs(NC - k), 1));
        for (int k = 0; k <= NC; k++) ring.push_back(G.at(u, xs(k), -1));
        int b0 = (int)P.size();
        for (auto& p : ring) { P.push_back(p); N.push_back(Vector3{0, 0, 0}); T.push_back(Vector2{0.5f, 0.5f}); }
        int nr = (int)ring.size();
        for (int k = 0; k < nr; k++) {
            int a = b0 + k, b = b0 + (k + 1) % nr;
            if (end) tri(ci, a, b);
            else tri(ci, b, a);
        }
    }
    for (size_t t = 0; t + 2 < I.size(); t += 3) {
        Vector3 a = P[I[t]], b = P[I[t + 1]], c = P[I[t + 2]];
        Vector3 n = Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a));
        for (int j = 0; j < 3; j++) N[I[t + j]] = Vector3Add(N[I[t + j]], n);
    }
    for (auto& n : N) n = Vector3Normalize(n);
    Mesh m = allocMesh((int)P.size(), (int)I.size() / 3);
    for (size_t i = 0; i < P.size(); i++) {
        m.vertices[i * 3] = P[i].x; m.vertices[i * 3 + 1] = P[i].y; m.vertices[i * 3 + 2] = P[i].z;
        m.normals[i * 3] = N[i].x; m.normals[i * 3 + 1] = N[i].y; m.normals[i * 3 + 2] = N[i].z;
        m.texcoords[i * 2] = T[i].x; m.texcoords[i * 2 + 1] = T[i].y;
    }
    for (size_t i = 0; i < I.size(); i++) m.indices[i] = I[i];
    UploadMesh(&m, false);
    if (airfoilPartCache.size() > 96) {
        for (auto& kv : airfoilPartCache) UnloadMesh(kv.second);
        airfoilPartCache.clear();
    }
    return airfoilPartCache[kb] = m;
}

AirfoilParams airfoilOf(const Component& k);
// a wing panel with its flap: the wing with the flap's well cut out, the outer part (ailerons) whole, and the flap
// turned down about its hinge (the front of the flap, on the camber line) by deg toward the craft's belly
void drawFlappedWing(const DrawCtx& c, const Component& k, const Matrix& M, Color col, float em, double deg) {
    const Vec3d& d = k.dims;
    AirfoilParams ap = airfoilOf(k);
    double aspect = d.x / std::max(d.y, 1e-4), xh = 1 - FLAP_CHORD;
    Matrix S = toMatrix(Mat3d{}, {}, d.x, d.y, d.z);
    auto pc = [&](const Mesh& m, const Matrix& L) {
        g_uv = {(float)std::max(d.x, d.y), (float)d.z};
        g_scale = {(float)d.x, (float)d.y, (float)d.z};
        g_shapeK = 1;
        piece(c, m, L, col, em);
    };
    pc(airfoilPart(aspect, ap, 0, FLAP_SPAN, 0, xh), MatrixMultiply(S, M));
    pc(airfoilPart(aspect, ap, FLAP_SPAN, 1, 0, 1), MatrixMultiply(S, M));
    AirfoilGeo G(aspect, ap);
    Vector3 a0 = G.at(0, xh, 0), a1 = G.at(FLAP_SPAN, xh, 0);
    Vec3d h0{a0.x * d.x, a0.y * d.y, 0}, h1{a1.x * d.x, a1.y * d.y, 0};
    Vec3d ax = norm(h1 - h0);
    Mat3d R = k.rot().toMat();
    Vec3d want{0, 0, (R * Vec3d{0, 0, 1}).y >= 0 ? -1.0 : 1.0};
    double th = deg * DEG * (dot(cross(ax, Vec3d{0, 1, 0}), want) > 0 ? 1.0 : -1.0);
    Mat3d Rf = Quatd::axisAngle(ax, th).toMat();
    Matrix F = MatrixMultiply(MatrixMultiply(MatrixMultiply(S, toMatrix(Mat3d{}, h0 * -1.0)), toMatrix(Rf, h0)), M);
    pc(airfoilPart(aspect, ap, 0, FLAP_SPAN, xh + 0.004, 1), F);
}

// ---- payload bay doors: a part of a fuselage's section (superellipse, as loftMesh) from the sill at x = hw to
// the top of the section, over z0..z1, as a shell of the given thickness inset from the hull; vertices relative to
// the hinge on the sill (metres, the fuselage's frame); side +1: the right door (+x), -1 the left one
std::map<std::string, Mesh> doorCache;
const Mesh& doorMesh(double W, double H, double expo, double hw, double z0, double z1, double inset, double th, int side) {
    char kb[160];
    std::snprintf(kb, sizeof kb, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %d", W, H, expo, hw, z0, z1, inset, th, side);
    auto it = doorCache.find(kb);
    if (it != doorCache.end()) return it->second;
    double ch = std::pow(std::clamp(hw / (0.5 * W), 0.0, 0.999), 1.0 / expo);
    double thh = std::acos(ch);
    auto sec = [&](double t, double sh) { // the section point at angle t (thh .. pi/2), shrunk by sh metres
        double c = std::cos(t), s = std::sin(t);
        double x = 0.5 * W * std::pow(std::max(c, 0.0), expo), y = 0.5 * H * std::pow(std::max(s, 0.0), expo);
        double r = std::max(std::hypot(x, y), 1e-3), k = std::max(0.0, 1 - sh / r);
        return Vec3d{x * k, y * k, 0};
    };
    Vec3d hinge = sec(thh, 0);
    const int NA = 14;
    std::vector<Vector3> P, Nn;
    std::vector<Vector2> T;
    std::vector<unsigned short> I;
    auto add = [&](Vec3d p, double u, double v) {
        p = p - hinge;
        P.push_back(Vector3{(float)(p.x * side), (float)p.y, (float)p.z});
        Nn.push_back(Vector3{0, 0, 0});
        T.push_back(Vector2{(float)u, (float)v});
    };
    auto tri = [&](int a, int b, int c) {
        if (side < 0) std::swap(b, c);
        I.push_back((unsigned short)a); I.push_back((unsigned short)b); I.push_back((unsigned short)c);
    };
    for (int layer = 0; layer < 2; layer++) {
        int base = (int)P.size();
        double sh = inset + (layer ? th : 0.0);
        for (int a = 0; a <= NA; a++) {
            double t = thh + (PI / 2 - thh) * a / NA;
            Vec3d p = sec(t, sh);
            add(Vec3d{p.x, p.y, z0}, (double)a / NA, 0);
            add(Vec3d{p.x, p.y, z1}, (double)a / NA, 1);
        }
        for (int a = 0; a < NA; a++) {
            int p0 = base + a * 2, p1 = p0 + 2;
            if (layer == 0) { tri(p0, p1, p0 + 1); tri(p1, p1 + 1, p0 + 1); }
            else { tri(p0, p0 + 1, p1); tri(p1, p0 + 1, p1 + 1); }
        }
    }
    // the edges of the shell (both ends and the long edges)
    auto quad = [&](Vec3d a, Vec3d b, Vec3d c2, Vec3d d) {
        int i0 = (int)P.size();
        add(a, 0, 0); add(b, 1, 0); add(c2, 1, 1); add(d, 0, 1);
        tri(i0, i0 + 1, i0 + 2);
        tri(i0, i0 + 2, i0 + 3);
    };
    for (int a = 0; a < NA; a++) {
        double t0 = thh + (PI / 2 - thh) * a / NA, t1 = thh + (PI / 2 - thh) * (a + 1) / NA;
        for (int e = 0; e < 2; e++) {
            double z = e ? z1 : z0;
            Vec3d o0 = sec(t0, inset), o1 = sec(t1, inset), i0 = sec(t0, inset + th), i1 = sec(t1, inset + th);
            if (e) quad({o0.x, o0.y, z}, {o1.x, o1.y, z}, {i1.x, i1.y, z}, {i0.x, i0.y, z});
            else quad({o1.x, o1.y, z}, {o0.x, o0.y, z}, {i0.x, i0.y, z}, {i1.x, i1.y, z});
        }
    }
    for (int e = 0; e < 2; e++) {
        double t = e ? PI / 2 : thh;
        Vec3d o = sec(t, inset), i = sec(t, inset + th);
        if (e) quad({o.x, o.y, z0}, {o.x, o.y, z1}, {i.x, i.y, z1}, {i.x, i.y, z0});
        else quad({o.x, o.y, z1}, {o.x, o.y, z0}, {i.x, i.y, z0}, {i.x, i.y, z1});
    }
    for (size_t t = 0; t + 2 < I.size(); t += 3) {
        Vector3 a = P[I[t]], b = P[I[t + 1]], c = P[I[t + 2]];
        Vector3 n = Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a));
        for (int j = 0; j < 3; j++) Nn[I[t + j]] = Vector3Add(Nn[I[t + j]], n);
    }
    for (auto& n : Nn) n = Vector3Normalize(n);
    Mesh m = allocMesh((int)P.size(), (int)I.size() / 3);
    for (size_t i = 0; i < P.size(); i++) {
        m.vertices[i * 3] = P[i].x; m.vertices[i * 3 + 1] = P[i].y; m.vertices[i * 3 + 2] = P[i].z;
        m.normals[i * 3] = Nn[i].x; m.normals[i * 3 + 1] = Nn[i].y; m.normals[i * 3 + 2] = Nn[i].z;
        m.texcoords[i * 2] = T[i].x; m.texcoords[i * 2 + 1] = T[i].y;
    }
    for (size_t i = 0; i < I.size(); i++) m.indices[i] = I[i];
    UploadMesh(&m, false);
    if (doorCache.size() > 64) {
        for (auto& kv : doorCache) UnloadMesh(kv.second);
        doorCache.clear();
    }
    return doorCache[kb] = m;
}

// ---- streamlined bodies (Shape::Loft) in their unit box (-0.5..0.5 on each axis; z = length, +z the nose)
struct LoftParams {
    double nose = 0.3, tail = 0.35, tailR = 0.3, up = 0, bluff = 0.5, boxy = 0;
};
LoftParams g_loft;
LoftParams loftOf(const Component& k) { return {k.noseLen, k.tailLen, k.tailR, k.upsweep, k.bluff, k.boxy}; }
std::map<long long, Mesh> loftCache;
const Mesh& loftMesh(const LoftParams& lp) {
    auto q = [](double v, double lo, double hi) { return (long long)std::lround((std::clamp(v, lo, hi) - lo) / (hi - lo) * 63); };
    long long key = q(lp.nose, 0.02, 0.6) | q(lp.tail, 0.02, 0.7) << 6 | q(lp.tailR, 0, 1) << 12 | q(lp.up, -1, 1) << 18 | q(lp.bluff, 0, 1) << 24 |
                    q(lp.boxy, 0, 1) << 30;
    auto it = loftCache.find(key);
    if (it != loftCache.end()) return it->second;
    const int NZ = 44, NA = 36;
    double nose = std::clamp(lp.nose, 0.02, 0.6), tail = std::clamp(lp.tail, 0.02, 0.7);
    if (nose + tail > 0.98) { double k = 0.98 / (nose + tail); nose *= k; tail *= k; }
    double expo = 2.0 / (2.0 + 2.5 * std::clamp(lp.boxy, 0.0, 1.0)); // superellipse: |c|^(2/n)
    // stations: denser toward both ends (cosine spacing), s = 0 tail end .. 1 nose tip
    std::vector<double> ss;
    for (int i = 0; i <= NZ; i++) ss.push_back(0.5 - 0.5 * std::cos(PI * i / NZ));
    auto radius = [&](double sv, double& yc) {
        yc = 0;
        if (sv > 1 - nose) { // nose: from round (ellipse) to pointed (parabola) by the bluntness
            double t = (1 - sv) / nose; // 0 at the tip
            double e = 1.0 - 0.5 * std::clamp(lp.bluff, 0.0, 1.0);
            return std::pow(std::max(0.0, 1 - (1 - t) * (1 - t)), e);
        }
        if (sv < tail) { // tail cone narrowing to the tail end, its centre line rising (upsweep)
            double t = sv / tail; // 0 at the tail end
            double sm = t * t * (3 - 2 * t);
            yc = lp.up * 0.5 * (1 - sm) * (1 - std::clamp(lp.tailR, 0.0, 1.0) * 0.5);
            return lp.tailR + (1 - lp.tailR) * std::pow(sm, 0.75);
        }
        return 1.0;
    };
    std::vector<Vector3> P, N;
    std::vector<Vector2> T;
    std::vector<unsigned short> I;
    for (int i = 0; i <= NZ; i++) {
        double yc, f = radius(ss[i], yc), z = ss[i] - 0.5;
        for (int a = 0; a < NA; a++) {
            double th = a * TAU / NA, c = std::cos(th), sn = std::sin(th);
            double x = 0.5 * f * (c < 0 ? -1 : 1) * std::pow(std::fabs(c), expo);
            double y = 0.5 * f * (sn < 0 ? -1 : 1) * std::pow(std::fabs(sn), expo) * (1 - std::fabs(yc)) + yc * 0.5;
            P.push_back(Vector3{(float)x, (float)y, (float)z});
            N.push_back(Vector3{0, 0, 0});
            T.push_back(Vector2{(float)a / NA, (float)ss[i]});
        }
    }
    auto tri = [&](int a, int b, int c) { I.push_back((unsigned short)a); I.push_back((unsigned short)b); I.push_back((unsigned short)c); };
    for (int i = 0; i < NZ; i++)
        for (int a = 0; a < NA; a++) {
            int p0 = i * NA + a, p1 = i * NA + (a + 1) % NA, p2 = p0 + NA, p3 = p1 + NA;
            tri(p0, p1, p2);
            tri(p1, p3, p2);
        }
    // the tail end: a cap when it is open
    {
        int ci = (int)P.size();
        double yc;
        radius(0.0, yc);
        P.push_back(Vector3{0, (float)(yc * 0.5), -0.5f});
        N.push_back(Vector3{0, 0, 0});
        T.push_back(Vector2{0.5f, 0});
        for (int a = 0; a < NA; a++) tri(ci, (a + 1) % NA, a);
    }
    for (size_t t = 0; t + 2 < I.size(); t += 3) {
        Vector3 a = P[I[t]], b = P[I[t + 1]], c = P[I[t + 2]];
        Vector3 n = Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a));
        for (int j = 0; j < 3; j++) N[I[t + j]] = Vector3Add(N[I[t + j]], n);
    }
    for (auto& n : N) n = Vector3Length(n) > 1e-12f ? Vector3Normalize(n) : Vector3{0, 0, 1};
    Mesh m = allocMesh((int)P.size(), (int)I.size() / 3);
    for (size_t i = 0; i < P.size(); i++) {
        m.vertices[i * 3] = P[i].x; m.vertices[i * 3 + 1] = P[i].y; m.vertices[i * 3 + 2] = P[i].z;
        m.normals[i * 3] = N[i].x; m.normals[i * 3 + 1] = N[i].y; m.normals[i * 3 + 2] = N[i].z;
        m.texcoords[i * 2] = T[i].x; m.texcoords[i * 2 + 1] = T[i].y;
    }
    for (size_t i = 0; i < I.size(); i++) m.indices[i] = I[i];
    UploadMesh(&m, false);
    if (loftCache.size() > 96) {
        for (auto& kv : loftCache) UnloadMesh(kv.second);
        loftCache.clear();
    }
    return loftCache[key] = m;
}
// the root end of a panel: the one toward the craft axis (its local +x pointing outward -> root at -x)
AirfoilParams airfoilOf(const Component& k) {
    AirfoilParams a;
    a.taper = k.taper;
    a.sweep = k.sweep;
    a.root = airfoilRoot(k); // (craft.cpp: unless fixed, the end toward the craft axis)
    return a;
}

// a truss as one mesh: every member a six-sided tube (one draw call for the whole lattice), cached by size
std::map<long long, Mesh> trussCache;
const Mesh& trussMesh(const Vec3d& d) {
    auto q = [](double v) { return (long long)std::lround(std::clamp(v, 0.0, 2000.0) * 100); }; // (cm)
    long long key = q(d.x) | q(d.y) << 18 | q(d.z) << 36;
    auto it = trussCache.find(key);
    if (it != trussCache.end()) return it->second;
    std::vector<std::pair<Vec3d, Vec3d>> mem;
    trussMembers(d, mem);
    double r = trussTubeRadius(d);
    const int S = 6;
    int maxMem = 65000 / (S * 2);
    if ((int)mem.size() > maxMem) mem.resize(maxMem); // (16-bit indices)
    if (trussCache.size() > 64) {
        for (auto& kv : trussCache) UnloadMesh(kv.second);
        trussCache.clear();
    }
    Mesh m = allocMesh((int)mem.size() * S * 2, (int)mem.size() * S * 2);
    int v = 0, t = 0;
    for (auto& e : mem) {
        Vec3d a = e.first, b = e.second, ax = b - a;
        double l = len(ax);
        if (l < 1e-9) ax = {0, 0, 1};
        else ax = ax / l;
        Vec3d p = anyPerp(ax), p2 = cross(ax, p);
        int base = v;
        for (int i = 0; i < S; i++) {
            double ang = TAU * i / S;
            Vec3d n = p * std::cos(ang) + p2 * std::sin(ang);
            for (int k = 0; k < 2; k++) {
                Vec3d pt = (k ? b : a) + n * r;
                m.vertices[v * 3] = (float)pt.x; m.vertices[v * 3 + 1] = (float)pt.y; m.vertices[v * 3 + 2] = (float)pt.z;
                m.normals[v * 3] = (float)n.x; m.normals[v * 3 + 1] = (float)n.y; m.normals[v * 3 + 2] = (float)n.z;
                m.texcoords[v * 2] = (float)i / S; m.texcoords[v * 2 + 1] = (float)k * (float)l;
                v++;
            }
        }
        for (int i = 0; i < S; i++) {
            int i0 = base + i * 2, i1 = base + ((i + 1) % S) * 2;
            unsigned short* ix = m.indices + t * 3;
            ix[0] = (unsigned short)i0; ix[1] = (unsigned short)(i1); ix[2] = (unsigned short)(i0 + 1);
            ix[3] = (unsigned short)(i1); ix[4] = (unsigned short)(i1 + 1); ix[5] = (unsigned short)(i0 + 1);
            t += 2;
        }
    }
    UploadMesh(&m, false);
    return trussCache[key] = m;
}

void shapePiece(const DrawCtx& c, Shape s, const Vec3d& d, const Matrix& M, Color col, float em) {
    g_uv = s == Shape::Box || s == Shape::Wedge || s == Shape::Airfoil || s == Shape::Loft ? Vector2{(float)std::max(d.x, d.y), (float)d.z}
                                                : Vector2{(float)(TAU * std::max(d.x, d.y)), (float)(s == Shape::Ellipsoid ? PI * d.z : d.z)};
    // the shader's object coordinates in metres (planar-mapped finishes: tiles, concrete)
    if (s == Shape::Cylinder) {
        float mx = (float)std::max(std::max(d.x, d.y), 1e-4);
        g_scale = {mx, mx, (float)d.z};
    } else if (s != Shape::Truss) g_scale = {(float)d.x, (float)d.y, (float)d.z};
    g_shapeK = s == Shape::Airfoil ? 1 : s == Shape::Loft ? 2 : 0;
    switch (s) {
    case Shape::Box: piece(c, cubeMesh, MatrixMultiply(toMatrix(Mat3d{}, {}, d.x, d.y, d.z), M), col, em); break;
    case Shape::Cylinder: {
        double mx = std::max(std::max(d.x, d.y), 1e-4);
        piece(c, frustum(d.x, d.y), MatrixMultiply(toMatrix(Mat3d{}, {0, 0, -d.z / 2}, mx, mx, d.z), M), col, em);
        break;
    }
    case Shape::Ellipsoid: piece(c, sphereMesh, MatrixMultiply(toMatrix(Mat3d{}, {}, d.x, d.y, d.z), M), col, em); break;
    case Shape::Wedge: piece(c, wedgeMesh, MatrixMultiply(toMatrix(Mat3d{}, {}, d.x, d.y, d.z), M), col, em); break;
    case Shape::Airfoil:
        piece(c, airfoilMesh(d.x / std::max(d.y, 1e-4), g_airfoil), MatrixMultiply(toMatrix(Mat3d{}, {}, d.x, d.y, d.z), M), col, em);
        break;
    case Shape::Loft: piece(c, loftMesh(g_loft), MatrixMultiply(toMatrix(Mat3d{}, {}, d.x, d.y, d.z), M), col, em); break;
    case Shape::Truss:
        g_uv = {1, 1};
        piece(c, trussMesh(d), M, col, em); // (built in metres: no scaling)
        break;
    default: break;
    }
}
// local-space frustum helper for detailed functional visuals (z0 = bottom)
void lcyl(const DrawCtx& c, const Matrix& M, double z0, double h, double rb, double rt, Color col, double ox = 0, double oy = 0, float em = 0) {
    g_uv = {(float)(TAU * std::max(rb, rt)), (float)h};
    double mx = std::max(std::max(rb, rt), 1e-4);
    piece(c, frustum(rb, rt), MatrixMultiply(toMatrix(Mat3d{}, {ox, oy, z0}, mx, mx, h), M), col, em);
}
float smooth01(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return (float)(t * t * (3 - 2 * t));
}
} // namespace
// a solar panel between its folded stack (t = 0) and its deployed wing (t = 1): the stack first swings
// out from the side of the body about its root, then the accordion opens to the full span
void solarPose(const Component& dep, const Component& sto, float t, Mat3d& R, Vec3d& pos, Vec3d& dims) {
    Mat3d Rd = dep.rot().toMat();
    // (the same test as the stowing: a panel folded in place towards its inner end, however it is turned, opens
    // from that end; taking it for a radial stack would hinge it on its local -x end, out in the open)
    bool radial = solarRadialMount(dep) && sto.dims.x < dep.dims.x * 0.6;
    if (!radial) { // generic panel: blend size and place (the edge at the hinge stays put)
        double s = smooth01(t);
        R = Rd;
        pos = sto.pos + (dep.pos - sto.pos) * s;
        dims = sto.dims + (dep.dims - sto.dims) * s;
        return;
    }
    Vec3d e = Rd * Vec3d{1, 0, 0};
    Vec3d hinge = dep.pos - e * (dep.dims.x / 2);
    double thick = sto.dims.x, Ls = sto.dims.y, Ws = sto.dims.z;
    const double split = 0.35;
    if (t < split) {
        double a = smooth01(t / split);
        R = Rd * rotZ((1 - a) * PI / 2); // local x: along the stack face; turns from tangential to radial
        dims = {Ls, thick, Ws};
        pos = sto.pos + (hinge + e * (Ls / 2) - sto.pos) * a;
    } else {
        double b = smooth01((t - split) / (1 - split));
        R = Rd;
        double span = Ls + (dep.dims.x - Ls) * b;
        dims = {span, thick + (dep.dims.y - thick) * b, Ws + (dep.dims.z - Ws) * b};
        pos = hinge + e * (span / 2);
    }
}
namespace {

struct CraftPlume {
    Plume p;
    Mat3d R; // component orientation (plume along -Z)
    bool wake = false; // entry plasma: widening cone instead of an exhaust column
    int shape = 0;     // 0 exhaust / wake cone (by 'wake'), 1 rim sheath, 2 shock cap (ellipsoid)
    float limb = 0;    // 1: bright silhouette, clear face-on (hollow glowing shell)
    double capZ = 0;   // shock cap: half depth along the flow
};
} // namespace

// pad layout (pad frame): hardstand size and flood-light heads
static double padHardstand(const PadSpec& ps) {
    double R = std::max(0.5, ps.rocketR), towerDist = R + 9;
    return std::max(80.0, 2 * (towerDist + 24));
}
int padFloodlights(const PadSpec& ps, Vec3d pos[4], Vec3d dir[4]) {
    double S = padHardstand(ps);
    Vec3d aim{0, 0, std::max(8.0, ps.deckH + ps.rocketLen * 0.45)};
    for (int k = 0; k < 4; k++) {
        Vec3d b{(k & 1 ? 1 : -1) * (S / 2 + 8), (k & 2 ? 1 : -1) * (S / 2 + 8), 0};
        pos[k] = b + norm(Vec3d{-b.x, -b.y, 0}) * 0.9 + Vec3d{0, 0, 34.8};
        dir[k] = norm(aim - pos[k]);
    }
    return 4;
}

void drawLaunchPad(const PadSpec& ps, const Mat3d& rot, const Vec3d& origin, const Vec3d& lightDir, float light, const Vec3d& camPos,
                   float ambient) {
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
    const double H = ps.deckH, R = std::max(0.5, ps.rocketR);
    const double slotW = std::max(8.0, 2 * R + 6);          // flame trench (runs east-west under the vehicle)
    const double towerDist = R + 9, towerHalf = 3.0;         // umbilical tower on the north side
    const double S = padHardstand(ps);                       // hardstand size
    const Color concrete{168, 166, 160, 255}, darkConc{70, 68, 64, 255}, steel{122, 126, 132, 255}, red{170, 48, 36, 255};
    {
        FinScope f(FIN_CONCRETE);
        // apron and the raised hardstand split by the trench
        cyl(c, -0.2, 0.4, S * 1.6, S * 1.6, Color{132, 132, 128, 255});
        double half = (S - slotW) / 2;
        box(c, Mat3d{}, {0, slotW / 2 + half / 2, H / 2}, S, half, H, concrete);
        box(c, Mat3d{}, {0, -(slotW / 2 + half / 2), H / 2}, S, half, H, concrete);
        // ramps down to the apron on the east and west ends of the trench
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            Mat3d tilt = rotY(sgn * 0.35);
            box(c, tilt, {sgn * (S / 2 + 9), slotW / 2 + half / 2, H * 0.42}, 22, half, H * 0.6, concrete);
            box(c, tilt, {sgn * (S / 2 + 9), -(slotW / 2 + half / 2), H * 0.42}, 22, half, H * 0.6, concrete);
        }
        box(c, Mat3d{}, {0, 0, 0.35}, S + 40, slotW, 0.5, darkConc); // trench floor
        // inverted-V flame deflector under the engines
        for (int sgn = -1; sgn <= 1; sgn += 2)
            box(c, rotY(sgn * 0.62), {sgn * (H * 0.42), 0, H * 0.42}, H * 1.25, slotW * 0.82, 0.7, Color{110, 104, 96, 255});
    }
    {
        // runway south of the complex (aircraft start here): asphalt, centre line, threshold bars
        FinScope f(FIN_CONCRETE);
        double cx = RUNWAY_START_X + RUNWAY_LEN / 2 - 150;
        box(c, Mat3d{}, {cx, RUNWAY_Y, 0.12}, RUNWAY_LEN + 300, RUNWAY_W, 0.24, Color{62, 64, 66, 255});
        for (double x = RUNWAY_START_X + 120; x < RUNWAY_START_X + RUNWAY_LEN - 120; x += 60)
            box(c, Mat3d{}, {x, RUNWAY_Y, 0.25}, 30, 0.9, 0.02, Color{225, 225, 220, 255});
        for (int e = 0; e < 2; e++) {
            double x = e == 0 ? RUNWAY_START_X - 60 : RUNWAY_START_X + RUNWAY_LEN + 20;
            for (int k = -4; k <= 4; k++)
                if (k != 0) box(c, Mat3d{}, {x, RUNWAY_Y + k * 4.4, 0.25}, 30, 1.8, 0.02, Color{225, 225, 220, 255});
        }
        // taxiway to the apron
        box(c, Mat3d{}, {RUNWAY_START_X + 200, RUNWAY_Y / 2, 0.1}, 23, -RUNWAY_Y - RUNWAY_W, 0.2, Color{70, 72, 74, 255});
    }
    {
        FinScope f(FIN_METAL);
        // launch mount: a frame around the exhaust hole, on four legs standing in the trench
        double a = R * 1.25 + 0.4, bw = 1.6, top = H;
        box(c, Mat3d{}, {0, a + bw / 2, top - 0.7}, 2 * (a + bw), bw, 1.4, steel);
        box(c, Mat3d{}, {0, -(a + bw / 2), top - 0.7}, 2 * (a + bw), bw, 1.4, steel);
        box(c, Mat3d{}, {a + bw / 2, 0, top - 0.7}, bw, 2 * a, 1.4, steel);
        box(c, Mat3d{}, {-(a + bw / 2), 0, top - 0.7}, bw, 2 * a, 1.4, steel);
        for (int k = 0; k < 4; k++) {
            double x = (k & 1 ? 1 : -1) * (a + bw / 2), y = (k & 2 ? 1 : -1) * (a + bw / 2);
            box(c, Mat3d{}, {x, y, (top - 1.4) / 2}, 1.2, 1.2, top - 1.4, Color{104, 108, 114, 255});
        }
        // hold-down clamps around the vehicle base
        for (int k = 0; k < 4; k++) {
            Mat3d Rz = rotZ(PI / 4 + k * PI / 2);
            box(c, Rz, Rz * Vec3d{R + 0.25, 0, top + 0.5}, 0.7, 0.5, 1.0, Color{60, 62, 66, 255});
        }
        // umbilical tower: four columns with horizontal and diagonal bracing
        double tH = ps.rocketLen + 14, ty = towerDist + towerHalf;
        Vec3d cols[4] = {{-towerHalf, ty - towerHalf, H}, {towerHalf, ty - towerHalf, H}, {towerHalf, ty + towerHalf, H}, {-towerHalf, ty + towerHalf, H}};
        for (auto& p : cols) strut(c, p, p + Vec3d{0, 0, tH}, 0.32, red);
        const double step = 5.0;
        int levels = (int)(tH / step);
        for (int l = 0; l <= levels; l++) {
            double z = H + l * step;
            for (int k = 0; k < 4; k++) {
                Vec3d p0 = cols[k], p1 = cols[(k + 1) % 4];
                p0.z = p1.z = z;
                strut(c, p0, p1, 0.16, red);
                if (l < levels) {
                    Vec3d q = (l + k) % 2 ? p0 : p1, r2 = (l + k) % 2 ? p1 : p0;
                    r2.z += step;
                    strut(c, q, r2, 0.12, red);
                }
            }
        }
        box(c, Mat3d{}, {0, ty, H + tH / 2}, 2.2, 2.2, tH, Color{96, 100, 106, 255}); // elevator shaft
        box(c, Mat3d{}, {0, ty, H + tH + 1.2}, 2 * towerHalf + 1, 2 * towerHalf + 1, 2.4, Color{200, 202, 206, 255});
        // lightning mast and obstruction lights
        strut(c, {0, ty, H + tH + 2.4}, {0, ty, H + tH + 22}, 0.35, Color{220, 222, 226, 255});
        for (int k = 0; k < 4; k++) box(c, Mat3d{}, cols[k] + Vec3d{0, 0, tH + 0.4}, 0.6, 0.6, 0.6, Color{255, 40, 30, 255}, 1.2f);
        box(c, Mat3d{}, {0, ty, H + tH + 22.3}, 0.7, 0.7, 0.7, Color{255, 40, 30, 255}, 1.2f);
        // swing arms (crew / upper stage / interstage); they rotate away at ignition
        const double fr[3] = {0.45, 0.72, 0.93};
        double hingeY = ty - towerHalf, L = std::max(1.0, hingeY - (R + 0.3));
        double ang = std::clamp(ps.armRetract, 0.0, 1.0) * 1.45;
        for (int k = 0; k < 3; k++) {
            double z = H + ps.rocketLen * fr[k];
            if (z > H + tH - 2) continue;
            Vec3d hinge{-towerHalf + 0.8, hingeY, z};
            Mat3d Rz = rotZ(ang);
            Vec3d mid = hinge + Rz * Vec3d{0.8, -L / 2, 0};
            box(c, Rz, mid, 1.6, L, 1.4, Color{150, 154, 160, 255});
            Vec3d tip = hinge + Rz * Vec3d{0.8, -L, 0};
            strut(c, hinge + Vec3d{0, 0, 2.2}, tip + Vec3d{0, 0, 0.7}, 0.08, Color{90, 94, 100, 255}); // support cable
            if (k == 2) box(c, Rz, hinge + Rz * Vec3d{0.8, -L + 1.2, 0.9}, 2.8, 2.4, 3.2, Color{226, 228, 232, 255}); // white room
        }
        // flood-light masts at the corners of the hardstand
        for (int k = 0; k < 4; k++) {
            Vec3d b{(k & 1 ? 1 : -1) * (S / 2 + 8), (k & 2 ? 1 : -1) * (S / 2 + 8), 0};
            strut(c, b, b + Vec3d{0, 0, 34}, 0.4, Color{150, 152, 156, 255});
            Vec3d toPad = norm(Vec3d{-b.x, -b.y, 0});
            Mat3d Rz = rotZ(std::atan2(toPad.y, toPad.x));
            box(c, Rz, b + Vec3d{0, 0, 34.8}, 0.8, 3.2, 1.6, Color{70, 72, 76, 255});
            box(c, Rz, b + toPad * 0.45 + Vec3d{0, 0, 34.8}, 0.12, 3.0, 1.4, Color{255, 246, 220, 255}, 0.9f + 2.5f * ps.night);
        }
    }
    {
        // water tower for the sound-suppression deluge
        FinScope f(FIN_PAINT);
        Vec3d wt{S / 2 + 70, -S / 2 - 30, 0};
        for (int k = 0; k < 4; k++) {
            Vec3d foot = wt + Vec3d{(k & 1 ? 1 : -1) * 6.0, (k & 2 ? 1 : -1) * 6.0, 0};
            strut(c, foot, wt + Vec3d{(k & 1 ? 1 : -1) * 3.0, (k & 2 ? 1 : -1) * 3.0, 34}, 0.45, Color{200, 202, 206, 255});
        }
        g_uv = {(float)(TAU * 8), (float)(PI * 7)};
        piece(c, sphereMesh, toMatrix(Mat3d{}, wt + Vec3d{0, 0, 40}, 8, 8, 7), Color{236, 238, 240, 255});
    }
    if (ps.night > 0.01f) {
        // lamp glare
        Vec3d lp[4], ld[4];
        int n = padFloodlights(ps, lp, ld);
        BeginBlendMode(BLEND_ADDITIVE);
        rlDisableDepthMask();
        for (int k = 0; k < n; k++) {
            Vector3 w = toF(origin + rot * lp[k]);
            facingBillboard(glowTex, w, 9.0f, Color{255, 236, 200, (unsigned char)(200 * ps.night)});
            facingBillboard(glowTex, w, 30.0f, Color{255, 220, 170, (unsigned char)(60 * ps.night)});
        }
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        EndBlendMode();
    }
    if (!g_shadowPass) rlEnableBackfaceCulling();
}

#include "render_base.inc" // surface base buildings (drawBase)

// the surface finish of a component
int finishOf(const Component& k) {
    const PartDef* def = (k.def >= 0 && k.def < (int)CATALOG.defs.size()) ? &CATALOG.defs[k.def] : nullptr;
    switch (k.func) {
    case Func::Tank: return finishForProp(k.prop);
    case Func::Structure:
    case Func::Wing:
    case Func::HeatShield: return finishForMaterial(k.material);
    case Func::Engine:
    case Func::Legs:
    case Func::RCS:
    case Func::Rotor:
    case Func::Jet:
    case Func::Wheel:
    case Func::RTG:
    case Func::Reactor:
    case Func::CMG:
    case Func::Hyperdrive: return FIN_METAL;
    case Func::Solar: return FIN_SOLAR;
    case Func::Sail: return FIN_FOIL;
    case Func::Command: return def && def->id == "probe_bus" ? FIN_FOIL : FIN_PAINT;
    case Func::Window: return FIN_GLASS;
    case Func::CargoBay: return FIN_TILES;
    default: return FIN_PAINT;
    }
}

// a payload bay: the liner of the bay, its sills, the two doors (from the fuselage around it, hinged on the sills,
// up to 175 degrees open) with their radiators (the forward panels stand off the doors by 35.5 degrees once open)
void drawBay(const DrawCtx& c, const std::vector<Component>& comps, int bi, int host, double open, float em) {
    const Component& b = comps[bi];
    Vec3d he = b.dims * 0.5;
    Matrix Mb = compMatrix(b);
    const Color liner{226, 226, 222, 255}, sill{120, 124, 130, 255}, rad{232, 234, 236, 255};
    auto bx = [&](Vec3d p, Vec3d s, Color col, int fin) {
        FinScope f(fin);
        g_uv = {(float)std::max(s.x, s.y), (float)s.z};
        g_scale = {(float)s.x, (float)s.y, (float)s.z};
        piece(c, cubeMesh, MatrixMultiply(toMatrix(Mat3d{}, p, s.x, s.y, s.z), Mb), col, em);
    };
    if (open <= 1e-3) return; // (closed: the hull covers it)
    // liner: floor, side walls, bulkheads (white blankets), the sill longerons
    bx({0, -he.y, 0}, {2 * he.x, 0.04, 2 * he.z}, liner, FIN_TILES);
    for (int s = -1; s <= 1; s += 2) {
        bx({s * he.x, 0, 0}, {0.04, 2 * he.y, 2 * he.z}, liner, FIN_TILES);
        bx({0, 0, s * he.z}, {2 * he.x, 2 * he.y, 0.04}, liner, FIN_TILES);
        bx({s * (he.x - 0.08), he.y - 0.08, 0}, {0.16, 0.16, 2 * he.z}, sill, FIN_METAL);
    }
    for (int r = 1; r < 12; r++) bx({0, -he.y + 0.06, -he.z + 2 * he.z * r / 12}, {2 * he.x - 0.1, 0.08, 0.1}, sill, FIN_METAL); // floor frames
    {
        // the remote manipulator arm stowed on the left (+X) sill: shoulder at the forward end, upper and lower
        // booms (38 cm, white thermal blankets), elbow, wrist and end effector toward the aft end (15.2 m in all)
        double L = std::min(15.2, 2 * he.z - 0.6), xa = he.x - 0.32, ya = he.y + 0.12, z0 = he.z - 0.3;
        const Color armW{232, 232, 228, 255}, joint{70, 72, 78, 255};
        auto seg = [&](double za, double zb, double r, Color col) {
            FinScope f(FIN_TILES);
            Vec3d a{xa, ya, za};
            g_uv = {(float)(TAU * r), (float)std::fabs(zb - za)};
            g_scale = {(float)r, (float)r, (float)std::fabs(zb - za)};
            piece(c, frustum(1, 1), MatrixMultiply(toMatrix(rotX(PI), a, r, r, za - zb), Mb), col, em);
        };
        double zs = z0, ze = z0 - 0.5, zel = z0 - 0.5 - L * 0.42, zw = zel - L * 0.46;
        seg(zs, ze, 0.26, joint);           // shoulder joints
        seg(ze, zel, 0.19, armW);           // upper boom
        seg(zel, zel - 0.45, 0.24, joint);  // elbow
        seg(zel - 0.45, zw, 0.19, armW);    // lower boom
        seg(zw, zw - 0.6, 0.22, joint);     // wrist
        seg(zw - 0.6, zw - 1.0, 0.2, armW); // end effector
    }
    double ang = open * 175.0 * DEG;
    double dep = clampd((open - 0.85) / 0.15, 0, 1) * 35.5 * DEG;
    if (host >= 0) {
        // doors shaped after the fuselage: the bay in the fuselage's frame
        const Component& L = comps[host];
        Mat3d RL = L.rot().toMat();
        Vec3d bl = RL.T() * (b.pos - L.pos);
        double expo = 2.0 / (2.0 + 2.5 * std::clamp(L.boxy, 0.0, 1.0));
        double W = L.dims.x, H = L.dims.y, hw = std::min(he.x, 0.49 * W);
        double ch = std::pow(std::clamp(hw / (0.5 * W), 0.0, 0.999), 1.0 / expo);
        double ys = 0.5 * H * std::pow(std::sqrt(std::max(0.0, 1 - ch * ch)), expo);
        double z0 = bl.z - he.z, z1 = bl.z + he.z, zm = (z0 + z1) / 2;
        Matrix ML = compMatrix(L);
        for (int side = -1; side <= 1; side += 2) {
            Vec3d hinge{side * hw, ys, 0};
            Matrix Md = MatrixMultiply(toMatrix(rotZ(-side * ang), hinge), ML);
            {
                FinScope f(FIN_TILES);
                g_uv = {(float)W, (float)(z1 - z0)};
                piece(c, doorMesh(W, H, expo, hw, z0, z1, 0.0, 0.06, side), Md, Color{236, 236, 232, 255}, em);
            }
            // radiators on the inside of the door: two fixed panels aft, two deployable ones forward
            FinScope f(FIN_RADIATOR);
            g_uv = {(float)W, (float)(z1 - z0)};
            piece(c, doorMesh(W, H, expo, hw, z0 + 0.15, zm - 0.05, 0.075, 0.03, side), Md, rad, em);
            Matrix Mr = MatrixMultiply(toMatrix(rotZ(-side * (ang - dep)), hinge), ML);
            g_uv = {(float)W, (float)(z1 - z0)};
            piece(c, doorMesh(W, H, expo, hw, zm + 0.05, z1 - 0.15, 0.075, 0.03, side), Mr, rad, em);
        }
    } else {
        // a bay of its own: flat doors on its top, hinged on its long upper edges
        for (int side = -1; side <= 1; side += 2) {
            Mat3d R = rotZ(-side * ang);
            Vec3d hinge{side * he.x, he.y, 0};
            Vec3d mid = hinge + R * Vec3d{-side * he.x * 0.5, 0.03, 0};
            FinScope f(FIN_TILES);
            g_uv = {(float)he.x, (float)(2 * he.z)};
            g_scale = {(float)he.x, 0.06f, (float)(2 * he.z)};
            piece(c, cubeMesh, MatrixMultiply(toMatrix(R, mid, he.x, 0.06, 2 * he.z), Mb), Color{236, 236, 232, 255}, em);
        }
    }
}

void drawCraft(const CraftVisual& cv, const Mat3d& rot, const Vec3d& origin, const Vec3d& pivot, const Vec3d& lightDir, float light,
               const Vec3d& camPos, float ambient) {
    if (!cv.comps) return;
    const auto& comps = *cv.comps;
    DrawCtx c;
    c.base = toMatrix(rot, origin - rot * pivot);
    c.lightDir = lightDir;
    c.light = light;
    c.camPos = camPos;
    c.ambient = ambient;
    setV3(shipShader, shipLight, lightDir);
    setF(shipShader, shipLightI, light);
    setF(shipShader, shipAmbient, ambient);
    setV3(shipShader, shipCamPos, camPos);
    setV3(plumeShader, plumeCamPos, camPos);
    std::vector<CraftPlume> plumes;
    struct Canopy { Vec3d top; double D; int n; double riser, pilot; };
    std::vector<Canopy> canopies; // craft-frame attachment points
    if (cv.xray) {
        BeginBlendMode(BLEND_ALPHA);
        rlDisableDepthMask();
    }
    if (cv.wire) rlEnableWireMode();
    rlDisableBackfaceCulling();
    setV3(shipShader, shipCraftUp, norm(rot * Vec3d{0, 1, 0}));
    // windows: their frames (world -> window) and sizes, applied to the surfaces within reach of them
    struct Win { Matrix M; float a[4]; float col[3]; Vec3d p; double r; };
    std::vector<Win> wins;
    for (const Component& k : comps) {
        if (k.func != Func::Window || wins.size() >= 24) continue;
        Mat3d Rw = rot * k.rot().toMat();
        Mat3d A = Rw.T();
        Vec3d o = origin - rot * pivot + rot * k.pos;
        Vec3d bt = A * o * -1.0;
        Win w;
        w.M = toMatrix(A, bt);
        w.a[0] = (float)(k.dims.x / 2);
        w.a[1] = (float)(k.dims.y / 2);
        w.a[2] = (float)std::clamp(k.taper, 0.1, 1.0);
        w.a[3] = (float)std::max(0.0, k.rim);
        w.col[0] = k.color.r / 255.f; w.col[1] = k.color.g / 255.f; w.col[2] = k.color.b / 255.f;
        w.p = k.pos;
        w.r = std::max(k.dims.x, k.dims.y) + k.rim + 0.3;
        wins.push_back(w);
    }
    {
        // crew capsules without windows of their own: four windows round the cone and the hatch window (as on
        // Apollo / Orion / Crew Dragon), drawn into the capsule's skin
        bool anyWin = false;
        for (const Component& k : comps) anyWin |= k.func == Func::Window;
        for (const Component& k : comps) {
            if (anyWin || k.func != Func::Command || k.shape != Shape::Cylinder || k.def < 0 || k.def >= (int)CATALOG.defs.size()) continue;
            const PartDef& d = CATALOG.defs[k.def];
            if (d.kind != PK::Capsule || d.crew <= 0 || wins.size() + 5 > 24) continue;
            double rb = k.dims.x, rt = k.dims.y, h = k.dims.z;
            Mat3d Rk = k.rot().toMat();
            for (int j = 0; j < 5; j++) {
                double az = j < 4 ? (45 + 90 * j) * DEG : 0.0, f = j < 4 ? 0.62 : 0.42;
                double r = rb + (rt - rb) * f;
                Vec3d n = norm(Vec3d{std::cos(az), std::sin(az), (rb - rt) / std::max(h, 1e-3)});
                Vec3d p{r * std::cos(az), r * std::sin(az), -h / 2 + h * f};
                Vec3d up = norm(Vec3d{0, 0, 1} - n * n.z), x = cross(up, n);
                Mat3d Rw = rot * Rk * Mat3d::fromCols(x, up, n), A = Rw.T();
                Vec3d o = origin - rot * pivot + rot * (k.pos + Rk * p);
                Win w;
                w.M = toMatrix(A, A * o * -1.0);
                double s = j < 4 ? 0.09 * rb : 0.07 * rb;
                w.a[0] = (float)s;
                w.a[1] = (float)(s * (j < 4 ? 0.85 : 1.0));
                w.a[2] = j < 4 ? 0.8f : 1.0f;
                w.a[3] = (float)(0.012 * rb);
                w.col[0] = 0.16f; w.col[1] = 0.17f; w.col[2] = 0.19f;
                w.p = k.pos + Rk * p;
                w.r = 2 * s + 0.3;
                wins.push_back(w);
            }
        }
    }
    std::vector<int> winSet, winUp; // (the set uploaded now)
    auto uploadWins = [&](const std::vector<int>& set) {
        if (set == winUp) return;
        winUp = set;
        int n = std::min<int>((int)set.size(), 8);
        float A[32], C[24];
        for (int j = 0; j < n; j++) {
            const Win& w = wins[set[j]];
            SetShaderValueMatrix(shipShader, shipWinM[j], w.M);
            for (int q = 0; q < 4; q++) A[j * 4 + q] = w.a[q];
            for (int q = 0; q < 3; q++) C[j * 3 + q] = w.col[q];
        }
        if (n > 0) {
            SetShaderValueV(shipShader, shipWinA, A, SHADER_UNIFORM_VEC4, n);
            SetShaderValueV(shipShader, shipWinC, C, SHADER_UNIFORM_VEC3, n);
        }
        setI(shipShader, shipNWin, n);
    };
    setI(shipShader, shipNWin, 0);
    setI(shipShader, shipCutOn, 0);
    // the payload bay and the fuselage it is cut into (the one whose box holds the bay's centre)
    int bayI = -1, bayHost = -1;
    for (size_t i = 0; i < comps.size() && bayI < 0; i++)
        if (comps[i].func == Func::CargoBay) bayI = (int)i;
    double bayOpen = clampd(cv.bayT, 0, 1);
    Vector3 cutLo{}, cutHi{};
    if (bayI >= 0) {
        const Component& b = comps[bayI];
        double best = 1e30;
        for (size_t i = 0; i < comps.size(); i++) {
            const Component& L = comps[i];
            if (L.shape != Shape::Loft || L.func == Func::CargoBay || L.dims.x < 0.2 || L.dims.y < 0.2 || L.dims.z < 0.2) continue;
            Mat3d RL = L.rot().toMat();
            Vec3d bl = RL.T() * (b.pos - L.pos);
            Vec3d bz = RL.T() * (b.rot().toMat() * Vec3d{0, 0, 1});
            if (std::fabs(bl.x) > L.dims.x / 2 || std::fabs(bl.y) > L.dims.y / 2 || std::fabs(bl.z) > L.dims.z / 2 || std::fabs(bz.z) < 0.99) continue;
            double v = L.dims.x * L.dims.y * L.dims.z;
            if (v < best) { best = v; bayHost = (int)i; }
        }
        if (bayHost >= 0) {
            const Component& L = comps[bayHost];
            Vec3d bl = L.rot().toMat().T() * (b.pos - L.pos), he = b.dims * 0.5;
            double expo = 2.0 / (2.0 + 2.5 * std::clamp(L.boxy, 0.0, 1.0)), hw = std::min(he.x, 0.49 * L.dims.x);
            double ch = std::pow(std::clamp(hw / (0.5 * L.dims.x), 0.0, 0.999), 1.0 / expo);
            double ys = 0.5 * L.dims.y * std::pow(std::sqrt(std::max(0.0, 1 - ch * ch)), expo);
            cutLo = {(float)(-hw - 0.01), (float)(ys - 0.03), (float)(bl.z - he.z)};
            cutHi = {(float)(hw + 0.01), (float)(L.dims.y), (float)(bl.z + he.z)};
        }
    }
    // drawing order: glass last (see-through canopies, without writing depth)
    std::vector<size_t> order;
    for (size_t i = 0; i < comps.size(); i++)
        if (finishOf(comps[i]) != FIN_GLASS || cv.xray) order.push_back(i);
    size_t firstGlass = order.size();
    for (size_t i = 0; i < comps.size(); i++)
        if (finishOf(comps[i]) == FIN_GLASS && !cv.xray) order.push_back(i);
    bool glassPass = false, cutSet = false;
    for (size_t oi = 0; oi < order.size(); oi++) {
        size_t i = order[oi];
        const Component& k = comps[i];
        if (oi == firstGlass && !glassPass) {
            glassPass = true;
            rlDrawRenderBatchActive();
            BeginBlendMode(BLEND_ALPHA);
            rlDisableDepthMask();
        }
        if (cutSet) {
            setI(shipShader, shipCutOn, 0);
            cutSet = false;
        }
        if (k.func == Func::Window && !cv.editor) continue; // (drawn into the hull around it)
        if (!wins.empty()) {
            winSet.clear();
            double rk = len(shapeHalfExtent(k.shape, k.dims));
            for (size_t j = 0; j < wins.size() && winSet.size() < 8; j++)
                if (len(wins[j].p - k.pos) < rk + wins[j].r && k.func != Func::Window) winSet.push_back((int)j);
            uploadWins(winSet);
        }
        if ((int)i == bayHost && bayOpen > 1e-3) {
            setI(shipShader, shipCutOn, 1);
            SetShaderValue(shipShader, shipCutLo, &cutLo, SHADER_UNIFORM_VEC3);
            SetShaderValue(shipShader, shipCutHi, &cutHi, SHADER_UNIFORM_VEC3);
            cutSet = true;
        }
        bool sel = (int)i == cv.highlight;
        float em = sel ? 0.35f : 0.f;
        if (cv.heat && i < cv.heat->size()) em += (float)std::clamp(((*cv.heat)[i] - 0.25) * 0.9, 0.0, 0.8);
        Color col = k.color;
        if (cv.xray) col.a = sel ? 210 : 80;
        Mat3d Rk = k.rot().toMat();
        Matrix M = compMatrix(k);
        const PartDef* def = (k.def >= 0 && k.def < (int)CATALOG.defs.size()) ? &CATALOG.defs[k.def] : nullptr;
        double h = k.dims.z;
        int fin = finishOf(k);
        FinScope finScope(fin);
        if (k.func == Func::Wing && k.shape == Shape::Airfoil && k.flap > 0 && !cv.wire) {
            drawFlappedWing(c, k, M, col, em, flapAngleAt(cv.flapPos, k.flap));
            continue;
        }
        if (k.func == Func::CargoBay) {
            if (cv.editor || cv.xray) shapePiece(c, k.shape, k.dims, M, Color{col.r, col.g, col.b, (unsigned char)(cv.xray ? col.a : 70)}, em);
            drawBay(c, comps, (int)i, (int)i == bayI ? bayHost : -1, bayOpen, em);
            continue;
        }
        if (k.shape == Shape::Airfoil) g_airfoil = airfoilOf(k);
        if (k.shape == Shape::Loft) g_loft = loftOf(k);
        switch (k.func) {
        case Func::Engine: {
            if (!def || k.shape != Shape::Cylinder) { shapePiece(c, k.shape, k.dims, M, col, em); break; }
            if (def->kind == PK::SRB) {
                double r = std::max(k.dims.x, k.dims.y), nozL = std::min(h * 0.08, 2.5);
                if (r >= 1.2) {
                    // a large segmented booster (Space Shuttle SRB type): flared aft skirt, four casing segments
                    // with their joints, a frustum and a nose cap that ends in a small blunt tip
                    double noseL = r * 2.6, skirtH = std::min(h * 0.07, 3.2), body = std::max(0.01, h - nozL - noseL);
                    Color joint{(unsigned char)(col.r * 0.78), (unsigned char)(col.g * 0.78), (unsigned char)(col.b * 0.78), col.a};
                    lcyl(c, M, -h / 2 + nozL * 0.35, skirtH, r * 1.38, r, col, 0, 0, em);
                    lcyl(c, M, -h / 2 + nozL, body, r, r, col, 0, 0, em);
                    for (int s = 1; s <= 3; s++)
                        lcyl(c, M, -h / 2 + nozL + body * s / 4.0 - 0.15, 0.3, r * 1.015, r * 1.015, joint, 0, 0, em);
                    lcyl(c, M, h / 2 - noseL, noseL * 0.55, r, r * 0.55, col, 0, 0, em);
                    lcyl(c, M, h / 2 - noseL * 0.45, noseL * 0.4, r * 0.55, r * 0.12, col, 0, 0, em);
                    lcyl(c, M, h / 2 - noseL * 0.05, noseL * 0.05, r * 0.12, 0.0, col, 0, 0, em);
                } else {
                    lcyl(c, M, -h / 2 + nozL, std::max(0.01, h - nozL - r * 1.2), r, r, col, 0, 0, em);
                    lcyl(c, M, h / 2 - r * 1.2, r * 1.2, r, 0.0, col, 0, 0, em);
                }
                bellNozzle(c, M, -h / 2, nozL, def->exitDiam * 0.5, Color{60, 55, 52, col.a}, 0, 0, em, 0.f, FIN_ABLATOR);
                bool burning = cv.engine ? (i < cv.engine->size() && (*cv.engine)[i] > 1e-3f)
                                         : cv.ignited && k.stage == cv.activeStage && cv.prop && i < cv.prop->size() && ((*cv.prop)[i] > 0 || cv.infinite);
                if (burning) {
                    double pa = std::clamp(cv.pressureAtm, 0.0, 1.0);
                    double kk = cv.engine ? std::clamp((double)(*cv.engine)[i], 0.0, 1.0) : 1.0; // tails off at burnout
                    CraftPlume cp;
                    cp.R = Rk;
                    cp.p.pos = k.pos + Rk * Vec3d{0, 0, -h / 2};
                    cp.p.exitR = def->exitDiam * 0.5 * (0.7 + 0.3 * kk);
                    cp.p.length = def->exitDiam * 0.5 * (8 + 10 * (1 - pa)) * (0.2 + 0.8 * kk);
                    cp.p.flare = 1.2 + 2.0 * (1 - pa) * (0.4 + 0.6 * kk);
                    cp.p.col = flameColor(Prop::Solid);
                    cp.p.intensity = (float)(0.4 + 0.6 * kk);
                    plumes.push_back(cp);
                }
                break;
            }
            double plateR = std::max(k.dims.y, 0.1), plateH = std::min(0.35, h * 0.2);
            double bellL = std::max(h - plateH, 0.1);
            lcyl(c, M, h / 2 - plateH, plateH, plateR * 0.9, plateR, Color{45, 47, 52, col.a}, 0, 0, em);
            double bellR;
            auto pos = clusterLayout(std::max(1, k.cluster), plateR, bellR, std::min(def->exitDiam * 0.5, std::max(k.dims.x, 0.05)));
            // in flight only engines that really produce thrust get a flame (none after burnout / flameout)
            float level = cv.engine ? (i < cv.engine->size() ? (*cv.engine)[i] : 0.f) : (float)cv.throttle;
            bool firing = cv.engine ? level > 1e-3f : cv.ignited && k.stage == cv.activeStage && cv.throttle > 0;
            for (size_t bi = 0; bi < pos.size(); bi++) {
                const Vec3d& o = pos[bi];
                bool bellFiring = firing && !((k.engOff >> bi) & 1); // (nozzles the pilot switched off stay dark)
                double throatR = bellR * 0.28, chamberL = bellL * 0.25;
                lcyl(c, M, h / 2 - plateH - chamberL, chamberL, throatR * 1.3, throatR * 1.5, Color{90, 92, 98, col.a}, o.x, o.y, em);
                double bz = h / 2 - plateH - bellL;
                Color bellCol = def->powerUse > 0 ? Color{110, 120, 140, col.a} : Color{75, 60, 50, col.a};
                float glow = bellFiring ? (float)(0.15 + 0.4 * level) : 0.f;
                bellNozzle(c, M, bz, bellL * 0.78, bellR, bellCol, o.x, o.y, em, glow, def->powerUse > 0 ? FIN_METAL : FIN_NOZZLE);
                if (bellFiring) {
                    double pa = std::clamp(cv.pressureAtm, 0.0, 1.0);
                    // thrust fraction (0..1): the flame grows longer, wider and brighter with thrust
                    double tk = std::clamp((double)level, 0.0, 1.0);
                    CraftPlume cp;
                    cp.R = Rk;
                    cp.p.pos = k.pos + Rk * Vec3d{o.x, o.y, bz};
                    cp.p.exitR = bellR * (0.7 + 0.3 * tk);
                    double elec = def->powerUse > 0 ? 0.4 : 1.0;
                    cp.p.length = (bellR * (4 + 14 * (1 - pa)) * (0.15 + 0.85 * tk) * elec + bellR * (0.8 + 1.2 * tk));
                    cp.p.flare = 1.0 + 2.5 * (1 - pa) * (0.4 + 0.6 * tk);
                    cp.p.col = flameColor(def->prop);
                    cp.p.intensity = (float)(0.3 + 0.7 * tk) * (def->prop == Prop::Hydrolox || def->prop == Prop::LH2 ? 0.55f : 1.0f);
                    plumes.push_back(cp);
                }
            }
            break;
        }
        case Func::Tank: {
            shapePiece(c, k.shape, k.dims, M, col, em);
            if (k.shape == Shape::Cylinder) {
                double band = std::min(0.12, h * 0.1);
                Color bc{(unsigned char)(col.r * 0.55), (unsigned char)(col.g * 0.55), (unsigned char)(col.b * 0.55), col.a};
                lcyl(c, M, -h / 2, band, k.dims.x * 1.01, k.dims.x * 1.01, bc, 0, 0, em);
                lcyl(c, M, h / 2 - band, band, k.dims.y * 1.01, k.dims.y * 1.01, bc, 0, 0, em);
            }
            break;
        }
        case Func::Solar: {
            float st = std::clamp(cv.solarT, 0.f, 1.f);
            if (cv.stowed && i < cv.stowed->size() && st < 0.999f) {
                const Component& sk = (*cv.stowed)[i];
                if (st <= 0.001f) {
                    shapePiece(c, sk.shape, sk.dims, compMatrix(sk), Color{38, 58, 118, col.a}, em + 0.05f);
                } else {
                    Mat3d Rs;
                    Vec3d ps, ds;
                    solarPose(k, sk, st, Rs, ps, ds);
                    shapePiece(c, k.shape, ds, toMatrix(Rs, ps), Color{38, 58, 118, col.a}, em + 0.05f);
                }
            } else shapePiece(c, k.shape, k.dims, M, Color{38, 58, 118, col.a}, em + 0.05f);
            break;
        }
        case Func::Rotor: {
            // mast and hub; blade sets turning in opposite directions (coaxial) when powered
            double D = def ? def->diam : 2.0, hr = std::max(k.dims.x, 0.02);
            int sets = def && def->canopies > 1 ? 2 : 1, blades = D > 3 ? 3 : 2;
            float level = cv.engine && i < cv.engine->size() ? (*cv.engine)[i] : 0.f;
            lcyl(c, M, -h / 2, h, hr, hr * 0.8, Color{70, 72, 78, col.a}, 0, 0, em);
            double spin = level > 0 ? GetTime() * TAU * (D > 3 ? 4.5 : 9.0) : 0.0; // drawn rate, slower than real (no strobing)
            for (int st = 0; st < sets; st++) {
                double z = h / 2 - st * std::min(0.12 * D, 0.8 * h);
                lcyl(c, M, z - hr * 0.6, hr * 1.2, hr * 1.6, hr * 1.6, Color{55, 57, 62, col.a}, 0, 0, em);
                double a0 = (st == 0 ? spin : -spin) + st * 0.6;
                for (int bl = 0; bl < blades; bl++) {
                    double a = a0 + bl * TAU / blades;
                    Mat3d Rb = rotZ(a);
                    double L = D * 0.5, chord = std::max(0.02, D * (D > 3 ? 0.045 : 0.075)), th = std::max(0.004, D * 0.006);
                    Vec3d p = Rb * Vec3d{L * 0.5, 0, z};
                    g_uv = {(float)L, (float)chord};
                    g_scale = {(float)L, (float)chord, (float)th};
                    piece(c, cubeMesh, MatrixMultiply(toMatrix(Rb * rotX(0.12), p, L, chord, th), M), Color{36, 38, 42, col.a}, em);
                }
            }
            break;
        }
        case Func::Jet: {
            // nacelle with the fan face in front (+z) and the exhaust cone at the back
            double r = std::max(k.dims.x, 0.1);
            JetCycle cyc = def ? def->cycle : JetCycle::Turbofan;
            lcyl(c, M, -h / 2 + r * 0.6, h - r * 0.6, r * 0.82, r, col, 0, 0, em);
            lcyl(c, M, h / 2 - 0.04, 0.04, r * 0.92, r * 0.92, Color{24, 25, 28, col.a}, 0, 0, em);
            if (cyc == JetCycle::Turboramjet || cyc == JetCycle::CombinedCycle || cyc == JetCycle::Scramjet) {
                // the translating inlet spike of a high-Mach intake (J58 / SABRE), sticking out of the cowl
                double sl = r * (cyc == JetCycle::Scramjet ? 0.9 : 1.6);
                lcyl(c, M, h / 2 - 0.3, 0.3 + sl * 0.15, r * 0.42, r * 0.42, Color{60, 62, 68, col.a}, 0, 0, em);
                lcyl(c, M, h / 2 + sl * 0.15, sl * 0.85, r * 0.42, r * 0.02, Color{150, 152, 158, col.a}, 0, 0, em);
            } else lcyl(c, M, h / 2 - 0.3, 0.26, r * 0.18, r * 0.05, Color{90, 92, 98, col.a}, 0, 0, em);
            lcyl(c, M, -h / 2, r * 0.6, r * 0.45, r * 0.82, Color{80, 70, 62, col.a}, 0, 0, em);
            float level = cv.engine && i < cv.engine->size() ? (*cv.engine)[i] : 0.f;
            int mode = cv.jetMode && i < cv.jetMode->size() ? (*cv.jetMode)[i] : (level > 1e-3f ? 1 : 0);
            if (level > 1e-3f) {
                CraftPlume cp;
                cp.R = Rk;
                cp.p.pos = k.pos + Rk * Vec3d{0, 0, -h / 2};
                cp.p.exitR = r * 0.45;
                double pa = std::clamp(cv.pressureAtm, 0.0, 1.0);
                if (mode == 3) {
                    // closed-cycle rocket mode: a hydrolox flame that widens in the thin air
                    cp.p.exitR = r * 0.7;
                    cp.p.length = r * (3.0 + 9.0 * (1 - pa)) * (0.3 + 0.7 * level) + r;
                    cp.p.flare = 1.0 + 2.5 * (1 - pa);
                    cp.p.col = flameColor(Prop::Hydrolox);
                    cp.p.intensity = (0.3f + 0.7f * level) * 0.6f;
                } else if (mode == 2) {
                    // afterburner: a long orange-yellow flame
                    cp.p.length = r * (4.0 + 4.0 * level);
                    cp.p.flare = 1.1;
                    cp.p.col = Vector3{1.0f, 0.62f, 0.30f};
                    cp.p.intensity = 0.55f + 0.35f * level;
                } else if (cyc == JetCycle::Scramjet || (cyc == JetCycle::Turboramjet && level > 0.5f)) {
                    // hot exhaust of a ramjet: a short glow
                    cp.p.length = r * (2.0 + 2.5 * level);
                    cp.p.flare = 1.0;
                    cp.p.col = Vector3{1.0f, 0.55f, 0.32f};
                    cp.p.intensity = 0.25f + 0.25f * level;
                } else {
                    cp.p.length = r * (1.5 + 3.0 * level);
                    cp.p.flare = 1.0;
                    cp.p.col = Vector3{1.0f, 0.66f, 0.43f};
                    cp.p.intensity = 0.12f + 0.25f * level; // a faint hot-air shimmer, no rocket flame
                }
                plumes.push_back(cp);
            }
            break;
        }
        case Func::Wheel: {
            // landing gear folds with the legs command (turning about the top of its strut); rover wheels stay
            float lt = cv.legsT >= 0 ? std::clamp(cv.legsT, 0.f, 1.f) : (cv.legs ? 1.f : 0.f);
            if (retractableWheel(k) && lt <= 0.02f) break; // up in its bay, the doors closed
            Matrix Mw = retractableWheel(k) && lt < 0.999f ? compMatrix(wheelAt(k, lt)) : M;
            double tr = def ? def->diam * 0.5 : 0.35, w = tr * (def && def->powerUse > 0 ? 0.32 : 0.55);
            lcyl(c, Mw, -h / 2, h, 0.06, 0.07, Color{150, 154, 160, col.a}, 0, 0, em);
            g_uv = {(float)(TAU * tr), (float)w};
            piece(c, frustum(1, 1), MatrixMultiply(toMatrix(rotY(PI / 2), {-w / 2, 0, -h / 2}, tr, tr, w), Mw), Color{30, 31, 33, col.a}, em);
            piece(c, frustum(1, 1), MatrixMultiply(toMatrix(rotY(PI / 2), {-w / 2 - 0.005, 0, -h / 2}, tr * 0.55, tr * 0.55, w + 0.01), Mw),
                  Color{165, 168, 172, col.a}, em);
            break;
        }
        case Func::RCS: {
            shapePiece(c, k.shape, k.dims, M, col, em);
            Vec3d he = shapeHalfExtent(k.shape, k.dims);
            double s = std::max(0.03, std::min(he.y, he.z) * 0.5);
            const Mat3d Rn[4] = {rotX(-PI / 2), rotX(PI / 2), Mat3d{}, rotX(PI)};
            const Vec3d e[4] = {{0, he.y, 0}, {0, -he.y, 0}, {0, 0, he.z}, {0, 0, -he.z}};
            for (int f = 0; f < 4; f++)
                piece(c, frustum(0.45, 1.0), MatrixMultiply(toMatrix(Rn[f], e[f], s * 0.5, s * 0.5, s * 0.6), M), Color{45, 45, 50, col.a}, em);
            break;
        }
        case Func::Legs: {
            float lt = cv.legsT >= 0 ? std::clamp(cv.legsT, 0.f, 1.f) : (cv.legs ? 1.f : 0.f);
            if (lt > 0.001f && lt < 0.999f) {
                // swinging out about the hinge on the mount while the strut extends
                Component sk = stowedLeg(k);
                Vec3d topD = k.pos + Rk * Vec3d{0, 0, h / 2};
                Vec3d topS = sk.pos + sk.rot().toMat() * Vec3d{0, 0, sk.dims.z / 2};
                Vec3d dirD = -(Rk * Vec3d{0, 0, 1});
                double s = smooth01(lt);
                Vec3d tp = topS + (topD - topS) * s;
                Vec3d dir = norm(Vec3d{0, 0, -1} * (1 - s) + dirD * s);
                double Ll = sk.dims.z + (h - sk.dims.z) * s, rl = std::max(k.dims.x, 0.03);
                Vec3d end = tp + dir * Ll;
                strut(c, tp, end, rl, col);
                strut(c, end - dir * 0.04, end + dir * 0.04, rl * 2.4 + (0.35 - rl * 2.4) * s, Color{120, 120, 126, col.a});
                break;
            }
            if (lt <= 0.001f) {
                // folded up against the side of the mount (launch configuration)
                Component sk = stowedLeg(k);
                Matrix Ms = compMatrix(sk);
                double hs = sk.dims.z, rl = std::max(sk.dims.x, 0.03);
                lcyl(c, Ms, -hs / 2, hs, rl, rl, col, 0, 0, em);
                lcyl(c, Ms, -hs / 2 - 0.05, 0.07, rl * 2.4, rl * 2.4, Color{120, 120, 126, col.a}, 0, 0, em);
                break;
            }
            double L = h;
            double r = std::max(k.dims.x, 0.03);
            lcyl(c, M, h / 2 - L, L, r, r, col, 0, 0, em);
            lcyl(c, M, h / 2 - L - 0.08, 0.1, 0.35, 0.35, Color{120, 120, 126, col.a}, 0, 0, em);
            break;
        }
        case Func::Parachute: {
            shapePiece(c, k.shape, k.dims, M, Color{230, 230, 230, col.a}, em);
            int nc = def ? std::max(1, def->canopies) : 1;
            double cda = def ? def->chuteCdA : 420.0;
            if (cv.chute)
                canopies.push_back({k.pos + Rk * Vec3d{0, 0, h / 2}, std::sqrt(4 * cda / (PI * 0.85 * nc)), nc, def ? def->chuteRiser : 0.0,
                                    def ? def->pilotChuteD : 0.0});
            break;
        }
        case Func::Fairing: {
            // one clamshell half (the other one is the same piece turned by 180 degrees)
            double mx = std::max({k.dims.x, k.dims.y, 1e-4});
            Matrix lm = MatrixMultiply(toMatrix(Mat3d{}, {0, 0, -h / 2}, mx, mx, h), M);
            const auto& hm = halfShell(k.dims.x, k.dims.y);
            g_uv = {(float)(PI * mx), (float)h};
            piece(c, hm.first, lm, col, em);
            piece(c, hm.second, lm, Color{165, 167, 172, col.a}, em);
            break;
        }
        case Func::Sail:
            shapePiece(c, k.shape, k.dims, M, Color{210, 210, 215, col.a}, em);
            {
                float sa = cv.sailT >= 0 ? std::clamp(cv.sailT, 0.f, 1.f) : (cv.sail ? 1.f : 0.f);
                if (sa > 0.001f && def) {
                    // booms extend and the film unfurls, turning a quarter turn on the way
                    double s = smooth01(sa);
                    double side = std::sqrt(def->sailArea) * (0.04 + 0.96 * s);
                    piece(c, cubeMesh, MatrixMultiply(toMatrix(rotZ(PI / 4 + (1 - s) * PI / 2), {0, 0, h / 2}, side, side, 0.004), M),
                          Color{235, 225, 170, 255}, 0.25f);
                }
            }
            break;
        case Func::Docking: {
            // docking port (+Z face): the collar, the soft-capture ring with its three guide petals, the hatch
            double r = std::max(std::max(k.dims.x, k.dims.y), 0.2);
            Color ring{(unsigned char)(col.r * 0.85), (unsigned char)(col.g * 0.85), (unsigned char)(col.b * 0.85), col.a};
            lcyl(c, M, -h / 2, h * 0.6, r * 0.95, r * 0.9, col, 0, 0, em);
            lcyl(c, M, -h / 2 + h * 0.6, h * 0.4, r * 0.82, r * 0.78, ring, 0, 0, em);
            lcyl(c, M, h / 2 - 0.03, 0.03, r * 0.5, r * 0.5, Color{70, 74, 82, col.a}, 0, 0, em);
            for (int p = 0; p < 3; p++) {
                double a = p * TAU / 3;
                Mat3d Rp = rotZ(a);
                piece(c, cubeMesh, MatrixMultiply(toMatrix(Rp * rotX(-0.35), Rp * Vec3d{0, r * 0.68, h / 2 - h * 0.05}, r * 0.34, 0.05, h * 0.55), M), ring, em);
            }
            break;
        }
        case Func::Hyperdrive: {
            shapePiece(c, k.shape, k.dims, M, col, em);
            double r = std::max(k.dims.x, k.dims.y) * 1.08;
            float g = cv.hyperGlow ? 1.5f : 0.6f;
            lcyl(c, M, -h * 0.25, h * 0.12, r, r, Color{160, 120, 255, col.a}, 0, 0, g);
            lcyl(c, M, h * 0.13, h * 0.12, r, r, Color{160, 120, 255, col.a}, 0, 0, g);
            break;
        }
        default: shapePiece(c, k.shape, k.dims, M, col, em); break;
        }
    }
    if (cutSet) setI(shipShader, shipCutOn, 0);
    setI(shipShader, shipNWin, 0);
    if (glassPass) {
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        if (!cv.xray) EndBlendMode();
    }
    for (auto& cp : canopies) drawCanopies(c, cp.top, cv.chuteDir, cp.D, cp.n, cv.chuteOpen, cp.riser, cp.pilot, cv.chuteReef);
    if (cv.wire) rlDisableWireMode();
    if (!g_shadowPass) rlEnableBackfaceCulling();
    if (cv.xray) {
        rlEnableDepthMask();
        EndBlendMode();
    }
    if (g_shadowPass) return; // (the glowing effects below cast no shadow)
    if (cv.plasma > 0.01f && !comps.empty()) {
        // entry heating: glowing shock layer on the side facing the flow, a trailing plasma wake and
        // sparks shed by the hot (ablating) surface
        Vec3d up = norm(cv.motionDir);
        // outline points of every component (cylinder rims, box corners), in craft axes
        std::vector<Vec3d> pts;
        for (const auto& k : comps) {
            Mat3d Rk = k.rot().toMat();
            if (k.shape == Shape::Cylinder) {
                for (int e = 0; e < 2; e++) {
                    double zz = e ? k.dims.z / 2 : -k.dims.z / 2, rr = e ? k.dims.y : k.dims.x;
                    for (int a = 0; a < 12; a++) pts.push_back(k.pos + Rk * Vec3d{rr * std::cos(a * TAU / 12), rr * std::sin(a * TAU / 12), zz});
                }
            } else {
                Vec3d he = shapeHalfExtent(k.shape, k.dims);
                double f = k.shape == Shape::Ellipsoid ? 0.62 : 1.0;
                for (int sx = -1; sx <= 1; sx += 2)
                    for (int sy = -1; sy <= 1; sy += 2)
                        for (int sz = -1; sz <= 1; sz += 2) pts.push_back(k.pos + Rk * Vec3d{sx * he.x * f, sy * he.y * f, sz * he.z * f});
            }
        }
        Vec3d c0{};
        for (const auto& p : pts) c0 += p;
        c0 = c0 / (double)std::max<size_t>(pts.size(), 1);
        double front = 0.1, lat = 0.1;
        for (const auto& p : pts) {
            Vec3d d = p - c0;
            front = std::max(front, dot(d, up));
            lat = std::max(lat, len(d - up * dot(d, up)));
        }
        float pl = std::clamp(cv.plasma, 0.0f, 1.0f);
        Vec3d frontP = c0 + up * front;
        // the shoulder (heat-shield rim): the forward-most outline point at nearly the full width; the
        // flow separates there, so the sheath starts at the rim and the afterbody sits in its wake
        double rimZ = -1e30, backZ = 1e30;
        for (const auto& p : pts) {
            Vec3d d = p - c0;
            double ax = dot(d, up);
            if (len(d - up * ax) >= 0.85 * lat) rimZ = std::max(rimZ, ax);
            backZ = std::min(backZ, ax);
        }
        if (rimZ < -1e29) rimZ = front;
        Vec3d rimP = c0 + up * rimZ, backP = c0 + up * backZ;
        double bodyL = std::max(front - backZ, lat * 0.5);
        Vector3 hot{1.0f, 0.42f + 0.2f * pl, 0.16f + 0.5f * pl * pl}; // orange, turning pink-white at peak heating
        Vec3d z = up, x = norm(anyPerp(z)), y = cross(z, x);
        Mat3d Rf = Mat3d::fromCols(x, y, z);
        // shock layer: on the vehicle's own surface facing the flow (two shells: the bright layer right on
        // the surface, a fainter one standing off further)
        {
            bool ok = plasmaShader.id != 0 && plasmaShader.id != rlGetShaderIdDefault() && plasmaShader.locs != nullptr;
            if (ok) {
                Vec3d fw = norm(rot * up);
                setV3(plasmaShader, plasmaFlow, fw);
                setV3(plasmaShader, plasmaCam, c.camPos);
                Vector3 hc{1.0f, 0.55f + 0.25f * pl, 0.32f + 0.45f * pl * pl};
                SetShaderValue(plasmaShader, plasmaHot, &hc, SHADER_UNIFORM_VEC3);
                float tm = (float)std::fmod(GetTime(), 1000.0);
                setF(plasmaShader, plasmaTime, tm);
                BeginBlendMode(BLEND_ADDITIVE);
                rlDisableDepthMask();
                for (int layer = 0; layer < 2; layer++) {
                    setF(plasmaShader, plasmaThick, (float)(lat * (layer ? 0.16 + 0.06 * pl : 0.035)));
                    setF(plasmaShader, plasmaInten, (layer ? 0.22f : 0.85f) * (0.35f + 0.65f * pl));
                    for (const auto& k : comps) {
                        Matrix Mk = compMatrix(k);
                        const Vec3d& d = k.dims;
                        const Mesh* mesh = &cubeMesh;
                        Matrix S = toMatrix(Mat3d{}, {}, d.x, d.y, d.z);
                        switch (k.shape) {
                        case Shape::Cylinder: {
                            double mx = std::max(std::max(d.x, d.y), 1e-4);
                            mesh = &frustum(d.x, d.y);
                            S = toMatrix(Mat3d{}, {0, 0, -d.z / 2}, mx, mx, d.z);
                            break;
                        }
                        case Shape::Ellipsoid: mesh = &sphereMesh; break;
                        case Shape::Wedge: mesh = &wedgeMesh; break;
                        case Shape::Airfoil: mesh = &airfoilMesh(d.x / std::max(d.y, 1e-4), airfoilOf(k)); break;
                        case Shape::Loft: mesh = &loftMesh(loftOf(k)); break;
                        case Shape::Truss:
                            mesh = &trussMesh(d);
                            S = MatrixIdentity();
                            break;
                        default: break;
                        }
                        DrawMesh(*mesh, plasmaMat, MatrixMultiply(MatrixMultiply(S, Mk), c.base));
                    }
                }
                rlDrawRenderBatchActive();
                rlEnableDepthMask();
                EndBlendMode();
            }
        }
        // sheath: from the rim, enclosing the afterbody, widening slowly downstream
        CraftPlume sheath;
        sheath.R = Rf;
        sheath.shape = 1;
        sheath.limb = 1.0f;
        sheath.p.pos = rimP;
        sheath.p.exitR = lat * 1.04 / 0.86 * (1.0 + 0.15 * pl);
        sheath.p.length = (rimZ - backZ) + lat * (1.5 + 2.5 * pl);
        sheath.p.flare = 1.0;
        sheath.p.col = hot;
        sheath.p.intensity = 0.25f + 0.55f * pl;
        plumes.push_back(sheath);
        // trailing wake behind the vehicle
        CraftPlume wake;
        wake.wake = true;
        wake.R = Rf;
        wake.limb = 0.5f;
        wake.p.pos = backP;
        wake.p.exitR = lat * (0.9 + 0.3 * pl);
        wake.p.length = lat * (2.0 + 4.0 * pl) + bodyL * 0.5;
        wake.p.flare = 1.0;
        wake.p.col = hot;
        wake.p.intensity = 0.12f + 0.35f * pl;
        plumes.push_back(wake);
        // shock-layer glow and sparks are billboards in world space
        auto toWorld = [&](const Vec3d& p) { return toF(origin + rot * (p - pivot)); };
        BeginBlendMode(BLEND_ADDITIVE);
        rlDisableDepthMask();
        // (a soft glow around the stagnation region: smaller than the vehicle, it only brightens the layer)
        Color gc{255, (unsigned char)(130 + 90 * pl), (unsigned char)(70 + 140 * pl * pl), (unsigned char)(25 + 70 * pl)};
        facingBillboard(glowTex, toWorld(frontP), (float)(lat * (0.9 + 0.5 * pl)), gc);
        double tnow = GetTime();
        int nSpark = (int)(10 + 30 * pl);
        for (int k = 0; k < nSpark; k++) {
            double h1 = std::fmod(k * 0.6180339887, 1.0), h2 = std::fmod(k * 0.7548776662 + 0.31, 1.0);
            double ph = std::fmod(tnow * (1.4 + h1) + h2, 1.0); // 0 at the hot surface, 1 far downstream
            double ang = h1 * TAU, rr = lat * (1.02 + 0.25 * h2 + 0.35 * ph);
            Vec3d p = rimP - up * (ph * (bodyL + 4 * lat)) + (x * std::cos(ang) + y * std::sin(ang)) * rr;
            unsigned char a = (unsigned char)(std::clamp((1.0 - ph) * (0.4 + 0.6 * pl), 0.0, 1.0) * 255);
            facingBillboard(glowTex, toWorld(p), (float)(lat * (0.12 + 0.1 * h2)), Color{255, 190, 120, a});
        }
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        EndBlendMode();
    }
    if (cv.puffs) {
        for (const RcsPuff& pf : *cv.puffs) {
            if (pf.level < 0.03f) continue;
            // plume axis along -Z of R: point -Z along the exhaust
            Vec3d z = -norm(pf.dir);
            Vec3d x = std::fabs(z.x) < 0.9 ? norm(cross({1, 0, 0}, z)) : norm(cross({0, 1, 0}, z));
            Vec3d y = cross(z, x);
            CraftPlume cp;
            for (int r = 0; r < 3; r++) {
                cp.R.m[r][0] = r == 0 ? x.x : r == 1 ? x.y : x.z;
                cp.R.m[r][1] = r == 0 ? y.x : r == 1 ? y.y : y.z;
                cp.R.m[r][2] = r == 0 ? z.x : r == 1 ? z.y : z.z;
            }
            cp.p.pos = pf.pos;
            cp.p.exitR = 0.025 * pf.size;
            cp.p.length = (0.25 + 0.9 * pf.level) * pf.size;
            cp.p.flare = 3.0;
            cp.p.col = pf.electric ? Vector3{0.45f, 0.62f, 1.0f} : Vector3{0.92f, 0.95f, 1.0f};
            cp.p.intensity = (0.25f + 0.45f * pf.level) * (pf.electric ? 0.8f : 1.0f);
            plumes.push_back(cp);
        }
    }
    if (!plumes.empty()) {
        BeginBlendMode(BLEND_ADDITIVE);
        rlDisableDepthMask();
        rlDisableBackfaceCulling();
        for (const auto& cp : plumes) {
            const Plume& pl = cp.p;
            setF(plumeShader, plumeLimb, cp.limb);
            if (cp.shape == 2) {
                // shock cap: ellipsoid, bright on its upstream half (the plume shader fades along -z)
                plumeMat.maps[MATERIAL_MAP_DIFFUSE].color = Color{(unsigned char)(std::min(1.f, pl.col.x) * 255), (unsigned char)(std::min(1.f, pl.col.y) * 255),
                                                                  (unsigned char)(std::min(1.f, pl.col.z) * 255), (unsigned char)(std::clamp(pl.intensity, 0.f, 1.f) * 255)};
                DrawMesh(sphereLo, plumeMat, MatrixMultiply(toMatrix(cp.R, pl.pos, pl.exitR, pl.exitR, cp.capZ), c.base));
                continue;
            }
            if (cp.shape == 1) {
                plumeMat.maps[MATERIAL_MAP_DIFFUSE].color = Color{(unsigned char)(std::min(1.f, pl.col.x) * 255), (unsigned char)(std::min(1.f, pl.col.y) * 255),
                                                                  (unsigned char)(std::min(1.f, pl.col.z) * 255), (unsigned char)(std::clamp(pl.intensity, 0.f, 1.f) * 255)};
                DrawMesh(sheathMesh, plumeMat, MatrixMultiply(toMatrix(cp.R, pl.pos, pl.exitR, pl.exitR, pl.length), c.base));
                continue;
            }
            for (int layer = 0; layer < 2; layer++) {
                double rr = pl.exitR * (layer == 0 ? 0.95 : 0.55);
                double L = pl.length * (layer == 0 ? 1.0 : 0.45);
                double fl = layer == 0 ? pl.flare : 1.0;
                float inten = pl.intensity * (layer == 0 ? 0.6f : 1.0f);
                plumeMat.maps[MATERIAL_MAP_DIFFUSE].color = Color{(unsigned char)(std::min(1.f, pl.col.x) * 255), (unsigned char)(std::min(1.f, pl.col.y) * 255),
                                                                  (unsigned char)(std::min(1.f, pl.col.z) * 255), (unsigned char)(std::clamp(inten, 0.f, 1.f) * 255)};
                Matrix m = toMatrix(cp.R, pl.pos, rr * (0.5 + 0.5 * fl), rr * (0.5 + 0.5 * fl), L);
                DrawMesh(cp.wake ? wakeMesh : plumeMesh, plumeMat, MatrixMultiply(m, c.base));
            }
        }
        setF(plumeShader, plumeLimb, 0.f);
        rlEnableBackfaceCulling();
        rlEnableDepthMask();
        EndBlendMode();
    }
}

void drawNavball(Rectangle r, const Mat3d& localToHorizon) {
    BeginShaderMode(navShader);
    SetShaderValueMatrix(navShader, navOrient, toMatrix(localToHorizon));
    DrawTexturePro(whiteTex, {0, 0, 4, 4}, r, {0, 0}, 0, WHITE);
    EndShaderMode();
}

} // namespace gfx

namespace gfx {
void setEnvironment(const Vec3d& up, Vector3 sky, Vector3 ground) { envSet(up, sky, ground); }
void resetEnvironment() {
    envSet({0, 0, 1}, {0.46f, 0.50f, 0.58f}, {0.13f, 0.13f, 0.14f});
    setSunTint({1, 1, 1}); // (the flight view tints the sunlight: dark red / black at dusk and at night)
}
} // namespace gfx

namespace gfx {
void setSpotLights(int n, const Vec3d* pos, const Vec3d* dir, Vector3 colour, float range) {
    n = std::clamp(n, 0, 4);
    Vector3 p[4]{}, d[4]{};
    for (int i = 0; i < n; i++) {
        p[i] = toF(pos[i]);
        d[i] = toF(norm(dir[i]));
    }
    setI(shipShader, shipNSpots, n);
    if (n > 0) {
        SetShaderValueV(shipShader, shipSpotPos, p, SHADER_UNIFORM_VEC3, n);
        SetShaderValueV(shipShader, shipSpotDir, d, SHADER_UNIFORM_VEC3, n);
        setV3(shipShader, shipSpotCol, colour);
        setF(shipShader, shipSpotRange, range);
    }
}
} // namespace gfx

namespace gfx {
double padHardstandSize(const PadSpec& ps) { return padHardstand(ps); }

void drawGroundShadow(const Vec3d& a, const Vec3d& b, const Vec3d& up, double halfW, double ext, float alpha) {
    if (alpha <= 0.003f || !shadowTex.id) return;
    Vec3d u = norm(up);
    Vec3d d = b - a;
    d = d - u * dot(d, u);
    Vec3d dir = len(d) > 1e-6 ? norm(d) : anyPerp(u);
    Vec3d side = norm(cross(u, dir));
    Vec3d lift = u * 0.05;
    Vec3d p0 = a - dir * ext - side * halfW + lift, p1 = b + dir * ext - side * halfW + lift;
    Vec3d p2 = b + dir * ext + side * halfW + lift, p3 = a - dir * ext + side * halfW + lift;
    rlDrawRenderBatchActive();
    BeginBlendMode(BLEND_MULTIPLIED);
    rlDisableDepthMask();
    rlDisableBackfaceCulling();
    rlSetTexture(shadowTex.id);
    rlBegin(RL_QUADS);
    rlColor4ub(0, 0, 0, (unsigned char)(std::clamp(alpha, 0.0f, 1.0f) * 255));
    auto vtx = [](const Vec3d& p, float s, float t) {
        rlTexCoord2f(s, t);
        rlVertex3f((float)p.x, (float)p.y, (float)p.z);
    };
    vtx(p0, 0, 0);
    vtx(p1, 1, 0);
    vtx(p2, 1, 1);
    vtx(p3, 0, 1);
    rlEnd();
    rlSetTexture(0);
    rlDrawRenderBatchActive();
    rlEnableBackfaceCulling();
    rlEnableDepthMask();
    EndBlendMode();
}
} // namespace gfx

namespace gfx {
namespace {
void facingBillboard(Texture2D tex, Vector3 pos, float size, Color c) {
    if (g_shadowPass) return;
    Matrix v = MatrixLookAt(curCam.position, curCam.target, curCam.up);
    Vector3 up{v.m1, v.m5, v.m9};
    Rectangle src{0, 0, (float)tex.width, (float)tex.height};
    DrawBillboardPro(curCam, tex, src, pos, up, Vector2{size, size}, Vector2{size * 0.5f, size * 0.5f}, 0.0f, c);
}
} // namespace
void setTerrainQuality(int q) { g_terrainQ = std::clamp(q, 0, 3); }
void setDetailRange(float k) {
    g_detRange = std::clamp(k, 0.5f, 4.0f);
    terrain::setDetailRange(g_detRange);
}
bool setPhysicalShading(bool on) {
    if (on && !g_pbrLoaded && !g_pbrFailed) {
        // compile the physically based variants once (a failure leaves the standard look in place)
        std::string pfs = withPbr(PLANET_FS, true), sfs = withPbr(SHIP_FS, true);
        std::string kfs = std::string("#version 330\n") + ATMO_GLSL + SKY_FS_MAIN;
        std::string pvs = withPbr(PLANET_VS);
        planetPbr = LoadShaderFromMemory(pvs.c_str(), pfs.c_str());
        shipPbr = LoadShaderFromMemory(SHIP_VS, sfs.c_str());
        skyShader = LoadShaderFromMemory(nullptr, kfs.c_str());
        if (shaderOk(planetPbr) && shaderOk(shipPbr) && shaderOk(skyShader)) {
            planetLocs(planetPbr, PLpbr);
            shipLocPbr = shipLocsOf(shipPbr);
            planetLocs(skyShader, SKL);
            skyRes = GetShaderLocation(skyShader, "res");
            skyFwd = GetShaderLocation(skyShader, "cFwd");
            skyRight = GetShaderLocation(skyShader, "cRight");
            skyUp = GetShaderLocation(skyShader, "cUp");
            skyTanY = GetShaderLocation(skyShader, "tanY");
            skyAspect = GetShaderLocation(skyShader, "aspect");
            skySun = GetShaderLocation(skyShader, "sunDir");
            g_pbrLoaded = true;
        } else {
            if (shaderOk(planetPbr)) UnloadShader(planetPbr);
            if (shaderOk(shipPbr)) UnloadShader(shipPbr);
            if (shaderOk(skyShader)) UnloadShader(skyShader);
            planetPbr = {};
            shipPbr = {};
            skyShader = {};
            g_pbrFailed = true;
            TraceLog(LOG_WARNING, "physically based shading: the shaders did not compile");
        }
    }
    bool use = on && g_pbrLoaded;
    g_pbr = use;
    planetShader = use ? planetPbr : planetStd;
    PL = use ? PLpbr : PLstd;
    planetMat.shader = planetShader;
    shipShader = use ? shipPbr : shipStd;
    applyShipLocs(use ? shipLocPbr : shipLocStd);
    shipMat.shader = shipShader;
    setI(shipShader, shipNSpots, 0);
    resetEnvironment();
    setSunTint({1, 1, 1});
    return use == on;
}
bool physicalShading() { return g_pbr; }

// ------------------------------------------------------------------ realistic shadows (shadow map + PCSS)
namespace {
const char* DEPTH_VS = R"(#version 330
in vec3 vertexPosition;
uniform mat4 mvp;
void main() { gl_Position = mvp * vec4(vertexPosition, 1.0); }
)";
const char* DEPTH_FS = R"(#version 330
out vec4 finalColor;
void main() { finalColor = vec4(1.0); }
)";
unsigned g_shFbo = 0, g_shTex = 0;
int g_shRes = 0, g_shQ = 1;
bool g_shOn = false, g_shFail = false, g_shReady = false;
Shader g_depthSh{};
struct ShLocs { int on = -1, vp = -1, a = -1, sun = -1, n = -1; };
ShLocs g_shLocPl, g_shLocShip;
Matrix g_shSaveP{}, g_shSaveV{}, g_shVP{};
float g_shA[4] = {0, 1, 1, 1};
float g_shSunTan = 0.0093f;
ShLocs shadowLocsOf(Shader& sh) {
    ShLocs L;
    L.on = GetShaderLocation(sh, "shadowOn");
    L.vp = GetShaderLocation(sh, "shadowVP");
    L.a = GetShaderLocation(sh, "shadowA");
    L.sun = GetShaderLocation(sh, "sunTan");
    L.n = GetShaderLocation(sh, "shadowN");
    // the map is bound as the material's BRDF map (slot 10): raylib binds it and sets the sampler
    sh.locs[SHADER_LOC_MAP_BRDF] = GetShaderLocation(sh, "shadowMap");
    setI(sh, L.on, 0);
    return L;
}
void releaseShadowMap() {
    if (g_shFbo) rlUnloadFramebuffer(g_shFbo); // (deletes the depth texture attached to it)
    g_shFbo = g_shTex = 0;
    g_shRes = 0;
    g_shReady = false;
}
bool ensureShadowMap() {
    int res = g_shQ ? 4096 : 2048;
    if (g_shFbo && g_shRes == res) return true;
    releaseShadowMap();
    if (g_shFail) return false;
    if (!shaderOk(g_depthSh)) {
        g_depthSh = LoadShaderFromMemory(DEPTH_VS, DEPTH_FS);
        if (!shaderOk(g_depthSh)) {
            g_shFail = true;
            return false;
        }
        g_depthMat = LoadMaterialDefault();
        g_depthMat.shader = g_depthSh;
    }
    if (g_shLocPl.on < 0 && shaderOk(planetPbr)) g_shLocPl = shadowLocsOf(planetPbr);
    if (g_shLocShip.on < 0 && shaderOk(shipPbr)) g_shLocShip = shadowLocsOf(shipPbr);
    g_shFbo = rlLoadFramebuffer();
    if (!g_shFbo) {
        g_shFail = true;
        return false;
    }
    rlEnableFramebuffer(g_shFbo);
    g_shTex = rlLoadTextureDepth(res, res, false);
    if (g_shTex) rlFramebufferAttach(g_shFbo, g_shTex, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
    // rlFramebufferAttach unbinds to FBO 0 on return: re-bind g_shFbo before setting GL_NONE (0) for depth-only FBO
    rlEnableFramebuffer(g_shFbo);
    glDrawBuffer(0);
    glReadBuffer(0);
    bool ok = g_shTex && rlFramebufferComplete(g_shFbo);
    rlDisableFramebuffer();
    if (!ok) {
        releaseShadowMap();
        g_shFail = true;
        TraceLog(LOG_WARNING, "realistic shadows: no depth render target on this graphics driver");
        return false;
    }
    g_shRes = res;
    return true;
}
void applyShadowUniforms(Shader& sh, const ShLocs& L, bool use) {
    if (!shaderOk(sh) || L.on < 0) return;
    setI(sh, L.on, use ? 1 : 0);
    if (!use) return;
    SetShaderValueMatrix(sh, L.vp, g_shVP);
    SetShaderValue(sh, L.a, g_shA, SHADER_UNIFORM_VEC4);
    setF(sh, L.sun, g_shSunTan);
    setI(sh, L.n, g_shQ ? 32 : 16);
}
} // namespace

bool setShadows(bool on, int quality) {
    quality = std::clamp(quality, 0, 1);
    if (quality != g_shQ) {
        g_shQ = quality;
        releaseShadowMap(); // (made again at the new size when next drawn)
    }
    g_shOn = on;
    return !on || !g_shFail;
}
bool shadowsActive() { return g_shOn && g_pbr && !g_shFail; }
bool shadowPassActive() { return g_shadowPass; }

bool beginShadowPass(const Vec3d& center, const Vec3d& toSun, double radius, double back, double ahead) {
    g_shReady = false;
    if (!shadowsActive() || !ensureShadowMap() || len(toSun) < 1e-9 || radius <= 0) return false;
    rlDrawRenderBatchActive();
    g_shSaveP = rlGetMatrixProjection();
    g_shSaveV = rlGetMatrixModelview();
    rlEnableFramebuffer(g_shFbo);
    rlViewport(0, 0, g_shRes, g_shRes);
    rlEnableDepthMask();
    rlClearScreenBuffers();
    Vec3d L = norm(toSun);
    Matrix V = MatrixLookAt(toF(center + L * back), toF(center), toF(anyPerp(L)));
    Matrix P = MatrixOrtho(-radius, radius, -radius, radius, 0.0, back + ahead);
    rlSetMatrixProjection(P);
    rlSetMatrixModelview(V);
    rlEnableDepthTest();
    rlDisableBackfaceCulling(); // (thin panels and wings: both faces)
    g_shVP = MatrixMultiply(V, P);
    g_shA[0] = 1.0f / g_shRes;
    g_shA[1] = (float)(back + ahead);
    g_shA[2] = (float)(1.0 / (2 * radius));
    g_shA[3] = (float)(2 * radius / g_shRes);
    g_shadowPass = true;
    return true;
}

void endShadowPass(unsigned prevFbo, int w, int h, double sunAngDiam) {
    if (!g_shadowPass) return;
    rlDrawRenderBatchActive();
    g_shadowPass = false;
    if (prevFbo) rlEnableFramebuffer(prevFbo);
    else rlDisableFramebuffer();
    rlViewport(0, 0, w, h);
    rlSetMatrixProjection(g_shSaveP);
    rlSetMatrixModelview(g_shSaveV);
    rlEnableBackfaceCulling();
    g_shSunTan = (float)std::tan(std::clamp(sunAngDiam, 1e-5, 0.3));
    g_shReady = true;
}

void shadowReceive(bool on) {
    bool use = on && g_shReady && g_pbr && g_shOn && g_shTex;
    applyShadowUniforms(planetPbr, g_shLocPl, use);
    applyShadowUniforms(shipPbr, g_shLocShip, use);
    Texture2D t{};
    if (use) t = Texture2D{g_shTex, g_shRes, g_shRes, 1, 19};
    planetMat.maps[MATERIAL_MAP_BRDF].texture = t;
    shipMat.maps[MATERIAL_MAP_BRDF].texture = t;
}

void setSunTint(Vector3 c) {
    if (g_pbr) setV3(shipShader, shipSunTint, c);
}

Vector3 sunTintAt(const Universe& U, int body, const Vec3d& posRel, const Vec3d& sunDir) {
    const Body& b = U.bodies[body];
    if (b.atmTop <= 0 || b.isStar) return {1, 1, 1};
    AtmP a = atmParams(b);
    // optical depth along the way to the sun (double precision, km)
    Vec3d p = posRel * 0.001, s = norm(sunDir);
    double R = a.Rkm, A = a.Akm;
    double bq = dot(p, s), c = dot(p, p) - A * A, d = bq * bq - c;
    if (d <= 0) return {1, 1, 1};
    double t1 = -bq + std::sqrt(d);
    if (t1 <= 0) return {1, 1, 1};
    double odR = 0, odM = 0;
    for (int i = 0; i < 48; i++) {
        double u0 = i / 48.0, u1 = (i + 1) / 48.0, ta = t1 * u0 * u0, tb = t1 * u1 * u1, ds = tb - ta;
        Vec3d q = p + s * (0.5 * (ta + tb));
        double h = len(q) - R;
        if (h < 0) return {0, 0, 0}; // (the planet is in the way)
        odR += std::exp(-h / a.HR) * ds;
        odM += std::exp(-h / a.HM) * ds;
    }
    return {(float)std::exp(-(a.bR.x * odR + a.bMe.x * odM)), (float)std::exp(-(a.bR.y * odR + a.bMe.y * odM)),
            (float)std::exp(-(a.bR.z * odR + a.bMe.z * odM))};
}

namespace {
void trimExtraCaches() {
    for (auto& kv : airfoilCache) UnloadMesh(kv.second);
    airfoilCache.clear();
    for (auto& kv : airfoilPartCache) UnloadMesh(kv.second);
    airfoilPartCache.clear();
    for (auto& kv : doorCache) UnloadMesh(kv.second);
    doorCache.clear();
    for (auto& kv : loftCache) UnloadMesh(kv.second);
    loftCache.clear();
    for (auto& kv : trussCache) UnloadMesh(kv.second);
    trussCache.clear();
}
void shutdownExtra() {
    trimExtraCaches();
    releaseShadowMap();
    if (shaderOk(g_depthSh)) { UnloadShader(g_depthSh); g_depthSh = {}; }
    for (Texture2D& t : g_baseTex)
        if (t.id) { UnloadTexture(t); t = {}; }
    g_baseTexReady = false;
}
} // namespace

void drawPhysicalSky(const Universe& U, int body, const Vec3d& camRel, const Vec3d& fwd, const Vec3d& up, float fovyDeg, const Vec3d& sunDir) {
    if (!g_pbr || !shaderOk(skyShader)) return;
    const Body& b = U.bodies[body];
    setAtm(skyShader, SKL, b, camRel, nullptr);
    Vector2 res{(float)GetRenderWidth(), (float)GetRenderHeight()};
    SetShaderValue(skyShader, skyRes, &res, SHADER_UNIFORM_VEC2);
    Vec3d f = norm(fwd), r = norm(cross(f, up)), u = cross(r, f);
    setV3(skyShader, skyFwd, f);
    setV3(skyShader, skyRight, r);
    setV3(skyShader, skyUp, u);
    setF(skyShader, skyTanY, (float)std::tan(fovyDeg * 0.5 * DEG));
    setF(skyShader, skyAspect, res.x / std::max(res.y, 1.0f));
    setV3(skyShader, skySun, norm(sunDir));
    BeginShaderMode(skyShader);
    DrawRectangle(0, 0, 100000, 100000, WHITE); // (the whole screen: the shader works on pixel coordinates)
    EndShaderMode();
}
int terrainQuality() { return g_terrainQ; }

} // namespace gfx
