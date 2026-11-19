#include "audio.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include "rl.hpp"
#include "winshell.hpp"

namespace sfx {
namespace {
constexpr int SR = 44100;
constexpr float TWO_PI = 6.28318530718f;

bool g_ready = false;
std::atomic<float> g_master{0.75f};
Sound g_sounds[(int)Ev::COUNT];
bool g_loaded[(int)Ev::COUNT] = {};
int g_fromFiles = 0;
AudioStream g_stream{};

// continuous-mix targets written by the game thread, read by the audio thread
std::atomic<float> t_engine{0}, t_vacuum{0}, t_crackle{0}, t_wind{0}, t_speed{0}, t_rcs{0};

struct Rng {
    uint32_t s = 0x9E3779B9u;
    float next() { // white noise in [-1, 1]
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return (float)(int32_t)s * (1.0f / 2147483648.0f);
    }
};
struct LowPass {
    float y = 0;
    float run(float x, float fc) {
        float a = 1.0f - std::exp(-TWO_PI * fc / SR);
        y += a * (x - y);
        return y;
    }
};

// ---------------------------------------------------------------- one-shot synthesis
std::vector<float> synth(Ev e) {
    Rng r;
    r.s = 12345u + (uint32_t)e * 7919u;
    LowPass lp1, lp2, lp3;
    auto len = [](float s) { return (size_t)(s * SR); };
    std::vector<float> o;
    switch (e) {
    case Ev::Click: {
        o.resize(len(0.035f));
        for (size_t i = 0; i < o.size(); i++) {
            float t = (float)i / SR;
            o[i] = 0.30f * std::sin(TWO_PI * 1900 * t) * std::exp(-t / 0.004f) + 0.08f * r.next() * std::exp(-t / 0.0015f);
        }
        break;
    }
    case Ev::Toggle: {
        o.resize(len(0.09f));
        for (size_t i = 0; i < o.size(); i++) {
            float t = (float)i / SR, t2 = t - 0.045f;
            float c = std::sin(TWO_PI * 1350 * t) * std::exp(-t / 0.003f);
            if (t2 > 0) c += 0.8f * std::sin(TWO_PI * 1650 * t2) * std::exp(-t2 / 0.003f);
            o[i] = 0.28f * c;
        }
        break;
    }
    case Ev::Stage: { // pyrotechnic separation: crack, structural thump, ringing metal
        o.resize(len(1.6f));
        const float f[4] = {410, 1033, 1781, 2650}, tau[4] = {0.55f, 0.35f, 0.22f, 0.14f};
        float ph = 0;
        for (size_t i = 0; i < o.size(); i++) {
            float t = (float)i / SR;
            float crack = lp1.run(r.next(), 3500) * 2.0f * std::exp(-t / 0.012f);
            ph += TWO_PI * (48 + 60 * std::exp(-t / 0.05f)) / SR;
            float thump = std::sin(ph) * std::exp(-t / 0.28f);
            float ring = 0;
            for (int k = 0; k < 4; k++) ring += std::sin(TWO_PI * f[k] * t + k) * std::exp(-t / tau[k]);
            o[i] = 0.7f * crack + 0.8f * thump + 0.07f * ring;
        }
        break;
    }
    case Ev::Ignite: { // turbopump spin-up and the roar building
        o.resize(len(1.8f));
        float ph = 0;
        for (size_t i = 0; i < o.size(); i++) {
            float t = (float)i / SR;
            float env = (1 - std::exp(-t / 0.18f)) * std::exp(-t / 0.9f);
            float rumble = lp2.run(lp1.run(r.next(), 900), 600);
            ph += TWO_PI * (300 + 1400 * std::min(1.0f, t / 0.6f)) / SR;
            float whine = std::sin(ph) * std::exp(-t / 0.5f) * 0.12f;
            o[i] = (2.4f * rumble + whine) * env + 0.8f * lp3.run(r.next(), 2000) * std::exp(-t / 0.008f);
        }
        break;
    }
    case Ev::Explosion:
    case Ev::ExplosionFar: {
        // blast crack, a falling "whump" with harmonics (audible on small speakers too), tank ruptures
        // following the first blast, debris clatter and a rumbling tail. The far version: the air has
        // taken the highs away and stretched the rumble.
        bool far = e == Ev::ExplosionFar;
        o.resize(len(far ? 4.5f : 3.8f));
        LowPass bLo, bHi, body1, body2, rum1, rum2, dLo, dHi, farLp1, farLp2;
        struct Burst { float t0, amp; } bursts[] = {{0.0f, 1.0f}, {0.30f, 0.5f}, {0.68f, 0.32f}, {1.12f, 0.2f}};
        float ph = 0, crack = 0;
        for (size_t i = 0; i < o.size(); i++) {
            float t = (float)i / SR, n = r.next();
            float band = bHi.run(n, 6000) - bLo.run(n, 280); // 280 Hz .. 6 kHz
            float blast = 0, envBody = 0, bright = 0;
            for (const Burst& b : bursts) {
                float tt = t - b.t0;
                if (tt < 0) continue;
                float atk = std::min(1.0f, tt / 0.0015f);
                blast += b.amp * atk * std::exp(-tt / 0.035f);
                envBody += b.amp * std::min(1.0f, tt / 0.006f) * std::exp(-tt / 0.55f);
                bright += b.amp * std::exp(-tt / 0.2f);
            }
            float fc = 420 + 2600 * std::min(1.0f, bright);
            float body = body2.run(body1.run(n, fc), fc);
            // whump: pitch falls from ~95 Hz to ~42 Hz; the soft clip adds the 2nd-4th harmonics
            float f0 = 55 + 60 * std::exp(-t / 0.25f);
            ph += TWO_PI * f0 / SR;
            float whump = std::tanh(2.6f * std::sin(ph) * std::exp(-t / 0.6f) * std::min(1.0f, t / 0.004f));
            // debris: sparse crackles, thinning out
            if (t > 0.05f && r.next() > 0.9989f - 0.003f * std::exp(-(t - 0.05f) / 0.9f)) crack = 1;
            crack *= 0.991f;
            float deb = (dHi.run(crack * n, 4200) - dLo.run(crack * n, 900)) * 2.2f * std::exp(-t / 1.3f);
            float rumble = (rum2.run(n, 420) - rum1.run(n, 90)) * std::exp(-t / (far ? 2.0f : 1.4f)); // 90..420 Hz
            float sOut = 1.6f * blast * band + 1.8f * envBody * body + 0.32f * whump + 0.6f * deb + 2.2f * rumble;
            if (far) sOut = farLp2.run(farLp1.run(sOut, 1400), 1400) * 1.6f + 1.4f * rumble;
            o[i] = sOut;
        }
        break;
    }
    case Ev::Chute: { // mortar pop, canopy rushing out and snapping open
        o.resize(len(1.5f));
        for (size_t i = 0; i < o.size(); i++) {
            float t = (float)i / SR;
            float fc = 300 + 1600 * std::sin(std::min(1.0f, t / 1.2f) * 3.1416f);
            float w = r.next();
            float n = lp2.run(w, fc) - lp3.run(w, fc * 0.4f);
            float env = std::sin(std::min(1.0f, t / 1.4f) * 3.1416f);
            float flap = 0.7f + 0.3f * std::sin(TWO_PI * 17 * t);
            float pop = lp1.run(r.next(), 1500) * 3.0f * std::exp(-t / 0.01f) + std::sin(TWO_PI * 90 * t) * std::exp(-t / 0.06f);
            o[i] = 1.6f * n * env * flap + 0.6f * pop;
        }
        break;
    }
    case Ev::Touchdown: {
        o.resize(len(0.6f));
        for (size_t i = 0; i < o.size(); i++) {
            float t = (float)i / SR;
            o[i] = 0.9f * std::sin(TWO_PI * (62 + 30 * std::exp(-t / 0.04f)) * t) * std::exp(-t / 0.13f) +
                   2.5f * lp2.run(lp1.run(r.next(), 500), 500) * std::exp(-t / 0.06f);
        }
        break;
    }
    case Ev::HyperIn: { // rising, chorused sweep
        o.resize(len(3.0f));
        float p1 = 0, p2 = 0;
        for (size_t i = 0; i < o.size(); i++) {
            float t = (float)i / SR;
            float f = 60 * std::pow(20.0f, std::min(1.0f, t / 2.6f));
            p1 += TWO_PI * f / SR;
            p2 += TWO_PI * f * 1.007f / SR;
            float env = std::min(1.0f, t / 0.4f) * std::min(1.0f, (3.0f - t) / 0.25f);
            float air = lp1.run(r.next(), 400 + f) * std::min(1.0f, t / 2.0f);
            o[i] = env * (0.22f * (std::sin(p1) + std::sin(p2)) + 0.9f * air);
        }
        break;
    }
    case Ev::HyperOut: {
        o.resize(len(1.6f));
        float ph = 0;
        for (size_t i = 0; i < o.size(); i++) {
            float t = (float)i / SR;
            float f = 1200 * std::pow(0.06f, std::min(1.0f, t / 1.4f));
            ph += TWO_PI * f / SR;
            float env = std::exp(-t / 0.7f);
            o[i] = env * (0.3f * std::sin(ph) + 1.4f * lp1.run(r.next(), 200 + f)) + 0.8f * lp2.run(r.next(), 1500) * std::exp(-t / 0.02f);
        }
        break;
    }
    case Ev::Tear: { // panels ripping away: screech and clatter
        o.resize(len(0.9f));
        for (size_t i = 0; i < o.size(); i++) {
            float t = (float)i / SR;
            float w = r.next();
            float band = lp2.run(w, 3200) - lp3.run(w, 1800);
            float clang = std::sin(TWO_PI * 870 * t) * std::exp(-t / 0.2f) + std::sin(TWO_PI * 1460 * t) * std::exp(-t / 0.12f);
            o[i] = 2.0f * band * std::exp(-t / 0.3f) + 0.15f * clang;
        }
        break;
    }
    case Ev::Milestone: { // two-note chime (bell-like partials)
        o.resize(len(1.6f));
        const float notes[2] = {880.0f, 1318.5f}, start[2] = {0.0f, 0.16f};
        for (size_t i = 0; i < o.size(); i++) {
            float t = (float)i / SR, s = 0;
            for (int k = 0; k < 2; k++) {
                float tt = t - start[k];
                if (tt < 0) continue;
                float env = std::min(1.0f, tt / 0.004f) * std::exp(-tt / 0.45f);
                s += env * (std::sin(TWO_PI * notes[k] * tt) + 0.35f * std::sin(TWO_PI * notes[k] * 2.76f * tt) * std::exp(-tt / 0.15f) +
                            0.2f * std::sin(TWO_PI * notes[k] * 5.4f * tt) * std::exp(-tt / 0.08f));
            }
            o[i] = 0.5f * s;
        }
        break;
    }
    default: break;
    }
    // normalise to a common peak, then soft-clip
    float peak = 1e-6f;
    for (float v : o) peak = std::max(peak, std::fabs(v));
    float k = 0.9f / peak;
    for (float& v : o) v = std::tanh(v * k * 1.2f) * 0.85f;
    return o;
}

// ---------------------------------------------------------------- files (embedded resources, app.rc)
// effects: RCDATA 301 + event index (src/audio/fx/<name>.wav); music: RCDATA 321.. (src/audio/bgm/*.mp3)
constexpr int FX_RES = 301, BGM_RES = 321, BGM_MAX = 8;
bool loadEmbeddedSound(int id, Sound& out) {
    const void* d = nullptr;
    size_t n = 0;
    if (!winshell::resource(id, d, n)) return false;
    Wave w = LoadWaveFromMemory(".wav", (const unsigned char*)d, (int)n);
    if (!w.data) return false;
    out = LoadSoundFromWave(w);
    UnloadWave(w);
    return IsSoundValid(out);
}

// music: streamed from the embedded MP3s; a track, then a pause
struct Bgm {
    std::vector<Music> tracks;
    bool on = true;
    float volume = 0.5f, fade = 0;
    int playing = -1, last = -1;
    float pause = 0;   // seconds until the next track
    bool paused = false;
    uint32_t rng = 0;
    float rand01() {
        rng = rng * 1664525u + 1013904223u;
        return (rng >> 8) / 16777216.0f;
    }
} g_bgm;

Sound toSound(const std::vector<float>& s) {
    std::vector<int16_t> pcm(s.size());
    for (size_t i = 0; i < s.size(); i++) pcm[i] = (int16_t)std::clamp(s[i] * 32767.0f, -32767.0f, 32767.0f);
    Wave w{};
    w.frameCount = (unsigned)pcm.size();
    w.sampleRate = SR;
    w.sampleSize = 16;
    w.channels = 1;
    w.data = pcm.data();
    return LoadSoundFromWave(w); // copies the samples
}

// relative loudness of each event in the mix
float eventGain(Ev e) {
    switch (e) {
    case Ev::Click: return 0.35f;
    case Ev::Toggle: return 0.4f;
    case Ev::Touchdown: return 0.7f;
    case Ev::Explosion: return 1.0f;
    case Ev::Milestone: return 0.55f;
    default: return 0.9f;
    }
}

// ---------------------------------------------------------------- continuous mix (audio thread)
struct MixState {
    Rng r;
    LowPass eng1, eng2, engVac, wLo, wHi, rcsLp, rcsTop, brown;
    float engine = 0, vacuum = 0, crackle = 0, wind = 0, speed = 0, rcs = 0, crk = 0;
} g_mix;

void mixCallback(void* buffer, unsigned int frames) {
    float* out = (float*)buffer;
    MixState& m = g_mix;
    const float te = t_engine, tv = t_vacuum, tc = t_crackle, tw = t_wind, ts = t_speed, tr = t_rcs, mv = g_master;
    const float k = 1.0f - std::exp(-1.0f / (0.06f * SR)); // ~60 ms parameter smoothing (no zipper noise)
    for (unsigned i = 0; i < frames; i++) {
        m.engine += k * (te - m.engine);
        m.vacuum += k * (tv - m.vacuum);
        m.crackle += k * (tc - m.crackle);
        m.wind += k * (tw - m.wind);
        m.speed += k * (ts - m.speed);
        m.rcs += k * (tr - m.rcs);
        float s = 0;
        if (m.engine > 1e-4f) {
            float n = m.r.next();
            // roar in air, muffled low rumble through the structure in vacuum
            float fc = 1100.0f - 900.0f * m.vacuum;
            float roar = m.engVac.run(m.eng2.run(m.eng1.run(n, fc), fc * 0.8f), fc) * 1.6f;
            float rumble = m.brown.run(n, 90);
            if (m.r.next() > 0.9985f - 0.002f * m.crackle) m.crk = 1;
            m.crk *= 0.985f;
            float crk = m.crk * m.r.next() * m.crackle * (1 - m.vacuum);
            s += m.engine * ((2.2f * roar + 3.0f * rumble) * (1.0f - 0.7f * m.vacuum) + 0.35f * crk);
        }
        if (m.wind > 1e-4f) {
            float n = m.r.next();
            float hi = 250 + 2600 * std::min(1.0f, m.speed), lo = hi * 0.35f;
            s += m.wind * 2.0f * (m.wHi.run(n, hi) - m.wLo.run(n, lo));
        }
        if (m.rcs > 1e-4f) {
            float n = m.r.next();
            s += m.rcs * 0.6f * (m.rcsTop.run(n, 7000) - m.rcsLp.run(n, 1800)); // gas hiss, band-limited
        }
        out[i] = std::tanh(s * 1.1f) * 0.8f * mv;
    }
}
} // namespace

bool init() {
    InitAudioDevice();
    if (!IsAudioDeviceReady()) return false;
    for (int i = 0; i < (int)Ev::COUNT; i++) {
        // the effect file, or (without it) the synthesis it was made from
        if (loadEmbeddedSound(FX_RES + i, g_sounds[i])) g_fromFiles++;
        else g_sounds[i] = toSound(synth((Ev)i));
        g_loaded[i] = IsSoundValid(g_sounds[i]);
    }
    for (int k = 0; k < BGM_MAX; k++) {
        const void* d = nullptr;
        size_t n = 0;
        if (!winshell::resource(BGM_RES + k, d, n)) continue;
        Music m = LoadMusicStreamFromMemory(".mp3", (const unsigned char*)d, (int)n); // (streams from the resource)
        if (!IsMusicValid(m)) continue;
        m.looping = false;
        g_bgm.tracks.push_back(m);
    }
    g_bgm.rng = (uint32_t)(GetTime() * 1e6) ^ 0x5DEECE66u ^ (uint32_t)(size_t)&g_bgm;
    g_bgm.pause = 6 + 14 * g_bgm.rand01(); // the first track a little after the start
    SetAudioStreamBufferSizeDefault(1024); // ~23 ms latency
    g_stream = LoadAudioStream(SR, 32, 1);
    SetAudioStreamCallback(g_stream, mixCallback);
    PlayAudioStream(g_stream);
    g_ready = true;
    return true;
}

void shutdown() {
    if (!g_ready) return;
    for (Music& m : g_bgm.tracks) {
        StopMusicStream(m);
        UnloadMusicStream(m);
    }
    g_bgm.tracks.clear();
    StopAudioStream(g_stream);
    UnloadAudioStream(g_stream);
    for (int i = 0; i < (int)Ev::COUNT; i++)
        if (g_loaded[i]) UnloadSound(g_sounds[i]);
    CloseAudioDevice();
    g_ready = false;
}

bool dump(const char* dir) {
    static const char* names[] = {"click", "toggle", "stage", "ignite", "explosion", "chute", "touchdown", "hyper_in", "hyper_out", "tear", "milestone", "explosion_far"};
    auto write = [&](const std::string& name, const std::vector<float>& s) {
        std::vector<int16_t> pcm(s.size());
        for (size_t i = 0; i < s.size(); i++) pcm[i] = (int16_t)std::clamp(s[i] * 32767.0f, -32767.0f, 32767.0f);
        Wave w{};
        w.frameCount = (unsigned)pcm.size();
        w.sampleRate = SR;
        w.sampleSize = 16;
        w.channels = 1;
        w.data = pcm.data();
        return ExportWave(w, (std::string(dir) + "/" + name + ".wav").c_str());
    };
    bool ok = true;
    for (int i = 0; i < (int)Ev::COUNT; i++) ok &= write(names[i], synth((Ev)i));
    struct Case { const char* name; Ambient a; };
    Case cases[] = {{"mix_engine_air", {1, 0, 0.5f, 0, 0, 0}}, {"mix_engine_vacuum", {1, 1, 0.5f, 0, 0, 0}},
                    {"mix_wind", {0, 0, 0, 1, 0.5f, 0}}, {"mix_rcs", {0, 1, 0, 0, 0, 1}}};
    g_master = 1;
    for (auto& c : cases) {
        g_mix = MixState{};
        setAmbient(c.a);
        std::vector<float> s(SR * 2);
        mixCallback(s.data(), (unsigned)s.size());
        ok &= write(c.name, s);
    }
    setAmbient({});
    return ok;
}

void setVolume(float v) { g_master = std::clamp(v, 0.0f, 1.0f); }

void play(Ev e, float gain) {
    int i = (int)e;
    if (!g_ready || i < 0 || i >= (int)Ev::COUNT || !g_loaded[i] || g_master <= 0) return;
    SetSoundVolume(g_sounds[i], std::clamp(gain * eventGain(e) * g_master, 0.0f, 1.0f));
    PlaySound(g_sounds[i]);
}

void setMusic(bool on, float volume) {
    g_bgm.on = on;
    g_bgm.volume = std::clamp(volume, 0.0f, 1.0f);
    if (!on && g_bgm.playing >= 0) {
        StopMusicStream(g_bgm.tracks[g_bgm.playing]);
        g_bgm.last = g_bgm.playing;
        g_bgm.playing = -1;
        g_bgm.pause = 20 + 40 * g_bgm.rand01();
    }
}

void pauseMusic(bool paused) {
    if (!g_ready || paused == g_bgm.paused) return;
    g_bgm.paused = paused;
    if (g_bgm.playing < 0) return;
    if (paused) PauseMusicStream(g_bgm.tracks[g_bgm.playing]);
    else ResumeMusicStream(g_bgm.tracks[g_bgm.playing]);
}

void update(float dt) {
    if (!g_ready || g_bgm.tracks.empty() || g_bgm.paused) return;
    dt = std::clamp(dt, 0.0f, 0.25f);
    Bgm& b = g_bgm;
    if (b.playing >= 0) {
        Music& m = b.tracks[b.playing];
        UpdateMusicStream(m);
        float len = GetMusicTimeLength(m), at = GetMusicTimePlayed(m);
        b.fade = std::min(1.0f, b.fade + dt / 3.0f); // fade in over 3 s
        float tail = len > 0 ? std::clamp((len - at) / 4.0f, 0.0f, 1.0f) : 1.0f; // and out over the last 4 s
        SetMusicVolume(m, b.volume * b.fade * tail);
        if (!IsMusicStreamPlaying(m) || (len > 0 && at >= len - 0.05f)) {
            StopMusicStream(m);
            b.last = b.playing;
            b.playing = -1;
            b.pause = 90 + 150 * b.rand01(); // 1.5 - 4 minutes of quiet before the next one
        }
        return;
    }
    if (!b.on || b.volume <= 0) return;
    b.pause -= dt;
    if (b.pause > 0) return;
    int n = (int)b.tracks.size(), k = std::min(n - 1, (int)(b.rand01() * n));
    if (n > 1 && k == b.last) k = (k + 1 + (int)(b.rand01() * (n - 1))) % n; // not the same one twice in a row
    b.playing = k;
    b.fade = 0;
    SeekMusicStream(b.tracks[k], 0);
    SetMusicVolume(b.tracks[k], 0);
    PlayMusicStream(b.tracks[k]);
}

int musicTracks() { return (int)g_bgm.tracks.size(); }
int musicPlaying() { return g_bgm.playing; }
float musicPause() { return g_bgm.playing >= 0 ? 0.0f : g_bgm.pause; }
void musicDebugSkipPause() { g_bgm.pause = 0; }
void musicDebugSeekEnd() {
    if (g_bgm.playing < 0) return;
    Music& m = g_bgm.tracks[g_bgm.playing];
    SeekMusicStream(m, std::max(0.0f, GetMusicTimeLength(m) - 1.0f));
}
float musicLength(int k) { return k >= 0 && k < (int)g_bgm.tracks.size() ? GetMusicTimeLength(g_bgm.tracks[k]) : 0.0f; }
int effectsFromFiles() { return g_fromFiles; }

void setAmbient(const Ambient& a) {
    auto c = [](float v) { return std::isfinite(v) ? std::clamp(v, 0.0f, 1.0f) : 0.0f; };
    t_engine = c(a.engine);
    t_vacuum = c(a.vacuum);
    t_crackle = c(a.crackle);
    t_wind = c(a.wind);
    t_speed = c(a.speed);
    t_rcs = c(a.rcs);
}
} // namespace sfx
