// GPU timing with GL_TIMESTAMP queries. The functions are fetched with wglGetProcAddress (OpenGL 3.3 core)
// so that raylib's loader stays untouched; results are read four frames later (no pipeline stall).
#include "gputime.hpp"
#include <chrono>
#include <cstdint>
#include <algorithm>
#include <map>
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include "rlgl.h"

namespace gputime {
namespace {
typedef unsigned int GLuint;
typedef unsigned long long GLuint64;
typedef int GLint;
typedef int GLsizei;
typedef unsigned int GLenum;
const GLenum GL_TIMESTAMP_ = 0x8E28, GL_QUERY_RESULT_ = 0x8866, GL_QUERY_RESULT_AVAILABLE_ = 0x8867;
typedef void(APIENTRY* PGenQueries)(GLsizei, GLuint*);
typedef void(APIENTRY* PDeleteQueries)(GLsizei, const GLuint*);
typedef void(APIENTRY* PQueryCounter)(GLuint, GLenum);
typedef void(APIENTRY* PGetQueryObjectui64v)(GLuint, GLenum, GLuint64*);
typedef void(APIENTRY* PGetQueryObjectiv)(GLuint, GLenum, GLint*);
PGenQueries genQueries = nullptr;
PDeleteQueries deleteQueries = nullptr;
PQueryCounter queryCounter = nullptr;
PGetQueryObjectui64v getU64 = nullptr;
PGetQueryObjectiv getIv = nullptr;
bool tried = false, ok = false, enabled = false;

bool validGlProc(void* p) {
    uintptr_t u = (uintptr_t)p;
    return u > 3 && u != (uintptr_t)-1;
}

const int FRAMES = 5, MAXQ = 64;
struct Frame {
    GLuint q[MAXQ] = {};
    const char* name[MAXQ] = {};
    double cpu[MAXQ] = {};
    int n = 0;
    bool pending = false;
};
Frame fr[FRAMES];
int cur = 0;
bool inFrame = false;
struct Acc { double gpu = 0, cpu = 0; int n = 0; int order = 0; };
std::map<std::string, Acc> acc;
int orderNext = 0, framesDone = 0;

double nowMs() {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

bool load() {
    if (tried) return ok;
    tried = true;
    void* pGen = (void*)wglGetProcAddress("glGenQueries");
    void* pDel = (void*)wglGetProcAddress("glDeleteQueries");
    void* pCnt = (void*)wglGetProcAddress("glQueryCounter");
    void* pU64 = (void*)wglGetProcAddress("glGetQueryObjectui64v");
    void* pIv = (void*)wglGetProcAddress("glGetQueryObjectiv");
    if (validGlProc(pGen) && validGlProc(pCnt) && validGlProc(pU64) && validGlProc(pIv)) {
        genQueries = (PGenQueries)pGen;
        deleteQueries = validGlProc(pDel) ? (PDeleteQueries)pDel : nullptr;
        queryCounter = (PQueryCounter)pCnt;
        getU64 = (PGetQueryObjectui64v)pU64;
        getIv = (PGetQueryObjectiv)pIv;
        ok = true;
        for (Frame& f : fr) genQueries(MAXQ, f.q);
    }
    return ok;
}

void collect(Frame& f) {
    if (!f.pending) return;
    GLint avail = 0;
    getIv(f.q[f.n - 1], GL_QUERY_RESULT_AVAILABLE_, &avail);
    if (!avail) return; // (still not done: try next time)
    GLuint64 t[MAXQ];
    for (int i = 0; i < f.n; i++) getU64(f.q[i], GL_QUERY_RESULT_, &t[i]);
    for (int i = 1; i < f.n; i++) {
        Acc& a = acc[f.name[i]];
        if (a.n == 0 && a.gpu == 0) a.order = orderNext++;
        a.gpu += t[i] >= t[i - 1] ? (double)(t[i] - t[i - 1]) * 1e-6 : 0.0;
        a.cpu += std::max(0.0, f.cpu[i] - f.cpu[i - 1]);
        a.n++;
    }
    framesDone++;
    f.pending = false;
}
} // namespace

bool available() { return load(); }

void setEnabled(bool on) { enabled = on; }

void frameBegin() {
    inFrame = false;
    if (!enabled || !load()) return;
    cur = (cur + 1) % FRAMES;
    Frame& f = fr[cur];
    collect(f);
    if (f.pending) { // (the GPU is far behind: drop this frame's timing)
        inFrame = false;
        return;
    }
    f.n = 0;
    inFrame = true;
    mark("start");
    for (int k = 1; k < FRAMES; k++) collect(fr[(cur + k) % FRAMES]);
}

void mark(const char* section) {
    if (!inFrame) return;
    Frame& f = fr[cur];
    if (f.n >= MAXQ) return;
    rlDrawRenderBatchActive(); // (the batched draws belong to the part before)
    queryCounter(f.q[f.n], GL_TIMESTAMP_);
    f.name[f.n] = section;
    f.cpu[f.n] = nowMs();
    f.n++;
}

void frameEnd() {
    if (!inFrame) return;
    mark("rest");
    fr[cur].pending = fr[cur].n > 1;
    inFrame = false;
}

std::vector<Part> averages() {
    std::vector<std::pair<int, Part>> v;
    for (auto& e : acc)
        if (e.second.n > 0) v.push_back({e.second.order, Part{e.first, e.second.gpu / std::max(framesDone, 1), e.second.cpu / std::max(framesDone, 1)}}); // (per frame: a part may come several times)
    std::sort(v.begin(), v.end(), [](auto& a, auto& b) { return a.first < b.first; });
    std::vector<Part> out;
    for (auto& p : v) out.push_back(p.second);
    return out;
}

void reset() {
    acc.clear();
    orderNext = 0;
    framesDone = 0;
    inFrame = false;
    for (Frame& f : fr) {
        f.pending = false;
        f.n = 0;
    }
}

void shutdown() {
    reset();
    if (ok && deleteQueries) {
        for (Frame& f : fr) {
            deleteQueries(MAXQ, f.q);
            for (GLuint& q : f.q) q = 0;
        }
    }
    ok = false;
    tried = false;
    enabled = false;
}
} // namespace gputime
