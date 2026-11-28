#include "base.hpp"
#include <algorithm>
#include <cmath>
#include "i18n.hpp"
#include "parts.hpp"
#include "sim.hpp"

const char* moduleKindName(ModuleKind k) {
    switch (k) {
    case ModuleKind::Habitat: return "Habitat";
    case ModuleKind::Greenhouse: return "Greenhouse";
    case ModuleKind::Isru: return "ISRU propellant plant";
    case ModuleKind::Fission: return "Fission power plant";
    case ModuleKind::Solar: return "Solar array field";
    case ModuleKind::Storage: return "Propellant storage";
    case ModuleKind::Comms: return "Communications mast";
    case ModuleKind::LaunchPad: return "Launch pad";
    default: return "";
    }
}

BaseStatus baseStatus(const SurfaceBase& b) {
    BaseStatus s;
    for (const BaseModule& m : b.modules) {
        switch (m.kind) {
        case ModuleKind::Habitat:
            s.habitats++;
            s.crew += 4;
            s.demandKw += 8;
            break;
        case ModuleKind::Greenhouse:
            s.greenhouses++;
            s.demandKw += 6;
            break;
        case ModuleKind::Isru:
            s.isru++;
            s.demandKw += 25;
            break;
        case ModuleKind::LaunchPad: s.hasPad = true; break;
        default: break;
        }
        s.powerKw += m.kw;
    }
    return s;
}

bool baseAllowed(const Universe& U, int body, std::string* why) {
    auto no = [&](const char* w) {
        if (why) *why = w;
        return false;
    };
    if (body < 0 || body >= U.count()) return no("No body");
    const Body& b = U.bodies[body];
    if (b.isStar) return no("Nothing can be built on a star");
    if (b.gasGiant) return no("A gas giant has no solid ground to build on");
    if (b.comet) return no("A comet's nucleus is too small and too active for a base");
    if (b.name == "Earth") return no("Earth has its launch sites already");
    if (b.name == "Venus") return no("The surface of Venus is too hot for a base (460 C, 92 bar)");
    if (why) why->clear();
    return true;
}

Vec3d baseCentreDir(const SurfaceBase& b) {
    return {std::cos(b.lat) * std::cos(b.lon), std::cos(b.lat) * std::sin(b.lon), std::sin(b.lat)};
}

void baseFrame(const SurfaceBase& b, Vec3d& east, Vec3d& north, Vec3d& up) {
    up = baseCentreDir(b);
    Vec3d z{0, 0, 1};
    // (at a pole the local east is undefined: any horizontal axis does)
    east = len(cross(z, up)) < 1e-9 ? Vec3d{0, 1, 0} : norm(cross(z, up));
    north = cross(up, east);
}

Vec3d baseDir(const SurfaceBase& b, double bodyRadius, double x, double y) {
    Vec3d e, n, u;
    baseFrame(b, e, n, u);
    double R = std::max(bodyRadius, 1.0);
    return norm(u + e * (x / R) + n * (y / R));
}

void dirToLatLon(const Vec3d& d, double& lat, double& lon) {
    Vec3d n = norm(d);
    lat = std::asin(std::clamp(n.z, -1.0, 1.0));
    lon = std::atan2(n.y, n.x);
}

namespace {

// a stable small hash: the look of a module depends on the base and on its place, not on chance
unsigned mix(unsigned a, unsigned b) {
    unsigned h = a * 0x9E3779B1u ^ (b + 0x7F4A7C15u + (a << 6) + (a >> 2));
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    return h;
}

int countKind(const SurfaceBase& b, ModuleKind k) {
    int n = 0;
    for (const BaseModule& m : b.modules) n += m.kind == k;
    return n;
}

// spiral of slots around (cx, cy): the k-th one at radius r0 * sqrt(k) (golden angle apart), so a cluster grows
// evenly and its members never overlap
void spiral(double cx, double cy, double r0, int k, double& x, double& y) {
    if (k == 0) {
        x = cx;
        y = cy;
        return;
    }
    double a = k * 2.39996323 + 0.6, r = r0 * std::sqrt((double)k);
    x = cx + r * std::cos(a);
    y = cy + r * std::sin(a);
}

} // namespace

void placeModule(SurfaceBase& b, BaseModule m) {
    int k = countKind(b, m.kind);
    unsigned h = mix((unsigned)b.id * 31u + (unsigned)m.kind, (unsigned)k);
    double x = 0, y = 0, heading = (h % 3600) * (PI / 1800.0);
    // the layout: living quarters in the middle, the greenhouses beside them (joined by tunnels when drawn), the
    // propellant plant and its tanks downwind, solar fields to the south, the reactors a safe distance away (shielded
    // by distance) and the launch pad far enough for its blast
    switch (m.kind) {
    case ModuleKind::Habitat: spiral(0, 0, 34, k, x, y); break;
    case ModuleKind::Greenhouse: spiral(-62, 30, 24, k, x, y); break;
    case ModuleKind::Isru: spiral(120, 58, 30, k, x, y); break;
    case ModuleKind::Storage:
        x = 150 + (k % 4) * 15.0;
        y = 100 + (k / 4) * 15.0;
        heading = 0;
        break;
    case ModuleKind::Fission: spiral(-330, -250, 46, k, x, y); break;
    case ModuleKind::Solar:
        x = -70 + (k % 4) * 46.0;
        y = -120 - (k / 4) * 34.0;
        heading = 0;
        break;
    case ModuleKind::Comms: spiral(34, 46, 30, k, x, y); break;
    case ModuleKind::LaunchPad:
        x = 520 + k * 160.0;
        y = -80;
        heading = 0;
        break;
    default: break;
    }
    if (m.kind == ModuleKind::Habitat || m.kind == ModuleKind::Greenhouse) heading = std::atan2(-y, -x + 1e-9); // (doors toward the middle)
    m.x = x;
    m.y = y;
    m.heading = heading;
    m.variant = (int)(h % 3);
    if ((int)b.modules.size() < BASE_MAX_MODULES) b.modules.push_back(m);
}

bool completeLaunchComplex(SurfaceBase& b) {
    BaseStatus s = baseStatus(b);
    if (s.hasPad || !s.requirementsMet()) return false;
    if ((int)b.modules.size() >= BASE_MAX_MODULES) return false;
    placeModule(b, {ModuleKind::LaunchPad});
    for (int i = 0; i < 4; i++) placeModule(b, {ModuleKind::Storage});
    return baseStatus(b).hasPad;
}

bool padOffset(const SurfaceBase& b, double& x, double& y) {
    for (const BaseModule& m : b.modules)
        if (m.kind == ModuleKind::LaunchPad) {
            x = m.x;
            y = m.y;
            return true;
        }
    return false;
}

int findBase(const World& W, int id) {
    for (int i = 0; i < (int)W.bases.size(); i++)
        if (W.bases[i].id == id) return i;
    return -1;
}

int baseNear(const World& W, int body, const Vec3d& d, double maxDist) {
    if (body < 0 || body >= W.U.count()) return -1;
    double R = W.U.bodies[body].radius, best = maxDist;
    int idx = -1;
    Vec3d n = norm(d);
    for (int i = 0; i < (int)W.bases.size(); i++) {
        const SurfaceBase& b = W.bases[i];
        if (b.body != body) continue;
        double dist = std::acos(std::clamp(dot(n, baseCentreDir(b)), -1.0, 1.0)) * R;
        if (dist <= best) {
            best = dist;
            idx = i;
        }
    }
    return idx;
}

std::string defaultBaseName(const World& W, int body) {
    int n = 1;
    for (const SurfaceBase& b : W.bases) n += b.body == body;
    std::string nm = body >= 0 && body < W.U.count() ? W.U.bodies[body].name : std::string("Surface");
    return nm + " Base " + std::to_string(n);
}

namespace {
// sunlight at the body as a fraction of the solar constant at Earth (1361 W/m^2)
double fluxScale(const World& W, int body) {
    const Universe& U = W.U;
    if (body < 0 || body >= U.count() || body >= (int)W.states.size()) return 1.0;
    const Body& star = U.bodies[U.bodies[body].root];
    double d = len(W.states[body].pos);
    if (d < 1e6 || star.luminosity <= 0) return 1.0;
    return star.luminosity / (4 * PI * d * d) / 1361.0;
}
} // namespace

BasePlan planBase(const World& W, int vi) {
    BasePlan p;
    if (vi < 0 || vi >= (int)W.vessels.size()) {
        p.why = "No vessel";
        return p;
    }
    const Vessel& v = W.vessels[vi];
    if (v.destroyed) {
        p.why = "No vessel";
        return p;
    }
    if (!baseAllowed(W.U, v.ref, &p.why)) return p;
    if (!v.landed) {
        p.why = "The vessel must stand on the ground";
        return p;
    }
    // the modules: catalog parts made for it, and the power sources a lander carries
    double solarKw = 0;
    std::vector<int> solarComps;
    for (int i = 0; i < (int)v.comps.size(); i++) {
        const Component& c = v.comps[i];
        if (c.def >= 0 && c.def < (int)CATALOG.defs.size() && CATALOG.defs[c.def].kind == PK::Module) {
            const PartDef& d = CATALOG.defs[c.def];
            if (d.baseModule < 0 || d.baseModule >= (int)ModuleKind::COUNT) continue;
            BaseModule m;
            m.kind = (ModuleKind)d.baseModule;
            m.kw = d.moduleKw;
            p.modules.push_back(m);
            p.comps.push_back(i);
        } else if (c.func == Func::Reactor && i < (int)v.phys.size() && v.phys[i].genW > 0) {
            BaseModule m;
            m.kind = ModuleKind::Fission;
            m.kw = v.phys[i].genW / 1000.0;
            p.modules.push_back(m);
            p.comps.push_back(i);
        } else if (c.func == Func::Solar && i < (int)v.phys.size() && v.phys[i].solarArea > 0) {
            solarKw += v.phys[i].solarArea * 1361.0 * fluxScale(W, v.ref) / 1000.0;
            solarComps.push_back(i);
        }
    }
    // solar panels are set up on the ground as one field per ~10 kW (at least one)
    if (!solarComps.empty()) {
        int n = std::clamp((int)std::ceil(solarKw / 10.0), 1, 8);
        for (int k = 0; k < n; k++) {
            BaseModule m;
            m.kind = ModuleKind::Solar;
            m.kw = solarKw / n;
            p.modules.push_back(m);
        }
        p.comps.insert(p.comps.end(), solarComps.begin(), solarComps.end());
    }
    bool hasHab = false, hasModulePart = false;
    for (const BaseModule& m : p.modules) {
        hasHab |= m.kind == ModuleKind::Habitat;
        hasModulePart |= m.kind == ModuleKind::Habitat || m.kind == ModuleKind::Greenhouse || m.kind == ModuleKind::Isru ||
                         m.kind == ModuleKind::Fission || m.kind == ModuleKind::Solar;
    }
    if (p.modules.empty() || !hasModulePart) {
        p.why = "The vessel carries no base modules";
        return p;
    }
    Vec3d sv = W.surfaceVel(v);
    if (len(sv) > 0.5) {
        p.why = "The vessel must stand still";
        return p;
    }
    p.joins = baseNear(W, v.ref, v.landPos, BASE_REACH);
    if (p.joins < 0) {
        if (!hasHab) {
            p.why = "A new base needs a habitat module";
            return p;
        }
        if ((int)W.bases.size() >= BASE_MAX) {
            p.why = "Too many bases";
            return p;
        }
    } else {
        SurfaceBase preview = W.bases[p.joins];
        BaseStatus before = baseStatus(preview);
        for (const BaseModule& m : p.modules) preview.modules.push_back(m);
        int extra = (!before.hasPad && baseStatus(preview).requirementsMet()) ? 5 : 0;
        if ((int)(W.bases[p.joins].modules.size() + p.modules.size()) + extra > BASE_MAX_MODULES) {
            p.why = "The base is full";
            return p;
        }
    }
    std::sort(p.comps.begin(), p.comps.end());
    p.comps.erase(std::unique(p.comps.begin(), p.comps.end()), p.comps.end());
    p.ok = true;
    return p;
}

int buildBase(World& W, int vi, const std::string& name) {
    BasePlan p = planBase(W, vi);
    if (!p.ok) return -1;
    Vessel& v = W.vessels[vi];
    int idx = p.joins;
    if (idx < 0) {
        SurfaceBase b;
        b.id = W.nextBaseId++;
        b.body = v.ref;
        // the centre a little away from the lander (it stays where it landed, at the edge of the base)
        {
            Vec3d up = norm(v.landPos), z{0, 0, 1};
            Vec3d e = len(cross(z, up)) < 1e-9 ? Vec3d{0, 1, 0} : norm(cross(z, up)), n = cross(up, e);
            double R = std::max(W.U.bodies[v.ref].radius, 1.0);
            dirToLatLon(norm(up + e * (-40.0 / R) + n * (50.0 / R)), b.lat, b.lon);
        }
        b.founded = W.t;
        std::string cleanName = cleanLineText(name, 48);
        b.name = cleanName.empty() ? defaultBaseName(W, v.ref) : cleanName;
        placeModule(b, {ModuleKind::Comms});
        W.bases.push_back(b);
        idx = (int)W.bases.size() - 1;
    }
    SurfaceBase& b = W.bases[idx];
    for (const BaseModule& m : p.modules) placeModule(b, m);
    bool padNow = completeLaunchComplex(b);
    std::string bodyName = W.U.bodies[b.body].name;
    // the ground of the base is levelled: what stands on it nearby keeps standing on it (by the change of the
    // ground under it), then the lander gives up what it unloaded (when nothing is left of it, it is gone)
    std::vector<double> before(W.vessels.size(), 0.0);
    for (size_t i = 0; i < W.vessels.size(); i++)
        if (W.vessels[i].landed && W.vessels[i].ref == b.body) before[i] = W.groundHeight(b.body, W.vessels[i].landPos);
    W.syncBaseTerrain();
    for (size_t i = 0; i < W.vessels.size(); i++) {
        Vessel& o = W.vessels[i];
        if (!o.landed || o.ref != b.body || (int)i == vi) continue;
        double d = W.groundHeight(b.body, o.landPos) - before[i];
        if (std::fabs(d) > 1e-6) W.moveLanded(o, norm(o.landPos) * d);
    }
    W.unloadComponents(vi, p.comps);
    if (p.joins < 0) W.toast(trf("Base founded: {0} on {1}", b.name, tr(bodyName)), {120, 220, 160, 255});
    else W.toast(trf("{0} modules added to {1}", std::to_string(p.modules.size()), b.name), {120, 220, 160, 255});
    if (padNow) W.toast(trf("{0} can now launch rockets: its launch pad is ready", b.name), {120, 220, 160, 255});
    return idx;
}
