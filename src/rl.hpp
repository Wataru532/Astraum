// The bridge to raylib for the outer layers (infrastructure, presentation, main): raylib, raymath and rlgl, and
// the conversions from the domain's double-precision types. The inner layers never include this file (their
// libraries cannot see it).
#pragma once
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
// raylib defines PI as a float macro: the domain's double constant takes its place
#ifdef PI
#undef PI
#endif
#include "mathx.hpp"
#include "types.hpp"

inline Vector3 toF(const Vec3d& v) { return {(float)v.x, (float)v.y, (float)v.z}; }

// raylib Matrix (column-major) from rotation + translation + scale
inline Matrix toMatrix(const Mat3d& R, const Vec3d& t = {}, double sx = 1, double sy = 1, double sz = 1) {
    Matrix m{};
    m.m0 = (float)(R.m[0][0] * sx); m.m4 = (float)(R.m[0][1] * sy); m.m8 = (float)(R.m[0][2] * sz); m.m12 = (float)t.x;
    m.m1 = (float)(R.m[1][0] * sx); m.m5 = (float)(R.m[1][1] * sy); m.m9 = (float)(R.m[1][2] * sz); m.m13 = (float)t.y;
    m.m2 = (float)(R.m[2][0] * sx); m.m6 = (float)(R.m[2][1] * sy); m.m10 = (float)(R.m[2][2] * sz); m.m14 = (float)t.z;
    m.m3 = 0; m.m7 = 0; m.m11 = 0; m.m15 = 1;
    return m;
}
