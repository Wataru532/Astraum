// Small value types of the domain that the presentation draws with: a colour and a float vector. The domain
// does not know raylib; its own types convert to and from raylib's Color / Vector3 (any struct of the same
// fields) implicitly, so the outer layers pass them straight to drawing calls.
#pragma once
#include <cstdint>
#include <type_traits>

struct Rgba8 {
    uint8_t r = 0, g = 0, b = 0, a = 255;
    constexpr Rgba8() = default;
    constexpr Rgba8(uint8_t r_, uint8_t g_, uint8_t b_, uint8_t a_ = 255) : r(r_), g(g_), b(b_), a(a_) {}
    // from another RGBA struct of bytes (raylib's Color)
    template <class C, std::enable_if_t<!std::is_same_v<C, Rgba8> && std::is_same_v<decltype(C::r), unsigned char> &&
                                            std::is_same_v<decltype(C::a), unsigned char>, int> = 0>
    constexpr Rgba8(const C& c) : r(c.r), g(c.g), b(c.b), a(c.a) {}
    template <class C, std::enable_if_t<!std::is_same_v<C, Rgba8> && std::is_same_v<decltype(C::r), unsigned char> &&
                                            std::is_same_v<decltype(C::a), unsigned char>, int> = 0>
    constexpr operator C() const {
        return C{r, g, b, a};
    }
};

struct Vec3f {
    float x = 0, y = 0, z = 0;
    constexpr Vec3f() = default;
    constexpr Vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    // from / to another struct of three floats x, y, z (raylib's Vector3)
    template <class V, std::enable_if_t<!std::is_same_v<V, Vec3f> && std::is_same_v<decltype(V::x), float> && std::is_same_v<decltype(V::z), float> &&
                                            sizeof(V) == 3 * sizeof(float), int> = 0>
    constexpr Vec3f(const V& v) : x(v.x), y(v.y), z(v.z) {}
    template <class V, std::enable_if_t<!std::is_same_v<V, Vec3f> && std::is_same_v<decltype(V::x), float> && std::is_same_v<decltype(V::z), float> &&
                                            sizeof(V) == 3 * sizeof(float), int> = 0>
    constexpr operator V() const {
        return V{x, y, z};
    }
};
