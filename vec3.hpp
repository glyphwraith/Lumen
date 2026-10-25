#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace lumen {

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator-() const { return {-x, -y, -z}; }
    double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator*=(double t) { x *= t; y *= t; z *= t; return *this; }
    double length_squared() const { return x * x + y * y + z * z; }
    double length() const { return std::sqrt(length_squared()); }
    bool near_zero() const { const double s = 1e-8; return std::fabs(x) < s && std::fabs(y) < s && std::fabs(z) < s; }
    double max_component() const { return std::max({x, y, z}); }
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3& a, const Vec3& b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline Vec3 operator*(double t, const Vec3& v) { return {t * v.x, t * v.y, t * v.z}; }
inline Vec3 operator*(const Vec3& v, double t) { return t * v; }
inline Vec3 operator/(const Vec3& v, double t) { return (1.0 / t) * v; }

inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Vec3 unit(const Vec3& v) { return v / v.length(); }
inline Vec3 reflect(const Vec3& v, const Vec3& n) { return v - 2 * dot(v, n) * n; }
inline Vec3 refract(const Vec3& uv, const Vec3& n, double eta_ratio) {
    double cos_t = std::fmin(dot(-uv, n), 1.0);
    Vec3 perp = eta_ratio * (uv + cos_t * n);
    Vec3 par = -std::sqrt(std::fabs(1.0 - perp.length_squared())) * n;
    return perp + par;
}

using Color = Vec3;
using Point = Vec3;

// Small, fast xorshift64* generator. One instance per thread.
class Rng {
public:
    explicit Rng(uint64_t seed) : s_(seed * 0x9E3779B97F4A7C15ULL + 0xD1B54A32D192ED03ULL) {
        if (!s_) s_ = 1;
        next(); next();
    }
    uint64_t next() {
        s_ ^= s_ >> 12; s_ ^= s_ << 25; s_ ^= s_ >> 27;
        return s_ * 0x2545F4914F6CDD1DULL;
    }
    double uniform() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
    double uniform(double a, double b) { return a + (b - a) * uniform(); }
    Vec3 random_color() { return {uniform(), uniform(), uniform()}; }
    Vec3 unit_vector() {
        double z = uniform(-1, 1), phi = uniform(0, 6.283185307179586);
        double r = std::sqrt(std::max(0.0, 1 - z * z));
        return {r * std::cos(phi), r * std::sin(phi), z};
    }
    Vec3 in_unit_sphere() {
        for (;;) {
            Vec3 p(uniform(-1, 1), uniform(-1, 1), uniform(-1, 1));
            if (p.length_squared() < 1) return p;
        }
    }
    Vec3 in_unit_disk() {
        for (;;) {
            Vec3 p(uniform(-1, 1), uniform(-1, 1), 0);
            if (p.length_squared() < 1) return p;
        }
    }
private:
    uint64_t s_;
};

}  // namespace lumen
