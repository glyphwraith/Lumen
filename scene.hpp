// Rays, materials, primitives and the BVH acceleration structure.
#pragma once
#include <algorithm>
#include <limits>
#include <memory>
#include <vector>
#include "vec3.hpp"

namespace lumen {

constexpr double kInf = std::numeric_limits<double>::infinity();

struct Ray {
    Point o;
    Vec3 d;
    Point at(double t) const { return o + t * d; }
};

class Material;

struct Hit {
    Point p;
    Vec3 n;                 // always faces against the incoming ray
    double t = 0;
    bool front = true;
    const Material* mat = nullptr;
};

// ---------------------------------------------------------------- materials
class Material {
public:
    virtual ~Material() = default;
    virtual bool scatter(const Ray&, const Hit&, Rng&, Color&, Ray&) const { return false; }
    virtual Color emitted() const { return {0, 0, 0}; }
};

class Lambertian : public Material {
public:
    explicit Lambertian(Color a) : albedo_(a) {}
    bool scatter(const Ray&, const Hit& h, Rng& rng, Color& att, Ray& out) const override {
        Vec3 dir = h.n + rng.unit_vector();
        if (dir.near_zero()) dir = h.n;
        out = {h.p, dir};
        att = albedo_;
        return true;
    }
private:
    Color albedo_;
};

class Metal : public Material {
public:
    Metal(Color a, double fuzz) : albedo_(a), fuzz_(std::min(fuzz, 1.0)) {}
    bool scatter(const Ray& in, const Hit& h, Rng& rng, Color& att, Ray& out) const override {
        Vec3 r = reflect(unit(in.d), h.n);
        out = {h.p, r + fuzz_ * rng.in_unit_sphere()};
        att = albedo_;
        return dot(out.d, h.n) > 0;
    }
private:
    Color albedo_;
    double fuzz_;
};

class Dielectric : public Material {
public:
    explicit Dielectric(double ior) : ior_(ior) {}
    bool scatter(const Ray& in, const Hit& h, Rng& rng, Color& att, Ray& out) const override {
        att = {1, 1, 1};
        double ratio = h.front ? 1.0 / ior_ : ior_;
        Vec3 u = unit(in.d);
        double cos_t = std::fmin(dot(-u, h.n), 1.0);
        double sin_t = std::sqrt(1.0 - cos_t * cos_t);
        bool total_internal = ratio * sin_t > 1.0;
        Vec3 dir = (total_internal || schlick(cos_t, ratio) > rng.uniform()) ? reflect(u, h.n) : refract(u, h.n, ratio);
        out = {h.p, dir};
        return true;
    }
private:
    static double schlick(double cosine, double ratio) {
        double r0 = (1 - ratio) / (1 + ratio);
        r0 *= r0;
        return r0 + (1 - r0) * std::pow(1 - cosine, 5);
    }
    double ior_;
};

class DiffuseLight : public Material {
public:
    explicit DiffuseLight(Color c) : color_(c) {}
    Color emitted() const override { return color_; }
private:
    Color color_;
};

// ---------------------------------------------------------------- bounding boxes
struct AABB {
    Point lo{kInf, kInf, kInf}, hi{-kInf, -kInf, -kInf};
    static AABB merge(const AABB& a, const AABB& b) {
        AABB r;
        r.lo = {std::min(a.lo.x, b.lo.x), std::min(a.lo.y, b.lo.y), std::min(a.lo.z, b.lo.z)};
        r.hi = {std::max(a.hi.x, b.hi.x), std::max(a.hi.y, b.hi.y), std::max(a.hi.z, b.hi.z)};
        return r;
    }
    bool hit(const Ray& r, double tmin, double tmax) const {
        for (int a = 0; a < 3; ++a) {
            double inv = 1.0 / r.d[a];
            double t0 = (lo[a] - r.o[a]) * inv, t1 = (hi[a] - r.o[a]) * inv;
            if (inv < 0) std::swap(t0, t1);
            tmin = std::max(t0, tmin);
            tmax = std::min(t1, tmax);
            if (tmax <= tmin) return false;
        }
        return true;
    }
    int longest_axis() const {
        Vec3 e = hi - lo;
        return (e.x > e.y && e.x > e.z) ? 0 : (e.y > e.z ? 1 : 2);
    }
};

// ---------------------------------------------------------------- primitives
class Hittable {
public:
    virtual ~Hittable() = default;
    virtual bool hit(const Ray& r, double tmin, double tmax, Hit& h) const = 0;
    virtual AABB bounds() const = 0;
};

class Sphere : public Hittable {
public:
    Sphere(Point c, double r, std::shared_ptr<Material> m) : c_(c), r_(r), m_(std::move(m)) {}
    bool hit(const Ray& ray, double tmin, double tmax, Hit& h) const override {
        Vec3 oc = ray.o - c_;
        double a = ray.d.length_squared(), hb = dot(oc, ray.d), c = oc.length_squared() - r_ * r_;
        double disc = hb * hb - a * c;
        if (disc < 0) return false;
        double sq = std::sqrt(disc);
        double root = (-hb - sq) / a;
        if (root < tmin || root > tmax) {
            root = (-hb + sq) / a;
            if (root < tmin || root > tmax) return false;
        }
        h.t = root;
        h.p = ray.at(root);
        Vec3 outward = (h.p - c_) / r_;
        h.front = dot(ray.d, outward) < 0;
        h.n = h.front ? outward : -outward;
        h.mat = m_.get();
        return true;
    }
    AABB bounds() const override {
        Vec3 e(r_, r_, r_);
        return {c_ - e, c_ + e};
    }
private:
    Point c_;
    double r_;
    std::shared_ptr<Material> m_;
};

// ---------------------------------------------------------------- BVH
class BVH : public Hittable {
public:
    using Ptr = std::shared_ptr<Hittable>;
    explicit BVH(std::vector<Ptr> objs) : BVH(objs, 0, objs.size()) {}

    bool hit(const Ray& r, double tmin, double tmax, Hit& h) const override {
        if (!box_.hit(r, tmin, tmax)) return false;
        bool hl = left_->hit(r, tmin, tmax, h);
        bool hr = right_->hit(r, tmin, hl ? h.t : tmax, h);
        return hl || hr;
    }
    AABB bounds() const override { return box_; }

private:
    BVH(std::vector<Ptr>& o, size_t s, size_t e) {
        AABB all;
        for (size_t i = s; i < e; ++i) all = AABB::merge(all, o[i]->bounds());
        int axis = all.longest_axis();
        size_t n = e - s;
        auto key = [axis](const Ptr& p) { AABB b = p->bounds(); return b.lo[axis] + b.hi[axis]; };
        if (n == 1) {
            left_ = right_ = o[s];
        } else if (n == 2) {
            left_ = o[s]; right_ = o[s + 1];
            if (key(left_) > key(right_)) std::swap(left_, right_);
        } else {
            std::sort(o.begin() + s, o.begin() + e, [&](const Ptr& a, const Ptr& b) { return key(a) < key(b); });
            size_t mid = s + n / 2;
            left_ = std::shared_ptr<BVH>(new BVH(o, s, mid));
            right_ = std::shared_ptr<BVH>(new BVH(o, mid, e));
        }
        box_ = AABB::merge(left_->bounds(), right_->bounds());
    }
    Ptr left_, right_;
    AABB box_;
};

}  // namespace lumen
