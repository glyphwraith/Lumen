#pragma once
#include "scene.hpp"

namespace lumen {

// Thin-lens camera with depth of field.
class Camera {
public:
    Camera(Point from, Point at, Vec3 up, double vfov_deg, double aspect, double aperture, double focus) {
        double theta = vfov_deg * 3.14159265358979323846 / 180.0;
        double vh = 2.0 * std::tan(theta / 2), vw = aspect * vh;
        w_ = unit(from - at);
        u_ = unit(cross(up, w_));
        v_ = cross(w_, u_);
        origin_ = from;
        horiz_ = focus * vw * u_;
        vert_ = focus * vh * v_;
        corner_ = origin_ - horiz_ / 2 - vert_ / 2 - focus * w_;
        lens_r_ = aperture / 2;
    }
    Ray ray(double s, double t, Rng& rng) const {
        Vec3 rd = lens_r_ * rng.in_unit_disk();
        Vec3 off = u_ * rd.x + v_ * rd.y;
        return {origin_ + off, corner_ + s * horiz_ + t * vert_ - origin_ - off};
    }
private:
    Point origin_, corner_;
    Vec3 horiz_, vert_, u_, v_, w_;
    double lens_r_;
};

}  // namespace lumen
