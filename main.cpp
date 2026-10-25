// lumen: a multithreaded CPU path tracer.
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include "lumen/camera.hpp"
#include "lumen/png.hpp"
#include "lumen/scene.hpp"

using namespace lumen;

struct Options {
    int width = 800, height = 450, spp = 100, depth = 40;
    unsigned threads = std::max(1u, std::thread::hardware_concurrency());
    uint64_t seed = 7;
    std::string out = "render.png";
};

static Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        auto is = [&](const char* f) { return !std::strcmp(argv[i], f) && i + 1 < argc; };
        if (is("--width")) o.width = std::atoi(argv[++i]);
        else if (is("--height")) o.height = std::atoi(argv[++i]);
        else if (is("--spp")) o.spp = std::atoi(argv[++i]);
        else if (is("--depth")) o.depth = std::atoi(argv[++i]);
        else if (is("--threads")) o.threads = (unsigned)std::atoi(argv[++i]);
        else if (is("--seed")) o.seed = (uint64_t)std::atoll(argv[++i]);
        else if (is("--out")) o.out = argv[++i];
        else {
            std::cerr << "usage: lumen [--width N] [--height N] [--spp N] [--depth N] [--threads N] [--seed N] [--out file.png]\n";
            std::exit(std::strcmp(argv[i], "--help") ? 1 : 0);
        }
    }
    return o;
}

static std::shared_ptr<Hittable> build_scene(uint64_t seed) {
    Rng rng(seed);
    std::vector<std::shared_ptr<Hittable>> objs;
    auto add = [&](Point c, double r, std::shared_ptr<Material> m) {
        objs.push_back(std::make_shared<Sphere>(c, r, std::move(m)));
    };

    add({0, -1000, 0}, 1000, std::make_shared<Lambertian>(Color(0.45, 0.45, 0.5)));

    for (int a = -11; a < 11; ++a)
        for (int b = -11; b < 11; ++b) {
            Point c(a + 0.9 * rng.uniform(), 0.2, b + 0.9 * rng.uniform());
            if ((c - Point(4, 0.2, 0)).length() < 0.9) continue;
            double pick = rng.uniform();
            if (pick < 0.55) {
                add(c, 0.2, std::make_shared<Lambertian>(rng.random_color() * rng.random_color()));
            } else if (pick < 0.80) {
                Color col(rng.uniform(0.5, 1), rng.uniform(0.5, 1), rng.uniform(0.5, 1));
                add(c, 0.2, std::make_shared<Metal>(col, rng.uniform(0, 0.4)));
            } else if (pick < 0.92) {
                add(c, 0.2, std::make_shared<Dielectric>(1.5));
            } else {
                Color glow = rng.random_color() * 0.7 + Color(0.3, 0.3, 0.3);
                add(c, 0.2, std::make_shared<DiffuseLight>(glow * 4.0));
            }
        }

    add({0, 1, 0}, 1.0, std::make_shared<Dielectric>(1.5));
    add({0, 1, 0}, -0.9, std::make_shared<Dielectric>(1.5));  // hollow glass shell
    add({-4, 1, 0}, 1.0, std::make_shared<Lambertian>(Color(0.4, 0.2, 0.1)));
    add({4, 1, 0}, 1.0, std::make_shared<Metal>(Color(0.7, 0.6, 0.5), 0.0));
    return std::make_shared<BVH>(std::move(objs));
}

static Color trace(Ray r, const Hittable& world, int max_depth, Rng& rng) {
    Color radiance(0, 0, 0), thru(1, 1, 1);
    for (int d = 0; d < max_depth; ++d) {
        Hit h;
        if (!world.hit(r, 1e-3, kInf, h)) {
            double t = 0.5 * (unit(r.d).y + 1.0);
            radiance += thru * ((1.0 - t) * Color(1, 1, 1) + t * Color(0.5, 0.7, 1.0));
            break;
        }
        radiance += thru * h.mat->emitted();
        Color att;
        Ray next;
        if (!h.mat->scatter(r, h, rng, att, next)) break;
        thru = thru * att;
        r = next;
        if (d > 3) {  // Russian roulette
            double p = std::clamp(thru.max_component(), 0.05, 0.95);
            if (rng.uniform() > p) break;
            thru = thru / p;
        }
    }
    return radiance;
}

int main(int argc, char** argv) {
    Options opt = parse(argc, argv);
    auto world = build_scene(opt.seed);
    double aspect = (double)opt.width / opt.height;
    Point from(13, 2, 3), at(0, 0.6, 0);
    Camera cam(from, at, {0, 1, 0}, 22, aspect, 0.1, 10.0);

    std::vector<uint8_t> pixels((size_t)opt.width * opt.height * 3);
    std::atomic<int> next_row{0}, rows_done{0};
    auto t0 = std::chrono::steady_clock::now();

    auto worker = [&](unsigned id) {
        for (;;) {
            int y = next_row.fetch_add(1);
            if (y >= opt.height) return;
            Rng rng(opt.seed * 1000003ULL + (uint64_t)y * 7919ULL + id);
            for (int x = 0; x < opt.width; ++x) {
                Color sum(0, 0, 0);
                for (int s = 0; s < opt.spp; ++s) {
                    double u = (x + rng.uniform()) / (opt.width - 1);
                    double v = (opt.height - 1 - y + rng.uniform()) / (opt.height - 1);
                    Color c = trace(cam.ray(u, v, rng), *world, opt.depth, rng);
                    sum += Color(std::min(c.x, 12.0), std::min(c.y, 12.0), std::min(c.z, 12.0));  // tame fireflies
                }
                sum = sum / opt.spp;
                size_t i = ((size_t)y * opt.width + x) * 3;
                pixels[i + 0] = (uint8_t)(255.999 * std::clamp(std::sqrt(sum.x), 0.0, 1.0));
                pixels[i + 1] = (uint8_t)(255.999 * std::clamp(std::sqrt(sum.y), 0.0, 1.0));
                pixels[i + 2] = (uint8_t)(255.999 * std::clamp(std::sqrt(sum.z), 0.0, 1.0));
            }
            int done = ++rows_done;
            if (done % 20 == 0 || done == opt.height)
                std::cerr << "\rrendering: " << 100 * done / opt.height << "%  " << std::flush;
        }
    };

    std::vector<std::thread> pool;
    for (unsigned i = 0; i < opt.threads; ++i) pool.emplace_back(worker, i);
    for (auto& t : pool) t.join();

    double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    write_png(opt.out, opt.width, opt.height, pixels);
    std::cerr << "\nwrote " << opt.out << " (" << opt.width << "x" << opt.height << ", " << opt.spp
              << " spp, " << opt.threads << " threads) in " << secs << "s\n";
    return 0;
}
