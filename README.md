# lumen

A multithreaded CPU path tracer written from scratch in modern C++17. No external dependencies, not even an image library.

![render](images/render.png)

*1200x675, 300 samples per pixel.*

## Features
- Monte Carlo path tracing with Russian roulette termination
- Materials: Lambertian (diffuse), metal with roughness, dielectric glass (Snell's law + Schlick approximation), emissive lights
- Bounding volume hierarchy (BVH) for fast ray-scene intersection
- Thin-lens camera with depth of field
- Work-stealing thread pool: threads pull image rows from an atomic counter
- Built-in PNG encoder (CRC32, Adler-32, zlib stored blocks)
- Firefly clamping and gamma correction

## Build and run
```bash
cmake -B build
cmake --build build
./build/lumen                                   # 800x450, 100 spp -> render.png
./build/lumen --width 1920 --height 1080 --spp 500 --out hd.png
```

### Options
| Flag | Default | Meaning |
|---|---|---|
| `--width`, `--height` | 800, 450 | image size |
| `--spp` | 100 | samples per pixel (more = less noise) |
| `--depth` | 40 | max ray bounces |
| `--threads` | all cores | worker threads |
| `--seed` | 7 | scene and sampling seed |
| `--out` | render.png | output file |

## Layout
```
include/lumen/vec3.hpp    vector math + fast xorshift RNG
include/lumen/scene.hpp   rays, materials, spheres, AABB, BVH
include/lumen/camera.hpp  thin-lens camera
include/lumen/png.hpp     PNG writer
src/main.cpp              scene, integrator, threaded renderer, CLI
```

## How it works
For every pixel, `lumen` fires many randomly jittered rays through the camera lens. Each ray bounces
through the scene: the material at each hit decides the next direction and how much light it absorbs
(`attenuation`). Light picked up from the sky or emissive spheres is accumulated and averaged over all
samples. The BVH turns the per-ray cost from linear in the object count into roughly logarithmic.

## Ideas for extending
- Triangle meshes + OBJ loader
- Textures (checker, image, Perlin noise)
- Importance sampling of lights (next-event estimation)
- Quads, boxes, volumes (fog), motion blur
- SIMD ray packets or a flattened, cache-friendly BVH
- Denoising pass

## License
MIT
