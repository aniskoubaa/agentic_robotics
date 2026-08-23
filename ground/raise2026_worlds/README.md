# `raise2026_worlds`

Gazebo Harmonic worlds for the RAISE 2026 summer school.

## Contents (to be filled)

- `worlds/greenhouse_2026.sdf` — the main world: 3 aisles × 5 tomato plants, Husky-traversable
- `worlds/greenhouse_2026_lite.sdf` — fewer plants, lower texture detail, for `CPU_ONLY` machines
- `meshes/tomato_plant/` — plant mesh with ripe/unripe variants
- `meshes/greenhouse_structure/` — walls, benches, lighting

## Renderer notes

- Both worlds target Ogre2 by default. The installer flips to Ogre1 via `GZ_RENDER_ENGINE=ogre` on `CPU_ONLY` tier.
- Avoid heavy GLSL shaders in custom materials — keep students with iGPU-only laptops in mind.
