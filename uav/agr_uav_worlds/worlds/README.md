# Worlds

```bash
agr-sim                            # default (PX4's empty world)
agr-sim world:=agr_city            # urban
agr-sim world:=agr_defense         # secured installation
agr-sim world:=baylands            # any PX4 world, by name
```

| World | Origin | Contents |
|---|---|---|
| `default` | PX4 | Ground plane only. The reproducibility baseline — use it whenever a result has to be comparable. |
| `agr_city` | here | 5×5 urban blocks, buildings 8–46 m, a 90 m landmark tower, parks, helipad at origin. Riyadh coordinates. |
| `agr_defense` | here | Fenced installation: perimeter + gate, 4 watchtowers, 2 hangars, apron, 2 helipads, container park, 40 m comms mast, and a marked restricted volume. |

## Why primitives and not Fuel models

Everything here is boxes and cylinders. Fuel models look better, but they are
fetched on first load: that stalls for tens of seconds behind a proxy and fails
outright offline. A world used by thirty machines in a classroom has to load
the same way every time, so nothing here touches the network.

## The restricted zone in `agr_defense`

The translucent red cylinder over the fuel depot has **no collision geometry**.
A UAV flies straight through it, and that is deliberate — the simulator will
not stop you entering restricted airspace, exactly as the real autopilot will
not. That gap is the thing a pre-flight verifier exists to close, and having it
visible on screen makes the point better than a polygon in a JSON file.

## Adding a world

Drop `<name>.sdf` in this folder and rebuild. `agr_uav_bringup` finds it by
name and points `PX4_GZ_WORLDS` here automatically; no launch file changes.
Copy the `<physics>`, `<scene>`, `<light>` and `<spherical_coordinates>` blocks
from an existing world — PX4 needs the spherical coordinates to map local ENU
onto GPS.
