# The TurtleBot platform: using the official simulators, and what they cost

Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
Date: 2026-08-25

A fifth platform, `turtlebot/`, and the first one whose simulator we did not
write. TurtleBot 3 and TurtleBot 4 both ship excellent official simulators for
Jazzy; this platform launches them **unchanged** and adds only the three
teaching layers on top.

Five packages, 7 executables:

| Package | Holds |
|---|---|
| `agr_tb_tools` | `robots.yaml` registry, `Base` motion helper, `list_robots` |
| `agr_tb_bringup` | `sim.launch.py` — includes the upstream launch file |
| `agr_tb_teleop` | `teleop_keyboard`, `camera_view` |
| `agr_tb_examples` | `01_read_odometry` .. `06_avoid_obstacle` |
| `agr_tb_demos` | `diagnose`, `demo_tour` |

Four robots: `tb3_burger`, `tb3_waffle`, `tb4_lite`, `tb4_standard`.

`setup.sh` gained an apt dependency step (`--deps-only`, `--no-deps`) covering
all five platforms — 17 packages, of which the TurtleBot 4 entry alone pulls
about 70 because a TB4 is a Create 3 with a mast on it.

---

## Measured results

| Thing | TB3 Burger | TB4 Lite |
|---|---|---|
| `diagnose` | **6/6** | **9/9** |
| drive 0.2 m/s × 4 s | — | 0.795 m, peak 0.240 m/s |
| turn 90° (odometry-closed) | — | error −2.8° |
| 0.8 m square, open loop | — | **22 mm gap, −2.6° heading over 3.2 m** |
| lidar | 360 beams, 5 Hz | 640 beams, ~18 Hz |
| camera | none (correct) | 320×240 OAK-D, 30 Hz |
| undock | n/a | ~30 s |

---

## Seven things the testing found

### 1. `/cmd_vel` is `TwistStamped` on BOTH robots under Jazzy

Not `Twist`. Publish a `Twist` and the robot does not move, nothing errors and
nothing warns. This is the single most important fact for anyone writing
TurtleBot code on Jazzy, and it invalidates most tutorials still in
circulation. TB4 additionally relays `/cmd_vel_unstamped` (`Twist`); TB3 does
not.

### 2. The official TB4 stack does not run in a normal shell

Launched plainly from a snap-polluted terminal it fails **twice**:

```
gz sim gui: symbol lookup error: /snap/core20/current/lib/.../libpthread.so.0
[ERROR] [spawner-46]: process has died ... diffdrive_controller
```

The GUI dies on the snap `libpthread` and takes the controller spawner with
it, leaving a simulator with no `/odom` and a robot that silently will not
move. With `agr-sim`'s existing environment hygiene applied: `/clock` 121 Hz,
`/scan` 17.7 Hz, `/odom` 18.6 Hz, controller activated.

This is the whole justification for the wrapper. We add no robot and no world;
we make the upstream one start.

### 3. A TurtleBot 4 spawns DOCKED and refuses to drive

`is_docked: True` on a fresh spawn. A docked Create 3 ignores velocity
commands. `/undock` works and takes about 30 s. `Base.ensure_ready()` does it
and says so.

This cost the most time to find, because the symptom is not "it refuses" — it
is that the robot creeps a few centimetres and then reverses.

### 4. The Create 3 reflex layer overrides your commands

Commanding +0.25 m/s near the dock: the wheels reach +0.2500 for about two
seconds, then go to **−0.14 m/s** and stay there while the command is still
being published. `ros2 param list /motion_control` shows why —
`REFLEX_DOCK_AVOID`, `REFLEX_CLIFF`, `REFLEX_BUMP`, `REFLEX_STUCK`,
`REFLEX_PROXIMITY_SLOWDOWN`.

Once properly undocked and clear of the dock, raw `/cmd_vel` works normally.
`Base.drive()` returns a dict with `reflex_reversed` so a script can tell the
difference between "I drove" and "I was overruled".

### 5. Create 3 sensor topics are BEST_EFFORT

A default (RELIABLE) subscription to `/dock_status` receives **nothing**, with
one easily missed warning at startup:

```
New publisher discovered on topic '/dock_status', offering incompatible QoS.
```

Same family as the PX4 QoS trap already documented in the UAV platform. Hence
`SENSOR_QOS` in `base.py`.

### 6. `TURTLEBOT3_MODEL` has no default

`spawn_turtlebot3.launch.py` does `os.environ['TURTLEBOT3_MODEL']` — unset, it
is a `KeyError` traceback rather than a message. `sim.launch.py` sets it from
the registry, so `model:=tb3_burger` is the only thing anyone has to type.

### 7. Leftover processes silently corrupt the graph

Two TurtleBot stacks running at once produce duplicate `diffdrive_controller`
and bridge nodes fighting over `/cmd_vel`, and the symptom is a robot that
moves at a fifth of the commanded speed. Diagnosing this wasted a cycle;
`agr-stop` gained TurtleBot patterns so it does not happen again. When in
doubt, `ros2 node list | sort | uniq -d`.

---

## Design note: why one platform and not two

The original proposal was a single `agr_tb_*` family with a model registry.
Testing nearly overturned that — TB3 and TB4 behave so differently that they
looked like two platforms — and then confirmed it, because the difference is
concentrated in exactly one place: the Create 3 layer.

So the registry stayed, the examples run unchanged on all four robots, and
everything TB4-specific lives behind `Base.ensure_ready()`, `Base.is_docked`
and the `reflex_reversed` flag. A student meets the same six lessons on either
robot; on a TB4 they additionally meet a machine that argues back.

That contrast is worth more than the tidiness. TB3 is where navigation is
taught because it is predictable and runs on a laptop. TB4 is where you learn
that a real robot has its own opinions — it will not move because it is
charging, its sensors need the right QoS, and its safety layer outranks you.

---

## Known limits

- **No Nav2 or SLAM integration yet.** Both robots ship it upstream
  (`turtlebot3_navigation2`, `turtlebot4_navigation`, `turtlebot3_cartographer`
  — all installed). Examples 05 and 06 are deliberately the *argument* for
  Nav2 rather than a use of it; wiring it in is the obvious next step.
- **`demo_tour`'s wander loop is inlined**, duplicating
  `06_avoid_obstacle`, so the demo stands alone. If a third caller appears it
  should move into `agr_tb_tools`.
- **TB4 `standard` is heavy.** RTF measured 0.82 on an Intel iGPU with the
  OAK-D depth and pointcloud streams. `tb4_lite` is the better default for a
  class.
- **Rates reported by `diagnose` are counted over wall time**, so on a
  fast-stepping world they read higher than the sensor's nominal rate. The
  number is honest about what arrived; it is not the sensor's spec.
