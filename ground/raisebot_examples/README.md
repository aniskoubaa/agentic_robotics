# `raisebot_examples`

Six short scripts, one idea each. The Day 1–3 labs in `raisebot_labs` are
longer, interactive and build toward an agent; these are the primer.

```bash
agr-sim ground tools:=true      # tools:=true is needed for 05 and 06
```

| # | Command | The one idea |
|---|---|---|
| 01 | `ros2 run raisebot_examples 01_drive` | `/cmd_vel` is a *stream*, not a command queue |
| 02 | `ros2 run raisebot_examples 02_read_lidar` | infinities are normal — filter before you `min()` |
| 03 | `ros2 run raisebot_examples 03_get_image` | this robot has *three* cameras, and they answer different questions |
| 04 | `ros2 run raisebot_examples 04_aim_the_ptz` | not every command is a service — and verify, don't assume |
| 05 | `ros2 run raisebot_examples 05_call_a_service` | the robot as a set of callable functions |
| 06 | `ros2 run raisebot_examples 06_navigate` | actuation vs navigation: who owns the loop |

## Options worth trying

```bash
ros2 run raisebot_examples 01_drive       --ros-args -p vx:=0.0 -p wz:=0.8
ros2 run raisebot_examples 03_get_image   --ros-args -p camera:=ptz     # or depth
ros2 run raisebot_examples 04_aim_the_ptz --ros-args -p pan:=1.2 -p tilt:=-0.3
ros2 run raisebot_examples 05_call_a_service --ros-args -p service:=/close_gripper
ros2 run raisebot_examples 06_navigate    --ros-args -p waypoint:=olive_grove
```

Waypoints: `home`, `tomato_row_1`, `tomato_row_2`, `olive_grove`.

## Two things that will confuse you once

**`/odom` is relative to the SPAWN point, not the world.** The robot spawns at
world `(-6.0, 1.5)` and odometry starts at `(0, 0)`. Waypoints are world
coordinates, so the number `06_navigate` prints will not match the target it
just reached. That is correct, and it is why `navigation_server` re-anchors
against ground truth rather than trusting odometry.

**Drive the robot into the crop rows and later examples will fail.** It gets
physically wedged and `nav_to_*` then times out reporting "travelled 0.00 m".
Reset it:

```bash
agr-stop && agr-sim ground tools:=true
```

## Compare across platforms

`05_call_a_service` and the legged/UAV squares are the two comparisons worth
making deliberately:

* **Service vs topic** — "did the gripper close?" has an answer; "drive at
  0.3 m/s" does not. That is the whole reason `raisebot_tools` exists.
* **Open vs closed loop** — the UAV's `04_fly_a_square` closes a 20 m square
  to 0.10 m; the Go2's `06_walk_a_square` finishes ~10° off. Same shape, and
  the difference is entirely whether anything measures and corrects.
