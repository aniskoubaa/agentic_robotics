# `agr_tb_demos`

```bash
ros2 run agr_tb_demos diagnose
ros2 run agr_tb_demos diagnose --ros-args -p active:=true   # actually drives it
ros2 run agr_tb_demos demo_tour
```

## `diagnose` — run this first, always

We did not write this simulator; it is the official TurtleBot stack installed
by apt. So most of what goes wrong is a **mismatch** between what upstream
publishes and what our scripts expect, and the checks are exactly the
mismatches that cost time building this:

1. simulator alive (`/clock`)
2. **`/cmd_vel` is the type we are about to publish** — the registry says
   `TwistStamped`; if upstream ever changes, this catches it instead of you
   debugging a robot that silently ignores you
3. odometry arriving
4. lidar streaming — subscribed BEST_EFFORT, because Create 3 sensors are
5. camera streaming, *or* a clean "this robot has none"
6–8. TurtleBot 4 only: Create 3 status, dock state, and the action set
9. optional: actually drive it out and back

Measured clean runs: **9/9 on `tb4_lite`**, **6/6 on `tb3_burger`**.

Exit code is 0 when everything passes, so it works as a smoke test.

## `demo_tour` — the showcase

Six narrated acts: read the state, undock if needed, drive a measured metre,
turn on the spot, look around with the lidar, then wander with obstacle
avoidance. It prints measurements, not opinions — and on a TurtleBot 4 it
reports whether the Create 3's reflexes overrode the command.

## If a TurtleBot 4 will not move

In this order:

1. **It is docked.** `ros2 run agr_tb_demos diagnose` check 7 says so. Press
   `u` in teleop, or let `Base.ensure_ready()` do it.
2. **A reflex is firing.** The Create 3 runs `REFLEX_DOCK_AVOID`,
   `REFLEX_CLIFF`, `REFLEX_BUMP` and others, and they will drive the robot
   backwards at 0.14 m/s while you ask it to go forwards. Get further from the
   dock. `ros2 param list /motion_control` shows the full set.
3. **It was launched from a polluted shell.** A bare `ros2 launch` from a
   terminal carrying snap paths kills TurtleBot 4's Gazebo GUI on a
   `libpthread` symbol *and* takes the `diffdrive_controller` spawner with it,
   leaving a simulator with no `/odom`. Use `agr-sim turtlebot`, which strips
   them.
