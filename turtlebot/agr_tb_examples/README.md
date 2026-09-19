# `agr_tb_examples`

Six short scripts, one idea each. They run **unchanged** on every TurtleBot in
the registry — the per-robot differences live in
`agr_tb_tools/config/robots.yaml`, not in these files.

```bash
agr-sim turtlebot                      # TurtleBot3 Waffle (default)
agr-sim turtlebot model:=tb4_lite      # official TurtleBot 4
ros2 run agr_tb_tools list_robots      # what else is available
```

| # | Command | The one idea |
|---|---|---|
| 01 | `ros2 run agr_tb_examples 01_read_odometry` | a base is three numbers, and they are a guess |
| 02 | `ros2 run agr_tb_examples 02_drive` | TwistStamped, and commands that do not latch |
| 03 | `ros2 run agr_tb_examples 03_read_lidar` | one array plus the geometry to read it |
| 04 | `ros2 run agr_tb_examples 04_get_image` | a ROS Image is a byte array, not a picture |
| 05 | `ros2 run agr_tb_examples 05_drive_a_square` | open loop keeps every error it ever makes |
| 06 | `ros2 run agr_tb_examples 06_avoid_obstacle` | sense, decide, act — with no map and no memory |

## The spine of the set is 05 versus 06

`05` drives a square open loop. Nothing ever checks the result against the
world, so the errors accumulate and the square does not close. Measured on a
TurtleBot 4 over a 0.8 m square: **22 mm of gap and 2.6° of heading error over
3.2 m driven** — and that gap is measured with the same odometry that did the
driving, so the true error is larger.

`06` never accumulates anything, because it only ever reacts to *now*. It also
never knows where it is, and will happily drive in circles. Watching it get
stuck is the most convincing argument for SLAM there is.

Compare `05` with `agr_uav_examples 04_fly_a_square`, which closes a **20 m**
square to **0.10 m**. The drone wins because PX4 runs a state estimator and a
position controller, so "go to (5, 0)" is a closed loop that keeps correcting.
Nothing in `05` corrects anything.

## Two things that will waste your afternoon if you skip them

**`/cmd_vel` is `geometry_msgs/TwistStamped`, not `Twist`.** This changed in
Jazzy and it is the single most common reason a TurtleBot script does nothing:
publish a `Twist` and there is no error, no warning, and no movement. Check it
before debugging anything else:

```bash
ros2 topic info /cmd_vel
```

**A TurtleBot 4 spawns docked and will not drive.** A docked Create 3 ignores
velocity commands entirely. Every example calls `Base.ensure_ready()`, which
undocks first and says so — it takes about 30 seconds.

## `04` on a Burger is supposed to refuse

The TurtleBot3 Burger has no camera. Asking a robot for a sensor it does not
have should produce a sentence, not a script that hangs forever, so `04`
checks the registry and tells you which robots do have one.
