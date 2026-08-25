# `agr_legged_examples`

Six short scripts, one idea each. Read them in order; each is meant to be read
in full in under a minute.

Start the robot first:

```bash
agr-sim legged
```

| # | Command | The one idea |
|---|---|---|
| 01 | `ros2 run agr_legged_examples 01_read_joints` | a quadruped is twelve numbers |
| 02 | `ros2 run agr_legged_examples 02_read_imu` | which way is up — the sensor legs need and wheels do not |
| 03 | `ros2 run agr_legged_examples 03_get_image` | a ROS Image is a byte array, not a picture file |
| 04 | `ros2 run agr_legged_examples 04_change_pose` | driving the joints directly, and why only one thing may |
| 05 | `ros2 run agr_legged_examples 05_walk` | walking looks exactly like driving — that is the point |
| 06 | `ros2 run agr_legged_examples 06_walk_a_square` | dead reckoning does not close, and here is the number |

## 04 needs the gait controller stopped

Both publish to the same position controller, and two publishers on one
controller means the legs get whichever message landed last — the robot
thrashes onto its back within seconds. `04_change_pose` detects this and
refuses. Relaunch without the gait:

```bash
agr-stop && agr-sim legged gait:=false
ros2 run agr_legged_examples 04_change_pose --ros-args -p pose:=crouch
```

Poses: `stand` (trunk at 0.324 m), `crouch` (0.164 m), `tuck` (0.110 m).

## 06 is supposed to fail

It walks four sides and four 90° turns on a timer, then reads the IMU and
prints its own heading error. A clean run lands about 10° off. That error is
the entire argument for closing the loop, which is what the labs do next.
