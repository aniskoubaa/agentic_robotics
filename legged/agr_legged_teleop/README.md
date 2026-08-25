# `agr_legged_teleop`

Drive the Go2 by hand. Same `/cmd_vel` interface as the wheeled RaiseBot —
the difficulty of walking lives in `agr_legged_bringup/gait.py`, not here.

| Mode | Command |
|---|---|
| Keyboard | `ros2 run agr_legged_teleop teleop_keyboard` |
| Camera | `ros2 run agr_legged_teleop camera_view` |

## Keys

| Key(s) | Action |
|---|---|
| `w` / `↑` | walk forward |
| `x` / `↓` | walk backward |
| `a` / `←` | turn left on the spot |
| `d` / `→` | turn right on the spot |
| `q` | **strafe left** |
| `e` | **strafe right** |
| `s` / SPACE | STOP |
| Ctrl-C | exit |

`q` and `e` are the two keys the RaiseBot does not have. A differential-drive
base cannot translate sideways at any speed; the Go2 can, because the gait
gives each foot a lateral component and the hip joints deliver it.

## What it can actually do

Measured on flat ground in `agr_inspection`, not estimated:

| command | achieved | note |
|---|---|---|
| forward 0.20 m/s | 0.17 m/s | steady, level |
| forward 0.25 m/s | 0.18 m/s | the clamp — above this it falls over |
| backward 0.15 m/s | 0.06 m/s | rearward stance is weaker |
| strafe 0.12 m/s | 0.11 m/s | |
| yaw 0.80 rad/s | 0.72 rad/s | |

The gait is open loop: no IMU feedback, no balance controller. It walks on
flat ground and will not climb the staircase in `agr_inspection` — that needs
state estimation and a real locomotion stack.

## Nothing moves?

```bash
ros2 run agr_legged_demos diagnose
```
