# `agr_uav_teleop`

Fly the drone by hand, in PX4 offboard mode.

```bash
agr-sim                                   # or airframe:=x500_mono_cam
ros2 run agr_uav_teleop teleop_keyboard
```

| Key | Action |
|---|---|
| `t` | **take off** (arm + climb to 3 m) |
| `l` | **land** (and disarm) |
| `w` / `x` | forward / backward |
| `q` / `e` | strafe left / right |
| `r` / `f` | up / down |
| `a` / `d` | yaw left / right |
| SPACE | hover — cancel all motion |
| Ctrl-C | land, then exit |

Motion is in the **body frame**: `w` goes where the nose points, not north.

Every direction above was verified in flight with the nose pointing north,
because two of the sign conventions are counter-intuitive and reasoning about
them gets it wrong:

* **Yaw is clockwise-positive.** NED measures heading from North toward East,
  so a positive `yawspeed` turns the aircraft *right*.
* **Body +y is right.** With the nose north, east is on your right hand.

Get either backwards and you have a drone that flies the mirror image of what
you pressed.

## Why this is not just the ground teleop with different keys

You cannot stop publishing. PX4 leaves offboard mode and enters failsafe if
setpoints stop arriving for about half a second, so the loop publishes on
*every* iteration whether or not a key was pressed — releasing a key means
"velocity zero", not "no command". The wheeled and legged teleops can simply
go quiet; this one cannot.

## Camera

```bash
agr-sim airframe:=x500_mono_cam world:=agr_city
ros2 run agr_uav_teleop camera_view
```

Only `x500_mono_cam` has a camera. `ros2 run agr_uav_examples 06_list_airframes`
shows which airframes carry what.

## Nothing works?

```bash
ros2 run agr_uav_demos diagnose
```
