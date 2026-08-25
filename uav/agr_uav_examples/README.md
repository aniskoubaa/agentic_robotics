# `agr_uav_examples`

Six short scripts, one idea each.

```bash
agr-sim                     # x500, or airframe:=x500_mono_cam for 05
```

| # | Command | The one idea |
|---|---|---|
| 01 | `ros2 run agr_uav_examples 01_read_telemetry` | PX4 is not a ROS node — everything crossed a bridge |
| 02 | `ros2 run agr_uav_examples 02_arm_and_disarm` | arming is a *request*; PX4 decides |
| 03 | `ros2 run agr_uav_examples 03_takeoff_and_land` | offboard mode has an order, and it fails silently |
| 04 | `ros2 run agr_uav_examples 04_fly_a_square` | a closed loop is what buys accuracy |
| 05 | `ros2 run agr_uav_examples 05_get_image` | not every airframe has a camera |
| 06 | `ros2 run agr_uav_examples 06_list_airframes` | feasibility is a property of the *platform* |

## The four rules that are not discoverable

Every one of these fails silently, which is why they are worth stating before
you read any code. All four live in `agr_uav_tools/offboard.py`.

1. **Setpoints before mode.** PX4 refuses offboard unless setpoints are
   *already* streaming. Ask first and the request is rejected with no error on
   any ROS topic.
2. **Never stop streaming.** A gap of ~0.5 s drops PX4 into failsafe. This is
   why "hover for five seconds" is a publishing loop, never a `sleep()`.
3. **NED, not ENU.** Up is *negative* z. A takeoff to 2.5 m is `z = -2.5`.
4. **The QoS must match.** PX4 publishes BEST_EFFORT; a default RELIABLE
   subscription never matches and never receives — indistinguishable from a
   dead bridge.

And one that cost real time to find: **`vehicle_status` publishes only on
change**, and the XRCE-DDS agent does not replay the last sample to a late
subscriber. A script that starts after the vehicle settles waits forever for a
message a healthy PX4 has no reason to send. Arming state here comes from
`vehicle_control_mode`, which streams at 2 Hz regardless.

## 04 is the one to compare

`04_fly_a_square` closes a 20 m square to about **0.10 m**. The Go2's
`06_walk_a_square` finishes ~10° off heading and metres away. Flying is not
easier than walking — PX4 runs a state estimator and a position controller, so
"go to (5, 0)" is a closed loop that keeps correcting. The legged gait is open
loop and nothing ever checks.
