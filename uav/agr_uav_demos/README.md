# `agr_uav_demos`

## `diagnose` — run this when it will not fly

```bash
ros2 run agr_uav_demos diagnose
ros2 run agr_uav_demos diagnose --ros-args -p active:=true   # actually arm it
```

"It won't arm" has about eight distinct causes and PX4 announces none of them
on a ROS topic by default. The topic that *would* tell you — `vehicle_status`
— publishes only on change, so on an idle vehicle it says nothing at all and a
naive check reports a healthy system as dead. This reads `failsafe_flags`
instead, which streams continuously and names the specific blocker.

```
[  OK  ] 1. PX4 bridge is up              31 /fmu/out topics
[  OK  ] 2. Position telemetry            46 Hz, n=0.1 e=-1.7 alt=-0.0 m
[  OK  ] 3. Control mode telemetry        1.7 Hz — disarmed, offboard ON
[ INFO ] 4. Vehicle status                exists but sent nothing — EXPECTED
[ WARN ] 5. Nothing is blocking flight    gcs_connection_lost (harmless here)
[  OK  ] 6. Battery                       100%, 16.2 V
[  OK  ] 7. Land detector                 LANDED
[  OK  ] 8. Camera                        1280x960, 30.0 Hz
[  OK  ] 9. Command path (arm)            PX4 accepted the request
```

Exit code 0 when all pass, so it also works as a smoke test in a script.

`active:=true` streams setpoints, requests offboard, arms, confirms, disarms —
proving the whole command path, not just that telemetry arrives. It has to
stream setpoints first: arming with a bare command works from `AUTO.LOITER`
but is refused in offboard mode, and the vehicle is often already in offboard
after a previous flight.

## `demo_flight` — the five-minute demo

```bash
agr-sim airframe:=x500_mono_cam world:=agr_city
ros2 run agr_uav_demos demo_flight
```

Arm, climb to 4 m, two 6 m legs, a 5 m orbit with the nose held inward, return
and yaw on the spot, land.

The orbit is the part worth watching. It is not a PX4 mode — it is the script
computing a circle of setpoints and streaming them at 20 Hz. Once you can
stream position setpoints, arbitrary trajectories are just arithmetic, and
that is the entire reason offboard mode exists.
