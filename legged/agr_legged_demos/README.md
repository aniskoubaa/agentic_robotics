# `agr_legged_demos`

Two scripts: one for when something is broken, one for when someone is
watching.

## `diagnose` — run this first

```bash
ros2 run agr_legged_demos diagnose
```

Nine checks in the order things actually break, each printing what it saw and
what to do about it. Exit code 0 when all pass, so it also works as a smoke
test in a script.

```
[  OK  ] 1. Gazebo is running            /clock ticking, 711 msgs in 2 s
[  OK  ] 2. Robot description loaded     robot_state_publisher is up
[  OK  ] 3. Position controller active   9 controller topics
[  OK  ] 4. Joint states arriving        all 12 joints, 500 Hz
[  OK  ] 5. Robot is upright             roll -0.0°, pitch -0.0°, 400 Hz
[  OK  ] 6. Camera streaming             640x480 rgb8, 15.0 Hz
[  OK  ] 7. Gait controller listening    1 subscriber(s) on /cmd_vel
[  OK  ] 8. Joint commands flowing       100 Hz, 12 values per message
[  OK  ] 9. Simulation clock             sim time 723.764s
```

A legged sim fails in layers and every layer looks the same from the top —
"the robot does not move". The useful thing a diagnostic does is tell you
*which* layer.

## `demo_walkabout` — the five-minute demo

```bash
ros2 run agr_legged_demos demo_walkabout
```

Nine narrated moves in 39 seconds: stand, walk, strafe both ways, turn on the
spot both ways, arc, reverse, stop. The strafe follows the walk deliberately —
seeing the same body move sideways with no turn is the moment the "why legs?"
question answers itself.

Add a camera view in another terminal:

```bash
ros2 run agr_legged_teleop camera_view
```
