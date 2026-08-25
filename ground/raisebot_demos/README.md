# `raisebot_demos`

Two scripts: one for when something is broken, one for when someone is
watching.

## `diagnose` — run this first

```bash
ros2 run raisebot_demos diagnose
```

Ten checks in the order things actually break. Exit code 0 when all pass, so
it also works as a smoke test in a script.

```
[  OK  ] 1. Gazebo is running          /clock ticking, 975 msgs in 2 s
[  OK  ] 2. ROS ↔ Gazebo bridge        all four core topics present
[  OK  ] 3. Odometry                   40 Hz, x=+13.83 y=-0.92
[  OK  ] 4. LiDAR                      360 beams, 104 return a range
[  OK  ] 5. Joint states               18 joints — arm 6, gripper 6, PTZ 2
[  OK  ] 6. Cameras                    wrist RGB; wrist depth; PTZ
[  OK  ] 7. Drive command path         1 subscriber(s) on /cmd_vel
[  OK  ] 8. Tool servers               all 6 responding
[  OK  ] 9. Robot is upright           roll +0.0°, pitch +0.0°
[  OK  ] 10. Simulation clock          sim time 333.572s
```

Check 8 is the one that saves the most time. A service call with no server
blocks **forever**, so a lab that "hangs" is almost always six missing
servers rather than a bug in your code. Start them with:

```bash
agr-sim ground tools:=true
# or, into a running sim:
ros2 launch raisebot_bringup tools.launch.py
```

Note `grasp_server` is checked by node name, not by service — it exposes none.
It watches the gripper joints in `/joint_states` and attaches a tomato when
they close, so a human teleoperator, a script and a trained policy all grasp
identically.

## `demo_greenhouse` — the five-minute demo

```bash
agr-sim ground tools:=true
ros2 run raisebot_demos demo_greenhouse
```

A narrated inspection run: aim the mast camera, drive to a crop row *by name*,
pose the arm, work the gripper, stow, visit the second row, come home. About
50 seconds.

Every step is a **service call**, not a control loop — so the terminal output
is exactly the trace an LLM agent would produce. That is the point of the
demo, and the bridge into the Day 1–3 labs.

Add a camera view in another terminal:

```bash
ros2 run raisebot_teleop camera_view
```
