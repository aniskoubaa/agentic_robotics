# `agr_arm_demos`

Two programs: one that tells you whether the robot works, and one that shows
what it can do.

```bash
ros2 run agr_arm_demos diagnose
ros2 run agr_arm_demos demo_pick_and_place
```

## `diagnose` — run this first, always

Eleven checks in the order things actually break, each printing what it saw
and what to do about it. A manipulation sim fails in layers and every layer
looks identical from the top ("the arm does not move"): Gazebo up but no
model, model spawned but no joint controllers, controllers fine but the tool
servers never started so every service call blocks forever. The point of the
tool is to tell you WHICH layer.

Exit code is 0 when everything passes, so it works as a smoke test in a
script.

```bash
ros2 run agr_arm_demos diagnose --ros-args -p active:=true
```

adds a twelfth check that actually moves the shoulder 0.2 rad and puts it
back. Off by default, because a diagnostic that moves a robot is a
diagnostic people stop running near anything fragile.

**Check 11 is the one nothing else would catch.** It compares this stack's
own forward kinematics against the TF tree that `robot_state_publisher`
builds from the same URDF. If those two ever disagree, every Cartesian number
the platform prints is quietly wrong and nothing else here would notice.
Measured agreement on this machine: 0.04 mm.

## `demo_pick_and_place` — the showcase

Resets the bench, goes to the overview pose, then picks all three blocks into
the drop tray, narrating as it goes. It reports whether the blocks are
actually in the tray according to Gazebo, not according to whether the code
thought it went well.

```bash
ros2 run agr_arm_demos demo_pick_and_place
ros2 run agr_arm_demos demo_pick_and_place --ros-args -p blocks:=1
```

Needs the tool servers: `agr-sim arm tools:=true`. Without `grasp_server` the
gripper closes on nothing; the demo warns and carries on so you can at least
watch the motion.

This is also the honest end-to-end test of the platform. If it finishes 3/3,
the kinematics, the controllers, the gripper and the grasp constraint are all
working together — which no single check proves on its own.
