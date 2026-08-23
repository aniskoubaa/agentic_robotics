# Ground platform

The AgriBot stack (originally the AgriBot summer-school stack), brought across **unmodified**: same package
names, same launch files, same labs. Nothing was renamed.

## Why the names are still `agribot_*`

Renaming these to `agr_ground_*` would touch every import, entry point, launch
reference and alias across ~140 files — including the Day-2 VLA labs, which
need LeRobot, a GPU and trained checkpoints to test properly. A rename that
cannot be verified is a rename that quietly breaks a course.

So the isolation here is **structural, not nominal**: `ground/` and `uav/` are
separate subtrees, share only `common/agr_core`, and never load each other's
packages. If the names should change later, that is one deliberate commit with
the labs actually exercised — not a side effect of moving folders.

## Packages

| Package | Purpose |
|---|---|
| `agribot_bringup` | `sim.launch.py`, `world_only.launch.py`, ros_gz bridge config |
| `agribot_worlds` | `greenhouse_2026.sdf`, `greenhouse_2026_lite.sdf` + plant/tree/chicken meshes |
| `agribot_description` | Husky + UR arm + Robotiq gripper URDF/xacro |
| `agribot_tools` | Tool servers: gripper, navigation, move_to_pose, detector (YOLO), inspector (VLM), grasp |
| `agribot_teleop` | Keyboard, phone (Flask) and joystick teleop; camera view |
| `agribot_labs` | Day 1–3 exercises: ROS 2 tools as functions, agentic inspector, teleop + data collection, VLA executor, full stack, hackathon |
| `agribot_demos` | One-command demo scripts |

## Running

```bash
agr-sim ground                                   # greenhouse + Husky
agr-sim ground world:=greenhouse_2026_lite.sdf   # lighter world, CPU-only machines
agr-sim ground y:=-1.0                           # spawn in another aisle
agr-sim ground --world-only                      # world with no robot
agr-stop
```

## Not carried over

**`yolov8n.pt`** (6.2 MB). Model weights do not belong in git — `ultralytics`
downloads it on first use. The detector lab fetches it automatically the first
time it runs, which needs network access once.

**The Day-2 reference checkpoint** (fine-tuned SmolVLA, 687 MB) was never in
this repo; `get_brain_d4` in the original course alias block downloads it from a GitHub
release. If you want the VLA labs here, bring that helper across too.

## What has been verified

`agr-sim ground` brings up the greenhouse, spawns the Husky, and starts the
ros_gz bridge — `/cmd_vel`, `/joint_states` and the gripper joint commands are
all live.

The Day-2/Day-3 ML labs (LeRobot, VLA, YOLO, VLM) have **not** been exercised
here. They need a GPU, python venvs and API keys, and they worked in the original
workspace; nothing in this move should have changed them, but "should" is not
"tested".
