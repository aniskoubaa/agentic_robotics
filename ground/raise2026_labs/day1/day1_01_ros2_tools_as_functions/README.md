# Lab 1.1 — `day1_01_ros2_tools_as_functions`

**Slot:** Day 1, 14:00 – 15:45 (105 min) · **Track:** both

## Goal

Wrap ROS 2 capabilities as Python functions with LLM tool schemas, so the
agent in D1L2 can call them as tools.

## How the lab is structured

You'll build up to the deliverable through a **numbered progression** —
each `starter/XX_…py` script teaches one ROS 2 concept and is testable by
itself. Run them **in order**.

| #  | Script                            | Concept                                 | Status     |
|----|-----------------------------------|-----------------------------------------|------------|
| 01 | `starter/01_drive_forward.py`     | Publisher → `/cmd_vel`                  | ✅ shipped |
| 02 | `starter/02_read_lidar.py`        | Subscriber ← `/scan`                    | ✅ shipped |
| 03 | `starter/03_read_camera.py`       | `cv_bridge`, `/ptz_camera/image_raw`    | ✅ shipped |
| 04 | `starter/04_aim_ptz.py`           | Pub to `/ptz/pan/cmd` + `/ptz/tilt/cmd` | ✅ shipped |
| 05 | `starter/05_call_gripper.py`      | First service call: `*_gripper`         | ✅ shipped (mock server) |
| 06 | `starter/06_call_robotic_arm.py`  | Service: arm named-pose goal            | ✅ shipped (mock server) |
| 07 | `starter/07_call_navigation.py`   | Service: autonomous nav to a waypoint   | ✅ shipped (mock server) |
| 08 | `starter/08_call_detector.py`     | Service: YOLO object detection          | ✅ shipped |
| 09 | `starter/09_call_inspector.py`    | Service: VLM-backed plant-health report | ✅ shipped |
| 10 | `starter/10_orchestrate.py`       | Chain everything via an LLM tool-use loop | ✅ shipped |

Script `10_orchestrate.py` is the climax: you type a goal and an LLM chains
the services from 05–09 as tools. That loop is also the springboard for
**Lab 1.2** (`day1_02_agentic_inspector`), which turns it into an autonomous,
report-producing agent.

> The tool-wrapping deliverable below (`starter/tools.py`, `starter/schemas.json`)
> is **not yet shipped** — `10_orchestrate.py` currently defines its tool
> schemas inline. Factoring those out into reusable `tools.py` + `schemas.json`
> is the student exercise.

## Functions to implement (in `starter/tools.py`)

| Function           | ROS 2 action                                            | Returns                            |
| ------------------ | ------------------------------------------------------- | ---------------------------------- |
| `nav_to_row(row)`  | service call to `nav_to_row`                            | `{"success": bool, "pose": ...}`   |
| `aim_camera(p,t)`  | publish to `/ptz/pan/cmd` + `/ptz/tilt/cmd`             | `{"pan": float, "tilt": float}`    |
| `capture_image()`  | one frame from `/ptz_camera/image_raw`                  | `{"image_b64": str, "ts": ...}`    |
| `get_robot_pose()` | read `/odom`                                            | `{"x": ..., "y": ..., "yaw": ...}` |
| `inspect_plant()`  | service call to `inspect_plant`                         | `{"ripeness": str, "confidence": float}` |

Plus a JSON tool-schema for each, compatible with both OpenAI and
Anthropic tool-calling formats.

## Prerequisites

- `raise-sim` is running (Gazebo open, you see the agroforestry plot)
- A terminal with the workspace sourced (any fresh shell does this via `~/.bashrc`)

## How to run the starters

Two equivalent ways — pick whichever you prefer.

**A. `ros2 run` with tab-completion** *(recommended once the workspace is built)*

```bash
ros2 run raise2026_labs 01_drive_forward.py     # press TAB after raise2026_labs
ros2 run raise2026_labs 02_read_lidar.py        # Ctrl-C to quit
ros2 run raise2026_labs 03_read_camera.py       # writes /tmp/raise_camera_frame.png
ros2 run raise2026_labs 04_aim_ptz.py           # 14-s PTZ look-around routine
# Scripts 05–09 each need a matching server in another terminal.
# The 05_d1 … 10_d1 aliases bundle the server + client for you, e.g.:
05_d1   # starts gripper_server, runs 05_call_gripper.py, cleans up
10_d1   # starts all 5 servers, runs 10_orchestrate.py (the LLM agent)
```

**B. Direct python3 from the source tree** *(useful while you're editing a script)*

```bash
cd ~/Dev_WS/raise_summer_school/RAISE2026/raise_ros2_ws/src/raise2026_labs/day1_01_ros2_tools_as_functions/starter
python3 01_drive_forward.py
python3 02_read_lidar.py
python3 03_read_camera.py
python3 04_aim_ptz.py
```

Tip: keep the phone teleop open and drive the Husky around while
`02_read_lidar.py` is running — watch the distance update as you move
closer to a plant.

## Deliverable

`starter/tools.py` (functions filled in) + `starter/schemas.json`
(tool-schema definitions).

## Grading (AssessX, `evaluator/evaluate.py`)

- Topic publishing correctness (40 %)
- Return-shape validation (30 %)
- Schema validity for Anthropic + OpenAI tool-calling (30 %)

## Files

- `README.md` (this file) — ✅
- `starter/01_…10_` progression scripts — ✅ shipped
- `starter/tools.py`, `starter/schemas.json` — ☐ to author (currently inline in `10_orchestrate.py`)
- `solution/` — reference implementation (private) — ☐
- `evaluator/evaluate.py`, `evaluator/scenarios.yaml` — ☐
- `tests/test_tools.py` — ☐
- `instructor.md` — common mistakes (forgetting `rclpy.spin_once`, wrong topic QoS, …)
