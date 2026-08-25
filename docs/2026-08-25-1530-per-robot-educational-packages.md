# Per-robot educational packages: teleop, examples, demos

**Date:** 2026-08-25
**Scope:** `~/ros2_ws/src/agentic_robotics` — all three platforms

## Goal

RaiseBot had teleop scripts from training; UAV and legged had nothing
comparable. Give every platform the same three teaching layers, add an
elementary examples layer to all three, and a demo/diagnostics layer to all
three.

## Result

| layer | ground | uav | legged |
|---|---|---|---|
| `*_teleop` | existed | **new** | **new** |
| `*_examples` | **new** | **new** | **new** |
| `*_demos` | **new** (was an empty skeleton) | **new** | **new** |

7 new packages, 28 new executables. Workspace went from 18 to 25 packages;
48 console entry points import clean.

## Capability gaps that had to close first

Three platform defects made the requested examples impossible to write. Each
was found by trying to write the example, and each is fixed and measured.

### 1. The Go2 could not walk

It could stand and nothing else, so "make the robot dog walk" had nothing to
call. Added `agr_legged_bringup/gait.py`: a trot gait with 3D leg IK, driven
by `/cmd_vel` so the quadruped speaks the same interface as the wheeled base.

The hip joints matter more than they look. With hips pinned at zero the robot
can only skid-steer, and it yaws at a few percent of the commanded rate
because the feet scrub. Giving each foot a lateral component — which is what
the hip is for — brought yaw to ~90% of command and added strafing, which no
wheeled base can do.

Measured limits, in the source:

| commanded | achieved | outcome |
|---|---|---|
| 0.20 m/s | 0.17 m/s | upright |
| 0.25 m/s | 0.18 m/s | upright |
| 0.35 m/s | 0.22 m/s | **on its back** |
| 0.50 m/s | 0.18 m/s | **on its back** |

Achieved speed barely rises across that range: the extra stride goes into
pitching the body, not into travel. That is an open-loop gait at its limit, so
`MAX_VX` clamps at 0.25 rather than a comment saying to be careful.

### 2. The Go2 had no camera

An IMU and nothing else, so every perception exercise had to be written for a
different platform. Added a head camera.

The world needed `gz-sim-sensors-system` named explicitly. With no `<plugin>`
tags at all, `gz sim` falls back to a default set that has no Sensors system,
and a camera declared in the URDF then advertises nothing — no error, just a
topic that never exists. Naming any plugin replaces the whole default set, so
Physics/UserCommands/SceneBroadcaster had to be listed too.

Mounted above the trunk and pitched 3°, not the 10° that seems natural for a
walking robot: at 10° the horizon sits 32% down the image and the remaining
two thirds is featureless ground plane. Both were rendered before choosing.

### 3. The RaiseBot PTZ camera could not aim

Commanding a 0.6 rad pan moved the joint at ~0.1 rad/s and usually never
arrived, so every "aim the camera and look" script photographed the old view.
Two causes in the `gz` `JointPositionController` config:

* `d_gain` 20 on a link with inertia 5e-5 kg·m² is wildly over-damped — the
  derivative term cancelled nearly all of the proportional one. The arm gets
  away with `d_gain` 80 because it swings tens of kilograms.
* `i_max`/`i_min` were absent. gz-math's PID defaults them to **zero**, which
  clamps the integral contribution to nothing, so steady-state error was never
  corrected. The arm's plugin sets them; the PTZ's did not.

Retuned to 500/10/1 with `i_max` 5. Targets now arrive in 0.25–0.6 s.

## Other fixes made along the way

**UAV had no camera on the ROS side.** PX4 telemetry crosses XRCE-DDS; the
camera is a Gazebo sensor and needs a separate `ros_gz_bridge` that did not
exist. Airframes now declare their sensor link in `airframes.yaml`, and the
launch file builds the Gazebo topic — it embeds both the world name and the
PX4 instance — and remaps it to plain `/camera`.

**Ground tool servers had no launch file.** Six nodes started by hand means six
terminals, which means nobody starts them, and a service call with no server
blocks *forever* — so the labs appear to hang. Added `tools.launch.py` and a
`tools:=` argument on the sim launch.

**`stand` spun forever holding its pose.** The position controller holds the
last command on its own, so a lingering publisher gains nothing and costs a
lot: everything else that drives the joints then shares one controller with
it, and the robot thrashes onto its back within seconds. It now exits.

## PX4 traps now documented in `agr_uav_tools/offboard.py`

Four rules that are not discoverable and each fail silently: setpoints must
stream *before* the mode request; they must never stop; positions are NED so
up is negative; the QoS must match or a subscription never receives.

And one that cost real time: **`vehicle_status` publishes only on change**, and
the XRCE-DDS agent does not replay the latched sample. A node starting after
the vehicle settles waits forever for a message a healthy PX4 has no reason to
send. `ros2 topic hz /fmu/out/vehicle_status_v4` returns nothing on an idle
vehicle, while `vehicle_control_mode` streams at 2 Hz with the same arming
flag. Arming state now comes from the latter.

## Recurring gotcha worth naming

`get_topic_names_and_types()` and `count_subscribers()` report what *this node
has discovered so far*, and DDS discovery is not instant. Calling either on the
first line of a check reports a live system as having nothing — the most
misleading possible diagnostic output. This bit three separate scripts here;
all now settle first.

## Cross-platform teaching contrasts

Deliberately built so the same exercise runs on two platforms:

* **The same square.** `agr_uav_examples 04_fly_a_square` closes a 20 m square
  to **0.10 m**; `agr_legged_examples 06_walk_a_square` finishes ~**10° off**
  heading and grades itself from the IMU. Flying is not easier than walking —
  PX4 runs an estimator and a position controller, so "go to (5, 0)" is a
  closed loop. The gait is open loop and nothing checks.
* **Topic versus service.** `raisebot_examples 01_drive` publishes and hopes;
  `05_call_a_service` calls a function that answers. An LLM can call a
  function; it cannot run a control loop.
* **What legs buy.** `q`/`e` in the legged teleop strafe sideways without
  turning.

## Verification

Every claim above was produced by running the code against a live simulator,
not by inspection.

* 25 packages build clean from scratch; 48 entry points import; 7 launch files
  construct.
* **Legged** — 9/9 diagnostics; all 6 examples run; all 4 pose transitions
  reach target height (stand 0.324 m, crouch 0.164 m, tuck 0.110 m); 39 s
  walkabout finishes upright; forward/back/strafe/yaw all measured.
* **UAV** — 9/9 diagnostics including the active arm test; takeoff/hover/land
  lands at 0.02 m; square closes to 0.10 m; camera writes 1280×960; full demo
  flight completes with the orbit; **all eight teleop keys verified in flight**
  with the nose north.
* **Ground** — 10/10 diagnostics; all 6 examples run; three cameras return
  frames including the depth colormap; PTZ reaches 1.181 rad when asked for
  1.2; all 4 waypoints arrive within 0.39 m; 7-step greenhouse demo completes.

## Known limitations

* The Go2 gait is open loop. It walks on flat ground and will **not** climb the
  staircase in `agr_inspection` — that needs state estimation and a balance
  controller.
* Drive the RaiseBot into the crop rows and it wedges; `nav_to_*` then times
  out reporting "travelled 0.00 m". Restart the sim.
* `vehicle_local_position` / `vehicle_global_position` are advertised but never
  carry a sample. Not root-caused; worked around via `vehicle_odometry`.
