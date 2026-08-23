# Agentic Robotics

ROS 2 simulation stack for teaching and research. UAV first; ground and legged
platforms slot in beside it without restructuring.

Built for **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + PX4 SITL**.

---

## Quick start

```bash
git clone https://github.com/aniskoubaa/agentic_robotics.git ~/ros2_ws/src/agentic_robotics
cd ~/ros2_ws/src/agentic_robotics && ./setup.sh
```

Then, in a **new** shell:

```bash
agr-sim                                  # one x500, Gazebo GUI
agr-sim headless:=true                   # no GUI
agr-sim airframe:=rc_cessna              # fixed-wing
agr-sim --multi count:=3                 # three vehicles
agr-stop                                 # stop everything
agr_help                                 # every command
```

Check it works:

```bash
ros2 run agr_uav_labs 01_check_bridge
```

---

## Packages

| Package | Purpose |
|---|---|
| `agr_core` | Interfaces shared by every platform — `MissionPlan`, `Waypoint`, `Violation`, `MissionConstraints`, `VerificationResult`, `VerifyMission.srv` |
| `agr_uav_description` | Airframe registry (`airframes.yaml`) + typed lookup |
| `agr_uav_bringup` | `single.launch.py`, `multi.launch.py` |
| `agr_uav_tools` | `px4_topics` resolver, `vehicle_monitor`, `list_airframes` |
| `agr_uav_worlds` | Gazebo worlds |
| `agr_uav_labs` | Numbered student exercises |

Adding a platform means adding `agr_ground_*` / `agr_legged_*` alongside;
`agr_core` is already platform-neutral.

---

## Airframes

```bash
$ ros2 run agr_uav_tools list_airframes
NAME            PX4 MODEL             AUTOSTART  CLASS       HOVER
rc_cessna       gz_rc_cessna          4003       fixedwing   NO
standard_vtol   gz_standard_vtol      4004       vtol        yes
x500            gz_x500               4001       multirotor  yes
x500_mono_cam   gz_x500_mono_cam      4010       multirotor  yes
```

The three are chosen because they make **different missions infeasible**, not
because they look different:

* **x500** — hovers, so "loiter here" is always feasible. The baseline.
* **standard_vtol** — adds mode transitions; a mission can be geometrically
  perfect and still infeasible below minimum airspeed.
* **rc_cessna** — cannot hover, cannot stop, has a minimum turn radius. Breaks
  every waypoint-spacing and loiter assumption that silently assumed a
  multirotor.

Capabilities live in `airframes.yaml`, so a new platform is a config edit.

---

## Four PX4 facts that cost a day each

Everything here exists because of one of these. They are documented in the code
at the point they matter.

**1. Anaconda hides ROS.** If anaconda is on `PATH`, `python3` is its
interpreter (3.11) while ROS 2 Jazzy needs the system 3.12. `colcon build`
dies with `No module named 'em'` even though empy is installed — it is just
invisible to that interpreter. `agr-build` and `agr-sim` strip anaconda
internally; your interactive shell is left alone.

**2. Topic names carry version suffixes — inconsistently.** At PX4 `03bf4a5e`:

```
/fmu/out/vehicle_status_v4          <- suffixed
/fmu/out/vehicle_local_position_v1  <- suffixed
/fmu/out/vehicle_global_position    <- NOT suffixed
/fmu/out/failsafe_flags             <- NOT suffixed
```

The suffix moves between releases. Resolve through `px4_topics.resolve()`.

**3. Namespaces are asymmetric.** PX4 gives instance 0 **no** namespace and
instance N the namespace `px4_N`. Single-vehicle code written against the bare
`/fmu/...` path works perfectly, then breaks the moment a second vehicle
appears. Every node takes `namespace` as a parameter, defaulting to `''`.

**4. `vehicle_status` publishes ON CHANGE.** On an idle, disarmed vehicle it is
silent — for minutes — while the bridge is entirely healthy. Two consequences:

* Never test liveness against it. `01_check_bridge` uses `battery_status`
  (periodic, 1 Hz), so silence genuinely means no data path.
* Never display a cached sample as if it were current. `vehicle_monitor` stamps
  every arrival and prints `[state 47s old]` past 5 s, because a stale
  confident `ARMED` is worse than no reading at all.

And underneath all of it: PX4 publishes **BEST_EFFORT**. A default (RELIABLE)
subscription is incompatible, so it never matches and never errors — no data,
indistinguishable from a dead bridge. Use `px4_topics.PX4_QOS`.

---

## Requirements

* Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8)
* PX4-Autopilot built for SITL (`make px4_sitl_default`)
* `px4_msgs` in the workspace at a commit **matching your PX4 build** — a
  mismatch does not error, it silently stops publishing some uORB topics
* Micro XRCE-DDS Agent on `PATH`

PX4 needs ROS sourced to **run**, not just to build: Gazebo comes from the ROS
vendor packages, so `libgz-*.so` is on `LD_LIBRARY_PATH` only while ROS is
sourced. `agr-sim` handles this.

---

## Licence

MIT. Prof. Anis Koubaa — anis.koubaa@gmail.com
