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
agr-sim ground tools:=true               # RaiseBot + its service servers
agr-sim legged                           # Unitree Go2
agr-stop                                 # stop everything
agr_help                                 # every command
```

Check it works — every platform has one, and it is the first thing to run
when anything is odd:

```bash
ros2 run agr_uav_demos diagnose        # UAV
ros2 run raisebot_demos diagnose       # ground
ros2 run agr_legged_demos diagnose     # legged
```

---

## Repository layout

Platforms live in separate subtrees and share exactly one package. Nothing in
`uav/` loads anything from `ground/`, or the reverse — that isolation is the
structure's whole job, so a new platform is a new folder rather than an edit
everywhere.

Each platform carries the **same three teaching layers**, so what you learn on
one transfers to the next:

| layer | what it is for |
|---|---|
| `*_teleop` | drive it by hand |
| `*_examples` | six short scripts, one idea each |
| `*_demos` | `diagnose` when it is broken, `demo_*` when someone is watching |

```
agentic_robotics/
├── agr-sim  agr-stop  agr-build      # the only commands you type
├── agr_aliases.sh  setup.sh
├── common/
│   └── agr_core/                     # interfaces shared by ALL platforms
├── uav/                              # PX4 SITL
│   ├── agr_uav_bringup/              # single- and multi-vehicle launch
│   ├── agr_uav_description/          # airframe registry + capabilities
│   ├── agr_uav_tools/                # PX4 topic resolver, offboard Pilot
│   ├── agr_uav_worlds/               # agr_city, agr_defense
│   ├── agr_uav_teleop/               # keyboard flight, camera view
│   ├── agr_uav_examples/             # 01..06
│   ├── agr_uav_demos/                # diagnose, demo_flight
│   └── agr_uav_labs/                 # numbered exercises
├── ground/                           # RaiseBot: Husky + UR5e + Robotiq
│   ├── raisebot_bringup/             # greenhouse launch + tools.launch.py
│   ├── raisebot_worlds/              # greenhouse_2026(_lite)
│   ├── raisebot_description/         # Husky + arm + gripper + PTZ mast
│   ├── raisebot_tools/               # gripper / nav / detector / inspector servers
│   ├── raisebot_teleop/              # keyboard, phone, joystick
│   ├── raisebot_examples/            # 01..06
│   ├── raisebot_demos/               # diagnose, demo_greenhouse
│   └── raisebot_labs/                # Day 1-3 exercises
└── legged/                           # Unitree Go2, 12 DOF
    ├── agr_legged_bringup/           # launch + stand + trot gait
    ├── agr_legged_description/       # URDF, head camera
    ├── agr_legged_worlds/            # agr_inspection: stairs, pipes, debris
    ├── agr_legged_teleop/            # keyboard (incl. strafe), camera view
    ├── agr_legged_examples/          # 01..06
    └── agr_legged_demos/             # diagnose, demo_walkabout
```

`common/agr_core` is platform-neutral and is the only thing all three share.

---

## Learning across the three

The examples are numbered to be read in order, and two pairs are worth running
back to back:

**The same square, on two platforms.** `agr_uav_examples 04_fly_a_square`
closes a 20 m square to about **0.10 m**. `agr_legged_examples
06_walk_a_square` finishes roughly **10° off heading** and metres away, and
grades itself from the IMU. Flying is not easier than walking — PX4 runs a
state estimator and a position controller, so "go to (5, 0)" is a closed loop
that keeps correcting. The legged gait is open loop and nothing ever checks.
That contrast is the argument for everything in the labs.

**Topic versus service.** `raisebot_examples 01_drive` publishes a velocity and
hopes; `05_call_a_service` calls a function that answers. "Did the gripper
close?" has an answer, "drive at 0.3 m/s" does not — and an LLM can call a
function but cannot run a control loop. That is why `raisebot_tools` exists.

**What legs buy you.** `agr_legged_teleop`'s `q` and `e` strafe: the Go2 walks
sideways without turning. A differential-drive base cannot do that at any
speed.

---

---

## Commands

```bash
# UAV
agr-sim                                  # x500, empty world, GUI
agr-sim world:=agr_city                  # urban
agr-sim world:=agr_defense               # secured installation
agr-sim airframe:=rc_cessna              # fixed-wing
agr-sim uav --multi count:=3             # three vehicles

# Ground (RaiseBot)
agr-sim ground                           # Husky in the greenhouse
agr-sim ground tools:=true               # ...and the six service servers
agr-sim ground world:=greenhouse_2026_lite.sdf
agr-sim ground --world-only

# Legged (Unitree Go2)
agr-sim legged                           # inspection site, stands, then trots
agr-sim legged gait:=false               # no gait — for the pose examples

agr-stop                                 # stops whichever is running
agr-build                                # colcon, with the right python
agr_help                                 # everything
```

---

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
