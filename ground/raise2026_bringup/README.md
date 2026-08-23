# `raise2026_bringup`

Launch files that bring up the simulation.

## What's here (v0.1)

| File                                  | Role                                                              |
| ------------------------------------- | ----------------------------------------------------------------- |
| `launch/sim.launch.py`                | World + Husky/UR5e + state publisher + minimal ROS↔Gz bridge      |
| `launch/world_only.launch.py`         | Gazebo + greenhouse world only (smoke test, no robot)             |
| `config/ros_gz_bridge.yaml`           | Bridge config: `/clock`, `/cmd_vel`, `/odom`                      |

## Deferred (later steps)

- `launch/nav.launch.py` — Nav2 with oracle localization. Added when D1L1 demo needs it.
- `launch/arm_moveit.launch.py` — MoveIt 2 for the UR5e arm.
- Wrist camera + joint_states bridges — require `<gazebo>` sensor blocks added to the URDF.

## Usage

After the workspace is built (`colcon build --symlink-install` inside the container):

```bash
# Full sim
ros2 launch raise2026_bringup sim.launch.py

# CPU-only laptop — use the lite world
ros2 launch raise2026_bringup sim.launch.py world:=greenhouse_2026_lite.sdf

# Spawn robot at a different aisle
ros2 launch raise2026_bringup sim.launch.py y:=-1.0

# World alone (no robot, no bridge — fast sanity check)
ros2 launch raise2026_bringup world_only.launch.py
```

## Convention

Python launch files only — needed for the substitution-vs-path-resolution
pattern (`OpaqueFunction` resolving `world:=...` into an SDF path). XML launch
files don't compose Python logic cleanly, so we stay with Python here.
