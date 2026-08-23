<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Verify & Tune the Auto-Demonstrator (live-sim runbook)

> **Read this before running the auto-demonstrator.** The grasp + spawn pieces
> talk to Gazebo and **could not be tested offline** — this runbook walks you
> through verifying each layer in the live sim and tuning the handful of values
> that depend on your exact robot pose. Do the steps in order; each one isolates
> one thing, so when something breaks you know exactly where.

## What we added (and why)
The Day-1 sim can't pick: no grasp physics, no IK, no tabletop tomato. So we added:
- **Graspable tomatoes** `tomato_red` / `tomato_green` (gravity off → they stay
  where spawned) — `raise2026_worlds/meshes/`.
- **`grasp_server`** — a deterministic "fake grasp": when the gripper closes near
  a registered tomato, it sticks to the tool; opening releases it. `raise2026_tools`.
- **World-pose bridge** `/gz_world_poses` — every model/link's WORLD pose, so the
  grasp uses one consistent frame. `raise2026_bringup/config/ros_gz_bridge.yaml`.
- **`05_auto_demonstrate.py`** — scripts the pick of the red tomato and records
  LeRobot episodes, using the *no-IK trick* (spawn the tomato where the tool lands).

## Build first
```bash
cd ~/Dev_WS/raise_summer_school/RAISE2026/raise_ros2_ws
# (strip anaconda from PATH, then:)
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

---

## Step 1 — Sim + world poses are flowing
```bash
raise-sim                                    # terminal 1
ros2 topic echo /gz_world_poses --once       # terminal 2
```
✅ **Expect:** a `TFMessage` listing many transforms. **Find the gripper frame**
— look for a `child_frame_id` like `gripper_mount_link` (or similar). Note its
exact name.

> ⚙️ **TUNE #1 — gripper link name.** If it isn't exactly `gripper_mount_link`,
> pass the real name everywhere: `--gripper-link <name>` (demonstrator) and
> `-p gripper_link:=<name>` (grasp_server). If `/gz_world_poses` is empty, the
> bridge entry didn't load — confirm the world is `greenhouse_2026` and rebuild
> `raise2026_bringup`.

## Step 2 — Spawning works
```bash
gz service -s /world/greenhouse_2026/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'sdf_filename: "model://tomato_red", name: "t_test", pose: {position: {x: -5, y: 1.5, z: 1.0}}'
```
✅ **Expect:** `data: true`, and a red ball appears in Gazebo. Remove it:
```bash
gz service -s /world/greenhouse_2026/remove --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean --req 'name: "t_test", type: MODEL'
```
> If `model://tomato_red` isn't found, the resource path didn't pick up the new
> model — confirm `install/raise2026_worlds/share/.../meshes/tomato_red/` exists
> and you re-sourced `install/setup.bash`.

## Step 3 — The grasp attaches
```bash
ros2 run raise2026_tools grasp_server         # terminal 3  (alias: grasp_d3)
ros2 topic echo /grasp/state                  # terminal 4
```
Then in another terminal, spawn a tomato right at the gripper and close it:
```bash
# 1) read the gripper tool point from /gz_world_poses, 2) spawn tomato_red there,
# 3) ros2 topic pub --once /grasp/register std_msgs/String "{data: tomato_red_0}"
# 4) close the gripper:  ros2 service call /close_gripper std_srvs/srv/Trigger
```
✅ **Expect:** `grasp_server` logs `grasped "tomato_red_0"`, `/grasp/state` shows
the name, and the tomato follows the gripper as you jog it (`01_teleop.py`).
Opening (`/open_gripper`) releases it.

> ⚙️ **TUNE #2 — attach_radius.** If close-but-no-grab: increase it
> `ros2 run raise2026_tools grasp_server -p attach_radius:=0.20`. If it grabs the
> wrong/too-far tomato: decrease it. **TUNE #3 — grasp_offset** shifts the grab
> point along the tool axis to sit between the fingers.

## Step 4 — Run the auto-demonstrator (a few episodes)
```bash
ros2 run raise2026_labs 05_auto_demonstrate.py --episodes 4 --team team07 --hf-user me
# alias:  05_d3 --episodes 4 --team team07 --hf-user me
```
On start it prints the **left/right grasp points** it found. Watch the arm: it
should approach → close on the red tomato → lift → swing → release, twice per
side, and report `✓ episode N`.

> ⚙️ **TUNE #4 — grasp poses.** If the arm pose looks wrong (tool not where a
> tomato should sit, or unreachable), edit `GRASP_LEFT` / `GRASP_RIGHT` /
> `POSE_HOME` near the top of `05_auto_demonstrate.py`. Because of the no-IK
> trick the tomato is spawned wherever the tool lands, so these only need to be
> **stable, reachable, and visibly distinct (left vs right)** — not precise.

✅ **Done when:** episodes save under `RAISE2026/datasets/me__raise_ripeness_sort_team07` (inside the repo)
with no errors. Scale up: `--episodes 40`.

## Step 5 — Replay an episode (verify the DATA before training)
The cheapest data check: play a recorded episode back into the sim and watch it.
```bash
ros2 run raise2026_tools grasp_server                                  # terminal 3
ros2 run raise2026_labs 06_replay_episode.py --task C --team team07 \
    --hf-user me --episode 0 --spawn                                   # terminal 2
# alias:  06_d3 --task C --team team07 --hf-user me --episode 0 --spawn
```
✅ **Expect:** the arm reproduces the recorded motion and (with `--spawn`) picks
the red tomato. **Smooth motion ending in a grasp → your data is good.** Jerky /
wrong / no-grasp → fix the demos, not the model. Replay 3–4 episodes before
training. (Without the sim, inspect the raw capture with LeRobot's viewer:
`python -m lerobot.scripts.visualize_dataset --repo-id <id> --episode-index 0`.)

---

## Then: train and use it
Exactly the same as the manual path — point the trainer at this dataset:
```bash
finetune_d4 --task C --team team07 --hf-user me --steps 3000 --launch
# when done:
export VLA_LOCAL_CKPT=~/raise_checkpoints/smolvla_C_team07
vla_d4 --task C --instruction "pick the red tomato"
```
Full details: [`HOW_TO_TRAIN_AND_USE.md`](./HOW_TO_TRAIN_AND_USE.md).
The `grasp_server` must also be running during `vla_executor.py` so the trained
policy's grip actually picks.

## The four things to tune (summary)
| # | What | Where | Symptom if wrong |
|---|------|-------|------------------|
| 1 | gripper link name | `--gripper-link` / `-p gripper_link:=` | "gripper link not in /gz_world_poses" |
| 2 | `attach_radius` | `grasp_server -p attach_radius:=` | closes but doesn't grab |
| 3 | `grasp_offset` | `grasp_server -p grasp_offset:=` | tomato sits off the fingers |
| 4 | `GRASP_LEFT/RIGHT`, `POSE_HOME` | top of `05_auto_demonstrate.py` | arm pose looks wrong/unreachable |
