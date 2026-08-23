<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# The RAISE 2026 Reference Dataset — what we built and how

> **In one sentence:** we made the robot demonstrate *"pick the red tomato"* to
> itself 50 times inside the simulator — scanning for the red one at a real plant row — no human teleoperation — and saved
> every demonstration in **LeRobot format**, ready to fine-tune SmolVLA.
>
> This document explains **what** is in the dataset, **how** it was created,
> and **why** each design decision was made — so a student (or instructor) can
> reproduce, extend, or critique it. Slides version:
> `lectures/day2_l2_vla_api_imitation/` (dataset-pipeline deck).

---

## 1. What the dataset is

| Property | Value |
|---|---|
| Name (repo id) | `raiseschool/raise_ripeness_sort_ref` |
| Location | `RAISE2026/datasets/raiseschool__raise_ripeness_sort_ref` (inside the repo — versioned with the code) |
| Task / instruction | **"pick the red tomato"** (red vs green distractor) |
| Episodes | 49 kept of 50 attempts (red LEFT in even episodes, RIGHT in odd — 25/24; scan-then-pick; 1 attempt failed the record-time grasp check and was auto-discarded) |
| Frames per episode | 76 (red-left) / 97 (red-right — includes the scan leg) @ 10 Hz |
| Total frames | 4228 |
| Format | LeRobot **v3.0** dataset (parquet + video/images + metadata) |
| Robot | UR5e + Robotiq 2F-85 on a Husky base (stationary), Gazebo Harmonic |

**Each frame is one training example** — a snapshot of "what the robot saw,
where it was, and what it was told to do next":

| Field | Shape | Meaning |
|---|---|---|
| `observation.images.wrist` | 224×224×3 RGB | what the wrist camera **saw** |
| `observation.state` | 7 floats | where the robot **was**: 6 arm joint angles + gripper |
| `action` | 7 floats | what was **commanded** next: 6 joint targets + gripper |
| `task` | string | the language instruction ("pick the red tomato") |

The VLA learns the mapping **(image, task, state) → action**. That's imitation
learning — no rewards, no trial-and-error; the model copies the demonstrator.

## 2. How it was created — the auto-demonstrator pipeline

Manual teleoperation of six joints with a keyboard is painful and produces
jerky demos. Instead, a **scripted expert** performs perfect picks and records
itself. One command:

```bash
# sim (headless works!)     ros2 launch raise2026_bringup sim.launch.py headless:=true
# the grasp                 ros2 run raise2026_tools grasp_server
# the demonstrator          05_d3 --episodes 50 --team ref
```

Per episode, `05_auto_demonstrate.py` does:

```
1. SPAWN    a red and a green tomato at the two reachable grasp points
            (red side alternates L/R — the "language grounding" variation)
2. HOME     move the arm to the start pose, gripper open
3. RECORD   approach → descend → close (grasp!) → lift → swing → release
            — capturing (image, state, action) at 10 Hz throughout
4. VERIFY   /grasp/state must confirm the RED tomato attached; failed
            grasps are DISCARDED, never recorded
5. CLEANUP  remove both tomatoes; next episode
```

### The three tricks that make it work

**① Deterministic grasping (`grasp_server`).** Gazebo's physics can't reliably
hold an object with friction, and the Robotiq model has no grasp plugin. The
`grasp_server` implements grasping *by decree*: when the gripper is commanded
closed within 15 cm of a registered tomato, the tomato is attached (teleported
to follow the tool each tick); commanded open ⇒ released. It watches the
**command** topic, so the scripted expert, a human, and the trained VLA policy
all grasp exactly the same way.

**② The no-IK trick.** There is no inverse kinematics in the sim. So instead of
computing *"which joint angles reach the tomato?"*, we invert the problem: move
the arm to a chosen joint pose, read **where the gripper tool actually ended
up** (TF + the robot's world pose from Gazebo), and **spawn the tomato there**.
Every grasp is reachable *by construction*.

**③ Gravity-off tomatoes.** The spawned tomatoes have `<gravity>false</gravity>`
— they float exactly where placed (no table needed) and can be carried, dropped,
and re-placed with zero physics tuning.

### Is that cheating? (the honest pedagogy note)

The *grasp mechanics* are simplified — but the **learning problem is real**:
the VLA never sees spawn coordinates, grasp radii, or the scripted plan. It
sees only **pixels + the instruction + joint state**, and must produce joint
actions. Whether the tomato sticks by friction or by decree is invisible in
the training data. What the model must genuinely learn:
- **language grounding** — "red" selects WHICH side to reach (the layout
  alternates, so a fixed motion scores ~50%);
- **visually-guided reaching** — where the tomato appears in the image
  determines the arm trajectory;
- **the manipulation sequence** — approach, descend, close, lift, place.

## 3. Why these design choices

| Choice | Why |
|---|---|
| Scripted expert instead of teleop | perfectly smooth, consistent demos; zero human time; grasp verified per episode — *data quality beats data heroics* |
| Red L / green R alternation | forces the policy to use the instruction+image, not memorize one trajectory; balanced 15/15 so neither side dominates |
| 10 Hz, ~7 s episodes | matches the control rate the executor uses; short, dense episodes are ideal for behavioral cloning |
| 224×224 wrist camera | SmolVLA's expected input size; resized with pure numpy (no cv_bridge — see §5) |
| `action` = the **commanded** target (not the measured pose) | behavioral cloning wants the *intent* at each timestep; measured positions lag the command |
| Discard failed grasps | a demo that failed teaches failure — the verify step keeps the dataset clean |

## 4. Validate it yourself (do this before training!)

```bash
# 1. numbers + a sample frame
~/raise_venvs/lerobot/bin/python3 - <<'PY'
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset('raiseschool/raise_ripeness_sort_ref',
                    root='<repo>/RAISE2026/datasets/raiseschool__raise_ripeness_sort_ref')
print(ds.num_episodes, 'episodes,', ds.num_frames, 'frames @', ds.fps, 'Hz')
PY

# 2. replay an episode into the sim and WATCH it (grasp_server running)
06_d3 --task C --team ref --episode 0 --spawn
```

If the replay looks like a clean pick — the data is good; train on it.
If it doesn't — fix the demos, not the model.

## 5. The environment it runs in (important!)

LeRobot (≥0.5) requires **numpy 2**; ROS's `cv_bridge` requires numpy 1 — so
LeRobot lives in a dedicated venv and the Day-2 scripts avoid `cv_bridge`
entirely (pure-numpy image conversion in `api_clients/vla_client/ros_image.py`):

```bash
/usr/bin/python3 -m venv --system-site-packages ~/raise_venvs/lerobot
~/raise_venvs/lerobot/bin/python3 -m pip install "lerobot[smolvla]"
```

Anything that imports lerobot (record / replay / train / execute) runs with
`~/raise_venvs/lerobot/bin/python3`; see [`HOW_TO_TRAIN_AND_USE.md`](./HOW_TO_TRAIN_AND_USE.md).

## 6. Next step: train on it

```bash
finetune_d4 --task C --team ref --hf-user raiseschool --steps 3000 --launch
tail -f ~/raise_checkpoints/smolvla_C_ref/train.log
# when done:
export VLA_LOCAL_CKPT=~/raise_checkpoints/smolvla_C_ref
vla_d4 --task C --instruction "pick the red tomato"
```

## 7. Reproduce / extend

- **More episodes:** `05_d3 --episodes 50 --team ref2`
- **More variety:** add jitter to the two grasp poses in `05_auto_demonstrate.py`
  (`GRASP_LEFT/RIGHT`), or add a third spawn point.
- **A row-conditioned task:** `05_d3 --row 2` weaves "in row 2" into the
  instruction — the hook for the mobile-manipulation extension (Task B).
- Design context: [`DAY2_LAB_DESIGN.md`](./DAY2_LAB_DESIGN.md) ·
  live-sim tuning: [`VERIFY_MANIPULATION.md`](./VERIFY_MANIPULATION.md)
