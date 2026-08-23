<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Lab 2.1 — Teach the Robot by Showing It

**Day 2, 14:00 – 15:45 (105 min) · for everyone · graded task: C ("pick the red tomato")**

## The objective of this lab

> **The objective of this lab is to produce a dataset of demonstrations — the
> robot picking the red tomato, recorded through its own camera — that a VLA
> model will be trained on this afternoon.**

You are not writing motion code. You are creating **examples** for the robot to
copy. One sentence to remember all day:

> **What you record this morning is the brain you command this afternoon.**

## Why (the whole idea in four lines)

A VLA learns by **imitation**: it sees thousands of examples of *"the camera
saw this, and the command was that"* and learns to copy the mapping. There is
no reward and no trial-and-error — the dataset *is* the teacher. Which means:
**garbage demos in, garbage policy out.** This lab is about producing *clean,
varied* demonstrations efficiently.

## Two ways to collect — pick your path

| | 🤖 **Path A — auto-demonstrator (recommended)** | ⌨️ Path B — manual teleop |
|---|---|---|
| Who drives | a scripted expert performs perfect picks | you, with the keyboard |
| Scripts | `05_auto_demonstrate.py` | `01_teleop.py` + `03_record.py` |
| 30 episodes takes | **~15 minutes, hands-free** | ~1 hour of concentration |
| Quality | every grasp verified, perfectly smooth | as good as your driving |
| You learn | how a data pipeline is engineered | how hard teleop really is (worth 10 min!) |

> 💡 **Suggested flow:** try Path B for ~10 minutes to *feel* why teleop is hard
> — then collect your real dataset with Path A. The result is identical in
> format; imitation learning doesn't care who the teacher is.

## The scripts (in order)

| # | Script | Alias | What it does / teaches |
|---|--------|-------|------------------------|
| 01 | `01_teleop.py` | `01_d3` | drive the arm+gripper from the keyboard (the publisher pattern, one level below Day 1) |
| 02 | `02_read_streams.py` | `02_d3` | watch `/wrist_camera` + `/joint_states` live — the exact streams the VLA learns from (pre-flight check) |
| 03 | `03_record.py` | `03_d3` | manual recording: fuse the streams into LeRobot episodes while YOU drive |
| 05 | `05_auto_demonstrate.py` | `05_d3` | **hands-free recording**: the expert SCANS (looks above left, then right), picks the red tomato it SEES, side alternating — every grasp verified. Active perception! |
| 06 | `06_replay_episode.py` | `06_d3` | **replay** a recorded episode into the sim — THE data check before training |
| 04 | `04_upload.py` | `04_d3` | *(optional — multi-machine schools only; this edition trains locally)* |

> ⚠️ **Venv rule:** scripts that touch LeRobot (03, 05, 06) run under the
> lerobot venv python — **the aliases handle this automatically.** Plain
> `ros2 run` works only for 01/02. Why: see `HOW_TO_TRAIN_AND_USE.md` §1.

## How to run (Path A — the recommended 20 minutes)

```bash
# terminal 1 — the sim, PARKED AT THE PLANT ROW (Day-2 poses expect this spot)
sim_d2                          # = raise-sim x:=-2.0 y:=2.15 yaw:=1.5708
# terminal 2 — the grasp server (the sim's "grasp physics"; keep it running)
grasp_d3
# terminal 3 — collect 50 verified episodes, hands-free (~25 min)
05_d3 --episodes 50 --team team07 --hf-user me
# then VERIFY the data by replaying an episode and watching the pick:
06_d3 --task C --team team07 --hf-user me --episode 0 --spawn
```

Your dataset lands **inside the repo**: `RAISE2026/datasets/me__raise_ripeness_sort_team07`.
See a finished example (with images!): [`../../../../datasets/raiseschool__raise_ripeness_sort_ref/`](../../../../../datasets/raiseschool__raise_ripeness_sort_ref/README.md).

## Deliverable

A LeRobot dataset (50 episodes for task C) that **replays as a clean pick**
(06's check). That dataset is what Lab 2.2 trains on.

## Grading (auto-scored: `evaluator/validate_dataset.py`)

- **50%** — episodes recorded vs the task target
- **30%** — clean data: no NaNs, 224×224 images, action↔state aligned
- **20%** — diversity: the red tomato must appear on BOTH sides across episodes

The reference dataset scores **100/100** (50 verified episodes, both-side balance).

## Want the full story?

- How collection works under the hood: [`../HOW_IT_WORKS_DATA_AND_TRAINING.md`](../HOW_IT_WORKS_DATA_AND_TRAINING.md)
- The complete beginner runbook: [`../HOW_TO_TRAIN_AND_USE.md`](../HOW_TO_TRAIN_AND_USE.md)
- Sim setup / tuning: [`../VERIFY_MANIPULATION.md`](../VERIFY_MANIPULATION.md)
