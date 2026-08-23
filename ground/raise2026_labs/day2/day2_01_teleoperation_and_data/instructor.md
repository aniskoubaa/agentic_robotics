<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Lab 2.1 — Instructor notes

## Pre-lab checklist (do the morning of)
1. `raise-sim` opens with the greenhouse; `ros2 topic echo /joint_states --once` works.
2. `grasp_d3` starts and logs `grasp_server ready`.
3. Venv healthy: `~/raise_venvs/lerobot/bin/python3 -c "import lerobot"` — if a
   student machine lacks it, the install is ~10 min (HOW_TO_TRAIN_AND_USE §1).
4. One smoke episode: `05_d3 --episodes 1 --dry-run` → `grasp OK`.
5. The reference dataset is in the repo (`RAISE2026/datasets/...ref/`) — show
   its README (the GIFs) as the "this is what you're producing" opener.

## Timing plan (105 min)
| min | activity |
|---|---|
| 0–10 | concept: imitation = copy the mapping; show the reference-dataset GIFs |
| 10–25 | Path B taste: everyone drives `01_d3` for 10 min (let them feel the pain) |
| 25–35 | `02_d3` pre-flight: what the model actually sees |
| 35–75 | Path A: `05_d3 --episodes 30`; while it runs, walk through the script's WHY comments (grasp-by-decree, no-IK trick) |
| 75–90 | `06_d3 --spawn` replay verification — every team replays ≥1 episode |
| 90–105 | run `evaluator/validate_dataset.py`, discuss scores; buffer |

## The 4 sim tunables (when a machine misbehaves)
Full walkthrough: `../VERIFY_MANIPULATION.md`. Quick table:
| Symptom | Fix |
|---|---|
| "no tool pose yet" | gripper link name → `--gripper-link` / `-p gripper_link:=` |
| closes but doesn't grab | `grasp_server -p attach_radius:=0.20` |
| tomato off the fingers | `-p grasp_offset:=` (default 0.13) |
| arm pose looks wrong | `GRASP_LEFT/RIGHT` in `task_packs/common/sim_poses.py` |

## Known failure modes (seen live)
- **`grasp did NOT attach ... state=tomato_green_0`** — stale registration; fixed
  in current `grasp_server` (re-registration re-resolves). If seen: restart `grasp_d3`.
- **Recorder saves 0-frame episodes** — streams not up yet; wait for `02_d3` to
  show both streams before recording.
- **`lerobot not installed`** — student used `ros2 run` instead of the alias for
  03/05/06 (system python has no lerobot — by design).
- **Wrong `--team`/`--hf-user` later** — the trainer won't find the dataset;
  names must match across 05 → finetune → executor.

## Grading
`evaluator/validate_dataset.py --task C --team <t> --hf-user <u>` (venv python).
Reference dataset = 80/100 (30/50 episodes). A team recording 50 episodes with
both-side variety should hit ~100. The JSON line is the gradebook entry.
