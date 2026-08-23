<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Day 2 — Completion Plan

> **Purpose:** everything to take Day 2 from "fully coded, offline-verified" to
> "runs in the sim, trains a model, grades students." Pick a phase to start; the
> phases are ordered by dependency, and Phase 1 is the gate for most of the rest.

## Where we are now (done ✅)
- **Shared client:** `api_clients/vla_client/` (canonical Action, local/remote backends, factory, server).
- **Task packs:** A / C / B (isolated, `--task` switch) + loader.
- **Lab 2.1 starters:** `01_teleop`, `02_read_streams`, `03_record`, `04_upload`, `05_auto_demonstrate`, `06_replay_episode`.
- **Lab 2.2:** `vla_executor`, `finetune_smolvla`.
- **Manipulation layer:** graspable `tomato_red`/`tomato_green`, `grasp_server`, `gz_utils`, `/gz_world_poses` bridge.
- **Docs:** design, model choice, local-vs-remote, how-to-train, verify-manipulation; both lab READMEs; demos; aliases.
- **Verified offline only:** SDF valid, all Python compiles, colcon builds, artifacts install. **Nothing has run in Gazebo yet.**

---

## Phase 1 — Live sim bring-up + tune  🚪 THE GATE
**Goal:** prove the manipulation pipeline works in the real sim and lock the 4 tunables.
**Who:** you drive the sim; I debug from what it prints.
**Steps:** `VERIFY_MANIPULATION.md` 1–5 — world poses flow → spawn works → grasp attaches → demonstrator runs → replay shows a clean pick.
**Tunables to lock:** gripper-link name, `attach_radius`, `grasp_offset`, `GRASP_LEFT/RIGHT` + `POSE_HOME`.
**Deliverable:** a handful of episodes that **replay as clean picks** (`06_d3 --spawn`).
**Acceptance:** arm approaches → grasps red → lifts → places, repeatably, both L and R.
**Effort:** ~1–2 h of live iteration. **Blocks:** Phases 2, 3, 5.
**Risk:** frame names / grasp geometry differ from my offline guesses — expected; that's what this phase fixes.

## Phase 2 — Record a real dataset + one real fine-tune
**Goal:** the **reference checkpoint** the default student path needs, and proof training works on the 4090.
**Who:** you run it (GPU); I tune the recipe.
**Steps:** `05_d3 --episodes 40` → `finetune_d4 --launch` (start `--steps 3000` to prove it, then a longer run). **Measure the wall-clock** and pin it into `DAY2_LAB_DESIGN.md §3.1` + `instructor.md`.
**Deliverable:** `~/raise_checkpoints/smolvla_C_...` + recorded real training time.
**Acceptance:** training completes; loss decreases; checkpoint loads.
**Effort:** ~30 min hands-on + hours background. **Needs:** Phase 1.

## Phase 3 — Executor end-to-end (the payoff)
**Goal:** `vla_executor.py` drives the arm to a real pick from a typed instruction.
**Steps:** `grasp_server` + `export VLA_LOCAL_CKPT=...` + `vla_d4 --task C --instruction "pick the red tomato"`. Replace the scaffold `grasped_and_lifted()` heuristic with real success (use `/grasp/state` from grasp_server — a clean signal).
**Deliverable:** a working spoken-instruction pick + latency report.
**Acceptance:** ≥ some success rate over a few instructions; ≤500 ms/action.
**Effort:** ~1 h. **Needs:** Phase 2.

## Phase 4 — Grading scaffolds  (can build NOW, no sim)
**Goal:** make both labs *gradeable*.
**Who:** me, solo.
**Build:**
- `day2_01/evaluator/validate_dataset.py` — checks episode count, no NaNs, 224×224, action↔state alignment, pose/ripeness diversity → score.
- `day2_02/evaluator/evaluate.py` — runs each pack's `scenarios.yaml`, scores success + latency + recovery (uses `/grasp/state`).
- `instructor.md` for both (run order, common failures, the locked tunables, recovery tips).
- `solution/` notes (private reference).
**Deliverable:** scripts + instructor guides matching the README grading tables.
**Acceptance:** `validate_dataset.py` scores a real Phase-2 dataset; `evaluate.py` runs against the sim once Phase 3 works.
**Effort:** ~2–3 h. **Needs:** nothing to build; needs Phases 1–3 to fully test.

## Phase 5 — Drive-to-row (your full objective: "move to the right place")
**Goal:** demos + executor include the Husky approaching the row before picking.
**Build:** add a base-drive phase to `05_auto_demonstrate` (scripted `/cmd_vel` to a row, stop in reach) and let `vla_executor` emit the `base` action (Task B already in the contract); instruction "pick the red tomato in row N".
**Deliverable:** mobile-manip demos + execution.
**Acceptance:** Husky drives to the row, then the arm picks, recorded + replayable.
**Effort:** ~3–4 h. **Needs:** Phase 1 (pick confirmed) first; base+arm world-frame coordination is the new risk.

## Phase 6 — Slides  (last)
**Goal:** the D2L1 architecture section + the D2L2 deck, explaining what the working labs do.
**Effort:** ~2–3 h. **Needs:** labs working (Phases 1–3) so the slides describe reality.

---

## Recommended order & the one decision
```
Phase 1 (gate) ─┬─► Phase 2 ─► Phase 3 ─► Phase 5 ─► Phase 6
                └─► Phase 4 (build in parallel; final test after 1–3)
```
**The only real choice right now:** are you **at the sim** (→ start Phase 1, the gate, and I build Phase 4 in parallel), or **away from it** (→ I build Phase 4 now; you do Phase 1 later)?

Everything meaningful flows from Phase 1 — until a demo replays as a clean pick in the live sim, Phases 2/3/5 can't be trusted. So the fastest route to "Day 2 done" is: **Phase 1 as soon as you're at the machine**, Phase 4 built alongside.
