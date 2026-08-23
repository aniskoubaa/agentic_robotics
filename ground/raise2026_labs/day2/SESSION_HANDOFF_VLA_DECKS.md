<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Session handoff — Day 2 VLA decks & design (→ next session: design the labs)

**Branch:** `day2-vla-comparison` (7 commits, **nothing pushed yet**). Off `main`.
**Date:** session of 2026-06-18.
**Next session goal:** design the Day-2 **labs** (2.1 record, 2.2 executor) — see "What's next".

---

## 1. What we decided (the locked design)

- **Model choice:** **SmolVLA** is the backbone of Day 2 (450M, LeRobot-native,
  fine-tunes on one 4090, ~5% behind OpenVLA at 16× fewer params). Details +
  verified facts in [`VLA_MODEL_CHOICE.md`](./VLA_MODEL_CHOICE.md).
- **Backends (configurable):** one `VLAClient` interface, two backends, a
  **canonical action contract** anchored on the **UR5e command space**:
  - **Default = `local-smolvla`** (in-process; works offline, no server).
  - **Opt-in = `remote-openvla`** (HTTP) — **self-hosted** on the 4090 box (LAN)
    or cloud (Modal/RunPod). **There is no public OpenVLA API.**
  - Switch via `VLA_BACKEND` / `VLA_REMOTE_URL`. See
    [`LOCAL_VS_REMOTE_VLA.md`](./LOCAL_VS_REMOTE_VLA.md).
- **Training paradigm:** **imitation learning (behavioral cloning)** on teleop
  demonstrations — **not RL**. (This was an explicit clarification the user raised.)
- **⚠️ Gotcha:** the *interface* unifies trivially but *action spaces differ*
  (SmolVLA joint actions vs OpenVLA EE-deltas). Anchor everything on UR5e command
  space; serve **fine-tuned** checkpoints, keep raw zero-shot only as a "why it
  flails" demo. Beginner path must just work.

## 2. The Day-2 scenario / objective (in plain terms)

> **"Teach the robot by showing it, then command it by speaking."**
> Morning (Lab 2.1): teleop the UR5e, record ~30–50 tomato-pick demos in LeRobot
> format. Afternoon (Lab 2.2): a VLA trained on those demos executes a typed
> instruction ("pick the ripe red tomato") — **no motion code written**.
> One thread: *what you record in the morning is the brain you command in the afternoon.*

**Success criteria:** student can (1) explain VLA vs Day-1 LLM, (2) collect good
demos, (3) run a VLA end-to-end from a typed instruction, (4) measure success +
latency and see *data quality > model size*, (5 advanced) compare local vs self-hosted.

**⚠️ OPEN DECISION (not locked):** the exact graded task. I proposed options and
the user did not yet pick:
- **A. Tabletop tomato pick-and-place (UR5e only)** — simplest, recommended core.
- **B. Mobile manipulation (Husky drives + UR5e picks)** — impressive, riskier.
- **C. Ripeness sorting (pick ripe, leave unripe)** — nice language twist, more data.
**Lock this first thing next session** — it sets the data, difficulty, failure modes.

## 3. What we built this session (all committed on the branch)

**Lectures — `RAISE2026/lectures/day2_l1_vlm_to_vla/`:**
- `RAISE2026_LectureD2L1_VLM_to_VLA.tex` (**22 frames**, builds clean) +
  `build_d2l1.sh`. The main D2L1 deck. Systematic build:
  *motivation → recap VLM → VLM-vs-VLA → 4 ingredients (SENSE / STATE / ACTION /
  INSTRUCTION) → TIME (frames/episodes) → the DATASET → teleop → on-disk example
  → record code → history → actions-as-tokens → imitation steps → imitation-not-RL
  → train code → whole-loop example → takeaways.*
- `RAISE2026_D2_Comparative_VLA_Models.tex` (**23 frames**, builds clean) +
  `build_comparative_vla.sh`. Per-model concept **and** minimal-code slides for
  OpenVLA / π0 / SmolVLA / GR00T / ACT-DP; comparison tables; "fits a 4090?";
  SmolVLA recommendation; install/train/use hands-on.

**Design docs — `RAISE2026/raise_ros2_ws/src/raise2026_labs/day2/`:**
- [`HOW_TO_BUILD_DAY2_LABS.md`](./HOW_TO_BUILD_DAY2_LABS.md) — build order + Day-1 linkage.
- [`VLA_MODEL_CHOICE.md`](./VLA_MODEL_CHOICE.md) — model decision + verified facts/sources.
- [`LOCAL_VS_REMOTE_VLA.md`](./LOCAL_VS_REMOTE_VLA.md) — backend architecture + config contract.

**All deck code verified against live upstream docs** (LeRobot v3.0
`LeRobotDataset.create/add_frame/save_episode/finalize` + DataLoader; SmolVLA
model card; openpi README `pi05_droid`+`gs://`; Isaac-GR00T `gr00t.policy` +
nested obs + `get_action` tuple; OpenVLA `predict_action`).

## 4. What's next — design the labs (the new session)

Labs are **README-only** today at
`RAISE2026/raise_ros2_ws/src/raise2026_labs/day2/{day2_01_teleoperation_and_data, day2_02_vla_executor}/`.

**Recommended build order (from `HOW_TO_BUILD_DAY2_LABS.md`):**
1. `api_clients/vla_client/` — `base.py` (VLAClient + canonical `Action`),
   `local_smolvla.py`, `remote_vla.py`, `factory.py`, `server.py`. Build first;
   pins the action contract everything conforms to. **Default local.**
2. Lab 2.1 starters: `01_teleop → 02_read_streams → 03_record → 04_upload`
   (record LeRobot v2.0/v3.0 from `/joint_states`, `/wrist_camera`, `/cmd_vel`).
3. Instructor pre-work: record a reference dataset → fine-tune SmolVLA → ship the
   checkpoint (= the default local brain) so labs "just work".
4. Lab 2.2 starter: `vla_executor.py` with `--backend` flag + latency log.
5. Demos: `sim/demos/d2l1_teleop_record.sh`, `d2l2_vla_rollout.sh` (scaffolds today).
6. Grading: `evaluator/`, `tests/`, `instructor.md` for both labs.
7. Slides last (the rest of D2L1 architecture section, D2L2 deck).

## 5. Conventions the new session MUST keep

- **Author tag** on every new Python/shell/XML/YAML/HTML:
  `Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>`.
- **Lab starter scripts get inline beginner comments** (overrides global no-comments).
- Each new starter gets an `XX_dN` **bashrc alias**; Day-2 uses `_d3` (Lab 2.1) /
  `_d4` (Lab 2.2). Add starters to `raise2026_labs/setup.py` LABS (paths already
  listed) → `colcon build`.
- **numpy:** pin `numpy<2` in the **same** pip resolve (never after) + CPU/GPU torch.
- **colcon:** strip anaconda from PATH + `-DPython3_EXECUTABLE=/usr/bin/python3`.
- **Lecture decks:** institute line `ISGIS – Sfax`; track `.tex/.pdf/build.sh`
  (LaTeX aux is gitignored); `pdflatex` ×2 via `build.sh`.
- **Git push** hangs on credential prompt in-tool → push via
  `git push "https://x-access-token:${GITHUB_TOKEN}@github.com/aniskoubaa/raise_summer_school" day2-vla-comparison`.

## 6. Branch commits (this session)

```
795efd2 D2L1 deck: systematic "four ingredients -> dataset" build (reorder + 4 slides)
a0b3c8b D2L1 deck: add beginner background + step-by-step scaffolding
2019f1f D2L1 deck: From VLM to VLA (opening section)
f218a12 Day 2: design doc for configurable local/remote VLA backends
1928698 Day 2 deck: add early "how to build a VLA" 5-step recipe slide
f35de41 Day 2: VLA model brainstorm + comparative lecture deck
```
(+ this handoff doc, uncommitted.) **Not pushed.** Decide push/PR vs keep local.

**Good first prompt next session:** *"Lock the Day-2 lab task (option A/B/C), then
build `api_clients/vla_client/` with the local-default backend per
LOCAL_VS_REMOTE_VLA.md."*
