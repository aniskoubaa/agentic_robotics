<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# How to Build the Day 2 Labs — a Simple Guide

> **Read this first, build slides last.** This file explains *what to make*, *in
> what order*, and *how each piece connects to Day 1*. When the labs work, the
> slides almost write themselves.

---

## 1. The one-sentence story

**Day 1: the robot *thinks* (an LLM calls tools).
Day 2: the robot *acts* (a VLA model moves the arm).**

| | Day 1 | Day 2 |
|---|---|---|
| Brain | LLM (text in → tool call out) | **VLA** (image + instruction in → **motor actions** out) |
| How a skill happens | call a pre-written ROS 2 service (`nav_to_row`, `inspect_plant`…) | the model **predicts the joint movements itself** |
| Where skills come from | a human wrote them | **a human demonstrated them** (Lab 2.1) and the model copied them (Lab 2.2) |
| Output | a sentence / a JSON report | a moving UR5e arm |

So Day 2 has two halves, and they chain just like Day 1's two labs did:

```
Lab 2.1  TEACH        Lab 2.2  EXECUTE
human teleops arm ──► dataset ──► VLA model ──► arm moves on its own
(collect demos)                   (run / fine-tune)
```

This is the **exact same shape** as Day 1, where `10_orchestrate.py` (Lab 1.1)
became the autonomous `agent.py` (Lab 1.2). Reuse that teaching rhythm.

---

## 2. How Day 2 reuses Day 1 (don't reinvent)

You already built these on Day 1 — **lean on them**:

- **The `.env` loader + API-key pattern** from `day1_02/starter/agent.py` →
  reuse verbatim for the VLA endpoint key in Lab 2.2.
- **The "wrap a ROS 2 capability as a clean Python function" idea** from
  `day1_01/starter/tools.py` → Lab 2.1's `capture_wrist()` and Lab 2.2's
  `arm.send(action)` are the same idea, one level lower (joints, not services).
- **The numbered-progression teaching style** from Lab 1.1 (`01_…py`, `02_…py`)
  → use it again so each Day 2 starter is runnable and testable on its own.
- **The "structured artifact, not prose" lesson** from Lab 1.2 (the agent emits
  a JSON report) → Lab 2.1 emits a **LeRobot dataset**; same principle, the
  deliverable is structured data a machine can consume.

> When you write the lab READMEs, add a **"How this builds on Day 1"** table —
> copy the one in `day1_02_agentic_inspector/README.md`. Students love the thread.

---

## 3. Build order (do it top to bottom)

1. **`api_clients/vla_client/`** — the remote-VLA helper. *Build this first*: both
   labs (and the demos) import it. Until it exists, nothing else runs.
2. **Lab 2.1 starters** — teleop → record → upload (the "TEACH" half).
3. **Lab 2.2 starters** — the VLA executor (the "EXECUTE" half), which *uses*
   `vla_client`.
4. **Demos** — fill in `sim/demos/d2l1_teleop_record.sh` and `d2l2_vla_rollout.sh`
   (today they're scaffolds). A demo = the happy-path of the lab, one command.
5. **Grading scaffolds** — `solution/`, `evaluator/`, `tests/`, `instructor.md`.
6. **Slides last** — `lectures/day2_l1_vlm_to_vla/` + `day2_l2_vla_api_imitation/`.

---

## 4. Lab 2.1 — Teleoperation & Data Collection (`day2_01_…`)

**Goal:** drive the UR5e by hand, record 30 pick-&-place demos in **LeRobot
format**, upload them.

**Build these starter scripts** (numbered, like Day 1 — beginner comments inline):

| # | File | What it teaches | Links to D1 |
|---|------|-----------------|-------------|
| 01 | `starter/01_teleop.py` | publish to the arm from keyboard/joystick (`/cmd_vel` for base, joint cmds for arm) | same publisher pattern as `day1_01/01_drive_forward.py` |
| 02 | `starter/02_read_streams.py` | subscribe to `/joint_states`, `/wrist_camera`, EE pose — print them at 30 Hz | same subscriber pattern as `day1_01/02_read_lidar.py` + `03_read_camera.py` |
| 03 | `starter/03_record.py` | sync those streams into one **LeRobot v2.0 episode** (images 224×224, action↔state aligned, no NaNs) | new — this is the heart of the lab |
| 04 | `starter/04_upload.py` | `POST /datasets/upload` to the shared GPU server, indexed by `team_id` | reuse the HTTP-client style from `vla_client` |

**Deliverable:** 30 episodes (5–15 s each) on the shared server.

**Key teaching point:** a VLA can only learn what you demonstrate — *garbage
demos in, garbage policy out*. That's why grading rewards **diversity** of
starting poses (20%) and **validation** (no NaNs, aligned, 224×224 — 30%).

**To author later:** `evaluator/validate_dataset.py`, `tests/`, `instructor.md`
(gripper-stuck recovery, save-frequency tuning).

---

## 5. Lab 2.2 — VLA Executor (`day2_02_…`)

**Goal:** implement `vla_execute(instruction) -> ExecutionResult` — capture the
wrist image, ask the VLA what to do, stream the actions to the arm.

**The loop (already sketched in the lab's README):**

```
img = capture_wrist()                 # ← reuse capture from Lab 2.1 / day1 camera read
actions = vla_client.act(img, text)   # ← the remote VLA call (api_clients/vla_client)
for a in actions: arm.send(a); sleep(dt)
success if gripper closed AND object lifted
```

**Build these starters:**

| File | What it does |
|------|--------------|
| `starter/vla_executor.py` | the loop above — the core deliverable |
| `starter/finetune_smolvla.py` | **advanced track only** — fine-tune SmolVLA on this morning's dataset, then benchmark zero-shot vs fine-tuned |

**Links to Day 1:** this is Lab 1.2's agent loop, but the "tool" is now a single
**VLA call** that returns low-level actions instead of an LLM choosing a named
service. Same "loop until success or step-limit" skeleton — point students at it.

**Track split (same beginner/advanced idea as Day 1):**
- *Beginner:* zero-shot against the hosted OpenVLA endpoint.
- *Advanced:* fine-tune SmolVLA on the Lab-2.1 dataset, compare.

**To author later:** `evaluator/scenarios.yaml` (5 instructions × 3 poses),
`tests/`, `instructor.md`.

---

## 6. The shared piece: `api_clients/vla_client/`

A tiny library both labs import. Keep it dead simple:

```python
# api_clients/vla_client/client.py  (sketch)
class VLAClient:
    def __init__(self, base_url, api_key): ...
    def act(self, image, instruction) -> list[Action]:
        "POST image+instruction to the hosted VLA, return an action chunk."
```

- Reads the endpoint URL + key from `.env` (**reuse the loader from
  `day1_02/starter/agent.py`** — don't write a new one).
- Must respect the **≤500 ms per action chunk** latency budget (Lab 2.2 grades it).
- This is what makes `d2l2_vla_rollout.sh` and `vla_executor.py` runnable.

---

## 7. Wiring + build checklist (do every time you add a script)

1. **Add the lab paths are already in `setup.py`** (`day2/day2_01_…`,
   `day2/day2_02_…`) — new `starter/*.py` files are picked up automatically by
   `all_starter_scripts()`. ✅ no edit needed unless you add a *new* lab folder.
2. **Give each new starter a bashrc alias** `XX_d3 / XX_d4` style — Day 2 labs use
   the **`_d3` (Lab 2.1)** and **`_d4` (Lab 2.2)** suffix (d1=D1L1, d2=D1L2,
   d3=D2L1, d4=D2L2). Add them to the RAISE 2026 alias block in `~/.bashrc`.
3. **Rebuild** so `ros2 run` sees the new scripts:
   ```bash
   cd ~/Dev_WS/raise_summer_school/RAISE2026/raise_ros2_ws
   # strip anaconda from PATH first (see conventions below)
   colcon build --symlink-install -DPython3_EXECUTABLE=/usr/bin/python3
   source install/setup.bash
   ```

---

## 8. Conventions you must not forget

- **Author tag** on every new Python/shell/XML/YAML file:
  `Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>`.
- **Starter scripts get beginner comments inline** (explain the ROS 2 / VLA
  concept) — this overrides the global "no comments" rule. It's teaching material.
- **numpy:** if any pip step touches `ultralytics`/`torch`/`opencv`, pin
  `numpy<2` **in the same `pip install` command** (never after) + use CPU torch.
- **colcon:** strip anaconda from `PATH` and pass
  `-DPython3_EXECUTABLE=/usr/bin/python3`, or rclpy fails to import.
- **Demos** = the lab's happy path in one command; keep them in `sim/demos/`.

---

## 9. Suggested first move

Build in this order and you're never blocked:

```
1. api_clients/vla_client/      ← unblocks everything
2. day2_01 starters 01→04       ← TEACH (collect data)
3. day2_02 starter vla_executor ← EXECUTE (uses vla_client)
4. the two d2 demo scripts
5. grading scaffolds
6. slides
```

When the executor moves the arm from a spoken instruction, **Day 2 is done** —
then make the slides to explain what the students just saw working.
