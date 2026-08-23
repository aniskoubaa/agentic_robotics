<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Day 2 — Lab Design: Teach a VLA by Demonstration, then Command it by Speaking

> **The Day-2 thread:** *what you record in the morning is the brain you command
> in the afternoon.* Students teleoperate the UR5e to collect demonstrations,
> **fine-tune SmolVLA** on those demonstrations, then drive the arm from a typed
> instruction — **without writing any motion code.**
>
> **Model:** SmolVLA (450M, LeRobot-native) — see [`VLA_MODEL_CHOICE.md`](./VLA_MODEL_CHOICE.md).
> **Backends:** local-default, self-hosted-remote opt-in — see [`LOCAL_VS_REMOTE_VLA.md`](./LOCAL_VS_REMOTE_VLA.md).
> **Build order & Day-1 linkage:** [`HOW_TO_BUILD_DAY2_LABS.md`](./HOW_TO_BUILD_DAY2_LABS.md).

This document is the **lab-level design**: the concrete task variants, the
training/fine-tuning flow with realistic timings, the stage-by-stage lab
structure, the action contract, and grading. It turns the locked decisions into
something buildable.

---

## 0. What changed after the design review (read this first)

Two decisions were locked this session:

1. **Ship all three task variants, in isolation.** Instead of picking one graded
   task, the lab carries **three swappable "task packs"** — **A** (tabletop pick),
   **C** (ripeness sorting), **B** (mobile manipulation). The lab code (record /
   fine-tune / execute) is **task-agnostic**; you select the task with one flag
   (`--task A|C|B`). The instructor chooses which pack(s) to run on the day; an
   ambitious cohort can run more than one.

2. **Fine-tuning runs in the background, not as a sit-and-wait block.** Fine-tuning
   SmolVLA to a usable policy is **hours, not minutes** (§3). So the live, graded
   labs are **2.1 (record)** and **2.2 (execute + benchmark)**; fine-tuning is a
   **bridge step you *launch* and walk away from** — instructor-precomputed for
   the default path, fire-and-forget for the advanced track. This is why there is
   **no standalone "sit and watch it train" lab** — the timing forbids it (§3.3).

---

## 1. Ground truth — the real command interface (verified against the sim)

Everything the VLA emits must land on the **actual** topics the Day-1 sim
exposes. Confirmed from `raise2026_bringup/config/ros_gz_bridge.yaml` and the
Day-1 tool servers:

| Channel | ROS 2 topic | Type | Notes |
|---|---|---|---|
| **Arm command** | `/ur5e_shoulder_pan_joint/cmd` … `/ur5e_wrist_3_joint/cmd` (×6) | `std_msgs/Float64` | **position, radians**, one publisher per joint |
| **Gripper command** | driving knuckle `gripper_robotiq_85_left_knuckle_joint` (+ mimic signs) | `std_msgs/Float64` | `0.0` = open → `~0.5` = closed (see `gripper_server.py`) |
| **Robot state** | `/joint_states` | `sensor_msgs/JointState` | all arm + gripper joints, the VLA's `state` input |
| **Vision** | `/wrist_camera/image_raw` | `sensor_msgs/Image` | the VLA's `image` input; resize to **224×224** for the dataset |
| **Depth (optional)** | `/wrist_camera/depth/image_raw`, `/wrist_camera/points` | `Image`, `PointCloud2` | not fed to SmolVLA; available for evaluation/ground-truth |
| **Base drive** (Task B only) | `/cmd_vel` | `geometry_msgs/Twist` | Husky drive, reused from Day 1 |

**This fixes the canonical action contract** — no guessing:

```python
# api_clients/vla_client/base.py
Action = {
    "joints":  [6 floats],   # radians, ORDER: shoulder_pan, shoulder_lift,
                             #          elbow, wrist_1, wrist_2, wrist_3
    "gripper": float,        # 0.0 open … 0.5 closed (driving knuckle)
}
# Task B additionally: "base": {"linear_x": float, "angular_z": float}
```

`arm.send(action)` is just six `Float64` publishes + one gripper publish — the
**same publisher pattern** students wrote in Day-1 `01_drive_forward.py`, one
level lower (joints instead of `/cmd_vel`).

---

## 2. The Day-2 arc (where training fits)

```
   Lab 2.1  TEACH (live, 105 min)        BRIDGE  TRAIN (background, hours)        Lab 2.2  EXECUTE (live, 90 min)
   ───────────────────────────────       ──────────────────────────────         ──────────────────────────────
   teleop UR5e  ──►  LeRobot dataset  ──► lerobot-train (SmolVLA)  ──► checkpoint ──► vla_client.act(img, instr)
   (30 demos / team)  (224×224, joints)   (instructor-precomputed                     ──► action chunk ──► UR5e
                                           default; advanced = your own)              measure success + latency
```

The three columns are exactly Day-1's rhythm (`10_orchestrate.py` → `agent.py`),
one level lower: the "skill" is no longer a hand-written ROS 2 service, it is a
**policy learned from the demonstrations students recorded that morning.**

---

## 3. Fine-tuning timing — and why it decides the lab structure

This is the question that determines whether training is its own lab. The honest
numbers:

### 3.1 What the references say
- SmolVLA-base is **450M params**, fine-tunes in **~12 GB VRAM** — comfortably
  inside the 24 GB 4090; community reports training it on a 12 GB 3080Ti.
- A **full** fine-tune is **~20k steps**, cited at **~4 h on an A100**
  ([`VLA_MODEL_CHOICE.md`](./VLA_MODEL_CHOICE.md) §4). A single 4090 is slower
  per step than an A100, so **budget more than 4 h for 20k steps** on our box.
- A **visible-result** fine-tune on ~30 demos needs far fewer steps
  (≈ **3k–6k**), but that is still **tens of minutes to ~1–2 h** on one 4090 —
  *not* something 20 students each sit and watch inside a 90-minute block.

> ✅ **MEASURED (RTX 4090 laptop 16 GB):** ~1.1 s/step, 10.8 GB VRAM @ batch 64.
> Reference recipe: **≈50 scan episodes + 6000 steps ≈ 2 h** → **100/100** on
> the Lab-2.2 evaluator in real greenhouse scenes (8/8 correct-color picks
> L+R, 0 wrong grabs, max 201 ms/action). Journey worth teaching: bare-ground
> 3k/30eps hit 100 trivially; adding greenhouse context dropped to 40 — the
> fix wasn't more data/steps but OBSERVABILITY (a scan choreography so the
> target is visible in the frames). See HOW_IT_WORKS §"the observability
> lesson".

### 3.2 The consequence
You cannot fit "fine-tune to convergence" inside a live lab session. Trying to
makes the room sit idle watching a loss curve — bad pedagogy and a scheduling
risk. So:

### 3.3 Structural decision (answers "two labs or three?")
**Two live labs + a background bridge.** No standalone sit-and-wait training lab.

| Path | Who | When training happens | What students do live |
|---|---|---|---|
| **Default (graded)** | everyone | **instructor pre-fine-tunes** the reference checkpoint **before the school** | call the working checkpoint in Lab 2.2 → it works → the "wow" fires |
| **Advanced** | opt-in teams | **launch a fire-and-forget job** at the end of Lab 2.1 on the shared GPU; it trains over the lunch/2.1→2.2 gap (or overnight) | in Lab 2.2 they **A/B their own checkpoint vs the reference** over the same scenarios |
| **Safety net** | any team whose VLA won't converge | ACT/Diffusion Policy trains in **minutes** on the same dataset | still get a moving arm to demo |

The fine-tuning **step itself** is one well-documented script
(`finetune_smolvla.py`) plus a launch helper that submits the job to the shared
GPU queue. It is taught and *understood* in Lab 2.2's reading, but it **runs in
the background** — never blocking the room.

---

## 4. The three task packs (designed in isolation)

A **task pack** is a self-contained scenario: its world objects, its instruction
set, its demo recipe, its dataset `repo_id`, and its evaluation scenarios. The
record/train/execute code never changes — you pass `--task A|C|B`. This keeps the
three tasks **truly isolated**: choosing one (or running several) is a flag, not
a code fork.

```
day2/task_packs/
  task_A_tabletop_pick/    world + instructions + scenarios.yaml + dataset card
  task_C_ripeness_sort/    "
  task_B_mobile_manip/     "  (+ base-drive demo recipe)
common/task_pack.py        loads a pack → (world, instructions[], scenarios[], repo_id)
```

| | **A — Tabletop pick** | **C — Ripeness sort** | **B — Mobile manipulation** |
|---|---|---|---|
| **One-liner** | pick the tomato, place in the bin | pick the **ripe (red)** one, leave green | Husky **drives** to the plant, then UR5e picks |
| **Embodiment** | UR5e + gripper | UR5e + gripper | **Husky + UR5e + gripper** |
| **Action space** | `joints[6] + gripper` | `joints[6] + gripper` | `joints[6] + gripper + base{vx, wz}` |
| **Instruction variety** | low ("pick the tomato") | **high** ("pick the ripe tomato", "leave the green ones") — the real language twist | medium ("go to the plant and pick a tomato") |
| **Demos / team** | **~30** | **~50** (needs ripe+unripe coverage) | ~40 (drive + pick are two phases) |
| **Why it's distinct** | cleanest signal, simplest grasp — the **reliable core** | tests **language grounding**: same scene, instruction selects the target | tests **whole-robot** policy + the base action head — most impressive |
| **Main failure mode** | grasp misses / object rolls | picks the wrong-color tomato | drives past / mis-aligns before the pick |
| **Risk in a 90-min slot** | **low** ✅ | medium | **high** ⚠️ |
| **Recommended use** | always run this first | run if the cohort is strong on 2.1 | demo/stretch; pre-validate heavily |

**Design rule for isolation:** A is a strict subset of C (same arm/gripper, C
just adds color-conditioned target selection and more demos), and B is A plus a
base-drive phase. So building **A first** gives you C for the cost of more data
+ richer instructions, and B for the cost of the base action head + a drive
phase in the demo recipe. Build A → C → B in that dependency order.

---

## 5. Lab 2.1 — Teleoperation & Data Collection (live, 105 min)

**Goal:** teleop the selected task, record `N` clean LeRobot episodes, upload them.

**Starters (numbered, beginner-commented — the Day-1 style):**

| # | File | Teaches | Day-1 link |
|---|------|---------|-----------|
| 01 | `starter/01_teleop.py` | publish joint (+ base for B) commands from keyboard/joystick | `day1_01/01_drive_forward.py` publisher pattern |
| 02 | `starter/02_read_streams.py` | subscribe `/joint_states` + `/wrist_camera/image_raw`, print at 30 Hz | `day1_01/02_read_lidar.py` + `03_read_camera.py` |
| 03 | `starter/03_record.py` | sync streams → one **LeRobot episode** (224×224, action↔state aligned, no NaNs) — the heart of the lab | new |
| 04 | `starter/04_upload.py` | `POST /datasets/upload` to the shared GPU server, indexed by `team_id` + `task` | HTTP-client style from `vla_client` |

**Deliverable:** `N` episodes (5–15 s each) for the chosen `--task`, on the
shared server under `${team_id}/raise_<task>`.

**Key teaching point — "garbage demos in, garbage policy out."** A VLA only
learns what you show it. Grading rewards **diversity** of starting poses and
**validation**, not raw count (see §8).

**To author later:** `evaluator/validate_dataset.py`, `tests/`, `instructor.md`
(gripper-stuck recovery, save-frequency tuning).

---

## 6. The bridge — fine-tune SmolVLA (background, hours)

This is the **training** the user asked about. It is one script + a launch
helper; it runs off the critical path (§3.3).

**`day2_02_vla_executor/starter/finetune_smolvla.py`** (advanced track + the
instructor's pre-school run) wraps:

```bash
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=${HF_USER}/raise_<task> \
  --batch_size=64 --steps=${STEPS} \
  --policy.device=cuda \
  --output_dir=outputs/smolvla_<task>_${team_id}
# STEPS: 3000 (quick visible result) … 20000 (full). MEASURE the wall-clock
# pre-school on the real 4090 and pin it into instructor.md (see §3.1 warning).
```

- A thin **launch helper** submits this to the shared-GPU job queue and returns
  immediately, so Lab 2.1 can end on "your training job is queued" rather than a
  spinner. The resulting checkpoint becomes that team's `VLA_LOCAL_CKPT`.
- **Default path skips this live**: the instructor's pre-fine-tuned checkpoint is
  already the served/local brain, so beginners go straight to Lab 2.2.
- **Conventions:** install `"lerobot[smolvla]"` with **`numpy<2` in the same pip
  resolve** + GPU torch; the job runs with anaconda stripped from `PATH`.

---

## 7. Lab 2.2 — VLA Executor (live, 90 min)

**Goal:** implement the backend-blind execution loop and run it from a typed
instruction; advanced teams benchmark their own checkpoint vs the reference.

```python
# starter/vla_executor.py  — the core deliverable
def vla_execute(instruction: str, max_steps: int = 50) -> ExecutionResult:
    client = make_vla_client(os.getenv("VLA_BACKEND", "local-smolvla"))  # default local
    for _ in range(max_steps):
        img    = capture_wrist()                       # reuse Lab 2.1 / Day-1 camera read
        state  = read_joint_states()
        action = client.act(img, instruction, state)   # canonical Action (§1)
        arm.send(action)                               # 6 Float64 + gripper
        if grasped_and_lifted(): return ExecutionResult.SUCCESS
    return ExecutionResult.TIMEOUT
```

- `--backend local-smolvla|remote-openvla` + a **one-line latency log** turns this
  into an optional local-vs-self-hosted benchmark (see [`LOCAL_VS_REMOTE_VLA.md`](./LOCAL_VS_REMOTE_VLA.md)).
- **Same skeleton as Day-1 `agent.py`**: loop until success or step/time limit.
  The "tool" is now a single VLA call returning low-level actions instead of an
  LLM choosing a named service — point students at that parallel explicitly.

**Track split**
- **Beginner:** call the instructor's pre-fine-tuned checkpoint (local default). It works.
- **Advanced:** point `VLA_LOCAL_CKPT` at *their* fine-tune from §6; A/B vs the
  reference over the same `scenarios.yaml`; report success + latency.

**To author later:** per-task `evaluator/scenarios.yaml`, `tests/`, `instructor.md`.

---

## 8. Grading (per task pack)

**Lab 2.1 — data quality (the lesson is *data > model size*):**
- `N` episodes uploaded for the chosen task (50%)
- Validation: no NaNs, action↔state alignment, image 224×224 (30%)
- Diversity score over starting poses — and over **ripe/unripe coverage for C** (20%)

**Lab 2.2 — execution:**
- Pick-and-place success over **5 scripted instructions** (60%) — for C, scored
  on picking the **correct-color** target
- Latency budget met (**≤500 ms per action chunk**) (20%)
- Recovery behavior on a failed grasp (20%)

Each task pack ships its own `scenarios.yaml` (5 instructions × 3 starting
poses); the evaluator is task-agnostic and reads the pack.

---

## 9. The shared piece: `api_clients/vla_client/` (build first)

Both labs + the demos import it; build it before anything else (per
[`HOW_TO_BUILD_DAY2_LABS.md`](./HOW_TO_BUILD_DAY2_LABS.md) §3).

| File | Role |
|---|---|
| `base.py` | `VLAClient` protocol + canonical `Action` (§1) |
| `local_smolvla.py` | in-process `SmolVLAPolicy.select_action` → `to_canonical` |
| `remote_vla.py` | HTTP backend → `POST url/act` → `to_canonical` |
| `factory.py` | `make_vla_client(backend)` — default `local-smolvla` |
| `server.py` | thin FastAPI hosting the checkpoint behind `POST /act` (returns canonical) |

**Config contract** (defaults make everything run fully local, no server):

| Setting | Default | Purpose |
|---|---|---|
| `VLA_BACKEND` | `local-smolvla` | `local-smolvla` \| `remote-openvla` |
| `VLA_LOCAL_CKPT` | `lerobot/smolvla_base` (instructor swaps to the fine-tuned ckpt) | local/served checkpoint |
| `VLA_REMOTE_URL` | *(unset)* | required only for `remote-*` |
| `VLA_DEVICE` | `cuda` else `cpu` | local inference device |

Reuse the **`.env` loader from `day1_02/starter/agent.py`** — do not write a new one.

---

## 10. Build & wiring checklist

1. **`api_clients/vla_client/`** → unblocks everything (default local).
2. **`day2/task_packs/{A,C,B}`** + `common/task_pack.py` → the `--task` switch.
3. **Lab 2.1 starters** `01→04` → record the chosen task.
4. **`finetune_smolvla.py`** + launch helper → background training (§6).
5. **Lab 2.2** `vla_executor.py` (`--backend`, `--task`, latency log).
6. **Demos** `sim/demos/d2l1_teleop_record.sh`, `d2l2_vla_rollout.sh` (one-command happy paths).
7. **Grading scaffolds** per pack; **slides last**.

**Conventions (do not forget):**
- **Author tag** on every new `.py/.sh/.xml/.yaml`: `Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>`.
- **Starter scripts get inline beginner comments** (overrides the global no-comments rule).
- Each new starter gets an `XX_dN` **bashrc alias** — Day-2 uses **`_d3`** (Lab 2.1) / **`_d4`** (Lab 2.2).
- **numpy:** pin `numpy<2` **in the same** pip resolve as torch/lerobot/opencv (never after).
- **colcon:** strip anaconda from `PATH` + `-DPython3_EXECUTABLE=/usr/bin/python3`, then `colcon build --symlink-install`.

---

## 11. Open items for the next build session

1. **Measure real 4090 fine-tune wall-clock** for `--steps 3000/6000/20000` and
   pin it into §3.1 + `instructor.md` (replaces the estimates).
2. Decide `N` per task once the demo recipe is timed (30 / 50 / 40 are planning numbers).
3. Pre-record the **reference dataset** + ship the **pre-fine-tuned checkpoint**
   so the default path "just works" on day one (§3.3).
4. Confirm the gripper "grasped & lifted" success check against the sim's contact
   sensing (or fall back to wrist-camera + joint-effort heuristic).
