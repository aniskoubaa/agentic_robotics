<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# How It Works — Data Collection & Training, Explained

> **This document explains the two halves of teaching our robot:** how the
> demonstration data was **collected** (the auto-demonstrator pipeline) and how
> the model is **trained** on it (behavioral cloning with SmolVLA). Every number
> in here is **measured from the real runs on this machine** — not estimated.
>
> Companion docs: the dataset itself ([`DATASET.md`](./DATASET.md) + the
> [visual README](../../../../datasets/raiseschool__raise_ripeness_sort_ref/README.md)),
> the step-by-step runbook ([`HOW_TO_TRAIN_AND_USE.md`](./HOW_TO_TRAIN_AND_USE.md)),
> and the student slides (`lectures/day2_l2_vla_api_imitation/`).

---

## The one-picture summary

```
 DATA COLLECTION (~28 min, hands-free)         TRAINING (~2 h on this GPU)
 ────────────────────────────────────          ─────────────────────────────────
 sim + grasp_server + auto-demonstrator        lerobot-train (SmolVLA, 450M)
                                        
 49 × [spawn 🍅🍅 → SCAN → pick red]      ──►   4228 frames of (image, words,
 recorded at 10 Hz through the wrist            state → action) → the model
 camera into LeRobot format                     learns to copy the mapping
                                        
        RAISE2026/datasets/…            ──►    ~/raise_checkpoints/smolvla_C_ref
```

---

# Part 1 — How the data was collected

## 1.1 What we're trying to capture

Imitation learning needs examples of *"in this situation, do this."* For a
robot arm, one example (one **frame**) is:

| What | Concretely | Captured from |
|---|---|---|
| the situation | wrist camera image (224×224) + 7 joint positions | `/wrist_camera/image_raw`, `/joint_states` |
| the words | "pick the red tomato" | the task instruction |
| what to do | the **next commanded** joint targets (7 values) | the `/…/cmd` topics |

We capture this tuple **10 times per second** while a pick is being performed.
One scan-and-pick = 76–97 frames. Forty-nine of them = **4228 training examples**.

## 1.2 Who performs the demonstrations? A script — and that's fine

Driving six joints with a keyboard is exhausting and produces jerky,
inconsistent motion. So instead, `05_auto_demonstrate.py` acts as a **scripted
expert**: it performs the same clean pick choreography every episode —

```
approach → descend → close (grasp!) → lift → swing to the bin → release
```

— while the recorder captures what the *camera* sees and what the *joints* do.
Crucially the script **scans first**: it hovers above the LEFT spot and only
descends if the tomato it SEES there is red — otherwise it pans right. The
model never sees the script, only its **footprints in the data**. Imitation
learning copies whoever demonstrates; a script demonstrates perfectly,
50 times in ~28 minutes, with zero human effort.

## 1.3 The three tricks that made it possible

The stock simulator could not pick at all — these three additions fixed that
(details + how to tune them: [`VERIFY_MANIPULATION.md`](./VERIFY_MANIPULATION.md)):

1. **Grasping by decree** (`grasp_server`): Gazebo's physics can't reliably
   hold an object by friction. Instead, when the gripper is *commanded* closed
   within 15 cm of a tomato, the tomato attaches to the tool; open = release.
   Deterministic — and it triggers off the **command**, because with a tomato
   between the fingers the knuckle joint physically *stalls* and the measured
   angle never reaches "closed" (we found this live).
2. **The no-IK trick**: there is no inverse kinematics. So we invert the
   problem — move the arm to a chosen pose, read *where the tool ended up*,
   and **spawn the tomato exactly there**. Reachable by construction.
3. **Gravity-off tomatoes**: spawned fruits float where placed — no table
   needed, carriable, zero physics tuning.

## 1.4 Variation is engineered in — that's the language lesson

Every episode spawns **one red and one green tomato**, and the **red one swaps
sides each episode** (25 left / 25 right, perfectly balanced). The demo always
picks the red one — after LOOKING. Consequence for the learner:

> A model that memorizes one motion is right ~50% of the time.
> To do better it **must** read the instruction *and* look at the image to
> decide which way to reach. That is language grounding, forced by data design.

## 1.5 Quality control at record time

- After the gripper closes, the recorder checks `/grasp/state` actually names
  the **red** tomato — a failed or wrong grasp means the take is **discarded**,
  never recorded. (A demo of failure teaches failure.)
- Frames with NaNs or missing streams are dropped.
- The final check is **open-loop replay** (`06_replay_episode.py --spawn`): play
  the recorded actions back into the sim and watch. Our episode 0 replays into
  a complete pick — grasp within ~3–5 cm of the tool, carry, release. If replay
  looks good, the data *provably* contains the skill.

**Measured output:** 49 episodes → 4228 frames → 607 MB of raw pixels →
**35.2 MB** on disk (parquet, ≈17× compression) → committed inside the repo at
`RAISE2026/datasets/raiseschool__raise_ripeness_sort_ref/`.

---

# Part 2 — How training is done

## 2.1 The idea: behavioral cloning (imitation, not RL)

Training is **supervised learning**, exactly like image classification — except
the "label" is the action:

```
input :  (wrist image, "pick the red tomato", joint state)
target:  the 7 action values the demonstrator commanded next
loss  :  how far the model's predicted action is from the demonstrated one
```

There is **no reward, no trial-and-error, no exploration** — the model never
moves the robot during training. It just gets better at predicting *what the
demonstrator would have done*. That's why data quality is everything.

## 2.2 The model: SmolVLA (450M), and what actually gets trained

SmolVLA = a small **V**ision-**L**anguage-**A**ction model:
- a compact **vision-language backbone** (SmolVLM-family) reads the image and
  the instruction;
- an **action expert** head turns that understanding into continuous joint
  actions (predicted in chunks of future steps, so motion is smooth).

Fine-tuning does **not** retrain everything. Measured from our run:

```
num_total_params     = 450M    ← the whole model
num_learnable_params = 100M    ← what we actually train (~22%)
"Reducing the number of VLM layers to 16"  ← lerobot trims the backbone
```

The pretrained vision-language understanding is largely kept (it already knows
what "red" looks like); training mostly teaches the **action expert** how *our*
UR5e moves. That's why ~50 episodes can be enough.

## 2.3 The actual command (what `finetune_d4` runs)

```bash
~/raise_venvs/lerobot/bin/lerobot-train \
  --policy.path=lerobot/smolvla_base \          # start from the pretrained model
  --dataset.repo_id=raiseschool/raise_ripeness_sort_ref \
  --dataset.root=<repo>/RAISE2026/datasets/raiseschool__raise_ripeness_sort_ref \
  --batch_size=64 --steps=6000 \
  --policy.device=cuda \
  --policy.push_to_hub=false \                  # local checkpoint only
  --rename_map='{"observation.images.wrist": "observation.images.camera1"}'
```

Three flags exist because real runs failed without them (all fixed in the
launcher so students never see the errors):
- `--dataset.root` — read the dataset from the **repo folder**, not the HF hub;
- `--policy.push_to_hub=false` — lerobot 0.5 otherwise demands a hub repo id;
- `--rename_map` — smolvla_base was pretrained with cameras named
  `camera1/2/3`; our single wrist camera must be mapped onto `camera1`.
  (The executor's client introspects the policy config at load time, so
  inference automatically uses the same key.)

## 2.4 What one training step does

1. Sample a **batch of 64 frames** from random episodes.
2. **Normalize** images/state/actions using the dataset's `meta/stats.json`
   (recorded exactly for this purpose).
3. Forward pass → the model predicts an action chunk per frame.
4. Loss = distance from the demonstrated actions; backprop; optimizer update.

With 4228 frames and batch 64, one **epoch** ≈ 66 steps — so 6000 steps means
the model sees every frame ~**89 times**. Small dataset, many passes: normal
for behavioral cloning.

## 2.5 Measured performance (this machine, RTX 4090 laptop 16 GB)

| Metric | Measured value |
|---|---|
| VRAM during training | **10.8 / 16 GB** (batch 64 fits) |
| Speed | ≈ **1.1 s/step** (sim sharing the GPU) |
| Reference recipe | 49 scan episodes, **6000 steps ≈ 2 h** wall-clock |
| Result | **100/100** — 8/8 correct picks in greenhouse scenes, 0 wrong grabs, ≤201 ms/action |
| Output | `~/raise_checkpoints/smolvla_C_ref` (+ `…train.log` beside it) |

Rule of thumb from these numbers: **~1 step/second** on this GPU, so pick
`--steps` by the time you have (3000 ≈ 1 h, 6000 ≈ 2 h, 20000 ≈ 6 h).

## 2.5b The observability lesson (the best bug of the school)

When we first added greenhouse context, the model **plateaued at 50% success —
exactly chance** — and neither more data (29→50 episodes) nor more training
(3000→6000 steps) moved it. The real cause: at every decision pose, the
spawned tomatoes were **hidden behind the gripper**. The information "which
side is red" was not in the observation, so no optimizer on earth could learn
it. We proved it by capturing exactly what the model sees, with the tomatoes
spawned — and then fixed the *choreography*, not the model: the scan
("look above a spot before deciding") puts the answer in every frame. Score
went from 40 → **100/100**.

> **When a policy is stuck at chance, audit the observation before you scale
> the data.** Look at what the camera actually sees at decision time.

## 2.6 The environment gotcha (read before reproducing)

LeRobot ≥ 0.5 requires **numpy 2**; ROS's `cv_bridge` requires numpy 1 — they
cannot share a Python. So LeRobot lives in its own venv
(`~/raise_venvs/lerobot`), and all Day-2 scripts convert camera images with
**pure numpy** (`vla_client/ros_image.py`) instead of cv_bridge, so they run in
either Python. Anything importing lerobot runs with the **venv python** — the
`03_d3 / 05_d3 / 06_d3 / finetune_d4 / vla_d4` aliases do this automatically.

## 2.7 After training: closing the loop

```bash
export VLA_LOCAL_CKPT=~/raise_checkpoints/smolvla_C_ref
vla_d4 --task C --instruction "pick the red tomato"     # grasp_server running!
```

The executor captures the wrist image → asks the fine-tuned model → streams the
predicted joints to the arm → the `grasp_server` makes the commanded close a
real pick. *What was recorded in the morning is the brain commanding the arm in
the afternoon* — the whole Day-2 thread, closed.
