<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# How to Train a VLA and Use It — Step by Step

> **Goal of this guide:** from nothing to a robot that picks the red tomato
> when you type *"pick the red tomato"* — by collecting demonstrations,
> training a SmolVLA model on them, and running it. Everything runs on **one
> machine**. Every number below is **measured**, not estimated.

The journey is five stages:

```
 ① install once → ② start sim → ③ collect demos → ④ train      → ⑤ use it
   (~10 min)                      (~25 min auto)    (~55 min bg)   (type a command)
```

> ⌨️ **No joystick needed.** The recommended path collects demos **hands-free**
> (a scripted expert drives). Keyboard teleop exists (`01_d3`) and is worth 10
> minutes to *feel* why manual demos are hard; the phone teleop only drives the
> Husky base, not the arm.

---

## ① Install the training tools (once)

LeRobot (the SmolVLA toolkit) goes into its **own virtual environment** — do
NOT install it into the system Python:

```bash
/usr/bin/python3 -m venv --system-site-packages ~/raise_venvs/lerobot
~/raise_venvs/lerobot/bin/python3 -m pip install --timeout 30 --retries 10 "lerobot[smolvla]"
```

> 🧠 **Why a venv?** LeRobot (≥0.5) *requires* numpy 2; ROS 2's `cv_bridge` is
> built for numpy 1 — they can never share one Python. The venv has its own
> numpy 2; the system Python keeps numpy 1 for the Day-1 tools. The Day-2
> scripts avoid cv_bridge entirely (pure-numpy image conversion), so they run
> in either Python.

Check it worked:
```bash
~/raise_venvs/lerobot/bin/python3 -c "import lerobot, numpy; print('lerobot OK, numpy', numpy.__version__)"
# numpy 2.x here is CORRECT (it's inside the venv)
```

> 📌 **Rule of thumb:** anything touching LeRobot (collect ③, train ④, run ⑤)
> uses the **venv python** — the aliases (`03_d3 05_d3 06_d3 finetune_d4
> vla_d4`) do this for you. Everything else is normal `ros2 run`.

---

## ② Start the simulator + grasp server

```bash
# terminal 1 — Gazebo, PARKED AT THE PLANT ROW (Day-2 poses + the trained
# model expect this spot; plain raise-sim spawns in the empty aisle and the
# scripts will warn you). add headless:=true on a server / for speed.
sim_d2            # = raise-sim x:=-2.0 y:=2.15 yaw:=1.5708
# terminal 2 — the grasp server: it IS the sim's grasping. Keep it running for
# BOTH collection and execution.
grasp_d3          # = ros2 run raise2026_tools grasp_server
```

---

## ③ Collect demonstrations (hands-free — 25 min)

```bash
# terminal 3
05_d3 --episodes 50 --team team07 --hf-user me
```

A scripted expert spawns a red + green tomato at a real plant row, then
**scans** — it looks above the left spot, and descends only if the tomato it
SEES is red (else it pans right). That "look before you act" makes every
frame decidable from the image alone, which is exactly what the policy can
learn. Every take whose grasp doesn't verify is discarded. 50 clean episodes
≈ 25 minutes.

**Then verify the data before training** (the golden rule):
```bash
06_d3 --task C --team team07 --hf-user me --episode 0 --spawn
```
The arm replays the recorded episode and the pick should reproduce. Smooth
motion ending in a grasp → train. Anything else → fix the demos, not the model.

Your dataset lives **inside the repo**:
`RAISE2026/datasets/me__raise_ripeness_sort_team07`
(see a finished example with images: `RAISE2026/datasets/raiseschool__raise_ripeness_sort_ref/`).

<details><summary>⌨️ Manual alternative (drive it yourself)</summary>

```bash
01_d3 --task C          # terminal 3 — keyboard teleop (h = help)
03_d3 --task C --team team07 --hf-user me    # terminal 4 — recorder prompts you per episode
```
Slower and harder — but a valuable 10-minute experience of why data pipelines
get automated.
</details>

---

## ④ Train the model (~55 min, runs in the background)

```bash
finetune_d4 --task C --team team07 --hf-user me --steps 6000 --launch
tail -f ~/raise_checkpoints/smolvla_C_team07.train.log     # watch it learn
```

> ⏱️ **Measured on the school GPU (RTX 4090 laptop, 16 GB):** ~1.1 s/step →
> **6000 steps ≈ 2 h** (the reference recipe; 3000 ≈ 1 h for a quick try),
> 10.8 GB VRAM at batch 64. The job survives closing the terminal. The launcher already handles the three flags that otherwise
> abort training (`--dataset.root`, `--policy.push_to_hub=false`, the
> camera `--rename_map`).

The checkpoint lands at
`~/raise_checkpoints/smolvla_C_team07/checkpoints/last/pretrained_model`.

---

## ⑤ Use it — the payoff

```bash
export VLA_LOCAL_CKPT=~/raise_checkpoints/smolvla_C_team07/checkpoints/last/pretrained_model
vla_d4 --task C --spawn --instruction "pick the red tomato"
```

`--spawn` sets up the scene (red + green tomato; `--red-side right` to flip).
If `VLA_LOCAL_CKPT` is unset, the executor auto-uses the **reference
checkpoint** when present (see `RAISE2026/checkpoints/README.md`). Expected:

```
  outcome     : SUCCESS  (red tomato grasped)
  steps       : 60            # includes the SCAN: look left → green → pan right → pick
  latency     : mean 17.2 ms, max ~200 ms
  ≤500 ms/action: OK ✓
```

**Score it properly** (10 alternating trials, the Lab-2.2 grade):
```bash
~/raise_venvs/lerobot/bin/python3 \
  src/raise2026_labs/day2/day2_02_vla_executor/evaluator/evaluate.py --task C --trials 10
```
The reference model scores **100/100** (8/8 picks in greenhouse scenes, 0 wrong grabs).

---

## Quick reference (copy-paste)

```bash
# once
/usr/bin/python3 -m venv --system-site-packages ~/raise_venvs/lerobot
~/raise_venvs/lerobot/bin/python3 -m pip install --timeout 30 --retries 10 "lerobot[smolvla]"

# every session
sim_d2                                                       # T1: sim at the plant row
grasp_d3                                                     # T2: grasp server
05_d3 --episodes 50 --team t --hf-user me                    # T3: collect (25 min)
06_d3 --task C --team t --hf-user me --episode 0 --spawn     # verify by replay
finetune_d4 --task C --team t --hf-user me --steps 6000 --launch   # train (~2 h)
export VLA_LOCAL_CKPT=~/raise_checkpoints/smolvla_C_t/checkpoints/last/pretrained_model
vla_d4 --task C --spawn                                      # the payoff
```

## If something goes wrong

| Symptom | Likely cause / fix |
|---|---|
| `lerobot not installed` | wrong python — use the venv (`~/raise_venvs/lerobot/bin/python3`; aliases do it) |
| Day-1 camera tools break with numpy errors | numpy 2 leaked into the **system** python — lerobot only ever goes in the venv |
| `gripper closed but no fruit in range` | tomato not at the tool: is `grasp_server` running? did you use `--spawn`? tune `attach_radius` (VERIFY_MANIPULATION.md) |
| run 1 works, run 2+ times out | the policy's chunk queue wasn't reset — use the shipped executor (it calls `client.reset()`), don't hand-roll the loop |
| training aborts at start | you're running an old launcher — the current one passes `--dataset.root`, `push_to_hub=false`, `--rename_map` automatically |
| model loads but arm flails | you're on the base (un-fine-tuned) checkpoint, or too few/too-similar demos — check `VLA_LOCAL_CKPT`, then record more variety |
| training feels stuck | it isn't — ~1.1 s/step; check `tail -f` of the `.train.log` |

## Deeper reading
- What's happening under the hood: [`HOW_IT_WORKS_DATA_AND_TRAINING.md`](./HOW_IT_WORKS_DATA_AND_TRAINING.md)
- The dataset explained with images: `RAISE2026/datasets/raiseschool__raise_ripeness_sort_ref/README.md`
- Sim manipulation layer + the 4 tunables: [`VERIFY_MANIPULATION.md`](./VERIFY_MANIPULATION.md)
- Checkpoint distribution: `RAISE2026/checkpoints/README.md`
