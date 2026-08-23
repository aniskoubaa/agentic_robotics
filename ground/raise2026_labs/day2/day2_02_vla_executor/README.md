<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Lab 2.2 — Command the Robot by Speaking

**Day 2, 16:00 – 17:30 (90 min) · for everyone (advanced = train your own model)**

## The objective of this lab

> **The objective of this lab is to type "pick the red tomato" and watch a
> model — trained on this morning's demonstrations — move the arm and do it,
> with you writing no motion code at all.**

This is the payoff of Day 2. And it works: the reference model scores
**100/100** on this lab's evaluator — **8/8 correct picks** in real greenhouse
scenes (foliage everywhere!), red on either side, never touching the green
tomato, max 167 ms per decision. The model even **scans**: it looks above one
spot, and if the tomato there is green it pans to the other side — active
perception it learned from the demonstrations.

## The three pieces (all provided, all measured)

| Piece | What it is | Reference numbers |
|---|---|---|
| **The brain** | SmolVLA fine-tuned on the Lab-2.1 dataset | trained in **~2 h** on the school GPU (6000 steps, 10.8/16 GB) |
| **The loop** | `vla_executor.py`: see → ask the model → act | mean ~10 ms per decision |
| **The truth** | `grasp_server`'s `/grasp/state` says what's really held | red attach = success; green = fail |

## First: look inside ONE brain call (2 minutes, do this before the full run)

A VLA is just a function: `action = model(image, instruction, state)`.
`vla_one_step.py` grabs those three inputs live from the sim, calls the model
**once**, and prints exactly what went in and what came out (plus a PNG card
of the camera frame next to the numbers):

```bash
vla_one_d4 --spawn                 # one call, every input/output explained
vla_one_d4 --steps 5               # consecutive calls — see the action CHUNK stream out
vla_one_d4 --spawn --execute       # publish the action: the arm actually moves
```

The `--execute` publish line, repeated at 10 Hz, IS the whole executor below.

## How to run (the 2-minute payoff)

```bash
# terminal 1 — the sim            sim_d2   (plant-row parking — the model expects it)
# terminal 2 — the grasp server   grasp_d3        (must run — it IS the grasp)
# terminal 3 — the executor (venv python; the alias handles it):
vla_d4 --task C --spawn --instruction "pick the red tomato"
```

`--spawn` places a red + green tomato at the trained positions first
(`--red-side right` to flip them). The checkpoint: if `VLA_LOCAL_CKPT` is
unset, the executor **auto-uses the local reference checkpoint**; point it at
your own fine-tune to run yours. You'll get:

```
  outcome     : SUCCESS  (red tomato grasped)
  steps       : 60            # includes the scan: look left → green → pan right → pick
  latency     : mean 17.2 ms, max ~200 ms
  ≤500 ms/action: OK ✓
```

One-command demo: `sim/demos/d2l2_vla_rollout.sh C "pick the red tomato"`.

## Train your own (the advanced track — background, not a wait)

Fine-tuning takes ~**2 h** for the full 6000-step recipe (measured; 3000 ≈ 1 h
for a quick try), so you *launch it and walk away*:

```bash
finetune_d4 --task C --team team07 --hf-user me --steps 6000 --launch
tail -f ~/raise_checkpoints/smolvla_C_team07.train.log        # watch it learn
# when finished:
export VLA_LOCAL_CKPT=~/raise_checkpoints/smolvla_C_team07/checkpoints/last/pretrained_model
vla_d4 --task C --spawn
```

Then A/B your model vs the reference over the same trials (below) and put the
comparison in your report.

## Three lessons hidden in this lab (ask about them!)

1. **The executor is backend-blind** — swap the local model for a self-hosted
   endpoint with two env vars, no code change (`../LOCAL_VS_REMOTE_VLA.md`).
2. **`client.reset()` before every episode** — the model predicts action
   *chunks* through a queue; without a reset, stale chunks from the previous
   run leak in. (Live symptom: run 1 works, runs 2+ time out.)
3. **Judge by ground truth, not vibes** — success is `/grasp/state` naming the
   red tomato, not "the arm looked right".

## Grading (auto-scored: `evaluator/evaluate.py`)

```bash
export VLA_LOCAL_CKPT=<your checkpoint>/pretrained_model
~/raise_venvs/lerobot/bin/python3 evaluator/evaluate.py --task C --trials 10
```

- **60%** — correct-color picks over the trials (red side alternates L/R)
- **20%** — every model call within the ≤500 ms budget
- **20%** — safety: never grabs the green tomato

Reference checkpoint: **100/100**. Beat it with better data (more episodes,
more variety) — that's the real lesson: *data quality > model size*.

## Want the full story?

- How training works under the hood: [`../HOW_IT_WORKS_DATA_AND_TRAINING.md`](../HOW_IT_WORKS_DATA_AND_TRAINING.md)
- The complete beginner runbook: [`../HOW_TO_TRAIN_AND_USE.md`](../HOW_TO_TRAIN_AND_USE.md)
- Getting the reference checkpoint: [`RAISE2026/checkpoints/README.md`](../../../../../checkpoints/README.md)
