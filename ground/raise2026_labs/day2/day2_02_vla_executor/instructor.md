<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Lab 2.2 — Instructor notes

## Pre-lab checklist
1. **The reference checkpoint exists** at
   `~/raise_checkpoints/smolvla_C_ref/checkpoints/003000/pretrained_model` on
   the demo box (reproduce: `finetune_d4 --task C --team ref --hf-user raiseschool --steps 3000 --launch`, ~55 min — see `RAISE2026/checkpoints/README.md`).
2. Dry payoff run before students arrive:
   `vla_d4 --task C --spawn` → expect `SUCCESS (red tomato grasped)` in ≲40 steps.
3. `grasp_d3` running (it is the grasp physics — without it nothing picks).
4. If students will train their own: the GPU fits ONE training at a time
   (10.8/16 GB) — queue them, or share the reference checkpoint.

## Timing plan (90 min)
| min | activity |
|---|---|
| 0–10 | the punchline first: run `vla_d4 --task C --spawn` live — it picks |
| 10–25 | unpack the loop (see→act→truth) + the backend-blind design |
| 25–40 | everyone runs the executor, flips `--red-side`, tries wordings |
| 40–55 | launch team fine-tunes (`finetune_d4 --launch`) — background! |
| 55–75 | run `evaluator/evaluate.py --trials 10` with the reference; teams compare when their checkpoints land (or after the session) |
| 75–90 | the two inference-bug stories (below) + wrap-up |

## Teach the two live-found bugs (they're the best content in the lab)
1. **`select_action` KeyError `observation.language.tokens`** — lerobot 0.5
   policies need the pre/post processor pipelines saved with the checkpoint
   (tokenize + normalize in, un-normalize out). Lesson: *inference must mirror
   the training pipeline exactly.*
2. **Trial 1 succeeds, trials 2+ time out** — SmolVLA predicts action CHUNKS
   through a queue; without `client.reset()` per episode, stale chunks leak.
   Lesson: *policies are stateful; episodes need clean boundaries.*

## Known failure modes
| Symptom | Cause / fix |
|---|---|
| arm flails randomly | un-fine-tuned base checkpoint — `VLA_LOCAL_CKPT` unset and no local reference; check the executor's printed `Checkpoint:` line |
| `TIMEOUT` with sane motion | tomatoes not in the scene — forgot `--spawn`; or grasp_server down |
| first call ~460 ms, rest ~5 ms | normal: chunk generation amortizes; budget checks the max (452–480 ms is fine) |
| `wrong_object` (green grabbed) | genuinely bad policy or swapped dataset colors — check the team's dataset with 06 replay |
| CUDA OOM during a student training | another training already running — one at a time on the 16 GB GPU |

## Grading
`evaluate.py --task C --trials 10` (venv python, `VLA_LOCAL_CKPT` set to the
team's checkpoint). Reference = **100/100**; the JSON line is the gradebook
entry. Success 60 / latency 20 / safety 20 — safety violations (green grabs)
should prompt a conversation, not just a deduction.
