<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Lab 2.2 — solution notes (instructor reference)

**The starters ARE the solution.** Unlike TODO-gapped labs, Day 2 ships
complete, working scripts — the student work is *operating the pipeline*
(collect → verify → train → run → score) and *improving the data/model*,
not filling code holes.

## The known-good sequence + expected outputs (all measured 2026-07-04)

```bash
raise-sim  &  grasp_d3                                   # sim + grasp (robot parked per sim_poses.PARK)
05_d3 --episodes 50 --team ref --hf-user raiseschool     # → 50/50 grasp OK, ~28 min (scan choreography)
06_d3 --task C --team ref --episode 0 --spawn            # → grasped tomato_red_0 ... released
validate_dataset.py --task C --team ref                  # → 100/100
finetune_d4 --task C --team ref --steps 6000 --launch    # → ~2 h
vla_d4 --task C --spawn                                  # → SUCCESS (~35 steps red-L / ~60 red-R incl. scan)
evaluate.py --task C --trials 8 --max-steps 120          # → reference: 100/100 (greenhouse scenes)
```

## What "better than reference" looks like (advanced-track targets)
- **Dataset 100/100:** the reference already hits it — beat the run count or add pose jitter.
- **Robustness:** add pose jitter in `sim_poses.py` grasps before recording —
  the policy generalizes to offsets the reference model hasn't seen.
- **Language:** record episodes for a second instruction ("pick the green
  tomato") and show instruction-switching in the same checkpoint.

## Where the bodies are buried (debug shortcuts)
- Inference path: `api_clients/vla_client/local_smolvla.py` (predict_action +
  processors + `reset()` — the two live-found bugs are documented inline).
- Grasp truth: `grasp_server` publishes `/grasp/state`; triggers on the
  COMMANDED knuckle (measured angle stalls on contact).
- Frame math: TF (base→gripper) ⊗ robot world pose from `gz topic` — the
  ros_gz Pose_V bridge drops entity names, do NOT use `/gz_world_poses`.
