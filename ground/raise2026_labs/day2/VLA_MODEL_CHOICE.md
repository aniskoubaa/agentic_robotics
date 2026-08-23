<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Day 2 — Choosing a VLA Model (decision + how-to)

> **Decision:** **SmolVLA** is the backbone of both Day-2 labs.
> π₀ is the advanced/stretch option; OpenVLA is demo-only; ACT is the safety net.
> Rationale, facts, and download/usage steps below.

---

## 1. The open-source VLA field, ranked for *our* case

Our constraints: **UR5e (6-DoF) + gripper**, RealSense wrist cam, **LeRobot-format
recording**, a **single RTX 4090 (24 GB)** to fine-tune on, **90–105 min student
labs**, ≤500 ms/action-chunk latency.

| Model | Size | Fine-tune on a 4090? | LeRobot-native? | Best for | Verdict |
|---|---|---|---|---|---|
| **SmolVLA** | ~450 M | ✅ trivially (~12 GB) | ✅ **built by the LeRobot team** | consumer HW, fast FT, our exact format | 🥇 **Primary** |
| **π₀ / π₀.₅** (openpi) | ~3 B | ⚠️ LoRA only, tight | ✅ (openpi + LeRobot loaders) | SOTA generalist, smooth 50 Hz actions | 🥈 advanced/stretch |
| **OpenVLA** (+ **OFT**) | 7 B | ❌ full FT needs A100 | partial | strongest raw success; OFT is fast (109 Hz) | demo / zero-shot only |
| **NVIDIA GR00T N1.5 / N1.7** | 3 B | ⚠️ heavy | via Isaac, not LeRobot | humanoids, dual-arm, reasoning | overkill |
| **ACT / Diffusion Policy** | ~80 M | ✅ trains in *minutes* | ✅ | *not* a VLA (no language) — solid baseline | 🛟 safety net |

---

## 2. Why SmolVLA wins for RAISE

1. **LeRobot-native.** We already record in LeRobot v2.0 format — SmolVLA consumes
   it *directly*. `/joint_states` → state, `/wrist_camera` → image, action stream
   → action. One ecosystem, no embodiment-gap glue.
2. **Fine-tunes inside a 90-min lab on one 4090.** ~50 demos recommended; our lab
   collects 30 — enough for a visible result. People train it on a 12 GB 3080Ti.
   OpenVLA-7B simply won't full-fine-tune on a 4090.
3. **"Good enough to be impressive."** 82–90 % on LIBERO — within ~5 % of OpenVLA
   at **16× fewer parameters**.
4. **Latency is a non-issue.** Async inference (30 % faster, 2× throughput),
   easily under the ≤500 ms/chunk budget on a 4090.

**π₀ is the advanced stretch goal**, not the default — current SOTA generalist
(flow-matching, 50 Hz), openpi ships fine-tune examples, but it's ~3 B and needs
LoRA on a 4090. Great "show the frontier" content; risky as the thing 20 students
must get running in 90 minutes.

---

## 3. ⚠️ Correction to the original lab plan

`day2_02` README said **beginner = "zero-shot against a hosted OpenVLA endpoint."**
True zero-shot VLA on a *novel embodiment + novel greenhouse task* usually performs
poorly — students would see the arm flail and conclude VLAs don't work.

**Better framing (same lab structure, the "wow" actually fires):**
- **Instructor** fine-tunes SmolVLA on a reference dataset **before the school** and
  hosts *that* checkpoint as the "VLA endpoint." Beginners call it → it works.
- **Advanced** track fine-tunes their *own* on the morning's 30 demos and benchmarks
  **their model vs the reference** over the same scenarios.
- Keep **ACT** in reserve: if a team's VLA won't converge in time, ACT trains in
  minutes on the same dataset and almost always moves the arm.

---

## 4. How to download & use SmolVLA (this *is* `api_clients/vla_client/` + server)

**Install (on the 4090 server):**
```bash
pip install "lerobot[smolvla]"     # numpy<2 in the SAME resolve + GPU torch (our convention)
```

**Base checkpoint:** `lerobot/smolvla_base` on Hugging Face.

**Fine-tune on a recorded dataset (Lab 2.1's output):**
```bash
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=${HF_USER}/raise_greenhouse \
  --batch_size=64 --steps=20000 \
  --policy.device=cuda \
  --output_dir=outputs/smolvla_raise
# ~4 h for 20k steps on an A100; budget more on a 4090, or cut steps for the lab
```

**Serve it as the remote endpoint the labs hit** — two options:
- LeRobot's built-in **async policy server** (`PolicyServer`), or
- a thin **FastAPI** wrapper exposing `POST /act {image, instruction} → action chunk`.

That endpoint *is* what `api_clients/vla_client/` talks to and what
`sim/demos/d2l2_vla_rollout.sh` drives. Choosing SmolVLA also fixes the shape of
`vla_client.act(img, instruction)`.

---

## 5. The locked Day-2 architecture

```
Lab 2.1  record (LeRobot v2.0) ──► lerobot-train (SmolVLA) ──► checkpoint
                                                                  │ host
Lab 2.2  vla_client.act(img, instr) ──HTTP──►  SmolVLA endpoint ──┘
                                          │
                                          ▼  action chunk → stream to UR5e
```

Everything recorded in the morning feeds the model run in the afternoon — that
single thread is the whole Day-2 lesson.

---

## 6. Verified facts (mid-2026) for the slides / README

| Model | Params | Released | License | Pretrain data |
|---|---|---|---|---|
| SmolVLA | 450 M | Jun 2025 (HF/LeRobot) | Apache-2.0 | ~480 LeRobot community datasets |
| OpenVLA | 7 B | Jun 2024 | MIT | 970 k Open-X-Embodiment episodes (22 embodiments) |
| OpenVLA-OFT | 7 B | Feb 2025 | MIT | OpenVLA + OFT recipe → 109.7 Hz, 0.073 s/chunk, LIBERO 76.5→97.1 % |
| π₀ / π₀.₅ | ~3 B | Feb 2025 (openpi) | Apache-2.0 | 10 k+ hours robot data; PaliGemma backbone, flow matching |
| GR00T N1.5 / N1.7 | 3 B | 2025 / early-access | NVIDIA (N1.7 commercial) | humanoid-focused, Isaac ecosystem |
| ACT / Diffusion Policy | ~80 M | 2023–24 | open | per-task (no language) — LeRobot baselines |

**Sources:** SmolVLA — huggingface.co/blog/smolvla, arXiv 2506.01844, `lerobot/smolvla_base`.
OpenVLA — arXiv 2406.09246, github.com/openvla/openvla. OpenVLA-OFT — arXiv 2502.19645.
π₀ — github.com/Physical-Intelligence/openpi, pi.website/blog/openpi. GR00T — research.nvidia.com/labs/gear, github.com/NVIDIA/Isaac-GR00T.
