<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Model checkpoints — how to get the Day-2 reference brain

> Trained checkpoints are **not stored in git** (the reference SmolVLA run is
> **1.3 GB**). This page tells you how to get one. The *dataset* they are
> trained on IS in the repo (`RAISE2026/datasets/`) — so anyone can reproduce
> the checkpoint exactly.

## The reference checkpoint (`smolvla_C_ref`, v3 — greenhouse scan policy, deterministic parking)

| | |
|---|---|
| Task | C — "pick the red tomato" among green distractors, at a real plant row |
| Trained on | `RAISE2026/datasets/raiseschool__raise_ripeness_sort_ref` (in this repo — 49 SCAN episodes, snap_to_park protocol) |
| Recipe | SmolVLA base, 6000 steps, batch 64 — **~2 h** on an RTX 4090 laptop (16 GB) |
| Result | **100/100** on the Lab-2.2 evaluator (8/8 picks in greenhouse scenes, 0 wrong grabs, ≤167 ms/action — fresh-session verified 2026-07-15) |
| Expected local path | `~/raise_checkpoints/smolvla_C_ref/checkpoints/006000/pretrained_model` |

The Day-2 tools **auto-detect the newest checkpoint** under
`~/raise_checkpoints/smolvla_C_ref/checkpoints/` when `VLA_LOCAL_CKPT` is unset —
and if nothing is installed there, they **download the public Hugging Face copy
automatically** (Option 1-b below) on first run. You can't end up on the
un-fine-tuned base model by accident.

## Option 1 — download from the PUBLIC GitHub Release (recommended: ~2 min, no login)

The checkpoint is a **public release asset** (687 MB tar.gz) on the student repo:
<https://github.com/aniskoubaa/agribot-student/releases/tag/day2-smolvla-ref-v3>

One command — no token, no login (the alias from `raise_aliases.sh`):

```bash
agr_vla_hf
```

Or by hand (identical result):

```bash
mkdir -p ~/raise_checkpoints/smolvla_C_ref/checkpoints/006000
curl -L -o /tmp/smolvla_C_ref_v3.tar.gz \
  https://github.com/aniskoubaa/agribot-student/releases/download/day2-smolvla-ref-v3/smolvla_C_ref_v3.tar.gz
tar -xzf /tmp/smolvla_C_ref_v3.tar.gz \
    -C ~/raise_checkpoints/smolvla_C_ref/checkpoints/006000 && rm /tmp/smolvla_C_ref_v3.tar.gz
```

(Or in the browser: open the release page, click `smolvla_C_ref_v3.tar.gz`,
then run the `tar -xzf ~/Downloads/...` line with the same `-C` target.)

The executor auto-detects the resulting path.

## Option 1-b — Hugging Face (same model, also public & keyless)

The checkpoint is also published at
<https://huggingface.co/scalexi/smolvla-agribot-ripeness-ref> — browse the
weights, read the model card, or load it directly by repo id (downloads the
files into the HF cache on first use):

```bash
export VLA_LOCAL_CKPT=scalexi/smolvla-agribot-ripeness-ref
agr_vla --task C --spawn
```

```python
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
policy = SmolVLAPolicy.from_pretrained("scalexi/smolvla-agribot-ripeness-ref")
```

## Option 2 — reproduce it (~2.5 h, one command)

The training dataset is in the repo, so anyone can retrain the same thing:

```bash
agr_finetune --task C --team ref --hf-user raiseschool --steps 6000 --launch
tail -f ~/raise_checkpoints/smolvla_C_ref.train.log     # done in ~2 h
```

## Option 3 — copy from the instructor box (LAN)

```bash
rsync -a instructor-box:~/raise_checkpoints/smolvla_C_ref ~/raise_checkpoints/
```

## Using ANY checkpoint

```bash
export VLA_LOCAL_CKPT=<path-to>/checkpoints/00XXXX/pretrained_model
agr_vla --task C --spawn
```

Point at the `pretrained_model` folder (it holds `model.safetensors`, the
config, and the pre/post processor files the inference pipeline needs).
