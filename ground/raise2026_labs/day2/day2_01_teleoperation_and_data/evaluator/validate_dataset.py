#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Lab 2.1 evaluator: score a recorded dataset.

Grades a team's LeRobot dataset against the Lab-2.1 rubric (README):
    50%  episode count vs the task pack's recommendation
    30%  validation — no NaNs, correct image size, action<->state alignment
    20%  diversity — variety of where the demonstrations start/reach

Run it with the lerobot venv python (the aliases do this for you):

    ~/raise_venvs/lerobot/bin/python3 validate_dataset.py --task C --team ref
    # or against any dataset folder directly:
    ~/raise_venvs/lerobot/bin/python3 validate_dataset.py --root <dataset dir> --repo-id <id>

Outputs a human-readable report AND a JSON line (for the gradebook).
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

try:
    import _d2paths                    # installed next to the ros2-run scripts
except ImportError:                    # running straight from the source tree
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    import _d2paths
_d2paths.bootstrap(lerobot=True)
DAY2 = _d2paths.DAY2
from task_pack import load_task   # noqa: E402

IMG_SIZE = 224
ALIGN_TOL_RAD = 0.35   # mean |action - state| beyond this means streams are misaligned


def load_dataset(repo_id: str, root: Path):
    try:                                        # LeRobot moved this module between releases
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    return LeRobotDataset(repo_id, root=str(root))


def main():
    ap = argparse.ArgumentParser(description='RAISE 2026 Lab 2.1 dataset validator')
    ap.add_argument('--task', default='C', help='A | C | B (sets the episode target)')
    ap.add_argument('--team', default='team00')
    ap.add_argument('--hf-user', default='raiseschool')
    ap.add_argument('--data-root', default=str(_d2paths.DATASETS_DIR))
    ap.add_argument('--root', default=None, help='explicit dataset dir (overrides task/team)')
    ap.add_argument('--repo-id', default=None)
    ap.add_argument('--sample-stride', type=int, default=7,
                    help='validate every Nth frame (fast, still catches systematic errors)')
    args = ap.parse_args()

    pack = load_task(args.task)
    repo_id = args.repo_id or pack.dataset_repo_id(args.hf_user, args.team)
    root = Path(args.root) if args.root else Path(args.data_root) / repo_id.replace('/', '__')
    if not root.is_dir():
        print(f'✗ no dataset at {root}')
        sys.exit(1)

    ds = load_dataset(repo_id, root)
    n_eps, n_frames, fps = ds.num_episodes, ds.num_frames, ds.fps
    print(f'Dataset {repo_id}: {n_eps} episodes, {n_frames} frames @ {fps} Hz\n')

    # ── 1) Episode count (50%) — linear credit up to the pack's target ──────
    target = pack.recommended_episodes
    count_score = min(1.0, n_eps / target)
    print(f'[count]      {n_eps}/{target} episodes            → {count_score * 50:.0f}/50')

    # ── 2) Validation (30%): NaNs, image size, action<->state alignment ─────
    idxs = range(0, n_frames, max(1, args.sample_stride))
    nan_bad = img_bad = 0
    align_errs, first_states = [], {}
    for i in idxs:
        it = ds[i]
        st = np.asarray(it['observation.state'], dtype=np.float32).reshape(-1)
        ac = np.asarray(it['action'], dtype=np.float32).reshape(-1)
        if np.isnan(st).any() or np.isnan(ac).any():
            nan_bad += 1
        img = it['observation.images.wrist']
        shape = tuple(img.shape)                      # CHW tensor or HWC array
        if IMG_SIZE not in shape or 3 not in shape:
            img_bad += 1
        # action is the *target* the state chases — same units, small gap.
        align_errs.append(float(np.mean(np.abs(ac[:6] - st[:6]))))
        ep = int(it['episode_index'])
        if ep not in first_states:                    # first sampled frame per episode
            first_states[ep] = st

    checked = len(list(idxs))
    mean_align = float(np.mean(align_errs))
    nan_ok = nan_bad == 0
    img_ok = img_bad == 0
    align_ok = mean_align < ALIGN_TOL_RAD
    val_score = (nan_ok + img_ok + align_ok) / 3.0
    print(f'[validate]   NaNs: {nan_bad}/{checked} frames {"✓" if nan_ok else "✗"}'
          f' | image {IMG_SIZE}px: {"✓" if img_ok else f"✗ ({img_bad} bad)"}'
          f' | align: {mean_align:.3f} rad {"✓" if align_ok else "✗"}'
          f'      → {val_score * 30:.0f}/30')

    # ── 3) Diversity (20%): spread of per-episode trajectories ──────────────
    # For task C the key variety is WHICH SIDE the pick goes (shoulder_pan sign
    # over the episode). We measure the spread of each episode's mean pan
    # command; two well-separated clusters (red-L vs red-R) score full marks.
    ep_pan = {}
    for i in range(0, n_frames, max(1, fps // 2)):    # ~2 samples/s
        it = ds[i]
        ep = int(it['episode_index'])
        ep_pan.setdefault(ep, []).append(float(np.asarray(it['action']).reshape(-1)[0]))
    means = np.array([np.mean(v) for v in ep_pan.values()])
    spread = float(np.std(means))
    # full credit at spread >= 0.15 rad (empirically: our L/R dataset ≈ 0.3)
    div_score = min(1.0, spread / 0.15)
    print(f'[diversity]  episode-pan spread: {spread:.3f} rad     → {div_score * 20:.0f}/20')

    total = count_score * 50 + val_score * 30 + div_score * 20
    print(f'\nTOTAL: {total:.0f}/100')
    print(json.dumps({'repo_id': repo_id, 'episodes': n_eps, 'frames': n_frames,
                      'count_score': round(count_score * 50, 1),
                      'validate_score': round(val_score * 30, 1),
                      'diversity_score': round(div_score * 20, 1),
                      'total': round(total, 1)}))


if __name__ == '__main__':
    main()
