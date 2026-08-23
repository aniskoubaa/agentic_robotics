#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Lab 2.2 (bridge) — fine-tune SmolVLA on this morning's demos.

WHAT  Take the LeRobot dataset you recorded in Lab 2.1 and fine-tune SmolVLA on
      it, producing the checkpoint the executor will run in the afternoon.

WHY   This is the "TRAIN" column of the Day-2 arc. IMPORTANT: fine-tuning is
      HOURS, not minutes (see DAY2_LAB_DESIGN.md §3). So this does NOT block the
      lab — you LAUNCH it and walk away:
        • Default path: the instructor already ran this before the school and
          ships the reference checkpoint, so beginners skip straight to running.
        • Advanced path: launch your OWN fine-tune (fire-and-forget) on the
          shared GPU, then in Lab 2.2 benchmark YOUR checkpoint vs the reference.

      This is imitation learning (behavioral cloning): SmolVLA copies the
      (image, instruction, state) -> action mapping in your demonstrations. It
      is NOT reinforcement learning — there is no reward, just demonstrations.

LEARN The whole training is one LeRobot command; this script just builds it with
      the right dataset id and our machine conventions. The knobs that matter:
        --steps     how long to train. 3000 = a quick visible result;
                    20000 = a full fine-tune. MEASURE the wall-clock on the real
                    4090 and pick what fits (see DAY2_LAB_DESIGN.md §3.1).
        --batch     64 is comfortable in the 4090's 24 GB for SmolVLA-450M.

Conventions baked in (this machine):
  • strip anaconda from PATH so colcon/rclpy's python is used, not conda's
  • numpy<2 must already be pinned in the lerobot env (cv_bridge needs it)

Usage:
    # Print the exact command (dry run — recommended first):
    ros2 run raise2026_labs finetune_smolvla.py --task A --team team07 --dry-run

    # Launch it in the background on the GPU box (fire-and-forget):
    ros2 run raise2026_labs finetune_smolvla.py --task A --team team07 \
        --hf-user raiseschool --steps 6000 --launch
    # or:  finetune_d4 ...
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

try:
    import _d2paths                    # installed next to the ros2-run scripts
except ImportError:                    # running straight from the source tree
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    import _d2paths
_d2paths.bootstrap(lerobot=False)
DAY2 = _d2paths.DAY2
from task_pack import load_task   # noqa: E402


# lerobot lives in its own venv (numpy-2; see HOW_TO_TRAIN_AND_USE.md §1).
# Prefer the venv's lerobot-train; fall back to PATH for custom setups.
LEROBOT_TRAIN = str(Path.home() / 'raise_venvs' / 'lerobot' / 'bin' / 'lerobot-train')
if not Path(LEROBOT_TRAIN).exists():
    LEROBOT_TRAIN = 'lerobot-train'


def build_command(repo_id: str, dataset_root: Path, output_dir: Path,
                  steps: int, batch: int) -> list[str]:
    """The LeRobot fine-tune invocation (lerobot-train), as an argv list."""
    return [
        LEROBOT_TRAIN,
        '--policy.path=lerobot/smolvla_base',     # start from the pretrained backbone
        f'--dataset.repo_id={repo_id}',           # YOUR Lab-2.1 dataset
        f'--dataset.root={dataset_root}',         # ...read it from the LOCAL folder
        f'--batch_size={batch}',
        f'--steps={steps}',
        '--policy.device=cuda',
        '--policy.push_to_hub=false',   # local checkpoint only (else 0.5 demands policy.repo_id)
        # smolvla_base was pretrained with camera keys camera1/2/3; our dataset
        # has ONE wrist camera. Map it onto camera1 (found live: training aborts
        # with "Feature mismatch" without this).
        '--rename_map={"observation.images.wrist": "observation.images.camera1"}',
        f'--output_dir={output_dir}',
    ]


def clean_env() -> dict:
    """A copy of the environment with anaconda stripped from PATH.

    rclpy + lerobot must run under the system python the workspace was built
    with; a conda python on PATH breaks the rclpy C extension. See memory
    [[feedback-anaconda-colcon]].
    """
    env = dict(os.environ)
    parts = [p for p in env.get('PATH', '').split(os.pathsep) if 'anaconda' not in p and 'conda' not in p]
    env['PATH'] = os.pathsep.join(parts)
    return env


def main():
    ap = argparse.ArgumentParser(description='RAISE 2026 SmolVLA fine-tune launcher')
    ap.add_argument('--task', default='A', help='A | C | B')
    ap.add_argument('--team', default='team00')
    ap.add_argument('--hf-user', default='raiseschool')
    ap.add_argument('--steps', type=int, default=6000, help='3000 quick … 20000 full (MEASURE first)')
    ap.add_argument('--batch', type=int, default=64)
    ap.add_argument('--out-root', default=str(Path.home() / 'raise_checkpoints'))
    ap.add_argument('--data-root', default=str(_d2paths.DATASETS_DIR),
                    help='where the dataset lives — inside the repo (RAISE2026/datasets), must match where it was recorded')
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--dry-run', action='store_true', help='print the command and exit (default)')
    g.add_argument('--launch', action='store_true', help='actually start training in the background')
    args = ap.parse_args()

    pack = load_task(args.task)
    repo_id = pack.dataset_repo_id(args.hf_user, args.team)
    # Same local folder 03_record.py wrote to, so training reads YOUR data.
    dataset_root = Path(args.data_root) / repo_id.replace('/', '__')
    output_dir = Path(args.out_root) / f'smolvla_{pack.id}_{args.team}'
    cmd = build_command(repo_id, dataset_root, output_dir, args.steps, args.batch)

    if not dataset_root.is_dir():
        print(f'⚠ no dataset at {dataset_root} — record with 03_record.py first '
              f'(or pass --data-root).\n')

    print(f'Task {pack.id}: {pack.name}')
    print(f'Dataset  : {repo_id}  ({dataset_root})')
    print(f'Output   : {output_dir}')
    print(f'Steps    : {args.steps}   Batch: {args.batch}\n')
    print('Command:\n   ' + ' '.join(cmd) + '\n')

    if not args.launch:
        print('Dry run — nothing started. Re-run with --launch to train in the background.')
        print('When it finishes, point the executor at the checkpoint:')
        print(f'   export VLA_LOCAL_CKPT={output_dir}')
        return

    # Fire-and-forget: detach so closing this terminal does not kill training.
    # Logs go to a file the student can `tail -f` while training runs for hours.
    # NOTE: the log lives NEXT TO the output dir, not inside it — lerobot-train
    # refuses to start if output_dir already exists (found live: pre-creating
    # it for the log aborted the run with FileExistsError).
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    log_path = output_dir.parent / f'{output_dir.name}.train.log'
    print(f'Launching in background. Logs: {log_path}')
    print(f'  watch with:  tail -f {log_path}')
    with open(log_path, 'w') as log:
        subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                         env=clean_env(), start_new_session=True)
    print('\n✓ training launched. You can close this terminal.')
    print(f'When done:  export VLA_LOCAL_CKPT={output_dir}   then run vla_executor.py')


if __name__ == '__main__':
    main()
