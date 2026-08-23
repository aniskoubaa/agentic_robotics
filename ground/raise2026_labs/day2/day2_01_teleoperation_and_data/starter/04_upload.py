#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Lab 2.1, Script 04 — ship your dataset to the shared GPU server.

WHAT  Pack the LeRobot dataset you recorded with Script 03 into one archive and
      POST it to the shared RAISE server, tagged with your team id + task. The
      server is where fine-tuning runs (it has the 4090), so this upload is what
      turns "my demos" into "my model's training data".

WHY   Fine-tuning happens on the GPU box, not your laptop (see DAY2_LAB_DESIGN.md
      §3 — training is hours, so it runs in the background on the shared server).
      Uploading is the hand-off from the TEACH half to the TRAIN bridge.

LEARN - The dataset is just a directory of parquet + images + metadata. We
        tar.gz it and send it as a multipart POST — the same HTTP-client idea as
        vla_client, one level simpler.
      - The server indexes it by (team_id, task) so each team's data — and the
        checkpoint trained from it — stays separate.

Run after recording:

    ros2 run raise2026_labs 04_upload.py --task A --team team07 \
        --server http://gpu-box.lan:8000
    # or:  04_d3 ...
"""

import argparse
import io
import sys
import tarfile
from pathlib import Path

try:
    import _d2paths                    # installed next to the ros2-run scripts
except ImportError:                    # running straight from the source tree
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    import _d2paths
_d2paths.bootstrap(lerobot=False)
DAY2 = _d2paths.DAY2
from task_pack import load_task   # noqa: E402


def tar_directory(path: Path) -> bytes:
    """Pack a directory into an in-memory .tar.gz (no temp files on disk)."""
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode='w:gz') as tar:
        tar.add(path, arcname=path.name)
    return buf.getvalue()


def main():
    ap = argparse.ArgumentParser(description='RAISE 2026 dataset uploader')
    ap.add_argument('--task', default='A', help='A | C | B')
    ap.add_argument('--team', default='team00')
    ap.add_argument('--hf-user', default='raiseschool')
    ap.add_argument('--root', default=str(_d2paths.DATASETS_DIR))
    ap.add_argument('--server', default='http://localhost:8000', help='shared GPU server base URL')
    args = ap.parse_args()

    pack = load_task(args.task)
    repo_id = pack.dataset_repo_id(args.hf_user, args.team)
    root = Path(args.root) / repo_id.replace('/', '__')
    if not root.is_dir():
        print(f'✗ no dataset at {root}. Record with 03_record.py first.')
        sys.exit(1)

    # Count episodes from the LeRobot metadata so the student sees what ships.
    meta = root / 'meta' / 'info.json'
    print(f'Packing {root} ...')
    blob = tar_directory(root)
    print(f'  archive size: {len(blob) / 1e6:.1f} MB'
          + (f'  (meta: {meta})' if meta.exists() else ''))

    try:
        import requests
    except ImportError:
        print('✗ `requests` not installed (pip install requests).')
        sys.exit(1)

    url = args.server.rstrip('/') + '/datasets/upload'
    print(f'Uploading to {url}  (team={args.team}, task={pack.id}) ...')
    try:
        r = requests.post(
            url,
            data={'team_id': args.team, 'task': pack.id, 'repo_id': repo_id},
            files={'archive': (f'{repo_id.replace("/", "__")}.tar.gz', blob, 'application/gzip')},
            timeout=300,
        )
        r.raise_for_status()
    except Exception as e:
        print(f'✗ upload failed: {e}')
        print('  (Is the shared server up? Ask the instructor for the --server URL.)')
        sys.exit(1)

    print(f'  ✓ uploaded. server says: {r.text[:200]}')
    print('\nYour data is on the GPU box. The instructor (or your advanced-track')
    print('fine-tune job) will train SmolVLA on it — see day2_02 / finetune_smolvla.py.')


if __name__ == '__main__':
    main()
