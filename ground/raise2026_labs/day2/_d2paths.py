# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Day-2 path bootstrap (the ONE place import/venv plumbing lives).

Every Day-2 starter/evaluator begins with:

    try:
        import _d2paths                    # installed next to the ros2-run scripts
    except ImportError:                    # running straight from the source tree
        import sys; from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
        import _d2paths
    _d2paths.bootstrap(lerobot=False)      # True for scripts that import lerobot

What bootstrap() does:
  1. Finds the Day-2 library root in ANY install layout (student report 2026-07-11:
     `ros2 run` scripts failed with ModuleNotFoundError: vla_client on machines
     where colcon COPIED sources instead of symlinking them):
       a. the source tree  (…/raise2026_labs/day2 — where this file lives), or
       b. the installed copy under share/raise2026_labs/day2_lib (shipped by
          setup.py data_files exactly for the copied-install case).
  2. Puts api_clients/ and task_packs/common/ on sys.path → `import vla_client`,
     `import task_pack`, `import sim_poses` work everywhere.
  3. lerobot=True: if lerobot isn't importable under THIS interpreter but the
     lerobot venv exists, RE-EXECs the script with the venv python — so
     `ros2 run raise2026_labs 03_record.py` just works instead of dying with
     a misleading "lerobot not installed" (student report finding #3).
  4. Exposes DATASETS_DIR: the in-repo RAISE2026/datasets when discoverable,
     else ~/raise_datasets.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

VENV_PY = Path.home() / 'raise_venvs' / 'lerobot' / 'bin' / 'python3'

_HERE = Path(__file__).resolve().parent


def _find_day2_root() -> Path:
    """The directory that contains api_clients/ + task_packs/ in this install."""
    candidates = [_HERE]                              # source tree: this file lives in day2/
    # Installed-copy fallback: share/raise2026_labs/day2_lib (see setup.py).
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory('raise2026_labs')) / 'day2_lib')
    except Exception:
        pass
    # Last resort: walk up from this file looking for the marker dir.
    candidates += [p / 'day2' for p in _HERE.parents[:4]]
    for c in candidates:
        if (c / 'api_clients' / 'vla_client' / '__init__.py').is_file():
            return c
    raise ImportError(
        'RAISE 2026: could not locate the Day-2 libraries (api_clients/vla_client).\n'
        f'Searched: {[str(c) for c in candidates]}\n'
        'Fix: rebuild the workspace —\n'
        '    cd <repo>/RAISE2026/raise_ros2_ws && colcon build --symlink-install\n'
        '    source install/setup.bash')


DAY2 = _find_day2_root()


def _find_datasets_dir() -> Path:
    """The repo's RAISE2026/datasets when reachable, else ~/raise_datasets."""
    for start in (DAY2, Path(sys.prefix)):
        p = start
        for _ in range(7):
            cand = p / 'datasets'
            if cand.is_dir() and (p / 'raise_ros2_ws').is_dir():   # p == RAISE2026
                return cand
            p = p.parent
    return Path.home() / 'raise_datasets'


DATASETS_DIR = _find_datasets_dir()


def bootstrap(lerobot: bool = False) -> None:
    """Make Day-2 imports work; optionally re-exec into the lerobot venv."""
    for sub in (DAY2 / 'api_clients', DAY2 / 'task_packs' / 'common'):
        s = str(sub)
        if s not in sys.path:
            sys.path.insert(0, s)

    if not lerobot:
        return
    try:
        import lerobot  # noqa: F401  (works: venv python, or lerobot on this interpreter)
        return
    except ImportError:
        pass
    # Are we already INSIDE the lerobot venv? Compare sys.prefix, NOT the
    # executable: the venv's bin/python3 is a SYMLINK to /usr/bin/python3, so
    # resolved-binary comparison wrongly says "same interpreter" (found live).
    in_venv = VENV_PY.is_file() and Path(sys.prefix).resolve() == VENV_PY.parent.parent.resolve()
    if VENV_PY.is_file() and not in_venv:
        # Re-exec THIS script under the venv python (argv0 = the venv path so
        # CPython picks up pyvenv.cfg). PYTHONPATH is inherited, so the ROS
        # workspace stays importable inside the venv.
        script = str(Path(sys.argv[0]).resolve())
        os.execv(str(VENV_PY), [str(VENV_PY), script] + sys.argv[1:])
    if in_venv:
        sys.exit(
            'RAISE 2026: the lerobot venv exists but LeRobot will not import inside it.\n'
            'Reinstall it:\n'
            '    ~/raise_venvs/lerobot/bin/python3 -m pip install --timeout 30 --retries 10 "lerobot[smolvla]"')
    sys.exit(
        'RAISE 2026: this script needs LeRobot, and the lerobot venv is missing.\n'
        'Install it once (~3 GB — do it at home, not on school WiFi):\n'
        '    /usr/bin/python3 -m venv --system-site-packages ~/raise_venvs/lerobot\n'
        '    ~/raise_venvs/lerobot/bin/python3 -m pip install --timeout 30 --retries 10 "lerobot[smolvla]"')
