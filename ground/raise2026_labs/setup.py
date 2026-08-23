# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

import os
import stat
from glob import glob

from setuptools import setup
from setuptools.command.develop import develop as _develop_orig

package_name = 'raise2026_labs'


# ── Custom develop command — silences pkg_resources DeprecationWarning ────
# colcon's `--symlink-install` runs `python3 setup.py develop`, which by
# default generates dev-script wrappers that look like:
#
#     __import__('pkg_resources').require('raise2026-labs==0.1.0')
#     with open(SRC) as f: exec(compile(f.read(), SRC, 'exec'))
#
# The pkg_resources call has been deprecated upstream and emits a noisy
# DeprecationWarning on every script invocation — irritating in a class-
# room. We override `install_egg_scripts` to write a SIMPLER wrapper that
# just execs the source file. The package is still installed by the rest
# of develop, so `import raise2026_labs` still works.
class _SilentDevelop(_develop_orig):
    def install_egg_scripts(self, dist):
        scripts = self.distribution.scripts or []
        if not scripts:
            return super().install_egg_scripts(dist)
        os.makedirs(self.script_dir, exist_ok=True)
        for src in scripts:
            self._write_simple_wrapper(src)

    def _write_simple_wrapper(self, src_path: str) -> None:
        abs_src = os.path.abspath(src_path)
        name = os.path.basename(src_path)
        target = os.path.join(self.script_dir, name)
        # Shebang MUST be the absolute path to the Python ament_python was
        # built with (system /usr/bin/python3 on this machine — 3.12 in
        # Ubuntu 24.04 / ROS 2 Jazzy). Using `#!/usr/bin/env python3` would
        # pick up the FIRST python3 on PATH, which on dev machines is often
        # anaconda's Python 3.11 — but rclpy's C extension is built for
        # 3.12, so it fails with `ModuleNotFoundError: No module named
        # 'rclpy._rclpy_pybind11'`. See [[feedback-anaconda-colcon]].
        import sys as _sys
        python_for_shebang = _sys.executable or '/usr/bin/python3'
        with open(target, 'w') as f:
            f.write(
                f'#!{python_for_shebang}\n'
                '# RAISE 2026 — dev-script wrapper (no pkg_resources, no DeprecationWarning).\n'
                f'__file__ = {abs_src!r}\n'
                "with open(__file__) as _f:\n"
                "    exec(compile(_f.read(), __file__, 'exec'))\n"
            )
        os.chmod(target, os.stat(target).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

# The six lab slots for the 3-day school, grouped under per-day folders.
# Scripts still install by basename into lib/raise2026_labs/, so
# `ros2 run raise2026_labs <script>.py` is unaffected by the nesting.
LABS = [
    'day1/day1_01_ros2_tools_as_functions',
    'day1/day1_02_agentic_inspector',
    'day2/day2_01_teleoperation_and_data',
    'day2/day2_02_vla_executor',
    'day3/day3_01_full_stack',
    'day3/day3_02_hackathon',
]


def all_starter_scripts():
    """Return every starter Python script across all labs (as repo-relative paths)."""
    files = []
    for lab in LABS:
        files.extend(sorted(glob(f'{lab}/starter/*.py')))
    return files


def lab_readme_data_files():
    """Install each lab's README under share/raise2026_labs/<lab>/README.md."""
    files = []
    for lab in LABS:
        readme = glob(f'{lab}/README.md')
        if readme:
            files.append((f'share/{package_name}/{lab}', readme))
    return files


def day2_lib_data_files():
    """Ship the Day-2 shared libraries with the install (copied-build proof).

    The starters import vla_client/task_pack/sim_poses from the SOURCE tree via
    _d2paths.py. On installs where colcon COPIES sources (student report
    2026-07-11: ModuleNotFoundError: vla_client via `ros2 run`), the source
    tree may not be resolvable from the installed script — so we also install
    the libs under share/raise2026_labs/day2_lib/ and _d2paths falls back to
    them. _d2paths.py itself is installed NEXT TO the scripts (lib/…) so the
    starters can always `import _d2paths` first.
    """
    files = [(f'lib/{package_name}', ['day2/_d2paths.py'])]
    patterns = [
        'day2/_d2paths.py',
        'day2/api_clients/vla_client/*.py',
        'day2/task_packs/common/*.py',
        'day2/task_packs/task_*/*.yaml',
    ]
    for pat in patterns:
        for f in sorted(glob(pat)):
            rel_dir = os.path.dirname(f).replace('day2', 'day2_lib', 1)
            files.append((f'share/{package_name}/{rel_dir}', [f]))
    # de-duplicate targets (data_files allows repeated dirs, but keep it tidy)
    merged = {}
    for target, fl in files:
        merged.setdefault(target, []).extend(fl)
    return [(t, sorted(set(fl))) for t, fl in merged.items()]


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + lab_readme_data_files() + day2_lib_data_files(),
    # `scripts=` installs files into install/<pkg>/lib/<pkg>/ as executables —
    # which is exactly where `ros2 run <pkg> <name>` looks. Tab-completion picks
    # them up automatically.
    scripts=all_starter_scripts(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anis Koubaa',
    maintainer_email='anis.koubaa@gmail.com',
    description='RAISE 2026 lab starter scripts.',
    license='MIT',
    entry_points={'console_scripts': []},
    cmdclass={'develop': _SilentDevelop},
)
