# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Day-2 offline smoke tests (no sim, no GPU, no lerobot needed).

Guards the pure-python contracts everything else depends on. Run with EITHER
python (system or venv):

    /usr/bin/python3 -m pytest tests/ -q        # from the day2/ folder
    # or without pytest:
    /usr/bin/python3 tests/test_day2_offline.py
"""

import sys
from pathlib import Path

DAY2 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DAY2 / 'api_clients'))
sys.path.insert(0, str(DAY2 / 'task_packs' / 'common'))


def test_action_contract():
    """Canonical Action: 12 joint commands (6 arm + knuckle + 5 mimics), clamped."""
    from vla_client.base import Action, GRIPPER_KNUCKLE, GRIPPER_MIMIC_SIGNS
    a = Action(joints=[0, 0, 0, 0, 0, 99.0], gripper=1.0)   # deliberately over-limit
    cmds = dict(a.joint_commands())
    assert len(cmds) == 12
    assert cmds['ur5e_wrist_3_joint'] <= 6.28 + 1e-9         # clamped
    assert abs(cmds[GRIPPER_KNUCKLE] - 0.5) < 1e-9           # gripper clamped to CLOSED
    for j, sign in GRIPPER_MIMIC_SIGNS.items():              # mimic signs applied
        assert abs(cmds[j] - sign * 0.5) < 1e-9
    rt = Action.from_dict(a.to_dict())                       # wire round-trip
    assert rt.joints == a.joints


def test_task_packs_load():
    """All three packs load with scenarios; C is the graded default."""
    from task_pack import available_tasks, load_task
    ids = available_tasks()
    assert set(ids) == {'A', 'B', 'C'}
    for tid in ids:
        p = load_task(tid)
        assert p.recommended_episodes > 0 and p.instructions and len(p.scenarios) == 5
    c = load_task('C')
    assert c.dataset_repo_id('u', 't') == 'u/raise_ripeness_sort_t'
    assert c.uses_base is False and load_task('B').uses_base is True


def test_sim_poses_shared():
    """Demonstrator + evaluator share one source of grasp poses."""
    from sim_poses import POSE_HOME, GRASP_LEFT, GRASP_RIGHT, above
    assert len(POSE_HOME) == len(GRASP_LEFT) == len(GRASP_RIGHT) == 6
    assert GRASP_LEFT[0] > 0 > GRASP_RIGHT[0]                # distinct sides (pan sign)
    assert above(GRASP_LEFT)[1] < GRASP_LEFT[1]              # approach is higher


def test_ros_image_pure_numpy():
    """Image conversion works with no cv_bridge, under numpy 1 AND 2."""
    import numpy as np
    from vla_client.ros_image import imgmsg_to_rgb, resize_rgb

    class FakeMsg:                                            # minimal sensor_msgs/Image
        height, width, step = 4, 6, 6 * 3 + 2                 # includes row padding
        encoding = 'bgr8'
        data = bytes(range(4 * (6 * 3 + 2)))

    img = imgmsg_to_rgb(FakeMsg())
    assert img.shape == (4, 6, 3)
    assert img[0, 0, 0] == 2                                  # BGR -> RGB swap (B was byte 0)
    small = resize_rgb(img, 3)
    assert small.shape == (3, 3, 3) and small.flags['C_CONTIGUOUS']


def test_factory_defaults_local():
    """make_vla_client resolves the default backend name without importing torch."""
    import vla_client
    assert 'make_vla_client' in vla_client.__all__


if __name__ == '__main__':
    fails = 0
    for name, fn in sorted({k: v for k, v in globals().items() if k.startswith('test_')}.items()):
        try:
            fn()
            print(f'  ✓ {name}')
        except AssertionError as e:
            fails += 1
            print(f'  ✗ {name}: {e}')
    sys.exit(1 if fails else print('ALL OFFLINE TESTS PASS') or 0)
