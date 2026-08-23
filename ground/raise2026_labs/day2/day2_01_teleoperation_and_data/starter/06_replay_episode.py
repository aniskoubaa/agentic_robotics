#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Lab 2.1, Script 06 — REPLAY a recorded episode into Gazebo.

WHAT  Take one episode you captured (by teleop or the auto-demonstrator) and play
      its recorded ACTION stream straight back to the sim arm. The robot redoes
      the exact motion you recorded — an "open-loop replay".

WHY   This is the cheapest, most honest data check there is. BEFORE you spend
      hours training, replay a few episodes and watch:
        • smooth, sensible motion that ends in a pick  → your data is good.
        • jerky / wrong / no-grasp motion              → fix the demos, not the model.
      A VLA can only be as good as what you replay here. Garbage in, garbage out —
      and this is where you catch the garbage.

HOW   We read the episode's `action` column (6 joint targets + gripper) from the
      LeRobot dataset and publish each one to /ur5e_*/cmd (+ gripper, with the
      mimic expansion) at the recorded frame rate — reusing the SAME canonical
      Action the VLA executor uses, so replay and real execution drive the arm
      identically.

      With --spawn, it also recreates the scene: it finds the frame where the
      gripper closes, drives there first to read where the tool lands, spawns the
      red (+ green) tomato there, then replays — so the GRASP reproduces too
      (grasp_server must be running).

PREREQUISITES:  the sim (raise-sim).  For the grasp to reproduce, also:
    ros2 run raise2026_tools grasp_server      # (alias: grasp_d3)

Usage:
    # replay episode 0 of your dataset (motion only):
    ros2 run raise2026_labs 06_replay_episode.py --task C --team team07 --hf-user me --episode 0
    # replay AND recreate the tomatoes so the pick reproduces:
    ros2 run raise2026_labs 06_replay_episode.py --task C --team team07 --hf-user me --episode 0 --spawn
    # (alias: 06_d3 ...)
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformListener

try:
    import _d2paths                    # installed next to the ros2-run scripts
except ImportError:                    # running straight from the source tree
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    import _d2paths
_d2paths.bootstrap(lerobot=True)
DAY2 = _d2paths.DAY2
from vla_client.base import Action, UR5E_JOINTS, GRIPPER_KNUCKLE, GRIPPER_MIMIC_SIGNS, GRIPPER_CLOSED   # noqa: E402
from raise2026_tools import gz_utils   # noqa: E402

REPO_SLUG = {'A': 'raise_tabletop_pick', 'C': 'raise_ripeness_sort', 'B': 'raise_mobile_manip'}


class Replayer(Node):
    def __init__(self, world, gripper_link):
        super().__init__('episode_replayer')
        self.world = world
        self.gripper_link = gripper_link
        all_joints = UR5E_JOINTS + [GRIPPER_KNUCKLE] + list(GRIPPER_MIMIC_SIGNS)
        self.pubs = {j: self.create_publisher(Float64, f'/{j}/cmd', 10) for j in all_joints}
        self.register_pub = self.create_publisher(String, '/grasp/register', 10)
        # Frames: TF (base->gripper) + robot world pose from gz — same math as
        # grasp_server / 05_auto_demonstrate (the Pose_V bridge drops names).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def spin(self, secs):
        end = time.time() + secs
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def tool_point(self, grasp_offset: float = 0.13):
        try:
            t = self.tf_buffer.lookup_transform('base_link', self.gripper_link,
                                                rclpy.time.Time())
        except Exception:
            return None
        wtb = gz_utils.get_model_world_pose('raise2026_robot', self.world)
        if wtb is None:
            return None
        p, q = t.transform.translation, t.transform.rotation
        off = gz_utils.rotate_vec((q.x, q.y, q.z, q.w), (0.0, 0.0, grasp_offset))
        p_base = (p.x + off[0], p.y + off[1], p.z + off[2])
        (bx, by, bz), bq = wtb
        w = gz_utils.rotate_vec(bq, p_base)
        return (bx + w[0], by + w[1], bz + w[2])

    def send_action(self, joints, gripper):
        """Publish one recorded action exactly like the VLA executor would."""
        action = Action(joints=list(joints), gripper=float(gripper))
        for joint, target in action.joint_commands():
            self.pubs[joint].publish(Float64(data=float(target)))

    def goto(self, joints, gripper, settle_s=3.0):
        """Drive to a pose for the SETUP phase: republish the target while
        settling. A single publish right after node creation is dropped before
        ROS discovery connects the publishers (found in the live replay test)."""
        end = time.time() + settle_s
        while rclpy.ok() and time.time() < end:
            self.send_action(joints, gripper)
            self.spin(0.2)


def load_episode_actions(repo_id, root, episode):
    """Return (actions, fps): actions is a list of 7-vectors (6 joints + gripper)."""
    try:                                        # LeRobot moved this module between releases
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset(repo_id, root=str(root))
    fps = getattr(ds, 'fps', 10) or 10
    # Frame index range for this episode.
    try:
        lo = int(ds.episode_data_index['from'][episode].item())
        hi = int(ds.episode_data_index['to'][episode].item())
        idxs = range(lo, hi)
    except (AttributeError, KeyError, IndexError):
        # Fallback: filter the underlying HF dataset by episode_index.
        hf = ds.hf_dataset
        idxs = [i for i, e in enumerate(hf['episode_index']) if int(e) == episode]
    actions = []
    for i in idxs:
        a = ds[i]['action']
        actions.append(np.asarray(a).reshape(-1).tolist())
    return actions, fps


def main():
    ap = argparse.ArgumentParser(description='RAISE 2026 episode replayer')
    ap.add_argument('--task', default='C', help='A | C | B (picks the dataset name)')
    ap.add_argument('--team', default='team00')
    ap.add_argument('--hf-user', default='raiseschool')
    ap.add_argument('--episode', type=int, default=0)
    ap.add_argument('--data-root', default=str(_d2paths.DATASETS_DIR),
                    help='datasets live INSIDE the repo (RAISE2026/datasets) so everything is versioned together')
    ap.add_argument('--spawn', action='store_true', help='recreate the tomatoes so the grasp reproduces')
    ap.add_argument('--world', default=gz_utils.DEFAULT_WORLD)
    ap.add_argument('--gripper-link', default='gripper_mount_link')
    ap.add_argument('--speed', type=float, default=1.0, help='replay speed multiplier')
    args = ap.parse_args()

    repo_id = f'{args.hf_user}/{REPO_SLUG[args.task.upper()]}_{args.team}'
    root = Path(args.data_root) / repo_id.replace('/', '__')
    if not root.is_dir():
        print(f'✗ no dataset at {root}. Record (03/05) first.')
        sys.exit(1)

    try:
        actions, fps = load_episode_actions(repo_id, root, args.episode)
    except ImportError:
        print('✗ lerobot not installed. pip install "lerobot[smolvla]" "numpy<2"')
        sys.exit(1)
    if not actions:
        print(f'✗ episode {args.episode} has no frames (or index out of range).')
        sys.exit(1)

    print(f'Replaying {repo_id} episode {args.episode}: {len(actions)} frames @ {fps} Hz'
          f'{"  (x%.1f)" % args.speed if args.speed != 1 else ""}')

    rclpy.init()
    node = Replayer(args.world, args.gripper_link)
    dt = 1.0 / (fps * args.speed)
    spawned = []
    try:
        if args.spawn:
            # Find the frame where the gripper first closes — the grasp moment.
            grasp_i = next((i for i, a in enumerate(actions) if a[6] > 0.6 * GRIPPER_CLOSED), None)
            if grasp_i is None:
                print('  (no gripper-close in this episode; replaying motion only)')
            else:
                print(f'  recreating scene at grasp frame {grasp_i} ...')
                # Rebuild the TRUE recording geometry: measure BOTH canonical
                # grasp points (same shared poses the demonstrator used), find
                # which one the episode's grasp frame lands on -> RED goes
                # there, GREEN at the OTHER point. (A naive "+18 cm offset"
                # for green parked it on the descend path and the replay
                # grasped the wrong tomato — found live.)
                from sim_poses import GRASP_LEFT, GRASP_RIGHT   # shared with 05
                node.goto(GRASP_LEFT, 0.0)
                lp = node.tool_point()
                node.goto(GRASP_RIGHT, 0.0)
                rp = node.tool_point()
                node.goto(actions[grasp_i][:6], 0.0)            # the episode's grasp pose
                pt = node.tool_point()
                # return to the episode's STARTING pose BEFORE spawning —
                # the retracting arm would sweep through a fresh tomato.
                node.goto(actions[0][:6], 0.0)
                if pt is None or lp is None or rp is None:
                    print(f'  ⚠ could not read tool poses ({args.gripper_link}); skipping spawn.')
                else:
                    d = lambda a, b: sum((a[i] - b[i]) ** 2 for i in range(3)) ** 0.5
                    red_pt, green_pt = (lp, rp) if d(pt, lp) < d(pt, rp) else (rp, lp)
                    gz_utils.spawn_model('tomato_red_0', 'model://tomato_red', *red_pt, world=args.world)
                    gz_utils.spawn_model('tomato_green_0', 'model://tomato_green', *green_pt, world=args.world)
                    node.spin(0.4)
                    node.register_pub.publish(String(data='tomato_red_0'))
                    node.register_pub.publish(String(data='tomato_green_0'))
                    node.spin(2.5)   # let grasp_server's slow sync resolve the poses
                    spawned = ['tomato_red_0', 'tomato_green_0']
                    print(f'  red at {tuple(round(v,3) for v in red_pt)}, green at the other point')

        print('  ▶ replaying ...')
        for i, a in enumerate(actions):
            node.send_action(a[:6], a[6])
            node.spin(dt)
            if i % fps == 0:
                print(f'\r    frame {i + 1}/{len(actions)}   ', end='', flush=True)
        print('\n  ✓ replay done.')
    except KeyboardInterrupt:
        pass
    finally:
        for n in spawned:
            gz_utils.remove_model(n, world=args.world)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
