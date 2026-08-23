#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Lab 2.1, Script 05 — AUTO-DEMONSTRATOR ("pick the red tomato").

WHAT  Generates pick demonstrations WITHOUT you teleoperating. A script drives
      the arm through a clean pick of the RED (ripe) tomato while leaving the
      green one, and records each rollout as a LeRobot episode — the same data
      03_record.py would produce by hand, but dozens of clean episodes hands-free.

WHY   Jogging six joints by keyboard to pick a tomato is painful, and the sim has
      no working grasp physics or IK. This script sidesteps both:
        • GRASP: the grasp_server (run it too!) makes the tomato stick to the
          gripper when it closes — deterministic, no physics tuning.
        • REACH (no IK): we move the arm to a chosen joint pose, see where the
          gripper tool ends up (from /gz_world_poses, world frame), and spawn the
          tomato THERE. So every grasp lands by construction.
      The model still has to learn the task from the CAMERA + the words "red
      tomato" — it never sees the spawn coordinates — so the learning is real.

      For language grounding, the red tomato is placed on the LEFT or RIGHT at
      random each episode (green takes the other side); the scripted demo picks
      whichever side is red. The policy must learn to follow "red", not a fixed
      motion.

PREREQUISITES (three things must be running):
    1. the sim:           raise-sim
    2. the grasp server:  ros2 run raise2026_tools grasp_server
    3. this script:       ros2 run raise2026_labs 05_auto_demonstrate.py --episodes 30
                          (or:  05_d3 --episodes 30)

⚠ TUNE IN THE LIVE SIM (cannot be verified offline): the LEFT/RIGHT grasp joint
  poses below, the gripper link name, and the grasp_server's attach_radius. The
  script prints the pose-frame names it sees on first run to help you set them.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.parameter import Parameter as RclpyParameter
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformListener

try:
    import _d2paths                    # installed next to the ros2-run scripts
except ImportError:                    # running straight from the source tree
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    import _d2paths
_d2paths.bootstrap(lerobot=True)
DAY2 = _d2paths.DAY2
from vla_client.base import (   # noqa: E402
    UR5E_JOINTS, GRIPPER_KNUCKLE, GRIPPER_MIMIC_SIGNS, GRIPPER_OPEN, GRIPPER_CLOSED,
)
from vla_client.ros_image import imgmsg_to_rgb, resize_rgb   # noqa: E402  (no cv_bridge — numpy-2 safe)
from raise2026_tools import gz_utils   # noqa: E402

STATE_JOINTS = UR5E_JOINTS + [GRIPPER_KNUCKLE]
IMG_SIZE = 224
FPS = 10                       # control + record rate
GRIPPER_LINK = 'gripper_mount_link'

# The tuned grasp poses are SHARED with the Lab-2.2 evaluator (it must test on
# the same reachable points the demos were recorded at) — one source of truth:
from sim_poses import POSE_HOME, GRASP_LEFT, GRASP_RIGHT, above as _above, check_parking, snap_to_park   # noqa: E402


class Demonstrator(Node):
    def __init__(self, world: str, gripper_link: str):
        super().__init__('auto_demonstrator')
        self.set_parameters([RclpyParameter('use_sim_time', RclpyParameter.Type.BOOL, True)])
        self.world = world
        self.gripper_link = gripper_link

        # Publishers: 6 arm joints + gripper knuckle + its 5 mimic siblings.
        self._all_joints = STATE_JOINTS + list(GRIPPER_MIMIC_SIGNS)
        self.pubs = {j: self.create_publisher(Float64, f'/{j}/cmd', 10) for j in self._all_joints}
        self.register_pub = self.create_publisher(String, '/grasp/register', 10)

        # Frames: TF gives base_link->gripper live; gz gives world->base (the
        # ros_gz Pose_V bridge drops entity names, so we read gz directly).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.measured = {}         # joint -> position
        self.latest_img = None

        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(Image, '/wrist_camera/image_raw', self._on_image, 10)
        # grasp_server tells us what it's holding — we use it to verify each
        # episode actually grasped (no failed-grasp garbage in the dataset).
        self.grasp_state = 'none'
        self.create_subscription(String, '/grasp/state',
                                 lambda m: setattr(self, 'grasp_state', m.data), 10)

        self.frames = []           # current episode's recorded frames
        self.recording = False

    # ── sensor callbacks ─────────────────────────────────────────────────────
    def _on_joints(self, msg: JointState):
        self.measured = dict(zip(msg.name, msg.position))

    def _on_image(self, msg: Image):
        # Pure-numpy conversion + resize (no cv_bridge/cv2 — numpy-2 safe).
        self.latest_img = resize_rgb(imgmsg_to_rgb(msg), IMG_SIZE)

    # ── helpers ──────────────────────────────────────────────────────────────
    def spin(self, secs: float):
        # SIM-time wait (/clock): with the Gazebo GUI open the sim may run
        # below 100% real-time; wall-clock pacing would then record frames
        # too fast relative to the physics (10 Hz wall != 10 Hz sim).
        start = self.get_clock().now()
        dur = RclpyDuration(seconds=secs)
        while rclpy.ok() and (self.get_clock().now() - start) < dur:
            rclpy.spin_once(self, timeout_sec=0.02)

    def tool_point(self, grasp_offset: float = 0.13):
        """World (x,y,z) of the grasp point between the fingers, or None.

        Same frame math as grasp_server: TF's base_link->gripper (exact, live)
        composed with the robot model's world pose read from gz. The offset
        pushes the point from the gripper mount out to the finger gap.
        """
        try:
            t = self.tf_buffer.lookup_transform('base_link', self.gripper_link,
                                                rclpy.time.Time())
        except Exception:
            self.get_logger().warn(f'TF base_link->{self.gripper_link} not available yet')
            return None
        wtb = gz_utils.get_model_world_pose('raise2026_robot', self.world)
        if wtb is None:
            self.get_logger().warn('could not read robot world pose from gz')
            return None
        p, q = t.transform.translation, t.transform.rotation
        off = gz_utils.rotate_vec((q.x, q.y, q.z, q.w), (0.0, 0.0, grasp_offset))
        p_base = (p.x + off[0], p.y + off[1], p.z + off[2])
        (bx, by, bz), bq = wtb
        w = gz_utils.rotate_vec(bq, p_base)
        return (bx + w[0], by + w[1], bz + w[2])

    def publish_gripper(self, value: float):
        self.pubs[GRIPPER_KNUCKLE].publish(Float64(data=value))
        for j, sign in GRIPPER_MIMIC_SIGNS.items():
            self.pubs[j].publish(Float64(data=sign * value))

    def move_to(self, joints, gripper, duration=1.5, record=False):
        """Interpolate from the current measured pose to `joints` and publish.

        When `record` is True, append a synced (image, state, action) frame each
        step — action is the joint+gripper target we command (true behavioral
        cloning labels).
        """
        start = [self.measured.get(j, joints[i]) for i, j in enumerate(UR5E_JOINTS)]
        steps = max(1, int(duration * FPS))
        for s in range(1, steps + 1):
            a = s / steps
            target = [start[i] + a * (joints[i] - start[i]) for i in range(6)]
            for j, v in zip(UR5E_JOINTS, target):
                self.pubs[j].publish(Float64(data=v))
            self.publish_gripper(gripper)
            if record:
                self._record_frame(target, gripper)
            self.spin(1.0 / FPS)

    def _record_frame(self, target_joints, gripper):
        state = [self.measured.get(j, 0.0) for j in STATE_JOINTS]
        if self.latest_img is None or any(np.isnan(state)):
            return
        action = list(target_joints) + [gripper]
        self.frames.append({
            'observation.images.wrist': self.latest_img.copy(),
            'observation.state': np.array(state, dtype=np.float32),
            'action': np.array(action, dtype=np.float32),
        })

    def register_graspable(self, name):
        self.register_pub.publish(String(data=name))

    def spawn_tomato(self, color: str, name: str, xyz):
        gz_utils.spawn_model(name, f'model://tomato_{color}', xyz[0], xyz[1], xyz[2],
                             world=self.world)
        self.spin(0.3)
        self.register_graspable(name)

    def cleanup(self, names):
        for n in names:
            gz_utils.remove_model(n, world=self.world)
        self.spin(0.3)


def make_features():
    return {
        'observation.images.wrist': {'dtype': 'image', 'shape': (IMG_SIZE, IMG_SIZE, 3),
                                     'names': ['height', 'width', 'channel']},
        'observation.state': {'dtype': 'float32', 'shape': (len(STATE_JOINTS),), 'names': STATE_JOINTS},
        'action': {'dtype': 'float32', 'shape': (len(STATE_JOINTS),), 'names': STATE_JOINTS},
    }


def main():
    ap = argparse.ArgumentParser(description='RAISE 2026 auto-demonstrator (pick the red tomato)')
    ap.add_argument('--episodes', type=int, default=30)
    ap.add_argument('--row', default='', help='optional row label woven into the instruction')
    ap.add_argument('--world', default=gz_utils.DEFAULT_WORLD)
    ap.add_argument('--gripper-link', default=GRIPPER_LINK)
    ap.add_argument('--team', default='team00')
    ap.add_argument('--hf-user', default='raiseschool')
    ap.add_argument('--data-root', default=str(_d2paths.DATASETS_DIR),
                    help='datasets live INSIDE the repo (RAISE2026/datasets) so everything is versioned together')
    ap.add_argument('--dry-run', action='store_true',
                    help='run the full pick choreography but save NOTHING — '
                         'use it to check motion/grasp before recording for real')
    args = ap.parse_args()

    instruction = (f'pick the red tomato in row {args.row}' if args.row
                   else 'pick the red tomato')
    repo_id = f'{args.hf_user}/raise_ripeness_sort_{args.team}'
    root = Path(args.data_root) / repo_id.replace('/', '__')

    dataset = None
    if not args.dry_run:
        try:
            # LeRobot moved this module between releases — try both paths.
            try:
                from lerobot.datasets.lerobot_dataset import LeRobotDataset
            except ImportError:
                from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        except ImportError:
            print('✗ lerobot not installed. pip install "lerobot[smolvla]" "numpy<2"')
            sys.exit(1)

    print(f'Auto-demonstrator → "{instruction}"' + ('   [DRY RUN — not saving]' if args.dry_run else ''))
    print(f'Dataset: {repo_id}  ({root})')
    print('Make sure the sim AND grasp_server are running.\n')

    if not args.dry_run:
        dataset = LeRobotDataset.create(repo_id=repo_id, fps=FPS, features=make_features(),
                                        robot_type='ur5e', root=str(root))
    rclpy.init()
    node = Demonstrator(args.world, args.gripper_link)

    # Discover the two grasp world points via the no-IK trick: go to each grasp
    # pose, read where the tool lands. Done once (base is stationary).
    # Deterministic geometry: snap the base to the canonical PARK pose first
    # (spawn settle varies run-to-run — enough to drift the camera views).
    snap_to_park(gz_utils, world=args.world)
    print('Locating reachable grasp points (no-IK trick) ...')
    # Generous settle time: the horizontal greenhouse poses swing far from
    # home, and reading the tool point BEFORE the arm has settled spawns the
    # tomato in the wrong place (live-found: episode-1 attach failure).
    node.move_to(GRASP_LEFT, GRIPPER_OPEN, duration=3.5)
    node.spin(2.5)
    left_pt = node.tool_point()
    node.move_to(GRASP_RIGHT, GRIPPER_OPEN, duration=3.5)
    node.spin(2.5)
    right_pt = node.tool_point()
    node.move_to(POSE_HOME, GRIPPER_OPEN, duration=2.5)
    if left_pt is None or right_pt is None:
        print('✗ could not compute the tool point (TF base_link->gripper, or the')
        print('  robot world pose from gz). Is the sim running? Check --gripper-link.')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    print(f'  left grasp point  : {tuple(round(v,3) for v in left_pt)}')
    print(f'  right grasp point : {tuple(round(v,3) for v in right_pt)}\n')
    # The trained model + these poses expect the plant-row parking (sim_d2).
    check_parking(lambda: gz_utils.get_model_world_pose('raise2026_robot', args.world))

    saved = 0
    try:
        for ep in range(args.episodes):
            # Randomize which side is red (vary the index, not a RNG — Math.random
            # equivalents aren't needed; alternate + small jitter gives diversity).
            red_left = (ep % 2 == 0)
            red_pt, green_pt = (left_pt, right_pt) if red_left else (right_pt, left_pt)
            red_pose = GRASP_LEFT if red_left else GRASP_RIGHT

            node.spawn_tomato('red', 'tomato_red_0', red_pt)
            node.spawn_tomato('green', 'tomato_green_0', green_pt)
            node.move_to(POSE_HOME, GRIPPER_OPEN, duration=1.5)

            # ── record the scripted SCAN-then-pick of the RED tomato ─────────
            # The camera rides the wrist, so the demo must LOOK before it acts:
            # hover above the LEFT spot first. If the tomato there is red, go
            # down; if it's green, pan to the RIGHT spot and go down there.
            # Every frame is decidable from the image alone ("red below me →
            # descend"), which is exactly what a single-frame policy can learn.
            # (A no-scan variant hid the target behind the gripper and the
            # policy plateaued at 50% = chance — see sim_poses.py.)
            node.frames = []
            node.move_to(_above(GRASP_LEFT), GRIPPER_OPEN, duration=1.5, record=True)   # scan LEFT
            node.move_to(_above(GRASP_LEFT), GRIPPER_OPEN, duration=0.6, record=True)   # dwell (look!)
            if not red_left:
                # green below us -> keep scanning: pan to the RIGHT spot
                node.move_to(_above(GRASP_RIGHT), GRIPPER_OPEN, duration=1.5, record=True)
                node.move_to(_above(GRASP_RIGHT), GRIPPER_OPEN, duration=0.6, record=True)
            node.move_to(red_pose,        GRIPPER_OPEN, duration=1.2, record=True)   # descend on RED
            node.move_to(red_pose,        GRIPPER_CLOSED, duration=0.8, record=True) # close => grasp
            node.spin(0.3)                                                           # let grasp_server latch
            grasped = node.grasp_state == 'tomato_red_0'
            node.move_to(_above(red_pose), GRIPPER_CLOSED, duration=1.2, record=True) # lift
            place = list(_above(red_pose)); place[0] += 0.6                          # swing to bin
            node.move_to(place,           GRIPPER_CLOSED, duration=1.5, record=True)
            node.move_to(place,           GRIPPER_OPEN, duration=0.8, record=True)   # release

            n = len(node.frames)
            side = 'L' if red_left else 'R'
            if not grasped:
                print(f'  ✗ episode {ep + 1}: grasp did NOT attach (state={node.grasp_state}) — discarded. '
                      f'Is grasp_server running? Tune attach_radius / grasp poses.')
            elif n < FPS:
                print(f'  ⚠ skipped episode {ep + 1} (only {n} frames — streams not ready?)')
            elif args.dry_run:
                saved += 1
                print(f'  ✓ [dry] episode {saved}/{args.episodes}  ({n} frames, red={side}, grasp OK)')
            else:
                for fr in node.frames:
                    # lerobot >= 0.5: the task string is a FIELD of the frame.
                    dataset.add_frame({**fr, 'task': instruction})
                dataset.save_episode()
                saved += 1
                print(f'  ✓ episode {saved}/{args.episodes}  ({n} frames, red={side}, grasp OK)')

            node.cleanup(['tomato_red_0', 'tomato_green_0'])
    except KeyboardInterrupt:
        pass
    finally:
        if dataset is not None:
            try:
                dataset.finalize()
            except Exception:
                pass
        node.cleanup(['tomato_red_0', 'tomato_green_0'])
        node.destroy_node()
        rclpy.shutdown()
        if args.dry_run:
            print(f'\n[dry run] {saved} episodes rehearsed OK — nothing saved. '
                  f'Re-run without --dry-run to record.')
        else:
            print(f'\nDone. {saved} episodes in {root}.')
            print(f'Train on it:  finetune_d4 --task C --team {args.team} --hf-user {args.hf_user} --launch')


if __name__ == '__main__':
    main()
