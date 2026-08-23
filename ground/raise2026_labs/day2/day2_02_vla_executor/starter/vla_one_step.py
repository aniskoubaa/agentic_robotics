#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Lab 2.2 warm-up: ONE call to the VLA, with every input and the
output made visible.

The objective of this script is to demystify the "brain". A VLA is just a
function:

        action = model( image , instruction , state )

Three inputs, one output — that's it. This script grabs those three inputs
LIVE from the simulator, calls the model ONCE, and prints exactly what went
in and what came out (plus a picture you can open: the camera frame next to
the numbers). Run it BEFORE vla_executor.py, so when you later watch the
full rollout you know precisely what happens on every one of its steps:
the executor is nothing more than this script inside a 10 Hz loop.

    # sim (sim_d2) + grasp_server running, then:
    vla_one_d4 --spawn                      # place tomatoes, 1 call, explain it
    vla_one_d4 --spawn --execute            # ...and actually SEND the action
    vla_one_d4 --steps 5                    # 5 calls in a row (see the chunk)

No joystick, no training required — it uses the reference checkpoint the
executor uses (or VLA_LOCAL_CKPT if you exported one).
"""

import argparse
import sys
import time
from pathlib import Path

# --- make the Day-2 imports work everywhere (source tree OR `ros2 run`),
#     and re-exec into the lerobot venv if needed (the model needs numpy 2) ---
try:
    import _d2paths
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    import _d2paths
_d2paths.bootstrap(lerobot=True)
DAY2 = _d2paths.DAY2

import numpy as np                                                        # noqa: E402
import rclpy                                                              # noqa: E402
from rclpy.node import Node                                               # noqa: E402
from sensor_msgs.msg import Image, JointState                            # noqa: E402
from std_msgs.msg import Float64                                          # noqa: E402
from tf2_ros import Buffer, TransformListener                             # noqa: E402

from vla_client import make_vla_client                                    # noqa: E402
from vla_client.base import (                                             # noqa: E402
    UR5E_JOINTS, GRIPPER_KNUCKLE, GRIPPER_MIMIC_SIGNS, GRIPPER_OPEN,
)
from vla_client.ros_image import imgmsg_to_rgb, resize_rgb                # noqa: E402
from sim_poses import POSE_HOME, GRASP_LEFT, GRASP_RIGHT, snap_to_park    # noqa: E402
from raise2026_tools import gz_utils                                      # noqa: E402

STATE_JOINTS = UR5E_JOINTS + [GRIPPER_KNUCKLE]   # the 7 numbers of "state"
IMG_SIZE = 224                                   # SmolVLA sees 224×224 RGB
# short labels so the printed table stays narrow
LABELS = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2',
          'wrist_3', 'gripper']


class OneStepIO(Node):
    """Tiny ROS node: subscribe to the camera + joints, publish joint targets."""

    def __init__(self):
        super().__init__('vla_one_step')
        self.latest_img = None      # the model's EYES
        self.measured = {}          # the model's PROPRIOCEPTION (where am I?)
        self.create_subscription(Image, '/wrist_camera/image_raw',
                                 self._on_image, 10)
        self.create_subscription(JointState, '/joint_states', self._on_state, 10)
        all_joints = STATE_JOINTS + list(GRIPPER_MIMIC_SIGNS)
        self.pubs = {j: self.create_publisher(Float64, f'/{j}/cmd', 10)
                     for j in all_joints}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_image(self, msg):
        # raw sim frame → RGB numpy → the exact 224×224 the model was trained on
        self.latest_img = resize_rgb(imgmsg_to_rgb(msg), IMG_SIZE)

    def _on_state(self, msg):
        self.measured = dict(zip(msg.name, msg.position))

    def spin(self, secs):
        end = time.time() + secs
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def state_vec(self):
        """The 7-float state vector — exactly what the dataset recorded."""
        if not all(j in self.measured for j in STATE_JOINTS):
            return None
        return np.array([self.measured[j] for j in STATE_JOINTS],
                        dtype=np.float32)

    def publish_arm(self, joints, gripper):
        for j, v in zip(UR5E_JOINTS, joints):
            self.pubs[j].publish(Float64(data=float(v)))
        self.pubs[GRIPPER_KNUCKLE].publish(Float64(data=float(gripper)))
        for j, sign in GRIPPER_MIMIC_SIGNS.items():
            self.pubs[j].publish(Float64(data=float(sign * gripper)))

    def goto(self, joints, gripper, settle_s=2.5):
        """Scripted setup move (NOT the model — just scene preparation)."""
        end = time.time() + settle_s
        while rclpy.ok() and time.time() < end:
            self.publish_arm(joints, gripper)
            self.spin(0.15)

    def tool_point(self, grasp_offset=0.13):
        """Where the gripper's grasp point is in WORLD coordinates (TF + gz)."""
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'gripper_mount_link',
                                                rclpy.time.Time())
        except Exception:
            return None
        wtb = gz_utils.get_model_world_pose('raise2026_robot')
        if wtb is None:
            return None
        p, q = t.transform.translation, t.transform.rotation
        off = gz_utils.rotate_vec((q.x, q.y, q.z, q.w), (0.0, 0.0, grasp_offset))
        pb = (p.x + off[0], p.y + off[1], p.z + off[2])
        (bx, by, bz), bq = wtb
        w = gz_utils.rotate_vec(bq, pb)
        return (bx + w[0], by + w[1], bz + w[2])


def _locate(node, pose, tries=4):
    """goto + measure, retrying — right after sim start, TF and the gz
    services need a few seconds before the first lookup succeeds."""
    for attempt in range(tries):
        node.goto(pose, GRIPPER_OPEN, settle_s=2.5 if attempt == 0 else 1.5)
        node.spin(0.5)
        pt = node.tool_point()
        if pt is not None:
            return pt
    return None


def spawn_scene(node, red_left=True):
    """Place a red + green tomato at the two trained grasp points."""
    snap_to_park(gz_utils)                       # exact training geometry first
    print('  (setup) locating the two grasp points with the arm ...')
    left_pt = _locate(node, GRASP_LEFT)
    right_pt = _locate(node, GRASP_RIGHT)
    if left_pt is None or right_pt is None:
        # say WHICH half is broken, so the fix is obvious
        if gz_utils.get_model_world_pose('raise2026_robot') is None:
            print('✗ cannot reach the simulator (gz). Start it first:  sim_d2')
        else:
            print('✗ sim is up but no TF from the robot yet — wait a few '
                  'seconds after sim start and re-run (ros2 topic echo /tf '
                  'should stream).')
        sys.exit(1)
    red_pt, green_pt = (left_pt, right_pt) if red_left else (right_pt, left_pt)
    for name in ('tomato_red_0', 'tomato_green_0'):
        gz_utils.remove_model(name)          # clear leftovers from earlier runs
    gz_utils.spawn_model('tomato_red_0', 'model://tomato_red', *red_pt)
    gz_utils.spawn_model('tomato_green_0', 'model://tomato_green', *green_pt)
    node.goto(POSE_HOME, GRIPPER_OPEN)           # back to where episodes start
    print(f'  (setup) red at {"LEFT" if red_left else "RIGHT"}, arm at HOME.\n')


def save_io_card(img, instruction, state, action, out_path):
    """One PNG: the camera frame next to the numbers — the whole call at a glance."""
    from PIL import Image as PImage, ImageDraw
    scale = 2
    cam = PImage.fromarray(img).resize((IMG_SIZE * scale, IMG_SIZE * scale),
                                       PImage.NEAREST)
    W, H = cam.width + 440, cam.height
    card = PImage.new('RGB', (W, H), (24, 26, 30))
    card.paste(cam, (0, 0))
    d = ImageDraw.Draw(card)
    x, y = cam.width + 16, 12

    def line(txt, color=(230, 230, 230), dy=17):
        nonlocal y
        d.text((x, y), txt, fill=color)
        y += dy

    line('VLA - one call', (255, 200, 80), 24)
    line(f'INPUT image : {IMG_SIZE}x{IMG_SIZE}x3 (left)', (140, 200, 255))
    line(f'INPUT words : "{instruction}"', (140, 200, 255))
    line('INPUT state :', (140, 200, 255))
    for name, v in zip(LABELS, state):
        line(f'   {name:<13} {v:+7.3f}', (200, 200, 200), 15)
    y += 8
    line('OUTPUT action (targets):', (120, 255, 140))
    for name, s, a in zip(LABELS, state, action):
        # PIL's built-in font is ASCII-only — no unicode arrows here
        arrow = '^' if a > s + 0.01 else ('v' if a < s - 0.01 else '=')
        line(f'   {name:<13} {a:+7.3f}  {arrow}', (200, 255, 200), 15)
    card.save(out_path)


def main():
    ap = argparse.ArgumentParser(
        description='One VLA call, inputs and output made visible')
    ap.add_argument('--instruction', default='pick the red tomato',
                    help='the language input — try changing it!')
    ap.add_argument('--steps', type=int, default=1,
                    help='how many calls to show (1 = a single step)')
    ap.add_argument('--spawn', action='store_true',
                    help='place red+green tomatoes at the trained spots first')
    ap.add_argument('--red-side', choices=['left', 'right'], default='left')
    ap.add_argument('--execute', action='store_true',
                    help='actually PUBLISH each action (the arm will move!)')
    ap.add_argument('--backend', default=None,
                    help='vla_client backend (default: local checkpoint)')
    ap.add_argument('--out', default=str(Path.home() / 'vla_one_step.png'),
                    help='where to save the input/output picture')
    args = ap.parse_args()

    # 1) load the brain (same client the full executor uses)
    print('Loading the model (first load takes ~20 s) ...')
    client = make_vla_client(args.backend)
    if hasattr(client, 'reset'):
        # first call pays one-time costs (CUDA init) — do it on a dummy frame
        client.act(np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8),
                   args.instruction, [0.0] * 7)
        client.reset()                       # fresh episode, empty action queue

    # 2) connect to the sim
    rclpy.init()
    node = OneStepIO()
    if args.spawn:
        spawn_scene(node, red_left=(args.red_side == 'left'))
    node.spin(1.0)                           # let the first image/state arrive

    img, state = node.latest_img, node.state_vec()
    if img is None or state is None:
        print('✗ no camera image / joint state — is the sim running? (sim_d2)')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    # 3) the call(s)
    print('═' * 62)
    print('A VLA is a function:  action = model(image, instruction, state)')
    print('═' * 62)
    action = None
    for step in range(args.steps):
        img, state = node.latest_img, node.state_vec()
        t0 = time.perf_counter()
        act = client.act(img, args.instruction, state)     # ← THE call
        ms = (time.perf_counter() - t0) * 1000.0
        action = np.array(act.joints + [act.gripper], dtype=np.float32)

        if step == 0:
            print(f'\nINPUT 1 — image  : {img.shape[1]}×{img.shape[0]} RGB from '
                  f'/wrist_camera/image_raw (what the robot SEES)')
            print(f'INPUT 2 — words  : "{args.instruction}"  (what we WANT)')
            print(f'INPUT 3 — state  : 7 floats from /joint_states (where it IS)')
            print(f'\nOUTPUT  — action : 7 floats = 6 joint targets + gripper '
                  f'(what to DO next)   [{ms:.0f} ms]\n')
            print(f'  {"joint":<14} {"state (in)":>11} {"action (out)":>13}   move?')
            print('  ' + '-' * 48)
            for name, s, a in zip(LABELS, state, action):
                arrow = '↑ up' if a > s + 0.01 else ('↓ down' if a < s - 0.01 else '· hold')
                print(f'  {name:<14} {s:>+11.3f} {a:>+13.3f}   {arrow}')
        else:
            deltas = ' '.join(f'{a:+.2f}' for a in action)
            print(f'  step {step + 1:>2}: action = [{deltas}]   [{ms:.0f} ms]')

        if args.execute:
            # this ONE line is what turns a prediction into robot motion —
            # the full executor (vla_executor.py) just repeats it at 10 Hz
            node.publish_arm(action[:6], action[6])
            node.spin(0.1)

    # 4) the picture (open it!)
    save_io_card(img, args.instruction, state, action, args.out)
    print(f'\n🖼  input/output card saved → {args.out}')
    if args.steps == 1:
        print('\nRun with --steps 5 to see consecutive calls (the model plans a')
        print('CHUNK of ~50 actions internally and streams it out one by one),')
        print('or with --execute to publish the action and watch the arm move.')
    if not args.execute:
        print('Nothing was sent to the robot (add --execute to send).')
    print('Next: the full loop →  vla_d4 --task C --spawn')

    if args.spawn:                           # leave the scene as we found it
        for name in ('tomato_red_0', 'tomato_green_0'):
            gz_utils.remove_model(name)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
