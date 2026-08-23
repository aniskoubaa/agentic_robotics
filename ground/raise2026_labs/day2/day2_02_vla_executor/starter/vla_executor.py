#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Lab 2.2 — the VLA executor (command the robot by speaking).

WHAT  Type an instruction ("pick the red tomato"). The executor captures the
      wrist image, asks the trained VLA what to do, and streams the predicted
      joint actions to the arm — until the tomato is grasped or it runs out of
      steps. You write NO motion code: the movements come from the model.

WHY   This is the "EXECUTE" half of Day 2, and the pay-off for Lab 2.1: the
      brain moving the arm was TRAINED on the demos recorded this morning.
      Same loop shape as Day-1's agent.py (loop until success or limit) — but
      the "tool" is a single VLA call returning low-level joint actions.

      Reference result (greenhouse scenes, measured 2026-07-15 on a fresh
      session): 8/8 correct-color picks both sides, 0 wrong grabs,
      max 167 ms → 100/100.

LEARN - make_vla_client() returns a BACKEND-BLIND client. Default is in-process
        SmolVLA; set VLA_BACKEND=remote-openvla + VLA_REMOTE_URL to A/B against
        a self-hosted endpoint. The loop is identical either way.
      - client.reset() clears the policy's action-chunk queue — ALWAYS call it
        before an episode (stale chunks from a previous run otherwise leak in).
      - SUCCESS is judged by /grasp/state (the grasp_server's ground truth):
        the red tomato attached = success; the green one = wrong-object fail.
      - Every client.act() call is timed against the ≤500 ms budget (graded).

PREREQUISITES (three things):
    1. the sim:          raise-sim   (or headless — see VERIFY_MANIPULATION.md)
    2. the grasp server: ros2 run raise2026_tools grasp_server    (grasp_d3)
    3. a checkpoint:     export VLA_LOCAL_CKPT=<...>/pretrained_model
       (if unset: the local reference checkpoint when installed, else the
        PUBLIC Hugging Face copy is downloaded automatically — no key needed.
        For the flail demo, explicitly set VLA_LOCAL_CKPT=lerobot/smolvla_base.)

Run WITH THE LEROBOT VENV PYTHON (the vla_d4 alias does this for you):
    vla_d4 --task C --spawn --instruction "pick the red tomato"
    # --spawn places a red+green tomato at the trained grasp points first.
    # Manual equivalent:
    #   ~/raise_venvs/lerobot/bin/python3 vla_executor.py --task C --spawn
"""

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

try:
    import _d2paths                    # installed next to the ros2-run scripts
except ImportError:                    # running straight from the source tree
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    import _d2paths
_d2paths.bootstrap(lerobot=True)
DAY2 = _d2paths.DAY2
from vla_client import make_vla_client, ExecutionResult                    # noqa: E402
from vla_client.base import UR5E_JOINTS, GRIPPER_KNUCKLE, GRIPPER_MIMIC_SIGNS, GRIPPER_OPEN   # noqa: E402
from vla_client.ros_image import imgmsg_to_rgb, resize_rgb                 # noqa: E402
from task_pack import load_task                                            # noqa: E402
from sim_poses import POSE_HOME, GRASP_LEFT, GRASP_RIGHT, check_parking, snap_to_park   # noqa: E402
from raise2026_tools import gz_utils                                       # noqa: E402

IMAGE_TOPIC = '/wrist_camera/image_raw'
STATE_TOPIC = '/joint_states'
STATE_JOINTS = UR5E_JOINTS + [GRIPPER_KNUCKLE]
IMG_SIZE = 224
LATENCY_BUDGET_MS = 500.0
RED, GREEN = 'tomato_red_0', 'tomato_green_0'

# The instructor reference checkpoint, when it exists on this machine —
# pick the LATEST training step under checkpoints/ (e.g. 003000, 006000).
# Reference-checkpoint discovery lives in the factory so vla_one_step.py and
# the evaluator behave identically when VLA_LOCAL_CKPT is unset.
from vla_client.factory import find_reference_ckpt                        # noqa: E402

REFERENCE_CKPT = find_reference_ckpt()


# ── .env loader (same helper as Day-1 agent.py) ────────────────────────────
def load_dotenv_into_environ():
    candidates = []
    for prefix in os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            candidates.append(os.path.join(prefix, '..', 'src', '.env'))
    candidates.append(os.path.join(os.getcwd(), '.env'))
    for path in candidates:
        if os.path.isfile(path):
            for line in open(path):
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    k, v = line.split('=', 1)
                    os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))
            return path
    return None


class ArmIO(Node):
    """Captures wrist image + joint state (+ grasp truth); publishes commands."""

    def __init__(self, enable_base: bool, world: str, gripper_link: str):
        super().__init__('vla_executor')
        # follow the simulator's clock (see spin() for why)
        self.set_parameters([RclpyParameter('use_sim_time', RclpyParameter.Type.BOOL, True)])
        self.world = world
        self.gripper_link = gripper_link
        self.latest_img = None
        self.latest_state = {}
        self.grasp_state = 'none'          # what the grasp_server is holding

        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, 10)
        self.create_subscription(JointState, STATE_TOPIC, self._on_state, 10)
        self.create_subscription(String, '/grasp/state',
                                 lambda m: setattr(self, 'grasp_state', m.data), 10)
        self.register_pub = self.create_publisher(String, '/grasp/register', 10)

        all_joints = STATE_JOINTS + list(GRIPPER_MIMIC_SIGNS)
        self.pubs = {j: self.create_publisher(Float64, f'/{j}/cmd', 10) for j in all_joints}
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', 10) if enable_base else None

        # Frames for --spawn: TF (base->gripper) + robot world pose from gz.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_image(self, msg: Image):
        self.latest_img = resize_rgb(imgmsg_to_rgb(msg), IMG_SIZE)

    def _on_state(self, msg: JointState):
        self.latest_state = dict(zip(msg.name, msg.position))

    def spin(self, secs: float):
        """Wait `secs` of SIM time while processing callbacks.

        Sim time (the /clock topic), NOT wall time: with the Gazebo GUI open
        the sim can run at e.g. 75% real-time, and a wall-clock 10 Hz loop
        then commands the robot 33% too fast relative to the physics — enough
        to push the policy off its training distribution (found live: picks
        that succeed headless time out with the GUI running). Pacing by sim
        time makes the executor correct at ANY real-time factor."""
        start = self.get_clock().now()
        dur = RclpyDuration(seconds=secs)
        while rclpy.ok() and (self.get_clock().now() - start) < dur:
            rclpy.spin_once(self, timeout_sec=0.02)

    def state_vec(self):
        if not all(j in self.latest_state for j in STATE_JOINTS):
            return None
        return np.array([self.latest_state[j] for j in STATE_JOINTS], dtype=np.float32)

    def send(self, action):
        """Publish a canonical Action (joint_commands expands the gripper mimics)."""
        for joint, target in action.joint_commands():
            self.pubs[joint].publish(Float64(data=float(target)))
        if self.base_pub is not None and action.base is not None:
            t = Twist()
            t.linear.x = float(action.base.get('linear_x', 0.0))
            t.angular.z = float(action.base.get('angular_z', 0.0))
            self.base_pub.publish(t)

    # ── scene setup for --spawn (same mechanics as the evaluator) ───────────
    def publish_arm(self, joints, gripper):
        for j, v in zip(UR5E_JOINTS, joints):
            self.pubs[j].publish(Float64(data=float(v)))
        self.pubs[GRIPPER_KNUCKLE].publish(Float64(data=float(gripper)))
        for j, sign in GRIPPER_MIMIC_SIGNS.items():
            self.pubs[j].publish(Float64(data=float(sign * gripper)))

    def goto(self, joints, gripper, settle_s=2.5):
        """Scripted setup move — republishes while settling (discovery-safe)."""
        end = time.time() + settle_s
        while rclpy.ok() and time.time() < end:
            self.publish_arm(joints, gripper)
            self.spin(0.15)

    def tool_point(self, grasp_offset=0.13):
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
        pb = (p.x + off[0], p.y + off[1], p.z + off[2])
        (bx, by, bz), bq = wtb
        w = gz_utils.rotate_vec(bq, pb)
        return (bx + w[0], by + w[1], bz + w[2])

    def spawn_scene(self, red_left: bool):
        """Place a red + green tomato at the two trained grasp points."""
        # The reference model was trained at the plant-row parking. Snap the
        # base there EXACTLY (spawn settle varies by cm — enough to drift the
        # scan views off the training distribution; found live 2026-07-15).
        snap_to_park(gz_utils, world=self.world)

        def locate(pose, tries=4):
            # retry: right after sim start TF + gz services need a few seconds
            for attempt in range(tries):
                self.goto(pose, GRIPPER_OPEN,
                          settle_s=2.5 if attempt == 0 else 1.5)
                self.spin(0.5)
                pt = self.tool_point()
                if pt is not None:
                    return pt
            return None

        left = locate(GRASP_LEFT)
        right = locate(GRASP_RIGHT)
        self.goto(POSE_HOME, GRIPPER_OPEN)
        if left is None or right is None:
            return False
        red_pt, green_pt = (left, right) if red_left else (right, left)
        gz_utils.spawn_model(RED, 'model://tomato_red', *red_pt, world=self.world)
        gz_utils.spawn_model(GREEN, 'model://tomato_green', *green_pt, world=self.world)
        self.spin(0.4)
        self.register_pub.publish(String(data=RED))
        self.register_pub.publish(String(data=GREEN))
        self.spin(2.5)                      # let grasp_server resolve the poses
        print(f'  scene ready: red on the {"LEFT" if red_left else "RIGHT"}')
        return True

    def cleanup_scene(self):
        for name in (RED, GREEN):
            gz_utils.remove_model(name, world=self.world)


def vla_execute(node: ArmIO, client, instruction: str, hz: float, max_steps: int) -> ExecutionResult:
    """The core loop: see -> ask the VLA -> act, until the grasp truth fires."""
    if hasattr(client, 'reset'):
        client.reset()                       # fresh episode: clear the chunk queue
    dt = 1.0 / hz
    latencies = []
    for step in range(max_steps):
        rclpy.spin_once(node, timeout_sec=0.0)
        img = node.latest_img
        state = node.state_vec()
        if img is None or state is None:
            time.sleep(dt)
            continue

        t0 = time.perf_counter()
        action = client.act(img, instruction, state)     # the VLA decides the motion
        latency_ms = (time.perf_counter() - t0) * 1000.0
        latencies.append(latency_ms)

        node.send(action)
        flag = '  ⚠ over budget' if latency_ms > LATENCY_BUDGET_MS else ''
        print(f'\r  step {step + 1:>3}/{max_steps} | act {latency_ms:6.1f} ms{flag}   ', end='', flush=True)

        # Let the controllers move and fresh sensor data arrive before the next step.
        end = time.perf_counter() + dt
        while time.perf_counter() < end:
            rclpy.spin_once(node, timeout_sec=0.005)

        # Ground truth from the grasp_server: what is actually being held?
        if node.grasp_state == RED:
            return _result('success', step + 1, latencies, 'red tomato grasped')
        if node.grasp_state == GREEN:
            return _result('wrong_object', step + 1, latencies, 'grabbed the GREEN one')
    return _result('timeout', max_steps, latencies, 'step limit reached')


def _result(status: str, steps: int, latencies: list, note: str) -> ExecutionResult:
    lat = latencies or [0.0]
    return ExecutionResult(
        status=status, steps=steps,
        mean_latency_ms=float(np.mean(lat)), max_latency_ms=float(np.max(lat)),
        latencies_ms=lat, note=note,
    )


def main():
    ap = argparse.ArgumentParser(description='RAISE 2026 VLA executor')
    ap.add_argument('--task', default='C', help='A | C | B (C = the graded Day-2 task)')
    ap.add_argument('--instruction', default=None, help='override the spoken instruction')
    ap.add_argument('--backend', default=None, help='local-smolvla (default) | remote-openvla')
    ap.add_argument('--spawn', action='store_true',
                    help='place a red+green tomato at the trained grasp points first')
    ap.add_argument('--red-side', choices=['left', 'right'], default='left',
                    help='with --spawn: which side gets the red tomato')
    ap.add_argument('--hz', type=float, default=10.0, help='control rate')
    ap.add_argument('--max-steps', type=int, default=120)   # scan episodes run up to ~97 frames
    ap.add_argument('--world', default=gz_utils.DEFAULT_WORLD)
    ap.add_argument('--gripper-link', default='gripper_mount_link')
    args = ap.parse_args()

    load_dotenv_into_environ()
    # No checkpoint chosen? Use the local reference checkpoint if it exists.
    if not os.getenv('VLA_LOCAL_CKPT') and REFERENCE_CKPT is not None:
        os.environ['VLA_LOCAL_CKPT'] = str(REFERENCE_CKPT)
        print(f'(VLA_LOCAL_CKPT unset — using the reference checkpoint {REFERENCE_CKPT})')

    pack = load_task(args.task)
    instruction = args.instruction or pack.instructions[0]

    print(f'Task {pack.id}: {pack.name}')
    print(f'Backend    : {args.backend or os.getenv("VLA_BACKEND", "local-smolvla")}')
    print(f'Checkpoint : {os.getenv("VLA_LOCAL_CKPT", "lerobot/smolvla_base (NOT fine-tuned — will flail)")}')
    print(f'Instruction: "{instruction}"\n')
    print('Loading the VLA (first load takes ~20 s) ...')
    client = make_vla_client(args.backend)
    # Warm up once: the very first act() pays one-time costs (CUDA graphs,
    # tokenizer init) that belong to startup, not the control loop — without
    # this, step 1 spuriously blows the 500 ms budget.
    if hasattr(client, 'reset'):
        client.act(np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8), instruction, [0.0] * 7)
        client.reset()

    rclpy.init()
    node = ArmIO(enable_base=pack.uses_base, world=args.world, gripper_link=args.gripper_link)
    spawned = False
    try:
        if args.spawn:
            spawned = node.spawn_scene(red_left=args.red_side == 'left')
            if not spawned:
                print('⚠ could not set up the scene (sim/TF not ready) — running anyway.')
        result = vla_execute(node, client, instruction, args.hz, args.max_steps)
    finally:
        if spawned:
            node.cleanup_scene()
        node.destroy_node()
        rclpy.shutdown()

    print('\n\n── result ──────────────────────────────')
    print(f'  outcome     : {result.status.upper()}  ({result.note})')
    print(f'  steps       : {result.steps}')
    print(f'  latency     : mean {result.mean_latency_ms:.1f} ms, max {result.max_latency_ms:.1f} ms')
    budget = 'OK ✓' if result.max_latency_ms <= LATENCY_BUDGET_MS else 'EXCEEDED ✗'
    print(f'  ≤{LATENCY_BUDGET_MS:.0f} ms/action: {budget}')


if __name__ == '__main__':
    main()
