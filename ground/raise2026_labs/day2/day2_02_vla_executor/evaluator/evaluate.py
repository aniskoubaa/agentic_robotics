#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Lab 2.2 evaluator: score a VLA policy on live rollouts.

Runs the trained policy against N scripted trials in the live sim and grades
per the Lab-2.2 rubric (README):
    60%  success — the CORRECT (red) tomato grasped, judged by /grasp/state
         (the deterministic grasp gives us clean ground truth)
    20%  latency — every model call within the ≤500 ms budget
    20%  safety/recovery — never irreversibly grabs the WRONG (green) tomato

Each trial recreates the training distribution: red + green tomato at the two
known grasp points (shared with the auto-demonstrator via sim_poses.py), red
side alternating L/R, arm reset to HOME — then the policy runs the whole pick.

PREREQUISITES: the sim (raise-sim / headless) + grasp_server running, and the
checkpoint selected via VLA_LOCAL_CKPT. Run with the lerobot venv python:

    export VLA_LOCAL_CKPT=~/raise_checkpoints/smolvla_C_ref
    ~/raise_venvs/lerobot/bin/python3 evaluate.py --task C --trials 10
"""

import argparse
import json
import os
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
from vla_client import make_vla_client                                   # noqa: E402
from vla_client.base import (                                             # noqa: E402
    UR5E_JOINTS, GRIPPER_KNUCKLE, GRIPPER_MIMIC_SIGNS, GRIPPER_OPEN,
)
from vla_client.ros_image import imgmsg_to_rgb, resize_rgb                # noqa: E402
from sim_poses import POSE_HOME, GRASP_LEFT, GRASP_RIGHT, snap_to_park    # noqa: E402
from raise2026_tools import gz_utils                                      # noqa: E402

STATE_JOINTS = UR5E_JOINTS + [GRIPPER_KNUCKLE]
IMG_SIZE = 224
LATENCY_BUDGET_MS = 500.0
RED, GREEN = 'tomato_red_0', 'tomato_green_0'


class EvalIO(Node):
    """Sensor/command IO for evaluation rollouts (same topics as the executor)."""

    def __init__(self, world: str, gripper_link: str):
        super().__init__('vla_evaluator')
        self.set_parameters([RclpyParameter('use_sim_time', RclpyParameter.Type.BOOL, True)])
        self.world = world
        self.gripper_link = gripper_link
        all_joints = STATE_JOINTS + list(GRIPPER_MIMIC_SIGNS)
        self.pubs = {j: self.create_publisher(Float64, f'/{j}/cmd', 10) for j in all_joints}
        self.register_pub = self.create_publisher(String, '/grasp/register', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_img = None
        self.measured = {}
        self.grasp_state = 'none'
        self.create_subscription(Image, '/wrist_camera/image_raw', self._on_image, 10)
        self.create_subscription(JointState, '/joint_states', self._on_state, 10)
        self.create_subscription(String, '/grasp/state',
                                 lambda m: setattr(self, 'grasp_state', m.data), 10)

    def _on_image(self, msg):
        self.latest_img = resize_rgb(imgmsg_to_rgb(msg), IMG_SIZE)

    def _on_state(self, msg):
        self.measured = dict(zip(msg.name, msg.position))

    def spin(self, secs):
        """Wait `secs` of SIM time (/clock) — correct at any real-time factor.

        With the Gazebo GUI open the sim can run below 100% real-time; a
        wall-clock loop then drives the robot too fast relative to physics
        (found live: GUI runs failed while headless runs passed)."""
        start = self.get_clock().now()
        dur = RclpyDuration(seconds=secs)
        while rclpy.ok() and (self.get_clock().now() - start) < dur:
            rclpy.spin_once(self, timeout_sec=0.02)

    def state_vec(self):
        if not all(j in self.measured for j in STATE_JOINTS):
            return None
        return np.array([self.measured[j] for j in STATE_JOINTS], dtype=np.float32)

    def publish_arm(self, joints, gripper):
        for j, v in zip(UR5E_JOINTS, joints):
            self.pubs[j].publish(Float64(data=float(v)))
        self.pubs[GRIPPER_KNUCKLE].publish(Float64(data=float(gripper)))
        for j, sign in GRIPPER_MIMIC_SIGNS.items():
            self.pubs[j].publish(Float64(data=float(sign * gripper)))

    def goto(self, joints, gripper, settle_s=2.5):
        """Scripted setup move (re-publishes while settling — discovery-safe)."""
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


def run_trial(node, client, instruction, red_pt, green_pt, hz, max_steps):
    """One rollout: spawn scene -> policy drives -> outcome from /grasp/state."""
    # Fresh episode for the policy: clear its action-chunk queue, or stale
    # chunks from the previous trial leak in (found live: t1 ok, t2-4 timeout).
    if hasattr(client, 'reset'):
        client.reset()
    # scene setup (arm already at HOME)
    gz_utils.spawn_model(RED, 'model://tomato_red', *red_pt, world=node.world)
    gz_utils.spawn_model(GREEN, 'model://tomato_green', *green_pt, world=node.world)
    node.spin(0.4)
    node.register_pub.publish(String(data=RED))
    node.register_pub.publish(String(data=GREEN))
    node.spin(2.5)                       # grasp_server slow-sync resolves poses

    dt, latencies = 1.0 / hz, []
    outcome, wrong_grab = 'timeout', False
    for _ in range(max_steps):
        img, state = node.latest_img, node.state_vec()
        if img is None or state is None:
            node.spin(dt)
            continue
        t0 = time.perf_counter()
        action = client.act(img, instruction, state)
        latencies.append((time.perf_counter() - t0) * 1000.0)
        for joint, target in action.joint_commands():
            node.pubs[joint].publish(Float64(data=float(target)))
        node.spin(dt)
        if node.grasp_state == RED:
            outcome = 'success'
            break
        if node.grasp_state == GREEN:
            outcome = 'wrong_object'     # grabbed the green one — hard fail
            wrong_grab = True
            break

    # cleanup for the next trial
    for name in (RED, GREEN):
        gz_utils.remove_model(name, world=node.world)
    node.spin(0.3)
    return outcome, wrong_grab, latencies


def main():
    ap = argparse.ArgumentParser(description='RAISE 2026 Lab 2.2 policy evaluator')
    ap.add_argument('--task', default='C', help='A | C | B')
    ap.add_argument('--trials', type=int, default=10)
    ap.add_argument('--instruction', default='pick the red tomato')
    ap.add_argument('--backend', default=None)
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--max-steps', type=int, default=120)   # scan runs can exceed 80
    ap.add_argument('--world', default=gz_utils.DEFAULT_WORLD)
    ap.add_argument('--gripper-link', default='gripper_mount_link')
    args = ap.parse_args()

    print(f'Evaluating backend={args.backend or os.getenv("VLA_BACKEND", "local-smolvla")}'
          f'  ckpt={os.getenv("VLA_LOCAL_CKPT", "lerobot/smolvla_base")}')
    print(f'Instruction: "{args.instruction}"   trials: {args.trials}\n')
    client = make_vla_client(args.backend)

    # Warm up once: the very first act() pays one-time costs (CUDA graph/JIT,
    # tokenizer init) that belong to deployment startup, not the control loop.
    # Without this, trial 1's first step can spuriously blow the 500 ms budget.
    if hasattr(client, 'reset'):
        import numpy as _np
        client.act(_np.zeros((224, 224, 3), dtype=_np.uint8), args.instruction,
                   [0.0] * 7)
        client.reset()

    rclpy.init()
    node = EvalIO(args.world, args.gripper_link)

    snap_to_park(gz_utils, world=args.world)   # exact training geometry
    # Locate the two grasp points once (no-IK trick, same poses as the demos).
    node.goto(GRASP_LEFT, GRIPPER_OPEN)
    left_pt = node.tool_point()
    node.goto(GRASP_RIGHT, GRIPPER_OPEN)
    right_pt = node.tool_point()
    if left_pt is None or right_pt is None:
        print('✗ could not locate grasp points (sim/TF not ready).')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    results = []
    try:
        for t in range(args.trials):
            red_left = (t % 2 == 0)
            red_pt, green_pt = (left_pt, right_pt) if red_left else (right_pt, left_pt)
            node.goto(POSE_HOME, GRIPPER_OPEN)               # reset arm
            outcome, wrong, lat = run_trial(node, client, args.instruction,
                                            red_pt, green_pt, args.hz, args.max_steps)
            results.append((outcome, wrong, lat))
            mean_l = np.mean(lat) if lat else 0.0
            print(f'  trial {t + 1:>2}/{args.trials}  red={"L" if red_left else "R"}'
                  f'  → {outcome:<12} (mean act {mean_l:5.1f} ms)')
    except KeyboardInterrupt:
        pass
    finally:
        node.goto(POSE_HOME, GRIPPER_OPEN, settle_s=1.5)
        node.destroy_node()
        rclpy.shutdown()

    if not results:
        sys.exit(1)
    n = len(results)
    successes = sum(1 for o, _, _ in results if o == 'success')
    wrongs = sum(1 for _, w, _ in results if w)
    all_lat = [v for _, _, lat in results for v in lat]
    lat_ok = all(v <= LATENCY_BUDGET_MS for v in all_lat) if all_lat else False

    success_score = successes / n * 60
    latency_score = 20.0 if lat_ok else 0.0
    safety_score = (n - wrongs) / n * 20
    total = success_score + latency_score + safety_score

    print(f'\n── score ─────────────────────────────────────')
    print(f'  success   : {successes}/{n}                    → {success_score:.0f}/60')
    print(f'  latency   : max {max(all_lat or [0]):.0f} ms (≤{LATENCY_BUDGET_MS:.0f})   → {latency_score:.0f}/20')
    print(f'  safety    : {n - wrongs}/{n} no-wrong-grab      → {safety_score:.0f}/20')
    print(f'  TOTAL     : {total:.0f}/100')
    print(json.dumps({'trials': n, 'successes': successes, 'wrong_grabs': wrongs,
                      'mean_latency_ms': round(float(np.mean(all_lat)), 1) if all_lat else None,
                      'max_latency_ms': round(float(np.max(all_lat)), 1) if all_lat else None,
                      'total': round(total, 1)}))


if __name__ == '__main__':
    main()
