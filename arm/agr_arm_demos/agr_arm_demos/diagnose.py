#!/usr/bin/env python3
"""
AGR Arm — diagnostics. Is this robot actually working?
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_arm_demos diagnose
    ros2 run agr_arm_demos diagnose --ros-args -p active:=true    # really moves it

WHAT  Ten checks, in the order things actually break, each printing what it
      saw and what to do about it. Run this FIRST whenever something is odd.

WHY   A manipulation sim fails in layers and every layer looks identical from
      the top: "the arm does not move". Gazebo up but no model. Model spawned
      but the joint controllers not loaded. Controllers fine but the tool
      servers never started, so every service call blocks forever. Each has a
      different fix and none of them is guessable from the symptom, so the
      useful thing a diagnostic does is tell you WHICH LAYER.

      The tenth check is different in kind: it compares this stack's forward
      kinematics against the TF tree that robot_state_publisher builds from
      the same URDF. If those two disagree, every Cartesian number the
      platform prints is wrong, and nothing else here would notice.

Exit code is 0 when everything passed, 1 otherwise, so it doubles as a smoke
test in a script.
"""
from __future__ import annotations

import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from agr_arm_tools import kinematics as K
from agr_arm_tools.arm import GRIPPER_SIGNS

OK, BAD, WARN = '  OK  ', ' FAIL ', ' WARN '

SERVICES = ('/open_gripper', '/close_gripper', '/rotate_gripper',
            '/move_to_home', '/move_to_ready', '/move_to_watch', '/move_to_stow')


class Diagnose(Node):
    def __init__(self) -> None:
        super().__init__('arm_diagnose')
        self.declare_parameter('active', False)
        self.active = bool(self.get_parameter('active').value)
        self.failures = 0
        self.step = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # ── reporting ─────────────────────────────────────────────────────────
    def report(self, tag: str, title: str, detail: str, fix: str = '') -> None:
        self.step += 1
        print(f'[{tag}] {self.step}. {title}')
        if detail:
            print(f'        {detail}')
        if tag == BAD:
            self.failures += 1
            if fix:
                print(f'        -> {fix}')

    def collect(self, msg_type, topic, seconds: float = 3.0) -> list:
        """Every message seen on `topic` within `seconds`. Empty means the
        topic is dead — which is not the same as the topic not existing."""
        got: list = []
        sub = self.create_subscription(msg_type, topic, got.append, 10)
        end = time.time() + seconds
        try:
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
        finally:
            self.destroy_subscription(sub)
        return got

    def wait_for(self, msg_type, topic, timeout: float):
        """Spin until the FIRST message on `topic`, up to `timeout`.

        Not the same as collect(): this returns the moment data appears,
        rather than sampling a fixed window. The distinction matters for the
        very first check, which runs seconds after launch, when DDS discovery
        may not have matched the publisher yet. A fixed 2 s window there
        reported "Gazebo is not running" against a simulator that was
        provably stepping — twice, on this machine, while writing this file.
        """
        got: list = []
        sub = self.create_subscription(msg_type, topic, got.append, 10)
        end = time.time() + timeout
        try:
            while rclpy.ok() and time.time() < end and not got:
                rclpy.spin_once(self, timeout_sec=0.05)
        finally:
            self.destroy_subscription(sub)
        return got, timeout - max(end - time.time(), 0.0)

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ── the checks ────────────────────────────────────────────────────────
    def run(self) -> int:
        print('\nAGR arm diagnostics\n' + '=' * 62)

        # 1. Is Gazebo running at all? /clock is the earliest sign of life.
        #    Ten seconds, not two: this is the check that runs first, right
        #    after a launch, and DDS discovery is what it is.
        first, waited = self.wait_for(Clock, '/clock', 10.0)
        if not first:
            self.report(BAD, 'Gazebo is running', 'no /clock in 10 s',
                        'start it:  agr-sim arm')
            print('\nNothing below can pass without a simulator. Stopping.\n')
            return 1
        clock = self.collect(Clock, '/clock', 2.0)
        self.report(OK, 'Gazebo is running',
                    f'/clock ticking at {len(clock) / 2.0:.0f} Hz'
                    + (f' (took {waited:.1f} s to discover)' if waited > 1.0 else ''))

        # Only NOW list the graph. get_topic_names_and_types() reports what
        # this node has discovered SO FAR, and DDS discovery is not instant:
        # asking on the first line of main() reliably reports topics as
        # absent that are provably alive two checks later. The /clock wait
        # above is what gives discovery the time it needs.
        topics = dict(self.get_topic_names_and_types())

        # 2. Did the robot spawn?
        if '/robot_description' in topics or '/joint_states' in topics:
            self.report(OK, 'Robot description loaded', 'robot_state_publisher is up')
        else:
            self.report(BAD, 'Robot description loaded', 'no /robot_description',
                        'the xacro failed to expand — check the launch output')

        # 3. Joint feedback: all six arm joints, not just some.
        js = self.collect(JointState, '/joint_states', 2.0)
        angles = None
        if not js:
            self.report(BAD, 'Joint states arriving', 'nothing on /joint_states',
                        'the joint state publisher plugin is missing from the URDF')
        else:
            have = dict(zip(js[-1].name, js[-1].position))
            missing = [j for j in K.JOINTS if j not in have]
            if missing:
                self.report(BAD, 'Joint states arriving',
                            f'{len(have)} joints, missing {missing}',
                            'the URDF and kinematics.JOINTS disagree')
            else:
                angles = [have[j] for j in K.JOINTS]
                # Expect roughly the physics rate, not 50 Hz: gz-sim 8's
                # JointStatePublisher has no update_rate parameter (see the
                # note in agr_arm_gazebo.urdf.xacro).
                self.report(OK, 'Joint states arriving',
                            f'all 6 arm joints, {len(js) / 2.0:.0f} Hz '
                            f'(this tracks the physics rate, not a cap)')

        # 4. Gripper joints. Six of them, and a missing one is a sign
        #    handling is wrong rather than that the gripper is absent.
        if js:
            have = set(js[-1].name)
            missing = [j for j in GRIPPER_SIGNS if j not in have]
            if missing:
                self.report(WARN, 'Gripper joints present',
                            f'missing {len(missing)}: {missing[0]} ...')
            else:
                self.report(OK, 'Gripper joints present', 'all 6 Robotiq joints')

        # 5. Command topics. The arm cannot be driven without these, and
        #    their absence means the bridge config is wrong, not the URDF.
        wanted = [f'/{j}/cmd' for j in K.JOINTS]
        absent = [t for t in wanted if t not in topics]
        if absent:
            self.report(BAD, 'Joint command topics bridged',
                        f'missing {len(absent)}: {absent[0]} ...',
                        'check agr_arm_bringup/config/ros_gz_bridge.yaml')
        else:
            self.report(OK, 'Joint command topics bridged',
                        'all 6 /ur5e_*_joint/cmd present')

        # 6. Cameras.
        for name, topic in (('wrist', '/wrist_camera/image_raw'),
                            ('bench', '/bench_camera/image_raw')):
            imgs = self.collect(Image, topic, 3.0)
            if imgs:
                m = imgs[-1]
                self.report(OK, f'{name.capitalize()} camera streaming',
                            f'{m.width}x{m.height} {m.encoding}, '
                            f'{len(imgs) / 3.0:.1f} Hz')
            else:
                self.report(BAD, f'{name.capitalize()} camera streaming',
                            f'nothing on {topic}',
                            'the world needs gz-sim-sensors-system, and a plain '
                            '<camera> publishes on its <topic> EXACTLY — see '
                            'agr_arm_gazebo.urdf.xacro')

        # 7. Tool servers. Their absence is the single most common cause of a
        #    lab that "hangs": a service call with no server blocks forever.
        services = {name for name, _ in self.get_service_names_and_types()}
        found = [s for s in SERVICES if s in services]
        if len(found) == len(SERVICES):
            self.report(OK, 'Tool servers running', f'{len(found)} services')
        elif found:
            self.report(WARN, 'Tool servers running',
                        f'only {len(found)} of {len(SERVICES)}: '
                        f'{sorted(set(SERVICES) - set(found))}')
        else:
            self.report(WARN, 'Tool servers running',
                        'none — service calls will block forever',
                        'start them:  agr-sim arm tools:=true')

        # 8. grasp_server. Without it the gripper closes and nothing is held.
        if '/grasp/state' in topics:
            state = self.collect(String, '/grasp/state', 2.0)
            if state:
                held = state[-1].data
                self.report(OK, 'Grasp server running',
                            f'holding: {held}, {len(state) / 2.0:.0f} Hz')
            else:
                self.report(WARN, 'Grasp server running',
                            '/grasp/state exists but is silent')
        else:
            self.report(WARN, 'Grasp server running',
                        'no /grasp/state — the gripper will close on nothing',
                        'start it:  agr-sim arm tools:=true')

        # 9. Is the arm actually holding its pose, or sagging?
        if angles is not None:
            js2 = self.collect(JointState, '/joint_states', 1.5)
            if js2:
                have2 = dict(zip(js2[-1].name, js2[-1].position))
                drift = max(abs(have2[j] - a) for j, a in zip(K.JOINTS, angles))
                if drift < 0.01:
                    self.report(OK, 'Arm is holding still',
                                f'moved {drift:.4f} rad in 1.5 s')
                else:
                    self.report(WARN, 'Arm is holding still',
                                f'drifting {drift:.4f} rad per 1.5 s — either it '
                                f'is mid-move, or a joint controller is missing')

        # 10. The one check nothing else would catch: does this stack's own
        #     forward kinematics agree with TF? If not, every Cartesian
        #     number the platform prints is quietly wrong.
        if angles is not None:
            self.spin_for(1.5)                 # let the TF buffer fill
            fresh = self.collect(JointState, '/joint_states', 0.5)
            if not fresh:
                self.report(WARN, 'Kinematics agrees with TF',
                            'no fresh joint sample to compare against')
            else:
                sample = fresh[-1]
                have = dict(zip(sample.name, sample.position))
                q = [have[j] for j in K.JOINTS]
                # Both readings are taken "latest", not matched by timestamp,
                # and that is a deliberate simplification rather than an
                # oversight. robot_state_publisher emits TF at 30 Hz while
                # /joint_states arrives at the physics rate near 1 kHz, so a
                # joint sample is almost always NEWER than the newest
                # transform and a stamped lookup would extrapolate or fail
                # every single time. What makes the comparison valid instead
                # is check 10 immediately above: if the arm is holding still
                # to 0.0002 rad, the few milliseconds between the two
                # readings are worth a fraction of a micrometre. Run this
                # while the arm is MOVING and the number is meaningless.
                try:
                    t = self.tf_buffer.lookup_transform(
                        'world', 'ur5e_tool0', rclpy.time.Time())
                except Exception as exc:
                    self.report(WARN, 'Kinematics agrees with TF',
                                f'no world->ur5e_tool0 transform '
                                f'({type(exc).__name__})')
                    t = None
                if t is not None:
                    tf_p = np.array([t.transform.translation.x,
                                     t.transform.translation.y,
                                     t.transform.translation.z])
                    fk_p = K.fk(q)[:3, 3]
                    err = float(np.linalg.norm(fk_p - tf_p))
                    if err < 0.001:
                        self.report(OK, 'Kinematics agrees with TF',
                                    f'fk() and world->ur5e_tool0 differ by '
                                    f'{err * 1000:.4f} mm (valid only because '
                                    f'the arm is still — see check 10)')
                    else:
                        self.report(BAD, 'Kinematics agrees with TF',
                                    f'fk() says {np.round(fk_p, 4)}, TF says '
                                    f'{np.round(tf_p, 4)} — {err * 1000:.1f} mm apart',
                                    'kinematics.CHAIN no longer matches the URDF')

        # 11. Optional: actually move it. Off by default, because a
        #     diagnostic that moves a robot is a diagnostic people stop
        #     running near anything fragile.
        if self.active and angles is not None:
            self.report(*self._active_test(angles))

        print('=' * 62)
        if self.failures:
            print(f'{self.failures} check(s) FAILED — see the -> lines above.\n')
        else:
            print('All checks passed. Try it out:\n'
                  '    ros2 run agr_arm_examples 06_pick_and_place\n')
        return 1 if self.failures else 0

    def _active_test(self, angles):
        """Command a small, safe joint move and see whether the arm follows."""
        from agr_arm_tools.arm import Arm
        arm = Arm(self)
        if not arm.wait_for_state(5.0):
            return WARN, 'Arm responds to commands', 'no joint states for the test'
        target = list(angles)
        target[0] += 0.20                     # shoulder pan: nothing to hit
        moved = arm.move_joints(target, timeout=12.0)
        settled = arm.joints[0]
        arm.move_joints(angles, timeout=12.0)  # always put it back
        if moved and abs(settled - target[0]) < 0.02:
            return (OK, 'Arm responds to commands',
                    f'shoulder pan moved 0.20 rad and came back '
                    f'(error {abs(settled - target[0]):.4f} rad)')
        return (BAD, 'Arm responds to commands',
                f'asked for {target[0]:+.3f}, reached {settled:+.3f}',
                'the JointPositionController plugins are not loaded')


def main() -> int:
    rclpy.init()
    node = Diagnose()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
