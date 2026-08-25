#!/usr/bin/env python3
"""
Go2 — diagnostics. Is this robot actually working?
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_legged_demos diagnose

WHAT  Nine checks, in the order things actually break, each printing what it
      saw and what to do about it. Run this FIRST whenever something is odd.

WHY   A legged sim fails in layers, and every layer looks the same from the
      top: "the robot does not move". Gazebo up but no model. Model spawned
      but controllers not activated. Controllers active but the gait node
      dead. Each has a different fix and none of them is obvious from the
      symptom, so the useful thing a diagnostic does is tell you WHICH layer.

Exit code is 0 when everything passed, 1 otherwise — so it also works as a
smoke test in a script.
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, JointState

JOINTS = [f'{leg}_{part}_joint'
          for leg in ('FL', 'FR', 'RL', 'RR')
          for part in ('hip', 'thigh', 'calf')]

OK, BAD, WARN = '  OK  ', ' FAIL ', ' WARN '


class Diagnose(Node):
    def __init__(self) -> None:
        super().__init__('go2_diagnose')
        self.failures = 0
        self.step = 0

    # ── reporting ─────────────────────────────────────────────────────────
    def report(self, tag: str, title: str, detail: str, fix: str = '') -> None:
        self.step += 1
        print(f'[{tag}] {self.step}. {title}')
        if detail:
            print(f'        {detail}')
        if tag == BAD:
            self.failures += 1
            if fix:
                print(f'        → {fix}')

    def collect(self, msg_type, topic, seconds: float = 3.0, qos: int = 10) -> list:
        """Every message seen on `topic` within `seconds`. Empty means the
        topic is dead — which is different from 'the topic does not exist'."""
        got: list = []
        sub = self.create_subscription(msg_type, topic, got.append, qos)
        end = time.time() + seconds
        try:
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
        finally:
            self.destroy_subscription(sub)
        return got

    def topics(self) -> dict:
        return dict(self.get_topic_names_and_types())

    # ── the checks ────────────────────────────────────────────────────────
    def run(self) -> int:
        print('\nGo2 diagnostics\n' + '=' * 60)

        # 1. Is Gazebo running at all? /clock is the earliest sign of life.
        clock = self.collect(Clock, '/clock', 2.0)
        if clock:
            self.report(OK, 'Gazebo is running',
                        f'/clock ticking, {len(clock)} msgs in 2 s')
        else:
            self.report(BAD, 'Gazebo is running', 'no /clock messages',
                        'start it:  agr-sim legged')
            print('\nNothing below can pass without a simulator. Stopping.\n')
            return 1

        # Only NOW list the graph. get_topic_names_and_types() reports what
        # this node has discovered so far, and DDS discovery is not instant:
        # asking on the first line of main() reliably reports topics as absent
        # that are provably alive two checks later. The /clock wait above is
        # what gives discovery time to finish.
        names = self.topics()

        # 2. Did the robot spawn? robot_description is published latched by
        #    robot_state_publisher, so its absence means the xacro failed.
        if '/robot_description' in names or '/joint_states' in names:
            self.report(OK, 'Robot description loaded',
                        'robot_state_publisher is up')
        else:
            self.report(BAD, 'Robot description loaded', 'no /robot_description',
                        'the xacro failed to expand — check the launch output')

        # 3. Controllers. This is the step that fails most often, because the
        #    spawners run in sequence and a slow machine can trip the ordering.
        ctrl_topics = [t for t in names if 'controller' in t]
        if '/joint_group_position_controller/commands' in names:
            self.report(OK, 'Position controller active',
                        f'{len(ctrl_topics)} controller topics')
        else:
            self.report(BAD, 'Position controller active',
                        'no /joint_group_position_controller/commands',
                        'ros2 control list_controllers   (expect both active)')

        # 4. Joint feedback — all twelve, not just some.
        js = self.collect(JointState, '/joint_states', 2.0)
        if not js:
            self.report(BAD, 'Joint states arriving', 'nothing on /joint_states',
                        'joint_state_broadcaster is not active')
        else:
            have = set(js[-1].name)
            missing = [j for j in JOINTS if j not in have]
            rate = len(js) / 2.0
            if missing:
                self.report(BAD, 'Joint states arriving',
                            f'{len(have)} joints, missing {missing}',
                            'the URDF and go2_controllers.yaml disagree')
            else:
                self.report(OK, 'Joint states arriving',
                            f'all 12 joints, {rate:.0f} Hz')

        # 5. Posture. A robot on its back still reports joint states happily.
        imu = self.collect(Imu, '/imu', 2.0)
        if not imu:
            self.report(WARN, 'IMU arriving', 'nothing on /imu — bridge missing?')
        else:
            q = imu[-1].orientation
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                              1 - 2 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            att = f'roll {math.degrees(roll):+.1f}°, pitch {math.degrees(pitch):+.1f}°'
            if abs(roll) < 0.35 and abs(pitch) < 0.35:
                self.report(OK, 'Robot is upright', f'{att}, {len(imu) / 2.0:.0f} Hz')
            else:
                self.report(BAD, 'Robot is upright', f'{att} — it has fallen',
                            'respawn it:  agr-stop && agr-sim legged')

        # 6. Camera.
        img = self.collect(Image, '/front_camera', 3.0)
        if img:
            m = img[-1]
            self.report(OK, 'Camera streaming',
                        f'{m.width}x{m.height} {m.encoding}, {len(img) / 3.0:.1f} Hz')
        else:
            self.report(BAD, 'Camera streaming', 'nothing on /front_camera',
                        'the world needs gz-sim-sensors-system; see '
                        'agr_inspection.sdf')

        # 7. Is anything listening to /cmd_vel? Without the gait controller,
        #    publishing velocity commands is silently a no-op.
        n_gait = self.count_subscribers('/cmd_vel')
        if n_gait > 0:
            self.report(OK, 'Gait controller listening',
                        f'{n_gait} subscriber(s) on /cmd_vel')
        else:
            self.report(BAD, 'Gait controller listening',
                        'nobody subscribes to /cmd_vel — the robot cannot walk',
                        'ros2 run agr_legged_bringup gait')

        # 8. Does the gait controller actually command the joints? A node can
        #    be alive and still be publishing nothing.
        from std_msgs.msg import Float64MultiArray
        cmds = self.collect(Float64MultiArray,
                            '/joint_group_position_controller/commands', 2.0)
        if cmds and len(cmds[-1].data) == 12:
            self.report(OK, 'Joint commands flowing',
                        f'{len(cmds) / 2.0:.0f} Hz, 12 values per message')
        elif cmds:
            self.report(BAD, 'Joint commands flowing',
                        f'{len(cmds[-1].data)} values, expected 12',
                        'a publisher disagrees with the controller about the '
                        'joint list')
        else:
            self.report(WARN, 'Joint commands flowing',
                        'no commands — normal if the gait is idle and holding')

        # 9. Sim time. Nodes started without use_sim_time compare wall-clock
        #    stamps against sim-clock stamps and silently misbehave.
        if clock:
            t = clock[-1].clock
            self.report(OK, 'Simulation clock',
                        f'sim time {t.sec}.{t.nanosec // 10**6:03d}s')

        print('=' * 60)
        if self.failures:
            print(f'{self.failures} check(s) FAILED — see the → lines above.\n')
        else:
            print('All checks passed. The robot is ready to walk:\n'
                  '    ros2 run agr_legged_teleop teleop_keyboard\n')
        return 1 if self.failures else 0

    def count_subscribers(self, topic: str) -> int:
        try:
            return self.count_subscribers_impl(topic)
        except Exception:
            return 0

    def count_subscribers_impl(self, topic: str) -> int:
        # Node.count_subscribers exists in Jazzy; guard anyway so a rename in
        # a future distro degrades to a warning rather than a traceback.
        return Node.count_subscribers(self, topic)


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
