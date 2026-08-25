#!/usr/bin/env python3
"""
RaiseBot — diagnostics. Is this robot actually working?
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run raisebot_demos diagnose

WHAT  Ten checks, in the order things actually break, each printing what it
      saw and what to do about it. Run this FIRST whenever something is odd.

WHY   The ground stack fails in layers and every layer looks the same from the
      top — "the robot does nothing". Gazebo up but the bridge down. Bridge up
      but the robot spawned inside a plant. Sensors fine but no tool servers,
      so every service call blocks forever. Each has a different fix and none
      is obvious from the symptom.

Exit code is 0 when everything passed, 1 otherwise, so this also works as a
smoke test in a script.
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState, LaserScan

OK, BAD, WARN, INFO = '  OK  ', ' FAIL ', ' WARN ', ' INFO '

CAMERAS = [('wrist RGB', '/wrist_camera/image_raw'),
           ('wrist depth', '/wrist_camera/depth/image_raw'),
           ('PTZ', '/ptz_camera/image_raw')]

# One representative service per tool server, so the check can name the
# server that is missing rather than just counting.
TOOL_SERVICES = [('navigation_server', '/nav_to_home'),
                 ('move_to_pose_server', '/move_to_home'),
                 ('gripper_server', '/open_gripper'),
                 ('detector_server', '/detect_wrist'),
                 ('inspector_server', '/inspect_plant')]

# grasp_server is deliberately NOT in that list: it exposes no services at
# all. It watches the gripper joints in /joint_states and attaches a tomato
# when they close, so that a human teleoperator, a script and a trained policy
# all grasp identically. Probing it for a service reports a healthy node as
# missing — which this diagnostic did until it was corrected.
GRASP_NODE = 'grasp_server'


class Diagnose(Node):
    def __init__(self) -> None:
        super().__init__('raisebot_diagnose')
        self.failures = 0
        self.step = 0

    def report(self, tag: str, title: str, detail: str = '', fix: str = '') -> None:
        self.step += 1
        print(f'[{tag}] {self.step}. {title}')
        if detail:
            print(f'        {detail}')
        if tag == BAD:
            self.failures += 1
        if fix and tag in (BAD, WARN):
            print(f'        → {fix}')

    def collect(self, msg_type, topic, seconds=2.0, qos=10) -> list:
        got: list = []
        sub = self.create_subscription(msg_type, topic, got.append, qos)
        end = time.time() + seconds
        try:
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
        finally:
            self.destroy_subscription(sub)
        return got

    def settle(self, seconds: float = 2.0) -> None:
        """Let DDS discovery finish before asking what exists. Calling
        get_topic_names_and_types() straight away reports a live system as
        having no topics at all — the most misleading output possible."""
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self) -> int:
        print('\nRaiseBot diagnostics\n' + '=' * 64)
        self.settle()

        # 1. Simulator.
        clock = self.collect(Clock, '/clock', 2.0)
        if clock:
            self.report(OK, 'Gazebo is running',
                        f'/clock ticking, {len(clock)} msgs in 2 s')
        else:
            self.report(BAD, 'Gazebo is running', 'no /clock',
                        'agr-sim ground')
            print('\nNothing below can pass without a simulator. Stopping.\n')
            return 1

        names = dict(self.get_topic_names_and_types())

        # 2. The ros_gz bridge. Everything on this robot crosses it.
        bridged = [t for t in ('/scan', '/odom', '/joint_states', '/cmd_vel')
                   if t in names]
        if len(bridged) == 4:
            self.report(OK, 'ROS ↔ Gazebo bridge', 'all four core topics present')
        else:
            missing = set(('/scan', '/odom', '/joint_states', '/cmd_vel')) - set(bridged)
            self.report(BAD, 'ROS ↔ Gazebo bridge', f'missing {sorted(missing)}',
                        'check ros_gz_bridge.yaml and the launch output')

        # 3. Odometry, and where the robot thinks it is.
        odom = self.collect(Odometry, '/odom', 2.0)
        if odom:
            p = odom[-1].pose.pose.position
            self.report(OK, 'Odometry', f'{len(odom) / 2.0:.0f} Hz, '
                                        f'x={p.x:+.2f} y={p.y:+.2f} '
                                        f'(relative to the SPAWN point, not the world)')
        else:
            self.report(BAD, 'Odometry', 'nothing on /odom')

        # 4. LiDAR — and how much of it is usable.
        scan = self.collect(LaserScan, '/scan', 2.0)
        if not scan:
            self.report(BAD, 'LiDAR', 'nothing on /scan')
        else:
            s = scan[-1]
            hits = [r for r in s.ranges
                    if math.isfinite(r) and s.range_min <= r <= s.range_max]
            nearest = min(hits) if hits else math.inf
            detail = (f'{len(s.ranges)} beams at {len(scan) / 2.0:.1f} Hz, '
                      f'{len(hits)} return a range, nearest {nearest:.2f} m')
            # Beams at the minimum range are the robot seeing its own chassis.
            if nearest <= s.range_min + 0.02:
                detail += ' (that is the robot\'s own body)'
            self.report(OK, 'LiDAR', detail)

        # 5. Joints — arm, gripper and PTZ all report here.
        js = self.collect(JointState, '/joint_states', 2.0)
        if not js:
            self.report(BAD, 'Joint states', 'nothing on /joint_states')
        else:
            have = set(js[-1].name)
            groups = {'UR5e arm': 'ur5e_', 'gripper': 'gripper_', 'PTZ': 'ptz_'}
            counts = {g: sum(1 for n in have if n.startswith(p))
                      for g, p in groups.items()}
            missing = [g for g, c in counts.items() if c == 0]
            detail = ', '.join(f'{g} {c}' for g, c in counts.items())
            if missing:
                self.report(BAD, 'Joint states', f'{detail} — no {missing}',
                            'the URDF did not expand fully')
            else:
                self.report(OK, 'Joint states', f'{len(have)} joints — {detail}')

        # 6. Cameras.
        alive, dead = [], []
        for label, topic in CAMERAS:
            got = self.collect(Image, topic, 2.0)
            (alive if got else dead).append(
                f'{label} {got[-1].width}x{got[-1].height}' if got else label)
        if not dead:
            self.report(OK, 'Cameras', '; '.join(alive))
        else:
            self.report(BAD, 'Cameras', f'live: {alive or "none"}; dead: {dead}',
                        'rendering failed — check for EGL errors in the sim log')

        # 7. Is anything listening to /cmd_vel? Without the bridge subscribing,
        #    driving is a silent no-op.
        n = self.count_subscribers('/cmd_vel')
        if n:
            self.report(OK, 'Drive command path', f'{n} subscriber(s) on /cmd_vel')
        else:
            self.report(BAD, 'Drive command path', 'nobody subscribes to /cmd_vel',
                        'the ros_gz bridge is not running')

        # 8. Tool servers. Their absence is the single most common reason a
        #    lab "hangs" — a service call with no server blocks forever.
        from std_srvs.srv import Trigger
        up, down = [], []
        for server, service in TOOL_SERVICES:
            client = self.create_client(Trigger, service)
            (up if client.wait_for_service(timeout_sec=1.0) else down).append(server)
            self.destroy_client(client)
        # Check the service-less one by node name instead.
        (up if GRASP_NODE in self.get_node_names() else down).append(GRASP_NODE)
        if not down:
            self.report(OK, 'Tool servers', f'all {len(up)} responding')
        elif up:
            self.report(WARN, 'Tool servers', f'up: {up}; MISSING: {down}',
                        'ros2 launch raisebot_bringup tools.launch.py')
        else:
            self.report(WARN, 'Tool servers', 'none are running',
                        'start them:  agr-sim ground tools:=true\n'
                        '        (without them every service call blocks forever)')

        # 9. Posture. A robot on its side still publishes perfectly good
        #    sensor data.
        if odom:
            q = odom[-1].pose.pose.orientation
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                              1 - 2 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            att = f'roll {math.degrees(roll):+.1f}°, pitch {math.degrees(pitch):+.1f}°'
            if abs(roll) < 0.35 and abs(pitch) < 0.35:
                self.report(OK, 'Robot is upright', att)
            else:
                self.report(BAD, 'Robot is upright', f'{att} — it has tipped over',
                            'agr-stop && agr-sim ground')

        # 10. Sim clock, so stale-time bugs are visible.
        t = clock[-1].clock
        self.report(OK, 'Simulation clock',
                    f'sim time {t.sec}.{t.nanosec // 10**6:03d}s')

        print('=' * 64)
        if self.failures:
            print(f'{self.failures} check(s) FAILED — see the → lines above.\n')
        else:
            print('All checks passed. Try:\n'
                  '    ros2 run raisebot_examples 02_read_lidar\n'
                  '    ros2 run raisebot_teleop teleop_keyboard\n')
        return 1 if self.failures else 0

    def count_subscribers(self, topic: str) -> int:
        try:
            return Node.count_subscribers(self, topic)
        except Exception:
            return 0


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
