#!/usr/bin/env python3
"""
TurtleBot — diagnostics. Is this robot actually working?
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_tb_demos diagnose
    ros2 run agr_tb_demos diagnose --ros-args -p active:=true    # really drives it

WHAT  Checks in the order things actually break, each printing what it saw
      and what to do about it. Run this FIRST whenever something is odd.

WHY   We did not write this simulator — it is the official TurtleBot stack,
      installed by apt — so most of what can go wrong is a MISMATCH between
      what upstream publishes and what our scripts expect. The checks below
      are exactly the mismatches that cost time on this machine:

        - /cmd_vel is TwistStamped under Jazzy. Publish a Twist and nothing
          moves and nothing complains.
        - the Create 3's status topics are BEST_EFFORT; a default RELIABLE
          subscription receives nothing at all.
        - a TurtleBot 4 spawns DOCKED and will not drive until undocked.
        - launched from a snap-polluted shell, TurtleBot 4's Gazebo GUI dies
          on a libpthread symbol and takes the diffdrive_controller spawner
          with it, leaving a sim with no /odom. `agr-sim` prevents that; a
          bare `ros2 launch` does not.

Exit code is 0 when everything passed, so it doubles as a smoke test.
"""
from __future__ import annotations

import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan

from agr_tb_tools import registry
from agr_tb_tools.base import SENSOR_QOS, Base

OK, BAD, WARN = '  OK  ', ' FAIL ', ' WARN '


class Diagnose(Node):
    def __init__(self):
        super().__init__('tb_diagnose',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('active', False)
        self.active = bool(self.get_parameter('active').value)
        self.robot = registry.get(
            os.environ.get('AGR_TB_MODEL') or registry.DEFAULT)
        self.failures = 0
        self.step = 0

    def report(self, tag, title, detail, fix=''):
        self.step += 1
        print(f'[{tag}] {self.step}. {title}')
        if detail:
            print(f'        {detail}')
        if tag == BAD:
            self.failures += 1
            if fix:
                print(f'        -> {fix}')

    def wait_for(self, msg_type, topic, timeout, qos=10):
        """Spin until the FIRST message, up to timeout. Not the same as
        sampling a fixed window: right after a launch, DDS discovery has not
        finished and a short fixed window reports a live topic as dead."""
        got = []
        sub = self.create_subscription(msg_type, topic, got.append, qos)
        end = time.time() + timeout
        try:
            while rclpy.ok() and time.time() < end and not got:
                rclpy.spin_once(self, timeout_sec=0.05)
        finally:
            self.destroy_subscription(sub)
        return got

    def collect(self, msg_type, topic, seconds, qos=10):
        got = []
        sub = self.create_subscription(msg_type, topic, got.append, qos)
        end = time.time() + seconds
        try:
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
        finally:
            self.destroy_subscription(sub)
        return got

    def run(self) -> int:
        r = self.robot
        print(f'\nTurtleBot diagnostics — {r.key}\n' + '=' * 62)
        print(f'  {r.description}')
        print(f'  upstream: {r.launch_package}/{r.launch_file}\n')

        # 1. Simulator alive.
        if not self.wait_for(Clock, '/clock', 10.0):
            self.report(BAD, 'Simulator is running', 'no /clock in 10 s',
                        f'start it:  agr-sim turtlebot model:={r.key}')
            print('\nNothing below can pass without a simulator. Stopping.\n')
            return 1
        clock = self.collect(Clock, '/clock', 2.0)
        self.report(OK, 'Simulator is running', f'/clock at {len(clock) / 2:.0f} Hz')

        topics = dict(self.get_topic_names_and_types())

        # 2. cmd_vel EXISTS and is the type we are going to publish.
        actual = topics.get(r.cmd_vel)
        if not actual:
            self.report(BAD, 'Drive topic present', f'no {r.cmd_vel}',
                        'the upstream launch did not come up — check its output')
        else:
            got = actual[0].split('/')[-1]
            if got == r.cmd_vel_type:
                self.report(OK, 'Drive topic type matches',
                            f'{r.cmd_vel} is {got}, which is what we publish')
            else:
                self.report(BAD, 'Drive topic type matches',
                            f'{r.cmd_vel} is {got}, registry says {r.cmd_vel_type}',
                            'fix cmd_vel_type in agr_tb_tools/config/robots.yaml — '
                            'publishing the wrong type moves nothing and warns nobody')

        # 3. Odometry.
        odom = self.collect(Odometry, r.odom, 2.0)
        if odom:
            p = odom[-1].pose.pose.position
            self.report(OK, 'Odometry arriving',
                        f'{len(odom) / 2:.0f} Hz, at ({p.x:+.2f}, {p.y:+.2f})')
        else:
            self.report(BAD, 'Odometry arriving', f'nothing on {r.odom}',
                        'on a TurtleBot 4 this usually means the '
                        'diffdrive_controller spawner died — see check 8')

        # 4. Lidar.
        scan = self.collect(LaserScan, r.scan, 3.0, SENSOR_QOS)
        if scan:
            s = scan[-1]
            valid = sum(1 for v in s.ranges if math.isfinite(v))
            self.report(OK, 'Lidar streaming',
                        f'{len(s.ranges)} beams, {valid} with a return, '
                        f'{len(scan) / 3:.1f} Hz')
        else:
            self.report(BAD, 'Lidar streaming', f'nothing on {r.scan}',
                        'check the QoS — Create 3 sensors are BEST_EFFORT')

        # 5. Camera, if this robot has one.
        if not r.has_camera:
            self.report(OK, 'Camera', f'{r.key} has none — correct for this robot')
        else:
            img = self.collect(Image, r.camera, 3.0)
            if img:
                m = img[-1]
                self.report(OK, 'Camera streaming',
                            f'{m.width}x{m.height} {m.encoding}, '
                            f'{len(img) / 3:.1f} Hz')
            else:
                self.report(BAD, 'Camera streaming', f'nothing on {r.camera}',
                            'the registry topic may be wrong for this model')

        # 6/7. TurtleBot 4 only: the Create 3 layer.
        if r.is_tb4:
            bot = Base(self)
            bot.sleep(2.5)
            if bot.is_docked is None:
                self.report(BAD, 'Create 3 status arriving',
                            'nothing on /dock_status',
                            'these topics are BEST_EFFORT — a default RELIABLE '
                            'subscription receives nothing')
            else:
                self.report(OK, 'Create 3 status arriving',
                            f'is_docked={bot.is_docked}, hazards={bot.hazards or "none"}')
                if bot.is_docked:
                    self.report(WARN, 'Robot is free to drive',
                                'it is DOCKED — velocity commands will be ignored '
                                'until it undocks (Base.ensure_ready does this)')
                else:
                    self.report(OK, 'Robot is free to drive', 'undocked')

            names = [n for n, _ in self.get_service_names_and_types()]
            has_undock = any('/undock' in n for n in names)
            self.report(OK if has_undock else WARN, 'Create 3 actions present',
                        'undock/dock/drive_distance available' if has_undock
                        else 'no /undock — the irobot_create nodes may not be up')

        # 8. Actually drive it.
        if self.active:
            self.report(*self._active_test())
        else:
            print('        (add -p active:=true to actually drive the robot)')

        print('=' * 62)
        if self.failures:
            print(f'{self.failures} check(s) FAILED — see the -> lines above.\n')
        else:
            print('All checks passed. Try it out:\n'
                  '    ros2 run agr_tb_teleop teleop_keyboard\n'
                  '    ros2 run agr_tb_examples 05_drive_a_square\n')
        return 1 if self.failures else 0

    def _active_test(self):
        bot = Base(self)
        if not bot.wait_for_state(10.0):
            return WARN, 'Robot responds to commands', 'no odometry for the test'
        bot.ensure_ready()
        result = bot.drive(0.12, seconds=2.5)
        back = bot.drive(-0.12, seconds=2.5)
        moved = result['moved']
        if result['reflex_reversed']:
            return (WARN, 'Robot responds to commands',
                    f'moved {moved:.3f} m, but a REFLEX drove it backwards '
                    f'mid-command — it is probably too close to the dock')
        if moved > 0.05:
            return (OK, 'Robot responds to commands',
                    f'drove {moved:.3f} m out and {back["moved"]:.3f} m back, '
                    f'peak {result["measured_max"]:.2f} m/s')
        return (BAD, 'Robot responds to commands',
                f'commanded 0.12 m/s for 2.5 s and moved {moved:.3f} m',
                'if this is a TurtleBot 4, it is probably still docked')


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
