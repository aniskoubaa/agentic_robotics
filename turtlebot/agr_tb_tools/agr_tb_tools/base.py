#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
Base — one object that drives a TurtleBot, whichever TurtleBot it is.

Same shape as agr_arm_tools.arm.Arm and agr_uav_tools.offboard.Pilot: it
WRAPS a node rather than subclassing one, and everything blocking spins that
node itself, so no example needs an executor or a thread.

    import rclpy
    from rclpy.node import Node
    from agr_tb_tools.base import Base

    rclpy.init()
    node = Node('my_script')
    bot = Base(node)                 # reads AGR_TB_MODEL, or pass model='tb4_lite'
    bot.wait_for_state()
    bot.ensure_ready()               # undocks a TurtleBot 4; no-op on a TurtleBot 3
    bot.drive(0.2, seconds=3.0)
    bot.turn(math.pi / 2)
    bot.stop()

WHAT THE HARDWARE MADE US HANDLE
Four things in here are not design choices, they are things measured on the
running robots that any script has to get right or silently do nothing:

1. /cmd_vel is geometry_msgs/TwistStamped on BOTH robots under Jazzy, not
   Twist. Publish a Twist and the robot does not move and nothing complains.
2. A TurtleBot 4 spawns DOCKED and the Create 3 refuses to drive until the
   /undock action has completed (about 30 s). That is real hardware
   behaviour, faithfully simulated.
3. The Create 3's sensor topics are BEST_EFFORT. A default (RELIABLE)
   subscription to /dock_status receives literally nothing and rclpy warns
   about "incompatible QoS" once, quietly, at startup.
4. The Create 3 runs a REFLEX layer — REFLEX_DOCK_AVOID, REFLEX_CLIFF,
   REFLEX_BUMP and friends — which will override your velocity command and
   drive the robot backwards at 0.14 m/s if it thinks you are about to hit
   something or are too near the dock. Measured. `drive()` reports it rather
   than pretending the command was obeyed.
"""
from __future__ import annotations

import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, LaserScan

from agr_tb_tools import registry

# The Create 3 publishes its own status topics BEST_EFFORT. Subscribing with
# the default RELIABLE profile gets you nothing at all, plus one easily
# missed warning line.
SENSOR_QOS = QoSProfile(depth=10,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)

# Images get a depth of ONE. A TurtleBot3 Waffle publishes 1920x1080 rgb8,
# which is 6.2 MB per frame, so a depth-10 queue is 62 MB of images nobody is
# going to look at. Every consumer here wants the newest frame anyway.
IMAGE_QOS = QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)

PUBLISH_HZ = 20.0
SETTLE_STOP_S = 0.6          # keep publishing zero this long, so it really stops


class Base:
    """A differential-drive TurtleBot."""

    def __init__(self, node, model: str | None = None):
        self.node = node
        # The model is normally chosen once, by agr-sim, and passed down in
        # the environment. An explicit argument still wins, for scripts that
        # want to be specific.
        self.robot = registry.get(
            model or os.environ.get('AGR_TB_MODEL') or registry.DEFAULT)

        self._odom = None
        self._scan = None
        self._imu = None
        self._image = None
        self._image_count = 0
        self._dock = None
        self._hazards = []

        r = self.robot
        node.create_subscription(Odometry, r.odom, self._on_odom, 10)
        node.create_subscription(LaserScan, r.scan, self._on_scan, SENSOR_QOS)
        if r.imu:
            node.create_subscription(Imu, r.imu, self._on_imu, SENSOR_QOS)
        if r.camera:
            node.create_subscription(Image, r.camera, self._on_image, IMAGE_QOS)

        self._stamped = r.cmd_vel_type == 'TwistStamped'
        msg_type = TwistStamped if self._stamped else Twist
        self._cmd = node.create_publisher(msg_type, r.cmd_vel, 10)

        # TurtleBot 4 only: the Create 3 dock/undock actions and status.
        self._undock_client = None
        self._dock_client = None
        if r.is_tb4:
            self._setup_create3()

    # ── TurtleBot 4 extras ──────────────────────────────────────────────────
    def _setup_create3(self) -> None:
        """Wire up the Create 3 pieces. Import here, not at module scope, so a
        machine with only TurtleBot 3 installed can still use this class."""
        try:
            from irobot_create_msgs.action import Dock, Undock
            from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
            from rclpy.action import ActionClient
        except ImportError:
            self.node.get_logger().warn(
                'irobot_create_msgs is not installed — dock/undock and hazards '
                'are unavailable. Install ros-jazzy-turtlebot4-simulator.')
            return
        self.node.create_subscription(
            DockStatus, '/dock_status', self._on_dock, SENSOR_QOS)
        self.node.create_subscription(
            HazardDetectionVector, '/hazard_detection',
            self._on_hazards, SENSOR_QOS)
        self._undock_client = ActionClient(self.node, Undock, '/undock')
        self._dock_client = ActionClient(self.node, Dock, '/dock')

    def _on_dock(self, msg) -> None:
        self._dock = msg

    def _on_hazards(self, msg) -> None:
        self._hazards = [d.type for d in msg.detections]

    @property
    def is_docked(self):
        """True / False, or None when the robot has no dock to speak of."""
        return None if self._dock is None else bool(self._dock.is_docked)

    @property
    def hazards(self) -> list:
        return list(self._hazards)

    def _run_action(self, client, goal, timeout: float, label: str) -> bool:
        if client is None:
            return False
        if not client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().warn(f'no {label} action server')
            return False
        send = client.send_goal_async(goal)
        handle = self._spin_until(send, 15.0)
        if handle is None or not handle.accepted:
            # A rejected goal is usually "already in that state", which is
            # success as far as the caller is concerned.
            self.node.get_logger().info(f'{label} goal was rejected')
            return False
        return self._spin_until(handle.get_result_async(), timeout) is not None

    def _spin_until(self, future, timeout: float):
        end = time.time() + timeout
        while time.time() < end and rclpy.ok() and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return future.result() if future.done() else None

    def undock(self, timeout: float = 60.0) -> bool:
        """Drive off the charger. Measured at about 30 s on a TurtleBot 4."""
        from irobot_create_msgs.action import Undock
        return self._run_action(self._undock_client, Undock.Goal(),
                                timeout, 'undock')

    def dock(self, timeout: float = 120.0) -> bool:
        from irobot_create_msgs.action import Dock
        return self._run_action(self._dock_client, Dock.Goal(), timeout, 'dock')

    def wait_for_dock_status(self, timeout: float = 6.0) -> bool:
        """Wait for the first /dock_status. Returns False if none arrives.

        Worth its own method because the answer is genuinely three-valued and
        the difference matters: docked, not docked, or nobody has told us yet.
        One second is not enough — this topic is BEST_EFFORT and the Create 3
        nodes come up well after Gazebo does, so a short wait reports "no
        /dock_status" on a robot that is working perfectly.
        """
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self._dock is not None:
                return True
        return False

    def ensure_ready(self, timeout: float = 60.0) -> bool:
        """Get the robot into a state where it will actually move.

        On a TurtleBot 3 this does nothing at all. On a TurtleBot 4 it
        undocks, because a docked Create 3 ignores velocity commands — which
        looks exactly like a broken simulator if you do not know.
        """
        if not self.robot.is_tb4:
            return True
        if not self.wait_for_dock_status():
            self.node.get_logger().warn(
                'no /dock_status in 6 s — the Create 3 nodes may not be up. '
                'Carrying on; if the robot does not move, that is why.')
            return False
        if not self.is_docked:
            return True
        print('  robot is docked; undocking (this takes about 30 s) ...')
        ok = self.undock(timeout)
        print(f'  undocked: {self.is_docked is False}')
        return ok

    # ── State ───────────────────────────────────────────────────────────────
    def _on_odom(self, msg) -> None:
        self._odom = msg

    def _on_scan(self, msg) -> None:
        self._scan = msg

    def _on_imu(self, msg) -> None:
        self._imu = msg

    def _on_image(self, msg) -> None:
        self._image = msg
        self._image_count += 1

    def wait_for_state(self, timeout: float = 20.0) -> bool:
        """Block until odometry arrives. Worth calling first in every script:
        DDS discovery is not instant, and reading `pose` on the next line
        gives you None from a perfectly healthy robot."""
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self._odom is not None:
                return True
        return False

    @property
    def ready(self) -> bool:
        return self._odom is not None

    @property
    def pose(self):
        """(x, y) in the odom frame, or None."""
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        return (p.x, p.y)

    @property
    def yaw(self) -> float:
        if self._odom is None:
            return float('nan')
        q = self._odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @property
    def speed(self) -> float:
        if self._odom is None:
            return float('nan')
        return self._odom.twist.twist.linear.x

    @property
    def scan(self):
        return self._scan

    @property
    def image(self):
        return self._image

    @property
    def image_count(self) -> int:
        return self._image_count

    def range_ahead(self, half_angle: float = 0.26) -> float:
        """Closest lidar return within +/- half_angle of straight ahead.

        inf when nothing is in range, which is a real answer and not an error:
        a lidar reports no return for anything past its maximum range.
        """
        s = self._scan
        if s is None:
            return float('nan')
        best = float('inf')
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min:
                continue
            angle = s.angle_min + i * s.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= half_angle:
                best = min(best, r)
        return best

    # ── Motion ──────────────────────────────────────────────────────────────
    def _twist(self, v: float, w: float):
        if self._stamped:
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.twist.linear.x = float(v)
            msg.twist.angular.z = float(w)
            return msg
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        return msg

    def send(self, v: float = 0.0, w: float = 0.0) -> None:
        """Publish one velocity command. The controller does NOT latch — stop
        publishing and the robot stops — so most callers want drive()."""
        self._cmd.publish(self._twist(v, w))

    def drive(self, v: float = 0.2, w: float = 0.0, seconds: float = 2.0):
        """Hold a velocity for `seconds`, then stop. Returns what happened.

        The return is a dict, not a bool, because on a TurtleBot 4 "did it do
        what I asked" has a genuinely interesting answer: the Create 3's
        reflexes can and do override the command. `commanded` is what you
        asked for and `measured` is what the wheels reported.
        """
        v = max(-self.robot.max_speed, min(self.robot.max_speed, v))
        w = max(-self.robot.max_turn, min(self.robot.max_turn, w))
        start = self.pose
        speeds = []
        end = time.time() + seconds
        period = 1.0 / PUBLISH_HZ
        nxt = 0.0
        while time.time() < end and rclpy.ok():
            now = time.time()
            if now >= nxt:
                self.send(v, w)
                nxt = now + period
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.ready:
                speeds.append(self.speed)
        self.stop()
        moved = math.dist(start, self.pose) if (start and self.pose) else float('nan')
        reversed_by_reflex = any(s < -0.05 for s in speeds) and v > 0
        return {
            'commanded': v,
            'measured_max': max(speeds, default=float('nan')),
            'measured_min': min(speeds, default=float('nan')),
            'moved': moved,
            'seconds': seconds,
            'reflex_reversed': reversed_by_reflex,
        }

    def stop(self, seconds: float = SETTLE_STOP_S) -> None:
        """Publish zero for a moment. One zero message is not enough — it can
        be dropped, and then the robot keeps going."""
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            self.send(0.0, 0.0)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def turn(self, radians: float, w: float = 0.6, tol: float = 0.05,
             timeout: float = 30.0) -> float:
        """Turn by `radians` using odometry, and return the error.

        Closed loop on the measured heading rather than open loop on a timer,
        because a timer is wrong by whatever the controller's acceleration
        ramp costs — several degrees per turn, which compounds. The angle is
        UNWRAPPED as it accumulates: comparing raw yaw values across the
        +/-pi seam is the classic way to make a 180 degree turn read as a
        tiny one.
        """
        if not self.ready:
            return float('nan')
        target = abs(radians)
        sign = 1.0 if radians >= 0 else -1.0
        turned = 0.0
        last = self.yaw
        end = time.time() + timeout
        while turned < target - tol and time.time() < end and rclpy.ok():
            self.send(0.0, sign * abs(w))
            rclpy.spin_once(self.node, timeout_sec=0.02)
            now = self.yaw
            step = now - last
            step = math.atan2(math.sin(step), math.cos(step))   # unwrap
            turned += abs(step)
            last = now
        self.stop()
        return (turned - target) * sign

    def sleep(self, seconds: float) -> None:
        """Spin the node for `seconds` — never time.sleep() with ROS."""
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def describe(self) -> str:
        p = self.pose
        where = f'({p[0]:+.2f}, {p[1]:+.2f})' if p else '(no odom)'
        ahead = self.range_ahead()
        bits = [f'{self.robot.key}', where,
                f'yaw {math.degrees(self.yaw):+6.1f}deg',
                f'ahead {ahead:.2f} m' if math.isfinite(ahead) else 'ahead clear']
        if self.robot.is_tb4:
            bits.append(f'docked={self.is_docked}')
            if self._hazards:
                bits.append(f'hazards={self._hazards}')
        return '  '.join(bits)
