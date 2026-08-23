# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""Move the Go2 between named 12-joint poses.

    ros2 run agr_legged_bringup stand              # stand up
    ros2 run agr_legged_bringup stand --pose tuck  # fold the legs
    ros2 run agr_legged_bringup stand --pose crouch

Why this exists: a quadruped spawned with zero joint angles has its legs
straight out sideways and simply falls over. Something has to command a stance
before the robot is useful, and without a locomotion controller that something
is this. It ramps rather than stepping — a step command on 12 position joints
makes the robot launch itself off the ground.
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Joint order must match go2_controllers.yaml exactly — the controller takes a
# flat array, so a mismatch silently drives the wrong joints.
JOINTS = [f'{leg}_{part}_joint'
          for leg in ('FL', 'FR', 'RL', 'RR')
          for part in ('hip', 'thigh', 'calf')]

# (hip, thigh, calf) per leg. The calf limit is [-2.7227, -0.83776] — it never
# straightens, so every pose below keeps it negative.
STAND_POSE = [0.0, 0.80, -1.55] * 4
CROUCH_POSE = [0.0, 1.25, -2.45] * 4
TUCK_POSE = [0.0, 1.45, -2.70] * 4

POSES = {'stand': STAND_POSE, 'crouch': CROUCH_POSE, 'tuck': TUCK_POSE}


class Stand(Node):
    def __init__(self, target: list[float], seconds: float) -> None:
        super().__init__('go2_stand')
        self._pub = self.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10)
        self._target = target
        self._steps = max(1, int(seconds * 50.0))
        self._i = 0
        # Start from the tuck rather than from wherever the sim happens to be:
        # we cannot read the current position without a state subscription, and
        # ramping from a known folded pose is safe from any starting condition.
        self._start = TUCK_POSE
        self.create_timer(0.02, self._tick)
        self.get_logger().info(
            f'ramping to target over {seconds:.1f}s ({self._steps} steps)')

    def _tick(self) -> None:
        if self._i > self._steps:
            return
        a = self._i / self._steps
        msg = Float64MultiArray()
        msg.data = [s + (t - s) * a for s, t in zip(self._start, self._target)]
        self._pub.publish(msg)
        self._i += 1
        if self._i == self._steps + 1:
            self.get_logger().info('pose reached — holding')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--pose', default='stand', choices=sorted(POSES))
    ap.add_argument('--seconds', type=float, default=2.0)
    ap.add_argument('--hold', type=float, default=0.0,
                    help='seconds to keep holding after the ramp; 0 = forever')
    args, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=ros_args)
    node = Stand(POSES[args.pose], args.seconds)
    try:
        if args.hold > 0:
            end = node.get_clock().now().nanoseconds + int((args.seconds + args.hold) * 1e9)
            while rclpy.ok() and node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.05)
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
