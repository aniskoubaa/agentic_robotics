#!/usr/bin/env python3
"""
Go2 example 02 — which way is up?
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_legged_examples 02_read_imu

WHAT  Read /imu and print roll and pitch, live, until Ctrl-C.

LEARN - A legged robot without an IMU has no idea it is falling. A wheeled
        robot mostly does not care. This is the sensor that separates them.
      - Orientation arrives as a QUATERNION (x, y, z, w) — four numbers, no
        gimbal lock, unreadable by humans. Converting to roll/pitch/yaw for
        DISPLAY is fine; doing maths in Euler angles is how you get a robot
        that misbehaves at exactly one orientation.
      - Drive the robot with teleop while this runs. Roll and pitch wobble by
        a couple of degrees with each step: that is the gait, seen from the
        inside.
"""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

# The IMU publishes at 200 Hz and the bridge delivers close to that. Printing
# every sample is unreadable on a terminal and megabytes of noise in a log, so
# throttle the DISPLAY. The subscription still sees every message — never
# throttle by dropping data you were asked to read.
PRINT_HZ = 10


def rpy(q) -> tuple:
    """Quaternion → (roll, pitch, yaw) in radians. The standard formulas —
    pitch is clamped because asin() of 1.0000001 raises near vertical."""
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                      1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def main():
    rclpy.init()
    node = Node('read_imu')

    last = [0.0]

    def on_imu(msg: Imu) -> None:
        now = time.time()
        if now - last[0] < 1.0 / PRINT_HZ:
            return
        last[0] = now
        r, p, y = rpy(msg.orientation)
        upright = 'upright' if abs(r) < 0.4 and abs(p) < 0.4 else 'FALLEN'
        print(f'\rroll {math.degrees(r):+6.1f}°  pitch {math.degrees(p):+6.1f}°  '
              f'yaw {math.degrees(y):+7.1f}°   {upright}   ', end='', flush=True)

    node.create_subscription(Imu, '/imu', on_imu, 10)
    print('reading /imu — Ctrl-C to stop\n')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print()
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
