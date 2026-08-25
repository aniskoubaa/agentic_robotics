#!/usr/bin/env python3
"""
Go2 example 06 — walk a square.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_legged_examples 06_walk_a_square

WHAT  Four sides and four 90-degree turns, using nothing but timed /cmd_vel.

LEARN - This is DEAD RECKONING: "drive for T seconds, therefore I have moved
        v*T metres". It is the simplest possible way to reach a place, and it
        is wrong. Watch where the robot finishes.
      - It is wrong because commanded speed is not achieved speed. The gait
        reaches about 85% of what you ask forward and about 90% in yaw — the
        feet slip a little on every step. Those percentages compound over
        eight legs of a square.
      - The fix is not a better constant. It is CLOSING THE LOOP: measure
        where you actually are (odometry, IMU, a camera) and correct. That is
        exactly what the RaiseBot's navigation_server does, and why it
        re-anchors against ground truth instead of trusting its own odometry.
      - So: run this, see the square that is not a square, and you have the
        motivation for everything in the labs that follows. The script
        measures its OWN heading error from the IMU at the end, because a
        lesson you can read off the terminal beats one you have to take on
        trust. A clean run lands ~10 deg off; if the robot clips something
        it is far worse.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu

SIDE_M    = 1.0
WALK_VX   = 0.20    # m/s
TURN_WZ   = 0.60    # rad/s
RATE_HZ   = 20
SETTLE_S  = 1.0     # pause between legs so the gait finishes its cycle


def send(node, pub, vx, wz, secs, label):
    print(f'  {label:<22} {secs:4.1f}s')
    cmd = Twist()
    cmd.linear.x, cmd.angular.z = vx, wz
    for _ in range(int(secs * RATE_HZ)):
        pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)
    pub.publish(Twist())
    for _ in range(int(SETTLE_S * RATE_HZ)):
        rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)


def yaw_of(msg: Imu) -> float:
    q = msg.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = Node('walk_a_square')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # Track heading so the script can grade itself. Four 90-degree left turns
    # must come back to the starting heading; whatever is left over is the
    # accumulated error, and it is never zero.
    heading = {}
    node.create_subscription(Imu, '/imu',
                             lambda m: heading.__setitem__('yaw', yaw_of(m)), 10)
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.02)
        if 'yaw' in heading:
            break
    start_yaw = heading.get('yaw')

    walk_s = SIDE_M / WALK_VX
    turn_s = (math.pi / 2) / TURN_WZ
    print(f'square: {SIDE_M} m sides, {walk_s:.1f}s walking + {turn_s:.1f}s '
          f'turning per corner\n')
    try:
        for i in range(4):
            send(node, pub, WALK_VX, 0.0, walk_s, f'side {i + 1} forward')
            send(node, pub, 0.0, TURN_WZ, turn_s, f'corner {i + 1} turn left')
    finally:
        pub.publish(Twist())
        for _ in range(50):                 # let the last stance settle
            rclpy.spin_once(node, timeout_sec=0.02)
        end_yaw = heading.get('yaw')
        node.destroy_node()
        rclpy.try_shutdown()

    print()
    if start_yaw is None or end_yaw is None:
        print('no /imu — cannot grade the run.')
    else:
        err = math.degrees((end_yaw - start_yaw + math.pi) % (2 * math.pi) - math.pi)
        print(f'heading at start {math.degrees(start_yaw):+7.1f}°')
        print(f'heading at end   {math.degrees(end_yaw):+7.1f}°')
        print(f'ERROR            {err:+7.1f}°  after four 90° turns that should '
              f'have cancelled')
        print('\nThat is dead reckoning. Every step slipped a little, nothing '
              'measured it,\nand nothing corrected it. Closing the loop is the '
              'rest of the course.')


if __name__ == '__main__':
    main()
