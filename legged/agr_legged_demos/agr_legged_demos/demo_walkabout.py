#!/usr/bin/env python3
"""
Go2 — the five-minute demo.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_legged_demos demo_walkabout

WHAT  A scripted sequence that shows, in order, everything the platform can
      do: stand, walk, strafe, turn on the spot, and stop. Narrated on the
      terminal as it goes, so it can be run in front of an audience.

WHY a separate script from the examples: an example is optimised for READING
    — one idea, no distractions. A demo is optimised for WATCHING. They pull
    in opposite directions, so they are different files.

Point a camera viewer at it in another terminal for the full effect:
    ros2 run agr_legged_teleop camera_view
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

RATE_HZ = 20

# (label, vx, vy, wz, seconds) — the strafe is deliberately placed straight
# after the walk, because seeing the same body move sideways with no turn is
# the moment the "why legs?" question answers itself.
SEQUENCE = [
    ('Standing still — the gait holds a stance',   0.00,  0.00,  0.00, 3.0),
    ('Walking forward',                            0.20,  0.00,  0.00, 6.0),
    ('Strafing left — no wheeled robot does this', 0.00,  0.12,  0.00, 5.0),
    ('Strafing right',                             0.00, -0.12,  0.00, 5.0),
    ('Turning on the spot, left',                  0.00,  0.00,  0.80, 4.0),
    ('Turning on the spot, right',                 0.00,  0.00, -0.80, 4.0),
    ('Walking an arc',                             0.15,  0.00, -0.50, 6.0),
    ('Backing up',                                -0.15,  0.00,  0.00, 4.0),
    ('Stop',                                       0.00,  0.00,  0.00, 2.0),
]


def main() -> int:
    rclpy.init()
    node = Node('demo_walkabout')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # Wait for DDS discovery before asking who is listening. One second is
    # NOT enough — measured: `ros2 topic info` showed the gait controller
    # subscribed while a freshly created node still counted zero. Five
    # seconds, and even then only WARN: a false negative must not abort a
    # demo running in front of an audience.
    for _ in range(250):
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.count_subscribers('/cmd_vel'):
            break

    if node.count_subscribers('/cmd_vel') == 0:
        print('WARNING: nothing appears to be subscribed to /cmd_vel.\n'
              '         If the robot does not move, the gait controller is\n'
              '         not running:  ros2 run agr_legged_bringup gait\n')

    total = sum(s[-1] for s in SEQUENCE)
    print(f'\nGo2 walkabout — {len(SEQUENCE)} moves, {total:.0f} seconds\n'
          + '=' * 56)
    try:
        for i, (label, vx, vy, wz, secs) in enumerate(SEQUENCE, 1):
            print(f'{i}/{len(SEQUENCE)}  {label}')
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
            end = time.time() + secs
            while rclpy.ok() and time.time() < end:
                pub.publish(cmd)
                rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        # Leave the robot standing, never walking.
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.try_shutdown()
    print('=' * 56 + '\ndone.\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
