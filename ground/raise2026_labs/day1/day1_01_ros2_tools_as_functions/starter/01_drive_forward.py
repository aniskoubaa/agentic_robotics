#!/usr/bin/env python3
"""
RAISE 2026 — Lab 1, Script 01 — interactive drive menu.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

WHAT  Show a menu of motion primitives and publish the matching Twist
      to /cmd_vel so the Husky drives, reverses, or rotates.

LEARN - A `Twist` has TWO 3-vectors: linear (m/s) and angular (rad/s).
        Mobile robots usually only use `linear.x` (forward/back) and
        `angular.z` (yaw).
      - /cmd_vel is a STREAMED topic: the safety controller stops the
        robot if commands stop arriving, so we publish in a short loop.
      - A "stop" command is just an all-zeros Twist. Always send one
        at the end of every motion.

Run while `raise-sim` is up:

    ros2 run raise2026_labs 01_drive_forward.py    # or:  01_d1
"""

import math
import time

import rclpy
from rclpy.node import Node

# `Twist` carries a 6-DOF velocity command. Defined in geometry_msgs.
# `ros2 interface show geometry_msgs/msg/Twist` to see all fields.
from geometry_msgs.msg import Twist


# ─── Tunables — change these to see what happens ────────────────────────────
LINEAR_SPEED   = 0.4    # m/s   forward / backward speed
ANGULAR_SPEED  = 0.6    # rad/s rotation speed (≈ 34°/s)
RATE_HZ        = 10     # how many Twist messages per second while moving


def drive(pub: 'rclpy.publisher.Publisher',
          node: Node,
          linear_x: float,
          angular_z: float,
          duration_s: float) -> None:
    """Publish a constant Twist for `duration_s` seconds, then stop.

    This is the core helper every menu item uses. Why a helper?
    Because every motion follows the SAME 3-step pattern:
      1) build the Twist                 (what to do)
      2) publish it at RATE_HZ for N s   (how long)
      3) publish an all-zeros Twist      (stop safely)
    Factoring it out keeps each menu callback tiny.
    """
    cmd = Twist()
    cmd.linear.x  = linear_x
    cmd.angular.z = angular_z

    period = 1.0 / RATE_HZ
    n_ticks = int(duration_s * RATE_HZ)

    node.get_logger().info(
        f'  linear={linear_x:+.2f} m/s  angular={math.degrees(angular_z):+.0f}°/s'
        f'  for {duration_s:.1f}s'
    )

    # Loop publishing the same Twist. `spin_once` keeps ROS 2 alive
    # (timers, log flushing, …) while we wait one period.
    for _ in range(n_ticks):
        pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=period)

    # ALWAYS send a stop. If the script crashes mid-motion, the robot
    # keeps the last commanded speed — dangerous on real hardware.
    pub.publish(Twist())


# ─── Menu actions ──────────────────────────────────────────────────────────
# Each value is a callable taking (pub, node) and doing ONE motion.
# Adding a new motion = add a new lambda — the menu loop is generic.
ACTIONS = {
    '1': ('drive forward 1 m',  lambda p, n: drive(p, n, +LINEAR_SPEED,        0.0, 2.5)),
    '2': ('drive backward 1 m', lambda p, n: drive(p, n, -LINEAR_SPEED,        0.0, 2.5)),
    '3': ('rotate left 90°',    lambda p, n: drive(p, n,  0.0, +ANGULAR_SPEED, math.pi / 2 / ANGULAR_SPEED)),
    '4': ('rotate right 90°',   lambda p, n: drive(p, n,  0.0, -ANGULAR_SPEED, math.pi / 2 / ANGULAR_SPEED)),
    '5': ('curve forward+left', lambda p, n: drive(p, n, +LINEAR_SPEED, +ANGULAR_SPEED / 2, 3.0)),
    '6': ('emergency stop',     lambda p, n: drive(p, n,  0.0,  0.0, 0.3)),
}

MENU = """
┌─── RAISE 2026 · Drive menu ─────┐
│   1) drive forward 1 m          │
│   2) drive backward 1 m         │
│   3) rotate left 90°            │
│   4) rotate right 90°           │
│   5) curve forward+left         │
│   6) emergency stop             │
│   q) quit                       │
└─────────────────────────────────┘"""


def main():
    rclpy.init()
    node = Node('drive_menu')

    # `/cmd_vel` is the standard mobile-robot velocity topic. The Husky
    # listens to it via the DiffDrive plugin in raise2026_gazebo.urdf.xacro.
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # Give DDS (the ROS 2 middleware) a moment to discover the bridge.
    # Without this, the very first published message often gets dropped.
    time.sleep(0.4)

    while rclpy.ok():
        print(MENU)
        try:
            choice = input('  choose > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if choice == 'q':
            break

        if choice not in ACTIONS:
            print(f'  ⚠ unknown choice: {choice!r}')
            continue

        label, action = ACTIONS[choice]
        node.get_logger().info(f'→ {label}')
        action(pub, node)
        node.get_logger().info('  ✓ done')

    # Final safety stop on exit.
    pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
