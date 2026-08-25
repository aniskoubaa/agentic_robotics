#!/usr/bin/env python3
"""
Go2 — keyboard teleop for the quadruped.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

  ┌─────────────┬─────────────────────────┐
  │  w   or  ↑  │  walk forward           │
  │  x   or  ↓  │  walk backward          │
  │  a   or  ←  │  turn left  (on the spot)│
  │  d   or  →  │  turn right (on the spot)│
  │  q          │  STRAFE left            │
  │  e          │  STRAFE right           │
  │  s / SPACE  │  STOP                   │
  │  Ctrl-C     │  exit                   │
  └─────────────┴─────────────────────────┘

Hold the key; the robot walks while you hold it. Tap `s` to stop.

Same keys, same /cmd_vel topic, as the wheeled RaiseBot — with two extra.
`q` and `e` STRAFE: the Go2 walks sideways without turning. A differential
drive base physically cannot do that, and it is the clearest one-key
demonstration of what legs buy you.

Nothing here knows the robot has legs. This node publishes a Twist and stops;
`agr_legged_bringup/gait.py` is what turns that Twist into twelve joint
angles. Keeping the two apart is the point — swap the gait for a learned
policy and this file does not change.
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# Kept below the gait's own clamps (see gait.py MAX_VX/MAX_VY/MAX_WZ) so that
# what you press is what the robot does, rather than a request it silently
# truncates.
LINEAR_SPEED  = 0.20    # m/s   forward / backward
STRAFE_SPEED  = 0.12    # m/s   sideways
ANGULAR_SPEED = 0.80    # rad/s turn

# The gait controller stops on its own if /cmd_vel goes quiet (IDLE_STOP_S).
# We republish at this rate so that holding a key reads as a held command.
REPEAT_HZ = 20

# key → (linear.x, linear.y, angular.z) as multiples of the speeds above.
KEYS = {
    'w': (1.0, 0.0, 0.0),  '\x1b[A': (1.0, 0.0, 0.0),   # ↑ forward
    'x': (-1.0, 0.0, 0.0), '\x1b[B': (-1.0, 0.0, 0.0),  # ↓ backward
    'a': (0.0, 0.0, 1.0),  '\x1b[D': (0.0, 0.0, 1.0),   # ← turn left
    'd': (0.0, 0.0, -1.0), '\x1b[C': (0.0, 0.0, -1.0),  # → turn right
    'q': (0.0, 1.0, 0.0),                               # strafe left
    'e': (0.0, -1.0, 0.0),                              # strafe right
    's': (0.0, 0.0, 0.0),
    ' ': (0.0, 0.0, 0.0),
}


def read_key(timeout: float) -> str:
    """Non-blocking stdin read. '' if nothing, else the key or escape
    sequence ('\\x1b[A' is the up arrow)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        if not select.select([sys.stdin], [], [], timeout)[0]:
            return ''
        ch = sys.stdin.read(1)
        if ch == '\x1b':                       # ESC '[' X — an arrow key
            if select.select([sys.stdin], [], [], 0.01)[0]:
                ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = Node('teleop_keyboard')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    print(__doc__)
    print(f'  walk {LINEAR_SPEED} m/s   strafe {STRAFE_SPEED} m/s   '
          f'turn {ANGULAR_SPEED} rad/s\n')

    cmd = (0.0, 0.0, 0.0)
    try:
        while rclpy.ok():
            key = read_key(1.0 / REPEAT_HZ)
            if key == '\x03':                  # Ctrl-C
                break
            if key:
                cmd = KEYS.get(key, KEYS.get(key.lower(), cmd))
            msg = Twist()
            msg.linear.x  = cmd[0] * LINEAR_SPEED
            msg.linear.y  = cmd[1] * STRAFE_SPEED
            msg.angular.z = cmd[2] * ANGULAR_SPEED
            pub.publish(msg)
    finally:
        pub.publish(Twist())                   # never walk away on exit
        node.destroy_node()
        rclpy.try_shutdown()   # idempotent: bare shutdown() raises if already down


if __name__ == '__main__':
    main()
