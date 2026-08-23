#!/usr/bin/env python3
"""
RAISE 2026 — keyboard teleop for the Husky.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

  ┌─────────────┬──────────────┐
  │  w   or  ↑  │  forward     │
  │  a   or  ←  │  turn left   │
  │  d   or  →  │  turn right  │
  │  x   or  ↓  │  backward    │
  │  s          │  STOP        │
  │  q          │  curve forward + left   │
  │  e          │  curve forward + right  │
  │  SPACE      │  STOP                   │
  │  Ctrl-C     │  exit                   │
  └─────────────┴──────────────┘

Hold the key; the robot moves while you hold it.
Tap `s` (or SPACE) to stop instantly.
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LINEAR_SPEED = 0.5    # m/s
ANGULAR_SPEED = 1.0   # rad/s

# Each key → (linear_x_factor, angular_z_factor) in robot frame.
# Escape sequences (\x1b[…) are the arrow keys.
KEYS = {
    # forward
    'w':       ( 1.0,  0.0),
    '\x1b[A':  ( 1.0,  0.0),    # ↑
    # backward
    'x':       (-1.0,  0.0),
    '\x1b[B':  (-1.0,  0.0),    # ↓
    # turn left
    'a':       ( 0.0,  1.0),
    '\x1b[D':  ( 0.0,  1.0),    # ←
    # turn right
    'd':       ( 0.0, -1.0),
    '\x1b[C':  ( 0.0, -1.0),    # →
    # curves
    'q':       ( 0.5,  1.0),
    'e':       ( 0.5, -1.0),
    # stop
    's':       ( 0.0,  0.0),
    ' ':       ( 0.0,  0.0),
}


def read_key(timeout: float = 0.1) -> str:
    """Non-blocking read from stdin. Returns '' if no key, or the key /
    escape sequence (e.g. '\\x1b[A' for the up arrow)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return ''
        ch = sys.stdin.read(1)
        # ESC + '[' + X is an arrow key — read the next 2 chars if available
        if ch == '\x1b':
            r, _, _ = select.select([sys.stdin], [], [], 0.01)
            if r:
                ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = Node('teleop_keyboard')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    print(__doc__)
    print(f'  Linear: {LINEAR_SPEED} m/s   Angular: {ANGULAR_SPEED} rad/s\n')

    try:
        while rclpy.ok():
            key = read_key()
            if key == '\x03':                   # Ctrl-C
                break
            if key.lower() in KEYS or key in KEYS:
                lin, ang = KEYS.get(key, KEYS.get(key.lower()))
                msg = Twist()
                msg.linear.x  = lin * LINEAR_SPEED
                msg.angular.z = ang * ANGULAR_SPEED
                pub.publish(msg)
    finally:
        pub.publish(Twist())                    # ensure the robot is stopped
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
