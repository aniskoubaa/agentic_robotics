#!/usr/bin/env python3
"""
TurtleBot — keyboard teleop, for whichever TurtleBot is running.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_tb_teleop teleop_keyboard

  ┌──────────────┬──────────────────────────────────────────┐
  │  w  or  UP   │  forward                                 │
  │  x  or  DOWN │  backward                                │
  │  a  or  LEFT │  turn left                               │
  │  d  or  RIGHT│  turn right                              │
  │  s / SPACE   │  STOP                                    │
  │  + / -       │  faster / slower                         │
  │  u / p       │  undock / dock      (TurtleBot 4 only)   │
  │  Ctrl-C      │  exit                                    │
  └──────────────┴──────────────────────────────────────────┘

Hold a key; the robot moves while you hold it. Release and it stops on its
own, because velocity commands do not latch — there is a watchdog in the
base, and this node stops publishing the moment you let go.

WHY THIS EXISTS WHEN turtlebot3_teleop ALREADY DOES
    Three reasons, all of them things measured on the running robots:

    - It publishes geometry_msgs/TwistStamped, which is what /cmd_vel
      actually is under Jazzy on both robots. A node still publishing Twist
      moves nothing and reports no error.
    - It knows about docking. Press `u` and it undocks a TurtleBot 4, which
      is otherwise the reason a brand new TB4 refuses to move at all.
    - It shows the lidar distance ahead while you drive, so the sensor and
      the motion are on screen together.

Terminal handling: raw mode is entered ONCE and held for the session.
Toggling it per keypress drops keys into the line discipline, which reads as
a teleop that misses every second press.
"""
import math
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from agr_tb_tools.base import Base

STEPS = (0.25, 0.5, 0.75, 1.0)          # fractions of the robot's max speed

KEYS = {
    'w': (1.0, 0.0), '\x1b[A': (1.0, 0.0),
    'x': (-1.0, 0.0), '\x1b[B': (-1.0, 0.0),
    'a': (0.0, 1.0), '\x1b[D': (0.0, 1.0),
    'd': (0.0, -1.0), '\x1b[C': (0.0, -1.0),
    's': (0.0, 0.0), ' ': (0.0, 0.0),
}


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self, timeout: float = 0.05) -> str:
        if not select.select([sys.stdin], [], [], timeout)[0]:
            return ''
        ch = sys.stdin.read(1)
        if ch == '\x1b' and select.select([sys.stdin], [], [], 0.01)[0]:
            ch += sys.stdin.read(2)
        return ch


def main():
    rclpy.init()
    node = Node('tb_teleop', parameter_overrides=[Parameter('use_sim_time', value=True)])
    bot = Base(node)

    print(f'\nwaiting for {bot.robot.odom} ...')
    if not bot.wait_for_state():
        print('no odometry — is `agr-sim turtlebot` running?')
        node.destroy_node()
        rclpy.try_shutdown()
        return

    step = 2
    cmd = (0.0, 0.0)
    note = ''
    if bot.robot.is_tb4:
        bot.sleep(1.0)
        if bot.is_docked:
            note = 'DOCKED — press u to undock, or it will not move'

    print(f'\n  {bot.robot.key}: w/x drive, a/d turn, s stop, +/- speed, '
          f'{"u/p undock/dock, " if bot.robot.is_tb4 else ""}Ctrl-C to exit\n\n')
    try:
        with RawTerminal() as term:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                key = term.read_key(0.05)

                if key in ('\x03', '\x04'):
                    break
                # '' is a substring of every string, so an unguarded
                # `key in '+='` is True on every idle pass. Check first.
                if not key:
                    pass
                elif key in KEYS:
                    cmd = KEYS[key]
                    note = ''
                elif key in ('+', '='):
                    step = min(step + 1, len(STEPS) - 1)
                elif key in ('-', '_'):
                    step = max(step - 1, 0)
                elif key == 'u' and bot.robot.is_tb4:
                    note = 'undocking (about 30 s) ...'
                    _redraw(bot, cmd, step, note)
                    bot.undock()
                    note = f'undocked: {bot.is_docked is False}'
                    cmd = (0.0, 0.0)
                elif key == 'p' and bot.robot.is_tb4:
                    note = 'docking ...'
                    _redraw(bot, cmd, step, note)
                    bot.dock()
                    note = f'docked: {bot.is_docked}'
                    cmd = (0.0, 0.0)

                scale = STEPS[step]
                bot.send(cmd[0] * bot.robot.max_speed * scale,
                         cmd[1] * bot.robot.max_turn * scale)
                _redraw(bot, cmd, step, note)
    except KeyboardInterrupt:
        pass
    finally:
        bot.stop()
        print('\n\nteleop finished; the robot is stopped.')
        node.destroy_node()
        rclpy.try_shutdown()


def _redraw(bot, cmd, step, note):
    ahead = bot.range_ahead()
    shown = f'{ahead:.2f} m' if math.isfinite(ahead) else 'clear'
    p = bot.pose or (float('nan'), float('nan'))
    sys.stdout.write(
        f'\r\033[K  speed {STEPS[step] * 100:3.0f}%   '
        f'v {cmd[0] * bot.robot.max_speed * STEPS[step]:+.2f} m/s   '
        f'w {cmd[1] * bot.robot.max_turn * STEPS[step]:+.2f} rad/s\r\n'
        f'\033[K  at ({p[0]:+.2f}, {p[1]:+.2f})  yaw {math.degrees(bot.yaw):+6.1f}deg   '
        f'ahead {shown:<9}'
        + (f'docked={bot.is_docked}  ' if bot.robot.is_tb4 else '')
        + f'{note}\033[1A')
    sys.stdout.flush()


if __name__ == '__main__':
    main()
