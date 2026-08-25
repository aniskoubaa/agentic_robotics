#!/usr/bin/env python3
"""
UAV — keyboard flight for PX4 in offboard mode.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

  ┌──────────┬──────────────────────────────┐
  │  t       │  TAKE OFF (arm + climb)      │
  │  l       │  LAND (and disarm)           │
  │  w / x   │  forward / backward          │
  │  q / e   │  left / right  (strafe)      │
  │  r / f   │  up / down                   │
  │  a / d   │  yaw left / yaw right        │
  │  SPACE   │  HOVER — cancel all motion   │
  │  Ctrl-C  │  land, then exit             │
  └──────────┴──────────────────────────────┘

Motion is in the BODY frame: `w` goes where the nose points, not north.

Unlike a ground robot, you cannot simply "stop publishing" — PX4 leaves
offboard mode and goes to failsafe if setpoints stop arriving for about half
a second. So this loop publishes on EVERY iteration whether or not you touched
a key, and releasing a key means "velocity zero", not "no command". That is
the single biggest difference between flying something and driving it.
"""
import math
import select
import sys
import termios
import tty
import time

import rclpy
from rclpy.node import Node

from agr_uav_tools.offboard import Pilot

SPEED = 2.0          # m/s   horizontal
CLIMB = 1.0          # m/s   vertical
YAW_RATE = 0.8       # rad/s
LOOP_HZ = 20.0
TAKEOFF_ALT = 3.0

# key → (forward, left, up, yaw) in multiples of the constants above.
#
# The two sign conventions here are both counter-intuitive and both were
# MEASURED rather than reasoned about, because reasoning about them gets it
# wrong:
#
#   YAW. NED measures yaw from North toward East — CLOCKWISE seen from above.
#   So a POSITIVE yawspeed turns the aircraft RIGHT. Verified: +0.5 rad/s for
#   4 s moved the heading -0.7 deg -> +108.4 deg.
#
#   STRAFE. With the nose north, "east" is to your RIGHT. So a positive body-y
#   is right, not left.
#
# Getting either backwards gives a drone that flies the mirror image of what
# you pressed, which in a real flight test is how you hit something.
KEYS = {
    'w': (1, 0, 0, 0),  'x': (-1, 0, 0, 0),
    'q': (0, 1, 0, 0),  'e': (0, -1, 0, 0),
    'a': (0, 0, 0, -1), 'd': (0, 0, 0, 1),
    'r': (0, 0, 1, 0),  'f': (0, 0, -1, 0),
    ' ': (0, 0, 0, 0),
    '\x1b[A': (1, 0, 0, 0),  '\x1b[B': (-1, 0, 0, 0),
    '\x1b[D': (0, 0, 0, -1), '\x1b[C': (0, 0, 0, 1),
}


class RawTerminal:
    """Put the terminal in raw mode for the WHOLE session, not per keypress.

    Toggling raw mode around each read looks tidier and is subtly broken: in
    between reads the terminal is back in canonical mode, where the line
    discipline buffers input until a newline. A key pressed during that
    window is swallowed and never delivered, so the robot ignores you at
    random. MEASURED: driving this node through a pty, every key after the
    first was lost.
    """

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self, timeout: float) -> str:
        if not select.select([sys.stdin], [], [], timeout)[0]:
            return ''
        ch = sys.stdin.read(1)
        if ch == '\x1b' and select.select([sys.stdin], [], [], 0.01)[0]:
            ch += sys.stdin.read(2)
        return ch


def fly_loop(node, pilot, term) -> bool:
    """The key loop. Returns True if the vehicle is still airborne."""
    cmd = (0, 0, 0, 0)
    flying = False
    while rclpy.ok():
        key = term.read_key(1.0 / LOOP_HZ)
        if key == '\x03':                       # Ctrl-C
            break
        if key == 't' and not flying:
            print('\r taking off ...                    ', end='', flush=True)
            flying = pilot.takeoff(TAKEOFF_ALT)
            print('\r airborne                          ' if flying
                  else '\r takeoff FAILED — see diagnose     ', end='',
                  flush=True)
            cmd = (0, 0, 0, 0)
            continue
        if key == 'l' and flying:
            print('\r landing ...                       ', end='', flush=True)
            pilot.land()
            deadline = time.time() + 20.0
            while rclpy.ok() and pilot.armed and time.time() < deadline:
                pilot.spin(0.1)
            flying = False
            print('\r landed                            ', end='', flush=True)
            continue
        if key in KEYS:
            cmd = KEYS[key]

        # Spin EVERY iteration, airborne or not. read_key() blocks on stdin,
        # not on ROS, so without this the subscriptions are never serviced:
        # position and yaw freeze at whatever they were before takeoff while
        # the vehicle genuinely flies away. Publishing does not need a spin,
        # which is what makes the symptom so confusing — the drone obeys and
        # the display insists nothing happened.
        rclpy.spin_once(node, timeout_sec=0.0)

        if flying:
            fwd, left, up, yaw_in = cmd
            # Body frame → NED. The nose direction is the current yaw, so
            # "forward" has to be rotated into north/east every iteration.
            yaw = pilot.yaw()
            # Body → NED. Forward is (cos yaw, sin yaw); LEFT is
            # (sin yaw, -cos yaw), because right is +90 deg clockwise.
            vn = (fwd * math.cos(yaw) + left * math.sin(yaw)) * SPEED
            ve = (fwd * math.sin(yaw) - left * math.cos(yaw)) * SPEED
            pilot.send_setpoint(0.0, 0.0, 0.0,
                                velocity=(vn, ve, up * CLIMB),
                                yawspeed=yaw_in * YAW_RATE)
            pos = pilot.position()
            if pos:
                print(f'\r n={pos[0]:+6.1f} e={pos[1]:+6.1f} '
                      f'alt={pos[2]:5.1f}m  yaw={math.degrees(yaw):+6.1f}°  '
                      f'{"ARMED" if pilot.armed else "     "}   ',
                      end='', flush=True)

    return flying


def main():
    rclpy.init()
    node = Node('uav_teleop_keyboard')
    node.declare_parameter('namespace', '')
    pilot = Pilot(node, node.get_parameter('namespace').value)

    print(__doc__)
    if not pilot.wait_for_telemetry():
        print('no telemetry from PX4 — is the sim running?  agr-sim')
        node.destroy_node(); rclpy.try_shutdown(); return
    print(f'  {SPEED} m/s horizontal   {CLIMB} m/s climb   '
          f'{YAW_RATE} rad/s yaw\n  press t to take off\n')

    flying = False
    try:
        with RawTerminal() as term:
            flying = fly_loop(node, pilot, term)
    finally:
        # Never exit leaving the vehicle airborne — a script that stops
        # publishing setpoints drops PX4 into failsafe, and "what the drone
        # does next" stops being your decision.
        if flying:
            print('\nlanding before exit ...')
            pilot.land()
            deadline = time.time() + 20.0
            while rclpy.ok() and pilot.armed and time.time() < deadline:
                pilot.spin(0.1)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
