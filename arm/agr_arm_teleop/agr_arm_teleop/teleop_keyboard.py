#!/usr/bin/env python3
"""
AGR Arm — keyboard teleop, in two different spaces.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_arm_teleop teleop_keyboard

  TOOL MODE (starts here) — you drive the GRIPPER, in metres
  ┌──────────────┬──────────────────────────────────────────┐
  │  w / x       │  tool forward / back        (world +x/-x)│
  │  a / d       │  tool left / right          (world +y/-y)│
  │  r / f       │  tool up / down             (world +z/-z)│
  │  q / e       │  spin the fingers about the vertical     │
  └──────────────┴──────────────────────────────────────────┘

  JOINT MODE — you drive ONE MOTOR, in radians
  ┌──────────────┬──────────────────────────────────────────┐
  │  1 . . 6     │  choose a joint (1 = shoulder pan)       │
  │  w / x       │  turn it + / -                           │
  └──────────────┴──────────────────────────────────────────┘

  ALWAYS
  ┌──────────────┬──────────────────────────────────────────┐
  │  TAB         │  switch between tool mode and joint mode │
  │  o / c       │  open / close the gripper                │
  │  h           │  go home                                 │
  │  + / -       │  bigger / smaller steps                  │
  │  Ctrl-C      │  exit                                    │
  └──────────────┴──────────────────────────────────────────┘

WHY TWO MODES — this is the lesson, not a feature
    Joint mode is what the robot actually is: six motors, six numbers, and
    every command certain to be executable. Press a key and that motor turns.
    Nothing can be out of reach, and nothing tells you where the hand will
    end up.

    Tool mode is what the task actually is: "5 cm to the left" means the same
    thing whatever configuration the arm is in. But every keypress has to be
    solved for, some of them have no answer, and the arm chooses its own path
    between the answers.

    Switch between them with TAB, and the trade is right there under your
    fingers. In joint mode, watch the tool position wander when you turn the
    shoulder. In tool mode, watch all six joints change to move the tool
    5 cm sideways — and watch it refuse when you push past the workspace.

Terminal handling note: raw mode is entered ONCE and held for the whole
session. Toggling it per keypress drops keys into the line discipline, which
shows up as a teleop that "misses" every second press.
"""

import math
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from agr_arm_tools import kinematics as K
from agr_arm_tools.arm import GRIPPER_CLOSED, GRIPPER_OPEN, Arm

STEP_M = 0.02          # metres per keypress in tool mode
STEP_RAD = 0.05        # radians per keypress in joint mode and for yaw
STEP_SCALE = (0.25, 0.5, 1.0, 2.0, 4.0)


class RawTerminal:
    """cbreak/raw stdin for the whole session, restored on the way out."""

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
        # Arrow keys arrive as a three-byte escape sequence. Read the tail
        # only if it is already waiting, so a bare ESC still returns at once.
        if ch == '\x1b' and select.select([sys.stdin], [], [], 0.01)[0]:
            ch += sys.stdin.read(2)
        return ch


def banner(mode: str, joint: int, scale: float) -> str:
    if mode == 'tool':
        return (f'TOOL  step {STEP_M * scale * 100:4.1f} cm | '
                f'w/x fwd  a/d left  r/f up  q/e spin | TAB=joint  o/c  h')
    return (f'JOINT {joint + 1} {K.JOINTS[joint][5:]:<20} '
            f'step {math.degrees(STEP_RAD * scale):4.1f} deg | '
            f'1-6 pick  w/x turn | TAB=tool  o/c  h')


def main():
    rclpy.init()
    node = Node('arm_teleop')
    arm = Arm(node)

    print('waiting for /joint_states ...')
    if not arm.wait_for_state():
        print('no /joint_states — is `agr-sim arm` running?')
        node.destroy_node()
        rclpy.try_shutdown()
        return

    mode = 'tool'
    joint = 0
    scale_i = 2
    # Target, not measurement. Jogging off the measured position accumulates
    # the controller's tracking error into every step, so a long run of
    # presses drifts; jogging off the last COMMAND does not.
    target = list(arm.joints)
    tool = list(K.fk(target, 'tcp')[:3, 3])
    yaw = 0.0
    note = ''

    print('\n  AGR arm teleop.  TAB switches mode, Ctrl-C exits.\n')
    try:
        with RawTerminal() as term:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                key = term.read_key(0.05)
                scale = STEP_SCALE[scale_i]

                if key in ('\x03', '\x04'):          # Ctrl-C / Ctrl-D
                    break
                # `key` is '' on every idle pass, and '' is a substring of
                # every string in Python — so `key in '123456'` is TRUE when
                # nothing was pressed. Left unguarded that reaches int('')
                # and the teleop dies on its first idle loop, before you can
                # press anything. Every membership test below is therefore
                # reached only once a key is known to be non-empty.
                if not key:
                    pass
                elif key == '\t':
                    mode = 'joint' if mode == 'tool' else 'tool'
                    target = list(arm.joints)
                    tool = list(K.fk(target, 'tcp')[:3, 3])
                    note = ''
                elif key in '123456':
                    joint = int(key) - 1
                    mode = 'joint'
                elif key in '+=':
                    scale_i = min(scale_i + 1, len(STEP_SCALE) - 1)
                elif key in '-_':
                    scale_i = max(scale_i - 1, 0)
                elif key == 'o':
                    arm.set_gripper(GRIPPER_OPEN, timeout=0.1)
                    note = 'opening'
                elif key == 'c':
                    arm.set_gripper(GRIPPER_CLOSED, timeout=0.1)
                    note = 'closing'
                elif key == 'h':
                    target = list(K.HOME)
                    tool = list(K.fk(target, 'tcp')[:3, 3])
                    yaw = 0.0
                    arm.move_joints(target, timeout=0.1)
                    note = 'home'
                elif mode == 'joint':
                    delta = STEP_RAD * scale
                    if key == 'w':
                        target[joint] += delta
                    elif key == 'x':
                        target[joint] -= delta
                    arm.move_joints(target, timeout=0.1)
                    tool = list(K.fk(target, 'tcp')[:3, 3])
                    note = ''
                elif mode == 'tool':
                    d = STEP_M * scale
                    moved = True
                    if key == 'w':
                        tool[0] += d
                    elif key == 'x':
                        tool[0] -= d
                    elif key == 'a':
                        tool[1] += d
                    elif key == 'd':
                        tool[1] -= d
                    elif key == 'r':
                        tool[2] += d
                    elif key == 'f':
                        tool[2] -= d
                    elif key == 'q':
                        yaw += STEP_RAD * scale
                    elif key == 'e':
                        yaw -= STEP_RAD * scale
                    else:
                        moved = False
                    if moved:
                        # Seed from the current TARGET so the solution stays
                        # in the same branch step after step. Seeding from
                        # HOME makes the arm occasionally flip its elbow
                        # half-way across the bench between two 2 cm steps.
                        q = K.ik_down(tool[0], tool[1], tool[2], yaw, seed=target)
                        if q is None:
                            # Put the ghost tool back where the real one is,
                            # or every further press compounds an unreachable
                            # request and the arm never moves again.
                            tool = list(K.fk(target, 'tcp')[:3, 3])
                            note = 'OUT OF REACH'
                        else:
                            target = list(q)
                            arm.move_joints(target, timeout=0.1)
                            note = ''

                p = arm.tcp
                held = arm.held_object or '-'
                sys.stdout.write(
                    f'\r\033[K{banner(mode, joint, scale)}\r\n'
                    f'\033[Ktcp ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m   '
                    f'yaw {math.degrees(yaw):+6.1f} deg   '
                    f'fingers {arm.tip_separation_mm:3.0f} mm   '
                    f'holding {held}   {note}\033[1A')
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        print('\n\nteleop finished; the arm holds its last position.')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
