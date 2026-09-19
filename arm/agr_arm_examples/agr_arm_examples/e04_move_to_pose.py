#!/usr/bin/env python3
"""
Arm example 04 — go to a named pose.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_arm_examples 04_move_to_pose
    ros2 run agr_arm_examples 04_move_to_pose --ros-args -p pose:=watch
    ros2 run agr_arm_examples 04_move_to_pose --ros-args -p pose:=tour

WHAT  Drive the arm to one of the four stored configurations — home, ready,
      watch, stow — or `tour` to visit all four in turn.

LEARN - A named pose is six stored numbers and nothing more. No solver, no
        possibility of failure, no arguments to get wrong. That makes it the
        right tool for the parts of a task that never change: go to the
        start, get out of the way, park.
      - It is the WRONG tool the moment the target depends on where an object
        is, because you cannot store a pose for a block you have not seen.
        That is what example 05 is for, and the contrast between this file
        and that one is the point of having both.
      - The same four poses are exposed as services by move_to_pose_server,
        so the identical action is available to a program that speaks
        services rather than topics:
            ros2 service call /move_to_watch std_srvs/srv/Trigger
        Same six numbers, different door into the robot.
"""
import math
import time

import rclpy
from rclpy.node import Node

from agr_arm_tools import kinematics as K
from agr_arm_tools.arm import Arm
from agr_arm_tools.move_to_pose_server import POSES


def go(arm, name):
    target = POSES[name]
    predicted = K.tcp(target)
    print(f'\n{name}: [' + ' '.join(f'{math.degrees(v):+6.1f}' for v in target) + ' ] deg')
    print(f'  forward kinematics says the grasp point will be '
          f'({predicted[0]:+.3f}, {predicted[1]:+.3f}, {predicted[2]:+.3f}) m')
    t0 = time.time()
    ok = arm.move_joints(target, timeout=20.0)
    actual = arm.tcp
    print(f'  arrived in {time.time() - t0:.1f} s: '
          f'({actual[0]:+.3f}, {actual[1]:+.3f}, {actual[2]:+.3f}) m, '
          f'off by {math.dist(actual, predicted) * 1000:.1f} mm   {"" if ok else "(TIMED OUT)"}')
    return ok


def main():
    rclpy.init()
    node = Node('move_to_pose')
    node.declare_parameter('pose', 'ready')
    choice = node.get_parameter('pose').value

    arm = Arm(node)
    if not arm.wait_for_state():
        print('no /joint_states — is `agr-sim arm` running?')
    elif choice == 'tour':
        print('visiting every stored pose in turn.')
        for name in ('home', 'ready', 'watch', 'stow', 'home'):
            go(arm, name)
    elif choice not in POSES:
        print(f'unknown pose {choice!r}. Choose from: '
              f'{", ".join(POSES)}, or "tour".')
    else:
        go(arm, choice)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
