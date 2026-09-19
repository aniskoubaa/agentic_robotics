#!/usr/bin/env python3
"""
Arm example 06 — pick a block up and put it in the tray.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_arm_examples 06_pick_and_place
    ros2 run agr_arm_examples 06_pick_and_place --ros-args -p block:=block_blue

WHAT  The whole task, in six moves, with every step printed as it happens.

THE SIX MOVES, AND WHY EACH ONE IS THERE
    1. open      — before going anywhere near the block. Opening ON TOP of it
                   knocks it away.
    2. hover     — directly above the block, 12 cm up.
    3. descend   — straight down onto it. Two moves, not one, and that is the
                   single most important line in this file. move_to() takes
                   the arm along a straight line in JOINT space, which bulges
                   in real space; a single diagonal move from the start pose
                   to the block sweeps the gripper through the bench on the
                   way. Approach from directly above and the last move is
                   vertical, which is a path you can reason about.
    4. close     — grasp_server sees the command and welds the block on.
    5. lift      — straight back up, same reason as 3 in reverse.
    6. carry, then release over the tray.

LEARN - Manipulation is SEQUENCING. Not one of these six moves is difficult;
        the difficulty is entirely in doing them in this order, and every
        wrong order fails in its own way. Try it: comment out the hover and
        watch what happens.
      - The descent height is not the block's centre. The fingertips reach
        25 mm PAST the tool centre point, so aiming the TCP at the middle of
        a 50 mm block on the bench puts the fingertips exactly on the table,
        the arm stalls against it, and the move times out short. Ask
        kinematics.lowest_tcp_over() instead of guessing.
      - "Am I holding it?" is a question about the world, not about the arm.
        This asks grasp_server via /grasp/state rather than inferring it.
"""
import math
import time

import rclpy
from rclpy.node import Node

from agr_arm_tools import gz_utils, kinematics as K
from agr_arm_tools.arm import Arm

TRAY = (0.66, 0.30)
HOVER = 0.12         # m above the block to line up from
LIFT = 0.95          # m to carry at — above the tray walls, below the ceiling


# move_to() has three outcomes and they mean different things — see Arm.
# The gripper calls have none of them (see Arm.set_gripper), so they get their
# own row of vocabulary rather than being reported as a failed IK solve.
MOVE_FLAGS = {True: 'ok', False: 'TIMED OUT', None: 'NO IK SOLUTION'}


def step(n, label, action, arm, kind='move', settle=0.0):
    t0 = time.time()
    result = action()
    if settle:
        arm.sleep(settle)          # inside the step, so the line below is true
    p = arm.tcp
    flag = MOVE_FLAGS.get(result, 'done') if kind == 'move' else 'done'
    print(f'  {n}. {label:<26} {flag:<15} {time.time() - t0:5.1f} s   '
          f'tcp ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})'
          + (f'   holding {arm.held_object}' if arm.held_object else ''))
    return kind != 'move' or (result is not False and result is not None)


def main():
    rclpy.init()
    node = Node('pick_and_place')
    node.declare_parameter('block', 'block_green')
    name = node.get_parameter('block').value

    arm = Arm(node)
    if not arm.wait_for_state():
        print('no /joint_states — is `agr-sim arm` running?')
        node.destroy_node()
        rclpy.try_shutdown()
        return

    pose = gz_utils.get_model_world_pose(name)
    if pose is None:
        print(f'"{name}" is not in the world. Try block_red, block_green or block_blue.')
        node.destroy_node()
        rclpy.try_shutdown()
        return
    bx, by, bz = pose[0]
    print(f'\n{name} is at ({bx:+.3f}, {by:+.3f}, {bz:.3f}) m')

    ok, why = arm.can_reach(bx, by, K.lowest_tcp_over(K.BENCH_Z))
    if not ok:
        print(f'  cannot reach it: {why}')
        node.destroy_node()
        rclpy.try_shutdown()
        return

    grasp_z = K.lowest_tcp_over(K.BENCH_Z)
    print(f'  grasping at z = {grasp_z:.3f} — that is {grasp_z - bz:+.3f} m from the\n'
          f'  block centre, so the fingertips clear the bench by 5 mm.\n')

    step(1, 'open the gripper', lambda: arm.open_gripper(), arm, kind='gripper')
    step(2, 'hover above it', lambda: arm.move_to(bx, by, bz + HOVER), arm)
    step(3, 'descend onto it', lambda: arm.move_to(bx, by, grasp_z), arm)
    step(4, 'close the gripper', lambda: arm.close_gripper(), arm,
         kind='gripper', settle=0.5)

    if not arm.holding_something:
        print(f'\n  nothing was picked up.'
              + ('' if arm.heard_from_grasp_server else
                 '  (grasp_server is not running — start it with '
                 '`agr-sim arm tools:=true`)'))
    step(5, 'lift straight up', lambda: arm.move_to(bx, by, LIFT), arm)
    step(6, 'carry to the tray', lambda: arm.move_to(TRAY[0], TRAY[1], 0.86), arm)
    step(7, 'release', lambda: arm.open_gripper(), arm,
         kind='gripper', settle=1.5)
    step(8, 'back off and go home', lambda: arm.home(), arm)  # move again
    arm.sleep(1.0)

    end = gz_utils.get_model_world_pose(name)
    if end is not None:
        p = end[0]
        # Generous height band on purpose: a block resting on another block
        # already in the tray is still in the tray.
        in_tray = (abs(p[0] - TRAY[0]) < 0.11 and abs(p[1] - TRAY[1]) < 0.11
                   and 0.74 < p[2] < 0.95)
        print(f'\n{name} ended at ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:.3f}) — '
              f'moved {math.dist(p, (bx, by, bz)) * 100:.0f} cm.')
        print(f'IN THE TRAY: {in_tray}')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
