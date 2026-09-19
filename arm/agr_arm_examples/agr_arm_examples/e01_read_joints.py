#!/usr/bin/env python3
"""
Arm example 01 — read the six joints, and find the hand.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_arm_examples 01_read_joints

WHAT  Subscribe to /joint_states, print every joint angle once, then work out
      where the gripper is.

LEARN - An arm IS six numbers. Not "has" six numbers — is. Every pose it can
        hold, every task it can do, every way it can fail is some choice of
        those six, and there is nothing else to know about its configuration.
      - Those six numbers do NOT tell you where the hand is. Getting from one
        to the other is forward kinematics, and it is a chain of six
        transforms, which is what kinematics.fk() does below. Note how the
        answer is one specific point — FK always has exactly one answer.
        Going the other way, in example 05, will not be so tidy.
      - JointState carries `name` and `position` as two PARALLEL ARRAYS, and
        the order is whatever the publisher felt like. Never index by
        position; zip them into a dict and look joints up by name. Assuming
        the order is the single most common bug in this file's job.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from agr_arm_tools import kinematics as K


def main():
    rclpy.init()
    node = Node('read_joints')
    latest = {}
    node.create_subscription(
        JointState, '/joint_states',
        lambda m: latest.update(zip(m.name, m.position)), 10)

    # Spin until a sample arrives. DDS discovery is not instant, so reading
    # `latest` on the next line would show an empty dict on a healthy robot.
    print('waiting for /joint_states ...')
    for _ in range(250):                       # ~5 s at 20 ms per spin
        rclpy.spin_once(node, timeout_sec=0.02)
        if all(j in latest for j in K.JOINTS):
            break

    if not all(j in latest for j in K.JOINTS):
        print('no /joint_states — is `agr-sim arm` running?')
    else:
        angles = [latest[j] for j in K.JOINTS]
        print(f'\n{"joint":<26}{"radians":>10}{"degrees":>10}\n' + '-' * 46)
        for name, value in zip(K.JOINTS, angles):
            short = name.replace('ur5e_', '').replace('_joint', '')
            print(f'{short:<26}{value:>10.4f}{math.degrees(value):>10.1f}')

        tool = K.fk(angles)
        grasp = K.tcp(angles)
        print(f'\nforward kinematics says:')
        print(f'  flange  (ur5e_tool0) at  ({tool[0, 3]:+.3f}, {tool[1, 3]:+.3f}, '
              f'{tool[2, 3]:+.3f}) m')
        print(f'  grasp point         at  ({grasp[0]:+.3f}, {grasp[1]:+.3f}, '
              f'{grasp[2]:+.3f}) m')
        print(f'  that is {grasp[2] - K.BENCH_Z:+.3f} m above the bench top.')
        print('\nCheck it yourself — this should print the same numbers:')
        print('  ros2 run tf2_ros tf2_echo world ur5e_tool0')

        fingers = [j for j in latest if 'knuckle_joint' in j]
        if fingers:
            knuckle = latest.get('gripper_robotiq_85_left_knuckle_joint', float('nan'))
            print(f'\nthe gripper is a seventh number: knuckle {knuckle:+.3f} rad')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
