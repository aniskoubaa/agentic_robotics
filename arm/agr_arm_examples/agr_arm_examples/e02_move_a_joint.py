#!/usr/bin/env python3
"""
Arm example 02 — turn one motor.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_arm_examples 02_move_a_joint
    ros2 run agr_arm_examples 02_move_a_joint --ros-args -p joint:=elbow -p angle:=1.2

WHAT  Publish one number to one joint, watch it get there, and watch what
      that does to the position of the hand.

LEARN - Commanding this arm is genuinely this simple. One std_msgs/Float64 on
        /<joint_name>/cmd is a position command, and a PID controller inside
        Gazebo does the rest. You can drive the whole robot from the command
        line:
            ros2 topic pub --once /ur5e_elbow_joint/cmd std_msgs/msg/Float64 "{data: 1.2}"
      - The controller LATCHES. Publish once and the joint holds that angle
        against gravity forever; there is no need to keep sending it.
      - Watch the tool position in the output. Turning ONE joint moves the
        hand along a CIRCULAR ARC, not a straight line, and the further out
        the joint is, the smaller the effect. That mismatch — one motor,
        curved motion — is exactly why example 05 exists.
"""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from agr_arm_tools import kinematics as K

SHORT = {name.replace('ur5e_', '').replace('_joint', ''): name for name in K.JOINTS}


def main():
    rclpy.init()
    node = Node('move_a_joint')
    node.declare_parameter('joint', 'shoulder_pan')
    node.declare_parameter('angle', 0.5)
    choice = node.get_parameter('joint').value
    angle = float(node.get_parameter('angle').value)

    if choice not in SHORT:
        print(f'unknown joint {choice!r}. Choose one of:\n  ' + '\n  '.join(SHORT))
        node.destroy_node()
        rclpy.try_shutdown()
        return

    joint = SHORT[choice]
    latest = {}
    node.create_subscription(
        JointState, '/joint_states',
        lambda m: latest.update(zip(m.name, m.position)), 10)
    pub = node.create_publisher(Float64, f'/{joint}/cmd', 10)

    for _ in range(250):
        rclpy.spin_once(node, timeout_sec=0.02)
        if all(j in latest for j in K.JOINTS):
            break
    if not all(j in latest for j in K.JOINTS):
        print('no /joint_states — is `agr-sim arm` running?')
        node.destroy_node()
        rclpy.try_shutdown()
        return

    before = [latest[j] for j in K.JOINTS]
    start_tcp = K.tcp(before)
    print(f'\n{joint} is at {latest[joint]:+.3f} rad; sending {angle:+.3f} rad')
    print(f'grasp point now at ({start_tcp[0]:+.3f}, {start_tcp[1]:+.3f}, '
          f'{start_tcp[2]:+.3f}) m\n')

    # WAIT FOR A SUBSCRIBER BEFORE PUBLISHING. This is not defensive
    # padding, it is the difference between this script working and not.
    # create_publisher() returns instantly, but DDS discovery — finding the
    # bridge on the other side and agreeing to talk — takes a moment, and a
    # message published before that lands nowhere at all. No error, no
    # warning, the joint simply never moves. The first version of this file
    # published on the line after create_publisher and reported "asked for
    # +0.400, settled at -0.000" against a perfectly healthy robot.
    #
    # Arm.move_joints() sidesteps this by republishing at 20 Hz, so the first
    # few lost messages do not matter. Here we publish exactly once, on
    # purpose, to show that the controller latches — so the one message has
    # to arrive.
    waited = 0.0
    while waited < 10.0 and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.05)
        waited += 0.05
    if pub.get_subscription_count() == 0:
        print(f'nothing is subscribed to /{joint}/cmd after {waited:.0f} s — '
              f'is the ros_gz bridge up?  (ros2 topic info /{joint}/cmd)')
    elif waited > 0.5:
        print(f'(took {waited:.1f} s for the bridge to notice this publisher)')

    pub.publish(Float64(data=angle))

    # One publish is enough — the controller latches. We spin only to watch.
    end = time.time() + 8.0
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)

    after = [latest[j] for j in K.JOINTS]
    end_tcp = K.tcp(after)
    moved = math.dist(start_tcp, end_tcp)
    settled_error = latest[joint] - angle
    print(f'{joint} settled at {latest[joint]:+.3f} rad '
          f'(asked for {angle:+.3f}, off by {settled_error:+.4f})')
    if abs(settled_error) > 0.05:
        print('  -- that is a long way off. Either the command never arrived, or\n'
              '     the joint is pressing against something. Run:\n'
              '       ros2 run agr_arm_demos diagnose')
    print(f'grasp point now at ({end_tcp[0]:+.3f}, {end_tcp[1]:+.3f}, '
          f'{end_tcp[2]:+.3f}) m')
    print(f'\nthe hand moved {moved * 100:.1f} cm — along an arc, not a line.')
    print('Try the same angle on `wrist_3` and then on `shoulder_pan`:')
    print('the same number of radians, wildly different distances travelled.')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
