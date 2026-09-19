#!/usr/bin/env python3
"""
Arm example 05 — say where the hand should be, in metres.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_arm_examples 05_move_to_xyz
    ros2 run agr_arm_examples 05_move_to_xyz --ros-args -p x:=0.45 -p y:=0.18 -p z:=0.90
    ros2 run agr_arm_examples 05_move_to_xyz --ros-args -p x:=0.95     # try the impossible

WHAT  Ask for a point in space, solve inverse kinematics for it, drive there,
      and measure how close the hand actually got.

LEARN - This is the inverse of example 01, and it is a much harder problem.
        Forward kinematics has exactly one answer and is a few matrix
        multiplications. Inverse kinematics can have EIGHT answers for a UR
        arm, or none at all, and getting one takes a solver.
      - Which of those answers you get matters. kinematics.ik() is seeded
        from where the arm is now, so it returns the solution NEAREST to the
        current pose. Ask for two points 5 cm apart and you get two similar
        configurations; seed it badly and the arm flips its elbow through the
        bench to reach a point just next door.
      - "No solution" is a real answer and this prints it rather than
        pretending. Run it with x:=0.95 and read what comes back.
      - The number that surprises everyone: the arm has an 850 mm reach, but
        a STRAIGHT-DOWN grasp puts the wrist 0.22 m above the point you asked
        for, and it is the wrist that has to be within reach. So the top-down
        workspace is much smaller than the sphere on the datasheet. Run
        `ros2 run agr_arm_tools workspace` to see the map.
"""
import math
import time

import rclpy
from rclpy.node import Node

from agr_arm_tools import kinematics as K
from agr_arm_tools.arm import Arm


def main():
    rclpy.init()
    node = Node('move_to_xyz')
    node.declare_parameter('x', 0.45)
    node.declare_parameter('y', 0.0)
    node.declare_parameter('z', 0.95)
    node.declare_parameter('yaw', 0.0)
    x = float(node.get_parameter('x').value)
    y = float(node.get_parameter('y').value)
    z = float(node.get_parameter('z').value)
    yaw = float(node.get_parameter('yaw').value)

    arm = Arm(node)
    if not arm.wait_for_state():
        print('no /joint_states — is `agr-sim arm` running?')
        node.destroy_node()
        rclpy.try_shutdown()
        return

    here = arm.tcp
    print(f'\ngrasp point is at ({here[0]:+.3f}, {here[1]:+.3f}, {here[2]:+.3f}) m')
    print(f'asking for        ({x:+.3f}, {y:+.3f}, {z:+.3f}) m, '
          f'tool down, yaw {math.degrees(yaw):+.0f} deg')

    # Ask BEFORE moving. reachable() gives a reason, not just a no, and the
    # reason tells you whether to move the object or change the approach.
    ok, result = arm.can_reach(x, y, z, yaw)
    if not ok:
        print(f'\n  UNREACHABLE: {result}')
        print('\n  Nothing was sent to the robot. Try somewhere inside the map:')
        print('    ros2 run agr_arm_tools workspace')
        node.destroy_node()
        rclpy.try_shutdown()
        return

    print('\ninverse kinematics found a solution:')
    print('  ' + ' '.join(f'{math.degrees(v):+7.1f}' for v in result) + '  deg')
    print('  (compare with where it started — every joint had to change to '
          'move one point)')

    t0 = time.time()
    arrived = arm.move_to(x, y, z, yaw, timeout=20.0)
    landed = arm.tcp
    error = math.dist(landed, (x, y, z))
    print(f'\narrived in {time.time() - t0:.1f} s at '
          f'({landed[0]:+.3f}, {landed[1]:+.3f}, {landed[2]:+.3f}) m')
    print(f'  off the requested point by {error * 1000:.1f} mm '
          f'({"within" if arrived else "OUTSIDE"} tolerance)')
    print(f'  worst joint still {arm.joint_error():.4f} rad from its command')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
