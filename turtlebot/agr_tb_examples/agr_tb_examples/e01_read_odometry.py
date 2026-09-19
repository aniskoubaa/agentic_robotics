#!/usr/bin/env python3
"""
TurtleBot example 01 — where am I?
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_tb_examples 01_read_odometry

WHAT  Subscribe to /odom, print the pose, then drive a metre and print it
      again — and show that the two do not disagree yet.

LEARN - A mobile base's whole state is (x, y, yaw). Three numbers, versus the
        six an arm needs and the twelve a quadruped needs. That is why
        wheeled robots are where navigation is taught.
      - Odometry is DEAD RECKONING. It counts wheel rotations and integrates.
        Nothing in it ever looks at the world, so it cannot notice a wheel
        slipping, and its error only ever grows. Over a metre it is
        excellent. Over fifty it is fiction. Example 05 measures exactly how
        fast the fiction sets in.
      - The orientation in a ROS pose is a QUATERNION, not an angle. Yaw is
        atan2(2(wz+xy), 1-2(y^2+z^2)) — worth reading once, then never again,
        which is why Base.yaw exists.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from agr_tb_tools.base import Base


def main():
    rclpy.init()
    node = Node('read_odometry',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    bot = Base(node)

    print(f'\nrobot: {bot.robot.key} — {bot.robot.description}')
    print(f'reading {bot.robot.odom} ...')
    if not bot.wait_for_state():
        print(f'\nnothing on {bot.robot.odom} — is the simulator running?')
        print('   agr-sim turtlebot')
        node.destroy_node()
        rclpy.try_shutdown()
        return

    bot.ensure_ready()
    x, y = bot.pose
    print(f'\nstart:  x {x:+.3f}   y {y:+.3f}   yaw {math.degrees(bot.yaw):+7.2f} deg')

    print('\ndriving forward 1.0 m worth of command ...')
    result = bot.drive(0.15, seconds=1.0 / 0.15)
    x2, y2 = bot.pose
    print(f'end:    x {x2:+.3f}   y {y2:+.3f}   yaw {math.degrees(bot.yaw):+7.2f} deg')
    print(f'\nodometry says it travelled {result["moved"]:.3f} m; '
          f'the command asked for 1.000 m.')
    print('The difference is the controller\'s acceleration ramp, not drift —')
    print('over one metre, dead reckoning is still telling the truth.')
    print('Run 05_drive_a_square to watch it stop telling the truth.')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
