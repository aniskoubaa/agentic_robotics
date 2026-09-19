#!/usr/bin/env python3
"""
TurtleBot example 05 — drive a square, and measure how badly it closes.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_tb_examples 05_drive_a_square
    ros2 run agr_tb_examples 05_drive_a_square --ros-args -p side:=0.8

WHAT  Four sides and four 90-degree turns, then compare where the robot
      thinks it is against where it started. It should be back at (0, 0)
      facing the way it began. It will not be.

THIS EXAMPLE IS SUPPOSED TO FAIL, and the size of the failure is the lesson.

LEARN - OPEN LOOP MEANS NOBODY IS CHECKING. Every side is "drive at v for t
        seconds", every corner is "turn until odometry says 90 degrees".
        Nothing ever compares the result against the world, so every small
        error is kept forever and added to the next one.
      - The turns are closed on ODOMETRY, which is already better than
        closing them on a timer — and still not good enough, because
        odometry is itself accumulating error.
      - This is the same experiment as `agr_uav_examples 04_fly_a_square`,
        which closes a 20 m square to about 0.10 m. The drone wins because
        PX4 runs a state estimator and a position controller: "go to (5, 0)"
        is a closed loop that keeps correcting. Nothing here corrects
        anything. That contrast is the entire argument for SLAM and Nav2.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from agr_tb_tools.base import Base


def main():
    rclpy.init()
    node = Node('drive_a_square',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    node.declare_parameter('side', 1.0)
    side = float(node.get_parameter('side').value)

    bot = Base(node)
    if not bot.wait_for_state():
        print('no odometry — is `agr-sim turtlebot` running?')
        node.destroy_node()
        rclpy.try_shutdown()
        return
    bot.ensure_ready()

    speed = min(0.15, bot.robot.max_speed * 0.6)
    seconds = side / speed
    start = bot.pose
    start_yaw = bot.yaw
    print(f'\n{bot.robot.key}: a {side:.2f} m square at {speed:.2f} m/s\n')
    print(f'  {"leg":<6}{"pose after":<24}{"yaw":>9}{"turn err":>10}')
    print('  ' + '-' * 49)

    for leg in range(4):
        bot.drive(speed, seconds=seconds)
        err = bot.turn(math.pi / 2)
        p = bot.pose
        print(f'  {leg + 1:<6}({p[0]:+.3f}, {p[1]:+.3f}){"":<8}'
              f'{math.degrees(bot.yaw):>+9.1f}{math.degrees(err):>+10.1f}')

    end = bot.pose
    gap = math.dist(start, end)
    heading = math.degrees(math.atan2(math.sin(bot.yaw - start_yaw),
                                      math.cos(bot.yaw - start_yaw)))
    perimeter = 4 * side

    print(f'\n  started at ({start[0]:+.3f}, {start[1]:+.3f}), '
          f'ended at ({end[0]:+.3f}, {end[1]:+.3f})')
    print(f'  the square did not close by {gap:.3f} m '
          f'({100 * gap / perimeter:.1f}% of the {perimeter:.1f} m driven)')
    print(f'  heading is {heading:+.1f} deg from where it started\n')
    print('  And note: that gap is measured with the SAME odometry that did the')
    print('  driving. The true error against the world is larger — odometry')
    print('  cannot see its own drift, which is exactly why it drifts.\n')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
