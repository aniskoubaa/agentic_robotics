#!/usr/bin/env python3
"""
TurtleBot example 06 — close the loop: lidar in, velocity out.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_tb_examples 06_avoid_obstacle
    ros2 run agr_tb_examples 06_avoid_obstacle --ros-args -p seconds:=60.0

WHAT  Drive forward. When the lidar says something is close, turn towards
      whichever side has more space. Repeat. Ctrl-C to stop.

LEARN - This is the first script here that REACTS. Examples 01-05 issue
        commands and hope; this one reads a sensor and decides, forty times a
        second. That loop — sense, decide, act — is the whole of reactive
        robotics, and it fits in the twenty lines below.
      - It has NO MAP AND NO MEMORY. It cannot tell a corridor from a dead
        end, and it will happily oscillate between two walls or drive in
        circles forever. That is not a bug to fix here; it is the reason Nav2
        and SLAM exist, and it is much more convincing once you have watched
        this get stuck.
      - Compare with 05: the square accumulates error because nothing checks.
        This never accumulates anything, because it only ever reacts to NOW —
        and pays for it by never knowing where it is.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from agr_tb_tools.base import Base

STOP_AT = 0.55           # m; start turning when something is this close
CLEAR_AT = 0.85          # m; go straight again once it is this far
SIDE_LOOK = math.radians(50)


def side_space(scan, sign):
    """Mean free space on one side: sign +1 is left, -1 is right."""
    if scan is None:
        return 0.0
    total, count = 0.0, 0
    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if 0.2 < sign * angle < SIDE_LOOK + 0.2:
            total += r if math.isfinite(r) else scan.range_max
            count += 1
    return total / count if count else 0.0


def main():
    rclpy.init()
    node = Node('avoid_obstacle',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    node.declare_parameter('seconds', 45.0)
    duration = float(node.get_parameter('seconds').value)

    bot = Base(node)
    if not bot.wait_for_state():
        print('no odometry — is `agr-sim turtlebot` running?')
        node.destroy_node()
        rclpy.try_shutdown()
        return
    bot.ensure_ready()

    speed = min(0.18, bot.robot.max_speed * 0.7)
    turn = min(0.7, bot.robot.max_turn * 0.4)
    print(f'\n{bot.robot.key}: wandering for {duration:.0f} s. Ctrl-C to stop.')
    print(f'  forward {speed:.2f} m/s, turning when something is under '
          f'{STOP_AT:.2f} m\n')

    turning = 0.0        # which way we are currently turning, 0 = straight
    turns = 0
    end = time.time() + duration
    last_report = 0.0
    try:
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            ahead = bot.range_ahead()

            if turning == 0.0 and math.isfinite(ahead) and ahead < STOP_AT:
                # Commit to a direction ONCE, and hold it until clear. Deciding
                # afresh every tick makes the robot dither on the spot when the
                # two sides are nearly equal.
                turning = 1.0 if side_space(bot.scan, +1) >= side_space(bot.scan, -1) else -1.0
                turns += 1
            elif turning != 0.0 and (not math.isfinite(ahead) or ahead > CLEAR_AT):
                turning = 0.0

            bot.send(0.0 if turning else speed, turning * turn)

            if time.time() - last_report > 2.0:
                last_report = time.time()
                where = 'turning ' + ('left' if turning > 0 else 'right') \
                    if turning else 'forward'
                shown = f'{ahead:.2f} m' if math.isfinite(ahead) else 'clear'
                print(f'  {where:<15} ahead {shown:<9} {bot.describe()}')
    except KeyboardInterrupt:
        pass
    finally:
        bot.stop()
        print(f'\n  stopped after {turns} avoidance turns.')
        print('  It never got lost, because it never tried to know where it was.\n')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
