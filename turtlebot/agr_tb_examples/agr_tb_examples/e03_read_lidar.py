#!/usr/bin/env python3
"""
TurtleBot example 03 — the sensor that makes navigation possible.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_tb_examples 03_read_lidar

WHAT  Take one LaserScan and turn it into something you can read: how many
      beams, over what arc, and what is closest in each direction.

LEARN - A LaserScan is ONE ARRAY plus the geometry to interpret it. `ranges`
        holds the distances; the angle of ranges[i] is
        angle_min + i * angle_increment. There are no angles stored anywhere.
        Every bearing you ever compute from a lidar comes out of that line.
      - INFINITY AND NAN ARE DATA, NOT ERRORS. A beam that hits nothing
        within range reports inf; a beam that fails reports nan. Feed those
        into a min() without filtering and you get nonsense, or a crash.
        Filtering them is the first thing any lidar code does.
      - The arc matters as much as the count. Both TurtleBots scan a full
        360 degrees, which is why they can do SLAM while driving forwards —
        a forward-facing lidar has to be turned to see where it has been.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from agr_tb_tools.base import Base

# Sectors to summarise, as (label, centre bearing in degrees).
SECTORS = [('ahead', 0), ('left', 90), ('behind', 180), ('right', -90)]
HALF_WIDTH = math.radians(15)


def sector_min(scan, centre_deg):
    centre = math.radians(centre_deg)
    best = float('inf')
    for i, r in enumerate(scan.ranges):
        # Drop the beams that carry no measurement BEFORE comparing anything.
        if not math.isfinite(r) or r < scan.range_min:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        delta = math.atan2(math.sin(angle - centre), math.cos(angle - centre))
        if abs(delta) <= HALF_WIDTH:
            best = min(best, r)
    return best


def main():
    rclpy.init()
    node = Node('read_lidar',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    bot = Base(node)

    print(f'\nwaiting for a scan on {bot.robot.scan} ...')
    bot.wait_for_state()
    for _ in range(400):                      # ~8 s; the lidar runs at 5-18 Hz
        rclpy.spin_once(node, timeout_sec=0.02)
        if bot.scan is not None:
            break

    scan = bot.scan
    if scan is None:
        print(f'nothing on {bot.robot.scan} — is `agr-sim turtlebot` running?')
    else:
        span = math.degrees(scan.angle_max - scan.angle_min)
        step = math.degrees(scan.angle_increment)
        valid = [r for r in scan.ranges if math.isfinite(r) and r >= scan.range_min]
        print(f'\n  {len(scan.ranges)} beams over {span:.0f} deg '
              f'({step:.2f} deg apart)')
        print(f'  range {scan.range_min:.2f} .. {scan.range_max:.2f} m')
        print(f'  {len(valid)} beams returned a measurement; '
              f'{len(scan.ranges) - len(valid)} were inf or nan')
        if valid:
            print(f'  closest thing anywhere: {min(valid):.2f} m\n')
        print(f'  {"sector":<10}{"nearest":>10}')
        print('  ' + '-' * 20)
        for label, deg in SECTORS:
            d = sector_min(scan, deg)
            print(f'  {label:<10}{(f"{d:.2f} m" if math.isfinite(d) else "clear"):>10}')
        print('\n  Drive with 02_drive in another terminal and watch these change.')
        print('  06_avoid_obstacle closes the loop: lidar in, velocity out.\n')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
