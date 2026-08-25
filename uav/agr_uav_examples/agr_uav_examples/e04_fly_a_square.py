#!/usr/bin/env python3
"""
UAV example 04 — fly a square.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_uav_examples 04_fly_a_square
    ros2 run agr_uav_examples 04_fly_a_square --ros-args -p side:=8.0

WHAT  Take off, fly four corners, come home, land. Reports the closing error.

LEARN - Compare this with the LEGGED square (06_walk_a_square). Same shape,
        same idea, wildly different accuracy — this one closes to a few tens
        of centimetres, the quadruped is tens of degrees out. The difference
        is not that flying is easier. It is that PX4 runs a state estimator
        and a position controller, so "go to (5, 0)" is a CLOSED LOOP: it
        measures where it is and keeps correcting. The legged gait is open
        loop and nothing ever checks.
      - So the lesson is not "drones are better". It is that the loop is what
        buys accuracy, and you can have it on any platform that can measure
        itself.
      - Setpoints are absolute positions in the local NED frame, with the
        origin where the vehicle booted. They are not "move by".
"""
import math

import rclpy
from rclpy.node import Node

from agr_uav_tools.offboard import Pilot


def main():
    rclpy.init()
    node = Node('fly_a_square')
    node.declare_parameter('side', 5.0)
    node.declare_parameter('altitude', 3.0)
    node.declare_parameter('namespace', '')
    side = float(node.get_parameter('side').value)
    alt = float(node.get_parameter('altitude').value)
    pilot = Pilot(node, node.get_parameter('namespace').value)

    print(f'taking off to {alt:.1f} m ...')
    if not pilot.takeoff(alt):
        print('takeoff failed — see `ros2 run agr_uav_demos diagnose`')
        pilot.disarm(); node.destroy_node(); rclpy.try_shutdown(); return

    home = pilot.position()
    hx, hy = home[0], home[1]
    corners = [(hx + side, hy), (hx + side, hy + side), (hx, hy + side), (hx, hy)]

    print(f'\nflying a {side:.0f} m square at {alt:.1f} m\n' + '-' * 46)
    for i, (x, y) in enumerate(corners, 1):
        ok = pilot.goto(x, y, alt)
        pos = pilot.position()
        err = math.dist((pos[0], pos[1]), (x, y))
        print(f'  corner {i}  target ({x:+6.1f},{y:+6.1f})  '
              f'reached ({pos[0]:+6.1f},{pos[1]:+6.1f})  '
              f'err {err:4.2f} m  {"ok" if ok else "TIMEOUT"}')

    pos = pilot.position()
    closing = math.dist((pos[0], pos[1]), (hx, hy))
    print('-' * 46)
    print(f'closing error: {closing:.2f} m over {4 * side:.0f} m flown')
    print('Compare with 06_walk_a_square on the Go2 — same shape, no estimator.')

    print('\nlanding ...')
    pilot.land()
    for _ in range(200):
        pilot.spin(0.1)
        if not pilot.armed:
            break
    print(f'landed, armed={pilot.armed}')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
