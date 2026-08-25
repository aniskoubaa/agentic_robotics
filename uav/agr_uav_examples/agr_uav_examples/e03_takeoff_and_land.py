#!/usr/bin/env python3
"""
UAV example 03 — take off, hover, land.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_uav_examples 03_takeoff_and_land
    ros2 run agr_uav_examples 03_takeoff_and_land --ros-args -p altitude:=5.0

WHAT  The shortest complete flight: up, wait, down.

LEARN - OFFBOARD MODE HAS AN ORDER, and getting it wrong fails silently.
        Setpoints must ALREADY be streaming before you ask PX4 to enter
        offboard; ask first and the request is simply rejected. Look at
        Pilot.enter_offboard — the warm-up loop is the whole trick.
      - AND YOU MUST NOT STOP. If setpoints stop arriving for about half a
        second PX4 leaves offboard and goes to failsafe. This is why "hover
        for five seconds" is a publishing loop and never a sleep().
      - Landing is NOT an offboard setpoint at ground level. It is a mode
        change (VEHICLE_CMD_NAV_LAND); PX4 then runs its own descent and
        disarms when it detects touchdown. Trying to fly to z=0 yourself
        means fighting the ground.
"""
import rclpy
from rclpy.node import Node

from agr_uav_tools.offboard import Pilot


def main():
    rclpy.init()
    node = Node('takeoff_and_land')
    node.declare_parameter('altitude', 2.5)
    node.declare_parameter('hover_seconds', 5.0)
    node.declare_parameter('namespace', '')
    alt = float(node.get_parameter('altitude').value)
    hover = float(node.get_parameter('hover_seconds').value)
    pilot = Pilot(node, node.get_parameter('namespace').value)

    print(f'taking off to {alt:.1f} m ...')
    if not pilot.takeoff(alt):
        print('takeoff failed. `ros2 run agr_uav_demos diagnose` will say why.')
        pilot.disarm()
        node.destroy_node(); rclpy.try_shutdown(); return

    pos = pilot.position()
    print(f'airborne at {pos[2]:.2f} m — hovering {hover:.0f}s')
    # A hover is a publishing loop. Stop publishing and PX4 drops offboard.
    start = pilot.position()
    pilot.hold(start[0], start[1], alt, hover)

    print('landing ...')
    pilot.land()
    for _ in range(200):                      # up to 20 s to touch down
        pilot.spin(0.1)
        if not pilot.armed:
            break
    pos = pilot.position()
    print(f'landed at {pos[2]:.2f} m, armed={pilot.armed}')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
