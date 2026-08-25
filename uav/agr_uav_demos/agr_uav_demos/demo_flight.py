#!/usr/bin/env python3
"""
UAV — the five-minute demo.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    agr-sim airframe:=x500_mono_cam world:=agr_city
    ros2 run agr_uav_demos demo_flight

WHAT  A scripted flight that shows the whole capability set in one run:
      arm, climb, translate, orbit a point, yaw in place, descend, land.
      Narrated on the terminal, so it can be run in front of an audience.

The orbit is the part worth watching. It is not a PX4 mode — it is this
script computing a circle of setpoints and streaming them at 20 Hz. Once you
can stream position setpoints, arbitrary trajectories are just arithmetic,
and that is the whole reason offboard mode exists.

Add a camera view in another terminal:
    ros2 run agr_uav_teleop camera_view
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node

from agr_uav_tools.offboard import Pilot, SETPOINT_HZ

ALT = 4.0
LEG = 6.0
ORBIT_R = 5.0
ORBIT_S = 20.0        # seconds for one full circle


def orbit(pilot: Pilot, cx: float, cy: float, radius: float,
          alt: float, seconds: float) -> None:
    """Fly a circle by streaming setpoints around it, nose always inward.

    Yaw is set to point AT the centre, which is what a camera doing an
    inspection orbit needs. atan2 of the vector from the aircraft to the
    centre — not the other way round, which flies backwards looking out.
    """
    end = time.time() + seconds
    start = time.time()
    while rclpy.ok() and time.time() < end:
        theta = 2 * math.pi * (time.time() - start) / seconds
        x = cx + radius * math.cos(theta)
        y = cy + radius * math.sin(theta)
        yaw = math.atan2(cy - y, cx - x)
        pilot.send_setpoint(x, y, alt, yaw)
        rclpy.spin_once(pilot.node, timeout_sec=1.0 / SETPOINT_HZ)


def main() -> int:
    rclpy.init()
    node = Node('demo_flight')
    node.declare_parameter('namespace', '')
    pilot = Pilot(node, node.get_parameter('namespace').value)

    print('\nUAV demo flight\n' + '=' * 56)
    if not pilot.wait_for_telemetry():
        print('no telemetry from PX4 — start the sim with `agr-sim`')
        node.destroy_node(); rclpy.try_shutdown(); return 1

    try:
        print(f'1/6  Arming and climbing to {ALT:.0f} m')
        if not pilot.takeoff(ALT):
            print('     takeoff failed — `ros2 run agr_uav_demos diagnose`')
            pilot.disarm()
            node.destroy_node(); rclpy.try_shutdown(); return 1
        home = pilot.position()
        hx, hy = home[0], home[1]

        print(f'2/6  Flying {LEG:.0f} m north')
        pilot.goto(hx + LEG, hy, ALT)

        print(f'3/6  Flying {LEG:.0f} m east')
        pilot.goto(hx + LEG, hy + LEG, ALT)

        print(f'4/6  Orbiting the start point at {ORBIT_R:.0f} m, nose inward')
        orbit(pilot, hx, hy, ORBIT_R, ALT, ORBIT_S)

        print('5/6  Returning home and yawing on the spot')
        pilot.goto(hx, hy, ALT)
        for deg in (90, 180, 270, 0):
            pilot.hold(hx, hy, ALT, 2.0, yaw=math.radians(deg))

        print('6/6  Landing')
        pilot.land()
        deadline = time.time() + 25.0
        while rclpy.ok() and pilot.armed and time.time() < deadline:
            pilot.spin(0.1)
        pos = pilot.position()
        print('=' * 56)
        print(f'done — down at {pos[2]:.2f} m, armed={pilot.armed}\n')
    except KeyboardInterrupt:
        print('\ninterrupted — landing')
        pilot.land()
        deadline = time.time() + 25.0
        while rclpy.ok() and pilot.armed and time.time() < deadline:
            pilot.spin(0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
