#!/usr/bin/env python3
"""
UAV example 01 — is the drone talking to us?
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_uav_examples 01_read_telemetry

WHAT  Subscribe to PX4's state and position and print them until Ctrl-C.

LEARN - PX4 is NOT a ROS node. It is separate flight-control firmware, and
        everything you see here crossed a bridge: PX4's internal uORB
        messages → the Micro XRCE-DDS agent → DDS → your subscription.
        Nothing works if that bridge is down, and that is the usual fault.
      - THE QoS MUST MATCH. PX4 publishes BEST_EFFORT. A default ROS 2
        subscription is RELIABLE, which is INCOMPATIBLE, so it never matches
        and never receives — no error, no warning, just silence that looks
        exactly like a dead vehicle. `px4_topics.PX4_QOS` is not optional.
      - TOPIC NAMES CARRY VERSION SUFFIXES that change between PX4 releases
        (/fmu/out/vehicle_status_v4 today). Hard-code one and your code breaks
        on the next PX4 bump, again silently. Resolve at runtime.
      - Position is NED: down is positive. This prints altitude as UP, which
        is the conversion every one of these scripts does at the boundary.
      - PREFLIGHT SHOWS '?' UNTIL SOMETHING CHANGES. vehicle_status publishes
        only on change and the bridge does not replay the last one, so a
        script that starts after the vehicle settled has genuinely never been
        told. '?' is the honest answer; printing FAIL would be a lie.
"""
import math

import rclpy
from rclpy.node import Node

from agr_uav_tools.offboard import Pilot

def main():
    rclpy.init()
    node = Node('read_telemetry')
    node.declare_parameter('namespace', '')
    pilot = Pilot(node, node.get_parameter('namespace').value)

    print('waiting for PX4 telemetry ...')
    if not pilot.wait_for_telemetry():
        print('nothing arrived in 15 s.\n'
              '  is the sim running?          agr-sim\n'
              '  is the XRCE agent running?   ros2 run agr_uav_demos diagnose')
        node.destroy_node(); rclpy.try_shutdown(); return

    print('connected — Ctrl-C to stop\n')
    try:
        while rclpy.ok():
            pilot.spin(0.25)
            pos = pilot.position()
            arm = 'ARMED' if pilot.armed else 'DISARMED'
            ok = pilot.preflight_ok
            checks = {True: 'PASS', False: 'FAIL', None: '?'}[ok]
            where = ('n=%6.1f e=%6.1f up=%5.1f' % pos) if pos else 'no position yet'
            print(f'\r{arm:<9} preflight {checks:<5} {where}  '
                  f'yaw {math.degrees(pilot.yaw()):+6.1f}°   ', end='', flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
