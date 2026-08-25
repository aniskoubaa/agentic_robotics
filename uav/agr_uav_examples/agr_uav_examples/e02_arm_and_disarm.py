#!/usr/bin/env python3
"""
UAV example 02 — arm the vehicle, then disarm it. No flying.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_uav_examples 02_arm_and_disarm

WHAT  Send one arm command, watch the state change, wait, send disarm.

LEARN - ARMING IS A REQUEST, NOT A SETTING. You publish a VehicleCommand and
        PX4 decides. It will refuse whenever its pre-flight checks fail, and
        the refusal arrives as "nothing happened" unless you watch
        vehicle_status.
      - Commands go on /fmu/in/vehicle_command as a generic VehicleCommand
        with a numeric `command` id and seven `param` floats. It is a MAVLink
        message wearing a ROS hat, which is why the interface looks like that.
      - `from_external = True` matters. PX4 distinguishes commands from
        onboard modules and commands from outside, and drops external ones
        that do not say so.
      - Arming a multirotor spins the propellers. In sim that is free; treat
        the habit as if it were not.
"""
import rclpy
from rclpy.node import Node

from agr_uav_tools.offboard import Pilot

HOLD_S = 5.0


def main():
    rclpy.init()
    node = Node('arm_and_disarm')
    node.declare_parameter('namespace', '')
    pilot = Pilot(node, node.get_parameter('namespace').value)

    if not pilot.wait_for_telemetry():
        print('no telemetry — is the sim running?')
        node.destroy_node(); rclpy.try_shutdown(); return

    # None, not False, when vehicle_status has never arrived — which is the
    # normal state of an idle vehicle. Reporting that as FAIL would be a lie.
    ok = pilot.preflight_ok
    label = {True: 'PASS', False: 'FAIL',
             None: 'unknown (vehicle_status publishes only on change)'}[ok]
    print(f'pre-flight checks: {label}')
    if ok is False:
        print('PX4 will refuse to arm. Run `ros2 run agr_uav_demos diagnose` '
              'to find out why.')

    print(f'armed now? {pilot.armed}')
    print('sending ARM ...')
    pilot.arm()
    for _ in range(40):                       # up to 4 s for the state to flip
        pilot.spin(0.1)
        if pilot.armed:
            break
    print(f'armed now? {pilot.armed}')

    if pilot.armed:
        print(f'holding armed for {HOLD_S:.0f}s — the propellers are spinning')
        pilot.spin(HOLD_S)
        print('sending DISARM ...')
        pilot.disarm()
        for _ in range(40):
            pilot.spin(0.1)
            if not pilot.armed:
                break
        print(f'armed now? {pilot.armed}')
    else:
        print('it refused. That is the interesting case — see diagnose.')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
