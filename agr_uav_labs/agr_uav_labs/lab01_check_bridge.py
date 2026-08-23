# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""Lab 01 — is the PX4 <-> ROS 2 bridge actually working?

    ros2 run agr_uav_labs 01_check_bridge

Every later lab assumes this passes. It fails with a specific diagnosis rather
than a timeout, because "nothing happened" is the least useful error there is.
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from px4_msgs.msg import BatteryStatus, VehicleStatus

from agr_uav_tools import px4_topics
from agr_uav_tools.px4_topics import PX4_QOS

TIMEOUT_S = 10.0     # how long to wait for the first message
DISCOVERY_S = 8.0    # how long to wait for the DDS graph to populate


def _await_fmu_topics(node, seconds: float) -> list:
    """Spin until /fmu topics show up in the graph, or `seconds` elapse."""
    deadline = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    found: list = []
    while rclpy.ok():
        found = [n for n, _ in node.get_topic_names_and_types() if '/fmu/' in n]
        if found or node.get_clock().now().nanoseconds > deadline:
            break
        rclpy.spin_once(node, timeout_sec=0.25)
    return found


def main(args=None) -> int:
    rclpy.init(args=args)
    node = Node('lab01_check_bridge')
    namespace = ''

    # DDS discovery is not instantaneous. A node that queries
    # get_topic_names_and_types() the moment it is created routinely sees an
    # empty graph even when PX4 has been publishing for minutes — the local
    # participant simply has not finished discovering the remote one. Reporting
    # "PX4 is not running" on the strength of that is a WRONG diagnosis, and
    # the most expensive kind: it sends you to debug a healthy system.
    # Spin until topics appear or the discovery window expires.
    all_fmu = _await_fmu_topics(node, DISCOVERY_S)
    print(f'1. /fmu topics visible ........ {len(all_fmu)}'
          f'   (waited up to {DISCOVERY_S:.0f}s for discovery)')
    if not all_fmu:
        print('\n   ✗ No /fmu topics at all.')
        print('     Either PX4 is not running, or the Micro XRCE-DDS agent is not.')
        print('     Start both with:  ros2 launch agr_uav_bringup single.launch.py')
        node.destroy_node(); rclpy.shutdown()
        return 1

    # Step 3 deliberately uses battery_status, NOT vehicle_status.
    #
    # PX4 publishes vehicle_status ONLY ON CHANGE. On an idle, disarmed vehicle
    # nothing has changed since boot, so a fresh subscriber receives nothing —
    # for minutes — while the bridge is completely healthy. Testing liveness
    # against it reports a working system as broken, which is the most
    # expensive kind of wrong answer a diagnostic can give.
    #
    # battery_status is published periodically (1 Hz here), so "no message"
    # genuinely means "no data path".
    topic = px4_topics.resolve(node, px4_topics.BATTERY_STATUS, 'out', namespace)
    print(f'2. battery_status resolved to .. {topic}')
    if topic is None:
        print('\n   ✗ /fmu topics exist but battery_status is not among them.')
        print('     Usually a px4_msgs / PX4 version mismatch.')
        node.destroy_node(); rclpy.shutdown()
        return 1

    got = {'msg': None}
    node.create_subscription(BatteryStatus, topic,
                             lambda m: got.__setitem__('msg', m), PX4_QOS)

    deadline = node.get_clock().now().nanoseconds + int(TIMEOUT_S * 1e9)
    while rclpy.ok() and got['msg'] is None:
        if node.get_clock().now().nanoseconds > deadline:
            break
        rclpy.spin_once(node, timeout_sec=0.2)

    if got['msg'] is None:
        print(f'3. data received .............. NO (waited {TIMEOUT_S:.0f}s)')
        print('\n   ✗ The topic is advertised but delivered nothing.')
        print('     PX4 is BEST_EFFORT — a default RELIABLE subscription never')
        print('     matches and never errors. Use agr_uav_tools.px4_topics.PX4_QOS.')
        node.destroy_node(); rclpy.shutdown()
        return 1

    print(f'3. data received .............. yes '
          f'(battery {got["msg"].remaining * 100:.0f}%)')

    # vehicle_status is reported separately, and its silence is NOT a failure.
    st_topic = px4_topics.resolve(node, px4_topics.VEHICLE_STATUS, 'out', namespace)
    st = {'msg': None}
    if st_topic:
        node.create_subscription(VehicleStatus, st_topic,
                                 lambda m: st.__setitem__('msg', m), PX4_QOS)
        d2 = node.get_clock().now().nanoseconds + int(3e9)
        while rclpy.ok() and st['msg'] is None and node.get_clock().now().nanoseconds < d2:
            rclpy.spin_once(node, timeout_sec=0.2)
    if st['msg'] is not None:
        print(f'4. vehicle_status ............. arming_state='
              f'{st["msg"].arming_state}, nav_state={st["msg"].nav_state}')
    else:
        print('4. vehicle_status ............. silent — EXPECTED on an idle '
              'vehicle (published on change only), not a fault')

    print('\n   ✓ Bridge is up. On to lab 02.')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
