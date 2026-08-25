#!/usr/bin/env python3
"""
RaiseBot example 06 — send the robot somewhere by name.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    agr-sim ground tools:=true
    ros2 run raisebot_examples 06_navigate
    ros2 run raisebot_examples 06_navigate --ros-args -p waypoint:=olive_grove

WHAT  Call /nav_to_<waypoint>, wait for it to finish, and report where the
      robot ended up according to its own odometry.

LEARN - Compare with 01_drive. There you said "go this fast for this long" and
        hoped. Here you say WHERE, and something else owns the loop that gets
        you there and tells you when it is done. That is the difference
        between actuation and navigation.
      - The waypoint is part of the SERVICE NAME, not an argument. Adding a
        waypoint is one line in navigation_server's WAYPOINTS dict and the
        service appears by itself.
      - ODOMETRY DRIFTS. A skid-steer base slips on every turn, so wheel
        odometry alone will confidently report arrival while sitting metres
        away. navigation_server re-anchors against Gazebo ground truth for
        exactly this reason — read its `_apply_anchor` and note that this is
        the sim standing in for the GPS/SLAM you would need outdoors.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger

WAYPOINTS = ('home', 'tomato_row_1', 'tomato_row_2', 'olive_grove')


def main():
    rclpy.init()
    node = Node('navigate')
    node.declare_parameter('waypoint', 'tomato_row_1')
    wp = node.get_parameter('waypoint').value
    name = f'/nav_to_{wp}'

    pose = {}
    node.create_subscription(
        Odometry, '/odom',
        lambda m: pose.update(x=m.pose.pose.position.x,
                              y=m.pose.pose.position.y), 10)

    client = node.create_client(Trigger, name)
    if not client.wait_for_service(timeout_sec=5.0):
        print(f'{name} is not being served.\n'
              f'  known waypoints: {", ".join(WAYPOINTS)}\n'
              '  start the servers:  agr-sim ground tools:=true')
        node.destroy_node(); rclpy.try_shutdown(); return

    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.02)
    start = (pose.get('x', 0.0), pose.get('y', 0.0))
    print(f'at ({start[0]:+.2f}, {start[1]:+.2f}) — calling {name}')

    future = client.call_async(Trigger.Request())
    # Driving takes a while. Keep spinning so /odom stays current.
    rclpy.spin_until_future_complete(node, future, timeout_sec=120.0)

    if not future.done():
        print('timed out — the robot is probably still driving')
    else:
        resp = future.result()
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.02)
        end = (pose.get('x', 0.0), pose.get('y', 0.0))
        print(f'  success: {resp.success}')
        print(f'  message: {resp.message}')
        print(f'  now at ({end[0]:+.2f}, {end[1]:+.2f}), '
              f'travelled {math.dist(start, end):.2f} m')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
