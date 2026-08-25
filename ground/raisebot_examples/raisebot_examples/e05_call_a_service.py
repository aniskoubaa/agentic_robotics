#!/usr/bin/env python3
"""
RaiseBot example 05 — the robot as a set of callable functions.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    agr-sim ground tools:=true            # the servers must be running
    ros2 run raisebot_examples 05_call_a_service
    ros2 run raisebot_examples 05_call_a_service --ros-args -p service:=/close_gripper

WHAT  Call one std_srvs/Trigger service and print what came back.

LEARN - A SERVICE IS A FUNCTION CALL: request in, response out, and the
        response says whether it worked. A topic cannot do that. "Did the
        gripper close?" has an answer; "drive at 0.3 m/s" does not.
      - THIS IS THE WHOLE PREMISE OF THE AGENTIC LABS. An LLM cannot run a
        velocity control loop, but it can absolutely call a function that
        returns {success, message}. Every tool server in raisebot_tools exists
        to put a function-shaped surface on top of a control-loop-shaped
        robot.
      - ALWAYS wait_for_service and ALWAYS check `success`. Calling a service
        nobody is serving blocks forever, and a service that returns
        success=False looks identical to one that worked if you never look.
      - `std_srvs/Trigger` has an EMPTY request. That is deliberate: a service
        with no arguments is trivially describable to a language model, so the
        waypoint goes in the service NAME (/nav_to_home) rather than in a
        field.
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

DEFAULT = '/open_gripper'


def main():
    rclpy.init()
    node = Node('call_a_service')
    node.declare_parameter('service', DEFAULT)
    name = node.get_parameter('service').value

    client = node.create_client(Trigger, name)
    print(f'looking for {name} ...')
    if not client.wait_for_service(timeout_sec=5.0):
        print(f'{name} is not being served.\n'
              '  start the tool servers:\n'
              '      ros2 launch raisebot_bringup tools.launch.py\n'
              '  or relaunch the sim with:  agr-sim ground tools:=true\n'
              '  list what IS available:    ros2 service list -t | grep Trigger')
        node.destroy_node(); rclpy.try_shutdown(); return

    print(f'calling {name} ...')
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)

    if not future.done():
        print('timed out — the server accepted the call but never answered')
    else:
        resp = future.result()
        # success=False is a normal, informative outcome, not an exception.
        print(f'  success: {resp.success}')
        print(f'  message: {resp.message}')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
