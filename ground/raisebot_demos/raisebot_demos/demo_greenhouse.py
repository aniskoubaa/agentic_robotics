#!/usr/bin/env python3
"""
RaiseBot — the five-minute demo.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    agr-sim ground tools:=true
    ros2 run raisebot_demos demo_greenhouse

WHAT  A narrated inspection run: drive to a crop row by name, aim the mast
      camera at it, pose the arm, work the gripper, then come home.

WHY this shape: every step is a SERVICE CALL, not a control loop. That is the
    whole premise of the agentic labs — an LLM cannot run a velocity loop, but
    it can call a function that returns {success, message}. Watch the terminal
    and you are watching exactly the trace an agent would produce.

The demo reports each step's success and keeps going, because a demo that
aborts on the first hiccup is worse than useless in front of an audience.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

# (narration, service) — in the order an inspection actually happens.
STEPS = [
    ('Driving to tomato row 1',            '/nav_to_tomato_row_1'),
    ('Raising the arm to look at a plant', '/move_to_above_plant'),
    ('Opening the gripper',                '/open_gripper'),
    ('Closing the gripper on a truss',     '/close_gripper'),
    ('Stowing the arm',                    '/move_to_home'),
    ('Driving to tomato row 2',            '/nav_to_tomato_row_2'),
    ('Driving back home',                  '/nav_to_home'),
]

# Aim the mast camera down the row before we set off.
PTZ_PAN, PTZ_TILT = 0.0, -0.25
CALL_TIMEOUT_S = 120.0


def call(node: Node, service: str) -> tuple:
    client = node.create_client(Trigger, service)
    if not client.wait_for_service(timeout_sec=3.0):
        node.destroy_client(client)
        return False, 'no server — start them with  agr-sim ground tools:=true'
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT_S)
    node.destroy_client(client)
    if not future.done():
        return False, f'timed out after {CALL_TIMEOUT_S:.0f}s'
    resp = future.result()
    return resp.success, resp.message


def main() -> int:
    rclpy.init()
    node = Node('demo_greenhouse')

    pan = node.create_publisher(Float64, '/ptz/pan/cmd', 10)
    tilt = node.create_publisher(Float64, '/ptz/tilt/cmd', 10)

    print('\nRaiseBot greenhouse demo\n' + '=' * 60)
    # Discovery first, or this publish reaches nobody.
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.02)
    print(f'0/{len(STEPS)}  Aiming the mast camera down the row')
    for _ in range(20):
        pan.publish(Float64(data=PTZ_PAN))
        tilt.publish(Float64(data=PTZ_TILT))
        rclpy.spin_once(node, timeout_sec=0.05)

    failures = 0
    try:
        for i, (label, service) in enumerate(STEPS, 1):
            print(f'{i}/{len(STEPS)}  {label}')
            started = time.time()
            ok, msg = call(node, service)
            mark = 'ok  ' if ok else 'FAIL'
            if not ok:
                failures += 1
            print(f'        [{mark}] {service}  ({time.time() - started:.1f}s)')
            print(f'        {msg}')
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    print('=' * 60)
    print(f'done — {len(STEPS) - failures}/{len(STEPS)} steps succeeded\n'
          if failures else 'done — every step succeeded\n')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
