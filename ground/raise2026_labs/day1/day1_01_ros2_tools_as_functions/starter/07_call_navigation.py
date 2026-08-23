#!/usr/bin/env python3
"""
RAISE 2026 — Lab 1, Script 07 — drive the Husky to a named waypoint.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

WHAT  Show a tiny menu (1/2/3/4) and call the matching navigation
      service. Each one drives the Husky base to a pre-defined (x, y)
      waypoint in the greenhouse world.

╔═══════════════════════════════════════════════════════════════════════════╗
║ WHERE ARE THE WAYPOINT COORDINATES IN THIS CODE?  →  THEY'RE NOT.         ║
║                                                                           ║
║ This script (the CLIENT) only knows TOOL NAMES like '/nav_to_home'.       ║
║ It does NOT plan the path or send any /cmd_vel messages.                  ║
║                                                                           ║
║ The actual (x, y) coordinates and the drive-controller live in:           ║
║     raise2026_tools/raise2026_tools/navigation_server.py                  ║
║                                                                           ║
║ Open it and look for the `WAYPOINTS` dict and the `_drive_to` method:     ║
║                                                                           ║
║     WAYPOINTS = {                                                         ║
║       'home':         ( 0.0,  0.0),                                       ║
║       'tomato_row_1': ( 3.0,  1.5),     ← edit these to match your        ║
║       'tomato_row_2': ( 3.0, -1.5),       world layout                    ║
║       'olive_grove':  ( 5.0,  3.0),                                       ║
║     }                                                                     ║
║                                                                           ║
║ The full data flow:                                                       ║
║                                                                           ║
║   THIS SCRIPT  ──Trigger srv──►  navigation_server.py                     ║
║      (client)                       ① reads /odom to know where it is     ║
║                                     ② P-controller: turn-in-place, then   ║
║                                       drive forward toward (x, y)         ║
║                                     ③ publishes /cmd_vel until within     ║
║                                       0.4 m of the goal                   ║
║                                                                           ║
║ The CALLBACK BLOCKS until the robot arrives (or 30 s timeout). The        ║
║ student-side call_async/spin_until_future_complete pattern handles        ║
║ that transparently — the response shows up when navigation finishes.      ║
║                                                                           ║
║ WHY this split (still)?                                                   ║
║ Because that's how LLM "tool use" works: the agent calls a NAME, the      ║
║ implementation hides the controller, sensors, and tuning. Identical to    ║
║ 05_call_gripper.py and 06_call_robotic_arm.py.                            ║
╚═══════════════════════════════════════════════════════════════════════════╝

LEARN - The "named waypoint" abstraction: navigation tools that the
        agent can pick by name (no path planning, no obstacle avoidance
        in the API — those are SERVER concerns).
      - A long-running service: the response comes back AFTER the
        robot has finished moving (or timed out). Client just waits.

Before running this, start the server in another terminal:

    ros2 run raise2026_tools navigation_server

Then run this client:

    ros2 run raise2026_labs 07_call_navigation.py    # or:  07_d1
"""

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


# Each menu entry maps keypress → (label shown, SERVICE NAME the server provides).
# No coordinates here — those live in navigation_server.py's WAYPOINTS dict.
ACTIONS = {
    '1': ('home          (back to origin)',           '/nav_to_home'),
    '2': ('tomato row 1  (front-left bed)',           '/nav_to_tomato_row_1'),
    '3': ('tomato row 2  (front-right bed)',          '/nav_to_tomato_row_2'),
    '4': ('olive grove   (further out)',              '/nav_to_olive_grove'),
}

MENU = """
┌─── RAISE 2026 · Navigation menu ────────────┐
│   1) home          (back to origin)         │
│   2) tomato row 1  (front-left bed)         │
│   3) tomato row 2  (front-right bed)        │
│   4) olive grove   (further out)            │
│   q) quit                                   │
└─────────────────────────────────────────────┘"""

SERVER_TIMEOUT = 5.0      # seconds to wait for the server to come up
# Navigation can take a while — the server gives up after 30 s, so 45 s
# here is comfortably more.
CALL_TIMEOUT   = 45.0


def main():
    rclpy.init()
    node = Node('navigation_menu_client')

    clients = {
        key: node.create_client(Trigger, srv_name)
        for key, (_, srv_name) in ACTIONS.items()
    }

    # Wait for all 4 services to be advertised before opening the menu.
    node.get_logger().info(f'Waiting up to {SERVER_TIMEOUT}s for the navigation server…')
    for key, (label, srv_name) in ACTIONS.items():
        if not clients[key].wait_for_service(timeout_sec=SERVER_TIMEOUT):
            node.get_logger().error(
                f'No server on {srv_name}. Start it with:\n'
                f'    ros2 run raise2026_tools navigation_server'
            )
            rclpy.shutdown()
            sys.exit(1)

    while rclpy.ok():
        print(MENU)
        try:
            choice = input('  choose > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if choice == 'q':
            break

        if choice not in ACTIONS:
            print(f'  ⚠ unknown choice: {choice!r}')
            continue

        label, srv_name = ACTIONS[choice]
        node.get_logger().info(f'→ calling {srv_name}  ({label})  — this may take ~30s')
        print('  (navigating … watch the Husky in Gazebo)')

        # call_async returns immediately; spin_until_future_complete blocks
        # until the SERVER replies. The server's reply happens when the
        # robot has arrived (or timed out). Pass a generous timeout so we
        # don't bail before the drive finishes.
        future = clients[choice].call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT)

        if not future.done():
            print(f'  ✗ no response within {CALL_TIMEOUT:.0f}s — cancelling')
            continue

        resp = future.result()
        if resp is None:
            print('  ✗ service call failed (no response)')
        else:
            mark = '✓' if resp.success else '✗'
            print(f'  {mark} success={resp.success}  message="{resp.message}"')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
