#!/usr/bin/env python3
"""
RAISE 2026 — Lab 1, Script 05 — interactive gripper menu.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

WHAT  Show a tiny menu (1/2/3) and call the matching gripper service:
        1) open        →  /open_gripper
        2) close       →  /close_gripper
        3) rotate 90°  →  /rotate_gripper
        q) quit

╔═══════════════════════════════════════════════════════════════════════════╗
║ WHERE ARE THE JOINT TARGETS IN THIS CODE?  →  THEY'RE NOT.                ║
║                                                                           ║
║ This script (the CLIENT) only knows TOOL NAMES like '/open_gripper'.      ║
║ It does NOT publish joint targets itself.                                 ║
║                                                                           ║
║ The actual finger-joint targets and the wrist-rotation step live in:      ║
║     raise2026_tools/raise2026_tools/gripper_server.py                     ║
║                                                                           ║
║ Open it and look for these constants:                                     ║
║     TARGET_OPEN   = 0.0       # radians for the driving knuckle joint     ║
║     TARGET_CLOSED = 0.5       # radians                                   ║
║     ROTATE_STEP   = math.pi/2 # +90° per call (on ur5e_wrist_3_joint)     ║
║                                                                           ║
║ Pattern: client picks a tool by NAME → server has the implementation.     ║
║ This is exactly how the LLM tool-use loop works in later labs.            ║
╚═══════════════════════════════════════════════════════════════════════════╝

LEARN - Multiple service clients in one node.
      - Looping in main() while still pumping ROS 2 callbacks via
        spin_until_future_complete.
      - Why services are good for "do this discrete action" tools
        (vs topics which are good for streaming data).

Before running this, start the server in another terminal:

    ros2 run raise2026_tools gripper_server

Then run this client:

    ros2 run raise2026_labs 05_call_gripper.py    # or:  05_d1
"""

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


# Menu mapping: keypress → (label shown, service name to call)
ACTIONS = {
    '1': ('open',         '/open_gripper'),
    '2': ('close',        '/close_gripper'),
    '3': ('rotate 90°',   '/rotate_gripper'),
}

MENU = """
┌─── RAISE 2026 · Gripper menu ───┐
│   1) open                       │
│   2) close                      │
│   3) rotate 90°                 │
│   q) quit                       │
└─────────────────────────────────┘"""

SERVER_TIMEOUT = 5.0   # seconds to wait for the server


def main():
    rclpy.init()
    node = Node('gripper_menu_client')

    # ─── Create one client per service ─────────────────────────────────────
    # We make all three up front, so we can check that all servers are
    # ready before the menu loop starts (better UX than failing mid-menu).
    clients = {
        key: node.create_client(Trigger, srv_name)
        for key, (_, srv_name) in ACTIONS.items()
    }

    # ─── Wait for the gripper_server to advertise the services ─────────────
    node.get_logger().info(f'Waiting up to {SERVER_TIMEOUT}s for the gripper server…')
    for key, (label, srv_name) in ACTIONS.items():
        if not clients[key].wait_for_service(timeout_sec=SERVER_TIMEOUT):
            node.get_logger().error(
                f'No server on {srv_name}. Start it with:\n'
                f'    ros2 run raise2026_tools gripper_server'
            )
            rclpy.shutdown()
            sys.exit(1)

    # ─── Menu loop ─────────────────────────────────────────────────────────
    while rclpy.ok():
        print(MENU)
        try:
            choice = input('  choose > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()                # clean newline on Ctrl-D / Ctrl-C
            break

        if choice == 'q':
            break

        if choice not in ACTIONS:
            print(f'  ⚠ unknown choice: {choice!r}')
            continue

        label, srv_name = ACTIONS[choice]
        node.get_logger().info(f'→ calling {srv_name}  ({label})')

        # `call_async` returns a Future. We spin the node until the
        # response is ready. This pattern (call_async + spin_until_future)
        # is THE canonical way to make a synchronous-looking service call
        # from rclpy.
        future = clients[choice].call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future)

        resp = future.result()
        if resp is None:
            print('  ✗ service call failed (no response)')
        else:
            print(f'  ✓ success={resp.success}  message="{resp.message}"')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
