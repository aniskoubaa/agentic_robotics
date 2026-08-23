#!/usr/bin/env python3
"""
RAISE 2026 — Lab 1, Script 06 — move the UR5e robotic arm.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

WHAT  Show a tiny menu (1/2/3/4) and call the matching named-pose
      service. Each one moves the UR5e arm to a pre-defined joint
      configuration.

╔═══════════════════════════════════════════════════════════════════════════╗
║ WHERE ARE THE JOINT ANGLES IN THIS CODE?  →  THEY'RE NOT.                 ║
║                                                                           ║
║ This script (the CLIENT) only knows TOOL NAMES like '/move_to_home'.      ║
║ It does NOT compute or send joint angles.                                 ║
║                                                                           ║
║ The actual 6-D joint configuration lives in the SERVER:                   ║
║     raise2026_tools/raise2026_tools/move_to_pose_server.py                ║
║                                                                           ║
║ Open that file and search for the `POSES` dict. You'll see, for example:  ║
║                                                                           ║
║     POSES = {                                                             ║
║       'home': [0.0, -π/2, +π/2, -π/2, -π/2, 0.0],   ← 6 radians, one      ║
║       'above_plant': [0.0, -1.0, 0.8, -1.4, π/2, 0.0],   per UR5e joint   ║
║       ...                                                                 ║
║     }                                                                     ║
║                                                                           ║
║ The full data flow:                                                       ║
║                                                                           ║
║   THIS SCRIPT  ──Trigger srv──►  move_to_pose_server.py                   ║
║      (client)                       (publishes 6× Float64 to              ║
║                                        /ur5e_<joint>/cmd topics)          ║
║                                          │                                ║
║                                          ▼                                ║
║                                    ros_gz_bridge                          ║
║                                          │                                ║
║                                          ▼                                ║
║                               JointPositionController (Gazebo)            ║
║                                          │                                ║
║                                          ▼                                ║
║                                  UR5e arm moves in sim                    ║
║                                                                           ║
║ WHY this split?                                                           ║
║ Because that's how LLM "tool use" works. The agent picks a NAME           ║
║ ('move_to_home') from a list. The implementation is hidden. Same with     ║
║ 05_call_gripper.py: angles for open/close live in gripper_server.py.      ║
╚═══════════════════════════════════════════════════════════════════════════╝

LEARN - The "named pose" abstraction: instead of giving an LLM a 6-D
        joint-vector argument (error-prone) or a cartesian target
        (needs IK + MoveIt), expose a small SET of named services.
        Each one becomes a discrete tool the agent can call.
      - Multiple service clients in one node — same as in 05, but
        scaled to 4 services instead of 3.

Before running this, start the server in another terminal:

    ros2 run raise2026_tools move_to_pose_server

Then run this client:

    ros2 run raise2026_labs 06_call_robotic_arm.py   # or:  06_d1
"""

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


# Each menu entry maps keypress → (label shown, SERVICE NAME the server provides).
# Notice: no joint angles here. The string '/move_to_home' is just a NAME — the
# matching joint targets live in move_to_pose_server.py's POSES dict.
# Same pattern as 05_call_gripper.py — easy to extend by adding a row.
ACTIONS = {
    '1': ('home          (default ready pose)', '/move_to_home'),
    '2': ('above plant   (extended forward-down)', '/move_to_above_plant'),
    '3': ('side view     (rotated 90° to one side)', '/move_to_side_view'),
    '4': ('stow          (compact folded for driving)', '/move_to_stow'),
}

MENU = """
┌─── RAISE 2026 · Move-to-pose menu ──────────┐
│   1) home          (default ready pose)     │
│   2) above plant   (extended forward-down)  │
│   3) side view     (rotated 90° to a side)  │
│   4) stow          (compact for driving)    │
│   q) quit                                   │
└─────────────────────────────────────────────┘"""

SERVER_TIMEOUT = 5.0   # seconds to wait for the server


def main():
    rclpy.init()
    node = Node('move_to_pose_menu_client')

    # One client per service, created up front. Same pattern as 05.
    clients = {
        key: node.create_client(Trigger, srv_name)
        for key, (_, srv_name) in ACTIONS.items()
    }

    # Verify all 4 services are advertised before opening the menu —
    # better UX than letting one of them fail mid-loop.
    node.get_logger().info(f'Waiting up to {SERVER_TIMEOUT}s for the move_to_pose server…')
    for key, (label, srv_name) in ACTIONS.items():
        if not clients[key].wait_for_service(timeout_sec=SERVER_TIMEOUT):
            node.get_logger().error(
                f'No server on {srv_name}. Start it with:\n'
                f'    ros2 run raise2026_tools move_to_pose_server'
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
        node.get_logger().info(f'→ calling {srv_name}  ({label})')

        # call_async + spin_until_future_complete — the canonical pattern
        # for "synchronous-looking" service calls in rclpy.
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
