#!/usr/bin/env python3
"""
RAISE 2026 — Lab 1, Script 09 — VLM inspector.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

WHAT  Send the current camera frame to an LLM with vision (GPT-4o)
      and get a short natural-language verdict back. Two flavours:
        1) inspect plant   (wrist camera + structured plant-health prompt)
        2) describe scene  (PTZ   camera + free-form scene description)

      A live OpenCV window ("RAISE 2026 - VLM inspector") shows the camera
      the VLM is looking at, an "Asking GPT-4o…" indicator while a call is
      in flight, and the most recent verdict + latency overlaid on the
      bottom of the frame. Watch your screen when you pick an option.

╔═══════════════════════════════════════════════════════════════════════════╗
║ WHERE IS THE LLM CALL?  →  IN THE SERVER, NOT HERE.                       ║
║                                                                           ║
║ This script only knows tool NAMES — '/inspect_plant' and '/describe_      ║
║ scene'. It doesn't talk to OpenAI, doesn't load any model, doesn't        ║
║ need an API key to run.                                                   ║
║                                                                           ║
║ The OpenAI client + the system prompts live in:                           ║
║     raise2026_tools/raise2026_tools/inspector_server.py                   ║
║                                                                           ║
║ Look for `PLANT_PROMPT` and `SCENE_PROMPT` — they're plain English. To    ║
║ change what the inspector "knows" how to do, you edit those strings.      ║
║ That's the whole "LLM agent" trick: behaviour = prompt.                   ║
║                                                                           ║
║ Compare with 08 (YOLO):                                                   ║
║   YOLO   →  structured output (class + confidence) from a fixed model.   ║
║   VLM    →  open-ended text from a prompt you can edit.                  ║
║   Both are tools the LLM agent in D1L2 will pick between.                ║
╚═══════════════════════════════════════════════════════════════════════════╝

LEARN - A "tool" can wrap a remote API call. The latency (~1-2 s) and
        cost (~$0.0001 per call with gpt-4o-mini at low detail) are
        server concerns, hidden behind a normal Trigger service.
      - Prompts ARE the behaviour. Edit PLANT_PROMPT in the server and
        you've changed what /inspect_plant does — no code recompile.

Before running this you need:
    export OPENAI_API_KEY=sk-...
    pip install --user --break-system-packages openai
    ros2 run raise2026_tools inspector_server   # in another terminal

Then run this client:

    ros2 run raise2026_labs 09_call_inspector.py    # or:  09_d1
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


ACTIONS = {
    '1': ('inspect plant  (wrist cam · structured verdict)',  '/inspect_plant'),
    '2': ('describe scene (PTZ cam   · free-form text)',      '/describe_scene'),
}

MENU = """
┌─── RAISE 2026 · VLM inspector menu ──────────────┐
│   1) inspect plant   (wrist cam · plant verdict) │
│   2) describe scene  (PTZ cam   · free-form text)│
│   q) quit                                        │
└──────────────────────────────────────────────────┘"""

SERVER_TIMEOUT = 10.0
# VLM calls take ~1-3 s for "low detail" images. 20 s is plenty of headroom.
CALL_TIMEOUT   = 20.0


def main():
    rclpy.init()
    node = Node('inspector_menu_client')

    clients = {
        key: node.create_client(Trigger, srv_name)
        for key, (_, srv_name) in ACTIONS.items()
    }

    node.get_logger().info(f'Waiting up to {SERVER_TIMEOUT:.0f}s for the inspector server …')
    for key, (label, srv_name) in ACTIONS.items():
        if not clients[key].wait_for_service(timeout_sec=SERVER_TIMEOUT):
            node.get_logger().error(
                f'No server on {srv_name}. Start it with:\n'
                f'    ros2 run raise2026_tools inspector_server\n'
                f'(needs OPENAI_API_KEY in env)'
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
        print('  (asking the VLM … look at the viewer window for the frame + verdict overlay)')

        t0 = time.time()
        future = clients[choice].call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT)
        elapsed = time.time() - t0

        if not future.done():
            print(f'  ✗ no response within {CALL_TIMEOUT:.0f}s')
            continue

        resp = future.result()
        if resp is None:
            print('  ✗ service call failed (no response)')
        else:
            mark = '✓' if resp.success else '✗'
            print(f'  {mark} {srv_name}  ({elapsed:.1f}s)')
            print(f'    ┌── VLM verdict ─────────────────────────────────')
            for line in resp.message.splitlines():
                print(f'    │ {line}')
            print(f'    └────────────────────────────────────────────────')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
