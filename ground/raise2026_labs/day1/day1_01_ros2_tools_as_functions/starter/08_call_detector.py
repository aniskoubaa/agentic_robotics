#!/usr/bin/env python3
"""
RAISE 2026 — Lab 1, Script 08 — call the YOLO object detector.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

WHAT  Show a menu of YOLO detection actions, one per camera × mode pair.
      The server is the first lab tool to run a real ML model — but to
      this client, it's just another Trigger service.

      A live OpenCV window pops up showing the camera frame with the YOLO
      bounding boxes drawn, updated on every detection (the stream modes
      animate it). Watch your screen when you pick an option.

╔═══════════════════════════════════════════════════════════════════════════╗
║ WHERE IS THE YOLO MODEL?  →  IN THE SERVER.                               ║
║                                                                           ║
║ This script only knows tool NAMES like '/detect_wrist'. It doesn't load   ║
║ a model, doesn't subscribe to images, doesn't even import torch.          ║
║                                                                           ║
║ The model + inference live in:                                            ║
║     raise2026_tools/raise2026_tools/detector_server.py                    ║
║                                                                           ║
║ The server uses `yolov8n` (the tiniest YOLOv8 variant, ~6 MB), pre-       ║
║ trained on COCO classes. Our greenhouse world has chickens and ducks,     ║
║ which YOLO recognises as 'bird' — so the detector "sees" something out    ║
║ of the box. For real PLANT disease classification you'd swap yolov8n.pt   ║
║ for a fine-tuned plant model — same code, different weights.              ║
║                                                                           ║
║ This is the SAME PATTERN as 05/06/07: client picks a name, server has     ║
║ the implementation. The next-lab LLM will call these tools the same way.  ║
╚═══════════════════════════════════════════════════════════════════════════╝

LEARN - A tool can run a real ML model behind the scenes — but its
        interface (Trigger in, success/message out) is unchanged.
      - Three useful "shapes" for a perception tool:
            now        → one-shot, fast, returns top classes
            annotated  → same + drops a PNG with drawn boxes
            stream     → samples for 5 s so you can watch live output
      - Two cameras give you two viewpoints with one server. The arm-
        mounted wrist cam is great after /move_to_above_plant; the
        PTZ mast cam is a wide always-on scout.

Before running this, start the server in another terminal:

    ros2 run raise2026_tools detector_server

Then run this client:

    ros2 run raise2026_labs 08_call_detector.py    # or:  08_d1
"""

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


# Menu: 6 entries = 2 cameras × 3 modes. Each maps to one Trigger service.
ACTIONS = {
    '1': ('wrist · detect now',           '/detect_wrist'),
    '2': ('wrist · save annotated PNG',   '/detect_wrist_annotated'),
    '3': ('wrist · stream for 5 s',       '/detect_wrist_stream'),
    '4': ('PTZ   · detect now',           '/detect_ptz'),
    '5': ('PTZ   · save annotated PNG',   '/detect_ptz_annotated'),
    '6': ('PTZ   · stream for 5 s',       '/detect_ptz_stream'),
}

MENU = """
┌─── RAISE 2026 · YOLO detector menu ──────────┐
│   ── wrist camera (arm end-effector) ──      │
│   1) detect now                              │
│   2) save annotated PNG                      │
│   3) stream detections for 5 s               │
│   ── PTZ camera (mast) ──                    │
│   4) detect now                              │
│   5) save annotated PNG                      │
│   6) stream detections for 5 s               │
│   q) quit                                    │
│   ↳ a viewer window shows the boxes live     │
└──────────────────────────────────────────────┘"""

SERVER_TIMEOUT = 60.0   # First run of the server downloads yolov8n.pt (~6 MB).
                        # Subsequent runs come up fast.
# Inference is fast (~50 ms), annotated mode saves a PNG (~100 ms), stream
# blocks for 5 s. 15 s is plenty for all modes.
CALL_TIMEOUT   = 15.0


def main():
    rclpy.init()
    node = Node('detector_menu_client')

    clients = {
        key: node.create_client(Trigger, srv_name)
        for key, (_, srv_name) in ACTIONS.items()
    }

    node.get_logger().info(
        f'Waiting up to {SERVER_TIMEOUT:.0f}s for the detector server '
        '(first run downloads YOLO weights, may take a moment) …'
    )
    for key, (label, srv_name) in ACTIONS.items():
        if not clients[key].wait_for_service(timeout_sec=SERVER_TIMEOUT):
            node.get_logger().error(
                f'No server on {srv_name}. Start it with:\n'
                f'    ros2 run raise2026_tools detector_server'
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

        future = clients[choice].call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT)

        if not future.done():
            print(f'  ✗ no response within {CALL_TIMEOUT:.0f}s')
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
