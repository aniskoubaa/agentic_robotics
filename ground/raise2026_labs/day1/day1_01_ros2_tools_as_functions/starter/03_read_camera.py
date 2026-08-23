#!/usr/bin/env python3
"""
RAISE 2026 — Lab 1, Script 03 — interactive camera explorer.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

WHAT  Show a menu of things you can do with the PTZ camera topic
      /ptz_camera/image_raw: save a frame, dump metadata, compute
      pixel statistics, or record a short timestamped sequence.

LEARN - sensor_msgs/Image is metadata (width / height / encoding /
        frame_id / timestamp) + raw pixel bytes packed into `data`.
      - `cv_bridge` converts an Image ↔ a numpy array OpenCV can use.
      - Once you have a numpy array, vision in ROS 2 is just normal
        OpenCV / NumPy — no special tricks.

Run while `raise-sim` is up:

    ros2 run raise2026_labs 03_read_camera.py    # or:  03_d1
"""

import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


TOPIC = '/ptz_camera/image_raw'
# Save frames into a `frames/` folder next to THIS script, so the
# output never escapes the lab directory no matter where it's run from.
FRAMES_DIR = Path(__file__).resolve().parent / 'frames'


class CameraOnce(Node):
    """Subscribe and cache the most recent Image."""

    def __init__(self):
        super().__init__('camera_once')
        self.bridge = CvBridge()
        self.latest_msg: 'Image | None' = None
        self.create_subscription(Image, TOPIC, self._on_image, 10)

    def _on_image(self, msg: Image) -> None:
        self.latest_msg = msg

    def wait_for_image(self, timeout_s: float = 3.0) -> 'Image | None':
        """Spin until a frame arrives, or `timeout_s` elapses."""
        self.latest_msg = None
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and self.latest_msg is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_msg


def _timestamped_name(ext: str = 'png') -> Path:
    """Return a unique, sortable filename under FRAMES_DIR."""
    FRAMES_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]   # ms precision
    return FRAMES_DIR / f'frame_{stamp}.{ext}'


# ─── Menu actions ──────────────────────────────────────────────────────────
def save_one_frame(node: CameraOnce) -> None:
    """Action 1 — convert one Image to BGR numpy and write a PNG."""
    msg = node.wait_for_image()
    if msg is None:
        print(f'  ✗ no message on {TOPIC} within 3 s — is the sim running?')
        return
    img = node.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    out = _timestamped_name()
    cv2.imwrite(str(out), img)
    h, w = img.shape[:2]
    print(f'  ✓ saved {w}×{h} frame → {out}')


def show_metadata(node: CameraOnce) -> None:
    """Action 2 — print the structural fields of one Image message."""
    msg = node.wait_for_image()
    if msg is None:
        print(f'  ✗ no message on {TOPIC} within 3 s — is the sim running?')
        return
    bytes_per_msg = len(msg.data)
    print('  ┌── Image metadata ────────────────────────────────┐')
    print(f'  │ frame_id          : {msg.header.frame_id:<29s} │')
    print(f'  │ width × height    : {msg.width} × {msg.height:<22d} │')
    print(f'  │ encoding          : {msg.encoding:<29s} │')
    print(f'  │ step (bytes/row)  : {msg.step:<29d} │')
    print(f'  │ data size         : {bytes_per_msg:<22d} bytes        │')
    print(f'  │ timestamp.sec     : {msg.header.stamp.sec:<29d} │')
    print(f'  │ timestamp.nanosec : {msg.header.stamp.nanosec:<29d} │')
    print('  └──────────────────────────────────────────────────┘')
    print('  ► raw bytes = width × height × bytes_per_pixel (3 for rgb8/bgr8)')


def show_pixel_stats(node: CameraOnce) -> None:
    """Action 3 — compute mean BGR + brightness for one frame.

    Demonstrates the "Image → numpy → standard math" workflow. Once
    you have a numpy array, any NumPy/OpenCV operation works.
    """
    msg = node.wait_for_image()
    if msg is None:
        print(f'  ✗ no message on {TOPIC} within 3 s — is the sim running?')
        return
    img = node.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # `img.mean(axis=(0, 1))` averages over both spatial dimensions and
    # leaves one mean per channel: [meanB, meanG, meanR].
    mean_bgr = img.mean(axis=(0, 1))
    # ITU-R BT.601 luma — a common scalar brightness measure.
    brightness = 0.114 * mean_bgr[0] + 0.587 * mean_bgr[1] + 0.299 * mean_bgr[2]
    print('  ┌── Pixel statistics ────────────────────┐')
    print(f'  │ mean B (blue)   : {mean_bgr[0]:6.1f}  (0..255)  │')
    print(f'  │ mean G (green)  : {mean_bgr[1]:6.1f}  (0..255)  │')
    print(f'  │ mean R (red)    : {mean_bgr[2]:6.1f}  (0..255)  │')
    print(f'  │ brightness      : {brightness:6.1f}  (0..255)  │')
    print(f'  │ min / max pixel : {int(img.min()):3d} / {int(img.max()):3d}                 │')
    print('  └────────────────────────────────────────┘')


def capture_sequence(node: CameraOnce, count: int = 5, gap_s: float = 1.0) -> None:
    """Action 4 — record N frames `gap_s` seconds apart.

    Useful when you want a small dataset: drive the robot a little
    between captures and you've got a tiny image set.
    """
    print(f'  Capturing {count} frames, one every {gap_s:.1f}s …')
    for k in range(1, count + 1):
        msg = node.wait_for_image()
        if msg is None:
            print(f'  ✗ {k}/{count}: no frame within 3 s — aborting')
            return
        img = node.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        out = _timestamped_name()
        cv2.imwrite(str(out), img)
        print(f'  ✓ {k}/{count}  →  {out.name}')
        if k < count:
            time.sleep(gap_s)


ACTIONS = {
    '1': ('save one frame to disk',  save_one_frame),
    '2': ('show image metadata',     show_metadata),
    '3': ('show pixel statistics',   show_pixel_stats),
    '4': ('capture sequence (5×)',   capture_sequence),
}

MENU = """
┌─── RAISE 2026 · Camera menu ────┐
│   1) save one frame to disk     │
│   2) show image metadata        │
│   3) show pixel statistics      │
│   4) capture sequence (5×)      │
│   q) quit                       │
└─────────────────────────────────┘"""


def main():
    rclpy.init()
    node = CameraOnce()

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

        label, action = ACTIONS[choice]
        node.get_logger().info(f'→ {label}')
        action(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
