#!/usr/bin/env python3
"""
RAISE 2026 — Lab 1, Script 02 — interactive LiDAR explorer.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

WHAT  Show a menu that demonstrates different ways to look at LiDAR data
      from /scan: raw metadata, nearest obstacle, 8-sector summary, and
      a 5-second live stream.

LEARN - sensor_msgs/LaserScan is metadata (angle_min, angle_max, ...) +
        a flat `ranges` array of distances at evenly spaced angles.
      - Index ↔ angle:  angle = angle_min + i × angle_increment
      - Real sensors return `inf` (nothing in range) and sometimes `nan` —
        always filter before doing math.
      - One-shot reads use `spin_once`; a live stream uses `spin` in a
        callback.

Run while `raise-sim` is up:

    ros2 run raise2026_labs 02_read_lidar.py     # or:  02_d1
"""

import math
import time

import rclpy
from rclpy.node import Node

# `LaserScan` = the standard 2-D LiDAR message.
#   ros2 interface show sensor_msgs/msg/LaserScan
from sensor_msgs.msg import LaserScan


TOPIC = '/scan'


class LidarOnce(Node):
    """Tiny helper node that grabs ONE LaserScan and stores it.

    For most menu actions we don't need to keep subscribing — we just
    want the latest scan, do something with it, and return to the menu.
    `latest` starts as None; the callback fills it; the helper below
    spins until it's populated.
    """

    def __init__(self):
        super().__init__('lidar_once')
        self.latest: 'LaserScan | None' = None
        self.create_subscription(LaserScan, TOPIC, self._on_scan, 10)

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest = msg

    def wait_for_scan(self, timeout_s: float = 3.0) -> 'LaserScan | None':
        """Spin until a scan arrives, or until `timeout_s` elapses."""
        self.latest = None
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and self.latest is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest


def show_metadata(msg: LaserScan) -> None:
    """Action 1 — print the structural fields of a LaserScan.

    These describe the SHAPE of the data: how many beams, what angular
    coverage, min/max valid distance. Knowing these is the first step
    before doing anything with `ranges`.
    """
    n = len(msg.ranges)
    print('  ┌── LaserScan metadata ────────────────────────────┐')
    print(f'  │ frame_id          : {msg.header.frame_id:<29s} │')
    print(f'  │ # beams (ranges)  : {n:<29d} │')
    print(f'  │ angle_min         : {math.degrees(msg.angle_min):+8.2f}°   ({msg.angle_min:+.3f} rad)   │')
    print(f'  │ angle_max         : {math.degrees(msg.angle_max):+8.2f}°   ({msg.angle_max:+.3f} rad)   │')
    print(f'  │ angle_increment   : {math.degrees(msg.angle_increment):+8.3f}°/beam              │')
    print(f'  │ range_min         : {msg.range_min:8.2f} m                  │')
    print(f'  │ range_max         : {msg.range_max:8.2f} m                  │')
    print(f'  │ scan period       : {msg.scan_time:8.3f} s                  │')
    print('  └──────────────────────────────────────────────────┘')
    print('  ► angle of beam i  =  angle_min + i × angle_increment')


def show_nearest(msg: LaserScan) -> None:
    """Action 2 — find the closest obstacle and its angle.

    The classic "is something in front of me" check. We zip `ranges`
    with their indices, filter to the valid ones, and take the min.
    """
    valid = [(i, r) for i, r in enumerate(msg.ranges)
             if 0.0 < r < float('inf') and not math.isnan(r)]
    if not valid:
        print('  ⚠ no valid readings (everything is inf/nan)')
        return
    i, r = min(valid, key=lambda ir: ir[1])
    bearing = math.degrees(msg.angle_min + i * msg.angle_increment)
    print(f'  nearest obstacle: {r:.2f} m  at bearing {bearing:+.1f}°  (beam #{i})')


def show_sectors(msg: LaserScan) -> None:
    """Action 3 — bin the scan into 8 sectors of 45° each.

    Common pattern when feeding a LiDAR into an LLM as a tool: a 360-
    element array is too verbose, but 8 cardinal-direction averages
    capture "where is stuff around me".
    """
    sectors = {  # label → list of distances
        'F  (front)':  [], 'FL (front-left)':  [], 'L  (left)':  [], 'BL (back-left)':  [],
        'B  (back)':   [], 'BR (back-right)':  [], 'R  (right)': [], 'FR (front-right)': [],
    }
    labels = list(sectors)
    for i, r in enumerate(msg.ranges):
        if not (0.0 < r < float('inf')):
            continue
        bearing = msg.angle_min + i * msg.angle_increment
        # Map bearing (rad) → 0..7 sector index. +π/8 shifts so that
        # "front" (bearing 0) is centered in its sector.
        idx = int(((bearing + math.pi / 8) % (2 * math.pi)) / (math.pi / 4))
        sectors[labels[idx]].append(r)

    print('  ┌── 8-sector min distance ─────────────────┐')
    for name, vals in sectors.items():
        if vals:
            print(f'  │ {name:<20s}  {min(vals):5.2f} m  ({len(vals):3d} beams) │')
        else:
            print(f'  │ {name:<20s}    --           ({len(vals):3d} beams) │')
    print('  └──────────────────────────────────────────┘')


def stream_nearest(node: 'LidarOnce', seconds: float = 5.0) -> None:
    """Action 4 — print nearest-obstacle distance ~5×/sec for N seconds.

    Demonstrates the "subscribe + react" pattern (vs the one-shot reads
    in actions 1-3). We just keep `spin_once`-ing and inspect the
    latest scan each iteration.
    """
    print(f'  Streaming nearest distance for {seconds:.0f}s — Ctrl-C to interrupt …')
    end = time.monotonic() + seconds
    last_print = 0.0
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if node.latest and now - last_print > 0.4:
                valid = [r for r in node.latest.ranges if 0.0 < r < float('inf')]
                if valid:
                    print(f'  t={end - now:4.1f}s   nearest = {min(valid):5.2f} m')
                last_print = now
    except KeyboardInterrupt:
        print()


ACTIONS = {
    '1': ('show metadata',              show_metadata),
    '2': ('show nearest obstacle',      show_nearest),
    '3': ('show 8-sector summary',      show_sectors),
    '4': ('stream nearest for 5 s',     stream_nearest),
}

MENU = """
┌─── RAISE 2026 · LiDAR menu ─────┐
│   1) show metadata              │
│   2) show nearest obstacle      │
│   3) show 8-sector summary      │
│   4) stream nearest for 5 s     │
│   q) quit                       │
└─────────────────────────────────┘"""


def main():
    rclpy.init()
    node = LidarOnce()

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

        if action is stream_nearest:
            stream_nearest(node)
        else:
            msg = node.wait_for_scan()
            if msg is None:
                print(f'  ✗ no message on {TOPIC} within 3 s — is the sim running?')
                continue
            action(msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
