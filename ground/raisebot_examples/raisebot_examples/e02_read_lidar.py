#!/usr/bin/env python3
"""
RaiseBot example 02 — what can the LiDAR see?
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run raisebot_examples 02_read_lidar

WHAT  Read one LaserScan and print the nearest obstacle in each direction.

LEARN - A LaserScan is ONE flat array of ranges plus the angles that describe
        it: `angle_min`, `angle_increment`. Beam i points at
        `angle_min + i * angle_increment`. There are no x/y coordinates in the
        message — you compute them, or you stay in beam space as this does.
      - INFINITIES ARE NORMAL. A beam that hits nothing comes back as `inf`
        (and sometimes `nan`). Feed those into a `min()` and you get a robot
        that thinks the world is infinitely far away, or worse, that
        propagates NaN through everything downstream. Filter first — that is
        the single most common LiDAR bug.
      - Ranges outside [range_min, range_max] are meaningless even when they
        are finite numbers.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# (label, from_deg, to_deg) — 0 deg is straight ahead, + is to the left.
SECTORS = [('left',        45, 135),
           ('front-left',  15,  45),
           ('FRONT',      -15,  15),
           ('front-right',-45, -15),
           ('right',     -135, -45),
           ('behind',     135, 180)]


def main():
    rclpy.init()
    node = Node('read_lidar')
    got = {}
    node.create_subscription(LaserScan, '/scan',
                             lambda m: got.setdefault('scan', m), 10)

    print('waiting for /scan ...')
    for _ in range(250):
        rclpy.spin_once(node, timeout_sec=0.02)
        if got:
            break

    if not got:
        print('nothing on /scan — is `agr-sim ground` running?')
        node.destroy_node(); rclpy.try_shutdown(); return

    scan = got['scan']
    print(f'\n{len(scan.ranges)} beams, '
          f'{math.degrees(scan.angle_min):+.0f}° to '
          f'{math.degrees(scan.angle_max):+.0f}°, '
          f'valid range {scan.range_min:.2f}–{scan.range_max:.1f} m\n')

    for label, lo, hi in SECTORS:
        best = math.inf
        for i, r in enumerate(scan.ranges):
            # Reject inf, nan and out-of-spec values BEFORE comparing.
            if not math.isfinite(r) or not (scan.range_min <= r <= scan.range_max):
                continue
            deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            if lo <= deg <= hi:
                best = min(best, r)
        bar = '#' * int(min(best, 10.0) * 4) if math.isfinite(best) else ''
        shown = f'{best:5.2f} m' if math.isfinite(best) else 'nothing'
        print(f'  {label:<12} {shown:>9}  {bar}')

    finite = [r for r in scan.ranges if math.isfinite(r)]
    print(f'\n{len(finite)} of {len(scan.ranges)} beams hit something; '
          f'the rest are inf and MUST be filtered out.')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
