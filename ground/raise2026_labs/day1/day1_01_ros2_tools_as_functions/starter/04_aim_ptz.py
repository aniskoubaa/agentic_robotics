#!/usr/bin/env python3
"""
RAISE 2026 — Lab 1, Script 04 — interactive PTZ aim menu.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

WHAT  Show a menu of camera poses and publish std_msgs/Float64 targets
      to /ptz/pan/cmd + /ptz/tilt/cmd to move the camera there.

LEARN - Two publishers, same message type, different topics.
      - Joints in ROS 2 take angles in RADIANS (not degrees). We use a
        small `deg()` helper so the menu is readable.
      - Position controllers in Gazebo apply the target via PID, so the
        joint takes ~1-2 s to physically settle. The "scan sweep"
        action below shows how to step smoothly instead of jumping.

Watch the result in the phone teleop feed (http://<host>:5000) or run:

    ros2 run rqt_image_view rqt_image_view /ptz_camera/image_raw

Run while `raise-sim` is up:

    ros2 run raise2026_labs 04_aim_ptz.py        # or:  04_d1
"""

import math
import time

import rclpy
from rclpy.node import Node

# `Float64` is the simplest non-trivial message in ROS 2 — just one
# `data` field. The PTZ JointPositionController plugin in Gazebo
# subscribes to it directly.
from std_msgs.msg import Float64


def deg(d: float) -> float:
    """Degrees → radians. Robotics math uses radians everywhere."""
    return d * math.pi / 180.0


def aim(pan_pub, tilt_pub, pan_rad: float, tilt_rad: float, hold_s: float = 2.0) -> None:
    """Publish a (pan, tilt) target and wait for the joint to settle.

    The Gazebo position controller is PID-based, so commanding a new
    target is INSTANT but the physical joint takes time to get there.
    `hold_s` is how long we wait before returning to the menu — too
    short and the next command interrupts before this one finishes.
    """
    pan_pub.publish(Float64(data=pan_rad))
    tilt_pub.publish(Float64(data=tilt_rad))
    print(f'  pan={math.degrees(pan_rad):+6.1f}°  tilt={math.degrees(tilt_rad):+6.1f}°  '
          f'(hold {hold_s:.1f}s)')
    time.sleep(hold_s)


def scan_sweep(pan_pub, tilt_pub) -> None:
    """Action 6 — smooth pan sweep from -60° to +60° and back.

    Shows the OTHER way to use a position controller: step the target
    in small increments instead of jumping. The camera tracks smoothly
    because each step is small enough that the PID converges before
    the next command arrives.
    """
    print('  Sweeping pan -60° → +60° → -60° (smooth) …')
    # Reset tilt to level once at the start so we sweep horizontally.
    tilt_pub.publish(Float64(data=0.0))
    time.sleep(0.5)
    # math.linspace would be nicer but stdlib doesn't have one, so we
    # use a hand-rolled range.
    step_deg = 5
    sweep = list(range(-60,  61, step_deg)) + list(range( 60, -61, -step_deg))
    for d in sweep:
        pan_pub.publish(Float64(data=deg(d)))
        time.sleep(0.10)            # 100 ms per 5° step  →  50°/s
    pan_pub.publish(Float64(data=0.0))
    time.sleep(0.5)


ACTIONS = {
    '1': ('center  (pan 0,  tilt 0)',       lambda pp, tp: aim(pp, tp, deg(  0), deg(  0))),
    '2': ('look left  (pan +45°)',          lambda pp, tp: aim(pp, tp, deg( 45), deg(  0))),
    '3': ('look right (pan -45°)',          lambda pp, tp: aim(pp, tp, deg(-45), deg(  0))),
    '4': ('look up    (tilt -15°)',         lambda pp, tp: aim(pp, tp, deg(  0), deg(-15))),
    '5': ('look down  (tilt +30°)',         lambda pp, tp: aim(pp, tp, deg(  0), deg( 30))),
    '6': ('smooth scan sweep -60° → +60°',  scan_sweep),
}

MENU = """
┌─── RAISE 2026 · PTZ aim menu ────┐
│   1) center  (pan 0,  tilt 0)    │
│   2) look left  (pan +45°)       │
│   3) look right (pan -45°)       │
│   4) look up    (tilt -15°)      │
│   5) look down  (tilt +30°)      │
│   6) smooth scan sweep ±60°      │
│   q) quit                        │
└──────────────────────────────────┘"""


def main():
    rclpy.init()
    node = Node('aim_ptz_menu')

    pan_pub  = node.create_publisher(Float64, '/ptz/pan/cmd',  10)
    tilt_pub = node.create_publisher(Float64, '/ptz/tilt/cmd', 10)

    # Let DDS discover the bridge before we start publishing.
    time.sleep(0.4)

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
        action(pan_pub, tilt_pub)
        node.get_logger().info('  ✓ done')

    # Return the camera to a neutral pose on exit (nice for the next run).
    pan_pub.publish(Float64(data=0.0))
    tilt_pub.publish(Float64(data=0.0))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
