#!/usr/bin/env python3
"""
RaiseBot example 04 — aim the PTZ camera.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run raisebot_examples 04_aim_the_ptz
    ros2 run raisebot_examples 04_aim_the_ptz --ros-args -p pan:=0.8 -p tilt:=-0.3

WHAT  Publish a pan and a tilt angle, wait, then save what the camera sees.

LEARN - NOT EVERY COMMAND IS A SERVICE. Pan and tilt are plain Float64 topics
        — one number, fire and forget, no reply. That is the right shape here
        because there is nothing to report: the joint goes where you put it.
        Compare 05_call_a_service, where the answer matters.
      - The units are RADIANS. Every angle in ROS is radians unless the field
        name says otherwise, and mixing degrees in is a bug that looks like a
        badly tuned controller.
      - Moving is not instant, and this is where people lose an afternoon.
        Publishing a setpoint and reading the camera in the same breath
        photographs the OLD view. So don't guess a sleep — WATCH
        /joint_states until the joint is actually where you asked, with a
        timeout for when it never gets there. Verify, don't assume; the
        verification is four lines.
"""
import math

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64

TOLERANCE = 0.02     # rad — close enough to call it aimed
TIMEOUT_S = 5.0


def main():
    rclpy.init()
    node = Node('aim_the_ptz')
    node.declare_parameter('pan', 0.6)      # + is left
    node.declare_parameter('tilt', -0.2)    # - looks down
    node.declare_parameter('out', 'raisebot_ptz_aimed.png')
    pan = float(node.get_parameter('pan').value)
    tilt = float(node.get_parameter('tilt').value)
    out = node.get_parameter('out').value

    pan_pub = node.create_publisher(Float64, '/ptz/pan/cmd', 10)
    tilt_pub = node.create_publisher(Float64, '/ptz/tilt/cmd', 10)

    bridge = CvBridge()
    latest = {}
    joints = {}
    node.create_subscription(Image, '/ptz_camera/image_raw',
                             lambda m: latest.__setitem__('msg', m), 10)
    node.create_subscription(JointState, '/joint_states',
                             lambda m: joints.update(zip(m.name, m.position)), 10)

    # Let discovery finish, or the first publish goes to nobody at all.
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.02)

    print(f'aiming: pan {pan:+.2f} rad ({math.degrees(pan):+.0f}°), '
          f'tilt {tilt:+.2f} rad ({math.degrees(tilt):+.0f}°)')
    pan_pub.publish(Float64(data=pan))
    tilt_pub.publish(Float64(data=tilt))

    print('waiting for the joints to actually get there ...')
    deadline = node.get_clock().now().nanoseconds + int(TIMEOUT_S * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        # Keep republishing: the gz controller holds the last value it heard,
        # and a single message sent before discovery finished reaches nobody.
        pan_pub.publish(Float64(data=pan))
        tilt_pub.publish(Float64(data=tilt))
        rclpy.spin_once(node, timeout_sec=0.02)
        if (abs(joints.get('ptz_pan_joint', 99) - pan) < TOLERANCE
                and abs(joints.get('ptz_tilt_joint', 99) - tilt) < TOLERANCE):
            break

    got_pan = joints.get('ptz_pan_joint')
    got_tilt = joints.get('ptz_tilt_joint')
    if got_pan is None:
        print('no /joint_states — cannot confirm where the camera is pointing')
    elif abs(got_pan - pan) < TOLERANCE and abs(got_tilt - tilt) < TOLERANCE:
        print(f'arrived: pan {got_pan:+.3f}, tilt {got_tilt:+.3f}')
    else:
        print(f'did NOT arrive in {TIMEOUT_S:.0f}s — pan {got_pan:+.3f} '
              f'(asked {pan:+.3f}), tilt {got_tilt:+.3f} (asked {tilt:+.3f})')

    if 'msg' not in latest:
        print('no image on /ptz_camera/image_raw — is `agr-sim ground` running?')
    else:
        msg = latest['msg']
        cv2.imwrite(out, bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        print(f'saved what it is looking at now → {out}')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
