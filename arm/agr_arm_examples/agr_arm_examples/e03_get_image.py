#!/usr/bin/env python3
"""
Arm example 03 — grab a frame from each eye and save it.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_arm_examples 03_get_image
    ros2 run agr_arm_examples 03_get_image --ros-args -p camera:=bench
    ros2 run agr_arm_examples 03_get_image --ros-args -p out:=/tmp/shot.png

WHAT  Wait for one Image on the wrist camera (and by default the bench camera
      too), convert each to an OpenCV array, and write PNGs.

LEARN - A ROS Image is NOT a picture file. It is a flat byte array plus
        width, height and an `encoding` string. cv_bridge is what knows how
        to read that layout into a numpy array.
      - ALWAYS pass desired_encoding. Ask for 'bgr8' and cv_bridge converts
        for you; leave it as 'passthrough' and you get whatever the camera
        felt like sending, which is how RGB images end up looking blue.
      - "Wait for exactly one message" has no one-liner: you spin until your
        callback has fired. That is the pattern below and it is worth
        recognising, because you will write it again.
      - Two cameras, and the difference is the whole of manipulation vision.
        The BENCH camera is bolted to the workcell: it always shows the same
        view, so you can find things in it, and the arm regularly stands in
        front of what you wanted to see. The WRIST camera moves with the
        tool: it shows the object you are about to grasp from 20 cm away,
        and shows you nothing useful when the arm is parked elsewhere.
        Look at both PNGs side by side and the trade is obvious.
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPICS = {
    'wrist': '/wrist_camera/image_raw',
    'bench': '/bench_camera/image_raw',
}


def grab(node, bridge, name, topic, out):
    got = {}
    sub = node.create_subscription(Image, topic, lambda m: got.setdefault('m', m), 10)
    print(f'waiting for one frame on {topic} ...')
    for _ in range(300):                       # ~6 s; bench camera runs at 15 Hz
        rclpy.spin_once(node, timeout_sec=0.02)
        if got:
            break
    node.destroy_subscription(sub)
    if not got:
        print(f'  no image on {topic} — is `agr-sim arm` running?')
        return False
    msg = got['m']
    cv2.imwrite(out, bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
    print(f'  {name}: {msg.width}x{msg.height}, encoding {msg.encoding!r} -> {out}')
    return True


def main():
    rclpy.init()
    node = Node('get_image')
    node.declare_parameter('camera', 'both')
    node.declare_parameter('out', '')
    choice = node.get_parameter('camera').value
    out = node.get_parameter('out').value

    wanted = list(TOPICS) if choice == 'both' else [choice]
    if any(c not in TOPICS for c in wanted):
        print(f'unknown camera {choice!r}; use wrist, bench or both')
    else:
        for name in wanted:
            path = out if out else f'arm_{name}.png'
            grab(node, CvBridge(), name, TOPICS[name], path)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
