#!/usr/bin/env python3
"""
Go2 example 03 — grab one camera frame and save it.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_legged_examples 03_get_image
    ros2 run agr_legged_examples 03_get_image --ros-args -p out:=/tmp/go2.png

WHAT  Wait for one Image message on /front_camera, convert it to an OpenCV
      array, and write it to a PNG.

LEARN - A ROS Image is NOT a picture file. It is a flat byte array plus
        width, height and an `encoding` string. cv_bridge is the thing that
        knows how to read that layout into a numpy array.
      - ALWAYS pass desired_encoding. Ask for 'bgr8' and cv_bridge converts
        for you; leave it as 'passthrough' and you get whatever the camera
        felt like sending, which is how RGB images end up looking blue.
      - "Wait for exactly one message" is a genuinely common need and there is
        no one-liner for it — you spin until your callback has fired. That is
        the pattern below, and it is worth recognising.
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC = '/front_camera'


def main():
    rclpy.init()
    node = Node('get_image')
    node.declare_parameter('out', 'go2_frame.png')
    out = node.get_parameter('out').value

    bridge = CvBridge()
    got = {}
    node.create_subscription(Image, TOPIC, lambda m: got.setdefault('msg', m), 10)

    print(f'waiting for one frame on {TOPIC} ...')
    for _ in range(250):                       # ~5 s; the camera runs at 15 Hz
        rclpy.spin_once(node, timeout_sec=0.02)
        if got:
            break

    if not got:
        print(f'no image on {TOPIC} — is `agr-sim legged` running?')
    else:
        msg = got['msg']
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(out, img)
        print(f'{msg.width}x{msg.height}, encoding {msg.encoding!r} → {out}')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
