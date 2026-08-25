#!/usr/bin/env python3
"""
RaiseBot example 03 — grab a camera frame and save it.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run raisebot_examples 03_get_image
    ros2 run raisebot_examples 03_get_image --ros-args -p camera:=ptz

WHAT  Wait for one Image, convert it with cv_bridge, write a PNG.

LEARN - THIS ROBOT HAS TWO CAMERAS and they answer different questions. The
        wrist camera is on the arm, close to whatever it is about to grasp.
        The PTZ camera is on the mast and can be aimed (see 04_aim_the_ptz).
        "The camera" is rarely a well-formed idea on a real robot.
      - A ROS Image is a flat byte array plus width, height and an `encoding`
        string — not a picture file. cv_bridge reads that layout into a numpy
        array.
      - ALWAYS pass desired_encoding='bgr8'. With 'passthrough' you get
        whatever the driver felt like sending, which is how RGB images end up
        looking blue after cv2.imwrite.
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

CAMERAS = {'wrist': '/wrist_camera/image_raw',
           'ptz': '/ptz_camera/image_raw',
           'depth': '/wrist_camera/depth/image_raw'}


def main():
    rclpy.init()
    node = Node('get_image')
    node.declare_parameter('camera', 'wrist')
    node.declare_parameter('out', '')
    which = node.get_parameter('camera').value
    if which not in CAMERAS:
        print(f'unknown camera {which!r}; choose one of {sorted(CAMERAS)}')
        node.destroy_node(); rclpy.try_shutdown(); return
    topic = CAMERAS[which]
    out = node.get_parameter('out').value or f'raisebot_{which}.png'

    bridge = CvBridge()
    got = {}
    node.create_subscription(Image, topic, lambda m: got.setdefault('msg', m), 10)

    print(f'waiting for one frame on {topic} ...')
    for _ in range(250):
        rclpy.spin_once(node, timeout_sec=0.02)
        if got:
            break

    if not got:
        print(f'nothing on {topic} — is `agr-sim ground` running?')
    else:
        msg = got['msg']
        # The depth image is 32-bit float metres, not colour. Asking for bgr8
        # would fail; normalise it to something you can actually look at.
        if which == 'depth':
            import numpy as np
            depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            finite = np.isfinite(depth)
            vis = np.zeros(depth.shape, dtype='uint8')
            if finite.any():
                lo, hi = depth[finite].min(), depth[finite].max()
                if hi > lo:
                    vis[finite] = (255 * (depth[finite] - lo) / (hi - lo)).astype('uint8')
            cv2.imwrite(out, cv2.applyColorMap(vis, cv2.COLORMAP_TURBO))
            print(f'{msg.width}x{msg.height} depth ({msg.encoding}), '
                  f'{lo:.2f}–{hi:.2f} m → {out}')
        else:
            cv2.imwrite(out, bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
            print(f'{msg.width}x{msg.height}, encoding {msg.encoding!r} → {out}')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
