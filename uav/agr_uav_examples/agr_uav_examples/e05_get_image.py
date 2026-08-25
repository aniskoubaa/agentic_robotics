#!/usr/bin/env python3
"""
UAV example 05 — grab a frame from the drone's camera.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    agr-sim airframe:=x500_mono_cam          # the plain x500 has NO camera
    ros2 run agr_uav_examples 05_get_image

The default PX4 world is bare ground, so the first frame is mostly horizon.
For something worth looking at:

    agr-sim airframe:=x500_mono_cam world:=agr_city

WHAT  Wait for one Image on /camera, convert it, save a PNG.

LEARN - NOT EVERY AIRFRAME HAS A CAMERA. `x500` does not; `x500_mono_cam`
        does. Which sensors exist is a property of the platform, declared in
        airframes.yaml, and code that assumes a camera will fail on three of
        the four airframes here.
      - The image does NOT come over the PX4 link. PX4 telemetry crosses
        XRCE-DDS; the camera is a Gazebo sensor and crosses a completely
        separate ros_gz_bridge. Two vehicles, two transports, one robot — and
        when one is broken the other keeps working, which is confusing until
        you know they are independent.
      - The Gazebo topic name embeds the world AND the vehicle instance:
        /world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image
        The launch file builds it and remaps it to plain /camera, because a
        topic name nobody can type is a topic nobody uses.
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC = '/camera'


def main():
    rclpy.init()
    node = Node('get_image')
    node.declare_parameter('out', 'uav_frame.png')
    out = node.get_parameter('out').value

    bridge = CvBridge()
    got = {}
    node.create_subscription(Image, TOPIC, lambda m: got.setdefault('msg', m), 10)

    print(f'waiting for one frame on {TOPIC} ...')
    for _ in range(300):                      # ~6 s; the bridge runs ~10 Hz
        rclpy.spin_once(node, timeout_sec=0.02)
        if got:
            break

    if not got:
        print(f'nothing on {TOPIC}.\n'
              '  the plain x500 has no camera — relaunch with:\n'
              '      agr-stop && agr-sim airframe:=x500_mono_cam')
    else:
        msg = got['msg']
        cv2.imwrite(out, bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        print(f'{msg.width}x{msg.height}, encoding {msg.encoding!r} → {out}')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
