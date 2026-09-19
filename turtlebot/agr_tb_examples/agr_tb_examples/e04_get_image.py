#!/usr/bin/env python3
"""
TurtleBot example 04 — grab a camera frame and save it.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_tb_examples 04_get_image
    ros2 run agr_tb_examples 04_get_image --ros-args -p out:=/tmp/shot.png

WHAT  Wait for one Image, convert it with cv_bridge, write a PNG.

LEARN - A ROS Image is NOT a picture file. It is a flat byte array plus
        width, height and an `encoding` string. cv_bridge is what knows how
        to read that layout into a numpy array.
      - ALWAYS pass desired_encoding. Ask for 'bgr8' and cv_bridge converts
        for you; leave it as 'passthrough' and you get whatever the camera
        sent, which is how RGB images end up looking blue.
      - IMAGE SIZE IS A TRANSPORT PROBLEM, not just a quality setting. The
        Waffle's camera is 1920x1080 rgb8: 6.2 MB per frame, and measured on
        this machine it manages about 3.7 Hz and can take fifteen seconds to
        deliver its FIRST frame over DDS. The TurtleBot 4's OAK-D preview is
        320x240 and arrives instantly. That gap is why `image_raw/compressed`
        exists and why onboard vision runs on small preview streams.
      - NOT EVERY ROBOT HAS ONE. A TurtleBot3 Burger has no camera at all,
        and asking a robot for a sensor it does not have should produce a
        clear sentence rather than a script that hangs forever waiting. The
        registry knows which robots have cameras; this script asks it first.
        (`ros2 run agr_tb_tools list_robots`)
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter

from agr_tb_tools.base import Base


def main():
    rclpy.init()
    node = Node('get_image', parameter_overrides=[Parameter('use_sim_time', value=True)])
    node.declare_parameter('out', '')
    out = node.get_parameter('out').value

    bot = Base(node)
    if not bot.robot.has_camera:
        print(f'\n  {bot.robot.key} has no camera — that is the robot, not a fault.')
        print('  The TurtleBot3 Burger is lidar-only. Try a robot that has one:')
        print('      agr-sim turtlebot model:=tb3_waffle')
        print('      agr-sim turtlebot model:=tb4_lite\n')
        node.destroy_node()
        rclpy.try_shutdown()
        return

    out = out or f'{bot.robot.key}_view.png'
    # Thirty seconds, and progress while it waits. A 6.2 MB frame genuinely
    # takes that long to show up the first time, and a script that sits
    # silent for fifteen seconds looks broken rather than patient.
    print(f'\nwaiting for one frame on {bot.robot.camera} ...')
    waited = 0.0
    while waited < 30.0 and bot.image is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        waited += 0.02
        if abs(waited % 5.0) < 0.02 and waited > 1.0:
            print(f'  still waiting ({waited:.0f} s) ...')

    if bot.image is None:
        print(f'  nothing on {bot.robot.camera} — is `agr-sim turtlebot` running?')
    else:
        msg = bot.image
        img = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(out, img)
        megabytes = len(msg.data) / 1e6
        print(f'  {msg.width}x{msg.height}, encoding {msg.encoding!r} -> {out}')
        print(f'  that frame was {megabytes:.1f} MB of raw bytes, and the first '
              f'one took {waited:.1f} s to arrive.')
        # Be honest about WHY it was slow. Most of the wait on a small stream
        # is DDS discovery — the subscription has to find the publisher and
        # agree terms before a single byte moves — and that cost is the same
        # whatever the frame size. Only a big frame adds transport time on
        # top, so only a big frame gets the lecture about compression.
        print('  Most of that is DDS discovery, which costs the same for a big\n'
              '  frame as a small one and only happens once.')
        if megabytes > 1.0:
            print(f'  This frame is also genuinely large. For anything continuous,\n'
                  f'  use the compressed transport instead:\n'
                  f'      ros2 topic hz {bot.robot.camera}/compressed')
        print()

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
