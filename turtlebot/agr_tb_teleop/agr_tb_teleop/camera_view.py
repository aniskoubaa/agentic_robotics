#!/usr/bin/env python3
"""
TurtleBot — camera viewer.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_tb_teleop camera_view

Press Q in the window, or Ctrl-C in the terminal, to quit.

The topic comes from the registry, so this works unchanged on a TurtleBot3
Waffle (/camera/image_raw) and a TurtleBot 4 (the OAK-D's
/oakd/rgb/preview/image_raw). On a Burger it says so and exits rather than
waiting forever for a camera that does not exist.

Drive with teleop_keyboard in another terminal. The OAK-D preview stream is
320x240 — small on purpose, because it is the stream meant for onboard
inference rather than for looking at.
"""
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from agr_tb_tools import registry


class CameraView(Node):
    def __init__(self):
        super().__init__('camera_view',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        import os
        self.robot = registry.get(os.environ.get('AGR_TB_MODEL') or registry.DEFAULT)
        if not self.robot.has_camera:
            raise SystemExit(
                f'\n  {self.robot.key} has no camera — that is the robot, not a '
                f'fault.\n  Try:  agr-sim turtlebot model:=tb3_waffle\n')
        self.bridge = CvBridge()
        self.count = 0
        self.t0 = time.time()
        self.window = f'{self.robot.key} — {self.robot.camera}'
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        self.create_subscription(Image, self.robot.camera, self.on_image, 10)
        self.get_logger().info(
            f'showing {self.robot.camera} — press Q in the window to quit')

    def on_image(self, msg: Image) -> None:
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.count += 1
        hz = self.count / max(time.time() - self.t0, 1e-6)
        label = f'{msg.width}x{msg.height}  {hz:.1f} Hz'
        for colour, thick in (((0, 0, 0), 3), ((255, 255, 255), 1)):
            cv2.putText(img, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, colour, thick)
        cv2.imshow(self.window, img)
        if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
            raise KeyboardInterrupt


def main():
    rclpy.init()
    try:
        node = CameraView()
    except SystemExit as exc:
        print(exc)
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
