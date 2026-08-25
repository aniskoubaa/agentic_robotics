#!/usr/bin/env python3
"""
UAV — minimal camera viewer.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    agr-sim airframe:=x500_mono_cam world:=agr_city
    ros2 run agr_uav_teleop camera_view

Press Q in the window to quit.

Only airframes that declare a camera publish this topic — `x500` does not,
`x500_mono_cam` does. See `ros2 run agr_uav_examples 06_list_airframes`.
"""
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC = '/camera'
WINDOW = 'UAV — Forward Camera'


class CameraView(Node):
    def __init__(self):
        super().__init__('camera_view')
        self.bridge = CvBridge()
        self.create_subscription(Image, TOPIC, self.on_image, 10)
        self.last_t = time.time()
        self.fps = 0.0
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        self.get_logger().info(f'subscribed to {TOPIC} — press Q in the window')

    def on_image(self, msg: Image) -> None:
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        now = time.time()
        dt = now - self.last_t
        self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt) if dt > 0 else self.fps
        self.last_t = now
        cv2.putText(img, f'{self.fps:5.1f} fps', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.imshow(WINDOW, img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.try_shutdown()


def main():
    rclpy.init()
    node = CameraView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
