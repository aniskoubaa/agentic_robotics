#!/usr/bin/env python3
"""
Go2 — minimal camera viewer.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

Subscribes to /front_camera and shows it in an OpenCV window.
Press Q in the window to quit, or Ctrl-C in the terminal.

    ros2 run agr_legged_teleop camera_view

The camera is on the head link, pitched 10 deg down, because a walking robot
wants to see where its feet are going rather than the horizon. Drive with
teleop_keyboard in another terminal and watch the view bob with the gait —
that bob is the single clearest picture of why legged perception is harder
than wheeled perception.
"""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC = '/front_camera'
WINDOW = 'Go2 — Front Camera'


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
        cv2.putText(img, f'{self.fps:5.1f} fps', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
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
        # Shutdown race: SIGTERM landing mid wait-set init surfaces as RCLError
        # rather than ExternalShutdownException. If the context is already down
        # we are unwinding anyway; if it is up, this is real — re-raise.
        if rclpy.ok():
            raise
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
