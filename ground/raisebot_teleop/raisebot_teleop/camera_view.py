#!/usr/bin/env python3
"""
RaiseBot — minimal camera viewer.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

Subscribes to /wrist_camera/image_raw and shows it in an OpenCV window.
Press Q (in the window) to quit, or Ctrl-C in the terminal.

Why write this when `rqt_image_view` exists?  Because students learning
ROS 2 + perception should see — in 30 lines — how to consume an Image
message and convert it to a cv2 numpy array.
"""

import sys
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


TOPIC = '/wrist_camera/image_raw'
WINDOW = 'RaiseBot — Wrist Camera'


class CameraView(Node):
    def __init__(self):
        super().__init__('camera_view')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, TOPIC, self.on_image, 10)
        self.last_t = time.time()
        self.fps = 0.0
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        self.get_logger().info(f'subscribed to {TOPIC}  —  press Q in window to quit')

    def on_image(self, msg: Image) -> None:
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # FPS overlay
        now = time.time()
        dt = now - self.last_t
        self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt) if dt > 0 else self.fps
        self.last_t = now
        cv2.putText(img, f'{self.fps:5.1f} fps', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow(WINDOW, img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.try_shutdown()   # idempotent: bare shutdown() raises if already down


def main():
    rclpy.init()
    node = CameraView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Shutdown race: when SIGTERM lands while the executor is mid
        # wait-set init, rcl raises RCLError instead of the tidy
        # ExternalShutdownException, and the node exits with a traceback that
        # looks like a crash. Which node loses this race varies run to run.
        # If the context is already down we are unwinding anyway — swallow it.
        # If it is still up, this is a genuine fault: re-raise untouched.
        if rclpy.ok():
            raise
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()   # idempotent: bare shutdown() raises if already down


if __name__ == '__main__':
    main()
