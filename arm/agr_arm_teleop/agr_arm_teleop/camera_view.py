#!/usr/bin/env python3
"""
AGR Arm — camera viewer, either eye.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_arm_teleop camera_view                       # wrist (default)
    ros2 run agr_arm_teleop camera_view --ros-args -p camera:=bench
    ros2 run agr_arm_teleop camera_view --ros-args -p camera:=both

Press Q in the window, or Ctrl-C in the terminal, to quit.

WHY TWO CAMERAS ARE WORTH SHOWING SIDE BY SIDE
    Run `camera:=both`, then drive the arm with teleop_keyboard in another
    terminal, and the entire eye-in-hand versus eye-to-hand argument plays
    out in front of you.

    The BENCH camera never moves. Every block is always in frame, in the same
    place, and you can plan a whole task from it — right up to the moment the
    arm leans over and its own elbow hides the block you were about to pick.

    The WRIST camera sees whatever the gripper is pointed at, close up and
    unoccluded, which is exactly what you need to line up the last two
    centimetres of a grasp. It also sees nothing at all when the arm is
    somewhere else, so you cannot use it to find the block in the first
    place.

    Real manipulation systems carry both for exactly this reason.
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPICS = {
    'wrist': '/wrist_camera/image_raw',
    'bench': '/bench_camera/image_raw',
}
WINDOW = 'AGR Arm — camera'


class CameraView(Node):
    def __init__(self):
        super().__init__('camera_view')
        self.declare_parameter('camera', 'wrist')
        choice = self.get_parameter('camera').value
        self.wanted = list(TOPICS) if choice == 'both' else [choice]
        bad = [c for c in self.wanted if c not in TOPICS]
        if bad:
            raise SystemExit(f'unknown camera {bad}; choose from '
                             f'{", ".join(TOPICS)} or "both"')

        self.bridge = CvBridge()
        self.frames = {}
        self.counts = {c: 0 for c in self.wanted}
        self.t0 = time.time()
        for name in self.wanted:
            self.create_subscription(
                Image, TOPICS[name],
                lambda msg, n=name: self.on_image(n, msg), 10)
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        self.get_logger().info(
            f'showing {", ".join(self.wanted)} — press Q in the window to quit')

    def on_image(self, name: str, msg: Image) -> None:
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.counts[name] += 1
        elapsed = max(time.time() - self.t0, 1e-6)
        cv2.putText(img, f'{name}  {msg.width}x{msg.height}  '
                         f'{self.counts[name] / elapsed:.1f} Hz',
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(img, f'{name}  {msg.width}x{msg.height}  '
                         f'{self.counts[name] / elapsed:.1f} Hz',
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        self.frames[name] = img
        self.show()

    def show(self) -> None:
        ready = [self.frames[n] for n in self.wanted if n in self.frames]
        if not ready:
            return
        if len(ready) > 1:
            # Match heights before stacking; the two cameras are the same
            # size today, but hstack raises rather than letterboxing if a
            # resolution is ever changed in the URDF.
            height = min(f.shape[0] for f in ready)
            ready = [cv2.resize(f, (int(f.shape[1] * height / f.shape[0]), height))
                     for f in ready]
        cv2.imshow(WINDOW, np.hstack(ready) if len(ready) > 1 else ready[0])
        if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
            raise KeyboardInterrupt


def main():
    rclpy.init()
    node = CameraView()
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
