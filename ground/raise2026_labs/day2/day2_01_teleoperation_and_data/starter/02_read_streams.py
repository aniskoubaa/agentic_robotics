#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — Lab 2.1, Script 02 — watch the streams a VLA learns from.

WHAT  Subscribe to the two signals a VLA consumes — the wrist camera image and
      the robot's joint state — and print them at ~30 Hz so you can SEE the data
      before you record it.

WHY   Script 03 will fuse exactly these streams into a training example. If they
      are not arriving cleanly (right size, no NaNs, both present), the dataset
      will be garbage — and "garbage demos in, garbage policy out". This script
      is your pre-flight check.

LEARN - observation.image  <- /wrist_camera/image_raw  (sensor_msgs/Image)
        observation.state  <- /joint_states            (sensor_msgs/JointState)
      - The VLA sees a 224x224 RGB image; here we just report the raw size so
        you know what 03 has to resize.
      - The "state" is the 6 arm joints + the gripper knuckle, pulled out of
        /joint_states by name (the order matters — it's vla_client's UR5E_JOINTS).

Run while `raise-sim` is up:

    ros2 run raise2026_labs 02_read_streams.py     # or:  02_d3
"""

import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

try:
    import _d2paths                    # installed next to the ros2-run scripts
except ImportError:                    # running straight from the source tree
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    import _d2paths
_d2paths.bootstrap(lerobot=False)
DAY2 = _d2paths.DAY2
from vla_client.base import UR5E_JOINTS, GRIPPER_KNUCKLE   # noqa: E402
from vla_client.ros_image import imgmsg_to_rgb              # noqa: E402  (no cv_bridge)

IMAGE_TOPIC = '/wrist_camera/image_raw'
STATE_TOPIC = '/joint_states'
STATE_JOINTS = UR5E_JOINTS + [GRIPPER_KNUCKLE]   # what we extract, in this order


class StreamWatcher(Node):
    def __init__(self):
        super().__init__('stream_watcher')
        self.last_image = None      # (h, w) of the most recent frame
        self.last_state = None      # dict joint_name -> position
        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, 10)
        self.create_subscription(JointState, STATE_TOPIC, self._on_state, 10)

    def _on_image(self, msg: Image):
        img = imgmsg_to_rgb(msg)          # pure-numpy conversion (no cv_bridge)
        self.last_image = img.shape[:2]   # (height, width)

    def _on_state(self, msg: JointState):
        # /joint_states lists ALL joints; pull out the ones we care about by name.
        self.last_state = dict(zip(msg.name, msg.position))

    def state_vector(self):
        """The 7-vector (6 joints + gripper) in canonical order, or None."""
        if self.last_state is None:
            return None
        try:
            return [self.last_state[j] for j in STATE_JOINTS]
        except KeyError:
            return None   # not all joints seen yet


def main():
    rclpy.init()
    node = StreamWatcher()
    print(f'Watching {IMAGE_TOPIC} + {STATE_TOPIC} at ~30 Hz. Ctrl-C to stop.\n')
    period = 1.0 / 30.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=period)
            vec = node.state_vector()
            img = node.last_image
            img_str = f'{img[1]}x{img[0]}' if img else '— (no frame)'
            if vec is None:
                state_str = '— (waiting for /joint_states)'
                nan_flag = ''
            else:
                state_str = ' '.join(f'{v:+.2f}' for v in vec)
                nan_flag = '  ⚠ NaN!' if any(np.isnan(vec)) else ''
            print(f'\rimage {img_str:>12} | state [{state_str}]{nan_flag}   ', end='', flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print('\nstopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
