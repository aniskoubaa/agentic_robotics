# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — pure-numpy ROS Image conversion (no cv_bridge).

WHY  cv_bridge is a C extension compiled against numpy 1.x (ROS Jazzy apt),
     but LeRobot >= 0.5 requires numpy >= 2 — the two cannot share a process.
     For the encodings our sim uses, cv_bridge is overkill anyway: an `rgb8`
     Image message is literally height*width*3 bytes. So the whole Day-2
     recording/execution stack converts with plain numpy and works under BOTH
     numpy generations (system python 1.26 and the lerobot venv's 2.x).

     Resizing is nearest-neighbour via numpy indexing — no OpenCV needed, and
     deterministic across environments.
"""

from __future__ import annotations

import numpy as np


def imgmsg_to_rgb(msg) -> np.ndarray:
    """sensor_msgs/Image (rgb8/bgr8) -> HxWx3 uint8 RGB array. Pure numpy.

    Handles row padding via msg.step (bytes per row can exceed width*3).
    """
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported encoding "{msg.encoding}" (expected rgb8/bgr8)')
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    arr = arr.reshape(msg.height, msg.step)[:, : msg.width * 3]
    img = arr.reshape(msg.height, msg.width, 3)
    if msg.encoding == 'bgr8':
        img = img[:, :, ::-1]
    return img


def resize_rgb(img: np.ndarray, size: int = 224) -> np.ndarray:
    """Nearest-neighbour resize to size x size. Pure numpy (no cv2)."""
    h, w = img.shape[:2]
    iy = np.linspace(0, h - 1, size).round().astype(int)
    ix = np.linspace(0, w - 1, size).round().astype(int)
    return np.ascontiguousarray(img[np.ix_(iy, ix)])
