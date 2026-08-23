#!/usr/bin/env python3
"""
RAISE 2026 — YOLO detector server.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

Exposes one Trigger service per (camera × action) pair:
    /detect_wrist           — single-shot top-3 detections from the wrist cam
    /detect_wrist_annotated — same, plus saves annotated PNG to /tmp
    /detect_wrist_stream    — print top-1 detection every 0.5 s for 5 s
    /detect_ptz             — top-3 from the PTZ mast camera
    /detect_ptz_annotated   — same, plus annotated PNG
    /detect_ptz_stream      — 5 s stream from the PTZ

LIVE VIEWER: an OpenCV window ("RAISE 2026 - YOLO detections") pops up and
shows the annotated frame (bounding boxes + class + confidence) on every
detection. The stream modes animate it. Press ESC/q in the window to close
it (the services keep running). If there's no display it degrades silently
to text + saved PNGs.

Pedagogy:
- First lab that runs a real ML model (`yolov8n` from Ultralytics). The
  service signature is still `std_srvs/Trigger` — same shape as the
  gripper / move_to_pose / navigation tools — so the LLM tool-call
  pattern from D1L2 sees this as just another tool.
- YOLO COCO classes include `bird` — our greenhouse world has chickens
  and ducks, so the detector "sees" something useful out of the box,
  without needing a custom-trained plant model.

Implementation notes:
- Model loads once at startup (~6 MB download on first run, then cached).
- We cache the LATEST camera frame from each topic in /wrist_camera and
  /ptz_camera, then run inference on demand in the service callback.
- MultiThreadedExecutor + ReentrantCallbackGroup so the camera
  subscribers keep firing while a service callback is running (esp.
  important for the stream variants).

Run:
    ros2 run raise2026_tools detector_server
"""

import math
import os
import threading
import time
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


# ── Configuration ──────────────────────────────────────────────────────────
WRIST_TOPIC = '/wrist_camera/image_raw'
PTZ_TOPIC   = '/ptz_camera/image_raw'

# yolov8n is the tiniest YOLOv8 variant (~6 MB, fast on CPU). The first
# call will trigger an automatic download from the Ultralytics CDN.
YOLO_MODEL_NAME = 'yolov8n.pt'

# How many top detections to include in the Trigger response message.
TOP_N = 3

# Stream-mode behaviour.
STREAM_DURATION_S = 5.0
STREAM_PERIOD_S   = 0.5

# Where annotated PNGs land. /tmp is fine for the lab — student opens
# whichever file path the service prints back.
ANNOTATED_DIR = '/tmp'

# Title of the live OpenCV viewer window.
VIEWER_WINDOW = 'RAISE 2026 - YOLO detections'


class DetectorServer(Node):
    def __init__(self):
        super().__init__('detector_server')

        # Reentrant group: camera subscribers + service callbacks share
        # the same executor, but can interleave.
        self._cb_group = ReentrantCallbackGroup()

        self._bridge = CvBridge()

        # Cached latest BGR numpy frames, one per camera. None until first
        # message arrives.
        self._wrist_frame: 'np.ndarray | None' = None
        self._ptz_frame:   'np.ndarray | None' = None

        # Latest annotated frame (with YOLO boxes) for the live viewer.
        # Written by the inference callbacks (worker threads), read by the
        # main-thread cv2 display loop — guarded by a lock.
        self._display_frame: 'np.ndarray | None' = None
        self._display_lock = threading.Lock()

        self.create_subscription(
            Image, WRIST_TOPIC, self._on_wrist, 10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Image, PTZ_TOPIC, self._on_ptz, 10,
            callback_group=self._cb_group,
        )

        # ── Load YOLO once ───────────────────────────────────────────────
        # `from ultralytics import YOLO` is heavy (pulls torch) so we
        # import lazily here, not at module top, to keep ROS 2 imports
        # cheap if someone just inspects the file.
        self.get_logger().info(f'Loading YOLO model "{YOLO_MODEL_NAME}" (may download on first run) …')
        from ultralytics import YOLO
        self._model = YOLO(YOLO_MODEL_NAME)
        # Names is a {class_id: 'name'} dict. We grab it once for fast lookup.
        self._class_names = self._model.names
        self.get_logger().info(f'  ✓ YOLO ready ({len(self._class_names)} COCO classes)')

        # ── Register one service per (camera × mode) ─────────────────────
        for cam in ('wrist', 'ptz'):
            self.create_service(
                Trigger, f'/detect_{cam}',
                self._make_handler(cam, 'now'),
                callback_group=self._cb_group,
            )
            self.create_service(
                Trigger, f'/detect_{cam}_annotated',
                self._make_handler(cam, 'annotated'),
                callback_group=self._cb_group,
            )
            self.create_service(
                Trigger, f'/detect_{cam}_stream',
                self._make_handler(cam, 'stream'),
                callback_group=self._cb_group,
            )

        services = ' '.join(
            f'/detect_{c}{s}' for c in ('wrist', 'ptz') for s in ('', '_annotated', '_stream')
        )
        self.get_logger().info(f'detector services ready: {services}')

    # ── Subscription callbacks just cache the frame ────────────────────────
    def _on_wrist(self, msg: Image) -> None:
        self._wrist_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _on_ptz(self, msg: Image) -> None:
        self._ptz_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _get_frame(self, cam: str) -> 'np.ndarray | None':
        return self._wrist_frame if cam == 'wrist' else self._ptz_frame

    # ── Service handler factory ────────────────────────────────────────────
    def _make_handler(self, cam: str, mode: str):
        def _handler(req: Trigger.Request, resp: Trigger.Response):
            frame = self._get_frame(cam)
            if frame is None:
                resp.success = False
                resp.message = f'no {cam} camera frame yet — is the sim running?'
                return resp

            if mode == 'now':
                return self._do_detect_once(cam, frame, resp)
            elif mode == 'annotated':
                return self._do_detect_annotated(cam, frame, resp)
            elif mode == 'stream':
                return self._do_detect_stream(cam, resp)
            else:
                resp.success = False
                resp.message = f'unknown mode: {mode}'
                return resp

        return _handler

    # ── Modes ──────────────────────────────────────────────────────────────
    def _do_detect_once(self, cam: str, frame: np.ndarray, resp: Trigger.Response):
        """Single-shot top-N detection, no PNG. Updates the live viewer."""
        results, _ = self._infer(frame, cam)
        tops = self._format_top_n(results, TOP_N)
        self.get_logger().info(f'▸ detect_{cam}  → {tops}')
        resp.success = True
        resp.message = f'{cam}: {tops}'
        return resp

    def _do_detect_annotated(self, cam: str, frame: np.ndarray, resp: Trigger.Response):
        """Top-N + write a PNG with YOLO's built-in box renderer."""
        results, annotated = self._infer(frame, cam)
        tops = self._format_top_n(results, TOP_N)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(ANNOTATED_DIR, f'yolo_{cam}_{stamp}.png')
        cv2.imwrite(path, annotated)
        self.get_logger().info(f'▸ detect_{cam}_annotated  → {tops}  ({path})')
        resp.success = True
        resp.message = f'{cam}: {tops}  → {path}'
        return resp

    def _do_detect_stream(self, cam: str, resp: Trigger.Response):
        """Print top-1 detection every STREAM_PERIOD_S, for STREAM_DURATION_S.

        Each sample updates the live viewer, so the window animates during
        the stream — the most visually obvious mode.
        """
        self.get_logger().info(f'▸ detect_{cam}_stream  (every {STREAM_PERIOD_S}s for {STREAM_DURATION_S}s)')
        end = time.monotonic() + STREAM_DURATION_S
        samples = 0
        while time.monotonic() < end and rclpy.ok():
            frame = self._get_frame(cam)
            if frame is None:
                break
            results, _ = self._infer(frame, cam)
            top1 = self._format_top_n(results, 1)
            self.get_logger().info(f'  t+{samples:02d}  {top1}')
            samples += 1
            time.sleep(STREAM_PERIOD_S)
        resp.success = True
        resp.message = f'streamed {samples} {cam} samples over {STREAM_DURATION_S:.0f}s'
        return resp

    # ── Inference helpers ──────────────────────────────────────────────────
    def _infer(self, frame: np.ndarray, cam: str = ''):
        """Run YOLO; stash an annotated frame for the viewer; return both.

        Returns (results, annotated_bgr). `results.plot()` draws the boxes,
        class names and confidences onto a copy of the frame. We label it
        with the camera name and publish it to the live viewer via the
        shared `_display_frame` (read by the main-thread cv2 loop).
        """
        # `verbose=False` suppresses Ultralytics' chatty per-frame log line.
        results = self._model(frame, verbose=False)[0]
        annotated = results.plot()                       # BGR with boxes drawn
        if cam:
            cv2.putText(annotated, f'{cam} camera', (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
        with self._display_lock:
            self._display_frame = annotated
        return results, annotated

    def _format_top_n(self, results, n: int) -> str:
        """Sort detections by confidence desc, return 'cls(conf), cls(conf), …'."""
        if results.boxes is None or len(results.boxes) == 0:
            return 'nothing detected'
        confs = results.boxes.conf.cpu().numpy()
        clss = results.boxes.cls.cpu().numpy().astype(int)
        idx = confs.argsort()[::-1][:n]
        parts = []
        for i in idx:
            name = self._class_names.get(int(clss[i]), f'class{clss[i]}')
            parts.append(f'{name}({confs[i]:.2f})')
        return ', '.join(parts)


def main():
    rclpy.init()
    node = DetectorServer()

    # Spin ROS in a BACKGROUND thread so the main thread is free to run the
    # OpenCV GUI loop. cv2's highgui must be driven from the main thread
    # (imshow + waitKey), and waitKey needs to be called regularly to pump
    # window events — which a sporadic service callback can't do.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Live viewer. The window appears on the first detection (we only
    # imshow once there's an annotated frame). ESC or 'q' closes it; the
    # server keeps running so the menu client still works.
    have_gui = True
    try:
        while rclpy.ok():
            with node._display_lock:
                frame = None if node._display_frame is None else node._display_frame.copy()
            if frame is not None and have_gui:
                try:
                    cv2.imshow(VIEWER_WINDOW, frame)
                except cv2.error:
                    # No display available (headless / no X11). Stop trying;
                    # detection still works, annotated PNGs still save.
                    node.get_logger().warn('no display for the viewer window — '
                                           'detections still run (use option 2 to save PNGs)')
                    have_gui = False
            # waitKey both refreshes the window and throttles this loop to
            # ~30 ms. Returns -1 when no key is pressed.
            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord('q')):   # ESC or q closes the viewer
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
