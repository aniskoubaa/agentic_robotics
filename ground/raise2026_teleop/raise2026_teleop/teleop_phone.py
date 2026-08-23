#!/usr/bin/env python3
"""
RAISE 2026 — phone teleop via a tiny Flask web app.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

The page shows a camera feed at the top (switchable between the PTZ mast
camera and the arm-mounted wrist camera), drive buttons in the middle, and
live sliders for linear / angular speed plus PTZ pan & tilt.

Architecture:
    Phone browser ←─ MJPEG stream  ── /stream   ←─ Flask  ←─ /ptz_camera or /wrist_camera
                  ── GET  /cameras ──→ Flask  (which cameras are live + active)
                  ── POST /camera  ──→ Flask  (switch active camera)
                  ── POST /cmd     ──→ Flask  ──→ /cmd_vel
                  ── POST /speed   ──→ Flask  (updates max speeds)
                  ── POST /ptz     ──→ Flask  ──→ /ptz/pan/cmd, /ptz/tilt/cmd
"""

import math
import os
import threading
import time

import cv2
from flask import (Flask, render_template, request, jsonify, Response,
                   send_from_directory, abort)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge


DIRECTIONS = {
    'forward':  ( 1.0,  0.0),
    'backward': (-1.0,  0.0),
    'left':     ( 0.0,  1.0),
    'right':    ( 0.0, -1.0),
    'stop':     ( 0.0,  0.0),
}
MAX_LINEAR    = 2.0
MAX_ANGULAR   = 3.0
PTZ_PAN_MIN, PTZ_PAN_MAX   = -math.pi,    math.pi          # rad
PTZ_TILT_MIN, PTZ_TILT_MAX = -math.pi/2,  math.pi/4        # rad

# Cameras the phone can show. Key = short name used by the UI; value = topic.
CAMERAS = {
    'ptz':   '/ptz_camera/image_raw',     # mast camera — pan/tilt controllable
    'wrist': '/wrist_camera/image_raw',   # arm end-effector camera
}
DEFAULT_CAMERA = 'ptz'
# A camera counts as "streaming" if we got a frame within this many seconds.
STREAM_STALE_S = 2.0

# ── Arm (UR5e) — named poses, published straight to the joint controllers ──
# Same joints/poses as move_to_pose_server; the phone publishes them directly
# so it needs no extra server running. Values are radians in this order:
UR5E_JOINTS = [
    'ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint',
    'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint',
]
ARM_POSES = {
    'home':         [0.0,       -math.pi/2,  math.pi/2,  -math.pi/2,  -math.pi/2, 0.0],
    'above_plant':  [0.0,       -1.0,        0.8,        -1.4,         math.pi/2, 0.0],
    'side_view':    [math.pi/2, -1.0,        1.2,        -math.pi/2,   math.pi/2, 0.0],
    'stow':         [0.0,       -2.5,        2.5,        -2.0,         0.0,       0.0],
}
WRIST_3_JOINT = 'ur5e_wrist_3_joint'      # shared: arm pose sets it; rotate steps it
WRIST_ROTATE_STEP = math.pi / 2           # ±90° per rotate tap

# ── Gripper (Robotiq 2F-85) — same mimic signs as gripper_server ──────────
GRIP_KNUCKLE = 'gripper_robotiq_85_left_knuckle_joint'
GRIP_MIMIC_SIGNS = {
    'gripper_robotiq_85_right_knuckle_joint':       -1.0,
    'gripper_robotiq_85_left_inner_knuckle_joint':  +1.0,
    'gripper_robotiq_85_right_inner_knuckle_joint': -1.0,
    'gripper_robotiq_85_left_finger_tip_joint':     -1.0,
    'gripper_robotiq_85_right_finger_tip_joint':    +1.0,
}
GRIP_OPEN   = 0.0
GRIP_CLOSED = 0.5

# Where phone snapshots are written.
SNAPSHOT_DIR = os.path.expanduser('~/raise2026_snapshots')


class PhoneTeleop(Node):
    def __init__(self):
        super().__init__('teleop_phone')

        self.pub_cmd  = self.create_publisher(Twist,   '/cmd_vel',      10)
        self.pub_pan  = self.create_publisher(Float64, '/ptz/pan/cmd',  10)
        self.pub_tilt = self.create_publisher(Float64, '/ptz/tilt/cmd', 10)

        # Arm + gripper joint-command publishers (direct, no server needed).
        self.pub_arm = {j: self.create_publisher(Float64, f'/{j}/cmd', 10)
                        for j in UR5E_JOINTS}
        self.pub_grip = {GRIP_KNUCKLE: self.create_publisher(Float64, f'/{GRIP_KNUCKLE}/cmd', 10)}
        for j in GRIP_MIMIC_SIGNS:
            self.pub_grip[j] = self.create_publisher(Float64, f'/{j}/cmd', 10)
        # Track the wrist-3 target so each rotate tap is cumulative.
        self.wrist3_target = 0.0

        self.linear  = 0.5
        self.angular = 1.0
        self.pan     = 0.0
        self.tilt    = 0.0

        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        # Per-camera latest JPEG + the wall-clock time of the last frame.
        # ts=0 means "never received a frame".
        self.frames = {name: {'jpeg': None, 'ts': 0.0} for name in CAMERAS}
        self.active_camera = DEFAULT_CAMERA
        # One subscription per camera; the callback factory captures the name.
        for name, topic in CAMERAS.items():
            self.create_subscription(Image, topic, self._make_image_cb(name), 10)

    # ── drive ─────────────────────────────────────────────────────────
    def set_speed(self, linear=None, angular=None):
        if linear  is not None: self.linear  = max(0.0, min(MAX_LINEAR,  float(linear)))
        if angular is not None: self.angular = max(0.0, min(MAX_ANGULAR, float(angular)))

    def send(self, direction: str) -> bool:
        if direction not in DIRECTIONS:
            return False
        lin_f, ang_f = DIRECTIONS[direction]
        msg = Twist()
        msg.linear.x  = lin_f * self.linear
        msg.angular.z = ang_f * self.angular
        self.pub_cmd.publish(msg)
        return True

    # ── ptz ───────────────────────────────────────────────────────────
    def set_ptz(self, pan=None, tilt=None):
        if pan is not None:
            self.pan = max(PTZ_PAN_MIN, min(PTZ_PAN_MAX, float(pan)))
            self.pub_pan.publish(Float64(data=self.pan))
        if tilt is not None:
            self.tilt = max(PTZ_TILT_MIN, min(PTZ_TILT_MAX, float(tilt)))
            self.pub_tilt.publish(Float64(data=self.tilt))

    # ── arm ───────────────────────────────────────────────────────────
    def move_arm(self, pose: str) -> bool:
        """Publish a named UR5e pose straight to the joint controllers."""
        if pose not in ARM_POSES:
            return False
        targets = ARM_POSES[pose]
        for joint, value in zip(UR5E_JOINTS, targets):
            self.pub_arm[joint].publish(Float64(data=value))
        # Keep our wrist-3 tracker in sync so a later rotate is relative.
        self.wrist3_target = targets[UR5E_JOINTS.index(WRIST_3_JOINT)]
        return True

    # ── gripper / wrist ───────────────────────────────────────────────
    def gripper(self, action: str) -> bool:
        """open / close the fingers, or rotate the wrist ±90°."""
        if action in ('open', 'close'):
            target = GRIP_OPEN if action == 'open' else GRIP_CLOSED
            self.pub_grip[GRIP_KNUCKLE].publish(Float64(data=target))
            for joint, sign in GRIP_MIMIC_SIGNS.items():
                self.pub_grip[joint].publish(Float64(data=target * sign))
            return True
        if action in ('rotate_cw', 'rotate_ccw'):
            step = WRIST_ROTATE_STEP if action == 'rotate_cw' else -WRIST_ROTATE_STEP
            self.wrist3_target += step
            # wrap to (-π, π] so many taps don't drift unbounded
            if self.wrist3_target >  math.pi: self.wrist3_target -= 2 * math.pi
            if self.wrist3_target <= -math.pi: self.wrist3_target += 2 * math.pi
            self.pub_arm[WRIST_3_JOINT].publish(Float64(data=self.wrist3_target))
            return True
        return False

    # ── snapshot ──────────────────────────────────────────────────────
    def snapshot(self) -> 'str | None':
        """Write the active camera's current frame to disk; return the path."""
        with self.frame_lock:
            jpg = self.frames[self.active_camera]['jpeg']
        if jpg is None:
            return None
        os.makedirs(SNAPSHOT_DIR, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(SNAPSHOT_DIR, f'{self.active_camera}_{stamp}.jpg')
        with open(path, 'wb') as f:
            f.write(jpg)
        return path

    # ── cameras ───────────────────────────────────────────────────────
    def _make_image_cb(self, name: str):
        """Build an Image callback that caches the JPEG for camera `name`."""
        def cb(msg: Image) -> None:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, jpg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            if ok:
                with self.frame_lock:
                    self.frames[name]['jpeg'] = jpg.tobytes()
                    self.frames[name]['ts'] = time.time()
        return cb

    def set_camera(self, name: str) -> bool:
        if name not in CAMERAS:
            return False
        self.active_camera = name
        return True

    def get_active_jpeg(self) -> bytes | None:
        with self.frame_lock:
            return self.frames[self.active_camera]['jpeg']

    def camera_status(self) -> dict:
        """Per-camera {streaming, active} — drives the UI switch + warnings."""
        now = time.time()
        with self.frame_lock:
            return {
                'active': self.active_camera,
                'cameras': {
                    name: {
                        'streaming': f['jpeg'] is not None and (now - f['ts']) < STREAM_STALE_S,
                    }
                    for name, f in self.frames.items()
                },
            }


def main():
    from ament_index_python.packages import get_package_share_directory

    rclpy.init()
    teleop = PhoneTeleop()
    threading.Thread(target=lambda: rclpy.spin(teleop), daemon=True).start()

    templates_dir = os.path.join(
        get_package_share_directory('raise2026_teleop'), 'templates'
    )
    app = Flask('raise2026_teleop_phone', template_folder=templates_dir)

    @app.route('/')
    def index():
        html = render_template(
            'index.html',
            linear=teleop.linear, angular=teleop.angular,
            pan=teleop.pan, tilt=teleop.tilt,
            max_linear=MAX_LINEAR, max_angular=MAX_ANGULAR,
            pan_min=PTZ_PAN_MIN, pan_max=PTZ_PAN_MAX,
            tilt_min=PTZ_TILT_MIN, tilt_max=PTZ_TILT_MAX,
            active_camera=teleop.active_camera,
        )
        # Don't let the phone browser cache the page — otherwise a UI update
        # (new buttons, fixes) won't show up until a manual hard-refresh.
        resp = Response(html)
        resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate'
        return resp

    @app.route('/cameras')
    def cameras():
        # Polled by the UI to update the switch + show "no signal" warnings.
        return jsonify(teleop.camera_status())

    @app.route('/camera', methods=['POST'])
    def camera():
        name = (request.get_json(silent=True) or {}).get('camera', '')
        ok = teleop.set_camera(name)
        return jsonify(ok=ok, **teleop.camera_status())

    @app.route('/cmd', methods=['POST'])
    def cmd():
        direction = (request.get_json(silent=True) or {}).get('direction', 'stop')
        return jsonify(ok=teleop.send(direction), direction=direction)

    @app.route('/speed', methods=['POST'])
    def speed():
        data = request.get_json(silent=True) or {}
        teleop.set_speed(linear=data.get('linear'), angular=data.get('angular'))
        return jsonify(ok=True, linear=teleop.linear, angular=teleop.angular)

    @app.route('/ptz', methods=['POST'])
    def ptz():
        data = request.get_json(silent=True) or {}
        teleop.set_ptz(pan=data.get('pan'), tilt=data.get('tilt'))
        return jsonify(ok=True, pan=teleop.pan, tilt=teleop.tilt)

    @app.route('/arm', methods=['POST'])
    def arm():
        pose = (request.get_json(silent=True) or {}).get('pose', '')
        return jsonify(ok=teleop.move_arm(pose), pose=pose)

    @app.route('/gripper', methods=['POST'])
    def gripper():
        action = (request.get_json(silent=True) or {}).get('action', '')
        return jsonify(ok=teleop.gripper(action), action=action)

    @app.route('/snapshot', methods=['POST'])
    def snapshot():
        path = teleop.snapshot()
        return jsonify(ok=path is not None, path=path or '')

    # ── snapshot gallery (list / view / delete) ───────────────────────
    @app.route('/gallery')
    def gallery():
        os.makedirs(SNAPSHOT_DIR, exist_ok=True)
        files = [f for f in os.listdir(SNAPSHOT_DIR) if f.lower().endswith('.jpg')]
        # newest first
        files.sort(key=lambda f: os.path.getmtime(os.path.join(SNAPSHOT_DIR, f)),
                   reverse=True)
        return jsonify(photos=files)

    @app.route('/photo/<path:name>')
    def photo(name):
        # basename() strips any path components → no directory traversal.
        safe = os.path.basename(name)
        if not safe.lower().endswith('.jpg') or not os.path.isfile(os.path.join(SNAPSHOT_DIR, safe)):
            abort(404)
        return send_from_directory(SNAPSHOT_DIR, safe, mimetype='image/jpeg')

    @app.route('/photo_delete', methods=['POST'])
    def photo_delete():
        name = os.path.basename((request.get_json(silent=True) or {}).get('name', ''))
        path = os.path.join(SNAPSHOT_DIR, name)
        if name.lower().endswith('.jpg') and os.path.isfile(path):
            os.remove(path)
            return jsonify(ok=True, name=name)
        return jsonify(ok=False, name=name)

    @app.route('/stream')
    def stream():
        # Serves whichever camera is currently active. Switching the active
        # camera (POST /camera) changes the feed mid-stream — the same
        # <img src="/stream"> keeps working, no reload needed.
        def gen():
            while True:
                jpg = teleop.get_active_jpeg()
                if jpg is None:
                    time.sleep(0.05); continue
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + jpg + b'\r\n')
                time.sleep(0.1)
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

    teleop.get_logger().info('Phone teleop ready: http://<laptop-ip>:5000/')
    app.run(host='0.0.0.0', port=5000, threaded=True)


if __name__ == '__main__':
    main()
