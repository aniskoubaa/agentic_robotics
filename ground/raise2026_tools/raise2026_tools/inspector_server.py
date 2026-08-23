#!/usr/bin/env python3
"""
RAISE 2026 — VLM inspector server (OpenAI GPT-4o vision).
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

Exposes two Trigger services that send the latest camera frame to a
multimodal LLM and return a short verdict:

    /inspect_plant   — wrist camera + structured plant-health prompt
                       returns: "verdict (healthy / wilted / diseased / unsure):
                                 key indicators; confidence"
    /describe_scene  — PTZ camera + free-form scene description
                       returns: 1-2 sentence natural-language description

Pedagogy:
- Same shape as 05/06/07/08: Trigger in, success+message out.
- But the implementation now calls a remote VLM. From the agent's POV
  it's *the same kind of tool* as YOLO — it's "perception" — just
  with a free-form output instead of a class+conf list.
- Side-by-side with 08 (YOLO), this lab makes the structured vs.
  open-ended trade-off concrete: YOLO is fast/cheap/rigid, the VLM
  is slow/expensive/flexible. Both are valid tools an agent picks.

Requires:
    pip install openai
    export OPENAI_API_KEY=sk-...

Run:
    ros2 run raise2026_tools inspector_server
"""

import base64
import math
import os
import threading
import time

import cv2
import numpy as np


def load_dotenv_into_environ() -> 'str | None':
    """Load KEY=value lines from the workspace `src/.env` into os.environ.

    Lets students keep their API keys in one file
    (raise_ros2_ws/src/.env) instead of editing ~/.bashrc. We only set
    keys that aren't ALREADY in the environment, so a real shell export
    still wins. Returns the path we loaded, or None.

    We find src/.env via COLCON_PREFIX_PATH (which points at the
    workspace `install/` dir once it's sourced) → `../src/.env`. Falls
    back to ./.env in the current directory.
    """
    candidates = []
    for prefix in os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            candidates.append(os.path.join(prefix, '..', 'src', '.env'))
    candidates.append(os.path.join(os.getcwd(), '.env'))

    for path in candidates:
        if not os.path.isfile(path):
            continue
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#') or '=' not in line:
                    continue
                key, val = line.split('=', 1)
                # strip optional surrounding quotes; don't clobber a real export
                os.environ.setdefault(key.strip(),
                                      val.strip().strip('"').strip("'"))
        return os.path.abspath(path)
    return None

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


WRIST_TOPIC = '/wrist_camera/image_raw'
PTZ_TOPIC   = '/ptz_camera/image_raw'

# gpt-4o-mini is ~16× cheaper than gpt-4o on vision, and good enough
# for short structured verdicts. Switch to "gpt-4o" if you want higher
# fidelity on harder questions.
# gpt-4o-mini is fine for free-form scene description (cheap, fast). For the
# structured plant-health verdict the bigger gpt-4o follows the "stem+spheres
# ARE the tomato plant" override much more reliably — picked per-service.
VLM_MODEL_INSPECT = 'gpt-4o'
VLM_MODEL_SCENE   = 'gpt-4o-mini'
VLM_MODEL         = VLM_MODEL_SCENE   # default for the viewer header / logs

# Images are sent as base64-encoded JPEG. JPEG quality 80 keeps the
# payload small (<100 KB for 640×480) without visibly degrading the
# scene. 'low' detail mode in the API uses a fixed 85 tokens per image
# regardless of size — cheap and fast for our use case.
JPEG_QUALITY = 80
# 'low' = 85 tokens per image (cheap, blurry). 'high' = 85 + (tiles × 170)
# tokens (much better at picking out small sphere colours). Picked per-service.
IMAGE_DETAIL_INSPECT = 'high'
IMAGE_DETAIL_SCENE   = 'low'
IMAGE_DETAIL         = IMAGE_DETAIL_SCENE

PLANT_PROMPT = (
    "This is a SIMULATED greenhouse. Tomato plants are stylized: a thin brown "
    "vertical STEM topped with a cluster of COLORED SPHERES. Any stem-and-"
    "spheres object IS a tomato plant — even if it looks tree-like.\n\n"
    "Apply this mapping mechanically based on the DOMINANT sphere colour:\n"
    "  bright GREEN spheres            → Verdict: healthy\n"
    "  TAN / BROWN / YELLOW spheres    → Verdict: wilted\n"
    "  GREEN + YELLOW-GREEN mottled    → Verdict: diseased\n"
    "  NO stem-and-spheres in image    → Verdict: unsure\n\n"
    "Pick exactly one. Do not hedge.\n\n"
    "Output EXACTLY 3 lines (fill in the <…> placeholders with real values, "
    "don't repeat the placeholder text):\n"
    "Verdict: <one of: healthy, wilted, diseased, unsure>\n"
    "Reason: <one short sentence describing the colour and count of plants you saw, e.g. 'All 3 plants had bright green spheres.'>\n"
    "Confidence: <one of: low, medium, high>"
)

SCENE_PROMPT = (
    "You are a small wheeled robot describing what you see in front of you. "
    "In 1-2 short sentences, describe the main objects and whether the path "
    "ahead looks clear to drive forward."
)

# Title of the live viewer window.
VIEWER_WINDOW = 'RAISE 2026 - VLM inspector'

# ── PTZ aim that the inspector uses for /inspect_plant ─────────────────────
# The robot drives in the +1.5 m (or -1.5 m) aisle of the tomato grid. From
# there, the crop rows sit ~1.5 m to the right or left. The PTZ mast camera
# can swing to point straight at them — much more reliable than asking the
# UR5e wrist camera to reach across the aisle (it can't). So /inspect_plant
# commands the PTZ before grabbing its frame.
#   pan = +90°  → look right (south side of the +1.5 m aisle)
#   tilt = +35° → look down at the row
# Pan sign convention (verified by probe):
#   +pi/2 → LEFT side of the robot when facing forward (north row in the +1.5 aisle)
#   -pi/2 → RIGHT side                              (south row)
INSPECT_PAN_LEFT   =  math.pi / 2
INSPECT_PAN_RIGHT  = -math.pi / 2
INSPECT_PAN_RAD    =  INSPECT_PAN_LEFT      # default for back-compat /inspect_plant
INSPECT_TILT_RAD   =  math.radians(35.0)
# PTZ joints are PID-driven and slow. From the +90° pose to the -90° pose
# is a 180° swing; 3 s isn't enough and the next inspection sees empty
# aisle. 5 s is comfortable for any aim change.
INSPECT_SETTLE_S  =  5.0


class InspectorServer(Node):
    def __init__(self):
        super().__init__('inspector_server')

        # ── Set up OpenAI client lazily but fail loudly if no key ──────────
        # First try to populate the env from raise_ros2_ws/src/.env so
        # students can keep keys in one file.
        loaded = load_dotenv_into_environ()
        if loaded:
            self.get_logger().info(f'loaded credentials from {loaded}')
        api_key = os.environ.get('OPENAI_API_KEY')
        if not api_key:
            self.get_logger().error(
                'OPENAI_API_KEY is not set. Either add it to '
                'raise_ros2_ws/src/.env:\n'
                '    OPENAI_API_KEY=sk-...\n'
                'or export it in ~/.bashrc, then re-run.'
            )
            raise SystemExit(1)
        from openai import OpenAI
        self._oai = OpenAI(api_key=api_key)
        self.get_logger().info(f'OpenAI client ready (model = {VLM_MODEL})')

        # ── Reentrant callbacks + cached frames ────────────────────────────
        self._cb_group = ReentrantCallbackGroup()
        self._bridge = CvBridge()
        self._wrist_frame: 'np.ndarray | None' = None
        self._ptz_frame:   'np.ndarray | None' = None

        # ── Live-viewer state ────────────────────────────────────────────
        # The cv2 GUI loop in main() reads these under a lock; the service
        # handlers update them when a call starts / finishes. `pending` flips
        # on while a VLM API call is in flight so we can show "Asking…".
        self._display_lock = threading.Lock()
        self._display_cam   = 'ptz'    # which camera the viewer shows by default
        self._last_verdict  = ''
        self._last_latency  = 0.0
        self._pending       = False

        self.create_subscription(Image, WRIST_TOPIC, self._on_wrist, 10,
                                 callback_group=self._cb_group)
        self.create_subscription(Image, PTZ_TOPIC, self._on_ptz, 10,
                                 callback_group=self._cb_group)

        # PTZ pan/tilt publishers — /inspect_plant aims the camera at the
        # crop row before grabbing a frame.
        self.pub_pan  = self.create_publisher(Float64, '/ptz/pan/cmd',  10)
        self.pub_tilt = self.create_publisher(Float64, '/ptz/tilt/cmd', 10)

        # ── Services ───────────────────────────────────────────────────────
        # /inspect_left   — aim PTZ at the LEFT side of the robot, inspect
        # /inspect_right  — aim PTZ at the RIGHT side, inspect
        # /inspect_plant  — alias of /inspect_left (kept for back-compat)
        # /describe_scene — use the PTZ's current frame at low detail
        for path, pan in (('/inspect_left',  INSPECT_PAN_LEFT),
                          ('/inspect_right', INSPECT_PAN_RIGHT),
                          ('/inspect_plant', INSPECT_PAN_LEFT)):
            self.create_service(
                Trigger, path,
                self._handle_call('ptz', PLANT_PROMPT,
                                  aim_ptz=True,
                                  aim_pan=pan,
                                  model=VLM_MODEL_INSPECT,
                                  detail=IMAGE_DETAIL_INSPECT),
                callback_group=self._cb_group,
            )
        self.create_service(
            Trigger, '/describe_scene',
            self._handle_call('ptz', SCENE_PROMPT,
                              aim_ptz=False,
                              model=VLM_MODEL_SCENE,
                              detail=IMAGE_DETAIL_SCENE),
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            'inspector services ready: /inspect_left /inspect_right '
            '/inspect_plant /describe_scene'
        )

    # ── Subscriptions just cache ──────────────────────────────────────────
    def _on_wrist(self, msg: Image) -> None:
        self._wrist_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _on_ptz(self, msg: Image) -> None:
        self._ptz_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # ── Encoder helper ────────────────────────────────────────────────────
    @staticmethod
    def _frame_to_b64_jpeg(frame: np.ndarray) -> str:
        """Encode a BGR numpy frame as a base64-string JPEG (for the API)."""
        ok, buf = cv2.imencode('.jpg', frame,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            raise RuntimeError('cv2.imencode failed')
        return base64.b64encode(buf.tobytes()).decode('ascii')

    # ── Service handler factory ───────────────────────────────────────────
    def _handle_call(self, cam: str, prompt: str, aim_ptz: bool = False,
                     aim_pan: float = INSPECT_PAN_RAD,
                     model: str = VLM_MODEL_SCENE, detail: str = IMAGE_DETAIL_SCENE):
        def _handler(req: Trigger.Request, resp: Trigger.Response):
            # Flip the viewer to this camera + show "Asking…" while we wait.
            with self._display_lock:
                self._display_cam = cam
                self._pending = True

            # /inspect_plant aims the PTZ at the crop row before grabbing.
            # Without this the camera is wherever the user last left it and
            # the VLM almost always answers "unsure". With it, the camera
            # reliably frames the row 1.5 m to the robot's right.
            if aim_ptz:
                side = 'LEFT' if aim_pan > 0 else 'RIGHT'
                self.pub_pan.publish(Float64(data=aim_pan))
                self.pub_tilt.publish(Float64(data=INSPECT_TILT_RAD))
                self.get_logger().info(
                    f'  aiming PTZ at the {side} crop row '
                    f'(pan={math.degrees(aim_pan):+.0f}°, '
                    f'tilt={math.degrees(INSPECT_TILT_RAD):+.0f}°) '
                    f'and waiting {INSPECT_SETTLE_S:.1f}s for it to settle…'
                )
                time.sleep(INSPECT_SETTLE_S)

            frame = self._wrist_frame if cam == 'wrist' else self._ptz_frame
            if frame is None:
                with self._display_lock:
                    self._pending = False
                resp.success = False
                resp.message = f'no {cam} frame yet — is the sim running?'
                return resp

            try:
                img_b64 = self._frame_to_b64_jpeg(frame)
                t0 = time.time()
                completion = self._oai.chat.completions.create(
                    model=model,
                    messages=[{
                        'role': 'user',
                        'content': [
                            {'type': 'text', 'text': prompt},
                            {
                                'type': 'image_url',
                                'image_url': {
                                    'url': f'data:image/jpeg;base64,{img_b64}',
                                    'detail': detail,
                                },
                            },
                        ],
                    }],
                    max_tokens=200,
                )
                latency_s = time.time() - t0
                verdict = completion.choices[0].message.content.strip()
                with self._display_lock:
                    self._last_verdict = verdict
                    self._last_latency = latency_s
                    self._pending = False
                self.get_logger().info(f'▸ {cam} via {model} ({latency_s:.1f}s)  → {verdict[:80]}')
                resp.success = True
                resp.message = verdict
                return resp
            except Exception as e:
                with self._display_lock:
                    self._last_verdict = f'ERROR: {e}'
                    self._pending = False
                self.get_logger().error(f'VLM call failed: {e}')
                resp.success = False
                resp.message = f'VLM error: {e}'
                return resp

        return _handler

    # ── Live viewer helpers ──────────────────────────────────────────────
    def _get_display_frame(self) -> 'np.ndarray | None':
        """Build the annotated frame the cv2 viewer should show right now."""
        with self._display_lock:
            cam = self._display_cam
            verdict = self._last_verdict
            latency = self._last_latency
            pending = self._pending
        frame = self._wrist_frame if cam == 'wrist' else self._ptz_frame
        if frame is None:
            return None
        img = frame.copy()
        # Camera label, top-left
        cv2.putText(img, f'{cam} camera', (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
        # "Asking…" while a VLM call is in flight (amber)
        if pending:
            cv2.putText(img, 'Asking GPT-4o…', (10, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2, cv2.LINE_AA)
        # Verdict block at the bottom, wrapped, translucent panel
        if verdict:
            self._draw_verdict_panel(img, verdict, latency)
        return img

    @staticmethod
    def _draw_verdict_panel(img: np.ndarray, text: str, latency_s: float) -> None:
        h, w = img.shape[:2]
        lines = []
        for raw in text.splitlines():
            # rough word-wrap: ~10 px per char at scale 0.55 → max chars/line
            max_chars = max(20, (w - 24) // 11)
            words, cur = raw.split(), ''
            for word in words:
                if len(cur) + len(word) + 1 <= max_chars:
                    cur = (cur + ' ' + word).strip()
                else:
                    lines.append(cur); cur = word
            if cur:
                lines.append(cur)
        if not lines:
            return
        line_h = 22
        pad_y = 12
        panel_h = pad_y * 2 + line_h * (len(lines) + 1)   # +1 for the latency line
        # Translucent dark panel at the bottom
        overlay = img.copy()
        cv2.rectangle(overlay, (0, h - panel_h), (w, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.62, img, 0.38, 0, img)
        y = h - panel_h + pad_y + line_h - 4
        for line in lines:
            cv2.putText(img, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 255), 1, cv2.LINE_AA)
            y += line_h
        # Latency footer (cyan)
        if latency_s > 0:
            cv2.putText(img, f'GPT-4o · {latency_s:.1f}s', (12, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 220, 255), 1, cv2.LINE_AA)


def main():
    rclpy.init()
    try:
        node = InspectorServer()
    except SystemExit:
        rclpy.shutdown()
        return

    # ROS spins on a background thread so the main thread can drive the
    # OpenCV viewer (highgui must run on the main thread, with regular
    # waitKey() pumping). Same pattern as detector_server's live viewer.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.get_logger().info(
        f'live viewer: "{VIEWER_WINDOW}" — shows the camera + last VLM verdict. '
        'Press ESC or q in the window to close it (services keep running).'
    )

    have_gui = True
    try:
        while rclpy.ok():
            img = node._get_display_frame()
            if img is not None and have_gui:
                try:
                    cv2.imshow(VIEWER_WINDOW, img)
                except cv2.error:
                    node.get_logger().warn(
                        'no display — viewer disabled (verdicts still return on the service).'
                    )
                    have_gui = False
            key = cv2.waitKey(40) & 0xFF
            if key in (27, ord('q')):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
