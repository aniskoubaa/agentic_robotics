# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
Thin helpers for talking to Gazebo Harmonic (gz-sim 8) from Python.

We drive the simulator's UserCommands services through the `gz service` CLI
rather than the gz-transport Python bindings, which are not reliably packaged.
On a world that loads the UserCommands system — agr_workcell does — these
exist:

    /world/<world>/create    gz.msgs.EntityFactory -> Boolean   (spawn)
    /world/<world>/set_pose  gz.msgs.Pose          -> Boolean   (teleport)
    /world/<world>/remove    gz.msgs.Entity        -> Boolean   (delete)

Everything is best-effort: a failed call returns False and the caller decides
whether to retry. These shell out once per call, which is fine at the ~15 Hz
grasp_server uses and hopeless for anything faster.

This is a near-copy of raisebot_tools/gz_utils.py, and the duplication is
deliberate. Each platform subtree stands alone — nothing under arm/ imports
from ground/ — so that a student can delete three of the four directories and
still have a working workspace.
"""

from __future__ import annotations

import shutil
import subprocess

DEFAULT_WORLD = 'agr_workcell'


def _gz_available() -> bool:
    return shutil.which('gz') is not None


def _call_service(service: str, reqtype: str, req: str, timeout_ms: int = 2000) -> bool:
    if not _gz_available():
        return False
    cmd = ['gz', 'service', '-s', service,
           '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
           '--timeout', str(timeout_ms), '--req', req]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True,
                             timeout=timeout_ms / 1000.0 + 1.0)
    except (subprocess.TimeoutExpired, OSError):
        return False
    return 'true' in out.stdout.lower()


def spawn_model(name: str, model_uri: str, x: float, y: float, z: float,
                yaw: float = 0.0, world: str = DEFAULT_WORLD) -> bool:
    req = (f'sdf_filename: "{model_uri}", name: "{name}", '
           f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{z: {yaw}}}}}, allow_renaming: true')
    return _call_service(f'/world/{world}/create', 'gz.msgs.EntityFactory', req, 3000)


def set_model_pose(name: str, x: float, y: float, z: float,
                   qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0,
                   world: str = DEFAULT_WORLD) -> bool:
    req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}')
    return _call_service(f'/world/{world}/set_pose', 'gz.msgs.Pose', req, 300)


def remove_model(name: str, world: str = DEFAULT_WORLD) -> bool:
    req = f'name: "{name}", type: MODEL'
    return _call_service(f'/world/{world}/remove', 'gz.msgs.Entity', req, 1000)


# ── World poses via the gz CLI ──────────────────────────────────────────────
# The ros_gz Pose_V -> TFMessage bridge DROPS entity names (child_frame_id
# arrives empty — verified live on both this platform and the ground one), so
# /gz_world_poses cannot tell you WHICH model moved. `gz topic -e -n 1` on the
# same Gazebo topic keeps the names in its protobuf text output, so that is
# what anything name-based has to use.
#
# TOP-LEVEL MODELS are reported in the WORLD frame; links inside a model are
# reported relative to their model. Every graspable block in agr_workcell is
# a top-level model precisely so this is unambiguous.

def get_world_poses(world: str = DEFAULT_WORLD, timeout_s: float = 4.0) -> dict:
    """{entity_name: ((x,y,z), (qx,qy,qz,qw))} from one dynamic_pose message."""
    if not _gz_available():
        return {}
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{world}/dynamic_pose/info', '-n', '1'],
            capture_output=True, text=True, timeout=timeout_s)
    except (subprocess.TimeoutExpired, OSError):
        return {}
    return _parse_pose_v(out.stdout)


def _parse_pose_v(text: str) -> dict:
    """Parse the protobuf text form of a gz.msgs.Pose_V.

    Protobuf text omits zero-valued fields entirely, so every component has
    to default sensibly — and the identity quaternion's w is 1, not 0.
    """
    poses = {}
    name, section, vals = None, None, {}
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith('name:'):
            if name is not None:
                poses[name] = _finish_pose(vals)
            name, section, vals = line.split('"')[1], None, {}
        elif line.startswith('position {'):
            section = 'p'
        elif line.startswith('orientation {'):
            section = 'o'
        elif line == '}':
            section = None
        elif section and ':' in line:
            key, value = line.split(':', 1)
            try:
                vals[f'{section}.{key.strip()}'] = float(value)
            except ValueError:
                pass
    if name is not None:
        poses[name] = _finish_pose(vals)
    return poses


def _finish_pose(vals: dict):
    pos = (vals.get('p.x', 0.0), vals.get('p.y', 0.0), vals.get('p.z', 0.0))
    quat = (vals.get('o.x', 0.0), vals.get('o.y', 0.0),
            vals.get('o.z', 0.0), vals.get('o.w', 1.0))
    return pos, quat


def get_model_world_pose(name: str, world: str = DEFAULT_WORLD):
    return get_world_poses(world).get(name)


# ── Publishing to a Gazebo topic ────────────────────────────────────────────
def publish_empty(topic: str, timeout_s: float = 3.0) -> bool:
    """Send one gz.msgs.Empty to `topic`. Used for DetachableJoint's
    attach/detach, which take no payload — the message merely arriving IS the
    command."""
    if not _gz_available():
        return False
    try:
        subprocess.run(['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Empty', '-p', ''],
                       capture_output=True, text=True, timeout=timeout_s)
    except (subprocess.TimeoutExpired, OSError):
        return False
    return True


# ── Reading a world's declared layout ───────────────────────────────────────
def parse_world_model_poses(sdf_path: str) -> dict:
    """{model_name: (x, y, z)} for every top-level <model> in a world SDF.

    Reads the file as WRITTEN, not as currently simulated — which is the
    point. It answers "where is this object supposed to start", which is what
    you need to put a scene back the way it was.
    """
    import xml.etree.ElementTree as ET
    try:
        root = ET.parse(sdf_path).getroot()
    except (OSError, ET.ParseError):
        return {}
    world = root.find('world')
    if world is None:
        return {}
    out = {}
    for model in world.findall('model'):
        name = model.get('name')
        pose = model.find('pose')
        if name is None or pose is None or not pose.text:
            continue
        parts = pose.text.split()
        if len(parts) >= 3:
            try:
                out[name] = tuple(float(v) for v in parts[:3])
            except ValueError:
                pass
    return out


def rotate_vec(q, v):
    """Rotate vector v=(x,y,z) by quaternion q=(x,y,z,w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx))
