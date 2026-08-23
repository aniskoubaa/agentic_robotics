# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — thin helpers to talk to Gazebo Harmonic (gz-sim 8) from Python.

We drive the sim's UserCommands services through the `gz service` CLI (rather
than the gz-transport python bindings, which aren't reliably installed). These
let us spawn / move / delete models at runtime — the primitives the grasp_server
and the Day-2 auto-demonstrator need.

Standard gz-sim 8 services on a world that loads the UserCommands system
(greenhouse_2026 does — see worlds/greenhouse_2026.sdf):
    /world/<world>/create    gz.msgs.EntityFactory -> gz.msgs.Boolean   (spawn)
    /world/<world>/set_pose  gz.msgs.Pose          -> gz.msgs.Boolean   (teleport)
    /world/<world>/remove    gz.msgs.Entity        -> gz.msgs.Boolean   (delete)

Everything here is best-effort: a failed CLI call returns False and logs, so a
caller can retry rather than crash. NOTE: these shell out per call — fine at the
~15-20 Hz the grasp-follow uses, but not for kHz control.
"""

from __future__ import annotations

import shutil
import subprocess

DEFAULT_WORLD = 'greenhouse_2026'


def _gz_available() -> bool:
    return shutil.which('gz') is not None


def _call_service(service: str, reqtype: str, req: str, timeout_ms: int = 2000) -> bool:
    """Invoke one `gz service` request; return True on a success reply."""
    if not _gz_available():
        return False
    cmd = [
        'gz', 'service', '-s', service,
        '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
        '--timeout', str(timeout_ms), '--req', req,
    ]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True,
                             timeout=timeout_ms / 1000.0 + 1.0)
    except (subprocess.TimeoutExpired, OSError):
        return False
    # A success reply prints "data: true".
    return 'true' in out.stdout.lower()


def spawn_model(name: str, model_uri: str, x: float, y: float, z: float,
                yaw: float = 0.0, world: str = DEFAULT_WORLD) -> bool:
    """Spawn `model_uri` (e.g. 'model://tomato_red') as entity `name` at a pose."""
    # EntityFactory takes either sdf_filename (a model:// URI / path) or raw sdf.
    req = (f'sdf_filename: "{model_uri}", name: "{name}", '
           f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{z: {yaw}}}}}, allow_renaming: true')
    return _call_service(f'/world/{world}/create', 'gz.msgs.EntityFactory', req, 3000)


def set_model_pose(name: str, x: float, y: float, z: float,
                   qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0,
                   world: str = DEFAULT_WORLD) -> bool:
    """Teleport model `name` to a world pose (used to make a grasped fruit follow)."""
    req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}')
    return _call_service(f'/world/{world}/set_pose', 'gz.msgs.Pose', req, 300)


def remove_model(name: str, world: str = DEFAULT_WORLD) -> bool:
    """Delete model `name` from the world (cleanup between episodes)."""
    req = f'name: "{name}", type: MODEL'
    return _call_service(f'/world/{world}/remove', 'gz.msgs.Entity', req, 1000)


# ── World poses via the gz CLI ──────────────────────────────────────────────
# The ros_gz Pose_V→TFMessage bridge DROPS entity names (child_frame_id comes
# through empty — verified live), so we read /world/<w>/dynamic_pose/info with
# `gz topic -e -n 1` instead: its protobuf text output keeps the names.
# NOTE: TOP-LEVEL MODELS (raise2026_robot, spawned tomatoes) are reported in the
# WORLD frame; links inside a model are relative to that model. So callers use
# get_model_world_pose('raise2026_robot') + TF (base_link→gripper) for the tool.

def get_world_poses(world: str = DEFAULT_WORLD, timeout_s: float = 4.0) -> dict:
    """Return {entity_name: ((x,y,z), (qx,qy,qz,qw))} from one dynamic_pose msg."""
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
    """Parse protobuf text of a gz.msgs.Pose_V. Zero-valued fields are omitted."""
    poses = {}
    name, section = None, None
    vals = {}
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith('name:'):
            # flush the previous entry
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
            k, v = line.split(':', 1)
            try:
                vals[f'{section}.{k.strip()}'] = float(v)
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
    """World pose of a TOP-LEVEL model (robot, spawned tomato), or None."""
    return get_world_poses(world).get(name)


def rotate_vec(q, v):
    """Rotate vector v=(x,y,z) by quaternion q=(x,y,z,w). Pure-python helper
    shared by grasp_server and the Day-2 demonstrator/replayer frame math."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx))
