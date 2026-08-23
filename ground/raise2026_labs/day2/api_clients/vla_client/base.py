# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — VLA client: the canonical action contract.

This module is the SINGLE SOURCE OF TRUTH for "what a VLA is allowed to say"
in Day 2. Every backend (local SmolVLA, remote HTTP) returns the same `Action`,
and the Lab 2.2 executor turns that `Action` into ROS 2 commands using ONLY the
constants defined here. Nothing downstream has to know which model produced it.

Why a canonical contract at all?
    The interface unifies trivially, but the *action spaces* of different VLAs do
    NOT (SmolVLA emits UR5e joint targets; OpenVLA emits 7-DoF end-effector
    deltas). If each backend spoke its own language the executor would need a
    different adapter per model. Instead every backend converts to ONE contract,
    anchored on the UR5e command space the Day-1 sim actually exposes.

The contract (verified against raise2026_bringup/config/ros_gz_bridge.yaml and
the Day-1 tool servers):
    Action.joints   -> 6 floats, radians, in UR5E_JOINTS order
    Action.gripper  -> 1 float, 0.0 (open) .. 0.5 (closed)
    Action.base     -> optional {linear_x, angular_z} for Task B (Husky drive)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Protocol, runtime_checkable


# ─── The UR5e command space (verified against the sim) ──────────────────────
# These joint names + topics match move_to_pose_server.py and the ros_gz_bridge.
# The VLA's `joints` vector is ALWAYS in this exact order.
UR5E_JOINTS = [
    'ur5e_shoulder_pan_joint',
    'ur5e_shoulder_lift_joint',
    'ur5e_elbow_joint',
    'ur5e_wrist_1_joint',
    'ur5e_wrist_2_joint',
    'ur5e_wrist_3_joint',
]

# Conservative UR5e joint limits (radians). We CLAMP every action to these so a
# half-trained policy can never command a wild target that explodes the sim.
JOINT_LIMITS = {
    'ur5e_shoulder_pan_joint':  (-6.28, 6.28),
    'ur5e_shoulder_lift_joint': (-3.14, 3.14),
    'ur5e_elbow_joint':         (-3.14, 3.14),
    'ur5e_wrist_1_joint':       (-6.28, 6.28),
    'ur5e_wrist_2_joint':       (-6.28, 6.28),
    'ur5e_wrist_3_joint':       (-6.28, 6.28),
}

# ─── The gripper (Robotiq 2F-85) — driver joint + mimic siblings ────────────
# DART (Gazebo Harmonic's physics engine) does NOT enforce URDF mimic joints, so
# moving the gripper means publishing the driving knuckle AND every mimic joint
# with its correct sign. These values are copied from gripper_server.py — keep
# them in sync. See memory [[feedback-dart-mimic-fix]].
GRIPPER_KNUCKLE = 'gripper_robotiq_85_left_knuckle_joint'
GRIPPER_MIMIC_SIGNS = {
    'gripper_robotiq_85_right_knuckle_joint':       -1.0,
    'gripper_robotiq_85_left_inner_knuckle_joint':  +1.0,
    'gripper_robotiq_85_right_inner_knuckle_joint': -1.0,
    'gripper_robotiq_85_left_finger_tip_joint':     -1.0,
    'gripper_robotiq_85_right_finger_tip_joint':    +1.0,
}
GRIPPER_OPEN = 0.0     # fully open (0 rad on the driving knuckle)
GRIPPER_CLOSED = 0.5   # roughly closed (within the 0..0.8 joint limit)


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


@dataclass
class Action:
    """One canonical robot command — what every VLA backend must return.

    `joints`  : 6 absolute joint-position targets (radians), UR5E_JOINTS order.
    `gripper` : driving-knuckle target, clamped to [GRIPPER_OPEN, GRIPPER_CLOSED].
    `base`    : optional Husky drive (Task B). None for arm-only tasks.
    """

    joints: list[float]
    gripper: float = GRIPPER_OPEN
    base: Optional[dict] = None   # {"linear_x": float, "angular_z": float}

    def __post_init__(self) -> None:
        if len(self.joints) != 6:
            raise ValueError(f'Action.joints must have 6 values, got {len(self.joints)}')

    def clamped(self) -> 'Action':
        """Return a safety-clamped copy (joint limits + gripper range)."""
        j = [_clamp(v, *JOINT_LIMITS[name]) for name, v in zip(UR5E_JOINTS, self.joints)]
        g = _clamp(self.gripper, GRIPPER_OPEN, GRIPPER_CLOSED)
        return Action(joints=j, gripper=g, base=self.base)

    # ── Serialization for the HTTP backend (remote_vla.py / server.py) ──────
    def to_dict(self) -> dict:
        d = {'joints': list(self.joints), 'gripper': float(self.gripper)}
        if self.base is not None:
            d['base'] = self.base
        return d

    @classmethod
    def from_dict(cls, d: dict) -> 'Action':
        return cls(joints=list(d['joints']),
                   gripper=float(d.get('gripper', GRIPPER_OPEN)),
                   base=d.get('base'))

    # ── The bridge to the real sim — the ONLY place mimic expansion lives ───
    def joint_commands(self) -> list[tuple[str, float]]:
        """Expand this action into (joint_name, target) pairs ready to publish.

        Returns the 6 arm joints, then the gripper driving knuckle, then every
        mimic sibling with its sign applied. The executor publishes each pair to
        `/{joint_name}/cmd` (std_msgs/Float64) — exactly the topics the bridge
        exposes. Centralizing the mimic math here means the executor stays a
        dumb publisher and there is one place to fix if the gripper changes.
        """
        a = self.clamped()
        cmds = list(zip(UR5E_JOINTS, a.joints))
        cmds.append((GRIPPER_KNUCKLE, a.gripper))
        cmds.extend((j, sign * a.gripper) for j, sign in GRIPPER_MIMIC_SIGNS.items())
        return cmds


@runtime_checkable
class VLAClient(Protocol):
    """The backend-blind interface the Lab 2.2 executor depends on.

    Both LocalSmolVLA and RemoteVLA implement this. The executor never imports
    a concrete backend — it asks make_vla_client(...) for "something that acts".
    """

    def act(self, image, instruction: str, state=None) -> Action:
        """Map (wrist image, language instruction, optional joint state) -> Action."""
        ...


@dataclass
class ExecutionResult:
    """What one vla_execute(...) rollout reports back (used by Lab 2.2 + grading)."""

    status: str                       # "success" | "timeout" | "error"
    steps: int = 0
    mean_latency_ms: float = 0.0
    max_latency_ms: float = 0.0
    note: str = ''
    latencies_ms: list = field(default_factory=list)

    @property
    def success(self) -> bool:
        return self.status == 'success'
