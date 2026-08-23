# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — vla_client: a backend-blind Vision-Language-Action client.

    from vla_client import make_vla_client, Action
    client = make_vla_client()              # default = in-process SmolVLA
    action = client.act(img, "pick the ripe tomato", state)
    for joint, target in action.joint_commands():
        publish(f"/{joint}/cmd", target)    # 6 arm joints + gripper (+ mimics)

Only base.py is imported eagerly (pure-python, no torch). The heavy backends
(local_smolvla, remote_vla) import their dependencies lazily, so this package
imports fine on a machine with no GPU/torch.
"""

from .base import (
    Action,
    ExecutionResult,
    VLAClient,
    UR5E_JOINTS,
    JOINT_LIMITS,
    GRIPPER_KNUCKLE,
    GRIPPER_MIMIC_SIGNS,
    GRIPPER_OPEN,
    GRIPPER_CLOSED,
)
from .factory import make_vla_client

__all__ = [
    'Action',
    'ExecutionResult',
    'VLAClient',
    'UR5E_JOINTS',
    'JOINT_LIMITS',
    'GRIPPER_KNUCKLE',
    'GRIPPER_MIMIC_SIGNS',
    'GRIPPER_OPEN',
    'GRIPPER_CLOSED',
    'make_vla_client',
]
