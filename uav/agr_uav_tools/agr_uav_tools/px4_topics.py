# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""Resolve PX4 DDS topic names.

Two things make PX4 topic names harder than they look, and both bite silently:

1. VERSION SUFFIXES. PX4 appends a message-version suffix to some topics and
   not others, and the suffix changes between releases. At PX4 03bf4a5e the
   live names are::

       /fmu/out/vehicle_status_v4          <- suffixed
       /fmu/out/vehicle_local_position_v1  <- suffixed
       /fmu/out/battery_status_v1          <- suffixed
       /fmu/out/vehicle_global_position    <- NOT suffixed
       /fmu/out/failsafe_flags             <- NOT suffixed

   Hard-code either form and the code breaks on the next PX4 bump, with an
   empty subscription rather than an error.

2. NAMESPACES ARE ASYMMETRIC. In multi-vehicle SITL, PX4 gives instance 0 NO
   namespace and instance N the namespace ``px4_N``::

       instance 0 -> /fmu/out/...
       instance 1 -> /px4_1/fmu/out/...

   So single-vehicle code written against the bare path works perfectly, then
   breaks the moment a second vehicle appears. Take the namespace as a
   parameter from the very first node, defaulting to ''.

Resolve at runtime through here and neither problem reaches your node.
"""
from __future__ import annotations

import re
from typing import List, Optional

from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

# ── THE PX4 QoS PROFILE. Use this, never a default subscription. ────────────
#
# RELIABILITY = BEST_EFFORT: PX4 publishes best-effort. A default (RELIABLE)
# subscription is INCOMPATIBLE, so it silently never matches — no error, no
# data, indistinguishable from a dead bridge.
#
# DURABILITY = TRANSIENT_LOCAL: this one is subtler and costs more time.
# PX4 offers TRANSIENT_LOCAL on every /fmu/out topic, and several of them —
# vehicle_status most importantly — publish ONLY ON CHANGE. A VOLATILE
# subscriber says "do not send me anything from before I arrived", so if you
# subscribe to vehicle_status while the vehicle sits idle and DISARMED, the
# state has not changed since boot and you wait forever on a perfectly healthy
# system. Requesting TRANSIENT_LOCAL delivers the last sample on subscribe.
#
# Offered TRANSIENT_LOCAL satisfies a requested VOLATILE, so the two match
# either way — which is exactly why the failure looks like a mystery rather
# than an incompatibility warning.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Topics whose base name we care about, so callers use a stable spelling.
VEHICLE_STATUS = 'vehicle_status'
LOCAL_POSITION = 'vehicle_local_position'
GLOBAL_POSITION = 'vehicle_global_position'
BATTERY_STATUS = 'battery_status'
FAILSAFE_FLAGS = 'failsafe_flags'


def prefix(namespace: str = '') -> str:
    """ROS prefix for a PX4 instance. Empty namespace -> bare '/fmu'."""
    ns = namespace.strip('/')
    return f'/{ns}/fmu' if ns else '/fmu'


def namespace_for_instance(instance: int) -> str:
    """PX4's own rule: instance 0 has no namespace, instance N is 'px4_N'."""
    return '' if instance == 0 else f'px4_{instance}'


def resolve(node, base: str, direction: str = 'out',
            namespace: str = '') -> Optional[str]:
    """Find the live topic for ``base``, with or without a version suffix.

    Returns None when nothing matches — meaning PX4 is not running, or the
    bridge is down. Callers should say that rather than silently waiting.
    """
    want = re.compile(rf'^{re.escape(prefix(namespace))}/{direction}/'
                      rf'{re.escape(base)}(_v\d+)?$')
    for name, _types in node.get_topic_names_and_types():
        if want.match(name):
            return name
    return None


def resolve_all(node, bases: List[str], direction: str = 'out',
                namespace: str = '') -> dict:
    """Resolve several at once; missing ones map to None."""
    return {b: resolve(node, b, direction, namespace) for b in bases}
