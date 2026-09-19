#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
Arm — one object that knows how to move the UR5e and work the gripper.

The same shape as agr_uav_tools.offboard.Pilot: it WRAPS a Node rather than
subclassing one, so a script can own its node, add its own subscriptions, and
still hand the arm around. Everything blocking here spins that node itself,
which is why none of the examples need an executor or a thread.

    import rclpy
    from rclpy.node import Node
    from agr_arm_tools.arm import Arm

    rclpy.init()
    node = Node('my_script')
    arm = Arm(node)
    arm.wait_for_state()

    arm.open_gripper()
    arm.move_to(0.45, 0.0, 0.90)      # hover above the green block
    arm.move_to(0.45, 0.0, 0.78)      # descend onto it
    arm.close_gripper()
    arm.move_to(0.45, 0.0, 0.95)      # lift

WHAT IS AND IS NOT HERE
    There is no trajectory planning and no collision checking. Every move is
    "solve IK, publish six numbers, wait until the joints stop moving", and
    the arm takes whatever path the PID controllers take between the two
    configurations. For a bench with three blocks on it that is honest and
    enough. It is also why move_to() is usually called twice per pick —
    hover, then descend — because the straight line between two IK solutions
    is a line in JOINT space, and in Cartesian space it bulges.
"""
from __future__ import annotations

import math
import time
from collections import deque

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from agr_arm_tools import kinematics as K

# ── Gripper joints and their signs ──────────────────────────────────────────
# The Robotiq 2F-85 URDF drives five joints off the left knuckle with <mimic>
# tags. DART does not implement mimic constraints, so every joint is
# commanded individually with the sign the URDF's mimic multiplier implies.
# Get one sign wrong and the fingers tear themselves apart rather than close.
GRIPPER_SIGNS = {
    'gripper_robotiq_85_left_knuckle_joint':        +1.0,
    'gripper_robotiq_85_right_knuckle_joint':       -1.0,
    'gripper_robotiq_85_left_inner_knuckle_joint':  +1.0,
    'gripper_robotiq_85_right_inner_knuckle_joint': -1.0,
    'gripper_robotiq_85_left_finger_tip_joint':     -1.0,
    'gripper_robotiq_85_right_finger_tip_joint':    +1.0,
}
KNUCKLE = 'gripper_robotiq_85_left_knuckle_joint'      # the driving joint

# Knuckle angle for fully open and for closed. OPEN is 0 by definition.
#
# CLOSED is 0.60 because that is where the pads meet a 50 mm cube, and NOT a
# larger number that would squeeze it. The reason is specific to how a grasp
# works here: grasp_server welds the block to the wrist with a gz
# DetachableJoint, and once welded the block is part of the same articulated
# body as the fingers, so the physics engine stops checking collision between
# them. The fingers would sail straight through a block they are supposedly
# gripping. Stopping them at its surface is what makes the picture match what
# is actually happening.
#
# It also means "the fingers stalled" is no longer available as a way to tell
# whether anything was picked up — see holding_something, which asks
# grasp_server instead.
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.60

# Finger-tip separation as a function of knuckle angle, fitted to four
# measured points (0.003 rad -> 135.3 mm, 0.174 -> 119.3, 0.424 -> 93.2,
# 0.624 -> 70.8). Residual under 1.5 mm across the range.
# NOTE this is the gap between the finger-tip LINK ORIGINS; the rubber pads
# sit inboard of those, so the free space for an object is roughly 24 mm less.
TIP_SEPARATION_MM_AT_ZERO = 135.9
TIP_SEPARATION_MM_PER_RAD = -104.2

GRIP_MIN_TIME = 0.4      # s before any "stopped" reading is believed at all
GRIP_ARRIVED = 0.05      # rad from the command that counts as having arrived
# The gripper creeps as it closes — the knuckle can advance at under
# 0.01 rad/s for seconds at a time while it drags four damped mimic joints.
# A short stall window mistakes that creep for a stall and hands back a
# gripper that is still closing: measured, a 0.5 s window returned at knuckle
# 0.37 on a grip that went on to reach 0.57.
GRIP_STALL_TIME = 1.0    # s of no knuckle movement that counts as a stall
GRIP_DRIFT = 0.003       # rad the knuckle may wander and still count as still

# ── When is a move finished? ────────────────────────────────────────────────
# Two conditions: close enough to the target, AND not moving any more. The
# position tolerance is deliberately loose, because the second condition is
# what makes the answer trustworthy — during a move the joints run at
# ~0.5 rad/s, so crossing 0.02 rad early does not end the wait.
#
# "Not moving" is measured as POSITION THAT IS NOT CHANGING, not as reported
# velocity, and that choice is worth the paragraph. /joint_states carries a
# velocity array and it is tempting to threshold it. Measured on this arm at
# a genuine standstill — every joint within 0.0002 rad of its target, which
# is as stopped as a thing can be — wrist_1 reports |v| up to 0.26 rad/s and
# the gripper knuckle reports a flat 0.5 rad/s that never changes at all.
# Those numbers are noise and quantisation from the physics engine, not
# motion. The positions, over the same interval, are rock steady. So the
# positions are what we watch.
# The two thresholds do different jobs and both are set from measurement.
#
# SETTLE_TOL is what "arrived" means, and it is set at 0.005 rad because the
# controllers demonstrably reach that in a couple of seconds — the residual
# at a true standstill is 0.0002 rad. Leaving it at 0.02 lets a joint stop
# 0.02 rad out, which 0.7 m from the shoulder is 14 mm of tool error, and
# that showed up as picks that missed by a centimetre.
#
# SETTLE_DRIFT stops an early return during the approach; it does not need to
# be tight, because SETTLE_TOL is now doing the accuracy work. Squeezing it
# to 0.0008 only bought a slower wait: moves went from ~2 s to ~7 s with no
# improvement in where the arm ended up.
SETTLE_TOL = 0.005         # rad from the target
SETTLE_DRIFT = 0.002       # rad of movement allowed across the window
SETTLE_WINDOW = 0.3        # s of history that has to be that still
PUBLISH_HZ = 20.0

# How close the grasp point has to be to a Cartesian request to call it
# arrived. Measured worst case over targets from 0.30 m to 0.75 m out is
# under 4 mm, so 10 mm passes comfortably while still catching a real miss —
# and a 50 mm block tolerates far more than 10 mm anyway.
POS_TOL = 0.010          # m


class Arm:
    """A UR5e on a bench, plus its gripper."""

    def __init__(self, node, tcp_offset: float = K.TCP_OFFSET):
        self.node = node
        self.tcp_offset = tcp_offset
        self._state: dict[str, float] = {}
        self._velocity: dict[str, float] = {}
        self._history: deque = deque()      # (t, joint tuple) for the drift test

        node.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        # grasp_server's verdict on what is currently held. Optional: if that
        # node is not running this stays None and holding_something says so
        # rather than guessing.
        self._held = None
        self._heard_grasp = False
        node.create_subscription(String, '/grasp/state', self._on_grasp, 10)
        self._arm_pubs = {j: node.create_publisher(Float64, f'/{j}/cmd', 10)
                          for j in K.JOINTS}
        self._grip_pubs = {j: node.create_publisher(Float64, f'/{j}/cmd', 10)
                           for j in GRIPPER_SIGNS}
        # Last thing we ASKED for, which is not the same as where the arm is.
        # Kept so move_relative and the grasp logic have an intent to build
        # on rather than re-deriving it from noisy measurements.
        self._target = None
        self._grip_target = GRIPPER_OPEN

    # ── State ───────────────────────────────────────────────────────────────
    def _on_joints(self, msg: JointState) -> None:
        self._state.update(zip(msg.name, msg.position))
        if len(msg.velocity) == len(msg.name):
            self._velocity.update(zip(msg.name, msg.velocity))
        if self.ready:
            now = time.time()
            self._history.append((now, self.joints))
            while self._history and now - self._history[0][0] > SETTLE_WINDOW * 2:
                self._history.popleft()

    def _on_grasp(self, msg: String) -> None:
        self._heard_grasp = True
        self._held = msg.data if msg.data and msg.data != 'none' else None

    def wait_for_state(self, timeout: float = 10.0) -> bool:
        """Block until /joint_states has delivered all six arm joints.

        Worth calling first in every script. DDS discovery is not instant,
        and a script that reads self.joints on its first line gets NaNs from
        a perfectly healthy robot.
        """
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if all(j in self._state for j in K.JOINTS):
                return True
        return False

    @property
    def ready(self) -> bool:
        return all(j in self._state for j in K.JOINTS)

    @property
    def joints(self) -> tuple:
        """The six arm joint angles as measured, in K.JOINTS order."""
        return tuple(self._state.get(j, float('nan')) for j in K.JOINTS)

    @property
    def tcp(self):
        """(x, y, z) of the grasp point in the world frame, from measurement."""
        return K.fk(self.joints, 'tcp')[:3, 3]

    @property
    def pose(self):
        """4x4 pose of ur5e_tool0 in the world frame, from measurement."""
        return K.fk(self.joints)

    @property
    def moving(self) -> bool:
        return self._creeping()

    @property
    def knuckle(self) -> float:
        return self._state.get(KNUCKLE, float('nan'))

    @property
    def tip_separation_mm(self) -> float:
        """Gap between the finger-tip link origins, from the measured knuckle."""
        return TIP_SEPARATION_MM_AT_ZERO + TIP_SEPARATION_MM_PER_RAD * self.knuckle

    # ── Joint-space motion ──────────────────────────────────────────────────
    def move_joints(self, q, timeout: float = 15.0, tol: float = SETTLE_TOL) -> bool:
        """Command six joint angles and wait until the arm settles there.

        Republishes at PUBLISH_HZ for the whole wait rather than sending once.
        The gz JointPositionController latches its target, so one message is
        enough in principle — but one message is also all it takes for a
        dropped packet during startup to leave the arm holding an old pose
        while the script happily proceeds. Republishing costs nothing.

        Returns False on timeout, and the arm is left wherever it got to.
        """
        q = list(q)
        self._target = tuple(q)
        end = time.time() + timeout
        period = 1.0 / PUBLISH_HZ
        next_pub = 0.0
        while time.time() < end and rclpy.ok():
            now = time.time()
            if now >= next_pub:
                for joint, value in zip(K.JOINTS, q):
                    self._arm_pubs[joint].publish(Float64(data=float(value)))
                next_pub = now + period
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if not self.ready:
                continue
            if (max(abs(a - b) for a, b in zip(self.joints, q)) < tol
                    and not self._creeping()):
                return True
        return False

    def _creeping(self) -> bool:
        """Has any joint moved measurably over the last SETTLE_WINDOW seconds?

        Not enough history yet counts as "still moving": a move that has only
        just started has nothing to compare against, and answering "stopped"
        there would end every wait immediately.
        """
        now = time.time()
        window = [q for t, q in self._history if now - t <= SETTLE_WINDOW]
        if len(window) < 5:
            return True
        for i in range(len(K.JOINTS)):
            column = [q[i] for q in window]
            if max(column) - min(column) > SETTLE_DRIFT:
                return True
        return False

    def joint_error(self, q=None) -> float:
        """Worst |measured - commanded| over the six joints, in radians."""
        q = self._target if q is None else q
        if q is None or not self.ready:
            return float('nan')
        return max(abs(a - b) for a, b in zip(self.joints, q))

    def home(self, **kw) -> bool:
        return self.move_joints(K.HOME, **kw)

    # ── Cartesian motion ────────────────────────────────────────────────────
    def move_to(self, x: float, y: float, z: float, yaw: float = 0.0,
                timeout: float = 15.0, tol: float = SETTLE_TOL,
                pos_tol: float = POS_TOL):
        """Put the grasp point at (x, y, z) with the tool pointing down.

        Returns True on arrival, False on timeout, and None when there is no
        IK solution at all — three different outcomes that a caller wants to
        tell apart. `if arm.move_to(...)` treats the last two the same, which
        is usually right; `is None` distinguishes "cannot" from "did not".

        IK is seeded from where the arm IS, so consecutive calls stay in the
        same solution branch and the arm does not fling its elbow around
        between two nearby waypoints.
        """
        seed = self.joints if self.ready else K.HOME
        q = K.ik_down(x, y, z, yaw, seed=seed)
        if q is None:
            return None
        self.move_joints(q, timeout=timeout, tol=tol)
        # Judge a Cartesian move by the Cartesian result. A joint-space
        # verdict is the wrong question here and gives the wrong answer in
        # both directions: wrist_3 can sit 0.02 rad off with the tool exactly
        # where it was asked for, because the tool is ON the wrist_3 axis,
        # and a small shoulder error 0.8 m out is worth centimetres.
        return math.dist(self.tcp, (x, y, z)) < pos_tol

    def move_relative(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, **kw):
        """Shift the grasp point by (dx, dy, dz), keeping the tool pointing down."""
        x, y, z = self.tcp
        return self.move_to(x + dx, y + dy, z + dz, **kw)

    def can_reach(self, x: float, y: float, z: float, yaw: float = 0.0):
        """(True, angles) or (False, reason) — ask before you move."""
        return K.reachable(x, y, z, yaw, seed=self.joints if self.ready else None)

    # ── Gripper ─────────────────────────────────────────────────────────────
    def set_gripper(self, knuckle: float, timeout: float = 6.0) -> None:
        """Drive every finger joint to match a knuckle angle, then wait for
        the fingers to STOP — either at the target or stalled on an object.

        There is no success/failure return on purpose. A gripper that closes
        on something stalls short of its command; that is what gripping IS,
        so "did the joint reach the target" is the wrong question. Ask
        holding_something or tip_separation_mm afterwards.

        Waiting for the knuckle to stop moving, rather than for a fixed
        number of seconds, is what makes this reliable. A fixed 2 s wait
        looked fine on an empty gripper and returned with the knuckle still
        at 0.00 the moment there was a block in the way — every check after
        it then read the state of a gripper that had not moved yet.
        """
        self._grip_target = float(knuckle)
        end = time.time() + timeout
        start = time.time()
        period = 1.0 / PUBLISH_HZ
        next_pub = 0.0
        recent: deque = deque()
        while time.time() < end and rclpy.ok():
            now = time.time()
            if now >= next_pub:
                for joint, sign in GRIPPER_SIGNS.items():
                    self._grip_pubs[joint].publish(Float64(data=knuckle * sign))
                next_pub = now + period
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if now - start < GRIP_MIN_TIME or not math.isfinite(self.knuckle):
                continue
            # Two ways to be finished, and BOTH are needed. "Arrived" covers
            # a free close or open. "Stalled" covers closing onto an object.
            # Testing only for "stopped" returns instantly every time,
            # because a gripper that has not started yet is also not moving —
            # which is how a release once returned in 0.8 s with the block
            # still gripped, and every check afterwards read a stale state.
            if abs(self.knuckle - knuckle) < GRIP_ARRIVED:
                return
            recent.append((now, self.knuckle))
            while recent and now - recent[0][0] > GRIP_STALL_TIME:
                recent.popleft()
            if (len(recent) > 10
                    and now - start > GRIP_MIN_TIME + GRIP_STALL_TIME):
                values = [k for _, k in recent]
                if max(values) - min(values) < GRIP_DRIFT:
                    return          # stalled: it is pressing on something

    def open_gripper(self, **kw) -> None:
        self.set_gripper(GRIPPER_OPEN, **kw)

    def close_gripper(self, **kw) -> None:
        self.set_gripper(GRIPPER_CLOSED, **kw)

    @property
    def held_object(self):
        """Name of the model grasp_server says is held, or None.

        None means one of two different things — nothing is held, or
        grasp_server is not running — and heard_from_grasp_server tells them
        apart. Worth checking before treating a None as "the pick failed".
        """
        return self._held

    @property
    def heard_from_grasp_server(self) -> bool:
        return self._heard_grasp

    @property
    def holding_something(self) -> bool:
        """Is the arm holding a block?

        This asks grasp_server, because on this platform grasp_server is the
        thing that decides. It would be nicer to read it off the hardware —
        "the fingers stopped early, so they are pressing on something" is how
        you would do it on a real Robotiq — but a welded block does not
        collide with the fingers that hold it, so they never stop early. The
        honest answer is to ask the component that knows.
        """
        return self._held is not None

    # ── Convenience ─────────────────────────────────────────────────────────
    def describe(self) -> str:
        p = self.tcp
        deg = ' '.join(f'{math.degrees(v):+7.1f}' for v in self.joints)
        return (f'[{deg} ] deg   tcp ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m'
                f'   fingers {self.tip_separation_mm:.0f} mm'
                + (f'   holding {self._held}' if self._held else ''))

    def sleep(self, seconds: float) -> None:
        """Spin the node for `seconds` — never use time.sleep() with ROS."""
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.02)
