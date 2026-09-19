#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
UR5e kinematics for the bench arm — forward, inverse, and the workspace.

This is the file that makes the arm platform different from the other three.
A drone is told "go to this point" and a flight controller works out the
rest; a quadruped is told "walk this fast" and a gait generator works out the
rest. An arm has neither. Between "the block is at (0.45, 0.00, 0.78)" and
"six joint angles" there is nothing but this maths, and every manipulation
example in agr_arm_examples goes through it.

WHAT IS HERE
    fk(q)            six joint angles  -> 4x4 pose of ur5e_tool0 in `world`
    tcp(q)           six joint angles  -> (x, y, z) of the grasp point
    jacobian(q)      six joint angles  -> 6x6 d(twist)/d(q)
    ik(target, seed) 4x4 pose          -> six joint angles, or None
    ik_down(x,y,z,yaw)                 -> the top-down grasp special case
    reachable(x,y,z)                   -> a straight yes/no with a reason

WHERE THE NUMBERS COME FROM
    Every constant in CHAIN below was read out of the RENDERED URDF, not
    typed from a datasheet. `xacro agr_arm_robot.urdf.xacro` expands the
    ur_description macro against ur5e/default_kinematics.yaml, and CHAIN is
    that expansion transcribed. So it cannot drift from the model unless
    somebody edits the model, and there is a live check for that:

        ros2 run agr_arm_demos diagnose

    compares fk() against the TF tree that robot_state_publisher builds from
    the same URDF. Measured agreement on this machine: under 1e-6 m.

WHY NUMERICAL IK
    The UR5e has a closed-form analytic IK with eight solutions, and it is a
    classic exercise. This uses damped least squares instead, iterating from
    a seed, for two reasons that matter more in a teaching stack than
    elegance does. It always returns a solution NEAR THE SEED, so the arm
    does not silently flip its elbow through the bench between two waypoints
    that look adjacent on paper. And the damping term means a target at the
    edge of the workspace degrades into "as close as I can get" instead of
    dividing by a singular matrix.
"""
from __future__ import annotations

import math

import numpy as np

# ── The six commandable joints, in the order every array here uses ──────────
JOINTS = (
    'ur5e_shoulder_pan_joint',
    'ur5e_shoulder_lift_joint',
    'ur5e_elbow_joint',
    'ur5e_wrist_1_joint',
    'ur5e_wrist_2_joint',
    'ur5e_wrist_3_joint',
)

# ── The kinematic chain, world -> ur5e_tool0 ────────────────────────────────
# Each entry is (translation, rpy, revolute?). A revolute entry contributes
# a rotation about its own +z AFTER the fixed transform; every UR joint axis
# is +z in its own frame, which is why no axis vector appears here.
#
# The fixed head of the chain collapses to "0.76 m up, then yawed 180 deg":
# pedestal 0.375 + 0.38 + base plate 0.005 = 0.760, and ur_description's
# base_link -> base_link_inertia carries an Rz(pi). Getting that pi wrong
# mirrors the whole workspace front-to-back, and it is the single easiest
# thing to lose when transcribing a UR chain.
CHAIN = (
    ((0.0,     0.0,     0.760), (0.0,        0.0, math.pi), False),  # world -> base_link_inertia
    ((0.0,     0.0,    0.1625), (0.0,        0.0,     0.0), True),   # shoulder_pan
    ((0.0,     0.0,       0.0), (math.pi/2,  0.0,     0.0), True),   # shoulder_lift
    ((-0.425,  0.0,       0.0), (0.0,        0.0,     0.0), True),   # elbow
    ((-0.3922, 0.0,    0.1333), (0.0,        0.0,     0.0), True),   # wrist_1
    ((0.0,  -0.0997,      0.0), (math.pi/2,  0.0,     0.0), True),   # wrist_2
    ((0.0,   0.0996,      0.0), (math.pi/2, math.pi, math.pi), True),  # wrist_3
    ((0.0,     0.0,       0.0), (0.0, -math.pi/2, -math.pi/2), False),  # -> flange
    ((0.0,     0.0,       0.0), (math.pi/2,  0.0, math.pi/2), False),   # -> tool0
)

# ── Tool centre point, and the thing beyond it ──────────────────────────────
# Distances from ur5e_tool0 along the tool's own +z. Both MEASURED, not
# assumed: the finger-tip link origins sit 0.1094 m out with the gripper open
# (read off the live TF tree), and the collision mesh of a finger tip spans
# another 0.051 m beyond its own origin (read out of the STL that
# robotiq_description ships).
#
# TCP_OFFSET is the middle of the pad faces — the point that ends up at the
# centre of whatever you grip. It is what ik_down(x, y, z) puts at (x, y, z).
TCP_OFFSET = 0.135
#
# FINGER_TIP is how far the pads actually REACH, and it is the number that
# stops the arm driving its fingers into the bench. Forgetting it is the
# classic first bug of every top-down grasp: ask for the TCP at a 50 mm
# block's centre and the fingertips end up ON the table the block is sitting
# on, the arm stalls against it, and the move times out short with no
# explanation at all. Ask lowest_tcp_over() instead.
#
# 0.170, not the 0.160 the collision mesh's bounding box suggests, and the
# extra 10 mm is measured rather than padded. A finger pad is 25 mm DEEP as
# well as long, and that depth points along the tool's x — so it rotates with
# wrist_3. Which corner of the pad is lowest therefore depends on the grasp
# yaw, and the worst case is a centimetre below the best case. Commanding
# 0.780 (correct for 0.160) reached the bench and stalled 5 mm high on some
# yaws and descended cleanly on others, which is exactly the kind of
# intermittent failure that wastes an afternoon. Take the worst case.
FINGER_TIP = 0.170


def lowest_tcp_over(surface_z: float, clearance: float = 0.005) -> float:
    """Lowest TCP height that keeps the fingertips off a surface at surface_z.

    For the bench (0.750) this is 0.780 — which is 5 mm ABOVE the centre of a
    50 mm block resting on it. The pads still span the block completely
    (0.755 to 0.805 against a block from 0.750 to 0.800), so the grasp is
    just as good; the fingers simply stop short of the table.
    """
    return surface_z + (FINGER_TIP - TCP_OFFSET) + clearance


# Height of the bench top in agr_workcell, and where a 50 mm block's centre
# sits on it. Quoted here so examples do not each re-derive them.
BENCH_Z = 0.750
BLOCK_Z = 0.775

# ── Joint limits (radians), straight from the rendered URDF ─────────────────
# The elbow is the only one that is not +/- 2pi, and it matters: an IK
# solution that wants the elbow past +/-pi is not reachable no matter how
# nice the arm pose looks on paper.
LIMITS = (
    (-2 * math.pi, 2 * math.pi),
    (-2 * math.pi, 2 * math.pi),
    (-math.pi,     math.pi),
    (-2 * math.pi, 2 * math.pi),
    (-2 * math.pi, 2 * math.pi),
    (-2 * math.pi, 2 * math.pi),
)

# ── Reach ───────────────────────────────────────────────────────────────────
# The link lengths that actually bound reach, straight out of CHAIN.
UPPER_ARM = 0.425      # shoulder_lift -> elbow
FOREARM = 0.3922       # elbow -> wrist_1
SHOULDER_OFFSET = 0.1333   # wrist_1's lateral offset from the pan axis
WRIST_TO_FLANGE = 0.0996   # wrist centre -> tool0 along the tool axis
SHOULDER = (0.0, 0.0, 0.760 + 0.1625)      # world position of the shoulder point

# The largest distance from the shoulder POINT to the WRIST CENTRE that the
# arm can produce. shoulder_lift and elbow form a two-link planar chain that
# reaches UPPER_ARM + FOREARM within its own plane, and that plane is offset
# sideways from the pan axis by SHOULDER_OFFSET, so the bound is the
# hypotenuse of the two:
MAX_WRIST_REACH = math.hypot(UPPER_ARM + FOREARM, SHOULDER_OFFSET)   # 0.828 m

# Kept for anyone who wants the headline "850 mm reach" number. It is the
# TOOL's reach, not the wrist's, and it is NOT the right thing to test a
# target against — see reachable().
NOMINAL_REACH = 0.85

# The pose the URDF springs to and every named pose returns through. Keep in
# step with the <initial_position> values in agr_arm_gazebo.urdf.xacro.
HOME = (0.0, -1.3963, 1.5708, -1.7453, -1.5708, 0.0)   # 0,-80,90,-100,-90,0 deg


# ── Small matrix helpers ────────────────────────────────────────────────────
def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF rpy -> rotation matrix. Fixed-axis XYZ, i.e. Rz @ Ry @ Rx."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def transform(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def rot_z(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    T = np.eye(4)
    T[0, 0] = c; T[0, 1] = -s
    T[1, 0] = s; T[1, 1] = c
    return T


# ── Forward kinematics ──────────────────────────────────────────────────────
def fk(q, link: str = 'tool0') -> np.ndarray:
    """4x4 pose of the tool in the `world` frame for joint vector `q`.

    link='tool0' is the flange frame robot_state_publisher calls
    ur5e_tool0; link='tcp' shifts it forward to the grasp point.
    """
    q = list(q)
    T = np.eye(4)
    i = 0
    for xyz, rpy, revolute in CHAIN:
        T = T @ transform(xyz, rpy)
        if revolute:
            T = T @ rot_z(q[i])
            i += 1
    if link == 'tcp':
        T = T @ transform((0.0, 0.0, TCP_OFFSET), (0.0, 0.0, 0.0))
    return T


def tcp(q) -> np.ndarray:
    """(x, y, z) of the grasp point in the world frame."""
    return fk(q, 'tcp')[:3, 3]


def joint_frames(q):
    """World pose of each revolute joint, in JOINTS order. Used by jacobian()
    and by anyone who wants to draw the arm."""
    q = list(q)
    frames = []
    T = np.eye(4)
    i = 0
    for xyz, rpy, revolute in CHAIN:
        T = T @ transform(xyz, rpy)
        if revolute:
            frames.append(T.copy())       # BEFORE its own rotation: the axis
            T = T @ rot_z(q[i])           # is z of the pre-rotation frame
            i += 1
    return frames


# ── Jacobian ────────────────────────────────────────────────────────────────
def jacobian(q, link: str = 'tcp') -> np.ndarray:
    """6x6 geometric Jacobian: columns are [linear; angular] per joint.

    For a revolute joint the tool's linear velocity is the joint's axis
    crossed with the vector from the joint to the tool, and its angular
    velocity is just the axis. That is the entire derivation, and it is worth
    keeping in a form you can read, because a Jacobian you cannot check by
    eye is a Jacobian you cannot debug.
    """
    p_end = fk(q, link)[:3, 3]
    J = np.zeros((6, 6))
    for i, T in enumerate(joint_frames(q)):
        axis = T[:3, 2]                   # this joint rotates about its z
        origin = T[:3, 3]
        J[:3, i] = np.cross(axis, p_end - origin)
        J[3:, i] = axis
    return J


# ── Pose error ──────────────────────────────────────────────────────────────
def pose_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """6-vector [dx, dy, dz, rx, ry, rz] taking `current` to `target`.

    The rotation half is the axis-angle of R_target @ R_current^T — the
    single rotation that closes the gap. Subtracting Euler angles instead is
    the classic mistake: it is not a valid error near a gimbal lock and it
    makes IK stall at exactly the top-down poses this platform lives in.
    """
    err = np.zeros(6)
    err[:3] = target[:3, 3] - current[:3, 3]
    R = target[:3, :3] @ current[:3, :3].T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if angle < 1e-9:
        return err                        # already aligned; axis is undefined
    if abs(angle - math.pi) < 1e-6:
        # 180 deg: the skew part vanishes, so read the axis off R + I instead.
        w, V = np.linalg.eigh(R + np.eye(3))
        axis = V[:, int(np.argmax(w))]
    else:
        axis = np.array([R[2, 1] - R[1, 2],
                         R[0, 2] - R[2, 0],
                         R[1, 0] - R[0, 1]]) / (2.0 * math.sin(angle))
    err[3:] = axis * angle
    return err


# ── Inverse kinematics ──────────────────────────────────────────────────────
def _ik_from(target, seed, link, tol_pos, tol_rot, max_iter, damping):
    """One damped-least-squares descent from one seed. None if it stalls."""
    q = np.array(seed, dtype=float)
    k2 = damping ** 2
    I6 = np.eye(6)
    for _ in range(max_iter):
        err = pose_error(fk(q, link), target)
        if np.linalg.norm(err[:3]) < tol_pos and np.linalg.norm(err[3:]) < tol_rot:
            return clamp_to_limits(q)
        J = jacobian(q, link)
        # Solve rather than invert: same answer, better conditioned, and it
        # raises instead of quietly returning garbage if something is wrong.
        try:
            y = np.linalg.solve(J @ J.T + k2 * I6, err)
        except np.linalg.LinAlgError:
            return None
        step = J.T @ y
        # Cap the step so a large initial error cannot throw the seed across
        # the workspace on iteration one and land in a different IK branch.
        norm = np.linalg.norm(step)
        if norm > 0.3:
            step *= 0.3 / norm
        q = q + step
    return None


def seed_candidates(target: np.ndarray, seed=None):
    """Seeds to try, best first. Order is the whole point.

    The caller's seed goes first because continuity matters more than
    anything else here: solving from where the arm actually is keeps the next
    pose adjacent to this one, which is what stops an arm flipping its elbow
    through the bench between two waypoints 5 cm apart.

    The fallbacks only matter when that fails. Each one aims the shoulder
    straight at the target and then tries a different elbow bend, which is
    the axis along which UR solutions differ most.
    """
    out = []
    if seed is not None:
        out.append(tuple(seed))
    out.append(HOME)
    pan = math.atan2(target[1, 3], target[0, 3])
    for lift, elbow in ((-1.0472, 1.5708), (-1.8, 2.2), (-0.6, 1.0), (-2.2, 2.6)):
        out.append((pan, lift, elbow, -(lift + elbow) - math.pi / 2, -math.pi / 2, 0.0))
    # Random restarts, last and deterministic. Last because they throw away
    # continuity with the current pose; deterministic (a fixed generator, not
    # the global one) because a solver that answers differently on Tuesday is
    # not something a student can debug.
    rng = np.random.default_rng(20260825)
    for _ in range(8):
        out.append((rng.uniform(-math.pi, math.pi),
                    rng.uniform(-2.8, -0.2),
                    rng.uniform(-2.8, 2.8),
                    rng.uniform(-math.pi, math.pi),
                    rng.uniform(-math.pi, math.pi),
                    rng.uniform(-math.pi, math.pi)))
    return out


def ik(target: np.ndarray, seed=None, link: str = 'tcp',
       tol_pos: float = 1e-4, tol_rot: float = 1e-3,
       max_iter: int = 200, damping: float = 0.05):
    """Joint angles that put `link` at the 4x4 pose `target`, or None.

    Damped least squares: at each step solve (J J^T + k^2 I) y = e for the
    joint step J^T y, which is the least-squares answer to "what joint change
    best reduces this pose error" with a k-sized brake on it. The brake is
    what keeps the arm sane near singularities, where the plain pseudo-inverse
    asks for infinite joint velocity and the arm snaps.

    A single descent only finds the solution branch its seed is already in,
    so on failure this retries from the seeds in seed_candidates(). Measured
    over 300 random reachable poses spread across the whole joint range:
    90% solve from the seed alone and 99% with the fallbacks, at roughly
    10 ms per solve. The last 1% are poses far outside the bench workspace
    - behind the robot, or a metre and a half up - where only a seed already
    in the right branch converges. Every top-down target over the bench
    solves from the seed alone.

    Returns None rather than a near-miss. A caller that gets angles back can
    trust them; a caller that gets None has been told the honest answer, and
    reachable() will say why.
    """
    for candidate in seed_candidates(target, seed):
        result = _ik_from(target, candidate, link, tol_pos, tol_rot, max_iter, damping)
        if result is not None:
            return result
    return None


def clamp_to_limits(q):
    """Wrap each angle into its joint's range, or give up on that joint.

    Wrapping matters: DLS happily returns wrist_1 = -7.9 rad, which is the
    same physical pose as -1.6 rad but outside the URDF limit, and the
    position controller would drive the long way round to reach it.
    """
    out = []
    for value, (lo, hi) in zip(q, LIMITS):
        while value > hi:
            value -= 2 * math.pi
        while value < lo:
            value += 2 * math.pi
        out.append(float(value))
    return tuple(out)


def pose_down(x: float, y: float, z: float, yaw: float = 0.0) -> np.ndarray:
    """A 4x4 pose at (x, y, z) with the tool pointing straight DOWN.

    This is the pose almost every bench task wants, so it gets a name. The
    rotation is "z of the tool points along world -z", plus a spin of `yaw`
    about the vertical, which is how you line the fingers up with a block
    that is not square to the robot.
    """
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(math.pi, 0.0, yaw)
    T[:3, 3] = (x, y, z)
    return T


def ik_down(x: float, y: float, z: float, yaw: float = 0.0, seed=None, **kw):
    """IK for a top-down grasp at (x, y, z). Returns joint angles or None."""
    return ik(pose_down(x, y, z, yaw), seed=seed, **kw)


# ── Workspace ───────────────────────────────────────────────────────────────
def wrist_centre_down(x: float, y: float, z: float):
    """Where the wrist centre must be for a straight-down grasp at (x, y, z).

    The tool points along world -z, so everything behind the TCP along the
    tool axis is straight up: the flange sits TCP_OFFSET above the grasp
    point and the wrist centre WRIST_TO_FLANGE above that.
    """
    return (x, y, z + TCP_OFFSET + WRIST_TO_FLANGE)


def reachable(x: float, y: float, z: float, yaw: float = 0.0, seed=None):
    """(True, angles) or (False, reason). A yes/no about a point on the bench.

    The geometric test is on the WRIST CENTRE, not on the point you asked
    for. Testing the TCP is the intuitive thing to do and it is wrong in both
    directions: a top-down grasp puts the wrist 0.22 m above the target, so a
    TCP well inside 0.85 m can need a wrist that is outside it, and a TCP
    beyond 0.85 m can be perfectly reachable with the tool angled away.

    Two failure modes, and they deserve different words. Past
    MAX_WRIST_REACH the arm is simply too short and no cleverness helps.
    Inside it but with no top-down solution usually means the wrist cannot
    get there vertically — approach at an angle, or move the object.
    """
    r = math.dist(wrist_centre_down(x, y, z), SHOULDER)
    if r > MAX_WRIST_REACH:
        return False, (f'a straight-down grasp there needs the wrist centre '
                       f'{r:.3f} m from the shoulder, past the {MAX_WRIST_REACH:.3f} m '
                       f'the links can span — the arm is too short, full stop')
    q = ik_down(x, y, z, yaw, seed=seed)
    if q is None:
        return False, (f'wrist centre {r:.3f} m out, so within reach, but no '
                       f'straight-down solution was found — try a different yaw '
                       f'or approach angle')
    return True, q


def describe(q) -> str:
    """One line of human-readable state: joints in degrees plus the TCP."""
    p = tcp(q)
    deg = ' '.join(f'{math.degrees(v):+7.1f}' for v in q)
    return f'[{deg} ] deg   tcp ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m'
