#!/usr/bin/env python3
"""
RAISE 2026 — navigation server (drive the Husky to a named waypoint).
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

Exposes one Trigger service per NAMED waypoint:
    /nav_to_home              — back to the origin (0, 0)
    /nav_to_tomato_row_1      — front-left tomato bed
    /nav_to_tomato_row_2      — front-right tomato bed
    /nav_to_olive_grove       — further-out olive trees

Pedagogy:
- Same shape as gripper_server and move_to_pose_server: each "tool" is
  a Trigger service with NO arguments. Adding a new waypoint is a one-
  line addition to the WAYPOINTS dict — `create_service` is generated
  automatically.
- We deliberately DON'T use Nav2 here. Nav2 needs a map, a planner, a
  costmap, etc. — too much for Day 1. The whole controller is ~25
  lines of Python: a P-controller on heading + a P-controller on
  distance, looping at 10 Hz.

Implementation:
- Pose source:  /odom (spawn-relative) + a fixed spawn offset (SPAWN_*) =
  WORLD coordinates that match the world-frame WAYPOINTS. /odom always
  publishes, so this is reliable; in sim the dead-reckoning drift over the
  short demo distances is negligible.
- Obstacle source: /scan (2-D LiDAR) for reactive avoidance.
- Action sink:  /cmd_vel  (Twist messages, bridged).
- Concurrency: a MultiThreadedExecutor + ReentrantCallbackGroup so the
  pose + scan subscribers keep firing while a service callback is running
  the drive loop. Without this, the callback would block its own
  sensor updates and the controller would freeze on the first sample.

Run:
    ros2 run raise2026_tools navigation_server
"""

import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


# ── Named waypoints (x, y) in the greenhouse_2026 world ────────────────────
# The tomato crop now has rows 3 m apart (y=-3, 0, +3) and plants 2 m apart
# (x=-4..+4), leaving wide DRIVE AISLES at y=+1.5 and y=-1.5. The robot
# spawns at (-6, +1.5) — west of the crop, in the north aisle, facing east.
# All waypoints sit in the +1.5 aisle (or the open west margin), so a
# straight drive reaches them with ~1.2 m of clearance on each side.
WAYPOINTS = {
    'home':          (-6.0,  1.5),   # depot: spawn, west end of the +1.5 aisle (open)
    'tomato_row_1':  (-2.0,  1.5),   # beside the west plants (arm reaches y=0 / y=+3 plants)
    'tomato_row_2':  ( 2.0,  1.5),   # beside the east plants, same aisle
    'olive_grove':   (-6.0,  5.0),   # north of the crop (clear at x=-6), toward the olive trees
}

# ── Controller tunables ────────────────────────────────────────────────────
GOAL_TOLERANCE_M    = 0.4    # within this distance → arrived
HEADING_TOLERANCE   = 0.15   # radians (~9°), if heading worse → turn-in-place
MAX_LINEAR_SPEED    = 0.5    # m/s — capped Husky forward speed
MAX_ANGULAR_SPEED   = 0.8    # rad/s — capped Husky yaw rate
LINEAR_GAIN         = 0.6    # m/s per m of remaining distance
ANGULAR_GAIN        = 1.5    # rad/s per rad of heading error
CONTROL_RATE_HZ     = 10
TIMEOUT_S           = 45.0   # give up after this many seconds (safety)

# ── Spawn offset (odom → world) ────────────────────────────────────────────
# /odom from the DiffDrive plugin is SPAWN-RELATIVE: it reads (0, 0) at
# startup wherever the robot spawned. Our WAYPOINTS are in WORLD coordinates,
# so we convert odom → world by adding the spawn pose. These MUST match the
# spawn args in sim.launch.py (defaults: x=-3, y=1, yaw=0). If you change the
# spawn there, change them here too. (In sim, odom dead-reckoning is accurate
# enough over the short demo distances that this is effectively ground truth.)
SPAWN_X   = -6.0
SPAWN_Y   =  1.5
SPAWN_YAW =  0.0

# ── Reactive obstacle-avoidance tunables ───────────────────────────────────
# This is NOT a planner. It's a reactive layer: while driving toward the
# goal, if /scan sees something close in the forward cone, we steer toward
# whichever side is more open and slow down. No map, no global path.
# The crop aisles are ~1.5 m to each plant row, so a moderate cone + trigger
# distance is fine: plants beside the aisle stay outside the cone, and the
# robot only reacts to something genuinely in its path (e.g. an animal that
# wandered into the lane).
AVOID_DISTANCE_M    = 0.9    # start avoiding when an obstacle is closer than this
AVOID_CONE_DEG      = 40     # only ±this around straight-ahead counts as "in the way"
FRONT_STOP_DEG      = 18     # hard "dead ahead" wedge for emergency slow-down
AVOID_TURN_SPEED    = 0.7    # rad/s yaw applied when steering around an obstacle

# ── Relative-motion increments ─────────────────────────────────────────────
# Used by /drive_forward, /drive_backward, /turn_left, /turn_right. The agent
# can call them repeatedly (e.g. "back 3 m" → 3× /drive_backward). Keeping a
# fixed step keeps the tool set Trigger-only (no custom message types in D1).
STEP_DISTANCE_M     = 1.0          # metres per forward/backward step
STEP_TURN_RAD       = math.pi / 2  # 90° per turn step
TURN_TOLERANCE_RAD  = 0.05         # ~3° — close enough when rotating in place


def yaw_from_quat(q) -> float:
    """Extract yaw (rotation around Z) from a quaternion.

    Standard formula. The DiffDrive plugin gives us a Quaternion;
    we need a scalar yaw for the heading controller.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    """Wrap an angle to (-π, +π]. Heading errors should never be 350° etc."""
    return ((a + math.pi) % (2.0 * math.pi)) - math.pi


class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')

        # Single reentrant group so /odom callbacks and service callbacks
        # can interleave on the MultiThreadedExecutor.
        self.cb_group = ReentrantCallbackGroup()

        # Cached WORLD pose (x, y, yaw) of the robot, derived from /odom +
        # the spawn offset. None until the first /odom message arrives.
        self._pose_xy_yaw: 'tuple[float, float, float] | None' = None
        # Cached most recent LaserScan, for reactive avoidance.
        self._scan: 'LaserScan | None' = None

        # Localization: /odom (spawn-relative) + a fixed spawn offset =
        # world coordinates. See SPAWN_* above. /odom always publishes, so
        # this is reliable; in sim the dead-reckoning drift is negligible.
        self.create_subscription(
            Odometry, '/odom', self._on_odom, 10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, 10,
            callback_group=self.cb_group,
        )
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # One Trigger service per named waypoint. Same factory pattern as
        # move_to_pose_server.
        for name in WAYPOINTS:
            self.create_service(
                Trigger, f'/nav_to_{name}', self._make_handler(name),
                callback_group=self.cb_group,
            )

        # Relative-motion services (move from wherever the robot is now).
        self.create_service(Trigger, '/drive_forward',
                            self._make_rel_handler('forward'),  callback_group=self.cb_group)
        self.create_service(Trigger, '/drive_backward',
                            self._make_rel_handler('backward'), callback_group=self.cb_group)
        self.create_service(Trigger, '/turn_left',
                            self._make_rel_handler('turn_left'),  callback_group=self.cb_group)
        self.create_service(Trigger, '/turn_right',
                            self._make_rel_handler('turn_right'), callback_group=self.cb_group)

        names = ' '.join(f'/nav_to_{n}' for n in WAYPOINTS)
        self.get_logger().info(
            f'navigation services ready: {names} '
            '/drive_forward /drive_backward /turn_left /turn_right'
        )

    def _on_odom(self, msg: Odometry) -> None:
        """Convert spawn-relative /odom to a WORLD (x, y, yaw) and cache it."""
        p = msg.pose.pose.position
        odom_yaw = yaw_from_quat(msg.pose.pose.orientation)
        # Rotate the odom offset by the spawn yaw, then translate by spawn xy.
        c, s = math.cos(SPAWN_YAW), math.sin(SPAWN_YAW)
        world_x = SPAWN_X + (p.x * c - p.y * s)
        world_y = SPAWN_Y + (p.x * s + p.y * c)
        world_yaw = wrap_angle(SPAWN_YAW + odom_yaw)
        self._pose_xy_yaw = (world_x, world_y, world_yaw)

    def _on_scan(self, msg: LaserScan) -> None:
        """Cache the most recent LaserScan."""
        self._scan = msg

    def _clearance(self) -> 'tuple[float, float, float]':
        """Return (front, left, right) minimum distances over the forward cone.

        - front: nearest valid return within ±FRONT_STOP_DEG of straight-ahead
        - left:  nearest within the left half of the avoidance cone
        - right: nearest within the right half
        We use these three numbers to decide: is something ahead, and which
        way is more open? Beam 0 of /scan is angle_min; we convert each beam
        index to its bearing and bin it. Bearing 0 = robot forward.
        """
        scan = self._scan
        big = float('inf')
        if scan is None or not scan.ranges:
            return big, big, big

        cone = math.radians(AVOID_CONE_DEG)
        front_wedge = math.radians(FRONT_STOP_DEG)
        front = left = right = big
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max) or math.isinf(r) or math.isnan(r):
                continue
            bearing = scan.angle_min + i * scan.angle_increment
            if abs(bearing) > cone:
                continue                      # outside the forward cone — ignore
            if abs(bearing) <= front_wedge:
                front = min(front, r)
            if bearing > 0:
                left = min(left, r)           # +bearing = left side
            else:
                right = min(right, r)
        return front, left, right

    def _make_handler(self, waypoint_name: str):
        """Build a Trigger callback closure for one waypoint."""
        target = WAYPOINTS[waypoint_name]

        def _handler(req: Trigger.Request, resp: Trigger.Response):
            return self._drive_to(waypoint_name, target, resp)

        return _handler

    def _make_rel_handler(self, motion: str):
        """Build a Trigger callback for a relative move from the current pose."""
        def _handler(req: Trigger.Request, resp: Trigger.Response):
            if not self._wait_for_pose():
                resp.success = False
                resp.message = 'no /odom received — is the sim running?'
                return resp
            if motion in ('forward', 'backward'):
                sign = 1.0 if motion == 'forward' else -1.0
                self.get_logger().info(f'▸ {motion} {STEP_DISTANCE_M:.1f} m')
                return self._drive_distance(sign * STEP_DISTANCE_M, motion, resp)
            else:  # turn_left / turn_right
                delta = STEP_TURN_RAD if motion == 'turn_left' else -STEP_TURN_RAD
                self.get_logger().info(f'▸ {motion} {math.degrees(abs(delta)):.0f}°')
                return self._rotate_by(delta, motion, resp)
        return _handler

    def _drive_distance(self, signed_dist: float, name: str, resp: Trigger.Response) -> Trigger.Response:
        """Drive straight by a RELATIVE distance (m), tracking how far we've
        actually travelled. Stops when the distance is reached, an obstacle
        blocks the path, or a short timeout fires — never hangs on the
        go-to-goal controller fighting a nearby plant.
        """
        x0, y0, _ = self._pose_xy_yaw
        target = abs(signed_dist)
        forward = signed_dist >= 0.0
        # A 1 m move at 0.5 m/s is ~2 s; 12 s is plenty and fails fast if stuck.
        deadline = time.monotonic() + 12.0
        period = 1.0 / CONTROL_RATE_HZ
        while rclpy.ok() and time.monotonic() < deadline:
            x, y, _ = self._pose_xy_yaw
            travelled = math.hypot(x - x0, y - y0)
            if travelled >= target:
                self._cmd_pub.publish(Twist())
                resp.success = True
                resp.message = f'{name} done ({travelled:.2f} m)'
                self.get_logger().info(f'  ✓ {resp.message}')
                return resp
            # When going forward, stop early if something is right ahead.
            if forward:
                front, _, _ = self._clearance()
                if front < 0.5:
                    self._cmd_pub.publish(Twist())
                    resp.success = True   # partial move is still "done", not an error
                    resp.message = (f'{name} stopped after {travelled:.2f} m — '
                                    f'obstacle {front:.2f} m ahead')
                    self.get_logger().info(f'  ⚠ {resp.message}')
                    return resp
            cmd = Twist()
            cmd.linear.x = (MAX_LINEAR_SPEED if forward else -MAX_LINEAR_SPEED) * 0.6
            self._cmd_pub.publish(cmd)
            time.sleep(period)
        self._cmd_pub.publish(Twist())
        x, y, _ = self._pose_xy_yaw
        resp.success = True
        resp.message = f'{name} reached {math.hypot(x - x0, y - y0):.2f} m (timeout)'
        return resp

    def _wait_for_pose(self, timeout_s: float = 3.0) -> bool:
        t0 = time.monotonic()
        while self._pose_xy_yaw is None and (time.monotonic() - t0) < timeout_s:
            time.sleep(0.05)
        return self._pose_xy_yaw is not None

    def _rotate_by(self, delta_yaw: float, name: str, resp: Trigger.Response) -> Trigger.Response:
        """Rotate in place by delta_yaw (rad), then stop. Blocks until done."""
        _, _, start_yaw = self._pose_xy_yaw
        target_yaw = wrap_angle(start_yaw + delta_yaw)
        deadline = time.monotonic() + TIMEOUT_S
        period = 1.0 / CONTROL_RATE_HZ
        while rclpy.ok() and time.monotonic() < deadline:
            _, _, yaw = self._pose_xy_yaw
            err = wrap_angle(target_yaw - yaw)
            if abs(err) < TURN_TOLERANCE_RAD:
                self._cmd_pub.publish(Twist())
                resp.success = True
                resp.message = f'{name} done ({math.degrees(delta_yaw):+.0f}°)'
                self.get_logger().info(f'  ✓ {resp.message}')
                return resp
            cmd = Twist()
            cmd.angular.z = max(-MAX_ANGULAR_SPEED,
                                min(MAX_ANGULAR_SPEED, ANGULAR_GAIN * err))
            self._cmd_pub.publish(cmd)
            time.sleep(period)
        self._cmd_pub.publish(Twist())
        resp.success = False
        resp.message = f'{name} timed out'
        return resp

    def _drive_to(self, name: str, target: tuple, resp: Trigger.Response) -> Trigger.Response:
        """The actual P-controller loop. Blocks until arrived or timeout."""
        # 1. Wait for the first world pose so we know where we are.
        t0 = time.monotonic()
        while self._pose_xy_yaw is None and (time.monotonic() - t0) < 3.0:
            time.sleep(0.05)
        if self._pose_xy_yaw is None:
            resp.success = False
            resp.message = 'no /odom received — is the sim running?'
            return resp

        tx, ty = target
        self.get_logger().info(f'▸ nav_to_{name}  → ({tx:+.2f}, {ty:+.2f})')

        deadline = time.monotonic() + TIMEOUT_S
        period = 1.0 / CONTROL_RATE_HZ

        while rclpy.ok() and time.monotonic() < deadline:
            x, y, yaw = self._pose_xy_yaw
            dx = tx - x
            dy = ty - y
            dist = math.hypot(dx, dy)

            # Arrived?
            if dist < GOAL_TOLERANCE_M:
                self._cmd_pub.publish(Twist())   # stop
                self.get_logger().info(f'  ✓ arrived at {name}  (dist={dist:.2f} m)')
                resp.success = True
                resp.message = f'arrived at {name} (dist={dist:.2f} m)'
                return resp

            # Heading to target — atan2 handles all 4 quadrants correctly.
            target_yaw = math.atan2(dy, dx)
            yaw_err = wrap_angle(target_yaw - yaw)

            # ── Reactive obstacle check (the new bit) ──────────────────────
            front, left, right = self._clearance()

            cmd = Twist()
            if front < AVOID_DISTANCE_M:
                # Something is close, dead-ahead. OVERRIDE go-to-goal: turn
                # toward whichever side is more open. CRUCIALLY we keep
                # moving forward while turning, so we ARC around the obstacle
                # instead of pivoting in place (pure pivoting gets stuck in a
                # reactive local minimum — it just spins). Only when the
                # obstacle is very close (<0.5 m) do we stop and pivot.
                turn = AVOID_TURN_SPEED if left > right else -AVOID_TURN_SPEED
                cmd.angular.z = turn
                cmd.linear.x = 0.18 if front > 0.5 else 0.0
                self.get_logger().info(
                    f'  ⚠ obstacle {front:.2f} m ahead → arcing '
                    f'{"left" if turn > 0 else "right"} '
                    f'(L={left:.2f} R={right:.2f})'
                )
            elif abs(yaw_err) > HEADING_TOLERANCE:
                # No obstacle, but facing wrong way → turn in place first.
                cmd.linear.x  = 0.0
                cmd.angular.z = max(-MAX_ANGULAR_SPEED,
                                    min(MAX_ANGULAR_SPEED, ANGULAR_GAIN * yaw_err))
            else:
                # No obstacle, facing roughly right → drive forward with a
                # small heading correction. If an obstacle is in the cone but
                # not dead-ahead, bias the steering away from the closer side.
                cmd.linear.x  = min(MAX_LINEAR_SPEED, LINEAR_GAIN * dist)
                steer = ANGULAR_GAIN * yaw_err
                if min(left, right) < AVOID_DISTANCE_M:
                    steer += AVOID_TURN_SPEED * 0.5 * (1 if left > right else -1)
                    cmd.linear.x *= 0.5         # slow down near obstacles
                cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, steer))
            self._cmd_pub.publish(cmd)
            time.sleep(period)

        # Timed out — safety stop.
        self._cmd_pub.publish(Twist())
        resp.success = False
        resp.message = f'timed out after {TIMEOUT_S:.0f} s while heading to {name}'
        self.get_logger().warn('  ✗ ' + resp.message)
        return resp


def main():
    rclpy.init()
    node = NavigationServer()
    # MultiThreadedExecutor lets the pose + scan callbacks keep updating
    # the cached state while a service callback is running its drive loop.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    # Final safety stop on shutdown.
    node._cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
