# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""Offboard flight control for PX4, small enough to read in one sitting.

Every UAV teleop, example and demo in this repo goes through `Pilot`. Flying a
PX4 vehicle from ROS 2 is not hard, but it has four rules that are not
discoverable and each one fails silently when broken:

1. SETPOINTS BEFORE MODE. PX4 refuses to enter offboard mode unless setpoints
   are ALREADY streaming. Send the mode request first and it is rejected, with
   no error on any ROS topic. Stream for ~1 s, then ask.

2. NEVER STOP STREAMING. If setpoints stop arriving for ~0.5 s, PX4 drops out
   of offboard into failsafe. This is a safety feature and it is why every
   loop below publishes on every iteration, even while "just hovering".

3. NED, NOT ENU. PX4 positions are North-East-DOWN. Up is NEGATIVE z. A
   takeoff to 2.5 m is z = -2.5. ROS conventions are the other way up, and
   mixing them is how a takeoff command flies into the ground.

4. THE QoS MUST MATCH. PX4 publishes BEST_EFFORT; a default (RELIABLE)
   subscription never matches and never receives, which is indistinguishable
   from a dead bridge. See px4_topics.PX4_QOS.

And one trap that cost real time to find: vehicle_status PUBLISHES ONLY ON
CHANGE, and the XRCE-DDS agent does not replay the latched sample to a late
subscriber. So a node that starts after the vehicle has settled waits forever
for a "status" that a perfectly healthy PX4 has no reason to send. MEASURED:
`ros2 topic hz /fmu/out/vehicle_status_v4` returns nothing on an idle
vehicle, while vehicle_control_mode streams at 2 Hz and carries the same
arming flag. Armed state therefore comes from vehicle_control_mode here, and
vehicle_status is used only for the extras it alone has.

Everything else here is bookkeeping.
"""
from __future__ import annotations

import math
import time

import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleControlMode, VehicleOdometry,
                          VehicleStatus)
from rclpy.node import Node

from agr_uav_tools import px4_topics

# PX4 command ids we use. Named, because `command = 400` in flight code is how
# reviewers stop reading.
CMD_ARM_DISARM = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM   # 400
CMD_SET_MODE = VehicleCommand.VEHICLE_CMD_DO_SET_MODE              # 176
CMD_LAND = VehicleCommand.VEHICLE_CMD_NAV_LAND                     # 21

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0     # param2 of DO_SET_MODE

SETPOINT_HZ = 20.0        # comfortably above PX4's ~2 Hz minimum
MODE_WARMUP_S = 1.0       # how long to stream setpoints before asking for mode


class Pilot:
    """Offboard control of one PX4 vehicle.

    Wraps a Node rather than subclassing it, so a script can own its own node
    and still use this — which is what the examples do.
    """

    def __init__(self, node: Node, namespace: str = '') -> None:
        self.node = node
        self.ns = namespace
        pre = px4_topics.prefix(namespace)
        qos = px4_topics.PX4_QOS

        self._pub_mode = node.create_publisher(
            OffboardControlMode, f'{pre}/in/offboard_control_mode', qos)
        self._pub_sp = node.create_publisher(
            TrajectorySetpoint, f'{pre}/in/trajectory_setpoint', qos)
        self._pub_cmd = node.create_publisher(
            VehicleCommand, f'{pre}/in/vehicle_command', qos)

        self.status: VehicleStatus | None = None
        self.control_mode: VehicleControlMode | None = None
        self.odom: VehicleOdometry | None = None

        # vehicle_status carries a version suffix that changes between PX4
        # releases; resolve it rather than hard-coding. It may not be
        # discoverable yet at construction time, so retry on first use.
        self._status_sub = None
        self._try_subscribe_status()
        node.create_subscription(VehicleOdometry, f'{pre}/out/vehicle_odometry',
                                 self._on_odom, qos)
        # The dependable one: streams at ~2 Hz whatever the vehicle is doing,
        # so a late subscriber still learns the arming state.
        node.create_subscription(VehicleControlMode,
                                 f'{pre}/out/vehicle_control_mode',
                                 self._on_control_mode, qos)

    # ── plumbing ──────────────────────────────────────────────────────────
    def _try_subscribe_status(self) -> bool:
        if self._status_sub is not None:
            return True
        topic = px4_topics.resolve(self.node, px4_topics.VEHICLE_STATUS,
                                   'out', self.ns)
        if topic is None:
            return False
        self._status_sub = self.node.create_subscription(
            VehicleStatus, topic, self._on_status, px4_topics.PX4_QOS)
        return True

    def _on_status(self, msg: VehicleStatus) -> None:
        self.status = msg

    def _on_control_mode(self, msg: VehicleControlMode) -> None:
        self.control_mode = msg

    def _on_odom(self, msg: VehicleOdometry) -> None:
        self.odom = msg

    def _stamp(self) -> int:
        """PX4 timestamps are microseconds since boot. PX4 only checks that
        they move forward, so the node clock is fine."""
        return int(self.node.get_clock().now().nanoseconds / 1000)

    def spin(self, seconds: float) -> None:
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    # ── state ─────────────────────────────────────────────────────────────
    @property
    def armed(self) -> bool:
        if self.control_mode is not None:
            return bool(self.control_mode.flag_armed)
        return bool(self.status
                    and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    @property
    def offboard_active(self) -> bool:
        """True once PX4 has ACCEPTED offboard mode. Asking for it is not the
        same as getting it, and the rejection is silent."""
        return bool(self.control_mode
                    and self.control_mode.flag_control_offboard_enabled)

    @property
    def preflight_ok(self) -> bool | None:
        """None when vehicle_status has never arrived — which is normal on an
        idle vehicle, and must not be reported as a failed check."""
        if self.status is None:
            return None
        return bool(self.status.pre_flight_checks_pass)

    def position(self) -> tuple | None:
        """(north, east, up) in metres, or None. Note UP — this converts out
        of PX4's NED so that callers never have to think about the sign."""
        if self.odom is None:
            return None
        n, e, d = self.odom.position
        return (float(n), float(e), -float(d))

    def yaw(self) -> float:
        """Heading in radians. PX4 odometry gives (w, x, y, z), not (x,y,z,w)
        — a field-order trap that silently returns a plausible wrong angle."""
        if self.odom is None:
            return 0.0
        w, x, y, z = (float(v) for v in self.odom.q)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def wait_for_telemetry(self, timeout_s: float = 15.0) -> bool:
        """Block until PX4 is demonstrably alive. False on timeout.

        Proof of life is odometry or control_mode, NOT vehicle_status —
        waiting on vehicle_status hangs forever on a healthy idle vehicle
        (see the module docstring).
        """
        end = time.time() + timeout_s
        while rclpy.ok() and time.time() < end:
            self._try_subscribe_status()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.odom is not None or self.control_mode is not None:
                return True
        return False

    # ── commands ──────────────────────────────────────────────────────────
    def send_command(self, command: int, **params) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._stamp()
        msg.command = command
        for i in range(1, 8):
            setattr(msg, f'param{i}', float(params.get(f'param{i}', 0.0)))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._pub_cmd.publish(msg)

    def send_setpoint(self, x: float, y: float, up: float,
                      yaw: float = 0.0, velocity: tuple | None = None,
                      yawspeed: float | None = None) -> None:
        """One position (or velocity) setpoint. `up` is metres UP; the NED
        conversion happens here so no caller has to remember it."""
        mode = OffboardControlMode()
        mode.timestamp = self._stamp()
        mode.position = velocity is None
        mode.velocity = velocity is not None
        self._pub_mode.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = mode.timestamp
        nan = float('nan')
        if velocity is None:
            sp.position = [float(x), float(y), float(-up)]
            sp.velocity = [nan, nan, nan]
        else:
            # Unused fields must be NaN, not 0.0. A zero position setpoint is
            # a valid instruction to fly to the origin, and PX4 will obey it.
            sp.position = [nan, nan, nan]
            vn, ve, vup = velocity
            sp.velocity = [float(vn), float(ve), float(-vup)]
        sp.acceleration = [nan, nan, nan]
        sp.jerk = [nan, nan, nan]
        # yaw and yawspeed are alternatives, not a pair. Send NaN for the one
        # you are not using: a yaw of 0.0 is a real instruction to point north.
        if yawspeed is None:
            sp.yaw = float(yaw)
            sp.yawspeed = 0.0
        else:
            sp.yaw = nan
            sp.yawspeed = float(yawspeed)
        self._pub_sp.publish(sp)

    def arm(self) -> None:
        self.send_command(CMD_ARM_DISARM, param1=1.0)

    def disarm(self) -> None:
        self.send_command(CMD_ARM_DISARM, param1=0.0)

    def land(self) -> None:
        self.send_command(CMD_LAND)

    def enter_offboard(self, hold: tuple = (0.0, 0.0, 0.0)) -> None:
        """Stream setpoints, then request offboard mode. Order matters — see
        rule 1 in the module docstring."""
        x, y, up = hold
        end = time.time() + MODE_WARMUP_S
        while rclpy.ok() and time.time() < end:
            self.send_setpoint(x, y, up)
            rclpy.spin_once(self.node, timeout_sec=1.0 / SETPOINT_HZ)
        self.send_command(CMD_SET_MODE, param1=1.0,
                          param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD)

    def takeoff(self, altitude_m: float = 2.5, timeout_s: float = 30.0) -> bool:
        """Arm, enter offboard, and climb. True once within 0.5 m of target.

        Returns rather than raises on failure: a script that cannot take off
        should print why and land, not unwind through a traceback with the
        vehicle still armed.
        """
        if not self.wait_for_telemetry():
            self.node.get_logger().error(
                'no telemetry from PX4 — is the sim running?')
            return False

        start = self.position() or (0.0, 0.0, 0.0)
        self.enter_offboard((start[0], start[1], altitude_m))
        self.arm()

        end = time.time() + timeout_s
        while rclpy.ok() and time.time() < end:
            self.send_setpoint(start[0], start[1], altitude_m)
            rclpy.spin_once(self.node, timeout_sec=1.0 / SETPOINT_HZ)
            pos = self.position()
            if pos and abs(pos[2] - altitude_m) < 0.3:
                self.hold(start[0], start[1], altitude_m, 1.5)
                return True
            # PX4 can reject the first arm request while it is still running
            # its own pre-flight checks. Re-ask rather than giving up.
            if not self.armed:
                self.arm()
        return False

    def hold(self, x: float, y: float, up: float, seconds: float,
             yaw: float = 0.0) -> None:
        """Sit at a setpoint. Not a sleep — the stream must not stop."""
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            self.send_setpoint(x, y, up, yaw)
            rclpy.spin_once(self.node, timeout_sec=1.0 / SETPOINT_HZ)

    def goto(self, x: float, y: float, up: float, yaw: float = 0.0,
             tolerance: float = 0.25, settle_s: float = 1.5,
             timeout_s: float = 40.0) -> bool:
        """Fly to a point, then SETTLE there. False on timeout.

        The settle is not politeness. Returning the instant the vehicle first
        crosses the tolerance sphere means every error you measure afterwards
        is exactly the tolerance — you are reporting your own threshold back
        to yourself, not the vehicle's accuracy. Holding the setpoint for a
        moment lets the position controller actually converge.
        """
        end = time.time() + timeout_s
        while rclpy.ok() and time.time() < end:
            self.send_setpoint(x, y, up, yaw)
            rclpy.spin_once(self.node, timeout_sec=1.0 / SETPOINT_HZ)
            pos = self.position()
            if pos and math.dist(pos, (x, y, up)) < tolerance:
                self.hold(x, y, up, settle_s, yaw)
                return True
        return False
