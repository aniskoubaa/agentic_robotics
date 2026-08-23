# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""Print one line per second of what the vehicle is doing.

The first thing anyone needs when a sim comes up is "is the bridge alive and
what is the aircraft doing?". This answers both, and — importantly — says which
of the two is wrong when nothing arrives.

    ros2 run agr_uav_tools vehicle_monitor
    ros2 run agr_uav_tools vehicle_monitor --ros-args -p namespace:=px4_1
"""
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from px4_msgs.msg import BatteryStatus, VehicleLocalPosition, VehicleStatus

from . import px4_topics
from .px4_topics import PX4_QOS

# Past this, a cached sample is reported as cached rather than as a reading.
STALE_AFTER_S = 5.0

ARMING_STATE = {1: 'DISARMED', 2: 'ARMED'}
NAV_STATE = {
    0: 'MANUAL', 2: 'POSCTL', 3: 'AUTO.MISSION', 4: 'AUTO.LOITER',
    5: 'AUTO.RTL', 10: 'ACRO', 14: 'OFFBOARD', 17: 'AUTO.TAKEOFF',
    18: 'AUTO.LAND',
}


class VehicleMonitor(Node):
    def __init__(self) -> None:
        super().__init__('vehicle_monitor')
        self.declare_parameter('namespace', '')
        self.declare_parameter('period_s', 1.0)
        ns = self.get_parameter('namespace').value

        self._ns = ns
        self._status: VehicleStatus | None = None
        self._pos: VehicleLocalPosition | None = None
        self._batt: BatteryStatus | None = None
        self._status_t = self._pos_t = self._batt_t = 0.0

        # base name -> (msg type, callback); entries are removed once subscribed.
        self._pending = {
            px4_topics.VEHICLE_STATUS: (VehicleStatus, self._on_status),
            px4_topics.LOCAL_POSITION: (VehicleLocalPosition, self._on_pos),
            px4_topics.BATTERY_STATUS: (BatteryStatus, self._on_batt),
        }
        self._warned = False

        # DO NOT resolve-and-subscribe once in __init__. PX4 takes ~10 s to boot
        # and only creates its DDS data writers at the end of that; a monitor
        # started by the same launch file comes up in ~0.2 s and would find
        # nothing, warn, and then sit silent forever even after the topics
        # appear. Retry on every tick until each one is found.
        self._try_subscribe()

        self.create_timer(float(self.get_parameter('period_s').value), self._tick)

    def _try_subscribe(self) -> None:
        """Subscribe to whatever is advertised now; leave the rest pending."""
        for base in list(self._pending):
            msg_type, cb = self._pending[base]
            topic = px4_topics.resolve(self, base, 'out', self._ns)
            if topic is None:
                continue
            self.create_subscription(msg_type, topic, cb, PX4_QOS)
            self.get_logger().info(f'subscribed: {topic}')
            del self._pending[base]

    # Stamp every arrival. PX4 publishes vehicle_status ON CHANGE, so on an
    # idle DISARMED vehicle the last one may be minutes old — and a display
    # that keeps reprinting it looks exactly like live telemetry. Showing the
    # age is the difference between "the aircraft is DISARMED" and "the
    # aircraft was DISARMED when I last heard, 4 minutes ago".
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_status(self, msg):
        self._status, self._status_t = msg, self._now()

    def _on_pos(self, msg):
        self._pos, self._pos_t = msg, self._now()

    def _on_batt(self, msg):
        self._batt, self._batt_t = msg, self._now()

    def _tick(self) -> None:
        if self._pending:
            self._try_subscribe()
        if self._status is None:
            if self._pending and not self._warned:
                # Say it once, with the diagnosis — not every second.
                self.get_logger().info(
                    f"waiting for PX4 to advertise "
                    f"{', '.join(sorted(self._pending))} "
                    f"(namespace: {self._ns or 'none, instance 0'})… "
                    f"PX4 takes ~10 s to boot.")
                self._warned = True
            return
        now = self._now()
        arm = ARMING_STATE.get(self._status.arming_state, f'?{self._status.arming_state}')
        nav = NAV_STATE.get(self._status.nav_state, f'nav={self._status.nav_state}')

        # vehicle_status is on-change; anything older than this is a cached
        # value, not a reading.
        age = now - self._status_t
        stale = f'  [state {age:.0f}s old]' if age > STALE_AFTER_S else ''

        pos = 'pos —'
        if self._pos is not None and self._pos.xy_valid:
            pos = f'x={self._pos.x:6.1f} y={self._pos.y:6.1f} z={self._pos.z:6.1f}'
        batt = 'batt —'
        if self._batt is not None:
            if now - self._batt_t > STALE_AFTER_S:
                batt = f'batt {self._batt.remaining * 100:5.1f}% (stale)'
            else:
                batt = f'batt {self._batt.remaining * 100:5.1f}%'
        self.get_logger().info(f'{arm:8s} {nav:14s} {pos}  {batt}{stale}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Shutdown race: when SIGTERM lands while the executor is mid
        # wait-set init, rcl raises RCLError instead of the tidy
        # ExternalShutdownException, and the node exits with a traceback that
        # looks like a crash. Which node loses this race varies run to run.
        # If the context is already down we are unwinding anyway — swallow it.
        # If it is still up, this is a genuine fault: re-raise untouched.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()   # idempotent: bare shutdown() raises if already down
