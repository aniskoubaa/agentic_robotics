#!/usr/bin/env python3
"""
UAV — diagnostics. Why won't it fly?
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_uav_demos diagnose
    ros2 run agr_uav_demos diagnose --ros-args -p active:=true   # try arming

WHAT  Checks the PX4 bridge, telemetry, estimator health and arming blockers,
      in the order they actually break, each with what to do about it.

WHY   "It won't arm" has about eight distinct causes and PX4 announces none of
      them on a ROS topic by default. Worse, the topic that WOULD tell you —
      vehicle_status — publishes only on change, so on an idle vehicle it says
      nothing at all and a naive check reports a healthy system as dead. This
      reads failsafe_flags instead, which streams continuously and names the
      specific blocker.

Exit code 0 when everything passed, 1 otherwise.
"""
from __future__ import annotations

import sys
import time

import rclpy
from px4_msgs.msg import (BatteryStatus, FailsafeFlags, VehicleControlMode,
                          VehicleLandDetected, VehicleOdometry, VehicleStatus)
from rclpy.node import Node
from sensor_msgs.msg import Image

from agr_uav_tools import px4_topics

OK, BAD, WARN, INFO = '  OK  ', ' FAIL ', ' WARN ', ' INFO '

# Flags that stop a multirotor arming or flying, and what each one means in
# SITL. Anything not listed here is reported generically.
BLOCKERS = {
    'angular_velocity_invalid': 'gyro not healthy',
    'attitude_invalid':         'attitude estimate not converged — give EKF2 a few more seconds',
    'local_position_invalid':   'no local position estimate — EKF2 has not converged',
    'local_altitude_invalid':   'no altitude estimate — usually a missing barometer',
    'global_position_invalid':  'no global position — GPS not fused yet',
    'home_position_invalid':    'home not set — PX4 sets it once the estimate is valid',
    'battery_unhealthy':        'battery not healthy (CBRK_SUPPLY_CHK not set?)',
    'gcs_connection_lost':      'no ground station — only blocks arming if NAV_DLL_ACT > 0',
    'fd_esc_arming_failure':    'ESCs failed to arm',
    'fd_motor_failure':         'motor failure detected',
}


class Diagnose(Node):
    def __init__(self, namespace: str = '') -> None:
        super().__init__('uav_diagnose')
        self.ns = namespace
        self.failures = 0
        self.step = 0

    def report(self, tag: str, title: str, detail: str = '', fix: str = '') -> None:
        self.step += 1
        print(f'[{tag}] {self.step}. {title}')
        if detail:
            print(f'        {detail}')
        if tag == BAD:
            self.failures += 1
        if fix and tag in (BAD, WARN):
            print(f'        → {fix}')

    def collect(self, msg_type, topic, seconds=3.0, qos=None) -> list:
        """Every message on `topic` within `seconds`.

        `qos` defaults to PX4's profile, which is right for /fmu/* and WRONG
        for anything else. Requesting TRANSIENT_LOCAL from a VOLATILE
        publisher — which is what ros_gz_bridge is — is INCOMPATIBLE, and the
        subscription then receives nothing at all. This diagnostic reported
        'no /camera' on a working camera until that was fixed.
        """
        got: list = []
        sub = self.create_subscription(msg_type, topic, got.append,
                                       px4_topics.PX4_QOS if qos is None else qos)
        end = time.time() + seconds
        try:
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
        finally:
            self.destroy_subscription(sub)
        return got

    def settle(self, seconds: float = 2.0) -> None:
        """Let DDS discovery finish before asking what exists.

        get_topic_names_and_types() reports what THIS node has discovered so
        far, and discovery is not instant. Calling it on the first line of a
        check reliably reports a live system as having no topics at all — the
        most misleading possible diagnostic output.
        """
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self, active: bool) -> int:
        pre = px4_topics.prefix(self.ns)
        print('\nUAV diagnostics\n' + '=' * 66)
        self.settle()

        # 1. Is the uXRCE-DDS bridge up at all? Without it, PX4 may be running
        #    perfectly and be completely invisible to ROS.
        fmu = [t for t, _ in self.get_topic_names_and_types()
               if t.startswith(f'{pre}/out/')]
        if fmu:
            self.report(OK, 'PX4 bridge is up', f'{len(fmu)} /fmu/out topics')
        else:
            self.report(BAD, 'PX4 bridge is up', 'no /fmu/out topics at all',
                        'start everything:  agr-sim   (this also starts '
                        'MicroXRCEAgent)')
            print('\nNothing below can pass without the bridge. Stopping.\n')
            return 1

        # 2. Odometry — the one position source that streams reliably.
        odom = self.collect(VehicleOdometry, f'{pre}/out/vehicle_odometry', 2.0)
        if odom:
            n, e, d = odom[-1].position
            self.report(OK, 'Position telemetry',
                        f'{len(odom) / 2.0:.0f} Hz, '
                        f'n={n:.1f} e={e:.1f} alt={-d:.1f} m')
        else:
            self.report(BAD, 'Position telemetry', 'no vehicle_odometry',
                        'PX4 is not running, or the XRCE agent lost its client')

        # 3. Arming state, from the topic that actually streams.
        cm = self.collect(VehicleControlMode, f'{pre}/out/vehicle_control_mode', 3.0)
        if cm:
            m = cm[-1]
            self.report(OK, 'Control mode telemetry',
                        f'{len(cm) / 3.0:.1f} Hz — '
                        f'{"ARMED" if m.flag_armed else "disarmed"}, '
                        f'offboard {"ON" if m.flag_control_offboard_enabled else "off"}')
        else:
            self.report(BAD, 'Control mode telemetry',
                        'no vehicle_control_mode — cannot tell if it is armed')

        # 4. vehicle_status. Its ABSENCE is normal; say so rather than failing.
        st_topic = px4_topics.resolve(self, px4_topics.VEHICLE_STATUS, 'out', self.ns)
        st = self.collect(VehicleStatus, st_topic, 3.0) if st_topic else []
        if st:
            s = st[-1]
            self.report(OK, 'Vehicle status',
                        f'nav_state={s.nav_state}, '
                        f'pre-flight checks '
                        f'{"PASS" if s.pre_flight_checks_pass else "FAIL"}')
        elif st_topic:
            self.report(INFO, 'Vehicle status',
                        f'{st_topic} exists but sent nothing in 3 s — EXPECTED. '
                        f'It publishes only on change, and the bridge does not '
                        f'replay the last sample to a late subscriber.')
        else:
            self.report(WARN, 'Vehicle status', 'topic not found at all')

        # 5. THE important one: what is actually blocking flight.
        ff = self.collect(FailsafeFlags, f'{pre}/out/failsafe_flags', 3.0)
        if not ff:
            self.report(WARN, 'Failsafe flags', 'no failsafe_flags')
        else:
            f = ff[-1]
            raised = [name for name in BLOCKERS if getattr(f, name, False)]
            if not raised:
                self.report(OK, 'Nothing is blocking flight',
                            'estimator converged, no failsafe flags raised')
            else:
                detail = '; '.join(f'{n} ({BLOCKERS[n]})' for n in raised)
                # gcs_connection_lost alone is harmless here — the bringup sets
                # NAV_DLL_ACT=0 precisely so SITL can fly without a GCS.
                only_gcs = raised == ['gcs_connection_lost']
                self.report(WARN if only_gcs else BAD,
                            'Nothing is blocking flight', detail,
                            'harmless: NAV_DLL_ACT=0 is set by the bringup'
                            if only_gcs else
                            'wait ~30 s for EKF2 to converge, then re-run')

        # 6. Battery — SITL reports 100%, so anything else is a real signal.
        bat_topic = px4_topics.resolve(self, px4_topics.BATTERY_STATUS, 'out', self.ns)
        bat = self.collect(BatteryStatus, bat_topic, 2.0) if bat_topic else []
        if bat:
            self.report(OK, 'Battery',
                        f'{bat[-1].remaining * 100:.0f}%, '
                        f'{bat[-1].voltage_v:.1f} V')
        else:
            self.report(WARN, 'Battery', 'no battery_status')

        # 7. On the ground or in the air?
        ld = self.collect(VehicleLandDetected, f'{pre}/out/vehicle_land_detected', 2.5)
        if ld:
            self.report(OK, 'Land detector',
                        'LANDED' if ld[-1].landed else 'IN THE AIR')
        else:
            self.report(WARN, 'Land detector', 'no vehicle_land_detected')

        # 8. Camera — informational. Three of the four airframes have none.
        # Plain QoS 10, not PX4_QOS: this one comes from ros_gz_bridge.
        img = self.collect(Image, '/camera', 2.0, qos=10)
        if img:
            self.report(OK, 'Camera',
                        f'{img[-1].width}x{img[-1].height}, '
                        f'{len(img) / 2.0:.1f} Hz')
        else:
            self.report(INFO, 'Camera',
                        'no /camera — expected unless the airframe declares '
                        'one (agr-sim airframe:=x500_mono_cam)')

        # 9. Optional: prove the command path end to end.
        if active:
            print('\n  --- active test: arming and disarming ---')
            from agr_uav_tools.offboard import Pilot
            pilot = Pilot(self, self.ns)
            pilot.wait_for_telemetry()
            was_armed = pilot.armed
            # Setpoints must be streaming BEFORE the arm request, and must
            # keep streaming while PX4 considers it. Arming with a bare
            # command works from AUTO.LOITER but is refused in offboard mode
            # — and the vehicle is often already in offboard after a previous
            # flight. Streaming a hold-here setpoint works from either.
            pos = pilot.position() or (0.0, 0.0, 0.0)
            pilot.enter_offboard((pos[0], pos[1], 0.0))
            pilot.arm()
            for _ in range(50):
                pilot.send_setpoint(pos[0], pos[1], 0.0)
                pilot.spin(0.1)
                if pilot.armed:
                    break
            if pilot.armed:
                self.report(OK, 'Command path (arm)', 'PX4 accepted the request')
                if not was_armed:
                    pilot.disarm()
                    pilot.spin(2.0)
            else:
                self.report(BAD, 'Command path (arm)',
                            'PX4 did not arm within 5 s',
                            'check the blockers in step 5')
        else:
            self.report(INFO, 'Command path',
                        'not tested — re-run with -p active:=true to actually '
                        'arm the vehicle')

        print('=' * 66)
        if self.failures:
            print(f'{self.failures} check(s) FAILED — see the → lines above.\n')
        else:
            print('All checks passed. Try:\n'
                  '    ros2 run agr_uav_examples 03_takeoff_and_land\n')
        return 1 if self.failures else 0


def main() -> int:
    rclpy.init()
    node = Diagnose()
    node.declare_parameter('active', False)
    node.declare_parameter('namespace', '')
    node.ns = node.get_parameter('namespace').value
    try:
        code = node.run(bool(node.get_parameter('active').value))
    except KeyboardInterrupt:
        code = 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
