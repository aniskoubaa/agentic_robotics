#!/usr/bin/env python3
"""
RAISE 2026 — gripper server (open / close / rotate).
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

Exposes three LLM-callable services:
    /open_gripper    (std_srvs/Trigger)  → fingers spread to 0 rad
    /close_gripper   (std_srvs/Trigger)  → fingers close to ~0.5 rad
    /rotate_gripper  (std_srvs/Trigger)  → rotates UR5e wrist_3 by +90°

Why this is more than a mock:
- Each callback publishes a target to the per-joint `JointPositionController`
  topics declared in raise2026_gazebo.urdf.xacro.
- The Robotiq 2F-85 URDF declares MIMIC constraints, but DART (Gazebo's
  physics engine) doesn't implement them. So we manually publish the
  correctly-signed target to every joint:

    driver:                    gripper_robotiq_85_left_knuckle_joint
      → right_knuckle           ×  -1
      → left_inner_knuckle      ×  +1
      → right_inner_knuckle     ×  -1
      → left_finger_tip         ×  -1
      → right_finger_tip        ×  +1

Run:
    ros2 run raise2026_tools gripper_server
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


# Robotiq 2F-85 driver joint + mimic signs (extracted from the rendered URDF).
KNUCKLE = 'gripper_robotiq_85_left_knuckle_joint'
MIMIC_SIGNS = {
    'gripper_robotiq_85_right_knuckle_joint':       -1.0,
    'gripper_robotiq_85_left_inner_knuckle_joint':  +1.0,
    'gripper_robotiq_85_right_inner_knuckle_joint': -1.0,
    'gripper_robotiq_85_left_finger_tip_joint':     -1.0,
    'gripper_robotiq_85_right_finger_tip_joint':    +1.0,
}

# Wrist joint we drive for "rotate gripper".
WRIST_3 = 'ur5e_wrist_3_joint'

# Target positions for the driving knuckle joint (radians).
TARGET_OPEN   = 0.0     # fully open (0 rad)
TARGET_CLOSED = 0.5     # roughly closed (within the 0..0.8 limit)

# Per-call wrist rotation step.
ROTATE_STEP   = math.pi / 2     # +90° per call


class GripperServer(Node):
    def __init__(self):
        super().__init__('gripper_server')

        # ── One publisher per joint command topic ──────────────────────────
        # Each topic is fed by a JointPositionController plugin declared in
        # raise2026_gazebo.urdf.xacro.
        self.pub_knuckle = self.create_publisher(Float64, f'/{KNUCKLE}/cmd', 10)
        self.pub_mimic = {
            j: self.create_publisher(Float64, f'/{j}/cmd', 10)
            for j in MIMIC_SIGNS
        }
        self.pub_wrist3 = self.create_publisher(Float64, f'/{WRIST_3}/cmd', 10)
        # Track our current wrist3 target so each rotate adds to it.
        self.wrist3_target = 0.0

        # ── Three services ─────────────────────────────────────────────────
        self.create_service(Trigger, '/open_gripper',   self._on_open)
        self.create_service(Trigger, '/close_gripper',  self._on_close)
        self.create_service(Trigger, '/rotate_gripper', self._on_rotate)

        self.get_logger().info(
            'gripper services ready: /open_gripper /close_gripper /rotate_gripper'
        )

    def _set_gripper(self, knuckle_target: float) -> None:
        """Publish the knuckle target plus correctly-signed mimics."""
        self.pub_knuckle.publish(Float64(data=knuckle_target))
        for joint, sign in MIMIC_SIGNS.items():
            self.pub_mimic[joint].publish(Float64(data=knuckle_target * sign))

    # ── Service callbacks ─────────────────────────────────────────────────
    def _on_open(self, req: Trigger.Request, resp: Trigger.Response):
        self._set_gripper(TARGET_OPEN)
        self.get_logger().info(f'▸ open  knuckle={TARGET_OPEN:.2f} rad')
        resp.success = True
        resp.message = f'gripper opened (knuckle={TARGET_OPEN:.2f} rad)'
        return resp

    def _on_close(self, req: Trigger.Request, resp: Trigger.Response):
        self._set_gripper(TARGET_CLOSED)
        self.get_logger().info(f'▸ close knuckle={TARGET_CLOSED:.2f} rad')
        resp.success = True
        resp.message = f'gripper closed (knuckle={TARGET_CLOSED:.2f} rad)'
        return resp

    def _on_rotate(self, req: Trigger.Request, resp: Trigger.Response):
        # Cumulative: every call rotates wrist_3 by another +90°.
        self.wrist3_target += ROTATE_STEP
        # Keep the target in (-π, π] so it doesn't drift unbounded over many calls.
        if self.wrist3_target > math.pi:
            self.wrist3_target -= 2 * math.pi
        self.pub_wrist3.publish(Float64(data=self.wrist3_target))
        deg = math.degrees(self.wrist3_target)
        self.get_logger().info(f'▸ rotate wrist_3 → {deg:+.0f}°')
        resp.success = True
        resp.message = f'wrist_3 rotated to {deg:+.0f}°'
        return resp


def main():
    rclpy.init()
    node = GripperServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
