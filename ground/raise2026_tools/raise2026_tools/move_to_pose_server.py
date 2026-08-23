#!/usr/bin/env python3
"""
RAISE 2026 — move_to_pose server (UR5e named poses).
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

Exposes one Trigger service per NAMED arm pose:
    /move_to_home          — back to the URDF default (arm up, elbow bent)
    /move_to_above_plant   — wrist pointing forward-down (good for inspection)
    /move_to_side_view     — shoulder rotated 90° (scouting to one side)
    /move_to_stow          — compact tucked pose (for driving)

Pedagogy:
- We deliberately use `std_srvs/Trigger` (the same type as the gripper
  services) instead of a custom .srv with a cartesian Pose. That keeps
  students on familiar ground: each "tool" the LLM can call is a
  single-purpose service with NO arguments. The named-pose abstraction
  also avoids inverse kinematics entirely — pedagogically much cleaner
  for Day 1 than wiring up MoveIt.

Implementation:
- Each callback publishes 6 `std_msgs/Float64` targets, one per UR5e
  joint. The bridge forwards them to the gz `JointPositionController`
  plugins declared in raise2026_gazebo.urdf.xacro, which apply the PID.

Run:
    ros2 run raise2026_tools move_to_pose_server
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


# ── UR5e joint name order ──────────────────────────────────────────────────
# Same order as you'd see in /joint_states. We publish to one /cmd
# topic per joint — those are bridged in raise2026_bringup/config/ros_gz_bridge.yaml.
UR5E_JOINTS = [
    'ur5e_shoulder_pan_joint',
    'ur5e_shoulder_lift_joint',
    'ur5e_elbow_joint',
    'ur5e_wrist_1_joint',
    'ur5e_wrist_2_joint',
    'ur5e_wrist_3_joint',
]

# ── Named poses (radians, in UR5E_JOINTS order) ────────────────────────────
# "home" matches the URDF default in raise2026_gazebo.urdf.xacro so a
# /move_to_home call returns the arm to where it started.
POSES = {
    'home':         [ 0.0,    -math.pi/2,   math.pi/2,    -math.pi/2,   -math.pi/2,  0.0 ],
    'above_plant':  [ 0.0,    -1.0,         0.8,          -1.4,          math.pi/2,  0.0 ],
    'side_view':    [ math.pi/2, -1.0,      1.2,          -math.pi/2,    math.pi/2,  0.0 ],
    'stow':         [ 0.0,    -2.5,         2.5,          -2.0,          0.0,        0.0 ],
}


class MoveToPoseServer(Node):
    def __init__(self):
        super().__init__('move_to_pose_server')

        # One publisher per joint — these match the topic= attribute on
        # each JointPositionController plugin in the URDF.
        self.pubs = {
            j: self.create_publisher(Float64, f'/{j}/cmd', 10)
            for j in UR5E_JOINTS
        }

        # One service per named pose.
        for name in POSES:
            self.create_service(Trigger, f'/move_to_{name}', self._make_handler(name))

        services = ' '.join(f'/move_to_{n}' for n in POSES)
        self.get_logger().info(f'move_to_pose services ready: {services}')

    def _make_handler(self, pose_name: str):
        """Build a Trigger callback closure for one named pose.

        We can't define one method per pose without a lot of repetition,
        so we use a factory: each `pose_name` captures into the inner
        function's closure. The inner function matches the rclpy service
        signature `(req, resp) → resp`.
        """
        targets = POSES[pose_name]

        def _handler(req: Trigger.Request, resp: Trigger.Response):
            # Publish all six joint targets in one tight loop. The PID
            # controllers will start moving as soon as they receive.
            for joint, value in zip(UR5E_JOINTS, targets):
                self.pubs[joint].publish(Float64(data=value))

            # Format degrees for the human-readable response message.
            deg_str = ' '.join(f'{math.degrees(v):+5.0f}°' for v in targets)
            self.get_logger().info(f'▸ move_to_{pose_name}  [{deg_str}]')
            resp.success = True
            resp.message = f'commanded UR5e to "{pose_name}" pose'
            return resp

        return _handler


def main():
    rclpy.init()
    node = MoveToPoseServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
