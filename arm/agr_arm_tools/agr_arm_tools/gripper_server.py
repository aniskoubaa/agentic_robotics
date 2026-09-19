#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
AGR Arm — gripper server.

Three services, all std_srvs/Trigger, all with no arguments:

    /open_gripper     fingers all the way apart (135 mm between the tips)
    /close_gripper    fingers together, or onto whatever is between them
    /rotate_gripper   spin wrist_3 by +90 deg, to line the fingers up

    ros2 run agr_arm_tools gripper_server
    ros2 service call /close_gripper std_srvs/srv/Trigger

WHY TRIGGER AND NOT A WIDTH
    Deliberately argument-free, exactly like the ground platform's gripper.
    Each callable skill is one service with no parameters, which is the shape
    an LLM planner can be handed a list of and use without inventing units.
    If you want a specific opening, publish to the joint topics yourself or
    use Arm.set_gripper() — this is the coarse, safe interface.

WHY SIX PUBLISHERS FOR ONE GRIPPER
    The Robotiq 2F-85 URDF drives five joints off the left knuckle with
    <mimic> tags, and DART does not implement mimic constraints. So the
    server publishes to every finger joint individually, each with the sign
    its mimic multiplier implies. Get a sign wrong and the fingers pull
    against each other instead of closing.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from agr_arm_tools.arm import (GRIPPER_CLOSED, GRIPPER_OPEN, GRIPPER_SIGNS,
                               KNUCKLE, TIP_SEPARATION_MM_AT_ZERO,
                               TIP_SEPARATION_MM_PER_RAD)

WRIST_3 = 'ur5e_wrist_3_joint'
ROTATE_STEP = math.pi / 2


class GripperServer(Node):
    def __init__(self):
        super().__init__('gripper_server')

        self.pubs = {j: self.create_publisher(Float64, f'/{j}/cmd', 10)
                     for j in GRIPPER_SIGNS}
        self.pub_wrist3 = self.create_publisher(Float64, f'/{WRIST_3}/cmd', 10)
        self.wrist3_target = 0.0
        self.state = {}
        self.create_subscription(
            JointState, '/joint_states',
            lambda m: self.state.update(zip(m.name, m.position)), 10)

        self.create_service(Trigger, '/open_gripper', self._on_open)
        self.create_service(Trigger, '/close_gripper', self._on_close)
        self.create_service(Trigger, '/rotate_gripper', self._on_rotate)
        self.get_logger().info(
            'gripper services ready: /open_gripper /close_gripper /rotate_gripper')

    def _set(self, knuckle: float) -> None:
        for joint, sign in GRIPPER_SIGNS.items():
            self.pubs[joint].publish(Float64(data=knuckle * sign))

    def _separation_mm(self) -> float:
        k = self.state.get(KNUCKLE)
        if k is None:
            return float('nan')
        return TIP_SEPARATION_MM_AT_ZERO + TIP_SEPARATION_MM_PER_RAD * k

    def _on_open(self, req, resp):
        self._set(GRIPPER_OPEN)
        self.get_logger().info(f'open   knuckle -> {GRIPPER_OPEN:.2f} rad')
        resp.success = True
        resp.message = f'gripper opening (knuckle {GRIPPER_OPEN:.2f} rad)'
        return resp

    def _on_close(self, req, resp):
        self._set(GRIPPER_CLOSED)
        self.get_logger().info(f'close  knuckle -> {GRIPPER_CLOSED:.2f} rad')
        resp.success = True
        # The reply cannot say whether anything was gripped: the fingers have
        # not moved yet when this returns, and "gripped" means they STALLED,
        # which takes a second or so to become true. Read /joint_states, or
        # ask Arm.holding_something, after giving it that second.
        resp.message = (f'gripper closing (knuckle {GRIPPER_CLOSED:.2f} rad); '
                        f'tips were {self._separation_mm():.0f} mm apart')
        return resp

    def _on_rotate(self, req, resp):
        self.wrist3_target += ROTATE_STEP
        if self.wrist3_target > math.pi:
            self.wrist3_target -= 2 * math.pi
        self.pub_wrist3.publish(Float64(data=self.wrist3_target))
        deg = math.degrees(self.wrist3_target)
        self.get_logger().info(f'rotate wrist_3 -> {deg:+.0f} deg')
        resp.success = True
        resp.message = f'wrist_3 rotating to {deg:+.0f} deg'
        return resp


def main():
    rclpy.init()
    node = GripperServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Shutdown race: SIGTERM landing while the executor is mid wait-set
        # init makes rcl raise RCLError instead of the tidy
        # ExternalShutdownException, and the node exits with a traceback that
        # looks like a crash. If the context is already down we are unwinding
        # anyway — swallow it. If it is still up, this is real: re-raise.
        if rclpy.ok():
            raise
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
