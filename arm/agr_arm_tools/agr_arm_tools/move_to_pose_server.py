#!/usr/bin/env python3
# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
AGR Arm — named arm poses, one Trigger service each.

    /move_to_home       the URDF's start pose, 0.30 m above the bench
    /move_to_ready      hovering over the row of blocks, ready to descend
    /move_to_watch      high and back, so the wrist camera sees the whole bench
    /move_to_stow       folded in over the pedestal, out of everyone's way

    ros2 run agr_arm_tools move_to_pose_server
    ros2 service call /move_to_ready std_srvs/srv/Trigger

WHY NAMED POSES WHEN THERE IS AN IK SOLVER NEXT DOOR
    Because they are different tools for different jobs, and knowing which to
    reach for is half of learning manipulation.

    A named pose is a fixed configuration with no arguments. It always works,
    always takes the same path, and cannot be asked for something impossible.
    That makes it the right primitive for the parts of a task that never
    change — go to the start, get out of the way, park.

    IK (agr_arm_tools.kinematics, or Arm.move_to) is for the parts that
    depend on where the object is. It can fail, and it should be asked
    whether a point is reachable before being told to go there.

    The poses below are stored as JOINT ANGLES, not as Cartesian points that
    get solved at start-up. A named pose that fails to load because the
    solver did not converge would defeat the entire purpose of having one.

Each callback publishes six Float64 targets and returns immediately — it does
NOT wait for the arm to arrive. Waiting inside a service callback blocks the
executor, and a caller who wants to know when the arm got there can watch
/joint_states, which is the honest way to find out anyway.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from agr_arm_tools import kinematics as K

# Named poses, in degrees because that is how you read them off a robot
# teach pendant, converted once below. The tool position each one produces
# was measured with kinematics.fk() and is quoted so you can picture it
# without running anything.
POSES_DEG = {
    # tcp (0.560, 0.133, 1.048) — matches <initial_position> in the URDF
    'home':  (0.0,  -80.0,  90.0, -100.0, -90.0,   0.0),
    # tcp (0.450, 0.130, 0.950) — 0.17 m above the blocks, tool pointing down
    'ready': (0.0,  -90.0, 117.0, -117.0, -90.0, -90.0),
    # tcp (0.300, 0.130, 1.150) — high and back; the whole bench is in frame
    'watch': (0.0, -116.0, 106.0,  -80.0, -90.0, -90.0),
    # tcp (0.227, 0.133, 0.870) — folded back over the pedestal
    'stow':  (0.0, -120.0, 150.0, -120.0, -90.0,   0.0),
}
POSES = {name: tuple(math.radians(d) for d in degs)
         for name, degs in POSES_DEG.items()}


class MoveToPoseServer(Node):
    def __init__(self):
        super().__init__('move_to_pose_server')
        self.pubs = {j: self.create_publisher(Float64, f'/{j}/cmd', 10)
                     for j in K.JOINTS}
        for name in POSES:
            self.create_service(Trigger, f'/move_to_{name}', self._handler(name))
        names = ' '.join(f'/move_to_{n}' for n in POSES)
        self.get_logger().info(f'named-pose services ready: {names}')

    def _handler(self, name: str):
        """Build one Trigger callback per pose.

        A factory rather than four near-identical methods: `name` is captured
        in the closure, and the inner function has the (req, resp) -> resp
        shape rclpy wants.
        """
        targets = POSES[name]

        def callback(req: Trigger.Request, resp: Trigger.Response):
            for joint, value in zip(K.JOINTS, targets):
                self.pubs[joint].publish(Float64(data=value))
            p = K.tcp(targets)
            self.get_logger().info(
                f'move_to_{name}  [' +
                ' '.join(f'{math.degrees(v):+5.0f}' for v in targets) +
                f' ] deg  -> tcp ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})')
            resp.success = True
            resp.message = (f'commanded "{name}"; grasp point will be '
                            f'({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m')
            return resp

        return callback


def main():
    rclpy.init()
    node = MoveToPoseServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
