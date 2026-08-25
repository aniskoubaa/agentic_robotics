#!/usr/bin/env python3
"""
Go2 example 05 — make the dog walk.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_legged_examples 05_walk
    ros2 run agr_legged_examples 05_walk --ros-args -p vx:=0.15 -p seconds:=6.0

WHAT  Publish a Twist on /cmd_vel and the robot trots. That is the whole
      script — eight lines of actual work.

LEARN - THIS IS THE POINT OF THE WHOLE STACK. Walking is a hard problem, and
        yet from here it looks exactly like driving the wheeled RaiseBot:
        publish linear.x, the robot moves. The difficulty was not removed, it
        was moved — into agr_legged_bringup/gait.py, behind an interface that
        every mobile robot already speaks.
      - /cmd_vel must be published REPEATEDLY. The gait controller stops if it
        goes quiet for half a second, which is what keeps the robot from
        trotting off when this script is Ctrl-C'd.
      - linear.y makes it STRAFE — walk sideways without turning. Try it. A
        wheeled base cannot do that at any speed.
      - Above about 0.25 m/s the robot falls over rather than going faster.
        The gait clamps it for you; see the measured table in gait.py.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

RATE_HZ = 20


def main():
    rclpy.init()
    node = Node('walk')
    for n, d in (('vx', 0.20), ('vy', 0.0), ('wz', 0.0), ('seconds', 5.0)):
        node.declare_parameter(n, d)
    vx, vy, wz, secs = (float(node.get_parameter(n).value)
                        for n in ('vx', 'vy', 'wz', 'seconds'))

    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    cmd = Twist()
    cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz

    print(f'walking: vx={vx:+.2f} m/s  vy={vy:+.2f} m/s  wz={wz:+.2f} rad/s '
          f'for {secs:.1f}s')
    for _ in range(int(secs * RATE_HZ)):
        pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)

    # Always stop. If this script dies mid-walk the gait controller's own
    # idle timeout catches it, but never rely on someone else's safety net.
    pub.publish(Twist())
    print('stopped')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
