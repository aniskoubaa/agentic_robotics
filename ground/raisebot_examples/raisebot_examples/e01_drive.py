#!/usr/bin/env python3
"""
RaiseBot example 01 — drive the Husky.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run raisebot_examples 01_drive
    ros2 run raisebot_examples 01_drive --ros-args -p vx:=0.3 -p seconds:=4.0

WHAT  Publish a Twist on /cmd_vel for a fixed time, then stop.

LEARN - A `Twist` is TWO 3-vectors: linear (m/s) and angular (rad/s). A
        differential-drive base uses exactly two of those six numbers —
        `linear.x` and `angular.z`. Setting `linear.y` does nothing at all,
        because the wheels cannot slide sideways. (The Go2 CAN — see
        agr_legged_examples 05_walk.)
      - /cmd_vel is a STREAMED topic, not a command queue. You are not saying
        "drive 1 metre", you are saying "right now, go this fast" — over and
        over. Stop publishing and the robot keeps the last speed until
        something else tells it otherwise.
      - Which is why the last thing this script does is publish an all-zeros
        Twist. ALWAYS send the stop. On real hardware, a script that crashes
        mid-motion leaves a robot driving.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

RATE_HZ = 20


def main():
    rclpy.init()
    node = Node('drive')
    for n, d in (('vx', 0.3), ('wz', 0.0), ('seconds', 3.0)):
        node.declare_parameter(n, d)
    vx, wz, secs = (float(node.get_parameter(n).value)
                    for n in ('vx', 'wz', 'seconds'))

    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    cmd = Twist()
    cmd.linear.x, cmd.angular.z = vx, wz

    print(f'driving: linear.x={vx:+.2f} m/s  angular.z={wz:+.2f} rad/s '
          f'for {secs:.1f}s')
    for _ in range(int(secs * RATE_HZ)):
        pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)

    pub.publish(Twist())
    print('stopped')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
