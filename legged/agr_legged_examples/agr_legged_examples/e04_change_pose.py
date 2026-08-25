#!/usr/bin/env python3
"""
Go2 example 04 — stand, crouch, tuck.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_legged_examples 04_change_pose
    ros2 run agr_legged_examples 04_change_pose --ros-args -p pose:=crouch

WHAT  Send the twelve joint angles for a named posture, ramping into it.

LEARN - This is the LOWEST level you can drive the robot at: a flat array of
        12 floats straight to the position controller. No gait, no IK.
      - The order of that array is not yours to choose — it is fixed by
        `joints:` in go2_controllers.yaml. Get it wrong and there is no error,
        just a robot that folds up in a way you did not ask for.
      - RAMP, never step. A step command on 12 position joints makes the robot
        launch itself off the ground: the controller will happily try to get
        there in one control period.
      - Only ONE thing may publish to a position controller. The gait
        controller publishes at 100 Hz, so if it is running, your ramp and its
        stance interleave message-by-message and the robot thrashes itself
        onto its back within a second or two. This script therefore CHECKS
        for another publisher and refuses. That check is the lesson: a
        command topic is not a request, it is a steering wheel, and two hands
        on it is not twice the control.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# (hip, thigh, calf) repeated for FL, FR, RL, RR — the controller's order.
POSES = {
    'stand':  [0.0, 0.80, -1.55] * 4,
    'crouch': [0.0, 1.25, -2.45] * 4,
    'tuck':   [0.0, 1.45, -2.70] * 4,
}
RAMP_S = 2.0
RATE_HZ = 50


def main():
    rclpy.init()
    node = Node('change_pose')
    node.declare_parameter('pose', 'stand')
    node.declare_parameter('force', False)
    name = node.get_parameter('pose').value
    if name not in POSES:
        print(f'unknown pose {name!r}; choose one of {sorted(POSES)}')
        node.destroy_node(); rclpy.try_shutdown(); return

    topic = '/joint_group_position_controller/commands'

    # Look BEFORE creating our own publisher, so the count is other people's.
    # Discovery is not instant: spin briefly or a live gait node reads as zero.
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.count_publishers(topic):
            break
    others = node.count_publishers(topic)
    force = bool(node.get_parameter('force').value)
    if others and not force:
        print(f'\n{others} other node is already publishing to {topic}\n'
              '(usually the gait controller). Two publishers on one\n'
              'position controller will thrash the robot onto its back.\n\n'
              'Stop the gait first — either:\n'
              '    agr-stop && agr-sim legged gait:=false\n'
              'or Ctrl-C the gait node if you started it by hand.\n\n'
              'or, if you really mean it:\n'
              '    ros2 run agr_legged_examples 04_change_pose '
              '--ros-args -p force:=true\n')
        node.destroy_node(); rclpy.try_shutdown(); return

    pub = node.create_publisher(Float64MultiArray, topic, 10)

    # Ramp from where the robot actually IS, not from an assumed pose — a
    # standing robot told to ramp "from the tuck" squats first, then rises.
    from sensor_msgs.msg import JointState
    JOINTS = [f'{l}_{p}_joint' for l in ('FL', 'FR', 'RL', 'RR')
              for p in ('hip', 'thigh', 'calf')]
    latest = {}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: latest.update(zip(m.name, m.position)), 10)
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.02)
        if all(j in latest for j in JOINTS):
            break
    start = ([latest[j] for j in JOINTS] if all(j in latest for j in JOINTS)
             else POSES['stand'])

    target = POSES[name]
    steps = int(RAMP_S * RATE_HZ)
    print(f'ramping to {name!r} over {RAMP_S:.0f}s')
    for i in range(steps + 1):
        a = i / steps
        msg = Float64MultiArray()
        msg.data = [s + (t - s) * a for s, t in zip(start, target)]
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)
    print('done — holding')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
