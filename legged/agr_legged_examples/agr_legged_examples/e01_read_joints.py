#!/usr/bin/env python3
"""
Go2 example 01 — read the twelve joints.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 run agr_legged_examples 01_read_joints

WHAT  Subscribe to /joint_states and print every joint angle once.

LEARN - A quadruped is TWELVE numbers: 4 legs x (hip, thigh, calf). Every
        pose, every gait, every fall is some sequence of those twelve.
      - /joint_states is a SENSOR topic — it reports where the joints ARE,
        which is not the same as where they were told to be. Compare this
        with what gait.py publishes and the difference is tracking error.
      - JointState carries `name` and `position` as two parallel arrays. Never
        assume the order matches your own list; zip them into a dict first.
        Assuming the order is the single most common bug in this file's job.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

LEGS = ('FL', 'FR', 'RL', 'RR')
PARTS = ('hip', 'thigh', 'calf')


def main():
    rclpy.init()
    node = Node('read_joints')
    latest = {}
    node.create_subscription(
        JointState, '/joint_states',
        lambda m: latest.update(zip(m.name, m.position)), 10)

    # Spin until a sample arrives. Publishing starts only when
    # joint_state_broadcaster is active, so an empty result means the
    # controllers are not up — not that the robot has no joints.
    print('waiting for /joint_states ...')
    for _ in range(200):                       # ~4 s at 20 ms per spin
        rclpy.spin_once(node, timeout_sec=0.02)
        if latest:
            break

    if not latest:
        print('no /joint_states — is `agr-sim legged` running?')
    else:
        print(f'\n{"leg":<6}' + ''.join(f'{p:>10}' for p in PARTS) + '\n' + '-' * 36)
        for leg in LEGS:
            row = ''.join(f'{latest.get(f"{leg}_{p}_joint", float("nan")):>10.3f}'
                          for p in PARTS)
            print(f'{leg:<6}{row}')
        print('\nradians. The calf is always negative: the leg cannot straighten.')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
