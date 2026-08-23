"""
RAISE 2026 — joystick teleop launch.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

Brings up:
  - joy_node                → publishes /joy from the connected controller
  - teleop_twist_joy_node   → reads /joy + emits /cmd_vel

Connect any USB / Bluetooth gamepad before running.
Test stick: `ros2 topic echo /joy`
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('raise2026_teleop'),
        'config', 'teleop_joy.yaml'
    )
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'autorepeat_rate': 20.0}],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config],
            remappings=[('/cmd_vel', '/cmd_vel')],
        ),
    ])
