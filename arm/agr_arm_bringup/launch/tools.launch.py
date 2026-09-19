"""
AGR Arm — the three tool servers, without restarting the simulator.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 launch agr_arm_bringup tools.launch.py

Same three nodes that `sim.launch.py tools:=true` starts. Split out because
they are the part you restart most often while writing a new skill, and
restarting Gazebo to reload one service server wastes half a minute each time.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SERVERS = ('gripper_server', 'move_to_pose_server', 'grasp_server')


def generate_launch_description():
    world = LaunchConfiguration('world')
    return LaunchDescription(
        [DeclareLaunchArgument(
            'world', default_value='agr_workcell',
            description='Gazebo world NAME (no .sdf) — grasp_server needs it '
                        'to address the right /world/<name>/set_pose service.')]
        + [Node(package='agr_arm_tools', executable=name, name=name,
                output='screen',
                parameters=[{'use_sim_time': True, 'world': world}])
           for name in SERVERS]
    )
