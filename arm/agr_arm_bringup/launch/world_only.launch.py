"""
AGR Arm — the workcell with no robot in it.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 launch agr_arm_bringup world_only.launch.py

Useful for exactly one thing: telling a world problem apart from a robot
problem. If the bench and the blocks appear here, the SDF and the rendering
pipeline are fine and whatever is wrong is in the URDF or the spawn.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')

    world_path = os.path.join(
        get_package_share_directory('agr_arm_worlds'), 'worlds', world)
    gz_flags = '-s -r' if headless else '-r'

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{gz_flags} {world_path}'}.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='agr_workcell.sdf'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
