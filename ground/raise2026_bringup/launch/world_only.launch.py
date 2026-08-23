"""
RAISE 2026 — world-only launch (smoke test).
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

Starts Gazebo Harmonic with the greenhouse world; no robot, no bridge.
Useful for verifying that the SDF + tomato_plant model resolve correctly.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    pkg_worlds = get_package_share_directory('raise2026_worlds')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_path = os.path.join(pkg_worlds, 'worlds', world)

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )]


def generate_launch_description():
    pkg_worlds = get_package_share_directory('raise2026_worlds')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='greenhouse_2026.sdf',
            description='SDF in raise2026_worlds/worlds/',
        ),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_worlds, 'meshes'),
        ),
        OpaqueFunction(function=launch_setup),
    ])
