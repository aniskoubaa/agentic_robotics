"""
RAISE 2026 — full sim bringup.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

Brings up Gazebo Harmonic with the greenhouse world, spawns Husky + UR5e from
the rendered URDF, starts robot_state_publisher, and bridges essential
ROS 2 ↔ Gazebo topics.

Deferred to later launches:
  - Nav2 (see nav.launch.py — added when D1L1 needs it)
  - MoveIt 2 for the UR5e (see arm_moveit.launch.py)
  - Wrist camera bridge (needs <gazebo> sensor block added to URDF)
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
from launch_ros.actions import Node

import xacro


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')

    pkg_worlds = get_package_share_directory('raise2026_worlds')
    pkg_description = get_package_share_directory('raise2026_description')
    pkg_bringup = get_package_share_directory('raise2026_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_worlds, 'worlds', world)
    robot_xacro = os.path.join(pkg_description, 'urdf', 'raise2026_robot.urdf.xacro')
    bridge_config = os.path.join(pkg_bringup, 'config', 'ros_gz_bridge.yaml')

    robot_description = xacro.process_file(robot_xacro).toxml()

    # ── Patch joint damping on the four "continuous" Robotiq mimic'd joints.
    # The Robotiq URDF declares mimic constraints that DART (Gazebo Harmonic
    # physics) cannot enforce. Without joint-level damping, those joints
    # free-wheel and shake violently. We inject <dynamics damping=...> into
    # each joint definition so DART applies physical viscous friction.
    import re as _re
    _shaky_joints = [
        'gripper_robotiq_85_left_inner_knuckle_joint',
        'gripper_robotiq_85_right_inner_knuckle_joint',
        'gripper_robotiq_85_left_finger_tip_joint',
        'gripper_robotiq_85_right_finger_tip_joint',
    ]
    for _j in _shaky_joints:
        # URDF spec: <dynamics> is a *sibling* of <axis> (child of <joint>),
        # not inside <axis>. The URDF→SDF converter then re-parents it into
        # SDF's <joint><axis><dynamics>. We also convert type="continuous"
        # → type="revolute" with explicit limits so DART can't wind the
        # joint to ±30 rad on the startup collision impulse.
        _pat = _re.compile(
            r'<joint\s+name="' + _re.escape(_j) + r'"\s+type="continuous">'
            r'(.*?)<axis\s+xyz="([^"]+)"\s*/>',
            _re.DOTALL,
        )
        _replacement = (
            r'<joint name="' + _j + r'" type="revolute">'
            r'\1<axis xyz="\2"/>'
            r'<limit lower="-1.0" upper="1.0" effort="100" velocity="10"/>'
            r'<dynamics damping="50.0" friction="2.0"/>'
        )
        robot_description = _pat.sub(_replacement, robot_description, count=1)

    # -r = run immediately (don't start paused).
    # -s (headless:=true) = server only, no GUI — used for hands-free dataset
    # recording (05_auto_demonstrate) and CI. Sensors still render via EGL on
    # the GPU, so the wrist camera works without a display.
    gz_flags = '-s -r' if headless else '-r'
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{gz_flags} {world_path}'}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'publish_frequency': 30.0,
        }],
        # Silence the "Moved backwards in time" warnings that fire when
        # sim time stamps occasionally arrive out of order from the bridge.
        arguments=['--ros-args', '--log-level', 'robot_state_publisher:=ERROR'],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'raise2026_robot',
            '-topic', 'robot_description',
            '-x', x, '-y', y, '-z', '0.2', '-Y', yaw,
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    return [gz_sim, rsp, spawn, bridge]


def generate_launch_description():
    pkg_worlds = get_package_share_directory('raise2026_worlds')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='greenhouse_2026.sdf',
            description='SDF in raise2026_worlds/worlds/. Use greenhouse_2026_lite.sdf on CPU_ONLY.',
        ),
        DeclareLaunchArgument('x', default_value='-6.0', description='Spawn X (m); west of the crop, in the +1.5 m drive aisle'),
        DeclareLaunchArgument('y', default_value='1.5',  description='Spawn Y (m); 1.5 = centre of the aisle between rows 1 (y=0) and 2 (y=+3)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw (rad); 0 faces +x (east), down the aisle'),
        DeclareLaunchArgument('headless', default_value='false', description='true = server only (no GUI); for dataset recording / CI'),

        # Resolve model://tomato_plant against raise2026_worlds/share/.../meshes
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_worlds, 'meshes'),
        ),

        OpaqueFunction(function=launch_setup),
    ])
