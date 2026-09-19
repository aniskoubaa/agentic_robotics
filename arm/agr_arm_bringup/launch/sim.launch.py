"""
AGR Arm — full sim bringup.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

Starts Gazebo Harmonic with the workcell, spawns the bench-mounted UR5e +
Robotiq 2F-85 from the rendered URDF, runs robot_state_publisher, and bridges
the ROS 2 <-> Gazebo topics listed in config/ros_gz_bridge.yaml.

    ros2 launch agr_arm_bringup sim.launch.py
    ros2 launch agr_arm_bringup sim.launch.py tools:=true
    ros2 launch agr_arm_bringup sim.launch.py headless:=true

There is no controller_manager and there are no spawners: every joint on this
robot is driven by a gz JointPositionController plugin declared in the URDF,
so the arm holds its home pose from the first physics step and there is no
activation ordering to lose a race to.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

# The four Robotiq joints that carry a URDF <mimic> tag. DART does not
# implement mimic constraints, so these are unconstrained revolute joints as
# far as the physics engine is concerned.
MIMIC_JOINTS = (
    'gripper_robotiq_85_left_inner_knuckle_joint',
    'gripper_robotiq_85_right_inner_knuckle_joint',
    'gripper_robotiq_85_left_finger_tip_joint',
    'gripper_robotiq_85_right_finger_tip_joint',
)


def damp_mimic_joints(urdf: str) -> str:
    """Make the Robotiq's mimic'd joints physically stable under DART.

    Left alone they are `continuous` (no limits) and undamped, so the spawn
    contact impulse spins them to tens of radians and the fingers visibly fly
    apart. Two edits per joint, both required:

      1. continuous -> revolute with explicit limits, so DART has something
         to clamp against.
      2. a <dynamics damping= friction=> element, which DART applies as real
         viscous friction independent of any PID, so the joints settle on
         their own without a controller fighting them.

    The damping value is a balance, not a large-is-safe knob. The ground
    platform uses 50, which certainly stops the fingers flying apart — and
    also means four damped joints resist every close hard enough that the
    gripper takes over ten seconds to shut. At 8 the fingers were free enough to
    shake wrist_3 — the joint that carries the whole gripper — into a small
    limit cycle 0.02 rad off its target. 20 with a little static friction
    keeps everything still and still lets the knuckle controllers (20 N m,
    see agr_arm_gazebo.urdf.xacro) close the gripper in about two seconds.
    Change one of the two and you have to re-check the other, and re-check
    wrist_3 while you are there.

    <dynamics> is a SIBLING of <axis> in URDF (a direct child of <joint>);
    the URDF->SDF converter re-parents it into SDF's <joint><axis><dynamics>.
    Putting it inside <axis> here is silently ignored.
    """
    for joint in MIMIC_JOINTS:
        pattern = re.compile(
            r'<joint\s+name="' + re.escape(joint) + r'"\s+type="continuous">'
            r'(.*?)<axis\s+xyz="([^"]+)"\s*/>',
            re.DOTALL,
        )
        replacement = (
            r'<joint name="' + joint + r'" type="revolute">'
            r'\1<axis xyz="\2"/>'
            r'<limit lower="-1.0" upper="1.0" effort="100" velocity="10"/>'
            r'<dynamics damping="20.0" friction="1.0"/>'
        )
        urdf = pattern.sub(replacement, urdf, count=1)
    return urdf


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')
    tools = LaunchConfiguration('tools').perform(context).lower() in ('true', '1', 'yes')

    pkg_worlds = get_package_share_directory('agr_arm_worlds')
    pkg_description = get_package_share_directory('agr_arm_description')
    pkg_bringup = get_package_share_directory('agr_arm_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_worlds, 'worlds', world)
    robot_xacro = os.path.join(pkg_description, 'urdf', 'agr_arm_robot.urdf.xacro')
    bridge_config = os.path.join(pkg_bringup, 'config', 'ros_gz_bridge.yaml')

    robot_description = damp_mimic_joints(xacro.process_file(robot_xacro).toxml())

    # -r runs immediately instead of starting paused; -s is server-only, which
    # still renders the cameras through EGL, so headless CI keeps its images.
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
        arguments=['--ros-args', '--log-level', 'robot_state_publisher:=ERROR'],
    )

    # No -x/-y/-Y: the URDF pins `world` to the Gazebo world with a fixed
    # joint, so the model's own pose is the identity by construction and a
    # spawn offset would only move the pedestal out from under the arm.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'agr_arm_robot', '-topic', 'robot_description'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    actions = [gz_sim, rsp, spawn, bridge]

    # The tool servers are what turn the arm from "drivable" into "callable".
    # Off by default so the bare sim stays lean; one argument away because
    # three extra terminals is the same thing as nobody running them.
    if tools:
        actions += [
            Node(package='agr_arm_tools', executable=name, name=name,
                 output='screen',
                 parameters=[{'use_sim_time': True, 'world': world.replace('.sdf', '')}])
            for name in ('gripper_server', 'move_to_pose_server', 'grasp_server')
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='agr_workcell.sdf',
            description='SDF in agr_arm_worlds/worlds/.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true = server only, no GUI (CI, dataset recording).'),
        DeclareLaunchArgument(
            'tools', default_value='false',
            description='start gripper_server, move_to_pose_server and grasp_server.'),
        OpaqueFunction(function=launch_setup),
    ])
