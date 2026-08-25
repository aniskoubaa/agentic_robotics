# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""Unitree Go2 in the inspection world.

    agr-sim legged
    agr-sim legged world:=agr_inspection
    agr-sim legged headless:=true
    agr-sim legged x:=-4.0 y:=0.0        # spawn at the ramp instead

Ordering matters and is enforced with event handlers rather than sleeps:
  gz sim → spawn robot → joint_state_broadcaster → position controller → stand
A controller spawned before the robot exists fails; a stance commanded before
the controller is active goes nowhere.
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *_a, **_k):
    def arg(n):
        return LaunchConfiguration(n).perform(context)

    world = arg('world')
    headless = arg('headless').lower() in ('true', '1', 'yes')
    world_path = os.path.join(
        get_package_share_directory('agr_legged_worlds'), 'worlds', f'{world}.sdf')
    if not os.path.isfile(world_path):
        avail = sorted(f[:-4] for f in os.listdir(os.path.dirname(world_path))
                       if f.endswith('.sdf'))
        raise FileNotFoundError(f'world {world!r} not found. Available: {avail}')

    xacro_file = os.path.join(
        get_package_share_directory('agr_legged_description'), 'urdf', 'go2.urdf.xacro')

    gz_args = f'-r {"-s " if headless else ""}{world_path}'
    gz = ExecuteProcess(
        cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', f'gz_args:={gz_args}'],
        output='screen')

    # Expand the xacro here rather than with a Command substitution: this
    # function already runs at launch time, and doing it inline means a broken
    # URDF fails with the xacro error itself instead of an empty
    # robot_description that only shows up as "robot never spawned".
    # Use the official meshes only if they were actually fetched. Auto-detecting
    # beats a flag the user must remember: the robot looks right when the assets
    # are there and still runs when they are not.
    mesh_dir = os.path.join(
        get_package_share_directory('agr_legged_description'), 'meshes')
    have_meshes = os.path.isfile(os.path.join(mesh_dir, 'base.dae'))
    use_meshes = arg('meshes')
    if use_meshes == 'auto':
        use_meshes = 'true' if have_meshes else 'false'
    if use_meshes == 'true' and not have_meshes:
        raise FileNotFoundError(
            'meshes:=true but no meshes are installed. Fetch them with\n'
            '    ros2 run agr_legged_description fetch_meshes.sh\n'
            'or pass meshes:=false to use primitive visuals.')

    try:
        robot_description = subprocess.run(
            ['xacro', xacro_file, f'use_meshes:={use_meshes}'],
            check=True, capture_output=True, text=True).stdout
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f'xacro failed on {xacro_file}:\n{exc.stderr}') from exc

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'go2',
                   '-x', arg('x'), '-y', arg('y'), '-z', arg('z')])

    jsb = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'joint_state_broadcaster'], output='screen')
    jgpc = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'joint_group_position_controller'], output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'])

    stand = Node(package='agr_legged_bringup', executable='stand', output='screen',
                 arguments=['--pose', arg('pose')])

    actions = [
        LogInfo(msg=f'[agr_legged] Unitree Go2 — 12 DOF, world: {world}, '
                    f'visuals: {"official meshes" if use_meshes == "true" else "primitives"}'),
        LogInfo(msg='[agr_legged] calf range is [-2.72, -0.84] rad: the leg never straightens'),
        gz, rsp, bridge, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[jgpc])),
    ]
    if arg('stand').lower() in ('true', '1', 'yes'):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=jgpc, on_exit=[stand])))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='agr_inspection'),
        DeclareLaunchArgument('headless', default_value='false'),
        # Clear ground between the pipe run and the staircase. NOT (0, 0):
        # pipe_0/pipe_1 lie along Y at x=0 with radius 0.16, so a robot spawned
        # at the origin lands straddling a pipe and settles pitched ~29 deg,
        # never reaching a level stance.
        DeclareLaunchArgument('x', default_value='1.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.34',
                              description='spawn height; feet sit 0.324 m below '
                                          'the trunk in the stand pose, so this '
                                          'drops the robot ~15 mm onto its feet'),
        DeclareLaunchArgument('pose', default_value='stand'),
        DeclareLaunchArgument('stand', default_value='true'),
        DeclareLaunchArgument('meshes', default_value='auto',
                              description='auto | true | false — official .dae visuals'),
        OpaqueFunction(function=_setup),
    ])
