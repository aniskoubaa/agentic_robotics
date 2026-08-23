# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""One UAV: XRCE-DDS agent + PX4 SITL + Gazebo + a monitor node.

    ros2 launch agr_uav_bringup single.launch.py
    ros2 launch agr_uav_bringup single.launch.py airframe:=rc_cessna
    ros2 launch agr_uav_bringup single.launch.py headless:=true
    ros2 launch agr_uav_bringup single.launch.py agent:=false   # reuse a running agent

Airframes come from agr_uav_description/config/airframes.yaml —
`ros2 run agr_uav_tools list_airframes` prints them.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agr_uav_bringup.px4_sitl import px4_sitl_process, xrce_agent_process
from agr_uav_description import load_airframe
from agr_uav_tools.px4_topics import namespace_for_instance


def _setup(context, *_args, **_kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    airframe_name = arg('airframe')
    instance = int(arg('instance'))
    headless = arg('headless').lower() in ('true', '1', 'yes')

    # Fail here, with the valid names, rather than deep inside PX4.
    frame = load_airframe(airframe_name)
    namespace = namespace_for_instance(instance)

    actions = [
        LogInfo(msg=f'[agr_uav] {frame.name} ({frame.px4_model}, '
                    f'SYS_AUTOSTART={frame.sys_autostart}) — '
                    f'class={frame.airframe_class}, '
                    f'hover={"yes" if frame.can_hover else "NO"}'),
        LogInfo(msg=f'[agr_uav] instance {instance} → ROS namespace '
                    f'{namespace or "(none)"}'),
    ]

    if arg('agent').lower() in ('true', '1', 'yes'):
        actions.append(xrce_agent_process(port=int(arg('agent_port'))))

    actions.append(px4_sitl_process(
        sys_autostart=frame.sys_autostart,
        px4_model=frame.px4_model,
        world=arg('world'),
        instance=instance,
        headless=headless,
        px4_dir=arg('px4_dir'),
    ))

    if arg('monitor').lower() in ('true', '1', 'yes'):
        actions.append(Node(
            package='agr_uav_tools',
            executable='vehicle_monitor',
            name='vehicle_monitor',
            output='screen',
            parameters=[{'namespace': namespace}],
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('airframe', default_value='x500',
                              description='registry name; see list_airframes'),
        DeclareLaunchArgument('world', default_value='default',
                              description='PX4 Gazebo world name'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='no Gazebo GUI'),
        DeclareLaunchArgument('instance', default_value='0',
                              description='PX4 instance id; 0 = no ROS namespace'),
        DeclareLaunchArgument('agent', default_value='true',
                              description='start the XRCE-DDS agent'),
        DeclareLaunchArgument('agent_port', default_value='8888'),
        DeclareLaunchArgument('monitor', default_value='true',
                              description='start vehicle_monitor'),
        DeclareLaunchArgument('px4_dir', default_value='',
                              description='PX4-Autopilot root; default $PX4_DIR or ~/PX4-Autopilot'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
