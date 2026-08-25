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
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agr_uav_bringup.px4_sitl import (gz_gui_process, gz_ready_process,
                                      gz_server_process,
                                      px4_sitl_process, resolve_world,
                                      xrce_agent_process)
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

    # Resolve the world to an absolute path now, so a typo fails here with the
    # list of valid names rather than as an empty Gazebo scene.
    world_path = resolve_world(arg('world'), arg('px4_dir'))
    actions.append(LogInfo(msg=f'[agr_uav] world: {world_path}'))

    if arg('agent').lower() in ('true', '1', 'yes'):
        actions.append(xrce_agent_process(port=int(arg('agent_port'))))

    # We start Gazebo; PX4 attaches to it (see px4_sitl.py for why).
    actions.append(gz_server_process(world_path, arg('px4_dir')))
    if not headless:
        actions.append(gz_gui_process(world_path, arg('px4_dir')))

    px4 = px4_sitl_process(
        sys_autostart=frame.sys_autostart,
        px4_model=frame.px4_model,
        world=arg('world'),
        instance=instance,
        headless=headless,
        px4_dir=arg('px4_dir'),
    )

    # Camera bridge, only for airframes that declare one. The UAV stack has
    # no ros_gz_bridge otherwise — PX4 telemetry comes over XRCE-DDS, which is
    # a different path entirely — so this is the one thing that has to cross
    # from Gazebo transport into ROS. Started with PX4 rather than with the
    # server because the model does not exist until PX4 spawns it, and the
    # bridge would advertise a topic that never carries anything.
    if frame.has_camera:
        gz_img = frame.gz_camera_topic(arg('world'), instance)
        ros_img = f'/{namespace}/camera' if namespace else '/camera'
        actions.append(LogInfo(msg=f'[agr_uav] camera: {gz_img} → {ros_img}'))
        camera_bridge = Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='camera_bridge', output='screen',
            arguments=[f'{gz_img}@sensor_msgs/msg/Image[gz.msgs.Image',
                       f'{gz_img.rsplit("/", 1)[0]}/camera_info'
                       f'@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
            remappings=[(gz_img, ros_img),
                        (f'{gz_img.rsplit("/", 1)[0]}/camera_info',
                         f'{ros_img}/camera_info')])
    else:
        camera_bridge = None

    after_ready = [px4]
    if camera_bridge is not None:
        after_ready.append(camera_bridge)
    if arg('monitor').lower() in ('true', '1', 'yes'):
        after_ready.append(Node(
            package='agr_uav_tools',
            executable='vehicle_monitor',
            name='vehicle_monitor',
            output='screen',
            parameters=[{'namespace': namespace}],
        ))

    # Start PX4 only once Gazebo is genuinely up (see gz_ready_process).
    ready = gz_ready_process(arg('world'))
    actions.append(ready)
    actions.append(RegisterEventHandler(
        OnProcessExit(target_action=ready, on_exit=after_ready)))

    return actions


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('airframe', default_value='x500',
                              description='registry name; see list_airframes'),
        DeclareLaunchArgument('world', default_value='default',
                              description='world name: default | agr_city | agr_defense '
                                          '| any PX4 world (baylands, forest, windy, …)'),
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
