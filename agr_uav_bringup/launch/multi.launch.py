# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""Several UAVs at once — the swarm starting point.

    ros2 launch agr_uav_bringup multi.launch.py                     # 3x x500
    ros2 launch agr_uav_bringup multi.launch.py airframes:=x500,standard_vtol,rc_cessna
    ros2 launch agr_uav_bringup multi.launch.py count:=5 headless:=true

Topic layout (PX4's rule, not ours):

    instance 0 -> /fmu/out/...          <- NO namespace
    instance 1 -> /px4_1/fmu/out/...
    instance 2 -> /px4_2/fmu/out/...

That asymmetry is why every node here takes `namespace` as a parameter.
ONE XRCE agent serves all of them; they differ by DDS key, not port.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agr_uav_bringup.px4_sitl import px4_sitl_process, xrce_agent_process
from agr_uav_description import load_airframe
from agr_uav_tools.px4_topics import namespace_for_instance

SPACING_M = 5.0   # metres between spawn points, so they don't collide at t=0


def _setup(context, *_args, **_kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    names = [n.strip() for n in arg('airframes').split(',') if n.strip()]
    count = int(arg('count'))
    if not names:
        names = ['x500']
    # `count` extends the list by repeating the last entry, so
    # `airframes:=x500 count:=5` gives five x500s.
    while len(names) < count:
        names.append(names[-1])
    names = names[:count] if count else names

    headless = arg('headless').lower() in ('true', '1', 'yes')
    frames = [load_airframe(n) for n in names]

    actions = [
        LogInfo(msg=f'[agr_uav] launching {len(frames)} vehicles: '
                    + ', '.join(f.name for f in frames)),
        xrce_agent_process(port=int(arg('agent_port'))),
    ]

    for i, frame in enumerate(frames):
        ns = namespace_for_instance(i)
        actions.append(LogInfo(
            msg=f'[agr_uav]   [{i}] {frame.name:<14} → {ns or "(no namespace)"}'))
        actions.append(px4_sitl_process(
            sys_autostart=frame.sys_autostart,
            px4_model=frame.px4_model,
            world=arg('world'),
            instance=i,
            headless=headless,
            model_pose=f'{i * SPACING_M},0',
            px4_dir=arg('px4_dir'),
        ))
        if arg('monitor').lower() in ('true', '1', 'yes'):
            actions.append(Node(
                package='agr_uav_tools',
                executable='vehicle_monitor',
                name='vehicle_monitor',
                namespace=ns or None,
                output='screen',
                parameters=[{'namespace': ns}],
            ))

    return actions


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('airframes', default_value='x500',
                              description='comma-separated registry names'),
        DeclareLaunchArgument('count', default_value='3',
                              description='how many vehicles; pads by repeating the last airframe'),
        DeclareLaunchArgument('world', default_value='default'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('agent_port', default_value='8888'),
        DeclareLaunchArgument('monitor', default_value='false',
                              description='one monitor per vehicle; noisy above 3'),
        DeclareLaunchArgument('px4_dir', default_value=''),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
