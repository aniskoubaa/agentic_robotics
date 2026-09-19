"""
TurtleBot — start one of the OFFICIAL simulators.
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

    ros2 launch agr_tb_bringup sim.launch.py
    ros2 launch agr_tb_bringup sim.launch.py model:=tb4_standard
    ros2 launch agr_tb_bringup sim.launch.py model:=tb3_waffle world:=turtlebot3_house

    (or just:  agr-sim turtlebot model:=tb4_lite)

WHAT THIS IS, AND WHAT IT IS NOT
    It is a THIN WRAPPER. It does not describe a robot, spawn anything,
    bridge a topic or define a world. All of that is upstream's, installed by
    apt, and included here unchanged — `ros2 launch turtlebot4_gz_bringup
    turtlebot4_gz.launch.py` is what actually runs.

    Three jobs, and only three:

    1. Look the robot up in agr_tb_tools/config/robots.yaml, so `model:=`
       takes a name from one list rather than requiring you to remember which
       package and launch file each robot uses.

    2. Set TURTLEBOT3_MODEL. TurtleBot 3's own launch files read it with
       os.environ['TURTLEBOT3_MODEL'] and NO default, so an unset variable is
       a KeyError traceback rather than a helpful message. This is the single
       most common way to fail to start a TurtleBot 3.

    3. Export AGR_TB_MODEL, so every script started afterwards knows which
       robot is running without being told again.

    The environment hygiene that actually makes the official stack work —
    stripping snap paths, pinning GZ_IP to loopback, forcing xcb on Wayland —
    lives in `agr-sim`, because it is not specific to TurtleBots. It matters
    more here than anywhere else, though: launched from a snap-polluted
    shell, TurtleBot 4's Gazebo GUI dies with a libpthread symbol lookup
    error AND the diffdrive_controller spawner dies with it, leaving a
    simulator with no /odom and a robot that will not move.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    from agr_tb_tools import registry

    key = LaunchConfiguration('model').perform(context)
    try:
        bot = registry.get(key)
    except KeyError as exc:
        return [LogInfo(msg=f'\n  {exc}\n  '
                            f'Run `ros2 run agr_tb_tools list_robots` to see them.\n')]

    world = LaunchConfiguration('world').perform(context) or bot.default_world
    if bot.worlds and world not in bot.worlds:
        return [LogInfo(msg=f'\n  {bot.key} has no world called {world!r}.\n'
                            f'  Choose one of: {", ".join(bot.worlds)}\n')]

    upstream = PathJoinSubstitution(
        [FindPackageShare(bot.launch_package), 'launch', bot.launch_file])

    # TurtleBot 3's launch files take no arguments at all — the world is baked
    # into which launch file you pick, and the model comes from the
    # environment. TurtleBot 4's take proper arguments. So the two families
    # are handed their settings in the two different ways they expect.
    if bot.family == 'turtlebot3':
        launch_file = bot.launch_file
        if world and world != 'turtlebot3_world':
            launch_file = f'{world}.launch.py'
        upstream = PathJoinSubstitution(
            [FindPackageShare(bot.launch_package), 'launch', launch_file])
        arguments = {}
    else:
        arguments = {
            'world': world,
            'model': bot.model,
            'rviz': LaunchConfiguration('rviz').perform(context),
        }

    return [
        LogInfo(msg=f'\n  starting {bot.key}: {bot.description}\n'
                    f'  world:   {world}\n'
                    f'  upstream: {bot.launch_package}/{bot.launch_file}\n'
                    + ('  NOTE: this robot spawns DOCKED and will not drive until\n'
                       '        it is undocked. Scripts using agr_tb_tools.Base\n'
                       '        call ensure_ready(), which does it for you.\n'
                       if bot.docked_at_start else '')),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', bot.model),
        SetEnvironmentVariable('AGR_TB_MODEL', bot.key),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream),
            launch_arguments=arguments.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='tb3_waffle',
            description='Which robot. See `ros2 run agr_tb_tools list_robots`.'),
        DeclareLaunchArgument(
            'world', default_value='',
            description='World name; empty means the robot\'s default.'),
        DeclareLaunchArgument(
            'rviz', default_value='false', choices=['true', 'false'],
            description='TurtleBot 4 only: start rviz alongside the sim.'),
        OpaqueFunction(function=launch_setup),
    ])
