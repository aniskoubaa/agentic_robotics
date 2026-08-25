# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""Every RaiseBot tool server, in one command.

    ros2 launch raisebot_bringup tools.launch.py
    agr-sim ground tools:=true          # ...or start them with the sim

These six nodes turn the robot into a set of CALLABLE FUNCTIONS: instead of
publishing a Twist and integrating odometry yourself, you call /nav_to_home
and it returns success or a reason. That shape is the whole premise of the
agentic labs — an LLM can call a service, it cannot drive a velocity loop.

Starting them by hand means six terminals, which in practice means nobody
runs them and the service examples look broken. Hence this file.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

# Every executable in raisebot_tools. Keep in step with its setup.py.
SERVERS = [
    'navigation_server',    # /nav_to_<waypoint>, /drive_forward, /turn_left, ...
    'move_to_pose_server',  # /move_to_<named arm pose>
    'gripper_server',       # /open_gripper, /close_gripper, /rotate_gripper
    'detector_server',      # /detect_<camera>
    'inspector_server',     # /inspect_<target>
    'grasp_server',         # /grasp_<object>
]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(package='raisebot_tools', executable=name, name=name,
             output='screen', parameters=[{'use_sim_time': True}])
        for name in SERVERS
    ])
