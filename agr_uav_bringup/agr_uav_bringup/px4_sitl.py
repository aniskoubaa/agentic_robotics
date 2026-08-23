# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""Build the ExecuteProcess actions for PX4 SITL and the XRCE-DDS agent.

Shared by single.launch.py and multi.launch.py so the invocation exists once.

PX4 SITL is not a ROS node — it is a plain binary that happens to speak DDS
through the Micro XRCE-DDS agent. Launching it from a ROS launch file therefore
means ExecuteProcess, not Node.

Two environment facts drive everything here:

* PX4 reads PX4_SYS_AUTOSTART / PX4_SIM_MODEL / PX4_GZ_WORLD, and spawns Gazebo
  itself. We do not launch `gz sim` separately; PX4 does it.
* `-i <instance>` is what makes multi-vehicle work. PX4's rcS then derives
  MAV_SYS_ID, UXRCE_DDS_KEY and the DDS namespace from it — instance 0 gets NO
  namespace, instance N gets `px4_N`.

NOTE ON LIBRARIES: the px4 binary resolves libgz-*.so from the ROS vendor
Gazebo under /opt/ros/<distro>/opt/gz_*_vendor/lib. `ros2 launch` always runs
with ROS sourced, so that is satisfied automatically here — but the same binary
run from a bare shell will fail with
"error while loading shared libraries: libgz-utils2.so.2".
"""
from __future__ import annotations

import os
from typing import Optional

from launch.actions import ExecuteProcess


def px4_root(explicit: str = '') -> str:
    """Where PX4-Autopilot lives. Explicit arg > $PX4_DIR > ~/PX4-Autopilot."""
    return explicit or os.environ.get('PX4_DIR') or os.path.expanduser('~/PX4-Autopilot')


def px4_binary(root: str) -> str:
    return os.path.join(root, 'build', 'px4_sitl_default', 'bin', 'px4')


def px4_sitl_process(*, sys_autostart: int, px4_model: str, world: str = 'default',
                     instance: int = 0, headless: bool = False,
                     model_pose: Optional[str] = None,
                     px4_dir: str = '') -> ExecuteProcess:
    """One PX4 SITL instance, with Gazebo."""
    root = px4_root(px4_dir)
    env = {
        'PX4_SYS_AUTOSTART': str(sys_autostart),
        'PX4_SIM_MODEL': px4_model,
        'PX4_GZ_WORLD': world,
    }
    if headless:
        env['HEADLESS'] = '1'
    if model_pose:
        # "x,y" or "x,y,z,roll,pitch,yaw" — without this every instance in a
        # multi-vehicle run spawns on top of the previous one at the origin.
        env['PX4_GZ_MODEL_POSE'] = model_pose

    return ExecuteProcess(
        cmd=[px4_binary(root), '-i', str(instance)],
        cwd=root,
        additional_env=env,
        output='screen',
        # PX4 flushes its ULog on SIGINT; SIGKILL truncates it and the flight
        # record is what the experiments are made of.
        sigterm_timeout='5',
        sigkill_timeout='10',
    )


def xrce_agent_process(*, port: int = 8888) -> ExecuteProcess:
    """The PX4 <-> ROS 2 bridge.

    ONE agent serves every vehicle: instances are distinguished by DDS key, not
    by port. Starting one per vehicle is a common and confusing mistake.
    """
    return ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', str(port)],
        output='screen',
        sigterm_timeout='5',
    )
