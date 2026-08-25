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


def _own_worlds_dir() -> Optional[str]:
    """share/worlds of agr_uav_worlds, if the package is installed."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('agr_uav_worlds'), 'worlds')
    except Exception:
        return None


def resolve_world(world: str, px4_dir: str = '') -> str:
    """Absolute path to <world>.sdf — ours first, then PX4's.

    Raises FileNotFoundError listing what IS available, because a typo'd world
    name should not turn into Gazebo silently starting an empty scene.
    """
    candidates = []
    own = _own_worlds_dir()
    if own:
        candidates.append(own)
    candidates.append(os.path.join(px4_root(px4_dir), 'Tools', 'simulation', 'gz', 'worlds'))

    for d in candidates:
        path = os.path.join(d, f'{world}.sdf')
        if os.path.isfile(path):
            return path

    available = sorted({os.path.splitext(f)[0]
                        for d in candidates if os.path.isdir(d)
                        for f in os.listdir(d) if f.endswith('.sdf')})
    raise FileNotFoundError(
        f"world {world!r} not found. Available: {', '.join(available)}")


def px4_gz_env(px4_dir: str = '') -> dict:
    """The PX4_GZ_* / GZ_SIM_* values PX4's build generated for this checkout.

    PX4 writes build/px4_sitl_default/rootfs/gz_env.sh at build time and sources
    it at runtime — which is why passing PX4_GZ_WORLDS in the environment has no
    effect: gz_env.sh exports over the top of it. Worse, PX4 sources that file
    ONLY on the branch where it starts Gazebo itself, so in standalone mode
    PX4_GZ_MODELS would be unset and the model-spawn URI would come out as
    file:///x500/model.sdf. So: read the generated values here and pass them in
    explicitly. Parsing PX4's own generated file beats hard-coding paths that
    would drift the moment PX4 rearranges its tree.
    """
    root = px4_root(px4_dir)
    env_sh = os.path.join(root, 'build', 'px4_sitl_default', 'rootfs', 'gz_env.sh')
    values: dict = {}
    if not os.path.isfile(env_sh):
        return values
    for line in open(env_sh, 'r', encoding='utf-8'):
        line = line.strip()
        if not line.startswith('export '):
            continue
        key, _, val = line[len('export '):].partition('=')
        if key.startswith('PX4_GZ_') and '$' not in val:
            values[key] = val.strip().strip('"')
    return values


def gz_environment(world_path: str, px4_dir: str = '') -> dict:
    """Environment for a `gz sim` we start ourselves."""
    px4 = px4_gz_env(px4_dir)
    resource = [d for d in (os.path.dirname(world_path),
                            px4.get('PX4_GZ_MODELS'),
                            px4.get('PX4_GZ_WORLDS')) if d]
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resource.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    env = {'GZ_SIM_RESOURCE_PATH': ':'.join(resource)}
    if px4.get('PX4_GZ_PLUGINS'):
        plugins = [px4['PX4_GZ_PLUGINS']]
        if os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH'):
            plugins.append(os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'])
        env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = ':'.join(plugins)
    if px4.get('PX4_GZ_SERVER_CONFIG'):
        env['GZ_SIM_SERVER_CONFIG_PATH'] = px4['PX4_GZ_SERVER_CONFIG']
    return env


def gz_server_process(world_path: str, px4_dir: str = '') -> ExecuteProcess:
    """The Gazebo server, running our chosen world file by absolute path."""
    return ExecuteProcess(
        cmd=['gz', 'sim', '--verbose=1', '-r', '-s', world_path],
        additional_env=gz_environment(world_path, px4_dir),
        output='screen', sigterm_timeout='5', sigkill_timeout='10',
    )


def gz_gui_process(world_path: str, px4_dir: str = '') -> ExecuteProcess:
    """The Gazebo GUI, attaching to the server above."""
    return ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        additional_env=gz_environment(world_path, px4_dir),
        output='log', sigterm_timeout='5', sigkill_timeout='10',
    )


def gz_ready_process(world: str, timeout_s: int = 120) -> ExecuteProcess:
    """Block until OUR world's scene is queryable, then exit.

    Getting this wrong cost two failed launches, so the reasoning is worth
    recording:

    * PX4's own wait is 30 one-second attempts starting the moment PX4 starts.
      A large world does not finish in time and PX4 exits with "Timed out
      waiting for Gazebo world" while Gazebo is loading perfectly well.

    * The first version of this gate waited for ``/world/*/clock``. Wrong on two
      counts. The clock topic is SLOWER than scene/info, not faster — measured
      on agr_defense: scene/info at 5 s, clock not yet up at that point. And the
      wildcard matches ANY world, so a Gazebo left over from a previous run
      satisfies the gate and PX4 then queries a world that is being torn down.

    So: wait for ``/world/<this world>/scene/info`` specifically — the exact
    service PX4 greps for, scoped to the world we just launched.
    """
    script = (
        "for i in $(seq 1 {t}); do "
        "  if gz service -i --service \"/world/{w}/scene/info\" 2>&1 "
        "     | grep -q \"Service providers\"; then "
        "    echo \"[agr_uav] world {w} ready after ${{i}}s\"; exit 0; fi; "
        "  sleep 1; "
        "done; "
        "echo \"[agr_uav] ERROR: world {w} scene not ready in {t}s\" >&2; exit 1"
    ).format(t=timeout_s, w=world)
    return ExecuteProcess(cmd=['bash', '-c', script], output='screen')


def px4_sitl_process(*, sys_autostart: int, px4_model: str, world: str = 'default',
                     instance: int = 0, headless: bool = False,
                     model_pose: Optional[str] = None,
                     px4_dir: str = '') -> ExecuteProcess:
    """One PX4 SITL instance, with Gazebo."""
    root = px4_root(px4_dir)
    # STANDALONE: we start Gazebo, PX4 attaches to the running world.
    #
    # The alternative — letting PX4 start Gazebo — cannot load a world we
    # author, because PX4 builds the path as ${PX4_GZ_WORLDS}/${world}.sdf and
    # gz_env.sh exports PX4_GZ_WORLDS over anything we pass. Owning the Gazebo
    # launch also means ONE server for multi-vehicle runs, instead of N PX4
    # instances racing to start it and N-1 losing.
    env = {
        'PX4_SYS_AUTOSTART': str(sys_autostart),
        'PX4_SIM_MODEL': px4_model,
        'PX4_GZ_STANDALONE': '1',
        # Name the world explicitly. Left empty, PX4 tries to discover it from
        # the clock topic — which is slower to appear than the scene service,
        # and which would also let it latch onto a leftover world from a
        # previous run. Setting it skips discovery entirely.
        'PX4_GZ_WORLD': world,

        # ── Arming checks that SITL cannot satisfy ──────────────────────
        # rcS applies PX4_PARAM_<NAME> AFTER the airframe file, so these
        # override the airframe's `param set-default`. Both are sim-only:
        #
        # NAV_DLL_ACT: airframe 4001 (gz_x500) sets this to 2, which makes a
        #   GCS datalink MANDATORY for arming — with no QGroundControl
        #   attached every arm request dies on "Preflight Fail: No connection
        #   to the GCS". These labs drive the vehicle from ROS 2, not from a
        #   GCS, so the datalink-loss failsafe is disabled here.
        # CBRK_SUPPLY_CHK: SITL has no power module, so the redundant-supply
        #   check reports "system power unavailable" forever. 894281 is PX4's
        #   magic circuit-breaker value for this check.
        #
        # Together these are the difference between a vehicle that arms and
        # one that refuses to; verified by takeoff to 2.5 m.
        'PX4_PARAM_NAV_DLL_ACT': '0',
        'PX4_PARAM_CBRK_SUPPLY_CHK': '894281',
    }
    # gz_env.sh is NOT sourced on the standalone branch, so PX4_GZ_MODELS must
    # be supplied here or the model-spawn URI comes out empty.
    env.update(px4_gz_env(px4_dir))
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
