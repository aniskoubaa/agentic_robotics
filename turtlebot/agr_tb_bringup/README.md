# `agr_tb_bringup`

Starts the **official** TurtleBot simulators. It adds nothing of its own.

```bash
agr-sim turtlebot                                  # tb3_waffle, default world
agr-sim turtlebot model:=tb4_standard world:=maze
agr-sim turtlebot model:=tb3_burger world:=turtlebot3_house
```

`sim.launch.py` has exactly three jobs:

1. **Look the robot up** in `agr_tb_tools/config/robots.yaml`, so `model:=`
   takes a name from one list instead of requiring you to remember which
   package and launch file each robot uses.
2. **Set `TURTLEBOT3_MODEL`.** TurtleBot 3's own launch files read it with
   `os.environ['TURTLEBOT3_MODEL']` and **no default**, so an unset variable is
   a `KeyError` traceback rather than a message. This is the most common way to
   fail to start a TurtleBot 3.
3. **Export `AGR_TB_MODEL`**, so every script started afterwards knows which
   robot is running without being told again.

Everything else — the robot description, the spawn, the bridge, the worlds — is
upstream's, included unchanged. `ros2 launch turtlebot4_gz_bringup
turtlebot4_gz.launch.py` is what actually runs.

## Why the wrapper earns its keep

The environment hygiene lives in `agr-sim`, not here, because it is not
specific to TurtleBots. But it matters more here than anywhere else in this
repo. Launched from a shell carrying snap paths — a VS Code terminal, for
instance — TurtleBot 4 fails **twice**:

```
gz sim gui: symbol lookup error: /snap/core20/current/lib/.../libpthread.so.0
[ERROR] [spawner-46]: process has died ... diffdrive_controller
```

The GUI dies, and it takes the controller spawner with it, leaving a simulator
with no `/odom` and a robot that silently will not move. With the same
environment stripped: `/clock` 121 Hz, `/scan` 17.7 Hz, `/odom` 18.6 Hz,
controller activated.
