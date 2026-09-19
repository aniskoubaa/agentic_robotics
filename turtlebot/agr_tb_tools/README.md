# `agr_tb_tools`

The registry that says which TurtleBots exist, and the `Base` helper that
drives any of them.

```bash
ros2 run agr_tb_tools list_robots
```

## `config/robots.yaml` — the registry

Same idea as the UAV platform's airframe registry, for the same reason: the
lessons should be written once and run on any robot, so the per-robot
differences live in one file instead of being scattered through every script
as an if-statement.

| Robot | Camera | Docked at start | Max m/s |
|---|---|---|---|
| `tb3_burger` | none | no | 0.22 |
| `tb3_waffle` | `/camera/image_raw` | no | 0.26 |
| `tb4_lite` | `/oakd/rgb/preview/image_raw` | **yes** | 0.31 |
| `tb4_standard` | `/oakd/rgb/preview/image_raw` | **yes** | 0.31 |

Every entry points at an **official upstream launch file**. Nothing here is a
robot we built, and every topic name in it was read off a running robot rather
than taken from documentation.

## `base.py` — the `Base` helper

Wraps a node rather than subclassing one, so a script can own its node and
still hand the robot around. Everything blocking spins that node itself, so no
example needs an executor or a thread.

```python
bot = Base(node)          # reads AGR_TB_MODEL, set by agr-sim
bot.wait_for_state()
bot.ensure_ready()        # undocks a TurtleBot 4; no-op on a TurtleBot 3
result = bot.drive(0.2, seconds=3.0)
err = bot.turn(math.pi / 2)
```

`drive()` returns a **dict, not a bool**, because "did it do what I asked" has
a genuinely interesting answer on a Create 3: `commanded` is what you asked
for, `measured_max`/`measured_min` are what the wheels reported, and
`reflex_reversed` is true when the robot's own safety layer overrode you.

### Four things in here are hardware facts, not design choices

1. **`/cmd_vel` is `TwistStamped` on both robots** under Jazzy. Publish a
   `Twist` and the robot does not move and nothing complains.
2. **A TurtleBot 4 spawns docked** and the Create 3 refuses to drive until
   `/undock` completes — about 30 s. That is real hardware behaviour,
   faithfully simulated.
3. **Create 3 sensor topics are BEST_EFFORT.** A default RELIABLE subscription
   to `/dock_status` receives literally nothing, with one easily missed
   warning at startup. Hence `SENSOR_QOS`.
4. **The Create 3 runs a reflex layer** that overrides velocity commands.
   Measured: commanding +0.25 m/s near the dock produced +0.25 for two seconds
   and then **−0.14 m/s** for as long as the command was held.

`turn()` closes on measured heading rather than a timer, and **unwraps** the
angle as it accumulates — comparing raw yaw across the ±π seam is the classic
way to make a 180° turn read as a tiny one.
