# `agr_tb_teleop`

Drive whichever TurtleBot is running, and look through its camera.

```bash
ros2 run agr_tb_teleop teleop_keyboard
ros2 run agr_tb_teleop camera_view
```

`w`/`x` drive, `a`/`d` turn, `s` or space stop, `+`/`-` change speed,
`Ctrl-C` to exit. Arrow keys work too. On a TurtleBot 4, `u` undocks and `p`
docks.

Release the key and the robot stops on its own — velocity commands do **not**
latch, there is a watchdog in the base, and this node simply stops publishing.

## Why not just use `turtlebot3_teleop`

Three reasons, all measured on the running robots:

- It publishes **`TwistStamped`**, which is what `/cmd_vel` actually is under
  Jazzy on both robots. A node still publishing `Twist` moves nothing and
  reports no error.
- It knows about **docking**. A brand new TurtleBot 4 refuses to move, and `u`
  is the answer.
- It shows the **lidar distance ahead** while you drive, so the sensor and the
  motion are on screen together.

## `camera_view`

Takes its topic from the registry, so it works unchanged on a Waffle
(`/camera/image_raw`) and a TurtleBot 4 (the OAK-D's
`/oakd/rgb/preview/image_raw`, 320×240). On a Burger it says the robot has no
camera and exits, rather than waiting forever.

Press `Q` in the window to quit.
