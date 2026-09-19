# `agr_arm_teleop`

Drive the arm by hand, and look through both of its eyes.

```bash
ros2 run agr_arm_teleop teleop_keyboard
ros2 run agr_arm_teleop camera_view --ros-args -p camera:=both
```

## Two modes, and the difference is the lesson

`TAB` switches between them.

**Tool mode** (the default) drives the gripper in metres. `w`/`x` forward and
back, `a`/`d` left and right, `r`/`f` up and down, `q`/`e` spin the fingers.
Every keypress solves inverse kinematics, so watch all six joints change to
move the hand 2 cm sideways — and watch it refuse, with `OUT OF REACH`, when
you walk off the edge of the workspace.

**Joint mode** drives one motor. `1`–`6` pick the joint, `w`/`x` turn it.
Nothing can fail and nothing can be out of reach; the tool position simply
goes wherever the geometry sends it. Turn the shoulder and watch the tool
readout wander.

Always available: `o`/`c` open and close the gripper, `h` go home, `+`/`-`
change the step size, `Ctrl-C` to exit. The arm holds its last position when
you leave.

## Why the display shows the target, not just the measurement

Jogging accumulates. If each step were taken from the arm's MEASURED
position, the controller's tracking error would be folded into every
keypress and a long run would drift. The teleop jogs from the last COMMAND
instead, and shows you the measurement separately.

## `camera_view`

`camera:=wrist` (default), `camera:=bench`, or `camera:=both` for side by
side. Run `both` while you drive, and the whole eye-in-hand versus
eye-to-hand argument plays out live: the bench camera always shows every
block and is regularly blocked by the arm's own elbow; the wrist camera shows
the thing you are about to grasp from 20 cm and nothing else at all.

Press `Q` in the window to quit.
