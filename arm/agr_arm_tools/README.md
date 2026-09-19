# `agr_arm_tools`

The parts that make the arm *callable* rather than merely drivable: the
kinematics, a motion helper, and three service servers.

```bash
ros2 run agr_arm_tools workspace          # no simulator needed
agr-sim arm tools:=true                   # starts all three servers
```

## `kinematics.py` — the file that makes this platform different

A drone is told "go to this point" and a flight controller works out the
rest. A quadruped is told "walk this fast" and a gait generator works out the
rest. An arm has neither: between "the block is at (0.45, 0.00, 0.78)" and
"six joint angles" there is nothing but this maths.

| Call | Does |
|---|---|
| `fk(q)` | six angles → 4×4 pose of `ur5e_tool0` in `world` |
| `tcp(q)` | six angles → (x, y, z) of the grasp point |
| `jacobian(q)` | six angles → 6×6 d(twist)/d(q) |
| `ik(target, seed)` | 4×4 pose → six angles, or `None` |
| `ik_down(x, y, z, yaw)` | the top-down grasp special case |
| `reachable(x, y, z)` | a yes/no **with a reason** |

Every constant in `CHAIN` was read out of the *rendered* URDF, not typed from
a datasheet, and `diagnose` check 11 compares `fk()` against the live TF tree
so it cannot drift silently. Measured agreement: **0.04 mm**.

IK is damped least squares from a seed, not the UR's closed form, and that is
a deliberate trade. It returns the solution *nearest the seed*, so the arm
does not flip its elbow through the bench between two waypoints 5 cm apart,
and it degrades into "as close as I can get" near singularities instead of
dividing by a singular matrix. Measured over 300 random reachable poses: 99%
solved, ~10 ms each, worst position error 0.1 mm.

### Three numbers worth knowing

- `TCP_OFFSET = 0.135` — the grasp point, mid-pad, from `ur5e_tool0`.
- `FINGER_TIP = 0.170` — how far the pads actually *reach*. **35 mm past the
  TCP.** Aim the TCP at a 50 mm block's centre and the fingertips are on the
  bench.
- `MAX_WRIST_REACH = 0.828` — what bounds a top-down grasp. Not 0.85: that is
  the *tool's* reach, and a straight-down grasp puts the *wrist* 220 mm above
  the point you asked for.

Use `lowest_tcp_over(surface_z)` rather than any of them directly.

## `arm.py` — the `Arm` helper

Wraps a node rather than subclassing one, so a script can own its node and
still hand the arm around. Everything blocking spins that node itself, so no
example needs an executor or a thread.

```python
arm = Arm(node); arm.wait_for_state()
arm.open_gripper()
arm.move_to(0.45, 0.0, 0.90)     # hover
arm.move_to(0.45, 0.0, 0.79)     # descend
arm.close_gripper()
print(arm.held_object)           # grasp_server's verdict
```

`move_to` returns three different things and they mean three different
things: `True` arrived, `False` did not get there in time, `None` there is no
IK solution at all.

There is **no trajectory planning and no collision checking**. Every move is
"solve IK, publish six numbers, wait for the joints to stop". That is why a
pick is always hover-then-descend: the straight line between two IK solutions
is straight in *joint* space, and in real space it bulges — enough to sweep
the gripper through the bench.

## The three servers

| Server | Exposes |
|---|---|
| `gripper_server` | `/open_gripper` `/close_gripper` `/rotate_gripper` |
| `move_to_pose_server` | `/move_to_home` `/move_to_ready` `/move_to_watch` `/move_to_stow` |
| `grasp_server` | `/grasp/state`, `/grasp/reset` |

All `std_srvs/Trigger`, all argument-free — the shape an LLM planner can be
handed a list of and use without inventing units.

### How a grasp actually works here

Gazebo Harmonic with DART will **not** hold a 50 mm cube between two rubber
pads. Measured: close the Robotiq on a block and the knuckle sails past the
block's width while the block squirts out sideways and ends up 5 cm away.

So a grasp is a real kinematic constraint. Each block has a gz
`DetachableJoint` declared against the arm's wrist link, and `grasp_server`
welds and breaks it. While welded the arm carries the block's mass, the
fingers still close around it, and opening still drops it.

Two asymmetries in that node are worth reading the source for:

- It **grabs on the commanded** knuckle angle, so the weld forms before the
  pads arrive and the block cannot be knocked away by its own grasp.
- It **releases on the measured** angle, so the pads are physically clear
  before the weld breaks. Releasing on the command fires the block 15 cm
  across the bench, because the stalled fingers are still pressing.

## `workspace` — run this before wondering why the arm ignored you

```bash
ros2 run agr_arm_tools workspace                 # a map of the bench
ros2 run agr_arm_tools workspace 0.45 0.18 0.78  # one point, yes or no
```

Pure kinematics, no ROS, no simulator. It is the arm's equivalent of the
UAV platform's `list_airframes`: the thing to check first when a target is
being refused.
