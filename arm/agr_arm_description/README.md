# `agr_arm_description`

The bench-mounted UR5e: pedestal, arm, Robotiq 2F-85, and two cameras.

| File | Holds |
|---|---|
| `arm_pedestal.urdf.xacro` | the pedestal, and the fixed joint to `world` |
| `agr_arm_robot.urdf.xacro` | the composition — a clean kinematic description |
| `agr_arm_gazebo.urdf.xacro` | sensors, controllers, and the grasp constraints |

Deliberately the **same** arm and gripper that RaiseBot carries on its Husky
(`ground/raisebot_description`). That is the point: a student who has driven
the mobile manipulator meets an identical kinematic chain here with the base
bolted down, so everything about joints, poses, IK and grasping transfers in
both directions. What changes is only what the base does.

## Bolting it down changes the problem

There is no odometry, no `/cmd_vel`, no navigation. The robot's entire state
is six joint angles, the world frame and the base frame are the same thing
forever, and anything the arm cannot reach it *cannot reach* — on the Husky
you drive closer, here you have hit the edge of an 850 mm sphere.

URDF has no notion of "static". The convention every URDF→SDF converter
honours is a link literally named `world` with a fixed joint to it. Drop that
joint and the pedestal is a free body that tips over the moment the arm
accelerates.

## Two cameras, on purpose

- `wrist_camera` — **eye-in-hand**, moves with the tool, sees what the gripper
  is about to touch and nothing else.
- `bench_camera` — **eye-to-hand**, bolted to the workcell, always shows the
  whole table and is regularly blocked by the arm's own elbow.

The wrist mount took three attempts and the failures are recorded in the
file, because all three look correct on paper. The one that works is offset
along the tool's **+y** — perpendicular to the plane the fingers open in —
because a camera offset along +x looks straight down the line of one finger
and that finger is between the lens and the object every single time.

## Gains are per joint

RaiseBot drives this same arm with one set (1500/50/80) on all six. Measured
here, that puts the wrist joints in a limit cycle: `wrist_3` sat 0.11 rad off
target with its velocity flipping ±2.5 rad/s forever. Critical damping is
`2·sqrt(p·J)`, and `wrist_3` turns a gripper about its own axis while
`shoulder_lift` swings twenty kilograms — so the shoulder's `d_gain` is
twenty times critical at the wrist.

`i_max` is set just *under* `cmd_max` rather than to a small "safe" number. A
UR5e reaching out needs about 80 N·m at the shoulder to hold itself up; cap
the integral below that and the proportional term must supply the rest, which
it can only do *by sitting at an error*. At `i_max=30` the arm drooped
0.028 rad reaching for the bench — 32 mm of tool error, enough to miss a
50 mm block entirely.

## Grasping is a constraint, not friction

DART will not hold a 50 mm cube between two pads — the block squirts out
sideways every time. So each block has a `DetachableJoint` declared against
`ur5e_wrist_3_link`, and `grasp_server` welds and breaks it.

Two things about that block of the file are worth knowing before editing it:

- **One plugin per block.** `DetachableJoint` names its child model at load
  time, so this file has to know the names of the graspable things in
  `agr_workcell`. Add a block to the world, add a line here.
- **`parent_link` is `ur5e_wrist_3_link`, not the gripper base.** sdformat
  *folds* fixed joints, so everything from the flange out collapses into the
  wrist link before the plugin ever sees it. Naming a folded link gets you a
  plugin that silently does nothing. `gz model -m agr_arm_robot` prints the
  links that actually exist.
