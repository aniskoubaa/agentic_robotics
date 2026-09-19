# `agr_arm_examples`

Six short scripts, one idea each. Read them in order; each is meant to be read
in full in under a minute.

Start the robot first:

```bash
agr-sim arm tools:=true
```

| # | Command | The one idea |
|---|---|---|
| 01 | `ros2 run agr_arm_examples 01_read_joints` | an arm IS six numbers — and they do not tell you where the hand is |
| 02 | `ros2 run agr_arm_examples 02_move_a_joint` | one Float64 drives one motor, and the hand travels an arc |
| 03 | `ros2 run agr_arm_examples 03_get_image` | eye-in-hand versus eye-to-hand, in two PNGs |
| 04 | `ros2 run agr_arm_examples 04_move_to_pose` | a stored pose always works and can never adapt |
| 05 | `ros2 run agr_arm_examples 05_move_to_xyz` | inverse kinematics: many answers, or none |
| 06 | `ros2 run agr_arm_examples 06_pick_and_place` | manipulation is sequencing |

## The spine of the set is 04 versus 05

They do the same kind of thing — put the arm somewhere — and they are the two
halves of the subject.

`04` sends six stored numbers. It cannot fail, cannot be asked for something
impossible, and cannot react to anything. Right for "go to the start", "get
out of the way", "park".

`05` is told a point in metres and has to solve for the joints. It can return
no answer at all, and it can return a different answer than you expected. Run
it once inside the workspace and once outside:

```bash
ros2 run agr_arm_examples 05_move_to_xyz --ros-args -p x:=0.45 -p y:=0.18 -p z:=0.90
ros2 run agr_arm_examples 05_move_to_xyz --ros-args -p x:=0.95
```

The second prints why, not just no.

## Things these will show you that the text will not

**A joint's effect depends on where it is in the chain.** Run 02 twice with
the same angle:

```bash
ros2 run agr_arm_examples 02_move_a_joint --ros-args -p joint:=shoulder_pan -p angle:=0.4
ros2 run agr_arm_examples 02_move_a_joint --ros-args -p joint:=wrist_3 -p angle:=0.4
```

Measured: the shoulder moves the hand 23 cm, the wrist moves it 0.0 cm. Same
0.4 radians.

**The reach on the datasheet is not the reach you get.** A UR5e reaches
850 mm, but a straight-down grasp puts the wrist 220 mm above the point you
asked for, and it is the wrist that must be within reach. See the map:

```bash
ros2 run agr_arm_tools workspace
```

**Your tool frame is not your fingertips.** The pads reach 35 mm past the
tool centre point, so aiming at a 50 mm block's centre puts them on the
bench. `06` uses `kinematics.lowest_tcp_over()` rather than guessing, and
says so as it runs.

## 06 needs the tool servers

Without `grasp_server` the gripper closes on nothing and the block stays put.
`06` notices and tells you. Start everything with `agr-sim arm tools:=true`,
or check with `ros2 run agr_arm_demos diagnose`.
