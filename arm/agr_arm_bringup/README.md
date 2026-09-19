# `agr_arm_bringup`

Launch files and the ROS 2 ↔ Gazebo topic bridge for the bench arm.

```bash
agr-sim arm                     # or: ros2 launch agr_arm_bringup sim.launch.py
agr-sim arm tools:=true         # ...with the three service servers
agr-sim arm headless:=true      # ...no GUI (CI, dataset recording)
agr-sim arm --world-only        # the workcell with no robot in it
```

## There is no controller_manager here

Every joint is driven by a gz `JointPositionController` plugin declared in
the URDF, so the arm holds its home pose from the first physics step and
there is no activation ordering to lose a race to. Everything commandable is
a `std_msgs/Float64` on a topic, which means the whole robot can be driven
from the command line:

```bash
ros2 topic pub --once /ur5e_elbow_joint/cmd std_msgs/msg/Float64 "{data: 1.2}"
```

The legged platform pays the `ros2_control` tax because a 12-joint gait needs
one synchronised command vector. A teaching arm does not.

## `sim.launch.py` patches the URDF on the way through

`damp_mimic_joints()` rewrites the four Robotiq joints that carry `<mimic>`
tags before the model is spawned. DART does not implement mimic constraints,
so those joints are unconstrained and undamped as far as the physics engine
is concerned, and the spawn impulse spins them until the fingers visibly fly
apart.

Two edits per joint: `continuous` → `revolute` with explicit limits, and a
`<dynamics damping="20" friction="1">` element. The damping value is a
balance, not a large-is-safe knob — it is paired with the knuckle
controllers' 20 N·m authority in `agr_arm_gazebo.urdf.xacro`, and changing
one means re-checking the other. Too high and the gripper takes ten seconds
to close; too low and the free fingers shake `wrist_3` into a limit cycle.

## What is bridged, and what is not

`config/ros_gz_bridge.yaml`. Note there is **no `/tf` entry**, unlike the
ground platform: the arm's base is bolted to the world, so the whole
transform tree is a function of `/joint_states` alone and
`robot_state_publisher` produces all of it.

`/joint_states` arrives at the **physics rate**, near 1 kHz — gz-sim 8's
`JointStatePublisher` has no `update_rate` parameter, whatever an SDF tag
might suggest. A `spin_once` loop reading it iterates about a thousand times
a second, so anything printing per message has to throttle itself.
