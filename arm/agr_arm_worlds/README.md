# `agr_arm_worlds`

One world: `agr_workcell.sdf`. A bench, three graspable blocks and a drop
tray. Primitives only, so it loads offline in under a second.

Deliberately the opposite of the greenhouse and the inspection site: nothing
here is more than a metre away, and the robot cannot go and get it. A
manipulation world is defined by what falls inside one 850 mm sphere, so what
it teaches is the **workspace** — where the numbers you type in metres
actually correspond to a reachable arm configuration.

## Layout (metres, world frame, floor at z = 0)

| Thing | Where |
|---|---|
| pedestal | the origin; arm base flange at z = 0.760 |
| bench top | x ∈ [0.15, 1.05], y ∈ [−0.40, 0.40], surface z = 0.750 |
| blocks | x = 0.45, y = +0.18 / 0.00 / −0.18, resting at z = 0.776 |
| drop tray | centred (0.66, 0.30), 0.20 × 0.20, 0.03 walls |

Everything the arm is asked to pick sits 0.44–0.50 m from the base, clear of
both the singular column overhead and the reach limit.

## Two things about the SDF that bite

**Naming any `<plugin>` replaces gz sim's whole default set** rather than
adding to it, so every system the world needs is listed explicitly. Miss
`Sensors` and the URDF's cameras advertise nothing at all — no error, just
topics that never appear.

**The physics step is 1 ms**, not the 2 ms the legged world uses. A 5 cm
block between two gripper pads is a small, stiff contact and it visibly
jitters at 2 ms. It also means `/joint_states` arrives at about 1 kHz, since
gz-sim's joint state publisher has no rate limit of its own.

## Blocks are top-level models

Not links inside a bigger model, and that is load-bearing in two places:
`grasp_server` teleports them by model name during a scene reset, and only
top-level entities are reported in the world frame by
`/world/<w>/dynamic_pose/info`.
