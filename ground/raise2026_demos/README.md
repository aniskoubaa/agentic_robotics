# `raise2026_demos`

One demo node per lecture, each **≤80 lines**. These are what `sim/demos/*.sh` wrap.

| Node                      | Lecture | What it demonstrates                                       |
| ------------------------- | ------- | ---------------------------------------------------------- |
| `d1l1_tools_demo`         | D1L1    | Direct service calls: `nav_to_row` + `move_to_pose` + gripper |
| `d1l2_agentic_inspector`  | D1L2    | Same services, but driven by LLM tool-calling              |
| `d2l1_teleop_record`      | D2L1    | Teleop + LeRobot dataset recording                         |
| `d2l2_vla_rollout`        | D2L2    | Remote VLA inference, action streaming to the arm          |
| `d3l1_planner_executor`   | D3L1    | LLM planner → tool calls → VLA skill calls                 |
