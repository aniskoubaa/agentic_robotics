# `raise2026_tools`

ROS 2 services that the LLM agent calls as **tools**.

## Planned tool services

Use stock interfaces (`std_srvs`, `geometry_msgs`, `nav2_msgs`) where possible. Custom `.srv` only when no stock fit exists.

| Service               | Type                                | First used  |
| --------------------- | ----------------------------------- | ----------- |
| `nav_to_row`          | `nav2_msgs/NavigateToPose`          | D1L1        |
| `move_to_pose`        | custom (arm Cartesian goal)         | D1L1        |
| `open_gripper`        | `std_srvs/Trigger`                  | D1L1        |
| `close_gripper`       | `std_srvs/Trigger`                  | D1L1        |
| `get_scene_objects`   | custom (returns object list + pose) | D1L2        |
| `inspect_plant`       | custom (VLM-backed ripeness report) | D2L1        |

## Custom srv files

Go under `srv/`. **Adding any custom interface promotes this package from `ament_python` to `ament_cmake`** (rosidl requirement). If we add custom types, the cleaner move is to split a `raise2026_interfaces` package and keep this one Python-only.
