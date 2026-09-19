# `raisebot_description`

URDF/xacro for the RaiseBot robot platform: **Husky A200 + UR5e + Robotiq 2F-85 + RealSense D435**.

## Contents (to be filled)

- `urdf/raisebot_robot.urdf.xacro` — top-level robot, composes upstream macros
- `urdf/husky_arm_mount.urdf.xacro` — UR5e mounting plate on top of the Husky
- `meshes/` — STL/DAE for any custom links not provided by upstream packages (e.g. the mount plate)
- `config/joint_limits.yaml` — sim-side overrides for arm joint limits

## Upstream packages used (installed via rosdep, not vendored)

- `clearpath_platform_description` (Clearpath A200/Husky)
- `ur_description` (Universal Robots)
- `realsense2_description` (Intel RealSense)
- `robotiq_description` — gripper (verify availability on Jazzy; may need a thin local xacro)
