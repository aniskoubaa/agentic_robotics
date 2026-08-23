cmake_minimum_required(VERSION 3.8)
project(agr_core)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/MissionPlan.msg"
  "msg/Violation.msg"
  "msg/MissionConstraints.msg"
  "msg/VerificationResult.msg"
  "srv/VerifyMission.srv"
)

ament_package()
