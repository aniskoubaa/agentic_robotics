cmake_minimum_required(VERSION 3.8)
project(raise2026_worlds)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds meshes
  DESTINATION share/${PROJECT_NAME}
)

# Auto-prepend our worlds + meshes to GZ_SIM_RESOURCE_PATH whenever the
# workspace is sourced. Without this, `<include><uri>model://tomato_plant</uri></include>`
# in the SDF falls back to Fuel and hangs at world load.
ament_environment_hooks("hook/gz_resource_path.dsv.in")

ament_package()
