cmake_minimum_required(VERSION 3.8)
project(agr_legged_description)
find_package(ament_cmake REQUIRED)
# meshes/ is fetched by fetch_meshes.sh and gitignored; install it only if present
install(DIRECTORY urdf config DESTINATION share/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
endif()
install(PROGRAMS fetch_meshes.sh DESTINATION share/${PROJECT_NAME})
# Gazebo rewrites package://<pkg>/… to model://<pkg>/… and then resolves it
# against GZ_SIM_RESOURCE_PATH. So the path must contain the directory that
# CONTAINS the package — share/ — not share/<pkg>. Without this the robot spawns
# and behaves correctly but renders as nothing at all, because collisions are
# primitives and only the visuals need the meshes. Silent, and confusing.
ament_environment_hooks("hook/gz_resource_path.dsv.in")

ament_package()
