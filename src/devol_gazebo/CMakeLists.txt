cmake_minimum_required(VERSION 3.10)
project(devol_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY sdf/ DESTINATION share/${PROJECT_NAME}/sdf)

# Puts share/devol_gazebo/worlds on GZ_SIM_RESOURCE_PATH when the workspace is sourced.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
