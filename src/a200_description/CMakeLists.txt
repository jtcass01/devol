cmake_minimum_required(VERSION 3.10)
project(a200_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)

# Lets Gazebo resolve package://a200_description/meshes/... URIs
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
