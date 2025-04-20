cmake_minimum_required(VERSION 3.5)

project(devol_description)

find_package(ament_cmake REQUIRED)
find_package(ur_description REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY
    urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY
    rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    srdf/
  DESTINATION share/${PROJECT_NAME}/srdf
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
