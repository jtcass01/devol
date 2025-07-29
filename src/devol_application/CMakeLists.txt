cmake_minimum_required(VERSION 3.8)
project(devol_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(gz-msgs10 QUIET REQUIRED)
find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

# Add library for devol_robot
add_library(devol_robot src/devol_robot.cpp)
target_link_libraries(devol_robot ${GZ-MSGS_LIBRARIES} gz-transport${GZ_TRANSPORT_VER}::core)
ament_target_dependencies(devol_robot "rclcpp" "moveit_msgs" "geometry_msgs" "moveit_ros_planning_interface")

add_library(devol_drive_robot src/devol_drive_robot.cpp)
target_link_libraries(devol_drive_robot ${GZ-MSGS_LIBRARIES} gz-transport${GZ_TRANSPORT_VER}::core)
ament_target_dependencies(devol_drive_robot "rclcpp" "moveit_msgs" "geometry_msgs" "moveit_ros_planning_interface")

add_executable(pick_and_place_demo src/pick_and_place_demo.cpp)
target_link_libraries(pick_and_place_demo devol_robot)
ament_target_dependencies(pick_and_place_demo "rclcpp" "moveit_msgs" "geometry_msgs" "moveit_ros_planning_interface")

add_executable(devol_drive_demo src/devol_drive_demo.cpp)
target_link_libraries(devol_drive_demo devol_drive_robot devol_robot)
ament_target_dependencies(devol_drive_demo "rclcpp" "moveit_msgs" "geometry_msgs" "moveit_ros_planning_interface")

install(TARGETS pick_and_place_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS devol_drive_demo
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
