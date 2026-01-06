#include "devol_drive_robot.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <memory>
#include <thread>

int main(int argc, char *argv[])
{
    int sleep_between_steps = 2500; // milliseconds

    // Start up ROS 2
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>("devol_drive_demo_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger = rclcpp::get_logger("devol_drive_demo_node");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() {
        executor.spin();
    });

    // Apollo
    moveit::planning_interface::MoveGroupInterface apollo_ur_manipulator_group_interface = moveit::planning_interface::MoveGroupInterface(node, "apollo_ur_manipulator");
    moveit::planning_interface::MoveGroupInterface apollo_hand_group_interface = moveit::planning_interface::MoveGroupInterface(node, "apollo_hand");
    moveit::planning_interface::PlanningSceneInterface apollo_planning_scene_interface;

    // Artemis
    moveit::planning_interface::MoveGroupInterface artemis_ur_manipulator_group_interface = moveit::planning_interface::MoveGroupInterface(node, "artemis_ur_manipulator");
    moveit::planning_interface::MoveGroupInterface artemis_hand_group_interface = moveit::planning_interface::MoveGroupInterface(node, "artemis_hand");
    moveit::planning_interface::PlanningSceneInterface artemis_planning_scene_interface;

    // // Load and configure the planning scene interface.
    // initialize_collision_objects(ur_manipulator_group_interface, planning_scene_interface, logger, node);

    // // Create a robot object, includes going to ready position
    DevolDriveRobot apollo_robot(apollo_ur_manipulator_group_interface, 
                                 apollo_hand_group_interface, 
                                 apollo_planning_scene_interface, 
                                 logger, node, "apollo_");
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    DevolDriveRobot artemis_robot(artemis_ur_manipulator_group_interface, 
                                  artemis_hand_group_interface, 
                                  artemis_planning_scene_interface, 
                                  logger, node, "artemis_");
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));

    apollo_robot.publish_drive_command(0, -5.0);
    artemis_robot.publish_drive_command(0, -5.0);
    
    rclcpp::shutdown();

    spinner.join();

    return 0;
}
