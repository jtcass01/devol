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

    // // Go to above pick up position
    // RCLCPP_INFO(logger, "Going to above pickup position");
    // bool success = robot.set_manipulator_goal(above_pickup_target_pose);
    // std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));

    // // Go to pick up position
    // if (success)
    // {
    //     RCLCPP_INFO(logger, "Going to pickup position");
    //     success = robot.set_manipulator_goal(pickup_target_pose);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    // }

    // if (success)
    // {
    //     // Attach the object to the robot's end effector
    //     robot.attach_object("target_block_0");
    //     RCLCPP_INFO(logger, "Attached the object to the end effector.");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));

    //     // Close the gripper
    //     RCLCPP_INFO(logger, "Setting gripper to grab position");
    //     robot.set_gripper_position(GRIPPER_POSITION::GRAB);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));

    //     robot.attach_object_to_end_effector();
    // }

    // if (success)
    // {
    //     // Go to above pick up position
    //     RCLCPP_INFO(logger, "Going to above pickup position");
    //     robot.set_manipulator_goal(above_pickup_target_pose);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    // }

    // // Go Home
    // if (success)
    // {
    //     RCLCPP_INFO(logger, "Going to home position");
    //     robot.go_home();
    //     RCLCPP_INFO(logger, "Returned to home position.");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    // }
    
    // if (success)
    // {
    //     // Go to above place position
    //     RCLCPP_INFO(logger, "Going to above place position");
    //     success = robot.set_manipulator_goal(above_place_target_pose);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    // }

    // if (success)
    // {
    //     // Go to place position
    //     RCLCPP_INFO(logger, "Going to place position");
    //     robot.set_manipulator_goal(place_target_pose);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    // }

    // if (success)
    // {
    //     robot.set_gripper_position(GRIPPER_POSITION::OPEN);
    //     RCLCPP_INFO(logger, "Gripper opened.");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));

    //     // Detach the object from the robot's end effector
    //     robot.detach_object("target_block_0");
    //     robot.detach_object_from_end_effector();
    //     RCLCPP_INFO(logger, "Detached the object from the end effector.");

    //     // Go to above place position
    //     RCLCPP_INFO(logger, "Going to above place position");
    //     success = robot.set_manipulator_goal(above_place_target_pose);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    // }

    // if (success)
    // {
    //     RCLCPP_INFO(logger, "Going to home position");
    //     robot.go_home();
    //     RCLCPP_INFO(logger, "Returned to home position.");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    // }
    
    rclcpp::shutdown();

    spinner.join();

    return 0;
}
