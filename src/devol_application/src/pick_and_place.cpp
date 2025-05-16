#include "devol_robot.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <memory>
#include <thread>

moveit_msgs::msg::CollisionObject createCollisionObject(const std::string &id, 
                                                        const std::string &frame_id, 
                                                        const shape_msgs::msg::SolidPrimitive &primitive, 
                                                        const geometry_msgs::msg::Pose &pose,
                                                        const rclcpp::Logger &logger,
                                                        const rclcpp::Node::SharedPtr &node)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = id;
    collision_object.header.frame_id = frame_id;
    collision_object.header.stamp = node->now();

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    RCLCPP_INFO(logger, "%s collision object created with ID: %s", id.c_str(), collision_object.id.c_str());
    RCLCPP_INFO(logger, "Position: (%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z);

    return collision_object;
}


void initialize_collision_objects(moveit::planning_interface::MoveGroupInterface &ur_manipulator_group_interface,
                                  moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                                  const rclcpp::Logger &logger,
                                  const rclcpp::Node::SharedPtr &node)
{
    // Add Floor
    moveit_msgs::msg::CollisionObject floor = [&]() {
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {10.0, 10.0, 0.01}; // Width, Depth, Height

        // Set the position of the box center
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = -0.305; 
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        return createCollisionObject("floor", ur_manipulator_group_interface.getPlanningFrame(), primitive, pose, logger, node);
    }();
    planning_scene_interface.applyCollisionObject(floor);

    // Add Stand
    moveit_msgs::msg::CollisionObject stand = [&]() {
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions = {0.3, 0.09};
    
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = -0.15;
        pose.orientation.w = 1.0;
    
        return createCollisionObject("stand", ur_manipulator_group_interface.getPlanningFrame(), primitive, pose, logger, node);
    }();
    planning_scene_interface.applyCollisionObject(stand);

    // Add Table
    moveit_msgs::msg::CollisionObject table = [&] {

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {2.5, 2.0, 1.05};
    
        geometry_msgs::msg::Pose pose;
        pose.position.x = -1.2;
        pose.position.y = -2.35;
        pose.position.z = 0.225;
        pose.orientation.w = 1.0;
    
        return createCollisionObject("table", ur_manipulator_group_interface.getPlanningFrame(), primitive, pose, logger, node);
    }();
    planning_scene_interface.applyCollisionObject(table);

    // Add Boxes
    moveit_msgs::msg::CollisionObject boxes = [&]() {
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {2.75, 2.25, 2.0};
    
        geometry_msgs::msg::Pose pose;
        pose.position.x = -1.573677;
        pose.position.y = 2.301994;
        pose.position.z = 0.75;
        pose.orientation.w = 1.0;
    
        return createCollisionObject("boxes", ur_manipulator_group_interface.getPlanningFrame(), primitive, pose, logger, node);
    }();
    planning_scene_interface.applyCollisionObject(boxes);

    // Add Representative Husky
    moveit_msgs::msg::CollisionObject husky = [&] {
        // Define the shape of the object
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.5, 1.0, 0.4}; // Width, Depth, Height

        // Set the position of the box center
        geometry_msgs::msg::Pose pose;
        pose.position.x = 1.71;
        pose.position.y = 0.2;
        pose.position.z = -0.1; 
        pose.orientation.w = 1.0;

        return createCollisionObject("husky", ur_manipulator_group_interface.getPlanningFrame(), primitive, pose, logger, node);
    }();
    planning_scene_interface.applyCollisionObject(husky);

    //Add the target block to be picked up
    moveit_msgs::msg::CollisionObject target_block = [&] {
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.055, 0.055, 0.075}; // Width, Depth, Height

        // Set the position of the box center
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0;
        pose.position.y = -1.4;
        pose.position.z = 0.788;
        pose.orientation.w = 1.0;

        return createCollisionObject("target_block", ur_manipulator_group_interface.getPlanningFrame(), primitive, pose, logger, node);
    }();
    planning_scene_interface.applyCollisionObject(target_block);
}

geometry_msgs::msg::Pose make_pose(double x, double y, double z, double qx, double qy, double qz, double qw)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;

    return pose;
}


int main(int argc, char *argv[])
{
    int sleep_between_steps = 2500; // milliseconds

    // Start up ROS 2
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>("pick_and_place_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger = rclcpp::get_logger("pick_and_place_node");

    geometry_msgs::msg::Pose above_pickup_target_pose = make_pose(0, -1.4, 1.05, 0.0, 1.0, 0.0, 0.0);
    geometry_msgs::msg::Pose pickup_target_pose = make_pose(0, -1.4, 0.95, 0.0, 1.0, 0.0, 0.0);
    geometry_msgs::msg::Pose above_place_target_pose = make_pose(1.71, 0.2, 0.40, 1.0, 0.0, 0.0, 0.0);
    geometry_msgs::msg::Pose place_target_pose = make_pose(1.71, 0.2, 0.35, 1.0, 0.0, 0.0, 0.0);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() {
        executor.spin();
    });

    // Load and configure ur_manipulator move group
    moveit::planning_interface::MoveGroupInterface ur_manipulator_group_interface = moveit::planning_interface::MoveGroupInterface(node, "ur_manipulator");

    // Load and configure hand move group
    moveit::planning_interface::MoveGroupInterface hand_group_interface = moveit::planning_interface::MoveGroupInterface(node, "hand");

    // Load and configure the planning scene interface.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    initialize_collision_objects(ur_manipulator_group_interface, planning_scene_interface, logger, node);

    // Create a robot object, includes going to ready position
    Robot robot(ur_manipulator_group_interface, hand_group_interface, planning_scene_interface, logger, node);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));

    // Go to above pick up position
    RCLCPP_INFO(logger, "Going to above pickup position");
    bool success = robot.set_manipulator_goal(above_pickup_target_pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));

    // Go to pick up position
    if (success)
    {
        RCLCPP_INFO(logger, "Going to pickup position");
        success = robot.set_manipulator_goal(pickup_target_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    }

    if (success)
    {
        // Attach the object to the robot's end effector
        robot.attach_object("target_block");
        RCLCPP_INFO(logger, "Attached the object to the end effector.");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Close the gripper
        RCLCPP_INFO(logger, "Setting gripper to grab position");
        robot.set_gripper_position(GRIPPER_POSITION::GRAB);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));

        robot.attach_object_to_end_effector();
    }

    if (success)
    {
        // Go to above pick up position
        RCLCPP_INFO(logger, "Going to above pickup position");
        robot.set_manipulator_goal(above_pickup_target_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    }

    // Go Home
    if (success)
    {
        RCLCPP_INFO(logger, "Going to home position");
        robot.go_home();
        RCLCPP_INFO(logger, "Returned to home position.");
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    }
    
    if (success)
    {
        // Go to above place position
        RCLCPP_INFO(logger, "Going to above place position");
        success = robot.set_manipulator_goal(above_place_target_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    }

    if (success)
    {
        // Go to place position
        RCLCPP_INFO(logger, "Going to place position");
        robot.set_manipulator_goal(place_target_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    }

    if (success)
    {
        robot.set_gripper_position(GRIPPER_POSITION::OPEN);
        RCLCPP_INFO(logger, "Gripper opened.");
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));

        // Detach the object from the robot's end effector
        robot.detach_object("target_block");
        robot.detach_object_from_end_effector();
        RCLCPP_INFO(logger, "Detached the object from the end effector.");

        // Go to above place position
        RCLCPP_INFO(logger, "Going to above place position");
        success = robot.set_manipulator_goal(above_place_target_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    }

    if (success)
    {
        RCLCPP_INFO(logger, "Going to home position");
        robot.go_home();
        RCLCPP_INFO(logger, "Returned to home position.");
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_between_steps));
    }
    
    rclcpp::shutdown();

    spinner.join();

    return 0;
}
