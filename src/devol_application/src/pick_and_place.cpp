#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char *argv[])
{
    // Start up ROS 2
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>("pick_and_place_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger = rclcpp::get_logger("pick_and_place_node");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() {
        executor.spin();
    });

    // Load and configure ur_manipulator move group
    using moveit::planning_interface::MoveGroupInterface;
    auto ur_manipulator_group_interface = MoveGroupInterface(node, "ur_manipulator");
    ur_manipulator_group_interface.setPlanningPipelineId("ompl");
    ur_manipulator_group_interface.setPlannerId("RRTConnectkConfigDefault");
    ur_manipulator_group_interface.setMaxVelocityScalingFactor(1.0);
    ur_manipulator_group_interface.setMaxAccelerationScalingFactor(1.0);

    RCLCPP_INFO(logger, "Planning pipeline: %s", ur_manipulator_group_interface.getPlanningPipelineId().c_str());
    RCLCPP_INFO(logger, "Planner ID: %s", ur_manipulator_group_interface.getPlannerId().c_str());
    RCLCPP_INFO(logger, "Planning time: %.2f", ur_manipulator_group_interface.getPlanningTime());

    // Load and configure hand move group
    auto hand_manipulator_group_interface = MoveGroupInterface(node, "hand");
    
    auto const ur_target_pose = [&node] {
        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.frame_id = "base_link";
        target_pose_msg.header.stamp = node->now();
        target_pose_msg.pose.position.x = 1.0;
        target_pose_msg.pose.position.y = 1.0;
        target_pose_msg.pose.position.z = 1.0;
        target_pose_msg.pose.orientation.x = 0.0;
        target_pose_msg.pose.orientation.y = 0.0;
        target_pose_msg.pose.orientation.z = 0.0;
        target_pose_msg.pose.orientation.w = 1.0;
        return target_pose_msg;
    }();
    ur_manipulator_group_interface.setPoseTarget(ur_target_pose);

    // Create collision targets
    auto const collision_object = [frame_id = ur_manipulator_group_interface.getPlanningFrame(), &node, &logger] {
        // Print the planning frame for debugging purposes
        RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.header.stamp = node->now();
        collision_object.id = "floor";

        // Define the shape of the object
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        // Set the dimensions of the box (in meters)
        primitive.dimensions[primitive.BOX_X] = 10.0; // Width
        primitive.dimensions[primitive.BOX_Y] = 10.0; // Depth
        primitive.dimensions[primitive.BOX_Z] = 0.01; // Height

        // Set the position of the box center
        geometry_msgs::msg::Pose floor_pose;

        floor_pose.position.x = -5.0;
        floor_pose.position.y = -5.0;
    }();

    auto const [success, plan] = [&ur_manipulator_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto const ok = ur_manipulator_group_interface.plan(plan);
        return std::make_pair(ok, plan);
    }();

    if (success)
    {
        ur_manipulator_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Planning succeeded");
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning failed");
    }

    rclcpp::shutdown();

    spinner.join();

    return 0;
}