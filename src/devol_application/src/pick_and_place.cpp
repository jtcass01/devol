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
    ur_manipulator_group_interface.setMaxVelocityScalingFactor(0.1);
    ur_manipulator_group_interface.setMaxAccelerationScalingFactor(0.1);

    RCLCPP_INFO(logger, "Planning pipeline: %s", ur_manipulator_group_interface.getPlanningPipelineId().c_str());
    RCLCPP_INFO(logger, "Planner ID: %s", ur_manipulator_group_interface.getPlannerId().c_str());
    RCLCPP_INFO(logger, "Planning time: %.2f", ur_manipulator_group_interface.getPlanningTime());

    // Load and configure hand move group
    auto hand_group_interface = MoveGroupInterface(node, "hand");
    
    // PLACE TARGET POSE: Husky: (1.71, 0.2, 0.35)
    auto const place_pose = [&node] {
        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.frame_id = "base_link";
        target_pose_msg.header.stamp = node->now();
        target_pose_msg.pose.position.x = 0.2;
        target_pose_msg.pose.position.y = -1.05;
        target_pose_msg.pose.position.z = 1.05;
        target_pose_msg.pose.orientation.x = 0.0;
        target_pose_msg.pose.orientation.y = 1.0;
        target_pose_msg.pose.orientation.z = 0.0;
        target_pose_msg.pose.orientation.w = 0.0;
        return target_pose_msg;
    }();
    ur_manipulator_group_interface.setPoseTarget(place_pose);

    // Add Floor
    auto const floor = [frame_id = ur_manipulator_group_interface.getPlanningFrame(), &node, &logger] {
        // Print the planning frame for debugging purposes
        RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

        // Add Floor
        moveit_msgs::msg::CollisionObject floor;
        floor.header.frame_id = frame_id;
        floor.header.stamp = node->now();
        floor.id = "floor";

        // Define the shape of the object
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        // Set the dimensions of the box (in meters)
        primitive.dimensions[primitive.BOX_X] = 10.0; // Width
        primitive.dimensions[primitive.BOX_Y] = 10.0; // Depth
        primitive.dimensions[primitive.BOX_Z] = 0.01; // Height

        // Set the position of the box center
        geometry_msgs::msg::Pose pose;

        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = -0.305; 

        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        floor.primitives.push_back(primitive);
        floor.primitive_poses.push_back(pose);
        floor.operation = floor.ADD;

        RCLCPP_INFO(logger, "Floor collision object created with ID: %s", floor.id.c_str());
        RCLCPP_INFO(logger, "Floor dimensions: %.2f x %.2f x %.2f", primitive.dimensions[primitive.BOX_X], primitive.dimensions[primitive.BOX_Y], primitive.dimensions[primitive.BOX_Z]);
        RCLCPP_INFO(logger, "Floor position: (%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z);
        
        return floor;
    }();

    // Add Stand
    auto const stand = [frame_id = ur_manipulator_group_interface.getPlanningFrame(), &node, &logger] {
        // Print the planning frame for debugging purposes
        RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

        // Add Floor
        moveit_msgs::msg::CollisionObject stand;
        stand.header.frame_id = frame_id;
        stand.header.stamp = node->now();
        stand.id = "stand";

        // Define the shape of the object
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);

        // Set the dimensions of the box (in meters)
        primitive.dimensions[0] = 0.3; // Radius
        primitive.dimensions[1] = 0.09; // Height

        // Set the position of the box center
        geometry_msgs::msg::Pose pose;

        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = -0.15; 

        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        stand.primitives.push_back(primitive);
        stand.primitive_poses.push_back(pose);
        stand.operation = stand.ADD;

        RCLCPP_INFO(logger, "Stand collision object created with ID: %s", stand.id.c_str());
        RCLCPP_INFO(logger, "Dimensions: %.2f x %.2f x %.2f", primitive.dimensions[primitive.BOX_X], primitive.dimensions[primitive.BOX_Y], primitive.dimensions[primitive.BOX_Z]);
        RCLCPP_INFO(logger, "Position: (%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z);
        
        return stand;
    }();

    // Add Table
    auto const table = [frame_id = ur_manipulator_group_interface.getPlanningFrame(), &node, &logger] {
        // Print the planning frame for debugging purposes
        RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

        // Add Table
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = frame_id;
        table.header.stamp = node->now();
        table.id = "table";

        // Define the shape of the object
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        // Set the dimensions of the box (in meters)
        primitive.dimensions[primitive.BOX_X] = 2.5; // Width
        primitive.dimensions[primitive.BOX_Y] = 2.0; // Depth
        primitive.dimensions[primitive.BOX_Z] = 1.05; // Height

        // Set the position of the box center
        geometry_msgs::msg::Pose pose;

        pose.position.x = -1.0;
        pose.position.y = -2.0;
        pose.position.z = 0.225; // was 0.225

        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        table.primitives.push_back(primitive);
        table.primitive_poses.push_back(pose);
        table.operation = table.ADD;

        RCLCPP_INFO(logger, "Table collision object created with ID: %s", table.id.c_str());
        RCLCPP_INFO(logger, "dimensions: %.2f x %.2f x %.2f", primitive.dimensions[primitive.BOX_X], primitive.dimensions[primitive.BOX_Y], primitive.dimensions[primitive.BOX_Z]);
        RCLCPP_INFO(logger, "position: (%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z);
        
        return table;
    }();

    // Add Boxes
    auto const boxes = [frame_id = ur_manipulator_group_interface.getPlanningFrame(), &node, &logger] {
        // Print the planning frame for debugging purposes
        RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

        // Add Floor
        moveit_msgs::msg::CollisionObject boxes;
        boxes.header.frame_id = frame_id;
        boxes.header.stamp = node->now();
        boxes.id = "boxes";

        // Define the shape of the object
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        // Set the dimensions of the box (in meters)
        primitive.dimensions[primitive.BOX_X] = 2.75; // Width
        primitive.dimensions[primitive.BOX_Y] = 2.25; // Depth
        primitive.dimensions[primitive.BOX_Z] = 2.0; // Height

        // Set the position of the box center
        geometry_msgs::msg::Pose pose;

        pose.position.x = -1.573677;
        pose.position.y = 2.301994;
        pose.position.z = 0.75; 

        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        boxes.primitives.push_back(primitive);
        boxes.primitive_poses.push_back(pose);
        boxes.operation = boxes.ADD;

        RCLCPP_INFO(logger, "Boxes collision object created with ID: %s", boxes.id.c_str());
        RCLCPP_INFO(logger, "dimensions: %.2f x %.2f x %.2f", primitive.dimensions[primitive.BOX_X], primitive.dimensions[primitive.BOX_Y], primitive.dimensions[primitive.BOX_Z]);
        RCLCPP_INFO(logger, "position: (%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z);
        
        return boxes;
    }();

    // Add Representative Husky
    auto const husky = [frame_id = ur_manipulator_group_interface.getPlanningFrame(), &node, &logger] {
        // Print the planning frame for debugging purposes
        RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

        // Add Floor
        moveit_msgs::msg::CollisionObject husky;
        husky.header.frame_id = frame_id;
        husky.header.stamp = node->now();
        husky.id = "husky";

        // Define the shape of the object
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        // Set the dimensions of the box (in meters)
        primitive.dimensions[primitive.BOX_X] = 0.5; // Width
        primitive.dimensions[primitive.BOX_Y] = 1.0; // Depth
        primitive.dimensions[primitive.BOX_Z] = 0.4; // Height

        // Set the position of the box center
        geometry_msgs::msg::Pose pose;

        pose.position.x = 1.71;
        pose.position.y = 0.2;
        pose.position.z = -0.1; 

        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        husky.primitives.push_back(primitive);
        husky.primitive_poses.push_back(pose);
        husky.operation = husky.ADD;

        RCLCPP_INFO(logger, "Boxes collision object created with ID: %s", husky.id.c_str());
        RCLCPP_INFO(logger, "dimensions: %.2f x %.2f x %.2f", primitive.dimensions[primitive.BOX_X], primitive.dimensions[primitive.BOX_Y], primitive.dimensions[primitive.BOX_Z]);
        RCLCPP_INFO(logger, "position: (%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z);
        
        return husky;
    }();

    //Add the target block to be picked up
    auto const target_block = [frame_id = ur_manipulator_group_interface.getPlanningFrame(), &node, &logger] {
        // Print the planning frame for debugging purposes
        RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());

        // Add target block
        moveit_msgs::msg::CollisionObject target_block;
        target_block.header.frame_id = frame_id;
        target_block.header.stamp = node->now();
        target_block.id = "target_block";

        // Define the shape of the object
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        // Set the dimensions of the box (in meters)
        primitive.dimensions[primitive.BOX_X] = 0.075; // Width
        primitive.dimensions[primitive.BOX_Y] = 0.075; // Depth
        primitive.dimensions[primitive.BOX_Z] = 0.075; // Height

        // Set the position of the box center
        geometry_msgs::msg::Pose pose;

        pose.position.x = 0.2;
        pose.position.y = -1.05;
        pose.position.z = 0.7875; // was 0.7875

        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        target_block.primitives.push_back(primitive);
        target_block.primitive_poses.push_back(pose);
        target_block.operation = target_block.ADD;

        RCLCPP_INFO(logger, "Boxes collision object created with ID: %s", target_block.id.c_str());
        RCLCPP_INFO(logger, "dimensions: %.2f x %.2f x %.2f", primitive.dimensions[primitive.BOX_X], primitive.dimensions[primitive.BOX_Y], primitive.dimensions[primitive.BOX_Z]);
        RCLCPP_INFO(logger, "position: (%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z);
        
        return target_block;
    }();


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(floor);
    planning_scene_interface.applyCollisionObject(stand);
    planning_scene_interface.applyCollisionObject(table);
    planning_scene_interface.applyCollisionObject(boxes);
    planning_scene_interface.applyCollisionObject(husky);
    planning_scene_interface.applyCollisionObject(target_block);

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
    
    // Open the gripper. 0.0-open, 0.8-close
    hand_group_interface.setJointValueTarget({{"robotiq_85_left_knuckle_joint", 0.0}});
    hand_group_interface.move();

    rclcpp::shutdown();

    spinner.join();

    return 0;
}