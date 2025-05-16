#include "devol_robot.h"

Robot::Robot(moveit::planning_interface::MoveGroupInterface &ur_manipulator_group_interface_,
             moveit::planning_interface::MoveGroupInterface &hand_group_interface,
             moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
             const rclcpp::Logger &logger,
             const rclcpp::Node::SharedPtr &node) :
             ur_manipulator_group_interface_(ur_manipulator_group_interface_),
             hand_group_interface_(hand_group_interface),
             planning_scene_interface_(planning_scene_interface),
             logger_(logger),
             node_(node)
{
    ur_manipulator_group_interface_.setPlanningPipelineId("ompl");
    ur_manipulator_group_interface_.setPlannerId("LBKPIECEkConfigDefault");
    ur_manipulator_group_interface_.setMaxVelocityScalingFactor(0.5);
    ur_manipulator_group_interface_.setMaxAccelerationScalingFactor(0.5);
    ur_manipulator_group_interface_.setGoalPositionTolerance(0.01);
    ur_manipulator_group_interface_.setGoalOrientationTolerance(0.05);
    ur_manipulator_group_interface_.setPlanningTime(30.0);

    while (pub_detach_.HasConnections() == false)
    {
        RCLCPP_INFO(logger_, "Waiting for connection to detach topic");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    while (pub_attach_.HasConnections() == false)
    {
        RCLCPP_INFO(logger_, "Waiting for connection to attach topic");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    this->detach_object_from_end_effector();
    this->go_home();
    this->set_gripper_position(GRIPPER_POSITION::OPEN);

    // Initialize the robot
    RCLCPP_INFO(logger_, "Robot initialized. In Ready Position.");
}


Robot::~Robot(){}

bool Robot::go_home()
{
    RCLCPP_INFO(logger_, "Going to ready position");
    ur_manipulator_group_interface_.setNamedTarget("ready");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode ok = ur_manipulator_group_interface_.plan(plan);

    if (ok)
    {
        ur_manipulator_group_interface_.execute(plan);
        RCLCPP_INFO(logger_, "Planning succeeded");
        return true;
    }

    RCLCPP_ERROR(logger_, "Planning failed");
    return false;
}

bool Robot::set_manipulator_goal(const geometry_msgs::msg::Pose &pose)
{
    ur_manipulator_group_interface_.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode ok = ur_manipulator_group_interface_.plan(plan);

    if (ok)
    {
        ur_manipulator_group_interface_.execute(plan);
        RCLCPP_INFO(logger_, "Planning succeeded");
        return true;
    }

    RCLCPP_ERROR(logger_, "Planning failed");
    return false;
}

void Robot::set_gripper_position(GRIPPER_POSITION position)
{
    switch(position)
    {
        case GRIPPER_POSITION::OPEN:
            RCLCPP_INFO(logger_, "Setting gripper to OPEN");
            hand_group_interface_.setJointValueTarget({{"robotiq_85_left_knuckle_joint", 0.0}});
            break;
        case GRIPPER_POSITION::CLOSE:
            RCLCPP_INFO(logger_, "Setting gripper to CLOSE");
            hand_group_interface_.setJointValueTarget({{"robotiq_85_left_knuckle_joint", 0.8}});
            break;
        case GRIPPER_POSITION::GRAB:
            RCLCPP_INFO(logger_, "Setting gripper to GRAB");
            hand_group_interface_.setJointValueTarget({{"robotiq_85_left_knuckle_joint", 0.105}});
            break;
        default:
            RCLCPP_ERROR(logger_, "Invalid gripper position");
            return;
    }
    hand_group_interface_.move();
}

void Robot::attach_object(const std::string &object_id)
{
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = ur_manipulator_group_interface_.getEndEffectorLink();
    attached_object.object.id = object_id;
    attached_object.object.header.frame_id = ur_manipulator_group_interface_.getPlanningFrame();
    attached_object.object.header.stamp = node_->now();
    attached_object.object.operation = attached_object.object.ADD;
    attached_object.touch_links = hand_group_interface_.getLinkNames();

    planning_scene_interface_.applyAttachedCollisionObject(attached_object);
    RCLCPP_INFO(logger_, "Attached the object to the end effector.");
}

void Robot::detach_object(const std::string &object_id)
{
    moveit_msgs::msg::AttachedCollisionObject detached_object;
    detached_object.link_name = ur_manipulator_group_interface_.getEndEffectorLink();
    detached_object.object.id = object_id;
    detached_object.object.header.frame_id = ur_manipulator_group_interface_.getPlanningFrame();
    detached_object.object.header.stamp = node_->now();
    detached_object.object.operation = detached_object.object.REMOVE;

    planning_scene_interface_.applyAttachedCollisionObject(detached_object);
    RCLCPP_INFO(logger_, "Detached the object from the end effector.");
}

void Robot::allow_collision_between(const std::string &object1, const std::string &object2)
{
    moveit_msgs::msg::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;

    // Add entry names
    planning_scene_msg.allowed_collision_matrix.entry_names.push_back(object1);
    planning_scene_msg.allowed_collision_matrix.entry_names.push_back(object2);

    // Create entries to allow collision
    moveit_msgs::msg::AllowedCollisionEntry entry;
    entry.enabled = {true, true};  // allow object1 <-> object2

    planning_scene_msg.allowed_collision_matrix.entry_values.push_back(entry);
    planning_scene_msg.allowed_collision_matrix.entry_values.push_back(entry);

    // Apply the planning scene update
    planning_scene_interface_.applyPlanningScene(planning_scene_msg);

    RCLCPP_INFO(logger_, "Allowed collision between %s and %s", object1.c_str(), object2.c_str());
}

void Robot::attach_object_to_end_effector()
{
    pub_attach_.Publish(gz::msgs::Empty());
}

void Robot::detach_object_from_end_effector()
{
    pub_detach_.Publish(gz::msgs::Empty());
}