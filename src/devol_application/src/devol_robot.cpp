#include "devol_robot.h"

DevolRobot::DevolRobot(moveit::planning_interface::MoveGroupInterface &ur_manipulator_group_interface,
             moveit::planning_interface::MoveGroupInterface &hand_group_interface,
             moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
             const rclcpp::Logger &logger,
             const rclcpp::Node::SharedPtr &node,
             const std::string &tf_prefix) :
             ur_manipulator_group_interface_(ur_manipulator_group_interface),
             hand_group_interface_(hand_group_interface),
             planning_scene_interface_(planning_scene_interface),
             tf_prefix_(tf_prefix),
             logger_(logger),
             node_(node)
{
    std::string attach_topic = tf_prefix + "devol_attach/attach";
    std::string detach_topic = tf_prefix + "devol_attach/detach";
    gripper_joint_ = tf_prefix + "robotiq_85_left_knuckle_joint";

    ur_manipulator_group_interface_.setPlanningPipelineId("ompl");
    ur_manipulator_group_interface_.setPlannerId("LBKPIECEkConfigDefault");
    ur_manipulator_group_interface_.setMaxVelocityScalingFactor(1.0);
    ur_manipulator_group_interface_.setMaxAccelerationScalingFactor(1.0);
    ur_manipulator_group_interface_.setGoalPositionTolerance(0.01);
    ur_manipulator_group_interface_.setGoalOrientationTolerance(0.05);
    ur_manipulator_group_interface_.setPlanningTime(60.0);

    pub_attach_ = gz_node_.Advertise<gz::msgs::Empty>(attach_topic);
    pub_detach_ = gz_node_.Advertise<gz::msgs::Empty>(detach_topic);


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
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    this->go_home();
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    this->set_gripper_position(GRIPPER_POSITION::OPEN);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Initialize the DevolRobot
    RCLCPP_INFO(logger_, "DevolRobot initialized. In Ready Position.");
}


DevolRobot::~DevolRobot(){}

void DevolRobot::go_home()
{
    std::string elbow_joint = tf_prefix_ + "elbow_joint";
    std::string shoulder_lift_joint = tf_prefix_ + "shoulder_lift_joint";
    std::string shoulder_pan_joint = tf_prefix_ + "shoulder_pan_joint";
    std::string wrist_1_joint = tf_prefix_ + "wrist_1_joint";
    std::string wrist_2_joint = tf_prefix_ + "wrist_2_joint";
    std::string wrist_3_joint = tf_prefix_ + "wrist_3_joint";

    ur_manipulator_group_interface_.setJointValueTarget({{elbow_joint, 1.8571}});
    ur_manipulator_group_interface_.setJointValueTarget({{shoulder_lift_joint, -2.6729}});
    ur_manipulator_group_interface_.setJointValueTarget({{shoulder_pan_joint, 0}});
    ur_manipulator_group_interface_.setJointValueTarget({{wrist_1_joint, 0.729}});
    ur_manipulator_group_interface_.setJointValueTarget({{wrist_2_joint, 1.8398}});
    ur_manipulator_group_interface_.setJointValueTarget({{wrist_3_joint, 0}});

    ur_manipulator_group_interface_.move();
    // RCLCPP_INFO(logger_, "Going to ready position");
    // ur_manipulator_group_interface_.setNamedTarget("ready");
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // moveit::core::MoveItErrorCode ok = ur_manipulator_group_interface_.plan(plan);

    // if (ok)
    // {
    //     ur_manipulator_group_interface_.execute(plan);
    //     RCLCPP_INFO(logger_, "Planning succeeded");
    //     return true;
    // }

    // RCLCPP_ERROR(logger_, "Planning failed");
    // return false;
}

bool DevolRobot::set_manipulator_goal(const geometry_msgs::msg::Pose &pose)
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

void DevolRobot::set_gripper_position(GRIPPER_POSITION position)
{
    switch(position)
    {
        case GRIPPER_POSITION::OPEN:
            RCLCPP_INFO(logger_, "Setting gripper to OPEN");
            hand_group_interface_.setJointValueTarget({{gripper_joint_, 0.0}});
            break;
        case GRIPPER_POSITION::CLOSE:
            RCLCPP_INFO(logger_, "Setting gripper to CLOSE");
            hand_group_interface_.setJointValueTarget({{gripper_joint_, 0.8}});
            break;
        case GRIPPER_POSITION::GRAB:
            RCLCPP_INFO(logger_, "Setting gripper to GRAB");
            hand_group_interface_.setJointValueTarget({{gripper_joint_, 0.105}});
            break;
        default:
            RCLCPP_ERROR(logger_, "Invalid gripper position");
            return;
    }
    hand_group_interface_.move();
}

void DevolRobot::attach_object(const std::string &object_id)
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

void DevolRobot::detach_object(const std::string &object_id)
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

void DevolRobot::allow_collision_between(const std::string &object1, const std::string &object2)
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

void DevolRobot::attach_object_to_end_effector()
{
    pub_attach_.Publish(gz::msgs::Empty());
}

void DevolRobot::detach_object_from_end_effector()
{
    pub_detach_.Publish(gz::msgs::Empty());
}