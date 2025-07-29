#include "devol_drive_robot.h"
#include "devol_robot.h"

DevolDriveRobot::DevolDriveRobot(
    moveit::planning_interface::MoveGroupInterface &ur_manipulator_group_interface,
    moveit::planning_interface::MoveGroupInterface &hand_group_interface,
    moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
    const rclcpp::Logger &logger,
    const rclcpp::Node::SharedPtr &node,
    const std::string &tf_prefix) :
    devol_(ur_manipulator_group_interface,
           hand_group_interface,
           planning_scene_interface,
           logger,
           node,
           tf_prefix),
    logger_(logger),
    node_(node)
{
    std::string cmd_topic = "/model/" + tf_prefix + "devol_drive/cmd_vel";
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, qos);

    // Initialize the DevolDriveRobot
    RCLCPP_INFO(logger_, "DevolDriveRobot initialized. In Ready Position.");
}

DevolDriveRobot::~DevolDriveRobot(){}

// Diff Drive Methods
void DevolDriveRobot::publish_drive_command(double linear_x, double angular_z)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;

    cmd_vel_pub_->publish(msg);

    RCLCPP_INFO(logger_, "Published Twist: linear.x=%.2f, angular.z=%.2f", linear_x, angular_z);
}


// Manipulator Methods
void DevolDriveRobot::go_home()
{
    devol_.go_home();
}


bool DevolDriveRobot::set_manipulator_goal(const geometry_msgs::msg::Pose &pose)
{
    return devol_.set_manipulator_goal(pose);
}

void DevolDriveRobot::set_gripper_position(GRIPPER_POSITION position)
{
    devol_.set_gripper_position(position);
}

void DevolDriveRobot::attach_object(const std::string &object_id)
{
    devol_.attach_object(object_id);
}

void DevolDriveRobot::detach_object(const std::string &object_id)
{
    devol_.detach_object(object_id);
}

void DevolDriveRobot::allow_collision_between(const std::string &object1, const std::string &object2)
{
    devol_.allow_collision_between(object1, object2);
}

void DevolDriveRobot::attach_object_to_end_effector()
{
    devol_.attach_object_to_end_effector();
}

void DevolDriveRobot::detach_object_from_end_effector()
{
    devol_.detach_object_from_end_effector();
}