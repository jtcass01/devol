#pragma once
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <gz/msgs.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>


enum GRIPPER_POSITION
{
    OPEN,
    CLOSE,
    GRAB
};

class Robot {
    public:
        Robot(moveit::planning_interface::MoveGroupInterface &ur_manipulator_group_interface,
              moveit::planning_interface::MoveGroupInterface &hand_group_interface,
              moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
              const rclcpp::Logger &logger,
              const rclcpp::Node::SharedPtr &node);
        ~Robot();

        bool go_home();
        bool set_manipulator_goal(const geometry_msgs::msg::Pose &pose);
        void set_gripper_position(GRIPPER_POSITION position);
        void attach_object(const std::string &object_id);
        void detach_object(const std::string &object_id);
        void allow_collision_between(const std::string &object1, const std::string &object2);
        void attach_object_to_end_effector();
        void detach_object_from_end_effector();

    private:
        moveit::planning_interface::MoveGroupInterface &ur_manipulator_group_interface_;
        moveit::planning_interface::MoveGroupInterface &hand_group_interface_;
        moveit::planning_interface::PlanningSceneInterface &planning_scene_interface_;
        gz::transport::Node gz_node_;
        gz::transport::Node::Publisher pub_attach_ = gz_node_.Advertise<gz::msgs::Empty>("/devol_attach/attach");
        gz::transport::Node::Publisher pub_detach_ = gz_node_.Advertise<gz::msgs::Empty>("/devol_attach/detach");
        

        const rclcpp::Logger &logger_;
        const rclcpp::Node::SharedPtr &node_;
};
