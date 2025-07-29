#pragma once
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <gz/msgs.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "devol_robot.h"

class DevolDriveRobot {
    public:
        DevolDriveRobot(moveit::planning_interface::MoveGroupInterface &ur_manipulator_group_interface,
                        moveit::planning_interface::MoveGroupInterface &hand_group_interface,
                        moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                        const rclcpp::Logger &logger,
                        const rclcpp::Node::SharedPtr &node,
                        const std::string &tf_prefix);
        ~DevolDriveRobot();

        // Diff Drive Methods
        void publish_drive_command(double linear_x, double angular_z);

        // Manipulator Methods
        void go_home();
        bool set_manipulator_goal(const geometry_msgs::msg::Pose &pose);
        void set_gripper_position(GRIPPER_POSITION position);
        void attach_object(const std::string &object_id);
        void detach_object(const std::string &object_id);
        void allow_collision_between(const std::string &object1, const std::string &object2);
        void attach_object_to_end_effector();
        void detach_object_from_end_effector();

    private:
        DevolRobot devol_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        const rclcpp::Logger &logger_;
        const rclcpp::Node::SharedPtr &node_;
};
