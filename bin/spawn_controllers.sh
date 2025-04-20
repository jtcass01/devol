. install/setup.sh
ros2 control load_controller --set-state active ur_manipulator_controller
ros2 control load_controller --set-state active hand_controller
ros2 control load_controller --set-state active joint_state_broadcaster
