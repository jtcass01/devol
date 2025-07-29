. install/setup.sh
ros2 control load_controller --set-state active apollo_ur_manipulator_controller
ros2 control load_controller --set-state active apollo_hand_controller
ros2 control load_controller --set-state active artemis_ur_manipulator_controller
ros2 control load_controller --set-state active artemis_hand_controller
ros2 control load_controller --set-state active joint_state_broadcaster
