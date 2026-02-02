ros2 control load_controller -s --set-state active ur_manipulator_controller --ros-args --log-level DEBUG -p stamped:=true
ros2 control load_controller -s --set-state active hand_controller --ros-args --log-level DEBUG -p stamped:=true
ros2 control load_controller -s --set-state active joint_state_broadcaster --ros-args --log-level DEBUG -p stamped:=true