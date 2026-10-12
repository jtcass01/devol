. bin/clean.sh
. bin/set_env.sh
colcon build
. install/setup.bash
ros2 launch devol_localization localization_sim.launch.py
