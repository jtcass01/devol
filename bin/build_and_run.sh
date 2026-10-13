#!/bin/bash

. /opt/ros/lyrical/setup.bash
. bin/clean.sh
. bin/set_env.sh
colcon build
. install/setup.bash
NUM_PARTICLES="${1:-10}"
ros2 launch devol_localization localization_sim.launch.py pf_num_particles:="$NUM_PARTICLES"