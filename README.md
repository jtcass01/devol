# devol

## Disclaimer
The package specific to the course project can be found at src/devol_localization.  None of it was written with the aid of gen AI. Other files in different packages like devol_localization_claude and devol_local_planner are from previous classwork, personal investigation, or were written with the aid of Anthropic Claude.

- The code for particle filter node can be found at src/devol_localization/devol_localization/pf_localization_node.py.
- The code for extended kalman filter node can be found at src/devol_localization/devol_localization/ekf_localization_node.py.
- The code for dead reckonining node can be found at src/devol_localization/devol_localization/dead_recknoning_node.py.

The results directory includes two test cases: PF with 10 particles, PF with 1000 particles, and PF with 300 particles, both ran with the EKF and DR localization algorithms as well. The accompaning .csv files contain raw time stamped telemetry values.

## WSL2 or Ubuntu 26.04 Installation/Runing Instructions.
1. Follow the installation instructions for ROS2 Lyrical on the oficial website: https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html

2. Install Dependencies. Some things may be missing here. I don't have a clean Ubuntu install to test it.  Reach out if you have trouble.
   ```bash
   sudo apt update
   sudo apt install -y \
     ros-lyrical-desktop \
     ros-lyrical-ros-gz \
     ros-lyrical-gz-ros2-control \
     ros-lyrical-ros2-controllers \
     ros-lyrical-xacro \
     ros-lyrical-joint-state-publisher-gui \
     ros-lyrical-ur \
     ros-lyrical-robotiq-description \
     ros-lyrical-realsense2-description \
     ros-lyrical-octomap-server \
     python3-colcon-common-extensions \
     python3-pytest \
     python3-numpy \
     python3-scipy \
     python3-matplotlib
   ```

3. Run the full simulation including metric logging and visualization of the Particle Filter, EKF, and Dead Reckoning Algorithms.
```
. bin/build_and_run.sh
```

4. Run the unittests in the repo.
```
colcon test
```

## Docker Instructions (Optional)

1. There is a docker container in this repo that was developed by Claude.  I did not use it for development but it's available for you to run the code.  If you need help setting it up, I am happy to join a meeting.
