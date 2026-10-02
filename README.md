# devol

**devol** simulates a Clearpath A200 Husky base carrying a UR manipulator (the "Devol" mobile manipulator) in
**ROS 2 Lyrical** and **Gazebo Jetty**. On this branch it is the code for a trade study comparing an
**extended Kalman filter (EKF)** and a **particle filter (PF)** for map-based localization of the base
(JHU 605.745, Reasoning Under Uncertainty; the paper sources are in `docs/Reasoning Under Uncertainty/`).

The simulation drives the robot through the waypoints of a factory world with an RRT\* planner. Both
filters estimate the robot's pose from wheel odometry and a 2D lidar against a map built from the
world's point cloud, and are scored against Gazebo's ground-truth pose.

| Package | What it holds |
|---|---|
| `a200_description` | Clearpath A200 base, lidars and vendored meshes |
| `devol_description` | UR arm, gripper and cameras mounted on the base |
| `devol_drive_description` | The full robot (base + arm), its Gazebo plugins and the ROS–Gazebo bridge |
| `devol_gazebo` | Worlds (`factory`, `empty`, `moon_terrain`), their waypoints (`poses.csv`) and static point clouds |
| `devol_sim` | Sim launch files, octomap map pipeline, RRT\* / A\* planners, PID path follower |
| `devol_localization` | EKF and PF nodes, plus the study tooling: noise injection, ground truth, scoring, kidnapping, live views and the replay runner (see [its README](src/devol_localization/README.md)) |
| `devol_moveit_config`, `devol_demo_py`, `devol` | Earlier MoveIt pick-and-place work. Not used by the study and skipped in the build below. |

## 1. Requirements

- **Ubuntu 26.04**, either native or as a WSL 2 distro on Windows 11. These instructions were written
  for WSL 2 (Ubuntu 26.04) with an NVIDIA GPU; native Ubuntu 26.04 needs the same packages and can
  skip the WSL display settings in step 4.
- About 10 GB of disk for ROS 2 and Gazebo.
- A GPU is strongly recommended. Even rendering on the GPU and with the GUIs off, the simulation runs
  at roughly one third of real time, so a 60 s drive takes about 3 minutes of wall-clock time.

## 2. Install ROS 2 Lyrical and Gazebo Jetty

ROS 2 Lyrical ships Gazebo Jetty through its `ros_gz` vendor packages, so there is no separate Gazebo
install. If the ROS 2 apt source is not configured yet, add it first (this follows the
[official Lyrical install guide](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html)):

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Then install ROS 2, Gazebo and everything the workspace uses:

```bash
sudo apt update
sudo apt install -y \
  ros-lyrical-desktop \
  ros-lyrical-ros-gz ros-lyrical-gz-ros2-control ros-lyrical-ros2-controllers ros-lyrical-xacro \
  ros-lyrical-ur ros-lyrical-robotiq-description ros-lyrical-realsense2-description \
  ros-lyrical-octomap-server \
  python3-colcon-common-extensions python3-pytest \
  python3-numpy python3-scipy python3-matplotlib python3-opencv ffmpeg
```

`ffmpeg` is only needed to save the live views as MP4 files.

## 3. Get the code and build

```bash
git clone -b Agent-Localization https://github.com/jtcass01/devol.git ~/devol
cd ~/devol
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-skip devol devol_moveit_config devol_demo_py
```

The three skipped packages are the old MoveIt work, which needs packages Lyrical does not ship
(`warehouse_ros_mongo`, and `devol_msgs`, which is not in this repository). Nothing in the simulation
or the study uses them.

## 4. Set up each terminal

Every terminal that runs the simulation needs:

```bash
source /opt/ros/lyrical/setup.bash
source ~/devol/install/setup.bash

# WSL only: render on the NVIDIA GPU instead of the CPU, and let Qt windows open under WSLg
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export QT_QPA_PLATFORM=xcb
```

Adding these lines to `~/.bashrc` saves retyping them. Without the `GALLIUM_DRIVER` lines Gazebo renders
in software and runs several times slower; without `QT_QPA_PLATFORM=xcb`, RViz exits at startup with
"Invalid parentWindowHandle".

## 5. Check the install with the offline tests

These run the filter, planner and scoring math on synthetic data, with no Gazebo. They take about four
minutes in total.

```bash
cd ~/devol/src/devol_localization && python3 -m pytest -q test
cd ~/devol/src/devol_sim && python3 -m pytest -q test/test_rrt_planner.py
```

Expected: `28 passed` and `18 passed`.

## 6. Run the simulation

```bash
ros2 launch devol_sim motion_planner_sim.launch.py rviz:=false
```

This starts Gazebo on the factory world, spawns the robot, builds the map (static point cloud →
`octomap_server` → `/devol_drive/projected_map`) and drives the robot through the three goals in
`src/devol_gazebo/worlds/factory/poses.csv` with the RRT\* planner. Useful arguments:

| Argument | Default | Meaning |
|---|---|---|
| `maze` | `factory` | World: `factory`, `empty` or `moon_terrain` |
| `planner` | `rrt_star` | `rrt_star`, `rrt`, or `a_star` (the older planner on the inflated 2D map) |
| `gz_gui` | `true` | Show the Gazebo window. `false` roughly doubles the simulation speed. |
| `rviz` | `true` | Show RViz. The study uses its own Matplotlib views instead, so `false` is recommended. |

To run the two filters next to the simulation by hand:

```bash
ros2 launch devol_localization ekf_localization.launch.py    # publishes /devol_drive/ekf_pose
ros2 launch devol_localization pf_localization.launch.py     # publishes /devol_drive/pf_pose and /devol_drive/pf_particles
```

## 7. Test cases

Both test cases run through `localization_sim.launch.py`, which starts the simulation above with the
Gazebo GUI and RViz off, drives the robot on Gazebo's ground-truth pose (so estimator error never
changes the route), runs both filters, and opens one live Matplotlib view per filter. Each view shows
the map, the ground-truth pose against the estimate, the lidar scan drawn from the estimate, the 2σ
covariance ellipse (EKF) or the particle cloud (PF), and position error over time against the filter's
own 2σ bound.

Each run writes its results to `output_dir`:

- `summary.json`: the run's configuration and, for the EKF, the PF and an odometry-only baseline, the
  position and heading RMSE, the error at each waypoint, the compute time per update, and (kidnap only)
  the time to recover to within 0.25 m of the true pose.
- `trajectory.csv` and `compute.csv`: the per-update estimates and timings behind those numbers.
- `ekf.mp4` and `pf.mp4` when `video_dir` is set.

Stop each run with Ctrl-C once the robot has reached Goal 3, when the console prints
`Successfully reached goal: Goal 3: Drive`.

> **Placeholder.** A dedicated test-case launch file with automatic pass/fail checks is still being
> written. Until it lands, run the commands below and compare `summary.json` with the expected outputs
> by hand. The launch file name and the exact pass thresholds will be filled in here.
> `TODO(test-case launch): <launch file and arguments>`

### Test case 1: nominal route

```bash
ros2 launch devol_localization localization_sim.launch.py scenario:=nominal \
  output_dir:=~/loc_results/tc1_nominal video_dir:=~/loc_results/tc1_nominal
```

The robot starts at (0.0, 0.0) and drives to the three goals. The expected outputs are the goal poses
from `poses.csv`, derived independently of either filter:

| Goal | x (m) | y (m) | yaw (rad) |
|---|---|---|---|
| Goal 1: Pick Up | −4.70 | 9.20 | 3.1416 |
| Goal 2: Place | 5.45 | 2.03 | 0.0 |
| Goal 3: Drive | 1.87 | −8.06 | −1.5708 |

Each filter's estimate at each goal should lie within the waypoint radius (0.5 m) of these poses, while
odometry alone drifts by metres over the route. `TODO(test-case launch): final pass threshold.`

### Test case 2: kidnapped robot

```bash
ros2 launch devol_localization localization_sim.launch.py scenario:=kidnap \
  kidnap_time:=30 kidnap_target:=5.45,2.03,0.0 \
  output_dir:=~/loc_results/tc2_kidnap video_dir:=~/loc_results/tc2_kidnap
```

After 30 s of simulation time the robot is teleported to (5.45 m, 2.03 m, 0.0 rad), the Goal 2 pose,
without telling the filters. The expected output is that teleport pose: after the kidnap, a filter
passes if its estimate returns to within 0.25 m of the true pose, and `summary.json` reports how long
that took. Offline, the PF recovers (its random-particle injection re-seeds the true pose) and the EKF,
which keeps a single Gaussian, is not expected to. `TODO(test-case launch): final pass threshold and
time limit.`

## 8. Full study (optional)

The study records each route once in Gazebo, then replays the recording through every noise level,
particle count and random seed. See
[`src/devol_localization/README.md`](src/devol_localization/README.md) for the details.

```bash
# Record the two routes (filters off). Stop each with Ctrl-C once Goal 3 is reached.
ros2 launch devol_localization localization_sim.launch.py filters:=false record_bag:=~/loc_bags/nominal
ros2 launch devol_localization localization_sim.launch.py filters:=false scenario:=kidnap record_bag:=~/loc_bags/kidnap

# Replay every configuration with 20 seeds each, then build the tables (mean ± 95% CI)
ros2 run devol_localization localization_study run --bag ~/loc_bags/nominal --kidnap-bag ~/loc_bags/kidnap \
  --out ~/loc_results/study
ros2 run devol_localization localization_study analyze --out ~/loc_results/study
```

## Troubleshooting

- **Gazebo is very slow:** check that `GALLIUM_DRIVER=d3d12` is set (step 4), and run with `gz_gui:=false rviz:=false`.
- **`package 'octomap_server' not found`:** install `ros-lyrical-octomap-server` (step 2).
- **`No module named 'scipy'`:** install `python3-scipy` (step 2).
- **The robot never moves and the planner logs "TF lookup failed":** the `odom → a200_base_link`
  transform is missing; make sure the build is from the current `Agent-Localization` branch and that
  `install/setup.bash` is sourced.
- **Bridges print a segmentation fault after Ctrl-C:** harmless; it happens only during shutdown.
