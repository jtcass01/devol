# devol

**devol** simulates a Clearpath A200 Husky base carrying a UR manipulator (the "Devol" mobile manipulator) in
**ROS 2 Lyrical** and **Gazebo Jetty**. It holds the code for a trade study comparing an
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
| `devol_sim` | Sim launch files, octomap map pipeline (point cloud publisher), goal markers, RViz config |
| `devol_local_planner` | Local motion planning: RRT\* / A\* planners, the A\* map padder and the PID path follower (`local_planner.launch.py`) |
| `devol_localization` | EKF and PF nodes, plus the study tooling: noise injection, ground truth, scoring, kidnapping, live views and the replay runner (see [its README](src/devol_localization/README.md)) |

The recommended way to run everything is the Docker image in [`docker/`](docker/), which bundles
Ubuntu 26.04, ROS 2 Lyrical, Gazebo Jetty and the built workspace. A native install is described in
[Native install](#native-install-without-docker) for those who prefer it.

## 1. Requirements

- **Windows 11** with **WSL 2** and **Docker Desktop** (WSL 2 backend). The GUI windows (Gazebo, RViz and
  the live filter views) open on the Windows desktop through WSLg.
- An **NVIDIA GPU** with a current Windows driver is strongly recommended. The container renders through
  Mesa's d3d12 driver on the GPU; without one Gazebo falls back to the CPU and runs several times slower.
  Even on the GPU with the GUIs off, the simulation runs at roughly one third of real time, so a 60 s
  drive takes about 3 minutes of wall-clock time.
- About 15 GB of disk for the image.

The compose file is written for Docker Desktop on Windows (it mounts WSLg and the WSL GPU driver from
Docker Desktop's VM). On a Linux host, use the [native install](#native-install-without-docker).

## 2. Install Docker Desktop

If Docker is not installed yet, install it from PowerShell (accept the administrator prompt), or download
it from [docker.com](https://www.docker.com/products/docker-desktop/):

```powershell
winget install --id Docker.DockerDesktop -e
```

Start Docker Desktop once and accept its terms. Keep the default **Use the WSL 2 based engine** setting.
If `docker` is not found afterwards, open a new terminal so it picks up the updated `PATH`; if Docker
reports that you lack permission, sign out of Windows and back in.

Check that the engine runs and can see the GPU:

```powershell
docker run --rm --gpus all ubuntu:26.04 ls -l /dev/dxg
```

## 3. Get the code and build the image

All commands from here on run from the repository root, in PowerShell or a WSL shell.

```bash
git clone https://github.com/jtcass01/devol.git
cd devol
docker compose -f docker/compose.yaml build
```

The first build downloads ROS 2 and Gazebo and takes 15 to 20 minutes; later builds reuse those layers
and only rebuild the workspace. The image is tagged `devol-sim:lyrical`. It contains `src/` as it was at
build time, so **rebuild the image after changing code** (or use the development shell in step 8).

## 4. Check the install

Confirm the container renders on the GPU. The renderer should read `D3D12 (NVIDIA ...)`, not `llvmpipe`:

```bash
docker compose -f docker/compose.yaml run --rm sim glxinfo -B
```

Then run the offline tests, which exercise the filter, planner and scoring math on synthetic data with no
Gazebo (about four minutes):

```bash
docker compose -f docker/compose.yaml run --rm sim bash -c "cd src/devol_localization && python3 -m pytest -q test && cd ../devol_local_planner && python3 -m pytest -q test/test_rrt_planner.py"
```

Expected: `38 passed` and `18 passed`.

## 5. Run the simulation

Every ROS command runs inside the container through `docker compose ... run --rm sim <command>`:

```bash
docker compose -f docker/compose.yaml run --rm sim ros2 launch devol_sim motion_planner_sim.launch.py rviz:=false
```

This starts Gazebo on the factory world, spawns the robot, builds the map (static point cloud →
`octomap_server` → `/devol_drive/projected_map`) and drives the robot through the three goals in
`src/devol_gazebo/worlds/factory/poses.csv` with the RRT\* planner. Stop it with Ctrl-C. Useful arguments:

| Argument | Default | Meaning |
|---|---|---|
| `maze` | `factory` | World: `factory`, `empty` or `moon_terrain` |
| `planner` | `rrt_star` | `rrt_star`, `rrt`, or `a_star` (the older planner on the inflated 2D map) |
| `gz_gui` | `true` | Show the Gazebo window. `false` roughly doubles the simulation speed. |
| `rviz` | `true` | Show RViz. The study uses its own Matplotlib views instead, so `false` is recommended. |

To run more than one process against the same simulation (for example the two filters by hand), open a
shell in a container and use it like a normal ROS terminal; `docker exec` adds further terminals to it:

```bash
docker compose -f docker/compose.yaml run --rm --name devol sim
docker exec -it devol bash
```

```bash
ros2 launch devol_localization ekf_localization.launch.py    # publishes /devol_drive/ekf_pose
ros2 launch devol_localization pf_localization.launch.py     # publishes /devol_drive/pf_pose and /devol_drive/pf_particles
```

The container uses `ROS_DOMAIN_ID=42` and `GZ_PARTITION=devol_docker`, so it does not talk to a
simulation running natively in WSL at the same time. Set `ROS_DOMAIN_ID` in the environment before
`docker compose` to change it.

## 6. Test cases

Both test cases have a preconfigured launch file. Each one starts the factory simulation headless,
drives the robot on Gazebo's ground-truth pose (so estimator error never changes the route), runs the
EKF and the PF at the study's nominal settings (2,000 particles, odometry noise k = 1, lidar noise
σ = 0.03 m, seed 0), and opens one live Matplotlib window per filter. Each window shows the map, the
ground-truth pose against the estimate, the lidar scan drawn from the estimate, the 2σ covariance
ellipse (EKF) or the particle cloud (PF), and position error over time against the filter's own 2σ
bound.

```bash
docker compose -f docker/compose.yaml run --rm sim ros2 launch devol_localization localization_test_cases.launch.py test_case:=1
docker compose -f docker/compose.yaml run --rm sim ros2 launch devol_localization localization_test_cases.launch.py test_case:=2
```

Test case 1 is the nominal route and test case 2 the kidnapped robot. Each `run --rm` starts a fresh
container, so no Gazebo from an earlier run can be left behind.

There is nothing to stop by hand. The run ends a few seconds after the robot reaches Goal 3, prints a
PASS/FAIL report, and leaves these files in `results/docker/test_case_<n>` in the repository (mounted
from `~/loc_results` in the container). A run that has not reached Goal 3 after 400 s of simulation time
(`max_duration`) ends and FAILs.

- `verdict.txt`: the PASS/FAIL report, one line per check.
- `summary.json`: for the EKF, the PF and an odometry-only (dead reckoning) baseline, the position and
  heading RMSE, the error at each waypoint, the compute time per update, and (test case 2) the time to
  recover to within 0.25 m of the true pose. `trajectory.csv` and `compute.csv` hold the per-update data.
- `ekf.mp4`, `pf.mp4` and `ekf.png`, `pf.png`: a video and a final screenshot of each live view.
- `posterior_t<sim time>.png`: the intermediate posterior (PF particles and EKF 2σ ellipse at the same
  instant), saved at 15 s and 45 s, and for test case 2 also 1, 5 and 15 s after the kidnap.

Useful arguments: `output_dir` (results folder; keep it under `/root/loc_results` so it lands on the
host), `viz:=false` (no windows), `record_video:=false`, `gz_gui:=true` (show Gazebo), `num_particles`,
`seed`, `max_duration`, and for test case 2 `kidnap_delay` (default 10 s).

### Test case 1: nominal route

The robot starts at (0.0, 0.0) and drives to the three goals. The expected outputs are the goal poses
from `src/devol_gazebo/worlds/factory/poses.csv`, derived independently of either filter:

| Goal | x (m) | y (m) | yaw (rad) |
|---|---|---|---|
| Goal 1: Pick Up | −4.70 | 9.20 | 3.1416 |
| Goal 2: Place | 5.45 | 2.03 | 0.0 |
| Goal 3: Drive | 1.87 | −8.06 | −1.5708 |

**PASS** when the EKF and the PF are both within 0.25 m of the ground-truth pose at every goal, and
dead reckoning from odometry alone is worse than both.

### Test case 2: kidnapped robot

10 s after the robot reaches Goal 1, it is teleported onto the Goal 2 pose (5.45 m, 2.03 m, 0.0 rad)
without the filters being told, then the planner drives on to Goal 3. The expected output is that
teleport pose.

**PASS** when, within 60 s of the kidnap, the PF returns to within 0.25 m of the true pose and stays
there for at least 3 s. The EKF's outcome is reported either way: it keeps a single Gaussian with no re-seeding, so it
is not expected to recover, and that contrast is part of the study.

## 7. Full study (optional)

The study records each route once in Gazebo, then replays the recording through every noise level,
particle count and random seed. See
[`src/devol_localization/README.md`](src/devol_localization/README.md) for the details. Run these in a
container shell (`docker compose -f docker/compose.yaml run --rm sim`) and keep bags and results under
`~/loc_results`, the folder that is saved to `results/docker/` on the host:

```bash
# Record the two routes (filters off). Stop each with Ctrl-C once Goal 3 is reached.
ros2 launch devol_localization localization_sim.launch.py filters:=false record_bag:=~/loc_results/bags/nominal
ros2 launch devol_localization localization_sim.launch.py filters:=false scenario:=kidnap record_bag:=~/loc_results/bags/kidnap

# Replay every configuration with 20 seeds each, then build the tables (mean ± 95% CI)
ros2 run devol_localization localization_study run --bag ~/loc_results/bags/nominal \
  --kidnap-bag ~/loc_results/bags/kidnap --out ~/loc_results/study
ros2 run devol_localization localization_study analyze --out ~/loc_results/study
```

## 8. Changing the code

The `dev` service mounts the repository's `src/` into the container, so edits on the host are picked up
after a rebuild inside it. Build outputs are kept in Docker volumes between sessions.

```bash
docker compose -f docker/compose.yaml run --rm dev
colcon build --symlink-install
source install/setup.bash
```

Rebuild the image (step 3) when you want the `sim` service to include your changes.

## Native install (without Docker)

The same stack can be installed directly on **Ubuntu 26.04**, native or as a WSL 2 distro.

1. Add the ROS 2 apt source by following the
   [official Lyrical install guide](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html),
   then install ROS 2, Gazebo (shipped through the `ros_gz` vendor packages) and the workspace's
   dependencies:

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

2. Build the workspace:

   ```bash
   git clone https://github.com/jtcass01/devol.git ~/devol
   cd ~/devol
   source /opt/ros/lyrical/setup.bash
   colcon build --symlink-install
   ```

3. Set up every terminal that runs the simulation (adding these to `~/.bashrc` saves retyping them):

   ```bash
   source /opt/ros/lyrical/setup.bash
   source ~/devol/install/setup.bash

   # WSL only: render on the NVIDIA GPU instead of the CPU, and let Qt windows open under WSLg
   export GALLIUM_DRIVER=d3d12
   export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
   export QT_QPA_PLATFORM=xcb
   ```

The commands in steps 4 to 7 then run directly, without the `docker compose ... run --rm sim` prefix.
Test case results go to `~/loc_results/test_case_<n>`. Start each test case only after the previous
Gazebo has fully exited (see Troubleshooting).

## Troubleshooting

- **Gazebo crashes right after the robot spawns, with `ODE INTERNAL ERROR` on `collision_trimesh_trimesh`:**
  an intermittent physics-engine assertion on the gripper meshes, seen once in the container. Run the
  same command again.
- **Gazebo is very slow:** check that `glxinfo -B` (step 4) reports the D3D12 NVIDIA renderer. Natively,
  check that `GALLIUM_DRIVER=d3d12` is set. Run with `gz_gui:=false rviz:=false`.
- **No windows appear from the container:** WSLg must be running. Open any WSL shell once, and check
  that `wsl --version` lists a WSLg version.
- **`docker: error getting credentials` or `docker` not found:** open a new terminal after installing
  Docker Desktop so its folder is on `PATH`.
- **The robot never moves and the planner logs "TF lookup failed":** the `odom → a200_base_link`
  transform is missing; make sure the image or build is from the current `main`.
- **(Native) A test case aborts saying a Gazebo server or `parameter_bridge` is still running:** an
  earlier run hasn't finished shutting down. Stop it, check that nothing is left, then relaunch:
  ```bash
  pkill -f gz-sim; pkill -f "gz sim"; pkill -f parameter_bridge
  pgrep -af "gz-sim|gz sim|parameter_bridge"   # should print nothing
  ```
- **(Native) `package 'octomap_server' not found` or `No module named 'scipy'`:** install
  `ros-lyrical-octomap-server` or `python3-scipy` (native step 1).
- **Bridges print a segmentation fault after Ctrl-C or at the end of a test case:** harmless; it happens
  only during shutdown.
