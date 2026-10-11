# Rough-draft test instructions (TA)

These steps run the Gazebo factory world with all three localization filters running side by side:

- **EKF**: extended Kalman filter.
- **PF**: particle filter (2,000 particles).
- **Hybrid**: an EKF that the PF re-seeds after it detects a kidnap.

They cover the two required test cases, each repeatable over several random seeds:

| Test case | What happens | Passes when |
|---|---|---|
| **1. Nominal route** | The robot drives from (0, 0) through Goal 1 → Goal 2 → Goal 3. | The EKF, PF and hybrid are each within 0.25 m of the true pose at every goal, and odometry alone (dead reckoning) is worse than all three. |
| **2. Kidnapped robot** | 10 s after reaching Goal 1, the robot is teleported to a random collision-free pose without the filters being told, then drives on to Goal 2 and Goal 3. | The PF and the hybrid each get back within 0.25 m of the true pose and stay there for 3 s, within 60 s of the kidnap. The EKF is reported but not expected to recover. |

The robot always drives on Gazebo's ground-truth pose, so the route is the same whatever the filters
estimate. The filters only estimate the pose from wheel odometry and the 2D lidar, with study noise added.

Everything below assumes a fresh **Ubuntu 26.04** desktop machine. An NVIDIA (or other) GPU with working
drivers is strongly recommended; without one Gazebo renders on the CPU and runs several times slower.

---

## 1. One-time setup

### 1.1 Install ROS 2 Lyrical and Gazebo Jetty

Add the ROS 2 apt repository by following the
[official ROS 2 Lyrical install guide for Ubuntu](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html)
(the "Setup Sources" section). Then install ROS 2, Gazebo (it comes with the `ros_gz` packages) and the
workspace's dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-lyrical-desktop \
  ros-lyrical-ros-gz ros-lyrical-gz-ros2-control ros-lyrical-ros2-controllers ros-lyrical-xacro \
  ros-lyrical-joint-state-publisher-gui \
  ros-lyrical-ur ros-lyrical-robotiq-description ros-lyrical-realsense2-description \
  ros-lyrical-octomap-server \
  ros-dev-tools python3-colcon-common-extensions python3-pytest \
  python3-numpy python3-scipy python3-matplotlib python3-opencv python3-pyqt5 ffmpeg
```

### 1.2 Get and build the code

```bash
git clone https://github.com/jtcass01/devol.git ~/devol
cd ~/devol
source /opt/ros/lyrical/setup.bash

# Pick up any dependency the list above missed
sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro lyrical
rosdep install --from-paths src --ignore-src -r -y --rosdistro lyrical

colcon build --symlink-install
```

### 1.3 Set up each terminal

Run this in every new terminal that launches the simulation (or append it to `~/.bashrc`):

```bash
source /opt/ros/lyrical/setup.bash
source ~/devol/install/setup.bash
```

### 1.4 Optional: check the build

The offline unit tests check the filter, planner and scoring math on synthetic data, without Gazebo
(about four minutes):

```bash
cd ~/devol/src/devol_localization && python3 -m pytest -q test
cd ~/devol/src/devol_local_planner && python3 -m pytest -q test/test_rrt_planner.py
```

---

## 2. Before every run: make sure no old simulation is running

A Gazebo server left over from an earlier run would feed its clock and poses into the new one. Before
each launch, run:

```bash
pkill -f gz-sim; pkill -f "gz sim"; pkill -f parameter_bridge
pgrep -af "gz-sim|gz sim|parameter_bridge"    # must print nothing
```

If the second command still prints a process, wait a few seconds and repeat.

---

## 3. Watch the filters live

### Test case 2: kidnapped robot (headline run)

```bash
ros2 launch devol_localization localization_sim.launch.py gz_gui:=true scenario:=kidnap kidnap_target_mode:=random
```

**What you should see:**

- The **Gazebo** window with the factory world and the robot (a Husky base carrying a UR arm).
- Three **Matplotlib** windows, titled `EKF vs Gazebo ground truth ...`, `PF vs ...` and `HYBRID vs ...`.
  Each window shows the map, the true pose against the filter's estimate, the lidar scan drawn from the
  estimate, the EKF's 2σ covariance ellipse or the PF's particle cloud, and position error over time
  against the filter's own 2σ bound.
- The robot drives to Goal 1 at (−4.70, 9.20). Ten seconds later the terminal prints a line like
  `Teleported ... to (x, y, yaw) at t=... s`, and the robot jumps in Gazebo.
- After the jump, the PF and the hybrid should find the new pose within a few seconds (their error
  plots drop back under 0.25 m). The EKF is expected to stay lost, with its error stuck at several
  metres.
- The robot then drives on to Goal 2 (5.45, 2.03) and Goal 3 (1.87, −8.06).

This launch keeps running after Goal 3; stop it with **Ctrl-C** once the robot has stopped there. It
saves no files (see section 4 for scored runs).

### Test case 1: nominal route (no kidnap)

Drop the kidnap arguments:

```bash
ros2 launch devol_localization localization_sim.launch.py gz_gui:=true
```

You see the same four windows. The robot drives the three goals without being moved, and all three
filters should stay on the true pose. Stop it with **Ctrl-C** after Goal 3.

### Changing the seed

Add `seed:=<n>` to either command (the default is `seed:=0`). The seed sets the noise added to the
odometry and lidar, the PF's random sampling and, in test case 2, where the robot is teleported. For
example:

```bash
ros2 launch devol_localization localization_sim.launch.py gz_gui:=true scenario:=kidnap kidnap_target_mode:=random seed:=3
```

Other arguments that are useful here:

| Argument | Default | Meaning |
|---|---|---|
| `gz_gui` | `false` | Show the Gazebo window. Leaving it off makes the simulation run faster. |
| `estimators` | `ekf,pf,hybrid` | Which filters run. `ekf,pf` leaves the hybrid out. |
| `num_particles` | `2000` | PF particle count. |
| `kidnap_target_mode` | `fixed` | `random` (a seeded random pose) or `fixed` (teleports onto Goal 2). |
| `kidnap_time` | `10.0` | Seconds after Goal 1 before the teleport. |
| `max_linear_velocity` | `1.5` | The robot's top speed in m/s. |

---

## 4. Scored runs over several seeds (PASS/FAIL and saved results)

`localization_test_cases.launch.py` runs the same simulation with the same three filters and adds
scoring. Test case 2 uses a random kidnap target by default, as above. Each run ends by itself a few
seconds after the robot reaches Goal 3, prints a PASS/FAIL report and saves its results. A run that has
not reached Goal 3 after 400 s of simulated time stops and FAILs.

One run of each test case:

```bash
ros2 launch devol_localization localization_test_cases.launch.py test_case:=1 gz_gui:=true seed:=0 output_dir:=~/loc_results/tc1_seed0
ros2 launch devol_localization localization_test_cases.launch.py test_case:=2 gz_gui:=true seed:=0 output_dir:=~/loc_results/tc2_seed0
```

Both test cases for seeds 0, 1 and 2 in one go (leave out `gz_gui:=true` for faster runs):

```bash
for seed in 0 1 2; do
  for tc in 1 2; do
    pkill -f gz-sim; pkill -f "gz sim"; pkill -f parameter_bridge; sleep 5
    ros2 launch devol_localization localization_test_cases.launch.py \
      test_case:=$tc seed:=$seed gz_gui:=true output_dir:=~/loc_results/tc${tc}_seed${seed}
  done
done
```

This launch checks for a leftover Gazebo itself and refuses to start if it finds one; the `pkill` and
`sleep` lines clear any that are still shutting down. Give each run its own `output_dir`, or the next run
overwrites it.

**Expected terminal report** (test case 1, then test case 2):

```
===== Test case 1: nominal three-waypoint route: PASS =====
PASS ekf: every waypoint within 0.25 m (Goal 1: Pick Up: 0.019 m, Goal 2: Place: 0.041 m, Goal 3: Drive: 0.034 m); position RMSE 0.059 m
PASS pf: every waypoint within 0.25 m (...)
PASS hybrid: every waypoint within 0.25 m (...)
PASS dead reckoning worse than both filters: mean waypoint error 7.058 m vs worst filter 0.097 m
```

```
===== Test case 2: kidnapping: PASS =====
kidnap seen at t = 38.24 s
PASS pf: back within 0.25 m 1.4 s after the kidnap
INFO ekf: did not recover (final error 12.62 m)
PASS hybrid: back within 0.25 m 1.8 s after the kidnap
```

The numbers vary with the seed.

**Where the results land:** in the `output_dir` you gave, for example `~/loc_results/tc2_seed0/`:

| File | Contents |
|---|---|
| `verdict.txt` | The PASS/FAIL report above. |
| `summary.json` | For each filter and the dead-reckoning baseline: position and heading RMSE, the error at each goal, compute time per update and, in test case 2, the time to recover after the kidnap. |
| `trajectory.csv`, `compute.csv` | Per-update estimates against ground truth, and per-update compute times. |
| `ekf.mp4`, `pf.mp4`, `hybrid.mp4` and matching `.png` | A video and a final screenshot of each live window. |
| `posterior_t<time>.png` | The PF's particles and the EKF's 2σ ellipse at the same moment, at 15 s and 45 s, and in test case 2 also 1, 5 and 15 s after the kidnap. |

---

## 5. If something goes wrong

- **The launch refuses to start, saying a Gazebo sim or bridge is still running:** run the commands in
  section 2 and try again.
- **Gazebo crashes right after the robot appears, with `ODE INTERNAL ERROR`:** an occasional physics
  engine error. Run the same command again.
- **Everything is very slow:** run without `gz_gui:=true`. The simulation runs slower than real time
  even on a GPU, so a run takes a few minutes of wall-clock time.
- **`package 'octomap_server' not found` or `No module named 'scipy'`:** a package from step 1.1 is
  missing; rerun that `apt install`.
- **Segmentation fault messages from the bridges after Ctrl-C or at the end of a run:** harmless; they
  happen only during shutdown.
