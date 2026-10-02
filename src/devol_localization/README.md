# devol_localization

EKF and particle-filter localization for the devol mobile manipulator, plus the tooling for the
PF-vs-EKF trade study in `docs/Reasoning Under Uncertainty/` (noise injection, ground truth,
scoring, kidnapping, live views and the replay runner).

## Nodes

| Executable | What it does |
|---|---|
| `ekf_localization` | EKF: odometry prediction, scan-matched pose correction. `init_mode` tf / pose / global. Publishes `ekf_pose`, `ekf_compute_time_ms`. |
| `pf_localization` | SIR Monte Carlo localization with augmented-MCL injection. Publishes `pf_pose`, `pf_particles`, `pf_compute_time_ms`. |
| `noise_injector` | Seeded study noise: odometry increments perturbed with alpha1..4 = 0.05 k (per 0.1 m / 0.1 rad segment, so independent of the odometry rate), lidar ranges + N(0, sigma_r^2). Publishes `/devol_drive/noisy/{odom,scan}`. |
| `localization_evaluator` | Scores a trial against ground truth: position / heading RMSE, waypoint error, compute time, recovery time to < 0.25 m (global and kidnap). Dead reckoning from the noisy odometry is scored as a third estimator. Writes `trajectory.csv`, `compute.csv`, `summary.json`; for a test case also judges PASS/FAIL (`verdict.txt`) and can end the run at the last goal. |
| `localization_viz` | Live Matplotlib view (no RViz): map, Gazebo pose vs estimate, lidar projected from the estimate, 2-sigma ellipse, every particle (`mode:=pf`), and error vs time against the filter's own 2-sigma bound. Optional MP4. |
| `ground_truth_tf` | Publishes `map -> odom` from ground truth so the planner and controller drive on the true pose, as the protocol requires. |
| `kidnapper` | Teleports the robot in Gazebo (`/world/maze_world/set_pose`) without telling the estimators, at a set sim time or a set delay after reaching a waypoint. |
| `localization_study` | `run`: replays the bags through every configuration and seed of the protocol. `analyze`: mean +/- 95% CI tables, Clopper-Pearson recovery rates and the sensitivity figure. |

Ground truth comes from a Gazebo `OdometryPublisher` added to the A200 (`a200.gazebo.xacro`), bridged
as `/devol_drive/ground_truth/odom` (world frame, which equals `map`).

## Running the tests in Gazebo

Prerequisite: the sim fixes from the WSL machine (odom -> `a200_base_link` on `/tf` via the bridged
`/model/devol_drive/tf`, the `a200_base_link` alias, the wheel-slip calibration). `ground_truth_tf`
only replaces `map -> odom`; the planner still needs `odom -> base` on `/tf`.

Verification test cases, preconfigured, with the EKF and PF views updating live. Each run stops by
itself a few seconds after the robot reaches Goal 3 (or FAILs after 400 s of sim time without
getting there), prints PASS/FAIL, and leaves `verdict.txt`,
`summary.json`, `ekf.mp4`/`pf.mp4` and final `ekf.png`/`pf.png` screenshots in `~/loc_results/test_case_<n>`:

```bash
ros2 launch devol_localization localization_test_cases.launch.py test_case:=1   # nominal route: EKF and PF within 0.25 m at every goal, dead reckoning worse
ros2 launch devol_localization localization_test_cases.launch.py test_case:=2   # kidnap onto Goal 2 after Goal 1: PF back within 0.25 m before 60 s
```

Start each test case only after the previous Gazebo has fully exited: a sim or bridge still shutting
down publishes its own clock and ground truth, which would be scored as the new run. If a Gazebo server
(`gz-sim-main` / `gz sim`) or `parameter_bridge` is still running, the test-case launch aborts with an
error (it does not wait). Stop them, check nothing is left, then relaunch:

```bash
pkill -f gz-sim; pkill -f "gz sim"; pkill -f parameter_bridge
pgrep -af "gz-sim|gz sim|parameter_bridge"   # should print nothing
```

The scorer also ignores ground truth that starts after 10 s of sim time.

Videos use ffmpeg when installed and OpenCV (`python3-opencv`) otherwise; without either the views
still run and only the PNG screenshots are saved.

Live run with both views (what the filters see, as they run):

```bash
ros2 launch devol_localization localization_sim.launch.py                       # nominal route
ros2 launch devol_localization localization_sim.launch.py scenario:=global      # PF uniform, EKF map-wide Gaussian at a seeded random pose
ros2 launch devol_localization localization_sim.launch.py scenario:=kidnap       # onto Goal 2, 10 s after Goal 1
# extra: output_dir:=~/loc_results/live  video_dir:=~/loc_results/live  viz_headless:=true  num_particles:=500
```

The protocol records each route once and replays it through every configuration:

```bash
# 1. Record (filters off; the controller drives on ground truth)
ros2 launch devol_localization localization_sim.launch.py filters:=false record_bag:=~/loc_bags/nominal
ros2 launch devol_localization localization_sim.launch.py filters:=false scenario:=kidnap record_bag:=~/loc_bags/kidnap
# stop each with Ctrl-C once the robot has reached Goal 3, then wait for the recorder to close the bag
# (localization_replay reindexes a bag left without metadata.yaml)

# 2. One replayed trial, with the views
ros2 launch devol_localization localization_replay.launch.py bag:=~/loc_bags/nominal viz:=true \
    k:=1 sigma_r:=0.03 num_particles:=2000 seed:=0 output_dir:=~/loc_results/check

# 3. The whole study (9 sweep configurations + global + kidnap, 20 seeds each), then the tables
ros2 run devol_localization localization_study run --bag ~/loc_bags/nominal --kidnap-bag ~/loc_bags/kidnap \
    --out ~/loc_results/study
ros2 run devol_localization localization_study analyze --out ~/loc_results/study
```

## Offline tests

```bash
cd src/devol_localization && python3 -m pytest test   # no ROS needed; from the repo root it needs install/setup.bash sourced
```
