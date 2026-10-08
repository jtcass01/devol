# Localization results: EKF vs particle filter vs hybrid

Two Gazebo test cases from `localization_test_cases.launch.py`, one run each. Every run drives all three
estimators side by side on the same simulated robot, sensors and noise, so each pair of rows below comes from
the same drive.

- **tc1_nominal**: the three-waypoint factory route (Pick Up, Place, Drive) with no kidnap.
- **tc2_kidnap**: the same route, but 10 s after Goal 1 the robot is teleported onto Goal 2 (5.45, 2.03) without
  the estimators being told (here at t = 38.66 s sim).

Setup: main at 3c7f9bf, ROS 2 Lyrical + Gazebo Jetty headless in Docker (the project's cloud Gazebo setup),
seed 0, N = 2000 particles, odometry noise k = 1, lidar sigma 0.03 m, wheel slip compliance 0.5. Both test cases
PASS (`verdict.txt` in each folder).

```
results/<algorithm>/<test case>/
  <algorithm>.mp4   top-down video at 2x speed (see "Videos")
  summary.json      the run's evaluator output (all estimators, config)
  verdict.txt       the test case's PASS/FAIL lines
  trajectory.csv    ground truth + this estimator: t, x, y, yaw, std_x, std_y, std_yaw
  compute.csv       this estimator's per-update compute time (ms)
results/metrics.json  every number in the tables below
results/tools/        scripts used to run, render and summarize
```

## Accuracy

| Algorithm | Test case | Position RMSE | Worst error | Waypoint errors (G1 / G2 / G3) | Kidnap recovery |
|---|---|---|---|---|---|
| EKF | tc1 nominal | **0.059 m** | 0.22 m | 0.026 / 0.125 / 0.046 m | n/a |
| PF | tc1 nominal | 0.078 m | 0.23 m | 0.052 / 0.104 / 0.064 m | n/a |
| Hybrid | tc1 nominal | **0.059 m** | 0.22 m | 0.026 / 0.125 / 0.046 m | n/a |
| EKF | tc2 kidnap | 8.24 m | 20.5 m | 0.015 / 8.07 / 20.50 m | **never** (ends 20.5 m off) |
| PF | tc2 kidnap | 1.35 m | 9.07 m | 0.015 / 8.05 / 0.084 m | **7.2 s** |
| Hybrid | tc2 kidnap | 3.00 m | 9.73 m | 0.015 / 8.07 / 0.044 m | **7.4 s** |

Dead reckoning for scale: RMSE 7.6 m (tc1) and 4.9 m (tc2).
Recovery = back within 0.25 m of ground truth and staying there 3 s. The G2 error in tc2 is large for every
filter because the robot is teleported onto Goal 2, so the estimators are still where it was taken from.

Whole-run RMSE in tc2 is dominated by the kidnap, so here it is by phase (computed from `trajectory.csv`):

| tc2 | Before kidnap (RMSE) | Kidnap to recovery (mean error) | After recovery (RMSE) |
|---|---|---|---|
| EKF | 0.062 m | 12.8 m mean for the rest of the run | never recovers |
| PF | 0.089 m | 2.3 m | 0.091 m |
| Hybrid | 0.062 m | 7.7 m | **0.044 m** |

Takeaways:
- Without a kidnap the EKF is the most accurate (0.059 vs 0.078 m RMSE). The hybrid is identical to the EKF
  because its supervisor never re-seeds when the PF agrees.
- After a kidnap the EKF never recovers; its scan matcher only searches near its current belief.
- The PF and the hybrid both recover in about 7 s. The PF's estimate moves toward the truth gradually while the
  particles converge; the hybrid holds its wrong EKF estimate until the supervisor re-seeds it from the PF in one
  jump, so its error during that window is higher, but afterwards it tracks at EKF accuracy (0.044 m vs the PF's
  0.091 m).

## Compute cost

Per update = the estimator's own compute time per filter step (`compute.csv`, wall-clock ms). CPU = the
estimator process's CPU time / wall time (100% = one core), sampled from `/proc` every 2 s.

| Algorithm | Test case | Per update mean (p95) | Update rate (sim) | CPU, % of one core |
|---|---|---|---|---|
| EKF | tc1 | 14.7 (29.1) ms | 20 Hz (every scan) | 20.7% |
| PF | tc1 | 10.1 (19.1) ms | 11 Hz (after a minimum motion) | 22.8% |
| Hybrid | tc1 | EKF part 14.6 (30.5) ms + PF 10.1 ms | 20 Hz + 11 Hz | 61.3% (EKF 20.7 + PF 22.8 + supervisor 17.7) |
| EKF | tc2 | 48.3 (165) ms | 20 Hz | 25.8% |
| PF | tc2 | 10.3 (19.9) ms | 11 Hz | 21.6% |
| Hybrid | tc2 | EKF part 18.1 (41.5) ms + PF 10.3 ms | 20 Hz + 11 Hz | 58.6% (EKF 20.2 + PF 21.6 + supervisor 16.7) |

How to read it:
- The container was saturated (3.6 of 4 cores busy, sim at 0.3x real time), so absolute ms are inflated about
  2-3x against an idle machine. Compare them with each other, not against hardware budgets. The earlier offline
  benchmark (`/mnt/project-files/compute-cost/README.md`, single thread, no sim) measured 8.1 ms per EKF scan and
  5.3 ms per PF update at N = 2000.
- After the kidnap the lone EKF's cost triples (48 ms mean, 165 ms p95), because it widens its scan-match search
  while lost. The hybrid's EKF is re-seeded and stays cheap.
- Process CPU is dominated by rclpy overhead: an almost idle Python node in this stack (the kidnapper or noise
  injector) costs about 14%. So the hybrid's three processes cost about 3x one estimator here, while its
  algorithmic cost is about the EKF plus the PF (the supervisor check was 0.5 ms per PF update offline).

## Videos

Rendered offline from each run's recorded data (`tools/render_video.py`), not screen-recorded, so they play at
2x sim speed regardless of how slowly the sim ran. Each frame shows the factory map, the Gazebo ground-truth
footprint and trail (green), the estimate and trail, and its 2-sigma position ellipse from the logged std. The PF
video also shows a random 400 of the 2000 particles, logged as published. The right column plots position and
heading error with the filter's own 2-sigma bound; the dashed line is the 0.25 m pass threshold. The legend's
"lidar from estimate" entry is empty in these videos because scans were not logged.

## Reproduce

With the cloud Gazebo image from `/mnt/project-files/gazebo-cloud/setup.sh` and `results/tools` mounted at
`/tools`, run one at a time:

```
docker run --rm --network none $M -v $PWD/results/tools:/tools devol-gzt bash /tools/run_tc.sh 1   # then 2
docker run --rm $M -v $PWD/results:/results -v $PWD/results/tools:/tools devol-gzt bash -c \
  'source /opt/ros/lyrical/setup.bash; source /ws/install/setup.bash; python3 /tools/render_video.py /out/tc1 /results 0.2 10'
python3 tools/summarize.py <out dir>
```

`run_tc.sh` also runs `particle_logger.py` (its own process, 16-17% CPU, not counted above) and `cpusample.py`.
One seed per case, so these are single samples. The multi-seed numbers in
`/mnt/project-files/gazebo-cloud/pr18/NOTES.md` (three seeds) agree: EKF 0.054-0.060 m and PF 0.067-0.082 m on
tc1; PF and hybrid recovering in 5-9 s on tc2 while the EKF never does.
