#!/usr/bin/env python3
"""Scores one localization trial against Gazebo ground truth.

Inputs
  ground_truth_topic  (nav_msgs/Odometry)  Gazebo OdometryPublisher pose of the robot in the world (= map) frame
  odom_topic          (nav_msgs/Odometry)  odometry the filters see (after noise injection); integrated from
                                           start_pose as the dead-reckoning baseline
  <name>_pose         (geometry_msgs/PoseWithCovarianceStamped) one per entry in `estimators`
  <name>_compute_time_ms (std_msgs/Float64) per-update compute time, when the estimator publishes it

Outputs, written to output_dir on shutdown and every write_period seconds:
  trajectory.csv  every pose sample (ground truth, dead reckoning and each estimator) with its std
  compute.csv     every compute-time sample
  summary.json    the trial configuration and the metrics of metrics.score_estimator per estimator

The scenario sets the recovery event: 'global' scores recovery from the first ground-truth sample,
'kidnap' from the first teleport seen in the ground truth, 'nominal' scores no recovery.

With test_case 1 or 2 the node also judges that verification test (metrics.judge_test_case), prints
PASS/FAIL and writes verdict.txt. With finish_on_goal it ends the trial by itself: settle_time s after
the ground truth reaches the last waypoint, or at max_duration s of sim time, it writes the results
and exits, which the launch files turn into a shutdown of the whole run.
"""

import csv
import json
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from devol_localization.metrics import (detect_jump, judge_test_case, score_estimator, write_summary,
                                        write_trajectory_csv)
from devol_localization.pose2d import compose, covariance_3x3, relative, yaw_from_quaternion

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"

GROUND_TRUTH = 'ground_truth'
DEAD_RECKONING = 'dead_reckoning'


def stamp_seconds(stamp) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


class LocalizationEvaluator(Node):
    def __init__(self) -> None:
        super().__init__('localization_evaluator')
        self.declare_parameter('output_dir', 'results/trial')
        self.declare_parameter('scenario', 'nominal')          # nominal | global | kidnap
        self.declare_parameter('ground_truth_topic', '/devol_drive/ground_truth/odom')
        self.declare_parameter('odom_topic', '/devol_drive/noisy/odom')
        self.declare_parameter('namespace', '/devol_drive')
        self.declare_parameter('estimators', ['ekf', 'pf'])
        self.declare_parameter('start_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('waypoints', [0.0])               # flat [x0, y0, x1, y1, ...]
        self.declare_parameter('waypoint_radius', 0.5)
        self.declare_parameter('recovery_threshold', 0.25)
        self.declare_parameter('recovery_timeout', 60.0)
        self.declare_parameter('kidnap_jump', 1.0)
        self.declare_parameter('config_json', '{}')
        self.declare_parameter('write_period', 15.0)
        self.declare_parameter('test_case', 0)                  # 0 none, 1 nominal route, 2 kidnapping
        self.declare_parameter('finish_on_goal', False)
        self.declare_parameter('finish_radius', 0.5)
        self.declare_parameter('settle_time', 3.0)
        self.declare_parameter('max_duration', 0.0)             # sim seconds; 0 = no limit

        gp = self.get_parameter
        self._out = Path(str(gp('output_dir').value)).expanduser()
        self._out.mkdir(parents=True, exist_ok=True)
        self._scenario = str(gp('scenario').value)
        self._start_pose = np.asarray(gp('start_pose').value, dtype=float)
        wp = list(gp('waypoints').value)
        self._waypoints = [(wp[i], wp[i + 1]) for i in range(0, len(wp) - 1, 2)]
        try:
            self._config = json.loads(str(gp('config_json').value))
        except json.JSONDecodeError:
            self._config = {'config_json': str(gp('config_json').value)}
        self._config.update({'scenario': self._scenario, 'start_pose': self._start_pose.tolist(),
                             'waypoints': self._waypoints})

        # name -> list of [t, x, y, yaw, std_x, std_y, std_yaw]
        self._poses: Dict[str, List[List[float]]] = {GROUND_TRUTH: [], DEAD_RECKONING: []}
        self._compute: Dict[str, List[List[float]]] = {}
        self._odom0: Optional[np.ndarray] = None
        self._test_case = int(gp('test_case').value)
        self._finish_on_goal = bool(gp('finish_on_goal').value)
        self._goal_time: Optional[float] = None
        self.finished = False
        self.finish_reason = ''

        ns = str(gp('namespace').value).rstrip('/')
        self.create_subscription(Odometry, gp('ground_truth_topic').value, self._gt_cb, 100)
        self.create_subscription(Odometry, gp('odom_topic').value, self._odom_cb, 100)
        for name in gp('estimators').value:
            self._poses[name] = []
            self._compute[name] = []
            self.create_subscription(PoseWithCovarianceStamped, f'{ns}/{name}_pose',
                                     lambda msg, n=name: self._est_cb(n, msg), 50)
            self.create_subscription(Float64, f'{ns}/{name}_compute_time_ms',
                                     lambda msg, n=name: self._compute_cb(n, msg), 50)
        self.create_timer(float(gp('write_period').value), self.write)
        self.get_logger().info(f'Scoring {list(self._poses)} ({self._scenario}) into {self._out}')

    # ------------------------------------------------------------ callbacks
    def _gt_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose
        t = stamp_seconds(msg.header.stamp)
        self._poses[GROUND_TRUTH].append([t, p.position.x, p.position.y,
                                          yaw_from_quaternion(p.orientation), 0.0, 0.0, 0.0])
        if not self._finish_on_goal or self.finished:
            return
        gp = self.get_parameter
        t0 = self._poses[GROUND_TRUTH][0][0]
        max_duration = float(gp('max_duration').value)
        if self._waypoints and self._goal_time is None:
            gx, gy = self._waypoints[-1]
            if np.hypot(p.position.x - gx, p.position.y - gy) <= float(gp('finish_radius').value):
                self._goal_time = t
                self.get_logger().info(f'Reached the last waypoint at t = {t:.1f} s')
        if self._goal_time is not None and t - self._goal_time >= float(gp('settle_time').value):
            self.finished, self.finish_reason = True, 'reached the last waypoint'
        elif max_duration > 0.0 and t - t0 >= max_duration:
            self.finished, self.finish_reason = True, f'max_duration {max_duration:g} s reached'

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose
        odom = np.array([p.position.x, p.position.y, yaw_from_quaternion(p.orientation)])
        if self._odom0 is None:
            self._odom0 = odom
        dr = compose(self._start_pose, relative(self._odom0, odom))
        self._poses[DEAD_RECKONING].append([stamp_seconds(msg.header.stamp), *dr, 0.0, 0.0, 0.0])

    def _est_cb(self, name: str, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        std = np.sqrt(np.maximum(np.diag(covariance_3x3(msg.pose.covariance)), 0.0))
        self._poses[name].append([stamp_seconds(msg.header.stamp), p.position.x, p.position.y,
                                  yaw_from_quaternion(p.orientation), *std])

    def _compute_cb(self, name: str, msg: Float64) -> None:
        self._compute[name].append([self.get_clock().now().nanoseconds * 1e-9, float(msg.data)])

    # --------------------------------------------------------------- output
    def write(self) -> dict:
        rows = [[r[0], name, *r[1:]] for name, samples in self._poses.items() for r in samples]
        write_trajectory_csv(self._out / 'trajectory.csv', rows)
        with open(self._out / 'compute.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'estimator', 'compute_ms'])
            for name, samples in self._compute.items():
                for t, ms in samples:
                    w.writerow([f'{t:.6f}', name, f'{ms:.4f}'])

        gt = np.asarray(self._poses[GROUND_TRUTH], dtype=float).reshape(-1, 7)
        if gt.shape[0] < 2:
            return {}
        gt_t, gt_poses = gt[:, 0], gt[:, 1:4]
        event: Optional[float] = None
        if self._scenario == 'global':
            event = float(gt_t[0])
        elif self._scenario == 'kidnap':
            event = detect_jump(gt_t, gt_poses, float(self.get_parameter('kidnap_jump').value))
            self._config['kidnap_time'] = event
        scores = {}
        for name, samples in self._poses.items():
            if name == GROUND_TRUTH:
                continue
            a = np.asarray(samples, dtype=float).reshape(-1, 7)
            scores[name] = score_estimator(
                gt_t, gt_poses, a[:, 0], a[:, 1:4],
                compute_ms=[ms for _, ms in self._compute.get(name, [])],
                waypoints=self._waypoints, event_time=event,
                threshold=float(self.get_parameter('recovery_threshold').value),
                timeout=float(self.get_parameter('recovery_timeout').value),
                waypoint_radius=float(self.get_parameter('waypoint_radius').value))
        write_summary(self._out / 'summary.json', self._config, scores)
        return scores

    def verdict(self) -> Optional[bool]:
        """Writes the final results and, for a test case, prints and saves its PASS/FAIL report."""
        scores = self.write()
        if not self._test_case:
            return None
        if not scores:
            passed, lines = False, ['FAIL no ground truth received (is the ground-truth bridge running?)']
        else:
            passed, lines = judge_test_case(self._test_case, scores, self._config.get('waypoint_names', ()),
                                            float(self.get_parameter('recovery_threshold').value))
        title = {1: 'Test case 1: nominal three-waypoint route', 2: 'Test case 2: kidnapping'}[self._test_case]
        reason = f'; {self.finish_reason}' if self.finish_reason else ''
        report = '\n'.join([f'===== {title}: {"PASS" if passed else "FAIL"} =====', *lines,
                            f'(results in {self._out}{reason})'])
        (self._out / 'verdict.txt').write_text(report + '\n')
        print('\n' + report + '\n', flush=True)
        return passed


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationEvaluator()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.finished:
            node.get_logger().info(f'Trial finished: {node.finish_reason}')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        s = (node._out / 'summary.json')
        if node.verdict() is None and s.exists():
            for name, m in json.loads(s.read_text())['estimators'].items():
                print(f'[localization_evaluator] {name}: pos RMSE {m["pos_rmse"]} m, yaw RMSE {m["yaw_rmse"]} rad, '
                      f'recovered {m["recovered"]} after {m["recovery_time"]} s', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
