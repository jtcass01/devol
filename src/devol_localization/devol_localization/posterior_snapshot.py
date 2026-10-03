#!/usr/bin/env python3
"""Saves "intermediate posterior" figures: the PF's particle set and the EKF's 2-sigma Gaussian at the
same instant, with ground truth (viz_core.posterior_figure). Off screen; writes PNGs only.

Snapshots are taken at `times` (sim seconds after the first ground-truth message) and at
`after_kidnap` seconds after a teleport is seen in the ground truth (a jump of more than
kidnap_jump metres between consecutive samples). Files: output_dir/posterior_t<sim time>.png.
"""

import os
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from devol_localization.pose2d import covariance_3x3, transform_points, yaw_from_quaternion

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


def stamp_seconds(stamp) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


def planar(pose) -> np.ndarray:
    return np.array([pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)])


class PosteriorSnapshot(Node):
    def __init__(self) -> None:
        super().__init__('posterior_snapshot')
        self.declare_parameter('output_dir', 'results/trial')
        self.declare_parameter('times', [20.0, 60.0])
        self.declare_parameter('after_kidnap', [1.0, 5.0, 15.0])
        self.declare_parameter('kidnap_jump', 1.0)
        self.declare_parameter('window', 10.0)
        self.declare_parameter('laser_pose', [0.32825, 0.0, 0.0])
        self.declare_parameter('map_topic', '/devol_drive/projected_map')
        self.declare_parameter('ground_truth_topic', '/devol_drive/ground_truth/odom')
        self.declare_parameter('scan_topic', '/devol_drive/noisy/scan')
        self.declare_parameter('title', '')
        gp = self.get_parameter
        self._out = os.path.expanduser(str(gp('output_dir').value))
        os.makedirs(self._out, exist_ok=True)
        self._times: List[float] = sorted(float(t) for t in gp('times').value)
        self._after_kidnap: List[float] = sorted(float(t) for t in gp('after_kidnap').value)
        self._jump = float(gp('kidnap_jump').value)
        self._window = float(gp('window').value)
        self._laser = np.asarray(gp('laser_pose').value, dtype=float)
        self._title = str(gp('title').value)

        self._map = None
        self._truth: Optional[np.ndarray] = None
        self._t0: Optional[float] = None
        self._due: List[float] = []          # absolute sim times still to capture
        self._kidnap_seen = False
        self._ekf = self._ekf_cov = self._pf = self._pf_cov = None
        self._particles: Optional[np.ndarray] = None
        self._scan = None

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, gp('map_topic').value, self._map_cb, map_qos)
        self.create_subscription(Odometry, gp('ground_truth_topic').value, self._gt_cb, 50)
        self.create_subscription(PoseWithCovarianceStamped, '/devol_drive/ekf_pose', self._ekf_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/devol_drive/pf_pose', self._pf_cb, 10)
        self.create_subscription(PoseArray, '/devol_drive/pf_particles', self._particles_cb, 1)
        self.create_subscription(LaserScan, gp('scan_topic').value, self._scan_cb, qos_profile_sensor_data)
        self.get_logger().info(f'Posterior snapshots at t0 + {self._times} s and kidnap + {self._after_kidnap} s '
                               f'into {self._out}')

    def _map_cb(self, msg: OccupancyGrid) -> None:
        info = msg.info
        self._map = (np.asarray(msg.data, dtype=np.int8).reshape((info.height, info.width)), info.resolution,
                     (info.origin.position.x, info.origin.position.y))

    def _ekf_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._ekf, self._ekf_cov = planar(msg.pose.pose), covariance_3x3(msg.pose.covariance)

    def _pf_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._pf, self._pf_cov = planar(msg.pose.pose), covariance_3x3(msg.pose.covariance)

    def _particles_cb(self, msg: PoseArray) -> None:
        self._particles = np.array([planar(p) for p in msg.poses]).reshape(-1, 3)

    def _scan_cb(self, msg: LaserScan) -> None:
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        self._scan = transform_points(self._laser, np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]))))

    def _gt_cb(self, msg: Odometry) -> None:
        t = stamp_seconds(msg.header.stamp)
        pose = planar(msg.pose.pose)
        if self._t0 is None:
            self._t0 = t
            self._due = [t + dt for dt in self._times]
        elif not self._kidnap_seen and np.hypot(*(pose[:2] - self._truth[:2])) > self._jump:
            self._kidnap_seen = True
            self._due = sorted(self._due + [t + dt for dt in self._after_kidnap])
            self.get_logger().info(f'Kidnap seen at t = {t:.2f} s')
        self._truth = pose
        while self._due and t >= self._due[0]:
            self._due.pop(0)
            self.save(t)

    def save(self, t: float) -> None:
        if self._map is None:
            self.get_logger().warning(f'No map yet; skipping the snapshot at t = {t:.1f} s')
            return
        from devol_localization.viz_core import posterior_figure
        grid, res, origin = self._map
        scan = transform_points(self._truth, self._scan) if self._scan is not None else None
        fig = posterior_figure(grid, res, origin, self._truth, self._ekf, self._ekf_cov, self._pf, self._pf_cov,
                               self._particles, scan, stamp=t, window=self._window,
                               title=f'{self._title} t = {t:.1f} s'.strip())
        path = os.path.join(self._out, f'posterior_t{t:06.1f}.png')
        fig.savefig(path, dpi=120)
        self.get_logger().info(f'Saved {path}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PosteriorSnapshot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
