#!/usr/bin/env python3
"""Live view of one localization filter against Gazebo ground truth (rclpy + Matplotlib, no RViz).

mode:=ekf shows the EKF estimate, its 2-sigma ellipse, the lidar scan projected from the estimate
and the ground-truth pose; mode:=pf adds every particle. See viz_core.py for the layout.

Inputs (defaults for mode ekf; pf swaps ekf for pf)
  map_topic          /devol_drive/projected_map           nav_msgs/OccupancyGrid (transient local)
  ground_truth_topic /devol_drive/ground_truth/odom       nav_msgs/Odometry
  pose_topic         /devol_drive/ekf_pose                geometry_msgs/PoseWithCovarianceStamped
  scan_topic         /devol_drive/noisy/scan              sensor_msgs/LaserScan (what the filter sees)
  particles_topic    /devol_drive/pf_particles            geometry_msgs/PoseArray (pf only)
  compute_topic      /devol_drive/ekf_compute_time_ms     std_msgs/Float64

Set video_file to also write an MP4 (needs ffmpeg), and headless:=true to only write the video.
Drawing runs in the main thread between executor spins.
"""

import os
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

from devol_localization.pose2d import (covariance_3x3, transform_points, wrap_angle,
                                       yaw_from_quaternion)
from devol_localization.viz_core import LocalizationFigure, VizState

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


def stamp_seconds(stamp) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


class LocalizationViz(Node):
    def __init__(self) -> None:
        super().__init__('localization_viz')
        self.declare_parameter('mode', 'ekf')
        mode = str(self.get_parameter('mode').value)
        if mode not in ('ekf', 'pf'):
            raise ValueError(f'mode must be ekf or pf, got {mode}')
        self.declare_parameter('map_topic', '/devol_drive/projected_map')
        self.declare_parameter('ground_truth_topic', '/devol_drive/ground_truth/odom')
        self.declare_parameter('pose_topic', f'/devol_drive/{mode}_pose')
        self.declare_parameter('scan_topic', '/devol_drive/noisy/scan')
        self.declare_parameter('particles_topic', '/devol_drive/pf_particles')
        self.declare_parameter('compute_topic', f'/devol_drive/{mode}_compute_time_ms')
        self.declare_parameter('laser_pose', [0.32825, 0.0, 0.0])
        self.declare_parameter('beam_step', 2)
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('window', 16.0)        # metres; 0 = whole map
        self.declare_parameter('history', 0.0)        # seconds of error plot; 0 = whole run
        self.declare_parameter('headless', False)
        self.declare_parameter('video_file', '')
        self.declare_parameter('title', '')

        gp = self.get_parameter
        self.mode = mode
        self.rate_hz = float(gp('rate_hz').value)
        self._laser_pose = np.asarray(gp('laser_pose').value, dtype=float)
        self._beam_step = max(1, int(gp('beam_step').value))
        headless = bool(gp('headless').value) or not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        self.figure = LocalizationFigure(mode, title=str(gp('title').value), window=float(gp('window').value),
                                         history=float(gp('history').value), interactive=not headless)
        self.headless = headless
        self._writer = None
        video = str(gp('video_file').value)
        if video:
            from matplotlib.animation import FFMpegWriter
            os.makedirs(os.path.dirname(os.path.abspath(video)), exist_ok=True)
            self._writer = FFMpegWriter(fps=self.rate_hz, bitrate=4000)
            self._writer.setup(self.figure.fig, video, dpi=100)
            self.get_logger().info(f'Writing video to {video}')

        self._truth: Optional[np.ndarray] = None
        self._estimate: Optional[np.ndarray] = None
        self._cov: Optional[np.ndarray] = None
        self._scan_base: Optional[np.ndarray] = None   # scan points in the base frame
        self._particles: Optional[np.ndarray] = None
        self._compute_ms: Optional[float] = None
        self._stamp = 0.0
        self._truth_trail = []
        self._est_trail = []
        self._err = []   # [t, pos_err, pos_bound, yaw_err_deg, yaw_bound_deg]
        self._map_dirty = None

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, gp('map_topic').value, self._map_cb, map_qos)
        self.create_subscription(Odometry, gp('ground_truth_topic').value, self._gt_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, gp('pose_topic').value, self._pose_cb, 10)
        self.create_subscription(LaserScan, gp('scan_topic').value, self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, gp('compute_topic').value, self._compute_cb, 10)
        if mode == 'pf':
            self.create_subscription(PoseArray, gp('particles_topic').value, self._particles_cb, 1)
        self.get_logger().info(f'Visualizing {mode} ({"headless" if headless else "window"}) at {self.rate_hz:g} Hz')

    # ------------------------------------------------------------ callbacks (store only)
    def _map_cb(self, msg: OccupancyGrid) -> None:
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int8).reshape((info.height, info.width))
        self._map_dirty = (grid, info.resolution, info.origin.position.x, info.origin.position.y)

    def _gt_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._truth = np.array([p.position.x, p.position.y, yaw_from_quaternion(p.orientation)])
        if not self._truth_trail or np.hypot(*(self._truth[:2] - self._truth_trail[-1])) > 0.05:
            self._truth_trail.append(self._truth[:2].copy())

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self._estimate = np.array([p.position.x, p.position.y, yaw_from_quaternion(p.orientation)])
        self._cov = covariance_3x3(msg.pose.covariance)
        self._stamp = stamp_seconds(msg.header.stamp)
        if not self._est_trail or np.hypot(*(self._estimate[:2] - self._est_trail[-1])) > 0.05:
            self._est_trail.append(self._estimate[:2].copy())
        if self._truth is not None:
            pos_err = float(np.hypot(*(self._estimate[:2] - self._truth[:2])))
            yaw_err = float(np.degrees(wrap_angle(self._estimate[2] - self._truth[2])))
            pos_bound = 2.0 * float(np.sqrt(max(np.linalg.eigvalsh(self._cov[:2, :2]).max(), 0.0)))
            yaw_bound = 2.0 * float(np.degrees(np.sqrt(max(self._cov[2, 2], 0.0))))
            self._err.append([self._stamp, pos_err, pos_bound, yaw_err, yaw_bound])

    def _scan_cb(self, msg: LaserScan) -> None:
        r = np.asarray(msg.ranges, dtype=float)[::self._beam_step]
        a = msg.angle_min + np.arange(len(msg.ranges))[::self._beam_step] * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        pts = np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])))
        self._scan_base = transform_points(self._laser_pose, pts)

    def _particles_cb(self, msg: PoseArray) -> None:
        self._particles = np.array([[q.position.x, q.position.y, yaw_from_quaternion(q.orientation)]
                                    for q in msg.poses]).reshape(-1, 3)

    def _compute_cb(self, msg: Float64) -> None:
        self._compute_ms = float(msg.data)

    # ------------------------------------------------------------ drawing (main thread)
    def render(self) -> None:
        if self._map_dirty is not None:
            self.figure.set_map(*self._map_dirty)
            self._map_dirty = None
        err = np.asarray(self._err, dtype=float).reshape(-1, 5)
        scan = None
        if self._scan_base is not None and self._estimate is not None:
            scan = transform_points(self._estimate, self._scan_base)
        status = '' if self._estimate is not None else f'waiting for {self.mode}_pose'
        self.figure.update(VizState(
            stamp=self._stamp, truth=self._truth, estimate=self._estimate, covariance=self._cov,
            scan_points=scan, particles=self._particles if self.mode == 'pf' else None,
            truth_trail=np.asarray(self._truth_trail).reshape(-1, 2),
            estimate_trail=np.asarray(self._est_trail).reshape(-1, 2),
            err_t=err[:, 0], pos_err=err[:, 1], pos_bound=err[:, 2], yaw_err=err[:, 3], yaw_bound=err[:, 4],
            compute_ms=self._compute_ms, status=status))
        self.figure.draw()
        if self._writer is not None:
            self._writer.grab_frame()

    def close(self) -> None:
        if self._writer is not None:
            self._writer.finish()
            self._writer = None
        self.figure.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationViz()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    period = 1.0 / max(node.rate_hz, 0.1)
    next_draw = time.monotonic()
    try:
        while rclpy.ok() and (node.headless or node.figure.is_open()):
            executor.spin_once(timeout_sec=max(0.0, min(0.05, next_draw - time.monotonic())))
            now = time.monotonic()
            if now >= next_draw:
                node.render()
                next_draw += period
                if next_draw < now:
                    next_draw = now + period
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
