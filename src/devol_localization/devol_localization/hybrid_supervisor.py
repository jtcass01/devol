#!/usr/bin/env python3
"""Hybrid EKF + PF supervisor node: re-seeds an EKF from the particle filter after a kidnap.

Runs next to an EKF instance of its own (hybrid_localization.launch.py starts one, publishing
hybrid_pose) and the regular particle filter node. After every particle filter update it runs
devol_localization.hybrid.KidnapMonitor on the two estimates and the latest scan; when the
monitor calls for it, it publishes the particle filter's mean and floored covariance on the
hybrid EKF's initial pose topic, which resets that EKF's prior. The standalone EKF and PF are not
touched, so their results in the trade study do not change.

Inputs
  ekf_pose_topic   (PoseWithCovarianceStamped) the hybrid EKF's pose
  pf_pose_topic    (PoseWithCovarianceStamped) the particle filter's pose
  pf_update_topic  (std_msgs/Float64) the PF's compute time, published once per filter update
  scan_topic       (LaserScan) the scan both filters use
  map_topic        (OccupancyGrid) the known map, for the scan likelihood test

Outputs
  reset_topic      (PoseWithCovarianceStamped) the hybrid EKF's initial pose topic
"""

from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

from devol_localization.graceful import init_with_stop_flag
from devol_localization.hybrid import HybridParams, KidnapMonitor, scan_log_likelihood
from devol_localization.particle_filter import LikelihoodField, PFParams
from devol_localization.pf_localization_node import set_yaw, yaw_from_quaternion

__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


def pose_and_cov(msg: PoseWithCovarianceStamped):
    p = msg.pose.pose
    x = np.array([p.position.x, p.position.y, yaw_from_quaternion(p.orientation)])
    c = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
    idx = [0, 1, 5]
    return x, c[np.ix_(idx, idx)]


class HybridSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('hybrid_supervisor')
        gp = self.get_parameter
        self.declare_parameter('ekf_pose_topic', '/devol_drive/hybrid_pose')
        self.declare_parameter('pf_pose_topic', '/devol_drive/pf_pose')
        self.declare_parameter('pf_update_topic', '/devol_drive/pf_compute_time_ms')
        self.declare_parameter('scan_topic', '/devol_drive/sensors/lidar2d_0/scan')
        self.declare_parameter('map_topic', '/devol_drive/projected_map')
        self.declare_parameter('reset_topic', '/devol_drive/hybrid_initialpose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('laser_pose', [0.32825, 0.0, 0.0])
        # Scan likelihood model; keep equal to the PF node's.
        self.declare_parameter('sigma_hit', 0.3)
        self.declare_parameter('z_hit', 0.9)
        self.declare_parameter('z_rand', 0.1)
        self.declare_parameter('max_beams', 30)
        self.declare_parameter('likelihood_max_dist', 2.0)
        # KidnapMonitor (see HybridParams).
        d = HybridParams()
        for name in (
            'gate',
            'pf_max_std_xy',
            'pf_max_std_yaw',
            'cov_floor_xy',
            'cov_floor_yaw',
            'min_loglik_gain',
            'min_pf_loglik',
        ):
            self.declare_parameter(name, float(getattr(d, name)))
        self.declare_parameter('confirm_updates', d.confirm_updates)
        self.declare_parameter('cooldown_updates', d.cooldown_updates)

        self._monitor = KidnapMonitor(
            HybridParams(
                gate=float(gp('gate').value),
                pf_max_std_xy=float(gp('pf_max_std_xy').value),
                pf_max_std_yaw=float(gp('pf_max_std_yaw').value),
                cov_floor_xy=float(gp('cov_floor_xy').value),
                cov_floor_yaw=float(gp('cov_floor_yaw').value),
                min_loglik_gain=float(gp('min_loglik_gain').value),
                min_pf_loglik=float(gp('min_pf_loglik').value),
                confirm_updates=int(gp('confirm_updates').value),
                cooldown_updates=int(gp('cooldown_updates').value),
            )
        )
        self._lik = PFParams(
            sigma_hit=float(gp('sigma_hit').value),
            z_hit=float(gp('z_hit').value),
            z_rand=float(gp('z_rand').value),
            max_beams=int(gp('max_beams').value),
        )
        self._lik_max_dist = float(gp('likelihood_max_dist').value)
        self._laser_pose = tuple(float(v) for v in gp('laser_pose').value)
        self._map_frame = str(gp('map_frame').value)

        self._field: Optional[LikelihoodField] = None
        self._scan: Optional[LaserScan] = None
        self._ekf: Optional[PoseWithCovarianceStamped] = None
        self._pf_updated = False

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._reset_pub = self.create_publisher(
            PoseWithCovarianceStamped, gp('reset_topic').value, 10
        )
        self.create_subscription(OccupancyGrid, gp('map_topic').value, self._map_cb, map_qos)
        self.create_subscription(
            LaserScan, gp('scan_topic').value, self._scan_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseWithCovarianceStamped, gp('ekf_pose_topic').value, self._ekf_cb, 10
        )
        self.create_subscription(Float64, gp('pf_update_topic').value, self._pf_update_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, gp('pf_pose_topic').value, self._pf_pose_cb, 10
        )
        self.get_logger().info(
            f'Hybrid supervisor: re-seeding the EKF on {gp("reset_topic").value}'
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        grid = np.asarray(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self._field = LikelihoodField(
            grid,
            msg.info.resolution,
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            max_dist=self._lik_max_dist,
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan = msg

    def _ekf_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._ekf = msg

    def _pf_update_cb(self, _msg: Float64) -> None:
        # The PF node publishes its pose right after this, from the same update.
        self._pf_updated = True

    def _pf_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        if not self._pf_updated:
            return  # dead-reckoned between updates; check only on fresh updates
        self._pf_updated = False
        if self._ekf is None or self._scan is None or self._field is None:
            return
        pf_x, pf_P = pose_and_cov(msg)
        ekf_x, ekf_P = pose_and_cov(self._ekf)
        s = self._scan
        ranges = np.asarray(s.ranges, dtype=float)
        angles = s.angle_min + np.arange(ranges.size) * s.angle_increment
        ll = scan_log_likelihood(
            self._field,
            self._lik,
            np.vstack((pf_x, ekf_x)),
            ranges,
            angles,
            s.range_min,
            s.range_max,
            self._laser_pose,
        )
        if not self._monitor.check(ekf_x, ekf_P, pf_x, pf_P, float(ll[0] - ll[1]), float(ll[0])):
            return
        x, P = self._monitor.reseed_prior(pf_x, pf_P)
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._map_frame
        out.pose.pose.position.x = float(x[0])
        out.pose.pose.position.y = float(x[1])
        set_yaw(out.pose.pose.orientation, float(x[2]))
        c = [0.0] * 36
        for i, ci in enumerate((0, 1, 5)):
            for j, cj in enumerate((0, 1, 5)):
                c[ci * 6 + cj] = float(P[i, j])
        out.pose.covariance = c
        self._reset_pub.publish(out)
        self._monitor.stats.reseeds += 1
        jump = float(np.hypot(*(x[:2] - ekf_x[:2])))
        self.get_logger().info(
            f'Kidnap detected: EKF re-seeded from the PF at x={x[0]:.2f} y={x[1]:.2f} '
            f'yaw={x[2]:.2f} ({jump:.2f} m from the EKF; re-seed {self._monitor.stats.reseeds})'
        )


def main(args=None):
    stop = init_with_stop_flag(args)
    node = HybridSupervisor()
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
