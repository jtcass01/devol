#!/usr/bin/env python3

from csv import writer
from datetime import datetime
from os import getcwd, makedirs
from os.path import isabs, join

import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy import init, spin, try_shutdown
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

# ROS 2 Messages
from geometry_msgs.msg import PointStamped, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from devol_localization.utils import odom_to_planar, pose_to_planar, yaw_from_quaternion


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'

ALGOS = ('EKF', 'PF', 'DR')
PLANAR_IDX = (0, 1, 5)  # x, y, yaw rows/cols of the ROS 6x6 pose covariance
COV_FIELDS = ('Var_X', 'Var_Y', 'Var_Yaw', 'Cov_XY', 'Cov_XYaw', 'Cov_YYaw')
COV_IJ = ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))
NAN_COV = np.full((3, 3), np.nan)


def fmt(values):
    return [f'{v:.9g}' for v in values]


def planar_covariance(msg: PoseWithCovarianceStamped):
    """3x3 (x, y, yaw) block of the ROS 6x6 pose covariance."""
    full = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
    return full[np.ix_(PLANAR_IDX, PLANAR_IDX)]


class LocalizationLoggingNode(Node):
    """Logs raw ground truth, estimates, covariances, and runtimes. All metrics are computed in post.

    EKF and DR covariances come from their PoseWithCovarianceStamped messages. The PF
    covariance is always computed from the particle cloud.
    """

    def __init__(self):
        super().__init__('localization_logging_node')

        self.declare_parameter('rate_hz', 10.0)  # max logging rate; <= 0 logs every synced set
        self.declare_parameter('results_dir', 'results')

        rate_hz = float(self.get_parameter('rate_hz').value)
        results_dir: str = self.get_parameter('results_dir').value

        # 5% tolerance so normal timestamp jitter at the source rate doesn't drop samples
        self.min_period = 0.95 / rate_hz if rate_hz > 0 else 0.0
        self.t0 = None
        self.last_write = None

        # CSV
        base = results_dir if isabs(results_dir) else join(getcwd(), results_dir)
        makedirs(base, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = join(base, f'localization_metrics_{timestamp}.csv')
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = writer(self.csv_file)

        header = ['Time(s)', 'GT_X', 'GT_Y', 'GT_Yaw']
        for a in ALGOS:
            header += [f'{a}_{f}' for f in ('X', 'Y', 'Yaw', *COV_FIELDS, 'Runtime')]
        header.append('PF_Num_Particles')
        self.csv_writer.writerow(header)

        # Subscribers
        self.gt_sub = Subscriber(self, Odometry, '/devol_drive/ground_truth/odom')
        self.ekf_sub = Subscriber(self, PoseWithCovarianceStamped, '/devol_drive/ekf_pose')
        self.ekf_rt_sub = Subscriber(self, PointStamped, '/devol_drive/ekf_pose/runtime')
        self.pf_sub = Subscriber(self, PoseWithCovarianceStamped, '/devol_drive/pf_pose')
        self.pf_cloud_sub = Subscriber(self, PoseArray, '/devol_drive/pf_pose_cloud')
        self.pf_rt_sub = Subscriber(self, PointStamped, '/devol_drive/pf_pose/runtime')
        self.dr_sub = Subscriber(self, PoseWithCovarianceStamped, '/devol_drive/dr_pose')
        self.dr_rt_sub = Subscriber(self, PointStamped, '/devol_drive/dr_pose/runtime')

        self.ts = ApproximateTimeSynchronizer(
            [self.gt_sub, self.ekf_sub, self.ekf_rt_sub, self.pf_sub, self.pf_cloud_sub,
             self.pf_rt_sub, self.dr_sub, self.dr_rt_sub],
            queue_size=20,
            slop=0.5,
        )
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info(
            f'Logging Node Started. Writing to {self.csv_filename} '
            f'(max rate: {rate_hz if rate_hz > 0 else "every sync"} Hz)'
        )

    def sync_callback(self, gt_msg, ekf_msg, ekf_rt_msg, pf_msg, pf_cloud_msg, pf_rt_msg,
                      dr_msg, dr_rt_msg):
        """Writes one CSV row per matched set of messages (throttled to rate_hz)."""
        t = Time.from_msg(gt_msg.header.stamp).nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
            self.get_logger().info('First synchronized sample received. Starting metrics collection.')
        t -= self.t0

        if self.last_write is not None and t - self.last_write < self.min_period:
            return
        self.last_write = t

        estimates = (
            ('EKF', ekf_msg, planar_covariance(ekf_msg), ekf_rt_msg),
            ('PF', pf_msg, planar_covariance(pf_msg), pf_rt_msg),
            ('DR', dr_msg, planar_covariance(dr_msg), dr_rt_msg),
        )

        row = [f'{t:.3f}', *fmt(odom_to_planar(gt_msg)[:3])]
        for _, msg, cov, rt in estimates:
            row += fmt(pose_to_planar(msg)[:3])
            row += fmt(cov[i, j] for i, j in COV_IJ)
            row.append(f'{rt.point.x:.6g}')
        row.append(str(len(pf_cloud_msg.poses)))

        self.csv_writer.writerow(row)
        self.csv_file.flush()  # keep data if the process crashes

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    init(args=args)
    node = LocalizationLoggingNode()
    try:
        spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try_shutdown()


if __name__ == '__main__':
    main()