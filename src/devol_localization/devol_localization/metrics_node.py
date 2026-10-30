#!/usr/bin/env python3

from math import hypot, atan2, sin, cos, sqrt
from csv import writer
from datetime import datetime
from message_filters import ApproximateTimeSynchronizer, Subscriber

from rclpy import init, spin, try_shutdown
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

# ROS 2 Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PointStamped

from devol_localization.utils import odom_to_planar, pose_to_planar


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


class ErrorTracker:
    """Tracks and calculates instantaneous error and RMSE for a specific estimator."""

    def __init__(self, name, color):
        self.name = name
        self.color = color

        # Cumulative stats for RMSE
        self.sum_sq_pos = 0.0
        self.sum_sq_yaw = 0.0
        self.count = 0

        # Cumulative stats for Integral of Absolute Error
        self.integral_pos = 0.0
        self.integral_yaw = 0.0
        self.last_time = None

        # History arrays for plotting
        self.times = []
        self.err_pos = []
        self.err_yaw = []
        self.rmse_pos = []
        self.rmse_yaw = []

        # Current values for CSV logging
        self.curr_err_pos = 0.0
        self.curr_err_yaw = 0.0
        self.curr_rmse_pos = 0.0
        self.curr_rmse_yaw = 0.0

    def update(self, t, est_pose, gt_pose):
        if gt_pose is None:
            return

        # Calculate Position Error (Euclidean distance)
        dx = est_pose[0] - gt_pose[0]
        dy = est_pose[1] - gt_pose[1]
        e_pos = hypot(dx, dy)

        # Calculate Heading Error (Shortest angular distance wrapped to [-pi, pi])
        dyaw = atan2(sin(est_pose[2] - gt_pose[2]), cos(est_pose[2] - gt_pose[2]))
        e_yaw = abs(dyaw)

        # Acculate Integral Error with Trapezoidal Rule
        if self.last_time is not None:
            dt = t - self.last_time
            if dt > 0:
                self.integral_pos += ((self.curr_err_pos + e_pos) / 2.0) * dt
                self.integral_yaw += ((self.curr_err_yaw + e_yaw) / 2.0) * dt
        self.last_time = t

        # Accumulate for RMSE
        self.sum_sq_pos += e_pos**2
        self.sum_sq_yaw += e_yaw**2
        self.count += 1

        # Update Current state
        self.curr_err_pos = e_pos
        self.curr_err_yaw = e_yaw
        self.curr_rmse_pos = sqrt(self.sum_sq_pos / self.count)
        self.curr_rmse_yaw = sqrt(self.sum_sq_yaw / self.count)

        # Append to history for plotting
        self.times.append(t)
        self.err_pos.append(self.curr_err_pos)
        self.err_yaw.append(self.curr_err_yaw)
        self.rmse_pos.append(self.curr_rmse_pos)
        self.rmse_yaw.append(self.curr_rmse_yaw)

class LocalizationLoggingNode(Node):
    def __init__(self):
        super().__init__('localization_logging_node')

        self.declare_parameter('rate_hz', 10.0)
        self.rate_hz = self.get_parameter('rate_hz').value

        # State Variables
        self.recording_started = False
        self.t0 = None  # Start time for x-axis zeroing

        # Initialize Trackers
        self.gt_pose = None
        self.ekf_pose = None
        self.ekf_runtime = 0.0
        self.ekf = ErrorTracker('EKF', None)
        self.pf_pose = None
        self.pf_runtime = 0.0
        self.pf_num_particles = 0
        self.pf = ErrorTracker('PF', None)
        self.dr_pose = None
        self.dr_runtime = 0.0
        self.dr = ErrorTracker('DR', None)

        # Setup CSV Logging
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = f'localization_metrics_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = writer(self.csv_file)

        # Write CSV Header
        self.csv_writer.writerow(
            [
                'Time(s)',
                'GT_X',
                'GT_Y',
                'GT_Yaw',
                'EKF_Pos_Err',
                'EKF_Pos_RMSE',
                'EKF_Pos_Integral',
                'EKF_Yaw_Err',
                'EKF_Yaw_RMSE',
                'EKF_Yaw_Integral',
                'EKF_Runtime',
                'PF_Pos_Err',
                'PF_Pos_RMSE',
                'PF_Pos_Integral',
                'PF_Yaw_Err',
                'PF_Yaw_RMSE',
                'PF_Yaw_Integral',
                'PF_Num_Particles',
                'PF_Runtime',
                'DR_Pos_Err',
                'DR_Pos_RMSE',
                'DR_Pos_Integral',
                'DR_Yaw_Err',
                'DR_Yaw_RMSE',
                'DR_Yaw_Integral',
                'DR_Runtime',
            ]
        )

        # Subscribers
        self.gt_sub = Subscriber(self, Odometry, '/devol_drive/ground_truth/odom')

        self.ekf_sub = Subscriber(self, PoseWithCovarianceStamped, '/devol_drive/ekf_pose')
        self.ekf_rt_sub = Subscriber(self, PointStamped, '/devol_drive/ekf_pose/runtime')

        self.pf_sub = Subscriber(self, PoseWithCovarianceStamped, '/devol_drive/pf_pose')
        self.pf_cloud_sub = Subscriber(self, PoseArray, '/devol_drive/pf_pose_cloud')
        self.pf_rt_sub = Subscriber(self, PointStamped, '/devol_drive/pf_pose/runtime')

        self.dr_sub = Subscriber(self, PoseWithCovarianceStamped, '/devol_drive/dr_pose')
        self.dr_rt_sub = Subscriber(self, PointStamped, '/devol_drive/dr_pose/runtime')

        # Syncronize Subs
        self.ts = ApproximateTimeSynchronizer(
            [
                self.gt_sub,
                self.ekf_sub,
                self.ekf_rt_sub,
                self.pf_sub,
                self.pf_cloud_sub,
                self.pf_rt_sub,
                self.dr_sub,
                self.dr_rt_sub,
            ],
            queue_size=20,
            slop=0.5,
        )
        self.ts.registerCallback(self.sync_callback)

        # Timer for logging
        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Logging Node Started. Writing to {self.csv_filename} at {self.rate_hz} Hz'
        )

    def get_sim_time(self):
        """Returns time in seconds. Initializes t0 on first call."""
        t = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    @property
    def all_algorithms_ready(self):
        return self.ekf_pose is not None and self.pf_pose is not None and self.dr_pose is not None

    def sync_callback(
        self,
        gt_msg: Odometry,
        ekf_msg: PoseWithCovarianceStamped,
        ekf_rt_msg: PointStamped,
        pf_msg: PoseWithCovarianceStamped,
        pf_cloud_msg: PoseArray,
        pf_rt_msg: PointStamped,
        dr_msg: PoseWithCovarianceStamped,
        dr_rt_msg: PointStamped,
    ):
        """This callback ONLY triggers when a matched set of timestamps arrives."""
        self.gt_pose = odom_to_planar(gt_msg)

        self.ekf_pose = pose_to_planar(ekf_msg)
        self.ekf_runtime = ekf_rt_msg.point.x

        self.pf_num_particles = len(pf_cloud_msg.poses)
        self.pf_pose = pose_to_planar(pf_msg)
        self.pf_runtime = pf_rt_msg.point.x

        self.dr_pose = pose_to_planar(dr_msg)
        self.dr_runtime = dr_rt_msg.point.x

        self.start_recording_if_ready()

        if self.recording_started:
            sim_time = self.get_sim_time()
            self.ekf.update(sim_time, self.ekf_pose, self.gt_pose)
            self.pf.update(sim_time, self.pf_pose, self.gt_pose)
            self.dr.update(sim_time, self.dr_pose, self.gt_pose)

    def start_recording_if_ready(self):
        if not self.recording_started and self.all_algorithms_ready and self.gt_pose is not None:
            # Start a common time baseline for all algorithms.
            self.t0 = self.get_clock().now().nanoseconds / 1e9
            self.recording_started = True
            self.get_logger().info(
                'All localization algorithms ready. Starting metrics collection.'
            )

    def timer_callback(self):
        if not self.recording_started or self.gt_pose is None:
            return

        t = self.get_sim_time()
        row = [
            f'{t:.3f}',
            f'{self.gt_pose[0]:.3f}',
            f'{self.gt_pose[1]:.3f}',
            f'{self.gt_pose[2]:.3f}',
            f'{self.ekf.curr_err_pos:.4f}',
            f'{self.ekf.curr_rmse_pos:.4f}',
            f'{self.ekf.integral_pos:.4f}',
            f'{self.ekf.curr_err_yaw:.4f}',
            f'{self.ekf.curr_rmse_yaw:.4f}',
            f'{self.ekf.integral_yaw:.4f}',
            f'{self.ekf_runtime:.6f}',
            f'{self.pf.curr_err_pos:.4f}',
            f'{self.pf.curr_rmse_pos:.4f}',
            f'{self.pf.integral_pos:.4f}',
            f'{self.pf.curr_err_yaw:.4f}',
            f'{self.pf.curr_rmse_yaw:.4f}',
            f'{self.pf.integral_yaw:.4f}',
            f'{self.pf_num_particles}',
            f'{self.pf_runtime:.6f}',
            f'{self.dr.curr_err_pos:.4f}',
            f'{self.dr.curr_rmse_pos:.4f}',
            f'{self.dr.integral_pos:.4f}',
            f'{self.dr.curr_err_yaw:.4f}',
            f'{self.dr.curr_rmse_yaw:.4f}',
            f'{self.dr.integral_yaw:.4f}',
            f'{self.dr_runtime:.6f}',
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()  # Ensure data is saved immediately in case of crash

    def destroy_node(self):
        """Safely close CSV when destroyed"""
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    init(args=args)
    node = LocalizationLoggingNode()

    try:
        # We can just spin normally now since there's no GUI to update!
        spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try_shutdown()


if __name__ == '__main__':
    main()