import time
import math
import csv
from datetime import datetime
import matplotlib.pyplot as plt
from message_filters import ApproximateTimeSynchronizer, Subscriber

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException

# ROS 2 Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray

# Assuming these are available as before
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
        e_pos = math.hypot(dx, dy)

        # Calculate Heading Error (Shortest angular distance wrapped to [-pi, pi])
        dyaw = math.atan2(math.sin(est_pose[2] - gt_pose[2]), math.cos(est_pose[2] - gt_pose[2]))
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
        self.curr_rmse_pos = math.sqrt(self.sum_sq_pos / self.count)
        self.curr_rmse_yaw = math.sqrt(self.sum_sq_yaw / self.count)

        # Append to history for plotting
        self.times.append(t)
        self.err_pos.append(self.curr_err_pos)
        self.err_yaw.append(self.curr_err_yaw)
        self.rmse_pos.append(self.curr_rmse_pos)
        self.rmse_yaw.append(self.curr_rmse_yaw)


class LocalizationMetricsNode(Node):
    def __init__(self):
        super().__init__('localization_metrics_node')

        self.declare_parameter('rate_hz', 10.0)
        self.rate_hz = self.get_parameter('rate_hz').value

        # State Variables
        self.recording_started = False
        self.t0 = None  # Start time for x-axis zeroing

        # Initialize Trackers
        self.gt_pose = None
        self.ekf_pose = None
        self.ekf = ErrorTracker('EKF', 'blue')
        self.pf_pose = None
        self.pf = ErrorTracker('PF', 'orange')
        self.dr_pose = None
        self.dr = ErrorTracker('DR', 'red')
        self.trackers = [self.ekf, self.pf, self.dr]

        # Setup CSV Logging
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = f'localization_metrics_{timestamp}.csv'
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

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
                'PF_Pos_Err',
                'PF_Pos_RMSE',
                'PF_Pos_Integral',
                'PF_Yaw_Err',
                'PF_Yaw_RMSE',
                'PF_Yaw_Integral',
                'PF_Num_Particles',
                'DR_Pos_Err',
                'DR_Pos_RMSE',
                'DR_Pos_Integral',
                'DR_Yaw_Err',
                'DR_Yaw_RMSE',
                'DR_Yaw_Integral',
            ]
        )

        # Subscribers
        self.gt_sub: Subscriber = Subscriber(self, Odometry, '/devol_drive/ground_truth/odom')
        self.ekf_sub: Subscriber = Subscriber(
            self, PoseWithCovarianceStamped, '/devol_drive/ekf_pose'
        )
        self.pf_sub: Subscriber = Subscriber(
            self, PoseWithCovarianceStamped, '/devol_drive/pf_pose'
        )
        self.pf_cloud_sub: Subscriber = Subscriber(self, PoseArray, '/devol_drive/pf_pose_cloud')
        self.dr_sub: Subscriber = Subscriber(
            self, PoseWithCovarianceStamped, '/devol_drive/dr_pose'
        )

        # Syncronize Subs
        self.ts = ApproximateTimeSynchronizer(
            [self.gt_sub, self.ekf_sub, self.pf_sub, self.pf_cloud_sub, self.dr_sub],
            queue_size=20,
            slop=0.5,
        )
        self.ts.registerCallback(self.sync_callback)

        # Matplotlib objects
        self.fig = None
        self.axs = None
        self.lines = {}

        self.get_logger().info(f'Metrics Node Started. Logging to {self.csv_filename}')

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
        pf_msg: PoseWithCovarianceStamped,
        pf_cloud_msg: PoseArray,
        dr_msg: PoseWithCovarianceStamped,
    ):
        """This callback ONLY triggers when a matched set of timestamps arrives."""
        self.gt_pose = odom_to_planar(gt_msg)
        self.ekf_pose = pose_to_planar(ekf_msg)
        self.pf_num_particles = len(pf_cloud_msg.poses)
        self.pf_pose = pose_to_planar(pf_msg)
        self.dr_pose = pose_to_planar(dr_msg)

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

    def log_to_csv(self, t):
        if self.gt_pose is None:
            return

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
            f'{self.pf.curr_err_pos:.4f}',
            f'{self.pf.curr_rmse_pos:.4f}',
            f'{self.pf.integral_pos:.4f}',
            f'{self.pf.curr_err_yaw:.4f}',
            f'{self.pf.curr_rmse_yaw:.4f}',
            f'{self.pf.integral_yaw:.4f}',
            f'{self.pf_num_particles}',
            f'{self.dr.curr_err_pos:.4f}',
            f'{self.dr.curr_rmse_pos:.4f}',
            f'{self.dr.integral_pos:.4f}',
            f'{self.dr.curr_err_yaw:.4f}',
            f'{self.dr.curr_rmse_yaw:.4f}',
            f'{self.dr.integral_yaw:.4f}',
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()  # Ensure data is saved immediately in case of crash

    def render(self):
        if self.fig is None or not self.recording_started:
            return

        t = self.get_sim_time()
        self.log_to_csv(t)

        # Update plots dynamically
        for i, tracker in enumerate(self.trackers):
            if tracker.count == 0:
                continue

            l_err_p, l_rm_p, l_err_y, l_rm_y = self.lines[tracker.name]

            # Position Data
            l_err_p.set_data(tracker.times, tracker.err_pos)
            l_rm_p.set_data(tracker.times, tracker.rmse_pos)

            # Yaw Data
            l_err_y.set_data(tracker.times, tracker.err_yaw)
            l_rm_y.set_data(tracker.times, tracker.rmse_yaw)

            # Rescale axes to fit new data
            self.axs[i, 0].relim()
            self.axs[i, 0].autoscale_view(True, True, True)
            self.axs[i, 1].relim()
            self.axs[i, 1].autoscale_view(True, True, True)

            # Dynamically update subplot titles with the Integral Error
            if tracker.name == 'PF':
                self.axs[i, 0].set_title(
                    f'{tracker.name} Position Error (m) | Integral: {tracker.integral_pos:.2f} | Particles: {self.pf_num_particles}'
                )
            else:
                self.axs[i, 0].set_title(
                    f'{tracker.name} Position Error (m) | Integral: {tracker.integral_pos:.2f}'
                )

            self.axs[i, 1].set_title(
                f'{tracker.name} Heading Error (rad) | Integral: {tracker.integral_yaw:.2f}'
            )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def __del__(self):
        # Safely close CSV when destroyed
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMetricsNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Setup Matplotlib 3x2 Grid
    plt.ion()
    fig, axs = plt.subplots(3, 2, figsize=(12, 10), sharex='col')
    fig.canvas.manager.set_window_title('Localization Metrics (Live)')
    fig.suptitle('Instantaneous Error vs Cumulative RMSE', fontsize=16)

    # Create line objects for fast redrawing
    for i, tracker in enumerate(node.trackers):
        # Position Error Subplot (Column 0)
        (l_err_p,) = axs[i, 0].plot([], [], color=tracker.color, alpha=0.3, label='Inst Error')
        (l_rm_p,) = axs[i, 0].plot([], [], color=tracker.color, linewidth=2, label='RMSE')
        axs[i, 0].set_title(f'{tracker.name} Position Error (m)')
        axs[i, 0].grid(True, linestyle='--', alpha=0.6)
        axs[i, 0].legend(loc='upper left')

        # Heading Error Subplot (Column 1)
        (l_err_y,) = axs[i, 1].plot([], [], color=tracker.color, alpha=0.3, label='Inst Error')
        (l_rm_y,) = axs[i, 1].plot([], [], color=tracker.color, linewidth=2, label='RMSE')
        axs[i, 1].set_title(f'{tracker.name} Heading Error (rad)')
        axs[i, 1].grid(True, linestyle='--', alpha=0.6)
        axs[i, 1].legend(loc='upper left')

        node.lines[tracker.name] = (l_err_p, l_rm_p, l_err_y, l_rm_y)

    # X-axis labels for the bottom row
    axs[2, 0].set_xlabel('Time (s)')
    axs[2, 1].set_xlabel('Time (s)')

    plt.tight_layout()

    node.fig = fig
    node.axs = axs

    # Standard execution/render loop
    period = 1.0 / max(node.rate_hz, 0.1)
    next_draw = None

    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            timeout = (
                0.05 if next_draw is None else max(0.0, min(0.05, next_draw - time.monotonic()))
            )
            executor.spin_once(timeout_sec=timeout)

            now = time.monotonic()

            if not node.all_algorithms_ready:
                next_draw = None
                continue

            if next_draw is None:
                next_draw = now

            if now >= next_draw:
                node.render()
                next_draw += period

                if next_draw < now:
                    next_draw = now + period

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        plt.close(fig)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
