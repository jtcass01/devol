#!/usr/bin/env python3

from typing import Tuple
from time import perf_counter
from scipy.spatial import KDTree
from numpy import (
    ndarray,
    array,
    zeros,
    diag,
    deg2rad,
    mean,
    sum,
    sin,
    cos,
    vstack,
    where,
    int8 as np_int8,
    eye,
    arctan2,
    trace,
    sqrt,
    clip,
    trace,
    sqrt,
    clip,
)
from numpy.linalg import svd, det, inv

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ROS 2 Messages
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

# Import devol localization classes
from devol_localization.base_localization_node import BaseLocalizationNode
from devol_localization.motion_models import UnicycleMotionModel
from devol_localization.utils import wrap_angle


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


# Chi-square threshold for 3 degrees of freedom at 95% confidence
CHI2_TRESHOLD_3DOF_95CONF: float = 7.815


class EKFLocalizationNode(BaseLocalizationNode):
    def __init__(self):
        super().__init__(
            'ekf_localization_node',
            '/devol_drive/ekf_pose',
            runtime_topic='/devol_drive/ekf_pose/runtime',
        )

        self.map_data = None
        self.W_pose: ndarray = diag([0.1**2, 0.1**2, deg2rad(3.0) ** 2])

        # State variables.
        self.x_est: ndarray = array(
            [0.0, 0.0, 0.0, 1.0]
        )  # [x, y, theta, s], where true yaw rate = s * raw odometry yaw
        self.covariance: ndarray = diag([0.1**2, 0.1**2, deg2rad(5.0) ** 2, 0.3**2])
        self.q_yaw_scale: float = 0.02

        # Time synchronization variables
        self.odom_buffer: list = []
        self.last_time: float = None
        self.current_v: float = 0.0
        self.current_omega: float = 0.0

        # Total runtime accumulator
        self.localization_runtime: float = 0.0

        # Subscribers / Publishers
        map_qos: QoSProfile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.get_logger().info('EKF Localization Node Started!')

    def odom_callback(self, msg: Odometry):
        """Base callback to track the latest odometry twist."""
        # Convert ROS timestamp to a float in seconds
        stamp = msg.header.stamp
        t_odom = stamp.sec + stamp.nanosec * 1e-9

        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z / self.x_est[3]

        # Add to the buffer
        self.odom_buffer.append((t_odom, v, omega))

    def odom_callback(self, msg: Odometry):
        """Base callback to track the latest odometry twist."""
        # Convert ROS timestamp to a float in seconds
        stamp = msg.header.stamp
        t_odom = stamp.sec + stamp.nanosec * 1e-9

        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z / self.x_est[3]

        # Add to the buffer
        self.odom_buffer.append((t_odom, v, omega))

    def map_callback(self, msg: OccupancyGrid):
        """Extracts obstacle coordinates and builds KDTree."""
        start_time: float = perf_counter()

        if not self.is_map_new(msg):
            self.localization_runtime += perf_counter() - start_time
            return

        grid: ndarray = array(msg.data, dtype=np_int8).reshape((msg.info.height, msg.info.width))

        # Find the row/col indices of all cells that are likely obstacles
        obstacle_y, obstacle_x = where(grid > 50)

        # Convert indices to real-world metric coordinates
        map_pts_x: ndarray = msg.info.origin.position.x + (obstacle_x + 0.5) * msg.info.resolution
        map_pts_y: ndarray = msg.info.origin.position.y + (obstacle_y + 0.5) * msg.info.resolution

        # Stack into an Nx2 array of obstacle points
        map_points: ndarray = vstack((map_pts_x, map_pts_y)).T

        # Build the KDTree for the ICP algorithm
        self.map_data: dict = {'points': map_points, 'kdtree': KDTree(map_points)}
        self.get_logger().info(
            f'Map received. Built KDTree with {len(map_points)} obstacle points.'
        )

        self.localization_runtime += perf_counter() - start_time

    def scan_match(self, msg: LaserScan, max_iter: int = 50, tolerance: float = 1e-4):
        """2D Iterative Closest Point using SVD."""
        # Filter out invalid rays
        ranges, angles = self.preprocess_scan(msg=msg)

        if len(ranges) < 10:
            return array([0.0, 0.0, 0.0, 1.0]), False, 0.0  # Not enough valid points
            return array([0.0, 0.0, 0.0, 1.0]), False, 0.0  # Not enough valid points

        # Nx2 array of local laser points
        laser_pts: ndarray = vstack((ranges * cos(angles), ranges * sin(angles))).T
        laser_offset: ndarray = self.get_laser_offset()

        R_offset: ndarray = array(
            [
                [cos(laser_offset[2]), -sin(laser_offset[2])],
                [sin(laser_offset[2]), cos(laser_offset[2])],
            ]
        )
        local_pts: ndarray = (R_offset @ laser_pts.T).T + laser_offset[:2]

        current_pose: ndarray = self.x_est[:3].copy()
        prev_error: float = float('inf')

        # Calculate adaptive correspondence gate based on positional covariance
        pos_variance = trace(self.covariance[:2, :2])
        #  3-sigma radius, clamped between 0.5m and 2.0m
        search_radius: float = float(clip(3.0 * sqrt(pos_variance), 0.5, 2.0))

        for _ in range(max_iter):
            # Transform local scan points into the global map frame
            theta: float = current_pose[2]
            R_curr: ndarray = array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
            global_pts: ndarray = (R_curr @ local_pts.T).T + current_pose[:2]

            # Find the closest Map points (Q) for every Global Scan point (P)
            distances, indices = self.map_data['kdtree'].query(global_pts)

            # Reject outlier points using the adaptive search radius
            good_matches: ndarray = distances < search_radius
            if sum(good_matches) < 10:
                break  # Lost tracking

            # SVD-Based Alignment (Arun et al., 1987 / Umeyama, 1991)
            P: ndarray = global_pts[good_matches]
            Q: ndarray = self.map_data['points'][indices[good_matches]]

            # Compute Centroids
            centroid_P: ndarray = mean(P, axis=0)
            centroid_Q: ndarray = mean(Q, axis=0)

            # Center the points around the origin
            P_centered: ndarray = P - centroid_P
            Q_centered: ndarray = Q - centroid_Q

            # Compute Cross-Covariance Matrix (H)
            H: ndarray = P_centered.T @ Q_centered

            # Singular Value Decomposition
            U, _, Vt = svd(H)

            # Optimal Rotation (R_delta)
            R_delta: ndarray = Vt.T @ U.T
            # Ensure it is a pure rotation, not a reflection
            if det(R_delta) < 0:
                Vt[-1, :] *= -1
                R_delta = Vt.T @ U.T

            # Optimal Translation (t_delta)
            t_delta: ndarray = centroid_Q - R_delta @ centroid_P

            # Apply the delta transformation to current pose guess
            # Apply the delta transformation to current pose guess
            current_pose[:2] = R_delta @ current_pose[:2] + t_delta
            current_pose[2] += arctan2(R_delta[1, 0], R_delta[0, 0])

            # Check for convergence based on Mean Squared Error
            mean_error: float = mean(distances[good_matches])
            if abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        # Format final pose
        current_pose = wrap_angle(current_pose)

        # Consider the match "valid" if the final average distance to walls is less than 20cm
        is_valid: bool = self.inlier_fraction_check(current_pose=current_pose, local_pts=local_pts)

        return current_pose, is_valid, prev_error

    def inlier_fraction_check(
        self,
        current_pose: ndarray,
        local_pts: ndarray,
        distance_threshold: float = 0.15,
        valid_threshold: float = 0.8,
    ) -> bool:
        # Project scan points to the global map frame using the pose
        final_theta: float = current_pose[2]
        R_final: ndarray = array(
            [[cos(final_theta), -sin(final_theta)], [sin(final_theta), cos(final_theta)]]
        )
        final_global_pts: ndarray = (R_final @ local_pts.T).T + current_pose[:2]

        # Query distances to the nearest map points.
        final_distances, _ = self.map_data['kdtree'].query(final_global_pts)

        # Calculate the fraction of points within distance_threshold
        inliers = sum(final_distances <= distance_threshold)
        inlier_fraction = float(inliers) / len(final_global_pts)

        # Consider the match "valid" if within valid_threshold
        return inlier_fraction >= valid_threshold

    def inlier_fraction_check(
        self,
        current_pose: ndarray,
        local_pts: ndarray,
        distance_threshold: float = 0.15,
        valid_threshold: float = 0.8,
    ) -> bool:
        # Project scan points to the global map frame using the pose
        final_theta: float = current_pose[2]
        R_final: ndarray = array(
            [[cos(final_theta), -sin(final_theta)], [sin(final_theta), cos(final_theta)]]
        )
        final_global_pts: ndarray = (R_final @ local_pts.T).T + current_pose[:2]

        # Query distances to the nearest map points.
        final_distances, _ = self.map_data['kdtree'].query(final_global_pts)

        # Calculate the fraction of points within distance_threshold
        inliers = sum(final_distances <= distance_threshold)
        inlier_fraction = float(inliers) / len(final_global_pts)

        # Consider the match "valid" if within valid_threshold
        return inlier_fraction >= valid_threshold

    def scan_callback(self, msg: LaserScan):
        """The main loop of the EKF."""
        start_time: float = perf_counter()

        if self.map_data is None:
            self.localization_runtime += perf_counter() - start_time
            return  # Wait for map to load

        # Get exact timestamp of the laser scan
        stamp = msg.header.stamp
        t_scan = stamp.sec + stamp.nanosec * 1e-9

        # Initialize time tracker on the very first scan
        if self.last_time is None:
            self.last_time = t_scan
            self.localization_runtime += perf_counter() - start_time
            return

        # Process all buffered odom messages stamped AT or BEFORE the scan time
        while self.odom_buffer and self.odom_buffer[0][0] <= t_scan:
            t_odom, v, omega = self.odom_buffer.pop(0)

            dt = t_odom - self.last_time
            if dt > 0:
                self.x_est, self.covariance = self._predict(v, omega, dt)
                self.last_time = t_odom

            # Keep track of the latest velocities for the partial step
            self.current_v = v
            self.current_omega = omega

        # Add one final partial step exactly up to the scan timestamp
        dt_partial = t_scan - self.last_time
        if dt_partial > 0:
            self.x_est, self.covariance = self._predict(
                self.current_v, self.current_omega, dt_partial
            )
            self.last_time = t_scan
        # Process all buffered odom messages stamped AT or BEFORE the scan time
        while self.odom_buffer and self.odom_buffer[0][0] <= t_scan:
            t_odom, v, omega = self.odom_buffer.pop(0)

            dt = t_odom - self.last_time
            if dt > 0:
                self.x_est, self.covariance = self._predict(v, omega, dt)
                self.last_time = t_odom

            # Keep track of the latest velocities for the partial step
            self.current_v = v
            self.current_omega = omega

        # Add one final partial step exactly up to the scan timestamp
        dt_partial = t_scan - self.last_time
        if dt_partial > 0:
            self.x_est, self.covariance = self._predict(
                self.current_v, self.current_omega, dt_partial
            )
            self.last_time = t_scan

        # update step (Using LaserScan ICP)
        y_pose, is_valid, match_error = self.scan_match(msg)

        if is_valid:
            self.x_est, self.covariance, accepted = self._update(y_pose, match_error)

            if not accepted:
                self.get_logger().debug('ICP match rejected by Mahalanobis check.')

        # Publish state
        self.publish_pose_and_tf_runtime(
            stamp=msg.header.stamp,
            cov_3x3=self.covariance,
            start_time=0.0,
            end_time=self.localization_runtime,
        )

        # Reset runtime for the next cycle
        self.localization_runtime = 0.0

    def _predict(self, v: float, omega: float, dt: float) -> Tuple[ndarray, ndarray]:
        """Prediction step. Apply Kinematic model"""
        new_x_est: ndarray = self.x_est
        new_cov: ndarray = self.covariance

        # Package the inputs for the motion model
        s: float = new_x_est[3]
        u: ndarray = array([v, s * omega])

        # Calculate the Jacobian using the previous step.
        A: ndarray = eye(4)
        A[:3, :3] = UnicycleMotionModel.linearize(state=new_x_est[:3], u=u, dt=dt)
        A[2, 3] = omega * dt

        # Proportional Control Noise Covariance (M)
        sigma_v: float = (
            self.alpha_tv_causedby_tv * abs(v) + self.alpha_tv_causedby_rv * abs(omega) + 0.001
        )
        sigma_w: float = (
            self.alpha_rv_causedby_tv * abs(v) + self.alpha_rv_causedby_rv * abs(omega) + 0.001
        )
        M: ndarray = diag([sigma_v**2, sigma_w**2])

        # Mapped Process Noise (R)
        theta: float = new_x_est[2]
        G = array([[cos(theta), 0.0], [sin(theta), 0.0], [0.0, 1.0], [0.0, 0.0]])
        R: ndarray = G @ M @ G.T * dt
        R[3, 3] += self.q_yaw_scale**2 * dt

        # Predict: Apply model forward kinematics
        new_x_est[:3] = UnicycleMotionModel.forward(
            state=new_x_est[:3], u=u, noise=zeros(3), dt=dt
        )

        # Update the covariance
        new_cov: ndarray = A @ new_cov @ A.T + R
        new_x_est: ndarray = wrap_angle(new_x_est)

        return new_x_est, new_cov

    def _update(self, y_pose: ndarray, match_error: float) -> Tuple[ndarray, ndarray, bool]:
        new_x_est: ndarray = self.x_est
        new_cov: ndarray = self.covariance

        error_scale: float = max(1.0, (match_error / 0.05) ** 2)
        W_dynamic: ndarray = self.W_pose * error_scale

        # Observation matrix (H is identity because we observe the direct [x, y, theta, s] pose from ICP scan matching)
        H: ndarray = eye(3, 4)

        # Innovation covariance (S)
        S: ndarray = H @ new_cov @ H.T + W_dynamic

        # Innovation (y_tilde) is the difference between observation and prediction
        y_tilde: ndarray = wrap_angle(y_pose - new_x_est[:3])

        # Statistical Acceptance Test
        mahalanobis_sq = y_tilde.T @ inv(S) @ y_tilde

        if mahalanobis_sq > CHI2_TRESHOLD_3DOF_95CONF:
            # The observation is statistically absurd given current belief.
            # Reject the update and keep prior estimate.
            return new_x_est, new_cov, False

        # Kalman Gain (K)
        K: ndarray = new_cov @ H.T @ inv(S)

        # Update the state estimate and shrink the covariance
        new_x_est: ndarray = new_x_est + K @ y_tilde
        new_x_est[3] = clip(new_x_est[3], 0.5, 1.2)
        new_cov: ndarray = (eye(4) - K @ H) @ new_cov

        # Wrap heading to [-pi, pi] to prevent runaway angle values over time
        new_x_est = wrap_angle(new_x_est)

        return new_x_est, new_cov, True


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
