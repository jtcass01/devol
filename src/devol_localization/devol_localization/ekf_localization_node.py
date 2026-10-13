import numpy as np
from scipy.spatial import KDTree

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ROS 2 Messages
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

# Import devol localization classes
from devol_localization.base_localization_node import BaseLocalizationNode
from devol_localization.ekf import EKF


class EKFLocalizationNode(BaseLocalizationNode):
    def __init__(self):
        super().__init__('ekf_localization_node', '/devol_drive/ekf_pose')

        self.map_data = None

        # EKF Covariances
        self.covariance = np.diag([0.1**2, 0.1**2, np.deg2rad(5.0) ** 2])
        self.V = np.diag([0.05**2, 0.05**2, np.deg2rad(2.0) ** 2])
        self.W_pose = np.diag([0.1**2, 0.1**2, np.deg2rad(3.0) ** 2])

        # Subscribers / Publishers
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.get_logger().info('EKF Localization Node Started!')

    def map_callback(self, msg: OccupancyGrid):
        """Extracts obstacle coordinates and builds KDTree."""
        if not self.is_map_new(msg):
            return

        grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

        # Find the row/col indices of all cells that are likely obstacles (> 50% probability)
        obstacle_y, obstacle_x = np.where(grid > 50)

        # Convert indices to real-world metric coordinates
        map_pts_x = msg.info.origin.position.x + obstacle_x * msg.info.resolution
        map_pts_y = msg.info.origin.position.y + obstacle_y * msg.info.resolution

        # Stack into an Nx2 array of obstacle points
        map_points = np.vstack((map_pts_x, map_pts_y)).T

        # Build the KDTree for the ICP algorithm
        self.map_data = {'points': map_points, 'kdtree': KDTree(map_points)}
        self.get_logger().info(
            f'Map received. Built KDTree with {len(map_points)} obstacle points.'
        )

    def scan_match(self, msg: LaserScan):
        """2D Iterative Closest Point using SVD."""
        # Convert LaserScan to 2D Cartesian points in the robot's local frame
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        # Filter out infinite/bad rays
        valid = (
            (ranges > msg.range_min)
            & (ranges < msg.range_max)
            & ~np.isinf(ranges)
            & ~np.isnan(ranges)
        )
        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) < 10:
            return np.array([0.0, 0.0, 0.0]), False  # Not enough valid points

        # Nx2 array of local laser points
        laser_pts = np.vstack((ranges * np.cos(angles), ranges * np.sin(angles))).T
        laser_offset = self.get_laser_offset()

        R_offset = np.array(
            [
                [np.cos(laser_offset[2]), -np.sin(laser_offset[2])],
                [np.sin(laser_offset[2]), np.cos(laser_offset[2])],
            ]
        )
        local_pts = (R_offset @ laser_pts.T).T + laser_offset[:2]

        current_pose = self.x_est.copy()
        prev_error = float('inf')

        max_iter = 50
        tolerance = 1e-4

        for _ in range(max_iter):
            # Transform local scan points into the global map frame
            theta = current_pose[2]
            R_curr = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
            global_pts = (R_curr @ local_pts.T).T + current_pose[:2]

            # Find the closest Map points (Q) for every Global Scan point (P)
            distances, indices = self.map_data['kdtree'].query(global_pts)

            # Reject outlier points that are too far from any wall
            good_matches = distances < 1.0
            if np.sum(good_matches) < 10:
                break  # Lost tracking

            # SVD-Based Alignment (Arun et al., 1987 / Umeyama, 1991)
            P = global_pts[good_matches]
            Q = self.map_data['points'][indices[good_matches]]

            # Compute Centroids
            centroid_P = np.mean(P, axis=0)
            centroid_Q = np.mean(Q, axis=0)

            # Center the points around the origin
            P_centered = P - centroid_P
            Q_centered = Q - centroid_Q

            # Compute Cross-Covariance Matrix (H)
            H = P_centered.T @ Q_centered

            # Singular Value Decomposition
            U, _, Vt = np.linalg.svd(H)

            # Optimal Rotation (R_delta)
            R_delta = Vt.T @ U.T
            # Ensure it is a pure rotation, not a reflection
            if np.linalg.det(R_delta) < 0:
                Vt[-1, :] *= -1
                R_delta = Vt.T @ U.T

            # Optimal Translation (t_delta)
            t_delta = centroid_Q - R_delta @ centroid_P

            # Apply the delta transformation to our current pose guess
            current_pose[:2] = R_delta @ current_pose[:2] + t_delta
            current_pose[2] += np.arctan2(R_delta[1, 0], R_delta[0, 0])

            # Check for convergence based on Mean Squared Error
            mean_error = np.mean(distances[good_matches])
            if abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        # Format final pose
        current_pose[2] = (current_pose[2] + np.pi) % (2 * np.pi) - np.pi

        # Consider the match "valid" if the final average distance to walls is less than 20cm
        is_valid = prev_error < 0.2

        return current_pose, is_valid, prev_error

    def scan_callback(self, msg: LaserScan):
        """The main loop of the EKF."""
        if self.map_data is None:
            return  # Wait for map to load

        dt = self.get_dt_from_header(msg.header)
        if dt is None:
            return

        # Scan Matching
        y_pose, is_valid, match_error = self.scan_match(msg)

        # Run the Extended Kalman Filter!
        if is_valid:
            error_scale = max(1.0, (match_error / 0.05) ** 2)
            W_dynamic = self.W_pose * error_scale

            self.x_est, self.covariance = EKF(
                DiffDrive=self.diff_drive,
                y_pose=y_pose,
                x_hat=self.x_est,
                sigma=self.covariance,
                u=self.u,
                dt=dt,
                V=self.V,
                W_pose=W_dynamic,
            )
        else:
            # No valid scan match. Just do the Prediction step manually using odometry.
            self.x_est = self.diff_drive.forward(self.x_est, self.u, np.zeros(3), dt)
            A, _ = self.diff_drive.linearize(self.x_est, self.u, dt)
            self.covariance = A @ self.covariance @ A.T + self.V

        # Wrap heading to [-pi, pi] to prevent runaway angle values over time
        self.x_est[2] = (self.x_est[2] + np.pi) % (2 * np.pi) - np.pi

        # Publish state
        self.publish_pose_and_tf(msg.header.stamp, self.covariance)


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
