from typing import Optional
from scipy.spatial import KDTree
from numpy import (ndarray, array, zeros, diag, deg2rad, mean, 
                   sum, sin, cos, vstack, where, int8 as np_int8, 
                   eye, pi, arctan2)
from numpy.linalg import svd, det, inv

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ROS 2 Messages
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

# Import devol localization classes
from devol_localization.base_localization_node import BaseLocalizationNode
from devol_localization.motion_models import UnicycleMotionModel

class EKFLocalizationNode(BaseLocalizationNode):
    def __init__(self):
        super().__init__('ekf_localization_node', '/devol_drive/ekf_pose')

        self.map_data = None

        # EKF Covariances
        self.covariance: ndarray = diag([0.1**2, 0.1**2, deg2rad(5.0) ** 2])
        self.V: ndarray = diag([0.05**2, 0.05**2, deg2rad(2.0) ** 2])
        self.W_pose: ndarray = diag([0.1**2, 0.1**2, deg2rad(3.0) ** 2])

        # Subscribers / Publishers
        map_qos: QoSProfile = QoSProfile(
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

        grid: ndarray = array(msg.data, dtype=np_int8).reshape((msg.info.height, msg.info.width))

        # Find the row/col indices of all cells that are likely obstacles
        obstacle_y, obstacle_x = where(grid > 50)

        # Convert indices to real-world metric coordinates
        map_pts_x: ndarray = msg.info.origin.position.x + obstacle_x * msg.info.resolution
        map_pts_y: ndarray = msg.info.origin.position.y + obstacle_y * msg.info.resolution

        # Stack into an Nx2 array of obstacle points
        map_points: ndarray = vstack((map_pts_x, map_pts_y)).T

        # Build the KDTree for the ICP algorithm
        self.map_data: dict = {'points': map_points, 'kdtree': KDTree(map_points)}
        self.get_logger().info(
            f'Map received. Built KDTree with {len(map_points)} obstacle points.'
        )

    def scan_match(self, msg: LaserScan):
        """2D Iterative Closest Point using SVD."""
        # Filter out invalid rays
        ranges, angles = self.preprocess_scan(msg=msg)

        if len(ranges) < 10:
            return array([0.0, 0.0, 0.0]), False  # Not enough valid points

        # Nx2 array of local laser points
        laser_pts: ndarray = vstack((ranges * cos(angles), 
                                     ranges * sin(angles))).T
        laser_offset: ndarray = self.get_laser_offset()

        R_offset: ndarray = array(
            [
                [cos(laser_offset[2]), -sin(laser_offset[2])],
                [sin(laser_offset[2]), cos(laser_offset[2])],
            ]
        )
        local_pts: ndarray = (R_offset @ laser_pts.T).T + laser_offset[:2]

        current_pose: ndarray = self.x_est.copy()
        prev_error: float = float('inf')

        max_iter = 50
        tolerance = 1e-4

        for _ in range(max_iter):
            # Transform local scan points into the global map frame
            theta: float = current_pose[2]
            R_curr: ndarray = array([[cos(theta), -sin(theta)], 
                                     [sin(theta), cos(theta)]])
            global_pts: ndarray = (R_curr @ local_pts.T).T + current_pose[:2]

            # Find the closest Map points (Q) for every Global Scan point (P)
            distances, indices = self.map_data['kdtree'].query(global_pts)

            # Reject outlier points that are too far from any wall
            good_matches: ndarray = distances < 1.0
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

            # Apply the delta transformation to our current pose guess
            current_pose[:2] = R_delta @ current_pose[:2] + t_delta
            current_pose[2] += arctan2(R_delta[1, 0], R_delta[0, 0])

            # Check for convergence based on Mean Squared Error
            mean_error: float = mean(distances[good_matches])
            if abs(prev_error - mean_error) < tolerance:
                break
            prev_error = mean_error

        # Format final pose
        current_pose[2] = (current_pose[2] + pi) % (2 * pi) - pi

        # Consider the match "valid" if the final average distance to walls is less than 20cm
        is_valid: bool = prev_error < 0.2

        return current_pose, is_valid, prev_error

    def scan_callback(self, msg: LaserScan):
        """The main loop of the EKF."""
        if self.map_data is None:
            return  # Wait for map to load

        dt: Optional[float] = self.get_dt_from_header(msg.header)
        if dt is None:
            return

        # predict step (Using Odometry Twist)
        u: ndarray = array([self.v, self.omega])
        ## Calculate the Jacobian using the previous step.
        A: ndarray = UnicycleMotionModel.linearize(state=self.x_est, 
                                                   u=u, dt=dt)
        ## Forward project th emean state
        self.x_est: ndarray = UnicycleMotionModel.forward(state=self.x_est, u=u, 
                                                          noise=zeros(3), dt=dt)
        ## Update the covariance
        self.covariance: ndarray = A @ self.covariance @ A.T + self.V
        
        # update step (Using LaserScan ICP)
        y_pose, is_valid, match_error = self.scan_match(msg)

        if is_valid:
            error_scale: float = max(1.0, (match_error / 0.05) ** 2)
            W_dynamic: ndarray = self.W_pose * error_scale

            # Observation matrix (H is identity because we observe the direct [x, y, theta] pose from ICP scan matching)
            H: ndarray = eye(3)

            # Innovation covariance (S)
            S: ndarray = H @ self.covariance @ H.T + W_dynamic

            # Kalman Gain (K)
            K: ndarray = self.covariance @ H.T @ inv(S)

            # Innovation (y_tilde) is the difference between observation and prediction
            y_tilde: ndarray = y_pose - self.x_est
            y_tilde[2] = (y_tilde[2] + pi) % (2 * pi) - pi  # Wrap angle error

            # Update the state estimate and shrink the covariance
            self.x_est: ndarray = self.x_est + K @ y_tilde
            self.covariance: ndarray = (eye(3) - K @ H) @ self.covariance

        # Wrap heading to [-pi, pi] to prevent runaway angle values over time
        self.x_est[2] = (self.x_est[2] + pi) % (2 * pi) - pi

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