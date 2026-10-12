import rclpy
import numpy as np

from devol_localization.base_localization_node import BaseLocalizationNode

class DeadReckoningNode(BaseLocalizationNode):
    def __init__(self):
        super().__init__('dead_reckoning_node', pose_topic='/devol_drive/dr_pose')

        # EKF-style Covariances (Covariance will grow unboundedly over time!)
        self.covariance = np.diag([0.1**2, 0.1**2, np.deg2rad(5.0)**2])
        self.V = np.diag([0.05**2, 0.05**2, np.deg2rad(2.0)**2]) # Process noise

        self.last_time = None

        # Run the integration loop at 20 Hz
        self.update_timer = self.create_timer(0.05, self.update_callback)

        self.get_logger().info('Dead Reckoning Node Started!')

    def update_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_time is None:
            self.last_time = current_time
            return
        
        dt = current_time - self.last_time
        self.last_time = current_time

        # Predict
        # Apply forward kinematics
        self.x_est = self.diff_drive.forward(self.x_est, self.u, np.zeros(3), dt)
        
        # update the covariance matrix
        A, _ = self.diff_drive.linearize(self.x_est, self.u, dt)
        self.covariance = A @ self.covariance @ A.T + self.V

        # Wrap heading to [-pi, pi]
        self.x_est[2] = (self.x_est[2] + np.pi) % (2 * np.pi) - np.pi

        # Use Base class to publish Pose and TF
        self.publish_pose_and_tf(self.get_clock().now().to_msg(), self.covariance)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DeadReckoningNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()