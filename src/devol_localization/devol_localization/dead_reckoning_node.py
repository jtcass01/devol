import rclpy
import numpy as np
from nav_msgs.msg import Odometry
from numpy import diag, deg2rad, array, ndarray, zeros

from devol_localization.base_localization_node import BaseLocalizationNode
from devol_localization.motion_models import UnicycleMotionModel


class DeadReckoningNode(BaseLocalizationNode):
    def __init__(self):
        super().__init__('dead_reckoning_node', pose_topic='/devol_drive/dr_pose')

        self.covariance = np.diag([0.1**2, 0.1**2, np.deg2rad(5.0) ** 2])
        self.V = np.diag([0.05**2, 0.05**2, np.deg2rad(2.0) ** 2])  # Process noise

        self.last_time = None

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.get_logger().info('Dead Reckoning Node Started!')

    def odom_callback(self, msg: Odometry):
        """Overrides base class implementation"""
        current_time: float = self.get_clock().now().nanoseconds / 1e9
        if self.last_time is None:
            self.last_time = current_time
            return

        dt: float = current_time - self.last_time
        self.last_time: float = current_time

        # Extract velocities (twist) from the Odometry message
        v: float = msg.twist.twist.linear.x
        omega: float = msg.twist.twist.angular.z
        u: ndarray = array([v, omega])

        # Get the Jacobian matrix before moving the state
        A: ndarray = UnicycleMotionModel.linearize(state=self.x_est, u=u, dt=dt)

        # Predict: Apply model forward kinematics
        self.x_est: ndarray = UnicycleMotionModel.forward(
            state=self.x_est, u=u, noise=zeros(3), dt=dt
        )
        # Wrap heading to [-pi, pi]
        self.x_est[2] = (self.x_est[2] + np.pi) % (2 * np.pi) - np.pi

        # Update the covariance matrix
        self.covariance = A @ self.covariance @ A.T + self.V

        # Use Base class to publish Pose and TF
        self.publish_pose_and_tf(self.get_clock().now().to_msg(), self.covariance)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DeadReckoningNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
