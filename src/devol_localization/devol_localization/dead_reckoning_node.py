from typing import Tuple, Optional
import rclpy
import numpy as np
from nav_msgs.msg import Odometry
from numpy import diag, deg2rad, array, ndarray, zeros, pi, sin, cos

from devol_localization.base_localization_node import BaseLocalizationNode
from devol_localization.motion_models import UnicycleMotionModel
from devol_localization.utils import wrap_angle


class DeadReckoningNode(BaseLocalizationNode):
    def __init__(self):
        super().__init__('dead_reckoning_node', pose_topic='/devol_drive/dr_pose')

        # Covariances
        self.covariance: ndarray = diag([0.1**2, 0.1**2, deg2rad(5.0) ** 2])
        # Proportional Control Noise Parameters
        self.alpha_tv_causedby_tv: float = 0.05
        self.alpha_tv_causedby_rv: float = 0.01
        self.alpha_rv_causedby_tv: float = 0.01
        self.alpha_rv_causedby_rv: float = 0.05

        self.get_logger().info('Dead Reckoning Node Started!')

    def odom_callback(self, msg: Odometry):
        """Overrides base class implementation"""
        self.v: float = msg.twist.twist.linear.x
        self.omega: float = msg.twist.twist.angular.z
        self.u: ndarray = array([self.v, self.omega])

        dt: Optional[float] = self.get_dt_from_header(msg.header)
        if dt is None:
            return

        self.x_est, self.covariance = self._predict(dt=dt)

        # Use Base class to publish Pose and TF
        self.publish_pose_and_tf(msg.header.stamp, self.covariance)

    def _predict(self, dt: float) -> Tuple[ndarray, ndarray]:
        new_x_est: ndarray = self.x_est
        new_cov: ndarray = self.covariance

        # Get the Jacobian matrix before moving the state
        A: ndarray = UnicycleMotionModel.linearize(state=new_x_est, 
                                                   u=self.u, dt=dt)

        # Proportional Control Noise Covariance (M)
        sigma_v: float = self.alpha_tv_causedby_tv * abs(self.v) + self.alpha_tv_causedby_rv * abs(self.omega)
        sigma_w: float = self.alpha_rv_causedby_tv * abs(self.v) + self.alpha_rv_causedby_rv * abs(self.omega)
        M: ndarray = diag([sigma_v**2, sigma_w**2])

        # Control Jacobian (V_u): Maps the [v, omega] noise into [x, y, theta] space
        theta: float = new_x_est[2]
        V_u: ndarray = array([
            [cos(theta) * dt, 0.0],
            [sin(theta) * dt, 0.0],
            [0.0, dt]
        ])

        # Mapped Process Noise (R)
        R: ndarray = V_u @ M @ V_u.T

        # Predict: Apply model forward kinematics
        new_x_est: ndarray = UnicycleMotionModel.forward(
            state=new_x_est, u=self.u, noise=zeros(3), dt=dt
        )

        # Update the covariance
        new_cov: ndarray = A @ new_cov @ A.T + R
        new_x_est = wrap_angle(new_x_est)

        return new_x_est, new_cov


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DeadReckoningNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
