#!/usr/bin/env python3

from typing import Tuple, Optional
from time import perf_counter
from numpy import diag, array, ndarray, sin, cos, zeros, deg2rad, eye

import rclpy
from nav_msgs.msg import Odometry

from devol_localization.base_localization_node import BaseLocalizationNode
from devol_localization.motion_models import UnicycleMotionModel
from devol_localization.utils import wrap_angle


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


class DeadReckoningNode(BaseLocalizationNode):
    def __init__(self):
        super().__init__(
            'dead_reckoning_node',
            pose_topic='/devol_drive/dr_pose',
            runtime_topic='/devol_drive/dr_pose/runtime',
        )

        # State variables.
        self.x_est: ndarray = array(
            [0.0, 0.0, 0.0]
        )  # [x, y, theta]
        self.covariance: ndarray = diag([0.1**2, 0.1**2, deg2rad(5.0) ** 2])
        self.q_yaw_scale: float = 0.02
        self.localization_runtime = 0.0

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.get_logger().info('Dead Reckoning Node Started!')

    def odom_callback(self, msg: Odometry):
        """Overrides base class implementation"""
        start_time: float = perf_counter()

        self.v: float = msg.twist.twist.linear.x
        self.omega: float = msg.twist.twist.angular.z

        dt: Optional[float] = self.get_dt_from_header(msg.header)
        if dt is None:
            return

        self.x_est, self.covariance = self._predict(dt=dt)
        self.localization_runtime += perf_counter() - start_time

        # Publish Pose, TF, and Runtime using the base class
        self.publish_pose_and_tf_runtime(
            stamp=msg.header.stamp,
            cov_3x3=self.covariance,
            start_time=0.0,
            end_time=self.localization_runtime,
        )

        # Reset runtime for the next cycle
        self.localization_runtime = 0.0

    def _predict(self, dt: float) -> Tuple[ndarray, ndarray]:
        """Prediction step. Apply Kinematic model"""
        new_x_est: ndarray = self.x_est
        new_cov: ndarray = self.covariance

        # Package the inputs for the motion model
        u: ndarray = array([self.v, self.omega])

        # Calculate the Jacobian using the previous step.
        A = UnicycleMotionModel.linearize(state=new_x_est, u=u, dt=dt)

        # Proportional Control Noise Covariance (M)
        sigma_v: float = (
            self.alpha_tv_causedby_tv * abs(self.v)
            + self.alpha_tv_causedby_rv * abs(self.omega)
            + 0.001
        )
        sigma_w: float = (
            self.alpha_rv_causedby_tv * abs(self.v)
            + self.alpha_rv_causedby_rv * abs(self.omega)
            + 0.001
        )
        M: ndarray = diag([sigma_v**2, sigma_w**2])

        # Mapped Process Noise (R)
        theta: float = new_x_est[2]
        G = array([[cos(theta), 0.0], [sin(theta), 0.0], [0.0, 1.0]])
        R: ndarray = G @ M @ G.T * dt

        # Predict: Apply model forward kinematics
        new_x_est[:3] = UnicycleMotionModel.forward(
            state=new_x_est[:3], u=u, noise=zeros(3), dt=dt
        )
        new_x_est: ndarray = wrap_angle(new_x_est)

        # Update the covariance
        new_cov: ndarray = A @ new_cov @ A.T + R

        return new_x_est, new_cov


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DeadReckoningNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
