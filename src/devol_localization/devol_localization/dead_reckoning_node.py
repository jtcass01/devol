#!/usr/bin/env python3

from typing import Tuple, Optional
from time import perf_counter
from numpy import diag, array, ndarray, sin, cos, zeros, deg2rad

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
        self.x_est: ndarray = array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.covariance: ndarray = diag([0.1**2, 0.1**2, deg2rad(5.0) ** 2])
        self.localization_runtime = 0.0

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.get_logger().info('Dead Reckoning Node Started!')

    def odom_callback(self, msg: Odometry):
        """Overrides base class implementation"""
        start_time: float = perf_counter()
        dt: Optional[float] = self.get_dt_from_header(msg.header)
        if dt is None:
            self.localization_runtime += perf_counter() - start_time
            return

        self.x_est, self.covariance = DeadReckoningNode._predict(
            prev_x=self.x_est,
            prev_cov=self.covariance,
            v=msg.twist.twist.linear.x,
            omega=msg.twist.twist.angular.z,
            dt=dt, 
            alpha_tv_causedby_tv=self.alpha_tv_causedby_tv,
            alpha_tv_causedby_rv=self.alpha_tv_causedby_rv,
            alpha_rv_causedby_tv=self.alpha_rv_causedby_tv,
            alpha_rv_causedby_rv=self.alpha_rv_causedby_rv)
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

    @staticmethod
    def mapped_process_noise(x: ndarray, M: ndarray, dt: float) -> ndarray:
        """R"""
        G: ndarray = UnicycleMotionModel.control_jacobian(x=x, dt=dt)
        return G @ M @ G.T * dt

    @staticmethod
    def _predict(prev_x: ndarray, prev_cov: ndarray, v: float, 
                 omega: float, dt: float, alpha_tv_causedby_tv: float, 
                 alpha_tv_causedby_rv: float, alpha_rv_causedby_tv: float, 
                 alpha_rv_causedby_rv: float) -> Tuple[ndarray, ndarray]:
        """Prediction step. Apply Kinematic model"""
        u: ndarray = array([v, omega])

        # Apply model forward kinematics
        x: ndarray = UnicycleMotionModel.forward(x=prev_x, u=u, noise=zeros(3), dt=dt)
        x: ndarray = wrap_angle(x)

        # Proportional Control Noise Covariance (M)
        M: ndarray = DeadReckoningNode.control_noise_covariance(
            u=u, alpha_tv_causedby_tv=alpha_tv_causedby_tv,
            alpha_tv_causedby_rv=alpha_tv_causedby_rv,
            alpha_rv_causedby_tv=alpha_rv_causedby_tv,
            alpha_rv_causedby_rv=alpha_rv_causedby_rv)

        # Mapped Process Noise (R)
        R: ndarray = DeadReckoningNode.mapped_process_noise(x=x, M=M, dt=dt)

        # Calculate the Jacobian using the previous step.
        A: ndarray = UnicycleMotionModel.state_jacobian(x=prev_x, u=u, dt=dt)

        # Update the covariance
        cov: ndarray = DeadReckoningNode.update_covariance(A=A, cov=prev_cov, R=R)

        return x, cov


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DeadReckoningNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
