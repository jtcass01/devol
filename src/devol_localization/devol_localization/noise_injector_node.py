#!/usr/bin/env python3

from math import sqrt

from nav_msgs.msg import Odometry
from numpy.random import default_rng
from rclpy import init, spin, try_shutdown
from rclpy.node import Node


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


class NoiseInjectorNode(Node):
    def __init__(self):
        super().__init__('noise_injector_node')

        # Declare Parameters
        self.declare_parameter('odom_in', '/devol_drive/odom')
        self.declare_parameter('odom_out', '/devol_drive/noisy/odom')
        self.declare_parameter('seed', '31')
        self.declare_parameter('k', 1.0)

        # Proportional Noise Parameters
        self.alpha_tv_causedby_tv: float = 0.10
        self.alpha_tv_causedby_rv: float = 0.01
        self.alpha_rv_causedby_tv: float = 0.01
        self.alpha_rv_causedby_rv: float = 0.20

        # Retrieve Parameters
        self.noise_scale: float = float(self.get_parameter('k').value)
        self.seed: int = int(self.get_parameter('seed').value)
        self.last_stamp = None

        # Initialize random number generator
        self.rng = default_rng(self.seed)

        self.odom_pub = self.create_publisher(Odometry, self.get_parameter('odom_out').value, 50)
        self.create_subscription(Odometry, self.get_parameter('odom_in').value, self.odom_callback, 50)

    def odom_callback(self, msg: Odometry) -> None:
        v: float = msg.twist.twist.linear.x
        omega: float = msg.twist.twist.angular.z
        v_hat, omega_hat = self.sample(v=v, omega=omega)
        msg.twist.twist.linear.x = v_hat
        msg.twist.twist.angular.z = omega_hat

        self.odom_pub.publish(msg)

    def sample(self, v: float, omega: float) -> tuple:
        variance_v: float = self.alpha_tv_causedby_tv * v**2 + self.alpha_tv_causedby_rv * omega**2
        variance_omega: float = self.alpha_rv_causedby_tv * v**2 + self.alpha_rv_causedby_rv * omega**2

        v_hat: float = v + self.rng.normal(0.0, self.noise_scale * sqrt(variance_v))
        omega_hat: float = omega + self.rng.normal(0.0, self.noise_scale * sqrt(variance_omega))

        return v_hat, omega_hat


def main(args=None):
    init(args=args)
    node = NoiseInjectorNode()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try_shutdown()


if __name__ == '__main__':
    main()
