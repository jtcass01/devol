#!/usr/bin/env python3
"""Injects the study's seeded odometry and lidar noise between the sensors and the estimators.

Inputs
  odom_in  (nav_msgs/Odometry)      wheel odometry from Gazebo or a bag
  scan_in  (sensor_msgs/LaserScan)  noise-free Gazebo lidar

Outputs
  odom_out (nav_msgs/Odometry)      odometry whose increments are perturbed with alpha1..4 = k * 0.05
  scan_out (sensor_msgs/LaserScan)  ranges with N(0, sigma_r^2) added

Parameters k, sigma_r and seed set the study configuration (see noise_model.py). With k = 0 and
sigma_r = 0 the node is a pass-through, so every estimator can always read the *_out topics.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from devol_localization.noise_model import OdometryNoiseInjector, add_range_noise
from devol_localization.pose2d import set_quaternion_yaw, yaw_from_quaternion

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


class NoiseInjectorNode(Node):
    def __init__(self) -> None:
        super().__init__('noise_injector')
        self.declare_parameter('odom_in', '/devol_drive/odom')
        self.declare_parameter('scan_in', '/devol_drive/sensors/lidar2d_0/scan')
        self.declare_parameter('odom_out', '/devol_drive/noisy/odom')
        self.declare_parameter('scan_out', '/devol_drive/noisy/scan')
        self.declare_parameter('k', 1.0)
        self.declare_parameter('alpha_nominal', 0.05)
        self.declare_parameter('sigma_r', 0.03)
        self.declare_parameter('seed', 0)
        self.declare_parameter('segment_length', 0.1)
        self.declare_parameter('segment_angle', 0.1)

        gp = self.get_parameter
        k = float(gp('k').value)
        self._sigma_r = float(gp('sigma_r').value)
        seed = int(gp('seed').value)
        odom_seq, scan_seq = np.random.SeedSequence(seed).spawn(2)
        self._odom_noise = OdometryNoiseInjector(k, seed=odom_seq, alpha_nominal=float(gp('alpha_nominal').value),
                                                 segment_length=float(gp('segment_length').value),
                                                 segment_angle=float(gp('segment_angle').value))
        self._scan_rng = np.random.default_rng(scan_seq)

        self._odom_pub = self.create_publisher(Odometry, gp('odom_out').value, 50)
        self._scan_pub = self.create_publisher(LaserScan, gp('scan_out').value, 10)
        self.create_subscription(Odometry, gp('odom_in').value, self._odom_cb, 50)
        self.create_subscription(LaserScan, gp('scan_in').value, self._scan_cb, qos_profile_sensor_data)
        self.get_logger().info(f'Noise injection: k={k:g} (alpha={k * float(gp("alpha_nominal").value):g}), '
                               f'sigma_r={self._sigma_r:g} m, seed={seed}')

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose
        noisy = self._odom_noise((p.position.x, p.position.y, yaw_from_quaternion(p.orientation)))
        p.position.x = float(noisy[0])
        p.position.y = float(noisy[1])
        set_quaternion_yaw(p.orientation, float(noisy[2]))
        self._odom_pub.publish(msg)

    def _scan_cb(self, msg: LaserScan) -> None:
        if self._sigma_r > 0.0:
            msg.ranges = add_range_noise(msg.ranges, self._sigma_r, msg.range_min, msg.range_max,
                                         self._scan_rng).astype(np.float32).tolist()
        self._scan_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NoiseInjectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
