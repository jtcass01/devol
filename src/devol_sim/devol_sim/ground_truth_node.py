from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node

# ROS 2 Messages
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from devol_localization.utils import (
    euler_to_quaternion,
    yaw_from_quaternion,
    compose,
    inverse,
    odom_to_planar,
)

__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


class GroundTruthNode(Node):
    def __init__(self) -> None:
        super().__init__('ground_truth_tf')
        self.declare_parameter('ground_truth_topic', '/devol_drive/ground_truth/odom')
        self.declare_parameter('odom_topic', '/devol_drive/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)

        self.gt: Optional[np.ndarray] = None
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter('ground_truth_topic').value, self.gt_callback, 50
        )
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 50
        )
        self.get_logger().info(
            f'Publishing {self.map_frame} -> {self.odom_frame} from ground truth'
        )

    def gt_callback(self, msg: Odometry) -> None:
        self.gt = odom_to_planar(msg)

    def odom_callback(self, msg: Odometry) -> None:
        if self.gt is None:
            self.get_logger().info('Waiting for ground truth')
            return
        gt_transform = compose(self.gt, inverse(odom_to_planar(msg)))

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame

        tf.transform.translation.x = float(gt_transform[0])
        tf.transform.translation.y = float(gt_transform[1])
        tf.transform.rotation = euler_to_quaternion(float(gt_transform[2]))
        self.broadcaster.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
