#!/usr/bin/env python3
"""Publishes map -> odom from Gazebo ground truth, so the planner and controller drive on the true pose.

The study requires the waypoint controller to drive on simulator ground truth, so estimator error
never changes the trajectory being scored. The controller looks up map -> base through
odom -> base (wheel odometry); this node supplies map -> odom = truth (+) odom^-1 on every odometry
message, which makes that lookup return the true pose. It replaces the static map -> odom of
spawn_entities.launch.py (start the sim with map_odom_tf:=none so the two do not fight).

Inputs
  ground_truth_topic (nav_msgs/Odometry) Gazebo OdometryPublisher, world (= map) frame
  odom_topic         (nav_msgs/Odometry) the raw wheel odometry that also drives odom -> base on /tf
"""

from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from devol_localization.pose2d import compose, inverse, set_quaternion_yaw, yaw_from_quaternion

__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


def planar_pose(msg: Odometry) -> np.ndarray:
    p = msg.pose.pose
    return np.array([p.position.x, p.position.y, yaw_from_quaternion(p.orientation)])


class GroundTruthTF(Node):
    def __init__(self) -> None:
        super().__init__('ground_truth_tf')
        self.declare_parameter('ground_truth_topic', '/devol_drive/ground_truth/odom')
        self.declare_parameter('odom_topic', '/devol_drive/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        gp = self.get_parameter
        self._map_frame = str(gp('map_frame').value)
        self._odom_frame = str(gp('odom_frame').value)
        self._truth: Optional[np.ndarray] = None
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, gp('ground_truth_topic').value, self._gt_cb, 50)
        self.create_subscription(Odometry, gp('odom_topic').value, self._odom_cb, 50)
        self.get_logger().info(
            f'Publishing {self._map_frame} -> {self._odom_frame} from ground truth'
        )

    def _gt_cb(self, msg: Odometry) -> None:
        self._truth = planar_pose(msg)

    def _odom_cb(self, msg: Odometry) -> None:
        if self._truth is None:
            self.get_logger().info('Waiting for ground truth', throttle_duration_sec=5.0)
            return
        m2o = compose(self._truth, inverse(planar_pose(msg)))
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self._map_frame
        tf.child_frame_id = self._odom_frame
        tf.transform.translation.x = float(m2o[0])
        tf.transform.translation.y = float(m2o[1])
        set_quaternion_yaw(tf.transform.rotation, float(m2o[2]))
        self._broadcaster.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
