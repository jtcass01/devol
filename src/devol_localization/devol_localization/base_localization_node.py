#!/usr/bin/env python3

from zlib import crc32
from numpy import ndarray, array, linspace, isinf, isnan, diag, deg2rad, zeros

from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from sensor_msgs.msg import LaserScan

from devol_localization.utils import euler_to_quaternion


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


class BaseLocalizationNode(Node):
    def __init__(self, node_name, pose_topic: str = '/devol_drive/pose',
                 runtime_topic: str = '/devol_drive/pose/runtime'):
        super().__init__(node_name)

        # Declare Parameters
        self.declare_parameter('odom_topic', '/devol_drive/odom')
        self.declare_parameter('map_topic', '/devol_drive/projected_map')
        self.declare_parameter('scan_topic', '/devol_drive/sensors/lidar2d_0/scan')
        self.declare_parameter('pose_topic', pose_topic)
        self.declare_parameter('runtime_topic', runtime_topic)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'a200_base_link')

        # Retrieve Parameters
        self.odom_topic: str = self.get_parameter('odom_topic').value
        self.map_topic: str = self.get_parameter('map_topic').value
        self.scan_topic: str = self.get_parameter('scan_topic').value
        self.pose_topic: str = self.get_parameter('pose_topic').value
        self.runtime_topic: str = self.get_parameter('runtime_topic').value
        self.map_frame: str = self.get_parameter('map_frame').value
        self.odom_frame: str = self.get_parameter('odom_frame').value
        self.base_frame: str = self.get_parameter('base_frame').value

        # Shared State and Models
        self.v: float = 0.0
        self.omega: float = 0.0
        self.x_est: ndarray = array([0.0, 0.0, 0.0])

        self.latest_odom_pose = array([0.0, 0.0, 0.0])
        self.last_map_crc = None
        self.last_scan_time = None

        # TF Handling
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Shared Publishers
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)

        # Start Shared Subscribers (Map and Scan are implemented by child classes)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        """Base callback to track the latest odometry twist."""
        self.v: float = msg.twist.twist.linear.x
        self.omega: float = msg.twist.twist.angular.z
        self.u: ndarray = array([self.v, self.omega])

    def is_map_new(self, msg: OccupancyGrid) -> bool:
        """Computes a CRC32 hash of the map data.
        Returns True if the map is new/changed, False if it is identical."""
        # Convert the map data array to continuous bytes and hash it
        current_crc = crc32(bytes(msg.data))

        if self.last_map_crc == current_crc:
            return False

        self.last_map_crc = current_crc
        return True

    def get_dt_from_header(self, header):
        """Calculates the time delta (dt) from a message header.
        Updates self.last_scan_time.
        Returns dt (float) if valid, or None if the loop should be skipped."""
        msg_time = header.stamp.sec + (header.stamp.nanosec / 1e9)

        if self.last_scan_time is None:
            self.last_scan_time = msg_time
            return None

        dt = msg_time - self.last_scan_time

        # Prevent massive dt jumps if processing fell way behind
        if dt > 1.0:
            self.get_logger().warn(f'Large dt detected ({dt:.2f}s). Resetting.')
            self.last_scan_time = msg_time
            return None

        self.last_scan_time = msg_time
        return dt

    def preprocess_scan(self, msg: LaserScan):
        """Standardizes LiDAR extraction for all algorithms."""
        angles: ndarray = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges: ndarray = array(msg.ranges)

        # Filter out infinite/bad rays
        eps = 0.02
        valid = (
            (ranges > msg.range_min)
            & (ranges < (msg.range_max - eps))
            & ~isinf(ranges)
            & ~isnan(ranges)
        )
        return ranges[valid], angles[valid]

    def get_laser_offset(self):
        """Looks up the physical LiDAR mount offset."""
        return array([0.32825, 0.0, 0.0])

    def publish_pose_and_tf_runtime(self, stamp, cov_3x3, start_time: float, end_time: float):
        """Standardized publishing of Pose + Covariance and REP-105 TF."""
        # Publish Pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame

        pose_msg.pose.pose.position.x = self.x_est[0]
        pose_msg.pose.pose.position.y = self.x_est[1]
        pose_msg.pose.pose.orientation = euler_to_quaternion(self.x_est[2])

        # Pack 3x3 covariance into ROS 6x6 format
        cov_6x6 = zeros((6, 6))
        cov_6x6[0, 0], cov_6x6[0, 1] = cov_3x3[0, 0], cov_3x3[0, 1]
        cov_6x6[1, 0], cov_6x6[1, 1] = cov_3x3[1, 0], cov_3x3[1, 1]
        cov_6x6[0, 5], cov_6x6[5, 0] = cov_3x3[0, 2], cov_3x3[2, 0]
        cov_6x6[1, 5], cov_6x6[5, 1] = cov_3x3[1, 2], cov_3x3[2, 1]
        cov_6x6[5, 5] = cov_3x3[2, 2]

        pose_msg.pose.covariance = tuple(cov_6x6.flatten())
        self.pub_pose.publish(pose_msg)

        # Publish the runtime as a PointStamped message
        runtime_msg = PointStamped()
        runtime_msg.header.stamp = stamp
        runtime_msg.frame_id = self.map_frame
        runtime_msg.point.x = end_time - start_time
        self.runtime_pub.publish(runtime_msg)