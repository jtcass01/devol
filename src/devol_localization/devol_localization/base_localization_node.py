import numpy as np

from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

from devol_localization.differential_drive import DifferentialDrive
from devol_localization.utils import euler_to_quaternion, yaw_from_quaternion, compose, inverse


class BaseLocalizationNode(Node):
    def __init__(self, node_name, pose_topic: str = '/devol_drive/pose'):
        super().__init__(node_name)

        # Declare Parameters
        self.declare_parameter('odom_topic', '/devol_drive/odom')
        self.declare_parameter('map_topic', '/devol_drive/projected_map')
        self.declare_parameter('scan_topic', '/devol_drive/sensors/lidar2d_0/scan')
        self.declare_parameter('pose_topic', pose_topic)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'a200_base_link')

        # Retrieve Parameters
        self.odom_topic = self.get_parameter('odom_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Shared State and Models
        self.diff_drive = DifferentialDrive(L=0.555, R=0.165)
        self.u = np.array([0.0, 0.0])
        self.last_scan_time = None
        self.latest_odom_pose = np.array([0.0, 0.0, 0.0])
        self.x_est = np.array([0.0, 0.0, 0.0])

        # TF Handling
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Shared Publishers
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)

        # Start Shared Subscribers (Map and Scan are implemented by child classes)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        """Updates control input (u) and tracks raw odometry for TF."""
        v_x = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        self.u = self.diff_drive.inverse(self.x_est, np.array([v_x, omega]))

        p = msg.pose.pose
        self.latest_odom_pose = np.array(
            [p.position.x, p.position.y, yaw_from_quaternion(p.orientation)]
        )

    def get_laser_offset(self, laser_frame):
        """Looks up the physical LiDAR mount offset."""
        return np.array([0.32825, 0.0, 0.0])

    def publish_pose_and_tf(self, stamp, cov_3x3):
        """Standardized publishing of Pose + Covariance and REP-105 TF."""
        # Publish Pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame

        pose_msg.pose.pose.position.x = self.x_est[0]
        pose_msg.pose.pose.position.y = self.x_est[1]
        pose_msg.pose.pose.orientation = euler_to_quaternion(self.x_est[2])

        # Pack 3x3 covariance into ROS 6x6 format
        cov_6x6 = np.zeros((6, 6))
        cov_6x6[0, 0], cov_6x6[0, 1] = cov_3x3[0, 0], cov_3x3[0, 1]
        cov_6x6[1, 0], cov_6x6[1, 1] = cov_3x3[1, 0], cov_3x3[1, 1]
        cov_6x6[0, 5], cov_6x6[5, 0] = cov_3x3[0, 2], cov_3x3[2, 0]
        cov_6x6[1, 5], cov_6x6[5, 1] = cov_3x3[1, 2], cov_3x3[2, 1]
        cov_6x6[5, 5] = cov_3x3[2, 2]

        pose_msg.pose.covariance = tuple(cov_6x6.flatten())
        self.pub_pose.publish(pose_msg)

        # # Broadcast map -> odom TF
        # map_to_odom = compose(self.x_est, inverse(self.latest_odom_pose))

        # tf = TransformStamped()
        # tf.header.stamp = stamp
        # tf.header.frame_id = self.map_frame
        # tf.child_frame_id = self.odom_frame

        # tf.transform.translation.x = map_to_odom[0]
        # tf.transform.translation.y = map_to_odom[1]
        # tf.transform.translation.z = 0.0
        # tf.transform.rotation = euler_to_quaternion(map_to_odom[2])

        # self.tf_broadcaster.sendTransform(tf)
