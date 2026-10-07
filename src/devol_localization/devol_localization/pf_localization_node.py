#!/usr/bin/env python3
"""Particle filter (MCL) localization node.

Inputs:
  - odometry  (nav_msgs/Odometry, default /devol_drive/odom): motion input
  - map       (nav_msgs/OccupancyGrid, default /devol_drive/projected_map): from octomap_server
  - scan      (sensor_msgs/LaserScan, default /devol_drive/sensors/lidar2d_0/scan):
              measurement matched against the map
  - initialpose (geometry_msgs/PoseWithCovarianceStamped): re-initialize around a pose

Outputs:
  - pose      (geometry_msgs/PoseWithCovarianceStamped, default /devol_drive/pf_pose):
              robot (x, y, yaw) in the map frame, published at the odometry rate
  - particles (geometry_msgs/PoseArray, default /devol_drive/pf_particles)
  - compute time per filter update in ms (std_msgs/Float64, default /devol_drive/pf_compute_time_ms)
  - map -> odom TF (REP 105), only when publish_tf is true

Services:
  - ~/global_localization (std_srvs/Empty): spread particles uniformly over the map
"""

import math
import time
import zlib
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException

from devol_localization.graceful import init_with_stop_flag
from devol_localization.particle_filter import (
    LikelihoodField,
    ParticleFilter,
    PFParams,
    odometry_delta,
    wrap_angle,
)


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def set_yaw(q, yaw: float) -> None:
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)


def compose(a, b) -> np.ndarray:
    """a ⊕ b for 2D poses (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array(
        [a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], float(wrap_angle(a[2] + b[2]))]
    )


def inverse(a) -> np.ndarray:
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]])


class PFLocalizationNode(Node):
    def __init__(self):
        super().__init__('pf_localization')

        # Topics and frames
        self.declare_parameter('odom_topic', '/devol_drive/odom')
        self.declare_parameter('map_topic', '/devol_drive/projected_map')
        self.declare_parameter('scan_topic', '/devol_drive/sensors/lidar2d_0/scan')
        self.declare_parameter('pose_topic', '/devol_drive/pf_pose')
        self.declare_parameter('particles_topic', '/devol_drive/pf_particles')
        self.declare_parameter('compute_time_topic', '/devol_drive/pf_compute_time_ms')
        self.declare_parameter('initialpose_topic', '/devol_drive/initialpose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'a200_base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('tf_tolerance', 0.1)

        # Laser mounting in the base frame, used when TF has no base -> laser
        # transform. Default is the a200 URDF chain to lidar2d_0_laser.
        self.declare_parameter('laser_x', 0.32825)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_yaw', 0.0)

        # Filter
        self.declare_parameter('num_particles', 500)
        self.declare_parameter('alpha1', 0.3)
        self.declare_parameter('alpha2', 0.22)
        self.declare_parameter('alpha3', 0.22)
        self.declare_parameter('alpha4', 0.22)
        self.declare_parameter('min_translation', 0.01)
        self.declare_parameter('sigma_hit', 0.3)
        self.declare_parameter('z_hit', 0.9)
        self.declare_parameter('z_rand', 0.1)
        self.declare_parameter('max_beams', 30)
        self.declare_parameter('likelihood_max_dist', 2.0)
        self.declare_parameter('resample_threshold', 0.5)
        self.declare_parameter('alpha_slow', 0.001)
        self.declare_parameter('alpha_fast', 0.1)
        self.declare_parameter('update_min_d', 0.05)
        self.declare_parameter('update_min_a', 0.05)
        self.declare_parameter('seed', -1)
        self.declare_parameter('particles_publish_count', 500)

        # Initialization: 'tf' (current map -> odom TF composed with odometry),
        # 'pose' (initial_x/y/yaw), or 'global' (uniform over the map).
        self.declare_parameter('init_mode', 'tf')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('initial_std_xy', 0.25)
        self.declare_parameter('initial_std_yaw', 0.2)

        gp = self.get_parameter
        self._map_frame = gp('map_frame').value
        self._odom_frame = gp('odom_frame').value
        self._base_frame = gp('base_frame').value
        self._publish_tf = bool(gp('publish_tf').value)
        self._tf_tolerance = float(gp('tf_tolerance').value)
        self._update_min_d = float(gp('update_min_d').value)
        self._update_min_a = float(gp('update_min_a').value)
        self._init_mode = str(gp('init_mode').value)
        self._lik_max_dist = float(gp('likelihood_max_dist').value)
        self._particles_publish_count = int(gp('particles_publish_count').value)
        seed = int(gp('seed').value)

        params = PFParams(
            num_particles=int(gp('num_particles').value),
            alpha1=float(gp('alpha1').value),
            alpha2=float(gp('alpha2').value),
            alpha3=float(gp('alpha3').value),
            alpha4=float(gp('alpha4').value),
            min_translation=float(gp('min_translation').value),
            sigma_hit=float(gp('sigma_hit').value),
            z_hit=float(gp('z_hit').value),
            z_rand=float(gp('z_rand').value),
            max_beams=int(gp('max_beams').value),
            resample_threshold=float(gp('resample_threshold').value),
            alpha_slow=float(gp('alpha_slow').value),
            alpha_fast=float(gp('alpha_fast').value),
        )
        self._pf = ParticleFilter(params, seed=None if seed < 0 else seed)

        # State
        self._map_crc: Optional[int] = None
        self._odom_pose: Optional[np.ndarray] = None  # latest odom -> base
        self._odom_stamp = None
        self._last_update_odom: Optional[np.ndarray] = None  # odom pose at last filter update
        self._map_to_odom = np.zeros(3)  # correction published as map -> odom
        self._cov = np.zeros((3, 3))  # particle covariance at last update
        self._laser_pose: Optional[np.ndarray] = None
        self._laser_frame: Optional[str] = None
        self._pending_global = self._init_mode == 'global'

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        # I/O
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, gp('pose_topic').value, 10
        )
        self._particles_pub = self.create_publisher(PoseArray, gp('particles_topic').value, 1)
        self._compute_pub = self.create_publisher(Float64, gp('compute_time_topic').value, 10)
        # Latched map: transient_local gets the last map even if it was published once
        # before this node started (octomap_server latches its projected map).
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, gp('map_topic').value, self._map_cb, map_qos)
        self.create_subscription(Odometry, gp('odom_topic').value, self._odom_cb, 50)
        self.create_subscription(LaserScan, gp('scan_topic').value, self._scan_cb, 5)
        self.create_subscription(
            PoseWithCovarianceStamped, gp('initialpose_topic').value, self._initialpose_cb, 1
        )
        self.create_service(Empty, '~/global_localization', self._global_srv)

        self.get_logger().info(
            f'PF localization: {params.num_particles} particles, init_mode={self._init_mode}, '
            f'publish_tf={self._publish_tf}'
        )

    # ------------------------------------------------------------- callbacks
    def _map_cb(self, msg: OccupancyGrid) -> None:
        data = np.asarray(msg.data, dtype=np.int8)
        crc = zlib.crc32(data.tobytes()) ^ (msg.info.width << 16) ^ msg.info.height
        if crc == self._map_crc:
            return
        q = msg.info.origin.orientation
        if abs(yaw_from_quaternion(q)) > 1e-6:
            self.get_logger().warning('Map origin is rotated; rotation is ignored.')
        grid = data.reshape((msg.info.height, msg.info.width))
        self._pf.field = LikelihoodField(
            grid,
            msg.info.resolution,
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            max_dist=self._lik_max_dist,
        )
        self._map_crc = crc
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m'
        )
        if self._pending_global:
            self._pf.init_uniform()
            self._pending_global = False
            self._start_tracking()
            self.get_logger().info('Initialized particles uniformly over the map')

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._odom_pose = np.array(
            [p.position.x, p.position.y, yaw_from_quaternion(p.orientation)]
        )
        self._odom_stamp = msg.header.stamp

        if not self._pf.initialized:
            self._try_initialize()
            return
        if self._last_update_odom is None:
            self._start_tracking()
        self._publish_pose(msg.header.stamp)

    def _scan_cb(self, msg: LaserScan) -> None:
        if not self._pf.initialized or self._pf.field is None or self._odom_pose is None:
            return
        if self._laser_pose is None or self._laser_frame != msg.header.frame_id:
            self._laser_pose = self._lookup_laser_pose(msg.header.frame_id)
            self._laser_frame = msg.header.frame_id

        rot1, trans, rot2 = odometry_delta(self._last_update_odom, self._odom_pose)
        moved_a = abs(wrap_angle(self._odom_pose[2] - self._last_update_odom[2]))
        if abs(trans) < self._update_min_d and moved_a < self._update_min_a:
            return

        t0 = time.perf_counter()
        self._pf.predict(rot1, trans, rot2)
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        self._pf.update(ranges, angles, msg.range_min, msg.range_max, self._laser_pose)
        self._pf.resample_if_needed()
        est, self._cov = self._pf.estimate()
        compute_ms = (time.perf_counter() - t0) * 1000.0

        self._last_update_odom = self._odom_pose.copy()
        # map -> odom = map -> base ⊕ (odom -> base)^-1
        self._map_to_odom = compose(est, inverse(self._last_update_odom))

        self._compute_pub.publish(Float64(data=compute_ms))
        self._publish_particles(msg.header.stamp)
        self._publish_pose(msg.header.stamp)

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        cov = msg.pose.covariance
        std_xy = math.sqrt(max(cov[0], cov[7], 1e-4))
        std_yaw = math.sqrt(max(cov[35], 1e-4))
        self._pf.init_gaussian(pose, std_xy, std_yaw)
        self._start_tracking()
        self.get_logger().info(f'Re-initialized at ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})')

    def _global_srv(self, request, response):
        if self._pf.field is None:
            self._pending_global = True
            self.get_logger().info('Global localization requested; waiting for a map')
        else:
            self._pf.init_uniform()
            self._start_tracking()
            self.get_logger().info('Particles spread uniformly over the map')
        return response

    # --------------------------------------------------------------- helpers
    def _try_initialize(self) -> None:
        if self._init_mode == 'pose':
            gp = self.get_parameter
            pose = (
                float(gp('initial_x').value),
                float(gp('initial_y').value),
                float(gp('initial_yaw').value),
            )
        elif self._init_mode == 'tf':
            try:
                tf = self._tf_buffer.lookup_transform(self._map_frame, self._odom_frame, Time())
            except TransformException as e:
                self.get_logger().info(
                    f'Waiting for {self._map_frame} -> {self._odom_frame}: {e}',
                    throttle_duration_sec=5.0,
                )
                return
            t = tf.transform
            map_odom = np.array(
                [t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation)]
            )
            pose = compose(map_odom, self._odom_pose)
        else:
            return  # 'global' initializes when the map arrives
        self._pf.init_gaussian(
            pose,
            float(self.get_parameter('initial_std_xy').value),
            float(self.get_parameter('initial_std_yaw').value),
        )
        self._start_tracking()
        self.get_logger().info(
            f'Initialized ({self._init_mode}) at ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})'
        )

    def _start_tracking(self) -> None:
        """Anchor odometry deltas and the map -> odom correction at the current odom pose."""
        if self._odom_pose is None:
            return
        self._last_update_odom = self._odom_pose.copy()
        est, self._cov = self._pf.estimate()
        self._map_to_odom = compose(est, inverse(self._last_update_odom))

    def _lookup_laser_pose(self, laser_frame: str) -> np.ndarray:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, laser_frame, Time(), timeout=Duration(seconds=0.5)
            )
            t = tf.transform
            pose = np.array([t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation)])
            self.get_logger().info(
                f'Laser pose from TF {self._base_frame} -> {laser_frame}: {pose}'
            )
        except TransformException:
            gp = self.get_parameter
            pose = np.array(
                [
                    float(gp('laser_x').value),
                    float(gp('laser_y').value),
                    float(gp('laser_yaw').value),
                ]
            )
            self.get_logger().warning(
                f'No TF {self._base_frame} -> {laser_frame}; using laser_x/y/yaw params {pose}'
            )
        return pose

    def _publish_pose(self, stamp) -> None:
        if self._odom_pose is None:
            return
        pose = compose(self._map_to_odom, self._odom_pose)
        cov = self._cov

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._map_frame
        msg.pose.pose.position.x = float(pose[0])
        msg.pose.pose.position.y = float(pose[1])
        set_yaw(msg.pose.pose.orientation, float(pose[2]))
        c = [0.0] * 36
        for i, ci in enumerate((0, 1, 5)):
            for j, cj in enumerate((0, 1, 5)):
                c[ci * 6 + cj] = float(cov[i, j])
        msg.pose.covariance = c
        self._pose_pub.publish(msg)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            # Future-date like AMCL so consumers can interpolate up to the next update.
            tf.header.stamp = (
                Time.from_msg(stamp) + Duration(seconds=self._tf_tolerance)
            ).to_msg()
            tf.header.frame_id = self._map_frame
            tf.child_frame_id = self._odom_frame
            tf.transform.translation.x = float(self._map_to_odom[0])
            tf.transform.translation.y = float(self._map_to_odom[1])
            set_yaw(tf.transform.rotation, float(self._map_to_odom[2]))
            self._tf_broadcaster.sendTransform(tf)

    def _publish_particles(self, stamp) -> None:
        if self._particles_pub.get_subscription_count() == 0:
            return
        parts = self._pf.particles
        if parts.shape[0] > self._particles_publish_count > 0:
            parts = parts[
                np.linspace(0, parts.shape[0] - 1, self._particles_publish_count).astype(int)
            ]
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self._map_frame
        for x, y, yaw in parts:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            set_yaw(p.orientation, float(yaw))
            msg.poses.append(p)
        self._particles_pub.publish(msg)


def main(args=None):
    stop = init_with_stop_flag(args)
    node = PFLocalizationNode()
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
