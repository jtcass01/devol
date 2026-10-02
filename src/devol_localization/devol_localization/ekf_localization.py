#!/usr/bin/env python3
"""EKF localization node.

Inputs
  odom_topic   (nav_msgs/Odometry)       dead-reckoned wheel odometry, drives the prediction
  map_topic    (nav_msgs/OccupancyGrid)  octomap_server projected map, the known map
  scan_topic   (sensor_msgs/LaserScan)   2D lidar, scan-matched against the map for the correction
  initialpose  (PoseWithCovarianceStamped, optional) re-initializes the filter

Outputs
  pose_topic   (geometry_msgs/PoseWithCovarianceStamped) pose (x, y, yaw) of the base in the map frame
  compute_time_topic (std_msgs/Float64) scan-update compute time in ms, for the study's compute metric
  TF map -> odom (only when publish_tf is true; the sim launches a static one today)

Frames follow REP 105: odometry owns odom -> base, this node estimates map -> base and
may publish the corresponding map -> odom correction.
"""

from threading import Lock
from time import perf_counter
from typing import Optional, Tuple

from numpy import ndarray, array, asarray, arctan2, column_stack, cos, cov, sin, diag, int8, pi, zeros

from rclpy import init as rclpy_init, try_shutdown as rclpy_try_shutdown, spin as rclpy_spin
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException

from devol_localization.ekf_core import PoseEKF, wrap_angle, CHI2_3DOF_99
from devol_localization.ekf_pipeline import EKFPipeline
from devol_localization.scan_matcher import DistanceField, ScanMatcher, scan_to_points

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


def yaw_from_quaternion(q: Quaternion) -> float:
    return wrap_angle(arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = float(sin(yaw / 2.0))
    q.w = float(cos(yaw / 2.0))
    return q


def compose(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Returns a (+) b for planar poses."""
    c, s = cos(a[2]), sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap_angle(a[2] + b[2]))


def inverse(a: Tuple[float, float, float]) -> Tuple[float, float, float]:
    c, s = cos(a[2]), sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap_angle(-a[2]))


class EKFLocalization(Node):
    def __init__(self) -> None:
        super().__init__('ekf_localization')

        self._lock: Lock = Lock()

        # Params
        self.declare_parameter('namespace', '/devol_drive')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('map_topic', 'projected_map')
        self.declare_parameter('scan_topic', 'sensors/lidar2d_0/scan')
        self.declare_parameter('pose_topic', 'ekf_pose')
        self.declare_parameter('compute_time_topic', 'ekf_compute_time_ms')
        self.declare_parameter('initial_pose_topic', 'initialpose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('map_transient_local', True)
        # Initial pose: by default taken from the (static) map -> odom TF composed with the
        # first odometry message, which is where the robot was spawned.
        self.declare_parameter('initial_pose_from_tf', True)
        # init_mode overrides initial_pose_from_tf when set: 'tf', 'pose' (initial_pose) or
        # 'global' (mean and covariance of the map's free space, the study's global-localization
        # test: a single Gaussian spanning the map).
        self.declare_parameter('init_mode', '')
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_std', [0.1, 0.1, 0.05])
        # Odometry motion model; variances per unit of motion (see ekf_core.PoseEKF).
        self.declare_parameter('odom_alphas', [0.02, 0.01, 0.01, 0.002])
        # Laser pose in the base frame (x, y, yaw); defaults from a200_macro.urdf.xacro.
        self.declare_parameter('laser_pose', [0.32825, 0.0, 0.0])
        # Scan matcher.
        self.declare_parameter('beam_step', 4)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('max_field_distance', 2.0)
        self.declare_parameter('inlier_distance', 0.3)
        self.declare_parameter('min_inlier_fraction', 0.5)
        self.declare_parameter('match_covariance_scale', 20.0)
        self.declare_parameter('match_min_std', [0.05, 0.02])
        self.declare_parameter('max_ambiguity', 0.95)
        self.declare_parameter('gate_chi2', CHI2_3DOF_99)
        # Search window half-widths [min, max], sized from 3 sigma of the covariance.
        self.declare_parameter('search_window_xy', [0.15, 1.5])
        self.declare_parameter('search_window_yaw', [0.05, 0.8])
        # Re-acquisition: after this many failed scans, inflate the covariance per failed scan.
        self.declare_parameter('lost_after', 5)
        self.declare_parameter('lost_inflation_std', [0.05, 0.03])

        ns: str = self._str('namespace').rstrip('/')
        self._map_frame: str = self._str('map_frame')
        self._odom_frame: str = self._str('odom_frame')
        self._publish_tf: bool = self.get_parameter('publish_tf').value
        self._init_mode: str = self._str('init_mode') or (
            'tf' if self.get_parameter('initial_pose_from_tf').value else 'pose')
        if self._init_mode not in ('tf', 'pose', 'global'):
            raise ValueError(f'init_mode must be tf, pose or global, got {self._init_mode}')
        self._free_space: Optional[Tuple[ndarray, ndarray]] = None
        self._initial_pose: Tuple[float, float, float] = tuple(self.get_parameter('initial_pose').value)
        self._initial_std: ndarray = asarray(self.get_parameter('initial_std').value, dtype=float)
        self._laser_pose: Tuple[float, float, float] = tuple(self.get_parameter('laser_pose').value)
        self._beam_step: int = int(self.get_parameter('beam_step').value)

        self._ekf: PoseEKF = PoseEKF(alphas=self.get_parameter('odom_alphas').value)
        self._pipeline: EKFPipeline = EKFPipeline(
            self._ekf,
            window_xy=tuple(self.get_parameter('search_window_xy').value),
            window_yaw=tuple(self.get_parameter('search_window_yaw').value),
            gate=float(self.get_parameter('gate_chi2').value),
            lost_after=int(self.get_parameter('lost_after').value),
            lost_inflation_std=tuple(self.get_parameter('lost_inflation_std').value))
        self._last_odom_msg: Optional[Odometry] = None
        self._match_ms: float = 0.0

        # TF
        self._tf_buffer: Buffer = Buffer()
        self._tf_listener: TransformListener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster: Optional[TransformBroadcaster] = TransformBroadcaster(self) if self._publish_tf else None

        # I/O
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=(DurabilityPolicy.TRANSIENT_LOCAL
                                         if self.get_parameter('map_transient_local').value
                                         else DurabilityPolicy.VOLATILE))
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, f'{ns}/{self._str("pose_topic")}', 10)
        self._compute_pub = self.create_publisher(Float64, f'{ns}/{self._str("compute_time_topic")}', 10)
        self.create_subscription(OccupancyGrid, f'{ns}/{self._str("map_topic")}', self.map_received, map_qos)
        self.create_subscription(Odometry, f'{ns}/{self._str("odom_topic")}', self.odom_received, 50)
        self.create_subscription(LaserScan, f'{ns}/{self._str("scan_topic")}', self.scan_received,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, f'{ns}/{self._str("initial_pose_topic")}',
                                 self.initial_pose_received, 10)
        self.create_timer(5.0, self.log_stats)

        self.get_logger().info(f'EKF localization started; publishing {ns}/{self._str("pose_topic")}, '
                               f'publish_tf={self._publish_tf}')

    def _str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    # ------------------------------------------------------------------ inputs

    def map_received(self, msg: OccupancyGrid) -> None:
        info = msg.info
        grid: ndarray = array(msg.data, dtype=int8).reshape((info.height, info.width))
        origin = (info.origin.position.x, info.origin.position.y, yaw_from_quaternion(info.origin.orientation))
        try:
            field = DistanceField(grid, info.resolution, origin,
                                  occupied_threshold=int(self.get_parameter('occupied_threshold').value),
                                  max_distance=float(self.get_parameter('max_field_distance').value))
        except ValueError as e:
            self.get_logger().error(f'Cannot use map: {e}')
            return
        matcher = ScanMatcher(field,
                              inlier_distance=float(self.get_parameter('inlier_distance').value),
                              min_inlier_fraction=float(self.get_parameter('min_inlier_fraction').value),
                              covariance_scale=float(self.get_parameter('match_covariance_scale').value),
                              min_std=tuple(self.get_parameter('match_min_std').value),
                              max_ambiguity=float(self.get_parameter('max_ambiguity').value))
        occupied = int(self.get_parameter('occupied_threshold').value)
        rows, cols = ((grid >= 0) & (grid < occupied)).nonzero()
        free_space = None
        if rows.size:
            xy = column_stack((info.origin.position.x + (cols + 0.5) * info.resolution,
                               info.origin.position.y + (rows + 0.5) * info.resolution))
            free_space = (xy.mean(axis=0), cov(xy.T))
        with self._lock:
            first: bool = self._pipeline.matcher is None
            self._pipeline.matcher = matcher
            self._free_space = free_space
        if first:
            self.get_logger().info(f'Map received: {info.width}x{info.height} @ {info.resolution:.3f} m')

    def odom_received(self, msg: Odometry) -> None:
        p = msg.pose.pose
        odom_pose: Tuple[float, float, float] = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        with self._lock:
            if not self._ekf.initialized and not self._initialize(odom_pose, msg):
                return
            self._pipeline.on_odom(odom_pose)
            self._last_odom_msg = msg
            self._publish(msg.header.stamp)

    def scan_received(self, msg: LaserScan) -> None:
        with self._lock:
            if self._pipeline.matcher is None or not self._ekf.initialized:
                return
            points = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                                    msg.range_max, self._beam_step, self._laser_pose)
            t0: float = perf_counter()
            lost_before: bool = self._pipeline.stats.failed_in_row >= self._pipeline.lost_after
            result = self._pipeline.on_scan(points)
            dt_ms: float = (perf_counter() - t0) * 1e3
            self._match_ms += dt_ms
            self._compute_pub.publish(Float64(data=dt_ms))
            if result is not None and lost_before:
                self.get_logger().info('Scan match re-acquired')
            if self._last_odom_msg is not None:
                self._publish(self._last_odom_msg.header.stamp)

    def initial_pose_received(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        c = asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
        cov = zeros((3, 3))
        idx = [0, 1, 5]
        for i in range(3):
            for j in range(3):
                cov[i, j] = c[idx[i], idx[j]]
        if cov.trace() <= 0.0:
            cov = diag(self._initial_std ** 2)
        with self._lock:
            self._ekf.reset(pose, cov)
        self.get_logger().info(f'Filter reset to x={pose[0]:.2f} y={pose[1]:.2f} yaw={pose[2]:.2f}')

    # ---------------------------------------------------------------- helpers

    def _initialize(self, odom_pose: Tuple[float, float, float], msg: Odometry) -> bool:
        if self._init_mode == 'global':
            if self._free_space is None:
                self.get_logger().info('Waiting for the map to initialize globally', throttle_duration_sec=5.0)
                return False
            mean, xy_cov = self._free_space
            P = zeros((3, 3))
            P[:2, :2] = xy_cov
            P[2, 2] = (2.0 * pi) ** 2 / 12.0   # uniform heading
            self._ekf.reset((float(mean[0]), float(mean[1]), 0.0), P)
            self.get_logger().info(f'Filter initialized globally at the free-space centroid '
                                   f'x={mean[0]:.2f} y={mean[1]:.2f}, std x={P[0, 0] ** 0.5:.1f} m '
                                   f'y={P[1, 1] ** 0.5:.1f} m')
            return True
        if self._init_mode == 'tf':
            try:
                tf = self._tf_buffer.lookup_transform(self._map_frame, self._odom_frame, Time(),
                                                      timeout=Duration(seconds=0.0))
            except TransformException:
                self.get_logger().info(f'Waiting for {self._map_frame} -> {self._odom_frame} to initialize',
                                       throttle_duration_sec=5.0)
                return False
            t = tf.transform
            map_to_odom = (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
            pose = compose(map_to_odom, odom_pose)
        else:
            pose = self._initial_pose
        self._ekf.reset(pose, diag(self._initial_std ** 2))
        self.get_logger().info(f'Filter initialized at x={pose[0]:.2f} y={pose[1]:.2f} yaw={pose[2]:.2f}')
        return True

    def _publish(self, stamp) -> None:
        x, P = self._ekf.x, self._ekf.P
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._map_frame
        msg.pose.pose.position.x = float(x[0])
        msg.pose.pose.position.y = float(x[1])
        msg.pose.pose.orientation = quaternion_from_yaw(x[2])
        cov = [0.0] * 36
        idx = [0, 1, 5]
        for i in range(3):
            for j in range(3):
                cov[idx[i] * 6 + idx[j]] = float(P[i, j])
        msg.pose.covariance = cov
        self._pose_pub.publish(msg)

        last_odom = self._pipeline.last_odom
        if self._tf_broadcaster is not None and last_odom is not None:
            map_to_odom = compose((x[0], x[1], x[2]), inverse(last_odom))
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._map_frame
            tf.child_frame_id = self._odom_frame
            tf.transform.translation.x = float(map_to_odom[0])
            tf.transform.translation.y = float(map_to_odom[1])
            tf.transform.rotation = quaternion_from_yaw(map_to_odom[2])
            self._tf_broadcaster.sendTransform(tf)

    def log_stats(self) -> None:
        with self._lock:
            s = self._pipeline.stats
            x = self._ekf.x.copy()
            std = self._ekf.P.diagonal() ** 0.5
            ready = self._ekf.initialized
            avg_ms = self._match_ms / s.scans if s.scans else 0.0
            line = (f'pose=({x[0]:.2f}, {x[1]:.2f}, {x[2]:.2f}) std=({std[0]:.2f}, {std[1]:.2f}, {std[2]:.3f}) '
                    f'scans={s.scans} matched={s.matched} fused={s.fused} gated={s.gated} '
                    f'failed_in_row={s.failed_in_row} reacquired={s.reacquired} match={avg_ms:.1f} ms')
            lost = s.failed_in_row >= self._pipeline.lost_after
        if not ready:
            return
        if lost:
            self.get_logger().warning('Scan matching lost; widening search. ' + line)
        else:
            self.get_logger().info(line)


def main(args=None) -> None:
    rclpy_init(args=args)
    node = EKFLocalization()
    try:
        rclpy_spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy_try_shutdown()


if __name__ == '__main__':
    main()
