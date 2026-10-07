"""The live EKF node must hand each message's header stamp to the pipeline, so scans are
fused at their own time (EKFPipeline). Needs rclpy, so it only runs in a ROS environment."""

import pytest

rclpy = pytest.importorskip('rclpy')

from builtin_interfaces.msg import Time  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from devol_localization.ekf_localization import EKFLocalization  # noqa: E402


def stamped(msg, sec: int, nanosec: int):
    msg.header.stamp = Time(sec=sec, nanosec=nanosec)
    return msg


def test_node_passes_odom_and_scan_stamps_to_the_pipeline():
    rclpy.init(args=['--ros-args', '-p', 'init_mode:=pose'])
    try:
        node = EKFLocalization()
        calls = []
        pipeline = node._pipeline
        pipeline.on_odom = lambda pose, stamp=None: calls.append(('odom', stamp))
        pipeline.on_scan = lambda points, stamp=None: calls.append(('scan', stamp))
        pipeline.matcher = object()

        odom = stamped(Odometry(), 12, 250_000_000)
        odom.pose.pose.orientation.w = 1.0
        node.odom_received(odom)
        scan = stamped(LaserScan(), 12, 300_000_000)
        scan.angle_min, scan.angle_increment = -1.0, 0.01
        scan.range_min, scan.range_max = 0.05, 25.0
        scan.ranges = [2.0] * 200
        node.scan_received(scan)

        assert calls == [('odom', pytest.approx(12.25)), ('scan', pytest.approx(12.3))]
        node.destroy_node()
    finally:
        rclpy.shutdown()
