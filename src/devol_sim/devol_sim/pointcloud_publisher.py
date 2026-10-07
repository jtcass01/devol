#!/usr/bin/env python3

# ===============================================================================
# Build
#
# colcon build --packages-select devol_sim
# source install/setup.bash
#
# To Run
#
# ros2 run devol_sim pointcloud_publisher       \
#   --ros-args                                     \
#   -p pointcloud_file:=/absolute/path/to/file.pcd \
#   -p frame_id:=map                               \
#   -p publish_rate:=1.0
#
#
# To Visualize
#
# rviz2
#
# ===============================================================================

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from pathlib import Path

import numpy as np


# ==============================================================================
class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')

        # Parameters
        self.declare_parameter('pointcloud_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('namespace', '/devol_drive')
        self.declare_parameter('topic_name', 'cloud_in')

        pointcloud_path = Path(
            self.get_parameter('pointcloud_file').get_parameter_value().string_value
        )
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self._topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        topic: str = f'{self._namespace}/{self._topic_name}'

        if not pointcloud_path.exists():
            raise FileNotFoundError(f'PointCloud file not found: {pointcloud_path}')

        self._points = self.load_ascii_pointcloud(pointcloud_path)
        # Packed once: the cloud is static, so every publish sends the same bytes.
        self._data = self._points.astype('<f4').tobytes()
        self._publisher = self.create_publisher(PointCloud2, topic, 10)
        self._timer = self.create_timer(1.0 / rate, self.publish_pointcloud)

        self.get_logger().info(f'Loaded {len(self._points)} points from {pointcloud_path}')

    # ==========================================================================
    def load_ascii_pointcloud(self, path: Path) -> np.ndarray:
        """Load XYZ points from an ASCII PointCloud file as an (N, 3) float32 array."""
        with open(path) as f:
            for header_lines, line in enumerate(f, start=1):
                if line.startswith('DATA ascii'):
                    break
        return np.loadtxt(
            path, skiprows=header_lines, usecols=(0, 1, 2), dtype=np.float32, ndmin=2
        )

    # ==========================================================================
    def publish_pointcloud(self):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.height = 1
        msg.width = len(self._points)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12  # 3 * float32
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = self._data

        self._publisher.publish(msg)


# ==============================================================================
def main():
    rclpy.init()
    node = PointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# ==============================================================================
if __name__ == '__main__':
    main()
