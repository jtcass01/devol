#!/usr/bin/env python3
"""
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from numpy import ndarray, ones, ceil, zeros_like, array, int8
from scipy.ndimage import binary_dilation
from threading import Lock


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_padder')

        self._lock: Lock = Lock()

        # Params
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('namespace', '/devol_drive')
        self.declare_parameter('robot_width', 0.784)  # in meters

        self._publish_rate: float = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self._namespace: str = str(self.get_parameter('namespace').get_parameter_value().string_value)
        self._robot_width: float = float(self.get_parameter('robot_width').get_parameter_value().double_value)

        # I/O
        self._map_pub = self.create_publisher(OccupancyGrid, f'{self._namespace}/map', 10)
        self._map_sub = self.create_subscription(OccupancyGrid, f'{self._namespace}/projected_map', self.map_received, 10)
 
        self._map = None
        self._padded_map = None
        self._inflation_radius = None
        self._grid_width = None
        self._grid_height = None
       
        # Publish at 1 Hz (static map)
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('Map publisher initialized')

    def map_received(self, msg: OccupancyGrid) -> None:
        try:
            map_data: ndarray = array(msg.data, dtype=int8)
            grid_width: int = msg.info.width
            grid_height: int = msg.info.height
            origin_x = float(msg.info.origin.position.x)
            origin_y = float(msg.info.origin.position.y)

            map_resolution = float(msg.info.resolution)

            inflation_distance_meters = self._robot_width
            inflation_radius = int(ceil(inflation_distance_meters / map_resolution))
            inflation_radius = max(1, inflation_radius)

            map = map_data.reshape((grid_height, grid_width))
            padded_map = self.pad_map(grid=map, inflation_radius=inflation_radius)

            with self._lock:
                self._padded_map = padded_map
                self._grid_width: int = grid_width
                self._grid_height: int = grid_height
                self._map_resolution = map_resolution
                self._origin_x = origin_x
                self._origin_y = origin_y
                self._inflation_radius = inflation_radius
        except Exception as e:
            self.get_logger().error(f'Error processing map: {e}')

    def pad_map(self, grid: ndarray, inflation_radius: int) -> ndarray:
        """Inflate obstacles in the map by the specified inflation radius.
        
        Args:
            grid: Input occupancy grid (100 = obstacle, 0 = free, -1 = unknown)
            inflation_radius: Radius for obstacle inflation in grid cells
            
        Returns:
            Inflated occupancy grid with same shape and values"""
        obstacles = grid == 100

        # Create structuring element for dilation
        structure = ones((inflation_radius, inflation_radius))
        
        # Dilate obstacles
        inflated = binary_dilation(obstacles, structure=structure)
        
        # Create output map
        inflated_map = zeros_like(grid, dtype=int8)
        inflated_map[inflated] = 100  # Set inflated areas as obstacles
        inflated_map[grid == -1] = -1  # Preserve unknown areas
        
        return inflated_map

    def publish_map(self):
        """Publish the occupancy grid map"""
        # Don't publish if map is not available
        with self._lock:
            if self._padded_map is None:
                return
            
            padded_map = self._padded_map.copy()
            map_resolution = self._map_resolution
            grid_width = self._grid_width
            grid_height = self._grid_height
            origin_x = self._origin_x
            origin_y = self._origin_y

        msg = OccupancyGrid()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Map metadata
        msg.info.resolution = map_resolution
        msg.info.width = grid_width
        msg.info.height = grid_height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Flatten the grid (row-major order) and convert to list
        msg.data = padded_map.flatten().tolist()
        
        self._map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()