#!/usr/bin/env python3

from rclpy import init, spin, try_shutdown
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

from numpy import array, float32, ones_like, where, ndarray
from scipy.ndimage import distance_transform_edt
from zlib import crc32
from math import sin, cos, sqrt, hypot, pi
from subprocess import run
from random import choice, uniform
from dataclasses import dataclass


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


@dataclass
class BodyBox:
    """Axis-aligned box of the robot body in base_footprint (z = 0 on the ground)."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

# Clearpath A200 + stowed UR5e + Robotiq gripper
DEFAULT_BODY = [
    BodyBox(-0.495, 0.495, -0.335, 0.335, 0.05, 0.45),  # chassis, wheels, top plate
    BodyBox(-0.10, 0.10, -0.30, 0.30, 0.45, 0.82),      # 300 mm sensor arch + VLP-16
    BodyBox(0.08, 0.35, -0.10, 0.42, 0.45, 1.55),       # stowed UR5e + Robotiq, offset to +y
]

def set_pose_command(
    world: str, entity: str, x: float, y: float, z: float, yaw: float, timeout_ms: int = 3000
):
    req = (
        f'name: "{entity}", position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: 0, y: 0, z: {sin(yaw / 2.0)}, w: {cos(yaw / 2.0)}}}'
    )
    return [
        'gz',
        'service',
        '-s',
        f'/world/{world}/set_pose',
        '--reqtype',
        'gz.msgs.Pose',
        '--reptype',
        'gz.msgs.Boolean',
        '--timeout',
        str(timeout_ms),
        '--req',
        req,
    ]

def calculate_safe_radius(boxes: list[BodyBox]) -> float:
    """Calculates the maximum horizontal distance from the origin to any corner of the body boxes."""
    max_r_sq = 0.0
    for box in boxes:
        corners = [
            (box.x_min, box.y_min),
            (box.x_min, box.y_max),
            (box.x_max, box.y_min),
            (box.x_max, box.y_max)
        ]
        for cx, cy in corners:
            r_sq = cx**2 + cy**2
            if r_sq > max_r_sq:
                max_r_sq = r_sq
    return sqrt(max_r_sq)

class KidnapperNode(Node):
    def __init__(self):
        super().__init__('kidnapper_node')
        
        # Declare Parameters
        self.declare_parameter('odom_topic', '/devol_drive/odom')
        self.declare_parameter('map_topic', '/devol_drive/projected_map')
        self.declare_parameter('scan_topic', '/devol_drive/sensors/lidar2d_0/scan')
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'a200_base_link')
        self.declare_parameter('target_x', -4.7)
        self.declare_parameter('target_y', 9.2)
        self.declare_parameter('tolerance', 0.3)
        self.declare_parameter('wait_time', 20.0)
        self.declare_parameter('world_name', 'maze_world')
        self.declare_parameter('entity_name', 'devol_drive')

        # Retrieve Parameters
        self.odom_topic: str = self.get_parameter('odom_topic').value
        self.map_topic: str = self.get_parameter('map_topic').value
        self.scan_topic: str = self.get_parameter('scan_topic').value
        self.pose_topic: str = self.get_parameter('pose_topic').value
        self.map_frame: str = self.get_parameter('map_frame').value
        self.odom_frame: str = self.get_parameter('odom_frame').value
        self.base_frame: str = self.get_parameter('base_frame').value
        self.target_x: float = self.get_parameter('target_x').value
        self.target_y: float = self.get_parameter('target_y').value
        self.tolerance: float = self.get_parameter('tolerance').value
        self.wait_time: float = self.get_parameter('wait_time').value

        # Automatic safe radius calculation with an extra 0.1m buffer
        self.safe_radius = calculate_safe_radius(DEFAULT_BODY) + 0.1
        self.get_logger().info(f'Calculated safe spawn radius: {self.safe_radius:.2f}m')
        
        # State variables
        self.last_map_crc = None
        self.map_dict = None
        self.state = 'WAITING_FOR_MAP'
        self.countdown_start_time = None
        
        # TF tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriptions and timers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.timer = self.create_timer(0.5, self.control_loop)

    def is_map_new(self, msg: OccupancyGrid) -> bool:
        """Computes a CRC32 hash of the map data.
        Returns True if the map is new/changed, False if it is identical."""
        # Convert the map data array to continuous bytes and hash it
        current_crc = crc32(bytes(msg.data))

        if self.last_map_crc == current_crc:
            return False

        self.last_map_crc = current_crc
        return True

    def map_callback(self, msg: OccupancyGrid):
        """Converts ROS OccupancyGrid into a distance transform map for finding safe spawns."""
        """Converts ROS OccupancyGrid into a distance transform map for the Likelihood field"""
        if not self.is_map_new(msg):
            return

        # Convert 1D array to 2D numpy array
        grid: ndarray = array(msg.data, dtype=float32).reshape(
            (msg.info.height, msg.info.width)
        )

        # Create binary map where obstacle = 0, free space = 1
        binary_map: ndarray = ones_like(grid)
        binary_map[grid > 50] = 0

        # Calculate Euclidean distance to nearest obstacle (in pixels)
        distance_grid: ndarray = distance_transform_edt(binary_map)
        edt_map: ndarray = distance_grid * msg.info.resolution

        # Pre-compute free space coordinates for the escape hatch
        free_y, free_x = where(binary_map == 1)
        res: float = msg.info.resolution
        ox: float = msg.info.origin.position.x
        oy: float = msg.info.origin.position.y

        # Convert pixel indices to real-world coordinates
        free_world_x: ndarray = free_x * res + ox + (res / 2.0)
        free_world_y: ndarray = free_y * res + oy + (res / 2.0)

        # Store in dict to pass to LaserModel easily
        self.map_dict: dict = {
            'grid': edt_map,
            'info': msg.info,
            'free_x': free_world_x,
            'free_y': free_world_y,
        }
        self.get_logger().info('Map received and Euclidean Distance Transform computed.')
        if self.state == 'WAITING_FOR_MAP':
            self.state = 'WAITING_FOR_ROBOT'

    def control_loop(self):
        if self.state in ['WAITING_FOR_MAP', 'DONE']:
            return
            
        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
        
        try:
            trans = self.tf_buffer.lookup_transform(map_frame, base_frame, Time())
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'Waiting for TF ({map_frame} to {base_frame}): {str(e)}', throttle_duration_sec=5.0)
            return

        if self.state == 'WAITING_FOR_ROBOT':
            target_x = self.get_parameter('target_x').value
            target_y = self.get_parameter('target_y').value
            tolerance = self.get_parameter('tolerance').value
            
            dist = hypot(robot_x - target_x, robot_y - target_y)
            if dist <= tolerance:
                self.get_logger().info(f'Target reached (dist: {dist:.2f}m). Starting countdown.')
                self.state = 'COUNTDOWN'
                self.countdown_start_time = self.get_clock().now()
                
        elif self.state == 'COUNTDOWN':
            wait_time = self.get_parameter('wait_time').value
            elapsed = (self.get_clock().now() - self.countdown_start_time).nanoseconds / 1e9
            
            self.get_logger().info(f'Kidnapping in {wait_time - elapsed:.1f}s...', throttle_duration_sec=2.0)
            
            if elapsed >= wait_time:
                self.get_logger().info('Executing kidnap sequence!')
                self.teleport_robot()
                self.state = 'DONE'

    def teleport_robot(self, target_z: float = 0.1):
        if not self.map_dict:
            self.get_logger().error('Map data is missing. Cannot teleport.')
            return

        edt_map = self.map_dict['grid']
        px_x = self.map_dict['free_pixel_x']
        px_y = self.map_dict['free_pixel_y']
        
        # Check clearance for all free pixels at once
        clearances = edt_map[px_y, px_x]
        safe_indices = where(clearances > self.safe_radius)[0]
        
        if len(safe_indices) == 0:
            self.get_logger().error('No spaces found with sufficient clearance for the robot!')
            return
            
        chosen_idx = choice(safe_indices)
        target_x = self.map_dict['free_x'][chosen_idx]
        target_y = self.map_dict['free_y'][chosen_idx]
        target_yaw = uniform(-pi, pi)
        
        world = self.get_parameter('world_name').value
        entity = self.get_parameter('entity_name').value
        
        cmd = set_pose_command(world, entity, target_x, target_y, target_z, target_yaw)
        self.get_logger().info(f'Teleporting to free space: x={target_x:.2f}, y={target_y:.2f}, yaw={target_yaw:.2f}')
        
        result = run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(f'Teleport command failed: {result.stderr}')
        else:
            self.get_logger().info('Teleport successful.')

def main(args=None):
    init(args=args)
    node = KidnapperNode()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try_shutdown()

if __name__ == '__main__':
    main()