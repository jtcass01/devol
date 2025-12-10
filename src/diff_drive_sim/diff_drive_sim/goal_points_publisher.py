#!/usr/bin/env python3
"""
Goal Points Publisher Node
Publishes the coordinates of the three goal spheres in the maze.
Reads goal positions from poses.csv file.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from ament_index_python.packages import get_package_share_directory
import os
import csv


class GoalPointsPublisher(Node):
    def __init__(self):
        super().__init__('goal_points_publisher')
        
        # Declare parameter for maze folder
        self.declare_parameter('maze', 'basic_maze')
        maze_folder = self.get_parameter('maze').value
        
        # Publisher for goal points as markers
        self.publisher_ = self.create_publisher(MarkerArray, '/goal_points', 10)
        
        # Read goal positions from CSV file
        pkg_share = get_package_share_directory('devol_gazebo')
        poses_file = os.path.join(pkg_share, 'worlds', maze_folder, 'poses.csv')
        
        self.goal_positions = []
        with open(poses_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row['name'].startswith('goal_'):
                    self.goal_positions.append({
                        'name': row['name'],
                        'x': float(row['x']),
                        'y': float(row['y']),
                        'z': float(row['z'])
                    })
        
        # Publish at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_goals)
        
        self.get_logger().info('Goal points publisher initialized')
        self.get_logger().info(f'Publishing {len(self.goal_positions)} goal points from poses.csv')
        for goal in self.goal_positions:
            self.get_logger().info(f"  {goal['name']}: ({goal['x']}, {goal['y']}, {goal['z']})")
    
    def publish_goals(self):
        """Publish goal points as a MarkerArray"""
        marker_array = MarkerArray()
        
        for i, goal in enumerate(self.goal_positions):
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            
            marker.ns = 'goal_points'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = goal['x']
            marker.pose.position.y = goal['y']
            marker.pose.position.z = goal['z']
            marker.pose.orientation.w = 1.0
            
            # Scale
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            
            # Color - bright green
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            # Lifetime
            marker.lifetime.sec = 0  # 0 means forever
            
            marker_array.markers.append(marker)
        
        self.publisher_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPointsPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
