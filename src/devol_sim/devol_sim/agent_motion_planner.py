#!/usr/bin/env python3
from __future__ import annotations
import numpy as np
from typing import List, Tuple
from threading import Thread, Event

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as RCLPY_Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import tf2_ros
from matplotlib.pyplot import ion, subplots, pause, close as plt_close

from devol_sim.a_star_planner import a_star_grid
from devol_sim.utils import quaternion_to_euler, euler_to_quaternion
from devol_sim.mobile_robot_goal import MobileRobotGoal


def euclidean_distance(p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
    x1, y1 = p1
    x2, y2 = p2
    return ((x2-x1)**2+(y2-y1)**2)**0.5


class AgentMotionPlanner(RCLPY_Node):
    """
    PID controller for differential drive robot using trailer hitch approach.
    Based on EN613 midterm solution.
    """
    def __init__(self):
        super().__init__('agent_motion_planner')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('viz_rate', 2.0)
        self.declare_parameter('namespace', '/devol_drive')
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('lookahead', 0.15)
        self.declare_parameter('intermediate_goal_tolerance', 0.15)
        self.declare_parameter('tf_to_frame', 'map')
        self.declare_parameter('tf_from_frame', 'a200_base_link')

        self._publish_rate: float = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self._viz_rate: float = float(self.get_parameter('viz_rate').get_parameter_value().double_value)
        self._namespace: str = str(self.get_parameter('namespace').get_parameter_value().string_value)
        self._goal_tolerance: float = float(self.get_parameter('goal_tolerance').get_parameter_value().double_value)
        self._lookahead: float = float(self.get_parameter('lookahead').get_parameter_value().double_value)
        self._intermediate_goal_tolerance: float = float(self.get_parameter('intermediate_goal_tolerance').get_parameter_value().double_value)
        self._dt: float = 1.0 / self._publish_rate

        # TF frames
        self._to_frame: str = str(self.get_parameter('tf_to_frame').get_parameter_value().string_value)
        tf_from_frame: str = str(self.get_parameter('tf_from_frame').get_parameter_value().string_value)
        self._from_frame = f'{self._namespace[1:]}/{tf_from_frame}'

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # I/O
        qos_profile = QoSProfile(depth=10)
        self._goal_pub = self.create_publisher(PoseStamped, f'{self._namespace}/goal_pose', qos_profile)
        self._map_sub = self.create_subscription(OccupancyGrid, f'{self._namespace}/map', self.map_received, 10)
        self._goals_sub = self.create_subscription(MarkerArray, f'{self._namespace}/goal_points', self.goal_points_received, 10)

        # State
        self._goal_index = 0
        self._goals: List[MobileRobotGoal] = []
        self._map_resolution = None
        self._origin_x = None
        self._origin_y = None
        self._map = None
        self._path = None
        self._path_index = 0
        self._robot_grid_state = None
        self._goal_grid_state = None
        self._stop_event: Event = Event()

        if self._viz_rate > 0.0:
            self._viz_thread: Thread = Thread(target=self.visualization_loop, daemon=True)
            self._viz_thread.start()

        # Timer loop
        self.timer = self.create_timer(self._dt, self.plan_to_goals)

        self.get_logger().info(f'AgentMotionPlanner started.')

    def stop(self):
        self._stop_event.set()
        self._viz_thread.join()

    def visualization_loop(self):
        ion()
        fig, ax = subplots(figsize=(6, 6))
        update_period: float = 1.0 / self._viz_rate

        while not self._stop_event.is_set():
            if self._map is None:
                pause(0.1)
                continue

            ax.clear()

            # Draw occupancy grid
            ax.imshow(self._map, cmap='gray_r', origin='lower')

            # Draw A* keypoints
            if self._path is not None:
                xs = [p[1] for p in self._path]
                ys = [p[0] for p in self._path]
                ax.plot(xs, ys, 'b.-', label='Keypoints')

            # Draw robot position
            if self._robot_grid_state is not None:
                ax.plot(self._robot_grid_state[1], self._robot_grid_state[0], 'ro', label='Robot')

            # Draw goals
            for goal in self._goals:
                r, c = self.world_to_grid(goal.x, goal.y)
                ax.plot(c, r, 'gs', markersize=8, label=goal.name)

            # Draw short-term goal
            if self._goal_grid_state is not None:
                ax.plot(self._goal_grid_state[1], self._goal_grid_state[0], 'yo', label='Interim Goal')

            ax.set_title("Live Map Visualization")
            ax.set_xlabel("grid X")
            ax.set_ylabel("grid Y")
            ax.legend(loc='upper right')

            pause(update_period)
        
        plt_close(fig)

    def map_received(self, msg: OccupancyGrid) -> None:
        map_data: np.ndarray = np.array(msg.data, dtype=np.int8)
        grid_width: float = msg.info.width
        grid_height: float = msg.info.height

        self._map_resolution = float(msg.info.resolution)
        self._origin_x = float(msg.info.origin.position.x)
        self._origin_y = float(msg.info.origin.position.y)

        self._map = map_data.reshape((grid_height, grid_width))

    def goal_points_received(self, msg: MarkerArray) -> None:
        # If there are already goals, don't accept more
        if len(self._goals) > 0:
            return

        # Make a goal for each marker
        for marker in reversed(msg.markers):
            marker: Marker = marker # Added for linter
            name: str = marker.text
            x: float = marker.pose.position.x
            y: float = marker.pose.position.y
            z: float = marker.pose.position.z
            _, _, yaw = quaternion_to_euler(marker.pose.orientation)
            goal: MobileRobotGoal = MobileRobotGoal(name=name,
                                                    x=x, y=y, z=z,
                                                    yaw=yaw)
            self.get_logger().info(f'Received {goal}')
            self._goals.append(goal)

    def send_goal_pose(self, position: Tuple[float, float], yaw: float) -> None:
        msg: PoseStamped = PoseStamped()
        msg.header.frame_id = self._to_frame
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]

        quat: Tuple[float, float, float, float] = \
            euler_to_quaternion(0.0, 0.0, yaw)
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        self._goal_pub.publish(msg)

    def world_shift_trailer_hitch(self, x, y, theta):
        x_trailer = x + self._lookahead * np.cos(theta)
        y_trailer = y + self._lookahead * np.sin(theta)
        return x_trailer, y_trailer

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        col = int((x - self._origin_x) / self._map_resolution)
        row = int((y - self._origin_y) / self._map_resolution)

        n_rows, n_cols = self._map.shape
        row = max(0, min(n_rows-1, row))
        col = max(0, min(n_cols-1, col))

        return row, col
    
    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        x = self._origin_x + (col+0.5) * self._map_resolution
        y = self._origin_y + (row+0.5) * self._map_resolution
        return x, y

    def add_yaw_to_path(self, path_grid: List[Tuple[int, int]], goal_yaw: float) -> List[Tuple[int, int, float]]:
        """
        Add yaw information to path by looking ahead to next waypoint.
        For the last waypoint, use the direction from the previous waypoint.
        
        Args:
            path_grid: List of (row, col) tuples in grid coordinates
            
        Returns:
            List of (row, col, yaw) tuples
        """
        if len(path_grid) == 0:
            return []
        
        if len(path_grid) == 1:
            # Single point path - use current robot yaw or default
            return [(path_grid[0][0], path_grid[0][1], 0.0)]
        
        path_with_yaw = []
        
        # For all waypoints except the last, look ahead
        for i in range(len(path_grid) - 1):
            current = path_grid[i]
            next_point = path_grid[i + 1]
            
            # Convert to world coordinates to compute yaw
            x_curr, y_curr = self.grid_to_world(current[0], current[1])
            x_next, y_next = self.grid_to_world(next_point[0], next_point[1])
            
            # Compute yaw pointing to next waypoint
            yaw = np.arctan2(y_next - y_curr, x_next - x_curr)
            
            path_with_yaw.append((current[0], current[1], yaw))
        
        # For the last waypoint, use direction from previous waypoint
        last_point = path_grid[-1]
        path_with_yaw.append((last_point[0], last_point[1], goal_yaw))
        
        return path_with_yaw
        
    def plan_to_goals(self):
        if self._map is None:
            return
        
        if (len(self._goals) == 0) or (self._goal_index >= len(self._goals)):
            return
        
        # Get robot's curent position from tf, it would be better if there was a particle filter on the other side of this.
        try:
            when = rclpy.time.Time()
            trans = self._tf_buffer.lookup_transform(
                self._to_frame, self._from_frame,
                when, timeout=Duration(seconds=0.5))
        except tf2_ros.LookupException:
            self.get_logger().warn('Transform isn\'t available, waiting...')
            return
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {str(e)}')
            return

        pose = trans.transform.translation
        roll, pitch, yaw = quaternion_to_euler(trans.transform.rotation)

        robot_state: Tuple[float, float] = (pose.x, pose.y)
        goal: MobileRobotGoal = self._goals[self._goal_index]
        self._robot_grid_state = self.world_to_grid(robot_state[0], robot_state[1])
        self._goal_grid_state = self.world_to_grid(goal.x, goal.y)

        if euclidean_distance(robot_state, [goal.x, goal.y]) <= self._goal_tolerance:
            # Goal acheived!
            self.get_logger().info(f'Successfully reached goal: {goal.name}')

            self._goal_index += 1
            self._path_index = 0
            self._path = None
        else:
            # Plan from robot state to goal state
            if self._path is None:
                self.get_logger().info(f'Calculating optimal path to {goal.name}')
                path_grid: List[Tuple[int, int]] = a_star_grid(self._map, self._robot_grid_state, self._goal_grid_state)
                self._path: List[Tuple[int, int, float]] = self.add_yaw_to_path(path_grid=path_grid, 
                                                                                goal_yaw=goal.yaw)
                self.get_logger().info(f'Path found: {self._path}')

            # If we have a path, let's walk it.
            if self._path_index < len(self._path):
                row, col, goal_yaw = self._path[self._path_index]
                intermediate_goal: Tuple[float, float] = self.grid_to_world(row, col)
                # Shift the intermediate goal by the trailer hitch
                trailer_hitch_goal: Tuple[float, float] = self.world_shift_trailer_hitch(intermediate_goal[0], intermediate_goal[1], yaw)

                # Check if at intermediate goal:
                if euclidean_distance(robot_state, intermediate_goal) <= self._intermediate_goal_tolerance:
                    self._path_index += 1
                else:
                    self.send_goal_pose(trailer_hitch_goal, goal_yaw)


def main(args=None):
    rclpy.init(args=args)
    node = AgentMotionPlanner()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
