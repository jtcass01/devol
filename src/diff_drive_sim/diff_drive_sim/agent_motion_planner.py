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
from visualization_msgs.msg import MarkerArray
import tf2_ros
from matplotlib.pyplot import ion, subplots, pause, close as plt_close

from diff_drive_sim.a_star_planner import a_star_grid
from diff_drive_sim.utils import quaternion_to_euler

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


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
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('lookahead', 0.3)
        self.declare_parameter('intermediate_goal_tolerance', 0.15)
        self._publish_rate: float = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self._goal_tolerance: float = float(self.get_parameter('goal_tolerance').get_parameter_value().double_value)
        self._lookahead: float = float(self.get_parameter('lookahead').get_parameter_value().double_value)
        self._intermediate_goal_tolerance: float = float(self.get_parameter('intermediate_goal_tolerance').get_parameter_value().double_value)
        self._dt: float = 1.0 / self._publish_rate

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # I/O
        qos_profile = QoSProfile(depth=10)
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)
        self._map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_received, 10)
        self._goals_sub = self.create_subscription(MarkerArray, '/goal_points', self.goal_points_received, 10)

        # State
        self._goal_index = 0
        self._goals = []
        self._map_resolution = None
        self._origin_x = None
        self._origin_y = None
        self._map = None
        self._path = None
        self._path_index = 0
        self._robot_grid_state = None
        self._goal_grid_state = None
        self._stop_event: Event = Event()

        self._viz_thread: Thread = Thread(target=self.visualization_loop, daemon=True)
        self._viz_thread.start()

        # TF frames
        self._to_frame = 'maze_world'
        self._from_frame = 'diff_drive/body_link'

        # Timer loop
        self.timer = self.create_timer(self._dt, self.plan_to_goals)

        self.get_logger().info(f'AgentMotionPlanner started.')

    def stop(self):
        self._stop_event.set()

    def visualization_loop(self):
        ion()
        fig, ax = subplots(figsize=(6, 6))

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
            for gx, gy in self._goals:
                r, c = self.world_to_grid(gx, gy)
                ax.plot(c, r, 'gs', markersize=8, label='Goal')

            # Draw short-term goal
            if self._goal_grid_state is not None:
                ax.plot(self._goal_grid_state[1], self._goal_grid_state[0], 'yo', label='Interim Goal')

            ax.set_title("Live Map Visualization")
            ax.set_xlabel("grid X")
            ax.set_ylabel("grid Y")
            ax.legend(loc='upper right')

            pause(0.1)
        
        try:
            plt_close(fig)
        except Exception:
            pass

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
            x = marker.pose.position.x
            y = marker.pose.position.y
            self._goals.append((x, y))

    def send_goal_pose(self, position: Tuple[float, float]) -> None:
        msg: PoseStamped = PoseStamped()
        msg.header.frame_id = self._to_frame
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
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
        goal_state: Tuple[float, float] = self._goals[self._goal_index]
        self._robot_grid_state = self.world_to_grid(robot_state[0], robot_state[1])
        self._goal_grid_state = self.world_to_grid(goal_state[0], goal_state[1])

        if euclidean_distance(robot_state, goal_state) <= self._goal_tolerance:
            # Goal acheived!
            self.get_logger().info(f'Successfully reached goal #{self._goal_index+1} at ({goal_state[0]}, {goal_state[1]})')

            self._goal_index += 1
            self._path_index = 0
            self._path = None
        else:

            # Plan from robot state to goal state
            if self._path is None:
                self.get_logger().info(f'Calculating optimal path')
                self._path: List[Tuple[int, int]] = a_star_grid(self._map, self._robot_grid_state, self._goal_grid_state)
                self.get_logger().info(f'Path found: {self._path}')

            # If we have a path, let's walk it.
            if self._path_index < len(self._path):
                intermediate_goal_grid: Tuple[int, int] = self._path[self._path_index]
                intermediate_goal: Tuple[float, float] = self.grid_to_world(intermediate_goal_grid[0], intermediate_goal_grid[1])
                # Shift the intermediate goal by the trailer hitch
                trailer_hitch_goal: Tuple[float, float] = self.world_shift_trailer_hitch(intermediate_goal[0], intermediate_goal[1], yaw)

                # Check if at intermediate goal:
                if euclidean_distance(robot_state, intermediate_goal) <= self._intermediate_goal_tolerance:
                    self._path_index += 1
                else:
                    self.send_goal_pose(trailer_hitch_goal)


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

