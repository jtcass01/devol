#!/usr/bin/env python3
from __future__ import annotations
from typing import Tuple, Set
from threading import Thread, Event, Lock
from numpy import array

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as RCLPY_Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
import tf2_ros
from matplotlib.pyplot import (ion, 
                               close as plt_close, 
                               figure as plt_figure, 
                               pause as plt_pause)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points 

from devol_sim.a_star_planner import a_star_grid
from devol_sim.utils import quaternion_to_euler

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


def euclidean_distance(p1: Tuple[int, int, int], p2: Tuple[int, int, int]) -> float:
    assert len(p1) == 3
    assert len(p2) == 3
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    return ((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)**0.5


class AgentMotionPlanner3D(RCLPY_Node):
    """
    """
    def __init__(self):
        super().__init__('agent_motion_planner_3d')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('viz_rate', 2.0)
        self.declare_parameter('namespace', '/devol_drive')
        self.declare_parameter('octomap_resolution', 0.05)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('lookahead', 0.3)
        self.declare_parameter('intermediate_goal_tolerance', 0.15)
        self.declare_parameter('tf_to_frame', 'map')
        self.declare_parameter('tf_from_frame', 'a200_base_link')

        self._publish_rate: float = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self._viz_rate: float = float(self.get_parameter('viz_rate').get_parameter_value().double_value)
        self._namespace: str = str(self.get_parameter('namespace').get_parameter_value().string_value)
        self._octomap_resolution: float = float(self.get_parameter('octomap_resolution').get_parameter_value().double_value)
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
        self._goal_pub = self.create_publisher(PoseStamped, f'/goal_pose', qos_profile)
        self._point_cloud_center_sub = self.create_subscription(PointCloud2, 
                                                                f'{self._namespace}/octomap_point_cloud_centers', 
                                                                self._cloud_received, 10)
        self._goals_sub = self.create_subscription(MarkerArray, f'{self._namespace}/goal_points', self._goal_points_received, 10)

        # State
        self._occupied_cells: Set[Tuple[float, float, float]] = set()
        self._occupied_cells_lock: Lock = Lock()
        self._goals_lock: Lock = Lock()
        self._robot_state_lock: Lock = Lock()
        self._goal_index = 0
        self._goals = []
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._origin_z = 0.0
        self._path = None
        self._path_index = 0
        self._robot_state = None
        self._goal_state = None
        self._stop_event: Event = Event()

        self._fig = None
        self._ax = None
        if self._viz_rate > 0.0:
            self._viz_thread: Thread = Thread(target=self.visualization_loop, daemon=True)
            self._viz_thread.start()

        # Timer loop
        self.timer = self.create_timer(self._dt, self.plan_to_goals)

        self.get_logger().info(f'AgentMotionPlanner3D started.')

    def stop(self):
        self._stop_event.set()
        if self._fig is not None:
            plt_close(self._fig)

    def visualization_loop(self):
        ion()
        self._fig = plt_figure(figsize=(10, 8))
        self._ax = self._fig.add_subplot(111, projection='3d')
        self.get_logger().info('Visualization loop started')
        update_period: float = 1.0 / self._viz_rate

        while not self._stop_event.is_set():
            try:
                # Clear previous plots
                self._ax.clear()

                # Copy occipied cells (thread-safe)
                with self._occupied_cells_lock:
                    occupied_cells = self._occupied_cells.copy()

                with self._goals_lock:
                    goals = array(self._goals.copy())

                with self._robot_state_lock:
                    robot_state = self._robot_state

                if len(occupied_cells) > 0:
                    cells_array = array(list(occupied_cells))

                    min_bound = cells_array.min(axis=0)
                    max_bound = cells_array.max(axis=0)
                    
                    # Plot occupied cells
                    self._ax.scatter(cells_array[:, 0], 
                                     cells_array[:, 1], 
                                     cells_array[:, 2],
                                     c='red', marker='s', s=1, alpha=0.01)
                    
                    # Plot goal Positions
                    if len(goals) > 0:
                        self._ax.scatter(goals[:, 0],
                                         goals[:, 1],
                                         goals[:, 2],
                                         c='green', marker='o', s=1, alpha=1.0,
                                         edgecolors='darkgreen', linewidths=2,
                                         label='Goals')
                        
                    if robot_state is not None:
                        self._ax.scatter(robot_state[0],
                                         robot_state[1],
                                         robot_state[2],
                                         c='blue', marker='^', s=200, alpha=1.0,
                                         edgecolors='darkblue', linewidths=2,
                                         label='Robot')

                    # Set labels
                    self._ax.set_xlabel('X (m)')
                    self._ax.set_ylabel('Y (m)')
                    self._ax.set_zlabel('Z (m)')
                    self._ax.set_title(f'Octomap Visualization\n{len(occupied_cells)} occupied cells')

                    # Set limits
                    self._ax.set_xlim(min_bound[0], max_bound[0])
                    self._ax.set_ylim(min_bound[1], max_bound[1])
                    self._ax.set_zlim(min_bound[2], max_bound[2])
                else:
                    # No data yet
                    self._ax.text(0.5, 0.5, 0.5, 'Waiting for octomap data...', 
                                horizontalalignment='center',
                                verticalalignment='center',
                                transform=self._ax.transAxes)
                    self._ax.set_xlabel('X (m)')
                    self._ax.set_ylabel('Y (m)')
                    self._ax.set_zlabel('Z (m)')

                # Draw and pause
                self._fig.canvas.draw_idle()
                self._fig.canvas.flush_events()
                plt_pause(update_period)
            except Exception as e:
                self.get_logger().error(f'Visualization error: {e}')
                self._stop_event.wait(update_period)

        self.get_logger().info('Visualization loop stopped.')
        plt_close(self._fig)

    def _cloud_received(self, msg: PointCloud2) -> None:
        with self._occupied_cells_lock:
            self._occupied_cells.clear()
            points = read_points(msg, skip_nans=True, field_names=['x', 'y', 'z'])

            for point in points:
                x, y, z = point
                cell = self._discretize(x=x, y=y, z=z)
                self._occupied_cells.add(cell)

    def _is_free(self, cell):
        with self._occupied_cells_lock:
            return cell not in self._occupied_cells

    def _discretize(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        return (
            round(x / self._octomap_resolution) * self._octomap_resolution,
            round(y / self._octomap_resolution) * self._octomap_resolution,
            round(z / self._octomap_resolution) * self._octomap_resolution
        )

    def _world_to_grid(self, x: float, y: float, z: float) -> int:
        col = int(round((x - self._origin_x) / self._octomap_resolution))
        row = int(round((y - self._origin_y) / self._octomap_resolution))
        height = int(round((z - self._origin_z) / self._octomap_resolution))
        return row, col, height

    def _goal_points_received(self, msg: MarkerArray) -> None:
        # Make a goal for each marker
        with self._goals_lock:
            # If there are already goals, don't accept more
            if len(self._goals) > 0:
                return

            for marker in reversed(msg.markers):
                x = marker.pose.position.x
                y = marker.pose.position.y
                z = marker.pose.position.z

                goal_cell = self._discretize(x, y, z)
                self._goals.append(goal_cell)

    def send_goal_pose(self, position: Tuple[float, float, float]) -> None:
        msg: PoseStamped = PoseStamped()
        msg.header.frame_id = self._to_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)

    def _grid_to_world(self, row: int, col: int, height: int) -> Tuple[float, float]:
        x = self._origin_x + (col+0.5) * self._octomap_resolution
        y = self._origin_y + (row+0.5) * self._octomap_resolution
        z = self._origin_z + (height+0.5) * self._octomap_resolution
        return x, y, z
    
    def plan_to_goals(self):
        with self._occupied_cells_lock:
            if len(self._occupied_cells) == 0:
                return
        
        if (len(self._goals) == 0) or (self._goal_index >= len(self._goals)):
            return
        
        # Get robot's curent position from tf, it would be better if there was a particle filter on the other side of this.
        try:
            trans = self._tf_buffer.lookup_transform(
                self._to_frame, self._from_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
            with self._robot_state_lock:
                self._robot_state = (
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                )

        except tf2_ros.LookupException:
            self.get_logger().warning('Transform isn\'t available, waiting...')
            return
        except Exception as e:
            self.get_logger().warning(f'TF lookup failed: {str(e)}')
            return
        
        # Convert robot position and goal to grid indices
        try:
            robot_grid = self._world_to_grid(*self._robot_state)

            with self._goals_lock:
                goal_world = self._goals[self._goal_index]

            goal_grid = self._world_to_grid(*goal_world)
            self.get_logger().info(f'Planning from robot {robot_grid} to goal {goal_grid}')
        except ValueError as e:
            self.get_logger().error(f'Grid conversion error: {str(e)}')
            return


def main(args=None):
    rclpy.init(args=args)
    node = AgentMotionPlanner3D()

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

