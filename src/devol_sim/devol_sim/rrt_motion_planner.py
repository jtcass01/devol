#!/usr/bin/env python3
"""RRT / RRT* drop-in replacement for agent_motion_planner.

Same interfaces as agent_motion_planner (goal_points in, goal_pose out, robot
pose from TF). Instead of a point robot on the padded C-space map, it collision
checks the robot's body against octomap_server's map:

* map_source 'octomap_3d' (default): the occupied voxels from
  occupied_cells_vis_array, checked against a stack of body boxes (base, arm),
  each against the voxels in its own height band.
* map_source 'projected_map': the 2D projected_map, checked against one
  rectangular footprint.
"""
from __future__ import annotations

from math import atan2, hypot
from threading import Event, Lock, Thread
from typing import List, Optional, Tuple

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as RCLPY_Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from devol_sim.mobile_robot_goal import MobileRobotGoal
from devol_sim.rrt_planner import (DEFAULT_BODY, FootprintCollisionChecker, GoalInCollision,
                                   MultiFootprintChecker, PlannerConfig, PlanResult, RRTPlanner,
                                   StartInCollision, body_checker_from_voxels, body_from_flat,
                                   densify, distance_to_segment, wrap_to_pi)
from devol_sim.utils import euler_to_quaternion, quaternion_to_euler

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


class RRTMotionPlanner(RCLPY_Node):
    def __init__(self):
        super().__init__('rrt_motion_planner')

        # Same parameters as agent_motion_planner
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('viz_rate', 0.0)
        self.declare_parameter('namespace', '/devol_drive')
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('lookahead', 0.15)
        self.declare_parameter('intermediate_goal_tolerance', 0.15)
        self.declare_parameter('tf_to_frame', 'map')
        self.declare_parameter('tf_from_frame', 'a200_base_link')
        # Map source
        self.declare_parameter('map_source', 'octomap_3d')
        # 3D: robot body as boxes, 6 values each (x_min x_max y_min y_max z_min z_max)
        # in base_footprint with z = 0 on the ground. Empty = DEFAULT_BODY.
        self.declare_parameter('voxel_topic', 'occupied_cells_vis_array')
        self.declare_parameter('body_boxes', [0.0])
        self.declare_parameter('ground_z', 0.0)
        # 2D: footprint (A200 overall size from a200_description: chassis
        # 0.9874 m long, wheels at +-0.2775 m and 0.1 m wide)
        self.declare_parameter('map_topic', 'projected_map')
        self.declare_parameter('unknown_is_occupied', True)
        self.declare_parameter('footprint_length', 0.99)
        self.declare_parameter('footprint_width', 0.67)
        self.declare_parameter('footprint_padding', 0.05)
        self.declare_parameter('footprint_offset', 0.0)
        # Planner
        self.declare_parameter('algorithm', 'rrt_star')
        self.declare_parameter('step_size', 1.0)
        self.declare_parameter('goal_bias', 0.1)
        self.declare_parameter('max_iterations', 20000)
        self.declare_parameter('max_planning_time', 2.0)
        self.declare_parameter('patience', 1500)
        self.declare_parameter('shortcut_attempts', 200)
        self.declare_parameter('require_goal_yaw', True)
        self.declare_parameter('waypoint_spacing', 0.5)
        self.declare_parameter('goal_approach_distance', 1.0)
        self.declare_parameter('corner_tolerance', 0.1)
        self.declare_parameter('turn_in_place_threshold', 0.35)
        # Re-plan from the current pose when the robot is this far from the segment it is
        # driving (e.g. after a kidnap teleport). <= 0 disables re-planning.
        self.declare_parameter('replan_distance', 1.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._publish_rate = float(p('publish_rate'))
        self._viz_rate = float(p('viz_rate'))
        self._namespace = str(p('namespace'))
        self._goal_tolerance = float(p('goal_tolerance'))
        self._lookahead = float(p('lookahead'))
        self._intermediate_goal_tolerance = float(p('intermediate_goal_tolerance'))
        self._to_frame = str(p('tf_to_frame'))
        self._from_frame = f'{self._namespace[1:]}/{p("tf_from_frame")}'
        self._map_source = str(p('map_source'))
        if self._map_source not in ('octomap_3d', 'projected_map'):
            raise ValueError(f"map_source must be 'octomap_3d' or 'projected_map', got {self._map_source!r}")
        boxes = [float(v) for v in p('body_boxes')]
        self._body = body_from_flat(boxes) if len(boxes) > 1 else DEFAULT_BODY
        self._ground_z = float(p('ground_z'))
        self._unknown_is_occupied = bool(p('unknown_is_occupied'))
        self._padding = float(p('footprint_padding'))
        self._footprint = dict(length=float(p('footprint_length')),
                               width=float(p('footprint_width')),
                               padding=self._padding,
                               offset=(float(p('footprint_offset')), 0.0))
        self._config = PlannerConfig(algorithm=str(p('algorithm')),
                                     step_size=float(p('step_size')),
                                     goal_bias=float(p('goal_bias')),
                                     goal_tolerance=float(p('step_size')),
                                     max_iterations=int(p('max_iterations')),
                                     max_planning_time=float(p('max_planning_time')),
                                     patience=int(p('patience')),
                                     shortcut_attempts=int(p('shortcut_attempts')),
                                     require_goal_yaw=bool(p('require_goal_yaw')),
                                     goal_approach_distance=float(p('goal_approach_distance')))
        self._waypoint_spacing = float(p('waypoint_spacing'))
        self._corner_tolerance = float(p('corner_tolerance'))
        self._turn_in_place_threshold = float(p('turn_in_place_threshold'))
        self._replan_distance = float(p('replan_distance'))

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # I/O
        qos_profile = QoSProfile(depth=10)
        map_topic = str(p('voxel_topic') if self._map_source == 'octomap_3d' else p('map_topic'))
        if not map_topic.startswith('/'):
            map_topic = f'{self._namespace}/{map_topic}'
        self._goal_pub = self.create_publisher(PoseStamped, f'{self._namespace}/goal_pose', qos_profile)
        self._path_pub = self.create_publisher(Path, f'{self._namespace}/rrt_path', qos_profile)
        if self._map_source == 'octomap_3d':
            self._map_sub = self.create_subscription(MarkerArray, map_topic, self.voxels_received, 10)
        else:
            self._map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_received, 10)
        self._goals_sub = self.create_subscription(MarkerArray, f'{self._namespace}/goal_points',
                                                   self.goal_points_received, 10)

        # State
        self._lock = Lock()
        self._goal_index = 0
        self._goals: List[MobileRobotGoal] = []
        self._map_key: Optional[Tuple] = None
        self._checker: Optional[FootprintCollisionChecker] = None
        self._path: Optional[List[Tuple[float, float, float]]] = None
        self._path_index = 0
        self._last_result: Optional[PlanResult] = None
        self._robot_pose: Optional[Tuple[float, float, float]] = None
        self._stop_event = Event()

        self._viz_thread: Optional[Thread] = None
        if self._viz_rate > 0.0:
            self._viz_thread = Thread(target=self.visualization_loop, daemon=True)
            self._viz_thread.start()

        self.timer = self.create_timer(1.0 / self._publish_rate, self.plan_to_goals)
        self.get_logger().info(f'RRTMotionPlanner ({self._config.algorithm}, {self._map_source}) started, '
                               f'map: {map_topic}')

    def stop(self):
        self._stop_event.set()
        if self._viz_thread is not None:
            self._viz_thread.join()

    # Callbacks
    def voxels_received(self, msg: MarkerArray) -> None:
        """octomap_server's occupied_cells_vis_array: one CUBE_LIST marker per
        tree depth, scale = leaf size at that depth, points = leaf centres."""
        markers = [m for m in msg.markers if m.action == Marker.ADD and len(m.points) > 0]
        if not markers:
            return
        # The map is republished with every cloud insertion; skip unchanged ones
        # cheaply (converting every point is the expensive part).
        key = tuple((round(m.scale.x, 6), len(m.points),
                     m.points[0].x, m.points[0].y, m.points[0].z,
                     m.points[-1].x, m.points[-1].y, m.points[-1].z) for m in markers)
        if key == self._map_key:
            return
        centers = np.concatenate([np.array([(q.x, q.y, q.z) for q in m.points]) for m in markers])
        sizes = np.concatenate([np.full(len(m.points), m.scale.x) for m in markers])
        resolution = float(min(m.scale.x for m in markers))
        checker = body_checker_from_voxels(centers, sizes, resolution, self._body,
                                           ground_z=self._ground_z, padding=self._padding)
        with self._lock:
            self._checker = checker
            self._map_key = key
        part = checker.parts[0]
        self.get_logger().info(f'3D map updated: {len(centers)} occupied voxels, '
                               f'{part.n_cols}x{part.n_rows} @ {resolution:.3f} m, {len(checker.parts)} body boxes')

    def map_received(self, msg: OccupancyGrid) -> None:
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int8)
        key = (info.width, info.height, info.resolution,
               info.origin.position.x, info.origin.position.y, hash(data.tobytes()))
        if key == self._map_key:
            return  # octomap_server republishes the same projection with every cloud
        grid = data.reshape((info.height, info.width))
        checker = FootprintCollisionChecker(grid, info.resolution,
                                            (info.origin.position.x, info.origin.position.y),
                                            occupied_threshold=50,
                                            unknown_is_occupied=self._unknown_is_occupied,
                                            **self._footprint)
        with self._lock:
            self._checker = checker
            self._map_key = key
        self.get_logger().info(f'Map updated: {info.width}x{info.height} @ {info.resolution:.3f} m')

    def goal_points_received(self, msg: MarkerArray) -> None:
        if len(self._goals) > 0:
            return
        for marker in reversed(msg.markers):
            marker: Marker = marker
            _, _, yaw = quaternion_to_euler(marker.pose.orientation)
            goal = MobileRobotGoal(name=marker.text,
                                   x=marker.pose.position.x,
                                   y=marker.pose.position.y,
                                   z=marker.pose.position.z,
                                   yaw=yaw)
            self.get_logger().info(f'Received {goal}')
            self._goals.append(goal)

    # Helpers
    def send_goal_pose(self, position: Tuple[float, float], yaw: float) -> None:
        msg = PoseStamped()
        msg.header.frame_id = self._to_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        q = euler_to_quaternion(0.0, 0.0, yaw)
        msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        self._goal_pub.publish(msg)

    def publish_path(self, path: List[Tuple[float, float, float]]) -> None:
        msg = Path()
        msg.header.frame_id = self._to_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            q = euler_to_quaternion(0.0, 0.0, yaw)
            pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
            pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
            msg.poses.append(pose)
        self._path_pub.publish(msg)

    def world_shift_trailer_hitch(self, x, y, theta):
        return x + self._lookahead * np.cos(theta), y + self._lookahead * np.sin(theta)

    def lookup_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            trans = self._tf_buffer.lookup_transform(self._to_frame, self._from_frame,
                                                     rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except tf2_ros.LookupException:
            self.get_logger().warning('Transform isn\'t available, waiting...', throttle_duration_sec=5.0)
            return None
        except Exception as e:
            self.get_logger().warning(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return None
        _, _, yaw = quaternion_to_euler(trans.transform.rotation)
        return trans.transform.translation.x, trans.transform.translation.y, float(yaw)

    def plan(self, start, goal: MobileRobotGoal) -> Optional[List[Tuple[float, float, float]]]:
        with self._lock:
            checker = self._checker
        planner = RRTPlanner(checker, self._config)
        try:
            result = planner.plan(start, (goal.x, goal.y, goal.yaw))
        except (StartInCollision, GoalInCollision) as e:
            self.get_logger().warning(f'Cannot plan to {goal.name}: {e}', throttle_duration_sec=5.0)
            return None
        self._last_result = result
        if result.freed_start_cells:
            self.get_logger().info(f'Treated {result.freed_start_cells} unknown map cells under the robot as free')
        if not result.success:
            self.get_logger().warning(
                f'No path to {goal.name} after {result.iterations} iterations '
                f'({result.planning_time:.2f} s), retrying', throttle_duration_sec=5.0)
            return None
        self.get_logger().info(
            f'{self._config.algorithm} path to {goal.name}: {result.cost:.2f} m, {len(result.path)} waypoints, '
            f'{result.nodes} nodes, first solution {result.first_solution_time:.2f} s, '
            f'total {result.planning_time:.2f} s')
        return densify(result.path, self._waypoint_spacing)

    # Main loop
    def plan_to_goals(self):
        if self._checker is None:
            return
        if len(self._goals) == 0 or self._goal_index >= len(self._goals):
            return

        pose = self.lookup_robot_pose()
        if pose is None:
            return
        self._robot_pose = pose
        x, y, yaw = pose
        goal = self._goals[self._goal_index]

        if hypot(goal.x - x, goal.y - y) <= self._goal_tolerance:
            self.get_logger().info(f'Successfully reached goal: {goal.name}')
            self._goal_index += 1
            self._path_index = 0
            self._path = None
            return

        if self._path is None:
            self.get_logger().info(f'Planning path to {goal.name}')
            self._path = self.plan(pose, goal)
            if self._path is None:
                return
            # The first waypoint is the robot's own position.
            self._path_index = 1
            self.publish_path(self._path)

        if self._replan_distance > 0.0 and self._path_index < len(self._path):
            (px, py, _), (wx, wy, _) = self._path[self._path_index - 1], self._path[self._path_index]
            off_path = distance_to_segment(x, y, px, py, wx, wy)
            if off_path > self._replan_distance:
                self.get_logger().warning(f'{off_path:.2f} m off the path to {goal.name}; re-planning')
                self._path = None
                return

        if self._path_index < len(self._path):
            wx, wy, _ = self._path[self._path_index]
            is_last = self._path_index == len(self._path) - 1
            if is_last:
                tolerance = self._goal_tolerance
            elif self.is_corner(self._path_index):
                tolerance = self._corner_tolerance
            else:
                tolerance = self._intermediate_goal_tolerance
            dist = hypot(wx - x, wy - y)
            if dist <= tolerance and not is_last:
                self._path_index += 1
                return
            # The plan turns in place at corners; do the same before driving off,
            # otherwise the follower swings wide or cuts the corner.
            heading = atan2(wy - y, wx - x)
            if dist > self._intermediate_goal_tolerance and \
                    abs(wrap_to_pi(heading - yaw)) > self._turn_in_place_threshold:
                self.send_goal_pose(self.world_shift_trailer_hitch(x, y, yaw), heading)
            else:
                # Hold the heading of the segment being driven. Asking for the next
                # segment's heading here makes diffdrive_pid's yaw term cancel its
                # steering term short of the waypoint (the robot stalls).
                px, py, _ = self._path[self._path_index - 1]
                self.send_goal_pose(self.world_shift_trailer_hitch(wx, wy, yaw), atan2(wy - py, wx - px))

    def is_corner(self, i: int) -> bool:
        """Does the path turn by more than turn_in_place_threshold at waypoint i?"""
        if i <= 0 or i >= len(self._path) - 1:
            return False
        (x0, y0, _), (x1, y1, _), (x2, y2, _) = self._path[i - 1:i + 2]
        turn = wrap_to_pi(atan2(y2 - y1, x2 - x1) - atan2(y1 - y0, x1 - x0))
        return abs(turn) > self._turn_in_place_threshold

    def visualization_loop(self):
        from matplotlib.patches import Polygon
        from matplotlib.pyplot import close as plt_close, ion, pause, subplots
        ion()
        fig, ax = subplots(figsize=(6, 8))
        while not self._stop_event.is_set():
            checker = self._checker
            if checker is None:
                pause(0.1)
                continue
            ax.clear()
            parts = checker.parts if isinstance(checker, MultiFootprintChecker) else [checker]
            (x0, x1), (y0, y1) = checker.x_bounds, checker.y_bounds
            # Darker = blocks a lower body box (the base); lighter = only the upper boxes.
            shade = np.zeros(parts[0].occupied.shape)
            for k, part in reversed(list(enumerate(parts))):
                shade[part.occupied] = 1.0 - 0.5 * k / max(1, len(parts) - 1)
            ax.imshow(shade, cmap='gray_r', origin='lower', extent=(x0, x1, y0, y1), vmin=0, vmax=1)
            result = self._last_result
            if result is not None and result.tree is not None:
                nodes, parents = result.tree
                for i in range(1, len(nodes)):
                    j = parents[i]
                    ax.plot([nodes[j, 0], nodes[i, 0]], [nodes[j, 1], nodes[i, 1]], color='0.7', lw=0.4)
            if self._path is not None:
                path = np.array(self._path)
                ax.plot(path[:, 0], path[:, 1], 'b.-', label='Path')
            if self._robot_pose is not None:
                rx, ry, ryaw = self._robot_pose
                c, s = np.cos(ryaw), np.sin(ryaw)
                for part in parts:
                    hl, hw = part.half_length, part.half_width
                    corners = [(part.offset_x + a * hl, part.offset_y + b * hw)
                               for a, b in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
                    ax.add_patch(Polygon([(rx + c * u - s * v, ry + s * u + c * v) for u, v in corners],
                                         fill=False, color='r'))
            for goal in self._goals:
                ax.plot(goal.x, goal.y, 'gs', markersize=8)
            ax.set_title(f'{self._config.algorithm} planner')
            ax.set_xlabel('x (m)')
            ax.set_ylabel('y (m)')
            pause(1.0 / self._viz_rate)
        plt_close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = RRTMotionPlanner()
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
