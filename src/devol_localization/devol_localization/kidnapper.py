#!/usr/bin/env python3
"""Kidnaps the robot: teleports it in Gazebo mid-run without telling the estimators.

trigger:=time kidnaps kidnap_time seconds of simulated time after the node sees the clock start.
trigger:=after_waypoint kidnaps kidnap_time seconds after the ground truth first comes within
waypoint_radius of `waypoint` (default: after Goal 1, while the robot drives toward Goal 2). The
default target is Goal 2's exact pose, so the planner, which does not replan when the robot is
moved off its path, counts Goal 2 as reached and plans Goal 3 from there. To kidnap, the node calls
Gazebo's /world/<world>/set_pose service (UserCommands system, present in the factory world) with
the `gz service` command-line tool, and publishes the target on event_topic so a recorded bag
carries the event. Wheel odometry does not see the jump, so every estimator is left with a pose
far from the truth, which is the outline's kidnapped-robot test. The evaluator finds the jump in
the ground truth itself, so the event topic is informational.
"""

import math
import shutil
import subprocess
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from devol_localization.pose2d import set_quaternion_yaw

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


def set_pose_command(world: str, entity: str, x: float, y: float, z: float, yaw: float, timeout_ms: int = 3000):
    req = (f'name: "{entity}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2.0)}, w: {math.cos(yaw / 2.0)}}}')
    return ['gz', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms), '--req', req]


class Kidnapper(Node):
    def __init__(self) -> None:
        super().__init__('kidnapper')
        self.declare_parameter('trigger', 'time')             # time | after_waypoint
        self.declare_parameter('kidnap_time', 30.0)
        self.declare_parameter('waypoint', [-4.7, 9.2])        # Goal 1
        self.declare_parameter('waypoint_radius', 0.5)
        self.declare_parameter('ground_truth_topic', '/devol_drive/ground_truth/odom')
        self.declare_parameter('target', [5.45, 2.03, 0.0])   # x, y, yaw in the world (= map) frame; Goal 2
        self.declare_parameter('z', 0.2)
        self.declare_parameter('world', 'maze_world')
        self.declare_parameter('entity', 'devol_drive')
        self.declare_parameter('event_topic', '/devol_drive/kidnap_event')
        gp = self.get_parameter
        self._delay = float(gp('kidnap_time').value)
        self._trigger = str(gp('trigger').value)
        if self._trigger not in ('time', 'after_waypoint'):
            raise ValueError(f'trigger must be time or after_waypoint, got {self._trigger}')
        self._waypoint = [float(v) for v in gp('waypoint').value]
        self._waypoint_radius = float(gp('waypoint_radius').value)
        if self._trigger == 'after_waypoint':
            self.create_subscription(Odometry, gp('ground_truth_topic').value, self._gt_cb, 10)
        self._target = [float(v) for v in gp('target').value]
        self._z = float(gp('z').value)
        self._world = str(gp('world').value)
        self._entity = str(gp('entity').value)
        self._pub = self.create_publisher(PoseStamped, gp('event_topic').value, 1)
        self._t0: Optional[float] = None
        self._done = False
        self._timer = self.create_timer(0.1, self._tick)
        if shutil.which('gz') is None:
            self.get_logger().error('`gz` command not found; the kidnap cannot run')
        when = (f'{self._delay:.1f} s after reaching {self._waypoint}' if self._trigger == 'after_waypoint'
                else f't+{self._delay:.1f} s')
        self.get_logger().info(f'Kidnap to {self._target} {when} (sim time)')

    def _gt_cb(self, msg: Odometry) -> None:
        if self._t0 is not None:
            return
        p = msg.pose.pose.position
        if math.hypot(p.x - self._waypoint[0], p.y - self._waypoint[1]) <= self._waypoint_radius:
            self._t0 = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(f'Reached {self._waypoint}; kidnapping in {self._delay:.1f} s')

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0 or self._done:
            return
        if self._t0 is None:
            if self._trigger == 'after_waypoint':
                return
            self._t0 = now
        if now - self._t0 < self._delay:
            return
        self._done = True
        self._timer.cancel()
        x, y, yaw = self._target
        cmd = set_pose_command(self._world, self._entity, x, y, self._z, yaw)
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=10.0)
            ok = out.returncode == 0 and 'true' in out.stdout
            log = self.get_logger().info if ok else self.get_logger().error
            log(f'Teleported {self._entity} to ({x:.2f}, {y:.2f}, {yaw:.2f}) at t={now:.2f} s: '
                f'{out.stdout.strip() or out.stderr.strip()}')
        except (OSError, subprocess.TimeoutExpired) as e:
            self.get_logger().error(f'Teleport failed: {e}')
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        set_quaternion_yaw(msg.pose.orientation, yaw)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Kidnapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
