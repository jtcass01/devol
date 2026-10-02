#!/usr/bin/env python3
"""Kidnaps the robot: teleports it in Gazebo mid-run without telling the estimators.

At kidnap_time seconds of simulated time after the node sees the clock start, the node calls
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
        self.declare_parameter('kidnap_time', 30.0)
        self.declare_parameter('target', [5.45, 2.03, 0.0])   # x, y, yaw in the world (= map) frame
        self.declare_parameter('z', 0.2)
        self.declare_parameter('world', 'maze_world')
        self.declare_parameter('entity', 'devol_drive')
        self.declare_parameter('event_topic', '/devol_drive/kidnap_event')
        gp = self.get_parameter
        self._delay = float(gp('kidnap_time').value)
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
        self.get_logger().info(f'Kidnap to {self._target} at t+{self._delay:.1f} s (sim time)')

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0 or self._done:
            return
        if self._t0 is None:
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
