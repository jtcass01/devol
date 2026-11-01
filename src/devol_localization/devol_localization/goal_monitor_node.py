#!/usr/bin/env python3

import sys
from rclpy import init, spin, try_shutdown
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from math import hypot

class GoalMonitorNode(Node):
    def __init__(self):
        super().__init__('goal_monitor_node')

        # Declare parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'a200_base_link')
        self.declare_parameter('goal_x', 1.87) 
        self.declare_parameter('goal_y', -8.06) 
        self.declare_parameter('tolerance', 0.5)

        # TF tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer
        self.timer = self.create_timer(0.5, self.check_goal)

    def check_goal(self):
        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value

        try:
            trans = self.tf_buffer.lookup_transform(map_frame, base_frame, Time())
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y
        except Exception as e:
            self.get_logger().warn(
                f'Waiting for TF ({map_frame} to {base_frame}): {str(e)}',
                throttle_duration_sec=5.0,
            )
            return

        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value
        tolerance = self.get_parameter('tolerance').value

        dist = hypot(robot_x - goal_x, robot_y - goal_y)
        if dist <= tolerance:
            self.get_logger().info(f'Goal 3 reached (dist: {dist:.2f}m). Terminating test run.')
            # Exit the node cleanly, triggering the OnProcessExit handler in the launch file
            sys.exit(0)

def main(args=None):
    init(args=args)
    node = GoalMonitorNode()
    try:
        spin(node)
    except SystemExit:
        pass  # Expected exit when goal is reached
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try_shutdown()

if __name__ == '__main__':
    main()