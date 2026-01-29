#!/usr/bin/env python3
from typing import Dict, List, Tuple
from math import  cos, sin
from time import sleep
from threading import Thread
from os.path import join

from rclpy import init as rclpy_init, shutdown as rclpy_shutdown, ok as rclpy_ok
from rclpy.node import Node, Service
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, Quaternion
from devol_msgs.srv import CheckCollision
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from moveit.core.planning_scene import PlanningScene
from moveit.core.collision_detection import CollisionRequest, CollisionResult


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Convert a yaw angle (around Z) to a geometry_msgs Quaternion."""
    q = Quaternion()
    q.w = cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = sin(yaw / 2.0)
    return q

class MultiRobotCollisionServer(Node):
    def __init__(self, robot_names: List[str]):
        super().__init__("multi_robot_collision_checker")

        self._robot_names = robot_names
        self._robot_models = {}
        for robot_name in self._robot_names:
            moveit_config_directory: str = join('/workspace', 'arc_lsi', 'src', 'devol', f'{robot_name}_moveit_config', 'config')
            urdf_path: str = join(moveit_config_directory, f'{robot_name}.urdf')
            srdf_path: str = join(moveit_config_directory, f'{robot_name}.srdf')
            self._robot_models[robot_name] = RobotModel(
                urdf_path, srdf_path
            )

        self.scene = PlanningScene(self._robot_models[self._robot_names[0]])

        self._server: Service = self.create_service(
            CheckCollision,
            'check_collision',
            self.handle_check_collision
        )


    def handle_check_collision(self, request, response): 
        robot_states: dict[str, Tuple[List[float], List[float]]] = {}

        for robot_index, robot_name in enumerate(request.robot_names):
            robot_states[robot_name] = (
                request.base_poses[robot_index], request
            )

        response.collision = self.check_collision(robot_states)
        return response

    def check_collision(
        self, robot_states: Dict[str, Tuple[List[float], List[float]]]
    ) -> bool:
        """
        robot_states: Dict[robot_name, (base_pose, joint_values)]
            base_pose = [x0, y0, yaw0]
            joint_values = [j00, j01, ...]
        """
        if len(robot_states) != len(self._robot_models):
            return False

        # Update all robots in the planning scene
        for name, (base, joints) in robot_states.items():
            state = RobotState(self._robot_models[name])
            state.setToDefaultValues()
            # Set joint values
            joint_names = state.getVariableNames()
            for i, val in enumerate(joints):
                state.setVariablePosition(joint_names[i], val)

            # Create Pose for the robot base
            pose = Pose()
            pose.position.x = base[0]
            pose.position.y = base[1]
            # Convert yaw to quaternion
            q = quaternion_from_yaw(base[2])
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            # Add/update robot in the scene with its base transform
            self.scene.setCurrentState(state, name, pose)

        # Perform collision check
        request = CollisionRequest()
        result = CollisionResult()
        self.scene.checkCollision(request, result)
        return result.collision


def main():
    rclpy_init()

    try:
        # Create the demo node
        robot_names = ["apollo", "artemis"]
        node = MultiRobotCollisionServer(robot_names)

        # Create Executor
        executor: MultiThreadedExecutor = MultiThreadedExecutor()
        executor.add_node(node)

        # Start ROS2 in a seperate thread
        spin_thread: Thread = Thread(
            target=executor.spin, daemon=True
        )
        spin_thread.start()

        try:
            while rclpy_ok():
                sleep(0.1)
        except KeyboardInterrupt:
            pass

        node.destroy_node()
        rclpy_shutdown()
        spin_thread.join()
    except Exception as e:
        print(f"Fatal error: {e}")

    return 0    


if __name__ == "__main__":
    main()
