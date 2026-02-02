#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy
from typing import List, Union, Dict

# ROS2 message imports
from geometry_msgs.msg import Pose, Twist

# Import your DevolRobot class
from devol_demo_py.devol_robot import DevolRobot, GRIPPER_POSITION


class DevolDriveRobot:
    """
    A robot class that combines differential drive capabilities with manipulation.
    This class wraps a DevolRobot and adds mobile base functionality.
    """
    
    def __init__(
        self,
        node: Node,
        robot_name: str,
        namespace: str = '',
        planning_group: str = "ur_manipulator",
        target_object_count: int = 3
    ):
        self._node: Node = node
        self._logger = node.get_logger()
        self._tf_prefix: str = f"{robot_name}_"
        
        # Initialize the underlying DevolRobot
        self._devol_robot: DevolRobot = DevolRobot(
            node=node,
            robot_name=robot_name,
            namespace=namespace,
            planning_group=planning_group,
            target_object_count=target_object_count
        )
        
        # Initialize differential drive publisher
        cmd_topic: str = f"{namespace}/cmd_vel"
        
        # Create QoS profile (equivalent to C++ QoS setup)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        
        self._cmd_vel_pub: Publisher = self._node.create_publisher(
            Twist,
            cmd_topic,
            qos_profile
        )
        
        self._logger.info("DevolDriveRobot initialized. In Ready Position.")
    
    # Differential Drive Methods
    def publish_drive_command(self, linear_x: float, angular_z: float) -> None:
        """
        Publish a drive command to move the robot base.
        
        Args:
            linear_x: Linear velocity in x direction (m/s)
            angular_z: Angular velocity around z axis (rad/s)
        """
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        self._cmd_vel_pub.publish(msg)
        
        self._logger.info(f"Published Twist: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}")

    async def initialize_control(self) -> bool:
        return await self._devol_robot.initialize_control()

    # Manipulator Methods - Direct delegation to DevolRobot
    async def go_home(self) -> bool:
        """Move robot manipulator to home/ready position."""
        return await self._devol_robot.go_home()
    
    async def stop_robot(self) -> bool:
        return await self._devol_robot.stop_robot()
    
    async def activate_controller(self, controller_name: str) -> bool:
        return await self._devol_robot.activate_controller(controller_name=controller_name)

    async def deactivate_controller(self, controller_name: str) -> bool:
        return await self._devol_robot.deactivate_controller(controller_name=controller_name)
    
    async def get_controllers(self, active_only: bool = False) -> Union[List[str], Dict[str, str]]:
        return await self._devol_robot.get_controllers(active_only)

    async def set_manipulator_pose_goal(self, pose: Pose) -> bool:
        """Set manipulator pose goal."""
        return await self._devol_robot.set_manipulator_pose_goal(pose)
    
    async def set_manipulator_joint_goal(self, joint_values: List[float]) -> bool:
        """Set manipulator joint goal."""
        return await self._devol_robot.set_manipulator_joint_goal(joint_values)
    
    async def send_joint_velocities(self, velocities: List[float]) -> bool:
        return await self._devol_robot.send_joint_velocities(velocities=velocities)

    async def set_gripper_position(self, position: GRIPPER_POSITION) -> bool:
        """Set gripper position."""
        return await self._devol_robot.set_gripper_position(position)
    
    def attach_object_ros(self, object_id: str) -> None:
        """Attach object in ROS planning scene."""
        self._devol_robot.attach_object_ros(object_id)
    
    def detach_object_ros(self, object_id: str) -> None:
        """Detach object from ROS planning scene."""
        self._devol_robot.detach_object_ros(object_id)
    
    def allow_collision_between(self, object1: str, object2: str) -> None:
        """Allow collision between two objects."""
        self._devol_robot.allow_collision_between(object1, object2)
    
    def attach_object_gz(self, object_id: str) -> None:
        """Attach object in Gazebo."""
        self._devol_robot.attach_object_gz(object_id)
    
    def detach_object_gz(self, object_id: str) -> None:
        """Detach object in Gazebo."""
        self._devol_robot.detach_object_gz(object_id)
    
    def detach_all_objects_gz(self) -> None:
        """Detach all objects in Gazebo."""
        self._devol_robot.detach_all_objects_gz()
    