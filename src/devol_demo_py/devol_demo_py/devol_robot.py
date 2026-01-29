#!/usr/bin/env python3
from asyncio import run as asyncio_run
from enum import Enum
from typing import List, Dict, Optional, Union
from time import sleep

from rclpy import init as rclpy_init, shutdown as rclpy_shutdown
from rclpy.node import Node, Publisher, Client
from rclpy.action import ActionClient
from rclpy.task import Future

# ROS2 message imports
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty, Float64MultiArray
from control_msgs.action import ParallelGripperCommand
from moveit_msgs.action import MoveGroup
from controller_manager_msgs.srv import SwitchController, ListControllers, LoadController, ConfigureController
from moveit_msgs.msg import (
    AttachedCollisionObject, 
    PlanningScene, 
    AllowedCollisionEntry,
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint
)

class GRIPPER_POSITION(Enum):
    OPEN = "OPEN"
    CLOSE = "CLOSE"
    GRAB = "GRAB"


class AttachDetachTopics:
    def __init__(self):
        self.attach_pub = None
        self.detach_pub = None


class DevolRobot:
    MANIPULATOR_DOF: int = 6

    def __init__(
        self,
        node: Node,
        robot_name: str,
        planning_group: str = "ur_manipulator",
        target_object_count: int = 3
    ):
        self._node: Node = node
        self._logger = node.get_logger()
        self._robot_name: str = robot_name
        self._tf_prefix: str = f"{self._robot_name}_"
        self._planning_group: str = planning_group

        # Member Variables
        self._manipulator_joints: List[str] = [
            f"{self._tf_prefix}elbow_joint",
            f"{self._tf_prefix}shoulder_lift_joint",
            f"{self._tf_prefix}shoulder_pan_joint",
            f"{self._tf_prefix}wrist_1_joint",
            f"{self._tf_prefix}wrist_2_joint",
            f"{self._tf_prefix}wrist_3_joint"
        ]
        self._end_effector_touch_links: List[str] = [
            f"{self._tf_prefix}robotiq_85_left_finger_tip_link",
            f"{self._tf_prefix}robotiq_85_right_finger_tip_link",
            f"{self._tf_prefix}robotiq_85_left_finger_link",
            f"{self._tf_prefix}robotiq_85_right_finger_link",
            f"{self._tf_prefix}robotiq_85_left_inner_knuckle_link",
            f"{self._tf_prefix}robotiq_85_right_inner_knuckle_link",
            f"{self._tf_prefix}robotiq_85_left_knuckle_link",
            f"{self._tf_prefix}robotiq_85_right_knuckle_link"
        ]
        self._gripper_joint: str = f"{self._tf_prefix}robotiq_85_left_knuckle_joint"
        self._gripper_position_map: Dict[GRIPPER_POSITION, float] = {
            GRIPPER_POSITION.OPEN: 0.0,
            GRIPPER_POSITION.CLOSE: 0.8,
            GRIPPER_POSITION.GRAB: 0.105
        }
        self._base_link: str = f"{self._tf_prefix}base_link"
        self._end_effector_link: str = f"{self._tf_prefix}tool0"
        self._ur_manipulator_controller_name: str = f"ur_manipulator_controller"
        self._hand_controller_name: str = f"hand_controller"
        self._velocity_controller_name: str = f"ur_velocity_controller"
        self._joint_state_broadcaster_name: str = f"joint_state_broadcaster"
        self._required_controllers: List[str] = [
            self._ur_manipulator_controller_name,
            self._hand_controller_name,
            self._velocity_controller_name,
            self._joint_state_broadcaster_name
        ]

        self._target_object_count: int = target_object_count
        self._manipulator_goal_future: Optional[Future] = None
        self._gripper_goal_future: Optional[Future] = None

        # Initialize ROS2 publishers for attach/detach
        self._attach_detach_topics: Dict[str, AttachDetachTopics] = {}

        # Initialize action clients
        self._hand_action_client: ActionClient = ActionClient(
            self._node,
            ParallelGripperCommand,
            f"/{self._robot_name}/{self._hand_controller_name}/gripper_cmd"
        )

        # Initialize Movegroup interfaces
        self._move_group_client: ActionClient = ActionClient(
            self._node,
            MoveGroup,
            f"/{self._robot_name}/move_action"
        )

        # Controller manager service client
        self._controller_switch_client: Client = self._node.create_client(
            SwitchController,
            f'/{self._robot_name}/controller_manager/switch_controller'
        )
        self._controller_list_client: Client = self._node.create_client(
            ListControllers,
            f'/{self._robot_name}/controller_manager/list_controllers'
        )
        self._controller_load_client: Client = self._node.create_client(
            LoadController, f'/{self._robot_name}/controller_manager/load_controller'
        )
        self._controller_configure_client: Client = self._node.create_client(
            ConfigureController, f'/{self._robot_name}/controller_manager/configure_controller'
        )
        self._velocity_command_pub: Publisher = self._node.create_publisher(
            Float64MultiArray,
            f"/{self._robot_name}/{self._velocity_controller_name}/commands",
            10
        )

        # Create publishers
        self._planning_scene_pub: Publisher = self._node.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )

        # Wait for clients to find servers and services
        self._wait_for_services()

        # Initialize attach and detach topics
        self._initialize_attach_detach_topics()

        # Detach all objects to begin
        self.detach_all_objects_gz()

        self._logger.info("DevolRobot initialized.")
    
    def _wait_for_services(self) -> None:
        service_clients: Dict[str, Client] = {
            'controller manager': self._controller_switch_client,
            'controller list': self._controller_list_client,
            'controller load': self._controller_load_client,
            'controller configure': self._controller_configure_client
        }
        
        for client_name, service_client in service_clients.items():
            if not service_client.wait_for_service(timeout_sec=5.0):
                self._logger.error(f"{client_name.capitalize()} service not available")
            else:
                self._logger.info(f"{client_name.capitalize()} client initialized successfully")

    async def initialize_control(self) -> bool:
        # Activate joint_state_broadcaster 
        if not await self.activate_controller(self._joint_state_broadcaster_name):
            return False

        # Activate hand_controller 
        if not await self.activate_controller(self._hand_controller_name):
            return False

        # Activate ur_velocity_controller 
        if not await self.activate_controller(self._velocity_controller_name):
            return False
        
        # Deactivate joint rajectory controller from moveit
        if not await self.deactivate_controller(self._ur_manipulator_controller_name):
            return False
        
        self._wait_for_client_servers()
        
        return True

    def _wait_for_client_servers(self) -> None:
        """Wait for servers to become available"""
        action_clients: Dict[str, ActionClient] = {
            "hand": self._hand_action_client,
            "move group": self._move_group_client
        }

        for client_name, action_client in action_clients.items():
            if not action_client.wait_for_server(timeout_sec=5.0):
                self._logger.error(f"{client_name.capitalize()} action server not available after waiting.")
            else:
                self._logger.info(f"{client_name.capitalize()} action client initialized successfully")

    def _initialize_attach_detach_topics(self) -> None:
        """Initialize Gazebo attach/detach topics"""
        for object_index in range(self._target_object_count):
            object_id = f"target_block_{object_index}"
            attach_topic = f"/{self._tf_prefix}attach_{object_index}/attach"
            detach_topic = f"/{self._tf_prefix}attach_{object_index}/detach"
            
            topics = AttachDetachTopics()
            topics.attach_pub = self._node.create_publisher(Empty, attach_topic, 10)
            topics.detach_pub = self._node.create_publisher(Empty, detach_topic, 10)
            
            self._attach_detach_topics[object_id] = topics
            self._logger.info(f"Initialized attach/detach topics for {object_id}")
            sleep(0.1)

    def _create_manipulator_joint_request(self, target: List[float]) -> MotionPlanRequest:
        request: MotionPlanRequest = MotionPlanRequest()
        request.group_name = self._planning_group
        request.num_planning_attempts = 10
        request.allowed_planning_time = 60.0
        request.max_velocity_scaling_factor = 1.0
        request.max_acceleration_scaling_factor = 1.0
        request.start_state.is_diff = True  # Use current state as start

        constraint: Constraints = Constraints()
        for joint_name, joint_value in zip(self._manipulator_joints, target):
            joint_constraint: JointConstraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(joint_value)
            joint_constraint.tolerance_above = 0.01  # Adjust as needed
            joint_constraint.tolerance_below = 0.01  # Adjust as needed
            joint_constraint.weight = 1.0
            
            constraint.joint_constraints.append(joint_constraint)

        request.goal_constraints = [constraint]

        # After creating the request and before returning it
        for i, constraint in enumerate(request.goal_constraints):
            self._logger.debug(f"Goal constraint {i} has {len(constraint.joint_constraints)} joint constraints")
            for j, joint_constraint in enumerate(constraint.joint_constraints):
                self._logger.debug(f"  Joint {j}: {joint_constraint.joint_name} = {joint_constraint.position}")

        return request

    def _create_manipulator_pose_request(self, target: Pose) -> MotionPlanRequest:
        request: MotionPlanRequest = MotionPlanRequest()
        request.group_name = self._planning_group
        request.num_planning_attempts = 10
        request.allowed_planning_time = 60.0
        request.max_velocity_scaling_factor = 1.0
        request.max_acceleration_scaling_factor = 1.0

        constraint: Constraints = Constraints()

        # Create Position Constraint
        position_constraint: PositionConstraint = PositionConstraint()
        position_constraint.header.frame_id = self._base_link
        position_constraint.link_name = self._end_effector_link
        position_constraint.target_point_offset.x = target.position.x
        position_constraint.target_point_offset.y = target.position.y
        position_constraint.target_point_offset.z = target.position.z
        position_constraint.weight = 1.0

        # Create Orientation Constraint
        orientation_constraint: OrientationConstraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self._base_link
        orientation_constraint.link_name = self._end_effector_link
        orientation_constraint.orientation = target.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.05
        orientation_constraint.absolute_y_axis_tolerance = 0.05
        orientation_constraint.absolute_z_axis_tolerance = 0.05
        orientation_constraint.weight = 1.0

        constraint.position_constraints = [position_constraint]
        constraint.orientation_constraints = [orientation_constraint]
        request.goal_constraints = [constraint]

        return request

    def _create_manipulator_named_request(self, target: str) -> MotionPlanRequest:
        self._logger.info("Going to ready position asyncronously")

        request: MotionPlanRequest = MotionPlanRequest()
        request.group_name = self._planning_group
        request.num_planning_attempts = 10
        request.allowed_planning_time = 60
        request.max_acceleration_scaling_factor = 1.0
        request.max_acceleration_scaling_factor = 1.0

        constraint: Constraints = Constraints()
        constraint.name = target
        request.goal_constraints = [constraint]

        return request
        
    async def _set_manipulation_goal(self, motion_plan_request: MotionPlanRequest, request_name: str ="manipulator goal") -> bool:
        if not self._move_group_client.server_is_ready():
            self._logger.error("MoveGroup action server is not ready.")
            return False

        goal: MoveGroup.Goal = MoveGroup.Goal()
        goal.request = motion_plan_request
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.plan_only = False # Plan and execute!
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        try:
            future: Future = self._move_group_client.send_goal_async(goal)
            result_handle = await future

            if not result_handle.accepted:
                self._logger.error(f"{request_name.capitalize()} was rejected by server.")
                return False
            
            self._logger.info(f"{request_name.capitalize()} accepted by server, executing...")

            result = await result_handle.get_result_async()

            if result.result.error_code.val == 1: # Success!
                self._logger.info(f"Successfully moved to {request_name.capitalize()}.")
                return True
            else:
                self._logger.error(f"{request_name.capitalize()} failed with error code: {result.result.error_code.val}")
                return False

        except Exception as e:
            self._logger.error(f"Exception encountered during set_manipulator_pose_goal({request_name.capitalize}): {e}")
            return False
        
    async def get_controllers(self, active_only: bool = False) -> Union[List[str], Dict[str, str]]:
        """Get all controllers with their states"""
        if not self._controller_list_client.service_is_ready():
            self._logger.error("Controller list service not ready")
            return {}

        try:
            request = ListControllers.Request()
            response = await self._controller_list_client.call_async(request)
            
            if active_only:
                return [c.name for c in response.controller if c.state == 'active']
            else:
                return {c.name: c.state for c in response.controller}
            
        except Exception as e:
            self._logger.error(f"Exception getting controller list: {e}")
            return [] if active_only else {}

    async def load_controller(self, controller_name: str) -> bool:
        """Load a controller"""
        if not self._controller_load_client.service_is_ready():
            self._logger.error("Controller load service not ready")
            return False
        
        try:
            request = LoadController.Request()
            request.name = controller_name
            
            response = await self._controller_load_client.call_async(request)
            
            if response.ok:
                self._logger.info(f"Successfully loaded controller: {controller_name}")
                return True
            else:
                self._logger.error(f"Failed to load controller {controller_name}")
                return False
                
        except Exception as e:
            self._logger.error(f"Exception loading controller {controller_name}: {e}")
            return False

    async def configure_controller(self, controller_name: str) -> bool:
        """Configure a controller"""
        if not self._controller_configure_client.service_is_ready():
            self._logger.error("Controller configure service not ready")
            return False
        
        try:
            request = ConfigureController.Request()
            request.name = controller_name
            
            response = await self._controller_configure_client.call_async(request)
            
            if response.ok:
                self._logger.info(f"Successfully configured controller: {controller_name}")
                return True
            else:
                self._logger.error(f"Failed to configure controller {controller_name}")
                return False
                
        except Exception as e:
            self._logger.error(f"Exception configuring controller {controller_name}: {e}")
            return False

    async def ensure_controller_loaded_and_configured(self, controller_name: str) -> bool:
        """Ensure a controller is loaded and configured"""
        controllers = await self.get_controllers()
        
        if controller_name not in controllers:
            self._logger.info(f"Loading controller: {controller_name}")
            if not await self.load_controller(controller_name):
                return False
                
            # After loading, configure it
            self._logger.info(f"Configuring controller: {controller_name}")
            if not await self.configure_controller(controller_name):
                return False
        else:
            state = controllers[controller_name]
            if state == "unconfigured":
                self._logger.info(f"Configuring controller: {controller_name}")
                if not await self.configure_controller(controller_name):
                    return False
            elif state in ["configured", "inactive", "active"]:
                self._logger.info(f"Controller {controller_name} already loaded (state: {state})")
            else:
                self._logger.warn(f"Controller {controller_name} in unexpected state: {state}")
        
        return True
    
    async def activate_controller(self, controller_name: str) -> bool:
        if not self._controller_switch_client.service_is_ready():
            self._logger.error("Controller switch service not ready")
            return False
        
        if await self.is_controller_active(controller_name=controller_name):
            self._logger.info(f"Controller {controller_name} is already active")
            return True
        
        if not await self.ensure_controller_loaded_and_configured(controller_name=controller_name):
            self._logger.error(f"Failed to ensure {controller_name} is loaded and configured.")

        self._logger.info(f"Activating controller: {controller_name}")

        request: SwitchController.Request = SwitchController.Request()
        request.activate_controllers = [controller_name]
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True

        try:
            response = await self._controller_switch_client.call_async(request)

            if response.ok:
                self._logger.info(f"Successfully activated controller: {controller_name}")
                return True
            else:
                self._logger.error(f"Failed to activate controller {controller_name}: {response.message}")
                return False
            
        except Exception as e:
            self._logger.error(f"Exception actiavting controller {controller_name}: {e}")

    async def deactivate_controller(self, controller_name: str) -> bool:
        """Deactivate a specific controller"""
        if not self._controller_switch_client.service_is_ready():
            self._logger.error("Controller switch service not ready")
            return False
        
        # First check if already inactive
        if not await self.is_controller_active(controller_name=controller_name):
            self._logger.info(f"Controller {controller_name} is already inactive")
            return True
        
        self._logger.info(f"Deactivating controller: {controller_name}")
        request = SwitchController.Request()
        request.activate_controllers = []
        request.deactivate_controllers = [controller_name]
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = False
        
        try:
            response = await self._controller_switch_client.call_async(request)
            
            if response.ok:
                self._logger.info(f"Successfully deactivated controller: {controller_name}")
                return True
            else:
                self._logger.error(f"Failed to deactivate controller {controller_name}: {response.message}")
                return False
                
        except Exception as e:
            self._logger.error(f"Exception deactivating controller {controller_name}: {e}")
            return False

    async def is_controller_active(self, controller_name: str) -> bool:
        """Check if a specific controller is active"""
        active_controllers = await self.get_controllers(active_only=True)
        return controller_name in active_controllers

    async def stop_robot(self) -> bool:
        zero_velocities: List[float] = [0.0] * self.MANIPULATOR_DOF
        return await self.send_joint_velocities(zero_velocities)
    
    async def go_home(self) -> bool:
        """Move robot to home/ready position"""
        self._logger.info("Going to ready position asynchronously")

        home_joint_states: List[float] = [
            1.8571, -2.6729, 0.0, 0.729, 1.8398, 0.0
        ]
        motion_plan_request: MotionPlanRequest = self._create_manipulator_joint_request(target=home_joint_states)
        return await self._set_manipulation_goal(motion_plan_request=motion_plan_request, request_name="go home")
    
    async def set_manipulator_pose_goal(self, pose: Pose) -> bool:
        """Set manipulator pose goal"""
        self._logger.info("Going to pose goal asynchronously")
        motion_plan_request: MotionPlanRequest = self._create_manipulator_pose_request(target=pose)
        return await self._set_manipulation_goal(motion_plan_request=motion_plan_request, request_name="manipulator pose goal")
    
    async def set_manipulator_joint_goal(self, joint_values: List[float]) -> bool:
        """Set manipulator joint goal"""
        self._logger.info("Going to joint goal asynchronously")
        motion_plan_request: MotionPlanRequest = self._create_manipulator_joint_request(target=joint_values)
        return await self._set_manipulation_goal(motion_plan_request=motion_plan_request, request_name="manipulator joint goal")

    async def send_joint_velocities(self, velocities: List[float]) -> bool:
        if (len(velocities)) != self.MANIPULATOR_DOF:
            self._logger.error(f"Expected {self.MANIPULATOR_DOF} velocity values, got {len(velocities)}")

        msg: Float64MultiArray = Float64MultiArray()
        msg.data = velocities

        try:
            self._velocity_command_pub.publish(msg)
            self._logger.debug(f"Sent velocity command: {velocities}")
            return True
        except Exception as e:
            self._logger.error(f"Failed to send velocity command: {e}")
            return False

    async def set_gripper_position(self, position: GRIPPER_POSITION) -> bool:
        """Set gripper position"""
        if not self._hand_action_client.server_is_ready():
            self._logger.error("Gripper action server is not ready.")
            return False
        
        goal = ParallelGripperCommand.Goal()
        goal.command.name = [self._gripper_joint]
        goal.command.effort = [500.0]
        
        if position not in self._gripper_position_map:
            self._logger.error("Invalid gripper position")
            return False
        
        goal.command.position = [self._gripper_position_map[position]]
        
        self._logger.info(f"Setting gripper to {position.value} ({goal.command.position[0]:.3f})")
        
        try:
            goal_handle = await self._hand_action_client.send_goal_async(goal)
            if not goal_handle.accepted:
                self._logger.error("Gripper goal was rejected by server.")
                return False
            
            self._logger.info("Gripper goal accepted by server, executing...")
            result = await goal_handle.get_result_async()
            
            if result.status == 4:  # SUCCEEDED
                self._logger.info(f"Gripper successfully moved to {position.value}")
                return True
            else:
                self._logger.error(f"Gripper movement to {position.value} failed with status: {result.status}")
                return False
                
        except Exception as e:
            self._logger.error(f"Exception during set_gripper_position: {e}")
            return False
    
    def attach_object_ros(self, object_id: str):
        """Attach object in ROS planning scene"""
        attached_object: AttachedCollisionObject = AttachedCollisionObject()
        attached_object.link_name = self._end_effector_link
        attached_object.object.id = object_id
        attached_object.object.header.frame_id = self._base_link
        attached_object.object.header.stamp = self.node.get_clock().now().to_msg()
        attached_object.object.operation = attached_object.object.ADD
        attached_object.touch_links = self._end_effector_touch_links

        scene_msg: PlanningScene = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.attached_collision_objects = [attached_object]

        self._planning_scene_pub.publish(scene_msg)
        self._logger.info(f"Attached the object {object_id} to the end effector.")
    
    def detach_object_ros(self, object_id: str):
        """Detach object from ROS planning scene"""
        detached_object: AttachedCollisionObject = AttachedCollisionObject()
        detached_object.link_name = self.ur_manipulator_group_interface.get_end_effector_link()
        detached_object.object.id = object_id
        detached_object.object.header.frame_id = self.ur_manipulator_group_interface.get_planning_frame()
        detached_object.object.header.stamp = self.node.get_clock().now().to_msg()
        detached_object.object.operation = detached_object.object.REMOVE
        
        scene_msg: PlanningScene = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.attached_collision_objects = [detached_object]

        self._planning_scene_pub.publish(scene_msg)
        self._logger.info(f"Detached the object {object_id} from the end effector.")
    
    def allow_collision_between(self, object1: str, object2: str) -> None:
        """Allow collision between two objects"""
        planning_scene_msg: PlanningScene = PlanningScene()
        planning_scene_msg.is_diff = True
        
        # Add entry names
        planning_scene_msg.allowed_collision_matrix.entry_names.append(object1)
        planning_scene_msg.allowed_collision_matrix.entry_names.append(object2)
        
        # Create entries to allow collision
        entry: AllowedCollisionEntry = AllowedCollisionEntry()
        entry.enabled = [True, True]  # allow object1 <-> object2
        
        planning_scene_msg.allowed_collision_matrix.entry_values.append(entry)
        planning_scene_msg.allowed_collision_matrix.entry_values.append(entry)
        
        # Apply the planning scene update
        self._planning_scene_pub.publish(planning_scene_msg)
        self._logger.info(f"Allowed collision between {object1} and {object2}")
    
    def attach_object_gz(self, object_id: str) -> None:
        """Attach object in Gazebo"""
        if object_id in self._attach_detach_topics:
            msg: Empty = Empty()
            self._attach_detach_topics[object_id].attach_pub.publish(msg)
            self._logger.info(f"Published attach message for {object_id}")
        else:
            self._logger.warn(f"No attach topic found for object: {object_id}")
    
    def detach_object_gz(self, object_id: str) -> None:
        """Detach object in Gazebo"""
        if object_id in self._attach_detach_topics:
            msg: Empty = Empty()
            self._attach_detach_topics[object_id].detach_pub.publish(msg)
            self._logger.info(f"Published detach message for {object_id}")
        else:
            self._logger.warn(f"No detach topic found for object: {object_id}")
    
    def detach_all_objects_gz(self):
        """Detach all objects in Gazebo"""
        for object_index in range(3):
            object_id: str = f"target_block_{object_index}"
            self.detach_object_gz(object_id)


# Example usage
async def main():
    rclpy_init()
    
    node = Node('devol_robot_node')

    robot: DevolRobot = DevolRobot(node=node, tf_prefix="")

    try:
        # Example usage
        if await robot.go_home():
            node.get_logger().info("Robot successfully moved home")
        
        # Set gripper position
        if await robot.set_gripper_position(GRIPPER_POSITION.OPEN):
            node.get_logger().info("Gripper opened successfully")
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy_shutdown()


if __name__ == '__main__':
    asyncio_run(main())