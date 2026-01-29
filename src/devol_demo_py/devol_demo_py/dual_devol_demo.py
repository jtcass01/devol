from sys import exit as sys_exit
from asyncio import run as asyncio_run, sleep as asyncio_sleep, create_task, gather, Task
from signal import signal, SIGINT
from threading import Thread
from typing import Dict, List

from rclpy import init as rclpy_init, shutdown as rclpy_shutdown
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from devol_demo_py.devol_drive_robot import DevolDriveRobot
from devol_demo_py.devol_robot import GRIPPER_POSITION

class DualDevolDemoNode(Node):
    NAME: str = "dual_devol_demo_node"

    def __init__(self):
        super().__init__(
            self.NAME, automatically_declare_parameters_from_overrides=True
        )

        self._logger = self.get_logger()
        self._robot_names: List[str] = ["apollo", "artemis"]
        self._robots: Dict[str, DevolDriveRobot] = {}

        signal(SIGINT, self._signal_handler)

        self._init_robots()

        self._logger.info(f"{self.__class__.__name__} initialized successfully")

    def _init_robots(self) -> None:
        try:
            for robot_name in self._robot_names:
                self._robots[robot_name] = DevolDriveRobot(
                    node=self,
                    robot_name=robot_name
                )
            self._logger.info("All robots successfully initialized!")
        except Exception as e:
            self._logger.error(f"Failed to initialize robots: {e}")

    async def run_demo_sequence(self) -> None:
        # 1. Initialize controllers
        for robot_name, robot in self._robots.items():
            try:
                control_init_success: bool = await robot.initialize_control()
                if not control_init_success:
                    return False
            except Exception as e:
                self._logger.error(f"Exception during {robot_name} robot control initialization: {e}")
                return False
            
            try:
                # Ensure velocity controller is inactive
                if not await robot.deactivate_controller(robot._devol_robot._velocity_controller_name):
                    return False
            
                # Ensure trajectory controller is active
                if not await robot.activate_controller(robot._devol_robot._ur_manipulator_controller_name):
                    return False
            except Exception as e:
                self._logger.error(f"Exception during {robot_name} robot controller activation: {e}")
                return False
            
            try:
                if await robot.stop_robot():
                    self._logger.info("Robot stopped successfully")
                else:
                    self._logger.error("Failed to stop robot")
                    return False
            except Exception as e:
                self._logger.error(f"Exception during robot stop: {e}")
                return False
            
        # 2. Send the robots home
        for robot_name, robot in self._robots.items():
            try:
                if await robot.go_home():
                    self._logger.info(f"{robot_name} moved to home position")
                else:
                    self._logger.error(f"Failed to move {robot_name} home")
                    return False
            except Exception as e:
                self._logger.error(f"Exception during {robot_name} go home: {e}")
                return False

        # 3. Send Close Gripper
        for robot_name, robot in self._robots.items():
            try:
                if await robot.set_gripper_position(GRIPPER_POSITION.CLOSE):
                    self._logger.info(f"{robot_name} gripper closed successfully")
                else:
                    self._logger.error(f"Failed to close {robot_name} gripper")
                    return False
            except Exception as e:
                self._logger.error(f"Exception during {robot_name} gripper close: {e}")
                return False
            
        # 4. Switch to velocity control and test
        for robot_name, robot in self._robots.items():
            try:
                # 2. Ensure trajectory controller is inactive
                if not await robot.deactivate_controller(robot._devol_robot._ur_manipulator_controller_name):
                    return False

                # 2. Ensure velocity controller is active
                if not await robot.activate_controller(robot._devol_robot._velocity_controller_name):
                    return False
            except Exception as e:
                self._logger.error(f"Exception during {robot_name} robot controller activation: {e}")
                return False

            try:
                joint_velocities = [5.0, 0.0, 0.0, 0.0, 0.0, -0.2]
                if await robot.send_joint_velocities(joint_velocities):
                    self._logger.info(f"Joint velocity control for {robot_name} executed successfully")
                    
                    # Let it run for a bit, then stop
                    await asyncio_sleep(3.0)  # Run for 2 seconds
                    
                    if await robot.stop_robot():
                        self._logger.info(f"{robot_name} stopped after velocity control")
                    else:
                        self._logger.warning(f"Failed to stop {robot_name} after velocity control")
                else:
                    self._logger.error(f"Failed to execute joint velocity control for {robot_name}")
                    return False
            except Exception as e:
                self._logger.error(f"Exception during {robot_name} velocity control: {e}")
                return False
        
        # 5. Drive in two different circles
        for robot_index, robot in enumerate(self._robots.values()):
            robot.publish_drive_command(linear_x=5.0/(robot_index+1), angular_z=1.0*robot_index)

        return True

    def _signal_handler(self, signum: int, frame: int) -> None:
        self._logger.info(f"Received shutdown signal[{signum}]. Stopping demo...")
        sys_exit(0)


async def run_node_async() -> None:
    rclpy_init()

    try:
        # Create the demo node
        demo_node: DualDevolDemoNode = DualDevolDemoNode()

        # Create Executor
        executor: MultiThreadedExecutor = MultiThreadedExecutor()
        executor.add_node(demo_node)

        # Start ROS2 in a seperate thread
        spin_thread: Thread = Thread(
            target=executor.spin, daemon=True
        )
        spin_thread.start()

        await demo_node.run_demo_sequence()

        demo_node.get_logger().info("Demo completed. Node will shutdown in 2 seconds...")
        await asyncio_sleep(2.0)
    except KeyboardInterrupt:
        demo_node.get_logger().info("Received KeyboardInterrupt. Shutting down...")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        try:
            demo_node.destroy_node()
        except:
            pass
        
        rclpy_shutdown()

        if 'spin_thread' in locals():
            spin_thread.join(timeout=1.0)


def main():
    try:
        asyncio_run(run_node_async())
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Fatal error: {e}")

    return 0    

if __name__ == "__main__":
    exit_code: int = main()
    sys_exit(exit_code)