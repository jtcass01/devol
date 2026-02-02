from sys import exit as sys_exit
from asyncio import run as asyncio_run, sleep as asyncio_sleep, create_task, gather, Task
from signal import signal, SIGINT
from threading import Thread

from rclpy import init as rclpy_init, shutdown as rclpy_shutdown
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from devol_demo_py.devol_drive_robot import DevolDriveRobot
from devol_demo_py.devol_robot import GRIPPER_POSITION

class DevolDriveDemoNode(Node):
    NAME: str = "devol_drive_demo_node"

    def __init__(self):
        super().__init__(
            self.NAME, automatically_declare_parameters_from_overrides=True
        )

        self._logger = self.get_logger()
        self._robot: DevolDriveRobot = DevolDriveRobot(node=self, robot_name="devol_drive", namespace='/devol_drive', target_object_count=3)

        signal(SIGINT, self._signal_handler)

        self._logger.info(f"{self.__class__.__name__} initialized successfully")

    async def run_demo_sequence(self) -> None:
        # 1. Initialize controllers
        try:
            control_init_success: bool = await self._robot.initialize_control()
            if not control_init_success:
                return False
        except Exception as e:
            self._logger.error(f"Exception during robot control initialization: {e}")
            return False
        print(f"Well..")

        try:
            # 2. Ensure velocity controller is inactive
            if not await self._robot.deactivate_controller(self._robot._devol_robot._velocity_controller_name):
                return False
        
            # 2. Ensure trajectory controller is active
            if not await self._robot.activate_controller(self._robot._devol_robot._ur_manipulator_controller_name):
                return False
        except Exception as e:
            self._logger.error(f"Exception during robot control initialization: {e}")
            return False

        # 3. Stop the robot
        try:
            if await self._robot.stop_robot():
                self._logger.info("Robot stopped successfully")
            else:
                self._logger.error("Failed to stop robot")
                return False
        except Exception as e:
            self._logger.error(f"Exception during robot stop: {e}")
            return False
        
        # 4. Go home
        # try:
        #     if await self._robot.go_home():
        #         self._logger.info("Robot moved to home position")
        #     else:
        #         self._logger.error("Failed to move robot home")
        #         return False
        # except Exception as e:
        #     self._logger.error(f"Exception during go home: {e}")
        #     return False

        # 5. Close gripper
        # try:
        #     if await self._robot.set_gripper_position(GRIPPER_POSITION.CLOSE):
        #         self._logger.info("Gripper closed successfully")
        #     else:
        #         self._logger.error("Failed to close gripper")
        #         return False
        # except Exception as e:
        #     self._logger.error(f"Exception during gripper close: {e}")
        #     return False

        # 6. Switch to velocity control and test
        try:
            # 2. Ensure velocity controller is inactive
            if not await self._robot.deactivate_controller(self._robot._devol_robot._ur_manipulator_controller_name):
                return False
        
            # 2. Ensure trajectory controller is active
            if not await self._robot.activate_controller(self._robot._devol_robot._velocity_controller_name):
                return False
        except Exception as e:
            self._logger.error(f"Exception during robot control initialization: {e}")
            return False

        # 7. Execute velocity control
        try:
            joint_velocities = [0.2, 0.0, 0.0, 0.0, 0.0, -0.2]
            if await self._robot.send_joint_velocities(joint_velocities):
                self._logger.info("Joint velocity control executed successfully")
                
                # Let it run for a bit, then stop
                await asyncio_sleep(2.0)  # Run for 2 seconds
                
                if await self._robot.stop_robot():
                    self._logger.info("Robot stopped after velocity control")
                else:
                    self._logger.warning("Failed to stop robot after velocity control")
                    
            else:
                self._logger.error("Failed to execute joint velocity control")
                return False
        except Exception as e:
            self._logger.error(f"Exception during velocity control: {e}")
            return False

        # Drive in a circle!
        self._robot.publish_drive_command(linear_x=1.0, angular_z=3.0)
        return True


    def _signal_handler(self, signum: int, frame: int) -> None:
        self._logger.info(f"Received shutdown signal[{signum}]. Stopping demo...")
        sys_exit(0)


async def run_node_async() -> None:
    rclpy_init()

    try:
        # Create the demo node
        demo_node: DevolDriveDemoNode = DevolDriveDemoNode()

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