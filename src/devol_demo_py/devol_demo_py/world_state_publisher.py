from sys import exit as sys_exit
from asyncio import run as asyncio_run, sleep as asyncio_sleep, create_task, gather, Task
from signal import signal, SIGINT
from threading import Thread
from typing import List, Dict
from time import sleep
from math import atan2, cos, sin

from rclpy import init as rclpy_init, shutdown as rclpy_shutdown, ok as rclpy_ok
from rclpy.node import Node, Subscription, Publisher
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import Float64MultiArray


class WorldStatePublisher(Node):
    NAME: str = "world_state_publisher"

    def __init__(self, publish_frequency: float = 20.):
        super().__init__(
            self.NAME, automatically_declare_parameters_from_overrides=True
        )

        self._logger = self.get_logger()
        self._robot_spawns: Dict[str, Dict[str, float]] = {
            "apollo": {"x": 0.0, "y": -1.0, "yaw": 0.0},
            'artemis': {'x': 0.0, 'y': 1.0, 'yaw': 0.0}
        }
        self._robots: List[str] = list(self._robot_spawns.keys())

        # Initialize state storage
        self._robot_states: Dict[str, Dict[str, float]] = {
            robot: {"x": 0.0, "y": 0.0, "yaw": 0.0, "joints": []}
            for robot in self._robots
        }

        # Subscriptions
        for robot in self._robots:
            self.create_subscription(
                Pose,
                f"/model/{robot}/pose",
                lambda msg, r=robot: self._pose_callback(msg,r),
                10)
            self.create_subscription(
                JointState,
                f"/{robot}/joint_states",
                lambda msg, r=robot: self._joint_state_callback(msg,r),
                10)

        # Publisher
        self._publisher: Publisher = self.create_publisher(
            Float64MultiArray,
            "/world_state",
            10
        )

        # Timer
        self.create_timer(1./publish_frequency, self._publish_world_state)
        
        signal(SIGINT, self._signal_handler)
        self._logger.info(f"{self.__class__.__name__} initialized successfully")

    def _pose_callback(self, msg: Pose, robot: str) -> None:
        q: Quaternion = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw: float = atan2(siny_cosp, cosy_cosp)

        self._robot_states[robot]['x'] = msg.position.x
        self._robot_states[robot]['y'] = msg.position.y
        self._robot_states[robot]['yaw'] = yaw

    def _joint_state_callback(self, msg: JointState, robot: str) -> None:
        self._robot_states[robot]['joints'] = list(msg.position)

    def _publish_world_state(self):
        data: List[float] = []

        for robot in self._robots:
            state = self._robot_states[robot]
            data.extend([state['x'], state['y'], state['yaw']])
            data.extend(state['joints'])
        msg: Float64MultiArray = Float64MultiArray()
        msg.data = data
        self._publisher.publish(msg)

    def _signal_handler(self, signum: int, frame: int) -> None:
        self._logger.info(f"Received shutdown signal[{signum}]. Stopping...")
        sys_exit(0)

def main():
    rclpy_init()

    try:
        # Create the demo node
        node: WorldStatePublisher = WorldStatePublisher()

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
    exit_code: int = main()
    sys_exit(exit_code)