from time import sleep
from typing import Optional, Tuple
from csv import writer

from numpy import ndarray, array, arctan2, pi, cos, sin, zeros, clip
from numpy.linalg import norm

from rclpy import spin as rclpy_spin, init as rclpy_init, shutdown as rclpy_shutdown
from rclpy.node import Node, Timer, Subscription, Publisher
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from tf2_ros import TransformListener, Buffer, LookupException
from std_msgs.msg import Float64MultiArray

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


def quaternion_rotation_matrix(Q: Quaternion) -> ndarray:
    """Creates a 3x3 rotation matrix in 3D space from a quaternion.

    Input
    :param Q: A 4 element array containing the quaternion (q0,q1,q2,q3) 

    Output
    :return: A 3x3 element matix containing the rotation matrix"""
    q0, q1, q2, q3 = Q.w, Q.x, Q.y, Q.z

    r_11: float = 1 - 2*(q2**2 + q3**2)
    r_12: float = 2*(q1*q2-q0*q3)
    r_13: float = 2*(q1*q3+q0*q2)
    r_21: float = 2*(q1*q2+q0*q3)
    r_22: float = 1 - 2*(q1**2 + q3**2)
    r_23: float = 2*(q2*q3-q0*q1)
    r_31: float = 2*(q1*q3-q0*q2)
    r_32: float = 2*(q2*q3+q0*q1)
    r_33: float = 1 - 2*(q1**2 + q2**2)

    return array([
        [r_11, r_12, r_13],
        [r_21, r_22, r_23],
        [r_31, r_32, r_33]
    ])


def quaternion_to_euler(Q: Quaternion) -> ndarray:
    """Takes a quaternion and returns the roll, pitch yaw array.

    Input
    :param Q0: A 4 element array containing the quaternion (q01,q11,q21,q31) 

    Output
    :return: A 3 element array containing the roll,pitch, and yaw (alpha,beta,gamma)"""
    R: ndarray = quaternion_rotation_matrix(Q)

    alpha: float = arctan2(R[2,1], R[2,2])
    beta: float = arctan2(-R[2,0], (R[2,1]**2+R[2,2]**2)**0.5)
    gamma: float = arctan2(R[1,0], R[0,0])

    return array([
        alpha, beta, gamma
    ])


class DiffDrivePID(Node):
    # Subscribes: 
    #   /tf
    #   /goal_pose
    # Publishes:
    #   /cmd_vel : geometry_msgs/Twist : This is the desired velocity (linear and angular) for the centerpoint of the robot
    MAX_LINEAR_VELOCITY: float 

    def __init__(self):
        super().__init__('diffdrive_pid')
        # Declare parameters with default values
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('lookahead', 0.5)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', pi)
        self.declare_parameter('position_tolerance', 0.005)
        self.declare_parameter('max_integral', 1.0)
        self.declare_parameter('x', 0.0) # Starting x position
        self.declare_parameter('y', -3.0) # Starting y position
        self.declare_parameter('yaw', 1.57) # Starting yaw position
        
        self._kp: float = self.get_parameter('kp').get_parameter_value().double_value
        self._ki: float = self.get_parameter('ki').get_parameter_value().double_value
        self._kd: float = self.get_parameter('kd').get_parameter_value().double_value
        self._lookahead: float = self.get_parameter('lookahead').get_parameter_value().double_value
        self._publish_rate: float = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._max_linear_velocity: float = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self._max_angular_velocity: float = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self._max_integral: float = self.get_parameter('max_integral').get_parameter_value().double_value
        self._position_tolerance: float = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self._start_x: float = float(self.get_parameter('x').get_parameter_value().double_value)
        self._start_y: float = float(self.get_parameter('y').get_parameter_value().double_value)
        self._start_yaw: float = float(self.get_parameter('yaw').get_parameter_value().double_value)

        self._dt: float = 1.0 / self._publish_rate
        self._integral_error = zeros(2)
        self._last_error = zeros(2)
        self._tf_buffer: Buffer = Buffer()
        self._tf_listener: TransformListener = TransformListener(self._tf_buffer, self)
        self._goal_pose: Optional[PoseStamped] = None
        self._goal_time: Optional[Time] = None

        self._goal_pose_sub: Subscription = \
            self.create_subscription(PoseStamped, 'goal_pose', 
                                     self._goal_pose_callback,
                                     10)
        self._cmd_vel_pub: Publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self._timer: Timer = self.create_timer(self._dt, self._control_loop)

        self._from_frame: str = 'diff_drive/body_link'
        self._to_frame: str = 'maze_world'

        # Extra DEBUG stuff left in -- just in case you are curious how pid was tuned.
        # self._pid_debug_pub: Publisher = self.create_publisher(Float64MultiArray, 'pid_debug', 10)
        # self._csv_file = open("pid_log.csv", "w", newline="")
        # self._csv_writer = writer(self._csv_file)
        # self._csv_writer.writerow([
        #     "time", "x", "y", "gx", "gy", 
        #     "error_x", "error_y", 
        #     "v_f", "omega"
        # ])

    def _publish_velocity_command(self, v_f: ndarray, omega: ndarray) -> None:
        cmd_vel: Twist = Twist()
        cmd_vel.linear.x = v_f
        cmd_vel.angular.z = omega
        self._cmd_vel_pub.publish(cmd_vel)

    def _goal_pose_callback(self, msg: PoseStamped) -> None:
        self._goal_pose = msg
        self._goal_time = self.get_clock().now()

    def _compute_control(self, x, y, yaw, gx, gy) -> Tuple[float, float]:
        proportional_error: ndarray = array([gx - x, gy - y])
        derivative_error: ndarray = (proportional_error - self._last_error) / self._dt
        self._integral_error += self._dt * (proportional_error + self._last_error) / 2

        # Clip integral error
        self._integral_error: ndarray = clip(self._integral_error, -self._max_integral, self._max_integral)

        u: float = self._kp * proportional_error \
            + self._kd * derivative_error \
            + self._ki * self._integral_error

        R: ndarray = array([[cos(yaw), sin(yaw)],
                            [-sin(yaw), cos(yaw)]])
        
        u_robot = R @ u

        v_f = u_robot[0]

        omega: float = u_robot[1] / self._lookahead

        self._last_error = proportional_error
        return v_f, omega

    def _control_loop(self) -> None:
        if self._goal_pose is None:
            return

        # Get lookup. This should be replaced with input from a particle filter.
        try:
            when = Time()
            trans = self._tf_buffer.lookup_transform(self._to_frame, self._from_frame,
                                                     when, timeout=Duration(seconds=5.0))
        except LookupException:
            self.get_logger().info('Transform isn\'t available, waiting...')
            sleep(1)
            return

        # Current pose
        x: float = trans.transform.translation.x
        y: float = trans.transform.translation.y
        _, _, yaw = quaternion_to_euler(trans.transform.rotation)

        # Check for reset condition
        if x == self._start_x and y == self._start_y and yaw == self._start_yaw and \
            (self.get_clock().now() - self._goal_time) > Duration(seconds=self._dt * 3):
            # Reset!
            self._goal_pose = None
            return

        # Compute trailer hitch point in front of the agent.
        trailer_x: float = x + self._lookahead * cos(yaw)
        trailer_y: float = y + self._lookahead * sin(yaw)

        # Goal pose
        gx: float = self._goal_pose.pose.position.x
        gy: float = self._goal_pose.pose.position.y

        # Error calculation
        v_f, omega = self._compute_control(x=trailer_x, y=trailer_y, yaw=yaw, gx=gx, gy=gy)

        if norm([gx - x, gy - y]) < self._position_tolerance:
            v_f, omega = 0.0, 0.0
        else:
            v_f = max(min(v_f, self._max_linear_velocity), -self._max_linear_velocity)
            omega = max(min(omega, self._max_angular_velocity), -self._max_angular_velocity)

        # Publish cmd_vel
        self._publish_velocity_command(v_f=v_f, omega=omega)

        # # Publish PID debug info
        # debug_msg = Float64MultiArray()
        # debug_msg.data = [x, y, gx, gy, err_x, err_y, v_f, omega]
        # self._pid_debug_pub.publish(debug_msg)
        # self._csv_writer.writerow([self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9,
        #                            x, y, gx, gy, err_x, err_y, v_f, omega])


def main(args=None):
    rclpy_init(args=args)

    diffdrive_pid: DiffDrivePID = DiffDrivePID()

    rclpy_spin(diffdrive_pid)

    diffdrive_pid.destroy_node()
    rclpy_shutdown()

if __name__ == "__main__":
    main()