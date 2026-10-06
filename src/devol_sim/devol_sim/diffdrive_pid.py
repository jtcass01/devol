from time import sleep
from typing import Optional, Tuple
from csv import writer
from tty import setcbreak
from sys import stdin
from termios import tcsetattr, TCSADRAIN, tcgetattr
from threading import Thread, Lock

from numpy import ndarray, array, pi, cos, sin, zeros, clip
from numpy.linalg import norm

from rclpy import spin as rclpy_spin, init as rclpy_init, shutdown as rclpy_shutdown
from rclpy.node import Node, Timer, Subscription, Publisher
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import TransformListener, Buffer, LookupException
from std_msgs.msg import Float64MultiArray

from devol_sim.utils import quaternion_to_euler

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]"""
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

class KeyboardHandler:
    """Handle keyboard input in a separate thread when tuning."""
    def __init__(self):
        self.key_buffer = []
        self.lock = Lock()
        self.running = True
        self.old_settings = None
        
        # Only initialize if we have a proper terminal
        if stdin.isatty():
            self.old_settings = tcgetattr(stdin)
            self.thread = Thread(target=self._read_keys, daemon=True)
            self.thread.start()
        else:
            print("Warning: Not running in a terminal. Keyboard input disabled.")
    
    def _read_keys(self):
        """Thread function to continuously read keys"""
        try:
            setcbreak(stdin.fileno())
            while self.running:
                if stdin in stdin.read_ready() if hasattr(stdin, 'read_ready') else True:
                    key = stdin.read(1)
                    if key:
                        # Handle escape sequences (arrow keys)
                        if key == '\x1b':
                            next_chars = stdin.read(2)
                            key = key + next_chars
                        
                        with self.lock:
                            self.key_buffer.append(key)
        except:
            pass
        finally:
            if self.old_settings:
                tcsetattr(stdin, TCSADRAIN, self.old_settings)
    
    def get_key(self):
        """Get the next key from the buffer, or None if empty"""
        with self.lock:
            if self.key_buffer:
                return self.key_buffer.pop(0)
        return None
    
    def __del__(self):
        self.running = False
        if self.old_settings:
            try:
                tcsetattr(stdin, TCSADRAIN, self.old_settings)
            except:
                pass


class DiffDrivePID(Node):
    # Subscribes: 
    #   /tf
    #   /goal_pose
    # Publishes:
    #   /cmd_vel : geometry_msgs/Twist : This is the desired velocity (linear and angular) for the centerpoint of the robot

    def __init__(self):
        super().__init__('diffdrive_pid')

        # Declare parameters with default values
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('lookahead', 0.5)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', pi)
        self.declare_parameter('position_tolerance', 0.005)
        self.declare_parameter('yaw_tolerance', 0.05)
        self.declare_parameter('max_integral', 1.0)
        self.declare_parameter('tune', True)
        self.declare_parameter('x', 0.0) # Starting x position
        self.declare_parameter('y', -3.0) # Starting y position
        self.declare_parameter('yaw', 1.57) # Starting yaw position
        self.declare_parameter('namespace', '/devol_drive')
        
        self._kp: float = self.get_parameter('kp').get_parameter_value().double_value
        self._ki: float = self.get_parameter('ki').get_parameter_value().double_value
        self._kd: float = self.get_parameter('kd').get_parameter_value().double_value
        self._lookahead: float = self.get_parameter('lookahead').get_parameter_value().double_value
        self._publish_rate: float = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._max_linear_velocity: float = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self._max_angular_velocity: float = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self._max_integral: float = self.get_parameter('max_integral').get_parameter_value().double_value
        self._position_tolerance: float = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self._yaw_tolerance: float = self.get_parameter('yaw_tolerance').get_parameter_value().double_value
        self._start_x: float = float(self.get_parameter('x').get_parameter_value().double_value)
        self._start_y: float = float(self.get_parameter('y').get_parameter_value().double_value)
        self._start_yaw: float = float(self.get_parameter('yaw').get_parameter_value().double_value)
        self._namespace: str = str(self.get_parameter('namespace').get_parameter_value().string_value)
        self._is_tuning: bool = bool(self.get_parameter('tune').get_parameter_value().bool_value)

        self._dt: float = 1.0 / self._publish_rate
        self._integral_error = zeros(3)
        self._last_error = zeros(3)
        self._tf_buffer: Buffer = Buffer()
        self._tf_listener: TransformListener = TransformListener(self._tf_buffer, self)
        self._goal_pose: Optional[PoseStamped] = None
        self._goal_time: Optional[Time] = None

        self._goal_pose_sub: Subscription = \
            self.create_subscription(PoseStamped, f'{self._namespace}/goal_pose', 
                                     self._goal_pose_callback,
                                     10)
        self._cmd_vel_pub: Publisher = self.create_publisher(Twist, f'{self._namespace}/cmd_vel', 10)

        self._timer: Timer = self.create_timer(self._dt, self._control_loop)

        self._from_frame: str = f'{self._namespace[1:]}/a200_base_link'
        self._to_frame: str = 'map'

        if self._is_tuning:
            self._keyboard_handler = KeyboardHandler()
            self._scale: float = 0.1  # Current adjustment scale
            self._selected_variable: str = 'P'  # Current selected variable: 'P', 'I', or 'D'
            self._print_status()

            # Extra DEBUG stuff left in -- just in case you are curious how pid was tuned.
            # self._pid_debug_pub: Publisher = self.create_publisher(Float64MultiArray, 'pid_debug', 10)
            # self._csv_file = open("pid_log.csv", "w", newline="")
            # self._csv_writer = writer(self._csv_file)
            # self._csv_writer.writerow([
            #     "time", "x", "y", "gx", "gy", 
            #     "error_x", "error_y", 
            #     "v_f", "omega"
            # ])

    def _print_status(self):
        """Print current PID values and control settings. USED ONLY WHEN TUNING"""
        print("\n" + "="*60)
        print(f"Selected Variable: {self._selected_variable}")
        print(f"Adjustment Scale: {self._scale}")
        print(f"Kp: {self._kp:.4f}  |  Ki: {self._ki:.4f}  |  Kd: {self._kd:.4f}")
        print("="*60)
        print("Controls:")
        print("  1/2/3: Select P/I/D variable")
        print("  Up/Down: Change scale (0.01, 0.1, 1.0)")
        print("  Left/Right: Decrease/Increase selected variable by scale")
        print("  q: Quit")
        print("="*60 + "\n")

    def _handle_keyboard_input(self):
        """Process keyboard input to adjust PID parameters"""
        key = self._keyboard_handler.get_key()
        if key is None:
            return
        
        # Handle quit
        if key == 'q' or key == 'Q':
            self.get_logger().info("Quit command received")
            rclpy_shutdown()
            return
        
        # Select variable (1, 2, 3)
        if key == '1':
            self._selected_variable = 'P'
            self._print_status()
        elif key == '2':
            self._selected_variable = 'I'
            self._print_status()
        elif key == '3':
            self._selected_variable = 'D'
            self._print_status()
        
        # Change scale (Up/Down arrows)
        elif key == '\x1b[A':  # Up arrow
            if self._scale == 0.01:
                self._scale = 0.1
            elif self._scale == 0.1:
                self._scale = 1.0
            elif self._scale == 1.0:
                self._scale = 0.01
            self._print_status()
        elif key == '\x1b[B':  # Down arrow
            if self._scale == 0.01:
                self._scale = 1.0
            elif self._scale == 0.1:
                self._scale = 0.01
            elif self._scale == 1.0:
                self._scale = 0.1
            self._print_status()
        
        # Adjust selected variable (Left/Right arrows)
        elif key == '\x1b[D':  # Left arrow - decrease
            if self._selected_variable == 'P':
                self._kp = max(0.0, self._kp - self._scale)
            elif self._selected_variable == 'I':
                self._ki = max(0.0, self._ki - self._scale)
            elif self._selected_variable == 'D':
                self._kd = max(0.0, self._kd - self._scale)
            self._print_status()
        elif key == '\x1b[C':  # Right arrow - increase
            if self._selected_variable == 'P':
                self._kp += self._scale
            elif self._selected_variable == 'I':
                self._ki += self._scale
            elif self._selected_variable == 'D':
                self._kd += self._scale
            self._print_status()

    def _publish_velocity_command(self, v_f: ndarray, omega: ndarray) -> None:
        cmd_vel: Twist = Twist()
        cmd_vel.linear.x = v_f
        cmd_vel.angular.z = omega
        self._cmd_vel_pub.publish(cmd_vel)

    def _goal_pose_callback(self, msg: PoseStamped) -> None:
        self._goal_pose = msg
        self._goal_time = self.get_clock().now()

    def _compute_control(self, x, y, yaw, gx, gy, gyaw) -> Tuple[float, float]:
        # Position control (XY)
        trailer_x: float = x + self._lookahead * cos(yaw)
        trailer_y: float = y + self._lookahead * sin(yaw)

        #  Calculate Errors
        position_error_norm: float = norm([gx-trailer_x, gy-trailer_y])
        proportional_error: ndarray = array([gx - trailer_x, gy - trailer_y, normalize_angle(gyaw - yaw)])
        derivative_error: ndarray = (proportional_error - self._last_error) / self._dt
        self._integral_error += self._dt * (proportional_error + self._last_error) / 2

        # Clip integral error
        self._integral_error: ndarray = clip(self._integral_error, -self._max_integral, self._max_integral)

        u: float = self._kp * proportional_error \
            + self._kd * derivative_error \
            + self._ki * self._integral_error

        R: ndarray = array([[cos(yaw), sin(yaw)],
                            [-sin(yaw), cos(yaw)]])
        
        u_robot_xy = R @ u[0:2] # only transform x,y components
        v_f = u_robot_xy[0]

        if position_error_norm < self._position_tolerance * 3:  # Close to position goal
            yaw_error = abs(normalize_angle(gyaw - yaw))
            if yaw_error > self._yaw_tolerance:
                # Keep some forward motion to allow orientation correction
                min_v_f = 0.1  # Minimum forward velocity for orientation correction
                if abs(v_f) < min_v_f:
                    v_f = min_v_f if v_f >= 0 else -min_v_f

        # Combined position-based steering and yaw control
        omega_position: float = u_robot_xy[1] / self._lookahead
        omega_yaw: float = u[2]
        omega: float = omega_position + omega_yaw

        self._last_error = proportional_error
        return v_f, omega

    def _control_loop(self) -> None:
        if self._is_tuning:
            self._handle_keyboard_input()
            # # Publish PID debug info
            # debug_msg = Float64MultiArray()
            # debug_msg.data = [x, y, gx, gy, err_x, err_y, v_f, omega]
            # self._pid_debug_pub.publish(debug_msg)
            # self._csv_writer.writerow([self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9,
            #                            x, y, gx, gy, err_x, err_y, v_f, omega])

        if self._goal_pose is None:
            return

        # Get lookup. This should be replaced with input from a particle filter.
        try:
            when = Time()
            trans = self._tf_buffer.lookup_transform(self._to_frame, self._from_frame,
                                                     when, timeout=Duration(seconds=5.0))
        except LookupException:
            self.get_logger().info(f'Transform ({self._from_frame} -> {self._to_frame}) isn\'t available, waiting...')
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

        # Goal pose
        gx: float = self._goal_pose.pose.position.x
        gy: float = self._goal_pose.pose.position.y
        _, _, gyaw = quaternion_to_euler(self._goal_pose.pose.orientation)

        # Error calculation
        v_f, omega = self._compute_control(x=x, y=y, yaw=yaw, gx=gx, gy=gy, gyaw=gyaw)

        position_error = norm([gx - x, gy - y])
        yaw_error = abs(normalize_angle(gyaw - yaw))

        if position_error < self._position_tolerance and yaw_error < self._yaw_tolerance:
            v_f, omega = 0.0, 0.0
        else:
            v_f = max(min(v_f, self._max_linear_velocity), -self._max_linear_velocity)
            omega = max(min(omega, self._max_angular_velocity), -self._max_angular_velocity)

        # Publish cmd_vel
        self._publish_velocity_command(v_f=v_f, omega=omega)

def main(args=None):
    rclpy_init(args=args)

    diffdrive_pid: DiffDrivePID = DiffDrivePID()

    rclpy_spin(diffdrive_pid)

    diffdrive_pid.destroy_node()
    rclpy_shutdown()

if __name__ == "__main__":
    main()
