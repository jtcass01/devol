import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ROS 2 Messages
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray

from devol_localization.utils import odom_to_planar, pose_to_planar


@dataclass
class BodyBox:
    """Axis-aligned box of the robot body in base_footprint (z = 0 on the ground)."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float


# Clearpath A200 + stowed UR5e + Robotiq gripper, from TF at spawn in Gazebo
DEFAULT_BODY = [
    BodyBox(-0.495, 0.495, -0.335, 0.335, 0.05, 0.45),  # chassis, wheels, top plate
    BodyBox(-0.10, 0.10, -0.30, 0.30, 0.45, 0.82),  # 300 mm sensor arch + VLP-16
    BodyBox(0.08, 0.35, -0.10, 0.42, 0.45, 1.55),  # stowed UR5e + Robotiq, offset to +y
]


class LocalizationVisNode(Node):
    def __init__(self):
        super().__init__('localization_vis_node')

        self.declare_parameter('rate_hz', 10.0)
        self.rate_hz = self.get_parameter('rate_hz').value

        # State Variables
        self.map_grid = None
        self.map_extent = None  # [xmin, xmax, ymin, ymax] in meters

        self.gt_pose = None
        self.ekf_pose = None
        self.pf_pose = None
        self.dr_pose = None
        self.pf_particles = None

        # Subscribers
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, '/devol_drive/projected_map', self.map_callback, map_qos
        )
        self.create_subscription(Odometry, '/devol_drive/ground_truth/odom', self.gt_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/devol_drive/ekf_pose', self.ekf_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/devol_drive/pf_pose', self.pf_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/devol_drive/dr_pose', self.dr_callback, 10
        )  # Added DR sub
        self.create_subscription(PoseArray, '/devol_drive/pf_pose_cloud', self.cloud_callback, 10)

        # Plotting objects (assigned in main)
        self.fig = None
        self.ax = None

        self.get_logger().info(f'Visualization Node Started at {self.rate_hz} Hz')

    @property
    def all_algorithms_ready(self):
        return (
            self.gt_pose is not None
            and self.ekf_pose is not None
            and self.pf_pose is not None
            and self.dr_pose is not None
        )

    def map_callback(self, msg: OccupancyGrid):
        # Convert to visual grayscale
        grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        vis_map = np.full(grid.shape, 200, dtype=np.uint8)  # Unknown = Gray
        vis_map[grid == 0] = 255  # Free = White
        vis_map[grid > 50] = 0  # Obstacle = Black
        self.map_grid = vis_map

        # Calculate physical boundaries of the map in meters
        xmin = msg.info.origin.position.x
        ymin = msg.info.origin.position.y
        xmax = xmin + (msg.info.width * msg.info.resolution)
        ymax = ymin + (msg.info.height * msg.info.resolution)
        self.map_extent = [xmin, xmax, ymin, ymax]

    def gt_callback(self, msg: Odometry):
        self.gt_pose = odom_to_planar(msg)

    def ekf_callback(self, msg: PoseWithCovarianceStamped):
        self.ekf_pose = pose_to_planar(msg)

    def pf_callback(self, msg: PoseWithCovarianceStamped):
        self.pf_pose = pose_to_planar(msg)

    def dr_callback(self, msg: PoseWithCovarianceStamped):
        self.dr_pose = pose_to_planar(msg)

    def cloud_callback(self, msg: PoseArray):
        self.pf_particles = np.array([pose_to_planar(p) for p in msg.poses])

    def draw_robot(self, pose, color, alpha=0.6):
        """Draws the 2D bounding boxes of the robot for a given (x, y, yaw)."""
        if pose is None:
            return

        x, y, yaw = pose
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        # Plot each box footprint
        for box in DEFAULT_BODY:
            # 4 corners of the box in the robot's local frame
            corners_local = [
                (box.x_min, box.y_min),
                (box.x_max, box.y_min),
                (box.x_max, box.y_max),
                (box.x_min, box.y_max),
            ]

            # Transform to global map coordinates
            corners_global = []
            for lx, ly in corners_local:
                gx = x + lx * cos_yaw - ly * sin_yaw
                gy = y + lx * sin_yaw + ly * cos_yaw
                corners_global.append((gx, gy))

            poly = Polygon(
                corners_global,
                closed=True,
                facecolor=color,
                edgecolor='black',
                alpha=alpha,
                linewidth=1,
                zorder=5,
            )
            self.ax.add_patch(poly)

        # Draw a small heading line indicating the forward X axis
        front_x = x + 0.6 * cos_yaw
        front_y = y + 0.6 * sin_yaw
        self.ax.plot([x, front_x], [y, front_y], color='black', linewidth=1.5, zorder=6)

    def render(self):
        if self.map_grid is None or self.ax is None or not self.all_algorithms_ready:
            return
        self.ax.clear()

        # Draw Map directly in physical meters!
        self.ax.imshow(self.map_grid, cmap='gray', origin='lower', extent=self.map_extent)

        # Draw PF Particles
        if self.pf_particles is not None and len(self.pf_particles) > 0:
            self.ax.plot(
                self.pf_particles[:, 0],
                self.pf_particles[:, 1],
                '.',
                color='gray',
                markersize=2,
                alpha=0.5,
                label='PF Particles',
            )

        # Draw Ground Truth
        if self.gt_pose is not None:
            self.draw_robot(self.gt_pose, color='green', alpha=0.7)
            self.ax.plot([], [], color='green', linewidth=4, alpha=0.7, label='Ground Truth')

        # Draw EKF Estimate
        if self.ekf_pose is not None:
            self.draw_robot(self.ekf_pose, color='blue', alpha=0.6)
            self.ax.plot([], [], color='blue', linewidth=4, alpha=0.6, label='EKF')

        # Draw PF Estimate
        if self.pf_pose is not None:
            self.draw_robot(self.pf_pose, color='orange', alpha=0.6)
            self.ax.plot([], [], color='orange', linewidth=4, alpha=0.6, label='PF')

        # Draw Dead Reckoning Estimate
        if self.dr_pose is not None:
            self.draw_robot(self.dr_pose, color='red', alpha=0.6)
            self.ax.plot([], [], color='red', linewidth=4, alpha=0.6, label='Dead Reckoning')

        self.ax.set_title('Live Localization: EKF vs PF vs DR')
        self.ax.set_xlabel('Meters (X)')
        self.ax.set_ylabel('Meters (Y)')
        self.ax.legend(loc='upper right', bbox_to_anchor=(1.25, 1))

        # Flush GUI events
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationVisNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Setup Matplotlib
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title('Devol Localization Viz')
    plt.tight_layout(rect=[0, 0, 0.85, 1])

    node.fig = fig
    node.ax = ax

    # Precise timing loop adopted from reference
    period = 1.0 / max(node.rate_hz, 0.1)
    next_draw = None

    try:
        # plt.fignum_exists checks if the user closed the window manually!
        while rclpy.ok() and plt.fignum_exists(fig.number):
            # Calculate exactly how much time we have left until we need to draw again
            timeout = (
                0.05 if next_draw is None else max(0.0, min(0.05, next_draw - time.monotonic()))
            )

            # Let ROS process messages in the background
            executor.spin_once(timeout_sec=timeout)

            # Check if it's time to render the next frame
            now = time.monotonic()

            if not node.all_algorithms_ready or node.map_grid is None:
                next_draw = None
                continue

            if next_draw is None:
                next_draw = now

            if now >= next_draw:
                node.render()
                next_draw += period

                if next_draw < now:  # Prevent getting stuck behind
                    next_draw = now + period

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        plt.close(fig)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
