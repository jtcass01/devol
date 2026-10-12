import time
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ROS 2 Messages
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray


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
        self.create_subscription(Odometry, '/devol_drive/odom', self.gt_callback, 10)
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
        self.gt_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def ekf_callback(self, msg: PoseWithCovarianceStamped):
        self.ekf_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def pf_callback(self, msg: PoseWithCovarianceStamped):
        self.pf_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def dr_callback(self, msg: PoseWithCovarianceStamped):
        self.dr_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def cloud_callback(self, msg: PoseArray):
        self.pf_particles = np.array([[p.position.x, p.position.y] for p in msg.poses])

    def render(self):
        if self.map_grid is None or self.ax is None:
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
            self.ax.plot(
                self.gt_pose[0], self.gt_pose[1], 'g*', markersize=12, label='Ground Truth'
            )

        # Draw EKF Estimate
        if self.ekf_pose is not None:
            self.ax.plot(self.ekf_pose[0], self.ekf_pose[1], 'bo', markersize=6, label='EKF')

        # Draw PF Estimate
        if self.pf_pose is not None:
            self.ax.plot(
                self.pf_pose[0], self.pf_pose[1], 's', color='orange', markersize=6, label='PF'
            )

        # Draw Dead Reckoning Estimate
        if self.pf_pose is not None:
            self.ax.plot(
                self.dr_pose[0], self.dr_pose[1], 'r^', markersize=6, label='Dead Reckoning'
            )

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
    plt.tight_layout()

    node.fig = fig
    node.ax = ax

    # Precise timing loop adopted from reference
    period = 1.0 / max(node.rate_hz, 0.1)
    next_draw = time.monotonic()

    try:
        # plt.fignum_exists checks if the user closed the window manually!
        while rclpy.ok() and plt.fignum_exists(fig.number):
            # Calculate exactly how much time we have left until we need to draw again
            timeout = max(0.0, min(0.05, next_draw - time.monotonic()))

            # Let ROS process messages in the background
            executor.spin_once(timeout_sec=timeout)

            # Check if it's time to render the next frame
            now = time.monotonic()
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
