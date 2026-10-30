#!/usr/bin/env python3

from typing import List
from threading import Lock, Thread
from numpy import array, ndarray, int8 as np_int8, full, cos, sin, uint8 as np_uint8
from zlib import crc32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Polygon
from dataclasses import dataclass
from message_filters import ApproximateTimeSynchronizer, Subscriber

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ROS 2 Messages
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PointStamped

from devol_localization.utils import odom_to_planar, pose_to_planar
from devol_localization.metrics_node import ErrorTracker


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


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
        super().__init__('localization_combined_vis_node')

        self.declare_parameter('rate_hz', 5.0)
        self.rate_hz = self.get_parameter('rate_hz').value

        # State Variables
        self.data_lock = Lock() # Protects data arrays across threads
        self.map_grid = None
        self.map_extent = None  # [xmin, xmax, ymin, ymax] in meters
        self.map_updated = False
        self.last_map_crc = None

        self.recording_started = False
        self.t0 = None  # Start time for x-axis zeroing

        self.gt_pose = None
        self.ekf_pose = None
        self.pf_pose = None
        self.pf_particles = None
        self.pf_num_particles = 0
        self.dr_pose = None

        # Initialize Trackers
        self.ekf = ErrorTracker('EKF', 'blue')
        self.pf = ErrorTracker('PF', 'orange')
        self.dr = ErrorTracker('DR', 'red')
        self.trackers = [self.ekf, self.pf, self.dr]

        self.runtimes = {'EKF': ([], []), 'PF': ([], []), 'DR': ([], [])}

        # Subscribers
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, '/devol_drive/projected_map', self.map_callback, map_qos
        )
        self.gt_sub: Subscriber = Subscriber(self, Odometry, '/devol_drive/ground_truth/odom')
        self.ekf_sub: Subscriber = Subscriber(
            self, PoseWithCovarianceStamped, '/devol_drive/ekf_pose'
        )
        self.ekf_rt_sub = Subscriber(self, PointStamped, '/devol_drive/ekf_pose/runtime')
        self.pf_sub: Subscriber = Subscriber(
            self, PoseWithCovarianceStamped, '/devol_drive/pf_pose'
        )
        self.pf_cloud_sub: Subscriber = Subscriber(self, PoseArray, '/devol_drive/pf_pose_cloud')
        self.pf_rt_sub = Subscriber(self, PointStamped, '/devol_drive/pf_pose/runtime')
        self.dr_sub: Subscriber = Subscriber(
            self, PoseWithCovarianceStamped, '/devol_drive/dr_pose'
        )
        self.dr_rt_sub = Subscriber(self, PointStamped, '/devol_drive/dr_pose/runtime')

        # Syncronize Subs
        self.ts = ApproximateTimeSynchronizer(
            [
                self.gt_sub,
                self.ekf_sub,
                self.ekf_rt_sub,
                self.pf_sub,
                self.pf_cloud_sub,
                self.pf_rt_sub,
                self.dr_sub,
                self.dr_rt_sub,
            ],
            queue_size=50,
            slop=1.0,
        )
        self.ts.registerCallback(self.sync_callback)

        # Plotting objects (assigned in main)
        self.fig = None
        self.ax_map = None
        self.axs = None
        self.lines = {}

        # Artist References for Map
        self.map_img = None
        self.pf_scatter = None
        self.gt_artists = None
        self.ekf_artists = None
        self.pf_artists = None
        self.dr_artists = None
        
        self.get_logger().info(f'Combined Visualization Node Started at {self.rate_hz} Hz')

    def get_sim_time(self):
        """Returns time in seconds. Initializes t0 on first call."""
        t = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    @property
    def all_algorithms_ready(self):
        return (
            self.gt_pose is not None
            and self.ekf_pose is not None
            and self.pf_pose is not None
            and self.dr_pose is not None
        )

    def is_map_new(self, msg: OccupancyGrid) -> bool:
        """Computes a CRC32 hash of the map data.
        Returns True if the map is new/changed, False if it is identical."""
        # Convert the map data array to continuous bytes and hash it
        current_crc = crc32(bytes(msg.data))

        if self.last_map_crc == current_crc:
            return False

        self.last_map_crc = current_crc
        return True
    
    def sync_callback(
        self,
        gt_msg: Odometry,
        ekf_msg: PoseWithCovarianceStamped,
        ekf_rt_msg: PointStamped,
        pf_msg: PoseWithCovarianceStamped,
        pf_cloud_msg: PoseArray,
        pf_rt_msg: PointStamped,
        dr_msg: PoseWithCovarianceStamped,
        dr_rt_msg: PointStamped,
    ):
        """This callback ONLY triggers when a matched set of timestamps arrives."""
        with self.data_lock:
            self.gt_pose = odom_to_planar(gt_msg)
            self.ekf_pose = pose_to_planar(ekf_msg)
            self.pf_particles = array([pose_to_planar(p) for p in pf_cloud_msg.poses])
            self.pf_num_particles = len(pf_cloud_msg.poses)
            self.pf_pose = pose_to_planar(pf_msg)
            self.dr_pose = pose_to_planar(dr_msg)

            if not self.recording_started and self.all_algorithms_ready:
                self.t0 = self.get_clock().now().nanoseconds / 1e9
                self.recording_started = True

            if self.recording_started:
                sim_time = self.get_sim_time()

                # Update trackers
                self.ekf.update(sim_time, self.ekf_pose, self.gt_pose)
                self.pf.update(sim_time, self.pf_pose, self.gt_pose)
                self.dr.update(sim_time, self.dr_pose, self.gt_pose)

                # Store runtimes locally
                self.runtimes['EKF'][0].append(sim_time)
                self.runtimes['EKF'][1].append(ekf_rt_msg.point.x)

                self.runtimes['PF'][0].append(sim_time)
                self.runtimes['PF'][1].append(pf_rt_msg.point.x)

                self.runtimes['DR'][0].append(sim_time)
                self.runtimes['DR'][1].append(dr_rt_msg.point.x)

    def map_callback(self, msg: OccupancyGrid):
        if not self.is_map_new(msg):
            return

        # Convert to visual grayscale
        grid: ndarray = array(msg.data, dtype=np_int8).reshape((msg.info.height, msg.info.width))
        vis_map: ndarray = full(grid.shape, 200, dtype=np_uint8)  # Unknown = Gray
        vis_map[grid == 0] = 255  # Free = White
        vis_map[grid > 50] = 0  # Obstacle = Black

        # Calculate physical boundaries of the map in meters
        xmin: float = msg.info.origin.position.x
        ymin: float = msg.info.origin.position.y
        xmax: float = xmin + (msg.info.width * msg.info.resolution)
        ymax: float = ymin + (msg.info.height * msg.info.resolution)

        with self.data_lock:
            self.map_grid = vis_map
            self.map_extent: List[float] = [xmin, xmax, ymin, ymax]
            self.map_updated = True

    def init_map_artists(self, ax_map):
        self.ax_map = ax_map
        self.ax_map.set_title('Live Localization')
        self.ax_map.set_xlabel('Meters (X)')
        self.ax_map.set_ylabel('Meters (Y)')

        # Image for the map (start with empty grid)
        self.map_img = self.ax_map.imshow([[0]], cmap='gray', origin='lower', vmin=0, vmax=255)

        # Scatter for PF Particles
        self.pf_scatter, = self.ax_map.plot([], [], '.', color='gray', markersize=2, alpha=0.5, label='PF Particles')

        # Helper to setup a robot tracker
        def _setup_robot(color, label):
            patches = []
            for _ in DEFAULT_BODY:
                # Provide a dummy 4-point polygon
                poly = Polygon([[0,0], [0,1], [1,1], [1,0]], closed=True, facecolor=color, edgecolor='black', alpha=0.6, zorder=5)
                poly.set_visible(False)
                self.ax_map.add_patch(poly)
                patches.append(poly)
            # Heading line
            line, = self.ax_map.plot([], [], color='black', linewidth=1.5, zorder=6)
            # Dummy line just for the legend
            self.ax_map.plot([], [], color=color, linewidth=4, alpha=0.6, label=label)
            return (patches, line)

        self.gt_artists = _setup_robot('green', 'Ground Truth')
        self.ekf_artists = _setup_robot('blue', 'EKF')
        self.pf_artists = _setup_robot('orange', 'PF')
        self.dr_artists = _setup_robot('red', 'Dead Reckoning')

        self.ax_map.legend(loc='upper right')

    def update_robot(self, pose, artists):
        """Update artist coordinates in place."""
        patches, line = artists
        if pose is None:
            for p in patches: p.set_visible(False)
            line.set_visible(False)
            return

        x, y, yaw = pose
        cos_yaw, sin_yaw = cos(yaw), sin(yaw)

        # Update Polygon vertices
        for i, box in enumerate(DEFAULT_BODY):
            corners_local = [
                (box.x_min, box.y_min),
                (box.x_max, box.y_min),
                (box.x_max, box.y_max),
                (box.x_min, box.y_max),
            ]
            corners_global = []
            for lx, ly in corners_local:
                gx = x + lx * cos_yaw - ly * sin_yaw
                gy = y + lx * sin_yaw + ly * cos_yaw
                corners_global.append((gx, gy))

            patches[i].set_xy(corners_global)
            patches[i].set_visible(True)

        # Update Heading Line
        front_x = x + 0.6 * cos_yaw
        front_y = y + 0.6 * sin_yaw
        line.set_data([x, front_x], [y, front_y])
        line.set_visible(True)

    def render(self, frame):
        if not self.recording_started:
            return

        with self.data_lock:
            if self.map_grid is not None and self.map_updated:
                self.map_img.set_data(self.map_grid)
                self.map_img.set_extent(self.map_extent)
                self.ax_map.set_xlim(self.map_extent[0], self.map_extent[1])
                self.ax_map.set_ylim(self.map_extent[2], self.map_extent[3])
                self.map_updated = False

            if self.pf_particles is not None and len(self.pf_particles) > 0:
                self.pf_scatter.set_data(self.pf_particles[:, 0], self.pf_particles[:, 1])

            self.update_robot(self.gt_pose, self.gt_artists)
            self.update_robot(self.ekf_pose, self.ekf_artists)
            self.update_robot(self.pf_pose, self.pf_artists)
            self.update_robot(self.dr_pose, self.dr_artists)

            if frame % 5 == 0:
                for i, tracker in enumerate(self.trackers):
                    if tracker.count == 0:
                        continue

                    l_err_p, l_rm_p, l_err_y, l_rm_y, l_rt = self.lines[tracker.name]

                    # Position Data
                    l_err_p.set_data(tracker.times, tracker.err_pos)
                    l_rm_p.set_data(tracker.times, tracker.rmse_pos)

                    # Yaw Data
                    l_err_y.set_data(tracker.times, tracker.err_yaw)
                    l_rm_y.set_data(tracker.times, tracker.rmse_yaw)

                    # Runtime Data
                    t_rt, y_rt = self.runtimes[tracker.name]
                    l_rt.set_data(t_rt, y_rt)

                    # Rescale axes bounds around new data bounds
                    self.axs[i][0].relim()
                    self.axs[i][0].autoscale_view(True, True, True)
                    self.axs[i][1].relim()
                    self.axs[i][1].autoscale_view(True, True, True)
                    self.axs[i][2].relim()
                    self.axs[i][2].autoscale_view(True, True, True)

                    # Dynamic titles
                    if tracker.name == 'PF':
                        self.axs[i][0].set_title(
                            f'{tracker.name} Pos Err (m) | Int: {tracker.integral_pos:.2f} | Parts: {self.pf_num_particles}'
                        )
                    else:
                        self.axs[i][0].set_title(
                            f'{tracker.name} Pos Err (m) | Int: {tracker.integral_pos:.2f}'
                        )

                    self.axs[i][1].set_title(
                        f'{tracker.name} Head Err (rad) | Int: {tracker.integral_yaw:.2f}'
                    )


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationVisNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    fig = plt.figure(figsize=(16, 8), dpi=100)
    fig.canvas.manager.set_window_title('Devol Localization Overview')

    gs = GridSpec(3, 4, width_ratios=[1.3, 1, 1, 1], figure=fig)
    ax_map = fig.add_subplot(gs[:, 0])
    node.init_map_artists(ax_map)

    # Right Side: 3x2 Grid for Metrics
    axs_plots = [
        [fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[0, 2]), fig.add_subplot(gs[0, 3])],
        [fig.add_subplot(gs[1, 1]), fig.add_subplot(gs[1, 2]), fig.add_subplot(gs[1, 3])],
        [fig.add_subplot(gs[2, 1]), fig.add_subplot(gs[2, 2]), fig.add_subplot(gs[2, 3])],
    ]

    for i, tracker in enumerate(node.trackers):
        (l_err_p,) = axs_plots[i][0].plot([], [], color=tracker.color, alpha=0.3, label='Inst Error')
        (l_rm_p,) = axs_plots[i][0].plot([], [], color=tracker.color, linewidth=2, label='RMSE')
        axs_plots[i][0].grid(True, linestyle='--', alpha=0.6)
        axs_plots[i][0].legend(loc='upper left')

        (l_err_y,) = axs_plots[i][1].plot([], [], color=tracker.color, alpha=0.3, label='Inst Error')
        (l_rm_y,) = axs_plots[i][1].plot([], [], color=tracker.color, linewidth=2, label='RMSE')
        axs_plots[i][1].grid(True, linestyle='--', alpha=0.6)
        axs_plots[i][1].legend(loc='upper left')

        (l_rt,) = axs_plots[i][2].plot([], [], color=tracker.color, linewidth=2, label='Runtime')
        axs_plots[i][2].grid(True, linestyle='--', alpha=0.6)
        axs_plots[i][2].legend(loc='upper left')

        node.lines[tracker.name] = (l_err_p, l_rm_p, l_err_y, l_rm_y, l_rt)

    axs_plots[2][0].set_xlabel('Time (s)')
    axs_plots[2][1].set_xlabel('Time (s)')
    axs_plots[2][2].set_xlabel('Time (s)')
    plt.tight_layout(rect=[0, 0, 1, 0.98])
    node.axs = axs_plots

    update_interval_ms = int(1000.0 / max(node.rate_hz, 0.1))
    ani = FuncAnimation(
        fig, 
        node.render, 
        interval=update_interval_ms, 
        blit=False, 
        cache_frame_data=False
    )

    try:
        plt.show()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()