import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ROS 2 Messages
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose

# Import devol localization classes
from devol_localization.base_localization_node import BaseLocalizationNode
from devol_localization.laser_model import ROSLaserModel
from devol_localization.particle_filter import particle_filter_2d, simple_resample
from devol_localization.utils import euler_to_quaternion


class ParticleFilterNode(BaseLocalizationNode):
    def __init__(self):
        super().__init__('particle_filter_node', '/devol_drive/pf_pose')

        self.declare_parameter('num_particles', 500)

        self.map_dict = None
        self.num_particles = self.get_parameter('num_particles').value

        # Initialize particles clustered around starting origin
        self.particles = np.random.multivariate_normal(
            self.x_est, np.diag([0.5, 0.5, 0.2]), self.num_particles
        )

        # Subscribers / Publishers
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.pub_particles = self.create_publisher(PoseArray, self.pose_topic + '_cloud', 10)

        self.get_logger().info('Particle Filter Node Started!')

    def map_callback(self, msg: OccupancyGrid):
        """Converts ROS OccupancyGrid into a distance transform map for the Likelihood field"""
        if not self.is_map_new(msg):
            return

        # Convert 1D array to 2D numpy array (row-major: Y is rows, X is cols)
        grid = np.array(msg.data, dtype=np.float32).reshape((msg.info.height, msg.info.width))

        # Create binary map where obstacle = 0, free space = 1
        binary_map = np.ones_like(grid)
        binary_map[grid > 50] = 0

        # Calculate Euclidean distance to nearest obstacle (in pixels)
        distance_grid = distance_transform_edt(binary_map)

        # Convert pixel distance to meters
        edt_map = distance_grid * msg.info.resolution

        # Store in dict to pass to ROSLaserModel easily
        self.map_dict = {'grid': edt_map, 'info': msg.info}
        self.get_logger().info('Map received and Euclidean Distance Transform computed.')

    def scan_callback(self, msg: LaserScan):
        """The main loop of the PF: computes dt, downsamples scan, runs PF, and publishes results."""
        if self.map_dict is None:
            return  # Wait for map

        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_scan_time is None:
            self.last_scan_time = current_time
            return
        dt = current_time - self.last_scan_time
        self.last_scan_time = current_time

        # Extract and Downsample LiDAR points
        step = max(1, len(msg.ranges) // 30)
        z_dists, z_thetas = [], []

        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max and not np.isinf(r) and not np.isnan(r):
                z_dists.append(r)
                z_thetas.append(msg.angle_min + i * msg.angle_increment)

        if len(z_dists) == 0:
            return  # No valid measurements

        # Handle Laser Offset (shift the particles forward so the rays cast from the laser)
        laser_offset = self.get_laser_offset(msg.header.frame_id)

        # Run the Particle Filter
        self.particles, self.x_est = particle_filter_2d(
            particles=self.particles,
            map_dict=self.map_dict,
            motion_model=self.diff_drive,
            sensor_model=ROSLaserModel,
            resample=simple_resample,
            u=self.u,
            dt=dt,
            z_dists=z_dists,
            z_thetas=z_thetas,
            laser_offset=laser_offset,
        )

        # Publish state and particles
        self.publish_state(msg.header.stamp)

    def publish_state(self, stamp):
        """Publishes the estimated pose and the particle cloud for RViz"""
        cov_3x3 = np.cov(self.particles, rowvar=False)
        self.publish_pose_and_tf(stamp, cov_3x3)

        # Publish Particle Cloud
        cloud_msg = PoseArray()
        cloud_msg.header.stamp = stamp
        cloud_msg.header.frame_id = self.map_frame
        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.orientation = euler_to_quaternion(p[2])
            cloud_msg.poses.append(pose)

        self.pub_particles.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
