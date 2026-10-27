from typing import Optional
import numpy as np
from scipy.ndimage import distance_transform_edt
from numpy import (ndarray, array, diag, cov, 
                   float32 as np_float32, ones_like, deg2rad)
from numpy.random import multivariate_normal

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ROS 2 Messages
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose

# Import devol localization classes
from devol_localization.base_localization_node import BaseLocalizationNode
from devol_localization.laser_model import LaserModel
from devol_localization.particle_filter import particle_filter_2d, low_variance_resample
from devol_localization.utils import euler_to_quaternion
from devol_localization.motion_models import UnicycleMotionModel

class ParticleFilterNode(BaseLocalizationNode):
    def __init__(self):
        super().__init__('particle_filter_node', '/devol_drive/pf_pose')

        self.declare_parameter('num_particles', '500')

        self.map_dict = None
        self.num_particles: int = int(self.get_parameter('num_particles').value)

        self.covariance: ndarray = diag([0.1**2, 0.1**2, deg2rad(5.0) ** 2])
        self.V: ndarray = diag([0.05**2, 0.05**2, deg2rad(2.0) ** 2])

        # Initialize particles clustered around starting origin
        self.particles: ndarray = multivariate_normal(
            self.x_est, self.covariance, self.num_particles
        )

        # Subscribers / Publishers
        map_qos: QoSProfile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 1)
        
        self.pub_particles = self.create_publisher(PoseArray, self.pose_topic + '_cloud', 10)

        self.get_logger().info('Particle Filter Node Started!')

    def map_callback(self, msg: OccupancyGrid):
        """Converts ROS OccupancyGrid into a distance transform map for the Likelihood field"""
        if not self.is_map_new(msg):
            return

        # Convert 1D array to 2D numpy array
        grid: ndarray = array(msg.data, dtype=np_float32).reshape((msg.info.height, msg.info.width))

        # Create binary map where obstacle = 0, free space = 1
        binary_map: ndarray = ones_like(grid)
        binary_map[grid > 50] = 0

        # Calculate Euclidean distance to nearest obstacle (in pixels)
        distance_grid: ndarray = distance_transform_edt(binary_map)
        edt_map: ndarray = distance_grid * msg.info.resolution

        # Store in dict to pass to LaserModel easily
        self.map_dict: dict = {'grid': edt_map, 'info': msg.info}
        self.get_logger().info('Map received and Euclidean Distance Transform computed.')

    def scan_callback(self, msg: LaserScan):
        """The main loop of the PF: computes dt, downsamples scan, runs PF, and publishes results."""
        if self.map_dict is None:
            return  # Wait for map

        dt: Optional[float] = self.get_dt_from_header(msg.header)
        if dt is None:
            return

        z_dists, z_thetas = self.preprocess_scan(msg=msg)

        if len(z_dists) == 0:
            return  # No valid measurements

        # Handle Laser Offset (shift the particles forward so the rays cast from the laser)
        laser_offset: ndarray = self.get_laser_offset()

        # Run the Particle Filter using Odometry values and the Unicycle model
        self.particles, self.x_est = particle_filter_2d(
            particles=self.particles,
            map_dict=self.map_dict,
            motion_model=UnicycleMotionModel,
            sensor_model=LaserModel,
            resample=low_variance_resample,
            u=array([self.v, self.omega]),
            dt=dt,
            z_dists=z_dists,
            z_thetas=z_thetas,
            laser_offset=laser_offset,
            V=self.V
        )

        # Publish state and particles
        self.publish_state(msg.header.stamp)

    def publish_state(self, stamp):
        """Publishes the estimated pose and the particle cloud for RViz"""
        cov_3x3: ndarray = cov(self.particles, rowvar=False)
        self.publish_pose_and_tf(stamp, cov_3x3)

        # Publish Particle Cloud
        cloud_msg: PoseArray = PoseArray()
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