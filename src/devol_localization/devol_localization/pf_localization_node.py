from typing import Optional, Tuple
from scipy.ndimage import distance_transform_edt
from numpy import (
    ndarray,
    array,
    cov,
    float32 as np_float32,
    ones_like,
    searchsorted,
    cumsum,
    average,
    arctan2,
    cos,
    sin,
    copy as np_copy,
    arange,
    exp as np_exp,
    max as np_max,
    zeros,
)
from numpy.random import multivariate_normal, uniform, normal

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ROS 2 Messages
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose

# Import devol localization classes
from devol_localization.base_localization_node import BaseLocalizationNode
from devol_localization.laser_model import LaserModel
from devol_localization.utils import euler_to_quaternion
from devol_localization.motion_models import UnicycleMotionModel


class ParticleFilterNode(BaseLocalizationNode):
    def __init__(self):
        super().__init__('particle_filter_node', '/devol_drive/pf_pose')

        self.declare_parameter('num_particles', '500')

        self.map_dict = None
        self.num_particles: int = int(self.get_parameter('num_particles').value)

        # Initialize particles clustered around starting origin
        self.particles: ndarray = multivariate_normal(
            self.x_est, self.covariance, self.num_particles
        )
        self.laser_poses: ndarray

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

    def odom_callback(self, msg: Odometry):
        """Base callback to track the latest odometry twist."""
        self.v: float = msg.twist.twist.linear.x
        self.omega: float = msg.twist.twist.angular.z
        self.u: ndarray = array([self.v, self.omega])

        dt: Optional[float] = self.get_dt_from_header(msg.header)
        if dt is None:
            return

        self.particles, self.laser_poses = self._predict(dt=dt)

    def map_callback(self, msg: OccupancyGrid):
        """Converts ROS OccupancyGrid into a distance transform map for the Likelihood field"""
        if not self.is_map_new(msg):
            return

        # Convert 1D array to 2D numpy array
        grid: ndarray = array(msg.data, dtype=np_float32).reshape(
            (msg.info.height, msg.info.width)
        )

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
        # Wait for map and ensure odometry has ran at least once.
        if self.map_dict is None or not hasattr(self, 'laser_poses'):
            return

        z_dists, z_thetas = self.preprocess_scan(msg=msg)

        if len(z_dists) == 0:
            return  # No valid measurements

        # Run the Particle Filter using Odometry values and the Unicycle model
        self.particles, self.x_est = self._update(z_dists=z_dists, z_thetas=z_thetas)

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

    def _predict(self, dt: float) -> Tuple[ndarray, ndarray]:
        ## Apply the motion model to the samples (Prediction Step)
        # Create noise proportional to the velocity commands for each particle
        sigma_v: float = self.alpha_tv_causedby_tv * abs(self.v) + self.alpha_tv_causedby_rv * abs(
            self.omega
        )
        sigma_w: float = self.alpha_rv_causedby_tv * abs(self.v) + self.alpha_rv_causedby_rv * abs(
            self.omega
        )

        new_particles: ndarray = array(
            [
                UnicycleMotionModel.forward(
                    particle,
                    array([normal(self.v, sigma_v), normal(self.omega, sigma_w)]),
                    zeros(3),
                    dt,
                )
                for particle in self.particles
            ]
        )

        # Inject independent X and Y lateral slip noise directly to the array
        new_particles[:, 0] += normal(0.0, 0.005, self.num_particles)
        new_particles[:, 1] += normal(0.0, 0.005, self.num_particles)

        # Calculate where the laser actually is for each newly predicted particle
        laser_poses: ndarray = np_copy(new_particles)
        laser_offset: ndarray = self.get_laser_offset()
        # Shift X and Y based on the laser's physical mount and the robot's current yaw
        laser_poses[:, 0] += laser_offset[0] * cos(laser_poses[:, 2]) - laser_offset[1] * sin(
            laser_poses[:, 2]
        )
        laser_poses[:, 1] += laser_offset[0] * sin(laser_poses[:, 2]) + laser_offset[1] * cos(
            laser_poses[:, 2]
        )
        # Shift Yaw
        laser_poses[:, 2] += laser_offset[2]

        return new_particles, laser_poses

    def _update(self, z_dists: list, z_thetas: list):
        ## Update
        # Computes log probabilities
        log_weights = array(
            [
                LaserModel.likelihood_field_range_finder(
                    z_dists=z_dists, z_thetas=z_thetas, x=particle, map_dict=self.map_dict, std=0.1
                )
                for particle in self.laser_poses
            ],
            dtype=float,
        )

        # shift all weights so their highest value is 0.
        max_log_weights: ndarray = np_max(log_weights)
        weights: ndarray = np_exp(log_weights - max_log_weights)

        # Prevent division by zero
        weights += 1e-40
        # normalize (\eta)
        weights /= sum(weights)

        # Estimate the state based on the weighted average of particles
        x_est: ndarray = average(self.particles, axis=0, weights=weights)

        # Wrap angle
        x_est[2] = arctan2(
            average(sin(self.particles[:, 2]), weights=weights),
            average(cos(self.particles[:, 2]), weights=weights),
        )

        # Resample
        new_particles: ndarray = ParticleFilterNode.low_variance_resample(self.particles, weights)

        return new_particles, x_est

    @staticmethod
    def low_variance_resample(particles: ndarray, weights: ndarray) -> ndarray:
        N = len(particles)

        # Generate a single random start point between 0 and 1/N
        step = 1.0 / N
        pointers = uniform(0, step) + arange(N) * step

        # Find where those evenly spaced pointers land in the cumulative weight array
        cumulative_sum = cumsum(weights)
        indexes = searchsorted(cumulative_sum, pointers)

        return particles[indexes]


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
