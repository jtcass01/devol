#!/usr/bin/env python3

from typing import Optional
from time import perf_counter
from scipy.ndimage import distance_transform_edt
from numpy import (
    ndarray,
    array,
    cov,
    float32 as np_float32,
    ones_like,
    ones,
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
    sum as np_sum,
    zeros,
    vstack,
    where,
    pi,
    mean as np_mean,
)

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ROS 2 Messages
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose

# Import devol localization classes
from devol_localization.base_localization_node import BaseLocalizationNode
from devol_localization.laser_model import LaserModel
from devol_localization.motion_models import UnicycleMotionModel
from devol_localization.utils import euler_to_quaternion


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


class ParticleFilterNode(BaseLocalizationNode):
    def __init__(self):
        super().__init__(
            'particle_filter_node',
            '/devol_drive/pf_pose',
            runtime_topic='/devol_drive/pf_pose/runtime',
        )

        self.declare_parameter('num_particles', 500)
        self.declare_parameter('z_hit', 1.0)
        self.declare_parameter('z_rand', 1e-6)
        self.declare_parameter('z_max', 0.0)
        self.declare_parameter('kappa', 1.0)
        self.declare_parameter('sigma_hit', 0.25)
        self.declare_parameter('alpha_slow', 0.001)
        self.declare_parameter('alpha_fast', 0.1)
        self.declare_parameter('max_escape_fraction', 0.05)
        self.declare_parameter('neff_threshold', 0.5)

        self.map_dict = None

        self.num_particles: int = self.get_parameter('num_particles').value
        self.z_hit: float = self.get_parameter('z_hit').value
        self.z_rand: float = self.get_parameter('z_rand').value
        self.z_max: float = self.get_parameter('z_max').value
        self.kappa: float = self.get_parameter('kappa').value
        self.sigma_hit: float = self.get_parameter('sigma_hit').value
        self.alpha_fast: float = self.get_parameter('alpha_fast').value
        self.alpha_slow: float = self.get_parameter('alpha_slow').value
        self.max_escape_fraction: float = self.get_parameter('max_escape_fraction').value
        self.neff_threshold: float = self.get_parameter('neff_threshold').value

        # Initialize particles clustered around starting origin
        self.particles: ndarray = self.rng.multivariate_normal(
            self.x_est, self.covariance, self.num_particles
        )
        self.weights: ndarray = ones(self.num_particles) / self.num_particles

        # Adaptive injection moving averages
        self.slow_avg_lik = 0.0
        self.fast_avg_lik = 0.0

        # Total runtime accumulator
        self.localization_runtime = 0.0

        # Subscribers / Publishers
        map_qos: QoSProfile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 1)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.pub_particles = self.create_publisher(PoseArray, self.pose_topic + '_cloud', 10)

        self.get_logger().info('Particle Filter Node Started!')

    def odom_callback(self, msg: Odometry):
        """Base callback to track the latest odometry twist."""
        start_time: float = perf_counter()
        self.v: float = msg.twist.twist.linear.x
        self.omega: float = msg.twist.twist.angular.z
        self.u: ndarray = array([self.v, self.omega])

        dt: Optional[float] = self.get_dt_from_header(msg.header)
        if dt is None:
            return

        self.particles = self._predict(dt=dt)
        self.localization_runtime += perf_counter() - start_time

    def map_callback(self, msg: OccupancyGrid):
        """Converts ROS OccupancyGrid into a distance transform map for the Likelihood field"""
        start_time: float = perf_counter()

        if not self.is_map_new(msg):
            self.localization_runtime += perf_counter() - start_time
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

        # Pre-compute free space coordinates for the escape hatch
        free_y, free_x = where(binary_map == 1)
        res: float = msg.info.resolution
        ox: float = msg.info.origin.position.x
        oy: float = msg.info.origin.position.y

        # Convert pixel indices to real-world coordinates
        free_world_x: ndarray = free_x * res + ox + (res / 2.0)
        free_world_y: ndarray = free_y * res + oy + (res / 2.0)

        # Store in dict to pass to LaserModel easily
        self.map_dict: dict = {
            'grid': edt_map,
            'info': msg.info,
            'free_x': free_world_x,
            'free_y': free_world_y,
        }
        self.get_logger().info('Map received and Euclidean Distance Transform computed.')
        self.localization_runtime += perf_counter() - start_time

    def scan_callback(self, msg: LaserScan):
        """The main loop of the PF: computes dt, downsamples scan, runs PF, and publishes results."""
        start_time: float = perf_counter()

        # Wait for map and ensure odometry has ran at least once.
        if self.map_dict is None:
            self.localization_runtime += perf_counter() - start_time
            return

        if self.z_max <= 0.0:
            self.z_max: float = float(msg.range_max)

        z_dists, z_thetas = self.preprocess_scan(msg=msg)

        if len(z_dists) == 0:
            self.localization_runtime += perf_counter() - start_time
            return  # No valid measurements

        # Run the Particle Filter using Odometry values and the Unicycle model
        self.particles, self.x_est = self._update(z_dists=z_dists, z_thetas=z_thetas)
        self.localization_runtime += perf_counter() - start_time

        # Publish state and particles
        self.publish_state(msg.header.stamp)

    def publish_state(self, stamp):
        """Publishes the estimated pose and the particle cloud for RViz"""
        cov_3x3: ndarray = cov(self.particles, rowvar=False)

        # Publish state
        self.publish_pose_and_tf_runtime(
            stamp=stamp, cov_3x3=cov_3x3, start_time=0.0, end_time=self.localization_runtime
        )

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

        # Reset runtime for the next cycle
        self.localization_runtime = 0.0

    def _predict(self, dt: float) -> ndarray:
        ## Apply the motion model to the samples (Prediction Step)
        # Create noise proportional to the velocity commands for each particle
        sigma_v: float = self.alpha_tv_causedby_tv * abs(self.v) + self.alpha_tv_causedby_rv * abs(
            self.omega
        )
        sigma_w: float = self.alpha_rv_causedby_tv * abs(self.v) + self.alpha_rv_causedby_rv * abs(
            self.omega
        )

        # Add a small floor (0.02) to heading noise to maintain diversity when stopped
        sigma_w = max(0.02, sigma_w)

        # Draw noise applied to the distance and heading moved using sqrt(dt)
        sqrt_dt = dt**0.5

        # Calculate nominal distances and headings moved in this timestep
        dist_moved = self.v * dt
        heading_moved = self.omega * dt

        # Vectorized generation of noisy movements (std_dev = sigma * sqrt(dt))
        noisy_dists = self.rng.normal(dist_moved, sigma_v * sqrt_dt, self.num_particles)
        noisy_headings = self.rng.normal(heading_moved, sigma_w * sqrt_dt, self.num_particles)

        # Convert back to effective velocities since UnicycleMotionModel expects velocity inputs
        noisy_vs = noisy_dists / dt
        noisy_ws = noisy_headings / dt

        # Apply the forward model using the noisy velocities for each particle
        new_particles: ndarray = array(
            [
                UnicycleMotionModel.forward(
                    self.particles[i],
                    array([noisy_vs[i], noisy_ws[i]]),
                    zeros(3),
                    dt,
                )
                for i in range(self.num_particles)
            ]
        )

        # Inject independent X and Y lateral slip noise directly to the array
        new_particles[:, 0] += self.rng.normal(0.0, 0.005, self.num_particles)
        new_particles[:, 1] += self.rng.normal(0.0, 0.005, self.num_particles)

        return new_particles

    def _update(self, z_dists: list, z_thetas: list):
        # LiDAR pose for each particle
        laser_poses: ndarray = np_copy(self.particles)
        laser_offset: ndarray = self.get_laser_offset()
        laser_poses[:, 0] += laser_offset[0] * cos(laser_poses[:, 2]) - laser_offset[1] * sin(
            laser_poses[:, 2]
        )
        laser_poses[:, 1] += laser_offset[0] * sin(laser_poses[:, 2]) + laser_offset[1] * cos(
            laser_poses[:, 2]
        )
        laser_poses[:, 2] += laser_offset[2]

        # Measurement likelihood
        d: ndarray = LaserModel.endpoint_distances(laser_poses, z_dists, z_thetas, self.map_dict)
        log_lik: ndarray = LaserModel.log_likelihood_mixture(
            d, self.sigma_hit, self.z_hit, self.z_rand, self.z_max
        )

        # Injection statistic
        avg_raw_likelihood: float = np_mean(np_exp(log_lik / d.shape[1]))
        if self.slow_avg_lik == 0.0:
            self.slow_avg_lik = avg_raw_likelihood
            self.fast_avg_lik = avg_raw_likelihood
        self.slow_avg_lik += self.alpha_slow * (avg_raw_likelihood - self.slow_avg_lik)
        self.fast_avg_lik += self.alpha_fast * (avg_raw_likelihood - self.fast_avg_lik)

        # Weight update
        log_weights: ndarray = log_lik / self.kappa
        self.weights *= np_exp(log_weights - np_max(log_weights))
        self.weights += 1e-40
        self.weights /= np_sum(self.weights)
        n_eff: float = float(1.0 / np_sum(self.weights**2))

        # Weighted state estimate and covariance
        x_est, self.cov_est = self.weighted_pose_estimate(particles=self.particles, 
                                                          weights=self.weights)

        # Augmented MCL injection fraction
        escape_fraction: float = 0.0
        if self.slow_avg_lik > 0.0:
            escape_fraction: float = max(0.0, 1.0 - self.fast_avg_lik / self.slow_avg_lik)
        escape_fraction: float = min(self.max_escape_fraction, escape_fraction)
        num_random: int = int(self.num_particles * escape_fraction)

        resampled: bool = (n_eff < self.neff_threshold * self.num_particles) or (num_random > 0)        
        if resampled:
            new_particles = self.resample_with_injection(num_random, x_est)
            self.weights = ones(self.num_particles) / self.num_particles
        else:
            new_particles = self.particles

        self.last_diagnostics = (n_eff, self.fast_avg_lik, self.slow_avg_lik, escape_fraction, resampled)
        return new_particles, x_est

    def resample(
        self, particles: ndarray, weights: ndarray, num_samples: Optional[int] = None
    ) -> ndarray:
        N = len(particles) if num_samples is None else num_samples

        # Generate a single random start point between 0 and 1/N
        step = 1.0 / N
        pointers = self.rng.uniform(0, step) + arange(N) * step

        # Find where those evenly spaced pointers land in the cumulative weight array
        cumulative_sum = cumsum(weights)
        indexes = searchsorted(cumulative_sum, pointers)

        return particles[indexes]

    def resample_with_injection(self, num_random: int, x_est: ndarray) -> ndarray:
        num_random: int = max(0, min(num_random, self.num_particles))
        num_normal: int = self.num_particles - num_random

        resampled: ndarray = self.resample(particles=self.particles, 
                                           weights=self.weights, 
                                           num_samples=num_normal)

        if num_random == 0:
            return resampled

        injected: ndarray = self.sample_free_space(num_random, x_est)
        return vstack((resampled, injected))

    def sample_free_space(self, n: int, x_est: ndarray) -> ndarray:
        free_x: ndarray = self.map_dict.get('free_x', array([]))
        free_y: ndarray = self.map_dict.get('free_y', array([]))

        if len(free_x) == 0:
            samples: ndarray = self.rng.multivariate_normal(x_est, self.cov_est, n)
            samples[:, 2] = arctan2(sin(samples[:,2]), cos(samples[:,2]))
            return samples

        res: float = self.map_dict['info'].resolution
        idx: ndarray = self.rng.choice(len(free_x), size=n, replace=True)

        samples = zeros((n, 3))
        samples[:, 0] = free_x[idx] + self.rng.uniform(-0.5*res, 0.5*res, n)
        samples[:, 1] = free_y[idx] + self.rng.uniform(-0.5*res, 0.5*res, n)
        samples[:, 2] = self.rng.uniform(-pi, pi, n)

        return samples

    @staticmethod
    def weighted_pose_estimate(particles: ndarray, weights: ndarray):
        x_est: ndarray = average(particles, axis=0, weights=weights)
        x_est[2] = arctan2(
            average(sin(particles[:,2]), weights=weights),
            average(cos(particles[:,2]), weights=weights)
        )

        diff: ndarray = particles - x_est
        diff[:,2] = arctan2(sin(diff[:,2]), cos(diff[:,2]))
        cov_est: ndarray = (diff * weights[:,None]).T @ diff
        return x_est, cov_est


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
