from typing import Optional, Tuple
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
    zeros,
    vstack,
    where,
    pi,
    mean as np_mean,
)
from numpy.random import multivariate_normal, uniform, normal, choice

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


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


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
        self.weights: ndarray = ones(self.num_particles) / self.num_particles

        # Adaptive injection moving averages
        self.slow_avg_lik = 0.0
        self.fast_avg_lik = 0.0

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

        # Open a CSV file for logging tuning metrics
        self.tuning_log_file = open('pf_tuning_metrics.csv', 'w')
        self.tuning_log_file.write('time,n_eff,avg_raw_likelihood\n')
        self.get_logger().info('Particle Filter Node Started!')

    def odom_callback(self, msg: Odometry):
        """Base callback to track the latest odometry twist."""
        self.v: float = msg.twist.twist.linear.x
        self.omega: float = msg.twist.twist.angular.z
        self.u: ndarray = array([self.v, self.omega])

        dt: Optional[float] = self.get_dt_from_header(msg.header)
        if dt is None:
            return

        self.particles = self._predict(dt=dt)

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

    def scan_callback(self, msg: LaserScan):
        """The main loop of the PF: computes dt, downsamples scan, runs PF, and publishes results."""
        # Wait for map and ensure odometry has ran at least once.
        if self.map_dict is None:
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
        noisy_dists = normal(dist_moved, sigma_v * sqrt_dt, self.num_particles)
        noisy_headings = normal(heading_moved, sigma_w * sqrt_dt, self.num_particles)

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
        new_particles[:, 0] += normal(0.0, 0.005, self.num_particles)
        new_particles[:, 1] += normal(0.0, 0.005, self.num_particles)

        return new_particles

    def _update(self, z_dists: list, z_thetas: list):
        ## Update
        # Calculate where the laser actually is for each newly predicted particle
        laser_poses: ndarray = np_copy(self.particles)
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

        # Computes log probabilities
        log_weights = array(
            [
                LaserModel.likelihood_field_range_finder(
                    z_dists=z_dists, z_thetas=z_thetas, x=particle, map_dict=self.map_dict, std=0.1
                )
                for particle in laser_poses
            ],
            dtype=float,
        )

        # shift all weights so their highest value is 0.
        max_log_weights: ndarray = np_max(log_weights)
        step_weights: ndarray = np_exp(log_weights - max_log_weights)

        # Calculate raw likelihood average for the adaptive injection logic
        avg_raw_likelihood = np_mean(np_exp(log_weights / len(z_dists)))

        # Initialize running averages on the very first update
        if self.slow_avg_lik == 0.0:
            self.slow_avg_lik = avg_raw_likelihood
            self.fast_avg_lik = avg_raw_likelihood

        # Update exponential moving averages (standard AMCL alpha values)
        self.slow_avg_lik += 0.001 * (avg_raw_likelihood - self.slow_avg_lik)
        self.fast_avg_lik += 0.1 * (avg_raw_likelihood - self.fast_avg_lik)

        # Multiply the new step weights into the running weight accumulator
        self.weights *= step_weights

        # Prevent division by zero
        self.weights += 1e-40
        # normalize (\eta)
        self.weights /= sum(self.weights)

        # Calculate Effective Sample Size (N_eff)
        n_eff = 1.0 / sum(self.weights**2)

        # Log to file
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.tuning_log_file.write(f'{current_time:.3f},{n_eff:.1f},{avg_raw_likelihood:.6f}\n')
        self.tuning_log_file.flush()

        # Estimate the state based on the weighted average of particles
        x_est: ndarray = average(self.particles, axis=0, weights=self.weights)

        # Wrap angle
        x_est[2] = arctan2(
            average(sin(self.particles[:, 2]), weights=self.weights),
            average(cos(self.particles[:, 2]), weights=self.weights),
        )

        # Conditional Resampling: Only resample if N_eff drops below 50%
        if n_eff < (self.num_particles / 2.0):
            # Calculate adaptive escape fraction
            escape_fraction = 0.0
            if self.slow_avg_lik > 0 and self.fast_avg_lik < self.slow_avg_lik:
                # The fast average dropped below the slow average; spike injection rate.
                escape_fraction = 1.0 - (self.fast_avg_lik / self.slow_avg_lik)

            # Cap the maximum injection
            escape_fraction = max(0.0, min(0.05, escape_fraction))

            num_random = int(self.num_particles * escape_fraction)
            num_normal = self.num_particles - num_random
            normal_particles: ndarray = ParticleFilterNode.low_variance_resample(
                self.particles, self.weights, num_samples=num_normal
            )

            # Draw fresh random particles uniformly from map freespace
            free_x = self.map_dict.get('free_x', array([]))
            free_y = self.map_dict.get('free_y', array([]))

            if num_random > 0:
                if len(free_x) > 0:
                    rand_indices = choice(len(free_x), size=num_random, replace=True)
                    random_particles = zeros((num_random, 3))
                    random_particles[:, 0] = free_x[rand_indices]
                    random_particles[:, 1] = free_y[rand_indices]
                    random_particles[:, 2] = uniform(-pi, pi, num_random)
                else:
                    # Safe fallback if map processing missed freespace coordinates
                    random_particles = multivariate_normal(
                        x_est, cov(self.particles, rowvar=False), num_random
                    )

                # Combine both groups to form the new generation
                new_particles: ndarray = vstack((normal_particles, random_particles))
            else:
                new_particles = normal_particles

            # Reset weights to uniform after a resample event
            self.weights = ones(self.num_particles) / self.num_particles
        else:
            new_particles = self.particles

        return new_particles, x_est

    @staticmethod
    def low_variance_resample(
        particles: ndarray, weights: ndarray, num_samples: Optional[int] = None
    ) -> ndarray:
        N = len(particles) if num_samples is None else num_samples

        # Generate a single random start point between 0 and 1/N
        step = 1.0 / N
        pointers = uniform(0, step) + arange(N) * step

        # Find where those evenly spaced pointers land in the cumulative weight array
        cumulative_sum = cumsum(weights)
        indexes = searchsorted(cumulative_sum, pointers)

        return particles[indexes]

    def destroy_node(self):
        """Ensure the log file is closed cleanly on shutdown."""
        if hasattr(self, 'tuning_log_file') and not self.tuning_log_file.closed:
            self.tuning_log_file.close()
        super().destroy_node()


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
