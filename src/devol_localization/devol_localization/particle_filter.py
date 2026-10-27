"""
This code was modified and adapted from Introduction to Robotics Coursework.
"""

# Import Libraries
from typing import Tuple, Any
from numpy import (
    array,
    ndarray,
    searchsorted,
    cumsum,
    zeros,
    average,
    arctan2,
    cos,
    sin,
    copy as np_copy,
    arange,
)
from numpy.random import uniform, multivariate_normal


def low_variance_resample(particles: ndarray, weights: ndarray) -> ndarray:
    N = len(particles)

    # Generate a single random start point between 0 and 1/N
    step = 1.0 / N
    pointers = uniform(0, step) + arange(N) * step

    # Find where those evenly spaced pointers land in the cumulative weight array
    cumulative_sum = cumsum(weights)
    indexes = searchsorted(cumulative_sum, pointers)

    return particles[indexes]


def particle_filter_2d(
    particles: ndarray,
    map_dict: dict,
    motion_model: Any,
    sensor_model: Any,
    resample: callable,
    u: ndarray,
    dt: float,
    z_dists: list,
    z_thetas: list,
    laser_offset: ndarray,
    V: ndarray,
) -> Tuple[ndarray, ndarray]:
    ## Predict
    # Apply the motion model to the samples (Prediction Step)
    new_particles = array(
        [
            motion_model.forward(particle, u, multivariate_normal(zeros(3), V), dt)
            for particle in particles
        ]
    )

    # Calculate where the laser actually is for each newly predicted particle
    laser_poses = np_copy(new_particles)
    # Shift X and Y based on the laser's physical mount and the robot's current yaw
    laser_poses[:, 0] += laser_offset[0] * cos(laser_poses[:, 2]) - laser_offset[1] * sin(
        laser_poses[:, 2]
    )
    laser_poses[:, 1] += laser_offset[0] * sin(laser_poses[:, 2]) + laser_offset[1] * cos(
        laser_poses[:, 2]
    )
    # Shift Yaw
    laser_poses[:, 2] += laser_offset[2]

    ## Update
    # Computes gaussian probability
    weights = array(
        [
            sensor_model.likelihood_field_range_finder(
                z_dists=z_dists, z_thetas=z_thetas, x=particle, map_dict=map_dict, std=0.1
            )
            for particle in laser_poses
        ],
        dtype=float,
    )

    # Prevent division by zero
    weights += 1e-40
    # normalize (\eta)
    weights /= sum(weights)

    # Estimate the state based on the weighted average of particles
    x_est: ndarray = average(new_particles, axis=0, weights=weights)

    # Wrap angle
    x_est[2] = arctan2(
        average(sin(new_particles[:, 2]), weights=weights),
        average(cos(new_particles[:, 2]), weights=weights),
    )

    # Resample
    new_particles = resample(new_particles, weights)

    return new_particles, x_est
