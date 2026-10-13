"""
This code was modified and adapted from Introduction to Robotics Coursework.
"""

# Import Libraries
from typing import Tuple
from numpy import (
    array,
    ndarray,
    searchsorted,
    cumsum,
    zeros,
    diag,
    deg2rad,
    average,
    arctan2,
    cos,
    sin,
    copy as np_copy,
)
from numpy.random import uniform, multivariate_normal

from devol_localization.differential_drive import DifferentialDrive
from devol_localization.laser_model import ROSLaserModel


def simple_resample(particles, weights):
    N = len(particles)
    cumulative_sum = cumsum(weights / sum(weights))
    cumulative_sum[-1] = 1.0  # avoid round-off error
    indexes = searchsorted(cumulative_sum, uniform(0, 1, size=N))
    return particles[indexes]


def particle_filter_2d(
    particles: ndarray,
    map_dict: dict,
    motion_model: DifferentialDrive,
    sensor_model: ROSLaserModel,
    resample: callable,
    u: ndarray,
    dt: float,
    z_dists: list,
    z_thetas: list,
    laser_offset: ndarray,
) -> Tuple[ndarray, ndarray]:
    V: ndarray = diag([0.1**2, 0.1**2, deg2rad(5.0) ** 2])

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
    # Compute the likelihood and the importances using REAL sensor data (Update Step)
    weights = array(
        [
            sensor_model.likelihood_field_range_finder(
                z_dists=z_dists,
                z_thetas=z_thetas,
                x=particle,
                map_dict=map_dict,
                z_max=20.0,
                std=0.1,
            )
            for particle in laser_poses
        ],
        dtype=float,
    )

    weights += 1e-40
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
