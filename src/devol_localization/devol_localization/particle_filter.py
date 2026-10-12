# Import Libraries
from numpy import (array, cos, sin, linspace, arange, 
                   pi, ndarray, exp, sqrt, eye, inf, 
                   ones, searchsorted, cumsum, zeros, 
                   argmax, diag, deg2rad, average)
from numpy.random import uniform, multivariate_normal
from typing import List, Tuple
from math import ceil
from typing import Iterable
from ipywidgets import interact, IntSlider

import matplotlib.pyplot as plt

from devol_localization.differential_drive import DifferentialDrive
from devol_localization.laser_model import LaserModel

def simple_resample(particles, weights):
    N = len(particles)
    cumulative_sum = cumsum(weights / sum(weights))
    cumulative_sum[-1] = 1. # avoid round-off error
    indexes = searchsorted(cumulative_sum, uniform(0,1,size=N))
    return particles[indexes]

def particle_filter_2d(x: ndarray, particles: ndarray, map: ndarray, motion_model: DifferentialDrive, 
                       sensor_model: LaserModel, resample: callable, u: ndarray, dt: float) -> None:
    V: ndarray = diag([0.1**2, 0.1**2, deg2rad(5.0)**2])
    # Apply motion to the true robot
    x = motion_model.forward(x, u, multivariate_normal(zeros(3), V), dt)

    # Take a measurement
    z_dists, z_thetas = sensor_model.scan_map(x, map=map, max_range=20.0, theta_res=0.2)

    # Apply the motion model to the samples
    new_particles = array([
        motion_model.forward(particle, u, multivariate_normal(zeros(3), V), dt)
        for particle in particles
    ])

    # Compute the likelihood and the importances
    weights = array([
        LaserModel.likelihood_field_range_finder(
            z_dists=z_dists,
            z_thetas=z_thetas,
            x=particle,
            map=map,
            z_max=20.0,
            std=2.0) for particle in new_particles
    ], dtype=float)
    weights += 1e-40
    weights /= sum(weights)

    x_est: ndarray = average(new_particles, axis=0, weights=weights)

    # Resample
    new_particles = resample(new_particles, weights)

    return new_particles, x, x_est