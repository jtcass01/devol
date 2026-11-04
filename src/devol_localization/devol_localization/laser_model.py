#!/usr/bin/env python3

from numpy import (
    ndarray,
    cos,
    sin,
    floor,
    clip,
    abs as np_abs,
    sum as np_sum,
    log,
    logaddexp,
    sqrt,
    pi,
)

__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


class LaserModel:
    @staticmethod
    def endpoint_distances(
        laser_poses: ndarray, z_dists: ndarray, z_thetas: ndarray, map_dict: dict
    ) -> ndarray:
        """Distance from every beam endpoint to the nearest obstacle. Shape (M, K)."""
        edt: ndarray = map_dict['grid']
        info = map_dict['info']
        res: float = info.resolution
        height, width = edt.shape

        # Beam endpoints in the map frame (M x K)
        ang: ndarray = laser_poses[:, 2:3] + z_thetas[None, :]
        ex: ndarray = laser_poses[:, 0:1] + z_dists[None, :] * cos(ang)
        ey: ndarray = laser_poses[:, 1:2] + z_dists[None, :] * sin(ang)

        # Cell indices
        cols: ndarray = floor((ex - info.origin.position.x) / res).astype(int)
        rows: ndarray = floor((ey - info.origin.position.y) / res).astype(int)

        # Out-of-bounds: clamp to the edge and add the L1 distance to the map boundary
        c: ndarray = clip(cols, 0, width - 1)
        r: ndarray = clip(rows, 0, height - 1)
        oob: ndarray = (np_abs(cols - c) + np_abs(rows - r)) * res

        return edt[r, c] + oob

    @staticmethod
    def log_likelihood_gaussian(d: ndarray, sigma_hit: float) -> ndarray:
        """Returns shape (M,)."""
        return -0.5 * np_sum(d**2, axis=1) / sigma_hit**2

    @staticmethod
    def log_likelihood_mixture(
        d: ndarray, sigma_hit: float, z_hit: float, z_rand: float, z_max: float
    ) -> ndarray:
        """Thrun Table 6.3 likelihood field: z_hit * N(d; 0, sigma) + z_rand / z_max."""
        log_hit: ndarray = (
            log(z_hit) - log(sqrt(2.0 * pi) * sigma_hit) - 0.5 * (d / sigma_hit) ** 2
        )
        log_rand: float = log(z_rand / z_max)
        return np_sum(logaddexp(log_hit, log_rand), axis=1)
