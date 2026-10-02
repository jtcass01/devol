#!/usr/bin/env python3
"""Bootstrap (SIR) Monte Carlo localization, written from scratch.

Pure numpy, no ROS, so the filter math can be tested offline.

- Motion model: odometry motion model (Thrun, Probabilistic Robotics, table 5.6).
- Measurement model: likelihood field for a 2D range finder (table 6.3).
- Resampling: low-variance (systematic) resampling (table 4.4), run when the
  effective sample size drops below a fraction of the particle count.

Poses are (x, y, yaw) in the map frame. Particle state is an (N, 3) array.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt


def wrap_angle(a):
    """Wrap angle(s) to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def odometry_delta(prev_pose, curr_pose) -> Tuple[float, float, float]:
    """Decompose the motion between two odometry poses into (rot1, trans, rot2)."""
    dx = curr_pose[0] - prev_pose[0]
    dy = curr_pose[1] - prev_pose[1]
    trans = float(np.hypot(dx, dy))
    # A pure rotation has no meaningful heading of travel; put it all in rot2.
    rot1 = float(wrap_angle(np.arctan2(dy, dx) - prev_pose[2])) if trans > 1e-6 else 0.0
    rot2 = float(wrap_angle(curr_pose[2] - prev_pose[2] - rot1))
    # Driving backwards: express as a reverse translation so rot1 stays small.
    if abs(rot1) > np.pi / 2.0:
        rot1 = float(wrap_angle(rot1 + np.pi))
        rot2 = float(wrap_angle(rot2 + np.pi))
        trans = -trans
    return rot1, trans, rot2


class LikelihoodField:
    """Distance-to-nearest-obstacle lookup built from an occupancy grid.

    grid: (height, width) int array, row 0 at the map origin (ROS OccupancyGrid
    layout). Values >= occupied_threshold are obstacles; -1 is unknown.
    """

    def __init__(self, grid: np.ndarray, resolution: float, origin_x: float, origin_y: float,
                 occupied_threshold: int = 50, max_dist: float = 2.0):
        self.grid = np.asarray(grid)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.height, self.width = self.grid.shape
        self.max_dist = float(max_dist)

        self.occupied = self.grid >= occupied_threshold
        if self.occupied.any():
            # EDT measures distance to the nearest zero, so feed it "not occupied".
            dist = distance_transform_edt(~self.occupied) * self.resolution
        else:
            dist = np.full(self.grid.shape, self.max_dist)
        self.dist = np.minimum(dist, self.max_dist)

        # Cells a robot could be in: anything known not to be an obstacle.
        # Octomap projections leave most free space as unknown (-1), so
        # unknown counts as free here.
        self.free_cells = np.argwhere(~self.occupied)

    def world_to_cell(self, x, y):
        col = np.floor((np.asarray(x) - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor((np.asarray(y) - self.origin_y) / self.resolution).astype(np.int64)
        return row, col

    def distance(self, x, y):
        """Distance (m) from each point to the nearest obstacle, max_dist outside the map."""
        row, col = self.world_to_cell(x, y)
        inside = (row >= 0) & (row < self.height) & (col >= 0) & (col < self.width)
        out = np.full(np.shape(row), self.max_dist)
        out[inside] = self.dist[row[inside], col[inside]]
        return out

    def is_free(self, x, y):
        row, col = self.world_to_cell(x, y)
        inside = (row >= 0) & (row < self.height) & (col >= 0) & (col < self.width)
        out = np.zeros(np.shape(row), dtype=bool)
        out[inside] = ~self.occupied[row[inside], col[inside]]
        return out


@dataclass
class PFParams:
    num_particles: int = 500
    # Odometry motion noise (Thrun's alpha1..alpha4), std-dev form:
    # rot std = a1*|rot| + a2*|trans|, trans std = a3*|trans| + a4*(|rot1|+|rot2|).
    # Sized for skid-steer odometry, whose yaw can be off by ~30% while
    # turning: too little rotation noise lets the particle cloud fall behind
    # the true heading and collapse (particle depletion).
    alpha1: float = 0.3
    alpha2: float = 0.1
    alpha3: float = 0.1
    alpha4: float = 0.05
    # Likelihood field model. Fewer beams and a wider sigma keep the
    # likelihood from being overconfident when the scan and map disagree.
    sigma_hit: float = 0.3
    z_hit: float = 0.9
    z_rand: float = 0.1
    max_beams: int = 30
    # Resample when N_eff < resample_threshold * N.
    resample_threshold: float = 0.5
    # Augmented MCL random-particle injection (table 8.3), used to recover
    # from divergence and kidnapping. Set both to 0 for plain SIR.
    alpha_slow: float = 0.001
    alpha_fast: float = 0.1


class ParticleFilter:
    def __init__(self, params: PFParams, field: Optional[LikelihoodField] = None,
                 seed: Optional[int] = None):
        self.params = params
        self.field = field
        self.rng = np.random.default_rng(seed)
        n = int(params.num_particles)
        self.particles = np.zeros((n, 3))
        self.weights = np.full(n, 1.0 / n)
        self.initialized = False
        self._w_slow = 0.0
        self._w_fast = 0.0

    # ------------------------------------------------------------------ init
    def init_gaussian(self, pose, std_xy: float, std_yaw: float) -> None:
        n = self.particles.shape[0]
        self.particles[:, 0] = pose[0] + self.rng.normal(0.0, std_xy, n)
        self.particles[:, 1] = pose[1] + self.rng.normal(0.0, std_xy, n)
        self.particles[:, 2] = wrap_angle(pose[2] + self.rng.normal(0.0, std_yaw, n))
        self.weights.fill(1.0 / n)
        self.initialized = True

    def init_uniform(self) -> None:
        """Global localization: uniform over non-obstacle cells of the map."""
        n = self.particles.shape[0]
        self.particles = self._sample_uniform(n)
        self.weights.fill(1.0 / n)
        self.initialized = True

    def _sample_uniform(self, n: int) -> np.ndarray:
        if self.field is None or len(self.field.free_cells) == 0:
            raise RuntimeError('uniform sampling needs a map with free cells')
        f = self.field
        idx = self.rng.integers(0, len(f.free_cells), n)
        rows, cols = f.free_cells[idx, 0], f.free_cells[idx, 1]
        out = np.empty((n, 3))
        out[:, 0] = f.origin_x + (cols + self.rng.random(n)) * f.resolution
        out[:, 1] = f.origin_y + (rows + self.rng.random(n)) * f.resolution
        out[:, 2] = self.rng.uniform(-np.pi, np.pi, n)
        return out

    # ---------------------------------------------------------------- motion
    def predict(self, rot1: float, trans: float, rot2: float) -> None:
        """Sample from the odometry motion model for every particle."""
        p = self.params
        n = self.particles.shape[0]
        at = abs(trans)
        std_rot1 = p.alpha1 * abs(rot1) + p.alpha2 * at
        std_trans = p.alpha3 * at + p.alpha4 * (abs(rot1) + abs(rot2))
        std_rot2 = p.alpha1 * abs(rot2) + p.alpha2 * at

        r1 = rot1 - self.rng.normal(0.0, 1.0, n) * std_rot1
        t = trans - self.rng.normal(0.0, 1.0, n) * std_trans
        r2 = rot2 - self.rng.normal(0.0, 1.0, n) * std_rot2

        heading = self.particles[:, 2] + r1
        self.particles[:, 0] += t * np.cos(heading)
        self.particles[:, 1] += t * np.sin(heading)
        self.particles[:, 2] = wrap_angle(heading + r2)

    # ----------------------------------------------------------- measurement
    def update(self, ranges: np.ndarray, angles: np.ndarray, range_min: float, range_max: float,
               laser_pose=(0.0, 0.0, 0.0)) -> float:
        """Weight particles with the likelihood field model.

        ranges/angles: beam ranges and angles in the laser frame.
        laser_pose: (x, y, yaw) of the laser in the robot base frame.
        Returns the effective sample size after the update.
        """
        if self.field is None:
            raise RuntimeError('measurement update needs a map')
        p = self.params
        ranges = np.asarray(ranges, dtype=float)
        angles = np.asarray(angles, dtype=float)

        # Max-range and invalid returns carry no endpoint information.
        valid = np.isfinite(ranges) & (ranges > range_min) & (ranges < range_max)
        idx = np.flatnonzero(valid)
        if idx.size == 0:
            return self.effective_sample_size()
        if idx.size > p.max_beams:
            idx = idx[np.linspace(0, idx.size - 1, p.max_beams).round().astype(int)]
        r = ranges[idx]
        a = angles[idx]

        x = self.particles[:, 0:1]
        y = self.particles[:, 1:2]
        th = self.particles[:, 2:3]
        lx, ly, lth = laser_pose
        c, s = np.cos(th), np.sin(th)
        sensor_x = x + c * lx - s * ly
        sensor_y = y + s * lx + c * ly
        beam = th + lth + a[None, :]
        ex = sensor_x + r[None, :] * np.cos(beam)
        ey = sensor_y + r[None, :] * np.sin(beam)

        d = self.field.distance(ex, ey)
        gauss = np.exp(-0.5 * (d / p.sigma_hit) ** 2) / (np.sqrt(2.0 * np.pi) * p.sigma_hit)
        prob = p.z_hit * gauss + p.z_rand / range_max
        log_lik = np.log(prob).sum(axis=1)

        # Average per-beam likelihood (geometric mean over beams keeps it
        # scale-free) drives the augmented-MCL injection rate.
        w_avg = float(np.mean(np.exp(log_lik / r.size)))
        if p.alpha_slow > 0.0 and p.alpha_fast > 0.0:
            if self._w_slow == 0.0:
                self._w_slow = self._w_fast = w_avg
            self._w_slow += p.alpha_slow * (w_avg - self._w_slow)
            self._w_fast += p.alpha_fast * (w_avg - self._w_fast)

        log_w = np.log(np.maximum(self.weights, 1e-300)) + log_lik
        log_w -= log_w.max()
        w = np.exp(log_w)
        self.weights = w / w.sum()
        return self.effective_sample_size()

    # ------------------------------------------------------------ resampling
    def effective_sample_size(self) -> float:
        return float(1.0 / np.sum(self.weights ** 2))

    def injection_probability(self) -> float:
        if self._w_slow <= 0.0 or self.field is None:
            return 0.0
        return max(0.0, 1.0 - self._w_fast / self._w_slow)

    def resample_if_needed(self) -> bool:
        n = self.particles.shape[0]
        p_inject = self.injection_probability()
        if p_inject > 0.01 or self.effective_sample_size() < self.params.resample_threshold * n:
            self.resample(p_inject)
            return True
        return False

    def resample(self, p_inject: float = 0.0) -> None:
        """Low-variance (systematic) resampling, optionally replacing a
        fraction p_inject of particles with uniform samples over the map."""
        n = self.particles.shape[0]
        positions = (self.rng.random() + np.arange(n)) / n
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0
        idx = np.searchsorted(cumulative, positions)
        self.particles = self.particles[idx]
        self.weights = np.full(n, 1.0 / n)

        n_inject = int(self.rng.binomial(n, min(p_inject, 1.0))) if p_inject > 0.0 else 0
        if n_inject > 0:
            slots = self.rng.choice(n, n_inject, replace=False)
            self.particles[slots] = self._sample_uniform(n_inject)

    # -------------------------------------------------------------- estimate
    def estimate(self) -> Tuple[np.ndarray, np.ndarray]:
        """Weighted mean pose (circular mean for yaw) and 3x3 covariance."""
        w = self.weights
        mx = float(np.dot(w, self.particles[:, 0]))
        my = float(np.dot(w, self.particles[:, 1]))
        myaw = float(np.arctan2(np.dot(w, np.sin(self.particles[:, 2])),
                                np.dot(w, np.cos(self.particles[:, 2]))))
        dev = np.column_stack((self.particles[:, 0] - mx,
                               self.particles[:, 1] - my,
                               wrap_angle(self.particles[:, 2] - myaw)))
        cov = (dev * w[:, None]).T @ dev
        return np.array([mx, my, myaw]), cov
