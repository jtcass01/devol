#!/usr/bin/env python3
"""Bootstrap (SIR) Monte Carlo localization, written from scratch.

Pure numpy, no ROS, so the filter math can be tested offline. References are listed in
THEORY.md at the package root; "PR" below is Thrun, Burgard and Fox, Probabilistic Robotics
(2005) [thrun2005probabilistic].

The belief bel(x_t) is represented by N weighted samples (particles) and updated with the
sampling-importance-resampling cycle of the bootstrap filter (Gordon, Salmond and Smith 1993
[gordon1993novel]); applied to localization against a known map this is Monte Carlo
localization (Dellaert et al. 1999 [dellaert1999monte]; Fox et al. 1999 [fox1999monte];
PR Table 8.2, built on the generic particle filter of PR Table 4.3). One filter update, as the
node drives it (pf_localization_node.py), is:

1. Prediction: draw x_t^[m] ~ p(x_t | u_t, x_{t-1}^[m]) for every particle with the odometry
   motion model (PR Section 5.4, sample_motion_model_odometry, Table 5.6). Using the motion
   model as the proposal is what makes this the bootstrap filter.
2. Correction: multiply each weight by the measurement likelihood p(z_t | x_t^[m], m) of the
   likelihood field range finder model (PR Section 6.4, Table 6.3). With the motion model as
   the proposal, the importance weight is exactly this likelihood (PR Table 4.3).
3. Resampling: low-variance (systematic) resampling (PR Table 4.4), run only when the
   effective sample size N_eff = 1 / sum(w^2) drops below a fraction of N (selective
   resampling, PR Section 4.3.4; N_eff as in Arulampalam et al. 2002 [arulampalam2002tutorial]).
   Between resamplings the weights carry over, so step 2 is the sequential importance
   sampling update w_t = w_{t-1} p(z_t | x_t).
4. Recovery: augmented MCL (PR Section 8.3.5, Table 8.3; Thrun et al. 2001 [thrun2001robust])
   replaces a fraction of the particles with uniform samples over the free space when the
   short-term average measurement likelihood falls below the long-term one, which is how the
   filter recovers from a kidnapping.

The pose estimate is the weighted sample mean, with a circular mean for yaw.

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
    """Decompose the motion between two odometry poses into (rot1, trans, rot2).

    PR Table 5.6 lines 2-4: rot1 = atan2(dy, dx) - yaw, trans = sqrt(dx^2 + dy^2),
    rot2 = yaw' - yaw - rot1.
    """
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

    This is the precomputed lookup table behind the likelihood field model (PR Section 6.4):
    dist(x, y) is the distance from a scan endpoint to the nearest occupied cell, the quantity
    the model scores with a zero-mean Gaussian. Computing it once with a Euclidean distance
    transform makes each beam's likelihood a single table lookup.

    grid: (height, width) int array, row 0 at the map origin (ROS OccupancyGrid
    layout). Values >= occupied_threshold are obstacles; -1 is unknown.
    """

    def __init__(
        self,
        grid: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        occupied_threshold: int = 50,
        max_dist: float = 2.0,
    ):
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
    # PR Table 5.6 instead passes the variance a1*rot^2 + a2*trans^2 (and so on) to sample().
    # Both make the std proportional to the size of the increment; the std form adds the two
    # terms instead of their squares, so these alphas are not numerically the book's.
    # Sized for skid-steer odometry, whose yaw can be off by ~30% while
    # turning and which drifts in translation during in-place spins. Too
    # little noise lets the particle cloud fall behind the true pose and
    # collapse to a few cm (particle depletion); alpha3/alpha4 below 0.22
    # did that on the recorded study_nominal Gazebo drive.
    alpha1: float = 0.3
    alpha2: float = 0.22
    alpha3: float = 0.22
    alpha4: float = 0.22
    # Increments translating less than this (m) are an in-place turn: the
    # noise is spread as a pure rotation (rot1 = 0, rot2 = rot1 + rot2), as
    # AMCL does, because the direction of a sub-cm creep is arbitrary.
    min_translation: float = 0.01
    # Likelihood field model (PR Table 6.3): per-beam likelihood
    # z_hit * N(dist; 0, sigma_hit^2) + z_rand / z_max. Fewer beams and a wider sigma keep the
    # likelihood from being overconfident when the scan and map disagree; the beams are
    # treated as independent, which PR Chapter 6 notes overstates the information in a
    # dense scan, so subsampling them also tempers that.
    sigma_hit: float = 0.3
    z_hit: float = 0.9
    z_rand: float = 0.1
    max_beams: int = 30
    # Resample when N_eff < resample_threshold * N (selective resampling, PR Section 4.3.4).
    resample_threshold: float = 0.5
    # Augmented MCL random-particle injection (table 8.3), used to recover
    # from divergence and kidnapping. Set both to 0 for plain SIR.
    # The book requires 0 <= alpha_slow << alpha_fast: w_slow then tracks the long-term and
    # w_fast the short-term average likelihood.
    alpha_slow: float = 0.001
    alpha_fast: float = 0.1


class ParticleFilter:
    """Monte Carlo localization (PR Table 8.2) with augmented-MCL recovery (PR Table 8.3).

    self.particles is the sample set X_t and self.weights the normalized importance weights.
    """

    def __init__(
        self, params: PFParams, field: Optional[LikelihoodField] = None, seed: Optional[int] = None
    ):
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
        """Position tracking: draw the initial particles from a Gaussian prior around a known
        pose (PR Section 7.1 calls this the local, or position tracking, problem)."""
        n = self.particles.shape[0]
        self.particles[:, 0] = pose[0] + self.rng.normal(0.0, std_xy, n)
        self.particles[:, 1] = pose[1] + self.rng.normal(0.0, std_xy, n)
        self.particles[:, 2] = wrap_angle(pose[2] + self.rng.normal(0.0, std_yaw, n))
        self.weights.fill(1.0 / n)
        self.initialized = True

    def init_uniform(self) -> None:
        """Global localization: uniform over non-obstacle cells of the map.

        The particle representation of the uniform prior bel(x_0) that global localization
        starts from (PR Section 8.3.2).
        """
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
        """Sample from the odometry motion model for every particle (prediction step,
        PR Tables 4.3 and 8.2, with sample_motion_model_odometry of Table 5.6).

        Each particle gets its own noisy copy of (rot1, trans, rot2) (Table 5.6 lines 5-7) and is
        moved by it (lines 8-10). The spread this adds is what represents the growth of motion
        uncertainty; there is no covariance to propagate.
        """
        p = self.params
        n = self.particles.shape[0]
        at = abs(trans)
        # Noise magnitudes only; the mean motion stays (rot1, trans, rot2).
        e1, e2 = (0.0, rot1 + rot2) if at < p.min_translation else (rot1, rot2)
        std_rot1 = p.alpha1 * abs(e1) + p.alpha2 * at
        std_trans = p.alpha3 * at + p.alpha4 * (abs(e1) + abs(e2))
        std_rot2 = p.alpha1 * abs(e2) + p.alpha2 * at

        # Table 5.6 lines 5-7: one noise sample per particle and per motion component.
        r1 = rot1 - self.rng.normal(0.0, 1.0, n) * std_rot1
        t = trans - self.rng.normal(0.0, 1.0, n) * std_trans
        r2 = rot2 - self.rng.normal(0.0, 1.0, n) * std_rot2

        # Table 5.6 lines 8-10: apply the sampled motion.
        heading = self.particles[:, 2] + r1
        self.particles[:, 0] += t * np.cos(heading)
        self.particles[:, 1] += t * np.sin(heading)
        self.particles[:, 2] = wrap_angle(heading + r2)

    # ----------------------------------------------------------- measurement
    def update(
        self,
        ranges: np.ndarray,
        angles: np.ndarray,
        range_min: float,
        range_max: float,
        laser_pose=(0.0, 0.0, 0.0),
    ) -> float:
        """Weight particles with the likelihood field model (correction step, PR Tables 4.3
        and 8.2, with likelihood_field_range_finder_model of Table 6.3).

        For each particle and each beam k with range z_k at angle a_k, the beam endpoint in the
        map is projected through the particle pose and the laser mount pose, its distance to
        the nearest obstacle is looked up, and the beam likelihoods are multiplied. Max-range
        readings are skipped, as in Table 6.3.

        ranges/angles: beam ranges and angles in the laser frame.
        laser_pose: (x, y, yaw) of the laser in the robot base frame.
        Returns the effective sample size after the update.
        """
        if self.field is None:
            raise RuntimeError('measurement update needs a map')
        p = self.params
        ranges = np.asarray(ranges, dtype=float)
        angles = np.asarray(angles, dtype=float)

        # Max-range and invalid returns carry no endpoint information (skipped in Table 6.3).
        valid = np.isfinite(ranges) & (ranges > range_min) & (ranges < range_max)
        idx = np.flatnonzero(valid)
        if idx.size == 0:
            return self.effective_sample_size()
        if idx.size > p.max_beams:
            idx = idx[np.linspace(0, idx.size - 1, p.max_beams).round().astype(int)]
        r = ranges[idx]
        a = angles[idx]

        # Beam endpoints in the map frame, one row per particle (as in Table 6.3).
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

        # q = prod_k (z_hit * prob(dist_k, sigma_hit) + z_rand / z_max) (Table 6.3),
        # summed in log space so a 30-beam product does not underflow.
        d = self.field.distance(ex, ey)
        gauss = np.exp(-0.5 * (d / p.sigma_hit) ** 2) / (np.sqrt(2.0 * np.pi) * p.sigma_hit)
        prob = p.z_hit * gauss + p.z_rand / range_max
        log_lik = np.log(prob).sum(axis=1)

        # Average per-beam likelihood (geometric mean over beams keeps it
        # scale-free) drives the augmented-MCL injection rate. In Table 8.3, w_avg is the
        # average particle likelihood, and w_slow / w_fast are its exponential moving averages.
        # The geometric mean per beam is used here instead of the raw product, which spans many
        # orders of magnitude between scans with different numbers of valid beams.
        w_avg = float(np.mean(np.exp(log_lik / r.size)))
        if p.alpha_slow > 0.0 and p.alpha_fast > 0.0:
            if self._w_slow == 0.0:
                self._w_slow = self._w_fast = w_avg
            self._w_slow += p.alpha_slow * (w_avg - self._w_slow)
            self._w_fast += p.alpha_fast * (w_avg - self._w_fast)

        # Sequential importance weight update w_t = w_{t-1} * p(z_t | x_t), then normalization
        # (the eta of the Bayes filter). Subtracting the max before exp avoids underflow.
        log_w = np.log(np.maximum(self.weights, 1e-300)) + log_lik
        log_w -= log_w.max()
        w = np.exp(log_w)
        self.weights = w / w.sum()
        return self.effective_sample_size()

    # ------------------------------------------------------------ resampling
    def effective_sample_size(self) -> float:
        """N_eff = 1 / sum_m (w^[m])^2 for normalized weights: N when all weights are equal,
        1 when one particle carries all the weight (Arulampalam et al. 2002; PR Section 4.3.4
        discusses the variance that resampling too often adds)."""
        return float(1.0 / np.sum(self.weights**2))

    def injection_probability(self) -> float:
        """max(0, 1 - w_fast / w_slow), the probability of replacing a particle with a random
        one in augmented MCL (PR Table 8.3)."""
        if self._w_slow <= 0.0 or self.field is None:
            return 0.0
        return max(0.0, 1.0 - self._w_fast / self._w_slow)

    def resample_if_needed(self) -> bool:
        """Selective resampling: resample when N_eff is low, and also whenever augmented MCL
        wants to inject random particles (injection happens during resampling, as in Table 8.3)."""
        n = self.particles.shape[0]
        p_inject = self.injection_probability()
        if p_inject > 0.01 or self.effective_sample_size() < self.params.resample_threshold * n:
            self.resample(p_inject)
            return True
        return False

    def resample(self, p_inject: float = 0.0) -> None:
        """Low-variance (systematic) resampling, optionally replacing a
        fraction p_inject of particles with uniform samples over the map.

        PR Table 4.4: a single random offset r ~ U(0, 1/N) places N equally spaced pointers
        r + (m - 1)/N on the cumulative weight; each pointer picks the particle whose weight
        interval contains it. Compared with N independent draws this costs O(N) and adds less
        sampling variance, and a set with equal weights is reproduced unchanged.
        Injection follows PR Table 8.3: each new particle is a uniform random pose
        with probability p_inject, otherwise a resampled one.
        """
        n = self.particles.shape[0]
        # (u + m) / N with u ~ U(0, 1) is Table 4.4's r + (m - 1) / M with r ~ U(0, 1/M).
        positions = (self.rng.random() + np.arange(n)) / n
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0
        idx = np.searchsorted(cumulative, positions)
        self.particles = self.particles[idx]
        self.weights = np.full(n, 1.0 / n)

        # Drawing the number of random particles from Binomial(N, p_inject) and placing them in
        # random slots matches deciding per particle with probability p_inject.
        n_inject = int(self.rng.binomial(n, min(p_inject, 1.0))) if p_inject > 0.0 else 0
        if n_inject > 0:
            slots = self.rng.choice(n, n_inject, replace=False)
            self.particles[slots] = self._sample_uniform(n_inject)

    # -------------------------------------------------------------- estimate
    def estimate(self) -> Tuple[np.ndarray, np.ndarray]:
        """Weighted mean pose (circular mean for yaw) and 3x3 covariance.

        The mean of a heading sample must be taken on the circle: atan2(sum w sin, sum w cos)
        (Mardia and Jupp 2000 [mardia2000directional], Chapter 2). The covariance is the
        weighted sample covariance with wrapped yaw deviations. Both summarize the particle set
        as one Gaussian for publishing and scoring; for a multimodal set (after a kidnap or
        during global localization) the mean can lie between the modes.
        """
        w = self.weights
        mx = float(np.dot(w, self.particles[:, 0]))
        my = float(np.dot(w, self.particles[:, 1]))
        myaw = float(
            np.arctan2(
                np.dot(w, np.sin(self.particles[:, 2])), np.dot(w, np.cos(self.particles[:, 2]))
            )
        )
        dev = np.column_stack(
            (
                self.particles[:, 0] - mx,
                self.particles[:, 1] - my,
                wrap_angle(self.particles[:, 2] - myaw),
            )
        )
        cov = (dev * w[:, None]).T @ dev
        return np.array([mx, my, myaw]), cov
