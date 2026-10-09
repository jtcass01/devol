"""Hybrid localization: an EKF for tracking, with a particle filter running alongside as a kidnap
detector that re-seeds the EKF when it has lost the robot.

Pure numpy, no ROS, so it can be tested offline. The EKF (through its EKFPipeline) and the
particle filter run unchanged on the same odometry and scans; this module only watches the two
and, when they disagree, decides which one is right and resets the EKF's prior (mean and
covariance) from the particle cloud.

The EKF is the published estimate. A re-seed needs all four of the following, on `confirm_updates`
consecutive particle filter updates:

1. Disagreement: the EKF mean lies outside the 99% region of the two estimates' combined
   covariance (squared Mahalanobis distance above the 3-DOF chi-square gate).
2. A confident particle filter: its position and heading spread are below the limits, so it has
   converged on one hypothesis rather than spreading out to search.
3. The scan prefers the particle filter: the scan's mean per-beam log likelihood, in the particle
   filter's own likelihood field, is higher at the particle filter's mean than at the EKF's mean
   by at least `min_loglik_gain`. This is what stops a diverged, collapsed particle cloud (which
   passes test 2) from dragging a good EKF away.
4. The scan fits the particle filter's mean well in absolute terms (mean per-beam log likelihood
   at least `min_pf_loglik`). Right after a kidnap the particle cloud often collapses onto a wrong
   pose for several seconds that still explains the scan better than the lost EKF does (test 3
   passes); re-seeding there throws away the EKF's own chance to re-acquire. On the recorded
   Gazebo drive a tracking PF scores 0.13 to 0.17 (sigma_hit 0.3), those wrong poses -0.5 to -0.3.

The particle filter's covariance is floored before it is used in test 1 and as the new EKF
prior, because the particle cloud is overconfident (its spread is much smaller than its error),
and an EKF re-seeded with a few-centimetre covariance would gate out its next scan matches.

Theory (references in THEORY.md at the package root). The two filters are left as they are
(ekf_core.py and particle_filter.py document them); this module only adds the switching logic.
- The design pairs the strengths that Gutmann and Fox (2002) [gutmann2002experimental] measured:
  Kalman filter localization is the most accurate and efficient while it tracks, Monte Carlo
  localization is the one that recovers after the robot is displaced. Probabilistic Robotics
  Section 7.1 [thrun2005probabilistic] explains why: a single Gaussian cannot represent the
  multimodal belief of the global-localization and kidnapped-robot problems, while particles
  can, with augmented MCL (Table 8.3) supplying the recovery.
- Test 1 is a chi-square test on the difference of two Gaussian estimates, with the sum of
  their covariances as its covariance (treating the two estimates as independent, which is
  conservative because they share odometry and scans): the same normalized innovation
  squared test the EKF uses to gate observations (Bar-Shalom et al. 2001
  [barshalom2001estimation], Section 5.4).
- Tests 3 and 4 score both candidate poses with the particle filter's own measurement model
  (the likelihood field of Probabilistic Robotics Section 6.4, Table 6.3), so the choice
  between the filters is made by the same p(z | x) the particle filter weights particles with.
- Re-seeding the EKF prior from the particle set is moment matching: the particle set is
  summarized by its weighted mean and covariance, as a Gaussian filter would represent it.
  Resetting a filter's belief from the current measurement is also the idea of sensor
  resetting localization (Lenser and Veloso 2000 [lenser2000sensor]), here driven by the
  particle filter's posterior rather than by one measurement.
"""

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from devol_localization.ekf_core import CHI2_3DOF_99, wrap_angle
from devol_localization.particle_filter import LikelihoodField, PFParams

__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


@dataclass
class HybridParams:
    gate: float = CHI2_3DOF_99
    # Particle filter confidence limits (std, m and rad).
    pf_max_std_xy: float = 0.3
    pf_max_std_yaw: float = 0.15
    # Floor on the particle filter covariance (std, m and rad) for the disagreement test and the
    # re-seeded EKF prior.
    cov_floor_xy: float = 0.15
    cov_floor_yaw: float = 0.08
    # Required advantage in mean per-beam log likelihood of the PF mean over the EKF mean.
    min_loglik_gain: float = 0.3
    # Minimum mean per-beam log likelihood of the scan at the PF mean (depends on sigma_hit, z_hit,
    # z_rand and range_max; 0.0 suits the PF defaults).
    min_pf_loglik: float = 0.0
    # Consecutive PF updates that must all call for a re-seed.
    confirm_updates: int = 5
    # PF updates to wait after a re-seed before checking again.
    cooldown_updates: int = 10


@dataclass
class HybridStats:
    checks: int = 0
    disagreements: int = 0
    vetoed_by_scan: int = 0
    vetoed_by_fit: int = 0
    reseeds: int = 0
    last_reseed_stamp: Optional[float] = None


def floor_covariance(cov: np.ndarray, floor_xy: float, floor_yaw: float) -> np.ndarray:
    """Returns cov (symmetrized) with its x, y and yaw variances raised to at least the floors."""
    out = np.asarray(cov, dtype=float).reshape(3, 3).copy()
    out = 0.5 * (out + out.T)
    floors = (floor_xy**2, floor_xy**2, floor_yaw**2)
    for i, f in enumerate(floors):
        if out[i, i] < f:
            out[i, i] = f
    return out


def scan_log_likelihood(
    field: LikelihoodField,
    params: PFParams,
    poses: np.ndarray,
    ranges: Sequence[float],
    angles: Sequence[float],
    range_min: float,
    range_max: float,
    laser_pose=(0.0, 0.0, 0.0),
) -> np.ndarray:
    """Mean per-beam log likelihood of one scan at each pose, with the particle filter's own
    likelihood field model (same beams, sigma_hit, z_hit and z_rand). Returns one value per pose,
    or zeros when the scan has no usable beams."""
    poses = np.atleast_2d(np.asarray(poses, dtype=float))
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    valid = np.isfinite(ranges) & (ranges > range_min) & (ranges < range_max)
    idx = np.flatnonzero(valid)
    if idx.size == 0:
        return np.zeros(poses.shape[0])
    if idx.size > params.max_beams:
        idx = idx[np.linspace(0, idx.size - 1, params.max_beams).round().astype(int)]
    r, a = ranges[idx], angles[idx]
    x, y, th = poses[:, 0:1], poses[:, 1:2], poses[:, 2:3]
    lx, ly, lth = laser_pose
    c, s = np.cos(th), np.sin(th)
    beam = th + lth + a[None, :]
    ex = x + c * lx - s * ly + r[None, :] * np.cos(beam)
    ey = y + s * lx + c * ly + r[None, :] * np.sin(beam)
    d = field.distance(ex, ey)
    gauss = np.exp(-0.5 * (d / params.sigma_hit) ** 2) / (np.sqrt(2.0 * np.pi) * params.sigma_hit)
    return np.log(params.z_hit * gauss + params.z_rand / range_max).mean(axis=1)


class KidnapMonitor:
    """Decides, after each particle filter update, whether the EKF should be re-seeded."""

    def __init__(self, params: Optional[HybridParams] = None) -> None:
        self.params: HybridParams = params or HybridParams()
        self.stats: HybridStats = HybridStats()
        self._streak: int = 0
        self._cooldown: int = 0

    def reset(self) -> None:
        self._streak = 0
        self._cooldown = 0

    def mahalanobis2(
        self, ekf_x: np.ndarray, ekf_P: np.ndarray, pf_x: np.ndarray, pf_P: np.ndarray
    ) -> float:
        """Squared Mahalanobis distance d^T (P_ekf + P_pf)^-1 d between the two means (test 1)."""
        p = self.params
        y = np.asarray(pf_x, dtype=float) - np.asarray(ekf_x, dtype=float)
        y[2] = wrap_angle(y[2])
        S = np.asarray(ekf_P, dtype=float) + floor_covariance(
            pf_P, p.cov_floor_xy, p.cov_floor_yaw
        )
        return float(y @ np.linalg.solve(S, y))

    def pf_confident(self, pf_P: np.ndarray) -> bool:
        """Test 2: the particle set has converged (its std is below the limits)."""
        p = self.params
        std_xy = float(np.sqrt(max(pf_P[0, 0], pf_P[1, 1])))
        return std_xy <= p.pf_max_std_xy and float(np.sqrt(pf_P[2, 2])) <= p.pf_max_std_yaw

    def check(
        self,
        ekf_x: np.ndarray,
        ekf_P: np.ndarray,
        pf_x: np.ndarray,
        pf_P: np.ndarray,
        loglik_gain: Optional[float] = None,
        pf_loglik: Optional[float] = None,
    ) -> bool:
        """Returns True when the EKF should be re-seeded from the particle filter now.

        :param loglik_gain: Scan log likelihood at the PF mean minus that at the EKF mean (mean
                            per beam). None skips test 3 (not recommended; see module doc).
        :param pf_loglik: Scan log likelihood at the PF mean (mean per beam). None skips test 4.
        """
        p = self.params
        self.stats.checks += 1
        if self._cooldown > 0:
            self._cooldown -= 1
            self._streak = 0
            return False
        disagree = self.mahalanobis2(ekf_x, ekf_P, pf_x, pf_P) > p.gate and self.pf_confident(pf_P)
        if disagree:
            self.stats.disagreements += 1
            if loglik_gain is not None and loglik_gain < p.min_loglik_gain:
                self.stats.vetoed_by_scan += 1
                disagree = False
            elif pf_loglik is not None and pf_loglik < p.min_pf_loglik:
                self.stats.vetoed_by_fit += 1
                disagree = False
        self._streak = self._streak + 1 if disagree else 0
        if self._streak >= p.confirm_updates:
            self._streak = 0
            self._cooldown = p.cooldown_updates
            return True
        return False

    def reseed_prior(self, pf_x: np.ndarray, pf_P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """EKF prior to restart from: the PF mean and its floored covariance."""
        p = self.params
        x = np.asarray(pf_x, dtype=float).copy()
        x[2] = wrap_angle(x[2])
        return x, floor_covariance(pf_P, p.cov_floor_xy, p.cov_floor_yaw)


class HybridLocalizer:
    """Glue: runs the monitor after each particle filter update and re-seeds the EKF.

    The caller drives the EKFPipeline and the ParticleFilter exactly as their own nodes do, and
    calls `after_pf_update` with the scan the particle filter has just been weighted with.
    """

    def __init__(self, pipeline, pf, params: Optional[HybridParams] = None) -> None:
        self.pipeline = pipeline
        self.pf = pf
        self.monitor: KidnapMonitor = KidnapMonitor(params)

    @property
    def stats(self) -> HybridStats:
        return self.monitor.stats

    def after_pf_update(
        self,
        ranges: Sequence[float],
        angles: Sequence[float],
        range_min: float,
        range_max: float,
        laser_pose=(0.0, 0.0, 0.0),
        stamp: Optional[float] = None,
    ) -> bool:
        """Checks the EKF against the particle filter; re-seeds it if needed. Returns True on a
        re-seed."""
        ekf = self.pipeline.ekf
        if not (ekf.initialized and self.pf.initialized):
            return False
        pf_x, pf_P = self.pf.estimate()
        gain: Optional[float] = None
        pf_ll: Optional[float] = None
        if self.pf.field is not None:
            ll = scan_log_likelihood(
                self.pf.field,
                self.pf.params,
                np.vstack((pf_x, ekf.x)),
                ranges,
                angles,
                range_min,
                range_max,
                laser_pose,
            )
            gain, pf_ll = float(ll[0] - ll[1]), float(ll[0])
        if not self.monitor.check(ekf.x, ekf.P, pf_x, pf_P, gain, pf_ll):
            return False
        x, P = self.monitor.reseed_prior(pf_x, pf_P)
        ekf.reset(x, P)
        # A pipeline that keeps a state history for late scans must not rewind past the re-seed.
        clear = getattr(self.pipeline, 'clear_history', None)
        if clear is not None:
            clear()
        self.pipeline.stats.failed_in_row = 0
        self.monitor.stats.reseeds += 1
        self.monitor.stats.last_reseed_stamp = stamp
        return True
