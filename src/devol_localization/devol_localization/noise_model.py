"""Controlled noise injection for the localization study, kept free of ROS for offline tests.

Gazebo's lidar is noise-free and its physics nearly deterministic, so the study injects noise
into the recorded streams with a seeded generator (annotated outline, protocol section):

- Odometry: increments are perturbed with the odometry motion model of Probabilistic Robotics
  (Table 5.6), alpha1 = alpha2 = alpha3 = alpha4 = k * 0.05. The book's variances grow with the
  square of each increment, so the injected noise would depend on the odometry rate. To keep it
  rate independent, the true odometry is cut into segments of fixed length (segment_length metres
  or segment_angle radians, whichever comes first) and each segment is perturbed once. Between
  segment boundaries the noisy odometry follows the true increments, so it is published at the
  input rate without jumps.
  A segment that translates less than min_translation (1 cm; an in-place turn, where skid-steer
  odometry still creeps a fraction of a millimetre) gets its noise spread as a pure rotation, as AMCL
  does: rot1 = 0 and rot2 = rot1 + rot2 in the variances, with the mean motion unchanged. Otherwise the
  direction of the creep is an arbitrary rot1 (e.g. a -0.1 rad turn decomposed into rot1 = -87 deg,
  rot2 = +81 deg) and Table 5.6 injects ~0.5 m and ~20 deg of noise while the robot stands still.
- Lidar: every finite range receives zero-mean Gaussian noise with standard deviation sigma_r.
"""

from typing import Optional, Sequence

import numpy as np

from devol_localization.pose2d import compose, relative, wrap_angle

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"

NOMINAL_ALPHA: float = 0.05


def odometry_delta(prev_pose: Sequence[float], pose: Sequence[float]):
    """(rot1, trans, rot2) between two odometry poses; reverse motion is a negative translation."""
    dx, dy = float(pose[0] - prev_pose[0]), float(pose[1] - prev_pose[1])
    trans = float(np.hypot(dx, dy))
    dyaw = float(wrap_angle(pose[2] - prev_pose[2]))
    if trans < 1e-9:
        return 0.0, 0.0, dyaw
    rot1 = float(wrap_angle(np.arctan2(dy, dx) - prev_pose[2]))
    if abs(rot1) > np.pi / 2.0:
        rot1 = float(wrap_angle(rot1 - np.pi))
        trans = -trans
    return rot1, trans, float(wrap_angle(dyaw - rot1))


def sample_motion(rot1: float, trans: float, rot2: float, alphas: Sequence[float],
                  rng: np.random.Generator, min_translation: float = 0.0):
    """Draws a perturbed (rot1, trans, rot2), Probabilistic Robotics Table 5.6.

    Below min_translation the variances treat the motion as a pure rotation (rot1 = 0,
    rot2 = rot1 + rot2); the mean is the given decomposition either way.
    """
    a1, a2, a3, a4 = alphas
    e1, e2 = (0.0, rot1 + rot2) if abs(trans) < min_translation else (rot1, rot2)
    r1 = rot1 + rng.normal(0.0, np.sqrt(a1 * e1 ** 2 + a2 * trans ** 2))
    t = trans + rng.normal(0.0, np.sqrt(a3 * trans ** 2 + a4 * (e1 ** 2 + e2 ** 2)))
    r2 = rot2 + rng.normal(0.0, np.sqrt(a1 * e2 ** 2 + a2 * trans ** 2))
    return r1, t, r2


def apply_motion(pose: Sequence[float], rot1: float, trans: float, rot2: float) -> np.ndarray:
    heading = pose[2] + rot1
    return np.array([pose[0] + trans * np.cos(heading), pose[1] + trans * np.sin(heading),
                     float(wrap_angle(heading + rot2))])


class OdometryNoiseInjector:
    """Turns a stream of true odometry poses into a noisy one."""

    def __init__(self, k: float, seed: Optional[int] = None, alpha_nominal: float = NOMINAL_ALPHA,
                 segment_length: float = 0.1, segment_angle: float = 0.1, min_translation: float = 0.01) -> None:
        """
        :param k: Noise scale; alpha1..4 = k * alpha_nominal. k = 0 passes odometry through.
        :param segment_length: Distance (m) after which an increment is perturbed.
        :param segment_angle: Rotation (rad) after which an increment is perturbed.
        :param min_translation: Segments translating less than this (m) get pure-rotation noise.
        """
        self.alphas = (k * alpha_nominal,) * 4
        self.enabled: bool = k > 0.0
        self.rng = np.random.default_rng(seed)
        self.segment_length = segment_length
        self.segment_angle = segment_angle
        self.min_translation = min_translation
        self._anchor_true: Optional[np.ndarray] = None   # true odom pose at the last boundary
        self._anchor_noisy: Optional[np.ndarray] = None  # noisy odom pose at the last boundary

    def reset(self) -> None:
        self._anchor_true = None
        self._anchor_noisy = None

    def __call__(self, true_pose: Sequence[float]) -> np.ndarray:
        """Returns the noisy odometry pose for the next true odometry pose."""
        pose = np.asarray(true_pose, dtype=float)
        if not self.enabled:
            return pose.copy()
        if self._anchor_true is None:
            # The noisy stream starts where the true one does.
            self._anchor_true = pose.copy()
            self._anchor_noisy = pose.copy()
            return pose.copy()
        rot1, trans, rot2 = odometry_delta(self._anchor_true, pose)
        if abs(trans) >= self.segment_length or abs(wrap_angle(rot1 + rot2)) >= self.segment_angle:
            noisy = sample_motion(rot1, trans, rot2, self.alphas, self.rng, self.min_translation)
            self._anchor_noisy = apply_motion(self._anchor_noisy, *noisy)
            self._anchor_true = pose.copy()
            return self._anchor_noisy.copy()
        # Inside a segment: follow the true increment from the last boundary.
        return compose(self._anchor_noisy, relative(self._anchor_true, pose))


def add_range_noise(ranges: Sequence[float], sigma: float, range_min: float, range_max: float,
                    rng: np.random.Generator) -> np.ndarray:
    """Adds N(0, sigma^2) to every valid range, clipped to the sensor limits.

    Returns that miss (inf, nan or outside the limits) are left untouched so the filters still
    treat them as no-returns.
    """
    r = np.asarray(ranges, dtype=float).copy()
    if sigma <= 0.0:
        return r
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r[valid] = np.clip(r[valid] + rng.normal(0.0, sigma, int(valid.sum())), range_min, range_max)
    return r
