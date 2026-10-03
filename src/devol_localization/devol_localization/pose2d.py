"""Planar pose helpers shared by the experiment nodes (noise injection, scoring, visualization).

Poses are (x, y, yaw) sequences. Pure numpy so they can be tested without ROS.
"""

from typing import Sequence

import numpy as np

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


def wrap_angle(a):
    """Wraps an angle (or array of angles) to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def yaw_from_quaternion(q) -> float:
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def set_quaternion_yaw(q, yaw: float) -> None:
    q.x = 0.0
    q.y = 0.0
    q.z = float(np.sin(yaw / 2.0))
    q.w = float(np.cos(yaw / 2.0))


def compose(a: Sequence[float], b: Sequence[float]) -> np.ndarray:
    """a (+) b."""
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], float(wrap_angle(a[2] + b[2]))])


def inverse(a: Sequence[float]) -> np.ndarray:
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([-c * a[0] - s * a[1], s * a[0] - c * a[1], float(wrap_angle(-a[2]))])


def relative(a: Sequence[float], b: Sequence[float]) -> np.ndarray:
    """a^-1 (+) b: pose b expressed in the frame of pose a."""
    return compose(inverse(a), b)


def transform_points(pose: Sequence[float], points: np.ndarray) -> np.ndarray:
    """Maps (N, 2) points from the frame of `pose` into the parent frame."""
    c, s = np.cos(pose[2]), np.sin(pose[2])
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    return np.column_stack((pose[0] + c * pts[:, 0] - s * pts[:, 1], pose[1] + s * pts[:, 0] + c * pts[:, 1]))


def covariance_3x3(cov36: Sequence[float]) -> np.ndarray:
    """(x, y, yaw) block of a 6x6 ROS pose covariance."""
    c = np.asarray(cov36, dtype=float).reshape(6, 6)
    idx = [0, 1, 5]
    return c[np.ix_(idx, idx)]
