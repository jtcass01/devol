"""Extended Kalman filter for planar (x, y, yaw) localization.

Pure numpy, no ROS, so the filter math can be tested offline.

Prediction uses the odometry motion model (rot1, trans, rot2) from
Probabilistic Robotics, Table 5.6, with noise parameters alpha1..alpha4.
The correction step takes a full pose observation (x, y, yaw) in the map
frame, such as the output of a scan matcher, so the measurement Jacobian is
the identity.
"""

from typing import Optional, Sequence, Tuple

from numpy import ndarray, array, asarray, cos, sin, arctan2, pi, eye, diag, zeros
from numpy.linalg import inv

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"

# Chi-square 99% quantile for 3 degrees of freedom.
CHI2_3DOF_99: float = 11.345


def wrap_angle(angle: float) -> float:
    """Wraps an angle to [-pi, pi)."""
    return float((angle + pi) % (2.0 * pi) - pi)


def odometry_delta(prev_pose: Sequence[float], pose: Sequence[float]) -> Tuple[float, float, float]:
    """Decomposes the motion between two odometry poses into (rot1, trans, rot2).

    Reverse motion is folded into a negative translation so that a robot
    backing up does not register two half-turns.
    """
    dx: float = float(pose[0] - prev_pose[0])
    dy: float = float(pose[1] - prev_pose[1])
    trans: float = float((dx * dx + dy * dy) ** 0.5)
    dyaw: float = wrap_angle(pose[2] - prev_pose[2])
    if trans < 1e-6:
        return 0.0, 0.0, dyaw
    rot1: float = wrap_angle(arctan2(dy, dx) - prev_pose[2])
    if abs(rot1) > pi / 2.0:
        rot1 = wrap_angle(rot1 - pi)
        trans = -trans
    rot2: float = wrap_angle(dyaw - rot1)
    return rot1, trans, rot2


class PoseEKF:
    """EKF over the state [x, y, yaw] in the map frame."""

    def __init__(self, alphas: Sequence[float] = (0.05, 0.005, 0.05, 0.005),
                 min_motion_std: Tuple[float, float] = (0.002, 0.002)) -> None:
        """
        :param alphas: Odometry noise (alpha1..alpha4): rot from rot, rot from trans,
                       trans from trans, trans from rot (Probabilistic Robotics, 5.4).
        :param min_motion_std: Floor on the (trans, rot) noise std per step, applied
                               only when the robot moves, so a parked robot stays put.
        """
        self.alphas: ndarray = asarray(alphas, dtype=float)
        self.min_motion_std: Tuple[float, float] = min_motion_std
        self.x: ndarray = zeros(3)
        self.P: ndarray = eye(3)
        self.initialized: bool = False

    def reset(self, pose: Sequence[float], covariance: ndarray) -> None:
        self.x = array([pose[0], pose[1], wrap_angle(pose[2])], dtype=float)
        self.P = asarray(covariance, dtype=float).reshape(3, 3).copy()
        self.initialized = True

    def predict(self, rot1: float, trans: float, rot2: float) -> None:
        """Propagates the state through one odometry increment."""
        a1, a2, a3, a4 = self.alphas
        theta: float = self.x[2]
        heading: float = theta + rot1

        self.x = self.x + array([trans * cos(heading), trans * sin(heading), rot1 + rot2])
        self.x[2] = wrap_angle(self.x[2])

        # Jacobian of the motion model w.r.t. the state.
        G: ndarray = eye(3)
        G[0, 2] = -trans * sin(heading)
        G[1, 2] = trans * cos(heading)

        # Jacobian w.r.t. the control (rot1, trans, rot2).
        V: ndarray = array([[-trans * sin(heading), cos(heading), 0.0],
                            [trans * cos(heading), sin(heading), 0.0],
                            [1.0, 0.0, 1.0]])

        r1, t, r2 = abs(rot1), abs(trans), abs(rot2)
        var_rot1: float = a1 * r1 ** 2 + a2 * t ** 2
        var_trans: float = a3 * t ** 2 + a4 * (r1 ** 2 + r2 ** 2)
        var_rot2: float = a1 * r2 ** 2 + a2 * t ** 2
        if t > 0.0 or r1 > 0.0 or r2 > 0.0:
            trans_floor, rot_floor = self.min_motion_std
            var_trans = max(var_trans, trans_floor ** 2)
            var_rot1 = max(var_rot1, rot_floor ** 2)
            var_rot2 = max(var_rot2, rot_floor ** 2)
        M: ndarray = diag([var_rot1, var_trans, var_rot2])

        self.P = G @ self.P @ G.T + V @ M @ V.T

    def innovation(self, z: Sequence[float], R: ndarray) -> Tuple[ndarray, ndarray, float]:
        """Returns (innovation, innovation covariance, squared Mahalanobis distance)."""
        y: ndarray = asarray(z, dtype=float) - self.x
        y[2] = wrap_angle(y[2])
        S: ndarray = self.P + R
        d2: float = float(y @ inv(S) @ y)
        return y, S, d2

    def correct(self, z: Sequence[float], R: ndarray,
                gate: Optional[float] = CHI2_3DOF_99) -> bool:
        """Fuses a pose observation z = [x, y, yaw] with covariance R.

        Returns False and leaves the state untouched if the observation fails the
        Mahalanobis gate (pass gate=None to disable gating).
        """
        y, S, d2 = self.innovation(z, R)
        if gate is not None and d2 > gate:
            return False
        K: ndarray = self.P @ inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(self.x[2])
        # Joseph form keeps P symmetric positive definite.
        I_K: ndarray = eye(3) - K
        self.P = I_K @ self.P @ I_K.T + K @ R @ K.T
        return True
