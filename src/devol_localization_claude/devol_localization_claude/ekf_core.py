"""Extended Kalman filter for planar (x, y, yaw) localization.

Pure numpy, no ROS, so the filter math can be tested offline. References are listed in
THEORY.md at the package root; "PR" below is Thrun, Burgard and Fox, Probabilistic Robotics
(2005) [thrun2005probabilistic].

The filter is EKF localization against a known map (Leonard and Durrant-Whyte 1991
[leonard1991mobile]; PR Section 7.4). It follows the generic EKF of PR Table 3.3:

    predict:  mu_bar    = g(u, mu)                              (line 2)
              Sigma_bar = G Sigma G^T + V M V^T                 (line 3, with R_t = V M V^T)
    correct:  K         = Sigma_bar H^T (H Sigma_bar H^T + Q)^-1  (line 4)
              mu        = mu_bar + K (z - h(mu_bar))            (line 5)
              Sigma     = (I - K H) Sigma_bar                   (line 6, Joseph form here)

PR writes the motion noise as R_t and the measurement noise as Q_t; this code follows the more
common Kalman filter convention and calls the measurement noise R.

Prediction uses the odometry motion model of PR Section 5.4 (decomposition into rot1, trans,
rot2, Tables 5.5 and 5.6) with noise parameters alpha1..alpha4, linearized the same way PR
Table 7.2 linearizes the velocity model: G is the Jacobian of g with respect to the state,
V the Jacobian with respect to the control, and M the control noise covariance mapped into
state space by V. Unlike the book, each variance grows linearly with the size of the
increment rather than with its square, so the accumulated uncertainty depends on the distance
and angle travelled and not on the odometry rate (with squared terms, 1 m split into 50 steps
carries 1/50 of the variance of a single 1 m step, which made the filter overconfident).

The correction step takes a full pose observation (x, y, yaw) in the map frame, such as the
output of the scan matcher in scan_matcher.py, so h is the identity and so is its Jacobian H.
This replaces the landmark range-bearing model of PR Table 7.2: the scan matcher
does the data association and the nonlinear work, and the EKF fuses its result as a direct
pose measurement. Observations are validated with a chi-square gate on the innovation's
Mahalanobis distance (Bar-Shalom, Li and Kirubarajan 2001 [barshalom2001estimation],
Section 5.4 on the normalized innovation squared).
"""

from typing import Optional, Sequence, Tuple

from numpy import ndarray, array, asarray, cos, sin, arctan2, pi, eye, diag, zeros
from numpy.linalg import inv

__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'

# Chi-square 99% quantile for 3 degrees of freedom: a consistent filter's squared Mahalanobis
# innovation for a 3-D measurement is chi-square(3) distributed, so 1% of good observations
# are rejected by this gate.
CHI2_3DOF_99: float = 11.345


def wrap_angle(angle: float) -> float:
    """Wraps an angle to [-pi, pi)."""
    return float((angle + pi) % (2.0 * pi) - pi)


def odometry_delta(
    prev_pose: Sequence[float], pose: Sequence[float]
) -> Tuple[float, float, float]:
    """Decomposes the motion between two odometry poses into (rot1, trans, rot2).

    PR Tables 5.5 and 5.6, lines 2-4: rot1 = atan2(dy, dx) - yaw, trans = sqrt(dx^2 + dy^2),
    rot2 = yaw' - yaw - rot1. The odometry frame's absolute pose is never used, only the
    relative motion, which is what makes the model insensitive to odometry drift (PR Section 5.4).

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
    """EKF over the state [x, y, yaw] in the map frame (mean self.x, covariance self.P)."""

    def __init__(self, alphas: Sequence[float] = (0.02, 0.01, 0.01, 0.002)) -> None:
        """
        :param alphas: Odometry noise (alpha1..alpha4), each a variance per unit of motion:
                       rot from rot (rad^2/rad), rot from trans (rad^2/m),
                       trans from trans (m^2/m), trans from rot (m^2/rad).
        """
        self.alphas: ndarray = asarray(alphas, dtype=float)
        self.x: ndarray = zeros(3)
        self.P: ndarray = eye(3)
        self.initialized: bool = False

    def reset(self, pose: Sequence[float], covariance: ndarray) -> None:
        """Sets the Gaussian prior bel(x_0) = N(pose, covariance) (PR Section 3.2)."""
        self.x = array([pose[0], pose[1], wrap_angle(pose[2])], dtype=float)
        self.P = asarray(covariance, dtype=float).reshape(3, 3).copy()
        self.initialized = True

    def predict(self, rot1: float, trans: float, rot2: float) -> None:
        """Propagates the state through one odometry increment (EKF prediction, PR Table 3.3
        lines 2-3, with the odometry motion model of PR Section 5.4).

        The noise-free motion model g(u, x), with u = (rot1, trans, rot2), is

            x'   = x + trans cos(yaw + rot1)
            y'   = y + trans sin(yaw + rot1)
            yaw' = yaw + rot1 + rot2
        """
        a1, a2, a3, a4 = self.alphas
        theta: float = self.x[2]
        heading: float = theta + rot1

        # Mean: mu_bar = g(u, mu) (Table 3.3 line 2).
        self.x = self.x + array([trans * cos(heading), trans * sin(heading), rot1 + rot2])
        self.x[2] = wrap_angle(self.x[2])

        # G = dg/dx evaluated at the prior mean (the state Jacobian G_t of PR Section 3.3 and
        # Table 7.2, here for the odometry model).
        G: ndarray = eye(3)
        G[0, 2] = -trans * sin(heading)
        G[1, 2] = trans * cos(heading)

        # V = dg/du, the Jacobian with respect to the control (rot1, trans, rot2); it maps the
        # control noise into state space (the role of V_t in PR Table 7.2).
        V: ndarray = array(
            [
                [-trans * sin(heading), cos(heading), 0.0],
                [trans * cos(heading), sin(heading), 0.0],
                [1.0, 0.0, 1.0],
            ]
        )

        # M: covariance of the control noise, diagonal in (rot1, trans, rot2), with the alpha
        # structure of PR Table 5.6 lines 5-7 (rotation noise from rotation and translation,
        # translation noise from translation and rotation). Here each variance is linear in the
        # increment, not quadratic (see the module docstring), and the rotation-from-translation
        # term a2 * t is split evenly between rot1 and rot2 so the total yaw variance from a
        # translation is a2 * t.
        r1, t, r2 = abs(rot1), abs(trans), abs(rot2)
        var_rot1: float = a1 * r1 + 0.5 * a2 * t
        var_trans: float = a3 * t + a4 * (r1 + r2)
        var_rot2: float = a1 * r2 + 0.5 * a2 * t
        M: ndarray = diag([var_rot1, var_trans, var_rot2])

        # Sigma_bar = G Sigma G^T + V M V^T (Table 3.3 line 3, with R_t = V M V^T as in Table 7.2).
        self.P = G @ self.P @ G.T + V @ M @ V.T

    def innovation(self, z: Sequence[float], R: ndarray) -> Tuple[ndarray, ndarray, float]:
        """Returns (innovation, innovation covariance, squared Mahalanobis distance).

        With h(x) = x and H = I: innovation y = z - mu_bar (yaw wrapped), innovation covariance
        S = H Sigma_bar H^T + R = Sigma_bar + R (the bracket in PR Table 3.3 line 4), and
        d2 = y^T S^-1 y, the normalized innovation squared used for gating.
        """
        y: ndarray = asarray(z, dtype=float) - self.x
        y[2] = wrap_angle(y[2])
        S: ndarray = self.P + R
        d2: float = float(y @ inv(S) @ y)
        return y, S, d2

    def correct(
        self, z: Sequence[float], R: ndarray, gate: Optional[float] = CHI2_3DOF_99
    ) -> bool:
        """Fuses a pose observation z = [x, y, yaw] with covariance R.

        Returns False and leaves the state untouched if the observation fails the
        Mahalanobis gate (pass gate=None to disable gating).
        """
        y, S, d2 = self.innovation(z, R)
        # Validation gate: an observation the filter considers too unlikely (an outlier, or a
        # wrong scan match) is dropped rather than fused.
        if gate is not None and d2 > gate:
            return False
        # Kalman gain K = Sigma_bar H^T S^-1 = Sigma_bar S^-1 (Table 3.3 line 4).
        K: ndarray = self.P @ inv(S)
        # mu = mu_bar + K (z - h(mu_bar)) (Table 3.3 line 5).
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(self.x[2])
        # Sigma = (I - K H) Sigma_bar (Table 3.3 line 6), written in the Joseph form
        # (I - K) Sigma_bar (I - K)^T + K R K^T, which equals it for the optimal gain but keeps P
        # symmetric positive definite under rounding (Bar-Shalom et al. 2001, Section 5.2).
        I_K: ndarray = eye(3) - K
        self.P = I_K @ self.P @ I_K.T + K @ R @ K.T
        return True


def global_initial_state(
    free_xy: ndarray, rng=None, mean: str = 'random'
) -> Tuple[ndarray, ndarray]:
    """Single-Gaussian stand-in for a uniform prior over the free space, for global localization.

    `mean='random'` draws the mean from the free cells and the heading uniformly (pass a seeded
    numpy Generator so each trial is reproducible); `'centroid'` uses the free-space centroid with
    heading 0. Note the factory's free-space centroid is within 0.1 m of the spawn pose (0, 0, 0),
    so 'centroid' starts the filter on the true pose and is not a global-localization test there.
    The covariance is the free space's second moment about the chosen mean, and the heading
    variance is that of a uniform heading, (2 pi)^2 / 12.

    A Gaussian cannot represent the multimodal belief that global localization needs (PR
    Sections 7.1 and 8.3 contrast the EKF's unimodal belief with MCL's); this moment
    match of the uniform prior is the closest unimodal stand-in, and it is what lets the trade
    study run the EKF on the global-localization scenario at all.
    """
    from numpy import outer
    from numpy.random import default_rng

    xy = asarray(free_xy, dtype=float).reshape(-1, 2)
    centroid = xy.mean(axis=0)
    if mean == 'centroid':
        x = array([centroid[0], centroid[1], 0.0])
    elif mean == 'random':
        rng = rng if rng is not None else default_rng()
        x = array([*xy[rng.integers(len(xy))], rng.uniform(-pi, pi)])
    else:
        raise ValueError(f"mean must be 'random' or 'centroid', got {mean}")
    d = centroid - x[:2]
    P = zeros((3, 3))
    P[:2, :2] = (xy - centroid).T @ (xy - centroid) / max(len(xy) - 1, 1) + outer(d, d)
    P[2, 2] = (2.0 * pi) ** 2 / 12.0
    return x, P
