"""This code was modified and adapted from Introduction to Robotics Coursework."""

from __future__ import annotations

import numpy as np
from numpy.linalg import inv
from devol_localization.differential_drive import DifferentialDrive


def EKF(
    DiffDrive: DifferentialDrive,
    y_pose: np.ndarray,
    x_hat: np.ndarray,
    sigma: np.ndarray,
    u: np.ndarray,
    dt: float,
    V: np.ndarray,
    W_pose: np.ndarray,
):
    """EKF that fuses odometry with a scan-matched pose observation."""
    # Prediction (Odometry / Motion Model)
    x_hat_prediction = DiffDrive.forward(x_hat, u, np.zeros(3), dt)
    A, _ = DiffDrive.linearize(x_hat, u, dt)
    sigma_prediction = A @ sigma @ A.T + V

    # Correction using Scan-Matched Pose
    # Since y_pose is [x,y,theta], the expected measurement y_hat is just the predicted state
    innovation_error = y_pose - x_hat_prediction

    # We must wrap the angle error to [-pi, pi]
    innovation_error[2] = (innovation_error[2] + np.pi) % (2 * np.pi) - np.pi

    # The Jacobian H is simply a 3x3 Identity matrix because y = x
    H = np.eye(3)

    # S and R (Kalman Gain)
    S = H @ sigma_prediction @ H.T + W_pose
    R = sigma_prediction @ H.T @ inv(S)

    # State update
    x_hat_2 = x_hat_prediction + R @ innovation_error

    # Covariance update using the Joseph Form:
    I_RH = np.eye(3) - R @ H

    sigma_2 = I_RH @ sigma_prediction @ I_RH.T + R @ W_pose @ R.T
    sigma_2 = (sigma_2 + sigma_2.T) / 2.0

    return x_hat_2, sigma_2
