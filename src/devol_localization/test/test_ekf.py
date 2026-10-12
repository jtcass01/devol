import numpy as np
import pytest
from numpy.testing import assert_array_almost_equal

from devol_localization.ekf import EKF


class MockDiffDrive:
    def forward(self, x, u, v, dt):
        # A dummy forward model: just adds control u to x
        return x + np.array([u[0] * dt, u[1] * dt, 0.0]) + v

    def linearize(self, x, u, dt):
        # Dummy Jacobian: Identity matrix
        return np.eye(3), np.eye(3)


def test_ekf_zero_innovation():
    """Test 1: If the measurement exactly matches the prediction, state remains unchanged."""
    diff_drive = MockDiffDrive()
    x_hat = np.array([1.0, 2.0, 0.5])
    u = np.array([0.0, 0.0])  # Robot is standing still
    dt = 1.0

    y_pose = np.array([1.0, 2.0, 0.5])  # LiDAR measures exactly where we are

    sigma = np.eye(3) * 0.1
    V = np.eye(3) * 0.1
    W = np.eye(3) * 0.1

    x_upd, sigma_upd = EKF(diff_drive, y_pose, x_hat, sigma, u, dt, V, W)

    # State should remain exactly 1.0, 2.0, 0.5
    assert_array_almost_equal(x_upd, y_pose)

    # Covariance should decrease (we got a measurement, so we are more certain)
    assert np.trace(sigma_upd) < np.trace(sigma + V)


def test_ekf_trust_measurement():
    """Test 2: High process uncertainty, low measurement noise.
    Filter should snap to the LiDAR measurement."""
    diff_drive = MockDiffDrive()
    x_hat = np.array([0.0, 0.0, 0.0])
    u = np.array([0.0, 0.0])
    dt = 1.0

    y_pose = np.array([5.0, -5.0, 1.5])  # LiDAR says we teleported

    sigma = np.eye(3) * 1000.0  # We have NO idea where we are
    V = np.eye(3) * 1000.0  # Odometry is terrible
    W = np.eye(3) * 0.001  # LiDAR is incredibly accurate

    x_upd, sigma_upd = EKF(diff_drive, y_pose, x_hat, sigma, u, dt, V, W)

    # Filter should trust the LiDAR completely
    assert_array_almost_equal(x_upd, y_pose, decimal=2)


def test_ekf_trust_prediction():
    """Test 3: High measurement noise, low process uncertainty.
    Filter should ignore the LiDAR and trust the odometry."""
    diff_drive = MockDiffDrive()
    x_hat = np.array([0.0, 0.0, 0.0])
    u = np.array([1.0, 0.0])  # Moving forward 1m/s
    dt = 1.0

    # Prediction should be [1.0, 0.0, 0.0] based on our mock
    expected_pred = np.array([1.0, 0.0, 0.0])

    y_pose = np.array([50.0, -50.0, 3.14])  # Lidar got a wildly bad reflection

    sigma = np.eye(3) * 0.001  # We are very confident in our state
    V = np.eye(3) * 0.001  # Odometry is perfect
    W = np.eye(3) * 1000.0  # LiDAR is blinded/noisy

    x_upd, sigma_upd = EKF(diff_drive, y_pose, x_hat, sigma, u, dt, V, W)

    # Filter should ignore LiDAR and stay near [1.0, 0.0, 0.0]
    assert_array_almost_equal(x_upd, expected_pred, decimal=2)


def test_ekf_angle_wrapping():
    """Test 4: Innovation should wrap correctly across the -pi/pi boundary."""
    diff_drive = MockDiffDrive()

    # Robot is pointing slightly left of straight backward (+179 deg)
    x_hat = np.array([0.0, 0.0, np.pi - 0.01])
    u = np.array([0.0, 0.0])
    dt = 1.0

    # Lidar measures slightly right of straight backward (-179 deg)
    y_pose = np.array([0.0, 0.0, -np.pi + 0.01])

    sigma = np.eye(3) * 0.1
    V = np.eye(3) * 0.1
    W = np.eye(3) * 0.1

    x_upd, sigma_upd = EKF(diff_drive, y_pose, x_hat, sigma, u, dt, V, W)

    # The filter shouldn't jump to 0. It should realize the error is tiny.
    # The average of +179 and -179 (when weights are equal) is +-180 (pi or -pi)
    assert abs(abs(x_upd[2]) - np.pi) < 0.02
