import numpy as np
import pytest
from unittest.mock import patch
from numpy.testing import assert_array_almost_equal

from devol_localization.particle_filter import simple_resample, particle_filter_2d


class MockDiffDrive:
    def forward(self, x, u, v, dt):
        # A dummy forward model: just adds the noise (v) to the state
        return x + v


class MockSensorModel:
    def __init__(self, predefined_weights):
        self.poses_received = []
        self.predefined_weights = predefined_weights
        self.call_count = 0

    def likelihood_field_range_finder(self, z_dists, z_thetas, x, map_dict, z_max, std):
        # Record the pose that was passed in so we can verify the offset math
        self.poses_received.append(x.copy())
        # Return the predetermined weight for this particle
        weight = self.predefined_weights[self.call_count]
        self.call_count += 1
        return weight


def test_simple_resample():
    """Test 1: If one particle has 100% of the weight, resampling should clone it."""
    particles = np.array([[1.0, 1.0, 1.0], [9.0, 9.0, 9.0], [3.0, 3.0, 3.0]])
    weights = np.array([0.0, 1.0, 0.0])

    resampled = simple_resample(particles, weights)

    expected = np.array([[9.0, 9.0, 9.0], [9.0, 9.0, 9.0], [9.0, 9.0, 9.0]])
    assert_array_almost_equal(resampled, expected)


@patch('devol_localization.particle_filter.multivariate_normal')
def test_laser_offset_application(mock_multivariate_normal):
    """Test 2: Verify the TF physical offset is applied to the laser properly."""
    # Force the random noise to be zero so particles don't move
    mock_multivariate_normal.return_value = np.zeros(3)

    # Robot is at origin facing 90 degrees left (pi/2)
    particles = np.array([[0.0, 0.0, np.pi / 2]])

    # Lidar is mounted 0.5m forward and 0.1m left of the center
    laser_offset = np.array([0.5, 0.1, 0.0])

    mock_sensor = MockSensorModel(predefined_weights=[1.0])

    _, _ = particle_filter_2d(
        particles=particles,
        map_dict={},
        motion_model=MockDiffDrive(),
        sensor_model=mock_sensor,
        resample=simple_resample,
        u=np.zeros(2),
        dt=1.0,
        z_dists=[],
        z_thetas=[],
        laser_offset=laser_offset,
    )

    # Because robot is facing pi/2 (North), the +0.5m forward offset moves it +0.5 in Y.
    # The +0.1m left offset moves it -0.1 in X.
    # Expected global laser pose: [-0.1, 0.5, pi/2]
    expected_laser_pose = np.array([-0.1, 0.5, np.pi / 2])

    actual_laser_pose = mock_sensor.poses_received[0]
    assert_array_almost_equal(actual_laser_pose, expected_laser_pose)


@patch('devol_localization.particle_filter.multivariate_normal')
def test_circular_mean_wrapping(mock_multivariate_normal):
    """Test 3: Verify that average heading is calculated correctly across the -pi/pi wrap boundary."""
    # Force the random noise to be zero
    mock_multivariate_normal.return_value = np.zeros(3)

    # Two particles: one pointing slightly right of backward (-179 deg), one slightly left (+179 deg)
    particles = np.array([[0.0, 0.0, -np.pi + 0.01], [0.0, 0.0, np.pi - 0.01]])

    # Give them exactly equal weight
    mock_sensor = MockSensorModel(predefined_weights=[0.5, 0.5])

    _, x_est = particle_filter_2d(
        particles=particles,
        map_dict={},
        motion_model=MockDiffDrive(),
        sensor_model=mock_sensor,
        resample=simple_resample,
        u=np.zeros(2),
        dt=1.0,
        z_dists=[],
        z_thetas=[],
        laser_offset=np.zeros(3),
    )

    # The circular mean of +179 and -179 is exactly +-180 (pi or -pi)
    # If we used a naive average, it would have returned 0.0 (straight forward), which is a catastrophic failure!
    estimated_heading = abs(x_est[2])
    assert abs(estimated_heading - np.pi) < 1e-5
