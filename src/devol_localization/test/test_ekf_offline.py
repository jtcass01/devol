"""Offline tests for the EKF and scan matcher: synthetic trajectory and scans against a toy grid."""

from numpy import (ndarray, array, zeros, arange, cos, sin, sqrt, mean, diag, full, inf,
                   minimum, int8)
from numpy.random import default_rng

from devol_localization.ekf_core import PoseEKF, odometry_delta, wrap_angle
from devol_localization.scan_matcher import DistanceField, ScanMatcher, scan_to_points

RES = 0.05
ORIGIN = (-1.0, -1.0, 0.0)
LASER_POSE = (0.32825, 0.0, 0.0)
ANGLE_MIN, ANGLE_MAX, N_BEAMS = -2.356, 2.356, 540
RANGE_MIN, RANGE_MAX = 0.05, 25.0


def toy_grid() -> ndarray:
    """12 m x 8 m room with walls, a few boxes and a partition, in OccupancyGrid layout."""
    h, w = int(8 / RES), int(12 / RES)
    g: ndarray = zeros((h, w), dtype=int8)
    g[:2, :] = 100
    g[-2:, :] = 100
    g[:, :2] = 100
    g[:, -2:] = 100

    def box(x0, y0, x1, y1):
        c0, r0 = int((x0 - ORIGIN[0]) / RES), int((y0 - ORIGIN[1]) / RES)
        c1, r1 = int((x1 - ORIGIN[0]) / RES), int((y1 - ORIGIN[1]) / RES)
        g[r0:r1, c0:c1] = 100

    box(2.0, 1.0, 2.6, 1.6)
    box(6.0, 4.0, 7.0, 4.4)
    box(4.0, 5.5, 4.3, 7.0)
    box(8.5, 0.5, 9.0, 2.5)
    return g


def raycast(grid: ndarray, pose, rng=None, sigma: float = 0.0) -> ndarray:
    """Simulated 2D lidar ranges from the robot base pose."""
    c, s = cos(pose[2]), sin(pose[2])
    lx = pose[0] + c * LASER_POSE[0] - s * LASER_POSE[1]
    ly = pose[1] + s * LASER_POSE[0] + c * LASER_POSE[1]
    angles = pose[2] + LASER_POSE[2] + ANGLE_MIN + (ANGLE_MAX - ANGLE_MIN) / (N_BEAMS - 1) * arange(N_BEAMS)
    steps = arange(RANGE_MIN, RANGE_MAX, RES / 2)
    xs = lx + cos(angles)[:, None] * steps[None, :]
    ys = ly + sin(angles)[:, None] * steps[None, :]
    cols = ((xs - ORIGIN[0]) / RES).astype(int)
    rows = ((ys - ORIGIN[1]) / RES).astype(int)
    h, w = grid.shape
    inside = (cols >= 0) & (rows >= 0) & (cols < w) & (rows < h)
    hit = zeros(xs.shape, dtype=bool)
    hit[inside] = grid[rows[inside], cols[inside]] >= 50
    ranges = full(N_BEAMS, inf)
    any_hit = hit.any(axis=1)
    ranges[any_hit] = steps[hit[any_hit].argmax(axis=1)]
    if rng is not None and sigma > 0.0:
        ranges = ranges + rng.normal(0.0, sigma, N_BEAMS)
    return minimum(ranges, RANGE_MAX)


def scan_points(ranges: ndarray, beam_step: int = 4) -> ndarray:
    return scan_to_points(ranges, ANGLE_MIN, (ANGLE_MAX - ANGLE_MIN) / (N_BEAMS - 1),
                          RANGE_MIN, RANGE_MAX, beam_step, LASER_POSE)


def trajectory(dt: float = 0.05):
    """Slow loop through the room: (x, y, yaw) ground truth at odometry rate."""
    poses = [array([0.0, 0.2, 0.0])]
    # (v, wz, duration): straights, two left turns, a turn in place, a reverse and a gentle arc.
    # Stays at least 0.7 m clear of every obstacle in toy_grid().
    segments = [(0.4, 0.0, 15.0), (0.25, 0.3, 5.24), (0.4, 0.0, 3.5), (0.25, 0.3, 5.24),
                (0.4, 0.0, 12.0), (0.0, 0.4, 4.0), (-0.2, 0.0, 4.0), (0.4, -0.1, 6.0)]
    for v, wz, duration in segments:
        for _ in range(int(duration / dt)):
            x, y, th = poses[-1]
            poses.append(array([x + v * dt * cos(th), y + v * dt * sin(th), wrap_angle(th + wz * dt)]))
    return poses


def noisy_odometry(truth, rng, alphas=(0.05, 0.005, 0.05, 0.005), bias_yaw_rate: float = 0.002):
    """Corrupts true increments with the same odometry noise model plus a small heading bias (slip)."""
    a1, a2, a3, a4 = alphas
    odom = [array([0.0, 0.0, 0.0])]
    for prev, cur in zip(truth[:-1], truth[1:]):
        r1, t, r2 = odometry_delta(prev, cur)
        r1n = r1 + rng.normal(0, sqrt(a1 * r1 ** 2 + a2 * t ** 2))
        tn = t + rng.normal(0, sqrt(a3 * t ** 2 + a4 * (r1 ** 2 + r2 ** 2)))
        r2n = r2 + rng.normal(0, sqrt(a1 * r2 ** 2 + a2 * t ** 2)) + bias_yaw_rate * abs(t)
        x, y, th = odom[-1]
        odom.append(array([x + tn * cos(th + r1n), y + tn * sin(th + r1n), wrap_angle(th + r1n + r2n)]))
    return odom


def test_odometry_delta_roundtrip_and_reverse():
    prev = array([1.0, 2.0, 0.3])
    for cur in (array([1.2, 2.1, 0.5]), array([0.8, 1.9, 0.25]), array([1.0, 2.0, -0.4])):
        r1, t, r2 = odometry_delta(prev, cur)
        ekf = PoseEKF()
        ekf.reset(prev, diag([1e-4] * 3))
        ekf.predict(r1, t, r2)
        assert abs(ekf.x[0] - cur[0]) < 1e-9 and abs(ekf.x[1] - cur[1]) < 1e-9
        assert abs(wrap_angle(ekf.x[2] - cur[2])) < 1e-9
    # Backing straight up is a negative translation, not a pair of half turns.
    r1, t, r2 = odometry_delta(prev, prev + array([-0.1 * cos(0.3), -0.1 * sin(0.3), 0.0]))
    assert t < 0 and abs(r1) < 1e-9 and abs(r2) < 1e-9


def test_correction_pulls_toward_measurement_and_gates_outliers():
    ekf = PoseEKF()
    ekf.reset([0.0, 0.0, 0.0], diag([0.04, 0.04, 0.01]))
    assert ekf.correct([0.1, -0.1, 0.05], diag([0.04, 0.04, 0.01]))
    assert abs(ekf.x[0] - 0.05) < 1e-9 and abs(ekf.x[1] + 0.05) < 1e-9
    assert ekf.P[0, 0] < 0.04
    before = ekf.x.copy()
    assert not ekf.correct([5.0, 5.0, 0.0], diag([0.01, 0.01, 0.01]))
    assert (ekf.x == before).all()


def test_scan_matcher_recovers_offset():
    grid = toy_grid()
    matcher = ScanMatcher(DistanceField(grid, RES, ORIGIN))
    truth = array([4.5, 3.0, 0.4])
    pts = scan_points(raycast(grid, truth))
    result = matcher.match(pts, truth + array([0.25, -0.2, 0.08]))
    assert result is not None
    assert sqrt((result.pose[0] - truth[0]) ** 2 + (result.pose[1] - truth[1]) ** 2) < 0.03
    assert abs(wrap_angle(result.pose[2] - truth[2])) < 0.01


def run_trial(seed: int, range_sigma: float = 0.01, scan_every: int = 2):
    """Runs dead reckoning and the EKF over one synthetic run. Returns (dr_rmse, ekf_rmse, ekf_yaw_rmse)."""
    rng = default_rng(seed)
    grid = toy_grid()
    matcher = ScanMatcher(DistanceField(grid, RES, ORIGIN))
    truth = trajectory()
    odom = noisy_odometry(truth, rng)

    ekf = PoseEKF()
    ekf.reset(truth[0], diag([0.01, 0.01, 0.0025]))
    map_to_odom = truth[0]
    dr_err, ekf_err, yaw_err = [], [], []
    for k in range(1, len(truth)):
        ekf.predict(*odometry_delta(odom[k - 1], odom[k]))
        if k % scan_every == 0:  # odometry at 20 Hz, scans at 10 Hz
            result = matcher.match(scan_points(raycast(grid, truth[k], rng, range_sigma)), ekf.x)
            if result is not None:
                ekf.correct(result.pose, result.covariance)
        c, s = cos(map_to_odom[2]), sin(map_to_odom[2])
        dr = array([map_to_odom[0] + c * odom[k][0] - s * odom[k][1],
                    map_to_odom[1] + s * odom[k][0] + c * odom[k][1]])
        dr_err.append(((dr - truth[k][:2]) ** 2).sum())
        ekf_err.append(((ekf.x[:2] - truth[k][:2]) ** 2).sum())
        yaw_err.append(wrap_angle(ekf.x[2] - truth[k][2]) ** 2)
    return sqrt(mean(dr_err)), sqrt(mean(ekf_err)), sqrt(mean(yaw_err))


def test_ekf_beats_dead_reckoning():
    for seed in range(3):
        dr_rmse, ekf_rmse, yaw_rmse = run_trial(seed)
        assert ekf_rmse < 0.05, ekf_rmse
        assert yaw_rmse < 0.02, yaw_rmse
        assert ekf_rmse < dr_rmse


if __name__ == '__main__':
    for sigma in (0.01, 0.03, 0.10):
        rows = [run_trial(seed, sigma) for seed in range(5)]
        print(f'range sigma {sigma:.2f} m: dead reckoning RMSE '
              f'{mean([r[0] for r in rows]):.3f} m, EKF RMSE {mean([r[1] for r in rows]):.3f} m, '
              f'EKF yaw RMSE {mean([r[2] for r in rows]):.4f} rad')
