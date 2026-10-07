"""Offline tests for the EKF and scan matcher: synthetic trajectory and scans against a toy grid."""

from numpy import (
    ndarray,
    array,
    zeros,
    arange,
    cos,
    sin,
    sqrt,
    mean,
    diag,
    full,
    inf,
    minimum,
    int8,
)
from numpy.linalg import inv
from numpy.random import default_rng

from devol_localization.ekf_core import PoseEKF, odometry_delta, wrap_angle
from devol_localization.ekf_pipeline import EKFPipeline
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
    return raycast_at(grid, ORIGIN, pose, rng, sigma)


def raycast_at(grid: ndarray, origin, pose, rng=None, sigma: float = 0.0) -> ndarray:
    c, s = cos(pose[2]), sin(pose[2])
    lx = pose[0] + c * LASER_POSE[0] - s * LASER_POSE[1]
    ly = pose[1] + s * LASER_POSE[0] + c * LASER_POSE[1]
    angles = (
        pose[2]
        + LASER_POSE[2]
        + ANGLE_MIN
        + (ANGLE_MAX - ANGLE_MIN) / (N_BEAMS - 1) * arange(N_BEAMS)
    )
    steps = arange(RANGE_MIN, RANGE_MAX, RES / 2)
    xs = lx + cos(angles)[:, None] * steps[None, :]
    ys = ly + sin(angles)[:, None] * steps[None, :]
    cols = ((xs - origin[0]) / RES).astype(int)
    rows = ((ys - origin[1]) / RES).astype(int)
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
    return scan_to_points(
        ranges,
        ANGLE_MIN,
        (ANGLE_MAX - ANGLE_MIN) / (N_BEAMS - 1),
        RANGE_MIN,
        RANGE_MAX,
        beam_step,
        LASER_POSE,
    )


def trajectory(dt: float = 0.05):
    """Slow loop through the room: (x, y, yaw) ground truth at odometry rate."""
    poses = [array([0.0, 0.2, 0.0])]
    # (v, wz, duration): straights, two left turns, a turn in place, a reverse and a gentle arc.
    # Stays at least 0.7 m clear of every obstacle in toy_grid().
    segments = [
        (0.4, 0.0, 15.0),
        (0.25, 0.3, 5.24),
        (0.4, 0.0, 3.5),
        (0.25, 0.3, 5.24),
        (0.4, 0.0, 12.0),
        (0.0, 0.4, 4.0),
        (-0.2, 0.0, 4.0),
        (0.4, -0.1, 6.0),
    ]
    for v, wz, duration in segments:
        for _ in range(int(duration / dt)):
            x, y, th = poses[-1]
            poses.append(
                array([x + v * dt * cos(th), y + v * dt * sin(th), wrap_angle(th + wz * dt)])
            )
    return poses


def noisy_odometry(
    truth, rng, alphas=(0.05, 0.005, 0.05, 0.005), bias_yaw_rate: float = 0.002, slip=(1.0, 1.0)
):
    """Corrupts true increments with Probabilistic Robotics odometry noise, a small heading bias,
    and optional systematic (distance, yaw) scale errors."""
    a1, a2, a3, a4 = alphas
    odom = [array([0.0, 0.0, 0.0])]
    for prev, cur in zip(truth[:-1], truth[1:]):
        r1, t, r2 = odometry_delta(prev, cur)
        r1, t, r2 = r1 * slip[1], t * slip[0], r2 * slip[1]
        r1n = r1 + rng.normal(0, sqrt(a1 * r1**2 + a2 * t**2))
        tn = t + rng.normal(0, sqrt(a3 * t**2 + a4 * (r1**2 + r2**2)))
        r2n = r2 + rng.normal(0, sqrt(a1 * r2**2 + a2 * t**2)) + bias_yaw_rate * abs(t)
        x, y, th = odom[-1]
        odom.append(
            array([x + tn * cos(th + r1n), y + tn * sin(th + r1n), wrap_angle(th + r1n + r2n)])
        )
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


def test_scan_matcher_search_finds_large_heading_error():
    grid = toy_grid()
    matcher = ScanMatcher(DistanceField(grid, RES, ORIGIN))
    truth = array([4.5, 3.0, 0.4])
    pts = scan_points(raycast(grid, truth))
    prior = truth + array([0.6, -0.5, 0.35])  # 20 deg off
    assert (
        matcher.match(pts, prior) is None
        or abs(matcher.match(pts, prior).pose[2] - truth[2]) > 0.05
    )
    result = matcher.match(pts, prior, half_xy=1.0, half_yaw=0.5)
    assert result is not None
    assert sqrt((result.pose[0] - truth[0]) ** 2 + (result.pose[1] - truth[1]) ** 2) < 0.03
    assert abs(wrap_angle(result.pose[2] - truth[2])) < 0.01


def corridor_grid(pillar_spacing: float = 0.0) -> ndarray:
    """30 m x 2 m corridor along x (ends out of lidar range), optionally lined with pillars."""
    h, w = int(2.0 / RES), int(30.0 / RES)
    g: ndarray = zeros((h, w), dtype=int8)
    g[:2, :] = 100
    g[-2:, :] = 100
    if pillar_spacing > 0.0:
        for x in arange(0.0, 29.0, pillar_spacing):
            c = int(x / RES)
            g[2:4, c : c + 2] = 100
            g[-4:-2, c : c + 2] = 100
    return g


def test_corridor_is_not_rejected_as_ambiguous():
    # Along a plain corridor the score is a ridge, not a second peak: fuse the match and let its
    # covariance carry the along-corridor uncertainty instead of dropping the scan.
    grid = corridor_grid()
    origin = (-15.0, -1.0, 0.0)
    matcher = ScanMatcher(DistanceField(grid, RES, origin))
    truth = array([0.0, 0.1, 0.05])
    pts = scan_to_points(
        raycast_at(grid, origin, truth),
        ANGLE_MIN,
        (ANGLE_MAX - ANGLE_MIN) / (N_BEAMS - 1),
        RANGE_MIN,
        RANGE_MAX,
        4,
        LASER_POSE,
    )
    result = matcher.match(pts, truth + array([0.2, 0.05, 0.03]), half_xy=0.5, half_yaw=0.2)
    assert result is not None
    assert abs(result.pose[1] - truth[1]) < 0.03
    assert abs(wrap_angle(result.pose[2] - truth[2])) < 0.01
    assert result.covariance[0, 0] > 10.0 * result.covariance[1, 1]


def test_repeated_structure_is_rejected_as_ambiguous():
    # Pillars every 0.6 m make poses 0.6 m apart look the same: a distinct second peak.
    grid = corridor_grid(pillar_spacing=0.6)
    origin = (-15.0, -1.0, 0.0)
    matcher = ScanMatcher(DistanceField(grid, RES, origin))
    truth = array([0.33, 0.1, 0.0])
    pts = scan_to_points(
        raycast_at(grid, origin, truth),
        ANGLE_MIN,
        (ANGLE_MAX - ANGLE_MIN) / (N_BEAMS - 1),
        RANGE_MIN,
        RANGE_MAX,
        4,
        LASER_POSE,
    )
    assert matcher.match(pts, truth, half_xy=0.9, half_yaw=0.1) is None


def run_trial(
    seed: int,
    range_sigma: float = 0.01,
    scan_every: int = 2,
    slip=(1.0, 1.0),
    initial_error=(0.0, 0.0, 0.0),
    initial_std=(0.1, 0.1, 0.05),
    blackout=(0, 0),
):
    """Runs dead reckoning and the EKF pipeline over one synthetic run.

    :param slip: (distance scale, yaw scale) systematic odometry error, like skid-steer slip.
    :param initial_error: Error added to the filter's initial pose, not reflected in initial_std.
    :param blackout: [start, end) odometry step indices with no scans.
    """
    rng = default_rng(seed)
    grid = toy_grid()
    truth = trajectory()
    odom = noisy_odometry(truth, rng, slip=slip)

    ekf = PoseEKF()
    ekf.reset(truth[0] + array(initial_error), diag(array(initial_std) ** 2))
    pipeline = EKFPipeline(ekf, ScanMatcher(DistanceField(grid, RES, ORIGIN)))
    pipeline.on_odom(odom[0])
    map_to_odom = truth[0]
    dr_err, ekf_err, yaw_err, nees = [], [], [], []
    for k in range(1, len(truth)):
        pipeline.on_odom(odom[k])
        if (
            k % scan_every == 0 and not blackout[0] <= k < blackout[1]
        ):  # odometry 20 Hz, scans 10 Hz
            pipeline.on_scan(scan_points(raycast(grid, truth[k], rng, range_sigma)))
        c, s = cos(map_to_odom[2]), sin(map_to_odom[2])
        dr = array(
            [
                map_to_odom[0] + c * odom[k][0] - s * odom[k][1],
                map_to_odom[1] + s * odom[k][0] + c * odom[k][1],
            ]
        )
        err = ekf.x - truth[k]
        err[2] = wrap_angle(err[2])
        dr_err.append(((dr - truth[k][:2]) ** 2).sum())
        ekf_err.append((err[:2] ** 2).sum())
        yaw_err.append(err[2] ** 2)
        nees.append(float(err @ inv(ekf.P) @ err))
    return {
        'dr_rmse': sqrt(mean(dr_err)),
        'rmse': sqrt(mean(ekf_err)),
        'yaw_rmse': sqrt(mean(yaw_err)),
        'final': sqrt(ekf_err[-1]),
        'nees': mean(nees[50:]),
        'stats': pipeline.stats,
    }


def test_ekf_beats_dead_reckoning():
    for seed in range(2):
        r = run_trial(seed)
        assert r['rmse'] < 0.05, r
        assert r['yaw_rmse'] < 0.02, r
        assert r['rmse'] < r['dr_rmse']


def test_ekf_holds_lock_with_slip_and_large_heading_error():
    # Gazebo run: ~5% distance and ~8% yaw odometry error, and the filter started 20 deg off
    # while believing its heading std was 3 deg. The old matcher never matched again.
    r = run_trial(0, slip=(1.05, 1.08), initial_error=(0.3, -0.3, 0.35))
    assert r['final'] < 0.1, r
    assert r['stats'].fused > 0.9 * r['stats'].scans, r
    assert r['rmse'] < r['dr_rmse'] / 3, r


def test_ekf_reacquires_after_scan_blackout():
    # 15 s without scans through two turns lets slip build up a large heading error.
    r = run_trial(1, slip=(1.05, 1.08), blackout=(250, 550))
    assert r['final'] < 0.1, r
    assert r['stats'].fused > 0.9 * (r['stats'].scans - 150), r


def test_ekf_covariance_is_consistent():
    # Mean NEES of a consistent 3-state filter is 3; allow a factor of 2 either way. The Gazebo
    # run reported 0.1 m std while the true error was metres.
    r = run_trial(2, range_sigma=0.03, slip=(1.05, 1.08))
    assert 1.5 < r['nees'] < 6.0, r


if __name__ == '__main__':
    for sigma in (0.01, 0.03, 0.10):
        rows = [run_trial(seed, sigma) for seed in range(5)]
        print(
            f'range sigma {sigma:.2f} m: dead reckoning RMSE '
            f'{mean([r["dr_rmse"] for r in rows]):.3f} m, EKF RMSE {mean([r["rmse"] for r in rows]):.3f} m, '
            f'EKF yaw RMSE {mean([r["yaw_rmse"] for r in rows]):.4f} rad, NEES {mean([r["nees"] for r in rows]):.1f}'
        )
    for label, kw in (
        ('slip + 20 deg start error', dict(slip=(1.05, 1.08), initial_error=(0.3, -0.3, 0.35))),
        ('slip + 15 s scan blackout', dict(slip=(1.05, 1.08), blackout=(250, 550))),
    ):
        r = run_trial(0, **kw)
        print(
            f'{label}: dead reckoning RMSE {r["dr_rmse"]:.3f} m, EKF RMSE {r["rmse"]:.3f} m, '
            f'final {r["final"]:.3f} m, NEES {r["nees"]:.1f}, fused {r["stats"].fused}/{r["stats"].scans}, '
            f're-acquired {r["stats"].reacquired}'
        )


def spin_with_late_scans(stamped: bool, lag: float = 0.3, seed: int = 0):
    """Spins in place at 3 rad/s while odometry over-counts the rotation by 25% (skid-steer slip
    in fast turns), with scans delivered `lag` seconds after their stamps, as when the matcher
    falls behind. Returns the yaw error (rad) once every scan has been processed."""
    rng = default_rng(seed)
    grid = toy_grid()
    pipeline = EKFPipeline(PoseEKF(), ScanMatcher(DistanceField(grid, RES, ORIGIN)))
    truth0 = array([4.5, 3.0, 0.4])
    pipeline.ekf.reset(truth0, diag([0.03, 0.03, 0.02]) ** 2)

    def turned(t):
        return 3.0 * (min(max(t, 0.5), 2.0) - 0.5)

    def truth_at(t):
        return array([truth0[0], truth0[1], wrap_angle(truth0[2] + turned(t))])

    events = [(k / 30.0, 0) for k in range(int(3.5 * 30))]
    events += [(k / 20.0 + lag, 1) for k in range(int(3.0 * 20))]
    events.sort()
    for t, kind in events:
        if kind == 0:
            odom = (0.0, 0.0, wrap_angle(1.25 * turned(t)))
            pipeline.on_odom(odom, t if stamped else None)
        else:
            stamp = t - lag
            points = scan_points(raycast(grid, truth_at(stamp), rng, 0.01))
            pipeline.on_scan(points, stamp if stamped else None)
    return abs(wrap_angle(pipeline.ekf.x[2] - truth_at(events[-1][0])[2])), pipeline.stats


def test_late_scans_are_fused_at_their_stamps():
    # The Gazebo study diverged in a fast turn at Goal 2: odometry over-counted the turn and
    # scans fused late against newer odometry pulled the heading the wrong way.
    err, stats = spin_with_late_scans(stamped=True)
    assert err < 0.02, (err, stats)
    assert stats.fused == stats.scans and stats.rewound > 0.9 * stats.scans, stats
    # Fused at arrival instead, the same scans lose lock.
    err, stats = spin_with_late_scans(stamped=False)
    assert err > 0.3 and stats.failed_in_row > 20, (err, stats)


def test_scan_newer_than_odometry_waits_for_the_next_message():
    grid = toy_grid()
    truth = array([4.5, 3.0, 0.4])
    pipeline = EKFPipeline(PoseEKF(), ScanMatcher(DistanceField(grid, RES, ORIGIN)))
    pipeline.ekf.reset(truth, diag([0.05, 0.05, 0.02]) ** 2)
    pipeline.on_odom((0.0, 0.0, 0.0), 1.0)
    assert pipeline.on_scan(scan_points(raycast(grid, truth)), 1.02) is None
    assert pipeline.stats.fused == 0
    assert pipeline.on_odom((0.0, 0.0, 0.0), 1.033) is not None
    assert pipeline.stats.fused == 1 and pipeline.stats.rewound == 1
    # A scan far ahead of the odometry clock (mismatched clocks) is fused at once, not held.
    assert pipeline.on_scan(scan_points(raycast(grid, truth)), 5.0) is not None


def test_scan_older_than_history_is_dropped_and_time_reset_clears_it():
    grid = toy_grid()
    truth = array([4.5, 3.0, 0.4])
    pipeline = EKFPipeline(PoseEKF(), ScanMatcher(DistanceField(grid, RES, ORIGIN)), history=1.0)
    pipeline.ekf.reset(truth, diag([0.05, 0.05, 0.02]) ** 2)
    for k in range(60):
        pipeline.on_odom((0.0, 0.0, 0.0), k / 30.0)
    assert pipeline.on_scan(scan_points(raycast(grid, truth)), 0.1) is None
    assert pipeline.stats.stale == 1
    # A bag loop restarts the clock: the history starts over instead of rewinding into the old run.
    pipeline.on_odom((0.0, 0.0, 0.0), 0.0)
    pipeline.on_odom((0.0, 0.0, 0.0), 0.033)
    assert pipeline.on_scan(scan_points(raycast(grid, truth)), 0.02) is not None
