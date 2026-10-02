"""Offline tests of the study tooling: noise injection, scoring and the visualization figure.

No ROS needed: `python3 -m pytest test/test_study_tools.py`.
"""

import json
from pathlib import Path

import numpy as np
import pytest

from devol_localization.metrics import (clopper_pearson, detect_jump, mean_ci95, read_trajectory_csv,
                                        recovery_time, score_estimator, write_summary, write_trajectory_csv)
from devol_localization.noise_model import OdometryNoiseInjector, add_range_noise
from devol_localization.pose2d import compose, inverse, relative, wrap_angle


def drive(n=2000, dt=0.02, v=0.5, w=0.15):
    """Arc at odometry rate dt: 40 s, 20 m."""
    poses = [np.zeros(3)]
    for _ in range(n):
        x, y, th = poses[-1]
        poses.append(np.array([x + v * dt * np.cos(th), y + v * dt * np.sin(th), float(wrap_angle(th + w * dt))]))
    return np.array(poses)


def test_pose_algebra_roundtrip():
    a, b = np.array([1.0, -2.0, 0.7]), np.array([0.3, 0.4, -2.9])
    assert np.allclose(compose(a, relative(a, b)), b)
    assert np.allclose(compose(a, inverse(a)), 0.0, atol=1e-12)


def test_zero_noise_is_pass_through():
    inj = OdometryNoiseInjector(0.0, seed=1)
    for p in drive(50):
        assert np.allclose(inj(p), p)
    r = np.array([1.0, np.inf, 2.0])
    assert np.array_equal(add_range_noise(r, 0.0, 0.05, 25.0, np.random.default_rng(0)), r)


def test_odometry_noise_is_seeded_continuous_and_rate_independent():
    truth = drive()

    def final_errors(step, k, seeds):
        errs = []
        for seed in seeds:
            inj = OdometryNoiseInjector(k, seed=seed)
            out = [inj(p) for p in truth[::step]]
            steps = np.hypot(*np.diff(np.array(out)[:, :2], axis=0).T)
            assert steps.max() < 0.5 * step  # never jumps, whatever the boundary
            errs.append(np.hypot(*(out[-1][:2] - truth[::step][-1][:2])))
        return np.array(errs)
    a = OdometryNoiseInjector(1.0, seed=7)
    b = OdometryNoiseInjector(1.0, seed=7)
    assert all(np.allclose(a(p), b(p)) for p in truth[:300])
    fast, slow = final_errors(1, 1.0, range(40)), final_errors(5, 1.0, range(40))
    # Same spread whether odometry comes at 50 Hz or 10 Hz, and more noise for larger k.
    assert 0.6 < np.sqrt(np.mean(fast ** 2)) / np.sqrt(np.mean(slow ** 2)) < 1.6
    big = final_errors(1, 4.0, range(40))
    assert np.sqrt(np.mean(big ** 2)) > 1.5 * np.sqrt(np.mean(fast ** 2))
    assert np.sqrt(np.mean(fast ** 2)) > 0.05


def test_range_noise_statistics_and_no_returns():
    rng = np.random.default_rng(0)
    r = np.full(20000, 5.0)
    r[:10] = np.inf
    noisy = add_range_noise(r, 0.1, 0.05, 25.0, rng)
    assert np.isinf(noisy[:10]).all()
    assert abs(noisy[10:].std() - 0.1) < 0.005 and abs(noisy[10:].mean() - 5.0) < 0.005


def test_recovery_and_jump_detection():
    t = np.arange(0.0, 60.0, 0.1)
    gt = np.column_stack((0.5 * t, np.zeros_like(t), np.zeros_like(t)))
    gt[t >= 30.0, 1] += 8.0       # teleport at t = 30
    assert detect_jump(t, gt) == pytest.approx(30.0)
    err = np.where(t < 30.0, 0.05, np.where(t < 42.0, 8.0, 0.1))
    assert recovery_time(t, err, 30.0) == pytest.approx(12.0)
    assert recovery_time(t, err, 30.0, timeout=10.0) is None
    assert recovery_time(t, np.where(t < 30, 0.05, 8.0), 30.0) is None


def test_score_estimator_metrics():
    t = np.arange(0.0, 20.0, 0.02)
    gt = np.column_stack((t, np.zeros_like(t), np.zeros_like(t)))
    est_t = t[::5] + 0.001
    est = np.interp(est_t, t, gt[:, 0])
    est_poses = np.column_stack((est, np.full_like(est, 0.1), np.full_like(est, 0.02)))
    s = score_estimator(t, gt, est_t, est_poses, compute_ms=[1.0, 2.0, 3.0], waypoints=[(10.0, 0.0), (50.0, 0.0)])
    assert s.pos_rmse == pytest.approx(0.1, abs=1e-6)
    assert s.yaw_rmse == pytest.approx(0.02, abs=1e-9)
    assert s.waypoint_errors[0] == pytest.approx(0.1, abs=1e-6) and np.isnan(s.waypoint_errors[1])
    assert s.compute_ms_median == 2.0 and s.recovered is None


def test_trial_files_roundtrip(tmp_path):
    rows = [[0.1 * i, 'pf', i, 0.0, 0.0, 0.1, 0.1, 0.01] for i in range(5)]
    write_trajectory_csv(tmp_path / 'trajectory.csv', rows)
    data = read_trajectory_csv(tmp_path / 'trajectory.csv')
    assert data['pf'][1].shape == (5, 3) and data['pf'][1][4, 0] == 4.0
    write_summary(tmp_path / 'summary.json', {'k': 1.0}, {'pf': score_estimator([0, 1], [[0, 0, 0], [1, 0, 0]],
                                                                                [0.5], [[0.5, 0.0, 0.0]])})
    body = json.loads((tmp_path / 'summary.json').read_text())
    assert body['estimators']['pf']['pos_rmse'] == 0.0 and body['estimators']['pf']['recovery_time'] is None


def test_aggregation_matches_outline_numbers():
    # Outline: 20/20 recoveries bound the rate above 0.83, 0/20 below 0.17.
    assert clopper_pearson(20, 20)[0] == pytest.approx(0.832, abs=1e-3)
    assert clopper_pearson(0, 20)[1] == pytest.approx(0.168, abs=1e-3)
    mean, half, n = mean_ci95(np.random.default_rng(0).normal(0.0, 1.0, 20))
    assert n == 20 and 0.3 < half < 0.7


def test_study_grid_is_one_factor_at_a_time():
    from devol_localization.localization_study import study_configs
    runs = study_configs()
    nominal = [r for r in runs if r['scenario'] == 'nominal']
    assert len(nominal) == 9   # 4 N + 3 more k + 2 more sigma
    assert sum('ekf' in r['estimators'] for r in nominal) == 6
    assert {r['scenario'] for r in runs} == {'nominal', 'global', 'kidnap'}


def test_figure_renders_offscreen(tmp_path):
    pytest.importorskip('matplotlib')
    from devol_localization.viz_core import LocalizationFigure, VizState
    grid = np.zeros((200, 300), dtype=np.int8)
    grid[:3, :] = 100
    grid[:, :3] = 100
    grid[50:60, 100:110] = -1
    for mode in ('ekf', 'pf'):
        fig = LocalizationFigure(mode, window=8.0, interactive=False)
        fig.set_map(grid, 0.05, -1.0, -1.0)
        t = np.linspace(0, 10, 50)
        fig.update(VizState(stamp=10.0, truth=np.array([3.0, 3.0, 0.5]), estimate=np.array([3.1, 2.9, 0.45]),
                            covariance=np.diag([0.02, 0.01, 0.001]), scan_points=np.random.rand(100, 2) * 5,
                            particles=np.random.rand(500, 3) if mode == 'pf' else None,
                            truth_trail=np.random.rand(20, 2), estimate_trail=np.random.rand(20, 2),
                            err_t=t, pos_err=0.1 + 0 * t, pos_bound=0.2 + 0 * t, yaw_err=t, yaw_bound=2 + 0 * t,
                            compute_ms=3.2))
        fig.draw()
        fig.save(str(tmp_path / f'{mode}.png'))
        fig.close()
        assert (tmp_path / f'{mode}.png').stat().st_size > 10000


def test_test_case_verdicts():
    from devol_localization.metrics import EstimatorScore, judge_test_case
    good = {'ekf': EstimatorScore(waypoint_errors=[0.05, 0.1, 0.08], pos_rmse=0.07),
            'pf': EstimatorScore(waypoint_errors=[0.1, 0.2, 0.12], pos_rmse=0.13),
            'dead_reckoning': EstimatorScore(waypoint_errors=[0.5, 3.0, 6.0])}
    ok, lines = judge_test_case(1, good, ['Goal 1', 'Goal 2', 'Goal 3'])
    assert ok and len(lines) == 3
    bad = dict(good, pf=EstimatorScore(waypoint_errors=[0.1, float('nan'), 0.3], pos_rmse=0.2))
    ok, lines = judge_test_case(1, bad, ['Goal 1', 'Goal 2', 'Goal 3'])
    assert not ok and 'not reached' in lines[1]
    kid = {'pf': EstimatorScore(event_time=40.0, recovered=True, recovery_time=9.3),
           'ekf': EstimatorScore(event_time=40.0, recovered=False, final_pos_err=7.0)}
    ok, lines = judge_test_case(2, kid)
    assert ok and lines[-1].startswith('INFO ekf: did not recover')
    assert not judge_test_case(2, {'pf': EstimatorScore()})[0]
    # A run cut off by max_duration before the last goal fails even if every other check passes.
    for case, scores in ((1, good), (2, kid)):
        ok, lines = judge_test_case(case, scores, timed_out='max_duration 400 s reached')
        assert not ok and lines[0].startswith('FAIL route')


def test_video_falls_back_to_opencv(tmp_path, monkeypatch):
    pytest.importorskip('matplotlib')
    cv2 = pytest.importorskip('cv2')
    from matplotlib.animation import writers
    from devol_localization.viz_core import LocalizationFigure, VideoRecorder
    monkeypatch.setattr(writers, 'is_available', lambda name: False)
    fig = LocalizationFigure('pf', interactive=False)
    rec = VideoRecorder(fig.fig, str(tmp_path / 'v.mp4'), 5.0)
    assert rec.backend == 'opencv'
    for _ in range(3):
        rec.grab()
    rec.finish()
    fig.close()
    assert cv2.VideoCapture(str(tmp_path / 'v.mp4')).get(cv2.CAP_PROP_FRAME_COUNT) == 3


def test_replay_reindexes_a_bag_without_metadata(tmp_path):
    import importlib.util
    pytest.importorskip('launch_ros')   # the launch file imports launch + launch_ros
    spec = importlib.util.spec_from_file_location(
        'replay', Path(__file__).resolve().parents[1] / 'launch' / 'localization_replay.launch.py')
    replay = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(replay)
    (tmp_path / 'bag_0.mcap').write_bytes(b'')
    calls = []

    def reindex(d):
        calls.append(d)
        (Path(d) / 'metadata.yaml').write_text('rosbag2_bagfile_information:\n  message_count: 12\n')
    replay.check_bag(str(tmp_path), reindex=reindex)
    assert calls == [str(tmp_path)]
    with pytest.raises(RuntimeError):
        replay.check_bag(str(tmp_path / 'missing'), reindex=reindex)
