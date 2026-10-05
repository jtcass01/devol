"""Offline tests of the hybrid EKF + particle filter (devol_localization.hybrid).

Reuses the particle filter tests' synthetic room. No ROS needed:
`python3 -m pytest test/test_hybrid.py`.
"""

import numpy as np
import pytest

from devol_localization.ekf_core import PoseEKF
from devol_localization.ekf_pipeline import EKFPipeline
from devol_localization.hybrid import HybridLocalizer, HybridParams, KidnapMonitor, floor_covariance
from devol_localization.particle_filter import LikelihoodField, ParticleFilter, PFParams, odometry_delta
from devol_localization.scan_matcher import DistanceField, ScanMatcher, scan_to_points

from test_particle_filter import RANGE_MAX, RES, make_map, noisy_odometry, raycast, trajectory

LASER = (0.32825, 0.0, 0.0)
CONFIDENT = np.diag([0.05, 0.05, 0.02]) ** 2


def test_floor_covariance_only_raises():
    P = floor_covariance(np.diag([0.01, 1.0, 1e-6]), 0.15, 0.08)
    assert np.allclose(np.diag(P), [0.15 ** 2, 1.0, 0.08 ** 2])


def test_monitor_ignores_agreement():
    m = KidnapMonitor(HybridParams(confirm_updates=1))
    x = np.array([1.0, 2.0, 0.3])
    assert not m.check(x, CONFIDENT, x + [0.1, -0.1, 0.02], CONFIDENT, loglik_gain=1.0)


def test_monitor_needs_a_streak_then_cools_down():
    m = KidnapMonitor(HybridParams(confirm_updates=3, cooldown_updates=2))
    ekf_x, pf_x = np.zeros(3), np.array([2.0, 1.0, 0.0])
    assert [m.check(ekf_x, CONFIDENT, pf_x, CONFIDENT, 1.0) for _ in range(3)] == [False, False, True]
    assert not m.check(ekf_x, CONFIDENT, pf_x, CONFIDENT, 1.0)   # cooling down
    assert not m.check(ekf_x, CONFIDENT, pf_x, CONFIDENT, 1.0)
    # One agreeing update in the middle restarts the streak.
    m = KidnapMonitor(HybridParams(confirm_updates=3))
    m.check(ekf_x, CONFIDENT, pf_x, CONFIDENT, 1.0)
    m.check(ekf_x, CONFIDENT, ekf_x, CONFIDENT, 1.0)
    assert [m.check(ekf_x, CONFIDENT, pf_x, CONFIDENT, 1.0) for _ in range(3)] == [False, False, True]


def test_monitor_vetoes_when_scan_prefers_the_ekf_or_pf_is_spread():
    m = KidnapMonitor(HybridParams(confirm_updates=1))
    ekf_x, pf_x = np.zeros(3), np.array([2.0, 1.0, 0.0])
    assert not m.check(ekf_x, CONFIDENT, pf_x, CONFIDENT, loglik_gain=-0.5)
    assert m.stats.vetoed_by_scan == 1
    spread = np.diag([1.0, 1.0, 0.5]) ** 2
    assert not m.check(ekf_x, CONFIDENT, pf_x, spread, loglik_gain=1.0)
    # The PF beats the EKF but does not fit the scan well itself: still searching, so wait.
    assert not m.check(ekf_x, CONFIDENT, pf_x, CONFIDENT, loglik_gain=1.0, pf_loglik=-0.4)
    assert m.stats.vetoed_by_fit == 1
    assert m.check(ekf_x, CONFIDENT, pf_x, CONFIDENT, loglik_gain=1.0, pf_loglik=0.15)


def run(kidnap_step=None, jump=np.zeros(3), hybrid=True, seed=3):
    """EKF (optionally with the hybrid) and PF over the synthetic route. kidnap_step moves both
    filters' beliefs by jump, which to the filters is a teleport the odometry never saw."""
    grid = make_map()
    rng = np.random.default_rng(seed)
    truth = trajectory()
    odom = noisy_odometry(truth, rng, frac=0.05)
    pipe = EKFPipeline(PoseEKF(), ScanMatcher(DistanceField(grid, RES, (0.0, 0.0, 0.0))))
    pipe.ekf.reset(truth[0], np.diag([0.1, 0.1, 0.05]) ** 2)
    pf = ParticleFilter(PFParams(num_particles=1000), LikelihoodField(grid, RES, 0.0, 0.0), seed=seed)
    pf.init_gaussian(truth[0], 0.2, 0.1)
    hyb = HybridLocalizer(pipe, pf)
    pipe.on_odom(odom[0])
    errs = []
    for k in range(1, len(truth)):
        pipe.on_odom(odom[k])
        pf.predict(*odometry_delta(odom[k - 1], odom[k]))
        if k == kidnap_step:
            pipe.ekf.x = pipe.ekf.x + jump
            pf.particles += jump
        ranges, angles = raycast(grid, truth[k])
        ranges = ranges + rng.normal(0, 0.01, ranges.size)
        pipe.on_scan(scan_to_points(ranges, angles[0], angles[1] - angles[0], 0.05, RANGE_MAX, 4, LASER))
        pf.update(ranges, angles, 0.05, RANGE_MAX, LASER)
        pf.resample_if_needed()
        if hybrid:
            hyb.after_pf_update(ranges, angles, 0.05, RANGE_MAX, LASER)
        errs.append(np.hypot(*(pipe.ekf.x[:2] - truth[k][:2])))
    return np.array(errs), hyb.stats


def test_hybrid_does_not_reseed_while_tracking():
    errs, stats = run()
    assert stats.reseeds == 0
    assert np.max(errs) < 0.25


@pytest.mark.parametrize('jump', [np.array([3.0, 3.0, 0.0]), np.array([-2.5, 2.0, 0.8])])
def test_hybrid_reseeds_ekf_after_kidnap(jump):
    alone, _ = run(kidnap_step=30, jump=jump, hybrid=False)
    errs, stats = run(kidnap_step=30, jump=jump)
    assert alone[-20:].min() > 1.0       # beyond the scan matcher's window, the EKF stays lost
    assert stats.reseeds >= 1
    assert np.all(errs[-20:] < 0.25)
