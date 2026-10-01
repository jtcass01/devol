"""Offline tests of the particle filter math on a synthetic map.

A toy occupancy grid stands in for /devol_drive/projected_map; scans are
ray-cast against it from a ground-truth trajectory and odometry is the true
motion plus noise. No ROS needed: `python3 -m pytest test/test_particle_filter.py`.
"""

import numpy as np
import pytest

from devol_localization.particle_filter import (
    LikelihoodField, ParticleFilter, PFParams, odometry_delta, wrap_angle)

RES = 0.05
BEAMS = 540
FOV = 2.356
RANGE_MAX = 12.0


def make_map():
    """20 m x 12 m room with asymmetric obstacles, ROS OccupancyGrid layout (row = y)."""
    h, w = int(12 / RES), int(20 / RES)
    g = np.full((h, w), -1, dtype=np.int8)   # unknown, like an octomap projection

    def box(x0, y0, x1, y1):
        g[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = 100

    box(0, 0, 20, 0.2)
    box(0, 11.8, 20, 12)
    box(0, 0, 0.2, 12)
    box(19.8, 0, 20, 12)
    box(4, 3, 5, 7)
    box(9, 8, 13, 9)
    box(14, 2, 15.5, 3.5)
    box(7, 1.5, 7.5, 3)
    box(16, 7, 16.5, 10)
    return g


def raycast(grid, pose, laser=(0.32825, 0.0, 0.0), step=RES / 2):
    angles = np.linspace(-FOV, FOV, BEAMS)
    c, s = np.cos(pose[2]), np.sin(pose[2])
    sx = pose[0] + c * laser[0] - s * laser[1]
    sy = pose[1] + s * laser[0] + c * laser[1]
    th = pose[2] + laser[2] + angles
    d = np.arange(step, RANGE_MAX, step)
    px = sx + np.cos(th)[:, None] * d[None, :]
    py = sy + np.sin(th)[:, None] * d[None, :]
    col = np.clip((px / RES).astype(int), 0, grid.shape[1] - 1)
    row = np.clip((py / RES).astype(int), 0, grid.shape[0] - 1)
    hit = grid[row, col] >= 50
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    ranges = np.where(first >= 0, d[np.maximum(first, 0)], np.inf)
    return ranges, angles


def trajectory():
    """Ground-truth loop through the free space: drive, turn, drive (10 cm steps)."""
    poses = [np.array([2.0, 2.0, 0.0])]
    legs = [(4.0, 0.0), (0.0, np.pi / 2), (5.0, 0.0), (0.0, -np.pi / 2), (6.0, 0.0),
            (0.0, -np.pi / 2), (4.0, 0.0)]
    for dist, turn in legs:
        steps = max(1, int(round(abs(dist) / 0.1))) if dist else 10
        for _ in range(steps):
            x, y, th = poses[-1]
            if dist:
                ds = dist / steps
                poses.append(np.array([x + ds * np.cos(th), y + ds * np.sin(th), th]))
            else:
                poses.append(np.array([x, y, wrap_angle(th + turn / steps)]))
    return poses


def noisy_odometry(truth, rng, frac=0.05):
    """Dead-reckoned odometry: true increments with proportional noise, integrated."""
    odom = [truth[0].copy()]
    for a, b in zip(truth[:-1], truth[1:]):
        r1, t, r2 = odometry_delta(a, b)
        r1 += rng.normal(0, frac * abs(r1) + 0.002)
        t += rng.normal(0, frac * abs(t))
        r2 += rng.normal(0, frac * abs(r2) + 0.002)
        x, y, th = odom[-1]
        h = th + r1
        odom.append(np.array([x + t * np.cos(h), y + t * np.sin(h), wrap_angle(h + r2)]))
    return odom


def run(pf, grid, truth, odom, range_sigma=0.01, rng=None, kidnap_at=None):
    """Filter the whole trajectory; return per-step position and yaw errors.

    kidnap_at=(step, offset) shifts every particle by offset at that step. To the
    filter that is the same as the robot being teleported by -offset without
    odometry seeing it, and it keeps the robot on its collision-free route.
    """
    rng = rng or np.random.default_rng(0)
    errs, yaw_errs = [], []
    for k in range(1, len(truth)):
        pf.predict(*odometry_delta(odom[k - 1], odom[k]))
        if kidnap_at is not None and k == kidnap_at[0]:
            pf.particles += kidnap_at[1]
        pose = truth[k]
        ranges, angles = raycast(grid, pose)
        ranges = ranges + rng.normal(0, range_sigma, ranges.size)
        pf.update(ranges, angles, 0.05, RANGE_MAX, (0.32825, 0.0, 0.0))
        pf.resample_if_needed()
        est, _ = pf.estimate()
        errs.append(np.hypot(*(est[:2] - pose[:2])))
        yaw_errs.append(abs(wrap_angle(est[2] - pose[2])))
    return np.array(errs), np.array(yaw_errs)


@pytest.fixture(scope='module')
def world():
    grid = make_map()
    field = LikelihoodField(grid, RES, 0.0, 0.0)
    return grid, field


def test_odometry_delta_roundtrip():
    rng = np.random.default_rng(1)
    for _ in range(200):
        a = rng.uniform([-5, -5, -np.pi], [5, 5, np.pi])
        b = a + rng.uniform([-1, -1, -np.pi], [1, 1, np.pi])
        r1, t, r2 = odometry_delta(a, b)
        h = a[2] + r1
        c = np.array([a[0] + t * np.cos(h), a[1] + t * np.sin(h), wrap_angle(h + r2)])
        assert np.allclose(c[:2], b[:2], atol=1e-9)
        assert abs(wrap_angle(c[2] - b[2])) < 1e-9


def test_reverse_motion_has_small_rot1():
    r1, t, r2 = odometry_delta((0, 0, 0), (-0.1, 0, 0))
    assert t < 0 and abs(r1) < 1e-9 and abs(r2) < 1e-9


def test_likelihood_field_distances(world):
    _, field = world
    # Inside the 4..5 x 3..7 box, distance is 0; 1 m left of it, ~1 m.
    assert field.distance(np.array([4.5]), np.array([5.0]))[0] == 0.0
    assert abs(field.distance(np.array([3.0]), np.array([5.0]))[0] - 1.0) < 0.06
    # Outside the map, the cap.
    assert field.distance(np.array([-3.0]), np.array([5.0]))[0] == field.max_dist


def test_low_variance_resampling_keeps_heavy_particle():
    pf = ParticleFilter(PFParams(num_particles=100), seed=0)
    pf.particles[:, 0] = np.arange(100)
    pf.weights = np.full(100, 1e-6)
    pf.weights[42] = 1.0
    pf.weights /= pf.weights.sum()
    pf.resample()
    assert np.all(pf.particles[:, 0] == 42)
    assert np.allclose(pf.weights, 0.01)


def test_tracking_beats_dead_reckoning(world):
    grid, field = world
    rng = np.random.default_rng(3)
    truth = trajectory()
    odom = noisy_odometry(truth, rng, frac=0.08)
    pf = ParticleFilter(PFParams(num_particles=500), field, seed=3)
    pf.init_gaussian(truth[0], 0.2, 0.1)
    errs, yaw_errs = run(pf, grid, truth, odom, rng=rng)

    dr_err = np.array([np.hypot(*(o[:2] - t[:2])) for o, t in zip(odom[1:], truth[1:])])
    pf_rmse = np.sqrt(np.mean(errs ** 2))
    dr_rmse = np.sqrt(np.mean(dr_err ** 2))
    print(f'PF RMSE {pf_rmse:.3f} m, yaw {np.degrees(np.sqrt(np.mean(yaw_errs**2))):.2f} deg; '
          f'dead reckoning RMSE {dr_rmse:.3f} m (final {dr_err[-1]:.3f} m)')
    assert pf_rmse < 0.10
    assert errs[-1] < 0.10
    assert np.sqrt(np.mean(yaw_errs ** 2)) < np.radians(3)
    assert pf_rmse < dr_rmse


def test_global_localization_converges(world):
    """Uniform prior, plain SIR. Success depends on a particle landing near the
    true pose, so it is a rate, not a guarantee: on this map 20000 particles
    converge on most seeds and 5000 on about half. Require 2 of 3 seeds."""
    grid, field = world
    truth = trajectory()
    successes = 0
    for seed in range(3):
        rng = np.random.default_rng(seed)
        odom = noisy_odometry(truth, rng, frac=0.05)
        pf = ParticleFilter(PFParams(num_particles=20000), field, seed=seed)
        pf.init_uniform()
        errs, _ = run(pf, grid, truth, odom, rng=rng)
        converged = np.flatnonzero(errs < 0.25)
        print(f'global localization seed {seed}: first < 0.25 m at step '
              f'{converged[0] if converged.size else None}')
        successes += bool(np.all(errs[-20:] < 0.25))
    assert successes >= 2


def test_kidnapping_recovery_with_injection(world):
    grid, field = world
    rng = np.random.default_rng(7)
    truth = trajectory()
    odom = noisy_odometry(truth, rng, frac=0.05)
    kidnap_step = 30
    jump = np.array([3.0, 3.0, 0.0])   # filter confidently ~4 m off after the kidnap

    params = PFParams(num_particles=2000, alpha_slow=0.001, alpha_fast=0.1)
    pf = ParticleFilter(params, field, seed=7)
    pf.init_gaussian(truth[0], 0.2, 0.1)
    errs, _ = run(pf, grid, truth, odom, rng=rng, kidnap_at=(kidnap_step, jump))
    after = errs[kidnap_step - 1:]
    recovered = np.flatnonzero(after < 0.25)
    print(f'kidnap: recovered after {recovered[0] if recovered.size else None} steps')
    assert after[0] > 1.0          # the jump really did break the estimate
    assert np.all(after[-20:] < 0.25)


def test_plain_sir_does_not_recover_from_kidnapping(world):
    """Documents why alpha_slow/alpha_fast exist: without injection the
    particle set has nothing near the new pose and stays lost."""
    grid, field = world
    rng = np.random.default_rng(7)
    truth = trajectory()
    odom = noisy_odometry(truth, rng, frac=0.05)
    pf = ParticleFilter(PFParams(num_particles=2000), field, seed=7)
    pf.init_gaussian(truth[0], 0.2, 0.1)
    errs, _ = run(pf, grid, truth, odom, rng=rng, kidnap_at=(30, np.array([3.0, 3.0, 0.0])))
    assert np.all(errs[-20:] > 0.25)
