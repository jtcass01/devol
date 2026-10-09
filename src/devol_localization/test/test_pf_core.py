"""Step tests for the particle filter core (devol_localization.particle_filter).

Each test checks one step of Monte Carlo localization (PR Tables 4.3, 4.4, 5.6, 6.3, 8.2 and
8.3) against its equation, as listed in THEORY.md, rather than running the filter on a map. The
end-to-end runs are in test_particle_filter.py. No ROS needed:
`python3 -m pytest test/test_pf_core.py`.
"""

import numpy as np
import pytest

from devol_localization.particle_filter import (
    LikelihoodField,
    ParticleFilter,
    PFParams,
    wrap_angle,
)

RES = 0.05
RANGE_MAX = 10.0


def wall_field():
    """4 m x 2 m map, free except for one wall: the cells with 2.00 <= x < 2.05.

    On the cell grid, a scan endpoint in column c is exactly |c - 40| * RES from the wall, so
    the likelihoods below can be written down by hand.
    """
    g = np.zeros((int(2.0 / RES), int(4.0 / RES)), dtype=np.int8)
    g[:, 40] = 100
    return LikelihoodField(g, RES, 0.0, 0.0, max_dist=2.0)


def beam_likelihood(d, p: PFParams):
    """One beam of the likelihood field model (PR Table 6.3): z_hit N(d; 0, sigma) + z_rand/z_max."""
    gauss = np.exp(-0.5 * (d / p.sigma_hit) ** 2) / (np.sqrt(2.0 * np.pi) * p.sigma_hit)
    return p.z_hit * gauss + p.z_rand / RANGE_MAX


def pf_at(poses, params=None, field=None, seed=0):
    poses = np.atleast_2d(np.asarray(poses, dtype=float))
    params = params or PFParams(num_particles=len(poses))
    pf = ParticleFilter(params, field, seed=seed)
    pf.particles = poses.copy()
    pf.weights = np.full(len(poses), 1.0 / len(poses))
    pf.initialized = True
    return pf


# ------------------------------------------------------------ likelihood field


def test_likelihood_field_cells_distances_and_free_space():
    f = wall_field()
    # floor() puts negative coordinates in column -1, which is outside the map.
    assert f.world_to_cell(-0.01, 0.0) == (0, -1)
    assert f.world_to_cell(1.0, 0.5) == (10, 20)
    # Cell-centre distances to the wall at column 40, capped at max_dist; max_dist off the map.
    assert f.distance(np.array([1.025, 2.025, 3.525, 0.025, -1.0]), np.full(5, 0.5)) == (
        pytest.approx([1.0, 0.0, 1.5, 2.0, 2.0])
    )
    assert f.is_free(np.array([1.0, 2.02, -1.0]), np.full(3, 0.5)).tolist() == [
        True,
        False,
        False,
    ]
    assert len(f.free_cells) == f.grid.size - f.grid.shape[0]
    # No obstacles at all: every distance is the cap.
    empty = LikelihoodField(np.zeros((10, 10), dtype=np.int8), RES, 0.0, 0.0, max_dist=1.5)
    assert np.all(empty.dist == 1.5)


# ------------------------------------------------------------------ initialization


def test_init_gaussian_statistics():
    pf = ParticleFilter(PFParams(num_particles=20000), seed=1)
    pf.init_gaussian([1.0, -2.0, 3.0], std_xy=0.2, std_yaw=0.1)
    assert pf.initialized
    assert pf.particles[:, :2].mean(axis=0) == pytest.approx([1.0, -2.0], abs=0.01)
    assert pf.particles[:, :2].std(axis=0) == pytest.approx([0.2, 0.2], rel=0.03)
    # Yaw is wrapped, so measure it around the mean on the circle.
    dyaw = wrap_angle(pf.particles[:, 2] - 3.0)
    assert np.all((pf.particles[:, 2] >= -np.pi) & (pf.particles[:, 2] < np.pi))
    assert abs(dyaw.mean()) < 0.005 and dyaw.std() == pytest.approx(0.1, rel=0.03)
    assert np.allclose(pf.weights, 1.0 / 20000)


def test_init_uniform_samples_only_free_space():
    f = wall_field()
    pf = ParticleFilter(PFParams(num_particles=5000), f, seed=2)
    pf.init_uniform()
    assert f.is_free(pf.particles[:, 0], pf.particles[:, 1]).all()
    # Both sides of the wall are covered, and the heading is uniform.
    assert (pf.particles[:, 0] < 2.0).any() and (pf.particles[:, 0] > 2.05).any()
    assert abs(pf.particles[:, 2].mean()) < 0.1
    assert pf.particles[:, 2].std() == pytest.approx(2 * np.pi / np.sqrt(12), rel=0.03)
    with pytest.raises(RuntimeError):
        ParticleFilter(PFParams(num_particles=10)).init_uniform()


# ---------------------------------------------------------------------- prediction


def test_predict_without_noise_applies_the_motion_model():
    zero = PFParams(num_particles=3, alpha1=0.0, alpha2=0.0, alpha3=0.0, alpha4=0.0)
    start = np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 3.0], [-1.0, 0.5, -2.0]])
    pf = pf_at(start, zero)
    rot1, trans, rot2 = 0.2, 0.5, -0.1
    pf.predict(rot1, trans, rot2)
    h = start[:, 2] + rot1
    assert pf.particles[:, 0] == pytest.approx(start[:, 0] + trans * np.cos(h))
    assert pf.particles[:, 1] == pytest.approx(start[:, 1] + trans * np.sin(h))
    assert pf.particles[:, 2] == pytest.approx(wrap_angle(h + rot2))


def test_predict_samples_the_std_form_motion_noise():
    """Recover each particle's sampled (rot1, trans, rot2) and compare their spread with the
    std model of PFParams: rot std = a1 |rot| + a2 |trans|, trans std = a3 |trans| +
    a4 (|rot1| + |rot2|)."""
    # Alphas small enough that no sample flips to a negative translation, so the sampled
    # decomposition can be read back from the particle poses.
    p = PFParams(num_particles=100000, alpha1=0.1, alpha2=0.05, alpha3=0.05, alpha4=0.05)
    pf = pf_at(np.zeros((p.num_particles, 3)), p, seed=3)
    rot1, trans, rot2 = 0.2, 1.0, -0.1
    pf.predict(rot1, trans, rot2)
    x, y, yaw = pf.particles.T
    r1 = np.arctan2(y, x)
    t = np.hypot(x, y)
    r2 = wrap_angle(yaw - r1)
    assert np.all(t > 0) and np.all(np.abs(r1) < np.pi / 2)  # no reverse fold to undo
    expected_std = [
        p.alpha1 * abs(rot1) + p.alpha2 * trans,
        p.alpha3 * trans + p.alpha4 * (abs(rot1) + abs(rot2)),
        p.alpha1 * abs(rot2) + p.alpha2 * trans,
    ]
    for sample, mean, std in zip((r1, t, r2), (rot1, trans, rot2), expected_std):
        assert sample.mean() == pytest.approx(mean, abs=3 * std / np.sqrt(p.num_particles) + 1e-3)
        assert sample.std() == pytest.approx(std, rel=0.02)


def test_predict_below_min_translation_uses_rotation_noise_only():
    """Under min_translation the noise is computed as for (0, trans, rot1 + rot2); the mean
    motion stays as given."""
    p = PFParams(num_particles=50000, min_translation=0.01)
    pf = pf_at(np.zeros((p.num_particles, 3)), p, seed=4)
    rot1, trans, rot2 = 1.5, 0.005, -1.4
    pf.predict(rot1, trans, rot2)
    yaw = pf.particles[:, 2]
    # Total yaw noise: rot1 std a2 * t plus rot2 std a1 |rot1 + rot2| + a2 * t, independent.
    std_yaw = np.hypot(p.alpha2 * trans, p.alpha1 * abs(rot1 + rot2) + p.alpha2 * trans)
    assert yaw.mean() == pytest.approx(rot1 + rot2, abs=0.002)
    assert yaw.std() == pytest.approx(std_yaw, rel=0.03)


# ------------------------------------------------------------------- correction


def test_update_weights_are_the_likelihood_field_product():
    """Two beams straight ahead (range 1 m) from particles whose endpoints are 1 m, 0 m and
    1.5 m from the wall: each weight is the product of the two beam likelihoods, normalized
    (PR Table 6.3), and the returned value is N_eff."""
    f = wall_field()
    p = PFParams(num_particles=3)
    pf = pf_at([[0.025, 1.0, 0.0], [1.025, 1.0, 0.0], [2.525, 1.0, 0.0]], p, f)
    ranges, angles = np.array([1.0, 1.0]), np.array([0.0, 0.0])
    n_eff = pf.update(ranges, angles, 0.05, RANGE_MAX)
    lik = beam_likelihood(np.array([1.0, 0.0, 1.5]), p) ** 2
    assert pf.weights == pytest.approx(lik / lik.sum())
    assert n_eff == pytest.approx(1.0 / np.sum((lik / lik.sum()) ** 2))


def test_update_projects_beams_through_the_laser_mount():
    """A laser 0.5 m ahead of the base, turned 90 deg, with a beam at -90 deg in its own frame,
    points along the robot's x axis from x + 0.5."""
    f = wall_field()
    p = PFParams(num_particles=2)
    pf = pf_at([[0.525, 1.0, 0.0], [1.025, 1.0, 0.0]], p, f)
    pf.update(np.array([1.0]), np.array([-np.pi / 2]), 0.05, RANGE_MAX, (0.5, 0.0, np.pi / 2))
    # Endpoints at x = 2.025 (on the wall) and 2.525 (0.5 m past it).
    lik = beam_likelihood(np.array([0.0, 0.5]), p)
    assert pf.weights == pytest.approx(lik / lik.sum())


def test_update_multiplies_the_previous_weights():
    """Sequential importance sampling: w_t is proportional to w_{t-1} p(z | x) (PR Table 4.3)."""
    f = wall_field()
    p = PFParams(num_particles=2)
    pf = pf_at([[0.025, 1.0, 0.0], [1.025, 1.0, 0.0]], p, f)
    prior = np.array([0.9, 0.1])
    pf.weights = prior.copy()
    pf.update(np.array([1.0]), np.array([0.0]), 0.05, RANGE_MAX)
    post = prior * beam_likelihood(np.array([1.0, 0.0]), p)
    assert pf.weights == pytest.approx(post / post.sum())


def test_update_skips_invalid_beams_and_subsamples_to_max_beams():
    f = wall_field()
    p = PFParams(num_particles=2, max_beams=2)
    pf = pf_at([[0.025, 1.0, 0.0], [1.025, 1.0, 0.0]], p, f)
    pf.weights = np.array([0.3, 0.7])
    # inf, below range_min and at range_max carry no endpoint: weights unchanged.
    pf.update(np.array([np.inf, 0.01, RANGE_MAX]), np.zeros(3), 0.05, RANGE_MAX)
    assert pf.weights == pytest.approx([0.3, 0.7])
    # Five valid beams with max_beams = 2 use the first and the last (1 m and 2.5 m ahead).
    pf.weights = np.array([0.5, 0.5])
    pf.update(np.array([1.0, 9.0, 9.0, 9.0, 2.5]), np.zeros(5), 0.05, RANGE_MAX)
    lik = beam_likelihood(np.array([1.0, 0.0]), p) * beam_likelihood(np.array([0.5, 1.5]), p)
    assert pf.weights == pytest.approx(lik / lik.sum())


def test_update_tracks_w_slow_and_w_fast_and_injects_after_a_drop():
    """Augmented MCL (PR Table 8.3): w_avg is the mean per-beam geometric mean likelihood,
    w_slow and w_fast are its exponential averages, and p_inject = max(0, 1 - w_fast/w_slow)."""
    f = wall_field()
    p = PFParams(num_particles=2)
    on_wall = [[1.025, 1.0, 0.0], [1.025, 1.0, 0.0]]
    pf = pf_at(on_wall, p, f)
    assert pf.injection_probability() == 0.0  # nothing seen yet
    beam = (np.array([1.0, 1.0]), np.zeros(2), 0.05, RANGE_MAX)

    good = beam_likelihood(0.0, p)
    pf.update(*beam)
    assert pf._w_slow == pytest.approx(good) and pf._w_fast == pytest.approx(good)
    assert pf.injection_probability() == 0.0

    # The robot is no longer where the particles think: every endpoint is 1 m off.
    pf.particles[:, 0] = 0.025
    bad = beam_likelihood(1.0, p)
    pf.update(*beam)
    w_slow = good + p.alpha_slow * (bad - good)
    w_fast = good + p.alpha_fast * (bad - good)
    assert pf._w_slow == pytest.approx(w_slow) and pf._w_fast == pytest.approx(w_fast)
    assert pf.injection_probability() == pytest.approx(1.0 - w_fast / w_slow)
    assert pf.injection_probability() > 0.01
    assert pf.resample_if_needed()  # injection forces a resampling even at full N_eff

    plain = pf_at(on_wall, PFParams(num_particles=2, alpha_slow=0.0, alpha_fast=0.0), f)
    plain.update(*beam)
    plain.particles[:, 0] = 0.025
    plain.update(*beam)
    assert plain.injection_probability() == 0.0  # plain SIR never injects


# ------------------------------------------------------------------- resampling


def test_effective_sample_size():
    pf = pf_at(np.zeros((10, 3)))
    assert pf.effective_sample_size() == pytest.approx(10.0)
    pf.weights = np.r_[1.0, np.zeros(9)]
    assert pf.effective_sample_size() == pytest.approx(1.0)
    pf.weights = np.r_[0.5, 0.5, np.zeros(8)]
    assert pf.effective_sample_size() == pytest.approx(2.0)


def test_resample_if_needed_follows_the_n_eff_threshold():
    pf = pf_at(np.zeros((10, 3)))
    pf.weights = np.r_[np.full(6, 1 / 6), np.zeros(4)]  # N_eff = 6 >= 0.5 N
    assert not pf.resample_if_needed()
    pf.weights = np.r_[np.full(4, 1 / 4), np.zeros(6)]  # N_eff = 4 < 0.5 N
    assert pf.resample_if_needed()
    assert np.allclose(pf.weights, 0.1)


def test_low_variance_resampling_copies_each_particle_floor_or_ceil_of_n_w():
    """Systematic resampling (PR Table 4.4) is O(N) and low variance: particle i is copied
    either floor(N w_i) or ceil(N w_i) times, and equal weights reproduce the set unchanged."""
    n = 200
    for seed in range(20):
        rng = np.random.default_rng(seed)
        pf = pf_at(np.column_stack([np.arange(n), np.zeros(n), np.zeros(n)]), seed=seed)
        w = rng.exponential(1.0, n) ** 3
        pf.weights = w / w.sum()
        expected = n * pf.weights
        pf.resample()
        counts = np.bincount(pf.particles[:, 0].astype(int), minlength=n)
        assert counts.sum() == n
        assert np.all(counts >= np.floor(expected) - 1e-9)
        assert np.all(counts <= np.ceil(expected) + 1e-9)
        assert np.allclose(pf.weights, 1.0 / n)

    pf = pf_at(np.column_stack([np.arange(n), np.zeros(n), np.zeros(n)]), seed=0)
    before = pf.particles.copy()
    pf.resample()
    assert np.array_equal(pf.particles, before)


def test_resample_injects_the_requested_fraction_of_uniform_particles():
    f = wall_field()
    n = 4000
    marker = np.array([-50.0, -50.0, 0.0])  # off the map, so any free particle was injected
    pf = pf_at(np.tile(marker, (n, 1)), PFParams(num_particles=n), f, seed=5)
    pf.resample(p_inject=0.25)
    injected = pf.particles[:, 0] > -50.0
    assert injected.mean() == pytest.approx(0.25, abs=3 * np.sqrt(0.25 * 0.75 / n))
    assert f.is_free(pf.particles[injected, 0], pf.particles[injected, 1]).all()
    pf = pf_at(np.tile(marker, (n, 1)), PFParams(num_particles=n), f, seed=5)
    pf.resample(p_inject=1.0)
    assert f.is_free(pf.particles[:, 0], pf.particles[:, 1]).all()


# --------------------------------------------------------------------- estimate


def test_estimate_weighted_mean_and_covariance():
    pf = pf_at([[0.0, 0.0, 0.1], [2.0, 4.0, 0.3]])
    pf.weights = np.array([0.75, 0.25])
    mean, cov = pf.estimate()
    yaw = np.arctan2(
        0.75 * np.sin(0.1) + 0.25 * np.sin(0.3), 0.75 * np.cos(0.1) + 0.25 * np.cos(0.3)
    )
    assert mean == pytest.approx([0.5, 1.0, yaw])
    dev = np.array([[-0.5, -1.0, 0.1 - yaw], [1.5, 3.0, 0.3 - yaw]])
    assert cov == pytest.approx((dev * pf.weights[:, None]).T @ dev)


def test_estimate_yaw_is_a_circular_mean_across_pi():
    """Headings just either side of +-pi average to pi, not to 0 (Mardia and Jupp, Chapter 2)."""
    pf = pf_at([[0.0, 0.0, np.pi - 0.1], [0.0, 0.0, -np.pi + 0.1]])
    mean, cov = pf.estimate()
    assert abs(wrap_angle(mean[2] - np.pi)) < 1e-9
    assert cov[2, 2] == pytest.approx(0.01)
