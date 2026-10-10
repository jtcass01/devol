"""Step tests for the EKF core (devol_localization.ekf_core).

Each test checks one step of PR Table 3.3 (and the odometry model of PR Section 5.4) against its
equation, as listed in THEORY.md, rather than running a filter on a map. The end-to-end runs are
in test_ekf_offline.py. No ROS needed: `python3 -m pytest test/test_ekf_core.py`.
"""

import numpy as np
import pytest

from devol_localization import noise_model, particle_filter, pose2d
from devol_localization.ekf_core import (
    CHI2_3DOF_99,
    PoseEKF,
    global_initial_state,
    odometry_delta,
    wrap_angle,
)


def g(x, u):
    """The noise-free odometry motion model g(u, x) of PR Section 5.4, written out here."""
    rot1, trans, rot2 = u
    h = x[2] + rot1
    return np.array([x[0] + trans * np.cos(h), x[1] + trans * np.sin(h), x[2] + rot1 + rot2])


def numeric_jacobian(f, v, eps=1e-6):
    v = np.asarray(v, dtype=float)
    cols = []
    for i in range(v.size):
        d = np.zeros_like(v)
        d[i] = eps
        cols.append((f(v + d) - f(v - d)) / (2.0 * eps))
    return np.column_stack(cols)


def control_cov(alphas, u):
    """M of PoseEKF.predict: variances linear in the increment, a2 * t split over rot1 and rot2."""
    a1, a2, a3, a4 = alphas
    r1, t, r2 = (abs(c) for c in u)
    return np.diag([a1 * r1 + 0.5 * a2 * t, a3 * t + a4 * (r1 + r2), a1 * r2 + 0.5 * a2 * t])


def random_spd(rng, scale=0.1):
    A = rng.normal(0.0, scale, (3, 3))
    return A @ A.T + np.eye(3) * scale**2


def is_spd(P):
    return np.allclose(P, P.T, atol=1e-12) and np.linalg.eigvalsh(P).min() > 0.0


# ------------------------------------------------------------------ shared models


def test_wrap_angle_range_and_agreement_across_modules():
    rng = np.random.default_rng(0)
    for a in np.concatenate([rng.uniform(-20.0, 20.0, 500), [-np.pi, np.pi, 0.0, 3 * np.pi]]):
        w = wrap_angle(a)
        assert -np.pi <= w < np.pi
        assert abs(np.sin(w) - np.sin(a)) < 1e-9 and abs(np.cos(w) - np.cos(a)) < 1e-9
        assert w == pytest.approx(float(particle_filter.wrap_angle(a)), abs=1e-12)
        assert w == pytest.approx(float(pose2d.wrap_angle(a)), abs=1e-12)


def test_odometry_delta_agrees_across_modules():
    """The EKF, the PF and the noise injector each have their own copy of PR Table 5.6 lines
    2-4; they must decompose the same motion the same way, reverse motion included."""
    rng = np.random.default_rng(1)
    for _ in range(500):
        a = rng.uniform([-5.0, -5.0, -np.pi], [5.0, 5.0, np.pi])
        b = a + rng.uniform([-1.0, -1.0, -np.pi], [1.0, 1.0, np.pi])
        ekf = np.array(odometry_delta(a, b))
        for other in (particle_filter.odometry_delta(a, b), noise_model.odometry_delta(a, b)):
            other = np.array(other)
            assert ekf[1] == pytest.approx(other[1], abs=1e-9)
            assert abs(wrap_angle(ekf[0] - other[0])) < 1e-9
            assert abs(wrap_angle(ekf[2] - other[2])) < 1e-9


def test_odometry_delta_folds_reverse_and_pure_rotation():
    # Reverse: a negative translation with |rot1| <= pi/2, never two half turns.
    r1, t, r2 = odometry_delta([0.0, 0.0, 0.5], [-0.3 * np.cos(0.6), -0.3 * np.sin(0.6), 0.7])
    assert t == pytest.approx(-0.3) and r1 == pytest.approx(0.1) and r2 == pytest.approx(0.1)
    # In place: all of the turn is rot2.
    assert odometry_delta([1.0, 2.0, 3.0], [1.0, 2.0, -3.0]) == pytest.approx(
        (0.0, 0.0, wrap_angle(-6.0))
    )


# ---------------------------------------------------------------------- prior


def test_reset_sets_the_prior_and_wraps_yaw():
    ekf = PoseEKF()
    assert not ekf.initialized
    P0 = np.diag([0.1, 0.2, 0.3])
    ekf.reset([1.0, 2.0, 7.0], P0)
    assert ekf.initialized
    assert ekf.x == pytest.approx([1.0, 2.0, wrap_angle(7.0)])
    assert np.array_equal(ekf.P, P0)
    P0[0, 0] = 99.0  # the filter keeps its own copy
    assert ekf.P[0, 0] == 0.1


# ----------------------------------------------------------------- prediction


@pytest.mark.parametrize(
    'x, u',
    [
        ([0.0, 0.0, 0.0], (0.1, 0.5, -0.2)),
        ([2.0, -1.0, 2.5], (-0.3, 0.2, 0.4)),
        ([1.0, 1.0, -2.0], (0.05, -0.4, 0.0)),  # reverse
    ],
)
def test_predict_mean_is_g(x, u):
    ekf = PoseEKF()
    ekf.reset(x, np.eye(3) * 0.01)
    ekf.predict(*u)
    expected = g(np.array(x), u)
    assert ekf.x[:2] == pytest.approx(expected[:2], abs=1e-12)
    assert abs(wrap_angle(ekf.x[2] - expected[2])) < 1e-12


@pytest.mark.parametrize(
    'x, u', [([0.5, -0.3, 0.7], (0.2, 0.8, -0.1)), ([0.0, 0.0, -2.9], (-0.4, -0.5, 0.3))]
)
def test_predict_state_jacobian_matches_finite_differences(x, u):
    """With no control noise, Sigma_bar = G Sigma G^T, G = dg/dx (PR Table 3.3 line 3)."""
    rng = np.random.default_rng(2)
    P0 = random_spd(rng)
    ekf = PoseEKF(alphas=(0.0, 0.0, 0.0, 0.0))
    ekf.reset(x, P0)
    ekf.predict(*u)
    G = numeric_jacobian(lambda s: g(s, u), x)
    assert ekf.P == pytest.approx(G @ P0 @ G.T, abs=1e-9)


@pytest.mark.parametrize(
    'x, u', [([0.5, -0.3, 0.7], (0.2, 0.8, -0.1)), ([0.0, 0.0, -2.9], (-0.4, -0.5, 0.3))]
)
def test_predict_control_jacobian_matches_finite_differences(x, u):
    """With a certain prior, Sigma_bar = V M V^T, V = dg/du (PR Table 7.2's R_t = V M V^T)."""
    alphas = (0.02, 0.01, 0.01, 0.002)
    ekf = PoseEKF(alphas=alphas)
    ekf.reset(x, np.zeros((3, 3)))
    ekf.predict(*u)
    V = numeric_jacobian(lambda c: g(np.array(x), c), u)
    assert ekf.P == pytest.approx(V @ control_cov(alphas, u) @ V.T, abs=1e-9)


def test_predict_variance_does_not_depend_on_the_odometry_rate():
    """The documented departure from PR Table 5.6: variances are linear in the increment, so
    1 m driven in 1 or in 50 odometry steps (and a quarter turn in 1 or 10) carries the same
    along-track and yaw variance."""
    alphas = (0.02, 0.01, 0.01, 0.002)

    def run(steps, u):
        ekf = PoseEKF(alphas=alphas)
        ekf.reset([0.0, 0.0, 0.0], np.zeros((3, 3)))
        for _ in range(steps):
            ekf.predict(*(c / steps for c in u))
        return ekf.P

    one, many = run(1, (0.0, 1.0, 0.0)), run(50, (0.0, 1.0, 0.0))
    assert one[0, 0] == pytest.approx(many[0, 0]) == pytest.approx(alphas[2])
    assert one[2, 2] == pytest.approx(many[2, 2]) == pytest.approx(alphas[1])
    one, many = run(1, (0.0, 0.0, np.pi / 2)), run(10, (0.0, 0.0, np.pi / 2))
    assert one[2, 2] == pytest.approx(many[2, 2]) == pytest.approx(alphas[0] * np.pi / 2)


def test_predict_keeps_covariance_symmetric_positive_definite():
    rng = np.random.default_rng(3)
    ekf = PoseEKF()
    ekf.reset([0.0, 0.0, 0.0], np.eye(3) * 1e-4)
    for _ in range(500):
        ekf.predict(*rng.uniform([-0.3, -0.2, -0.3], [0.3, 0.2, 0.3]))
        assert is_spd(ekf.P)


# ----------------------------------------------------------------- correction


def test_innovation_wraps_yaw_and_returns_nis():
    ekf = PoseEKF()
    ekf.reset([1.0, 1.0, np.pi - 0.01], np.diag([0.04, 0.04, 0.01]))
    R = np.diag([0.01, 0.01, 0.01])
    y, S, d2 = ekf.innovation([1.1, 0.9, -np.pi + 0.01], R)
    assert y == pytest.approx([0.1, -0.1, 0.02])
    assert S == pytest.approx(ekf.P + R)
    assert d2 == pytest.approx(float(y @ np.linalg.inv(S) @ y))


def test_correct_scalar_case_matches_the_kalman_equations():
    """Diagonal P and R decouple into three scalar filters: mu += p/(p+r) y, p' = p r/(p+r)."""
    p, r = np.array([0.04, 0.09, 0.01]), np.array([0.01, 0.03, 0.04])
    ekf = PoseEKF()
    ekf.reset([0.0, 0.0, 0.0], np.diag(p))
    z = np.array([0.1, -0.2, 0.05])
    assert ekf.correct(z, np.diag(r))
    assert ekf.x == pytest.approx(p / (p + r) * z)
    assert ekf.P == pytest.approx(np.diag(p * r / (p + r)))


def test_correct_joseph_form_equals_the_standard_update():
    """Joseph form (I-K) P (I-K)^T + K R K^T equals (I-K) P for the optimal gain."""
    rng = np.random.default_rng(4)
    for _ in range(50):
        P, R = random_spd(rng, 0.2), random_spd(rng, 0.1)
        ekf = PoseEKF()
        ekf.reset([0.0, 0.0, 0.0], P)
        assert ekf.correct([0.01, -0.01, 0.0], R, gate=None)
        K = P @ np.linalg.inv(P + R)
        assert ekf.P == pytest.approx((np.eye(3) - K) @ P, abs=1e-12)
        assert is_spd(ekf.P)


def test_correct_wraps_the_corrected_yaw():
    ekf = PoseEKF()
    ekf.reset([0.0, 0.0, np.pi - 0.01], np.diag([0.01, 0.01, 0.01]))
    assert ekf.correct([0.0, 0.0, -np.pi + 0.03], np.diag([0.01, 0.01, 1e-6]))
    assert -np.pi <= ekf.x[2] < np.pi
    assert abs(wrap_angle(ekf.x[2] - (-np.pi + 0.03))) < 1e-3


def test_gate_is_the_chi_square_99_percent_quantile():
    assert CHI2_3DOF_99 == pytest.approx(11.345, abs=1e-3)
    P = np.eye(3) * 0.5
    R = np.eye(3) * 0.5  # S = I, so d2 = |y|^2
    for d2, fused in ((CHI2_3DOF_99 - 0.01, True), (CHI2_3DOF_99 + 0.01, False)):
        ekf = PoseEKF()
        ekf.reset([0.0, 0.0, 0.0], P)
        z = [np.sqrt(d2), 0.0, 0.0]
        assert ekf.correct(z, R) is fused
        if not fused:
            assert np.array_equal(ekf.x, np.zeros(3)) and np.array_equal(ekf.P, P)
    ekf = PoseEKF()
    ekf.reset([0.0, 0.0, 0.0], P)
    assert ekf.correct([10.0, 0.0, 0.0], R, gate=None)  # no gate fuses anything


# ------------------------------------------------------------- global prior


def free_space():
    xs, ys = np.meshgrid(np.arange(0.0, 4.0, 0.1), np.arange(0.0, 2.0, 0.1))
    return np.column_stack([xs.ravel(), ys.ravel()])


def test_global_initial_state_centroid_moments():
    xy = free_space()
    x, P = global_initial_state(xy, mean='centroid')
    assert x == pytest.approx([*xy.mean(axis=0), 0.0])
    assert P[:2, :2] == pytest.approx(np.cov(xy.T))
    assert P[2, 2] == pytest.approx((2 * np.pi) ** 2 / 12)
    assert P[:2, 2] == pytest.approx([0.0, 0.0]) and P[2, :2] == pytest.approx([0.0, 0.0])


def test_global_initial_state_random_mean_is_seeded_and_covers_the_spread():
    xy = free_space()
    x, P = global_initial_state(xy, rng=np.random.default_rng(5))
    x2, _ = global_initial_state(xy, rng=np.random.default_rng(5))
    assert np.array_equal(x, x2)
    assert np.any(np.all(np.isclose(xy, x[:2]), axis=1))  # a free cell
    assert -np.pi <= x[2] < np.pi
    # Second moment about the drawn mean: the spread plus the offset from the centroid.
    d = xy.mean(axis=0) - x[:2]
    assert P[:2, :2] == pytest.approx(np.cov(xy.T) + np.outer(d, d))
    with pytest.raises(ValueError):
        global_initial_state(xy, mean='spawn')
