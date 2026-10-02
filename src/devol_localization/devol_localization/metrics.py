"""Scoring for the localization study, kept free of ROS so trials can be re-scored offline.

Metrics follow the annotated outline: position RMSE, heading RMSE (wrapped), position error at
each waypoint, per-update compute time, and for the adversarial tests the recovery time, which is
the simulated time from the event (start of a global-localization trial, or the kidnap) until the
position error falls below the threshold (0.25 m) and stays below it for the rest of the trial.
Trials that are still above the threshold at the end, or recover only after the timeout, count as
failures. Aggregation reports mean +/- 95% t-interval and Clopper-Pearson intervals for rates.
"""

import csv
import json
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from devol_localization.pose2d import wrap_angle

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"

RECOVERY_THRESHOLD: float = 0.25


def interpolate_poses(t_ref: np.ndarray, poses_ref: np.ndarray, t_query: np.ndarray) -> np.ndarray:
    """Linearly interpolates (x, y, yaw) poses, unwrapping yaw so it interpolates the short way."""
    t_ref = np.asarray(t_ref, dtype=float)
    poses_ref = np.asarray(poses_ref, dtype=float).reshape(-1, 3)
    yaw = np.unwrap(poses_ref[:, 2])
    out = np.empty((len(t_query), 3))
    out[:, 0] = np.interp(t_query, t_ref, poses_ref[:, 0])
    out[:, 1] = np.interp(t_query, t_ref, poses_ref[:, 1])
    out[:, 2] = wrap_angle(np.interp(t_query, t_ref, yaw))
    return out


def detect_jump(t: np.ndarray, poses: np.ndarray, jump: float = 1.0, max_speed: float = 3.0) -> Optional[float]:
    """Time of the first ground-truth displacement no drive could make (a Gazebo teleport), or None."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    if len(poses) < 2:
        return None
    step = np.hypot(np.diff(poses[:, 0]), np.diff(poses[:, 1]))
    dt = np.maximum(np.diff(np.asarray(t, dtype=float)), 1e-6)
    idx = np.flatnonzero((step > jump) & (step > max_speed * dt))
    return float(t[idx[0] + 1]) if idx.size else None


def recovery_time(t: np.ndarray, pos_err: np.ndarray, event_time: float,
                  threshold: float = RECOVERY_THRESHOLD, timeout: Optional[float] = None) -> Optional[float]:
    """Seconds after event_time until the error drops below threshold for good; None if it never does."""
    t = np.asarray(t, dtype=float)
    err = np.asarray(pos_err, dtype=float)
    after = t >= event_time
    if not after.any():
        return None
    t, err = t[after], err[after]
    if err[-1] >= threshold:
        return None
    bad = np.flatnonzero(err >= threshold)
    rec = 0.0 if bad.size == 0 else float(t[bad[-1] + 1] - event_time)
    if timeout is not None and rec > timeout:
        return None
    return rec


@dataclass
class EstimatorScore:
    samples: int = 0
    pos_rmse: float = float('nan')
    yaw_rmse: float = float('nan')
    pos_mean: float = float('nan')
    pos_max: float = float('nan')
    final_pos_err: float = float('nan')
    waypoint_errors: List[float] = field(default_factory=list)
    compute_ms_mean: float = float('nan')
    compute_ms_median: float = float('nan')
    compute_ms_p95: float = float('nan')
    event_time: Optional[float] = None
    recovered: Optional[bool] = None
    recovery_time: Optional[float] = None


def errors_against_truth(gt_t, gt_poses, est_t, est_poses) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(times, position errors, wrapped yaw errors) of estimate samples inside the ground-truth span."""
    gt_t = np.asarray(gt_t, dtype=float)
    est_t = np.asarray(est_t, dtype=float)
    est_poses = np.asarray(est_poses, dtype=float).reshape(-1, 3)
    inside = (est_t >= gt_t[0]) & (est_t <= gt_t[-1])
    est_t, est_poses = est_t[inside], est_poses[inside]
    gt = interpolate_poses(gt_t, gt_poses, est_t)
    pos = np.hypot(est_poses[:, 0] - gt[:, 0], est_poses[:, 1] - gt[:, 1])
    yaw = wrap_angle(est_poses[:, 2] - gt[:, 2])
    return est_t, pos, yaw


def waypoint_times(gt_t, gt_poses, waypoints: Sequence[Sequence[float]], radius: float = 0.5) -> List[Optional[float]]:
    """Time of closest ground-truth approach to each waypoint, or None if it never came within radius."""
    gt_poses = np.asarray(gt_poses, dtype=float).reshape(-1, 3)
    out: List[Optional[float]] = []
    for wx, wy in waypoints:
        d = np.hypot(gt_poses[:, 0] - wx, gt_poses[:, 1] - wy)
        i = int(np.argmin(d))
        out.append(float(gt_t[i]) if d[i] <= radius else None)
    return out


def score_estimator(gt_t, gt_poses, est_t, est_poses, compute_ms: Sequence[float] = (),
                    waypoints: Sequence[Sequence[float]] = (), event_time: Optional[float] = None,
                    threshold: float = RECOVERY_THRESHOLD, timeout: Optional[float] = None,
                    waypoint_radius: float = 0.5) -> EstimatorScore:
    s = EstimatorScore()
    if len(gt_t) < 2 or len(est_t) == 0:
        return s
    t, pos, yaw = errors_against_truth(gt_t, gt_poses, est_t, est_poses)
    s.samples = int(len(t))
    if s.samples == 0:
        return s
    s.pos_rmse = float(np.sqrt(np.mean(pos ** 2)))
    s.yaw_rmse = float(np.sqrt(np.mean(yaw ** 2)))
    s.pos_mean = float(np.mean(pos))
    s.pos_max = float(np.max(pos))
    s.final_pos_err = float(pos[-1])
    for wt in waypoint_times(gt_t, gt_poses, waypoints, waypoint_radius):
        s.waypoint_errors.append(float('nan') if wt is None else float(pos[int(np.argmin(np.abs(t - wt)))]))
    c = np.asarray(compute_ms, dtype=float)
    if c.size:
        s.compute_ms_mean = float(c.mean())
        s.compute_ms_median = float(np.median(c))
        s.compute_ms_p95 = float(np.percentile(c, 95))
    if event_time is not None:
        s.event_time = float(event_time)
        s.recovery_time = recovery_time(t, pos, event_time, threshold, timeout)
        s.recovered = s.recovery_time is not None
    return s


# ------------------------------------------------------------------ trial files

TRAJECTORY_FIELDS = ['t', 'estimator', 'x', 'y', 'yaw', 'std_x', 'std_y', 'std_yaw']


def write_trajectory_csv(path: Path, rows: Iterable[Sequence]) -> None:
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(TRAJECTORY_FIELDS)
        for r in rows:
            w.writerow([r[0], r[1]] + [f'{v:.6f}' for v in r[2:]])


def read_trajectory_csv(path: Path) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    """estimator -> (times, (N, 3) poses); ground truth is stored as estimator 'ground_truth'."""
    data: Dict[str, List[List[float]]] = {}
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            data.setdefault(row['estimator'], []).append(
                [float(row['t']), float(row['x']), float(row['y']), float(row['yaw'])])
    out = {}
    for name, rows in data.items():
        a = np.asarray(rows)
        order = np.argsort(a[:, 0], kind='stable')
        out[name] = (a[order, 0], a[order, 1:4])
    return out


def write_summary(path: Path, config: dict, scores: Dict[str, EstimatorScore]) -> None:
    def clean(v):
        if isinstance(v, float) and not np.isfinite(v):
            return None
        if isinstance(v, list):
            return [clean(x) for x in v]
        return v
    body = {'config': config,
            'estimators': {k: {kk: clean(vv) for kk, vv in asdict(s).items()} for k, s in scores.items()}}
    with open(path, 'w') as f:
        json.dump(body, f, indent=2)


# ------------------------------------------------------------------ aggregation

def mean_ci95(values: Sequence[float]) -> Tuple[float, float, int]:
    """(mean, 95% t-interval half-width, n) over the finite values."""
    from scipy.stats import t as student_t
    v = np.asarray([x for x in values if x is not None and np.isfinite(x)], dtype=float)
    n = int(v.size)
    if n == 0:
        return float('nan'), float('nan'), 0
    if n == 1:
        return float(v[0]), float('nan'), 1
    half = float(student_t.ppf(0.975, n - 1) * v.std(ddof=1) / np.sqrt(n))
    return float(v.mean()), half, n


def clopper_pearson(successes: int, n: int, confidence: float = 0.95) -> Tuple[float, float]:
    from scipy.stats import beta
    if n == 0:
        return float('nan'), float('nan')
    a = 1.0 - confidence
    lo = 0.0 if successes == 0 else float(beta.ppf(a / 2, successes, n - successes + 1))
    hi = 1.0 if successes == n else float(beta.ppf(1 - a / 2, successes + 1, n - successes))
    return lo, hi


# ------------------------------------------------------------------ test cases

def judge_test_case(case: int, scores: Dict[str, EstimatorScore], waypoint_names: Sequence[str] = (),
                    threshold: float = RECOVERY_THRESHOLD) -> Tuple[bool, List[str]]:
    """PASS/FAIL of the verification test cases, with one report line per check.

    1, nominal route: the EKF and the PF are within `threshold` of ground truth at every waypoint,
       and dead reckoning's mean waypoint error is larger than every filter waypoint error.
    2, kidnapping: the teleport is seen in the ground truth and the PF falls back below `threshold`
       for good before the recovery timeout. The EKF's outcome is reported but not judged.
    """
    lines: List[str] = []
    ok = True

    def wp_text(errs):
        names = list(waypoint_names) + [f'waypoint {i + 1}' for i in range(len(waypoint_names), len(errs))]
        return ', '.join(f'{n}: {"not reached" if not np.isfinite(e) else f"{e:.3f} m"}' for n, e in zip(names, errs))

    if case == 1:
        worst = 0.0
        for name in ('ekf', 'pf'):
            s = scores.get(name)
            if s is None or not s.waypoint_errors:
                lines.append(f'FAIL {name}: no waypoint errors (estimator not running or route not driven)')
                ok = False
                continue
            errs = np.asarray(s.waypoint_errors, dtype=float)
            good = bool(np.all(np.isfinite(errs)) and np.all(errs < threshold))
            worst = max(worst, float(np.nanmax(errs)) if np.isfinite(errs).any() else np.inf)
            ok &= good
            lines.append(f'{"PASS" if good else "FAIL"} {name}: every waypoint within {threshold} m '
                         f'({wp_text(errs)}); position RMSE {s.pos_rmse:.3f} m')
        dr = scores.get('dead_reckoning')
        if dr is not None and dr.waypoint_errors:
            dr_mean = float(np.nanmean(dr.waypoint_errors))
            good = dr_mean > worst
            ok &= good
            lines.append(f'{"PASS" if good else "FAIL"} dead reckoning worse than both filters: mean waypoint '
                         f'error {dr_mean:.3f} m vs worst filter {worst:.3f} m')
        else:
            ok = False
            lines.append('FAIL dead reckoning: no waypoint errors')
    elif case == 2:
        pf = scores.get('pf')
        event = pf.event_time if pf is not None else None
        if event is None:
            lines.append('FAIL kidnap: no teleport found in the ground truth')
            return False, lines
        lines.append(f'kidnap seen at t = {event:.2f} s')
        good = bool(pf.recovered)
        ok &= good
        lines.append(f'{"PASS" if good else "FAIL"} pf: ' + (
            f'back within {threshold} m {pf.recovery_time:.1f} s after the kidnap' if good
            else f'did not return within {threshold} m (final error {pf.final_pos_err:.2f} m)'))
        ekf = scores.get('ekf')
        if ekf is not None:
            lines.append('INFO ekf: ' + (f'recovered after {ekf.recovery_time:.1f} s' if ekf.recovered
                                         else f'did not recover (final error {ekf.final_pos_err:.2f} m)'))
    else:
        raise ValueError(f'unknown test case {case}')
    return ok, lines
