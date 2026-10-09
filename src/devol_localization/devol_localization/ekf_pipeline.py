"""Glue between odometry, the scan matcher and the EKF, kept free of ROS for offline tests.

Each scan is matched inside a search window sized from the filter covariance (3 sigma,
clamped). When matching keeps failing, the covariance is inflated on every failed scan so
the window widens until the matcher re-acquires the pose or the window reaches its limit
(inflation stops there, so a long run of unusable scans does not blow up the covariance).

Scans are fused at their own timestamps. The pipeline keeps a short history of filter states
at each odometry message; a scan that arrives after newer odometry (the matcher fell behind, or
transport delay) rewinds the filter to the scan time, is fused there, and the odometry since
then is re-applied. A scan stamped after the newest odometry waits for the next odometry
message. Without this, a scan taken during a fast turn is fused as if taken later, which pulls
the heading back by the rotation in between (0.15 s at 3 rad/s is 26 degrees).

Theory (references in THEORY.md at the package root):
- Search window: the EKF's predicted covariance bounds where the true pose can be, so the
  matcher searches +/- 3 sigma around the predicted mean, the same idea as a validation region
  in target tracking (Bar-Shalom et al. 2001 [barshalom2001estimation]).
- Inflation while lost: adding process noise while the observations keep failing is a
  heuristic (not from the references) that widens the belief, and with it the search window,
  until a match lands inside the gate again. It is the EKF's counterpart of augmented MCL's
  random particles; a single Gaussian cannot do more, which is why the kidnapped-robot case
  needs the particle filter (Probabilistic Robotics Section 7.1 [thrun2005probabilistic]).
- Late scans: re-running the filter from the stored state at the scan's time over the
  odometry since then is the exact (reprocessing) way to fuse an out-of-sequence measurement;
  Bar-Shalom (2002) [barshalom2002oosm] derives cheaper retrodiction-based alternatives.
"""

from bisect import bisect_right
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from numpy import ndarray, asarray, clip, diag, sqrt

from devol_localization.ekf_core import PoseEKF, odometry_delta, wrap_angle, CHI2_3DOF_99
from devol_localization.scan_matcher import ScanMatcher, MatchResult

__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'


@dataclass
class PipelineStats:
    scans: int = 0
    matched: int = 0
    fused: int = 0
    gated: int = 0
    failed_in_row: int = 0
    reacquired: int = 0
    rewound: int = 0  # scans fused after rewinding past newer odometry
    stale: int = 0  # scans dropped: older than the history, or superseded while waiting
    match_calls: int = 0  # matcher runs, successful or not


@dataclass
class _HistoryEntry:
    stamp: float
    odom: Tuple[float, float, float]
    x: ndarray
    P: ndarray


def _interpolate(p0: Sequence[float], p1: Sequence[float], f: float) -> Tuple[float, float, float]:
    return (
        p0[0] + f * (p1[0] - p0[0]),
        p0[1] + f * (p1[1] - p0[1]),
        wrap_angle(p0[2] + f * wrap_angle(p1[2] - p0[2])),
    )


class EKFPipeline:
    def __init__(
        self,
        ekf: PoseEKF,
        matcher: Optional[ScanMatcher] = None,
        window_xy: Tuple[float, float] = (0.15, 1.5),
        window_yaw: Tuple[float, float] = (0.05, 0.8),
        gate: Optional[float] = CHI2_3DOF_99,
        lost_after: int = 5,
        lost_inflation_std: Tuple[float, float] = (0.05, 0.03),
        history: float = 2.0,
        max_scan_lead: float = 0.5,
    ) -> None:
        """
        :param window_xy: (min, max) half-width of the search window in metres.
        :param window_yaw: (min, max) half-width of the search window in radians.
        :param lost_after: Consecutive failed scans before the covariance starts inflating.
        :param lost_inflation_std: (position, yaw) std added per failed scan once lost.
        :param history: Seconds of filter states kept for fusing late scans at their stamps.
        :param max_scan_lead: A scan stamped more than this after the newest odometry is fused
                              at once instead of waiting, so a clock mismatch between the two
                              topics degrades to unaligned fusion rather than no fusion.
        """
        self.ekf: PoseEKF = ekf
        self.matcher: Optional[ScanMatcher] = matcher
        self.window_xy: Tuple[float, float] = window_xy
        self.window_yaw: Tuple[float, float] = window_yaw
        self.gate: Optional[float] = gate
        self.lost_after: int = lost_after
        self.lost_inflation_std: Tuple[float, float] = lost_inflation_std
        self.stats: PipelineStats = PipelineStats()
        self.history: float = history
        self.max_scan_lead: float = max_scan_lead
        self._last_odom: Optional[Tuple[float, float, float]] = None
        self._history: deque = deque()
        self._pending: Optional[Tuple[ndarray, float]] = None

    @property
    def last_odom(self) -> Optional[Tuple[float, float, float]]:
        return self._last_odom

    def clear_history(self) -> None:
        """Forget stored states, e.g. after the filter is re-initialized."""
        self._history.clear()
        self._pending = None

    def on_odom(
        self, odom_pose: Sequence[float], stamp: Optional[float] = None
    ) -> Optional[MatchResult]:
        """Predicts with the odometry increment. Returns the fused match of a scan that was
        waiting for this message, if any."""
        pose = (float(odom_pose[0]), float(odom_pose[1]), float(odom_pose[2]))
        if self.ekf.initialized and self._last_odom is not None:
            self.ekf.predict(*odometry_delta(self._last_odom, pose))
        self._last_odom = pose
        if stamp is None or not self.ekf.initialized:
            return None
        if self._history and stamp < self._history[-1].stamp:
            # Time went backwards (bag loop or sim reset): start over.
            self.clear_history()
        self._history.append(_HistoryEntry(stamp, pose, self.ekf.x.copy(), self.ekf.P.copy()))
        while self._history[0].stamp < stamp - self.history:
            self._history.popleft()
        if self._pending is not None and self._pending[1] <= stamp:
            points, scan_stamp = self._pending
            self._pending = None
            return self._fuse_at(points, scan_stamp)
        return None

    def search_window(self) -> Tuple[float, float]:
        """3-sigma half-widths (position, yaw) from the predicted covariance, clamped."""
        P: ndarray = self.ekf.P
        half_xy: float = float(clip(3.0 * sqrt(max(P[0, 0], P[1, 1])), *self.window_xy))
        half_yaw: float = float(clip(3.0 * sqrt(P[2, 2]), *self.window_yaw))
        return half_xy, half_yaw

    def on_scan(self, points: ndarray, stamp: Optional[float] = None) -> Optional[MatchResult]:
        """Matches base-frame scan points and fuses the result. Returns the fused match, if any.

        :param stamp: Scan time, on the same clock as the odometry stamps. Without it (or without
                      stamped odometry) the scan is fused at the latest odometry time.
        """
        if self.matcher is None or not self.ekf.initialized:
            return None
        self.stats.scans += 1
        if stamp is None or not self._history:
            return self._match_and_fuse(points)
        if stamp > self._history[-1].stamp + self.max_scan_lead:
            return self._match_and_fuse(points)
        if stamp > self._history[-1].stamp:
            if self._pending is not None:
                self.stats.stale += 1
            self._pending = (points, stamp)
            return None
        return self._fuse_at(points, stamp)

    def _fuse_at(self, points: ndarray, stamp: float) -> Optional[MatchResult]:
        """Rewinds to stamp, fuses the scan there and re-applies the later odometry.

        The odometry pose at the scan time is interpolated linearly between the two stored
        odometry messages around it, and the stored state there is predicted forward to it.
        """
        hist: List[_HistoryEntry] = list(self._history)
        if stamp < hist[0].stamp:
            self.stats.stale += 1
            return None
        j: int = bisect_right([e.stamp for e in hist], stamp) - 1
        if j == len(hist) - 1:
            # Stamped exactly at the newest odometry: fuse now and keep the stored state current.
            result = self._match_and_fuse(points)
            hist[j].x, hist[j].P = self.ekf.x.copy(), self.ekf.P.copy()
            return result
        e, nxt = hist[j], hist[j + 1]
        f: float = (stamp - e.stamp) / (nxt.stamp - e.stamp) if nxt.stamp > e.stamp else 0.0
        odom_at_scan = _interpolate(e.odom, nxt.odom, f)
        self.ekf.x, self.ekf.P = e.x.copy(), e.P.copy()
        self.ekf.predict(*odometry_delta(e.odom, odom_at_scan))
        self.stats.rewound += 1
        result: Optional[MatchResult] = self._match_and_fuse(points)
        # Keep the corrected state, so a later scan that rewinds past this one keeps its correction.
        self._history.insert(
            j + 1, _HistoryEntry(stamp, odom_at_scan, self.ekf.x.copy(), self.ekf.P.copy())
        )
        prev = odom_at_scan
        for later in list(self._history)[j + 2 :]:
            self.ekf.predict(*odometry_delta(prev, later.odom))
            later.x, later.P = self.ekf.x.copy(), self.ekf.P.copy()
            prev = later.odom
        return result

    def _match_and_fuse(self, points: ndarray) -> Optional[MatchResult]:
        """Measurement update: scan-match around the predicted pose (the measurement model),
        then fuse the match with the gated EKF correction."""
        self.stats.match_calls += 1
        half_xy, half_yaw = self.search_window()
        result: Optional[MatchResult] = self.matcher.match(points, self.ekf.x, half_xy, half_yaw)
        if result is not None:
            self.stats.matched += 1
            if self.ekf.correct(result.pose, result.covariance, gate=self.gate):
                self.stats.fused += 1
                if self.stats.failed_in_row >= self.lost_after:
                    self.stats.reacquired += 1
                self.stats.failed_in_row = 0
                return result
            self.stats.gated += 1
        self.stats.failed_in_row += 1
        if self.stats.failed_in_row >= self.lost_after and (
            half_xy < self.window_xy[1] or half_yaw < self.window_yaw[1]
        ):
            pos_std, yaw_std = self.lost_inflation_std
            self.ekf.P = self.ekf.P + diag(asarray([pos_std**2, pos_std**2, yaw_std**2]))
        return None
