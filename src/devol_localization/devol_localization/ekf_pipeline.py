"""Glue between odometry, the scan matcher and the EKF, kept free of ROS for offline tests.

Each scan is matched inside a search window sized from the filter covariance (3 sigma,
clamped). When matching keeps failing, the covariance is inflated on every failed scan so
the window widens until the matcher re-acquires the pose or the window reaches its limit
(inflation stops there, so a long run of unusable scans does not blow up the covariance).
"""

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from numpy import ndarray, asarray, clip, diag, sqrt

from devol_localization.ekf_core import PoseEKF, odometry_delta, CHI2_3DOF_99
from devol_localization.scan_matcher import ScanMatcher, MatchResult

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


@dataclass
class PipelineStats:
    scans: int = 0
    matched: int = 0
    fused: int = 0
    gated: int = 0
    failed_in_row: int = 0
    reacquired: int = 0


class EKFPipeline:
    def __init__(self, ekf: PoseEKF, matcher: Optional[ScanMatcher] = None,
                 window_xy: Tuple[float, float] = (0.15, 1.5),
                 window_yaw: Tuple[float, float] = (0.05, 0.8),
                 gate: Optional[float] = CHI2_3DOF_99,
                 lost_after: int = 5,
                 lost_inflation_std: Tuple[float, float] = (0.05, 0.03)) -> None:
        """
        :param window_xy: (min, max) half-width of the search window in metres.
        :param window_yaw: (min, max) half-width of the search window in radians.
        :param lost_after: Consecutive failed scans before the covariance starts inflating.
        :param lost_inflation_std: (position, yaw) std added per failed scan once lost.
        """
        self.ekf: PoseEKF = ekf
        self.matcher: Optional[ScanMatcher] = matcher
        self.window_xy: Tuple[float, float] = window_xy
        self.window_yaw: Tuple[float, float] = window_yaw
        self.gate: Optional[float] = gate
        self.lost_after: int = lost_after
        self.lost_inflation_std: Tuple[float, float] = lost_inflation_std
        self.stats: PipelineStats = PipelineStats()
        self._last_odom: Optional[Tuple[float, float, float]] = None

    @property
    def last_odom(self) -> Optional[Tuple[float, float, float]]:
        return self._last_odom

    def on_odom(self, odom_pose: Sequence[float]) -> None:
        pose = (float(odom_pose[0]), float(odom_pose[1]), float(odom_pose[2]))
        if self.ekf.initialized and self._last_odom is not None:
            self.ekf.predict(*odometry_delta(self._last_odom, pose))
        self._last_odom = pose

    def search_window(self) -> Tuple[float, float]:
        P: ndarray = self.ekf.P
        half_xy: float = float(clip(3.0 * sqrt(max(P[0, 0], P[1, 1])), *self.window_xy))
        half_yaw: float = float(clip(3.0 * sqrt(P[2, 2]), *self.window_yaw))
        return half_xy, half_yaw

    def on_scan(self, points: ndarray) -> Optional[MatchResult]:
        """Matches base-frame scan points and fuses the result. Returns the fused match, if any."""
        if self.matcher is None or not self.ekf.initialized:
            return None
        self.stats.scans += 1
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
        if self.stats.failed_in_row >= self.lost_after and (half_xy < self.window_xy[1]
                                                            or half_yaw < self.window_yaw[1]):
            pos_std, yaw_std = self.lost_inflation_std
            self.ekf.P = self.ekf.P + diag(asarray([pos_std ** 2, pos_std ** 2, yaw_std ** 2]))
        return None
