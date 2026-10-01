"""Scan-to-map matching that turns a 2D lidar scan into a pose observation.

The occupancy grid is converted once into a distance field (metres to the
nearest occupied cell). A scan is matched by Gauss-Newton / Levenberg-Marquardt
on the squared distances of its endpoints to the nearest obstacle, starting
from the filter's prior pose. The result and an approximate covariance feed
the EKF correction step.

Pure numpy/scipy, no ROS.
"""

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from numpy import (ndarray, asarray, array, cos, sin, zeros, diag, isfinite, clip,
                   floor, minimum, gradient, int64, float64, sqrt, arange)
from numpy.linalg import solve, inv, LinAlgError
from scipy.ndimage import distance_transform_edt

from devol_localization.ekf_core import wrap_angle

__author__ = "Jacob Taylor Cassady"
__email__ = "jcassad1@jh.edu"


@dataclass
class MatchResult:
    pose: ndarray            # [x, y, yaw] in the map frame
    covariance: ndarray      # 3x3
    inlier_fraction: float
    rms_error: float         # metres, over inliers
    iterations: int


class DistanceField:
    """Distance (m) from every grid cell to the nearest occupied cell."""

    def __init__(self, grid: ndarray, resolution: float, origin: Tuple[float, float, float],
                 occupied_threshold: int = 50, max_distance: float = 2.0) -> None:
        """
        :param grid: (height, width) int array in ROS OccupancyGrid convention,
                     row 0 at origin y, values -1 (unknown) or 0..100.
        :param origin: (x, y, yaw) of cell (0, 0)'s corner in the map frame.
        """
        if abs(origin[2]) > 1e-6:
            raise ValueError('Rotated occupancy grid origins are not supported.')
        self.resolution: float = float(resolution)
        self.origin_x: float = float(origin[0])
        self.origin_y: float = float(origin[1])
        self.max_distance: float = float(max_distance)
        occupied: ndarray = asarray(grid) >= occupied_threshold
        self.height, self.width = occupied.shape
        if occupied.any():
            dist: ndarray = distance_transform_edt(~occupied) * self.resolution
        else:
            dist = zeros(occupied.shape) + self.max_distance
        self.dist: ndarray = minimum(dist, self.max_distance).astype(float64)
        # Gradients in metres per metre, axis 0 = y (rows), axis 1 = x (cols).
        self.grad_y, self.grad_x = gradient(self.dist, self.resolution)

    def lookup(self, xs: ndarray, ys: ndarray) -> Tuple[ndarray, ndarray, ndarray]:
        """Bilinear distance and gradient at map-frame points. Points off the grid get
        max_distance and zero gradient."""
        # Cell centres sit at origin + (i + 0.5) * res.
        u: ndarray = (xs - self.origin_x) / self.resolution - 0.5
        v: ndarray = (ys - self.origin_y) / self.resolution - 0.5
        inside: ndarray = (u >= 0) & (v >= 0) & (u < self.width - 1) & (v < self.height - 1)
        d: ndarray = zeros(xs.shape) + self.max_distance
        gx: ndarray = zeros(xs.shape)
        gy: ndarray = zeros(xs.shape)
        if not inside.any():
            return d, gx, gy
        ui, vi = u[inside], v[inside]
        c0: ndarray = floor(ui).astype(int64)
        r0: ndarray = floor(vi).astype(int64)
        fu: ndarray = ui - c0
        fv: ndarray = vi - r0
        w00 = (1 - fu) * (1 - fv)
        w01 = fu * (1 - fv)
        w10 = (1 - fu) * fv
        w11 = fu * fv

        def interp(field: ndarray) -> ndarray:
            return (w00 * field[r0, c0] + w01 * field[r0, c0 + 1]
                    + w10 * field[r0 + 1, c0] + w11 * field[r0 + 1, c0 + 1])

        d[inside] = interp(self.dist)
        gx[inside] = interp(self.grad_x)
        gy[inside] = interp(self.grad_y)
        return d, gx, gy


def scan_to_points(ranges: Sequence[float], angle_min: float, angle_increment: float,
                   range_min: float, range_max: float, beam_step: int = 1,
                   laser_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> ndarray:
    """Returns valid scan endpoints as an (N, 2) array in the robot base frame."""
    r: ndarray = asarray(ranges, dtype=float)
    angles: ndarray = angle_min + angle_increment * arange(r.size)
    idx: ndarray = arange(0, r.size, max(1, int(beam_step)))
    r, angles = r[idx], angles[idx]
    # Beams at range_max are misses, not obstacles.
    valid: ndarray = isfinite(r) & (r > range_min) & (r < range_max * 0.99)
    r, angles = r[valid], angles[valid]
    lx, ly, lyaw = laser_pose
    a: ndarray = angles + lyaw
    return array([lx + r * cos(a), ly + r * sin(a)]).T.reshape(-1, 2)


class ScanMatcher:
    def __init__(self, field: DistanceField, max_iterations: int = 20,
                 inlier_distance: float = 0.3, min_points: int = 30,
                 min_inlier_fraction: float = 0.5, covariance_scale: float = 10.0,
                 min_std: Tuple[float, float] = (0.02, 0.01)) -> None:
        """
        :param inlier_distance: Endpoints farther than this from any obstacle are ignored.
        :param covariance_scale: Inflates the Gauss-Newton covariance, which is
                                 overconfident because neighbouring beams are correlated.
        :param min_std: Floor on the reported (position, yaw) std.
        """
        self.field: DistanceField = field
        self.max_iterations: int = max_iterations
        self.inlier_distance: float = inlier_distance
        self.min_points: int = min_points
        self.min_inlier_fraction: float = min_inlier_fraction
        self.covariance_scale: float = covariance_scale
        self.min_std: Tuple[float, float] = min_std

    def _residuals(self, pose: ndarray, pts: ndarray) -> Tuple[ndarray, ndarray, ndarray]:
        c, s = cos(pose[2]), sin(pose[2])
        rx: ndarray = c * pts[:, 0] - s * pts[:, 1]
        ry: ndarray = s * pts[:, 0] + c * pts[:, 1]
        d, gx, gy = self.field.lookup(pose[0] + rx, pose[1] + ry)
        # d(world point)/d(yaw) = [-ry, rx].
        J: ndarray = array([gx, gy, -gx * ry + gy * rx]).T
        return d, J, d < self.inlier_distance

    def match(self, points: ndarray, prior: Sequence[float]) -> Optional[MatchResult]:
        """Aligns base-frame scan points to the map starting from prior [x, y, yaw]."""
        if points.shape[0] < self.min_points:
            return None
        pose: ndarray = asarray(prior, dtype=float).copy()
        lam: float = 1e-3
        d, J, inl = self._residuals(pose, points)
        cost: float = float((d[inl] ** 2).sum())
        it: int = 0
        for it in range(1, self.max_iterations + 1):
            if inl.sum() < self.min_points:
                return None
            Ji, di = J[inl], d[inl]
            H: ndarray = Ji.T @ Ji
            g: ndarray = Ji.T @ di
            try:
                step: ndarray = -solve(H + lam * diag(diag(H) + 1e-9), g)
            except LinAlgError:
                return None
            trial: ndarray = pose + step
            trial[2] = wrap_angle(trial[2])
            d_t, J_t, inl_t = self._residuals(trial, points)
            # Compare costs over the same inlier set to keep the test fair.
            trial_cost: float = float((d_t[inl] ** 2).sum())
            if trial_cost < cost:
                pose, d, J, inl = trial, d_t, J_t, inl_t
                cost = float((d[inl] ** 2).sum())
                lam = max(lam * 0.3, 1e-7)
                if abs(step[0]) < 1e-4 and abs(step[1]) < 1e-4 and abs(step[2]) < 1e-4:
                    break
            else:
                lam *= 10.0
                if lam > 1e4:
                    break

        n_in: int = int(inl.sum())
        frac: float = n_in / points.shape[0]
        if n_in < self.min_points or frac < self.min_inlier_fraction:
            return None
        Ji, di = J[inl], d[inl]
        sigma2: float = float((di ** 2).sum() / max(n_in - 3, 1))
        sigma2 = max(sigma2, (0.25 * self.field.resolution) ** 2)
        try:
            cov: ndarray = inv(Ji.T @ Ji) * sigma2 * self.covariance_scale
        except LinAlgError:
            return None
        pos_floor, yaw_floor = self.min_std
        floor_diag: ndarray = array([pos_floor ** 2, pos_floor ** 2, yaw_floor ** 2])
        cov = cov + diag(clip(floor_diag - diag(cov), 0.0, None))
        cov = 0.5 * (cov + cov.T)
        return MatchResult(pose=pose, covariance=cov, inlier_fraction=frac,
                           rms_error=float(sqrt((di ** 2).mean())), iterations=it)
