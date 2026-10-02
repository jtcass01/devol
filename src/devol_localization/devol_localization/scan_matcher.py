"""Scan-to-map matching that turns a 2D lidar scan into a pose observation.

The occupancy grid is converted once into a distance field (metres to the
nearest occupied cell). A scan is matched in two stages: a coarse correlative
search over a pose window around the filter's prior (sized from its covariance),
then Levenberg-Marquardt on the squared distances of the endpoints to the nearest
obstacle. The result and an approximate covariance feed the EKF correction step.

Pure numpy/scipy, no ROS.
"""

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from numpy import (ndarray, asarray, array, cos, sin, zeros, full, diag, isfinite, clip, exp,
                   floor, minimum, gradient, int64, float64, sqrt, arange, argmax, unravel_index,
                   where, argsort, rint, linspace, eye)
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
    ambiguity: float = 0.0   # second-best / best search score, 0 without a search


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
                 min_inlier_fraction: float = 0.5, covariance_scale: float = 20.0,
                 min_std: Tuple[float, float] = (0.05, 0.02), search_points: int = 60,
                 max_ambiguity: float = 0.95, peak_dip: float = 0.9,
                 max_peak_checks: int = 200, max_std: float = 10.0) -> None:
        """
        :param inlier_distance: Endpoints farther than this from any obstacle are ignored.
        :param covariance_scale: Inflates the Gauss-Newton covariance, which is
                                 overconfident because neighbouring beams are correlated.
        :param min_std: Floor on the reported (position, yaw) std.
        :param search_points: Endpoints used by the coarse correlative search.
        :param max_ambiguity: Reject a match when a distinct pose in the search window
                              scores at least this fraction of the best one.
        :param peak_dip: A pose counts as distinct only if the search score drops below this
                         fraction of its own score on the straight line to the best pose, so a
                         ridge (a corridor) is not taken for a second peak.
        :param max_peak_checks: Bound on the candidates checked for a dip.
        :param max_std: Cap on the reported std (metres or radians) of a direction the scan
                        does not constrain.
        """
        self.field: DistanceField = field
        self.max_iterations: int = max_iterations
        self.inlier_distance: float = inlier_distance
        self.min_points: int = min_points
        self.min_inlier_fraction: float = min_inlier_fraction
        self.covariance_scale: float = covariance_scale
        self.min_std: Tuple[float, float] = min_std
        self.search_points: int = search_points
        self.max_ambiguity: float = max_ambiguity
        self.peak_dip: float = peak_dip
        self.max_peak_checks: int = max_peak_checks
        self.max_std: float = max_std

    def search(self, points: ndarray, prior: Sequence[float], half_xy: float,
               half_yaw: float, max_cells: int = 31, max_yaws: int = 61) -> Tuple[ndarray, float]:
        """Coarse correlative search over a pose window centred on prior.

        Scores every pose on an (x, y, yaw) grid by how close the scan endpoints land to
        obstacles, so the best pose is found even when the prior is far outside the basin
        of attraction of the gradient refinement. Returns (best pose, ambiguity), where
        ambiguity is the best score among clearly different poses divided by the best score.
        """
        prior = asarray(prior, dtype=float)
        res: float = self.field.resolution
        xy_step: float = max(2.0 * res, 2.0 * half_xy / (max_cells - 1))
        yaw_step: float = max(0.0087, 2.0 * half_yaw / (max_yaws - 1))
        n_xy: int = int(half_xy / xy_step)
        n_yaw: int = int(half_yaw / yaw_step)
        offsets: ndarray = arange(-n_xy, n_xy + 1) * xy_step
        yaws: ndarray = prior[2] + arange(-n_yaw, n_yaw + 1) * yaw_step
        dx: ndarray = (offsets[:, None] + zeros(offsets.size)[None, :]).ravel()
        dy: ndarray = (zeros(offsets.size)[:, None] + offsets[None, :]).ravel()

        step: int = max(1, points.shape[0] // self.search_points)
        pts: ndarray = points[::step]
        sigma: float = max(0.1, xy_step)
        dist: ndarray = self.field.dist
        h, w = dist.shape
        scores: ndarray = zeros((yaws.size, dx.size))
        for k, yaw in enumerate(yaws):
            c, s = cos(yaw), sin(yaw)
            wx: ndarray = prior[0] + c * pts[:, 0] - s * pts[:, 1]
            wy: ndarray = prior[1] + s * pts[:, 0] + c * pts[:, 1]
            cols: ndarray = floor((wx[None, :] + dx[:, None] - self.field.origin_x) / res).astype(int64)
            rows: ndarray = floor((wy[None, :] + dy[:, None] - self.field.origin_y) / res).astype(int64)
            inside: ndarray = (cols >= 0) & (rows >= 0) & (cols < w) & (rows < h)
            d: ndarray = full(cols.shape, self.field.max_distance)
            d[inside] = dist[rows[inside], cols[inside]]
            scores[k] = exp(-0.5 * (d / sigma) ** 2).mean(axis=1)

        k_best, j_best = unravel_index(argmax(scores), scores.shape)
        best: float = float(scores[k_best, j_best])
        pose: ndarray = array([prior[0] + dx[j_best], prior[1] + dy[j_best], wrap_angle(yaws[k_best])])

        # Second peak: best score among poses well away from the winner that the winner is not
        # connected to by a ridge. Along a corridor the score falls off slowly in one direction;
        # that is not a second hypothesis (the refinement covariance carries that direction's
        # uncertainty), so a candidate only counts when the score dips on the way to it.
        far_xy: ndarray = ((dx - dx[j_best]) ** 2 + (dy - dy[j_best]) ** 2) > (2.0 * xy_step + 0.2) ** 2
        far_yaw: ndarray = abs(yaws - yaws[k_best]) > 2.0 * yaw_step + 0.05
        other: ndarray = far_yaw[:, None] | far_xy[None, :]
        grid: ndarray = scores.reshape(yaws.size, offsets.size, offsets.size)
        start: ndarray = array([k_best, j_best // offsets.size, j_best % offsets.size])
        flat_scores: ndarray = scores.ravel()
        flat: ndarray = where(other.ravel())[0]
        # Only candidates that could make the match ambiguous need the (slower) dip check.
        close: ndarray = flat[flat_scores[flat] >= self.max_ambiguity * best]
        rest: ndarray = flat[flat_scores[flat] < self.max_ambiguity * best]
        second: float = float(flat_scores[rest].max()) if rest.size else 0.0
        for idx in close[argsort(-flat_scores[close])][:self.max_peak_checks]:
            k, j = divmod(int(idx), dx.size)
            end: ndarray = array([k, j // offsets.size, j % offsets.size])
            n: int = int(abs(end - start).max()) + 1
            path: ndarray = rint(linspace(0.0, 1.0, n)[:, None] * (end - start) + start).astype(int64)
            if grid[path[:, 0], path[:, 1], path[:, 2]].min() < self.peak_dip * scores[k, j]:
                second = float(scores[k, j])
                break
        return pose, (second / best if best > 0.0 else 1.0)

    def _residuals(self, pose: ndarray, pts: ndarray,
                   inlier_distance: float) -> Tuple[ndarray, ndarray, ndarray]:
        c, s = cos(pose[2]), sin(pose[2])
        rx: ndarray = c * pts[:, 0] - s * pts[:, 1]
        ry: ndarray = s * pts[:, 0] + c * pts[:, 1]
        d, gx, gy = self.field.lookup(pose[0] + rx, pose[1] + ry)
        # d(world point)/d(yaw) = [-ry, rx].
        J: ndarray = array([gx, gy, -gx * ry + gy * rx]).T
        return d, J, d < inlier_distance

    def _refine(self, points: ndarray, pose: ndarray, inlier_distance: float
                ) -> Optional[Tuple[ndarray, ndarray, ndarray, ndarray, int]]:
        """Levenberg-Marquardt on squared endpoint distances over the inlier set."""
        lam: float = 1e-3
        d, J, inl = self._residuals(pose, points, inlier_distance)
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
            d_t, J_t, inl_t = self._residuals(trial, points, inlier_distance)
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
        return pose, d, J, inl, it

    def match(self, points: ndarray, prior: Sequence[float], half_xy: float = 0.0,
              half_yaw: float = 0.0) -> Optional[MatchResult]:
        """Aligns base-frame scan points to the map near prior [x, y, yaw].

        With a non-zero window (half_xy metres, half_yaw radians) a coarse correlative
        search picks the starting pose for the gradient refinement; size the window from
        the filter covariance so the matcher can re-acquire after odometry drifts.
        """
        if points.shape[0] < self.min_points:
            return None
        pose: ndarray = asarray(prior, dtype=float).copy()
        ambiguity: float = 0.0
        if half_xy > 0.0 or half_yaw > 0.0:
            pose, ambiguity = self.search(points, pose, half_xy, half_yaw)
            if ambiguity > self.max_ambiguity:
                return None

        refined = self._refine(points, pose, 2.0 * self.inlier_distance)
        if refined is None:
            return None
        refined = self._refine(points, refined[0], self.inlier_distance)
        if refined is None:
            return None
        pose, d, J, inl, it = refined

        n_in: int = int(inl.sum())
        frac: float = n_in / points.shape[0]
        if n_in < self.min_points or frac < self.min_inlier_fraction:
            return None
        Ji, di = J[inl], d[inl]
        sigma2: float = float((di ** 2).sum() / max(n_in - 3, 1))
        sigma2 = max(sigma2, (0.25 * self.field.resolution) ** 2)
        # A direction the scan cannot observe (along a featureless corridor) has a singular
        # information matrix; a weak prior caps its variance at max_std^2 instead of failing.
        scale: float = sigma2 * self.covariance_scale
        try:
            cov: ndarray = inv(Ji.T @ Ji + eye(3) * (scale / self.max_std ** 2)) * scale
        except LinAlgError:
            return None
        pos_floor, yaw_floor = self.min_std
        floor_diag: ndarray = array([pos_floor ** 2, pos_floor ** 2, yaw_floor ** 2])
        cov = cov + diag(clip(floor_diag - diag(cov), 0.0, None))
        cov = 0.5 * (cov + cov.T)
        return MatchResult(pose=pose, covariance=cov, inlier_fraction=frac,
                           rms_error=float(sqrt((di ** 2).mean())), iterations=it,
                           ambiguity=ambiguity)
