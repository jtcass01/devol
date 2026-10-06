"""Matplotlib figure for watching a localization filter run, kept free of ROS for offline tests.

Layout: the map on the left, with the Gazebo ground-truth pose and the estimate drawn as robot
footprints with their trails, the lidar scan projected from the estimated pose (it lines up with
the walls only when the estimate is right), the 2-sigma position ellipse and, for the particle
filter, every particle. On the right, position and heading error against ground truth over time
with the filter's own 2-sigma bound, so overconfidence shows as error escaping the band.

All drawing happens in the caller's thread; the ROS node calls update() from its main loop
(background-thread Matplotlib has crashed the planner before).
"""

from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from devol_localization.pose2d import transform_points

__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'

# Categorical slots from the project's chart palette (fixed order).
COLOR_TRUTH = '#1baf7a'
COLOR_EKF = '#2a78d6'
COLOR_PF = '#eb6834'
COLOR_HYBRID = '#8a3ec2'
COLOR_SCAN = '#4a3aa7'
TEXT_PRIMARY = '#0b0b0b'
TEXT_SECONDARY = '#52514e'
GRID = '#e4e3df'

# Husky A200 chassis, metres, base frame (approximate outer dimensions).
FOOTPRINT = np.array(
    [[0.495, 0.335], [-0.495, 0.335], [-0.495, -0.335], [0.495, -0.335], [0.495, 0.335]]
)
NOSE = np.array([[0.2, 0.0], [0.55, 0.0]])


@dataclass
class VizState:
    """Everything one frame shows. Poses are (x, y, yaw) in the map frame."""

    stamp: float = 0.0
    truth: Optional[np.ndarray] = None
    estimate: Optional[np.ndarray] = None
    covariance: Optional[np.ndarray] = None  # 3x3 (x, y, yaw)
    scan_points: Optional[np.ndarray] = None  # (N, 2) in the map frame
    particles: Optional[np.ndarray] = None  # (M, 3)
    truth_trail: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    estimate_trail: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    err_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    pos_err: np.ndarray = field(default_factory=lambda: np.zeros(0))
    pos_bound: np.ndarray = field(default_factory=lambda: np.zeros(0))  # 2 sigma
    yaw_err: np.ndarray = field(default_factory=lambda: np.zeros(0))  # degrees
    yaw_bound: np.ndarray = field(default_factory=lambda: np.zeros(0))  # 2 sigma, degrees
    compute_ms: Optional[float] = None
    status: str = ''


def ellipse_points(
    mean: Sequence[float], cov2: np.ndarray, n_sigma: float = 2.0, n: int = 64
) -> np.ndarray:
    """Outline of the n-sigma ellipse of a 2x2 covariance."""
    vals, vecs = np.linalg.eigh(np.asarray(cov2, dtype=float))
    vals = np.maximum(vals, 0.0)
    t = np.linspace(0.0, 2.0 * np.pi, n)
    circle = np.vstack((np.cos(t), np.sin(t)))
    pts = vecs @ (n_sigma * np.sqrt(vals)[:, None] * circle)
    return (pts + np.asarray(mean[:2], dtype=float)[:, None]).T


def map_image(grid: np.ndarray) -> np.ndarray:
    """RGB image of an OccupancyGrid (row 0 = lowest y): free white, occupied dark, unknown grey."""
    g = np.asarray(grid)
    img = np.empty(g.shape + (3,), dtype=np.float32)
    img[:] = 0.86
    img[(g >= 0) & (g < 50)] = 1.0
    img[g >= 50] = 0.18
    return img


class LocalizationFigure:
    def __init__(
        self,
        mode: str = 'ekf',
        title: str = '',
        window: float = 0.0,
        history: float = 0.0,
        interactive: bool = True,
    ) -> None:
        """
        :param mode: 'ekf', 'hybrid' (drawn like the EKF) or 'pf' (draws particles and uses the PF colour).
        :param window: Side of the square view that follows the robot, metres; 0 shows the whole map.
        :param history: Seconds of error history to show; 0 shows the whole run.
        """
        import matplotlib

        if not interactive:
            matplotlib.use('Agg', force=True)
        import matplotlib.pyplot as plt
        from matplotlib.gridspec import GridSpec

        self._plt = plt
        self.mode = mode
        self.window = window
        self.history = history
        color = {'pf': COLOR_PF, 'hybrid': COLOR_HYBRID}.get(mode, COLOR_EKF)
        label = {'pf': 'PF', 'hybrid': 'Hybrid EKF+PF'}.get(mode, 'EKF')
        self._map_extent = None

        if interactive:
            plt.ion()
        self.fig = plt.figure(figsize=(13.0, 7.2), facecolor='white')
        gs = GridSpec(
            2,
            2,
            width_ratios=[1.55, 1.0],
            hspace=0.32,
            wspace=0.18,
            left=0.05,
            right=0.97,
            top=0.92,
            bottom=0.08,
            figure=self.fig,
        )
        self.ax_map = self.fig.add_subplot(gs[:, 0])
        self.ax_pos = self.fig.add_subplot(gs[0, 1])
        self.ax_yaw = self.fig.add_subplot(gs[1, 1], sharex=self.ax_pos)
        self.fig.suptitle(
            title or f'{label} localization vs Gazebo ground truth',
            color=TEXT_PRIMARY,
            fontsize=13,
            x=0.05,
            ha='left',
        )

        ax = self.ax_map
        ax.set_aspect('equal')
        ax.set_xlabel('x (m)', color=TEXT_SECONDARY)
        ax.set_ylabel('y (m)', color=TEXT_SECONDARY)
        self._map_artist = None
        (self._truth_trail,) = ax.plot([], [], color=COLOR_TRUTH, lw=1.5, alpha=0.8, zorder=3)
        (self._est_trail,) = ax.plot([], [], color=color, lw=1.5, alpha=0.8, zorder=3, ls='--')
        self._particles = (
            ax.scatter(
                [], [], s=4, color=color, alpha=0.5, linewidths=0, zorder=9, label='particles'
            )
            if mode == 'pf'
            else None
        )
        self._scan = ax.scatter(
            [], [], s=5, color=COLOR_SCAN, linewidths=0, zorder=5, label='lidar from estimate'
        )
        (self._ellipse,) = ax.plot(
            [], [], color=color, lw=1.5, zorder=6, label=r'2$\sigma$ position'
        )
        (self._truth_body,) = ax.plot(
            [], [], color=COLOR_TRUTH, lw=2.0, zorder=7, label='Gazebo ground truth'
        )
        (self._truth_nose,) = ax.plot([], [], color=COLOR_TRUTH, lw=2.0, zorder=7)
        (self._est_body,) = ax.plot(
            [], [], color=color, lw=2.0, zorder=8, label=f'{label} estimate'
        )
        (self._est_nose,) = ax.plot([], [], color=color, lw=2.0, zorder=8)
        ax.legend(loc='upper left', fontsize=8, framealpha=0.9, markerscale=3)
        self._status = ax.text(
            0.99,
            0.01,
            '',
            transform=ax.transAxes,
            ha='right',
            va='bottom',
            fontsize=9,
            family='monospace',
            color=TEXT_PRIMARY,
            bbox=dict(facecolor='white', alpha=0.85, edgecolor=GRID),
        )

        for a, ylabel in (
            (self.ax_pos, 'position error (m)'),
            (self.ax_yaw, 'heading error (deg)'),
        ):
            a.set_ylabel(ylabel, color=TEXT_SECONDARY)
            a.grid(True, color=GRID, lw=0.8)
            for s in ('top', 'right'):
                a.spines[s].set_visible(False)
        self.ax_yaw.set_xlabel('sim time (s)', color=TEXT_SECONDARY)
        (self._pos_line,) = self.ax_pos.plot([], [], color=color, lw=2.0, label='error')
        (self._pos_bound,) = self.ax_pos.plot(
            [], [], color=color, lw=1.0, ls=':', label=r'filter 2$\sigma$'
        )
        self.ax_pos.axhline(0.25, color=TEXT_SECONDARY, lw=1.0, ls='--')
        self.ax_pos.text(
            1.0,
            0.25,
            ' 0.25 m',
            transform=self.ax_pos.get_yaxis_transform(),
            va='center',
            ha='left',
            fontsize=8,
            color=TEXT_SECONDARY,
            clip_on=False,
        )
        self.ax_pos.legend(loc='upper left', fontsize=8)
        (self._yaw_line,) = self.ax_yaw.plot([], [], color=color, lw=2.0)
        (self._yaw_bound_hi,) = self.ax_yaw.plot([], [], color=color, lw=1.0, ls=':')
        (self._yaw_bound_lo,) = self.ax_yaw.plot([], [], color=color, lw=1.0, ls=':')
        self.ax_yaw.axhline(0.0, color=TEXT_SECONDARY, lw=0.8)

        if interactive:
            self.fig.show()

    # ------------------------------------------------------------------ map
    def set_map(
        self, grid: np.ndarray, resolution: float, origin_x: float, origin_y: float
    ) -> None:
        h, w = np.asarray(grid).shape
        extent = (origin_x, origin_x + w * resolution, origin_y, origin_y + h * resolution)
        img = map_image(grid)
        if self._map_artist is None:
            self._map_artist = self.ax_map.imshow(
                img, origin='lower', extent=extent, interpolation='nearest', zorder=1
            )
        else:
            self._map_artist.set_data(img)
            self._map_artist.set_extent(extent)
        self._map_extent = extent
        self.ax_map.set_xlim(extent[0], extent[1])
        self.ax_map.set_ylim(extent[2], extent[3])

    # --------------------------------------------------------------- update
    def update(self, s: VizState) -> None:
        def body(line, nose, pose):
            if pose is None:
                line.set_data([], [])
                nose.set_data([], [])
                return
            b = transform_points(pose, FOOTPRINT)
            n = transform_points(pose, NOSE)
            line.set_data(b[:, 0], b[:, 1])
            nose.set_data(n[:, 0], n[:, 1])

        body(self._truth_body, self._truth_nose, s.truth)
        body(self._est_body, self._est_nose, s.estimate)
        self._truth_trail.set_data(s.truth_trail[:, 0], s.truth_trail[:, 1])
        self._est_trail.set_data(s.estimate_trail[:, 0], s.estimate_trail[:, 1])
        self._scan.set_offsets(
            s.scan_points if s.scan_points is not None and len(s.scan_points) else np.zeros((0, 2))
        )
        if self._particles is not None:
            self._particles.set_offsets(
                s.particles[:, :2]
                if s.particles is not None and len(s.particles)
                else np.zeros((0, 2))
            )
        if s.estimate is not None and s.covariance is not None:
            e = ellipse_points(s.estimate, s.covariance[:2, :2])
            self._ellipse.set_data(e[:, 0], e[:, 1])
        else:
            self._ellipse.set_data([], [])

        center = s.truth if s.truth is not None else s.estimate
        if self.window > 0.0 and center is not None:
            h = self.window / 2.0
            self.ax_map.set_xlim(center[0] - h * 1.3, center[0] + h * 1.3)
            self.ax_map.set_ylim(center[1] - h, center[1] + h)

        t = s.err_t
        self._pos_line.set_data(t, s.pos_err)
        self._pos_bound.set_data(t, s.pos_bound)
        self._yaw_line.set_data(t, s.yaw_err)
        self._yaw_bound_hi.set_data(t, s.yaw_bound)
        self._yaw_bound_lo.set_data(t, -s.yaw_bound)
        if t.size:
            t0 = max(t[0], t[-1] - self.history) if self.history > 0.0 else t[0]
            self.ax_pos.set_xlim(t0, max(t[-1], t0 + 1.0))
            shown = t >= t0
            # Scale to the error and the typical bound, so a wide initial covariance does not flatten the plot.
            top = max(np.nanmax(s.pos_err[shown]), np.nanmedian(s.pos_bound[shown]), 0.3)
            self.ax_pos.set_ylim(0.0, 1.15 * top)
            ytop = max(np.nanmax(np.abs(s.yaw_err[shown])), np.nanmedian(s.yaw_bound[shown]), 2.0)
            self.ax_yaw.set_ylim(-1.15 * ytop, 1.15 * ytop)

        lines = [f't = {s.stamp:8.2f} s']
        if s.pos_err.size:
            lines.append(f'pos err  {s.pos_err[-1]:6.3f} m (2σ {s.pos_bound[-1]:.3f})')
            lines.append(f'yaw err  {s.yaw_err[-1]:6.2f}° (2σ {s.yaw_bound[-1]:.2f})')
        if s.particles is not None:
            lines.append(f'particles {len(s.particles):5d}')
        if s.compute_ms is not None:
            lines.append(f'update   {s.compute_ms:6.2f} ms')
        if s.status:
            lines.append(s.status)
        self._status.set_text('\n'.join(lines))

    def draw(self) -> None:
        """Renders pending changes and services the GUI event loop (interactive) or just renders (Agg)."""
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def is_open(self) -> bool:
        return self._plt.fignum_exists(self.fig.number)

    def save(self, path: str) -> None:
        self.fig.savefig(path, dpi=100)

    def close(self) -> None:
        self._plt.close(self.fig)


class VideoRecorder:
    """Writes the figure's frames to an MP4 with ffmpeg, or OpenCV's mp4v codec when ffmpeg is missing.

    A missing encoder never stops the live view: with neither available, recording is skipped and
    `backend` is None.
    """

    def __init__(self, fig, path: str, fps: float) -> None:
        self.fig = fig
        self.path = path
        self.backend: Optional[str] = None
        self._writer = None
        self._size = None
        from matplotlib.animation import writers

        if writers.is_available('ffmpeg'):
            from matplotlib.animation import FFMpegWriter

            self._writer = FFMpegWriter(fps=fps, bitrate=4000)
            self._writer.setup(fig, path, dpi=100)
            self.backend = 'ffmpeg'
            return
        try:
            import cv2
        except ImportError:
            return
        self._cv2 = cv2
        self._fps = fps
        self.backend = 'opencv'

    def grab(self) -> None:
        if self.backend == 'ffmpeg':
            self._writer.grab_frame()
        elif self.backend == 'opencv':
            self.fig.canvas.draw()
            rgba = np.asarray(self.fig.canvas.buffer_rgba())
            frame = self._cv2.cvtColor(rgba, self._cv2.COLOR_RGBA2BGR)
            if self._writer is None:
                self._size = (frame.shape[1], frame.shape[0])
                self._writer = self._cv2.VideoWriter(
                    self.path, self._cv2.VideoWriter_fourcc(*'mp4v'), self._fps, self._size
                )
            elif (frame.shape[1], frame.shape[0]) != self._size:  # window resized
                frame = self._cv2.resize(frame, self._size)
            self._writer.write(frame)

    def finish(self) -> None:
        if self._writer is None:
            return
        if self.backend == 'ffmpeg':
            self._writer.finish()
        else:
            self._writer.release()
        self._writer = None


def posterior_figure(
    grid,
    resolution: float,
    origin: Sequence[float],
    truth,
    ekf_pose,
    ekf_cov,
    pf_pose,
    pf_cov,
    particles,
    scan_points=None,
    stamp: float = 0.0,
    window: float = 8.0,
    title: str = '',
):
    """One frame comparing the two posteriors at the same instant: the PF's particle set and the EKF's
    single Gaussian (2-sigma ellipse), with ground truth. Returns a Matplotlib figure (Agg)."""
    from matplotlib.figure import Figure
    from matplotlib.lines import Line2D

    fig = Figure(figsize=(7.5, 6.6), facecolor='white')
    ax = fig.add_subplot(1, 1, 1)
    h, w = np.asarray(grid).shape
    ax.imshow(
        map_image(grid),
        origin='lower',
        interpolation='nearest',
        zorder=1,
        extent=(origin[0], origin[0] + w * resolution, origin[1], origin[1] + h * resolution),
    )
    handles = []
    if particles is not None and len(particles):
        p = np.asarray(particles)
        ax.quiver(
            p[:, 0],
            p[:, 1],
            np.cos(p[:, 2]),
            np.sin(p[:, 2]),
            color=COLOR_PF,
            alpha=0.35,
            zorder=4,
            angles='xy',
            scale_units='xy',
            scale=1.0 / 0.25,
            width=0.002,
            headwidth=3,
        )
        handles.append(
            Line2D(
                [],
                [],
                color=COLOR_PF,
                marker='>',
                ls='',
                alpha=0.6,
                label=f'PF particles ({len(p)})',
            )
        )
    if scan_points is not None and len(scan_points):
        ax.scatter(
            scan_points[:, 0], scan_points[:, 1], s=4, color=COLOR_SCAN, linewidths=0, zorder=3
        )
        handles.append(
            Line2D(
                [], [], color=COLOR_SCAN, marker='o', ls='', ms=3, label='lidar from ground truth'
            )
        )
    for pose, cov, color, label in (
        (pf_pose, pf_cov, COLOR_PF, 'PF'),
        (ekf_pose, ekf_cov, COLOR_EKF, 'EKF'),
    ):
        if pose is None:
            continue
        if cov is not None:
            e = ellipse_points(pose, np.asarray(cov)[:2, :2])
            ax.plot(e[:, 0], e[:, 1], color=color, lw=2.0, zorder=6)
        b, n = transform_points(pose, FOOTPRINT), transform_points(pose, NOSE)
        ax.plot(b[:, 0], b[:, 1], color=color, lw=2.0, ls='--', zorder=7)
        ax.plot(n[:, 0], n[:, 1], color=color, lw=2.0, zorder=7)
        err = (
            ''
            if truth is None
            else f', error {np.hypot(pose[0] - truth[0], pose[1] - truth[1]):.2f} m'
        )
        handles.append(
            Line2D([], [], color=color, lw=2.0, label=f'{label} mean and 2σ ellipse{err}')
        )
    if truth is not None:
        b, n = transform_points(truth, FOOTPRINT), transform_points(truth, NOSE)
        ax.plot(b[:, 0], b[:, 1], color=COLOR_TRUTH, lw=2.5, zorder=8)
        ax.plot(n[:, 0], n[:, 1], color=COLOR_TRUTH, lw=2.5, zorder=8)
        handles.append(Line2D([], [], color=COLOR_TRUTH, lw=2.5, label='Gazebo ground truth'))
    # Frame the truth and both estimates (each with window/2 of margin), within the map.
    poses = [q for q in (truth, ekf_pose, pf_pose) if q is not None]
    if window > 0.0 and poses:
        xy = np.asarray([q[:2] for q in poses])
        lo, hi = xy.min(axis=0) - window / 2, xy.max(axis=0) + window / 2
        extent = (origin[0], origin[0] + w * resolution, origin[1], origin[1] + h * resolution)
        ax.set_xlim(max(lo[0], extent[0]), min(hi[0], extent[1]))
        ax.set_ylim(max(lo[1], extent[2]), min(hi[1], extent[3]))
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)', color=TEXT_SECONDARY)
    ax.set_ylabel('y (m)', color=TEXT_SECONDARY)
    ax.legend(handles=handles, loc='upper left', fontsize=8, framealpha=0.9)
    ax.set_title(
        title or f'Posteriors at t = {stamp:.1f} s', color=TEXT_PRIMARY, loc='left', fontsize=11
    )
    fig.tight_layout()
    return fig
