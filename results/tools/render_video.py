"""Renders <estimator>.mp4 for one test-case run from its recorded trajectory.csv, particles.npz and compute.csv.

usage: render_video.py <run dir> <out dir> [step_s=0.5] [fps=20]
Each frame is the whole factory map at one sim instant with the Gazebo ground truth, the estimate, its
2-sigma position ellipse (from the logged std_x/std_y) and, for the PF, a 400-particle subsample of the
cloud it published; the right column is the position and heading error so far.
"""

import csv, json, sys
import numpy as np
import matplotlib

matplotlib.use('Agg')
from matplotlib.animation import FFMpegWriter
from devol_localization.viz_core import LocalizationFigure, VizState

run, out = sys.argv[1], sys.argv[2]
step = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
fps = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0

rows = {}
for r in csv.DictReader(open(f'{run}/trajectory.csv')):
    rows.setdefault(r['estimator'], []).append(
        [float(r[k]) for k in ('t', 'x', 'y', 'yaw', 'std_x', 'std_y', 'std_yaw')]
    )
traj = {k: np.array(v) for k, v in rows.items()}
gt = traj['ground_truth']
summary = json.load(open(f'{run}/summary.json'))
kidnap = (
    summary['config'].get('kidnap_time') if summary['config'].get('scenario') == 'kidnap' else None
)
comp = {}
for r in csv.DictReader(open(f'{run}/compute.csv')):
    comp.setdefault(r['estimator'], []).append((float(r['t']), float(r['compute_ms'])))
comp = {k: np.array(v) for k, v in comp.items()}
P = np.load(f'{run}/particles.npz')
pt, pxy = P['t'], P['xy'].astype(np.float32)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


gyaw = np.unwrap(gt[:, 3])
for est in ('ekf', 'pf', 'hybrid'):
    e = traj[est]
    gx, gy = np.interp(e[:, 0], gt[:, 0], gt[:, 1]), np.interp(e[:, 0], gt[:, 0], gt[:, 2])
    gth = np.interp(e[:, 0], gt[:, 0], gyaw)
    pos_err = np.hypot(e[:, 1] - gx, e[:, 2] - gy)
    yaw_err = np.degrees(wrap(e[:, 3] - gth))
    pos_bound = 2 * np.hypot(e[:, 4], e[:, 5])
    yaw_bound = 2 * np.degrees(e[:, 6])
    s = summary['estimators'][est]
    title = {'ekf': 'EKF', 'pf': 'Particle filter (N = 2000)', 'hybrid': 'Hybrid EKF + PF'}[est]
    case = 'test case 2 (kidnap)' if kidnap else 'test case 1 (nominal, no kidnap)'
    fig = LocalizationFigure(
        mode=est, title=f'{title}, {case}, seed 0', window=0.0, interactive=False
    )
    fig.set_map(P['map_data'], float(P['map_resolution']), *P['map_origin'])
    if fig._particles is not None:
        fig._particles.set_sizes([7.0])
        fig._particles.set_alpha(0.75)
    w = FFMpegWriter(
        fps=fps,
        codec='libx264',
        extra_args=['-crf', '30', '-preset', 'slow', '-pix_fmt', 'yuv420p'],
    )
    path = f'{out}/{est}/{"tc2_kidnap" if kidnap else "tc1_nominal"}/{est}.mp4'
    import os

    os.makedirs(os.path.dirname(path), exist_ok=True)
    t_end = min(e[-1, 0], gt[-1, 0])
    with w.saving(fig.fig, path, dpi=80):
        for t in np.arange(max(e[0, 0], gt[0, 0]), t_end + step / 2, step):
            ig = np.searchsorted(gt[:, 0], t, 'right') - 1
            ie = np.searchsorted(e[:, 0], t, 'right') - 1
            if ie < 0:
                continue
            st = VizState(
                stamp=t,
                truth=gt[ig, 1:4],
                estimate=e[ie, 1:4],
                covariance=np.diag([e[ie, 4] ** 2, e[ie, 5] ** 2, e[ie, 6] ** 2]),
                truth_trail=gt[: ig + 1 : 5, 1:3],
                estimate_trail=e[: ie + 1 : 5, 1:3],
                err_t=e[: ie + 1, 0],
                pos_err=pos_err[: ie + 1],
                pos_bound=pos_bound[: ie + 1],
                yaw_err=yaw_err[: ie + 1],
                yaw_bound=yaw_bound[: ie + 1],
            )
            if est == 'pf':
                ip = np.searchsorted(pt, t, 'right') - 1
                if ip >= 0:
                    c = pxy[ip]
                    st.particles = c[~np.isnan(c[:, 0])]
                    st.status = '(subsample of 2000)\n'
            c = comp.get(est)
            if c is not None:
                ic = np.searchsorted(c[:, 0], t, 'right') - 1
                st.compute_ms = c[ic, 1] if ic >= 0 else None
            if kidnap is not None and t >= kidnap:
                rec = s.get('recovery_time')
                st.status += f'kidnapped at {kidnap:.1f} s' + (
                    f', back <0.25 m at {kidnap + rec:.1f} s'
                    if rec is not None and t >= kidnap + rec
                    else ''
                )
            st.status = st.status.strip()
            fig.update(st)
            w.grab_frame()
    print(path, os.path.getsize(path) // 1024, 'KB')
