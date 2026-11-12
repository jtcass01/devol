#!/usr/bin/env python3

import re
import warnings
from pathlib import Path

import numpy as np
from cycler import cycler
from matplotlib.pyplot import close, rcParams, subplots
from pandas import Categorical, DataFrame, concat, isna, read_csv
from scipy.stats import chi2, sem
from scipy.stats import t as student_t


__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'

RESULTS_DIR = Path('test_results')
TABLE_DIR = Path('tables')
OUT_DIR = Path('figures')
ALGOS = ['EKF', 'DR', 'PF']
REF_NOISE = 0.0  # noise scale used for the headline accuracy figure
KIDNAP_JUMP = 1.0  # [m] ground truth (or shared error) jump that marks the kidnap

# Mahalanobis recovery criterion
HOLD_TIME = 2.0  # [s] window over which the criterion must hold
RECOVERY_HORIZON = 20.0  # [s] every kidnapped run is judged over this window after the kidnap
NEES_DOF = 3  # x, y, yaw
NEES_CONF = 0.95  # chi-square confidence for the windowed NEES bound
MAX_POS_SIGMA = 0.3  # [m] filter must also be this confident in position
MAX_YAW_SIGMA = 0.2  # [rad] and in yaw, so "very uncertain" does not count as recovered
NEES_GATE = chi2.ppf(NEES_CONF, NEES_DOF)  # per-sample gate, used for coverage

# Absolute-error criterion, kept only as a sensitivity check
RECOVER_POS_THRESH = 0.3  # [m]
RECOVER_YAW_THRESH = 0.2  # [rad]

# Kidnap-aligned error profiles
PROFILE_TAU = np.arange(-5.0, 40.0 + 1e-9, 0.1)  # [s] time since kidnap

METRIC_NAMES = {
    'pos_rmse': 'Position RMSE (m)',
    'yaw_rmse': 'Yaw RMSE (rad)',
    'anees': 'ANEES',
    'nees_coverage': 'NEES Coverage',
    'recovered': 'Recovery Rate (NEES)',
    'recovered_abs': 'Recovery Rate (Abs. Error)',
    'recovery_s': 'Recovery Time (s)',
    'post_pos_rmse': 'Post-Recovery Position RMSE (m)',
    'runtime_ms': 'Average Runtime (ms)',
}
FULL_NAMES = {
    'EKF': 'Extended Kalman Filter (EKF)',
    'DR': 'Dead Reckoning (DR)',
    'PF': 'Particle Filter (PF)',
}
COV_IJ = {
    'Var_X': (0, 0),
    'Var_Y': (1, 1),
    'Var_Yaw': (2, 2),
    'Cov_XY': (0, 1),
    'Cov_XYaw': (0, 2),
    'Cov_YYaw': (1, 2),
}

# JHU brand palette (verify hex values against brand.jhu.edu)
JHU = {
    'heritage_blue': '#002D72',
    'spirit_blue': '#68ACE5',
    'orange': '#FF9E1B',
    'green': '#009B77',
    'red': '#CF4520',
    'purple': '#A15A95',
    'gold': '#F1C400',
}
COLORS = list(JHU.values())
rcParams.update(
    {
        'axes.prop_cycle': cycler(color=COLORS),
        'axes.grid': True,
        'grid.linestyle': ':',
        'grid.alpha': 0.6,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'errorbar.capsize': 4,
        'savefig.dpi': 300,
        'savefig.bbox': 'tight',
    }
)

PATH_RE = re.compile(r'particles_(\d+)/noise_([\d.]+)/(baseline|kidnapped)/run_(\d+)/')
PALETTE = {}  # label -> color, filled in main() so every figure and table uses the same order


def set_palette(particles):
    labels = ALGOS + [f'PF-{p}' for p in particles]
    PALETTE.clear()
    PALETTE.update({lbl: COLORS[i % len(COLORS)] for i, lbl in enumerate(labels)})


def label_of(algo, particles):
    return f'PF-{particles}' if algo == 'PF' else algo


# per-sample metrics
def covariance_stack(df, a):
    P = np.empty((len(df), 3, 3))
    for name, (i, j) in COV_IJ.items():
        P[:, i, j] = P[:, j, i] = df[f'{a}_{name}'].to_numpy()
    return P


def nees(err, P):
    out = np.full(len(err), np.nan)
    idx = np.flatnonzero(np.isfinite(P).all(axis=(1, 2)) & np.isfinite(err).all(axis=1))
    if idx.size:
        idx = idx[np.linalg.eigvalsh(P[idx]).min(axis=1) > 0]
        sol = np.linalg.solve(P[idx], err[idx][..., None])[..., 0]
        out[idx] = np.einsum('ij,ij->i', err[idx], sol)
    return out


def derive_metrics(df):
    for a in ALGOS:
        dx = (df[f'{a}_X'] - df['GT_X']).to_numpy()
        dy = (df[f'{a}_Y'] - df['GT_Y']).to_numpy()
        dyaw_raw = (df[f'{a}_Yaw'] - df['GT_Yaw']).to_numpy()
        dyaw = np.arctan2(np.sin(dyaw_raw), np.cos(dyaw_raw))
        df[f'{a}_Pos_Err'] = np.hypot(dx, dy)
        df[f'{a}_Yaw_Err'] = np.abs(dyaw)
        df[f'{a}_NEES'] = nees(np.column_stack([dx, dy, dyaw]), covariance_stack(df, a))
        with np.errstate(invalid='ignore'):
            df[f'{a}_Sigma_Pos'] = np.sqrt(df[f'{a}_Var_X'] + df[f'{a}_Var_Y'])
            df[f'{a}_Sigma_Yaw'] = np.sqrt(df[f'{a}_Var_Yaw'])
    return df


# per-run metrics
def kidnap_time(df):
    gt_step = np.hypot(df['GT_X'].diff(), df['GT_Y'].diff())
    err_step = concat([df[f'{a}_Pos_Err'].diff() for a in ALGOS], axis=1).min(axis=1)
    for step in (gt_step, err_step):
        if step.notna().any():
            i = step.idxmax()
            if step[i] > KIDNAP_JUMP:
                return df.at[i, 'Time(s)']
    return np.nan


def held(mask, n):
    return mask.astype(float).rolling(n, min_periods=n).min().to_numpy() == 1


def first_start(tt, ok, n):
    return tt[int(np.argmax(ok)) - n + 1] if ok.any() else np.nan


def accuracy_metrics(pos, yaw, nees_v):
    finite = nees_v[np.isfinite(nees_v)]
    return dict(
        pos_rmse=np.sqrt(np.nanmean(pos**2)),
        yaw_rmse=np.sqrt(np.nanmean(yaw**2)),
        anees=finite.mean() if finite.size else np.nan,
        nees_coverage=(finite < NEES_GATE).mean() if finite.size else np.nan,
    )


def recovery_metrics(df, a, times, t0):
    out = dict(
        recovery_s=np.nan,
        recovered=np.nan,
        recovered_abs=np.nan,
        post_pos_rmse=np.nan,
        post_yaw_rmse=np.nan,
    )
    if np.isnan(t0):
        return out
    after = times >= t0
    tt, d = times[after], df.loc[after]
    if len(tt) < 2:
        return out
    n = max(1, int(round(HOLD_TIME / np.median(np.diff(tt)))))

    # Mahalanobis criterion: windowed mean NEES within the chi-square bound for n*DOF,
    # AND the filter is confident for the whole window.
    gate = chi2.ppf(NEES_CONF, NEES_DOF * n) / n
    consistent = (d[f'{a}_NEES'].rolling(n, min_periods=n).mean() < gate).to_numpy()
    confident = held(
        (d[f'{a}_Sigma_Pos'] < MAX_POS_SIGMA) & (d[f'{a}_Sigma_Yaw'] < MAX_YAW_SIGMA), n
    )
    rec = first_start(tt, consistent & confident, n) - t0

    # Absolute-error criterion (sensitivity check only)
    abs_ok = held(
        (d[f'{a}_Pos_Err'] < RECOVER_POS_THRESH) & (d[f'{a}_Yaw_Err'] < RECOVER_YAW_THRESH), n
    )
    rec_abs = first_start(tt, abs_ok, n) - t0

    # Judge every run over the same window. A recovery starting at RECOVERY_HORIZON
    # still needs HOLD_TIME of data after it to be confirmed.
    if tt[-1] - t0 < RECOVERY_HORIZON + HOLD_TIME:
        return out  # too short to judge; excluded from recovery stats
    rec = rec if rec <= RECOVERY_HORIZON else np.nan
    rec_abs = rec_abs if rec_abs <= RECOVERY_HORIZON else np.nan

    out.update(
        recovery_s=rec,
        recovered=float(not np.isnan(rec)),
        recovered_abs=float(not np.isnan(rec_abs)),
    )
    if not np.isnan(rec):
        post = tt >= t0 + rec
        out['post_pos_rmse'] = np.sqrt(np.nanmean(d[f'{a}_Pos_Err'].to_numpy()[post] ** 2))
        out['post_yaw_rmse'] = np.sqrt(np.nanmean(d[f'{a}_Yaw_Err'].to_numpy()[post] ** 2))
    return out


def ci95(x):
    """Half-width of the 95% t-interval of the mean."""
    x = x.dropna()
    return sem(x) * student_t.ppf(0.975, len(x) - 1) if len(x) > 1 else np.nan


def summarize(d, by, col):
    return d.groupby(by, observed=True)[col].agg(mean='mean', ci=ci95, n='count').reset_index()


def load(root=RESULTS_DIR):
    runs, traces = [], []
    for csv in sorted(root.glob('particles_*/noise_*/*/run_*/*.csv')):
        m = PATH_RE.search(csv.as_posix())
        if not m:
            print(f'Skipping {csv}: unrecognized path')
            continue
        try:
            df = read_csv(csv)
            if len(df) < 2:
                raise ValueError('fewer than 2 rows')
            df = derive_metrics(df)
            particles, noise, cond, seed = int(m[1]), float(m[2]), m[3], int(m[4])
            times = df['Time(s)'].to_numpy()
            kidnapped = cond == 'kidnapped'
            t0 = kidnap_time(df) if kidnapped else np.nan
            if kidnapped and np.isnan(t0):
                print(f'Warning: no kidnap detected in {csv}')

            for a in ALGOS:
                label = label_of(a, particles)
                pos, yaw, nees_v = (
                    df[f'{a}_{k}'].to_numpy() for k in ('Pos_Err', 'Yaw_Err', 'NEES')
                )
                row = dict(
                    particles=particles,
                    noise=noise,
                    condition=cond,
                    seed=seed,
                    algo=a,
                    label=label,
                    runtime_ms=df[f'{a}_Runtime'].mean() * 1e3,
                    **accuracy_metrics(pos, yaw, nees_v),
                )
                if kidnapped:
                    row.update(recovery_metrics(df, a, times, t0))
                    if not np.isnan(t0) and np.isnan(row['recovered']):
                        print(
                            f'Warning: {csv} ({a}) ended < {RECOVERY_HORIZON + HOLD_TIME:g} s '
                            'after kidnap; excluded from recovery stats'
                        )
                runs.append(row)

                if not np.isnan(t0):
                    rel = times - t0
                    traces.append(
                        dict(
                            label=label,
                            noise=noise,
                            **{
                                k: np.interp(PROFILE_TAU, rel, v, left=np.nan, right=np.nan)
                                for k, v in (('pos', pos), ('yaw', yaw), ('nees', nees_v))
                            },
                        )
                    )
        except Exception as e:
            print(f'Skipping {csv}: {e}')
    return DataFrame(runs), traces


def by_estimator(df):
    pf = df[df.algo == 'PF']
    other = (
        df[df.algo != 'PF']
        .groupby(['label', 'algo', 'noise', 'condition', 'seed'], as_index=False)
        .mean(numeric_only=True)
    )
    out = concat([other, pf], ignore_index=True)
    out['label'] = Categorical(out.label, categories=list(PALETTE), ordered=True)
    return out


# Plotting
def lines(ax, d, x, col, ylabel):
    s = summarize(d, ['label', x], col)
    for label, g in s.groupby('label', observed=True):
        ax.errorbar(
            g[x], g['mean'], yerr=g['ci'], marker='o', label=str(label), color=PALETTE[str(label)]
        )
    ax.set_ylabel(ylabel)
    return s.assign(metric=col)


def bars(ax, d, col, ylabel):
    """Mean +/- 95% CI bars with the individual runs overlaid."""
    s = summarize(d, ['label'], col)
    names = s.label.astype(str).tolist()
    x = np.arange(len(names))
    ax.bar(x, s['mean'], yerr=s['ci'], color=[PALETTE[n] for n in names], alpha=0.85)
    rng = np.random.default_rng(0)
    for i, name in enumerate(names):
        y = d.loc[d.label == name, col].dropna().to_numpy()
        ax.scatter(
            i + rng.uniform(-0.2, 0.2, y.size),
            y,
            s=10,
            color='k',
            alpha=0.5,
            linewidths=0,
            zorder=3,
        )
    ax.set_xticks(x, names, rotation=30)
    ax.set_ylabel(ylabel)
    return s.assign(metric=col)


def plot_recovery_curves(kid_runs, noises):
    """Fraction of runs recovered within tau seconds of the kidnap (unrecovered = failure)."""
    taus = np.linspace(0, RECOVERY_HORIZON, 201)
    fig, axs = subplots(1, len(noises), figsize=(4.5 * len(noises), 4), sharey=True, squeeze=False)
    for ax, nz in zip(axs[0], noises):
        g = kid_runs[(kid_runs.noise == nz) & kid_runs.recovered.notna()]
        for label in PALETTE:
            r = g[g.label == label]
            if r.empty:
                continue
            rec = np.where(r.recovered == 1, r.recovery_s, np.inf)  # never recovered = inf
            frac = (rec[None, :] <= taus[:, None]).mean(axis=1)
            ax.step(taus, frac, where='post', color=PALETTE[label], label=f'{label} (n={len(r)})')
        ax.set(title=f'Noise {nz:g}', xlabel='Time since kidnap (s)', ylim=(0, 1.05))
    axs[0, 0].set_ylabel('Fraction of runs recovered')
    axs[0, 0].legend(fontsize='small')
    fig.suptitle(f'Recovery within {RECOVERY_HORIZON:g} s of kidnapping (Mahalanobis criterion)')
    save(fig, 'recovery_curves')


def plot_profiles(traces, noise):
    """Median and IQR of error and NEES vs. time since kidnap, one band per estimator."""
    groups = {}
    for tr in traces:
        if tr['noise'] == noise:
            groups.setdefault(tr['label'], []).append(tr)
    if not groups:
        return
    panels = (('pos', 'Position error (m)'), ('yaw', 'Yaw error (rad)'), ('nees', 'NEES'))
    fig, axs = subplots(3, 1, figsize=(9, 9), sharex=True)
    for label in (lbl for lbl in PALETTE if lbl in groups):
        for ax, (key, _) in zip(axs, panels):
            Y = np.vstack([tr[key] for tr in groups[label]])
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)  # all-NaN columns
                lo, med, hi = np.nanpercentile(Y, [25, 50, 75], axis=0)
            ax.plot(PROFILE_TAU, med, color=PALETTE[label], label=label)
            ax.fill_between(PROFILE_TAU, lo, hi, color=PALETTE[label], alpha=0.2, linewidth=0)
    for ax, (_, ylabel) in zip(axs, panels):
        ax.set_ylabel(ylabel)
        ax.axvline(0, color='k', lw=0.8)
    axs[2].set_yscale('log')
    axs[2].axhline(NEES_DOF, color='k', ls=':', lw=1, label=f'Expected ({NEES_DOF})')
    axs[2].axhline(NEES_GATE, color='k', ls='--', lw=1, label=f'{NEES_CONF:.0%} gate')
    axs[2].legend(loc='upper right', fontsize='small')
    axs[0].legend(ncol=4, fontsize='small')
    axs[-1].set_xlabel('Time since kidnap (s)')
    fig.suptitle(f'Kidnap-aligned error profiles, noise {noise:g} (median, IQR band)')
    save(fig, f'kidnap_profile_noise_{noise:g}')


def save(fig, name, summaries=()):
    OUT_DIR.mkdir(exist_ok=True)
    for ext in ('png', 'pdf'):  # PDF is vector, better for LaTeX
        fig.savefig(OUT_DIR / f'{name}.{ext}')
    close(fig)
    if summaries:
        table = concat(summaries)
        table.to_csv(OUT_DIR / f'{name}_summary.csv', index=False)
        print(f'\n=== {name} (mean, 95% CI half-width, n) ===\n{table.to_string(index=False)}')


# Latex
def full_name(label):
    """'EKF' -> 'Extended Kalman Filter (EKF)', 'PF-500' -> 'Particle Filter (500)'."""
    return FULL_NAMES.get(label) or f'Particle Filter ({label[3:]})'


def fmt(m, c, digits):
    if isna(m):
        return '--'
    return f'{m:.{digits}f}' if isna(c) else f'{m:.{digits}f} $\\pm$ {c:.{digits}f}'


def latex(s, name, caption, cols='metric', head=None, digits=3):
    """Write a table with one row per estimator and mean +/- 95% CI in each cell.

    cols: summary column whose values become the table columns.
    head: format string for column headers (e.g. 'Noise {:g}'); default uses METRIC_NAMES."""
    s = s.assign(label=s.label.astype(str))
    order = [lbl for lbl in PALETTE if lbl in set(s.label)]
    p = s.pivot(index='label', columns=cols, values=['mean', 'ci']).reindex(order)
    keys = p['mean'].columns
    heads = ['Algorithm'] + [METRIC_NAMES[k] if head is None else head.format(k) for k in keys]
    rows = [
        ' & '.join(
            [full_name(lbl)]
            + [fmt(p['mean'].at[lbl, k], p['ci'].at[lbl, k], digits) for k in keys]
        )
        for lbl in p.index
    ]

    out = [
        r'\begin{table}[!htbp]',
        r'    \centering',
        f'    \\caption{{{caption} Values are mean $\\pm$ 95\\% CI.}}',
        f'    \\label{{tab:{name}}}',
        f'    \\begin{{tabular}}{{l{"c" * len(keys)}}}',
        r'        \hline',
        '        ' + ' & '.join(f'\\textbf{{{h}}}' for h in heads) + r' \\',
        r'        \hline',
        *[f'        {row} \\\\' for row in rows],
        r'        \hline',
        r'    \end{tabular}',
        r'\end{table}',
    ]
    TABLE_DIR.mkdir(exist_ok=True)
    (TABLE_DIR / f'{name}.tex').write_text('\n'.join(out) + '\n')


def main():
    raw, traces = load()
    if raw.empty:
        raise SystemExit(f'No results found under {RESULTS_DIR}/')
    set_palette(sorted(raw.particles.unique()))
    noises = sorted(raw.noise.unique())
    est = by_estimator(raw)
    base = est[est.condition == 'baseline']
    kid = est[est.condition == 'kidnapped']
    noise_x = 'Odometry noise scale'
    noise_col = 'Noise {:g}'

    # Accuracy at the reference noise level
    fig, axs = subplots(1, 2, figsize=(11, 4))
    ref = base[base.noise == REF_NOISE]
    s = [
        bars(axs[0], ref, 'pos_rmse', 'Position RMSE (m)'),
        bars(axs[1], ref, 'yaw_rmse', 'Yaw RMSE (rad)'),
    ]
    fig.suptitle(f'Accuracy, baseline, noise scale {REF_NOISE:g} (95% CI, dots = runs)')
    save(fig, 'accuracy', s)
    latex(
        concat(s),
        'accuracy',
        f'Baseline localization accuracy at odometry noise scale {REF_NOISE:g}.',
    )

    # Noise sensitivity (log y so DR drift doesn't flatten the other curves)
    fig, axs = subplots(1, 2, figsize=(11, 4), sharex=True)
    s = [
        lines(axs[0], base, 'noise', 'pos_rmse', 'Position RMSE (m)'),
        lines(axs[1], base, 'noise', 'yaw_rmse', 'Yaw RMSE (rad)'),
    ]
    for ax in axs:
        ax.set(xlabel=noise_x, yscale='log')
    axs[0].legend()
    fig.suptitle('Accuracy vs. odometry noise, baseline (95% CI)')
    save(fig, 'noise', s)
    latex(
        s[0],
        'noise_pos',
        'Baseline position RMSE (m) vs. odometry noise scale.',
        cols='noise',
        head=noise_col,
    )
    latex(
        s[1],
        'noise_yaw',
        'Baseline yaw RMSE (rad) vs. odometry noise scale.',
        cols='noise',
        head=noise_col,
    )

    # Consistency: does each filter's covariance honestly describe its error?
    fig, axs = subplots(1, 2, figsize=(11, 4), sharex=True)
    s = [
        lines(axs[0], base, 'noise', 'anees', 'ANEES'),
        lines(
            axs[1],
            base,
            'noise',
            'nees_coverage',
            f'Fraction of samples inside {NEES_CONF:.0%} gate',
        ),
    ]
    axs[0].axhline(NEES_DOF, color='k', ls='--', lw=1, label=f'Consistent ({NEES_DOF})')
    axs[0].set_yscale('log')
    axs[1].axhline(NEES_CONF, color='k', ls='--', lw=1, label=f'Consistent ({NEES_CONF:g})')
    axs[1].set_ylim(0, 1.05)
    for ax in axs:
        ax.set_xlabel(noise_x)
    axs[0].legend()
    fig.suptitle('Filter consistency vs. odometry noise, baseline (95% CI)')
    save(fig, 'consistency', s)
    latex(
        s[0],
        'consistency_anees',
        f'Baseline average NEES vs. odometry noise scale (consistent filter: {NEES_DOF}; '
        'higher is overconfident, lower is underconfident).',
        cols='noise',
        head=noise_col,
        digits=2,
    )
    latex(
        s[1],
        'consistency_coverage',
        f'Fraction of baseline samples with NEES inside the {NEES_CONF * 100:g}\\% '
        f'$\\chi^2_{{{NEES_DOF}}}$ gate (consistent filter: {NEES_CONF:g}).',
        cols='noise',
        head=noise_col,
        digits=2,
    )

    # Kidnap recovery summary
    crit_txt = (
        f'mean NEES over {HOLD_TIME:g} s within {NEES_CONF:.0%} chi-square bound, '
        f'sigma_pos < {MAX_POS_SIGMA} m, sigma_yaw < {MAX_YAW_SIGMA} rad'
    )
    crit_tex = (
        f'mean NEES over {HOLD_TIME:g} s within the {NEES_CONF * 100:g}\\% $\\chi^2$ '
        f'bound, $\\sigma_{{pos}} <$ {MAX_POS_SIGMA} m and '
        f'$\\sigma_{{yaw}} <$ {MAX_YAW_SIGMA} rad'
    )
    fig, axs = subplots(1, 3, figsize=(16, 4), sharex=True)
    s = [
        lines(axs[0], kid, 'noise', 'recovered', 'Recovery rate'),
        lines(axs[1], kid, 'noise', 'recovery_s', 'Time to recover (s), recovered runs'),
        lines(axs[2], kid, 'noise', 'post_pos_rmse', 'Post-recovery position RMSE (m)'),
    ]
    axs[0].set_ylim(0, 1.05)
    for ax in axs:
        ax.set_xlabel(noise_x)
    axs[0].legend()
    fig.suptitle(f'Kidnapping recovery ({crit_txt}; 95% CI)')
    save(fig, 'recovery', s)
    latex(
        s[0],
        'recovery_rate',
        f'Fraction of kidnapped runs recovered within {RECOVERY_HORIZON:g} s ({crit_tex}).',
        cols='noise',
        head=noise_col,
        digits=2,
    )
    latex(
        s[1],
        'recovery_time',
        f'Time to recover (s) from kidnapping, recovered runs only ({crit_tex}).',
        cols='noise',
        head=noise_col,
        digits=1,
    )
    latex(
        s[2],
        'recovery_rmse',
        'Position RMSE (m) from the moment of recovery to the end of the run.',
        cols='noise',
        head=noise_col,
    )

    # Criterion sensitivity: do the Mahalanobis and absolute-error criteria agree?
    sens = concat(
        [summarize(kid, ['label'], c).assign(metric=c) for c in ('recovered', 'recovered_abs')]
    )
    latex(
        sens,
        'recovery_criteria',
        f'Recovery rate under the Mahalanobis criterion ({crit_tex}) and the absolute-error '
        f'criterion (error $<$ {RECOVER_POS_THRESH} m and $<$ {RECOVER_YAW_THRESH} rad for '
        f'{HOLD_TIME:g} s), pooled over noise levels.',
        digits=2,
    )

    # Recovery over time (handles unrecovered runs without survivorship bias)
    plot_recovery_curves(raw[raw.condition == 'kidnapped'], noises)

    # Kidnap-aligned error and NEES profiles
    for nz in noises:
        plot_profiles(traces, nz)

    # Runtime vs. particle count (baseline, pooled over noise levels)
    rt = raw[raw.condition == 'baseline']
    rt = rt.assign(label=Categorical(rt.algo, categories=list(PALETTE), ordered=True))
    particles = sorted(raw.particles.unique())
    fig, ax = subplots(figsize=(6, 4))
    s = [lines(ax, rt, 'particles', 'runtime_ms', 'Mean update time (ms)')]
    ax.set(xscale='log', yscale='log', xlabel='PF particle count')
    ax.set_xticks(particles, [str(p) for p in particles])
    ax.minorticks_off()
    ax.legend()
    ax.set_title('Runtime per update (95% CI)')
    save(fig, 'runtime', s)
    latex(
        summarize(base, ['label'], 'runtime_ms').assign(metric='runtime_ms'),
        'runtime',
        'Average runtime per update step across algorithms, baseline runs pooled over all '
        'noise levels.',
    )


if __name__ == '__main__':
    main()
