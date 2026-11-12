#!/usr/bin/env python3
import re
from pathlib import Path
from typing import Optional

from cycler import cycler
from matplotlib.pyplot import close, rcParams, subplots
from numpy import flatnonzero, isnan, nan
from pandas import Categorical, DataFrame, concat, isna, read_csv
from scipy.stats import sem, t

RESULTS_DIR = Path('test_results')
TABLE_DIR = Path('tables')
OUT_DIR = Path('figures')
ALGOS = ['EKF', 'DR', 'PF']
REF_NOISE = 0.0  # noise scale used for the headline accuracy figure
KIDNAP_JUMP = 1.0  # [m] simultaneous error jump (all estimators) that marks the kidnap
RECOVER_POS_THRESH = 0.3  # [m] error considered "recovered"
RECOVER_YAW_THRESH = 0.2  # [rad] error considered "recovered"
HOLD_TIME = 2.0  # [s] error must stay below RECOVER_THRESH this long to count

METRIC_NAMES = {
    'pos_rmse': 'Position RMSE (m)',
    'yaw_rmse': 'Yaw RMSE (rad)',
    'recovered': 'Recovery Rate',
    'recovery_s': 'Recovery Time (s)',
    'runtime_ms': 'Average Runtime (ms)',
}
FULL_NAMES = {'EKF': 'Extended Kalman Filter (EKF)', 'DR': 'Dead Reckoning (DR)'}

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


# ----------------------------------------------------------------------------- metrics
def kidnap_time(df) -> float:
    """Time of the largest error jump shared by ALL estimators, or NaN if none found."""
    if len(df) < 2:
        return nan
    jump = concat([df[f'{a}_Pos_Err'].diff() for a in ALGOS], axis=1).min(axis=1)
    i = jump.idxmax()
    return df.loc[i, 'Time(s)'] if jump[i] > KIDNAP_JUMP else nan


def recovery_time(times, pos_err, yaw_err, t0) -> Optional[float]:
    """Seconds from kidnap until error stays below RECOVER_THRESH for HOLD_TIME (NaN = never)."""
    if isnan(t0):
        return nan
    after = times >= t0
    times = times[after]
    ok = (pos_err[after] < RECOVER_POS_THRESH) & (yaw_err[after] < RECOVER_YAW_THRESH)
    for i in flatnonzero(ok):
        if times[i] + HOLD_TIME > times[-1]:
            break
        if ok[(times >= times[i]) & (times <= times[i] + HOLD_TIME)].all():
            return times[i] - t0
    return nan


def ci95(x):
    """Half-width of the 95% t-interval of the mean."""
    x = x.dropna()
    return sem(x) * t.ppf(0.975, len(x) - 1) if len(x) > 1 else nan


def summarize(d, by, col):
    return d.groupby(by, observed=True)[col].agg(mean='mean', ci=ci95, n='count').reset_index()


# ----------------------------------------------------------------------------- loading
def load(root=RESULTS_DIR):
    """One row per (run, algorithm)."""
    rows = []
    for csv in root.glob('particles_*/noise_*/*/run_*/*.csv'):
        m = PATH_RE.search(csv.as_posix())
        try:
            df = read_csv(csv) if m else DataFrame()
            if df.empty:
                raise ValueError('empty or unrecognized path')
            particles, noise, cond, seed = int(m[1]), float(m[2]), m[3], int(m[4])
            times = df['Time(s)'].to_numpy()
            t0 = kidnap_time(df) if cond == 'kidnapped' else nan
            if cond == 'kidnapped' and isnan(t0):
                print(f'Warning: no kidnap detected in {csv}')
            last = df.iloc[-1]
            for a in ALGOS:
                rec = recovery_time(
                    times, df[f'{a}_Pos_Err'].to_numpy(), df[f'{a}_Yaw_Error'].to_numpy(), t0
                )
                rows.append(
                    dict(
                        particles=particles,
                        noise=noise,
                        condition=cond,
                        seed=seed,
                        algo=a,
                        pos_rmse=last[f'{a}_Pos_RMSE'],
                        yaw_rmse=last[f'{a}_Yaw_RMSE'],
                        runtime_ms=df[f'{a}_Runtime'].mean() * 1e3,
                        recovery_s=rec,
                        recovered=nan if isnan(t0) else float(not isnan(rec)),
                    )
                )
        except Exception as e:
            print(f'Skipping {csv}: {e}')
    return DataFrame(rows)


def by_estimator(df):
    """One row per (estimator label, noise, condition, seed).

    EKF and DR ignore the particle count, so the same seed is run once per particle count.
    Those repeats are averaged per seed; pooling them would count each seed several times
    and make the CI falsely narrow.
    """
    pf = df[df.algo == 'PF'].assign(label=lambda d: 'PF-' + d.particles.astype(str))
    other = (
        df[df.algo != 'PF']
        .groupby(['algo', 'noise', 'condition', 'seed'], as_index=False)
        .mean(numeric_only=True)
        .assign(label=lambda d: d.algo)
    )
    order = ['EKF', 'DR'] + [f'PF-{p}' for p in sorted(df.particles.unique())]
    out = concat([other, pf], ignore_index=True)
    out['label'] = Categorical(out.label, categories=order, ordered=True)
    return out


# ----------------------------------------------------------------------------- plotting
def color(s, label):
    return COLORS[list(s.label.cat.categories).index(label) % len(COLORS)]


def lines(ax, d, x, col, ylabel):
    s = summarize(d, ['label', x], col)
    for label, g in s.groupby('label', observed=True):
        ax.errorbar(g[x], g['mean'], yerr=g['ci'], marker='o', label=label, color=color(s, label))
    ax.set_ylabel(ylabel)
    return s.assign(metric=col)


def bars(ax, d, col, ylabel):
    s = summarize(d, ['label'], col)
    ax.bar(s.label.astype(str), s['mean'], yerr=s['ci'], color=[color(s, l) for l in s.label])
    ax.set_ylabel(ylabel)
    ax.tick_params(axis='x', rotation=30)
    return s.assign(metric=col)


def save(fig, name, summaries):
    OUT_DIR.mkdir(exist_ok=True)
    fig.savefig(OUT_DIR / f'{name}.png')
    close(fig)
    table = concat(summaries)
    table.to_csv(OUT_DIR / f'{name}_summary.csv', index=False)
    print(f'\n=== {name} (mean, 95% CI half-width, n) ===\n{table.to_string(index=False)}')


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
    p = s.pivot(index=['rank', 'label'], columns=cols, values=['mean', 'ci']).droplevel('rank')
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


# ----------------------------------------------------------------------------- main
def main():
    raw = load()
    if raw.empty:
        raise SystemExit(f'No results found under {RESULTS_DIR}/')
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
    fig.suptitle(f'Accuracy, baseline, noise scale {REF_NOISE} (95% CI)')
    save(fig, 'accuracy', s)
    latex(
        concat(s),
        'accuracy',
        f'Baseline localization accuracy at odometry noise scale {REF_NOISE:g}.',
    )

    # Noise sensitivity
    fig, axs = subplots(1, 2, figsize=(11, 4), sharex=True)
    s = [
        lines(axs[0], base, 'noise', 'pos_rmse', 'Position RMSE (m)'),
        lines(axs[1], base, 'noise', 'yaw_rmse', 'Yaw RMSE (rad)'),
    ]
    for ax in axs:
        ax.set_xlabel(noise_x)
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

    # Kidnap recovery
    fig, axs = subplots(1, 3, figsize=(16, 4), sharex=True)
    s = [
        lines(axs[0], kid, 'noise', 'recovered', 'Recovery rate'),
        lines(axs[1], kid, 'noise', 'recovery_s', 'Time to recover (s), recovered runs'),
        lines(axs[2], kid, 'noise', 'pos_rmse', 'Position RMSE (m)'),
    ]
    axs[0].set_ylim(0, 1.05)
    for ax in axs:
        ax.set_xlabel(noise_x)
    axs[0].legend()
    fig.suptitle(
        f'Kidnapping recovery (< {RECOVER_POS_THRESH} m and < {RECOVER_YAW_THRESH} rad held for {HOLD_TIME} s, 95% CI)'
    )
    save(fig, 'recovery', s)
    crit = f'pos error $<$ {RECOVER_POS_THRESH} m and yaw error $<$ {RECOVER_YAW_THRESH} rad held for {HOLD_TIME:g} s'
    latex(
        s[0],
        'recovery_rate',
        f'Kidnapping recovery rate ({crit}).',
        cols='noise',
        head=noise_col,
        digits=2,
    )
    latex(
        s[1],
        'recovery_time',
        f'Time to recover (s) from kidnapping, recovered runs only ({crit}).',
        cols='noise',
        head=noise_col,
        digits=1,
    )
    latex(
        s[2],
        'recovery_rmse',
        'Position RMSE (m) over kidnapped runs.',
        cols='noise',
        head=noise_col,
    )

    # Runtime vs. particle count (baseline, pooled over noise levels)
    rt = raw[raw.condition == 'baseline'].assign(
        label=lambda d: d.algo, rank=lambda d: d.algo.map(ALGOS.index)
    )
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
        summarize(base, [], 'runtime_ms').assign(metric='runtime_ms'),
        'runtime',
        'Average runtime per update step across algorithms, baseline runs pooled over all '
        'noise levels.',
    )


if __name__ == '__main__':
    main()
