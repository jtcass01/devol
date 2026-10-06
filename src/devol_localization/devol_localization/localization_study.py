#!/usr/bin/env python3
"""Runs and summarizes the localization study's replay trials.

  ros2 run devol_localization localization_study run --bag ~/loc_bags/nominal --kidnap-bag ~/loc_bags/kidnap \
      --out ~/loc_results
  ros2 run devol_localization localization_study analyze --out ~/loc_results

`run` replays the recorded bags through localization_replay.launch.py once per configuration and
seed: the one-factor-at-a-time sweep about the nominal point (N = 2000, k = 1, sigma_r = 0.03 m)
over N in {100, 500, 2000, 5000}, k in {0, 1, 2, 4} and sigma_r in {0.01, 0.03, 0.10}, then the
global-localization and kidnapping trials at the nominal point. Runs that vary only N skip the EKF,
whose result would repeat the nominal one. Trials with a summary.json are skipped, so an
interrupted study resumes where it stopped.

`analyze` reads every summary.json and writes study_summary.csv / .md (mean +/- 95% CI over seeds,
recovery rates with Clopper-Pearson intervals) and the sensitivity figures.
"""

import argparse
import csv
import json
import subprocess
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List

import numpy as np

from devol_localization.metrics import (
    RECOVERY_DWELL,
    RECOVERY_THRESHOLD,
    clopper_pearson,
    mean_ci95,
    rescore_recovery,
)

__author__ = 'Jacob Taylor Cassady'
__email__ = 'jcassad1@jh.edu'

NOMINAL = {'num_particles': 2000, 'k': 1.0, 'sigma_r': 0.03}
GRID = {
    'num_particles': [100, 500, 2000, 5000],
    'k': [0.0, 1.0, 2.0, 4.0],
    'sigma_r': [0.01, 0.03, 0.10],
}


def config_name(c: dict) -> str:
    return f'N{c["num_particles"]}_k{c["k"]:g}_s{c["sigma_r"]:g}'


def study_configs() -> List[dict]:
    """(scenario, config, estimators) for every distinct run of the protocol."""
    runs, seen = [], set()
    for factor, values in GRID.items():
        for v in values:
            c = dict(NOMINAL, **{factor: v})
            key = config_name(c)
            if key in seen:
                continue
            seen.add(key)
            estimators = 'pf' if c['num_particles'] != NOMINAL['num_particles'] else 'ekf,pf'
            runs.append({'scenario': 'nominal', **c, 'estimators': estimators})
    runs.append({'scenario': 'global', **NOMINAL, 'estimators': 'ekf,pf'})
    runs.append({'scenario': 'kidnap', **NOMINAL, 'estimators': 'ekf,pf'})
    return runs


def run(args) -> int:
    out = Path(args.out).expanduser()
    bags = {'nominal': args.bag, 'global': args.bag, 'kidnap': args.kidnap_bag}
    todo = [
        r
        for r in study_configs()
        if args.scenarios == 'all' or r['scenario'] in args.scenarios.split(',')
    ]
    if args.configs:
        todo = [r for r in todo if config_name(r) in args.configs.split(',')]
    for scenario in {r['scenario'] for r in todo}:
        bag = bags[scenario]
        d = Path(bag).expanduser() if bag else None
        if d and not (d / 'metadata.yaml').is_file() and any(d.glob('*.mcap')):
            print(
                f'{bag} has no metadata.yaml (recorder stopped before closing); reindexing',
                flush=True,
            )
            subprocess.run(['ros2', 'bag', 'reindex', str(d), '-s', 'mcap'], check=False)
        if d and not (d / 'metadata.yaml').is_file():
            print(
                f'{bag} is not a rosbag2 directory (no metadata.yaml); not running any trials',
                flush=True,
            )
            return 2
    failures = 0
    for r in todo:
        bag = bags[r['scenario']]
        if not bag:
            print(f'skipping {r["scenario"]} trials: no bag given', flush=True)
            continue
        for seed in range(args.first_seed, args.first_seed + args.trials):
            trial = out / r['scenario'] / config_name(r) / f'seed{seed:02d}'
            if (trial / 'summary.json').exists() and not args.force:
                continue
            cmd = [
                'ros2',
                'launch',
                'devol_localization',
                'localization_replay.launch.py',
                f'bag:={Path(bag).expanduser()}',
                f'scenario:={r["scenario"]}',
                f'estimators:={r["estimators"]}',
                f'k:={r["k"]}',
                f'sigma_r:={r["sigma_r"]}',
                f'num_particles:={r["num_particles"]}',
                f'seed:={seed}',
                f'output_dir:={trial}',
                f'rate:={args.rate}',
                f'maze:={args.maze}',
                'viz:=false',
            ]
            print(f'[{r["scenario"]} {config_name(r)} seed {seed}]', ' '.join(cmd), flush=True)
            try:
                subprocess.run(
                    cmd,
                    timeout=args.timeout,
                    check=False,
                    stdout=None if args.verbose else subprocess.DEVNULL,
                )
            except subprocess.TimeoutExpired:
                print('  timed out', flush=True)
            if not (trial / 'summary.json').exists():
                failures += 1
                print('  no summary.json written', flush=True)
                if failures == 1 and seed == args.first_seed and r is todo[0]:
                    print(
                        'The first trial produced no results; stopping. Rerun with --verbose to see why.',
                        flush=True,
                    )
                    return 1
    analyze(args)
    return 1 if failures else 0


def load_rows(
    out: Path, recovery_timeout: float = 60.0, recovery_dwell: float = RECOVERY_DWELL
) -> List[dict]:
    """Per-trial rows. Recovery is re-scored from trajectory.csv with the current rule (threshold held
    for recovery_dwell s), so older trials need only re-analysis, and dead reckoning is never scored."""
    rows = []
    for path in sorted(out.glob('*/*/seed*/summary.json')):
        body = json.loads(path.read_text())
        c = body['config']
        for est, m in body['estimators'].items():
            # The EKF and dead reckoning do not depend on N: count them at the nominal N only.
            if est != 'pf' and c.get('num_particles') != NOMINAL['num_particles']:
                continue
            if est == 'dead_reckoning':
                m = {**m, 'recovered': None, 'recovery_time': None}
            elif m.get('event_time') is not None and (path.parent / 'trajectory.csv').is_file():
                rec = rescore_recovery(
                    path.parent / 'trajectory.csv',
                    est,
                    m['event_time'],
                    RECOVERY_THRESHOLD,
                    recovery_timeout,
                    recovery_dwell,
                )
                m = {**m, 'recovered': rec is not None, 'recovery_time': rec}
            rows.append(
                {
                    'scenario': c['scenario'],
                    'num_particles': c['num_particles'],
                    'k': c['k'],
                    'sigma_r': c['sigma_r'],
                    'seed': c['seed'],
                    'estimator': est,
                    **m,
                }
            )
    return rows


def summarize(rows: List[dict]) -> List[dict]:
    groups: Dict[tuple, List[dict]] = defaultdict(list)
    for r in rows:
        groups[(r['scenario'], r['estimator'], r['num_particles'], r['k'], r['sigma_r'])].append(r)
    table = []
    for (scenario, est, n, k, s), g in sorted(groups.items()):
        row = {
            'scenario': scenario,
            'estimator': est,
            'num_particles': n if est == 'pf' else '',
            'k': k,
            'sigma_r': s,
            'trials': len(g),
        }
        for metric, scale in (
            ('pos_rmse', 1.0),
            ('yaw_rmse', 180.0 / np.pi),
            ('compute_ms_mean', 1.0),
        ):
            mean, half, _ = mean_ci95(
                [None if x[metric] is None else x[metric] * scale for x in g]
            )
            row[metric] = mean
            row[metric + '_ci95'] = half
        wp = [
            np.nanmean([e if e is not None else np.nan for e in x['waypoint_errors']])
            if x['waypoint_errors']
            else np.nan
            for x in g
        ]
        row['waypoint_err'], row['waypoint_err_ci95'], _ = mean_ci95(wp)
        if scenario != 'nominal' and est != 'dead_reckoning':
            ok = sum(1 for x in g if x['recovered'])
            lo, hi = clopper_pearson(ok, len(g))
            row.update({'recovered': ok, 'recovery_rate_lo95': lo, 'recovery_rate_hi95': hi})
            row['recovery_time'], row['recovery_time_ci95'], _ = mean_ci95(
                [x['recovery_time'] for x in g if x['recovered']]
            )
        table.append(row)
    return table


def fmt(mean, half, digits=3) -> str:
    if mean is None or not np.isfinite(mean):
        return '–'
    return f'{mean:.{digits}f}' + (
        '' if half is None or not np.isfinite(half) else f' ± {half:.{digits}f}'
    )


def write_tables(out: Path, table: List[dict]) -> None:
    keys = sorted(
        {k for r in table for k in r},
        key=lambda k: list(table[0]).index(k) if k in table[0] else 99,
    )
    with open(out / 'study_summary.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(table)
    lines = [
        '| scenario | estimator | N | k | σr (m) | trials | pos RMSE (m) | heading RMSE (°) | waypoint err (m) '
        '| compute (ms) | recovered | recovery time (s) |',
        '|' + '---|' * 12,
    ]
    for r in table:
        rec = (
            ''
            if 'recovered' not in r
            else (
                f'{r["recovered"]}/{r["trials"]} '
                f'[{r["recovery_rate_lo95"]:.2f}, {r["recovery_rate_hi95"]:.2f}]'
            )
        )
        lines.append(
            f'| {r["scenario"]} | {r["estimator"]} | {r["num_particles"]} | {r["k"]:g} | {r["sigma_r"]:g} '
            f'| {r["trials"]} | {fmt(r["pos_rmse"], r["pos_rmse_ci95"])} '
            f'| {fmt(r["yaw_rmse"], r["yaw_rmse_ci95"], 2)} '
            f'| {fmt(r["waypoint_err"], r["waypoint_err_ci95"])} '
            f'| {fmt(r["compute_ms_mean"], r["compute_ms_mean_ci95"], 2)} | {rec} '
            f'| {fmt(r.get("recovery_time"), r.get("recovery_time_ci95"), 1) if rec else ""} |'
        )
    (out / 'study_summary.md').write_text('\n'.join(lines) + '\n')


def plot_sensitivity(out: Path, table: List[dict]) -> None:
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    colors = {'ekf': '#2a78d6', 'pf': '#eb6834', 'dead_reckoning': '#52514e'}
    nominal = [r for r in table if r['scenario'] == 'nominal']

    def series(est, factor):
        pts = [
            r
            for r in nominal
            if r['estimator'] == est
            and all(r[f] == NOMINAL[f] for f in ('k', 'sigma_r') if f != factor)
            and (
                est != 'pf'
                or factor == 'num_particles'
                or r['num_particles'] == NOMINAL['num_particles']
            )
        ]
        pts.sort(key=lambda r: r[factor] if factor != 'num_particles' or est == 'pf' else 0)
        return pts

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2), facecolor='white')
    for ax, factor, label in zip(
        axes,
        ('num_particles', 'k', 'sigma_r'),
        ('particle count N', 'odometry noise scale k', 'lidar range noise σr (m)'),
    ):
        for est in ('dead_reckoning', 'ekf', 'pf'):
            pts = series(est, factor)
            if not pts:
                continue
            if factor == 'num_particles' and est != 'pf':
                if est == 'ekf':
                    ax.axhline(
                        pts[0]['pos_rmse'],
                        color=colors[est],
                        lw=1.5,
                        ls='--',
                        label='EKF (nominal)',
                    )
                continue
            x = [p[factor] for p in pts]
            ax.errorbar(
                x,
                [p['pos_rmse'] for p in pts],
                yerr=[np.nan_to_num(p['pos_rmse_ci95']) for p in pts],
                color=colors[est],
                lw=2,
                marker='o',
                ms=8,
                capsize=3,
                label=est.replace('_', ' '),
            )
        if factor == 'num_particles':
            ax.set_xscale('log')
        ax.set_xlabel(label)
        ax.set_ylabel('position RMSE (m)')
        ax.grid(True, color='#e4e3df')
        ax.legend(fontsize=8)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
    fig.suptitle(
        'Position RMSE, mean ± 95% CI over trials (one factor varied about N=2000, k=1, σr=0.03 m)',
        x=0.01,
        ha='left',
    )
    fig.tight_layout()
    fig.savefig(out / 'sensitivity_pos_rmse.png', dpi=120)
    plt.close(fig)


def analyze(args) -> int:
    out = Path(args.out).expanduser()
    rows = load_rows(
        out,
        getattr(args, 'recovery_timeout', 60.0),
        getattr(args, 'recovery_dwell', RECOVERY_DWELL),
    )
    if not rows:
        print(f'no summary.json under {out}', flush=True)
        return 1
    table = summarize(rows)
    write_tables(out, table)
    plot_sensitivity(out, table)
    print((out / 'study_summary.md').read_text(), flush=True)
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = p.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run', help='replay the trials, then analyze')
    r.add_argument(
        '--bag', required=True, help='nominal-route bag (also used for global localization)'
    )
    r.add_argument('--kidnap-bag', default='', help='bag recorded with scenario:=kidnap')
    r.add_argument('--out', required=True)
    r.add_argument('--trials', type=int, default=20)
    r.add_argument('--first-seed', type=int, default=0)
    r.add_argument('--rate', type=float, default=1.0)
    r.add_argument('--maze', default='factory')
    r.add_argument(
        '--scenarios', default='all', help='all, or a comma list of nominal,global,kidnap'
    )
    r.add_argument(
        '--configs',
        default='',
        help='comma list of configuration names to run, e.g. N2000_k1_s0.03',
    )
    r.add_argument('--timeout', type=float, default=900.0, help='seconds per trial')
    r.add_argument(
        '--force', action='store_true', help='re-run trials that already have a summary'
    )
    r.add_argument('--verbose', action='store_true')
    a = sub.add_parser('analyze', help='summarize existing trials')
    a.add_argument('--out', required=True)
    for q in (r, a):
        q.add_argument(
            '--recovery-timeout',
            type=float,
            default=60.0,
            help='s after the event (as the scorer)',
        )
        q.add_argument(
            '--recovery-dwell',
            type=float,
            default=RECOVERY_DWELL,
            help='s the error must stay under 0.25 m at the end of a trial to count as recovered',
        )
    args = p.parse_args(argv)
    return run(args) if args.cmd == 'run' else analyze(args)


if __name__ == '__main__':
    sys.exit(main())
