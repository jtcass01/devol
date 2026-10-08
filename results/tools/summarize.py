"""Accuracy and compute tables for results/README.md from the tc1/tc2 run dirs (summary.json, compute.csv, cpu.json)."""

import csv, json, sys
import numpy as np

out = sys.argv[1]
procs = {
    'ekf': ['ekf_localization'],
    'pf': ['pf_localization'],
    'hybrid': ['hybrid_ekf', 'pf_localization', 'hybrid_supervisor'],
}
res = {}
for tc in ('tc1', 'tc2'):
    s = json.load(open(f'{out}/{tc}/summary.json'))
    cpu = json.load(open(f'{out}/{tc}/cpu.json'))
    node_cpu = {}
    for e in cpu.values():
        c = e['cmd']
        if '__node:=' in c:
            name = c.split('__node:=')[1].split()[0]
            node_cpu[name] = 100 * e['cpu'] / max(e['last'] - e['first'], 1e-9)
    wall = max(e['last'] for e in cpu.values())
    busy = sum(e['cpu'] for e in cpu.values()) / wall
    comp = {}
    for r in csv.DictReader(open(f'{out}/{tc}/compute.csv')):
        comp.setdefault(r['estimator'], []).append((float(r['t']), float(r['compute_ms'])))
    t = [
        float(r['t'])
        for r in csv.DictReader(open(f'{out}/{tc}/trajectory.csv'))
        if r['estimator'] == 'ground_truth'
    ]
    print(
        f'{tc}: wall {wall:.0f} s, sim {max(t) - min(t):.0f} s, RTF {(max(t) - min(t)) / wall:.2f}, avg cores busy {busy:.2f}; nodes',
        {k: round(v, 1) for k, v in sorted(node_cpu.items(), key=lambda kv: -kv[1])[:12]},
    )
    for est in ('ekf', 'pf', 'hybrid'):
        e = s['estimators'][est]
        c = np.array(comp.get(est, [(0, np.nan)]))
        rate = len(c) / (c[-1, 0] - c[0, 0]) if len(c) > 1 else np.nan
        parts = {p: node_cpu.get(p) for p in procs[est]}
        res[(est, tc)] = dict(
            rmse=e['pos_rmse'],
            max=e['pos_max'],
            final=e['final_pos_err'],
            wp=e['waypoint_errors'],
            recovered=e['recovered'],
            rec_t=e['recovery_time'],
            ms_mean=float(np.mean(c[:, 1])),
            ms_p95=float(np.percentile(c[:, 1], 95)),
            rate=rate,
            cpu=parts,
            cpu_total=sum(v for v in parts.values() if v is not None),
        )
    dr = s['estimators']['dead_reckoning']
    res[('dead_reckoning', tc)] = dict(
        rmse=dr['pos_rmse'], max=dr['pos_max'], final=dr['final_pos_err']
    )
    print(tc, 'verdict:', open(f'{out}/{tc}/verdict.txt').read().strip().splitlines()[0])
json.dump({f'{k[0]}/{k[1]}': v for k, v in res.items()}, open(f'{out}/table.json', 'w'), indent=1)
for k, v in res.items():
    print(k, {a: (round(b, 3) if isinstance(b, float) else b) for a, b in v.items()})
