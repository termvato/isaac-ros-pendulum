"""Per-seed summary from rl/lab_eval.py CSVs: largest lean held (body upright at the end) and
where the arm parks, one dot per seed, one column per training setup.

    python3 analysis/plot_seeds.py --out results/figures/seeds_r12_vs_r8.png \
        "resets 12 deg=results/lab_ppo_lqrrew_s?.csv" "resets 8 deg=results/lab_ppo_r8_s*.csv"
"""

import argparse
import csv
import glob
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

COLOURS = ['#2a78d6', '#eb6834']
INK, MUTED, SURFACE = '#0b0b0b', '#52514e', '#fcfcfb'
LQR, LIMIT = 7.0, 7.48  # deg: harness bisection, analysis/recovery_limit.py


def summarise(path):
    """(largest theta0 held, final arm speed there); a start counts as held if its record runs to
    the end of the episode (lab_eval stops recording at the first fall) with the body upright."""
    last = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            last[float(row['theta0'])] = row
    held = [a for a, r in last.items() if float(r['t']) > 9.9 and abs(float(r['theta_deg'])) < 0.5]
    if not held:
        return 0.0, None
    a = max(held)
    return a, float(last[a]['w_arm'])


p = argparse.ArgumentParser()
p.add_argument('groups', nargs='+', help='label=glob')
p.add_argument('--out', required=True)
args = p.parse_args()

fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(9, 4.5), facecolor=SURFACE)
for i, group in enumerate(args.groups):
    label, pattern = group.rsplit('=', 1)
    paths = sorted(glob.glob(pattern), key=lambda s: int(re.findall(r'_s(\d+)\.csv$', s)[0]))
    pts = [summarise(p) for p in paths]
    xs = [i + (j - (len(pts) - 1) / 2) * 0.04 for j in range(len(pts))]
    ax0.scatter(xs, [a for a, _ in pts], s=50, color=COLOURS[i], edgecolor=SURFACE, lw=1.5, zorder=3)
    ax1.scatter([x for x, (_, w) in zip(xs, pts) if w is not None],
                [w for _, w in pts if w is not None], s=50, color=COLOURS[i], edgecolor=SURFACE, lw=1.5, zorder=3)
    print(f'{label}: ' + ', '.join(f'{a:g}/{w if w is None else round(w, 1)}' for a, w in pts))
ax0.axhline(LIMIT, color=INK, lw=1, ls='--')
ax0.text(1.45, LIMIT + 0.08, f'physical limit {LIMIT}', color=MUTED, ha='right', fontsize=9)
ax0.axhline(LQR, color=MUTED, lw=1)
ax0.text(1.45, LQR - 0.25, f'LQR {LQR}', color=MUTED, ha='right', fontsize=9)
ax1.axhspan(-1, 1, color='#e6e5e0')
ax1.text(1.45, 1.5, 'despun', color=MUTED, ha='right', fontsize=9)
labels = [g.rsplit('=', 1)[0] for g in args.groups]
for ax, title in ((ax0, 'largest lean held [deg] (0 = never)'), (ax1, 'arm speed at 10 s [rad/s]')):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, fontsize=10, loc='left')
    ax.set_xticks(range(len(labels)), labels, color=MUTED)
    ax.set_xlim(-0.5, len(labels) - 0.5)
    ax.grid(axis='y', color='#e6e5e0', lw=0.8)
    ax.tick_params(colors=MUTED)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
ax1.axhline(0, color=MUTED, lw=0.8)
fig.tight_layout()
os.makedirs(os.path.dirname(args.out), exist_ok=True)
fig.savefig(args.out, dpi=130)
print(f'wrote {args.out}')
