"""Plot pre-spun recoveries from rl/mrv_bench.py: body angle, arm speed and motor torque.

Each trace stops at its first pass beyond 45 deg (the env auto-resets after that, so the rest of
the trace is a different start).

    python3 analysis/plot_mrv_bench.py results/mrv_bench.csv --angles 5.5 5.75 12 \
        --out results/figures/mrv_bench_prespun.png
"""

import argparse
import csv
import os
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

COLOURS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']
INK, MUTED, SURFACE, GRID = '#0b0b0b', '#52514e', '#fcfcfb', '#e6e5e0'

p = argparse.ArgumentParser()
p.add_argument('csv')
p.add_argument('--angles', type=float, nargs='+', required=True)
p.add_argument('--stall', type=float, default=0.0653)
p.add_argument('--out', required=True)
args = p.parse_args()

traces = defaultdict(lambda: defaultdict(list))
with open(args.csv) as f:
    for r in csv.DictReader(f):
        if float(r['arm0']) >= 0:  # pre-spun starts only
            continue
        tr = traces[float(r['theta0'])]
        if tr['done']:
            continue
        for k in ('t', 'theta_deg', 'w_arm', 'effort'):
            tr[k].append(float(r[k]))
        if abs(float(r['theta_deg'])) > 45.0:
            tr['done'].append(True)

fig, axes = plt.subplots(3, 1, figsize=(8, 7.5), sharex=True, facecolor=SURFACE)
for colour, angle in zip(COLOURS, args.angles):
    theta0 = min(traces, key=lambda a: abs(a - angle))  # label the start actually plotted
    tr = traces[theta0]
    for ax, key in zip(axes, ('theta_deg', 'w_arm', 'effort')):
        ax.plot(tr['t'], tr[key], color=colour, lw=2, label=f'{theta0:g} deg')
for ax, ylabel in zip(axes, ('body angle [deg]', 'arm speed [rad/s]', 'motor torque [N.m]')):
    ax.set_facecolor(SURFACE)
    ax.set_ylabel(ylabel, color=MUTED)
    ax.grid(color=GRID, lw=0.8)
    ax.axhline(0, color=MUTED, lw=0.8)
    ax.tick_params(colors=MUTED)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
axes[0].set_ylim(-10, 46)
axes[2].axhline(args.stall, color=MUTED, lw=1, ls='--')
axes[2].text(0.0, args.stall * 1.05, 'stall', color=MUTED, fontsize=9, va='bottom')
axes[0].legend(frameon=False, labelcolor=INK)
axes[-1].set_xlabel('time [s]', color=MUTED)
fig.suptitle('MRV model: arm pre-spun to -58.7 rad/s, then full voltage the other way',
             color=INK, x=0.07, ha='left')
fig.tight_layout()
os.makedirs(os.path.dirname(args.out), exist_ok=True)
fig.savefig(args.out, dpi=130)
print(f'wrote {args.out}')
