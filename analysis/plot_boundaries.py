"""Bisected boundaries before and after the harness fix, one row per controller, against the
physical limit. Each CSV is an isaac/harness.py bisect_*.csv; the boundary is the last
recovered / first failed lean of the bisection proper (sanity pair and repeats excluded).

    python3 analysis/plot_boundaries.py --out results/figures/boundaries_harness_fix.png \
        "LQR=results/bisect_lqr_0p025_nodamp.csv:results/bisect_hold_lqr_0p025.csv" ...
"""

import argparse
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

INK, MUTED, SURFACE, GRID = '#0b0b0b', '#52514e', '#fcfcfb', '#e6e5e0'
OLD, NEW = '#b0afa8', '#2a78d6'
LIMIT = 7.47  # deg, analysis/recovery_limit.py (7.470 with --despin)


def boundary(path):
    """(highest recovered, lowest failed) over the bisection rows."""
    rows = [r for r in csv.DictReader(open(path)) if r['note'] == 'bisect']
    fails = [float(r['theta0']) for r in rows if r['recovered'] != 'True']
    hi = min(fails)
    lo = max(float(r['theta0']) for r in rows if r['recovered'] == 'True' and float(r['theta0']) < hi)
    return lo, hi


p = argparse.ArgumentParser()
p.add_argument('rows', nargs='+', help='label=old.csv:fixed.csv')
p.add_argument('--out', required=True)
args = p.parse_args()

fig, ax = plt.subplots(figsize=(8, 0.7 * len(args.rows) + 1.6), facecolor=SURFACE)
for y, row in enumerate(reversed(args.rows)):
    label, paths = row.rsplit('=', 1)
    (old, _), (new, _) = (boundary(q) for q in paths.split(':'))
    ax.plot([old, new], [y, y], color=GRID, lw=3, zorder=1)
    ax.scatter([old], [y], s=70, color=OLD, edgecolor=SURFACE, lw=2, zorder=3)
    ax.scatter([new], [y], s=70, color=NEW, edgecolor=SURFACE, lw=2, zorder=3)
    ax.text(old - 0.04, y, f'{old:.2f}', color=MUTED, ha='right', va='center', fontsize=9)
    ax.text(new, y + 0.28, f'{new:.2f}', color=INK, ha='center', va='center', fontsize=9, zorder=4,
            bbox=dict(facecolor=SURFACE, edgecolor='none', pad=1))
    print(f'{label}: {old:.3f} -> {new:.3f}')
ax.axvline(LIMIT, color=INK, lw=1, ls='--')
ax.text(LIMIT - 0.03, len(args.rows) - 0.45, f'physical limit {LIMIT}', color=MUTED, ha='right', fontsize=9)
ax.scatter([], [], s=70, color=OLD, label='old harness (previous trial\'s command leaks in)')
ax.scatter([], [], s=70, color=NEW, label='fixed harness (held at the lean, then released)')
ax.legend(frameon=False, labelcolor=INK, loc='lower left', fontsize=9)
ax.set_yticks(range(len(args.rows)), [r.rsplit('=', 1)[0] for r in reversed(args.rows)], color=INK)
ax.set_ylim(-1.3, len(args.rows) - 0.2)
ax.set_xlim(5.9, 7.75)
ax.set_xlabel('largest lean recovered through ROS [deg]', color=MUTED)
ax.set_facecolor(SURFACE)
ax.grid(axis='x', color=GRID, lw=0.8)
ax.tick_params(colors=MUTED)
for side in ('top', 'right', 'left'):
    ax.spines[side].set_visible(False)
fig.tight_layout()
os.makedirs(os.path.dirname(args.out), exist_ok=True)
fig.savefig(args.out, dpi=130)
print(f'wrote {args.out}')
