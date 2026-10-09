"""Plot learning curves from analysis/export_tb.py CSVs: reward, episode length, exploration noise.

    python3 analysis/plot_training.py results/training/*lqrrew*.csv --out results/figures/training_lqrrew.png
"""

import argparse
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

COLOURS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#4a3aa7', '#e34948']
INK, MUTED, SURFACE = '#0b0b0b', '#52514e', '#fcfcfb'

p = argparse.ArgumentParser()
p.add_argument('csvs', nargs='+')
p.add_argument('--out', required=True)
args = p.parse_args()

fig, axes = plt.subplots(3, 1, figsize=(8, 7.5), sharex=True, facecolor=SURFACE)
for colour, path in zip(COLOURS, args.csvs):
    rows = list(csv.DictReader(open(path)))
    it = [int(r['iteration']) for r in rows]
    label = os.path.basename(path)[:-4].split('_', 2)[-1]  # drop the date-time prefix
    for ax, key in zip(axes, ('mean_reward', 'mean_episode_length', 'noise_std')):
        ax.plot(it, [float(r[key]) for r in rows], color=colour, lw=1.5, label=label)
for ax, ylabel in zip(axes, ('mean reward / episode', 'episode length [steps]', 'exploration noise std')):
    ax.set_facecolor(SURFACE)
    ax.set_ylabel(ylabel, color=MUTED)
    ax.grid(color='#e6e5e0', lw=0.8)
    ax.tick_params(colors=MUTED)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
axes[1].axhline(1200, color=MUTED, lw=0.8, ls='--')  # full 10 s episode
axes[0].legend(frameon=False, labelcolor=INK)
axes[-1].set_xlabel('training iteration', color=MUTED)
fig.tight_layout()
os.makedirs(os.path.dirname(args.out), exist_ok=True)
fig.savefig(args.out, dpi=130)
print(f'wrote {args.out}')
