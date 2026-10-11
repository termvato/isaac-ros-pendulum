"""Plot recoveries from trajectory CSVs (harness traj_*.csv or rl/lab_eval.py output).

One figure per start angle: body angle, arm speed and torque against time, one line per
controller. Takes the trial nearest each requested angle and says so in the legend.

    python3 analysis/plot_traj.py --angles 3 5 7 9 --out results/figures \
        LQR=results/traj_lqr_0p025_nodamp.csv PPO=results/lab_ppo_base.csv
"""

import argparse
import csv
import os
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Categorical slots in fixed order (dataviz default palette, light surface).
COLOURS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#4a3aa7', '#e34948']
INK, MUTED, SURFACE = '#0b0b0b', '#52514e', '#fcfcfb'


def load(path):
    """Return {theta0: {column: [values]}} from one trajectory CSV."""
    trials = defaultdict(lambda: defaultdict(list))
    finished = set()  # the harness repeats bracket angles; keep the first trial only
    with open(path) as f:
        for row in csv.DictReader(f):
            theta0, t = float(row['theta0']), float(row['t'])
            trial = trials[theta0]
            if theta0 in finished or (trial['t'] and t < trial['t'][-1]):
                finished.add(theta0)
                continue
            for k in ('t', 'theta_deg', 'w_arm', 'effort'):
                trial[k].append(float(row[k]))
    return trials


def main():
    p = argparse.ArgumentParser()
    p.add_argument('runs', nargs='+', help='label=path.csv')
    p.add_argument('--angles', type=float, nargs='+', required=True)
    p.add_argument('--out', default='results/figures')
    p.add_argument('--tag', default='', help='suffix for the file names')
    p.add_argument('--no_torque', action='store_true',
                   help="drop the torque panel: a harness CSV's effort column is the last command the "
                        "harness received, which can lag what physics applied by tens of ms")
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)
    runs = [(r.rsplit('=', 1)[0], load(r.rsplit('=', 1)[1])) for r in args.runs]

    for angle in args.angles:
        keys = ('theta_deg', 'w_arm') if args.no_torque else ('theta_deg', 'w_arm', 'effort')
        ylabels = ('body angle [deg]', 'arm speed [rad/s]', 'arm torque [N.m]')[:len(keys)]
        fig, axes = plt.subplots(len(keys), 1, figsize=(8, 2.5 * len(keys)), sharex=True, facecolor=SURFACE)
        for colour, (label, trials) in zip(COLOURS, runs):
            theta0 = min(trials, key=lambda a: abs(a - angle))
            tr = trials[theta0]
            name = label if abs(theta0 - angle) < 1e-6 else f'{label} ({theta0:.2f} deg)'
            for ax, key in zip(axes, keys):
                ax.plot(tr['t'], tr[key], color=colour, lw=2, label=name)
        for ax, ylabel in zip(axes, ylabels):
            ax.set_facecolor(SURFACE)
            ax.set_ylabel(ylabel, color=MUTED)
            ax.grid(color='#e6e5e0', lw=0.8)
            ax.axhline(0, color=MUTED, lw=0.8)
            ax.tick_params(colors=MUTED)
            for side in ('top', 'right'):
                ax.spines[side].set_visible(False)
            for side in ('left', 'bottom'):
                ax.spines[side].set_color('#c3c2b7')
        axes[1].axhspan(-1, 1, color='#e6e5e0', label='despun (|w| < 1)')
        axes[0].legend(frameon=False, labelcolor=INK)
        axes[-1].set_xlabel('time [s]', color=MUTED)
        fig.suptitle(f'Recovery from {angle:g} deg lean', color=INK, x=0.07, ha='left')
        fig.tight_layout()
        path = os.path.join(args.out, f'recovery_{angle:g}deg{args.tag}.png')
        fig.savefig(path, dpi=130)
        plt.close(fig)
        print(f'wrote {path}')


if __name__ == '__main__':
    main()
