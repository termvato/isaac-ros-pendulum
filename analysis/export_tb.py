"""Dump TensorBoard scalars from rsl_rl runs to CSV, so plots need no TensorBoard.

Run with Isaac Lab's Python (it has tensorboard):
    ~/IsaacLab/isaaclab.sh -p analysis/export_tb.py logs/rsl_rl/pendulum_direct/<run> ... --out results/training
"""

import argparse
import csv
import os

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

TAGS = ["Train/mean_reward", "Train/mean_episode_length", "Policy/mean_noise_std"]

p = argparse.ArgumentParser()
p.add_argument("runs", nargs="+")
p.add_argument("--out", default="results/training")
args = p.parse_args()
os.makedirs(args.out, exist_ok=True)
for run in args.runs:
    acc = EventAccumulator(run)
    acc.Reload()
    series = {t: {s.step: s.value for s in acc.Scalars(t)} for t in TAGS}
    path = os.path.join(args.out, os.path.basename(run.rstrip("/")) + ".csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["iteration", "mean_reward", "mean_episode_length", "noise_std"])
        for it in sorted(series[TAGS[0]]):
            w.writerow([it] + [series[t].get(it) for t in TAGS])
    print(f"wrote {path}")
