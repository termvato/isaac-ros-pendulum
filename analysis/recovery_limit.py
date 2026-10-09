"""Largest lean ANY controller could recover from, on the linearised plant, as a linear program.

Same A and B as analysis/lqr_design.py. Torque is held for each 1/120 s control step (as on the
real loop), |torque| <= max_torque and |arm speed| <= 100 rad/s at every step. Start from rest at
theta0, and maximise theta0 such that the body ends upright and still. With --despin the arm must
also end at 0. The plant is linear in theta0 and the torques, so this is exact for the linear model
and an LP solver finds the true optimum (no seeds, no training).

Not in the bound: sin(theta) vs theta (~0.3% at 8 deg) and what PhysX does at the speed limit.

    python3 analysis/recovery_limit.py [--max_torque 0.025] [--horizon 2] [--despin]
"""

import argparse

import numpy as np
from scipy.linalg import expm
from scipy.optimize import linprog

A = np.array([[0.0, 1.0, 0.0],
              [40.9, 0.0, 0.0],
              [-40.9, 0.0, 0.0]])
B = np.array([0.0, -556.0, 53187.0])

p = argparse.ArgumentParser()
p.add_argument('--max_torque', type=float, default=0.025)
p.add_argument('--max_arm_speed', type=float, default=100.0)
p.add_argument('--horizon', type=float, default=2.0, help='seconds to come to rest (much past 4 s the matrix powers overflow)')
p.add_argument('--despin', action='store_true', help='arm must also end at 0 rad/s')
p.add_argument('--dt', type=float, default=1 / 120)
p.add_argument('--out', help='write the optimal trajectory as a trajectory CSV (plot_traj.py format)')
args = p.parse_args()

# Zero-order-hold discretisation: x[k+1] = Ad x[k] + Bd u[k].
M = expm(np.block([[A, B[:, None]], [np.zeros((1, 4))]]) * args.dt)
Ad, Bd = M[:3, :3], M[:3, 3]
N = round(args.horizon / args.dt)

# x[k] = Ad^k e0 theta0 + sum_j Ad^(k-1-j) Bd u[j]. Decision vector z = [u_0..u_{N-1}, theta0].
e0 = np.array([1.0, 0.0, 0.0])
rows = np.zeros((N + 1, 3, N + 1))  # rows[k] maps z to x[k]
powers = [np.eye(3)]
for _ in range(N):
    powers.append(Ad @ powers[-1])
for k in range(N + 1):
    rows[k, :, N] = powers[k] @ e0
    for j in range(k):
        rows[k, :, j] = powers[k - 1 - j] @ Bd

arm = rows[1:, 2, :]  # arm speed at steps 1..N
A_ub = np.vstack([arm, -arm])
b_ub = np.full(2 * N, args.max_arm_speed)
end = [0, 1, 2] if args.despin else [0, 1]
A_eq = rows[N, end, :]
b_eq = np.zeros(len(end))
c = np.zeros(N + 1)
c[N] = -1.0  # maximise theta0
bounds = [(-args.max_torque, args.max_torque)] * N + [(0, None)]
res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method='highs')
if not res.success:
    raise SystemExit(res.message)
theta0 = res.x[N]
x = rows @ res.x
print(f'max recoverable lean: {np.degrees(theta0):.2f} deg '
      f'(torque {args.max_torque} N.m, arm {args.max_arm_speed} rad/s, {args.horizon} s, '
      f'{"despun" if args.despin else "arm free"}); peak |arm| {abs(x[:, 2]).max():.1f} rad/s, '
      f'final arm {x[-1, 2]:+.1f} rad/s')
if args.out:
    import csv
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['theta0', 't', 'theta_deg', 'w_base', 'w_arm', 'effort'])
        u = list(res.x[:N]) + [0.0]
        for k in range(N + 1):
            w.writerow([round(np.degrees(theta0), 2), k * args.dt, np.degrees(x[k, 0]), x[k, 1], x[k, 2], u[k]])
    print(f'wrote {args.out}')
