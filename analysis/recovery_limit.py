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

from plant_model import load

p = argparse.ArgumentParser()
p.add_argument('--urdf', default='urdf/body.urdf')
p.add_argument('--max_torque', type=float, default=0.025)
p.add_argument('--no_load_speed', type=float, default=None,
               help='DC motor line: available torque runs from +-max_torque at rest to 0 at this '
                    'speed when driving, and up to 2x when braking against the spin')
p.add_argument('--max_arm_speed', type=float, default=100.0)
p.add_argument('--horizon', type=float, default=2.0, help='seconds to come to rest (much past 4 s the matrix powers overflow)')
p.add_argument('--arm0', type=float, default=0.0, help='arm speed at the start (pre-spun), rad/s')
p.add_argument('--despin', action='store_true', help='arm must also end at 0 rad/s')
p.add_argument('--dt', type=float, default=1 / 120)
p.add_argument('--out', help='write the optimal trajectory as a trajectory CSV (plot_traj.py format)')
args = p.parse_args()
A, B, _ = load(args.urdf)

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

x_free = np.array([powers[k] @ np.array([0.0, 0.0, args.arm0]) for k in range(N + 1)])  # from arm0
arm = rows[1:, 2, :]  # arm speed at steps 1..N
A_ub = [arm, -arm]
b_ub = [np.full(N, args.max_arm_speed) - x_free[1:, 2], np.full(N, args.max_arm_speed) + x_free[1:, 2]]
if args.no_load_speed:  # u_k + (stall / no_load) * w_k <= stall, and the mirror image
    line = rows[:N, 2, :] * (args.max_torque / args.no_load_speed)
    line[np.arange(N), np.arange(N)] += 1.0
    A_ub += [line, -line]
    c = x_free[:N, 2] * (args.max_torque / args.no_load_speed)
    b_ub += [args.max_torque - c, args.max_torque + c]
A_ub, b_ub = np.vstack(A_ub), np.concatenate(b_ub)
end = [0, 1, 2] if args.despin else [0, 1]
A_eq = rows[N, end, :]
b_eq = -x_free[N, end]
c = np.zeros(N + 1)
c[N] = -1.0  # maximise theta0
# With the motor line, the line itself bounds the torque (up to 2x stall when braking against the
# spin); a box at 1x stall here would silently remove plugging (it did until 2026-10-11).
box = 2 * args.max_torque if args.no_load_speed else args.max_torque
bounds = [(-box, box)] * N + [(0, None)]
res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method='highs')
if not res.success:
    raise SystemExit(res.message)
theta0 = res.x[N]
x = rows @ res.x + x_free
print(f'max recoverable lean: {np.degrees(theta0):.2f} deg ({args.urdf}, '
      f'torque {args.max_torque} N.m, arm {args.max_arm_speed} rad/s, {args.horizon} s, '
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
