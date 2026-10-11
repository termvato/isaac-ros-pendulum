"""Re-enact the MRV bench test in Isaac Lab: hold at a lean, arm pre-spun to full speed one way,
release, command full torque the other way, open loop. Does the body get back to upright?

Independent of analysis/recovery_limit.py (full nonlinear PhysX, no linearisation, no solver).
Motor model on the plant side, as on hardware: available torque follows the DC motor line,
+-stall at rest falling to 0 at no-load speed when driving, up to 2x stall when braking.

    PYTHONPATH=. ~/IsaacLab/isaaclab.sh -p rl/mrv_bench.py --headless --out results/mrv_bench.csv
"""

import argparse
import functools
print = functools.partial(print, flush=True)  # app.close() exits hard and drops buffered stdout

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--urdf", default="urdf/mrv.urdf")
parser.add_argument("--stall", type=float, default=0.0653, help="stall torque at the arm, N.m (8 V)")
parser.add_argument("--no_load", type=float, default=58.7, help="no-load arm speed, rad/s (8 V)")
parser.add_argument("--angles", type=float, nargs="+",
                    default=[a / 4 for a in range(4, 57)], help="start leans, deg (default 1 to 14)")
parser.add_argument("--seconds", type=float, default=3.0)
parser.add_argument("--out", required=True)
parser.add_argument("--rate", type=float, default=0.0,
                    help="with a GUI: sim speed relative to real time (0.25 = 4x slow motion); 0 = flat out")
parser.add_argument("--spacing", type=float, default=2.0, help="m between robots")
parser.add_argument("--hold", action="store_true", help="keep the window open after the run")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import csv  # noqa: E402
import math  # noqa: E402
import os  # noqa: E402
import time  # noqa: E402

import torch  # noqa: E402

from rl.pendulum_env import PendulumEnv, PendulumEnvCfg  # noqa: E402

# Two starts per angle: arm at rest, and arm pre-spun to -no_load (the bench test).
starts = [(a, 0.0) for a in args.angles] + [(a, -args.no_load) for a in args.angles]

cfg = PendulumEnvCfg()
cfg.robot_cfg = cfg.robot_cfg.replace(spawn=cfg.robot_cfg.spawn.replace(asset_path=os.path.abspath(args.urdf)))
cfg.robot_cfg.actuators["arm"].effort_limit_sim = 1.0  # the motor line below is the limit, not PhysX
cfg.robot_cfg.actuators["arm"].velocity_limit_sim = 1.0e3  # nor the URDF speed cap: no double counting
cfg.scene.num_envs = len(starts)
cfg.scene.env_spacing = args.spacing
cfg.viewer.eye = (0.0, -2.5, 0.5)  # looking along the pivot axis, so the lean is in view
cfg.viewer.lookat = (0.0, 0.0, 0.2)
cfg.max_theta = math.radians(80.0)  # never auto-reset inside the run
cfg.action_scale = 1.0  # actions are torques in N.m (the env clips to +-1, far above the motor)
env = PendulumEnv(cfg)

env.reset()
base, arm = env._base_dof_idx[0], env._arm_dof_idx[0]
pos = torch.zeros_like(env.pendulum.data.joint_pos)
vel = torch.zeros_like(pos)
pos[:, base] = torch.tensor([math.radians(a) for a, _ in starts], device=env.device)
vel[:, arm] = torch.tensor([w for _, w in starts], device=env.device)
env.pendulum.write_joint_state_to_sim(pos, vel)
env.joint_pos, env.joint_vel = pos.clone(), vel.clone()
obs = env._get_observations()["policy"]

for (a, w0), o in zip(starts, env.scene.env_origins.tolist()):
    print(f"robot at x={o[0]:+.2f} y={o[1]:+.2f}: lean {a:g} deg, arm start {w0:+.1f} rad/s")
if args.rate:
    t0 = time.time()
    while time.time() - t0 < 5.0:  # a moment to find the robots before they move
        app.update()

slope = args.stall / args.no_load
rows = []
t_start = time.time()
for k in range(int(args.seconds * 120)):
    if args.rate:  # pace the loop so the viewer shows it at the requested speed
        time.sleep(max(0.0, t_start + k / 120 / args.rate - time.time()))
    w = obs[:, 2]
    # full positive voltage (the direction that rights a positive lean); the motor delivers what its
    # line allows at the current arm speed: up to 2x stall while braking a pre-spun arm (plugging),
    # stall at rest, falling to 0 at no-load. (Until 2026-10-11 this commanded 1x stall, which
    # capped plugging at 1x.)
    torque = torch.clamp(torch.full_like(w, 2 * args.stall), -args.stall - slope * w, args.stall - slope * w)
    for i, ((a, w0), o, u) in enumerate(zip(starts, obs.tolist(), torque.tolist())):
        rows.append((a, w0, k / 120, math.degrees(o[0]), o[1], o[2], u))
    obs, _, _, _, _ = env.step(torque.unsqueeze(1))
    obs = obs["policy"]

os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
with open(args.out, "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(["theta0", "arm0", "t", "theta_deg", "w_base", "w_arm", "effort"])
    wr.writerows(rows)

def reaches_upright(a, w0):
    """True if the body gets to theta <= 0 before it passes 45 deg. Scoring stops there: past the
    env's fall threshold it auto-resets to a random lean, and the trace after that is not this run."""
    for t, x in [(r[2], r[3]) for r in rows if r[0] == a and r[1] == w0]:
        if x <= 0.0:
            return True
        if x > 45.0:
            return False
    return False


for label, w0 in (("from rest", 0.0), ("pre-spun", -args.no_load)):
    reached = [a for a, s in starts if s == w0 and reaches_upright(a, w0)]
    print(f"{label}: body reaches upright from leans up to {max(reached, default=0):g} deg "
          f"({len(reached)} of {len(args.angles)} tried)")
print(f"wrote {args.out}")

while args.hold and app.is_running():
    app.update()
env.close()
app.close()
