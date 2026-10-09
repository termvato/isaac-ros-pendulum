"""Run an exported policy in Isaac Lab from fixed leans, no noise, and print where the arm ends up.

Answers one question: does the policy behave the same in Lab as through ROS in Isaac Sim?
One env per start angle, all at rest, 10 s, same physics as training.

    PYTHONPATH=. ~/IsaacLab/isaaclab.sh -p rl/lab_eval.py --headless \
        --policy <run>/exported/policy.pt --out results/lab_<label>.csv
"""

import argparse
import functools
print = functools.partial(print, flush=True)  # app.close() exits hard and drops buffered stdout

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--policy", required=True, help="TorchScript policy.pt from play.py (normalizer included)")
parser.add_argument("--angles", type=float, nargs="+", default=[3.0, 5.0, 7.0, 9.0], help="start leans, deg")
parser.add_argument("--out", required=True, help="trajectory CSV, same columns as the harness's traj_*.csv")
parser.add_argument("--angular_damping", type=float, default=None, help="override the env's body angular damping")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import csv  # noqa: E402
import math  # noqa: E402
import os  # noqa: E402

import torch  # noqa: E402

from rl.pendulum_env import PendulumEnv, PendulumEnvCfg  # noqa: E402

cfg = PendulumEnvCfg()
cfg.scene.num_envs = len(args.angles)
cfg.max_theta = math.radians(45.0)  # evaluation's "fell", not training's 15 deg
if args.angular_damping is not None:
    cfg.robot_cfg.spawn.rigid_props.angular_damping = args.angular_damping
env = PendulumEnv(cfg)
policy = torch.jit.load(args.policy, map_location=env.device).eval()

env.reset()
base, arm = env._base_dof_idx[0], env._arm_dof_idx[0]
pos = torch.zeros_like(env.pendulum.data.joint_pos)
pos[:, base] = torch.tensor([math.radians(a) for a in args.angles], device=env.device)
vel = torch.zeros_like(pos)
env.pendulum.write_joint_state_to_sim(pos, vel)
env.joint_pos, env.joint_vel = pos.clone(), vel.clone()
obs = env._get_observations()["policy"]

steps = int(cfg.episode_length_s * 120) - 2  # stop before the time-out reset
rows = []
# The env auto-resets an env that falls (to a random lean), so stop recording it at its first
# fall; otherwise the trace jumps back to a new start and looks like a recovery.
alive = [True] * len(args.angles)
with torch.no_grad():
    for k in range(steps + 1):
        act = policy(obs)
        torque = (act.clamp(-1.0, 1.0) * cfg.action_scale).squeeze(1).tolist()
        for i, (a, o, u) in enumerate(zip(args.angles, obs.tolist(), torque)):
            if alive[i]:
                rows.append((a, k / 120, math.degrees(o[0]), o[1], o[2], u))
        obs, _, fell, _, _ = env.step(act)
        obs = obs["policy"]
        alive = [al and not f for al, f in zip(alive, fell.tolist())]

os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
with open(args.out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["theta0", "t", "theta_deg", "w_base", "w_arm", "effort"])
    w.writerows(rows)
for a in args.angles:
    end = [r for r in rows if r[0] == a][-1]
    print(f"{a:5.1f} deg: {'FELL' if end[1] < steps / 120 - 1e-6 else 'ok  '} at t={end[1]:.2f} s, "
          f"theta {end[2]:+.3f} deg, w_arm {end[4]:+.1f} rad/s")
print(f"wrote {args.out}")

env.close()
app.close()
