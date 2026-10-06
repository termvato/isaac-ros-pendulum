"""Plant check: does Isaac Lab's import of urdf/body.urdf behave like the URDF says?

Zero torque from 5 deg at rest; time to pass 45 deg must match the hand-integrated
pendulum, and the joint limits and drives must be what we think they are.

    ~/IsaacLab/isaaclab.sh -p rl/plant_check.py --headless
"""

import argparse
import math
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
simulation_app = AppLauncher(args_cli).app

# everything else after the app exists
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rl.pendulum_env import DEG, PendulumEnv, PendulumEnvCfg

THETA0 = 5.0 * DEG
THETA_FALL = 45.0 * DEG


def expected_fall_time():
    # URDF numbers. Zero arm torque means the arm keeps its inertial angle, so its spin
    # inertia drops out and only its mass counts, as a point at the joint (0.3 m).
    mgl = 9.81 * (0.03 * 0.15 + 0.01 * 0.3)
    i_b = 2.26e-4 + 0.03 * 0.15**2 + 0.01 * 0.3**2
    th, w, t, dt = THETA0, 0.0, 0.0, 1e-5
    while th < THETA_FALL:
        w += mgl / i_b * math.sin(th) * dt
        th += w * dt
        t += dt
    return t


def main():
    cfg = PendulumEnvCfg()
    cfg.scene.num_envs = 1
    cfg.initial_theta_range = [THETA0, THETA0]
    cfg.initial_vel_range = [0.0, 0.0]
    cfg.max_theta = THETA_FALL
    env = PendulumEnv(cfg)
    env.reset()

    d = env.pendulum.data
    print("joints:          ", env.pendulum.joint_names)
    print("vel limits rad/s:", d.joint_vel_limits[0].tolist())
    print("effort limits:   ", d.joint_effort_limits[0].tolist())
    print("stiffness:       ", d.joint_stiffness[0].tolist())
    print("damping:         ", d.joint_damping[0].tolist())

    base = env._base_dof_idx[0]
    arm = env._arm_dof_idx[0]
    zero = torch.zeros(1, 1, device=env.device)
    steps = 0
    with torch.inference_mode():
        while True:
            theta, theta_dot, w_arm = (
                d.joint_pos[0, base].item(), d.joint_vel[0, base].item(), d.joint_vel[0, arm].item()
            )
            _, _, terminated, truncated, _ = env.step(zero)
            steps += 1
            if terminated[0] or truncated[0]:
                break

    step_dt = cfg.sim.dt * cfg.decimation
    print(f"start theta:     {math.degrees(THETA0):.2f} deg")
    print(f"last theta:      {math.degrees(theta):.2f} deg, theta_dot {theta_dot:.3f}, w_arm {w_arm:.3f} rad/s")
    print(f"terminated:      {bool(terminated[0])} (timeout would mean it never fell)")
    print(f"fall time Lab:   {steps * step_dt:.3f} s (+-{step_dt:.4f})")
    print(f"fall time URDF:  {expected_fall_time():.3f} s")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
