"""The LQR law as a TorchScript policy, so rl/lab_eval.py can run LQR in Isaac Lab exactly as it
runs a PPO policy. Same law and gains as pendulum_controller.py: u = k . [theta, theta_dot, w_arm],
returned as a normalised action u / max_torque (the env clips to +-1 and scales back).

    ~/IsaacLab/isaaclab.sh -p rl/lqr_policy.py --out logs/lqr/policy.pt
"""

import argparse
from pathlib import Path

import torch
import yaml

GAINS = (9.16291716e-01, 1.43911551e-01, 6.32455532e-04)  # pendulum_controller.py defaults, R = 1e5
with open(Path(__file__).resolve().parents[1] / "config" / "limits.yaml") as f:
    MAX_TORQUE = yaml.safe_load(f)["/**"]["ros__parameters"]["max_torque"]  # [N.m]


class LqrPolicy(torch.nn.Module):
    def __init__(self, gains=GAINS, max_torque=MAX_TORQUE):
        super().__init__()
        self.register_buffer("k", torch.tensor(gains, dtype=torch.float32) / max_torque)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return (obs * self.k).sum(dim=-1, keepdim=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--out", required=True)
    args = p.parse_args()
    torch.jit.script(LqrPolicy()).save(args.out)
    print(f"wrote {args.out}")
