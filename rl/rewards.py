"""Reward terms with no Isaac imports, so they can be unit tested on plain torch tensors.

Product reward, after the DeepMind Control Suite (Tassa et al. 2018, cartpole:
upright * centered * small_control * small_velocity). Terms multiply instead of add, so one
cannot be traded against another. Each softened term is (1 + bell) / 2, as dm_control softens
its secondary terms, so it can at most halve the reward: a catch must spin the arm up, and a hard
arm factor would score every catch near zero (the trap arm-speed shaping fell into on 10-09).
"""

import torch


def bell(x: torch.Tensor, width: float) -> torch.Tensor:
    """exp(-(x/width)^2): 1 at x = 0, 0.37 at x = width."""
    return torch.exp(-torch.square(x / width))


def product_reward(
    theta: torch.Tensor,
    w_arm: torch.Tensor,
    torque: torch.Tensor,
    theta_width: float,
    w_arm_width: float,
    torque_width: float,
) -> torch.Tensor:
    """upright * (1 + small_arm) / 2 * (1 + small_control) / 2, in [0, 1].

    torque_width = 0 drops the control term. Per-step reward is never negative, so falling
    forfeits future reward without a separate penalty.
    """
    reward = bell(theta, theta_width) * (1.0 + bell(w_arm, w_arm_width)) / 2.0
    if torque_width:
        reward = reward * (1.0 + bell(torque, torque_width)) / 2.0
    return reward
