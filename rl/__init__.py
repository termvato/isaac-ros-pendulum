"""Reaction-wheel pendulum env for Isaac Lab."""

import gymnasium as gym

from . import agents

gym.register(
    id="Pendulum-Direct-v0",
    entry_point=f"{__name__}.pendulum_env:PendulumEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.pendulum_env:PendulumEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PendulumPPORunnerCfg",
    },
)
