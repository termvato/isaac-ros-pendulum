# Adapted from Isaac Lab's direct cartpole env (BSD-3-Clause, Isaac Lab Project Developers).
# Reaction-wheel pendulum: passive base_body, actuated body_arm, same plant (urdf/body.urdf)
# and same torque limit (config/limits.yaml) as the ROS controller.

from __future__ import annotations

import math
from collections.abc import Sequence
from pathlib import Path

import torch
import yaml

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils import configclass
from isaaclab.utils.math import sample_uniform

REPO = Path(__file__).resolve().parents[1]
URDF_PATH = str(REPO / "urdf" / "body.urdf")
with open(REPO / "config" / "limits.yaml") as f:
    MAX_TORQUE = yaml.safe_load(f)["/**"]["ros__parameters"]["max_torque"]

DEG = math.pi / 180.0

PENDULUM_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=URDF_PATH,
        fix_base=True,  # bolt base_link to the world
        # importer adds a PD drive to every joint; zero it so nothing holds the body up
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)
        ),  # belt and braces: the actuator cfg below zeroes these again at sim start
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            max_angular_velocity=1.0e4,  # deg/s; keep PhysX's body cap above the 100 rad/s joint limit
        ),  # world-frame cap per body, not the motor limit; arm world speed = body + arm, so 100 rad/s here would bind early
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0), joint_pos={"base_body": 0.0, "body_arm": 0.0}
    ),
    actuators={
        # pure effort input; velocity limit left to the URDF
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["body_arm"], effort_limit_sim=MAX_TORQUE, stiffness=0.0, damping=0.0
        ),
        # passive: zero effort limit, so nothing can ever push base_body
        "base": ImplicitActuatorCfg(
            joint_names_expr=["base_body"], effort_limit_sim=0.0, stiffness=0.0, damping=0.0
        ),  # effort limit is a magnitude: PhysX clamps to [-limit, +limit], so 0 blocks both directions
    },
)


@configclass
class PendulumEnvCfg(DirectRLEnvCfg):
    # env: physics 240 Hz, policy 120 Hz, 10 s episodes
    decimation = 2
    episode_length_s = 10.0
    action_scale = MAX_TORQUE  # [N.m] per unit action
    action_space = 1  # one commanded value: arm torque
    observation_space = 3  # theta, theta_dot, w_arm
    state_space = 0  # no privileged critic inputs; critic sees the same 3 obs as the actor

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 240, render_interval=decimation)

    # robot
    robot_cfg: ArticulationCfg = PENDULUM_CFG.replace(prim_path="/World/envs/env_.*/Robot")  # copy with one robot per env_N
    base_dof_name = "base_body"
    arm_dof_name = "body_arm"

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096, env_spacing=1.0, replicate_physics=True, clone_in_fabric=True
    )

    # reset
    initial_theta_range = [-12.0 * DEG, 12.0 * DEG]  # [rad]
    initial_vel_range = [-0.1, 0.1]  # [rad/s], both joints; training only, the harness evaluates from rest
    max_theta = 15.0 * DEG  # training fall threshold [rad]; evaluation uses 45 deg

    # reward
    theta_width = 6.0 * DEG  # width of the upright bell [rad]
    despin_gate_width = 2.0 * DEG  # arm-speed penalty only applies inside this [rad]
    w_arm_scale = 100.0  # [rad/s]; arm speed limit, so (w_arm/scale)^2 is 1 at saturation and c reads as a fraction
    rew_scale_despin = 0.1  # c
    rew_fall = -10.0  # P


class PendulumEnv(DirectRLEnv):
    cfg: PendulumEnvCfg

    def __init__(self, cfg: PendulumEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self._base_dof_idx, _ = self.pendulum.find_joints(self.cfg.base_dof_name)
        self._arm_dof_idx, _ = self.pendulum.find_joints(self.cfg.arm_dof_name)
        self.action_scale = self.cfg.action_scale

        self.joint_pos = self.pendulum.data.joint_pos
        self.joint_vel = self.pendulum.data.joint_vel

    def _setup_scene(self):
        self.pendulum = Articulation(self.cfg.robot_cfg)
        # add ground plane
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
        # clone and replicate
        self.scene.clone_environments(copy_from_source=False)
        # we need to explicitly filter collisions for CPU simulation
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])
        # add articulation to scene
        self.scene.articulations["pendulum"] = self.pendulum
        # add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        # PPO samples from an unbounded Gaussian, so clip to +-1 before scaling
        self.actions = self.action_scale * actions.clone().clamp(-1.0, 1.0)

    def _apply_action(self) -> None:
        self.pendulum.set_joint_effort_target(self.actions, joint_ids=self._arm_dof_idx)

    def _get_observations(self) -> dict:
        # same three states as the LQR, same order
        obs = torch.cat(
            (
                self.joint_pos[:, self._base_dof_idx[0]].unsqueeze(dim=1),
                self.joint_vel[:, self._base_dof_idx[0]].unsqueeze(dim=1),
                self.joint_vel[:, self._arm_dof_idx[0]].unsqueeze(dim=1),
            ),
            dim=-1,
        )
        observations = {"policy": obs}
        return observations

    def _get_rewards(self) -> torch.Tensor:
        total_reward = compute_rewards(
            self.cfg.theta_width,
            self.cfg.despin_gate_width,
            self.cfg.w_arm_scale,
            self.cfg.rew_scale_despin,
            self.cfg.rew_fall,
            self.joint_pos[:, self._base_dof_idx[0]],
            self.joint_vel[:, self._arm_dof_idx[0]],
            self.reset_terminated,
        )  # one reward per env per policy step; PPO sums them discounted by gamma
        return total_reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self.joint_pos = self.pendulum.data.joint_pos
        self.joint_vel = self.pendulum.data.joint_vel

        time_out = self.episode_length_buf >= self.max_episode_length - 1
        fell = torch.abs(self.joint_pos[:, self._base_dof_idx[0]]) > self.cfg.max_theta
        return fell, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.pendulum._ALL_INDICES
        super()._reset_idx(env_ids)

        joint_pos = self.pendulum.data.default_joint_pos[env_ids]
        joint_pos[:, self._base_dof_idx] = sample_uniform(
            self.cfg.initial_theta_range[0],
            self.cfg.initial_theta_range[1],
            joint_pos[:, self._base_dof_idx].shape,
            joint_pos.device,
        )
        joint_vel = sample_uniform(
            self.cfg.initial_vel_range[0],
            self.cfg.initial_vel_range[1],
            self.pendulum.data.default_joint_vel[env_ids].shape,
            joint_pos.device,
        )

        default_root_state = self.pendulum.data.default_root_state[env_ids]
        default_root_state[:, :3] += self.scene.env_origins[env_ids]

        self.joint_pos[env_ids] = joint_pos
        self.joint_vel[env_ids] = joint_vel

        self.pendulum.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self.pendulum.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self.pendulum.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)


@torch.jit.script
def compute_rewards(
    theta_width: float,
    despin_gate_width: float,
    w_arm_scale: float,
    rew_scale_despin: float,
    rew_fall: float,
    theta: torch.Tensor,
    w_arm: torch.Tensor,
    reset_terminated: torch.Tensor,
):
    # positive bell for being upright, so surviving always beats falling early
    rew_upright = torch.exp(-torch.square(theta / theta_width))
    # arm-speed penalty, gated to near-upright so recovery may spend the speed budget freely
    rew_despin = -rew_scale_despin * torch.square(w_arm / w_arm_scale) * torch.exp(
        -torch.square(theta / despin_gate_width)
    )
    rew_termination = rew_fall * reset_terminated.float()
    return rew_upright + rew_despin + rew_termination
