import ast
from pathlib import Path

import pytest
import torch

from rl.lqr_policy import GAINS, MAX_TORQUE, LqrPolicy

CONTROLLER = Path(__file__).resolve().parents[1] / "src/pendulum_nodes/pendulum_nodes/pendulum_controller.py"


def controller_defaults():
    """The default gains declared in pendulum_controller.py, read from its source."""
    out = {}
    for node in ast.walk(ast.parse(CONTROLLER.read_text())):
        if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "declare_parameter":
            name, default = node.args
            if name.value.startswith("k_"):
                out[name.value] = default.value
    return out


def test_gains_match_ros_controller():
    d = controller_defaults()
    assert GAINS == (d["k_theta_base"], d["k_w_base"], d["k_w_arm"])


def test_max_torque_from_limits_yaml():
    assert MAX_TORQUE == 0.025


def test_matches_controller_law():
    # the node computes u = k . x then clamps to +-max_torque; the env clips the action to +-1
    # and scales by max_torque, which is the same clamp
    policy = LqrPolicy()
    obs = torch.tensor([[0.05, -0.3, 40.0], [-0.1, 0.2, -80.0], [0.0, 0.0, 0.0]])
    action = policy(obs)
    assert action.shape == (3, 1)
    for x, a in zip(obs.tolist(), action[:, 0].tolist()):
        u = sum(k * v for k, v in zip(GAINS, x))
        u_node = max(-MAX_TORQUE, min(MAX_TORQUE, u))
        u_env = max(-1.0, min(1.0, a)) * MAX_TORQUE
        assert u_env == pytest.approx(u_node, rel=1e-5, abs=1e-9)


def test_scripts_and_round_trips(tmp_path):
    path = tmp_path / "policy.pt"
    torch.jit.script(LqrPolicy()).save(str(path))
    loaded = torch.jit.load(str(path))
    obs = torch.randn(8, 3)
    assert torch.allclose(loaded(obs), LqrPolicy()(obs))
