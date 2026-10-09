import math

import pytest
import torch

from rl.rewards import bell, product_reward

DEG = math.pi / 180


def r(theta, w_arm, torque=0.0, torque_width=0.0):
    t = lambda v: torch.tensor([float(v)])  # noqa: E731
    return product_reward(t(theta), t(w_arm), t(torque), 6 * DEG, 10.0, torque_width).item()


def test_bell():
    assert bell(torch.tensor(0.0), 2.0).item() == 1.0
    assert bell(torch.tensor(2.0), 2.0).item() == pytest.approx(math.exp(-1))
    assert bell(torch.tensor(-2.0), 2.0).item() == bell(torch.tensor(2.0), 2.0).item()


def test_upright_and_despun_pays_one():
    assert r(0.0, 0.0) == 1.0


def test_arm_factor_floor_is_half():
    # a catch spins the arm up; that may halve the reward but never zero it
    assert r(0.0, 100.0) == pytest.approx(0.5)
    assert r(0.0, 1e6) >= 0.5


def test_parked_arm_pays_less_than_despun():
    assert r(0.0, 30.0) < r(0.0, 1.0) < r(0.0, 0.0)


def test_lean_is_multiplicative_not_additive():
    # leaning scales the whole reward, so a despun arm cannot buy back a fall
    assert r(12 * DEG, 0.0) == pytest.approx(math.exp(-4))
    assert r(12 * DEG, 30.0) == pytest.approx(r(12 * DEG, 0.0) * r(0.0, 30.0))


def test_bounded_and_nonnegative():
    theta = torch.linspace(-0.3, 0.3, 61)
    w = torch.linspace(-100, 100, 61)
    out = product_reward(theta, w, torch.zeros(61), 6 * DEG, 10.0, 0.0)
    assert out.min().item() >= 0.0 and out.max().item() <= 1.0


def test_torque_term_off_by_default_and_halves_at_most():
    assert r(0.0, 0.0, torque=0.025) == 1.0
    assert r(0.0, 0.0, torque=1.0, torque_width=0.01) == pytest.approx(0.5)


def test_batched_shape():
    n = 4096
    out = product_reward(torch.zeros(n), torch.zeros(n), torch.zeros(n), 6 * DEG, 10.0, 0.0)
    assert out.shape == (n,)
