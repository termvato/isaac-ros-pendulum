import pytest
import torch

from rl.delay import ActionDelay


def test_zero_delay_passes_through():
    d = ActionDelay(0)
    a = torch.tensor([[0.3]])
    assert d(a) is a


def test_delay_shifts_by_steps_with_zeros_first():
    d = ActionDelay(2)
    out = [d(torch.tensor([[float(k)]])).item() for k in range(1, 6)]
    assert out == [0.0, 0.0, 1.0, 2.0, 3.0]


def test_batched_shape_kept():
    d = ActionDelay(1)
    assert d(torch.ones(13, 1)).shape == (13, 1)
    assert torch.equal(d(torch.full((13, 1), 2.0)), torch.ones(13, 1))


def test_negative_delay_rejected():
    with pytest.raises(ValueError):
        ActionDelay(-1)
