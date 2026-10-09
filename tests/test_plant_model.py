from pathlib import Path

import pytest

from analysis.plant_model import load

URDF = Path(__file__).resolve().parents[1] / "urdf"


def test_body_urdf_matches_hand_derivation():
    # analysis/lqr_design.py's hand-derived A and B for the benchmark pendulum
    A, B, _ = load(URDF / "body.urdf")
    assert A[1, 0] == pytest.approx(40.85, rel=1e-3)
    assert B[1] == pytest.approx(-555.2, rel=1e-3)
    assert B[2] == pytest.approx(53186.8, rel=1e-3)
    assert A[2, 0] == -A[1, 0]


def test_mrv_urdf_totals():
    # measured on a scale: 358 g total, COM 210 mm above the foot
    _, _, info = load(URDF / "mrv.urdf")
    assert info["mass"] == pytest.approx(0.3581, rel=1e-3)
    assert info["com"] == pytest.approx(0.210, abs=5e-4)
    assert info["I_a"] == pytest.approx(2.093e-4)
