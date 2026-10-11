import re
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("scipy")

REPO = Path(__file__).resolve().parents[1]


def limit(*args):
    out = subprocess.run([sys.executable, "analysis/recovery_limit.py", *args], cwd=REPO,
                         capture_output=True, text=True, check=True).stdout
    return float(re.search(r"max recoverable lean: ([\d.]+) deg", out).group(1))


def test_benchmark_limit():
    assert limit("--despin") == pytest.approx(7.47, abs=0.01)


def test_mrv_plugging_is_allowed():
    # pre-spun against the motor line, braking may use up to 2x stall; a 1x box gave 4.41
    mrv = ("--urdf", "urdf/mrv.urdf", "--max_torque", "0.0653", "--no_load_speed", "58.7",
           "--max_arm_speed", "1000")
    assert limit(*mrv) == pytest.approx(2.82, abs=0.01)
    assert limit(*mrv, "--arm0", "-58.7") == pytest.approx(5.65, abs=0.01)
