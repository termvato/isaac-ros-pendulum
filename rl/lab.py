"""Run one of Isaac Lab's rsl_rl scripts (train.py, play.py) with Pendulum-Direct-v0 registered.

Lab's scripts only import isaaclab_tasks, so they cannot see our gym id. Registering is
safe before the app launches because the entry points are strings, resolved later.

Usage, from the repo root:
    ~/IsaacLab/isaaclab.sh -p rl/lab.py train --task Pendulum-Direct-v0 --headless --seed 1
    ~/IsaacLab/isaaclab.sh -p rl/lab.py play --task Pendulum-Direct-v0 --headless --num_envs 1 \
        --load_run <run dir name>
Logs land in ./logs/rsl_rl/pendulum_direct/; play.py writes the ONNX to <run>/exported/.
"""

import os
import runpy
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAB_DIR = os.path.expanduser("~/IsaacLab/scripts/reinforcement_learning/rsl_rl")

script = os.path.join(LAB_DIR, sys.argv.pop(1) + ".py")
sys.path.insert(0, REPO)
sys.path.insert(0, LAB_DIR)  # for the scripts' local `import cli_args`

import rl  # noqa: E402,F401  registers Pendulum-Direct-v0

sys.argv[0] = script
runpy.run_path(script, run_name="__main__")
