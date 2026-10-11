#!/usr/bin/env bash
# Train one PPO seed with the default (final) configuration, export it, and evaluate it in Lab.
#     scripts/train_eval.sh <seed> [name] [extra Hydra overrides, e.g. --max_iterations 2000]
# Writes logs/rsl_rl/pendulum_direct/<date>_<name>/ (policy.pt and policy.onnx under exported/)
# and results/lab_ppo_<name>.csv. ~7 min on an RTX 4070 Laptop.
set -e
cd "$(dirname "$(realpath "$0")")/.."
seed=$1; name=${2:-s$seed}; shift; [ $# -gt 0 ] && shift
LAB=~/IsaacLab/isaaclab.sh
mkdir -p logs results
echo "== train $name $(date -u +%T)"
$LAB -p rl/lab.py train --task Pendulum-Direct-v0 --headless --run_name "$name" --seed "$seed" "$@" \
    > "logs/train_$name.log" 2>&1
run=$(ls -td logs/rsl_rl/pendulum_direct/*_"$name" | head -1)
# play.py exports policy.pt and policy.onnx, then keeps simulating; stop it once both exist
$LAB -p rl/lab.py play --task Pendulum-Direct-v0 --headless --num_envs 1 --load_run "$(basename "$run")" \
    > "logs/play_$name.log" 2>&1 &
for i in $(seq 120); do [ -f "$run/exported/policy.onnx" ] && [ -f "$run/exported/policy.pt" ] && break; sleep 5; done
sleep 5
pkill -f -- "--load_run $(basename "$run")" || true; sleep 10
pkill -9 -f -- "--load_run $(basename "$run")" || true  # play ignores SIGTERM at times
wait || true
[ -f "$run/exported/policy.pt" ] || { echo "export failed, see logs/play_$name.log"; exit 1; }
echo "== evaluate $name $(date -u +%T)"
PYTHONPATH=. $LAB -p rl/lab_eval.py --headless --policy "$run/exported/policy.pt" \
    --angles 3 4 5 5.5 6 6.5 7 7.2 7.4 7.6 7.8 8 9 --out "results/lab_ppo_$name.csv" > "logs/lab_eval_$name.log" 2>&1
grep -E " deg: " "logs/lab_eval_$name.log"
echo "policy: $run/exported/policy.onnx"
