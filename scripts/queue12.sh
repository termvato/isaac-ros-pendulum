#!/usr/bin/env bash
# 10-09 overnight part 2: product (DeepMind-style) reward, seeds 1-10, 500 its, after queue11.
# Base config: gamma 0.997, fixed LR 1e-3, resets +-8 deg.
cd ~/isaac-ros-pendulum
echo "== start $(date -u +%T)"
R8=0.13962634  # 8 deg in rad
G2=0.03490659  # 2 deg in rad

train() {  # name, extra args...
  local name=$1; shift
  until [ $(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits) -lt 2000 ]; do
    echo "waiting for GPU: $(nvidia-smi --query-compute-apps=pid,name --format=csv,noheader | tr '\n' ' ')"; sleep 30
  done
  echo "== train $name $(date -u +%T)"
  ~/IsaacLab/isaaclab.sh -p rl/lab.py train --task Pendulum-Direct-v0 --headless --run_name $name \
      agent.algorithm.gamma=0.997 agent.algorithm.schedule=fixed agent.algorithm.learning_rate=1e-3 \
      "env.initial_theta_range=[-$R8,$R8]" "$@" > logs/train_$name.log 2>&1
  local run=$(ls -td logs/rsl_rl/pendulum_direct/*_$name | head -1)
  ~/IsaacLab/isaaclab.sh -p rl/lab.py play --task Pendulum-Direct-v0 --headless --num_envs 1 \
      --load_run $(basename $run) > logs/play_$name.log 2>&1 &
  for i in $(seq 120); do [ -f $run/exported/policy.onnx ] && break; sleep 5; done; sleep 5
  pkill -f -- "--load_run $(basename $run)"; sleep 10; pkill -9 -f -- "--load_run $(basename $run)"; wait
  PYTHONPATH=. ~/IsaacLab/isaaclab.sh -p rl/lab_eval.py --headless --policy $run/exported/policy.pt \
      --angles 3 4 5 5.5 6 6.5 7 7.2 7.4 7.6 7.8 8 9 --out results/lab_ppo_$name.csv \
      > logs/lab_eval_$name.log 2>&1
  echo "$name lab: $(grep -E 'deg: ' logs/lab_eval_$name.log | awk '{printf "%s%s(%s) ", $1, ($3=="ok"?"+":"x"), $(NF-1)}')"
  RUNS="$RUNS $run"
}

while pgrep -f "queue11.s[h]" > /dev/null; do sleep 60; done
echo "== queue11 finished $(date -u +%T)"

# smoke test: 2 iterations must run end to end before committing the night to it
until [ $(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits) -lt 2000 ]; do sleep 30; done
if ! ~/IsaacLab/isaaclab.sh -p rl/lab.py train --task Pendulum-Direct-v0 --headless --run_name prod_smoke \
    --max_iterations 2 env.reward_type=product > logs/train_prod_smoke.log 2>&1 \
    || ! grep -q "Learning iteration 1/2" logs/train_prod_smoke.log; then
  echo "== smoke test FAILED, see logs/train_prod_smoke.log"; exit 1
fi
echo "== smoke ok $(date -u +%T)"

RUNS=""
for s in $(seq 1 10); do train prod_s$s --seed $s env.reward_type=product; done

~/IsaacLab/isaaclab.sh -p analysis/export_tb.py $RUNS --out results/training > logs/export_tb_prod.log 2>&1
echo "== leftover processes: $(pgrep -fc 'rl/lab.p[y]|lab_eva[l]' || true)"
echo "== done $(date -u +%T)"
