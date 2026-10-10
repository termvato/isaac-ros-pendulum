#!/usr/bin/env bash
# 10-10: seed 10 (despun at 500 and 2000 its) for 5000 its, then the ROS harness on it.
# Base config: LQR-cost reward, gamma 0.997, fixed LR 1e-3, resets +-8 deg.
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


RUNS=""
train lr5k_s10 --seed 10 --max_iterations 5000
~/IsaacLab/isaaclab.sh -p analysis/export_tb.py $RUNS --out results/training > logs/export_tb_5k.log 2>&1
(
source /opt/ros/jazzy/setup.bash; source install/setup.bash
run=$(ls -td logs/rsl_rl/pendulum_direct/*_lr5k_s10 | head -1)
echo "== harness lr5k_s10 $(date -u +%T)"
ros2 run pendulum_nodes pendulum_policy --ros-args --params-file config/limits.yaml \
    -p model:=$PWD/$run/exported/policy.onnx -p use_sim_time:=true > logs/node_lr5k_s10.log 2>&1 &
pid=$!; sleep 4
./isaac/run_harness.sh --label ppo_lr5k_s10_0p025 > logs/harness_lr5k_s10.log 2>&1
kill $pid; sleep 2; pkill -9 -f "pendulum_polic[y]"
grep -E "BOUNDARY|RuntimeError|trial +3.00" logs/harness_lr5k_s10.log | tail -1
)
echo "== leftover processes: $(pgrep -fc 'rl/lab.p[y]|lab_eva[l]|pendulum_polic[y]' || true)"
echo "== done $(date -u +%T)"
