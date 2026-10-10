#!/usr/bin/env bash
# 10-10: re-run the ROS harness after the stale-command fix (HOLD_S), LQR and the PPO policies
# that despin, to replace the contaminated boundaries.
cd ~/isaac-ros-pendulum
source /opt/ros/jazzy/setup.bash; source install/setup.bash
echo "== start $(date -u +%T)"
for name in lqr lr_s1 lr_s10 lr5k_s10; do
  echo "== harness $name $(date -u +%T)"
  if [ $name = lqr ]; then
    ros2 run pendulum_nodes pendulum_controller --ros-args --params-file config/limits.yaml \
        -p use_sim_time:=true > logs/node_hold_$name.log 2>&1 &
  else
    run=$(ls -td logs/rsl_rl/pendulum_direct/*_$name | head -1)
    ros2 run pendulum_nodes pendulum_policy --ros-args --params-file config/limits.yaml \
        -p model:=$PWD/$run/exported/policy.onnx -p use_sim_time:=true > logs/node_hold_$name.log 2>&1 &
  fi
  pid=$!; sleep 4
  ./isaac/run_harness.sh --label hold_${name}_0p025 > logs/harness_hold_$name.log 2>&1
  kill $pid; sleep 2; pkill -9 -f "pendulum_polic[y]|pendulum_controlle[r]"
  grep -E "BOUNDARY|RuntimeError" logs/harness_hold_$name.log | tail -1
done
echo "== leftover processes: $(pgrep -fc 'pendulum_polic[y]|pendulum_controlle[r]' || true)"
echo "== done $(date -u +%T)"
