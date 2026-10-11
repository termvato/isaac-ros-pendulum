#!/usr/bin/env bash
# 10-11: LQR boundaries at 0.05 N.m and with no torque limit on the fixed harness, replacing the
# 09-25 hand-sweep numbers (pre-fix, 0.5 deg grid). 1000 N.m stands in for "no limit": the
# unclamped LQR asks for at most ~2.4 N.m. Usage: scripts/queue15.sh [torque ...]
cd ~/isaac-ros-pendulum
source /opt/ros/jazzy/setup.bash; source install/setup.bash
echo "== start $(date -u +%T)"
for tq in ${@:-0.05 1000.0}; do
  label=hold_lqr_$(echo $tq | tr . p)
  echo "== harness $label $(date -u +%T)"
  ros2 run pendulum_nodes pendulum_controller --ros-args --params-file config/limits.yaml \
      -p use_sim_time:=true -p max_torque:=$tq > logs/node_$label.log 2>&1 &
  pid=$!; sleep 4
  ./isaac/run_harness.sh --label $label > logs/harness_$label.log 2>&1
  kill $pid; sleep 2
  # killing `ros2 run` leaves the node itself running, and a second controller on /joint_command
  # races the next one (last writer wins); stop every controller before the next setting
  pkill -f "lib/pendulum_nodes/pendulum_controlle[r]"; sleep 2
  grep -E "BOUNDARY|RuntimeError" logs/harness_$label.log | tail -1
done
echo "== leftover processes: $(pgrep -fc 'pendulum_controlle[r]' || true)"
echo "== done $(date -u +%T)"
