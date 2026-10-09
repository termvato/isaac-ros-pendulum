# Project journal

How this project actually went, including the wrong turns. The README says what the system
is; this file says how it got that way. Newest entries at the bottom.

## 2026-09-13 to 09-20: from zero ROS to a closed loop

- Picked the interface before writing code: Isaac Sim is the plant, the controller is a
  separate ROS 2 node, `/joint_states` out and `/joint_command` in, both
  `sensor_msgs/JointState`. Later found Isaac's own graph wizard uses the same convention.
- Environment cost days before any robotics: NVIDIA driver 595 crashes Isaac Sim 5.1 in the
  RTX renderer; pinned 580.
- First silent failure: the URDF importer stores joint speed limits in deg/s, so a 10 rad/s
  limit appears as 572.95 in a field labelled like rad/s.

## 2026-09-21 to 09-26: LQR, and the result that shaped everything after

- LQR from the URDF numbers balances. Isaac models no torque limit and delivered 118x the
  intended torque budget without complaint, so the clamp lives in the controller.
- **The design is speed-limited, not torque-limited.** Peak arm speed is ~1265 x initial lean
  (rad/s per rad) whatever the motor, so arm saturation starts at ~4.5 deg for any torque
  limit. Halving the motor twice cost only 1.4 deg of recoverable lean.
- PD never despins the arm, so each recovery leaves momentum for the next one; it fell at
  4 deg because it started already spun up. That is the case for feeding arm speed back.
- Logged sim timing is exact (120.000 Hz in sim time); real hardware will jitter.

## 2026-10-05 to 10-06: the experiment

- Question: in the speed-saturated regime, does a learned policy (PPO, Isaac Lab) recover
  from larger leans than LQR under identical limits? Recovered means upright **and** arm
  despun at 10 s, for both controllers.
- Built a bisection harness: LQR at 0.025 N.m recovers 7.03 deg, fails 7.08, deterministic.
- Built the Lab env; plant check: free fall to 45 deg in 0.458 s vs 0.454 s by hand.
- Found Isaac Sim's standalone mode steps physics at 60 Hz by default. LQR fell from 3 deg
  and nothing errored.

## 2026-10-09: changed how the work is split, and the first policy

**Why the commits speed up from here.** Until now I wrote most of the code myself to learn
the tooling. From today the mechanical work (wrappers, training runs, export, harness runs,
plotting) is delegated to an AI coding assistant (Claude Code), and my job is the research:
writing predictions before each run, deciding what to test, reviewing what the code assumes,
and interpreting results. A robotics researcher doesn't keep in their head how to wire ROS
nodes; they need to know what their setup assumes, because every silent failure in this
project was an assumption, not a wiring bug. Rules: predictions are written down before
results are seen (`results/predictions_2026-10-09.md`), and any judgement call (reward,
hyperparameters) stops and waits for me.

What happened:
- Training is cheap: 500 PPO iterations at 4096 envs take ~2 min (~7 min with Isaac start
  and shutdown). So compute is not the bottleneck; evaluation and thinking are.
- Seed 1 failed the harness sanity check at 3 deg: body upright to 0.04 deg, arm still at
  58 rad/s after 10 s. It balances but does not despin.
- Ran the same policy in Lab: identical to three figures (57.7 vs 57.65 rad/s). So the
  Lab-to-Sim transfer is clean here, and the failure is training, not transfer. Without the
  despin requirement it holds the body from 9 deg, where LQR falls at 7.08.
- **Hidden plant term found.** The body settled at 0.043 deg, not 0, from every start. Cause:
  PhysX applies a default angular damping (drag) to every rigid body, which the URDF never
  mentions. The motor has to push against drag to hold arm speed, its reaction tilts the
  body, and gravity balances it at a small lean. Zeroing the damping gave exactly 0.000 deg.
  Now zeroed explicitly in both the Lab env and the harness, so the URDF owns the plant, and
  LQR is being re-measured on the corrected plant.
- **The policy learned an arm-speed setpoint.** Queried directly, at upright with the body
  still, it commands full torque one way below 58 rad/s and full torque the other way above
  it: bang-bang regulation to 58 rad/s, where LQR regulates to 0. Retrained on the corrected
  plant, the same seed parks at 49.7 rad/s; with the despin weight doubled, 45.5.
- Two candidate causes for no despin: (1) the despin penalty is ~3% of the reward signal;
  (2) despinning takes ~3 s of gentle lean while gamma = 0.995 at 120 Hz only "sees" ~1.7 s
  ahead. Testing each separately.
- Open design question: the despin penalty only applies within 2 deg of upright, which also
  caps how hard the policy can lean to despin. Revisit if the runs above don't fix it.
