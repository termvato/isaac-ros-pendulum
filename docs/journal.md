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
- Built a bisection harness: LQR at 0.025 N.m recovers ~7.0 deg. (Logged at the time as
  "deterministic"; that was wrong, see 10-09.)
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
  Lab-to-Sim transfer is clean here, and the failure is training, not transfer.
- **Retracted the same day:** I first read the Lab run as "PPO holds the body from 9 deg".
  The graph showed the trace jumping from 45 deg back to 7: the env had auto-reset after the
  fall and the evaluation kept recording. Fixed; PPO falls from 9 deg too. Lesson: plot
  before believing a table.
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
- **Results on the corrected plant (seed 1 each):**

  | controller | body recovers to | arm at 10 s |
  |---|---|---|
  | LQR (Isaac Sim via ROS) | 7.0 deg (+-0.05, see below) | ~0 rad/s, despun |
  | PPO baseline (Lab) | 7.4 deg | 49.7 rad/s |
  | PPO, despin weight doubled (Lab) | 7.4 deg | 45.5 rad/s |
  | PPO, gamma 0.999 (Lab) | 7.4 deg | 29.6 rad/s |

  None of the policies despins, so under the agreed definition none recovers at all. On the
  body alone they are 0.4 deg ahead of LQR, but that is Lab against Sim-via-ROS, not yet a
  fair comparison. All three hit the same 7.4 deg, which fits the 09-25 claim that the
  boundary is set by the arm speed limit, not by the controller.
- **Why no despin, from the graph** (`results/figures/recovery_7deg.png`). Every controller
  pins the arm at 100 rad/s for the same ~0.2 s. Then LQR swings the body ~2.2 deg past
  vertical and holds it there long enough for gravity to drain the arm: by the momentum
  budget, final arm speed is proportional to the integral of lean over the recovery, and LQR
  drives that integral to zero. The policies undershoot less, so the integral stays positive
  and the arm keeps the rest. Longer horizon (gamma 0.999) undershoots more and keeps less.
- **LQR's boundary is not deterministic.** At 6.98 deg one trial recovered and its repeat
  fell; at 7.03 the reverse. The committed 10-05 CSV shows the same flip, so the "repeat
  agrees" claim in my 10-05 notes was wrong. Likely cause: the ROS timer is not locked to the
  physics step, so which tick a command lands on depends on wall-clock scheduling. Report the
  boundary as 7.0 +- 0.05 deg.
- **Seed spread on the hand-made reward is large.** Seeds 1, 2 and 3 hold the body up to
  7.4, 5.5 and 6.5 deg (each +- 0.1 to 0.3), and park the arm at +49.7, +6.0 and -14.8 rad/s.
  So the parked speed is an arbitrary, seed-dependent setpoint (even its sign), and seed 1's
  7.4 deg was the lucky one: on average this policy is *below* LQR's 7.0 on the body alone.
  Prediction was ~1 deg of spread; actual is ~2 deg.
- The despin-weight series on seed 1 (c = 0.1, 0.2, 0.4 gives 49.7, 45.5, 9.1 rad/s) cannot
  be read as a trend: one seed per point, and seed-to-seed noise in parked speed is ~65 rad/s.
- R sweep for LQR (1e4 to 1e6): boundary 7.0 to 7.08 deg, settling 1.02 to 1.10 s at 6 deg,
  traces nearly identical (`results/figures/recovery_6deg_Rsweep.png`). Saturation decides
  the recovery, not the gains.
- **LQR-cost reward (gamma 0.997, 5 seeds, plus seed 1 at 2000 iterations): still no despin.**
  Body holds up to 6.5, 7.4, 4.0, 7.4, 7.2 deg (lower edge of each bracket; 7.4 for the long
  run); arm parks at +38.7, -18.0, -21.4, +17.2, +13.3 rad/s (-9.9 long). Spread did not
  shrink; my "a bowl-shaped objective will tighten it" prediction was wrong.
- 500 iterations had plateaued: the 2000-iteration curve is flat (and noisy) from ~200 on,
  and its first 500 iterations overlay seed 1's exactly, so training is deterministic per seed.
- Lab and Isaac Sim via ROS agree for all six policies: end state at 3 deg matches to three
  figures (e.g. 38.68 vs 38.7 rad/s). The interface claim holds for learned controllers.
- **Physical recovery limit: 7.48 deg** (`analysis/recovery_limit.py`). Linear program over all
  torque sequences at 120 Hz on the linearised plant, |torque| <= 0.025 N.m, |arm| <= 100 rad/s,
  start from rest, end upright and still. Requiring despin as well costs nothing (still 7.48).
  Checks: no arm limit gives 19.5 deg (the torque-only bound, by hand 0.025 x 556 / 40.9 rad);
  doubling torque gives only 8.36 deg (speed-limited, as measured on 09-25). So LQR's 7.0 deg is
  94% of what is physically possible and PPO seed 1's 7.4 deg is 99%. The most PPO can ever win
  is ~0.5 deg. Several seeds tying at exactly 7.4 was them hitting the ceiling.
- Resets narrowed to +-8 deg, seed 1: recovers 7.4 deg, tracks LQR almost exactly, despins by
  ~1 s, then drifts to a 9.5 rad/s park. Training crashed at iteration ~300: the adaptive
  learning-rate schedule had climbed to its 0.01 cap, one update broke the critic (value loss
  5 -> 775), reward fell from ~1100 to -43.
- **Resets +-8 deg, 10 seeds** (`results/figures/seeds_r12_vs_r8.png`). Parked arm speed
  tightened from -21..+39 rad/s (5 seeds, +-12) to -3..+10.5 (9 seeds), and the policies now
  despin then drift to a small park; still 0 of 10 under the 1 rad/s bar (best: +1.1). Body:
  five seeds at 7.2 to 7.4 deg (at or above LQR), but seed 3 never balances and seed 8 holds only
  4 deg. Learning curves are not smoother: every seed still crashes and recovers, and seed 3's
  final iteration lands in a dip. Prediction ("smoother curves") wrong. Seed 8 first died of GPU
  out-of-memory: export processes ignored SIGTERM and piled up; now killed with -9 as fallback.
- MRV URDF (`urdf/mrv.urdf`) written from the Fusion numbers. Construction: leg 0 to 280 mm,
  100 mm cube with its top flush with the leg top (centre 230 mm), arms on the side faces.
  That puts the COM at 221 mm, not the 135 mm logged on 09-25; Fusion was probably missing
  masses. mgl 0.777 N.m, inertia about the foot 1.93e-2.
- **Why training crashes** (`results/figures/training_r8_vs_r12_s1.png`). rsl_rl's "adaptive"
  schedule raises the learning rate whenever an update changes the policy less than a KL target.
  A converged policy barely changes, so the rate climbed to its 0.01 cap (10x the start), sat
  there, and one large update broke the critic. PPO's clip did not stop it: each batch gets 20
  gradient steps, so at a high rate the policy can still move far. Because training stops at a
  fixed iteration, each seed is frozen wherever it is in this crash-and-recover cycle, which is
  a plausible source of the seed spread. Next batch: fixed learning rate 1e-3, one change.
- **Plant decision.** MRV is not harder to simulate (same three links, two joints, no
  collisions), so compute is not the reason. The method is developed on the benchmark plant,
  whose limit is known (7.48 deg), and then applied to MRV, so that each step changes one thing.
  MRV also needs a motor model whose torque falls with speed before it can be compared fairly.
- **Considered, not yet tried: residual RL** (Johannink et al. 2018; Silver et al. 2018):
  torque = LQR + a learned correction, so training starts from a controller that already
  despins. It would change the question to "can learning extend LQR's boundary?".
- **Reading:** Block, Astrom and Spong, *The Reaction Wheel Pendulum* (2007) covers this
  device's classical control (friction, saturation, swing-up); no RL. Ng, Harada and Russell
  (1999) for reward shaping that provably leaves the best policy unchanged.
- **Fixed learning rate 1e-3, resets +-8, 10 seeds** (`results/figures/training_lr_s1-5.png`,
  `seeds_r12_r8_lr.png`). Prediction ("fewer crashes") right: no crash in any curve. Learning
  starts later (iteration ~150-200 vs ~115) and is still rising at 500, so 500 may now be too
  short. Every seed balances (none at 0), body 4.0 to 7.4 deg (five at 7.2 to 7.4). **First
  policies that meet the full definition:** seed 1 holds 7.4 deg and parks at -0.4 rad/s, seed 10
  holds 7.2 deg at -0.7 rad/s, both under the 1 rad/s bar (Lab only, ROS check owed). The other
  parked speeds spread wider again (-28 to +33). Seed 5's export first failed on a network fetch
  for a default Isaac asset; re-exported.
- **MRV limit and the bench contradiction.** COM measured at ~210 mm above the foot pivot. With
  the 8 V motor line (torque falls to 0 at no-load, up to 2x stall braking), the linear-program
  limit is 2.8 deg from rest and 4.4 deg with the arm pre-spun the bench way. Re-enacted in Isaac
  Lab (`rl/mrv_bench.py`, full nonlinear PhysX, open-loop full reverse): 2.75 and 4.25 deg. The
  two methods agree, so the maths is not the problem. The bench test recovered ~12 deg (jig with
  bearings at the foot, protractor, one arm, released at rest). 12.4 deg is the momentum ceiling
  for an infinitely strong motor (I_a x 117 rad/s / sqrt(mgl x I_p)), so no motor torque explains
  it. Open. Remaining suspects: something outside the model helping, the arm inertia, the angle.
  (The re-enactment's first scoring counted traces after the env's auto-reset as recoveries and
  said 14 deg; fixed to stop at the first fall. Same bug as lab_eval on 10-09 morning.)
- **Potential-based arm-speed shaping, k = 0.25: failed, stopped after seed 1.** The policy never
  learned to balance (episodes ~45 steps all 500 iterations) and falls from every lean with the
  motor nearly idle. Mechanism: the shaping charges k * w^2 the moment the arm spins up, and
  every catch needs a spin to ~60 rad/s, so k = 0.25 bills ~900 up front against an alive bonus
  of 1 per step. The refund comes later as the arm drains, but an untrained policy never gets
  that far, so it learns that using the arm is expensive and stops using it. Invariance of the
  best policy (Ng 1999) says nothing about whether training can find it. Prediction (5 of 10
  despin) not tested; 0 of 1.
- **Shaping sweep: no k works** (`results/figures/seeds_shaping_sweep.png`, seeds 2-4, which did
  not despin unshaped). k = 0.001 and 0.003: no despin, parked speeds as scattered as before, and
  seed 3 holds only 3-4 deg (6.5 unshaped). k = 0.01 and 0.03 (one seed each, the rest cut to
  save time): 5 and 4 deg, still parked at ~15 rad/s. The body boundary falls as k rises and the
  arm never gets closer to zero: any up-front charge for arm speed weakens the catch before it
  buys any despin.
- **Gated shaping (k = 0.25, active within ~2 deg and 0.2 rad/s of upright and still): never
  learns to balance, all 3 seeds.** My gate design was wrong. A gated potential is lowest
  (most negative) when the body is upright, still and the arm is spinning, which is exactly where
  every catch ends. Leaving that state, by leaning away, raises the potential back towards 0 and
  pays up to +900. So the shaping rewarded tipping over with a spinning arm. Still policy-
  invariant in theory; in training it is a large signal pointing the wrong way.
- **Conclusion for shaping on arm speed:** the catch must spin the arm up, so any potential in
  w_arm either bills the catch (ungated) or rewards abandoning upright (gated). Dropped.

## 2026-10-09 evening: shaping dropped, overnight batch

- **His read: PPO looks like the wrong tool for despin.** Agreed as a view, with a caveat: the
  failure is specific. PPO matches LQR and nearly reaches the physical limit on the catch. What it
  cannot do reliably is finish the despin, because the arm parks on stable speeds that cost almost
  nothing per step, and draining them pays off seconds later, beyond what gamma 0.997
  (horizon ~2.8 s) values. Next method is for after the statement, not before.
- **His idea: tie arm and recovery into one reward instead of separate terms.** Research found:
  - DeepMind Control Suite (Tassa et al. 2018) multiplies terms: cartpole is
    `upright * centered * small_control * small_velocity`, each a tolerance in [0, 1]. A product is
    a soft AND: no term can be traded off against another.
  - Additive rewards let one term be ignored if others compensate; a geometric mean of terms
    reportedly gave better training and lower variance (quadrotor control, arXiv 2012.06656,
    from the abstract-level summary, not read in full).
  - Constraints as Terminations (CaT, Chane-Sane et al. 2024, LAAS): treat "arm despun by the end"
    as a constraint, violations raise a termination probability, works with plain PPO.
  - Risk for this plant: a hard product `bell(theta) * bell(w_arm)` scores every catch near zero,
    since each catch must spin the arm, the same trap as ungated shaping. dm_control softens it
    with factors like `(1 + small_control) / 2`, so the worst a term can do is halve the reward.
    Not tried. Recorded as the first candidate if the RL work resumes.
- **Overnight batch launched 20:20 UTC, no reward changes** (`queue11.sh` in the scratchpad):
  ROS harness bisection of lr_s1 and lr_s10 (the two that despun in Lab); fixed-LR seeds 11-20 at
  500 iterations (despin rate out of 20); seeds 1-10 at 2000 iterations (does longer training
  leave the parking habit). No prediction written before launch.
- **ROS harness, the two despinning policies: both despin, neither beats LQR.** lr_s1: recovers
  6.98, fails 7.03 deg, arm -0.3 to -0.4 rad/s at 10 s. lr_s10: about 6.33-6.4 deg, arm -0.78 rad/s.
  The repeats flipped on both, so the edges are fuzzy at the 0.05 deg scale. LQR on the same
  harness: 7.03 / 7.08. In Lab the same policies held 7.4 and 7.2. **This is the first Lab-vs-ROS
  boundary comparison for PPO.** The earlier lqrrew seeds only ran the 3 deg sanity check in ROS,
  which they failed on despin. Gap 0.4-0.8 deg, unexplained. Hypothesis, not tested: the ROS loop adds latency
  (a timer node acting on the last stashed state) that Lab's in-loop evaluation does not have, and
  a policy riding the physical limit loses more to it than LQR does. Cheapest test: LQR in Lab.
  If it also shows ~7.4, the comparison is like for like in each simulator and PPO ties LQR.
- **LQR in Lab: 7.46 deg, identical to PPO's best. Correction to everything I said about PPO
  "reaching 99% against LQR's 94%".** That compared PPO in Lab against LQR through ROS. Like for
  like (`rl/lqr_policy.py` exports the LQR law as a TorchScript policy, run by the same
  `rl/lab_eval.py`, fine grid 7.2-7.58 deg): LQR holds 7.46 and falls at 7.48; lr_s1 holds 7.46 and
  falls at 7.48; lr_s10 holds 7.3 and falls at 7.4. The physical limit is 7.48. So in Lab LQR is
  already at the limit, and the best PPO seed ties it to within 0.02 deg. Through ROS both lose
  ~0.45 deg (LQR 7.03, lr_s1 6.98), consistent with the loop latency hypothesis, untested.
  `analysis/plot_seeds.py`'s LQR line is now the Lab number.
- **Statement result, now:** under identical limits, PPO matched LQR and did not beat it, because
  LQR on this plant already recovers 99.7% of the physically possible lean (LP bound). Nothing is
  left to win on the catch; the only open question was despin, which PPO does in 2 of 10 seeds.
- **Product reward added** (`reward_type = "product"`, `rl/rewards.py`): `bell(theta, 6 deg) *
  (1 + bell(w_arm, 10 rad/s)) / 2`. My parameter calls: arm width 10 rad/s so parking at 30 rad/s
  pays ~half and despun pays ~all; the (1 + x)/2 floor so a catch at 100 rad/s costs at most half
  and never zeroes the reward; no control or body-speed term (one change at a time). Queued after
  the overnight batch as `queue12.sh`, seeds 1-10, 500 iterations, base config otherwise.
- **Unit tests added** (`tests/`, run with `PYTHONPATH=. ~/isaacsim/python.sh -m pytest tests -q`,
  14 pass): product reward bounds, floor, multiplicative structure, batch shape; LQR policy gains
  match `pendulum_controller.py`'s declared defaults (read from its source), torque from
  limits.yaml, same output as the node's clamp law, TorchScript round trip; plant model
  reproduces body.urdf's hand-derived A and B and MRV's 358 g / 210 mm. Product reward smoke-tested
  in Lab at 64 envs, 3 iterations, before queuing; queue12 repeats the smoke test at full scale
  and aborts if it fails. Unknown `reward_type` values now raise instead of silently using "bell".
- **Decision (his): the PPO-vs-LQR part is complete after tonight's runs.** No MRV training or
  MRV LQR runs; the MRV URDF, plant model and bench re-enactment stay as built. The bench
  discrepancy (12 deg measured vs 4.4 deg modelled pre-spun) is left open and noted as such.

## 2026-10-10: overnight results (`results/figures/seeds_overnight.png`)

- **500 iterations, seeds 11-20: none despun.** Total 2 of 20 (seeds 1 and 10). His prediction
  ("one or two more, out of luck"): 0 more. Each seed parks at the same arm speed from every start
  lean, so the parked speed is a property of the policy, not of the recovery.
- **2000 iterations, seeds 1-10: every seed holds 7.4 deg, and arm speeds collapse towards zero**
  (-15.6 to +10.5, 7 of 10 within 5 rad/s, against -28 to +33 at 500). Only seed 10 is under the
  1 rad/s bar (+0.2). Seed 1 got worse (-0.4 at 500, -2.7 at 2000). So longer training removes the
  body-boundary spread entirely and drifts the parked speed towards zero, slowly; the drift is not
  monotone per seed.
- **Product reward, seeds 1-10: no better.** 1 despun (s8, -0.4; s1 close at +1.3); 6 hold 7.4;
  s2 never balances, s6 holds 3, s9 holds 4. Worse on the catch than the LQR-cost reward, no better
  on despin. Not pursued.
- **The part is closed (his call, 10-09).** Result for the statement: under identical limits PPO
  matches LQR at the physical limit (7.46 of 7.48 deg in Lab) and does not beat it; despin
  is unreliable: 2 of 20 seeds at 500 iterations, 1 of 10 at 2000.
- **Seed 10 at 5000 iterations** (picked as the best seed, so not evidence of a rate): Lab holds
  7.4 deg, arm +0.3 rad/s from every lean (2000 its: +0.2, so no change). ROS harness: recovers
  6.33, falls 6.375 deg (repeat recovered at 6.375, so the edge is fuzzy), despun to +0.2-0.3 rad/s
  every recovery. The same bracket as the 500-iteration seed 10 through ROS, and ~0.7 deg below LQR's
  7.03. The extra training bought nothing measurable. His prediction: stays despun (right),
  marginally better (no), ROS ~7.03 like LQR (no, 6.33).
