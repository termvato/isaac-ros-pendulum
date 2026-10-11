# Predictions

Each prediction was written down before the results it predicts, unless noted, and committed
with a timestamp, so it can be checked against the outcome. Quotes are tidied for spelling and
grammar only; the originals are in the git history of `results/predictions_2026-10-09.md`.

Reference at the time: LQR at 0.025 N.m recovered 7.03 deg through the ROS harness. That value
was later superseded by the harness fix; the corrected figure is 7.45 deg.

## 2026-10-09: first PPO runs

> The cartpole env with 10 environments was balancing pretty well in 150 iterations. With 4096
> and 500 iterations I think it will be pretty strong as well. I think the lean will not vary
> much across the 5 seeds, as the policy will learn to use whatever its initial conditions are
> to push in the right direction, though it depends on the seeds. I think our 5 PPOs will beat
> LQR, simply because we did not tune LQR at all, and there might be room to improve by
> saturating faster, which means using more of the torque.

**Outcome: wrong.** PPO tied LQR at 7.45 deg; LQR was already within 0.02 deg of the physical
limit, so there was no room to saturate faster. The seeds varied widely (next entry).

## 2026-10-09: seed spread

Written after seeing seed 1's learning curves, before any bisection.

> I think the spread between seeds is going to be around 1 degree.

**Outcome: wrong.** About 2 deg on the first reward (7.4, 5.5 and 6.5 deg); later 3 to 7.4 deg
across 20 seeds.

## 2026-10-09: LQR's cost as the reward (seeds 1-3)

> I think yes, they beat LQR. If they beat it without despin, despin comes after the hardest
> challenge the controller faces; it is cleaning up. Predicted result: PPO with LQR's despin
> reward gets the best of both worlds, recovering fully from 7.4 degrees and despinning the arm.

**Outcome: half right.** The best seeds held 7.4 deg, but none of 5 despun.

## 2026-10-09: starting leans narrowed from +-12 to +-8 deg (10 seeds)

> I think the learning graphs are going to be much smoother. This is almost like curriculum
> learning.

**Outcome: wrong.** Every seed still crashed and recovered during training. Parked arm speeds
did tighten. No prediction on despin: 0 of 10.

## 2026-10-09: fixed learning rate instead of the adaptive schedule (10 seeds)

> I think the fixed learning rate will produce fewer crashes.

**Outcome: right.** No crashes in any of the 10 seeds, and the first two seeds to fully despin.

## 2026-10-09: the benchmark LQR gains on the MRV model

> I think it falls from around 2 deg, matching my difficulty balancing it in real life.

**Outcome: not run** (MRV runs were dropped). For reference, the linear-program limit for any
controller on the MRV model is 2.8 deg from rest.

## 2026-10-09: potential-based shaping on arm speed, k = 0.25

> I think 5 out of 10 despin.

**Outcome: wrong.** Stopped after seed 1, which never learned to balance at all.

## 2026-10-09: shaping sweep (k = 0.001 to 0.03) and gated shaping

No prediction recorded. **Outcome:** no despin at any k; gated shaping never balanced.

## 2026-10-09: seeds 11-20 at 500 iterations, seeds 1-10 at 2000 iterations

Written after the batch started, before any of its results.

> I think we might see one or two more that despin out of luck. Maybe we see a pattern across
> them.

**Outcome: wrong on despin, right on the pattern.** No more seeds despun at 500 iterations (2 of
20 in total), 1 of 10 at 2000. The pattern: each seed parks its arm at the same speed from every
starting lean.

## 2026-10-09: multiplicative reward (DeepMind Control Suite style)

No prediction recorded. **Outcome:** 1 of 10 despun, and the catch got worse.

## 2026-10-09: what the policy does at its parked speed

> I bet it's zero torque at the parked speed. The problem is how to shift that towards zero.

**Outcome: not tested yet.**

## 2026-10-10: seed 10 trained for 5000 iterations

Written while it trained.

> It stays despun but gets marginally better. ROS puts it at 7.03 degrees, around the same as
> LQR.

**Outcome: right, once the harness was fixed.** Still despun (+0.3 rad/s); 7.36 to 7.45 deg in
ROS; equal to LQR, which is also 7.45 deg on the fixed harness (the 7.03 I quoted was LQR's
value on the old harness).
