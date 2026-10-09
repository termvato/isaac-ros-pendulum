# Predictions, written before seeing any PPO result (2026-10-09)

Verbatim, his words:

> the cartpole env with N=10 was balancing pretty well in 150 iterations. i think with 4096
> and 500 iterations it will be pretty strong aswell. I think the lean will vary not that
> much across the 5 seeds as the simulation will learn how to use whatever its initial
> conditions are to push towards the right direction, i guess it depends on our seeds. I
> think our 5 ppos will beat LQr simply because we did not tune lqr at all and i think there
> might be room to improve by saturating faster which means we can use more of the torque.

Reference: LQR at 0.025 N.m recovers 7.031 deg, fails 7.078 deg (harness, 10-05).

Seed spread, committed 10-09 after seeing seed 1 curves but before any bisection:

> i think the spread between each is going to be around 1 degree.

LQR-cost reward (gamma 0.997, seeds 1-3), committed before results, ~12:00:

> i think yea they beat lqr, if without despin they beat it despin is post the hardest
> challenge the controller will face and is like cleaning up so to say. [Predicted result:]
> PPO with LQR's despin rewards reaps the good from both worlds recovering fully from 7.4
> degrees and despinning the arms

## Batch: resets narrowed to +-8 deg (LQR-cost reward, gamma 0.997, nothing else changed), seeds 1-10

> i think the learning graphs are going to be way smoother which is good. this is almost like
> curriculum learning.

(No prediction given on despin or on the parked arm speeds.)

## Batch: fixed learning rate 1e-3 (schedule "fixed", was "adaptive"), resets +-8 deg, LQR-cost reward, gamma 0.997, seeds 1-10

> i think the fixed learning rate will produce less crashes as you said.

(No prediction on despin, body boundary or seed spread.)
