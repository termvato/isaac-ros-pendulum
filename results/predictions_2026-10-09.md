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
