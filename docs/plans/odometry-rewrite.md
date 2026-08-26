# Odometry in the loop, and what it takes to rewrite the estimator

Started 2026-08-26. Status: **measured, and FLOWN AROUND rather than
rewritten.** `sim_odometry.py` and `run_drive --odometry` put the estimator in
the loop; `hw/odometry.py` is unchanged in behaviour (its stale roller verdict
was rewritten, nothing else). The estimator is still the one described below
-- what changed is that two policies now survive it, so fixing it is no longer
on the critical path.

---

## Why this is not an offline problem

In sim every controller reads `extract_state(data, ...)` -- MuJoCo ground
truth. On the bike the same call reads `HardwareData`, whose velocity was
written by `hw/odometry.VelocityEstimator` from real sensors. So a policy is
trained against a velocity it will never be given, and the estimator's error
depends on how the bike moves, which depends on the policy. Neither half of
that loop can be judged while they are kept apart.

The seam already existed: `hw/run_bike._sense()` does
`data.set_velocity(body_to_world(v_lon, v_lat, yaw))`, and `drive.py` makes no
MuJoCo calls at all, so it cannot tell truth from estimate. Only TRAINING
bypasses it.

**And nothing new needed simulating.** The model already carries the whole
hardware sensor suite -- `ahrs_gyro`, `ahrs_accel`, `ahrs_quat` on `ahrs_site`
at the IMU's real chassis position, plus the input-shaft and steer encoders.

## Closing the loop drops the bike

RL policy (`general_rl_smooth_diff_pi`), 12000 steps, roll > 60 deg = fell:

| command | on truth | on the estimate |
|---|---|---|
| standstill | roll 2.32 deg | **FELL at 1.10 s** |
| forward 0.6 | roll 1.15 deg | roll 5.02 deg |
| reverse -0.4 | roll 2.77 deg | **FELL at 1.19 s** |
| crab left | roll 3.39 deg | **FELL at 1.03 s** |

**This is the optimistic case.** The sim sensors are CLEAN: no gyro bias, no
accelerometer noise, no encoder quantisation. Every number here is a FLOOR on
the error, not a prediction of it.

So this is not a tuning gap. A policy trained on truth cannot fly on this
estimator at all.

## The rollers are no longer "the estimator that does not work"

`hw/odometry.py`'s docstring records them at 7-23 mm/s RMS, correlation
-0.20..+0.96, over-predicting **2.5-3.8x**, and concludes they are diagnostic
only. THAT VERDICT IS STALE -- it predates a lot of contact-model change.
Measured 2026-08-26, RL policy, per-sample estimates against truth:

| regime | front constraint | roller kinematics | cos^2 blend |
|---|---|---|---|
| standstill | 44.4 / 0.892 | **25.9 / 0.928** | 31.2 / 0.941 |
| forward 0.6 | **14.4 / 0.935** | 23.4 / 0.852 | 14.4 / 0.935 |
| reverse -0.4 | 27.6 / 0.815 | **20.8 / 0.886** | 26.4 / 0.827 |
| crab left | 136.6 / 0.847 | 100.6 / 0.812 | **76.6 / 0.939** |

(RMS mm/s / correlation.) Over-prediction is now 1.11-1.24x, not 2.5-3.8x, and
correlation never drops below 0.81. The rollers BEAT the front constraint at
standstill and in reverse -- exactly where the front constraint is structurally
blind, because `v_lon * tan(theta)` carries no lateral information at
`v_lon = 0`.

Speed-aware weighting (trust the front constraint in proportion to
`|v_lon| / V_REF`, V_REF = 0.25) improves every regime: standstill 31.2 ->
24.6, forward 14.4 -> 13.9, crab 76.6 -> 72.7.

## THE RESULT THAT MATTERS: open-loop accuracy is the wrong objective

Driving the same regimes with each estimator IN THE LOOP:

| command | truth | front | speed-aware blend |
|---|---|---|---|
| standstill | roll 2.32 | FELL 1.10 s | FELL 1.83 s |
| forward 0.6 | roll 1.15 | roll 5.02 | roll 7.19 |
| reverse -0.4 | roll 2.77 | FELL 1.19 s | **FELL 0.88 s** |
| crab left | roll 3.39 | FELL 1.03 s | **FELL 0.79 s** |

**The blend is more accurate open-loop and WORSE closed-loop**, in three of
four regimes. An estimator selected on RMS would have been shipped as an
improvement and would fall off the bike sooner.

The likely mechanism, and it should be tested before the rewrite assumes it:
`roller_lateral = lat_per_d * (w_a - w_b)`, and the differential is EXACTLY
what the balance controller commands. So the roller estimate is largely a
function of the control INPUT rather than of the outcome. Open-loop it
correlates well because the command does produce the motion; closed-loop the
controller is partly measuring its own action. RMS cannot see that.

## Filtering: tested, and it hurts

A first-order low-pass on the blend costs correlation at every setting and in
every regime -- alpha 0.20 takes forward 0.938 -> 0.874 and crab 0.939 ->
0.832; alpha 0.08 is worse. The error is not high-frequency noise to be
smoothed off, it is a wrong instantaneous answer, and the phase lag costs more
than the smoothing gains.

A LEAD term (overshoot, to catch up to a velocity command) trades differently
and was not tested. But the sim sensors are clean, so there is no measurement
noise here for any filter to remove: filtering only becomes a meaningful
question once the noise model below exists. Tuning it now would be tuning
against the wrong signal.

## What the rewrite needs

1. **A closed-loop objective.** Survival and peak roll with the estimator in
   the loop, not RMS against truth. The two disagree, measured above.
2. **A sensor-noise model -- SMALLER THAN IT FIRST LOOKED.** `randomize.py`
   covers mass, friction and actuator strength: plant axes only. Nothing
   models gyro bias, accelerometer noise or encoder quantisation, so the
   in-sim estimator is better than the bike's will ever be, and every number
   in this document is a FLOOR.

   But the scope shrank twice. Encoder quantisation is already handled --
   `RateFilter` exists precisely for it, was swept against ground truth, and
   `DO NOT DEADBAND` in its docstring is the trap that came out of that. And
   the closed-loop result above deflates lag. What is left is the noise that
   feeds the FAST loop, where nothing is averaging it away: **gyro bias and
   AHRS orientation error.** Those are the numbers to go get for the TM151;
   the encoder side is done.
3. **Fusion that is not self-referential.** Whatever weights the rollers has to
   account for the differential being the control input. Options not yet
   explored: use the roller estimate only where the commanded differential is
   small; use it as an innovation against a propagated state rather than as a
   direct measurement; high-pass the command out of it.
4. **Validity gates rather than estimates.** From the session: no sensed v_lon
   means skidding or stopped; an identical differential means no crabbing is
   occurring. Both are cheap and both are things the current estimator does not
   know it knows.

## Resolved

**Does `SimOdometry` need to match the hardware's velocity SOURCE?** It reads
the instantaneous joint velocity; the Pi differences encoder counts through
`RateFilter` (25 ms window, ~8 ms group delay). NO — not for the LQR. Closed
loop, the bike balanced identically on ideal, 50-ms-averaged and differenced
feedback: max roll 1.5-1.7 deg either way. The fast state comes from the AHRS,
and these rates feed only slow outer loops, so the lag has nowhere to do
damage. `hw/dynamixel.py`'s module docstring records the same result from the
other side.

**THAT RESULT IS LQR-LAND, AND DOES NOT AUTOMATICALLY EXTEND TO A POLICY.** An
RL policy sees all 15 observation entries flat — there is no "slow outer loop"
for a rate to be quarantined in — and ours is demonstrably velocity-sensitive
on a fast timescale: the estimated v_lat falls over 0.8-1.3 s, and that alone
is enough to drop a truth-trained policy (below). Lag tolerance probably
transfers for v_lon, where the LQR result is most directly about the same
signal. Do not assume it for v_lat, and do not cite the 1.5-1.7 deg number in
a training argument without saying which controller produced it.

## Open questions

1. Why did the FRONT constraint degrade? The docstring records a free fit
   returning tan-coefficient 0.985 and `L_eff` 0.2033 against a geometric
   0.200. Today the same pooled fit gives 0.652 and `L_eff` 0.137. Something
   moved and it is not the formula, which is textbook-correct for a rigid
   bicycle. Suspect the contact model or the reference point.
2. Does the policy need to SEE confidence? `lateral_from_front` already returns
   one. Feeding it costs `OBS_DIM` 15 -> 16 and a `general_spec.py` change, so
   both sides move together. Deferred until the estimator is settled.
3. Pitch: assumed to never change enough to matter. Confirm before spending an
   observation slot on it.

## The sequencing was wrong, and training answered it first

Agreed earlier on 2026-08-26: tune the estimator FIRST, then decide what
states retraining needs. **That is not what happened, and the result is better
than the plan.** Two policies were trained against the estimator as it stands
-- `general_rl_nolat` (never observe v_lat) and `general_rl_odo` (observe the
ESTIMATE during training) -- and both survive it. So the estimator did not
have to be fixed to be flown.

Eval grid, 20 commands, identical seeds, randomization off
(`analysis/chatter.py`). The left block is each policy on the signal it was
TRAINED on; the right block is every policy on the ESTIMATE, which is the
deployment question:

| | trained-on: score / surv | on the ESTIMATE: score / surv |
|---|---|---|
| `general_rl_smooth_diff_pi` (truth) | 0.808 / 1.00 | **0.044 / 0.15** |
| `general_rl_odo` | 0.772 / 1.00 | 0.772 / 1.00 |
| `general_rl_nolat` | 0.780 / 1.00 | 0.742 / 1.00 |

The standing driver loses 85% of the grid to the estimator. `odo` gives up
0.036 of score against it on truth and keeps ALL of it on the estimate.

**`nolat` is not immune, and this corrects a claim made earlier the same day.**
It zeroes v_lat but still takes v_lon from the estimate, so it moves too:
0.780 -> 0.742, and drift 0.211 m -> 1.546 m, a 7x. Zeroing one axis buys
survival, not independence.

**`odo` is also far quieter, by a margin nothing predicted.** Mean squared
per-step action change summed over channels: 0.251 against `smooth_diff_pi`'s
1.566 and `nolat`'s 1.548. At rest it is not close -- 0.003 against 2.049 and
1.805 -- and it sits pinned to an actuator bound 1.2% of steps against 50.6%
and 37.2%. `steer_rest` 0.2 deg against 23.1 and 3.1.

This supports the reading that came out of `nolat`: the chatter was the policy
CHASING A SIGNAL, not a smoothness penalty being too cheap
(`rl_general_smooth_diff.yaml` raised that penalty 5x and did not move it).
But `nolat` REMOVES the signal and stays chattery, while `odo` KEEPS it and
goes quiet -- so "remove the signal" is not the mechanism. The better reading
is that a policy trained on a noisy signal learns how much of it to believe,
and one trained on a clean signal never had to.

**What `odo` costs.** Crab is partly given up (`crab_ratio` 0.35/0.52 against
`smooth_diff_pi`'s 0.49/0.58) and drift is 3x worse (0.956 m against 0.296),
which is the v_lon position error of section 2 arriving as expected. `nolat`'s
apparently excellent crab ratios (0.92/1.11) are an artefact: `crab_head_err`
98.6 deg says it TURNED to face the crab direction and drove forward. Read
the two together or the metric lies.

## Not in scope here

Deciding the default. `control.general_move` still names
`general_rl_smooth_diff_pi`, which does not survive the estimate.

An earlier version of this paragraph said moving it "moves `design_digest` for
a change with no physical content". FALSE, corrected 2026-08-26 -- that
predates the plant/design split. `plant_digest` excludes the whole `control`
subtree; `design_digest` covers only `rate_hz`, `lqr` and `drive.speed_grid`.
Repointing moves NEITHER, measured. It is a one-line change; it is left here
because it is a decision about what should drive, not because it is expensive.

## A trap worth naming

Adding a sensor to `build_model` changes the MODEL but not `bike_params.yaml`,
so NEITHER `plant_digest` NOR `design_digest` would catch it. Policies trained
before and after such a change would look interchangeable and would not be.
