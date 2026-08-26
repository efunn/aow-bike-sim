# Odometry in the loop, and what it takes to rewrite the estimator

Started 2026-08-26. Status: **measured, not rewritten.** `sim_odometry.py` and
`run_drive --odometry` exist; `hw/odometry.py` is untouched.

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
2. **A sensor-noise model.** `randomize.py` covers mass, friction and actuator
   strength -- plant axes only. Gyro bias, accelerometer noise and encoder
   quantisation do not exist, so the in-sim estimator is better than the bike's
   will ever be. Needs real numbers for the TM151 and the Dynamixel encoders.
   Until this exists, every number in this document is a floor.
3. **Fusion that is not self-referential.** Whatever weights the rollers has to
   account for the differential being the control input. Options not yet
   explored: use the roller estimate only where the commanded differential is
   small; use it as an innovation against a propagated state rather than as a
   direct measurement; high-pass the command out of it.
4. **Validity gates rather than estimates.** From the session: no sensed v_lon
   means skidding or stopped; an identical differential means no crabbing is
   occurring. Both are cheap and both are things the current estimator does not
   know it knows.

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

## Not in scope here

Retraining. The obs distribution changes the moment the estimate is fed, so
every policy retrains -- see the separate training plan. Sequencing agreed
2026-08-26: tune the estimator FIRST, then decide what states retraining needs.

## A trap worth naming

Adding a sensor to `build_model` changes the MODEL but not `bike_params.yaml`,
so NEITHER `plant_digest` NOR `design_digest` would catch it. Policies trained
before and after such a change would look interchangeable and would not be.
