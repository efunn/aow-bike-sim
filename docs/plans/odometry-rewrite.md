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

## The encoder path, closed 2026-08-27

`SimOdometry` read the joint-VELOCITY sensors: instantaneous, unquantised,
unfiltered. The Pi differences `Present Position` counts through `RateFilter`.
That gap is now modelled (`encoder="counts"`), and it costs nothing.

**One count is 0.236 mm of travel at the wheel** — 2*pi/4096 rad at the servo,
times belt_ratio 3.0, times the 0.0512 m rolling radius. As a VELOCITY it is
worth q/T where T is the DIFFERENCING SPAN, not the sample period, so sampling
faster does not shrink it:

| span | quantisation |
|---|---|
| 10 ms | 23.6 mm/s |
| **25 ms** (the default) | **9.4 mm/s** |
| 50 ms | 4.7 mm/s |

Open-loop, per regime, `general_rl_odo` driving (RMS against truth, mm/s):

| regime | v_lon ideal → counts | v_lat ideal → counts |
|---|---|---|
| standstill | 4.4 → **34.6** | 3.1 → 6.6 |
| forward 0.6 | 15.4 → 16.0 | 10.9 → 11.6 |
| reverse -0.4 | 11.1 → 16.6 | 4.5 → 5.6 |
| crab 0.3 | 4.9 → **29.0** | 8.4 → 17.4 |

Quantisation costs ~30 mm/s exactly where the wheel barely turns, and nothing
while driving, where slip already dominates at 122 mm/s.

**On the eval grid it costs NOTHING.** `general_rl_odo`, trained on the ideal
encoder, scores **0.771 / survival 1.00 on `counts`** against 0.766 / 1.00 on
`ideal` — and drifts LESS (0.478 m against 0.919). No retraining needed.

**Because "ideal" is not the better sensor, it is the UNFILTERED one.**
Measured on one shared trajectory, `counts` has 10.8 mm/s of spread against
`ideal`'s 15.2: the 25 ms `RateFilter` removes more than the quantisation puts
in. `ideal` is a floor on ERROR, not on noise, and `counts` trades variance for
lag. That is why the eval score moves the "wrong" way.

### The pen says the drift is the POLICY, not the encoder

`analysis/pen_odometry.py --policy general_rl_odo --encoder ideal counts`,
holding station 12 s (`analysis/plots/pen_odometry_odo_encoder.png`):

| encoder | mode | final drift | path length |
|---|---|---|---|
| – | truth | 764.0 mm | 770.3 mm |
| ideal | front | 758.5 | 771.9 |
| ideal | lon_only | 752.8 | 772.7 |
| counts | front | **574.9** | 946.5 |
| counts | lon_only | 594.4 | 900.8 |

**Truth drifts 764 mm too**, so the encoder is not the cause — and `counts`
DRIFTS LESS than `ideal` while wandering more (path 946 against 772). The
filter's smoothing costs path length and buys net displacement.

The cause is a steady-state velocity error in the policy: on a zero command it
drives away at a constant **−64 mm/s**, in a near-straight line. Reading the
observation next to the truth settles it — on `truth` the policy SEES
−64.2 mm/s and does not correct. It is not nulling a biased estimate; it simply
accepts the error. On `counts` the true creep is smaller (−48.1 mm/s) because
the estimate reads −62.9 and the policy pushes back harder against a number
that is wrong in the helpful direction.

Against the other policies, on truth: `general_rl_smooth_diff_pi` 341.8 mm with
a 1073.9 mm path (jitters, stays put) and `general_rl_nolat` 273.1 / 329.8
(quiet AND still). So `odo`'s drift is real and it is roughly 2.2x
`smooth_diff_pi`'s. **That is the price of this policy**, and it is the
`drift_m 0.956` the eval grid reports, now with a mechanism.

ONE HOLD IS NOT AN EVAL, and this nearly misled: every policy survives this
command, including the truth-trained one, which looks merely mediocre here
(620 mm on counts) while scoring **0.010 with survival 0.05** on the
20-command grid. The pen shows the SHAPE of a path; the grid decides.

### The servo's OWN velocity estimate is not free — and the lag budget is ~20 ms

`encoder="reported"` models Present Velocity(128), the register
`ServoBus(velocity_source="reported")` takes wholesale. Same encoder counts,
different filter: the servo smooths like a **~50 ms boxcar** (25.0 ms of group
delay) against our 25 ms / taper 0.5 (8.3 ms). `hw/dynamixel.py` re-derives
velocity from position specifically to avoid it, and that choice was argued
from open-loop lag. Closed loop, on the eval grid:

| encoder | filter | lag | score | surv | vel_err | drift_m | steer_rest |
|---|---|---|---|---|---|---|---|
| `ideal` | none | 0.0 ms | 0.766 | 1.00 | 0.103 | 0.919 | 0.5° |
| `counts` | 25 ms / 0.5 | 8.3 ms | **0.771** | **1.00** | 0.094 | 0.478 | 1.2° |
| `reported` | 50 ms / 1.0 | 25.0 ms | **0.573** | **0.85** | 0.228 | 1.300 | 5.6° |

**Three of twenty episodes.** The design decision is now backed by a
closed-loop number rather than by an RMS argument.

Sweeping the filter span on the same encoder path finds the cliff:

| lag | score | surv |
|---|---|---|
| 8.3 ms | 0.771 | 1.00 |
| 12.8 | 0.761 | 1.00 |
| 15.0 | 0.763 | 1.00 |
| 17.2 | 0.737 | 1.00 |
| **20.0** | 0.713 | **1.00** |
| 21.7 | 0.676 | 0.95 |
| 25.0 | 0.573 | 0.85 |
| 40.0 | 0.400 | 0.70 |

**Survival holds to 20 ms and first breaks at ~22.** Score decays gently below
that (−8% from 8 to 20 ms) and falls away sharply above. So the lag budget is
~20 ms and our 8.3 sits at less than half of it — which is the precise answer
to "does 8 ms matter": no, and there is 2.4x of headroom before it does.
Consistent with a 113 ms fall time constant: the cliff is at ~18% of it.

One seed per point, so read the trend rather than any single row (`drift_m`
bounces between 0.15 and 1.25 without a clean ordering).

### And the estimator now has its own clock

It used to inherit whatever its caller looped at — **50 Hz** from `GeneralEnv`
(`ctrl_dt`), **2500 Hz** from teleop (which passed `model.opt.timestep`),
against the Pi's **100 Hz**. Three callers, three different estimators, since
`VelocityEstimator` integrates. Now it ticks at `odo_hz` (default 100) and
holds its value in between, exactly as a reader between sense ticks does on the
bike.

**The rate turns out barely to matter**, which is the reassuring answer:
`general_rl_odo` max roll across a 50x range of tick rates —

| odo_hz | standstill | fwd 0.6 | rev -0.4 | crab 0.3 |
|---|---|---|---|---|
| 2500 | 0.4° | 1.8° | 0.3° | 1.0° |
| 100 | 0.4° | 2.1° | 0.2° | 1.6° |
| 50 | 0.4° | 2.0° | 0.3° | 1.7° |

**This closes the caveat left open below.** The lag-tolerance result was
LQR-land and explicitly not extended to a policy; it now is, measured on the
policy that is actually velocity-sensitive. Note it says nothing about gyro
bias or AHRS error, which remain unmodelled.

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

## THE AHRS IS THE DOMINANT SENSOR RISK, not the encoder

`sim_ahrs.py` puts the TM151's datasheet error on the ORIENTATION path -- roll,
roll_rate, yaw_rate, which are observation entries 0, 1 and 2 and the fast
loop. The velocity work above was worth 3 episodes of 20 at its worst. This is
worth more, and it lands on a policy that never trained against it.

Eval grid, 20 commands, randomization off. `general_rl_odo` on its own encoder
model; the other two on truth velocity, so these rows isolate the AHRS:

| level | part, condition | roll/pitch RMS | `general_rl_odo` | `smooth_diff_pi` | `nolat` |
|---|---|---|---|---|---|
| `none` | — | — | 0.766 / **1.00** | 0.808 / **1.00** | 0.780 / **1.00** |
| `tm151_static` | ours, bench | 0.5° | 0.734 / **1.00** | 0.261 / **0.45** | 0.491 / **0.70** |
| `tm171` | upgrade, moving | 1.0° | 0.689 / **1.00** | 0.126 / **0.25** | — |
| `tm151` | **ours, moving** | 1.5° | 0.537 / **0.90** | 0.110 / **0.20** | 0.149 / **0.25** |

Every level names a PART and a CONDITION. An earlier version called these
`static` and `typical`, which left "is `typical` the TM151?" a fair question
with no answer in the name. It is: `tm151` IS the part we have, at the dynamic
accuracy a moving bike gets, and it is the row to design against.

**The bench figure alone halves the standing default.** `smooth_diff_pi` goes
from clearing the grid to 0.45 survival on the 0.5° STATIC accuracy — the
number a stationary calibration would report. At the dynamic 1.5° it holds 0.20.

**`general_rl_odo` is far more robust, and nothing trained it to be.** It holds
1.00 through `tm171` and only loses two episodes at `typical`, while carrying
the encoder model as well. The plausible reading is that a policy trained on a
wrong-but-correlated velocity learned not to over-trust a sensor, and that
transferred to a different noisy channel — but that is a hypothesis, not a
measurement. It is also 6x quieter and saturates 1.2% of steps against 50.6%,
so "less reactive" explains it equally well.

**Orientation error is the damage, not gyro noise** (`--ahrs-channels`, at
`tm151`):

| channel | `general_rl_odo` | `smooth_diff_pi` |
|---|---|---|
| `gyro` only | 0.768 / 1.00 | 0.222 / 0.40 |
| `orient` only | 0.583 / 0.95 | 0.130 / 0.25 |
| both | 0.537 / 0.90 | 0.110 / 0.20 |

That is the actionable split: orientation is what a better part fixes, and the
`tm171` row prices it.

### Which TM151 → TM171 difference actually buys that

The parts differ in internal update rate (400 vs 800 Hz), gyro non-linearity
(0.3 vs 0.2 % FS), accel misalignment (0.5 vs 0.3°), yaw (1.0 vs 0.8° static,
3.0 vs 2.6° per 25 min) — **and in dynamic roll/pitch, 1.5 vs 1.0°**, which is
the easy one to miss because the STATIC block is identical (<0.5° both) and
hides it. Ablated on the grid, `general_rl_odo`:

| configuration | score | surv |
|---|---|---|
| TM151, everything as spec | 0.537 | 0.90 |
| TM171, everything as spec | 0.689 | 1.00 |
| TM151 but TM171 **orientation RMS** only | **0.635** | **0.95** |
| TM151 but TM171 yaw drift only | 0.542 | 0.90 |
| TM151 but TM171 misalignment only | 0.519 | 0.85 |

Orientation RMS carries it; yaw drift and misalignment do nothing against a
seed noise floor of about ±0.02. Update rate is irrelevant — the Pi senses at
100 Hz and both parts run 400 Hz or better internally. Gyro non-linearity is
quoted as % of the ±1000°/s FULL SCALE, and the bike peaks at 22.6 / 29.8 /
34.7 °/s, under 3.5% of it, so the deviation there is a small fraction of the
3°/s worst case. Recorded and not modelled. For `odo` the upgrade buys back full survival
(0.90 → 1.00). For `smooth_diff_pi` it buys almost nothing (0.20 → 0.25) —
**you cannot buy your way out of a policy that trusts its attitude.**

### The correlation time is a GUESS, and it was swept before this was believed

`TAU_ORIENT_S = 2.0` has no datasheet source. Sweeping it at `tm151`:

| tau | 0.1 | 0.5 | 1 | 2 | 5 | 20 | 60 |
|---|---|---|---|---|---|---|---|
| score | 0.514 | 0.447 | 0.506 | 0.438 | 0.503 | 0.613 | 0.624 |
| surv | 0.85 | 0.80 | 0.85 | 0.80 | 0.85 | 0.95 | 0.95 |

Flat across 0.1–5 s and only eases beyond 20 s, where the error becomes a
near-constant offset the controller trims out. **The guess does not drive the
conclusion**, which is the only reason the table above is quotable.

### What this does NOT yet say

Nothing here has retrained. The obvious next move is the one that worked for
velocity: put `ahrs_level: tm151` in the env during training and see whether
a policy learns to distrust its attitude the way `odo` learned to distrust its
velocity. That is a training run, not an analysis.

## Mounting position: one real answer, one thing the model cannot see

Asked 2026-08-27, because it had been settled "theoretically and with some
basic tests" before and was worth checking against the sim. Five probe sites on
ONE chassis over ONE trajectory, which isolates the lever arm from the chaotic
divergence that merely moving the 12 g sensor causes:

| position | max \|gyro − origin\| | RMS \|accel − origin\| |
|---|---|---|
| origin `[0,0,0]` | **0.00e+00** °/s | 0.0000 m/s² |
| as-built `[.05,0,.13]` | **0.00e+00** | 9.87 |
| high mast `[.05,0,.30]` | **0.00e+00** | 21.47 |
| far forward `[.20,0,.13]` | **0.00e+00** | 16.92 |
| off-axis `[.05,.10,.13]` | **0.00e+00** | 10.01 |

**The gyro is mount-independent, exactly** — to machine precision, because
angular velocity is a property of the rigid body. `roll_rate` and `yaw_rate`,
observation entries 1 and 2, do not care where the unit goes. That is a real
result and it is the one that matters most, since those are fast-loop entries.

**The accelerometer cares a great deal.** The lever-arm terms `α×r` and
`ω×(ω×r)` are the same order as gravity (against a 16.1 m/s² signal at the
origin) and grow with the arm. Not new: `hw/odometry.py` already records a
τ=0.3 s accelerometer blend degrading `v_lon` from 8.8 to 174 mm/s RMS for
exactly this reason, which is why the accelerometer there is a fallback.

**But the orientation output is mount-independent HERE BY CONSTRUCTION, and
that is a limitation rather than a finding.** `sim_ahrs` applies a fixed
datasheet RMS whatever `bike.ahrs.pos` says. A real unit fuses the gyro against
the accelerometer AS A GRAVITY REFERENCE, so lever-arm acceleration corrupts
that reference and a badly-placed unit should read worse than its datasheet
number. The datasheet says as much: footnote [3] warns that parts without
vibration resistance are "susceptible to low frequency linear acceleration",
and the dynamic figure is quoted for "typical low-dynamic movements... indoor
robotic vehicles, low-speed driving". A balancing bike with the IMU on a mast
is not obviously inside that envelope.

So a flat eval across mounting positions would prove nothing. Settling it needs
a fusion model or a bench measurement on the real part. **Open.**

### The magnetometer gives heading authority, not balance

**No magnetometer is modelled** — there is none in the MuJoCo model, and yaw
comes from the true quaternion plus a drift term. But the QUESTION it settles
can be asked of the model directly, by injecting yaw error and roll/pitch error
separately.

The physics first. Roll and pitch are bounded by **gravity**, sensed by the
accelerometer — and rotating about the gravity vector does not change what the
accelerometer reads, so it says nothing about yaw. Yaw is bounded by the
**magnetic field**, and nothing else on the bike provides an absolute heading.
Two orthogonal references, applied to orthogonal corrections, which is why the
datasheet quotes roll/pitch and yaw accuracy on separate rows and describes an
"adaptive magnetic field filter" that "resists magnetic interference" rather
than one that keeps interference out of the attitude.

So the TM151 has two yaw regimes: **magnetometer-aided** (static <1.0° RMS,
bounded) and **pure inertial** (the "Dynamic accuracy (Inertial)" row, quoted
as a DRIFT of 3.0° every 25 minutes because nothing bounds it). Losing the
compass moves you from the first to the second.

Measured, `general_rl_odo`, injecting each axis alone:

| injected | score | surv | head_deg |
|---|---|---|---|
| none | 0.774 | 1.00 | 25.2 |
| TM151, all axes | 0.615 | 1.00 | 32.1 |
| **roll/pitch only, yaw perfect** | 0.544 | **0.90** | 38.9 |
| yaw only, 1° (datasheet) | 0.751 | 1.00 | 27.2 |
| yaw 10° (bad magnetic siting) | 0.632 | 0.95 | 28.9 |
| yaw 30° (compass useless) | 0.500 | **0.90** | 40.1 |

**Roll/pitch alone reproduces the falls**; that is the balance channel, and it
is the accelerometer's, not the magnetometer's. At the datasheet's 1° yaw costs
almost nothing (0.751 against 0.774, survival intact) and what it costs is
tracking.

**But large yaw error does eventually cost falls, by an indirect route worth
knowing.** Yaw never enters the roll loop — it is consumed only by
`set_velocity` (body→world) and `command_to_body`. That second one is the path:
`psi` rotates the WORLD velocity command into the body frame, so a 30° heading
error resolves "drive forward" as partly lateral. The bike crabs while it
believes it is going straight, crabbing spends the rear differential, and the
differential is exactly the channel that catches roll. Balance headroom is
consumed by a heading mistake without any sensor lying about roll.

So: keep it away from the coils, and below ~10° it is a tracking problem rather
than a stability one.

**Caveat on the filter.** That the magnetometer stays out of roll/pitch is a
property of a well-built fusion filter, not a law. One fusing both reference
vectors symmetrically can leak magnetic error into attitude. The TM151's
internals are not documented at that level and this cannot check them.

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
4. Does AHRS mounting position degrade the ATTITUDE, not just the
   accelerometer? The model cannot answer it (see above). A bench measurement
   on the real TM151 would. **The chassis origin is not one of the arms** — it
   is the rear axle centre, inside the wheel, and an earlier draft of this item
   proposed it. The realisable spread is CoM `[.083,0,.073]` (0 mm from the
   CoM) against as-built `[.05,0,.13]` (66 mm) against a deliberately bad mast
   (229 mm) — 16 m/s² of spurious specific force between the ends, which
   should be ample to separate. Cheap once there is a bike to mount it on.

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
