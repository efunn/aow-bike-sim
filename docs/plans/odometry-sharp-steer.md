# Odometry at sharp steer: the lateral estimate coasts and never stops

Started 2026-10-21, from the assembled bike on `bike.sh mirror`. **Status:
the runaway is FOUND and fixed on the Pi side (an accelerometer sign, below),
and confirmed on the bike. What is left is a slower coasting drift: ~0.1 m/s,
1.3 m over 14 s at full steer, measured once.** Follows `odometry-rewrite.md`,
which is the history; its errors are listed at the end of this file rather
than edited there.

## What the bike showed

**Tilting at sharp steer reads as travel.** Front wheel turned past ~60 deg,
rear wheel not turning, bike tilted by hand to either side: the mirror shows
the bike moving toward that side, and it keeps going.

Three things turned up on the same bench session, all fixed now. None was
the estimator's design; the third was its input:

| symptom | cause | fix |
|---|---|---|
| steering drawn reversed | no steer sign in `BikeBus`; the servo hangs horn-down, so its + is the model's right | `control.onboard.steer_sign: -1`, read and write |
| front wheel spins forward when rolled backward | the mirror drew its spin from `\|v_world\|` | `telemetry.front_roll_speed`, signed along the wheel's heading |
| tilt at full steer "zooms off" to that side | the TM151 reports the gravity VECTOR (-g on z when level); the estimator and the sim both assume specific force (+g up) | `hw/ahrs.parse_combo` negates the accelerometer |

The steer sign also fed `steer_pos` to the estimator, so before 2026-10-21
every hardware `v_lat` taken at a non-zero steer used theta with the wrong
sign. In a turn that doubles the error instead of cancelling it:
`v_lon tan(theta) - yaw_rate L` becomes `-2 v_lon tan(theta)`. No recorded
hardware v_lat from before that date should be trusted.

## The mechanism

`hw/odometry.VelocityEstimator.update`:

    v_lat_pred = v_lat + a_lat * dt
    v_lat      = v_lat_pred + conf * (v_lat_c - v_lat_pred)

`v_lat_c` is the front-wheel constraint, and `conf` is cos^2-weighted on the
wheel's ground heading theta:

| steer | theta (rake 20) | tan theta | conf |
|---|---|---|---|
| 30 deg | 28.5 deg | 0.54 | 0.70 |
| 45 | 43.2 | 0.94 | 0.40 |
| 60 | 58.4 | 1.63 | 0.13 |
| 75 | 74.1 | 3.51 | **0.00** |

At conf 0 the state is pure integration of
`a_lat = accel_y - g sin(roll) cos(pitch) - v_lon yaw_rate`, and **nothing
pulls it back**. There is no leak, so any residual in `a_lat` becomes a
velocity that persists until the wheel straightens.

**The runaway was the accelerometer's sign.** Five bench recordings from
2026-10-17 (`traces/bike/261017-*`) all read z of -8.0 to -9.8 m/s^2 at rest,
and a_y fits -9.7 to -9.9 x sin(roll) over rolls up to +-100 deg. So the part
reports the gravity vector, and `a_y - g sin(roll)` came out as
-2 g sin(roll) instead of 0: 3.4 m/s^2 at a 10 deg tilt. At conf 0 that
integrates with nothing to stop it. The residual over the same recordings, at
|roll| < 30 deg:

| run | residual RMS, as read | negated |
|---|---|---|
| 174039_run_bench_policy | 4.26 m/s^2 | 0.48 |
| 194749_run_bench_policy_battery | 2.99 | 0.46 |
| 201234_run_rearm | 3.34 | 0.53 |
| 174510_run_bench_policy (held level) | 0.05 | 0.02 |

These recordings predate the 2026-10-19 mount-rotation fix. The mount is a
45 deg pitch, so it cannot turn a +g reading into -g on z; the conclusion does
not depend on it. A fresh recording on the current code is still the
confirmation.

**This is a deploy-side bug, not a training one.** The sim's accelerometer is
MuJoCo's, which reads specific force, so training always had the right sign.
No retrain is needed for this part.

**Confirmed on the bike, 2026-10-21** (`traces/bike/261021-181828_run_tilt_steer90`,
47 s, torque off, steer held at -75 to -113 deg, tilts to +-43 deg). The
accelerometer now reads z +9.77 m/s^2 at rest. The gravity-removal residual
averages under 0.1 m/s^2 in every 2 s window. No runaway.

**What is left: the coasting itself, now measured.** At conf 0 the estimate
settled on a steady v_lat of about +0.1 m/s and held it, so the position crept
**1.3 m in one direction over ~14 s**, with the rear wheel still. Each time
the wheel came back toward straight (t 28 s, 38 s), conf rose to 0.94-0.99
and v_lat snapped to 0 within the window. So the drift is bounded by how long
the wheel stays past ~60 deg, and nothing else bounds it.

The fast part is real motion. With the slow offset removed, v_lat follows
roll rate at -0.037 m/rad (correlation -0.64): a point a few cm above the
tyre contact really does swing sideways when the bike tilts. That is the
in-and-out on each tilt, a couple of cm at 20 deg.

The steady offset is what a small residual leaves when integrated with no
leak; 0.05 m/s^2 for 2 s is 0.1 m/s. The candidate sources, which are not
yet separated:

- **Real motion of the sensor.** It sits at [0.0989, 0, 0.0842], so a tilt swings
  it sideways through `alpha x r`. That integrates to about roll_rate x height
  and returns to zero when the tilt stops, moving position by a few cm.
- **Roll error.** g sin(1 deg) = 0.17 m/s^2 per degree of disagreement between
  the TM151's roll and its own accelerometer, for example while its filter
  catches up after a hand tilt.
- **Accelerometer misalignment.** The datasheet gives 0.5 deg, about
  0.09 m/s^2 of bias. That is about 9 cm/s after one second at conf 0, even
  upright. This is a datasheet-derived estimate.

## Why the coasting cannot be fixed on the Pi alone

**Training flies the same estimator.** `gpu/odometry.py:189-203` is
`VelocityEstimator.update` line for line, on the `tm151_filter` AHRS model
(`gpu/ahrs.py`: its own complementary filter on a corrupted gyro and
accelerometer). `general_puffer_fixes_72M` has `obs_odometry: true` and
`odometry_encoder: counts`. So the policy trained against this coasting, at
whatever residual the sim AHRS produces.

A Pi-only change makes deployment differ from training. Any estimator change
lands in `hw/odometry.py`, `sim_odometry.py` and `gpu/odometry.py` together,
with a retrain. GPU training is paused for the bike build, so this queues
behind it.

**Whether it is why the policies avoid big steer angles is OPEN.** The
measurements below show small steer can arise without the estimator, not that
the estimator plays no part.

## Measured 2026-10-21

**1. On the bike, torque off**
(`traces/bike/261021-181828_run_tilt_steer90`, numbers above). The tilt no
longer runs off. A steady ~0.1 m/s offset coasts at conf 0 and clears when
the wheel straightens.

**2. The same test in sim**
(`analysis/odo_tilt_hold.py`: `control.general_move`'s own sensors, TM151 filter,
counts encoder, detailed drivetrain). Drives were commanded to 0, and a
"hand" PD held position and yaw and drove roll through +-20 deg twice each
way (achieved +-19.8, tracking 0.6 deg RMS), then 10 s still and upright.
32 s, 5 AHRS seeds, true sideways travel under 2 mm in every run.

| steer | conf | median estimated sideways travel | median peak \|v_lat\| |
|---|---|---|---|
| 0 deg | 1.00 | 0.011 m | 0.005 m/s |
| 30 | 0.69 | 0.012 | 0.010 |
| 45 | 0.39 | 0.012 | 0.013 |
| 60 | 0.12 | 0.007 | 0.023 |
| 75 | 0.00 | 0.741 | 0.140 |
| 90 | 0.00 | 1.455 (seeds: +1.46, -4.60, -2.39, +0.15, -0.70) | 0.153 |

So **training sees the same drift the bike shows.** At 90 deg the sim gives
peaks of 0.15 m/s and metres of travel, against the bike's ~0.1 m/s and 1.3 m
in 14 s. Training does not under-represent it. **And it is confined to
past ~60-75 deg:** at 60 deg, conf 0.12 is enough to hold the estimate within
a centimetre.

**Is it a transient or a bias? Mostly carried transients.** In the final
10 s still phase at 90 deg, v_lat barely moved: seed 1 went from -0.229 to
-0.197 m/s, seed 0 from +0.001 to +0.020. That is a bias of a few mm/s^2 at
most. The offset is what each tilt left behind, held forever because nothing
decays it. The bike repeat does not need the bike perfectly upright, only
still: gravity removal handles any steady tilt.

**3. How far the policies steer**
(`analysis/odo_steer_use.py`: per_command.py's 20-command grid, same cfg and
seeds, each policy on its own training sensors). Steer is folded to the wheel
heading in [0, 90] deg, the angle the confidence uses.

| policy | v_lat it sees | steps past 45 / 60 / 75 deg | median | p95 | max |
|---|---|---|---|---|---|
| `general_puffer_fixes_72M` (current) | estimate | 0.2 / 0.0 / 0.0 % | 2.7 | 23.1 | 50.9 |
| `general_rl_smooth_diff_pi` | truth | 3.1 / 0.6 / 0.1 % | 7.4 | 40.3 | 82.0 |
| `general_rl_odo_ahrs` | estimate | ~0 / 0 / 0 % (one fall: 2 % past 75) | 3.0 | 12.1 | 81.3 |
| `general_rl_nolat` | none (zeroed) | 0.1 / 0.0 / 0.0 % | 5.4 | 14.1 | 86.3 |

The current policy never enters the coasting region on the grid. **What this
does and does not show:**

- Small steer CAN come from something other than the estimator: `nolat`
  never observes v_lat at all and steers as little as `odo_ahrs`.
- It does NOT show the estimator plays no part in the CURRENT policy. Those
  are CPU policies on an older plant and the ideal drivetrain, a different
  trainer and reward, one episode per command. No GPU policy was ever trained
  on true v_lat, so there is no same-recipe comparison.
- And the estimate degrades gradually, not at a cliff: conf is already 0.70
  at 30 deg and 0.39 at 45. A policy could be avoiding the 45-60 deg band,
  where the estimate is half accelerometer, not just the conf-0 region.

Two things that are NOT confounds, checked: the reward's effort term is
`w_effort (a0^2 + a1^2 + a2^2)` with a0 the steer RATE (the action order is
steer rate, hub, differential), so nothing penalises steer angle directly. And
the grid never forces big steer: its spins can be done by pivoting on the
rear differential, which is what the current policy does (spin p95 24 deg).

**The test that settles it is a training A/B**: the current recipe once with
the estimate and once with true v_lat (or with a better estimator), same
seeds, then this steer-use count on both. That is a GPU run, so it waits until
GPU training resumes after the bike build.

**4. Every lateral estimator against truth, on the eval grid**
(`analysis/odo_compare.py`). `general_puffer_fixes_72M` drove per_command's 20
commands on its own sensors and plant (TM151 filter, counts, detailed
drivetrain), seeing the shipped estimator; 0 of 20 fell. At each 100 Hz tick
the alternatives were computed from the same readings and compared to truth:
the chassis origin in the true yaw frame, the reward's v_lat. This is OPEN
LOOP: it ranks accuracy, and the policy was flying none of the alternatives.

v_lat RMS error, mm/s (truth RMS 78), by folded wheel heading:

| estimator | all | corr | 0-15 deg | 15-30 | 30-45 | 45-90 |
|---|---|---|---|---|---|---|
| ticks | 29980 | | 26184 | 3354 | 390 | 52 |
| **shipped** (front + accel coast) | **13.8** | 0.98 | 11.3 | 18.5 | **46.6** | **81.1** |
| front constraint only | 14.4 | 0.98 | 11.4 | 18.2 | 56.9 | 95.0 |
| roller kinematics | 32.8 | 0.92 | 31.1 | 30.6 | 95.8 | 70.6 |
| conf blend front/roller | 14.8 | 0.98 | 11.2 | 17.3 | 71.5 | 73.2 |
| speed-aware blend | 17.1 | 0.98 | 14.1 | 17.6 | 75.7 | 73.2 |
| front, conf-weighted to 0 | 30.8 | 0.92 | 11.0 | 26.1 | 221.5 | 273.3 |
| front + accel, leak 0.25 s | 14.5 | 0.98 | 11.3 | 18.3 | 58.0 | 99.2 |
| front + accel, leak 1 s | 13.9 | 0.98 | 11.3 | 18.4 | 49.3 | 84.6 |

v_lon from the hub: 16.6 mm/s RMS against a truth RMS of 564.

What it says:

- **The shipped estimator is the best of these overall** and good where the
  policy spends its time: 11 mm/s below 15 deg, which is 87% of ticks.
- **It degrades steeply with steer angle: 4x worse at 30-45 deg, 7x past 45,
  where the error is the size of the signal.** So big steer DOES make the
  policy's v_lat unreliable, from 30 deg on, not only past 75. That is
  consistent with the steer-avoidance hypothesis; it does not prove it.
  Large-steer ticks are also the hardest manoeuvres, so the bins mix steer
  angle with how violent the motion is.
- **The accelerometer coast is worth having** at 30-45 deg: 46.6 against
  56.9 for the front term alone. Pulling to 0 instead is far worse (221).
- **A leak costs accuracy here.** 1 s is nearly free (49.3), 0.25 s is not.
  The grid never holds full lock, so it cannot show what a leak buys.
- **The rollers are no longer better at standstill**: hold 25.0 against the
  shipped 13.2. `odometry-rewrite.md`'s 2026-08-26 result (rollers 25.9
  against front 44.4) no longer holds. Which change flipped it is not
  isolated; nearly everything differs:

  | | 2026-08-26 | now |
  |---|---|---|
  | policy | a CPU SB3 policy trained on truth | `general_puffer_fixes_72M` (GPU, MinGRU, trained on the estimate) |
  | sensors | clean: no AHRS error, ideal encoder | TM151 filter model, counts encoder (2026-08-27) |
  | drivetrain | ideal: velocity straight onto the input shafts | detailed (opt-in 2026-09-14, default 2026-10-02): XC430 firmware loop, differential detent, roller slop |
  | contact | shared | rear rollers, front tyre and righting parts each their own (2026-09-09) |
  | steer | ideal | XC330 current law and gearbox friction (2026-09-26), headset friction (2026-09-30) |
  | geometry | v1 | measured front wheel (2026-10-05), v2 as built, rake 20 (2026-10-19) |
  | what was run | 4 regimes | the 20-command grid |

  Roller slop is the obvious suspect for the rollers getting worse, since
  it is play between the commanded differential and the roller, but that
  is a guess.
- Spins move the rear axle sideways: truth v_lat RMS 79-102 mm/s on
  `spin +-2/3`, against 9-16 on straight driving.

## A trackstand is fine

A trackstand with the front wheel turned, the rear wheel still and the bike
tilting does NOT read as sideways travel, as long as the wheel's ground
heading stays under ~60 deg. The front term (conf 0.12 at 60 deg, 0.39 at 45)
keeps pulling the estimate to v_lon tan(theta) - yaw_rate L, which is ~0 with
the rear still and no yaw. Sim, 32 s of +-20 deg tilts: about 1 cm at any
steer up to 60 deg. When the rear wheel rocks, v_lon tan(theta) is real
motion: the front contact rolls along its heading.

Only past ~75 deg (conf 0) does the estimate coast. On the eval grid the
current policy never gets past 51 deg, though whether the estimator is part of
why is open (above).

## Which point the estimate tracks

**The policy is rewarded on the chassis origin.** The training reward, both
paths, uses the freejoint's `qvel[:2]`: the velocity of the chassis origin,
which is the rear axle centre (`gpu/obs_reward.py:130`,
`control.balance.extract_state`). The observation is the estimator's.

**The estimator's derivation names the rear contact.** v_lon is
hub rate x outer radius. v_lat is the front-wheel constraint, written as "the
rear's velocity plus the yaw lever arm". For a rigid body, the point under the
axle and the axle itself differ by roll_rate x 51 mm sideways. So if the
mismatch mattered, the error against the origin would correlate with roll
rate at about 51 mm per rad.

**In sim it does not** (`analysis/odo_refpoint.py`). `control.general_move` was flown on its own sensors
(TM151 filter, counts encoder, seed 0) for 4.8 s per regime. Error RMS in
mm/s against each point, with the error's slope against roll rate:

| regime | v_lat vs origin | v_lat vs point below axle | slope vs roll rate | roll rate RMS |
|---|---|---|---|---|
| standstill | 15.7 | 16.6 | +15 mm/rad (corr +0.16) | 0.17 rad/s |
| shoved | 31.8 | 37.9 | -6 (-0.07) | 0.36 |
| straight 0.6 | 6.3 | 7.1 | -11 (-0.09) | 0.05 |
| circle R0.8 | 25.0 | 27.3 | +0 (+0.00) | 0.22 |

The estimate is closer to the origin in every regime, and the roll-rate
slope is a fraction of 51 mm with near-zero correlation. So the reference
point is not where the error comes from. Why it is so small is not
established. A guess: the rear's sideways motion goes through the driven
rollers (set by the differential, with slip), so the rigid "point under the
axle" is not a fixed rolling point the way it would be on a plain wheel.

One limit: it is one AHRS seed. It DID fly the detailed drivetrain the
policy trained on. The policy's "being flown on the ideal one" NOTE was
false: it checks a `_drivetrain_active` flag that `run_drive` and `record`
set and `test_odometry_in_the_loop._fly` (which the script copied) did not.
The test now sets it.

**The AHRS position never bent this.** Orientation, and so roll, pitch, yaw
and the gyro, are the same at every point of a rigid body.
`odometry-rewrite.md` measured gyro differences of exactly 0 across five
mounting sites. Only the accelerometer depends on position, and the sim's is
MuJoCo's at `ahrs_site`, `bike.ahrs.pos` [0.0989, 0, 0.0842] (the v2 CAD),
plus the sensor's own offset inside the housing
(`sim_ahrs.TM151_ACCEL_OFFSET_M`, rotated by the 45 deg mount).
`odometry-rewrite.md` quotes [0.05, 0, 0.13], the pre-v2 position; its
lever-arm table is about that mount. What the sim did
NOT carry was the TM151's sign convention, which only the hardware path
reads.

## Candidate fixes (not chosen)

Each one must land in all three estimators.

| idea | for | against |
|---|---|---|
| **Leak v_lat toward 0 at low conf** (time constant tau) | smallest change; bounds the drift by `residual x tau` | a real crab at sharp steer also decays; tau is a new unmeasured constant (randomise it, see the AHRS tau lesson) |
| **Stationary gate**: hub and ring both still -> v_lat toward 0 | exactly the reported case; "identical differential means no crabbing" was already item 4 of `odometry-rewrite.md` | needs a threshold on encoder noise; does nothing while driving |
| **Rollers as the low-conf fallback** | measured better than the front constraint at standstill (25.9 vs 44.4 mm/s, 2026-08-26) | self-referential, since the differential is the control input; the blend fell sooner closed-loop in 3 of 4 regimes |
| **Lever-arm correction** of the accelerometer from the gyro (`alpha x r`, `omega x (omega x r)`) | removes a known, modelled term | alpha is a differentiated gyro, which is noisy; it does not touch roll lag |
| **Show the policy `conf`** | lets it learn when v_lat is meaningless; `odometry-rewrite.md` open question 2 | an obs-layout change: `general_spec.py`, both sides, retrain |

**Selection is closed-loop survival on the eval grid**, as `odometry-rewrite.md`
established: open-loop RMS has picked the wrong estimator before. Add one
open-loop number alongside it: drift in metres after the measured tilt
sequence above, in sim. That is the failure being fixed, and the grid never
holds the bike tilted at full lock.

## Errors found in the earlier docs (2026-10-21)

Checked against the working tree, not corrected in place yet:

| where | says | actually |
|---|---|---|
| `odometry-rewrite.md`, status block | `hw/odometry.py` "still fails seven open-loop tests, all registered in `tests/expected_failures.txt`" | stale since 2026-09-21. The tests were rewritten (that doc's own last section), and the registry has no odometry entries |
| `odometry-rewrite.md`, "Not in scope" | `control.general_move` "still names `general_rl_smooth_diff_pi`" | stale; it names `general_puffer_fixes_72M` |
| `odometry-rewrite.md`, "What the rewrite needs" 2 | "Nothing models gyro bias, accelerometer noise" | superseded: `sim_ahrs` `tm151_filter` (and `gpu/ahrs.py`) corrupt both |
| `odometry-rewrite.md`, open question 1 | front-constraint fit degraded to tan-coef 0.652, L_eff 0.137 | the same doc's 2026-09-21 section retired that test as "measured its fixture's conditioning", so this may be answered. Untested: truth v_lat is the freejoint origin (rear axle centre, 51 mm up), the constraint is at the contact, and they differ by about roll_rate x r |
| `control/general_spec.py`, `zero_lat` docstring | v_lat "is reconstructed through the driven rear wheel, which slips" | it comes from the front-wheel constraint; the rear-roller estimate is diagnostic only (`hw/odometry.py`) |
| all of them | no steer sign anywhere in the hardware path | true until 2026-10-21 (above) |
| `odometry-rewrite.md`, mounting position | the AHRS at [0.05, 0, 0.13] | pre-v2; the v2 CAD has it at [0.0989, 0, 0.0842] (`bike.ahrs.pos`), 45 deg nose-down |
| `test_odometry_in_the_loop._fly` | (implicitly) printed "flown on the ideal one" | it flies the detailed drivetrain; the flag was never set, fixed 2026-10-21 |
| `hw/ahrs.py`, `AhrsSample.accel` | "gravity included", fed straight to an estimator that assumes +g up | the TM151 sends -g up; negated in `parse_combo` 2026-10-21 |
