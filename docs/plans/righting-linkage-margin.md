# Righting linkage: margin on the servo, traded against speed

STATUS: plan, nothing started (2026-09-26). Follows the XC330 model change
(`righting-servo-model.md`), which made the stroke's current requirement
measurable. The previous optimisation round is `wing-linkage-design-and-optimization.md`;
by the user's account it was left half-baked.

## What the current four-bar needs (sim, measured 2026-09-26)

`analysis/righting_current_sweep.py`, bike on its side, crank goal stepped as
teleop does, wheels passive, 12 V:

| | counts to reach upright |
|---|---|
| goal stepped (teleop's 9/4) | 630 (by hand in teleop: 625) |
| goal ramped at 15 deg/s (quasi-static, `--slew-dps 15`) | 660 |
| 11.1 V / 9.9 V, stepped | 630 / 650 |
| Current Limit | 910 |

So the swing's momentum over the hump is worth ~30 counts (~5 %); the lift is
essentially quasi-static.

The quasi-static stroke, 910 counts, 15 deg/s:

| roll [deg] | 80 | 75 | 70 | 65 | 60 | 55 | 50 | 40 | 30 | 20 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| crank [deg] | -10 | 1 | 11 | 20 | 28 | 36 | 44 | 59 | 73 | 90 | 110 |
| motor torque [N m] | 0.24 | 0.47 | 0.51 | 0.54 | **0.54** | 0.53 | 0.50 | 0.42 | 0.32 | 0.35 | 0.11 |

- Peak 0.539 N m at ~61 deg of roll, crank ~26 deg.
- Work: the system (1.135 kg) CoM rises 66 mm, m g dh = 737 mJ; the motor
  does 910 mJ over 147 deg of crank (gearbox friction and the rest ~19 %).
- MEAN motor torque over the travel 0.356 N m; PEAK/MEAN = 1.51.

## The framing

The work is fixed: m g dh plus losses, ~0.9 J. For any mechanism the
average motor torque is `W / crank travel`, so for a given travel, moving
lever arms around really is zero-sum ON AVERAGE -- the user's intuition. What
geometry CAN change:

1. **The profile's shape.** Peak/mean is 1.51 now. A transmission whose ratio
   varies to keep motor torque constant would need the mean, 0.36 N m, over
   the same 147 deg -- ~450 counts instead of 660. That is the "something
   small" the angles buy, and it is not small: a third off the peak.
2. **The travel.** Torque falls as 1/travel. A four-bar crank is bounded
   short of toggle (~180 deg), so for this mechanism family the floor is
   roughly W/pi ~0.29 N m. Past that needs a different transmission (gearing,
   a multi-turn crank; check whether mode 5 permits multi-turn on the XC330
   before relying on it).
3. **Dynamics.** Spending momentum through the hump: worth ~5 % here as
   stepped; a deliberate wind-up (swing away first, or pump) is a separate
   question.

SPEED is the same picture from the other end: the fastest stroke keeps the
motor at its maximum-power point (tau ~ stall/2, w ~ no-load/2, ~2.4 W by the
datasheet line) for the whole lift, so t_min ~ W / P_max ~ 0.4 s. That too
wants a FLAT profile, just at a higher torque level (shorter travel). Margin
and speed are one knob -- the ratio's LEVEL -- once the profile is flat; the
profile's flatness is what is free.

## The thought experiment the user proposed

Ignore packaging: where do the servo and links go for (a) the most margin,
(b) the most speed? With the framing above:

- (a) the longest crank travel the mechanism allows, with the ratio shaped
  so the torque is flat -- a "5 mm crank" is the limit of that direction
  (long travel per unit wing motion), bounded by toggle.
- (b) the travel that puts the flat torque at ~stall/2.

Both answers are ratio PROFILES r(theta) = d(roll)/d(crank), not linkage
dimensions. Step 1 below computes the ideal profile directly; step 2 asks
which four-bars can approximate it.

## Steps

1. **Ideal profile, mechanism-free.** From the table above (required
   output torque vs roll, i.e. the load the bike presents), compute the
   crank-angle schedule that keeps motor torque flat, for a range of total
   travels (90-270 deg). Output: counts needed vs travel, and stroke time vs
   travel. This is the Pareto front no linkage can beat.
2. **How close can a four-bar get?** Fit the four-bar's parameters (crank
   length, attach point, ground pivot) to the ideal r(theta) at a chosen
   travel; score by `righting_current_sweep.py`'s counts at 9.9 V, not by a
   torque budget. The previous round optimised against `limits.torque_nm`,
   which is exactly the budget-not-requirement number that mis-predicted this
   (785 vs 630).
3. **Pick a point** with the user: counts at 9.9 V against stroke time,
   then the packaging constraints the thought experiment ignored.

## Not in scope

- The servo's own law above ~200 mA (unmeasured; `righting-servo-model.md`
  step 0). Every count here extrapolates it.
- Policy catching after the lift (the sweep's wheels are passive).
