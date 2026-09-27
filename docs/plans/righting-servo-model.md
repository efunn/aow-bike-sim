# Righting servo model: from the bench's measured lines

STATUS (2026-09-26): steps 1-7 IMPLEMENTED (step 6's teleop: self-righting
checked, the floor rig pending); step 0 blocked on the horn coupling. Covers the righting
servo's current law and the gearbox friction of BOTH XC330s (steer too).

What implementing it found, beyond the plan below:

- The friction limit is the FIXED POINT `(f0 + c tau)/(1 -+ c)` by branch
  (friction opposing or helping the motor), not `f0 + c L` from last step's
  force: the lagged form let a suddenly applied load slip before it
  converged. Same thresholds, no lag (gearbox_friction.py).
- MuJoCo's default friction softness lets a sub-limit load creep 0.08-0.32
  rad/s on the lever: stiffened (solref 2 x timestep, solimp 0.99/0.999) where
  a servo holds a gravity load. NOT on the steer: there it made the LQR's
  0.8 m/s U-turn a knife edge, while the default held for any constant
  friction 0-10 mN m.
- `balance.run` got the hook too: the LQR's fast-reverse turn (4.7 deg
  against a 6 deg tolerance with NO friction) missed on the constant
  approximation and passes with the real friction.
- Teleop's starting cap now converts `limits.torque_nm` as OUTPUT torque
  while moving: 0.55 N m -> 785 counts (679 as motor torque). Breaking away
  at the stroke's peak would need ~0.84 N m of motor torque, over the 0.74
  the Current Limit gives. 300 counts gives ~0.20 N m at the crank moving.
- The floor rig's roll servo at P 700 is P-gain-limited, not cap-limited:
  7 deg under a 0.3 N push at any Goal Current (the sqrt law's rising gain
  near zero error had hidden that).
- `servo_kp/kv` (the native actuator) re-derived with the measured k:
  1.48 / 0.0473.
- TELEOP (user, 2026-09-26, `--swing-linkage`, 12 V): 525 mA lifts the bike
  a little, 605 gets it partway, 625 lifts it fully to level -- inside the
  910 limit, and under the 785 that `limits.torque_nm` converts to (that
  figure is a design budget, not the stroke's true need). The user's read:
  the mechanism wants more mechanical advantage.
  Reproduced headless by `analysis/righting_current_sweep.py` (fallen, crank
  goal stepped as teleop's 9/4, wheels passive): first upright at 630 counts
  at 12 V and 11.1 V, 650 at 9.9 V, both sides; ~0.51 N m of motor torque at
  the threshold against the 0.74 the Current Limit gives.
- Braking at speed does not match: see righting_servo.py's docstring.
- Calibration (`analysis/servo_lift_sim.py`): lift within 1-5 mA of the bench
  on the four fitted loads (208 vs 189 on the 236 g one), fall within 3-4.
- Plant digest 8d8b25a809ea2f1a -> 9f162dc4727110e2; bundle re-exported
  (worst fit R^2 0.960); red set unchanged (3 accepted).
The measurements are in `docs/plans/pre-assembly-bench-checklist.md`
("Loaded runs", "RELEASE test",
"PWM sweep"), the captures in `traces/servo_lift/`, the script
`analysis/servo_lift.py` (`fit` reproduces the lines below).

## What the bench says (XC330-T181, id 103, mode 5, 12 V, P700/D1400)

Measured on the X330 fixture's lever, lifting from rest, Goal Current in 2
mA steps. THE HEAVY WEIGHT IS 137 g (marked on it, re-weighed with its
screw and nuts); it was typed as 117 through every run, and the numbers
here were redone with 137 on 2026-09-26 (`servo_lift.CORRECTIONS` applies
it on load; the traces' own meta still say 117).

| load | torque at level | lift above | load wins below |
|---|---|---|---|
| 34 g at 25 mm | 8.3 mN m | 35 mA | never |
| 34 g at 44 mm | 14.7 | 49 | 17 |
| 137 g at 25 mm | 33.6 | 79 | 27 |
| 137 g at 44 mm | 59.1 | 129 | 43 |
| 236 g at 44 mm (step 0, one rep) | 101.9 | 189 (180-198) | not reached (holds at 78) |

| | fit | loads |
|---|---|---|
| drive edge (`--pwm-sweep`) | I0 = 16.5 mA, both directions, ~1 mA hysteresis | -- |
| lift, <= 130 mA | 20.3 mA + 1.822 mA per mN m, residuals <= 2.5 mA | 4 |
| lift, all five | 24.3 mA + 1.653 mA per mN m, residuals -3.6..+7 mA | 5 |
| fall | 8.0 mA + 0.587 mA per mN m | 3 |
| holds a released load (0 mA) | ~8.5 mN m after a lift, more after a lowering; varies with angle | release sweeps (34 g only, unaffected) |
| no load | held at 20 mA, moved at 25 | breakaway + lever-alone runs |

The 236 g point sits 17 mA under the four-load line (206 predicted, 180-198
measured): above ~130 mA the servo makes somewhat MORE torque per mA than
the low-current line. One load, one rep -- a direction, not a shape.

One consistent reading (fit over the four low loads), with the motor's
torque `k (I - I0)` above the edge:

- `k = 0.830 N m/A` (0.83 mN m per mA), `I0 = 16.5 mA`
- friction `0.51 x |load| + 5.1 mN m`, the same both ways under this
  split (the hand back-drive said 5-8 mN m for the constant)
- with the 236 g point included: `k = 0.893`, `0.48 x load + 7.3`

Only as good as its assumptions: `k` and the 0.51 are split by assuming
forward and back-drive friction scale alike, one servo, static and slow.
The model-free lines above do not need that assumption.

**THE SQRT LAW IS OUT IN THIS RANGE.** `righting_servo` maps Goal Current to
stall torque as `tau = ts sqrt(I / I_stall)` (0.47 N m at 300), which says
the 59.1 mN m load needs ~5 mA past the edge before friction. It took ~110.

## THE CONSEQUENCE, before anything else

What the bike needs, INFERRED from the sim (not measured): at 300 counts
the sqrt law's ~0.47 N m lifts the bike at 12 V and not at 11.1 V
(bike_params' comment on `righting_current`), so ~0.45 N m at the servo.

| | at 300 mA | current for 0.45 N m of load |
|---|---|---|
| sqrt law (the model now) | 0.47 N m stall | ~275 mA |
| lift line, four loads | 0.154 N m liftable | ~840 mA |
| lift line, five loads | 0.167 N m liftable | ~770 mA |

So at 300 counts the measured lines lift about a third of what the model
does, and the bike would need ~770-840 mA -- under the 910 Current Limit,
but reached by extrapolating from 189 mA, 4x past the data. It reaches
every self-righting margin: "300 counts lifts the bike only under plug"
(righting_servo's docstring), `self_righting.py lift`, the linkage
configs' `limits.torque_nm: 0.55` -> counts conversion.

The 236 g point bends the right way for the bike (less current than the
line). What happens at 300-900 mA is unmeasured. The one argument for
torque per mA FALLING up there is weak: the XL330 stall sweep (BAM issue
#14, current control, 20-450 mA) fitted Present Current against Present
PWM as `I ~ d^2`, and IF Present PWM were the applied duty, torque (from
phase current, ~d at stall) would go as sqrt(I). But that sweep measured
no torque, and on the XC330 Present PWM under the current controller is
NOT the applied duty (mode 5 holds at 14 % what mode 3 lifts at 5 %). The
lever measures torque against current directly, and says linear or better
to ~190 mA.

## Steps

0. **Measure 130-910 mA.** The lift sweep with heavier loads on the same
   fixture: `servo_lift.py --mass-g M --radius-mm 44 --home-ma 450`, then
   `fit` over all loads. Decide: one line, a bending curve, or something
   else. Everything below assumes a line or piecewise; adjust step 1 if not.

   **First run, 2026-09-26** (`traces/servo_lift/260926-203643_servo_lift_236g44R`):
   236 g at 44 mm RIGHT (137 g front + 99 g back on one screw), 101.9 mN m at
   level, lifting from ~-4 deg (home at 450 mA drooped 4-6 deg below level
   with ~177 mA flowing). One rep, random order, 70-330 mA. **The hub's pins
   sheared ~122 s in**, after ~30 lifts peaking at 200-320 mA; the user
   powered off at 136 s. Only tries before 122 s are counted (`until_s` in
   `CORRECTIONS`): lifts at 198 mA and every try above, holds 78-180 mA,
   creeps 0.7 deg (under the 1 deg "moved" bar) at 72 and 74.

   **The pins, measured by failure**: the fitting hub (the "1.5 mm" pins,
   one dot and one perimeter at 0.45 mm extrusion; the "1.75 mm" hub did
   not fit) sheared one layer above the base, under cyclic 0.1-0.2 N m
   (static load 0.10, motor peaks higher) after an hour or more of earlier
   runs at <= 60 mN m. The hub also walks off the horn axially over time,
   which loads the pins further. ~4-8 N per pin at the 6 mm bolt circle,
   far under the 0.4 N m estimated here before. Bolting the hub to the horn
   is ruled out for the bench for now (user); the final build's screwed
   arrangement is undecided.

   So the rest of step 0 waits on a MECHANISM question, not a test one:
   how the horn couples to what it drives. The bike's righting linkage
   needs the same answer at a higher torque (~0.45 N m, INFERRED from the
   sim, plus peaks), so whatever carries that in the final build can carry
   the fixture's lever too, and step 0 runs unchanged on it. Until then
   steps 1-7 can proceed on the four-load line with the extrapolation
   flagged; the 236 g point says the line is, if anything, conservative up
   to ~190 mA.

1. **`src/aow_sim/righting_servo.py`: the measured current law REPLACES
   the sqrt law** (decided 2026-09-26: no `bus_sqrt` switch kept -- the
   bench refutes it from 8 to 102 mN m; the derivation stays in the
   docstring as the rejected alternative, with why, and in git):
   - motor torque from the clipped current demand:
     `tau_m = k sign(i) max(0, |i| - I0)`, then limited by the motor line
     `ts (s u - w/w0)` at `u = +-1` (the no-load ceiling stays measured:
     11.5-12.1 rad/s on both bench units);
   - braking (demand opposing motion) keeps "plug" -- measured, 242/243
     frames -- with the same edge and gain;
   - `current` (what Present Current reports) = the demand clipped, as the
     bench saw: Present Current reads exactly the goal at stall.

2. **Gearbox friction, ONE model for both XC330s** -- the righting servo's
   crank and the steer. Same servo, same gearbox, and the bench agrees
   across modes: static 0.51 x load + 5.1 mN m (mode 5, four loads) vs
   0.53 x load + 7.0 (mode 3, two loads); running 0.15 x load + 5.7 (mode
   3, the joint fit of the four speed lines).

   Mechanism -- MuJoCo's own dry friction, re-limited every substep, so
   stick/slip stays in the solver and Python only does arithmetic:

       L = |actuator torque + this dof's friction constraint force|   (last step)
       dof_frictionloss[dof] = f0 + c L     static (f0, c) if |w| < w_eps, else running

   `L` is the torque the gearbox transmits to the load, which is what the
   fits are in: driving, friction opposes the motor and `L = tau_m - F`;
   back-driven it helps and `L = tau_m + F` -- one formula for both, no
   direction switch. The friction row's index is `data.ne` + the number of
   lower dofs with non-zero frictionloss (checked: 0 mismatches in 3000
   steps of the main model; assert it in a test, since it relies on
   MuJoCo's constraint ordering). One step of lag (0.4 ms).

   COST, measured 2026-09-26 on the main model (M4, 20 dof, median of 6
   interleaved blocks of 5000 steps): `mj_step` 18.1 us with no steer
   friction, 18.2-18.3 with a constant one (one more constraint row), 18.5
   with the per-substep update -- ~+0.4 us, ~2 %, per servo. The general
   env already loops substeps in Python (`general_env.py:888`), so no new
   loop. The righting servo's update rides in its existing `pre_step`.

   Where the steer's update runs: a `pre_step` like the righting servo's,
   called before every `mj_step` in `general_env`, `run_drive` (teleop)
   and `record`. The other loops (`balance`, `ball_env`, `flick_env`,
   `pivot_env`, `righting`, `linearize`, `sim_odometry`) call `mj_step`
   without hooks; for them the MODEL's default `frictionloss` is the
   no-load static value (5 mN m) -- a constant-friction approximation, not
   zero. Wire them later only if one of them turns out to care.

   What it does to the steer: with friction modelled, `stall_torque` is
   the MOTOR's torque (friction separate), and the usable torque against a
   load drops to `(stall - f0) / (1 + c)`: ~0.52 N m breaking away at the
   datasheet 0.80, ~0.43 at the bench's 0.66. That is a real plant change
   for the steering, on top of the stall-torque question.

   Direction/angle/history effects documented, NOT modelled (agreed).

3. **Constants into `config/bike_params.yaml`**, `servos.xc330_t181`, each
   with `source: measured` and a pointer to the plan doc:
   `current_deadband` 0.0165 A, `current_torque_gain` 0.830 N m/A, and a
   `gearbox_friction` block read by BOTH servos: static `{fraction: 0.51,
   constant: 0.005}`, running `{fraction: 0.15, constant: 0.006}`,
   `stick_speed` (w_eps, rad/s -- pick from the release sweeps' creep).
   Re-fit once step 0 has more than one point above 130 mA.
   Make `measured` the default here. THIS MOVES `plant_digest` (accepted:
   the plant has moved since the last training anyway). Work CLAUDE.md's "Before changing a physical
   parameter" list: re-export the deploy bundle (`python -m
   aow_sim.export_deploy`), list the policies it makes provisional (anything
   trained with the swing linkage in the plant), note it in `docs/status.md`.

4. **`floor_rig`**: `servo_friction_ma` / `servo_friction_nm` go -- the
   servo now carries its own friction. Remove from `config/floor_rig.yaml`
   (it is 22, flagged an upper end), `floor_rig.py` (`add_chain`,
   `servo_friction_nm`) and `tests/test_floor_rig.py` (the servo-friction
   test becomes "the attached servo's friction is the righting servo's").

5. **Tests of the IMPLEMENTATION, not the calibration** (`tests/test_righting_servo.py`,
   marker `righting`). Each reads the constants from `bike_params` and
   derives its expectation from them, so a re-fit that changes the numbers
   leaves them green; only breaking the code turns them red:
   - a lever in MuJoCo (the fixture's geometry, any load) lifts at the
     current the params' own line predicts, `I0 + ((1 + c) L + f0) / k`,
     within a step -- i.e. `pre_step`'s torque and friction reach the joint
     as specified;
   - zero torque below `I0` at stall; the no-load speed still `w0` at Goal
     Current max;
   - the friction row index assumption (`data.ne` + lower frictionloss
     dofs) holds, for the main model and the floor rig;
   - the steer: at a fixed motor torque `tau` it is back-driven by a load
     above `(tau + f0)/(1 - c)` and drives one below `(tau - f0)/(1 + c)`,
     holding in between, with c and f0 read from params.
   Today's righting tests: the law-independent ones (plug braking, Present
   Current's sign, the gains conversion, the no-load ceiling) move to the
   measured law; the ones that test the sqrt law itself go with it.
   Then the `righting`, `contact`, `geometry`, `deploy` markers and the full
   suite (it moves a digest); read the red-set verdict.

   The CALIBRATION check -- does the sim reproduce the bench table -- is
   analysis, not a test: a new `analysis/servo_lift_sim.py` that runs the
   fixture's sweeps in sim and prints sim vs bench per load. Rerun it after
   any re-fit. Separate from `servo_lift.py` because that one runs on the
   Pi, where nothing may import mujoco.

6. **Re-derive, do not carry forward**, then drive it:
   - `analysis/self_righting.py lift` ("300 counts lifts the bike"), the
     teleop starting cap (`limits.torque_nm` -> counts),
     `analysis/servo_modes.py`'s mode comparison, and the floor rig's
     `stiff_roll` / `fixed_roll` presets (their counts were chosen under
     the sqrt law). If 300 counts no longer lifts the bike, that is a
     design finding, not a test to loosen.
   - TELEOP, by the user, once the measured law is the default: the main
     bike (`mjpython -m aow_sim.run_drive --teleop`) and the floor rig
     (`... --teleop --rig`) -- self-righting at the new counts, holding,
     nothing unphysical. The check that "nothing crazy happened".

7. **Docs**: `docs/status.md` (what moved, what is provisional), the bench
   plan's pointer here, `docs/measurements/servo-measurements.yaml` gets the
   lift/fall table as an entry beside `xc330_current_position`.

## Not in scope

- The steering's STALL TORQUE (mode 3/4): kept at the datasheet 0.80 N m,
  noted in `bike_params.yaml` as possibly ~0.6 (bench 0.62-0.69); to be
  measured in place on a bike test rig. Its gearbox friction IS in scope
  (step 2).
- BAM-style identification of the same servo: `docs/plans/bam-current-position.md`.
