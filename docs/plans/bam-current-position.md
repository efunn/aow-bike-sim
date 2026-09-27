# BAM models for current-based position control (XL330 and XC330)

STATUS: notes for later, nothing started (2026-09-26). Not bike work: a
contribution to Rhoban's BAM (Better Actuator Models,
https://github.com/Rhoban/bam), continuing the user's issue
https://github.com/Rhoban/bam/issues/14 ("Improving/verifying XL330i
current-based position control model"). Written with the X330 fixture's
bench work fresh; everything here that says "measured" was measured on the
XC330-T181 in `docs/plans/pre-assembly-bench-checklist.md`, not on an XL330.

## Where issue #14 stands (read 2026-09-26)

- The user's bench data (XL330-M288-T, 5.2 V): at stall, bus current vs duty
  fits `I = (vin/R) d^2` (R^2 0.983, R 2.98 Ohm) better than phase
  `I = (vin/R) d`; the bus model looks right at >= 100 mA.
- `error_gain` for mode 5: measured `k5 = 2.525e-3 A/(Kp rad)` against a
  derived `(4096/2pi)/(256 x 1000) = 2.546e-3`; BAM HEAD had 0.01, ~4x too
  large. Gregwar (maintainer) asked for a PR with that change, and a
  testbench to gather trajectories and check the fit.
- Agreed in the thread: mode 5 has no back-EMF damping of its own (the
  current loop regulates it away), which is why it needs a D gain where mode
  3 does not. Gregwar: put kd on the BAM side and identify a
  `velocity_error_gain` -- KP=0, a few KD values, move the horn by hand,
  log Present Velocity and Present Current, fit. Use kp/kd PAIRS that
  were tested to work (`kp_kds = [[50, 25], [100, 40], ...]`), not a grid;
  goal current need not vary. Masses and arm: "the same as the one we used
  in our data".
- Open on BAM's side: whether `XL330CurrentActuator.compute_torque` (bus
  current -> duty by the quadratic, torque from phase current -- the same
  derivation as this repo's `righting_servo`) holds. Marked UNTESTED there.

## What this repo's XC330 bench already knows (feeds straight in)

| finding | where it bites BAM |
|---|---|
| Drive edge at 16.5 mA, both directions, ~1 mA hysteresis; below it Present PWM ~0 while Present Current reads the goal exactly | the bus model has no deadband; below ~17 mA it predicts torque the servo does not make |
| Lift current = 16.1 + 2.23 mA per mN m, 8-50 mN m at stall; linear, not sqrt | the bus law says ~4 mA past the edge for 50 mN m; it took ~110. Up to ~130 mA the XC330 does NOT follow it |
| Present Current reads ~12-14 mA with almost no drive in modes 3 and 16 | a sensing offset of ~14-16 mA is one reading of the edge (untested: a supply meter settles it) |
| Mode 5's Present PWM is not the duty the motor gets (mode 3 lifts a load at 5 % that mode 5 holds at 14 % "PWM"); Goal PWM does not cap mode 5 (`servo-measurements.yaml`, `xc330_current_position`) | BAM's recorder logs Present PWM as `load`/duty -- in mode 5 that column is not what it claims |
| Present Velocity lags position 2-3 frames (read 88 deg/s while position moved 343) | fit on differenced position, or model the lag; do not treat the register as the state |
| Gearbox friction ~0.5 x load (+ ~7 mN m back-driving), static/sliding stick-slip, history- and angle-dependent holding | BAM's `load_friction_base` (m3) is exactly this term: XL330 m3 fitted 0.154 |
| Firmware P scaling mode 3: 2.849e-3 duty/(P rad), matches the issue's XL330 2.855e-3 | same firmware constant across the two models |
| Stall torque per duty (mode 3): 0.53-0.69 N m against the datasheet 0.80 | BAM identifies kt and R directly; a cross-check |

## The rig, and what it needs

The X330 fixture (`aow_sim.cad_x330_fixture`, IDLER) takes either servo --
same case -- with a two-armed lever, a 10-32 mass slot to 44 mm each side,
and `analysis/servo_lift.py` already runs modes 3, 5 and 16 on it with the
safety that dropped loads taught (catch, lower before torque off, GC off).
Differences from BAM's pendulum, to solve before recording:

1. **Geometry.** BAM's `Pendulum` testbench is a point mass on a uniform rod
   hanging at q = 0, bias `sin(q)`, swinging to +-pi/2. The fixture's lever
   is two-armed and balanced, level at rest, bias `m g r cos(q)`, and the
   yoke stops it at +-45 deg. Either:
   - a `Testbench` subclass for the lever (`compute_mass` = m r^2 + the
     lever's own inertia; `compute_bias` = m g r cos(q - q_level)) and BAM's
     trajectories rescaled into +-35 deg; or
   - run it as a free pendulum: take the yoke off (no stops) so the lever
     hangs, arm down, off the table's edge. The arms are in front of the
     plate's +X edge, so they may clear it -- CHECK in CAD; without the
     yoke the lever rides on the horn alone (no idler support), fine for
     BAM's light masses. Then BAM's own trajectories and `Pendulum` apply
     unchanged.
2. **Masses.** Ask for / match BAM's XL330 pendulum mass and length (their
   params JSONs do not record them; the logs do: `mass`, `arm-mass`,
   `length`). The fixture's masses today: 34, 38, 117 g, at up to 44 mm.
3. **`lift_and_drop` turns torque OFF with the arm up** -- BAM's
   back-drive/Stribeck identification. On this rig that drops the load onto
   the stop, which is what sheared the pins (2026-09-25). As a free
   pendulum it swings through the bottom instead: fine. With the yoke on,
   drop from a low angle or skip it.
4. **Recorder.** BAM's `bam.dynamixel.record` polls through rustypot at 1
   Mbps and logs `position, speed, load (= Present PWM as duty),
   input_volts, temp` with a host timestamp. Either run it as is (swap the
   `load` column's meaning for mode 5, see above), or write BAM's log JSON
   from a `bench_log` capture: 3 Mbps, Realtime Tick timing, raw registers,
   Present Current included. The converter is small; the tick timing is
   the better clock.
5. **Supply.** XL330 at 5 V (the issue's setup; BAM's defaults assume 7.5 V
   and pass `--vin`); XC330 at 12 V. Gregwar: make sure the supply's own
   limit is not what is being identified.

## Test list, per servo (XL330-M288, XC330-T181)

1. Static `error_gain` (the issue's method): goal past a stop, sweep P,
   regress output on P x error, modes 3 and 5. Done for XL330; for XC330 the
   mode-3 number exists (2.849e-3), mode 5 to do.
2. `velocity_error_gain` (Gregwar's): KP=0, KD in a few values, move by
   hand, log Present Velocity (lagged -- see above) and Present Current.
3. The drive edge and the static torque line: `servo_lift.py --pwm-sweep`
   and the lift sweep, as done for XC330; repeat for XL330 (its edge and
   slope will differ: 288:1, 5 V, ~1920 mA range).
4. Trajectories for identification, mode 5, Goal Current at max: BAM's four
   (`sin_sin`, `lift_and_drop`, `up_and_down`, `sin_time_square`) at kp/kd
   pairs tuned by hand to be well damped -- e.g. start from the righting
   gains P700/D1400 and halve/double.
5. Same trajectories in mode 3 (BAM's existing `xl330` path) as the control:
   the identified friction should agree across modes; only the drive
   stage differs.
6. Fit `m1`..`m6` (BAM's model family) on 4; compare against 3's static
   lines, which BAM's fit never saw -- the independent check.

## Questions to put on the issue first

- Is a lever with stops and rescaled trajectories acceptable, or should it
  be the free pendulum?
- For mode 5, what should the `load` column carry -- Present PWM (not the
  applied duty in mode 5, measured) or Present Current?
- Does BAM want the deadband / sensing offset in `XL330CurrentActuator`, and
  would the XC330 (a different motor on the same firmware) be welcome as a
  second actuator?
