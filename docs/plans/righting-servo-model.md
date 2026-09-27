# Righting servo model: from the bench's measured lines

STATUS: plan, nothing implemented (2026-09-26). The measurements are in
`docs/plans/pre-assembly-bench-checklist.md` ("Loaded runs", "RELEASE test",
"PWM sweep"), the captures in `traces/servo_lift/`, the script
`analysis/servo_lift.py` (`fit` reproduces the lines below).

## What the bench says (XC330-T181, id 103, mode 5, 12 V, P700/D1400)

Measured on the X330 fixture's lever, 34 g and 117 g at 25-44 mm (8.3-50.5
mN m), lifting from rest, Goal Current 0-250 mA in 2 mA steps:

| | measured | fit |
|---|---|---|
| drive edge | Present PWM ~0 through 16 mA, 2.4 % at 17, both directions, ~1 mA hysteresis | I0 = 16.5 mA |
| lifts the load above | 16.1 mA + 2.23 mA per mN m | 4 loads, <= 1 mA residual |
| the load wins below | 6.3 mA + 0.73 mA per mN m | 3 loads |
| holds a released load (0 mA) | ~8.5 mN m after a lift, more after a lowering; varies with angle | release sweeps |
| no load | held at 20 mA, moved at 25 | breakaway + lever-alone runs |

One consistent reading, with the motor's torque `k (I - I0)` above the edge:

- `k = 0.677 N m/A` (0.68 mN m per mA), `I0 = 16.5 mA`
- driving friction `0.51 x |load|` -- the lift line's intercept sits at the
  edge, so no constant term forward
- back-drive friction `0.51 x |load| + ~7 mN m` -- the fall line's 6.3 mA
  intercept; the constant matches the hand back-drive (5-8 mN m)

Only as good as its assumptions: `k` and the 0.51 are split by assuming
forward and back-drive friction scale alike, one servo, static and slow.

**THE SQRT LAW IS OUT IN THIS RANGE.** `righting_servo` maps Goal Current to
stall torque as `tau = ts sqrt(I / I_stall)`, which says 50.5 mN m needs
~4 mA past the edge. It took ~110.

## THE CONSEQUENCE, before anything else

At the bike's `righting_current: 300` the sqrt law gives ~0.47 N m of
stall torque. The measured line gives `0.677 x (300 - 16.5) = 0.19 N m`,
and lifting a load L costs `1.51 L`, so ~0.13 N m of liftable load. That
is a ~3.7x cut in what the model thinks the righting servo can lift at 300
counts, and it reaches every self-righting margin: "300 counts lifts the
bike only under plug" (righting_servo's docstring), `self_righting.py lift`,
the linkage configs' `limits.torque_nm: 0.55` -> counts conversion (0.55 N m
at the output while driving needs `16.5 + 0.55 x 1.51 / 0.677` ~ 1240 mA by
the line, ~830 without the driving friction -- either way near or past the
910 Current Limit).

BUT the line is measured only to ~130 mA. Above it nothing is measured on
the XC330. The XL330 stall sweep (BAM issue #14) found current vs duty
following the bus law (`I ~ d^2`) above ~100 mA, and duty is what sets
phase current and torque -- so the sqrt behaviour may take over higher up.
Extrapolating the line to 300-910 mA is exactly as unjustified as
extrapolating the sqrt law down was.

## Steps

0. **Measure 130-910 mA first.** The lift sweep with heavier loads on the
   same fixture: 117 + 38 + 34 g all at 44 mm is ~81 mN m (lift ~200 mA by the
   line); to reach 300 mA needs ~125 mN m -> ~290 g at 44 mm; 910 mA would
   need ~0.4 N m -> ~0.9 kg (check the hub pins: 0.4 N m at the 6 mm bolt
   circle is ~17 N per pin across four Phi 1.5 pins -- ~9 MPa in PLA shear,
   printable, but the lever arms and the idler take it too). `servo_lift.py
   --mass-g M --radius-mm 44` as run; `--currents` up to 900; `fit` over all
   loads. Decide: one line, a line then sqrt, or something else. Everything
   below assumes the answer is a line or piecewise; adjust step 1 if not.

1. **`src/aow_sim/righting_servo.py`**: add a measured law beside the bus
   law, selected by a constructor arg (`current_law="measured"|"bus_sqrt"`,
   keep `bus_sqrt` for comparison, as `braking` keeps "regen"):
   - motor torque from the clipped current demand:
     `tau_m = k sign(i) max(0, |i| - I0)`, then limited by the motor line
     `ts (s u - w/w0)` at `u = +-1` (the no-load ceiling stays measured:
     11.5-12.1 rad/s on both bench units);
   - braking (demand opposing motion) keeps "plug" -- measured, 242/243
     frames -- with the same edge and gain;
   - `current` (what Present Current reports) = the demand clipped, as the
     bench saw: Present Current reads exactly the goal at stall.
   Rewrite the module docstring's bus-current paragraph with the bench
   numbers; keep the sqrt derivation as the rejected alternative, with why.

2. **Gearbox friction on the servo's joint**, in `pre_step` next to the
   torque (MuJoCo's `frictionloss` cannot scale with load):
   - driving: output torque `tau_m / (1 + 0.51)` in the direction of motion
     (equivalently friction `0.51 |tau_out|`);
   - back-driven (load moving the motor): friction `0.51 |tau_load| + 7 mN m`;
   - at rest: stick while `|net| < ` the back-drive static value (~9 mN m at
     low load, the release sweeps), slide below it (sliding ran 30-40 deg/s
     once going: sliding friction well under static);
   - direction/angle/history effects documented, NOT modelled (agreed).
   A first cut can be `frictionloss = 7 mN m` plus the driving efficiency
   factor, and the load-proportional back-drive term later.

3. **Constants into `config/bike_params.yaml`**, `servos.xc330_t181`, each
   with `source: measured` and a pointer to the plan doc:
   `current_deadband` 0.0165 A, `current_torque_gain` 0.677 N m/A,
   `gearbox_friction_fraction` 0.51, `backdrive_friction` 0.007 N m.
   THIS MOVES `plant_digest`. Work CLAUDE.md's "Before changing a physical
   parameter" list: re-export the deploy bundle (`python -m
   aow_sim.export_deploy`), list the policies it makes provisional (anything
   trained with the swing linkage in the plant), note it in `docs/status.md`.

4. **`floor_rig`**: `servo_friction_ma` / `servo_friction_nm` go -- the
   servo now carries its own friction. Remove from `config/floor_rig.yaml`
   (it is 22, flagged an upper end), `floor_rig.py` (`add_chain`,
   `servo_friction_nm`) and `tests/test_floor_rig.py` (the servo-friction
   test becomes "the attached servo's friction is the righting servo's").

5. **Tests** (`tests/test_righting_servo.py`, marker `righting`):
   - a simulated lever -- the fixture's geometry, 34 g / 117 g at 44 mm --
     lifts above and falls below the measured lines within ~5 mA (encode
     the bench table in the test; `traces/` is not in git);
   - no torque below the 16.5 mA edge at stall;
   - the no-load ceiling unchanged (11.8 rad/s at Goal Current max).
   Then the `righting`, `contact`, `geometry`, `deploy` markers and the
   full suite (it moves a digest); read the red-set verdict.

6. **Re-derive, do not carry forward**: `analysis/self_righting.py lift`
   ("300 counts lifts the bike"), the teleop starting cap
   (`limits.torque_nm` -> counts), `analysis/servo_modes.py`'s mode
   comparison, and the floor rig's `stiff_roll` / `fixed_roll` presets
   (their counts were chosen under the sqrt law). If 300 counts no longer
   lifts the bike, that is a design finding, not a test to loosen.

7. **Docs**: `docs/status.md` (what moved, what is provisional), the bench
   plan's pointer here, `docs/measurements/servo-measurements.yaml` gets the
   lift/fall table as an entry beside `xc330_current_position`.

## Not in scope

- The steering (mode 3/4): kept at the datasheet 0.80 N m, noted in
  `bike_params.yaml` as possibly ~0.5 (bench 0.53-0.69); to be measured in
  place on a bike test rig.
- BAM-style identification of the same servo: `docs/plans/bam-current-position.md`.
