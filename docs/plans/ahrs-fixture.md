# AHRS fixture: the TM151's dynamic error

Status: **parked** (2026-09-24): no new mounts; the fixture comes back only if the bike's AHRS misbehaves. Code: `analysis/ahrs_fixture.py`, CAD `aow_sim.cad_ahrs_fixture`. The model it produced is the opt-in `ahrs_level: tm151_filter`.

## Log, 2026-09-23 to 09-24 (from status.md)

*Moved verbatim from `docs/status.md` on 2026-10-02, when that file was rewritten as a short navigation layer. "Above", "below", "Health" and numbered "What to do next" items refer to status.md as it was then (`git show d0dffeb:docs/status.md`).*

**AHRS fixture (2026-09-23), for the DYNAMIC error the standing falls hinge
on, and for where the AHRS may go.** `analysis/ahrs_fixture.py`: yaw servo 151
carrying roll servo 152 carrying the TM151, encoders as truth, on the Pi.
`session` (15.5 min) runs rest, fast steps, chirps, a replay of the sim's own
standing flights, yaw-only replay and random sway behind one ident prologue;
the joint axes and the accelerometer's lever arm are fitted from each capture.
Chirps report the AHRS's GAIN AND PHASE about the roll joint per octave, which
separates under-reading the lean from over-reading it where an RMS cannot.
`tune` sweeps the servos' P/I/D on a replayed flight, no AHRS needed.

*Where the sim bike rolls:* NOT about its ground contact. Over 60 standing
flights the instantaneous roll centre (v_lat / roll rate at the rear axle,
|roll rate| > 30 deg/s) is a median 261 mm above the ground, p25-p75 202-319;
one fixed centre at 252 mm explains 69% of the axle's lateral velocity. The
same on the 28 flights that never fell (253 / 262 mm). That is ABOVE the
as-built AHRS (~180 mm) and ~130 mm above the CoM, so the fixture's lever arm
stands for distance from that centre, and 0 mm is a real configuration.

*Measured, d = 30 mount (2026-09-23), two `session`s with the same seeds,
servos P 1500 / D 1000 (`tune`, loaded: best on both joints, stable at every
setting up to P 2500).* The sensing point fits 32-41 mm BELOW the roll axis
(the vendor accelerometer reads -1 g face up -- easy to invert), so this mount
stands for an AHRS ~35 mm below the roll centre; the as-built one is ~80.

**The encoders are not the truth at this resolution.** Between each servo's
output shaft and the sensor are a horn, screws and a printed bracket. In the
step holds the plate sat 0.13-0.54 deg from where the encoder said, depending
on which way it leaned, while the fused output matched its own accelerometer to
0.01 deg; a hand wiggle with torque on moved the plate ~2 deg about roll for
~1 deg at the shaft (XL330 backlash, about right by hand), and 1 deg in pitch,
which no joint drives. Heading ran off -4.0 deg over one session and -1.9 over
the identical repeat, then held flat for 10 min torque-off: the yaw stage
settling, not a gyro property (it would repeat). So the headline is ENCODER-
FREE: the TM151's raw gyro integrated through each segment, pinned to its
accelerometer in the still holds either side (`self_reference`). Its gyro scale
checks against the accelerometer to 0.3-0.4% over the step holds; the
encoder-based fit said 0.95-0.98, which was the mechanism losing motion.

**The TM151 is, to a good approximation, a complementary filter whose time
constant rises CONTINUOUSLY with motion** (`filter-model`; `rest_model` in
the report). tau fitted per 2 s window, ~490 windows a session, both runs:

| mean deviation of \|acc\| from its median | tau, median | mean rotation rate | tau, median |
|---|---|---|---|
| < 2 mg | 0.20 s | < 1 deg/s | 0.22 s |
| 2-5 mg | 0.35-0.43 s | 1-15 deg/s | 0.49-0.51 s |
| 5-10 mg | 0.58-0.64 s | 15-60 deg/s | 1.0-1.1 s |
| 10-50 mg | 1.0 s | > 60 deg/s | 0.8 s |

So "at rest" is below a few mg, and a standing bike is always past ~10 mg:
**~1 s while riding**, never gyro-only (the resting 0.19 s is 0.41 deg off
in motion, gyro alone 0.39). At rest the same filter at 0.19 s tracks the
fused tilt at r 0.92-0.93 (the accelerometer alone 0.84).

*What raises tau* (`sweep`, 2026-09-24: constant-rate yaw sweeps at 5-40
deg/s): **not slow acceleration.** In the cruises the |acc| deviation below
~2 Hz was 0.72-0.92 mg (holds 0.33-0.41) and tau still went to 0.5-5 s
(median ~2); the 9-12 mg of |acc| deviation there is ABOVE 2 Hz -- the
stage vibrating as it turns. So rotation, or fast vibration, and this rig
cannot give one without the other. For turning that is the useful half:
the slow lateral acceleration of a turn is not what the gate looks at.

**The rig is PARKED (2026-09-24)**: no new mounts; the bike gets built,
and the fixture comes back only if the bike's AHRS misbehaves.

*Where the accelerometer is, settled by turning the mount over* (d = 30 mount
turned 180 deg about x, 2026-09-24; the sensing point now ABOVE the roll
axis). Every fit keeps it ~12 mm off the yaw axis the SAME way round in the
sensor frame -- sessions (10.4, 10.0) -> (8.8, 9.5) mm, yaw chirps (6.2,
10.1) -> (8.0, 7.9) -- so it is the part, not the servos: not the fit's
signs (synthetic check), not gyro/accelerometer timing (+-5 ms moves it
< 1 mm). The roll-rich sessions put it 32.6 mm from the roll axis below and
40.8 above; a rattle fixed to the rig adds to one side and takes from the
other (synthetic rig: a phantom arm that does not turn over), so the chip is
~36.7 mm out, down at the circuit board. `sim_ahrs.TM151_ACCEL_OFFSET_M` =
(8.4, 9.4, -5.5) mm from the housing's centre at mid-height. The datasheet
drawing's triad sits ~5 mm the other way: illustrative. (A flipped `jog`
said 58 mm: 6 s of roll, before the tape was re-pressed.)

*The flipped mount filtered harder.* Rest tau 0.35 s (0.19 upright), best
moving tau ~2 s (0.7-1), and the varying part of the replay error 0.10-0.13
deg (0.16-0.19 upright; with the mean offset 0.28-0.34). The lever-arm
prediction at tau 2 is small (0.06 deg) and NOT found (k -0.1 to -0.55): at
that tau there is little to find, so the sign test is inconclusive rather
than failed. The plate is top-heavy that way up and rattles more at rest
(accelerometer tilt std 0.12-0.14 deg against 0.10-0.11): **the vibration the
part feels moves tau, not only rotation**, which fits the sweep. The sim
cannot know its bike's vibration, so tau_motion is a range to randomise.
In that session the 8 steps and 2 chirps ran on YAW, not roll: `sweep`'s
`set_defaults(axis=...)` had rewritten the --axis default every command
shares. Fixed (`--sweep-axis`, and a test on every command's default); the
rest, replays, yaw replays and sway are unaffected.

The error column is the segment's varying part. What the model leaves wanders
with a 1/e time of 1-4 s (0.5-5), worst excursion 0.28-0.62 deg: the
accelerometer pulls it back on the ~1 s tau, so in standing it is BOUNDED.
Open: **every moving segment also sits at a steady -0.15 to -0.28 deg**
(always negative, both runs; yaw-only -0.05 to -0.08) of which the lever arm
explains -0.04 -- the sensor, or the reference's gyro integration in motion,
not yet told apart. The same filter prices other placements (same motion,
same side): 0 mm -> ~0.1 left, **80 mm -> 0.37-0.39 on the replays**,
150 mm -> 0.8. Beyond ~35 mm that is extrapolation; the flipped mount (35 mm
on the other side) is its test -- same size, opposite sign.

- NOT CIRCULAR where it matters: gyro scale against the accelerometer
  (0.3%); timing against the ENCODERS, which backlash moves in amplitude, not
  time (gyro 1-3 ms late, fused 1-5 ms early, encoder timing good to ~1-2
  ms); the rest noise explained by the accelerometer; the lever arm fitted
  separately (from alpha x r + w x (w x r)) and then predicting the error.
- It REPEATS: run-to-run, the fused error varies no more than the plate does.
- Static: 0.015 deg RMS noise; absolute level is unmeasured (no level
  reference), bounded by the fitted accelerometer bias, 1-2.5 mg ~ 0.1 deg.
- The sim's `tm151` level is 1.5 deg RMS of independent Gauss-Markov noise at
  0.19 s -- the right tau AT REST, the wrong shape in motion, and 5-10x too
  large for standing at this height. **New, opt-in: `ahrs_level:
  tm151_filter`** runs this filter on the sim's own (corrupted) gyro and
  accelerometer at the AHRS site, with the chip offset, tau gated on the
  smoothed rotation rate (0.19 s -> 1.0 s between 1 and 15 deg/s) and a
  0.1 deg / 2 s residual wander. On the fixture's raw data it matches the
  real fused output to 0.103 deg moving / 0.006 at rest (upright mount) and
  0.195 / 0.012 (flipped). No policy trains on it yet; `tm151_to_site` is
  identity (TM151 x forward, z up) until the bike's mount exists.
  **Not yet a drop-in on the sim bike** (eval grid, 2026-09-24, 15 s
  fixed-command episodes,
  `general_rl_cmd_curriculum2b`, same seeds): survival 0.95 truth, 0.90
  `tm151`, **0.05 `tm151_filter`** (it drives, then falls after 0.8-11 s,
  median ~3.5; the AHRS is reset each episode, so not a reset fault) -- the estimate's PITCH runs to +10-17
  deg on a pure hold. The sim's accelerometer on a standing bike is violent:
  |acc| > 0.2 g off 1 g in 60% of samples, > 0.5 g in 34% (near free fall to
  3 g within half a second -- contact bounce, sampled at an instant), and
  ungated that averages to ~8 deg of tilt. Skipping the accelerometer beyond
  a gate (`FILTER_ACC_GATE`, `run_drive --ahrs-gate`) gives 0.55 at 0.3 g,
  0.65 at 0.1 g, 0.70 at 0.05 g -- still well short of 0.90. Open, and not
  answerable on the fixture (it never saw > ~50 mg): is the sim's
  accelerometer realistic (item 2, the contact), and does the TM151 gate on
  |acc|. The real bike's AHRS log answers both. **Kept as a WORK IN
  PROGRESS** (2026-09-24): driven in teleop it "doesn't drive THAT bad",
  a bit better with `--ahrs-gate 0.1`; it improves as the bike does. Teleop's
  respawn now calls `SimAhrs.restart()` -- without it the filter kept the
  fallen attitude and the fresh bike fell at once. If the real AHRS
  misbehaves, the next rig is likely a pseudo-treadmill for the whole bike
  rather than more fixture mounts. One unit, room temperature, standing motion only: a steady turn
  or forward acceleration lasting more than ~1 s is exactly what such a
  filter reads as tilt -- in a coordinated turn the specific force lies in
  the bike's plane, so it would pull the reading toward UPRIGHT, by up to
  the lean itself -- and nothing here has measured it.
- The fixture's AHRS logger stalls 0.19 s at t = 183.5 s in BOTH sessions
  (inside `rest`; harmless there). Whether the onboard reader has the same
  stall is unchecked.

    python analysis/ahrs_fixture.py analyse traces/ahrs_fixture/260923-221539_session_loaded_p1500d1000
    python analysis/ahrs_fixture.py repeat traces/ahrs_fixture/260923-221539_session_loaded_p1500d1000 \
        traces/ahrs_fixture/260923-224528_session_loaded_p1500d1000_repeat
    python analysis/ahrs_fixture.py filter-model traces/ahrs_fixture/260923-224528_session_loaded_p1500d1000_repeat

Also: **the TM151's clock runs 0.3% fast**; the Combo gyro is in rad/s; the
Pi logged undervoltage on a different supply brick that session and none on
the usual 5 V 2 A one. Captures under `traces/ahrs_fixture/` (Dropbox).

*The fixture CAD (2026-09-23).* `python -m aow_sim.cad_ahrs_fixture` writes
the `AHRS fixture` feature (studio `ahrs-fixture-gen`), inserted into the
`ahrs-fixture` Part Studio: nine printed parts round the servo and TM151
envelopes, each named with the side that prints UP (every mount pin points
up as printed). Yaw servo far end +X, so its horn-to-idler U goes round the
SHAFT end, on the roll servo's side; roll servo far end down, horn toward the
TM151, which sits on the roll axis beyond the horn, centred over the yaw axis
(the only shape where the 0 mount puts the sensing point on both axes). Both
servos stay centred at 180: the idler U's cannot turn over, so +-d is the d
mount turned half a turn about the roll axis -- one printed part, a screw ON
the axis so it lands the same either way.

The COVERS are full-wrap now (a new option on the `X330 case shell` feature,
off by default there, cover half only): walls round the whole servo, the cap
only over the far-end wrap, walls past the cap cut back at exactly 70 deg so
they print cap-down -- carried round the shaft-end corners and across the end
wall as a cone about each inner corner of the U (was a V from the outer
corner, which left each corner's first layer a level 2.3 mm strip hanging off
the side wall: seen on the first print, 2026-09-23); down to the back face where
the base is not, and only to just above the cable connectors in their window
-- the user's hand-drawn walls in wing-linkage-shorter, read back through the
API. The BASES keep the far-end wrap: a wrapped base cannot be printed. The yaw
base's plate runs out to +X past the yaw servo's far end (clamp there; four
10-32 clearance holes, 1.5 x 1 in, on the tab), and the roll base carries a
cable clip on each side, printed straight up from its bed. Lessons from the user's printing, now DEFAULTS in
`servo_mounts.yaml`: base and cover each grip half the case (grip 2 -> 11.5),
horn and case pins half length (2.6 -> 1.3, 3.0 -> 1.5; full-length ones
snapped). Joints: the as-built 6-32 flat head + small-pattern nut, a 45 deg
ridge with a hole-wide flat and tapered ends, nut slots clean through the nut
part, and two 0.2 mm sacrificial bridging layers over each slot whose roof is
a ceiling as printed. Bores lying horizontal as printed (the yaw arm's, the
TM151 mount's) are teardropped; the check now finds any that are not.

DERIVED: legs 23.62 from the shaft (the cover's corner + 1 -- nudged out from
20.29 by the wrap), roll horn 32.94 from the yaw axis (TM151 centred), roll
axis 56.10 above the yaw horn face at d = 30 -- the swept roll stage clears
the yaw arm by 8 mm, the as-printed d mount binding. TM151 mount plate
widened to +-18.35 so the pin reliefs keep a 1.2 rim. `--check` builds all three
mounts in Onshape in ONE call (one each until 2026-09-23): one body per part and NO interference at rest or at
+-15/30/45 deg on either joint, with the envelopes carrying the real pin
holes and idler recess so every pin, shell, horn well and plug is tested,
not excused; a print check of every face steeper than 70 deg; and a check
for horizontal holes left with a flat crown (shown to catch both teardropped
bores with the teardrops switched off); and for HANGING EDGES -- a level,
convex edge whose two faces both rise from it, which no face check sees
(shown to catch the four V corners, then 0 with the cones). What it flags is
bridges only: the nut-slot layers, the J4 groove's flat and one blind hole
end. The `ahrs-fixture` Part Studio also holds the user's own chamfers and
fillets after the generated feature; a push alone updates it in place
(tested 2026-09-23 with a marker attribute, both directions), so it is never
deleted and re-inserted. All seven read back OK after the corner change. TM151 hole pattern 4x Phi 2.10
(M2) on 31 x 30, dimensioned; the sensing point's height is a GUESS (7.1).
