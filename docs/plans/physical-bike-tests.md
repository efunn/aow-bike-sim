# What to test now the bike balances

Started 2026-10-24. The assembled v2 bike balances on the floor under the
pointer policy, hands off, for as long as it was left (50 s on a zero
command, until stopped), and turns on the spot when asked. This is the list
of what that makes testable, in the order that unblocks the most. Contact
tests from here on are on the **whole bike on the target floor**, not on the
drop rig or the subsystem stations.

## Where it stands (2026-10-24)

| | |
|---|---|
| balance | 14-50 s engages, mostly ended by hand; it fell a few times during the turning sessions (17:23-17:32) and once on its tether, and not in the 50 s no-input engage |
| pivots | heading followed the station: commanded -23 -> 67 deg, flown -35 -> 68 deg |
| standstill | it never stands still: speed estimate 0.07 m/s median, 0.25 m/s p95 on a zero command (run 174349) |
| shake | a ~9 Hz oscillation in both drive commands, with roll and yaw rate following; the sim has none (below) |
| policy | the pointer, `general_puffer_fixes_72M`: plant bc66ca20 (before the v2 bike), steer rate from the exact joint velocity (`steer_rate_encoder` absent = `joint`), while the Pi forms it from differenced counts |

**Fixed on the way, both config-only:**

- **Drive A/B roles swapped** (`config/onboard.yaml`): drive_a is the RIGHT
  servo, 102. Each servo forward alone, bike held on the floor: the left one
  slides the rear left, which is the opposite of the model's drive_a. The
  plant's error is the sign of `k_roller` (the wheel is handed through the
  roller bevel mesh; confirmed by flipping the bike over); recorded in
  `omni-wheel-measurements.yaml`. The plant fix waits for a retrain, below.
- **The bike's own U2D2** named in `onboard.yaml`; the AHRS needs an
  ethernet-side USB socket (`pi-bench-bringup.md`, end of section 7).

## The tool

`analysis/policy/run_vs_sim.py <traces/bike/run>` plots a run's longest
engaged stretch beside the run's own policy standing in the sim on its own
sensors (same AHRS level, odometry and detailed drivetrain): time series on
shared y-limits, Welch spectra, and a std / >2 Hz / >10 Hz table. Run it on
every session below; it is the common yardstick.

What it showed on run 174349 (no inputs, 49.6 s), bike / sim:

| signal | >2 Hz RMS | >10 Hz RMS |
|---|---|---|
| roll rate (gyro x) | 27.3 / 10.4 deg/s | 11.1 / 2.5 |
| pitch rate (gyro y) | 3.4 / 0.9 deg/s | 3.0 / 0.6 |
| steer target | 3.3 / 1.0 deg | 0.70 / 0.08 |
| drive A cmd | 5.4 / 2.2 rad/s | 2.7 / 0.5 |
| drive B cmd | 7.3 / 2.6 rad/s | 3.8 / 0.6 |

The gyro held still reads 0.25-0.5 deg/s above 10 Hz, so this is motion,
not sensor noise. The ~41 Hz peak in the commands is the 9 Hz one's image
through the policy's 50 Hz zero-order hold (50 - 9), not a second
oscillation; the sim's bump near 45 Hz is the same for its ~5 Hz content.

## The tests

### 1. Fly a policy trained on this plant  [no code]

`general_puffer_rake20_80M` (and 72M) trained on 6446d8fa, the plant the
bike is, with `steer_rate_encoder: counts`, the signal the Pi sends. The
pointer has neither. Same session as 174349: engage, no inputs, 30-60 s,
then `run_vs_sim.py`. If the 9 Hz peak goes, the cause was the policy's
training, not the hardware. Known against it: it wobbles more than the
pointer going straight at 1.2 m/s in the sim (`status.md`, next-step 3), so
keep the speed low.

```bash
./scripts/bike.sh bench --torque    # with control.general_move repointed, or run_bike's policy flag if one is added
```

(Repointing `control.general_move` is a one-line edit that moves neither
digest.)

### 2. What makes a 9 Hz drive oscillation, in the sim  [sim only]

The loop is drive -> sideways push -> roll -> policy -> drive. Sweep what
the real drivetrain could have more of than the model, one at a time, on
the pointer standing, and see which one makes the sim's spectra look like
the bike's: `roller_slop.half_play_deg` (7.8), the servo's
`command_delay_s` / `measurement_delay_s` (4 ms each), `coulomb_friction`
(0.15, one value for both sides), `rotor_inertia`.
`analysis/policy/slop_sensitivity.py` already does the slop's two GUESSes
on the eval grid; this needs the same with a spectrum out. Whichever one
reproduces it names the measurement for test 3.

### 3. Re-measure the reassembled drivetrain  [wheel in the air]

The drivetrain came apart and went back together, so its 09-13 fits are
for a different assembly: the per-side friction (bench: servo 102's side
~25 % higher, `0.16:0.21`), the roller play, the detent. Bike upside down
or on a stand, the same Station A captures, `analysis/servo/drivetrain_fit.py
coast controller detent`. A pair, if the sides differ, goes in as
`coulomb_friction: [drive_a, drive_b]`, which is now [102 right, 101 left].

### 4. The steer on the built fork  [bike on a stand]

`analysis/servo/steer_response.py` drives the goal open loop; it was run
on a bare shaft (10-17). On the fork with the wheel the load and the
headset friction are real. `steer_friction.py` maps the friction by angle
(the bushing's tight spot). Both feed `actuators` / headset friction in
`bike_params.yaml`.

### 5. The AHRS mount calibration  [a level jig]

Still a `GUESS`, accepted at every start by `--bench`. Held as upright as
hands manage, the bike reads a steady 2-3 deg roll in standby. The chassis
exists now, so the procedure in `untethered-setup.md` ("AHRS mounting
calibration", into `config/ahrs_mount.yaml`) can be run on it; until then
the policy balances about whatever offset the mount carries.

### 6. Standstill drift, measured  [the target floor, a camera]

The 0.07 / 0.25 m/s above is the odometry's estimate. Tape a start mark,
film from above, compare with the record's `pos`. Answers whether the bike
wanders or the estimate does -- the sim's standstill is 0.11-0.21 m/s for
these policies (`status.md`), so drift is expected; how much is real is not.

### 7. Contact, on the whole bike, on the target floor

The subsystem rigs (`pre-assembly-bench-checklist.md` F1-F3, the drop rig)
were for a bike that did not exist. Now:

- **Drop rebound (F2, whole bike).** Torque off, drop from a few measured
  heights onto the floor, film at 240 fps, the AHRS recording
  (`bench/force_drop.py --ahrs` logs the TM151). The sim twin is the
  whole-bike model dropped the same way. Label results by **sink at bike
  weight in mm**, never by `dampratio` (`floor_sweep.py` does).
- **Traction limit (F3, whole bike).** Pull sideways and forwards with a
  spring scale until it slides, drives holding (torque on, standby is not
  enough: the policy servos are limp there), and read the peak.
- **The record as the identification.** Every balancing session is
  already a closed-loop trace. Running `run_vs_sim.py` against the sim at
  a few `contact_solref` / friction points (as `floor_sweep.py` varies
  them) asks which floor makes the sim's spectra match the bike's. Only
  after tests 1-3, or a drivetrain mismatch will be fitted as contact.

### 8. Queued for the next retrain, together

- `k_roller: -2.4` in `bike_params.yaml` AND revert the `onboard.yaml`
  A/B swap, in one change (either alone breaks the sideways sign).
- The per-side friction and slop from test 3; the mount from test 5.
- Train with `steer_rate_encoder: counts` (the pointer did not).

Each moves the plant digest, so every policy becomes provisional at once;
batch them.

## Not tests, but now worth watching

- `run_bike` rides out up to 3 bad servo reads in a row (`BAD_READS_MAX`;
  one `rc=-3002` ended a session on 10-24). The record's meta counts them
  (`bad_reads`); if they become frequent, the bus wiring or the 3 Mbps baud
  is the question.
- Records before 2026-10-24 have 101 as drive_a; each run's meta `roles`
  says which.
