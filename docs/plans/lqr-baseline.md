# The analytic LQR: health log

Status: **reference** (2026-10-02). The LQR is a baseline, not what drives the bike (`CLAUDE.md`). Its current red set is in `docs/status.md`; this file holds the dated findings behind it.

## Log, 2026-09-16 to 10-01 (from status.md)

*Moved verbatim from `docs/status.md` on 2026-10-02, when that file was rewritten as a short navigation layer. "Above", "below", "Health" and numbered "What to do next" items refer to status.md as it was then (`git show d0dffeb:docs/status.md`).*

Fixed on 2026-09-22 though it greened
nothing else: the gain schedule's +-1.2 m/s ends were identified with the bike
IN THE AIR (settle_rolling snapshotted mid-bounce, zero contacts, so the fit
said no input did anything); `settle_rolling` now refuses an airborne state
and the deploy bundle is re-exported (only those two gain rows moved).
`test_gain_schedule_designs_everywhere` is green: its fit bar is `MIN_FIT_R2`
and the "reversed-caster" sign-flip assert is dropped -- the identified plant
has steer acting on roll with the same sign at every speed. The LQR group is
not reachable by re-weighting (16-point sweep); `q_roll_rate` 60 clears
`sprint[-0.5]` alone -- not applied.

**THE LQR NOW SEES THE DRIVE SERVOS (2026-09-22), 7 -> 3.** The identified
model had no state for them, and the crawl acts only through the servo loop
(nothing in the first 5 ms, -0.38 rad/s of roll rate by 80 ms), so a fit over
one control period read its column as zeros and the design balanced on steer
alone: 50-330 rad of steer per rad of roll, the +-15 deg relay below. Two
states added (`linearize.STATE_NAMES`): `crawl_rate`, the differential shaft
speed, and `crawl_lag`, the differential servo lag. Both are MEASURED
(`balance.CrawlSensor`), never read from the simulator: the drive servos'
Present Position counts through the odometry RateFilter (on the Pi,
`w_servo_a/b` via `crawl.feed`), and the integral of commanded minus measured
differential, leaking at `crawl_lag_tau_s` 0.2. Swing-linkage standstill hold:
steer 9.4 -> 0.1 deg RMS, pinned 100% -> 0%, roll 5.0 -> 0.05 deg RMS. At rest
on teleop's sensors (TM151 + odometry, 3 seeds x 20 s) the swing-linkage bike
held all three, where it fell in 1-7 s. **The plain bike does not** (re-measured
2026-10-01, below: falls 3 of 3 at 11-19 s). Both sprints and three command_heading cases went
green; reverse_circle went red again. `DriveController._K0`, the RL envs'
feedforward crawl fallback, stays the 8-state design (no exported move uses
it: all 66 are `action_space: full`). `--lqr` starts teleop on it, and a
`--record` trace logs both controllers in one convention: `rl_*` (what the
general policy would observe, from the same sensor estimate) and `lqr_*`
(the LQR's error state). Not LQR states, measured: pitch moves 0.06 deg std
while driving against the TM151's 1.5 deg RMS error; forward speed is held
within 0.01 m/s by the separate speed loop.
NOT on hardware: the crawl_lag stand-in for the XC430's internal integrator
is unmeasured.

**THE BIKE CAN FLY IT (2026-09-22), as a bench mode.** Station key `l`
switches `run_bike` between the policy and the LQR; the station's status line
shows what the bike is actually flying. The bundle now carries the design at
the bike's 100 Hz too. In sim at 100 Hz on the bike's sensors it STANDS (12 of
12 x 20 s) but falls within 2-6 s of starting to DRIVE, from the TM151
attitude error (odometry alone: drives and turns, falls in a standing 180).
`pytest -m lqr` at 100 Hz is 6 red of 36 (circles, two forward turns) against
2 at 200. Checked on the Pi itself, from /tmp: builds from the bundle with no
mujoco, 1.15 ms median tick. Nothing has run with torque.

**`r_steer` 6 -> 20 (2026-09-22), 9 -> 7.** Clears pivot[180] and
reverse_circle, turns nothing red, leaves the 40 s standstill hold unchanged
(0.83 vs 0.85 deg tail roll RMS). The +-15 deg steer relay found alongside it
(command on `steer_limit_deg` 73-97% of a hold) was the PRE-crawl-state design,
which balanced on steer alone; the crawl states above removed it on truth. An
MPC was costed as the fix then (N=40, 200 ms horizon: 2.1 ms on a Pi 3B+,
numpy only) and is not needed for that any more.

**THE RELAY COMES BACK UNDER THE TM151 ERROR MODEL (2026-10-01).** Standstill,
zero command, teleop's own step loop run headless (default teleop bike: no
linkage, ideal drives, 200 Hz), 20 s, 3 seeds; scratch harness, not in the repo:

| sensors | fell | roll RMS | steer peak-to-peak | diff RMS |
|---|---|---|---|---|
| truth | 0 / 3 | 0.06 deg | 0.4 deg | 0.4 rad/s |
| odometry only | 0 / 3 | 0.16 | 2.9 | 3.0 |
| `tm151` only | 2 / 3 | 3.6-9.0 | **30** | 43-58 |
| **`tm151` + odometry (teleop default)** | **3 / 3** (11-19 s) | 6.6-7.2 | **30** | 28-34 |
| `tm151_filter` + odometry | 0 / 3 | 0.2 | 2.1 | 1.0 |

So `run_drive --teleop --lqr` oscillates and falls standing still, and
`--ahrs tm151_filter` holds. Mechanism: the standstill design still carries a
large steer-per-roll gain, and `tm151` reports 1.5 deg RMS of attitude
wander -- the datasheet dynamic figure, against ~0.2-0.3 deg measured at one
mount (Sensor modelling row above). `tm151_filter` (0.1 deg residual) is the
closer model of the part. The 100 Hz "STANDS 12 of 12" above was not
re-measured.

**AHRS error costs far more WITHOUT the velocity estimate, for both
controllers (2026-10-01), mechanism OPEN.** LQR on `tm151_filter` with no
odometry falls 3 of 3 in 2.4-3.1 s; with odometry it holds. RL
`cmd_curriculum2b` (trained odometry + `tm151`) on the 20-command grid:

| odometry | AHRS | survival | score |
|---|---|---|---|
| on | `tm151` (as trained) | 0.90 | 0.572 |
| on | none | 1.00 | 0.723 |
| off | `tm151` | **0.75** | **0.318** |
| off | none | 1.00 | 0.642 |

Ruled out: the AHRS-only code path (same seed and physics, its readings are
bit-identical to the odometry path's), the world-frame rotation of velocity
by the AHRS yaw, and coupling through the estimator's own AHRS input (LQR
still holds 3 of 3 with the estimator fed true attitude). Left: something in
the estimate itself -- its lag, or measuring at the front contact rather than
the body -- makes both controllers less sensitive to a false lean. Untested.

**Re-measured, not moved.** Several registry reasons had drifted from what the
tests report: `test_reverse_circle_tracks` FALLS ("radius err 0.681" was a bike
lying down); `test_gain_schedule_designs_everywhere` is one fit cell (yaw_rate
at -0.5 m/s) at R^2 0.9489 against 0.95, not a failed design;
`straight_sprint[-0.5]` stays upright only because of the 4 ms delay. Each
steer-servo half was confirmed causal by switching it off with
`bike_params.yaml` untouched.

**Deprecated, -7.** The trajectory flick (`moves/flick.yaml`, optimised
2026-07-19, never re-run) and the scripted flip: early LQR-handoff manoeuvres
that fall on the current plant. Skipped (`DEPRECATED_MOVE` in test_drive.py),
noted in `optimize_flick.py`, `control/flick.py`, `DriveController` (a
`FutureWarning` on use) and `control.flip`. `flick_rl` is NOT deprecated.

**LQR: marginally functional, and that is the intended state.** Holds the bike
at standstill at 1.17° peak roll over 40 s ON TRUTH; on teleop's default
sensors it falls standing still (see "the relay comes back" above). Recovered by two weights after the
drive plant was armed: `q_roll_rate` 6.0 → 30.0 (the velocity-PI servo puts a
pole at the origin that the 8-state model does not carry; de-tuning `r_drive`
over a 100000× range does not substitute) and `q_steer` 0.5 → 5.0 (the steer was
pinned at its clamp 86% of the time, which read as oscillation and was really a
saturated actuator). Those two are the knobs to reach for when contact moves.

**`MIN_FIT_R2` is 0.93, and the bar was the thing that was wrong.** Nothing has
ever cleared 0.98 — not the current plant (**0.9489** since the 09-16 steer
damping change, 0.9412 before it), not the servo without its integral term
(0.9757). The fit gets *worse* as the contact gets more realistic:

| `contact_solref` dampratio | 0.3 | 0.5 | **1.0 (ships)** | 2.0 |
|---|---|---|---|---|
| worst R² | 0.8148 | 0.9297 | **0.9412** | 0.9602 |

(That row is at `steer_kv` 0.05 and has NOT been re-swept at 0.0676; the
shipping column alone is now 0.9489. The monotonic trend is the load-bearing
part and a damping change to a different joint has no reason to reverse it.)

The drop test implies ~0.30 — the worst-fitting value. There is no setting that
is both faithful and well-fitting. **Re-derive the bar from the measurement
rather than carrying 0.93 forward**, once the contact is measured.

## The 2026-10-05 front wheel

The new front tire (80 mm crown, `mujoco-modeling-decisions.md`) makes steer
push harder on roll: steer cambers the wheel by steer x sin(rake), and on an
80 mm crown that moves the contact ~6x further than on the old 14 mm one.

**Standstill, on the tests' plant (detailed drivetrain, truth).** The LQR
rang between its +-15 deg steer stops and fell after 4.9 s, sooner when
kicked. Fixed by `control.lqr.q_steer_standstill` 50: a heavier steer weight
in the v = 0 design only, faded out by 0.25 m/s on the schedule. Holds with
roll-rate kicks to 2 rad/s. What did not work: q_steer raised everywhere
(30-400 fix standstill but fall at 0.8 m/s, except 100); widening the
identification's roll amplitude x2-x8 (falls at every width). `pytest -m lqr`
red at q_steer_standstill 5 / 20 / 50 / 100 / 200: 21 / 18 / 16 / 18 / 18
(before the bundle re-export, one stale-bundle case in each).

**At speed, not fixed.** Straight at 0.8 m/s survives but drifts 0.149 m
off line; turns at 0.8 m/s fall. q_steer 3-12 x r_steer 8-50 and lean_ff
0.55-1.0 turned none green. Registered red, pending sign-off.

**Teleop (`run_drive --teleop --lqr`) is a different loop from the tests**:
the AHRS error model (`--ahrs tm151`) and the odometry estimate by default,
and the startup policy's drivetrain (ideal for the current pointer).
Standstill, 20 s, 3 seeds, that loop run headless (scratch harness):

| sensors | old wheel | new wheel | new + q_steer_standstill 50 |
|---|---|---|---|
| truth | held 3/3 | held 3/3 | held 3/3 |
| `tm151` + odometry (teleop default) | fell 2/3 (11.7, 14.5 s) | fell 3/3 (1.8-2.9 s) | fell 3/3 (2.5-9.9 s) |
| `tm151_filter` + odometry | held 3/3 | fell 3/3 (3.7-4.3 s) | held 3/3 |

So in teleop the LQR falls on its default sensors with either wheel (as
2026-10-01 found for the old one), sooner on the new one; to see what the
tests see, `--ahrs none --no-odometry`; `--ahrs tm151_filter`, the closer
model of the part, holds.


---

## Moved from `docs/status.md`, 2026-10-16 (verbatim)

The health log as status.md carried it before its 10-16 rewrite; current
state is in status.md.

**NEW FRONT WHEEL (2026-10-05).** The original bike's wheel, measured
(102.5 x 24 mm, 7 mm shoulders), its 10 mm flat tread modelled as an 80 mm
crown (`GUESS`; why: `mujoco-modeling-decisions.md`). Costs: the LQR needed
`q_steer_standstill` to stand and still falls turning at speed
(`lqr-baseline.md`); the then pointer's (`cmd_curriculum2b`) eval score fell 0.572 -> 0.454, mostly big
turns (4 of 6 survive, was 6 of 6). **Outstanding:** every policy is
provisional until retrained on this wheel.

**THE DETAILED DRIVETRAIN IS THE DEFAULT PLANT (2026-10-02).** The overlay
moved, unchanged, from `config/drivetrain_model.yaml` into `bike_params.yaml`'s
`drivetrain_model:` block, which put `plant_digest` at 9953690d, exactly the
stamp of the four smoothness-loss exports; removing `min_pinion_radius` the
same day moved it on to **95630b212f03ffc4** with no physics change. `drivetrain_model.base_params` is the IDEAL drive: the LQR is
designed on it (`linearize.design_plant`, an ideal twin of whatever model it
is handed) and flown on the detailed one; the flick/pivot/ball envs and every
export without a `drivetrain_model:` record (`cmd_curriculum2b` included) run on it;
teleop compiles its startup policy's own plant. **A model built with the
block IS the ideal plant until `DrivetrainSim.attach` switches it** (native
drive zeroed, measured rotor inertia, rigid roller couplings off, slop tendons
on; `release` switches back, both pinned bit-exact by tests): so every loop
that never attaches the hook -- contact studies, settles, the other moves'
envs -- runs exactly as before, and the hooked ones (RL envs, teleop,
`balance.run`, `record`) get the detailed drive.
`drivetrain_model.attach_hooks` is the one call a physics loop needs.

**Tests, 2026-10-14**: `878 passed, 52 skipped`, **no accepted reds**.
`test_sensor_modes[0.19]` (the sensor-mode policy at tau 0.19, red since the
new front wheel) went green with the GeneralEnv observation fixes (fresh
prev_action, AHRS at reset): red at their parent commit, green at it.
Before that, **2026-10-09**: `1 failed, 812 passed, 52 skipped`. The LQR's 15 accepted reds
are skipped with the deprecated `lqr` marker and still judged under
`pytest -m lqr` (15 failed, 20 passed, unchanged). The teleop tests now run the
viewer's `pre_step` (the detailed drive's XC430 loop): without it the new
pointer had no drive torque and four went red.

**Tests, 2026-10-05**, after the new front wheel: `16 failed, 790 passed,
17 skipped`, red set unchanged (16 accepted). 8 registered 2026-10-05,
pending the user's sign-off: seven LQR cases at speed or under a push, and
the sensor-mode policy at tau 0.19. Two 2026-10-02 entries went green and
left. `test_hw_telemetry`'s gearbox check now runs the drive airborne: on the
floor the bike fell over and the check measured how it landed.

The 2026-10-02 entries: **7 registered 2026-10-02** (user: "fine for now"). All seven
are the analytic LQR (`PivotController` is `LQRBalance` plus a pivot
reference) flown closed-loop on the detailed drive: `test_tilt_recovery[lqr]`,
`test_command_heading[0.0-90]`, `test_stop_from_circle`,
`test_straight_sprint[-0.5]`, `test_pivot_completes_upright[+-90, 180]`.
Bisected by switching the drivetrain's parts (an earlier harness, 5 reds):
detent alone and slop alone pass all of them; the servo part fails them.
Within it, **the input-shaft Coulomb friction (0.15 N m at the servo, ~10%
of stall) is most of it**: off, 2 stay red; off and latency zeroed, 1.
Latency alone fixes nothing. Which tests are red shifts with how each test
steps between its `run()` segments (11 if the drive is never released), so
read the family, not the list. Before the change: 3 failed, 748 passed.

All three are the analytic LQR in reverse: `command_heading[-0.5-90]`,
`command_heading[-1.2-90]` (since the headset friction, 09-30) and
`reverse_circle`. The standing-endurance test is registered red and
`prospective` (skipped unless named): `cmd_curriculum2b` fell 10 of 18.
Its entry left 10-09 with the pointer move: `smooth_temporal` passes.

**Digests, checked 2026-10-05**: `plant_digest` 12d9639e3ce90f36,
`design_digest` 5fe8c19379988031; `deploy/bundle.npz` re-exported and
matches both (worst schedule fit R^2 0.962). **Copy it to the Pi by hand**:
`deploy/` is gitignored. No export carries this plant: the four
smoothness-loss exports are at 9953690d, `cmd_curriculum2b` at e1ec36bf -- both
before the wheel. History: `params-digest-split.md`.
