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
