# Project status — 2026-09-09

**How to read this.** This file is the navigation layer: what is true now, what
to do next, and where the detail lives. It is rewritten, not appended to. Every
section either states a fact measured on the date at the top of this file, or
points at the plan doc that owns the subject. **If you want the reasoning,
follow the link — it is not repeated here.**

Rewritten 2026-09-08 in a docs triage. The previous version had grown to 2801
lines across six partial amendments and had become the sole record of several
workstreams; that material moved into the plan docs listed below, verbatim.

---

## Where the project is, in five lines

1. **The simulator is trusted for control design.** Parametric MuJoCo model,
   procedural omni-wheel contact, and — since 08-26 — the *sensors* the bike
   will actually have, in the loop and in training.
2. **RL drives.** `general_rl*` policies balance and drive from a live
   (velocity, heading) command. The analytic LQR is a reference baseline only.
3. **The onboard software path is built and proven in sim** — hardware shim,
   deploy bundle, odometry, AHRS protocol — with no assembled bike to run it on.
4. **Hardware has started.** Four servos on a bench, 2026-09-01; the rear
   drivetrain assembly built and characterised by hand 2026-09-12/13. The
   Digi-Key order landed 2026-09-08: Pi 3, cables, electronics. No pack, no
   charger.
5. **The goal is a functioning bike with sample videos, in weeks.** Everything
   below is ranked against that.

---

## What to do next

Ranked by what unblocks the most, not by interest.

| # | do | why now | doc |
|---|---|---|---|
| 1 | **Weigh the electronics stack and pack** | Two `GUESS`es die in ten minutes; the parts are on the bench today | `first-physical-test.md` §0a |
| 2 | **Contact calibration — P0, P0b, P1, once per floor** | Needs no printing: a weight, a caliper, slow-mo, a tilting board. The contact is the least-measured thing in the sim and the one no policy has been randomised over — and the SPREAD across surfaces is what sets the randomization range | `floors-and-the-contact-model.md` |
| 3 | **Finish the drivetrain station** | Built and characterised 2026-09-12/13: belt ratio 3.0 confirmed, a 7.5° detent in the differential, and a velocity-loop resonance at ~22 Hz at firmware P 400 that P 200 does not have. Roller slop measured by hand 2026-09-13: ±1.5 mm at the roller's 22 mm diameter, 15.6° p-p, from the same gear chain as the detent (20 detents per roller turn). `k_roller` 2.4 confirmed by counting roller turns. Open: choose the firmware Velocity P **and I** gains — a P/I grid cut time stuck at the diff detents from 49 % (factory) to 12 % at P 400 / I 3840, but P 400 rings near 22 Hz and I 3840 rings harder (peak 1.09–1.20), and the sim must model whichever ships — then a torque-scale check (D5, a known added inertia -- no lever arm needed) and fitting the five drivetrain `GUESS`es from the captures. **The sim cannot pick the gain (2026-09-14):** on the drive model fitted to these captures, policies trained at each gain tie — 3 seeds each, median 0.749 at P 100 against 0.750 at P 400, each on its own plant — but P 400 buzzes the rear wheel ~3× harder at 8–32 Hz. Decide on the physical bike, with a policy trained at the gain that ships | `drivetrain-measurements.yaml`, `drivetrain-model.md` |
| 4 | **Umbilical bring-up on the laptop** | Verification steps 1–2 need no pack at all | `untethered-setup.md` §"Bench power" |

All four are bench work. **The sim-side item is being taken now:**

> ### → NEXT, in progress: fix the eval score's directional gate
>
> `_score` is `survive_rate × track_geo` (`train_general_rl.py:243`).
> `speed_ratio_fwd` is computed and deliberately excluded, so the score cannot
> see a policy that abandons a direction — demonstrated once at a cost of 12M
> steps, and **every future long run is exposed**. Ranked risk #2.
>
> Open decision before the edit: gate `BestByScore` on a
> `min(speed_ratio_fwd, speed_ratio_rev)` floor (selection-only, leaves every
> recorded number comparable) versus adding a directional term to `_score`
> itself (stronger, but re-bases every score in `docs/` and every
> `moves/*.yaml`). See `eval-score-rewrite.md`.

Not next, and deliberately: the ball shot (works, off the path), the
privileged critic (speculative), the odometry rewrite (flown around; its
tests were restructured 2026-09-21, see Health).

**The self-righting wings came off that list on 2026-09-17: the co-rotating
four-bar is BUILT and operating** -- this file had carried "design done, build
last" for eight days. Open on it: read `righting_sign` (stow is settled at 180)
off the bike (below), and Station C / R6 for `righting_current` in counts.

**The crank's servo is now modelled as the firmware runs it (2026-09-21).**
`righting_servo.CurrentBasedPositionServo`, on by default under teleop
`--swing-linkage` (`--righting-ideal` for the old PD + clip). The loop
regulates BUS current: the position PID is converted to amps, Goal Current caps
it, and the motor line is the duty ceiling. So stall torque follows
sqrt(Goal Current), and **300 counts is ~0.47 N.m at 12 V, not 0.27**. It
brakes by PLUGGING (reverse duty), which is measured: 242/243 bench frames
braking against the motion had reversed PWM. The 4 ms command delay is measured
(frame-quantised; see below). The
old clip let the crank reach 17.5 rad/s against an 11.8 no-load. `[` / `]`
step Goal Current by 20 counts, as the ground station does. Fall set, policy
on throughout, crank to centre at the hand-off gate, 416 counts (0.55 N.m at
stall): **2/2 at 12 V, 1/2 at 11.1 V, 2/2 at 9.9 V**, and latching the
endpoint recovers the fall to the left only. That is one pose per side and not
monotonic in voltage, so it is marginal, not settled. Under the unmeasured
alternative braking reading ("regen") it is 1/2 everywhere, which is why the
braking data matters. Bench results are under `xc330_current_position` in
servo-measurements.yaml: Goal PWM is NOT a duty ceiling in mode 5, and the D
term's per-tick reading holds to order of magnitude.

**The steer servo is modelled as the firmware runs it (2026-09-21).**
`actuators.steer_clip: duty` builds the mode-4 law, clip(kp e) - kv w,
natively: a kp-only actuator plus kv as joint damping. It is identical below
saturation (lead < 22.9 deg), and capped at 11.8 rad/s above it, where the old
clip held 23.2. `actuators.steer_command_delay_s` adds a 4 ms native actuator
delay. That was measured on both XC330s, frame to the servo's own Position
Trajectory (`xc330_command_delay`), and the same 4.0 ms held in mode 5. It is
frame-quantised: the true value lies in (2, 4] ms. **The plant digest moves**
to `8d8b25a809ea2f1a`, and the deploy bundle is re-exported. No `moves/` export
matched the previous digest anyway. The LQR is now DESIGNED with actuator
delays zeroed and flown with them (`linearize._undelayed`). Designing on the
delayed plant gave garbage (fit R^2 0.74), because the reduced model has no
state for the delay line. With that, the balance LQR is unaffected up to 8 ms,
but **the drive-mode LQR falls going straight at 0.6 m/s with >= 2 ms**. That
knocked over the old odometry fixture (those tests no longer ride the LQR,
see Health). The 180 deg pivot's 0.10 deg overshoot on the clip cleared at
`r_steer` 20. A latency-aware LQR (the delay line as state) is NOT the fix for
the at-speed reds: re-measured 2026-09-22, the same six are red with the delay
at 0 and the clip at `total` (see Health). Driven in teleop, the RL policy felt
unchanged by the clip; the delay is not yet teleop-tested.

---

## The five workstreams

| workstream | state | blocker | owner doc |
|---|---|---|---|
| **Simulation & model** | Working. 17 parameters still `GUESS`. A detailed drivetrain (fitted XC430 loop, diff detent, roller slop) exists as an opt-in overlay, in the eval env, teleop and training (per config). Trained at P 100 and P 400, 3 seeds each (2026-09-14): a tie on score. Teleop builds a policy's own drivetrain from its record | Physical parts to measure; the built bike, to confirm the drivetrain model | `mujoco-modeling-decisions.md`, `drivetrain-model.md` |
| **Control — RL** | Working, and primary. Trains against the onboard sensors | Crab still one-sided; `turn_asym` stuck ~0.2 | `general-rl-improvements.md` |
| **Sensor modelling** | Largely DONE. Velocity estimate, encoder quantisation, TM151 error — all in training, validated against a real unit over USB | Dynamic attitude accuracy: measured at one mount (2026-09-23): ~0.2-0.3 deg RMS on replayed standing flights against the sim's 1.5, most of it the sensor's own acceleration read as tilt through a complementary filter (tau 0.19 s at rest rising continuously to ~1 s in motion); the sim's error model is not re-fitted yet. Next: the other mounts ("What to do next" #5) | `sensor-workstream.md` |
| **Control — analytic (LQR)** | Reference baseline only. Marginally healthy | Nothing now; degrades when contact moves | `old/stationary-balance-controller.md` |
| **Hardware / untethered** | Servo bench 2026-09-01. Rear drivetrain assembly on the bench 2026-09-12/13, hand-held, recorded with `analysis/drivetrain_bench.py`. Bus at 500 Hz on the Mac only after `adjust-ftdi-latency`. **Onboard software readied for a Pi bench session 2026-09-15** — ground station, firmware-gain writes, fourth servo, fall cut/re-arm; none of it has touched hardware | Firmware P-gain choice, torque calibration, then the chassis | `pi-bench-bringup.md`, `first-physical-test.md`, `drivetrain-measurements.yaml` |
| **CAD** | Layout, drivetrain, steering and righting stations pinned. Electronics packing deferred on purpose. The X330 idler side and the 6-32 crank/idler joint are generated features now (2026-09-23), beside the horn pin and case shell | AHRS fixture brackets, "What to do next" #5 | `cad-onshape-workflow.md` |
| **Self-righting mechanism** | Four-bar built and operating. Its servo is modelled in current-based position mode, bus-regulated, plugging (2026-09-21); centring at hand-off recovers marginally | `righting_current` untuned; D gain and braking near zero current unmeasured | `wing-linkage-design-and-optimization.md`, `righting_servo.py` |

---

## Where everything is written down

`docs/plans/` holds reasoning and grows forever. This file is the layer on top.
Every doc opens with a **status banner** saying whether it is active, open,
parked or reference — read that before the body.

| doc | what it owns |
|---|---|
| `first-physical-test.md` | **the build order.** Which parts unlock which unknowns, in what sequence. Plus the three servo bench stations and the 09-01 bench results |
| `untethered-setup.md` | the physical bike: power, wiring, onboard software, Pi setup, verification. The umbilical path is §"Bench power" |
| `pre-assembly-bench-checklist.md` | DRAFT: floor-rig ideas and proposed pre-assembly experiments, nothing built or run. The rig's current state is in MuJoCo (`aow_sim.floor_rig`, `--rig` in teleop) |
| `pi-bench-bringup.md` | **the Pi bench session**: what the four servos + TM151 + Pi 3B+ can prove without a chassis, the seven code gaps that block it, and what to measure |
| `sensor-workstream.md` | the odometry/AHRS arc and the full policy standings — why sensor-trained policies win 1.00 to 0.20 |
| `eval-score-rewrite.md` | how policies are SCORED, why the early numbers hid a failure, and the command-distribution audit |
| `seed-sweep-and-personalities.md` | **the 12-seed sweep**: determinism, the failure taxonomy, why reverse is easier, early abort detection, and the options not taken |
| `general-rl-improvements.md` | collected RL findings. Reference, not a queue |
| `mujoco-modeling-decisions.md` | why the model is built the way it is. Reference |
| `odometry-rewrite.md` | the estimator itself, and why rewriting it was skipped |
| `floors-and-the-contact-model.md` | **the multi-floor plan** — solref/solimp history, why the parameters read backwards, the per-geom blocker, and how the spread across surfaces becomes a randomization range |
| `aow-contact-approximations.md` | the contact surrogate survey, the timestep/mesh result, and the two blocked drive-plant fixes |
| `drivetrain-model.md` | **the detailed drivetrain**: servo, detent and slop fitted to the Station A captures, the replay check against them, what the existing policies do on it, and why firmware gains and policy are one decision |
| `cad-onshape-workflow.md` | the Onshape round trip and its API quota, plus everything drawn so far |
| `wing-linkage-design-and-optimization.md` | the righting mechanism as it now stands |
| `self-righting.md` | where recovery stops being possible; the fall cases. Reference |
| `params-digest-split.md` | the two digests and what each answers |
| `asymmetric-actor-critic.md` | a parked option, kept to cover the bases |
| `ball-shot-move.md` | the ball shot. Works, parked |
| `plans/old/` | six retired docs — built, superseded, or never started |

`docs/measurements/` holds hand-entered protocol/data pairs, plus
`servo-logging.md`: which registers to record off a Dynamixel, per frame and
per session, bench versus onboard, and why each earns its bytes.
`docs/cad/` holds generated CAD output (`.fs`, `.png`, the layout YAML) — never
hand-edit those; regenerate with `aow_sim.cad_layout` / `cad_servo_mount` /
`cad_swing_linkage` / `cad_ahrs_fixture`.

---

## Health

**Test suite, measured 2026-09-22** with `pytest -n 10 --dist load`:

    3 failed, 563 passed, 16 skipped, 66.8 s
    red set unchanged (3 accepted failures) -- tests/expected_failures.txt

3 = 2 analytic LQR (both REVERSE turns at -0.5 m/s: command_heading[-0.5-90]
and reverse_circle) + 1 standing endurance, a policy metric in its own registry
section. It was 28 on 09-21. Wall time went 42 -> ~60 s with the endurance
test (18 x 60 s flights in one test). Fixed 2026-09-22 though it greened
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
on teleop's sensors (TM151 + odometry, 3 seeds x 20 s) it now holds all three,
where it fell in 1-7 s. Both sprints and three command_heading cases went
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
(0.83 vs 0.85 deg tail roll RMS). Found on the way, and NOT fixed by it: the
LQR's steer runs as a +-15 deg relay. The standstill design asks ~50 rad of
steer per rad of roll (40-90 deg for sub-degree motion), so the command sits
on `steer_limit_deg` 73-97% of a hold for every `q_steer` 5-40 x `r_steer`
6/20, and ~95% of the last 0.5 s before the sprint[0.8] fall. Not the steer
servo model (76-79% with the delay or the clip off) and not the contact: all
six stay red at `contact_solref` dampratio 0.5 / 1 / 2 / 4, and 1.0 is the
LQR's best (22 / 8 / 17 / 19 red of 35). A constraint-aware controller (MPC)
is the candidate; the box-constrained solve fits the Pi 3B+ (N=40, 200 ms
horizon, 20 warm-started iterations: 2.1 ms against the 5 ms tick, numpy only).

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

**Odometry tests rewritten, -12 +2.** They compared the estimator open-loop
against sim truth on an analytic-LQR trajectory, so the delay's fall on the
straight errored eleven of them. Moved to the RL policy, the longitudinal
error split as 5-7 mm/s gearing arithmetic, 2-3 reference point, and the rest
(47-109) REAR-WHEEL SLIP: they were measuring the contact model under a
driver and calling it the estimator. Now:

| file | question | marker |
|---|---|---|
| `test_hw_odometry.py` | the estimator's code, on synthetic no-slip inputs, exact to 1e-9 -- 83 tests, 0.14 s. Geometry read from `bike_params`, not the estimator: planting a zeroed rake, a 2% gearing error or a 5% wheelbase error is caught (48 / 24 / 40 red); built from the estimator's own constants, all three passed | `pure` |
| `test_odometry_in_the_loop.py` | `control.general_move` on the sensors its yaml says it trained with (estimate + TM151): survives all four regimes (max roll <= 9.5 deg), tracks within 15% when moving; slip tripwire at 1.5x a recorded baseline (rear 67-142, front lateral 35-50 mm/s) | `contact`, `policy` |
| `test_sim_odometry.py` | encoder test now under the policy; counts-vs-ideal lag 39.8 mm/s RMS, tripwire 60 | `contact` (+`policy`) |

The tan-coefficient test is retired (it measured its fixture's conditioning).
`test_hold_ramps_to_full_speed_with_auto_repeat` moved to the RL policy and
passes.

**NEW, registered: the policy falls over standing still on its own sensors.**
Estimate + TM151 attitude (teleop's default): 64 seeds x 60 s, 32 fell, MTBF
79 s (95% CI 56-116), highest in the first 10 s; teleop
`--teleop --swing-linkage` fell at ~35 s. It comes with the orientation-error
model (alone: falls at 3.2 s; gyro-only, estimate-only and truth stay up), and
the power-on misalignment does not predict it (p 0.23). The test (`tests/test_policy_endurance.py`, a
POLICY METRIC in its own registry section: it moves with the policy, not the
code) is 18 fixed seeds x 60 s, pass if <= 4 fall, derived from a 600 s MTBF
target; today 9 of 18 (11 before the 09-22 yaw-drift fix moved the AHRS rng stream). Paths: longer standing episodes in training (evals are 5 s, episodes
<= 15 s), and a bench RMS for the TM151 -- the model uses the datasheet's
"<1.5 deg" bound as its RMS.

**What makes it fall** (`analysis/ahrs_fall_cause.py`, 2026-09-22, 64 seeds):
the ROLL orientation error, and nothing else to speak of -- roll-only error
36 -> 24/64 falls, everything-but-roll 3/64. ALL 36 falls go LEFT (-roll).
Not one tick: the bike is savable on clean sensors until 0.3-0.8 s before the
fall, preceded by a ~1 deg roll-error excursion held ~0.5 s. But no excursion
does it ALONE: pulses up to 3 deg on a calm clean-sensor bike never fall, and
replaying each fall's own recorded error onto a calm bike reproduces 0/36 from
the last 2 s and 5/36 from the last 12 s. The fall needs the sustained sway
the error keeps up (roll RMS 2.4 vs 1.1 deg clean) -- a trajectory, not a
pattern -- which is why a bench DYNAMIC RMS is the number that decides it.
Handedness is the policy's, not the sensor's: injected phantom leans that
push the bike left fall 3x as often as ones that push it right.

Standing is not still, and the falls are failed CATCHES. The rear wheel makes
a forward excursion (~220 mm in ~0.85 s, steer swinging 30-45 deg each way)
5.3 times a minute -- the forward creep on a zero command is their sum -- and
32 of 36 falls are one that is not caught. Nothing in the rear-wheel motion
warns before it. Excursions go both ways equally, but only LEFT ones fail
(32/142 vs 0/98); nearly all follow ~1 s of roll error claiming a left lean,
and a failed one over-steers ~100 deg. But that average trigger does not
CAUSE falls: injected at up to 3x, a clean bike never falls (0/60), and on
top of the normal error it moves which windows fall without raising the rate
(7.0% control, 7.0-8.3% injected, 470 paired windows). Figures:
`analysis/plots/ahrs_fall_cause_excursions.png` (group means) and
`..._excursion_examples.png` (three single events per group).

**It is this policy, not the sensor model.** The same standing flight across
`moves/personality0..11` (64 seeds x 60 s each): `personality1` -- byte-
identical to curriculum2b -- is the WORST of twelve (36/64, MTBF 75 s);
`personality0` and `personality8` never fell (MTBF > 1041 s at 95%), 6, 7
and 11 fell once each. Fall side is per policy (1 and 5 left, 2 and 10
right). But every good stander is one seed-sweep-and-personalities.md
records as ignoring heading commands; `personality1` is the only one that
turns as told. So repointing `control.general_move` buys standing at the cost
of driving -- the fix is a policy that does both, not a pointer edit.

The slip numbers rest on `friction_sliding` 0.9 and `contact_solimp`, both
GUESS, so contact calibration (item 2 above) will move them and the
baseline.

On 09-17, +36 passing against 09-14, all from the onboard bring-up work: 18 in the new
`tests/test_hw_runbike.py` (the fall guard and the UDP command struct, both
`pure`), 11 added to `test_hw_dynamixel.py` (the fourth servo, gain
resolution), 2 in `test_hw_ahrs.py` (a non-Combo frame is skipped silently,
which is what made a healthy TM151 read as a dead one), and 2 to the `boundary`
list (`hw/ground.py`, `control/recovery.py`).

**THE CONTROL LOOP RUNS ON THE PI (2026-09-16).** 60 s, four servos and the
TM151 both live, policy stepping, nothing energised: **tick jitter mean 0.17 ms,
p99 0.93, max 2.91** at 100 Hz, against a p99 < 1 ms gate. Per-phase the servo
read is 2.08 ms, the SyncWrite 0.58, and `DriveController.step` **0.03 ms** —
the same as on an M4 Mac, so SBC compute was never a risk and the bus is the
budget. Getting there fixed three bugs of mine (a preflight that raced its own
reader, a fixed-block serial read capping the AHRS at 100 Hz, and a telemetry
percentile costing up to 49.5 ms per tick) and one design trap (`poll_timeout`
defaulting to the same 50 ms as the staleness limit). `SCHED_FIFO` made it
worse and is off, and the `gc.disable()` that fixed the jitter once was REMOVED
once the telemetry fix made it unnecessary (re-measured: indistinguishable).
**With four servos ENERGISED and the AHRS live, 30 s: 0.24 / 0.79 / 1.56 ms**,
no tick over 10 ms. The 40-60 ms outlier that only appeared under torque was
`np.percentile`'s first call, not electrics -- warmed before the loop. A steer
zero is now captured at startup: the bare XC330 came up 1.4 turns wound, so the
first absolute command would have slammed it. Detail: `pi-bench-bringup.md`.

**The AHRS path is proven on hardware (2026-09-16).** `hw/ahrs.py::parse_frame`
decodes real TM151 Ep_Combo frames — quaternion, gyro 0.2 deg/s at rest,
`|accel|` 9.772 against g 9.807. `AhrsReader` also gained an opt-in
`poll=True` that requests each frame, for a unit whose output profile has Combo
switched off: **230 Hz, age-at-tick 4.7 ms mean / 10.4 ms max, zero stale
raises in 300 ticks**. Detail and the three ways the "no configuration API"
claim was checked: `pi-bench-bringup.md`.

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

**PUSH IS BACK, and this time it is in flash (2026-09-16).** The Combo output
profile was ticked from a Windows machine; the earlier attempt through the
QEMU/Linux GUI applied live and was gone at the next power cycle. The proof is
a power cycle, not a checkbox — the unit was unplugged from that machine and
plugged into the Mac, cutting its only power, and came up streaming Combo
unprompted at **198.3 Hz** with nothing transmitted to it. Through `AhrsReader`
at 100 Hz ticks: **201.4 Hz, requests 0, age-at-tick 2.32 ms mean / 4.87 p99 /
4.93 max, zero stale raises in 367 ticks** — better than the polled path on
every number. `control.onboard.ahrs_poll` is now **false**.

It still streams `Status`(22), `RPY`(35) and `Raw_GyroAccMag`(41) at 200 Hz
alongside Combo: 33.7 kB/s total, of which Combo is 8.7. Free on USB CDC, and
`parse_frame` drops the other three on the command check — but it would NOT fit
a 460800 UART (336 kbps of 460.8, no flow control), which is the wiring
`untethered-setup.md` specifies for the Zero 2 W. Untick them before that move.

**Preflight now checks the sensor's own QoS.** `Ep_Combo` carries
`Ep_Status_SysState`, so the grade costs one mask and no extra subscription.
The bar is 3 (`basic service`) rather than 4, because 3 is what a healthy unit
holds in the first ~30 s after power-on — exactly when the bike is being armed —
and 2 is excluded on the vendor's own words for it, "very limited measurement
accuracy". The bench unit reads 5 (`very good service`). This is the only one
of preflight's four checks that can catch a sensor whose numbers are all
plausible and all wrong.

**The whole loop ran on the Pi against the new config (2026-09-16), torque
off.** Tick jitter **mean 0.15 ms, p99 0.91, max 1.50** on a 10 ms budget; pack
11.9 V. The AHRS ran pushed — `requests 0`, 200.6 Hz, 73.0 B/frame,
age-at-tick **mean 2.09 / p99 4.32 / max 5.18 ms**, zero ticks over 10 ms and
zero stale raises in 984 — which is BETTER than the Mac (2.41 mean) and half
the old four-stream push figure (4.3 / 9.7). The position gains were read back
off the bus rather than trusted: id 103 mode 4 P900/I0/D0, id 104 mode 5
P700/I0/D1400.

Three config gaps the run found, all fixed: `control.onboard.servo_ids` was
READ by `run_bike` and never defined, so it fell through to `(1, 2, 3)` and
every bench run died on `read id=1 Model Number: rc=-3001`; `righting_id` was
4 against a bench numbered 101-104; and `Goal Current` on the righting servo
came up at `Current Limit` (910), i.e. no torque cap, with nothing setting it.
`righting_current: 300` is now written next to the gains at every startup. See
`hw/control_tables/README.md` for the reboot measurement that says why it has
to be every startup.

**Torque on, 2026-09-16, operator holding the rear assembly.** The steer does
NOT drift: it settled at -57 deg within a second and stayed there (-58.2 to
-56.1 over 10 s), because with the wheels turning the odometry moves and the
policy's observation stops being constant. The unbounded -229.2 deg/s march
seen with `--no-torque` is an open-loop artifact, not a property of the
controller. Tick jitter mean 0.14 / p99 0.76 / max 0.84 ms, the best yet, and
torque dropped cleanly on all four servos at `--seconds`.

**Extended position's turn counter is RAM (measured).** Driving the steer one
revolution put `Present Position` at 4931 counts; a reboot brought it back to
835, losing exactly 4096. So steer winding cannot accumulate across power
cycles toward `clamp_extended`'s +-256 turn ceiling. `control_tables/README.md`
carries the table and why the first version of this test proved nothing.

**The ground station binds the viewer's keys (2026-09-16).** Arrows and `/`
are primary, `w`/`s`/`a`/`d`/space are aliases, and `/` re-aims the heading at
the bike the way `run_drive`'s `zero_command` does — which needed `psi` and
`righting_current` added to the telemetry, since the station cannot know
either. Still a TERMINAL station: there is no graphical one, and reusing the
MuJoCo viewer as a live mirror is the obvious candidate rather than a second
UI. **`steer_zero_deg` is pinned at 180** — the orientation the fork clamp will
be built to, chosen so straight-ahead sits mid-range rather than on the
single-turn wrap. It is NOT yet true of the hardware, so bench sessions want
`--steer-zero capture`; without it the bare shaft (73.4 deg) is told it is
106.6 deg off straight, which is visible in the telemetry as a completely
different drive split.

**Telemetry schema v2, and the mirror's half of it (2026-09-16).**
`hw/telemetry.py` holds `build` (Pi, mujoco-free) and `apply_pose` (laptop)
fifty lines apart on purpose: the failure they are both exposed to is drifting
apart, and a renamed field is read as a `.get()` default rather than raising.
438 B/packet, 21.4 kB/s at 50 Hz, versioned so a stale deploy fails at connect.

The rear wheel renders from TWO NUMBERS. `_aow_assembly`'s gearbox is a pair of
linear tendon equalities, so hub, ring and all eight rollers are a closed form
of the input-shaft angles -- no solver needed in a render. Checked against
MuJoCo's own constraint solve after 1500 driven steps, agreeing to <2e-3 rad.
What the bike genuinely does not know (ride height, front-wheel angle, the omni
internals' absolute phase) is DRAWN and the module says so per field.

**Two link bugs found by measuring the packet rate rather than trusting it.**
A station sending at 50 Hz got **27.8 Hz** of telemetry back. `CommandLink._run`
compared `now - last_tx > period` against a clock driven by datagram ARRIVALS,
which aliases; fixing it to an accumulated deadline gave 34.7 Hz, still short,
because the loop could only transmit on a wake and `recvfrom` only wakes on a
command or a 100 ms timeout. Waiting on `select` with the time-to-deadline as
its timeout gives **50.0 Hz, gap mean 20.02 ms / p99 29.6 / max 40.7**, and the
control loop is untouched (tick jitter mean 0.15 / max 1.30 ms). Both bugs read
as "the radio is struggling" and neither was.

**The radio is not the constraint, measured.** Pi to Mac, zero loss at every
size tried: 218 B, 1.1 kB, 3.9 kB, 11.9 kB and 31.9 kB per packet at 50 Hz
(the last is **1.56 MB/s**), and 1.1 kB / 7.9 kB at 200 Hz. Encoding is the
budget that binds, and barely: a full pose plus four servos x six registers is
855 B and **154.8 us on the Pi, 1.55% of a 10 ms tick**. Caveat: house wifi,
stationary bike, one client -- and `untethered-setup.md` specifies a
LAPTOP-HOSTED AP for the real link, which is one hop rather than two. Re-measure
before trusting it with the bike moving.

**A latent test bug, not mine, found on the Pi.** `pytest tests/` gave 3
failures in `test_hw_no_mujoco.py` and `pytest tests/test_hw_no_mujoco.py`
alone gave 18 -- and running that file alone is exactly how you check the
import boundary on the bike. The fixture's teardown did `sys.modules.clear()`
then restored a snapshot taken at SETUP, so numpy (first imported during the
test, via `control.policy`) was evicted permanently and the next re-import hit
`cannot load module more than once per process`. Invisible on a laptop, where
an earlier test file has always imported numpy first. Now 1 failure alone, and
that one legitimately needs mujoco.

**The mirror is up (2026-09-16).** `mjpython -m aow_sim.run_drive --mirror
aowbike.local` drives the real bike and renders it in the MuJoCo viewer with no
physics at all: `interactive.mirror_loop` calls `frame(model, data)` per
rendered frame, `telemetry.apply_pose` writes the pose, `mj_forward` does the
kinematics. Teleop's own `_KeyState`/`_Axis` and ramp constants drive it, so
hold-to-accelerate and the 35 deg lead clamp behave exactly as in the
simulator, and `MIRROR_KEYS` is module-level so a test can prove both stations
land on the same `OperatorState`.

Checked headless against the live bike over 12 s: **rendered roll tracks
telemetry roll to 0.003 deg**, chassis sits at the settled rest height, steer
matches, 401 packets, zero empties. `_overlay`'s dial needed no change -- green
tick commanded heading, cyan actual -- and it is fed `cmd_psi`/`cmd_v_world`
from the packet, i.e. the command THE BIKE SAYS IT RECEIVED, which differs from
what the station last sent exactly when one was dropped.

Two bugs the live run found. The bike transmitted its telemetry dict before the
control loop had filled it, so the first packets on the wire were `{}` and the
station's version check reported "schema vNone -- re-sync the Pi" against a Pi
that was fine; the bike now stays silent until it has something to say, and
`check_version` tolerates an empty packet, both. And `--mirror` under plain
`python` printed "mirror stopped after 0 telemetry packets" underneath the
mjpython hint, which reads as though it ran.

**Preflight's standing condition got its own escape hatch (2026-09-16).**
The AHRS mount is `GUESS` and stays that way until the bike can be jigged level
on a level floor -- so preflight blocked EVERY run, and the only way past was
`--no-preflight`, which also disarms the |accel|, |gyro| and QoS checks. A gate
that fires every single time is not a gate: it trains the operator to reach for
the flag that turns off the gates which do catch things. Findings now carry a
kind, `--allow-guess-mount` accepts exactly that one, and the error only
suggests it when it would actually clear the block. The mount still blocks by
default -- on the assembled bike an uncalibrated mount is a permanent roll bias
and is worth stopping for.

**Five things the first mirror session found (2026-09-16), all from looking
at it rather than from a test:**

  * **The lead clamp blocked BOTH directions** once the command left the
    +-35 deg band, so the heading command froze with no way back -- and the
    band can be left with no key pressed at all, because the bike moves.
    Presented as "the heading command does nothing". Teleop's own `turn`
    blocks only the direction that GROWS the lead; the predicate is now
    `run_drive.lead_blocks`, module level and tested.
  * **6/7/8 were unbound.** They are teleop's heading snaps (+90/-90/180) and
    pass `clamp=False`, because a snap is meant to lead until the bike catches
    up.
  * **A fixed ride height put the front wheel underground** on a nose-down
    pitch, and floated the rear on a wheelie -- the attitude rotates the body
    about its origin. `telemetry.ground_the_wheels` now puts whichever wheel is
    lower on the floor, exact for a surface of revolution (`centre_z - radius`,
    no bounding-box estimate). Consequence: the mirror can never show a wheel
    genuinely lifting off, because one is always pinned.
  * **The camera was framed for a bike that travels.** It now TRACKS the
    chassis at 0.9 m with `-`/`=` to zoom, which is safe whether or not the
    dead-reckoned position wanders.
  * **`--port`/`--ahrs-port` are in `control.onboard` now** as
    `dxl_port`/`ahrs_port`, by-id rather than `ttyUSB0`. Typing two 70-character
    paths on every run is how `--no-preflight` got into the habit too. Flags
    still override.

**Telemetry runs at the CONTROL rate now, not half of it.** The dict was built
every other tick to match a 50 Hz link; that halving is also 0-20 ms of extra
staleness on top of the transmit period, and the mirror shows it as lag. Both
are at 100 Hz: measured **95.5 Hz, 40.0 kB/s, gap mean 10.47 / p99 20.03 ms**,
and the control loop is unmoved with a station attached (tick jitter mean 0.16
/ p99 0.94 / max 1.50 ms). What remains is irreducible without more work:
AHRS age ~2 ms, one control tick <=10, one transmit period <=10, the radio, and
one render frame <=17 -- so ~25-45 ms typical against teleop's zero, because
teleop's state is local. The mirror is a picture of somewhere else.

**A measurement artifact that bit twice.** Both attempts to measure the
telemetry rate from a loop that drains to the NEWEST packet once per frame
read back the loop's own frame rate (27.8 Hz, then 49.4) rather than the
link's. Counting every packet gives 95.5. Drain-to-newest is right for a
mirror and wrong for a rate measurement, and the two look identical in the
output.

**The mirror's odometry drift is now watchable on purpose.** `pos` is
dead-reckoned and drifts, and that drift IS the odometry error made visible --
so the bike walks around the floor by default, `0` re-centres it, and `p` pins
it at the origin and hides the floor grid (a world reference with no world
motion left to reference reads as though nothing is working).

**The rear wheel's angle is MEASURED now, not integrated.** It was summed on
the station from `w_shaft` x display-dt, which was wrong twice: the dt came
from the packet (the bike's 10 ms tick) while the call happens once per
rendered frame, so the wheels turned at 60% of reality; and `vel` is FILTERED,
so even a correct integral would lag and would lose whatever happened between
the packets that arrived. `read_state` already computes an unwrapped per-tick
position delta in order to difference the velocity, so the bus now sums that
same delta into `turned` and the packet carries it. The station prefers the
sent angle outright and keeps integration only as the fallback for a bike too
old to send one. NOT yet confirmed against a hand-turned wheel on the bench --
the plumbing and the arithmetic are tested, the physical check is outstanding.

**THE RIGHTING MECHANISM IS IN THE MIRROR (2026-09-17).** `run_drive --mirror
HOST --swing-linkage` now draws the co-rotating four-bar from the righting
servo's measured angle. The servo sends `righting_pos` (a new telemetry field;
no version bump, adding one never needed it), and `build_model.
SwingLinkageSolver` turns that into all five joint angles in closed form.

WHY A SOLVER AND NOT JUST THE CRANK. The loop is closed by `mjEQ_CONNECT`, and
an equality is only satisfied by the constraint solver during a dynamics step
-- `mj_forward` computes constraint FORCES, not constraint-satisfying
positions. The mirror deliberately never steps, so writing the crank alone and
calling `mj_forward` leaves the couplers and wings where they were and the
mechanism visibly comes apart. Every joint has to be written.

Verified against MuJoCo's own constraint rather than against the arithmetic:
both `mjEQ_CONNECT` site pairs stay coincident to **< 1e-9 m at 21 travels
across the full +-136.6 deg stroke**, and end to end over a real socket a servo
at 280 deg renders crank +100, wings R +62.1 / L +11.7, gap 0.000 um.

The measured angle beats the commanded one on purpose: a wing held back by its
Goal Current is DRAWN held back, which is the thing an operator tuning
`righting_current` is watching for. Both are sent; `righting` is the fallback
for a bike that reports no reading.

**THE MECHANISM IS BUILT AND SWEEPS ITS MOTIONS (2026-09-17)**, which this
document said was "design done, build last" for eight days. It was constructed
separately; the testbed XC330s are bare, so it can be driven without risk.

`righting_stow_deg: 180` is `source: design` and **forced rather than chosen**:
the stroke is +-136.6 deg in one 0-360 servo range, so 180 +- 136.6 spans
43.4 .. 316.6 and fits, and more than ~43 deg off centre runs an end of the
stroke into extended position where a power cycle loses the turn count.

`righting_sign: 1` is **TBD on purpose and is not a measurement waiting to
happen** -- the motor can be installed either way and the bike's packing
solution is unsettled, so it may still be flipped. Consequence of it being
wrong is entirely visual: the render deploys to the wrong side. Nothing flies
on it and neither digest sees it. One observation when the motor is mounted.

MEASURED IN PASSING, and not what the config's name suggests: this crank
**turns all the way round**. `stroke.crank_travel_deg: 136.6` is the DESIGN
stroke -- what reach and clearance ask for -- not a kinematic limit; the loop
closes at every angle in 360 deg. Pinned by a test, because "it stopped at the
end of its travel" would be the wrong explanation for a station that froze.

Also fixed: the mirror read `stroke.crank_travel_deg` from `hw/ground.py`'s
fixed default rather than from the config the MODEL was built with. The two
`_smaller` configs differ (136.6 vs 132.1 deg), so `--swing-linkage <other>`
would have commanded a stroke the rendered mechanism could not reach.

`test_every_telemetry_field_is_read_by_the_mirror` **now exists.** The
`hw/telemetry.py` docstring has named it since the module was written and no
such test was ever added; the coverage was hand-written assertions that a new
field slips straight past. It partitions FIELDS into pose and display, and
perturbs every pose field on its own to prove `qpos`/`qvel` moves. Confirmed to
bite by breaking the reader and watching it fail.

**THE VIEWER'S LAG WAS THE OVERLAY, NOT THE SOLVER.** Reported as heavy lag
while rolling the bike and backdriving the servos, and the four-bar was the
obvious suspect. Measured instead:

| per rendered frame | before | after |
|---|---|---|
| `SwingLinkageSolver.pose` | 23.7 us | 23.7 us |
| `apply_pose` (whole pose pipeline) | 0.033 ms | 0.033 ms |
| `mj_forward`, worst case (rolled 90, 8 contacts) | 0.098 ms | 0.098 ms |
| **`_overlay`** | **4.004 ms** | **0.256 ms** |

`floor_geoms` walks every geom building a named accessor each time, and the
reference grid reached it TWICE PER GRID POINT -- 208 calls and 8132
`str.startswith` per frame -- to recompute one plane's orientation that cannot
vary across the grid. Hoisted out of `floor_z`, and `floor_geoms` memoised on
the model (weak keys; the set is fixed at compile time). **15.6x, and it is
TELEOP'S BUG TOO** -- same overlay, same path, and it has been there far longer
than the mirror. Pinned by a call COUNT rather than a stopwatch.

**WINGS NO LONGER SINK THROUGH THE FLOOR.** `ground_the_wheels` is now
`ground_the_bike` and includes the righting panels. They are boxes, so the
lowest point is a corner that moves with roll -- upright a fully deployed wing
still clears the wheel contact by 5.7 mm and nothing changes, but at 61 deg it
reaches **78 mm below** it. A rolled bike with a wing out now rests ON THE
WING and the wheels lift clear, which is what the mechanism is for.

Re-arm now says so. `r` was a key that appeared to do nothing: `FallGuard`
stores consent and spends it only once the bike is back under **12 deg** and
settled for **0.2 s**, and nothing in the viewer said either thing. The mirror
now prints state transitions and acknowledges the keypress.

**"THE LINKAGE REGRESSED TO A DIFFERENT MECHANISM" WAS THE SERVO AT ~0 deg.**
Reported as `--swing-linkage` showing an older or wrong geometry; the config
was right (`_smaller`) and the compiled model is byte-identical to HEAD across
all seven build combinations. The servo was sitting near 0 instead of the 180
stow, i.e. -180 deg of crank travel -- OUTSIDE the +-136.6 deg stroke, but this
crank turns all the way round, so the loop closes there and what got drawn was
a real pose the mechanism can never be COMMANDED to. It reads as a different
machine because kinematically it is one. Driving the servo back to 180 fixed
it. The mirror now warns once per excursion, and still draws it: hiding it
would swap a confusing picture for a frozen one.

**THE MIRROR'S CHUGGING IS THE WIFI LINK. MEASURED 2026-09-17.** Not the
renderer, not the loop, not the four-bar. `--frame-stats` on the real mirror
reported the loop HEALTHY while the operator was watching it chug -- 60 fps
(n~300 per 5 s window), sync 0.10-0.17 ms, 10.7 ms of sleep headroom, and 0 of
300 frames over budget in the very window the stutter was seen. That is the
signature of a mirror redrawing the SAME pose: the loop cannot be at fault
because the loop is idle.

So the arrival pattern was measured instead -- 800 UDP packets at 100 Hz from
the Pi, telemetry-sized, **with the Pi otherwise IDLE (load 0.10, `run_bike`
not running)**:

| | |
|---|---|
| received | 793 of 800, 99.8/s -- looks fine on the average |
| gap p50 / p90 / p99 | 9.95 / 10.44 / 11.81 ms -- smooth most of the time |
| **gaps > 33 ms** | **6 in 8 s: five of 100-150 ms, one of 552 ms** |
| link | **-74 dBm**, rate adapted 130 -> 65 Mbit/s, 36 tx failures |

A 100-150 ms gap is 6-9 HELD FRAMES at 60 fps, several times a second-ish.
The average hides it completely, which is why "95.5 Hz telemetry, zero loss"
from the earlier bench session was true and useless for this question: it
measured throughput, and the thing the eye sees is the tail.

**BUT THE STALLS DID NOT REPEAT, SO THEIR CAUSE IS OPEN.** Re-measured later
the same day, same probe, same band, bike re-powered:

| `ATA-5G` | signal | gap p99 | max | gaps >33 ms |
|---|---|---|---|---|
| the run above | -74 dBm | 11.8 ms | 552 ms | **6 in 8 s** |
| 30 s | -66 | 11.4 | 23 | **0** |
| 60 s | -70 | 10.8 | 24 | **0** |

-70 dBm was clean for a full minute, so weak signal alone does not explain
them. Not yet ruled out: macOS AWDL (AirDrop/Continuity takes the laptop's
radio off-channel periodically -- the laptop end, which nothing here has
measured), background scans on either end, other traffic on the router. **The
link is still where the chugging came from; WHY the link stalled is not
known.** Next time it chugs, run the probe then, and check AWDL
(`ifconfig awdl0`) on the laptop. AWDL was `active` during all three clean
runs, so it does not stall the link on its own; it may still be a factor.

**THE BIKE IS NOW ON `ATA` (2.4 GHz)**, `autoconnect-priority` 10, with
`ATA-5G` kept as the fallback. Same IP (192.168.0.117); the laptop stays on
`ATA-5G` and reaches it, same router. 60 s probe on `ATA`: -54 dBm, 5862/5862,
gap p99 13.6 ms, max 25 ms, **0 gaps >33 ms** -- as clean as `ATA-5G` was the
same day, so this proves margin, not a cure. **The operator then ran the
mirror on it and saw no chugging.** The between-sitting difference is most
likely DOORS: laptop and bike were in the same spots both times, and which
doors were open is what changed (operator's observation). 2.4 GHz goes through
them better, which is the reason to stay on it. Getting there took the operator's
passphrase; three things found on the way, all in `untethered-setup.md`:

  * The "13 dB better" was ONE scan. Six repeat scans later: `ATA-5G` -63/-64,
    `ATA` -58/-60, `ATA_EXT` -51/-52 -- about 5 dB. Stable within a sitting,
    different between sittings.
  * The Imager stores the wifi key as the 64-hex PSK, which is salted with the
    SSID, so `ATA-5G`'s stored key cannot be reused for `ATA` even when the
    passphrase is the same. Copying it failed at the 4-way handshake; the
    operator then typed it at a hidden prompt.
  * The fallback works: the failed attempt dropped ssh for ~20 s and the bike
    returned to `ATA-5G` on its own. And a NEW `nmcli` profile persists (it is
    a keyfile in `/etc/NetworkManager/system-connections/`); only EDITS to the
    netplan-generated `netplan-*` profiles are lost on `netplan apply`. An
    earlier version of this file said all nmcli changes were lost.

**DECIDED: A ROUTER, NOT AN ACCESS POINT ON EITHER END.** House wifi for now;
if trouble comes back, move the router or use a different one. The laptop and
Pi APs are both out -- each end has one radio, so either leaves the laptop
(and any Claude session on it) offline, there is no cell service at the bench
to tether from, and hosting an AP on the M4 is awkward (legacy CLI routes). A
USB wifi adapter on the Mac would not rescue it either: macOS on Apple Silicon
has essentially no drivers for them (general knowledge, not tested here).
Options table in `untethered-setup.md`. Consequence worth having: **the bike
keeps internet**, so its clock stays NTP-set and offline timestamps are not a
problem to solve yet.

**WHICH BOARD SHIPS IS OPEN.** The Zero 2 W is probably not happening; the end
state is a custom or different SBC with an unknown radio, so every link number
here is provisional against a radio nobody has chosen, the 95.5 Hz one
included.

**`iw`, NOT `nmcli`, FOR SCANS.** NetworkManager rate-limits rescans, and
three `nmcli dev wifi rescan` calls returned its cache, which is where "there
is no 2.4 GHz network" came from. `sudo iw dev wlan0 scan` is the
measurement.

**PHASE 2 IS UP (2026-09-17): every servo's health in the packet, and every
tick recorded on the bike.** Run on the Pi against the real bus, torque off.

  * **Read block** +8 bytes a servo (`HEALTH_BLOCK`, `hw/dynamixel.py`): PWM,
    effort, input voltage, temperature, Hardware Error Status. 22 of block 1's
    28 indirect entries. "effort" is ONE slot with two meanings -- Present
    Load (fraction of max torque) on the XC430 drives and Present Current (A)
    on the XC330s, same address 126 -- decoded per servo by its own register,
    and keyed `load` / `A` in the packet so the two never share a column.
  * **Cost of the read**, 2000 reads x 4 alternating runs: mean +0.01 ms
    (1.92 -> 1.93), p99 +0.4-0.9 ms (1.98 -> 2.37-2.89). A 10 ms tick.
  * **Packet** 474 -> ~790 B with four servos (`servos`, and `run`, the record
    it is being written into). 96 packets/s delivered.
  * **The mirror shows it** as a text readout (`telemetry.status_text`):
    state, pack, jitter, QoS, record name, and per servo temperature, effort
    and duty. A hardware error replaces that servo's line and is printed to
    the terminal once.
  * **The onboard record** (`bench_log.RunRecorder`): on by default,
    `traces/bike/<stamp>_run[_tag]/` on the Pi, loads as an ordinary bench
    `Capture` -- raw registers decoded from its own meta, what was written to
    each servo, and 24 columns of what the controller had (`BIKE_COLS` in
    `run_bike.py`). meta carries the whole `bike_params`, the policy and both
    digests. ~300 KB per 20 s compressed. `--no-record`, `--tag`.

**THE RECORDER'S FIRST DESIGN STALLED THE LOOP, measured and replaced.** A
writer thread handed 1000-row chunks to `np.savez`; at the hand-off the loop
went 31 ticks late, worst 24 ms, starting at exactly tick 1000. The save is
12 ms on its own -- in a thread it fights the control loop for the GIL for
~0.4 s. Now each tick appends one 585 B row to a buffered file on the loop's
own thread (0.105 ms mean, 0.138 p99 on the Pi), and `close()` converts it.

| 30 s, link up, torque off | tick jitter p99 | max |
|---|---|---|
| recording off | 0.94 ms | 2.10 ms |
| recording on | 1.02 ms | 4.30 ms |

A kill or brownout loses at most the unflushed ~1 s; `Capture.load` reads the
partial `rows.bin` and meta says `complete: false`. Pull records with
`rsync -a efun@aowbike.local:'~/aow-bike-sim/traces/bike/' traces/bike/`.

**THE CONTROLLER RAN AT HALF SPEED ON THE BIKE -- fixed 2026-09-18, before
any torque-on run.** `control.rate_hz` is 200 (the simulator's rate, what
the LQR was designed at) and DriveController took its `dt` from it; the bike
ticks at 100 Hz. So on hardware the policy was queried at **25 Hz instead of
the 50 it trained at**, the steer target integrated half the commanded rate,
and the velocity low-pass had twice its time constant. Found by replaying a
bench record through the controller: the policy asked for -458 deg/s and the
target moved at -229. `run_bike` now builds its controller at the loop's
own rate (`controller_params`) and calls `tick()`, which computes every call
-- a 10 ms schedule would skip the ~10% of ticks the 1 ms servo clock reads
as 9 ms (measured 9/10/11 ms: 305/2384/300 of 3000). A test pins 50 policy
queries per second and the full integral. Cost on the Pi: `policy.action`
0.39 ms; tick jitter p99 0.86 and 0.89 ms in two 20 s runs. One unexplained
cluster (9 ticks, worst 15.4 ms, ~1 s into one run) did not repeat.

**FIRST TORQUE-ON BENCH RUN (2026-09-18, 93 s): drive A tripped its own
overload protection** while the operator held the wheel -- 100 % PWM at
80-98 % load for 3.5 s in total, 44 C, latched error 32 at 88.6 s. Three
things were wrong, all fixed and tested, the reboot verified live:

  * the policy stayed engaged and drove the dead servo for 5 s. A latched
    error on a drive or the steer is now a CUT that holds until it clears
    (the righting servo's is announced only);
  * the run then DIED on the cut's torque-off: the reply's error byte was
    128, the protocol's alert bit ("a hardware error is latched"), which
    says nothing about the instruction -- it had worked. Eight status checks
    treated any non-zero byte as failure; now only bits 0-6 do;
  * a latched error persists until reboot, and would have failed start-up
    too. `ServoBus.open` now reboots any latched servo before configuring it
    and says so; restarting run_bike is the recovery. Verified: id 101 32 ->
    0. That first run after the reboot then failed preflight on something
    unrecorded (the next run passed) -- open.

**THE HEADING COMMAND STARTED FROM THE AHRS'S ZERO, not from the bike.**
Reported by the operator as "the zero point is odd". The station's heading
started at 0.0 -- the TM151's own yaw zero, an arbitrary direction -- so
every connect commanded a turn: measured +45 to +66 deg of lead at all four
connects of the 2026-09-18 sessions, outside the +-35 deg clamp band before a
key was pressed. And the `psi` the bike reported was the CONTROLLER's, frozen
through every cut (101 deg stale in one session), which the station re-aims
and clamps against. Now: the bike reports its measured yaw; the station sends
NO heading until it has heard one (the bike keeps its own), and while the bike
is not engaged the command follows it, so a re-arm starts aligned. Verified
live: 0.0 deg of lead at connect and at reconnect.

**SCHED_FIFO IS NOW GRANTED ON THE PI** (2026-09-18): `ulimit -r` was 0, so
run_bike's real-time request was refused with a warning every start. Set by
the operator in `/etc/security/limits.d/90-aow-rtprio.conf` (`efun - rtprio
80`; takes effect at the next login). A system setting on the Pi, not in
this repo -- a fresh SD card needs it again. It CONTRADICTS a 09-16
measurement (FIFO made the loop worse by starving the AHRS reader) that did
not reproduce: 0.00% of ticks reused an IMU sample, with or without FIFO. See
the dated note in `pi-bench-bringup.md`. One run each, so indicative:

| tick lateness | median | p99 | worst |
|---|---|---|---|
| before, two 20 s runs | 0.09 ms | 0.86-0.90 ms | 1.6-3.6 ms |
| after, 142 s, mirror connected | 0.03 ms | 0.76 ms | 1.12 ms |

**THE LOW-VOLTAGE CUTOFF IS REBUILT** (2026-09-18; each threshold seen to fire on the brick).
It read drive A alone as a raw 1 Hz sample and ENDED the process at 10.2 V.
But each servo reads the rail at its own connector: on the brick all four agree
to 0.1 V at rest, and with the drives loaded the spread is 0.2-0.3 V typical
and 0.7 V worst, drive A lowest. On a pack, that would have tripped a
mid-move sag. Now (`PackMonitor`): the HIGHEST servo reading, low-passed every
tick (tau 2 s). It warns below 10.5 V and CUTS AND HOLDS below 9.9 V, like a
servo fault, so the station is told why. Both latch; a flat pack means a
full power cycle, and the HELD row says POWER OFF AND SWAP BATTERY. It won't arm a pack already below the cut. The servos' own voltage
limit is no backstop as set: Min Voltage Limit 6.0 V on all four, and Shutdown
(52 = overheat, shock, overload) does not include a voltage error. Replayed over the three torque-on
records, the filtered value bottoms at 11.68 V against drive A's raw 11.0.
On the brick (~12.0 V), `--pack-warn 12.5` shows the warning and adding
`--pack-cut 12.2` shows the cut. Both fired on the first tick after contact.
**A HOLD WAS SILENT to a station that arrived later.** The message rides
EventLog's 5 s, so a reconnect showed only "cut", was told "r once upright",
and `r` did nothing. Now `hold` is a telemetry LEVEL: a HELD row in the
mirror and on the terminal status line. The reconnect names the reason, and
a refused `r` is answered. This covers pack, servo fault and link holds. The
pack WARNING had the same flaw (gone after 5 s) and is a `warn` level, a
WARN row, until the cut replaces it with HELD.
**Outstanding:** a pack has never been on the bike.

**THE BENCH WINDING IS THE ATTITUDE, as the operator suspected.** Replaying
the torque-off bench record: with the recorded +3.4 deg roll the policy asks
for its full -458 deg/s steer rate on 100% of queries; the same record with
roll and pitch levelled splits 51/49 in sign. A lean the bike cannot correct
(no wheels on the floor, and torque off) is steered into forever. With torque
ON on the bench the wheel WILL follow, so expect the steer to spin
continuously at up to 458 deg/s until the mount is calibrated or the bike is
on the floor -- the lead bound caps the lead, not the rotation.

**THE BIKE'S SESSION, REWORKED 2026-09-18 -- including two ways it could
energise a bike it should not have.** All checked live on the Pi.

  * **`r` ENERGISED A `--no-torque` RUN.** A re-arm called `bus.arm()`
    unconditionally. The operator confirms it happened on the bench. Now
    gated on `--torque`, and a test pins it.
  * **A SIGNAL'S SHUTDOWN COULD NOT TURN TORQUE OFF.** SIGTERM and SIGHUP
    (`kill`, `timeout`, a dropped `ssh -t`) used to skip `shutdown()`
    entirely. Once they were routed through it, the torque-off came back
    `COMM_PORT_BUSY` (-1000): the exception had landed mid-transaction and
    left the SDK port marked busy. Now a signal inside the loop only requests
    the stop, honoured between ticks, and `ServoBus.close()` clears the flag
    and tries every servo (it used to stop at the first failure). SIGTERM,
    SIGHUP and ctrl-C each verified clean on the Pi.
  * **A second `run_bike` dropped the first one's bike.** Opening the bus
    turns torque off on every servo, and that happened before the second
    instance found port 9910 taken. The port is now bound first, as the
    single-instance lock; verified, the second exits without touching the bus.
  * **A dead link no longer ends the run.** Cut (policy servos limp, the
    righting servo holds), keep sensing and recording, wait; a returning
    station re-arms with `r`. Nothing re-arms while the link is down, not
    even `--auto-rearm`, and the settle dwell restarts on reconnect. The
    start-up wait is now forever by default (`--link-wait S` to bound it).
  * **A re-arm stalled the loop 68 ms** reloading the policy from disk, so the
    first ticks after a fall ran late. `run_bike` now reuses the loaded
    policy (stateless); after the fix, re-arm max jitter 2.69 ms.
  * **The mirror sent 30 commands/s, not 50** (measured from `link_age`): a
    20 ms gate against 16.7 ms frames fires every other frame. Now every
    frame, capped at 100 Hz, and still tied to rendering on purpose.
  * **`r` is a COUNT now, not a one-packet flag.** `rearm: true` rode in the
    single packet after the press, so one lost datagram ate it. The station
    sends `rearm_n` in every packet and the bike acts on a change -- except
    from a station it has not heard before, or while the link is down (a
    press made blind is not consent). Verified live across a link drop.
  * **The steer target is bounded: 45 deg of lead over the measured steer**
    (`control/steer.py`, `advance_target`), in training AND on the bike. The
    policy outputs a steer RATE, integrated into a position target with no
    bound -- torque off it ran 167 -> -4412 deg in 20 s; now it stops at -45.
    Invisible while the wheel follows: the steer actuator is already at its
    0.8 N.m limit above ~39 deg of lead at the fastest steer seen in sim, and a
    20 s mixed-command sim run is bit-identical with and without it (peak lead
    16.3 deg). A steer held by a "rock" for 0.3 s or more drops the bike either
    way; the bound halves the unwind rate (42 -> 22 rad/s at 0.3 s, 56 -> 9 at
    1 s). Existing policies need no retraining. A servo in Velocity Control
    Mode was rejected: it would drop the position hold the policies trained
    against, which is a different plant.
    On the REAL servo (P-only, Position P 900, no profile, PWM limit 885,
    read off servo 103), MEASURED on the first torque-on bench run: PWM
    rises ~3.3 %/deg of lead -- 37 % at 10-12 deg, 74 % at 20-30 deg -- and
    never saturated; the largest lead was 24.8 deg with the wheel turning at
    ~5 rad/s. Extrapolated, full PWM near ~30 deg: close to the sim's 23, so
    45 deg is very likely past saturation on hardware too, but that is not
    yet observed. An earlier note here predicted 11 deg from an e-manual
    formula quoted from memory; the measurement refutes it. (Lead is target
    minus the reading taken before the write, so it carries a few degrees of
    timing bias at speed.)
  * **The command is 12 packed bytes** (`telemetry.encode_command`), was
    76-150 B of JSON: version, rearm count, velocity x/y [mm/s], heading
    WRAPPED [1e-4 rad] (the bike only uses wrap_pi(psi_cmd - psi)), righting
    goal [1e-3 rad] and current, with -32768 = "not commanded". A JSON
    station is refused by name, once. Telemetry stays JSON for now.
  * **The bike tells the station what happened** (`telemetry.EventLog`):
    cut, re-arm, link lost/back, hardware error set/cleared, failsafe --
    decided and worded on the bike, each message riding every packet for
    5 s, printed once by the mirror and the terminal station and shown on the
    readout. The mirror's own state-change message is gone. Quiet cost ~11 B
    a packet; 791-972 B measured with messages in flight.
  * `--bench` = `--no-torque --allow-guess-mount`; `--bench --torque`
    energises. The steer zero stays the config's; `--steer-zero capture`
    is still there for a bare shaft.

The terminal station (`hw/ground.py`) is the fallback, not the main station,
and its docstring now says why its local mode is bench-only: run on the Pi,
the heartbeat comes from the bike itself, so a stalled wifi does not trip the
link watchdog.

`--frame-stats` now also prints LINK stats in `--mirror`: rx/s, inter-arrival
gap p50/p99/max, the fraction of frames that redrew a stale pose, and packet
age at draw time. Frame timing structurally cannot see this failure, which is
the lesson -- the instrument measured the loop, and the fault was in the data.

**AND THE VIEWER LOCK, FIXED ALONG THE WAY.**
Reported as chugging in `--mirror` but not in teleop, which pointed at the one
thing the two loops differ in that touches the render thread. It was a real
waste and not the reported fault -- sync now measures 0.10-0.17 ms, but it was
never the cause. The mirror's `apply_camera` took `viewer.lock()`
UNCONDITIONALLY every frame to compare one float, so every mirror frame took
the render mutex TWICE (there and inside `viewer.sync()`) where teleop takes
it once. Teleop's camera defaults to `free`, whose `apply_camera` returns
before any lock -- hence the asymmetry.

That mutex is held by the render thread WHILE IT RENDERS, so taking it is a
wait, not a few instructions. The mirror's camera is fixed-azimuth tracking
set once in `on_start` and only zoom ever changes it, so the lock bought
nothing. Now behind a dirty flag set by `-`/`=`. Pinned by a test that counts
acquisitions: **30 in 30 idle frames before, 0 after**, verified to fail when
the guard is removed.

Teleop's follow/overhead/wheel cameras DO lock every frame and have to -- they
recompute azimuth from the bike's yaw. So the prediction is that teleop in
follow mode chugs the same way; untested.

**AND THE COMPUTE WAS NEVER THE PROBLEM.** Measured the real teleop
closure -- policy, odometry, AHRS model, physics, overlay, four-bar -- over
3000 frames at 42 physics steps each:

| | ms |
|---|---|
| step (42x, policy included) | 2.02 mean, 2.37 p99 |
| draw | 0.83 mean, 0.96 p99 |
| whole frame | 2.85 mean, **3.22 p99**, budget 16.67 |
| frames over budget | **1 of 3000, and it is frame 0** |

The model is light to render too: 2 meshes, 708 vertices, 39 geoms. Scene
geoms cap at 201. What is left is `viewer.sync()` and the OS compositor, which
cannot be measured from a machine without the window on it -- so rather than
guess a third time, `--frame-stats [SECONDS]` now reports step / draw / SYNC /
sleep live in both loops. Sync is the column a headless profile cannot
produce; a healthy loop shows a large sleep. Each line covers only the frames
since the last, so an intermittent spike lands in the line it happened in
rather than being averaged away.

**A FALSE ALARM WORTH KILLING, NOT YET FIXED.** Every teleop run prints
"this policy is an artifact of a DIFFERENT machine". It is wrong: teleop
injects `floor_tilt_steps` into `params["sim"]` for the spawn dial, which
moves the IN-MEMORY plant digest eda849e7afaaca0f -> 044e2677741b0b33. The
comment there says "so neither digest moves", meaning the file. The policy
check hashes the mutated copy. Left alone deliberately -- digest behaviour is
not something to change in passing -- but it is training the reader to ignore
the one warning that would matter.

Suite **20 failed, 414 passed, 9 skipped -- red set unchanged**, 14 new tests.
Neither digest moves on disk (the new keys are under `control`).

**THE DRIVE SERVOS ARE MIRRORED AND THE FLIGHT CODE DID NOT KNOW (2026-09-16).**
`servo_sign_turning_hub_forward: [1, -1]` was measured on 2026-09-13 -- both
horns face outboard -- and written into
`docs/measurements/drivetrain-measurements.yaml`. Nothing under `src/` ever
read it. `analysis/drivetrain_bench.py` defaults to `--signs 1 -1` and has
always been right; `hw/dynamixel.py` and `hw/odometry.py` assumed [1, 1].

That swaps the two modes outright. A commanded common mode reaches the
hardware as a differential -- the bike crabs when told to drive -- and a real
forward roll reads as pure roller motion with `v_lon` near zero.

Caught on the bench by rolling the rear wheel forward by hand and reading the
two servos: **+3048 and -3048 counts**, `turned_a +13.962` against
`turned_b -13.958`, giving hub 0.002 rad. Signed, the same numbers give **hub
13.96 rad = 2.22 turns** and ring 0.002 -- a forward roll with the rollers
still, which is what the hand did.

`control.onboard.servo_sign` now carries it and `ServoBus` is the only
consumer, applied once on read and once on write. It is the boundary between
the servo frame and the input-shaft frame, and everything upstream -- the
estimator's hub mix, the ctrl vector, the model's joints -- is written in the
input-shaft frame and must not know servos exist. Moves NEITHER digest: the
simulator has no servos, so no trained policy can see it.

**EVERY TORQUE-ON RUN BEFORE THIS IS SUSPECT**, including 2026-09-16's. The
commanded `+32.6 / -7.4 rad/s` reached the hardware with B inverted, so what
the bike physically did is not what the telemetry said. Nothing was damaged --
the bike was held -- but do not read those numbers as a controller result.

A measurement that exists, is correct, is written down, and has no consumer is
the failure mode this repo keeps finding. Worth a grep before trusting that a
recorded fact is in force.

**`pytest -m hardware` passed 7/7 on 2026-09-15** — four servos on the Mac,
ids 101-104 at 3 Mbps. It failed 1/7 first (63 frames/s at a requested 200 Hz);
`adjust-ftdi-latency` fixed it, and a direct measurement then gave a 4-servo
FastSyncRead at **2.000 ms mean / 2.034 p99**, i.e. a 500 Hz read-only ceiling
and a tick that is entirely the FTDI latency timer. A SyncWrite is 25 µs and a
SECOND one is free: read + one write and read + two writes are both 2.000 ms.
See `pi-bench-bringup.md` §2.3.

Read the verdict line, not the FAILED count. The 23:

| file | red | what it is |
|---|---|---|
| `test_drive.py` | 15 | 7 trajopt (re-authored once the as-built mass is known) + 8 analytic LQR |
| `test_hw_odometry.py` | 7 | estimator quality, no falls — see `odometry-rewrite.md` |
| `test_teleop.py` | 1 | analytic LQR, reaches v_max then tips |

A bare `pytest` is SERIAL and takes ~145 s. Run the marker, not the suite —
`pytest -m pure` is 0.4 s and is the edit loop. `pytest --markers` is the
reference for which marker covers what.

**LQR: marginally functional, and that is the intended state.** Holds the bike
at standstill at 1.17° peak roll over 40 s. Recovered by two weights after the
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

**The three ground contacts are separately addressable (2026-09-09).**
`roller`, `front_tire` and `righting` can each carry their own `solref` and
friction via an optional `sim.contact_parts` block. `geom_priority` is the
mechanism — `solref` combines by solmix-weighted average and `friction` by
elementwise max, so separate values alone would give the mean, never the softer
part. Landed as a **bit-exact no-op**, verified over a 3000-step driven rollout:

| | max abs Δqpos vs pre-split | bitwise identical |
|---|---|---|
| split, identical params | `0.000e+00` | **yes** |
| split, `front_tire` overridden | `1.164e-01` | no |

`tests/test_contact_parts.py`, 9 tests under the `contact` marker, pins both —
the no-op *and* that an override bites, because a bug dropping `contact_parts`
would otherwise pass as a perfect no-op.

**Digests, re-verified 2026-09-22 — `deploy/bundle.npz` re-exported for the
10-state LQR (gains 9 x 2 x 10, plus `K0_legacy`) and matches all three
(`plant_digest`, `design_digest`, and the legacy whole-file `params_digest`
c70acbea4b2ba655):**

    plant_digest   8d8b25a809ea2f1a    was this trained against the machine I am running?
    design_digest  a973a9ca3d503b6d    were these gains designed against the weights I am running?

**`plant_digest` MOVED on 2026-09-16** (was `e1ec36bfa670217e`), and everything
in `moves/` is now an artifact of a different machine — `load_move` says so on
load. What moved it: `actuators.steer_kv` 0.05 -> 0.0676, the XC330's own
back-EMF droop `stall_torque/no_load_speed`, replacing a number that was a
GUESS in spirit; and `righting.{arm,wings}.servo_kp/kv` from tuning knobs
(30.0 / 1.0) to the servo's firmware gains referred to its shaft (1.62 /
0.0519). The bundle was re-exported and the LQR redesigned: worst fit 0.9412 ->
0.9489, gains moved 21% of scale on the drive input and 27% on the steer input
at the worst speed (v = -0.50).

**ACCEPTED, not outstanding, for the policies.** They were all trained at the
old value and are provisional in the digest's sense, but the change makes the
simulator MORE like the bike, not less — so retraining is the fix and reverting
is not. `general_rl_cmd_curriculum2b` has not been re-evaluated on the new
plant; do that with `analysis/per_command.py` before reading anything into a
hardware run.

`deploy/bundle.npz` matches both. A failing digest check is the mechanism
working — fix it with `python -m aow_sim.export_deploy`, never by loosening the
check. See `params-digest-split.md`.

---

## What drives the bike

`control.general_move` names **`general_rl_cmd_curriculum2b`** (repointed
2026-09-14). That is the command-family curriculum line: sensor-trained like
`odo_ahrs`, plus pitch, and the config the 12-seed sweep ran.
`seed-sweep-and-personalities.md` §8 has `personality1`, bit-identical to it,
as the best of the twelve on behaviour, while `_score` ranks it 8th. Teleop's
`--general` now defaults to the pointer instead of a hardcoded
`general_rl_cmd_curriculum2`. The pointer was chosen on 09-11 and never landed:
the digest worry that stopped it was about the legacy whole-file digest, and a
pointer edit moves neither `plant_digest` nor `design_digest`.

**Six policies trained ON the detailed drivetrain (2026-09-14), not pointed
at.** `general_rl_drivetrain_p{100,400}_{0,1,2}` are curriculum2's config at
firmware Velocity P 100 or P 400. Score on the eval grid:

| policy | ideal | drivetrain P 100 | drivetrain P 400 |
|---|---|---|---|
| `curriculum2b` (the pointer) | 0.595 | 0.638 | 0.177 |
| P 100 seeds | 0.36–0.48 | **0.741–0.751** | 0.180–0.503 |
| P 400 seeds | 0.05–0.14 | 0.42–0.54 | **0.625–0.803** |

Each arm beats the pointer on its own gain, and every policy is specific to the
plant it trained on, the pointer included. **The pointer stays because the
drivetrain model is not yet confirmed against the built bike**, not because
these lose. P 400 buzzes the rear wheel ~3× harder at 8–32 Hz (`chatter.py
--plant`). Teleop builds a policy's own drivetrain from its record
(`--general general_rl_drivetrain_p100_1`). Reproduce the table with
`analysis/drivetrain_eval.py --variants ideal full full_p400`.

The table below predates the curriculum line and is kept for the sensor
argument it makes. The split that matters is not one good policy against the rest — it is
**trained-against-the-sensors against not**. On the eval grid at
`--encoder counts --ahrs tm151`, score / survival:

| policy | trained against | tau 2.0 | tau 0.19 (measured) |
|---|---|---|---|
| `smooth_diff_pi` | MuJoCo truth | 0.110 / **0.20** | — |
| `odo_ahrs` (default until 09-14) | estimate + attitude, tau 2.0 | **0.672** / 1.00 | 0.570 / 0.95 |
| `odo_ahrs_rand2` | + attitude randomised, narrow | 0.654 / 1.00 | **0.663** / **1.00** |

Every truth-trained policy falls in four episodes out of five.

**Two pointer questions from before the curriculum line** (superseded as
pointer candidates by it, kept as findings):

1. **`rand2` vs `odo_ahrs`.** `rand2` is better at the tau the bike actually has
   and is nearly tau-invariant (spread 0.009 against 0.102). `odo_ahrs` keeps a
   tighter worst-case heading excursion once settled and much lower chatter. The
   case for keeping the current pointer is weaker than it looked.
2. **`glide_pitch_hub3` is a separate line entirely** — the calm one. Rim travel
   over a 15 s hold 7.58 → 3.35 m, airborne 58% → 13%, peak contact force
   7.23 → 3.66× weight, kick recovery 8/8 through dv 0.35. It has never been
   merged with the sensor line.

Neither is decided. Both are cheap to try — **repointing `control.general_move`
moves NEITHER digest.** Full standings and the whole sensor arc:
`sensor-workstream.md`.

**Caveats that keep biting.** Every row above was measured over 15 s episodes;
an eval now runs 5 s, so nothing here is comparable to a number measured after
2026-08-30. And every row is `--encoder counts` — a grid that does not force the
encoder scores older policies on a different bike, not a better result.

---

## The parameters that are still guesses

18 marked `source: GUESS` in `config/bike_params.yaml`, each with a note on how
to identify it. Never quietly promote one. The load-bearing ones:

| parameter | value | dies at |
|---|---|---|
| `chassis.mass` | 0.45 kg — **44% of the bike** | stage 2, the frame |
| `contact_solimp` | MuJoCo stock | contact calibration (`dmin` confounds both bench tests) |
| `friction_sliding` | 0.9 | the incline slide, P0b — cheapest test in the project |
| `input_armature`, hub/roller damping + frictionloss | 5 of them | the drivetrain station, spin-downs. Fitted 2026-09-14 into the OPT-IN overlay only (armature 2.44e-4 at the input shaft; Coulomb 0.05 N.m there, ~60x the hub guess) — not promoted in `bike_params.yaml` |
| `payload.pack.mass` / `electronics.mass` | 0.115 / 0.076 kg | **a scale, today** |
| `min_pinion_radius` | 0.006 m | print a test pinion |
| `righting_sign` | +1 | mounting the motor -- a build DECISION, not a measurement. Station-only (the render); nothing flies on it |

**The contact model as it currently ships**, since it comes up often:

    contact_solref: [0.005, 1.0]     # positive convention: (timeconst_s, dampratio)
    contact_solimp: [0.9, 0.95, 0.001, 0.5, 2.0]   # MuJoCo stock, GUESS
    friction_sliding: 0.9   friction_torsional: 0.005

~16× stiffer than MuJoCo's default. It ships **critically damped so it cannot
bounce**, which is known to be wrong — the real wheel bounced 2–3 times, implying
dampratio ~0.30. The plan is to measure, then **switch to the negative
`solref: [-stiffness, -damping]` form**, which is what MuJoCo recommends for
system ID and is the only thing that decouples the two numbers: in the positive
form `timeconst` sets damping *and* stiffness while `dampratio` sets stiffness
only, so a static reading fixes only the product. Under a negative pair the
`dmin` confound shrinks from 2.3× to 6%. The derivation, the prediction tables
and the procedure are in `measurements/contact-protocol.md`, which is **correct
and current** — fixed 2026-08-22, re-checked 09-08. What is missing is data:
every field in `contact-measurements.yaml` is still 0.0.

---

## Risks, ranked

1. **No trained policy has ever seen contact-stiffness variation.** The
   randomization axis exists but sits commented out in `config/rl_general.yaml`.
   **Most likely single cause of a policy that works in sim and not on the
   floor.** It depends on no measurement, which is the point — so it is the one
   thing a long run grid should do before the bike exists.

   **Re-measured 2026-09-11, `analysis/floor_sweep.py`, `general_rl_odo_ahrs`,
   score / survival over the 20-command grid, mu 0.9. The 09-09 column is kept
   beside it because it was acted on and because the two disagree in ways that
   change the conclusions — it was taken with NO AHRS in the observation, which
   `policy_env_overrides` could not supply until 09-11 (see
   `docs/plans/sensor-workstream.md`). The right-hand column is the policy
   against the attitude error model it actually trained on, tm151 at its own
   declared tau of 2.0:**

   | sink @ bike weight | `contact_solref` | 09-09, no AHRS | **09-11, tm151** |
   |---|---|---|---|
   | 13.17 mm | `[0.020, 2.00]` | 0.594 / 1.00 | 0.600 / 1.00 |
   | 3.80 mm | `[0.020, 1.00]` | 0.657 / 1.00 | 0.660 / 1.00 |
   | 1.08 mm | `[0.005, 2.00]` | 0.673 / 1.00 | 0.679 / 1.00 |
   | 0.52 mm | `[0.020, 0.30]` | **0.719** / 1.00 | 0.651 / 0.95 |
   | 0.11 mm | `[0.005, 0.50]` | 0.719 / 1.00 | 0.616 / 0.90 |
   | **0.39 mm** | **`[0.005, 1.00]` — ships** | 0.663 / 1.00 | **0.686** / 1.00 |
   | 0.04 mm | `[0.005, 0.30]` | 0.374 / **0.70** | **0.175** / **0.35** |

   **Four rows move by under 0.03 — inside the ±0.02 seed-noise floor — and two
   move a lot, so the ordering did NOT survive the correction.** The stiff-end
   cliff is twice as deep as published (0.374 / 0.70 → 0.175 / 0.35), and the
   two rows that tied for best at 0.719 both fall below the shipped contact and
   both start dropping episodes. The shipped `[0.005, 1.00]` is now the top row
   in the table rather than the fifth.

   **Rows are sorted softest first, and `solref` is labelled by SINK because the
   raw pair reads backwards.** `dampratio` appears only in the stiffness term,
   as `1/dampratio^2`, so dropping it 1.0 → 0.30 makes the contact ~10× STIFFER,
   not softer. `timeconst` is the honest softness knob — it sets damping and
   stiffness together, sink going as `timeconst^2`.

   What it says:

   - **Soft contacts are fine.** Survival is 1.00 from 13 mm of sink all the way
     down to the shipped 0.39 mm, and score varies by 0.09 across a 34× span.
     Unchanged by the correction.
   - **The cliff is at the STIFF end**, and only there: `[0.005, 0.30]` —
     0.04 mm of sink — drops survival to **0.35**, not the 0.70 first
     published. That is the corner the drop test points at, because a stiff
     contact against unchanged damping is what bounces, and it is a worse
     corner than this section said for two days.
   - **Sink alone does not determine the outcome; damping matters
     independently.** This survives, but the evidence for it inverted. It used
     to be that two rows TIED FOR BEST at 0.719 across a 5× sink spread
     (`[0.005, 0.50]` and `[0.020, 0.30]`); corrected, those two rows sit at
     0.616 / 0.90 and 0.651 / 0.95 while the shipped contact — between them in
     sink — leads at 0.686 / 1.00. Non-monotonic in sink either way, which is
     why the negative `(-stiffness, -damping)` form is the one to measure into.

   Read the table as *how the current policy copes*, not as what is achievable:
   nothing has ever trained over this axis. A run with `dampratio_range`
   uncommented is the experiment that would flatten it.

   **Friction is NOT the flat axis for survival — that claim was an artifact of
   the same missing AHRS.** Re-measured 2026-09-11 across mu 0.5–2.0 at the
   shipped contact, whole-grid score moves 0.686 → 0.686 → **0.549** and
   survival drops to **0.90 at mu 2.0**. The 09-09 reading was 0.630 → 0.663 →
   0.643 at survival 1.00 throughout, which is where "the flat axis" came from.
   What DOES survive is the effect it was quoted for: the `hold` family's drift
   still rises monotonically, 0.754 → 1.242 → 1.815 m, i.e. **more grip means
   more wander, not less**. The
   policy uses slip as a brake, so a grippier floor converts more of its sawing
   into travel. The whole-grid score cannot see this: `hold` is 1 command of 20
   through a geometric mean. Use `--by-family`.

   **A CORRECTION, recorded because it was acted on.** An earlier version of this
   section read the dampratio axis backwards — it called `dampratio 0.30` a soft
   contact and concluded "drive on the hardest surface available; a foam mat is
   the failure mode." **That is inverted.** Soft is the safe direction here and
   stiff is the cliff. The error came from trusting the parameter's NAME over
   `contact-protocol.md`'s own CORRECTION, which states the formulas plainly.

2. **A third of training runs silently produce a policy that drives BACKWARDS
   when told forward** — measured 2026-09-10 across 12 seeds: 4 broken, 1
   partial, 7 competent, all differing only by `algo.seed`. `_score` is still
   `survive_rate × track_geo`, so three of the broken ones outrank the best
   policy in the set. Detectable from ~5M of 20M steps, but a naive abort would
   have killed the run that recovered to the second-best result. Nothing is
   implemented; see `seed-sweep-and-personalities.md` for the taxonomy, the
   reward mechanism behind it, and the options.
3. **The front tire and the righting wings still have the compliance of a TPU
   roller — now fixable in one line.** `roller`, `front_tire` and `righting` were
   made separately addressable on 2026-09-09 (`sim.contact_parts`, via
   `geom_priority`), shipped as a **bit-exact no-op**: all three still resolve to
   the same globals, so nothing moved and no export became provisional. The
   fidelity gap is unchanged until someone sets a value — a rigid printed wing
   striking the ground is still modelled as rubber. What changed is that it is
   now a one-line, deliberate edit that moves `plant_digest`, rather than a
   refactor. See `floors-and-the-contact-model.md` §4.
4. **Authority derating.** Real servos will not deliver modelled torque at
   modelled bandwidth. **Partly quantified 2026-09-14** on the fitted servo
   model (`drivetrain-model.md`): the ideal-drive policies measured hold at
   factory gains and lose most of the grid at P 400 (≤ 0.18), the gain the bench
   liked. That loss was the policy/plant mismatch: policies trained AT P 400 get
   it back (0.63–0.80), at the cost of a rear wheel that buzzes ~3× harder.
   The torque SCALE is still the datasheet 1.6 N.m. The bench replay cannot
   see it (fitted inertia and friction scale with it); the bike can. D5 or
   servo-strength randomisation covers it.
5. **Left/right asymmetry, and crab.** `turn_asym` has never gone below ~0.2 at
   any run length; crab is one-sided in every champion. The plant is
   mirror-symmetric and the handedness *flips sign between policies* —
   spontaneous symmetry breaking, not plant asymmetry.
6. **Steer homing at power-up is undesigned.** The XC330 loses its multi-turn
   count across a power cycle and the fix depends on an unmade mechanical choice.
   Blocks first power-on, not ordering.
7. **Front-wheel liftoff is undetected by the estimator.** Quantified —
   `analysis/liftoff.py` measures 79 mm of clearance in one arm.

*Retired 2026-09-08: "the contact protocol docs are stale." They were fixed on
08-22 and re-verified against `analysis/contact_calibration.py` on 09-08. The
risk was the claim, not the docs.*

---

## Explicitly not being worked on

The PD cascade (legacy, kept compiling as a manual-debug fallback). The trajopt
moves `flick` / `flick_fwd` / `flip` — 7 of the 23 red tests — re-authored once
the as-built mass is known, once rather than twice. Roll-phase and surface
contact studies. A live gamepad front-end. The ball shot. Re-tuning the LQR
weights, deferred until the contact model is pinned. The disturbance curriculum,
parked until the params files are reconciled.

**The wheel-only `--variant testbed` model does have a physical counterpart** —
it is stage 1 of `first-physical-test.md`. An earlier version of this file said
otherwise; that was written before the build order existed.
