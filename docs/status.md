# Project status — 2026-09-09

**How to read this.** The navigation layer: what is true now, what to do next,
and where the detail lives. Rewritten, not appended to. **For the reasoning,
follow the link — it is not repeated here.**

Rewritten 2026-10-02 (1798 lines -> ~300). Every dated block that was here
moved VERBATIM into the plan doc that owns it, under a heading saying so; the
old file is `git show d0dffeb:docs/status.md`. Where each went: "Where
everything is written down", below.

---

## Where the project is, in five lines

1. **The simulator is trusted for control design**: parametric MuJoCo, an
   omni-wheel contact, the bike's own sensors in the loop and in training,
   measured XC330 laws and headset friction. The contact is the least-measured
   part; bench pre-tests started 2026-10-01.
2. **RL drives.** `general_rl*` policies balance and drive from a live
   (velocity, heading) command. The analytic LQR is a reference baseline.
3. **The onboard path runs on the Pi bench** (09-15 to 09-18): 100 Hz loop,
   TM151 pushing from flash, mirror, telemetry, recorder, fall/cut/re-arm,
   low-voltage cut. Torque on only with the bike held. No pack yet.
4. **Hardware is printed module by module.** Built: servo bench, rear
   drivetrain station, the righting four-bar (09-17), the front steer
   (printed 09-30). Designed, not printed: the rear drive and the printed
   righting module. The whole bike is assembled in CAD (09-30).
5. **The goal is a functioning bike with sample videos, in weeks.** Ranked
   against that.

---

## What to do next

| # | do | why now | doc |
|---|---|---|---|
| 1 | **Contact calibration**: stiffen the force sensor's mount (resolder or glue) and check it with a rigid-block drop; then the drop rig with added mass at the wheel (front, then rear), a joint `solref` + `solimp` fit with the dial (P0) data, and the P0b incline slide | First rig run (10-03, front, 65 drops): the rig's load path reads ~7-30x softer than the dial says the wheel alone is, and the base rings at ~170-180 Hz -- probably the sensor mount, so fits so far describe the rig, not the tire. MuJoCo's contact is per unit mass (same bounce at any mass), so a fit reaches the bike only through each wheel's effective mass | `drop-release-rig.md`, `contact-protocol.md`, `analysis/drop_rig_sim.py` |
| 2 | **Print the rear drive and the righting module**, then the chassis | Held until the steer print settled the shared fits; the bushing and end-play results are in (09-30) | `drive-design.md`, `righting-design.md`, `bike-assembly-design.md` |
| 3 | **Finish the drivetrain station**: firmware Velocity P and I, the D5 torque-scale check, fit the five drivetrain `GUESS`es | The sim cannot pick the gain (policies tie at P 100 / P 400; P 400 buzzes ~3x harder). Decide on the bike | `drivetrain-model.md`, `drivetrain-measurements.yaml` |
| 4 | **Sim: the eval score's directional gate** | `_score` is still `survive_rate x track_geo`; a policy that abandons a direction outranks good ones. Risk #2 | `eval-score-rewrite.md` |
| 5 | **Pointer decision: `general_rl_smooth_temporal`** | Candidate since 10-01: ~19x less steer chatter, best score (0.820 vs the pointer's 0.572), passes the endurance bar. One seed | `policy-smoothness-losses.md` §5.1 |

Weighing waits for the as-built parts: the mass entries will be re-cut by
module (the Pi 3 is a stand-in, the printed modules are new), so weigh once
they exist. One printed part weighed sooner would calibrate the CAD estimates
(`cad_bike.printed_mass`).

Bench loose ends: the second XC330 is still id 103 and must become 104 before
the two share a bus.

Not next, deliberately: the ball shot, the privileged critic, the odometry
rewrite, the AHRS fixture (parked 09-24), the self-righting linkage choice
(parked 09-27; 625 counts rights the built one in teleop, 300 cannot).

---

## The workstreams

| workstream | state | blocker | doc |
|---|---|---|---|
| **Simulation & model** | Working; 18 `GUESS`es. Opt-in detailed drivetrain (fitted XC430 loop, detent, slop). XC330s from the bench (current law, load-proportional gearbox friction); the steer as its firmware runs it, 4 ms delay; headset friction since 09-30 | Contact and masses, unmeasured | `mujoco-modeling-decisions.md`, `drivetrain-model.md` |
| **Control — RL** | Primary. Pointer `general_rl_cmd_curriculum2b`; candidate `smooth_temporal`. Every export trained on the old front wheel: provisional | Crab one-sided; `turn_asym` ~0.2; the pointer falls standing on its own sensors (10 of 18); on the new front wheel it survives 4 of 6 big turns | `general-rl-improvements.md`, `policy-smoothness-losses.md` |
| **Sensor modelling** | Largely done. TM151 measured on the fixture: a complementary filter, tau 0.19 s at rest to ~1 s moving, ~0.2-0.3 deg RMS at one mount against the sim's 1.5. `tm151_filter` model exists, work in progress (survival 0.05 on the eval grid) | The bike's own AHRS log | `sensor-workstream.md`, `ahrs-fixture.md` |
| **Control — LQR** | Reference. New front wheel (10-05): stands on truth and on `tm151_filter` (with `q_steer_standstill` 50), falls standing on teleop's default `tm151` within ~10 s, falls turning at 0.8 m/s | The at-speed design; re-tune once contact moves | `lqr-baseline.md` |
| **Hardware / onboard** | Pi 3 bench proven: tick jitter p99 < 1 ms with four servos energised. 2.4 GHz house wifi (decided: a router, no AP). The SBC that ships: open | Chassis, pack | `pi-bench-bringup.md`, `untethered-setup.md` |
| **CAD** | Steer printed and print-checked; drive, righting, whole bike designed. CAD wheelbase 250 (the sim keeps 200) | Wheelbase and the righting stack to tighten (user) | `bike-assembly-design.md`, `cad-onshape-workflow.md` |
| **Self-righting** | Four-bar built. Linkage options parked (sim, 9.9 V: 537-575 counts vs 643 as built; the diamond, 547, is the lean) | Decision later | `righting-linkage-margin.md`, `righting-servo-model.md` |
| **Contact bench** | Drop rig built and run (10-03): the cam fires every drop and records each, release-to-impact timed; analysed at the contact through the arm (lever 205/124, m_eff 89 g fitted -- to re-derive from the weighed parts). MuJoCo twin `analysis/drop_rig_sim.py`, with `--fit` | The sensor mount's stiffness; the arm's parts weighed | `drop-release-rig.md`, `contact-measurements.yaml` |

---

## Health

**NEW FRONT WHEEL (2026-10-05).** The original bike's wheel, measured
(102.5 x 24 mm, 7 mm shoulders), its 10 mm flat tread modelled as an 80 mm
crown (`GUESS`; why: `mujoco-modeling-decisions.md`). Costs: the LQR needed
`q_steer_standstill` to stand and still falls turning at speed
(`lqr-baseline.md`); the pointer's eval score fell 0.572 -> 0.454, mostly big
turns (4 of 6 survive, was 6 of 6). **Outstanding:** every policy is
provisional until retrained on this wheel.

**THE DETAILED DRIVETRAIN IS THE DEFAULT PLANT (2026-10-02).** The overlay
moved, unchanged, from `config/drivetrain_model.yaml` into `bike_params.yaml`'s
`drivetrain_model:` block, which put `plant_digest` at 9953690d, exactly the
stamp of the four smoothness-loss exports; removing `min_pinion_radius` the
same day moved it on to **95630b212f03ffc4** with no physics change. `drivetrain_model.base_params` is the IDEAL drive: the LQR is
designed on it (`linearize.design_plant`, an ideal twin of whatever model it
is handed) and flown on the detailed one; the flick/pivot/ball envs and every
export without a `drivetrain_model:` record (the pointer included) run on it;
teleop compiles its startup policy's own plant. **A model built with the
block IS the ideal plant until `DrivetrainSim.attach` switches it** (native
drive zeroed, measured rotor inertia, rigid roller couplings off, slop tendons
on; `release` switches back, both pinned bit-exact by tests): so every loop
that never attaches the hook -- contact studies, settles, the other moves'
envs -- runs exactly as before, and the hooked ones (RL envs, teleop,
`balance.run`, `record`) get the detailed drive.
`drivetrain_model.attach_hooks` is the one call a physics loop needs.

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
`prospective` (skipped unless named): the pointer falls 10 of 18, the four
smoothness-loss exports 0 of 18. Take its entry out in the commit that moves
`control.general_move`.

**Digests, checked 2026-10-05**: `plant_digest` 12d9639e3ce90f36,
`design_digest` 5fe8c19379988031; `deploy/bundle.npz` re-exported and
matches both (worst schedule fit R^2 0.962). **Copy it to the Pi by hand**:
`deploy/` is gitignored. No export carries this plant: the four
smoothness-loss exports are at 9953690d, the pointer at e1ec36bf -- both
before the wheel. History: `params-digest-split.md`.

**Open, known:**

- **AHRS error costs far more without the velocity estimate**, for the LQR
  and RL alike; mechanism untested (`lqr-baseline.md`).
- **`tm151_filter` is not a drop-in**: the sim's accelerometer on a standing
  bike is violent (contact bounce), and the filter reads it as ~8 deg of
  tilt. Waits on the contact and the bike's AHRS log (`ahrs-fixture.md`).
- **The mirror's wifi stalls** (100-550 ms gaps, 09-17) did not repeat;
  cause unknown. Probe when it chugs again (`pi-bench-bringup.md`).
- **Not yet checked on hardware**: the rear wheel's sent angle against a
  hand-turned wheel; whether the onboard AHRS reader has the fixture logger's
  0.19 s stall; one unrecorded preflight failure after a reboot (09-18).
- **`MIN_FIT_R2` 0.93** is a bar to re-derive once the contact is measured:
  the LQR fit gets worse as the contact gets more realistic.

---

## What drives the bike

`control.general_move` = **`general_rl_cmd_curriculum2b`** (09-14):
sensor-trained (odometry estimate + TM151), command-family curriculum.
Candidate replacement: `general_rl_smooth_temporal` (10-01), above. Six
policies trained on the detailed drivetrain each beat the pointer at their
own gain and lose at the other; the pointer stays until the drivetrain model
is confirmed on the bike. Trained-on-sensors beats trained-on-truth, 1.00 to
0.20 survival. Standings and pointer history: `sensor-workstream.md`,
`drivetrain-model.md`.

**Do not compare `metrics:` blocks across `moves/*.yaml`**; re-run
`analysis/per_command.py` (`CLAUDE.md`).

---

## The guesses

**15 `source: GUESS` in `config/bike_params.yaml`** (2026-10-05: the front wheel's `crown_radius`, a stand-in for the flat tread). 2026-10-02: the drivetrain block brought in the roller slop's `centring_stiffness` and `damping`; `righting.wings.min_pinion_radius` left (a constant of the retired geared wings, now `build_model.MIN_PINION_RADIUS`); `input_armature` and the hub/roller joint damping and frictionloss became `design` -- the IDEAL plant's own inertia and only drive losses, kept at their values (the measured inertia is the overlay's `rotor_inertia`), and ZEROED by `DrivetrainSim` on the detailed plant, where the measured coast friction already contains them (`drivetrain_fit.py coast`, the whole wheel in the air) and the slop tendon alone damps a roller in its play. That zeroing changed the detailed plant in code, which no digest sees: the drivetrain-trained exports are now ~1% off their training plant in drive friction (estimated), each with a note there
on how to identify it. Never promote one quietly: promoting is a physical
parameter change (the `CLAUDE.md` checklist).

| parameter | value | what is known | dies at |
|---|---|---|---|
| `chassis.mass` | 0.45 kg, **44% of the bike** | CAD printed-mass estimates per part, uncalibrated (the chassis's new parts ~80 g printed) | the frame; weigh one printed part first |
| `fork_mass` + `front_wheel.mass` | 0.025 + 0.060 kg | **86 g together** (fork halves + wheel + axle, 09-30), not split | a scale |
| `front_wheel.crown_radius` | 80 mm | the real tread is a 10 mm flat; the crown stands in for the tire flattening under load | the dial test repeated with the wheel tilted 1-2 deg |
| `ahrs.mass` | 0.012 kg | the TM151 is on the bench | a scale |
| `payload.electronics.mass` | 0.076 kg | written for a Zero 2 W stack; the bench runs a Pi 3 | a scale |
| `payload.battery.mass` | 0.115 kg | no pack yet | the pack |
| `contact_solimp` | MuJoCo stock | hand drops: the real contact is softer at first touch and stiffer at peak than any `solref` gives | the drop rig + bench fit |
| `friction_sliding` | 0.9 | nothing | P0b incline slide |
| `friction_torsional` | 0.005 | deferred; the risk is setting it high | later |
| `drivetrain_model.roller_slop.centring_stiffness` / `.damping` | 1e-3 / 2.9e-5 | the only damping of a roller inside its +-7.8 deg play; set near critical (zeta 0.95). **Sensitivity, 2026-10-02** (`analysis/slop_sensitivity.py`, eval grid): stiffness x0.1, or the slop removed, moves `smooth_temporal` by <= 0.005 (noise floor 0.000); **stiffness x10 drops it 0.818 -> 0.598**, 2 falls. Damping x0.1 / x10 does nothing to it, but `drivetrain_p100_1` (a chaotic grid: x1.01 changes which commands fall) loses 0.21 at damping x10. The hand note "springs back, not every time" points SOFT, the harmless side | qualitative first: does a released roller snap back (stiff) or drift / stick (soft)? A slow-mo release only if stiff |
| `righting_sign` | +1 | a build decision; the render only | mounting the motor |
| `hockey.ball.radius` / `.mass` | 33.5 mm / 60 g | the ball shot is parked | the ball |

So: six die on a scale once the as-built parts exist; three with the
contact work, plus the front crown with a tilted dial test; the roller
slop's spring and damping with a measured in-play flick; three are parked
(`righting_sign`, the hockey ball).

**How the drivetrain is carried.** `bike_params.yaml` is the IDEAL drive.
The detailed one is the overlay `config/drivetrain_model.yaml`, and a policy
trained on it records its own copy in its `moves/*.yaml` (`drivetrain_model:`
block, gains included); teleop, the eval env and the endurance test rebuild
that policy's plant from its record. The pointer trained on the ideal drive;
`smooth_temporal` and the `drivetrain_p*` policies on the overlay at P 100 /
P 400.

**The contact as it ships:**

    contact_solref: [0.005, 1.0]     # positive: (timeconst_s, dampratio)
    contact_solimp: [0.9, 0.95, 0.001, 0.5, 2.0]   # MuJoCo stock, GUESS

- Settled on the bike (`analysis/contact_calibration.py`): rear 0.148 mm at
  its own 5.8 N, 0.57 mm at 44 N; front 0.049 mm at 4.2 N.
- **Critically damped, so it cannot bounce.** The real front wheel does.
- **Not a stiffness in N/m**: the sink scales with the masses compiled around
  the contact, so fit on the bike's own model after the weighing (it is why
  front and rear differ at the same settings: MuJoCo sees the rear rollers on
  a free hub, ignoring the belts, so the rear acts like a 134 g body and the
  front like 215 g; one point at 11 N, rear 0.25 mm, front 0.14). `dampratio`
  enters only the stiffness, as 1/dampratio^2; label any table by sink in mm.
- Plan: measure, then the negative `solref` form. `dmin` 0.9 -> 0.5 moves the
  rear rest sink 3.6x.

**Bench pre-tests (rough, not fits; `contact-measurements.yaml`):**

| dial, 2026-10-01: sink added by 26.7 N from the wheel's own weight | measured | sim |
|---|---|---|
| one roller, flat | 0.36 mm | 0.36 |
| two small ends | 0.36 | ~0.24 |
| front | 0.36 | 0.31 |
| one roller, big end | 0.61 | 0.47 |
| two big ends | 0.48 | 0.30 |

Drop, 2026-10-02, front wheel + fork (86 g) by hand onto the force sensor:
restitution ~0.40 over 1-2.6 mm, contact ~7 ms, peaks 11-16 N. `dampratio`
~0.3 matches the bounce, but the sim's peaks are 20-30% lower and its
contacts ~40% shorter. **Heads-up:** `[0.005, 0.30]` is the floor sweep's
stiff-end cliff (survival 0.35), so the fix is not that pair alone.

Outstanding with the bench fit: the `contact_solimp` comment in
`bike_params.yaml` quotes posed numbers ("0.375 -> 0.647 mm, 2.3x"; settled
0.148 -> 0.527, 3.6x). Left until a measured claim replaces the whole comment.

---

## Risks, ranked

1. **No policy has trained over contact-stiffness variation.** The axis sits
   commented out in `config/rl_general.yaml`. Soft is safe (survival 1.00
   from 1.8 mm of sink down to the shipped 0.15 mm); the cliff is at the
   stiff end, where the drop test points. The most likely single cause of a
   policy that works in sim and not on the floor
   (`floors-and-the-contact-model.md`).
2. **A third of training runs drive backwards when told forward**, and the
   score cannot see it (12 seeds, 09-10). Item 4 above
   (`seed-sweep-and-personalities.md`).
3. **The front tire and the wings still have a TPU roller's compliance.** A
   one-line `sim.contact_parts` edit now, not a refactor.
4. **Authority derating.** The torque scale is still the datasheet's
   1.6 N m; D5 or servo-strength randomisation covers it.
5. **Left/right asymmetry and crab**, flipping sign between policies.
6. **Steer homing at power-up is undesigned**: the turn count is RAM.
7. **Front-wheel liftoff is invisible to the estimator** (79 mm in one arm).

---

## Where everything is written down

Every plan doc opens with a status banner; read that first. **Bold** marks
what moved there from this file on 2026-10-02.

| doc | owns |
|---|---|
| `first-physical-test.md` | the build order |
| `untethered-setup.md` | the physical bike: power, wiring, wifi, onboard software |
| `pi-bench-bringup.md` | the Pi bench, **and the 09-15 to 09-18 bring-up log** (loop, AHRS push, mirror, telemetry, recorder, cuts, wifi, the mirrored drive servos) |
| `pre-assembly-bench-checklist.md` | DRAFT floor-rig ideas and experiments |
| `sensor-workstream.md` | the odometry/AHRS arc and policy standings, **the standing-fall analysis and pointer history** |
| `ahrs-fixture.md` | **NEW: the TM151 fixture, its findings and CAD** (parked) |
| `lqr-baseline.md` | **NEW: the LQR's dated health log, `MIN_FIT_R2`** |
| `odometry-rewrite.md` | the estimator, **and the 09-21 test rewrite** |
| `eval-score-rewrite.md` | how policies are scored; the open directional gate |
| `seed-sweep-and-personalities.md` | the 12-seed sweep and failure taxonomy |
| `policy-smoothness-losses.md` | CAPS-style losses and the 10-01 results |
| `general-rl-improvements.md` | collected RL findings, reference |
| `mujoco-modeling-decisions.md` | why the model is built as it is |
| `floors-and-the-contact-model.md` | solref/solimp, **the floor sweep (risk #1) and the contact split** |
| `aow-contact-approximations.md` | contact surrogates, timestep/mesh |
| `drop-release-rig.md` | the drop rig, in design |
| `drivetrain-model.md` | the detailed drivetrain, the P 100 / P 400 question |
| `steering-design.md` | the steer in CAD and its first print, **and the 09-21 to 09-30 status log** |
| `drive-design.md` | the rear drive in CAD |
| `righting-design.md` / `righting-linkage-margin.md` | the printed righting module / the linkage search |
| `righting-servo-model.md` | the XC330 model from the bench, **and its status log** |
| `bam-current-position.md` | BAM-style identification for mode 5 |
| `wing-linkage-design-and-optimization.md` | the righting mechanism; `wing-linkage-metal.md` superseded |
| `bike-assembly-design.md` | the whole bike in CAD |
| `cad-onshape-workflow.md` | the Onshape round trip and its quota |
| `self-righting.md` | where recovery stops being possible |
| `params-digest-split.md` | the two digests, **and their history** |
| `asymmetric-actor-critic.md` | a parked option |
| `ball-shot-move.md` | the ball shot, parked |
| `plans/old/` | retired docs |

`docs/measurements/` holds the protocol/data pairs (contact, drivetrain,
servo, floor, omni wheel) and `servo-logging.md`. `docs/cad/` is generated:
regenerate, never hand-edit. `bench/` holds the force-sensor and drop-rig
tools, `bench/logs/` their captures.

---

## Explicitly not being worked on

The PD cascade (legacy). The trajopt moves `flick` / `flick_fwd` / `flip`
(deprecated and skipped; `flick_rl` is not), re-authored once the as-built
mass is known. A live gamepad front-end. The ball shot. Re-tuning the LQR
before the contact is pinned. The disturbance curriculum, parked until the
params files are reconciled.
