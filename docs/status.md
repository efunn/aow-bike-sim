# Project status — 2026-10-16

**How to read this.** The navigation layer: what is true now, what to do next,
and where the detail lives. Rewritten, not appended to. **For the reasoning,
follow the link — it is not repeated here.**

Rewritten 2026-10-02 (1798 lines -> ~300; the old file is `git show
d0dffeb:docs/status.md`) and again 2026-10-16, when solved problems still
listed as open were moved out: the GPU stack, the retrains on today's plant,
and the 10-14 env fixes had all landed without this file catching up.

---

## Where the project is, in five lines

1. **The simulator is trusted for control design**: parametric MuJoCo, an
   omni-wheel contact, the bike's own sensors in the loop and in training,
   measured XC330 laws, headset friction, the detailed drivetrain and the
   righting module. The contact is the least-measured part; bench pre-tests
   started 2026-10-01.
2. **RL drives, and it trains on a rented GPU.** The pointer is a
   GPU-trained MinGRU policy (`general_puffer_fixes_72M`, 10-16); MuJoCo
   Warp + PufferLib train 72M steps in ~12 min on an RTX 5090 (~$0.23). The
   box's CPU stack stays as the reference. The analytic LQR is DEPRECATED
   (10-09).
3. **The onboard path runs on the Pi bench** (09-15 to 09-18): 100 Hz loop,
   TM151 pushing from flash, mirror, telemetry, recorder, fall/cut/re-arm,
   low-voltage cut. Torque on only with the bike held. No pack yet.
4. **Hardware is printed module by module.** Built: servo bench, rear
   drivetrain station, the front steer (09-30), the printed righting module
   (by 10-16; blades not yet fitted). Designed, not printed: the rear drive.
   CAD v2 (10-11) lays out the whole bike; since 10-16 the drive's case
   sides grow forward as its side rails (the chassis, in two halves).
5. **The goal is a functioning bike with sample videos, in weeks.** Ranked
   against that; the showcase moves are in `wishlist.md`.

---

## What to do next

| # | do | why now | doc |
|---|---|---|---|
| 1 | **Contact calibration**: stiffen the force sensor's mount (resolder or glue) and check it with a rigid-block drop; then the drop rig with added mass at the wheel (front, then rear), a joint `solref` + `solimp` fit with the dial (P0) data, and the P0b pull test and tread print (friction) | First rig run (10-03, front, 65 drops): the rig's load path reads ~7-30x softer than the dial says the wheel alone is, and the base rings at ~170-180 Hz -- probably the sensor mount, so fits so far describe the rig, not the tire. MuJoCo's contact is per unit mass (same bounce at any mass), so a fit reaches the bike only through each wheel's effective mass | `drop-release-rig.md`, `measurements/contact-protocol.md`, `analysis/drop_rig_sim.py` |
| 2 | **Print the rear drive**, then fit the blades to the righting module | The bushing and end-play results are in (09-30); the module is built (10-16) | `drive-design.md`, `righting-design.md`, `aow-bike-whole-v2.md` |
| 3 | **Finish the drivetrain station**: firmware Velocity P and I, the D5 torque-scale check, fit the five drivetrain `GUESS`es | The sim cannot pick the gain (policies tie at P 100 / P 400; P 400 buzzes ~3x harder). Decide on the bike | `drivetrain-model.md`, `drivetrain-measurements.yaml` |
| 4 | **Sim: the eval score's directional gate** | `_score` is still `survive_rate x track_geo`; a policy that abandons a direction outranks good ones. Every export since v3 drives both straight commands forward, so this is insurance for future runs. Risk #2 | `eval-score-rewrite.md` |

Weighing waits for the as-built parts: the mass entries will be re-cut by
module (the Pi 3 is a stand-in, the printed modules are new), so weigh once
they exist. One printed part weighed sooner would calibrate the CAD estimates
(`cad_bike.printed_mass`).

Bench loose ends: the second XC330 is still id 103 and must become 104 before
the two share a bus.

Not next, deliberately: a trained (RL) ball shot (close-range scripted plays
exist in teleop), the privileged critic, the odometry rewrite, the AHRS
fixture (parked 09-24), the self-righting linkage choice (parked 09-27).

---

## The workstreams

| workstream | state | blocker | doc |
|---|---|---|---|
| **Simulation & model** | Working; 21 `GUESS`es in bike_params. Plant digest **bc66ca20** (10-14): the measured front wheel (10-05), the detailed drivetrain as the default plant (10-02), the V2 righting module (10-09) with the V3 blade (10-14). XC330s from the bench (current law, load-proportional gearbox friction); the steer as its firmware runs it, 4 ms delay; headset friction since 09-30. Optional per-episode floor tilt for training, CPU and GPU, off by default (10-16) | Contact and masses, unmeasured | `mujoco-modeling-decisions.md`, `drivetrain-model.md` |
| **Control — RL** | Primary. Pointer `general_puffer_fixes_72M` (10-16): GPU, MinGRU (stateful), `tm151_filter` + odometry counts, detailed drivetrain, today's plant. On one `per_command.py --summary` grid (`--ahrs tm151_filter`): 0/20 falls, 7/9 turns resolved forward, 2/2 straight forward; chatter score 0.863, per-step action change 0.008 (v3 seeds 0.035-0.048); endurance minute **0/18 falls** (10-16). Two GPU seeds pick the same direction on all 20 grid commands. CPU arms on today's plant, finished on the box: `queue_general_v3` (5 seeds, MLP) and `queue_general_v3_mingru` (5 seeds, MinGRU actor); turns forward 4-8/9 (s0-s2 measured) and 5-7/9 (all five), 0 falls. `v3_s1` and `v3_mingru_s0` are on top of `moves/` | Crab never trained (`v_lat_frac` 0; crab L/R ~0.45/0.63 on the pointer); not flown on the bike | `gpu-training.md`, `general-rl-improvements.md`, `policy-smoothness-losses.md` |
| **GPU training** | Phases 1-3 done (10-14 to 10-16): MuJoCo Warp env as one CUDA graph per policy step, bridged into PufferLib 5.0's CUDA trainer, patched for reward scale, advantage normalisation, the temporal smoothness term and a paced shared curriculum. Checkpoints convert to moves (`gpu/export_policy.py`). Cards: 5090 12 min / $0.23 per 72M steps, 4090 17 min, A40 38 min. Runs are not bit-repeatable (float atomics) | Phase 4 (MLP / loss parity) and 5 (five GPU seeds vs v3's five on one grid) not started; no sim-to-sim check of a GPU policy in C beyond the eval grid | `gpu-training.md` |
| **Sensor modelling** | Largely done. TM151 measured on the fixture: a complementary filter, tau 0.19 s at rest to ~1 s moving, ~0.2-0.3 deg RMS at one mount against the sim's 1.5. `tm151_filter` with the accelerometer weighting from the drop rig (10-07) is the training level since 10-09, provisional; unproven on sustained acceleration | The bike's own AHRS log; the wave cams (sustained acceleration) | `sensor-workstream.md`, `ahrs-fixture.md` |
| **Control — LQR** | **Deprecated 10-09.** `lqr` tests skipped unless named (`pytest -m lqr`: the 15 registered reds). The code stays: the RL envs' crawl fallback and the bundle's gain schedule are built on it. Last state (10-05): stands on truth and `tm151_filter`, falls turning at 0.8 m/s | none: not being tuned | `lqr-baseline.md` |
| **Hardware / onboard** | Pi 3 bench proven: tick jitter p99 < 1 ms with four servos energised. 2.4 GHz house wifi (decided: a router, no AP). The SBC that ships: open | Chassis, pack | `pi-bench-bringup.md`, `untethered-setup.md` |
| **CAD** | Steer printed and print-checked; the righting module printed (blades not on). **v2 (`aow-bike-whole-v2.md`, then `aow-bike-whole-v2-complete.md` from 10-16).** Drive: fixture mock off, the user's chainstay finishing folded in; **its case sides are now the bike's SIDE RAILS** (10-16): each grows forward to the steering's side pad and down to the righting's bridge (now as wide as the gap between the rails), one 6-32 a side at each, with mirrored 15 deg cage posts (a side 6-32 each, the nut slot through fore-aft) and a full-width tab a side holding the TM151 on its own M2 holes, USB up. The steering and the righting slide out of the rails downward (ridges along the slide, grooves open at the rail's edge). Steering v2: servo out the FRONT, pads 3.65 to take the rail, ONE upper <-> lower screw at the rear (lower case ~12.2 g ASA). Righting: stubs and the V3 spline blade, centred; bridge lugs for the rails. Snug 6-32 nut slots (6.45 x 2.7, GUESS) in the v2 modules. **The power holder** (`cad_power`, 10-16): one printed panel on the left between the battery and the rail (printed -X), the power board on standoffs outboard of it square to the drive (XT30 up and back), the switch panel-mounted in a shelf round the XT30, two strap slits; two 6-32 into the left rail on two new case-side ears, fore and aft of the board (nut in the panel, screws cut to 8). No tray: the battery is where it was. The Pi + U2D2 carrier as an OUTLINE (`cad_pi_carrier`): the Pi on a plate over the front face, the U2D2 under it between the drive pulleys, slid up so its JSTs clear the AHRS; not attached yet. The v2 bike: `--check` clean at rest, steered and every pose. v1 (`cad_bike`) is frozen (as-built nut slots, no rails) and still builds | Print-fit the snug nut slots and the M2 tabs; the Pi carrier's attachment (with a third power-carrier side screw near the left drive pulley, user), print-fit the holder's cut screws and nuts; the straps (`aow-bike-whole-v2-complete.md`); delete the drive's four hand features; CoM not computed; wing-to-blade (not yet, user) | `aow-bike-whole-v2-complete.md`, `aow-bike-whole-v2.md`, `cad-onshape-workflow.md` |
| **Self-righting** | V2 linkage with the **V3 blade** is the default (10-14, user): level at 116.98 deg (commanded 116.9), lying at 85 deg, 566 counts in the sim, toe stop at ~130. V2's blade is `--blade config/righting_blade.yaml`; a blade file may carry its own `mass` and `crank_travel_deg`. **Lie down / get up / auto-up** (10-16): `control/swing_sit.py`, teleop PageDown / PageUp, and by itself after a fall (`--no-auto-up`); a fall after the hand-off folds the blade and re-arms the auto-up. Sim only: down 5.8 s at <= 102 deg/s, up 3.3 s; 24/24 in teleop | Weigh the printed parts; a crank coast-down for the XC330's reflected inertia; fly lie-down / get-up on the bike | `righting-linkage-margin.md`, `righting-servo-model.md`, `wishlist.md` |
| **Contact bench** | Drop rig built and run (10-03): release-to-impact timed, analysed at the contact through the arm (lever 205/124, m_eff 89 g fitted -- to re-derive from the weighed parts). MuJoCo twin `analysis/drop_rig_sim.py --fit`. AHRS mode drawn (10-06): a jam-on follower tip and an 11-cam wave kit, nothing printed. `force_drop.py --ahrs` (10-07) logs the TM151 and the servo with the drops, one clock. On the bar the TM151 steps within ~0.07 deg of the cam's; the arm is not rigid, so stations do not share a rotation | Print the wave cams + the tip, then `bench/wave_run.py` (sustained acceleration); power for the force sensor on the Pi (three USB devices reboot-loop it); a local truth per station; the sensor epoxied down; the arm's parts weighed | `drop-release-rig.md`, `contact-measurements.yaml` |

---

## Health

**Tests, 2026-10-16**: `907 passed, 52 skipped`, red set unchanged, **no
accepted reds** in the default run (`pytest -n 10 --dist loadgroup`, 52 s).
The 15 registered reds are all the deprecated LQR and are only judged under
`pytest -m lqr`. `prospective` (the endurance minute, skipped unless named)
passes on the pointer: 0/18 falls.

**Digests, 2026-10-16**: `plant_digest` bc66ca20e1af66d5, `design_digest`
5fe8c19379988031; `deploy/bundle.npz` (re-exported 10-14) matches both.
**Copy it to the Pi by hand**: `deploy/` is gitignored. History:
`params-digest-split.md`.

**Which exports carry today's plant (bc66ca20):** 23 -- every
`general_puffer_*` checkpoint, `general_rl_v3_s0..s4`,
`general_rl_v3_mingru_s0..s4`. Everything else in `moves/` trained on an
older plant (old front wheel, wingless or V1 module) and is provisional.
Read a move's own `plant_digest`, never its name.

**Env fixes of 10-14 that split the history** (re-run `per_command.py`
rather than compare across the date): eval rows carried a steer kick from
the row before (`reset_actuator_state`); the obs's prev_action was one step
stale in training; the reset obs read MuJoCo truth for roll, rates and pitch.
`queue_general_v3` trained before them, the MinGRU arm and every GPU run
after.

**Open, known:**

- **AHRS error costs far more without the velocity estimate**, for the LQR
  and RL alike; mechanism untested (`lqr-baseline.md`).
- **`tm151_filter` is provisional.** The unweighted filter read contact
  bounce as ~8 deg of tilt; the accelerometer weighting fitted on the drop
  rig and fixture (`FILTER_ACC_D0_G` 0.12, `FILTER_ACC_HOLD_S` 0.1) fixed
  that. Unmeasured: sustained acceleration (turns, crab -- 5.4-5.8 deg
  there). Its `ahrs_tau_s` only times the 0.1 deg residual wander (2.0 s);
  the filter's own 0.19 / 1.0 s is built in.
- **A MinGRU policy latches state at reset.** On `fixes_52M`, 7 of 9
  slow units are written in the first steps after a reset and held for the
  episode, so a bogus first observation can persist (`analysis/mingru_memory.py`,
  `gpu-training.md`). Replay must reset the hidden state per engage.
- **Why `fixes_72M` drops a swing-sit hand-off on the base params** that
  `fixes_52M` catches: one trace, not chased (it recovered on its own
  drivetrain; the sequencer now retries).
- **The mirror's wifi stalls** (100-550 ms gaps, 09-17) did not repeat;
  cause unknown. Probe when it chugs again (`pi-bench-bringup.md`).
- **Not yet checked on hardware**: the rear wheel's sent angle against a
  hand-turned wheel; whether the onboard AHRS reader has the fixture logger's
  0.19 s stall; one unrecorded preflight failure after a reboot (09-18).
- **`MIN_FIT_R2` 0.93** is a bar to re-derive once the contact is measured:
  the LQR fit gets worse as the contact gets more realistic.

---

## What drives the bike

`control.general_move` = **`general_puffer_fixes_72M`** (10-16; was
`general_rl_smooth_temporal` from 10-09). Numbers in the RL row above.
Teleop flies it on `tm151_filter` by default (`--general NAME` for another);
the scripted plays (`--play config/plays/*.yaml`, End) were tuned on
`seed1_72M`.

**`moves/` layout (10-16).** The top level holds the pointer, today's GPU
finals (`fixes_52M`, `seed1_72M`, `next_72M`), `v3_s1`, `v3_mingru_s0`, the
previous pointer, and the moves teleop's keys and the tests name (`flick*`,
`ball_rl`, `pivot_rl`, `general_rl`). The other 84 are in `moves/archive/`,
tracked, and still load by bare name (`control.flick.move_path`); they are
out of the teleop `,` menu. Retiring a move is a `mv`. `moves-layout.md`.

The previous pointer, `smooth_temporal`: odometry estimate + TM151 at 0.19,
detailed drivetrain at P 100, the temporal smoothness loss, stood the
endurance minute; trained on an older plant. Pointer history:
`sensor-workstream.md`, `drivetrain-model.md`.

**No-diff probe, `general_rl_nodiff_2` (10-09, one seed, not the pointer).**
A one-off test of how a plain bike could use the swing linkage with tip
masses to balance; NOT a line of work -- no further no-diff runs are planned
(user, 10-09). Common-mode drive + steer + the four-bar swing wings with
100 g tips, the differential faded out by the curriculum (zero by ~10M of
18M steps), a wing on the floor a fall. On its own sensors it stands the
endurance minute 0/18 falls (`tm151` and `tm151_filter` both) -- but CREEPS
forward ~7 cm/s on a zero command (4.4-5.7 m in 60 s). The wing is BALLAST:
the crank sits near -95 deg, the bike leans ~3.8 deg, the low wing hovers
4-6 mm off the floor at its closest and ~11 mm median, never touching.
Frozen at that angle it still stands 0/18 (roll spread 0.8 deg vs 1.25
live); frozen at stow 18/18 fall (MTBF ~11 s). The 5 s training eval cannot
see the creep. Trained on the V1 linkage (`swing_linkage_smaller`), NOT V2,
the bike's module; it does not transfer. `rl_general_nodiff.yaml` was moved
to V2 with the rest of the configs, for consistency only.

**Do not compare `metrics:` blocks across `moves/*.yaml`**; re-run
`analysis/per_command.py` (`CLAUDE.md`).

---

## The guesses

**21 `source: GUESS` in `config/bike_params.yaml`.** Never promote one
quietly: promoting is a physical parameter change (the `CLAUDE.md`
checklist).

| parameter | value | what is known | dies at |
|---|---|---|---|
| `chassis.mass` | 0.45 kg, **37% of the bike** | CAD printed-mass estimates per part, uncalibrated | the frame; weigh one printed part first |
| `righting.module` masses (crank, coupler, wing_hub, blade, fixed) | 11.2 / 2.3 / 14.9 / 28.6 / 64.1 g, **167 g** with both sides | CAD check volumes (10-07), 2 perimeters / 15 % infill, steel pins | weigh the printed parts |
| `righting.module.crank_armature` | 1e-3 kg m2 | the XC330's reflected rotor inertia; >= 1e-5 stops the bare actuator ringing; 1e-4 -> 1e-3 slows the full stroke 6 % | a crank coast-down, as `drivetrain_fit.py coast` |
| `fork_mass` + `front_wheel.mass` | 0.025 + 0.060 kg | **86 g together** (09-30), not split | a scale |
| `front_wheel.crown_radius` | 80 mm | the real tread is a 10 mm flat; the crown stands in for the tire flattening under load | the dial test repeated with the wheel tilted 1-2 deg |
| `ahrs.mass` | 0.012 kg | the TM151 is on the bench | a scale |
| `payload.electronics.mass` | 0.076 kg | written for a Zero 2 W stack; the bench runs a Pi 3 | a scale |
| `payload.battery.mass` | 0.115 kg | no pack yet | the pack |
| `contact_solimp` | MuJoCo stock | hand drops: the real contact is softer at first touch and stiffer at peak than any `solref` gives | the drop rig + bench fit |
| `friction_sliding` | 0.9 | nothing | P0b pull test |
| `friction_torsional` | 0.005 | deferred; the risk is setting it high | later |
| `drivetrain_model.roller_slop.centring_stiffness` / `.damping` | 1e-3 / 2.9e-5 | the only damping of a roller inside its +-7.8 deg play (zeta 0.95). Sensitivity (10-02, `analysis/slop_sensitivity.py`): stiffness x0.1 or no slop moves `smooth_temporal` <= 0.005; **x10 drops it 0.818 -> 0.598**. The hand note "springs back, not every time" points SOFT, the harmless side | qualitative first: does a released roller snap back (stiff) or drift / stick (soft)? |
| `righting_sign` | +1 | a build decision; the render only | mounting the motor |
| `hockey.ball.radius` / `.mass` | 33.5 mm / 60 g | used by teleop's `--hockey` and the plays | the ball |

So: six die on a scale once the as-built parts exist; three with the
contact work, plus the front crown with a tilted dial test; the roller
slop's spring and damping with a measured in-play flick; three are parked
(`righting_sign`, the hockey ball). The remaining drivetrain `GUESS`es wait
on the station (next, #3).

**How the drivetrain is carried.** The detailed drive is the
`drivetrain_model:` block in `bike_params.yaml` (since 10-02);
`drivetrain_model.base_params` gives the IDEAL drive, which the LQR is
designed on and the flick/pivot/ball envs run on. **A model built with the
block IS the ideal plant until `DrivetrainSim.attach` switches it** (native
drive zeroed, measured rotor inertia, slop tendons on; `release` switches
back, both pinned bit-exact by tests), so loops that never attach the hook
-- contact studies, settles -- run on the ideal drive, and the hooked ones
(RL envs, teleop, `balance.run`, `record`) on the detailed one;
`drivetrain_model.attach_hooks` is the one call a physics loop needs. A
policy records its own copy in its `moves/*.yaml`, and teleop, the eval env
and the endurance test rebuild that policy's plant from it.

**The contact as it ships:**

    contact_solref: [0.005, 1.0]     # positive: (timeconst_s, dampratio)
    contact_solimp: [0.9, 0.95, 0.001, 0.5, 2.0]   # MuJoCo stock, GUESS

- Settled on the bike (`analysis/contact_calibration.py`): rear 0.148 mm at
  its own 5.8 N, 0.57 mm at 44 N; front 0.049 mm at 4.2 N.
- **Critically damped, so it cannot bounce.** The real front wheel does.
- **Not a stiffness in N/m**: the sink scales with the masses compiled around
  the contact, so fit on the bike's own model after the weighing (the rear
  acts like a 134 g body, the front like 215 g). `dampratio` enters only the
  stiffness, as 1/dampratio^2; label any table by sink in mm.
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

1. **No policy has trained over contact-stiffness variation**, CPU or GPU
   (`randomization:` varies mass, friction and actuator strength, not
   `solref`). Soft is safe (survival 1.00 from 1.8 mm of sink down to the
   shipped 0.15 mm); the cliff is at the stiff end, where the drop test
   points. The most likely single cause of a policy that works in sim and
   not on the floor (`floors-and-the-contact-model.md`).
2. **A policy can trade away a direction and the score cannot see it.** A
   third of the 09-10 sweep's runs drove backwards when told forward (12
   seeds). Every export since v3 drives both straight commands forward, but
   nothing gates it (next, #4; `seed-sweep-and-personalities.md`).
3. **A GPU-trained policy is checked against C MuJoCo only on the eval
   grid.** MJWarp lays more contact points than C (0.7 mm / 0.24 deg apart
   in 100 ms open loop); the pointer flies fine in C on every test so far,
   but phase 5's sim-to-sim comparison has not been done (`gpu-training.md`).
4. **The front tire and the wings still have a rubber roller's compliance.** A
   one-line `sim.contact_parts` edit now, not a refactor.
5. **Authority.** The torque scale is still the datasheet's 1.6 N m;
   training randomises actuator strength +-15 % (`actuator_frac`), the
   station's D5 check measures it.
6. **Left/right asymmetry and crab**, flipping sign between policies; crab
   is not trained.
7. **Steer homing at power-up is undesigned**: the turn count is RAM.
8. **Front-wheel liftoff is invisible to the estimator** (79 mm in one arm).

---

## Where everything is written down

Every plan doc opens with a status banner; read that first.

| doc | owns |
|---|---|
| `first-physical-test.md` | the build order |
| `wishlist.md` | the showcase moves: ramps, ball shots, lie-down / get-up, wheelie |
| `untethered-setup.md` | the physical bike: power, wiring, wifi, onboard software |
| `pi-bench-bringup.md` | the Pi bench and the 09-15 to 09-18 bring-up log |
| `pre-assembly-bench-checklist.md` | DRAFT floor-rig ideas and experiments |
| `gpu-training.md` | MuJoCo Warp + PufferLib: phases, pod runbook, card choice, results |
| `moves-layout.md` | `moves/` and `moves/archive/` |
| `sensor-workstream.md` | the odometry/AHRS arc, policy standings and pointer history |
| `ahrs-fixture.md` | the TM151 fixture, its findings and CAD (parked) |
| `lqr-baseline.md` | the LQR's dated health log, `MIN_FIT_R2` |
| `odometry-rewrite.md` | the estimator and the 09-21 test rewrite |
| `eval-score-rewrite.md` | how policies are scored; the open directional gate |
| `seed-sweep-and-personalities.md` | the 12-seed sweep and failure taxonomy |
| `policy-smoothness-losses.md` | CAPS-style losses and the 10-01 results |
| `general-rl-improvements.md` | collected RL findings, reference |
| `mujoco-modeling-decisions.md` | why the model is built as it is |
| `floors-and-the-contact-model.md` | solref/solimp, the floor sweep (risk #1) and the contact split |
| `aow-contact-approximations.md` | contact surrogates, timestep/mesh |
| `drop-release-rig.md` | the drop rig, built and run (10-03) |
| `drivetrain-model.md` | the detailed drivetrain, the P 100 / P 400 question |
| `steering-design.md` | the steer in CAD and its first print, and the 09-21 to 09-30 log |
| `drive-design.md` | the rear drive in CAD |
| `righting-design.md` / `righting-linkage-margin.md` | the printed righting module / the linkage search |
| `righting-servo-model.md` | the XC330 model from the bench, and its status log |
| `bam-current-position.md` | BAM-style identification for mode 5 |
| `wing-linkage-design-and-optimization.md` | the righting mechanism; `wing-linkage-metal.md` superseded |
| `bike-assembly-design.md` | the whole bike in CAD, v1 (frozen 10-11) |
| `aow-bike-whole-v2.md` | the v2 bike and module updates (10-11) |
| `aow-bike-whole-v2-complete.md` | v2 continued: side rails, AHRS mount, cage stubs, carriers (10-16) |
| `cad-onshape-workflow.md` | the Onshape round trip and its quota |
| `self-righting.md` | where recovery stops being possible |
| `params-digest-split.md` | the two digests and their history |
| `asymmetric-actor-critic.md` | a parked option |
| `ball-shot-move.md` | the trained ball shot, parked |
| `plans/old/` | retired docs |

`docs/measurements/` holds the protocol/data pairs (contact, drivetrain,
servo, floor, omni wheel) and `servo-logging.md`. `docs/cad/` is generated:
regenerate, never hand-edit. `bench/` holds the force-sensor and drop-rig
tools, `bench/logs/` their captures. `analysis/gpu/` holds the GPU study's
pod scripts and results by phase.

---

## Explicitly not being worked on

The PD cascade (legacy). The analytic LQR (deprecated 10-09). The trajopt
moves `flick` / `flick_fwd` / `flip` (deprecated and skipped; `flick_rl` is
not), re-authored once the as-built mass is known. A live gamepad front-end.
A trained ball shot. The disturbance curriculum, parked until the params
files are reconciled. More no-diff runs.
