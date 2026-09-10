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
4. **Hardware has started.** Four servos on a bench, 2026-09-01. The Digi-Key
   order landed 2026-09-08: Pi 3, cables, electronics. No pack, no charger.
5. **The goal is a functioning bike with sample videos, in weeks.** Everything
   below is ranked against that.

---

## What to do next

Ranked by what unblocks the most, not by interest.

| # | do | why now | doc |
|---|---|---|---|
| 1 | **Weigh the electronics stack and pack** | Two `GUESS`es die in ten minutes; the parts are on the bench today | `first-physical-test.md` §0a |
| 2 | **Contact calibration — P0, P0b, P1, once per floor** | Needs no printing: a weight, a caliper, slow-mo, a tilting board. The contact is the least-measured thing in the sim and the one no policy has been randomised over — and the SPREAD across surfaces is what sets the randomization range | `floors-and-the-contact-model.md` |
| 3 | **Build the drivetrain station** | Five `GUESS`es die, and it is the load fixture the servo torque/droop tests need. The next thing that unblocks anything on the build track | `first-physical-test.md` §1 |
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

Not next, and deliberately: the self-righting wings (design done, build last),
the ball shot (works, off the path), the privileged critic (speculative), the
odometry rewrite (flown around, seven accepted red tests).

---

## The five workstreams

| workstream | state | blocker | owner doc |
|---|---|---|---|
| **Simulation & model** | Working. 17 parameters still `GUESS` | Physical parts to measure | `mujoco-modeling-decisions.md` |
| **Control — RL** | Working, and primary. Trains against the onboard sensors | Crab still one-sided; `turn_asym` stuck ~0.2 | `general-rl-improvements.md` |
| **Sensor modelling** | Largely DONE. Velocity estimate, encoder quantisation, TM151 error — all in training, validated against a real unit over USB | Dynamic attitude accuracy needs a moving bike | `sensor-workstream.md` |
| **Control — analytic (LQR)** | Reference baseline only. Marginally healthy | Nothing now; degrades when contact moves | `old/stationary-balance-controller.md` |
| **Hardware / untethered** | First measurements 2026-09-01. Bus layer validated at 500 Hz on 4 servos. Nothing assembled | Chassis, and the drivetrain station | `first-physical-test.md`, `untethered-setup.md` |
| **CAD** | Layout, drivetrain, steering and righting stations pinned. Electronics packing deferred on purpose | Nothing — it is being worked on | `cad-onshape-workflow.md` |
| **Self-righting mechanism** | Side project. Moved to asymmetric output / symmetric layout; torque analysis not trusted | Will be resolved by building, not by analysis | `wing-linkage-design-and-optimization.md` |

---

## Where everything is written down

`docs/plans/` holds reasoning and grows forever. This file is the layer on top.
Every doc opens with a **status banner** saying whether it is active, open,
parked or reference — read that before the body.

| doc | what it owns |
|---|---|
| `first-physical-test.md` | **the build order.** Which parts unlock which unknowns, in what sequence. Plus the three servo bench stations and the 09-01 bench results |
| `untethered-setup.md` | the physical bike: power, wiring, onboard software, Pi setup, verification. The umbilical path is §"Bench power" |
| `sensor-workstream.md` | the odometry/AHRS arc and the full policy standings — why sensor-trained policies win 1.00 to 0.20 |
| `eval-score-rewrite.md` | how policies are SCORED, why the early numbers hid a failure, and the command-distribution audit |
| `seed-sweep-and-personalities.md` | **the 12-seed sweep**: determinism, the failure taxonomy, why reverse is easier, early abort detection, and the options not taken |
| `general-rl-improvements.md` | collected RL findings. Reference, not a queue |
| `mujoco-modeling-decisions.md` | why the model is built the way it is. Reference |
| `odometry-rewrite.md` | the estimator itself, and why rewriting it was skipped |
| `floors-and-the-contact-model.md` | **the multi-floor plan** — solref/solimp history, why the parameters read backwards, the per-geom blocker, and how the spread across surfaces becomes a randomization range |
| `aow-contact-approximations.md` | the contact surrogate survey, the timestep/mesh result, and the two blocked drive-plant fixes |
| `cad-onshape-workflow.md` | the Onshape round trip and its API quota, plus everything drawn so far |
| `wing-linkage-design-and-optimization.md` | the righting mechanism as it now stands |
| `self-righting.md` | where recovery stops being possible; the fall cases. Reference |
| `params-digest-split.md` | the two digests and what each answers |
| `asymmetric-actor-critic.md` | a parked option, kept to cover the bases |
| `ball-shot-move.md` | the ball shot. Works, parked |
| `plans/old/` | six retired docs — built, superseded, or never started |

`docs/measurements/` holds hand-entered protocol/data pairs.
`docs/cad/` holds generated CAD output (`.fs`, `.png`, the layout YAML) — never
hand-edit those; regenerate with `aow_sim.cad_layout` / `cad_servo_mount` /
`cad_swing_linkage`.

---

## Health

**Test suite, measured 2026-09-09** with `pytest -n 10 --dist load`:

    23 failed, 285 passed, 9 skipped, 38.7 s
    red set unchanged (23 accepted failures) -- tests/expected_failures.txt

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
ever cleared 0.98 — not the current plant (0.9412), not the servo without its
integral term (0.9757). The fit gets *worse* as the contact gets more realistic:

| `contact_solref` dampratio | 0.3 | 0.5 | **1.0 (ships)** | 2.0 |
|---|---|---|---|---|
| worst R² | 0.8148 | 0.9297 | **0.9412** | 0.9602 |

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

**Digests, both current 2026-09-09:**

    plant_digest   e1ec36bfa670217e    was this trained against the machine I am running?
    design_digest  2db6c647ff3a2d59    were these gains designed against the weights I am running?

`deploy/bundle.npz` matches both. A failing digest check is the mechanism
working — fix it with `python -m aow_sim.export_deploy`, never by loosening the
check. See `params-digest-split.md`.

---

## What drives the bike

`control.general_move` names **`general_rl_odo_ahrs`**.

The split that matters is not one good policy against the rest — it is
**trained-against-the-sensors against not**. On the eval grid at
`--encoder counts --ahrs tm151`, score / survival:

| policy | trained against | tau 2.0 | tau 0.19 (measured) |
|---|---|---|---|
| `smooth_diff_pi` | MuJoCo truth | 0.110 / **0.20** | — |
| `odo_ahrs` ← **current default** | estimate + attitude, tau 2.0 | **0.672** / 1.00 | 0.570 / 0.95 |
| `odo_ahrs_rand2` | + attitude randomised, narrow | 0.654 / 1.00 | **0.663** / **1.00** |

Every truth-trained policy falls in four episodes out of five.

**Two open pointer questions, and they are different questions:**

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

17 marked `source: GUESS` in `config/bike_params.yaml`, each with a note on how
to identify it. Never quietly promote one. The load-bearing ones:

| parameter | value | dies at |
|---|---|---|
| `chassis.mass` | 0.45 kg — **44% of the bike** | stage 2, the frame |
| `contact_solimp` | MuJoCo stock | contact calibration (`dmin` confounds both bench tests) |
| `friction_sliding` | 0.9 | the incline slide, P0b — cheapest test in the project |
| `input_armature`, hub/roller damping + frictionloss | 5 of them | the drivetrain station, spin-downs |
| `payload.pack.mass` / `electronics.mass` | 0.115 / 0.076 kg | **a scale, today** |
| `min_pinion_radius` | 0.006 m | print a test pinion |

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

   **Measured 2026-09-09, `analysis/floor_sweep.py`, `general_rl_odo_ahrs`,
   score / survival over the 20-command grid, mu 0.9:**

   | sink @ bike weight | `contact_solref` | score / survival |
   |---|---|---|
   | 13.17 mm | `[0.020, 2.00]` | 0.594 / 1.00 |
   | 3.80 mm | `[0.020, 1.00]` | 0.657 / 1.00 |
   | 1.08 mm | `[0.005, 2.00]` | 0.673 / 1.00 |
   | 0.52 mm | `[0.020, 0.30]` | **0.719** / 1.00 |
   | **0.39 mm** | **`[0.005, 1.00]` — ships** | 0.663 / 1.00 |
   | 0.04 mm | `[0.005, 0.30]` | 0.374 / **0.70** |

   **Rows are sorted softest first, and `solref` is labelled by SINK because the
   raw pair reads backwards.** `dampratio` appears only in the stiffness term,
   as `1/dampratio^2`, so dropping it 1.0 → 0.30 makes the contact ~10× STIFFER,
   not softer. `timeconst` is the honest softness knob — it sets damping and
   stiffness together, sink going as `timeconst^2`.

   What it says:

   - **Soft contacts are fine.** Survival is 1.00 from 13 mm of sink all the way
     down to the shipped 0.39 mm, and score varies by 0.08 across a 34× span.
   - **The cliff is at the STIFF end**, and only there: `[0.005, 0.30]` —
     0.04 mm of sink — drops survival to 0.70. That is the corner the drop test
     points at, because a stiff contact against unchanged damping is what
     bounces.
   - **Two different rows tie at the best score (0.719)** — `[0.005, 0.50]` and
     `[0.020, 0.30]`, at sinks of 0.11 and 0.52 mm. So sink alone does not
     determine the outcome; damping matters independently, which is exactly why
     the negative `(-stiffness, -damping)` form is the one to measure into.

   Read the table as *how the current policy copes*, not as what is achievable:
   nothing has ever trained over this axis. A run with `dampratio_range`
   uncommented is the experiment that would flatten it.

   **Friction is the flat axis for survival, but not inert.** Across mu 0.5–2.0
   at the shipped contact, whole-grid score moves 0.630 → 0.663 → 0.643 and
   survival never leaves 1.00 — but the `hold` family's drift rises monotonically
   1.685 → 1.994 → 2.275 m, i.e. **more grip means more wander, not less**. The
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
   modelled bandwidth. Not yet quantified.
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
