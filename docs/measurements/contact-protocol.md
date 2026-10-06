# Contact Model Measurement Protocol

> **Status: CORRECT and CURRENT — but nothing has been measured.** Every field in
> `contact-measurements.yaml` is still 0.0, and that is the only thing wrong here.
>
> The document was fixed on 2026-08-22 (`c1b8a8d`): the CORRECTION at the top is
> in place, the withdrawn "timeconst 0.020 is ruled out" claim is marked
> withdrawn, and **the negative `solref` convention is written down as THE PLAN
> rather than an option**. Its §P0 table was re-checked against
> `analysis/contact_calibration.py` on 2026-09-08 and agrees to within the
> timestep change (3.60 vs 3.78, 0.37 vs 0.389, 1.95 vs 1.982 mm). **Those
> model numbers were all from a posed curve that is not a resting state, and
> were replaced 2026-10-01** by a settled whole-bike curve (§P0 table): 0.57 mm
> at 44 N on the rear, not 1.95.
> `docs/status.md`'s ranked risk #8 — "the contact protocol docs are stale" — is
> itself the stale claim, and is retired in the status rewrite.
>
> **Running this on SEVERAL floors?** `docs/plans/floors-and-the-contact-model.md`
> is the layer on top: same three tests, once per surface, recorded in
> `floor-measurements.yaml`, with the spread across surfaces setting the
> randomization range. It also carries the solref/solimp history and the
> per-geom blocker.
>
> **Read §P0's CORRECTION before taking any reading**: `timeconst` sets damping
> AND stiffness, `dampratio` sets stiffness only, so a static reading fixes only
> the PRODUCT and P0 and P1 must be solved jointly — unless you switch to
> negative `solref` first, which is the point of switching.

Companion data sheet: `contact-measurements.yaml` (same section numbering).
Extends §7 of `omni-wheel-protocol.md`, whose item 6 already flagged that
contact softness needs calibrating; this is that item, worked out.

This document covers the contact in **both directions**, which are separate
measurements on separate config lines and do not constrain each other:

```yaml
sim:
  contact_solref: [0.005, 1.0]   # NORMAL: [timeconst_s, dampratio]   -> P0, P1
  friction_sliding: 0.9          # TANGENTIAL: Coulomb mu             -> P0b
  friction_torsional: 0.005      # TANGENTIAL: spin about the normal  -> P0b
```

Status: **all four numbers are uncalibrated.** `0.005` was set 2026-08-08 by
picking a number ~16x stiffer than MuJoCo's stock `[0.02, 1]`, not by
measuring; `1.0` has never been anything but MuJoCo's default; the two friction
values are marked `GUESS` in `bike_params.yaml` and always have been. All of
them are testable on the bench with a weight, a caliper, a phone and a board
that tilts.

## Why this is worth doing at all

The rear wheel sinking into the floor is not a rendering artefact, it is the
contact model, and at the stock `[0.02, 1]` it sank **2.1–2.8 mm** under a hold
command — a fifth of the 11 mm roller radius, and near-identical across five
different trained policies. When a number is the same for five controllers
that behave differently, it is not a property of the controller.

**CORRECTION (2026-08-09).** An earlier version of this document claimed the
two entries are separately identifiable, one experiment each. **That was
wrong.** With a positive `solref = (timeconst, dampratio)` MuJoCo forms

```
b = 2 / (d_width * timeconst)                       <- damping
k = d(r) / (d_width^2 * timeconst^2 * dampratio^2)  <- stiffness
```

The names mislead. `timeconst` enters **both** — it is the only thing setting
damping, and it also sets stiffness as `1/timeconst^2`. `dampratio` sets
**stiffness only**, as `1/dampratio^2`; it does not appear in `b` at all. It
is the ratio of actual to critical damping *for the resulting stiffness*, so
lowering it 1.0 → 0.5 leaves damping alone and makes the contact **four times
stiffer**, which is what makes it underdamped and bouncy. Verified against
this model, rear wheel at rest on the bike's own weight (settled,
2026-10-01): the sink falls 3.9x for 1.0 → 0.5 and 10.8x for 1.0 → 0.3,
against the 4x and 11.1x the formula predicts. Under load the ratio shrinks
(2.2x at 44 N), because `solimp` stiffens the contact with depth.

**Neither form of `solref` is a stiffness in N/m.** At rest, MuJoCo's sink at a
given force scales with the masses compiled around the contact
(`body_invweight0`). On a wheel-on-a-slide rig at 44 N
(`analysis/static_contact_sim.py`, 2026-10-01), the front tire sank 0.447 /
0.072 / 0.010 mm with a 0.01 / 0.5 / 4 kg carriage, and 0.660 / 0.142 / 0.020
with a negative solref. On the bike, doubling the `GUESS` chassis mass raised
the rear load 42% but its sink only 31%. So fit on the bike's own model, and
**refit after the weighing**.

**Consequence for the bench tests.** A static load-deflection reading
constrains the **product** `timeconst * dampratio`, not `timeconst` alone, so
it cannot fix either number by itself. The static and drop tests have to be
solved **jointly**. Concretely: a 4.5 kg reading of "about 1 mm" on the rear
implies `timeconst ≈ 0.009` if you assume `dampratio 1.0` — which is what the
config ships — and `≈ 0.018` at 0.5, a factor of two coming from an
assumption rather than a measurement (settled, 2026-10-01; the posed curve
said 0.0035 and 0.0075–0.010). (An earlier version of this said 0.5 was
shipping. It was tried and reverted; the config has been at 1.0 since.)

**THE NEGATIVE CONVENTION IS THE PLAN, not an option.** Measure the contact,
then express it as `solref = (-stiffness, -damping)` rather than fitting the
`(timeconst, dampratio)` pair. The whole difficulty above — that a static
reading constrains only the PRODUCT, so the two tests have to be solved jointly
— is an artefact of the positive parameterisation. A system ID produces
stiffness and damping directly, and the negative form takes them directly.

Note what this does NOT fix: the LQR's linear fit. `MIN_FIT_R2` was lowered
0.98 → 0.93 because no dampratio clears the old bar, and the fit gets WORSE as
the contact gets more realistic (0.9412 at the shipped 1.0, 0.8148 at the ~0.30
the drop test implies). Switching parameterisation changes how the number is
expressed, not how nonlinear the contact is. Re-derive `MIN_FIT_R2` from
whatever the measured contact gives.

**The original note on the convention.** MuJoCo reads a negative
`solref` as `(-stiffness, -damping)` directly, and its own docs recommend that
form for system identification. Then the static test gives stiffness, the drop
test gives damping, and neither contaminates the other — which is the clean
version of what this document originally claimed. Worth switching to before
the bench session rather than after.

| | enters | bench test | rig needed |
|---|---|---|---|
| `timeconst` | damping **and** stiffness | both, jointly | weight + caliper |
| `dampratio` | stiffness only (`1/dampratio^2`) | both, jointly | drop + slow-mo |

Model-side companion: `analysis/contact_calibration.py` computes both curves,
so a bench number can be read straight off a table instead of bisected by
retraining.

```sh
python analysis/contact_calibration.py
python analysis/contact_calibration.py --load-kg 4.5 --drop-mm 35
```

---

## Priority

**Do them in this order**, with one exception: P0b measures the TANGENTIAL
direction and is independent of everything else here, so it can be done first,
last, or while the weights are already out. The list is ordered by information
per unit of effort, not by how interesting the test is.

### P0 — Static load-deflection (the first of the normal-direction pair)

The highest-value measurement in this document and the one that needs no rig.
It pins the PRODUCT `timeconst * dampratio` (see the correction above), so the
numbers below are quoted at a stated `dampratio` and are not readings of
`timeconst` on their own.

Set a known weight on top of the wheel assembly, axis horizontal, resting on
the target floor material. Measure how far the axle drops relative to
unloaded. Repeat at 2–3 loads so the curve's shape is visible, not just one
point — the contact is not linear.

Model predictions below assume `dampratio = 1.0`, which is what the config
ships. At 0.5 the same deflection implies a `timeconst` roughly 2x larger.
Rear wheel, whole bike settled with the load on the rear axle
(`contact_calibration.static_curve`, 2026-10-01), at the rest pose's roller
phase (an axle ~2° from straight down):

| rear load | model prediction at `timeconst` = |
|---|---|
| | 0.020 → 0.010 → 0.005 → 0.0035 → 0.002 |
| at rest, its own ~5.8 N | 0.77 / 0.41 / **0.15** / 0.07 / 0.02 mm |
| 10 N | 1.04 / 0.54 / 0.24 / 0.13 / 0.04 mm |
| 20 N | 1.77 / 0.76 / 0.39 / 0.24 / 0.08 mm |
| 4.5 kg, 44.1 N | 2.49 / 1.18 / **0.57** / **0.41** / 0.17 mm |

The roller phase moves the 0.005 row by about ±0.1 mm at 44 N: 0.44 mm with
the two big ends of one axle bridging, 0.64 mm on a single roller edge
(`static_curve(..., axle_deg=...)`).

**This table replaced a posed one** (bike lowered to a height, force read from
one `mj_forward`), which over-read the sink above the wheel's own load: 1.95
for 0.005 at 44.1 N, and "10 N bike weight" was really 10 N on the rear,
about twice its share.

A first pass (2026-08-08, guided by hand, load ~4.5 kg) put the deflection
"in the mm range, say 1 mm, maybe more". The posed table read that as killing
`0.020`; settled, 0.020 sinks 2.49 mm at 44.1 N, which the hand check makes
unlikely but does not rule out. It also lands between `0.010` (1.18 mm) and
`0.005` (0.57 mm). **A careful reading of this one number closes the
question**, at a known roller phase.

Method notes:
- Load through the axle, not the tyre crown, so the reading is contact
  deflection and not frame flex.
- Measure axle height, not a contact patch: the patch is where the
  measurement is ambiguous.
- Unloaded reference on the same surface, same spot — the ripple across roll
  phase is ~0.6 mm of real geometry (see §1 of the wheel protocol) and will
  otherwise be read as compliance.
- Note the roll phase used, and prefer a repeatable one (single roller in
  contact). Phase is a real confound here, worth §P2 on its own.

### P0b — Pull test → the friction coefficient; tread print → torsional

> **A rough plan, not a spec (rewritten 2026-10-05).** The rig gets built and
> recorded first; details here follow it. Record what the rig actually was
> alongside the numbers, and fix this section afterwards.

It calibrates `sim.friction_sliding` (0.9, GUESS) and estimates
`friction_torsional` (0.005, GUESS). It is independent of P0 and P1: normal
and tangential directions are orthogonal, so do them in any order.

**Replaces the incline slide.** The incline (`mu = tan(theta)`) needs a board
that tilts, which a real floor is not. A pull runs on the floor as it lies.

**Setup.**
- The drop rig's wheel mount, wheel **locked**: front through the spokes,
  rear by tightening the axle. A wheel free to roll measures rolling
  resistance instead. That is the one way to get a confidently wrong answer.
- On the rear, the rollers are not locked, only held by the differential's
  detent (`drivetrain_model.detent`). Observed: the material slides before the detent
  lets go. So the rolling direction is clean, and the roller slop does not
  enter it, because a pull along the tire drags each roller along its own
  axis.
- Weight added over the wheel. Light blocks either side keep the rig upright.
  They **guard, they do not hold**: if the rig leans on a block during a pull,
  that block's friction is in the reading.
- **Normal force: the whole weighted rig on a kitchen scale, wheel only on
  the scale**, blocks off it, in the same stance as the pull. That reading is
  what the wheel presses into the floor.
- **Pull force: a hand spring scale** (2 kg), pulled slowly, horizontally and
  low (axle height or below), along the tire.

**Loads and repeats.**
- Three loads on the wheel: about 0.3, 0.6 and 1.0 kg. The model's bike is
  ~1.0 kg, so roughly 0.5 kg a wheel (computed, not weighed); these bracket
  it. Under ~1.5 kg, the 2 kg scale reaches mu 1.3.
- 3-5 pulls at each.
- Read the **steady value while sliding** (kinetic). The breakaway peak
  (static) is hard to catch by eye, so film the scale if you want it.
  MuJoCo has one coefficient; if the two differ, prefer kinetic, because the
  contact spends its time sliding, not breaking away.
- Same surface as P0/P1, and clean. Rubber picks up dust and its mu falls.

**Analysis: mu is the SLOPE of pull force against wheel load**, not one
ratio.
- A constant drag (a block brushing, the scale's own friction) lands in the
  intercept and drops out.
- If the three points curve, grip depends on load. MuJoCo assumes it does
  not, so that is a finding.
- The spread across pulls sets `randomization.friction_frac`, currently 0.2
  about 0.9. So every policy in `moves/` has seen only mu 0.72-1.08.
- A 2 kg spring scale reads to ~20-50 g. At 1 kg that is +-0.02-0.05 in mu.

**Directions.**
- **Rolling direction only, both wheels.** A sideways pull tips a single
  wheel: its contact is millimetres wide. A block that catches it then takes
  part of the pull, and the scale over-reads.
- MuJoCo cannot use a sideways number anyway. A contact `<pair>` takes two
  tangential frictions, but checked on MuJoCo 3.10 (a box on a plane, 0.2 and
  1.0, pushed at mu 0.6): the two directions stay on the floor's x and y
  axes at box yaw 0, 45 and 90 deg. A bike that turns would swap its forward
  and sideways grip. Wheel-relative anisotropy would need a custom contact.
- **Front tire, leaned (optional).** The patch moves from the 10 mm flat
  tread onto the shoulder as the wheel leans; `crown_radius` is a GUESS. A
  pull or a print at a measured lean angle is worth having even if the rig is
  finicky to set at an angle. Record the angle with it.

**Torsional friction, from a tread print.**
- In MuJoCo `friction_torsional` is a LENGTH. The contact resists spin up to
  `friction_torsional` x normal force. Checked on a sphere at condim 4: 0.8x
  that torque creeps at ~1 rad/s, 1.2x spins it up to ~30.
- For a patch of equivalent radius a: about (2/3) mu a with even pressure,
  or (3 pi / 16) mu a, about 0.59 mu a, with Hertz-like pressure.
- **Measure a.** Ink or chalk the tread, set the weighted rig down on paper
  at each of the three loads, and measure the print. Then
  a = sqrt(area / pi). Take mu from the pull.
- **Estimate before measuring:** front tire, 10 mm flat on a 51.25 mm
  radius, sinking 0.05-0.13 mm (dial, 2026-10-01). That gives a patch about
  6 x 10 mm, a ~ 4.4 mm, and **~0.0026 m at mu 0.9**, against the shipped
  0.005. The rear sits on one or two rollers, so its patch, and its value,
  should come out smaller.
- **Why not measure the twisting torque directly:** at 0.5 kg it is about
  0.013 N.m. On a 0.2 m lever that is a ~7 g pull, under a spring scale's
  resolution.
- Low priority either way. The risk is setting it too HIGH, not too low (see
  below).

#### Why this one matters, measured 2026-08-22

Slip here is the tangential velocity of the roller surface against the floor,
taken in the contact frame, not inferred from kinematics. Two regimes, and
they disagree:

| | steady crab | policy holding station |
|---|---|---|
| shipped, mu 0.9 | 2.6 mm/s | 35.1 mm/s |
| mu 0.4 | 2.6 | 85.2 |
| mu 1.4 | 2.4 | 29.1 |
| mu 2.0 | 2.2 | **15.9** |

**Under a steady crab `mu` does nothing** — the contact is nowhere near the
friction cone, so grip is not what limits it. **Under a hold it dominates**:
the policy's rapid reversals spike the tangential force, the cone is reached
intermittently, and 0.9 → 2.0 cuts slip by 55%. Any friction test that only
exercises steady motion will therefore conclude, wrongly, that the coefficient
does not matter.

Two knobs that look like they should help and do not, same hold case:

| | mean slip |
|---|---|
| shipped (`impratio` 10) | 35.1 mm/s |
| `impratio` 30 | 37.0 |
| `impratio` 100 | **45.6** — worse |
| `condim` 6 | 37.7 |
| `friction_torsional` 0.005 → 0.05 | 43% of crab travel lost |

MuJoCo's own guidance is to raise `impratio` to make contacts less slippery,
and that is right for numerical creep BELOW the friction limit — which is not
what is happening here. Raising it costs solver conditioning and buys nothing.
Stacked on mu 2.0 it made things slightly worse (15.9 → 16.8 mm/s). Torsional
friction actively fights the mechanism: it resists the roller spinning at the
patch, which is how the wheel crawls.

So: **`friction_sliding` is the knob, and it is the only one.**

### P1 — Drop test, rebound height

Constrains the pair jointly with P0. The shipped `1.0` was **known to be
wrong**: it is critical damping, and a critically damped contact **cannot
bounce at all**.
Simulated drop from 35 mm at the shipped setting produces zero rebounds. The
physical wheel audibly bounces two or three times, so this is a qualitative
mismatch, not a tuning disagreement.

Drop the wheel from a measured height onto the target surface; film at high
frame rate; read the apex of the first rebound. `e = sqrt(h1 / h0)`.

| `dampratio` | apexes from 35 mm | first-rebound `e` |
|---|---|---|
| 1.00 (shipped) | *no bounce* | — |
| 0.70 | 0.9, 0.2 mm | 0.16 |
| 0.50 | 3.0, 0.2 mm | 0.29 |
| 0.45 | 3.8, 0.4 mm | 0.33 |
| 0.35 | 6.0, 1.3 mm | 0.41 |
| **0.30** | **7.5, 1.9 mm** | **0.46** |
| 0.25 | 9.3, 3.0 mm | 0.52 |
| 0.20 | 11.6, 5.6 mm | 0.58 |

A first pass (2026-08-08, dropped 3–4 cm onto a smooth wood cutting board,
hands guiding but not pushing) reported: first rebound clearly visible,
second barely, 2–3 audible. That description lands on **`dampratio` ≈ 0.30**,
whose 7.5 mm and 1.9 mm apexes are exactly "one obvious, one barely".

Method notes:
- **Rebound height is the number, not the bounce count.** The simulated drop
  in the table is the whole bike, which has a second contact and can topple,
  so its bounce *count* is unreliable above ~3; the first apex is monotone in
  `dampratio` and is what to match. A wheel-alone drop is cleaner than the
  model's own whole-bike drop here.
- Release, don't throw. Any hand force at release changes `h0` and `e` reads
  wrong by the square root of the error.
- An IMU on the assembly is **not** needed. Apex ratio is all `e` requires,
  and a phone at 240 fps resolves a 7 mm rebound. Add the IMU only if impact
  *duration* becomes interesting, which it is not for `solref`.

### P2 — Roll-phase dependence

Drops and static loads at different contact phases: one roller, two adjacent
rollers, two rollers bridging a gap. This tests something different from P0
and P1 — not the material, but whether the 8-axle discretisation gives
angle-dependent stiffness, and how much of the sim's "bumpiness" is the real
chord geometry (§1 of the wheel protocol) versus contact softness.

Deferred, because `solref` is a single global pair and cannot express phase
dependence anyway. Its value is as a **check**: if measured stiffness varies
strongly with phase and the sim's does not, the mesh or the layout is wrong,
not `solref`.

### P3 — Surface dependence

Only if the bike is meant to run on something other than hard flat floor.

Note the mixing rule before designing this test: MuJoCo combines a contact
pair by **`solmix`-weighted average**, verified 2026-08-08 — floor `[0.001,
1.0]` against roller `[0.010, 0.3]` yields a contact of `[0.0055, 0.65]`. So
"compliant rubber on rigid wood" *cannot* be expressed by setting the two geoms
independently; you get the average, not the softer one. To make the roller
dominate, raise `geom_solmix` on the roller geoms.

For now this does not matter: essentially all the compliance is the rubber, so a
single global pair calibrated from a rubber-on-wood test **is** the material
value. Splitting only earns its keep when the surface changes.

---

## What to do with the numbers

1. Put raw readings in `contact-measurements.yaml`.
2. Read the matching `timeconst` / `dampratio` off the tables above, or re-run
   `analysis/contact_calibration.py` with your load and drop height. For P0b
   mu is the slope of pull force against wheel load.
3. Update `sim.contact_solref` and/or `sim.friction_sliding` in
   `config/bike_params.yaml`, and change the friction `source:` off `GUESS`.
4. Regenerate the deploy bundle — it is pinned to a params digest and
   `hw.state` raises rather than silently flying stale gains:
   `python -m aow_sim.export_deploy`.
5. Re-check that trained policies still survive: `python analysis/chatter.py`
   (the eval-grid table at the top). Changing `0.02` → `0.005` cost nothing —
   all five policies stayed at survive 1.00 and two improved — but that is not
   guaranteed for a change that also makes the floor bouncy.

## Open recommendation: randomize it

`DomainRandomizer` (`control/randomize.py`) perturbs body masses, the
tangential friction coefficient and actuator strength. It does **not** touch
`geom_solref` — the array is not even saved for restore. So contact stiffness
is the one uncertain parameter the policy gets no exposure to, and it is
currently the *least* well known of them.

A `solref_frac` alongside `friction_frac` would likely buy more transfer
robustness than pinning the nominal value exactly, and the two are
complementary: measure to get the centre, randomize to cover what the
measurement cannot resolve. Worth doing before the next long training run.

**For friction the randomizer already exists and its width is now the
question.** `friction_frac: 0.2` about a nominal 0.9 means every policy in
`moves/` has only ever seen **mu 0.72–1.08**. If P0b comes back near that
window, widen `friction_frac` and leave the nominal alone — cheaper than a
plant move and it buys robustness the exact value does not. If it comes back
at 1.5+, the nominal has to move, and that is a `params_digest` change and a
retrain: work the "Before changing a physical parameter" list in `CLAUDE.md`.

Either way, do NOT pick the value that looks best in `wheel_slowmo`. The
sensitivity table in P0b was measured precisely so that the number can be
argued from the bench instead of from the render.
