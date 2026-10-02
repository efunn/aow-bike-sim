# Floors, and the contact model that has to cover all of them

> **Status: ACTIVE, and the measurement has not started.** Opened 2026-09-09.
> The plan is to take the rig onto **several real floors**, run the same three
> bench tests on each, and keep every surface's numbers — so the SPREAD across
> floors becomes a randomization range in training rather than a single floor
> being picked and hoped for.
>
> Nothing is measured yet: every field in
> `docs/measurements/contact-measurements.yaml` is still 0.0, and
> `floor-measurements.yaml` beside it is an empty sheet per surface.
>
> This doc is the multi-floor layer. **The procedures are not restated here** —
> `docs/measurements/contact-protocol.md` is P0 / P0b / P1 and is correct and
> current. Read its CORRECTION before taking any reading.

---

## 1. Why the parameters read backwards, first

Get this wrong and every conclusion inverts. It has happened twice in this
project — once in the protocol doc (corrected 2026-08-09) and once in
`docs/status.md` (corrected 2026-09-09, after the wrong version had been acted
on).

With a **positive** `solref = (timeconst, dampratio)` MuJoCo forms

```
b = 2 / (d_width · timeconst)                       ← damping
k = d(r) / (d_width² · timeconst² · dampratio²)     ← stiffness
```

- `timeconst` sets **damping and stiffness**. It is the honest softness knob.
- `dampratio` sets **stiffness only**, as `1/dampratio²`. It does not appear
  in `b` at all.

So **`dampratio` 1.0 → 0.30 makes the contact ~10× STIFFER**, and it bounces
because the stiffness rose while the damping did not. The name says the
opposite of what it does.

Consequences worth carrying:

- A **static** reading fixes only the PRODUCT `timeconst · dampratio`. P0 alone
  cannot separate them; P0 and P1 must be solved jointly.
- `contact_solimp.dmin` is a **third** unknown on top of that pair.
- **Always label a solref by its SINK in mm**, never by the raw pair. Sink is
  unambiguous; the pair is not. `analysis/floor_sweep.py` does this and sorts
  its rows by it for the same reason.

**The fix is the negative form.** `solref = [-stiffness, -damping]` is read by
MuJoCo directly, is what a system ID produces, and is what its own docs
recommend for identification. It decouples the two, and it shrinks the `dmin`
confound from 2.3× to 6%. **Measure into the negative form; do not fit the
positive pair.**

---

## 2. How the contact got where it is

Reconstructed from git 2026-09-09. Sink is the rear wheel with the bike at
rest on its own weight (~5.8 N on the rear), the whole bike settled
(`analysis/contact_calibration.rest_sink`, 2026-10-01). The "stiffer" factors
are the formula's `k`. The sink moves less than `k`, because `solimp`
stiffens the contact with depth.

| when | commit | `contact_solref` | sink | change |
|---|---|---|---|---|
| before 08-08 | — | *(unset)* → MuJoCo stock `[0.02, 1]` | 0.767 mm | inherited, invisible |
| 2026-08-08 | `7fcc160` | `[0.005, 1.0]` | **0.148 mm** | **~16× stiffer** |
| 2026-08-09 | `0efac06` | `[0.005, 0.5]` | 0.038 mm | a further ~4× stiffer |
| 2026-08-10 | `511421f` | `[0.005, 1.0]` | **0.148 mm** | reverted, "(working)" |
| 2026-08-14 | `6f74312` | unchanged | — | `contact_solimp` written down explicitly |
| now | — | `[0.005, 1.0]` | 0.148 mm | net **~16× stiffer than stock** |

Until 2026-10-01 this column read 3.803 / 0.390 / 0.108 mm. That came from a
posed curve (bike lowered to a height, force read from one `mj_forward`)
evaluated at 10 N of REAR force, about twice the rear's real load. It
over-read the sink: posing is not a resting state.

**Yes, the physics was made stiffer — twice, and it stuck once.** The 08-08
change was the deliberate one: at the stock `[0.02, 1]` the rear wheel sank
2.1–2.8 mm under a hold, a fifth of the 11 mm roller radius, near-identically
across five different trained policies. A number that is the same for five
controllers that behave differently is a property of the contact, not the
controller. The 08-09 excursion to `dampratio 0.5` lasted one day.

`contact_solimp` has never been changed. `6f74312` wrote MuJoCo's stock
`[0.9, 0.95, 0.001, 0.5, 2.0]` into the file explicitly — a physics no-op whose
point was to stop it being an invisible inherited choice. It is still
`source: GUESS`, and `dmin` is not negligible: 0.9 → 0.5 changes the rear's
sink at rest by **3.6×** (0.148 → 0.527 mm, settled, 2026-10-01; the posed
curve said 2.3×), which confounds both bench tests.

### Two policies DID train at 0.5, and the names came out inverted

The one-day window is visible in `moves/`:

| move | added | `contact_solref` then |
|---|---|---|
| `general_rl_smooth_stiff` | `0efac06`, 08-09 | **`[0.005, 0.5]`** |
| `general_rl_smooth_bouncy_lat` | `79651b0`, 08-09 | **`[0.005, 0.5]`** |
| `general_rl_glide_og` | `a3577f0`, 08-10 | `[0.005, 1.0]` |
| `general_rl_glide_pitch_og` | `8c9cb10`, 08-10 | `[0.005, 1.0]` |

`79651b0`'s subject says it outright: *"new move with bouncy solref
[0.005, 0.5]"*.

**`_stiff` and `_bouncy_lat` were trained at the SAME value.** The names are
inverted because the sign of the parameter was misunderstood at the time: at
fixed `timeconst`, stiffer IS bouncier, because the stiffness rose while the
damping did not. The reversal in §1 is not a hypothetical — it is sitting in two
export names from a month ago, and it is why they read as opposites.

Neither is a current policy: both predate the `plant_digest` field, so they
carry none, and neither is `control.general_move`. Everything in flight —
`general_rl_odo_ahrs`, `..._rand2`, `..._cmd_curriculum2b` — carries
`e1ec36bfa670217e`, i.e. dampratio **1.0**.

**Randomization over the contact has never happened at all.** `solref_frac` and
`dampratio_range` are commented out in all seven configs that mention them. Two
policies trained at a different FIXED value; none has ever seen the axis vary.

### The tension worth knowing before re-testing 0.5

`dampratio 0.5` was reverted on 08-10 as not working — chosen from teleop feel
plus the slow-mo study, **never measured**. But the eval grid now prefers it:
`general_rl_odo_ahrs` scores **0.719** at `[0.005, 0.50]` against **0.663** at
the shipped `[0.005, 1.00]`, and that policy trained at 1.0 (confirmed by
`plant_digest e1ec36bfa670217e`, which moves if dampratio moves).

So the preference is **not** "it trained there". A value rejected on feel is
preferred by the score, for a policy that never saw it. Do not resolve that by
argument — it is a measurement question, and the floor tests answer it.

---

## 3. What the sim currently says about floors

`python analysis/floor_sweep.py` — sweeps friction × contact and runs the full
20-command eval grid at each point. Read-only; touches no config and no
`moves/`. Measured 2026-09-09, `general_rl_odo_ahrs`, mu 0.9:

Sink column relabelled 2026-10-01: the rear wheel at rest, whole bike
settled, as in §2. The scores are unchanged. The posed labels were 13.17 /
3.80 / 1.08 / 0.52 / 0.39 / 0.04 mm, in the same order.

| sink | `contact_solref` | score / survival |
|---|---|---|
| 1.835 mm | `[0.020, 2.00]` | 0.594 / 1.00 |
| 0.767 mm | `[0.020, 1.00]` | 0.657 / 1.00 |
| 0.380 mm | `[0.005, 2.00]` | 0.673 / 1.00 |
| 0.209 mm | `[0.020, 0.30]` | **0.719** / 1.00 |
| **0.148 mm** | **`[0.005, 1.00]` — ships** | 0.663 / 1.00 |
| 0.014 mm | `[0.005, 0.30]` | 0.374 / **0.70** |

- **Soft is the safe direction.** Survival is 1.00 from 1.8 mm of sink down to
  the shipped 0.15 mm; score varies 0.08 across a 12× span of sink.
- **The cliff is at the STIFF end and only there** — 0.014 mm of sink drops
  survival to 0.70.
- **Sink alone does not determine the outcome.** `[0.005, 0.50]` and
  `[0.020, 0.30]` tie at 0.719 with sinks of 0.038 and 0.209 mm, so damping
  matters independently of stiffness. Another argument for the negative form.

**Friction is flat for survival but not inert.** Across mu 0.5–2.0 the
whole-grid score moves 0.630 → 0.663 → 0.643 and survival never leaves 1.00 —
while the `hold` family's drift rises **monotonically** 1.685 → 1.994 → 2.275 m.
**More grip means more wander**, because the policy uses slip as a brake and a
grippier floor converts more of its sawing into travel. The whole-grid score
cannot see this: `hold` is 1 command of 20 through a geometric mean. Use
`--by-family`.

    python analysis/floor_sweep.py
    python analysis/floor_sweep.py --mu 0.5 0.9 1.4 2.0 --dampratio 1.0 --by-family
    python analysis/floor_sweep.py --timeconst 0.005 0.020 --dampratio 0.3 1.0 2.0

Read all of it as *how the current policy copes*, not as what is achievable:
**nothing has ever trained over the contact axis.** `solref_frac` and
`dampratio_range` sit commented out in `config/rl_general.yaml`.

---

## 4. The three ground contacts are now separate — LANDED 2026-09-09

`roller` → ground, `front_tire` → ground and `righting` → ground each carry
their own contact parameters. **Shipped as a bit-exact no-op**: all three
resolve to the same globals, so no physics moved and no export became
provisional.

### The mechanism is `geom_priority`, not separate solref values

Separate values do not do what they look like they do. Measured 2026-09-09:

| | combination rule |
|---|---|
| `solref` / `solimp` | solmix-weighted average |
| `friction` | elementwise **max** |

    floor [0.001, 1.0]  x  roller [0.010, 0.3]  ->  contact [0.0055, 0.65]

An exact average — so *"compliant TPU on rigid wood"* cannot be expressed by
setting the two geoms independently. You get the mean, never the softer one.

`geom_priority` is the knob that does: where two geoms differ in priority, the
**higher one dictates every contact parameter** and no combination happens. Each
of the three parts is given priority 1 against the floor's 0. (`solmix` also
works — roller `solmix 1000` returns `[0.009991, 0.300699]`, its own value — but
priority is exact rather than asymptotic.)

### Why it is bit-exact today

With every part on the same values, dictation and combination agree:
`max(x, x) == x` and `avg(x, x) == x`. Verified at trajectory level over a
3000-step **driven** rollout — driven because a bike settling under gravity
barely loads the contact:

| | max abs Δqpos vs the pre-split model | bitwise identical |
|---|---|---|
| split, identical params | `0.000e+00` | **yes** |
| split, `front_tire` overridden | `1.164e-01` | no |

`plant_digest e1ec36bfa670217e` and `design_digest 2db6c647ff3a2d59` unmoved;
`pytest -m deploy` green.

### Using it

Overrides are **optional and absent from `bike_params.yaml` on purpose** —
adding the block moves `plant_digest` and makes every export in `moves/`
provisional. The code path exists now; changing a value is a separate,
deliberate act.

```yaml
sim:
  contact_parts:
    front_tire: {solref: [0.002, 1.0], friction_sliding: 1.3}
    righting:   {solref: [0.001, 1.0]}
```

Keys: `solref`, `friction_sliding`, `friction_torsional`. Anything omitted falls
back to the global. An unknown part name **raises** rather than silently falling
back — a typo that quietly used the globals would look like it worked.

Which geoms are which: **roller** = the 16 `roller_*` cones. **front_tire** = 1.
**righting** = `wing_*`, `wing_*_leg`, `wing_*_foot`, `wing_*_skid*`,
`swing_wing_*`, `swing_*_plate`, `case_skirt_*`, `case_upper_*`. Deliberately
NOT righting: `stick_*` (the hockey striker) and `bumper_*` (retired) — both
keep the global contact.

`tests/test_contact_parts.py` (marker `contact`, 9 tests) pins both halves: the
trajectory is bitwise identical to the pre-split model, **and** an override
actually changes it. The second matters as much as the first — without it, a bug
that dropped `contact_parts` entirely would pass by being a perfect no-op.

### What it does NOT do

It does not make a per-geom split necessary for the floor work. One global pair
still models any single surface correctly, and randomizing over measured
per-surface pairs still needs no split. What it buys is **fidelity**: a rigid
printed wing striking the ground is no longer forced to have the compliance of a
TPU roller, and the front tire — whose contact carries the odometry estimator's
whole lateral channel — is no longer forced to be the same material as the rear.

## 5. The per-floor protocol

> **Open question (2026-09-24):** the incline test below needs a board that
> tilts, which a real floor is not. The draft
> [pre-assembly-bench-checklist.md](pre-assembly-bench-checklist.md) proposes
> per-floor alternatives (F1-F3, friction from the drive's traction limit).
> They are untested; this section stands until one holds up on hardware.

Per surface, three tests, all from `contact-protocol.md`. None needs printing;
all need a weight, a caliper, a phone that shoots slow-mo, and a board that
tilts.

| test | gives | notes |
|---|---|---|
| **P0** static load-deflection | the stiffness half | 2–3 loads, not one — the contact is not linear. Reference the **axle**, not the tyre crown, to keep frame flex out |
| **P0b** incline slide | `mu = tan(theta)` | cheapest test in the project. **Block the rotation**, or you measure rolling resistance |
| **P1** drop rebound | the damping half | `e = sqrt(h1/h0)` off a 240 fps clip, on the bare rolling chassis |

Solve P0 and P1 **jointly** per surface, then express the result as
`(-stiffness, -damping)`.

**Record the surface itself, not just the numbers** — material, backing (a mat
on concrete is not the same as a mat on joists), temperature, and whether it
was clean. A floor that cannot be identified later cannot be re-tested.

### The data sheet

`docs/measurements/floor-measurements.yaml` — one block per surface, same field
names as `contact-measurements.yaml` so the two can be read by one parser.
`contact-measurements.yaml` stays the canonical single-surface sheet for the
reference floor; this one is the per-surface set.

---

## 6. What the spread is for

**This is the point of testing several floors rather than choosing one.**

The mean across surfaces sets the nominal; **the SPREAD sets the randomization
range**. `contact-measurements.yaml` already carries a field for exactly this —
`friction_frac_suggested`, annotated *"from the SPREAD of p0b, not the mean."*

Today `randomization.friction_frac: 0.2` spans mu 0.72–1.08 around a **guessed**
0.9, and the contact axis is not randomized at all. After the floor set:

```yaml
randomization:
  friction_frac: <from the mu spread across surfaces>
  solref_frac: <from the stiffness spread>          # currently commented out
  dampratio_range: [<min>, <max>]                   # currently commented out
```

A policy trained across the measured span drives on **any** of the floors,
which is strictly better than picking the best one and hoping. And it retires
ranked risk #1 in `docs/status.md` — *no trained policy has ever seen contact
variation* — which is the most likely single cause of a policy that works in
sim and not on the floor.

**The contact randomization does not wait for these measurements.** It depends
on no number: turning `solref_frac` and `dampratio_range` on with a plausible
span is available today and decouples the largest sim-to-real risk from the
build timeline. The measurements make the span *right*; they are not what makes
it *possible*. If a long unattended run grid is going to happen while the bike
is being built, this is what it should be doing.

---

## 7. Order of work

0. ~~**Split the three ground contacts**~~ **DONE 2026-09-09** (§4), as a
   bit-exact no-op. Values are still identical; setting one is a later,
   deliberate change.
1. **Turn on contact randomization** with a plausible span — waits for no
   measurement, and is what a long unattended run grid should be doing while
   the bike is built.
2. **Measure floor 1** (the reference surface) end to end: P0, P0b, P1.
3. **Repeat per surface**, recording each into `floor-measurements.yaml`.
4. **Re-express as negative solref**, jointly per surface.
5. **Set the randomization ranges from the measured spread**, retrain, and
   re-derive `MIN_FIT_R2` from the measurement rather than carrying 0.93
   forward.

Step 1 is sim-side and can happen now. Steps 2–4 are bench work needing only
things already on hand. The split (§4) is done and changed nothing; deciding
what values the three parts should actually get is a separate question that the
bench tests inform.

---

## Related

- `docs/measurements/contact-protocol.md` — the procedures, and the CORRECTION.
  Correct and current; re-verified 2026-09-08.
- `docs/measurements/contact-measurements.yaml` — the single-surface sheet.
- `docs/measurements/floor-measurements.yaml` — the per-surface set.
- `analysis/floor_sweep.py` — friction × contact against the eval grid.
- `analysis/contact_calibration.py` — static and drop curves; read a bench
  number straight off a table instead of bisecting by retraining.
- `docs/plans/aow-contact-approximations.md` — the surrogate survey.
- The floor sweep table and its corrections: the last section below (moved
  from `docs/status.md` 2026-10-02).

---

## From status.md: the floor sweep as risk #1, and the contact split

*Moved verbatim from `docs/status.md` on 2026-10-02, when that file was rewritten as a short navigation layer. "Above", "below", "Health" and numbered "What to do next" items refer to status.md as it was then (`git show d0dffeb:docs/status.md`).*

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

   | rear sink at rest | `contact_solref` | 09-09, no AHRS | **09-11, tm151** |
   |---|---|---|---|
   | 1.835 mm | `[0.020, 2.00]` | 0.594 / 1.00 | 0.600 / 1.00 |
   | 0.767 mm | `[0.020, 1.00]` | 0.657 / 1.00 | 0.660 / 1.00 |
   | 0.380 mm | `[0.005, 2.00]` | 0.673 / 1.00 | 0.679 / 1.00 |
   | 0.209 mm | `[0.020, 0.30]` | **0.719** / 1.00 | 0.651 / 0.95 |
   | **0.148 mm** | **`[0.005, 1.00]` — ships** | 0.663 / 1.00 | **0.686** / 1.00 |
   | 0.038 mm | `[0.005, 0.50]` | 0.719 / 1.00 | 0.616 / 0.90 |
   | 0.014 mm | `[0.005, 0.30]` | 0.374 / **0.70** | **0.175** / **0.35** |

   **Sink column relabelled 2026-10-01 (settled, `contact_calibration.rest_sink`);
   the scores did not move.**

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

   - **Soft contacts are fine.** Survival is 1.00 from 1.8 mm of sink all the way
     down to the shipped 0.15 mm, and score varies by 0.09 across a 12× span.
     Unchanged by the correction.
   - **The cliff is at the STIFF end**, and only there: `[0.005, 0.30]` —
     0.014 mm of sink — drops survival to **0.35**, not the 0.70 first
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
