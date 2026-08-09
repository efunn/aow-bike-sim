# Contact Model Measurement Protocol

Companion data sheet: `contact-measurements.yaml` (same section numbering).
Extends §7 of `omni-wheel-protocol.md`, whose item 6 already flagged that
contact softness needs calibrating; this is that item, worked out.

Everything here calibrates **one config line**:

```yaml
sim:
  contact_solref: [0.005, 1.0]   # [timeconst_s, dampratio]
```

Status: **the pair is uncalibrated.** `0.005` was set 2026-08-08 by picking a
number ~16x stiffer than MuJoCo's stock `[0.02, 1]`, not by measuring; `1.0`
has never been anything but MuJoCo's default. Both are testable on the bench
with a weight, a caliper and a phone.

## Why this is worth doing at all

The rear wheel sinking into the floor is not a rendering artefact, it is the
contact model, and at the stock `[0.02, 1]` it sank **2.1–2.8 mm** under a hold
command — a fifth of the 11 mm roller radius, and near-identical across five
different trained policies. When a number is the same for five controllers
that behave differently, it is not a property of the controller.

The two entries in `solref` are **separately identifiable by two separate
experiments**, which is what makes this cheap:

| | fixes | bench test | rig needed |
|---|---|---|---|
| `timeconst` | contact **stiffness** | static load-deflection | weight + caliper |
| `dampratio` | contact **damping** / restitution | drop test | drop height + slow-mo |

Model-side companion: `analysis/contact_calibration.py` computes both curves,
so a bench number can be read straight off a table instead of bisected by
retraining.

```sh
python analysis/contact_calibration.py
python analysis/contact_calibration.py --load-kg 4.5 --drop-mm 35
```

---

## Priority

**Do them in this order.** The list is ordered by information per unit of
effort, not by how interesting the test is.

### P0 — Static load-deflection (do this one first)

The highest-value measurement in this document and the one that needs no rig.

Set a known weight on top of the wheel assembly, axis horizontal, resting on
the target floor material. Measure how far the axle drops relative to
unloaded. Repeat at 2–3 loads so the curve's shape is visible, not just one
point — the contact is not linear.

| load | model prediction at `timeconst` = |
|---|---|
| | 0.020 → 0.010 → 0.005 → 0.0035 → 0.002 |
| bike weight, 10.0 N | 3.60 / 1.02 / 0.37 / — / 0.07 mm |
| 2x bike weight, 20.0 N | 10.73 / 2.81 / 0.81 / — / 0.17 mm |
| 4.5 kg, 44.1 N | bottoms out / 7.17 / **1.95** / **0.99** / 0.39 mm |

A first pass (2026-08-08, guided by hand, load ~4.5 kg) put the deflection
"in the mm range, say 1 mm, maybe more". That is already enough to kill
`0.020` outright — it sinks 3.6 mm under the bike's *own* weight — but not
enough to separate `0.005` (1.95 mm) from `0.0035` (0.99 mm). **A careful
reading of this one number closes the question.**

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

### P1 — Drop test, rebound height

Fixes `dampratio`, and the current value is **known to be wrong**: `1.0` is
critical damping, and a critically damped contact **cannot bounce at all**.
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
"compliant TPU on rigid wood" *cannot* be expressed by setting the two geoms
independently; you get the average, not the softer one. To make the roller
dominate, raise `geom_solmix` on the roller geoms.

For now this does not matter: essentially all the compliance is the TPU, so a
single global pair calibrated from a TPU-on-wood test **is** the material
value. Splitting only earns its keep when the surface changes.

---

## What to do with the numbers

1. Put raw readings in `contact-measurements.yaml`.
2. Read the matching `timeconst` / `dampratio` off the tables above, or re-run
   `analysis/contact_calibration.py` with your load and drop height.
3. Update `sim.contact_solref` in `config/bike_params.yaml`.
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
