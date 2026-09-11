# Possible linkage mechanism for rocker wing deployment

> **Status: ACTIVE side project, and the live home for the righting mechanism.**
>
> **Direction (2026-09-08).** The mechanism has moved to an **asymmetric output
> from a symmetric layout**, away from the symmetric-output/asymmetric-layout
> form in `self-righting.md`. The asymmetric one is preferred on the geometry.
>
> **Open, and treated as unsettled: the dynamic / torque-output analysis.** The
> static τ(θ) story is not trusted enough to design against on its own. The plan
> is deliberately empirical — build something, then use MuJoCo to work out how to
> rock and leverage the bike upright within the real power output, rather than
> resolving it analytically first. Expect ad-hoc analyses and sim runs against
> this doc rather than a single optimisation pass.
>
> **Incomplete as a record.** The swing-linkage work of 2026-09-02/03 —
> three-angle optimisation (22x faster), the constraint configs, the 65 mm
> flat-deploy result, "vertical rest still does not pay", and the driveable
> Onshape sketch feature — was written up only in `docs/status.md`, and a later
> rewrite of that file dropped it. **WHICH CONFIG WON is now recorded here**,
> under "Which swing-linkage config won" — it was `_smaller`. The rest of that
> work is still not pulled down, and `status.md` no longer has it either, so
> anything not in this file is currently only in the git history.
>
> Tools: `analysis/wing_linkage.py`, `analysis/swing_linkage.py`,
> `analysis/swing_demo.py`, `src/aow_sim/cad_swing_linkage.py`. Figures in
> `analysis/plots/`. Station C of `first-physical-test.md` is the bench protocol
> for measuring τ(θ) on the real thing.

The current design uses a gear train to rotate the wings around a point near the base of the bike. Here, we explore the possibility of using a linkage mechanism (in effect, a 4-bar linkage for each wing, both driven by the same servo) to drive the wings instead. They still rotate around a similar point near the base of the bike.

# General organization

The wing XC330 sits at the midline of the bike (looking from the front/back), and some distance off the ground.

The first link from the XC330 extends away from its target wing, then the second link reaches back toward the wing, attaching at some point along its length. The next link is simply the distance from the attach point down to the pivot point. The final (fixed) link is a virtual one, between the main wing pivot and the XC330 centerpoint. Thus, forming a 4 bar linkage.

When viewed from the front, one linkage train will likely go up and over, and the other goes down and under. For example, the right side first link goes UP and LEFT, and the right side second link goes DOWN and RIGHT; the left side first link goes DOWN and RIGHT, and the left side second link goes UP and LEFT.

Asymmetry will be necessary in some of the linkages to achieve the desired motion.

Note that the pivot point and second link <-> wing attachment need not be coincident with the actual wing plane (treat it as a long line) and could even be asymmetical too, although the pivot point is likely the one hard symmetrical thing.

Don't worry too much about the 2D linkage design overlapping in weird ways, as this can be solved by curved linkage arms and separation in the Z-direction (but, a 360 degree rotation of the servo is likely impossible in practice).

# Building the linkage model (variables)

For now, put these in a separate config file strictly for this study. Draw everything starting from stowed, which will solve for the wings being stowed symmetrically.

## bike geometry (fixed for a given geometry consideration)
- bike_height (exists)
- bike_width (exists)
- wheel_radius (exists)
- ground_clearance (new? where the bottom of the wing lies when stowed)
- wing_length (derived from the above)

## mechanism geometry
- servo_offset (say, mm z+ from the wheel_radius)
- wing_pivot_offset (say, mm y+ from centerline; keep symmetrical)
- wing_attach_offset (y,z pair in mm from the wing pivot point)
- wing_stow_offset (y mm outboard from the pivot (or attach?) points; must be comfortably outboard of the wing 'link'; this defines where the physical outside edge of the wing sits relative to the 'link' of the mechanism geometry)
- wing_first_link_length (lh, rh) (mm length) (some may be driven dimensions)
- wing_second_link_length (lh, rh) (mm length) (some may be driven dimensions)
- angle_between_first_links (degrees) (nominally 180? but can change; this is how one UP/LEFT link and one DOWN/RIGHT link differ in alignment)

## plan for reproducing the geometry accurately
- first, generate an image of the linkage with labels and we can see how it lines up with my concept (it's a bit complicated so may take a couple revisions to get correct)

# Plots/videos/analysis
- A plot with stowed/extended and a couple midpoints
- eventually, a video of the 2D mechanism deploying
- eventually, analysis to optimize the linkage mechanism
  - kinematic analysis (it stows properly, the final extended position goes far enough and is roughly symmetrical; if that's easy then we can further optimize symmetric deployment)
  - static/dynamic analysis (does it exert enough force? can the torque curve be optimized?)

---

# Status and how to re-tune it

Everything below was added as the study was built. The brief above is the
original concept and is left as written; this section is what it turned into.

## Which dimensions drive, and which are driven

**Three tiers. Only the first is yours to choose freely; the third must never
be hand-edited, because the optimiser and the model both recompute it.**

### 1. The bike envelope — you fix these

| in `config/wing_linkage.yaml` | what it means |
|---|---|
| `bike.bike_width` | wing tip to wing tip, stowed. Also the roof DIAMETER |
| `bike.bike_height` | **top of the bike** — the roof CREST — above the floor. Same meaning as in `bike_params.yaml` |
| `bike.wheel_radius` | rear axle height; the only floor↔axle conversion |

These are the design constraints. Change them and re-run the optimiser —
everything else moves.

### 2. Mechanism geometry — the OPTIMISER searches these

`analysis/wing_linkage.py::_VARS`, nine variables:

    wing_pivot_offset      ground_clearance       servo_offset
    wing_attach_offset y   wing_attach_offset z
    wing_first_link_length right / left
    first_link_angle_deg   angle_between_first_links

Bounds are deliberately wide (crank angles unrestricted, servo may sit below
the axle or high on a mast, attach may sit below its own pivot). An early pass
pinned three of them against tighter bounds, which meant those numbers were
reporting my guesses rather than the mechanism's preference.

### 3. DRIVEN — computed, never edited

| quantity | where from | rule |
|---|---|---|
| `wing_second_link_length` (both) | `Linkage.__init__` | whatever closes the four-bar AT STOW |
| `wing_length` | `Linkage` | `(bike_height − bike_width/2) − ground_clearance` |
| stowed panel top | `Linkage` | `bike_height − bike_width/2`, i.e. the roof AXIS |
| `wing_stow_offset` | `Linkage` | `bike_width/2 − wing_pivot_offset` |
| `stroke.servo_travel_deg` | optimiser | the best simultaneous pose |
| `stroke.goal_current_nm` | fall set | measured, not chosen |
| roof radius / height | `build_model.derive_linkage_roof` | radius = stow half-span, axis at the panel top |

The coupler lengths being *derived from the stowed pose* is the load-bearing
idea: it is why the two sides come out asymmetric on their own rather than
being told to. **The asymmetry is an output, not an input.**

## Re-tuning after a change to the envelope

```sh
# 1. kinematics only -- can both wings reach 90 deg in one monotonic stroke?
python analysis/wing_linkage.py --optimize --iters 400

# 2. add statics -- minimise peak servo torque among designs that pass (1)
python analysis/wing_linkage.py --optimize --torque --iters 400 \
    --save config/wing_linkage_optimized.yaml

# 3. or optimise for a SELF-LOCKING deployed pose instead (the current pick)
python analysis/wing_linkage.py --optimize --lock --iters 400 \
    --save config/wing_linkage_locking.yaml
```

Run several `--seed` values. All three modes are stochastic, and agreement
across seeds is the only evidence the result is a property of the topology
rather than one lucky basin.

Then look at it, in this order — **every wrong answer in this study passed its
own numeric test and was caught by a picture**:

```sh
C=config/wing_linkage_locking.yaml
python analysis/wing_linkage.py --config $C --panels     # stow vs deploy
python analysis/wing_linkage.py --config $C --righting   # bike pushing itself up
python analysis/wing_linkage.py --config $C --torque     # servo torque, both sides
python analysis/wing_linkage.py --config $C --forces     # pin loads for sizing
python analysis/wing_linkage.py --config $C --righting --video
```

`--tag` suffixes the default filename, which is what keeps one config's figures
from overwriting another's. The tracked set in `analysis/plots/` is exactly:

```sh
python analysis/wing_linkage.py                                   # baseline config,
python analysis/wing_linkage.py --deploy                          #   untagged
python analysis/wing_linkage.py --panels
python analysis/wing_linkage.py --torque
python analysis/wing_linkage.py --righting
python analysis/wing_linkage.py --righting --video
python analysis/wing_linkage.py --video

O=config/wing_linkage_optimized.yaml
python analysis/wing_linkage.py --config $O --tag _opt            # torque-optimised
python analysis/wing_linkage.py --config $O --deploy --tag _opt

python analysis/wing_linkage.py --config $C --panels --tag _lock  # the current pick
python analysis/wing_linkage.py --config $C --torque --tag _lock
python analysis/wing_linkage.py --config $C --video  --tag _lock
```

Every tracked figure has to be reproducible **at the name it is tracked under**.
The variants started life as one-off `--out` runs, which meant nothing in the
repo recorded which config drew them — and one of them turned out to be a copy
of the baseline figure under an `_opt` name.

Finally re-check it in MuJoCo, which has contact and inertia the 2D model does
not:

```sh
python -m aow_sim.record --script right --linkage
mjpython -m aow_sim.run_drive --teleop --linkage   # 9 extend, 4 retract, . shove
```

## The constraints, and why each exists

Each was added because an optimiser walked through the gap where it wasn't.

| constraint | value | what it prevents |
|---|---|---|
| both wings reach the target | 90° ± `_KIN_TOL` | one wing deploying while the other sits at 40° |
| **at the same servo angle** | `best_pose` | scoring each wing's best pose separately; there is only one servo |
| deployment is SIGNED | `sweep_window` | a wing driven 90° *inboard*, through the bike, scoring as a success |
| monotonic window | first turning point | running on past the toggle into where a side retracts |
| `MIN_TRANSMISSION_DEG` | 30° | coupler∥rocker, where the servo must supply more torque than the load |
| `MIN_FLOOR_MM` | 2 mm | wings through the floor while the bike is upright |
| `_TORQUE_BUDGET` | 0.55 N·m | buying a spectacular end toggle by making mid-stroke unliftable |

**Two collinearities, opposite effects** — the distinction the whole `--lock`
mode rests on:

* `crank ∥ coupler` — INPUT dead point. MA → ∞. The load cannot backdrive the
  servo. This is the toggle clamp, and it is what `--lock` puts at full
  deployment.
* `coupler ∥ rocker` — output extreme. MA → 0. The servo must supply *more*
  than the load. This is what `MIN_TRANSMISSION_DEG` forbids.

## Where it stands

Current pick, `config/wing_linkage_locking.yaml`:

| | geared 2:1 | linkage |
|---|---|---|
| peak servo torque (2D) | 0.339 N·m | 0.541 N·m |
| MuJoCo fall set | 8/8 | **8/8 from 0.40 N·m** |
| margin vs 9.9 V stall | 1.95× | **1.65×** |
| holds deployed pose | continuous current | **free — MA ≈ 52** |
| current-based position mode | **0/8, somersaults** | **8/8, 0.35 s** |
| total bike height | 216.2 mm | 216.2 mm (same) |
| pin loads | — | coupler 21.7 N, **wing pivot 32.4 N** |

The linkage's case is not peak torque, where gears win. It is that it needs no
commanded trajectory: cap the current, command the endpoint, and the toggle
decelerates the wing into the end pose by itself. Gears under the same command
throw the bike clean over. A rate schedule is a thing that must be re-tuned
whenever mass or contact moves; the linkage does not have one.

**Open, and honest:** the linkage carries less torque margin than gears
(1.65× vs 1.95× against the 9.9 V cutoff stall), and every torque figure here
is quasi-static — no impact from the wing meeting the floor, no inertia.

> **Corrected.** An earlier version of this file claimed the linkage "forces a
> taller roof (216 → 276 mm) which raises the CoM". That was wrong, and it was
> a naming slip rather than a property of the mechanism: `bike_height` meant
> the roof CREST in `bike_params.yaml` but the WING TOP here, so the panel ran
> to the full height and the roof was then stacked on top of it. With one
> meaning — crest, in both files — the stowed panel tops out at the roof axis
> exactly as the geared wing tip does, the tips are tangent to the rolling
> surface, and total height is 216.2 mm for both mechanisms. The wing is
> shorter (181 → 94.6 mm) and the two roof derivations are now the same rule.
>
> Correcting it made the design measurably better, which is the tell that it
> was a bug and not a trade: the fall-set requirement fell from 0.66 N·m to
> **0.40** (margin 1.00× → 1.65×), the response became monotonic in the current
> cap instead of cliffy (it had been 7/8 at 0.40, 6/8 at 0.55, 8/8 only at
> 0.66), and the CoM dropped 128.4 → 126.0 mm.

Outputs live in `traces/linkage_*`.

---

## The mechanisms as of 2026-09-03 — moved here from docs/status.md

Moved verbatim 2026-09-08. This is the design record for the swing mechanism
and the four-bar linkage that status.md had been carrying. Dates in the text
are the dates the work was done.

### The SWING mechanisms — co-rotating, and the reason they exist

Two new mechanisms and a study, all from 2026-08-25. `config/swing_wings.yaml`
(geared), `config/swing_linkage*.yaml` (four-bar), `analysis/swing_linkage.py`,
`analysis/swing_demo.py`. Figures at `analysis/plots/swing_linkage_*`. The
four-bar variants: the hand-drawn `swing_linkage.yaml` (BUILT), `_opt` and
`_margin` from the first searches, and `_compact` / `_vertical` from
2026-09-03 — see the standing note at the end of this section.

**The one idea:** the mirrored pair's joint equality is
`theta_left = -1 * theta_right`, so both wings deploy outward together. Flip
that sign to `+1` — an idler or a belt instead of a direct gear mesh — and the
pair CO-ROTATES: one wing swings down and out while the other comes up and in.

That buys the thing the mirrored pair cannot do: present ONE flat face on ONE
side with the other tucked away, so the far side can brace the bike while a
ball hits the near one. Deploying the mirrored pair far enough to reach a ball
on the right plants the left wing too, and the bike becomes a four-point stance
— measured at a frozen -0.17 deg roll, which is a parking brake, not balance.

It costs side-agnosticism, and the rule is NOT obvious. The splayed rest V is an
outrigger during a fall, so this variant lands on its BACK at ~119 deg, and past
90 deg the lever inverts. Measured from that pose, both sides:

| command | best roll reached |
|---|---|
| latched `sign(roll)` at stroke start | 111.5 / 111.4 deg — no progress |
| continuously updated `sign(roll)` | 111.5 / 111.4 deg — no progress |
| continuous `-sign(roll)` | 66.1 / 63.8 deg — off its back, on its side |
| **flip the sign at 90 deg** | **14.0 / 13.7 deg, up in 0.98 / 0.93 s** |

Two forms, and they are alternatives to each other and to everything in
`righting`:

- **geared** (`build_model(..., swing=True)`, teleop `--swing`) — two hinges,
  one equality, one actuator. Simple, and what the RL policies were trained
  against. Its stroke is limited to +-45 deg by the RISING wing, which reaches
  the drive servos at 50 deg and passes through the battery at 80.
- **four-bar** (`build_model(..., swing_linkage=True)`, teleop
  `--swing-linkage`) — traced from the `wing-linkage-straight` Part Studio.
  Verified against the 2D study to **0.01 mm** over the stroke and 0.01 deg on
  the wing joint. The rocker arc bounds the rising wing geometrically rather
  than by a configured stroke limit.

`analysis/swing_linkage.py` is the design tool, built so a practical problem
(something too weak, a calibration issue) can be expressed as a constraint and
re-searched. It reports a HARD feasibility table — every constraint pass/fail
with margins — separately from the objective, because the objective is a
weighted sum of soft penalties and a soft penalty is zero AT a boundary: runs
repeatedly read `objective 0.000` while sitting on four limits at once.

**Every metric in it was wrong once**, and each is documented beside the wrong
version so nobody re-derives it:

| metric | was | is |
|---|---|---|
| righting | `TARGET_WING_DEG = 90` borrowed from the mirrored study | hand-off roll: the panel's angle from horizontal. The 90 works there only because its stowed wing starts flat on the ground |
| far wing | millimetres from the centreline | ANGLE from vertical. The as-drawn sketch sits at -0.2 deg while reading 29.4 mm; the two do not rank candidates alike |
| brace | reach the floor | finish FLAT. "Reach the floor" let a design plant its TOP edge, having rotated past horizontal |
| protrusion | max over the stroke, then the mean of two endpoints | `max(rest, end)`. A mean lets one endpoint grow while the other shrinks at zero cost |
| transmission | floor over the whole stroke | first 85% only. The minimum sits at 98-100% of the stroke, which is exactly where a four-bar SHOULD approach its dead point |
| torque | computed, printed, never scored | scored against 0.45 N.m |
| protrusion, again | the panel's TOP corner (`top_extents`) | max over BOTH panel ends (`panel_extents`). Top-only is a proxy that holds while the rest pose is a splayed V and breaks under `vertical_rest`, where the rising panel's top comes IN while its foot goes OUT |
| interference | not checked at all | `min_link_gap`: every non-adjacent pair of drawn members, both sweep directions |
| chassis | `far_inboard_deg`, an angle | still the angle for the LEAN, plus `panel_keepout_gap` for whether the volume is occupied. The angle was never a clearance check and cannot become one |
| `--max-half` | accepted, unpacked, never read — a no-op flag since the first version | scored as a hard cap on `stow_half_width` |
| stroke, reach, brace | walked the whole assembly range at 2° | three closed-form toggle angles plus the panel-flat pose |
| torque | `np.gradient` over a uniform 1° grid | exact four-bar velocity ratio at a point, peak by golden section |
| sweep direction | both, on the argument that ±t see different pairs | one. They see the same pairs REFLECTED, and both sides are already evaluated at every +t |
| keep-out width | 18 mm, from the `shape: box` entries of cad_layout.yaml | 32 mm, measured. The derivation missed the CYLINDERS — the servo pulleys' outer edges |

`angle_between_cranks` sets the REST SETPOINT and nothing else that binds.
Measured, holding the lengths and sweeping only that angle, the far wing's
minimum clearance from vertical is -0.2 deg at 20, 30, 45, 60, 75 and 90 alike;
only the stroke length moves. The inward limit is fixed by the crank/coupler
collinear pose, which the LENGTHS determine.

**The study stopped sweeping (2026-09-03).** Every kinematic question is now
answered at closed-form crank angles — rest, the rising wing's folded toggle,
the deploying wing's extended toggle, and the travel that lays the panel flat.
**22x faster**: 44.6 -> 1.3 ms per objective evaluation, 810 -> 47 four-bar
solves. Peak servo torque is the one thing with no shortcut (31 of those 47) —
it is an interior maximum and no single pose stands in for it. `--check` walks
every metric at 0.25 deg and prints the disagreement.

Three rest poses via `mechanism.wing_angle_mode`, and **one pose can be pinned,
not two** — the rocker's swing between rest and the toggle is a link-length
property, so choosing the parked attitude fixes the deployed one:

| mode | pins | hand-off roll |
|---|---|---|
| `fixed` | neither; the config's angle is used | whatever falls out |
| `vertical_rest` | panels upright when parked | whatever falls out |
| `flat_deploy` | panel flat at the deployed toggle | **0 by construction** |

**Standing.** Comparisons need a common panel: `wing_z_max` is a panel-axis
coordinate, not a height, so the same number is a different length of wing on
every linkage. Re-searched with `panel_span_mode: length` at 100 mm, four seeds
each, all converging to spread 0.000:

| | built (hand-drawn) | `fixed` | `flat_deploy` | `vertical_rest` |
|---|---|---|---|---|
| rest half-width, own panel | 73.3 mm | 65.3 | 66.3 | 72.2 |
| **rest half-width, 100 mm panel** | — | 65.8 | **65.0** | 67.9 |
| peak servo torque | 0.510 | 0.494 | 0.446 | 0.550 |
| feasibility | 7/8 (far wing) | 8/8 | 8/8 | 8/8 |

The hand-drawn geometry is **built and works** — it self-rights the weight of
the righting assembly; the whole bike is untested. It is also the only one
holding margin everywhere: 2.06 mm of chassis clearance and 10.98 mm between
couplers, where the searched results sit on two or three limits at once.
`flat_deploy` is the one to build next if width matters.

**Vertical rest does not pay.** It was meant to trade a wider swing for a
narrower parked envelope; on a fair panel it is 2.9 mm wider and spends the
whole torque budget. The gain is real only against a FIXED linkage —
verticalising the built geometry without touching a link length takes it
73.3 -> 47.8 mm — and evaporates once the lengths are re-searched.

**Outstanding.** The width argument was never the whole argument for vertical
rest: the splayed V is the outrigger that makes this variant land on its back at
~119 deg and need the 90 deg sign flip, and nothing in a 2D kinematic study sees
it. That wants a sim run. `_compact` sits AT the hand-picked 15 mm
`wing_pivot_x` floor, where the Part Studio carries `pin_diameter` 1/16 in and
`pin_support_radius` 0.1 in to derive it from. `clearance.wing_width_mm` is a
0.0 placeholder. The keep-out is projected from `cad_layout.yaml`, which is the
simulator's belief and not a measured chassis; the drive belts at |y| 26.5 are
left out of it deliberately. The mirror parity for the RL channel (`-1, -1` in
`general_spec`) is unverified against a trained policy; `--check`'s sketch-point
half is stale against the older `swing-wings-geom-mock`.

**The geometry now goes into CAD as a DRIVEABLE sketch.** `aow_sim.cad_swing_linkage`
emits a Feature Studio holding one custom feature, `AOW swing linkage`: pick a
plane, type a crank input, and it draws the pair at that pose. Insert it several
times on one plane at different inputs and the overlay is the motion study, with
no assembly and no mates. Every `mechanism` number is a dialog field, defaulted
from a config through `SwingLinkage` rather than off the yaml -- `wing_z_min` is
always derived and never written back, so `swing_linkage_smaller.yaml`'s -20.0
is the as-drawn value against the -27.08 the study actually uses.

The four-bar is solved in FeatureScript, so it is a SECOND implementation of the
circle-circle branch rule. `--check` is what keeps the two honest: one billable
call runs the port on Onshape and diffs every joint, crank tip, panel foot and
panel top against `analysis/swing_linkage.py` at seven crank inputs on both
sides. All 56 agree to the printed digit, and the sketch builds 17 edges.
Nothing has been pushed to a studio yet -- it needs its own Feature Studio tab,
since a push overwrites one wholesale and `cad_layout` and `cad_servo_mount` own
the two that exist.

Two Onshape facts fell out of that check and are recorded in the generated
source: `newSketch` handed a plane QUERY on a datum plane draws nothing at all
and reports no error, and `evPlane` wants the FACE a datum plane owns rather
than its body.

**The reasoning is not here.** Every constraint, every metric that was wrong
once, the four-bar identities the closed forms rest on, and the objective's
rung structure live in `analysis/swing_linkage.py`'s docstrings, beside the code
they govern. `config/swing_linkage_constraints.yaml` is the annotated input
schema. This section carries only what moves.

### The wing LINKAGE — a second mechanism, and a real alternative

`docs/plans/wing-linkage-design-and-optimization.md`, `analysis/wing_linkage.py`,
`config/wing_linkage*.yaml`. Figures in `analysis/plots/wing_linkage_*`, with
`--tag _opt` / `_lock` marking which config drew each one.

A four-bar per wing, both on one servo, as an alternative to the gear train.
Gears give a rigidly mirrored pair and a fixed ratio; a linkage gives a ratio
that VARIES through the stroke, which is the point — the deployed pose can be
put at the crank's input-side dead point, where the wing cannot backdrive the
servo.

The geometry is optimised in a standalone 2D study, then built in MuJoCo
(`build_model(..., linkage=True)`, closed with `mjEQ_CONNECT` site
constraints), driven by the same `RightingSequencer`, and available in teleop
(`run_drive --teleop --linkage`).

| | geared 2:1 | linkage |
|---|---|---|
| peak servo torque (2D) | 0.339 N·m | 0.541 N·m |
| MuJoCo eight-fall set | 8/8 | **8/8 across 0.38–0.50 N·m** |
| holds deployed pose | continuous current | **free — MA ≈ 52** |
| current-based position mode | **0/8, somersaults** | **8/8, 0.35 s** |
| total bike height | 216.2 mm | 216.2 mm |
| peak pin loads | — | coupler 21.7 N, **wing pivot 32.4 N** |

**The linkage's case is not peak torque, where gears win.** It is that it needs
no commanded trajectory: cap the current, command the endpoint, and the toggle
decelerates the wing into the end pose by itself. Gears under the same command
throw the bike clean over — 0/8 — and need a tuned rate schedule to be safe,
which is a thing that must be re-tuned whenever mass or contact moves.

**The goal current is a WINDOW, not a minimum**, and this is the single most
important operational fact about it. Too little cannot lift the bike; too much
throws it past upright, because the four-bar's self-limiting only bleeds off so
much and the short wing has little inertia to absorb the rest:

    0.34 -> 1/8     0.38..0.50 -> 8/8     0.54 -> 7/8     0.62 -> 0/8

Configured at **0.44 N·m**, the middle of that window rather than an edge. The
geared pair has no such ceiling — there, more torque is simply more margin.

#### What the optimiser taught, mostly by cheating

Every constraint in `analysis/wing_linkage.py` exists because a search walked
through the gap where it wasn't, and **every wrong answer passed its own
numeric test and was caught by looking at a picture**:

* scoring raw wing rotation → wings folding 180° *through* the bike;
* scoring `|angle|` → a wing driven 90° INBOARD, scoring a perfect zero, and
  dipping 55 mm below the floor on the way;
* scoring each wing's best pose separately → forgetting there is only one
  servo, so only the simultaneous pose is reachable;
* torque-only → parking in an output-side dead point, where the load happens to
  be near zero so it costs nothing on the metric while being the least
  buildable part of the design.

Render the mechanism before believing the objective.

#### Drivers vs driven

Three tiers, recorded in the config files as well as the plan doc. **You
choose** `bike_width`, `bike_height`, `wheel_radius`. **The optimiser searches**
nine mechanism variables. **Driven, never hand-edited**: both coupler lengths
(whatever closes the four-bar at stow — which is why the two sides come out
asymmetric on their own, an OUTPUT and not an input), wing length, stow offset,
servo travel, goal current, and the roof geometry.

`bike_height` now means the roof CREST in every file. It previously meant the
wing top in the linkage config alone, which made that bike a roof-radius taller
and got mis-reported as the linkage "forcing a taller roof". Fixing it shortened
the wing 181 → 84.6 mm, dropped the fall-set requirement from 0.66 N·m to the
0.38–0.50 window, and collapsed the two roof derivations into one rule.

#### Which swing-linkage config won — 2026-09-02/03

**`config/swing_linkage_smaller.yaml`.** `swing_linkage_smaller_v2.yaml` was
tried as an even smaller envelope and did not buy enough to justify itself.
`_smaller` is the one that was exported to CAD for construction, which is the
strongest evidence available: CAD is downstream of the decision.

This was the gap the banner at the top of this file flags — the 09-02/03 work
was recorded "only in `docs/status.md`", and a later rewrite of that file
dropped it, leaving fourteen `config/swing_linkage*.yaml` and nothing saying
which one mattered. Written down here because this doc is the one that grows
rather than being rewritten.

Consequences now wired into the code:

- `build_model.SWING_LINKAGE_CFG` points at `_smaller`, so teleop,
  `analysis/swing_linkage.py` and `cad_swing_linkage` finally agree. They did
  not: the CAD path built `_smaller` while the other two drove the bare
  `swing_linkage.yaml`.
- `run_drive --swing-linkage` takes an optional config path for the other
  thirteen.

#### Not decided

The linkage is **not** a replacement for the geared pair. **"Built" here means
BUILT IN SIMULATION** — both are modelled and driveable (`--swing` and
`--swing-linkage`), and both pass the fall set. **NEITHER HAS EVER BEEN
PHYSICALLY CONSTRUCTED**, and there has never been a real geared pair; an
earlier wording said only "both are built" and read as hardware.

The choice is a real trade: gears have more torque margin and a simpler part
count; the linkage has a self-locking deployed pose and needs no trajectory.
Nothing downstream depends on the answer, so it can wait for the mechanical
design.

**Except that it no longer can, quite.** The linkage side HAS been taken to CAD
for construction — see "Which swing-linkage config won" above — so the geared
pair is the one with no physical path behind it.
