# Possible linkage mechanism for rocker wing deployment

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
