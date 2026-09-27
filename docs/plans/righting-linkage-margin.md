# Righting linkage: margin on the servo, traded against speed

STATUS: PARKED 2026-09-27, decision later. Steps 1-4 done; step 5, picking a
point, is the user's, after the front steer is built. Follows the XC330 model change (`righting-servo-model.md`), which made
the stroke's current requirement measurable. The previous optimisation round
is `wing-linkage-design-and-optimization.md`; by the user's account it was left
half-baked, and step 1 found why its torque numbers could not be designed
against (below).

All numbers are SIM, for the live geometry `config/swing_linkage_smaller.yaml`
(`build_model.SWING_LINKAGE_CFG`), unless marked otherwise.

## What the current four-bar needs

`analysis/righting_current_sweep.py`, bike on its side, crank goal stepped as
teleop does, wheels passive, 12 V:

| | counts to reach upright |
|---|---|
| goal stepped (teleop's 9/4) | 630 (MEASURED by hand in teleop: 625) |
| goal ramped at 15 deg/s (quasi-static, `--slew-dps 15`) | 660 |
| 11.1 V / 9.9 V, stepped | 630 / 650 |
| Current Limit | 910 |

So the swing's momentum over the hump is worth ~30 counts (~5 %); the lift is
essentially quasi-static. Stepped at 910 counts the bike is up (within 5 deg)
in 0.53 s at 12 V and 0.87 s at 9.9 V, and with passive wheels goes over the
far side.

## Step 1: the load, and the best any transmission could do

`analysis/righting_ideal_profile.py`, figure `analysis/plots/righting_ideal_profile.png`
(`--plot`). The stroke run quasi-statically (10 deg/s, 910 counts), recording
the torque each coupler puts on its wing's hinge -- the loop equality's force
projected onto the hinge. That is the load the bike presents to ANY
transmission at that wing angle, for this panel and hinge.

**The load is the whole story.** Virtual work through the four-bar's ratio r =
d(wing)/d(crank) plus the gearbox's running friction line,
`tau_motor = 1.15 tau_wing r + 5.7 mN m`, reproduces the servo model's motor
torque to a median 4 mN m (p95 8). The rising wing does -3 mJ of the 780:
negligible. So a transmission can be scored from the load curve and its own
ratio, with no sim per candidate.

| crank [deg] | -9 | 6 | 22 | 41 | 64 | 75 | 82 | 94 | 110 | 123 |
|---|---|---|---|---|---|---|---|---|---|---|
| wing [deg] | -4 | 3 | 12 | 24 | 39 | 46 | 51 | 59 | 67 | 73 |
| roll [deg] | 79 | 73 | 64 | 52 | 36 | 29 | 25 | 18 | 10 | 5 |
| **load at the wing hinge [N m]** | **0.88** | 0.85 | 0.79 | 0.68 | 0.49 | 0.43 | 0.54 | 0.45 | 0.26 | 0.14 |
| ratio r (as built) | 0.40 | 0.50 | 0.59 | 0.65 | 0.68 | 0.66 | 0.65 | 0.61 | 0.50 | 0.33 |
| motor torque [N m] | 0.41 | 0.49 | **0.54** | 0.51 | 0.39 | 0.35 | 0.41 | 0.32 | 0.15 | 0.05 |

- **The load is largest at the START** (bike on its side, 0.88 N m) and falls
  as it comes up. From 79 deg down to ~29 deg of roll the whole bike rides on
  the panel with both wheels in the air; at ~29 deg the front tyre and a rear
  roller touch down, the pivot moves off the panel, and the load steps back
  up -- the second hump.
- The as-built motor peak (0.539 N m at ~64 deg of roll) is where the ratio
  rising outruns the load falling. Peak/mean 1.38 over the 132 deg of crank
  it uses (crank moving -> within 5 deg of level); wing work 780 mJ, motor
  901 mJ.
- **Breakaway** (NOT in the front below): the stroke starts from rest with
  the bike already on the panel, so the first instant is on the STATIC
  friction line (0.51, not 0.15). As built that needs 0.536 N m -- as much as
  the hump -- and its low starting ratio is what keeps it there. A flat
  profile needs its first few degrees geared ~24 % lower for the same reason;
  that costs negligible travel.

**The front.** Over a crank travel THETA the wing work is fixed, so the
lowest possible peak is the FLAT profile, r(wing) = tau_out / load(wing).
Stroke time is quasi-steady (motor on its line at the required torque, inertia
ignored), which reads 20-25 % fast against the stepped sim (0.42 vs 0.53 s at
12 V, 0.73 vs 0.87 s at 9.9 V) but ranks designs the same way. Margin = the
Current Limit's torque (or stall at the supply, whichever is lower) over the
need.

| crank travel | motor | counts | margin 12 V | time 12 V | margin 9.9 V | time 9.9 V |
|---|---|---|---|---|---|---|
| **as built, 132 deg** | 0.539 | 666 | 1.38x | 0.42 s | 1.23x | 0.73 s |
| flat, 120 | 0.434 | 539 | 1.71x | 0.39 | 1.52x | 0.63 |
| **flat, 132** | 0.395 | 492 | 1.88x | 0.38 | 1.67x | 0.59 |
| flat, 180 | 0.291 | 367 | 2.55x | 0.42 | 2.27x | 0.58 |
| flat, 270 | 0.196 | 253 | 3.78x | 0.53 | 3.37x | 0.69 |
| flat, 360 | 0.148 | 195 | 5.00x | 0.65 | 4.45x | 0.83 |

    python analysis/righting_ideal_profile.py --plot

What it says:

1. **Shape alone, same travel: 666 -> 492 counts**, and slightly FASTER. The
   as-built ratio climbs through the stroke while the load falls; the ideal
   ratio climbs about twice as steeply (0.36 at the start, ~1 or more at the
   end).
2. **Margin is nearly free in speed up to ~2x the travel.** A flat torque is
   also the quasi-static minimum-TIME profile (1 / (s - tau/ts) is convex),
   so speed and margin want the same shape and differ only in the travel. At
   9.9 V, 270 deg of flat travel is 3.4x margin in 0.69 s -- more margin and
   no slower than the as-built 1.23x in 0.73 s. The time is flat from ~120 to
   ~200 deg; the fastest point is ~150 deg.
3. **Capping the ratio costs ~nothing.** The flat profile races the wing
   through the unloaded tail, and a fast wing throws the bike over the far
   side. `--r-max 1.0` (at most one wing degree per crank degree) moves the
   132 deg row from 492 to 503 counts. The tail carries little work.
4. **A single four-bar stroke is bounded near 180 deg of crank** (toggle to
   toggle). Past that needs a reduction between the servo and the crank, or
   multi-turn. The XC330's mode 5 is documented as multi-turn; that is
   UNVERIFIED here.

### The previous round's torque model was backwards -- FIXED in place

`analysis/swing_linkage.py`'s `torque_at` / `torque_curve` / `peak_torque`
scored every candidate against a load shaped backwards for this mechanism.
Its kinematics were right (ratio 0.465 vs the sim's 0.46 at the start, a 0.676
peak in both), but it carried the mirrored study's `roll = 90 - wing` (which
`handoff_roll`'s own docstring says does not hold here) and rotated the CoM by
the WING angle, so the bike on its side was scored as upright:

| crank | old model: load at the wing | sim |
|---|---|---|
| 0 deg | 0.056 N m | 0.87 N m |
| 90 deg | 0.81 N m | 0.45 N m |

Its peak for `_smaller`, 0.506 N m, sat near the truth only by coincidence of
magnitude, at crank 90 deg instead of 26. So `limits.torque_nm` and the
peak-torque term steered that search toward mechanical advantage at the END of
the stroke. Its kinematic constraints (clearance, keep-outs, toggles,
hand-off) were not affected.

**Replaced (2026-09-27) by `resting_pose`**, a planar quasi-static model:

- The bike rests on the hull edge of {panel foot, panel top, wheel contact}
  that has its CoM over it. **On the panel** (wheels in the air), the hinge
  load is EXACTLY the weight of everything but the wing times its horizontal
  lever about the hinge -- a function of roll alone. **On a panel end and the
  wheels**, moments about the panel end give the wheels' share; the sliding
  panel end's friction, reacted at the wheels, adds a term.
- Mass and CoM come from the MuJoCo model (1.135 kg), not the mirrored study's
  constants (1.159 kg, 123.5 mm), which are gone from this file.
- Friction after touchdown: `FLOOR_FRICTION_EFF` 0.4, FITTED to the sim (rms
  27 mN m; 0 reads up to 88 mN m low, 0.9 up to 136 high). The sim's own floor
  is a GUESS 0.9, and the omni's rollers let the wheel side slide.
- Against the sim, per wing degree: median 4.5 mN m; roll within 0.1 deg on
  the panel. The worst, 0.22 N m OVER at wing 45 deg (p95 108), is the
  touchdown instant, which the point wheel reaches ~2 deg early and the sim
  passes through gradually -- conservative. **Peak motor torque through the
  as-built stroke: 0.541 N m (model) vs 0.539 (sim).** `righting_ideal_profile.py`
  prints the comparison; its figure overlays the two.
- `peak_torque` is now a 2 deg scan plus golden section: the old
  single-turning-point claim was true only of the backwards load (the real one
  has two humps). 3 ms per candidate.
- The righting video (`--righting --video`) draws the bike from the same
  pose, with its real roll; it had the panel-flat-only rotation and labelled
  roll as `90 - wing`.

Still in output-torque units at the crank: `limits.torque_nm` 0.55 is a crank
torque, and the motor needs `1.15 tau + 5.7 mN m` of it moving (~785 counts).
Step 2 scores counts directly.

**Not checked:** `analysis/wing_linkage.py`, the MIRRORED mechanism's study,
has its own copy of the old constants and load case. It is not the live
mechanism; its load model was not compared against a sim.

## The design space, as the user framed it (2026-09-27)

- **One turn of servo, shared by both sides.** Mode 5 is documented as
  multi-turn (+-256 rev), but the turn count is RAM (`status.md`, measured on
  the steer), so a stroke past one turn needs a re-zero at power-up or a
  sensor on the wing. So servo travel per side <= ~180 deg, which bounds
  EVERY transmission at the flat-180 row: 367 counts, 2.27x at 9.9 V.
- **The co-rotating pair binds before that.** Both wings ride one crank, so
  the RISING wing at the end of a stroke is the deploying wing's function at
  -T, and a four-bar's rocker is periodic in the crank: approaching a half
  turn of stroke, the "stowed" wing comes round and deploys on the far side.
  Unbounded, the search found 540 counts at 162 deg with the rising wing
  ending 69 deg off vertical, 17 mm off the floor. `_smaller`'s returns to
  exactly its rest attitude. Now a constraint (below).
- **A gear sector per wing is worse, not better.** A constant ratio over the
  full 180 deg needs 578 counts moving and 708 to break away, against the
  as-built four-bar's 666 / 662: the load is heaviest at the start, and only a
  varying ratio puts the advantage there.
- **Poses.** REST and FLAT (end of stroke, to within 1 deg) are held exactly;
  MINIMUM STOWED is a bound on both sides -- no further inboard than
  `limits.far_inboard_deg`, and no further OUT than the rest attitude (+2
  deg) anywhere in the stroke. The user's note that it is "linked to the rest
  angle" is the second half.
- **Objective:** minimise the counts needed at 9.9 V (running peak AND
  breakaway), subject to those, servo travel <= 175 deg, d(wing)/d(crank) <=
  1, a stroke no slower than now, and the study's clearance constraints.

### Which way should the hinges move? (asked 2026-09-27; PLANAR MODEL)

On the panel the hinge load is the chassis's weight times its lever about the
hinge -- but the WORK is set by the CoM's lift from the fallen pose to upright,
and that depends on where the STOWED PANEL is, not the hinge. Moving hinge and
panel together, laterally, holding the panel's angles (`swing_synthesis.load_table`
with a shifted hinge):

| right hinge at y [mm] | work to right it | flat profile over 144 deg |
|---|---|---|
| +15 (crossed to the far side) | 1019 mJ | 585 counts |
| +5 (crossed) | 866 | 501 |
| 0 (one shared rod) | 797 | 463 |
| **-5 (as built)** | **733** | **428** |
| -15 | 615 | 363 |
| -40 | 430 | 261 |

- **Outboard is better, crossed is worse.** A panel further out on the fall
  side props the fallen bike higher (same 75 deg of roll, CoM further off
  the floor), so there is less to lift. Crossing the hinges does the opposite.
- **Not zero-sum, up to a point** (the user's "once it hits that inflection
  point, the wheels fall back down"). The work is the CoM's rise to its
  HIGHEST point, not to the end. Up to ~40 mm out the CoM rises all the way
  (start 58 / 68 / 92 mm at -5 / -15 / -40, top 122 = upright), so the saving
  is real. At -60 the propped bike is lifted over the top -- 132 mm at wing
  30 deg -- and drops back to 122 on its wheels: from there on the gain
  reverses. `resting_pose`'s load is now SIGNED so that stretch cannot be
  scored as load.
- **So the old search pulled the hinges together for WIDTH**, not load: its
  objective was the parked protrusion, and its backwards load model scored
  the start's lever as the hinge's own lateral offset, which made central
  hinges look free.
- **A shared rod, panel left where it is** (`--pivot-x 0 --keep-panel
  --stagger-rockers`, wings side by side on the rod): with `--one-pin` too it
  is the user's DIAMOND -- one crank pin, two couplers, two wings, one rod.
  547 counts in the sim against 537 for separate hinges: about 10 counts.

### Flat vs convex panel (asked 2026-09-27; DERIVED, not simulated)

On the panel, the hinge load is a function of roll alone, WHATEVER the panel's
shape. What the shape changes is how much roll a degree of wing buys: a flat
panel lying on the floor gives 1:1; a convex one rocks until the CoM is over
its contact, so it gives less (an arc centred on the hinge gives none -- the
wing just rolls under the bike). So a convex panel is a ratio stage in series
with the four-bar, and for the profile it is second-order, as the earlier
notes said. It changes the WORK only the way the table above does: through how
high its outermost reach props the fallen bike. It can matter for the
DYNAMICS (rolling vs tipping onto an edge), and after touchdown the panel
end's reach enters the load directly. The sim's panel is a flat box; the
printed one is convex.

### A panel shape that limits damage in a fall (asked 2026-09-27; REASONING, not simulated)

- **Aim impacts at the hinge.** A contact force whose line of action passes
  through the hinge puts no torque into the linkage or the servo. That is a
  surface that is an arc CENTRED ON THE HINGE where the panel is likely to be
  hit -- the same shape that is useless for lifting (above), so it wants to
  be the outer face at rest, not the part that ends flat. Friction still puts
  mu N R through.
- **Stopping distance is the big lever.** The fall releases about what the
  lift costs, ~0.8 J. Stopped rigidly in ~2 mm of print that is ~400 N at the
  contact; 10 mm of give (TPU skin, a flexure rib) is ~80 N.
- **There is no chassis to land on** (the user, 2026-09-27): the wings ARE the
  outer shell, so they take every landing. That puts all the weight on the
  two points above and below it: the load path through the hinge, and give.
  (The sim's fallen bike also touches a `bumper_right` geom for its first
  instant; if the real bike has nothing there, the sim's fallen pose is
  slightly off.)
- **Low ratio at rest protects the servo.** A back-driving torque on the wing
  reaches the crank times d(wing)/d(crank); near a toggle at rest it barely
  does. The free-end candidate below starts at 0.38 against the reference's
  0.47.
- **A convex, continuous outer surface** turns a slam onto an edge into a
  roll, spreading the impulse -- the turtle-shell instinct, and a reason for
  the curve that is about landing, not lifting.

Checkable in the sim later: the fall set with the crank's constraint torque
recorded, per panel shape.

## Step 3: synthesis -- `analysis/swing_synthesis.py`

Searches the four-bar lengths, the angle between the crank arms and the servo
height; for each candidate the panel's bearing and offset off the rocker are
DERIVED so it lands back on `_smaller`'s panel line (the attach point is free
on the wing), so every candidate passes through the rest pose, and the stroke
ends where the panel is flat. Scored from one load table (same panel and
hinge, so the load is a function of wing angle alone) through the friction
lines, 6 ms per candidate, differential evolution. `_smaller`'s own `bounds:`
block by default; `--wide` for more.

| `_smaller`'s panel, hinge, bounds | config | planar: moving peak, counts at 9.9 V | travel | SIM: counts at 9.9 V |
|---|---|---|---|---|
| built today | `swing_linkage_smaller.yaml` | 669 | 137 | **643** |
| best, couplers in one plane | -- | 664 | 136 | -- |
| couplers staggered, self-locking end | `swing_linkage_stagger_lock.yaml` | 604 | 144 | **575** |
| couplers staggered, free end | `swing_linkage_stagger.yaml` | 580 | 134 | **537** |
| **diamond: one crank pin, one wing rod** (`--pivot-x 0 --keep-panel --stagger-couplers --stagger-rockers --one-pin`) | `swing_linkage_shared_rod.yaml` | 586 | 128 | **547** |

SCORED BY THE MOVING PEAK (2026-09-27), breakaway from rest a limit (it must
fit under what the servo gives at 9.9 V), ties to the most compact linkage.
The first version scored the larger of moving and breakaway; breakaway at
crank 0 is conservative (the sim's fallen bike back-drives the crank first)
and set every staggered design to the same ~0.50 N m, hiding the moving
peaks the sim ranks by -- a shared rod scored like the free end and needed
585 against 518 in the sim. It also left the linkage's SCALE free (seeds put
the same design's servo at 20, 26 or 41 mm). Now the planar number sits
25-45 counts above the sim, in the same order. The diamond comes out at the
config's lower bounds (crank 12 mm, servo 10 mm above the axle).

SIM is `analysis/swing_stepthrough.py --sim`: the sweep's own fallen/trial,
goal stepped, bisected to 10 counts. Each config's header carries the command
that made it and its sim check. The stepthrough page it writes
(`analysis/plots/swing_stepthrough.html`) steps any two of them side by side
through the stroke, reading the same files the sim does.

    python analysis/swing_synthesis.py --stagger-couplers --tag _staggered \
        --save config/swing_linkage_stagger.yaml
    python analysis/righting_current_sweep.py --config config/swing_linkage_stagger.yaml --supply 9.9
    python analysis/swing_stepthrough.py --sim

- **The couplers passing each other is what binds `_smaller`.** In one plane
  (the study's default, one plane per member TYPE) nothing does better than
  750; `_smaller` is already about optimal there. In separate planes -- your
  "gears in different planes", applied to the couplers; `clearance.planes`
  now takes `couplerR` / `couplerL` -- the search goes to one short crank arm
  (`angle_between_cranks` ~0, crank ~14 mm) carrying both couplers. Whether
  the left coupler's pin can pass through the right coupler's plane is a CAD
  question the plane model does not see.
- **Self-locking costs ~60 counts, and is not needed** (the user,
  2026-09-27). `_smaller` ends at the extended toggle (ratio 0), so the
  deployed wing holds without current; it does NOT lock at rest (ratio 0.47
  there). The free-end design ends at ratio 0.68: a brace load back-drives
  the servo, and the wing arrives at speed (the position loop stops it).
- **The planar breakaway is conservative.** It is taken from rest at crank 0;
  in the sim the fallen bike back-drives the crank ~10 deg first, to a lower
  ratio. The planar counts sit 60-130 above the sim's, in the same order.
- **Margin in sim terms at 9.9 V** (stall there caps at ~812 counts' worth):
  built today 1.26x, self-locking 1.41x, free end 1.57x, shared rod 1.39x.
- **Still far from the flat bound** (465 counts over 132 deg): the 30 deg
  transmission-angle floor binds both staggered results.

### Pin loads and servo fit (2026-09-27)

Short levers carry more force. `swing_linkage.pin_forces` (the wing's free
body, quasi-static) and the sim's loop-closure force during a stepped lift
(700 counts, 9.9 V, up to upright only), from `swing_stepthrough.py --sim`:

| design | coupler / hinge pin, slow | coupler pin, sim lift | falling back over from upright (sim, rough) |
|---|---|---|---|
| built today | 20 / 15 N | 30 N | 92 N |
| staggered, free end | 27 / 19 | 45 | 104 |
| diamond | 41 / 32 | 72 | 162 |
| both wings out | 41 / 35 | 126 | 74 |

The last column is NOT an overshoot of the stroke: with passive wheels the
upright bike simply falls over again (it is unstable standing still), onto
the far wing. It rides on the sim's rigid contact and is a rough bound, not a
measurement.

**Where the sim-lift peak happens:** in the first 1-3 ms on every design,
crank still at its fallen -6 to -11 deg -- the servo stepping from holding to
full current against a bike that has not moved. Not a solver artifact: with
nothing commanded the same start reads only the static load (15 N built
today, 41 N the diamonds). The low starting ratio that buys margin also
multiplies this force. After it, the median through the lift is 12-37 N. A
SOFT START fixes most of it (sim, 700 counts, 9.9 V):

| Goal Current ramped over | diamond: pin peak, to upright | both wings out | built today |
|---|---|---|---|
| 0 ms | 72 N, 0.72 s | 126 N, 0.64 s | 30 N, 0.87 s |
| 50 ms | 66, 0.76 | 79, 0.66 | 29, 0.91 |
| 150 ms | 57, 0.83 | 65, 0.70 | 27, 0.99 |
| 300 ms | 52, 0.92 | 67, 0.74 | 25, 1.11 |

**Roll-scheduled Goal Current (the user's AHRS idea), tested in the sim on
the diamond and on `_smaller`, 9.9 and 12 V:** no effect on the lift. Full
current above 25-40 deg of roll tapering to 0-300 counts at upright gave the
same time to upright and the same pin peaks as a constant 910, because the
servo is SPEED-limited (on its voltage line) through the lift, not
current-limited, so the cap never binds. Scheduling the goal POSITION does not
help either: stopping or reversing the crank below 20-30 deg of roll leaves
the bike short of upright, and stopping below 12 deg still lets it fall over
-- the falling over is the passive wheels, not momentum. Where the AHRS could
help instead (UNTESTED): drop the Goal Current when a fall is detected so a
wing gives way on impact rather than loading the pins. For pin sizing (CALCULATED, not tested), at 160 N: a steel pin
in single shear sees 81 / 36 / 20 MPa at 1/16, 3/32, 1/8 in -- fine for steel
in all three. The weak part is the printed boss: bearing F / (d t) over a
5 mm boss is 20 / 13 / 10 MPa, and repeated knocks ovalise printed holes.
1/8 in at the crank pin and the hinge rod is the conservative choice.

The XC330 case (20 x 34 mm in the front view, shaft 9.5 mm from one end)
sits behind the horn, in the fore/aft span of the panels and the hinge rod.
Closest approach over the whole stroke, both directions, 1/8 in rod: the
diamond fits pointing UP (9.0 mm clear) or sideways (8.5); pointing DOWN it
hits the hinge rod (-6.0). Not checked: the wheel, and anything else in the
chassis at that station.

## Steps

1. ~~**Ideal profile, mechanism-free.**~~ Done, above.
2. ~~**Fix the load model.**~~ Done, above.
3. ~~**Synthesis from poses.**~~ Done, above.
4. ~~**Verify finalists in the sim**~~ at 9.9 V. Done, above.
5. **Pick a point** with the user, from the stepthrough page: self-locking is
   not required; whether staggered couplers are buildable; then packaging,
   and the stowed panel's width against the work it saves.

## Not in scope

- The servo's own law above ~200 mA (unmeasured; `righting-servo-model.md`
  step 0). Every count here extrapolates it.
- Catching the bike after the lift (the sweep's wheels are passive).
