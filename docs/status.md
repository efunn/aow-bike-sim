# Project status — 2026-08-21

Midpoint snapshot. The design logs under `docs/plans/` are where decisions and
their reasoning live; this file is the layer on top of them — what is true
right now, what is next, and what is broken. It is meant to be re-written, not
appended to.

---

## In one paragraph

The simulator is done enough to be trusted for control design. A general RL
policy balances and drives the bike from a live (velocity vector, heading)
command, and the analytic LQR stack exists alongside it as a reference
baseline. The onboard software path is built and proven in sim — the hardware
shim, the deploy bundle, the odometry estimator, the AHRS protocol — with no
hardware to run it on yet. The tethered rig's parts are on hand; the untethered
electronics are specced and sourced but not ordered. **The test suite is back
to a defensible state** (7 failed / 217 passed, and all 7 are the trajopt moves
already scheduled for re-authoring) after `contact_solref` was reverted to
`[0.005, 1.0]`. Two things changed shape since the last snapshot: **pitch is
now observed and priced**, which closed out a long-running investigation into
why crab does not work, and **the self-righting mechanism is now a complete,
verified design** rather than a recommendation — a mirrored wing pair whose
whole envelope derives from two measurable numbers, now with a **second,
independently optimised mechanism** (a four-bar linkage) built and measured
alongside it. A fifth workstream opened since: **CAD**, which
is where the bike stops being parametric and starts being drawn. It already
pushed back — the belt geometry pinned a placeholder that had been made up, and
the rear of the bike went from 99 mm wide to 80 mm as a result. The remaining
open engineering question is still **the contact model**, which is the
least-known parameter in the sim and the one no policy has been randomized
over.

---

## The five workstreams

| workstream | state | what "done" looks like | blocker |
|---|---|---|---|
| **Simulation & model** | Working. Parametric MJCF from `config/bike_params.yaml`, procedural omni-wheel contact meshes, **17** parameters still marked `GUESS` | Every `GUESS` replaced by a measurement or a deliberate randomization range | Physical parts to measure |
| **Control — RL** | Working, and the primary path. Pitch is now observable and priced (`obs_pitch`, `w_pitch`) | One champion policy, symmetric left/right, exercised over the randomization ranges the hardware will actually see | Crab still one-sided; turn asymmetry stuck ~0.27–0.32 across every run |
| **Control — analytic (LQR)** | Reference baseline only; nothing drives with it. Fit is currently **healthy** (worst R² 0.9893) | Re-tuned once the contact model is pinned | Nothing right now — it will degrade again when contact damping moves |
| **Hardware / untethered** | Software complete and tested in sim; nothing physical assembled | Bike balances untethered on a mat | Parts, chassis, servo homing decision |
| **CAD** | Started 2026-08-18. Layout exports from `aow_sim.cad_layout` into Onshape; drivetrain, steering and self-righting stations pinned, electronics packing deferred | A drawn bike whose as-built numbers replace the `GUESS`es in `bike_params.yaml` | Nothing — it is the thing being worked on |

---

## What changed since 2026-08-09

**The contact revert fixed most of the red.** `sim.contact_solref` went back to
`[0.005, 1.0]` in `511421f`. That resolved the `test_hw_odometry` failures (the
bike was falling over mid-episode, so the velocity estimator was measuring
nothing), the `test_teleop` failures, and the analytic drive circles. Damping
0.5 is expected to return once the negative-solref sysid lands — see the
correction below — so the guards for it stay in place.

**`sim.contact_solimp` is now written down explicitly.** It was an invisible
inherited default that every result to date silently assumed, and it is not
negligible: `dmin` 0.9 → 0.5 changes sink at bike weight by 2.3×, which
confounds both bench tests. See the note in `bike_params.yaml` — the joint
(timeconst, dampratio) identification is really a three-parameter problem until
`dmin` is pinned or the negative convention makes it moot.

**Two hazards from the last snapshot are closed.** Policy exports can no longer
be silently overwritten (`97ab064` — they stamp a `params_digest` and refuse
collision), and the "linearization takes minutes" belief that justified caching
the LQR design was wrong by two orders of magnitude (`fff1c96` — it is 0.39 s
for a single design, 2.0 s with the gain schedule, and every in-sim path
re-derives it on startup, so it can never be stale).

---

## The pitch workstream — why crab does not work

This is the substantive control finding of the period, and it is a negative
result that closed a hypothesis rather than a win.

**The hypothesis.** The differential is the only actuator for both roll balance
and lateral crawl, and under a hold command it is already p95-saturated by
balance alone. A sustained pure crab is therefore not physically available at
standstill. The only manoeuvre that produces net lateral motion is an
oscillatory wriggle — the parallel-park bracket — but that has instantaneous
speed far above commanded speed, and the baseline reward scores *instantaneous*
velocity error every step. Measured: at a 0.144 m/s crab command, a perfect
wriggle scores *worse than standing still*. So the reward forbade the one
manoeuvre that could satisfy the command.

**Arm 1 — `general_rl_glide_og`** low-passed the measured velocity
(`vel_window_s`) so the gait's time average is what gets tracked. It worked, in
the sense that the policy did start oscillating — 2× more than any other policy
against a common 1 s reference. **But the oscillation was pitch, not the
bracket**, and pitching produces no lateral travel: net `v_lat` under a crab
command came out at 0.064 m/s, *below* `general_rl_smooth_diff_og`'s 0.082 from
plain per-channel smoothing. It also cost hold quality (drift 0.20 → 1.46 m).

**Why pitch was free.** `w_upright` priced roll and *nothing priced pitch*, and
a time-averaged velocity reward makes a pitch oscillation free twice over —
a wheelie that comes back down averages to nothing. Arm 1 reached **23° nose-up
with the front wheel 79 mm off the floor** on a plain accelerate command.

**Two tools were built to see this, and both earn their keep.**

- `analysis/liftoff.py` — supersedes the airborne-*percentage* columns in
  `hold_spectrum.py` and `chatter.py`, which are actively misleading. Binary
  "is the wheel touching" counts the rear omni's own ~0.6 mm envelope ripple
  identically to a visible wheelie: the rear reads 40–60% "airborne" while
  never clearing 5 mm, the front reads about the same while clearing 79 mm.
  Same percentage, two orders of magnitude apart in behaviour. It reports the
  gap *distribution* in mm, and separates pitch from hop by comparing measured
  pitch, implied pitch, and common lift. `corr(front gap, pitch) = 0.999`
  confirmed arm 1's liftoff was pitch, not hopping.
- `analysis/pen.py` — the top-down ground track per command, which is what the
  operator actually sees. A mean velocity cannot represent "it draws a slow
  squiggly repeated S"; a squiggle whose lobes cancel and a straight line have
  the same mean.

**Arm 2 — `general_rl_glide_pitch_og`** changes three things, each aimed at one
link in the chain: `v_lat_frac` 0.4 → 0.12 (at 0.4 the crab command reaches
102% of the differential's kinematic ceiling and **8.1% of sampled commands are
impossible for any controller**; at 0.12 that is 0.0%, and the optimal response
to a mostly-impossible channel is to ignore it, which is what every policy so
far has done), `w_pitch` 0.0 → 2.0 (a 23° wheelie now costs ~11% of the
per-step maximum — priced, not forbidden), and `obs_pitch: true` (required by
the second: charging for state the policy cannot see is a known defect).
Observation is 19 wide, and **width is not the contract** — the move yaml
records `obs_layout` and replay compares it element-wise.

**The exit criterion is already written down, and it matters:** if crab is
still flat *and* pitch is now controlled, the wriggle hypothesis is dead and
the honest next step is the open-loop gait sweep, not another arm.

---

## Self-righting — now a design, not a recommendation

`docs/plans/self-righting.md` part 4–5. The mechanism is a **mirrored wing pair
on one XC330**, and everything about its geometry now derives from two numbers
you can measure on the finished bike.

**The gear train is the reversal.** The two wing gears mesh each other directly
and the servo drives one of them, so the mirror-symmetric deployment comes out
of the gear train itself — the separate idler the original sketch worried about
is gone. It also decouples the reduction from the stance: equal discs on pivots
`2·pivot_y` apart means `r_disc = pivot_y` *whatever the ratio*, so
`bike_width ≥ 4·pivot_y` independent of the reduction. Width is bought with
pivot spacing, and narrow wins — at a 35 mm half-span (140 mm wide) the roof
becomes the widest thing on the bike and side falls perch on it, spread 86°.

**The envelope is derived, not tuned.** `params.derive_righting()` computes
`roof.radius`, `roof.height`, `wings.crank_length` and `wings.length` from
`bike_width` (120 mm) and `bike_height` (165 mm above the rear axle), so the
stowed wing tips sit exactly *on* the roof surface. That tangency is the whole
point: tips on the rolling envelope cannot prop the bike up, tips outside it
become outriggers and caught it at 154°. Pinned by
`test_righting_envelope_is_derived_and_tangent`.

**The roof earns its place.** The bare bike's inverted "shelf" is 2.69 mm of
CoM height — about 27 mJ — against falls that arrive with 75–300 mJ. It is a
rounding error, and the inverted state is reachable on purpose (reverse at
speed, then a 180 flip). A capsule ridge along +X replaces the flat-topped AHRS
as the top of the bike; roll becomes unstable while pitch stays neutral, which
is what a ridge should do.

| | current design |
|---|---|
| reduction | 4:1 |
| peak servo torque | **0.520 N·m — 0.79 of the 9.9 V stall** |
| servo travel | 1.08 turns → **extended-position mode required** |
| gear fit | disc 30 mm, pinion 7.50 mm, ceiling **5:1** |
| inverted drops recovered | **10/10** across 160–200° roll, 0–15° pitch |
| ordinary side falls | rest spread **0.5°** |
| deploy stroke | 0.63 s (scheduled rate, 3.2× faster than flat 0.7 rad/s) |
| retract | 0.57 s |
| current draw | 0.57 A peak, 0.20 mAh per attempt |
| mass | +143 g (+14.1%), and CoM moves **down** 124.3 → 123.5 mm |

The deploy rate is scheduled on how far over the bike is, because torque peaks
at the **end** of the stroke (0.52 N·m inside 20° of upright) against only
0.13–0.15 N·m through the 20–60° middle — the mechanism is cheap while levering
and expensive while catching the bike as it arrives. Fast through the middle,
easing into the finish, is 3.2× quicker at a slightly *lower* peak.

Still `GUESS`: `min_pinion_radius`, which the 5:1 ceiling hangs entirely off.

---

## Simplified contact models — surveyed, and the answer is "no, but"

`docs/plans/aow-contact-approximations.md` (2026-08-15) closes the "fast
approximation models deferred" item in `mujoco-modeling-decisions.md`.
Reproduce with `python analysis/contact_surrogates.py`.

- **No contact surrogate is worth building.** Swapping the 16 cone meshes for
  primitives buys 10–20%; deleting the whole roller multibody buys 1.7–2.0×
  and is the hard ceiling, because the cost is DOFs and equality rows, not
  collision geometry. The two textbook reductions are both unavailable: a
  ball wheel drops the toppling lever arm from `h_com` to `h_com − R` (−42%),
  and MuJoCo's anisotropic friction is world-locked on a flat floor
  (`t1 = [0,1,0]` at every wheel yaw), so the LeKiwi capsule trick does not
  survive a steering bike. Mark it **rejected**, not deferred.
- **Policies do not depend on roller detail.** `general_rl_smooth_stiff`,
  unmodified weights, eval grid, rear wheel swapped: survival 1.00 on every
  scheme, tracking within 5.5%, ordered by ride roughness. The blind spot the
  survey was looking for is not there.
- **The speedup was in `sim.timestep`, and it is now taken: 2.0e-4 → 4.0e-4,
  with `mesh_segments` 32 → 64.** Contact statistics are converged from 5e-5
  clear through 6e-4, so the contact never bound it — **the LQR's
  finite-amplitude system ID does.** Worst fit R² runs 0.9748 (2e-4) → 0.9727
  (4e-4) → 0.9408 (6e-4, collapsing at +0.80 m/s alone), and
  `test_gain_schedule` floors it at 0.95. So 6e-4 is off the table despite
  scoring fine on every policy eval — no eval can see this bound.
- **`mesh_segments: 32` was bouncing the front tyre off its own facets** —
  17.5% of a 0.5 m/s run off the ground at a 0.26 mm swing, against 0.2% at 64
  segments for ~3% of step time. Pure numerics, unlike the rear wheel's 8-fold
  ripple, which is measured real geometry (`omni-wheel-protocol.md` §1) and
  does not move with tessellation at all.
- **What the change cost, all checked rather than assumed:** red set unmoved at
  7 failed / 217 passed (suite 262 s → 161 s), eval grid 0.764 → 0.760 at
  survival 1.00 (29.4 s → 16.9 s), righting sequence / fall attitudes /
  inverted drops / hockey all unchanged, deploy bundle re-exported
  (`7af0ce42dfc91154`). Every `moves/*` export trained before it warns on load.
- **GATED NOW: a policy trained at 4e-4 matches the 2e-4 baseline.**
  `general_rl_glide_pitch_dt4e4`, 6M steps on unmodified
  `config/rl_general_glide_pitch.yaml`, scores **track_geo 0.886 / survive
  1.00** against `general_rl_glide_pitch_og`'s identical 0.886 at 2e-4.
  Head error improved 4.7° → 2.6°; crab stayed symmetric (.242/.245 vs
  .248/.262). The righting handoff still catches the bike on both mechanisms
  (arm 81°/1.12 s, wings 88°/0.61 s — same as the pre-change baselines).
  Two caveats worth keeping: the budgets differ (baseline 4M, this 6M), and
  it is ONE seed. `drift_m` regressed 0.588 → 1.392 and `turn_asym` 0.102 →
  0.153; both are underdetermined in the reward, so treat them as noise until
  a second seed disagrees.
- **`contact_solref` outweighs all of it**: at fixed geometry it swings contact
  loss 53% → 0% and peak load 5×. Still a `GUESS`. Also worth widening the
  randomizer's `dampratio_range` above 1.0 — it currently samples only the
  bouncy half, and overdamped is where filled TPU plausibly lives.
- **`timestep` and `contact_solref` are ONE decision, not two.** MuJoCo's
  `refsafe` (on by default) silently raises a positive `timeconst` to
  `2 × timestep`, so past `dt = timeconst/2` the sim stops modelling the
  contact that was configured — at `timeconst 0.005` that is `dt = 2.5e-3`, and
  static sink is bit-exact at every step below it. Well before that, peak force
  drifts once there are fewer than ~10 steps per contact time constant (the
  shipped pair sits at 12.5). **Two live consequences:** if the bench lands on
  a stiffer contact the timestep ceiling drops with it, and switching to the
  recommended **negative** solref convention removes the guard entirely — stiff
  pairs then diverge rather than being clamped. `(-4e4, -150)` reproduces the
  current positive pair exactly, if that conversion gets made.

---

## The wing LINKAGE — a second mechanism, and a real alternative

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

### What the optimiser taught, mostly by cheating

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

### Drivers vs driven

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

### Not decided

The linkage is **not** a replacement for the geared pair. Both are built, both
pass the fall set, and the choice is a real trade: gears have more torque margin
and a simpler part count; the linkage has a self-locking deployed pose and needs
no trajectory. Nothing downstream depends on the answer, so it can wait for the
mechanical design.


## CAD — the bike stops being parametric

Started 2026-08-18. Drawn in Onshape; `python -m aow_sim.cad_layout` exports the
component layout from the parameters, as YAML for reading and as a FeatureScript
Feature Studio for Onshape. Both are **exports** — generated, never edited, with
the regeneration command in the header.

`config/bike_params_cad.yaml` is a scratch copy of `bike_params.yaml` that the
CAD work edits freely. It eventually becomes the authoritative one. **Never pass
it to `export_deploy`**: `params_digest` hashes the whole tree, so a bundle built
from a diverged file carries a digest no bike matches, and refusing that is the
entire point of the check. Keeping the work here is also why none of it has
moved the digest — `deploy/bundle.npz` and all 23 `moves/*.npz` are still valid.

**What CAD has already sent back into the model.** This is the value of the
workstream and it arrived immediately:

- **`input_pulley_offset` was made up.** Its own comment said so — "placeholder
  until the mount/pulley design is done" — and nothing derived from it. Pinned
  from real belt geometry (9 mm HTD5M, 45T/15T on the 370 mm belts bought,
  centre distance 107.35 mm from the belt equation) it becomes 7.5 mm, the
  minimum a 9 mm belt allows over a 33 mm wheel. **Rear width 99 → 80 mm.**
  It reached 75 briefly, on the wheel clearance alone. The belt plane is now
  **derived as the larger of two clearances** — the pulley missing the wheel
  (24.0) and the drive-servo mount plate missing the pulley (26.5) — and the
  mount binds. Every millimetre of plate or of plate-to-pulley gap is two on
  the bike, and the servos themselves do not move for any of it. The 3 mm that bought is what makes the two servo cases
  symmetric about the centreline (±17.0), which is what lets a single flat plate
  on one side of the bike bolt to both of them.
- **The drive-servo mount is drawn, and it set the width.** A plate on EACH
  side takes both servos — they face opposite ways, so one plane per side meets
  one horn-side face and one back face — on the 22 x 40 case pattern with M2.5
  machine screws, sixteen in all. The P.C.D 16 on the horn side is on the
  ROTATING horn and there is no idler by default, so 22 x 40 is the only static
  pattern common to both faces. The second plate cost NOTHING in width: the
  belt plane had already been pushed out to clear the first. A four-walled
  sleeve joins them and carries the torque in bearing, so the sixteen screws
  only retain; one of its six faces is the ceiling in the print and gets
  deleted in CAD, which face depending on the build axis. `AOW mount` and
  `AOW planes` are their own features in the Feature Studio, hence their own
  tickboxes.
- **The servo gap is now zero.** The 2 mm was clearance for a packing solve in
  which nothing located the cases; the sleeve does, so a gap between them was
  slop. Separation re-solved 16.35 -> 15.2563 deg, and it has a closed form now
  that the cases are parallel. The fit clearance moved to the sleeve cavity,
  where FDM shrinkage actually lives.
- **The Onshape workflow is written down** in
  `docs/plans/cad-onshape-workflow.md`: what the platform will and will not
  name, how query variables replace naming, why a regenerated Feature Studio
  can be pasted over the old one safely, the API quota, and the three separate
  ways a ROBOTIS datasheet depth turned out not to be a face.
- **Seven planes are exported**, being where CAD actually starts. One is the
  fork's datum: it holds the axle direction and the raked steering axis at
  once, so it is the front view tilted back by 15 deg. Four are the belt runs,
  two per side — NOT mirror images, because the servos straddle 45 deg rather
  than sharing it. Two are prospective BUILD planes added 2026-08-21: the
  fork's datum offset 8 mm along its own normal, and the rear motor+dropout's,
  which lies in the lower servo's long outer face with its normal on the
  mount's tangential axis (135 deg), so the part builds up-and-rearward off a
  face that already exists rather than off a datum nobody can point at. All
  derived, none eyeballed.
- **The belts are solids now, and each side carries both of them.** Eight
  prisms: the four real runs (104.66 x 9 x 3.6 mm, inner face on the tangent
  line, opaque near-black) plus each run mirrored onto the other side
  (translucent magenta, its own suppressible feature node). A plane has no
  thickness and no ends, so it could never be checked against; the mirror
  exists because the 15.256 deg of straddle makes the two sides' keep-outs
  differ, and a chainstay that is the same part on both sides must clear both.

  The corridor arithmetic, in the frame that matters: measured as an angular
  station about the rear axle, each belt hull occupies 205.700 deg and the
  symmetric free window is **155.478 to 294.522 deg**, 139.044 wide, centred on
  exactly 225 = `drive_servo_angle_deg` + 180 (ray-sampled to confirm). In
  RUN-ANGLE — a different frame, 90 deg away — the bands are left 24.522 to
  50.222 and right 39.778 to 65.478, and they OVERLAP by 10.444 deg, so there
  is no threading between the two belts. Recorded, not drawn: a drawn sector
  would wrongly exclude a chainstay that ducks under both belts.
- **The steer servo was 10.17 mm off the steering axis**, which direct drive at
  `gear_ratio: 1.0` does not permit. Its position is now solved, not chosen.
- **The TM151 is 40 × 34 × 12.6 mm and 19 g**, against the 30 × 30 × 12 mm / 12 g
  placeholders the sim still carries.
- **The drive servos' separation is a solved 2D packing problem** — two
  rectangles free to rotate about their own shafts, separating-axis tested —
  not a guess. Now 15.2563° with the gap closed, and reducible to a closed form
  because the two cases ended up parallel; the alternatives are tabled in the
  config.
- **The self-righting linkage moved 75 → 130 mm** to clear the drive belts and
  then the servo cases. **`analysis/linkage_through_belt.py` (2026-08-21) says
  most of that is recoverable, and cheaply.** The belt is genuinely what binds
  today — belt-limited minimum station 123.5 mm, so ~6.5 mm of the 130 is
  margin — but delete the belts and the floor is 109.0, set by the mount
  sleeve. Sweeping tooth counts and belt lengths with the servo cases carried
  radially along with the centre distance:

  | change | station | won | ratio | top speed |
  |---|---|---|---|---|
  | as built, 45T/15T on 370 mm | 123.5 | — | 3.00 | 1.06 |
  | **same pulleys, 340 mm belt** | 111.0 | 12.5 | 3.00 | **unchanged** |
  | **36T/12T on a 310 mm belt** | 105.0 | 18.5 | 3.00 | **unchanged** |
  | 32T/12T on a 290 mm belt | 98.0 | 25.5 | 2.67 | 0.94 |
  | 28T/12T on a 280 mm belt | 96.0 | 27.5 | 2.33 | 0.82 |

  It saturates at 96 mm — below ratio 2.33 nothing more is won, because that is
  where the servo cases meet the rear wheel rather than where the belt runs
  out. **Not adopted, and nothing is changed by it.** Smaller pulleys at the
  same ratio, closer together, is the free lunch and the thing to check first;
  the cases are translated rather than re-solved, so any shortlist entry wants
  re-deriving through `cad_layout` before it is believed.

- **The Feature Studio pushes over the API now.** `--push` and `--shot` on
  `aow_sim.cad_layout`, one billable call each, with the document and tab ids
  in `config/onshape.yaml` and API keys in the macOS Keychain — never in this
  checkout, which is Dropbox-synced, where gitignored is not un-synced. Every
  call is logged with what it did (`python -m aow_sim.onshape --log`). The
  annual quota is 2500 and the cycle is anchored to **13 Oct**, not January and
  not the "Tracking start date" the usage page shows — that field disagreed
  with the same page's own elapsed-day count. Copy-paste still works and is
  quota-exempt, so exhausting the API strands nothing.

  **Usage at 2026-08-21: 34 / 2500, with 53 days left in the cycle** — roughly
  47 calls a day available, so the budget is not a live concern. It becomes one
  only if something polls: `getPartStudioFeatures` reads the document's actual
  tickbox and suppression state in one call, which is genuinely useful and is
  also the endpoint that would burn a year in a week if put in a loop. The
  ground rules are in `CLAUDE.md`, "Onshape — the CAD round trip".

**Two traps worth not re-learning**, both of which produced confident wrong
answers before the user caught them from the CAD:

- **A 2D projection is not an interference.** The battery reads as overlapping
  the drive pulley by 13 mm in side view and clears it by 1.00 mm in 3D — the
  pack is 35 mm wide and the pulleys start at 18.5 mm, so they never share
  lateral space.
- **The roof is a cylinder, so its constraint is radial.** "Stay below the roof
  axis" is a *sufficient* condition, not the real one, and using it understated
  the battery's headroom by 36 mm.

**Outstanding.** `bike_params_cad.yaml` still has the drive servos at their old
`[45, 30, 75]` — `cad_layout` derives the real position every run and prints it
but does not write it back, so a MuJoCo model built from that file is not yet
the layout the CAD shows. Electronics packing is deferred until the tethered
version's wire routing is understood.

---

## Tooling added

- **`src/aow_sim/cad_layout.py`** — the layout export, YAML and FeatureScript
  from one data model, with the frame conversion done once in code rather than
  per component by hand. `--righting {linkage,wings,none}`, `--bumpers`,
  `--chassis-box`, `--linkage-config`.
- **`analysis/wing_linkage.py` grew buildability metrics** — `stow_half_width`,
  `stow_roof_margin`, `pivot_crossover` — each added after an optimiser found a
  design that scored well and could not be made. Five opt-in flags
  (`--fit-envelope`, `--max-crank`, `--no-crossover`, `--crank-angle`,
  `--min-pivot`), all off by default so the committed configs stay reproducible.
- **`aow_sim.record` can test any bike** — `--params`, `--linkage-config`,
  `--mirror`, `--recover-deg`. The mirror flag matters because the policy's
  recovery is asymmetric (16.3° right vs 11.8° left) and a mechanism that works
  one way is not verified until both run.
- **`analysis/wing_linkage.py`** — the whole four-bar study in one file:
  kinematic solve, three optimiser objectives (kinematics / peak torque /
  self-locking deployed pose), quasi-static pin loads, and both the
  mechanism-frame and ground-frame animations. Reads its own config and touches
  nothing in `bike_params.yaml`, so it cannot move the params digest.
- **`analysis/contact_surrogates.py`** — the harness behind the above: a
  parameterised omni-wheel builder (cone meshes / spheres / capsules / smooth
  torus, any roller count) on a loaded carriage rig, plus the transfer arm that
  replays a trained policy across schemes. Its `cones-8` row must score
  identically to the unpatched model; that equality is the control.
- **`scripts/tb_summary.py`** — reads a running TensorBoard over its own JSON
  API, so watching a remote run needs no data sync and no change to how the
  board is launched. Default view bins the reward curve against the curriculum,
  because a falling reward is usually the curriculum getting harder rather than
  the policy getting worse and the two are indistinguishable apart. `--eval`
  prints the eval matrix, which is where slow regressions hide that `score`
  cannot see.
- **`aow_sim.record --script demo`** — one continuous run: balance, loop out
  under a pitch-blind policy, land, right, drive away. The policy is never
  switched off and is never overridden; the loop-out is a genuine failure from
  a real teleop input.
- **Visual fixes that apply everywhere**: checkered floor (0.25 m squares,
  `--grid` to override), directional sun, shadow coverage derived from the
  floor size (the stock `shadowclip` gave a ±2.4 m shadow box, so the bike
  stopped casting a shadow as soon as it drove anywhere), and a `corner` camera
  preset for runs that have both a pitch event and a roll event.
  `build_model.tune_lighting()` is shared by the recorder, teleop and the
  viewer so they cannot drift.

---

## Health: the LQR layer is knowingly red

`pytest` at HEAD: **37 failed, 182 passed, 2 skipped, 10 errors** in 141 s,
reproducible across runs. Was 7 failed / 217 passed / 2 skipped before the
drive plant was armed on 2026-08-21.

| file | red | what it is |
|---|---|---|
| `test_drive.py` | 19 | 7 trajopt (the old accepted set) + 12 gain-schedule/circle/heading |
| `test_hw_odometry.py` | 12 | 2 failed + 10 errors, all downstream of the schedule |
| `test_balance.py` | 8 | the LQR balance controller |
| `test_pivot.py` | 4 | pivots run on the scheduled LQR |
| `test_teleop.py` | 3 | teleop's analytic modes |
| `test_hw_replay.py` | 1 | stale `deploy/bundle.npz` — run `python -m aow_sim.export_deploy` |

**Only 7 of those 47 are the previously accepted set** (the trajopt `flick` /
`flip` moves, still queued for re-authoring once the as-built mass is known).
Everything else arrived with `drive_ki: 0.6`, and every one of them is in the
LQR layer: the gain schedule identifies at worst R² 0.941 against the 0.98
floor, and controllers designed on that fit fall over. **Every bare-plant test
in `test_model.py` passes**, so this is the reference baseline being broken, not
the physics — which is the one place the project can afford it, since RL is what
drives and LQR is the comparison.

**They are deliberately NOT in `tests/expected_failures.txt`.** They are not an
accepted cost; they are the visible price of arming a change whose
identification half is unsolved, and registering ~47 nodeids would convert an
open blocker into a background fact nobody looks at again. The registry will
shout `NEWLY RED` on every run until the identification is fixed or the plant is
disarmed. That is the intended behaviour. If it ever needs silencing, the entry
should say exactly that and carry a date — not name 47 tests individually.

**That acceptance mechanism is still machine-checked.** `tests/expected_failures.txt`
lists the seven trajopt nodeids with a reason and a date, and `tests/conftest.py`
ends every run with a verdict on whether the red set *moved* — `NEWLY RED`,
`UNEXPECTEDLY GREEN`, or `STALE ENTRY`. Still not an xfail, for the same reason
as the guard below: these tests run, fail, and exit non-zero. The registry only
judges.

### Which tests a change moves

Markers say what sends you back to a test, and are registered with their
descriptions in `pyproject.toml` (`pytest --markers`). `--strict-markers` is on,
so a misspelt one is an error rather than a silent empty selection.

| marker | run it after | tests | wall |
|---|---|---|---|
| `contact` | any `sim:` change — `contact_solref`, `timestep`, `mesh_segments` | 87 | ~99 s |
| `geometry` | any other `bike_params` change — a dimension, mass, gear ratio | 19 | 0.5 s |
| `spec` | changing a `control/*_spec.py` layout, or the `HardwareData` shim | 56 | ~60 s |
| `policy` | retraining or re-exporting anything in `moves/` | 6 | — |
| `deploy` | step 1 of the `bike_params` checklist: is the bundle stale? | 2 | — |
| `boundary` | touching imports under `hw/` | 19 | 0.0 s |
| `pure` | always — no model build, the inner loop | 50 | **0.24 s** |

`pytest -m pure` is the edit loop. `pytest -m contact` is what the timestep
change above should have been read against, and is most of the suite's
wall-clock.

Two standing guards, both deliberate and neither an xfail:

- `control/linearize.py` warns on any design whose worst fit drops under
  `MIN_FIT_R2` (0.98), naming the worst operating point. Silent right now.
- `test_lqr_model_fit_and_steering` **fails outright** when that happens. It is
  explicitly *not* xfailed: a non-strict xfail would report the regression as a
  green run, which is exactly the failure mode to avoid when the breakage is
  expected.

### Policy exports, current standings

| export | survive | track | vel_err | head_err | reverse | notes |
|---|---|---|---|---|---|---|
| `general_rl_og` | 0.92 | 0.702 | 0.225 | 9.9° | yes | 10M final |
| `general_rl` ← **config default** | 1.00 | 0.763 | 0.291 | 2.5° | **no** | 6M snapshot, pre-contact-change |
| `general_rl_1k` | 1.00 | 0.816 | 0.163 | 3.1° | — | loops out on reverse+180 (up_z −0.99) |
| `general_rl_smooth_og` | 1.00 | 0.816 | 0.167 | 3.6° | 1.006 | crab 0.21 / 0.30 |
| `general_rl_smooth_diff_og` | 1.00 | 0.800 | 0.166 | 4.3° | 0.983 | crab 0.21 / 0.28; best net v_lat (0.082) |
| `general_rl_smooth_stiff` | 1.00 | 0.805 | 0.191 | 3.0° | 1.122 | crab 0.39 / **0.02** |
| `general_rl_glide_og` | — | — | — | — | — | arm 1: oscillates 2×, but it is **pitch**; 23° nose-up |
| `general_rl_glide_pitch_og` | — | — | — | — | — | arm 2: `v_lat_frac` 0.12, `w_pitch` 2.0, 19-wide obs |

`control.general_move` still points at `general_rl`, which predates the contact
change and does not reverse. **This is a one-line fix and has been outstanding
across two snapshots.**

### Live training run

Last read 2026-08-14 at 6.01 M steps (`rl_general.yaml` with `v_lat_frac` 0.12
and contact damping 0.5): curriculum topped out at 4.997 M (83% through, vs
3.59 M for the previous run — the changes made it markedly harder to
bootstrap), reward plateaued at +3.9% over the final tenth. Against the
previous run it is **behind on every comparable axis** — score 0.685 vs 0.767,
survive 0.950 vs 1.000, `turn_asym` 0.319 vs 0.265 — with one genuine win,
`steer_rest_deg` 10.2 vs 19.2, which improved *within* the run (83 → 10) so it
is a real trend. Caveat: `v_lat_frac` changed the command distribution and the
plant changed too, so this is not a clean A/B. The training box was unreachable
at the time of writing, so this is the last reading, not the current state.

`turn_asym` has now sat at 0.2–0.32 across every run and has never improved
with more steps. It is not a training-length problem.

### Wings in the policy — tried four ways, answer is no (2026-08-15)

Four 5–6 M runs asked whether the general policy should observe and drive the
righting wings. It should not, and the reason is structural rather than a
tuning failure.

| run | wings available | `w_wing` | outcome |
|---|---|---|---|
| wings1 | from step 0 | flat 0.05 | total crutch: 89°, duty 1.0, **20/20 falls** forced stowed |
| wings2 | from step 0 | ramp → 1.0 | worse: deployed less but let roll reach 43° and got caught harder (24% of weight on the feet vs 9%) |
| wings3 | gated open 0.5–0.8 | ramped on `_diff` | never used — the two schedules collided, no free window |
| wings4 | gated open 0.4–0.6 | ramped 0.85–1.0 | **never used**, with a verified 0.57 M-step window where they were open and nearly free |

Available early and it never learns to balance; available late and it has no
use for them. **The cause is that episodes terminate at `fall_roll_deg` 60, so
the fallen state the wings exist for is outside the training distribution by
construction** — see "Still open" in `docs/plans/self-righting.md` for what
changing that would require. Do not re-open this by tuning the reward.

Two results worth keeping out of the detour:

* **`general_wings3_rl` does a genuine flick** — 211° of steer sweep in the
  BODY frame against only 94° in the WORLD frame, i.e. it rotates the bike
  around a wheel whose ground heading barely moves. Every other policy is the
  inverse (65–80° body, 206–221° world): it cranks the wheel and drives round.
  First policy in the repo to do the manoeuvre the flick move was authored for.
* **The crab ceiling is roll headroom, confirmed.** `crab_ratio` hit 0.65/0.70
  with the wings pinning roll to ~5°, and fell straight back to ~0.27 as they
  withdrew. Crab is bounded by the differential being spent on balance, not by
  the policy failing to learn it.

**Default is unchanged: the wings are operated separately from the policy.**
The scaffolding stays and is inert when off — `obs_wings`/`act_wings` default
false, `ActionBounds.wing_rate_max` defaults to 0.0 so every existing 3-arg
move yaml still constructs, and `rl_general.yaml` still yields obs 15 / act 3 /
`nu` 3 with no wings in the model. flick/pivot/ball are untouched.

---

## Critical path

Two tracks. The sim track does not wait on the build.

**CAD track (new, and does not wait on either of the others):**

1. **Draw the details.** The layout export is good enough to build on; the
   envelope will move as real structures appear.
2. **Feed the as-built numbers back into `bike_params_cad.yaml`**, then make it
   authoritative and retire `bike_params.yaml`. That is the step that turns
   `GUESS` into `measured` and moves the digest for the first time.
3. **Re-verify self-righting at whatever envelope it settles at.** The 75 mm
   layout reaches 13–14.5° against a 12° hand-off window, on all four fall
   cases — see the linkage section. Not a mechanism failure, but it needs
   `RECOVER_DEG` re-derived rather than assumed.

**Sim track:**

1. **Point `control.general_move` at a policy trained under the current
   contact model.** Outstanding for two snapshots; one line.
2. **Fix the eval score before spending another long run.** `_score =
   survive_rate × track` rose monotonically across exactly the span in which
   the 12M `smooth_bouncy_lat` run lost forward drive entirely, so `BestByScore`
   selected a policy that refuses a direction and called it the best of the
   run. Either add a directional term or gate `BestByScore` on
   `min(speed_ratio_fwd, speed_ratio_rev)` clearing a floor. **Every future run
   is exposed to this.**
3. **Enable the contact randomization that already exists.** `solref_frac` and
   `dampratio_range` sit commented out in `config/rl_general.yaml`. It depends
   on no measurement, which is the point — it decouples the largest sim-to-real
   risk from the build timeline. The survival cliff is close: eval survival is
   1.00 from dampratio 1.0 down to 0.5, then 0.9 at 0.3 and 0.7 at 0.2, and the
   first drop test puts the true value near 0.3.
4. **Resolve the crab question one way or the other.** Read arm 2 against its
   own exit criterion; if pitch is controlled and crab is still flat, run the
   open-loop gait sweep instead of a third arm.
5. **DONE — the new timestep is validated by a trained run.**
   `general_rl_glide_pitch_dt4e4` (6M, `rl_general_glide_pitch.yaml`, 34 min
   local at 32/512) scores track_geo 0.886 against the 2e-4 baseline's 0.886,
   survival 1.00 both, righting handoff intact. One seed and a different step
   budget, so a second seed would settle `drift_m` and `turn_asym`, which both
   regressed. `sim.timestep: 4e-4` + `mesh_segments: 64` need no further gate.
6. **Adopt `general_rl_glide_pitch_hub3` and retire the older glide arms.**
   `reward.w_hub_idle` prices hub MAGNITUDE at low commanded speed, faded on
   `max(commanded, measured)` speed with `hub_idle_v_scale: 0.6`. It is the
   first policy that is meaningfully calmer without being worse at anything:
   rim travel over a 15 s hold 7.58 -> 3.35 m, airborne 58% -> 13%, peak
   contact force 7.23 -> 3.66 x weight, and kick recovery 8/8 through dv 0.35
   where the baseline manages 7/8. Point `control.general_move` at it once it
   has been driven in teleop for more than a few minutes.
7. **The eval grid cannot see any of this, and that is the next real gap.**
   Across the four glide arms the selection score moved 0.886 / 0.898 / 0.915 /
   0.902 -- a spread of 0.03 -- while contact load moved 4x and the recovery
   envelope moved from 1/8 to 8/8. `_score` is survival x tracking over 20
   commands with randomization OFF, so it never disturbs the bike and never
   looks at the contact. Two additions, in order of value:
     - a DISTURBANCE arm (analysis/kick_recovery.py is the measurement; it
       needs to become a selection criterion, not just a report), and
     - a CONTACT-LOAD term, since "holds station by sawing the wheel 7 m" and
       "holds station" score identically today.
   This is a change to the trainer's scoring, so it re-bases every score in
   docs/ and in every moves/*.yaml. Decide whether the old numbers get
   re-measured or annotated before touching it.
8. **Un-block the two drive-plant fixes** (aow-contact-approximations.md §6b).
   `drive_kv` is 31x stiffer than the bare motor and there is no actuator lag
   at all; both single-parameter fixes break the suite, and it needs the
   Dynamixel velocity-PI emulation that mujoco-modeling-decisions.md deferred.
   Until then the plant lets a policy hold station by a means the hardware
   does not have -- which is what every hub-price arm above is working around
   rather than fixing.

**Build track:**

6. **Mechanical design and assembly of the full bike.** Carry the
   `bike_width` / `bike_height` envelope (120 × 165 mm) in at design time — the
   roof and the stowed wings both derive from it, so retrofitting means
   redoing both.
7. **Print a test pinion.** `min_pinion_radius` is a `GUESS` and the 5:1 gear
   ceiling hangs entirely off it. A mechanical-design input, not a follow-up.
8. **Weigh everything as built** — chassis (`GUESS` 0.45 kg), pack (0.115),
   electronics stack (0.076). Re-run `export_deploy`.
9. **Measure contact off the assembled bike** (§ below), then re-centre
   `sim.contact_solref` and re-check the policies with `analysis/chatter.py`.
10. **Bench verification 1–5** from `docs/plans/untethered-setup.md#verification`
   — loop timing, AHRS, odometry, failsafes. These gate first power-on.
11. **First untethered balance** on training wheels, then without.
12. **Self-righting**, deliberately last. The design is finished and verified in
    sim; it should not consume build time until the bike balances.

---

## Physical tests, adapted to "full bike, no rigs"

> **Correction — read before taking any contact measurement.** `solref`'s two
> numbers are not separately identifiable, and the protocol docs still say they
> are. MuJoCo's positive-solref form gives `b = 2/(d_width·timeconst)` and
> `k = d(r)/(d_width²·timeconst²·dampratio²)`, so **`timeconst` sets damping
> AND stiffness, while `dampratio` sets stiffness only** (as `1/dampratio²`).
> Dropping dampratio 1.0 → 0.5 does not reduce damping — it makes the contact
> **4× stiffer**, which is what makes it bounce.
>
> Two consequences. (a) A static reading fixes only the **product**
> `timeconst·dampratio`; the static and drop tests must be solved jointly, and
> `contact_solimp.dmin` is a third unknown on top. (b) **The "timeconst 0.020
> is ruled out" conclusion is withdrawn** — it came from a table generated at a
> hardcoded dampratio 1.0 while the config shipped 0.5. The tool is fixed;
> `docs/measurements/contact-protocol.md` and `contact-measurements.yaml` still
> carry the old tables and **need regenerating**.
>
> Cheapest real fix: switch to the **negative** convention,
> `solref: [-stiffness, -damping]`, which MuJoCo's own docs recommend for
> system identification. That genuinely decouples the two.

1. **Static load-deflection.** Park the bike, add known mass, measure **rear
   axle height above the floor** loaded vs unloaded, same spot and roll phase.
   Referencing the axle keeps frame flex out of the reading. Either lift the
   front onto a support so all added mass goes through the rear, or record the
   split. Still the single highest-value measurement.
2. **Drop test, rebound height.** `e = sqrt(h1/h0)` off a 240 fps clip, **on
   the bare rolling chassis before the pack and electronics go on.** The
   observed 2–3 bounces already falsify a shipped `dampratio` of 1.0.
3. **Print a test pinion** and find the smallest that survives.
4. **Spin-down tests**, bike on a stand — driven hub and flicked roller, for
   joint damping and frictionloss. A stand is not a rig.
6. **Incline slide test** for sliding friction (`GUESS` 0.9).
7. **Weigh everything** at assembly.
8. **Known-circle drive** for front tire lateral stiffness. Needs a driving
   bike, so it lands after first balance.
9. **Bench: loop timing** at 1/2/3 Mbps, `latency_timer` 16 vs 1. Gate: p99
   tick jitter < 1 ms at 100 Hz.
10. **Bench: AHRS** at 460800 baud, quaternion against known orientations,
   age-of-data at the tick. Expect no data for ~3 s after power-on — ~30 s if
   the unit is still on its factory static-boot default.
11. **Failsafes, deliberately triggered** — WiFi kill, pack below LVC, bike
    laid on its side.

Roll-phase and surface dependence stay deferred; they check the mesh, not the
contact parameters. The wheel-only `--variant testbed` model has **no physical
counterpart under this plan** — a sim-side tool, not pending work.

---

## Decided vs. placeholder

**Decided and unlikely to move:** Pi Zero 2 W running the same Python source as
the sim, no ROS, no microcontroller. One 3S bus, bought balance charger,
perfboard splitter instead of the U2D2 hub. WiFi/UDP teleop with the laptop as
ground station. RL as the controller that actually drives; LQR as the
comparison baseline. Moves authored offline, replayed with numpy only.

**Decided this period:** the self-righting mechanism is a mirrored wing pair
with a direct wing-to-wing mesh, a derived `bike_width`/`bike_height` envelope,
and a roof ridge. It beats the single arm on every number and no longer has an
open geometry question — only `min_pinion_radius` to verify.

**Placeholder:** 17 parameters marked `GUESS` in `config/bike_params.yaml`, each
with a note on how to identify it. The load-bearing ones are contact friction,
chassis/pack/electronics mass, front tire lateral stiffness, and
`min_pinion_radius`.

**ARMED 2026-08-21, and the LQR layer is red because of it — the drive plant
was ~31x over-capable.** `drive_kv: 0.5` against a 0.016016 bare-motor droop, so
the modelled servo makes full stall torque at every speed and reverses 21.8
times a second while told to stand still. Found 2026-08-21.

The two "31x" defects in `aow-contact-approximations.md` §6b are **one number,
not two**: `input_armature / kv_droop = 3.0e-4 / 0.016016 = 18.7 ms`, which is
exactly the XC430's own electromechanical time constant. Setting `drive_kv` to
the droop value therefore fixes the stiffness AND the bandwidth together, and a
separate `drive_tau` knob would double-count. That is why none exists. At the
droop value the affine actuator also reproduces the whole torque-speed line by
itself — max force at max command is 0.5333 = `stall_torque / belt_ratio`
exactly — so the flat `forcerange` stops being a lie rather than needing a
speed-dependent clamp.

What `drive_kv` alone cannot do is hold: it is open-loop voltage control, and
the real firmware closes an integral term. So `actuators.drive_ki` now exists
and `build_model` builds a **velocity PI** when it is non-zero — a PI velocity
loop is a P *position* loop whose setpoint ramps at the commanded speed, which
MuJoCo expresses natively as `dyntype=integrator` with an affine bias. `ctrl`
still means commanded input-shaft velocity, so `nu` is unchanged and no caller
learns a new command; `na` goes 0 → 2.

**It now ships ARMED at `drive_kv: 0.016016`, `drive_ki: 0.6`** — a deliberate
choice to make `bike_params.yaml` the file to experiment in, taking the LQR
breakage as a known cost. `ki = 0` remains bit-exact with the old plant and is
still tested — the PI branch at ki = 0 would drop the `kv*ctrl` term rather than
reduce to the velocity actuator, so `build_model` branches rather than folding,
and `test_drive_ki_zero_is_still_the_p_only_plant` is the guard on that. It has
to keep working: every export in `moves/` was trained against the P-only form.
`test_shipped_plant_is_the_pi_form` catches a silent revert the other way.

Measured on the way in, replaying `general_rl_smooth_diff_og` told to hold
station for 15 s:

| plant | rim past the contact | hub reversals | bike drift | worst LQR fit R² |
|---|---|---|---|---|
| former `kv 0.5, ki 0` | 5.09 m | 21.8 /s | 0.35 m | 0.9727 |
| `kv 0.016016, ki 0` | 2.96 m | 8.8 /s | 1.33 m | 0.9757 |
| **shipped** `kv 0.016016, ki 0.6` | 3.25 m | 9.1 /s | 0.63 m | 0.9412 |
| `kv 0.016016, ki 0.855` | 4.38 m | 9.9 /s | 0.78 m | 0.9439 |

The reversal rate halves and stays halved; the integrator buys the
station-keeping back (drift 1.33 → 0.63 m) without giving the reversals up.
This is a *replay* of a policy trained on the shipped plant, so read it as what
the plant permits, not as what a retrained policy would do.

**Landing this also turned up a real bug that `na = 0` was hiding.** Five
hand-rolled resets — the four RL envs and `linearize._set_reduced_state` —
restore `qpos`/`qvel` but never `data.act`, so with an integrator every episode
and every identification rollout inherits the previous one's wind-up as an
unbounded hidden input. Cost when it leaks: worst gain-schedule fit R² 0.9727 →
**0.7543**. All five now call `build_model.reset_actuator_state`, which is a
no-op at `na = 0`; that recovers **0.9412**.

**OUTSTANDING, and it is the cost being paid to keep this armed.** 0.9412 is
under the 0.98 floor, and with gains designed on that fit the LQR layer falls
over — the 37 failed + 10 errors in the health section above. Every bare-plant
test in `test_model.py` passes throughout, so this is the reference baseline,
not the physics; that is what made arming affordable rather than reckless, RL
being what actually drives. It is still a real debt, not a shrug. The
residual is the genuine hidden-state problem: the integrator is a state the
8-state lateral model has nowhere to put. **A missing intercept was tested as
the explanation and disproved** — adding a constant column to the regression
changes the fit by nothing at all. Nor is a 9th LQR state the answer: on
hardware that integral lives inside Dynamixel firmware and the Pi cannot
observe it, so an LQR designed on it would use a signal the bike does not have.

So the open question is how to identify a plant whose actuator carries memory
the controller cannot see — not a parameter, and not yet solved.
`analysis/wheel_slowmo.py --drive-tau` predates all of this and now
double-counts the bandwidth; prefer overriding `drive_kv`/`drive_ki`.

**None of the numbers above are measured.** `drive_kv 0.016016` is derived from
the datasheet block, and is a LOWER bound — firmware KVP makes the real loop
stiffer. `drive_ki 0.6` is the Dynamixel X-series default `KVI/KVP` ratio at an
assumed 1 kHz loop; `0.855` is `Ti = tau_m`. `docs/measurements/servo-protocol.md`
§4 measures both directly, and its `settling_time_s` is `Ti`, i.e. `ki`. §1's
large-step slew matters more than that document implies, because it measures
`input_armature` — still a `GUESS` at 3.0e-4 — and the whole bandwidth result
is `tau = J/kv`.

---

## Risks, ranked

1. **No trained policy has ever seen contact-stiffness variation.** The
   randomization axis exists but is disabled, so every export sits on one point
   of a curve that falls off a cliff just below the current nominal. Most
   likely single cause of a policy that works in sim and not on the floor.
2. **The eval score cannot see a policy trading away a direction.** Demonstrated
   once already, at a cost of 12M steps. Unfixed, and every future run is
   exposed.
3. **Authority derating.** Real servos will not deliver modelled torque at
   modelled bandwidth. Called out in `untethered-setup.md` and not yet
   quantified.
4. **Left/right asymmetry, and crab.** `turn_asym` has not moved below ~0.2 in
   any run regardless of length, and crab is one-sided in every champion. The
   plant is mirror-symmetric (`axle_cant_deg` is 0.0, measured), and the
   handedness **flips sign between policies** — the cleanest available evidence
   that this is spontaneous symmetry breaking, not plant asymmetry. Fixing it
   widens the recoverable set and reduces how often righting is needed at all.
6. **Steer homing at power-up is undesigned.** The XC330 loses its multi-turn
   count across a power cycle, and the fix depends on an unmade mechanical
   choice (hard stop vs magnetic index). Blocks first power-on, not ordering.
7. **Front-wheel liftoff is undetected by the estimator.** The lateral estimator
   assumes the front wheel is down. `analysis/liftoff.py` now measures how
   often that is false — 79 mm of clearance on arm 1 — so this is quantified
   rather than suspected.
8. **The contact protocol docs are stale.** They still describe wheel-only rigs
   and carry tables generated under the wrong `dampratio`. Anyone following
   them literally will take the wrong measurement.

---

## Explicitly not being worked on

The PD cascade (legacy, kept compiling as a manual-debug fallback). The trajopt
moves `flick`/`flick_fwd`/`flip` — the 7 remaining test failures — which will be
re-authored once the as-built mass is known rather than twice. Roll-phase and
surface contact studies. A live gamepad front-end. The ball-shot move, which
works and is not on the critical path. Re-tuning the LQR weights, deliberately
deferred until the contact model is pinned.
