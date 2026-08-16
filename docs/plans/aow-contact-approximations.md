# Simplified AOW contact models — survey and measurements

`docs/plans/mujoco-modeling-decisions.md` closes its known-simplifications
section with:

> Fast approximation models (anisotropic-friction capsule à la Ekumen's LeKiwi
> omni-base, or a ballbot-style reduction) deferred until this reference model
> is validated — note the capsule trick only handles *passive* rollers, so an
> active-roller approximation will need actuated lateral contact velocity.

This is that deferred survey, run 2026-08-15. The motivating worry was that we
might be leaving a 10× speedup on the table, and that policies might be
learning contact behaviour specific to 8 discrete rollers and their gaps.

Everything below is reproducible: `python analysis/contact_surrogates.py`.
Numbers are from an M4 laptop, single-threaded, on a loaded-carriage rig that
drives the omni wheel alone at ~1 m/s under 0.60 kg — the rear-axle share of
the bike. The rig's rear-wheel contact-loss numbers agree with the whole bike
under the LQR at the same speed (5.8% vs 5.9% at 0.5 m/s), which is the reason
to trust it.

**Headline: the 10× is real but it is not in the contact model.** It is in
`sim.timestep` and it is available today without changing any geometry.

---

## 1. Where the step time actually goes

| rear wheel | nv | ngeom | neq | ksteps/s | speedup |
|---|---|---|---|---|---|
| 8 axles × 2 cone **meshes** (shipped) | 15 | 22 | 10 | 123 | 1.00 |
| 8 axles × 2 spheres at the ridge stations | 15 | 22 | 10 | 142 | 1.15 |
| 8 axles × 1 sphere | 15 | 14 | 10 | 151 | 1.22 |
| 8 axles × 1 capsule | 15 | 14 | 10 | 139 | 1.12 |
| smooth torus, no rollers at all | 7 | 7 | 2 | 215 | 1.74 |

The convex-hull mesh collision is **not** the expense. Replacing all 16 cone
meshes with analytic primitives buys 10–20%. Deleting the entire roller
multibody — 8 hinge DOFs and 8 equality rows — buys 1.7–2.0×, and that is the
hard ceiling on "make the contact geometry cheaper". Cost scales roughly as
1/n_axles (4 axles: 1.6× faster; 16: 0.6×; 32: 0.2×), which says the same thing
from the other side: you are paying for the constraint solve, not the collision.

**So: no approximation of the contact *shape* can be worth more than 2×.**

## 1b. The policy does not care which roller model it gets

The experiment that actually answers "are policies learning 8-roller contact
detail": `general_rl_smooth_stiff`, unmodified weights, the standard 20-command
eval grid, randomization off, only the rear wheel swapped.

| rear wheel | score | survive | track_geo | vel_err | head err |
|---|---|---|---|---|---|
| unpatched shipped model | 0.764 | 1.00 | 0.764 | 0.219 | 4.1° |
| cones-8 (this file's fork) | 0.764 | 1.00 | 0.764 | 0.219 | 4.1° |
| cones-16 | 0.780 | 1.00 | 0.780 | 0.167 | 4.5° |
| capsules-8 | 0.762 | 1.00 | 0.762 | 0.217 | 3.6° |
| spheres2-8 | 0.750 | 1.00 | 0.750 | 0.250 | 3.5° |
| spheres1-8 | 0.722 | 1.00 | 0.722 | 0.258 | 6.9° |

The fork reproduces the shipped builder bit for bit, which is the control.
Beyond that: **survival is 1.00 on every scheme**, and tracking moves by at
most 5.5%. The ordering follows §4's ride roughness — spheres1-8 is both the
roughest ride and the worst score, cones-16 is both the smoothest and the best
— so what little the policy loses it loses to a bumpier road, not to a contact
feature it had memorised.

This is the good news and it should be read as such: **the blind spot the
survey was looking for does not appear to be there.** A caveat worth keeping:
one policy, one grid, and all of these schemes share the same 8-fold (or
finer) periodic envelope. It says the policy is insensitive to roller *shape*;
it does not test a genuinely smooth wheel, because no smooth wheel can crab.

## 1c. Where the roller machinery's 2× actually goes — and why it can't be slept

Only 1–2 of the 8 rollers can touch the floor at once, which makes "can the
other 6–7 be made cheap?" the obvious question. Ablation says no, and the reason
is worth writing down because it is a property of the engine, not of the wheel.
`analysis/contact_surrogates.py --study ablate`; the `ghost` modes keep the
roller bodies but stop them colliding and let a torus carry the contact, so the
contact is held identical while the machinery varies.

| variant | nv | neq | µs/step | |
|---|---|---|---|---|
| `cones` (shipped) | 15 | 10 | 7.76 | |
| `free` (no equalities) | 15 | 2 | 6.96 | |
| `ghost` (rollers non-colliding) | 15 | 10 | 7.65 | |
| `ghostfree` (…and no equalities) | 15 | 2 | 6.87 | |
| `torus` (no rollers at all) | 7 | 2 | 3.73 | |

Splitting the 3.92 µs of roller machinery (`ghost` − `torus`):

- **8 DOFs: 3.14 µs (80%)**
- 8 equality rows: 0.78 µs (20%)
- 16 colliding roller meshes: 0.11 µs (3%) — **free**, which is the same
  conclusion §1 reached from the other side.

The stage breakdown is the interesting part. Of the 3.14 µs the DOFs cost,
**2.04 µs is inside `CONSTRAINT`** — a stage those DOFs do not participate in,
since `ghostfree` has no roller equalities at all. Forward kinematics accounts
for 0.29 µs and inertia for 0.16. The constraint Jacobian is `nefc × nv` and the
Newton solve runs over every DOF, so **a DOF that nothing constrains still taxes
the solver.**

Sleeping does not rescue this, though not for the reason it first appears — see
§7: MuJoCo 3.10 *does* have island sleeping, but the bike is a single island
containing every roller DOF, it never goes quiet enough to sleep, and the
gearbox's tendon equalities make the model refuse the flag outright.

**The deeper point, though, is that those 8 DOFs are not real freedom.**
`roller_i = k · ring_rel` exactly, so the mechanism has *zero* degrees of freedom
beyond hub + ring: the shipped model is a redundant-coordinate formulation
(15 DOFs + 8 constraints) of a 7-DOF system. The exact reduction is standard for
gear trains — a joint whose coordinate is a fixed linear function of another,
resolved at the coordinate level rather than by the solver. MuJoCo has no such
primitive (a body's joints are relative to its parent, and there is no
cross-tree gear coupling), which is why this is expressed as `mjEQ_JOINT`
equalities instead. Half of the reduction *is* available and exact — the rollers'
spin kinetic energy is ½(n·I_r·k²)·ω², i.e. **1.066e-5 kg·m² of `armature` on
`ring_spin`** — but it buys nothing on its own, because what forces the bodies to
exist is contact: MuJoCo derives contact-point velocity from body twists, so
lateral surface velocity requires a body actually spinning about the axle. Same
wall the torus hits.

And they are not idle in any case. Geared at k = 2.4, the 8 rollers reflect
**5.2× the ring gear's own inertia** onto the crawl channel — though only 3.5% of
the servo's reflected rotor armature (3.0e-4 kg·m²), which dominates everything.
So the load implication is real but small, and a reduced model that lumped it
wrongly would be making a 3.5% error at the input shaft.

Bottom line for the theoretical exercise: the ceiling is the 2.08× that `torus`
sets, it is all DOFs, none of it is reachable without giving up crab, and 1.7× of
it was already taken from the timestep (§2) at no modelling risk. The two
mechanisms that look like they should help — island sleeping and MJX — are
scoped in §7; neither is available today.

## 2. The 10× is in the timestep

Contact statistics at ~1 m/s, 8 cone rollers, nothing else changed:

| dt | ride swing | off ground | peak Fn/W | rest sink |
|---|---|---|---|---|
| 5e-5 | 0.822 mm | 31.1% | 6.80 | 0.381 mm |
| 1e-4 | 0.830 | 30.9% | 6.81 | 0.382 |
| **2e-4 (shipped)** | 0.837 | 31.2% | 6.84 | 0.389 |
| 4e-4 | 0.865 | 30.8% | 6.89 | 0.397 |
| 6e-4 | 0.872 | 31.1% | 6.91 | 0.403 |
| 1e-3 | 0.962 | 32.6% | 7.11 | 0.429 |
| 2e-3 | 1.160 | 35.0% | 8.36 | 0.523 |

Converged from 5e-5 through 6e-4; drifting by 1e-3. Cross-checked on the whole
bike with the LQR balance scenario (3° tilt, 10 s): survives and settles to the
same 0.022 m drift and ~0.1° tail roll RMS at 2e-4, 4e-4, 6e-4 and 1e-3, and
falls at 2e-3.

And cross-checked where it matters most — the same trained policy on the same
eval grid, nothing changed but `sim.timestep` and `sim.mesh_segments`:

| timestep | segments | score | survive | track_geo | grid wall time |
|---|---|---|---|---|---|
| **2e-4 / 32 (shipped)** | | 0.764 | 1.00 | 0.764 | 27.5 s |
| 4e-4 | 32 | 0.767 | 1.00 | 0.767 | 15.1 s |
| 6e-4 | 32 | 0.760 | 1.00 | 0.760 | 10.6 s |
| 1e-3 | 32 | 0.758 | 1.00 | 0.758 | 7.1 s |
| 2e-4 | 64 | 0.763 | 1.00 | 0.763 | 28.5 s |
| 6e-4 | 64 | 0.758 | 1.00 | 0.758 | 11.0 s |

**2.6× at 6e-4 for a 0.5% score change; 3.9× at 1e-3 for 0.8%** — both inside
the spread between the roller variants in §1b, i.e. inside the noise this eval
can resolve.

### What actually bounds it — and it is not the contact

**Landed at `sim.timestep: 4.0e-4`, not 6e-4.** Neither the contact statistics
nor the policy eval can see the binding constraint. The LQR's finite-amplitude
system ID can: it fits one linear model per grid speed, and larger steps make
the plant look less linear at exactly the point that was already worst.

| dt | worst fit R² | at | `test_gain_schedule` (floor 0.95) |
|---|---|---|---|
| 2e-4 | 0.9748 | +0.25 m/s | pass |
| 3e-4 | 0.9744 | +0.25 | pass |
| **4e-4** | 0.9727 | +0.25 | pass |
| 5e-4 | 0.9718 | +0.25 | pass |
| 6e-4 | **0.9408** | +0.80 | **fail** |

The collapse is local: at 6e-4 every other speed is still ≥ 0.986 and only
+0.80 m/s falls out. 4e-4 leaves the fit indistinguishable from 2e-4 (0.9727
vs 0.9748) and still buys most of the speedup. Going further would degrade the
reference baseline to buy wall-clock, which is the wrong trade for a knob that
buys nothing physical — the accepted degradations in this repo are all in
exchange for *better* physics.

**Measured after landing 4e-4 + `mesh_segments: 64`:**

- `pytest`: **7 failed / 217 passed** — bit-identical to the red set before the
  change (the accepted flip ×2 + flick ×5 trajopt group). Wall clock 262 s →
  161 s.
- Eval grid: score 0.764 → 0.760, survival 1.00, wall 29.4 s → 16.9 s (1.74×).
- Impact cases, all re-run rather than assumed: righting sequence with wings
  (fell to 88°, handover 0.63 s → 0.61 s, ends upright and balancing), single
  arm (81°, 1.11 s → 1.12 s, upright), 12 real falls per bumper geometry
  (resting roll 70.7–70.8° and 81.0° unchanged, spread 0.0°, touchdown KE
  156–313 → 162–366 mJ against the same barriers), 10 inverted drops (0/10 stay
  inverted, unchanged), and the ball/hockey suite green throughout.
- Deploy bundle re-exported (digest `7af0ce42dfc91154`).

Remaining caveats:

- Every trained policy predates it and is now an artifact of a different plant;
  `check_move_digest` says so on load. Replay surviving the change is necessary
  but not sufficient — it does not show that a policy *trained* at 4e-4
  transfers back.
- `params_digest` hashes the whole parameter file, so a pure solver knob
  invalidates the deploy bundle and every move's digest exactly as a physical
  measurement would. That is conservative in the right direction, but it does
  mean "digest changed" carries no information about *what* changed.

## 2b. `timestep` and `contact_solref` are the same decision

The timestep is not independent of the contact model. Two couplings, one hard
and silent, one soft and gradual. Reproduce with `analysis/contact_surrogates.py
--study refsafe`; to watch it, `analysis/wheel_slowmo.py --compare 4e-4,3e-3
--slowmo 5` stacks two timesteps in one clip on a shared frame clock and a
shared clearance scale.

**The hard one: MuJoCo silently raises your timeconst to `2 × timestep`.**
That is `mjDSBL_REFSAFE`, on by default (`disableflags == 0` here). Measured at
`timestep 1e-3`, so the floor is `2e-3`:

| timeconst | refsafe ON, rest sink | refsafe OFF |
|---|---|---|
| 2.5e-4 | 0.0096 mm | diverges |
| 5e-4 | 0.0096 | diverges |
| 1e-3 | 0.0096 | diverges |
| 1.5e-3 | 0.0096 | 0.0054 |
| 2e-3 | 0.0096 | 0.0096 |
| 5e-3 | 0.0597 | 0.0597 |

Every timeconst at or below `2 × dt` produces the *same* contact, because it
is not the contact you asked for — it is `2 × dt`. Read the other way, at the
shipped `timeconst 0.005` the static sink is **exactly** 0.0597 mm for every
timestep from 1e-4 through 2.5e-3, and only moves at 3e-3 where `2 × dt`
crosses 0.005. So while you stay under the clamp, contact stiffness is
genuinely timestep-independent; once you cross it, the model softens without
warning. Current margin: `2 × 4e-4 / 0.005 = 0.16`, i.e. clamping would start
at `dt = 2.5e-3`.

**The soft one: the step has to resolve the contact's own time constant.**
Long before the clamp, peak contact force drifts. The governing number is
`timeconst / timestep` — steps per contact time constant — and it holds across
timeconst, which is what makes it a rule rather than a coincidence:

| timeconst | steps/tc at which peak force is still within 2% | clamp at |
|---|---|---|
| 0.002 | ≥ 10 (dt ≤ 2e-4) | dt = 1e-3 |
| 0.005 (shipped) | ≥ 12 (dt ≤ 4e-4) | dt = 2.5e-3 |
| 0.020 | ≥ 33 (dt ≤ 6e-4) | dt = 1e-2 |

Roughly **ten steps per contact time constant**. The shipped pair sits at 12.5.

**Two consequences that matter for work already queued.**

1. **The contact bench measurement can move the timestep ceiling.** If
   `contact-protocol.md` lands on a *stiffer* contact — timeconst 0.002, say —
   then today's `4e-4` is only 5 steps/tc and already 4% off, and refsafe would
   start clamping at 1e-3. The timestep is not a setting to fix now and forget;
   re-check it when `contact_solref` is identified.
2. **Moving to the negative convention removes the guard.** `status.md`
   recommends `solref: [-stiffness, -damping]` to decouple the two numbers for
   system ID. Measured: refsafe does not apply, and a stiff pair simply
   diverges instead of being clamped —

   | solref | dt 2e-4 | 1e-3 | 3e-3 | 5e-3 |
   |---|---|---|---|---|
   | (-8e3, -60) | 0.273 mm | 0.273 | 0.273 | 0.279 |
   | (-2e5, -400) | 0.012 | 0.012 | 0.012 | diverges |
   | (-1e6, -900) | 0.0024 | 0.0024 | diverges | diverges |

   Arguably an improvement — loud failure beats a silently softened contact —
   but it means the timestep stops being safe by default. `(-4e4, -150)`
   reproduces the shipped positive pair's 0.0597 mm sink exactly, which is a
   useful anchor when the conversion is made.

## 3. Why the obvious one-contact surrogate is unavailable

The tempting reduction is a **ball wheel**: one sphere at the rear axle, two
driven rotational DOFs, one contact point, no rollers. It is wrong, and not
subtly.

Lateral offset of the contact patch from the wheel centre-plane, per degree of
lean:

| lean | thin disc, R·sin φ | ball | shipped 8-roller AOW | torus surrogate |
|---|---|---|---|---|
| 5° | 4.46 mm | 0 | 4.44 | 4.45 |
| 10° | 8.89 | 0 | 6.81 | 6.16 |
| 15° | 13.25 | 0 | 11.07 | 10.53 |
| 20° | 17.51 | 0 | 13.71 | 14.83 |

The real wheel is a **thin disc**: leaning walks the contact *outboard, away
from the lean*, so the bike is an inverted pendulum pivoting at the ground and
the toppling lever arm is `h_com · sin φ`. A ball keeps the contact under the
axle, pivoting the pendulum at the axle instead: the arm becomes
`(h_com − R) · sin φ`. With h_com ≈ 0.12 m and R = 0.0512 m that is **42% less
destabilising torque**, and a capsize timescale off by ~24%. A ball-wheel bike
is a different bike, and any policy trained on it would be trained on the wrong
one.

(The shipped wheel undershoots the ideal thin disc past ~10° because it is only
33 mm wide and the contact runs out onto the cone taper. That is real geometry.)

**The anisotropic-friction capsule is also unavailable**, and for a MuJoCo
reason rather than a physics one. On a flat floor the contact tangent frame is
world-locked: for a plane contact MuJoCo builds the frame from the normal, so
`t1` is world +Y at every wheel yaw (verified: 0°/30°/60°/90° all give
`t1 = [0,1,0]`). Friction anisotropy therefore cannot follow the axle of a
wheel that changes heading. The LeKiwi trick works for a fixed-heading omni
base; it does not survive a steering bike without rewriting `d.contact.frame`
every step. That closes the option recorded in `mujoco-modeling-decisions.md`
— it should be marked rejected, not deferred.

What is left, if a smooth continuous omni contact is ever wanted: a driven
lateral surface velocity has to come from a body that actually rotates, so the
roller multibody is not optional in MuJoCo. The remaining routes are outside
MuJoCo's contact solver — a reduced-order analytic bike+omni ODE (fast, JAX-
vectorizable, big fidelity commitment; useful for controller screening, not for
final policies), or MJX with primitive geoms for GPU-batched envs, which is a
much larger throughput lever than anything in §1 and is the thing worth
scoping if raw sample count ever becomes the binding constraint.

## 4. The 8-roller chatter is real, and simplifying makes it worse

Rolling at ~1 m/s:

| rear wheel | ride swing | off ground | peak Fn/W |
|---|---|---|---|
| cones-4 | 13.696 mm | 80.2% | 22.69 |
| **cones-8 (shipped)** | 0.837 | 31.2% | 6.84 |
| cones-16 | 0.091 | 16.0% | 4.17 |
| cones-24 | 0.128 | 30.8% | 3.89 |
| spheres1-8 | 1.680 | 63.9% | 10.96 |
| spheres2-8 | 1.715 | 64.0% | 11.06 |
| capsules-8 | 0.632 | 39.8% | 7.55 |
| torus (no crab) | 0.104 | 30.2% | 5.33 |

Read the two columns together, per `analysis/liftoff.py`: cones-24 breaks
contact as often as cones-8 but swings 0.13 mm doing it rather than 0.84 mm.

Two things follow.

**The ripple is not an artefact.** `docs/measurements/omni-wheel-protocol.md` §1
measures Ø102.35 max / Ø101.75 min on the real wheel — 0.60 mm — and says
plainly that rolling bumpiness is real physics the sim reproduces. The 0.837 mm
here is that 0.60 mm plus the dynamic overshoot it causes. Smoothing it by
fiat would be *removing* fidelity, not adding it.

**Every geometric simplification roughens the ride.** One sphere per axle is
twice as rough as the cone pair it replaces, because the cone's contact line
sweeps across stations as the wheel turns and that averages the envelope. So
the simplification directions are not free even in fidelity terms — they cost
speed-neutral accuracy.

## 5. What actually controls the chatter: `contact_solref`

8 cone rollers throughout, only the contact model varying:

| solref (timeconst, dampratio) | rest sink | off ground | peak Fn/W |
|---|---|---|---|
| [0.005, 0.5] | 0.015 mm | 53.4% | 7.77 |
| **[0.005, 1.0] (shipped)** | 0.060 | 31.2% | 6.84 |
| [0.01, 1.0] | 0.225 | 14.3% | 3.82 |
| [0.01, 2.0] | 0.593 | 8.9% | 3.73 |
| [0.02, 2.0] | 1.651 | 9.9% | 2.12 |
| [0.04, 2.0] | 3.952 | 0.0% | 1.46 |

`contact_solref` swings contact loss from 53% to 0% and peak load by 5× at
*identical* geometry. It moves the thing we were worried about far harder than
the roller model does — and it is still tagged `GUESS`, with the bench tests
that would pin it only specified, not run
(`docs/measurements/contact-protocol.md`).

Note the direction: **overdamped** (dampratio > 1) is where filled TPU
plausibly lives and is where the bouncing goes away at a sane sink depth. The
domain randomizer currently draws dampratio from 0.2–1.0 — only the bouncier
half of the plausible band, and by `control/randomize.py`'s own note that is
the axis policies are most brittle to.

**Conclusion for the original worry.** "Are policies learning weird contact
physics from the 8 rollers and their gaps?" gets two answers, and neither one
argues for a surrogate. Empirically (§1b) the policy does not care which roller
model it is given. Structurally, the 8-fold bumpiness is real and measured, so
there is nothing to remove. What *is* unpinned is how hard the wheel reacts to
it — and that is one bench measurement, not a model rewrite.

## 6. One unrelated bug this turned up

The front tyre is a smooth crowned mesh, so any contact loss it shows is pure
tessellation. Full bike, LQR, driving straight at 0.5 m/s:

| mesh_segments | facet sagitta | front off ground | front ride swing | rear off ground |
|---|---|---|---|---|
| **32 (shipped)** | 241 µm | 17.5% | 0.258 mm | 5.8% |
| 64 | 60 µm | 0.2% | 0.064 mm | 5.5% |
| 128 | 15 µm | 0.0% | 0.031 mm | 5.6% |

A round wheel bouncing off its own facets 18% of the time, at a swing that
matches the facet height. The rear column is the control — it barely moves,
because the omni wheel's ripple is geometry and does not care about
tessellation. Cost of `mesh_segments: 32 → 64` on the full bike: ~3% of step
time (128 costs 19%).

This is free fidelity and should probably just be taken, but it is a physics
change like any other: every existing policy trained against a front wheel
that buzzed.

---

## 7. The two things that look like free speedups and are not

**Island sleeping (`mjENBL_SLEEP`).** MuJoCo 3.10 does have it — islands are on
by default (`mjDSBL_ISLAND` is a *disable* flag) and sleeping is opt-in via
`mjENBL_SLEEP` + `opt.sleep_tolerance` (default 1e-4). It is the obvious
candidate for "only 1–2 rollers matter at a time". It does not apply here, for
three independent reasons, in increasing order of how fundamental they are:

1. **The model refuses to run with it.** `mj_wakeEquality: tendon equality does
   not yet support sleeping` — the gearbox's two `mjEQ_TENDON` constraints. That
   one is incidental and would lift if MuJoCo adds support, or if the 2×2 mix
   were applied in software and hub/ring driven directly.
2. **There is no subset to sleep.** Measured standing on its wheels, the bike is
   `nisland = 1`, `ntree = 1`, and all 8 roller DOFs report `dof_island == 0` —
   the same island as the chassis freejoint. The equality constraints that make
   the rollers turn are precisely what welds them into the chassis's island, and
   sleep granularity is the *tree*, which here is the entire bike.
3. **Nothing ever goes quiescent.** The whole point of §4 is that this wheel
   buzzes: the rear contact is intermittent 30–50% of the time and residual
   speeds sit around 1e-1, three orders above `sleep_tolerance`. A balancing
   bike is the worst possible sleep candidate.

Where it *could* pay is the fallen bike in the righting studies, which really
does lie still — but that is seconds of a study, not the training loop.

**MJX / GPU.** This is where "8 identical subtrees are duplicate work" would
actually cash out, and it is the right long-term answer to throughput. But it is
a port, not a flag, and three things have to be true first — none of which are
today:

- **It needs a GPU.** MJX's win is `vmap` over thousands of envs on one device;
  a single MJX env is typically *slower* than C MuJoCo, and MJX on CPU is not a
  win at all. The remote box is a 16C/32T Threadripper with no GPU, and the
  laptop is Apple silicon, where JAX support is experimental. Neither machine
  can show a gain.
- **It replaces the training stack**, not just the model: SB3 + `SubprocVecEnv`
  gives way to batched on-device rollouts.
- **Several of this model's choices are exactly the ones MJX restricts.** Worth
  checking before any scoping: `cone: elliptic` (MJX has favoured pyramidal),
  `condim: 4`, `integrator: implicitfast`, the 16 cone **meshes** plus the
  crowned tyre (MJX wants primitives and statically-bounded contact counts), and
  `mjEQ_TENDON` again. Unverified here — `mujoco-mjx` and `jax` are not
  installed, so this list is a checklist, not a measurement.

The one real connection back to §1: if MJX ever happens, the `spheres1`/
`spheres2` variants stop being pointless. They buy ~15% on CPU and are not worth
the fidelity argument there, but "primitives instead of meshes, statically
bounded contacts" is precisely what an MJX port would need — so that work is
prep for a port rather than a CPU optimisation.

## Recommendations

1. **Done: `sim.timestep: 4.0e-4` and `sim.mesh_segments: 64` are landed.**
   ~1.7× on the eval grid and the suite, replay score unchanged at survival
   1.00, the accepted red set unmoved, impact cases re-run, deploy bundle
   re-exported. The remaining gate is a **full training run** — everything
   above is replay, and replay cannot show that a policy trained at 4e-4
   transfers back. Do that before the next long run, not after. (§2, §6)
2. **Do not build a contact surrogate.** For speed the ceiling is 2×, dominated
   by DOF and constraint cost either way. For robustness there is nothing to
   buy: the policy already transfers across every roller scheme at survival
   1.00. And every candidate either breaks the lean geometry (ball), is
   inexpressible in MuJoCo (anisotropic capsule), or is rougher than what it
   replaces (spheres, capsules). (§1, §1b, §3, §4)
3. **Run the contact bench protocol.** `contact_solref` is a 5× lever on peak
   contact load and the single largest unknown in the model — far larger than
   anything the roller topology does. **Re-check `sim.timestep` against
   whatever it lands on**: the two are one decision, not two (§2b). (§5)
4. **Widen the randomizer's dampratio band above 1.0.** It currently samples
   only the bouncy half of the plausible range. (§5)
5. **Done: `analysis/chatter.py` handles any policy width.** It divided a
   4-channel wings action by a hard-coded 3-vector of bounds and died on the
   broadcast. Now each policy is normalized by its own
   `ActionBounds.to_list()[:act_dim]`, a zero bound is rejected rather than
   silently dividing, and the `wing` column prints "-" for three-channel
   policies. Read the per-channel cells across a mixed set, not the totals —
   a four-channel policy sums one more term. It immediately earns its keep:
   `general_wings_rl` sits at **100% wing saturation with 0.000 per-step
   change**, which is the "total crutch" of the wings1 run showing up as a
   number rather than a description.
