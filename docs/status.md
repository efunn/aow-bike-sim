# Project status — 2026-08-14

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
to a defensible state** (7 failed / 214 passed, and all 7 are the trajopt moves
already scheduled for re-authoring) after `contact_solref` was reverted to
`[0.005, 1.0]`. Two things changed shape since the last snapshot: **pitch is
now observed and priced**, which closed out a long-running investigation into
why crab does not work, and **the self-righting mechanism is now a complete,
verified design** rather than a recommendation — a mirrored wing pair whose
whole envelope derives from two measurable numbers. The remaining open
engineering question is still **the contact model**, which is the least-known
parameter in the sim and the one no policy has been randomized over.

---

## The four workstreams

| workstream | state | what "done" looks like | blocker |
|---|---|---|---|
| **Simulation & model** | Working. Parametric MJCF from `config/bike_params.yaml`, procedural omni-wheel contact meshes, **17** parameters still marked `GUESS` | Every `GUESS` replaced by a measurement or a deliberate randomization range | Physical parts to measure |
| **Control — RL** | Working, and the primary path. Pitch is now observable and priced (`obs_pitch`, `w_pitch`) | One champion policy, symmetric left/right, exercised over the randomization ranges the hardware will actually see | Crab still one-sided; turn asymmetry stuck ~0.27–0.32 across every run |
| **Control — analytic (LQR)** | Reference baseline only; nothing drives with it. Fit is currently **healthy** (worst R² 0.9893) | Re-tuned once the contact model is pinned | Nothing right now — it will degrade again when contact damping moves |
| **Hardware / untethered** | Software complete and tested in sim; nothing physical assembled | Bike balances untethered on a mat | Parts, chassis, servo homing decision |

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

## Tooling added

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

## Health: the suite is defensible again

`pytest` at HEAD: **7 failed, 214 passed, 2 skipped.** Was 21 failed + 10
errors + 179 passed.

All 7 are in `test_drive.py` and all are the trajopt moves — `test_flip_completes`
(×2) and five `flick` tests. These are the **already-accepted** set: those moves
no longer survive the modelled payload and are deliberately queued for
re-authoring once the as-built mass is known, rather than being re-optimised
twice. Nothing else is red.

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

---

## Critical path

Two tracks. The sim track does not wait on the build.

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

**Build track:**

5. **Mechanical design and assembly of the full bike.** Carry the
   `bike_width` / `bike_height` envelope (120 × 165 mm) in at design time — the
   roof and the stowed wings both derive from it, so retrofitting means
   redoing both.
6. **Print a test pinion.** `min_pinion_radius` is a `GUESS` and the 5:1 gear
   ceiling hangs entirely off it. A mechanical-design input, not a follow-up.
7. **Weigh everything as built** — chassis (`GUESS` 0.45 kg), pack (0.115),
   electronics stack (0.076). Re-run `export_deploy`.
8. **Measure contact off the assembled bike** (§ below), then re-centre
   `sim.contact_solref` and re-check the policies with `analysis/chatter.py`.
9. **Bench verification 1–5** from `docs/plans/untethered-setup.md#verification`
   — loop timing, AHRS, odometry, failsafes. These gate first power-on.
10. **First untethered balance** on training wheels, then without.
11. **Self-righting**, deliberately last. The design is finished and verified in
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
5. **Incline slide test** for sliding friction (`GUESS` 0.9).
6. **Weigh everything** at assembly.
7. **Known-circle drive** for front tire lateral stiffness. Needs a driving
   bike, so it lands after first balance.
8. **Bench: loop timing** at 1/2/3 Mbps, `latency_timer` 16 vs 1. Gate: p99
   tick jitter < 1 ms at 100 Hz.
9. **Bench: AHRS** at 460800 baud, quaternion against known orientations,
   age-of-data at the tick. Expect no data for ~3 s after power-on — ~30 s if
   the unit is still on its factory static-boot default.
10. **Failsafes, deliberately triggered** — WiFi kill, pack below LVC, bike
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
5. **Steer homing at power-up is undesigned.** The XC330 loses its multi-turn
   count across a power cycle, and the fix depends on an unmade mechanical
   choice (hard stop vs magnetic index). Blocks first power-on, not ordering.
6. **Front-wheel liftoff is undetected by the estimator.** The lateral estimator
   assumes the front wheel is down. `analysis/liftoff.py` now measures how
   often that is false — 79 mm of clearance on arm 1 — so this is quantified
   rather than suspected.
7. **The contact protocol docs are stale.** They still describe wheel-only rigs
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
