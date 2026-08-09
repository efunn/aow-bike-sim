# Project status — 2026-08-09

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
hardware to run it on yet. Parts are ordered. The two open engineering
questions are **the contact model**, which is currently the least-known
parameter in the sim and the one the policy is not randomized over, and **the
self-righting mechanism**, which has a recommended design (a mirrored wing pair
on a fourth servo) and no committed decision. The immediate risk is not
technical difficulty; it is that the pre-hardware test suite is red in 31
places, including the tests that were meant to gate first power-on.

---

## The four workstreams

| workstream | state | what "done" looks like | blocker |
|---|---|---|---|
| **Simulation & model** | Working. Parametric MJCF from `config/bike_params.yaml`, procedural omni-wheel contact meshes, 19 parameters still marked `GUESS` | Every `GUESS` replaced by a measurement or a deliberate randomization range | Physical parts to measure |
| **Control — RL** | Working, and the primary path. `general_rl_smooth_stiff` (5M steps, trained under the current contact model) is the best export: survive 1.00, track 0.805, reverse works | One champion policy, symmetric left/right, exercised over the randomization ranges the hardware will actually see | Crab is one-sided; turn asymmetry 0.275 |
| **Control — analytic (LQR)** | Degraded **on purpose**, and further than intended. Reference baseline only; nothing drives with it | Re-tuned once the contact model is pinned | `contact_solref` damping |
| **Hardware / untethered** | Software complete and tested in sim; nothing physical assembled | Bike balances untethered on a mat | Parts, chassis, servo homing decision |

---

## Critical path to a bike that stands itself up

**Plan as of 2026-08-09: mechanical design starts Monday 2026-08-10, and it
builds the full bike — no individual test rigs.** That decision reorders what
follows, because the contact measurements were written as wheel-only bench
tests. They are still doable off the assembled bike; see the next section for
what changes.

Two tracks. The sim track does not wait on the build.

**Sim track — can start now, blocked on nothing:**

1. **Enable the contact randomization that already exists.** `DomainRandomizer`
   gained `solref_frac` and `dampratio_range` in `0efac06`; they sit commented
   out in `config/rl_general.yaml` so prior runs still reproduce. Uncommenting
   them is the whole task, and **it does not depend on any measurement** —
   which is the point, because it decouples the largest sim-to-real risk from
   the build timeline. The survival cliff is close: holding `timeconst`, eval
   survival is 1.00 from dampratio 1.0 down to 0.5, then 0.9 at 0.3 and 0.7 at
   0.2, and the first drop test puts the true value near 0.3.
2. **Fix the test suite** (see Health below) so the build has a trustworthy
   baseline to debug against.
3. **Retrain** with randomization on, fixing the crab asymmetry in the same
   run.

**Build track — starts Monday:**

4. **Mechanical design and assembly of the full bike.** Carry the two
   righting-mechanism dimensions into it or deliberately defer them: the
   `bike_width` / `bike_height` envelope (120 × 165 mm) is what the roof and
   the stowed wings are both derived from, so it wants deciding at design time
   rather than retrofitting.
5. **Print a test pinion.** `min_pinion_radius` is a `GUESS`, the 4.83:1 gear
   fit ceiling hangs entirely off it, and the wing torque margin is already
   0.88 of stall. This is now a mechanical-design input, not a follow-up.
6. **Weigh everything as built** — chassis (`GUESS` 0.45 kg), pack (0.115),
   electronics stack (0.076). Re-run `export_deploy`.
7. **Measure contact off the assembled bike** (§ below), then re-centre
   `sim.contact_solref` and re-check that the policies still survive with
   `analysis/chatter.py`.
8. **Bench verification 1–5** from `docs/plans/untethered-setup.md#verification`
   — loop timing, AHRS, odometry, failsafes. These gate first power-on.
9. **First untethered balance** on training wheels, then without.
10. **Self-righting**, a separate decision, deliberately last.

Self-righting can keep progressing on paper in parallel — the analysis tools
re-sweep the geometry on demand — but it should not consume build time until
the bike balances.

---

## Health: the test suite is red, and wider than intended

`pytest` at HEAD: **179 passed, 21 failed, 10 errors, 1 xfailed, 2 skipped.**

The root cause is a single parameter. `sim.contact_solref` damping went
1.0 → 0.5 in `0efac06`, chosen from teleop feel and the slowmo contact study.
That change is defensible and is documented, but its blast radius was scoped
to one test. Isolated, holding everything else fixed:

| contact damping | circle R=0.8 tracking |
|---|---|
| 1.0 | tracks, mean radius error **0.011 m** |
| 0.5 (current) | fails, mean radius error **0.150 m** |

What that breaks, by suite:

- `test_drive.py` (14) — circles, flips, flicks, stops, heading commands. The
  analytic controller can no longer drive the shapes it could. **Expected**, if
  the LQR is accepted as a degraded reference.
- `test_hw_odometry.py` (2 failed + 10 errors) — the bike **falls over** during
  the episode, so the estimator is never measured. **Not expected, and this is
  the one that matters**: these are the tests that pin the velocity estimator
  before it flies on a Pi, and they are now measuring nothing.
- `test_teleop.py` (2) — the general policy falls. Cause is different and
  simpler: `control.general_move` still points at `general_rl` (2026-08-06),
  trained under the old contact model. The newer
  `general_rl_smooth_stiff` was trained under the current one and is better on
  every metric. **One-line fix.**
- `test_hw_replay.py` (1) — deploy bundle digest is stale. **The mechanism
  working as designed.** Fixed by `python -m aow_sim.export_deploy`.

Three of these four are cheap to resolve. Before hardware arrives the suite
should be green or explicitly marked, because the first real sim-to-real bug
will be diagnosed against it.

### Policy exports, current standings

| export | survive | track | vel_err | head_err | reverse | notes |
|---|---|---|---|---|---|---|
| `general_rl_og` | 0.92 | 0.702 | 0.225 | 9.9° | yes | 10M final |
| `general_rl` ← **config default** | 1.00 | 0.763 | 0.291 | 2.5° | **no** | 6M snapshot, pre-contact-change |
| `general_rl_1k` | 1.00 | 0.816 | 0.163 | 3.1° | — | |
| `general_rl_smooth_og` | 1.00 | 0.816 | 0.167 | 3.6° | 1.006 | crab 0.21 / 0.30 |
| `general_rl_smooth_diff_og` | 1.00 | 0.800 | 0.166 | 4.3° | 0.983 | crab 0.21 / 0.28 |
| `general_rl_smooth_stiff` | 1.00 | 0.805 | 0.191 | 3.0° | 1.122 | **current physics**; crab 0.39 / **0.02** |

`smooth_stiff` is the only export trained against the contact model the sim
now has. Its one regression is crab: left works, right barely moves.

---

## Decided vs. placeholder

**Decided and unlikely to move:** Pi Zero 2 W running the same Python source as
the sim, no ROS, no microcontroller. One 3S bus, bought balance charger,
perfboard splitter instead of the U2D2 hub. WiFi/UDP teleop with the laptop as
ground station. RL as the controller that actually drives; LQR as the
comparison baseline. Moves authored offline, replayed with numpy only.

**Recommended, not committed:** the self-righting mechanism. A mirrored wing
pair on one XC330 through a reduction beats the single arm on every number —
it stops itself at upright, hands off in 2.09 s vs 3.53 s, and never needs to
know which side it fell on. But the torque margin is 0.88 of stall, the gear
fit ceiling is 4.83:1, and `min_pinion_radius` — the parameter the whole
ceiling hangs off — is a `GUESS`. **Print a test pinion before committing to
this design.**

**Placeholder:** 19 parameters marked `GUESS` in `config/bike_params.yaml`,
each with a note on how to identify it. The load-bearing ones are contact
friction, chassis/pack/electronics mass, front tire lateral stiffness, and
`min_pinion_radius`.

---

## Physical tests, adapted to "full bike, no rigs"

From `docs/measurements/contact-protocol.md` and
`docs/plans/untethered-setup.md`, re-ordered for measurement off an assembled
bike. Those protocols were written wheel-only; **the protocol docs have not
been rewritten and still describe the rig versions.** The adaptations below are
the current intent.

1. **Static load-deflection.** Park the bike, add known mass to the chassis,
   and measure **rear axle height above the floor**, loaded vs. unloaded, same
   spot and same roll phase. Referencing the axle rather than the chassis keeps
   frame flex out of the reading — that was the reason the original protocol
   said "load through the axle", and it is preserved. The new confound is the
   **front wheel taking a share of the load**: either lift the front onto a
   support so all added mass goes through the rear, or record the split. Still
   the single highest-value measurement. A hand-guided first pass already
   killed `timeconst 0.020`; a controlled reading separates 0.005 from 0.0035.
2. **Drop test, rebound height.** `e = sqrt(h1/h0)` off a 240 fps clip. Works
   on the whole bike and is arguably more representative — but **do it on the
   bare rolling chassis, before the pack and electronics go on.** The observed
   2–3 bounces already falsify the shipped `dampratio` of 1.0.
3. **Print a test pinion** and find the smallest that survives. Now a
   mechanical-design input — see the critical path.
4. **Spin-down tests**, bike on a stand, wheel free — driven hub and flicked
   roller, for joint damping and frictionloss. A stand is not a rig.
5. **Incline slide test** for sliding friction (`GUESS 0.9`). Whole bike on a
   tilting board works.
6. **Weigh everything** at assembly.
7. **Known-circle drive** on the floor, integrated position vs. commanded, for
   front tire lateral stiffness. Needs a driving bike, so it lands after first
   balance.
8. **Bench: loop timing** at 1/2/3 Mbps with `latency_timer` 16 vs 1. Gate:
   p99 tick jitter < 1 ms at 100 Hz.
9. **Bench: AHRS** at 230400 baud (115200 will not carry 200 Hz Combo frames),
   quaternion against known orientations, age-of-data at the tick.
10. **Failsafes, deliberately triggered** — WiFi kill, pack below LVC, bike
    laid on its side.

Roll-phase (§P2) and surface dependence (§P3) stay deferred; they check the
mesh, not the contact parameters. The wheel-only `--variant testbed` model and
the empirical calibration in `omni-wheel-protocol.md` §7 have **no physical
counterpart under this plan** — treat them as sim-side tools, not as pending
work.

---

## Risks, ranked

1. **The pre-hardware test suite is not trustworthy right now.** 31 red,
   including the hardware-verification tests. Sim-to-real debugging without a
   green baseline is guesswork. Cheapest fix in the project.
2. **No trained policy has ever seen contact-stiffness variation.** The
   randomization axis exists but is disabled, so every export sits on one
   point of a curve that falls off a cliff just below the current nominal.
   This is the most likely single cause of a policy that works in sim and not
   on the floor.
3. **Authority derating.** Real servos will not deliver modelled torque at
   modelled bandwidth. Called out in `untethered-setup.md#the-real-sim-to-real-risk`
   and not yet quantified.
4. **Left/right asymmetry in the policy.** `general_rl` recovers 30–50% less
   lean to the left than to the right, and crab is one-sided in the current
   champion. Both are artifacts — the plant is mirror-symmetric. Fixing this
   widens the recoverable set and reduces how often righting is needed at all,
   which makes it the highest-value RL work outstanding.
5. **Steer homing at power-up is undesigned.** The XC330 loses its multi-turn
   count across a power cycle, and the fix depends on an unmade mechanical
   choice (hard stop vs. magnetic index). Blocks first power-on, not ordering.
6. **Front-wheel liftoff is undetected.** The lateral estimator assumes the
   front wheel is down. Fine until anything aggressive is attempted.
7. **Nothing in the code prevents a policy export from being overwritten.**
   `_finish()` in `train_general_rl.py` writes `moves/<name>.{npz,yaml}`
   unconditionally, so re-using an `--export-name` silently destroys hours of
   training. The trainers share this CLI shape. A refusal-or-suffix on collision
   is a few lines and removes the hazard entirely.

---

## Explicitly not being worked on

The PD cascade (legacy, kept compiling as a manual-debug fallback). The
trajopt moves `flick`/`flick_fwd`/`flip`, which no longer survive the modelled
payload and will be re-authored once the as-built mass is known rather than
twice. Roll-phase and surface contact studies. A live gamepad front-end — the
virtual pad exists and the keyboard drives it, a real controller just plugs in.
The ball-shot move, which works and is not on the critical path.
