# The web demo against teleop: what differs, and what might come back

> **Status (2026-10-23):** a review list. ONE item is applied to the Python
> side: the failed get-up retry (row 1), now in `control/swing_sit.py` and
> shared by teleop and the page. The rest is open. Every item below is either a change the browser demo (`src/aow_sim/web/`)
> made that teleop (`run_drive --teleop`) does not have, or something learned
> while building it that is true of the shared plant. Numbers are from the
> page's own simulation, which replays the Python env to ~1e-15 open loop and
> ~1 cm over 14 s closed loop (`web/check.mjs`), unless a row says otherwise.
> Each is marked **port?** (worth considering for mjpython), **page-only**
> (a demo choice), **finding** (a fact about the shared sim), or **TODO** (the
user has asked for it on the Python side).

---

## 1. Behaviour: teleop vs the page

| # | what | teleop | page | mark |
|---|---|---|---|---|
| 1 | a failed get-up | was: lift times out -> `limp` -> `lying`, waits for PageUp | retries 1 s after a lift that failed; a chosen lie-down stays down | **done** in `swing_sit` (both) |
| 2 | the shove | `.`: 8 N for 0.35 s along **world y**, no marker | Space: 3 N, Shift+Space: 1.5 N, 0.35 s, across **the bike** (`-sin psi, cos psi`), alternating sides, a red arrow while it acts | **TODO** |
| 3 | fresh turn press | jumps the heading 10 deg, then slews | no jump; slews only. Same lead clamp, same snap disarm/re-arm | page-only |
| 4 | blade keys | 9 / 4 step left / centre / right at the FULL stroke (116.9 deg crank), where the blade meets the floor | Z / C step to 91 deg of crank (70 % of stroke, user's number from the mechanism sim) | **port?** (as a third stop) |
| 5 | the ball | `--hockey`: the study ball (67 mm, 60 g), stick panels on the chassis, pelt 4 m/s | 160 mm, 10 g, bouncy, air drag, no stick panels, pelt 2.5 m/s; spawns in front of the bike; drawn as a beach ball | **TODO**: fix the ball physics (section 2); the page's ball is a stand-in, not the answer |
| 6 | command after a get-up | `engage()` holds what the bike is doing: `v_cmd = qvel`, heading as is | velocity command zeroed, heading where the bike points, key state cleared | page-only |
| 7 | AHRS model | `tm151_filter` by default | off: attitude and heading are MuJoCo truth (felt in teleop first: no visible difference) | page-only |
| 8 | controller rate | `DriveController` at 200 Hz, policy held 4 ticks, steer integrator and `swing_sit` stepped at 200 Hz | the env's 50 Hz: policy, steer integrator and `swing_sit` per 50 Hz step | page-only (the env's semantics, which is what trained) |
| 9 | righting module | jointed, crank on `CurrentBasedPositionServo` | the same (the eval env welds it: `righting_rigid`) | same as teleop |
| 10 | trail | the chassis origin | the rear hub (`aow_hub`), 2 s solid + 0.5 s fade | page-only |
| 11 | steer readout | -- | mod 180, +-90 deg (multi-turn joint, symmetric wheel) | page-only |

### 1. The failed get-up (the "press X twice" case) -- applied

Seen by hand in the page: after repeated knockdowns the bike can come to rest
rolled onto its cage, and the lift jackknifes the blade against the shell,
lifting it the wrong way; the lift times out, the sequence goes limp and lies
there. Pressing X (get up) again usually brought it up, sometimes on the
second press. The user believes teleop has the same edge case. The code said
"the operator (or the next auto-up) tries again" at the timeout, but the
auto-up only fires from `riding`, so nothing ever did.

Now in `SwingSit` itself (`retry_s`, default 1.0 s; None is the old
behaviour): a lift that times out goes limp, lies for `retry_s`, and lifts
again on the side it lies on THEN. A lie-down the operator chose is not
retried. `sit.js` mirrors it line for line; the page's own copy of the logic
is gone. Pinned by `test_swing_sit.py::test_a_failed_lift_tries_again_and_a_chosen_lie_down_does_not`;
teleop's `test_swing_lie_down_get_up_and_auto_up` still passes.

Measured in the page sim, 20 knockdowns in a row (3 N, some doubled before it
had recovered), no resets: **all 20 riding again in 3.8-7.5 s, 2 of them on
a retried lift** (3 of 20 in an earlier run on a different ball model --
the knockdowns are chaotic). Without the retry those would have stayed down.
Not yet flown in teleop by hand.

### 2. The shove

Swept on this plant, both sides, 4 detent phases each:

| force (0.35 s) | impulse | result |
|---|---|---|
| 0.5-1.5 N | 0.18-0.53 N s | the policy catches it, peak roll 3-18 deg |
| 2-8 N | 0.7-2.8 N s | over, every time; the round cage then rolls it onto its back (168-180 deg) |

So teleop's 8 N is ~4x the threshold, which matches its comment ("comfortably
past the recoverable set") but leaves nothing between "nothing" and
"barrel roll". A recoverable nudge was the more interesting key in the page.

Teleop writes `xfrc_applied[chassis, 1]`, which is WORLD y: across the bike
only while it heads along x, and straight along its length when it heads
along y. The page pushes across the bike.

**TODO for teleop** (user, 2026-10-23):

  * push along the BIKE's lateral axis, not world y;
  * reduce the force to the page's: 3 N for the knockdown; consider the
    page's 1.5 N nudge as a second key;
  * draw the marker: an arrow at the chassis's centre of mass, pointing along
    the force, shown for exactly the pulse and scaled with the force (the
    page's is red, driving in from 0.3 m to 0.1 m over the pulse, then
    fading).

---

## 2. Findings about the shared sim

### The ball has no rolling resistance (condim 4)

`_add_hockey` gives the ball `friction=[0.4, torsional, 0.0001]` at
`sim.condim` (4). **MuJoCo applies rolling friction only at condim 6**, so the
0.0001 never acts: the ball rolled 53 m from 4 m/s and was still going at
20 s. At condim 6, rolling friction 0.005 stops it in 4.2 m (1.1 m from
2 m/s).

Who builds that ball: teleop `--hockey`, and the ball-shot move's env
(`control/ball_env.py`, `hockey=True`). **The general policies never had it**
(`ball_prob: 0.0` in every `rl_general*` config and in the pointer's yaml).

**TODO: fix the ball physics (approach TBD).** condim 6 and a real rolling
friction is the minimum. The page's ball (160 mm, 10 g, stiff bouncy contact,
air drag) is a demo stand-in chosen for play, and the user is not satisfied
with it yet; the hockey ball's mass and radius in `config/studies/ball.yaml`
are still `GUESS`. The ball lives under `hockey`, which is in `params.STUDY_PATHS`, so
fixing it moves no digest (checked: `plant_digest` unchanged by `hockey`
edits; a `righting.module` edit does move it).

### A small ball can prop the bike over through a blade

With a blade out at 91 deg and the 67 mm ball beside it, driving past FELL
(right blade, 0.3 and 0.6 m/s); with no ball the same drive stays up
(1-2 deg). The blade rides up onto the ball and props the bike like a
kickstand -- whatever the ball's mass, since the floor holds it up from
below. Driving into and past it, 6 drives x 4 phases:

| ball | falls | ball pushed (median) |
|---|---|---|
| 67 mm, 20 g, grippy | 2 / 6 (one phase) | -- |
| 120 mm, 10 g, bouncy | 8 / 24 | 0.48 m |
| 160 mm, 10 g | 0 / 24 | 0.32 m |

Relevant if a ball (or any obstacle of that size) ever goes back into
training with the wings out.

The physical version (user): a small enough object can get caught INSIDE the
bike -- between a blade and the shell, or the rollers -- and a metal one (a
screw, a pebble with ore in it, a washer) near the electronics can short
them. Not modelled anywhere: the sim's bike has no interior collision
geometry for it to lodge in.

### A bouncy ball jitters against the wheel unless the contact is stiff

Pushed by the front wheel at 0.5 m/s:

| ball contact (stiffness, damping; direct solref) | rebound | hop span, airborne |
|---|---|---|
| 2.5e3, 10 | 0.62 | 21.5 mm, 14 % |
| 1e4, 20 | 0.62 | 8.0 mm, 3 % |
| **3e4, 70** (the page) | **0.48** | **1.7 mm, 0 %** |
| 3e4, 110 | 0.36 | 1.2 mm, 0 % |

At rest every setting is still (5 um). Rebound is labelled by MEASUREMENT
(dropped 0.3 m, sqrt of the height ratio), per the `contact_solref` warning
in CLAUDE.md.

How the page keeps this the ball's alone: MuJoCo mixes each contact from its
two geoms (friction the larger; solref/solimp weighted by `solmix`), so the
ball's `solmix` is set to 1000 and its own solref wins every ball contact.
No explicit `<pair>` elements needed. The air drag is a page-side force on
the ball (MuJoCo's fluid model is global and would drag the bike too).

### The lie-down's limp flips the far blade up -- open question

What the model does (page sim, lie-down to the left, every 0.1 s):

| phase | crank | far blade | servo torque | crank's friction limit |
|---|---|---|---|---|
| `lower`, end | 0 deg | 0 deg | -300 to -400 mN m (holding the bike's weight on the lower blade) | ~110 mN m |
| `limp`, Goal Current -> 0 over 1 s | 2-3 deg | 1 deg | falling with the current | 57-410 mN m |
| `lying`, first 0.2 s | 3 -> 71 deg at up to 7.7 rad/s | 1 -> 37 deg | 0 | 57-84 mN m |

Once the current is gone, ~300 mN m of load from the bike's weight meets
~60-80 mN m of holding and back-drives the crank 70 deg. The holding is the
XC330's gearbox friction (no-load static, ~10 mN m) plus the V2 linkage's
measured pin friction (`righting.module.linkage_friction_nm` 51 mN m +
`_q2` theta^2). The user's read is that the mechanism does not do this.

**The likely gap: the linkage friction was measured UNLOADED.** Its own
comment in `bike_params.yaml`: "stubs only, no blades, unloaded ... Not in
it: ... the bike's weight on the pins." Lying on a blade puts the bike's
weight through those pins, and pin friction grows with the load on them
(mu x N x pin radius), so the real holding torque while lying is plausibly
several times the 51 mN m the model uses. The gearbox side has a
load-proportional law (`0.51 x load + 5.1 mN m` static), but its LOAD is the
torque the motor passes, ~0 with the torque off, so that does not cover it
either. Two smaller unknowns: whether the XC330 brakes (shorted windings) at
Torque Enable 0, and whether the stowed blade bears weight while lying
(CAD).

**Correction to an earlier claim in this session:** changing this is NOT
gated by the policy. The pointer (`general_puffer_fixes_72M`) trained with
the module WELDED (`righting_rigid: true`), so crank and linkage friction
never reached it. What a change does cost: `righting.module` is in
`plant_digest`, so every export's digest stops matching and the deploy
bundle needs re-exporting (`python -m aow_sim.export_deploy`), and the
`righting` tests and `analysis/moves/swing_sit.py` numbers move.

To decide it: lie the real bike down with the crank centred, torque off,
and watch the far blade. **Testable as soon as the printed blades are
fitted (user, 2026-10-23: planned for 10-24)**, and the user notes the sim
may turn out right. If the blade holds, the fix is the ZERO-TORQUE linkage
friction -- a load term, or a higher holding value, re-measured loaded as
the bike_params comment already asks. If it flips too, the model is right
and nothing changes.

### An XML round trip of the compiled model is not faithful -- low severity

What was seen: `spec.to_xml()` -> `from_xml_string` -> apply the runtime
switches (`DrivetrainSim`, gearbox friction) -> `mj_saveLastXML` -> reload,
compared array by array with the env's model:

  * every float off by up to ~1e-7 (XML prints floats short);
  * the meshes re-processed on reload (vertex order, `mesh_quat`, the
    polygon tables) -- the same shapes, different internals, so contact
    points can differ;
  * **`eq_active0` and `tendon_limited` not written back**: the rollers'
    rigid couplings came back ACTIVE and the slop tendons unlimited, i.e.
    the roller slop silently disappeared. No error anywhere.

`mj_saveModel` (.mjb) is byte-exact, which is what the page ships.

**Why it is low severity here: nothing in the pipeline goes through XML.**
Training, eval, teleop and the deploy path compile from `MjSpec` in memory;
the GPU path hands the compiled model straight to MJWarp
(`gpu/env.py`, `mjw.put_model(env.model)`). The only XML writer is
`build_model`'s CLI (`-o`), and what it writes is the spec BEFORE any
runtime switch -- the ideal drivetrain, as `edit_spec` documents (the
detailed plant exists only once `DrivetrainSim.attach` runs). Whether that
XML alone also carries the ~1e-7 / mesh differences was not measured;
the mechanism is the same, so expect it.

When it would bite: anyone taking the model to another runtime or tool
through XML (a viewer, another simulator, a web page) gets the IDEAL plant
at best, and a quietly different contact if the meshes matter. Ship an
.mjb, or rebuild from `MjSpec` and re-apply the runtime switches.

---

## 3. Page-only, listed for completeness

Viewer and input choices with no Python counterpart: streaming physics per
display frame (teleop already steps per physics step), shadows from every
part including the translucent shell, the green heading arc with round caps
and theme colours, the red shove arrow (scaled with force, shown for exactly
the pulse), the shell pulsing while lying down or getting up, a blob shadow
for the ball, the beach-ball look (six blue/orange gores, one cap of each colour, so
the spin shows), the steer readout mod 180 (the wheel is symmetric: 180 on
the servo reads 0), the touch pad, telemetry behind T.

---

## Reproducing

```sh
python -m aow_sim.web.export --trace traces/web_trace.json
```

```sh
node src/aow_sim/web/check.mjs traces/web_trace.json
```

The sweeps above were one-off node scripts against `site/js/sim.js` (shove
force, ball size / contact, knockdown series, lie-down log); the constants
they chose, and their tables, are in `src/aow_sim/web/export.py`.
