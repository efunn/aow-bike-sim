# Project status — 2026-10-21

**How to read this.** The navigation layer: what is true now, what to do next,
and where the detail lives. Rewritten, not appended to. **For the reasoning,
follow the link — it is not repeated here.**

Rewritten 2026-10-02 (1798 lines -> ~300; the old file is `git show
d0dffeb:docs/status.md`) and again 2026-10-16, when solved problems still
listed as open were moved out: the GPU stack, the retrains on today's plant,
and the 10-14 env fixes had all landed without this file catching up.

---

## Where the project is, in five lines

1. **The simulator is trusted for control design**: parametric MuJoCo, an
   omni-wheel contact, the bike's own sensors in the loop and in training,
   measured XC330 laws, headset friction, the detailed drivetrain and the
   righting module. The contact is the least-measured part; bench pre-tests
   started 2026-10-01.
2. **RL drives, and it trains on a rented GPU.** The pointer is a
   GPU-trained MinGRU policy (`general_puffer_fixes_72M`, 10-16); MuJoCo
   Warp + PufferLib train 72M steps in ~12 min on an RTX 5090 (~$0.23). The
   box's CPU stack stays as the reference. The analytic LQR is DEPRECATED
   (10-09).
3. **The onboard path runs on the Pi** (bench 09-15 to 09-18): 100 Hz loop,
   TM151 pushing from flash, mirror, telemetry, recorder, fall/cut/re-arm,
   low-voltage cut.
4. **The bike is assembled** (10-21) from the v2 CAD, minus the wing blades
   and the rollcage. First bench session on it fixed the steer sign, the
   accelerometer sign and the mirror's front-wheel spin
   (`odometry-sharp-steer.md`).
5. **The goal is a functioning bike with sample videos, in weeks.** Ranked
   against that; the showcase moves are in `wishlist.md`.

---

## What to do next

| # | do | why now | doc |
|---|---|---|---|
| 1 | **Test the assembled bike**: the pointer policy on the floor, held first | The point of the build. If it does not drive, the contact bench is the fallback (the drop-rig fit, the P0b pull test), not before | `first-physical-test.md`, `pi-bench-bringup.md` |
| 2 | **Blades and rollcage**: design the blade attachment and the lighter caps, print, fit; then a `righting_current` that lifts | The righting can't be flown without blades. `righting_current` 300 is a placeholder: 625 mA lifted the bike by hand in teleop, the sim's sweep needs ~600-650 | `righting-design.md`, `aow-bike-whole-v2-touchups.md` |
| 3 | **Retrain on today's plant: 4 seeds under MPS** | One seed ran 10-23 (`general_puffer_rake20_{52,72,80}M`, plant 6446d8fa): as good as the pointer on the grid, but NOT smoother -- straight at 1.2 m/s it wobbles 5.9 deg (80M) against the pointer's 2.5 on the same plant, shrinking with training (52M 17.1). One seed cannot say plant or seed. MPS runs 4 seeds on one 5090 at 2.3x the throughput (~46 min for 4 x 80M), so the next run is four; `scripts/gpu.sh up HOST PORT SEEDS=4` (written 10-23, its multi-seed path not yet run on a GPU) | `gpu-training.md` |
| 4 | **The style bit: find the pivot** (not a priority) | Run 1 (add) refused in-place turns; run 2 (mult, 10-18) turns but steers, and stops 20-33 deg short on the big ones. 16k worlds x horizon 16 trains in 18 min but its bit-off hold creeps 0.02-0.05 m/s | `wishlist.md`, `gpu-training.md` |

**The sim is the v2 bike since 10-19**: positions, orientations, rake 20
and wheelbase 200.5 from the v2 CAD; the TM151 tilted 45 deg (the box and
the accelerometer's place in the part; the IMU site stays on the chassis
axes, as the mount calibration delivers); the rollcages + caps + sheet as
one shell-inertia capsule (`righting.rollcage`, ~212 g, the heaviest single
GUESS); the righting XC330 placed on its own. Every mass is a shape you
can see: each part its CAD tight box in its own frame (45 deg drive, 20
deg steering), the rails their CAD outlines extruded, the pulleys discs, the belts their
runs, the righting's fixed block its five parts (`bike.frame_parts`,
`righting.module.fixed`); cosmetics to replace them are next. 1248 -> 1104 g, CoM (86.8, 0.0,
66.4) -> (87.3, 0.5 left, 60.4) mm from the rear axle; inertia about the CoM
UP despite the lighter bike (roll/pitch/yaw 2.97/6.25/4.34 -> 3.37/7.43/5.73
e-3 kg m2), the 212 g rollcage at the radius. Then (10-21) the cables, fasteners and belts
weighed by item: 1104 -> 1157 g, CoM (87.3, 0.5, 60.4) -> (86.1, 0.5, 60.0). The power board weighed 16 g
(was 56 GUESS). What moves is untouched: drive inputs, steer, wheels,
the righting four-bar (its wing hubs read 11.5 g in the v2 CAD against 14.9
listed). `bike_params_cad.yaml` is retired in practice (user): nothing in
the sim reads it. Weighing sheet: `traces/weigh_v2_printed.txt`.

Bring-up (10-17): all four servos share one bus at 3 Mbps as configured,
101/102 XC430 velocity, 103 XC330 extended position (steer), 104 XC330
current-based position (righting). The Pi's tree was resynced from this
checkout (no `--delete`, `bench/` left alone); the 100 Hz loop runs torque-off
with the GPU MinGRU pointer (p99 jitter 0.56 ms).

Righting module V2, first powered runs (10-17; stubs only, clamped free,
horn now screwed; `analysis/mechanism/righting_sweep.py`, captures in
`traces/righting_sweep/`). Kinetic friction at 15 deg/s, as (I+ - I-)/2:
~90 mA near stow, ~110-120 at +-15-75, ~140 at +-75-90, ~170-190 at
+-90-120 (bare XC330 ~22). Breakaway by goal creep: 110-130 near stow,
~150-190 at +-75-100, up to ~250 at +120. With Position I 0 the crank parks
~6 deg short of its goal (~111 for the V3 blade's commanded 116.9). Friction rose with temperature over
the session (76 -> 93 mA near stow, 26 -> 32 C). The rise toward the ends
tracks the linkage's joint rates per crank degree (crank-pin rate alone: rms
12 vs 32 mA for a constant), but those curves are all correlated, so which
joint it is remains unknown. `righting_sign` +1 measured (horn forward: right
wing down = +). The wings share crankpin and rocker rod, so their transmission
angles are equal (73 deg at stow, 30 at 129); each passes its folded toggle at
~+-62, and only the RIGHT wing's (crank -60) shows a bump, ~+25 mA: a joint on
that side, not the geometry. Friction is flat with speed (ramps at 15 to 240
deg/s, peak current <= 243 mA). The bike's own command, a goal step at 300 mA
(`--step`): ~610 deg/s, 90 % of a 120 deg step in 0.21 s, no overshoot,
parks ~6 deg short; braking current peaks ~380 mA, ABOVE the 300 cap. A
stepping test (`servo_breakaway.py`) threw the
crank at 630 deg/s and is not for built mechanisms.

First policy runs on the bench (10-17; new 5 V 2 A TRACO board, no
undervoltage flags; drive servos bare, steer and righting modules clamped,
steer column installed, no fork/wheel; `run_bike --bench --torque --no-link --auto-rearm`,
records in `traces/bike/261017-17*`). 100 Hz, no dropped frames; fall cut and
auto re-arm work. With the AHRS held level the policy SAWS THE STEER: 5.2 Hz,
growing 1 -> 8 deg rms over ~15 s, in its own command (the servo follows).
Not roll; the velocity estimate carries it too. In the sim, pinned upright
(`analysis/policy/steer_pinned.py`), the same policy is quiet (0.02 deg) and only
limit-cycles (0.43 deg, 4.9 Hz) once BOTH (a) the steer servo is given the
bench's response and (b) the policy sees the steer rate as the Pi forms it.
(a): open-loop sweep of the XC330 at P 900 with the steering column and
headset installed (no fork or wheel, hanging)
(`analysis/servo/steer_response.py`, `traces/bench/steer_response_261017-175708.json`):
gain/phase 0.81/-69 deg at 5 Hz against the sim's 0.66/-63. Repeated on the
bare righting XC330 (mode 4, `steer_response_261017-190803.json`): within
0.02 at +-5 deg, so the column adds nothing and the units agree. NOW IN THE
PLANT: `steer_kp` 2.3, `steer_armature` 5.5e-4, `crank_armature` 5.5e-4 (kv
held at the free-spin droop; a kv fit, 0.055, let the steer outrun its motor
at 14.5 rad/s). Constant-speed ramps confirm it above 0.45 rad/s (droop
29 ms, running friction 5.75 mN m from the lag's intercept). Below ~0.25 rad/s
the bare servo tracks within ~1 encoder count, with no droop or friction --
NOT modelled, deliberately: a one-count effect that the wheel's scrub and the
horn-to-wheel backlash will swamp on the bike. Next for small motions: the
assembled steering, loaded, with the backlash read off the wheel side. (b): `bike_bus` differences the steer
counts through RateFilter (~8 ms) while training observed the exact joint
velocity. Fixed in the training path (10-17): `steer_rate_encoder: counts`
(`sim_odometry.SimSteerRate`; GeneralEnv, the GPU odometry kernel, the move
yaml, teleop `--steer-rate`), ON in `rl_general_v3.yaml`. Every existing
policy is `joint`; teleop `--steer-rate counts` flies one as the Pi does. The
remaining 20x amplitude gap is unexplained (small-signal servo nonlinearity,
sensor noise and the wheel-less drives are untested).

Not next, deliberately: a trained (RL) ball shot (close-range scripted plays
exist in teleop), the privileged critic, changes to the odometry estimator
(`odometry-sharp-steer.md`), the AHRS fixture (parked 09-24), the contact
calibration (the fallback in #1).

---

## The workstreams

| workstream | state | blocker | doc |
|---|---|---|---|
| **Simulation & model** | Working; the v2 bike since 10-19, 5 `GUESS`es left in bike_params (the rest `estimate` / `model`). Plant digest **6446d8fa** (10-21). Before the v2 bike, 7893a119 (10-17): the measured front wheel (10-05), the detailed drivetrain as the default plant (10-02), the V2 righting module (10-09) with the V3 blade (10-14), its linkage's measured friction on the crank (10-17; righting needs 530 -> 600 counts at 12 V -- before the crank armature moved, not re-run). The steer XC330's measured small-signal response, kp 2.3 + rotor inertia 5.5e-4, also the crank's (10-17). XC330s from the bench (current law, load-proportional gearbox friction); the steer as its firmware runs it, 4 ms delay; headset friction since 09-30. Optional per-episode floor tilt for training, CPU and GPU, off by default (10-16) | Contact and masses, unmeasured | `mujoco-modeling-decisions.md`, `drivetrain-model.md` |
| **Control — RL** | Primary. Pointer `general_puffer_fixes_72M` (10-16): GPU, MinGRU (stateful), `tm151_filter` + odometry counts, detailed drivetrain, today's plant. On one `per_command.py --summary` grid (`--ahrs tm151_filter`): 0/20 falls, 7/9 turns resolved forward, 2/2 straight forward; chatter score 0.863, per-step action change 0.008 (v3 seeds 0.035-0.048); endurance minute **0/18 falls** (10-16). Two GPU seeds pick the same direction on all 20 grid commands. CPU arms on today's plant, finished on the box: `queue_general_v3` (5 seeds, MLP) and `queue_general_v3_mingru` (5 seeds, MinGRU actor); turns forward 4-8/9 (s0-s2 measured) and 5-7/9 (all five), 0 falls. `v3_s1` and `v3_mingru_s0` are on top of `moves/`. All of them trained before plant 7893a119 (10-17), so provisional. **The style bit** (10-17): a command entry, `obs_style`, active only at a zero speed command, that pays for the front wheel holding the world heading it had when the bit came on (`general_spec.wheel_err`; obs 17 -> 20). CPU and GPU env, export, teleop Home, `per_command --style`. Baseline: no policy pivots (wheel/yaw 0.92-1.05). **Run 2** (`general_puffer_mult_*`, 10-18, multiplier form, 16k worlds): turns with the bit on but steered, no pivot; bit-off hold creeps 0.02-0.05 m/s. **On the v2 plant** (`general_puffer_rake20_{52,72,80}M`, 10-23, 6446d8fa, no style bit): 0/20 falls, 7/9 forward, standstill 0.13-0.21 m/s like the pointer; steer wobble at 1.2 m/s straight 5.9-17 deg against the pointer's 2.5 flown on the same plant; not the pointer. **Run 1** (`general_puffer_style_{52,72,80}M`, GPU, plant 7893a119): bit off, as good as the pointer (0/20 falls, 7/9 forward, standstill 0.11-0.19 m/s); **bit on, it mostly refuses the in-place turn** (yaw 0-17 deg on 9 of 15 spin episodes, wheel held), one real pivot that overshot, one fall. The reward lets refusing outbid turning (`wishlist.md`) | Crab never trained (`v_lat_frac` 0; crab L/R ~0.45/0.63 on the pointer); not flown on the bike | `gpu-training.md`, `general-rl-improvements.md`, `policy-smoothness-losses.md` |
| **GPU training** | Phases 1-3 done (10-14 to 10-16): MuJoCo Warp env as one CUDA graph per policy step, bridged into PufferLib 5.0's CUDA trainer, patched for reward scale, advantage normalisation, the temporal smoothness term and a paced shared curriculum. Checkpoints convert to moves (`gpu/export_policy.py`). Cards: 5090 12 min / $0.23 per 72M steps, 4090 17 min, A40 38 min. Runs are not bit-repeatable (float atomics). **Style run (10-17):** 5090, 4096 worlds, 80M steps in 29 min of training, ~34 min / ~$0.68 of pod; 20-wide obs end to end (`install.sh` sets the CUDA side's width from the graph). The first GPU run of the Pi-style steer rate, no alarm. `pod_launch.sh` now writes `results/watch.log` itself. **Run 2 (10-18):** 16k worlds x horizon 16 (same batch), 80M steps in 18 min of training at 82-90k steps/s; `run_pod.sh` scales the curriculum lead with the world count (a flat 0.1 at 16k climbed 4x slower per sample). Bit-off hold creeps at 16k (see row 4). **V2 plant run (10-23):** 5090, 80M in 1724 s; EUR-NO-1's host had ~280 KB/s to pypi (terminated). **MPS (10-23):** 4 runs at once on one 5090 give 116k steps/s in total (one run ~51k; plain processes ~55k at 2-8 runs, time-sliced); 8 give 127k (`bench_seeds.sh`) | Phase 4 (MLP / loss parity) and 5 (five GPU seeds vs v3's five on one grid) not started; no sim-to-sim check of a GPU policy in C beyond the eval grid | `gpu-training.md` |
| **Sensor modelling** | Largely done. TM151 measured on the fixture: a complementary filter, tau 0.19 s at rest to ~1 s moving, ~0.2-0.3 deg RMS at one mount against the sim's 1.5. `tm151_filter` with the accelerometer weighting from the drop rig (10-07) is the training level since 10-09, provisional; unproven on sustained acceleration | The bike's own AHRS log; the wave cams (sustained acceleration) | `sensor-workstream.md`, `ahrs-fixture.md` |
| **Control — LQR** | **Deprecated 10-09.** `lqr` tests skipped unless named (`pytest -m lqr`: the 15 registered reds). Never drove the bike. The code stays: the bundle's gain schedule is built on it (the crawl fallback serves only 2-action feedforward moves, no general policy). Its settings are `config/studies/analytic_control.yaml` since 10-21. Last state (10-05): stands on truth and `tm151_filter`, falls turning at 0.8 m/s | none: not being tuned | `lqr-baseline.md` |
| **Hardware / onboard** | Pi 3 bench proven: tick jitter p99 < 1 ms with four servos energised. 2.4 GHz house wifi (decided: a router, no AP). The SBC that ships: open | Chassis, pack | `pi-bench-bringup.md`, `untethered-setup.md` |
| **CAD** | v2 is the bike as built (`cad.bike_v2`, 10-19): drive case sides as the side rails, the steering and righting modules sliding into them, the power holder, the Pi + U2D2 carrier, the rollcage rings and caps; `--check` clean in every pose. v1 (`cad.bike`) is frozen. Printed and assembled 10-21 except the blades and the rollcage | The blades' attachment to the wing hubs; lighter rollcage caps; the sheet (radius 66.2 GUESS, not modelled). Print-fit details: the touch-ups doc | `aow-bike-whole-v2-touchups.md`, `aow-bike-whole-v2-complete.md`, `aow-bike-whole-v2.md`, `cad-onshape-workflow.md` |
| **Self-righting** | V2 linkage with the **V3 blade** is the default (10-14, user): level at 116.98 deg (commanded 116.9), lying at 85 deg, 566 counts in the sim (10-14); `righting_current_sweep.py` at the real +-117 stroke (it read V1's 136.6 until 10-17): 530 counts frictionless, **600 with the linkage friction** (10-17), toe stop at ~130. V2's blade is `--blade config/linkage/righting_blade.yaml`; a blade file may carry its own `mass` and `crank_travel_deg`. **Lie down / get up / auto-up** (10-16): `control/swing_sit.py`, teleop PageDown / PageUp, and by itself after a fall (`--no-auto-up`); a fall after the hand-off folds the blade and re-arms the auto-up. Sim only: down 5.8 s at <= 102 deg/s, up 3.3 s; 24/24 in teleop | The blades; a `righting_current` that lifts; fly lie-down / get-up on the bike | `righting-linkage-margin.md`, `righting-servo-model.md`, `wishlist.md` |
| **Web demo** | Built 10-23 (`src/aow_sim/web/`): the pointer policy in MuJoCo 3.10's WASM build, a static page for an iframe, keyboard and touch teleop. The plant is the eval env with the AHRS model off, the righting module jointed (crank servo model) and a ball. Teleop's extras: shove (3 N knockdown; the policy catches up to 1.5 N, the 8 N teleop key is ~4x past it), auto-up and lie down / get up (`swing_sit` ported), blades to 91 deg of crank, the heading arc and trail, a 160 mm bouncy ball the bike can push without falling (0 / 24; a 67 mm one propped it over). `check.mjs` replays a Python trace through the JS (policy ~1e-15 open loop; closed loop ~4 cm apart after 14 s, no fall; per-frame streaming bit-identical). ~4 MB on the wire; 60 fps in the desktop app's browser. Not yet deployed | Hosting (Cloudflare Pages); phones not tried on a real device | `src/aow_sim/web/README.md` |
| **Contact bench** | A fallback, parked: used only if the bike does not drive on the current contact. Drop rig built and run (10-03), its MuJoCo twin `analysis/contact/drop_rig_sim.py --fit`; `force_drop.py --ahrs` logs the TM151 with the drops. The wave cams are printed and roughly work in the rig (user, 10-21); `bench/wave_run.py` drives them | -- | `drop-release-rig.md`, `contact-measurements.yaml` |

---

## Health

**Tests, 2026-10-19**: on the v2 plant, red set unchanged, **no accepted
reds**. `test_sensor_modes.py` lost its IDEAL and SENSOR floors (user): quality
floors for two August policies on the old `tm151` level, which the v2 plant
took below them (odo_ahrs 0.72 against 0.78) and nothing uses; the policy in
use on its own sensors is `test_odometry_in_the_loop.py` (14 passed on v2).
Its wiring guards stay. `test_teleop.py::test_swing_lie_down_get_up_and_auto_up`'s fall is now
teleop's own `.` shove (8 N sideways, 0.35 s), checked for having gone past
60 deg at any point. It was a -8 rad/s roll rate set in one step, checked
still lying 2 s later: at the v2 plant that kick left the bike still on its
roof, so the auto-up fired at ~1 s and had it up by 2 s -- red for being
fast. With the key it tumbles, lands at ~-80 deg by 1.4 s, is up by ~3 s (`pytest -n 10 --dist loadgroup`, 56 s). The Warp parity tests
(`test_righting_linkage_friction.py`, and the steer-rate and style-bit
kernels) skip here; they passed in `aow-sim-gpu` (Warp 1.18, CPU device),
which has no pytest -- run their functions directly.
The 15 registered reds are all the deprecated LQR and are only judged under
`pytest -m lqr`. `prospective` (the endurance minute, skipped unless named)
passes on the pointer: 0/18 falls.

**Digests, 2026-10-21**: `plant_digest` 6446d8fa5b925702 -- the XC330s'
firmware registers moved from `onboard.yaml` into the plant
(`actuators.steer_gains`, `righting.module.gains`), with the sim's gains now
derived from them: steer kp 2.3 -> 2.30004, righting kp/kv 1.48 / 0.0473 ->
1.4795 / 0.04734 (the old values were the same derivations, rounded).
`deploy/bundle.npz` re-exported; the 4327521f one kept as
`deploy/bundle_plant-4327521f.npz`. Before it, 4327521f0ca7a1cb -- NO physics
change from 6c0ad5cd: `bike_params.yaml` split into the bike, `onboard.yaml`
and `config/studies/` (whose blocks `plant_digest` now leaves out); the
default model is bit-identical and so is the bundle but for its digests
(the 6c0ad5cd one kept as `deploy/bundle_plant-6c0ad5cd.npz`). Before it,
6c0ad5cd33676eb5 -- the bike weighed by item: `chassis.mass` (cables + fasteners) 30 GUESS -> 61 g
measured, the drive belts 4 GUESS -> 15 g each; the model is 53 g heavier
(1157 g whole). `deploy/bundle.npz` re-exported (worst LQR fit R^2 0.985).
Before it, 28946489e64c05de -- NO physics change from 0c0518e9 (the config/
reorg moved paths only); its bundle, rebuilt from that commit's params, is
`deploy/bundle_plant-28946489.npz`. `design_digest` 5fe8c19379988031
(unchanged). 0c0518e9 is the v2 bike as built (10-19), its bundle
`deploy/bundle_plant-0c0518e9.npz`. Before 0c0518e9: 7893a119, the steer's kp 2.3 + armature, then
4b5355ed and bc66ca20; `deploy/bundle_plant-bc66ca20.npz` is untouched. `config/ahrs_mount.yaml`
is the CAD's 45 deg nose-down mount (source: design, pinned equal to
bike.ahrs.quat by a test), and `hw/ahrs.MountCalibration` now applies it on
the correct side: before 10-19 it composed on the LEFT and rotated vectors by
the inverse, harmless near identity and wrong at 45 deg (roll would have read
as roll + yaw). `capture` still zeroes a reference pose, now as a separate
earth offset.
**To the Pi:** `scripts/bike.sh sync` carries the params and the bundle
together (`deploy/` is gitignored, so a commit does not, but the sync's
rsync does). History: `params-digest-split.md`.

**Which exports carry today's plant:** none -- the v2 bike (10-19) moved
the whole mass layout, the rake and the AHRS, so EVERY export is now
provisional by behaviour, not only by digest. Before that, none since 10-17. The 23 on bc66ca20 --
every `general_puffer_*` checkpoint, `general_rl_v3_s0..s4`,
`general_rl_v3_mingru_s0..s4` -- trained on the OLD STEER (kp 2.0, no rotor
inertia) and the exact steer rate. OUTSTANDING: the steer is driven by every
policy, so unlike the crank this is behaviour, not just digest; the next run
(rl_general_v3, which now also observes the Pi's steer rate) retrains on both.
The crank's linkage friction and armature are ACCEPTED as before: the policies never drive the crank
(GeneralEnv holds it at stow, where the term is its 51 mN m constant, and the
GPU port has no crank at all), so they are stale by digest, not by
behaviour -- unmeasured; one `per_command.py` grid would settle it. Everything else in `moves/` trained on an
older plant (old front wheel, wingless or V1 module) and is provisional.
Read a move's own `plant_digest`, never its name.

**Env fixes of 10-14 that split the history** (re-run `per_command.py`
rather than compare across the date): eval rows carried a steer kick from
the row before (`reset_actuator_state`); the obs's prev_action was one step
stale in training; the reset obs read MuJoCo truth for roll, rates and pitch.
`queue_general_v3` trained before them, the MinGRU arm and every GPU run
after.

**Open, known:**

- **`tm151_filter` is unmeasured under sustained acceleration** (turns,
  crab). The accelerometer weighting fitted on the drop rig fixed contact
  bounce reading as tilt; long accelerations are untested.
- **A MinGRU policy latches state at reset**: most slow units are written
  in the first steps after a reset (`analysis/policy/mingru_memory.py`), so
  a bogus first observation can persist. Replay resets the hidden state per
  engage.
- **Not yet checked on the bike**: the mirror's front-wheel spin fix
  (10-21); the coasting lateral drift at sharp steer
  (`odometry-sharp-steer.md`).

---

## What drives the bike

`control.general_move` = **`general_puffer_fixes_72M`** (10-16; was
`general_rl_smooth_temporal` from 10-09). Numbers in the RL row above.
Teleop flies it on `tm151_filter` by default (`--general NAME` for another);
the scripted plays (`--play config/plays/*.yaml`, End) were tuned on
`seed1_72M`.

**`moves/` layout (10-16).** The top level holds the pointer, today's GPU
finals (`fixes_52M`, `seed1_72M`, `next_72M`), `v3_s1`, `v3_mingru_s0`, the
previous pointer, and the moves teleop's keys and the tests name (`flick*`,
`ball_rl`, `pivot_rl`, `general_rl`). The other 84 are in `moves/archive/`,
tracked, and still load by bare name (`control.flick.move_path`); they are
out of the teleop `,` menu. Retiring a move is a `mv`. `moves-layout.md`.

The previous pointer, `smooth_temporal` (10-09 to 10-16): odometry estimate
+ the plain `tm151` level at tau 0.19 (before `tm151_filter`),
detailed drivetrain at P 100, the temporal smoothness loss, stood the
endurance minute; trained on an older plant. Pointer history:
`sensor-workstream.md`, `drivetrain-model.md`.

**Do not compare `metrics:` blocks across `moves/*.yaml`**; re-run
`analysis/policy/per_command.py` (`CLAUDE.md`).

---

## The guesses

**5 `source: GUESS`, 23 `estimate`, 8 `model` in `config/bike_params.yaml`** (10-21). The CAD-volume masses are `estimate`, modelling choices (`crown_radius`, the ideal drive's losses) `model`; GUESS is left for unknown physics (contact `solimp` and friction, the roller centring spring, the rollcage sheet). Never promote one
quietly: promoting is a physical parameter change (the `CLAUDE.md`
checklist).

| parameter | value | what is known | dies at |
|---|---|---|---|
| `contact_solimp` | MuJoCo stock | hand drops: the real contact is softer at first touch and stiffer at peak than any `solref` gives | the contact bench, if the bike needs it |
| `friction_sliding` | 0.9 | nothing | P0b pull test, same |
| `friction_torsional` | 0.005 | nothing; the risk is setting it high | same |
| `drivetrain_model.roller_slop.centring_stiffness` | 1e-3 | by hand, a released roller "springs back, not every time": the soft, harmless side. Stiffness x10 drops `smooth_temporal`'s score 0.818 -> 0.598; x0.1 or no slop moves it <= 0.005 (`analysis/policy/slop_sensitivity.py`) | -- |
| `righting.rollcage.mass.sheet` | 35 g | material and thickness assumed | the sheet |

**Estimates** (23): the printed parts' and the righting module's masses
from CAD volumes, the rollcage cages and caps (the caps will be lighter),
the fork (and the front wheel as 86 g less it). The bike as weighed
(10-21, no blades, battery or rollcage) is 772 g against the model's 791;
~11 g of that is the wing hubs' stubs, in the CAD and not on the bike.
**Model** (8): `crown_radius` (the tread is a 10 mm flat, modelled as a
crown), the ideal drive's armature and losses, the XC330 stick speed, the
roller-slop damping (critical, to match "no visible overshoot").

**How the drivetrain is carried.** The detailed drive is the
`drivetrain_model:` block in `bike_params.yaml` (since 10-02);
`drivetrain_model.base_params` gives the IDEAL drive, which the LQR is
designed on and the flick/pivot/ball envs run on. **A model built with the
block IS the ideal plant until `DrivetrainSim.attach` switches it** (native
drive zeroed, measured rotor inertia, slop tendons on; `release` switches
back, both pinned bit-exact by tests), so loops that never attach the hook
-- contact studies, settles -- run on the ideal drive, and the hooked ones
(RL envs, teleop, `balance.run`, `record`) on the detailed one;
`drivetrain_model.attach_hooks` is the one call a physics loop needs. A
policy records its own copy in its `moves/*.yaml`, and teleop, the eval env
and the endurance test rebuild that policy's plant from it.

**The contact as it ships:**

    contact_solref: [0.005, 1.0]     # positive: (timeconst_s, dampratio)
    contact_solimp: [0.9, 0.95, 0.001, 0.5, 2.0]   # MuJoCo stock, GUESS

- Settled on the bike (`analysis/contact/contact_calibration.py`): rear 0.148 mm at
  its own 5.8 N, 0.57 mm at 44 N; front 0.049 mm at 4.2 N.
- **Critically damped, so it cannot bounce.** The real front wheel does.
- **Not a stiffness in N/m**: the sink scales with the masses compiled around
  the contact, so fit on the bike's own model after the weighing (the rear
  acts like a 134 g body, the front like 215 g). `dampratio` enters only the
  stiffness, as 1/dampratio^2; label any table by sink in mm.
- Plan: measure, then the negative `solref` form. `dmin` 0.9 -> 0.5 moves the
  rear rest sink 3.6x.

**Bench pre-tests (rough, not fits; `contact-measurements.yaml`):**

| dial, 2026-10-01: sink added by 26.7 N from the wheel's own weight | measured | sim |
|---|---|---|
| one roller, flat | 0.36 mm | 0.36 |
| two small ends | 0.36 | ~0.24 |
| front | 0.36 | 0.31 |
| one roller, big end | 0.61 | 0.47 |
| two big ends | 0.48 | 0.30 |

Drop, 2026-10-02, front wheel + fork (86 g) by hand onto the force sensor:
restitution ~0.40 over 1-2.6 mm, contact ~7 ms, peaks 11-16 N. `dampratio`
~0.3 matches the bounce, but the sim's peaks are 20-30% lower and its
contacts ~40% shorter. **Heads-up:** `[0.005, 0.30]` is the floor sweep's
stiff-end cliff (survival 0.35), so the fix is not that pair alone.

Outstanding with the bench fit: the `contact_solimp` comment in
`bike_params.yaml` quotes posed numbers ("0.375 -> 0.647 mm, 2.3x"; settled
0.148 -> 0.527, 3.6x). Left until a measured claim replaces the whole comment.

---

## Risks, ranked

1. **No policy has trained over contact-stiffness variation**, CPU or GPU
   (`randomization:` varies mass, friction and actuator strength, not
   `solref`). Soft is safe (survival 1.00 from 1.8 mm of sink down to the
   shipped 0.15 mm); the cliff is at the stiff end, where the drop test
   points. The most likely single cause of a policy that works in sim and
   not on the floor (`floors-and-the-contact-model.md`).
2. **A policy can trade away a direction and the score cannot see it.** A
   third of the 09-10 sweep's runs drove backwards when told forward (12
   seeds). Every export since v3 drives both straight commands forward, but
   the eval score (`survive_rate x track_geo`) would rank a policy that
   drives backwards on a forward command above a good one; a directional
   gate is written up, not built (`eval-score-rewrite.md`).
3. **GPU policies train on MuJoCo Warp and are judged in C MuJoCo.** The two
   lay contacts slightly differently (0.7 mm / 0.24 deg apart after 100 ms
   open loop); the pointer flies fine in C on every check so far.
4. **The front tire and the wings still have a rubber roller's compliance.** A
   one-line `sim.contact_parts` edit now, not a refactor.
5. **Drive torque is the datasheet's.** The drivetrain fit is in PWM duty,
   which becomes torque through the datasheet's 1.6 N m stall torque; nothing
   has measured that scale. Training varies actuator strength +-15 %
   (`actuator_frac`).
6. **Left/right asymmetry and crab**, flipping sign between policies; crab
   is not trained.
7. **Steer homing at power-up is undesigned**: the turn count is RAM.
8. **Front-wheel liftoff breaks the velocity estimate.** The estimator
   assumes the front wheel rolls on the floor; nothing tells it when it
   lifts. The policy sees pitch, the estimate does not. Seen once, in an
   old smoothness arm that wheelied 79 mm with nothing pricing pitch
   (`sensor-workstream.md`).

---

## Where everything is written down

Every plan doc opens with a status banner; read that first.

| doc | owns |
|---|---|
| `first-physical-test.md` | the build order |
| `wishlist.md` | the showcase moves: ramps, ball shots, lie-down / get-up, wheelie |
| `untethered-setup.md` | the physical bike: power, wiring, wifi, onboard software |
| `pi-bench-bringup.md` | the Pi bench and the 09-15 to 09-18 bring-up log |
| `pre-assembly-bench-checklist.md` | DRAFT floor-rig ideas and experiments |
| `gpu-training.md` | MuJoCo Warp + PufferLib: phases, pod runbook, card choice, results |
| `web-demo-divergences.md` | the browser demo against teleop: what differs, findings about the shared sim (ball condim, limp flip), what might port |
| `moves-layout.md` | `moves/` and `moves/archive/` |
| `sensor-workstream.md` | the odometry/AHRS arc, policy standings and pointer history |
| `ahrs-fixture.md` | the TM151 fixture, its findings and CAD (parked) |
| `lqr-baseline.md` | the LQR's dated health log, `MIN_FIT_R2` |
| `odometry-rewrite.md` | the estimator and the 09-21 test rewrite |
| `eval-score-rewrite.md` | how policies are scored; the open directional gate |
| `seed-sweep-and-personalities.md` | the 12-seed sweep and failure taxonomy |
| `policy-smoothness-losses.md` | CAPS-style losses and the 10-01 results |
| `general-rl-improvements.md` | collected RL findings, reference |
| `mujoco-modeling-decisions.md` | why the model is built as it is |
| `floors-and-the-contact-model.md` | solref/solimp, the floor sweep (risk #1) and the contact split |
| `aow-contact-approximations.md` | contact surrogates, timestep/mesh |
| `drop-release-rig.md` | the drop rig, built and run (10-03) |
| `drivetrain-model.md` | the detailed drivetrain, the P 100 / P 400 question |
| `steering-design.md` | the steer in CAD and its first print, and the 09-21 to 09-30 log |
| `drive-design.md` | the rear drive in CAD |
| `righting-design.md` / `righting-linkage-margin.md` | the printed righting module / the linkage search |
| `righting-servo-model.md` | the XC330 model from the bench, and its status log |
| `bam-current-position.md` | BAM-style identification for mode 5 |
| `wing-linkage-design-and-optimization.md` | the righting mechanism; `wing-linkage-metal.md` superseded |
| `bike-assembly-design.md` | the whole bike in CAD, v1 (frozen 10-11) |
| `aow-bike-whole-v2.md` | the v2 bike and module updates (10-11) |
| `aow-bike-whole-v2-complete.md` | v2 continued: side rails, AHRS mount, cage stubs, carriers (10-16) |
| `cad-onshape-workflow.md` | the Onshape round trip and its quota |
| `self-righting.md` | where recovery stops being possible |
| `params-digest-split.md` | the two digests and their history |
| `asymmetric-actor-critic.md` | a parked option |
| `ball-shot-move.md` | the trained ball shot, parked |
| `plans/old/` | retired docs |

`docs/measurements/` holds the protocol/data pairs (contact, drivetrain,
servo, floor, omni wheel) and `servo-logging.md`. `docs/cad/` is generated:
regenerate, never hand-edit. `bench/` holds the force-sensor and drop-rig
tools, `bench/logs/` their captures. `analysis/gpu/` holds the GPU study's
pod scripts and results by phase.

---

## Explicitly not being worked on

The PD cascade (legacy). The analytic LQR (deprecated 10-09). The trajopt
moves `flick` / `flick_fwd` / `flip` (deprecated and skipped; `flick_rl` is
not), re-authored once the as-built mass is known. A live gamepad front-end.
A trained ball shot. The disturbance curriculum, parked until the params
files are reconciled. More no-diff runs.
