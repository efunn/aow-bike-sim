# Project status — 2026-10-19

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
3. **The onboard path runs on the Pi bench** (09-15 to 09-18): 100 Hz loop,
   TM151 pushing from flash, mirror, telemetry, recorder, fall/cut/re-arm,
   low-voltage cut. Torque on only with the bike held. No pack yet.
4. **Hardware is printed module by module.** Built: servo bench, rear
   drivetrain station, the front steer (09-30), the printed righting module
   (by 10-16; blades not yet fitted). Designed, not printed: the rear drive.
   CAD v2 (10-11) lays out the whole bike; since 10-16 the drive's case
   sides grow forward as its side rails (the chassis, in two halves).
5. **The goal is a functioning bike with sample videos, in weeks.** Ranked
   against that; the showcase moves are in `wishlist.md`.

---

## What to do next

| # | do | why now | doc |
|---|---|---|---|
| 1 | **Contact calibration**: stiffen the force sensor's mount (resolder or glue) and check it with a rigid-block drop; then the drop rig with added mass at the wheel (front, then rear), a joint `solref` + `solimp` fit with the dial (P0) data, and the P0b pull test and tread print (friction) | First rig run (10-03, front, 65 drops): the rig's load path reads ~7-30x softer than the dial says the wheel alone is, and the base rings at ~170-180 Hz -- probably the sensor mount, so fits so far describe the rig, not the tire. MuJoCo's contact is per unit mass (same bounce at any mass), so a fit reaches the bike only through each wheel's effective mass | `drop-release-rig.md`, `measurements/contact-protocol.md`, `analysis/contact/drop_rig_sim.py` |
| 2 | **Print the rear drive**, then fit the blades to the righting module | The bushing and end-play results are in (09-30); the module is built (10-16) | `drive-design.md`, `righting-design.md`, `aow-bike-whole-v2.md` |
| 3 | **Finish the drivetrain station**: firmware Velocity P and I, the D5 torque-scale check, fit the five drivetrain `GUESS`es | The sim cannot pick the gain (policies tie at P 100 / P 400; P 400 buzzes ~3x harder). Decide on the bike | `drivetrain-model.md`, `drivetrain-measurements.yaml` |
| 4 | **The style bit: find the pivot** (GPU training PAUSED, user 10-18: the bike is being built, policies get tried on it first) | Run 1 (add) refused in-place turns; run 2 (mult, 10-18) turns but STEERS, and stops 20-33 deg short on the big ones. Refusing is fixed, the pivot is not found. Also: 16k x horizon 16 trains in 18 min (4k: 29) but its bit-off hold creeps (-0.02..-0.05 m/s; run 1 ~0) -- decide whether that is acceptable before using 16k again | `wishlist.md`, `gpu-training.md` |
| 5 | **Sim: the eval score's directional gate** | `_score` is still `survive_rate x track_geo`; a policy that abandons a direction outranks good ones. Every export since v3 drives both straight commands forward, so this is insurance for future runs. Risk #2 | `eval-score-rewrite.md` |
| 6 | **Weigh the as-built parts into the v2 sim** (the sim moved to the v2 CAD 10-19: rake 20, wheelbase 200.5, every part placed) | Every frame mass is one `bike.frame_parts` / `bike.payload` entry, a CAD printed estimate or GUESS until weighed: change `mass` only. The six printed frame parts + spacers read 88.2 g from CAD against the slicer's 81. Then a policy on the v2 plant -- nothing has trained on it | `traces/weigh_v2_printed.txt`, `config/bike_params.yaml` `bike:` |

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
exist in teleop), the privileged critic, the odometry rewrite, the AHRS
fixture (parked 09-24), the self-righting linkage choice (parked 09-27).

---

## The workstreams

| workstream | state | blocker | doc |
|---|---|---|---|
| **Simulation & model** | Working; 19 `GUESS`es in bike_params. Plant digest **7893a119** (10-17): the measured front wheel (10-05), the detailed drivetrain as the default plant (10-02), the V2 righting module (10-09) with the V3 blade (10-14), its linkage's measured friction on the crank (10-17; righting needs 530 -> 600 counts at 12 V -- before the crank armature moved, not re-run). The steer XC330's measured small-signal response, kp 2.3 + rotor inertia 5.5e-4, also the crank's (10-17). XC330s from the bench (current law, load-proportional gearbox friction); the steer as its firmware runs it, 4 ms delay; headset friction since 09-30. Optional per-episode floor tilt for training, CPU and GPU, off by default (10-16) | Contact and masses, unmeasured | `mujoco-modeling-decisions.md`, `drivetrain-model.md` |
| **Control — RL** | Primary. Pointer `general_puffer_fixes_72M` (10-16): GPU, MinGRU (stateful), `tm151_filter` + odometry counts, detailed drivetrain, today's plant. On one `per_command.py --summary` grid (`--ahrs tm151_filter`): 0/20 falls, 7/9 turns resolved forward, 2/2 straight forward; chatter score 0.863, per-step action change 0.008 (v3 seeds 0.035-0.048); endurance minute **0/18 falls** (10-16). Two GPU seeds pick the same direction on all 20 grid commands. CPU arms on today's plant, finished on the box: `queue_general_v3` (5 seeds, MLP) and `queue_general_v3_mingru` (5 seeds, MinGRU actor); turns forward 4-8/9 (s0-s2 measured) and 5-7/9 (all five), 0 falls. `v3_s1` and `v3_mingru_s0` are on top of `moves/`. All of them trained before plant 7893a119 (10-17), so provisional. **The style bit** (10-17): a command entry, `obs_style`, active only at a zero speed command, that pays for the front wheel holding the world heading it had when the bit came on (`general_spec.wheel_err`; obs 17 -> 20). CPU and GPU env, export, teleop Home, `per_command --style`. Baseline: no policy pivots (wheel/yaw 0.92-1.05). **Run 2** (`general_puffer_mult_*`, 10-18, multiplier form, 16k worlds): turns with the bit on but steered, no pivot; bit-off hold creeps 0.02-0.05 m/s. **Run 1** (`general_puffer_style_{52,72,80}M`, GPU, plant 7893a119): bit off, as good as the pointer (0/20 falls, 7/9 forward, standstill 0.11-0.19 m/s); **bit on, it mostly refuses the in-place turn** (yaw 0-17 deg on 9 of 15 spin episodes, wheel held), one real pivot that overshot, one fall. The reward lets refusing outbid turning (`wishlist.md`) | Crab never trained (`v_lat_frac` 0; crab L/R ~0.45/0.63 on the pointer); not flown on the bike | `gpu-training.md`, `general-rl-improvements.md`, `policy-smoothness-losses.md` |
| **GPU training** | Phases 1-3 done (10-14 to 10-16): MuJoCo Warp env as one CUDA graph per policy step, bridged into PufferLib 5.0's CUDA trainer, patched for reward scale, advantage normalisation, the temporal smoothness term and a paced shared curriculum. Checkpoints convert to moves (`gpu/export_policy.py`). Cards: 5090 12 min / $0.23 per 72M steps, 4090 17 min, A40 38 min. Runs are not bit-repeatable (float atomics). **Style run (10-17):** 5090, 4096 worlds, 80M steps in 29 min of training, ~34 min / ~$0.68 of pod; 20-wide obs end to end (`install.sh` sets the CUDA side's width from the graph). The first GPU run of the Pi-style steer rate, no alarm. `pod_launch.sh` now writes `results/watch.log` itself. **Run 2 (10-18):** 16k worlds x horizon 16 (same batch), 80M steps in 18 min of training at 82-90k steps/s; `run_pod.sh` scales the curriculum lead with the world count (a flat 0.1 at 16k climbed 4x slower per sample). Bit-off hold creeps at 16k (see row 4) | Phase 4 (MLP / loss parity) and 5 (five GPU seeds vs v3's five on one grid) not started; no sim-to-sim check of a GPU policy in C beyond the eval grid | `gpu-training.md` |
| **Sensor modelling** | Largely done. TM151 measured on the fixture: a complementary filter, tau 0.19 s at rest to ~1 s moving, ~0.2-0.3 deg RMS at one mount against the sim's 1.5. `tm151_filter` with the accelerometer weighting from the drop rig (10-07) is the training level since 10-09, provisional; unproven on sustained acceleration | The bike's own AHRS log; the wave cams (sustained acceleration) | `sensor-workstream.md`, `ahrs-fixture.md` |
| **Control — LQR** | **Deprecated 10-09.** `lqr` tests skipped unless named (`pytest -m lqr`: the 15 registered reds). Never drove the bike. The code stays: the bundle's gain schedule is built on it (the crawl fallback serves only 2-action feedforward moves, no general policy). Its settings are `config/studies/analytic_control.yaml` since 10-21. Last state (10-05): stands on truth and `tm151_filter`, falls turning at 0.8 m/s | none: not being tuned | `lqr-baseline.md` |
| **Hardware / onboard** | Pi 3 bench proven: tick jitter p99 < 1 ms with four servos energised. 2.4 GHz house wifi (decided: a router, no AP). The SBC that ships: open | Chassis, pack | `pi-bench-bringup.md`, `untethered-setup.md` |
| **CAD** | Steer printed and print-checked; the righting module printed (blades not on). **v2 (`aow-bike-whole-v2.md`, then `aow-bike-whole-v2-complete.md` from 10-16, `aow-bike-whole-v2-touchups.md` from 10-19). The CAD's rake is 20 deg since 10-19 (keeping the wheelbase); the sim's is still 15 (row 6); frozen v1 keeps 15.** Drive: fixture mock off, the user's chainstay finishing folded in; **its case sides are now the bike's SIDE RAILS** (10-16): each grows forward to the steering's side pad and down to the righting's bridge (now as wide as the gap between the rails), one 6-32 a side at each, ending fore and aft in a CAGE TONGUE (10-19) inside each rollcage ring and a full-width tab a side holding the TM151 on its own M2 holes, USB up. The steering and the righting slide out of the rails downward (ridges along the slide, grooves open at the rail's edge). Steering v2: servo out the FRONT, pads 3.65 to take the rail, ONE upper <-> lower screw, at the FRONT since 10-19 (its nut block's underside 20 deg from level; lower case ~12.2 g ASA). Righting: stubs and the V3 spline blade, centred; bridge lugs for the rails. Snug 6-32 nut slots (6.45 x 2.7, GUESS) in the v2 modules. **The power holder** (`cad.power`, 10-16): one printed panel on the left between the battery and the rail (printed -X), the power board on standoffs outboard of it square to the drive (XT30 up and back), the switch panel-mounted in a gusseted shelf round the XT30; one strap round the battery near the switch (over its top, under its bottom, up the panel's outer face inside the switch). Two 6-32 into the LEFT rail only, on two left-only case-side ears fore and aft of the board (nut in the panel, the aft screw cut to 8); it slides in from the back + top (the drive's -Z), clear of the Pi carrier; its aft edge reaches back to the aft lug only in an ear at the bottom (10-19). Both rear tongues carry a battery stop (behind it and along its side). **The rollcages** (10-19, `cad.bike_v2.rollcage`): one part, at the rear and turned 180 deg at the front; a 270 deg ring the blades' width (137.4) at z 100, 15 deep, printed Y-outboard; a bayonet for an elliptical half-dome cap (three 15 deg V-grooves, clockwise on; the cap's inside a 60 deg cone); a groove for the flexible sheet (1.0 to 0.5 over 3); a bar a side whose socket slides Y-inboard over the rail's tongue, its chin chamfered short of the bed, one 6-32 along Y (head outboard, under the cap; 3/8 in). The holder now comes out only after the rear ring. The checks now probe every rail screw hole open through the mating part's ridge (the holder's and steering v2's ridges had plugged theirs: fixed, 10-16). No tray: the battery is where it was. **The Pi + U2D2 carrier** (`cad.pi_carrier`, 10-16): a plate over the front face, the Pi's PCB's footprint exactly, 1 off the rails' top edges; the U2D2 under it between the drive pulleys on two M2 (its JSTs in front of the AHRS's housing; their generous keep-out still meets it 6 mm into the lift), the Pi over it on four separate printed spacers, 4-40 self-tapping into the plate. Its three screws and the holder's fore one go through; only the holder's aft is cut down (short of the battery). Three 6-32 into the rails: one on the LEFT front ear (run on forward), one a side low in the rails' lower arms above the righting's bridge; it lifts out straight up. The v2 bike: `--check` clean at rest, steered, every pose, and with the Pi carrier lifted 3..15 mm (84 bodies, 10-19). v1 (`cad.bike`) is frozen (as-built nut slots, no rails) and still builds | Print-fit the snug nut slots and the M2 tabs; print-fit the holder's cut screws and nuts, and the Pi carrier's spacers, pilots and nuts; room for a screwdriver at the Pi carrier's low screws past the blades (unchecked); the strap (over the servo wires' exit); print-fit the bayonet (0.2), the cage sockets (0.15) and the cap's cone; the sheet's radius (66.2, GUESS) and the sheet itself (not modelled; the blades sit across it); the user's mocks in `aow-bike-whole-v2` (rolled back, not deleted); the straps (`aow-bike-whole-v2-complete.md`); delete the drive's four hand features; CoM not computed; wing-to-blade (not yet, user) | `aow-bike-whole-v2-touchups.md`, `aow-bike-whole-v2-complete.md`, `aow-bike-whole-v2.md`, `cad-onshape-workflow.md` |
| **Self-righting** | V2 linkage with the **V3 blade** is the default (10-14, user): level at 116.98 deg (commanded 116.9), lying at 85 deg, 566 counts in the sim (10-14); `righting_current_sweep.py` at the real +-117 stroke (it read V1's 136.6 until 10-17): 530 counts frictionless, **600 with the linkage friction** (10-17), toe stop at ~130. V2's blade is `--blade config/linkage/righting_blade.yaml`; a blade file may carry its own `mass` and `crank_travel_deg`. **Lie down / get up / auto-up** (10-16): `control/swing_sit.py`, teleop PageDown / PageUp, and by itself after a fall (`--no-auto-up`); a fall after the hand-off folds the blade and re-arms the auto-up. Sim only: down 5.8 s at <= 102 deg/s, up 3.3 s; 24/24 in teleop | Weigh the printed parts; a crank coast-down for the XC330's reflected inertia; fly lie-down / get-up on the bike | `righting-linkage-margin.md`, `righting-servo-model.md`, `wishlist.md` |
| **Contact bench** | Drop rig built and run (10-03): release-to-impact timed, analysed at the contact through the arm (lever 205/124, m_eff 89 g fitted -- to re-derive from the weighed parts). MuJoCo twin `analysis/contact/drop_rig_sim.py --fit`. AHRS mode drawn (10-06): a jam-on follower tip and an 11-cam wave kit, nothing printed. `force_drop.py --ahrs` (10-07) logs the TM151 and the servo with the drops, one clock. On the bar the TM151 steps within ~0.07 deg of the cam's; the arm is not rigid, so stations do not share a rotation | Print the wave cams + the tip, then `bench/wave_run.py` (sustained acceleration); power for the force sensor on the Pi (three USB devices reboot-loop it); a local truth per station; the sensor epoxied down; the arm's parts weighed | `drop-release-rig.md`, `contact-measurements.yaml` |

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
4b5355ed and bc66ca20; `deploy/bundle_plant-bc66ca20.npz` is untouched. NOT yet on the Pi, whose
params and bundle are both still bc66ca20 (consistent). `config/ahrs_mount.yaml`
is the CAD's 45 deg nose-down mount (source: design, pinned equal to
bike.ahrs.quat by a test), and `hw/ahrs.MountCalibration` now applies it on
the correct side: before 10-19 it composed on the LEFT and rotated vectors by
the inverse, harmless near identity and wrong at 45 deg (roll would have read
as roll + yaw). `capture` still zeroes a reference pose, now as a separate
earth offset.
**Copy it to the Pi by hand**: `deploy/` is gitignored. History:
`params-digest-split.md`.

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

- **AHRS error costs far more without the velocity estimate**, for the LQR
  and RL alike; mechanism untested (`lqr-baseline.md`).
- **`tm151_filter` is provisional.** The unweighted filter read contact
  bounce as ~8 deg of tilt; the accelerometer weighting fitted on the drop
  rig and fixture (`FILTER_ACC_D0_G` 0.12, `FILTER_ACC_HOLD_S` 0.1) fixed
  that. Unmeasured: sustained acceleration (turns, crab -- 5.4-5.8 deg
  there). Its `ahrs_tau_s` only times the 0.1 deg residual wander (2.0 s);
  the filter's own 0.19 / 1.0 s is built in.
- **A MinGRU policy latches state at reset.** On `fixes_52M`, 7 of 9
  slow units are written in the first steps after a reset and held for the
  episode, so a bogus first observation can persist (`analysis/policy/mingru_memory.py`,
  `gpu-training.md`). Replay must reset the hidden state per engage.
- **Why `fixes_72M` drops a swing-sit hand-off on the base params** that
  `fixes_52M` catches: one trace, not chased (it recovered on its own
  drivetrain; the sequencer now retries).
- **The mirror's wifi stalls** (100-550 ms gaps, 09-17) did not repeat;
  cause unknown. Probe when it chugs again (`pi-bench-bringup.md`).
- **Not yet checked on hardware**: the rear wheel's sent angle against a
  hand-turned wheel; whether the onboard AHRS reader has the fixture logger's
  0.19 s stall; one unrecorded preflight failure after a reboot (09-18).
- **`MIN_FIT_R2` 0.93** is a bar to re-derive once the contact is measured:
  the LQR fit gets worse as the contact gets more realistic.

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

The previous pointer, `smooth_temporal`: odometry estimate + TM151 at 0.19,
detailed drivetrain at P 100, the temporal smoothness loss, stood the
endurance minute; trained on an older plant. Pointer history:
`sensor-workstream.md`, `drivetrain-model.md`.

**No-diff probe, `general_rl_nodiff_2` (10-09, one seed, not the pointer).**
A one-off test of how a plain bike could use the swing linkage with tip
masses to balance; NOT a line of work -- no further no-diff runs are planned
(user, 10-09). Common-mode drive + steer + the four-bar swing wings with
100 g tips, the differential faded out by the curriculum (zero by ~10M of
18M steps), a wing on the floor a fall. On its own sensors it stands the
endurance minute 0/18 falls (`tm151` and `tm151_filter` both) -- but CREEPS
forward ~7 cm/s on a zero command (4.4-5.7 m in 60 s). The wing is BALLAST:
the crank sits near -95 deg, the bike leans ~3.8 deg, the low wing hovers
4-6 mm off the floor at its closest and ~11 mm median, never touching.
Frozen at that angle it still stands 0/18 (roll spread 0.8 deg vs 1.25
live); frozen at stow 18/18 fall (MTBF ~11 s). The 5 s training eval cannot
see the creep. Trained on the V1 linkage (`swing_linkage_smaller`), NOT V2,
the bike's module; it does not transfer. `rl_general_nodiff.yaml` was moved
to V2 with the rest of the configs, for consistency only.

**Do not compare `metrics:` blocks across `moves/*.yaml`**; re-run
`analysis/policy/per_command.py` (`CLAUDE.md`).

---

## The guesses

**5 `source: GUESS`, 23 `estimate`, 8 `model` in `config/bike_params.yaml`** (10-21). The CAD-volume masses are `estimate`, modelling choices (`crown_radius`, the ideal drive's losses) `model`; GUESS is left for unknown physics (contact `solimp` and friction, the roller centring spring, the rollcage sheet). Never promote one
quietly: promoting is a physical parameter change (the `CLAUDE.md`
checklist).

| parameter | value | what is known | dies at |
|---|---|---|---|
| `chassis.mass` (the residual) | 0.061 kg measured, spread over the frame parts by mass | the cables and 6-32s weighed by item (10-21); the 4-40s and M2s not; the rollcage's and wings' 6-32s not fitted | -- (weighed; recount when the rollcage and wings go on) |
| **the bike as weighed** (10-21: no blades, battery or rollcage) | 772 g; the model 791 | model 19 g heavy; ~11 g of it the wing hubs' stubs (in the CAD, not fitted) | the printed parts weighed |
| `frame_parts` printed masses (12 parts) | 7.7-20.7 g each, ~146 g | CAD printed estimate; for the batch the estimate is 9 % over the slicer's total; nothing weighed | `traces/weigh_v2_printed.txt` |
| `righting.rollcage` mass | 68.5 + 108.5 + 35 g | cages and caps CAD printed estimates; the caps WILL be lighter (user, 10-21), their design not done; the sheet's material and thickness assumed | the caps' design, then weigh them |
| `righting.module` masses (crank, coupler, wing_hub, blade, fixed) | 11.2 / 2.3 / 14.9 / 38.8 / 41.8 g (+ the 23 g XC330 on its own) | CAD check volumes (10-07), 2 perimeters / 15 % infill, steel pins | weigh the printed parts |
| `fork_mass` + `front_wheel.mass` | 0.025 + 0.060 kg | **86 g together** (09-30), not split | a scale |
| `front_wheel.crown_radius` | 80 mm | the real tread is a 10 mm flat; the crown stands in for the tire flattening under load | the dial test repeated with the wheel tilted 1-2 deg |
| `contact_solimp` | MuJoCo stock | hand drops: the real contact is softer at first touch and stiffer at peak than any `solref` gives | the drop rig + bench fit |
| `friction_sliding` | 0.9 | nothing | P0b pull test |
| `friction_torsional` | 0.005 | deferred; the risk is setting it high | later |
| `drivetrain_model.roller_slop.centring_stiffness` / `.damping` | 1e-3 / 2.9e-5 | the only damping of a roller inside its +-7.8 deg play (zeta 0.95). Sensitivity (10-02, `analysis/policy/slop_sensitivity.py`): stiffness x0.1 or no slop moves `smooth_temporal` <= 0.005; **x10 drops it 0.818 -> 0.598**. The hand note "springs back, not every time" points SOFT, the harmless side | qualitative first: does a released roller snap back (stiff) or drift / stick (soft)? |
| `righting_sign` | +1 | a build decision; the render only | mounting the motor |
| `hockey.ball.radius` / `.mass` | 33.5 mm / 60 g | used by teleop's `--hockey` and the plays | the ball |

So: the masses die on a scale once the as-built parts exist; three with the
contact work, plus the front crown with a tilted dial test; the roller
slop's spring and damping with a measured in-play flick; three are parked
(`righting_sign`, the hockey ball). The remaining drivetrain `GUESS`es wait
on the station (next, #3).

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
   nothing gates it (next, #4; `seed-sweep-and-personalities.md`).
3. **A GPU-trained policy is checked against C MuJoCo only on the eval
   grid.** MJWarp lays more contact points than C (0.7 mm / 0.24 deg apart
   in 100 ms open loop); the pointer flies fine in C on every test so far,
   but phase 5's sim-to-sim comparison has not been done (`gpu-training.md`).
4. **The front tire and the wings still have a rubber roller's compliance.** A
   one-line `sim.contact_parts` edit now, not a refactor.
5. **Authority.** The torque scale is still the datasheet's 1.6 N m;
   training randomises actuator strength +-15 % (`actuator_frac`), the
   station's D5 check measures it.
6. **Left/right asymmetry and crab**, flipping sign between policies; crab
   is not trained.
7. **Steer homing at power-up is undesigned**: the turn count is RAM.
8. **Front-wheel liftoff is invisible to the estimator** (79 mm in one arm).

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
