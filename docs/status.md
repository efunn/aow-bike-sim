# Project status — 2026-11-03

**How to read this.** The navigation layer: what is true now, what to do next,
and where the detail lives. Rewritten, not appended to. **For the reasoning,
follow the link — it is not repeated here.**

Rewritten 2026-10-02, 10-16, 10-26 and 10-31; v6 updated 11-02, the manual crank, v6's inputs and the righting policy and bench 11-03. On 10-26 the dated logs that had
collected here were moved, verbatim, to the doc that owns each (a section
headed "from status.md" at its end): the 10-17 bring-up and first bench
policy runs (`pi-bench-bringup.md`), the righting module's first powered
runs (`righting-servo-model.md`), the digest history to 10-21
(`params-digest-split.md`), the v2 bike's masses in the sim
(`mujoco-modeling-decisions.md`). The 10-24 to 10-26 shake hunt is in
`physical-bike-tests.md`.

---

## Where the project is, in five lines

1. **The simulator is trusted for control design**: parametric MuJoCo, an
   omni-wheel contact, the bike's own sensors in the loop and in training,
   measured XC330 laws, the detailed drivetrain, the righting module, and
   since 10-31 in the default plant: the wheel play, the floor's measured
   grip (mu 0.52), the front axle's drag, and the tire's and rollers' own
   contact damping from drops of the bike itself. Contact stiffness and
   shape (`solimp`) are still the shipped guesses.
2. **RL drives, and it trains on a rented GPU**: MinGRU policies, MuJoCo
   Warp + PufferLib, 72-80M steps in 12 min (5090) to 89 min (A40). The
   pointer is `general_puffer_a40_v4_80M` since 10-26 (was
   `general_puffer_fixes_72M`, 10-16). v5 (10-29) flown 10-30: s1 best,
   softslip unstable (user). v6 (11-02), the first trained on the measured
   plant, never falls on it and is too cautious: slow in reverse and on big
   turns, on the bike and in the sim alike.
   The analytic LQR is DEPRECATED (10-09).
3. **The onboard path runs on the Pi**: 100 Hz loop, TM151, mirror,
   telemetry, recorder, fall/cut/re-arm, low-voltage cut, and the swing-sit
   lie-down / get-up from the mirror.
4. **The bike balances on the floor (10-24), and since 10-26 without the
   shake**: `general_puffer_a40_v4_80M`, trained with the drivetrain detent
   and the wheel play the floor runs fitted, cut the 9-10 Hz oscillation 4x
   in roll rate against the pointer flown minutes later
   (`physical-bike-tests.md`, test 9).
5. **The first goal is met: a functioning bike with sample videos** (set
   2026-09-08; the demo links went into the README 2026-10-30). Since then
   there is no fixed goal: the work follows ideas as they come (user,
   2026-11-02), so the table below is a menu, not a ranking. Showcase moves
   are in `wishlist.md`.

---

## What to do next

| # | do | why now | doc |
|---|---|---|---|
| 1 | **v6's caution: which training change bought it** | v6 (11-02, `general_puffer_5090_v6_s{1..4}_{52,72,80}M`) falls 0/20 on the measured plant at every mu 0.36-0.68, where v5 s1 falls 10/20 and v4 9/20 -- but reverses at 0.44 of the command on the bike (v5 s1 0.76) and settles big heading changes in 2.3 s (v5 s1 1.3, v4 1.0); on the grid its big turns take 5.8-10.3 s to 10 deg (`chatter.py`), and it stands still (`standstill.py`). `analysis/policy/cmd_replay.py` flies a bike run's own commands in the sim and shows the same. Taking the axle drag or the play off the SIM's bike changes neither, so it is learned. New in v6's training against v5 s1: the wheels' bouncy contacts, mu 0.36-0.68 (was 0.72-1.08), the axle drag 0-0.035; the play is v5's. One warm-start fine-tune from v5 s1 per change (`gpu.sh up ... LOAD=`, untried on a real run) would separate them. Not, at runtime, the three inputs v5 added (11-03): masking the detent phase and diff rate (zeroed, phase fixed, or rate only) keeps the grid at 0-1 / 20 falls and moves v6 s1's replay of run 261102-203829 between forward 0.66-0.75 and reverse 0.53-0.62 of the command, against 0.75 / 0.58 unmasked -- whether TRAINING with them made it cautious is untested. Nor the rear axle's sideways slide (new in v5): v4 on the measured plant falls 9 / 20 with its own play, 7 / 20 with the slide added, 3 / 20 on today's play (front cut to a third) | `gpu-training.md` (v6), `results_5090_v6/cmd_replay.txt` |
| 2 | **v4 is the pointer; the standing-snap falls are explained** | 10-31: left-turn snaps fell 7 / 11 after the 10-28 axle nut (3 / 41 before; right turns never), and 1 / 13 with the nut loosened. The sim on the measured plant gives 16 / 20, 1 / 20 and 0 / 20 for the same three axle drags, left only: the floor's grip and the axle's drag together, and v4's harder left turn. v5 s1 falls both ways on that plant (15 / 10 at the loosened axle) -- a prediction to fly. The reversing wobble and steer swing of 10-28 (test 10) stand; v4 at rest is unchanged | `physical-bike-tests.md` (tests 10, 11) |
| 3 | **Drivetrain rolled in (11-04)**: `bike_params.yaml` now carries the 10-24 sawtooth detent and the filmed roller spring (the overlay `traces/drivetrain/drivetrain_sawtooth_1024.yaml`, which every policy since v4 trained on), plus the diff's speed drag: none up to 4 rad/s of ring-vs-hub, then 0.12 N m s/rad (`detent.viscous_*`, estimate; Python and the Warp kernel, parity 8e-8 N m on CPU Warp). Replaying the 10-24 coast's recorded duty on the testbed: the diff spins 1.73 rad/s (servo) at duty 0.4 as the bike does (3.18 without), PWM-0 coasts halve in 10 ms (bike 8-10), torque-off in 15 (bike 9). A drag from 0 (0.028) turned every double jump of the filmed creep into a single (49/0, 47/0; bike 25/13, 5/21); with the onset, 39/5 and 37/5 (35/7 and 35/6 without drag). One duty, so onset and slope trade off; a coast at more duties would pin them. Common mode is still under the bike's drag (spins 3.36 vs 2.68; the 09-13 bench 2.88-2.97), left as is. Which policies flew it: none trained with the drag | `physical-bike-tests.md` (test 3, item 8) |
| 4 | **Blades and rollcage**: the sim's lying rest | The bike lies down and gets up from the mirror (10-26). The caps and sheet will not be fitted (11-03): the cage is the two rings, each its own contact (its convex hull, closed across its opening), nothing between them. RE-RUN 11-03 on the rings and the weighed bike (Self-righting, "What the righting sim lacks"): lying, 74.7-75.8 deg against the bike's 73.5-74.0, but on the blade where the bike is on its cage; the bike's lift torque ~0.2 N m over the model's. What follows is the capsule and the 1157 g model. The sim put the lying rest at crank ~77 deg past centre, the bike 26.5-28; the capsule was the first suspect, untested. The stepthrough's quasi-static V3 is within 5 deg of the bike's measured roll-vs-crank from crank 40 up, runs up to 10 deg ahead of it from 0 to 35, and rests at 76-81 deg roll where the bike lies at 74. The SIM flying the bike's own sequence (v4, right and left): its roll-vs-crank matches the bike's from crank 105 down to ~15 (within ~3 deg), then rolls on to 86 where the bike stops at 74; its lying crank sags to -80 (bike -26); its lift draws ~470 mA peak where the bike draws ~590; it reaches crank 110-111 where the bike stalls at 100-102 (left 96-101), props at 5 deg roll (bike 12), and on the right then tips over the top while propped, which the bike never has. The bike's lift TORQUE (Present Current through the servo model's k (|I| - i0)) is on the stepthrough's torque chart: 0.50-0.53 N m from crank 10 to 40 with the battery, 0.44-0.46 without, where the quasi-static V3 (battery in the model) needs 0.33-0.37 -- the model is ~0.15 N m light, part of it the lift's own acceleration | `righting-design.md`, `aow-bike-whole-v2-touchups.md` |
| 5 | **The style bit: find the pivot** (not a priority) | Run 2 (mult, 10-18) turns but steers, 20-33 deg short on big turns | `wishlist.md`, `gpu-training.md` |

Not next, deliberately: a trained (RL) ball shot, the privileged critic,
changes to the odometry estimator (`odometry-sharp-steer.md`), the AHRS
fixture (parked 09-24), the contact bench (a fallback).

---

## The workstreams

| workstream | state | blocker | doc |
|---|---|---|---|
| **Simulation & model** | The v2 bike since 10-19, plant digest **d6e4355c** (11-03; 47f14be2 and 46c263de before it the same day and 10-31, 6446d8fa from 10-21). **The bike as weighed (11-03)**: 1003 g as driven (battery, blades, rings, no lead), CoM 83.3 mm ahead of the rear axle (rod balance, 10-31) and **108.9 +-0.5 mm** above the ground, centred to +-0.5 mm sideways (hung from the rings' notches, both sides; finger balances read 111-126 and are not used). The model: 1003.1 g, 84.3 mm, 108.4 mm, 0.6 mm left. To get there: the rollcage's caps and sheet are not fitted, so the 212 g `roof` capsule is now the two rings (34.1 g each, the slicer; their own contact; mass at the CAD centroid, 9 mm under the drawn ring's; + their 6-32s); the cables (41 g) a part up high; the Pi's mass 10 mm to its port end; the blades the slicer's 38.1 g; `wing_hub` split (rocker + knuckle, steel pin, the slicer's stub, the wings' 6-32s); every other printed CAD estimate x `bike.printed_mass_scale` 0.930 (`params.PRINTED_PARTS`). In `bike_params.yaml` since 10-31: `wheel_play` (v5's: front axle 0.33 mm, headset 0.5, rear 2.5, rear sliding 1.0, total travel), `sim.contact_parts` (front tire and rollers, negative solref: shipped stiffness, damping 0.59 / 0.27 x critical from the pivot drops; since 11-03 the blades and rings, friction 0.10 from a pull on the bike lying on them), `sim.friction_sliding` 0.52, `bike.front_wheel.axle_friction_nm` 0.015. Setting the global `contact_solref` (a `floor:` overlay, teleop `--solref`, the contact studies) also clears the wheels' own (`params.with_contact_solref`), so the soft-floor exports replay as trained. Optional on top, off in `bike_params.yaml`: the sawtooth detent and measured roller spring, carried by a policy's move yaml | Contact stiffness and `solimp`, masses; the sway and steer-servo gaps (`physical-bike-tests.md`) | `mujoco-modeling-decisions.md`, `drivetrain-model.md`, `physical-bike-tests.md`, `contact-measurements.yaml` |
| **Control — RL** | **Pointer `general_puffer_a40_v4_80M`** (10-26, plant 54fdba72 = 6446d8fa + detent overlay + play, the Pi's steer rate, `tm151_filter` + odometry counts): 0/20 falls, 7/9 turns forward, 2/2 straight on its own plant, less standstill drift than the old pointer in the sim, the shake gone on the bike (next #1). Endurance minute (`-m prospective`, 18 x 60 s) passes; it flies the move's detent but not its play (the test does not read `wheel_play`). **v5 (10-29, not flown)**: `general_puffer_5090_v5floor_s{1..4}_{52,72,80}M`, the detent phase and diff rate observed, front play a third, the rear lateral slide, one floor per seed (s1 shipped, s2 soft, s3 slippery, s4 both); on their own floors all match v4 on the grid, s1 80M cleanest. On the bike (10-30, user): s1 the best of the four, softslip not stable, the detent phase's effect not visible; slip training gives a worse policy. **v6 (11-02)**: v5 s1's config on the measured plant (move digest e4e2d636 = 46c263de + the detent overlay), mu 0.36-0.68 and axle drag 0-0.035 randomized; four seeds on a secure 5090, ~90 min, ~$1.90. On that plant 0/20 falls for every seed and every mu tried, 7-8/9 turns forward, straight speed tracked; slow in reverse (grid -0.5: s1 -0.39, s3 -0.21; on joystick commands s2-s4 hardly reverse) and on big turns. Flown 11-02 (s1, a standstill minute and a drive): acts like the sim, "way too cautious" (user); the training-side cause is next #1. The old pointer `general_puffer_fixes_72M` (10-16, plant bc66ca20, before the v2 bike, exact-joint steer rate): same grid, endurance 0/18 falls. `general_puffer_rake20_{52,72,80}M` (10-23, v2 plant, no play): as good on the grid, wobbles at 1.2 m/s, shook like the pointer on the bike. Style bit runs 1-2 (10-17/18): no pivot yet. CPU arms (`general_rl_v3_*`, MLP and MinGRU actor) trained before 10-17's plant; `v3_s1` shakes at 11.2 Hz on the bike | Crab never trained (`v_lat_frac` 0) | `gpu-training.md`, `general-rl-improvements.md`, `next-retrain.md` |
| **GPU training** | Phases 1-3 done (10-14 to 10-16). `scripts/gpu.sh` runs the runbook; `SEEDS=N` runs N under MPS (4 on a 5090: 116k steps/s in total). Cards: 5090 12 min / $0.23 per 72M steps, 4090 17 min, A40 38 min at 4096 x 64 for the v2 plant; the v4 plant (play joints) ran 19.7 -> 16.0k steps/s on an A40, 80M in 89 min, ~$1.05 of pod (no 4090 / 5090 to be had 10-26). Runs are not bit-repeatable | Phase 4 (MLP / loss parity) and 5 (GPU vs CPU seeds) not started; per-world randomisation of the play not ported. `axle_friction_range` ported (10-31); the wheels' negative solref matches C on a CUDA 5090 too (11-02, rebound / fall 0.243 vs 0.260 front, 0.517 vs 0.520 rear; that pod installed Warp 1.19). **Warm start (11-02)**: `gpu.sh up ... LOAD=<.bin> LR= STEPS_M=` starts every seed from a checkpoint's weights (`puffer_aow.patch`; upstream loads only in eval); a 6.8M smoke run started at the trained entropy (0.45 against a fresh 4.24), moved the weights 2.1 %, and refused a truncated file -- weights only, the curriculum restarts | `gpu-training.md` |
| **Sensor modelling** | Largely done. TM151 measured on the fixture: a complementary filter, tau 0.19 s at rest to ~1 s moving. `tm151_filter` is the training level since 10-09; unproven on sustained acceleration | The wave cams (sustained acceleration) | `sensor-workstream.md`, `ahrs-fixture.md` |
| **Control — LQR** | **Deprecated 10-09.** `lqr` tests skipped unless named. The code stays: the bundle's gain schedule is built on it | none | `lqr-baseline.md` |
| **Hardware / onboard** | Pi 3B+, tick jitter p99 < 1 ms with four servos. drive_a is servo 102 (right) in `onboard.yaml`, compensating `k_roller`'s sign until the swap is reverted with it. `run_bike` rides out up to 3 bad servo reads in a row. The preflight accepts the CAD-located AHRS mount (`source: design`, 10-25). The AHRS enumerates only on the ethernet-side USB sockets. `scripts/bike.sh` runs sync / bench / run / hold / mirror / status. **Link drops (10-27) were the Mac idle-sleeping on battery** (display off at 2 min, sleep 1 min later; pad input over raw USB is not user activity): every unexplained drop that afternoon matched pmset's log to the second. The mirror now holds `caffeinate -di` while it runs, reopens a pad that drops or is plugged in late, and reads a dropped pad as neutral. **The Pi's clock is wrong after a power-up** until timesyncd reaches a server (28.5 min slow 14 s after one boot, 10-27; ~10.6 min in run 261027-162626's `created`); each record now carries an SNTP check at start and close (`hw/clock.py`), and `mirror_replay --info` prints the corrected start | Chassis, pack, the SBC that ships | `pi-bench-bringup.md`, `untethered-setup.md` |
| **CAD** | v2 is the bike as built (`cad.bike_v2`, 10-19), printed and assembled 10-21; v1 frozen. Blades on (10-26). The user's hand finishing (77 features in whole-v2, 5 in righting) generated since 10-28 (`cad.finish`), the rollcage rear given the front's; the bike's colours; caps and keep-outs off by default; README picture `docs/media/cad_bike_v2.png` | Lighter rollcage caps; the sheet (radius 66.2 GUESS, not modelled). The hand features are still in both tabs, under the rollback bar, for the user to delete; five one-sided case-side / blade edits left unmirrored (touch-ups doc, round 3) | `aow-bike-whole-v2-touchups.md`, `aow-bike-whole-v2-complete.md`, `cad-onshape-workflow.md` |
| **Self-righting** | V2 linkage with the V3 blade is the default (10-14): level at 116.98 deg, lying at 85 deg in the sim. **On the bike (10-26)**: the mirror's PageDown / PageUp (pad X / Start) lie it down and get it up on the Pi, `righting_current` 785. Get-up stops PROPPED, drives limp; Enter hands it to the policy -- flown once, the pointer, engaged at 1.9 deg roll (run 154338), across a policy swap: v4 had laid it down at the end of the run before. Lie-down works from a policy riding (pointer and v4, 10-26) or from resting on a blade. The lower end is 25 deg past centre (`LIE_REST_DEG`): stopping at centre, the crank back-drove and the bike dropped. Crank 120 deg/s (run 261026-042118, 5 cycles): down 2.1 s, up 1.25 s, with a pause on the blade going down. The lower now ramps in roll along the bike's measured map (`LIE_MAP`, 60 deg/s): flown 10-26 (run 261026-173521, left x3), 1.44 s from a prop, at most 0.6-2.1 deg ahead of the map (was 5.4-6.7), peak 107-124 deg/s (was 166-179), ~2 deg onto the cage (was ~5). A get-up against a wall (same run) stopped at crank -64, was propped by the stall rule and held at 700 mA, and tripped the righting servo's OVERLOAD in 1.2 s; the sequence then retried a dead crank for a minute. Now: a crank over 20 deg behind its ramp that moves under 3 deg in 0.3 s is BLOCKED and lowers back (a first, lag-only rule -- 35 deg for 0.2 s -- sent 9 of 13 get-ups back down with the BATTERY on, run 261026-175020: the crank then lags 35-58 deg at 700-757 mA, near the 785 cap, and still lifts; the replayed new rule fires on none of them and catches the wall in 0.3 s); a prop needs the crank within 20 deg of the lean; a failed lift lowers rather than steps; no auto-retry on the bike; a righting fault ends the sequence and refuses the next. **Not yet flown**: those, and the steer eased straight (nearer of 0 / 180) from the start, lying included, and held through `propped` WITH THE BATTERY ON the lift is at the servo's limit (run 261026-180445, new rule): 5 of 9 get-ups stalled at crank 12-23 deg, roll 63-65 -- where the stepthrough's V3 torque peaks (crank 14-27) -- with PWM pinned at 885 (100 %) for 55-75 % of every lift, ~690 mA at 11.8 V, so Goal Current is not the limit and raising it would not help. Failures came at servo temperatures of 41-42 C, successes mostly 36-38; the steer was not involved. The blocked rule caught each and lowered back; no overload. Runs 261026-184729 and -190044 (battery on; roll + = lying on the RIGHT, confirmed by the user right-then-left): the LEFT side fails, the right mostly lifts -- yoga mat right 6/6, left 0/2; marmoleum right 3/8 then 4/4, left 0/1 then 0/3; servo 27-36 C, so not heat. Successes pass crank 0 at 94-115 deg/s, failures at 69-91: loaded before the hump. At a stall the crank is dead (0-3 deg/s, 100 % PWM, ~700 mA) while the bike coasts up 8-15 deg on the blade's curved face and falls back. A ROCK (back to a 50 deg windup, up again at 240 deg/s) got 0 of 18 up and was taken out: a failed lift lowers, goes limp and waits for the next press. Blocked now waits 0.5 s (was 0.3), restarted only by a new best roll: run -191739 (0.5 s, any rise restarting it) got one left lift up, then rocked on the wing 62-76 deg at ~1.25 Hz for 2.3 s with the crank dead at 650 mA, which a rise-restarts rule let run to the 3 s timeout. Over all battery lifts (lift phase only) a failure matches a success to crank 0 -- current, crank speed, roll -- and stalls at crank 10-20 for ~40 mA (~6 %) more load; the LEFT draws 15-25 mA (~3 %) more than the right at every crank, and lifted 4 of 22 against 21 of 36. Riding, the policy holds +0.4 to +0.6 deg roll (11-03, run 261103-130944: +0.52 with the turning lean regressed out), and the AHRS has no roll bias to speak of (a phone level app, i.e. the phone's IMU, on the right drive pulley's face read 6.2 deg with the bike's roll at 6.2; 11-03, user, one reading. Repeated at bike roll 0, propped on a wing, phone held the same way: left pulley face 0 to -1.0, right 0 to +1.0 -- the two faces splay ~1 deg, so the first check carried ~0.5 of face error, and their mean puts the AHRS zero within ~0.5. Propped at roll 0 on the RIGHT wing the bike wants to fall LEFT; on the left wing it is stable. Found by hand, steer straight, the balance roll reads positive with the steer near 0 and near 180 alike (user; hard to pin down exactly). Both fit a physical lean AND an AHRS reading high: at a reading of 0 a bike whose AHRS reads 0.7 high is truly leaning 0.7 left. THE SIM SAYS AN AHRS OFFSET IS ENOUGH (scratch, v4, 3 seeds, the accelerometer's mounting tilt set to 1 deg about x, which tm151_filter settles onto): reading 1 deg HIGH, the true lean barely moves (+0.14..+0.27, against +0.34..+0.37 with none), the policy reads +1.15..+1.22, holds the steer +13 standing and +4.6 at 0.4 m/s, and crabs 3.5 deg left -- the bike's pattern; 1 deg low flips steer and crab. And the string hang (from the ring apexes on the centre marks, +0.3 and +1.2 deg turned round, user 11-03) solved with the riding reading: CoM ~0 (+-0.6 mm), AHRS ~0.7 high (+-0.3). The notch hang agrees with neither reading by more than its +-0.5 mm (the notches are hand-finished). The policies trained on a misalignment drawn within +-0.5 deg per axis. Deciding test: the phone on each drive pulley face with the bike held still, each face read twice with the phone turned end over end -- the faces' ~1 deg splay and the phone's own zero both cancel) -- but hung from both sides the CoM is centred to +-0.5 mm (11-03): the two hang angles differ by 0.15 deg where each mm sideways moves them 0.45 deg apart each way, so 0.17 mm left (the +-0.5 is the notches' own placement). A physical 0.52 deg lean would need ~1.0 mm (an earlier reading here, that no AHRS bias squared the two, assumed the lean was physical; it was not -- see the trim below). Not the wheel play either (it recentres, user). It is fixed: every run since 10-28 (18, five policies) leans right, standing and moving, medians +0.3 to +1.9 deg, and every one holds the steer LEFT, +2 to +12. Steer moves the balance: hand-balanced in standby with the steer set (run 261103-152812, user) the bike balances at -0.5..-0.8 deg roll at steer -51, +1.0..+1.5 at -23, +2.1..+2.4 at +32, +3.7..+4.0 at +56, ~0.04 deg per deg; the model, the same way at a third of the slope (-0.46, +0.09, +0.36 at 0, +0.70, +1.19). At steer 0 the bike's hand balance sits ~1 deg right of the model's (by feel, so rough). Riding straight forward (yaw rate under 0.05 rad/s, 9 runs), the steer is +0.7 to +4.8 and the diff mostly drives the rear contact LEFT (up to 37 mm/s): the estimator reads a 0.7-4.5 deg crab to the left in every run. Steer and diff agree on it, and it is not the steer's zero: lined up straight by hand the steer reads ~+0.5 (user, 11-03, the mirror's new `steer` row), so the wheel really is turned ~0.2-4.3 deg left. THE CRAB FOLLOWS THE LEAN, in the sim on true velocities (scratch, cmd_replay's env, v4 and v6 s1, 3 seeds, a held straight command): on today's model (CoM 0.6 mm left) both lean +0.3..+0.5 standing and moving, and at 0.4 m/s steer +0.6..+3.1 and crab 0.5-2.4 deg left (~0 at 0.7); with the battery moved to put the CoM 1.8 mm left, lean +0.9..+1.2, standing steer +11..+16, crab +3.5..+4.5; 0.6 mm right and 1.8 mm right flip all three. So the policy crabs left because the bike balances right, and the bike's numbers (lean +0.5..+1.0, standing steer +2..+12, crab 0.7-4.5) are the sim's at an effective ~1-2 mm left -- where the hang puts the CoM at 0.17. The wheels, measured by the user (11-03): the rear is aligned, the FRONT ~1.25 mm off the midplane (1-1.5; likely the original toy hub; to fix when the steering assembly is reprinted). In the model's static balance a front contact 1.25 mm toward the bike's RIGHT moves the balance roll +0.40 -> +0.83 deg (toward the bike's left: -0.03), and which side it is on flips with the steer's pose (0 or 180, user). TESTED AT REST (11-03, v4, standstill, ~30 s engaged each): wheel left (pose 180, run 261103-165056) roll +0.75, wheel right (pose 0, -165626) +0.67, 5 s medians steady to ~0.1, steer +6.7 / +5.0 -- 0.08 deg apart where the model's 1.25 mm predicts 0.86. So flipping the wheel does not move the lean at rest, and the ~+0.7 right reading is not the wheel. The older records' split, which is riding and across starts, had read: split by pose, every run with both leans further right near 180 than near 0 (7 runs, by 0.06-1.0 deg, median ~0.5; pooled: near 0 +0.07..+0.99, near 180 +0.58..+1.90). The model predicts 0.86 between the poses for 1.25 mm, so the bike reads as ~0.7 mm, or as the offset not all at the contact. The mean of the two poses, ~+0.65, is still ~0.25 right of the model's +0.40 (its CoM 0.6 mm left; the hang's 0.17 would give ~+0.1): small, and within one bubble-level reading of an AHRS bias. The CoM is not the left side's weakness either: at the sim's ~10 mA per mm, +-0.5 mm is +-5 mA of the 15-25. Lying at the sequence's crank (+-24.6) the bike rests at -73.5 / +74.0 deg, symmetric to ~0.5 deg over 10-26..10-27, so neither the crank's zero nor the resting geometry is off by side. The cause is open: what acts only during a lift (each side's linkage pins, each blade on the floor) is untested by side. Pack 12.2 V early, 11.3 V by -191739. PUMPING (SwingSit `pump`, run_bike PUMP_S) -- full push while the bike swings up on its wing, limp while it swings back -- is BUILT AND OFF: flown 10-26 (run 261026-195319, power brick, Goal Current turned down to 485 / 565 to make it stall), 0 of 2 up, and the second tripped the righting servo's OVERLOAD (5.8 s above 400 mA; the fault path ended the sequence and cut, as it should). One attempt per press, then down. Sim, Goal Current cut until a held lift stalls (46-48 %): 4 of 6 pumped lifts propped, 0 of 6 held; 42-44 %: none. The sim's evidence is weak: its stalled bike barely rocks (+-0.4 deg, not the bike's 62-76), its limp crank is flung at up to 1400 deg/s (past the XC330's ~680 no-load; the bike's gearbox back-drove 25 deg in 0.3 s), and its props then tipped over the top, the sim's prop problem (it reaches crank 111; the bike 100-102). The wing-floor friction, then a guess, is measured since 11-03: 0.10 (below). [ / ] now set the swing-sit's current (they were ignored from a lie-down to the next riding), so a get-up on the power brick, which does not stall, can be made to by turning the current down. The stepthrough's quasi-static V3 says a CoM 5-10 mm toward the side lying down needs 0.41-0.44 N m against 0.39, and a HIGHER CoM more (+10 mm: 0.48, +20: 0.59; -10: 0.35) -- trends only, and on the 1157 g model; the weighed model is ~0.2 N m light (below) | The lift's torque margin with the battery, and the left side's (a CoM offset toward it, by the model); pumping sidelined, code and sim analysis kept; fly the rest; the left side's prop at 15-17 deg, outside Enter's window: the crank stalls 13 deg short of its goal drawing ~405 mA of 785, so the position loop's gain stops it, not the current (run 261026-175020: 29.6 s propped at -13.0 deg, crank 10.5 short at 0.33 A, no error). Built 10-27, NOT FLOWN: Enter on a prop settled outside the window and leaning onto its blade NUDGES the crank goal out at 5 deg/s (never past the full stroke), engages 0.5 deg inside the window, else goes back to the lean and says so. Flown 10-27 (run 261027-135627, left x3): goal at the stroke in 1.4 s, crank moved under 2 deg at 0.48-0.51 A of 0.785 (position-loop droop), best roll 11.9 -- it does not get there against 12. Back ON (10-27, 10 deg/s, to 14.5): with the crank let go the left get-ups settle at -15.3 to -15.7 (crank back-driven ~3 deg, run 261027-143131) against -12.3 to -13.0 held, so Enter re-powers it to take that back; flown (run 261027-144242), it does almost nothing: 100 % PWM, crank stalled at -96.9 from a standstill -- OFF again (`NUDGE`), never having got a prop inside the window. The get-up's lift now ramps its goal to the full stroke (116.9) instead of the 110 lean: on the left it ran at 100 % PWM to crank 90 and then eased to 577-695 of 885 as the crank closed on 110, stalling at 96-99 with roll 14-16. Flown (run 261027-145541): the left now props at ~12; the RIGHT reached its prop at -67 to -97 deg/s and went over twice in four (at 110: -21 / -35, stayed), so the target is per side (`LIFT_TO_DEG`: left the stroke, right 110) -- not yet flown. Instead Enter takes a prop out to 15 deg (`PROP_ENGAGE_DEG`; the fall re-arm stays 12): props measured by hand, crank unpowered, left +1 at crank -116 and -13.6 at -96.9, right +4.8 at +109 and +14.8 at +94 -- untested past 12. In standby the righting servo now starts UNPOWERED until first used; Enter on a bike set on its blade by hand (crank past 45 deg, leaning onto it) is taken as a prop, so it gets the same 15 deg and kickstand. A prop now LETS THE CRANK GO (Goal Current to 0 over 1 s, `PROP_CURRENT`; the linkage holds unpowered, per the user), and the mirror's HUD shows roll and crank, to find the props the policy can take; and PARK (the mirror's Home / fn+left, the station's `p`; command v5 adds `park_n`), a lie-down that stops propped. Both sides alike. `python -m aow_sim.mirror_replay <run>` re-renders the mirror from a record for video; the live mirror takes `--heading-offset DEG` (drawn world only) and shows the bike's heading in its HUD. **Manual crank limits (11-03, `control/crank_lead.py`, one code path for the station, run_bike and teleop)**: 9/4 and the triggers keep the goal within 40 deg of the crank (`--righting-lead-deg`; d-pad right steps 30 / 45 / 60 / off), and a crank 5+ deg short of its goal that makes no progress for 0.75 s drops to Goal Current 200 until the goal moves. The two manual overloads of 11-02 (runs 261102-204438 / -231204: stalled at roll 62-64 deg, PWM 885 for 1.7 / 2.3 s) replay as held 0.8 / 1.8 s before them, with no hold on a lift that got up; a prop held 10 deg short of the stroke end (0.31-0.34 A for 9-14 s) now drops to the hold too. Flown on the bike (run 261103-114128): 24 holds at 201 mA, no overload, but 10 held a failed left lift part-way up (roll 35-51 deg, one 27.8 s). So every hold now ramps on to 0 over 1 s (`LET_GO_S`; `LET_GO_ROLL_DEG` 0, a threshold keeps holds starting under it at 200): a failed lift lowers, a prop sits unpowered as the user's manual props and the sequence's do. First flown with a 20 deg split (run 261103-120130): the ramp tracked exactly, but the station's 40 deg lead dragged the goal after the falling crank a link's lag behind, the gap closed 55 -> 40 deg, that read as the operator backing off and put 785 back mid-fall -- the lift pumped back up; an unpowered crank swung by a fall over the top read as progress the same way. Now, letting go, nothing ends a hold until the crank has been still 0.3 s (`SETTLE_S`), and then only the goal: further into the lift, or the gap closed 3 deg from its widest. Replayed, both are held through. Flown (run 261103-121011, left only): 16 holds, 10 props sat unpowered 1.8-8.3 s, 3 failed lifts lowered and stayed down, no error; against the 200 mA run, left lifts up 10 of 17 (was 1 of 13, a crude count), mean current 109 mA (214), servo warming 1.8 C/min (4.1) -- one run each, technique not held fixed. **A learned get-up (11-03, `righting_rl_v3`, SB3, 32 envs, 9 inputs mirrored onto the lying side, crank goal + Goal Current; not flown anywhere)**: in sim it props 36 of 48 from rest and from mid-fall takeovers at 60 deg (servo strength 1.0-0.7: all; 0.6: overload) where the scripted lift props 11 and goes over 23. The design is a switch, not a blend: the drive policy lets go past 60 deg, this props, the drive policy takes the prop. **The module on the bench (11-03)**: the servo pushes the same both ways (so the left side's weak lifts are not the servo); PWM pins at 100 % at 756 mA, under the 785 cap, at 11.8 V; Present PWM in mode 5 is NOT the duty (the bike's records: ~2x the model at stall, half its rise with speed) -- nothing should key on it; OVERLOAD is current and time, t = 0.645 A s / (I - 0.413 A), all recovered within 1 s below it, and replayed over all 77 bike runs that law predicts all 7 real overloads 0.04-0.14 s early. No station key takes the righting servo's torque off. **Ballast (11-03)**: `righting.module.ballast`, lead under the crank servo's case, in the model at 0 g (not fitted: no geom, digest unchanged). Sim (scratch, on the old 1157 g model; to re-run): 30 g there lowers the CoM enough to help both get-ups (least Goal Current 580 -> 560, slow-ramp peak 0.498 -> 0.473 N m; 30 g on top: 0.515); each mm of lateral CoM moves ~10 mA between the sides' lifts (sim's own centred gap 7 mA, +-20-40 per crank angle), but the hung CoM is centred (above), so that is not the left side's excess. **Wing-floor friction (11-03, user)**: a pull scale on the bike lying on its side on the marmoleum, ~100 g to slide it; lying, the sim carries all its weight on the wing and the rear ring (no wheel), so mu ~0.10, now `sim.contact_parts.righting` for the blades and rings (they were the tire's 0.52). It turned at very little torque; torsional stays the global GUESS. In the sim it moves the lift's current under 10 mA. **What the righting sim lacks (re-run 11-03 on the ring cage and the weighed bike)**: (1) the left side's weakness: the sim's two sides lift within 3 mA. Ruled out: the CoM (above), the servo (bench), the resting geometry, the loaded friction (below, equal by side where the left stalls), and the unloaded linkage friction, which runs the OTHER way -- every one of the 12 sweeps of 10-17 (`righting_sweep.py summary`; crank + is the right-side lift) reads the right half of the stroke 15-20 mA stiffer at crank 15-45 and 15-28 mA at 75-105. Loaded pins and each blade on the floor are still untested by side. (2) ~0.2 N m of lift torque: the bike's current gives 0.50-0.53 N m at crank 10-40; the stepthrough's quasi-static on the weighed model 0.27-0.32 (0.33-0.37 on the 1157 g one: the gap grew from 0.15 to 0.21-0.24, the model having lost 154 g the bike never had), its slow sim ramp 0.32-0.37; the sim flying the bike's own get-up (120 deg/s, 785 mA, 11.6 V) draws 320-375 mA at crank 0-40 against the bike's 600-660, so the lift's speed is not it. It is FRICTION UNDER LOAD, by the bike's own lifts against its lowers at the same crank (moving > 20 deg/s; runs 10-26..11-03, 197 lifts, 93 lowers; scratch): half their sum, the gravity, 315-372 mA at crank 0-40 against the sim's 303-324 flying the same sequences, but half their difference, the friction, 276-296 mA against the sim's 13-55 -- the bike LOWERS on 38-90 mA where the sim holds 265-293. ~0.2 N m, the whole gap, and of a piece with the linkage holding a prop unpowered and the sim's limp crank flung at 1400 deg/s. The lifts run at full PWM, where current also follows speed, so this is an estimate; the model's linkage friction is the unloaded sweep's, and its gearbox passes 0.15 x load running. FITTED 11-04: the XC330 now runs on ROBOTIS's performance graph (stall 0.80 -> 0.559, current law 0.83 (I - 0.0165) -> 0.736 (I - 0.0444), the steer's kp / kv / armature rescaled with it; bike_params' comments carry the voltage reasoning). Flying the bike's own get-up the sim now PROPS both sides (crank 98.7 / -103.5, roll 11.9 / -8.9; the bike 100-102, ~12), drawing 384-448 mA at crank 0-40 (bike 600-660): the remaining gap ~0.1 N m. What led to it: the model's k (|I| - I0), 0.83 N m/A, was fitted at 35-129 mA and is extrapolated to the lift's 600-660 mA, where it gives the 0.50-0.53. ROBOTIS's own XC330-T181 performance graph (`docs/robotis/`, "closer to real-world performance than the rated stall torque"; voltage not stated) puts 0.65 A at ~0.37 N m and ~17 rpm, and the bike lifts at 100 % PWM, 100-114 deg/s (17-19 rpm) and 628-652 mA: on the graph's curve, at ~0.36-0.38 N m. Against the sim's quasi-static 0.27-0.32 and slow ramp 0.32-0.37 that leaves ~0-0.05, not 0.2. The graph's motor is also much weaker at speed than the model's line (stall 0.80, 113 rpm: ~0.59 N m at the output at 17 rpm against the graph's ~0.37), which fits the sim's lift going over the top. The lift/lower split read in mA assumes one law through zero and for braking, which the servo model's own notes doubt. The steer servo shares the `xc330_t181` block. By side: the friction agrees to ~10 mA at crank 0-40 (left +29-48 at 50-80), and the left's extra ~10 mA sits in the gravity half. (3) The cage in the lift's first 25 deg: lying, the sim rests at 74.7-75.8 deg (the bike 73.5-74.0; the capsule had 86) but on the wing (7.9 N) and the rear ring (1.9 N), where the bike's cage holds it (its crank back-drives to -27 unpowered and the roll does not move); through crank 0 the sim rolls on to 83 on its blade (the bike 75-77), and held at crank 0 / -10 it lies on the blade alone at ~80 where `LIE_MAP` has the bike at 69-73 on its cage. Not the rings: the user's calliper reads 137.6 mm across (68.8 radius, the model's 68.7; the hand finishing was rounding and more support at the frame). It is the blade against the crank: from crank -10 to 20, where the bike rests on its blade, the model's quasi-static lies 5-6 deg further over than `LIE_MAP` (crank 0: 74.2 vs 69.2; 20: 59.6 vs 54.1), within ~1-6 from 40 up. Cause untested (the blade as printed against `blade_v3.yaml`, the loaded friction above). The chainstays have NO collision geometry in the model (11-04, scratch): at the lying crank (-25) the sim's rest puts each chainstay's mass box 1.9 mm below the floor, clear by 2-4 mm from crank 0 up, so it cannot be the crank 0-20 mismatch or the lift's friction, only the lying rest. (The model's `input_*_pulley` reads 3.5 mm under too, but it is a placeholder on the internal shaft; the real wheel pulley sits behind the chainstay.) The bike lies on both rings, the chainstay and the wing (user). The CAD's chainstay end (user, 11-04: coaxial with the rear axle, dia 19 from 35.2 to 38.2 mm out, dia 15 at 40.2, swept 51.3 mm forward-up at 45 deg) as collision in scratch sits 1.7 mm clear of the floor at the sequence's crank -25 (the wing still holds it, 74.8 deg) and 6-9 mm at crank 0; the mass box, which reaches past the part, is what read 1.9 mm under. In the plant since 11-04 (`bike.chainstay_contact`: one convex mesh a side, the profile swept along the arm, at the rings' mu 0.10; drawn as stubs from the wheel; colliding only in the jointed build). With the wing pulled further in (crank -40 to -90 on either side) the sim lies on the chainstay and both rings at 73.7 deg (chainstay 4.4 N, front ring 4.0, rear 1.4), the bike's 3-point rest (user) and its 73.5-74.0 at crank +-24.6 -- so the bike is on its chainstay where the model's wing still bears, ~1 deg and a few crank degrees apart. Lying still, the bike's AHRS pitch reads -2.0..-3.5 deg (nose up) on its right side and -0.9..+0.8 on its left, the sim's 0.0 on both: few samples, and half of it could be the mount's yaw. (4) The wing hub as one point at its 10-07 centroid. (5) The sim's get-up goes over the top on both sides (crank 112); the bike props. With the measured mu the scripted ramp props seed 0 at no servo strength on both sides (0.6 props the left only, 0.62-1.0 go over, 0.55 and below overload; on the 47f14be2 plant it propped at 0.6-0.65, and at 0.7 before the weighing), so its weak case is registered red. `righting_rl_v3` trained on the old mu and mass and is provisional. | `righting-policy-integration.md`, `righting-linkage-margin.md`, `righting-servo-model.md`, `wishlist.md` |
| **Web demo** | Built 10-23 (`src/aow_sim/web/`): the pointer in MuJoCo's WASM build, keyboard and touch teleop, shove, auto-up and lie down / get up, a ball. `check.mjs` replays a Python trace through the JS. ~4 MB, 60 fps. 10-26: `a40_v4_80M` with its sawtooth detent, wheel play and counts steer rate; lie down / get up as `run_bike` runs them, except it hands off from the prop at once (the sim does not stay propped). Not yet deployed; `scripts/site.sh` builds a static site around it at `/aow/sim/` (10-26) | Hosting; phones not tried | `src/aow_sim/web/README.md`, `web-demo-divergences.md` |
| **Mini bike** | Opened 11-04: the same wheelbase and wheels on XL330s. Stage A built and flown in teleop: `config/variants/mini.yaml` (M288 drives, 32:21 step-up) and `mini_m077.yaml` (M077, the pair flipped), flown with `run_drive --teleop --params`; each servo's factory velocity gains (the M077 chatters at the XC430's). The mini's lighter parts in the old places: 748 g, CoM 100.2 mm up (today 1003 g, 108.4). `mini.yaml` owns its `frame_parts` and `payload` (`!replace`), seeded as those parts in today's places, for the layout work; `analysis/mechanism/mass_table.py` lists them. A variant is the base plus only the keys it changes (`params._read_bike`); the built bike's digest did not move | Every XL330 loop and friction value is a GUESS from the XC430's; stage B (steer, righting) not started | `mini-bike.md` |
| **Contact bench** | A fallback, parked: drop rig built and run (10-03) with its MuJoCo twin; the wave cams printed and roughly work (10-21) | -- | `drop-release-rig.md`, `contact-measurements.yaml` |

---

## Health

**Tests, 2026-10-31**: `pytest -n 10 --dist loadgroup`, 1162 passed, 58
skipped, 1 red then: `test_model.py::test_rest_stability`, since split (11-02,
`-m "contact or geometry or deploy"` 193 passed, nothing red): the bike's
joints keep 0.05 rad/s at rest; the rollers, ring and play joints get 1.0 of
their own -- the rear axle's play makes the roller on the floor chatter at
~277 Hz (0.53 rad/s; 0.009 without play), shrinking with the timestep, so the
solver's, not the bike's. Re-pointed, not loosened, for
the plant change: the digest tripwire (two log lines), the contact-parts
no-op test (now "a part without an override gets the globals"), DOF counts
(full 25 -> 31, testbed 12 -> 15), the wheel-play tests' "default", the floor
rig's wheelie (pinned mu 0.9: at 0.52 the wheels spin), two righting-module
env tests (`wheel_play: False`: with play the unpowered bike tips at 0.46 s).
The 15 registered reds are the deprecated LQR, judged only under `pytest -m
lqr`. **11-03**: 1182 passed, 1 red, registered: the scripted get-up's weak
case (`test_righting_rl.py`), which no strength props once the blades slide at
mu 0.10 (Self-righting, (5)). The contact-parts bitwise test now leaves the
friction overrides out of its no-priority twin, which takes the larger friction. **11-04**: 1197 passed, 2 red, both registered: v4 turns just short of two teleop bars on the graph-fitted steer (LB snap 44.7 deg > 45; the lead clamp, which already passed or not run to run). The scripted get-up's weak case props both sides again at 0.7 (re-pointed from 0.65, which overloads) and left the registry. The drivetrain tests pin the detent's options on a plain bump of their own (`PLAIN_BUMP`) instead of the default, and the roller to the film (damping ratio 0.45-0.75, overshoot 2-20 %, was critical / none); the rigid-module test sees the chainstay contact geoms in both builds, colliding only jointed. The Warp parity checks run in a venv with Warp 1.18 / MuJoCo 3.10
(CPU device), not under pytest (`analysis/gpu/phase2/check_*.py`,
`analysis/gpu/phase3/check_contact_parts.py`).

**Digests**: `plant_digest` **3631d750142a6d60** (11-04: the drivetrain
roll-in, c084bc8e, the chainstay contact, d3cbdf0a, then the XC330 fitted to
its performance graph; before them d6e4355c, the
blades' and rings' floor friction 0.10, measured; 47f14be2, the bike as
weighed, and 46c263de, 10-31), `design_digest` 5fe8c19379988031;
`deploy/bundle.npz` re-exported against both (LQR worst fit R^2 0.962), so
**the Pi needs `scripts/bike.sh sync`** before `run_bike` starts.
`config/ahrs_mount.yaml` is the CAD's 45 deg nose-down mount, plus since 11-03 `roll_trim_deg` 0.7 (estimate: the bike read ~0.7 deg roll high, Self-righting), in the bundle only -- the mount stays out of plant_digest. Flown at rest the same day (v4, run 261103-173716, 35 s): roll +0.07 (5 s medians -0.26..+0.26) and the standing steer -1.4 (IQR -4.0..+0.7), against +0.67..+0.75 and +5.0..+6.7 in the two runs before it -- the left steer is gone. Riding not yet flown with it. History:
`params-digest-split.md`.

**Which exports carry today's plant**: none. Every export is provisional
against d6e4355c: each trained on a bike 154 g heavier, with the 212 g roof
high up (v6, e4e2d636, was 46c263de + its detent overlay; mu 0.52 and the
fitted wheel contacts reach the older ones too). In the sim, v4 on the new
plant sits at the edge of two tests (a shove from standstill, the teleop
lead clamp), passing or not run to run. `righting_rl_v3` trained on the old
plant too. A move that records its own `wheel_play` (v4, v5) still flies
it; one that records none now gets `bike_params`' play. The old pointer
`fixes_72M` is older again (bc66ca20: old steer, old mass layout, exact
steer rate). Read a move's own `plant_digest`, never its name.
`run_bike` does not check a move's digest; the bundle's is what stops it.

**Env fixes of 10-14 split the history**: re-run `per_command.py` rather
than compare policies across that date.

**Open, known:**

- **`tm151_filter` is unmeasured under sustained acceleration** (turns,
  crab).
- **A MinGRU policy latches state at reset**: a bogus first observation can
  persist (`analysis/policy/mingru_memory.py`). Replay resets the hidden
  state per engage.
- **Not yet checked on the bike**: the mirror's front-wheel spin fix
  (10-21); the mirror's keep-awake and pad reconnect, and the record's
  clock checks (10-27); the coasting lateral drift at sharp steer
  (`odometry-sharp-steer.md`).

---

## What drives the bike

`control.general_move` = **`general_puffer_a40_v4_80M`** (10-26; was
`general_puffer_fixes_72M` from 10-16). It reaches the Pi with `bike.sh
sync`. Its plant options (detent overlay, its own 1.0 / 1.5 mm wheel play)
are in its move yaml, so teleop and replay rebuild them from there; fly it
on today's play with `--wheel-play config/bike_params.yaml`. The web demo flies it too (re-exported 10-26). A session
flies another with `run_bike --move NAME` (`bike.sh bench --torque --move
NAME`), which also sets that policy's drive firmware gains. Teleop flies the
pointer on `tm151_filter` by default (`--general NAME` for another); a
policy that trained on the detent or the play flies with them, from its
move yaml.

**`moves/` layout.** The top level holds the pointer, the GPU finals
(`fixes_52M`, `fixes_72M`, `seed1_72M`, `next_72M`, `rake20_*`, `a40_v4_*`),
`v3_s1`, `v3_mingru_s0`, the pointer before those, `smooth_temporal`, and the moves
teleop's keys and the tests name (`flick_rl`, `ball_rl`, `pivot_rl`,
`general_rl`). The rest are in `moves/archive/`, tracked, loading by bare
name (`moves-layout.md`).

**Do not compare `metrics:` blocks across `moves/*.yaml`**; re-run
`analysis/policy/per_command.py` (`CLAUDE.md`).

---

## The guesses

**8 `source: GUESS`, 27 `estimate`, 12 `model` in `config/bike_params.yaml`** (recounted 11-03, the sheet gone and the wing hub split; 10-31: `friction_sliding` measured; the wheel play's four centring stiffnesses and one damping GUESS, its two front travels `estimate`, its four wall time constants `model`). The CAD-volume masses are `estimate`, modelling choices (`crown_radius`, the ideal drive's losses) `model`; GUESS is left for unknown physics (contact `solimp` and friction, the roller centring spring). Never promote one
quietly: promoting is a physical parameter change (the `CLAUDE.md`
checklist).

| parameter | value | what is known | dies at |
|---|---|---|---|
| `contact_solimp` | MuJoCo stock | hand drops: the real contact is softer at first touch and stiffer at peak than any `solref` gives | the contact bench, if the bike needs it |
| `wheel_play.*.centring_stiffness`, `rear_axle_lateral.damping` | 4 N m/rad, 2000 N/m, 1 N s/m | nothing measured; with the rear play the roller on the floor chatters at rest, the solver's (`test_rest_stability` bounds it) | a pluck test of the play, if it matters |
| `friction_torsional` | 0.005 | nothing; the risk is setting it high | same |
| `drivetrain_model.roller_slop.centring_stiffness` | 1e-3 | ~1e-2 from the flick video (10-24), carried by the sawtooth overlay that v4 trained on, not yet in bike_params. By hand, a released roller "springs back, not every time". Stiffness x10 drops `smooth_temporal`'s score 0.818 -> 0.598; x0.1 or no slop moves it <= 0.005 (`analysis/policy/slop_sensitivity.py`) | -- |

**Estimates**: the printed parts' and the righting module's masses from CAD
volumes, scaled together by `bike.printed_mass_scale` (0.930) so the whole
bike weighs its measured 1003 g; the slicer's figures for the rings, blades
and wing stubs; the wing pin (CAD volume x steel); the bare 6-32s (0.8 g);
the cables' and the Pi's placement; the fork (and the front wheel as 86 g
less it).
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

**The contact as it ships (10-31):**

    contact_solref: [0.005, 1.0]           # the floor, the righting parts, anything else
    contact_parts:                         # the wheels' own, negative: (-stiffness, -damping)
      front_tire: {solref: [-44321, -249]} # 0.59 x critical
      roller:     {solref: [-44321, -115]} # 0.27 x critical
    contact_solimp: [0.9, 0.95, 0.001, 0.5, 2.0]   # MuJoCo stock, GUESS
    friction_sliding: 0.52                 # measured, the front tire sideways

- Stiffness is the shipped pair's (1/(dmax*0.005)^2), so the sink is
  unchanged: rear 0.148 mm at its own 5.8 N, front 0.049 mm at 4.2 N
  (`contact_calibration.py`). Damping from the pivot drops of the whole bike
  (`analysis/contact/pivot_drop.py`, `contact-measurements.yaml`
  `pivot_drop_2026_10_31`): the shipped damping could not bounce, the real
  wheels do, the rollers about twice the tire.
- **Not a stiffness in N/m** in either form: MuJoCo works per unit of the
  mass it compiles around each contact. Positive and negative pairs are the
  same contact (timeconst x dampratio sets the stiffness, timeconst alone the
  damping); the negative form keeps them apart and does not move with
  `solimp`'s dmax.
- Not measured: stiffness on the floor (a 200 Hz drop cannot see it; the
  floor itself does not give, user), `solimp`, the rollers' and the righting
  parts' grip.

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

1. **Only v6 has trained on a contact that bounces**, and it came out
   cautious (next #1); every earlier export trained on the shipped
   critically damped contact, and on the measured one v5 s1 falls 10/20.
   The old floor sweep's cliff (`[0.005, 0.30]` everywhere, survival 0.35)
   was a STIFFER contact; these keep the stiffness. Nothing trains over
   stiffness variation (`solref_frac` is not ported, and refuses a negative
   pair) (`floors-and-the-contact-model.md`).
2. **A policy can trade away a direction and the score cannot see it.** A
   third of the 09-10 sweep's runs drove backwards when told forward (12
   seeds). Every export since v3 drives both straight commands forward, but
   the eval score (`survive_rate x track_geo`) would rank a policy that
   drives backwards on a forward command above a good one; a directional
   gate is written up, not built (`eval-score-rewrite.md`).
3. **GPU policies train on MuJoCo Warp and are judged in C MuJoCo.** The two
   lay contacts slightly differently (0.7 mm / 0.24 deg apart after 100 ms
   open loop); the pointer flies fine in C on every check so far.
4. **The wings, blades and rollcage still use the floor's contact
   stiffness** (`[0.005, 1.0]`, unmeasured); their sliding mu is the
   measured 0.10 since 11-03 (`contact_parts.righting`). The bike lies on
   them.
5. **Drive torque is the datasheet's.** The drivetrain fit is in PWM duty,
   which becomes torque through the datasheet's 1.6 N m stall torque; nothing
   has measured that scale. Training varies actuator strength +-15 %
   (`actuator_frac`).
6. **Left/right asymmetry and crab**, flipping sign between policies: v4
   turns harder left and falls only there (test 11); crab is not trained.
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
| `physical-bike-tests.md` | the bike on the floor since 10-24: the shake hunt, wheel play, the tests in order |
| `next-retrain.md` | the v4 run: every model change since the pointer, and its result on the bike |
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
| `righting-policy-integration.md` | the learned get-up: design, steps to the bike, the B1/B2/B2b bench results |
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
