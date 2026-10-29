# Project status — 2026-10-26

**How to read this.** The navigation layer: what is true now, what to do next,
and where the detail lives. Rewritten, not appended to. **For the reasoning,
follow the link — it is not repeated here.**

Rewritten 2026-10-02, 10-16 and 10-26. On 10-26 the dated logs that had
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
   (optional, fitted to floor runs) wheel play. The contact is the
   least-measured part.
2. **RL drives, and it trains on a rented GPU**: MinGRU policies, MuJoCo
   Warp + PufferLib, 72-80M steps in 12 min (5090) to 89 min (A40). The
   pointer is `general_puffer_a40_v4_80M` since 10-26 (was
   `general_puffer_fixes_72M`, 10-16). The analytic LQR is DEPRECATED
   (10-09).
3. **The onboard path runs on the Pi**: 100 Hz loop, TM151, mirror,
   telemetry, recorder, fall/cut/re-arm, low-voltage cut, and the swing-sit
   lie-down / get-up from the mirror.
4. **The bike balances on the floor (10-24), and since 10-26 without the
   shake**: `general_puffer_a40_v4_80M`, trained with the drivetrain detent
   and the wheel play the floor runs fitted, cut the 9-10 Hz oscillation 4x
   in roll rate against the pointer flown minutes later
   (`physical-bike-tests.md`, test 9).
5. **The goal is a functioning bike with sample videos, in weeks.** Ranked
   against that; the showcase moves are in `wishlist.md`.

---

## What to do next

| # | do | why now | doc |
|---|---|---|---|
| 1 | **v4 is the pointer (10-26, "for now", user); fly it more** | On the bike (10-26, one run each, back to back): roll rate 7-12 Hz 4.0 deg/s against the pointer's 16.4, steer target and drive commands 8-12x down in the band; 2-7 Hz unchanged (13 deg/s, both). In the sim it matches the pointer on the grid and standstill. Not yet: `chatter.py`, the steer wobble at 1.2 m/s (rake20, the v2-plant run before it, wobbled 5.9 deg against 2.5), a second run for drift (odometry 0.85 m in 73 s against the pointer's 0.24 in 55 s). The wheel play stays out of `bike_params.yaml` until it is refitted (next #2). **10-28, steer shimmed and front axle tightened** (user; no parameter moved, test 10): at rest the shake stays gone and the odometry drifts 0.17 m in 131 s; the steer's friction about doubled (Coulomb 12-14 -> 28-31 mN m by datasheet kt), which is where the model's ~31 already sat, so no refit; reversing fast the steer swings slowly (25-57 deg peak to peak, policy-driven) over a 4.7 Hz wobble in yaw, roll and the rear's differential drive, and makes 60 % of its command; on the bike all three policies have that wobble reversing (yaw 48-59 at -0.5; fixes_72M / rake20 / v4 flown 10-28), v4 reverses slowest (47 % of the command) and fell least (1 against 3 and 3); the sim's shipped floor gives it only to v4, a soft grippy one (sink 0.75 mm) to all three and stops the older two falling -- the contact's compliance is the lead (P0 on the bike's wheels); not an odometry loop (it persists with the true velocity); 2 of 4 180 deg snaps fell, both on a slippery wood floor (fine on a grippier one, user) | `physical-bike-tests.md` (tests 9, 10), `next-retrain.md` |
| 2 | **Refit the wheel play for the stiffened bike; the other sim-bike gaps** | **10-28: the steer shim and axle nut took the old pointer's 9-10 Hz shake out** (fixes_72M at rest, roll rate 7-12 Hz 16.4 -> 5.2 deg/s, steer target 1.40 -> 0.15 deg; v4 4.0), so the play was the shake, and the front play v4 trained on (axle 1.0 mm, headset 1.5) is mostly gone from the bike; the user calls the stiffened bike much better both ways. Reversing, v4 asks for ~half the commanded speed (the wheel delivers it), more so as the bike moves more; the older two ask for 88-116 %. The play fit (fitted on the pointer and the MLP) overstates v4's 7-12 Hz roll rate 2x (7.6-8.1 deg/s, 3 seeds, against the bike's 4.0); the same plant without the play gives 3.2-3.3. Below 3 Hz the bike sways ~2x the sim; the steer servo's own shaft moves 2x the sim's at 7-12 Hz for a matching target. Reversing, the bike's rear-drive wobble is ~2x the sim's on the shipped floor, matched on a soft one or the old detent (test 10). Then drift measured with a camera (test 6) and contact on the target floor (test 7) | `physical-bike-tests.md` |
| 3 | **Queued plant changes**: `k_roller -2.4` with the `onboard.yaml` A/B swap reverted, in one change; the drivetrain Coulomb (0.15 kept; coast says 0.16-0.18) | Left out of v4 on purpose so old and new policies fly on one `onboard.yaml`; moves the plant digest | `physical-bike-tests.md` (item 8) |
| 4 | **Blades and rollcage**: the lighter caps; then the sim's lying rest | The bike lies down and gets up from the mirror (10-26). The sim puts the lying rest at crank ~77 deg past centre, the bike 26.5-28; the cage's contact (one capsule closed across the rings' opening) is the first suspect, untested. The stepthrough's quasi-static V3 is within 5 deg of the bike's measured roll-vs-crank from crank 40 up, runs up to 10 deg ahead of it from 0 to 35, and rests at 76-81 deg roll where the bike lies at 74. The SIM flying the bike's own sequence (v4, right and left): its roll-vs-crank matches the bike's from crank 105 down to ~15 (within ~3 deg), then rolls on to 86 where the bike stops at 74; its lying crank sags to -80 (bike -26); its lift draws ~470 mA peak where the bike draws ~590; it reaches crank 110-111 where the bike stalls at 100-102 (left 96-101), props at 5 deg roll (bike 12), and on the right then tips over the top while propped, which the bike never has. The bike's lift TORQUE (Present Current through the servo model's k (|I| - i0)) is on the stepthrough's torque chart: 0.50-0.53 N m from crank 10 to 40 with the battery, 0.44-0.46 without, where the quasi-static V3 (battery in the model) needs 0.33-0.37 -- the model is ~0.15 N m light, part of it the lift's own acceleration | `righting-design.md`, `aow-bike-whole-v2-touchups.md` |
| 5 | **The style bit: find the pivot** (not a priority) | Run 2 (mult, 10-18) turns but steers, 20-33 deg short on big turns | `wishlist.md`, `gpu-training.md` |

Not next, deliberately: a trained (RL) ball shot, the privileged critic,
changes to the odometry estimator (`odometry-sharp-steer.md`), the AHRS
fixture (parked 09-24), the contact bench (a fallback).

---

## The workstreams

| workstream | state | blocker | doc |
|---|---|---|---|
| **Simulation & model** | The v2 bike since 10-19, plant digest **6446d8fa** (10-21). The detailed drivetrain is the default plant; optional on top, off in `bike_params.yaml`: the sawtooth detent and measured roller spring (`traces/drivetrain/drivetrain_sawtooth_1024.yaml`) and `wheel_play:` (front axle 1.0 mm, headset 1.5, rear axle 2.5, 4 N m/rad; `traces/drivetrain/wheel_play_fit_1026.yaml`, fitted to floor runs 174349 / 225207), both carried by a policy's move yaml when it trained on them. With the play the sim reproduces the pointer's shake (15.1 against the bike's 16-22 deg/s) and the MLP's faster one | Contact and masses, unmeasured; the sway and steer-servo gaps (next #2) | `mujoco-modeling-decisions.md`, `drivetrain-model.md`, `physical-bike-tests.md` |
| **Control — RL** | **Pointer `general_puffer_a40_v4_80M`** (10-26, plant 54fdba72 = 6446d8fa + detent overlay + play, the Pi's steer rate, `tm151_filter` + odometry counts): 0/20 falls, 7/9 turns forward, 2/2 straight on its own plant, less standstill drift than the old pointer in the sim, the shake gone on the bike (next #1). Endurance minute (`-m prospective`, 18 x 60 s) passes; it flies the move's detent but not its play (the test does not read `wheel_play`). **v5 (10-29, not flown)**: `general_puffer_5090_v5floor_s{1..4}_{52,72,80}M`, the detent phase and diff rate observed, front play a third, the rear lateral slide, one floor per seed (s1 shipped, s2 soft, s3 slippery, s4 both); on their own floors all match v4 on the grid, s1 80M cleanest; needs `bike.sh sync` (20-wide obs). The old pointer `general_puffer_fixes_72M` (10-16, plant bc66ca20, before the v2 bike, exact-joint steer rate): same grid, endurance 0/18 falls. `general_puffer_rake20_{52,72,80}M` (10-23, v2 plant, no play): as good on the grid, wobbles at 1.2 m/s, shook like the pointer on the bike. Style bit runs 1-2 (10-17/18): no pivot yet. CPU arms (`general_rl_v3_*`, MLP and MinGRU actor) trained before 10-17's plant; `v3_s1` shakes at 11.2 Hz on the bike | Crab never trained (`v_lat_frac` 0) | `gpu-training.md`, `general-rl-improvements.md`, `next-retrain.md` |
| **GPU training** | Phases 1-3 done (10-14 to 10-16). `scripts/gpu.sh` runs the runbook; `SEEDS=N` runs N under MPS (4 on a 5090: 116k steps/s in total). Cards: 5090 12 min / $0.23 per 72M steps, 4090 17 min, A40 38 min at 4096 x 64 for the v2 plant; the v4 plant (play joints) ran 19.7 -> 16.0k steps/s on an A40, 80M in 89 min, ~$1.05 of pod (no 4090 / 5090 to be had 10-26). Runs are not bit-repeatable | Phase 4 (MLP / loss parity) and 5 (GPU vs CPU seeds) not started; per-world randomisation of the play not ported | `gpu-training.md` |
| **Sensor modelling** | Largely done. TM151 measured on the fixture: a complementary filter, tau 0.19 s at rest to ~1 s moving. `tm151_filter` is the training level since 10-09; unproven on sustained acceleration | The wave cams (sustained acceleration) | `sensor-workstream.md`, `ahrs-fixture.md` |
| **Control — LQR** | **Deprecated 10-09.** `lqr` tests skipped unless named. The code stays: the bundle's gain schedule is built on it | none | `lqr-baseline.md` |
| **Hardware / onboard** | Pi 3B+, tick jitter p99 < 1 ms with four servos. drive_a is servo 102 (right) in `onboard.yaml`, compensating `k_roller`'s sign until the swap is reverted with it. `run_bike` rides out up to 3 bad servo reads in a row. The preflight accepts the CAD-located AHRS mount (`source: design`, 10-25). The AHRS enumerates only on the ethernet-side USB sockets. `scripts/bike.sh` runs sync / bench / run / hold / mirror / status. **Link drops (10-27) were the Mac idle-sleeping on battery** (display off at 2 min, sleep 1 min later; pad input over raw USB is not user activity): every unexplained drop that afternoon matched pmset's log to the second. The mirror now holds `caffeinate -di` while it runs, reopens a pad that drops or is plugged in late, and reads a dropped pad as neutral. **The Pi's clock is wrong after a power-up** until timesyncd reaches a server (28.5 min slow 14 s after one boot, 10-27; ~10.6 min in run 261027-162626's `created`); each record now carries an SNTP check at start and close (`hw/clock.py`), and `mirror_replay --info` prints the corrected start | Chassis, pack, the SBC that ships | `pi-bench-bringup.md`, `untethered-setup.md` |
| **CAD** | v2 is the bike as built (`cad.bike_v2`, 10-19), printed and assembled 10-21; v1 frozen. Blades on (10-26). The user's hand finishing (77 features in whole-v2, 5 in righting) generated since 10-28 (`cad.finish`), the rollcage rear given the front's; the bike's colours; caps and keep-outs off by default; README picture `docs/media/cad_bike_v2.png` | Lighter rollcage caps; the sheet (radius 66.2 GUESS, not modelled). The hand features are still in both tabs, under the rollback bar, for the user to delete; five one-sided case-side / blade edits left unmirrored (touch-ups doc, round 3) | `aow-bike-whole-v2-touchups.md`, `aow-bike-whole-v2-complete.md`, `cad-onshape-workflow.md` |
| **Self-righting** | V2 linkage with the V3 blade is the default (10-14): level at 116.98 deg, lying at 85 deg in the sim. **On the bike (10-26)**: the mirror's PageDown / PageUp (pad X / Start) lie it down and get it up on the Pi, `righting_current` 785. Get-up stops PROPPED, drives limp; Enter hands it to the policy -- flown once, the pointer, engaged at 1.9 deg roll (run 154338), across a policy swap: v4 had laid it down at the end of the run before. Lie-down works from a policy riding (pointer and v4, 10-26) or from resting on a blade. The lower end is 25 deg past centre (`LIE_REST_DEG`): stopping at centre, the crank back-drove and the bike dropped. Crank 120 deg/s (run 261026-042118, 5 cycles): down 2.1 s, up 1.25 s, with a pause on the blade going down. The lower now ramps in roll along the bike's measured map (`LIE_MAP`, 60 deg/s): flown 10-26 (run 261026-173521, left x3), 1.44 s from a prop, at most 0.6-2.1 deg ahead of the map (was 5.4-6.7), peak 107-124 deg/s (was 166-179), ~2 deg onto the cage (was ~5). A get-up against a wall (same run) stopped at crank -64, was propped by the stall rule and held at 700 mA, and tripped the righting servo's OVERLOAD in 1.2 s; the sequence then retried a dead crank for a minute. Now: a crank over 20 deg behind its ramp that moves under 3 deg in 0.3 s is BLOCKED and lowers back (a first, lag-only rule -- 35 deg for 0.2 s -- sent 9 of 13 get-ups back down with the BATTERY on, run 261026-175020: the crank then lags 35-58 deg at 700-757 mA, near the 785 cap, and still lifts; the replayed new rule fires on none of them and catches the wall in 0.3 s); a prop needs the crank within 20 deg of the lean; a failed lift lowers rather than steps; no auto-retry on the bike; a righting fault ends the sequence and refuses the next. **Not yet flown**: those, and the steer eased straight (nearer of 0 / 180) from the start, lying included, and held through `propped` WITH THE BATTERY ON the lift is at the servo's limit (run 261026-180445, new rule): 5 of 9 get-ups stalled at crank 12-23 deg, roll 63-65 -- where the stepthrough's V3 torque peaks (crank 14-27) -- with PWM pinned at 885 (100 %) for 55-75 % of every lift, ~690 mA at 11.8 V, so Goal Current is not the limit and raising it would not help. Failures came at servo temperatures of 41-42 C, successes mostly 36-38; the steer was not involved. The blocked rule caught each and lowered back; no overload. Runs 261026-184729 and -190044 (battery on; roll + = lying on the RIGHT, confirmed by the user right-then-left): the LEFT side fails, the right mostly lifts -- yoga mat right 6/6, left 0/2; marmoleum right 3/8 then 4/4, left 0/1 then 0/3; servo 27-36 C, so not heat. Successes pass crank 0 at 94-115 deg/s, failures at 69-91: loaded before the hump. At a stall the crank is dead (0-3 deg/s, 100 % PWM, ~700 mA) while the bike coasts up 8-15 deg on the blade's curved face and falls back. A ROCK (back to a 50 deg windup, up again at 240 deg/s) got 0 of 18 up and was taken out: a failed lift lowers, goes limp and waits for the next press. Blocked now waits 0.5 s (was 0.3), restarted only by a new best roll: run -191739 (0.5 s, any rise restarting it) got one left lift up, then rocked on the wing 62-76 deg at ~1.25 Hz for 2.3 s with the crank dead at 650 mA, which a rise-restarts rule let run to the 3 s timeout. Over all battery lifts (lift phase only) a failure matches a success to crank 0 -- current, crank speed, roll -- and stalls at crank 10-20 for ~40 mA (~6 %) more load; the LEFT draws 15-25 mA (~3 %) more than the right at every crank, and lifted 4 of 22 against 21 of 36. Riding, the policy holds +0.4 to +0.6 deg roll: ~1 mm of CoM left, if the AHRS has no mount bias. Pack 12.2 V early, 11.3 V by -191739. PUMPING (SwingSit `pump`, run_bike PUMP_S) -- full push while the bike swings up on its wing, limp while it swings back -- is BUILT AND OFF: flown 10-26 (run 261026-195319, power brick, Goal Current turned down to 485 / 565 to make it stall), 0 of 2 up, and the second tripped the righting servo's OVERLOAD (5.8 s above 400 mA; the fault path ended the sequence and cut, as it should). One attempt per press, then down. Sim, Goal Current cut until a held lift stalls (46-48 %): 4 of 6 pumped lifts propped, 0 of 6 held; 42-44 %: none. The sim's evidence is weak: its stalled bike barely rocks (+-0.4 deg, not the bike's 62-76), its limp crank is flung at up to 1400 deg/s (past the XC330's ~680 no-load; the bike's gearbox back-drove 25 deg in 0.3 s), and its props then tipped over the top, the sim's prop problem (it reaches crank 111; the bike 100-102). The sim's wing-floor contact is probably much grippier than the bike's too (user, 10-26; a note, unmeasured, not being changed). [ / ] now set the swing-sit's current (they were ignored from a lie-down to the next riding), so a get-up on the power brick, which does not stall, can be made to by turning the current down. The stepthrough's quasi-static V3 says a CoM 5-10 mm toward the side lying down needs 0.41-0.44 N m against 0.39, and a HIGHER CoM more (+10 mm: 0.48, +20: 0.59; -10: 0.35) -- trends only, the model is ~0.15 N m light | The lift's torque margin with the battery, and the left side's (a CoM offset toward it, by the model); pumping sidelined, code and sim analysis kept; fly the rest; the left side's prop at 15-17 deg, outside Enter's window: the crank stalls 13 deg short of its goal drawing ~405 mA of 785, so the position loop's gain stops it, not the current (run 261026-175020: 29.6 s propped at -13.0 deg, crank 10.5 short at 0.33 A, no error). Built 10-27, NOT FLOWN: Enter on a prop settled outside the window and leaning onto its blade NUDGES the crank goal out at 5 deg/s (never past the full stroke), engages 0.5 deg inside the window, else goes back to the lean and says so. Flown 10-27 (run 261027-135627, left x3): goal at the stroke in 1.4 s, crank moved under 2 deg at 0.48-0.51 A of 0.785 (position-loop droop), best roll 11.9 -- it does not get there against 12. Back ON (10-27, 10 deg/s, to 14.5): with the crank let go the left get-ups settle at -15.3 to -15.7 (crank back-driven ~3 deg, run 261027-143131) against -12.3 to -13.0 held, so Enter re-powers it to take that back; flown (run 261027-144242), it does almost nothing: 100 % PWM, crank stalled at -96.9 from a standstill -- OFF again (`NUDGE`), never having got a prop inside the window. The get-up's lift now ramps its goal to the full stroke (116.9) instead of the 110 lean: on the left it ran at 100 % PWM to crank 90 and then eased to 577-695 of 885 as the crank closed on 110, stalling at 96-99 with roll 14-16. Flown (run 261027-145541): the left now props at ~12; the RIGHT reached its prop at -67 to -97 deg/s and went over twice in four (at 110: -21 / -35, stayed), so the target is per side (`LIFT_TO_DEG`: left the stroke, right 110) -- not yet flown. Instead Enter takes a prop out to 15 deg (`PROP_ENGAGE_DEG`; the fall re-arm stays 12): props measured by hand, crank unpowered, left +1 at crank -116 and -13.6 at -96.9, right +4.8 at +109 and +14.8 at +94 -- untested past 12. In standby the righting servo now starts UNPOWERED until first used; Enter on a bike set on its blade by hand (crank past 45 deg, leaning onto it) is taken as a prop, so it gets the same 15 deg and kickstand. A prop now LETS THE CRANK GO (Goal Current to 0 over 1 s, `PROP_CURRENT`; the linkage holds unpowered, per the user), and the mirror's HUD shows roll and crank, to find the props the policy can take; and PARK (the mirror's Home / fn+left, the station's `p`; command v5 adds `park_n`), a lie-down that stops propped. Both sides alike. `python -m aow_sim.mirror_replay <run>` re-renders the mirror from a record for video; the live mirror takes `--heading-offset DEG` (drawn world only) and shows the bike's heading in its HUD | `righting-linkage-margin.md`, `righting-servo-model.md`, `wishlist.md` |
| **Web demo** | Built 10-23 (`src/aow_sim/web/`): the pointer in MuJoCo's WASM build, keyboard and touch teleop, shove, auto-up and lie down / get up, a ball. `check.mjs` replays a Python trace through the JS. ~4 MB, 60 fps. 10-26: `a40_v4_80M` with its sawtooth detent, wheel play and counts steer rate; lie down / get up as `run_bike` runs them, except it hands off from the prop at once (the sim does not stay propped). Not yet deployed; `scripts/site.sh` builds a static site around it at `/aow/sim/` (10-26) | Hosting; phones not tried | `src/aow_sim/web/README.md`, `web-demo-divergences.md` |
| **Contact bench** | A fallback, parked: drop rig built and run (10-03) with its MuJoCo twin; the wave cams printed and roughly work (10-21) | -- | `drop-release-rig.md`, `contact-measurements.yaml` |

---

## Health

**Tests, 2026-10-26**: `pytest -n 10 --dist loadgroup` on the v4 pointer, 1105
passed, 56 skipped in 52 s; red set unchanged, **no accepted reds**. One
test read the pointer as "an export from before `steer_rate_encoder`" and
now names `general_puffer_fixes_72M` instead. The 15 registered reds are the deprecated
LQR and are judged only under `pytest -m lqr`. The Warp parity tests skip
here; they pass in `aow-sim-gpu` (Warp 1.18, CPU device), which has no
pytest -- run their functions directly (`analysis/gpu/phase2/check_*.py`).

**Digests**: `plant_digest` **6446d8fa5b925702** (10-21), `design_digest`
5fe8c19379988031; `deploy/bundle.npz` matches both, and older bundles are
kept as `deploy/bundle_plant-<digest>.npz`. Neither has moved since 10-21:
the detent overlay and the wheel play are options outside
`bike_params.yaml`. `config/ahrs_mount.yaml` is the CAD's 45 deg nose-down
mount (`source: design`). **To the Pi:** `scripts/bike.sh sync` carries the
params, the bundle and `moves/` (`deploy/` is gitignored; the rsync carries
it). History: `params-digest-split.md`.

**Which exports carry today's plant**: `general_puffer_rake20_{52,72,80}M`
(6446d8fa) exactly; `general_puffer_a40_v4_{52,72,80}M` on 54fdba72, which
is 6446d8fa plus the two options it records. Everything else is older and
provisional, the old pointer `fixes_72M` included (bc66ca20: old steer,
old mass layout, exact steer rate). Read a move's own `plant_digest`, never its name.
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
sync`. Its plant options (detent overlay, wheel play) are in its move yaml,
not `bike_params.yaml`, so teleop and replay rebuild them from there and
neither digest moved. The web demo flies it too (re-exported 10-26). A session
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

**5 `source: GUESS`, 23 `estimate`, 8 `model` in `config/bike_params.yaml`** (recounted 10-26). The CAD-volume masses are `estimate`, modelling choices (`crown_radius`, the ideal drive's losses) `model`; GUESS is left for unknown physics (contact `solimp` and friction, the roller centring spring, the rollcage sheet). Never promote one
quietly: promoting is a physical parameter change (the `CLAUDE.md`
checklist).

| parameter | value | what is known | dies at |
|---|---|---|---|
| `contact_solimp` | MuJoCo stock | hand drops: the real contact is softer at first touch and stiffer at peak than any `solref` gives | the contact bench, if the bike needs it |
| `friction_sliding` | 0.9 | nothing | P0b pull test, same |
| `friction_torsional` | 0.005 | nothing; the risk is setting it high | same |
| `drivetrain_model.roller_slop.centring_stiffness` | 1e-3 | ~1e-2 from the flick video (10-24), carried by the sawtooth overlay that v4 trained on, not yet in bike_params. By hand, a released roller "springs back, not every time". Stiffness x10 drops `smooth_temporal`'s score 0.818 -> 0.598; x0.1 or no slop moves it <= 0.005 (`analysis/policy/slop_sensitivity.py`) | -- |
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
