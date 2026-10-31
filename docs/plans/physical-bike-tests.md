# What to test now the bike balances

Started 2026-10-24. The assembled v2 bike balances on the floor under the
pointer policy, hands off, for as long as it was left (50 s on a zero
command, until stopped), and turns on the spot when asked. This is the list
of what that makes testable, in the order that unblocks the most. Contact
tests from here on are on the **whole bike on the target floor**, not on the
drop rig or the subsystem stations.

## Where it stands (2026-10-24)

| | |
|---|---|
| balance | 14-50 s engages, mostly ended by hand; it fell a few times during the turning sessions (17:23-17:32) and once on its tether, and not in the 50 s no-input engage |
| pivots | heading followed the station: commanded -23 -> 67 deg, flown -35 -> 68 deg |
| standstill | about 1 m of net travel in the 50 s no-input engage (by eye), i.e. ~0.02 m/s. The odometry's 0.07 m/s median \|v\| is mostly the shake, not drift |
| shake | a ~9 Hz oscillation in the steer target, both drive commands, roll and yaw rate; the sim has 5-25x less in that band (below). **Gone under the v4 retrain (10-26, test 9)**: roll rate 7-12 Hz 16.4 -> 4.0 deg/s against the pointer flown minutes later |
| policy | the pointer, `general_puffer_fixes_72M`: plant bc66ca20 (before the v2 bike), steer rate from the exact joint velocity (`steer_rate_encoder` absent = `joint`), while the Pi forms it from differenced counts |

**Fixed on the way, both config-only:**

- **Drive A/B roles swapped** (`config/onboard.yaml`): drive_a is the RIGHT
  servo, 102. Each servo forward alone, bike held on the floor: the left one
  slides the rear left, which is the opposite of the model's drive_a. The
  plant's error is the sign of `k_roller` (the wheel is handed through the
  roller bevel mesh; confirmed by flipping the bike over); recorded in
  `omni-wheel-measurements.yaml`. The plant fix waits for a retrain, below.
- **The bike's own U2D2** named in `onboard.yaml`; the AHRS needs an
  ethernet-side USB socket (`pi-bench-bringup.md`, end of section 7).

## Next (end of 10-24)

What 10-24 settled: the 8-9 Hz shake is the bike's, not the policy's (test
1), it needs the wheels on the floor (2b), no single parameter the model
has reproduces it (2), and the drivetrain is stiff from servo to roller
(filmed, test 3). The pieces measured since, none yet in the sim together:

**Items 1 and 2 DONE 10-24 (evening): neither makes the 9 Hz.** Model
options added (`drivetrain_model.py`, all off by default, bit-identical to
before on the testbed and the whole bike): `detent.hill_shape: sawtooth`
(+ `hill_flip_deg`), `detent.kinetic_fraction`, `detent.friction_minus_scale`,
`detent.hill_offset_nm`.
The best-effort detent, from the P 4000 map where it can say:

| `--set drivetrain_model.detent.` | value | from |
|---|---|---|
| `friction` | 0.12 N m | map, diff minus common: ~0.25 - ~0.115 duty, x 1.067 N m/duty |
| `width_deg` | 1000 | the map's friction is flat across the notch (a cos^2 that wide is) |
| `hill_shape`, `hill_nm` | sawtooth, 0.12 | map angle-only, +-0.10 duty |
| `kinetic_fraction` | 0.15 | **GUESS, fitted** to the filmed creep's jump sizes |

Filmed creep replayed (`replay_jumps.py`, 261024-221037), singles/doubles:

| | + | - |
|---|---|---|
| bike | 25 / 13 | 5 / 21 |
| model as is | 49 / 0 | 48 / 0 |
| flat friction + sawtooth, kinetic 1 | 48 / 0 | 45 / 0 (at every size tried) |
| best effort (table above) | 23 / 13 | 23 / 12 |
| + `friction_minus_scale` 1.3 | 23 / 13 | 22 / 12 |
| + `hill_offset_nm` 0.015 / 0.05 | 23 / 13 | 23 / 12, 24 / 12 |

So: flat static friction with kinetic = static never coasts into a
double; a kinetic fraction ~0.15-0.2 gives the bike's MIX with no
per-notch variation (the bike's doubles come in runs, not at fixed notches:
+ and - double at different places). THE HANDEDNESS IS NOT EXPLAINED. The
map's static asymmetry (out +0.198 / back -0.227 duty, so - needs ~0.015
N m more) reads the same as handed friction (`friction_minus_scale`) or a
sawtooth with unequal ends (`hill_offset_nm`, a constant angle-only push);
at the measured size neither moves the replay (1.3 / 0.015: one jump
changes), and only an unmeasured 1.45-1.6 scale pushes - toward doubles
(15/16 at 1.6, bike 5/21). An earlier draft of this note credited the 1.3
scale with the - side's mix; the symmetric sawtooth gives the same mix.
A map taken at another ring angle (rotate the wheel ~90 deg) would split
the two readings if the bias is gravity on an unbalanced ring or roller.
(10-29: unlikely. With the wheel unloaded in the air, gravity on the
ring or a roller is negligible. Loads through the wheel and axle are
the likelier source of any bias, and the loaded 10-25 map read close
to the hung one.)
The + side is chaotic in kinetic and hill: 0.2 vs 0.15 flips it between
mixed and all-singles. Centred on a bump friction instead, the sawtooth's
flip position gives handedness (friction 0.21, hill 0.04, flip 15-60
cycle-deg early: + singles, - doubles), but the map's friction is flat.

Floor twin, 7-12 Hz roll rate in deg/s (bike 24.5 on 191558, 21.9 on
174349), two seeds each where shown:

| variant | 191558 (rake20) | 174349 (fixes_72M) |
|---|---|---|
| model as is | 4.1, 3.7 | 3.2, 3.4 |
| roller centring 0.01 / 5e-5 | 3.6 | 3.5 |
| roller centring 0.007, 0.011 | 3.8, 3.7 | 3.6, 3.5 |
| best-effort detent (with the 1.3 scale) | 4.7 | 4.9 |
| both | 4.4, 4.5 | 4.9, 4.7 |
| both, kinetic 1 | 5.3 | 5.0 |

`traces/bike/<run>/spectra_sawtooth.png`: the bike's roll rate and drive
commands have a resonance at 8-10 Hz, 50-100x the sim's power there; the
sim has no peak there in any variant, the detent only lifts a 6-8 Hz
shoulder ~2x. The measured roller spring does nothing visible. Contact
(item 3) is now the lead, with these in.

1. ~~**Floor sim twin with the measured roller centring spring**~~ (done,
   above).
2. ~~**The detent as a SAWTOOTH angle-only push**~~ (done, above; the
   reading that led to it:) With the drivetrain shown stiff (film), the P 4000 map's
   (out + back)/2 is the angle-only push wherever both passes have a
   reading: it rises steadily from ~-0.09 to ~+0.10 duty over ~5.5 deg,
   then flips back within ~2 deg, inside the stretch both passes jump
   through, so the flip may be instant. It pushes AWAY from the flip on
   both sides: a sharp-topped hill (the detent) with rest points between.
   That gives a ramp then a release in BOTH directions of travel with
   similar ramps (5.3 / 5.1 deg, 0.046 / 0.053 duty/deg measured), and a
   jump landing ~1.25 deg short of the next valley (2.5 + 5 = 7.5). The
   friction is then roughly flat (~0.21-0.25 duty); its "dip" sits in the
   jumped-over stretch and is likely interpolation. The model has it the
   other way round (a symmetric friction bump, no angle-only push), and
   the sine `hill_nm` is a stand-in: friction 0.21 + sine 0.1 at 90 deg
   reproduced the jump sizes' handedness (test 3), but a pure sawtooth
   is the same both ways, so the handedness needs something else. To do:
   a sawtooth shape for the hill, then the jump-size replay and the floor
   twin again. The single/double MIX also needs notch-to-notch variation
   (a model change; fidelity, probably not the 9 Hz).
3. **Contact with both of those in**: a few `sim.contact_solref` / friction
   points, labelled by sink in mm (test 7's third bullet).
4. **Is the 40-48 Hz pitch buzz notch passing?** A notch every 7.5 deg of
   ring-vs-hub; compare the buzz frequency with the sideways roller speed
   in a floor record. Untested idea.
5. Then the retrain batch, section 8.
6. **Idea, unverified: the detent may be DETERMINISTIC, so the sim need
   not randomise it.** The sim draws the detent phase per episode
   (`DrivetrainSim.reset(rng)`; "nothing has measured where the wall
   sits", bike_params). It has now been located: in the raw drive-encoder
   frame (ring-vs-hub = 1.5 (P101 + P102), `drivetrain_trace.py
   bench_detent`) the three hung creeps of 10-24 put the sawtooth flip at
   0.35, 0.55 and 0.45 deg mod 7.5 (202227, 210446, 221037_creep_film;
   over two hours and two creep speeds). One servo turn moves ring-vs-hub
   1.5 x 360 = 540 deg = 72 notches, so a power-up turn-count reset keeps
   the phase. If that holds, the detent is a fixed function of the
   encoders the Pi already reads: the twin could place it from the run's
   start encoders instead of a draw, and a policy could even be told it.
   The user (10-25): the belt ratio is exact, the belts cannot skip, and
   the XC430's absolute single-turn angle survives resets -- which closes
   the ways the phase could move. It is one angle, not a combination:
   the detent sees only ring-vs-hub = (a - b)/2 (the hub's rolling does
   not move it), i.e. 1.5 (theta101 + theta102) mod 7.5 deg, and each
   servo turn adds a whole 72 notches, so even the single-turn readings
   give it. What it does NOT remove: backlash between each servo and the
   ring, a direction-dependent offset (+ and - jumps start ~2.6 deg
   apart); the floor runs cannot confirm the phase (their rests spread
   evenly). The catch is the policy: it cannot use a phase it cannot see.
   A fixed phase in training WITHOUT a new observation would only make
   every episode start at the same notch, something to overfit to, so the
   per-episode draw stays right for today's policies. Using it needs a
   notch-phase obs (built 10-29 as `obs_detent`, below; computable on the
   Pi from Present Position), a `general_spec` layout change on both sides,
   and a retrain. Cheap without any of that: start the twin
   (`run_vs_sim`) at the bike run's own phase instead of a draw.
   **Tested 10-29 (`analysis/servo/detent_phase.py`): the phase is fixed
   until the drivetrain is rebuilt.** Diff creeps with the bike on its
   side and the wheels up, after a ride and two power cycles, then the
   wheel turned ~120 deg between runs on one power-up (261029-000445,
   000645, 000756): flip 0.28 / 0.41 / 0.43 deg. With the four 10-24/25
   creeps, 7 captures over 5 days, 3+ power cycles, hung, loaded and on
   its side, wheel angle all round: **0.43 deg mod 7.5, max deviation
   0.15 deg (2 % of a notch)**. Single jump starts scatter ~0.4-0.5 deg
   (circular s.d.) around it. The 09-13 testbed creeps sit at 2.8-2.9:
   the drivetrain was rebuilt in between (section 3), so a refit of a
   belt or servo moves it and the number must be re-measured (one ~45 s
   creep).
   In raw Present Position counts (signed, Drive Mode 0, Homing Offset 0,
   as every capture recorded): the phase is the SUM S = c101 + c102, one
   count of S = 3 x 360 / 8192 = 0.132 deg of ring, one notch = 4096 / 72
   = 56.889 counts of S. The flip sits at S = 3.2 counts mod 56.889
   (captures 2.1-4.2; jump starts scatter 3-4 counts). As an integer:
   phase = (72 S - 230) mod 4096, a 12-bit notch angle.
   One wrapped input would cover the push (its one discontinuity is the
   flip's own), but sin and cos were chosen (user): the phase is the
   observation, not a model of the push. Drive Mode matters only through its Reverse bit, which negates one servo's
   Present Position and turns S into a difference. The 3.2-count offset
   could live in one servo's Homing Offset instead of the yaml, but that
   moves the frame everything else reads.
   A fourth capture (000007) was lost: 0-byte files after a power cycle;
   `Capture.save` now fsyncs, as `RunRecorder.close` already did.
   **Built 10-29: `obs_detent`** (`config/rl/rl_general_v5.yaml`, obs 17
   -> 20): sin and cos of `general_spec.detent_angle` (0 at the flip, 2 pi
   a notch) and the MEASURED diff rate, input_a - input_b from the
   differenced encoders through the odometry's 25 ms RateFilter, over
   diff_max (prev_diff's units: the command and what the drive made, side
   by side). The sim measures from the episode's draw
   (`DrivetrainSim.flip`, the sawtooth's top; the draw stays), the bike
   from `onboard.yaml` `detent_flip` (0.43 deg in the bench's frame,
   converted through `servo_ids` / `servo_sign`: the model's frame is its
   negative while A and B are swapped). CPU env, Warp kernels, `drive.py`
   and `run_bike` form it; `tests/test_detent_obs.py`. At 50 Hz the phase
   aliases above ~1.1 rad/s of servo diff (~23 Hz of notch per rad/s), so
   it can help only slow: standing, stick-slip, reversals. The detent
   stays in the physics either way.
7. **The detent is barely visible in the FLOOR runs' PWM (10-25).**
   `analysis/servo/floor_detent.py`: the diff-mode duty averaged by notch
   phase in the hung creep's encoder frame, split by direction, over both
   floor engages (174349, 191558). Moving, it is flat: angle-only part
   under 0.04 duty p-p, at the noise, against the hung map's +-0.1
   sawtooth; out/back +-0.27-0.31 (friction plus whatever the balance
   asks). Held still, it steps at the flip by -0.08 / -0.05 duty, the
   sign the hung map has (-0.2 there). So the floor data says the detent
   is there and at the hung creep's phase, but cannot size it under load.
8. **Test: creep and map with the rollers LOADED.** The bike held upright
   on the floor (a hand or a jig on the chassis, the rear roller carrying
   its share of the weight, the policy off), then the 10-24 bench set:
   `drivetrain_bench.py creep` at 0.5-1 % diff both ways (filmed if the
   tape still shows) and `map` at P 4000. Gives the loaded detent
   (friction, sawtooth size, its phase against the hung one: backlash
   shows as a shift) and how a loaded roller's slop sits in it, which is
   what the floor twin flies. Also settles item 6's backlash question.
   The steer is held straight by its own servo, `--hold-steer` (extended
   position at the flight gains, restored afterwards; it refuses a fork
   more than 45 deg off straight). Hold the chassis upright in roll but
   let the rear swing: the rollers must be free to roll it sideways, or
   the creep becomes a stall against floor friction. While it is set up,
   a loaded diff chirp at the flight gains measures the servo + floor
   response through 9 Hz directly, the link the closed-loop floor records
   cannot separate (10-25: from position, the bike's rollers lag their
   command ~115 deg at 9 Hz on the floor, the twin's ~65):

   ```sh
   P=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB8HMQ3-if00-port0
   B="--port $P --ids 101 102 --signs 1 -1 --hold-steer"
   python analysis/servo/drivetrain_bench.py creep $B --gains 100:1920 --note "upright on floor, rollers loaded"
   python analysis/servo/drivetrain_bench.py map --position-gains 4000:0:0 --hold 0.6 $B --note "upright on floor, rollers loaded, P4000"
   python analysis/servo/drivetrain_bench.py chirp --modes diff --f0 2 --f1 20 --seconds 15 --gains 100:1920 $B --note "upright on floor, rollers loaded"
   ```

   **Run 10-25 (261025-191638_creep, 192127_map, 192222_chirp).** The
   steer held: it ended 0-7 counts (<= 0.6 deg) from straight in all three.
   - Map at P 4000: loaded is the hung map. Diff friction 0.219 duty
     (hung 0.213), the same 7.5 deg sawtooth, angle-only p-p 0.365 (0.329),
     at the same phase; common 0.129 (0.111). The floor adds almost no
     static resistance with the rear free to swing.
   - Creep (`replay_jumps.py`): loading makes the stick-slip FINER. Bike
     singles / doubles, loaded against hung 202227: +1 % 19/2 (8/8),
     -1 % 11/5 (3/8), +3 % 42/13 (3/33), -3 % 20/22 (3/30). The - way
     still doubles more. The sim, in the air, is all singles at 1 %.
   - Diff chirp, +-1.67 rad/s at the servo, from position: gain 0.25-0.28
     at 2-5 Hz (-22 to -34 deg), 0.13 at 9 Hz (-49), coherence falling below
     0.5 above 8.5 Hz and gone by 11. The sim testbed fed the same stream
     sticks harder (0.18 at 6 Hz, 0.03 at 9). Both are friction-bound: at
     9 Hz the response is ~0.2 deg of shaft, two encoder counts. The bike's
     own 9 Hz band in 174349 is 1.55 rad/s RMS (sinusoid ~2.2), on top of
     a 2.5 RMS command that keeps the servo moving, so this chirp, with
     nothing else moving, measures a different regime. Not yet the
     loaded 9 Hz response; it needs a larger amplitude.
   - **Rerun at +-4.4 rad/s (`--amp 0.4`, 4-14 Hz, 261025-192925_chirp):
     the loaded servo is NOT slow.** Coherence 0.99 over 4-12 Hz.

     | diff, cmd -> velocity from position | 6 Hz | 9 Hz | 12 Hz |
     |---|---|---|---|
     | bike, held upright, loaded | 0.35 -43 | 0.35 -49 | 0.33 -69 |
     | sim testbed, same stream | 0.50 -62 | 0.31 -78 | 0.18 -85 |
     | bike in flight, 174349 (closed loop, 100 Hz) | 0.36 -77 | 0.31 -102 | 0.32 -97 |
     | twin in flight (10-25 loop breakdown) | | 0.5 -65 | |

     Held, the bike's rollers lag the model's by ~30 deg LESS at 9 Hz, at
     about the same gain. So the 10-25 suspect "servo plus floor load lags
     more on the bike" is cleared for the servo against a held bike. The
     flight record's extra ~53 deg (~16 ms at 9.4 Hz) is somewhere else.
     The candidates are the following; none is measured.
     - **Bookkeeping.** The record pairs tick k's command, written after the
       read, the AHRS and the policy, with tick k's position, read first.
       That adds at most the in-tick write offset, under ~8 ms (<= ~27 deg).
       Not logged: `rec.add` keeps the write's duration, not its start.
     - **The free bike's dynamic load.** It is absent with the chassis
       held. The twin flies that load and does not show the extra lag.
     - **The flight waveform.** The command is a 50 Hz staircase with common
       mode and steer motion on top.
   - **Next: common and diff together, a stepped sine.** In the six floor
     runs, common's 7-12 Hz command is 0.26-0.44 of diff's by RMS (0.16-0.40
     coherent) and LEADS it by 21-63 deg (fixes_72M ~35; rake20 63). That is
     in the bench's frame, the same per-servo goals as `cmd`. `chirp --mix
     RATIO PHASE` drives that combination, and `--steps N --dwell S` holds
     each of N log-spaced frequencies for whole cycles. The quick look prints
     a per-step lock-in from position, diff and common separately. Run the
     pure-diff steps as the control. `--ahrs` also records every AHRS sample
     (chassis frame, the bundle's mount, `ahrs.npz` on the capture's clock),
     which asks whether the pitch shake appears with the fork held and only
     the rear wheel driving:

     ```sh
     python analysis/servo/drivetrain_bench.py chirp --mix 0.3 35 --amp 0.4 --f0 4 --f1 14 --steps 12 --dwell 2 --gains 100:1920 $B --ahrs --note "upright on floor, rollers loaded, mix 0.3@+35"
     python analysis/servo/drivetrain_bench.py chirp --modes diff --amp 0.4 --f0 4 --f1 14 --steps 12 --dwell 2 --gains 100:1920 $B --ahrs --note "upright on floor, rollers loaded, diff steps"
     ```

   - **Run 10-25 (261025-194827 / 194902 mix, 194945 diff; 194827 let
     the bike roll a little).** The fork was held, but NOT straight
     (found later 10-25, `shutdown.steer_hold`): 24 counts (2.1 deg) off
     in all three, and 28 (2.5 deg) in 200325. `--hold-steer` wrote the
     straight goal before torque on, and a goal written with torque off
     is dropped (control_tables/README.md), so the steer held wherever
     it was. Only 191638 (90 -> 0) and 200407 (28 -> 0) moved to
     straight; 192127 / 192222 / 192925 started within 9 counts. Fixed:
     torque on, then a raised-cosine move to straight, the goal read back.
     - **Common does not change the diff response.** The two mix runs
       repeat to ~0.02 and a few degrees. Diff, from position: 0.33-0.35 at
       -45 to -58 over 7-10 Hz with or without common, as the 192925 sweep
       gave. Common is 0.11-0.21 at 4-5 Hz (a poor fit, residual 0.4: the
       wheel against the floor), then 0.28-0.32 at -32 to -50 over 8-10 Hz.
     - **The AHRS is clean in the window**: 198 Hz, QoS 5. Its only gaps
       (~200 ms) come after the capture ends, at the frame conversion.
     - **Above ~7 Hz the rollers turn but the rear barely moves sideways.**
       The hand held roll, by the procedure, and left yaw free. Free
       rolling with the front contact as the pivot gives 21.6 deg/s of yaw
       per servo rad/s of diff (3 x 2.4 x 10.5 mm / 0.2005 m). Measured
       yaw against that, by lock-in on the measured diff velocity, in all
       three runs:

       | step | yaw / free rolling | lateral accel the rear would need |
       |---|---|---|
       | 4-5 Hz | 0.8-1.2 | ~0.4 g |
       | 7 Hz | 0.24-0.49 | ~0.5 g |
       | 9-14 Hz | 0.12-0.20 | 0.6-0.9 g |

       The phase slides steadily through the band. Its absolute value
       depends on sign conventions that are not checked here, so read
       only the trend.

       Gyro per diff COMMAND at 9-10 Hz:

       | | roll | yaw | pitch |
       |---|---|---|---|
       | held, mix / diff only | 0.6-0.8 / 1.5-1.65 | 0.9-1.4 / 1.1-1.3 | 0.20-0.27 / 0.15-0.22 |
       | flight, diff alone | 13.5 / 13.8 | 10.8 / 11.0 | 0.41 / 0.39 |
       | flight, diff and steer together | 7.7 / 8.3 | 5.9 / 1.1 | 0.28 / 0.42 |

       Rows are 174349 / 191558. The fit with steer is poorly conditioned:
       diff and steer are 93-97 % coherent at 9.4 Hz in flight. Still,
       flight yaw at 9 Hz is mostly the STEER's, and the rear's own share
       (1-6) brackets the held bike's ~1.1. That is ~0.15 of free rolling
       at the servo's 0.33 gain. Roll is not comparable, held by hand.

       What stops the rear above 7 Hz: the leading candidate is the
       **roller slop**. The rollers' ~+-7.8 deg of play (the 10-24 film)
       is 15.6 deg p-p of roller angle. A backlash of that width, nothing
       fitted, gives the fundamental's fraction from the measured roller
       stroke (ring-vs-hub x 2.4):

       | step | roller stroke amp | yaw / free rolling | backlash +-7.8 |
       |---|---|---|---|
       | 4 Hz | 32-34 deg | 1.1-1.2 | 0.85 |
       | 7-9 Hz | 10-14 | 0.14-0.49 | 0.31-0.54 |
       | 11-14 Hz | 6-9 | 0.12-0.18 | 0-0.11 |

       It has the shape. The measured ratio falls ~1-2 Hz earlier,
       though, and keeps a ~0.13 floor that pure backlash does not, so it
       may not be the whole story. Other candidates:
       - **Slip.** The lateral acceleration needed passes ~0.5 g where
         the ratio falls; the weight share and roller mu are not measured.
       - **A linear roll-off.**

       AMPLITUDE separates all three at a fixed frequency: with slop the
       ratio RISES with stroke, with slip it FALLS, and a linear roll-off
       stays put. Test: the diff steps at `--amp 0.2` and `--amp 0.6`. The
       model has the slop (`drivetrain_model.roller_slop`), so if the slop
       is confirmed, the next question is whether the floor twin's rear
       shows the same drop.
   - **Next, the converse: steer swept on the ground, rear held.**
     `steer_response.py --hold-drives --ahrs`. Both drives sit at
     velocity mode, goal 0, P 100 / I 1920, as they fly; their mode and
     gains come back after. The steer is first moved smoothly to straight
     (180 mod 180; refused beyond 45 deg; since 10-25, the 200807 sweep
     ran ~3.3 deg off), then runs one faded tone at a time around it. At
     each tone it reports the shaft's gain and phase, how far the drives
     drifted, and the gyro per steer goal. In flight the 9 Hz steer goal
     is ~+-2.4-2.6 deg (7-12 Hz band, both floor runs) and the shaft
     ~+-1.4-1.6, hence 2.5 and 5. NO Enter prompt: torque comes on at
     launch.

     ```sh
     python analysis/servo/steer_response.py --port $P --id 103 --amp-deg 2.5 5 --freqs 4 5 6 7 8 9 10 11 12 14 --hold-drives --ahrs --out traces/bench/steer_ground_$(date +%y%m%d-%H%M%S).json --note "upright on floor, drives held"
     ```

   - **Amplitude test (10-25: 261025-200325 at `--amp 0.2`, 194945 at
     0.4, 200407 at 0.6): the drop is SLOP, not slip.** Yaw / free
     rolling by step:

     | Hz | 0.2 | 0.4 | 0.6 | lateral g at 0.6 |
     |---|---|---|---|---|
     | 4 | 0.30 | 1.10 | 0.95 | 0.75 |
     | 6.3 | 0.03 | 0.64 | 1.02 | 0.82 |
     | 7.9 | 0.01 | 0.14 | 1.15 | 0.89 |
     | 8.9 | - | 0.14 | **1.53** | 0.92 |
     | 9.95 | - | 0.18 | **1.52** | 0.98 |
     | 11.2 | - | 0.12 | 0.27 | 1.30 |
     | 14 | - | 0.14 | 0.09 | 1.58 |

     At each frequency the ratio RISES with stroke. Slip would make it
     fall; at 0.6 the rear follows fully while the rolling would need
     0.8-1.0 g. "-": the servo barely moved (gain <= 0.06), so the ratio
     is noise. The +-7.8 deg backlash tracks the 0.2 and 0.4 rows.

     NEW at 0.6: the ratio peaks at ~1.5 at 9-10 Hz, its phase sliding
     ~80 deg from 7 to 11 Hz, then collapses at 11 Hz. Two readings,
     not separated:
     - **A yaw mode near the shake frequency.** Yaw above free rolling
       means the front moves the other way.
     - **Slip onset**, where the needed lateral acceleration passes
       ~1.3 g.

     Servo diff gain, from position, at 9 Hz: 0.09 / 0.35 / 0.32 for
     0.2 / 0.4 / 0.6.
   - **Steer on the ground, drives held (10-25,
     `traces/bench/steer_ground_261025-200807.json` + `_ahrs.npz`).**
     - **The drives did not move.** Not one count (0.26 deg of
       ring-vs-hub) at any tone boundary over 60 s. As expected: the
       geared XC430 is hard to backdrive from the ground. The script
       reads only between tones.
     - **The shaft against its goal:**

       | | 5 Hz | 9 Hz | 14 Hz |
       |---|---|---|---|
       | +-2.5 | 0.62 -86 | 0.33 -124 | 0.18 -173 |
       | +-5 | 0.72 -82 | 0.43 -123 | 0.25 -175 |

       Bare on the bench (10-17), the shaft gave 0.83, -87 at 5.2 Hz, so
       the ground costs gain more than phase at 5 Hz. The flight
       closed-loop steer was 0.5-0.6 at -137 to -156 over 6-12 Hz.
     - **Gyro per steer.** At 9 Hz, per steer GOAL: roll 2.8-3.0, yaw
       2.4-2.6, pitch 0.21 deg/s per deg. Per SHAFT degree that is roll
       6.6-9.1, yaw 6.0-7.3. The flight's steer share (the two-input
       fit above, per measured steer degree) is roll 10-11 and yaw 9-17,
       the same order; that fit is poorly conditioned.

   - **Pitch at 9 Hz: front and rear about equally.** Each held test's
     pitch per input at 8-10 Hz, times that input's 7-12 Hz RMS in flight
     (174349 / 191558):

     | source | pitch per input | flight input | predicts |
     |---|---|---|---|
     | rear, diff only | 0.17-0.18 | 1.55-1.63 rad/s | 0.27-0.29 deg/s |
     | rear, diff + common | 0.26 | same | ~0.41 |
     | front, steer | 0.20-0.22 /deg | 1.69-1.86 deg | 0.34-0.41 |
     | flight, measured | | | 0.94-0.95 |

     Together 0.55-0.8 depending on phase. Neither wheel dominates by
     size.

     **By PHASE, the flight's 9 Hz pitch is the rear WITH its forward
     drive.** Each held test's complex response, weighted by the flight's
     own steer/diff relation, against flight pitch per diff command (all
     in written servo-unit goals and chassis-frame gyro):

     | phase vs flight diff cmd, 8.6 / 9.4 / 10.2 Hz | |
     |---|---|
     | front (steer) alone | -1 / -12 / -23 (191558 -20 at 9.4) |
     | rear, diff only (amp 0.4; 0.6) | -82 / -83 / -86 (-70 / -67 / -75) |
     | rear, diff + common (mix 0.3 @ +35) | -169 / -170 / -171 |
     | flight, measured, 174349 | -173 / -171 / -162 (coherence 0.35-0.53) |
     | flight, measured, 191558 | -134 at 9.4 (coherence 0.46) |

     Only the rear with common has the flight's phase, at 0.20-0.29 of
     the measured 0.33-0.41 magnitude. The steer's share arrives almost
     opposite to it: added, the two cancel to 0.07-0.12, and the flight
     shows no such cancellation. So in flight the steer's pitch is not
     what the held test measured: there the rear was LOCKED, in flight
     it is free. Superposition of the held tests does not hold.

     **Steering geometry is NOT this pitch.**
     - **Static, in the model.** Turning the fork either way lifts the
       nose, an even function of steer: -0.004 deg at +-5, -0.018 at
       +-10, -0.028 at +-20, i.e. ~0.17e-3 deg/deg^2. A 9 Hz steer about
       straight would pitch the chassis at 18 Hz.
     - **On the bench, the even part is the same order.** In the steer
       test the bike's pitch at 2f is ~10x smaller than at f. Fitted
       against the shaft angle squared at the 4-8 Hz tones it is
       0.36-1.0e-3 deg/deg^2, at a phase of -65 to -152. That is
       dynamic: the chassis is not quasi-static at 8-16 Hz, so the
       static SIGN on the real bike is unmeasured.
     - **The bike's pitch at f is not geometry.** An off-centre fork turns
       the even effect linear. The sweep ran about 3.3 deg off straight
       (180 is centre): steer_response sweeps around wherever the shaft
       starts and never centres it first. The twin makes ~0.00
       deg/s of pitch per steer deg centred and 0.01 from 3.3 deg, against
       the bike's 0.19-0.22, so the conclusion holds either way. The twin
       has the geometry; the bike's first-order pitch comes from something
       the twin lacks (not identified).
     - **To get the static sign and size: slow sweeps to the flight's
       range.** In the six floor runs the steer's 5-95th percentile is
       about +-13-28 deg and its p99 reaches +-36; three runs sit 6-8 deg
       to one side on median (171115, 172337, 191558). Since 10-25
       `steer_response` prints the AHRS attitude's nose-up pitch against
       the shaft angle from straight for tones <= 0.5 Hz, binned by 5 deg,
       out and back separately:

       ```sh
       python analysis/servo/steer_response.py --port $P --id 103 --amp-deg 30 40 --freqs 0.1 0.2 --seconds 25 --fade 4 --hold-drives --ahrs --out traces/bench/steer_static_$(date +%y%m%d-%H%M%S).json --note "upright on floor, drives held, slow sweep"
       ```

       It takes 100 s, peaking at 50 deg/s. The model's answer to compare:
       nose UP ~0.028 deg at 20 deg.

       **Run 10-25 (`traces/bench/steer_static_261025-203535`).** Centred
       from -7.8 deg. The shaft tracked 0.98-0.99 out to +-38.8 deg. As
       the user saw, the fork drags the bike round the held rear wheel:
       yaw 4.9-6.8 deg p-p, which the twin matches (5.2-7.0). Fit nose-up
       pitch = a + b steer + c steer^2 + d roll + e roll^2
       (`held_twin.py static ... --k 5`); "even" is c x 35^2:

       | tone | bike even | twin even | bike b (deg/deg) | twin b |
       |---|---|---|---|---|
       | +-30 @ 0.1 Hz | -0.035 | +0.012 | -1.8e-3 | 0 |
       | +-30 @ 0.2 Hz | -0.101 | +0.003 | -1.4e-3 | 0 |
       | +-40 @ 0.1 Hz | -0.101 | -0.000 | -0.8e-3 | 0 |
       | +-40 @ 0.2 Hz | -0.074 | -0.016 | -0.0e-3 | 0 |

       - **The real bike noses DOWN by ~0.07-0.1 deg at +-35**, either
         way. The twin is nearly flat (+0.01 inside +-30, -0.02 at 40).
       - **The bike also has an odd part the twin lacks.** About 0.05
         deg nose-up at 30 deg of LEFT steer, in three of four tones.
       - **The hold is ONE FINGER on the rear rollcage** (the user, 10-25),
         a balancing touch, so the bike's 2.8-4.2 deg of roll p-p is the
         roll needed to keep it up. The twin's k 5 roll spring is stiffer
         (0.3-0.7). At k 1.8, about the least that keeps it up, the twin
         rolls 1.3-3.8 deg p-p and its even part becomes:
         - +-30 tones: -0.007 to +0.008, still flat (bike -0.035 to -0.10);
         - +-40 tones: -0.047 to -0.063 (bike -0.07 to -0.10).

         So the twin's +-40 nose-down is mostly the roll it is allowed.
         The bike's +-30 nose-down and its odd part are not in the twin
         at any hand stiffness tried.
       - **Caveats.** Residuals 0.024-0.033 deg. The roll term's sign is
         not stable across tones: a nuisance fit, not a correction.

       Size against 9 Hz: static pitch per steer degree, at a fork up to
       8 deg off centre (the flight medians), is at most ~2.8e-3 deg/deg.
       With the flight's 9 Hz shaft RMS (~1.0-1.1 deg) that is <= ~0.16-
       0.18 deg/s. The bench's dynamic 9 Hz steer pitch at flight
       amplitudes is 0.36-0.40, and flight's total is 0.95. So the static
       shape is at most a minor share; the steer's 9 Hz pitch is mostly
       dynamic.

     Uncertainties: flight pitch is only ~0.5 coherent with the diff.
     The flight record pairs each tick's gyro, read first, with that
     tick's command, written after the policy; that is worth +-20-30 deg
     at 9.4 Hz, against ~1 ms on the bench.
   - **The held twin (10-25, `analysis/servo/held_twin.py`).** The floor
     model with fixes_72M's drivetrain overlay and the steer gearbox
     hooks. The bench commands replayed (ctrl by inverting
     `BikeBus.write_commands`), and the hand as a roll-only spring-damper
     (k 2 and 10 N m/rad; 0.5 is under m g h and falls). The real hold
     is one finger on the rear rollcage, a balancing touch, so the low k
     is nearer. At 9 Hz,
     bike / twin:

     | | bike | twin |
     |---|---|---|
     | diff servo, amp 0.4 | 0.35 -53 | 0.29-0.31 -70 |
     | yaw / free rolling, amp 0.4, 7 Hz | 0.24 | 0.68-1.46 |
     | yaw / free rolling, amp 0.4, >= 10 Hz | 0.12-0.18 | 0.00-0.02 |
     | yaw / free rolling, amp 0.6, peak | 1.53 at 8.9-10 Hz | 1.0-1.5 at 7.9 Hz |
     | yaw / free rolling, amp 0.6, 11 Hz | 0.27 | 0.60-0.88 |
     | pitch per diff cmd | 0.13-0.22 | 0.00-0.06 |
     | steer shaft +-2.5 / +-5 | 0.33 -124 / 0.43 -123 | 0.20 -122 / 0.39 -112 |
     | roll per steer SHAFT deg | 6.6-9.1 | 2.8-3.3 |
     | yaw per steer shaft deg | 6.0-7.3 | 4.2-4.7 |
     | pitch per steer goal deg | 0.21 | ~0.00 |

     What it says:
     - **Rear.** The twin's slop is the same backlash, but its drop
       comes ~1-2 Hz later and goes to zero; the bike keeps a ~0.15 floor.
       At a large stroke the twin has a yaw peak too, at ~8 Hz rather
       than the bike's 9-10, and it does not collapse at 11 Hz.
     - **Front: the bigger gap.** Per degree of steer shaft the real bike
       rolls ~2.5x and yaws ~1.5x what the twin does, whatever the hand
       stiffness. The twin's small-amplitude steer is stickier (0.20
       against 0.33).
     - **Pitch: the twin makes almost none** from either wheel; the bike
       makes 0.2 deg/s per unit from each. Rigid against something
       compliant (tyre, frame, headset): not identified.

     - **The 38-48 Hz pitch buzz IS there with the fork held and only the
       rear driving.** The diff-only run peaks at 44.1 Hz, at the sensor's
       full 200 Hz, which rules out the flight log's 100 Hz folding a
       52-56 Hz line down. Pitch rate, 35-55 Hz RMS: 0.10-0.12 deg/s before
       torque, 2.7-4.4 at the 4 Hz steps, 0.2-0.5 at 11-14 Hz; flight
       2.4-2.5. It follows the stroke's size, not its speed. A
       ring-vs-hub p-p of 27 deg goes with 2.7-4.4, 6-8 deg with
       0.2-0.5, at about the same peak speed. Notch crossings per
       second do not separate the steps (a shaft dithering on one notch
       edge counts every time): correlation 0.57. A guess, not measured:
       detent release clicks ringing a ~44 Hz structural mode, which
       flight's large roller strokes hit all the time. The 9 Hz pitch is
       small both ways (7-12 Hz: 0.5-0.9 deg/s held, ~1.0 in flight).

9. **Test: what rings? A guided pluck test (`analysis/servo/tap_test.py`,
   queued 10-25).** The bike responds at 9 Hz 1.5-2.5x harder than the
   twin (item 8), the flight records' yaw-vs-roll phase jumps ~90 deg
   between 6 and 7 Hz (test 2), and the user names two flex points: the
   axle and the steerer tube (the fork itself is fairly rigid). A pluck
   rings the whole structure and the AHRS hears it wherever it sits. The
   script prompts each spot (Enter, three plucks ~3 s apart, Enter): a
   baseline, the steerer sideways, each tyre sideways, the rollcage
   sideways and fore-aft, each wing tip, a knuckle on the frame. Servos
   locked as in flight (drives at velocity 0, steer straight), or
   `--limp`. Held as for item 8: wheels down, one finger on the rollcage.
   It prints each spot's strongest 3-60 Hz gyro peak over the baseline's,
   and its 6-10 Hz power over the baseline's.

   ```sh
   python analysis/servo/tap_test.py --note "upright on floor, one finger on rollcage"
   python analysis/servo/tap_test.py --limp --tag limp --note "..."
   ```

   **Run 10-25 (traces/bench/tap_261025-210730; `tap_test.py --report`
   reprints it).** Drives held (the operator heard them buzz); the steer
   torqued but held 8.3 deg off straight (below); the wings were limp, which the script did not then
   touch, so a flick moved the wing rather than ringing it: the wing
   spots are left out. 2-4 plucks found per spot, AHRS 192 Hz.
   Ring-down peaks, 2-30 Hz, as power over that axis's largest:

   | spot | roll | yaw |
   |---|---|---|
   | steerer | 2.9 (1.00), **7.1 (0.47)** | **7.2 (1.00)**, 9.3 (0.52) |
   | front_axle | 3.5, 4.7, 6.2 (0.23) | 2.9, 4.6, 6.2 (0.75) |
   | rear_axle | 2.7 (1.00), 6.2 (0.06) | **6.2 (1.00)**, 2.8-3.3 (0.53) |
   | cage_side | 3.0 (1.00), 6.2 (0.31) | **6.2 (1.00)**, 7.4-8.5 (0.54-0.62) |
   | cage_fore_aft | 3.7, 6.6 (0.48) | 2.8, 5.7-6.5 (0.92-0.98) |
   | frame_knock | 2.8 (1.00) | 2.6, 4.1, 5.6 (0.66) |

   - **Nothing rings at 9 Hz.** No spot's largest ring is between 8 and
     11 Hz; the nearest is the steerer's yaw at 9.3 (0.52 of its 7.2).
   - **A ~6.2 Hz ring, mostly YAW, from almost every spot**: the largest
     yaw ring for the rear axle and the rollcage sideways, 0.75-0.98 for
     the front axle and fore-aft. (The wings did not swing, by eye.) The same frequency
     wherever plucked, so a mode of the bike as held, not of one part.
     The flight records' yaw-vs-roll phase jumps ~90 deg between 6 and
     7 Hz (test 2): this is a candidate for it. Plucking the steerer
     gives 7.1-7.2 instead, in roll and yaw both.
   - **Roll rings at 2.5-3.7 Hz everywhere**: the whole bike rocking,
     on the finger hold and the tyres (the twin's hand at k 5 gives 2.3).
   - **Pitch: not excited** (<= 2 deg/s at the plucks); no reading.
   - Damping: the log-decrement estimates scatter around zero, some
     negative; not quoted.
   - **The steer held 8.3 deg off straight.** It started at 95 counts
     and ended at 94 (`steer_hold.present_after`), the goal 0; torque was
     on (the operator). The straight goal was written before torque on
     and dropped (item 8's correction above), so it held where it was.
     The steerer's 7.1-7.2 is with a torqued fork 8.3 deg off. Fixed in
     `drivetrain_bench._steer_hold_on`, which `tap_test` uses; `tap_test`
     also reads each held servo back after the settle (torque, goal,
     shaft, Bus Watchdog, error), stops to ask if one is > 2 deg off,
     reads torque and error again at the end, and holds the wings as
     flown, its goals written after torque on too.

   The twin plucked the same way (`held_twin.py pluck --k 1.8 5`, 1 N,
   push 0.5 s, let go, servos locked) has no ring common to the spots:
   yaw 4.8 (steerer), 7.4 / 12.5 (front axle), a broad 3.8-6.9 (rear
   axle), 3.6 (cage side, k 5). Its force and hold only stand in for
   the hand's, so this says the 6.2 is not obviously there; it is not
   a careful absence.

   **Is the 6.2 Hz the rear rollers?** (the operator's guess, 10-25). A
   rough number for it, inferred, not measured: the drives' hold spring
   (2.69 duty/rad at I 1920, x 0.62-0.69 N m per duty, ~1.8 N m/rad per
   servo), both servos at 0.0756 m of sideways contact travel per servo
   rad, is ~630 N/m at the rear contact; through the 0.2005 m wheelbase
   that is ~25 N m/rad of yaw, against the twin's 0.021 kg m^2 of yaw
   inertia about the front contact: **~5.5 Hz**. But the operator saw
   the drives not move at all, and the ring's ~0.9 mm at the rear sits
   inside the rollers' +-7.8 deg play (~1.4 mm), where only the weak
   centring spring holds. Either the servos moved under ~1 deg (the
   estimate says ~0.7 deg), or the spring is somewhere else (tyre or
   roller rubber, the axle). `tap_test` now logs every held servo's
   shaft during the plucks (`servos.npz`) and prints the rollers'
   ring-vs-hub p-p and spectrum at each pluck. Next run, wings held:

   ```sh
   python analysis/servo/tap_test.py --spots rear_axle cage_side steerer --plucks 4 --note "locked, I 1920"
   python analysis/servo/tap_test.py --spots rear_axle cage_side steerer --plucks 4 --drive-gains 100:960 --tag i960 --note "locked, I 960"
   ```

   If it is the drives' hold spring, halving I moves it by ~1/sqrt(2)
   (6.2 -> ~4.4 Hz) and the rollers ring with the gyro; a rubber or
   axle mode stays put.

   **Run 10-25 (tap_261025-213003 at I 1920, 213210_i960 at I 960;
   3 plucks asked, 4-5 found per spot).** Every hold checked: the steer
   ramped 263 -> 0 counts and held 0 (both runs), the wings held 0.0 deg
   off, all four servos torqued with no error at the end. Shafts logged
   at 145 Hz, no read errors. Yaw ring-down peaks (largest = 1.00):

   | spot | yaw, I 1920 | yaw, I 960 | rollers ring-vs-hub p-p | steer p-p |
   |---|---|---|---|---|
   | rear_axle | **7.1** (1.00) | **6.7** (1.00) | 0.03 / 0.01 deg | 0.00 / 0.00 |
   | cage_side | **6.2** (1.00) | **6.3** (1.00) | 0.04 / 0.00 | 0.09 / 0.09 |
   | steerer | 5.4 (1.00), 6.6 (0.89) | 4.3 (1.00), 9.0 (0.72) | 0.03 / 0.04 | 0.00 / 0.04 |

   - **It is not the drives' hold spring.** Halving I left the ring at
     6.2-7.1 Hz (that spring would put it near 4.4), and the drive
     shafts moved 0.00-0.04 servo deg p-p, under one encoder count
     (0.088 deg), as the operator saw. The steer shaft did not move
     either (<= 0.09).
   - So the 6-7 Hz yaw ring's spring is downstream of every servo:
     the rollers inside their play (invisible to the encoders), roller
     or tyre rubber, the axle, the steerer tube. The estimate above
     (~5.5 Hz) assumed the servos were the spring and is dropped.
   - **What the two plucks do (the operator):** both the rear-axle and
     the rollcage-sideways push take the rollers to one END of their
     play, and the release lets them go. So the ring starts from the
     slop's wall and runs through the play, the one place the encoders
     cannot see. The hand-measured centring spring (~1e-2 N m/rad per
     roller, unloaded, 10-24 video) through the 10.5 mm roller radius
     and the wheelbase is only ~90 N/m sideways and ~2-3 Hz of yaw
     (inferred), too soft for 6-7 Hz on its own. Open: a stiffer loaded
     roller, a spring at the front contact while the rear is free in its
     play, or backlash dynamics (whose frequency moves with amplitude).
     The twin plucked with 1e-3 and 1e-2 springs (scratch, 1 and 4 N)
     gives no clean ring either way, so it does not decide it.
   - **Per pluck (`--report` now prints it; 0.7-60 Hz, strongest yaw
     peak 4-30 Hz):** the rear-axle and rollcage plucks large enough to
     read (yaw peak 10-27 deg/s) ring at 6.0-6.6 Hz (210730), 7.1-8.6
     (213003, I 1920) and 6.3-7.8 (213210, I 960). So the ring is 6-8.6
     Hz and wanders by ~1-2 Hz between plucks of about the same size,
     which a fixed linear mode would not; plucks under ~6 deg/s are too
     small to read. The summed-spectrum peaks above hide that spread.
   - **Next, the pluck's size through the play** (spots `rear_part`,
     `rear_end`): part of the free play and let go before the end, then
     to the end of it. Backlash should ring at a different frequency
     for the two; a spring mode should not.

     ```sh
     python analysis/servo/tap_test.py --spots rear_part rear_end --plucks 5 --note "locked, rear part vs end of play"
     ```

     **Run 10-25 (tap_261025-214929; holds checked: steer 1.2 deg off,
     wings 0.0).** With a third spot the operator added,
     `whole_wheelbase`: the wheels kept down and the frame twisted, front
     and rear together, loading the roller play and the fork / steerer /
     front wheel at once. 7 plucks found per spot. Per pluck:

     | spot | yaw peak, deg/s | ring, Hz |
     |---|---|---|
     | rear_part | 2.8-7.8 | 4.3-10.7, scattered |
     | rear_end | 9.5-20.4 | **5.9-7.8** in 5 of 7 (10.1, 18.9) |
     | whole_wheelbase | 16.5-42.0 | **4.0-5.3** in 5 of 7 (9.6, 12.1) |

     - **The end-of-play ring repeats: 6-7 Hz,** the fourth run with it
       (summed: 6.0, 6.7).
     - **Part of the play does not decide it.** Every part-play pluck is
       under 8 deg/s, where a single pluck's ring cannot be read (the
       floor is ~6); its summed spectrum is flat between 4.4 and 6.2. A
       part-play pluck is at most ~1.4 mm at the contact, so it cannot
       be made bigger: the amplitude test needs a quieter reading, not a
       harder push.
     - **Twisting the whole wheelbase rings LOWER, 4-5 Hz,** at twice the
       size. Front and rear compliance in series would be softer than
       either alone, which fits; it is a different push, so it is not an
       amplitude test of the same ring.
     - Shafts again still: drives 0.00-0.07 servo deg p-p; the steer
       moved 0.26 deg in the twist, 0.00-0.09 otherwise.
     - **What the 6-7 Hz implies for the model** (inferred): pivoting
       about the front contact with the twin's 0.021 kg m^2, 6.5 Hz needs
       ~35 N m/rad of yaw stiffness, ~900 N/m sideways at the rear
       contact, all of it beyond the servos; the hand-measured centring
       spring gives ~90 N/m per roller, ~10x short. **But the twin does
       not follow that picture:** plucked (scratch, 1 N) with the roller
       centring at 1e-2, 5e-2 and 1e-1 N m/rad it gives no clean yaw
       ring at any (rear-axle yaw peaks spread 5.4-10.6 Hz, and 2.8 Hz
       at 1e-1), so the pivot-about-the-front estimate is not how the
       twin moves, and "raise the spring 10x" is not supported. The
       measured spring (0.007-0.011) and the detent were already flown
       in the floor twin (10-24 evening, above): no 8-10 Hz peak. No
       slop-model change follows from the plucks yet; the part that
       rings has to be found first.
   - The steerer's own peak moves between runs (5.4, 4.3; 7.1-7.2 when
     it sat 8.3 deg off), so it is not a clean ring of its own.
   - The rear-axle ring sits a little higher with the fork held
     straight (6.7-7.1) than at 8.3 deg off (6.2); with 1.5 s windows
     the bins are ~0.7 Hz wide, so that is at the edge of resolution.

## The tool

`analysis/policy/run_vs_sim.py <traces/bike/run>` plots a run's longest
engaged stretch beside the run's own policy standing in the sim on its own
sensors (same AHRS level, odometry and detailed drivetrain): time series on
shared y-limits, Welch spectra, and a std / >2 Hz / >10 Hz table. Run it on
every session below; it is the common yardstick.

What it showed on run 174349 (no inputs, 49.6 s), bike / sim:

| signal | >2 Hz RMS | 7-12 Hz RMS | 7-12 Hz, bike/sim |
|---|---|---|---|
| roll rate (gyro x) | 27.3 / 10.1 deg/s | 21.9 / 3.2 | 6.8x |
| yaw rate (gyro z) | 24.2 / 12.1 deg/s | 18.4 / 4.0 | 4.6x |
| steer target | 3.3 / 1.1 deg | 1.73 / 0.07 | **25x** |
| steer (encoder) | 2.6 / 1.0 deg | 0.99 / 0.07 | 14x |
| drive A cmd | 5.4 / 1.9 rad/s | 4.1 / 0.47 | 8.7x |
| drive B cmd | 7.3 / 2.5 rad/s | 5.8 / 0.63 | 9.3x |

(Sim with the Pi's steer rate and the steer's gearbox and headset friction,
both added to `run_vs_sim.py` on 10-24; the first version had neither.) The
steer's spectral PEAK is at 2-3 Hz, which once read as "the steer is not in
it"; the 9 Hz bump is there, 2.9x its neighbouring bands. The encoder carries
~0.57 of the target at 9 Hz, about what the fitted servo passes there, so
the shaft follows a command that oscillates.

The gyro held still reads 0.25-0.5 deg/s above 10 Hz, so this is motion,
not sensor noise. The ~41 Hz peak in the commands is the 9 Hz one's image
through the policy's 50 Hz zero-order hold (50 - 9), not a second
oscillation; the sim's bump near 45 Hz is the same for its ~5 Hz content.

## The tests

### 1. Fly a policy trained on this plant  [DONE 10-24: the shake stays]

Run 261024-191558, `general_puffer_rake20_80M`, no inputs, 84 s engaged,
0 bad reads. 7-12 Hz RMS bike / sim, against the pointer's run 174349:

| signal | rake20 bike / sim | pointer bike (174349) |
|---|---|---|
| roll rate | 24.5 / 4.1 deg/s | 21.9 |
| yaw rate | 20.4 / 5.3 deg/s | 18.4 |
| steer target | 2.03 / 0.11 deg | 1.73 |
| drive A / B cmd | 4.9 / 6.0 against 0.63 / 0.65 rad/s | 4.1 / 5.8 |

Same shake, a little larger, peak 7.8-8.8 Hz. A policy trained on this
plant with the Pi's steer rate does not remove it, so it is not the
pointer's training: the bike has it. The wings are not it either: the crank
sat -34 to +24 deg off stow across the runs, never commanded, and the
quietest stow (+4 deg, 174349) shook the same. (Since 10-24 an engage sends
uncommanded wings to stow.)

**An MLP, no memory (10-25, run 261025-225207).** `general_rl_v3_s1`: CPU
PPO, 128x128, same plant digest (bc66ca20), AHRS level, odometry, firmware
gains, drivetrain overlay and obs layout as the pointer; only the policy
differs (and it trained before the 10-14 env fixes, so its prev_* entries
were one step stale in training). Engages 8.4-20.1 s (fell, misplaced) and
31.3-83.8 s (clean, no input). `run_vs_sim.py`, the clean one, 7-12 Hz RMS
bike / sim:

| signal | v3_s1 bike / sim | pointer 174349 | rake20 191558 |
|---|---|---|---|
| roll rate, deg/s | 15.6 / 3.1 | 21.9 | 24.5 |
| yaw rate, deg/s | 21.2 / 3.8 | 18.4 | 20.4 |
| steer target, deg | 1.38 / 0.12 | 1.73 | 2.03 |
| drive A / B cmd, rad/s | 6.2 / 5.6 against 0.64 / 0.83 | 4.1 / 5.8 | 4.9 / 6.0 |
| >10 Hz, drive A / B, rad/s | 5.8 / 5.5 | 2.7 / 3.8 | 2.8 / 3.7 |

It shakes as hard, but the PEAK MOVED. Welch, 1024 points, 5-20 Hz, the
peak and its half-power span:

| run | policy | roll rate | yaw rate | drive A |
|---|---|---|---|---|
| 174349 | fixes_72M (MinGRU) | 8.9 (8.1-10.3) | 8.9 (8.1-9.7) | 8.9 (8.1-10.3) |
| 191558 | rake20_80M (MinGRU) | 7.9 (7.4-9.1) | 7.9 (7.4-9.0) | 8.8 (7.2-9.0) |
| 225207, clean | v3_s1 (MLP) | 11.2 (10.8-11.3) | 11.2 (10.8-11.3) | 11.0 (10.8-11.3) |
| 225207, first 11 s | v3_s1 | 11.2 | 11.2 | 11.2 |

Both v3_s1 sections agree, and its peak is narrower than the MinGRU
policies'. A frequency that follows the policy is a CLOSED-LOOP
oscillation: a structural mode, or a sensor ringing at its own frequency,
would stay put whatever flies it (the pluck tests, item 9, found nothing
at 9 Hz either). Read: the bike's loop has less phase margin than the
sim's, and each policy crosses -180 deg at its own frequency. Where the
extra lag is (the AHRS path, the Pi's loop, the actuators under load) this
does not say. Of the delays, only the drive and steer servos' were swept
in the sim (12, 24 ms, table in test 2); a delay on the AHRS / observation
side was not. A delay that makes the sim ring at ~9 Hz under fixes_72M AND
~11 Hz under v3_s1 would be the finding; one frequency alone would not.

**What drives the diff (10-26, scratch replay).** The MLP has no memory,
so its observation can be rebuilt from the record (quat, gyro, v_world,
steer through the Pi's RateFilter, psi and its command, its own previous
action) and the policy run on it open-loop: the replayed diff matches
the recorded one at **r 0.9998**, rms error 0.36 against 18 rad/s. So
the Pi computes exactly what the policy says on what the bike measured;
the spikes are not an implementation error between sensing and command.
The diff sits at its +-40 rad/s bound on 8 % of policy ticks.

At 10.5-12 Hz the inputs are one motion, seen by independent sensors:
coherence roll rate (AHRS) / steer rate (steer encoder) 0.96, roll rate
/ v_lat (drive encoders + AHRS) 0.91, roll rate / yaw rate 0.92. Sensor
noise would not be coherent across them; this is the bike moving.
Linearised (finite-difference Jacobian x 9-13 Hz input), the diff's
9-13 Hz content comes from roll rate (~40 %) and yaw (~40 %, net of
yaw_rate and v_lat, which the odometry builds from it: coherence 1.00),
with steer rate -11 % (it opposes). The policy is saturating, so the
linear sum explains only r 0.70.

From the diff command to the motion, bike against its sim twin (120 s,
cross-spectra; closed-loop, so read at the oscillation as P = y/u):

| Hz | diff -> roll rate, bike / sim | diff -> yaw rate, bike / sim |
|---|---|---|
| 3 | 2.07 at +80 / 2.18 at +70 | 2.20 at -100 / 2.64 at -102 |
| 5 | 2.35 at +51 / 2.32 at +59 | (coherence 0.25) / 2.89 at -118 |
| 11 | **1.30 at -20** / 2.29 at +63 | **1.71 at +65** / 2.77 at -118 |
| 13 | 1.07 at -21 / 2.16 at +69 | 1.30 at +59 / 2.63 at -112 |

(gain in deg/s per rad/s of diff, phase in deg.) Below ~5 Hz the bike
answers the diff as the sim does. At 11 Hz roll rate lags the sim by
~80 deg at 0.57x the gain, and yaw rate is nearly inverted. A pure delay
would shift both outputs by the same phase, so this is plant dynamics
the model lacks above ~7 Hz, not only lag. The bike's steer also moves
with the shake (diff -> steer rate coherence 0.95; the sim's 0.08). The
inventory of compliance (user, 10-26) points at the front end: steerer
clearance plus the front axle's rotation, against a chassis, wings and
rollcage that are stiff.

What was planned:

`general_puffer_rake20_80M` (and 72M) trained on 6446d8fa, the plant the
bike is, with `steer_rate_encoder: counts`, the signal the Pi sends. The
pointer has neither. Same session as 174349: engage, no inputs, 30-60 s,
then `run_vs_sim.py`. If the 9 Hz peak goes, the cause was the policy's
training, not the hardware. Known against it: it wobbles more than the
pointer going straight at 1.2 m/s in the sim (`status.md`, next-step 3), so
keep the speed low.

```bash
./scripts/bike.sh bench --torque --move general_puffer_rake20_80M
```

`--move` overrides `control.general_move` for the session (the firmware
gains follow it) and stops at startup if `moves/<name>.npz` is not on the
Pi; `bike.sh sync` carries `moves/`.

### 2. What makes the 9 Hz oscillation, in the sim  [sim only; first pass done]

`run_vs_sim.py --set PATH=VALUE` flies the sim twin with one parameter
changed. First pass, 10-24, run 174349, the pointer, one change at a time
(seed 0 unless noted; seeds 0-2 of the base agree within ~7 %):

| changed | sim 7-12 Hz roll rate / steer target | sim spectral peak |
|---|---|---|
| nothing (base) | 3.2 deg/s / 0.07 deg | 4-5 Hz |
| drive servo command or measurement delay 4 -> 12, 24 ms | (>2 Hz RMS within 1.2x of base) | 4-6 Hz |
| roller half-play 7.8 -> 3, 12, 16 deg | (>2 Hz within 0.75-1.45x) | 4-6 Hz |
| drive Coulomb friction 0.15 -> 0.05, 0.30; rotor inertia x0.5, x2; detent x2; velocity filter 12 -> 24 ms | (>2 Hz within 0.9-1.3x) | 3-6 Hz |
| Pi vs exact steer rate | 3.2-3.9 / 0.07-0.10 | 4-5 Hz |
| steer command delay 12, 24 ms; steer kp x0.6, x1.5; steer armature x2, x4 | 3.6-4.0 / 0.08-0.09 | |
| headset friction x5, x20 | 3.6, 4.4 / 0.10, 0.17 (encoder 0.16, 0.33: stick-slip, shaft above target, unlike the bike) | |
| contact timeconst 0.002, 0.012; sliding friction 0.5 | 3.1-3.7 / 0.07-0.09 | |
| *bike* | *21.9 / 1.73* | *9-10 Hz* |

No single parameter at these sizes gives the sim a 9 Hz mode; the most any
raised the band is ~1.4x. So the bike has something the model does not
contain, rather than more of something it does. Candidates, untested:

- **A structural mode the AHRS sees**: the AHRS mount, the frame, the fork,
  the wings on their linkage. A gyro on a part that rings at 9 Hz feeds the
  policy a roll rate the chassis does not have, and the policy answers it.
  The 45-48 Hz pitch content points the same way. Cheapest test on the
  bike: `bench` (no torque), tap the frame, the AHRS mount and a wing in
  turn, and read the gyro's ring-down frequency from the record.
- **The real TM151's gyro path** (filtering, latency) against `tm151_filter`.
- **The policy**: test 1 changes it and nothing else.

`--set` reaches anything in `load_params()`, so this table can grow.

**Phase (10-24, cross-spectra of the records, scratch analysis).** Yaw
rate against roll rate, run 191558 (rake20) and its sim twin:

| Hz | 1-4 | 5 | 6 | 7-10 |
|---|---|---|---|---|
| bike | +158 to +173 deg, coh 0.93-0.97 | -174, coh 0.64 | +172, coh 0.30 | **+86 to +93**, coh 0.48-0.91 |
| sim | +160 to +175, coh 0.95-1.0 | +179 | -179 | +174 to +175 |

Up to 5 Hz the bike and the sim rock the same way (yaw opposite roll, gain
within ~20 %), so the model's rigid-body motion is right there. Between 6
and 7 Hz the bike's relation jumps by ~90 deg, with coherence dipping
where it does; above it the bike has a second motion the sim has not got.
A phase jump across a narrow band is what a resonance looks like: some
part of the bike has a mode near 6-7 Hz, and the 8-9 Hz shake sits on
its far side. Which part is open.

At the peak, against roll rate (bike / sim at the same frequency):

| | 174349, 9.6 Hz | 191558, 7.8 Hz |
|---|---|---|
| drive differential (a-b)/2, i.e. rollers / sideways | -17 / -75 deg | -29 / -84 |
| drive common (a+b)/2 | coherence 0.68-0.80 only | |
| steer target | +99 / +21 | +80 / -4 |
| steer encoder after target | 157 deg | 133 deg |

The 9 Hz is in the DIFFERENTIAL: drives A and B run in anti-phase, so the
hub stays still and the rollers push sideways. The steer shaft trails its
target by the bench-fitted servo's lag plus ~11 ms (fit: ~119 deg at
9.6 Hz, ~100 at 7.8), about half of which is the record's sampling (read
before write, one 100 Hz tick); the steer servo on the fork is not grossly
slower than benched. On the bike the commands lead the gyro by 55-80 deg
more than in the sim, which follows if the policy's inputs (yaw rate above
all) arrive in a different phase relation; it is not on its own evidence of
a sensor delay.

**Steer or rear wheel? (partial coherence, same two runs.)** At 7-10.5 Hz
the steer target and the drive differential are almost perfectly coherent
with each other (the policy moves both from one state), so the records can
only separate them partly. Coherence with each, and with each once the
other's share is removed (174349 / 191558):

| gyro axis, 7-10.5 Hz | steer | drive diff | steer, diff removed | diff, steer removed | both |
|---|---|---|---|---|---|
| roll rate | 0.89 / 0.92 | 0.93 / 0.96 | 0.02 / 0.01 | **0.35 / 0.47** | 0.93 / 0.96 |
| yaw rate | 0.76 / 0.83 | 0.77 / 0.83 | 0.04 / 0.12 | 0.09 / 0.15 | 0.78 / 0.85 |
| pitch rate | 0.34 / 0.43 | 0.33 / 0.40 | 0.01 / 0.08 | 0.00 / 0.03 | 0.34 / 0.45 |

Roll's 9 Hz carries something the rear wheel's sideways command explains
and the steer does not; never the other way. Pitch is mostly NOT
explained by either command (both together 0.34-0.45), and the forward
(common) drive adds 0.01-0.13 on top. The 38-48 Hz pitch buzz is coherent
with no command at all (<= 0.02), so it is not the commands' 50 Hz steps
either. Closed loop, coherence is association, not cause.

**Is the rear hopping? No.** AHRS pitch angle at 7-12 Hz is 0.016-0.038
deg RMS (both runs), ~0.1 mm between the wheels over the 200 mm
wheelbase. Vertical accel at 7-12 Hz is 0.02-0.03 g RMS; while balancing
it stays within ~0.75-1.35 g and never nears 0 g. The only deep low-g ticks
(down to ~0 g, 174349 at 20.4-21.2 s) are the fall at the end of the first
engage, roll rate climbing to 500 deg/s. Most of the vertical accel is
above 12 Hz (0.6-0.9 m/s2 RMS in 12-50 Hz): vibration, the same buzz as
the 40-48 Hz pitch content. Plot: `traces/bike/261024-174349_run/pitch_hop.png`
(scratch, not reproducible by a tracked script).

**Wheel play reproduces it (10-26).** `wheel_play:` in build_model, absent
from bike_params (so plant_digest and the deploy bundle do not move): a
massless body with limited hinges between a part and what it carries,
sized from free play measured by hand as total sideways travel at the
ground (user, 10-26): front wheel on its axle 2.5 mm (yaw + camber,
+-1.4 deg), headset clearance 2.5 mm (fork tilt, +-0.49 deg about the
fork top, 0.145 m above the contact), rear wheel on its axle 2.5 mm
(+-1.4 deg). Centring spring 0.01 N m/rad, damping 1e-3, walls at
solref 2 ms: GUESSES. One `run_vs_sim.py --no-plot --set
'wheel_play={...}'` each, seed 0, each run's own policy. 7-12 Hz roll
rate RMS (deg/s) at the peak (Hz):

| play | fixes_72M (bike 21.9 at 9.6) | v3_s1 MLP (bike 15.6 at 11.1) |
|---|---|---|
| none | 3.2 at 4.3 | 3.1 at 2.3 |
| rear axle | 3.6 at 4.3 | 3.5 at 3.7 |
| front axle | 9.9 at 6.2 | 16.5 at 9.6 |
| headset | 9.4 at 6.6 | 12.5 (no peak) |
| **front axle + headset** | **21.2 at 7.4** (seed 1: 23.3) | **40.1 at 9.8** (seed 1: 37.5) |
| same, centring 0.1 | 22.9 at 7.6 | 39.4 at 9.2 |
| front + rear | 23.5 at 6.6 | 48.4 at 8.4 |
| all three doubled (5 mm) | 19.7 at 5.5 | 32.4 at 6.6 |

The front's two play joints together make the sim shake at the bike's
size, and the MLP shakes FASTER than the MinGRU, as on the bike (9.8
against 7.4; bike 11.1 against 8.9-9.6). The first sim change of
anything tried here to move the 7-12 Hz band by more than ~1.4x. The
rear's play alone does nothing. Still off: the sim's frequencies sit
1.3-2 Hz low, and more play lowers them further, so the bike's play is
probably smaller or its walls stiffer than guessed; the sim's yaw rate
in the band stays at a third of the bike's (6.8 against 18.4, pointer);
and the MLP overshoots the bike's size 2.6x. Next: sweep travel and wall
stiffness for the frequency, then the yaw. Every row is one seed except
where noted.

**Rough fit (10-26), `traces/drivetrain/wheel_play_fit_1026.yaml`.** 32
grid points, both runs at once; score = squared log ratios of the 7-12
Hz RMS of roll rate, yaw rate, drive A and steer target, plus the peak's
error in Hz squared. Free play alone cannot fit: the frequency barely
moves with travel or wall stiffness (pointer 6.9-8.0 Hz, MLP 8.9-10.0),
and the two policies want opposite sizes. A centring SPRING inside the
play moves the frequency (k 1 -> 30 N m/rad: 8.2 -> 16.3 Hz, pointer),
and the rear axle's play raises the yaw. Best: front axle 1.0 mm,
headset 1.5 mm, rear axle 2.5 mm, each k 4 N m/rad, walls 2 ms --
compliance more than slop, which fits "with a good finger/thumb
torque". rake20 (run 191558) was held out of the fit:

| 7-12 Hz | fixes_72M bike / fit | v3_s1 bike / fit | rake20 bike / no play / fit |
|---|---|---|---|
| roll rate, deg/s | 21.8 / 15.1 | 14.7 / 20.5 | 23.7 / 3.8 / 14.1 |
| yaw rate, deg/s | 18.2 / 11.0 | 19.8 / 13.7 | 19.8 / 4.9 / 10.8 |
| drive A, rad/s | 4.05 / 3.59 | 5.82 / 9.01 | 4.74 / 0.59 / 3.78 |
| steer target, deg | 1.73 / 1.27 | 1.32 / 1.57 | 1.95 / 0.10 / 1.11 |
| peak, Hz | 8.9 / 9.6 | 11.2 / 10.9 | 7.9 / 5.3 / 9.8 |

On the held-out policy the fit lifts every band 4-11x toward the bike,
to 55-80 % of it, but puts its peak at 9.8 Hz against the bike's 7.9: the
bike's two MinGRUs differ by 1 Hz and the fit does not reproduce that.
Yaw stays at 55-70 % throughout. One seed per point; a rough fit, on
the page (`drivetrain_traces.html`) as a sim option. A fourth group,
`rear_stays` (the chainstays' side-to-side and up-and-down flex, hinged
at the midpoint of their front ends, 51 mm ahead of and above the axle)
was added the same day; teleop takes any of it with `--wheel-play YAML`.

**Refit with the chainstays (10-26),
`traces/drivetrain/wheel_play_fit_1026_stays.yaml`.** 12 more points over
the stays' travel and spring, with and without the rear axle's play. No
better on the fitted runs (score 1.54 against 1.52, within one seed),
and the stays stand in for the rear axle play rather than adding to it.
It does generalise better to the held-out run:

| 7-12 Hz | rake20 bike | axle-only fit | stays fit |
|---|---|---|---|
| roll rate, deg/s | 23.7 | 14.1 | 14.8 |
| yaw rate, deg/s | 19.8 | 10.8 | 12.7 |
| drive A, rad/s | 4.74 | 3.78 | 3.95 |
| peak, Hz | 7.9 | 9.8 | **8.6** |

Front axle 1.0 mm and headset 1.5 mm at 4 N m/rad, stays 4.0 mm at 10.
NOT ADOPTED: the stays do not flex that much (user, 10-26), so the
axle-only fit (`wheel_play_fit_1026.yaml`: front axle, headset, rear
axle) stays the one on the page and in the teleop command, the rear
axle being the play that is plainly there. `rear_stays` stays available
for a suspension study; the stays are not as omnidirectional as the
steerer, so expect `travel_mm_vertical` to carry it and the lateral
hinge to stay small. Yaw is still 55-75 % of the bike's either way.

**Logged alike, the sim matches on the drive too (10-26).** A first
reading of the page's spectrum badges said the fitted sim's roller rate
peaks at 4.3 Hz where the bike's is at 9.0: WRONG, an artifact of
comparing the sim's exact joint angle with the bike's 4096-count
encoders, and the sim's true firmware reference with the bike's one
rebuilt from Present PWM. `drivetrain_trace.py` now logs the sim
through the bike's registers (encoder counts, Present Velocity as a
50 ms boxcar in 0.229 rpm steps, PWM in 1/885 duty, the XC330 steer
counts, 100 Hz) and rebuilds its reference the bike's way. Then the
whole-run roller-rate peak is 9.8 Hz (sim) against 9.0 (bike), and band
RMS, 174349 against the fitted sim:

| | 7-12 Hz bike / sim | 12-25 Hz | 25-49 Hz |
|---|---|---|---|
| roller rate (encoders), deg/s | 223 / 183 | 85 / 91 | 44 / 34 |
| roller command (rebuilt), deg/s | 1000 / 692 | 540 / 530 | 1071 / 980 |
| diff velocity measured, rad/s | 2.3 / 1.8 | 0.6 / 0.5 | 0.1 / 0.1 |
| diff command, rad/s | 9.7 / 8.7 | 3.5 / 4.8 | 3.8 / 4.0 |
| steer target, deg/s | 97 / 76 | 42 / 44 | 41 / 31 |
| **steer joint (encoder), deg/s** | **55 / 26** | 14 / 10 | **15 / 4.3** |

The one channel that stays apart is the steer servo's own shaft: 2x the
sim's at the shake and 3.5x above 25 Hz, while its target matches. The
bike's steer shaft moves more for the same command: a softer or laggier
servo loop under load, or the fork's play loading the shaft back --
the steer model is the next thing to look at.

### 2b. Hung off the rollcage, policy flying  [DONE 10-24: the 9 Hz needs the floor]

Run 261024-201156_run_hung: rake20, hung from ONE string (a pendulum, not
the rigid hang below), 31 s engaged, 0 bad reads. 7-12 Hz RMS, hung
against the two floor runs (191558 rake20, 174349 pointer):

| signal | hung | floor |
|---|---|---|
| roll rate | 0.20 deg/s | 24.5 / 21.9 |
| yaw rate | 0.21 deg/s | 20.4 / 18.4 |
| steer target | 0.08 deg | 2.03 / 1.73 |
| drive differential | 0.05 rad/s | 5.3 / 4.8 |

The 8-9 Hz is gone, ~100x down. Instead every channel peaks at **3.5 Hz**:
the steer target ~+-4 deg (1.3 -> 2.7 -> 0.8 deg RMS over the 30 s), the
shaft 65 deg behind it, the hanging bike pitching +-20 deg/s. The rear wheel
spun at a steady ~15 rad/s (input shaft).

**The 3.5 Hz is the hang's own mode, not a steer loop.** In the 5.2 s after
the cut (servos limp; roll +0.3 deg, so not a fall) the steer stops within
~0.2 s, while the bike goes on pitching at 3.5 Hz: envelope 10 deg/s just
after the cut, 7 at +2 s, 0.6 at the end, a ring-down with damping ratio
~0.025 (rough fit). So it is the bike rocking on its string at its
attachment, and under the policy the steer was locked onto it. It is
therefore NOT the same thing as the 10-17 bench's 5.2 Hz or the sim's
pinned 4-4.6 Hz, which have no hanging mode to lock onto. The 2.1 s of
standby before the engage is too still to show anything (0.2-0.4 deg/s).
A slower swing (pitch and roll angle, ~0.3 Hz) shows in the 5.2 s after
the cut, too short to measure. Plot:
`traces/bike/261024-201156_run_hung/hung_5s.png` (scratch).

So the 8-9 Hz shake needs the floor. Per the caveat below, that does not
yet say the contact MODEL is wrong; it says the loop closes through the
wheels on the ground. What was planned:


Does the 8-9 Hz need the floor? Hang the bike so it CANNOT rock: two
straps or a clamp on the rollcage, wheels clear, not one cord (a 25 cm
pendulum swings at ~1 Hz and the drives' reaction torque would rock it).
Then the usual engage, no inputs, 30-60 s:

```bash
./scripts/bike.sh bench --torque --move general_puffer_rake20_80M --tag hung
```

The engage gate wants upright and still, which a rigid hang is; if
preflight refuses, add `--no-preflight`. Expect the rear wheel to spin up
(the sim's drive output sits at 0.5-1.0 of full) and the steer to wander,
possibly whole turns: hands clear, Backspace to stop.

The sim's answer for this setup, chassis pinned 10 cm up, Pi steer rate
(`analysis/policy/steer_pinned.py --policy P --steer0 0 50 --seconds 30
--pi-steer-rate`, 10-24): **no 9 Hz for either policy**, only a 4-4.6 Hz
steer wobble of 0.1-1.6 deg. Reading it:

- **9 Hz there when hung** -> not the floor. It is the policy, the
  actuators, the structure or the sensor, closed through the bike alone.
  The strong result.
- **9 Hz gone when hung** -> it needs the floor, which is NOT the same as
  "the contact model is wrong": the rollers' sideways push only rolls the
  bike through the floor, so the loop the shake rides on is open in the
  air even with a perfect contact model.

The 10-17 bench run (bare steer shaft, AHRS level and still) is the nearest
precedent: a 5.2 Hz steer limit cycle, ~8 deg, which the sim made only with
the bench-fitted servo and the Pi's steer rate, ~20x smaller. Compare the
hung record's 7-12 Hz band with the floor runs' (table under test 1).

### 3. Re-measure the reassembled drivetrain  [coast + creep DONE 10-24]

Captures `traces/drivetrain/261024-202033_coast` and `261024-202227_creep`,
bike hung from the rollcage, ids 101/102 signs +1/-1 as on 09-13 (servo
101 = left, now drive_b). `drivetrain_fit.py --session 261024 coast detent`,
at the servo shaft:

| | 09-13 bench, 101 / 102 | 10-24 bike, 101 / 102 |
|---|---|---|
| inertia, common | 2.78 / 2.75e-3 kg m2 | 2.90 / 2.87e-3 |
| inertia, diff | 2.27 / 2.41e-3 | 2.30 / 2.22e-3 |
| Coulomb, common | 0.167 / 0.217 N m | 0.163 / 0.177 |
| Coulomb, diff | 0.157 / 0.200 | 0.114 / 0.114 |
| viscous, common | 0.019 / 0.002 N m s/rad | 0.038 / 0.032 |
| viscous, diff | 0.030 / 0.035 | **0.165 / 0.156** |

The sides now agree within ~8 % (the 09-13 "B ~25 % higher" is gone), and
the inertia is unchanged. The differential's viscous drag is ~5x; with its
lower Coulomb, total drag in diff at 3 rad/s is ~0.6 N m against ~0.25-0.31
(Coulomb and viscous trade off in a decay fit, so read the sum, not either
alone). The model carries one `coulomb_friction` (0.15) and no servo-side
viscous term of this size.

Detent. `drivetrain_fit.py detent` folds the diff creep onto 7.5 deg and
splits the push into a part that flips with direction ("friction-like") and
one that does not ("conservative"). Folded, 10-24 read a conservative swing
of 0.46-0.54 duty p-p against 0.04-0.17 on 09-13, which was first written up
here as a spring-like hill the model lacks. **That was wrong.** Unfolded
(push against the whole ring-vs-hub angle, 10-24 scratch analysis; plot
`traces/drivetrain/261024-202227_creep/unfolded_vs_0913.png`):

| | 09-13 | 10-24 |
|---|---|---|
| strongest period in the angle-dependent push | 7.5 deg (0.020-0.031) | **~15 deg** (14.7-15.4; 0.032-0.054) |
| 7.5 deg component | 0.020-0.031 | 0.001-0.004 |
| angle-dependent part, RMS | 0.025-0.031 duty | 0.047-0.057 |
| push in the motion direction, 3 % creep | ~0.17-0.30 | ~0.25-0.55 |
| same period at 1 % and 3 % | yes, 7.5 | yes, 15.4 and 15.1 |

The push is a stick-slip SAWTOOTH, a slow rise and a sharp drop, every
~15 deg in both directions, the ring-vs-hub speed swinging ~0-400 deg/s.
A sawtooth drops at a different angle in each direction, which the split
books as "conservative": the folded spikes are that artifact, and the "12x"
was a peak-to-peak of them (by RMS it is ~2x). The same period at both
speeds says it is fixed in ANGLE (a tight spot, the plan_creep docstring's
test), not the velocity loop's own cycle. So the reassembled differential
is stickier, at twice the spacing; what part has the 15 deg period is open.

The model's detent is a friction bump (`detent.friction` 0.096 N m, 7.5 deg,
2.5 deg wide), so this IS expressible. Sim twin of 191558 (rake20), 7-12 Hz:

| sim detent | roll rate | yaw rate | drive A / B | sim peak |
|---|---|---|---|---|
| as now | 4.1 deg/s | 5.3 | 0.63 / 0.65 rad/s | 4.1 Hz |
| period 15 | 3.7 | 4.7 | 0.57 / 0.59 | 4.1 |
| period 15, friction x2 | 4.4 | 5.4 | 0.59 / 0.76 | 4.1 |
| period 15, x2, width 5 | 5.2 | 6.5 | 0.71 / 0.81 | 4.1 |
| period 15, x4, width 5 | **9.0** | 10.8 | 1.21 / 1.30 | 4.3 |
| period 15, x2, width 5, Coulomb 0.14 | 5.4 | 6.6 | 0.73 / 0.86 | 4.3 |
| *bike* | *24.5* | *20.4* | *4.9 / 6.0* | *7.8-8.8* |

The strongest effect of anything tried, and still the wrong frequency: a
stickier differential makes the sim shake more but at its own ~4 Hz, not
the bike's 8-9. (Mapping duty to the model's N m for a sawtooth is not
clean, so which row is "measured" is not settled; x4 is likely above it.)

**Direction.** The 15 deg period and the sawtooth's size are the same both
ways; the push is not: "back" diff takes 0.350 / 0.393 duty against "out"'s
0.310 / 0.342 (1 % / 3 %), 13-15 % more, on each servo separately and at
both speeds. 09-13 leant 4 % the other way. A mild handedness, new with the
rebuild.

**Map and two more creep speeds (10-24, 261024-210224_map,
261024-210446_creep).**

- The 4-speed creep corrects the "15 deg, fixed in angle" above. The diff
  ripple period is 7.49 deg at 6 % (both ways), 15.1 at 3 %, 7.6 / 15.4 at
  1 % (one per direction), too stuck to read at 0.5 % (76 % of the time
  stopped). So the notch is still the tooth's 7.5 deg; at some speeds the
  stick-slip skips every other one, which is the 15 deg.
- `map` at the servos' own Position P 460 did not give a static map: the
  shaft lagged its goal 10-20 deg of ring-vs-hub and moved in jumps of 7.1
  and 13.7 deg (about one and two notches), so most readings are the loop
  winding up while stuck. What it does give: breakaway in diff ~0.29-0.31
  duty, in common ~0.17-0.19, common steadier. `--position-gains P:I:D`
  added for a stiffer rerun.
- **Rerun at Position P 4000 (261024-211612_map): a real static map.** The
  shaft followed its goal within 1.4-1.7 deg median (3.2 max) of
  ring-vs-hub, no jumps, 40 deg covered both ways, no buzz; gains and mode
  restored. `drivetrain_fit.py --session 261024-2116 map`:

  | | friction (out - back)/2 | angle-only (out + back)/2 |
  |---|---|---|
  | diff (rollers) | 0.21 duty mean, dipping to 0.15-0.17 once per notch | **~+-0.09 duty, period 7.5 deg** (troughs 30 deg apart over 4 cycles) |
  | common (wheel) | 0.11 duty, flat | flat (sd 0.024) |

  So at rest there IS a push that depends on angle alone, once per tooth,
  in the differential only: the creep's split exaggerated it ("12x"), the
  retraction above went too far. In the model's units (detent.friction's
  own conversion, 0.09 duty = 0.096 N m on ring_spin) it is ~+-0.1 N m on
  ring_spin; the model's detent has friction only.
  Resolution: at rest in position mode Present PWM moves in steps of P x
  one encoder count, 31 raw = 0.035 duty at P 4000 (velocity mode: steps
  of 1), so each reading is to 0.035 duty; the period is solid, the
  amplitude good to ~one step.
- **The hill in the sim (10-24).** `drivetrain_model.detent.hill_nm` (default
  0, not in `bike_params.yaml`, so no digest moves) adds the angle-only push:
  `-hill_nm sin(2 pi (phi - phase)/period + hill_phase_deg)` on ring-vs-hub.
  Sim twin of 191558, 7-12 Hz:

  | hill | roll rate | yaw rate | drive A / B | sim peak |
  |---|---|---|---|---|
  | 0 (as now) | 4.1 deg/s | 5.3 | 0.63 / 0.65 | 4.1 Hz |
  | 0.05 N m | 3.9 | 4.9 | 0.61 / 0.59 | 4.1 |
  | 0.1, phase 0 / 90 / 180 / 270 | 4.7 / 5.7 / 5.1 / 5.6 | 5.9-7.1 | 0.69-0.82 / 0.72-0.84 | 4.1-4.5 |
  | 0.1, detent friction x2 | 5.3 | 6.6 | 0.74 / 0.81 | 4.1 |
  | 0.2 | 7.5 | 9.1 | 0.97 / 1.05 | 4.1 |
  | 0.4 | 13.5 | 16.3 | 1.75 / 1.68 | 4.3 |
  | *bike* | *24.5* | *20.4* | *4.9 / 6.0* | *7.8-8.8* |

  At the measured size (~0.1) it adds 15-40 %; at 4x it reaches half the
  bike. Never at the bike's frequency: the sim keeps its own ~4 Hz at every
  size and phase. So the detent, friction and hill both as measured, is a
  contributor to how much the sim shakes, not the source of the 8-9 Hz. (The
  duty -> N m conversion is detent.friction's own, stall torque 1.6 N m;
  a 2x error there would not move the frequency either.)
- **But the map's angle is the SERVO's (10-24, plot
  `traces/drivetrain/261024-211612_map/map_phase.png`, scratch).** Even at
  P 4000 the diff pass is still stick-slip, finer: the push climbs over
  ~5.3 deg of measured angle and drops ~0.27 duty as the shaft jumps
  ~2.5 deg, once per 7.5 deg notch, both directions. Common never sticks
  and shows no ramp. The only angle sensor is the servo encoder, so a climb
  of push with servo angle while the notch is stuck is whatever STRETCHES
  between servo and notch (belt, servo gearbox), not the notch's shape; and
  out and back climb their ramps in opposite directions, so (out + back)/2
  mixes two windups. [Superseded by the film below: no stretch, so both
  passes' readings are real away from the jumps, and the angle-only part
  is a sawtooth, Next item 2.] **The "+-0.09 duty angle-only sine" is therefore not
  established as notch geometry** (the hill sweep above stands as a
  negative: even 4x it does not make 8-9 Hz). Aligning the passes by GOAL
  instead of measured angle shifts them ~3.1 deg (lag +1.4 out, -1.7 back)
  and changes the split visibly, which is how sensitive it is.
- **What the ramps do measure, if they are stretch: the drivetrain's
  compliance.** Slope 0.046-0.053 duty/deg of ring-vs-hub, i.e. ~13-15
  N m/rad at each servo shaft (stall 1.6 N m per duty). The model has none:
  the rotor's inertia sits rigidly on the input shaft. A 2.3e-3 kg m2 rotor
  on 11-14 N m/rad rings at 11-12 Hz. Hung, the roller side is light and
  free; on the floor it is held by the bike, so the rotor has something to
  ring against. That fits "needs the floor" and "nothing in the model makes
  it", and lands near the bike's 8-9 Hz once everything else hangs on it.
  An inference: not tested in the sim, and the ramp could instead be the
  notch's own climb. To tell them apart: film a roller at 240 fps during a
  1 % diff creep; a roller that stops while the servo turns is stretch, one
  that climbs smoothly is geometry. In the sim: a spring between rotor and
  input shaft (a model change).
- **Filmed (10-24, `traces/first20sec.mp4`, `second20sec.mp4`, record
  `261024-221037_creep_film`, 1 % diff creep, hung): no stretch.** Tracking
  the blue tape on the belt and on a roller: in the window where both are
  visible they jump in the same video frame (4/4, +-1 frame) and sit nearly
  still between (drift 0.8 px of belt and 1.7 px of roller per hold, against
  6.7 and 10.1 px jumps). The servo encoders agree: 87-88 % of the advance
  is in the jumps, 0.7-1.3 deg of ring-vs-hub creeping per stuck stretch,
  while the push builds to the 0.36-0.41 duty breakaway. On a 13-15 N m/rad
  spring that push would have wound the servo ~6-7 deg ahead of the belt.
  So servo, belt and roller move as one piece, stiffer than that by ~10x:
  **the compliance lead is ruled out**, and the map's ramps are the notch
  itself, resistance building over ~5 deg of travel then letting go,
  whichever the direction. Jump per cycle: 7.45 deg (+, one notch) and
  12.8 deg (-, skipping), the handedness again. (Video timing: the files are
  30 fps exports of "1/4 speed", i.e. 120 fps capture, 8.3 ms a frame;
  the video's ~52 frames between jumps = 0.43 s against the record's 0.49.)
- **The roller's slop, seen at each chunk (same film; plots
  `belt_vs_roller.png`, `roller_jumps.png` in the record's directory,
  scratch).** On the 4 jumps with both tapes in view, the belt rises
  smoothly over ~40-50 ms with no overshoot (peak 1.00-1.03 of its step);
  the roller gets there first, by ~20-25 ms, falls back to ~0.8 of its step
  by ~40 ms, then moves with the belt and settles by ~100 ms. So the roller
  moves against its gear by ~20 % of the jump, ~3 deg of roller (tape shift
  read as R sin theta, R ~50 px), inside its +-7.8 deg play, and is pulled
  back quickly: one rebound, no ringing, more like reaching the end of the
  play than a soft centring spring (the model's centring spring, a GUESS,
  is critically damped at ~10 Hz). Rollers unloaded, in the air; a
  projected tape centroid at 8.3 ms a frame, so the ~3 deg is rough. The
  other direction (second video, no belt in view) rises over ~50 ms and
  overshoots 10-15 %.
- **One notch or two, by ear and by record (same film, audio via ffmpeg;
  `chunk_sounds.png`, scratch).** Jumps measured at the servo encoders
  fall in two sizes, ~7.5 and ~13.6 deg of ring-vs-hub, almost nothing
  between (3 of 131 at 1-3 %); more doubles the faster the creep (3 %:
  nearly all) and in the - direction (1 %: 20/26 against 11/38 for +). All
  61 loud chunks in the two videos sit within 60 ms of a recorded jump (the
  first video is the + pass, the second the -; this also confirms the
  120 fps timing). Doubles are louder (median 239 / 294 x the noise floor
  against 178 / 170 for singles) and are often two clicks: a second one
  25-45 ms after the first in 15 of 31 doubles, in 0 of 30 singles.
- **The sim's detent against the jumps (10-24; scratch script saved as
  `replay_jumps.py` beside the record).** The filmed creep's commands
  through the testbed (`drivetrain_fit._simulate`), jumps counted the same
  way as the bike's. The model as it is (0.096 N m, 2.5 deg): all singles
  both ways (49 / 48), never a double. A grid over friction 0.096-0.38,
  width 2.5 / 5, hill 0 / 0.1 at four phases: the sim is ALL singles or
  ALL doubles in a pass, flipping at ~0.21-0.27 N m (2.5 deg wide); the
  bike mixes (+ 25 / 13, - 5 / 21). Friction 0.21 + hill 0.1 at phase 90
  gives + all single, - all double: the handedness, without the mix. The
  mix needs notches that differ from one another, which the model's
  identical notches cannot give. In the floor twin of 191558 that detent
  raises the 7-12 Hz roll rate 4.1 -> 6.3 deg/s, still at 4.1 Hz (bike
  24.5 at 7.8).
- **Roller slop, pushed and released by hand (10-24, `traces/rollerslop.mp4`,
  1/8-speed export = 240 fps capture; plot `traces/rollerslop_snaps.png`,
  scratch).** Roller tape measured against the belt tape so the swinging
  bike cancels. Four push-and-release cycles: pushed ~9-10 px (960-wide
  frame; ~8 deg of roller at R ~70 px, rough) and held, then released:
  back to the same rest level every time, 90 % in 12.5-17 ms, first peak
  at 17-21 ms overshooting 4-17 %, dipping to 0.61-0.85 at 42-50 ms,
  within 10 % by 54-188 ms. A second-order fit to the first peak gives
  wn ~170-220 rad/s (28-35 Hz), damping ratio ~0.5-0.7, so with the
  model's roller inertia 2.31e-7 kg m2 a centring spring of
  ~0.7-1.1e-2 N m/rad (model: 1.0e-3, GUESS, i.e. 7-11x softer) and
  damping ~4-7e-5 N m s/rad (model 2.9e-5). The dip after the overshoot is
  not second-order: something else (bushing friction?) is in it. Not
  promoted into bike_params: one video, by hand, unloaded.

What was planned:

The drivetrain came apart and went back together, so its 09-13 fits are
for a different assembly: the per-side friction (bench: servo 102's side
~25 % higher, `0.16:0.21`), the roller play, the detent. Bike upside down
or held up, the rear wheel clear of everything: the same Station A
captures, by the same script, on the Pi. Stop `run_bike` first (it holds the
port). Ids and signs as at 09-13 (A = 101), so the new fits compare like for
like:

```bash
cd ~/aow-bike-sim
P=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB8HMQ3-if00-port0
B="$HOME/venv/bin/python analysis/servo/drivetrain_bench.py"
$B jog      --port $P --ids 101 102 --signs 1 -1 --note "assembled bike, held up"   # common = wheel, diff = rollers
$B step     --port $P --ids 101 102 --signs 1 -1
$B reversal --port $P --ids 101 102 --signs 1 -1
$B chirp    --port $P --ids 101 102 --signs 1 -1
$B coast    --port $P --ids 101 102 --signs 1 -1
$B creep    --port $P --ids 101 102 --signs 1 -1
```

Each waits for Enter before torque and arms a 100 ms bus watchdog. Pull
`traces/drivetrain/` back with rsync, then

```bash
python analysis/servo/drivetrain_fit.py --session <YYMMDD of the captures>
```

(`--session` added 10-24; without it the fit reads the 09-13 set). The
09-13 coast for comparison: Coulomb 0.167 / 0.217 N m common, 0.157 / 0.200
diff, servo 101 / 102. A pair goes in as `coulomb_friction: [drive_a,
drive_b]`, which is now [102 right, 101 left]. Not yet run on the Pi: its
frame rate at the default `--rate 500` is the first thing the summary says.

### 4. The steer on the built fork  [bike upside down, weighted]

`analysis/servo/steer_response.py` drives the goal open loop; it was run
on a bare shaft (10-17). The headset friction in the model scales with the
axial load (`headset_friction_nm` + `_per_n` x thrust), so upside down with
the front wheel weighted to its share of the bike reproduces the headset's
load, as the earlier weighted tests did. It does not reproduce the tyre's
scrub on the floor, which only an on-floor test has. `steer_friction.py` maps the friction by angle
(the bushing's tight spot). Both feed `actuators` / headset friction in
`bike_params.yaml`.

### 5. The AHRS mount  [low priority]

`source: design` since the v2 bike (the TM151 on its own holes, the CAD's
+45 deg); preflight accepts it since 10-25, which before that blocked
anything not `measured` and was passed by `--bench`. The "2-3
deg held upright" once written here was a hand-held standby reading, which
is not upright, so it is no evidence. What the balancing records give
instead: the median roll the AHRS reads while the bike balances itself is
+0.6 to +1.6 deg across the six engages since the swap (+0.56 in the 50 s
no-input one). That is mount offset plus any sideways CoM offset, which the
policy cannot tell apart and neither can this. Worth a jig only if the
drift turns out to follow it.

**Why no jig can settle roll.** A reversal (read, turn the bike 180 deg
about vertical on the same marks, read again: half the sum is the mount,
half the difference the floor) needs the support to repeat the chassis's
attitude to the accuracy wanted, ~0.17 mm per 100 mm of base for 0.1 deg.
Pitch has a base, the two wheel contacts. Roll has none: whatever props
the bike sets its roll, and the upright that matters is where it BALANCES
(CoM over the contact line), which no prop gives. So roll is known no
better than the CAD sets it, and the AHRS is the finest level on the bike.

**Backup, not queued: train over the mount error instead.** Estimate the
error as a budget and randomise it per episode. What training already has:
`sim_ahrs` draws a fixed accelerometer misalignment per power-on, uniform
+-0.5 deg per axis (`MISALIGN_DEG`, the TM151 datasheet's "< 0.5 deg"),
and under `tm151_filter` that tilts the filter's gravity reference, so
every policy since 10-09 has seen a roll and pitch offset of up to 0.5
deg. What it lacks is the build's share, all of it unmeasured:

| term | how to estimate |
|---|---|
| the sensor's own axes against its case | datasheet, already in (0.5 deg) |
| the TM151's holes against the tabs | print hole-position error over the hole spacing (CAD) |
| the tabs' face against the drive frame | print flatness / warp over the face's length |
| the drive module against the chassis datum | assembly stack, screws and press fits |
| the wheels' contact line against the chassis | already randomised elsewhere? check `randomization:` before adding |

Sum them (root-sum-square for independent errors, straight sum for a
bound), and widen the draw to that. Two cautions from the 08-28 attempt
(`sensor-workstream.md`, "Randomising the AHRS error parameters"): keep
the floor at the datasheet value rather than a log-uniform spread whose
median sits below the bike, and judge the result with `chatter.py`, which
`_score` alone missed. The balancing records' +0.6 to +1.6 deg median roll
is an upper bound on the mount's share (it includes any CoM offset), so a
range that does not reach ~1.5 deg is narrower than the evidence allows.

### 6. Standstill drift, measured  [the target floor, a camera]

Not now (needs floor space and a camera). By eye, 174349 travelled ~1 m
in 50 s. Tape a start mark, film from above, compare with the record's
`pos`. Answers whether the bike
wanders or the estimate does -- the sim's standstill is 0.11-0.21 m/s for
these policies (`status.md`), so drift is expected; how much is real is not.

### 7. Contact, on the whole bike, on the target floor

The subsystem rigs (`pre-assembly-bench-checklist.md` F1-F3, the drop rig)
were for a bike that did not exist. Now:

- **Drop rebound (F2, whole bike).** Torque off, drop from a few measured
  heights onto the floor, film at 240 fps, the AHRS recording
  (`bench/force_drop.py --ahrs` logs the TM151). The sim twin is the
  whole-bike model dropped the same way. Label results by **sink at bike
  weight in mm**, never by `dampratio` (`floor_sweep.py` does).
- **Traction limit (F3, whole bike).** Pull sideways and forwards with a
  spring scale until it slides, drives holding, and read the peak.
  `./scripts/bike.sh hold` (added 10-24) torques every servo with the policy
  disabled: drives at zero velocity, steer held at the angle it is at,
  wings live. Enter / Y re-captures the pose, Backspace goes limp. (Until
  10-24 a standby tick wrote nothing, so the wings ignored the station
  except under the policy; standby now sends the wing goal on its own.)
- **The record as the identification.** Every balancing session is
  already a closed-loop trace. Running `run_vs_sim.py` against the sim at
  a few `contact_solref` / friction points (as `floor_sweep.py` varies
  them) asks which floor makes the sim's spectra match the bike's. Only
  after tests 1-3, or a drivetrain mismatch will be fitted as contact.

### 8. Queued for the next retrain, together

- `k_roller: -2.4` in `bike_params.yaml` AND revert the `onboard.yaml`
  A/B swap, in one change (either alone breaks the sideways sign).
- The drivetrain as re-measured in test 3: Coulomb (coast: ~0.16-0.18
  common, 0.114 diff per servo, the sides now equal), the detent as
  calibrated against the jump sizes (the best-effort set in Next: flat
  friction, sawtooth; its kinetic fraction 0.15 is a fitted GUESS;
  `traces/drivetrain/drivetrain_sawtooth_1024.yaml` holds it with the
  roller spring), the roller centring spring (~1e-2 N m/rad against the
  1e-3 GUESS; x10 cut `smooth_temporal`'s chatter score 0.818 -> 0.598 in
  `slop_sensitivity.py`, so check chatter after); the mount from test 5
  if it is ever run.
- Train with `steer_rate_encoder: counts` (the pointer did not).

Each moves the plant digest, so every policy becomes provisional at once;
batch them. Tracked with the wheel play and the GPU status of each in
`next-retrain.md` (10-26). **Trained 10-26** (`rl_general_v4.yaml`): the
sawtooth overlay with the roller spring, the wheel play fit, the Pi's steer
rate. Left out on purpose: `k_roller` + the swap revert (so old and new
policies fly on one `onboard.yaml`) and the Coulomb (0.15 kept), the mount.

### 9. Fly the v4 retrain  [DONE 10-26: the 9 Hz is gone]

`general_puffer_a40_v4_80M` (trained with the sawtooth detent, the roller
spring and the wheel play fit; `next-retrain.md`) and the pointer, back to
back on the same floor, no inputs, each ended by the swing-sit lie-down:
runs 261026-154111 (v4, 73 s engaged) and 261026-154338 (pointer, 55 s).
`run_vs_sim.py` on each, the bike column (engaged stretch less 0.5 s at
each end):

| 7-12 Hz RMS | pointer 174349 (10-24) | pointer 154338 | **v4 154111** |
|---|---|---|---|
| roll rate, deg/s | 21.9 | 16.4 | **4.0** |
| yaw rate, deg/s | 18.4 | 19.7 | 7.9 |
| steer target, deg | 1.73 | 1.40 | 0.115 |
| drive A / B command, rad/s | 4.1 / 5.8 | 3.1 / 4.9 | 0.37 / 0.63 |
| roll-rate peak, Hz | 9.6 | 10.0 | 5.3 |

Roll rate by band (Welch, 2.56 s segments), and the odometry:

| | 0.5-2 Hz | 2-7 Hz | 7-12 Hz | median \|v\|, m/s | net travel |
|---|---|---|---|---|---|
| pointer 174349 | 7.7 | 13.3 | 21.0 | 0.074 | 0.61 m in 50 s |
| pointer 154338 | 10.1 | 12.2 | 15.7 | 0.077 | 0.24 m in 55 s |
| v4 154111 | 6.5 | 13.1 | 3.9 | 0.055 | 0.85 m in 73 s |

- **The shake was the policy's training, not a property of the bike.** The
  same hardware, minutes apart, 4x less roll rate and 8-12x less steer and
  drive command in the band. Test 1's rake20 did not train on the play;
  this one did. What is left below 7 Hz is the pointer's own (2-7 Hz equal),
  now the biggest thing in the spectrum.
- **The sim with the play now overstates it; without the play it matches.**
  v4's twin (`run_vs_sim --seed N`, `--set wheel_play={}` for none), 7-12 Hz
  RMS, seeds 0 / 1 / 2:

  | | roll rate, deg/s | yaw rate, deg/s | drive A cmd, rad/s |
  |---|---|---|---|
  | bike 154111 | 4.0 | 7.9 | 0.37 |
  | sim, its training plant (detent + play) | 7.8 / 7.6 / 8.1 | 9.0 / 8.5 / 9.5 | 0.99 / 0.92 / 0.82 |
  | sim, same without the play | 3.2 / 3.3 / 3.3 | 3.9 / 4.1 / 3.9 | 0.37 / 0.38 / 0.32 |

  The pointer on the play fit gives 15.1 against its bike's 16.4, so the
  fit holds for the policies it was fitted on (the pointer and the MLP,
  174349 / 225207) and not for one trained against it. Not tested, why:
  three closed-loop runs of two policies do not pin play, spring and
  damping apart, and a wall-and-spring hinge rings where real play with
  friction in it may not. Next: refit with 154111 as a third constraint.
- **Drift, one sample each:** the odometry puts v4 at 0.85 m net in 73 s
  against the pointer's 0.24 in 55 s; `standstill.py` in the sim had v4
  drifting less. Not settled by one run each; test 6 is the measurement.
- **The swing-sit across a policy swap** (user: used at the start and end of
  the runs, the first time with a different policy each side). The records:
  v4 lay down at the end of its run (engaged -> sit -> lying -> cut), and the
  pointer's process, started next, got it up from there, stopped PROPPED and
  took the Enter hand-off (engaged at roll 1.9 deg) -- the first hand-off
  from propped on the bike -- then lay it down again. Lie-downs clean on both:
  peak roll rate 80 (v4) and 67 deg/s (pointer) through the hand-off.
- **v4's own engage** is from standby in its record (no sit or propped state
  in that process), leaning 5.4 deg: caught, with the steer to full lock both
  ways (+-45 deg), roll -7.5..+8.3 deg and 116 deg/s in the first second.

The page: `analysis/policy/plots/drivetrain_traces.html` has both runs
(v4 beside its own plant, one sim; the pointer's three as before).

### 10. The steer and front axle stiffened  [flown 10-28, v4]

The user's hardware change: the steerer shimmed, and a nut on the front
axle, which now rolls a bit tight. No parameter moved. Three records:
261027-191736 (the steer fitted 90 deg off, cancelled; never engaged; its
stamp is the Pi's clock before it synced), 261028-212855 (436 s of play
and driving on a few surfaces) and 261028-214417 (132 s at rest on the
test floor). The user's impression: forward very smooth, backward fine
but with speed wobbles, where before it struggled to go backward at all
-- so much that it was barely driven backward before ("I could tell it
was going to fall over"), and this time at full speed, hence the
reversing fall. On a slippery wood floor the rear slides much more; it
drives nicer on grippy or soft floors.

**At rest on the test floor, v4 before (154111) and after (214417):**

| | before | after |
|---|---|---|
| engaged, s | 73 | 131 |
| roll rate 7-12 Hz / 2-7 Hz, deg/s | 4.0 / 13.6 | 4.7 / 11.8 |
| yaw rate 7-12 Hz / 2-7 Hz, deg/s | 7.9 / 16.5 | 5.9 / 11.5 |
| steer target 7-12 Hz, deg | 0.115 | 0.109 |
| steer tracking error (target - encoder), RMS deg | 2.7 | 3.5 |
| steer current, mean abs, mA | 20 | 34 |
| odometry: net drift / path, m | 0.85 / 5.6 | 0.17 / 6.3 |

The 9 Hz shake stays gone, the slow sway is a little lower, and the drift
is a fifth of before in nearly twice the time (one run each; test 6 is the
measurement).

**The steer's friction about doubled, and the model now matches it.** A
fit, over every sample with the steer moving faster than 0.5 rad/s, of
the XC330's current to `Fc sign(w) + B w + J dw/dt + c` (a scratch
script; the steer's angle differenced for w):

| record | Coulomb, mA | as torque (datasheet kt 0.91), mN m | viscous, mA per rad/s |
|---|---|---|---|
| 154111, 163044, 175020 (before) | 13.1-15.8 | 12-14 | 13-15 |
| 214417, 212855 (after) | 30.5-34.0 | 28-31 | 19 |

(A first cut, half the +/- difference of the mean current, gave 27-36 ->
53-61 mA: it lumped the viscous part in.) The model's steer friction --
gearbox plus headset, `headset_friction_nm` + `_per_n` x the sim's
3.8 N of thrust -- is 31 mN m while the steer moves (headset ~20, gearbox
~11; v4 at rest, 2 seeds). So the model sat ~2x above the bike before
the shim and at it after: **no refit**. Rough because: kt is the
datasheet's, unmeasured on this unit (the fixture put its torque per duty
at 0.62-0.69 of the 0.80 rated, which would scale both rows down
~20 %); the fit takes any floor torque on the fork that follows the
steer's direction as friction; and the mode-5 current deadband (16.5 mA)
does not carry over -- subtracted, the before rows go negative. The same
fit on the sim is no check: its steer moves above 0.5 rad/s 5-16 % of the
time against the bike's 29-49 %, and the coefficients swing in sign
seed to seed. Test 4 separates headset from floor if it ever matters.

**The front axle does not show.** The drives log Present Load (a fraction
of max torque), and its mean is set by the balancing, not by rolling:
10-15 % at rest and 15-17 % driving forward, before and after alike.
Rolling drag this small is under that.

**Backward wobbles; forward does not.** 212855, stretches of at least
1 s under a steady command, first 0.5 s dropped:

| | s | cmd, m/s | speed, m/s | yaw rate 2-7 Hz | roll rate 7-12 Hz | steer target pk |
|---|---|---|---|---|---|---|
| forward | 5.1 | +0.66 | +0.65 | 10.9 | 2.1 | 3.7 Hz |
| backward | 21.1 | -0.76 | -0.45 | 57.2 | 8.6 | 3.6 Hz |

Backward reaches 60 % of its command and yaws at ~3.6 Hz. Before the
change there are only 6.5 s of backward driving across four records, in
1-3 s pieces (yaw 2-7 Hz 28-73 deg/s, speed 71-87 % of command): it was
barely reversed then because it felt about to fall, so the records cannot
compare; the user's impression is that it improved. One of the three falls in 212855 was reversing, at
168.5 s with the command at -1.2 m/s.

**Reversing fast, the steer swings slowly; under it, the rear wobbles at
4.7 Hz.** The user sees the steering swing back and forth when reversing
quickly, "like it is about to fall", and settle when slowed. The record
has it: commanded past about -0.7 m/s, the steer swings 25-57 deg peak to
peak within a second (5-14 at -0.5), at 1 Hz or under, and the steer
TARGET swings as much or more, so the policy is moving it, not the fork
flapping past it. The reversing fall at 168.5 s is the fork going round:
steer +3 -> 39 -> 58 -> 85 deg in 0.4 s, then the bike over.

The faster wobble, the one in the yaw, is not in the steer.
Cross-spectra against the yaw rate at its peak, 212855 backward (14 s in
stretches of 1.8 s or more):

| at 4.7 Hz | 2-7 Hz RMS | coherence with yaw |
|---|---|---|
| yaw rate, deg/s | 56 | -- |
| roll rate, deg/s | 51 | 0.89 |
| drive differential cmd (A - B), rad/s | 24 | 0.97 |
| steer target / encoder, deg | 4.6 / 3.5 | 0.01 / 0.03 |

The fork is not in it: the bike yaws and rolls together with the rear
omni-wheel's sideways (roller) drive. Reversing, the front's trail is
behind the wheel's direction of travel, an unstable caster that the steer
servo holds; the policy balances on the rear instead. The sim does the
same thing at half the size: v4, -0.8 vs +0.8 m/s, mu 0.9, 2 seeds,
coherent with yaw at 3.9 Hz:

| sim | yaw 2-7 Hz | drive diff cmd 2-7 Hz | steer target 2-7 Hz |
|---|---|---|---|
| +0.8 | 14-15 | 3.7-3.8 | 5.5-5.6 |
| -0.8 | 19-21 | 14-15 | 4.5-4.9 |
| -0.8, no wheel play | 24-27 | 12-14 | 3.4-4.0 |
| bike, +0.66 / -0.76 | 11 / 56 | 4.9 / 24 | 8.1 / 4.6 (the steer incoherent backward) |

Floor friction, the sim at -0.8: yaw 2-7 Hz 20 / 17 / 10 / 4-6 deg/s at
mu 0.9 / 0.6 / 0.4 / 0.3 -- less on a slipperier floor in the sim, where
the user finds the slippery floor worse; which floor each of 212855's
stretches was on is not in the record. So the mode is the policy's own
reversing loop through the rear, and something on the bike makes it ~2x
the sim's. The physical "shimmy damper" would be damping in the rear's
sideways motion, which in the model is only what the drivetrain carries
(servo friction, the roller play and detent, the drive delay); the
contact has no tyre compliance or relaxation length, so a classic tyre
shimmy cannot form in the sim at all.

**The rear skids on drive reversals (user: "the drive gets traction and
then when it tries to change direction it skids along the floor").** The
speed estimate is the drive encoders', the rear wheel's own speed, so it
cannot see a skid directly. Against the accelerometer, 1-10 Hz: at rest
the wheel's dv/dt and the body's x acceleration are the same size (0.9
and 1.1 m/s2 RMS); reversing faster than -0.7 the wheel's is 4.5 against
the body's 2.1, correlation 0.08. A wheel changing speed twice as hard as
the bike follows it is what a skid looks like; the accelerometer is noisy
(the drivetrain's own vibration), so this is suggestive only.

**How much of it the policy decides: a matrix in the sim.** Three
policies, four plants, all with the steer and AHRS each policy expects;
the shipped floor (mu 0.9, sink 0.135 mm); a step command at 2 s, 10 s
flown, 2 seeds (scratch script). yaw 2-7 Hz deg/s / drive differential
2-7 Hz rad/s; FELL = rolled past 45 deg, each one a fork that went round
(steer peak to peak ~200 deg) about 1.5 s after the step:

| reversing -0.8 / -1.2 m/s | A: v4's plant (detent + all play) | B: "stiffened" (rear play only) | C: v4 detent, no play | D: old detent, no play |
|---|---|---|---|---|
| v4 (trained on A) | 18-21/14-15, 16-20/7-9 | 21/16, 7-8/4 | 24-28/12-14, 18/8 | **58-60/24-25, 61/25** |
| rake20_80M (trained on D) | 10-12/6-7, FELL 2/2 | 5-7/3-4, FELL 1/2 | 4-5/3, 6-7/2-3 | 5-6/3, 7-8/3 |
| fixes_72M (old plant, ~D) | 8-12/6-7, FELL 2/2 | FELL 1/2, FELL 2/2 | 5/2, 6-8/1-4 | 3/1, 5-13/1-5 |
| bike, -0.76 cmd | | 56/24 at 60 % of the command | | |

- **In the sim on its shipped floor, the 4.7 Hz rear wobble is v4's; the
  bike disagrees** (all three wobble there, next section).
  At -0.4 and -0.8 the older policies reverse quietly (yaw 3-14) on every
  plant; v4 wobbles on all four, worst on the old detent without play,
  where it matches the bike's 56/24. At -1.2 on the play plants the older
  two fall even on a 1 s ramp like the mirror's (8 of 8; at -0.8 on B the
  ramp saves fixes_72M, 2 of 2), and not out of a fast wobble: the speed
  reaches -1.0 to -1.2, then over ~1 s the roll creeps to 5-11 deg, the
  steer turns slowly to 20-30 deg, the rear stops and lurches forward
  (+0.4 m/s), and the fork goes round (to -100 deg). One run (rake20 on
  B, a step) weaved for 5 s first: steer +-60 deg, yaw +-300 deg/s,
  forward and back at ~0.6 Hz -- the slow swing the user sees, which v4
  never shows in the sim. Read so: v4 holds a reversing balance the
  older two lose at speed, and the rear wobble is how it holds it (an
  inference).
- **Whether the fork goes round reversing fast is policy x plant.** The
  older policies, flown on the play they never trained on, lose the fork
  at -1.2 (and on A wobble forward at yaw 51-69, the old shake); v4
  never does on this floor. A step is harsher than the mirror's ~1 s ramp.
- **v4 on the stiffened bike (B against A):** forward quieter (yaw 4-5
  against 14-15), -0.4 worse (45-54 against 16-20), -1.2 better (7-8
  against 16-20). The slow steer swing the user sees does not appear in
  the sim short of a fork going round (v4's 0.3-2 Hz steer under 2 deg
  RMS everywhere it survives).

**Floors, v4 on B.** No floor here is measured yet
(`docs/measurements/floor-measurements.yaml` is the template), so these
are settings, labelled by sink at rest (`contact_calibration.rest_sink`);
mu 0.35 is a guess for a slippery floor, the shipped 0.9 a GUESS:

| floor | sink, mm | mu | -0.8: yaw/diff, speed | -1.2: yaw/diff, speed |
|---|---|---|---|---|
| hard, grippy (shipped) | 0.135 | 0.9 | 21/16, -0.85 | 7-8/4, -1.02 |
| hard, slippery | 0.135 | 0.35 | 4/4, -0.88 | 4/3-16, -0.99 / FELL 1/2 |
| soft, grippy (`[0.02, 1.0]`) | 0.75 | 0.9 | **68-72/24-25, -0.62** | 64-67/25, -0.80 |
| soft, slippery | 0.75 | 0.35 | 30-31/22-23, -0.77 | 19-22/20-22, -1.0 |
| stiff, underdamped (`[0.005, 0.3]`), grippy | 0.012 | 0.9 | FELL 2/2 | FELL 1/2 |
| same, slippery | 0.012 | 0.35 | FELL 2/2 | FELL 2/2 |

**Bouncy at the shipped stiffness, in MuJoCo's negative form.** The
positive solref `[timeconst, dampratio]` couples the two: damping is
2/timeconst (over `solimp`'s dmax), and dampratio enters only the
stiffness, as 1/(timeconst x dampratio)^2 -- so a lower dampratio is
bouncier AND stiffer (0.3: 0.012 mm of sink). The negative form
`[-stiffness, -damping]` sets each alone. Checked: `[-40000, -400]`
gives the shipped `[0.005, 1.0]`'s sink and drop exactly; `[-40000,
-120]` (damping ratio ~0.3) keeps the sink (0.139 mm) and rebounds the
rear 1.7 mm from a 10 mm drop, where the shipped floor does not
rebound. v4 on B over it: -0.8 yaw/diff 15-21/11-15 (shipped 21/16),
but reversing at -1.2 it fell in 1 of 2 runs (grippy) and 2 of 2
(slippery) against 0 / 1 on the same floors without the bounce.

The soft grippy floor is the first sim to give both halves of the bike's
reversing: the wobble (yaw ~70, diff 25) AND the speed shortfall (78 %
and 66 % of the command against the bike's 60 %). Which floor 212855's
backward stretches were on is not in the record. Dampratio 0.3 (the
bench's bounce) is ten times stiffer, the naming trap; it is where v4
loses the fork, and is not a model of a bouncy floor so much as of none.

Real floors fall in roughly four corners, with mu for rubber typical
of textbook tables, not measured here: hard and slippery (polished wood,
laminate, smooth tile; ~0.3-0.5, lower when dusty), hard and grippy
(concrete, textured tile; ~0.6-0.9), soft and grippy (yoga or foam mat,
rubber; ~0.8-1+, and dead: soft floors damp rather than bounce), soft
and slippery (a rug sliding on wood, a vinyl skin over foam; rare
indoors). Bounce mostly comes from the hard side and the wheel itself,
not the floor. The rollers' own material sets mu as much as the floor
does. In the model the corners are `friction_sliding` against
`contact_solref` / `contact_solimp` (sink), with `contact_parts` to give
the rollers, the tyre and the wings their own; P0 / P0b / P1 measure a
floor's (section 5 of `floors-and-the-contact-model.md`).

**The rollers keep their sideways grip at speed on the bike; in the sim
they lose it.** The question (user): once it is rolling, do the rollers
give less sideways traction? Two rates for the rear, sideways: what the
rollers drive (no-slip roller kinematics from the drive servos'
measured speeds, `VelocityEstimator.roller_lateral`) and what the rear
does (the front constraint's rear lateral velocity, `v_lon tan(theta) -
yaw_rate L`, true while the front tyre does not slide). 2-7 Hz, and the
command -> servo lag of the drive differential at 4.7 Hz (the bike's
from Present Velocity; the sim's from the true shaft, `input_*_vel`):

| | rear actual / roller-driven, RMS (corr) | at 4.7 Hz: gain, phase | drive diff cmd -> servo |
|---|---|---|---|
| bike 212855, rest | 1.22 (0.81) | 1.05, +11 | 63 ms, gain 0.62 |
| bike 212855, forward | 1.12 (0.81) | 1.16, -1 | 61 ms, 0.67 |
| bike 212855, reverse < -0.6 (11 s) | **1.87 (0.64)** | **1.23, -1**, coh 0.96 | 62 ms, 0.54 |
| bike 175020 (before), rest / forward | 1.41 / 1.26 | 1.30 / 0.93 | 68 / 64 ms |
| sim v4 on B, shipped: rest / +0.8 / -0.8 | 1.64 / **0.57 / 0.68** | 1.49 / 0.46 / 0.70 | 37 / 30 / 25 ms |
| sim, soft grippy: rest / +0.8 / -0.8 | 1.47 / 1.41 / 1.30 | 1.43 / 1.27 / 1.29 | 34-42 ms |
| sim, old detent, shipped: -0.8 | 0.99 | 0.98 | 30 ms |

Ratios over 1 at rest on both sides say the roller kinematic gain reads
~20-40 % low against the constraint, so read each column against its own
rest row. Reversing, the bike's rear follows the rollers at the wobble
frequency (1.23, in phase), as at rest, and on top moves about as much
again sideways that the rollers did not drive (the correlation drops to
0.64): sliding. The sim on its shipped floor is the one whose rollers
lose grip at speed (0.57-0.68 of what they drive). On the soft grippy
floor -- where the sim matched the bike's wobble -- they keep it
(1.30). One reading, an inference: the bike's rear has more sideways bite
at speed than the floor v4 trained on, so the push the policy learned
where the rollers slip overshoots, and the rear rings.

The drive differential reaches the servos ~62-68 ms after the command on
the bike in every regime, before and after the stiffening, against 25-42
ms in the sim. Most of that gap is likely measurement: Present Velocity
carries the model's 4 ms measurement delay and 12 ms filter (~15 ms at
4.7 Hz), and the record reads before it commands (up to 10 ms). Swept
anyway, v4 on B with the drive's `command_delay_s` 4 / 15 / 25 / 35 ms,
2 seeds: reversing -0.8, the differential grows 16 -> 19-21 -> 22-24 ->
21-22 rad/s, the yaw stays 15-24 deg/s, the speed -0.85 -> -0.75; -1.2
and forward stay quiet. Not the 2x.

**All three policies, reversing on the bike (10-28 evening).** Records
261028-231419 (fixes_72M), -231802 (rake20_80M), -232101 (v4), ~2.5
min each, `--move` per process, driving with reverses. Stretches of a
steady command, first 0.5 s dropped:

| | reverse -0.3..-0.6: speed / yaw 2-7 / diff 2-7 | reverse < -0.6: s, speed, yaw | forward: speed, yaw | falls (in or just after a reverse) |
|---|---|---|---|---|
| fixes_72M | 110 % / 48 / 19 | 3.1 s, 62 %, 24 | 94 %, 9 | 3 (3) |
| rake20_80M | 105 % / 59 / 23 | 7.2 s, 79 %, 27 | 99 %, 15 | 4 (3; one forward at +1.2) |
| v4 | **47 %** / 51 / 20 | 19.2 s, **47 %**, 43 | 95 %, 10 | 2 (1; one a 180 snap at rest) |

- **The 4.7 Hz rear wobble is not v4's on the bike: all three have it**
  reversing at -0.5 (yaw 48-59, the drive differential 19-23 rad/s),
  where the sim's shipped floor gave the older two 3-13. The rollers grip
  for all three (rear actual over roller-driven 1.15-1.25, corr 0.81-0.98).
- **v4 is the slow reverser**: 47 % of the command at both levels, where
  the older two make 105-110 % at -0.5. It also reversed fast the longest
  (19 s) and fell once doing it; the older two fell 3 times each, mostly
  in the stop after a -1.2 reverse: the steer turns 30-80 deg, the rear
  lurches forward (+0.5-1.0 m/s), over.
- **The old pointer's shake looks halved** since the stiffening: roll
  rate 7-12 Hz at rest 13.6 deg/s in 154338 (10-26, one 56 s stretch) and
  6.7 here (12 short stretches between driving; same Welch, so not quite
  like for like). v4: 3.4 then, 4.4 now. Front play as part of the old
  shake would fit; one run.

**v4 holds back on purpose.** The user's reading: v4 learned that going
backwards falls, so it trades away the commanded speed. The record
agrees on the mechanism: the policy's own wheel target (the drive
command's common mode as a rear speed) against the user's command, and
what the wheel then made:

| reversing | policy's target / user's cmd: -0.3..-0.6, < -0.6 | wheel made / target |
|---|---|---|
| fixes_72M | 116 %, 72 % | 94 % |
| rake20_80M | 112 %, 88 % | 90-93 % |
| v4 (232101) | **47 %, 56 %** | 84-93 % |
| v4 (212855) | --, 50 % | 89 % |

So v4 asks for half the reverse speed, and the wheel delivers what it
asks; it is not slip. Over 0.5 s windows, v4's target falls as the bike
moves more: 68 % of the command in the calmer half (by yaw-rate std), 53
% in the wobblier half (212855: 85 / 74 %; Spearman against roll-rate
std -0.30 / -0.60); the older two hold 113-119 % in both halves. In the
sim on its own training plant v4 reverses at the full command (-0.85
for -0.8, no falls), so it is not a blanket rule it learned; it backs
off on the bike, more when the bike is moving about. Which way the
causation runs, and why it holds back even in the calm windows (68 %),
is not in these records.

**The stiffening removed the old pointer's shake.** The user: tightening
the steering and axle made it "way way better" backwards, and forwards
too (still some wobble there), more than the runs captured. A
stationary run of the old pointer after it (261028-232511, 124 s,
`run_vs_sim --no-plot --seed 1`, bike side, 7-12 Hz):

| | fixes_72M 154338 (10-26, before) | fixes_72M 232511 (10-28, after) | v4 154111 (10-26) |
|---|---|---|---|
| roll rate, deg/s | 16.4 | **5.2** | 4.0 |
| yaw rate, deg/s | 19.7 | 7.7 | 7.9 |
| steer target, deg | 1.40 | 0.15 | 0.115 |
| drive A / B cmd, rad/s | 3.1 / 4.9 | 0.66 / 0.73 | 0.37 / 0.63 |
| roll-rate peak | 10.0 Hz | 4.7 Hz | 5.3 Hz |

Taking the play out of the front axle and headset did for the policy
that never trained on play what v4's training did: the 9-10 Hz shake is
gone, and the bike is within ~1.5x of that policy's own sim (roll rate
5.2 against 3.4, yaw 7.7 against 4.2). That is the play diagnosis
(tests 2 and 9) confirmed on the hardware from the other side. It also
dates the fitted play: v4 trained on 1.0 mm of front axle and 1.5 mm of
headset play that the bike no longer has.

**The soft floor gives the older policies the bike's wobble too.** Sim, B
plant, a 1 s ramp, 2 seeds, yaw 2-7 / diff 2-7, speed:

| | shipped, -0.5 | shipped, -1.0 | soft grippy, -0.5 | soft grippy, -1.0 |
|---|---|---|---|---|
| fixes_72M | 5 / 2, 112 % | FELL 2/2 | 9-11 / 4, 111 % | **66-70 / 24-25, 92 %** |
| rake20_80M | 11-12 / 5-6, 111 % | 4 / 1.5, FELL 1/2 | 17-32 / 7-11, 108 % | 31-47 / 15-19, 104 % |
| v4 | 34-38 / 14, 106 % | 7-9 / 6-7, 103 % | 68 / 21, 83 % | 64-71 / 25-26, 84 % |
| bike | 48-59 / 19-23 (all three) | | | |

On the soft floor (0.75 mm of sink against 0.135) all three wobble, and
the older two stop falling at -1.0 (0 of 4, against 3 of 4 on the
shipped floor), closer to the bike in both, though not matched: at -0.5
the sim's older policies wobble less than the bike's, and the bike's v4
reverses slower (47 % against 83 %). Read so: the bike's ground contact
is more like the soft setting than the shipped one, for all three
policies. The contact's compliance is the rollers' and tyre's as well as
the floor's; P0 (load against sink, on the bike's own wheels, on the test
floor) is the measurement that says which.

**Not an odometry loop.** The question (user): when the steer passes some
angle, does the velocity estimate get stuck or lost and feed back? The
lateral estimate is the front wheel's no-slip constraint, `v_lat = v_lon
tan(theta) - yaw_rate L` (`hw/odometry.py`); past ~75 deg of wheel
heading its confidence goes to 0 and it coasts on the accelerometer.
On the bike that happens only AFTER the fork has gone round: through
the reversing fall's lead-up the wheel heading is 1-7 deg and the
confidence 0.98-1.0; it drops (0.46 -> 0) at 40-87 deg, in the last
0.3 s. The estimate does carry the wobble: v_lat swings +-0.2-0.5 m/s
sample to sample there, nearly all of it the `yaw_rate L` term (60
deg/s x 0.2 m). Testing whether the wobble needs that path: v4 in the
sim with the policy given the TRUE lateral velocity (`SimOdometry`
channels "lon"), 2 seeds:

| v4, true v_lat | yaw/diff, speed | with the estimate |
|---|---|---|
| B, shipped floor, -0.8 | 26-29/16-19, -0.81 | 21/16, -0.85 |
| B, shipped floor, -1.2 | 25-38/16-23, -0.94 (1 fell at 11.7 s) | 7-8/4, -1.02 |
| B, soft grippy, -0.8 | 70-73/25-26, -0.46 | 68-72/24-25, -0.62 |
| D (old detent), shipped, -0.8 | 63-66/25-26, -0.67 | 58-60/24-25, -0.79 |

The wobble is there, as large or larger, with the true velocity, so in
the sim it does not need the estimator; nor does the speed shortfall.
The bike's estimator could still differ from the sim's (its gyro, its
noisier accelerometer), which these runs cannot test.

The user's reading of the hardware change: mostly the front axle and the
steerer's play went, and the added friction is handled by the drives and
the steer.

**The other two falls were 180 deg heading snaps from rest.** Every 180
snap on v4, with how it went:

| record | t, s | peak yaw, deg/s | roll min, deg | turned |
|---|---|---|---|---|
| 175020 (before) | 28.9 | 278 | -12 | 174 deg in 1.5 s |
| 175020 (before) | 251.9 | 268 | -12 | 182 |
| 180445 (before) | 158.8 | 257 | -12 | 180 |
| 212855 (after) | 149.8 | 187 | -59 | 59 deg, FELL at 0.7 s |
| 212855 (after) | 352.9 | 220 | -56 | 110 deg, FELL at 1.0 s |
| 212855 (after) | 382.6 | 246 | -8 | 186 |
| 212855 (after) | 389.8 | 255 | -9 | 178 |

Each spin pins drive A at its 33.3 rad/s action limit for ~0.4 s. The two
that fell spun slower (187 and 220 deg/s at peak) and leaned out instead.
The three 90 deg snaps in the same record were fine. Both falls were on
a really slippery wood floor (user); tried again on a grippier floor,
the snaps were fine. So the floor, not the steering: the rear slides
there instead of spinning the bike round.

**The sim spins like the bike did before, and friction alone does not
make it fall.** v4 on its training plant (`run_vs_sim.sim` with a
command; a scratch script, not tracked), seeds 0 / 1:

| sim | 180 snap: peak yaw, roll min | backward -0.8: speed, yaw 2-7 Hz |
|---|---|---|
| as trained | 257 / 238, -11 / -9 | -0.85 / -0.86, 16.1 / 17.8 |
| headset friction x2 | 253 / 238, -10 / -7 | -0.85 / -0.86, 24.2 / 25.7 |
| + front axle 5 mN m (`front_spin` frictionloss) | 243 / 257, -10 / -11 | -0.86 / -0.85, 29.0 / 22.6 |
| + front axle 20 mN m | 234 / 243, -11 / -10 | -0.85 / -0.86, 35.7 / 20.1 |

No variant fell on a snap. More steer and axle friction raises the
backward wobble 1.5-2x, toward the bike's 57, but the sim still reaches
its reversing speed where the bike makes 60 %. The axle values are
guesses (the axle's drag is unmeasured). The sim floor's mu is 0.9 (a
GUESS) and the CPU configs randomise it +-20 %; a slippery wood floor
is probably below that. Open: the backward speed gap and the 2x
reversing wobble.

### 11. Standing 90 deg snaps fall since the axle nut  [sim done 10-31; floor and axle to measure]

On the bike, a 90 deg heading snap from rest (the d-pad), counted as a fall
when the bike cuts within 1.5 s and before the next snap:

| | falls / snaps |
|---|---|
| v4, 10-24 to 10-27 (before the shim and nut) | 3 / 102 |
| v4, 10-28 and 10-29 | 2 / 7 |
| v5 s1, 10-29 | 2 / 2 |

In the snaps that held, nothing moved across the change (first 0.45 s,
medians, before / after): steer target swing 36 / 36 deg, worst tracking
error 13 / 14 deg, peak yaw 160 / 161 deg/s. The drive command sits at its
~33 rad/s limit in nearly every snap. Pack 11.8-12.3 V and the firmware
gains are the same in every session. Falls come 0.5-0.8 s after the snap;
3 of the 4 since 10-28 were +90.

Axle drag measured by hand: **100 g at 30 mm to turn the wheel in the
air, 0.03 N m.** In the sim (`bike.front_wheel.axle_friction_nm`, dry
friction on `front_spin`; `analysis/policy/axle_friction_snaps.py`), it
is the PAIR, floor grip and axle drag together, that falls. Falls out of
40 standing snaps, no wheel play, the shipped floor stiffness:

| v4 80M, floor mu \ axle N m | 0 | 0.015 | 0.03 | 0.06 | 0.10 |
|---|---|---|---|---|---|
| 0.9 / 0.7 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| 0.6 | 0 | 0 | 0 | 0 | 18 |
| 0.5 | 0 | 0 | 3 | 20 | 32 |
| 0.45 | 0 | 0 | 7 | 27 | 40 |
| 0.4 | 0 | 4 | 23 | 39 | 40 |

v5 s1, which turns harder (peak yaw ~230 deg/s against ~185), falls a
step sooner: at mu 0.6, 0 / 2 / 7 / 34 / 40 across the same axle values,
and 17 of 40 at mu 0.5 with a free axle. A soft floor (solref 0.02) falls
more at the same mu. The slip-trained v5 s3 turns in ~2 s at 115-150
deg/s and holds to mu 0.45; the user judges it a worse policy overall.

At mu 0.9 the axle alone needs ~0.12 N m (v4 on its trained play) to ~0.2
(no play) before any snap falls, the front wheel then mostly skidding.
LESS front play makes the sim more tolerant of axle drag, not less, so
the shim is not the suspect through this route. With no play every fall
in the sim is +90, none -90 (cause not looked for).

Next, physical: the floor's mu by a tilt test (the bike on a board of the
floor, tilted until it slides; mu = tan(angle): 0.45 = 24 deg, 0.6 = 31,
0.9 = 42), front tyre and rollers apart; then the axle loosened and the
snaps flown again on the same floor. Teleop takes both: `--floor-mu`,
`--axle-friction`. If the floor reads low, the retrain randomises floor
mu (~0.4-0.9) and axle drag (0-0.06 N m) together; GPU friction is
already per world, `dof_frictionloss` too.

## Not tests, but now worth watching

**10-26 evening, three sessions on the v4 pointer** (261026-162523,
-163044, -163421), driving around. What the records show:

- **A steer ~180 deg off is fine for the policy and was not for the
  get-up.** In 163044 the bike fell at 34 s; propped (steer limp) the fork
  swung from +100 to +181 deg, and the policy engaged at +196 and flew
  68 s around +180: it sees the steer as sin/cos(2 delta), so that is
  straight. The NEXT process (163421) read +201 at power-up, and the
  get-up held the steer at `data.ctrl`, 0 in a fresh process, so the servo
  drove the wheel ~200 deg back to 0 at the get-up's start. FIXED: the
  sequence now holds the steer where it reads (`run_bike._sit_step`,
  `test_hw_swing_sit.py`).
- **The first get-up in 163421 fell back**: the lift (from crank +39 deg,
  back-driven past stow while the process was down) stalled at crank -96
  against its -110 goal with the bike at roll -16, outside the 12 deg
  recover gate; after the 3 s `lift_timeout_s` the sequence let go
  (current faded over 1 s, the crank sent to the rest pose 25 deg the
  other way) and the bike dropped to -80 in 0.3 s. The auto-retry 1 s
  later reached crank -102, roll -11.7, and propped. Both lifts went the
  right way. Open: a lift that ends 4 deg outside the gate.
- **Screen dim = link lost.** At 144.6 s in 163044 the station stopped
  sending; the 1 s link cut fired at 145.5 and the bike went over onto a
  blade (roll -36). Packets came back at 154 s, but no press after that
  moved anything to the end of the record (204 s; further dropouts at
  181.7 and 203.3). Why is not in the record -- run_bike's event lines
  ("LINK BACK from ...", "... ignored -- ...") print only in its terminal.

- `run_bike` rides out up to 3 bad servo reads in a row (`BAD_READS_MAX`;
  one `rc=-3002` ended a session on 10-24). The record's meta counts them
  (`bad_reads`); if they become frequent, the bus wiring or the 3 Mbps baud
  is the question.
- Records before 2026-10-24 have 101 as drive_a; each run's meta `roles`
  says which.
