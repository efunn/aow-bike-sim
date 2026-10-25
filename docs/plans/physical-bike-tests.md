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
| shake | a ~9 Hz oscillation in the steer target, both drive commands, roll and yaw rate; the sim has 5-25x less in that band (below) |
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
   notch-phase obs (sin/cos of 2 pi phi / 7.5 deg, computable on the Pi
   from Present Position), a `general_spec` layout change on both sides,
   and a retrain. Cheap without any of that: start the twin
   (`run_vs_sim`) at the bike run's own phase instead of a draw.
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

Still a `GUESS` in config, accepted at every start by `--bench`. The "2-3
deg held upright" once written here was a hand-held standby reading, which
is not upright, so it is no evidence. What the balancing records give
instead: the median roll the AHRS reads while the bike balances itself is
+0.6 to +1.6 deg across the six engages since the swap (+0.56 in the 50 s
no-input one). That is mount offset plus any sideways CoM offset, which the
policy cannot tell apart and neither can this. Worth a jig only if the
drift turns out to follow it.

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
batch them.

## Not tests, but now worth watching

- `run_bike` rides out up to 3 bad servo reads in a row (`BAD_READS_MAX`;
  one `rc=-3002` ended a session on 10-24). The record's meta counts them
  (`bad_reads`); if they become frequent, the bus wiring or the 3 Mbps baud
  is the question.
- Records before 2026-10-24 have 101 as drive_a; each run's meta `roles`
  says which.
