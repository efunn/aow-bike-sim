# The detailed drivetrain model

> **Status: ACTIVE.** Opened 2026-09-14. A rough first version is built and
> **opt-in** — `config/drivetrain_model.yaml`, `src/aow_sim/drivetrain_model.py`
> — wired into the eval env, teleop and training (per config, no run yet), and
> NOT into the deploy bundle or `bike_params.yaml`. Owns: what the Station A captures say about the
> drive, how that became a model, what the existing policies do on it, and what
> is still unmeasured. The captures themselves are in
> `docs/measurements/drivetrain-measurements.yaml`.

## The answer first

Can the existing policies drive the detailed drivetrain? **At factory firmware
gains, yes, with a cost. At any stiffer gain, no.** Eval grid, 20 commands,
`--encoder counts --ahrs tm151 --ahrs-tau 0.19`, score / survival:

| plant | `general_rl_odo_ahrs` | `general_rl_odo_ahrs_rand2` |
|---|---|---|
| ideal (what they trained on) | 0.644 / 0.95 | 0.665 / 1.00 |
| servo, P 100 / I 1920 | 0.641 / 0.95 | 0.691 / 1.00 |
| servo + detent | 0.508 / 0.85 | 0.647 / 0.95 |
| servo + detent + roller slop (**full**) | **0.636 / 0.95** | **0.601 / 0.90** |
| roller slop alone | 0.497 / 0.90 | 0.623 / 0.95 |
| full, P 200 / I 1920 | 0.285 / 0.55 | 0.455 / 0.75 |
| full, P 400 / I 1920 | 0.076 / 0.30 | 0.000 / 0.00 |
| full, P 400 / I 3840 | 0.058 / 0.20 | 0.047 / 0.15 |
| full, P 400, loop lag removed (diagnostic) | 0.173 / 0.40 | 0.294 / 0.60 |

    python analysis/drivetrain_eval.py --variants ideal servo servo+detent full slop \
        full_p200 full_p400 full_p400_i3840 full_p400_nolag

`general_rl_cmd_curriculum2b`, the current training line, on the same grid:
ideal 0.595 / 0.90, full P 100 0.638 / 0.95, full P 400 0.177 / 0.50 — the same
pattern, a little less steep at P 400.

Read with the grid's resolution in mind: one episode is 0.05 of survival, so
`servo+detent` scoring below `full` for `odo_ahrs` is inside the noise, not a
sign that slop helps. What is not noise:

- **The servo model alone costs nothing.** The fitted firmware loop, the motor
  line, the measured inertia and a Coulomb friction 60x the old guess, and
  both policies score the same.
- **The losses concentrate on the ±170–180° turns** in every degraded row
  that still mostly survives — the spins, where the diff channel works hardest.
- **Firmware P is the cliff, and it is monotonic.** P 100 survives, P 200
  loses a quarter to a half of the grid, P 400 loses nearly all of it. Removing
  the loop lag at P 400 (so no ~20 Hz peak) recovers only part of that, so the
  cause is mostly the stiffer P term itself — these policies learned a drive
  with no P action beyond back-EMF — and the resonance makes it worse.

**The consequence for the open firmware-gain decision:** P 400 / I 3840 is the
setting that cut detent stalls most on the bench (49 % → 12 % stuck), and it is
the setting no existing policy can fly. The gain and the policy cannot be
chosen separately. A policy TRAINED at P 400 is the untested half, and is the
experiment this points at.

`ideal` row caveat: `rand2` reproduces status.md's 0.663 / 1.00; `odo_ahrs`
reads 0.644 here against status.md's 0.570, same sensor flags. Not chased.

## What changed, component by component

### 1. The XC430 servo — yes, the existing model needed replacing

The shipped drive is a native MuJoCo velocity-PI at the input shaft,
`force = ki ∫(ctrl − w) − kv w`, kv the bare-motor droop 0.016016, ki 0.6
guessed from a 1 kHz loop assumption, a flat torque clamp, no delay, armature a
GUESS. The captures replace every part of that, because Present PWM IS the
controller output and can be regressed directly — no mechanics needed:

    duty   = Kp (v_ref − LPF(w_meas)) + Ki (∫v_ref − θ_meas),  clipped to ±1
    torque = stall_torque (duty − w / no_load_speed)

| quantity | fitted | evidence |
|---|---|---|
| Kp per P unit | **4.0e-4** duty/(rad/s) | median 4.02e-4 over reversals + chirps at P 100/200/400; ROBOTIS's unit conversion gives 3.68e-4 |
| Ki per I unit | **1.45e-3** duty/rad | 1.29–1.49e-3 on steps (R² 0.997–1.000); the hand hold spring gives 2.69 duty/rad at I 1920 (1.40e-3). 2x the ROBOTIS conversion at 1 kHz, so a 2 kHz loop is the likely reading — inferred |
| command delay | 4 ms | `latency` block; 4 vs 6 ms made no difference in replay |
| loop measurement lag + velocity filter | 4 ms + 12 ms | effective, fit by replay: puts a peak at P 400 and none at P 200 |
| rotor inertia at servo | 2.2e-3 kg m² | coast 2.3–2.8e-3, P 400 chirps 1.8–2.3e-3, bare-shaft tau_m 14 ms → 2.0e-3 |
| Coulomb friction at servo | 0.15 N.m | coast 0.16–0.22 common, 0.16–0.20 diff; same in both modes, so it lives at the input shafts |

Kp and Ki are fitted **per table unit** on purpose: any gain pair becomes a
one-line change (`--servo-gains 400:3840`), which is what let the eval above ask
the firmware question at all.

Ki from the chirps and the P 400 reversals scatters low (0.27–0.84e-3); those
captures barely excite the integral, and the steps that do agree tightly.

### 2. Detent and roller slop — two components, not one

`drivetrain-measurements.yaml` reads them as one mechanism with three numbers,
and as a statement about ORIGIN that stands. As model structure they are two,
because they act on different coordinates and are seen by different tests:

- **The detent is a property of the drive coordinate.** It stalls a 1 % diff
  creep with the wheel in the air, rollers unloaded. Nothing loads a roller
  there, so whatever resists is a function of ring-vs-hub angle, not of a
  roller's deflection. Modelled as a friction bump, raised cosine, 2.5° wide
  every 7.5°, peak 0.096 N.m on ring-vs-hub (0.09 duty per servo above the
  floor).
- **The slop is a relation between each roller and the drive.** It only
  matters when something loads the roller — the ground. Modelled per roller as
  a tendon `roller_spin_i − k_roller·ring_spin` with ±7.8° hard limits and a
  weak centring spring, replacing the rigid equality.

Friction rather than a conservative hill, from the direction-split fold
(`drivetrain_fit.py detent`): at 1 % the push shared by both directions sits
0.09–0.12 duty above its floor at the same phase. At 3 % neither shows much
(a ~0.02 duty ripple). A quasi-static torque ramp would settle it properly.

**The 20 Hz dwell does NOT argue for compliance over a gap.** An unloaded
roller follows its drive through a gap given any centring spring or friction:
at ±7° and 20 Hz its inertia needs ~4e-4 N.m. So `dynamic_check` cannot
separate the two.

**The loaded hand check does (2026-09-14, `roller_slop.loaded_hand_check`):**
pressed down hard on a table, the play is the same size as unloaded and still
takes very little effort, and pushing a roller past a wall from the ground is
very hard. A gap with hard walls, not compliance — which is what the model
already is. The centring spring and damping inside the gap stay `GUESS`es,
but the damping is now set by a second hand check (`roller_slop.hand_flick`): a
roller flicked by any amount, or held at the end of its play and let go,
returns to centre with no visible overshoot. The tendon damping is critical
against the roller's own spin inertia — 2.9e-5 on top of the joint's 1e-6,
where 2e-6 rang for ~200 ms — and a test pins the ratio.

## Does the model reproduce the bench?

`python analysis/drivetrain_fit.py replay` runs every recorded command stream
through the MuJoCo **testbed** (wheel on a stand, in the air) with the overlay
— the implementation itself, not a copy of its equations — and computes
identical metrics on both:

| capture | real | sim |
|---|---|---|
| chirp common, plateau 3–10 Hz, P 100 / 200 / 400 / 400:3840 | 0.15·0.07 / 0.38·0.32 / 0.55·0.46 / 0.70·0.50 | 0.16 / 0.30 / 0.54 / 0.69 |
| chirp diff, same | 0.21·0.05 / 0.35·0.28 / 0.48·0.39 / 0.62·0.38 | 0.14 / 0.28 / 0.49 / 0.64 |
| peak above 10 Hz, P 400 common / diff | 0.97·0.83 / 1.20·1.07 @ 19–23 Hz | 0.77 / 0.85 @ 19–21 Hz |
| square diff 25 %, 10 / 15 Hz, P 400 | 1.11 / 1.38 | 1.04 / 1.28 |
| square diff 25 %, 10 / 15 Hz, P 100 | 0.55·0.48 / 0.43·0.48 | **0.25 / 0.25** |
| 1 % diff creep, time stuck, P100:I1920 → P400:I3840 | 48–53 % → 9–14 % | 62–64 % → 11–12 % |
| same, I 960 / 1920 / 3840 at P 400 | 39 / 18–22 / 9–14 % | 55–62 / 43–44 / 11–12 % |
| p99 burst over command, 1 % creep | 5.2–9.0x | 4.0–6.2x |

(`a·b` = servo 101 · 102.) The trend across the whole 9-point P/I grid comes out
with the right sign and roughly the right size, and it comes out of the detent:
with the detent off, the sim never stalls at all.

**Known gaps, in order of how much they could matter:**

1. **P 100 reversals track at half the real swing** (0.25 vs 0.41–0.55). The
   factory-gain sim is SLUGGISH where the hardware is not, so the P 100 rows
   above may be pessimistic about the servo.
2. **The ~20 Hz peak is under-predicted** (0.77–0.85 vs 0.83–1.20), so the P 400
   rows may be optimistic about the resonance.
3. **Common-mode stickiness is absent** (sim 0 % stuck against 1–11 %): the
   unplaced 8.44° ripple is not modelled.
4. **Servo B's side is stiffer** in every capture; the model uses one value. It
   CAN be matched by per-side friction: `coulomb_friction` takes `[a, b]`, and
   replaying at `[0.16, 0.21]` (the two coast fits) splits the sim the way the
   bench splits. Chirp plateau, common / diff, [101, 102]:

   | | bench | sim symmetric 0.15 | sim [0.16, 0.21] |
   |---|---|---|---|
   | P 100 | 0.15, 0.07 / 0.21, 0.05 | 0.16, 0.16 / 0.14, 0.14 | 0.13, 0.06 / 0.12, 0.06 |
   | P 200 | 0.38, 0.32 / 0.35, 0.28 | 0.30, 0.30 / 0.28, 0.28 | 0.27, 0.15 / 0.26, 0.14 |
   | P 400 | 0.55, 0.46 / 0.48, 0.39 | 0.54, 0.54 / 0.49, 0.49 | 0.51, 0.42 / 0.47, 0.38 |

   Close at P 100 and P 400, too hard on B at P 200. Not the default — it is a
   property of one assembly — but it says the asymmetry is a friction-shaped
   thing, so training can randomise each side's friction independently
   (`randomization.drive_friction_side_frac`, off by default).

       python analysis/drivetrain_fit.py replay --friction 0.16 0.21

### The creep test, rebuilt

`python analysis/drivetrain_creep_replay.py` → `analysis/plots/drivetrain_creep_replay.png`:
the bench's creep time series (capture `260913-013834_creep`, same 20 ms
differencing as `drivetrain_creep_plot.py`) against the testbed replay under
each plant, plus a sim-only roller push and flick.

| plant | diff +1 % stuck / burst | diff +3 % | common +1 % |
|---|---|---|---|
| bench | 46 % / 7.6x | 39 % / 3.2x | 8 % / 2.8x |
| full | 62 % / 5.9x | 36 % / 2.5x | 0 % |
| without slop | 62 % / 5.9x | 36 % / 2.6x | 0 % |
| without detent | 0 % | 0 % | 0 % |
| without servo (old native PI) | 62 % / 5.9x | 31 % / 2.7x | 0 % |
| ideal | 0 % | 0 % | 0 % |

- **The detent is the whole of the stick-slip.** Without it nothing stalls,
  with any servo. The old native PI stalls just like the fitted loop.
- **The creep test cannot see the slop.** Only the servo shafts are encoded and
  nothing loads a roller in the air, so `without slop` is identical. Its
  eval-grid cost comes entirely from ground contact.
- **The sim is too regular.** Every tooth in the model is the same wall, so its
  stalls come like a metronome; the bench's vary in height and spacing, and its
  common mode has the unmodelled 8.44° ripple.
- **Sim flick, critically damped to match the hand flick:** a roller flicked at
  20 rad/s peaks at ~6.5° and returns without overshoot, most of the way in
  ~100 ms, with a slow last half-degree against the roller's own friction.

## How it is built

- **Opt-in overlay.** Loaded into params as `drivetrain_model`, which moves
  `plant_digest` for exactly the runs that use it; nothing else sees it.
  `edit_spec` is the last thing `build_spec` does and is a no-op without it —
  pinned by a test comparing the compiled XML.
- **`ctrl[drive_a/b]` keeps its meaning.** The drive actuators become
  zero-force command holders and two torque actuators are appended LAST, so no
  caller changes and no actuator index moves.
- **`DrivetrainSim.pre_step(data)` before every `mj_step`** runs the firmware
  loop and the frictions. Wired into `GeneralEnv`'s substep loop and
  `teleop_loop(pre_step=)`; it resets itself on a rewound `data.time`.
- **Friction is elasto-plastic in Python, not MuJoCo `frictionloss`.** Both the
  input-shaft Coulomb friction and the detent: a stiff spring to an anchor that
  slides past the limit. MuJoCo's soft friction constraint creeps on these
  light joints — measured 0 % stuck at 1 % creep for every friction solref from
  0.02 down to 0.001 — and without a static floor the detent never stalls.
- **The analytic LQR is designed on the ideal plant** and flown on this one
  (`run_drive` passes `design=`), which is the hardware situation anyway.

Teleop:

    mjpython -m aow_sim.run_drive --teleop --drivetrain
    mjpython -m aow_sim.run_drive --teleop --drivetrain --servo-gains 400:3840
    mjpython -m aow_sim.run_drive --teleop --drivetrain --drivetrain-without roller_slop

Training, per config (`config/rl_general.yaml` documents the keys):

    env:
      drivetrain_model: true            # or a path; absent = ideal drives
      drivetrain_without: [roller_slop] # optional
      servo_gains: [400, 1920]          # optional, firmware table units
    randomization:
      drive_supply_randomize: false     # true: the servo follows actuator_frac
      drive_friction_side_frac: 0.0     # per-side Coulomb friction x U(1-f, 1+f)

`train_general_rl` resolves the overlay once, so `plant_digest` covers it, and
writes the RESOLVED overlay dict into the move yaml as `drivetrain_model`.
`policy_env_overrides` carries it, so every env built from the policy —
per_command, chatter, floor_sweep — rebuilds the plant it trained on, even after
this yaml moves on. Teleop says so when such a policy is flown without
`--drivetrain`. Both randomisation knobs are off by default on purpose: the
first runs should add the drivetrain and nothing else. `actuator_frac` still
randomises the STEER servo; set it to 0 as well for a fully fixed battery.
Checked 2026-09-14 by a 2000-step smoke run through export.

### Cost

`general_rl_odo_ahrs` under its sensors (odometry + TM151), hold command,
3000 env steps each, one process, M4, 2026-09-14:

| plant | env steps/s | vs ideal |
|---|---|---|
| ideal | 813 | 1.000 |
| servo | 755 | 0.928 |
| servo + detent | 737 | 0.906 |
| full | 750 | 0.922 |
| roller slop alone | 829 | 1.019 |

    python analysis/drivetrain_eval.py --bench 3000 --policies general_rl_odo_ahrs \
        --variants ideal servo servo+detent full slop

**~8 % slower, all of it the per-step Python loop**; the slop tendons are free
(inside run-to-run noise). A 6M-step run pays ~8 % more wall time. Not yet
optimised: the loop runs every 0.4 ms physics step; running the firmware at
~1.25 kHz (every second step, closer to the real loop anyway) or moving it into
a MuJoCo plugin are the two obvious cuts.

**The first cut was tried and is not worth it (2026-09-14).** `update_every: 2`
(firmware at 1.25 kHz, frictions still every step — held across steps the
stick springs changed tracking) takes the overhead from ~6 % to ~2.3 %, i.e.
the env ~4 % faster: 749–755 → 778–784 env steps/s against 800–826 ideal. The
replay is unchanged at P 400 and in the creep grid, but the P 100 15 Hz square
wave collapses (0.25 → 0.05; bench 0.43–0.48), and the eval moved −0.16 /
+0.05 in score across the two policies. The grid cannot separate that from
noise on this plant with one seed per command, and 4 % does not justify
finding out. `update_every` stays 1; the knob stays for a plugin-less
multi-seed check if training time ever makes it matter.

## Rejected

- **Editing `bike_params.yaml` directly.** It moves `plant_digest`, stales the
  deploy bundle and makes every export provisional — a checklist's worth of
  cost to find out whether the detail matters. The overlay asks first.
- **MuJoCo `frictionloss` for the detent and input friction.** Built first; see
  above for why it gives no stalls.
- **A conservative (potential) detent.** The fold does not support it at 1 %.
- **A native actuator approximation.** A duty clip inside the sum and two delays
  are not expressible; a user-callback (`mjcb_act_*`) is process-global and
  would bind every model in the process.
- **Validating on a lumped 1-DOF copy of the equations.** Used while fitting,
  then replaced by replaying the testbed model, so the check exercises the code
  that ships.
- **MuJoCo's default tendon limit softness for the slop walls.** Rollers
  overshot the ±9° wall by up to 56° under a dithered diff; `wall_timeconst_s`
  0.001 holds it to ~1.5°.

## Open, ranked

1. **Retrain on the full plant, and at P 400**, before choosing firmware gains
   — the eval says the two are one decision. **Queued, not yet run
   (2026-09-14):** `config/rl_general_drivetrain_p100.yaml` and `_p400.yaml`,
   `rl_general_cmd_curriculum2.yaml` (the line the 12-seed sweep ran;
   `cmd_curriculum2b` is its seed 1) with the drivetrain on, I 1920 in both
   arms and the battery fixed (`actuator_frac` 0), three seeds each,
   interleaved by `config/queue_drivetrain_gains.txt` through
   `./scripts/rl.sh queue`. ~14.5 h serial at 20M steps each. Training loads the plant now; the
   detent phase is already drawn per episode, and supply and per-side friction
   randomisation exist and are off. Not randomised at all yet: detent friction,
   slop size, the firmware gains themselves.
2. **Roller return time.** The hand flick settles the SHAPE (no overshoot); how
   fast a roller returns (sim: most of the way in ~100 ms) is unfilmed, and is
   what the centring-spring `GUESS` controls.
3. **Torque scale.** `duty × stall_torque` uses the datasheet 1.6 N.m. The bench
   replay is blind to it — inertia and friction are fitted in the same units,
   so an error cancels — but the bike is not: it sets drive strength against
   measured masses. D5 (a disc of known inertia, re-run a coast or chirp) gives
   it with no lever arm; randomising servo strength in training covers it too.
   Present Load cannot help: it is computed from PWM and speed.
4. **Quasi-static diff torque ramp** (PWM mode, up and down): the detent's true
   shape and hysteresis.
5. **The P 100 small-amplitude gap** above, before trusting factory-gain rows
   to better than the grid's noise.
