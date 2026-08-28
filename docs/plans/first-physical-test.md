# The first physical test — what to build, in what order

Written 2026-08-28. The question this answers: which parts and subassemblies
unlock the most sim uncertainty per unit of build effort.

**The headline: the two highest-value tests need NOTHING printed.** Both
protocols are already written, with empty data sheets waiting for numbers, and
both run on parts that exist. Do them before spending days on the printer,
because one of them can invalidate the control approach and the other pins the
least-measured parameter in the simulator.

Nothing here is new measurement design. `docs/measurements/servo-protocol.md`,
`contact-protocol.md` and `omni-wheel-protocol.md` are the procedures; this doc
only ORDERS them and says what each unblocks.

---

## Stage 0 — no printing, no assembly

### 0a. Weigh what you already have  [minutes]

The largest single unknown in the mass budget is `chassis.mass 0.45 kg
GUESS` — 44% of a 1.02 kg bike, standing on a guess. It cannot be weighed
until the frame exists, but two others can be, and both carry a "-> weigh the
actual X" note already:

    battery pack       0.115 kg  GUESS
    electronics stack  0.076 kg  GUESS

Caveat: `docs/status.md` records the untethered electronics as specced and
sourced but NOT ORDERED, so these may not be in hand. If they are, this costs
ten minutes and retires two GUESSes.

### 0b. THE SERVO REVERSAL TEST — the go/no-go  [one XC430, U2D2, power]

`servo-protocol.md` §2, which calls itself "the highest-value test here" and
"needs no load fixture". Free-spinning wheel, square wave on `Goal Velocity`,
25 Hz, sweep amplitude, then 5/10/15 Hz to find where tracking dies.

**Why this is first: it can invalidate every trained policy.** The sim permits
21.8 hub reversals/s and 6.37 m of rim travel per 15 s at the contact patch.
If the hardware achieves a small fraction of the commanded swing at 25 Hz —
which is the stated expectation — then the channel every RL policy leans on
hardest does not exist on the bike, and the fix is a rate penalty in the reward
BEFORE more training, not after.

Two additions worth making while the capture rig is up:

  * **Record 126 and `Present PWM`(124)** alongside position and velocity.
    Contiguous with what is already read, so the indirect block goes from 10 to
    14 bytes of `N_INDIRECT` 28 -- same single SyncRead, no extra lag.

    **126 IS THE SAME ADDRESS AND WIDTH ON BOTH SERVOS, and means different
    things**, which is a decode concern and NOT an addressing one:

        addr  XC430-W150 (drive x2)          XC330-T181
        124   Present PWM                    Present PWM, 0.113 %
        126   Present Load, -1000..1000,     Present Current, 1.0 mA
              0.1 % of max torque

    So the indirect map is byte-identical across servos and the SyncRead stays
    uniform; what is per-model is a SCALE CONSTANT at decode time, exactly as
    `VEL_LSB_RAD_S` already is. (And if a field ever does misalign, indirect
    addressing can point a dummy register at the gap to keep one virtual
    layout across models.)

    The XC430 has no current register at all -- zero mentions of
    `Present Current` or `Current Limit`, against 15 of the latter on the
    XC330 -- so the XC330 measures amps and the XC430 INFERS a percentage.

    **EXPECT `Present Load` TO BE WORST EXACTLY HERE.** The XC430 page defines
    it only as "currently applied load ... about 50% of the maximum torque".
    Reasoned, not from the datasheet: PWM duty at a given torque depends on
    SPEED, because back-EMF eats supply voltage -- so a duty-derived load
    estimate is speed-dependent, and the reversal test is nothing but rapid
    speed change. Capture PWM and velocity alongside it, and calibrate against
    a known hanging load at a few STEADY speeds before believing anything
    dynamic. See `eval-score-rewrite.md` option C.
  * **Test at the demand of the policy you would actually fly.** The 6.37 m
    figure is not universal: `general_rl_glide_pitch_hub3` cut rim travel over
    a 15 s hold to 3.35 m. A servo that fails at 6.37 and passes at 3.35 is a
    different conclusion — it says pick the calmer policy, not redesign the
    reward.

### 0c. Contact calibration  [the omni wheel you already measured]

`contact-protocol.md` §P0, §P0b, §P1. All four numbers are uncalibrated and
the data sheet is all zeros. Needs a weight, a caliper, a phone that shoots
slow-mo, and a board that tilts.

**Why it matters more than anything else in the sim:** `docs/status.md` calls
the contact model the least-measured parameter and the one no policy has been
randomised over; the last contact change is what broke the LQR (nine of the
red tests), and recovering it took `q_steer` 0.5 -> 5.0 and `q_roll_rate`
6.0 -> 30.0 with no explanation of why it cost that much.

Read the protocol's own CORRECTION first: `timeconst` sets damping AND
stiffness, `dampratio` sets stiffness only, so a static load-deflection reading
constrains the PRODUCT and the static and drop tests must be solved JOINTLY. A
single static reading fixes nothing by itself.

The incline slide test (omni §7.4, mu = tan(angle)) rides along free with the
tilting board and retires `friction_sliding 0.9 GUESS`.

---

## Stage 1 — the first thing worth printing: the drivetrain station

Servo -> belt -> input shaft -> hub, on a rigid plate. Not a bike; a testbed.
The sim already has its mirror: `--variant testbed`, so results compare
like-for-like rather than by eye.

Unlocks, via `omni-wheel-protocol.md` §7:

| test | retires |
|---|---|
| roller flick spin-down | `roller_joint_damping`, `roller_joint_frictionloss` |
| driven-wheel spin-down | `hub_joint_damping`, `hub_joint_frictionloss`, `input_armature` |
| servo step/ramp through the belt | validates `drive_kv`, `drive_tau` |
| — | verifies `belt_ratio 3.0`, currently `source: design` |

It is also the LOAD FIXTURE that servo-protocol §3 (torque-speed envelope) and
§4 (droop) need and §2 deliberately does without. So stage 0b and stage 1
are the same protocol, split at the point where a fixture becomes necessary.

Five of the 21 GUESS parameters die here, and they are the drivetrain ones that
no amount of sim work can resolve.

## Stage 1b — modelling servo load, if it is to become an observation

Proposed 2026-08-28: give the policy **mean drive load** and **differential
drive load** (the load-side duals of the `hub` and `diff` commands) plus
**steering current** from the XC330. This section is the "how would we simulate
that honestly" half, because putting it in training before the model is pinned
repeats a mistake this project has already paid for once.

### What the sim can already produce

MuJoCo gives `data.actuator_force` per actuator, post-clamp to `forcerange`.
That is the direct analogue of both readings:

    actuator_force / forcerange   -> fraction of max torque -> XC430 Present Load
    actuator_force / kt           -> amps                   -> XC330 Present Current

`bike_params.yaml` already carries `stall_torque` and `stall_current` for both
parts, with a note that kt is voltage-independent. kt = 1.143 N·m/A (XC430),
0.909 (XC330). So the forward model is short; all the work is in the gaps.

### The gaps, and the bench test that pins each

| gap | in sim today | test |
|---|---|---|
| torque-speed droop | **absent** — `forcerange` is flat at every speed | servo §3 |
| loop stiffness / integral action | `drive_kv` 0.016 derived, never measured | servo §4 |
| gearbox friction and efficiency | only as joint `frictionloss`/`damping`, both GUESS | omni §7.2 spin-down |
| no-load current offset | **absent entirely** | servo §5 |
| the internal filter | **absent** | servo §1 step response |
| quantisation | absent | free — 0.1 % / 1 mA, datasheet |

Servo §6 already records that the expected outcome (droop AND integral action)
is not expressible in MuJoCo's affine actuator and needs a callback or
`dyntype`. That cost lands BEFORE any of this becomes an observation, and it
moves `params_digest`.

### THE PREREQUISITE, and it is not optional

Mean and diff load are not restatements of the commands — they are the plant's
REACTION. That makes them the closest thing to a contact-force sensor this bike
will ever carry, and it means they are most informative exactly where the
contact model is least trustworthy.

**So training on simulated load before the contact is calibrated would
specialise the policy to a GUESSED contact** — precisely how
`general_rl_odo_ahrs` specialised to a guessed `ahrs_tau` of 2.0 and gave back
half its gain at the measured 0.19. Stage 0c is a hard prerequisite for
load-in-the-observation, not a nice-to-have. And whatever ships should
RANDOMISE the load-model constants that remain unmeasured, for the same reason.

### One caution on the mean/diff split

Mean load is fine. **Diff load is a difference of two independently INFERRED
estimates**, so its noise is ~sqrt(2)x each — and worse, any unit-to-unit bias
in the inference does not cancel, it lands entirely in the difference. `diff`
is the channel doing the balance work and is already p95-saturated under a hold
command, so the channel you most want arrives dirtiest.

The XC330's MEASURED milliamps do not have this problem; the XC430's inferred
percentage does. If only one load channel is affordable, steering current is
the trustworthy one and mean drive load is the informative one.

## Stage 2 — chassis / frame

Unlocks `chassis.mass 0.45` (the big one), `fork_mass 0.025`, and all of
`omni-wheel-protocol.md` §6 — wheelbase, rake, fork offset, and the position of
every component over ~5 g in the chassis frame. That last item is what turns
the CAD layout from a drawing into `bike_params_cad.yaml` being authoritative,
which is CAD-track step 2 on the critical path.

**This is also the first stage that moves `plant_digest`**, so every trained
policy becomes provisional at this point. Expect to retrain, and prefer to
arrive here with the contact and drivetrain numbers already in hand so it is
ONE retraining rather than three.

## Stage 3 — steering, then the whole bike

Everything above is bench work with a wall socket. Only after it does the
untethered path (pack, Pi, buck, wiring) buy anything, and `untethered-setup.md`
already specifies it.

---

## Explicitly NOT first: the self-righting wings

They are a complete, verified DESIGN, and they are operated separately from the
policy — four training runs established the general policy should not drive
them, because episodes terminate at `fall_roll_deg` 60 and the fallen state the
wings exist for is outside the training distribution by construction. They also
need `RECOVER_DEG` re-derived at whatever envelope the linkage settles at.

None of that is unblocked by printing them early, and printing them early costs
the most filament of anything here.

---

## The through-line

Stage 0 answers "is the sim lying to me about the actuator, and about the
floor". Those are the two places where sim-to-real failure would be silent and
total. Everything after it is refinement of a model that has already been
checked in the two ways it could be most wrong.

If only one thing gets done: **0b**. It is the only test on this page whose
result could mean "stop training and change the reward".
