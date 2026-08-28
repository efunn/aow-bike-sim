# Drive Servo Velocity-Loop Measurement Protocol

Companion data sheet: `servo-measurements.yaml` (same section numbering).

Everything here calibrates **the drive actuator model**, which today is two
config lines and one absence:

```yaml
actuators:
  drive_kv: 0.5        # torque per rad/s of velocity error, at the input shaft
  # (no bandwidth limit exists — the model has no actuator dynamics at all)
```

Status: **both are placeholders and both are wrong by ~31x in the same
direction.** `mujoco-modeling-decisions.md` deferred this as "placeholder for
Dynamixel velocity-PI emulation at the real control rate (later phase, after
testbed system-ID)". This is that system-ID, specified.

Servos: 2x **XC430-W150** in velocity mode (`MODE_VELOCITY = 1`), driving the
input shafts through a `belt_ratio` of 3. `hw/dynamixel.py` already talks to
them and already reports `Present Velocity` — the servo is its own instrument
for most of what follows.

## Why this is worth doing

The sim's drive plant says yes to commands the hardware cannot execute, and the
policies have learned to live there. Commanded to **stand still** for 15 s,
`general_rl_smooth_diff_og` reverses the hub direction **21.8 times a second**
and runs **6.37 m of rim** past the contact patch while the bike moves 0.35 m.
Full derivation in `docs/plans/aow-contact-approximations.md` §6b; the two
defects, both from the datasheet block in `bike_params.yaml`:

| | model | hardware | ratio |
|---|---|---|---|
| velocity-loop stiffness | `drive_kv` 0.5 | bare-motor droop `kt·ke/R / belt²` = 0.0160 | **31x** |
| velocity-loop settling | `J/kv` = 0.60 ms | `tau_m = J·R/(kt·ke)` = 18.7 ms | **31x** |

Neither can be fixed by editing a number — §6b records both single-parameter
attempts and why each broke the suite. **What the fix needs is exactly what
these tests measure**, so do not guess a value from this document's tables;
measure, then decide the model form from what comes back (§6).

## What you need

- One XC430-W150, powered at the pack voltage the bike will actually run
  (**record it** — kt is voltage-independent but no-load speed is not).
- U2D2 or equivalent, `hw/dynamixel.py`'s bus at 3 Mbps with
  `Return Delay Time` already zeroed (it does this in `_configure_modes`).
- A known inertia on the output: the actual omni wheel is ideal, since that is
  the load the model cares about. Otherwise any disc whose inertia you can
  compute.
- For §3–§4 only: a way to apply a **known steady torque** — a lever arm of
  measured length with hanging masses is enough, and is better than a friction
  brake because the torque is known rather than inferred.

**Set `Profile Velocity` and `Profile Acceleration` to 0 before anything
below.** Non-zero profiles make the servo run its own trajectory generator, and
you would be measuring that instead of the velocity loop. This is the single
most likely way to get a confidently wrong answer here.

Log `Present Velocity` (128) and `Present Position` (132) against
`Realtime Tick` (120), which is the servo's own clock and immune to bus jitter —
`hw/dynamixel.py` already reads exactly this block.

---

## 1. Velocity step response → the bandwidth (`drive_tau`)

Wheel free-spinning, off the ground.

1. Torque on, `Goal Velocity` = 0, let it settle.
2. Step `Goal Velocity` to a target, hold 1 s, step back to 0. Log throughout.
3. Repeat for targets of **2, 5, 10, 20, 50, 100%** of no-load speed.

Fit a first-order response to each rise. Report the time constant per step
size, because the two regimes answer different questions:

- **Small steps (2–10%)** stay inside the current limit and give the true
  closed-loop bandwidth. This is the number `drive_tau` should take.
- **Large steps (50–100%)** saturate current and are slew-limited instead. The
  rise is a ramp, not an exponential — report the slope (rad/s²), which is a
  direct check on `input_armature` (predicted 1778 rad/s² at the input shaft).

Expect the small-step constant to be **at or below 18.7 ms**. It cannot exceed
it by much — that is the motor's own electromechanical time constant, which the
firmware PI can partially compensate but not escape. If it comes back well
above 18.7 ms, suspect a non-zero profile (see above) before believing it.

Also record **command latency** separately: time from the bus write completing
to the first non-zero `Present Velocity`. That is transport plus firmware loop
period, and it is additive to the time constant, not part of it.

## 2. The reversal test → does the hardware do what the policy does

**This is the highest-value test here** and needs no load fixture. It measures
the exact behaviour the sim permits and the hardware is suspected not to have.

1. Wheel free-spinning.
2. Command a square wave on `Goal Velocity` between `+w` and `−w`, at **25 Hz**
   (the observed chatter rate — 21.8 hub reversals/s).
3. Sweep `w` over 10, 25, 50% of no-load speed. Log velocity and current.
4. Repeat at 5, 10, 15 Hz to find where tracking falls apart.

Report, per frequency: **peak-to-peak velocity actually achieved** as a
fraction of the commanded 2w, and the rim distance travelled per second.

The sim's answer is 6.37 m per 15 s at the contact patch. If the hardware
achieves a small fraction of the commanded swing at 25 Hz — which is the
expectation — that number is the size of the sim-to-real gap in the channel the
policy leans on hardest, and it is the headline result of this whole protocol.

## 3. Torque–speed envelope → is a flat `forcerange` defensible

The model allows **full stall torque at every speed**. A real DC motor's
available torque falls linearly to zero at no-load speed. This tests which.

**CORRECTION (2026-08-28). THIS SECTION ASKED FOR A REGISTER THE DRIVE SERVO
DOES NOT HAVE.** Step 3 read "record steady `Present Velocity` and `Present
Current`", and the `kt` below is 1.6/1.4, i.e. the XC430-W150's own number — so
the section is unambiguously written for the drive servo. **The XC430 has no
`Present Current` register at all.** Its address 126 is `Present Load`, a
signed percentage of max torque in 0.1% units; `Present Current` (1.0 mA) is
the XC330-T181's register at the same address and width. The XC430 doc has zero
mentions of `Present Current` or `Current Limit`; the XC330 has fifteen of the
latter and supports current-based control. So the XC330 MEASURES amps and the
XC430 INFERS a percentage, and as written this test could not be run.

**The repair is a simplification.** The lever arm already applies a KNOWN
torque, so current was only ever a convenience for reading torque back. Drop it
and use the lever arm directly:

1. Command `Goal Velocity` = maximum.
2. Apply increasing load torque via the lever arm, in steps, letting each
   settle.
3. At each step record the applied torque (from the lever arm and the hung
   mass) and steady `Present Velocity`. Plot torque against speed.
4. Record `Present Load`(126) at each step as well. It is not needed for the
   plot — it is CALIBRATED BY it, which is the more valuable output: a
   lever-arm torque against a reported percentage is the only way to turn
   `Present Load` into N·m, and nothing else on this bike can do it.

A straight line from stall at zero to zero at no-load speed confirms the droop
and means the flat `forcerange` is wrong. Anything flatter means the firmware
is current-limiting differently and the model is closer than feared.

`kt` = stall_torque / stall_current = 1.143 N·m/A at the XC430's output is kept
here as the datasheet-implied value, and is now only needed if you measure
current EXTERNALLY (bench supply readout or an inline shunt). Verify it from
your own stall measurement rather than assuming the datasheet. For the XC330,
kt = 0.80/0.88 = 0.909 N·m/A and `Present Current` gives it to you directly.

## 4. Droop and integral action → does `drive_kv` need to stay stiff

This is the test that decides whether the model can use a low `drive_kv` at
all. In sim, lowering it to the derived 0.016 broke `test_rest_stability`,
because open-loop voltage control cannot hold against back-drive.

1. Command a mid-range `Goal Velocity`.
2. Apply a **known steady** load torque.
3. Record steady-state velocity, and the time to return to it.

- Velocity returns to the commanded value → the PI has **integral action**, the
  loop is stiff in steady state, and `drive_kv` must stay high.
- Velocity settles low by a repeatable amount → measure the slope
  `Δtorque / Δvelocity_error`. **That slope is `drive_kv`**, measured, at the
  servo output; divide by `belt_ratio²` for the input shaft.

## 5. Holding at zero → the rest-stability case

1. `Goal Velocity` = 0, torque on.
2. Apply a disturbance torque via the lever arm, then remove it.
3. Record whether the output shaft holds, creeps, or back-drives, and the
   steady position error under load.

The bike stands still on this behaviour. Any model that cannot hold here will
fail `test_rest_stability` no matter how good its bandwidth number is.

---

## 6. What the results decide

The model form follows the measurements, not the other way round:

| §4 integral action | §3 droop | what the sim needs |
|---|---|---|
| yes | yes | stiff `drive_kv` **and** a speed-dependent torque limit — not expressible in MuJoCo's affine actuator, so a callback or `dyntype` |
| yes | no | stiff `drive_kv` + `drive_tau` from §1. Two config lines, and `linearize.py` needs the actuator state before the lag can be enabled |
| no | yes | low `drive_kv` = the §4 slope, flat `forcerange` stays. Simplest outcome |
| no | no | the current model, and §6b of the plan doc is wrong |

Row 1 is the expected outcome and the most work. §2 is what tells you whether
any of it matters enough to do.

Whatever lands, it moves `params_digest` and invalidates every policy in
`moves/` — work `CLAUDE.md`'s "Before changing a physical parameter" list, and
budget a retrain.
