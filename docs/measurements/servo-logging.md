# What to record from a Dynamixel

> **Status: ACTIVE, reference.** Written 2026-09-13 from the drivetrain station
> captures (2x XC430-W150, `drivetrain-measurements.yaml`) and the 09-01 servo
> bench (XC330-T181, `servo-measurements.yaml`). Every verdict below names its
> evidence. Revisit when a new servo model joins the bus, or when the 9 V load
> test (below) runs.

Two questions, answered separately because the answers differ: what a **bench
capture** should record (everything that could matter later, since a re-run
means holding the rig up again), and what the **onboard control loop** should
read (the minimum that feeds a decision, every 20 ms, on a Pi).

## Per frame

One FastSyncRead through indirect block 1. **The block is 28 bytes for reads
AND writes together**, and the XC330 has no second block, so a mixed bus lives
inside that budget (`IndirectMap` enforces it).

| register (addr, bytes) | bench | onboard | why |
|---|---|---|---|
| Realtime Tick (120, 2) | **yes** | **yes** | The servo's own ms clock, sampled with the encoder: the only timing truth. 1 ms resolution, so at 500 Hz a single-frame dt is 2 +- 1 ms; difference over spans. |
| Present Position (132, 4) | **yes** | **yes** | The velocity source. Differenced over a span it beats the firmware's velocity on lag. Single-turn 0-4095 in velocity and PWM mode: unwrap along the shortest path. |
| Present PWM (124, 2) | **yes** | **add** | The controller's output, i.e. what is being applied. It reacts in the **same frame** the new goal lands (65 of 90 step edges, 14 more one frame later). The one effort signal with no delay; saturation shows here first. |
| Hardware Error Status (70, 1) | **yes** | **add** | An overload latches and drops torque mid-run with no error anywhere else; every later frame reads a motionless shaft as if it were a result. |
| Present Velocity (128, 4) | yes | optional | The firmware's filtered estimate: ~22 ms to react to a step (median 11 frames), 2.9x extra attenuation at 40 Hz on the bare shaft. Worth logging to keep measuring that filter; **do not control on it**. |
| the goal register, read back (104 Goal Velocity, 4; 100 Goal PWM, 2) | yes | optional | A SyncWrite returns no status, so reading the goal back is the only per-frame proof a write landed. Point one read slot at the same register the write slot writes. **Torque on only** for a goal: with torque off the servo acknowledges a Goal Velocity write and keeps 0. Measured: the goal reads back the **next frame**, 100 %, paced and unpaced. **Velocity Trajectory (136)**, which the first captures read instead, is NOT a copy of it: it is the servo's own velocity reference and follows **~4 ms after the frame that computed the command** (4.1-4.5 ms by cross-correlation over a chirp), reaching a square step 3 frames later through intermediate values even with profiles at 0. Measure that once; it does not need re-reading every frame. |
| Present Input Voltage (144, 2) | yes | slow poll | 0.1 V steps. Sagged 11.9 -> 11.4 V under load on a 12 V brick. No-load speed and stall torque scale with it; onboard it is the battery gauge. About 1 Hz is plenty. |
| Present Temperature (146, 1) | yes | slow poll | Winding resistance rises with it; one session went 27 -> 43 C. |
| Bus Watchdog (98, 1) | yes | slow poll | Reads 255 once tripped, after which goal writes are **silently** ignored (SyncWrite returns no status). |
| Present Load (126, 2), XC430 only | only for the 9 V test | **drop** | Not information: **Present Load = LPF_50ms(duty - omega/11 rad/s)**, delayed a further 12 ms, R^2 0.98-0.99. Rebuilt from PWM and position it arrives **48 ms earlier**. ROBOTIS calls it inferred, not measured. |
| Present Current (126, 2), XC330 only | yes | pending | Measured mA, but failed a consistency check on 09-01 (implied stall 10-11 A against 0.88). Stall calibration (R6) before any number is used. |

Maps in use:

| map | read | write | total |
|---|---|---|---|
| drivetrain bench (`analysis/drivetrain_bench.py`) | Tick, Position, Velocity, PWM, Load, Voltage, Temperature, Error, Bus Watchdog = 19, + the goal read back (4 / 2) | Goal Velocity 4 / Goal PWM 2 | 27 / 23 |
| onboard today (`ServoBus.READ_BLOCK`) | Tick, Position, Velocity = 10 | goal 4 | 14 |
| onboard proposed | + PWM, Error = 13 | goal 4 | 17 |

The proposed onboard map is a proposal: `ServoBus` feeds the `HardwareData`
shim, so adding to it is a spec change on both sides, not a one-liner.

## Per frame, host side

Recorded by `aow_sim.hw.bench_log.record` alongside the registers:

- host time at the start of the read, and the read and write durations -- a
  GC pause or a slow write shows up here and nowhere else
- the command sent to **each servo**, in the write register's units (not the
  common/diff command it came from, so a wrong sign convention mislabels a
  segment but corrupts nothing)
- a dropped-frame flag: a failed read is kept and masked, not fatal

## Per session

- **A configuration snapshot before and after**: operating mode, drive mode,
  gains, limits, profiles, return delay, watchdog, firmware
  (`dynamixel.CONFIG_REGISTERS`). Writing Operating Mode silently resets gains
  (`hw/control_tables/README.md`), and "after" is how a sweep proves it
  restored what it changed.
- The git commit and dirty paths; the params the analysis will need, **with
  their `source:`**; argv.
- A note of the rig: what is attached, how it is held, anything applied by
  hand. The servos cannot record that.

## How to store it

**Raw register fields, with each register's address, size, unit and sign
stored beside them**, decoded on load. On 2026-09-13 Velocity Trajectory turned
out to have no unit in ROBOTIS's table, and the first analysis compared counts
with rad/s. Because the fields were raw, that cost a re-decode, not a re-run.
The same gap existed for Velocity Limit before that.

## Traps that each cost a capture

- **macOS: run `adjust-ftdi-latency` after every U2D2 plug-in.** Without it
  every read took **16.0 ms** (63 Hz) on 2026-09-12; after it, 1.3-1.6 ms.
  `assert_low_latency` only checks the Linux sysfs file, so on a Mac nothing
  fails. Read the summary's `read ... ms median` line. Do not set
  `IOSSDATALAT` from Python instead: it broke communication until a replug.
- **Profile Velocity and Profile Acceleration at 0**, or a step measures the
  firmware's trajectory generator. **Return Delay Time 0**, or the servo waits
  500 us before each answer. `DynamixelBus.prepare` does both.
- **Gains after the mode, then read back.**
- **A watchdog trips in your own pauses.** Converting an 89 s capture after its
  last frame was long enough to trip a 100 ms Bus Watchdog. Stop traffic
  cleanly first (`record(..., finish=...)`).
- **Verify the sign convention by eye before believing common/diff labels.**
- **A unit missing upstream decodes silently as raw counts.** Check a new
  register's decoded magnitude before using it.
- **With torque off, Goal Velocity and Goal Position writes are acknowledged
  and discarded** (`rc=0 err=0`; Goal Velocity reads 0, Goal Position keeps its
  old value). Goal PWM and Profile Velocity hold. A torque-off write check has
  to use a register that holds -- `idle --write` uses Profile Velocity.
  `hw/control_tables/README.md` has the table.

## What to derive, not record

| quantity | from | note |
|---|---|---|
| shaft velocity | Position over a span T | quantisation noise is one count / T; 1 count = 1.53 mrad at the servo shaft |
| effort | `duty - omega / omega_nl` | what Present Load is, without its 50 ms filter. `omega_nl` 11 rad/s at 12 V; whether it should scale with the supply is the open 9 V question |
| torque | effort x stall torque | **uncalibrated**: datasheet stall torque, no gearbox losses. The lever-arm test turns it into N.m |
| hub and roller motion | the two servo angles, `belt_ratio`, `drivetrain.mix_*` | inferred, not measured -- belt stretch is invisible to it |

## Open

- **Sensed or computed Present Load.** The fit cannot tell: a sensed winding
  current obeys the same formula at one voltage and temperature. Across
  33-40 C the slope stayed flat, where copper predicts ~-2.5 %, so it leans
  computed. Settle it by repeating any capture at ~9 V: computed still crosses
  zero at 11 rad/s, sensed near 8.3.
- Whether to read Present Current on the steering XC330 onboard waits on R6.
