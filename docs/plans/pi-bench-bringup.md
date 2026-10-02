# Pi bench bring-up — the full loop on a separate controller

> **Status: ACTIVE, written 2026-09-15. Nothing here has been run.**
>
> This doc owns ONE session: `hw/run_bike.py` running on a Raspberry Pi 3B+
> with four servos and a TM151, on the bench, with no chassis. It is the
> delta layer over `untethered-setup.md`, which owns the onboard design and
> is not repeated here. Where the two disagree about hardware, this doc is
> describing what is on the desk and that one is describing the flying bike.
>
> **The seven code gaps in §2 are CLOSED (2026-09-15) and none of them has
> touched hardware.** They were written, tested against 24 new tests, and
> nothing in them has seen a servo. Treat §2 as "what to look for when it does
> not work", not as "this works".

## 1. What is on the bench, and what it stands for

| on the desk | stands in for | what it can and cannot prove |
|---|---|---|
| Pi 3B+, 5 V 2 A USB brick | Pi Zero 2 W on the pack | timing, install path, driver gotchas. **Not** the Zero's USB OTG budget, and not the pack's rail |
| 12 V brick → U2D2 Power Hub | the 3S pack + splitter | everything except the LVC failsafe, which 12 V can never trip |
| 2× XC430 in the rear wheel harness | the rear drivetrain, as built | real belt, real differential, real inertia — with the wheel in the air |
| 2× XC330, bare | steer + self-righting | electrical and protocol path only. No fork, no trail, no wings, no load |
| TM151 over USB | TM151 on the GPIO UART | the sensor, the parse, the rate. **Not** the mounting calibration, which needs a chassis |
| MacBook | ground station | code sync, export, Wizard/ImuAssistant config |

What the bench CAN answer, and the only reasons to run it: **does the loop
close at rate on this hardware**, and **do the signs come out right end to
end**. It cannot say anything about balance — there is no bike.

## 2. Seven gaps between the code and this session

Ordered by what blocks first. Each one was a claim about the tree on the
morning of 2026-09-15, and each is followed by what was done about it. The
observation is kept because it is the part worth re-reading when the bench
disagrees; the file:line is where to look.

| # | was | now |
|---|---|---|
| 2.1 | no ground station; `run_bike` tripped `command link dead (inf s)` on tick 1 | `hw/ground.py`, a terminal station over UDP; `run_bike` WAITS for it, and `--no-link` is the bench escape |
| 2.2 | no gains written; the mode write erased the position gains every startup | read-before-write on the EEPROM pair, gains written after and read back, sourced from the policy's own record |
| 2.3 | `ServoBus` was a 3-tuple | optional `righting_id`, same SyncRead/SyncWrite, `control.onboard.righting_id` |
| 2.4 | a fall ended the process | `FallGuard`: cut, hysteresis, dwell on angle AND rate, consent, full re-engage |
| 2.5 | `--rate` did not reach `RateFilter` | `control_hz` passed to `ServoBus` |
| 2.6 | torque on before preflight | `ServoBus.open()` leaves torque off; `arm()` is explicit, after preflight |
| 2.7 | `assert_alias_margin` never called | called from `ServoBus.open()` |

**What is NOT fixed and is not a code question**: steer homing (§2.6), the
mounting calibration, and `CMD_STALE_S`. See §8.

### 2.1 `run_bike` cannot run without a ground station — it trips on tick 1

`CommandLink.cmd_t` starts at `0.0`, `age()` returns `inf` while it is falsy,
and `_check_failsafes` is reached before any command can have arrived
(`hw/run_bike.py:303`). With nothing sending UDP to port 9910 the first
iteration prints `FAILSAFE: command link dead (inf s)` and shuts down.

**Nothing in the repo sends to that port.** There is no ground-station client:
teleop lives inside the MuJoCo viewer's key callback (`run_drive.py`), which
is not reusable off a rendering host.

**Done both ways.** `hw/ground.py` is a terminal program — raw-mode stdin,
50 Hz of JSON datagrams, telemetry printed on one line — so it runs over ssh
from anywhere including the Pi itself, against `127.0.0.1`. The radio is then a
separate experiment with its own failure modes, which is the whole reason to do
it this way for session one. `run_bike` now *waits* for the first command
rather than starting deaf (`wait_for_link`), because the watchdog should mean
"the operator went away", which is only meaningful once they have been there.
`--no-link` turns the link failsafes off for a bench run with no operator.

**It binds the SAME KEYS as the viewer (2026-09-16).** The first version used
`w`/`s`/`a`/`d` and space, which nothing else in the repo uses: teleop is
arrows for throttle/brake and turn, and `/` for zero-command. Two UIs for one
bike, each with its own muscle memory, is a mistake that only shows up under
pressure. The arrows and `/` are now the primary binding and the letters are
aliases, kept because an escape sequence has more ways to go missing over ssh
than a letter does. `/` also does what teleop's `/` does in full — zero the
velocity AND re-aim the heading command at where the bike actually points —
which needed the bike to start reporting its own `psi` in telemetry. Same
packet now carries `righting_current`, so `[` and `]` step from the value that
is on the servo instead of from zero.

**There is still NO GRAPHICAL station.** The terminal one is what exists; the
gap is recorded in `untethered-setup.md`.

The struct lost a field on the way: it carried `"mode"` and the bike never read
it. `general_rl` is the only deployable controller, so a mode key was a control
the operator would believe they had.
`test_every_field_the_station_sends_is_read_by_the_bike` is what found it, and
what stops the two halves drifting again.

### 2.2 The firmware gains are never written, and the mode write erases half of them

`ServoBus._configure_modes` (`hw/dynamixel.py:1094`) writes Return Delay Time
and Operating Mode at every `open()`. It writes **no gains at all**. Two
consequences:

* **Velocity P/I (78/76) are RAM registers** — the X-series EEPROM ends at 63.
  They reset to their power-on defaults every time the servo is powered, so
  setting them in DYNAMIXEL Wizard does not make them stick. Whatever the
  factory default is, that is what the bike flies. *Verify this on the bench
  in ten seconds: write P 400, power-cycle, read it back.*
* **Writing Operating Mode(11) silently resets Position P and D**, measured
  2026-09-01 and documented in `hw/control_tables/README.md`. So the steer's
  position gains cannot be set ahead of time by any route: `open()` rewrites
  the mode on every startup and the gains go back to 900/0 behind it.

This is the most consequential silent setting on the bike, because the policy
is gain-specific: on the fitted drivetrain the P 100 seeds score 0.741–0.751
on their own gain and 0.180–0.503 on the other one (`docs/status.md`). Flying
`general_rl_drivetrain_p100_1` against a servo that came up at P 400 is the
mismatch case, and nothing anywhere would say so.

**Done.** `_configure_servos` reads Return Delay Time and Operating Mode
first and writes only on a difference — which makes the startup idempotent and,
more to the point, stops it destroying the position gains on every run. The
gains go in after, are read back, and a mismatch raises. `resolve_gains` picks
the source in the order teleop already uses for the same decision: an explicit
`--servo-gains P:I` beats the policy's own training gain, which beats
`control.onboard.gains`. The chosen source is printed at startup, because a
startup silently picking one of three sources is how the old `engage_general`
bug happened.

**Correction to the framing above**: P 100 / I 1920 IS the factory default, so
the common case is a P 100 policy accidentally matching a servo nobody
configured. The exposure is the other direction — a P 400 policy, or a bench
script that left the gains somewhere else — and the point stands either way:
the gain must be asserted rather than assumed.

### 2.3 `ServoBus` is three servos; the self-righting XC330 is a fourth on the same chain

`ServoBus.__init__` takes `ids=(1, 2, 3)` and unpacks them into
`id_a, id_b, id_steer` (`hw/dynamixel.py:1012`). The righting servo has
nowhere to go.

Do **not** solve it with an out-of-band direct write from the control thread:
a single register write is another USB round trip, ~0.5–1.5 ms, which is a
whole tick's budget (§4).

The clean fix is to put it in the same map. Current-based position mode (5)
takes its goal in **Goal Position(116)** — the same register the steer uses —
so it maps to the same indirect write slot and costs one more 4-byte param in
the SyncWrite that already happens. Goal Current is a torque cap set once at
startup, not per tick. Work to do: a role-keyed `ids` dict instead of a
3-tuple, `_wraps` (extended position does not wrap), `to_controller_units`,
and a righting command field in the link protocol.

**Done**, with one correction. The claim that an out-of-band write costs
~0.5–1.5 ms was wrong: that figure is a *read* round trip, where the request,
the status packet and the FTDI latency timer all appear. A SyncWrite is a
broadcast with no status packet, so a second write-only transfer costs its wire
time and a USB scheduling slot, not a round trip — which is why ten servos at
500 Hz is an ordinary thing to do. M2 measures it rather than settling it by
argument.

**MEASURED 2026-09-15**, four servos on the Mac at 3 Mbps, FTDI latency timer
2 ms, torque off, 300 iterations each:

| | mean | p50 | p99 |
|---|---|---|---|
| FastSyncRead, 4 servos | 2.000 ms | 2.000 | 2.034 |
| one SyncWrite, 4 servos | **0.025 ms** | 0.025 | 0.031 |
| two SyncWrites, 3 + 1 | 0.177 ms | **0.037** | 0.069 |
| read + one write (a whole tick) | 2.000 ms | 1.999 | 2.042 |
| **read + TWO writes** | **2.000 ms** | 2.000 | 2.022 |

A SyncWrite costs 25 µs and a second one is free inside the read's latency
window — a tick is 2.000 ms either way. The read is the entire budget, and the
read is the latency timer: 2.00 ms measured against a 2 ms timer is not a
coincidence, and the same bus ran ~16 ms/frame before `adjust-ftdi-latency`
(`test_the_tick_advances_and_paces_the_capture` failed at 63 frames/s, then
passed). So the whole bus question is the latency timer and nothing else, and
the Pi's udev rule setting it to 1 is the single most load-bearing line in the
OS config.

It went in the shared block anyway, because it is nearly free there and because
a policy may drive this servo later. Goal Current stayed out, and NOT for
timing reasons: the XC430-W150 does not have the register — it has Goal
PWM(100) and Present Load(126) where the XC330 has Goal Current(102) and
Present Current(126) — so no shared indirect layout can include it.

**What an absent register actually does.** Probed on the bench 2026-09-15,
all four servos, torque off. The FIRST version of this probe wrote only ZERO
and concluded the address was a silent black hole. That was a bad test — zero
is the one value a nonexistent register accepts — and writing something else
gives the opposite answer:

| id 101, XC430-W150 (no such register) | result | reads back |
|---|---|---|
| `write2 @102 = 123` | rc=0 **err=4 (data range)** | 0 |
| `write2 @102 = 4095` | rc=0 **err=4** | 0 |
| `write2 @102 = 0` | rc=0 err=0 | 0 |
| `read2 @102` | rc=0 **err=0** | **0** |

| id 103, XC330-T181 (Goal Current) | result | reads back |
|---|---|---|
| `write2 @102 = 123` | rc=0 err=0 | **123** |
| `write2 @102 = 4095` | rc=0 **err=6 (data limit)** | 123 — clamped by Current Limit(38) = 910 |
| `read2 @102` | rc=0 err=0 | its goal current |

So it is **not** blank read/write RAM that only the XC330 firmware interprets.
On the XC430 there is nothing behind the address: writes of anything but zero
are REFUSED with a data-range error, the value never sticks, and a reboot
changes nothing. Address 900 — past the end of the table on both — gives err=7.

The asymmetry is the thing to take away: **writes tell the truth and reads do
not.** A read of an absent address returns `rc=0 err=0 value=0`, with
`Hardware Error Status(70)` clean. So a mixed indirect READ block containing
102 returns plausible zeros from the XC430s and says nothing. And a mixed
WRITE block is worse than the direct write measured above, because SyncWrite is
a broadcast with no status packet — the err=4 that a direct write reports is
simply not on the wire. Same family as the firmware-50 indirect trap in
`hw/control_tables/README.md`. Keeping Goal Current out of the shared block is
right, and a separate write to the one servo costs 25 µs (measured above). `set_righting_current` is a separate write taking RAW counts,
because ROBOTIS's e-manual (~1 mA/LSB) and the vendored model file (6.709e-4,
unit_name "N/m") disagree about what the register means and Station C R6 is the
test that settles it.

The righting servo is driven by hand, latch-a-target, exactly as teleop already
does the wings — and teleop's `manual` flag is the precedent to copy when a
policy eventually wants the channel back.

### 2.4 A fall is terminal; there is no re-engage

`_check_failsafes` returns a reason and `run()` breaks out of the loop
(`hw/run_bike.py:303`). Roll > 60° is treated exactly like a dead link.

**Design note, because the framing matters.** "Cut the policy past the
unrecoverable roll" is two different rules:

* *Unrecoverability* is **not a roll angle.** `analysis/no_return.py` measured
  the no-return state at roll −3.2…13.6° with the rear contact already
  travelling in the catch direction — what has run out is crawl authority, not
  lean margin, and the bike still looks upright. The recoverable set is a curve
  in (roll, roll rate) and it moves with speed. A single angle cannot express
  it. See `self-righting.md` §1.
* *A safety cut* — stop thrashing while lying on the floor — **is** a plain
  angle, and 60° is the right one because it is `fall_roll_deg`, beyond which
  the policy never trained and is extrapolating.

So implement the safety cut, and do not call it unrecoverability. What it
needs beyond a threshold:

| | why |
|---|---|
| hysteresis (cut 60°, re-arm ~30°) | a bare threshold chatters at the boundary |
| a dwell (|roll| under the re-arm angle AND |roll_rate| small, for ~0.5 s) | a bike swinging through upright is not a bike ready to drive |
| `ctl.reset()` + `engage_general()` on re-engage, not a bare resume | the policy carries `prev_action`, the 50 Hz ZOH schedule and the velocity filters; resuming feeds it stale state |
| `est.reset()` and the `RateFilter`s cleared | same argument, on the estimator |
| the command re-anchored to zero velocity | the operator's last command predates the fall |
| `_sense()` keeps running while cut | roll is how you decide to come back |
| an operator veto | re-engaging on its own in someone's hands is worse than waiting |

**Done**, as `FallGuard` — a pure state machine with no clock and no I/O, so
the rules are testable at a desk, which matters for the one piece whose bugs
only show up on a bike that is already falling. The cut angle and all three
re-arm conditions live in `control.onboard`, which moves neither digest
(verified: `plant_digest` e1ec36bfa670217e and `design_digest` 2db6c647ff3a2d59
are both unchanged by the block).

On a cut it torques off **only** `bus.policy_ids` — the two drives and the
steer — and leaves the righting servo live, since lying down is the one moment
that servo has a job. Re-engaging runs the full `_engage()`: a fresh
`VelocityEstimator`, `ctl.reset`, `engage_general`, command re-anchored to
zero.

**THIS ARCHITECTURE ALREADY EXISTED AND THE FIRST VERSION OF THIS SECTION SAID
IT DID NOT.** `control/righting.py` has run `lift -> balance -> retract` since
`eab90c9` (2026-08-14, *"righting: detect falls but keep the policy live"*),
with a `keep_policy` switch deciding whether the policy is cut while the bike
is down or runs throughout, and a hand-off test that is **measured rather than
chosen**:

    RECOVER_DEG  = 12.0   # inside the policy's cold recoverable set on BOTH
                          #   sides at standstill -- 16.3 right, 11.8 left, so
                          #   the weaker side sets it (analysis/no_return.py)
    HANDOFF_RATE = 3.0    # rad/s; inside the window but still moving fast is
                          #   a bike on its way past, not a hand-off

The search that missed it looked in `run_drive.py` and the training envs, which
is where teleop's manual **respawn** and the `fall_roll_deg` episode
termination live — both "start over", because in sim you can — and stopped
there. `git log -S"re-engage"` finds it in three commits.

`FallGuard` now takes both constants as its re-arm defaults, and
`tests/test_hw_runbike.py` asserts they stay equal to the sequencer's. The
numbers they replaced (30 deg, 60 deg/s) were invented, and 30 deg is outside
the policy's recoverable set on its weak side by a factor of three — which is
exactly the kind of thing measuring instead of choosing is for.

The code is still not shared: `righting.py` imports mujoco, drives a MuJoCo
actuator and schedules a stroke against a model. When the mechanism is built,
the whole sequence wants porting onboard and this guard becomes its `balance`
phase. `keep_policy=True` is also already the answer to "eventually we may keep
the policy engaged while self-righting happens" — that mode exists, it is just
the other branch of the same switch.

### 2.5 `--rate` does not reach the velocity filter

`BikeRunner.__init__` builds `ServoBus(self.params, port=port)` without
`control_hz` (`hw/run_bike.py:148`), so `RateFilter` quantises its 25 ms
window against a hard-coded 10 ms nominal tick whatever `--rate` says. Since
the bench session is a rate sweep, this bites immediately: at `--rate 200` the
filter is a 2-tap over 10 ms, not 25 ms. One-line fix, but it changes what
every rate above 100 Hz measures. **Done** — `control_hz` now reaches
`ServoBus`.

### 2.6 Torque is on before preflight, and the steer's first goal is absolute

`run()` calls `self.bus.open()`, which ends in `self.torque(True)`, and only
then runs `preflight_ahrs` under the comment *"Before any torque: the bike is
stationary here and never again"*. That comment is false as the code stands.

It matters more for the steer than for the drives. Goal Velocity comes up at
0, so the drives sit still — but the steer is in extended position mode and
`write_commands` sends an **absolute** count with no homing offset and no
ramp (`hw/dynamixel.py:1262`). Whatever the policy's first steer output is,
the servo goes there at whatever `Profile Velocity(112)` allows, which is 0 =
unlimited by default.

On this bench the steer XC330 is bare, so it is a spin rather than a bang —
which is exactly why the bench is the place to watch what it does in the
moment torque enables, before there is a fork on it. Set a `Profile Velocity`
for the session. The real fix is the steer homing routine, which is a known
open item blocked on a mechanical choice (`untethered-setup.md`, Open items).

**Half done.** `ServoBus.open()` no longer enables torque — it sets up the map,
the modes and the gains and stops — and `arm()` is a separate, explicit step
that `run_bike` calls after preflight and after the policy is engaged. The
comment in `run()` is now true at the line it is written on. The absolute-goal
step and the homing routine are untouched, so **watch the steer in the moment
`arm()` runs** and set a Profile Velocity before there is a fork on it.

### 2.7 `assert_alias_margin` is never called

Defined at `hw/dynamixel.py:345`, exercised only by
`tests/test_hw_dynamixel.py`. Its own docstring says "it runs where the
numbers are known", and it does not. At 100 Hz the margin is 40×, so nothing
is at risk today — but the check exists precisely because the margin
disappears silently if `belt_ratio`, `v_max` or the rate move, and a rate
sweep moves one of them. **Done** — called from `ServoBus.open()`, where the
numbers are known, which is what its docstring always claimed.

## 3. Deltas from `untethered-setup.md`

| that doc says | on this bench |
|---|---|
| Pi Zero 2 W, one USB | **Pi 3B+, four USB ports** behind one shared 480 Mbps hub (LAN7515, shared with Ethernet). Both serial devices on one bus |
| AHRS on the GPIO UART | **AHRS over USB.** The datasheet (V1.1.6 §Communication Interface) gives USB 2.0 full-speed **Virtual COM Port**, i.e. CDC — `/dev/ttyACM0` on the Pi, `cu.usbmodem…` on the Mac — so there is **no FTDI latency timer to fix on that port**. `AhrsReader` needs nothing but `--ahrs-port`. The datasheet also says both interfaces can be used **simultaneously**, so ImuAssistant can stay on USB while the Pi reads the UART, once there is a UART |
| §2a: `enable_uart`, `dtoverlay=disable-bt`, kill the serial console | **Skip all of it** while the AHRS is on USB. It comes back if the Zero 2 W does |
| laptop hosts the AP | **no radio at all for the first session** — run the ground station on the Pi over ssh to 127.0.0.1. Wifi and AP directionality are a separate test with its own gate (§5) |
| 3S pack, LVC at 10.2 V | 12 V brick. The LVC can never fire at its real threshold. *(2026-09-18: now `PackMonitor`, 10.5 warn / 9.9 cut on the filtered max servo reading. `--pack-warn 12.5 --pack-cut 12.2` makes it fire on the brick)* |
| Ethernet does not exist | **it does on a 3B+.** A cable to the router makes first boot and every reflash independent of wifi. Use it |

**Power is the one place the 3B+ is worse than the design target.** Its
recommended supply is 5.1 V / 2.5 A and the brick on the desk is 2 A, feeding
two USB serial devices as well. Under-voltage on a Pi is a throttle, and a
throttle is jitter. Check `vcgencmd get_throttled` before and after every
timing run — `0x0` or the numbers mean nothing. Bits 0/16 are under-voltage
now / since boot.

## 4. What to measure, and the gates

The compute is not the constraint and it is worth saying so with a number.
`DriveController.step` with the deployed policy (17→128→128→3, ~19k MACs),
built from `deploy/bundle.npz` through the hardware shim: **mean 23 µs, p99
41 µs, max 138 µs** over 2000 ticks — measured on the M4 Mac, 2026-09-15, not
on the Pi. Even at 20× that is under 1 ms against a 10 ms tick. **The USB
round trip is the budget, not the policy.**

**THE LOOP CLOSES ON THE PI, 2026-09-16.** `run_bike` ran 60 s with four
servos and the TM151 both live, policy stepping, nothing energised
(`--no-torque`). Per-phase, at 100 Hz on a 10 ms budget:

| phase | mean | p99 | max |
|---|---|---|---|
| servo FastSyncRead | 2.08 ms | 3.11 | 4.12 |
| `ahrs.latest()` | 0.01 ms | 0.01 | 0.03 |
| **`DriveController.step`** | **0.03 ms** | 0.04 | 1.85 |
| SyncWrite | 0.58 ms | 1.67 | 2.43 |
| **total** | **2.7 ms** | | |

**The policy costs 0.03 ms on a Pi 3B+** — against 0.023 ms measured on an M4
Mac, i.e. the same. Every worry about SBC compute was misplaced; the budget is
the servo bus, exactly as §4 predicted, and it uses a quarter of it.

**Four bugs stood between the first run and that number, and three were mine:**

1. **`run_bike` raced its own reader.** `run()` called `preflight_ahrs`
   immediately after `ahrs.start()`, and preflight returned on its FIRST
   `latest()` failure — microseconds later, before a byte had arrived. It
   reported a dead sensor on a working port, and in poll mode could never win,
   since the first sample costs a request round trip. Fixed with
   `AhrsReader.wait_ready()`, and preflight now samples for its whole window
   instead of bailing on the first miss.
2. **`AhrsReader` read fixed 256-byte blocks.** pyserial blocks until the block
   fills; at ~19.5 kB/s that is ~13 ms, capping the reader near 100 Hz however
   fast the sensor answered — while the round trip itself measured 4.97 ms.
   Reading `in_waiting` doubled it to 200 Hz.
3. **The jitter measurement was the jitter.** The telemetry dict recomputed
   `np.percentile(self.jitter[-500:], 99)` every tick over an unbounded list:
   1.13 ms typical, **49.5 ms worst**. Bounded `deque`, percentile at 2 Hz,
   dict built at 50 Hz to match the link rather than 100. p99 28.5 -> 0.93 ms.
4. **Cyclic GC froze every thread** -- while (3) was still true. Collections
   stop the control loop and the AHRS reader together, which is why a late tick
   and a stale sample always arrived at the same instant. `gc.disable()` took
   p99 28.5 -> 0.93 ms.

   **And then it stopped being necessary, which is the part worth keeping.**
   Once (3) was fixed the loop no longer generated enough garbage to trigger
   collections, and re-measuring over 3000 ticks showed gc on 0.14/0.73/1.65 ms
   against gc off 0.16/0.82/1.18 -- indistinguishable, with `on` marginally
   better. The workaround was REMOVED: carrying `gc.disable()` on a 1 GB board
   means any future reference cycle leaks without bound, and it was fixing a
   symptom that no longer existed. A fix that was right on Tuesday is not
   automatically right on Wednesday.

**And one that was a genuine design trap:** `poll_timeout` defaulted to 0.05,
the SAME value as `latest()`'s `max_age`. One unanswered request therefore made
a staleness trip certain — the age climbed in exact 10 ms steps, one per control
tick, from 34 to 54 ms while the control thread was never more than 0.1 ms late.
Now 0.015, three times the measured round trip and three times under the limit.

**FINAL, 30 s with four servos ENERGISED and the AHRS live: mean 0.24, p99
0.79, max 1.56 ms.** No tick over 10 ms. The gate was p99 < 1 ms.

**The 40-60 ms outlier was the instrumentation AGAIN, and the chase is worth
recording because three plausible theories were wrong.** It reproduced only
with torque on (40.31 ms, then 59.93 ms), which suggested electricity; a
separate 5 V supply for the Pi killed the brownout story; a harness doing
sense+step+write under full torque showed **zero** ticks over 10 ms; and
`pack_voltage()`, the one blocking bus call left on the control thread,
measured 1.5 ms mean / 2.35 ms max. Only when `run_bike` was made to report
WHICH tick did it fall out:

    ticks over 10 ms late: [(50, 41.1), (51, 34.6), (52, 28.1),
                            (53, 21.3), (54, 14.7), (55, 10.3)]

One stall at tick 50 and five ticks of catch-up. `k % 50 == 0` is the 2 Hz
jitter statistic, and tick 50 is its FIRST call -- `np.percentile` pulling in
its sorting machinery on an A53. Warming it before the loop starts fixed it
outright. **The instrumentation was late to its own measurement, for the second
time in one session** (the first being the per-tick percentile over an unbounded
list). Suspect the measuring apparatus early.

**The steer zero earns its place, measured:** the bare XC330 came up at
**+515 deg**, over 1.4 turns wound, because extended position mode loses its
multi-turn count across a power cycle. Without the startup capture the first
command would have slammed it from there to the policy's angle at unlimited
profile velocity. This removes the jump; it does NOT home the bike, and
**steer homing is still undesigned** (`untethered-setup.md`, Open items).

**What the run does NOT show is control quality.** With the wheel in the air
the odometry reports a steady ~0.4 m/s that never responds to correction, so
the policy oscillates against a fiction at ~1 Hz with `drive_b` pinned at its
+-33.30 rad/s clamp. That is the bench lying to it, not a policy defect, and
nothing about balance can be read from a bench like this.

**`SCHED_FIFO` made things WORSE** (mean 2.92 -> 24.51 ms): at priority 80 the
control thread preempts the AHRS reader it is waiting on. Left off; the gate is
met without it. Do not reach for it before the GC and the reader are right.

> **2026-09-18: DID NOT REPRODUCE, and SCHED_FIFO is now on.** Granted via
> `/etc/security/limits.d/90-aow-rtprio.conf` (`efun - rtprio 80`) -- scoped
> to the user, unlike the `setcap` in (c) below, which lands on the system
> interpreter. One 142 s `--no-torque` run with the mirror connected:
> tick lateness median 0.03 ms, p99 0.76, worst 1.12 (against 0.09 / 0.86-0.90
> / 1.6-3.6 without), and **0.00% of ticks reused the previous IMU sample**
> in that run and in four without it -- so the reader is not being starved,
> which was the 09-16 mechanism. The code has moved a long way since (reader,
> GC warm-up, the loop), so the two are not the same experiment. One run: if
> it regresses, the record shows it -- `bike_slip_ms`, and identical
> consecutive `bike_quat` rows for a starved reader.

**M1 and M2 are DONE on the Pi, 2026-09-16.**

| | Mac (timer 2 ms) | **Pi 3B+ (timer 1 ms)** |
|---|---|---|
| FastSyncRead, 4 servos, mean | 2.00 ms | **1.00 ms** |
| p99 | 2.03 | **1.04** |
| read-only ceiling | 500 Hz | **998 Hz** |

**The ceiling is 1000/latency_timer on both machines and the host does not
enter into it.** The Pi 3B+ — slower CPU, USB behind an internal hub shared
with Ethernet — beats the Mac, because the Mac's timer was 2 and the Pi's is
now 1. At the planned 100 Hz that is **10% duty**, and 500 Hz would be 50%. The
worry in `untethered-setup.md` that a Zero 2 W's single USB OTG would be the
constraint is not supported by anything measured here; the FTDI latency timer
is the whole story.

`pytest` on the Pi passes **7/7** — the same set as the Mac — with one
Pi-specific gotcha: `-m hardware` alone still COLLECTS the whole suite and dies
on 15 import errors, because there is no MuJoCo. Name the file:

    AOW_DXL_PORT=/dev/ttyUSB0 AOW_DXL_IDS=101,102,103,104 \
        ~/venv/bin/python3 -m pytest tests/test_hw_bench.py -q

| # | measurement | how | gate |
|---|---|---|---|
| M1 | servos answer, indirect map sticks, firmware floor | see above | **7 passed** |
| M2 | bus round trip vs baud and latency timer | `read_s` / `write_s` from a `DynamixelBus.capture`, at 1/2/3 Mbps × `latency_timer` 1 and 16, **4 servos in the block**. `analysis/drivetrain_bench.py idle --write` does this but takes only 2 ids — a 4-id sweep wants a small script | know the curve; expect latency 16 to cap near 30 Hz, which is the point of measuring it |
| M3 | tick jitter at rate | `run_bike` prints p99 and max at shutdown; `jitter_ms` is live in telemetry | **p99 < 1 ms at 100 Hz** — **MET 2026-09-16: mean 0.17, p99 0.93, max 2.91 ms over 60 s**, four servos and the AHRS both live |
| M4 | AHRS rate and age at the control tick | `python analysis/tm151_record.py --port /dev/ttyACM0`, then age-of-data inside the loop | 200 Hz sustained, age < 10 ms |
| M5 | `DriveController.step` on the Pi | time it around the call for a few thousand ticks | < 1 ms p99 |
| M6 | throttling | `vcgencmd get_throttled` | `0x0` |
| M7 | command-age p99 over the link | once the radio is in, log `link.age()` for minutes | sets `CMD_STALE_S`, which is a guess today (see the note at `run_bike.py:60`) |

M2 and M3 are the two the whole session exists for. M7 is the one that must
happen before `CMD_STALE_S` is trusted, and it is the reason to keep the radio
out of the first session rather than to skip it.

## 5. The order of the session

**Phase 0 — Mac, before the card is flashed.**
Pick the policy and set `control.general_move`; read its
`drivetrain_model.servo` gains out of `moves/<name>.yaml` and write them down
— they are what §2.2 has to install. `python -m aow_sim.export_deploy`, and
check the digest line it prints. Set servo IDs (1/2/3 drive A/drive B/steer,
4 righting) and 3 Mbps baud in DYNAMIXEL Wizard over the U2D2. Configure the
TM151 in ImuAssistant: 460800, 200 Hz, `Ep_Combo` enabled, **auto boot mode**
(static boot is the factory default and costs 10–30 s of what looks like a
dead sensor).

**Phase 1 — flash the card, Mac + SD adapter.** Raspberry Pi Imager is a
separate download, not a macOS built-in:

```bash
brew install --cask raspberry-pi-imager
```

(or the .dmg from raspberrypi.com/software). Then, in the app: **Raspberry Pi
Device** = Pi 3 · **Operating System** = Raspberry Pi OS (other) → Raspberry Pi
OS **Lite (64-bit)** — aarch64 because numpy and dynamixel-sdk both ship
aarch64 wheels and nothing should compile on a 1.4 GHz A53, Lite because there
is no display. Pick the card, press Next, and say **yes** to "edit settings",
which is the part that matters:

| field | value | what breaks without it |
|---|---|---|
| hostname | `aowbike` | `ssh pi@aowbike.local` does not resolve |
| username / password | `pi` + something | no login at all |
| **Public-key authentication only**, paste `~/.ssh/id_*.pub` | your key | password logins over wifi |
| wifi SSID + password | your 2.4 or 5 GHz network | headless first boot; no console to fix it from |
| **wifi country** | CA | some channels unavailable and TX power capped — reads as unexplained short range |
| locale/timezone | yours | log timestamps |

Write, eject, put the card in the Pi. Hand-rolling this is possible but worse:
the OS takes its headless config from a generated `custom.toml`/`firstrun.sh`
on the boot partition, and the Imager writes it correctly.

**Done 2026-09-16, and what actually came up** — worth recording because three
things differ from what this section assumed:

| | |
|---|---|
| host | `aowbike.local` -> 192.168.0.117, MAC `b8:27:eb:*`, up in minutes |
| **user** | **`efun`, not `pi`** — the Imager's username field is whatever you type, and every `ssh pi@...` in these docs was wrong for this card |
| OS | **Debian 13 (trixie)**, aarch64, **Python 3.13.5** — newer than the Bookworm/3.11 this plan was written against |
| supply | `vcgencmd get_throttled` = **`0x0`** at idle, 38.6 C. The 2 A brick is fine, as predicted |
| network | joined a **5 GHz** SSID. Fine for the bench; range and penetration are worse than 2.4, which matters for the AP-directionality question later |
| clock | correct, NTP over wifi. There is no RTC, so this lasts exactly as long as the network does |
| U2D2 | `/dev/ttyUSB0`, FT232H enumerated behind the onboard hub; user is in `dialout`, so no sudo to open it |
| TM151 | `/dev/ttyACM0`, **`0483:5740 STMicroelectronics Virtual COM Port`** — an STM32 native USB CDC device, which is the prediction in §3 confirmed: no FTDI latency timer on that port, and **the baud is ignored** (identical byte counts from 57600 to 1 Mbps) |
| stable names | both devices have `/dev/serial/by-id/` paths — `usb-FTDI_USB__-__Serial_Converter_FTB8HNE3-if00-port0` and `usb-STMicroelectronics_STM32_Virtual_ComPort_338139793235-if00`. **Prefer these to `ttyUSB0`/`ttyACM0`**, which are enumeration-order and would swap if a second FTDI device ever appeared |
| `iw` | **not installed.** Pi OS Lite trixie manages wifi with NetworkManager; use `nmcli`, or `apt install iw` for the verification commands in `untethered-setup.md` §2d |
| sudo | **impossible as provisioned, and not because a password was skipped.** See below |
| boot | **44.7 s to multi-user.** Half of it is provisioning that has finished — see below |

**Boot time: half of it was cloud-init, and `blame` does not show that.**
`systemd-analyze blame` put `cloud-init-main` twelfth at 3.0 s and never
surfaced `cloud-init-local` at all. `critical-chain` — which shows what
actually GATES, rather than wall time per unit — tells a different story:

    multi-user.target @44.674s
    └─ssh.service @44.411s
      └─network.target @44.387s
        └─NetworkManager.service @29.296s +15.088s
          └─ ...
            └─cloud-init-local.service @10.622s +18.053s     <- the big one
              └─cloud-init-main.service @7.005s +3.613s
                └─boot-firmware.mount
                  └─systemd-fsck ... +1.154s

**~22 s of the 44 is cloud-init**, running before NetworkManager even starts,
to re-apply a provisioning that was finished days ago. Disabled 2026-09-16 with
`sudo touch /etc/cloud/cloud-init.disabled` (undo: delete the file), which
should roughly halve the boot. Two things were checked first, because both
would have stranded the board:

  * `/etc/sudoers.d/090-efun` is a REAL FILE (`efun ALL=(ALL) NOPASSWD:ALL`),
    not something the `bootcmd` has to recreate each boot — so sudo survives;
  * the wifi lives in `/etc/netplan/90-NM-*.yaml` rendered to NetworkManager,
    on disk and independent of cloud-init — so the network survives.

What is NOT worth doing: `NetworkManager-wait-online` looks like 5.9 s in
`blame` but is **not on the critical chain** — `network.target` is gated by
`NetworkManager.service` itself. The remaining 15 s IS NetworkManager, i.e.
wifi association and DHCP on a 5 GHz network, which is the radio rather than
the Pi. And note the sensor has its own cold start: the datasheet gives
**10-30 s on static boot, 3.2 s on auto boot**, and static is the factory
default the unit is still in — so until ImuAssistant sets auto-boot, the AHRS
and not the Pi may be what the bike is waiting for.

**Why sudo cannot work on this card.** The Imager wrote a **cloud-init** seed,
not the classic `userconf-pi` path — `/boot/firmware/{user-data,meta-data,
network-config}`, `ds=nocloud` in `cmdline.txt`, and `userconfig.service`
**masked**. So a hand-written `/boot/firmware/userconf.txt` is inert on this
image: nothing consumes it, and it sits on a FAT partition with a password hash
in it, which is worth deleting rather than leaving.

The account itself explains the rest. From `user-data`:

    user:
      name: efun
      lock_passwd: true        <- no usable password
      ssh_authorized_keys: [...]
      sudo: null               <- NO sudoers entry was written at all
    ssh_pwauth: false

Being in the `sudo` *group* is not enough when the password is locked and there
is no NOPASSWD rule. There is no route from inside the machine; every one of
them needs root.

**The fix is two files on the FAT partition, which macOS mounts natively.**
Shut down, card into the Mac, and in `bootfs`:

```yaml
# user-data: replace `sudo: null` with a real rule
  sudo: ALL=(ALL) NOPASSWD:ALL
```

```
# meta-data: any new value re-runs cloud-init's user module on next boot
instance-id: rpi-imager-bench-2
```

Then card back, boot. `NOPASSWD` rather than a password because login is already
key-only: the ssh key is the security boundary either way, and it is what Pi OS
itself did for years. Setting `lock_passwd: false` with a `passwd:` hash is the
alternative if a console login is ever wanted. **Do this before disabling
cloud-init for boot time** — turning it off removes the mechanism that applies
the fix.

**And the trap was live on the first look:**

    $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
    16

That is the FTDI default and it is the single highest-leverage line in the OS
config. **The ceiling is 1000/latency_timer Hz, measured on both machines** —
the Mac read 2.00 ms/frame at a 2 ms timer (500 Hz) and 63 frames/s at 16 ms
(62.5 Hz), and the Pi refuses to open at all until it is 1. So the "caps the
loop at ~30 Hz" in `untethered-setup.md` §2b is pessimistic by 2x; it is ~62 Hz,
which is worse, not better — 62 Hz looks plausible enough to go unnoticed.

**THE UDEV RULE AS THIS DOC FIRST GAVE IT DOES NOT APPLY, and the failure is
silent.** `udevadm trigger` defaults to `--action=change`, while the rule
matched `ACTION=="add"` — so reload-rules and trigger both report success and
the timer stays at 16. Diagnosed 2026-09-16 after exactly that. Use:

    ACTION=="add|change", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"

and trigger with `--action=add` (or leave it and let the next replug do it).
With `add|change` both trigger forms now apply it, verified both ways.

`DynamixelBus.open()` fired on the first attempt here, which is the check doing
its job:

    RuntimeError: /sys/bus/usb-serial/devices/ttyUSB0/latency_timer is 16 ms;
    the Dynamixel loop cannot exceed ~62 Hz.
    Fix:  echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

**All four servos answer from the Pi** (ping only, which is rate-independent,
so the timer does not invalidate it), bus 11.9-12.0 V, torque off:

| id | model | firmware | |
|---|---|---|---|
| 101, 102 | XC430-W150 (1070) | 50 | the latest XC430 build; its numbering is unrelated to the XC330's |
| 103, 104 | XC330-T181 (1210) | **53** | both clear `MIN_FIRMWARE` — neither is the silent-indirect unit | Then just power the Pi over micro-USB — Imager bakes the wifi credentials and
the ssh key into the image, so first boot joins the network with sshd already
listening and `ssh efun@aowbike.local` works with nothing else attached. **The
USB-A Ethernet adapter is a recovery path, not a step**: it is what you reach
for if the Pi never appears, which in practice means a typo'd passphrase, an
unset country code, or a 5 GHz-only SSID. Without it, that failure costs a
re-flash, because there is no console.
Then three OS items, **all needing sudo** — and **not** the UART ones, since
the AHRS is on USB:

```sh
# a. FTDI latency timer 16 -> 1 ms, now and across every replug
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' \
    | sudo tee /etc/udev/rules.d/99-u2d2-latency.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb-serial
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer      # must read 1

# b. WiFi power save off. brcmfmac defaults it ON, and nmcli's "0 (default)"
#    means the driver default, i.e. on -- it does NOT mean off.
sudo nmcli connection modify netplan-wlan0-ATA-5G 802-11-wireless.powersave 2
sudo nmcli connection up netplan-wlan0-ATA-5G

# c. SCHED_FIFO for the control thread without running as root
sudo setcap cap_sys_nice+ep $(readlink -f ~/venv/bin/python3)
```

**(c) is broader than it looks.** A venv's `bin/python3` is a symlink, so
`readlink -f` resolves to `/usr/bin/python3.13` and the capability lands on the
system interpreter — every python3 on the box may then raise its scheduling
priority. To keep it inside the venv, build it with real binaries instead:
`python3 -m venv --copies ~/venv` and setcap `~/venv/bin/python3` directly.
Without any of it, `_try_realtime()` warns and continues at normal priority,
which is a fine place to start measuring from.

**Phase 2 — install.** venv, `pip install numpy pyyaml pyserial dynamixel-sdk
pytest`, `pip install --no-deps -e ~/aow-bike-sim`, then the boundary check
(`import aow_sim.hw.run_bike; assert 'mujoco' not in sys.modules`).

**Phase 3 — bus, no policy, no torque.** M1, then M2. Nothing moves in this
phase; `test_hw_bench.py` is read-only by design.

**Phase 4 — AHRS, no bus.** M4. Hand-rotate the sensor and check the
quaternion signs against `hw/ahrs.py`'s stated conventions.

> **RESOLVED 2026-09-16 — the bike reads this sensor today, by POLLING.** The
> section below is the arc as it happened; the outcome is:
>
> | | |
> |---|---|
> | ODR 200 Hz | **saved to flash**, survives power cycles (verified by replug) |
> | Combo in the output profile | **RAM only** — applied live, never saved, gone after a power cycle |
> | `AhrsReader(poll=True)` | **230 Hz, age-at-tick 4.7 ms mean / 10.4 ms max, 0 stale raises in 300** |
> | `AhrsReader(poll=False)` on the same unit | **0 frames, 300/300 stale** — 127 kB of other messages arriving |
> | `control.onboard.ahrs_poll` | **true**, until the profile is saved to flash |
>
> **There is no API for the output profile and this was checked three ways**:
> the public `TransducerM_Lib_Protocol_C` v1.2 exposes `Init` / `TX_Request` /
> `RX` and nothing else; the library the GUI's own **`Generate Code`** button
> exports (`V1-2-1-Beta`) is the same three functions with no `Set_*`,
> `SysSetting_*` or `Ep_Settings` anywhere; and the firmware answers request
> commands 4, 21, 23, 24, 25 and 42 that no available header defines. The
> setters exist — `SysSetting_SetEnable_TxContinous`, `Set_SendPkg_Continous`,
> `Set_InhibitTime`, `Set_SilentTimeAfterPowerOn` are all symbols inside the
> GUI binary — but only inside it. Ask the vendor for that API; do not guess
> the frame format, because the datasheet warns a bad write can leave the
> module unrecoverable.
>
> **The GUI hangs the moment the profile checkbox is clicked**, reproducibly,
> and the write still lands first — which is how the profile got set at all.
> The mechanism is bandwidth: ticking Combo adds ~14.6 kB/s at 200 Hz on top
> of ~20.8 kB/s already streaming, into a QEMU-emulated 16550 programmed at
> 115200 (11.5 kB/s). Raise the CONNECTION baud before touching anything, and
> reduce the ODR before changing the profile.
>
> **BLOCKED 2026-09-16, and it is a configuration job, not a wiring one.** The
> TM151 is alive and transmitting — but it is streaming
> `rpy`(35) + `raw_gyro_acc_mag`(41) + `status`(22) at **50 Hz each**, and **no
> `Ep_Combo`(43) at all**. Combo is the only message `hw/ahrs.py` decodes,
> because it is the only one carrying quaternion, gyro and accel atomically
> under one timestamp; mixing attitude and rate from different sample instants
> injects phase error into the balance loop. So the fix is Phase 0's
> ImuAssistant step — enable `Ep_Combo`, set the ODR to 200 Hz, set auto-boot —
> over USB from a PC, and it cannot be done from the Pi.
>
> **This failure looked exactly like an unplugged sensor**, which is the part
> worth keeping. `parse_frame` skips a non-Combo frame silently — good CRC,
> wrong command — so `AhrsReader` sat at `frames=0, errors=0`, and
> `preflight_ahrs` said "AHRS not producing fresh frames". Fixed by counting
> `bytes_in` as well as frames: `BikeRunner._ahrs_diagnosis` now separates the
> three cases, and against the live sensor it says
>
>     9690 bytes arrived but NO Ep_Combo frames (0 parse errors). The sensor is
>     alive and streaming the wrong message. Enable Ep_Combo at 200 Hz in
>     ImuAssistant; `python analysis/tm151_serial.py` will name what it is
>     sending instead
>
> `analysis/tm151_serial.py`'s `Decoder` is what named the three messages, and
> it is the tool to re-check with after the reconfiguration.
>
> **ImuAssistant is Windows-only** — the datasheet's software table says
> Windows 7/8/8.1/10/11 and nothing else. "Over USB from a PC" should have said
> *Windows*: the Mac reads this sensor perfectly well (`analysis/tm151_record.py`
> did), because READING it and CONFIGURING it are different jobs and only the
> second needs the GUI. The vendored EasyProfile library has no configuration
> commands either — `EasyObjectDictionary.h` defines request(12), ack(13),
> status(22) and the data messages, and nothing that sets an ODR or enables a
> message — so there is no protocol route to do it from a script.
>
> **It WILL answer a request for Combo, though, and that is a real bench
> path — for verification only.** Measured 2026-09-16: `request(43)` gets a
> valid Combo back, decoding cleanly (quat, gyro, accel at 1.000 g, temp 19 C,
> and an internal `rate_hz` of **400**, so the 50 Hz is an OUTPUT setting, not
> the sensor's rate). Polled hard:
>
>     252 requests, 252 combo answers (100%) in 5 s -> 50 Hz sustained
>     request -> answer latency: mean 19.9 ms, p50 20.2, p99 23.0
>
> **The sensor has never been in any other configuration.** The August capture
> the whole AHRS error model rests on — `analysis/recordings/tm151_rest.npz`,
> 2026-08-26 — holds 15052 samples over 300.0 s of the SAME three messages,
> device timestamps giving **50.0 Hz**, no Combo, recorded at 115200 baud, 0 bad
> CRCs. `sensor-workstream.md:197` says "at 50 Hz" plainly; what nobody joined
> up is that 50 Hz and those three kinds mean the unit was never configured for
> the bike's path at all. Its own `status.rate_hz` field read **401** then and
> 400 now, so 50 Hz is the OUTPUT setting over a 400 Hz internal rate, unchanged
> across both sessions.
>
> **What that does and does not invalidate.** `ahrs_tau_s` **0.19 s survives**:
> at 50 Hz that is ~9.5 samples per tau over 1580 tau-lengths, far from Nyquist,
> and the fit reported r² 0.999. What is bandwidth-limited is anything about
> NOISE — the gyro peak-to-peak check in `analysis/tm151_check.py` saw a 25 Hz
> band, and a 50 Hz output decimated from 400 Hz can alias higher-frequency
> content down into it. Re-measure the noise figures once Combo streams at
> 200 Hz; do not re-litigate tau.
>
> 100% answered, but **50 Hz and ~20 ms of latency**, because the reply is
> queued to the sensor's own 50 Hz output slot. That is half the control rate
> and two ticks of attitude lag against a 113 ms fall time constant. So polling
> is good enough to check the parse, the mount and the SIGNS — Phase 4 — and
> not good enough to close the loop in Phase 5. **ImuAssistant stays on the
> critical path**, and the free-running push is the design for a reason.
>
> **There IS a Linux build, and it runs on the training box.**
> `traces/vendor/ImuAssistant_Linux64_V3-9-23.tar.xz` (kept out of git — 18 MB
> of vendor binary, re-downloadable) is a Qt5/X11 app, and `file` says **ELF
> 64-bit x86-64**: not the Pi (aarch64), not the Mac (arm64 macOS). It ships no
> source, and the vendored protocol library has no configuration commands
> either — `EasyObjectDictionary.h` stops at the data messages — so there is no
> way to script the reconfiguration and no way to avoid running this binary.
> The training box would run it, and is 3000 miles away.
>
> **So: QEMU on the Mac, x86_64 emulation, one-time.** The design choice that
> matters is how the sensor reaches the guest:
>
> | | |
> |---|---|
> | USB passthrough | **avoid.** macOS's own CDC-ACM driver claims the TM151, and QEMU on macOS cannot detach a kernel driver the way it can on Linux |
> | **`-serial /dev/cu.usbmodem*`** | **use this.** The guest sees `/dev/ttyS0`, a platform serial device that `QSerialPortInfo` enumerates, so it appears in the app's port list |
> | socat to a PTY | last resort. A pty has no `device` link under `/sys/class/tty`, so Qt's port enumeration will not list it |
>
> The second row is only safe because of a measurement: the TM151 over USB is a
> CDC device and **ignores baud entirely** (identical byte counts from 57600 to
> 1 Mbps, §Phase 1), so a raw byte pipe loses nothing. Over the GPIO UART this
> trick would not be available.
>
> A **live ISO** avoids installing anything under emulation — Debian 13.7 live
> Xfce, 3.56 GB, run the app from the live session and shut down. Get the
> tarball into the guest with `-drive file=fat:rw:<dir>`, which exposes a host
> directory as a FAT disk.
>
> **It is a one-time trip either way.** The datasheet says the configuration is
> stored in the unit and survives power cycles, which is what lets the Pi's TX
> line stay optional — so this is a trip to make once, not a dependency to
> carry.

**Phase 5 — the loop, wheels clear.** Rear harness clamped with the wheel
free, steer and righting servos bare on the bench, everyone's hands out of the
way. Two terminals on the Pi over ssh:

```sh
python -m aow_sim.hw.run_bike --bundle deploy/bundle.npz --port /dev/ttyUSB0 \
    --ahrs-port /dev/ttyACM0
python -m aow_sim.hw.ground --host 127.0.0.1
```

Start `run_bike` first — it waits for the station and says so. `--no-link`
instead of the second terminal if nobody is driving; `--servo-gains 400:1920`
to fly a gain other than the policy's own. M3, M5, M6, then §6.

**Phase 6 — the radio.** Only now: wifi, then the laptop AP, then M7, then the
directionality question.

## 6. The one bench result worth the whole session

With the AHRS on the desk and the wheel in the air, **tilt the sensor by hand
and watch which way the wheel spins.**

That closes the entire sign chain in one observation: AHRS frame → mount
quaternion → `HardwareData` slots → the policy's roll and roll-rate inputs →
`ctrl` → belt ratio → servo direction. Every one of those is a place a sign
can be wrong, every one of them is currently unverified on hardware, and a
sign error anywhere in it is a bike that drives itself over on the first
attempt while every test in the suite stays green.

It costs nothing and needs no chassis. Do it before anything is bolted to
anything. `analysis/drivetrain_bench.py jog` already exists to check the two
drive signs in isolation (`--signs`, documented as UNVERIFIED, from the CAD)
— this is the same check with the policy and the sensor in the loop.

**Then keep tilting, past the cut angle.** The same gesture exercises the whole
fall path, which is otherwise untestable without throwing a bike on the floor:

| tilt the AHRS to | expect |
|---|---|
| past 60° | `CUT: roll ... — policy out`, the wheel stops, the steer goes limp, the righting servo stays powered |
| back to flat, held still 0.5 s | nothing — it is waiting to be asked |
| `r` at the station | `RE-ARMED: policy engaged, command zeroed` and the wheel answers tilt again |
| flat but waved around | nothing, because the rate gate is doing its job |

Four observations, no chassis, and they cover every branch of `FallGuard`
against the real sensor rather than against a unit test's numbers.

## 7. Redeploy, and what a change costs

Only the **first** setup touches the OS. After that nothing is reflashed:

| changed | ship | cost |
|---|---|---|
| control code | `rsync -av --delete --exclude .git --exclude runs --exclude traces --exclude __pycache__ ./ efun@aowbike.local:~/aow-bike-sim/` | seconds, over wifi or Ethernet |
| a policy | `rsync -av moves/ efun@aowbike.local:~/aow-bike-sim/moves/` | seconds |
| `bike_params.yaml`, gains | `python -m aow_sim.export_deploy` then rsync `deploy/` | seconds, and the digest check catches it if you forget |
| the OS itself | reflash | never, after Phase 1 |

The editable install means a code rsync needs no reinstall. The bundle's
digest is what makes this safe to do casually: a stale `deploy/bundle.npz`
refuses to load rather than flying wrong numbers.

Worth adding once the shape is known: `scripts/pi_setup.sh` (Phase 1 OS items
+ Phase 2) and a `scripts/deploy.sh` for the three rsyncs, so neither is a
remembered sequence. Not yet — write them after the session, from what was
actually typed.

## 8. Still open after this session

* **Steer homing at power-up** (§2.6). Blocks the fork going on, not the
  bench.
* **The AHRS mounting calibration** needs a chassis; `preflight_ahrs` will
  report `absent` until then and that is correct.
* **`CMD_STALE_S`** stays a guess until M7.
* **The LVC failsafe** cannot be tested on a 12 V brick.
* **There is no SPI option on this part.** The TM151 datasheet lists exactly two interfaces, UART
  (TTL 3.3 V) and USB 2.0 VCP. Nothing in this repo has ever implemented or specced SPI for it —
  `hw/ahrs.py` is pyserial only, and the string does not appear in `src/` or `docs/`. The fallback
  for a USB port that is needed elsewhere is the GPIO UART, not SPI.
* **The Zero 2 W's own timing.** Everything measured here is a 3B+ with four
  USB ports and a better radio. The numbers transfer as an upper bound on what
  the software costs, not as the flying budget.
* **Whether the drivetrain model is right.** The wheel is in the air, so the
  bench sees the servo loop and the differential but no contact. The P 100 /
  P 400 decision (`docs/status.md` item 3) still wants the bike on the floor.

---

## Bring-up log, 2026-09-15 to 09-18 (from status.md)

*Moved verbatim from `docs/status.md` on 2026-10-02, when that file was rewritten as a short navigation layer. "Above", "below", "Health" and numbered "What to do next" items refer to status.md as it was then (`git show d0dffeb:docs/status.md`).*

On 09-17, +36 passing against 09-14, all from the onboard bring-up work: 18 in the new
`tests/test_hw_runbike.py` (the fall guard and the UDP command struct, both
`pure`), 11 added to `test_hw_dynamixel.py` (the fourth servo, gain
resolution), 2 in `test_hw_ahrs.py` (a non-Combo frame is skipped silently,
which is what made a healthy TM151 read as a dead one), and 2 to the `boundary`
list (`hw/ground.py`, `control/recovery.py`).

**THE CONTROL LOOP RUNS ON THE PI (2026-09-16).** 60 s, four servos and the
TM151 both live, policy stepping, nothing energised: **tick jitter mean 0.17 ms,
p99 0.93, max 2.91** at 100 Hz, against a p99 < 1 ms gate. Per-phase the servo
read is 2.08 ms, the SyncWrite 0.58, and `DriveController.step` **0.03 ms** —
the same as on an M4 Mac, so SBC compute was never a risk and the bus is the
budget. Getting there fixed three bugs of mine (a preflight that raced its own
reader, a fixed-block serial read capping the AHRS at 100 Hz, and a telemetry
percentile costing up to 49.5 ms per tick) and one design trap (`poll_timeout`
defaulting to the same 50 ms as the staleness limit). `SCHED_FIFO` made it
worse and is off, and the `gc.disable()` that fixed the jitter once was REMOVED
once the telemetry fix made it unnecessary (re-measured: indistinguishable).
**With four servos ENERGISED and the AHRS live, 30 s: 0.24 / 0.79 / 1.56 ms**,
no tick over 10 ms. The 40-60 ms outlier that only appeared under torque was
`np.percentile`'s first call, not electrics -- warmed before the loop. A steer
zero is now captured at startup: the bare XC330 came up 1.4 turns wound, so the
first absolute command would have slammed it. Detail: `pi-bench-bringup.md`.

**The AHRS path is proven on hardware (2026-09-16).** `hw/ahrs.py::parse_frame`
decodes real TM151 Ep_Combo frames — quaternion, gyro 0.2 deg/s at rest,
`|accel|` 9.772 against g 9.807. `AhrsReader` also gained an opt-in
`poll=True` that requests each frame, for a unit whose output profile has Combo
switched off: **230 Hz, age-at-tick 4.7 ms mean / 10.4 ms max, zero stale
raises in 300 ticks**. Detail and the three ways the "no configuration API"
claim was checked: `pi-bench-bringup.md`.

**PUSH IS BACK, and this time it is in flash (2026-09-16).** The Combo output
profile was ticked from a Windows machine; the earlier attempt through the
QEMU/Linux GUI applied live and was gone at the next power cycle. The proof is
a power cycle, not a checkbox — the unit was unplugged from that machine and
plugged into the Mac, cutting its only power, and came up streaming Combo
unprompted at **198.3 Hz** with nothing transmitted to it. Through `AhrsReader`
at 100 Hz ticks: **201.4 Hz, requests 0, age-at-tick 2.32 ms mean / 4.87 p99 /
4.93 max, zero stale raises in 367 ticks** — better than the polled path on
every number. `control.onboard.ahrs_poll` is now **false**.

It still streams `Status`(22), `RPY`(35) and `Raw_GyroAccMag`(41) at 200 Hz
alongside Combo: 33.7 kB/s total, of which Combo is 8.7. Free on USB CDC, and
`parse_frame` drops the other three on the command check — but it would NOT fit
a 460800 UART (336 kbps of 460.8, no flow control), which is the wiring
`untethered-setup.md` specifies for the Zero 2 W. Untick them before that move.

**Preflight now checks the sensor's own QoS.** `Ep_Combo` carries
`Ep_Status_SysState`, so the grade costs one mask and no extra subscription.
The bar is 3 (`basic service`) rather than 4, because 3 is what a healthy unit
holds in the first ~30 s after power-on — exactly when the bike is being armed —
and 2 is excluded on the vendor's own words for it, "very limited measurement
accuracy". The bench unit reads 5 (`very good service`). This is the only one
of preflight's four checks that can catch a sensor whose numbers are all
plausible and all wrong.

**The whole loop ran on the Pi against the new config (2026-09-16), torque
off.** Tick jitter **mean 0.15 ms, p99 0.91, max 1.50** on a 10 ms budget; pack
11.9 V. The AHRS ran pushed — `requests 0`, 200.6 Hz, 73.0 B/frame,
age-at-tick **mean 2.09 / p99 4.32 / max 5.18 ms**, zero ticks over 10 ms and
zero stale raises in 984 — which is BETTER than the Mac (2.41 mean) and half
the old four-stream push figure (4.3 / 9.7). The position gains were read back
off the bus rather than trusted: id 103 mode 4 P900/I0/D0, id 104 mode 5
P700/I0/D1400.

Three config gaps the run found, all fixed: `control.onboard.servo_ids` was
READ by `run_bike` and never defined, so it fell through to `(1, 2, 3)` and
every bench run died on `read id=1 Model Number: rc=-3001`; `righting_id` was
4 against a bench numbered 101-104; and `Goal Current` on the righting servo
came up at `Current Limit` (910), i.e. no torque cap, with nothing setting it.
`righting_current: 300` is now written next to the gains at every startup. See
`hw/control_tables/README.md` for the reboot measurement that says why it has
to be every startup.

**Torque on, 2026-09-16, operator holding the rear assembly.** The steer does
NOT drift: it settled at -57 deg within a second and stayed there (-58.2 to
-56.1 over 10 s), because with the wheels turning the odometry moves and the
policy's observation stops being constant. The unbounded -229.2 deg/s march
seen with `--no-torque` is an open-loop artifact, not a property of the
controller. Tick jitter mean 0.14 / p99 0.76 / max 0.84 ms, the best yet, and
torque dropped cleanly on all four servos at `--seconds`.

**Extended position's turn counter is RAM (measured).** Driving the steer one
revolution put `Present Position` at 4931 counts; a reboot brought it back to
835, losing exactly 4096. So steer winding cannot accumulate across power
cycles toward `clamp_extended`'s +-256 turn ceiling. `control_tables/README.md`
carries the table and why the first version of this test proved nothing.

**The ground station binds the viewer's keys (2026-09-16).** Arrows and `/`
are primary, `w`/`s`/`a`/`d`/space are aliases, and `/` re-aims the heading at
the bike the way `run_drive`'s `zero_command` does — which needed `psi` and
`righting_current` added to the telemetry, since the station cannot know
either. Still a TERMINAL station: there is no graphical one, and reusing the
MuJoCo viewer as a live mirror is the obvious candidate rather than a second
UI. **`steer_zero_deg` is pinned at 180** — the orientation the fork clamp will
be built to, chosen so straight-ahead sits mid-range rather than on the
single-turn wrap. It is NOT yet true of the hardware, so bench sessions want
`--steer-zero capture`; without it the bare shaft (73.4 deg) is told it is
106.6 deg off straight, which is visible in the telemetry as a completely
different drive split.

**Telemetry schema v2, and the mirror's half of it (2026-09-16).**
`hw/telemetry.py` holds `build` (Pi, mujoco-free) and `apply_pose` (laptop)
fifty lines apart on purpose: the failure they are both exposed to is drifting
apart, and a renamed field is read as a `.get()` default rather than raising.
438 B/packet, 21.4 kB/s at 50 Hz, versioned so a stale deploy fails at connect.

The rear wheel renders from TWO NUMBERS. `_aow_assembly`'s gearbox is a pair of
linear tendon equalities, so hub, ring and all eight rollers are a closed form
of the input-shaft angles -- no solver needed in a render. Checked against
MuJoCo's own constraint solve after 1500 driven steps, agreeing to <2e-3 rad.
What the bike genuinely does not know (ride height, front-wheel angle, the omni
internals' absolute phase) is DRAWN and the module says so per field.

**Two link bugs found by measuring the packet rate rather than trusting it.**
A station sending at 50 Hz got **27.8 Hz** of telemetry back. `CommandLink._run`
compared `now - last_tx > period` against a clock driven by datagram ARRIVALS,
which aliases; fixing it to an accumulated deadline gave 34.7 Hz, still short,
because the loop could only transmit on a wake and `recvfrom` only wakes on a
command or a 100 ms timeout. Waiting on `select` with the time-to-deadline as
its timeout gives **50.0 Hz, gap mean 20.02 ms / p99 29.6 / max 40.7**, and the
control loop is untouched (tick jitter mean 0.15 / max 1.30 ms). Both bugs read
as "the radio is struggling" and neither was.

**The radio is not the constraint, measured.** Pi to Mac, zero loss at every
size tried: 218 B, 1.1 kB, 3.9 kB, 11.9 kB and 31.9 kB per packet at 50 Hz
(the last is **1.56 MB/s**), and 1.1 kB / 7.9 kB at 200 Hz. Encoding is the
budget that binds, and barely: a full pose plus four servos x six registers is
855 B and **154.8 us on the Pi, 1.55% of a 10 ms tick**. Caveat: house wifi,
stationary bike, one client -- and `untethered-setup.md` specifies a
LAPTOP-HOSTED AP for the real link, which is one hop rather than two. Re-measure
before trusting it with the bike moving.

**A latent test bug, not mine, found on the Pi.** `pytest tests/` gave 3
failures in `test_hw_no_mujoco.py` and `pytest tests/test_hw_no_mujoco.py`
alone gave 18 -- and running that file alone is exactly how you check the
import boundary on the bike. The fixture's teardown did `sys.modules.clear()`
then restored a snapshot taken at SETUP, so numpy (first imported during the
test, via `control.policy`) was evicted permanently and the next re-import hit
`cannot load module more than once per process`. Invisible on a laptop, where
an earlier test file has always imported numpy first. Now 1 failure alone, and
that one legitimately needs mujoco.

**The mirror is up (2026-09-16).** `mjpython -m aow_sim.run_drive --mirror
aowbike.local` drives the real bike and renders it in the MuJoCo viewer with no
physics at all: `interactive.mirror_loop` calls `frame(model, data)` per
rendered frame, `telemetry.apply_pose` writes the pose, `mj_forward` does the
kinematics. Teleop's own `_KeyState`/`_Axis` and ramp constants drive it, so
hold-to-accelerate and the 35 deg lead clamp behave exactly as in the
simulator, and `MIRROR_KEYS` is module-level so a test can prove both stations
land on the same `OperatorState`.

Checked headless against the live bike over 12 s: **rendered roll tracks
telemetry roll to 0.003 deg**, chassis sits at the settled rest height, steer
matches, 401 packets, zero empties. `_overlay`'s dial needed no change -- green
tick commanded heading, cyan actual -- and it is fed `cmd_psi`/`cmd_v_world`
from the packet, i.e. the command THE BIKE SAYS IT RECEIVED, which differs from
what the station last sent exactly when one was dropped.

Two bugs the live run found. The bike transmitted its telemetry dict before the
control loop had filled it, so the first packets on the wire were `{}` and the
station's version check reported "schema vNone -- re-sync the Pi" against a Pi
that was fine; the bike now stays silent until it has something to say, and
`check_version` tolerates an empty packet, both. And `--mirror` under plain
`python` printed "mirror stopped after 0 telemetry packets" underneath the
mjpython hint, which reads as though it ran.

**Preflight's standing condition got its own escape hatch (2026-09-16).**
The AHRS mount is `GUESS` and stays that way until the bike can be jigged level
on a level floor -- so preflight blocked EVERY run, and the only way past was
`--no-preflight`, which also disarms the |accel|, |gyro| and QoS checks. A gate
that fires every single time is not a gate: it trains the operator to reach for
the flag that turns off the gates which do catch things. Findings now carry a
kind, `--allow-guess-mount` accepts exactly that one, and the error only
suggests it when it would actually clear the block. The mount still blocks by
default -- on the assembled bike an uncalibrated mount is a permanent roll bias
and is worth stopping for.

**Five things the first mirror session found (2026-09-16), all from looking
at it rather than from a test:**

  * **The lead clamp blocked BOTH directions** once the command left the
    +-35 deg band, so the heading command froze with no way back -- and the
    band can be left with no key pressed at all, because the bike moves.
    Presented as "the heading command does nothing". Teleop's own `turn`
    blocks only the direction that GROWS the lead; the predicate is now
    `run_drive.lead_blocks`, module level and tested.
  * **6/7/8 were unbound.** They are teleop's heading snaps (+90/-90/180) and
    pass `clamp=False`, because a snap is meant to lead until the bike catches
    up.
  * **A fixed ride height put the front wheel underground** on a nose-down
    pitch, and floated the rear on a wheelie -- the attitude rotates the body
    about its origin. `telemetry.ground_the_wheels` now puts whichever wheel is
    lower on the floor, exact for a surface of revolution (`centre_z - radius`,
    no bounding-box estimate). Consequence: the mirror can never show a wheel
    genuinely lifting off, because one is always pinned.
  * **The camera was framed for a bike that travels.** It now TRACKS the
    chassis at 0.9 m with `-`/`=` to zoom, which is safe whether or not the
    dead-reckoned position wanders.
  * **`--port`/`--ahrs-port` are in `control.onboard` now** as
    `dxl_port`/`ahrs_port`, by-id rather than `ttyUSB0`. Typing two 70-character
    paths on every run is how `--no-preflight` got into the habit too. Flags
    still override.

**Telemetry runs at the CONTROL rate now, not half of it.** The dict was built
every other tick to match a 50 Hz link; that halving is also 0-20 ms of extra
staleness on top of the transmit period, and the mirror shows it as lag. Both
are at 100 Hz: measured **95.5 Hz, 40.0 kB/s, gap mean 10.47 / p99 20.03 ms**,
and the control loop is unmoved with a station attached (tick jitter mean 0.16
/ p99 0.94 / max 1.50 ms). What remains is irreducible without more work:
AHRS age ~2 ms, one control tick <=10, one transmit period <=10, the radio, and
one render frame <=17 -- so ~25-45 ms typical against teleop's zero, because
teleop's state is local. The mirror is a picture of somewhere else.

**A measurement artifact that bit twice.** Both attempts to measure the
telemetry rate from a loop that drains to the NEWEST packet once per frame
read back the loop's own frame rate (27.8 Hz, then 49.4) rather than the
link's. Counting every packet gives 95.5. Drain-to-newest is right for a
mirror and wrong for a rate measurement, and the two look identical in the
output.

**The mirror's odometry drift is now watchable on purpose.** `pos` is
dead-reckoned and drifts, and that drift IS the odometry error made visible --
so the bike walks around the floor by default, `0` re-centres it, and `p` pins
it at the origin and hides the floor grid (a world reference with no world
motion left to reference reads as though nothing is working).

**The rear wheel's angle is MEASURED now, not integrated.** It was summed on
the station from `w_shaft` x display-dt, which was wrong twice: the dt came
from the packet (the bike's 10 ms tick) while the call happens once per
rendered frame, so the wheels turned at 60% of reality; and `vel` is FILTERED,
so even a correct integral would lag and would lose whatever happened between
the packets that arrived. `read_state` already computes an unwrapped per-tick
position delta in order to difference the velocity, so the bus now sums that
same delta into `turned` and the packet carries it. The station prefers the
sent angle outright and keeps integration only as the fallback for a bike too
old to send one. NOT yet confirmed against a hand-turned wheel on the bench --
the plumbing and the arithmetic are tested, the physical check is outstanding.

**THE RIGHTING MECHANISM IS IN THE MIRROR (2026-09-17).** `run_drive --mirror
HOST --swing-linkage` now draws the co-rotating four-bar from the righting
servo's measured angle. The servo sends `righting_pos` (a new telemetry field;
no version bump, adding one never needed it), and `build_model.
SwingLinkageSolver` turns that into all five joint angles in closed form.

WHY A SOLVER AND NOT JUST THE CRANK. The loop is closed by `mjEQ_CONNECT`, and
an equality is only satisfied by the constraint solver during a dynamics step
-- `mj_forward` computes constraint FORCES, not constraint-satisfying
positions. The mirror deliberately never steps, so writing the crank alone and
calling `mj_forward` leaves the couplers and wings where they were and the
mechanism visibly comes apart. Every joint has to be written.

Verified against MuJoCo's own constraint rather than against the arithmetic:
both `mjEQ_CONNECT` site pairs stay coincident to **< 1e-9 m at 21 travels
across the full +-136.6 deg stroke**, and end to end over a real socket a servo
at 280 deg renders crank +100, wings R +62.1 / L +11.7, gap 0.000 um.

The measured angle beats the commanded one on purpose: a wing held back by its
Goal Current is DRAWN held back, which is the thing an operator tuning
`righting_current` is watching for. Both are sent; `righting` is the fallback
for a bike that reports no reading.

**THE MECHANISM IS BUILT AND SWEEPS ITS MOTIONS (2026-09-17)**, which this
document said was "design done, build last" for eight days. It was constructed
separately; the testbed XC330s are bare, so it can be driven without risk.

`righting_stow_deg: 180` is `source: design` and **forced rather than chosen**:
the stroke is +-136.6 deg in one 0-360 servo range, so 180 +- 136.6 spans
43.4 .. 316.6 and fits, and more than ~43 deg off centre runs an end of the
stroke into extended position where a power cycle loses the turn count.

`righting_sign: 1` is **TBD on purpose and is not a measurement waiting to
happen** -- the motor can be installed either way and the bike's packing
solution is unsettled, so it may still be flipped. Consequence of it being
wrong is entirely visual: the render deploys to the wrong side. Nothing flies
on it and neither digest sees it. One observation when the motor is mounted.

MEASURED IN PASSING, and not what the config's name suggests: this crank
**turns all the way round**. `stroke.crank_travel_deg: 136.6` is the DESIGN
stroke -- what reach and clearance ask for -- not a kinematic limit; the loop
closes at every angle in 360 deg. Pinned by a test, because "it stopped at the
end of its travel" would be the wrong explanation for a station that froze.

Also fixed: the mirror read `stroke.crank_travel_deg` from `hw/ground.py`'s
fixed default rather than from the config the MODEL was built with. The two
`_smaller` configs differ (136.6 vs 132.1 deg), so `--swing-linkage <other>`
would have commanded a stroke the rendered mechanism could not reach.

`test_every_telemetry_field_is_read_by_the_mirror` **now exists.** The
`hw/telemetry.py` docstring has named it since the module was written and no
such test was ever added; the coverage was hand-written assertions that a new
field slips straight past. It partitions FIELDS into pose and display, and
perturbs every pose field on its own to prove `qpos`/`qvel` moves. Confirmed to
bite by breaking the reader and watching it fail.

**THE VIEWER'S LAG WAS THE OVERLAY, NOT THE SOLVER.** Reported as heavy lag
while rolling the bike and backdriving the servos, and the four-bar was the
obvious suspect. Measured instead:

| per rendered frame | before | after |
|---|---|---|
| `SwingLinkageSolver.pose` | 23.7 us | 23.7 us |
| `apply_pose` (whole pose pipeline) | 0.033 ms | 0.033 ms |
| `mj_forward`, worst case (rolled 90, 8 contacts) | 0.098 ms | 0.098 ms |
| **`_overlay`** | **4.004 ms** | **0.256 ms** |

`floor_geoms` walks every geom building a named accessor each time, and the
reference grid reached it TWICE PER GRID POINT -- 208 calls and 8132
`str.startswith` per frame -- to recompute one plane's orientation that cannot
vary across the grid. Hoisted out of `floor_z`, and `floor_geoms` memoised on
the model (weak keys; the set is fixed at compile time). **15.6x, and it is
TELEOP'S BUG TOO** -- same overlay, same path, and it has been there far longer
than the mirror. Pinned by a call COUNT rather than a stopwatch.

**WINGS NO LONGER SINK THROUGH THE FLOOR.** `ground_the_wheels` is now
`ground_the_bike` and includes the righting panels. They are boxes, so the
lowest point is a corner that moves with roll -- upright a fully deployed wing
still clears the wheel contact by 5.7 mm and nothing changes, but at 61 deg it
reaches **78 mm below** it. A rolled bike with a wing out now rests ON THE
WING and the wheels lift clear, which is what the mechanism is for.

Re-arm now says so. `r` was a key that appeared to do nothing: `FallGuard`
stores consent and spends it only once the bike is back under **12 deg** and
settled for **0.2 s**, and nothing in the viewer said either thing. The mirror
now prints state transitions and acknowledges the keypress.

**"THE LINKAGE REGRESSED TO A DIFFERENT MECHANISM" WAS THE SERVO AT ~0 deg.**
Reported as `--swing-linkage` showing an older or wrong geometry; the config
was right (`_smaller`) and the compiled model is byte-identical to HEAD across
all seven build combinations. The servo was sitting near 0 instead of the 180
stow, i.e. -180 deg of crank travel -- OUTSIDE the +-136.6 deg stroke, but this
crank turns all the way round, so the loop closes there and what got drawn was
a real pose the mechanism can never be COMMANDED to. It reads as a different
machine because kinematically it is one. Driving the servo back to 180 fixed
it. The mirror now warns once per excursion, and still draws it: hiding it
would swap a confusing picture for a frozen one.

**THE MIRROR'S CHUGGING IS THE WIFI LINK. MEASURED 2026-09-17.** Not the
renderer, not the loop, not the four-bar. `--frame-stats` on the real mirror
reported the loop HEALTHY while the operator was watching it chug -- 60 fps
(n~300 per 5 s window), sync 0.10-0.17 ms, 10.7 ms of sleep headroom, and 0 of
300 frames over budget in the very window the stutter was seen. That is the
signature of a mirror redrawing the SAME pose: the loop cannot be at fault
because the loop is idle.

So the arrival pattern was measured instead -- 800 UDP packets at 100 Hz from
the Pi, telemetry-sized, **with the Pi otherwise IDLE (load 0.10, `run_bike`
not running)**:

| | |
|---|---|
| received | 793 of 800, 99.8/s -- looks fine on the average |
| gap p50 / p90 / p99 | 9.95 / 10.44 / 11.81 ms -- smooth most of the time |
| **gaps > 33 ms** | **6 in 8 s: five of 100-150 ms, one of 552 ms** |
| link | **-74 dBm**, rate adapted 130 -> 65 Mbit/s, 36 tx failures |

A 100-150 ms gap is 6-9 HELD FRAMES at 60 fps, several times a second-ish.
The average hides it completely, which is why "95.5 Hz telemetry, zero loss"
from the earlier bench session was true and useless for this question: it
measured throughput, and the thing the eye sees is the tail.

**BUT THE STALLS DID NOT REPEAT, SO THEIR CAUSE IS OPEN.** Re-measured later
the same day, same probe, same band, bike re-powered:

| `ATA-5G` | signal | gap p99 | max | gaps >33 ms |
|---|---|---|---|---|
| the run above | -74 dBm | 11.8 ms | 552 ms | **6 in 8 s** |
| 30 s | -66 | 11.4 | 23 | **0** |
| 60 s | -70 | 10.8 | 24 | **0** |

-70 dBm was clean for a full minute, so weak signal alone does not explain
them. Not yet ruled out: macOS AWDL (AirDrop/Continuity takes the laptop's
radio off-channel periodically -- the laptop end, which nothing here has
measured), background scans on either end, other traffic on the router. **The
link is still where the chugging came from; WHY the link stalled is not
known.** Next time it chugs, run the probe then, and check AWDL
(`ifconfig awdl0`) on the laptop. AWDL was `active` during all three clean
runs, so it does not stall the link on its own; it may still be a factor.

**THE BIKE IS NOW ON `ATA` (2.4 GHz)**, `autoconnect-priority` 10, with
`ATA-5G` kept as the fallback. Same IP (192.168.0.117); the laptop stays on
`ATA-5G` and reaches it, same router. 60 s probe on `ATA`: -54 dBm, 5862/5862,
gap p99 13.6 ms, max 25 ms, **0 gaps >33 ms** -- as clean as `ATA-5G` was the
same day, so this proves margin, not a cure. **The operator then ran the
mirror on it and saw no chugging.** The between-sitting difference is most
likely DOORS: laptop and bike were in the same spots both times, and which
doors were open is what changed (operator's observation). 2.4 GHz goes through
them better, which is the reason to stay on it. Getting there took the operator's
passphrase; three things found on the way, all in `untethered-setup.md`:

  * The "13 dB better" was ONE scan. Six repeat scans later: `ATA-5G` -63/-64,
    `ATA` -58/-60, `ATA_EXT` -51/-52 -- about 5 dB. Stable within a sitting,
    different between sittings.
  * The Imager stores the wifi key as the 64-hex PSK, which is salted with the
    SSID, so `ATA-5G`'s stored key cannot be reused for `ATA` even when the
    passphrase is the same. Copying it failed at the 4-way handshake; the
    operator then typed it at a hidden prompt.
  * The fallback works: the failed attempt dropped ssh for ~20 s and the bike
    returned to `ATA-5G` on its own. And a NEW `nmcli` profile persists (it is
    a keyfile in `/etc/NetworkManager/system-connections/`); only EDITS to the
    netplan-generated `netplan-*` profiles are lost on `netplan apply`. An
    earlier version of this file said all nmcli changes were lost.

**DECIDED: A ROUTER, NOT AN ACCESS POINT ON EITHER END.** House wifi for now;
if trouble comes back, move the router or use a different one. The laptop and
Pi APs are both out -- each end has one radio, so either leaves the laptop
(and any Claude session on it) offline, there is no cell service at the bench
to tether from, and hosting an AP on the M4 is awkward (legacy CLI routes). A
USB wifi adapter on the Mac would not rescue it either: macOS on Apple Silicon
has essentially no drivers for them (general knowledge, not tested here).
Options table in `untethered-setup.md`. Consequence worth having: **the bike
keeps internet**, so its clock stays NTP-set and offline timestamps are not a
problem to solve yet.

**WHICH BOARD SHIPS IS OPEN.** The Zero 2 W is probably not happening; the end
state is a custom or different SBC with an unknown radio, so every link number
here is provisional against a radio nobody has chosen, the 95.5 Hz one
included.

**`iw`, NOT `nmcli`, FOR SCANS.** NetworkManager rate-limits rescans, and
three `nmcli dev wifi rescan` calls returned its cache, which is where "there
is no 2.4 GHz network" came from. `sudo iw dev wlan0 scan` is the
measurement.

**PHASE 2 IS UP (2026-09-17): every servo's health in the packet, and every
tick recorded on the bike.** Run on the Pi against the real bus, torque off.

  * **Read block** +8 bytes a servo (`HEALTH_BLOCK`, `hw/dynamixel.py`): PWM,
    effort, input voltage, temperature, Hardware Error Status. 22 of block 1's
    28 indirect entries. "effort" is ONE slot with two meanings -- Present
    Load (fraction of max torque) on the XC430 drives and Present Current (A)
    on the XC330s, same address 126 -- decoded per servo by its own register,
    and keyed `load` / `A` in the packet so the two never share a column.
  * **Cost of the read**, 2000 reads x 4 alternating runs: mean +0.01 ms
    (1.92 -> 1.93), p99 +0.4-0.9 ms (1.98 -> 2.37-2.89). A 10 ms tick.
  * **Packet** 474 -> ~790 B with four servos (`servos`, and `run`, the record
    it is being written into). 96 packets/s delivered.
  * **The mirror shows it** as a text readout (`telemetry.status_text`):
    state, pack, jitter, QoS, record name, and per servo temperature, effort
    and duty. A hardware error replaces that servo's line and is printed to
    the terminal once.
  * **The onboard record** (`bench_log.RunRecorder`): on by default,
    `traces/bike/<stamp>_run[_tag]/` on the Pi, loads as an ordinary bench
    `Capture` -- raw registers decoded from its own meta, what was written to
    each servo, and 24 columns of what the controller had (`BIKE_COLS` in
    `run_bike.py`). meta carries the whole `bike_params`, the policy and both
    digests. ~300 KB per 20 s compressed. `--no-record`, `--tag`.

**THE RECORDER'S FIRST DESIGN STALLED THE LOOP, measured and replaced.** A
writer thread handed 1000-row chunks to `np.savez`; at the hand-off the loop
went 31 ticks late, worst 24 ms, starting at exactly tick 1000. The save is
12 ms on its own -- in a thread it fights the control loop for the GIL for
~0.4 s. Now each tick appends one 585 B row to a buffered file on the loop's
own thread (0.105 ms mean, 0.138 p99 on the Pi), and `close()` converts it.

| 30 s, link up, torque off | tick jitter p99 | max |
|---|---|---|
| recording off | 0.94 ms | 2.10 ms |
| recording on | 1.02 ms | 4.30 ms |

A kill or brownout loses at most the unflushed ~1 s; `Capture.load` reads the
partial `rows.bin` and meta says `complete: false`. Pull records with
`rsync -a efun@aowbike.local:'~/aow-bike-sim/traces/bike/' traces/bike/`.

**THE CONTROLLER RAN AT HALF SPEED ON THE BIKE -- fixed 2026-09-18, before
any torque-on run.** `control.rate_hz` is 200 (the simulator's rate, what
the LQR was designed at) and DriveController took its `dt` from it; the bike
ticks at 100 Hz. So on hardware the policy was queried at **25 Hz instead of
the 50 it trained at**, the steer target integrated half the commanded rate,
and the velocity low-pass had twice its time constant. Found by replaying a
bench record through the controller: the policy asked for -458 deg/s and the
target moved at -229. `run_bike` now builds its controller at the loop's
own rate (`controller_params`) and calls `tick()`, which computes every call
-- a 10 ms schedule would skip the ~10% of ticks the 1 ms servo clock reads
as 9 ms (measured 9/10/11 ms: 305/2384/300 of 3000). A test pins 50 policy
queries per second and the full integral. Cost on the Pi: `policy.action`
0.39 ms; tick jitter p99 0.86 and 0.89 ms in two 20 s runs. One unexplained
cluster (9 ticks, worst 15.4 ms, ~1 s into one run) did not repeat.

**FIRST TORQUE-ON BENCH RUN (2026-09-18, 93 s): drive A tripped its own
overload protection** while the operator held the wheel -- 100 % PWM at
80-98 % load for 3.5 s in total, 44 C, latched error 32 at 88.6 s. Three
things were wrong, all fixed and tested, the reboot verified live:

  * the policy stayed engaged and drove the dead servo for 5 s. A latched
    error on a drive or the steer is now a CUT that holds until it clears
    (the righting servo's is announced only);
  * the run then DIED on the cut's torque-off: the reply's error byte was
    128, the protocol's alert bit ("a hardware error is latched"), which
    says nothing about the instruction -- it had worked. Eight status checks
    treated any non-zero byte as failure; now only bits 0-6 do;
  * a latched error persists until reboot, and would have failed start-up
    too. `ServoBus.open` now reboots any latched servo before configuring it
    and says so; restarting run_bike is the recovery. Verified: id 101 32 ->
    0. That first run after the reboot then failed preflight on something
    unrecorded (the next run passed) -- open.

**THE HEADING COMMAND STARTED FROM THE AHRS'S ZERO, not from the bike.**
Reported by the operator as "the zero point is odd". The station's heading
started at 0.0 -- the TM151's own yaw zero, an arbitrary direction -- so
every connect commanded a turn: measured +45 to +66 deg of lead at all four
connects of the 2026-09-18 sessions, outside the +-35 deg clamp band before a
key was pressed. And the `psi` the bike reported was the CONTROLLER's, frozen
through every cut (101 deg stale in one session), which the station re-aims
and clamps against. Now: the bike reports its measured yaw; the station sends
NO heading until it has heard one (the bike keeps its own), and while the bike
is not engaged the command follows it, so a re-arm starts aligned. Verified
live: 0.0 deg of lead at connect and at reconnect.

**SCHED_FIFO IS NOW GRANTED ON THE PI** (2026-09-18): `ulimit -r` was 0, so
run_bike's real-time request was refused with a warning every start. Set by
the operator in `/etc/security/limits.d/90-aow-rtprio.conf` (`efun - rtprio
80`; takes effect at the next login). A system setting on the Pi, not in
this repo -- a fresh SD card needs it again. It CONTRADICTS a 09-16
measurement (FIFO made the loop worse by starving the AHRS reader) that did
not reproduce: 0.00% of ticks reused an IMU sample, with or without FIFO. See
the dated note in `pi-bench-bringup.md`. One run each, so indicative:

| tick lateness | median | p99 | worst |
|---|---|---|---|
| before, two 20 s runs | 0.09 ms | 0.86-0.90 ms | 1.6-3.6 ms |
| after, 142 s, mirror connected | 0.03 ms | 0.76 ms | 1.12 ms |

**THE LOW-VOLTAGE CUTOFF IS REBUILT** (2026-09-18; each threshold seen to fire on the brick).
It read drive A alone as a raw 1 Hz sample and ENDED the process at 10.2 V.
But each servo reads the rail at its own connector: on the brick all four agree
to 0.1 V at rest, and with the drives loaded the spread is 0.2-0.3 V typical
and 0.7 V worst, drive A lowest. On a pack, that would have tripped a
mid-move sag. Now (`PackMonitor`): the HIGHEST servo reading, low-passed every
tick (tau 2 s). It warns below 10.5 V and CUTS AND HOLDS below 9.9 V, like a
servo fault, so the station is told why. Both latch; a flat pack means a
full power cycle, and the HELD row says POWER OFF AND SWAP BATTERY. It won't arm a pack already below the cut. The servos' own voltage
limit is no backstop as set: Min Voltage Limit 6.0 V on all four, and Shutdown
(52 = overheat, shock, overload) does not include a voltage error. Replayed over the three torque-on
records, the filtered value bottoms at 11.68 V against drive A's raw 11.0.
On the brick (~12.0 V), `--pack-warn 12.5` shows the warning and adding
`--pack-cut 12.2` shows the cut. Both fired on the first tick after contact.
**A HOLD WAS SILENT to a station that arrived later.** The message rides
EventLog's 5 s, so a reconnect showed only "cut", was told "r once upright",
and `r` did nothing. Now `hold` is a telemetry LEVEL: a HELD row in the
mirror and on the terminal status line. The reconnect names the reason, and
a refused `r` is answered. This covers pack, servo fault and link holds. The
pack WARNING had the same flaw (gone after 5 s) and is a `warn` level, a
WARN row, until the cut replaces it with HELD.
**Outstanding:** a pack has never been on the bike.

**THE BENCH WINDING IS THE ATTITUDE, as the operator suspected.** Replaying
the torque-off bench record: with the recorded +3.4 deg roll the policy asks
for its full -458 deg/s steer rate on 100% of queries; the same record with
roll and pitch levelled splits 51/49 in sign. A lean the bike cannot correct
(no wheels on the floor, and torque off) is steered into forever. With torque
ON on the bench the wheel WILL follow, so expect the steer to spin
continuously at up to 458 deg/s until the mount is calibrated or the bike is
on the floor -- the lead bound caps the lead, not the rotation.

**THE BIKE'S SESSION, REWORKED 2026-09-18 -- including two ways it could
energise a bike it should not have.** All checked live on the Pi.

  * **`r` ENERGISED A `--no-torque` RUN.** A re-arm called `bus.arm()`
    unconditionally. The operator confirms it happened on the bench. Now
    gated on `--torque`, and a test pins it.
  * **A SIGNAL'S SHUTDOWN COULD NOT TURN TORQUE OFF.** SIGTERM and SIGHUP
    (`kill`, `timeout`, a dropped `ssh -t`) used to skip `shutdown()`
    entirely. Once they were routed through it, the torque-off came back
    `COMM_PORT_BUSY` (-1000): the exception had landed mid-transaction and
    left the SDK port marked busy. Now a signal inside the loop only requests
    the stop, honoured between ticks, and `ServoBus.close()` clears the flag
    and tries every servo (it used to stop at the first failure). SIGTERM,
    SIGHUP and ctrl-C each verified clean on the Pi.
  * **A second `run_bike` dropped the first one's bike.** Opening the bus
    turns torque off on every servo, and that happened before the second
    instance found port 9910 taken. The port is now bound first, as the
    single-instance lock; verified, the second exits without touching the bus.
  * **A dead link no longer ends the run.** Cut (policy servos limp, the
    righting servo holds), keep sensing and recording, wait; a returning
    station re-arms with `r`. Nothing re-arms while the link is down, not
    even `--auto-rearm`, and the settle dwell restarts on reconnect. The
    start-up wait is now forever by default (`--link-wait S` to bound it).
  * **A re-arm stalled the loop 68 ms** reloading the policy from disk, so the
    first ticks after a fall ran late. `run_bike` now reuses the loaded
    policy (stateless); after the fix, re-arm max jitter 2.69 ms.
  * **The mirror sent 30 commands/s, not 50** (measured from `link_age`): a
    20 ms gate against 16.7 ms frames fires every other frame. Now every
    frame, capped at 100 Hz, and still tied to rendering on purpose.
  * **`r` is a COUNT now, not a one-packet flag.** `rearm: true` rode in the
    single packet after the press, so one lost datagram ate it. The station
    sends `rearm_n` in every packet and the bike acts on a change -- except
    from a station it has not heard before, or while the link is down (a
    press made blind is not consent). Verified live across a link drop.
  * **The steer target is bounded: 45 deg of lead over the measured steer**
    (`control/steer.py`, `advance_target`), in training AND on the bike. The
    policy outputs a steer RATE, integrated into a position target with no
    bound -- torque off it ran 167 -> -4412 deg in 20 s; now it stops at -45.
    Invisible while the wheel follows: the steer actuator is already at its
    0.8 N.m limit above ~39 deg of lead at the fastest steer seen in sim, and a
    20 s mixed-command sim run is bit-identical with and without it (peak lead
    16.3 deg). A steer held by a "rock" for 0.3 s or more drops the bike either
    way; the bound halves the unwind rate (42 -> 22 rad/s at 0.3 s, 56 -> 9 at
    1 s). Existing policies need no retraining. A servo in Velocity Control
    Mode was rejected: it would drop the position hold the policies trained
    against, which is a different plant.
    On the REAL servo (P-only, Position P 900, no profile, PWM limit 885,
    read off servo 103), MEASURED on the first torque-on bench run: PWM
    rises ~3.3 %/deg of lead -- 37 % at 10-12 deg, 74 % at 20-30 deg -- and
    never saturated; the largest lead was 24.8 deg with the wheel turning at
    ~5 rad/s. Extrapolated, full PWM near ~30 deg: close to the sim's 23, so
    45 deg is very likely past saturation on hardware too, but that is not
    yet observed. An earlier note here predicted 11 deg from an e-manual
    formula quoted from memory; the measurement refutes it. (Lead is target
    minus the reading taken before the write, so it carries a few degrees of
    timing bias at speed.)
  * **The command is 12 packed bytes** (`telemetry.encode_command`), was
    76-150 B of JSON: version, rearm count, velocity x/y [mm/s], heading
    WRAPPED [1e-4 rad] (the bike only uses wrap_pi(psi_cmd - psi)), righting
    goal [1e-3 rad] and current, with -32768 = "not commanded". A JSON
    station is refused by name, once. Telemetry stays JSON for now.
  * **The bike tells the station what happened** (`telemetry.EventLog`):
    cut, re-arm, link lost/back, hardware error set/cleared, failsafe --
    decided and worded on the bike, each message riding every packet for
    5 s, printed once by the mirror and the terminal station and shown on the
    readout. The mirror's own state-change message is gone. Quiet cost ~11 B
    a packet; 791-972 B measured with messages in flight.
  * `--bench` = `--no-torque --allow-guess-mount`; `--bench --torque`
    energises. The steer zero stays the config's; `--steer-zero capture`
    is still there for a bare shaft.

The terminal station (`hw/ground.py`) is the fallback, not the main station,
and its docstring now says why its local mode is bench-only: run on the Pi,
the heartbeat comes from the bike itself, so a stalled wifi does not trip the
link watchdog.

`--frame-stats` now also prints LINK stats in `--mirror`: rx/s, inter-arrival
gap p50/p99/max, the fraction of frames that redrew a stale pose, and packet
age at draw time. Frame timing structurally cannot see this failure, which is
the lesson -- the instrument measured the loop, and the fault was in the data.

**AND THE VIEWER LOCK, FIXED ALONG THE WAY.**
Reported as chugging in `--mirror` but not in teleop, which pointed at the one
thing the two loops differ in that touches the render thread. It was a real
waste and not the reported fault -- sync now measures 0.10-0.17 ms, but it was
never the cause. The mirror's `apply_camera` took `viewer.lock()`
UNCONDITIONALLY every frame to compare one float, so every mirror frame took
the render mutex TWICE (there and inside `viewer.sync()`) where teleop takes
it once. Teleop's camera defaults to `free`, whose `apply_camera` returns
before any lock -- hence the asymmetry.

That mutex is held by the render thread WHILE IT RENDERS, so taking it is a
wait, not a few instructions. The mirror's camera is fixed-azimuth tracking
set once in `on_start` and only zoom ever changes it, so the lock bought
nothing. Now behind a dirty flag set by `-`/`=`. Pinned by a test that counts
acquisitions: **30 in 30 idle frames before, 0 after**, verified to fail when
the guard is removed.

Teleop's follow/overhead/wheel cameras DO lock every frame and have to -- they
recompute azimuth from the bike's yaw. So the prediction is that teleop in
follow mode chugs the same way; untested.

**AND THE COMPUTE WAS NEVER THE PROBLEM.** Measured the real teleop
closure -- policy, odometry, AHRS model, physics, overlay, four-bar -- over
3000 frames at 42 physics steps each:

| | ms |
|---|---|
| step (42x, policy included) | 2.02 mean, 2.37 p99 |
| draw | 0.83 mean, 0.96 p99 |
| whole frame | 2.85 mean, **3.22 p99**, budget 16.67 |
| frames over budget | **1 of 3000, and it is frame 0** |

The model is light to render too: 2 meshes, 708 vertices, 39 geoms. Scene
geoms cap at 201. What is left is `viewer.sync()` and the OS compositor, which
cannot be measured from a machine without the window on it -- so rather than
guess a third time, `--frame-stats [SECONDS]` now reports step / draw / SYNC /
sleep live in both loops. Sync is the column a headless profile cannot
produce; a healthy loop shows a large sleep. Each line covers only the frames
since the last, so an intermittent spike lands in the line it happened in
rather than being averaged away.

**A FALSE ALARM WORTH KILLING, NOT YET FIXED.** Every teleop run prints
"this policy is an artifact of a DIFFERENT machine". It is wrong: teleop
injects `floor_tilt_steps` into `params["sim"]` for the spawn dial, which
moves the IN-MEMORY plant digest eda849e7afaaca0f -> 044e2677741b0b33. The
comment there says "so neither digest moves", meaning the file. The policy
check hashes the mutated copy. Left alone deliberately -- digest behaviour is
not something to change in passing -- but it is training the reader to ignore
the one warning that would matter.

Suite **20 failed, 414 passed, 9 skipped -- red set unchanged**, 14 new tests.
Neither digest moves on disk (the new keys are under `control`).

**THE DRIVE SERVOS ARE MIRRORED AND THE FLIGHT CODE DID NOT KNOW (2026-09-16).**
`servo_sign_turning_hub_forward: [1, -1]` was measured on 2026-09-13 -- both
horns face outboard -- and written into
`docs/measurements/drivetrain-measurements.yaml`. Nothing under `src/` ever
read it. `analysis/drivetrain_bench.py` defaults to `--signs 1 -1` and has
always been right; `hw/dynamixel.py` and `hw/odometry.py` assumed [1, 1].

That swaps the two modes outright. A commanded common mode reaches the
hardware as a differential -- the bike crabs when told to drive -- and a real
forward roll reads as pure roller motion with `v_lon` near zero.

Caught on the bench by rolling the rear wheel forward by hand and reading the
two servos: **+3048 and -3048 counts**, `turned_a +13.962` against
`turned_b -13.958`, giving hub 0.002 rad. Signed, the same numbers give **hub
13.96 rad = 2.22 turns** and ring 0.002 -- a forward roll with the rollers
still, which is what the hand did.

`control.onboard.servo_sign` now carries it and `ServoBus` is the only
consumer, applied once on read and once on write. It is the boundary between
the servo frame and the input-shaft frame, and everything upstream -- the
estimator's hub mix, the ctrl vector, the model's joints -- is written in the
input-shaft frame and must not know servos exist. Moves NEITHER digest: the
simulator has no servos, so no trained policy can see it.

**EVERY TORQUE-ON RUN BEFORE THIS IS SUSPECT**, including 2026-09-16's. The
commanded `+32.6 / -7.4 rad/s` reached the hardware with B inverted, so what
the bike physically did is not what the telemetry said. Nothing was damaged --
the bike was held -- but do not read those numbers as a controller result.

A measurement that exists, is correct, is written down, and has no consumer is
the failure mode this repo keeps finding. Worth a grep before trusting that a
recorded fact is in force.

**`pytest -m hardware` passed 7/7 on 2026-09-15** — four servos on the Mac,
ids 101-104 at 3 Mbps. It failed 1/7 first (63 frames/s at a requested 200 Hz);
`adjust-ftdi-latency` fixed it, and a direct measurement then gave a 4-servo
FastSyncRead at **2.000 ms mean / 2.034 p99**, i.e. a 500 Hz read-only ceiling
and a tick that is entirely the FTDI latency timer. A SyncWrite is 25 µs and a
SECOND one is free: read + one write and read + two writes are both 2.000 ms.
See `pi-bench-bringup.md` §2.3.
