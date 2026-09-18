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
| 3S pack, LVC at 10.2 V | 12 V brick. The LVC can never fire; `VOLTAGE_MIN` is untestable here |
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
