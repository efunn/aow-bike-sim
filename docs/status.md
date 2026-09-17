# Project status — 2026-09-09

**How to read this.** This file is the navigation layer: what is true now, what
to do next, and where the detail lives. It is rewritten, not appended to. Every
section either states a fact measured on the date at the top of this file, or
points at the plan doc that owns the subject. **If you want the reasoning,
follow the link — it is not repeated here.**

Rewritten 2026-09-08 in a docs triage. The previous version had grown to 2801
lines across six partial amendments and had become the sole record of several
workstreams; that material moved into the plan docs listed below, verbatim.

---

## Where the project is, in five lines

1. **The simulator is trusted for control design.** Parametric MuJoCo model,
   procedural omni-wheel contact, and — since 08-26 — the *sensors* the bike
   will actually have, in the loop and in training.
2. **RL drives.** `general_rl*` policies balance and drive from a live
   (velocity, heading) command. The analytic LQR is a reference baseline only.
3. **The onboard software path is built and proven in sim** — hardware shim,
   deploy bundle, odometry, AHRS protocol — with no assembled bike to run it on.
4. **Hardware has started.** Four servos on a bench, 2026-09-01; the rear
   drivetrain assembly built and characterised by hand 2026-09-12/13. The
   Digi-Key order landed 2026-09-08: Pi 3, cables, electronics. No pack, no
   charger.
5. **The goal is a functioning bike with sample videos, in weeks.** Everything
   below is ranked against that.

---

## What to do next

Ranked by what unblocks the most, not by interest.

| # | do | why now | doc |
|---|---|---|---|
| 1 | **Weigh the electronics stack and pack** | Two `GUESS`es die in ten minutes; the parts are on the bench today | `first-physical-test.md` §0a |
| 2 | **Contact calibration — P0, P0b, P1, once per floor** | Needs no printing: a weight, a caliper, slow-mo, a tilting board. The contact is the least-measured thing in the sim and the one no policy has been randomised over — and the SPREAD across surfaces is what sets the randomization range | `floors-and-the-contact-model.md` |
| 3 | **Finish the drivetrain station** | Built and characterised 2026-09-12/13: belt ratio 3.0 confirmed, a 7.5° detent in the differential, and a velocity-loop resonance at ~22 Hz at firmware P 400 that P 200 does not have. Roller slop measured by hand 2026-09-13: ±1.5 mm at the roller's 22 mm diameter, 15.6° p-p, from the same gear chain as the detent (20 detents per roller turn). `k_roller` 2.4 confirmed by counting roller turns. Open: choose the firmware Velocity P **and I** gains — a P/I grid cut time stuck at the diff detents from 49 % (factory) to 12 % at P 400 / I 3840, but P 400 rings near 22 Hz and I 3840 rings harder (peak 1.09–1.20), and the sim must model whichever ships — then a torque-scale check (D5, a known added inertia -- no lever arm needed) and fitting the five drivetrain `GUESS`es from the captures. **The sim cannot pick the gain (2026-09-14):** on the drive model fitted to these captures, policies trained at each gain tie — 3 seeds each, median 0.749 at P 100 against 0.750 at P 400, each on its own plant — but P 400 buzzes the rear wheel ~3× harder at 8–32 Hz. Decide on the physical bike, with a policy trained at the gain that ships | `drivetrain-measurements.yaml`, `drivetrain-model.md` |
| 4 | **Umbilical bring-up on the laptop** | Verification steps 1–2 need no pack at all | `untethered-setup.md` §"Bench power" |

All four are bench work. **The sim-side item is being taken now:**

> ### → NEXT, in progress: fix the eval score's directional gate
>
> `_score` is `survive_rate × track_geo` (`train_general_rl.py:243`).
> `speed_ratio_fwd` is computed and deliberately excluded, so the score cannot
> see a policy that abandons a direction — demonstrated once at a cost of 12M
> steps, and **every future long run is exposed**. Ranked risk #2.
>
> Open decision before the edit: gate `BestByScore` on a
> `min(speed_ratio_fwd, speed_ratio_rev)` floor (selection-only, leaves every
> recorded number comparable) versus adding a directional term to `_score`
> itself (stronger, but re-bases every score in `docs/` and every
> `moves/*.yaml`). See `eval-score-rewrite.md`.

Not next, and deliberately: the self-righting wings (design done, build last),
the ball shot (works, off the path), the privileged critic (speculative), the
odometry rewrite (flown around, seven accepted red tests).

---

## The five workstreams

| workstream | state | blocker | owner doc |
|---|---|---|---|
| **Simulation & model** | Working. 17 parameters still `GUESS`. A detailed drivetrain (fitted XC430 loop, diff detent, roller slop) exists as an opt-in overlay, in the eval env, teleop and training (per config). Trained at P 100 and P 400, 3 seeds each (2026-09-14): a tie on score. Teleop builds a policy's own drivetrain from its record | Physical parts to measure; the built bike, to confirm the drivetrain model | `mujoco-modeling-decisions.md`, `drivetrain-model.md` |
| **Control — RL** | Working, and primary. Trains against the onboard sensors | Crab still one-sided; `turn_asym` stuck ~0.2 | `general-rl-improvements.md` |
| **Sensor modelling** | Largely DONE. Velocity estimate, encoder quantisation, TM151 error — all in training, validated against a real unit over USB | Dynamic attitude accuracy needs a moving bike | `sensor-workstream.md` |
| **Control — analytic (LQR)** | Reference baseline only. Marginally healthy | Nothing now; degrades when contact moves | `old/stationary-balance-controller.md` |
| **Hardware / untethered** | Servo bench 2026-09-01. Rear drivetrain assembly on the bench 2026-09-12/13, hand-held, recorded with `analysis/drivetrain_bench.py`. Bus at 500 Hz on the Mac only after `adjust-ftdi-latency`. **Onboard software readied for a Pi bench session 2026-09-15** — ground station, firmware-gain writes, fourth servo, fall cut/re-arm; none of it has touched hardware | Firmware P-gain choice, torque calibration, then the chassis | `pi-bench-bringup.md`, `first-physical-test.md`, `drivetrain-measurements.yaml` |
| **CAD** | Layout, drivetrain, steering and righting stations pinned. Electronics packing deferred on purpose | Nothing — it is being worked on | `cad-onshape-workflow.md` |
| **Self-righting mechanism** | Side project. Moved to asymmetric output / symmetric layout; torque analysis not trusted | Will be resolved by building, not by analysis | `wing-linkage-design-and-optimization.md` |

---

## Where everything is written down

`docs/plans/` holds reasoning and grows forever. This file is the layer on top.
Every doc opens with a **status banner** saying whether it is active, open,
parked or reference — read that before the body.

| doc | what it owns |
|---|---|
| `first-physical-test.md` | **the build order.** Which parts unlock which unknowns, in what sequence. Plus the three servo bench stations and the 09-01 bench results |
| `untethered-setup.md` | the physical bike: power, wiring, onboard software, Pi setup, verification. The umbilical path is §"Bench power" |
| `pi-bench-bringup.md` | **the Pi bench session**: what the four servos + TM151 + Pi 3B+ can prove without a chassis, the seven code gaps that block it, and what to measure |
| `sensor-workstream.md` | the odometry/AHRS arc and the full policy standings — why sensor-trained policies win 1.00 to 0.20 |
| `eval-score-rewrite.md` | how policies are SCORED, why the early numbers hid a failure, and the command-distribution audit |
| `seed-sweep-and-personalities.md` | **the 12-seed sweep**: determinism, the failure taxonomy, why reverse is easier, early abort detection, and the options not taken |
| `general-rl-improvements.md` | collected RL findings. Reference, not a queue |
| `mujoco-modeling-decisions.md` | why the model is built the way it is. Reference |
| `odometry-rewrite.md` | the estimator itself, and why rewriting it was skipped |
| `floors-and-the-contact-model.md` | **the multi-floor plan** — solref/solimp history, why the parameters read backwards, the per-geom blocker, and how the spread across surfaces becomes a randomization range |
| `aow-contact-approximations.md` | the contact surrogate survey, the timestep/mesh result, and the two blocked drive-plant fixes |
| `drivetrain-model.md` | **the detailed drivetrain**: servo, detent and slop fitted to the Station A captures, the replay check against them, what the existing policies do on it, and why firmware gains and policy are one decision |
| `cad-onshape-workflow.md` | the Onshape round trip and its API quota, plus everything drawn so far |
| `wing-linkage-design-and-optimization.md` | the righting mechanism as it now stands |
| `self-righting.md` | where recovery stops being possible; the fall cases. Reference |
| `params-digest-split.md` | the two digests and what each answers |
| `asymmetric-actor-critic.md` | a parked option, kept to cover the bases |
| `ball-shot-move.md` | the ball shot. Works, parked |
| `plans/old/` | six retired docs — built, superseded, or never started |

`docs/measurements/` holds hand-entered protocol/data pairs, plus
`servo-logging.md`: which registers to record off a Dynamixel, per frame and
per session, bench versus onboard, and why each earns its bytes.
`docs/cad/` holds generated CAD output (`.fs`, `.png`, the layout YAML) — never
hand-edit those; regenerate with `aow_sim.cad_layout` / `cad_servo_mount` /
`cad_swing_linkage`.

---

## Health

**Test suite, measured 2026-09-16** with `pytest -n 10 --dist load`:

    20 failed, 358 passed, 9 skipped, 45.3 s
    red set unchanged (20 accepted failures) -- tests/expected_failures.txt

**Three came OFF the red list**, and by a route nobody was looking down:
`actuators.steer_kv` 0.05 -> 0.0676 cleared
`test_front_constraint_beats_roller_kinematics` and both
`test_fused_estimator_end_to_end[standstill*]`. Confirmed causal by swapping
that one value and re-running -- 3 passed at 0.0676, 3 failed at 0.05. The
estimator was never what failed in those three; it was being judged against a
plant whose steering was under-damped. The other four odometry entries are
unmoved, so `odometry-rewrite.md` still has a job.

+36 passing against 09-14, all from the onboard bring-up work: 18 in the new
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

Read the verdict line, not the FAILED count. The 23:

| file | red | what it is |
|---|---|---|
| `test_drive.py` | 15 | 7 trajopt (re-authored once the as-built mass is known) + 8 analytic LQR |
| `test_hw_odometry.py` | 7 | estimator quality, no falls — see `odometry-rewrite.md` |
| `test_teleop.py` | 1 | analytic LQR, reaches v_max then tips |

A bare `pytest` is SERIAL and takes ~145 s. Run the marker, not the suite —
`pytest -m pure` is 0.4 s and is the edit loop. `pytest --markers` is the
reference for which marker covers what.

**LQR: marginally functional, and that is the intended state.** Holds the bike
at standstill at 1.17° peak roll over 40 s. Recovered by two weights after the
drive plant was armed: `q_roll_rate` 6.0 → 30.0 (the velocity-PI servo puts a
pole at the origin that the 8-state model does not carry; de-tuning `r_drive`
over a 100000× range does not substitute) and `q_steer` 0.5 → 5.0 (the steer was
pinned at its clamp 86% of the time, which read as oscillation and was really a
saturated actuator). Those two are the knobs to reach for when contact moves.

**`MIN_FIT_R2` is 0.93, and the bar was the thing that was wrong.** Nothing has
ever cleared 0.98 — not the current plant (**0.9489** since the 09-16 steer
damping change, 0.9412 before it), not the servo without its integral term
(0.9757). The fit gets *worse* as the contact gets more realistic:

| `contact_solref` dampratio | 0.3 | 0.5 | **1.0 (ships)** | 2.0 |
|---|---|---|---|---|
| worst R² | 0.8148 | 0.9297 | **0.9412** | 0.9602 |

(That row is at `steer_kv` 0.05 and has NOT been re-swept at 0.0676; the
shipping column alone is now 0.9489. The monotonic trend is the load-bearing
part and a damping change to a different joint has no reason to reverse it.)

The drop test implies ~0.30 — the worst-fitting value. There is no setting that
is both faithful and well-fitting. **Re-derive the bar from the measurement
rather than carrying 0.93 forward**, once the contact is measured.

**The three ground contacts are separately addressable (2026-09-09).**
`roller`, `front_tire` and `righting` can each carry their own `solref` and
friction via an optional `sim.contact_parts` block. `geom_priority` is the
mechanism — `solref` combines by solmix-weighted average and `friction` by
elementwise max, so separate values alone would give the mean, never the softer
part. Landed as a **bit-exact no-op**, verified over a 3000-step driven rollout:

| | max abs Δqpos vs pre-split | bitwise identical |
|---|---|---|
| split, identical params | `0.000e+00` | **yes** |
| split, `front_tire` overridden | `1.164e-01` | no |

`tests/test_contact_parts.py`, 9 tests under the `contact` marker, pins both —
the no-op *and* that an override bites, because a bug dropping `contact_parts`
would otherwise pass as a perfect no-op.

**Digests, re-verified 2026-09-11 — `deploy/bundle.npz` matches all three
(`plant_digest`, `design_digest`, and the legacy whole-file
`params_digest` aa232834f462a229):**

    plant_digest   eda849e7afaaca0f    was this trained against the machine I am running?
    design_digest  2db6c647ff3a2d59    were these gains designed against the weights I am running?

**`plant_digest` MOVED on 2026-09-16** (was `e1ec36bfa670217e`), and everything
in `moves/` is now an artifact of a different machine — `load_move` says so on
load. What moved it: `actuators.steer_kv` 0.05 -> 0.0676, the XC330's own
back-EMF droop `stall_torque/no_load_speed`, replacing a number that was a
GUESS in spirit; and `righting.{arm,wings}.servo_kp/kv` from tuning knobs
(30.0 / 1.0) to the servo's firmware gains referred to its shaft (1.62 /
0.0519). The bundle was re-exported and the LQR redesigned: worst fit 0.9412 ->
0.9489, gains moved 21% of scale on the drive input and 27% on the steer input
at the worst speed (v = -0.50).

**ACCEPTED, not outstanding, for the policies.** They were all trained at the
old value and are provisional in the digest's sense, but the change makes the
simulator MORE like the bike, not less — so retraining is the fix and reverting
is not. `general_rl_cmd_curriculum2b` has not been re-evaluated on the new
plant; do that with `analysis/per_command.py` before reading anything into a
hardware run.

`deploy/bundle.npz` matches both. A failing digest check is the mechanism
working — fix it with `python -m aow_sim.export_deploy`, never by loosening the
check. See `params-digest-split.md`.

---

## What drives the bike

`control.general_move` names **`general_rl_cmd_curriculum2b`** (repointed
2026-09-14). That is the command-family curriculum line: sensor-trained like
`odo_ahrs`, plus pitch, and the config the 12-seed sweep ran.
`seed-sweep-and-personalities.md` §8 has `personality1`, bit-identical to it,
as the best of the twelve on behaviour, while `_score` ranks it 8th. Teleop's
`--general` now defaults to the pointer instead of a hardcoded
`general_rl_cmd_curriculum2`. The pointer was chosen on 09-11 and never landed:
the digest worry that stopped it was about the legacy whole-file digest, and a
pointer edit moves neither `plant_digest` nor `design_digest`.

**Six policies trained ON the detailed drivetrain (2026-09-14), not pointed
at.** `general_rl_drivetrain_p{100,400}_{0,1,2}` are curriculum2's config at
firmware Velocity P 100 or P 400. Score on the eval grid:

| policy | ideal | drivetrain P 100 | drivetrain P 400 |
|---|---|---|---|
| `curriculum2b` (the pointer) | 0.595 | 0.638 | 0.177 |
| P 100 seeds | 0.36–0.48 | **0.741–0.751** | 0.180–0.503 |
| P 400 seeds | 0.05–0.14 | 0.42–0.54 | **0.625–0.803** |

Each arm beats the pointer on its own gain, and every policy is specific to the
plant it trained on, the pointer included. **The pointer stays because the
drivetrain model is not yet confirmed against the built bike**, not because
these lose. P 400 buzzes the rear wheel ~3× harder at 8–32 Hz (`chatter.py
--plant`). Teleop builds a policy's own drivetrain from its record
(`--general general_rl_drivetrain_p100_1`). Reproduce the table with
`analysis/drivetrain_eval.py --variants ideal full full_p400`.

The table below predates the curriculum line and is kept for the sensor
argument it makes. The split that matters is not one good policy against the rest — it is
**trained-against-the-sensors against not**. On the eval grid at
`--encoder counts --ahrs tm151`, score / survival:

| policy | trained against | tau 2.0 | tau 0.19 (measured) |
|---|---|---|---|
| `smooth_diff_pi` | MuJoCo truth | 0.110 / **0.20** | — |
| `odo_ahrs` (default until 09-14) | estimate + attitude, tau 2.0 | **0.672** / 1.00 | 0.570 / 0.95 |
| `odo_ahrs_rand2` | + attitude randomised, narrow | 0.654 / 1.00 | **0.663** / **1.00** |

Every truth-trained policy falls in four episodes out of five.

**Two pointer questions from before the curriculum line** (superseded as
pointer candidates by it, kept as findings):

1. **`rand2` vs `odo_ahrs`.** `rand2` is better at the tau the bike actually has
   and is nearly tau-invariant (spread 0.009 against 0.102). `odo_ahrs` keeps a
   tighter worst-case heading excursion once settled and much lower chatter. The
   case for keeping the current pointer is weaker than it looked.
2. **`glide_pitch_hub3` is a separate line entirely** — the calm one. Rim travel
   over a 15 s hold 7.58 → 3.35 m, airborne 58% → 13%, peak contact force
   7.23 → 3.66× weight, kick recovery 8/8 through dv 0.35. It has never been
   merged with the sensor line.

Neither is decided. Both are cheap to try — **repointing `control.general_move`
moves NEITHER digest.** Full standings and the whole sensor arc:
`sensor-workstream.md`.

**Caveats that keep biting.** Every row above was measured over 15 s episodes;
an eval now runs 5 s, so nothing here is comparable to a number measured after
2026-08-30. And every row is `--encoder counts` — a grid that does not force the
encoder scores older policies on a different bike, not a better result.

---

## The parameters that are still guesses

17 marked `source: GUESS` in `config/bike_params.yaml`, each with a note on how
to identify it. Never quietly promote one. The load-bearing ones:

| parameter | value | dies at |
|---|---|---|
| `chassis.mass` | 0.45 kg — **44% of the bike** | stage 2, the frame |
| `contact_solimp` | MuJoCo stock | contact calibration (`dmin` confounds both bench tests) |
| `friction_sliding` | 0.9 | the incline slide, P0b — cheapest test in the project |
| `input_armature`, hub/roller damping + frictionloss | 5 of them | the drivetrain station, spin-downs. Fitted 2026-09-14 into the OPT-IN overlay only (armature 2.44e-4 at the input shaft; Coulomb 0.05 N.m there, ~60x the hub guess) — not promoted in `bike_params.yaml` |
| `payload.pack.mass` / `electronics.mass` | 0.115 / 0.076 kg | **a scale, today** |
| `min_pinion_radius` | 0.006 m | print a test pinion |

**The contact model as it currently ships**, since it comes up often:

    contact_solref: [0.005, 1.0]     # positive convention: (timeconst_s, dampratio)
    contact_solimp: [0.9, 0.95, 0.001, 0.5, 2.0]   # MuJoCo stock, GUESS
    friction_sliding: 0.9   friction_torsional: 0.005

~16× stiffer than MuJoCo's default. It ships **critically damped so it cannot
bounce**, which is known to be wrong — the real wheel bounced 2–3 times, implying
dampratio ~0.30. The plan is to measure, then **switch to the negative
`solref: [-stiffness, -damping]` form**, which is what MuJoCo recommends for
system ID and is the only thing that decouples the two numbers: in the positive
form `timeconst` sets damping *and* stiffness while `dampratio` sets stiffness
only, so a static reading fixes only the product. Under a negative pair the
`dmin` confound shrinks from 2.3× to 6%. The derivation, the prediction tables
and the procedure are in `measurements/contact-protocol.md`, which is **correct
and current** — fixed 2026-08-22, re-checked 09-08. What is missing is data:
every field in `contact-measurements.yaml` is still 0.0.

---

## Risks, ranked

1. **No trained policy has ever seen contact-stiffness variation.** The
   randomization axis exists but sits commented out in `config/rl_general.yaml`.
   **Most likely single cause of a policy that works in sim and not on the
   floor.** It depends on no measurement, which is the point — so it is the one
   thing a long run grid should do before the bike exists.

   **Re-measured 2026-09-11, `analysis/floor_sweep.py`, `general_rl_odo_ahrs`,
   score / survival over the 20-command grid, mu 0.9. The 09-09 column is kept
   beside it because it was acted on and because the two disagree in ways that
   change the conclusions — it was taken with NO AHRS in the observation, which
   `policy_env_overrides` could not supply until 09-11 (see
   `docs/plans/sensor-workstream.md`). The right-hand column is the policy
   against the attitude error model it actually trained on, tm151 at its own
   declared tau of 2.0:**

   | sink @ bike weight | `contact_solref` | 09-09, no AHRS | **09-11, tm151** |
   |---|---|---|---|
   | 13.17 mm | `[0.020, 2.00]` | 0.594 / 1.00 | 0.600 / 1.00 |
   | 3.80 mm | `[0.020, 1.00]` | 0.657 / 1.00 | 0.660 / 1.00 |
   | 1.08 mm | `[0.005, 2.00]` | 0.673 / 1.00 | 0.679 / 1.00 |
   | 0.52 mm | `[0.020, 0.30]` | **0.719** / 1.00 | 0.651 / 0.95 |
   | 0.11 mm | `[0.005, 0.50]` | 0.719 / 1.00 | 0.616 / 0.90 |
   | **0.39 mm** | **`[0.005, 1.00]` — ships** | 0.663 / 1.00 | **0.686** / 1.00 |
   | 0.04 mm | `[0.005, 0.30]` | 0.374 / **0.70** | **0.175** / **0.35** |

   **Four rows move by under 0.03 — inside the ±0.02 seed-noise floor — and two
   move a lot, so the ordering did NOT survive the correction.** The stiff-end
   cliff is twice as deep as published (0.374 / 0.70 → 0.175 / 0.35), and the
   two rows that tied for best at 0.719 both fall below the shipped contact and
   both start dropping episodes. The shipped `[0.005, 1.00]` is now the top row
   in the table rather than the fifth.

   **Rows are sorted softest first, and `solref` is labelled by SINK because the
   raw pair reads backwards.** `dampratio` appears only in the stiffness term,
   as `1/dampratio^2`, so dropping it 1.0 → 0.30 makes the contact ~10× STIFFER,
   not softer. `timeconst` is the honest softness knob — it sets damping and
   stiffness together, sink going as `timeconst^2`.

   What it says:

   - **Soft contacts are fine.** Survival is 1.00 from 13 mm of sink all the way
     down to the shipped 0.39 mm, and score varies by 0.09 across a 34× span.
     Unchanged by the correction.
   - **The cliff is at the STIFF end**, and only there: `[0.005, 0.30]` —
     0.04 mm of sink — drops survival to **0.35**, not the 0.70 first
     published. That is the corner the drop test points at, because a stiff
     contact against unchanged damping is what bounces, and it is a worse
     corner than this section said for two days.
   - **Sink alone does not determine the outcome; damping matters
     independently.** This survives, but the evidence for it inverted. It used
     to be that two rows TIED FOR BEST at 0.719 across a 5× sink spread
     (`[0.005, 0.50]` and `[0.020, 0.30]`); corrected, those two rows sit at
     0.616 / 0.90 and 0.651 / 0.95 while the shipped contact — between them in
     sink — leads at 0.686 / 1.00. Non-monotonic in sink either way, which is
     why the negative `(-stiffness, -damping)` form is the one to measure into.

   Read the table as *how the current policy copes*, not as what is achievable:
   nothing has ever trained over this axis. A run with `dampratio_range`
   uncommented is the experiment that would flatten it.

   **Friction is NOT the flat axis for survival — that claim was an artifact of
   the same missing AHRS.** Re-measured 2026-09-11 across mu 0.5–2.0 at the
   shipped contact, whole-grid score moves 0.686 → 0.686 → **0.549** and
   survival drops to **0.90 at mu 2.0**. The 09-09 reading was 0.630 → 0.663 →
   0.643 at survival 1.00 throughout, which is where "the flat axis" came from.
   What DOES survive is the effect it was quoted for: the `hold` family's drift
   still rises monotonically, 0.754 → 1.242 → 1.815 m, i.e. **more grip means
   more wander, not less**. The
   policy uses slip as a brake, so a grippier floor converts more of its sawing
   into travel. The whole-grid score cannot see this: `hold` is 1 command of 20
   through a geometric mean. Use `--by-family`.

   **A CORRECTION, recorded because it was acted on.** An earlier version of this
   section read the dampratio axis backwards — it called `dampratio 0.30` a soft
   contact and concluded "drive on the hardest surface available; a foam mat is
   the failure mode." **That is inverted.** Soft is the safe direction here and
   stiff is the cliff. The error came from trusting the parameter's NAME over
   `contact-protocol.md`'s own CORRECTION, which states the formulas plainly.

2. **A third of training runs silently produce a policy that drives BACKWARDS
   when told forward** — measured 2026-09-10 across 12 seeds: 4 broken, 1
   partial, 7 competent, all differing only by `algo.seed`. `_score` is still
   `survive_rate × track_geo`, so three of the broken ones outrank the best
   policy in the set. Detectable from ~5M of 20M steps, but a naive abort would
   have killed the run that recovered to the second-best result. Nothing is
   implemented; see `seed-sweep-and-personalities.md` for the taxonomy, the
   reward mechanism behind it, and the options.
3. **The front tire and the righting wings still have the compliance of a TPU
   roller — now fixable in one line.** `roller`, `front_tire` and `righting` were
   made separately addressable on 2026-09-09 (`sim.contact_parts`, via
   `geom_priority`), shipped as a **bit-exact no-op**: all three still resolve to
   the same globals, so nothing moved and no export became provisional. The
   fidelity gap is unchanged until someone sets a value — a rigid printed wing
   striking the ground is still modelled as rubber. What changed is that it is
   now a one-line, deliberate edit that moves `plant_digest`, rather than a
   refactor. See `floors-and-the-contact-model.md` §4.
4. **Authority derating.** Real servos will not deliver modelled torque at
   modelled bandwidth. **Partly quantified 2026-09-14** on the fitted servo
   model (`drivetrain-model.md`): the ideal-drive policies measured hold at
   factory gains and lose most of the grid at P 400 (≤ 0.18), the gain the bench
   liked. That loss was the policy/plant mismatch: policies trained AT P 400 get
   it back (0.63–0.80), at the cost of a rear wheel that buzzes ~3× harder.
   The torque SCALE is still the datasheet 1.6 N.m. The bench replay cannot
   see it (fitted inertia and friction scale with it); the bike can. D5 or
   servo-strength randomisation covers it.
5. **Left/right asymmetry, and crab.** `turn_asym` has never gone below ~0.2 at
   any run length; crab is one-sided in every champion. The plant is
   mirror-symmetric and the handedness *flips sign between policies* —
   spontaneous symmetry breaking, not plant asymmetry.
6. **Steer homing at power-up is undesigned.** The XC330 loses its multi-turn
   count across a power cycle and the fix depends on an unmade mechanical choice.
   Blocks first power-on, not ordering.
7. **Front-wheel liftoff is undetected by the estimator.** Quantified —
   `analysis/liftoff.py` measures 79 mm of clearance in one arm.

*Retired 2026-09-08: "the contact protocol docs are stale." They were fixed on
08-22 and re-verified against `analysis/contact_calibration.py` on 09-08. The
risk was the claim, not the docs.*

---

## Explicitly not being worked on

The PD cascade (legacy, kept compiling as a manual-debug fallback). The trajopt
moves `flick` / `flick_fwd` / `flip` — 7 of the 23 red tests — re-authored once
the as-built mass is known, once rather than twice. Roll-phase and surface
contact studies. A live gamepad front-end. The ball shot. Re-tuning the LQR
weights, deferred until the contact model is pinned. The disturbance curriculum,
parked until the params files are reconciled.

**The wheel-only `--variant testbed` model does have a physical counterpart** —
it is stage 1 of `first-physical-test.md`. An earlier version of this file said
otherwise; that was written before the build order existed.
