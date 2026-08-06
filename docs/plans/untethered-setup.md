# Untethered setup

Transition from the umbilical (12 V/5 A brick, U2D2 to a laptop) to a
self-contained bike. Decisions below; ordering can proceed in parallel with
tethered testing and RL training. Status: decided, nothing ordered (2026-08).

Requirements this was decided against: functional, fast, low power, durable,
fits the bike (size over weight), fully understandable codebase, <$1000
(target <$100), basic soldering only, no custom PCBA.

## Summary of decisions

| question | decision |
|---|---|
| Microcontroller | **Raspberry Pi Zero 2 W** — runs the existing Python control stack unmodified |
| Firmware language | **Python** (numpy), same source tree as the sim. Not C, not MicroPython |
| ROS | **No.** 3 actuators, 1 IMU, one loop — ROS 2 is pure overhead here |
| Power | **One 3S bus** for all three servos; 5 V buck for the Pi |
| Charging | **Buy a hobby balance charger.** Do not build a charger circuit |
| U2D2 Power Hub | **Not used** — replaced by a ~25×25 mm perfboard splitter |
| Teleop | **WiFi/UDP from the laptop**, which stays a full ground station |

## Why a Linux SBC and not a microcontroller

The compute load is negligible. `general_rl` is a 15→128→128→3 MLP, ~18.7k
MACs, queried at 50 Hz, and `control/policy.py` already replays it in pure
numpy with no torch. The LQR is a 3×8 matrix-vector product. The bottleneck is
serial I/O, not arithmetic — so "fast" and "low power" do not discriminate
between the options, and the tiebreaker is code reuse.

A Pi Zero 2 W runs `src/aow_sim/control/` **unmodified**: the same
`MLPPolicy`, the same `SteerFrame`, the same `DriveController`. An MCU port
means re-implementing and re-validating `general_spec` / `steer` / `drive` by
hand, then porting every future sim change twice. That directly contradicts
the "fully understand the codebase" requirement — one codebase is easier to
understand than two that must be kept in agreement.

Cost of the choice: Linux is not hard-real-time. Mitigated by running the
control thread `SCHED_FIFO` on an isolated core and starting at 100 Hz rather
than 200 (see *Loop rate* below). If measured jitter turns out unacceptable,
the fallback is the hybrid — an MCU owning the bus at fixed rate — not a full
rewrite.

The Zero 2 W is 65×30 mm, ~11 g, ~1.5 W, ~$15. 512 MB RAM is ample: policy
replay is numpy-only and the deployment bundle (below) keeps MuJoCo and scipy
off the bike entirely.

### Port budget (this is what makes the Zero 2 W work)

The Zero 2 W has exactly one usable USB port, so every peripheral has to be
placed deliberately. It works out with no hub:

| device | connection |
|---|---|
| U2D2 | the single micro-USB OTG port (FTDI, `ftdi_sio`) |
| TM151 AHRS | **GPIO UART0** (pins 8/10) — TTL 3.3 V compatible, no level shifter |
| Teleop | WiFi (onboard). The gamepad plugs into the *laptop*, not the bike |

The TM151's USB-C virtual COM port is an alternative, not a requirement — the
UART path is what frees the USB port for the U2D2.

## Power

### One bus at 3S

Both servo types are 3S-native and share a single rail across the whole
discharge window. From the datasheets in `docs/robotis/`:

| servo | input range | recommended | stall @ 11.1 V | stall @ 12.0 V |
|---|---|---|---|---|
| XC430-W150 ×2 (drive) | 6.5–14.8 V | 12.0 V | 1.4 N·m, 1.3 A | 1.6 N·m, 1.4 A |
| XC330-T181 (steer) | 6.5–12.0 V | **11.1 V** | 0.76 N·m, 0.80 A | 0.80 N·m, 0.88 A |

**Watch the XC330's 12.0 V ceiling.** A freshly charged 3S sits at 12.6 V,
which is over it. Options: charge to ~12.3 V for testing, accept the brief
over-voltage window near full charge, or put a small drop ahead of the steer
servo. Decide at build time — it is not a reason to change chemistry, since
the pack spends almost all of its life below 12 V.

No separate rails, no dual-voltage wiring. This is the main reason the power
system is simple.

### Budget

| load | average | peak |
|---|---|---|
| 2× XC430 (drive, continuous balancing crawl) | 0.3–0.6 A each | 1.4 A each |
| 1× XC330 (steer) | 0.2–0.4 A | 0.88 A |
| Pi + AHRS via buck (~2 W) | ~0.2 A | ~0.3 A |
| **total** | **1.2–2.0 A** | **~4 A** |

A 1300 mAh 3S gives **35–60 min** of active driving. C-rating is irrelevant at
these currents — pick the pack on physical size and mass, not discharge spec.

**Bulk capacitance is not optional.** Motor current transients on a shared
pack will brown out the Pi. 470 µF low-ESR at the buck input and 1000 µF on
the servo rail. This is the one electrical detail that actually bites.

### Charging

Buy an ISDT Q6 Nano or SkyRC B6neo (~$45) and make the pack swappable via
XT30 + JST-XH balance lead. A 3S charger is the highest-risk,
lowest-value custom electronics in this project; there is no reason to build one.

### Low-voltage cutoff, free

The servos already measure the bus. Read **Present Input Voltage, control
table address 144** — no divider, no ADC, no extra part.

### No Power Hub Board

The U2D2 PHB only distributes power and the TTL bus, and it is bulky. Replace
it with a ~25×25 mm perfboard: three 3-pin JST-EH (B3B-EH-A) wired in parallel
(VDD/GND/DATA), an XT30 pigtail to the pack, and a pigtail to the U2D2's TTL
port. Basic soldering, through-hole only, no PCBA.

## Parts to order

| item | part | ~$ |
|---|---|---|
| Battery ×2 | 3S 1300–1500 mAh, XT30 + JST-XH (~115 g, 72×35×26 mm) | 50 |
| Charger | ISDT Q6 Nano or SkyRC B6neo | 45 |
| SBC | Raspberry Pi Zero 2 **WH** (pre-soldered header) | 18 |
| Storage | 32 GB A1 microSD | 10 |
| 5 V rail | Pololu D24V22F5 (5 V 2.5 A, 21×16 mm); D24V10F5 also sufficient | 16 |
| Bulk caps | 470 µF 25 V low-ESR + 1000 µF | 5 |
| Distribution | perfboard, 3× B3B-EH-A, XT30 pigtails, 20 AWG silicone | 20 |
| Safety | inline main switch + 5 A fuse, cutting **servo** power independent of the Pi | 12 |
| Cabling | micro-USB OTG adapter, TM151 UART pigtail | 16 |
| | **total** | **~190** |

Already owned: U2D2, TM151, 3 servos. Gamepad is laptop-side — use whatever is
on hand.

**Not buying, and why:** U2D2 Power Hub (bulky, replaced by the perfboard);
OpenRB-150 (would force a C++ rewrite); RC transmitter/receiver (WiFi carries
a richer command set — see below); any custom PCB.

Against the <$100 stretch target: the core electronics is ~$75 (Pi + SD + buck
+ connectors + safety). The remaining ~$115 is packs and a charger — reusable
consumables, and free if a charger is already on hand.

## Mass and CoM

This is the part that has to reach the simulator before more RL training.

The modeled bike is **825 g** with CoM at **z = 0.124 m**, x = 0.082 m forward
of the rear axle. Untethering adds:

| item | mass |
|---|---|
| 3S 1300 mAh pack | ~115 g |
| Pi Zero 2 W | ~11 g |
| U2D2 + cable | ~25 g |
| buck + distribution board + wiring | ~40 g |
| **total payload** | **~190 g (+23%)** |

**Placement barely matters for balance; the mass does.** Across the entire
plausible mounting envelope the CoM height only moves 0.117→0.131 m, and the
inverted-pendulum fall time constant (τ = 1/√(g/h)) only moves 0.109→0.116 s —
a 6% spread. So do not over-optimize pack position. What does matter:

- **Keep it on the centerline.** Any lateral offset is a standing roll bias the
  controller has to trim out continuously.
- **Keep it near the rear axle** — yaw inertia is what the pivot and flick
  moves fight, and it is far more sensitive to fore/aft placement than roll is
  to height.
- **Model what you actually build.** Enter the as-built mass and position in
  `config/bike_params.yaml` and re-run the LQR design; that matters more than
  any placement choice.

### Authority derating — the real sim-to-real risk

Two effects compound and both are currently unmodeled:

1. **+23% mass** — outside the `mass_frac: 0.1` domain randomization band.
2. **−12% servo torque**, because the sim's `stall_torque`/`no_load_rpm` are
   the 12 V column and a 3S pack averages 11.1 V, sagging to 9.9 V at cutoff.
   `control.drive.v_max: 1.2` was derived as ~70% of a 12 V no-load ceiling.

Together that is roughly a **30% authority reduction** relative to what
policies are currently trained against. Both are fixable in config today, and
both must land before the next `general_rl` run, or the trained policy will be
optimistic about a bike that does not exist.

## Software architecture

### No ROS

Three actuators, one IMU, one control loop, one command source. ROS 2 would
add DDS latency and RAM pressure on a 512 MB board and insert machinery
between the operator and code that is meant to be fully understood. If ROS is
worth learning — and it is — learn it on a project whose structure benefits
from it. A telemetry-only ROS 2 bridge could be bolted on later without ever
entering the balance loop.

### Which controller deploys

**`general_rl` is the deployable controller. LQR is a bench tool.**

This falls out of the observation contracts, not preference. `general_spec.build_obs`
takes roll, roll_rate, yaw_rate, steer, steer_rate, v_lon, v_lat, the live
command, and prev_action — **no world position**, by design (see the
"stationary observation" rationale in `control/general_spec.py`). Every one of
those is directly measurable on hardware.

`LQRBalance` is not deployable in the same sense: `extract_state`
(`control/balance.py`) needs `e_lon`/`e_lat`, world XY against an anchor,
which onboard can only be dead-reckoned and will drift. That is fine for the
seconds-long horizon following a fresh anchor, and fine on the bench — but it
is not a controller you leave running.

### Process structure

One Python process, three threads:

- **Control thread — `SCHED_FIFO`, 100 Hz.** Per tick: one **FastSyncRead** →
  build the state shim → `DriveController.step` → one **SyncWrite**. Bus at
  **3 Mbps** (X-series supports up to 4.5 Mbps).

  **Indirect addressing is what makes that one-in/one-out.** Set up once at
  startup (`Indirect Address 1` = 168 → `Indirect Data 1` = 224), then:
  - *Read:* Realtime Tick (120), Present Position (132) and Present Velocity
    (128) are **not** contiguous in the control table. Indirection maps them
    into one 10-byte block, so a single FastSyncRead returns all three servos
    in **one** status packet rather than a round trip each.
  - *Write:* the drives need Goal Velocity (104) and the steer needs Goal
    Position (116) — different registers, which SyncWrite cannot mix. But each
    servo can map *its own* goal register to the *same* indirect address, so
    one 4-byte SyncWrite drives all three. No BulkWrite needed. (SyncWrite is
    already the fast path: it is broadcast with no status packets, which is why
    Protocol 2.0 has no "fast write" instruction.)

  Conveniently **XC430-W150 and XC330-T181 have identical control tables** for
  every register used here, including the indirect blocks — so one table
  serves both and no per-model handling is needed.

- **Timing comes from the servos, not from `sleep`.** Realtime Tick (120) is
  the servo's own millisecond clock, stamped when it sampled its encoder.
  Its delta is the control-loop `dt`, so the estimator and the controller's
  zero-order hold integrate over time that actually elapsed rather than the
  rate we hoped for. It wraps at 32768 ms — handled, and tested.

- **Velocity re-estimated from position, not taken from Present Velocity
  (128).** The servo's own estimate is a ~50 ms boxcar on XL330/XC330 — ~25 ms
  of lag against a 113 ms fall time constant. Position and velocity ride in
  the same read block, so re-deriving it costs nothing. Counts are differenced
  over the measured tick delta and passed through a short, recency-weighted
  moving average (`RateFilter`), with the servo's own number kept alongside
  for cross-checking.

  Defaults swept in sim against ground truth at 100 Hz (RMS error, mm/s of
  bike speed):

  | window | taper | lag | standstill | drive 0.6 | circle |
  |---|---|---|---|---|---|
  | 10 ms (raw diff) | — | 5.0 ms | 8.33 | 7.49 | 6.66 |
  | **25 ms** | **0.5** | **8.3 ms** | **7.85** | **5.04** | **4.38** |
  | 50 ms | 0.5 | 21.7 ms | 10.74 | 3.84 | 3.34 |
  | *servo's Present Velocity* | | *~25 ms* | | *~9.5* | *~8.5* |

  So the default is **both quieter and ~3× less laggy** than the servo's
  estimate. Longer windows keep helping while driving but get *worse* at
  standstill, where the residual is real crawl motion being smoothed away
  rather than quantization noise — hence 25 ms, not 50. Window quantizes to
  whole ticks (20 ms and 25 ms are the same 2-tap filter at 100 Hz).

  Keep it in proportion: closed-loop, the bike balanced identically on ideal /
  50 ms-averaged / differenced feedback (max roll 1.5–1.7°), because the fast
  state comes from the AHRS and these rates only feed slow outer loops. Cheap
  insurance for the real machine, not a fix for a demonstrated instability.
- **AHRS thread — 200 Hz**, GPIO UART → a latest-value slot (single writer, no
  lock needed).
- **Link thread — 50 Hz**: UDP command receive, telemetry transmit.

### Loop rate — and why the servos are NOT sampled as fast as possible

Three rates, deliberately decoupled. They are not one number:

| what | rate | set by |
|---|---|---|
| `general_rl` policy | 50 Hz | how it was trained; `_gen_every` retimes it |
| controller tick | 100 Hz | the servo bus round trip (below) |
| AHRS | 200 Hz | free-running; costs the control loop nothing |

**The 100 Hz is a bus budget, not control theory.** The servo bus is the only
synchronous request/response in the system — the servos produce data only when
asked, over a shared half-duplex line, in the same thread that must then write
commands. Per tick at 3 Mbps:

| | |
|---|---|
| FastSyncRead instruction + status, SyncWrite | ~100 bytes ≈ **330 µs** wire |
| Return Delay Time (factory default 250 × 2 µs) | **500 µs** → now set to 0 |
| U2D2 / FTDI USB round trip | **~0.5–1.5 ms**, and it dominates |

So ~1–2 ms per tick on a Zero 2 W: 10–20% duty at 100 Hz, but 50–100% at
500 Hz, leaving nothing for the controller. **100 Hz is a conservative
starting point for the Pi, not a derived limit** — it is `--rate`
configurable, p99 jitter is logged every tick and printed at shutdown. Measure
and raise it. A laptop sustaining 500 Hz on the same U2D2 is not a
contradiction: the Pi Zero 2 W's single USB 2.0 OTG on a slow SoC is the
difference, which is exactly why the number should be re-measured on the
target rather than inherited.

**Oversampling the servos does not improve velocity.** Position quantization
noise over a span T is q/T regardless of how many samples fall inside T —
consecutive differences share endpoints and telescope. Measured, holding the
filter window fixed at 25 ms:

| sampling | 100 Hz | 200 Hz | 500 Hz |
|---|---|---|---|
| velocity noise | 22.6 | 23.9 | 25.6 mm/s |

5× the bus traffic, no gain (marginally worse), and it eats the timing margin
the control loop needs. Faster servo sampling *does* help for catching
transients that would otherwise alias (ball impacts) and for system ID — but
those are recording problems with no realtime deadline, so do them on the
tethered rig where the bus is free.

**The AHRS is the opposite case and should run fast.** It free-runs and pushes
— no request, no response, no contention, its own UART, its own thread. 200 Hz
(or 400) costs the control loop nothing; the only cost is baud (see
`hw/ahrs.py`: 200 Hz needs 230400).

**`latency_timer=1` is mandatory.** The `ftdi_sio` default is 16 ms, which
makes any loop above ~30 Hz impossible and is a notorious silent failure. Set
it with a udev rule *and* assert it at startup rather than trusting it.

### Concurrency: three threads, zero locks

| thread | rate | does |
|---|---|---|
| control (main, `SCHED_FIFO` 80) | 100 Hz | FastSyncRead → `DriveController` → SyncWrite |
| ahrs | 200 Hz | UART → parse → publish |
| link | 50 Hz | UDP command in, telemetry out |

Every cross-thread handoff is a **single atomic rebind of a freshly built
object** — `AhrsReader._latest`, `CommandLink.cmd`, `CommandLink.telemetry` —
with one writer and one reader each. No locks anywhere, on purpose: a control
loop must never block on a sensor, and a reader only ever wants the *newest*
value, so there is nothing to queue and nothing to contend for. A torn read is
impossible because the object is fully constructed before the name is rebound.

Staleness is handled by failing, not blocking: `AhrsReader.latest(max_age=
0.05)` raises rather than returning a frozen attitude, because a controller
confidently balancing against stale attitude is worse than one that cuts
torque.

#### Threads, not processes — measured, not assumed

These are threads, so the GIL serializes them. `serial.read()` releases it for
most of the AHRS thread's life, but frame parsing is Python and holds it. What
bounds the damage is not total CPU but **how long the sensor thread holds the
GIL in one go**, since that is the longest a control tick can be stalled
waiting for it:

| | per frame | at 200 Hz | worst-case stall on a 10 ms tick |
|---|---|---|---|
| `parse_frame` here | 7.5 µs | 0.15% CPU | 0.07% |
| scaled ~15× for a Zero 2 W | 112 µs | 2.2% CPU | **1.1%** |

Running the real pattern (100 Hz loop + 200 Hz thread doing real `parse_frame`
work) changed p99 tick jitter by **0.011 ms**. So threads are adequate and
multiprocessing is not needed for this workload. The CRC being table-driven
(7.4× faster) is part of why.

**Do not lower `sys.setswitchinterval`.** It defaults to 5 ms, which looks
alarming next to a 10 ms period, but that bound only bites if a thread does
5 ms of uninterrupted CPU and nothing here does. Measured, lowering it made
worst-case jitter *worse* (2.5 → 3.3 → 4.1 ms at 5 ms / 0.5 ms / 0.1 ms),
because the extra context switches cost more than the contention they avoid.

**Multiprocessing works fine on the Pi** — it is Linux with 4 A53 cores, and
separate processes genuinely run in parallel where threads cannot. It is the
right escalation if the jitter log ever says so, and the split would be: AHRS
read+parse in its own process publishing to an `mp.Array`, control loop
untouched.

What must NOT move to another process is the servo I/O. `dynamixel-link` puts
the Dynamixel port in a child process, which is exactly right for a recording
or monitoring rig — the parent samples whenever it likes and nothing waits.
But here read → compute → write is one tight sequential chain on the critical
path, so splitting it would add IPC latency twice per tick to the one loop
that cannot afford it. Keep the bus in the control process; move sensors that
only ever *publish*.

### The laptop stays a ground station

WiFi/UDP was chosen over a 2.4 GHz RC receiver specifically because the
command surface is richer than steering and throttle. The bike receives a full
command struct — `v_cmd_world`, `psi_cmd`, controller mode (the `,` toggle),
move triggers, re-zero, log start/stop — which is exactly the surface
`run_drive.py` teleop already exposes. The pyobjc hold-to-turn key handling
stays laptop-side and keeps working unchanged, and telemetry streams back for
live plotting.

The Pi should run its own WiFi AP (hostapd) so the link never depends on house
networking.

### Failsafes

WiFi has no failsafe of its own, so it is built in software. All four are
required before the first untethered balance attempt:

1. **Command-age watchdog.** No packet for >150 ms → zero the velocity command
   (the policy keeps balancing, which *is* the safe state). >1 s → torque off.
2. **LVC** from address 144 → park and torque off below ~10.2 V (3.4 V/cell).
3. **Fall detect.** |roll| > 60° (matching `fall_roll_deg`) → torque off, so it
   does not thrash on its side.
4. **Physical switch** cutting servo power independently of the Pi.

## Onboard code layout

### Deployment bundle — no MuJoCo on the bike

`DriveController.__init__` calls `design_gain_schedule(params, model)`, which
numerically linearizes the MuJoCo model and pulls in scipy. That is a poor fit
for a 512 MB board and a slow startup.

Instead, `python -m aow_sim.export_deploy` runs on the laptop and writes
`deploy/bundle.npz`: the gain schedule (`speeds`, `Ks`), `qpos_eq`, the address
map (steer qposadr/dofadr, actuator ids), `ctrlrange`/`ctrllimited`, and
nq/nv/nu. A `from_bundle` constructor loads it.

This is the same trick `moves/*.npz` already uses to make policy replay
torch-free. **Onboard install is `numpy + pyyaml + pyserial + dynamixel-sdk`**
— no MuJoCo, no scipy, no torch, no gymnasium, no MJCF compile at boot.

Getting to *no MuJoCo* took four fixes, each of which had made the claim false
in a way that is invisible on a laptop where everything is installed:

- `control/__init__.py` eagerly imported `.balance`, so even
  `from aow_sim.control.steer import ...` — a numpy-only module — pulled the
  whole stack in. Now lazy via PEP 562 `__getattr__`.
- `balance.extract_state` called `mujoco.mju_quat2Mat`. Replaced with a
  six-line `quat_to_mat`, tested identical to MuJoCo's to 1e-12.
- `load_params` lived in `build_model.py` next to `mjSpec`. Split into
  `aow_sim/params.py`; `build_model` re-exports it.
- `LQRDesign` — the dataclass whose docstring says it "exists so the
  controllers can be built WITHOUT MuJoCo" — sat in `linearize.py`, which
  imports MuJoCo at module scope. Moved to `control/lqr_design.py`.

Remaining MuJoCo calls in `pivot`/`flick` are lazy and offline-only (trajopt
scoring), so replay does not touch them.

`tests/test_hw_no_mujoco.py` enforces all of this by importing every onboard
module with MuJoCo/scipy/torch masked out. It asserts the mask works *first* —
the initial version used the `find_module` API that Python 3.12 removed, so it
blocked nothing and passed everything.

### Hardware abstraction — duck-type mjData

The controllers touch mjData shallowly: 113 `data.qpos`, 76 `data.qvel`, 21
`data.time`, 21 `data.ctrl`, and nothing else on the balance/drive path. So
the shim is a duck type, not a refactor — `DriveController.step(model, data)`
then runs **verbatim** on hardware.

`src/aow_sim/hw/`:

- **`state.py`** — `HardwareData` with `.qpos` (nq zeros), `.qvel` (nv zeros),
  `.time`, `.ctrl` (nu). Only `qpos[0:7]`, `qpos[steer_qposadr]`, `qvel[0:6]`,
  and `qvel[steer_dofadr]` are ever filled. Plus `DeployModel`, exposing the
  handful of `model.*` attributes `_Base.__init__` reads.
- **`dynamixel.py`** — bus wrapper: SyncRead(128,8) / BulkWrite, mode config,
  torque enable, `latency_timer` assertion, address-144 voltage. Unit
  conversions come from `control/steer.py` (`XC330_COUNTS_PER_RAD`,
  `clamp_extended`) — never duplicated.
- **`ahrs.py`** — TM151 reader: quaternion → `qpos[3:7]`, body-frame gyro →
  `qvel[3:6]`. `extract_state` already does `mju_quat2Mat` on `qpos[3:7]` and
  reads `qvel[3]`/`qvel[5]` as body-frame roll/yaw rate, so these are direct
  writes. Needs a **mounting-misalignment calibration** — the AHRS sits at
  `[0.05, 0, 0.13]`, not at the chassis origin.
- **`odometry.py`** — the only piece with real uncertainty (below).
- **`run_bike.py`** — the three-thread process, UDP protocol, failsafes.

### Velocity estimation

`qvel[:2]` (world linear velocity) has no sensor. Synthesize it from the
drivetrain math that is **already written and sign-verified**:

```
ω_input   = ω_servo × belt_ratio (3.0)
ω_hub     = (ω_a + ω_b)/2              → v_lon = ω_hub × outer_radius (0.0512)
d         = ω_a − ω_b                  → v_lat = lat_gain(params) × d
```

### Lateral: use the FRONT wheel, not the rear rollers

Inverting the AOW's roller kinematics does not work — the rollers are
*designed* to slip in that axis, so the encoder reports what was commanded,
not what happened. Measured over upright episodes it lands anywhere from +0.96
to −0.20 correlation with truth depending on regime, and open-loop it
over-predicts 2.5–3.8×.

The front wheel is an ordinary tire and **cannot slide sideways**. A normal
bicycle has that rolling constraint at *both* contacts, which over-determines
the planar velocity; the AOW deliberately removes the rear one — leaving
exactly one constraint and exactly one unknown. Writing the front contact
velocity as the rear's plus the yaw lever arm and requiring it to lie along
the front wheel's ground heading θ:

```
v_front_body = (v_lon, v_lat + yaw_rate·L)
-v_lon·sin θ + (v_lat + yaw_rate·L)·cos θ = 0
```
```
v_lat = v_lon·tan θ − yaw_rate·L
```

Every input is a good measurement — hub odometry, the steer encoder (via
`wheel_heading`, which already handles the raked axis and is π-periodic so
multi-turn winding needs no rebasing), and the AHRS gyro. None of them touch a
slipping surface.

**Measured, 6495 samples over standstill / shoved standstill / straight
0.6 m/s / circles at R=0.8 and R=0.5:**

| estimator | RMS error | correlation |
|---|---|---|
| rear-roller kinematics | 7–23 mm/s | −0.20 … +0.96 |
| **front-wheel constraint** | **6.1 mm/s** | **+0.993** |

A free least-squares fit of the same regressors returns tan-coefficient
**0.985** (theory: 1.0), **L_eff = 0.2033 m** (geometric wheelbase: 0.200) and
a roll-arm of ~0, reaching 5.2 mm/s. The formula as written, with **no
calibration constant at all**, is within 17% of the best achievable fit. The
geometry *is* the answer — there is nothing to identify on hardware.

End-to-end, driven by the model's own AHRS sensors (so with the real
lever-arm terms — the TM151 sits at `[0.05, 0, 0.13]`, not at the chassis
origin) at the real 100 Hz control rate:

| regime | v_lon | v_lat |
|---|---|---|
| standstill | 0.3 mm/s | 1.2 mm/s |
| standstill + shoves | 1.8 mm/s | 13.5 mm/s |
| straight 0.6 m/s | 8.7 mm/s | 1.0 mm/s |
| circles R=0.8 / R=0.5 | 8.0 / 7.4 mm/s | 5.2 / 5.2 mm/s |

**The accelerometer is a fallback, not a co-equal sensor.** Both direct
measurements are already good to a few mm/s, so a conventional complementary
filter actively hurts: a 0.3 s longitudinal blend degraded v_lon from 8.8 to
**174 mm/s** RMS by integrating the AHRS lever-arm terms. Integration is used
only where the constraint is blind.

**Where the constraint fails:**
- **θ → ±90°.** `cos θ → 0` and the perpendicular front wheel constrains
  nothing. Not hypothetical — the `flip` maneuver pre-steers to exactly 90° to
  free the front (`control.flip.hold_deg`). Confidence is weighted by `cos²θ`
  and the estimator coasts on integrated acceleration through it.
- **Front wheel off the ground** (wheelie, hard braking, bump). The constraint
  silently stops being true and no onboard sensor says so. A known blind spot.
- **Front tire actually sliding** near the friction limit — degrades
  gracefully, unlike the rear.

`qpos[:2]` is the integral of this and **will drift**. Acceptable, because
`general_rl` never reads it and LQR/moves only need it over the seconds after
a fresh anchor.

## Open items

- ~~**TM151 frame parsing is not implemented.**~~ **Done.**
  `hw/ahrs.py::parse_frame` is a port of SYD Dynamics' EasyProfile C library
  (`TransducerM_Lib_Protocol_C` v1.2), not a guess: `AA 55 | size | payload |
  crc16`, CRC-16/Modbus over `size + payload`. It decodes `EP_CMD_COMBO_`
  (43), the one message carrying quaternion, gyro and accel atomically under a
  single timestamp — mixing attitude and rate from different sample instants
  would inject phase error into the balance loop. 20 tests, frames built by an
  independent encoder written from the C struct offsets.

  **Set the AHRS baud before power-on:** a Combo frame is 68 + 5 = 73 bytes,
  so 200 Hz needs ~146 kbps. **115200 will not keep up** — use 230400, or
  460800 if you want 400 Hz.
- **Front-wheel liftoff is a blind spot.** The lateral estimator assumes the
  front wheel is on the ground. Under hard acceleration or braking it may not
  be, and nothing onboard detects it. Candidate proxies: a large pitch-rate
  excursion, or v_lon accelerating faster than the constraint predicts. Not
  needed to start; needed before anything aggressive.
- **Front tire lateral stiffness is a `GUESS`.** The constraint's accuracy on
  hardware depends on the real tire not sliding. Check it on the floor by
  driving a known circle and comparing integrated position.
- **Steer homing at power-up.** `control/steer.py` notes that "real hardware
  re-homes at power-up" — the XC330 loses its extended-position multi-turn
  count across a power cycle. A startup homing routine is needed and it depends
  on an unmade mechanical decision: hard stop, or magnet/hall index. Blocks
  first power-on, not ordering.
- **The open-loop moves no longer survive the payload.** With the battery and
  electronics modelled, `flick`, `flick_fwd` and the scripted `flip` fail
  their tests (the closed-loop RL moves all still pass — a good advertisement
  for them). The trajopt knots were authored for the 825 g bike. Re-author with
  `python -m aow_sim.optimize_flick --name flick_untethered` when the as-built
  mass is known; regenerating them against estimated mass would just be work
  thrown away twice.
- **XC330 over-voltage near full charge** (above) — pick one of the three
  mitigations at build time.
- **As-built payload mass and position** — the numbers above are estimates;
  weigh and measure at assembly, then re-run `python -m aow_sim.export_deploy`.
- **Servo IDs** — `hw/dynamixel.py` assumes drive_a=1, drive_b=2, steer=3. Set
  them with ROBOTIS Wizard before first run.

## Verification

Bench-first. The bike should not attempt to balance untethered until 1–3 pass.

1. **Loop timing, no bike.** Pi + U2D2 + 3 servos on the pack. Measure
   SyncRead+BulkWrite round trip and tick jitter over 60 s at 1/2/3 Mbps, and
   with `latency_timer` at 16 vs 1 (to see the failure mode).
   **Gate: p99 tick jitter < 1 ms at 100 Hz.**
2. **AHRS.** TM151 on GPIO UART at 200 Hz; check the quaternion against known
   hand-held orientations; measure age-of-data at the control tick.
3. **Replay equivalence — the test that proves the shim. DONE.**
   `tests/test_hw_replay.py` drives the real controller in a MuJoCo loop, then
   replays only the slots the hardware backend can populate through a
   bundle-built controller and requires bit-identical `ctrl` over 1500 steps —
   for LQR line mode, for the general policy, and for a controller built from
   `deploy/bundle.npz` with no MuJoCo model and no linearization. Verified to
   have teeth: starving the shim of `steer_qpos`, `steer_qvel`, the
   quaternion, or `time` is each caught within 50 frames.
   `tests/test_hw_odometry.py` and `tests/test_hw_ahrs.py` pin the odometry
   accuracy and the quaternion conventions. **All need no hardware.**
4. **Odometry.** Bike on the testbed stand; command known wheel speeds and
   compare the estimator's `v_lon`/`v_lat`. Then push the bike a measured
   distance on the floor and check integrated position.
5. **Failsafes, deliberately triggered.** Kill the laptop WiFi mid-run (expect
   command zeroed, then torque off), pull the pack below LVC on the bench, lay
   the bike on its side.
6. **First untethered balance** with physical outriggers matching the model's
   `training_wheels` (half_span 0.10, clearance 0.002), on a mat, with a safety
   line — then without.
