# Physical build

**The bike is under construction.** The subassemblies exist, the self-righting mechanism is built and operating, and the onboard control loop is proven on a Raspberry Pi against real servos and a real AHRS. This page describes what the machine is and how it goes together, and what has been proven on real hardware so far.

---

## Bike summary

A ~1 kg self-balancing RC bike, about a 200 mm wheelbase, whose rear wheel is an active omni wheel: 8 driven axles carrying 16 truncated-cone rollers. Two servos drive that wheel as a differential: together they roll the bike forward/backward, and in opposite directions they move the contact patch sideways, which allows the bike to balance without a rider or a flywheel.

Four Dynamixel servos, one AHRS, one single-board computer, one battery. No custom PCB, no microcontroller firmware, basic soldering only.

## The CAD is generated from the same numbers as the simulator

`config/bike_params.yaml` holds every physical measurement with its provenance marked `measured`, `tooth-count`, `datasheet`, `design` or `GUESS`, and it is the **single source** for both the MuJoCo model and the CAD. The drawn bike is not a separate model that has to be kept in step; it is generated:

```sh
python -m aow_sim.cad_layout --format featurescript --push --shot
python -m aow_sim.cad_servo_mount
python -m aow_sim.cad_swing_linkage
```

Those emit **FeatureScript** — Onshape's scripting language — write it straight into a Feature Studio, and render the result. So a dimension cannot silently diverge between the thing being simulated and the thing being physically built: change the parameter, regenerate, and both move together.

`docs/cad/` holds the generated output (`.fs`, `.png`, the layout YAML). **Never hand-edit those** — regenerate them.

The full round trip, the API quota it runs against, and some good-to-knows in [cad-onshape-workflow.md](../plans/cad-onshape-workflow.md).

## What is already proven

| | when | what it means |
|---|---|---|
| **Servo reversal** | 2026-09-01 | The drive scheme works on real hardware, not just in sim |
| **Rear drivetrain assembly** | 2026-09-12/13 | Belt ratio 3.0 confirmed; a 7.5° detent in the differential and ±1.5 mm of roller slop measured and now modelled |
| **The control loop on a Pi** | 2026-09-16 | 100 Hz with four servos energised and the AHRS live: tick jitter mean 0.24 ms, p99 0.79, max 1.56 over 30 s. `DriveController.step` itself is 0.03 ms — the same as on an M4 Mac, so the bus is the budget and SBC compute was never a risk |
| **Self-righting four-bar** | 2026-09-17 | Built and operating |

**Not proven:** the chassis (under construction), the contact model against a real floor, the pack and its low-voltage cutoff, and anything about how a trained policy behaves on the real machine.

---

## Parts

The design-intent list. **Exact manufacturer part numbers, links and substitutions are TBD**.

### Actuation and sensing

| item | part | qty | notes |
|---|---|---|---|
| Rear drive | Dynamixel **XC430-W150** | 2 | The omni-wheel differential. Velocity mode, through a 3:1 belt |
| Steering | Dynamixel **XC330-T181** | 1 | Extended-position mode — continuous, 360°+ |
| Self-righting | Dynamixel **XC330-T181** | 1 | Current-based position mode, driving a four-bar |
| Orientation | **TM151** AHRS | 1 | UART or USB serial; both transports supported |
| Bus interface | ROBOTIS **U2D2** | 1 | Bench and umbilical use |
| Rear wheel | **HC-802** RC bike donor | 1 | The active omni wheel. Geometry reverse-engineered in `measurements/omni-wheel-protocol.md` |

### Electronics, and how it is wired

One 3S pack feeds **one bus**. The servos take it directly; a small regulator drops it to 5 V for the single-board computer. There is no power hub board and no custom PCB.

    pack ──▶ main switch ──▶ fuse ──▶ ┬──▶ servo chain (4× Dynamixel, daisy-chained)
                                      └──▶ 5 V regulator ──▶ SBC

The SBC is a Raspberry Pi for now. The bike is **1 power + 2 USB**: one USB each for the servo bus and AHRS.

Exact part numbers, substitutions and a complete from-scratch order can eventually be derived from [untethered-setup.md](../plans/untethered-setup.md#parts-to-order).

### Printing and fabrication

Chassis, fork, servo mounts, the belt drive and the righting linkage are 3D printed, from the generated CAD above.

---

## Bring-up

**Works tethered.** Can use a 12V bench. See [untethered-setup.md § Bench power](../plans/untethered-setup.md#bench-power--the-same-bike-tethered).

### Onboard software

The Pi runs the controllers, **not** the simulator, and the onboard code is written so it can never access one: `tests/test_hw_no_mujoco.py` hides mujoco, scipy, torch, gymnasium, SB3, matplotlib and tensorboard behind an import blocker and asserts that all eighteen onboard modules still import. For example, `control/__init__.py` once eagerly imported `.balance`; then importing the numpy-only `control.steer` pulled in MuJoCo transitively. 

Nothing stops you putting MuJoCo on the Pi, so **`--no-deps` is required** to avoid an install. The project's base dependencies are unconditional, so a plain `pip install -e .` drags MuJoCo and scipy onto the Pi. Install the runtime explicitly, then the package without its dependencies:

```sh
pip install numpy pyyaml pyserial dynamixel-sdk
pip install --no-deps -e ~/aow-bike-sim
```

The LQR gain schedule ships to the bike as a **digest-pinned bundle**, so the Pi needs neither a MuJoCo model nor scipy:

```sh
python -m aow_sim.export_deploy        # -> deploy/bundle.npz
```

That bundle is hashed against the physical parameters it was built from. **If loading it raises, that is the mechanism working** — it means `bike_params.yaml` moved and the bundle is stale. Re-export; never loosen the check.

### Running it

```sh
# on the bike
python -m aow_sim.hw.run_bike --bundle deploy/bundle.npz

# on a laptop -- the real bike drawn in the MuJoCo viewer
mjpython -m aow_sim.run_drive --mirror aowbike.local

# or the no-MuJoCo terminal fallback station
python -m aow_sim.hw.ground --host aowbike.local
```

`run_bike` is three threads:

| thread | rate | what it does |
|---|---|---|
| control | 100 Hz | SyncRead → `DriveController` → SyncWrite |
| AHRS | 200 Hz | UART into a latest-value slot |
| link | 50 Hz | UDP command in, telemetry out |

**The policy itself runs at 50 Hz**, no matter what the loop around it does. `DriveController` reads `control_rate_hz` off the move file and holds each action for the correct number of ticks, so the loop rate only has to be a multiple of the policy's. The simulator uses 200 Hz and the bike uses 100 Hz; both query the policy every 50 Hz either way.

### The failsafes

WiFi has no failsafe of its own, so these are mandatory:

1. **Command age** — past 150 ms, the velocity command is zeroed (the policy keeps balancing, which *is* the safe state); past 1 s, torque off. Armed only once the ground station has been heard from.
2. **Pack voltage** — warn below 10.5 V, cut and hold below 9.9 V, read off the highest servo's input voltage low-passed over 2 s. No extra hardware.
3. **AHRS stale** — torque off.
4. **A physical switch** cutting servo power independently of the Pi. The only one that still works if the software fails.

**Falling over is not in that list.** It is expected, survivable and self-clearing, so it is a state transition rather than a reason to stop: the fall guard drops torque on the three servos the policy drives, leaves the righting servo live, and lets the policy back in once the bike has settled.

### The radio

**The bike and the ground station share an ordinary router.** Currently, the bike joins at 2.4 GHz band and the ground station (laptop) at 5 GHz. Same box, same LAN, so the UDP link is a normal LAN hop.

**Turn off power-save on the Pi.** It is on by default and produces latency spikes of tens to hundreds of milliseconds — comfortably inside the 150 ms command-stale window, so the bike would zero its velocity command mid-drive for no visible reason. Asserted off at startup, not assumed.

Bandwidth is a non-issue: a command struct at 50 Hz.

The radio in full, the options that were weighed, and how to change the bike's network over ssh: [untethered-setup.md](../plans/untethered-setup.md#the-radio-and-what-actually-goes-wrong-with-it) and [§ Changing the bike's network](../plans/untethered-setup.md#changing-the-bikes-network-without-a-monitor-or-the-sd-card).

---

## Before you change a physical parameter

`config/bike_params.yaml` is not an ordinary file. Every entry carries a `source:` — `measured`, `tooth-count`, `datasheet`, `design` or `GUESS` — and `GUESS` means "placeholder, still to be identified". Never quietly promote one.

Changing a value there:

1. re-export the deploy bundle (almost always);
2. re-check the LQR fit — `control/linearize.py` warns below its R² floor;
3. treats every policy trained under the old value as provisional;
4. moves tests — `pytest -m "contact or geometry or deploy"` is the reachable set;
5. needs the cost written down as **accepted** or **outstanding**. An accepted
   cost with no note becomes an unnoticed one.

---

## Detailed documents

| doc | what it owns |
|---|---|
| [untethered-setup.md](../plans/untethered-setup.md) | power, wiring, onboard architecture, the full sourcing list, the radio |
| [first-physical-test.md](../plans/first-physical-test.md) | the build order, and the three bench stations |
| [pi-bench-bringup.md](../plans/pi-bench-bringup.md) | what the Pi + four servos + AHRS proved without a chassis |
| [drivetrain-model.md](../plans/drivetrain-model.md) | the fitted drivetrain: servo loop, diff detent, roller slop |
| [floors-and-the-contact-model.md](../plans/floors-and-the-contact-model.md) | the multi-floor contact plan |
| [self-righting.md](../plans/self-righting.md) | where recovery stops being possible |
| [wing-linkage-design-and-optimization.md](../plans/wing-linkage-design-and-optimization.md) | the righting mechanism as built |
| [cad-onshape-workflow.md](../plans/cad-onshape-workflow.md) | the CAD round trip |
| `measurements/` | protocol + data pairs: what to measure, how, and the numbers |
