# The mini bike

> **Status: ACTIVE, stage A done, layout next.** Opened 2026-11-04. A smaller bike on XL330s
> and an ESP32-S3 instead of the Pi; the hardware side is
> `aow-bike-hw/docs/requirements.md`. Owns: how the mini is simulated, in
> what order, and what each stage found. "Smaller" is mostly SHORTER: the
> same wheelbase and wheels, with the servos, pulleys and electronics shrunk.

## How it is simulated

A **variant** is a file under `config/variants/` holding only what differs
from its base, named by a top-level `variant_of:` (relative to the file).
`load_params(path)` lays it over the base (`params._read_bike`), so every fix
to `bike_params.yaml` reaches the mini without copying, and a variant can
build on another.

- **Only existing keys.** A key the base does not hold is an error
  (`_merge_variant`), so a typo cannot quietly load the base bike under the
  variant's name. The exceptions: a new block under `servos:`, and
  `drivetrain.drive_servo`.
- **`!replace` hands a block over whole.** A block tagged `!replace` takes
  the base's place instead of merging, so it may add, drop and rename keys
  (it must still replace a block the base has). `mini.yaml` owns
  `bike.frame_parts` and `bike.payload` this way: the layout is the mini's,
  while contact, the drivetrain model, the wheels, the sensor models and the
  randomisation stay inherited. A part named as one of `params.PRINTED_PARTS`
  is still times `printed_mass_scale`; a new or renamed one is not.
- **`drivetrain.drive_servo`** names the drive servos' block. It is absent
  from `bike_params.yaml` (`params.drive_servo` falls back to `xc430_w150`),
  so adding it moved nothing: the built bike's plant digest is still
  d6e4355c.
- **A variant is a different plant.** Its `plant_digest` differs, so a policy
  trained on the built bike warns when flown on it, as it should.
- **Teleop flies the variant's drive.** `run_drive --params <variant>` takes
  the drivetrain overlay from the variant, not from the startup policy's
  training record, and says so (`DRIVETRAIN from ... (a variant)`).
- `tests/test_variants.py` (`pure`) pins all of the above.

```sh
mjpython -m aow_sim.run_drive --teleop --params config/variants/mini.yaml       # M288 drives
mjpython -m aow_sim.run_drive --teleop --params config/variants/mini_m077.yaml  # M077 drives
```

Nothing else takes a variant yet: training, the GPU path and the analysis
scripts call `load_params()` with no argument. An RL config key (`bike:`,
recorded in the export as `floor:` is) is the plan when a mini policy is
trained, not before.

## The layout (11-04)

`mini.yaml`'s `frame_parts` and `payload` were seeded as today's parts with
the mini's masses, each where the part it replaces sits, and build exactly
the plant they did as overrides (digest 912bc03c, 747.7 g, CoM 100.2 mm up).
Lay the mini out by editing those two blocks: positions in the chassis frame,
origin the rear axle, +x forward, +z up; the entry format is
`bike_params.yaml`'s `frame_parts` comment. After each edit:

```sh
python analysis/mechanism/mass_table.py config/variants/mini.yaml --against
```

lists every mass, its x / y / z and how far it pulls the CoM height, then
the total against the built bike. Still today's, not yet the mini's: the
rollcage rings and the wings (tied to the righting linkage's geometry), the
righting module, the fork and steering bodies. They come after the frame
parts and payload.

## Stages

Change one thing at a time, on a bike the sim already matches.

| stage | what changes | file | state |
|---|---|---|---|
| **A** | the two drives: XL330-M288 (or M077) and the 3M 32T/21T pulleys | `mini.yaml`, `mini_m077.yaml` | built, flown in teleop (11-04) |
| B | steer and righting to the M288 | | |
| C | the shorter layout, masses, the ESP32 and 2S pack | | |

**The servo plan** (user, 11-04): everything M288 for now. The drives, and
possibly the steer, can swap to the M077 and back, which is the comparison
worth having; the righting stays M288 for its torque.

## Stage A: the drives

| | built bike | `mini.yaml` | `mini_m077.yaml` |
|---|---|---|---|
| drive servo | XC430-W150, 12 V | XL330-M288, 6 V | XL330-M077, 6 V |
| stall / no-load | 1.6 N m / 106 rpm | 0.60 / 123 | 0.228 / 456 |
| pulleys, servo : wheel | 45 : 15 (HTD 5M) | 32 : 21 (3M) | 21 : 32 |
| `belt_ratio` (input / servo) | 3.0 | 1.524 | 0.656 |
| max at the input shaft | 33.3 rad/s, 0.53 N m | 19.6, 0.39 | 31.3, 0.35 |
| servo mass | 65 g | 18 g | 18 g |

Spun up in the air (gravity off, both drives at the ctrl limit, 1 s; scratch):
every plant reaches ~91 % of its no-load speed, the base's 10.06 of 11.10
rad/s at the servo, the M288's 11.68 of 12.88, the M077's 43.35 of 47.75.

**The firmware gains are each servo's own (11-04).** First flown at the
XC430's Velocity P 100 / I 1920, the M077 jittered hard in teleop. It was the
servo loop, not the policy: a register gain means more loop gain on a faster
servo (kp x stall / J = kp x w0 / the mechanical time constant), and the M077
is 4.3x the XC430's no-load speed. ROBOTIS ships it at P 40 / I 400 for that
reason; the M288 at P 180 / I 1600. The pointer on the floor, each bike file's
own overlay, hold and 0.4 m/s forward, 3 seeds each (scratch
`variant_jitter.py`): duty jitter is the RMS of the duty's step-to-step change.

| plant | gains | falls | duty jitter | duty at +-1 |
|---|---|---|---|---|
| built bike (XC430) | 100 / 1920 | 0/6 | 0.053 | 0 % |
| M288 | 100 / 1920 | 0/6 | 0.105 | 0 % |
| M288 | **180 / 1600** (factory) | 0/6 | 0.119 | 0 % |
| M077 | 100 / 1920 | 0/6 | **0.753** | 6.4 % |
| M077 | **40 / 400** (factory) | 0/6 | 0.071 | 0 % |

In the air (a step to 40 % of the limit) the M077 overshot 29 % at 100 / 1920
and 0 % at its factory gains. So the variants carry the factory gains, and
teleop with `--params <variant>` holds them over the policy's record (as if
`--servo-gains` were given; it says so). The bike itself sets these registers
at startup (`run_bike` writes the policy's training gain): a mini bike must
write its own servo's, or it will chatter the same way.

## Mass (11-04)

`mini.yaml` also swaps in the mini's lighter parts, each where the part it
replaces sits: the layout, and so most of the CoM height, is stage C.

| part | today | mini | source |
|---|---|---|---|
| drive servos | 2 x 65 g (XC430) | 2 x 18 g (XL330) | datasheet |
| steer servo (mass only; the actuator is still the XC330's) | 23 g | 18 g | datasheet |
| drive pulleys, 45T HTD 5M -> 32T 3M | 2 x 19.8 g | 2 x 4 g | GUESS |
| drive belts, 9 mm 5M -> 6 mm 3M | 2 x 15 g | 2 x 4 g | GUESS |
| Pi 3B+ + U2D2 -> ESP32-S3 board with the bus buffer | 50 + 9 g | 6 + 1 g | GUESS |
| Pi carrier plate | 12.6 g | 4 g | GUESS |
| AHRS (mass only; the sensor model is still the TM151's) | 19 g | 3 g | GUESS |
| cables, less the two USB ones | 41 g | 25 g | estimate |
| battery, 3S LiPo -> the 2S Li-ion | 76 g | 63 g | measured |

(Printed parts' values are still times `printed_mass_scale` 0.930.) The
righting servo stays the XC330 until stage B.

| | mass | CoM ahead of the rear axle | CoM height |
|---|---|---|---|
| today's bike | 1003 g | 84.3 mm | 108.4 mm |
| mini (either drive) | **748 g** | 86.1 mm | **100.2 mm** |

The pointer still holds still and drives at 0.4 m/s with no falls in 6 tries
on each (the jitter check above: M288 0.125, M077 0.071). What holds the CoM
up now is the chassis's 425 g at 128 mm, mostly the rollcage rings, the
righting module and the battery; moving those is the shorter layout.

**What is a guess.** The 6 V column is the highest ROBOTIS publishes; on 2S
the servos likely run faster. The firmware-loop scales (`kp_per_unit`,
`ki_per_unit`) are the XC430's fits, carried over because ROBOTIS's units
match; the delays and velocity filter are inherited unmarked. Rotor inertia
holds the XC430's loaded mechanical time constant (15.3 ms); Coulomb
friction holds its 9.4 % of stall, though part of that is the hub and
rollers, which do not shrink. All `GUESS`, each with how to measure it.
The servo boxes keep the XC430's shape; only their mass changes.

**What the policy sees.** The pointer was trained on the XC430's drive, so
flying it on stage A asks how far it carries, not what a mini policy could
do. Its own plant (54fdba72) is already not today's, so teleop's notes list
detent and slop differences that are not the variant's.
