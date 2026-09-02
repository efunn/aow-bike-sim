# Control tables

One file per servo model, in ROBOTIS's own `.model` format, **vendored verbatim**
from `ROBOTIS-GIT/DynamixelSDK` (Apache-2.0):

    python/src/dynamixel_easy_sdk/control_table/xc430_w150.model
    python/src/dynamixel_easy_sdk/control_table/xc330_t181.model

Fetched 2026-09-01 from `raw.githubusercontent.com/.../main/...`:

    xc430_w150.model   md5 2797dd99fd80d4441583e7dd1eb53a64
    xc330_t181.model   md5 0fbe4e5cbd618fa21a62f12f79d6e420

**Kept byte-identical to upstream on purpose** — so a later `curl | diff` says
whether ROBOTIS changed anything, which a hand-retyped table can never do.
Anything we know that upstream does not goes in `overrides.py`, never in these
files. Re-fetch and re-diff rather than editing.

Parsed by `aow_sim.hw.control_table`. Sections used: `[control table]`
(address / size / name) and `[unit info]` (per-LSB scale). `[type info]` is
carried through but nothing reads it yet.

## The two model differences that actually bite

Both **verified against the hardware** on 2026-09-01 (4 servos on a U2D2 at
3 Mbps, 12 V brick), not inferred from the tables:

**1. Address 126 is the same address and width on both, and means different
things.**

| addr | XC430-W150 (model 1070) | XC330-T181 (model 1210) |
|---|---|---|
| 124 | Present PWM | Present PWM |
| 126 | **Present Load**, ±1000, 0.1% of max torque | **Present Current**, signed, mA |

The XC430 has no current register at all — no `Present Current`, no
`Current Limit`, no `Goal Current` — so it *infers* a percentage from duty
while the XC330 *measures* amps. An indirect map that points slot N at 126 on
every servo is byte-identical across models and still needs a per-model scale
at decode time. That is what `Register.unit` is for.

**2. The XC430 has a second indirect block; the XC330 does not.**

| | block 1 | block 2 |
|---|---|---|
| XC430-W150 | Address 168 / Data 224 | Address 578 / Data 634 |
| XC330-T181 | Address 168 / Data 224 | **none** |

Probed directly: reading address 578 returns `rc=0 err=0` on IDs 101/102
(XC430) and `rc=0 err=7` — a data-range error — on IDs 103/104 (XC330).

**Upstream's `Indirect Address Write` / `Indirect Address Read` entries are a
naming convention, not a hardware partition.** They are just two pointers into
the same general-purpose block — 168 and 578 on the XC430, 168 and 180 on the
XC330 — and the hardware does not care which entries carry goals and which
carry feedback. Every entry is freely allocatable in either direction, so
`IndirectMap` allocates the block as one pool and ignores the labels.

**Consequence:** block 1 is common to both models, is 28 entries, and is
sufficient for everything the bench tests need — so that is all this code
uses. One SyncWrite sets it up across a mixed bus. The 28-byte budget covers
reads and writes together; `IndirectMap` enforces it and names the overflow
rather than letting the setup silently address the wrong registers.

## Writing Operating Mode RESETS the position gains

Measured 2026-09-01 on an XC330-T181: writing `Operating Mode`(11) silently
rewrites `Position P Gain`(84) and `Position D Gain`(80) to that mode's
defaults, discarding whatever was there.

| mode | Position P | Position D |
|---|---|---|
| 3 position | 900 | 0 |
| 4 extended position | 900 | 0 |
| 5 current-based position | 700 | **1400** |

**Set gains AFTER the mode, never before, and read them back.** Any experiment
that compares two operating modes and sets its gains first is comparing the
gains as much as the modes — which is exactly what happened to the first
version of `r5_control_mode_comparison` in `docs/measurements/servo-measurements.yaml`.

Note also that `apply_map` drops torque, and anything that drops torque is a
good place for a gain to be quietly restored, so read back after that too.

## What is NOT in these files

Upstream's `[unit info]` covers velocity and current only. Everything else the
tests need — PWM %, Present Load %, position counts/rev, input voltage — is in
`overrides.py`, each with a `source:` note in the same spirit as
`config/bike_params.yaml`. Upstream's XC330 `Present Current` unit is given in
**N·m**, i.e. already multiplied by an assumed torque constant; `overrides.py`
carries the raw mA instead and leaves kt to `bike_params.yaml`, which annotates
its provenance.
