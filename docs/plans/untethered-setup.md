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
| Teleop | **WiFi/UDP from the laptop**, which stays a full ground station and hosts the AP |

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
| TM151 AHRS | **GPIO UART0** (RX only, pin 10) — TTL 3.3 V compatible, no level shifter. See *TM151 wiring* under Sourcing |
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
it with a ~25×25 mm perfboard carrying **two** 3-pin JST-EH (B3B-EH-A) wired in
parallel (VDD/GND/DATA) — one to the head of the servo chain, one to the U2D2's
TTL port — plus an XT30 pigtail to the pack. Basic soldering, through-hole only,
no PCBA.

**The servos daisy-chain**, which is what keeps it to two headers. Each X-series
servo has two identical connectors wired straight through internally, so VDD,
GND and DATA all pass to the next one. Both board connections then use **stock
Dynamixel cables**, so there is no crimping anywhere in the build — see the
sourcing note on why that matters.

The chain is also the electrically better topology: it is one continuous line
with taps, where a star would hang a stub off every spoke. The only cost is that
the first cable carries all three servos' current, and at stock cable length
that is ~0.12 V at a 3.6 A peak — nothing.

**The U2D2's VDD pin is connected**, matching every standard Power Hub Board
wiring. The U2D2 draws its logic power from USB; the VDD pin on its TTL port is
bus pass-through, and X-series buses routinely run at 12 V and above. Connecting
it is what lets the board→U2D2 link be an unmodified 3-pin cable.

**Wiring diagram: [`untethered-wiring.svg`](untethered-wiring.svg)** — every
component, both power domains, and the Pi's pin assignments on one page.

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
| Safety | inline main switch + fuse, cutting **servo** power independent of the Pi | 12 |
| Cabling | micro-USB OTG adapter, TM151 UART pigtail | 16 |
| | **total** | **~190** |

Already owned: U2D2, TM151, 3 servos. Gamepad is laptop-side — use whatever is
on hand.

*Substitutions and exact part numbers are in **Sourcing** below — notably a
Traco TSR 2-2450 in place of the Pololu buck, which Digi-Key Canada does not
appear to carry.*

**Not buying, and why:** U2D2 Power Hub (bulky, replaced by the perfboard);
OpenRB-150 (would force a C++ rewrite); RC transmitter/receiver (WiFi carries
a richer command set — see below); any custom PCB.

Against the <$100 stretch target: the core electronics is ~$75 (Pi + SD + buck
+ connectors + safety). The remaining ~$115 is packs and a charger — reusable
consumables, and free if a charger is already on hand.

## Sourcing — three orders, shipping to Canada

The table above is the design intent. This section is the orderable form of it,
with a few substitutions where the intended part is not available from a
Canadian-friendly supplier.

**It is written as a complete from-scratch build**, assuming nothing on hand
beyond the tethered rig listed above. Consumables like wire, heat shrink and
solder are included for that reason, as is a charger — skip whatever the bench
already has. The point is that this list still works for someone rebuilding the
bike somewhere else.

**Why three and not one.** The split is forced by the LiPos: they are UN3480
hazmat, ground-only, and Digi-Key does not sell hobby packs or balance chargers
at all. Buying them domestically also sidesteps the cross-border shipping
restriction entirely. The Pi is a second forced split — `SC0721` on digikey.ca
was out of stock at time of writing (0 units, with a nonsense Jun-2027 restock
date) and Digi-Key's listing is the headerless board regardless.

### Order 1 — [Digi-Key Canada](https://www.digikey.ca)

CAD pricing, DDP, so no brokerage surprise at the door.

| what | mfr PN | qty | purpose | link |
|---|---|---|---|---|
| 5 V regulator | Traco **TSR 2-2450** | 2 | 3S bus → 5 V for the Pi, into GPIO pins 2/4. 1 in use, 1 spare: its failure bricks the bike | [detail](https://www.digikey.ca/en/products/detail/traco-power/TSR-2-2450/9383726) |
| Buck input cap | Panasonic **EEU-FR1E471** | 2 | 470 µF at the regulator input. **Not** for regulator stability — the Traco needs no external caps — but to ride out pack sag from motor transients. Holds the Pi ~16 ms above the 6.5 V dropout | [detail](https://www.digikey.ca/en/products/detail/panasonic-industry/EEU-FR1E471/2433553) |
| Servo rail cap | Panasonic **EEU-FR1E102** | 2 | 1000 µF across the servo rail, damping the transient at its source rather than riding it out downstream. The pair is deliberate, not redundant: different nodes, different jobs | [search](https://www.digikey.ca/en/products/result?keywords=EEU-FR1E102) |
| Board header | JST **B3B-EH-A** | 4 | The splitter board's two VDD/GND/DATA taps — one to the servo chain, one to the U2D2. 2 in use; spares because one always dies in a desolder | [search](https://www.digikey.ca/en/products/result?keywords=B3B-EH-A) |
| Perfboard | 2.54 mm through-hole, ≥50×50 mm | 1 | Cut to ~25×25 mm; the Power Hub replacement | [search](https://www.digikey.ca/en/products/result?keywords=perfboard%20prototype%20board) |
| Main switch | SPST, DC-rated ≥10 A @ 12 VDC | 1 | Failsafe 4 — kills servo power independent of the Pi | [search](https://www.digikey.ca/en/products/result?keywords=toggle%20switch%20SPST%2012VDC) |
| Fuse holder + fuses | inline blade holder, **7.5 A** blade | 1 + 5 | Protects the 20 AWG trunk against a short or a jammed drivetrain | [search](https://www.digikey.ca/en/products/result?keywords=inline%20blade%20fuse%20holder) |
| Trunk wire | 20 AWG silicone, red/black | ~2 m | Pack → switch → fuse → splitter board | [search](https://www.digikey.ca/en/products/result?keywords=20%20AWG%20silicone%20hook%20up%20wire) |
| Pigtail wire | 22 AWG silicone, 3 colours | ~2 m | Servo drops and the U2D2 TTL pigtail — see the gauge note below | [search](https://www.digikey.ca/en/products/result?keywords=22%20AWG%20silicone%20hook%20up%20wire) |
| Heat shrink | assortment, 2–8 mm | 1 | Every joint in the list above | [search](https://www.digikey.ca/en/products/result?keywords=heat%20shrink%20tubing%20assortment) |
| Barrel jack pigtail | female **5.5 × 2.5 mm**, flying leads | 2 | Adapts the 12 V brick to an XT30/XT60. One makes the bench-power lead (below) — which is what lets bring-up start before the packs arrive; the second is the charger's DC input, if that route is taken | [search](https://www.digikey.ca/en/products/result?keywords=dc%20power%20jack%20pigtail%202.5mm) |
| GPIO jumper leads | female–female Dupont, 2.54 mm | 1 pk | Mates the TM151's 5-pin male header to the Pi's male GPIO pins, and lands the buck's 5 V on pins 2/4. See the TM151 wiring note below | [search](https://www.digikey.ca/en/products/result?keywords=jumper%20wire%20female%20to%20female%202.54mm) |

Optional, and only if the corresponding decision goes that way:

| what | mfr PN | qty | purpose | link |
|---|---|---|---|---|
| Schottky | **1N5822** (3 A, 40 V) | 3 | ~0.4 V drop ahead of the steer servo, *if* the XC330 ever misbehaves near full charge. That call is **decided as accept-it** — these are bin insurance so the fix needs no second order | [search](https://www.digikey.ca/en/products/result?keywords=1N5822) |

### What is deliberately *not* in that list

**JST crimp contacts and housings — the servos already ship with EH cables.**
An earlier draft had `EHR-3` housings and 50× `SEH-001T-P0.6` contacts for
building servo cables. That is work that does not need doing: every XC430 and
XC330 comes with a 3-pin JST-EH cable, and those plug straight into the
`B3B-EH-A` headers on the splitter board. Only two things would change that —
stock cables too short for the as-built routing, or wanting the U2D2 pigtail
terminated in a connector rather than soldered. **Neither is known yet, so do
not buy for them.** And if it does come up, note that `SEH-001T-P0.6` needs a
real JST crimp tool; hand-crimping these with generic pliers produces
intermittent joints on the one bus every servo shares. Buying a pre-made
Dynamixel extension cable beats crimping.

**No second buck for the AHRS.** The TM151 runs off the Pi's own 5 V/3.3 V pins
and its UART is 3.3 V TTL — no level shifter, no separate supply.

**No USB hub.** The port budget (above) exists precisely so there is nothing to
buy here.

### Notes on the choices above

**`SEH-001T-P0.6` is rated 22–26 AWG and will not crimp 20 AWG** — which is
part of why the wire plan splits by run: 20 AWG silicone for the pack→board
trunk, **22 AWG for the servo drops**. 22 AWG carries 1.4 A per servo with room
to spare. Check Digi-Key's silicone-jacket stock at both gauges before counting
on it; silicone wire is a staple at any hobby shop if the selection is thin.

**7.5 A fuse, not the 5 A in the table above.** Peak draw is ~4 A (see
*Budget*), and a 5 A fast-blow sits close enough to that to nuisance-trip on a
three-servo stall transient. 7.5 A still protects 20 AWG, whose chassis rating
is ~11 A.

**The switch must be DC-rated.** Many panel rockers are specified for AC only;
on a 12 V inductive DC load their contacts arc and can weld closed — which
defeats the entire point of a failsafe that cuts servo power independent of the
Pi. Filter on a DC current rating, not just amps.

**Feeding 5 V into GPIO pins 2/4 bypasses the Pi's input protection.** That is
the normal way to power a Zero from a buck and it is what frees the micro-USB
OTG port for the U2D2 — but it means the regulator's output is the only thing
between the pack and the SoC. It is a reason to fit the 470 µF and to bench the
rail before the Pi is ever connected to it.

**TM151 wiring.** From the TM151/TM171 datasheet V1.1.6 §3, pin numbers as
printed on the baseboard:

| TM151 | → | Pi | note |
|---|---|---|---|
| Pin 1 RXD | ← | pin 8 (TXD) | optional — nothing in the design transmits |
| Pin 2 TXD | → | **pin 10 (RXD)** | the crossover, and the only wire that carries data |
| Pin 3 VCC | ← | pin 2 or 4 (5V) | 4.5–5.5 V, 80 mA / 0.4 W typical |
| Pin 4 GND | | pin 6 or 9 (GND) | |
| Pin 5 GND | | — | internally linked to pin 4; one is enough |

Both it and the Pi's GPIO are 2.54 mm male pins, so female–female jumper leads
mate at both ends with nothing else to buy.

**No level shifter, and nothing to meter.** The datasheet specifies both UART
pins as *"running at TTL 3.3 V and is compatible with TTL 5.0 V"* — 3.3 V out,
5 V-tolerant in. So the TM151→Pi direction is 3.3 V into a 3.3 V GPIO, and the
Pi→TM151 direction is 3.3 V into a 5 V-tolerant input. Both are safe as wired,
by specification rather than by assumption.

**The two grounds are one net** — *"Pin 5 … is internally linked together with
Pin 4 and thus Pin 4 and Pin 5 play the same role."* They are not separate power
and signal grounds, so there is nothing to gain from routing them separately.
Connect one.

**Pin 1 (RXD) is optional.** The AHRS free-runs and pushes (see *Loop rate*),
`hw/ahrs.py` has no write path at all, and baud/rate/message config is done over
USB with ImuAssistant in *One-time device configuration*. Wiring it costs one
jumper and preserves the ability to reconfigure or recover the unit in place
rather than unmounting it for a USB trip; leaving it open costs nothing today.

**Bundle the jumpers rather than running loose leads.** Individual Dupont
connections back off under vibration, and this is a machine whose normal failure
mode is falling over. Crimp them into one 5-pin housing, or lock separate leads
with heat shrink and a dab of hot glue at each end.

**A reversed plug is survivable but not free.** VCC sits on the centre pin of
five, so it lands correctly either way and there is no reverse-polarity event —
and the board carries reverse voltage protection to −15 V regardless. What does
happen is that both TXD pins get shorted to ground: the TM151's on pin 2 and the
Pi's on pin 8 if it is wired. Both are current-limited outputs and normally
survive, but "probably got away with it" is a poor substitute for a keyed
housing.

### Order 2 — [PiShop.ca](https://www.pishop.ca) (Waterloo, ships domestic)

- [Raspberry Pi Zero 2 W with header](https://www.pishop.ca/product/raspberry-pi-zero-2-w-with-header/) (SC0721)
- 2× 32 GB A1 microSD — the second is the recovery path for the
  `/boot/firmware` UART edits in *Pi setup*, which are easy to get wrong once
- micro-USB **OTG** adapter for the U2D2 — must be OTG, not a charge-only cable

If Digi-Key restocks the Pi before ordering, the headerless `SC0721` is fine
and collapses this into order 1: only pins 6/8/10 (GND/TXD/RXD) are used, so
"WH" buys convenience on 37 pins that do nothing in this design.

### Order 3 — Canadian hobby shop

[EpicFPV](https://epicfpv.ca) (Calgary) and [DroneDynamics.ca](https://dronedynamics.ca)
both stock the charger; any domestic RC shop works.

- 2× **3S 1300–1500 mAh** LiPo
- **ISDT Q6 Nano** or **SkyRC B6neo** — but see the AC note below
- LiPo charging bag
- **XT30 and XT60 matched pairs, both** — see below
- JST-XH balance extension
- 20 AWG / 22 AWG silicone wire, if Digi-Key's selection is thin

### XT30 vs XT60 — buy both, decide when the packs arrive

Many 1300 mAh 3S packs ship XT60 rather than the XT30 assumed above. Both are
the same Amass design at different sizes; they do not intermate.

| | XT30 | XT60 |
|---|---|---|
| rated | 30 A | 60 A |
| peak draw here | 4 A | 4 A |
| pair mass | ~2 g | ~6 g |
| footprint | small | roughly double |

**Electrically the choice is meaningless** — XT30 is already ~7× overrated for
this bike. It comes down to the requirements line at the top of this document,
*size over weight*, which XT30 wins; against XT30's smaller, closer solder pads,
which are fussier to get clean than XT60's.

There is nothing to commit to now, because the bike-side connector is a pigtail
soldered to the splitter board — the last joint made, and five minutes to redo.
So buy pairs of **both** and solder whichever matches the packs. Prefer XT30 if
both are available.

**Buy matched pairs, not singles.** Male/female labelling on XT connectors is
genuinely inconsistent between vendors — listings disagree about which half
belongs on the battery, and the same physical part is called both. A pair costs
about what a single does and guarantees the mating half whatever the packs turn
up with.

**Do not use an XT30↔XT60 adapter**, though they are sold everywhere. It adds a
junction to the single path carrying all servo current, plus bulk and ~10 g
exactly where the pack should sit flush, and buys nothing that five minutes with
an iron does not.

### Bench power — the same bike, tethered

Nothing about the design requires the pack. Build a **female 5.5 × 2.5 mm barrel
socket → pack-side XT30/XT60** adapter and the 12 V brick impersonates a battery
at the bike's one power input. Same splitter board, same buck, same switch and
fuse, same code — the bike cannot tell the difference.

This is worth building first, because **it decouples order 3 from getting
started.** Verification steps 1 and 2 (loop timing and jitter at 1/2/3 Mbps;
AHRS on the GPIO UART) need no battery at all, so Digi-Key and PiShop can be
ordered now and bring-up can begin while the packs and charger are still
undecided.

It also happens to be electrically kind: 12.0 V is dead-on the XC330's ceiling,
so the over-voltage question does not arise on the bench at all.

Three things to respect:

- **Polarity, metered, before first plug-in.** Centre-positive to XT30-positive.
  Reversed, this feeds the servos and the buck backwards, and neither has
  reverse-polarity protection. Label the finished adapter on both ends.
- **5 A is a real ceiling here.** The peak budget is ~4 A, so wheels-clear
  bench work has margin but a three-servo stall does not. Expect the brick to
  fold back or trip rather than sag gracefully, which shows up as a Pi reboot
  mid-test. Do not read a brownout on the brick as a fault in the bike, and do
  not do stall or hard-acceleration testing on it.
- **The brick masks the LVC failsafe.** At 12 V the address-144 cutoff can
  never fire, so *Verification* step 5's low-voltage test still needs a real
  pack — or a variable bench supply, if one is worth buying later.

The bike having exactly one power input is what makes this safe: the pack and
the brick are physically exclusive, so there is no way to have both sources
fighting on the rail.

### Powering the charger from the existing Dynamixel brick

Both candidate chargers are **DC-input only** — no AC brick in the box. The
[ROBOTIS SMPS 12V 5A](https://en.robotis.com/shop_en/item.php?it_id=903-0126-000)
already on the bench is the obvious donor:

| | |
|---|---|
| brick output | 12 V 5 A (60 W), **5.5 mm OD × 2.5 mm ID barrel, centre-positive** |
| Q6 Nano input | DC 10–30 V, via **XT60** |
| B6neo input | DC 10–28 V via XT60, or USB-C PD 12–20 V |

So 12 V sits inside both windows, and 60 W is far more than the ~16 W a 1 C
charge of a 1300 mAh 3S needs. **The adapter is barrel-jack-female → XT60-male**
— either the Digi-Key pigtail listed above with an XT60 soldered on, or the
ready-made "DC 5.5×2.5 to XT60 charge lead" most hobby shops stock, which is
the easier buy.

Two things to get right:
- **2.5 mm ID, not 2.1 mm.** The 2.1 mm jack is far more common and the
  ROBOTIS plug will not seat in it.
- **Verify polarity with a meter before plugging in a charger.** Centre-positive
  is the ROBOTIS convention and is what the barrel marking says, but reverse
  polarity into a charger's DC input is an instant kill and the check costs
  ten seconds.

The alternative that removes this entirely is the **SkyRC B6ACneo** — same
charger with mains input built in (60 W AC / 200 W DC). It costs a little more
and needs no adapter, no barrel pigtail, and no polarity check.

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
`hw/ahrs.py`: 200 Hz needs 460800).

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

**The laptop runs the access point, not the Pi.** The requirement was only that
the link never depend on house networking, and a laptop-hosted AP satisfies that
just as well while being the better side to host it on: bigger antennas and more
transmit power on the ground station help *both* directions of a link whose weak
end is a chip antenna on a 65 mm board, and it keeps `hostapd`/`dnsmasq` off a
512 MB SBC that is also running the balance loop. AP mode on the CYW43438 works,
but it is the chip's weaker path and there is no reason to lean on it.

The Zero 2 W is then an ordinary client. Note that client reconnects after an AP
hiccup take seconds, not milliseconds — long enough for `CMD_DEAD_S` to fire and
drop torque. That is the correct behaviour, but it means a flaky ground-station
AP shows up as unexplained torque-offs rather than as sluggish steering.

### The radio, and what actually goes wrong with it

The Zero 2 W's wireless is a Cypress/Broadcom **CYW43438** on SDIO, driven by
`brcmfmac`: 2.4 GHz only, single chip antenna at one end of the board. It has a
reputation, and it is worth separating the parts of that reputation which matter
here from the parts which do not.

**Bandwidth is not one of the problems.** The command struct at 50 Hz is
kilobits against a chip that does tens of megabits, and the SDIO throughput
ceiling everyone cites is three orders of magnitude away from what this link
asks for. Latency and dropouts are the only axes that matter.

| concern | verdict here |
|---|---|
| **Power-save latency spikes** | **The real one.** Tens to hundreds of ms, default-on, lands inside `CMD_STALE_S = 0.15`. Fixed in *OS configuration* item d — and it must be asserted at startup, not assumed |
| **Supply dips during TX bursts** | **Real, and specific to this build.** See below |
| **Bluetooth coexistence** | Already solved. One radio time-shares WiFi and BT, and `dtoverlay=disable-bt` is already set to free the PL011 UART for the TM151 — a constraint that pays twice |
| **Firmware halts** | Version-dependent, not common, but indistinguishable from "laptop went away". Pin a known-good image; consider a link-health check that resets `wlan0` |
| **Slow client reconnect** | Seconds, longer than `CMD_DEAD_S`. Correct behaviour, confusing symptom (above) |
| **Regulatory domain unset** | Caps channels and TX power; reads as unexplained short range. Set it (item d) |
| **AP-mode instability** | Sidestepped by hosting the AP on the laptop |
| **SDIO throughput ceiling** | Irrelevant at this data rate |

**The supply interaction is the one this design creates for itself.** WiFi
transmit draws sharp current spikes, and the Zero 2 W is known to be sensitive
to 5 V rail dips. This build feeds 5 V into GPIO pins 2/4 — bypassing the Pi's
own input protection — from a buck on a pack that three servos are also pulling
transients from. That is two independent transient sources on one rail with only
the 470 µF between them and the SoC.

The bulk capacitance above is justified purely by motor current. It is also
carrying the radio, and nothing has measured whether it carries both at once.
Until it has, treat "random Pi reboots" as a power-integrity question before a
software one — see *Verification* step 1.

**Antenna placement is a mounting constraint, not just an electrical one.** The
chip antenna sits at one end of the board and a LiPo pack is effectively RF
opaque. Do not sandwich the Pi between the pack and the chassis. This is the one
WiFi consideration that has to be settled at build time rather than fixed in
config later.

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
  so 200 Hz needs ~146 kbps. **115200 will not keep up.** 230400 carries it
  arithmetically but at 63% sustained utilization with no flow control, and the
  datasheet (V1.1.6) recommends **460800 for 200 Hz** and 921600/1M for 400 Hz.
  Following the vendor; a dropped frame is a stale attitude in the balance loop
  and the higher rate costs nothing but a config field.
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
- ~~**XC330 over-voltage near full charge.**~~ **Decided: accept it.** A fresh
  3S at 12.6 V is 5% over the XC330's 12.0 V datasheet ceiling, for the first
  minutes of a pack's discharge only. Judged not worth a hardware mitigation.
  Two cheap outs remain if a servo ever misbehaves near full charge: set the
  charger to 4.10 V/cell, which tops out at 12.3 V for ~5% of capacity and no
  hardware at all; or fit one of the 1N5822s in the steer servo's VDD line for
  a ~0.4 V drop. The diodes are in the Digi-Key order as insurance and are
  expected to stay in the parts bin.
- **As-built payload mass and position** — the numbers above are estimates;
  weigh and measure at assembly, then re-run `python -m aow_sim.export_deploy`.
- **Servo IDs** — `hw/dynamixel.py` assumes drive_a=1, drive_b=2, steer=3. Set
  them with ROBOTIS Wizard before first run.

## Pi setup and deployment

**Never develop on the Pi.** It has no MuJoCo, so most of the test suite cannot
run there by design. The split:

| laptop | Pi |
|---|---|
| edit, train, `pytest`, `export_deploy` | run `hw.run_bike`, nothing else |
| MuJoCo + scipy + torch | numpy + pyyaml + pyserial + dynamixel-sdk |

### 1. Image the card headless — no monitor, no keyboard

Raspberry Pi Imager, **Raspberry Pi OS Lite (64-bit)** — Lite because there is
no display and the desktop only costs RAM. Before writing, open the gear /
"Edit settings" pane and set hostname (`aowbike`), your SSH **public key**
(not a password), WiFi SSID/password, and locale. These are baked into the
image, so first boot comes up on the network with SSH already listening:

```sh
ssh pi@aowbike.local
```

64-bit matters: `dynamixel-sdk` and numpy both ship aarch64 wheels, so nothing
has to compile on a 1 GHz A53.

### 2. OS configuration — three things, each a silent failure if skipped

**a. Free the GPIO UART.** Linux claims it for a serial console, which is
exactly the port the TM151 needs. In `/boot/firmware/config.txt`:

```
enable_uart=1
dtoverlay=disable-bt
```

`disable-bt` moves the capable PL011 UART to the GPIO pins (the Pi's
`/dev/serial0` is otherwise the weaker mini-UART, whose baud is tied to the
core clock and drifts). Then remove `console=serial0,115200` from
`/boot/firmware/cmdline.txt`, and:

```sh
sudo systemctl disable --now serial-getty@ttyAMA0.service
```

**b. `latency_timer=1` for the U2D2.** The `ftdi_sio` default is 16 ms, which
caps the control loop at ~30 Hz no matter the baud. As a udev rule so it
survives reboots and re-plugs:

```sh
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' | sudo tee /etc/udev/rules.d/99-u2d2-latency.rules
```

`assert_low_latency()` checks this at startup and names the fix rather than
letting the loop silently run slow.

**c. Let the control thread go real-time** without running the whole program
as root:

```sh
sudo setcap cap_sys_nice+ep $(readlink -f $(which python3))
```

Without it `_try_realtime()` warns and continues at normal priority.

**d. Turn WiFi power save off.** The `brcmfmac` driver enables WiFi power
management by default, and it produces latency spikes of tens to hundreds of
milliseconds. `run_bike.py` sets `CMD_STALE_S = 0.15`, so a power-save stall
lands squarely inside the command-age watchdog: the bike intermittently zeroes
its velocity command with no external cause. That is the failsafe working
correctly on a fault that does not exist, which is the worst kind to diagnose on
hardware.

```sh
sudo iw dev wlan0 set power_save off
```

Make it persistent — a `systemd` unit or a `NetworkManager` connection property,
depending on the image — and assert it at startup the same way `latency_timer`
is, rather than trusting it. This is the same species of bug as the FTDI latency
timer: a driver default that silently degrades a real-time path and gets blamed
on the wrong subsystem.

Also **set the regulatory domain** (`country=` in `wpa_supplicant.conf`, or via
Imager's locale settings). Without it some channels are unavailable and transmit
power is capped, which reads as unexplained short range.

### 3. One-time device configuration (done from the laptop, before mounting)

- **Dynamixels**, via DYNAMIXEL Wizard 2.0 over the U2D2: set IDs to 1/2/3
  (drive A, drive B, steer) and baud to **3 Mbps** on all three. Everything
  else — operating mode, Return Delay Time, indirect blocks — `ServoBus.open()`
  sets on every startup, so it is not part of this step.
- **TM151**, via ImuAssistant: output **460800 baud** (the datasheet's
  recommendation for 200 Hz ODR; 115200 cannot carry 200 Hz Combo frames at all
  — see `hw/ahrs.py`), output rate 200 Hz, and enable the `Ep_Combo` message.
  Also set **Auto boot mode**: the datasheet gives cold start as **3.2 s** on
  auto boot against **10–30 s on static boot, and static is the factory
  default**. That 30 s is long enough to look like a dead sensor during
  bring-up, and it delays every power-on once the systemd unit exists.

  This configuration is stored in the unit and survives power cycles — which is
  what lets the Pi's TX line stay optional. It is also why a factory reset means
  a USB trip, not a field fix.

### 4. Sync the code

`rsync`, not `git` — during bring-up you want the working tree, not commits:

```sh
rsync -av --delete --exclude .git --exclude runs --exclude traces --exclude '__pycache__' ./ pi@aowbike.local:~/aow-bike-sim/
```

### 5. Install — `--no-deps` is load-bearing

`pyproject.toml`'s base `dependencies` list mujoco and scipy for the simulator,
so a plain `pip install -e .` drags both onto the Pi and undoes the whole
point. On the bike:

```sh
python3 -m venv ~/venv && source ~/venv/bin/activate
pip install numpy pyyaml pyserial dynamixel-sdk
pip install --no-deps -e ~/aow-bike-sim
```

The runtime list is recorded as the `onboard` extra in `pyproject.toml` so it
lives in one place. Sanity check that the boundary held:

```sh
python3 -c "import aow_sim.hw.run_bike; import sys; assert 'mujoco' not in sys.modules; print('clean')"
```

### 6. Export the bundle (laptop) and ship it

```sh
python -m aow_sim.export_deploy          # -> deploy/bundle.npz
rsync -av deploy/ moves/ pi@aowbike.local:~/aow-bike-sim/
```

Re-run this whenever gains, `bike_params.yaml`, or a policy changes — the
bundle carries a digest of the params it was built from and `load_bundle`
refuses a mismatch rather than flying stale gains.

### 7. Run

```sh
python -m aow_sim.hw.run_bike --bundle deploy/bundle.npz
```

Bike **on a stand with the wheels clear** until the Verification steps below
pass. Add `--rate` to try other loop rates; watch `jitter_ms` and `dt_ms` in
the telemetry.

### 8. Once it works: autostart and a self-hosted network

A `systemd` unit (`Restart=on-failure`, `After=network.target`) makes the bike
come up balancing on power-on with no laptop. Do this **only after** the
failsafes are verified — an autostarting balance loop on a bench is a bike
that throws itself off the bench.

Run `hostapd`/`dnsmasq` **on the laptop** so the command link never depends on
house WiFi and the failsafe never fires because someone's router rebooted. See
*The laptop stays a ground station* for why the AP lives on that side.

**There is no RTC on a Zero 2 W, and in AP-island mode there is no NTP either.**
Nothing on the control path cares — every timing call in `hw/` is
`time.monotonic()`, `HardwareData` documents that its origin does not matter,
and the control `dt` comes from the servos' Realtime Tick rather than the Pi.
What suffers is log timestamps and file mtimes, which `fake-hwclock` leaves at
roughly-last-shutdown rather than at the epoch. Since the ground station is
already on the other end of the link, make it the time source — point `chrony`
at the laptop, or skip syncing entirely and let laptop-side telemetry carry the
wall clock while bike-side logs carry monotonic ticks. A DS3231 on I²C solves it
in hardware for $2, but it is a part and some pins for a problem the existing
link already solves.

## Verification

Bench-first. The bike should not attempt to balance untethered until 1–3 pass.

1. **Loop timing, no bike.** Pi + U2D2 + 3 servos on the pack. Measure
   SyncRead+BulkWrite round trip and tick jitter over 60 s at 1/2/3 Mbps, and
   with `latency_timer` at 16 vs 1 (to see the failure mode).
   **Gate: p99 tick jitter < 1 ms at 100 Hz.**

   Same session, **scope the 5 V rail** under the three loads together — WiFi
   transmitting, servos moving, control loop running. The bulk caps were sized
   against motor transients alone and the radio was never in that budget, and
   feeding the Pi through GPIO leaves no input protection between the buck and
   the SoC. **Gate: no dip below 4.75 V.** Do this before blaming any reboot on
   software.
2. **AHRS.** TM151 on GPIO UART at 200 Hz; check the quaternion against known
   hand-held orientations; measure age-of-data at the control tick.
2b. **Link latency, before trusting the watchdog.** With power save off and the
   laptop hosting the AP, log command-age over several minutes of normal
   driving range. **Set `CMD_STALE_S` from the measured p99 rather than from the
   round 150 ms it currently holds** — that number is a guess, and if the real
   distribution has a tail past it the bike will zero its command for no visible
   reason. Repeat once with power save deliberately left on, to learn what the
   failure looks like.
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
