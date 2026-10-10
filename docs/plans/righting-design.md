# layout and CAD for righting module

> **Status:** new document (2026-09-29). Designed, not yet printed. The spec
> below was written after the fact, from the design conversation, in the
> format of `drive-design.md` and `steering-design.md`. Supersedes the
> machined concept in `wing-linkage-metal.md` (too many machined parts). The
> linkage choice itself (`righting-linkage-margin.md` step 5) is still open;
> this module draws the diamond.

3D printed design for the self-righting mechanism: the diamond swing linkage
(one crank pin, one wing rod), its XC330 and the two wings, as a module that
can later be incorporated into an overall model of the bike. Everything is
printed in ASA except the rods, which are metal stock.

## linkage geometry (imported)

- The four-bar comes from a **linkage config**, the same files the sim and
  `analysis/swing_stepthrough.py` read: `linkage.config` in
  `config/righting_cad.yaml` (now `config/swing_linkage_shared_rod.yaml`).
  - From it: crank, coupler and rocker lengths, the servo shaft height, the
    stroke, and the panel's line at rest.
  - The panel's standoff and angle off the rocker are re-derived onto that
    same line, as `swing_synthesis` does.
- **Scaled about the rod** by `linkage.scale` (1.25). All four lengths scale
  together, so every angle, and so the servo current, is unchanged. It buys
  wall thickness and 1/scale of the pin forces.
- **Diamond-type only.** A config with two crank arms or two hinges
  (`angle_between_cranks` or `wing_pivot_x` not 0) is refused. The stack, the
  bearings and the L/R symmetry all depend on one pin and one rod, so a
  staggered linkage would be a different module, not different numbers.
- **What a new linkage config changes by itself:**
  - the crank, couplers, rocker arms, ears and panel positions;
  - the crank axis height, and so the servo, the cases and the bridge height.

  What it does not change: the axial stack, the bosses, the joints, the
  frame's outline.
- **Re-run `--check` after any linkage change.** Several clearances were set
  for this linkage and only hold for it:
  - the crank web vs the rocker arm sharing its plane (4.5 mm here);
  - the rear knuckle under the servo case;
  - the panel tab inside the couplers' radius.

## parts

Orientation and printing directions: Y-forward (along the rod and the crank),
X-right, Z-up; origin on the wing rod at the mid-plane between the couplers.
The XC330 sits at the rear, horn forward. Each L part is its R twin turned
180° about Z unless noted.

- **Rods,** metal stock, one diameter (`rod.dia`, 6 mm or 1/4 in; check
  material and reamers): the wing rod, the crankpin, two rocker pins. Every
  hole they run or press in is drawn at that size, to ream.
- **Half crank R / L,** printed web face down: web, shoulder, Φ14 journal.
  Only R has lugs (the steering headset upper's four, inside Φ14), into the
  horn hub. So these are two parts.
- **Couplers,** printed bossed face up. One part, both sides.
- **No washers** (since 2026-10-07, `stack.thrust: loop`). `washer`
  brings a loose one back on the crankpin and one on the rod, printed two
  layers or bought and measured.
- **Rocker R / L,** printed inner face up: hub on the rod (r 5.5), arm to
  its coupler, ear straight from the coupler joint to the panel boss.
- **Knuckle R / L,** printed inner face up: the rocker's outline, in the
  OTHER wing's coupler layer, the rocker pin through rocker, coupler and
  knuckle (since 2026-10-07; it was dog-legged, beyond the far bearing). A
  mirror part of the rocker: the same outline and volume, with its ring at
  the pin, where the rocker's is at the rod.
- **Wing R / L,** printed outer face down: since 2026-10-07 a STUB (to 30
  up the panel) whose two Y ends are tongues chamfered 45 deg on both faces,
  and a tab at each boss. One 6-32 along Y into each, the head in the tab,
  the nut in the rocker or knuckle.
- **Wing blade R / L:** DUMMY, the wing proper: an inverted U sliding down
  over the stub, its legs on the tongues. The panel's own thickness.
- **Front bulkhead,** printed inner face up: the front journal bearing, the
  rod.
- **Lower case,** printed inner face down: the rear journal bearing and the
  rod in its front wall, the hub's cavity, the XC330's horn half. It is the
  steering's lower case.
- **Upper case,** printed cap down: the XC330's back half, the steering's
  cable window and "H" cap. Since 2026-10-07 the servo is turned long end
  down, and the case is held by one 6-32 forward into the lower case, from a
  lug on its top (it had ears, then one central screw, up to the bridge).
- **Horn hub:** the steering's, same numbers (`steering_cad.yaml`), so the
  same part.
- **Bridge,** printed top up: ties the front bulkhead and the lower case,
  one 6-32 each with a ridge, and stops at the lower case's rear face (it
  reached back over the upper case). 2 over the crank. Its top face carries
  the chassis attachment (one joint now).
- **Chassis plate:** PLACEHOLDER.

## integration into whole bike

- In its own feature studio and part studio, `aow-bike-righting` inside
  `aow-bike`.
- Eventually inserted into a larger part studio for the whole bike. The
  bridge's top face is the mount: two 6-32 nut slots with ridges, at Y 0 and
  −50, into a chassis that is still TBD.
- The rod is 41.2 above the floor, the crank axis 25.1 above that, and the
  bridge top 102.8 off the floor. Packing under the drive was only a rough 2D
  check at 1.0×; see Outstanding.

---

## Decisions (2026-09-29, four rounds)

- **Printed, with one rod size,** from the machined concept's load path:
  - The crank is a supported crankshaft, so pin forces go into two printed
    bearings.
  - The horn hub takes torque only, through the lugs' clearance, so **no
    Oldham** (user): misalignment was never the worry.
  - Wing forces go rod → both bearings.
- **Wing joint along Y,** the same way as every other hole (user):
  - The head is countersunk into the boss from its far face, so the
    countersink faces up as printed.
  - The nut slot is in the panel's tab: the nut goes in first and is trapped
    once the panel is on.
  - The nut in the boss would put its slot (through both ways, by the shared
    helper) across the ear or the arm.
  - **No ridge:** the panel bears on the boss's face and the tab on its end.
    A ridge along the panel's oblique direction also would not extrude.
- **Rocker ≠ knuckle, only by the arm.** An arm on the rear knuckle would
  reach Z 18.3 inside the lower case's width at the rise, 5 mm into it. The
  rocker's ear runs straight; the knuckle's dog-leg (knee at −10°, 20 out)
  keeps the rear one under Z 6.5 against the case's 13.2. Both ears end at
  the boss's centre (their round end stood out as a bump before).
- **Thrust bosses (user's idea):** a ring 0.4 high (two layers) at each
  running pair, so parts bear on a ring at the hole, not face to face. **A
  boss can only be on a face that is up as its part prints,** which decides
  whose it is:

  | running pair | boss on |
  |---|---|
  | crank web or rocker arm ↔ coupler | the coupler, both eyes, one face; coupler L is coupler R flipped |
  | coupler ↔ coupler | neither (both faces are beds): the **centre washer** |
  | crank shoulder ↔ bearing | the shoulder |
  | knuckle ↔ bearing | the knuckle |
  | rocker L ↔ front bulkhead | the bulkhead |
  | rocker R ↔ lower case | none possible: clearance only, the hub within r 9 of the rod |

  Each bossed gap opens to boss + 0.2 clearance, 0.6.
- **The centre washer is necessary (user) and four numbers:** `washer`
  thickness, OD, ID (null = the rod's) and clearance. Swap in an off-the-shelf
  nylon washer by measuring it; the whole stack follows its thickness. 0
  removes it. A bore under the rod is refused. `kind: bought` leaves it out of
  the print check.
- **Both bearings' bridge joints: nut slot along Y, blind** (user), out
  through the face each part prints on, so each is a pocket open at the
  bottom.
- **The upper case reaches the bridge with two side ears,** flush with its
  cap. A pad behind the cap made the cap print as an overhang.
- **Frame:** the bridge ties everything with 6-32 × 3/8 and ridges, as the
  steering's cases tie to its mount plate.

## As built

`python -m aow_sim.cad_righting` (config `config/righting_cad.yaml`), one
feature `AOW righting` (dialog: pose rest / ±25/50/75/100 %, mocks on/off) in
the Feature Studio **aow-bike-righting features**, inserted in the Part
Studio **aow-bike-righting** (`onshape.yaml` tabs `righting_features` /
`righting`). Render: `docs/cad/righting.png`. Tests:
`pytest tests/test_cad_righting.py` (10, pure, no API).

| along Y from the mid-plane, each side | mm |
|---|---|
| centre washer (0.4) | 0–0.2 |
| couplers (bosses to 6.7) | 0.3–6.3 |
| web + rocker arm (and the wing's rocker-side tab) | 6.9–12.9 |
| hub zone: rocker hub, ear, boss; crank shoulder (ring to 21.3) | 12.9–20.9 |
| bearing: front bulkhead / lower case front wall | 21.5–31.5 |
| knuckle (ring to 31.7; wing tab to 50.1) | 32.1–44.1 |
| rear: journal end 32.0, hub 32.5–37.7, horn face 37.7, cap to 67.7 | |

| `--check`, one call | result |
|---|---|
| bodies; plain parts' bounding boxes as computed | 24; all 13 |
| L/R twins the same solid | 5 of 5 |
| interference at rest and at ±25/50/56/75/100 % | 0 |
| pins where the kinematics solver puts them | worst 1e-10 mm |
| flat-crowned holes / hanging edges as printed | 0 / 0 |
| downward faces left | the nut-slot roofs (blind-end 3 mm, through-slot up to 6.5, bridging layers where the helper adds them), groove roofs, the case-pin relief slivers the steering cases have too |
| mass, solid | ASA 297 g (the placeholder panels are 130 of it), steel 35 g |

## Tightened for the whole bike (2026-09-30, user)

In `aow-bike-whole` the module clashed with the drive (`bike-assembly-design.md`).
Changes, each a config switch so the first design stays one edit away:

- **knuckle R behind knuckle L** (`wing.knuckle_r: rear`). Both wings'
  second supports are at the rear, under the servo. knuckle R bears on knuckle
  L's back face over its own ring. The rod stops at the front bulkhead, so
  the module ends at the front bearing (the journal, 32.0). Wing R keeps its
  length, shifted back 12.6 to meet its tab. knuckle R and wing R become
  MIRRORS of their L partners, not the same parts (the check compares their
  volumes).
- **The upper case on one central 6-32** (`cases.uc_joint: central`), not
  the two ears.
  - Its top grows 2 mm (`cases.top_extra`): the nut slot otherwise cut
    1 mm into the servo. Exactly 1.0 mm of wall is left.
  - The screw sits behind the lower case (y -56.0): its slot cannot sit
    over it.
- **Bridge +-16, ending at -61.2** (was +-23, -67.7). The bulkheads' top
  band and the chassis plate placeholder are also +-16. All of them sit
  inside the drive pulleys' inner faces at |x| 17.
- **Chassis joints at y 0 and -30, ridges 6** (were 0 and -50, 10). In the
  bike the placeholder is the deck, and the drive block's wedge stands on it
  over -55..-38.

`--check`: 24 bodies, 0 interference at rest and over 10 poses, the
mirrored pairs equal in volume. ASA 269.5 g solid (was 297.3). Tests 11.

## Every moving link one 6 mm plate (2026-10-05, user)

"Rockers, half-crank, and knuckles should be able to fit in ~6mm each
only the y-axis (similar to the couplers)" (user). This is the outboard wing
joint from Outstanding, `wing.joint: outboard` (`hub_zone` is the first
design):

- **The rocker is one plate in the web plane:** hub, arm, ear and the
  wing's boss. The wing's rocker-side tab moves past it in Y, beside the
  bulkhead (the wing is outboard of the frame). The 6-32 goes in from the
  coupler side: head 4.6 deep in the rocker, nut in the tab.
- **The hub zone is gone.** The crank's shoulder is its web, with the
  thrust ring on the bulkhead side.
- **Knuckles 6** (`stack.knuckle`, was 12): the head 4.6 deep, a 1.4 pocket.
- **Chassis joints at y +8 / -8** (were 0 / -30): at 150 in the bike, -30
  lands under the drive block's wedge too. +-8 keeps 10.5 to the bridge's
  own screws at +-18.5.

| along Y from the mid-plane, each side | mm (was) |
|---|---|
| couplers | 0.3–6.3 |
| crank web / rocker plate | 6.9–12.9 |
| wing's rocker-side tab, outboard of the bulkhead | 12.9–18.9 |
| bearing: front bulkhead / lower case front wall | 13.5–23.5 (21.5–31.5) |
| knuckle L / knuckle R behind it | 24.1–30.1 / 30.7–36.7 (32.1–44.1 / 44.7–56.7) |
| rear: journal end 24.0, horn face 29.7 | (32.0, 37.7) |

So the module is 8 shorter at the front and the servo 8 further forward;
the knuckles save another 12 behind it.

**Checked before Onshape, by a planar sweep** of the parts' own primitives:
XZ outlines per Y slab, holes subtracted, every pair of groups, 110 poses.
The script is scratch (shapely is not a dependency here). The crank web and
the rocker's arm, ear and boss share a plane and come no closer than 4.47.
That matches the 4.5 the old arm had, which is how the sweep was trusted.
The wing tabs stay over 8 from the frame.

`--check`: 24 bodies, 0 interference at rest and over 10 poses, pins exact,
0 flat crowns / hanging edges. Tests 12 (one new: every moving link is one
layer thick).

**Mass as printed, ~105 g ASA** (2 perimeters, 15 % infill: skin 0.9 over
the area plus 15 % of the rest, UNCALIBRATED). The two wings, placeholder
panels, are ~52 of it. Nothing prints solid (user); solid it would be 222
(270 before). The estimate came from the parts' outlines, whose solid total
matches Onshape's less the upper case. From the next `--check` the judge
prints the printed figure itself: the body rows carry the area now, and
`printed_mass` moved to `cad_servo_mount` for both generators.

## Rear chamfers and shorter wings (2026-10-05 pm, user)

For the whole bike (`bike-assembly-design.md`, sixth pass):

- **`cases.rear_chamfer` 7:** 45 deg on the upper case's -Y +Z edge, the
  corner the drive's case sides come down onto ("mostly dead space
  supporting the attachment to the bridge", user).
  - 7 is the most with the joint behind the lower case: 8 meets the
    joint's ridge.
  - NOT CHECKED: the cap's connector access, by eye.
- **`frame.bridge_chamfer` [7, 0]:** the bridge's rear-top edge, ONE
  full-width chamfer at the most that keeps the central screw's head seat
  whole: 4.6, through the counterbore and 0.3 over the seat (user: "the
  chamfer can intersect the counterbore"). `[7, 5]` would put a 7 step
  outboard of |x| 5, which buys 1.75 in the bike under the drive case side's
  skirt.
- **`frame.bridge_half` 14.75** (was 16), and **`frame.bridge_rear`
  [10, 11.5]**: its last 10 mm +-11.5, to pass under the drive case sides'
  skirts (|x| 12) in the bike. The central joint's ridge shortens to fit
  (10.35, was 11.05).
- **The central screw's slot turned along Y** (`uc_nut_slot: rear`,
  user), out through the case's rear face, its print bed. The case's
  chamfer crosses the slot's open end: the check's one hanging edge, a 6.55
  bridge, accepted. `uc_joint_dy` moves the joint forward with a relief in
  the lower case's top, but stays 0: in 3D the lower case's undercut blocks
  the nut and the screw below it (user).
- **`wing.knuckle_tab: inner`** (user): each wing's knuckle tab on the
  knuckle's +Y side, beside the lower case, the screw's head in the
  knuckle 4.6 from its far face. (`outer` = the first design.)
- **`wing.panel_thickness` 5** (was 6): the synthesis's own
  `wing_width_mm`. It holds the panel's LINE 2.5 outside its keep-out
  (`swing_linkage._keepouts`; the only place it uses a thickness). At 6,
  centred, the inner face sat 0.5 inside the 35 core, and that was the
  whole of wing L's clash with drive pulley L in the bike.
- **`wing.panel_back` 50 -> 41:** the wings end 9 shorter at the front (77
  long), clear of the front tyre's steer sweep at every pose with the axle
  80 ahead of the rod. They are still mirrors.
- **The servo stays at the rear,** better for its cables (user).
- **What binds the module's rear in the bike now:** the bridge's middle
  over the central screw, against the drive's XC430 A.

`--check`: 24 bodies, 0 interference over 10 poses, mirrors equal, 0 flat
crowns, 1 hanging edge (the slot bridge above). ~103 g printed
(uncalibrated; Onshape's areas now).

## Knuckles inside, nuts in the links (2026-10-07, user)

"The 'knuckles' can move inside the bulkheads ... so that they straddle
the coupler (basically same part as the rockers). The wing hinge pin can
then be shorter." And: the nuts belong in the knuckles and rockers, with
the screws from opposite sides; with both from one side, assembly is
close to impossible.

| switch | value | what |
|---|---|---|
| `wing.knuckle_at` | `coupler` (was the outside design: knuckle L behind the lower case, R behind it) | each knuckle in the OTHER wing's coupler layer, on its own wing's side: rocker and knuckle straddle the wing's coupler |
| `wing.knuckle_hub_wall` | 2.5 (r 5.5, was 6.5) | the other wing's coupler sweeps over the rod 7.6 from its axis at the full stroke |
| `wing.panel_rear` | 36 | wing L's rear end = wing R's front end (the wings are twins again) |
| `wing.nut_in` | `link` (was `wing`) | head in the wing's tab from outside, nut in the rocker / knuckle, slid in from the boss's top end (blind slot, 2 deep, 1.0 wall) |

**Why the coupler layer, not a layer of their own.** A knuckle layer
between web and bulkhead would push each bulkhead out 6.6. That is +13.2 on
the module and on the wheelbase, since both ends are fitted (0.5 to the
drive, 5 to the front wheel). In the coupler layer the stack does not grow.
The bulkheads stay at 13.5-23.5, and the rod runs bulkhead to bulkhead,
47 long (was 61).

**What sits where now** (the washers and these rings were replaced the
same day; see "The servo turned" below). Each coupler layer holds one
coupler and the other wing's knuckle. The two knuckles meet at the
mid-plane over a second loose washer on the rod, as the couplers do on the crankpin: both faces are
print beds. Each knuckle's outer face carries a ring at the rod against the
other side's rocker hub. Every L part is its R turned again, wings and
knuckles included.

**Clearances, planar sweep over the stroke** (81 poses, holes ignored;
scratch, shapely):

| pair | closest | where |
|---|---|---|
| knuckle x the other wing's coupler | 2.19 (1.19 at hub wall 3.5) | coupler body over the knuckle's hub, full stroke |
| crank web x rocker (unchanged) | 4.47 | |
| coupler x the wing's tab | 6.57 | |
| everything else new | > 8 | |

The knee's place (-10..-40 deg, r 16-24) makes no difference to the
first row; the hub radius sets it. `test_the_knuckles_sit_beside_the_other_wings_coupler`
holds 2.0 (it fails at 1.19).

`--check`: 25 bodies, 0 interference at rest and over 10 poses, pins
exact, every L part equals its R, ~102 g printed. The rocker's and
knuckle's 8 downward faces are the nut slot's ~8 roof and the hole's
sacrificial layers, the same as the bearings' slots. The one hanging edge is
the old one, accepted.
`cad_bike --fit righting`: FITS, unchanged at 0.50 rear and 5.25 front;
wings L 2.6 / R 7.6 spare. Bike `--check` clean, 58 bodies.

## Flipping the XC330 (measured, not drawn: 2026-10-07)

The user's idea: turn the servo 180 deg about Y, its long end down, to
shorten the wheelbase; the bridge would need a new shape. What the rear
binds on now is the bridge's tail, and behind the bridge the upper case,
all at the top-rear corner, under the drive's case sides. Two throwaway
`--fit 2` scans at the bike's 143.5, going back 1 mm a step
(`traces/bike_cad/fit2_servo_*.py`, the outputs beside them):

| righting as | goes back to | then binds |
|---|---|---|
| drawn | 143.5 | the bridge's middle chamfer on XC430 A |
| upper case, servo and horn removed; bridge cut at the lower case's rear face (y -23.5) | <= 131.5 (the scan's end) | nothing new; wing L 1.0 to pulley L, which is sideways and stays 1.0 along Y |
| the same, but the upper case, servo and horn TURNED 180 deg about the horn axis | **131.5** (0.77; 0.06 at 130.5) | the turned upper case's former top corner (its central screw's pad), now at the bottom, on drive case side R |

So the flip is worth ~12 off the wheelbase (223.5 -> ~211.5), if the bridge
stops at the lower case. (A note here first said the connectors would then
face the floor. They do not: the cap faces -Y, and a turn about Y keeps it
there; user.)

## The servo turned, the knuckle the rocker's shape, no washers (2026-10-07, user)

User: the knuckle can take the rocker's shape (its dog-leg was for the
servo cases, no longer near it), the rocker pin through both; choose the
thrust faces to drop the washers, using the pin's new third part; and draw
the turned servo "with no attachment to the bridge so I can see the best
candidate installation"; the lower case "is basically just sliding down
relative to the servo horn".

| switch | value | what |
|---|---|---|
| `wing.knuckle_shape` | `rocker` | the knuckle is the rocker's outline; both on `knuckle_hub_wall` 2.5. Pin pressed in rocker and knuckle, the coupler running between |
| `stack.thrust` | `loop` | rings below; rocker and knuckle print inner face up |
| `washer.thickness` | 0 | none; `stack.gap` 0.6 (the mid gap, = boss + clearance) |
| `cases.servo_end` | `down` | XC330 turned 180 deg about Y with both cases; the bridge stops at the lower case; no `jU`, no rear chamfers, no bridge chamfer or narrow tail |

**Choosing the thrust faces.** A printed ring can only stand on a face that
is up as its part prints. Enumerating the print faces of rocker, knuckle,
coupler and front bulkhead (the crank web and the lower case are fixed by
other needs) gives exactly one washer-free family, with no group of parts
able to slide along Y. Rocker and knuckle print inner face up and the
couplers outer face up:

| ring | on | bears on |
|---|---|---|
| crank shoulders (as before) | each half crank's outer face, at the journal | the bearings |
| coupler (as before) | outer face, at the crankpin and the rocker pin | crank web; its own rocker |
| rocker (new) | inner face, at the rod | the OTHER wing's knuckle |
| knuckle (new) | inner face, at its rocker pin | its OWN coupler, across the mid-plane |

So the loop is crank -> coupler -> its wing -> the other wing -> the other
coupler -> crank, and the crank sits between the bearings. Nothing bears at
the mid-plane at the crankpin or the rod, nor on the front bulkhead (its
ring is gone); each coupler is held between its own rocker and knuckle.
Every other running pair is a plain 0.6. The chain is long, so a part's
axial play adds up across up to five 0.2 clearances: OUTSTANDING, check on
the print whether a plain 0.6 face ever touches.
`test_the_thrust_rings_locate_every_moving_group` reads the rings off the
layout, finds what each bears on, and checks no set of groups slides (it
fails on the first scheme without washers).

**Sweep:** the rocker-shaped knuckle keeps 2.19 to the other coupler (the
hub sets it, not the arm); the rocker's 4.47 to the crank web is now 4.90
on the thinner hub.

**The turned servo.** The lower case's horn-half shell comes down round
the horn with it. Its bottom is now behind the rod, so the rod's hole stops
1 into it (blind; the rod cannot walk out the back). The bridge keeps its
height, so the bike's deck does not move; the crank alone would let it come
down to ~47.6 above the rod (now 55.6), which is the lowering lever.

| check | result |
|---|---|
| righting `--check` | 23 bodies, 0 interference over 10 poses, pins exact, L = R turned, rocker and knuckle the same volume; ~100 g printed; 0 hanging edges |
| `cad_bike --fit righting` | FITS at 143.5 (1.04 rear, 5.25 front); room to **131.5 (+12.0)**, then the turned upper case's rear-top edge (\|x\| 12, ~39 above the rod) on drive case side R (0.06 at 130.5) |
| bike `--check` | clean, 56 bodies |

`--fit righting` now scans back in 1 mm steps (`fit2_probe(step, nmax)`):
55 steps of 0.25 timed out with this much room (a free 500).

The placement stays at 143.5 until the upper case has its attachment;
moving it re-fits the electronics (`--fit e`) and the chassis joints.

## Knuckle R's ring, a lower bridge, smooth bulkheads (2026-10-07 evening, user)

| switch | now | why |
|---|---|---|
| knuckle R's ring at the rod (inner face) | knuckle R only, so L and R are now two parts | closes the rod's mid-plane gap (user) |
| `washer.thickness` | 0.4 again, on the crankpin only | the couplers' shared faces are both beds; "will need a little washer in there" (user) |
| `frame.crank_clear` | 2: the bridge's underside 48.58, was 55.64 | the crank and couplers are highest at rest; nothing of the servo's is above them now |
| `frame.bulkhead_shape` | `hull`: one outline round the rod boss, the journal boss and +-12.35 (the lower case's width) from the journal up | was bands to +-16 (user) |
| jR, the lower case's bridge nut | through along Y, the nut in from the rear face | user; it passes 6.6 over the turned shell |
| `chassis.joints_y`, `ridge_half` | one joint at +9.5, ridge 4 | the bike at 120.5 puts the drive block's wedge over module y < +1.6 |

The thrust test now expects the knuckle-knuckle contact at the rod. It
still finds no group of parts that can slide.

In the bike (`bike-assembly-design.md`), with the drive's rear blocks
chamfered, the righting went back to **120.5 (wheelbase 200.5)**. There
the turned upper case passes between the drive case sides, 0.597 to case
side L's skirt. Next binds: wing L at the +0.56 pose on drive pulley L,
0.42 at 119.5.

`--check`: 24 bodies, 0 interference over 10 poses, pins exact, ~96 g
printed, 0 hanging edges.

## The upper case held, two chassis joints, the wing as a stub and blade (2026-10-07 late, user)

**The upper case** (`cases.uc_attach: lower`). One 6-32 along +Y on the
centreline over the turned servo:
- **head:** in a lug on the upper case's top, its back face at 45 deg, so it
  prints cap-down without hanging;
- **screw axis:** 42.59 above the rod, just high enough for the head's
  pocket to clear the case's top (38.64);
- **nut:** in a block the lower case grows back from its bulkhead, between
  the upper case's front (-37.4, the joint plane) and the bulkhead
  (-23.5). Its slot runs UP out of the block's top, so the nut goes in from
  above (user; it first ran -Z into the servo pocket);
- **jR:** the block closes jR's rear face, so the lower case's bridge nut
  slot is blind again.

The block's top is 4.4 from XC430 A, 1 mm ahead of the station.
`--fit righting` still FITS at 120.5 (wheelbase unchanged, as the user
expected).

**Two chassis joints** again (+-8, ridge 6; user: the chassis is a
placeholder). In the bike the -8 joint is under the drive block's wedge.
`cad_bike` now NOTES this rather than refusing, and the wedge stands on the
deck's top, so the plate keeps the grooves.

**The wing** (`wing.blade: ends`; user: "the wing chamfer can go on two
sides like roughly Z-aligned ... the wing fits like an inverted U overtop
... `<<     >>`"):

| piece | what |
|---|---|
| stub (printed) | a 6 border round the tabs and bosses (`stub_border`; user: "minimize how much I'm printing to test the mechanism"): 32 up the panel, Y -24.9..18.3 on wing R (was the whole span less 2.5 a side); each Y end a tongue, both faces chamfered 2.25 at 45 deg to a 0.5 flat, the edges along the panel |
| blade (dummy) | the panel's own outline: body over the stub's top, two legs at the stub's ends that hold the tongues (0.2 clearance). Its body spans the panel, +-41 on both wings (82 long, `panel_rear` = `panel_back`), so it overhangs its stub unequally (16 behind, 23 in front on wing R) |

What holds the blade:
- the stub's top takes it pushed down;
- the two legs' chamfers hold it in n both ways;
- the legs hold it in Y;
- it slides on DOWN the panel, and nothing holds it up yet (OPEN).

Nothing grows, so the full-stroke floor clearance stays the panel's 1.36.
The chamfers face 45 deg down as the stub prints outer face down: 0
downward faces.

REJECTED, the same evening:
- a one-sided dovetail sliding along Y, with a skin outward. The wing lies
  flat 1.36 off the floor at full stroke, so the skin had to stay under
  that (1.6 went 0.44 in; 1.0 left 0.16).
- tapering that skin at the tip: the wing lies flat, so it bought nothing.

**An oblique sketch plane would not extrude.** The chamfers run along the
panel, so the tongues are a new primitive (`poly`): an outline extruded
along any direction. Sketched straight on its oblique plane it failed
(EXTRUDE_FAILED, two billed evals: an eval that returns an error still
bills). It is now sketched on a world plane, as the prisms are, and moved
into place with normalised axes. Every earlier sketch here had an
axis-aligned normal.

`test_the_wing_is_a_stub_and_the_blade_an_inverted_u_over_it` pins the
faces, the spans and the 45 deg. `--check`: 26 bodies, 0 interference over
10 poses, 0 hanging edges, ~66 g printed (the blades not counted; a stub is
8.7 cm^3 solid, was 12.8 before the trim).

The blades sat 5 apart in Y (user's catch). That was an artefact: wing L had
been kept at -36..41 when the wings became twins, and wing R is its turn.
Both are now +-41 (user). `--fit righting` FITS at 120.5, unchanged. The
closest wing point is wing L's knuckle tab (module y -12.3), sideways to
drive pulley L at the +0.56 pose, 0.42 at 119.5; the panel's ends come near
nothing. With the stub trimmed, the wings' front room grew: L 16.2, R 22.8
spare (the panel's lower front corner had set it).


## Fitting the whole bike: the constraints to keep (2026-10-05)

The layout is settled for now and nothing is printed (user). The righting
will still be reworked before it prints, "without messing up the whole bike
fit". The bike rebuilds this module from its generator every time, so an
edit here reaches `aow-bike-whole` by itself. What it must keep:

| rule | now | set by |
|---|---|---|
| 0.5 to the drive, at rest AND every pose (`placement.rear_clear`) | 0.597 at the bike's 120.5 (wheelbase 200.5) | the turned upper case between the case sides (flat along Y); next, wing L on drive pulley L, 0.42 at 119.5. Before the turn: the bridge's middle chamfer on XC430 A at 143.5 |
| the righting's chassis joints vs the drive block's wedge | +-8: -8 is under it, head buried (noted; the chassis is a placeholder) | the wedge stands on the deck's top, so the ridges clear |
| the wing off the floor at the full stroke (it lies flat) | 1.36 | the panel's thickness; the blade adds nothing outward |
| the module less its wings 5 from the straight front wheel (`front_clear`) | 5.25 | the front bulkhead (y 23.5) |
| the wings clear the front tyre's whole steer sweep, every pose, 2 spare (`wing_margin`) | L 16.2 / R 22.8 spare | `wing.panel_back` = `panel_rear` 41; the blade's body starts 32 up the panel |
| the bridge's last 10 under the drive case sides' skirts (\|x\| 12) | +-11.5 | `frame.bridge_rear` |
| the deck (the chassis plate's top) is what the Pi's plate stops 1 over | module z 68.2 | `chassis`, `frame.bridge` |
| a wing's panel inside the synthesis's keep-out (35 core + half its width) | `panel_thickness` 5 = `wing_width_mm` | the linkage file |

**After an edit:**

    pytest -m cad                                  # free: generators, the wing sweep rule
    python -m aow_sim.cad_righting --check         # 1 call: the module itself
    python -m aow_sim.cad_bike --fit righting      # 1 call: does it still fit the bike?

`--fit righting` runs the pose-aware 0.5 scan, 1 mm steps from 1 ahead of
the station, and the front-wheel distance, and prints FITS or what binds. It
also gives the room either way: how far the righting could go back now.
`cad_bike` prints a NOTE whenever the righting no longer matches
`placement.righting_digest`, which it was last fitted against.

Measured ceiling: with the central screw gone altogether (bridge chamfer
7, case 8.5) the module would go back to 142.0, 1.5 better
(`traces/bike_cad/fit2_no_central_screw.txt`). Moving that screw, +Y or
otherwise, buys at most that. Flipping the servo is the larger lever
(below, measured 2026-10-07: ~12).

## In the sim, as the default bike (2026-10-09, user)

User: V2 "is what is going on the bike so yea it needs to be in all the
environments, including the toe and blade". Decisions: the default plant (not
an opt-in); the pill roof stays, no bumpers; masses from the CAD volumes,
`GUESS`, the fixed parts ADDED to `chassis.mass`; the toe a uniform 5 mm.

**What builds it.** `build_model` with no mechanism flag builds bike_params
`righting.module`. `params.resolve_righting_module` inlines the scaled
linkage (this file's `linkage.config` at `linkage.scale`) and the blade
outline (`config/righting_blade.yaml`, was `swing_explore/blade_toe.yaml`)
into params, so `plant_digest` moves when either file does.
`swing_linkage=False` is the wingless bike every earlier export trained on;
`wings=` / `linkage=` / `swing=` / a `swing_linkage_cfg` build their study
mechanism instead.

**The blade** is two convex meshes per wing, blade and toe, placed about the
wing rod from the outline: the vertices match the file to 0.1 mm at qpos0.
The two blades share a contact bit with nothing else, so **the toes are the
end stop**: as first built they touched at crank 136.25 both ways (held
136.6-136.7 under the full 0.55 N.m). The toe went from flared (7.8 mm across
the tip) to a uniform 5 mm on the same tip midpoint: stop 136.25 -> 136.5
on the planar search, nothing material lost.

**Masses, GUESS** (CAD check volumes of 10-07, 2 perimeters / 15 % infill,
`cad_servo_mount.printed_mass`):

| body | g |
|---|---|
| crank: both halves, horn hub, steel crankpin | 11.2 |
| each coupler | 2.3 |
| each wing hub: rocker, knuckle, stub, steel rocker pin | 14.9 |
| each blade (660 mm2 x 82 mm) | 28.6 |
| fixed: cases, bulkhead, bridge, steel rod, XC330 | 64.1 |

Bike 1.016 -> 1.228 kg: +167 g of module, +45 g of roof.

**The crank needed an armature.** V2's crank alone is 2.5e-6 kg m2 (the
wings reach it only through the loop's soft constraints), and the bare
position actuator rang at 1200-1300 deg/s holding any angle but stow. The
XC330's reflected rotor inertia was missing from every servo in the model;
`crank_armature` 1e-3 (GUESS, the order of the XC430's measured 2.2e-3)
stops it. On the firmware model it costs 6 % of the full-rate stroke
(0 -> 110 deg: 0.183 s at 1e-4, 0.194 at 1e-3).

**Measured on it:** the stepthrough's sim rights the bike at 547 mA at 9.9 V
(the flat-panel diamond scored 547 too); the blade clears the floor by
1.64 mm upright at crank 127. The ground station (`hw/ground.py`) strokes
the module's stroke now -- it read V1's 136.6, which on this hardware drives
the toes together. GeneralEnv holds the crank at stow on the firmware model
for policies that do not drive it: env throughput 1.26x the wingless cost
(1.16x on the bare actuator).

**The commanded stroke is 129.3, level on the blade (2026-10-09, user).**
`swing_linkage_shared_rod.yaml` carried 128.4, the synthesis's "flat": its
placeholder panel line -- which is the blade's CENTRE line -- horizontal in
the bike frame. The stepthrough now rests V2 on the blade's whole section
(both faces and the toe) with the sim's wing CoM, and it and the sim trace
cross roll 0 at ~129.3 (the trace had been plotted from the crank's
post-fall angle, -10.6 deg on V2, ~10 deg right of everything else; fixed).
Measured in the sim with no policy (steer straight, wheels held, 9.9 V):
commanded 125 settles at crank 125.8 / +2.4 deg roll; 126 and over carries
the bike past level onto its other side. Cost: knuckle R to coupler L, least
over the stroke, 2.19 mm at 128.4 -> 1.99 at 129.3 (0.41 at the toe stop),
under the CAD test's 2.0 -- open.

**The toe is 2.2 mm longer: it stops the crank before the links collide
(2026-10-09, user).** Each knuckle closes on the other wing's coupler past
~130 deg (2.19 mm least at 128.4, 1.99 at 129.3, 1.83 at 130, 0.41 at
136.25, touching at 138.0) -- "the knuckle and coupler really are gonna run
into each other at past 130deg". The uniform toe runs on along its centreline
to 37.4 mm from the foot: the toes now meet at 129.75-130.0 planar, 129.9 in
the sim, held at 130.5 under full torque, where the knuckle still has ~1.7 mm.
`test_the_knuckles_sit_beside_the_other_wings_coupler` sweeps to 131 (1.5 mm
floor) instead of the commanded stroke (2.0). OPTIONAL, LATER: a dogleg
coupler, bent round the other knuckle, for more travel; the toe would then be
shortened again.

## Outstanding

- **Shrink the stack: the 6 mm plates are DONE (2026-10-05), lowering is not.**
  Notes from 2026-09-30, kept for the lowering: The axial stack and
  the mechanism's height set the bike's wheelbase
  (`bike-assembly-design.md`, Outstanding). Keep the bulkhead's 10 (the
  journal's bearing length) and the couplers' 6 (user).
  - **The web's 6 and the hub zone's 8 come from where the wing joint is**
    (user's catch, 2026-09-30). The rocker-side tab in the web plane forces
    web >= tab (the nut); the boss and countersunk head fill the hub zone.
  - **Move the joint outboard:** to the boss's far side, past the hub zone
    in Y, beside the bulkhead (+-18) and clear of it, since the wing is at
    |x| >= 30. Then neither layer carries the joint. They only need the
    rocker arm's strength and the crank's shoulder, and the rocker's 14 mm
    sleeve is not needed (the knuckle shares the wing).
  - **Lowering the mechanism** shortens the bike ~1:1 under the drive's
    underside, but that is a linkage re-optimisation.

  Kept as is for now (user).
- **Held until the steer module prints,** as the drive is (user). It shares
  the ideas here: the printed bearings, the 6-32 joints, the bridging, the
  cases and the horn hub. Those judgements come from it first.
- **Try the seam fix here first** (user, 2026-09-30). The steer's
  printed bushing binds in two spots 180 deg apart, where the shaft and
  bore seams meet and where they sit opposite (`steering-design.md`, "The
  seams"). Force the seams of the journals and their bearings somewhere
  harmless.
- **Print tolerances are `GUESS`:** the journal bore (0.2 diametral), the
  running clearance over a boss (0.2), the washer's (0.1 a side),
  line-to-line rod holes to ream.
- **The panels are placeholders,** 6 mm solid on the sim's line, and most of
  the mass.
- **Not checked against the rest of the bike:** drive, wheels, chassis. The
  chassis plate is a placeholder.
- **Torque still reaches the servo:** a knock twisting a wing about the rod
  goes through the linkage to the gearbox. Pin forces do not.
- **The 1.25× linkage is in the sim (2026-10-09)** as the default bike, blades
  and toe stop included; 547 mA at 9.9 V. Its masses are CAD `GUESS`es and the
  crank's armature is a `GUESS`: weigh the parts, coast the crank down.
