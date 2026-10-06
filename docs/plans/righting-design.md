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
- **Centre washer,** loose on the crankpin between the couplers: printed
  two layers, or bought and measured into `washer`.
- **Rocker R / L,** printed hub-zone face down: hub on the rod, arm to its
  coupler, ear straight from the coupler joint to the panel boss.
- **Knuckle R / L,** printed ring face up: the rocker's ear and boss without
  the arm, dog-legged low. It is each wing's second support, beyond the far
  bearing.
- **Wing R / L,** printed outer face down: the panel, a tab at each boss.
  One 6-32 along Y into each, the head in the boss, the nut in the tab.
- **Front bulkhead,** printed inner face up: the front journal bearing, the
  rod.
- **Lower case,** printed inner face down: the rear journal bearing and the
  rod in its front wall, the hub's cavity, the XC330's horn half. It is the
  steering's lower case.
- **Upper case,** printed cap down: the XC330's back half, the steering's
  cable window and "H" cap, two ears up to the bridge.
- **Horn hub:** the steering's, same numbers (`steering_cad.yaml`), so the
  same part.
- **Bridge,** printed top up: ties the front bulkhead and both cases, one
  6-32 each with a ridge. Its top face carries the chassis attachment.
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

## Fitting the whole bike: the constraints to keep (2026-10-05)

The layout is settled for now and nothing is printed (user). The righting
will still be reworked before it prints, "without messing up the whole bike
fit". The bike rebuilds this module from its generator every time, so an
edit here reaches `aow-bike-whole` by itself. What it must keep:

| rule | now | set by |
|---|---|---|
| 0.5 to the drive, at rest AND every pose (`placement.rear_clear`) | 0.50 at the bike's 143.5 | the bridge's middle chamfer (4.6, held by the central screw's head seat) on XC430 A; the narrowed rear end passes the case sides' skirts at 0.5 |
| the module less its wings 5 from the straight front wheel (`front_clear`) | 5.25 | the front bulkhead (y 23.5) |
| the wings clear the front tyre's whole steer sweep, every pose, 2 spare (`wing_margin`) | L 2.6 / R 9.2 spare | `wing.panel_back` 41 |
| the bridge's last 10 under the drive case sides' skirts (\|x\| 12) | +-11.5 | `frame.bridge_rear` |
| the deck (the chassis plate's top) is what the Pi's plate stops 1 over | module z 68.2 | `chassis`, `frame.bridge` |
| a wing's panel inside the synthesis's keep-out (35 core + half its width) | `panel_thickness` 5 = `wing_width_mm` | the linkage file |

**After an edit:**

    pytest -m cad                                  # free: generators, the wing sweep rule
    python -m aow_sim.cad_righting --check         # 1 call: the module itself
    python -m aow_sim.cad_bike --fit righting      # 1 call: does it still fit the bike?

`--fit righting` runs the pose-aware 0.5 scan from 1.75 ahead of the
station and the front-wheel distance, and prints FITS or what binds. It
also gives the room either way: how far the righting could go back now.
`cad_bike` prints a NOTE whenever the righting no longer matches
`placement.righting_digest`, which it was last fitted against.

Measured ceiling: with the central screw gone altogether (bridge chamfer
7, case 8.5) the module would go back to 142.0, 1.5 better
(`traces/bike_cad/fit2_no_central_screw.txt`). Moving that screw, +Y or
otherwise, buys at most that. Lowering the mechanism is the larger lever.

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
- **The 1.25× linkage has not been re-run in the sim,** only on the planar
  scorer.
