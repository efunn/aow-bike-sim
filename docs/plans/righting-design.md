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

## Outstanding

- **Held until the steer module prints,** as the drive is (user). It shares
  the ideas here: the printed bearings, the 6-32 joints, the bridging, the
  cases and the horn hub. Those judgements come from it first.
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
