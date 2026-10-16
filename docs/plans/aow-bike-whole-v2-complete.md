# layout and CAD for drive module (continued)

> **Status:** new document (2026-10-16)

New design instructions for aow-bike-whole-v2 (last work in: `aow-bike-whole-v2.md`)

## aow-bike-drive

This part takes on the main 'chassis' component of the bike, in two halves

in general, nut slots can be a bit tighter so that the nuts don't fall out (will make assembly easier; the current size for 6-32s on ASA are pretty free and can fall through, which was good/ok for the initial setup)

### side rails

- in principle, extend forward from the front of the bike, as monoparts from the drive servo sides, so they print inward on the bike from the rough outside edges of the drive servos (as the drive servo cases already do)

### AHRS mount

- two attachment points on each drive case side, positioned roughly where it already is
- supplied M2 screws and nuts (a bit tight, but doable):
  - pcb thickness (ref): 1.6 mm
  - screw length: 4 mm
  - nut dimension: 4 mm flats, 1.5mm thk
- so it's only like 2 layer lines here between nut and PCB but it should be solid with the nut + 4x screws
- thinking tiny nut slots here

### roll cages (front and rear)

- for now make these stubs extending out and upward from the rail (doesn't go up and over the bike yet)
- front and rear attach should try to be symmetric (height and longitudinal position along the bike matched relative to the wheels)
- front attach probably just in front of the steer servo, rear attach above and behind the drive servos
- eyeballing it, would be just above the center of each front and rear wheel

## aow-bike-righting

- bridge should attach with one 6-32 from each side of the bike to the new rails (in between the rails I guess?)
- blade design mockup fine for now, need to design rollcage and rest of bike before coming back to this setup

## aow-bike-steering-v2

- nothing crazy comes to mind yet, leave in principle but can change for better rail integration

## aow-bike-whole-v2

The remaining parts which don't belong to the 3 main submodules

### Pi + U2D2 carrier

Back to feeling like best layout is having the Pi on top, and the U2D2 underneath (skinnier, fits between the drive pulleys). This should be a self-contained carrier, not exactly sure how best to get it attached to the main rail so just build the self-contained carrier first before deciding.

### Electronics/battery/switch

Nice to have these all in a single carrier which attaches with 1 screw from each side of the bike

### power breakout board (electronics module)

Built the power breakout board, it looks roughly like the ascii drawing:
```
_____|   |_____
|             |
=           o |
|             |
=           o |
_______________
```
This is the view from the left side of the bike, and it should sit above the LHS drive servo/pulley (the one with extra space here).

XT30 comes out the top, 2 pin for 5V pi power and 3 pin JST connector for 12V dynamixel power come out the front (left, on this diagram); 2x 4-40 screws on the back (right, on this diagram) to mount to the carrier

Made on perfboard. External dims rough, 4-40 screws as position references (and the XT30 referenced to it); note that I used a corner of the perfboard so the bottom right corner is roughly aligned with the expected grid, but the top left corner floats a bit off the grid:
- 0.1in grid perfboard (9x9 full holes visible; cut on the 10th in the bottom right)
- w/h 26.5 mm/25 mm
- from bottom right (cut on grid):
  - 4-40 screw over 2,2 grid points (~5mm in on w/h); next 4-40 up 6 grid points (~15.24mm)
- XT30 connector on the 3 middle top pins (only 2 used); extends ~11mm above the board; slightly offset (shifted ~0.5 pin spacings towards the rear of the bike)
- power pin is the bottom two pins, jst pins are the 4th-6th pins

### power switch

sits aft of the power breakout board, in principle at the same height as the XT30 connector so that from the top/back they sit flush with each other

### battery

- in principle, stays in the same spot, I need to find a strap to secure it but in principle a single strap slot for onewrap type velcro, although the battery shouldn't fly out without the velcro attached

## Round 1 (2026-10-16): rails, AHRS tabs, cage stubs, the bridge joint

Pushed: `aow-bike-drive`, `aow-bike-righting`, `aow-bike-steering-v2`,
`aow-bike-whole-v2`. Each module's `--check` and the bike's are clean: no
interference at rest, steered 30/90/180/270, or at any righting pose; 0
hanging edges, 0 flat crowns (bar the chainstay fillets' known false hits).
16 Onshape calls (0 -> 16 of the 13 Oct 2026 year). 4 of them were drive
checks spent on Claude's own build bugs: a groove sketched along an oblique
direction will not extrude (three checks to find it), and the rail's
separate pieces left as stray bodies.

**Nut slots, snug.** A second 6-32 fit in `servo_mounts.yaml`,
`nut_slot_*_snug`: 6.45 across the flats (was 6.55, 1/4" + 0.1) and 2.7
thick (was 3.0, 3/32" + 0.3). GUESS, to try on a print. Drive, righting and
steering `--v2` use it; v1 (frozen) and everything already printed keep the
as-built slots.

**Side rails (drive).** Each case side grows forward as one part, printed
on its outer face (|x| 20.6) like the case side. Drive config `rail:`, the
outline given in the BIKE side view:

| | |
|---|---|
| thickness | 4.6 everywhere (inner face \|x\| 16.0) = the house joint plate. Inside the case side's own outline it stays the plate's 3.6 (the servos are there) |
| band | from the case side's lower-front corner forward under the bridge's underside (z 38.6), up over the fork's top (module z 68.75, 23 behind the steer axis) to the steering's side pad |
| under the AHRS | wholly (user): the top edge runs from the case's lower-front corner to the pad, 1.7 under the TM151's board at its closest |
| steering joint | one 6-32 a side, on the pad (bike y 180.9, z 73.0); the ridge on the pad, the groove in the rail |
| bridge joint | one 6-32 a side into a lug on the bridge (bike y 120.5, z 43.6); ridge on the lug |
| front cage stub | from the pad up to a 12-wide flat at z 100 over y 194.5..206.5, centred on the front axle |
| rear cage stub | from the case side's top edge up and back to the same flat at z 100 over y -6..6, centred on the rear axle |

- The left rail is built and **mirrored** to the right. The case sides were
  one part turned 180 deg; with rails they are mirror images (equal volumes,
  checked).
- The rail overlaps the case side by only 1 mm along the case's edges.
  Deeper, it filled part of an M2.5 counterbore, which reaches |z| 27.75 of
  the case's 29.75 (`--check` caught it as an L/R volume mismatch). The
  generator now refuses that.
- The fork sweeps r 25.5 round the steer axis below module z 67.25 (legs to
  |x| 22.5, plate +-12.1). The first outline met fork L at 30 deg steer.
  The rail now keeps 3.45 clear of that disc at every angle, and
  `cad_bike_v2` refuses less than 1.
- Its joints are checked against the steering's and the righting's own
  (`cad_bike_v2` refuses a miss, and says the value to use).
- The rear stub ends short of the skirt's wire gap on the left (drive y
  86.9..97.5). On the right the case's wire gap is on its lower edge, where
  the rail's band runs. The user: there is enough clearance, and the wires
  have to come out there (the other side is buried between the servos,
  which touch). To be cleaned up later.

**AHRS mount (drive).** Two tabs a side off the case sides' front edges,
on the TM151's own holes (31 x 30, `ahrs_fixture_cad.yaml`).
- The board's back face is on the tabs (drive y 122.0, 2.6 further out than
  the old envelope). Its lower edge is on the case's lower face, the
  pin-header edge up the face.
- Each M2 goes in through the board, a 0.5 web, then the nut (4 AF x 1.5,
  slot +0.1). The 4 mm screw ends 0.3 past the nut and 0.7 short of the
  skirt wall.
- The nut slot drops in from the tab's inner end, open upward as the part
  prints. The hole is teardropped, since it lies level as printed. The
  nut-slot and hole clearances are a GUESS.

**Steering v2.** Side pads 5.0 -> 3.65, so the pad's face plus the rail's
4.6 puts the head on the rail's outer face. The nut slots move 1.35 into the
case (user: room there), 1.8 off the turning cavity. Lower case ~12.9 g
printed ASA (was 13.2).

**Righting.** `rail:` in its config.
- The bridge grows a lug a side out to the rail's inner face (|x| 16.0),
  5 above the bridge's underside and 10 tall, with the ridge on it. The nut
  drops in from the lug's top.
- The rails are drawn there as placeholders, which the v2 bike drops for
  the real ones.
- The old chassis-plate placeholder narrows inside the lugs. It is unused in
  v2.

**Not done this round** (next): the Pi + U2D2 carrier, the
electronics/battery/switch carrier, the power breakout board and the switch
(dimensions needed: the switch's body and cut-out), the battery strap slot
(the strap's width). The power board envelope is still the oversize box on
the drive's face.

## Round 2 (2026-10-16 pm, user's review)

Pushed: `aow-bike-drive`, `aow-bike-righting`, `aow-bike-whole-v2`
(steering v2 unchanged). Each `--check` and the bike's are clean. The one
thing reported is the AHRS USB plug's keep-out reaching the power board's
old envelope on the face, which is to move (below). 9 calls (16 -> 25).

**Righting: the bridge fills the gap between the rails.**
- Its half-width is 16.0 (the rails' inner faces; was 14.75) and it is 10
  tall (was 8). The rail joints are now on its sides, with no lugs.
- The old chassis placeholder and its two nut slots and ridges on the
  bridge's top are gone whenever the rails are on. v1 keeps them.
- `rail.bridge_height` in `righting_cad.yaml`.

**Cage stubs.**
- **Rear: back, then up** (user: up-and-back from the case's top edge is
  where the power switch is to go). From the case side's rear chamfer,
  vertical in the bike at y 29.2, it runs straight back at z 57..69 (5.8
  over the tyre) to y -6, then up to the flat at z 100 over y -6..6. Front
  unchanged.
- **A 6-32 mount in each, in from the side,** like every rail joint: the
  house joint's nut half.
  - The head goes in the future cage part, 4.6 outside the rail. The groove
    is in the rail's outer face, along the stub, and the ridge will be the
    cage's.
  - The nut sits 2 past the joint plane, in a full-thickness boss inward.
    The rear boss's inner face is at |x| 12.0 (1.0 off the battery); the
    front is at 12.85 (0.5 off the steering's lower case, which the first
    try at 12.0 hit).
  - The nut slot runs up out of the stub's top flat, its roof bridged in
    two layers as the rail prints.
  - Screws at z 94. That is clear of belt R's side-view footprint, so a head
    outside the rear stub fits; lower down, belt R runs 2.6 outside the
    rail's outer face.
- The rail's thick part is now cut back by the case side's real outline,
  chamfered corners included, so the rear stub keeps its 4.6 where the
  corner is cut away. The counterbore guard is now each piece's distance to
  every M2.5 counterbore.

**AHRS.**
- **Web 0.9,** two line widths (was 0.5). The board moves 0.4 forward
  (back face at drive y 122.4); the 4 mm screw ends flush with the nut's
  far face once the nut is pulled up.
- **Turned USB up/back** (user): the USB-C edge is up the face toward the
  battery, the pin-header edge down. The holes move with it (0.45 off
  centre): tabs at drive z -25.7 and 5.3. Pointing down, the plug's
  overmold reached the bridge.
- **The model is the TM151, not a box:** the board with its four
  Phi 2.1 holes, the housing off-centre, the USB-C receptacle, and a
  keep-out for the plug (typical overmold, 30 x 12.4 x 6.5, GUESS). A
  keep-out's hits are reported by `cad_bike_v2 --check`, not failed. It
  now reaches the power board's old box on the face, which the electronics
  carrier replaces.

## Rounds 3-4 (2026-10-16 evening, user's reviews)

All four studios pushed. Each `--check` and the bike's are clean (the
AHRS plug's keep-out still reaches the power board's old box). Calls 25 ->
41: three billed bike checks found real clashes (the ridges in their
grooves' tapered ends, twice; nothing else).

**The modules slide out of the rails downward** (user: the steering and
the righting must come out "without removing the chassis sides from the
drive servos"). The house ridge along Y locked them in. Now:
- **Righting:** the ridge is VERTICAL on each side of the bridge, from 3
  over the screw all the way down to the bridge's underside, cut flush with
  it (the bed).
- **Steering:** the ridge runs UP THE STEERING AXIS on each pad, from the
  pad's bottom (the case's bed) to 3.5 over the screw
  (`v2.side_ridge_above`). The nut slot stays perpendicular to it, out the
  front.
- **The rail's grooves** run the same way and out through the rail's lower
  edge, 2.5 past it, so their 45 deg end taper falls outside the rail. At
  first it fell inside, and the ridges, now running to their parts' beds,
  met it. The generator refuses a taper inside the rail
  (`rail.*_groove_open`).
- The steering's upper <-> lower lugs are well above the rail and do not
  come into it (user).

**AHRS:** ONE tab a side, spanning both holes, the case side's full 8.6
(|x| 12.0 to 20.6).

**Cage posts** (`rail.cage_posts`, `cage_post_axis_deg`, `cage_post_*`):
- **Where:** short posts at 15 deg. The front one is parallel to the
  steering axis just ahead of the steering's foot (module y 13.55..25.55,
  from z 68.75 over the fork). The rear one is its mirror about
  mid-wheelbase: the same height, axis at 75 deg. The screws are at bike
  (199.7 | 0.8, 78.5), 99.46 either side of 100.25. `cad_bike_v2` refuses
  them unmirrored, or the front off the steering axis.
- **The rail ends** just past the steering's foot (module y 14) and runs
  into the front post. A short bar back off the case side's rear chamfer,
  at ~15 deg, carries the rear post.
- **Thickness:** the rail's 4.6, nothing inward. A boss there blocked the
  front fork's nut slots.
- **The post is the male locating feature:** the cage's socket slides DOWN
  over it along its axis, with 2 mm lead-ins at its top corners.
- **The joint:** one 6-32 from the side, the head in the cage part 4.6
  outside the rail. The nut slot runs through the post front to back (user:
  "nut slots can enter from the front and back"), centred in the 4.6 with
  0.95 walls (two line widths), its roof bridged in two layers.

## Round 5 (2026-10-16 night, user's review)

Pushed: drive, steering v2, the bike (righting unchanged). Each `--check`
and the bike's are clean. Calls 41 -> 47.

**Cage posts, redrawn.**
- **Part of the outline:** the front post is drawn INTO the rail's outline,
  so there is no seam where a separate piece met the rail. The rear post is
  its exact mirror about mid-wheelbase, computed, not typed (the user found
  it 1.73 deg off its stub and 0.47 proud).
- **The rear arm** continues the post's bottom and the band's top edges
  (15 deg) and ends ON the case side: on its rear chamfer face and, at the
  top, along its top edge from the chamfer's corner. No overlap into the
  case, which is what had made it meet the case untidily.
- **Config:** `rail.cage_post_*` in the steering's module frame, with
  `steer_axle` and `steer_rake_deg`, which `cad_bike_v2` checks against
  the bike's.
- **Nut slots:** from the screw out through the post's FREE edge only
  (front post forward, rear post back), 7 long. Through both edges they ran
  on into the rail, e.g. into the steering's side joint.

**The rail beside the steering** is flush with the lower case's bottom
(module z 68.25, was 68.75: a 0.5 clearance to something already clear).
The fork's top is 1.0 under it; the fork's swept disc keeps 3.1.

**Steering upper <-> lower: ONE 6-32 at the rear,** on the centreline. It
replaces the two side screws, whose lugs and blocks stood over the rails
and would not slide past them.
- The head sits in a lug off the upper case's rear face (pocket 0.5 off
  it), its back face up to the head, then 45 deg (it prints cap-down).
- The nut is in a block off the lower case's rear walls (straight down,
  then 45 deg), its slot out the back, roof bridged.
- Lower case ~12.2 g, upper ~7.9 g printed ASA.
- The U2D2's envelope sat there and is PARKED (not drawn) until the
  Pi + U2D2 carrier: user, "the u2d2 won't be there anymore".

