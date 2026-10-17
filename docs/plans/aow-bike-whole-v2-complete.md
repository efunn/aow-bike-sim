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


## The battery + power carrier, first pass (2026-10-16 night)

Pushed: drive, the bike. The drive's `--check` and the bike's are clean: no
interference at rest, steered 30/90/180/270 or at any righting pose, no
stray bodies, nothing in a keep-out. Calls 47 -> 54. Three of the seven were
waste: one drive check re-run by a careless command, and two bike checks on
Claude's own build bugs (cutters interleaved with the adds under one parent
Id, which Onshape refuses; standoffs started 0.5 outside the plate instead
of inside it). Layout `aow_sim.cad_power`, numbers `bike_v2_cad.yaml
power:`, drawn by `cad_bike_v2`. ONE printed part, `power carrier`,
16.3 cm^3 (17.5 g if solid ASA; printed, less).

**The joint, one 6-32 a side, on the rails' rear arms** (user: "off the rear
rollcage arms, but as close to the drive case as possible"; the screw
"should enter fully clear from the side view").
- **Where:** bike (12.9, 75.3). That is ym 7.0, zm 76.0 in the rear post's
  mirrored frame, the cage screws' height.
- **Why there:** nearer the case, the head is behind belt R's upper run in
  side view; at ym 7.0 it clears that run's outer face by 0.8 past the head.
  Further back, it runs into the room the cage's socket needs ahead of the
  post.
- **Turned round from the house joint:**
  - the flat head is in the rail's outer face (|x| 20.6);
  - the nut is in a lug on the carrier, between the battery and the rail (|x| 11.3..16);
  - the nut sits 1.0 past the joint plane and 1.0 short of the battery, not
    the house 2.0, because the lug is only 4.7 thick;
  - the screw is cut down to 8 (user: "I can shorten the 6-32 screws"), so
    its tip is at the nut's far face, 1.6 off the battery's side.
- **Locating:**
  - the lug's ridge runs up its face, into a groove in the rail's inner face that opens UP through the arm's top edge, so the carrier drops in from above;
  - the nut slot runs up out of the lug's top;
  - drive `rail.carrier_*`. `cad_power` refuses a joint off the rail's.

**The tray** (drive frame, on the case sides' top edges):
- **The pack:** 2.5 forward and 3 up from where it was, so the strap's tunnel fits under the floor ahead of the wire gap.
- **Floor:** 4.0, |x| 15.7, sitting on the skirts.
- **Side walls:** 20 of the pack's 30.
- **Rear wall:** the full 30. The pack sits against it down the 45 deg slope.
- **Front lip:** 5.
- **Removal:** with the strap off, the pack lifts out along the drive's +z.

**One strap slot** (3/4" One-Wrap, user):
- **The slot:** a tunnel under the floor, across, drive y 98.6..118.45. The strap runs round the pack's front third and the floor.
- **The wire gap:** the left case side's servo B wires come up through its top edge at drive y 86.9..97.5. The floor and the plate are notched there, and the tunnel starts ahead of the notch.
- **Not further back:** further back, the strap would run up the rear arms.
- **GUESS:** the strap's thickness, 2.0.

**The left plate (3.0) is the bracket.** The tray's left wall runs up and forward over the left drive pulley.
- **The board** sits on 2.0 standoffs, solder side in (user: ~2 needed), components out to |x| 28.9, inside the pulley's 34.2.
  - Its rear-bottom corner is at bike (66, 105), 2.7 over the pulley's flange.
  - The 4-40s are on its grid: (2, 2) and 6 pitches up.
  - The XT30 sits on the middle three top pins, half a pitch aft, 5.5 thick and flush with the back face, its top at bike z 141.
- **The switch** (v1's: 13 x 19.4 cut-out, 18.5 deep) sits in a level panel aft of the board, its bezel's top flush with the XT30's.
- **The plate's bottom** is 0.5 over the case side's front-top corner (the skirt's front wall is under it).
- **Over the strap:** where the strap crosses the pack, nothing of the plate stands above the pack.

**Parked: the Pi.** Its box (drive y 151..171) ran through the board, and it
moves to the Pi + U2D2 carrier (user: Pi on top).

**GUESS:**
- the XT30's length along the board's edge (10.2);
- the front headers' height (8.5);
- the plug keep-outs (14 forward, 16 over the XT30);
- the 4-40 board holes (2.9) and pilots (2.3, self-tapping);
- the switch's bezel.

**Open:**
- the carrier's print orientation: the standoffs and the switch panel stand off the plate's outer face, so it cannot print on that face;
- the hole teardrops;
- the cage socket must keep forward of bike y ~8 inboard, where the lug's rear edge is;
- where the battery's and the servo wires run.

## The carriers, second pass (2026-10-16 night, user's review)

Pushed: the bike. `--check` clean, first time: 74 bodies, no interference
at rest, steered or at any righting pose. Calls 54 -> 56.

**The power board and switch move to the battery's side**, over the
drive's back-top (user: "on the side of the battery ... instead of the
front-top"). The front-top is the Pi + U2D2's.
- **Board:** outboard of the left rail. Its solder face is 0.5 off the rail's outer face, on 6.8 standoffs from the plate. Its rear-bottom corner is at bike (6.9, 81).
- **Three bounds set where it goes:**
  - its front stays behind drive y 98.6, where the strap rises beside the plate (it reaches 98.57);
  - its front-bottom corner is 5.5 over the case side's top edge, the servo cable's room (user: it routes "under the battery case -> forward", the board "not fully slammed against the drive case");
  - the rail screw's head stays clear of everything in side view, 2.0 round it.
- **Switch:** aft of the board, centred across on the XT30, its bezel flush with the XT30's top (bike z 117).
- **Open:** the switch's panel spans the rail's x over the rear cage post (bike y -15..6.5, z 112..114). The cage's rear upright cannot rise straight up out of that post's socket.

**The Pi + U2D2 carrier, an outline** (`cad_pi_carrier`, `bike_v2_cad.yaml pi_carrier:`). It is self-contained and attached to nothing yet (user).
- **Plate:** 2.0, parallel to the drive's front face at drive y 151, 7.4 past the pulleys' rims.
- **Pi:** on 2.6 standoffs (M2.5 pilots, GUESS) on the plate's outer side, its box 2 further out than before (drive y 163).
- **U2D2:** under the plate, between the drive pulleys, 1.1 off the AHRS's housing, in a short cradle at mid-length. Its ends and connectors are left open, since which long side has the connectors is not recorded.
- **Next:** the user expects a third power-carrier side screw near the left drive pulley, sharing a mounting arm with this carrier.

## The carriers, third pass (2026-10-16 night, user's review)

Pushed: the bike. `--check` clean, first time: 76 bodies, no interference,
no keep-out reached. Calls 56 -> 58.

**The power board and switch are square to the drive** (user: "aligned with
the drive case/battery ... xt30 pointing up and back, other plugs point up
and forward"). The second pass had them square to the BIKE, which put the
board behind the pack and the switch behind the axle.
- **Board:** still in the bike's side plane, outboard of the left rail on 6.8 standoffs, its edges along the drive's y and z. Its rear-bottom corner is at drive (88, 33.75), 4.0 off the case sides' top edge for the servo cable (it runs under the tray, forward), 10.8 off the left pulley's flange.
- **Switch:** aft of the board (drive -y), its panel parallel to the drive's back face, its bezel flush with the XT30's top. It sits at drive y 67..86.5, which clears the rail screw's head in side view (0.9 past the head's 1 mm margin): that is what sets the board's y.
- **No strap for now** (user: "just remove the strap for now"). The tray is back where the pack was: floor 2.0, pack at drive (0, 87, 46.75), 1 up from its first place. The wire notch stays.

**The U2D2's plugs** (ROBOTIS drawing, user): the USB-C in one end, the JST
connectors (RS485, TTL, UART) in the other, plugging in along its length.
Mounted with the USB up the face as the AHRS's, the JSTs down, behind the
AHRS. Keep-outs: the USB plug as the AHRS's (GUESS), the JSTs 14 (GUESS).

## The holder (2026-10-16 night, user's review): a panel, not a case

Pushed: drive, the bike. Each `--check` clean (bike: 76 bodies, no
interference at rest, steered or at any righting pose). Calls 58 -> 64. One
of them was waste: a bike check whose finding point sat on a slit cutter's
edge, so `partAt` returned the cutter with the part (BOOLEAN_SAME_INPUT).
The layout now refuses a finding point inside any cutter.

**The power holder** (`cad_power`) is ONE printed panel on the left, between
the battery and the left rail (user: "instead of a case, it's just a panel
(printed -X)").
- **Size:** |x| 11.3..16, drive y 68.6..133.4, 9.8 cm^3.
- **Print:** its battery-side +X face is flat, on the bed. The ridges, the standoffs and the shelf stand off the -X face. Every hole and slit runs along x, upright as printed.
- **The shelf** sits over the board's top edge, round the XT30 (0.3 clear). The switch is panel-mounted in it, aft of the board, its bezel flush with the XT30's top.
- **Two strap slits**, drive y 99.2..119.2: the lower in the cable gap under the board, the upper behind the board. A strap runs between the panel and the board there. Placeholders: the straps are the user's call.
- **The wire notch:** the panel sits on the case sides' top edge and is notched over the left wire gap.
- **No tray:** the battery is back where it was, drive (0, 87, 45.75).

**Two 6-32 into the LEFT rail, fore and aft of the board** (user).
- **The rail grows two ears, both on the case side** (drive `rail.back_ear`, `rail.front_ear`; mirrored, so the right rail has them too, unused).
  - **Aft:** on the case side's top edge under the switch, the head at drive (80, 35.5). The rear arm made the part "unnecessarily long", user.
  - **Fore:** off the case side's front edge at its top, over the AHRS tab and board (9.8 clear), near the left drive pulley. The head at drive (125, 27).
- **The joint:** the head is in the ear's outer face. The nut sits in the panel a wall past the joint plane and a wall short of the battery, and the screw is cut to 8. The ridge goes vertical on the panel, into a groove open up through each ear's top: the panel drops in from above. The nut slots run out the panel's aft and front edges.
- **Side view:** both heads are clear, 1 mm round each: of the board, standoffs, switch and shelf (3.3 to spare), and of the left pulley's belt (1.7).
- **The rear-arm joint is gone.** The arm only carries the cage post now.

**The U2D2** (the ROBOTIS drawing): its JSTs are in its face near the end
away from the USB, facing the drive (down and back in the bike). It slides up
the face (`pi_carrier.u2d2_z` 27.1) until they are 0.5 past the AHRS
housing's upper end. The JSTs' keep-out is small (14 x 17 x 6, GUESS).

## The holder, second pass (2026-10-16 night, user's review)

Pushed: drive, the bike. Each `--check` clean: the drive 0 interference,
the bike 76 bodies, no interference at rest, steered or at any righting
pose, no keep-out reached. Calls 64 -> 68, none wasted (two pushes refused
locally for want of a tab name, free).

**Left only.** The holder's two ears and joints are on the LEFT rail only
(user: "the features are mirrored to RHS unnecessarily"). `rail_layout`
returns them as `leftPieces` / `leftJoints`. The drive studio adds them
after the mirror, unites them into the left case side and cuts their
joints there. The case sides are no longer equal volumes: the drive check
wants the left heavier (+604.9 mm^3), not equal.

**Slides in from the back + top** (user: "rather than straight up and down,
so it can be removed independently from the pi+u2d2 carrier"). Back + top
in the bike is the drive's +Z, so both grooves are closed toward drive -Z
and open along +Z (`ridge_deg` 315). The holder's ridges are drawn along Z
and turned 45 deg about X onto the slide. It comes to rest on the case
side's top edge, and nothing moves in drive y, so it never comes near the Pi
carrier.

| | was | now |
|---|---|---|
| aft joint | back ear under the switch, drive (80, 35.5) | back ear in the rear arm's corner with the top edge, drive (70.45, 35.5); ear y 64.7..76.2, z to 41.25 |
| groove | open up (bike) | open along drive +Z; the aft one 13 open, so it runs out of the ARM across its whole width (a new check: both sides of every groove, against every piece) |
| board | corner drive y 91 | 95.2: its aft standoff 0.5 ahead of the strap |
| switch | centred on the XT30, 1.35 off the panel | its body 2.5 off the panel's outer face (`switch.strap_room`): the strap runs between them |
| straps | two slits ahead of the board (placeholders) | ONE strap round the battery near the switch, drive y 76.7..96.7 |
| shelf | cantilevered | two gussets (1.6 thick, 6.5 tall at the panel), just outside the strap |
| U2D2 | `u2d2_z` 27.1, JSTs past the housing's upper end | -23.25: the JSTs' keep-out 0.5 in FRONT of the AHRS envelope's lower end |

**Rejected: the aft screw in the arm proper.** The arm runs 30 deg off the
slide. A groove along +Z from a head on the arm's mid-line runs about 19 up
the arm before its aft side gets out, into the rear cage post. The
back ear in the corner gets the head "basically inside the arm" (user) with
a groove that runs out in 13.

**The strap** (user: "near the power switch ... literally go above and
below the battery directly"):
- **Over the battery:** over its top, through a slit in the panel at the top's height (z 60.75..63.75).
- **Under the battery:** in the 2.25 between it and servo B's face, inside the skirts, then out through a notch in the panel's bottom edge (the strap lies on the case side's top edge there).
- **Between them:** up the panel's outer face, inside the switch's body.
- Its start is set by the rear arm (its edge at the top edge, 76.18, + 0.5). It overlaps the wire gap's first 9.8: the servo B wires come up under it. The strap is still the user's call (placeholder: 3/4 in, 2 thick).

**The battery stops** (user: "some hints (like 2mm lateral, going slightly
behind the battery) so the battery couldn't slide backwards into the rear
wheel"). On BOTH rear arms: a lip off the inner face in to |x| 9 (2 past the
battery's side), 2 thick, drive y 55.7..57.7 (0.3 behind the battery), z
42.5..50, on the arm and 1 off the cage post's base. Both case sides now
reach |x| 9. `cad_power` refuses a stop off the battery.

**Outstanding:**
- **The cage socket:** the holder slides out inboard of the rear cage post (1.7 off it in side view at rest). If the cage's socket wraps the post's inner face, the cage has to come off first.
- **Print-fit** the cut screws and the panel's nuts.
- **The strap** over the servo wires' exit.
- **The Pi carrier's attachment.**
- **GUESSes:** strap thickness 2, the gussets, the stop's size; and, as before, the XT30's length, the headers, the keep-outs, the pilots, the Pi standoffs, the U2D2's JSTs.

## The holder, third pass (2026-10-16 night, user's review)

Pushed: drive, the bike, steering v2. Checks clean: the drive, 0
interference; the bike, 76 bodies, no interference at rest, steered or at
any pose. Calls 68 -> 75.

**The screw holes into the ears were plugged** (user: "the two ears are
completely missing screw holes"). The rails' holes were cut. But the
holder's ridge, which sits in the rail's groove right over the hole, was
added AFTER the holder's own hole was cut, so it plugged the screw's way for
its 1.2 mm. The steering v2's side pads had the same order; the righting's
bridge did not. Both now add the ridge first and drill through it.
- **Why it got through:** the drive check builds the rails alone, where the hole is open. Only the assembled bike has the ridge in the groove.
- **New check, in both the drive and the bike checks:** three points on each rail joint's screw axis must find no body, and one point in the wall beside it must find the rail (so a missing ear cannot pass for an open hole). The bike's first run found 4 of 24 wrong (the steering and the holder, at 4 in from the outer face), then 0.
- One call went on a first version of the probe without the wall point: it read the ear holes open in the drive alone, which proved nothing.

| | was | now |
|---|---|---|
| lower strap notch | 3 tall, a step into the 5 of the wire notch | 5, the wire notch's height (user) |
| upper strap slit | 3 tall at the battery's top | from the battery's top to the shelf's underside (user: the battery may sit higher, cables under it) |
| battery stop | lip to \|x\| 9 | to \|x\| 8 (user: "an extra mm"), and a SIDE LEG from it 3 forward along the battery's side, 0.3 off it, on both arms (user: "constrained laterally a bit") |
| gussets | 2 | 3: the aft one 1 further back (1.5 off the switch's cutout, for its springy clips), the one ahead of the strap, and one past the XT30's hole (the shelf runs on to take it) |

## The Pi + U2D2 carrier, attached (2026-10-16 night, user's spec)

Pushed: drive, the bike. Checks clean: the drive, 0 interference, 36 hole
probes right; the bike, 80 bodies, no interference at rest, steered, at any
righting pose, or with the carrier lifted 3/6/9/12/15 mm straight up. Calls
75 -> 81, one of them wasted (a re-run of an unchanged check, by a typo).

**Assembly** (user): the U2D2 onto the plate on its two M2 tapping screws
("located centrally and 42mm apart" on its Pi-side face), then the Pi on
four spacers with 4-40 machine screws self-tapping into the plate, then the
carrier into the bike on three 6-32.

| | was | now |
|---|---|---|
| attachment | none (an outline) | three 6-32, the holder's joint turned round: head in the rail's outer face, nut in a lug of the carrier's inboard of the rail (lug \|x\| 11.3..16, screws cut to 8) |
| top joint | | LEFT only, drive (141, 27), on the front ear run on forward to drive y 150 (1 short of the plate; was 130.4). Its lug 2.5 off the holder's front |
| low joints | | one a side, drive (147, -55.5), in the rails' lower arms under the plate, 1.75 of wall to the arm's top edge. The lugs' bottoms 8.9 over the righting's bridge (bike z 48.6) |
| way out | | STRAIGHT UP in the bike (the drive's +Y+Z): grooves open upward (`ridge_deg` 270), the ridges along the bike's Z. Away from the AHRS (behind the U2D2) and from the holder, which slides out back + top |
| U2D2 | in the cradle | the same, plus two M2 through 2.4 holes in the plate, at drive z -44.25 and -2.25 |
| Pi | on 2.6 standoffs printed on, M2.5 pilots | on four SEPARATE spacers (user), 4 tall, 6 OD, 3.3 bore (GUESS); 4-40 pilots 2.3 (GUESS) through the plate and a 4 boss behind it (6 of thread). The Pi's box out 1.4 to drive y 164.4 |

- **The bosses** are 1.8 off the right belt (on the right pulley at the upper holes).
- **The screw heads** are clear in side view: 4.9 past their 1 mm margin off the holder's board and switch, 10.3 off the belts.
- **The lift check is new:** the carrier, its spacers, the Pi and the U2D2 are lifted together and swept against everything else in the same eval, to 15 (the ridges are off the rails by 14 for the top one, 9.3 for the low ones). Its first run caught the top ridge hitting the ear's top-front corner at 9 to 12 mm. The rail's groove run-out check sampled three lines across the groove, and the corner sat between them. It now samples nine, and the top groove is open 13.5 (13.0 is the least that passes).
- **Rejected:** lifting off the face (the drive's +Y). It also clears the AHRS, but the low grooves would run 17.1 along the arm before getting out of the top edge, against 8.2 straight up, and the lugs would travel that much further toward the steering.

**Outstanding:**
- **Side access to the low screws:** the righting's blades are not checked for a screwdriver's room.
- **Print-fit** the spacers, the 4-40 pilots and the lugs' nuts.
- **GUESSes:** the spacers, the pilot, the M2 heads' 1.6 (the spacers leave 2.4 over them), the U2D2's JSTs.
- **The Pi's holes** are drawn at the datasheet's 2.75; the user is opening them for the 4-40s.

## The Pi carrier, second pass (2026-10-16 night, user's review)

Pushed: the bike. `--check` clean: 80 bodies, no interference, probes
right. Calls 81 -> 83.

| | was | now |
|---|---|---|
| low lugs | 0.51 proud of the plate's outer face (user) | flush with it; the ridges 3 down / 4 up of each head (was 5 up) to stay inside |
| plate | the Pi's PCB + 1 all round, down to the U2D2's screw | the PCB's footprint exactly, x +-28 (user: "no necessary reason to extend beyond" it) |
| Pi | `pi_z` 0 | -5.8, down-forward (user: "like another 7mm"). The plate's bottom-back corner is then 1.05 off the rail arm's top edge in side view. At 7 it would be 0.06 off, so the limit is the arm |
| U2D2 | `u2d2_z` -23.25, its JSTs' keep-out past the AHRS envelope's lower end (-29.75) | -17.0, up-back 6.25: the keep-out 0.59 past the HOUSING's lower end (-23.41), the bump (user). The board under it ends lower but is behind the keep-out |

**The lift check now reaches a keep-out:** with the JSTs plugged in, lifting
the carrier 3 to 6 mm sweeps their plugs into the AHRS housing's lower end.
Straight up moves the U2D2 out from the face and up it equally, and the
plugs have 4.9 to go out before they are past the housing, against 0.59 up.
So either the JSTs come out before the carrier lifts out, or the U2D2 goes
back to `u2d2_z` -21.4 at most, 1.85 up from -23.25 rather than 6.25 (the
user's call). The keep-out's 17 along the U2D2 is still a GUESS.

**Then** (user): the U2D2 back down 2 for room, to `u2d2_z` -19.0 (4.25 up
from where it started), its JSTs' keep-out 2.59 past the housing; the user
says the keep-out is generous (the plugs sit flush with its face, only
wires come out). The lift check still reaches it, at 6 only. And THROUGH
holes for all three of the carrier's screws and the holder's fore one (user:
"the rear one is the only that should get cut down so it doesn't pierce
thru to the battery"): no screw needs cutting but the holder's aft. The
shortest that passes its nut is 8.0. The longest before the tip meets
something inboard, against the electronics' boxes and the carrier (the
screws are not modelled): the carrier's top 14.4 (the U2D2's USB plug
keep-out), its low ones 31.9 (each other's lug), the holder's fore 20.6
(the AHRS's USB plug keep-out). Pushed: the bike, check clean. Calls 83 -> 85.
