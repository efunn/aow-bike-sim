# The whole bike in CAD

> **Status:** a first assembly, rough in places (user). 2026-09-30: four
> passes took the righting under the drive and the electronics onto the
> drive's front face, wheelbase 300 -> 250. 2026-10-05: the righting's links
> went to 6 mm plates, and the underside electronics and the cage were
> switched off. The Pi came down onto the drive's face and beside the deck,
> the righting went back to 0.5 from the drive, and the front was refitted
> to the user's rule. Wheelbase 223.5.
> Interference-free, nothing printed. See Outstanding for what changes next.

The drive, steering and righting modules placed in one Part Studio, plus
the parts that tie them together.

## spec (user, 2026-09-30)

- Composed of the existing `aow-bike-drive`, `aow-bike-steering`,
  `aow-bike-righting`.
- Added parts: battery holder, electronics carrier, chassis, roll cage
  (roof).
- **Battery:** 58 x 30 x 22 mm, 76 g. The holder may attach to the drive.
- **Electronics carrier:**
  - one side: the Pi 3B+;
  - the other side: U2D2, AHRS, a small power distribution board, with
    2 x 150 mm USB-A to C cables bent round from the Pi to the U2D2 and
    AHRS;
  - a power switch exposed to the side: 13 x 19.2-19.6 mm cutout, panel
    0.75-3 mm, ~18.5 mm behind the panel.
- **Chassis:** whatever is left to connect the components.
- **Roll cage:** protects the top of the bike.
- CAD only: nothing changes in the simulation. The wheelbase is free in CAD.

## Decisions (2026-09-30)

- **Composition, not import.** `aow_sim.cad_bike` merges the three modules'
  geometry layers into one Feature Studio and calls `driveBuild`,
  `steeringBuild` and `rightingBuild` under their own ids before moving
  each into place.
  - The shared helpers are textually identical and kept once. A
    disagreement is refused.
  - An FS `import` was not used: it pins a microversion, and the check
    eval cannot import, so the check would test different code from the
    push.
  - A module change reaches the bike on the next `cad_bike --push`.
- **Bike frame:** origin on the rear axle, +Y forward, +X right, +Z up,
  the axes of every module:
  - the drive is turned 45 deg about X;
  - the steering is moved to the front axle and raked 15 deg;
  - the righting is translated.
- **Wheelbase 300, not 200.** `cad_bike --fit 2` (one call,
  `traces/bike_cad/fit2_righting.txt`, 5 mm grid, zero margin) slid the
  righting along the wheelbase:

  | the righting's rod must sit, ahead of the rear axle | mm |
  |---|---|
  | clear of the rear wheel, axle, chainstays only | >= 125 |
  | clear of the whole drive at 45 deg | >= 190 |
  | the front axle ahead of the rod (steered tyre's ball + fork) | >= 100 |

  The binding length is the righting's axial stack, servo cap to rod tip,
  113 mm. The 1.25x linkage scale does not change it. Set at rod 195,
  wheelbase 300 (`config/bike_cad.yaml`).
- **Chassis = the modules' placeholders, united.** The drive's fixture
  block, the steering's mount plate (bench tab trimmed, an extension added
  above the upper case) and the righting's chassis plate already carry their
  modules' joints. The chassis unites them with:
  - a deck in pieces round the righting's plate (so its countersinks stay
    open), narrow as the wedge at the rear, where the lower drive pulley
    comes down to deck height at |x| 17-34 up to y ~121;
  - a wedge under the drive block;
  - a gusset behind the steering plate;
  - four posts for the carrier, on pads.
- **Battery** stood on its 58 x 22 side between the drive pulleys (+-11
  against their inner faces at +-17), on the case pair's back. The tray's
  tongue screws onto the drive block's top. Its centre lands at bike
  (y 30, z 96), against the sim's battery at (26, 100): the same place.
- **Carrier horizontal on the posts**, Pi on top with its USB end aft. The
  U2D2, AHRS and power board hang in cradles underneath. The switch is in a
  tab off the rear-right corner, facing +X between the drive and the right
  wing, where nothing covers it. The carrier's sides are behind the wing
  panels and its top is the Pi's.
- **Roll cage:** a spine in the mid-plane (printed on its side), from a
  foot on the drive block's front face to a foot behind the steering
  plate's extension, a rail at z 136. Four arched ribs, each on a flat
  saddle on the rail, one 6-32 each.
- **New joints are `bkJoint`:** the modules' 6-32 x 3/8 flat head with a
  ONE-WAY nut slot to the face the nut goes in from. The modules'
  `screwJoint` cuts its slot 200 mm both ways, which in a part the size of
  the chassis cuts through everything in line.

## Second layout (2026-09-30, user's review of the first)

Review notes (user):
- electronics can sit above the drive, leaning into its upper half;
- USB cables point forward, stacked on one side;
- AHRS on the centreline;
- the righting can move back under the drive servos, with knuckle R behind
  knuckle L;
- the upper case needn't sit so far back;
- the battery case "could be just part of the drive case sides", left for now.

| change | why | effect, measured (`--fit 2`, 2.5 mm grid, zero margin) |
|---|---|---|
| knuckle R behind knuckle L (`righting_cad.yaml` `wing.knuckle_r: rear`) | the module ends at the front bearing; the rod stops there | front axle >= 85 ahead of the rod, was 100 |
| bridge +-23 -> +-16; upper case on ONE central 6-32 (`cases.uc_joint: central`), no ears | the ears (+-22) and bridge, NOT their length, hit drive pulley L (inner face at 17): the only clash from station 165 to 185 | rod >= 157.5 behind the drive, was 190 |
| upper case top +2 mm (`cases.top_extra`) | the central nut slot needs it: the top over the servo was 4.05 | everything above the rod 2 mm higher |
| bridge ends past the central screw (-61.2, was -67.7); chassis plate placeholder +-16, its rear joint -50 -> -30 | the plate is the bike's deck there; at -45 the drive block's wedge would bury the screw head | -- |
| wheelbase 250, rod at 160 | 157.5 + 85, with margins | 50 shorter |
| electronics leaning on the drive block's front face | user | top of the electronics ~z 166, the cage rail 173.9 (was 136) |

Why the upper case sits where it does: its length is the servo stack
behind the rear bearing (horn face 37.7, the XC330 23 deep, the cap 4). Its
width was the problem, not its length.

**The leaning carrier** is built in the drive frame: x across, y out of
the block's 45 deg face, z along it.
- **Mounting:** four chassis posts off the block's face (x +-11, z +-24.5),
  one 6-32 each from the carrier's top. The carrier underside is 16 off the
  face (the U2D2's 14.9).
- **Top:** the Pi, USB edge down the slope, a 30 mm cable keep-out past it
  (user). The cables wrap the lower edge to the underside.
- **Two widths.**
  - Below z 18 on the face it is +-28, as narrow as the Pi. That part lies
    inside the wings' span: the far wing, swung ~14 deg inboard at -50 %,
    comes to |x| ~41 there. The first try, at +-38 all the way, clashed
    at +-50 %.
  - Above z 18 it sits behind the wings and is +-38. That holds the U2D2
    (right), the switch (left tab, facing -x) and the power board. The
    board is the tallest (25), past the face's top edge, where it can hang
    deeper.
- **AHRS:** on the centreline, low, over the face.
- **Cage:** its rear foot moved from the block's face, now taken by the
  posts, to the carrier's upper end, over a boss for the nut. The rail
  height and its bend down to the steering foot come from the electronics'
  envelopes plus 8 mm (the carrier plate plus 1).

**Third pass, the same day (user): the electronics lower, the power board
as tall as the U2D2** (its 25 was a guess that set the layout).
`cad_bike --fit e` (one call, `traces/bike_cad/fit_electronics.txt`)
builds the bike without the electronics, then slides their envelopes
down the drive's face and off it. Each position is tested at rest, steered
and at five righting poses:

| down the face / off it, mm | result |
|---|---|
| -10 / +0, -10 / +6, -12.5 / +0, -15 / +3 | clean |
| -15 / +0 | the far wing (inboard, -56 %) meets the U2D2 |
| -17.5 / +0 and lower | the far wing meets the U2D2 and the switch tab |
| any / +12 | the cable keep-out meets the steering plate and lower case at rest |

So it went down 15 and 3 further off: sitting further off the servos buys
just those 3 mm. The four posts stay on the block's face, so the underside
was re-laid round them:
- the AHRS on the centreline between the posts;
- the U2D2 across the strip below the lower pair, its cable end at the edge
  the cables wrap round;
- the power board above the upper pair, on the right;
- the switch above the AHRS, on the left.

The cage rail came down 173.9 -> 169.3, held now by the power board's upper
corner. The electronics' lower end came down ~11.

**Fourth pass, the same day (user): the real cable path.**
- The two cables use the Pi's LEFT USB stack: plugs 17.5 long x 15 wide,
  then a 6 mm bundle with a 6 mm hard stub, then a 6 mm-radius bend.
- They U-turn round the carrier's lower edge to the U2D2 and AHRS, whose
  USB-C ends both face that edge, side by side. The U2D2 runs lengthwise
  (turned 90 deg), the AHRS ~7 right of centre beside it.
- The power board and switch ride high for now (user: they will not stay
  there).

Consequences:
- **The U-turn reaches below the carrier, so it, not the wings, sets how low
  the package goes.** `--fit e` with that envelope: every position lower
  than edge -33 (the carrier's lower edge, along the face) puts the cables
  into the deck over the righting, and at -10 into its bridge. So the
  carrier is back at -33, 19 off the face; the -15 slide of the third pass
  is undone. The layout is in carrier coordinates (`u` up from the edge),
  so `edge` moves it all.
- **One row of posts.** With both devices along the lower edge, a lower row
  would land inside them. The carrier sits on the upper pair (x +-11,
  z 24.5) plus the cage's rear foot, now past the Pi's upper end, so its
  strut cannot rise through the board. The lower half cantilevers ~55 mm
  past the posts: outstanding.
- **The cage rail is 175.2,** set by the Pi's GPIO header at the board's
  upper end. The cage top is 184, 235 above the floor.
- **The cables are modelled as cables, not a keep-out box** (user: the box
  "doesn't represent the cable narrowing").
  - Two USB-A plug bodies (17.5 x 15, stacked), then a 6 mm pipe per cable:
    the 6 mm hard stub straight out, then bends at a true 6 mm centreline
    radius (`cad_bike.rounded_path`, which refuses a leg too short for its
    bends). Down past the carrier's lower edge, back under it into each
    device's USB-C plug.
  - The upper plug's cable nests one bundle outside the lower one's in the
    turn and shifts 8 across on the way down, so it reaches the AHRS without
    crossing the U2D2's.
  - The USB-C end is a guess: a 12 x 6.5 x 15 plug body behind the same hard
    stub. It puts both devices' connector ends at the carrier's lower edge
    (u 1).
  - A test checks every bend's radius from the samples.

**Fifth pass (2026-10-05, user): pared down to place the Pi.**

- **Switched off, not deleted:** the U2D2, AHRS, power board, switch, the
  two USB cables (`electronics.underside: false`, "they need to go back in
  later") and the cage (`rollcage.enabled: false`, "get the rest of the
  design sorted before getting that in"). The test fixture `L_full`
  switches them all back on at the old placement and checks they still
  build.
- **The Pi turned 180 deg** (`pi.usb_edge: up`): USB/Ethernet up the
  slope, so the GPIO header changes side. Its M2.5 holes go through the
  board, not only the plate.
- **The carrier is one plate round the Pi** (+-28, 89 long), on two rows
  of posts again. The cantilever is gone.
- **Gap 8 off the drive's face** (was 19): the least that leaves each post
  its nut (2 past the joint plane, 3 thick, 1 wall).
- **The righting's 6 mm plates** (`righting-design.md`) let it back from
  157.5 to 150 (`--fit 2`). The front stays at 85: the wing panels against
  the front tyre's sweep set it, and they did not change. Wheelbase
  250 -> 235.
- **`--fit e`** slid the plate 0..-25 down the face at gap 8/11/14: clean
  everywhere, at rest, steered and posed. Edge -33 -> -58, which is also as
  low as the upper post row allows. The plate's lower edge is then ~7.5
  above the deck over the righting.
- **Ghost wings** (dialog tickbox, on by default): each wing again at its
  most inboard pose. That is the far wing at 56 % of the throw, 14.2 deg
  in, translucent and untagged, so poses and the check ignore it.
- **`placement.righting_digest`:** the bike rebuilds the righting from its
  generator every time, so a linkage or stack change comes in on its own.
  The placement and the edge are measured numbers, though: `cad_bike`
  prints a NOTE when the righting no longer matches the digest they were
  probed against. Re-run `--fit 2`, `--fit e` and `--check`, then update it.

**Sixth pass (2026-10-05 pm, user): the front by the user's rule, the
righting's rear chamfered, the Pi tight.** The process (user): this is
subassembly design (steer, drive, righting) against how the rest can be
packed. The packing only has to be "kinda possible", and every constraint
that turns up gets written down (below).

- **The front:** the righting less its wings sits 5 from the front wheel
  standing straight. That is the front bulkhead, 5.25 at 80 ahead of the
  rod, was 85. `--fit 2` now measures it by distance.
- **The wings clear the tyre's whole steer sweep at every pose,** 2 to
  spare. `layout()` refuses otherwise (`wing_front_room`). That took
  `wing.panel_back` 50 -> 41 (panels 77 long, were 86). The binding pose is
  not rest: a wing swung inboard comes nearer the tyre.
- **The rear:** at 150 the drive's case sides came down onto the rear-top
  corners of the bridge (0.75, at |x| 14) and the upper case (1.8, at |x|
  13), at 45 deg. `--fit 2` TIGHT rows report where it is tight.
  - Chamfers: 6 on the upper case's -Y +Z edge (2.8 wall left over the
    servo; 7 at most, then the central joint's ridge), and 6 on the bridge's
    rear-top edge outboard of |x| 5 (clear of the central screw's head).
  - The righting goes back to 147.5.
- **The Pi:**
  - standoffs 3 (were 6, a guess);
  - the carrier one joint plate (4.6) thick on 2 mm pads, nuts in the
    drive block (the block is chassis);
  - edge -60.5: its corner sits 1.5 over the deck over the righting
    (`--check` bounding box);
  - the board's underside is 9.6 off the block's face, was 17 this
    morning and 28 before.
- **Wheelbase 227.5** (147.5 + 80).

**Seventh pass (2026-10-05, late pm, user): slammed.**
- **The righting to ~0.5 from the drive.** `--fit 2` now scans back in
  0.25 steps (REARD rows) until the distance drops under 0.5: 143.5, at
  0.66. The chamfers grew first: case 7, bridge 7 outboard, and 4.6 in the
  middle, THROUGH the central screw's counterbore and stopping 0.3 over its
  head seat (user: "the chamfer can intersect the counterbore").
- **The central screw moved 6 forward** (user), its nut slot turned along
  Y, the nut partly over the lower case in a relief. REVERTED in the eighth
  pass: the lower case's undercut blocks it in 3D.
- **But the wings bind first.** The 0.5 scan now tests every righting pose
  as well as rest. At rest the case and bridge would allow 139.75; with the
  poses it stops at 143.5, where wing L at +56 % (the far wing, 14 deg
  inboard) brings its rear end, mid-height on the panel's inner face, to
  drive pulley L. A rest-only scan had passed 139.75 and the full check
  caught it.
- **Fasteners (user):** 6-32 with a captive nut and a countersunk head for
  the general assembly; self-tap into printed parts is fine; no self-tap
  into Dynamixel horns (XL330 reused, XC330 only at final assembly; the
  XC430 has brass inserts).
- **The Pi's plate ON the face** (gap 0), with a pocket under each drive
  pulley (they stand ~0.7 proud; 1.7 deep with 1 mm round the flange).
  - The middle stops 1 over the deck; two legs outboard of +-17 carry the
    board's lower holes.
  - Printed standoffs on the plate, a self-tap screw into each (user's plan;
    which part they finally come from is open). Pilot 2.2 (GUESS), 5.4 deep,
    stopping 0.5 short of the pockets.
  - (An intermediate version drew heat-set inserts, which nobody had asked
    for, and set a false limit on them. Removed.)
- **Edge -69.5:** clean (`--fit e`); 1.5 further and the board itself
  meets the deck.
- **Wheelbase 223.5** (143.5 + 80).

**Eighth pass (2026-10-05, evening, user): the righting made to work.**
- **The central screw is back behind the lower case** (`uc_joint_dy` 0).
  Forward, the lower case's undercut blocked the nut and the screw below
  it (user, seen in 3D). The slot stays turned along Y.
- **The bridge is one full-width chamfer** (4.6, the head seat's limit)
  and 14.75 wide (was 16).
- **The wings' knuckle tabs moved to the knuckles' +Y side** (`knuckle_tab:
  inner`), beside the lower case, as the rocker-side tabs sit beside the
  bulkhead.
- **The panels are 5 thick** (were 6): the synthesis holds the panel's line
  2.5 outside its 35 core (`wing_width_mm` 5). At 6 the inner face sat 0.5
  inside it. That was the whole of wing L's interference with drive pulley
  L; the wings no longer bind.
- **What binds the rear now: drive case side R's SKIRT.** The band wraps
  the XC430 at |x| 12-17, and its lower rim comes down at 45 deg over the
  bridge's rear-top edge, with XC430 A across the middle. It is not the
  fixture tab at |x| 16-20.6. So narrowing the bridge to 14.75 bought
  nothing (0.467 at 145 either way); depth of chamfer is the only lever
  there. The earlier 7 mm step outboard of |x| 5 bought 1.75.
- **The bridge's last 10 mm narrowed to +-11.5** (user: "<24mm to fit
  under the drive case sides"). It passes 0.5 inside the skirts. The
  central joint's ridge shortens to 10.35 to stay inside it.
- **Righting at 143.5, wheelbase 223.5** (pose-aware scan): the narrow
  rear end slides past the skirts at 0.5, then the bridge's middle chamfer
  meets XC430 A (0.48 at 143.25).
- **The chassis is +-16 throughout** (`deck_half` 16, was 25 for the short
  piece ahead of the righting; user). The 32 came from the drive pulleys at
  deck height (|x| 17-34) as far forward as y ~121, and the Pi's carrier
  legs (|x| 17-28) now come down beside the deck too.

**Settled for now (2026-10-05, user):** the carrier on two screws, on a
diagonal, (11, 14) and (-11, -24.5) on the face (`post_mirror` false). The
centreline is avoided because of the tray nut's slot. "Happy with the
layout for now"; nothing printed in this setup. The righting will be
reworked against `cad_bike --fit righting` (one call: the pose-aware 0.5
scan and the front wheel, FITS or what binds), with the constraints in
`righting-design.md`, "Fitting the whole bike".

**Constraints, as found (kept current; user: "please keep noting things
that appear as constraints to you"):**

| what binds | where | number | what would free it |
|---|---|---|---|
| righting rear | the bridge's middle chamfer (4.6, held by the central screw's head seat) on XC430 A; the narrowed rear end passes the drive's skirts (\|x\| 12) at 0.5; every pose | rod >= 143.5 ahead of the rear axle | the central screw's head seat lower, or the screw elsewhere |
| wings vs drive pulley L | at 6 thick the inner face sat 0.5 inside the synthesis's 35 core | -- | fixed: panels 5, the synthesis's own width |
| righting front | front bulkhead to the straight front wheel, 5 (user) | front axle 80 ahead of the rod | the bulkhead's outline, or the user's 5 |
| wings vs front tyre | wing L's front end against the steer sweep, worst swung inboard | panel_back <= 43.1 at 80 (41 used) | shorter panels; a real steer limit (the ball is +-180 deg) |
| Pi plate on the face | the drive pulleys stand ~0.7 proud | pockets 1.7 deep, 2.9 of plate left | -- |
| Pi down the face | the board itself, over the deck | edge -69.5 (-71 touches) | -- |
| Pi plate's middle | the deck over the righting, +-16 | 1 over it | -- |
| carrier screws | two, on the block's face and the plate, off the tray nut's slot (x 0) | (11, 14), (-11, -24.5) | -- |
| carrier nuts | slots out of the block's sides, under the drive's case sides | -- | they go in first; the case-side screws are arbitrary (user) and may cross them |
| tray tongue nut | its slot leaves through the block's face, under the carrier | -- | assemble the tray first |
| Pi underside | 3 standoff: header and port pins, microSD socket | -- | measure the pins on the real board |
| AHRS height | the bike may end under the ~252 mm roll centre (user) | -- | how much the fore-aft place matters: the user's to work out |
| upper case chamfer | the central joint's ridge (8 meets it) | 7 used, the most | the joint further forward, which the lower case's undercut blocks |
| upper case's turned nut slot | the case's chamfer crosses its open end: a 6.55 bridge as printed (the check's one hanging edge) | -- | accepted |

**Where the switched-off parts are meant to go (user, 2026-10-05,
tentative):**
- the U2D2 and the power distribution board on the front chassis, near the
  steer servo;
- the AHRS further back, near the battery. The user recalls "high and far
  back, orientation unimportant". On record: the bike's roll centre
  measured ~252 mm above the ground (`ahrs-fixture.md`), above the AHRS's
  old ~180, so higher means less accelerometer lever arm. Its mount is
  part of the digest-pinned `DeployModel` (the AHRS mount), so a new
  orientation means re-exporting the bundle, not ignoring it. Nothing found
  for "far back";
- the power switch near the front/steer. Its geometry stays in
  `electronics.switch`: cutout 13 x 19.4, panel 0.75-3 (2 drawn), 18.5
  behind the panel, the bezel GUESS, and the tab code in `cad_bike` behind
  `underside`.

**Printed mass** is now estimated beside the solid figure, using
`skin x area + 15 % x the rest` with skin 0.9 mm (`cad_bike.printed_mass`).
UNCALIBRATED: weigh a printed part and fit `SKIN`.

## Wheelbase 200.5: the righting's servo turned (2026-10-07, user)

The righting's XC330 turned long end down, its bridge 7.1 lower (2 over
the crank), and the drive case sides' rear blocks chamfered
(`drive-design.md`, `case.rear_chamfer`). Each moved what binds at the
rear, so the placement was re-fitted:

| | was | now | how |
|---|---|---|---|
| `placement.righting_y` | 143.5 | **120.5** | `--fit righting`, 1 mm steps: the turned upper case passes between the case sides at 0.597, flat along Y; next binds wing L (+0.56 pose) on drive pulley L, 0.42 at 119.5 |
| `placement.wheelbase` | 223.5 | **200.5** | + the front's 80 (front bulkhead 5.25 from the straight wheel, unchanged) |
| deck top | 58.24 | 51.18 | the bridge |
| `electronics.edge` | -69.5 | **-44.5** | one probe, plate slid UP the face in 5 mm steps: the lower case and deck sit under the old spot; clear from +25. The plate is now one piece (no notch needed) |
| righting `chassis.joints_y` | +8 / -8 | +-8 again (late; +9.5 alone for an afternoon) | user: two nut slots, the chassis is a placeholder. -8 is under the wedge, its head buried: `cad_bike` NOTES it (it refused), and the wedge now stands on the deck's top so the righting plate keeps the grooves |

`deckPt`, how the chassis finds its body after the union, moved into the
wedge: mid-deck it fell into the -8 joint's screw hole
(CANNOT_RESOLVE_ENTITIES; a billed eval). The Pi's plate could probably come down again
between +20 and +25 (not resolved finer).

## As built

`python -m aow_sim.cad_bike` (config `config/bike_cad.yaml`), one feature
`AOW bike` (dialog: steer angle, righting pose, envelopes on/off, ghost
wings on/off) in the
Feature Studio **aow-bike-whole features**, inserted in the Part Studio
**aow-bike-whole** (`onshape.yaml` tabs `bike_features` / `bike_assembly`).
Renders: `docs/cad/bike.png`, `docs/cad/bike_right.png`, `docs/cad/bike_left.png`
(2026-09-30: they predate the fifth pass; the model is read in Onshape, renders only on request). Tests:
`pytest tests/test_cad_bike.py` (pure).

| new part (sixth pass) | print | ~g printed (15 %, 2 perimeters, uncalibrated) / g if solid |
|---|---|---|
| chassis | deck down | ~30 / 68.7 |
| battery tray | floor down | ~10 / 11.4 |
| electronics carrier | underside up | ~12.5 / 22.2 |

| `--check`, one call | result |
|---|---|
| bodies | 57 |
| interference across modules and new parts: at rest, steered 30 / 90 / 180 / 270 deg, righting at every pose | 0 |
| downward faces left | nut-slot roofs and edges (both post rows lean 45 deg as printed), the tray's strap slots, the righting joints' grooves |

Landmarks, bike frame, mm: wheelbase 223.5, righting rod at y 143.5; deck z
53.64-58.24; the carrier y 87-154, z 51.9-118; the floor -51.2.

## Things that cost a call (2026-09-30)

- **`getProperty` throws in a Part Studio during regeneration** and works
  in an eval, so `--check` passed and the inserted feature rendered blank.
  Every part is found by a point now. `build_fs` refuses `getProperty` in
  the studio. The modules' parts keep their own names in the studio, so
  "lower case", "upper case", "horn hub" and "X330 case" appear twice
  (steering and righting). The check prefixes them for its report.
- **A reserved word read as a map key** (`BK.switch`) parses as the
  keyword. `lint_fs` now catches it (it only saw declared names before).
- **FeatureScript's `match()` is anchored at both ends:** `"^floor"` does
  not match "floor (mock)".
- **An id's operations must be contiguous:** the pre-placement trims have
  their own parent id.
- **A part found by a point on a screw's axis also finds that joint's
  cutter**, which then cuts itself. `bkJoint` resolves its parts before
  making the cutter.
- **Tangent unions come back non-manifold:** a bar's round end on another
  bar of the same width, a boss tangent to a rail.

## Outstanding

**The state, plainly (user, 2026-09-30): a first assembly, and a mess in
places.** Known to change:

- **The underside electronics go back in** (U2D2, AHRS, power board,
  switch, the cables), somewhere other than under a carrier now 8 off the
  face. The cables are drawn for the Pi's USB edge DOWN; the generator
  refuses `underside` with `usb_edge: up` until they are re-routed. When
  they return, the drawn cables were minimal paths: the real ones are
  longer, and need not be modelled fully (user, 2026-09-30).
- **The righting's stack: the 6 mm plates are done; lowering it is not.**
- **The battery holder will likely be a different part or arrangement,**
  for example coming directly off the XC430 cases instead of a tray on the
  drive block's top. The tray here is a placeholder for where the pack sits.
- **The cage is off, and was extremely tentative** (spine and ribs); it
  may be replaced outright. Switched back on it does not clear the
  electronics at 235: it was laid out for 250.
- **The post screws go in before the Pi:** their heads sit under the board.
- **CoM not computed.**
- **Lowering the whole righting mechanism shortens the wheelbase more:**
  its binding corner tucks under the drive's 45 deg underside, so ~1 mm
  back per mm lower. That moves the wing pivot, so it belongs to the
  linkage optimisation (`righting-linkage-margin.md`). Kept as is for now
  (user).
- **Re-run `--fit 2`** after any righting or drive change (the digest NOTE
  says when): its probe excludes the deck plate, so its zero is the bridge's.
- **The sim still has wheelbase 200 and its own payload layout.** Nothing
  here reached `bike_params.yaml`, on purpose.
- **The USB-C ends are guesses:** the plug body (12 x 6.5 x 15), its hard
  stub (6), which side of the AHRS its connector is on, and the left
  stack's x on the Pi board (-28.5..-13.5). Measure them and the cables
  re-route themselves.
- **Device sizes:** the power board is `GUESS` (30 x 30 x 25). The AHRS
  and U2D2 mounting is cradles only; the plan is zip ties or VHB.
- **The drive's wires** exit the left case side's skirt at its +Z edge,
  under the battery tray's floor (which rests on the case sides). Not
  checked against a real harness.
- **Joints in the chassis reused from the placeholders** keep their
  modules' print orientation. The drive block's bores got teardrops for
  the chassis's deck-down print; its nut-slot bridging layers are cut for
  the block's own orientation, so expect cleanup there.
- **Finishing (chamfers, fillets, cosmetic cuts) goes through the modules,
  not `-whole`** (user, 2026-09-30). Hand-model it in a module's Part
  Studio, read it back with `cad_hand_edits`, Claude folds it into that
  generator, check, then roll into `-whole`. The steps are in
  `cad-onshape-workflow.md`, "Modules, the whole bike, and folding hand
  edits in". Hand edits made in `-whole` are a last step only: a re-push
  can orphan them.
- **Not printed.** The chassis is one part, about 135 mm long, now with
  four posts leaning 45 deg off the block's face as it prints. It could
  split at the righting plate.
