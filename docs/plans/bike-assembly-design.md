# The whole bike in CAD

> **Status:** new document (2026-09-30). A first assembly, rough in places
> (user): four passes in one day took the righting under the drive and the
> electronics onto the drive's front face, wheelbase 300 -> 250.
> Interference-free, nothing printed. The righting's stack, the
> electronics' height, the battery holder and the cage are all expected to
> change: see Outstanding.

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

**Printed mass** is now estimated beside the solid figure, using
`skin x area + 15 % x the rest` with skin 0.9 mm (`cad_bike.printed_mass`).
UNCALIBRATED: weigh a printed part and fit `SKIN`.

## As built

`python -m aow_sim.cad_bike` (config `config/bike_cad.yaml`), one feature
`AOW bike` (dialog: steer angle, righting pose, envelopes on/off) in the
Feature Studio **aow-bike-whole features**, inserted in the Part Studio
**aow-bike-whole** (`onshape.yaml` tabs `bike_features` / `bike_assembly`).
Renders: `docs/cad/bike.png`, `docs/cad/bike_right.png`, `docs/cad/bike_left.png`. Tests:
`pytest tests/test_cad_bike.py` (pure).

| new part (second layout) | print | g solid / ~g printed (15 %, 2 perimeters, uncalibrated) |
|---|---|---|
| chassis | deck down | 76.6 / ~34 |
| battery tray | floor down | 11.4 / ~10 |
| electronics carrier | underside up | 24.4 / ~19 |
| cage spine | on its side | 17.6 / ~10 |
| cage rib x 3 | flat | 3.0 / ~2.3 each |

| `--check`, one call | result |
|---|---|
| bodies | 67 |
| interference across modules and new parts: at rest, steered 30 / 90 / 180 / 270 deg, righting at every pose | 0 |
| downward faces left | nut-slot roofs and edges (posts, cage boss), the tray's strap slots, the switch cutout's top, the righting joints' grooves |

Landmarks, bike frame, mm: deck z 53.64-58.24; the electronics' top ~166
(y ~85); cage rail 173.9 to y 90, then down to the steering foot at
(172.6, 118); ribs at y 0 / 40 / 80 to z 183; the floor -51.2.

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
places.** These four are known to change:

- **The righting's stack will shrink.** The wing joint moves outboard (the
  web's 6 and the hub zone's 8 are self-imposed by where it sits). The
  mechanism may also be lowered. Both shorten the wheelbase; see below.
- **The electronics can still go further down.** The cables set the limit
  now, and some more jigging of their route gains room. The drawn cables
  are also somewhat unrealistic: the real ones are longer than these
  minimal paths and will be routed with slack. No need to model them fully;
  the drawn paths only show the plugs, stubs and bend radii fit.
- **The battery holder will likely be a different part or arrangement,**
  for example coming directly off the XC430 cases instead of a tray on the
  drive block's top. The tray here is a placeholder for where the pack sits.
- **The cage spine and ribs are extremely tentative** and may be replaced
  outright. They show that something can span the electronics and anchor at
  both ends, not what the roll cage will be.

- **The bike is taller now.** The cage top moved from z ~145 to ~184
  (196 -> 235 above the floor), because the leaning electronics reach
  ~166. The CoM rises with them; not computed.
- **The carrier hangs on one row of posts and the cage foot** (fourth
  pass). The lower ~55 mm cantilevers. A lower support wants the devices'
  final places first.
- **The righting's axial stack could be thinner** (user, 2026-09-30).
  The bulkhead (10, the journal's bearing length) and the couplers (6)
  stay. The web's 6 and the hub zone's 8 are SELF-IMPOSED by where the
  wing joint was put (user's catch):
  - the wing's rocker-side tab sits in the web plane, which forces web >=
    tab (6, for the nut);
  - its boss and countersunk head sit in the hub zone.

  The wing is outboard (|x| >= 30) of everything near the rod except its
  own rocker. So the joint can move to the boss's far side, past the hub
  zone in Y, beside the bulkhead and clear of it (the bulkhead is +-18). The
  coupler plane never sees the tab then, and both layers only need what the
  rocker and crank need: the arm's strength and a crank shoulder. The
  rocker's 14 mm sleeve on the rod is not needed either; the knuckle shares
  the wing. Not tried; see `righting-design.md`.
- **Lowering the whole righting mechanism shortens the wheelbase more:**
  its binding corner tucks under the drive's 45 deg underside, so ~1 mm
  back per mm lower. That moves the wing pivot, so it belongs to the
  linkage optimisation (`righting-linkage-margin.md`). Kept as is for now
  (user).
- **Re-run `--fit 2`** after any righting or drive change: its probe
  excludes the deck plate, so its zero is the bridge's.
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
