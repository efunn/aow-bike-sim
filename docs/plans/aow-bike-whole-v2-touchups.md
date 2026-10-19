# layout and CAD for whole bike (continued)

> **Status:** new document (2026-10-19)

New design instructions for aow-bike-whole-v2 (last work in: `aow-bike-whole-v2-complete.md`)

I made a bunch of new example geometry in `aow-bike-whole-v2`, but build everything new from scratch. I will do hand edits to be rolled into the modules after getting this round done. The purpose of my new stuff in here was to mock up how I'd like the, so you may want to reference some geometry/ideas, but fine to break things and I'll delete my add-ons afterwards.

## aow-bike-steering

- steer tube angle/rake increased to 20deg (keep wheelbase)
- lower+upper case attachment screw moves to the front +Y (makes room in the assembly, because the steer module tilts back)
  - note that the current undercut angle for the nut slot support is 45deg, but can cut under more (e.g. 20 or 70deg depending on your perspective) so the case doesn't get any bigger
- the pi/u2d2 carrier shouldn't be affected here; it might look like a clearance problem but try not to move it and just report the problem clearance if it comes up

## aow-bike-drive

- front and rear rails at 20deg now (front necessarily matching new steer tube angle, rear is just following the design)
- rollcage attachment to be detailed...

## rollcage

This can live in aow-bike-drive or aow-bike-whole-v2, whichever is more convenient (probably whole-v2); X-inboard/outboard refer to +/-X relative to the centerline; Y-inboard/outboard refer to +/-Y relative to the midpoint of the bike's wheelbase. The rollcage should be the same part at the front and rear, except rotated around Z, which is necessary to keep the handedness of the endcap bayonet mount attachment.

- rollcages are rings with a diameter equal to the maximum width of the wing blades at rest/stow (e.g. 137.4mm)
- center-Z of the rollcage is roughly at the height of the wing blades (nominally, 100mm above the origin/rear wheel center)
- not a full ring; nominally 270deg with an opening at the bottom
- 15mm in Y, extending Y-inboard from the wheelbase endpoints; ~10mm X-inboard from the max diameter (thicker, because of the bayonet mount described later)
- printed Y-outboard
- on the Y-inboard side, a v-groove for a flexible plastic piece (0.02in thk) that fits between the two rollcages (the groove extends almost the whole e.g. 270deg, but stops short of each end, so that the flexible plastic piece cannot fall out)
- on the Y-outboard side, a stepped bayonet mount extending Y-outboard from the base (5mm or half the X/thk, on the X-inboard part of the ring; I made it 10mm Y-outboard but can maybe be shorter, which also helps the end caps fit a bit easier)
  - mocked in my `rear-pill-ring` part; a v-groove with the female part on the rollcage
  - I drew 3 attach points with 15deg of travel/contact for the v-grooves (should clear the wheels to attach; a rotational problem around the Y-axis)
- half domes for the front and rear cover, both attaching clockwise
  - the underside of the dome should be a polygon/cone type shape to print better (this gets printed Y-outboard as well)
  - the dome is elliptical (e.g. 137.4mm diameter-X, but only 50mm height-Y), and starts its curve immediately Y-outboard from the main 15mm of the rollcage ring
- rollcage attachment to drive case:
  - rollcages slide on Y-inboard
  - I think the 6-32 screws attach Y-inboard (one on each +/-X of the bike for each of the front and rear rollcage); this breaks the rule of all attachments coming from the sides of the bike, but it seems necessary
    - note that if you're looking at my mocks, I tentatively drew it with the screws X-inboard, but decided it won't work well with the geom
  - a bar on each side of the ring extends X-inboard to the drive case
    - should be a fairly robust interface/locating feature here

## Round 1 (2026-10-19): the 20 deg rake, the steering's front screw, the rollcage

Pushed: `aow-bike-steering-v2`, `aow-bike-drive`, `aow-bike-whole-v2`. Every
`--check` clean:
- **Steering v2:** each part's extents as wanted; no interference at rest
  or steered; 0 hanging edges.
- **Drive:** no interference; 52 screw-hole probes, 0 wrong (39 open on the
  axes, 13 solid in the walls); left and right mirror images but for the
  left's ears.
- **Bike:** 84 bodies; no interference at rest, steered 30/90/180/270, at
  every righting pose, or with the Pi carrier lifted; the rings exactly the
  blades' width.

12 Onshape calls (85 -> 97). Two were wasted, both in
`cad-onshape-workflow.md` now: an oblique sketch again (`EXTRUDE_FAILED`)
and `opRevolve`'s `angleBack` (every ring, cap and lug came out 360 deg).
The user's mocks were read back with `cad_hand_edits bike_v2` (2 calls)
before any of it; they are left in place, to be deleted.

**Rake 20, CAD only.** `bike_params_cad.yaml` `bike.rake_deg` 20. The frozen
v1 reads a new `rake_deg_v1` 15, so `bike.fs` and `steering.fs` change text
only. The simulator's `bike_params.yaml` stays at 15 until the as-built pass
(status row 6). The rail's steering-side points, its steering joint (bike
y 174.66, z 71.04) and that groove (110 deg) are re-derived. A new point on
the rail's top edge at (150, 71.6) keeps 1.75 of wall round the Pi
carrier's low joint, which the lower 20 deg edge had cut to 0.56.

**Steering: the case screw at the front.** One 6-32 on the centreline
past the servo's far end:
- **Head:** in a lug off the upper case's front face (to module y 38.05,
  its top 45 deg).
- **Nut:** in a block off the lower case's front walls. The block's
  underside is 20 deg from level (`v2.nut_block_underside_deg`, user) and
  meets the front chamfer at module (18.4, 91.1).
- **Order:** the block goes on after the chamfer cut. Before it, the cut
  notched the block, leaving a 10.5 mm level edge hanging in the air; the
  check caught it.

The Pi carrier is untouched and nothing touches it. By sampling the
cases' boxes (not measured in Onshape) the upper case comes about 10 mm
from the Pi's GPIO header and 14 mm from its board.

**The rails' cage tongues** replace 10-16's cage posts. The ring prints
Y-outboard, so a bar's clamp face has to be a ceiling in it. The rail's
end inside the ring is therefore a tongue, and the bar's socket slides over
it Y-inboard. One 6-32 along Y per tongue: the head in the bar, the nut in
the tongue.

| | front | rear (the mirror) |
|---|---|---|
| tongue's face, bike y | 199.5 | 1.0 |
| tongue, bike z | 74.8 .. 83.5 | same |
| screw axis | \|x\| 16.475, z 79.52 | same |
| head | 4.6 out of the face (in the cap's bore) | same |
| inboard part, for the nut's 6.45 across X | \|x\| 12.35 .. 16.5, 7 long | the same, under the holder's panel (0.7) and beside the battery (1.35) |

- **Why there:** the front tongue's inboard part is ahead of the
  steering's foot, where X is free, and over the fork's top plane.
- **Nut slot:** 0.9 walls each side in X (two lines), 1.0 under it; it runs
  up out of the tongue's top, and the bar's top wall then keeps the nut in.
- **Battery stop:** now on the rear tongue's inboard part, drive z
  50.8..53.55 (was 42.5..50: the arm no longer reaches there).
- **Holder's aft groove:** opened 13 -> 18, out through the new post's top.

**The rollcage** (`bike_v2_cad.yaml` `rollcage:`, `cad_bike_v2.rollcage`):
one part at the rear, turned 180 deg about Z at the front.

| | |
|---|---|
| ring | OD 137.4 = the blades' widest at rest (\|x\| 68.70, measured by `--check`), centre z 100, 15 deep, 10 wall, 270 deg |
| bayonet base | the inner 5 of the wall, 10 out past the ring's face |
| grooves | V 2.5 deep, 8 wide, 45 deg; at 45, 165 and 285 deg from the bottom, clockwise seen from outboard; 15 travel, then a 3 stop |
| lugs (cap) | 14 deg, 0.2 off every face; the first goes in through the ring's opening, the others through 17 deg gaps in the base |
| sheet groove | V on the inboard face, r 66.2 (GUESS), 3 wide, 1.5 deep, stopping 5 deg short of the ring's ends |
| bars | a socket 0.15 round the tongue, 2 walls; inboard of the rail (the chin) the top wall ends 0.5 off the holder's panel and the bottom wall short of the steering's pads |
| cap | elliptical, 50 high, 270 deg; inside, a cone from the skirt's bore to 2.5 under the top, 59.6 deg off vertical as it prints (the user's sketch: about 60); wall 1.8 at least |
| volumes | ring 71.5 cm3, cap 154.8 cm3 (solid; the caps are thick in the middle, as the cone forces) |
| screw | 6-32 flat head, 3/8 in: 0.5 past the nut, and a 1/2 in would bottom in the hole |

Rejected:
- **X screws:** the user, from the mock.
- **A nut along Y in the rail's 4.6:** the nut is 6.45 across flats.
- **The bar's clamp face facing outboard,** the rail's lug outboard of it:
  printable, but the ring could not slide on past the lug.
- **The front tongue beside the steering's cup:** its nut walls came to 0.6.
- **A cross ridge to locate the ring:** the socket already locates X and Z.

Open:
- The power holder now comes out only after the rear ring. Its aft
  ridge's path crosses the left bar, as it did the old cage socket.
- The sheet is not modelled and its radius is a GUESS (it has cutouts
  for the blades in side view, user).
- The caps are open at the bottom like the rings; the mock's flat
  underside is not drawn, nor its stripes.
- Print-fit the bayonet (0.2), the socket (0.15), the cap's 60 deg cone and
  the chins' 45 deg ends.

## Round 2 (2026-10-19, the user's review): the holder's ear, the chins, the cap

The user: the weird geometry was driven by fitting round the power holder,
and the rollcage was not yet printable for it. The mocks are rolled back
in `aow-bike-whole-v2`, not deleted; the studio shows only the generated
feature. Pushed `aow-bike-steering-v2` and `aow-bike-whole-v2`. Both
`--check`s clean (the bike: 84 bodies, no interference anywhere, 52 hole
probes right). 5 calls (97 -> 101).

- **The power holder's panel** reached back to the aft nut's lug (drive y
  65.73) for its whole height; only the bottom needs it. Now an EAR there,
  up to the aft ridge's top + 1 (drive z 43.5), and above it the panel
  stops at the shelf's aft gusset (71.20): 5.47 off its upper part.
- **The bars' chins** (inboard of the rail, carrying the screws' heads)
  lose their two 45 deg wall ends. Each is now the bar's full height,
  stopping short of the bed on ONE 45 deg chamfer at its inboard corner,
  the edge along Z (user). It reaches y 8.44 at |x| 12.35 and 12.29 at
  16.2. The front pads bind (the bar's lowest z); the cut-back panel has
  room to spare.
- **The cap** is one ellipse (`skEllipse`), not six arcs.
- **The sheet's groove** is 1.0 wide at the face to a 0.5 flat end 3.0 deep
  (user), not a V.
- **The steering's front screw**: its hole runs on through the nut block's
  underside (1.5 past it), not stopping 0.2 short (user: "partway
  clipping").

Then (user): two more, pushed to `aow-bike-drive` and `aow-bike-whole-v2`,
both `--check`s clean. 4 calls (101 -> 105).
- **The rails' top edges at the tongues**: the notch between the band's top
  (square to the steering axis) and the post's edge is filled, the edge
  straight from the tongue's top corner to the band's top beside the pad,
  all four. The rear left's was where the holder's groove ran through. The
  steering joint's wall goes 1.50 -> 3.55.
- **Bridge hints** on each bar's socket ceiling, where the screw's hole
  leaves it (it is a bridge as the ring prints): a hole-wide strip across
  the span, then the hole's square, 0.2 each.
