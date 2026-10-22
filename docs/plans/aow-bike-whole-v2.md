# layout and CAD for drive module

> **Status:** new document (2026-10-11)

A new onshape document aow-bike-whole-v2

## sub-modules

### aow-bike-drive

update existing document

- for now, drop the fixture mock and the extensions to the case sides that meet with it
- roll in the chamfer/fillets that I manually added in `aow-bike-drive` (and then I can remove my manual edits); needs to be symmetric (I just added them to one side in my hand edit)

### aow-bike-righting

update existing document

- add latest blade design (curved spline shape)
- 'wing' design (the stub blade attaches to) will probably need a new shape/attach method to the blade

### aow-bike-steering-v2

new steering document

- flip the steer servo 180deg about Z (so the servo hangs out from instead of out back)
- single 6-32 csk attaches upper case to lower case (hanging out the front, similar to righting upper to lower attach)
- attachment of lower case to (tbd: rest of bike) is from the sides +/-X of the lower case

### remaining components

all free/floating, no case designs yet

- Pi 3B+
- AHRS
- u2d2
- power module
  - the envelope of this may change but ok to keep it a bit big/oversize for now (which I believe it currently is)
- battery

in general, can reposition these to find the best layout, then design case/routing around it; battery should still be in the rear/above rear wheel/drive case area; AHRS I think needs to be as rigid/close to the drive servos (seems to be the most structurally sound part of the whole assembly)

## Decisions (2026-10-11)

- **Tabs, not a new document** (user). Every v2 module and the v2 bike live
  as new tabs in the existing Onshape document, so inserted features follow
  their studios without re-pinning versions. Ids in `config/onshape.yaml`.
- **The righting's CAD blade is V3** (`config/swing_explore/blade_v3.yaml`).
  The sim stays on V2 for now (user: the CAD may yet change the blade).
- **Wing to blade, for now** (user): draw the stubs only (the countersunk
  screw "rectangle bits", not extending into the wing), and the blade as the
  whole external V3 shape. No design yet for how they attach.
- **Steering v2:** the servo hangs out the FRONT (turned 180 deg about Z).
- **Order:** drive, steering-v2, righting, then the v2 layout with the
  free-floating electronics.

## Drive (done 2026-10-11)

`python -m aow_sim.cad.drive --check --push drive_features`, the existing
`aow-bike-drive` tab.

- **Fixture mock off.** The case sides now end at the skirt's front wall
  (Y 118.9, was 142.9 with the tabs). The mock and the tabs are still in the
  code, behind the dialog's "Fixture mock" box (default off). v1's
  `cad.bike` builds them always, because its chassis grows from the block.
- **Your hand edits are in the generator,** on both chainstays. They were
  read back with `cad.hand_edits drive` (`traces/hand_edits/drive_hand_edits.txt`)
  and are set in `drive_cad.yaml` `chainstay:`:

  | edit | where | size |
  |---|---|---|
  | chamfer | front-outer corner (outer face x front face) | 15 |
  | chamfer | the riser's rear-top edge | 5 |
  | chamfer, adds material | a gusset in the concave corner, riser x bed | 5 |
  | fillet | both long edges of each of those three chamfer faces | R3 |
  | chamfer | the outer face's rim, round the boss and on along the side edges to the riser top (one tangent chain) | 2 |

- `--check`: 0 interference (at nominal and with the chainstays back 3),
  L/R the same part, 0 hanging edges. The two "flat crown" hits are the R3
  convex fillets, which the hole detector mistakes for holes.
- **To do (user):** delete the four hand features from `aow-bike-drive`.
  They now duplicate the generated ones and will error.
- `cad.hand_edits` now lists every face per feature. With only one example
  face, Chamfer 2's second edge (the concave gusset) was ambiguous.

## Steering v2 (done 2026-10-11)

`python -m aow_sim.cad.steering --v2 --check --push steering_v2_features`.
New tabs `aow-bike-steering-v2 features` and `aow-bike-steering-v2`, with
`AOW steering v2` inserted (SCREWS / CROSS). v1's tabs are untouched.
Config: `steering_cad.yaml` `v2:`.

- **Built in v1's frame and the case group turned 180 deg about Z** at the
  end. Exact: the fork, headset, horn hub and horn are all symmetric under
  the turn (the hub's M2s on the axes, the lugs at 45 deg). The fork, the
  headset and the axial stack are unchanged.
- **No mount plate,** and no mount pad on the upper case.
- **Upper case -> lower case: one 6-32 countersunk,** running down. The head
  is in a lug off the upper case's front face, 10.25 out, 10.5 wide. It sits
  4.6 above the joint plane (the upper case's underside), with a 45 deg top
  back to the case so it prints cap-down. The nut is in a block the lower
  case grows forward, the slot blind from the block's front face, its roof
  bridged in two layers.
- **Lower case -> bike: two 6-32 a side, from +-X.** Nuts in the lower case,
  the ridge along Y on the case (the house joint), slots open out of the
  bed. Mid-cup height (z 83.75). One joint just ahead of the turning cavity
  (y 13.8), one near the cup's front (y 31.6). The heads go in whatever the
  bike becomes: nothing is drawn there.
- **The cup runs forward to the lug's front face** (y 38.55) at full width,
  to carry those joints. That makes the lower case the heaviest part,
  41.7 g if solid (upper 13.3; superseded, see round 5). It can be pocketed or narrowed once the
  bike-side parts exist.
- `--check` (both variants): 0 interference at rest and steered
  30/90/180/270, 0 hanging edges. The side nut slots' tops are 3 mm
  bridges as printed. The insert reply said ERROR, which
  `onshape.insert_custom_feature` documents as normal on insert. **Not yet
  looked at in Onshape.**

## Righting (done 2026-10-11)

`python -m aow_sim.cad.righting --check --push righting_features`, the
existing `aow-bike-righting` tab.

- **`wing.blade: outline`** (new; `ends` was the stub plus U blade). Each
  wing is:
  - **two printed stubs,** "wing R stub (rocker)" and "wing R stub
    (knuckle)". Each is just the tab block a 6-32 passes through, 20 x 10 x
    6, with nothing of the panel. Each prints with its countersunk outer
    face up, so it has no overhangs and needs no teardrop;
  - **a dummy blade:** the blade file's whole section
    (`wing.blade_file` = `config/swing_explore/blade_v3.yaml`), extruded
    along the panel's span (+-41). The file's own `y:` span is the sim's
    (v1 mid-wheelbase) and is not read.
- **The stubs stand on the blade's inner face,** which is the panel's
  (V3's points 1-2 = V2's). A blade point (x, y) is module (x, y + floor_z),
  floor_z = -41.2. V2's foot midpoint lands on the panel's foot to 0.01.
  `layout()` refuses a blade file whose inner face is not the panel's.
- **The poses run to the blade file's own stroke,** 116.9 for V3 (its
  level point), not the linkage's 129.3 (V2's level). At 129.3, V3 was
  through the mock floor at both full strokes, the only interference
  `--check` found.
- `--check`: 0 interference at rest and over 10 poses; L/R stubs and
  blades are twins; 0 hanging edges, 0 flat crowns. ~59 g printed ASA,
  blades not counted.
- **v1's `cad.bike` is frozen on `ends` and the linkage stroke**
  (`cad.bike.righting_v1`). V3's toe reaches v1's front tyre sweep at
  wheelbase 200.5, and v1 is not being re-fitted.
- **Side effect, not acted on:** `analysis/swing_stepthrough.py` reads the
  CAD's blade as its default design, so a re-run now sweeps V3 there. Its
  label still says "the CAD's dummy U blade". Left alone, since it is the
  sim-side study.
- **Not looked at in Onshape yet.** Any hand edits on the old wing in
  `aow-bike-righting` will have lost their geometry.

## The v2 bike (first layout, 2026-10-11)

`python -m aow_sim.cad.bike_v2 --check --push bike_v2_features`. New tabs
`aow-bike-whole-v2 features` and `aow-bike-whole-v2`, with `AOW bike v2`
inserted (steer, righting pose and envelopes on/off in the dialog). Config:
`config/bike_v2_cad.yaml`; tests: `tests/test_cad_bike_v2.py` (`cad`).

- **Composition as v1** (`cad.bike.merge_layers`, the same transforms),
  built from scratch with none of v1's chassis, tray, carrier, cage, fit
  rules or digest: drive (no fixture), steering v2, righting (V3). The
  righting's floor mock and chassis-plate placeholder are dropped.
- **Placement:** the righting's rod at 120.5 (v1's fit, carried over), the
  front axle 86 ahead of it, **wheelbase 206.5** (v1 200.5). 86 is the least
  that keeps the V3 blades 2 clear of the front tyre's steer sweep at every
  pose (2.3 spare). With v1's 80 the toe reached it.
- **The electronics are envelopes, each placed by a centre in the bike or
  the drive frame** (the drive's is 45 deg up), with an optional tilt about
  X. A first layout to move from:

  | | frame | where | bike-frame box (y, z) |
  |---|---|---|---|
  | battery 22 x 58 x 30 | drive | on the drive's back, on the case sides (v1's spot, no tray) | y -1.9..60.3, z 62.8..125.0 |
  | AHRS 34 x 12.6 x 40 | drive | flat on the drive's front face, between the servos | y 70.3..107.5, z 70.3..107.5 |
  | Pi 3B+ 56 x 20 x 85 | drive | parallel to the face, past the pulleys' rims | y 72.5..146.7, z 72.5..146.7 |
  | U2D2 18 x 48 x 14.9 | drive | on the battery | y 8.3..52.8, z 105.6..150.0 |
  | power board 30 x 30 x 25 (oversize, GUESS) | drive | on the battery, behind the U2D2 | y -20.7..18.2, z 83.6..122.5 |

- `--check`: no interference between modules and envelopes at rest,
  steered 30/90/180/270, or with the righting at every pose. Each envelope
  sits exactly where its config puts it.
- **Moving things:** edit `components:` and re-run (check 1 + push 1).
  Or move a part in the browser with a Transform after the feature (free),
  and say where it went. A read-back (`cad.hand_edits bike_v2`, 2 calls)
  gives the transform's values.

## Round 2 (2026-10-11 pm, user's review)

**Blades centred on the wheelbase; the stubs stay put.**
- New `wing.blade_y` in `righting_cad.yaml` sets both blades' span:
  [-61.25, 20.75], which is mid-wheelbase 100.25 with the rod at 120.5.
- The left blade is now the right one **mirrored** across the midline. A
  180 deg turn would have put it the other way along Y.
- `cad.bike_v2` refuses a wheelbase the blades are not centred on, and
  says the span to use.

**Wheelbase back to 200.5** (front 80, v1's). Centred, the blades' front
ends sit 20.75 ahead of the rod, not 41, so they have 16.5 spare to the
front tyre's sweep and no longer set the front.

**The blades are the real spline.**
- `params.blade_segments` returns a blade file's outline as lines and
  cubic Beziers. `blade_points` now flattens from it and is bit-identical
  for V2 and V3, so the plant digest is unaffected.
- The CAD blade prism carries the exact segments, and `rgPrim` sketches
  the curve with `skBezier`. Each blade's volume went from 107978.0 to
  107996.3 mm3: the curve bulging just past its 40 chords.

**Steering v2:**

| | was | now |
|---|---|---|
| lower case -> bike | 2 x 6-32 a side, mid-cup, slots out of the bed | **1 a side, on the axis (y 0) at the bushing's mid-height (z 75.55)**, pointing at the steerer shaft; the nut slot runs **forward out of the case's front** |
| side faces | the cup's | a **3.5 pad** a side, from just behind the ridge to the front. Without it the clearance hole came 0.85 from the bushing bore; now 1.35 |
| front-bottom corner | square | **26 mm 45 deg chamfer**, full width; leaves 2.0 under the upper case's nut |
| lower case, if solid (superseded, round 5) | 41.7 g | 35.3 g |

- `--check`: 0 interference, 0 hanging edges, 0 flat crowns.
- The side nut slots' roofs are 3 mm bridges as printed (82 mm2 each, flagged
  as downward faces).

**Electronics:** nothing sits on the battery now. The user: "battery needs
to be able to come in and out easily".
- **U2D2:** on edge between the AHRS and the Pi, between the pulleys
  (drive frame y 133-148).
- **Pi:** moved forward 6 mm to make room (drive y 151-171).
- **Power board:** low, under the Pi's lower edge and over the righting
  (bike y 122-152, z 48-73).
- The user is not settled on where the U2D2 and power board go. The AHRS
  stays where it is (user: "looks good").

`cad.bike_v2 --check`: no interference at rest, steered or at any pose.

## Round 3 (2026-10-11 evening, user's review)

**Steering v2:**

| | was | now |
|---|---|---|
| side pads | 3.5, behind the ridge to the front | **5.0** (each bike nut slot's inner face on the cup's original side), **the cup's whole length** |
| upper -> lower | one 6-32 through a lug off the front, the nut in a block under it | **two, one a side** (x +-17.2): each head in a full-height lug off the upper case's side (flush with the cap, so it prints cap-down from the bed), the lug's front face **on the connector window's aft edge**; each nut in a block on the lower case's pad, its overhang a 45 deg underside, the slot out of the side. The screw sits 1 outside the nest bore so the head's pocket keeps a wall |
| the front | 26 mm 45 deg chamfer | **the foot (12.55 ahead of the axis), straight up 12.7, then 45 deg forward** to the case's front edge |
| lower / upper case, if solid (superseded, round 5) | 35.3 / 13.3 g | 31.5 / 15.2 g |

- The undercut meets the front edge at the underside of the servo shell's
  cap, 4 under the horn face. Run to the walls' bottom first, it left the
  cap's last 3 mm hanging: a 74 mm2 level face, which `--check` caught.
- `--check`: 0 interference, 0 hanging edges, 0 flat crowns. Downward faces
  left: the nut slots' roofs, the bike joints' 3 mm bridges and the upper
  joints' bridged layers.

**The Pi in detail:** the board with its four M2.5 holes, the USB +
Ethernet block (ports 2 proud of the board's end, up the drive's slope)
and the GPIO header. v1's numbers, now in `bike_v2_cad.yaml` `pi:`.
Each piece is checked to sit where the config puts it.

**U2D2:** "in parallel with the steering" (user, tentative). A new `steer`
frame for components; the U2D2's long side runs along the raked steering
axis, centred just aft of the steering cases (module z 76-124). Its
connectors are on one end and one long side.

**Power board:** on the drive's front face with the AHRS (user). The face
is 59.5 tall, so the AHRS slid to its lower part (drive z -29.75..10.25),
and the power board sits above it. It overhangs the face's upper edge by
11, forward of the battery and not over it.

`cad.bike_v2 --check`: no interference at rest, steered or at any pose.

## Round 4 (2026-10-11 late): the upper <-> lower joints shortened

| | was | now |
|---|---|---|
| upper -> lower screws | x +-17.2, the head's pocket cutting the upper case's side (a 10 counterbore down the lug) | **x +-18.3**: the pocket 0.5 off the upper case's side face, never in it |
| upper case lugs | full height, flush with the cap | **short**: the outer face up to the head (z 108.55), then 45 deg back to the case's side (it prints cap-down) |
| lower case blocks | from the side pad up, 45 deg past the pad | **the cup's side straight up to z 86.7, then 45 deg out** to the block, 1 under its nut slot |
| the front's vertical | to z 80.95 (the undercut met the shell cap's underside) | **to z 85.25**: as high as leaves 2 of floor under the servo's front end. The cut now follows the servo shell, so it takes the shell's cap with it and nothing hangs |
| lower / upper case, if solid (superseded, round 5) | 31.5 / 15.2 g | 27.2 / 14.0 g |

`--check`: 0 interference, 0 hanging edges, 0 flat crowns. The upper
case's bed is back to 997.9 mm2. `cad.bike_v2 --check`: clean.

## Round 5: printed mass, not solid (2026-10-11 late)

The steering's check reported **solid PLA**. Nothing here prints in PLA or
solid (user: ASA, 15 % infill). It now uses `cad.servo_mount.printed_mass`,
as the righting's does: a 0.9 skin over the whole surface, 15 % infill
inside, ASA 1.07 g/cm3, UNCALIBRATED. The check records each part's
surface area for it. The solid figures in the rounds above are left as
written and marked superseded; they compare only with each other.

| steering v2, ~printed ASA | g |
|---|---|
| lower case | **13.2** |
| upper case | **8.4** |
| fork R / L | 5.5 each |
| headset lower | 5.0 |
| headset upper | 1.3 |
| horn hub | 1.0 (SCREWS), 1.3 (PINS) |

Still on solid PLA, unchanged: `cad.drop_rig`, `cad.x330_fixture`.

## Outstanding

- **Delete the drive's four hand features** in `aow-bike-drive`.
- **The layout itself is still a first guess.** The Pi stands at 45 deg
  in front of the drive's face. CoM is not computed yet: two
  envelope masses are unknown (AHRS, U2D2 unweighed, power board a guess).
- **The U2D2 and power board places are open** (user). The battery's
  removal path is clear of them, but not designed.
- **Wing to blade:** no attachment designed (user: not yet).
- **The sim is still V2 and wheelbase 200,** as decided. The CAD's
  wheelbase is 200.5 again.
