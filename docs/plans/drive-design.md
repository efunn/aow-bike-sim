# layout and CAD for drive module

> **Status:** new document (2026-09-28)

3D printed design for the differential drive module of the bike. We need a mockup of the rear wheel, and several 3D printed parts which can later be incorporated into an overall model of the bike.

NOTE: I already hand-drew a version of this (printed, and working, in `aow-bike`, also the same as in `aow-bike-rig`), including a mock rear wheel. So this process will be a combination of assessing my previous drawing and making everything work from scratch.

## mock rear wheel
- in `rear-wheel` folder on `aow-bike/aow-bike` on onshape. Doesn't need to be this fancy, but cool to see all the rollers/case.
- missing from my mock: the spline shape comes from a derived `rear-spline` in `https://cad.onshape.com/documents/d9c429f0be6f95c4c07be5c2/w/4f842deec1234995214fe4f3/e/9339074becf1cea1b1f85d48` `aow-spline-shape` where I tested and printed various splines to fit the HC-802's rear wheel. Configuration 5 was the best I found

## parts

Orientation and printing directions: Z-up, Y-forward X-right. Y-forward is the mean belt direction; in the bike assembly Y is tilted up by the drive assembly orientation (e.g. 45deg)

- same as my previous drawing:
    - rear wheel spline+pulley combination part printed X+/X-, inward (I used an existing HTD generator, but probably trivial for you to include/create your own version). I used a cone for the lateral bearing surface here
    - drive pulley (uses 4 screws onto the XC430 horn with 4 localizing pins); can create your own HTD generator to replace this
- new part layout: (drive casees and chainstay were integrated in my existing design, but should be separate parts now)
    - drive case sides printed X+/X-, inward (should be identical parts on each side, as rotated 180 deg from one side to the other should fit): integrated spacers for the XC430 case attachment and a slot for the wires to come out from one side (both already in my previous design, wire slot does not need to come straight out like the XC330 did). Coming out the back side (-Y) are slots for the chainstays to fit into. Let's try a single screw (so one screw on each side of the bike); this should be a short slot feature with the screw hole, allowing for belt tension adjustment; we'll use the same 3/8" countersunk 6-32s as the rest of the assembly.
    - chainstays printed X+/X-, inward (should be identical). I used a stepped chamfer for the bearing surface here. I'm using the hex and 5mm screw from the HC-802, whose dimensions can be derived from my drawings (but lmk if you need the measurements). A feature extends (in the X-direction) to meet the drive case side, with the locating feature to match the drive case
    - something will also need to extend off the drive case sides for screwing into a test fixture or the rest of the bike, and including some locating features (probably in +Y?)

## integration into whole bike

- in its own feature studio and part studio `aow-bike-drive` inside `aow-bike`
- eventually, inserted into a larger part studio for the whole bike
- for the separate assembly, designed around Y-forward, but will sit at an angle (nominally, I think 45deg?) in the larger part studio

## Read from the hand drawing (2026-09-28)

Three calls against the `bike` tab (raw output in `traces/drive/`): every
cylinder, cone and plane of `spline-rear-wheel`, `drive-pulley`,
`drive-case-lh` and `rear-wheel-mock`. Everything that printed and worked is
carried into `config/drive_cad.yaml` as `measured (hand drawing)`:

- **Spline pulley:** 15T, 10 mm tooth band, flanges with a 55° cone, a 45°
  chamfer from the inner flange to the Φ15 hub, the 5-lobe spline stub
  (config 5: Φ11.8 / Φ8.8 / 35° lobes, the variables to tune), Φ5.2 bore, a
  60° thrust cone opening into a Φ15 counterbore.
- **Drive pulley:** 45T; a Φ20.5 pad 1.6 proud onto the horn; 4 Φ1.4 × 2
  pins on the axes of the PCD 16; 4 M2 × 6 on the diagonals, Φ4 counterbores
  from outside, two bridge layers.
- **Case:** 3.6 plate, a 5 mm skirt (1.21 wall, 0.04 fit) round the servo
  pair, 8 spacers Φ4.3 × 2.8 into the XC430's own screw counterbores, 8 M2.5,
  a Φ22.1 horn relief, a 10.6 mm wire gap in the skirt.
- **Chainstay:** 5 thick outboard of the pulley (1.0 gap), Φ19 boss, 8 AF
  hex pocket to the clamp face at 25.25 (50.5 head to nut, user), Φ14 stub,
  35° relief cone, 60° bearing cone.
- **The groove:** the drawing's HTD generator has a root arc, two flank arcs
  with their centres crossed over the centreline, and 0.43 tip fillets. It
  is reproduced to 0.01 mm per tooth count (`teeth_15`, `teeth_45`).

One reading worth knowing: the drawing's chainstay cone sits **0.14 mm into**
the pulley's (apexes at 17.78 and 17.92). That is the preload, and it is
kept (`cone_preload`). The chainstay's X is set by the case side, the tongue
clamped there as the one-piece part was, so the M5 seats the cones by
flexing the arm. `--check` lists the overlap as intended.

## Decisions (2026-09-28)

- **Every part is built on the left, and the right is the same solid turned
  180° about Y.** That turn maps servo A (horn −X, below the belt line) onto
  servo B (horn +X, above it), each side's horn relief onto the other's, the
  wire gap onto the other side, and each belt onto the other. So the pairs are
  one part by construction, and `--check` confirms the volumes match.
- **The belt plane is set from the servo side** (horn face + pad + 0.6 flange
  gap). The spline pulley is checked to land its teeth in the same band.
- **Tension joint (user):**
  - The chainstay's tongue slides along Y in a channel at the case side's −Y
    edge, open behind and closed in front.
  - At nominal (the exact belt length, which is what drove well) the tongue is
    on the channel's end. More tension means sliding it back and shimming that
    gap.
  - One 6-32 × 3/8 flat head, head outboard: it has to be reachable with the
    other side on. It is driven through a Φ8.1 bore from the chainstay's
    outer face, and its countersink sits in the riser, 0.8 short of the clamp
    face.
  - **What clamps (user): the riser's big flat on the case side's outer
    face.** The tongue is a plain rectangular key, 7 wide and 2.5 deep. It
    locates in Z and against rotation (a V would cam out), and it stops 0.3
    short of the channel's floor so it never takes the clamp. The channel's
    ceiling is a 7.3 mm bridge, down from 14.3.
  - Every ceiling on the case side is on the 0.2 layer grid, counted from
    its bed (the outer face):
    - channel 2.4;
    - nut slot 3.8–6.8, so the web is 1.4;
    - tip slot to 7.4, only 0.6 deep (user);
    - roof 8.4.

    The nut bears on the web when tight, which leaves the tip 0.45 past it.
  - Where a slot passes through a ceiling (the screw slot through the
    channel's, the tip slot through the nut slot's), that ceiling gets a
    counterbore's two layers. Layer 1 is cut across the ceiling's whole span
    for the slot's length; layer 2 is the slot itself, bridged along Y.
  - On the chainstay, the hex pocket's floor snaps to the grid (15.0 up from
    its outer face). The clamp span is 50.4, not the measured 50.5, which is
    within the M5's adjustment. The head bore's ring is on the grid too, and
    the countersink web comes out at 0.95.
  - The nut slides in a slot along Y, entered from the case side's back
    edge. The screw's slot runs through the web into it.
  - The travel is 3 mm. That is what fits between the wheel and the 45T's
    flange; the user said whatever is available is fine.
- **The chainstay crosses the belt plane inside the loop.** The riser sits
  2.77 mm inside the belt's tooth side and 2 mm behind the 45T's flange.
- **Fixture (user): 2 × 6-32 a side, with a ridge along Y as the locator,**
  into a mock block between the case sides' +Y tabs. The heads sit at
  Z = ±23, 0.67 mm outside the drive pulley's flange, so a driver reaches them
  with the pulley on. The block is a placeholder, to see where the real part
  goes.
- **M2.5 counterbores:**
  - Each counterbore's first bridge layer is a hole-wide strip clipped to the
    counterbore's own circle, so its ends are the counterbore's arc. A
    rectangle either cut past the circle or stopped short of it (`cbLayers`,
    also used on the drive pulley's M2s).
  - The two screws next to the horn get their counterbore run on into the
    relief as a slot aimed at the horn's centre, with the layers turned to
    match, as the hand drawing does. That avoids the sliver.
- **Wires:** they leave the back face, kink 90° against the flat plate, and
  run out through the skirt's gap: +Z on the left, −Z on the right.
- **45° nominal tilt** in the bike (user). The module is drawn with Y along
  the belt; the whole-bike studio tilts it.

## As built (`aow_sim.cad_drive`)

| along X, each side | mm |
|---|---|
| wheel hub face / spline pulley shoulder | 15.00 |
| horn face / case side outer face | 19.00 / 20.60 |
| pulley inner flange / tooth band / outer face | 21.20 / 22.70–32.70 / 34.20 |
| M5 clamp face (hex pocket floor) | 25.25 |
| chainstay | 35.20–40.20 |

| along Y | mm |
|---|---|
| belt centres (45T/15T, 370) | 107.35 |
| case rear edge (tire 51.2 + 3) | 54.20 |
| riser / tension screw / channel end | 47.52 / 61.23 / 67.52 |
| servos | 71.15–117.65 |
| fixture tab | 118.90–142.90 |

`--check`, one call: 0 interference at nominal and with the axle group moved
back the full 3 mm (the cone preload listed as intended). The L/R pairs have
identical volumes. No flat-crowned holes and no hanging edges. The remaining
downward faces are intended bridges:
- the counterbore layers and the hex floor;
- the channel's 7.3 mm top, the nut slot's roof and the tip slot's roof;
- the flange lips over the grooves (as in the hand drawing).

## Rear blocks chamfered for the righting (2026-10-07, user)

"The drive cases corners in -Y (the part going towards the chainstays)
could be cut down/chamfered." `case.rear_chamfer` 16.9 cuts the rear
block's +-Z corners at 45 deg, from the -Y end to where the servo pocket
starts. That leaves the end 12.85 tall of 29.75. It holds only the
chainstay's channel and the tension screw and nut, within +-3.7 of
mid-height. In the bike, the righting's turned upper case had bound on case
side R's lower corner there. With the chamfer, nothing bound within 19 mm.
`--check`: 0 interference, L/R the same part, 0 hanging edges.
The case sides printed from the hand drawing are square.

## Outstanding

- **The fits the drawing did not have:**
  - tongue clearance 0.15 per side (`GUESS`);
  - the XC430 envelope's counterbore and pin-hole sizes (`GUESS`). These only
    make the check honest; the spacers themselves are the drawing's.
- **Not printed yet, deliberately (user, 2026-09-28).** The steer module
  prints first, and it shares ideas with this one: the screw joints, the
  counterbore bridging, the fits. Its results decide the fit judgements
  here before any of these parts are printed.
- **The fixture mock is TBD and will likely move,** and so will how the case
  sides attach to it: the +Y tabs, 2 x 6-32 a side, the Y ridge. The screw
  count per side (1 or 2) is for the user to try.
- **The hex axle arrangement may change (user).** A different nut would need
  room for a small socket down the chainstay's hex pocket. Not critical yet.
- **Not yet inserted into a whole-bike studio.**
