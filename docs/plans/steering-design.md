# layout and CAD for steering module

> **Status:** new document (2026-09-28)

3D printed design for the steering component of the bike. We need a mockup of the front wheel, and several 3D printed parts which can later be incorporated into an overall model of the bike.

## mock front wheel

- tire diameter known
- tire params: `tire-width = 24mm` `tire-flat-width = 10mm`, `tire-min-diameter = 71mm`
- wheel hub params: `axle-diameter = 4mm`, `hub-width = 13mm`; the hub diameter is 4 numbers, as it is not symmetrical: one side: `hub-max-dia-side-A = 12.5mm`, `hub-insert-dia-side-A = 9mm` `hub-max-dia-side-B = 11.5mm`, `hub-insert-dia-side_B = 6.5mm`
- axle params (metal rod from original bike): `axle-diameter = 4mm` (nominal), `axle-length = 46mm`, `axle-head-height = 1mm` `axle-head-dia = 7mm`, `axle-head-knurl-from-end = 11mm`. Under the head, some knurling exists to help grip the plastic.

## parts

Orientation and printing directions: Z-up, Y-forward X-right. Z-up is the steering axis; in the bike assembly, the Z-axis will be tilted by the steerer angle.

- fork halves printed X+/-, inward towards the wheel. Above the wheel, countersunk for 6-32 holes and a locating feature to attach to the headset lower half. Only asymmetric (possible) is related to the wheel-axle shaft features (the hub and insert are roughly flush but not perfect, and the metal axle piece has a head and knurling to keep it in place)
- headset lower piece printed Z+. Nut slots and locating features for the fork halves. Locating feature between fork halves/headset lower should take into account the loading path from ground to headset. This part extends all the way through the headset bearing piece, with a stepped surface and a locating feature at the top for the headset upper piece. A central screw hole, with the screw extending from the bottom all the way through; not 100% sure what screw size/length is required here. I have 6-32, 8-32, and 10-32 screws in various lengths, but no particularly long 6-32 or 8-32. May be possible to have the screw pretty far up pocketed and then screw length can be artibrarily chosen from available hardware.
- headset upper piece printed Z+ with a nut slot for the headset central screw; at the top, a spline/screwdriver feature that will interface with an inverse feature attached to the servo horn
- lower servo case + bearing printed Z+. The bearing that the headset lower and uppers are sandwiched around. Includes a 6-32 nut slot and locating feature in Y- for attachment to a test fixture or the bike chassis (TBD).
- upper servo case printed Z-. Holds the servo in place. Includes a 6-32 nut slot and locating feature in Y- similar to the lower servo case. Unlike the previous idler setups, the upper servo case can cover the entire back of the servo and potentially wrap-around.
- servo horn hub printed Z-. Flush to the servo horn, 4 screw holes for PHS M2 6mm self-tap screws, and the matching spline/screwdrive feature from headset upper piece. Ideally could also print Z+ with the 1.5mm pins for testing setups; this tends to constrain the spline/screwdriver feature (hub needs to be the 'negative' side and cares about overhangs)

## integration into whole bike

- in its own feature studio and part studio `aow-bike-steering` inside `aow-bike`
- eventually, inserted into a larger part studio for the whole bike
- for the separate assembly, designed around Z-up, but will sit at steer angle (15deg, nominally) in the larger part studio
---

## Decisions (2026-09-28, four rounds)

- **No ball bearing.** The toy has none; the headset turns in a **printed
  plain bushing** in the lower servo case (Φ14 shaft, 0.2 diametral, `GUESS`
  -- ream/turn to fit). A metal shaft stays off the table: it would need
  keying to align the two headset halves.
- **Height relaxed:** `steering.servo_clearance` 40 -> **45** (bike_params_cad;
  `cad_layout` regenerated, the steer servo's derived `pos` pasted back). It
  buys a 14.6 mm bushing and a 12 mm fork block.
- **Axle:** knurled end in the +X leg (the head's side), the other end in
  the -X leg, **one Φ3.9 press-fit hole in both** (`GUESS`) -- so the **two
  fork halves are the same part**: print two, no labels (`--check` confirms
  their volumes match). Shank leg face to leg face (45 mm), the 1 mm head
  proud on +X. Both bosses **Φ6** on the hub, under the smaller insert (6.5),
  then **flared at 45 deg** back to the leg's width -- the same contact,
  more material behind it; prints narrowing upward. Legs **15 mm** wide in Y.
- **Tire OD 102.5 mm** (measured, max over the turn), CAD only -- see
  Outstanding.
- **Fork -> headset lower:** the block is **wider than the gap between the
  legs** (±17.5 vs ±13.5); above the tire each fork half steps out, and the
  step is a 4 mm **ledge the block's underside sits on** -- the vertical load
  goes in in bearing and it locates Z. **Cheeks** off the fork plate close
  round the block's ±Y faces (0.1 clearance, 3 thick) and locate Y, so the
  fork joint has **no ridge** (a second Y locator would fight them): three
  sides, Z- and ±Y. **One 6-32 a side** clamps X. Wider also moved the nut
  slots off the thrust boss.
- **Central screw from BELOW:** a 6-32 x 3/8 flat head up a Φ7.9 bore in the
  headset lower, seated in the shaft just under the step, into a **hex nut
  dropped into the headset upper from the top**. Reaching it means unscrewing
  the fork sides -- about the same labour as taking the servo off, which a
  screw from the top needs.
- **Headset upper -> lower:** a double-D spigot, now narrower (Φ11, **6
  across the flats** -- 1.2 each side of the screw hole), in a socket 0.3
  deeper so the upper bears on the step. **`upper_socket_trim`** is an
  upper-only tuning knob: it raises the upper's socket ceiling (and its
  bridging) and moves nothing else; past 0.3 it lands on the spigot top and
  preloads it, taking the web's bending out of the clamp. The step itself
  (`socket_extra_depth`) still moves both parts, for when the whole joint is
  off. The socket's ceiling is **bridged in two steps round the screw hole**
  (layer 1 a hole-wide strip across the flats, layer 2 the hole's square).
  Step 0.2 above the bushing (end play); the upper's skirt is the lift-off
  retainer.
- **Coupling:** the hub's **cross socket** runs from a centre island
  (r 2.8) **out through the rim**; the upper's lugs run from 1.0 outside the
  nut pocket to the upper's rim. `CROSS` (4 lugs) or `FLAT` (2) in the
  dialog. Not a full cross: the PINS hub would hang its middle.
- **Horn hub, walls >= 1.0 mm (two perimeters), set by the SCREWS version:**
  counterbore to socket arm 1.04, counterbore to rim 1.15, centre island
  1.26. M2 heads Φ3.4 x 1.5 (measured) in 1.8-deep counterbores; M2 x 6
  engages 2.6 of the horn's 3.0. Hub and upper radius 9.0.
- **PINS hub = the X330 fixture's latest horn attach**, read from
  `config/x330_fixture_cad.yaml` so the two cannot drift: Φ1.5 pins 1.6
  long, no root relief, Φ16.0 well with a 0.4 lead-in. The well's 45 deg
  undercut at its floor leaves **0.4 mm** of wall at the hub's Φ18 rim --
  under two perimeters, accepted because the wall rule is set by the SCREWS
  version; widening the hub would push the lower case's near-row pins off.
- **Case pins in BOTH hole rows.** Upper case: always. Lower case: the near
  row stands 0.52 mm clear of the turning cavity, so it is on
  (`lower_near_pins`, refused under `min_pin_seat`).
- **Upper case wraps the whole servo**, walls nesting over the lower's all
  round. Over the cable connectors' window (grown 0.5 mm) its thick grip
  wall is cut away **down to the lower case's walls**; the edge left spanning
  the window is in the 1.6 mm nest shell only, as a **20 deg gable** (apex
  2.1 mm below), not an 11.5 mm flat bridge. The connectors plug in along Z
  into the back face: the cap is cut away over them from each side in to a
  **12 mm centre strip** (measured), so the cap is an **"H"** with room to
  plug and unplug.
- **Y- attachment:** one 6-32 per case into a **placeholder mount plate**,
  the **ridge now on the case** and the groove in the plate. The plate prints
  **-Y side up** (front face on the bed; the grooves' 3.6 mm ceilings are its
  only bridges) so a **clamp tab** can rise off its bottom edge, back, in
  the plane that is **level at the bike's 15 deg rake**: tab flat on a
  bench, the steering axis is raked like the bike's. 45 x 50 x 5, four 10-32
  (teardropped) on the X330 fixture's 1.5 x 1 in pattern, and the plate's
  bottom widened into a foot so the tab's root does not hang.
- **Load path, ground to headset:** tire -> axle -> fork legs -> the ledges
  under the block -> the block's thrust boss (Φ18 annulus) on the lower
  case's flat underside.

## As built

`python -m aow_sim.cad_steering` (config `config/steering_cad.yaml`), one
feature `AOW steering` in the Feature Studio **aow-bike-steering features**,
inserted in the Part Studio **aow-bike-steering** (`onshape.yaml` tabs
`steering_features` / `steering`). Render: `docs/cad/steering.png`.
`python -m aow_sim.cad_steering` prints the full stack; the landmarks, mm up
the steering axis from the axle:

| z | |
|---|---|
| 122.25 | servo back face |
| 112.20 | cable window's bridged edge |
| 99.25 | servo case face |
| 96.25 | horn face |
| 91.05 | hub underside |
| 90.55 | headset upper top (lugs to 93.85; screw tip 90.43) |
| 87.55 | nut pocket floor |
| 83.05 | step: the upper sits here |
| 82.85 | bushing top |
| 80.90 | central screw's head face |
| 68.25 | thrust face |
| 55.25 | block underside, on the fork ledges (screw at 61.25) |
| 51.25 | tire top |

`--check` (both `SCREWS/CROSS` and `PINS/FLAT`): every part's extent matches
`layout()`; **no interference at rest or steered 30/90/180/270 deg** (only the
axle's press fit in fork L, intended); no flat-crowned holes; no hanging
edges. Downward faces left, all intended: the stepped bridging over the
upper's screw hole, the PINS hub's socket arms (2.7 wide), and the case-pin
reliefs' 0.09 mm^2 slivers. The cable window prints without a bridge.

## Outstanding

- **`bike.front_wheel.radius` is still 0.050 (`design`) against the measured
  51.25.** Not changed: it is a physical parameter, so it goes through the
  CLAUDE.md checklist (deploy bundle, LQR fit, policies, tests).
- Print tolerances, all `GUESS`: bushing bore, end play, spigot fit, axle
  knurl/press holes, fork cheek clearance. Expect to change them after a test print.
- Mount plate is a placeholder; replace with the fixture or chassis interface.
- The lower case is solid out to the mount plane; shave by hand as wanted.
