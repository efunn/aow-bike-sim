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

## First print (2026-09-30)

Printed in ASA and assembled, **PINS hub** (the printed pins can shear, as
a fuse). On the Pi bench it is clamped **upside down**, by the mount plate's
tab, so the wheel's load pulls the right way through the headset. The rake
reads the expected 15 deg, off the tab.

| fit | result | action |
|---|---|---|
| headset in the Φ14 bushing (`bore_clearance` 0.2) | fits nicely, no reaming; one tight spot where the two parts' printed seams meet | keep. On a reprint, a small notch to put the seam in one place |
| axial, screw fully tight (`end_play` 0.2) | slight play | **keep** (user): the screw bottoms on the step and cannot back off. Zero or negative play would leave the screw holding the stack together, and it would need threadlock |
| mount plate tab | 15 deg on the bench, as designed | none |

The two are now `print-checked` in `steering_cad.yaml`. Straight ahead is
set at 180 deg on the servo, the nominal `steer_zero_deg`. Any offset is
rigging and will change on the final build.

### Bench tests: headset friction

The one XC330 on the Pi bench is now **id 103**; it was 104. The other XC330
is still 103 and is not on this bus: set it to 104 before the two share one.
Two tests, both in current-based position mode, restoring the servo's mode
and gains on exit:

| test | measures | compare with |
|---|---|---|
| `servo_breakaway.py --ids 103` | the current at which the steer starts to move, both ways, at 3 start angles | the bare XC330, 2026-09-25: nothing at <= 20 mA, everything at >= 25 |
| `steer_friction.py` | running friction against steer angle, 10 deg bins, at 20 and 60 deg/s: where the seam's tight spot is and how big | the gearbox model, 0.15 x load + 5.7 mN m; better, the same run on a bare XC330 |

The Pi's `~/aow-bike-sim` was refreshed from this checkout on 2026-09-30
(it has no git; copy tracked files across, never `--delete`):

```sh
ssh -t efun@aowbike.local 'cd ~/aow-bike-sim && ~/venv/bin/python analysis/steer_friction.py --port /dev/ttyUSB0 --note "upside down, no load"'
ssh -t efun@aowbike.local 'cd ~/aow-bike-sim && ~/venv/bin/python analysis/servo_breakaway.py --port /dev/ttyUSB0 --ids 103 --note "steer installed, upside down"'
rsync -av efun@aowbike.local:'~/aow-bike-sim/traces/{steer_friction,servo_breakaway}' traces/
python analysis/steer_friction.py summary traces/steer_friction/<dir> --plot
```

The steering turns 2 full turns each way (the breakaway's kinetic phase
runs further), so the wheel's sweep has to be clear. `steer_friction`
caps the current at 150 mA, about 0.11 N m at the printed pins, and stops if
the output falls 30 deg behind the ramp.

**Results, 2026-09-30** (first print, upside down, no added load, 12.0 V).
Captures: `traces/steer_friction/260930-130440_steer_friction` and
`traces/servo_breakaway/260930-131115_servo_breakaway`. The same XC330
answered as id 104 in the bare run of 2026-09-25, so that run is the
same-unit baseline. That rests on the id alone; nobody checked the serial.
Torque is k (I - I0), with k 0.83 N m/A and I0 16.5 mA.

| | bare, 09-25 | installed | headset adds |
|---|---|---|---|
| breakaway, every trial moved | 25 mA (7.1 mN m) | **40 mA** (19.5); 30 mA moved 9/12 | ~12 mN m, static |
| kinetic, stops at | 20 mA (2.9) | **32-34 mA** (13-15) | ~11 mN m |
| speed at 40 mA | 8.7-9.0 rad/s | 4.1-6.4 rad/s | |

Running friction against steer angle (`steer_friction`, both reps):

| steer angle | at the output, 20 / 60 deg/s |
|---|---|
| most of the turn | 7-12 mN m |
| **+35 deg, broad peak (+5..+65)** | **31 / 29 mN m** |
| -145 deg, second peak (-165..-145), sharp drop after it | 20 / 20 mN m |

- **It is Coulomb friction.** The map repeats to ~2 mN m at three times
  the speed, so it is not viscous.
- **The two peaks are 180 deg apart:** the printed seams, see below.
- **The peaks are the headset, not the gearbox.** The bare run's kinetic
  phase (09-25, same unit) turned 200-300 deg at each fixed current. In
  30 deg bins its speed is flat against angle to ~0.1-0.2 rad/s from 30 to
  58 mA. The speed slope is ~0.11 rad/s per mA, so that is under ~1-2 mN m
  of angle-dependent gearbox friction. Installed, at the same currents, the
  speed dips 0.8-1.0 rad/s through the -30..+90 bins, about 7 mN m. Only at
  30 deg resolution and at ~9 rad/s; a finer bare map would still need its
  own run.
- **The +35 peak's band starts at +5, right beside straight ahead**, so a
  steer to the + side runs into it within a few degrees. The breakaway
  started at steer ~0, +120 and -120, so it never started on a peak; its
  40 mA is the static friction off them.
- **Not reconciled:** the kinetic phase kept turning down to 34-36 mA
  (~15 mN m), below the map's 31 mN m peak. A guess, not tested: the
  motor's reflected inertia carries it through the peak at speed. Treat the
  map's peak height as an upper figure.
- **Scale:** the XC330's usable torque against a load is ~0.52 N m, and
  the worst spot is about 6% of that.

**Loaded, 182 g added (2026-09-30)**, `260930-135344_steer_friction_load182`.
A rough rig: the weight is wedged against the fork and tyre by a 10-32
through the spokes. Its CoM sits ~25 mm to the side of the wheel and ~25 mm
off the axle, so it is off the steer axis. Plot:
`compare_0_182.png` in that capture.

| | 0 g | 182 g |
|---|---|---|
| mean friction at the output | 12.4 mN m | **22.0** (20 and 60 deg/s agree to 0.4) |
| +35 deg peak | 31 | **29-30, unchanged** |
| -145 deg peak | 20 | **34** |
| one-way push, amplitude | 2.9 | **20.1**, a sinusoid crossing zero near -130 / +40 |

- **The push is the weight's gravity** about the 15 deg axis. 20 mN m
  means a ~43 mm horizontal offset. That is consistent with the rough
  placement, though not checked against it. The half-difference cancels it
  out of the friction.
- **The +35 peak does not grow with load**, so it is not the thrust
  face; a tight spot in the bore or the spigot is the likely source. The
  -145 peak and the baseline do grow: about +10 mN m for +182 g
  (+1.79 N).
- **Not separated:** the off-axis weight also puts a tilting moment on the
  headset. That loads the bushing sideways, so some of the +10 may be the
  bore rather than the thrust face. A centred weight would separate the
  two.

**Balanced, 282 g added (2026-09-30)**, `260930-144648_steer_friction_load282`:
137 g + 145 g, ~25 mm fore/aft and ~25 mm lateral, in opposite directions.
The thrust face carries ~368 g, the bike's ~365. Plot: `compare_loads.png`
in that capture (all three loads).

| thrust face | added | mean friction (20 deg/s) | median | -145 | +45 | one-way push |
|---|---|---|---|---|---|---|
| 86 g | 0 | 12.4 mN m | 10.5 | 19.5 | 27.1 | ±3 |
| 268 g | 182 off-centre | 22.0 | 21.1 | 34.4 | 30.2 | ±20 |
| 368 g | 282 balanced | **26.6** | 25.8 | 26.5 | **46.6** | -8..+3 |
| 158 g | 72 balanced (34 + 38 g, same placement) | 15.3 | | | 27.5 | -4..+2 |
| 86 g | 0, **repeat after all the load runs** | **12.4** | 11.5 | 20.7 | 23.0 | -3..+1 |

- **The mean is linear in thrust load:** a least-squares line through
  all four gives **7.7 + 0.052 x (thrust in g) mN m**, residuals within
  0.6, rms 0.4. That is ~5.3 mN m per N. The three centred runs alone
  give 7.7 + 0.051. At 60 deg/s it is up to ~1 lower. 72 g is
  `260930-145459_steer_friction_load72`.
- **The angle shape did not repeat between rigs.** Balanced, the -145
  peak fell back and the +45 peak rose to 47 mN m, with the whole
  0..+150 half raised. The rig was handled between runs (weights
  wedged in, removed, re-wedged) and the seams are wearing, so the
  map's shape is not a property to model. The mean is the usable result.
- The one-way push is small, so the pair is close to balanced.
- **No drift in the mean after ~20 min of loaded running:** the 0 g repeat
  reads 12.4 mN m again (12.8 at 60 deg/s, also unchanged). The +35 peak
  came down, 31 -> 24, which fits the seams smoothing that the user felt.
  The -145 peak held (19.5 -> 20.7). The fit holds for this print as it is
  now.

### The seams: why two peaks, 180 deg apart

The shaft and the bushing each have a printed seam: a small ridge along
the part's length where every layer starts and ends. Before final
assembly the user could feel and see both. After the friction runs they
feel smoother but are still there, in the same places. Assembled, the
bushing's edges cannot be seen, so which peak is which alignment is
reasoned, not observed.

A round shaft in a round bore binds on the total interference across a
diameter. The shaft seam turns with the steer and the bore seam stays
put, so they come onto one diameter twice a turn:

| alignment | interference across the diameter | the shaft |
|---|---|---|
| **seam on seam** (touching) | h_shaft + h_bore, all at one point | can shift into its clearance, away from the bump |
| **seam opposite seam** | h_shaft + h_bore, one bump at each end | trapped between the two; cannot shift away |

Same total, different contact, so two peaks of different height and
shape: the sharp -145 and the broad +35. It also fits the loaded run,
though that is inferred. The off-centre weight pushed the shaft
sideways, which helps where it can escape and hurts where it cannot, and
only one peak grew. An oval shaft or bore would also give two peaks 180
deg apart, but roughly equal ones.

**Fix: force the seam in printing** (user), so it lands somewhere
harmless, for example into a small notch in the bore and on the shaft,
or a seam placed off the running faces. The user is trying it first on
the self-righting module's printed bearings, the next print; the steer
gets it on its reprint. If both peaks drop, the seams are confirmed.

**Not in the sim, on purpose** (user): the seam can be fixed in printing,
so the peaks are not modelled at all.

**In the sim: the load line only** (2026-09-30). This is
`bike.steering.headset_friction_nm` / `_per_n`, the headset under the
gearbox: a + b x |T|.
- **The load:** T is the headset's reaction along the steer axis, from a
  `headset_force` sensor on the steer body. At rest it is 3.1 N axial and
  0.95 N side, the same side/axial ratio as the bench's 15 deg rig (0.28
  against tan 15 = 0.27), so the side load is in b implicitly.
- **The constants:** the four-load fit with the gearbox's running line
  taken out, (tau - 5.7) / 1.15, per newton along the 15 deg axis.
  `test_gearbox_friction` checks they give back the bench line at all four
  loads.
- **What is left out:** static friction, about 1.5x on the unloaded
  breakaway, is not modelled. Cost, effects and the tests it moved are in
  status.md.

## Outstanding

- **`bike.front_wheel.radius` is still 0.050 (`design`) against the measured
  51.25.** Not changed: it is a physical parameter, so it goes through the
  CLAUDE.md checklist (deploy bundle, LQR fit, policies, tests).
- Print tolerances still `GUESS`: spigot fit, axle knurl/press holes, fork
  cheek clearance. Nothing reported against them from the first print.
- The other XC330 is still id 103: set it to 104 before it joins this bus.
- **Weighed 2026-09-30: both fork halves + front wheel (axle in) = 86 g.**
  The headset pieces are not included. The axle is too tight to take out
  for now, so the halves and the wheel have not been weighed apart.
  bike_params still carries `front_wheel.mass` 0.060 + `fork_mass` 0.025
  (`GUESS`, 85 g together, a coincidence). Not changed: it is a physical
  parameter, so it goes through the CLAUDE.md checklist, and the split
  between the two is not known yet.
- **Loaded friction: 182 g off-centre and 282 g balanced done (above).** 72 g and a 0 g repeat done too. No drift in the mean (12.4 both times, `260930-150300_steer_friction_load0_repeat`); the +35 peak came down 31 -> 24. The bench weight has to match
  the load through the thrust face, not the load on the tyre. On the bike
  that is the ground force minus the weight of everything that turns
  (m_r: 86 g + the headset pieces, unweighed). Upside down it is m_r plus
  what is added. So added = F_front - 2 m_r: 450 - 172 ≈ **280 g**, a
  little less once the headset is counted. F_front ~450 g is the sim's
  (1.016 kg, 44% front, with guessed masses). Steps: 0 / 150 / 280 / 450 g
  added (the last ~1.5x, for bumps), one `steer_friction --tag loadNNN`
  run each. That gives friction against load, and whether the +35 deg
  peak is the thrust face (grows with load) or the bore.
- Mount plate is a placeholder; replace with the fixture or chassis interface.
- The lower case is solid out to the mount plane; shave by hand as wanted.
