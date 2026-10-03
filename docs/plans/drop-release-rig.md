# Drop release rig: one XL330, a hinged arm, a swappable snail cam

Status: **design sketch, nothing built** (2026-10-02). Layout from the
user's hand sketch of the front rig (2026-10-02); lengths not yet set. Replaces hand drops
for the contact drop test (`contact-protocol.md` §P1). The hand drops on
2026-10-02 showed what it has to fix:

| hand-drop problem | what fixed it |
|---|---|
| wheel partly supported at rest: rest force 78-93 g for a weighed 86 g | nothing touches the arm after the drop: the cam is in its dwell, clear |
| rocks and rolls on the button: 5 of 16 drops rocked or marginal | the hinge allows one arc only |
| landing squarely on a 12.2 mm button by hand (not seen to fail, but nothing ensured it) | the axle sits over the button, fixed |
| height uncontrolled, 0.3-2.9 mm | the cam's steps; `force_drop.py` still measures each one |

![how the cam works](drop-cam-explained.svg)

## Side view

```
   hinge block                clamp block          fork (lying level)
   (two bearings)  two rods   (fork's spigot)    ______________
      o==========================[#########]====|______o_______|  <- axle
      |                               |                (  wheel  )
      |                            follower            (   r50   )
   hinge post                         |                 \_______/
      |                             [cam]               [sensor]  <- button
      |                            [XL330]                           under the axle
   ===+=================================================================  base
      |<------------- d -------------->|
      |<---------------------- L ----------------------->|
```

The hinge axis is parallel to the wheel's axle, so the arm runs in the
wheel's plane like the bike frame. The fork lies level, its spigot clamped
in the block, so the axle is at the arm's height and the arm is level at
impact. The cam is under the block, at d from the hinge, not under the axle.

## Dimensions

| item | value | why |
|---|---|---|
| L, hinge axis to wheel axle | **~200 mm** (not set) | long, so the arc is near-vertical (2 mm drop = 0.6 deg) and the wheel's own spin inertia adds only (r/L)^2 ~ 3% to the impact mass |
| d, hinge axis to follower | under the clamp block, d < L (not set) | the cam's steps reach the axle as **L/d** times larger: at d/L = 0.75 the 0.5/1/1.5/2 cam drops 0.67/1.33/2/2.67 mm |
| hinge axis height | **= the axle's height with the wheel resting**, adjustable +-2 mm (slotted post) | the arm is level at impact, so the contact moves straight down |
| axle height at rest | sensor seat + 8.25 mm (FS20 incl. button, datasheet) + wheel radius (front 50 mm) | measure it on the built base; the slots take up the difference |
| arm | **two parallel rods**, hinge block to clamp block | stiff in twist, so the wheel cannot lean; keep them light (see "Impact mass") |
| hinge | bearings in the hinge block, one per rod side, on a steel pin | low friction; their spacing across the arm stops the wheel leaning |
| wheel interface, front | the **clamp block**: a double-D socket like the headset upper (dia 11, 6 mm across the flats, central 6-32), its axis along the arm | reuses the fork's own spigot (`config/steering_cad.yaml` `spigot_*`); no new fork parts |
| wheel interface, rear | the same block holding the rear dropouts; the axle nut locks the wheel's roller phase | one arm for both wheels; still to design with the new dropouts |
| follower | **a small printed flat** under the clamp block, in line with the cam; face square to the arm's motion | the release is the cam's step corner passing the flat's downstream corner. On a ramp the contact sits ~5 mm off centre, hence ~6-10 mm wide. Printed corners round to ~0.45 mm, so the release still wants the cam at speed: the script holds at the start of the top flat as a run-up |
| cam | `bench/drop_cam.py`: per 90 deg segment a 20 deg bottom dwell at r 12 mm, a 40 deg ramp, a 30 deg top flat (constant radius) at 13.5 / 14 / 14.5 / 15 mm, then the step; 5 mm thick, D-bore dia 8 / 7 flat | four drops per turn, 0.5 / 1 / 1.5 / 2 mm; the top flat is the servo's run-up to speed before the edge, the bottom dwell is where it brakes and parks. Ramp pressure angles 9.6-17.7 deg |
| cam hub | boss on the XL330 horn: D-shaft dia 8 / 7 flat, 6 mm long, one M2 into the end through a washer | swap a cam with one screw; the start is set by hand each run, so no indexing |
| cam centre height | **follower resting height - 13 mm** (r_dwell 12 + gap 1) | the follower rests 1 mm above the dwell (L/d x 1 mm at the axle) |
| servo mount | fixed on the base, no shims (user, 2026-10-02) | the gap is trimmed at the axle instead; it shifts all four drops together |
| sensor | in a pocket in the base (25.1 x 17.3 mm body), button centred under the axle | the FS20 deflects 0.05 mm at 14.7 N (stiff: >= 294 N/mm) |

The XL330 horn's hole pattern is not drawn anywhere here: take it from
ROBOTIS's drawing when the hub is modelled.

## Setting the drop heights

The cam fixes the RATIOS between drops exactly (each step times L/d at the
axle); for round numbers at the axle, give `drop_cam.py --drops` the wanted
heights times d/L. Where they sit is set by the follower's height over the
cam, and that is trimmed at the AXLE (prop / shim it), not under the servo
(user, 2026-10-02): at d = L the follower 0.3 mm low gives 0.8 / 1.3 / 1.8 /
2.3 mm. Anything from just under 0.5 mm high (the 0.5 drop then vanishes)
to just under 1 mm low (the gap closes) still gives four real drops. The rear wheel's radius changes with the roller part on
the button (51.0 at the cones' big ends, 49.5 at their small ends, 51.2 at
the ridge), and every mm of it moves all four drops by a mm: re-trim per
contact, or regenerate the rear interface for it
(`wheels.rear_radius_offset`). `force_drop.py` reports the height each drop
actually had (`h_measured_mm`), so trim until the first cycle reads about
0.5 / 1 / 1.5 / 2, then leave it.

Check the gap by hand with the wheel resting: a 0.5 mm feeler under the follower,
with the cam in a dwell, should slide. No gap means the arm is partly held by
the cam and the rest force reads low again.

## Impact mass

On a hinge the mass the sensor feels at impact is the arm's inertia about
the hinge over L^2, which is not quite the resting share:

| | resting share | felt at impact |
|---|---|---|
| wheel + fork (axle above the contact) | m_w | m_w + I_w / L^2 |
| clamp block + follower, mass m_b at d | m_b d / L | m_b (d / L)^2 |
| arm (rods), mass m_a | m_a d / (2L) | m_a d^2 / (3L^2) |

(The rods taken as running hinge to block.) I_w / L^2 is at most
m_w (r / L)^2 ~ 3 g for a 60 g wheel at r 50 mm. So pass `--mass-g` = the
"felt" column summed (+ ~2 g for the wheel's spin), not the resting force.
Weigh the block, the rods and the wheel + fork before the first run.

Peak force grows with sqrt(mass): ~100 g felt at 2 mm on ~50 N/mm is ~14 N,
inside the ~19 N where the reading clips. The rear assembly is heavier: start
it with a 0.5-1.5 mm cam.

## Release speed

Printed corners round to ~0.45 mm (cam and follower together), so the
follower only free-falls if the step's edge passes faster than
sqrt(g * 0.45 mm) ~ 66 mm/s: ~280 deg/s at the 13.5 mm top radius, ~45% of
the XL330-M288's 618 deg/s no-load (estimated). Slower, it is let down over
the corner and the drop starts soft and short. With the follower at d, it
falls at about g d / L (most of the mass at the axle), so the need drops by
sqrt(d / L), ~13% at 0.75 (inferred); `force_drop.py --cam` keeps the d = L
figure, which errs safe.

How many degrees the servo needs to get there depends on its reflected
inertia, which is not known here: bracketing it, ~2-8 deg to 45% of no-load
speed and ~4-21 deg to 65%. Hence the 30 deg top flat as the run-up, and the
park 12 deg past the step, clear of where the servo starts braking for its
goal. `force_drop.py --cam` measures it instead of trusting this: every drop
prints the edge speed as the step passes (from position reads during the
move, at 3 Mbps; a synthetic 500 deg/s trace read back as 483).

## Parts

- one base which holds the PCB with the force sensors and to which the XL330 attaches
  - force sensor PCB:
    - found in [key-holder-v5](https://cad.onshape.com/documents/ac102712f8d1c43274e5ca9a/w/98e7a679d8a4a7765bb5abd0/e/9f9e8466f192aa7d04dcac53)
    - `pcb-body`, `pcb-base`, and the `PCB` part are the actual circuit board, 5x self tap M3 screws to the base
    - `fs-arrangement` sketch shows the force sensor placements
    - not pictured: the teensy which is on tall headers; it sits at the back of the PCB and no part of the wheel can intersect it
    - ignore keyswitch holder/keycap and the weird finger guard at the front; these are not present (it's just the bare sensors)
- XL330 case halves, attaching with csk 6-32s to the base
- drop cam which simply fits onto the XL330 horn with printed pins
- interface pieces for (1) front wheel fork halves and (2) rear chainstay halves
  - follower for drop cam
  - 5.75mm tall x 19.75mm wide x ~20mm length slot for a plastic bar to be friction fit into (the lever arm)
- everything past the lever arm attach I'll rig manually

**Generated 2026-10-02:** `python -m aow_sim.cad_drop_rig`, the custom
feature `AOW drop rig` in aow-bike's `drop-rig` Part Studio (studio
`drop-rig features`), numbers in `config/drop_rig_cad.yaml`. Render:
![drop rig](../cad/drop_rig.png)

| part | print | from |
|---|---|---|
| base | Z+ | the PCB on 5 standoffs (M3 self-tap pilots 2.5 GUESS, THROUGH so a tap can run all the way), the shells' two 6-32s from below; board outline, holes and buttons read off key-holder-v5 through the API, not measured on the board |
| back shell, cover | Y-, Y+ | the X330 fixture's IDLER shells, legs down to the base |
| cam | Y+ | `drop_cam.py`'s profile on the horn pins (no D-hub, no M2), its back face 1 mm off the cover |
| front interface | X+ | cad_steering's headset block (ledges, cheeks, two 6-32 nut slots) + bar slot + follower |
| rear interface | X+ | cad_drive's case-side joint both sides (channel, tension slot, nut slot) + bar slot + follower |

The wheel stands on button 3 (user: 2 or 3, for a compacter base; the
wheels pass 6.6 mm over button 2). The arm runs along the row toward the
board's far end, cam centre 80 mm along it, just past the board's edge.
Base 135 x 72 mm, ~69 g solid PLA. Drawn nominal: wheel resting, arm
level, the follower 26.7 mm over the button, 1 mm over the cam's dwell, so
a lift sketched off the cam reads the drop itself. Each interface carries
its own wheel's radius (front: tire OD 102.5; rear: omni outer_radius +
`wheels.rear_radius_offset`, the contact under test) and puts the follower
at that one height, so one base and one cam serve both. `--check` (one call) builds both: no interference.

- **The follower is NOT the ~10 mm centred flat above.** Parked past a
  step, a centred 10 mm flat rests on the next ramp and the step's
  corner, up to 1.85 mm over the gap (computed, `cad_drop_rig.park_clearance`):
  the arm would be held off the sensor. It runs 1 mm downstream of the cam's
  centre (that corner is the release) and 4.5 mm upstream (a flat follower's
  contact offset on the 2 mm ramp is 4.3).
- **The XL330's horn faces -Y** (its body on the +Y side), so the cam,
  turning clockwise looking at the horn as `drop_cam.py` draws it, runs
  +X under the follower. The release is the pad's square +X end; the
  45 deg chamfer that lets the interfaces print X+ is upstream, over the
  incoming ramp. Parked 12 deg past a step: 0.87 mm clear, worst segment
  (1.0 at 8-10 deg). With the chamfer downstream instead it was 0.45 at 12.
- **One bar-slot floor for both wheels:** 73.1 mm from the axle (the rear
  block sets it; the front's floor is 5.85 thick). Bottom the bar in
  either interface and the axle lands over the button and the follower
  over the cam -- the slot floor is the X reference, so push the bar home.
- **Bigger drops: not with four per turn.** A flat follower's contact
  offset on a 40 deg ramp is (gap + drop) / 0.70 rad: 7.2 mm at 4 mm, 8.6
  at 5 mm. A pad that long rests on the cam when parked (-0.14 / -0.72 mm,
  best park, with the chamfer then downstream; recheck). One 5 mm drop per
  turn (a 340 deg ramp) cleared 0.84 mm. The
  cam itself would fit: 22 mm off the tire at r 15, ~19 at r 18.
- The bar slot's centre is 3.9 mm over the axle, not on it: the housing is
  flush with the block's top. Printed X+ the slot stands open-topped.
- The rear interface's nut slots and channel ends are 2-3 mm bridges.
- Not modelled: the Teensy (position unknown here), the bar, the hinge.

## Running it

```sh
python bench/drop_cam.py                                   # the 0.5/1/1.5/2 cam -> bench/drop_cam.{svg,dxf}
python bench/force_drop.py --cam --cam-where              # once per assembly: cam.index and cam.dir
python bench/force_drop.py --wheel front --mass-g <felt> --cam --repeats 3
```

One script, one terminal: `force_drop.py` fires each drop itself, so each
one is labelled by the step that released it. The cam sits on the horn
pins in one position, so `Present Position mod 4096` is its angle;
`--cam-where` prints it, torque off, while the cam is turned by hand: the
reading as the first drop releases the follower is `cam.index`. That, the
direction, the U2D2's port (default: the one `usbserial` present), the
servo's ID, the cam's drops and the current live in
`config/drop_rig_bench.yaml`; each has a flag to override it.
Current-based position mode with one Goal Current (300 mA) written with
every move, and every move FORWARD: per step a setpoint on its top flat
(`--margin-deg` 30 short of the step, the run-up) and one in the dwell
(`--park-deg` 12 past it). Backward would drive the follower into a step's
undercut. It starts wherever the cam is, drives forward to the next top
flat (a drop on the way is not recorded) and zeroes there with the wheel
lifted. On any exit it goes forward to the next dwell, then torque off.
Profile Velocity is not used: ROBOTIS applies it in (extended) position
mode only. Not yet run on the rig (dry-run against a simulated servo and
sensor, 2026-10-03).

The XL330 is 5 V: its own supply through the U2D2, never the 12 V chain
(`untethered-setup.md`).

## Next

- ~~Generate the parts that have to fit existing geometry~~: done
  2026-10-02, see "Parts". Roller phase: locked with the axle nut (user).
- Print and check: the M3 pilots, the fork and chainstay joints in their
  new blocks, the bar's friction fit, the follower's parked gap with a
  feeler.
- **Hinge and arm: not decided** (user, 2026-10-02). The two rods and the
  hinge block above are the sketch, not a choice; d and L are unset.

## Open

- The sensor board's layout and how the base clamps to it: not known here.
- The fork lies level, so at impact its legs bend where on the bike they
  are loaded mostly along their length. Any bend is in series with the tire
  and reads as a softer contact. Not measured: check with the dial on the
  axle vs on the clamp block under a weight, before trusting a sink.
- Whether the real wheel's bounce depends on mass (the sim's front does not,
  its rear does): bolt weights to the wheel end and repeat.
- First run: does the follower leave the step cleanly? A slow release shows
  up as `h_measured` short of the cam's step (by up to the ~0.45 mm corner
  rounding) and a softer rise in the trace. If so, a longer run-up
  (`--margin-deg`) or a crisper corner.
