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
      |                            [shims]
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
| servo mount | on a shim stack (0.25 / 0.5 mm shims) under the XL330 | sets the gap, which shifts all four drops together |
| sensor | in a pocket in the base (25.1 x 17.3 mm body), button centred under the axle | the FS20 deflects 0.05 mm at 14.7 N (stiff: >= 294 N/mm) |

The XL330 horn's hole pattern is not drawn anywhere here: take it from
ROBOTIS's drawing when the hub is modelled.

## Setting the drop heights

The cam fixes the RATIOS between drops exactly (each step times L/d at the
axle); for round numbers at the axle, give `drop_cam.py --drops` the wanted
heights times d/L. The servo's height sets where they sit: at d = L a servo 0.3 mm high gives 0.8 / 1.3 / 1.8 / 2.3 mm. Anything
from 0.5 mm low to 1 mm high still gives four real drops and a clear gap.
`force_drop.py` reports the height each drop actually had (`h_measured_mm`),
so shim until the first cycle reads about 0.5 / 1 / 1.5 / 2, then leave it.

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
sqrt(d / L), ~13% at 0.75 (inferred); `drop_release.py` keeps the d = L
figure, which errs safe.

How many degrees the servo needs to get there depends on its reflected
inertia, which is not known here: bracketing it, ~2-8 deg to 45% of no-load
speed and ~4-21 deg to 65%. Hence the 30 deg top flat as the run-up, and the
park 12 deg past the step, clear of where the servo starts braking for its
goal. `drop_release.py` measures it instead of trusting this: every drop
prints the edge speed as the step passes (from position reads during the
move, at 3 Mbps; a synthetic 500 deg/s trace read back as 483).

## Running it

```sh
python bench/drop_cam.py                                   # the 0.5/1/1.5/2 cam -> bench/drop_cam.{svg,dxf}
python bench/force_drop.py --wheel front --mass-g <felt> \
    --heights 0.5,1,1.5,2 --repeats 3 --order cycle        # terminal 1: catches every impact
python bench/drop_release.py --port <U2D2> --drops 12      # terminal 2: 3 turns of the cam
```

Before the first run, with torque off, turn the cam by hand until the
TALLEST step (2 mm) has just passed the follower: the arm rests on the sensor and
the next drop is the 0.5 mm one. `drop_release.py` starts from wherever it
finds the cam, refuses if torque is already on, and runs in current-based
position mode (multi-turn): ~300 mA up the ramp, the full Current Limit for
the drop. On any exit it backs down to the last park if still on the ramp,
or runs on through the step if on the top flat, then turns torque off.

The XL330 is 5 V: its own supply through the U2D2, never the 12 V chain
(`untethered-setup.md`).

## Next

- **Generate the parts that have to fit existing geometry:** the cam as a
  solid on the XL330 horn hub (`drop_cam.py` already gives the profile), and
  the two clamp blocks: front, the fork's spigot; rear, the chainstay /
  new dropouts. Roller phase: locked with the axle nut (user, 2026-10-02).
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
