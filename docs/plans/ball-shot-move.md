# concept and approach for hitting a ball

This move starts from a standstill some distance behind and to the side of a road hockey ball. The bike accelerates and turns to hit the ball as hard and accurately as possible; the bike is able to recover from missed shots.

## bike physical changes

- motors/rear drivetrain moved inboard
- the motors can be vertically stacked to reduce the overall width (drivetrains/belts will be vertically offset at the motor end, converging at the axle)
- this should move the rear pulleys inboard, too
- a 'hockey stick' (thin panel) attached to each side of the bike extending between roughly the center-x of the bike to the rear, partially covering the rear axle/wheel, with enough ground clearance to both hit the ball and allow lean maneuvers
- lowest z-point of the stick should not touch the ground during the move (just confirm by limiting roll/lean in the sim, no need to check for ground contacts in the RL sim, although the stick should still collide with the ground; assume hard plastic like ABS)
- stick will need collision logic (especially with the ball)
- stick may be curved (later?) and/or at a slight angle to the body's main axis (could still be flat)
- make the stick appear translucent to maximize visibilty in the sim

## ball
- a road hockey ball

## RL simulation
- bike starts stationary, facing forward with the ball in front and to the right (policy could later be mirrored for ball-left starts; x and y offsets configurable)
- ball is stationary, with its starting position known relative to the starting position of the bike
- optimize for:
  - launch speed of the ball
  - launch angle of the ball (nominally, straight forward in x)
  - hit the ball with the 'stick' and not with the front/back wheels
  - bike recovers and can safely transition to the nominal stationary roll controller (end orientation not important)
  - if there is no ball (catch trials?), the bike does not fall over using the same policy.
---

# observed behaviour on hardware — the wheel shot

Recorded 2026-08-24, analysed from nine phone clips in
`traces/ball_shot_traces/` (gitignored, Dropbox-synced — not in git, but not
lost either). Timestamps below are clip-relative seconds and are the citation:
open the clip and look.

**These clips are the whole evidentiary record.** The chassis they show is no
longer running, so nothing here can be re-measured, re-shot at a higher frame
rate, or checked against a repeat trial. Every number below is what 30 fps
consumer video can support and no more. Treat this section as a fixed
observation set to design against, not as a measurement that can be refined
later.

## what is in the clips

Two different rigs, and they are easy to mix up:

- **top level, 6 clips** — the AOW bike: the HC-802 omni rear wheel in a
  black 3D-printed chassis at the shortened wheelbase and reduced rake. The
  top view at `Snapchat-1498590690.mp4` 7.57 s shows the entire electronics
  bay: one small green PCB and a red/black lead pair. No Pi, no Dynamixels,
  no TM151, no umbilical. It is the donor's own board in a new chassis, so
  **none of these clips show any controller in this repo.**
- **`original_bike/`, 3 clips** — the donor intact: a red "SOUL GTS / RS300"
  sportbike with rider figure, i.e. the HC-802 clone named in
  `prelim-architecture.md`. Stock everything, kept for reference.

| clip | s | ball | operator | behaviour | outcome |
|---|---|---|---|---|---|
| `975093333` | 1.9 | yes | fingertip on front tyre to 1.07 s | lurch; rear wheel punts the ball at 1.17 s | stays up |
| `1247565720` | 1.6 | yes | finger blocking front tyre to 0.77 s | same; shot at 0.77 s | stays up |
| `158024408` | 2.6 | yes | hand at front 0.65–1.5 s | same, weaker shot 1.5–1.75 s | falls ~1.85 s |
| `1498590690` | 10.0 | no | repeated finger pokes at the front wheel | perturbed standstill balance, survives 9.3 s | falls ~9.6 s |
| `156276133` | 10.0 | no | none | free ride, slow S-curves, drives ~8 m away | up |
| `1216730766` | 12.1 | yes | none, filmed from overhead | ~10 s loitering beside the ball, then a dribble | up |
| `ob/1940365976` | 6.0 | no | open-hand slaps | perturbed balance, recovers every time | up |
| `ob/2055006982` | 7.9 | no | pushes and holds | same; holds large lean angles | up |
| `ob/807706018` | 8.3 | no | none | free ride, wide slow arcs, persistent lean | up |

Ball speeds, scaled off the ball's Ø67 mm in-frame: **1.0–1.2, 1.0–1.1, 0.7,
0.34 m/s** for the four clips with a ball. Only the 0.34 is solid. The other
three are filmed side-on and the ball flies toward the camera — its apparent
diameter grows 192 → 234 px through the shot in `975093333`, so those figures
carry perspective inflation of order ±25 % and the true speeds are more likely
at the low end of each range.

## the shot is a saturated recovery, not a strike

The same sequence produces every fast shot:

1. The operator pins the **front** wheel with a fingertip. That is the
   steering — the balance actuator at any speed above zero — so pinning it
   takes the authority away.
2. Roll error builds and the rear channel answers. **Only the rear wheel is
   motion-blurred**; chassis, front wheel and floor are all sharp in the same
   frame (`1498590690` 7.57 s, `975093333` 1.10–1.13 s). It is spinning hard
   with the bike essentially parked.
3. The finger releases and the tail swings out.
4. The ball leaves at ~1 m/s roughly **perpendicular to the bike's long
   axis**, while the chassis centroid is doing 0.08–0.10 m/s. Ten to one.
   That energy is not chassis momentum; it comes off the wheel surface.
5. The wheel is sharp again within two frames. It spun up, kicked, and
   stopped.

`1216730766` is the control case. No operator, so no lurch: the bike leans on
the ball and dribbles it at 0.34 m/s with contact held for over a second. Same
bike, same wheel, no saturation — and roughly a third of the speed.

So the wheel shot is **the rear channel at full authority catching a roll,
with the ball inside the sweep.** Nobody aimed it.

## what this says about the move as specified above

The concept at the top of this file optimises a *deliberate* strike with the
stick panel, and `config/rl_ball.yaml` prices a wheel contact as a fault
(`w_wheel_hit: 200.0`). `moves/ball_rl.yaml` duly reports `stick_hit_rate:
1.0`, `wheel_hit_rate: 0.0`, `mean_launch_speed: 0.378`.

The hardware's fastest observed shot is ~3× that, and it is the exact contact
the reward pays the policy to avoid. That is not an argument for deleting the
penalty — a controlled stick strike is aimable and a saturated recovery is
not, which is the whole reason the stick exists. It is an argument that
**0.378 m/s is low enough to be worth explaining**, and the explanation on
offer is that the stick strike as currently shaped transfers less than an
accidental wheel contact does.

Open, and not answerable from these clips: whether the launch comes from the
drive channel's tangential surface speed or from the differential's roller
(lateral) speed. Those are different actuators and reproducing the effect in
sim means knowing which.

**Do not reach for a high-speed camera to settle it — the frame rate was never
the problem.** At `v_max` 1.2 m/s the rim turns 3.7 rev/s (224 rpm); the
8-fold axle symmetry puts unambiguous unwrapping at 60 fps Nyquist, 120 fps at
4× oversample. The rollers are slower still in absolute terms — 0.4 m/s
lateral is 6.2 rev/s, ~25 fps at 4× oversample. A phone's 240 fps already
oversamples both channels. What actually binds is:

- **Exposure.** Freezing the rim to 2 mm of arc at 1.2 m/s needs 1/600 s;
  1 mm needs 1/1200 s. Phone slow-mo pins exposure near 1/fps, so 240 fps
  still smears ~5 mm — most of a roller's 7.5 mm length, and exactly why the
  rear wheel reads as a smear in the clips above. That is a lighting budget,
  not a camera spec.
- **Fiducials.** Eight identical axles and sixteen near-featureless rollers
  are unreadable at any frame rate. One paint dot on the rim and one on a
  single roller makes 120 fps sufficient; without them 5000 fps still cannot
  tell you which roller it is looking at.

For ball exit speed alone, 240 fps is ample (4.2 mm/frame at 1 m/s, ~6 % of
the ball's diameter). 1000 fps only earns its place if the contact event
itself is wanted — 60 g off a rubber roller is perhaps 5–10 ms, i.e. 1–2
frames at 240 and 5–10 at 1000.

All of which is moot on a bench rig, where inputs A and B drive independently:
command drive-only, then differential-only, and watch which one flings a ball.
The question is only hard because the flight in these clips was an
uncommanded transient on a chassis that no longer runs.
