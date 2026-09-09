# general concept for a fast and tight 180 degree turn

> **Status: CONCEPT ONLY — never implemented, parked 2026-07-19.** No code
> implements this move; `mujoco-modeling-decisions.md` records the goal and the
> reasoning about why the midline turn is hard. Nothing is blocked on it.
> Retired to `old/` 2026-09-08.

This move takes advantage of the 360 degree steering, turning the bike 180 degrees along its midline, instead of pivoting around the front wheel.

## rough manual steps
- start turning the front wheel
- begin reversing while continuing to turn the front wheel
- also apply differential control of the rear wheel as necessary to yaw the bike
- once the front wheel passes 90 degrees, drive probably swaps to the forward direction
- the front wheel should end up aligned straight with the bike (rotated 180 degrees from its original orientation relative the bike)
- the bike should end up rotated 180 degrees and as close as possible to its original directional position (i.e. not shifted laterally)
- the overall maneuver will roughly minimize the lateral deviation of the bike at any point during the maneuver (+/- 0.5x the wheelbase)