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