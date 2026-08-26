"""Body-frame velocity estimation from wheel encoders + steer angle + AHRS.

`qvel[:2]` has no sensor on the bike, but `general_rl` observes v_lon and
v_lat (control/general_spec.py), so both have to be estimated.

LONGITUDINAL — hub kinematics, directly.
    w_input = w_servo * belt_ratio
    w_hub   = mix_hub_a * w_in_a + mix_hub_b * w_in_b
    v_lon   = w_hub * outer_radius
Measured against sim ground truth: 8.8 mm/s RMS at a 0.6 m/s cruise (~1.5%).

LATERAL — the front wheel's rolling constraint, NOT the rear rollers.

The obvious approach is to invert the AOW's roller kinematics
(`lat_gain(params) * (w_in_a - w_in_b)`). It is not chosen here, but the
reason has CHANGED and the old reason is no longer true.

    WHAT THIS SAID UNTIL 2026-08-26, AND WHY IT WAS RETIRED. "It does not
    work: the rollers are designed to slip in that axis, so the encoder
    reports what was commanded rather than what happened. Between +0.96 and
    -0.20 correlation with truth depending on regime, over-predicting 2.5-3.8x
    open-loop." That verdict predates a lot of contact-model change and does
    not survive re-measurement. Same estimator, current contact, RL policy,
    per-sample against truth (RMS mm/s / correlation):

        regime        front constraint   roller kinematics
        standstill      44.4 / 0.892       25.9 / 0.928
        forward 0.6     14.4 / 0.935       23.4 / 0.852
        reverse -0.4    27.6 / 0.815       20.8 / 0.886
        crab left      136.6 / 0.847      100.6 / 0.812

    Over-prediction is now 1.11-1.24x, not 2.5-3.8x; correlation never drops
    below 0.81; and the rollers BEAT the front constraint at standstill and in
    reverse — exactly where `v_lon * tan(theta)` is structurally blind,
    because at v_lon = 0 it carries no lateral information at all.

SO WHY IS THE FRONT CONSTRAINT STILL PRIMARY? Not accuracy. Measured
2026-08-26 with each estimator IN THE LOOP, a speed-aware front+roller blend
is MORE accurate open-loop and WORSE closed-loop in three regimes of four
(reverse fell at 0.88 s against the front constraint's 1.19 s; crab at 0.79 s
against 1.03 s). The likely mechanism is self-reference:
`roller_lateral = lat_per_d * (w_a - w_b)`, and that differential is EXACTLY
what the balance controller commands — so closing the loop makes the
controller partly measure its own action, which no RMS figure can see.

THE STANDING RULE THAT CAME OUT OF THIS: select estimators on CLOSED-LOOP
SURVIVAL, not on RMS against truth. The two disagree, and they disagree in the
direction that ships a regression as an improvement. See
docs/plans/odometry-rewrite.md.

The front wheel, however, is an ordinary tire and cannot slide sideways. A
normal bicycle has that constraint at BOTH contacts, which over-determines the
planar velocity; the AOW deliberately removes the rear one, leaving exactly one
constraint and exactly one unknown. Writing the front contact velocity as the
rear's plus the yaw lever arm,

    v_front_body = (v_lon, v_lat + yaw_rate * L)

and requiring it to lie along the front wheel's ground heading theta:

    -v_lon * sin(theta) + (v_lat + yaw_rate * L) * cos(theta) = 0
==> v_lat = v_lon * tan(theta) - yaw_rate * L                          (*)

Every input is a good measurement — hub odometry, the steer encoder, and the
AHRS gyro. None of them touch a slipping surface.

MEASURED (2026-08, payload model, 6495 samples over standstill / shoved
standstill / straight 0.6 m/s / circles at R=0.8 and R=0.5):

    estimator                          RMS err     corr
    roller kinematics                 7-23 mm/s   -0.20..+0.96
    (*) front-wheel constraint         6.1 mm/s      +0.993

(Both rows are 2026-08; neither reproduces against the current contact model.
The 2026-08-26 re-measurement above is the live one. This block is retained
because the free-fit argument below is what justifies having no fudge
factors, and that argument is about STRUCTURE rather than about the numbers.)

and a free least-squares fit of the same three regressors returns
tan-coefficient 0.985 (theory: 1.0), L_eff 0.2033 m (geometric wheelbase:
0.200), roll-arm ~0.003 (i.e. none) — reaching 5.2 mm/s. So (*) as written,
with NO calibration constant, is within 17% of the best achievable fit. That
is the reason this module has no fudge factors: the geometry is the answer.

WHERE (*) FAILS, and what happens instead:
  * theta -> +-90 deg. cos(theta) -> 0 and the front wheel, now perpendicular,
    constrains nothing about v_lat. This is not hypothetical: the `flip`
    maneuver pre-steers to exactly 90 deg to free the front (control.flip.
    hold_deg). Confidence is weighted by cos^2(theta) and the estimator coasts
    on integrated acceleration through it.
  * Front wheel off the ground (wheelie, hard braking, launching off a bump).
    The constraint silently stops being true and there is no onboard sensor
    that says so. Bounded by the same accelerometer fallback, but it is a
    known blind spot — see the doc's open items.
  * Front tire actually sliding (hard cornering near the friction limit).
    Degrades gracefully rather than catastrophically, unlike the rear.

The roller estimate is retained as `roller_lateral` for diagnostics and as a
last-resort fallback, NOT as a primary source.
"""

from __future__ import annotations

import numpy as np

from ..control.balance import lat_gain
from ..control.steer import wheel_heading

GRAVITY = 9.81

# Below this |cos(theta)| the front wheel is too close to perpendicular to say
# anything about v_lat. cos(75 deg) ~ 0.26.
COS_MIN = 0.26


class VelocityEstimator:
    """Encoders + steer angle + AHRS -> body-frame (v_lon, v_lat)."""

    def __init__(self, params: dict, lon_smooth: float = 1.0):
        dt_p = params["drivetrain"]
        self.belt_ratio = float(dt_p["belt_ratio"])
        self.mix_hub_a = float(dt_p["mix_hub_a"])
        self.mix_hub_b = float(dt_p["mix_hub_b"])
        self.r_wheel = float(params["omni_wheel"]["outer_radius"])
        self.wheelbase = float(params["bike"]["wheelbase"])
        self.rake = np.deg2rad(float(params["bike"]["rake_deg"]))
        self.lat_per_d = lat_gain(params)      # diagnostics only
        # Fraction of the wheel measurement adopted each tick. 1.0 = trust it
        # outright, which is the right default: encoder quantization is 0.229
        # rpm, i.e. ~0.4 mm/s at the contact, far below the 8.8 mm/s the
        # kinematics itself is good to. Lower it only if hardware encoder
        # noise turns out worse than the datasheet implies.
        self.lon_smooth = float(lon_smooth)
        self.v_lon = 0.0
        self.v_lat = 0.0
        self.confidence = 1.0

    def reset(self, v_lon: float = 0.0, v_lat: float = 0.0) -> None:
        self.v_lon, self.v_lat = float(v_lon), float(v_lat)

    # -- individual channels ----------------------------------------------

    def longitudinal(self, w_servo_a: float, w_servo_b: float) -> float:
        """Hub kinematics -> forward speed at the rear contact [m/s]."""
        wa = w_servo_a * self.belt_ratio
        wb = w_servo_b * self.belt_ratio
        return (self.mix_hub_a * wa + self.mix_hub_b * wb) * self.r_wheel

    def roller_lateral(self, w_servo_a: float, w_servo_b: float) -> float:
        """No-slip roller kinematics. DIAGNOSTIC ONLY, and not because it is
        inaccurate — re-measured 2026-08-26 it beats the front constraint at
        standstill and in reverse. It is kept out of the primary path because
        it is a function of the COMMANDED differential, so feeding it back
        makes the controller measure its own action. See module docstring."""
        wa = w_servo_a * self.belt_ratio
        wb = w_servo_b * self.belt_ratio
        return self.lat_per_d * (wa - wb)

    def lateral_from_front(self, v_lon: float, steer_joint: float,
                           yaw_rate: float) -> tuple[float, float]:
        """Front-wheel rolling constraint -> (v_lat, confidence in [0, 1]).

        `steer_joint` is the raw steer joint angle in radians, any winding:
        `wheel_heading` converts it to the ground-trace heading through the
        raked axis, and is pi-periodic, so a wound multi-turn angle encodes
        identically to its wrapped equivalent.
        """
        theta = wheel_heading(steer_joint, self.rake)
        c = np.cos(theta)
        conf = float(np.clip((abs(c) - COS_MIN) / (1.0 - COS_MIN), 0.0, 1.0)) ** 2
        if abs(c) < 1e-6:
            return 0.0, 0.0
        v_lat = v_lon * (np.sin(theta) / c) - yaw_rate * self.wheelbase
        return float(v_lat), conf

    # -- fused update ------------------------------------------------------

    def update(self, dt: float, w_servo_a: float, w_servo_b: float,
               steer_joint: float, yaw_rate: float, accel_body,
               roll: float, pitch: float = 0.0) -> tuple[float, float]:
        """One estimator tick -> (v_lon, v_lat) in the body frame.

        accel_body : TM151 specific force [m/s^2], chassis frame, gravity
                     INCLUDED (as an accelerometer reports it).
        roll/pitch : from the AHRS quaternion, to subtract gravity.

        THE ACCELEROMETER IS A FALLBACK, NOT A CO-EQUAL SENSOR. Both direct
        measurements here are already good to a few mm/s, so a conventional
        complementary filter actively hurts: integrating specific force over
        even 0.3 s injects the AHRS lever-arm terms (the sensor sits at
        [0.05, 0, 0.13], not at the chassis origin, so it also reads
        alpha x r and omega x (omega x r)). Measured, a tau=0.3 s longitudinal
        blend degraded v_lon from 8.8 mm/s to 174 mm/s RMS. So integration is
        used ONLY where the front-wheel constraint has nothing to say.
        """
        a = np.asarray(accel_body, dtype=float)
        a_lat = a[1] - GRAVITY * np.sin(roll) * np.cos(pitch)
        a_lat -= self.v_lon * yaw_rate          # centripetal, not translation

        # Longitudinal: the wheel measurement, directly. No singularity, no
        # slip worth modelling, nothing for the accelerometer to add.
        v_lon_w = self.longitudinal(w_servo_a, w_servo_b)
        self.v_lon += (v_lon_w - self.v_lon) * self.lon_smooth

        # Lateral: propagate with acceleration, then correct toward the
        # constraint with gain = confidence. At conf 1 that is the measurement
        # outright; at conf 0 (front wheel perpendicular) it is pure coasting,
        # which is the best available answer while the constraint is blind.
        v_lat_c, conf = self.lateral_from_front(self.v_lon, steer_joint, yaw_rate)
        self.confidence = conf
        v_lat_pred = self.v_lat + a_lat * dt
        self.v_lat = v_lat_pred + conf * (v_lat_c - v_lat_pred)
        return self.v_lon, self.v_lat


def body_to_world(v_lon: float, v_lat: float, yaw: float) -> np.ndarray:
    """Body-frame velocity -> world XY, which is what qvel[:2] wants.

    MuJoCo's freejoint carries LINEAR velocity in the world frame and ANGULAR
    velocity in the body frame; extract_state() rotates the linear part back
    by yaw. Handing HardwareData a body-frame vector here would be silent and
    destabilizing, so the conversion lives in one named function.
    """
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([c * v_lon - s * v_lat, s * v_lon + c * v_lat])
