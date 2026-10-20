"""Virtual gamepad: the one place a human input becomes a bike command.

Dependency-free (numpy only). No HID reading lives here — this is the *shape*
of a controller, so that a real one can be plugged in later without touching
anything downstream:

    keyboard  --ramps-->  Pad  --apply()-->  DriveController.set_command_polar
    gamepad   --HID---->  Pad  ------^         (not implemented yet)
    script    --------->  Pad  ------^         (aow_sim.record drawings)

The mapping is not arbitrary. The general policy's command IS a velocity
vector plus a heading (see general_spec.py), and that is exactly what two
analog sticks are:

    LEFT stick   = the velocity vector      Y forward/back, X crab
    RIGHT stick X = heading RATE            (heading is a setpoint, so the
                                             stick slews it, like the arrows)

Sticks centred means `(0, 0)` velocity — an ordinary point of the command
space, not a singularity, which is the whole reason the command is a vector
rather than (course, speed).

Why a rate on the right stick and not an absolute heading: a stick returns to
centre when released, and an absolute mapping would then snap the commanded
heading back to zero. Rate integrates, so letting go holds the heading you
turned to — matching the arrow keys' behaviour today.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Real-controller deadzone. Sticks do not return exactly to centre, and an
# uncorrected 2% bias integrates into a slow, permanent turn.
DEADZONE = 0.08


def _dz(v: float, dead: float = DEADZONE) -> float:
    """Deadzone with the live range rescaled, so the first responsive position
    is still a *small* command rather than a jump to `dead`."""
    v = float(np.clip(v, -1.0, 1.0))
    if abs(v) <= dead:
        return 0.0
    return float(np.sign(v) * (abs(v) - dead) / (1.0 - dead))


@dataclass
class Pad:
    """Controller state. Axes in [-1, 1]; buttons are edge-triggered by the
    caller (True only on the frame they are pressed)."""

    ly: float = 0.0          # LEFT stick Y  -> longitudinal velocity (+fwd)
    lx: float = 0.0          # LEFT stick X  -> lateral velocity (+left/crab)
    rx: float = 0.0          # RIGHT stick X -> heading rate (+left/CCW)
    buttons: set[str] = field(default_factory=set)

    def pressed(self, name: str) -> bool:
        return name in self.buttons


def apply(pad: Pad, psi: float, dt: float, v_max: float, crab_max: float,
          turn_rate: float) -> tuple[float, float, float]:
    """(pad, current heading setpoint) -> (v_lon, v_lat, new heading setpoint).

    The entire control law, shared by teleop, the recorder's drawing scripts,
    and any future HID reader — so a shape drawn by a script is reachable by
    hand, and vice versa.
    """
    v_lon = _dz(pad.ly) * v_max
    v_lat = _dz(pad.lx) * crab_max
    psi = psi + _dz(pad.rx) * turn_rate * dt
    return v_lon, v_lat, psi


def to_polar(v_lon: float, v_lat: float) -> tuple[float, float]:
    """(v_lon, v_lat) -> (speed, course relative to the commanded heading), the
    form `DriveController.set_command_polar` takes. Both zero gives speed 0 at
    course 0, which resolves to the vector (0, 0)."""
    return float(np.hypot(v_lon, v_lat)), float(np.arctan2(v_lat, v_lon))


def from_state(s) -> Pad:
    """A controller snapshot in the STANDARD layout (`gamepad_usb.PadState`:
    +x right, +y up) -> the bike's `Pad` (+lx and +rx LEFT, the bike's +y and
    CCW). The one place the two sign conventions meet: push the left stick
    right and the bike crabs right; push the right stick right and it turns
    clockwise."""
    return Pad(ly=float(s.ly), lx=-float(s.lx), rx=-float(s.rx))


# Physical buttons -> what they do, by the stations' own keys. Names are the
# standard layout's (`gamepad_usb.PadState.buttons`), so any controller that
# reports them binds the same. The stations hold the key codes
# (run_drive.TELEOP_PAD_KEYS / MIRROR_PAD_KEYS); this table is the intent.
BUTTON_MAP = {
    "LB": "snap_l",       # heading +90 deg, CCW  (`6`)
    "RB": "snap_r",       # heading -90 deg, CW   (`7`)
    "BACK": "snap_180",   # about-face            (`8`)
    "A": "re-aim",        # heading setpoint to where the bike points (`/`);
                          # the sticks zero the velocity on release, but the
                          # heading is a setpoint and holds
    "Y": "engage / reset",  # mirror ENTER (engage, re-arm); teleop BACKSPACE
    "X": "lie down",        # teleop PageDown (swing_sit); nothing on the bike yet
    "B": "get up / disengage",  # teleop PageUp; mirror BACKSPACE (standby)
    "START": "camera",    # teleop `\\`
    "DPAD_UP": "righting current +",    # `]`
    "DPAD_DOWN": "righting current -",  # `[`
    # Triggers: RT - LT slews the righting crank's goal (analog; equal pulls
    # cancel), teleop and mirror both.
}
