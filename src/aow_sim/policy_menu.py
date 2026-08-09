"""An in-scene menu for picking which controller drives, live, in teleop.

WHY IT LOOKS LIKE THIS. A real mouse-driven dropdown is not available:
`mujoco.viewer.launch_passive` exposes exactly three hooks — `key_callback`,
`show_left_ui`, `show_right_ui` — and no way to add a widget to either panel
(the sections in the C `simulate` app are built with mjUI, which the Python
viewer does not surface). There is no 2D overlay hook either; the only thing
this process can put on screen is scene geometry. So the menu is drawn as
mjvGeom LABELS anchored in front of the camera, which reads as a panel and
tracks the view, but is driven by keys rather than the mouse.

KEY CHOICE, THE HARD WAY. MuJoCo's viewer owns every letter A-Z, teleop spends
all ten digits across its two modes, and the obvious free-looking keys are not
free: TAB was tried first and turns out to be the viewer's own left-panel
toggle, so it opened the menu AND the panel. The viewer's bindings cannot be
enumerated from here, which makes any unused key a guess.

So the menu does not take a new key. It takes ',' — already the "change who is
driving" key, and already proven safe in this viewer — and supersedes it,
because the menu is a strictly better version of the same action: the old ','
blind-toggled between the policy and the LQR, and the menu shows what it is
switching to and offers every other policy besides. The cursor opens on the
OTHER controller, so ',' then ENTER reproduces the old toggle in two
keystrokes. While open the menu borrows the arrows and ENTER, which costs
nothing — throttle and steering are meaningless mid-selection.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from .control.flick import MOVES_DIR

KEY_MENU, KEY_ENTER, KEY_UP, KEY_DOWN = ord(","), 257, 265, 264
# NOT TAB (258): the viewer binds it to the left UI panel and handles it as
# well as passing it on, so it opened the menu and the panel together.

ANALYTIC = "[LQR - analytic]"
"""Sentinel entry. Not a move file: selecting it drops the general policy and
hands the actuators back to the analytic controller, i.e. what ',' does.

ASCII ONLY, and that is not a style preference — MuJoCo's built-in label font
has no glyphs beyond ASCII and renders an em-dash as a hollow box."""


def list_general_policies(moves_dir: Path | str | None = None) -> list[dict]:
    """Every `kind: general` move in moves/, newest first, with its metrics.

    Sorted by mtime rather than by name because the useful ordering while
    driving is "what did I just train", not alphabetical — `general_rl_1k`
    sorting above `general_rl_smooth_stiff` is exactly backwards.

    Malformed or unreadable yaml is skipped rather than raised: this runs to
    build a menu in a live viewer, and one bad file should not end the
    session.
    """
    d = Path(moves_dir or MOVES_DIR)
    out = []
    for p in d.glob("*.yaml"):
        try:
            doc = yaml.safe_load(p.read_text())
        except Exception:
            continue
        if not isinstance(doc, dict) or doc.get("kind") != "general":
            continue
        out.append({"name": p.stem,
                    "metrics": (doc.get("trained") or {}).get("metrics") or {},
                    "digest": str(doc.get("params_digest", "")),
                    "mtime": p.stat().st_mtime})
    out.sort(key=lambda e: -e["mtime"])
    return out


def summarize(entry: dict) -> str:
    """One line of what this policy actually does, for the load message.

    Only the metrics that have ever changed a decision here: tracking, heading
    error, and whether it will reverse — reverse refusal being the failure that
    cost the most time to notice (docs/plans/general-rl-improvements.md §1).
    """
    m = entry["metrics"]
    if not m:
        return "no metrics recorded"
    bits = []
    for key, label, fmt in (("track", "track", "{:.3f}"),
                            ("vel_err", "vel_err", "{:.3f}"),
                            ("head_err_deg", "head", "{:.1f}°"),
                            ("survive_rate", "survive", "{:.2f}")):
        if key in m:
            bits.append(f"{label} " + fmt.format(m[key]))
    # BOTH directions, not just reverse. The original version reported only
    # speed_ratio_rev, on the assumption that reverse refusal was THE failure
    # (docs/plans/general-rl-improvements.md §1) — and then
    # general_rl_smooth_bouncy_lat arrived refusing FORWARD instead
    # (fwd 0.056, rev 0.946) and the menu called it "reverse ok". A direction
    # the policy will not drive is the thing you most need to know before
    # engaging it, whichever direction it is.
    for key, label in (("speed_ratio_fwd", "fwd"), ("speed_ratio_rev", "rev")):
        r = m.get(key)
        if r is not None:
            bits.append(f"{label} " + ("ok" if r > 0.5 else f"REFUSES ({r:.2f})"))
    return "  ".join(bits)


def _camera_basis(cam):
    """Right/up/forward unit vectors for a free camera.

    MuJoCo's azimuth/elevation describe the direction the camera LOOKS, so
    forward points from the eye toward `lookat` and the eye sits behind it at
    `distance`. Rebuilt every frame because teleop drives the camera (chase and
    overhead modes rewrite azimuth/elevation continuously).
    """
    az, el = np.radians(cam.azimuth), np.radians(cam.elevation)
    fwd = np.array([np.cos(el) * np.cos(az),
                    np.cos(el) * np.sin(az),
                    np.sin(el)])
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    n = np.linalg.norm(right)
    # Straight down (overhead mode) makes fwd parallel to world up and the
    # cross product degenerate; fall back to a right vector derived from the
    # azimuth alone, which is what "screen right" means there.
    right = (right / n if n > 1e-6
             else np.array([-np.sin(az), np.cos(az), 0.0]))
    return right, np.cross(right, fwd), fwd


def draw(scn, cam, entries, cursor: int, active: str) -> None:
    """Append the menu to `scn` as labelled marker geoms. Call AFTER the
    overlay that resets ngeom, or the menu is wiped the same frame.

    Placement is in CAMERA space scaled by `cam.distance`, so the panel holds
    its apparent size and screen position as the view zooms — a world-fixed
    offset would swing off screen the moment the chase camera moved.
    """
    import mujoco

    right, up, fwd = _camera_basis(cam)
    d = float(cam.distance)
    origin = np.asarray(cam.lookat, float) + fwd * d * 0.35
    # Upper right, stepping down. Tuned against a 1280x800 window at the
    # default 1.6 m chase distance; these are fractions of `distance`, so they
    # hold as it changes.
    x0, y0, dy = 0.30 * d, 0.24 * d, 0.055 * d

    for i, e in enumerate(entries):
        name = e if isinstance(e, str) else e["name"]
        pos = origin + right * x0 + up * (y0 - i * dy)
        selected = (i == cursor)
        is_active = (name == active)
        # STATE IS CARRIED BY THE TEXT, not by colour. MuJoCo renders geom
        # labels in a fixed white regardless of the geom's rgba, so a colour
        # scheme here is invisible — verified by rendering it. The rgba below
        # only tints the marker dot the label hangs off; '>' marks the cursor
        # and '*' marks what is actually driving, and those two differ
        # whenever you are moving through the list without committing.
        rgba = (np.array([1.0, 1.0, 1.0, 1.0]) if selected else
                np.array([0.4, 1.0, 0.4, 0.9]) if is_active else
                np.array([0.6, 0.6, 0.6, 0.5]))
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.full(3, 0.002 * d), pos, np.eye(3).flatten(),
                            rgba.astype(np.float32))
        g.label = f"{'> ' if selected else '  '}{name}{' *' if is_active else ''}"
        scn.ngeom += 1


def open_cursor(entries, active: str, last_policy: str) -> int:
    """Where the cursor sits when the menu opens: on the controller you are
    NOT currently using, so ',' + ENTER is the old blind toggle.

    Falls back to the first policy in the list when the last one used is gone
    (renamed, or never set because teleop started on the analytic controller).
    """
    want = last_policy if active == ANALYTIC else ANALYTIC
    names = [e if isinstance(e, str) else e["name"] for e in entries]
    if want in names:
        return names.index(want)
    return 1 if len(entries) > 1 else 0


def label_help() -> str:
    return (", policy menu (↑/↓ choose, ENTER load, , closes) — "
            "* is driving now, > is the cursor")
