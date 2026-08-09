"""An in-scene menu for picking which controller drives, live, in teleop.

WHY IT LOOKS LIKE THIS. A real mouse-driven dropdown is not available:
`mujoco.viewer.launch_passive` exposes exactly three hooks — `key_callback`,
`show_left_ui`, `show_right_ui` — and no way to add a widget to either panel
(the sections in the C `simulate` app are built with mjUI, which the Python
viewer does not surface). There is no 2D overlay hook either; the only thing
this process can put on screen is scene geometry. So the menu is drawn as
mjvGeom LABELS anchored in front of the camera, which reads as a panel and
tracks the view, but is driven by keys rather than the mouse.

Keys are chosen around a hard constraint: MuJoCo's viewer owns every letter
A-Z, and teleop already spends 0-9, the arrows, and most punctuation. The menu
therefore takes ONE key (TAB) and, while it is open, borrows the arrows and
Enter — which costs nothing, because throttle and steering are meaningless
during a selection anyway.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from .control.flick import MOVES_DIR

KEY_TAB, KEY_ENTER, KEY_UP, KEY_DOWN = 258, 257, 265, 264

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
    rev = m.get("speed_ratio_rev")
    if rev is not None:
        bits.append("reverse " + ("ok" if rev > 0.5 else f"REFUSES ({rev:.2f})"))
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


def label_help() -> str:
    return ("TAB policy menu (↑/↓ choose, ENTER load, TAB close) — "
            "* is driving now")
