"""The swing-linkage stepthrough page: righting designs stepped through their stroke.

RENDER IT (from the repo root):

    python analysis/swing_stepthrough.py --sim     # ~17 s: the tracked page, sim rows filled
    python analysis/swing_stepthrough.py           # ~6 s: the same, sim rows read "not run"

Either writes analysis/plots/swing_stepthrough.html (self-contained); open it
in a browser. Re-run after editing any file a design reads -- nothing
re-renders on its own. Commit the page from a --sim run, so its sim numbers
are real.

THE DESIGNS (`DEFAULT`, in this file):

    V1   config/swing_linkage_smaller.yaml      the earlier four-bar, flat panels
    V2   config/swing_linkage_shared_rod.yaml   the bike's righting module, as the
         + config/righting_blade.yaml           sim builds it by default
    V3   V2's linkage                           exploring: a new blade
         + config/swing_explore/blade_v3.yaml

TO CHANGE A BLADE, edit its yaml and re-render. The outline's entries --
points, straight runs at an angle, cubic curves -- are documented in
`aow_sim.params.blade_points`; the right wing seen from behind, x out from the
midline, y up from the floor. A blade other than the module's own (V3) gets,
on the page and in its sim: its own outline, its file's `mass`, and its
commanded stroke set to its LEVEL point, worked out here (or its file's
`crank_travel_deg`, if given, pins it). Its dropdown on the page lists the
file. To try blades without adding them to `DEFAULT`, give one
or more (each is the V2 linkage with that blade); the two panes open on the
last two, and `--labels` names them:

    python analysis/swing_stepthrough.py --blade config/swing_explore/my_blade.yaml
    python analysis/swing_stepthrough.py --sim \
        --blade config/righting_blade.yaml config/swing_explore/my_blade.yaml --labels V2 mine

That page holds only the `--blade` entries and goes to
analysis/swing_stepthrough_custom.html (scratch, gitignored; `--out` to put it
elsewhere), so the tracked page only changes on a plain run. To add a design
for good, add a line to `DEFAULT`.

THE PAGE: two panes, each a design picked from a dropdown, under one stroke
slider; each shows the mechanism in the bike frame and the bike resting on
the floor, both seen from behind, with its summary, diagnostics (clearances:
red touching, amber under target) and files; then torque, pin-force, ratio
and roll curves for both. Notes under "More info".

HOW (for changing this file): the configs are the sim's own
(`build_model(..., swing_linkage_cfg=)`), posed with
`swing_linkage.SwingLinkage` (held to the builder's own solver by
tests/test_hw_telemetry.py) and scored with `swing_synthesis.stroke`. A CAD
entry (V2, V3) is the diamond exactly as `cad_righting` builds it -- links at
`linkage.scale` (1.25), the XC330 long end DOWN, every righting part's front
section posed with the CAD's own group transforms (no Onshape call) -- with
the blade swept on each wing; past its commanded stroke it runs on to where
the two toes meet (the stop). Its diagnostics: that angle, where the links
themselves first touch, and the blade's least clearance to every part
sharing its fore/aft span and to the floor with the bike standing on its
wheels. The bike rests on the blade's whole section (a curve's facets
interpolated, `swing_linkage.rest_on_points`), with that bike's mass
properties (`swing_linkage.mass_props`). `--sim` runs each in the sim at
9.9 V -- the righting sweep's own `fallen` and `trial`, goal stepped as
teleop does -- bisecting the Goal Current to 10 mA, plus a slow ramped
stroke for the faded chart traces. The servo case is drawn DOWN whatever the
1 mm rule says (the rule keeps the case off the hinge rod; the CAD breaks it
on purpose).
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import swing_linkage as sl  # noqa: E402
import swing_synthesis as ss  # noqa: E402
from aow_sim.build_model import SWING_LINKAGE_CFG, load_params  # noqa: E402
from aow_sim.params import blade_points  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = Path(__file__).resolve().parent / "templates" / "swing_stepthrough.html"
OUT = Path(__file__).resolve().parent / "plots" / "swing_stepthrough.html"
# A page of other entries (--blade, config arguments): scratch, gitignored.
OUT_CUSTOM = Path(__file__).resolve().parent / "swing_stepthrough_custom.html"
# The XC330 case in the FRONT view (shaft along fore/aft): 20 x 34 mm, the
# shaft 7.5 mm off centre along the 34 -- so 9.5 mm from one end, 24.5 from
# the other (docs/cad/cad_layout.yaml, `righting.linkage_crank_servo`,
# envelope and note). The body sits behind the horn, in the same fore/aft
# span as the panels (60-150 mm in CAD) and the wing hinge rod, so those are
# what it can hit; the links are in front of the horn.
CASE_W, CASE_L, SHAFT_FROM_END = 20.0, 34.0, 9.5
ORIENT = {"down": (0.0, -1.0), "up": (0.0, 1.0)}      # tried in this order
FIT_MM = 1.0         # least clearance that counts as fitting
ROD_R = 1.6          # a 1/8 in hinge rod
PANEL_HALF = 3.0     # the panel box's half-thickness in the sim
SERVO_CLEAR_MM = 5.0  # the blade to the servo and its cases (user, 2026-10-09)
# Least clearance between two parts of different groups that share fore/aft
# space, below which the page WARNS. 2 mm (user, 2026-10-09), so the knuckle
# on the other wing's coupler (1.81 at the ~130 toe stop) shows as a warning
# on purpose: the toes stop the crank before the links meet, and the hard
# floor is test_cad_righting's 1.5.
LINK_CLEAR_MM = 2.0
LINK_SHOW_MM = 8.0    # pairs listed in the diagnostics: closer than this


def servo_rect(shaft, orient: str):
    """(ymin, ymax, zmin, zmax) of the case, long axis along `orient`."""
    ay, az = ORIENT[orient]
    far, near = CASE_L - SHAFT_FROM_END, SHAFT_FROM_END
    if ay == 0.0:
        lo, hi = (shaft[1] - near, shaft[1] + far) if az > 0 else (shaft[1] - far, shaft[1] + near)
        return (shaft[0] - CASE_W / 2, shaft[0] + CASE_W / 2, lo, hi)
    lo, hi = (shaft[0] - near, shaft[0] + far) if ay > 0 else (shaft[0] - far, shaft[0] + near)
    return (lo, hi, shaft[1] - CASE_W / 2, shaft[1] + CASE_W / 2)


def rect_sdf(p, r) -> float:
    """Signed distance from point p to an axis-aligned rectangle (- inside)."""
    cy, cz = (r[0] + r[1]) / 2, (r[2] + r[3]) / 2
    hy, hz = (r[1] - r[0]) / 2, (r[3] - r[2]) / 2
    dy, dz = abs(p[0] - cy) - hy, abs(p[1] - cz) - hz
    out = np.hypot(max(dy, 0.0), max(dz, 0.0))
    return float(out + min(max(dy, dz), 0.0))


def servo_fit(lk, T: float, shaft) -> dict:
    """The case's orientation: DOWN (long end toward the wing hinge) if it
    clears the hinge rod(s) and both panels by FIT_MM over the whole stroke,
    both directions; else UP; else None, not viable. The user's rule."""
    out = {}
    for name in ORIENT:
        worst, what = np.inf, ""
        for mirror in (1.0, -1.0):
            r = servo_rect([mirror * shaft[0], shaft[1]], name)
            if mirror < 0:
                r = (-r[1], -r[0], r[2], r[3])
            for t in np.linspace(0.0, T, 61):
                for side in (-1, 1):
                    pz = lk.pose(side, float(t))
                    c = rect_sdf(pz["pivot"], r) - ROD_R
                    if c < worst:
                        worst, what = c, "hinge rod"
                    for k in np.linspace(0.0, 1.0, 25):
                        q = pz["foot"] + k * (pz["top"] - pz["foot"])
                        c = rect_sdf(q, r) - PANEL_HALF
                        if c < worst:
                            worst, what = c, "wing panel"
        out[name] = {"clear": round(float(worst), 1), "what": what}
    for name in ORIENT:
        if out[name]["clear"] >= FIT_MM:
            return {"case": name, **out[name]}
    return {"case": None, **out["down"]}


def _travel_deg(params, path: Path | None) -> float:
    """`stroke.crank_travel_deg` of a config, or of the bike's righting module
    (V2) for None -- which is also what `rcs.fallen(..., None)` builds."""
    cfg = (params["righting"]["module"]["linkage"] if path is None
           else yaml.safe_load(path.read_text()))
    return float(cfg["stroke"]["crank_travel_deg"])


def lift_pin_peak(params, path: Path | None, counts: int = 700, supply: float = 9.9) -> float:
    """Peak coupler-pin force [N] in the sim while the stepped stroke lifts the
    bike to upright (not the fall after it). On every design it lands in the
    first 1-3 ms: the servo stepping from holding to full current against a
    bike that has not moved yet. Not a solver artifact -- with nothing
    commanded the same start reads only the static load."""
    import mujoco
    import righting_current_sweep as rcs
    from aow_sim.control.righting import roll_pitch
    from aow_sim.righting_servo import CURRENT_LIMIT
    dep = np.deg2rad(_travel_deg(params, path))
    m, d0, srv, hooks = start = rcs.fallen(params, 1.0, 2.0, path)
    best = {g: rcs.trial(params, 1.0, g, CURRENT_LIMIT, 1.0, 4.0, 5.0, start)["min_roll"]
            for g in (dep, -dep)}
    d = mujoco.MjData(m)
    d.qpos[:], d.qvel[:] = d0.qpos, d0.qvel
    mujoco.mj_forward(m, d)
    srv.set_supply(supply / 12.0)
    srv.set_goal_current(counts)
    d.ctrl[m.actuator("swing").id] = min(best, key=best.get)
    peak, done = [0.0], [False]

    def on(dd):
        if done[0]:
            return
        rows = np.flatnonzero(dd.efc_type[:dd.nefc] == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
        for e in range(m.neq):
            r = rows[dd.efc_id[rows] == e]
            if len(r):
                peak[0] = max(peak[0], float(np.linalg.norm(dd.efc_force[r])))
        done[0] = abs(roll_pitch(dd.qpos[3:7])[0]) < 5.0
    rcs.step(m, d, hooks, 4.0, on)
    return round(peak[0], 1)


def sim_trace(params, path: Path | None, end_deg: float, slew_dps: float = 40.0,
              supply: float = 9.9, every: int = 5) -> dict:
    """The sim through the stroke, for the charts: the bike settled on its
    side as `lift_pin_peak` starts it, then the crank goal RAMPED at
    `slew_dps` (slow, so it reads against the quasi-static curves) to
    `end_deg` -- past the flat for a CAD entry, where only the sim has an
    answer. Goal Current at the firmware limit, so the servo never stalls
    and the torque is what the stroke asks. Samples [crank deg, servo torque
    N m, most-loaded pin N, roll deg] every `every` steps. The sim has its
    own flat panel and no toe stop: past the flat it is the bike falling
    over centre, nothing catching it."""
    import mujoco
    import righting_current_sweep as rcs
    from aow_sim.control.righting import roll_pitch
    from aow_sim.righting_servo import CURRENT_LIMIT
    dep = np.deg2rad(_travel_deg(params, path))
    m, d0, srv, hooks = start = rcs.fallen(params, 1.0, 2.0, path)
    best = {g: rcs.trial(params, 1.0, g, CURRENT_LIMIT, 1.0, 4.0, 5.0, start)["min_roll"]
            for g in (dep, -dep)}
    sgn = float(np.sign(min(best, key=best.get)))
    d = mujoco.MjData(m)
    d.qpos[:], d.qvel[:] = d0.qpos, d0.qvel
    mujoco.mj_forward(m, d)
    srv.set_supply(supply / 12.0)
    srv.set_goal_current(CURRENT_LIMIT)
    aid = m.actuator("swing").id
    qadr = m.jnt_qposadr[m.actuator_trnid[aid, 0]]
    end, rate = np.deg2rad(end_deg), np.deg2rad(slew_dps)
    out, k = [], [0]

    def on(dd):
        dd.ctrl[aid] = sgn * min(dd.time * rate, end)
        k[0] += 1
        if k[0] % every:
            return
        rows = np.flatnonzero(dd.efc_type[:dd.nefc] == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
        pin = max((float(np.linalg.norm(dd.efc_force[rows[dd.efc_id[rows] == e]]))
                   for e in range(m.neq)), default=0.0)
        # The crank's ABSOLUTE angle, as every other curve on the page uses.
        # It was measured from `q0`, where the fall left it (back-driven
        # ~10 deg on V2), which slid the whole trace ~10 deg to the right.
        out.append([float(np.degrees(sgn * dd.qpos[qadr])), float(sgn * srv.torque),
                    pin, float(roll_pitch(dd.qpos[3:7])[0])])
    rcs.step(m, d, hooks, end_deg / slew_dps + 1.0, on)
    a = np.array(out)
    a[:, 3] *= np.sign(a[0, 3]) or 1.0          # roll positive on the side it starts
    return {"samples": a, "slew": slew_dps}


def on_frames(trace: dict, frames: list) -> None:
    """The trace onto the page's frames by crank angle: each frame takes the
    first sample to reach its angle; frames the sim never reached stay empty."""
    a = trace["samples"]
    reach = np.maximum.accumulate(a[:, 0])
    for fr in frames:
        j = int(np.searchsorted(reach, fr["t"] - 1e-6))
        ok = j < len(a)
        fr["sim_m"] = round(float(a[j, 1]), 4) if ok else None
        fr["sim_p"] = round(float(a[j, 2]), 1) if ok else None
        fr["sim_r"] = round(float(a[j, 3]), 2) if ok else None


DIAMOND = ROOT / "config" / "swing_linkage_shared_rod.yaml"
# (config, label, blade): blade None is a config design as the sim builds it;
# a blade yaml (or "cad", the CAD's own dummy) makes it the CAD entry -- the
# diamond as cad_righting builds it, with that blade swept.
DEFAULT = [
    (SWING_LINKAGE_CFG, "V1", None),                                       # built today
    (DIAMOND, "V2", ROOT / "config" / "righting_blade.yaml"),  # the bike's module
    (DIAMOND, "V3", ROOT / "config" / "swing_explore" / "blade_v3.yaml"),  # exploring
]


def blade_params(params: dict, spec: dict | None) -> dict:
    """The bike with `spec`'s blade on the righting module: the live params
    for the module's own blade (or the CAD dummy), else a copy with the
    outline, span and -- if the file gives one -- the blade mass swapped.
    What a CAD entry's rest, wing CoM and sim are computed on."""
    mod = params["righting"]["module"]
    if spec is None or (ROOT / mod["blade_file"]).resolve() == (ROOT / spec["_file"]).resolve():
        return params
    p = copy.deepcopy(params)
    m = p["righting"]["module"]
    m["blade_outline"] = blade_points(spec["outline"])
    m["blade_y"] = [float(v) for v in spec["y"]]
    if "mass" in spec:
        m["blade"]["mass"] = float(spec["mass"]["value"])
    if "crank_travel_deg" in spec:     # the blade's own level point
        m["linkage"]["stroke"]["crank_travel_deg"] = float(spec["crank_travel_deg"])
    return p
NOTES = [
    "<b>V1.</b> Linkage V1 (swing_linkage_smaller.yaml), the earlier four-bar, from its config.",
    "<b>V2.</b> Linkage V2 (swing_linkage_shared_rod.yaml), from the CAD: the bike's righting "
    "module, what the sim builds by default, blades from righting_blade.yaml.",
    "<b>V3.</b> Exploring (2026-10-10): V2's linkage with a new blade, "
    "swing_explore/blade_v3.yaml -- the outer face vertical at stow, V2's "
    "inner face and toe kept, curving into the toe (a cubic, tangent to both; the face angle one number). "
    "Its rest, wing CoM and sim carry "
    "that blade, its mass (38.8 g each, GUESS; V2's 28.6) and its own stroke, to its level point.",
    "<b>Where V1 and V2 come from.</b> V2's CAD is generated as solid parts "
    "(cad_righting, 1.25x, with righting_blade.yaml), so the page reads its real "
    "part outlines and checks the blade against them. V1's CAD is a 2D "
    "sketch generated from the same config (cad_swing_linkage): lines, no "
    "parts. So V1 is drawn and checked from its config's links and limits.",
    "<b>View.</b> The bike is drawn seen from behind: the right wing on the "
    "right, as the blade files' (x, y) have it.",
    "<b>Colours.</b> Orange: the wing that pushes the bike up (the right one). Blue: the one "
    "that rises and tucks in. Burgundy: the servo's crank. The slider runs the "
    "stroke from lying on its side (0%) to the commanded stroke (100%); V2 runs "
    "on past it to where its toes meet.",
    "<b>Quasi-static.</b> At each crank angle the linkage is solved, the bike "
    "set where it rests on the floor (V2 on its blade's whole section, toe and "
    "both faces; V1 on its panel line), and every force is a static balance: no "
    "speed, inertia or impacts. Past level the bike falls over centre, "
    "which has no static balance; only the sim has values there.",
    "<b>mA.</b> The servo's Goal Current: the ceiling on the current its "
    "position loop may use. <i>Current limit @ 9.9 V (sim)</i>: the lowest "
    "ceiling that still rights the bike in the sim on a low battery. "
    "<i>Planar</i>: the quasi-static estimate, 25-45 mA high. The % is of "
    "812 mA, the most the motor can draw at 9.9 V (stalled).",
    "<b>Sim traces.</b> The faded lines on the torque, pin and roll charts: "
    "the sim, the crank driven slowly through the whole stroke.",
    "<b>Pin forces.</b> <i>F in coupler/hinge pins</i>: the most on the "
    "coupler and hinge pins, quasi-static; the pin colours are these. <i>F "
    "coupler pin, step (sim)</i>: the peak when the servo is told to go "
    "straight to full travel from lying on its side. It comes in the first "
    "few milliseconds, before the bike moves.",
    "<b>Diagnostics.</b> Each design's least clearances over the stroke, "
    "worst first; red is a hit or under target. V2: the blade against every "
    "part it shares fore/aft space with, and the floor, plus where the toes "
    "meet (the stop) and the links touch. V1: its links and panels against "
    "its config's limits, and the floor.",
    "<b>Layers.</b> Parts in different fore/aft layers cross in the front "
    "view without touching. Back on the left.",
    "<b>Other marks.</b> The dashed outline round the crank shaft is the "
    "servo. The dashed yellow box is where panels must not go. The grey bar "
    "is the wheel end-on.",
]


def plane_of(lk, member: str):
    return lk.planes.get(member, lk.planes.get(member[:-1]))


def sim_counts(params, path: Path | None, supply: float = 9.9) -> int:
    """Lowest Goal Current (10-count steps) that rights the bike, stepped."""
    import righting_current_sweep as rcs
    from aow_sim.righting_servo import CURRENT_LIMIT
    dep = np.deg2rad(_travel_deg(params, path))
    start = rcs.fallen(params, 1.0, 2.0, path)
    best = {g: rcs.trial(params, 1.0, g, CURRENT_LIMIT, 1.0, 4.0, 5.0, start)["min_roll"]
            for g in (dep, -dep)}
    goal = min(best, key=best.get)
    ok = lambda c: rcs.trial(params, 1.0, goal, c, supply / 12.0, 4.0, 5.0,  # noqa: E731
                             start)["righted"]
    lo, hi = 300, CURRENT_LIMIT
    if not ok(hi):
        return -1
    while hi - lo > 10:
        mid = (lo + hi) // 2
        lo, hi = (lo, mid) if ok(mid) else (mid, hi)
    return hi


def _section(q):
    """One cad_righting primitive as (front section in module (x, z), y0, y1)."""
    from shapely.geometry import MultiPoint, Point, Polygon, box
    from shapely.ops import unary_union
    from aow_sim import cad_righting as cr
    k = q["k"]
    if k == "cyl":
        return Point(q["x"], q["z"]).buffer(q["r"], 16), q["y0"], q["y1"]
    if k == "prism":
        return Polygon(q["pts"]).buffer(0), q["y0"], q["y1"]
    if k == "slot":
        return unary_union([Polygon(q["pts"]).buffer(0), Point(q["a"]).buffer(q["ra"], 16),
                            Point(q["b"]).buffer(q["rb"], 16)]), q["y0"], q["y1"]
    if k == "box":
        return box(q["lo"][0], q["lo"][2], q["hi"][0], q["hi"][2]), q["lo"][1], q["hi"][1]
    if k == "poly":
        c = cr.poly_corners(q)
        return MultiPoint(c[:, [0, 2]]).convex_hull, float(c[:, 1].min()), float(c[:, 1].max())
    if k == "prismX":
        p = np.array(q["pts"])
        return (box(q["x0"], p[:, 1].min(), q["x1"], p[:, 1].max()),
                float(p[:, 0].min()), float(p[:, 0].max()))
    raise ValueError(f"no section for a {k}")


def _rings(geom) -> list:
    """Exterior rings of a (multi)polygon, rounded, for the page."""
    gs = getattr(geom, "geoms", [geom])
    return [[[round(x, 1), round(z, 1)] for x, z in g.exterior.coords] for g in gs if not g.is_empty]


def cad_parts(blade: dict | None):
    """The righting module from `cad_righting`, ready to pose: the linkage,
    the layout, and each part's front-section pieces with their fore/aft
    span. The CAD's own blades are replaced by `blade` (or kept as the
    default outline) and swept as wR / wL."""
    from shapely.geometry import Polygon, box
    from shapely.ops import unary_union
    from aow_sim import cad_righting as cr
    data = cr.load()
    L = cr.layout(data)
    parts, cad_blade = [], None
    for p in L["parts"]:
        if p["name"] == "floor (mock)" or not p["add"]:
            continue
        pieces = [_section(q) for q in p["add"]]
        if p["name"] == "wing R blade":
            cad_blade = (unary_union([g for g, _, _ in pieces]),
                         min(a for _, a, _ in pieces), max(b for _, _, b in pieces))
        if p["name"].endswith("blade"):
            continue
        if p["name"] in ("wing R", "wing L"):
            # The blade (toe and all) IS the panel; of the CAD's printed wing
            # only its tabs stand -- the 6 mm plates up to the rocker and
            # knuckle. The stub under the blade is dropped.
            pieces = [_section(q) for q in p["add"]
                      if q["k"] == "prism" and q["y1"] - q["y0"] <= 6.5]
        parts.append({"n": p["name"], "g": p["group"], "pieces": pieces})
    # The XC330 itself, long end DOWN (the CAD's `cases.servo_end`), horn
    # forward: its case runs back from the horn hub's rear face, past the horn.
    cz = float(L["C"][1])
    hw, far = CASE_W / 2, CASE_L - SHAFT_FROM_END
    hub = next(p for p in L["parts"] if p["name"] == "horn hub")
    y1 = min(q["y0"] for q in hub["add"]) - data["x330"]["hornThickness"]
    parts.append({"n": "XC330", "g": "fixed",
                  "pieces": [(box(-hw, cz - far, hw, cz + SHAFT_FROM_END),
                              y1 - data["x330"]["caseDepth"], y1)]})
    fz = float(L["floor_z"])
    if blade is None:
        geom, y0, y1 = cad_blade
        src = "the CAD's dummy U blade"
    else:
        geom = Polygon([(o, u + fz) for o, u in blade_points(blade["outline"])]).buffer(0)
        y0, y1 = (float(v) for v in blade["y"])
        src = blade.get("_file", "--blade")
    return data, L, parts, {"geom": geom, "y": (y0, y1), "src": src,
                            "travel": None if blade is None else blade.get("crank_travel_deg"),
                            "summary": None if blade is None else blade_summary(blade, geom)}


def blade_summary(blade: dict, geom) -> list:
    """The blade file as the page's dropdown lists it, [label, value] rows:
    its outline as written (a curve's control point marked), span, stroke
    and mass -- the file at a glance, not its flattened points."""
    pt = lambda p: f"({p[0]:g}, {p[1]:g})"  # noqa: E731
    rows = [["outline (x, y) mm, right wing from behind", ""]]
    k = 0
    for e in blade["outline"]:
        k += 1
        if isinstance(e, dict) and "curve" in e:
            c = e["curve"]
            h = ", ".join(f"{v:g}" for v in c["handles"])
            rows.append([f"  {k} curve to", pt(c["to"])])
            if c.get("start_deg", 0) or c.get("end_deg", 0):     # a deliberate kink
                rows.append(["      off tangent, start / end [deg]",
                             f"{c.get('start_deg', 0):g} / {c.get('end_deg', 0):g}"])
            rows.append(["      handles [mm]", h])
        elif isinstance(e, dict):
            rows.append([f"  {k} run, angle from level [deg] / length [mm]",
                         f"{e['dir_deg']:g} / {e['len']:g}"])
        else:
            rows.append([f"  {k} point", pt(e)])
    y0, y1 = blade["y"]
    rows += [["span fore/aft [mm]", f"{y1 - y0:g}"],
             ["section [mm²]", f"{geom.area:.0f}"]]
    if "crank_travel_deg" in blade:
        rows.append(["commanded stroke [deg], pinned", f"{blade['crank_travel_deg']:g}"])
    if "mass" in blade:
        rows.append([f"mass each [g] ({blade['mass'].get('source', '')})",
                     f"{blade['mass']['value'] * 1000:g}"])
    return rows


def _posed(geom, g):
    from shapely import affinity
    if g is None:
        return geom
    out = affinity.rotate(geom, -g["a"], origin=tuple(g["p"]))
    return affinity.translate(out, g["d"][0], g["d"][1])


def blade_meet(data, L, bl) -> float | None:
    """The crank angle where the two blades first touch, or None: deploying R
    against the tucking L. Both hang off the crank, so a touch is a hard stop
    for the whole mechanism; mirror-symmetric, so the same angle stops the
    other end. Searched up to the deploying side's toggle, past the flat."""
    from shapely import affinity
    from aow_sim import cad_righting as cr
    lk = data["lk"]
    bR = bl["geom"]
    bL = affinity.scale(bR, -1.0, 1.0, origin=(0, 0))
    for t in np.arange(0.0, sl.critical_angles(lk).deploy, 0.25):
        P = cr.poses(lk, float(t), L["C"], L["pin"], L["J"])
        if _posed(bR, P["wR"]).intersection(_posed(bL, P["wL"])).area > 1e-3:
            return round(float(t), 2)
    return None


def link_limit(data, L, parts, T: float) -> dict | None:
    """Past the flat, the first crank angle where two parts of different
    groups touch that never touched in the stroke: the mechanism's own end
    of travel, blades aside. Parts that always overlap in section (a pin in
    its boss, layers that share a span) are held to their worst in-stroke
    overlap. 0.5 deg steps up to the deploying side's toggle."""
    from itertools import combinations
    from aow_sim import cad_righting as cr
    lk = data["lk"]

    def overlaps(t):
        P = cr.poses(lk, float(t), L["C"], L["pin"], L["J"])
        ps = [(p["n"], p["g"], [(_posed(g, P.get(p["g"])), a, b) for g, a, b in p["pieces"]])
              for p in parts]
        return {(n1, n2): sum(ga.intersection(gb).area for ga, a0, a1 in A for gb, b0, b1 in B
                              if a1 > b0 and a0 < b1)
                for (n1, g1, A), (n2, g2, B) in combinations(ps, 2) if g1 != g2}
    base: dict = {}
    for t in np.linspace(0.0, T, 13):
        for k, v in overlaps(t).items():
            base[k] = max(base.get(k, 0.0), v)
    for t in np.arange(np.ceil(T * 2) / 2, sl.critical_angles(lk).deploy, 0.5):
        new = [k for k, v in overlaps(t).items() if v - base[k] > 0.1]
        if new:
            return {"t": float(t), "what": " / ".join(sorted({" on ".join(k) for k in new}))}
    return None


def cad_sweep(data, L, parts, bl, frames, pz: float, T: float, meet) -> dict:
    """Pose every part and both blades at each frame; least clearance of each
    blade to everything it shares fore/aft span with, over the stroke."""
    from shapely import affinity
    from aow_sim import cad_righting as cr
    lk = data["lk"]
    fz = float(L["floor_z"])
    bR = bl["geom"]
    bL = affinity.scale(bR, -1.0, 1.0, origin=(0, 0))
    y0, y1 = bl["y"]
    best: dict[str, dict] = {}

    def note(key, v, t, kind="mm"):
        if key not in best or v < best[key]["v"]:
            best[key] = {"v": round(float(v), 2), "t": round(float(t), 1), "kind": kind}

    # PART AGAINST PART, as V1's coupler check does for a config: every pair of
    # parts in different groups sharing fore/aft span, at every frame (the
    # frames run on to the toe stop). Pairs already overlapping in section at
    # stow -- a pin in its boss, a hub on the rod, faces that bear -- are
    # joints, not clearances, and are left out.
    from itertools import combinations
    P0 = cr.poses(lk, 0.0, L["C"], L["pin"], L["J"])
    def sec(part, P):
        return [(_posed(g, P.get(part["g"])), a, c) for g, a, c in part["pieces"]]
    def share(A, B):
        return any(a1 > b0 and a0 < b1 for _, a0, a1 in A for _, b0, b1 in B)
    pairs = []
    for p1, p2 in combinations(parts, 2):
        if p1["g"] == p2["g"] or (p1["g"] == "fixed" and p2["g"] == "fixed"):
            continue
        A, B = sec(p1, P0), sec(p2, P0)
        if not share(A, B):
            continue
        if any(ga.intersection(gb).area > 1e-3 for ga, a0, a1 in A for gb, b0, b1 in B
               if a1 > b0 and a0 < b1):
            continue
        pairs.append((p1, p2))
    links: dict[str, dict] = {}

    poses = []
    for fr in frames:
        t = fr["t"]
        P = cr.poses(lk, t, L["C"], L["pin"], L["J"])
        tight = None
        for p1, p2 in pairs:
            A, B = sec(p1, P), sec(p2, P)
            v = min(ga.distance(gb) - (ga.intersection(gb).area > 1e-3) * ga.intersection(gb).area
                    for ga, a0, a1 in A for gb, b0, b1 in B if a1 > b0 and a0 < b1)
            key = f"{p1['n']} x {p2['n']}"
            if key not in links or v < links[key]["v"]:
                links[key] = {"v": round(float(v), 2), "t": round(float(t), 1), "kind": "mm"}
            if tight is None or v < tight[1]:
                tight = (key, float(v))
        fr["link"] = None if tight is None else [tight[0], round(tight[1], 2)]
        poses.append({g: {k: [round(float(x), 3) for x in v] if isinstance(v, list)
                          else round(float(v), 3) for k, v in P[g].items()}
                      for g in ("crank", "wR", "wL", "cR", "cL")})
        blades = {"R": _posed(bR, P["wR"]), "L": _posed(bL, P["wL"])}
        for side, b in blades.items():
            own = "w" + side
            for part in parts:
                if part["g"] == own:
                    continue
                for geom, a, c in part["pieces"]:
                    if c <= y0 or a >= y1:
                        continue
                    pg = _posed(geom, P.get(part["g"]))
                    hit = b.intersection(pg).area
                    if hit > 1e-3:
                        note(f"{part['n']}", -hit, t, "mm2")
                    else:
                        note(f"{part['n']}", b.distance(pg), t)
            # Upright on the wheels only within the stroke (driving, or the
            # wing stowing after a righting); past the flat the bike stands
            # on the blade by construction.
            if t <= T + 1e-6:
                note("floor, bike standing on its wheels", min(z for _, z in b.exterior.coords) - fz, t)
        # The blades are not checked against each other: they may cross
        # (user, 2026-10-09), e.g. toes under the bike in different layers.
    rows = sorted(({"what": k, **v} for k, v in best.items()),
                  key=lambda r: (r["kind"] != "mm2", r["v"]))
    for r in rows:
        r["target"] = SERVO_CLEAR_MM if r["what"] in ("XC330", "lower case") else 0.0
    # The parts against each other, up to the toe stop: the closest pairs.
    rows += sorted(({"what": k, **v, "target": LINK_CLEAR_MM} for k, v in links.items()
                    if v["v"] < LINK_SHOW_MM), key=lambda r: r["v"])
    # A CAD design FAILS only by touching (overlap, or under 0 mm); under its
    # target it WARNS (user, 2026-10-09). A config design's targets stay hard.
    for r in rows:
        r["fail"] = 0.0
    # Layers, numbered from the back: the moving parts' fore/aft faces,
    # merged within 1 mm, cut the span into cells; each part lists the
    # cells it fills. The pins and the washer only thread through them.
    moving = [p for p in parts if p["g"] in ("crank", "cR", "cL", "wR", "wL")
              and p["n"] != "centre washer"]
    ends = sorted(v for p in moving if "pin" not in p["n"]
                  for _, a, b in p["pieces"] for v in (a, b))
    cuts = [ends[0]]
    for v in ends[1:]:
        if v - cuts[-1] > 1.0:
            cuts.append(v)
    layers = []
    for p in moving:
        idx = sorted({k + 1 for _, a, b in p["pieces"] for k in range(len(cuts) - 1)
                      if min(b, cuts[k + 1]) - max(a, cuts[k]) > 1.0})
        layers.append({"n": p["n"], "g": p["g"], "l": idx})
    layers.sort(key=lambda q: (q["l"][0] if q["l"] else 0, len(q["l"])))
    return {"pz": pz, "floor_z": fz, "poses": poses, "src": bl["src"], "y": [y0, y1],
            "layers": layers,
            "meet": meet, "T": round(float(T), 1), "limit": link_limit(data, L, parts, T),
            "link_target": LINK_CLEAR_MM,
            "parts": [{"n": p["n"], "g": p["g"],
                       "rings": [r for geom, _, _ in p["pieces"] for r in _rings(geom)]}
                      for p in parts],
            "blade": {"R": _rings(bR)[0], "L": _rings(bL)[0]}, "clear": rows}


def rest_over_centre(lk, t: float, pts=None) -> dict:
    """Past level: the bike on the deploying wing's far point and the wheels,
    CoM beyond the wheels -- over centre, tipping toward the other side.
    `resting_pose` refuses it (no static rest); for the picture it is posed
    on that edge anyway, the commanded crank fixing the roll. `pts`: the
    wing's support points at `t` (the blade), else the panel's two ends."""
    pz = lk.pose(-1, t)
    pts = [pz["top"], pz["foot"]] if pts is None else list(pts)
    wheel = np.array([0.0, -lk.wheel_radius])
    for end in pts:
        v = end - wheel
        roll = -np.degrees(np.arctan2(v[1], v[0]))
        if roll > 90:
            roll -= 180
        elif roll < -90:
            roll += 180
        floor = sl._rot(wheel, roll)[1]
        if all(sl._rot(q, roll)[1] >= floor - 1e-6 for q in pts):
            return {"to_floor": (float(roll), float(floor)), "regime": "over"}
    raise ValueError(f"no over-centre rest at crank {t:.1f}")


def level_point(lk, blade0, mp: dict, upto: float | None) -> float | None:
    """The crank angle where the bike, resting on the blade (`blade0`, sketch
    mm at stow) and its wheels, comes to roll 0: stepped at 0.5 deg, then
    bisected to 1e-4 deg. None if it never does before `upto` (the toe stop)
    or the deploying side's toggle."""
    def roll(t):
        r = sl.resting_pose_on(lk, t, blade0, mp["wing_com"], mp=mp)
        return -1.0 if r is None else r["to_floor"][0]    # no rest: over centre
    top = upto if upto is not None else sl.critical_angles(lk).deploy
    prev = 0.0
    for t in np.arange(0.5, top + 1e-9, 0.5):
        if roll(t) <= 0:
            lo, hi = prev, float(t)
            while hi - lo > 1e-4:
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if roll(mid) > 0 else (lo, mid)
            return lo
        prev = float(t)
    return None


def blade_in_sketch(lk, blade: dict) -> np.ndarray:
    """The blade's section at stow (cad_parts' polygon: module frame, mm out
    from the centreline and up from the wing rod) in the planar model's
    sketch frame (right wing at -x, z from the axle). Its vertices are the
    supports `resting_pose_on` turns with the wing."""
    xy = np.asarray(blade["geom"].exterior.coords, float)[:-1]
    return np.column_stack([-xy[:, 0], xy[:, 1] + float(lk.pivot(-1)[1])])


def turned_points(lk, t: float, pts0) -> np.ndarray:
    """`pts0` (sketch mm, at stow) turned with the deploying wing to crank t."""
    p0, pz = lk.pose(-1, 0.0), lk.pose(-1, t)
    h = np.asarray(pz["pivot"], float)
    a = (np.arctan2(*(pz["top"] - pz["foot"])[::-1])
         - np.arctan2(*(p0["top"] - p0["foot"])[::-1]))
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    return (np.asarray(pts0, float) - h) @ R.T + h


def _seg_dist(a, b, c, d) -> float:
    """Least distance between segments ab and cd (2D)."""
    from shapely.geometry import LineString
    return float(LineString([a, b]).distance(LineString([c, d])))


def config_diag(lk, cfg: dict, frames: list, T: float, planes: dict, fit: dict) -> list:
    """A config design's checks, in the CAD entry's row format: the servo
    case (its own 1 mm rule), the couplers if they share a plane, the panels
    against the chassis keep-out, and the floor with the bike upright."""
    R = lk.wheel_radius
    cl = cfg.get("clearance") or {}
    half = float(cl.get("wing_width_mm", 2 * PANEL_HALF)) / 2     # the config's own panel
    width = float(cl.get("coupler_width_mm", 8.0))
    rows = [{"what": f"servo case ({fit['what']})", "v": fit["clear"], "t": None,
             "kind": "mm", "target": FIT_MM}]
    box = ((cfg.get("clearance") or {}).get("panel_keepout") or [None])[0]
    gap = (np.inf, 0.0)
    keep = (np.inf, 0.0)
    floor = (np.inf, 0.0)
    for fr in frames:
        t = fr["t"]
        if planes.get("couplerR") == planes.get("couplerL"):
            g = _seg_dist(fr["crank_tipR"], fr["jointR"], fr["crank_tipL"], fr["jointL"])
            gap = min(gap, (g, t))
        for tag in ("R", "L"):
            ends = [np.array(fr["foot" + tag]), np.array(fr["top" + tag])]
            if box:
                rect = (-box["half_width"], box["half_width"], box["z_lo"] - R,
                        box["z_hi"] - R if box.get("z_hi") is not None else 1e3)
                for k in np.linspace(0.0, 1.0, 11):
                    q = ends[0] + k * (ends[1] - ends[0])
                    keep = min(keep, (rect_sdf(q, rect) - half, t))
            if t <= T + 1e-6:
                floor = min(floor, (min(e[1] for e in ends) + R - half, t))
    if np.isfinite(gap[0]):
        rows.append({"what": "coupler to coupler, centrelines (same plane)", "v": gap[0],
                     "t": gap[1], "kind": "mm", "target": width})
    if np.isfinite(keep[0]):
        rows.append({"what": "panel on chassis keep-out", "v": keep[0], "t": keep[1],
                     "kind": "mm", "target": 0.0})
    rows.append({"what": "floor, bike standing on its wheels", "v": floor[0], "t": floor[1], "kind": "mm", "target": 0.0})
    for r in rows:      # to 0.1 mm: the synthesis holds these AT their limit
        r["v"] = round(float(r["v"]), 1)
        r["t"] = None if r["t"] is None else round(float(r["t"]), 1)
    return sorted(rows, key=lambda r: r["v"] - r["target"])


def design(path: Path, label: str, sc: dict, params, run_sim: bool,
           cad: tuple | None = None, bparams: dict | None = None) -> dict:
    cfg = yaml.safe_load(path.read_text())
    lk = cad[0]["lk"] if cad else sl.SwingLinkage(copy.deepcopy(cfg))
    table = ss.load_table(*ss.reference_panel(cfg, None), lk.wheel_radius)
    st = ss.stroke(lk, table, sc)
    # The COMMANDED stroke, as the sim's crank, teleop and the ground station
    # use it: the config's crank_travel_deg (V2: 129.3, level on the blade).
    # The synthesis's own geometric command (its "flat", st["T"]) only
    # scores the planar stroke -- for V2 it was 128.4, 0.9 short of level.
    T = float(cfg["stroke"]["crank_travel_deg"])
    # A CAD entry's bike carries ITS blade (V3's is heavier than V2's).
    variant = bool(cad) and bparams is not None and bparams is not params
    mp = sl.mass_props(bparams if variant else None)
    M, mw = mp["total"], mp["wing"]
    # A CAD entry RESTS ON ITS BLADE -- every vertex of the section, toe and
    # both faces -- with the wing's CoM from the sim. The panel line is the
    # blade's centre line, which left out its half-thickness and the toe.
    blade0 = blade_in_sketch(lk, cad[3]) if cad else None
    # A CAD blade whose toes meet past the flat runs on to them: that touch,
    # not the commanded travel, is the mechanism's end.
    meet = blade_meet(cad[0], cad[1], cad[3]) if cad else None
    # Where the bike, lying on its side, comes LEVEL on this blade -- also
    # the crank where the blade first touches the floor with the bike
    # standing on its wheels. A variant blade (V3) is COMMANDED to it,
    # rounded down to 0.1 (as V2's 129.3 is from 129.35), unless its file
    # pins `crank_travel_deg`; the module's own blade keeps the config's.
    level = level_point(lk, blade0, mp, meet) if cad else None
    if cad and cad[3].get("travel") is not None:
        T = float(cad[3]["travel"])
    elif variant and level is not None:
        T = np.floor(level * 10.0) / 10.0
    end = meet if meet is not None and meet > T else T
    frames = []
    for f in np.linspace(0.0, 1.0, 121):
        t = float(f * end)
        fr = {"t": round(t, 2)}
        for side, tag in ((-1, "R"), (1, "L")):
            pz = lk.pose(side, t)
            for k in ("crank_tip", "joint", "pivot", "foot", "top"):
                fr[k + tag] = [round(float(v), 1) for v in pz[k]]
        # Past the flat (a CAD entry running on to its toe stop) there is no
        # static rest: the bike goes over centre on the panel's far end.
        if blade0 is None:
            rest = sl.resting_pose(lk, t) or rest_over_centre(lk, t)
        else:
            rest = (sl.resting_pose_on(lk, t, blade0, mp["wing_com"], mp=mp)
                    or rest_over_centre(lk, t, turned_points(lk, t, blade0)))
        pz = lk.pose(-1, t)
        g = abs(sl.ratio_at(lk, -1, t, pz) or 0.0)
        load = max(float(np.interp(pz["wing_deg"], *table)), 0.0)
        wing_c = (0.5 * (pz["foot"] + pz["top"]) if blade0 is None
                  else turned_points(lk, t, [mp["wing_com"]])[0])
        com = ((M - mw) * mp["rest"] + mw * wing_c) / M
        # Past the flat the quasi-static model has no load: the bike is
        # falling over centre. Pins and motor are left empty (a gap in the
        # charts) rather than given a number nothing computed.
        pins = sl.pin_forces(lk, t) if rest["regime"] != "over" else None
        cr_, hg = ((round(pins[s]["coupler"], 1), round(pins[s]["hinge"], 1)) if pins else (None, None)
                   for s in ("R", "L"))
        motor = (1 + sc["c_run"]) * load * g + sc["f_run"]
        fr.update({"roll": round(rest["to_floor"][0], 3), "floor": round(rest["to_floor"][1], 3),
                   "fc": cr_[0], "fh": cr_[1],
                   "pin": {"crank_tipR": cr_[0], "jointR": cr_[0], "pivotR": cr_[1],
                           "crank_tipL": hg[0], "jointL": hg[0], "pivotL": hg[1],
                           "shaft": round(pins["shaft"], 1) if pins else None},
                   "regime": rest["regime"], "r": round(g, 4),
                   "motor": round(float(motor), 4) if pins and np.isfinite(motor) else None,
                   "com": [round(float(v), 2) for v in com],
                   "wing": round(float(pz["wing_deg"]), 2)})
        frames.append(fr)
    m = cfg["mechanism"]
    planes = {k: plane_of(lk, k) for k in ("crank", "couplerR", "couplerL", "rockerR", "rockerL")}
    sim = sim_counts(params, path) if run_sim and not cad else None
    lift_pin = lift_pin_peak(params, path) if run_sim and not cad else None
    fit = servo_fit(lk, T, [0.0, float(lk.shaft[1])])
    if cad:
        r = servo_rect([0.0, float(lk.shaft[1])], "down")
        fit = {"case": "down", "clear": round(rect_sdf(lk.pivot(1), r) - 3.0, 1),
               "what": "wing rod (6 mm), allowed"}
    for k, fr in enumerate(frames):
        fr["i"] = k
    print(f"{label:<40} planar {st['counts']:.0f}  sim {sim if sim is not None else '--'}  "
          f"travel {T:.1f}")
    sweep = cad_sweep(*cad, frames, float(lk.pivot(1)[1]), T, meet) if cad else None
    if sweep:
        lim = sweep["limit"]
        meet = sweep["meet"] if sweep["meet"] is not None else "never"
        touch = f"{lim['t']:.1f} ({lim['what']})" if lim else "never"
        print(f"  blade ({sweep['src']}): blades meet at crank {meet}, commanded "
              f"stroke {sweep['T']:.1f}, links touch at {touch};\n"
              f"least clearance over the stroke:")
        for r in sweep["clear"]:
            unit = "mm2 OVERLAP" if r["kind"] == "mm2" else "mm"
            flag = "  < target" if r["kind"] == "mm2" or r["v"] < r["target"] else ""
            print(f"    {r['what']:<34} {abs(r['v']) if r['kind'] == 'mm2' else r['v']:8.2f} {unit}"
                  f"  at crank {r['t']:.0f}{flag}")
    return {"name": label, "file": str(path.relative_to(ROOT)) if path.is_relative_to(ROOT)
            else str(path), "T": round(T, 1), "planes": planes, "cad": sweep,
            "level": None if level is None else round(level, 2),
            "T_is_level": bool(variant and cad[3].get("travel") is None and level is not None),
            "counts": st["counts"], "brk": round(st["brk"], 4), "sim": sim,
            "pins": {"coupler": max(f["fc"] for f in frames if f["fc"] is not None),
                     "hinge": max(f["fh"] for f in frames if f["fh"] is not None),
                     "lift_sim": lift_pin},
            "fit": fit,
            "blade": None if not cad or not cad[3].get("summary") else
                     {"file": Path(cad[3]["src"]).name, "rows": cad[3]["summary"]},
            "diag": None if cad else config_diag(lk, cfg, frames, T, planes, fit),
            "shaft": [0.0, round(float(lk.shaft[1]), 2)],
            # as posed (the CAD's are scaled), not as the file has them
            "lengths": {"crank [mm]": round(float(lk.crank), 1),
                        "coupler [mm]": round(float(lk.coupler), 1),
                        "rocker [mm]": round(float(lk.rocker), 1),
                        "between the crank arms [deg]": round(float(lk.between), 1),
                        "servo shaft above the axle [mm]": round(float(lk.shaft[1]), 1),
                        "wing hinges off centre [mm]": round(float(lk.pivot_x), 1)},
            "frames": frames}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("configs", nargs="*", type=Path)
    ap.add_argument("--labels", nargs="*", default=None,
                    help="names for the entries, in order: configs, then blades")
    ap.add_argument("--sim", action="store_true", help="run each in the sim at 9.9 V")
    ap.add_argument("--out", type=Path, default=None,
                    help=f"default {OUT.relative_to(ROOT)}; with --blade or configs, "
                         f"{OUT_CUSTOM.relative_to(ROOT)} (scratch)")
    ap.add_argument("--blade", type=Path, nargs="*", default=None,
                    help="blade front-section yamls: one CAD entry each ('cad' for the "
                         "CAD's own dummy blade)")
    args = ap.parse_args()
    if args.configs or args.blade:
        # --labels name the entries in order: configs first, then blades.
        labels = list(args.labels or [])
        lab = lambda i, default: labels[i] if i < len(labels) else default  # noqa: E731
        entries = [(p.resolve(), lab(i, p.stem), None) for i, p in enumerate(args.configs)]
        n = len(entries)
        entries += [(DIAMOND, lab(n + i, Path(f).stem), "cad" if str(f) == "cad" else Path(f))
                    for i, f in enumerate(args.blade or [])]
    else:
        entries = list(DEFAULT)
    params = load_params()
    sc = ss.servo_consts(params)

    def blade_spec(b):
        if b == "cad":
            return None
        spec = yaml.safe_load(Path(b).read_text())
        p = Path(b).resolve()
        spec["_file"] = str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p)
        return spec
    specs = [blade_spec(b) if b is not None else None for _, _, b in entries]
    bps = [blade_params(params, sp) for sp in specs]
    designs = [design(Path(p), label, sc, params, args.sim,
                      cad_parts(sp) if b is not None else None, bp)
               for (p, label, b), sp, bp in zip(entries, specs, bps)]
    pairs = [(p, label) for p, label, _ in entries]
    ref = yaml.safe_load(Path(pairs[0][0]).read_text())
    box = ((ref.get("clearance") or {}).get("panel_keepout") or [{}])[0]
    R = float(ref["bike"]["wheel_radius"])
    geo = {"R": R, "case": {"w": CASE_W, "l": CASE_L, "near": SHAFT_FROM_END},
           "rod_r": ROD_R, "keepout": {"half": float(box.get("half_width", 35.0)),
                               "z_lo": float(box.get("z_lo", 50.0)) - R},
           "stall99": sc["ts"] * ss.SUPPLY,
           # the most current the servo can turn into torque at 9.9 V: its
           # stall there, or the firmware limit if that is lower
           "max_ma": round(min(ss.counts(sc, sc["ts"] * ss.SUPPLY), ss.CURRENT_LIMIT_A * 1000)),
           "limit_nm": sc["k"] * (ss.CURRENT_LIMIT_A - sc["i0"])}
    # The CAD entries' sim: the bike's own righting module, the default build
    # (bike_params righting.module -- these links scaled as cad_righting
    # does, toes and all), carrying each entry's blade (`blade_params`: V2's
    # is the module's own, V3's swaps the outline and its mass). The CAD's
    # dummy blade is not simulated.
    if args.sim:
        k = float(params["righting"]["module"]["linkage"]["_source"]["scale"])
        for d, sp, bp in zip(designs, specs, bps):
            if not d["cad"] or sp is None:
                continue
            if bp is not params:      # the variant's stroke: its file's, or its level point
                bp["righting"]["module"]["linkage"]["stroke"]["crank_travel_deg"] = d["T"]
            sim, lift = sim_counts(bp, None), lift_pin_peak(bp, None)
            on_frames(sim_trace(bp, None, d["frames"][-1]["t"]), d["frames"])
            print(f"{d['name']}: the righting module (CAD linkage x{k:g}, {sp['_file']}) "
                  f"in the sim: {sim} mA, coupler pin at the start {lift} N")
            d["sim"], d["pins"]["lift_sim"] = sim, lift
    if args.sim:          # the config designs' traces, on their own configs
        for (path, _), d in zip(pairs, designs):
            if not d["cad"]:
                on_frames(sim_trace(params, Path(path), d["frames"][-1]["t"]), d["frames"])
    # Under the Browser pane's 512 KB: frames go as rows under one key list
    # (the page rebuilds the objects), and the CAD entries share one copy of
    # the righting parts, which do not change with the blade.
    for d in designs:
        fr = d["frames"]
        pk = list(fr[0]["pin"])
        keys = [k for k in fr[0] if k != "pin"]
        d["frames"] = {"keys": keys, "pin": pk,
                       "rows": [[f[k] for k in keys] + [[f["pin"][k] for k in pk]] for f in fr]}
        if d["cad"]:
            geo["cad_parts"] = d["cad"].pop("parts")
    data = json.dumps({"designs": designs, "geo": geo, "notes": NOTES},
                      separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
    tag = '<script id="data" type="application/json">'
    page = TEMPLATE.read_text(encoding="utf-8")
    if page.count(tag + "/*DATA*/</script>") != 1:
        raise SystemExit(f"{TEMPLATE.name}: expected exactly one data tag")
    if args.out is None:
        args.out = OUT_CUSTOM if (args.configs or args.blade) else OUT
    args.out.write_text(page.replace(tag + "/*DATA*/", tag + data), encoding="utf-8")
    print(f"wrote {args.out.relative_to(ROOT) if args.out.is_relative_to(ROOT) else args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
