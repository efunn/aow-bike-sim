"""An interactive page stepping swing-linkage designs through their righting stroke.

Reads the SAME config files the sim does (`build_model(..., swing_linkage_cfg=)`,
`righting_current_sweep.py --config`), poses them with `swing_linkage.SwingLinkage`
(held to the builder's own solver by tests/test_hw_telemetry.py), rests the bike
on the floor with `swing_linkage.resting_pose`, and scores them with
`swing_synthesis.stroke`. Writes one self-contained HTML file from
`analysis/templates/swing_stepthrough.html`: two panes with a shared stroke
slider, each showing the mechanism in the bike frame and the bike on the floor,
plus ratio and motor-torque curves.

`--sim` also runs each config in the sim at 9.9 V -- the sweep's own `fallen`
and `trial`, goal stepped as teleop does -- bisecting to 10 counts on one side
(the pair is mirror-symmetric). ~30 s a config. Without it the page says "not
run" rather than carrying a number from somewhere else.

    python analysis/swing_stepthrough.py --sim            # the four tracked designs
    python analysis/swing_stepthrough.py config/a.yaml config/b.yaml --labels A B
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

ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = Path(__file__).resolve().parent / "templates" / "swing_stepthrough.html"
OUT = Path(__file__).resolve().parent / "plots" / "swing_stepthrough.html"
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


def lift_pin_peak(params, path: Path, counts: int = 700, supply: float = 9.9) -> float:
    """Peak coupler-pin force [N] in the sim while the stepped stroke lifts the
    bike to upright (not the fall after it). On every design it lands in the
    first 1-3 ms: the servo stepping from holding to full current against a
    bike that has not moved yet. Not a solver artifact -- with nothing
    commanded the same start reads only the static load."""
    import mujoco
    import righting_current_sweep as rcs
    from aow_sim.control.righting import roll_pitch
    from aow_sim.righting_servo import CURRENT_LIMIT
    dep = np.deg2rad(float(yaml.safe_load(path.read_text())["stroke"]["crank_travel_deg"]))
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


DEFAULT = [
    (SWING_LINKAGE_CFG, "Built today (_smaller)"),
    (ROOT / "config" / "swing_linkage_stagger.yaml", "Staggered couplers, free end"),
    (ROOT / "config" / "swing_linkage_stagger_lock.yaml", "Staggered couplers, self-locking end"),
    (ROOT / "config" / "swing_linkage_shared_rod.yaml", "Diamond: one crank pin, one wing rod"),
    (ROOT / "config" / "swing_explore" / "diamond_bothout.yaml",
     "Wild: diamond, both wings end out"),
]
NOTES = [
    "<b>What changes between designs.</b> Every design has the same wings, the "
    "same wing hinge position (or one shared rod) and the same servo. Only the "
    "links between the servo and the wings change. The slider runs the "
    "self-righting stroke from lying on its side (0%) to upright (100%). Red "
    "is the wing that pushes the bike up, blue the one that rises and tucks "
    "in, purple the servo's crank.",
    "<b>Counts.</b> The servo's Goal Current setting, 1 count = 1 mA. It is the "
    "most current the servo may use, so a design that rights the bike at fewer "
    "counts has more headroom. At a low battery (9.9 V) the servo can give "
    "about 810 counts' worth. <i>Sim, 9.9 V</i> is the lowest setting that "
    "rights the bike in the MuJoCo simulation with a low battery. <i>Planar, "
    "moving</i> is the fast 2D estimate the design search uses; it reads "
    "25-45 counts high but ranks designs the same way.",
    "<b>Pin loads.</b> <i>Slow stroke</i> is the force on the coupler pins "
    "(also the crank pin) and on the wing hinge with the bike lifted slowly. "
    "<i>Sim, at the start</i> is the coupler-pin peak in the simulation for a "
    "stepped stroke at 700 counts, 9.9 V, up to upright. It always happens in "
    "the first few milliseconds: the servo switches from holding to full "
    "current while the bike has not started moving, and the low ratio at the "
    "start that buys margin also multiplies that force. Ramping the Goal "
    "Current up over 50-150 ms cuts it (126 to 65-79 N on the both-wings-out "
    "design) for a few hundredths of a second. Neither number includes a fall.",
    "<b>Pin colours.</b> Each pin is filled by its force in the slow stroke, "
    "on one scale shared by both panes (the bar at the top). The servo shaft "
    "is the pin at the crank's centre.",
    "<b>Servo case.</b> The dashed outline around the crank shaft is the "
    "XC330 case (20 x 34 mm, its shaft 9.5 mm from one end). It sits behind "
    "the horn, in the same front-to-back span as the wing panels and the "
    "hinge rod, so those are what it can hit (the links are in front of the "
    "horn). It points down if that clears both by 1 mm over the whole stroke, "
    "else up; drawn orange when neither fits.",
    "<b>Layers.</b> The links sit in layers front to back. Links in different "
    "layers can cross in the front view without touching; a dashed left "
    "coupler means the couplers are in different layers, and the panel lists "
    "which part is in which. An orange ring appears only where two links in "
    "the SAME layer would hit.",
    "<b>Other marks.</b> The dashed yellow box is where the wing panels must "
    "not go (servos, battery); the links may pass through it. The grey bar is "
    "the wheel seen end-on.",
    "<b>Limitations.</b> The poses are slow-motion (quasi-static) poses from "
    "the 2D model; the sim runs the real dynamics for its numbers. Wheels are "
    "passive, so catching the bike once it is up is not modelled. The hinge "
    "rod is taken as 1/8 in; the panel as the sim's 6 mm-thick flat box.",
]


def plane_of(lk, member: str):
    return lk.planes.get(member, lk.planes.get(member[:-1]))


def sim_counts(params, path: Path, supply: float = 9.9) -> int:
    """Lowest Goal Current (10-count steps) that rights the bike, stepped."""
    import righting_current_sweep as rcs
    from aow_sim.righting_servo import CURRENT_LIMIT
    dep = np.deg2rad(float(yaml.safe_load(path.read_text())["stroke"]["crank_travel_deg"]))
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


def design(path: Path, label: str, sc: dict, params, run_sim: bool) -> dict:
    cfg = yaml.safe_load(path.read_text())
    lk = sl.SwingLinkage(copy.deepcopy(cfg))
    table = ss.load_table(*ss.reference_panel(cfg, None), lk.wheel_radius)
    st = ss.stroke(lk, table, sc)
    T = st["T"]
    mp = sl.mass_props()
    M, mw = mp["total"], mp["wing"]
    frames = []
    for f in np.linspace(0.0, 1.0, 121):
        t = float(f * T)
        fr = {"t": round(t, 2)}
        for side, tag in ((-1, "R"), (1, "L")):
            pz = lk.pose(side, t)
            for k in ("crank_tip", "joint", "pivot", "foot", "top"):
                fr[k + tag] = [round(float(v), 2) for v in pz[k]]
        rest = sl.resting_pose(lk, t)
        pz = lk.pose(-1, t)
        g = abs(sl.ratio_at(lk, -1, t, pz) or 0.0)
        load = max(float(np.interp(pz["wing_deg"], *table)), 0.0)
        com = ((M - mw) * mp["rest"] + mw * 0.5 * (pz["foot"] + pz["top"])) / M
        pins = sl.pin_forces(lk, t)
        fr.update({"roll": round(rest["to_floor"][0], 3), "floor": round(rest["to_floor"][1], 3),
                   "fc": round(pins["R"]["coupler"], 1), "fh": round(pins["R"]["hinge"], 1),
                   "pin": {"crank_tipR": round(pins["R"]["coupler"], 1),
                           "jointR": round(pins["R"]["coupler"], 1),
                           "pivotR": round(pins["R"]["hinge"], 1),
                           "crank_tipL": round(pins["L"]["coupler"], 1),
                           "jointL": round(pins["L"]["coupler"], 1),
                           "pivotL": round(pins["L"]["hinge"], 1),
                           "shaft": round(pins["shaft"], 1)},
                   "regime": rest["regime"], "r": round(g, 4),
                   "motor": round((1 + sc["c_run"]) * load * g + sc["f_run"], 4),
                   "com": [round(float(v), 2) for v in com],
                   "wing": round(float(pz["wing_deg"]), 2)})
        frames.append(fr)
    m = cfg["mechanism"]
    planes = {k: plane_of(lk, k) for k in ("crank", "couplerR", "couplerL", "rockerR", "rockerL")}
    sim = sim_counts(params, path) if run_sim else None
    lift_pin = lift_pin_peak(params, path) if run_sim else None
    fit = servo_fit(lk, T, [0.0, float(lk.shaft[1])])
    print(f"{label:<40} planar {st['counts']:.0f}  sim {sim if sim is not None else '--'}  "
          f"travel {T:.0f}")
    return {"name": label, "file": str(path.relative_to(ROOT)) if path.is_relative_to(ROOT)
            else str(path), "T": round(T, 1), "planes": planes,
            "counts": st["counts"], "brk": round(st["brk"], 4), "sim": sim,
            "pins": {"coupler": max(f["fc"] for f in frames),
                     "hinge": max(f["fh"] for f in frames), "lift_sim": lift_pin},
            "fit": fit,
            "shaft": [0.0, round(float(lk.shaft[1]), 2)],
            "lengths": {k: round(float(m[k]), 1) for k in (
                "crank_length", "coupler_length", "rocker_length", "angle_between_cranks",
                "servo_offset", "wing_pivot_x")},
            "frames": frames}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("configs", nargs="*", type=Path)
    ap.add_argument("--labels", nargs="*", default=None)
    ap.add_argument("--sim", action="store_true", help="run each in the sim at 9.9 V")
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()
    if args.configs:
        pairs = [(p.resolve(), (args.labels or [])[i] if i < len(args.labels or []) else p.stem)
                 for i, p in enumerate(args.configs)]
    else:
        pairs = DEFAULT
    params = load_params()
    sc = ss.servo_consts(params)
    designs = [design(Path(p), label, sc, params, args.sim) for p, label in pairs]
    ref = yaml.safe_load(Path(pairs[0][0]).read_text())
    box = ((ref.get("clearance") or {}).get("panel_keepout") or [{}])[0]
    R = float(ref["bike"]["wheel_radius"])
    geo = {"R": R, "case": {"w": CASE_W, "l": CASE_L, "near": SHAFT_FROM_END},
           "rod_r": ROD_R, "keepout": {"half": float(box.get("half_width", 35.0)),
                               "z_lo": float(box.get("z_lo", 50.0)) - R},
           "stall99": sc["ts"] * ss.SUPPLY,
           "limit_nm": sc["k"] * (ss.CURRENT_LIMIT_A - sc["i0"])}
    data = json.dumps({"designs": designs, "geo": geo, "notes": NOTES},
                      separators=(",", ":")).replace("</", "<\\/")
    tag = '<script id="data" type="application/json">'
    page = TEMPLATE.read_text(encoding="utf-8")
    if page.count(tag + "/*DATA*/</script>") != 1:
        raise SystemExit(f"{TEMPLATE.name}: expected exactly one data tag")
    args.out.write_text(page.replace(tag + "/*DATA*/", tag + data), encoding="utf-8")
    print(f"wrote {args.out.relative_to(ROOT) if args.out.is_relative_to(ROOT) else args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
