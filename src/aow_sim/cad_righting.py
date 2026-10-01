"""The righting module, printed: the diamond swing linkage as a supported
crankshaft in ASA, metal rod stock only for the wing rod, the crankpin and the
two rocker pins. ONE custom feature, `AOW righting`, in its own Feature
Studio. Spec: docs/plans/righting-design.md. Numbers: config/righting_cad.yaml,
the steering module's hub and lugs (config/steering_cad.yaml), the X330
envelope and mount interface cad_servo_mount reads.

MODULE FRAME: origin on the wing rod's axis at the mid-plane between the
couplers, +Y forward, +X right, +Z up. The XC330 at the rear, horn forward.

    part              print  what it is
    half crank R / L  Y- / Y+ web, shoulder, Phi 14 journal; the rear one's
                              lugs drive it from the horn hub (the front
                              one's are spare: one part)
    coupler R / L     Y+      both on the crankpin
    rocker R / L      Y+ / Y- hub on the rod, arm to the coupler, ear to the
                              panel's boss
    knuckle R / L     Y+ / Y- the wing's second support, beyond the far bearing
    wing R / L        inner   the panel, with a tab at each boss
    front bulkhead    Y+      front journal bearing, the rod
    lower case        Y-      rear journal bearing, the rod, hub cavity, the
                              servo's horn half (steering's lower case)
    upper case        Y+      the servo's back half, a pad up to the bridge
    horn hub          Y+      steering's, on the horn: M2 x 6, cross socket
    bridge            Z+      ties the three; nut slots + ridges on top for
                              the chassis
    chassis plate     Z+      PLACEHOLDER for the chassis

THE LOAD PATH: pin forces go crankpin -> webs -> journals -> the two bearings;
the horn hub only takes torque through the lugs' clearance. Wing forces go
panel -> boss/tab -> rocker and knuckle -> rod -> both bearings. Torque about
the rod still reaches the servo through the linkage -- nothing here stops it.

Every metal hole is drawn at `rod.dia`, line-to-line: ream to press (webs,
rocker arms, bulkheads) or to run (couplers, rocker and knuckle hubs).

    python -m aow_sim.cad_righting                   # write docs/cad/righting.fs
    python -m aow_sim.cad_righting --check           # ONE call, `check` tab
    python -m aow_sim.cad_righting --push righting_features
    python -m aow_sim.cad_righting --shot            # -> docs/cad/righting.png
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

from . import cad_ahrs_fixture as af
from . import cad_servo_mount as sm
from .params import _normalize, load_params

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "analysis"))
import swing_synthesis as ss                    # noqa: E402

PARAMS = "config/righting_cad.yaml"
STEER_PARAMS = "config/steering_cad.yaml"
OUT_FS = "docs/cad/righting.fs"
OUT_PNG = "docs/cad/righting{}.png"
SPLIT_MARK = af.SPLIT_MARK
TAB = "righting_features"
PART_STUDIO = "righting"

CHECK_POSES = (-1.0, -0.75, -0.56, -0.5, -0.25, 0.25, 0.5, 0.56, 0.75, 1.0)
DIALOG_POSES = (("REST", "Rest", 0.0),
                ("R25", "Right wing 25%", 0.25), ("R50", "Right wing 50%", 0.5),
                ("R75", "Right wing 75%", 0.75), ("R100", "Right wing down (100%)", 1.0),
                ("L25", "Left wing 25%", -0.25), ("L50", "Left wing 50%", -0.5),
                ("L75", "Left wing 75%", -0.75), ("L100", "Left wing down (100%)", -1.0))
TURN_C = (0.93, 0.56, 0.20)       # moving printed parts, as steering's
FIXED_C = (0.62, 0.64, 0.68)
STEEL_C = (0.30, 0.30, 0.32)
MOCK_C = (0.85, 0.85, 0.85)


# ------------------------------------------------------------------- inputs
def load(path: str = PARAMS, steer_path: str = STEER_PARAMS,
         cad_path: str = sm.CAD_PARAMS, mount_path: str = sm.MOUNT_PARAMS) -> dict:
    s = _normalize(yaml.safe_load(Path(path).read_text()))
    st = _normalize(yaml.safe_load(Path(steer_path).read_text()))
    mm = lambda v: v * 1000.0    # noqa: E731
    hub = {k: mm(v) for k, v in st["hub"].items()}
    coupling = {k: mm(v) for k, v in st["coupling"].items()}
    ref = yaml.safe_load((ROOT / s["linkage"]["config"]).read_text())
    k = float(s["linkage"]["scale"])
    m = ref["mechanism"]
    # THE DIAMOND ONLY: one crank pin (the couplers side by side on it) and one
    # wing rod on the centreline (both rockers on it). Every stack, bearing and
    # symmetry here depends on both; a staggered or two-hinge config is a
    # different module, not a different number.
    if abs(m["angle_between_cranks"]) > 1e-3 or abs(m["wing_pivot_x"]) > 1e-6:
        raise ValueError(
            f"{s['linkage']['config']} is not a diamond (angle_between_cranks "
            f"{m['angle_between_cranks']:g}, wing_pivot_x {m['wing_pivot_x']:g}): "
            "cad_righting draws one crank pin and one wing rod only")
    pz = m["wing_pivot_z"]
    x = [m["crank_length"] * k, m["coupler_length"] * k, m["rocker_length"] * k,
         m["angle_between_cranks"], pz + k * (m["servo_offset"] - pz)]
    lk = ss.candidate(ref, x, ss.reference_panel(ref, None), 0.0)
    params = load_params(cad_path)
    mounts = sm.load_mounts(mount_path)
    sv = params["servos"][af.SERVO_KEY]
    d, w, h = sv["box_size"]
    x330 = {"caseDepth": mm(d), "caseWidth": mm(w), "caseHeight": mm(h),
            "shaftFromEnd": mm(sv["shaft_from_end"]),
            "hornThickness": mm(sv["horn_thickness"]), "hornDiameter": mm(sv["horn_diameter"])}
    return {"s": s, "hub": hub, "coupling": coupling, "lk": lk, "scale": k,
            "T": float(ref["stroke"]["crank_travel_deg"]), "pivot_z": pz,
            "wheel_r": ref["bike"]["wheel_radius"], "x330": x330,
            "table": sm.servo_table(params, mounts), "screw": sm.screw_table(mounts)}


# ------------------------------------------------------------ primitives
def cyl(p, y0, y1, r):
    return {"k": "cyl", "x": float(p[0]), "z": float(p[1]), "y0": float(y0), "y1": float(y1),
            "r": float(r)}


def prism(pts, y0, y1):
    """An (X, Z) outline extruded along Y."""
    return {"k": "prism", "pts": [[float(q[0]), float(q[1])] for q in pts],
            "y0": float(y0), "y1": float(y1)}


def prism_x(pts, x0, x1):
    """A (Y, Z) outline extruded along X."""
    return {"k": "prismX", "pts": [[float(q[0]), float(q[1])] for q in pts],
            "x0": float(x0), "x1": float(x1)}


def box(lo, hi):
    return {"k": "box", "lo": [float(min(a, b)) for a, b in zip(lo, hi)],
            "hi": [float(max(a, b)) for a, b in zip(lo, hi)]}


def taper_quad(a, b, ra, rb) -> list:
    a, b = np.asarray(a, float), np.asarray(b, float)
    u = (b - a) / np.linalg.norm(b - a)
    perp = np.array([-u[1], u[0]])
    c = (ra - rb) / np.linalg.norm(b - a)
    s = math.sqrt(1 - c * c)
    n1, n2 = c * u + s * perp, c * u - s * perp
    return [a + ra * n1, b + rb * n1, b + rb * n2, a + ra * n2]


def slot(a, b, ra, rb, y0, y1) -> dict:
    """A tapered link as ONE profile (two tangent arcs, two lines): a quad
    unioned with two cylinders leaves faces exactly tangent, and Onshape's
    boolean refuses those (BOOLEAN_INVALID, 2026-09-29)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    q = taper_quad(a, b, ra, rb)
    u = (b - a) / np.linalg.norm(b - a)
    return {"k": "slot", "pts": [[float(v[0]), float(v[1])] for v in q],
            "mids": [[float(v[0]), float(v[1])] for v in (b + rb * u, a - ra * u)],
            "a": [float(a[0]), float(a[1])], "b": [float(b[0]), float(b[1])],
            "ra": float(ra), "rb": float(rb), "y0": float(y0), "y1": float(y1)}


def radial_bar(c, ang_deg, r0, r1, hw, y0, y1) -> dict:
    a = math.radians(ang_deg)
    d = np.array([math.cos(a), math.sin(a)])
    n = np.array([-math.sin(a), math.cos(a)])
    c = np.asarray(c, float)
    return prism([c + r0 * d - hw * n, c + r1 * d - hw * n, c + r1 * d + hw * n,
                  c + r0 * d + hw * n], y0, y1)


def turned_prim(pr: dict) -> dict:
    """The same primitive turned 180 deg about Z: (x, y, z) -> (-x, -y, z)."""
    q = copy.deepcopy(pr)
    if q["k"] == "cyl":
        q["x"], q["y0"], q["y1"] = -q["x"], -q["y1"], -q["y0"]
    elif q["k"] == "prism":
        q["pts"] = [[-x, z] for x, z in q["pts"]][::-1]
        q["y0"], q["y1"] = -q["y1"], -q["y0"]
    elif q["k"] == "slot":
        q["pts"] = [[-x, z] for x, z in q["pts"]]
        q["mids"] = [[-x, z] for x, z in q["mids"]]
        q["a"], q["b"] = [-q["a"][0], q["a"][1]], [-q["b"][0], q["b"][1]]
        q["y0"], q["y1"] = -q["y1"], -q["y0"]
    elif q["k"] == "box":
        lo, hi = q["lo"], q["hi"]
        q["lo"], q["hi"] = [-hi[0], -hi[1], lo[2]], [-lo[0], -lo[1], hi[2]]
    else:
        raise ValueError(f"cannot turn a {q['k']}")
    return q


def turned_vec(v) -> list:
    return [-float(v[0]), -float(v[1]), float(v[2])]


def bbox(prims: list) -> tuple[list, list]:
    lo, hi = [np.inf] * 3, [-np.inf] * 3
    for q in prims:
        if q["k"] == "cyl":
            a = [q["x"] - q["r"], q["y0"], q["z"] - q["r"]]
            b = [q["x"] + q["r"], q["y1"], q["z"] + q["r"]]
        elif q["k"] == "slot":
            a = [min(q["a"][0] - q["ra"], q["b"][0] - q["rb"]), q["y0"],
                 min(q["a"][1] - q["ra"], q["b"][1] - q["rb"])]
            b = [max(q["a"][0] + q["ra"], q["b"][0] + q["rb"]), q["y1"],
                 max(q["a"][1] + q["ra"], q["b"][1] + q["rb"])]
        elif q["k"] == "prism":
            p = np.array(q["pts"])
            a = [p[:, 0].min(), q["y0"], p[:, 1].min()]
            b = [p[:, 0].max(), q["y1"], p[:, 1].max()]
        elif q["k"] == "prismX":
            p = np.array(q["pts"])
            a = [q["x0"], p[:, 0].min(), p[:, 1].min()]
            b = [q["x1"], p[:, 0].max(), p[:, 1].max()]
        else:
            a, b = q["lo"], q["hi"]
        lo = [min(u, v) for u, v in zip(lo, a)]
        hi = [max(u, v) for u, v in zip(hi, b)]
    return lo, hi


# ------------------------------------------------------------------- layout
def layout(data: dict) -> dict:
    """Every part as primitives in the module frame, the joints, the anchors,
    and the poses. Raises if a wall goes too thin."""
    s, lk, xc, t = data["s"], data["lk"], data["x330"], data["table"]["XC330"]
    t = {k: v * 1000 for k, v in t.items() if k != "hornHoleCount"}
    hb, cp, sc = data["hub"], data["coupling"], data["screw"]
    st, rd, ck, wg, cs, fr, ch, jn = (s[k] for k in ("stack", "rod", "crank", "wing", "cases",
                                                     "frame", "chassis", "joint"))
    wall = s["print"]["min_wall"]
    jp = jn["joint_plate"]

    def need(ok, what):
        if not ok:
            raise ValueError(what)

    pz = data["pivot_z"]

    def xz(p):
        return np.array([-p[0], p[1] - pz])

    O = np.zeros(2)
    C = xz(lk.shaft)
    pr = lk.pose(-1, 0.0)
    pin, J = xz(pr["crank_tip"]), xz(pr["joint"])
    foot, top = xz(pr["foot"]), xz(pr["top"])
    w = (top - foot) / np.linalg.norm(top - foot)
    n_in = np.array([-w[1], w[0]])
    if np.dot(n_in, O - foot) < 0:
        n_in = -n_in
    half = wg["panel_thickness"] / 2

    def on_panel(u, n):
        return foot + u * w + n * n_in

    # THRUST BOSSES: a ring stands `boss` (two layers) off one of the two
    # faces at every running interface, so parts bear on a small ring at the
    # hole, not face to face. A boss can only be on a face that is UP as its
    # part prints, which decides whose it is; see the plan doc's table. Those
    # interfaces open to boss + clearance. The couplers' shared face has no
    # candidate (both are print beds): plain gap, and only ~11 deg of
    # relative swing there.
    g = st["gap"]
    gb = st["boss"] + st["clearance"]
    # the couplers' shared face: a loose washer on the crankpin between them
    # (neither face can carry a printed boss). `washer.thickness` 0 = none.
    ws = s["washer"]
    wsh = ws["thickness"]
    Y = {"cpl_in": (wsh / 2 + ws["clearance"]) if wsh > 0 else g / 2}
    Y["cpl_out"] = Y["cpl_in"] + st["coupler"]
    Y["web_in"] = Y["cpl_out"] + gb
    Y["web_out"] = Y["web_in"] + st["web"]
    Y["hz_out"] = Y["web_out"] + st["hub_zone"]
    Y["bh_in"] = Y["hz_out"] + gb
    Y["bh_out"] = Y["bh_in"] + st["bulkhead"]
    Y["kn_in"] = Y["bh_out"] + gb
    Y["kn_out"] = Y["kn_in"] + st["knuckle"]
    # knuckle R BEHIND knuckle L (user, 2026-09-30): both wings' second
    # supports at the rear, so the module ends at the front bulkhead and the
    # rod stops there. knuckle R bears on knuckle L's back face (they turn
    # opposite ways), over a ring on its own front face.
    rear_kr = wg["knuckle_r"] == "rear"
    Y["kr_in"] = Y["kn_out"] + gb
    Y["kr_out"] = Y["kr_in"] + st["knuckle"]
    Y["rod"] = Y["kn_out"] + 1.0
    Y["rod_back"] = (Y["kr_out"] if rear_kr else Y["kn_out"]) + 1.0
    Y["rod_front"] = Y["bh_out"] if rear_kr else Y["rod"]
    Y["jn_end"] = Y["bh_out"] + ck["journal_proud"]
    Y["hub_front"] = Y["jn_end"] + cp["hub_gap"]           # the hub's socket face
    Y["horn_face"] = Y["hub_front"] + hb["thickness"]
    Y["tab_out"] = Y["web_out"] - wg["tab"]                # rocker-side tab, into the web plane

    rr = rd["dia"] / 2
    hub_r = rr + rd["boss_wall"]
    bz, ring_r = st["boss"], rr + st["boss_ring"]
    ws_id = rd["dia"] if ws["id"] is None else ws["id"]     # null: follows the rod
    if wsh > 0:
        need(ws_id >= rd["dia"], f"the washer's bore ({ws_id:g}) is under the rod's {rd['dia']:g}")
        need(ws["od"] / 2 <= hub_r + 3.0, "the washer is much wider than the coupler eyes")

    # ---- the wing's boss on the panel, and the ear that reaches it
    u0, u1 = wg["boss_from_foot"], wg["boss_from_foot"] + wg["boss_length"]
    n0, n1 = half, half + wg["boss_depth"]
    boss = [on_panel(u0, n0), on_panel(u1, n0), on_panel(u1, n1), on_panel(u0, n1)]
    # the rocker-side tab sits in the web plane beside the rocker arm; its
    # inner corner is as far out as the couplers reach, so it must not run on
    # into their plane
    need(Y["tab_out"] >= Y["web_in"] - 1e-9, "the panel's rocker-side tab runs into the coupler plane")
    # the head is in the boss, so the nut sits 2 mm past the joint plane, in the tab
    need(wg["tab"] - 2.0 - sc["nutSlotThickness"] >= wall, "the tab leaves no wall past the nut")
    screw = on_panel((u0 + u1) / 2, (n0 + n1) / 2)
    need((n1 - n0) / 2 - sc["headDia"] / 2 >= wall, "the boss is too shallow for the 6-32 head")
    ehw = wg["ear_half_width"]
    # the ear ends at the boss's centre, its round end inside the boss (it
    # stood out of it as a bump while the nut slot was in the boss)
    ear_end = screw
    kr = math.radians(wg["knee_deg"])
    knee = wg["knee_r"] * np.array([math.cos(kr), math.sin(kr)])
    panel_q = [on_panel(0, half), on_panel(np.linalg.norm(top - foot), half),
               on_panel(np.linalg.norm(top - foot), -half), on_panel(0, -half)]

    parts = []

    def part(slug, name, group, up, add, cut=(), anchor=None, note="", twin=None,
             custom=False, color=None, mock=False, kind="print"):
        parts.append({"slug": slug, "name": name, "group": group,
                      "up": None if up is None else [float(v) for v in up],
                      "add": list(add), "cut": list(cut),
                      "anchor": None if anchor is None else [float(v) for v in anchor],
                      "note": note, "twin": twin, "mirror": None, "custom": custom, "mock": mock, "kind": kind,
                      "color": list(color or (TURN_C if group != "fixed" else FIXED_C))})

    def xyz(p2, y):
        return [p2[0], y, p2[1]]

    # ---- right side (rear half, Y < 0; the knuckle forward); left = turned
    wy, cy, hz = (-Y["web_out"], -Y["web_in"]), (-Y["cpl_out"], -Y["cpl_in"]), (-Y["hz_out"], -Y["web_out"])
    jr = ck["journal_dia"] / 2
    lug0 = cp["socket_r0"] + ck["lug_root_gap"]
    lug1 = math.sqrt(jr ** 2 - (cp["lug_width"] / 2) ** 2) - 0.1
    lug_tip = Y["hub_front"] + cp["socket_depth"] - cp["lug_tip_gap"]
    need(lug1 - lug0 >= 2.0, "the lugs are under 2 mm long")
    lugs = [radial_bar(C, 45 + 90 * k, lug0, lug1, cp["lug_width"] / 2, -lug_tip, -Y["jn_end"])
            for k in range(4)]
    crank_base = [slot(C, pin, ck["shoulder_dia"] / 2, hub_r, *wy),
                  cyl(C, -Y["hz_out"], -Y["web_out"], ck["shoulder_dia"] / 2),
                  cyl(C, -Y["hz_out"] - bz, -Y["hz_out"], ck["shoulder_dia"] / 2 - st["shoulder_ring_inset"]),
                  cyl(C, -Y["jn_end"], -Y["hz_out"], jr)]
    part("crankR", "half crank R", "crank", [0, -1, 0], crank_base + lugs,
         [cyl(pin, wy[0] - 1, wy[1] + 1, rr)],
         anchor=xyz((C + pin) / 2, (wy[0] + wy[1]) / 2),
         note="Print the web face down. Crankpin hole: ream to press. The lugs drive it from the horn hub.")
    part("couplerR", "coupler R", "cR", [0, -1, 0],
         [slot(pin, J, hub_r, hub_r, *cy),
          cyl(pin, cy[0] - bz, cy[0], ring_r), cyl(J, cy[0] - bz, cy[0], ring_r)],
         [cyl(pin, cy[0] - 1, cy[1] + 1, rr), cyl(J, cy[0] - 1, cy[1] + 1, rr)],
         anchor=xyz((pin + J) / 2, (cy[0] + cy[1]) / 2),
         note=f"{lk.coupler:.2f} mm centres. Both holes: ream to run. Print the bossed face up.")
    # the rocker's ear runs STRAIGHT from the coupler joint to the boss: the
    # knuckle's dog-leg is only there to pass under the servo cases, and
    # nothing inside the bay asks for it
    rocker_hz = [slot(O, J, hub_r, hub_r, *hz), slot(J, ear_end, hub_r, ehw, *hz), prism(boss, *hz)]
    part("rockerR", "rocker R", "wR", [0, 1, 0],
         rocker_hz + [slot(O, J, hub_r, hub_r, *wy)],
         [cyl(O, hz[0] - 1, wy[1] + 1, rr), cyl(J, hz[0] - 1, wy[1] + 1, rr)],
         anchor=xyz(J / 2, (hz[0] + hz[1]) / 2),
         note="Print the hub-zone face down: the arm stands on its own shadow. Rod: ream to run; pin: to press.")
    kn = (Y["kn_in"], Y["kn_out"])
    part("knuckleR", "knuckle R", "wR", [0, -1, 0],
         [slot(O, knee, hub_r, ehw, *kn), slot(knee, ear_end, ehw, ehw, *kn), prism(boss, *kn),
          cyl(O, kn[0] - bz, kn[0], ring_r)],
         [cyl(O, kn[0] - 1, kn[1] + 1, rr)],
         anchor=xyz(knee / 2, (kn[0] + kn[1]) / 2),
         note="The wing's second support, beyond the far bearing: the rocker's ear and boss without "
              "its arm (an arm here would reach 5 mm into the lower case at the rise). Rod: ream "
              "to run. Print the ringed face up.")
    part("wingR", "wing R", "wR", list(np.array([n_in[0], 0, n_in[1]])),
         [prism(panel_q, -wg["panel_back"], Y["kn_out"] + wg["tab"]),
          prism(boss, -Y["web_out"], -Y["tab_out"]),
          prism(boss, Y["kn_out"], Y["kn_out"] + wg["tab"])],
         anchor=xyz(on_panel(50.0, 0.0), 0.0),
         note="Print the outer face down. A 6-32 along Y into each tab from its boss; the nut goes in first.")
    part("rpinR", "rocker pin R", "wR", None, [cyl(J, hz[0], -Y["cpl_in"], rr)],
         anchor=xyz(J, (cy[0] + cy[1]) / 2), kind="steel", color=STEEL_C,
         note="Rod stock: pressed in the rocker, runs in the coupler.")

    for p in list(parts):
        if p["slug"].endswith("R"):
            q = copy.deepcopy(p)
            q["slug"] = p["slug"][:-1] + "L"
            q["name"] = p["name"][:-1] + "L"
            q["group"] = {"wR": "wL", "cR": "cL"}.get(p["group"], p["group"])
            q["add"] = [turned_prim(v) for v in p["add"]]
            q["cut"] = [turned_prim(v) for v in p["cut"]]
            q["anchor"] = turned_vec(p["anchor"])
            q["up"] = None if p["up"] is None else turned_vec(p["up"])
            q["twin"] = p["slug"]
            if p["slug"] == "crankR":           # the front half has no lugs
                q["add"] = [turned_prim(v) for v in crank_base]
                q["twin"] = None
                q["note"] = "Print the web face down. Crankpin hole: ream to press. No lugs: it only supports."
            parts.append(q)

    if rear_kr:
        kr = (-Y["kr_out"], -Y["kr_in"])
        span = wg["panel_back"] + Y["kn_out"] + wg["tab"]      # wing L's length, kept
        p0 = -(Y["kr_out"] + wg["tab"])
        for q in parts:
            if q["slug"] == "knuckleR":
                q.update(up=[0.0, 1.0, 0.0],
                         add=[slot(O, knee, hub_r, ehw, *kr), slot(knee, ear_end, ehw, ehw, *kr),
                              prism(boss, *kr), cyl(O, kr[1], kr[1] + bz, ring_r)],
                         cut=[cyl(O, kr[0] - 1, kr[1] + 1, rr)],
                         anchor=xyz(knee / 2, (kr[0] + kr[1]) / 2),
                         note="The right wing's second support, BEHIND knuckle L (it bears on knuckle "
                              "L's back face over its ring). The mirror of knuckle L, not the same "
                              "part. Rod: ream to run. Print the ringed face up.")
            elif q["slug"] == "wingR":
                q["add"] = [prism(panel_q, p0, p0 + span),
                            prism(boss, -Y["web_out"], -Y["tab_out"]),
                            prism(boss, p0, -Y["kr_out"])]
                q["anchor"] = xyz(on_panel(50.0, 0.0), p0 + span / 2)
            elif q["slug"] in ("knuckleL", "wingL"):
                q["mirror"], q["twin"] = q["twin"], None    # same volume, not the same solid

    # ---- the frame
    central = cs["uc_joint"] == "central"
    Zb0 = C[1] + xc["caseHeight"] - xc["shaftFromEnd"] + t["caseSideClearance"] + t["caseTopWall"] \
        + t["caseNestClearance"] + t["caseBottomWall"]            # the case shells' top
    Zb = Zb0 + (cs["top_extra"] if central else 0.0)              # the bridge's underside
    need(Zb >= C[1] + lk.crank + hub_r + 1.0, "the bridge would foul the crank")

    def bulkhead(y0, y1, extra=()):
        return [cyl(O, y0, y1, fr["rod_boss_r"]), cyl(C, y0, y1, fr["journal_boss_r"]),
                box([-fr["web_half"], y0, 0.0], [fr["web_half"], y1, Zb]),
                box([-fr["top_half"], y0, Zb - fr["top_band"]], [fr["top_half"], y1, Zb])] + list(extra)

    bore_r = (ck["journal_dia"] + ck["bore_clearance"]) / 2
    bh = (Y["bh_in"], Y["bh_out"])
    part("bulkheadF", "front bulkhead", "fixed", [0, -1, 0],
         bulkhead(*bh) + [cyl(O, bh[0] - bz, bh[0], ring_r)],
         [cyl(O, bh[0] - 1, bh[1] + 1, rr), cyl(C, bh[0] - 1, bh[1] + 1, bore_r)],
         anchor=[0.0, (bh[0] + bh[1]) / 2, C[1] / 2],
         note="Front journal bearing (ream to fit), the rod (ream to press). Print the inner face up.")

    # ---- the servo: horn face at yH facing +Y, long end UP (local y = +Z)
    yH = -Y["horn_face"]
    inner = xc["caseWidth"] / 2 + t["caseSideClearance"]
    topOuter = inner + t["caseTopWall"]
    nestBore = topOuter + t["caseNestClearance"]
    botOuter = nestBore + t["caseBottomWall"]
    cavR = max(hb["radius"], jr) + cs["cavity_clearance"]
    sfe, endY = xc["shaftFromEnd"], xc["caseHeight"] - xc["shaftFromEnd"]
    near_l, far_l = sfe + t["caseSideClearance"], endY + t["caseSideClearance"]   # local y: -near .. far
    wallN, wallF = near_l + t["caseTopWall"], far_l + t["caseTopWall"]
    nestN = wallN + t["caseNestClearance"]
    upN, upF = nestN + t["caseBottomWall"], wallF + t["caseNestClearance"] + t["caseBottomWall"]
    zCF, zBack = -xc["hornThickness"], -xc["hornThickness"] - xc["caseDepth"]
    wallTop = zBack + t["caseGripLength"]
    upBot = wallTop + t["caseNestLength"]
    upTopZ = zBack - t["caseFaceClearance"] - t["caseCapThickness"]
    yWrap = endY - t["caseWrapLength"]                       # local y where the shell's far wrap starts

    def W(x, yl, zl):
        """Servo-local (x across, y along the case, z along the horn axis) -> module."""
        return [-x, yH + zl, C[1] + yl]

    def lbox(lo, hi):
        a, b = W(*lo), W(*hi)
        return box(a, b)

    fw = (-Y["bh_out"], -Y["bh_in"])
    part("lowercase", "lower case", "fixed", [0, -1, 0],
         bulkhead(*fw, extra=[box([-topOuter, fw[0], C[1] - wallN], [topOuter, fw[1], C[1] + wallF])])
         + [lbox([-topOuter, -wallN, wallTop], [topOuter, wallF, -Y["bh_out"] - yH])],
         [lbox([-inner, -near_l, zBack + 1], [inner, far_l, zCF]),
          cyl(C, yH + zCF - 0.5, fw[0], cavR),
          cyl(C, fw[0] - 1, fw[1] + 1, bore_r), cyl(O, fw[0] - 1, fw[1] + 1, rr)],
         anchor=[0.0, (fw[0] + fw[1]) / 2, C[1] / 2],
         note="Rear journal bearing (ream to fit), the rod, the hub's cavity, the XC330's horn half.")
    need(C[1] - wallN > 6.5 + 2.0, "the lower case comes down onto the rear knuckle's sweep")
    # the upper case: the shared BOTTOM shell, built in FeatureScript, plus these
    m = cs["cable_window_margin"]
    winNear, winFar = t["caseWindowNear"] - m, yWrap
    winApex = wallTop + (winFar - winNear) / 2 * math.tan(math.radians(cs["bridge_taper_deg"]))
    slotIn = cs["cable_strip_width"] / 2
    # two EARS up the case's sides to the bridge, flush with the cap so the
    # case still prints cap-down (a pad behind the cap made the cap overhang)
    ear0, ear1 = botOuter - 1.0, botOuter + cs["ear_width"]
    ear_z1 = upTopZ + cs["ear_depth"]
    ear_x = (ear0 + ear1) / 2
    uc_add = [lbox([-botOuter, -upN, upTopZ], [botOuter, yWrap + 1.0, upBot])]
    if central:
        # the top grows by top_extra over the case's whole top, flush with
        # the cap (it still prints cap-down); one screw on the centreline
        # behind the lower case, whose top it must not sit over
        lc_back = min(q["lo"][1] for q in [lbox([-topOuter, -wallN, wallTop], [topOuter, wallF, -Y["bh_out"] - yH])])
        uc_add.append(box([-botOuter, yH + upTopZ, Zb0 - 0.5], [botOuter, lc_back + 6.8, Zb]))
        # head jp above the bridge's underside, nut 2 mm under it, 3 thick
        nut_wall = Zb - 2.0 - sc["nutSlotThickness"] - (C[1] + xc["caseHeight"] - xc["shaftFromEnd"]) \
            - t["caseSideClearance"]
        need(nut_wall >= wall - 1e-6, f"the upper case's nut slot leaves {nut_wall:.2f} mm over the servo")
        y_uc = lc_back - 0.5 - sc["nutSlotWidth"] / 2
    else:
        need(cs["ear_y0"] > winFar, "the upper case's ears reach down to the cable window")
        need(ear_z1 < wallTop, "the upper case's ears run forward onto the lower case")
        uc_add += [lbox([ear0, cs["ear_y0"], upTopZ], [ear1, Zb - C[1], ear_z1]),
                   lbox([-ear1, cs["ear_y0"], upTopZ], [-ear0, Zb - C[1], ear_z1])]
    uc_cut = [lbox([-nestBore, -nestN, wallTop], [nestBore, yWrap + 1.1, upBot + 1.0]),
              lbox([-inner, -near_l, zBack], [inner, yWrap + 1.1, wallTop + 1.0]),
              prism_x([[yH + zBack, C[1] + winFar], [yH + wallTop, C[1] + winFar],
                       [yH + winApex, C[1] + (winFar + winNear) / 2], [yH + wallTop, C[1] + winNear],
                       [yH + zBack, C[1] + winNear]], -botOuter - 1.0, botOuter + 1.0),
              lbox([slotIn, winNear, upTopZ - 1.0], [botOuter + 1.0, winFar, zBack + 0.1]),
              lbox([-botOuter - 1.0, winNear, upTopZ - 1.0], [-slotIn, winFar, zBack + 0.1])]
    if central:
        uc_anchor = [-(botOuter - 1.0), yH + upTopZ + 1.0, (Zb0 + Zb) / 2]
        uc_note = "The XC330's back half, one 6-32 up into the bridge. Connectors plug in through the cap."
    else:
        uc_anchor = W(-(ear_x + 3.0), cs["ear_y0"] + 3.0, (upTopZ + ear_z1) / 2)
        uc_note = "The XC330's back half, two ears up to the bridge. Connectors plug in through the cap."
    part("uppercase", "upper case", "fixed", [0, 1, 0], [], anchor=uc_anchor, custom=True, note=uc_note)
    uc_shell_pt = W(-(botOuter - 0.5), (wallF + yWrap) / 2, upTopZ + 0.5)

    # ---- the horn hub (steering's) and the XC330's near hole row
    hub_front, hub_back = -Y["hub_front"], yH
    sockHalf = cp["lug_width"] / 2 + cp["lug_clearance"]
    hubCbR = (hb["screw_head_dia"] + hb["screw_head_clearance"]) / 2
    cb_depth = hb["thickness"] - hb["screw_flange"]
    bc = t["hornBoltCircle"] / 2
    hub_cut = [radial_bar(C, 45 + 90 * k, cp["socket_r0"], hb["radius"] + 1.0, sockHalf,
                          hub_front - cp["socket_depth"], hub_front + 1.0) for k in range(4)]
    for k in range(4):
        p = C + bc * np.array([math.cos(k * math.pi / 2), math.sin(k * math.pi / 2)])
        hub_cut += [cyl(p, hub_back - 1.0, hub_front + 1.0, hb["screw_hole_dia"] / 2),
                    cyl(p, hub_front - cb_depth, hub_front + 1.0, hubCbR)]
    a = math.radians(22.5)
    part("hub", "horn hub", "crank", [0, 1, 0], [cyl(C, hub_back, hub_front, hb["radius"])], hub_cut,
         anchor=[C[0] + 5 * math.cos(a), hub_back + 0.5, C[1] + 5 * math.sin(a)],
         note="aow-bike-steering's horn hub: M2 x 6 into the horn, the cross socket for the lugs.")

    # ---- metal
    if wsh > 0:
        part("washer", "centre washer", "crank", [0, 1, 0],
             [cyl(pin, -wsh / 2, wsh / 2, ws["od"] / 2)], [cyl(pin, -wsh / 2 - 1, wsh / 2 + 1, ws_id / 2)],
             anchor=[pin[0] + (ws_id + ws["od"]) / 4, 0.0, pin[1]], color=FIXED_C, kind=ws["kind"],
             note="Loose on the crankpin between the couplers: print two layers, or cut from Delrin/PTFE shim.")
    part("crankpin", "crankpin", "crank", None, [cyl(pin, -Y["web_out"], Y["web_out"], rr)],
         anchor=[pin[0], wsh / 2 + ws["clearance"] + 1.0 if wsh > 0 else 0.0, pin[1]],
         kind="steel", color=STEEL_C,
         note="Rod stock: pressed in both webs, the couplers run on it.")
    part("rod", "wing rod", "fixed", None, [cyl(O, -Y["rod_back"], Y["rod_front"], rr)],
         anchor=[0.0, 0.0, 0.0], kind="steel", color=STEEL_C,
         note="Rod stock: pressed in both bearings, the hubs run on it.")

    # ---- bridge and the chassis placeholder
    bt = fr["bridge"]
    if central:
        # the bridge ends just past the screw's head and the groove round it
        y_back = y_uc - max(sc["headDia"] / 2, jn["ridge_flat"] / 2 + jn["ridge_height"]) - 1.5
    else:
        y_back = yH + upTopZ
        need(fr["bridge_half"] >= ear1 + 0.5, "the bridge is narrower than the upper case's ears")
    part("bridge", "bridge", "fixed", [0, 0, 1],
         [box([-fr["bridge_half"], y_back, Zb], [fr["bridge_half"], Y["bh_out"], Zb + bt])],
         anchor=[fr["bridge_half"] - 3.0, 12.0, Zb + 2.0],
         note="Ties the front bulkhead, lower and upper cases. Top face: the chassis joints.")
    ph = ch["plate_half"]
    # ends 3 mm past the rear chassis joint's groove (ridge_half + 1 along Y)
    y_plate = max(y_back, min(ch["joints_y"]) - ch["ridge_half"] - 1.0 - 3.0)
    part("chassis", "chassis plate (placeholder)", "fixed", [0, 0, 1],
         [box([-ph, y_plate, Zb + bt], [ph, Y["bh_out"], Zb + bt + jp])],
         anchor=[ph - 2.0, 12.0, Zb + bt + 1.0], mock=True, color=MOCK_C,
         note="PLACEHOLDER for the chassis: where the bridge's two 6-32 and ridges land.")
    fz = -(data["wheel_r"] + pz)
    part("floor", "floor (mock)", "fixed", None,
         [box([-130.0, y_back - 5, fz - 2.0], [130.0, Y["kn_out"] + 10, fz])],
         anchor=[100.0, 0.0, fz - 1.0], mock=True, kind="mock", color=MOCK_C)

    # ---- joints: 6-32 x 3/8, ridge, nut slot (FIXTURE / SCREW_OPT helpers)
    joints = []

    def joint(tag, head, zin, slot_dir, pocket, csk, nut, ridge_on_nut, ridge_dir, ridge_half,
              csk_up, nut_up, ridge=True, slot_len=None):
        joints.append({"tag": tag, "ridge": ridge, "through": slot_len is None,
                       "slotLen": 0.0 if slot_len is None else float(slot_len),
                       "head": [float(v) for v in head], "zIn": [float(v) for v in zin],
                       "slot": [float(v) for v in slot_dir], "pocket": float(pocket),
                       "csk": csk, "nut": nut, "ridgeOnNut": bool(ridge_on_nut),
                       "ridgeDir": [float(v) for v in ridge_dir], "ridgeHalf": float(ridge_half),
                       "cskUp": [float(v) for v in csk_up], "nutUp": [float(v) for v in nut_up]})

    down, X3, Y3, Z3 = [0, 0, -1], [1, 0, 0], [0, 1, 0], [0, 0, 1]
    ym = (bh[0] + bh[1]) / 2
    joint("jF", [0, ym, Zb + jp], down, Y3, bt - jp, "bridge", "bulkheadF", True, X3, 12.0, Z3, [0, -1, 0],
          slot_len=(bh[1] - bh[0]) / 2 + 1.0)
    # both bearings' nut slots run along Y and stop, out through the face each
    # prints on -- a pocket open at the bottom, no roof (user). The front
    # bulkhead prints inner face up, so its slot leaves by the outer face.
    joint("jR", [0, -ym, Zb + jp], down, Y3, bt - jp, "bridge", "lowercase", True, X3, 12.0, Z3, [0, -1, 0],
          slot_len=(bh[1] - bh[0]) / 2 + 1.0)
    # the ears' nut slots run along Y: vertical as the case prints, and clear
    # of the case's own top wall (along X they would cut straight through it)
    if central:
        # the slot runs along X out through both of the case's side walls
        joint("jU", [0, y_uc, Zb + jp], down, X3, bt - jp, "bridge", "uppercase", True, X3,
              botOuter - 3.0, Z3, Y3)
    else:
        for sx, tag in ((1, "jU0"), (-1, "jU1")):
            joint(tag, [sx * ear_x, yH + (upTopZ + ear_z1) / 2, Zb + jp], down, Y3, bt - jp, "bridge",
                  "uppercase", True, Y3, cs["ear_depth"] / 2 - 2.0, Z3, Y3)
    for i, yj in enumerate(ch["joints_y"]):
        joint(f"jC{i}", [0, yj, Zb + bt + jp], down, X3, 0.0, "chassis", "bridge", True, Y3,
              ch["ridge_half"], Z3, Z3)
    # wing: head in the boss (countersunk from the far face), nut in the tab.
    # NO RIDGE: the panel's inner face bears on the boss's outer face and the
    # tab on its end face, so two faces and the screw locate it -- as the
    # steering's fork joint. (A ridge along the panel's oblique direction
    # would not extrude in Onshape, 2026-09-29.)
    wv = [w[0], 0.0, w[1]]
    niv = [n_in[0], 0.0, n_in[1]]
    s3 = [screw[0], 0.0, screw[1]]
    for tag, jplane, dirn, far, part_slug in (("jWR", -Y["web_out"], 1, -Y["hz_out"], "rockerR"),
                                              ("jKR", Y["kn_out"], 1, Y["kn_in"], "knuckleR")):
        head = [s3[0], jplane - dirn * jp, s3[2]]
        pocket = abs(head[1] - far)
        up = [0, 1, 0] if part_slug == "rockerR" else [0, -1, 0]
        joint(tag, head, [0, dirn, 0], wv, pocket, part_slug, "wingR", False, wv,
              0.0, up, niv, ridge=False)
        joint(tag[:-1] + "L", turned_vec(head), [0, -dirn, 0], turned_vec(wv), pocket,
              part_slug[:-1] + "L", "wingL", False, turned_vec(wv), 0.0,
              turned_vec(up), turned_vec(niv), ridge=False)
    if rear_kr:
        # knuckle R behind: head in it 4.6 from its back face, the screw
        # running back into the wing's tab
        joints[:] = [j for j in joints if j["tag"] != "jKR"]
        head = [s3[0], -Y["kr_out"] + jp, s3[2]]
        joint("jKR", head, [0, -1, 0], wv, abs(head[1] + Y["kr_in"]), "knuckleR", "wingR", False,
              wv, 0.0, [0, 1, 0], niv, ridge=False)

    for p in parts:
        p["bbox"] = bbox(p["add"]) if p["add"] else None
    near = math.hypot(t["caseHoleSpanX"] / 2, endY - xc["caseHeight"] / 2 - t["casePinRowOffset"])
    lower_seat = near - (t["caseReliefDia"] - t["casePinClearance"]) / 2 - cavR
    RG = {"yH": yH, "zC": float(C[1]), "ucAdd": uc_add, "ucCut": uc_cut,
          "ucShellPt": [float(v) for v in uc_shell_pt],
          "lowerNearPins": 1.0 if (cs["lower_near_pins"] > 0 and lower_seat >= cs["min_pin_seat"]) else 0.0,
          "joints": joints}
    L = {"Y": Y, "parts": parts, "RG": RG, "C": C, "pin": pin, "J": J, "foot": foot, "top": top,
         "Zb": Zb, "floor_z": fz, "lower_seat": lower_seat, "knee": knee, "screw": screw,
         "cavR": cavR, "lug": (lug0, lug1, lug_tip)}
    L["poses"] = {f"{f:+.2f}": poses(lk, data["T"] * f, C, pin, J)
                  for f in sorted(set(CHECK_POSES) | {p[2] for p in DIALOG_POSES})}
    return L


# -------------------------------------------------------------------- poses
def _ang(v):
    return math.degrees(math.atan2(v[1], v[0]))


def poses(lk, t: float, C, pin0, J0) -> dict:
    """Per group: a rotation `a` [deg, right hand about +Y] about `p`, then a
    shift `d`, both in (X, Z). a = bearing(before) - bearing(after)."""
    lk.reset()
    pR, pL = lk.pose(-1, t), lk.pose(1, t)
    off = np.array([0.0, lk.pivot(1)[1]])

    def xz(p):
        return np.array([-p[0], p[1]]) - off

    pin, JR, JL = xz(pR["crank_tip"]), xz(pR["joint"]), xz(pL["joint"])
    J0L = np.array([-J0[0], J0[1]])
    O = np.zeros(2)
    out = {"crank": {"p": list(C), "a": _ang(pin0 - C) - _ang(pin - C), "d": [0.0, 0.0]},
           "wR": {"p": [0.0, 0.0], "a": _ang(J0 - O) - _ang(JR - O), "d": [0.0, 0.0]},
           "wL": {"p": [0.0, 0.0], "a": _ang(J0L - O) - _ang(JL - O), "d": [0.0, 0.0]},
           "cR": {"p": list(pin0), "a": _ang(J0 - pin0) - _ang(JR - pin), "d": list(pin - pin0)},
           "cL": {"p": list(pin0), "a": _ang(J0L - pin0) - _ang(JL - pin), "d": list(pin - pin0)}}
    for g in out.values():
        g["p"] = [float(v) for v in g["p"]]
        g["d"] = [float(v) for v in g["d"]]
        g["a"] = float(g["a"])
    out["_marks"] = {"pin": [float(v) for v in pin], "JR": [float(v) for v in JR],
                     "JL": [float(v) for v in JL]}
    return out


def apply(g: dict, p) -> np.ndarray:
    a = math.radians(g["a"])
    v = np.asarray(p, float) - g["p"]
    return np.array([v[0] * math.cos(a) + v[1] * math.sin(a),
                     -v[0] * math.sin(a) + v[1] * math.cos(a)]) + g["p"] + g["d"]


def self_check(L: dict) -> None:
    pin0, J0 = L["pin"], L["J"]
    J0L = np.array([-J0[0], J0[1]])
    for key, ps in L["poses"].items():
        mk = ps["_marks"]
        for got, want in ((apply(ps["crank"], pin0), mk["pin"]), (apply(ps["wR"], J0), mk["JR"]),
                          (apply(ps["wL"], J0L), mk["JL"]), (apply(ps["cR"], pin0), mk["pin"]),
                          (apply(ps["cR"], J0), mk["JR"]), (apply(ps["cL"], pin0), mk["pin"]),
                          (apply(ps["cL"], J0L), mk["JL"])):
            if np.abs(got - np.asarray(want)).max() > 1e-6:
                raise AssertionError(f"pose {key}: transform lands at {got}, solve says {want}")


# ------------------------------------------------------------ FeatureScript
def _fs(v) -> str:
    if isinstance(v, dict):
        return "{ " + ", ".join(f'"{k}" : {_fs(x)}' for k, x in v.items()) + " }"
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_fs(x) for x in v) + "]"
    if isinstance(v, str):
        return json.dumps(v)
    if isinstance(v, bool):
        return "true" if v else "false"
    if v is None:
        return "undefined"
    t = format(float(v), ".10f").rstrip("0").rstrip(".")
    return "0" if t in ("-0", "") else t


def _fs_map(name: str, d: dict) -> str:
    rows = []
    for k, v in d.items():
        if isinstance(v, str):
            rows.append(f'    "{k}" : "{v}"')
        elif k in ("cskAngle",) or k.endswith("Deg"):
            rows.append(f'    "{k}" : {v:g} * degree')
        else:
            rows.append(f'    "{k}" : {v:.6g} * millimeter')
    return f"export const {name} = {{\n" + ",\n".join(rows) + "\n};"


def fs_data(L: dict) -> str:
    keep = ("slug", "name", "group", "up", "add", "cut", "anchor", "note", "custom", "mock",
            "color", "kind")
    parts = [{k: p[k] for k in keep} for p in L["parts"]]
    poses_ = {k: {g: v for g, v in ps.items() if not g.startswith("_")} for k, ps in L["poses"].items()}
    return (f"export const RG = {_fs(L['RG'])};\n\n"
            f"export const RG_PARTS = {_fs(parts)};\n\n"
            f"export const RG_POSES = {_fs(poses_)};\n")


FS = r'''FeatureScript %VERSION%;
import(path : "onshape/std/geometry.fs", version : "%VERSION%.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_righting --push righting_features
 *
 * Every primitive of the simple parts, the upper case's additions and cuts,
 * the joints and the poses are computed in aow_sim.cad_righting and carried
 * here as RG, RG_PARTS, RG_POSES. Millimetres. MODULE FRAME: origin on the
 * wing rod at the mid-plane between the couplers, +Y forward, +X right, +Z
 * up; the XC330 at the rear, horn forward.
 */

%X330%

%FIXTURE%

%CASE_OPT%

%SCREW_OPT%

%DATA%

// ---- the servo-mount geometry, copied from the horn-mount-gen studio ----
%SERVO_MOUNT%
// ---- end of the copy ----

%HELPERS%
/**
 * Case pins in the servo's OTHER hole row, onto `target`: aow-bike-steering's
 * nearRowPins, verbatim.
 */
export function nearRowPins(context is Context, id is Id, cs is CoordSystem, part is string,
                            target is Query)
{
    const t  = SERVO_MOUNT_TABLE["XC330"];
    const yA = cross(cs.zAxis, cs.xAxis);
    const sh = coordSystem(cs.origin - 2 * t.casePinRowOffset * yA, cs.xAxis, cs.zAxis);
    const g  = caseShellGeometry(context, id + "g", sh, mergeMaps(CASE_OPT, { "part" : part }));
    const rel = qUnion([qCreatedBy(id + "g" + "reliefRev", EntityType.BODY),
                        qCreatedBy(id + "g" + "reliefRing", EntityType.BODY)]);
    opDeleteBodies(context, id + "junk", { "entities" : qSubtraction(qCreatedBy(id + "g", EntityType.BODY),
                                                                   qUnion([g.pins, rel])) });
    opBoolean(context, id + "cut", { "tools" : rel, "targets" : target,
            "operationType" : BooleanOperationType.SUBTRACTION });
    opBoolean(context, id + "add", { "tools" : qUnion([g.pins, target]),
            "operationType" : BooleanOperationType.UNION });
}

export function rgV(a is array) returns Vector
{
    return vector(a[0], a[1], a[2]) * millimeter;
}

export function rgU(a is array) returns Vector
{
    return vector(a[0], a[1], a[2]);
}

/**
 * One 6-32 x 3/8 joint: the shared screwJoint (screw bore, countersink, nut
 * slot, teardrops, bridging, ridge and groove) with two options it lacks: no
 * ridge (`j.ridge` false) and a BLIND slot (`j.through` false: out along
 * `j.slot` for `j.slotLen`, back only past the nut).
 */
export function rgJoint(context is Context, id is Id, j is map, cskPart is Query, nutPart is Query)
{
    const f  = FIXTURE;
    const mm = millimeter;
    const head = rgV(j.head);
    const zIn = rgU(j.zIn);
    const cs = coordSystem(head, rgU(j.slot), zIn);
    const nd = f.joint_plate + 2 * mm;
    screwJointGeometry(context, id + "scr", cs, mergeMaps(SCREW_OPT, {
            "nutDepth" : nd, "headPocket" : j.pocket * mm, "bothWays" : j.through,
            "nutSlotLength" : j.through ? 200 * mm : j.slotLen * mm }));
    const bore = qCreatedBy(id + "scr" + "screwRev", EntityType.BODY);
    const slot = qCreatedBy(id + "scr" + "slotExt", EntityType.BODY);
    if (j.ridge)
    {
        const jp = head + f.joint_plate * zIn;
        const up = j.ridgeOnNut ? -zIn : zIn;
        const ridgePart = j.ridgeOnNut ? nutPart : cskPart;
        const other     = j.ridgeOnNut ? cskPart : nutPart;
        const r = ridgeSolid(context, id + "ridge", jp, rgU(j.ridgeDir), up, f.ridge_height,
                             0 * mm, j.ridgeHalf * mm);
        opBoolean(context, id + "rAdd", { "tools" : qUnion([ridgePart, r]),
                "operationType" : BooleanOperationType.UNION });
        const g = ridgeSolid(context, id + "groove", jp, rgU(j.ridgeDir), up, f.ridge_height,
                             f.ridge_clearance, j.ridgeHalf * mm + 1 * mm);
        opBoolean(context, id + "gCut", { "tools" : g, "targets" : other,
                "operationType" : BooleanOperationType.SUBTRACTION });
    }
    opBoolean(context, id + "slotCut", { "tools" : slot, "targets" : nutPart,
            "operationType" : BooleanOperationType.SUBTRACTION });
    opBoolean(context, id + "cut", { "tools" : bore, "targets" : qUnion([cskPart, nutPart]),
            "operationType" : BooleanOperationType.SUBTRACTION });
    const rB = SCREW_OPT.holeDia / 2;
    for (var pu in [[cskPart, rgU(j.cskUp), "tdC"], [nutPart, rgU(j.nutUp), "tdN"]])
    {
        if (abs(dot(pu[1], zIn)) > 0.5)
            continue;
        const upP = normalize(pu[1] - dot(pu[1], zIn) * zIn);
        polyPrism(context, id, pu[2], head - 1 * mm * zIn, zIn, cross(upP, zIn),
                  [vector(0 * mm, 0 * mm), vector(rB / sqrt(2), rB / sqrt(2)),
                   vector(0 * mm, rB * sqrt(2)), vector(-rB / sqrt(2), rB / sqrt(2))],
                  SCREW_OPT.holeDepth + 1 * mm);
        opBoolean(context, id + (pu[2] ~ "Cut"), {
                "tools" : qCreatedBy(id + (pu[2] ~ "Ext"), EntityType.BODY),
                "targets" : pu[0], "operationType" : BooleanOperationType.SUBTRACTION });
    }
    const along = dot(rgU(j.nutUp), zIn);
    if (abs(along) > 0.5)
    {
        const L  = f.bridge_layer;
        const rH = SCREW_OPT.holeDia / 2;
        const w  = SCREW_OPT.nutSlotWidth;
        const d0 = along > 0 ? nd + SCREW_OPT.nutSlotThickness : nd;
        const sg = along > 0 ? 1 : -1;
        const l1 = boxIn(context, id + "br1", cs, vector(-rH, -w / 2, d0 + (sg > 0 ? 0 * mm : -L)),
                         vector(rH, w / 2, d0 + (sg > 0 ? L : 0 * mm)));
        const l2 = boxIn(context, id + "br2", cs, vector(-rH, -rH, d0 + sg * L + (sg > 0 ? 0 * mm : -L)),
                         vector(rH, rH, d0 + sg * L + (sg > 0 ? L : 0 * mm)));
        opBoolean(context, id + "brCut", { "tools" : qUnion([l1, l2]), "targets" : nutPart,
                "operationType" : BooleanOperationType.SUBTRACTION });
    }
}

/** A cylinder along Y, a prism from an (X, Z) or (Y, Z) outline, a slot, or a box. */
export function rgPrim(context is Context, id is Id, q is map) returns Query
{
    const mm = millimeter;
    if (q.k == "cyl")
    {
        fCylinder(context, id, { "bottomCenter" : vector(q.x, q.y0, q.z) * mm,
                                 "topCenter"    : vector(q.x, q.y1, q.z) * mm,
                                 "radius"       : q.r * mm });
    }
    else if (q.k == "box")
    {
        fCuboid(context, id, { "corner1" : rgV(q.lo), "corner2" : rgV(q.hi) });
    }
    else
    {
        // sketch plane normal +Y (x axis +X, second axis -Z), or normal +X
        // (x axis +Y, second axis +Z)
        const alongX = q.k == "prismX";
        const pl = alongX ? plane(vector(q.x0, 0, 0) * mm, vector(1, 0, 0), vector(0, 1, 0))
                          : plane(vector(0, q.y0, 0) * mm, vector(0, 1, 0), vector(1, 0, 0));
        var sk = newSketchOnPlane(context, id + "sk", { "sketchPlane" : pl });
        const P = function(v) { return alongX ? vector(v[0], v[1]) * mm : vector(v[0], -v[1]) * mm; };
        if (q.k == "slot")
        {
            skLineSegment(sk, "l1", { "start" : P(q.pts[0]), "end" : P(q.pts[1]) });
            skArc(sk, "ab", { "start" : P(q.pts[1]), "mid" : P(q.mids[0]), "end" : P(q.pts[2]) });
            skLineSegment(sk, "l2", { "start" : P(q.pts[2]), "end" : P(q.pts[3]) });
            skArc(sk, "aa", { "start" : P(q.pts[3]), "mid" : P(q.mids[1]), "end" : P(q.pts[0]) });
        }
        else
        {
            for (var i = 0; i < size(q.pts); i += 1)
                skLineSegment(sk, "s" ~ i, { "start" : P(q.pts[i]), "end" : P(q.pts[(i + 1) % size(q.pts)]) });
        }
        skSolve(sk);
        opExtrude(context, id + "ext", {
                "entities"  : qSketchRegion(id + "sk"),
                "direction" : alongX ? vector(1, 0, 0) : vector(0, 1, 0),
                "endBound"  : BoundingType.BLIND,
                "endDepth"  : (alongX ? q.x1 - q.x0 : q.y1 - q.y0) * mm });
        opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(id + "sk", EntityType.BODY) });
    }
    return qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID);
}

/** One simple part: primitives united, cuts subtracted. */
export function rgPart(context is Context, id is Id, p is map)
{
    var adds = [];
    for (var i = 0; i < size(p.add); i += 1)
        adds = append(adds, rgPrim(context, id + ("a" ~ i), p.add[i]));
    if (size(adds) > 1)
        opBoolean(context, id + "uni", { "tools" : qUnion(adds),
                                         "operationType" : BooleanOperationType.UNION });
    // EVALUATED before the cutters exist: they are created under this same id
    const body = qUnion(evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID)));
    if (size(p.cut) > 0)
    {
        var cuts = [];
        for (var i = 0; i < size(p.cut); i += 1)
            cuts = append(cuts, rgPrim(context, id + ("c" ~ i), p.cut[i]));
        opBoolean(context, id + "sub", { "tools" : qUnion(cuts), "targets" : body,
                                         "operationType" : BooleanOperationType.SUBTRACTION });
    }
}

/** A part found by its anchor point, among everything the build made. */
export function rgFind(context is Context, scope is Id, slug is string) returns Query
{
    for (var p in RG_PARTS)
        if (p.slug == slug)
            return partAt(context, scope, rgV(p.anchor));
    throw regenError("no part " ~ slug);
}

/**
 * Every part and the hardware. Returns the bodies by motion group and each
 * printed part with the direction that prints UP.
 */
export function rightingBuild(context is Context, id is Id, opt is map) returns map
{
    const mm = millimeter;
    const step = function(label is string) { if (opt.debug == true) println("STEP|" ~ label); };
    for (var p in RG_PARTS)
    {
        if (p.custom || (p.mock && !opt.mocks))
            continue;
        step(p.slug);
        rgPart(context, id + p.slug, p);
    }

    // ---- the XC330, horn facing +Y, its long end up; the near hole row
    step("servo");
    const cs = coordSystem(vector(0, RG.yH, RG.zC) * mm, vector(-1, 0, 0), vector(0, 1, 0));
    const sv = x330Envelope(context, id + "servo", cs);
    {
        const T = SERVO_MOUNT_TABLE["XC330"];
        const zc = -X330.hornThickness;
        const zb = zc - X330.caseDepth;
        const rowY = X330.caseHeight - X330.shaftFromEnd - X330.caseHeight / 2 - T.casePinRowOffset;
        var nh = [];
        for (var sx in [1, -1])
        {
            const x = sx * T.caseHoleSpanX / 2;
            nh = append(nh, cylIn(context, id + ("nhf" ~ (sx > 0 ? "p" : "n")), cs,
                    vector(x, rowY, zc - T.caseReliefDepthHorn), vector(x, rowY, zc + 1 * mm), T.caseReliefDia / 2));
            nh = append(nh, cylIn(context, id + ("nhb" ~ (sx > 0 ? "p" : "n")), cs,
                    vector(x, rowY, zb - 1 * mm), vector(x, rowY, zb + T.caseReliefDepthBack), T.caseReliefDia / 2));
        }
        opBoolean(context, id + "nearHoles", { "tools" : qUnion(nh), "targets" : sv.caseQ,
                "operationType" : BooleanOperationType.SUBTRACTION });
    }

    // ---- lower case: the shared horn-half shell onto the body, near-row pins
    step("lower case shell");
    caseShellBuild(context, id + "lcShell", cs, mergeMaps(CASE_OPT, { "part" : "TOP" }),
                   rgFind(context, id, "lowercase"));
    if (RG.lowerNearPins > 0.5)
        nearRowPins(context, id + "lcNear", cs, "TOP", rgFind(context, id, "lowercase"));

    // ---- upper case: the shared back-half shell, the wrap and the pad, the
    // nest, the cavity, the cable window and the connector slots
    step("upper case");
    caseShellBuild(context, id + "ucShell", cs, mergeMaps(CASE_OPT, { "part" : "BOTTOM" }), qNothing());
    var ucAdd = [qUnion(evaluateQuery(context, partAt(context, id, rgV(RG.ucShellPt))))];
    for (var i = 0; i < size(RG.ucAdd); i += 1)
        ucAdd = append(ucAdd, rgPrim(context, id + ("ucA" ~ i), RG.ucAdd[i]));
    opBoolean(context, id + "ucU", { "tools" : qUnion(ucAdd), "operationType" : BooleanOperationType.UNION });
    const ucT = qUnion(evaluateQuery(context, rgFind(context, id, "uppercase")));
    var ucCut = [];
    for (var i = 0; i < size(RG.ucCut); i += 1)
        ucCut = append(ucCut, rgPrim(context, id + ("ucC" ~ i), RG.ucCut[i]));
    opBoolean(context, id + "ucSub", { "tools" : qUnion(ucCut), "targets" : ucT,
            "operationType" : BooleanOperationType.SUBTRACTION });
    nearRowPins(context, id + "ucNear", cs, "BOTTOM", rgFind(context, id, "uppercase"));

    // ---- the 6-32 joints
    step("joints");
    for (var j in RG.joints)
    {
        if (!opt.mocks && (j.csk == "chassis" || j.nut == "chassis"))
            continue;
        step("joint " ~ j.tag);
        rgJoint(context, id + j.tag, j, rgFind(context, id, j.csk), rgFind(context, id, j.nut));
    }

    // ---- names, colours, groups, print orientation
    step("dress");
    var groups = { "fixed" : [], "crank" : [], "cR" : [], "cL" : [], "wR" : [], "wL" : [] };
    var prints = [];
    for (var p in RG_PARTS)
    {
        if (p.mock && !opt.mocks)
            continue;
        const q = qUnion(evaluateQuery(context, rgFind(context, id, p.slug)));
        dress(context, q, p.name, color(p.color[0], p.color[1], p.color[2]), p.note);
        groups[p.group] = append(groups[p.group], q);
        if (p.up != undefined && p.kind == "print")
            prints = append(prints, [p.name, q, rgU(p.up)]);
    }
    if (opt.mocks)
    {
        dress(context, sv.caseQ, "X330 case", color(0.16, 0.16, 0.18), "");
        dress(context, sv.horn, "X330 horn", color(0.3, 0.3, 0.32), "");
        groups["fixed"] = append(groups["fixed"], sv.caseQ);
        groups["crank"] = append(groups["crank"], sv.horn);
    }
    else
        opDeleteBodies(context, id + "noServo", { "entities" : qUnion([sv.caseQ, sv.horn]) });
    return { "groups" : groups, "prints" : prints };
}

/** The transform that puts one group at one pose. */
export function rgXf(g is map) returns Transform
{
    const mm = millimeter;
    return transform(vector(g.d[0], 0, g.d[1]) * mm)
         * rotationAround(line(vector(g.p[0], 0, g.p[1]) * mm, vector(0, 1, 0)), g.a * degree);
}

/** Move every moving group to pose `key` from rest, or back with `back`. */
export function rgPose(context is Context, id is Id, groups is map, key is string, back is boolean)
{
    const ps = RG_POSES[key];
    for (var g in ["crank", "cR", "cL", "wR", "wL"])
    {
        if (size(groups[g]) == 0)
            continue;
        const xf = rgXf(ps[g]);
        opTransform(context, id + g, { "bodies" : qUnion(groups[g]),
                                       "transform" : back ? inverse(xf) : xf });
    }
}

%SPLIT%

export enum RightingPose
{
%ENUM%
}

annotation { "Feature Type Name" : "AOW righting",
             "Feature Type Description" : "Printed swing linkage: crankshaft on two bearings, XC330 cases and horn hub, wings; Y forward, origin on the wing rod" }
export const aowRighting = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Pose" }
        definition.pose is RightingPose;
        annotation { "Name" : "Show mocks (servo, chassis plate, floor)", "Default" : true }
        definition.mocks is boolean;
    }
    {
        const r = rightingBuild(context, id + "build", { "mocks" : definition.mocks });
        const keys = %KEYS%;
        const key = keys[definition.pose];
        if (key != "+0.00")
            rgPose(context, id + "pose", r.groups, key, false);
        reportFeatureInfo(context, id, "%INFO%");
    });
'''


def build_fs(L: dict, data: dict, fs_version: str = "3044") -> str:
    t = data["table"]["XC330"]
    pick = lambda keys: {"servo": "XC330", **{k: t[k] * 1000 for k in keys}}  # noqa: E731
    fixture = {k: v for k, v in data["s"]["joint"].items()}
    enum = ",\n".join(f'    annotation {{ "Name" : "{label}" }} {name}' for name, label, _f in DIALOG_POSES)
    keys = "{ " + ", ".join(f'RightingPose.{name} : "{f:+.2f}"' for name, _l, f in DIALOG_POSES) + " }"
    lk = data["lk"]
    info = (f"Diamond x{data['scale']:g}: crank {lk.crank:.2f}, coupler {lk.coupler:.2f}, "
            f"rocker {lk.rocker:.2f}; crank axis {L['C'][1]:.2f} above the rod, rod "
            f"{-L['floor_z']:.1f} above the floor; rods {data['s']['rod']['dia']:g} mm")
    subs = {"%VERSION%": fs_version, "%X330%": _fs_map("X330", data["x330"]),
            "%FIXTURE%": _fs_map("FIXTURE", fixture),
            "%CASE_OPT%": _fs_map("CASE_OPT", pick(sm.CASE_DIALOG)),
            "%SCREW_OPT%": _fs_map("SCREW_OPT", data["screw"]),
            "%DATA%": fs_data(L),
            "%SERVO_MOUNT%": af._servo_mount_layer(data, fs_version),
            "%HELPERS%": af.FS_HELPERS.rstrip("\n") + "\n",
            "%SPLIT%": SPLIT_MARK, "%ENUM%": enum, "%KEYS%": keys, "%INFO%": info}
    text = FS
    for k, v in subs.items():
        text = text.replace(k, v)
    return text


# ------------------------------------------------------------------- check
MARKS = ("crankpin", "rocker pin R", "rocker pin L")


def check_wrapper(fs: str) -> str:
    keys = [f"{f:+.2f}" for f in CHECK_POSES]
    return f"""function(context is Context, queries)
{{
{sm.geometry_layer(fs, SPLIT_MARK)}
    const name = function(q) returns string
    {{
        const n = getProperty(context, {{ "entity" : q, "propertyType" : PropertyType.NAME }});
        return n == undefined ? "UNNAMED" : n;
    }};
    const mm = millimeter;
    const id = makeId("chk");
    const r = rightingBuild(context, id, {{ "mocks" : true, "debug" : true }});
    const all = evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID));
    for (var b in all)
    {{
        const bb = evBox3d(context, {{ "topology" : b, "tight" : true }});
        println("BODY|" ~ name(b) ~ "|" ~ toString(bb.minCorner[0] / mm) ~ "|" ~ toString(bb.minCorner[1] / mm)
                ~ "|" ~ toString(bb.minCorner[2] / mm) ~ "|" ~ toString(bb.maxCorner[0] / mm) ~ "|"
                ~ toString(bb.maxCorner[1] / mm) ~ "|" ~ toString(bb.maxCorner[2] / mm) ~ "|"
                ~ toString(evVolume(context, {{ "entities" : b }}) / (mm * mm * mm)));
    }}
    for (var i = 0; i + 1 < size(all); i += 1)
        for (var c in evCollision(context, {{ "tools" : all[i], "targets" : qUnion(subArray(all, i + 1, size(all))) }}))
            println("COLL|rest|" ~ name(c.toolBody) ~ "|" ~ name(c.targetBody) ~ "|" ~ toString(c["type"]));
    const keys = {_fs(keys)};
    for (var k = 0; k < size(keys); k += 1)
    {{
        const key = keys[k];
        rgPose(context, id + ("go" ~ k), r.groups, key, false);
        for (var g in ["crank", "cR", "cL", "wR", "wL"])
        {{
            const mine = evaluateQuery(context, qUnion(r.groups[g]));
            var others = [];
            for (var b in all)
                if (!isIn(b, mine))
                    others = append(others, b);
            for (var c in evCollision(context, {{ "tools" : qUnion(mine), "targets" : qUnion(others) }}))
                println("COLL|" ~ key ~ "|" ~ name(c.toolBody) ~ "|" ~ name(c.targetBody) ~ "|" ~ toString(c["type"]));
        }}
        for (var b in all)
            if (isIn(name(b), {_fs(list(MARKS))}))
            {{
                const bb = evBox3d(context, {{ "topology" : b, "tight" : true }});
                println("MARK|" ~ key ~ "|" ~ name(b) ~ "|" ~ toString((bb.minCorner[0] + bb.maxCorner[0]) / 2 / mm)
                        ~ "|" ~ toString((bb.minCorner[2] + bb.maxCorner[2]) / 2 / mm));
            }}
        rgPose(context, id + ("bk" ~ k), r.groups, key, true);
    }}
{af.PRINT_CHECK_FS}    return "ran to completion";
}}
"""


# parts whose bounding box is exactly their primitives' (no joint or shell adds to them)
PURE = ("half crank R", "half crank L", "coupler R", "coupler L", "crankpin", "rocker pin R",
        "rocker pin L", "wing rod", "horn hub", "rocker R", "rocker L", "knuckle R", "knuckle L")


def judge(console: str, L: dict, data: dict) -> bool:
    rows = [l.split("|") for l in console.splitlines()]
    ok = True
    by_name = {p["name"]: p for p in L["parts"]}
    vols = {}
    bodies = [r for r in rows if r[0] == "BODY"]
    strays = [r for r in bodies if r[1] == "UNNAMED"]
    if strays:
        ok = False
        print(f"  {len(strays)} UNNAMED bodies -- a cutter left behind  FAIL")
    bad = 0
    for r in bodies:
        vols[r[1]] = float(r[8])
        p = by_name.get(r[1])
        if p is None or r[1] not in PURE:
            continue
        got = [float(v) for v in r[2:8]]
        want = p["bbox"][0] + p["bbox"][1]
        err = max(abs(a - b) for a, b in zip(got, want))
        if err > 1e-3:
            bad += 1
            print(f"  {r[1]:24} bbox off by {err:.4f}  FAIL  got {np.round(got, 2)} want {np.round(want, 2)}")
    ok &= bad == 0
    print(f"  {len(bodies)} bodies; {len(PURE)} plain parts' bounding boxes as computed: {'ok' if not bad else 'NO'}")
    for r in rows:
        if r[0] == "PART" and int(r[2]) != 1:
            ok = False
            print(f"  part {r[1]} is {r[2]} bodies  FAIL")
    for p in L["parts"]:
        if p["twin"] or p.get("mirror"):
            twin = next(q["name"] for q in L["parts"] if q["slug"] == (p["twin"] or p["mirror"]))
            a, b = vols.get(twin), vols.get(p["name"])
            same = None not in (a, b) and abs(a - b) <= 1e-6 * max(a, b)
            ok &= same
            print(f"  {p['name']:18} same part as {twin:18} ({a}, {b}): {'ok' if same else 'FAIL'}")
    colls = [r for r in rows if r[0] == "COLL" and "ABUT" not in r[4]]
    pairs = sorted({(r[1], *sorted(r[2:4])) for r in colls})
    ok &= not pairs
    print(f"  interference, at rest and over {len(CHECK_POSES)} poses: {len(pairs)}")
    for r in pairs[:60]:
        print(f"    {r[0]:6}  {r[1]}  x  {r[2]}")
    worst = 0.0
    for r in rows:
        if r[0] != "MARK":
            continue
        mk = L["poses"][r[1]]["_marks"]
        want = {"crankpin": mk["pin"], "rocker pin R": mk["JR"], "rocker pin L": mk["JL"]}[r[2]]
        worst = max(worst, abs(float(r[3]) - want[0]), abs(float(r[4]) - want[1]))
    good = worst < 1e-3
    ok &= good
    print(f"  pins where the solver puts them, every pose: worst {worst:.2e} mm  {'ok' if good else 'FAIL'}")
    dens = data["s"]["materials"]
    asa = sum(v for n, v in vols.items() if n in by_name and by_name[n]["kind"] == "print")
    steel = sum(v for n, v in vols.items() if n in by_name and by_name[n]["kind"] == "steel")
    print(f"  mass, solid: ASA {asa * dens['asa'] / 1000:.1f} g, steel {steel * dens['steel'] / 1000:.1f} g "
          f"(no servo, screws or chassis plate)")
    for r in rows:
        if r[0] == "BED":
            over = sorted((x for x in rows if x[0] == "OVER" and x[1] == r[1]), key=lambda o: -float(o[2]))
            print(f"    {r[1]:24} bed {float(r[2]):7.1f} mm^2, {len(over)} downward faces"
                  + "".join(f"\n        {float(o[2]):7.2f} mm^2, {float(o[3]):6.2f} up, at ({o[4]})"
                            for o in over[:10]))
    crowns = [r for r in rows if r[0] == "CROWN"]
    print(f"  horizontal holes with a flat crown as printed (want a teardrop): {len(crowns)}")
    for r in crowns:
        print(f"    {r[1]:24} r {r[2]} mm, axis through ({r[3]})")
    hangs = [r for r in rows if r[0] == "HANG"]
    print(f"  level edges hanging in the air as printed: {len(hangs)}")
    for r in hangs:
        print(f"    {r[1]:24} {float(r[2]):5.2f} mm long, {float(r[3]):6.2f} up, at ({r[4]})")
    return ok


def check(text: str, L: dict, data: dict, target: str | None) -> bool:
    """ONE billable call. The whole reply is saved: a failed eval still bills,
    and its notices are the only record of why."""
    from . import onshape

    url = onshape.resolve(target, "check")
    reply = onshape.eval_featurescript(check_wrapper(text), url)
    Path("traces").mkdir(exist_ok=True)
    Path("traces/righting_check.json").write_text(json.dumps(reply, indent=1, default=str))
    console = reply.get("console") or ""
    Path("traces/righting_check.txt").write_text(console)
    for line in onshape.notice_lines(reply):
        print(f"  {line}")
    if any(n["message"]["level"] == "ERROR" for n in reply.get("notices", [])):
        steps = [l for l in console.splitlines() if l.startswith("STEP|")]
        print(f"  last step reached: {steps[-1] if steps else 'none'}")
        print(onshape.budget_line())
        return False
    ok = judge(console, L, data)
    print(onshape.budget_line())
    return ok


# ------------------------------------------------------------------ report
def report(L: dict, data: dict) -> str:
    Y, lk = L["Y"], data["lk"]
    out = [f"  diamond x{data['scale']:g}: crank {lk.crank:.2f}  coupler {lk.coupler:.2f}  "
           f"rocker {lk.rocker:.2f}  stroke +-{data['T']:.1f} deg; rods {data['s']['rod']['dia']:g} mm",
           f"  crank axis {L['C'][1]:.2f} above the rod; rod {-L['floor_z']:.1f} above the floor; "
           f"bridge underside {L['Zb']:.2f}, top {L['Zb'] + data['s']['frame']['bridge']:.2f} above the rod",
           "  along Y from the mid-plane (magnitudes):"]
    for k in ("cpl_in", "cpl_out", "web_in", "web_out", "hz_out", "bh_in", "bh_out", "kn_in", "kn_out"):
        out.append(f"    {Y[k]:7.2f}  {k}")
    if data["s"]["wing"]["knuckle_r"] == "rear":
        out.append(f"    {Y['kr_in']:7.2f}..{Y['kr_out']:.2f}  knuckle R, BEHIND knuckle L")
    out.append(f"    rod from {-Y['rod_back']:.2f} to {Y['rod_front']:.2f}")
    out.append(f"  behind the rear bearing: journal end {Y['jn_end']:.2f}, hub {Y['hub_front']:.2f}-"
               f"{Y['horn_face']:.2f}, horn face {Y['horn_face']:.2f}")
    lug0, lug1, tip = L["lug"]
    out.append(f"  lugs r {lug0:.2f}-{lug1:.2f} (inside the Phi {2 * (lug1 + 0.1):.1f} journal), tip at "
               f"{tip:.2f}; hub cavity r {L['cavR']:.2f}; lower case near-row pins "
               f"{'ON' if L['RG']['lowerNearPins'] else 'OFF'} ({L['lower_seat']:.2f} clear)")
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", default=PARAMS)
    ap.add_argument("-o", "--output", default=OUT_FS)
    ap.add_argument("--fs-version", default="3044")
    ap.add_argument("--check", metavar="TAB|URL", nargs="?", const="", default=None,
                    help="build in Onshape and check it; ONE billable call, `check` tab")
    ap.add_argument("--push", metavar="TAB|URL", nargs="?", const="", default=None,
                    help=f"replace a Feature Studio's contents (`{TAB}` only)")
    ap.add_argument("--shot", metavar="TAB|URL", nargs="?", const="", default=None,
                    help=f"render a Part Studio (default `{PART_STUDIO}`); ONE billable call")
    ap.add_argument("--view", default="isometric")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the check script instead of spending a call")
    args = ap.parse_args()

    data = load(args.params)
    L = layout(data)
    self_check(L)
    text = build_fs(L, data, args.fs_version)
    sm.lint_fs(text)
    Path(args.output).write_text(text)
    print(f"wrote {len(text)} chars -> {args.output}  ({len(L['parts'])} parts)")
    print(report(L, data))
    if args.dry_run:
        print(check_wrapper(text))
        return
    if args.check is not None:
        if not check(text, L, data, args.check or None):
            raise SystemExit("check FAILED -- not pushing")
    if args.push is not None:
        from . import onshape
        if args.push != TAB:
            raise SystemExit(f"refusing to push at {args.push or 'the default'!r}: "
                             f"this generator owns `{TAB}` only")
        url = onshape.resolve(args.push, args.push)
        reply = onshape.push_feature_studio(text, url)
        print(f"pushed {len(text)} chars -> {url}  (microversion "
              f"{reply.get('sourceMicroversion', '?')})")
        print(onshape.budget_line())
    if args.shot is not None:
        from . import onshape
        url = onshape.resolve(args.shot or None, PART_STUDIO)
        tag = "" if args.view == "isometric" else "_" + args.view
        out, _ = onshape.shaded_view(url, Path(OUT_PNG.format(tag)), view=args.view)
        print(f"rendered -> {out}")
        print(onshape.budget_line())


if __name__ == "__main__":
    main()
