"""The machined swing linkage: the diamond four-bar as a supported crankshaft,
every pivot in a Delrin bushing, the XC330 coupled for torque only. ONE custom
feature, `AOW wing linkage metal`, in its own Feature Studio. Spec:
docs/plans/wing-linkage-metal.md. Numbers: config/wing_linkage_metal_cad.yaml.

    python -m aow_sim.cad_wing_linkage_metal                  # write the .fs, print the layout
    python -m aow_sim.cad_wing_linkage_metal --check          # ONE call, `check` tab
    python -m aow_sim.cad_wing_linkage_metal --push wing_metal_features
    python -m aow_sim.cad_wing_linkage_metal --shot           # -> docs/cad/wing_linkage_metal.png

WHAT IT IS FOR. The righting linkage takes the hardest knocks on the bike: the
sim's pin peaks are 72 N lifting and ~160 N in a fall (the diamond, 9.9 V,
docs/plans/righting-linkage-margin.md). So the load path is designed first:

  - the crank is a CRANKSHAFT -- two identical half-cranks (turned journal +
    milled web) and a pressed steel crankpin -- running in a bushing in each
    bulkhead. The XC330 drives it through an OLDHAM coupling with a Delrin
    disc, which passes torque and nothing else: no radial or axial load
    reaches the servo's output bearing, whatever the pins see;
  - both wings' rockers run on one fixed steel rod, on long bushed hubs, and
    each wing has a SECOND knuckle on the rod beyond the far bulkhead, so a
    panel is held at both ends and a knock on it goes rod -> bulkheads.

THE FOUR-BAR is config/swing_linkage_shared_rod.yaml scaled about the rod
(`linkage.scale`). Scaling every length keeps every angle, so the current the
stroke needs is unchanged (planar scorer, 1.0-1.5); it buys wall thickness and
1/scale of the pin forces. The panel is re-derived onto the same line through
`swing_synthesis.candidate`, as the synthesis does, so it is the same panel.

DATA-DRIVEN, deliberately. Python computes every primitive of every part --
cylinders along Y, prisms from an (X, Z) outline, boxes -- and the transforms
for each pose; the FeatureScript is a small loop that builds them. So the one
billed check can hold Onshape's result against this file's numbers: every
body's bounding box, and where the pins land at each pose.

L/R PAIRS ARE ONE PART. The stack along Y is symmetric about the mid-plane and
the rest pose about X = 0, so each left part is its right twin turned 180 deg
about Z -- built here by mapping the right part's primitives, and checked by
volume.

MODULE FRAME: origin on the rod axis at the mid-plane between the couplers, +Y
forward, +X right, +Z up.
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

from . import cad_servo_mount as sm
from .params import _normalize, load_params

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "analysis"))
import swing_synthesis as ss                    # noqa: E402

PARAMS = "config/wing_linkage_metal_cad.yaml"
OUT_FS = "docs/cad/wing_linkage_metal.fs"
OUT_PNG = "docs/cad/wing_linkage_metal{}.png"
SPLIT_MARK = "// ==== UI LAYER BELOW -- dropped by --check ===="
TAB = "wing_metal_features"
PART_STUDIO = "wing_metal"

# Poses as a fraction of the stroke, + deploys the RIGHT wing. The check runs
# all of them; +-0.56 is where the web comes closest to the rocker arm that
# shares its plane (2D, 4.3 mm at 1.25 scale).
CHECK_POSES = (-1.0, -0.75, -0.56, -0.5, -0.25, 0.0, 0.25, 0.5, 0.56, 0.75, 1.0)
DIALOG_POSES = (("REST", "Rest", 0.0),
                ("R25", "Right wing 25%", 0.25), ("R50", "Right wing 50%", 0.5),
                ("R75", "Right wing 75%", 0.75), ("R100", "Right wing down (100%)", 1.0),
                ("L25", "Left wing 25%", -0.25), ("L50", "Left wing 50%", -0.5),
                ("L75", "Left wing 75%", -0.75), ("L100", "Left wing down (100%)", -1.0))

COLORS = {"aluminium": (0.72, 0.74, 0.78), "steel": (0.30, 0.30, 0.32),
          "delrin": (0.95, 0.94, 0.88), "printed": (0.93, 0.56, 0.20),
          "mock": (0.16, 0.16, 0.18), "panel": (0.45, 0.70, 0.50),
          "floor": (0.85, 0.85, 0.85)}


# ------------------------------------------------------------------- inputs
def load(path: str = PARAMS, cad_path: str = sm.CAD_PARAMS) -> dict:
    s = _normalize(yaml.safe_load(Path(path).read_text()))
    ref = yaml.safe_load((ROOT / s["linkage"]["config"]).read_text())
    k = float(s["linkage"]["scale"])
    m = ref["mechanism"]
    pz = m["wing_pivot_z"]
    x = [m["crank_length"] * k, m["coupler_length"] * k, m["rocker_length"] * k,
         m["angle_between_cranks"], pz + k * (m["servo_offset"] - pz)]
    lk = ss.candidate(ref, x, ss.reference_panel(ref, None), 0.0)
    sv = load_params(cad_path)["servos"]["xc330_t181"]
    d, w, h = (v * 1000 for v in sv["box_size"])
    xc = {"depth": d, "width": w, "height": h, "shaft_from_end": sv["shaft_from_end"] * 1000,
          "horn_t": sv["horn_thickness"] * 1000, "horn_r": sv["horn_diameter"] * 500}
    return {"s": s, "lk": lk, "T": float(ref["stroke"]["crank_travel_deg"]), "xc": xc,
            "pivot_z": pz, "wheel_r": ref["bike"]["wheel_radius"], "scale": k}


def xz(p) -> np.ndarray:
    """Study sketch (y, v) -> module (X, Z): X = -y, rod on the origin."""
    return np.array([-p[0], p[1] - _PZ[0]])


_PZ = [0.0]


# ---------------------------------------------------------------- 2D shapes
def taper_quad(a, b, ra, rb) -> list:
    """The quad between the two external tangents of circles (a, ra), (b, rb):
    with the two cylinders it makes a tapered link."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    u = (b - a) / np.linalg.norm(b - a)
    perp = np.array([-u[1], u[0]])
    c = (ra - rb) / np.linalg.norm(b - a)
    s = math.sqrt(1 - c * c)
    n1, n2 = c * u + s * perp, c * u - s * perp
    return [a + ra * n1, b + rb * n1, b + rb * n2, a + ra * n2]


def cyl(p, y0, y1, r):
    return {"k": "cyl", "x": float(p[0]), "z": float(p[1]), "y0": float(y0), "y1": float(y1),
            "r": float(r)}


def prism(pts, y0, y1):
    return {"k": "prism", "pts": [[float(q[0]), float(q[1])] for q in pts],
            "y0": float(y0), "y1": float(y1)}


def box(lo, hi):
    return {"k": "box", "lo": [float(v) for v in lo], "hi": [float(v) for v in hi]}


def link(a, b, ra, rb, y0, y1) -> list:
    """A tapered link as ONE profile -- two tangent arcs, two lines -- so it is
    one extrude. Built first as a quad unioned with two cylinders, whose flat
    faces lie exactly tangent to the cylinders: Onshape's boolean refused it
    (BOOLEAN_INVALID, 2026-09-29)."""
    q = taper_quad(a, b, ra, rb)
    u = (np.asarray(b, float) - np.asarray(a, float))
    u = u / np.linalg.norm(u)
    return [{"k": "slot", "pts": [[float(v[0]), float(v[1])] for v in q],
             "mids": [[float(v[0]), float(v[1])] for v in (np.asarray(b) + rb * u,
                                                           np.asarray(a) - ra * u)],
             "a": [float(a[0]), float(a[1])], "b": [float(b[0]), float(b[1])],
             "ra": float(ra), "rb": float(rb), "y0": float(y0), "y1": float(y1)}]


def turned(pr: dict) -> dict:
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
    else:
        lo, hi = q["lo"], q["hi"]
        q["lo"], q["hi"] = [-hi[0], -hi[1], lo[2]], [-lo[0], -lo[1], hi[2]]
    return q


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
        else:
            a, b = q["lo"], q["hi"]
        lo = [min(u, v) for u, v in zip(lo, a)]
        hi = [max(u, v) for u, v in zip(hi, b)]
    return lo, hi


# ------------------------------------------------------------------- layout
def layout(data: dict) -> dict:
    """Every part as primitives, the axial stack, and the poses."""
    s, lk, xcd = data["s"], data["lk"], data["xc"]
    _PZ[0] = data["pivot_z"]
    st, sh, bu, wl, pn, fr, od, sv = (s[k] for k in ("stack", "shafts", "bushings", "walls",
                                                    "panel", "frame", "oldham", "servo"))
    g = st["washer"]
    Y = {}
    Y["cpl_in"] = g / 2
    Y["cpl_out"] = Y["cpl_in"] + st["coupler"]
    Y["web_in"] = Y["cpl_out"] + g
    Y["web_out"] = Y["web_in"] + st["web"]
    Y["hz_out"] = Y["web_out"] + st["hub_zone"]
    Y["bh_in"] = Y["hz_out"] + bu["flange"]
    Y["bh_out"] = Y["bh_in"] + st["bulkhead"]
    Y["kn_in"] = Y["bh_out"] + g
    Y["kn_out"] = Y["kn_in"] + st["knuckle"]
    Y["rod"] = Y["kn_out"] + sh["rod_past_knuckle"]
    # Oldham, behind the rear bulkhead (negative Y; magnitudes here)
    Y["jn_end"] = Y["bh_out"] + od["gap"]                 # journal end face
    Y["disc_in"] = Y["jn_end"] + od["gap"]
    Y["disc_out"] = Y["disc_in"] + od["disc_thickness"]
    Y["hub_in"] = Y["disc_out"] + od["gap"]
    Y["horn_face"] = Y["hub_in"] + od["hub_thickness"]
    Y["case_face"] = Y["horn_face"] + xcd["horn_t"]
    Y["case_back"] = Y["case_face"] + xcd["depth"]
    Y["back_plate"] = Y["case_back"] + sv["back_plate"]

    O = np.zeros(2)
    C = xz(lk.shaft)
    pr = lk.pose(-1, 0.0)
    pin = xz(pr["crank_tip"])
    J = xz(pr["joint"])
    foot, top = xz(pr["foot"]), xz(pr["top"])
    w = (top - foot) / np.linalg.norm(top - foot)
    n_in = np.array([-w[1], w[0]])
    if np.dot(n_in, O - foot) < 0:
        n_in = -n_in
    half = pn["thickness"] / 2
    u0, u1 = pn["pad_from_foot"], pn["pad_from_foot"] + pn["pad_length"]
    pad = [foot + u * w + o * n_in for u, o in ((u0, half), (u1, half),
                                                (u1, half + pn["pad_thickness"]),
                                                (u0, half + pn["pad_thickness"]))]
    pad_c = foot + (u0 + u1) / 2 * w + (half + pn["pad_thickness"] / 2) * n_in
    # 0.5 inside the pad's outer face, not tangent to it: a tangent contact is
    # what the boolean refuses
    ear_end = pad_c + (wl["ear_half_width"] - pn["pad_thickness"] / 2 + 0.5) * n_in
    panel = [foot + half * n_in, top + half * n_in, top - half * n_in, foot - half * n_in]

    r_rod, r_jn = sh["rod_dia"] / 2, sh["journal_dia"] / 2
    r_pin, r_rp = sh["crankpin_dia"] / 2, sh["rocker_pin_dia"] / 2
    parts = []

    def part(slug, name, mat, group, add, cut=(), note="", twin=None):
        parts.append({"slug": slug, "name": name, "mat": mat, "group": group,
                      "add": list(add), "cut": list(cut), "note": note, "twin": twin})

    def twin_of(p, slug, name, group):
        part(slug, name, p["mat"], group, [turned(q) for q in p["add"]],
             [turned(q) for q in p["cut"]], p["note"], twin=p["slug"])

    # ---- the right half (rear, Y < 0); left twins are turned copies
    wy = (-Y["web_out"], -Y["web_in"])
    cy = (-Y["cpl_out"], -Y["cpl_in"])
    part("crankR", "half crank R", "aluminium", "crank",
         link(C, pin, wl["web_hub_r"], wl["pin_boss_r"], *wy)
         + [cyl(C, -Y["hz_out"], -Y["web_out"], sh["journal_shoulder_dia"] / 2),
            cyl(C, -Y["jn_end"], -Y["hz_out"], r_jn)],
         [cyl(pin, wy[0] - 1, wy[1] + 1, r_pin),
          box([C[0] - r_jn - 1, -Y["jn_end"] - 1, C[1] - od["slot_width"] / 2],
              [C[0] + r_jn + 1, -Y["jn_end"] + od["slot_depth"], C[1] + od["slot_width"] / 2])],
         "Turned journal + milled web, pressed on the crankpin. Oldham slot across X "
         "in the journal end (the front twin's is unused).")
    part("couplerR", "coupler R", "aluminium", "cR",
         link(pin, J, wl["big_eye_r"], wl["small_eye_r"], *cy),
         [cyl(pin, cy[0] - 1, cy[1] + 1, bu["big_end_od"] / 2),
          cyl(J, cy[0] - 1, cy[1] + 1, bu["small_end_od"] / 2)],
         f"{lk.coupler:.2f} mm centres. Delrin bushings both eyes.")
    part("bigR", "big end bushing R", "delrin", "cR", [cyl(pin, *cy, bu["big_end_od"] / 2)],
         [cyl(pin, cy[0] - 1, cy[1] + 1, r_pin)])
    part("smallR", "small end bushing R", "delrin", "cR", [cyl(J, *cy, bu["small_end_od"] / 2)],
         [cyl(J, cy[0] - 1, cy[1] + 1, r_rp)])
    part("washJR", "washer rocker R", "delrin", "cR", [cyl(J, -Y["web_in"], -Y["cpl_out"], 5.0)],
         [cyl(J, -Y["web_in"] - 1, -Y["cpl_out"] + 1, r_rp)])
    hub = (-Y["hz_out"], -Y["web_in"])
    ear = (-Y["hz_out"], -Y["web_out"])
    part("rockerR", "rocker R", "aluminium", "wR",
         link(O, J, wl["rocker_hub_r"], wl["rocker_eye_r"], *wy)
         + link(O, ear_end, wl["rocker_hub_r"], wl["ear_half_width"], *ear)
         + [prism(pad, -Y["kn_out"], -Y["web_out"])],
         [cyl(O, hub[0] - 1, hub[1] + 1, bu["rod_od"] / 2),
          cyl(J, wy[0] - 1, wy[1] + 1, r_rp)],
         f"Hub on the rod (Delrin bushed), arm to the coupler at {lk.rocker:.2f} mm, "
         "ear to the panel pad. The pad runs back past the bulkhead.")
    part("rodbushR", "rocker bushing R", "delrin", "wR",
         [cyl(O, *hub, bu["rod_od"] / 2), cyl(O, -Y["bh_in"], -Y["hz_out"], bu["rod_flange_dia"] / 2)],
         [cyl(O, -Y["bh_in"] - 1, hub[1] + 1, r_rod)])
    part("rpinR", "rocker pin R", "steel", "wR", [cyl(J, -Y["web_out"], -Y["cpl_in"], r_rp)],
         note="Pressed in the rocker arm, runs in the coupler's small end.")
    kn = (Y["kn_in"], Y["kn_out"])
    part("knuckleR", "knuckle R", "aluminium", "wR",
         link(O, ear_end, wl["rocker_hub_r"], wl["ear_half_width"], *kn)
         + [prism(pad, *kn)],
         [cyl(O, kn[0] - 1, kn[1] + 1, bu["rod_od"] / 2)],
         "The wing's second support, ahead of the far bulkhead: carries panel loads "
         "to the rod, drives nothing.")
    part("knbushR", "knuckle bushing R", "delrin", "wR",
         [cyl(O, *kn, bu["rod_od"] / 2), cyl(O, Y["bh_out"], Y["kn_in"], bu["rod_flange_dia"] / 2)],
         [cyl(O, Y["bh_out"] - 1, kn[1] + 1, r_rod)])
    part("panelR", "wing panel R (mock)", "panel", "wR",
         [prism(panel, -Y["kn_out"], Y["kn_out"])],
         note="Placeholder panel on the linkage config's line; bolts to the two pads.")
    bh = (-Y["bh_out"], -Y["bh_in"])
    top_z, tb = fr["top_z"], fr["top_band"]
    part("bulkheadR", "bulkhead R", "aluminium", "fixed",
         [cyl(O, *bh, fr["bulkhead_rod_r"]), cyl(C, *bh, fr["bulkhead_boss_r"]),
          box([-fr["bulkhead_half_width"], bh[0], 0.0], [fr["bulkhead_half_width"], bh[1], top_z]),
          box([-fr["top_half_width"], bh[0], top_z - tb], [fr["top_half_width"], bh[1], top_z])],
         [cyl(O, bh[0] - 1, bh[1] + 1, r_rod), cyl(C, bh[0] - 1, bh[1] + 1, bu["journal_od"] / 2)],
         "Rod clamped, crank bushing pressed. Top face screws to the bridge.")
    part("jbushR", "journal bushing R", "delrin", "fixed",
         [cyl(C, *bh, bu["journal_od"] / 2), cyl(C, bh[1], -Y["hz_out"], bu["journal_flange_dia"] / 2)],
         [cyl(C, bh[0] - 1, -Y["hz_out"] + 1, r_jn)])

    for p in list(parts):
        if p["slug"].endswith("R"):
            g2 = {"wR": "wL", "cR": "cL"}.get(p["group"], p["group"])
            twin_of(p, p["slug"][:-1] + "L", p["name"][:-1] + "L"
                    if p["name"].endswith(" R") else p["name"].replace(" R ", " L "), g2)

    # ---- the unpaired parts
    for i, y in enumerate((-Y["web_in"], -Y["cpl_in"], Y["cpl_out"])):
        part(f"washP{i}", f"washer crankpin {i + 1}", "delrin", "crank",
             [cyl(pin, y, y + g, 5.0)], [cyl(pin, y - 1, y + g + 1, r_pin)])
    part("crankpin", "crankpin", "steel", "crank", [cyl(pin, -Y["web_out"], Y["web_out"], r_pin)],
         note="Steel dowel, pressed in both webs.")
    part("rod", "wing rod", "steel", "fixed", [cyl(O, -Y["rod"], Y["rod"], r_rod)],
         note="Steel, clamped in both bulkheads; clips at the ends.")
    sw, sd = od["slot_width"], od["slot_depth"]
    rd = od["disc_dia"] / 2
    part("oldham", "oldham disc", "delrin", "crank",
         [cyl(C, -Y["disc_out"], -Y["disc_in"], rd),
          box([C[0] - rd / 2, -Y["disc_in"], C[1] - sw / 2],
              [C[0] + rd / 2, -Y["disc_in"] + od["tongue"], C[1] + sw / 2]),
          box([C[0] - sw / 2, -Y["disc_out"] - od["tongue"], C[1] - rd / 2],
              [C[0] + sw / 2, -Y["disc_out"], C[1] + rd / 2])],
         note="Tongue across X forward (journal), across Z back (horn hub): torque only.")
    rh = xcd["horn_r"]
    screws = [cyl(C + 6.0 * np.array([math.cos(a), math.sin(a)]), -Y["horn_face"] - 1,
                  -Y["hub_in"] + 1, od["horn_screw_dia"] / 2)
              for a in (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4)]
    part("hornhub", "horn hub", "aluminium", "crank", [cyl(C, -Y["horn_face"], -Y["hub_in"], rh)],
         [box([C[0] - sw / 2, -Y["hub_in"] - sd, C[1] - rh - 1],
              [C[0] + sw / 2, -Y["hub_in"] + 1, C[1] + rh + 1])] + screws,
         "4 x M2 into the horn (PCD 12), slot across Z for the Oldham disc.")
    part("horn", "XC330 horn (mock)", "mock", "crank",
         [cyl(C, -Y["case_face"], -Y["horn_face"], rh)])
    lo_z = C[1] - xcd["shaft_from_end"]
    hw = xcd["width"] / 2
    part("servo", "XC330 (mock)", "mock", "fixed",
         [box([-hw, -Y["case_back"], lo_z], [hw, -Y["case_face"], lo_z + xcd["height"]])])
    cw = sv["cradle_wall"]
    holes = []
    for hx in (-sv["back_holes"][0] / 2, sv["back_holes"][0] / 2):
        for hz in (-sv["back_holes"][1] / 2, sv["back_holes"][1] / 2):
            zc = lo_z + xcd["height"] / 2 + hz
            holes.append(cyl([hx, zc], -Y["back_plate"] - 1, -Y["case_back"] + 1,
                             sv["back_hole_dia"] / 2))
    part("cradle", "servo cradle", "printed", "fixed",
         [box([hw, -Y["case_back"], sv["cradle_floor_z"]], [hw + cw, -Y["bh_out"], top_z]),
          box([-hw - cw, -Y["case_back"], sv["cradle_floor_z"]], [-hw, -Y["bh_out"], top_z]),
          box([-hw - cw, -Y["back_plate"], sv["back_plate_floor_z"]], [hw + cw, -Y["case_back"], top_z])],
         holes, "Holds the XC330 by its back face; the horn face is free. Hangs from the bridge.")
    part("bridge", "bridge", "aluminium", "fixed",
         [box([-fr["top_half_width"], -Y["back_plate"], top_z],
              [fr["top_half_width"], Y["bh_out"], top_z + fr["bridge"]])],
         note="Ties the bulkheads and the cradle; its top face is the mount to the bike.")
    fz = -(data["wheel_r"] + data["pivot_z"])
    part("floor", "floor (mock)", "floor", "fixed",
         [box([-130, -Y["back_plate"], fz - 2], [130, Y["rod"], fz])])

    for p in parts:
        p["bbox"] = bbox(p["add"])
    return {"Y": Y, "parts": parts, "C": C, "pin": pin, "J": J, "foot": foot, "top": top,
            "floor_z": fz, "poses": {f"{f:+.2f}": poses(lk, data["T"] * f, C, pin, J)
                                     for f in sorted(set(CHECK_POSES) | {p[2] for p in DIALOG_POSES})}}


def _ang(v):
    return math.degrees(math.atan2(v[1], v[0]))


def poses(lk, t: float, C, pin0, J0) -> dict:
    """Transforms for each group at crank travel `t`: rotation `a` [deg, right
    hand about +Y] about a point, then a translation. a = bearing(before) -
    bearing(after), in the X-Z plane (a +Y rotation carries X toward -Z)."""
    lk.reset()
    pR, pL = lk.pose(-1, t), lk.pose(1, t)
    pin = xz(pR["crank_tip"])
    JR, JL = xz(pR["joint"]), xz(pL["joint"])
    J0L = np.array([-J0[0], J0[1]])
    O = np.zeros(2)
    out = {"crank": {"p": list(C), "a": _ang(pin0 - C) - _ang(pin - C), "d": [0.0, 0.0]},
           "wR": {"p": [0.0, 0.0], "a": _ang(J0 - O) - _ang(JR - O), "d": [0.0, 0.0]},
           "wL": {"p": [0.0, 0.0], "a": _ang(J0L - O) - _ang(JL - O), "d": [0.0, 0.0]},
           "cR": {"p": list(pin0), "a": _ang(J0 - pin0) - _ang(JR - pin), "d": list(pin - pin0)},
           "cL": {"p": list(pin0), "a": _ang(J0L - pin0) - _ang(JL - pin), "d": list(pin - pin0)},
           "fixed": {"p": [0.0, 0.0], "a": 0.0, "d": [0.0, 0.0]}}
    out["_marks"] = {"pin": list(pin), "JR": list(JR), "JL": list(JL)}
    for g in out.values():
        for k in ("p", "d"):
            if k in g:
                g[k] = [float(v) for v in g[k]]
        if "a" in g:
            g["a"] = float(g["a"])
    return out


def apply(g: dict, p) -> np.ndarray:
    """The same transform in numpy, to check the convention before Onshape does."""
    a = math.radians(g["a"])
    v = np.asarray(p, float) - g["p"]
    rx, rz = v[0] * math.cos(a) + v[1] * math.sin(a), -v[0] * math.sin(a) + v[1] * math.cos(a)
    return np.array([rx, rz]) + g["p"] + g["d"]


def self_check(L: dict) -> None:
    """Every pose's transforms carry the rest pins onto the solved ones."""
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


# ----------------------------------------------------------- FeatureScript
def _fs(v) -> str:
    """A python value as a FeatureScript literal (numbers stay unitless)."""
    if isinstance(v, dict):
        return "{ " + ", ".join(f'"{k}" : {_fs(x)}' for k, x in v.items()) + " }"
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_fs(x) for x in v) + "]"
    if isinstance(v, str):
        return json.dumps(v)
    if isinstance(v, bool):
        return "true" if v else "false"
    t = format(float(v), ".10f").rstrip("0").rstrip(".")
    return "0" if t in ("-0", "") else t


def fs_data(L: dict) -> str:
    parts = [{"slug": p["slug"], "name": p["name"], "group": p["group"],
              "color": list(COLORS[p["mat"]]), "note": p["note"] or p["mat"],
              "add": p["add"], "cut": p["cut"]} for p in L["parts"]]
    poses = {k: {g: v for g, v in ps.items() if not g.startswith("_")}
             for k, ps in L["poses"].items()}
    return (f"export const WLM_PARTS = {_fs(parts)};\n\n"
            f"export const WLM_POSES = {_fs(poses)};\n")


FS = r'''FeatureScript %VERSION%;
import(path : "onshape/std/geometry.fs", version : "%VERSION%.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_wing_linkage_metal --push wing_metal_features
 *
 * Every primitive of every part is computed in aow_sim.cad_wing_linkage_metal
 * and carried here as WLM_PARTS; this file only builds them. Millimetres.
 * MODULE FRAME: origin on the wing rod at the mid-plane between the couplers,
 * +Y forward, +X right, +Z up. Poses: WLM_POSES, a rotation about +Y through
 * `p` by `a` degrees, then a shift `d`, per group.
 */

%DATA%

/** One primitive: a cylinder along Y, a prism from an (X, Z) outline, or a box. */
export function wlmPrim(context is Context, id is Id, q is map) returns Query
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
        fCuboid(context, id, { "corner1" : vector(q.lo[0], q.lo[1], q.lo[2]) * mm,
                               "corner2" : vector(q.hi[0], q.hi[1], q.hi[2]) * mm });
    }
    else if (q.k == "slot")
    {
        var sk = newSketchOnPlane(context, id + "sk", {
                "sketchPlane" : plane(vector(0, q.y0, 0) * mm, vector(0, 1, 0), vector(1, 0, 0)) });
        const P = function(v) { return vector(v[0], -v[1]) * mm; };
        skLineSegment(sk, "l1", { "start" : P(q.pts[0]), "end" : P(q.pts[1]) });
        skArc(sk, "ab", { "start" : P(q.pts[1]), "mid" : P(q.mids[0]), "end" : P(q.pts[2]) });
        skLineSegment(sk, "l2", { "start" : P(q.pts[2]), "end" : P(q.pts[3]) });
        skArc(sk, "aa", { "start" : P(q.pts[3]), "mid" : P(q.mids[1]), "end" : P(q.pts[0]) });
        skSolve(sk);
        opExtrude(context, id + "ext", {
                "entities"  : qSketchRegion(id + "sk"),
                "direction" : vector(0, 1, 0),
                "endBound"  : BoundingType.BLIND,
                "endDepth"  : (q.y1 - q.y0) * mm });
        opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(id + "sk", EntityType.BODY) });
    }
    else
    {
        // sketch plane normal +Y, x axis +X, so the sketch's second axis is -Z
        var sk = newSketchOnPlane(context, id + "sk", {
                "sketchPlane" : plane(vector(0, q.y0, 0) * mm, vector(0, 1, 0), vector(1, 0, 0)) });
        var pts = [];
        for (var p in q.pts)
            pts = append(pts, vector(p[0], -p[1]) * mm);
        // std has no polygon: one segment per edge, closed
        for (var i = 0; i < size(pts); i += 1)
            skLineSegment(sk, "s" ~ i, { "start" : pts[i], "end" : pts[(i + 1) % size(pts)] });
        skSolve(sk);
        opExtrude(context, id + "ext", {
                "entities"  : qSketchRegion(id + "sk"),
                "direction" : vector(0, 1, 0),
                "endBound"  : BoundingType.BLIND,
                "endDepth"  : (q.y1 - q.y0) * mm });
        opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(id + "sk", EntityType.BODY) });
    }
    return qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID);
}

/** One part: its primitives united, its cuts subtracted, named and coloured. */
export function wlmPart(context is Context, id is Id, p is map) returns Query
{
    var adds = [];
    for (var i = 0; i < size(p.add); i += 1)
        adds = append(adds, wlmPrim(context, id + ("a" ~ i), p.add[i]));
    if (size(adds) > 1)
        opBoolean(context, id + "uni", { "tools" : qUnion(adds),
                                         "operationType" : BooleanOperationType.UNION });
    // EVALUATED before the cutters exist: they are created under this same id,
    // so a lazy query would make every cutter its own target too.
    const body = qUnion(evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID)));
    if (size(p.cut) > 0)
    {
        var cuts = [];
        for (var i = 0; i < size(p.cut); i += 1)
            cuts = append(cuts, wlmPrim(context, id + ("c" ~ i), p.cut[i]));
        opBoolean(context, id + "sub", { "tools" : qUnion(cuts), "targets" : body,
                                         "operationType" : BooleanOperationType.SUBTRACTION });
    }
    const q = qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID);
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME, "value" : p.name });
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.APPEARANCE,
                           "value" : color(p.color[0], p.color[1], p.color[2]) });
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.DESCRIPTION, "value" : p.note });
    return q;
}

/** The transform that puts one group at one pose. */
export function wlmXf(g is map) returns Transform
{
    const mm = millimeter;
    return transform(vector(g.d[0], 0, g.d[1]) * mm)
         * rotationAround(line(vector(g.p[0], 0, g.p[1]) * mm, vector(0, 1, 0)), g.a * degree);
}

/** Build every part at rest; returns the bodies by group. */
export function wlmBuild(context is Context, id is Id, mocks is boolean, trace is boolean) returns map
{
    var groups = {};
    for (var p in WLM_PARTS)
    {
        if (!mocks && isIn(p.slug, ["servo", "horn", "floor", "panelR", "panelL"]))
            continue;
        if (trace)
            println("BUILD|" ~ p.slug);
        const q = wlmPart(context, id + p.slug, p);
        groups[p.group] = append(groups[p.group] == undefined ? [] : groups[p.group], q);
    }
    return groups;
}

/** Move every group to pose `key` (from rest), or back with `back`. */
export function wlmPose(context is Context, id is Id, groups is map, key is string, back is boolean)
{
    const ps = WLM_POSES[key];
    for (var g in ["crank", "cR", "cL", "wR", "wL"])
    {
        if (groups[g] == undefined)
            continue;
        const xf = wlmXf(ps[g]);
        opTransform(context, id + g, { "bodies" : qUnion(groups[g]),
                                       "transform" : back ? inverse(xf) : xf });
    }
}

%SPLIT%

export enum WingPose
{
%ENUM%
}

annotation { "Feature Type Name" : "AOW wing linkage metal",
             "Feature Type Description" : "Diamond swing linkage as a supported crankshaft, every pivot bushed, Oldham to the XC330; Y forward, origin on the wing rod" }
export const aowWingLinkageMetal = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Pose" }
        definition.pose is WingPose;
        annotation { "Name" : "Show mocks (servo, panels, floor)", "Default" : true }
        definition.mocks is boolean;
    }
    {
        const groups = wlmBuild(context, id + "build", definition.mocks, false);
        const keys = %KEYS%;
        const key = keys[definition.pose];
        if (key != "+0.00")
            wlmPose(context, id + "pose", groups, key, false);
        reportFeatureInfo(context, id, "%INFO%");
    });
'''


def build_fs(L: dict, data: dict, fs_version: str = "3044") -> str:
    enum = ",\n".join(f'    annotation {{ "Name" : "{label}" }} {name}'
                      for name, label, _f in DIALOG_POSES)
    keys = "{ " + ", ".join(f'WingPose.{name} : "{f:+.2f}"' for name, _l, f in DIALOG_POSES) + " }"
    lk = data["lk"]
    info = (f"Diamond x{data['scale']:g}: crank {lk.crank:.2f}, coupler {lk.coupler:.2f}, "
            f"rocker {lk.rocker:.2f}; crank axis {L['C'][1]:.2f} above the rod; "
            f"stroke +-{data['T']:.1f} deg")
    text = FS
    for k, v in {"%VERSION%": fs_version, "%DATA%": fs_data(L), "%SPLIT%": SPLIT_MARK,
                 "%ENUM%": enum, "%KEYS%": keys, "%INFO%": info}.items():
        text = text.replace(k, v)
    return text


# ------------------------------------------------------------------- check
def check_wrapper(fs: str, L: dict) -> str:
    """Build once at rest; print every body; then at every check pose move the
    four moving groups, print every interference of a moving body with
    anything, and where the pins' bodies landed; move back. ONE call."""
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
    const groups = wlmBuild(context, id, true, true);
    const all = evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID));
    for (var b in all)
    {{
        const bb = evBox3d(context, {{ "topology" : b, "tight" : true }});
        println("BODY|" ~ name(b) ~ "|" ~ toString(bb.minCorner[0] / mm) ~ "|" ~ toString(bb.minCorner[1] / mm)
                ~ "|" ~ toString(bb.minCorner[2] / mm) ~ "|" ~ toString(bb.maxCorner[0] / mm) ~ "|"
                ~ toString(bb.maxCorner[1] / mm) ~ "|" ~ toString(bb.maxCorner[2] / mm) ~ "|"
                ~ toString(evVolume(context, {{ "entities" : b }}) / (mm * mm * mm)));
    }}
    for (var p in WLM_PARTS)
        println("PART|" ~ p.name ~ "|" ~ toString(size(evaluateQuery(context,
                qBodyType(qCreatedBy(id + p.slug, EntityType.BODY), BodyType.SOLID)))));
    // at rest, every pair once
    for (var i = 0; i + 1 < size(all); i += 1)
        for (var c in evCollision(context, {{ "tools" : all[i], "targets" : qUnion(subArray(all, i + 1, size(all))) }}))
            println("COLL|rest|" ~ name(c.toolBody) ~ "|" ~ name(c.targetBody) ~ "|" ~ toString(c["type"]));
    // at each pose, each moving group against everything outside it
    const keys = {_fs(keys)};
    const marks = ["crankpin", "big end bushing R", "big end bushing L", "small end bushing R",
                   "small end bushing L", "rocker pin R", "rocker pin L"];
    for (var k = 0; k < size(keys); k += 1)
    {{
        const key = keys[k];
        wlmPose(context, id + ("go" ~ k), groups, key, false);
        for (var g in ["crank", "cR", "cL", "wR", "wL"])
        {{
            const mine = evaluateQuery(context, qUnion(groups[g]));
            var others = [];
            for (var b in all)
                if (!isIn(b, mine))
                    others = append(others, b);
            for (var c in evCollision(context, {{ "tools" : qUnion(mine), "targets" : qUnion(others) }}))
                println("COLL|" ~ key ~ "|" ~ name(c.toolBody) ~ "|" ~ name(c.targetBody) ~ "|" ~ toString(c["type"]));
        }}
        for (var b in all)
            if (isIn(name(b), marks))
            {{
                const bb = evBox3d(context, {{ "topology" : b, "tight" : true }});
                println("MARK|" ~ key ~ "|" ~ name(b) ~ "|" ~ toString((bb.minCorner[0] + bb.maxCorner[0]) / 2 / mm)
                        ~ "|" ~ toString((bb.minCorner[2] + bb.maxCorner[2]) / 2 / mm));
            }}
        wlmPose(context, id + ("bk" ~ k), groups, key, true);
    }}
    return "ran to completion";
}}
"""


def judge(console: str, L: dict, data: dict) -> bool:
    rows = [l.split("|") for l in console.splitlines()]
    ok = True
    by_name = {p["name"]: p for p in L["parts"]}
    vols = {}
    bodies = [r for r in rows if r[0] == "BODY"]
    for r in bodies:
        p = by_name.get(r[1])
        if p is None:
            print(f"  stray body {r[1]}  FAIL")
            ok = False
            continue
        got = [float(v) for v in r[2:8]]
        want = p["bbox"][0] + p["bbox"][1]
        err = max(abs(a - b) for a, b in zip(got, want))
        vols[r[1]] = float(r[8])
        if err > 1e-3:
            ok = False
            print(f"  {r[1]:24} bbox off by {err:.4f}  FAIL  got {np.round(got, 3)} want {np.round(want, 3)}")
    print(f"  {len(bodies)} bodies, every bounding box as computed: {'ok' if ok else 'NO'}")
    for r in rows:
        if r[0] == "PART" and int(r[2]) != 1:
            ok = False
            print(f"  part {r[1]} is {r[2]} bodies  FAIL")
    for p in L["parts"]:
        if p["twin"]:
            a, b = vols.get(by_name_slug(L, p["twin"])), vols.get(p["name"])
            same = None not in (a, b) and abs(a - b) < 1e-3
            ok &= same
            print(f"  {p['name']:26} same part as its twin ({a}, {b}): {'ok' if same else 'FAIL'}")
    colls = [r for r in rows if r[0] == "COLL" and "ABUT" not in r[4]]
    pairs = sorted({(r[1], *sorted(r[2:4])) for r in colls})
    ok &= not pairs
    print(f"  interference over {len(CHECK_POSES)} poses: {len(pairs)}")
    for r in pairs[:60]:
        print(f"    {r[0]}  {r[1]}  x  {r[2]}")
    worst = 0.0
    for r in rows:
        if r[0] != "MARK":
            continue
        mk = L["poses"][r[1]]["_marks"]
        want = {"crankpin": mk["pin"], "big end bushing R": mk["pin"], "big end bushing L": mk["pin"],
                "small end bushing R": mk["JR"], "rocker pin R": mk["JR"],
                "small end bushing L": mk["JL"], "rocker pin L": mk["JL"]}[r[2]]
        worst = max(worst, abs(float(r[3]) - want[0]), abs(float(r[4]) - want[1]))
    good = worst < 1e-3
    ok &= good
    print(f"  pins where the solver puts them, every pose: worst {worst:.2e} mm  {'ok' if good else 'FAIL'}")
    dens = {k: v for k, v in data["s"]["materials"].items()}
    mass = {}
    for p in L["parts"]:
        if p["mat"] in dens and p["name"] in vols:
            mass[p["mat"]] = mass.get(p["mat"], 0.0) + vols[p["name"]] * dens[p["mat"]] / 1000
    print("  mass: " + ", ".join(f"{k} {v:.1f} g" for k, v in mass.items())
          + f"; total {sum(mass.values()):.1f} g (no panels, no servo, no screws)")
    return ok


def by_name_slug(L: dict, slug: str) -> str:
    return next(p["name"] for p in L["parts"] if p["slug"] == slug)


def check(text: str, L: dict, data: dict, target: str | None) -> bool:
    """ONE billable call."""
    from . import onshape

    url = onshape.resolve(target, "check")
    reply = onshape.eval_featurescript(check_wrapper(text, L), url)
    for line in onshape.notice_lines(reply):
        print(f"  {line}")
    console = reply.get("console") or ""
    Path("traces").mkdir(exist_ok=True)
    Path("traces/wing_linkage_metal_check.txt").write_text(console)
    # The whole reply too: a failed eval still bills, and its notices are the
    # only record of why -- lost once already to a filter on the terminal.
    Path("traces/wing_linkage_metal_check.json").write_text(json.dumps(reply, indent=1, default=str))
    if any(n["message"]["level"] == "ERROR" for n in reply.get("notices", [])):
        print(console[-3000:])
        print(onshape.budget_line())
        return False
    ok = judge(console, L, data)
    print(onshape.budget_line())
    return ok


# ------------------------------------------------------------------ report
def report(L: dict, data: dict) -> str:
    Y, lk, s = L["Y"], data["lk"], data["s"]
    out = [f"  diamond x{data['scale']:g}: crank {lk.crank:.2f}  coupler {lk.coupler:.2f}  "
           f"rocker {lk.rocker:.2f}  stroke +-{data['T']:.1f} deg",
           f"  crank axis {L['C'][1]:.2f} above the rod; rod {-L['floor_z']:.1f} above the floor; "
           f"rest crankpin {np.round(L['pin'], 2)}, rocker pin R {np.round(L['J'], 2)}",
           "  along Y (magnitudes, each side of the mid-plane):"]
    for k in ("cpl_in", "cpl_out", "web_in", "web_out", "hz_out", "bh_in", "bh_out", "kn_in",
              "kn_out", "rod"):
        out.append(f"    {Y[k]:7.2f}  {k}")
    out.append(f"  behind the rear bulkhead: journal end {Y['jn_end']:.2f}, Oldham disc "
               f"{Y['disc_in']:.2f}-{Y['disc_out']:.2f}, horn face {Y['horn_face']:.2f}, "
               f"case {Y['case_face']:.2f}-{Y['case_back']:.2f}, back plate {Y['back_plate']:.2f}")
    # Loads: the sim's diamond pin peaks (righting-linkage-margin.md) at 1/scale.
    # CALCULATED, not measured, and the fall figure assumes it all went through
    # the linkage -- with the knuckles most of it should go into the rod instead.
    k = data["scale"]
    sh, bu, st = s["shafts"], s["bushings"], s["stack"]
    out.append("  bearing pressure, F / (d L), CALCULATED from the sim's 1.0x pin peaks / scale:")
    for what, d, length in (("crankpin in the big end", sh["crankpin_dia"], st["coupler"]),
                            ("rocker pin in the small end", sh["rocker_pin_dia"], st["coupler"]),
                            ("crank journal (one side)", sh["journal_dia"], st["bulkhead"]),
                            ("rocker hub on the rod", sh["rod_dia"], Y["hz_out"] - Y["web_in"])):
        out.append(f"    {what:30} {72 / k / (d * length):5.2f} MPa lifting (72 N / {k:g}), "
                   f"{162 / k / (d * length):5.2f} MPa in a fall (162 N / {k:g})")
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
        print(check_wrapper(text, L))
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
