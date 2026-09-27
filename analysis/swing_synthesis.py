"""Swing linkage for margin: panel poses in, link lengths out, scored by current.

Step 3 of docs/plans/righting-linkage-margin.md. The user's framing: fix the
panel's poses -- REST, and FLAT on the floor at the end of the stroke -- keep
MINIMUM STOWED (the rising wing's innermost excursion) as a bound, freeze the
hinge, and let the links be whatever gets there with the most margin.

HOW THE POSES ARE HELD. The panel is fixed in the HINGE frame: the same line,
bottom and length at rest as the reference config, on every candidate. The
four-bar's lengths, the angle between the crank arms and the servo height are
searched; for each, the panel's bearing and offset off the rocker
(`wing_angle_from_rocker`, `wing_norm_offset`) are DERIVED so the panel lands
back on that line -- the coupler attach point is a free point on the wing, not
the panel's anchor. Then
  - REST is met by construction,
  - FLAT is where the stroke ends (`critical_angles(...).command` = the crank
    travel that lays the panel horizontal, if it comes before the toggle;
    `hand-off roll` in the feasibility table checks it),
  - MINIMUM STOWED is `limits.far_inboard_deg`, a bound, as before.
This is the same family three-position synthesis (Freudenstein) would give --
every four-bar through those poses -- parametrised by lengths instead of by
precision-point crank angles, so `swing_linkage.py`'s kinematics, constraints,
builder and CAD path take the result unchanged.

HOW IT IS SCORED. Because the panel and hinge are fixed, the load at the hinge
is a function of the WING ANGLE only, identical for every candidate: one table,
built once from `swing_linkage.rest_on_panel` (checked against the sim in
righting_ideal_profile.py). A candidate is then pure kinematics:
    motor, moving  = (1 + c_run) load(wing) |r| + f_run   over the stroke
    motor, at rest = (1 + c_st)  load(wing0) |r0| + f_st  breakaway, stroke start
    need           = the MOVING peak, as Goal Current counts (measured law)
minimised (ties to the most compact linkage) subject to breakaway fitting
under what the servo gives at 9.9 V, the study's feasibility rows (torque row
dropped), servo
travel <= --max-travel (the one-turn window, both sides sharing it), |r| <=
--r-max through the stroke (a fast wing at the end throws the bike over), and a
quasi-steady stroke time at 9.9 V no slower than the reference's.

`--pivot-x` moves both hinges (0 = one shared rod); the panel moves with its
hinge and the load table is rebuilt for it, unless `--keep-panel`.

    python analysis/swing_synthesis.py                      # couplers in one plane
    python analysis/swing_synthesis.py --stagger-couplers --tag _staggered
    python analysis/swing_synthesis.py --stagger-couplers --end-toggle --tag _staggered_toggle
    python analysis/swing_synthesis.py --pivot-x 0 --keep-panel --stagger-couplers \
        --stagger-rockers --one-pin --tag _shared           # the diamond: one pin, one rod
    python analysis/swing_synthesis.py ... --save <candidate>.yaml    # for the sim sweep

Results, 2026-09-27 (`_smaller`'s panel, hinge and bounds; flat to 1 deg;
rising wing no further out than rest + 2 deg), planar / sim sweep at 9.9 V:

    reference `_smaller`               754 / 660 counts
    couplers in one plane              750 /  --    coupler vs coupler binds
    staggered, self-locking end        656 / 580 (620 quasi-static)
    staggered, free end                614 / 520
The planar scorer's breakaway is from rest at crank 0; the sim's fallen bike
back-drives the crank first, to a lower ratio, so the sim needs less. The
ranking agrees."""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import swing_linkage as sl  # noqa: E402
from aow_sim.build_model import SWING_LINKAGE_CFG, load_params  # noqa: E402

SEARCHED = ("crank_length", "coupler_length", "rocker_length",
            "angle_between_cranks", "servo_offset")
BOUNDS = {"crank_length": (10.0, 60.0), "coupler_length": (10.0, 160.0),
          "rocker_length": (10.0, 120.0), "angle_between_cranks": (0.0, 170.0),
          "servo_offset": (10.0, 120.0)}
"""`--wide` only. By default the reference config's own `bounds:` block is
searched -- `_smaller` caps the servo 50 mm above the axle -- because that is
the envelope someone decided was buildable."""
SUPPLY = 9.9 / 12.0
COMPACT_NM_PER_MM = 2e-5
CURRENT_LIMIT_A = 0.910
PLOTS = Path(__file__).resolve().parent / "plots"


# -- the servo, from bike_params ------------------------------------------

def servo_consts(params: dict) -> dict:
    s = params["servos"]["xc330_t181"]
    return {"k": float(s["current_torque_gain"]), "i0": float(s["current_deadband"]),
            "ts": float(s["stall_torque"]), "w0": float(s["no_load_rpm"]) * np.pi / 30,
            "c_run": float(s["friction_running_fraction"]),
            "f_run": float(s["friction_running_nm"]),
            "c_st": float(s["friction_static_fraction"]),
            "f_st": float(s["friction_static_nm"])}


def ss_avail(sc: dict) -> float:
    """Motor torque the servo can give at 9.9 V under the Current Limit."""
    return min(sc["k"] * (CURRENT_LIMIT_A - sc["i0"]), sc["ts"] * SUPPLY)


def counts(sc: dict, tau: float) -> float:
    return (sc["i0"] + tau / sc["k"]) * 1000.0


def margin(sc: dict, tau: float) -> float:
    return ss_avail(sc) / tau


# -- the panel, held in the hinge frame -----------------------------------

def reference_panel(ref_cfg: dict, pivot_x: float | None, keep_panel: bool = False,
                    panel_out: float = 0.0):
    """(foot, top, hinge) of the right panel at rest, sketch mm. With
    `pivot_x`, the hinge moves to -pivot_x and the panel moves WITH it --
    unless `keep_panel`, where the panel stays put in the chassis and only the
    hinge moves (a shared rod under the same wing)."""
    lk = sl.SwingLinkage(copy.deepcopy(ref_cfg))
    pz = lk.pose(-1, 0.0)
    foot, top, hinge = pz["foot"].copy(), pz["top"].copy(), pz["pivot"].copy()
    if pivot_x is not None:
        shift = np.array([-pivot_x - hinge[0], 0.0])
        hinge = hinge + shift
        if not keep_panel:
            foot, top = foot + shift, top + shift
    # `panel_out`: the stowed panel further OUT on its own side [mm], hinge
    # unmoved. Props the fallen bike higher -- less to lift, a wider bike.
    foot, top = foot + [-panel_out, 0.0], top + [-panel_out, 0.0]
    return foot, top, hinge


def load_table(foot, top, hinge, wheel_radius: float, hi_deg: float = 120.0):
    """Hinge load vs wing angle (rest = 0) for this panel and hinge."""
    wd = np.arange(-10.0, hi_deg, 0.1)
    out = np.full_like(wd, np.nan)
    for i, w in enumerate(wd):
        f = sl._rot(foot - hinge, w) + hinge
        t = sl._rot(top - hinge, w) + hinge
        st = sl.rest_on_panel(f, t, hinge, wheel_radius)
        if st is not None:
            out[i] = st["load"]
    return wd, out


def candidate(ref_cfg: dict, x, panel, pivot_x: float | None):
    """A SwingLinkage with these lengths and the panel put back on its line.
    Raises ValueError if the panel cannot sit there (offset below the
    coupler's width, or it does not close)."""
    foot, top, hinge = panel
    cfg = copy.deepcopy(ref_cfg)
    m = cfg["mechanism"]
    for k, v in zip(SEARCHED, x):
        m[k] = float(v)
    if pivot_x is not None:
        m["wing_pivot_x"] = float(pivot_x)
    m["wing_angle_mode"] = "fixed"
    # A throwaway to find the rest joint: the panel does not affect it.
    m["wing_angle_from_rocker"] = 0.0
    m["wing_norm_offset"] = 1e3
    J = sl.SwingLinkage(copy.deepcopy(cfg)).rest_joint(-1)
    P = hinge
    w = (top - foot) / np.linalg.norm(top - foot)
    r = J - P
    psi = np.degrees(np.arctan2(r[1], r[0]))
    wa = np.degrees(np.arctan2(w[1], w[0]))
    # side -1: panel bearing = rocker bearing - wing_angle_from_rocker, and
    # the origin J - norm * n lies on the panel line (n = (w_y, -w_x)).
    m["wing_angle_from_rocker"] = float((psi - wa + 180.0) % 360.0 - 180.0)
    m["wing_norm_offset"] = float(-(w[0] * (J - foot)[1] - w[1] * (J - foot)[0]))
    lk = sl.SwingLinkage(cfg)
    pz = lk.pose(-1, 0.0)
    if pz is None or max(np.abs(pz["foot"] - foot).max(),
                         np.abs(pz["top"] - top).max()) > 0.05:
        raise ValueError("panel did not land on its line")
    return lk


# -- scoring ----------------------------------------------------------------

def far_off_vertical(lk, travel: float) -> float:
    """The RISING (left) panel's lean from vertical at `travel` [deg], + =
    top outboard. Same sign as `rest_wing_deg`."""
    pz = lk.pose(1, travel)
    v = pz["top"] - pz["foot"]
    return float(np.degrees(np.arctan2(v[0], v[1])))


def stroke(lk, table, sc: dict, step: float = 1.0) -> dict | None:
    """Motor torque through the commanded stroke, and what it adds up to."""
    T = sl.critical_angles(lk).command
    if T <= 0.0:
        return None
    ts = np.linspace(0.0, T, max(int(np.ceil(T / step)), 2) + 1)
    wd, r = [], []
    for t in ts:
        pz = lk.pose(-1, float(t))
        g = sl.ratio_at(lk, -1, float(t), pz) if pz is not None else None
        if g is None:
            return None
        wd.append(pz["wing_deg"])
        r.append(abs(g))
    wd, r = np.asarray(wd), np.asarray(r)
    L = np.interp(wd, *table)
    if not np.isfinite(L).all():
        return None
    L = np.maximum(L, 0.0)               # falling forward needs no servo
    run = (1 + sc["c_run"]) * L * r + sc["f_run"]
    brk = (1 + sc["c_st"]) * L[0] * r[0] + sc["f_st"]
    # The MOVING peak is the score. Breakaway from rest at crank 0 is a limit
    # (`violations`), not the score: taken there it is conservative -- in the
    # sim the fallen bike back-drives the crank a few degrees first, to a
    # lower ratio -- and as the score it set every staggered design to the
    # same ~0.50 N m, hiding the moving peaks that the sim does rank by
    # (free end 518 counts, shared rod 585, both "620" here).
    need = float(run.max())
    room = SUPPLY - run / sc["ts"]
    time = (float(np.trapezoid(1.0 / (sc["w0"] * room), np.radians(ts)))
            if (room > 0).all() else np.inf)
    far = [far_off_vertical(lk, float(t)) for t in ts[:: max(len(ts) // 12, 1)]]
    far.append(far_off_vertical(lk, float(T)))
    return {"T": float(T), "t": ts, "wing": wd, "r": r, "motor": run,
            "far_out": float(max(far) - far_off_vertical(lk, 0.0)),
            "run": float(run.max()), "brk": float(brk), "need": need,
            "counts": counts(sc, need), "counts_brk": counts(sc, brk),
            "margin": margin(sc, need),
            "time": time, "r_max": float(r.max())}


def violations(lk, cfg, st, lim: dict) -> dict:
    """Every hard constraint's shortfall, 0 when met, in its own unit."""
    b = sl._budgets(lk)._replace(torque=1e9, handoff=lim["handoff"])
    if lim.get("trans") is not None:
        b = b._replace(trans_min=lim["trans"])
    out = {}
    for name, val, op, limit, unit, ok in sl.feasibility(lk, cfg, b):
        if ok is None or name == "peak servo torque":
            continue
        out[name] = 0.0 if ok else abs(val - limit)
    out["servo travel"] = max(0.0, st["T"] - lim["max_travel"])
    out["breakaway"] = max(0.0, st["brk"] - lim["brk_max"])
    # The rising wing may dip inboard (`far wing outboard` bounds that) but
    # not swing back OUT past its rest attitude. `_smaller` returns to exactly
    # rest.
    #
    # WHAT THIS RULES OUT, deliberately and not because it is bad: the co-
    # rotating pair's rockers are periodic in the crank, so a stroke that
    # approaches a half turn carries the rising wing through its inboard limit
    # and back OUT. Unbounded, the search goes straight there (540 counts at
    # 162 deg, the rising wing ending 69 deg off vertical, 17 mm off the
    # floor): one stroke ends with BOTH wings out -- a two-wing stance, and
    # further round a sequential deploy of one wing then the other. That is
    # its own class of mechanism (the user has seen it), with its own
    # questions -- the far wing meeting the floor near upright, whether it
    # braces or snags -- so it wants its own study, not a quiet win here.
    # `--far-out 180` lifts the bound.
    out["rising wing out"] = max(0.0, st["far_out"] - lim["far_out"])
    # The deployed pose SELF-LOCKS only if the stroke ends at the extended
    # toggle (ratio -> 0): a brace load then cannot back-drive the servo.
    # `_smaller` ends there by construction (flat_deploy).
    if lim["end_toggle"]:
        out["ends at toggle"] = max(0.0, st["r"][-1] - 0.05)
    out["ratio"] = max(0.0, st["r_max"] - lim["r_max"])
    out["stroke time"] = max(0.0, st["time"] - lim["time"]) if np.isfinite(st["time"]) else 10.0
    return out


_W = {"servo travel": 0.1, "ratio": 10.0, "stroke time": 10.0, "breakaway": 10.0,
      "rising wing out": 0.1, "ends at toggle": 10.0}


def objective(x, ref_cfg, panel, pivot_x, table, sc, lim):
    """Two-stage, as swing_linkage._objective: 100 + weighted shortfall while
    anything is violated, else the motor torque needed [N m]."""
    try:
        lk = candidate(ref_cfg, x, panel, pivot_x)
    except Exception:
        return 1e4
    try:
        if sl.assembly_residual(lk) > 0.0:
            return 1e3 + sl.assembly_residual(lk)
        st = stroke(lk, table, sc)
        if st is None:
            return 1e3
        v = violations(lk, lk.cfg, st, lim)
    except Exception:
        return 1e3
    bad = sum(_W.get(k, 0.1) * s for k, s in v.items())
    if bad > 1e-9:
        return 100.0 + bad
    # Ties to the COMPACT linkage: the score is scale-free along a whole
    # family (seeds put the same shared-rod design's servo at 20, 26 or 41 mm
    # above the axle), so without this the size is an accident of the seed.
    # 2e-5 N m per mm: 100 mm of links and servo height ~ 2.4 counts.
    m = lk.cfg["mechanism"]
    return st["need"] + COMPACT_NM_PER_MM * sum(float(m[k]) for k in (
        "crank_length", "coupler_length", "rocker_length", "servo_offset"))


# -- reporting ----------------------------------------------------------------

def summary(name: str, lk, st) -> str:
    m = lk.cfg["mechanism"]
    lens = " ".join(f"{k.split('_')[0]} {m[k]:.1f}" for k in SEARCHED)
    return (f"{name:>10}  counts {st['counts']:4.0f} moving ({st['run']:.3f} N m; "
            f"breakaway {st['counts_brk']:.0f})  margin@9.9V {st['margin']:.2f}x  "
            f"time@9.9V {st['time']:.2f} s  travel {st['T']:.0f} deg  "
            f"r {st['r'].min():.2f}..{st['r_max']:.2f} (end {st['r'][-1]:.2f})  rising wing +{st['far_out']:.0f} "
            f"deg past rest\n{'':>12}{lens}  "
            f"wing_from_rocker {m['wing_angle_from_rocker']:.1f}  "
            f"norm_offset {m['wing_norm_offset']:.1f}")


def plot(ref, best, table, sc, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(16, 5))
    for (name, lk, st), col in ((ref, "0.3"), (best, "C3")):
        ax[0].plot(st["t"] / st["T"], st["r"], color=col, label=name)
        ax[1].plot(st["t"] / st["T"], st["motor"], color=col,
                   label=f"{name}: {st['counts']:.0f} counts")
        ax[1].plot([0], [st["brk"]], "v", color=col)
    lo_w, hi_w = best[2]["wing"][0], best[2]["wing"][-1]
    sel = (table[0] >= lo_w) & (table[0] <= hi_w)
    W = np.trapezoid(table[1][sel], np.radians(table[0][sel]))
    flat = (1 + sc["c_run"]) * W / np.radians(best[2]["T"]) + sc["f_run"]
    ax[1].axhline(flat, color="C3", ls=":", label=f"flat over the same travel: "
                  f"{counts(sc, flat):.0f}")
    ax[0].set_ylabel("d(wing)/d(crank)")
    ax[1].set_ylabel("motor torque [N m]  (v = breakaway)")
    for a in ax[:2]:
        a.set_xlabel("fraction of the stroke")
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    a = ax[2]
    for (name, lk, st), col, ls in ((ref, "0.6", "--"), (best, "C3", "-")):
        for t in (0.0, st["T"]):
            for side in (-1, 1):
                pz = lk.pose(side, t)
                if pz is None:
                    continue
                pts = [lk.shaft, pz["crank_tip"], pz["joint"], pz["pivot"]]
                a.plot(*np.array(pts).T, ls, color=col, lw=1.2 if t else 2.0)
                a.plot(*np.array([pz["foot"], pz["top"]]).T, ls, color=col, lw=4,
                       alpha=0.5 if t else 1.0)
    a.axhline(-best[1].wheel_radius, color="k", lw=0.8)
    a.set_aspect("equal")
    a.set_title("rest (thick) and end of stroke; grey dashed = reference")
    a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"wrote {out.relative_to(Path.cwd()) if out.is_relative_to(Path.cwd()) else out}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, default=SWING_LINKAGE_CFG,
                    help="reference: its panel, hinge, bike and limits are kept")
    ap.add_argument("--pivot-x", type=float, default=None,
                    help="move both hinges to +-this [mm]; 0 = one shared rod")
    ap.add_argument("--max-travel", type=float, default=175.0,
                    help="servo travel per side [deg]; both share one turn")
    ap.add_argument("--r-max", type=float, default=1.0,
                    help="cap on d(wing)/d(crank) through the stroke")
    ap.add_argument("--handoff", type=float, default=1.0,
                    help="how far from flat the stroke may end [deg]; the "
                         "config's hand-off window (6) lets it stop short")
    ap.add_argument("--far-out", type=float, default=2.0,
                    help="how far past its REST attitude the rising wing may "
                         "swing back out by the end of the stroke [deg]")
    ap.add_argument("--end-toggle", action="store_true",
                    help="require the stroke to end at the extended toggle "
                         "(ratio <= 0.05), so the deployed wing self-locks")
    ap.add_argument("--stagger-couplers", action="store_true",
                    help="put the two couplers in SEPARATE planes (couplerR 1, "
                         "couplerL 3), so they may pass each other; per type "
                         "they share one, and that is what binds by default")
    ap.add_argument("--keep-panel", action="store_true",
                    help="with --pivot-x: the panel stays where it is and only "
                         "the hinge moves (otherwise the panel moves with it)")
    ap.add_argument("--stagger-rockers", action="store_true",
                    help="the two wings' rockers in SEPARATE planes (rockerR 2, "
                         "rockerL 4) -- side by side on one rod")
    ap.add_argument("--panel-out", type=float, default=0.0,
                    help="move the stowed panel this much further out [mm], "
                         "hinge unmoved")
    ap.add_argument("--min-transmission", type=float, default=None,
                    help="override the config's transmission-angle floor [deg]")
    ap.add_argument("--one-pin", action="store_true",
                    help="both couplers on ONE crank pin (angle_between_cranks "
                         "0). With --pivot-x 0 the links form a diamond: one "
                         "pin, two couplers, two wings, one rod")
    ap.add_argument("--wide", action="store_true",
                    help="search this file's BOUNDS instead of the config's "
                         "`bounds:` block (the buildable envelope)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--popsize", type=int, default=20)
    ap.add_argument("--maxiter", type=int, default=150)
    ap.add_argument("--workers", type=int, default=-1)
    ap.add_argument("--save", type=Path, default=None)
    ap.add_argument("--tag", default="", help="suffix for the figure's name")
    args = ap.parse_args()

    from scipy.optimize import differential_evolution
    ref_cfg = yaml.safe_load(args.config.read_text())
    if args.stagger_couplers:
        cl = ref_cfg.setdefault("clearance", {})
        planes = {k: v for k, v in (cl.get("planes") or {}).items() if k != "coupler"}
        cl["planes"] = {**planes, "couplerR": 1, "couplerL": 3}
    if args.stagger_rockers:
        cl = ref_cfg.setdefault("clearance", {})
        planes = {k: v for k, v in (cl.get("planes") or {}).items() if k != "rocker"}
        cl["planes"] = {**planes, "rockerR": 2, "rockerL": 4}
    sc = servo_consts(load_params())
    ref_lk = sl.SwingLinkage(copy.deepcopy(ref_cfg))
    panel = reference_panel(ref_cfg, args.pivot_x, args.keep_panel, args.panel_out)
    table = load_table(*panel, ref_lk.wheel_radius)
    # The reference, scored the same way (its own hinge and table).
    ref_table = (table if args.pivot_x is None
                 else load_table(*reference_panel(ref_cfg, None), ref_lk.wheel_radius))
    ref_st = stroke(ref_lk, ref_table, sc)
    print(summary("reference", ref_lk, ref_st))
    lim = {"max_travel": args.max_travel, "r_max": args.r_max,
           "brk_max": ss_avail(sc),
           "time": ref_st["time"], "handoff": args.handoff,
           "far_out": args.far_out, "end_toggle": args.end_toggle,
           "trans": args.min_transmission}

    cb = {} if args.wide else (ref_cfg.get("bounds") or {})
    bounds = [tuple(cb.get(k, BOUNDS[k])) for k in SEARCHED]
    if args.one_pin:          # DE wants lo < hi; 1e-6 deg is one pin
        bounds[SEARCHED.index("angle_between_cranks")] = (0.0, 1e-6)
    print("bounds: " + ", ".join(f"{k} {lo:g}..{hi:g}" for k, (lo, hi) in zip(SEARCHED, bounds)))
    r = differential_evolution(
        objective, bounds, args=(ref_cfg, panel, args.pivot_x, table, sc, lim),
        seed=args.seed, popsize=args.popsize, maxiter=args.maxiter, tol=1e-7,
        workers=args.workers, updating="deferred" if args.workers != 1 else "immediate",
        polish=False)
    best_lk = candidate(ref_cfg, r.x, panel, args.pivot_x)
    best_st = stroke(best_lk, table, sc)
    print(summary("best", best_lk, best_st))
    print(f"objective {r.fun:.4f} after {r.nit} generations, {r.nfev} evaluations: "
          f"{r.message}")
    print()
    v = violations(best_lk, best_lk.cfg, best_st, lim)
    sl.print_feasibility(best_lk, best_lk.cfg, sl._budgets(best_lk)._replace(
        torque=1e9, handoff=args.handoff, **({} if args.min_transmission is None
                                              else {"trans_min": args.min_transmission})))
    for k in [k for k in v if k in ("servo travel", "ratio", "stroke time",
                                     "rising wing out", "ends at toggle")]:
        print(f"     {'PASS' if v[k] <= 0 else 'FAIL'} {k:<24} shortfall {v[k]:.3f}")
    plot(("reference", ref_lk, ref_st), ("best", best_lk, best_st), table, sc,
         PLOTS / f"swing_synthesis{args.tag}.png")
    if args.save is not None:
        out = copy.deepcopy(best_lk.cfg)
        out.setdefault("stroke", {})["crank_travel_deg"] = round(best_st["T"], 2)
        argv, skip = [], False
        for x in sys.argv[1:]:                  # the command, minus where it saved
            if skip:
                skip = False
            elif x == "--save":
                skip = True
            elif not x.startswith("--save="):
                argv.append(x)
        args.save.write_text(
            f"# From: python analysis/swing_synthesis.py {' '.join(argv)}\n"
            f"# {best_st['counts']:.0f} counts at 9.9 V on the planar scorer "
            f"(reference {ref_st['counts']:.0f}), margin {best_st['margin']:.2f}x, "
            f"travel {best_st['T']:.0f} deg.\n"
            f"# Sim check: python analysis/righting_current_sweep.py --config "
            f"{args.save} --supply 9.9\n"
            + yaml.safe_dump(out, sort_keys=False))
        print(f"wrote {args.save}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
