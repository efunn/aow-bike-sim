"""What is the best any transmission could do for the swing wing's righting stroke?

Step 1 of docs/plans/righting-linkage-margin.md. It is MECHANISM-FREE in one
precise sense: the panel, its hinge and the bike are held as they are in
`build_model.SWING_LINKAGE_CFG`, and only the transmission between the servo and
the wing hinge -- the four-bar's ratio r = d(wing)/d(crank) -- is free.

1. The LOAD. Run the stroke quasi-statically in sim (as
   `righting_current_sweep.py --slew-dps` does) and record the torque the
   coupler puts on each wing's hinge: the loop-closing equality's force
   projected onto that hinge. That is what the bike presents to ANY
   transmission at that wing angle, for this panel. The rising wing's share
   is printed, and it is negligible here.
2. Check that the load is the whole story. Virtual work through the recorded
   ratio plus the gearbox's RUNNING friction line,
       tau_motor = (1 + c) * tau_wing * r + f0,
   must reproduce the servo model's own motor torque. If it does not, a
   transmission designed from the load curve would miss.
3. The FRONT. Over a crank travel THETA the wing work W is fixed, so the
   lowest possible peak is the FLAT profile, tau_out = W / THETA, reached by
   r(wing) = tau_out / tau_wing(wing). Printed per travel: the motor torque,
   the Goal Current, the margin under the Current Limit, and the stroke time.

Stroke time is QUASI-STEADY: at each crank angle the motor runs where its line
meets the required torque, w = w0 (s - tau / ts), and inertia is ignored.
Checked against stepped runs at 910 counts (2026-09-26): 0.42 s here vs 0.53 s
in sim at 12 V; 0.73 vs 0.87 s at 9.9 V. That is 20-25 % fast, but it ranks
designs the same way; step 2's sweep is the real number. A flat torque at fixed
work is also the quasi-steady minimum-time profile (Jensen: 1 / (s - tau / ts)
is convex), so margin and speed want the same SHAPE and differ only in travel.

`--r-max` caps the ratio. The flat profile would race the wing through the
unloaded end of the stroke, and a wing arriving fast throws the bike over the
far side; the four-bar's toggle is what brakes it now. The cap is a stand-in
for that until step 2 scores real linkages dynamically.

NOT modelled: breakaway. The stroke starts from rest with the bike's weight
already on the panel, so its first instant needs the STATIC line (c 0.51, not
0.15) -- 1.31x the running torque at the same load. `breakaway` is printed for
the as-built design; a flat profile needs its first few degrees geared ~24 %
lower to match, which costs negligible travel and is not included.

    python analysis/righting_ideal_profile.py
    python analysis/righting_ideal_profile.py --r-max 1.5 --plot
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import righting_current_sweep as rcs  # noqa: E402
import swing_linkage  # noqa: E402
from aow_sim import gearbox_friction  # noqa: E402
from aow_sim.build_model import SWING_LINKAGE_CFG, load_params  # noqa: E402
from aow_sim.control.righting import roll_pitch  # noqa: E402
from aow_sim.righting_servo import AMPS_PER_COUNT, CURRENT_LIMIT  # noqa: E402

PLOT = Path(__file__).resolve().parent / "plots" / "righting_ideal_profile.png"
TRAVELS_DEG = (90, 120, 132, 150, 180, 220, 270, 360, 540)


def hinge_torque(m, d, dof: int, eq: int) -> float:
    """Generalized force the equality `eq` puts on `dof`: J^T f over its rows."""
    rows = np.flatnonzero((d.efc_type[:d.nefc] == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
                          & (d.efc_id[:d.nefc] == eq))
    J = d.efc_J.reshape(-1, m.nv) if not mujoco.mj_isSparse(m) else None
    if J is None:
        raise RuntimeError("dense Jacobian assumed; this model is sparse")
    return float(J[rows, dof] @ d.efc_force[rows])


def stroke(params, side: float = 1.0, slew_dps: float = 10.0, seconds: float = 16.0,
           counts: int = CURRENT_LIMIT, supply: float = 1.0, settle_s: float = 2.0):
    """The quasi-static stroke, recorded. Returns arrays keyed by name."""
    dep = np.deg2rad(float(yaml.safe_load(SWING_LINKAGE_CFG.read_text())
                           ["stroke"]["crank_travel_deg"]))
    m, d0, srv, hooks = rcs.fallen(params, side, settle_s)
    # The goal sign that lifts, as the sweep picks it.
    best = {g: rcs.trial(params, side, g, CURRENT_LIMIT, 1.0, 4.0, 5.0,
                         (m, d0, srv, hooks))["min_roll"] for g in (dep, -dep)}
    goal = min(best, key=best.get)
    d = mujoco.MjData(m)
    d.qpos[:], d.qvel[:] = d0.qpos, d0.qvel
    mujoco.mj_forward(m, d)
    srv.set_supply(supply)
    srv.set_goal_current(counts)
    aid = m.actuator("swing").id
    crank = m.joint("swing_crank_joint")
    eqs = {}
    for i in range(m.neq):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SITE, m.eq_obj1id[i]) or ""
        for tag in ("right", "left"):
            if name == f"swing_coupler_{tag}_end":
                eqs[tag] = i
    wings = {t: m.joint(f"swing_wing_{t}_joint") for t in eqs}
    slew = np.deg2rad(slew_dps)
    rec = []

    def on(dd):
        dd.ctrl[aid] = float(np.clip(dd.time * slew * np.sign(goal), -abs(goal), abs(goal)))
        row = [dd.time, dd.qpos[crank.qposadr[0]], roll_pitch(dd.qpos[3:7])[0], srv.torque]
        for t in ("right", "left"):
            row += [dd.qpos[wings[t].qposadr[0]], hinge_torque(m, dd, wings[t].dofadr[0], eqs[t])]
        rec.append(row)
    rcs.step(m, d, hooks, seconds, on)
    a = np.asarray(rec)
    sgn = np.sign(goal)
    out = {"t": a[:, 0], "crank": sgn * a[:, 1], "roll": np.abs(a[:, 2]), "tau_m": a[:, 3]}
    # The deploying wing is the one that moves with the crank's sign.
    r_, l_ = sgn * a[:, 4], sgn * a[:, 6]
    near_right = abs(r_[-1] - r_[0]) > abs(l_[-1] - l_[0])
    out["wing"], out["tau_w"] = (r_, sgn * a[:, 5]) if near_right else (l_, sgn * a[:, 7])
    out["far"], out["tau_far"] = (l_, sgn * a[:, 7]) if near_right else (r_, sgn * a[:, 5])
    return out


def lift_window(rec, level_deg: float = 5.0) -> slice:
    """From the crank's first 0.5 deg of motion to the first time within
    `level_deg` of upright."""
    c = np.degrees(rec["crank"])
    k0 = int(np.argmax(np.abs(c - c[0]) > 0.5))
    k1 = int(np.argmax(rec["roll"] < level_deg))
    if k1 <= k0:
        raise RuntimeError("the stroke never reached level; raise --seconds or counts")
    return slice(k0, k1)


def flat_level(phi, L, travel: float, r_max: float | None) -> float | None:
    """Output torque of the flattest profile over `travel` [rad] of crank:
    r = min(tau / L, r_max), tau chosen so the crank travel is `travel`."""
    L = np.maximum(L, 1e-6)

    def crank_travel(tau):
        r = tau / L if r_max is None else np.minimum(tau / L, r_max)
        return float(np.trapezoid(1.0 / r, phi))
    if r_max is not None and np.trapezoid(np.full_like(phi, 1.0 / r_max), phi) >= travel:
        return None                   # the cap alone uses up the travel
    lo, hi = 1e-4, 10.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if crank_travel(mid) > travel else (lo, mid)
    return 0.5 * (lo + hi)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--slew-dps", type=float, default=10.0)
    ap.add_argument("--seconds", type=float, default=16.0)
    ap.add_argument("--level-deg", type=float, default=5.0)
    ap.add_argument("--r-max", type=float, default=None,
                    help="cap on d(wing)/d(crank) for the ideal profile")
    ap.add_argument("--plot", action="store_true", help=f"write {PLOT.name}")
    args = ap.parse_args()

    params = load_params()
    srv = params["servos"]["xc330_t181"]
    k, i0 = float(srv["current_torque_gain"]), float(srv["current_deadband"])
    ts = float(srv["stall_torque"])
    w0 = float(srv["no_load_rpm"]) * 2 * np.pi / 60
    fr = gearbox_friction.constants(params)
    c_run, f_run = fr["running_frac"], fr["running_nm"]
    c_st, f_st = fr["static_frac"], fr["static_nm"]
    cap_i = CURRENT_LIMIT * AMPS_PER_COUNT

    def counts(tau_m):
        return (i0 + tau_m / k) / AMPS_PER_COUNT

    def margin(tau_m, s):
        return min(k * (cap_i - i0), ts * s) / tau_m

    def qs_time(theta, tau_m, s):
        room = s - tau_m / ts
        return float(np.trapezoid(1.0 / (w0 * room), theta)) if (room > 0).all() else np.inf

    rec = stroke(params, slew_dps=args.slew_dps, seconds=args.seconds)
    win = lift_window(rec, args.level_deg)
    th, phi = rec["crank"][win], rec["wing"][win]
    L, tm, roll = rec["tau_w"][win], rec["tau_m"][win], rec["roll"][win]
    W = float(np.trapezoid(L, phi))
    w_far = float(np.trapezoid(rec["tau_far"][win], rec["far"][win]))
    travel = float(th[-1] - th[0])
    r = np.gradient(phi, th)
    pred = (1 + c_run) * L * r + f_run

    print(f"quasi-static stroke, {args.slew_dps:g} deg/s, {CURRENT_LIMIT} counts, 12 V; "
          f"lift window = crank moving -> within {args.level_deg:g} deg of level\n")
    print(f"crank travel {np.degrees(travel):.0f} deg, wing travel "
          f"{np.degrees(phi[-1] - phi[0]):.0f} deg, roll {roll[0]:.0f} -> {roll[-1]:.0f} deg")
    print(f"work at the deploying wing {W * 1e3:.0f} mJ; rising wing {w_far * 1e3:+.1f} mJ; "
          f"motor {np.trapezoid(tm, th) * 1e3:.0f} mJ\n")

    # swing_linkage.py's planar load model on the same geometry, by wing angle:
    # the model the linkage search scores with, checked against this run.
    lk = swing_linkage.SwingLinkage(yaml.safe_load(SWING_LINKAGE_CFG.read_text()))
    _, wd2, load2, _ = swing_linkage.torque_curve(
        lk, swing_linkage.critical_angles(lk).command, step=0.25)
    L2 = np.interp(np.degrees(phi), wd2, load2, left=np.nan, right=np.nan)

    print("the load, and the check that virtual work + running friction is the whole story:")
    print(f"  {'crank':>6} {'wing':>6} {'roll':>5} {'tau_wing':>8} {'2D model':>8} {'r':>6} "
          f"{'motor sim':>9} {'predicted':>9}")
    for j in np.linspace(0, len(th) - 1, 14).astype(int):
        print(f"  {np.degrees(th[j]):6.1f} {np.degrees(phi[j]):6.1f} {roll[j]:5.1f} "
              f"{L[j]:8.3f} {L2[j]:8.3f} {r[j]:6.3f} {tm[j]:9.3f} {pred[j]:9.3f}")
    # Per WING DEGREE, not per sample: the sim lingers at touchdown (the crank
    # all but stalls there), so a per-sample statistic is mostly that instant.
    grid = np.arange(np.ceil(max(wd2[0], np.degrees(phi).min())),
                     np.floor(min(wd2[-1], np.degrees(phi).max())) + 1)
    order = np.argsort(phi)
    e2 = np.abs(np.interp(grid, wd2, load2)
                - np.interp(grid, np.degrees(phi[order]), L[order])) * 1e3
    j2 = int(e2.argmax())
    print(f"  2D model vs sim hinge load, per wing degree: median {np.median(e2):.1f}, "
          f"p95 {np.percentile(e2, 95):.0f}, max {e2[j2]:.0f} mN m at wing "
          f"{grid[j2]:.0f} deg (the wheels' touchdown)")
    # The worst samples are TRANSIENTS -- breakaway at the start, and the
    # wheels touching down (~29 deg of roll), where the crank all but stops and
    # a finite-difference ratio over a few substeps is noise. The median and
    # p95 are the check; the max is printed with where it happened.
    e = np.abs(tm - pred) * 1e3
    j = int(e.argmax())
    print(f"  |sim - predicted| median {np.median(e):.1f}, p95 {np.percentile(e, 95):.1f} "
          f"mN m; max {e[j]:.0f} at {roll[j]:.0f} deg roll (a transient)\n")

    brk = (1 + c_st) * L[0] * r[0] + f_st
    print(f"as built: peak {tm.max():.3f} N m -> {counts(tm.max()):.0f} counts; "
          f"peak/mean {tm.max() / (np.trapezoid(tm, th) / travel):.2f}; breakaway at "
          f"the start {brk:.3f} N m -> {counts(brk):.0f}")
    for s in (1.0, 0.825):
        print(f"  {12 * s:.1f} V: margin {margin(tm.max(), s):.2f}x, "
              f"quasi-steady time {qs_time(th, tm, s):.2f} s")

    cap = "" if args.r_max is None else f", ratio capped at {args.r_max:g}"
    print(f"\nthe FLAT profile over each crank travel{cap} (margin = the Current "
          f"Limit's torque, or stall at the supply, over the need):")
    print(f"  {'travel':>6} {'motor':>6} {'counts':>6} {'r min..max':>11} | "
          f"{'margin 12V':>10} {'time':>5} | {'margin 9.9V':>11} {'time':>5}")
    front = []
    for deg in sorted(set(TRAVELS_DEG) | {round(np.degrees(travel))}):
        T = np.radians(deg)
        tau = flat_level(phi, L, T, args.r_max)
        if tau is None:
            print(f"  {deg:>6} -- the cap alone needs more travel")
            continue
        rr = tau / np.maximum(L, 1e-6)
        if args.r_max is not None:
            rr = np.minimum(rr, args.r_max)
        theta = np.concatenate([[0.0], np.cumsum(np.diff(phi) / (0.5 * (rr[1:] + rr[:-1])))])
        tmf = (1 + c_run) * L * rr + f_run
        pk = float(tmf.max())
        cells = []
        for s in (1.0, 0.825):
            cells.append((margin(pk, s), qs_time(theta, tmf, s)))
        front.append((deg, pk, cells, theta, tmf))
        mark = "  <- as built's travel" if deg == round(np.degrees(travel)) else ""
        print(f"  {deg:>6} {pk:6.3f} {counts(pk):6.0f} {rr.min():5.2f}..{rr.max():<5.2f} | "
              f"{cells[0][0]:9.2f}x {cells[0][1]:5.2f} | {cells[1][0]:10.2f}x "
              f"{cells[1][1]:5.2f}{mark}")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(15, 4.6))
        a = ax[0]
        a.plot(np.degrees(phi), L, "k", label="sim")
        a.plot(wd2, load2, "C1--", label="swing_linkage.py planar model")
        a.legend(fontsize=8)
        a.set_xlabel("deploying wing angle [deg]")
        a.set_ylabel("load at the wing hinge [N m]")
        a.set_title(f"the load: {W * 1e3:.0f} mJ, any transmission")
        top = a.secondary_xaxis("top", functions=(
            lambda x: np.interp(x, np.degrees(phi), roll),
            lambda y: np.interp(y, roll[::-1], np.degrees(phi)[::-1])))
        top.set_xlabel("roll [deg]")
        a.grid(alpha=0.3)
        a = ax[1]
        a.plot((th - th[0]) / travel, tm, "k", lw=2, label="as built (sim)")
        for deg, pk, cells, theta, tmf in front:
            if deg in (132, 180, 270):
                a.plot(theta / theta[-1], tmf, label=f"flat over {deg} deg")
        a.set_xlabel("fraction of the crank stroke")
        a.set_ylabel("motor torque [N m]")
        a.set_title("motor torque through the stroke")
        a.legend()
        a.grid(alpha=0.3)
        a = ax[2]
        for i, (s, col) in enumerate(((1.0, "C0"), (0.825, "C3"))):
            a.plot([f[2][i][1] for f in front], [f[2][i][0] for f in front], "o-",
                   color=col, label=f"flat, {12 * s:.1f} V")
            for deg, pk, cells, theta, tmf in front:
                a.annotate(f"{deg}", (cells[i][1], cells[i][0]), fontsize=7,
                           xytext=(3, 3), textcoords="offset points")
            a.plot(qs_time(th, tm, s), margin(tm.max(), s), "*", ms=14, color=col,
                   label=f"as built, {12 * s:.1f} V")
        a.set_xlabel("stroke time, quasi-steady [s]")
        a.set_ylabel("margin under the Current Limit [x]")
        a.set_title("the front: labels are crank travel [deg]")
        a.legend(fontsize=8)
        a.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(PLOT, dpi=120)
        print(f"\nwrote {PLOT.relative_to(Path.cwd()) if PLOT.is_relative_to(Path.cwd()) else PLOT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
