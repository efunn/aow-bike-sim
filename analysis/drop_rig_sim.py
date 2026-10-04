"""The drop rig in MuJoCo: a wheel on a hinged arm, dropped onto the floor.

The bench rig (docs/plans/drop-release-rig.md, bench/force_drop.py) holds a
wheel at the end of an arm that pivots on an axle; the cam lifts the arm and
lets it fall, and the wheel lands on the force sensor. This is the same rig,
with the wheel exactly as the bike builds it (static_contact_sim's front tire,
build_model._add_aow's rear) and the bike's contact settings, so a simulated
drop can be read the way force_drop.py reads a real one and laid over it.

THE ARM IS BUILT FROM TWO MEASUREMENTS, not from parts: the static force at
the contact (the resting force) and the effective mass there (m_eff = I /
r^2, config/drop_rig_bench.yaml arm.m_eff_g). Those two are what the
contact sees -- and MuJoCo's soft contact scales with the effective mass the
solver compiles around it (static_contact_sim's docstring), so getting it
right is not optional. The wheel's own mass is the bike's; the arm body
makes up the rest of both. The free wheel spins on its own joint, so its
spin inertia stays out of m_eff, as on the bench; --pinned locks it.

Geometry (drop_rig_bench.yaml arm, user, 2026-10-03): contact 205 mm from
the pivot, wheel axle 6.5 mm above it. The floor is the sensor: the wheel
rests on it with the arm about level. A drop sets the arm so the axle sits
h_contact above its settled rest, still, and lets go -- the cam's release,
without the cam. Heights default to the cam's steps through force_drop's
contact_drop (lever and step offset), so they are the bench's contact drops.

    python analysis/drop_rig_sim.py                              # front, config contact
    python analysis/drop_rig_sim.py --compare bench/logs/archive/drops_20261003-2224.csv
    python analysis/drop_rig_sim.py --solref=-34677,-76.3 --compare ...   # "=": a leading "-"
    python analysis/drop_rig_sim.py --wheel rear --rest-n 1.2 --m-eff-g 150
    python analysis/drop_rig_sim.py --compare bench/logs/archive/drops_20261003-2224.csv --fit

--fit: a negative solref (-stiffness, -damping) and solimp, matched to the
bench's mean force curves at every height at once.

THE FIT BELONGS TO THIS RIG'S EFFECTIVE MASS. MuJoCo turns solref into force
through the mass it compiles around the contact (body_invweight0), so the same
pair gives the same contact TIME and bounce at any mass, and a force that
scales with the mass: measured here, (-63500, -48) at 89 / 139 / 189 g gave
12.0 ms and e 0.73-0.74 every time, and peaks 5.3 / 8.6 / 11.9 N. A real tire
has one stiffness in N/m, so on the bench added mass should lengthen the
contact. To carry a fit to the bike, match the bike's own effective mass at
each wheel, or fit on the bike's model (docs/measurements/contact-protocol.md).

Rear: pinned (the bench pins it) and needs its own --rest-n / --m-eff-g; no
rear rig run exists yet to take them from.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import brentq

from aow_sim.build_model import _add_aow, _add_world, _apply_options, load_params

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bench"))
from static_contact_sim import _add_front  # noqa: E402

G = 9.80665
SETTLE_S = 1.0           # the arm onto the floor before the first drop
RECORD_S = 1.4           # after release: impact, bounces and >= 0.2 s at rest
# front, free wheel, 20261003-2224: the resting force at the contact, N
FRONT_REST_N = 0.738


def rig_config() -> dict:
    import yaml
    from force_drop import BENCH_CFG
    return yaml.safe_load(BENCH_CFG.read_text())


def build(wheel="front", rest_n=FRONT_REST_N, m_eff_g=None, pinned=None,
          solref=None, solimp=None, phase_deg=0.0) -> tuple[mujoco.MjModel, dict]:
    """The rig, compiled. Returns the model and what the arm was built from."""
    p = load_params()
    if solref is not None:
        p["sim"]["contact_solref"] = list(solref)
    if solimp is not None:
        p["sim"]["contact_solimp"] = list(solimp)
    arm_cfg = rig_config()["arm"]
    r = arm_cfg["r_contact_mm"] * 1e-3
    up = arm_cfg.get("axle_above_pivot_mm", 6.5) * 1e-3
    m_eff = (m_eff_g if m_eff_g is not None else arm_cfg["m_eff_g"]) * 1e-3
    pinned = (wheel == "rear") if pinned is None else pinned
    radius = (p["omni_wheel"]["outer_radius"] if wheel == "rear"
              else p["bike"]["front_wheel"]["radius"])

    spec = mujoco.MjSpec()
    _apply_options(spec, p)
    _add_world(spec, p)
    pivot = spec.worldbody.add_body(name="pivot", pos=[0, 0, radius - up])
    arm = pivot.add_body(name="arm")
    arm.add_joint(name="arm_hinge", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    hub = arm.add_body(name="axle", pos=[r, 0, up])
    if wheel == "rear":
        _add_aow(spec, hub, p)
    else:
        _add_front(spec, hub, p)
    # the wheel subtree's mass, from a compile with a placeholder arm
    arm.explicitinertial = True
    arm.mass, arm.inertia = 1e-3, [1e-6] * 3
    m0 = spec.compile()
    sub = [b for b in range(m0.nbody) if b >= m0.body("axle").id]
    m_w = float(sum(m0.body_mass[b] for b in sub))
    # the arm makes up the rest: static moment and inertia about the pivot
    moment = rest_n / G * r - m_w * r                # kg m, about the pivot
    inertia = m_eff * r ** 2 - m_w * (r ** 2 + up ** 2)
    x_a = r / 2
    m_a = moment / x_a
    i_cg = inertia - m_a * x_a ** 2
    if m_a <= 0 or i_cg <= 0:
        raise ValueError(f"the wheel ({m_w * 1e3:.1f} g) alone outweighs rest_n {rest_n} N or "
                         f"m_eff {m_eff * 1e3:.0f} g: no arm can make up the difference")
    arm.explicitinertial = True
    arm.mass, arm.ipos, arm.inertia = m_a, [x_a, 0, 0], [i_cg, i_cg, i_cg]
    if wheel == "rear":
        locks = (("hub_spin", np.radians(phase_deg)), ("ring_spin", 0.0))
    else:
        locks = (("front_spin", np.radians(phase_deg)),) if pinned else ()
    for joint, value in locks:
        eq = spec.add_equality()
        eq.type = mujoco.mjtEq.mjEQ_JOINT
        eq.name1 = joint
        eq.data[:5] = [value, 0, 0, 0, 0]
        eq.solref = [0.002, 1.0]
    m = spec.compile()
    return m, dict(wheel=wheel, pinned=pinned, rest_n=rest_n, m_eff_g=m_eff * 1e3,
                   wheel_g=m_w * 1e3, arm_g=m_a * 1e3, r=r, up=up, radius=radius,
                   solref=list(p["sim"]["contact_solref"]), solimp=list(p["sim"]["contact_solimp"]))


def _set_phase(m, d, phase_deg):
    for j in ("hub_spin", "input_a_spin", "input_b_spin"):
        if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j) >= 0:
            d.qpos[m.jnt_qposadr[m.joint(j).id]] = np.radians(phase_deg)


def floor_force(m, d, floor, f=np.zeros(6)) -> float:
    """Total normal force between the floor and anything on it, N."""
    n = 0.0
    for i in range(d.ncon):
        c = d.contact[i]
        if floor in (c.geom1, c.geom2):
            mujoco.mj_contactForce(m, d, i, f)
            n += f[0] * abs(c.frame[2])
    return n


def drops(heights_mm, phase_deg=0.0, record_s=RECORD_S, **kw) -> tuple[list, dict]:
    """Each contact drop in heights_mm (mm, at the contact): (t, force), t = 0
    at release. The arm first settles onto the floor; each drop starts from
    that rest, the axle lifted by h."""
    m, info = build(phase_deg=phase_deg, **kw)
    d = mujoco.MjData(m)
    _set_phase(m, d, phase_deg)
    hinge = m.jnt_qposadr[m.joint("arm_hinge").id]
    axle, floor = m.body("axle").id, m.geom("floor").id
    for _ in range(int(SETTLE_S / m.opt.timestep)):
        mujoco.mj_step(m, d)
    rest_q, rest_qpos = float(d.qpos[hinge]), d.qpos.copy()
    rest_z = float(d.xpos[axle][2])
    info["rest_force_n"] = floor_force(m, d, floor)
    info["rest_angle_deg"] = np.degrees(rest_q)

    def axle_z(q):
        d.qpos[:] = rest_qpos
        d.qpos[hinge] = q
        mujoco.mj_kinematics(m, d)
        return float(d.xpos[axle][2])
    out = []
    for h in heights_mm:
        # lifting is negative about +y (it turns +x towards +z)
        q = brentq(lambda q: axle_z(q) - rest_z - h * 1e-3, rest_q - 0.2, rest_q)
        mujoco.mj_resetData(m, d)
        d.qpos[:] = rest_qpos
        d.qpos[hinge] = q
        mujoco.mj_forward(m, d)
        n = int(record_s / m.opt.timestep)
        t, f = np.empty(n), np.empty(n)
        for i in range(n):
            mujoco.mj_step(m, d)
            t[i], f[i] = d.time, floor_force(m, d, floor)
        out.append((t, f))
    return out, info


def analyse(t, f, h_mm, m_eff_g):
    """force_drop's own analysis, t = 0 at the first sample over its edge."""
    import force_drop as fd
    hit = np.flatnonzero(f > fd.HIT_N)
    if len(hit) == 0:
        return None, None
    t = t - t[hit[0]]
    a = fd.analyse(t, f, h_mm, None, m_eff_g=m_eff_g)
    return t, a


def first_touch(t, f) -> float:
    """When f first crosses force_drop's edge, interpolated between steps (a
    fit needs it smooth in the parameters; the step is 0.4 ms). nan if never."""
    import force_drop as fd
    i = np.flatnonzero(f > fd.HIT_N)
    if len(i) == 0:
        return float("nan")
    i = i[0]
    if i == 0:
        return float(t[0])
    return float(t[i - 1] + (fd.HIT_N - f[i - 1]) / (f[i] - f[i - 1]) * (t[i] - t[i - 1]))


SOLIMP = ("dmin", "dmax", "width", "midpoint", "power")


def _decode(z, fit_imp, solimp0):
    """Unbounded fit variables -> (solref, solimp). Negative solref: k, b > 0
    by exp. solimp: only the terms named in fit_imp move, the rest stay at
    solimp0, each kept in MuJoCo's range: 0.01 < dmin < dmax < 0.9999,
    width > 0, 0 < midpoint < 1, power 1..6."""
    sig = lambda x: 1 / (1 + np.exp(-x))
    solref = [-float(np.exp(z[0])), -float(np.exp(z[1]))]
    imp, it = list(solimp0), iter(z[2:])
    if "dmin" in fit_imp:
        top = 0.9999 if "dmax" in fit_imp else imp[1]
        imp[0] = 0.01 + (top - 0.01) * sig(next(it))
    if "dmax" in fit_imp:
        imp[1] = imp[0] + (0.9999 - imp[0]) * sig(next(it))
    if "width" in fit_imp:
        imp[2] = float(np.exp(next(it)))
    if "midpoint" in fit_imp:
        imp[3] = sig(next(it))
    if "power" in fit_imp:
        imp[4] = 1 + 5 * sig(next(it))
    return solref, [float(x) for x in imp]


def _encode(solref, solimp, fit_imp=SOLIMP):
    logit = lambda y: float(np.log(y / (1 - y)))
    dmin, dmax, width, mid, power = solimp
    z = [np.log(-solref[0]), np.log(-solref[1])]
    if "dmin" in fit_imp:
        top = 0.9999 if "dmax" in fit_imp else dmax
        z.append(logit((dmin - 0.01) / (top - 0.01)))
    if "dmax" in fit_imp:
        z.append(logit((dmax - dmin) / (0.9999 - dmin)))
    if "width" in fit_imp:
        z.append(np.log(width))
    if "midpoint" in fit_imp:
        z.append(logit(mid))
    if "power" in fit_imp:
        z.append(logit((power - 1) / 5))
    return np.array(z)


def fit(bench, steps, h_c, solref0, solimp0, fit_imp=("dmin", "width"), window_ms=80.0,
        maxfev=400, **kw) -> tuple[list, list, float]:
    """solref (negative: stiffness, damping) and the solimp terms in fit_imp
    (the rest held at solimp0) that make the rig's
    drops match the bench's mean force curves, every height at once: the
    squared difference over [-2, window_ms] ms from first touch -- the first
    contact, the flight and the second contact, so the bounce timing (the
    restitution) is in it as well as the shape. Powell, derivative-free: a
    contact's onset moves in 0.4 ms steps. Returns (solref, solimp, rms N)."""
    from scipy.optimize import minimize
    grids = {h: (bench[h][0], bench[h][1]) for h in steps if h in bench}
    if not grids:
        raise ValueError("no bench heights to fit against")
    record_s = 0.06 + window_ms * 1e-3            # release -> impact is < 30 ms here
    calls = [0]

    def cost(z):
        solref, solimp = _decode(z, fit_imp, solimp0)
        if solimp[2] > 0.05:                      # width beyond 5 cm: no longer a contact
            return 1e3
        try:
            runs, _ = drops(h_c, solref=solref, solimp=solimp, record_s=record_s, **kw)
        except ValueError:
            return 1e3
        sq, n = 0.0, 0
        for h, (t, f) in zip(steps, runs):
            if h not in grids:
                continue
            tg, fb = grids[h]
            m = (tg >= -0.002) & (tg <= window_ms * 1e-3)
            t0 = first_touch(t, f)
            if t0 != t0:
                return 1e3
            fs = np.interp(tg[m], t - t0, f, left=0.0, right=0.0)
            sq += float(((fs - fb[m]) ** 2).sum())
            n += int(m.sum())
        calls[0] += 1
        rms = np.sqrt(sq / n)
        if calls[0] % 25 == 0:
            print(f"  fit: {calls[0]} sims, rms {rms:.3f} N  solref {np.round(solref, 1)}  "
                  f"solimp {np.round(solimp, 4)}", flush=True)
        return rms
    z0 = _encode(solref0, solimp0, fit_imp)
    res = minimize(cost, z0, method="Powell", options=dict(maxfev=maxfev, xtol=1e-3, ftol=1e-4))
    solref, solimp = _decode(res.x, fit_imp, solimp0)
    return solref, solimp, float(res.fun)


def bench_means(path, heights):
    """The bench run's per-height mean force trace and mean summary row."""
    import pandas as pd
    import force_drop as fd
    path = Path(path)
    if path.name.endswith("_summary.csv"):
        path = path.with_name(path.name.replace("_summary.csv", ".csv"))
    summ = path.with_name(path.stem + "_summary_reanalysed.csv")    # --reanalyse's, if run
    if not summ.exists():
        summ = path.with_name(path.stem + "_summary.csv")
    print(f"bench: {summ.name}")
    s = pd.read_csv(summ)
    s = s[s.outcome == "ok"] if "outcome" in s else s
    d = pd.read_csv(path)
    scale = np.array(fd.CALIBRATED_N_PER_COUNT)
    tg = np.arange(-0.005, 0.1, 1e-4)
    out = {}
    for h in heights:
        g = s[np.isclose(s.height_mm, h)]
        W = []
        for k in g["drop"]:
            x = d[d["drop"] == k]
            tt = x.t_s.values
            c = x[[f"{ch}_counts" for ch in fd.CHANNELS]].values.astype(float)
            fz = (c - c[tt < -0.002].mean(0)) * scale
            W.append(np.interp(tg, tt, fz[:, int(fz.max(0).argmax())]))
        if W:
            out[h] = (tg, np.mean(W, 0), g.mean(numeric_only=True))
    return out


COLS = ("peak_n", "contact_ms", "flight_ms", "e_flight", "e_ratio", "rest_n")


def main() -> None:
    import force_drop as fd
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--wheel", choices=("front", "rear"), default="front")
    ap.add_argument("--pinned", action="store_true", help="lock the front wheel (rear: always)")
    ap.add_argument("--rest-n", type=float, default=None,
                    help=f"static force at the contact, N (front default {FRONT_REST_N}, from 2224)")
    ap.add_argument("--m-eff-g", type=float, default=None,
                    help="effective mass at the contact, g (default: drop_rig_bench.yaml arm.m_eff_g)")
    ap.add_argument("--steps", default=None,
                    help="cam steps, mm at the follower (default: the yaml's cam.heights), "
                         "turned into contact drops by force_drop.contact_drop")
    ap.add_argument("--solref", help="two numbers; negative = (-stiffness, -damping)")
    ap.add_argument("--solimp", help="five numbers")
    ap.add_argument("--phase-deg", type=float, default=0.0, help="the pinned wheel's angle")
    ap.add_argument("--compare", metavar="CSV", help="a bench run (drops_<time>.csv) to lay over")
    ap.add_argument("--fit", action="store_true",
                    help="fit a negative solref and solimp to --compare's curves, then compare "
                         "with the result (starts from --solref / --solimp, else (-63500, -48) "
                         "and the config's solimp)")
    ap.add_argument("--fit-solimp", default="dmin,width",
                    help="--fit: which solimp terms move (of dmin,dmax,width,midpoint,power; "
                         "'' for none); the rest stay at --solimp / the config's. Default "
                         "dmin,width: dmax only scales the stiffness and damping with them, "
                         "so it trades off against solref")
    ap.add_argument("--fit-ms", type=float, default=80.0,
                    help="--fit: ms after first touch to match (default 80: contact, flight, "
                         "second contact)")
    ap.add_argument("--maxfev", type=int, default=400, help="--fit: simulations at most")
    ap.add_argument("--out", default=None, help="figure (default analysis/drop_rig_sim_<wheel>.png)")
    args = ap.parse_args()
    if args.wheel == "rear" and (args.rest_n is None or args.m_eff_g is None):
        sys.exit("rear: no rear rig run yet -- pass --rest-n and --m-eff-g")
    cfg = rig_config()
    steps = ([float(x) for x in args.steps.split(",")] if args.steps
             else [float(h) for h in cfg["cam"]["heights"]])
    h_c = [fd.contact_drop(h, cfg) for h in steps]
    num = lambda s: [float(x) for x in s.split(",")] if s else None
    solref, solimp = num(args.solref), num(args.solimp)
    rig_kw = dict(phase_deg=args.phase_deg, wheel=args.wheel, pinned=args.pinned or None,
                  rest_n=args.rest_n or FRONT_REST_N, m_eff_g=args.m_eff_g)
    if args.fit:
        if not args.compare:
            sys.exit("--fit needs --compare: the bench run to fit against")
        if solref and solref[0] > 0:
            sys.exit("--fit works in the negative (-stiffness, -damping) form; start it there")
        solimp0 = solimp or list(load_params()["sim"]["contact_solimp"])
        fit_imp = tuple(x for x in args.fit_solimp.split(",") if x)
        if set(fit_imp) - set(SOLIMP):
            sys.exit(f"--fit-solimp: unknown {set(fit_imp) - set(SOLIMP)}; of {','.join(SOLIMP)}")
        print(f"fitting solref + solimp {list(fit_imp) or 'none'} (held: "
              f"{dict((n, v) for n, v in zip(SOLIMP, solimp0) if n not in fit_imp)}) to "
              f"{Path(args.compare).name}, {args.fit_ms:g} ms per height, <= {args.maxfev} sims")
        solref, solimp, rms = fit(bench_means(args.compare, steps), steps, h_c,
                                  solref or [-63500.0, -48.0], solimp0,
                                  fit_imp=fit_imp, window_ms=args.fit_ms,
                                  maxfev=args.maxfev, **rig_kw)
        print(f"fitted: solref [{solref[0]:.1f}, {solref[1]:.2f}]  solimp "
              f"[{', '.join(f'{x:.4g}' for x in solimp)}]  rms {rms:.3f} N")
        print("  valid at THIS rig's effective mass only: MuJoCo scales the contact's force "
              "with the mass it compiles around it (see the docstring)")
    runs, info = drops(h_c, solref=solref, solimp=solimp, **rig_kw)
    print(f"{info['wheel']} wheel ({'pinned' if info['pinned'] else 'free'}), solref {info['solref']}, "
          f"solimp {info['solimp']}")
    print(f"arm: wheel {info['wheel_g']:.1f} g + arm {info['arm_g']:.1f} g -> m_eff {info['m_eff_g']:.0f} g; "
          f"resting {info['rest_force_n']:.3f} N (asked {info['rest_n']:.3f}), "
          f"arm {info['rest_angle_deg']:+.2f} deg from level")
    bench = bench_means(args.compare, steps) if args.compare else {}    # (again, after a fit: cheap)
    rows = []
    for h, hc, (t, f) in zip(steps, h_c, runs):
        tt, a = analyse(t, f, hc, info["m_eff_g"])
        rows.append((h, hc, tt, f, a))
        line = f"  step {h:g} mm (contact {hc:.2f} mm): "
        if a is None:
            print(line + "no impact")
            continue
        line += "  ".join(f"{c} {a[c]:.3g}" for c in COLS)
        if h in bench:
            b = bench[h][2]
            line += "\n      bench:  " + "  ".join(f"{c} {b[c]:.3g}" for c in COLS if c in b)
        print(line)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(len(rows), 1, figsize=(10, 2.6 * len(rows)), sharex=True, squeeze=False)
    for a_, (h, hc, tt, f, a) in zip(ax[:, 0], rows):
        if h in bench:
            a_.plot(bench[h][0] * 1e3, bench[h][1], "k", lw=1.4, label="bench, mean")
        if tt is not None:
            a_.plot(tt * 1e3, f, "C3", lw=1, label="sim")
        a_.set_title(f"step {h:g} mm, contact drop {hc:.2f} mm", fontsize=9)
        a_.set_ylabel("N")
        a_.grid(alpha=.3)
        a_.set_xlim(-5, 100)
    ax[0, 0].legend()
    ax[-1, 0].set_xlabel("ms from the first sample over 0.1 N")
    fig.suptitle(f"{info['wheel']} ({'pinned' if info['pinned'] else 'free'}), "
                 f"solref {info['solref']}, m_eff {info['m_eff_g']:.0f} g", fontsize=10)
    plt.tight_layout()
    out = args.out or Path(__file__).resolve().parent / f"drop_rig_sim_{args.wheel}.png"
    plt.savefig(out, dpi=80)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
