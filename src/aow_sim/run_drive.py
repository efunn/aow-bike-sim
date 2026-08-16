"""Drive harness: python -m aow_sim.run_drive [--view | --teleop].

Headless (default) -- the envelope report:
  1. Straight sprints (fwd/back, config accel): cruise quality, braking
     distance, cross-track, survival.
  2. Accel sweep at v_max until failure -> max clean accel/decel.
  3. Turn-rate envelope: max clean 90-degree command_heading rate per speed,
     forward and reverse, with the equivalent turn radius.
  4. U-turn swept width at 0.8 m/s (the practical sharpness number).
  5. Binary-search circle envelopes: tightest tracked circle and tightest
     stop-from-circle, both directions, at +-0.5 m/s.
  6. Fastest-circle sweep at the tightest radius (+ margin).
  7. 180-degree flip (crawl front-pivot) and two-arc flick scenarios.

--view: scripted demo (sprint, circle lap, stop, flip, flick).
--teleop (macOS: mjpython): RC-style driving. The general RL policy drives by
  default when moves/<control.general_move> exists and matches the current
  obs spec; `,` toggles down to the analytic controller, which owns the
  one-shot maneuvers instead.

  Arrows throttle and steer; 1/3 crab (general mode only); 5 stop; 2 dial;
  / re-zero; ; ' trail history; \\ camera. With --wings: 9 extends the
  righting pair, 4 retracts it, . shoves the bike over. The banner printed at
  startup is the authoritative list, and the README teleop table explains the
  modes.
  Keys avoid A-Z, [ ], `, Space, Tab, Esc and F1-F5 -- all owned by MuJoCo's
  viewer, which sees every keypress before this module does.

--general NAME picks the policy; --ui restores the viewer's side panels.
Use `python -m aow_sim.record` for a headless video of any of this.
"""

from __future__ import annotations

import argparse
import copy

import mujoco
import numpy as np

from .build_model import build_model, load_params, tune_lighting
from .control import DriveController, run
from .control.balance import extract_state
from .control.linearize import settle_upright

UPRIGHT_LIMIT_DEG = 60.0


def _fresh(model, eq_qpos):
    data = mujoco.MjData(model)
    data.qpos[:] = eq_qpos
    a = np.deg2rad(0.5)
    data.qpos[3:7] = [np.cos(a / 2), np.sin(a / 2), 0, 0]
    mujoco.mj_forward(model, data)
    return data


class _Roll:
    def __init__(self, c):
        self.c, self.deg = c, []

    def __call__(self, dd):
        self.deg.append(np.degrees(extract_state(dd, self.c._ref_pos).roll))

    @property
    def ok(self):
        r = np.abs(self.deg)
        return bool(np.all(np.isfinite(r)) and r.max() < UPRIGHT_LIMIT_DEG)


def sprint_scenario(model, params, eq_qpos, v_target: float) -> dict:
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    c.set_speed(v_target)
    roll = _Roll(c)
    ys, vs = [], []

    def rec(dd):
        roll(dd)
        s = extract_state(dd, c._ref_pos)
        ys.append(dd.qpos[1])
        vs.append(s.v_lon)

    t_ramp = abs(v_target) / params["control"]["drive"]["accel"]
    run(model, data, c, t_ramp + 2.0, on_step=rec)
    cruise_v = float(vs[-1])          # sampled at end of cruise, pre-brake
    x_brake = data.qpos[0]
    c.command_stop()
    run(model, data, c, t_ramp + 2.5, on_step=rec)
    return {
        "v_target": v_target,
        "cruise v": round(cruise_v, 3),
        "max |roll| [deg]": round(float(np.max(np.abs(roll.deg))), 2),
        "max cross-track [m]": round(float(np.max(np.abs(ys))), 3),
        "brake+settle [m]": round(abs(float(data.qpos[0] - x_brake)), 3),
        "final v": round(float(vs[-1]), 3),
        "survived": roll.ok,
    }


def accel_sweep(model, params, eq_qpos, accels=(1.5, 2.5, 4.0, 6.0, 9.0)):
    print("\naccel sweep (0 -> v_max -> 0):")
    best = 0.0
    for a in accels:
        p = copy.deepcopy(params)
        p["control"]["drive"]["accel"] = a
        res = sprint_scenario(model, p, eq_qpos, p["control"]["drive"]["v_max"])
        ok = res["survived"] and res["max |roll| [deg]"] < 20
        print(f"  {a:4.1f} m/s^2: roll={res['max |roll| [deg]']:5.2f}  "
              f"cross={res['max cross-track [m]']:.3f}  "
              f"brake={res['brake+settle [m]']:.2f} m  {'ok' if ok else 'FAIL'}")
        if ok:
            best = a
        else:
            break
    print(f"  -> max clean accel/decel: {best:.1f} m/s^2")
    return best


def circle_ok(model, params, eq_qpos, radius, direction, v=0.5,
              stop_test=False) -> tuple[bool, float]:
    """One ramped lap on the circle; optionally command a stop mid-circle.
    Returns (success, mean radius error)."""
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    c.command_circle(data, radius, direction)
    c.set_speed(v)
    roll = _Roll(c)
    radii = []

    def rec(dd):
        roll(dd)
        radii.append(float(np.linalg.norm(dd.qpos[:2] - c._center)))

    lap_t = (abs(v) / params["control"]["drive"]["accel"]
             + 2 * np.pi * radius / abs(v))
    run(model, data, c, lap_t, on_step=rec)
    tail = np.array(radii[len(radii) // 3:])
    err = float(np.mean(np.abs(tail - radius)))
    ok = roll.ok and err < max(0.10 * radius, 0.03)
    if ok and stop_test:
        c.command_stop()
        roll2 = _Roll(c)
        run(model, data, c,
            abs(v) / params["control"]["drive"]["accel"] + 3.0,
            on_step=roll2)
        ok = roll2.ok and abs(roll2.deg[-1]) < 5.0
    return ok, err


def tightest_search(model, params, eq_qpos, direction, stop_test=False,
                    lo=0.2, hi=1.0, tol=0.02, v=0.5) -> float:
    """Binary search the smallest radius that succeeds (assumes monotone)."""
    ok, _ = circle_ok(model, params, eq_qpos, hi, direction, v=v,
                      stop_test=stop_test)
    if not ok:
        return float("nan")
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        ok, _ = circle_ok(model, params, eq_qpos, mid, direction, v=v,
                          stop_test=stop_test)
        if ok:
            hi = mid
        else:
            lo = mid
    return hi


def turn_ok(model, params, eq_qpos, v, rate, delta_deg=90.0) -> bool:
    """command_heading turn at a forced slew rate: upright + tracks."""
    p = copy.deepcopy(params)
    p["control"]["drive"]["yaw_slew_sharp"] = rate
    p["control"]["drive"]["turn_rate_margin"] = 10.0   # cap = rate, not margin
    data = _fresh(model, eq_qpos)
    c = DriveController(p, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    c.set_speed(v)
    run(model, data, c, 2.0)
    psi0 = c._psi
    c.command_heading(data, np.deg2rad(delta_deg))
    roll = _Roll(c)
    run(model, data, c, np.deg2rad(abs(delta_deg)) / rate + 3.0, on_step=roll)
    err = abs(np.degrees(c._psi - psi0) - delta_deg)
    return roll.ok and err < 10.0


def turn_rate_envelope(model, params, eq_qpos,
                       speeds=(0.4, 0.8, 1.2, -0.4, -0.6, -1.0, -1.2)):
    """Binary-search the max clean 90-degree turn rate per speed."""
    print("\nturn-rate envelope (90-degree command_heading, tol 0.1 rad/s):")
    for v in speeds:
        lo, hi = 0.3, 4.0
        if not turn_ok(model, params, eq_qpos, v, lo):
            print(f"  v={v:+.1f}: < {lo} rad/s (FAIL at floor)")
            continue
        while hi - lo > 0.1:
            mid = 0.5 * (lo + hi)
            if turn_ok(model, params, eq_qpos, v, mid):
                lo = mid
            else:
                hi = mid
        r_turn = abs(v) / lo
        print(f"  v={v:+.1f}: max rate {lo:.2f} rad/s  (turn radius ~{r_turn:.2f} m)")


def uturn_width(model, params, eq_qpos, v=0.8) -> float:
    """180-degree turn at speed: swept lateral width (practical sharpness)."""
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    c.set_speed(v)
    run(model, data, c, 2.0)
    c.command_heading(data, np.deg2rad(180))
    roll = _Roll(c)
    ys = []

    def rec(dd):
        roll(dd)
        ys.append(dd.qpos[1])

    run(model, data, c, 6.0, on_step=rec)
    return float(np.ptp(ys)) if roll.ok else float("nan")


def flip_scenario(model, params, eq_qpos, direction=1) -> dict:
    """180-degree swap-ends flip from standstill. Reports upright, final yaw
    error, the peak and final center excursion (in wheelbases), and settle."""
    L = params["bike"]["wheelbase"]
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    psi0 = c._psi
    T = c.command_flip(data, direction)
    C0 = c._flip_center.copy()
    roll = _Roll(c)
    devs = []

    def rec(dd):
        roll(dd)
        cc, ss = np.cos(c._psi), np.sin(c._psi)
        devs.append(float(np.linalg.norm(
            dd.qpos[:2] + (L / 2) * np.array([cc, ss]) - C0)))

    run(model, data, c, T + 5.0, on_step=rec)
    tail = np.abs(roll.deg)[-int(0.5 / model.opt.timestep):]  # roll.deg already in deg
    return {
        "direction": direction,
        "duration [s]": round(T, 2),
        "yaw err [deg]": round(np.degrees(c._psi - psi0) - 180 * np.sign(direction), 1),
        "peak excursion [L]": round(max(devs) / L, 2),
        "final excursion [L]": round(devs[-1] / L, 2),
        "max |roll| [deg]": round(float(np.max(np.abs(roll.deg))), 2),
        "settled RMS [deg]": round(float(np.sqrt(np.mean(tail**2))), 2),
        "survived": roll.ok,
    }


def flick_scenario(model, params, eq_qpos, direction=1, name="flick") -> dict:
    """Optimized two-arc 180 flick from standstill. Reports upright, final yaw,
    the side-to-side lateral envelope (the bounded axis; x is free), x-shift,
    and settle. Requires moves/<name>.yaml (run optimize_flick.py first)."""
    L = params["bike"]["wheelbase"]
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    run(model, data, c, 1.0)
    psi0 = c._psi
    p0 = data.qpos[:2].copy()
    yaw0 = psi0
    T = c.command_flick(data, direction, name=name)
    roll = _Roll(c)
    lats = []

    def rec(dd):
        roll(dd)
        d = dd.qpos[:2] - p0
        lats.append(abs(-np.sin(yaw0) * d[0] + np.cos(yaw0) * d[1]))

    run(model, data, c, T + 4.0, on_step=rec)
    d = data.qpos[:2] - p0
    x_shift = float(np.cos(yaw0) * d[0] + np.sin(yaw0) * d[1])
    tail = np.abs(roll.deg)[-int(0.5 / model.opt.timestep):]
    return {
        "move": name,
        "direction": direction,
        "duration [s]": round(T, 2),
        "yaw err [deg]": round(np.degrees(c._psi - psi0) - 180 * np.sign(direction), 1),
        "lateral env [L]": round(max(lats) / L, 2),
        "x shift [L]": round(x_shift / L, 2),
        "max |roll| [deg]": round(float(np.max(np.abs(roll.deg))), 2),
        "settled RMS [deg]": round(float(np.sqrt(np.mean(tail**2))), 2),
        "survived": roll.ok,
    }


def fastest_circle(model, params, eq_qpos, radius,
                   vs=(0.5, 0.75, 1.0, 1.2)) -> float:
    best = 0.0
    for v in vs:
        ok, _ = circle_ok(model, params, eq_qpos, radius, +1, v=v)
        if ok:
            best = v
        else:
            break
    return best


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--params", default=None)
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--teleop", action="store_true")
    ap.add_argument("--hockey", action="store_true",
                    help="add the ball-shot stick panels + ball (teleop key 1 fires it)")
    ap.add_argument("--general", default=None, metavar="NAME",
                    help="always-on policy to drive with (moves/NAME.{yaml,npz}); "
                         "overrides control.general_move for this session")
    ap.add_argument("--ui", action="store_true",
                    help="restore the viewer's side panels (off by default: "
                         "teleop is keyboard-driven and Reset is Backspace)")
    ap.add_argument("--linkage", action="store_true",
                    help="the four-bar wing mechanism instead of the geared "
                         "pair (config/wing_linkage_locking.yaml); same 9/4 "
                         "keys, but the actuator drives the CRANK")
    ap.add_argument("--wings", action="store_true",
                    help="add the self-righting wing pair (teleop: 9 extends, "
                         "4 retracts). Nothing deploys on its own — the fallen "
                         "state is worth watching")
    args = ap.parse_args()
    params = load_params(args.params)
    model = build_model(params, variant="full", hockey=args.hockey,
                        righting=args.wings or args.linkage,
                        wings=args.wings and not args.linkage,
                        linkage=args.linkage)
    # Same lighting the recorder applies, so a teleop session and a video of
    # the same thing do not look like two different simulators.
    tune_lighting(model)
    eq = settle_upright(model)

    if args.teleop:
        _teleop(model, params, eq.qpos, hockey=args.hockey,
                general=args.general, show_ui=args.ui,
                wings=args.wings, linkage=args.linkage)
        return
    if args.view:
        _view_demo(model, params, eq.qpos, hockey=args.hockey,
                   show_ui=args.ui)
        return

    v_max = params["control"]["drive"]["v_max"]
    print("straight sprints:")
    for vt in (0.8, v_max, -0.5, -v_max):
        res = sprint_scenario(model, params, eq.qpos, vt)
        print("  " + "  ".join(f"{k}={v}" for k, v in res.items()))
    max_acc = accel_sweep(model, params, eq.qpos)

    turn_rate_envelope(model, params, eq.qpos)
    w = uturn_width(model, params, eq.qpos)
    print(f"\nU-turn at 0.8 m/s: swept width {w:.2f} m")

    print("\ncircle envelopes at 0.5 m/s (binary search, tol 2 cm):")
    for direction, tag in ((+1, "CCW"), (-1, "CW")):
        r_track = tightest_search(model, params, eq.qpos, direction)
        r_stop = tightest_search(model, params, eq.qpos, direction,
                                 stop_test=True)
        print(f"  {tag}: tightest tracked R = {r_track:.2f} m; "
              f"tightest stop-from-circle R = {r_stop:.2f} m")
    print("\nreverse circle envelopes at -0.5 m/s:")
    for direction, tag in ((+1, "CCW"), (-1, "CW")):
        r_track = tightest_search(model, params, eq.qpos, direction, v=-0.5)
        print(f"  {tag}: tightest tracked R = {r_track:.2f} m")
    r_ref = 0.5 if np.isnan(r_track) else max(r_track + 0.1, 0.4)
    v_best = fastest_circle(model, params, eq.qpos, r_ref)
    print(f"\nfastest circle at R = {r_ref:.2f} m: {v_best:.2f} m/s")

    print("\n180-degree swap-ends flip (standstill, crawl front-pivot):")
    for direction in (+1, -1):
        res = flip_scenario(model, params, eq.qpos, direction)
        print("  " + "  ".join(f"{k}={v}" for k, v in res.items()))
    print("  (peak excursion ~1 L is intrinsic: exact center-spin is a delta=90"
          " singularity)")

    from .control.flick import MOVES_DIR
    print("\n180-degree two-arc flick (lateral bounded, x free):")
    variants = (("flick", "trajopt reverse-first"),
                ("flick_fwd", "trajopt forward-first"),
                ("flick_rl", "RL policy (closed-loop)"))
    for move, label in variants:
        if (MOVES_DIR / f"{move}.yaml").exists():
            try:
                res = flick_scenario(model, params, eq.qpos, +1, name=move)
            except ValueError as e:      # stale RL move (obs spec changed)
                print(f"  [{label}] {e}")
                continue
            print(f"  [{label}] " + "  ".join(f"{k}={v}" for k, v in res.items()))
        else:
            how = ("python -m aow_sim.train_flick_rl" if move == "flick_rl"
                   else f"python -m aow_sim.optimize_flick"
                        f"{' --reverse-first' if move == 'flick' else f' --name {move}'}")
            print(f"  [{label}] no moves/{move}.yaml — run `{how}`")
    print(f"\nsummary: v_max ±{v_max} m/s straight OK, max accel {max_acc:.1f} m/s^2")


def _view_demo(model, params, eq_qpos, hockey=False, show_ui=False):
    # Uses the passive viewer (launch_passive via teleop_loop), not the managed
    # mujoco.viewer.launch app — the latter spins up its own _Simulate and is
    # unreliable under mjpython on macOS ("_Simulate ... unknown exception").
    from .interactive import teleop_loop
    from .control.flick import MOVES_DIR
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    if hockey:
        if not (MOVES_DIR / "ball_rl.yaml").exists():
            raise SystemExit("no moves/ball_rl.yaml — run "
                             "`python -m aow_sim.train_ball_rl`")
        # Ball-shot-only demo: fire the RL ball move from standstill.
        plan = [(1.0, lambda d: (_reset_ball(model, d, params),
                                 c.command_ball(d, name="ball_rl")))]
        intro = "viewer demo: ball-shot (RL) from standstill"
    else:
        plan = [
            (1.0, lambda d: c.set_speed(0.8)),
            (4.0, lambda d: c.command_circle(d, 0.8, +1)),
            (14.0, lambda d: c.command_stop()),
            (17.0, lambda d: c.command_flip(d, +1)),
        ]
        if (MOVES_DIR / "flick.yaml").exists():
            plan.append((22.0, lambda d: c.command_flick(d, +1)))
        intro = "viewer demo: sprint 0.8 m/s, circle R=0.8, stop, flip, flick"

    stage = {"i": 0}
    overlay_on = [True]

    def step(m, d):
        if stage["i"] < len(plan) and d.time >= plan[stage["i"]][0]:
            plan[stage["i"]][1](d)
            stage["i"] += 1
        c.step(m, d)

    teleop_loop(model, data, step, lambda k: None, intro, "aow_sim.run_drive",
                draw=lambda scn, m, d: _overlay(scn, m, d, c, overlay_on),
                show_ui=show_ui)


_DIAL_R = 0.30      # m, ground-dial radius — the rim carries heading ticks
_VEL_R = 0.20       # m, full-scale radius of the inner velocity gauge (= v_max).
                    #   Strictly inside the heading ticks (which start at
                    #   0.80*_DIAL_R = 0.24 m) so a commanded course pointing
                    #   along the commanded heading can never bury one in the
                    #   other — that is what made the orange arrow invisible.
_DIAL_Z = 0.004     # m, just above the floor so the dial isn't z-fighting
_CMD = (0.25, 1.0, 0.35, 1.0)      # green  — commanded
_CMD_V = (1.0, 0.55, 0.1, 1.0)     # orange — commanded velocity
_ACT = (0.2, 0.8, 1.0, 1.0)        # cyan   — actual heading
_ACT_V = (1.0, 0.92, 0.35, 1.0)    # yellow — actual velocity
_RING = (0.65, 0.65, 0.62, 0.35)   # grey   — dial rim
_TRAIL = (0.90, 0.10, 0.10)        # red    — where the bike has actually been
_TRAIL_SOLID_S = 2.0               # s of history drawn at full opacity
_TRAIL_FADE_S = 0.5                # s of older history fading to clear
_GRID_PITCH = 0.5                  # m between floor grid lines
_GRID_HALF = 3.0                   # m, grid extent either side of the bike
_GRID_RGBA = (0.55, 0.55, 0.55, 0.28)


def _command_ref(c, data):
    """(commanded heading [rad], commanded world velocity [m/s]) for the
    current mode. The general policy carries a full velocity VECTOR (it can
    crab), so read it directly; the analytic modes expose a heading plus a
    speed along it via viz_reference."""
    if c.mode == "general":
        return float(c._gen_psi_cmd), np.asarray(c._gen_v_cmd, float)[:2]
    h, s = c.viz_reference(data)
    return float(h), float(s) * np.array([np.cos(h), np.sin(h)])


def _overlay(scn, model, data, c, on, v_max=1.2, reset=True,
             command=None, trail=None, grid=False, trail_level=None):
    """Ground dial under the bike showing the teleop command against reality.

    Headings live ON the rim as radial ticks; velocities are arrows from the
    centre, scaled so the rim means v_max. Keeping the two on different radii
    matters: the commanded course normally points along the commanded
    heading, so drawing both as centre-out rays from the same origin buries
    the (short) velocity arrow inside the (full-length) heading one.

      green tick  = commanded heading      cyan tick   = actual heading
      orange ray  = commanded velocity     yellow ray  = actual velocity
    """
    # reset=True for the viewer's user_scn (ours alone, cleared each frame);
    # reset=False to append onto an mjvScene that already holds the model
    # geoms, as the offscreen recorder does.
    if reset:
        scn.ngeom = 0
    if not on[0]:
        return
    p = data.body("chassis").xpos
    base = np.array([p[0], p[1], _DIAL_Z])

    def seg(a, b, kind, width, rgba):
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, kind, np.zeros(3), np.zeros(3), np.zeros(9),
                            np.asarray(rgba, np.float32))
        mujoco.mjv_connector(g, kind, width, a, b)
        scn.ngeom += 1

    def at(heading, radius):
        return base + radius * np.array([np.cos(heading), np.sin(heading), 0.0])

    def ray(heading, length, width, rgba):
        seg(base, at(heading, length), mujoco.mjtGeom.mjGEOM_ARROW, width, rgba)

    def tick(heading, r0, r1, width, rgba):
        seg(at(heading, r0), at(heading, r1),
            mujoco.mjtGeom.mjGEOM_ARROW, width, rgba)

    # rim: a dashed circle, the reference dial the ticks sit on
    n = 48
    for i in range(0, n, 2):
        a0, a1 = 2 * np.pi * i / n, 2 * np.pi * (i + 1) / n
        seg(at(a0, _DIAL_R), at(a1, _DIAL_R),
            mujoco.mjtGeom.mjGEOM_LINE, 4.0, _RING)

    # Floor grid: a world-FIXED reference. Without it a follow camera keeps the
    # bike centred and stationary-looking, so translation is invisible -- the
    # same trap the recorder hit. Snapped to the pitch so it reads as fixed
    # ground while only ever spanning the area around the bike.
    if grid:
        cx = round(float(p[0]) / _GRID_PITCH) * _GRID_PITCH
        cy = round(float(p[1]) / _GRID_PITCH) * _GRID_PITCH
        n = int(_GRID_HALF / _GRID_PITCH)
        for i in range(-n, n + 1):
            u = i * _GRID_PITCH
            seg(np.array([cx + u, cy - _GRID_HALF, 0.001]),
                np.array([cx + u, cy + _GRID_HALF, 0.001]),
                mujoco.mjtGeom.mjGEOM_LINE, 1.5, _GRID_RGBA)
            seg(np.array([cx - _GRID_HALF, cy + u, 0.001]),
                np.array([cx + _GRID_HALF, cy + u, 0.001]),
                mujoco.mjtGeom.mjGEOM_LINE, 1.5, _GRID_RGBA)

    # Path history: solid red for the recent _TRAIL_SOLID_S, then fading to
    # clear over _TRAIL_FADE_S. The dial travels with the bike and so cannot
    # show displacement; this can.
    if trail is not None:
        now = float(data.time)
        solid = _TRAIL_SOLID_S if trail_level is None else float(trail_level)
        if trail and now < trail[-1][0]:
            trail.clear()          # viewer reset rewound the clock
        # level 0 is PEN UP, not erase: stop laying down new points but keep
        # (and keep drawing) everything already drawn. That is what lets a
        # disconnected shape -- the crossbar and stem of a T -- be drawn
        # without retracing.
        if solid > 0.0:
            trail.append((now, float(p[0]), float(p[1])))
        horizon = solid + _TRAIL_FADE_S
        if np.isfinite(solid) and solid > 0.0:
            while trail and now - trail[0][0] > horizon:
                trail.pop(0)
        # An unbounded (or merely long) trail would exhaust scn.maxgeom and
        # silently truncate whatever is drawn after it -- including the dial.
        # Stride down to a budget instead, keeping both endpoints.
        pts = list(trail)
        budget = max(64, (scn.maxgeom - scn.ngeom) - 64)
        if len(pts) > budget:
            idx = np.linspace(0, len(pts) - 1, budget).astype(int)
            pts = [pts[i] for i in idx]
        for (t0, x0, y0), (t1, x1, y1) in zip(pts, pts[1:]):
            age = now - t0
            if not np.isfinite(solid) or solid == 0.0:
                a = 1.0                      # inf / pen-up: never fades
            else:
                a = 1.0 if age <= solid else max(
                    0.0, (horizon - age) / _TRAIL_FADE_S)
            if a > 0.01:
                seg(np.array([x0, y0, 0.002]), np.array([x1, y1, 0.002]),
                    mujoco.mjtGeom.mjGEOM_LINE, 5.0, (*_TRAIL, a))

    R = data.body("chassis").xmat.reshape(3, 3)
    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    v = np.asarray(data.qvel[:2], float)
    vscale = _VEL_R / max(v_max, 1e-6)      # inner gauge full scale == v_max
    # `command` lets a recorder replay the (heading, velocity) that was
    # live at each stored frame; without it the dial would show the
    # controller's FINAL command on every frame of the playback.
    h_cmd, v_cmd = _command_ref(c, data) if command is None else command

    # velocities: centre-out arrows inside the gauge, actual under commanded
    speed = float(np.linalg.norm(v))
    if speed > 1e-3:
        ray(float(np.arctan2(v[1], v[0])),
            min(vscale * speed, _VEL_R), 0.010, _ACT_V)
    cmd_speed = float(np.linalg.norm(v_cmd))
    if cmd_speed > 1e-3:
        ray(float(np.arctan2(v_cmd[1], v_cmd[0])),
            min(vscale * cmd_speed, _VEL_R), 0.020, _CMD_V)

    # headings: ticks straddling the rim, clear of the velocity gauge
    tick(yaw, _DIAL_R * 0.88, _DIAL_R * 1.08, 0.012, _ACT)
    tick(h_cmd, _DIAL_R * 0.80, _DIAL_R * 1.30, 0.024, _CMD)


# -- teleop input model ---------------------------------------------------
#
# MuJoCo's viewer key callback is Callable[[int], None]: it reports a key
# going down, never coming up, and in practice does NOT deliver OS
# auto-repeat while a key is held. So a purely hold-based model degenerates
# to a single event per press — which is exactly the "arrows don't do much"
# failure: one short ramp, then the coast-down eats it again.
#
# The model below therefore works under either behavior:
#   * a FRESH press applies a discrete step immediately. This is the only
#     thing that can work when auto-repeat is absent, and it makes holding a
#     key degrade gracefully into "tap it a few times".
#   * an event arriving within _REPEAT_GAP of the previous one proves auto-
#     repeat is live and the key is genuinely held, which switches on the
#     continuous ramp.
#   * after _COAST_DELAY with no events the axis counts as released, and the
#     speed target coasts to zero (heading, being a setpoint, just stays).
# Splitting machine repeat from human tapping: OS auto-repeat runs ~25-33 Hz
# (0.03-0.04 s apart) and bottoms out near 8 Hz; sustained human tapping is
# slower than that. 0.12 s sits in the gap, so repeats ramp and taps step.
_REPEAT_GAP = 0.12     # s, an event this soon after the last is auto-repeat
_GRACE_REPEAT = 0.18   # s, ramp runs this long past the last repeat (must
                       #   exceed the slowest repeat interval or it stutters)
_COAST_DELAY = 0.50    # s of silence before the speed target coasts down

# Driving-game longitudinal feel. These set the TARGET; SpeedProfile still
# rate-limits the actual command at control.drive.accel (1.5 m/s^2), so the
# ramp rates sit at or above it — otherwise the teleop double-limits and
# everything feels sluggish.
_STEP_V = 0.25         # m/s applied instantly on a fresh press
_ACCEL = 1.5           # m/s^2 while a held key streams repeats
_ACCEL_REV = 1.1       # m/s^2 while building reverse speed
_BRAKE = 2.5           # m/s^2 when the opposite key is held
_DECAY = 0.6           # m/s^2 coast-down once released

_STEP_PSI = np.deg2rad(10.0)   # heading nudge on a fresh press
_TURN_RATE = 1.2               # rad/s continuous slew while held
# How far the commanded heading may lead the actual heading. Without this a
# held turn key winds the command far past anything the bike can follow, and
# it keeps spinning long after you let go. Snaps deliberately bypass it.
_LEAD_MAX = np.deg2rad(35.0)


# macOS virtual keycodes for the arrows (Carbon kVK_* constants).
# macOS virtual keycodes for the keys hold-detection tracks. These are
# POSITIONAL, not character codes: 18/20 are the ANSI-1 and ANSI-3 key
# positions, so on a non-US layout the keys printing "1"/"3" may sit
# elsewhere and disagree with the GLFW side (ord("1")). Harmless if so --
# the crab axis just falls back to auto-repeat inference, the same path used
# when Input Monitoring is denied.
_MAC_KEYS = {"left": 123, "right": 124, "down": 125, "up": 126,
             "crab_left": 18, "crab_right": 20}


class _KeyState:
    """True physical key up/down, when the OS can supply it.

    MuJoCo's viewer callback (Callable[[int], None]) reports only a key going
    DOWN — never up, and in practice not auto-repeat either — so
    hold-to-drive cannot come from the viewer at all. Two macOS routes are
    tried, and whichever proves itself first wins:

      "monitor" — an NSEvent local monitor. Sees keyDown/keyUp delivered to
                  our OWN process, needs NO permission, and is inherently
                  focus-scoped. Requires an NSApplication event loop, which
                  the viewer window provides.
      "quartz"  — polling CGEventSourceKeyState. Works without a window but
                  needs Input Monitoring permission; without it the call does
                  not fail, it just reports every key as up forever.

    Both degrade silently: until one has actually observed a key going down
    (`confirmed`), the caller keeps using the tap-to-step model, so a missing
    permission can never make things worse than tapping.
    See `python -m aow_sim.check_input` to diagnose which routes work."""

    POLL_HZ = 120.0          # plenty for input; the caller runs at 5 kHz

    def __init__(self):
        self.source = None       # None | "monitor" | "quartz"
        self.confirmed = False   # True once a key has actually read as down
        self._down = {}          # name -> bool, from the NSEvent monitor
        self._cache = {}         # name -> bool, from Quartz polling
        self._t = -1e9
        self._monitor = None
        self._handler = None     # kept alive; the monitor does not retain it
        self._quartz = None
        try:
            import platform
            if platform.system() == "Darwin":
                self._install_monitor()
                self._init_quartz()
        except Exception:
            pass
        self.available = self._monitor is not None or self._quartz is not None

    def _install_monitor(self):
        try:
            from AppKit import (  # type: ignore[import-not-found]
                NSEvent, NSEventMaskKeyDown, NSEventMaskKeyUp,
                NSEventTypeKeyDown)
        except Exception:
            return
        codes = {kc: name for name, kc in _MAC_KEYS.items()}

        def handler(event):
            try:
                name = codes.get(int(event.keyCode()))
                if name is not None:
                    is_down = int(event.type()) == int(NSEventTypeKeyDown)
                    self._down[name] = is_down
                    if is_down:
                        self.confirmed = True
                        self.source = "monitor"
            except Exception:
                pass
            return event          # pass it through; never swallow input

        try:
            self._handler = handler
            self._monitor = (
                NSEvent.addLocalMonitorForEventsMatchingMask_handler_(
                    NSEventMaskKeyDown | NSEventMaskKeyUp, handler))
        except Exception:
            self._monitor = self._handler = None

    def _init_quartz(self):
        try:
            from Quartz import (  # type: ignore[import-not-found]
                CGEventSourceKeyState,
                kCGEventSourceStateHIDSystemState as _HID)
            self._quartz = lambda kc: bool(CGEventSourceKeyState(_HID, kc))
        except Exception:
            self._quartz = None

    @property
    def routes(self) -> list:
        """Backends that installed OK (not proof any of them will report)."""
        return ([] if self._monitor is None else ["NSEvent monitor"]) + \
               ([] if self._quartz is None else ["Quartz polling"])

    def down(self, name) -> bool:
        if self.source == "monitor":
            return bool(self._down.get(name, False))
        if self.source == "quartz":
            return bool(self._cache.get(name, False))
        return False

    def poll(self, now) -> None:
        """Refresh Quartz state at POLL_HZ. The monitor is event-driven, so
        once it has proven itself there is nothing to poll."""
        if self.source == "monitor" or self._quartz is None:
            return
        if (now - self._t) < 1.0 / self.POLL_HZ:
            return
        self._t = now
        for name, kc in _MAC_KEYS.items():
            try:
                v = self._quartz(kc)
            except Exception:        # never let input polling kill the sim
                self._quartz = None
                self.available = self._monitor is not None
                return
            self._cache[name] = v
            if v:
                self.confirmed = True
                self.source = "quartz"


class _Axis:
    """One bidirectional teleop input.

    Two hold-detection paths, in preference order:
      * real key state (`physical_hold`) when _KeyState can supply it —
        proper hold-to-accelerate with an instant release;
      * otherwise auto-repeat inference: `press` returns True on a fresh
        press (apply the discrete step), and `ramping` turns on only once a
        repeat has proven the key is held.

    `armed` gates the physical path on the viewer having actually delivered a
    keydown, so keys pressed while another window has focus can never drive
    the bike."""

    def __init__(self):
        self.last = -1e9
        self.dir = 0
        self.repeating = False
        self.armed = False

    def press(self, now, direction) -> bool:
        fresh = (now - self.last) > _REPEAT_GAP or direction != self.dir
        self.dir = direction
        self.repeating = not fresh      # a repeat proves the key is held
        self.last = now
        self.armed = True
        return fresh

    def physical_hold(self, down_pos, down_neg) -> int:
        """Ramp direction from true key state; 0 when released."""
        if not self.armed:
            return 0
        if self.dir > 0 and down_pos:
            return 1
        if self.dir < 0 and down_neg:
            return -1
        self.armed = False              # observed physically released
        return 0

    def ramping(self, now) -> bool:
        return self.repeating and (now - self.last) < _GRACE_REPEAT

    def released(self, now) -> bool:
        return (now - self.last) > _COAST_DELAY

    def clear(self):
        self.last = -1e9
        self.dir = 0
        self.repeating = False
        self.armed = False


def _reset_ball(model, data, params):
    """Re-park the ball at its bike-frame start pose (hockey model only), so a
    fresh shot can be attempted. No-op if the model has no ball."""
    try:
        jid = int(model.body("ball").jntadr[0])
    except Exception:
        return
    q, v = int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])
    ball = params["hockey"]["ball"]
    data.qpos[q:q + 2] = ball["start"]
    data.qpos[q + 2] = ball["radius"]
    data.qpos[q + 3:q + 7] = [1, 0, 0, 0]
    data.qvel[v:v + 6] = 0.0


def _teleop(model, params, eq_qpos, hockey=False, general=None,
            show_ui=False, wings=False, linkage=False):
    from .interactive import teleop_loop

    from . import policy_menu

    # Which always-on policy to drive with: --general wins, else the config's
    # control.general_move, else the trainer's default export name. A LIST
    # because the TAB menu rebinds it live — this is the startup choice, not
    # the session's.
    gen_name = [general or params["control"].get("general_move", "general_rl")]
    # cursor/entries are rebuilt on every open, so a policy exported from
    # another terminal mid-session shows up without restarting teleop.
    menu = {"open": False, "cursor": 0, "entries": []}
    data = _fresh(model, eq_qpos)
    c = DriveController(params, model)
    c.reset(model, data)
    pending = []
    # `v` is the operator's speed INTENT and `psi` the absolute commanded
    # heading; the controller (or policy) still rate-limits how it gets there.
    # `want_general` is the operator's intent, not the controller's state:
    # the viewer's reset (and any auto-reset on time rewind) drops
    # DriveController back to line mode, so the intent has to live out here
    # to be re-applied. See `ensure_mode`.
    # `v` is the operator's longitudinal speed INTENT, `v_lat` the sideways
    # (crab) one -- the rear omni decouples travel direction from heading, and
    # the general policy takes a velocity VECTOR, so crab is a first-class
    # command rather than a maneuver.
    state = {"v": 0.0, "v_lat": 0.0, "psi": c._psi, "psi_sent": c._psi,
             "want_general": True}
    overlay_on = [True]
    trail = []                 # (t, x, y) history for the red path trace
    # Seconds of SOLID history; 0 = pen up (stop drawing, keep what is drawn),
    # inf = never expires. [ and ] step through it.
    trail_levels = [0.0, 2.0, 4.0, 10.0, float("inf")]
    trail_level = [1]          # index; default 2 s
    # free = the viewer's own mouse camera; follow = chase from behind;
    # overhead = plan view. Both tracked modes need the floor grid to be
    # legible, since a tracked camera holds the bike still in frame.
    cam_mode = ["free"]
    cam_free_pending = [False]   # re-frame free view on the switch INTO it
    lead_armed = [True]          # heading lead clamp; a snap disarms it
    view = [None]              # the viewer handle, once teleop_loop hands it over
    chassis_id = model.body("chassis").id
    ax_v, ax_psi, ax_lat = _Axis(), _Axis(), _Axis()
    keys = _KeyState()
    announced = [False]
    v_max = c.profile.v_max
    # Crab is clamped to the envelope THIS policy trained on (moves/<name>.yaml
    # carries v_lat_frac); asking for more is off-distribution. Resolved lazily
    # in apply() because the policy is not loaded until engage().
    crab_max = [0.4 * v_max]

    # -- the righting wings, when --wings built them ------------------------
    # Deliberately MANUAL and one-shot: 9 latches a target of full deploy, 0 a
    # target of stow, and the pair slews there. No auto-deploy and no auto
    # hand-off, because the whole point of driving this by hand is to watch
    # what a fallen bike actually does before deciding what should trigger.
    # The viewer only ever reports a key going DOWN, so latch-a-target is the
    # only thing that can work here -- hold-to-move would need key-up.
    # The LINKAGE drives a different joint through a different range, and the
    # units are not comparable: the geared pair is commanded in WING degrees
    # (0..deploy_deg) while the four-bar is commanded in CRANK degrees
    # (0..servo_travel_deg), with a ratio that varies 0.17..0.70 through the
    # stroke instead of being fixed. A schedule tuned for one is meaningless
    # for the other -- see the rate note below.
    wing = None
    if wings or linkage:
        if linkage:
            import yaml as _yaml
            from .build_model import LINKAGE_CFG
            lcfg = _yaml.safe_load(LINKAGE_CFG.read_text())
            joint = "wing_crank_joint"
            stow_rad = 0.0
            deploy_rad = np.deg2rad(float(lcfg["stroke"]["servo_travel_deg"]))
            # CURRENT-BASED POSITION MODE, which is how the XC330 will actually
            # be driven: a position setpoint plus a goal current, moving as
            # fast as it can under that cap. There is no commanded trajectory,
            # so `rate` is effectively infinite and the current limit is the
            # only throttle. Ramping it was my invention and it was slow --
            # measured 5.2 s for a stroke the mechanism does in ~0.3 s.
            #
            # Safe here only because the four-bar SELF-LIMITS: near full
            # deployment its ratio collapses toward zero, so the wing
            # decelerates into the end pose however hard the crank is driven.
            # The geared pair has no such property and somersaults the bike
            # under the same command (0/8 falls recovered), which is why this
            # is not shared with it.
            rate, gear = float("inf"), 1.0
        else:
            wcfg = params["righting"]["wings"]
            joint = "wing_right_joint"
            stow_rad = np.deg2rad(wcfg["stow_deg"])
            deploy_rad = np.deg2rad(wcfg["deploy_deg"])
            rate, gear = 0.7, wcfg["gear_ratio"]
        if linkage:
            # Goal current, from the linkage config rather than a literal --
            # it is the only throttle on the stroke, and it moved once already
            # (0.40 -> 0.66) when the roof derivation was fixed. A number that
            # tracks a measurement belongs next to the measurement.
            cap = float(lcfg["stroke"]["goal_current_nm"])
            model.actuator_forcerange[model.actuator("wings").id] = [-cap, cap]
        wing = {"aid": model.actuator("wings").id,
                "jadr": model.joint(joint).qposadr[0],
                "stow": stow_rad, "deploy": deploy_rad,
                "rate": rate, "gear": gear, "linkage": linkage,
                "cmd": stow_rad, "target": stow_rad,
                # A repeatable shove, so knocking the bike over is one key
                # rather than a mouse gesture. Same lateral force pulse
                # analysis/self_righting.py falls the bike with; 8 N for 0.35 s
                # is comfortably past the recoverable set at any speed.
                # Set the moment 9 or 4 is pressed: the operator has taken
                # the wings back from a policy that drives them. Without this
                # a wing-driving policy owns the channel unconditionally, so
                # the manual keys do nothing -- and a bike that has flipped
                # cannot be righted by hand, because the policy has never seen
                # a fallen state and just holds whatever it holds.
                "manual": False,
                "push_n": 0, "push_dir": 1.0, "push_force": 8.0,
                "push_s": 0.35, "body": model.body("chassis").id}

    def policy_owns_wings() -> bool:
        """True when the engaged general policy is DRIVING the wings itself.

        A policy trained with `act_wings` commands the wing channel from its
        4th action. Stamping the teleop target over it every step leaves the
        wings stowed no matter what the policy asks for -- and a policy that
        learned to ride on them then falls over instantly, which is exactly
        what happened the first time this was driven."""
        return (c.mode == "general"
                and bool(getattr(c._gen, "act_wings", False))
                and not wing["manual"])

    def wing_step(m, d):
        """Slew the pair toward its latched target and hold it there.

        MUST run after `c.step`: the balance controllers write the WHOLE ctrl
        vector (`data.ctrl[:] = self._u`), so anything written before them is
        overwritten every step. The exception is a wing-driving policy, which
        writes that channel itself inside `c.step` and must not be clobbered
        here -- see policy_owns_wings."""
        if wing is None:
            return
        if policy_owns_wings():
            # Keep the latched target tracking the policy so that handing
            # control back (engage the analytic controller, or load a policy
            # without act_wings) does not snap the wings to a stale setpoint.
            wing["cmd"] = wing["target"] = float(d.qpos[wing["jadr"]])
        else:
            t, cmd = wing["target"], wing["cmd"]
            step = wing["rate"] * m.opt.timestep
            wing["cmd"] = (min(cmd + step, t) if t > cmd else max(cmd - step, t))
            d.ctrl[wing["aid"]] = wing["cmd"]
        # The shove, if one is running. xfrc_applied PERSISTS, so it has to be
        # cleared on the step the pulse ends or the bike gets pushed forever.
        if wing["push_n"] > 0:
            wing["push_n"] -= 1
            d.xfrc_applied[wing["body"], 1] = (
                wing["push_dir"] * wing["push_force"] if wing["push_n"] else 0.0)

    def on_key(keycode):
        pending.append(keycode)

    def zero_command(d):
        """Drop every residual command: speed intent, key state, and the
        heading command re-aimed at where the bike actually points. Runs on
        every mode change so a policy never inherits a stale setpoint."""
        state["v"] = 0.0
        state["v_lat"] = 0.0
        state["psi"] = state["psi_sent"] = c._psi
        ax_v.clear()
        ax_psi.clear()
        lead_armed[0] = True        # command re-anchored on the bike
        ax_lat.clear()
        if c.mode == "general":
            c.set_command_polar(0.0, psi_cmd=c._psi)
        else:
            c.set_speed(0.0)

    def engage(d, quiet=False) -> bool:
        """Hand control to the general policy. Returns False (and clears the
        intent) if there is no usable policy, so the caller falls back to the
        analytic controller instead of retrying every step."""
        try:
            c.engage_general(d, name=gen_name[0])
        except FileNotFoundError:
            state["want_general"] = False
            if not quiet:
                print(f"no moves/{gen_name[0]}.yaml — using the analytic "
                      "controller (train with `python -m "
                      "aow_sim.train_general_rl`)")
            return False
        except ValueError as e:          # stale policy (obs spec changed)
            state["want_general"] = False
            # NOT gated on `quiet`. An obs-dim mismatch is a configuration
            # error, not a transient: `ensure_mode` calls this with
            # quiet=True after a viewer reset, so gating it meant a stale
            # policy dropped to the analytic controller with no message at
            # all and the operator just wondered why the bike felt different.
            # Printed once so a per-step retry cannot spam the console.
            if not state.get("gen_obs_warned"):
                state["gen_obs_warned"] = True
                print(e)
            return False
        # The policy loaded. Say whether it was trained against the physics
        # currently in bike_params.yaml — nothing else reports this, and a
        # policy silently belonging to an older plant is the failure mode
        # that put a stale export in control.general_move for three days.
        from .control.flick import check_move_digest
        check_move_digest(c._gen, params)
        zero_command(d)                  # never inherit a stale setpoint
        # Re-engaging hands the wings back to the policy, so the manual
        # override is a temporary grab (right the bike by hand, then re-engage)
        # rather than a one-way door.
        if wing is not None and wing["manual"]:
            wing["manual"] = False
            print("wings: returned to the policy")
        return True

    def select(d, name: str) -> None:
        """Act on a menu choice: the analytic sentinel, or a policy by name."""
        if name == policy_menu.ANALYTIC:
            state["want_general"] = False
            c.command_line(d)
            zero_command(d)
            print("analytic controller (LQR) — maneuver keys live again")
            return
        prev = gen_name[0]
        gen_name[0] = name
        state["want_general"] = True
        if engage(d):
            entry = next((e for e in menu["entries"]
                          if not isinstance(e, str) and e["name"] == name), None)
            print(f"driving moves/{name}"
                  + (f"\n  {policy_menu.summarize(entry)}" if entry else ""))
        else:
            gen_name[0] = prev          # keep driving what was working

    def ensure_mode(d):
        """Re-assert the operator's intent after anything resets the
        controller. The viewer's reset rewinds data.time, which makes
        _Base.step call DriveController.reset -> command_line, silently
        dropping the policy; without this the bike comes back under the
        analytic controller every time you rewind."""
        if state["want_general"] and c.mode != "general":
            engage(d, quiet=True)

    def lead_now():
        """Signed heading command lead over the bike, wrapped to +-180."""
        return float(np.arctan2(np.sin(state["psi"] - c._psi),
                                np.cos(state["psi"] - c._psi)))

    def turn(delta, clamp=True):
        """Move the heading command. `clamp` keeps a *held* turn from winding
        the command past what the bike can follow (which would leave it
        spinning long after release); snaps pass clamp=False because a
        commanded 90/180 is meant to lead.

        A snap DISARMS the clamp until the bike catches up. Without that, the
        snap leaves a 90-180 deg lead, and the gate below -- which only ever
        blocks the direction that would grow the lead -- kills continuous
        turning one way while allowing the other. That reads as "steering
        broke after a snap". While disarmed both directions are free; the
        clamp re-arms in `apply` the moment the lead falls back inside the
        band, so its anti-windup job resumes as soon as it can be done
        without fighting a deliberate command."""
        if clamp and lead_armed[0]:
            lead = lead_now()
            if (delta > 0 and lead >= _LEAD_MAX) or \
               (delta < 0 and lead <= -_LEAD_MAX):
                return
        if not clamp:
            lead_armed[0] = False        # a snap is meant to lead; let it
        state["psi"] += delta

    def apply(m, d):
        """Turn key state into a (speed, heading) command. Runs every physics
        step, so the ramps are step-rate independent."""
        now, dt = d.time, m.opt.timestep
        keys.poll(now)
        if keys.confirmed and not announced[0]:
            announced[0] = True     # say so the first time hold really works
            print(f"hold-to-drive active (via {keys.source})")

        if c.mode in ("flick", "flip", "ball", "pivot_rl"):
            # A one-shot move owns the actuators. Track the bike so the
            # heading command doesn't end up 180 deg stale at handoff (which
            # would jam the lead clamp), and leave the speed intent alone —
            # the pivot deliberately sets it for its exit glide.
            state["psi"] = state["psi_sent"] = c._psi
            return

        # Hold detection: true key state once it has actually reported a key
        # down (instant release), else auto-repeat inference with a timed
        # coast. The `confirmed` gate matters: with Input Monitoring denied,
        # Quartz reports every key as up forever, and trusting that would
        # both break holding AND make taps bleed away instantly — strictly
        # worse than the fallback. One real hold flips it on within ~8 ms.
        if keys.available and keys.confirmed:
            ramp_v = ax_v.physical_hold(keys.down("up"), keys.down("down"))
            ramp_psi = ax_psi.physical_hold(keys.down("left"),
                                            keys.down("right"))
            ramp_lat = ax_lat.physical_hold(keys.down("crab_left"),
                                            keys.down("crab_right"))
            coasting = ramp_v == 0
            coasting_lat = ramp_lat == 0
        else:
            ramp_v = ax_v.dir if ax_v.ramping(now) else 0
            ramp_psi = ax_psi.dir if ax_psi.ramping(now) else 0
            ramp_lat = ax_lat.dir if ax_lat.ramping(now) else 0
            coasting = ramp_v == 0 and ax_v.released(now)
            coasting_lat = ramp_lat == 0 and ax_lat.released(now)

        # Longitudinal: throttle builds, the opposite key brakes hard through
        # zero into reverse, and releasing coasts the target back to zero.
        if ramp_v:
            v = state["v"]
            if ramp_v > 0:
                v += (_BRAKE if v < -1e-9 else _ACCEL) * dt
            else:
                v -= (_BRAKE if v > 1e-9 else _ACCEL_REV) * dt
            state["v"] = float(np.clip(v, -v_max, v_max))
        elif coasting:
            v = state["v"]
            state["v"] = float(v - np.sign(v) * min(abs(v), _DECAY * dt))

        # Lateral: same shape as the throttle (build / brake through zero /
        # coast back to zero), but symmetric — left and right are the same
        # maneuver, so there is no _ACCEL_REV equivalent here.
        if ramp_lat:
            vl = state["v_lat"]
            rate = _BRAKE if vl * ramp_lat < -1e-9 else _ACCEL
            state["v_lat"] = float(np.clip(vl + ramp_lat * rate * dt,
                                           -crab_max[0], crab_max[0]))
        elif coasting_lat:
            vl = state["v_lat"]
            state["v_lat"] = float(vl - np.sign(vl) * min(abs(vl), _DECAY * dt))

        # Re-arm the lead clamp once the bike has caught up to within the
        # band. Done here rather than in `turn` so it re-arms while coasting,
        # not only on the next keypress.
        if not lead_armed[0] and abs(lead_now()) <= _LEAD_MAX:
            lead_armed[0] = True

        # Heading: continuous slew while held. No decay — it is a setpoint.
        if ramp_psi:
            turn(ramp_psi * _TURN_RATE * dt)

        if c.mode == "general":
            # Clamp to the envelope the loaded policy declares (yaml
            # v_lat_frac), now that it is definitely loaded.
            crab_max[0] = float(getattr(c._gen, "v_lat_frac", 0.4)) * v_max
            # One velocity VECTOR: magnitude plus course off the commanded
            # heading. Both components zero resolves to (0, 0) -- an ordinary
            # point of command space, not a singularity, which is exactly why
            # the command is a vector and not (course, speed).
            c.set_command_polar(float(np.hypot(state["v"], state["v_lat"])),
                                float(np.arctan2(state["v_lat"], state["v"])),
                                psi_cmd=state["psi"])
        else:
            # command_heading takes a delta, so send only what is new.
            delta = state["psi"] - state["psi_sent"]
            if abs(delta) > 1e-12:
                c.command_heading(d, delta)
                state["psi_sent"] = state["psi"]
            # set_speed snaps out of the reverse instability band, so the
            # commanded arrow can jump there — that is the controller being
            # honest about an unsafe speed, not the teleop stuttering.
            c.set_speed(state["v"])

    def apply_camera(d):
        """Point the viewer camera. Runs on the sim/render thread (step), not
        in the key callback, which the viewer services on its own thread."""
        v = view[0]
        if v is None:
            return
        if cam_mode[0] == "free":
            # Hand the camera back ONCE per switch, re-framed to the 3/4 view
            # the viewer opens with and centred on the bike -- otherwise free
            # mode inherits whatever overhead left behind (elevation -89, i.e.
            # staring straight down) and reads as a broken toggle. After the
            # handoff it is the user's camera again: mouse orbit/pan/zoom are
            # not fought for, which is the whole point of free mode.
            if cam_free_pending[0]:
                cam_free_pending[0] = False
                with v.lock():
                    v.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                    v.cam.trackbodyid = -1
                    v.cam.lookat[:] = d.body("chassis").xpos
                    v.cam.azimuth, v.cam.elevation, v.cam.distance = (
                        135.0, -25.0, 2.4)
            return
        R = d.body("chassis").xmat.reshape(3, 3)
        yaw = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
        with v.lock():
            v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            v.cam.trackbodyid = chassis_id
            # azimuth == yaw puts the camera BEHIND the bike looking along
            # its heading -- verified by reading scn.camera[0].pos back out:
            # at azimuth yaw the offset dots -1.44 with forward, at yaw+180
            # it dots +1.60. yaw+180 therefore filmed the bike head-on, and
            # in plan view pointed it down the screen.
            if cam_mode[0] == "follow":
                v.cam.azimuth, v.cam.elevation, v.cam.distance = (
                    yaw, -18.0, 1.6)
            else:                       # overhead: heading points up-screen
                v.cam.azimuth, v.cam.elevation, v.cam.distance = (
                    yaw, -89.0, 2.6)

    def step(m, d):
        apply_camera(d)
        ensure_mode(d)          # survive viewer resets before reading keys
        while pending:
            k = pending.pop(0)
            general = c.mode == "general"
            # -- the policy menu owns the keyboard while it is open ---------
            # Deliberately swallowing everything, not just the keys it uses:
            # picking a controller and driving are different activities, and
            # a stray throttle tap landing between opening the menu and
            # choosing would be applied to whatever gets loaded next.
            if menu["open"]:
                if k == policy_menu.KEY_MENU:
                    menu["open"] = False
                elif k == policy_menu.KEY_UP:
                    menu["cursor"] = max(0, menu["cursor"] - 1)
                elif k == policy_menu.KEY_DOWN:
                    menu["cursor"] = min(len(menu["entries"]) - 1,
                                         menu["cursor"] + 1)
                elif k == policy_menu.KEY_ENTER:
                    e = menu["entries"][menu["cursor"]]
                    menu["open"] = False
                    select(d, e if isinstance(e, str) else e["name"])
                continue
            if k == policy_menu.KEY_MENU:
                # Rebuilt on open so a policy trained in another terminal
                # since teleop started is listed without a restart.
                menu["entries"] = ([policy_menu.ANALYTIC]
                                   + policy_menu.list_general_policies())
                here = (gen_name[0] if c.mode == "general"
                        else policy_menu.ANALYTIC)
                menu["cursor"] = policy_menu.open_cursor(
                    menu["entries"], here, gen_name[0])
                menu["open"] = True
                continue
            # ',' used to blind-toggle policy <-> analytic right here. The menu
            # opened above replaces it and does strictly more: it names what it
            # is switching to, lists every other policy, and opens the cursor
            # on the OTHER controller so ', ENTER' is the old toggle in two
            # keystrokes. Selecting the analytic entry runs the same
            # command_line + zero_command this branch did — see `select`.
            # In general mode the maneuver keys stay shadowed by teleop
            # functions either way; that modal layer is unchanged.
            if k in (265, 264):     # throttle / brake-reverse
                dirn = 1 if k == 265 else -1
                if ax_v.press(d.time, dirn):    # fresh press -> discrete step
                    v = state["v"]
                    state["v"] = float(np.clip(v + dirn * _STEP_V,
                                               -v_max, v_max))
            elif k in (263, 262):   # turn left / right
                dirn = 1 if k == 263 else -1
                if ax_psi.press(d.time, dirn):
                    turn(dirn * _STEP_PSI)
            # ; ' and \\ specifically: the viewer's own bindings take [ and ]
            # (Cycle cameras), ` (bounding boxes), Esc (free camera), Space,
            # Tab, +/-, F1-F5 and every letter A-Z. These three are what is
            # left. Do not "tidy" them back to brackets -- the keypress lands
            # in both places and the viewer's camera cycles underneath you.
            elif k in (ord(";"), ord("'")):   # trail history length
                trail_level[0] = int(np.clip(
                    trail_level[0] + (1 if k == ord("'") else -1),
                    0, len(trail_levels) - 1))
                lv = trail_levels[trail_level[0]]
                print("trail: " + ("PEN UP (keeps what is drawn)" if lv == 0
                                   else "infinite" if not np.isfinite(lv)
                                   else f"{lv:.0f} s"))
            elif k == ord("/"):     # re-zero the command in either mode
                zero_command(d)
                print("command zeroed")
            elif k == ord("5"):     # stop now (not a coast-down)
                state["v"] = 0.0
                state["v_lat"] = 0.0      # "stop" means all motion, not just
                ax_v.clear()              #   the longitudinal component
                ax_lat.clear()
                if not general:
                    c.command_stop()
            elif k == ord("2"):     # toggle reference overlay
                overlay_on[0] = not overlay_on[0]
            elif k == ord("\\"):    # cycle camera: free -> follow -> overhead
                order = ("free", "follow", "overhead")
                cam_mode[0] = order[(order.index(cam_mode[0]) + 1) % 3]
                cam_free_pending[0] = cam_mode[0] == "free"
                print(f"camera: {cam_mode[0]}")
            # Wing keys sit ABOVE the mode split so they work in both analytic
            # and general mode -- the mechanism is orthogonal to who is
            # balancing. With --wings they shadow 9 (flick_fwd) and 4 (flip),
            # both analytic-only bindings.
            #
            # 9 and 4, NOT 0: the viewer binds 0-9 to geom-group visibility as
            # well, and every geom in this model is group 0, so `0` ghosts the
            # floor and the whole bike out from under you. Groups 4 and 9 are
            # empty here, so toggling them does nothing visible.
            elif wing is not None and k == ord("."):
                # Shove it over, alternating sides so both get tested -- the
                # policy is measurably worse to the left (see
                # docs/plans/self-righting.md part 1).
                wing["push_dir"] = -wing["push_dir"]
                wing["push_n"] = int(wing["push_s"] / m.opt.timestep)
                print(f"SHOVE {'left' if wing['push_dir'] > 0 else 'right'} "
                      f"({wing['push_force']:.0f} N for {wing['push_s']:.2f} s)")
            elif wing is not None and k in (ord("9"), ord("4")):
                if not wing["manual"]:
                    wing["manual"] = True
                    wing["cmd"] = float(d.qpos[wing["jadr"]])   # no snap
                    if c.mode == "general" and getattr(c._gen, "act_wings", False):
                        print("wings: MANUAL override (policy no longer drives "
                              "them; re-engage the policy to hand them back)")
                wing["target"] = wing["deploy"] if k == ord("9") else wing["stow"]
                turns = abs(np.degrees(wing["target"] - wing["stow"])) \
                    * wing["gear"] / 360.0
                print(f"wings {'EXTEND' if k == ord('9') else 'RETRACT'} -> "
                      f"{np.degrees(wing['target']):.0f}° at the wing, "
                      f"{turns:.2f} turns at the servo"
                      + ("  (multi-turn)" if turns > 1.0 else ""))
            elif general:
                # -- general-mode layer: heading snaps replace the moves ----
                if k == ord("6"):
                    turn(np.pi / 2, clamp=False)      # snaps are meant to lead
                elif k == ord("7"):
                    turn(-np.pi / 2, clamp=False)
                elif k == ord("8"):
                    turn(np.pi, clamp=False)
                elif k in (ord("1"), ord("3")):
                    # Crab left / right. General mode only: the analytic
                    # controller has no lateral command at all, and 1/3 stay
                    # bound to the ball shot and flick there.
                    dirn = 1 if k == ord("1") else -1
                    if ax_lat.press(d.time, dirn):   # fresh press -> a step
                        vl = state["v_lat"]
                        state["v_lat"] = float(np.clip(
                            vl + dirn * _STEP_V, -crab_max[0], crab_max[0]))
                elif k == ord("0"):
                    _reset_ball(m, d, params)
            else:
                # -- analytic-mode layer: the maneuver library ---------------
                if k in (ord("6"), ord("7")):   # circle left / right
                    c.command_circle(d, 0.8, +1 if k == ord("6") else -1)
                elif k in (ord("8"), ord("9"), ord("3")):
                    # 8 trajopt reverse-first, 9 trajopt forward-first, 3 RL
                    zero_command(d)
                    move = {ord("8"): "flick", ord("9"): "flick_fwd",
                            ord("3"): "flick_rl"}[k]
                    try:
                        c.command_flick(d, +1, name=move)
                    except FileNotFoundError:
                        print(f"no moves/{move}.yaml yet")
                    except ValueError as e:   # stale RL move (obs spec changed)
                        print(e)
                elif k == ord("1"):     # ball-shot (RL): re-park, then fire
                    zero_command(d)
                    _reset_ball(m, d, params)
                    try:
                        c.command_ball(d, name="ball_rl")
                    except FileNotFoundError:
                        print("no moves/ball_rl.yaml yet — run "
                              "`python -m aow_sim.train_ball_rl`")
                    except ValueError as e:
                        print(e)
                elif k == ord("0"):     # re-park the ball at its start pose
                    _reset_ball(m, d, params)
                elif k == ord("4"):     # crawl front-pivot 180 (in place)
                    zero_command(d)
                    c.command_flip(d, +1)
                elif k == ord("."):     # pivot 180 holding the front's line
                    ve = abs(state["v"])
                    try:
                        c.command_pivot_rl(d, +1, name="pivot_rl", v_end=ve)
                        state["v"] = -ve    # the glide exits backward
                        state["psi"] = state["psi_sent"] = c._psi + np.pi
                    except FileNotFoundError:
                        print("no moves/pivot_rl.yaml yet — run "
                              "`python -m aow_sim.train_pivot_rl`")
                    except ValueError as e:
                        print(e)
        apply(m, d)
        c.step(m, d)
        wing_step(m, d)         # after c.step: it rewrites the whole ctrl vector

    def draw_frame(scn, m, d):
        """Dial + trail, then the policy menu on top.

        Order matters: _overlay resets scn.ngeom, so anything drawn before it
        is discarded. The menu needs the live camera, which only exists once
        the viewer has handed the handle over (on_start), hence the guard."""
        _overlay(scn, m, d, c, overlay_on, v_max, trail=trail, grid=True,
                 trail_level=trail_levels[trail_level[0]])
        if menu["open"] and view[0] is not None:
            active = gen_name[0] if c.mode == "general" else policy_menu.ANALYTIC
            policy_menu.draw(scn, view[0].cam, menu["entries"],
                             menu["cursor"], active)

    # The viewer only ever reports a key going down, so how well "hold" works
    # depends on whether real key state is readable — say which one is live.
    # The general policy is the default driver; fall back to the analytic
    # controller (with a reason) if there is no usable one.
    engage(data)
    mode_help = (
        f"\n  driving moves/{gen_name[0]} — ',' opens the policy menu; the "
        "analytic LQR is\n  its first entry and the cursor starts there, so "
        "', ENTER' is the old\n  toggle. In policy mode 6/7/8 snap the heading "
        "90°L/90°R/180°"
        if c.mode == "general" else
        "\n  , policy menu (no usable general policy — driving the analytic "
        "controller)")

    routes = keys.routes
    if routes:
        hold_help = (
            f"\n  (hold-to-drive via {' + '.join(routes)}; the viewer itself "
            "reports no key releases.\n   If holding does nothing, run "
            "`python -m aow_sim.check_input`. Taps always work.)")
    else:
        import platform
        why = ("install it into THIS interpreter:  pip install -e '.[teleop]'"
               if platform.system() == "Darwin"
               else "no key-state backend exists for this platform")
        hold_help = ("\n  (TAP the arrows to step the command — hold-to-drive "
                     f"needs pyobjc;\n   {why})")
    ball_help = "\n  1 ball-shot (RL)   0 reset ball" if hockey else ""
    wing_help = (
        "\n  WINGS: 9 extend   4 retract   . shove it over (alternates sides)"
        "\n         Manual only — nothing deploys or hands off by itself. "
        "9/4/. shadow the\n         analytic-mode flick_fwd/flip/pivot for "
        "this session. You can also push\n         by hand: double-click the "
        "bike, then Ctrl + right-drag." if wings else "")
    # Number keys + arrows: MuJoCo's viewer binds every letter A-Z (F=force
    # display, etc.), so letters would double up. Number keys 0-9 are free; 4/5
    # toggle (empty) geom groups harmlessly; arrows are free while unpaused.
    teleop_loop(model, data, step, on_key,
                "teleop (number keys — MuJoCo's viewer owns the letters):\n"
                "  ↑/↓ throttle / brake-reverse (release to coast down)\n"
                "  ←/→ hold to turn continuously   / re-zero command   "
                "5 stop   2 overlay\n"
                "  ; / \' trail shorter/longer (pen-up 2s 4s 10s inf)   "
                "\\ camera (free/follow/overhead)\n"
                "  " + policy_menu.label_help() + "\n"
                "  analytic-only keys: 6/7 circle L/R   8/9 flick (trajopt "
                "rev/fwd)   3 flick (RL)\n"
                "                      4 flip   . pivot (RL, front wheel "
                "holds its line)"
                + mode_help + ball_help + wing_help + hold_help,
                "aow_sim.run_drive",
                draw=draw_frame,
                show_ui=show_ui,
                on_start=lambda v: view.__setitem__(0, v))
    return c      # returned so the input model can be driven headlessly


if __name__ == "__main__":
    import mujoco.viewer  # noqa: F401

    main()
