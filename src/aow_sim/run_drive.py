"""Drive harness: python -m aow_sim.run_drive [--view | --teleop].

Headless (default):
  1. Straight sprints (fwd/back, config accel): cruise quality, braking
     distance, cross-track, survival.
  2. Accel sweep at v_max until failure -> max clean accel/decel.
  3. Binary-search envelopes (per the target baselines): tightest tracked
     circle and tightest stop-from-circle, both directions, at 0.5 m/s.
  4. Fastest-circle sweep at the tightest radius (+ margin).

--view: scripted demo (sprint, one circle lap, stop).
--teleop (macOS: mjpython): ↑/↓ speed ±0.25 m/s (through zero into reverse),
  ←/→ heading nudge ±15°, C / V circle left/right (R=0.8 m), Space stop.
"""

from __future__ import annotations

import argparse
import copy

import mujoco
import numpy as np

from .build_model import build_model, load_params
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
    args = ap.parse_args()
    params = load_params(args.params)
    model = build_model(params, variant="full", hockey=args.hockey)
    eq = settle_upright(model)

    if args.teleop:
        _teleop(model, params, eq.qpos, hockey=args.hockey)
        return
    if args.view:
        _view_demo(model, params, eq.qpos, hockey=args.hockey)
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


def _view_demo(model, params, eq_qpos, hockey=False):
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
                draw=lambda scn, m, d: _overlay(scn, m, d, c, overlay_on))


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


def _command_ref(c, data):
    """(commanded heading [rad], commanded world velocity [m/s]) for the
    current mode. The general policy carries a full velocity VECTOR (it can
    crab), so read it directly; the analytic modes expose a heading plus a
    speed along it via viz_reference."""
    if c.mode == "general":
        return float(c._gen_psi_cmd), np.asarray(c._gen_v_cmd, float)[:2]
    h, s = c.viz_reference(data)
    return float(h), float(s) * np.array([np.cos(h), np.sin(h)])


def _overlay(scn, model, data, c, on, v_max=1.2):
    """Ground dial under the bike showing the teleop command against reality.

    Headings live ON the rim as radial ticks; velocities are arrows from the
    centre, scaled so the rim means v_max. Keeping the two on different radii
    matters: the commanded course normally points along the commanded
    heading, so drawing both as centre-out rays from the same origin buries
    the (short) velocity arrow inside the (full-length) heading one.

      green tick  = commanded heading      cyan tick   = actual heading
      orange ray  = commanded velocity     yellow ray  = actual velocity
    """
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

    R = data.body("chassis").xmat.reshape(3, 3)
    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    v = np.asarray(data.qvel[:2], float)
    vscale = _VEL_R / max(v_max, 1e-6)      # inner gauge full scale == v_max
    h_cmd, v_cmd = _command_ref(c, data)

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
_MAC_ARROWS = {"left": 123, "right": 124, "down": 125, "up": 126}


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
        codes = {kc: name for name, kc in _MAC_ARROWS.items()}

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
        for name, kc in _MAC_ARROWS.items():
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


def _teleop(model, params, eq_qpos, hockey=False):
    from .interactive import teleop_loop

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
    state = {"v": 0.0, "psi": c._psi, "psi_sent": c._psi, "want_general": True}
    overlay_on = [True]
    ax_v, ax_psi = _Axis(), _Axis()
    keys = _KeyState()
    announced = [False]
    v_max = c.profile.v_max

    def on_key(keycode):
        pending.append(keycode)

    def zero_command(d):
        """Drop every residual command: speed intent, key state, and the
        heading command re-aimed at where the bike actually points. Runs on
        every mode change so a policy never inherits a stale setpoint."""
        state["v"] = 0.0
        state["psi"] = state["psi_sent"] = c._psi
        ax_v.clear()
        ax_psi.clear()
        if c.mode == "general":
            c.set_command_polar(0.0, psi_cmd=c._psi)
        else:
            c.set_speed(0.0)

    def engage(d, quiet=False) -> bool:
        """Hand control to the general policy. Returns False (and clears the
        intent) if there is no usable policy, so the caller falls back to the
        analytic controller instead of retrying every step."""
        try:
            c.engage_general(d, name="general_rl")
        except FileNotFoundError:
            state["want_general"] = False
            if not quiet:
                print("no moves/general_rl.yaml — using the analytic "
                      "controller (train with `python -m "
                      "aow_sim.train_general_rl`)")
            return False
        except ValueError as e:          # stale policy (obs spec changed)
            state["want_general"] = False
            if not quiet:
                print(e)
            return False
        zero_command(d)                  # never inherit a stale setpoint
        return True

    def ensure_mode(d):
        """Re-assert the operator's intent after anything resets the
        controller. The viewer's reset rewinds data.time, which makes
        _Base.step call DriveController.reset -> command_line, silently
        dropping the policy; without this the bike comes back under the
        analytic controller every time you rewind."""
        if state["want_general"] and c.mode != "general":
            engage(d, quiet=True)

    def turn(delta, clamp=True):
        """Move the heading command. `clamp` keeps a *held* turn from winding
        the command past what the bike can follow (which would leave it
        spinning long after release); snaps pass clamp=False because a
        commanded 90/180 is meant to lead."""
        if clamp:
            lead = float(np.arctan2(np.sin(state["psi"] - c._psi),
                                    np.cos(state["psi"] - c._psi)))
            if (delta > 0 and lead >= _LEAD_MAX) or \
               (delta < 0 and lead <= -_LEAD_MAX):
                return
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
            coasting = ramp_v == 0
        else:
            ramp_v = ax_v.dir if ax_v.ramping(now) else 0
            ramp_psi = ax_psi.dir if ax_psi.ramping(now) else 0
            coasting = ramp_v == 0 and ax_v.released(now)

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

        # Heading: continuous slew while held. No decay — it is a setpoint.
        if ramp_psi:
            turn(ramp_psi * _TURN_RATE * dt)

        if c.mode == "general":
            c.set_command_polar(state["v"], psi_cmd=state["psi"])
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

    def step(m, d):
        ensure_mode(d)          # survive viewer resets before reading keys
        while pending:
            k = pending.pop(0)
            general = c.mode == "general"
            if k == ord(","):
                # Mode toggle. In general mode the maneuver keys are shadowed
                # by teleop functions (the policy owns the actuators), so this
                # is a real modal layer, not just an extra binding.
                if general:
                    state["want_general"] = False
                    c.command_line(d)
                    zero_command(d)
                    print("general policy OFF — analytic controller "
                          "(maneuver keys live again)")
                else:
                    state["want_general"] = True
                    if engage(d):
                        print("general policy ON (command zeroed):\n"
                              "  ↑/↓ throttle/brake   ←/→ hold to turn\n"
                              "  6/7 snap 90° L/R   8 snap 180°   5 stop   "
                              "/ re-zero   , off")
            elif k in (265, 264):   # throttle / brake-reverse
                dirn = 1 if k == 265 else -1
                if ax_v.press(d.time, dirn):    # fresh press -> discrete step
                    v = state["v"]
                    state["v"] = float(np.clip(v + dirn * _STEP_V,
                                               -v_max, v_max))
            elif k in (263, 262):   # turn left / right
                dirn = 1 if k == 263 else -1
                if ax_psi.press(d.time, dirn):
                    turn(dirn * _STEP_PSI)
            elif k == ord("/"):     # re-zero the command in either mode
                zero_command(d)
                print("command zeroed")
            elif k == ord("5"):     # stop now (not a coast-down)
                state["v"] = 0.0
                ax_v.clear()
                if not general:
                    c.command_stop()
            elif k == ord("2"):     # toggle reference overlay
                overlay_on[0] = not overlay_on[0]
            elif general:
                # -- general-mode layer: heading snaps replace the moves ----
                if k == ord("6"):
                    turn(np.pi / 2, clamp=False)      # snaps are meant to lead
                elif k == ord("7"):
                    turn(-np.pi / 2, clamp=False)
                elif k == ord("8"):
                    turn(np.pi, clamp=False)
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

    # The viewer only ever reports a key going down, so how well "hold" works
    # depends on whether real key state is readable — say which one is live.
    # The general policy is the default driver; fall back to the analytic
    # controller (with a reason) if there is no usable one.
    engage(data)
    mode_help = (
        "\n  general RL policy is ON (default) — ',' switches to the analytic\n"
        "  controller and back; in policy mode 6/7/8 snap the heading "
        "90°L/90°R/180°"
        if c.mode == "general" else
        "\n  , engage the general RL policy (unavailable — analytic control)")

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
    # Number keys + arrows: MuJoCo's viewer binds every letter A-Z (F=force
    # display, etc.), so letters would double up. Number keys 0-9 are free; 4/5
    # toggle (empty) geom groups harmlessly; arrows are free while unpaused.
    teleop_loop(model, data, step, on_key,
                "teleop (number keys — MuJoCo's viewer owns the letters):\n"
                "  ↑/↓ throttle / brake-reverse (release to coast down)\n"
                "  ←/→ hold to turn continuously   / re-zero command   "
                "5 stop   2 overlay\n"
                "  analytic-only keys: 6/7 circle L/R   8/9 flick (trajopt "
                "rev/fwd)   3 flick (RL)\n"
                "                      4 flip   . pivot (RL, front wheel "
                "holds its line)"
                + mode_help + ball_help + hold_help,
                "aow_sim.run_drive",
                draw=lambda scn, m, d: _overlay(scn, m, d, c, overlay_on,
                                                v_max))
    return c      # returned so the input model can be driven headlessly


if __name__ == "__main__":
    import mujoco.viewer  # noqa: F401

    main()
