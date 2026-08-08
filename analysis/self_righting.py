"""Falling over and getting back up: rest attitude, side geometry, and what a
single extra XC330 can lift.

Companion to analysis/no_return.py, which established that the bike is lost
long before it looks lost and hits the floor ~0.25 s after the fall becomes
visible. Nothing can be caught in 0.25 s, so the mechanism is a RIGHTING
mechanism, not a catch: it acts after the bike is down. That makes three
questions, one per subcommand.

  profile   Roll-plane energy landscape. Rotate the bike about +X, drop it onto
            the floor at each angle, and plot CoM height vs roll. The minima are
            the attitudes it can rest in, the maxima between them are what a
            tumble has to climb, and the climb from the fallen minimum back to
            upright is the work the arm owes. Purely geometric -- no dynamics,
            no controller, milliseconds per candidate -- so it is the tool to
            sweep side geometry with. `--sweep` does exactly that over
            (half_span, height).

  rest      The same question with dynamics. Real falls, from the no_return
            pulse disturbance, with the chassis lumps made collidable. Reports
            the resting attitude, what is carrying the load, and how repeatable
            it is. This is where the static story gets checked, because a fall
            arrives at the floor with ~1.5 J of rotational energy and the
            barriers in `profile` are a tenth of that.

  lift      The arm. Ramp it into the floor quasi-statically and read the
            actuator force against the XC330's 0.80 N.m, sweeping arm length
            and pivot height for a combination that comes up.

  sequence  The whole thing end to end for the configured geometry: fall,
            settle, right, hand back to the general policy, retract.

Everything reads config/bike_params.yaml's `righting` block, and everything
here is a study of a part that does not exist yet -- see
docs/plans/self-righting.md.

  python analysis/self_righting.py profile
  python analysis/self_righting.py profile --sweep
  python analysis/self_righting.py rest --n 12
  python analysis/self_righting.py lift --sweep
  python analysis/self_righting.py sequence

Read-only apart from the PNGs it writes.
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from aow_sim.build_model import build_model, load_params
from aow_sim.control.balance import extract_state, quat_to_mat
from aow_sim.control.drive import DriveController
from aow_sim.control.linearize import design_all, settle_upright


GRAVITY = 9.81
RECOVER_DEG = 12.0   # hand-back target; inside the general policy's cold
                     # recoverable set on BOTH sides at standstill (see
                     # analysis/no_return.py: 16.3 deg right, 11.8 deg left)


# --------------------------------------------------------------------------
# model plumbing


def variant_params(bumper: bool = True, arm: bool = True, **override) -> dict:
    """A copy of the parameters with parts of the `righting` block removed and
    bumper dimensions optionally overridden -- so a geometry sweep is a dict
    edit and a rebuild, not an edit to the YAML."""
    p = copy.deepcopy(load_params())
    if not bumper:
        p["righting"].pop("bumper", None)
    elif override:
        p["righting"]["bumper"] = {**p["righting"]["bumper"], **override}
    if not arm:
        p["righting"].pop("arm", None)
    return p


def _dynamic_geoms(model) -> list[int]:
    floor = model.geom("floor").id
    return [i for i in range(model.ngeom)
            if model.geom_contype[i] and i != floor]


def _roll_pitch(quat) -> tuple[float, float]:
    R = quat_to_mat(quat)
    return (np.degrees(np.arctan2(R[2, 1], R[2, 2])),
            np.degrees(-np.arcsin(np.clip(R[2, 0], -1.0, 1.0))))


# --------------------------------------------------------------------------
# profile: the roll-plane energy landscape


@dataclass
class Landscape:
    roll: np.ndarray        # [deg]
    com_h: np.ndarray       # [m] CoM height with the bike resting at that roll
    support: list[str]      # which geom is touching
    touch_deg: float        # first roll at which something other than a wheel
                            #   carries it -- must stay clear of riding leans
    rest_deg: float         # the fallen minimum
    rest_h: float
    barrier_j: float        # rest -> the next maximum (resists tumbling on)
    righting_j: float       # rest -> upright (the work the arm owes)


def landscape(params: dict, step_deg: float = 1.0) -> Landscape:
    """CoM height as a function of roll, with the bike dropped onto the floor
    at each angle. Roll-plane only: pitch and yaw are held, so this is the
    design curve and not a prediction of where a tumbling bike stops."""
    model = build_model(params, righting=True)
    data = mujoco.MjData(model)
    floor = model.geom("floor").id
    dyn = _dynamic_geoms(model)
    wheels = {"front_tire"} | {f"roller_{i}_{s}" for i in range(8) for s in "ab"}

    rolls = np.arange(0.0, 180.0 + step_deg, step_deg)
    com_h = np.empty_like(rolls)
    support: list[str] = []
    for k, deg in enumerate(rolls):
        a = np.deg2rad(deg) / 2
        data.qpos[:] = 0.0
        data.qpos[2] = 0.5                      # clear of the floor
        data.qpos[3:7] = [np.cos(a), np.sin(a), 0.0, 0.0]
        mujoco.mj_forward(model, data)
        d = [mujoco.mj_geomDistance(model, data, floor, g, 2.0, None) for g in dyn]
        j = int(np.argmin(d))
        com_h[k] = data.subtree_com[0][2] - d[j]
        support.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, dyn[j]))

    touch = next((rolls[k] for k, s in enumerate(support) if s not in wheels),
                 float("nan"))
    # The fallen rest attitude is the first local minimum past the wheels; the
    # barrier is the next local maximum after it.
    interior = np.arange(1, len(rolls) - 1)
    mins = [k for k in interior if com_h[k] <= com_h[k - 1] and com_h[k] < com_h[k + 1]]
    rest_k = mins[0] if mins else int(np.argmin(com_h))
    after = com_h[rest_k:]
    maxs = [k for k in range(1, len(after) - 1)
            if after[k] >= after[k - 1] and after[k] > after[k + 1]]
    peak = after[maxs[0]] if maxs else float(np.max(after))
    m = float(model.body_mass.sum())
    return Landscape(rolls, com_h, support, float(touch), float(rolls[rest_k]),
                     float(com_h[rest_k]), m * GRAVITY * (peak - com_h[rest_k]),
                     m * GRAVITY * (com_h[0] - com_h[rest_k]))


def cmd_profile(args) -> None:
    if args.sweep:
        print("side-geometry sweep -- capsule rails, one per side, "
              "z measured above the rear axle\n")
        print(f"{'half_span':>9} {'height':>7} {'touch':>7} {'rest':>7} "
              f"{'CoM@rest':>9} {'barrier':>9} {'righting':>9}  support")
        print(f"{'[mm]':>9} {'[mm]':>7} {'[deg]':>7} {'[deg]':>7} "
              f"{'[mm]':>9} {'[mJ]':>9} {'[mJ]':>9}")
        bare = landscape(variant_params(bumper=False, arm=False), args.step)
        print(f"{'none':>9} {'':>7} {bare.touch_deg:>7.0f} {bare.rest_deg:>7.0f} "
              f"{bare.rest_h * 1000:>9.1f} {bare.barrier_j * 1000:>9.0f} "
              f"{bare.righting_j * 1000:>9.0f}  "
              f"{bare.support[int(bare.rest_deg / args.step)]}")
        for span in args.spans:
            for h in args.heights:
                p = variant_params(bumper=True, arm=False,
                                   half_span=span, height=h)
                L = landscape(p, args.step)
                print(f"{span * 1000:>9.0f} {h * 1000:>7.0f} {L.touch_deg:>7.0f} "
                      f"{L.rest_deg:>7.0f} {L.rest_h * 1000:>9.1f} "
                      f"{L.barrier_j * 1000:>9.0f} {L.righting_j * 1000:>9.0f}  "
                      f"{L.support[int(L.rest_deg / args.step)]}")
        print("\n  touch    = roll at which something other than a wheel takes "
              "the load;\n           it has to stay well clear of riding lean "
              "(~10 deg) and of the\n           recoverable set (up to ~31 deg)."
              "\n  barrier  = energy from the rest attitude to the next hump. "
              "A fall arrives\n           with ~1.5 J of roll KE, so this is "
              "about how much has to be\n           dissipated at impact, not "
              "about whether it holds statically."
              "\n  righting = energy from the rest attitude back to upright: "
              "the arm's job.")
        return

    named = [("bare shell", variant_params(bumper=False, arm=False)),
             ("with bumpers", variant_params(bumper=True, arm=False))]
    for name, p in named:
        L = landscape(p, args.step)
        print(f"\n{name}:")
        print(f"  first non-wheel contact  {L.touch_deg:6.1f} deg "
              f"({L.support[int(L.touch_deg / args.step)]})")
        print(f"  fallen rest attitude     {L.rest_deg:6.1f} deg, "
              f"CoM {L.rest_h * 1000:.1f} mm, on "
              f"{L.support[int(L.rest_deg / args.step)]}")
        print(f"  barrier onward           {L.barrier_j * 1000:6.0f} mJ")
        print(f"  righting work            {L.righting_j * 1000:6.0f} mJ "
              f"(CoM {L.com_h[0] * 1000:.1f} mm upright)")
    _plot_profile(args.out, named, args.step)


def _plot_profile(out: Path, named, step) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping the plot")
        return
    fig, ax = plt.subplots(figsize=(7.5, 4.4))
    for (name, p), colour in zip(named, ("#7f7f7f", "#1f77b4")):
        L = landscape(p, step)
        ax.plot(L.roll, L.com_h * 1000, color=colour, label=name)
        ax.plot(L.rest_deg, L.rest_h * 1000, "o", color=colour)
        ax.annotate(f"{L.rest_deg:.0f}°", (L.rest_deg, L.rest_h * 1000),
                    textcoords="offset points", xytext=(4, -12),
                    fontsize=8, color=colour)
    ax.set_xlabel("roll [deg]")
    ax.set_ylabel("CoM height resting on the floor [mm]")
    ax.set_title("roll-plane energy landscape — where a fallen bike settles")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"\nwrote {out}")


# --------------------------------------------------------------------------
# rest: real falls


def _fall_to_rest(params: dict, force: float, dur: float, v0: float,
                  design, settle_s: float = 3.0):
    """Push the upright bike over and let it come to rest. The controller runs
    until the fall is committed and is then cut, which is what the real bike
    would do (a policy sawing the bars on the floor is not a fall test)."""
    model = build_model(params, righting=True)
    data = mujoco.MjData(model)
    eq = settle_upright(model)
    data.qpos[:] = eq.qpos
    data.qvel[:] = 0.0
    data.qvel[0] = v0
    mujoco.mj_forward(model, data)
    ctrl = DriveController(params, model, design=design)
    ctrl.reset(model, data)
    ctrl.profile.v_ref = v0
    ctrl.set_speed(v0)

    dt = model.opt.timestep
    chassis = model.body("chassis").id
    wheels = {"front_tire"} | {f"roller_{i}_{s}" for i in range(8) for s in "ab"}
    n_push = int(round(dur / dt))
    live = True
    ke_touch, roll_touch = float("nan"), float("nan")
    inertia = _roll_inertia(model, mujoco.MjData(model))
    for i in range(int(round((2.0 + settle_s) / dt))):
        data.xfrc_applied[chassis, 1] = force if i < n_push else 0.0
        s = extract_state(data, ctrl._ref_pos)
        if live and abs(s.roll) > np.deg2rad(45.0):
            live = False                        # committed; stop driving
            data.ctrl[:] = 0.0
        if live:
            ctrl.step(model, data)
        else:
            data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        # Energy the side geometry has to swallow: roll KE at the instant the
        # first non-wheel part touches down. This is what the `profile`
        # barrier is competing against, and it is why the barrier and the
        # touchdown angle cannot be traded off independently -- touching down
        # later means touching down faster.
        if np.isnan(ke_touch) and _contact_names(model, data) - wheels:
            ke_touch = 0.5 * float(data.qvel[3]) ** 2 * inertia
            roll_touch = np.degrees(s.roll)
    roll, pitch = _roll_pitch(data.qpos[3:7])
    return {"roll": roll, "pitch": pitch, "yaw_rate": float(data.qvel[5]),
            "settled": bool(np.linalg.norm(data.qvel[3:6]) < 0.05),
            "on": sorted(_contact_names(model, data)),
            "ke_touch": ke_touch, "roll_touch": roll_touch}


def _contact_names(model, data) -> set[str]:
    return {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
            for c in data.contact[:data.ncon]
            for g in (c.geom1, c.geom2)} - {"floor"}


def _roll_inertia(model, data) -> float:
    """Whole-bike inertia about the fore/aft axis through the CoM [kg m^2]."""
    mujoco.mj_forward(model, data)
    com = data.subtree_com[0]
    tot = 0.0
    for b in range(1, model.nbody):
        m = model.body_mass[b]
        if m <= 0:
            continue
        r = data.xipos[b] - com
        tot += m * (r[1] ** 2 + r[2] ** 2) + model.body_inertia[b][0]
    return float(tot)


def cmd_rest(args) -> None:
    cases = [(f, d, v) for f in (4.0, 8.0, -4.0, -8.0)
             for d in (0.25, 0.40) for v in (0.0, 0.6)][:args.n]
    configs = [("bare shell", variant_params(bumper=False, arm=False))]
    if args.spans:
        for span, h in zip(args.spans, args.heights):
            configs.append((f"bumper {span * 1000:.0f}/{h * 1000:.0f}",
                            variant_params(bumper=True, arm=False,
                                           half_span=span, height=h)))
    else:
        configs.append(("configured bumper", variant_params(bumper=True,
                                                            arm=False)))
    for label, p in configs:
        # The gain schedule only depends on the un-fallen bike, and every
        # config here shares it apart from a few grams of bumper.
        design = design_all(p, build_model(p))
        L = landscape(p, 1.0)
        print(f"\n{label} -- roll-plane prediction: rest at {L.rest_deg:.0f} deg, "
              f"barrier {L.barrier_j * 1000:.0f} mJ")
        print(f"  {'push':>7} {'dur':>5} {'v0':>5} | {'touch':>6} {'KE':>7} | "
              f"{'roll':>7} {'pitch':>6}  resting on")
        rolls, kes = [], []
        for f, d, v in cases:
            r = _fall_to_rest(p, f, d, v, design)
            rolls.append(r["roll"])
            kes.append(r["ke_touch"])
            print(f"  {f:>+6.1f}N {d:>5.2f} {v:>5.1f} | {r['roll_touch']:>6.0f} "
                  f"{r['ke_touch'] * 1000:>6.0f}mJ | {r['roll']:>7.1f} "
                  f"{r['pitch']:>6.1f}  {'' if r['settled'] else '(moving) '}"
                  f"{', '.join(r['on'][:4])}")
        a = np.abs(rolls)
        k = np.array(kes)[np.isfinite(kes)]
        print(f"  |roll| at rest: {a.min():.1f}..{a.max():.1f} deg, "
              f"spread {a.max() - a.min():.1f} deg;  touchdown KE "
              f"{k.min() * 1000:.0f}..{k.max() * 1000:.0f} mJ vs a "
              f"{L.barrier_j * 1000:.0f} mJ barrier")


# --------------------------------------------------------------------------
# lift: the arm


def _lift(params: dict, rest_qpos=None, rate: float = 0.7, hold: float = 1.5):
    """Ramp the arm into the floor from the fallen pose and report what it
    costs. `rate` is the commanded arm slew [rad/s] -- slow on purpose, so the
    reading is the quasi-static torque and not an impulse."""
    model = build_model(params, righting=True)
    data = mujoco.MjData(model)
    if rest_qpos is None:
        rest_qpos = _settle_fallen(params)
    data.qpos[:] = rest_qpos
    mujoco.mj_forward(model, data)
    roll0, _ = _roll_pitch(data.qpos[3:7])
    # The hinge axis is body +X and so is roll, so the arm angle and the roll
    # angle have the same handedness: the arm points at the floor when
    # roll + arm ~ 180 deg. Swinging with sign(roll) takes it there, and past
    # 180 deg the foot lands INBOARD of the contact line, which is the side it
    # has to push on to rotate the bike back up rather than further over.
    direction = float(np.sign(roll0)) or 1.0

    aid = model.actuator("righting").id
    jadr = model.joint("righting_joint").qposadr[0]
    dt = model.opt.timestep
    n = int(round((np.pi / rate + hold) / dt))
    cmd = float(data.qpos[jadr])
    peak_tau = 0.0
    best_roll = abs(roll0)
    ts, rolls, taus, cmds = [], [], [], []
    for i in range(n):
        cmd += direction * rate * dt
        cmd = float(np.clip(cmd, -np.pi, np.pi))
        data.ctrl[aid] = cmd
        mujoco.mj_step(model, data)
        roll, _ = _roll_pitch(data.qpos[3:7])
        tau = abs(float(data.actuator_force[aid]))
        peak_tau = max(peak_tau, tau)
        best_roll = min(best_roll, abs(roll))
        if i % 25 == 0:
            ts.append(i * dt)
            rolls.append(roll)
            taus.append(float(data.actuator_force[aid]))
            cmds.append(cmd)
    # Report at the SERVO, not at the arm: the reduction is a design choice
    # and the number that has to be defended is the one on the datasheet.
    gear = params["righting"]["arm"]["gear_ratio"]
    return {"roll0": roll0, "best_roll": best_roll, "peak_tau": peak_tau / gear,
            "t": np.array(ts), "roll": np.array(rolls),
            "tau": np.array(taus) / gear, "cmd": np.array(cmds), "gear": gear,
            "stall": model.actuator_forcerange[aid][1] / gear}


def _settle_fallen(params: dict, roll_deg: float = 100.0, settle: float = 2.0):
    """Drop the bike onto its side and let it stop moving; returns the qpos."""
    model = build_model(params, righting=True)
    data = mujoco.MjData(model)
    base = settle_upright(build_model(params, righting=True))
    data.qpos[:] = base.qpos
    a = np.deg2rad(roll_deg) / 2
    data.qpos[3:7] = [np.cos(a), np.sin(a), 0.0, 0.0]
    data.qpos[2] += 0.02
    mujoco.mj_forward(model, data)
    for _ in range(int(round(settle / model.opt.timestep))):
        mujoco.mj_step(model, data)
    return data.qpos.copy()


def cmd_lift(args) -> None:
    if args.sweep:
        print("arm sweep -- peak actuator torque and the best roll reached\n")
        gear = load_params()["righting"]["arm"]["gear_ratio"]
        print(f"reduction {gear:g}:1 -- torques below are AT THE SERVO\n")
        print(f"{'length':>7} {'pivot z':>8} {'peak tau':>9} {'frac of':>8} "
              f"{'best roll':>10}  verdict")
        print(f"{'[mm]':>7} {'[mm]':>8} {'[N.m]':>9} {'9.9V':>8} {'[deg]':>10}")
        for L in args.lengths:
            for z in args.pivot_z:
                p = variant_params(bumper=True, arm=True)
                p["righting"]["arm"] = {**p["righting"]["arm"], "length": L,
                                        "pivot": [p["righting"]["arm"]["pivot"][0],
                                                  0.0, z]}
                r = _lift(p)
                ok = ("UP" if r["best_roll"] < RECOVER_DEG else
                      "partial" if r["best_roll"] < r["roll0"] - 20 else "no")
                # 9.9 V is the 3S cutoff; the model's servo numbers are the
                # 12 V column, so the worst-case torque is 9.9/12 of stall.
                frac = r["peak_tau"] / (r["stall"] * 9.9 / 12.0)
                print(f"{L * 1000:>7.0f} {z * 1000:>8.0f} {r['peak_tau']:>9.3f} "
                      f"{frac:>8.2f} {r['best_roll']:>10.1f}  {ok}")
        print(f"\n  'UP' = reached |roll| < {RECOVER_DEG:.0f} deg, inside the "
              "general policy's\n  standstill recoverable set on both sides "
              "(analysis/no_return.py).")
        return

    p = variant_params(bumper=True, arm=True)
    r = _lift(p)
    print(f"from rest at {r['roll0']:.1f} deg:")
    print(f"  reduction             {r['gear']:g}:1")
    print(f"  peak servo torque     {r['peak_tau']:.3f} N.m "
          f"(XC330 stall {r['stall']:.2f} N.m at 12 V, 0.76 at 11.1 V, "
          f"{r['stall'] * 9.9 / 12:.2f} at the 9.9 V cutoff)")
    print(f"  best roll reached     {r['best_roll']:.1f} deg "
          f"({'inside' if r['best_roll'] < RECOVER_DEG else 'OUTSIDE'} "
          f"the recoverable set)")
    _plot_lift(args.out, r)


def _plot_lift(out: Path, r) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping the plot")
        return
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.5, 5.2), sharex=True)
    a1.plot(r["t"], r["roll"], color="#1f77b4", label="chassis roll")
    a1.plot(r["t"], np.degrees(r["cmd"]), color="#7f7f7f", lw=0.9,
            label="commanded arm angle")
    a1.axhline(RECOVER_DEG, color="#2ca02c", lw=0.8, ls="--")
    a1.axhline(-RECOVER_DEG, color="#2ca02c", lw=0.8, ls="--",
               label="recoverable set")
    a1.set_ylabel("[deg]")
    a1.legend(fontsize=8)
    a1.grid(alpha=0.3)
    a2.plot(r["t"], r["tau"], color="#d62728")
    for s in (r["stall"], -r["stall"]):
        a2.axhline(s, color="k", lw=0.8, ls="--")
    a2.set_ylabel("arm servo torque [N·m]")
    a2.set_xlabel("time [s]")
    a2.grid(alpha=0.3)
    fig.suptitle("righting stroke (dashed = XC330 stall torque)", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"wrote {out}")


# --------------------------------------------------------------------------


def cmd_sequence(args) -> None:
    """Fall, settle, right, hand back to the general policy, retract."""
    params = variant_params(bumper=True, arm=True)
    model = build_model(params, righting=True)
    design = design_all(params, build_model(params))
    data = mujoco.MjData(model)
    data.qpos[:] = _settle_fallen(params)
    mujoco.mj_forward(model, data)
    roll0, _ = _roll_pitch(data.qpos[3:7])
    aid = model.actuator("righting").id
    jadr = model.joint("righting_joint").qposadr[0]
    dt = model.opt.timestep
    direction = float(np.sign(roll0)) or 1.0

    ctrl = None
    cmd = float(data.qpos[jadr])
    phase = "lift"
    log = []
    t_hand = float("nan")
    for i in range(int(round(args.seconds / dt))):
        roll, _ = _roll_pitch(data.qpos[3:7])
        if phase == "lift":
            cmd += direction * args.rate * dt
            data.ctrl[aid] = float(np.clip(cmd, -np.pi, np.pi))
            if abs(roll) < RECOVER_DEG and abs(data.qvel[3]) < 3.0:
                phase, t_hand = "balance", i * dt
                ctrl = DriveController(params, model, design=design)
                ctrl.reset(model, data)
                ctrl.engage_general(data, args.move)
                ctrl.set_command(v_cmd_world=(0.0, 0.0))
        else:
            ctrl.step(model, data)
            # Retract only once the policy is holding it: pulling the prop out
            # early is the same as never having put it there. Stop at the stow
            # angle rather than sweeping through it and out the other side.
            if i * dt - t_hand > args.retract_after:
                stow = np.deg2rad(params["righting"]["arm"]["stow_deg"])
                step = args.rate * dt
                cmd = (max(cmd - step, stow) if cmd > stow
                       else min(cmd + step, stow))
            data.ctrl[aid] = float(np.clip(cmd, -np.pi, np.pi))
        mujoco.mj_step(model, data)
        if i % 50 == 0:
            log.append((i * dt, roll, np.degrees(cmd),
                        float(data.actuator_force[aid])))
    roll_end, _ = _roll_pitch(data.qpos[3:7])
    print(f"fell to {roll0:.0f} deg; handed over at t = {t_hand:.2f} s; "
          f"final roll {roll_end:.1f} deg, arm at "
          f"{np.degrees(float(data.qpos[jadr])):.0f} deg")
    print("upright and balancing" if abs(roll_end) < 20
          else "DID NOT recover")
    if args.out:
        _plot_sequence(args.out, np.array(log), t_hand)


def _plot_sequence(out: Path, log, t_hand) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, ax = plt.subplots(figsize=(7.5, 4.0))
    ax.plot(log[:, 0], log[:, 1], label="chassis roll [deg]", color="#1f77b4")
    ax.plot(log[:, 0], log[:, 2], label="arm angle [deg]", color="#7f7f7f", lw=0.9)
    ax.axvline(t_hand, color="#2ca02c", ls="--", lw=0.9, label="hand-off")
    ax.set_xlabel("time [s]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("fall → right → hand back to the policy → retract", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"wrote {out}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p1 = sub.add_parser("profile", help="roll-plane energy landscape")
    p1.add_argument("--sweep", action="store_true")
    p1.add_argument("--step", type=float, default=1.0)
    p1.add_argument("--spans", type=float, nargs="+",
                    default=[0.035, 0.045, 0.055, 0.065, 0.075])
    p1.add_argument("--heights", type=float, nargs="+",
                    default=[0.04, 0.06, 0.08, 0.10, 0.12])
    p1.add_argument("--out", type=Path,
                    default=Path(__file__).parent / "righting_profile.png")
    p1.set_defaults(func=cmd_profile)

    p2 = sub.add_parser("rest", help="real falls -> resting attitude")
    p2.add_argument("--n", type=int, default=8)
    p2.add_argument("--spans", type=float, nargs="+", default=[],
                    help="override half_span per candidate; default = the "
                         "geometry in bike_params.yaml")
    p2.add_argument("--heights", type=float, nargs="+", default=[])
    p2.set_defaults(func=cmd_rest)

    p3 = sub.add_parser("lift", help="arm torque and stroke")
    p3.add_argument("--sweep", action="store_true")
    p3.add_argument("--lengths", type=float, nargs="+",
                    default=[0.08, 0.10, 0.12, 0.14])
    p3.add_argument("--pivot-z", type=float, nargs="+",
                    default=[0.03, 0.055, 0.08, 0.105])
    p3.add_argument("--out", type=Path,
                    default=Path(__file__).parent / "righting_lift.png")
    p3.set_defaults(func=cmd_lift)

    p4 = sub.add_parser("sequence", help="fall -> right -> hand off -> retract")
    p4.add_argument("--move", default="general_rl")
    p4.add_argument("--rate", type=float, default=0.7, help="arm slew [rad/s]")
    p4.add_argument("--retract-after", type=float, default=1.0)
    p4.add_argument("--seconds", type=float, default=12.0)
    p4.add_argument("--out", type=Path,
                    default=Path(__file__).parent / "righting_sequence.png")
    p4.set_defaults(func=cmd_sequence)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
