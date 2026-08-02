"""Headless replay of a move with a recorded state trajectory.

    python -m aow_sim.rollout_move flick_rl
    python -m aow_sim.rollout_move flick --direction -1 --out traces/

Replays moves/<name>.yaml through DriveController from a settled standstill
(the same setup as run_drive's scenario tables) and records every physics
step: commanded and measured steer (multi-turn radians), unwrapped chassis
yaw, roll, and longitudinal speed. Prints a summary; with --out writes
<name>_trace.csv and (if matplotlib is importable) <name>_trace.png.

The RL moves are closed-loop policies, so their steer/hub profiles exist
only as *behavior* — unlike the trajopt moves there are no knots in the
yaml to read. This tool is how you look at what a trained policy actually
does with the state.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

from .build_model import build_model, load_params
from .control import DriveController, run
from .control.balance import extract_state
from .control.linearize import settle_upright


def rollout(name: str, direction: int = 1, mirror: bool = False,
            settle_s: float = 1.0, extra_s: float = 4.0,
            v_start: float = 0.0, v_end: float = 0.0) -> dict:
    """Replay moves/<name> headless and return the recorded trace arrays.
    `v_start`/`v_end` apply to pivot moves (glide entry / target exit speed)."""
    params = load_params()
    ball = name.startswith("ball")
    pivot = name.startswith("pivot")
    model = build_model(params, variant="full", hockey=ball)
    eq = settle_upright(model)
    data = mujoco.MjData(model)
    data.qpos[:] = eq.qpos
    a = np.deg2rad(0.5)
    data.qpos[3:7] = [np.cos(a / 2), np.sin(a / 2), 0, 0]
    mujoco.mj_forward(model, data)
    c = DriveController(params, model)
    c.reset(model, data)

    aid = c.aid["steer"]
    sj = c._sj
    trace: dict[str, list] = {k: [] for k in (
        "t", "steer_cmd", "steer_meas", "yaw", "roll", "v_lon", "mode")}

    def rec(dd):
        s = extract_state(dd, c._ref_pos)
        trace["t"].append(dd.time)
        trace["steer_cmd"].append(float(dd.ctrl[aid]))
        trace["steer_meas"].append(float(dd.qpos[sj]))
        trace["yaw"].append(float(c._psi))
        trace["roll"].append(float(s.roll))
        trace["v_lon"].append(float(s.v_lon))
        trace["mode"].append(c.mode)

    run(model, data, c, settle_s, on_step=rec)
    if pivot and v_start > 0:                 # glide up to speed first
        c.set_speed(v_start)
        accel = params["control"]["drive"]["accel"]
        run(model, data, c, v_start / accel + 1.5, on_step=rec)
    t_cmd = data.time
    if ball:
        from .run_drive import _reset_ball
        _reset_ball(model, data, params)
        T = c.command_ball(data, name=name, mirror=mirror)
        move_mode = "ball"
    elif pivot:
        T = c.command_pivot_rl(data, direction, name=name, v_end=v_end)
        move_mode = "pivot_rl"
    else:
        T = c.command_flick(data, direction, name=name)
        move_mode = "flick"
    run(model, data, c, T + extra_s, on_step=rec)

    out = {k: (np.array(v) if k != "mode" else v) for k, v in trace.items()}
    out.update(name=name, horizon=T, t_cmd=t_cmd, move_mode=move_mode)
    # hand-back time: first step after t_cmd where the controller left the move
    idx = [i for i, (t, mo) in enumerate(zip(out["t"], out["mode"]))
           if t > t_cmd and mo != move_mode]
    out["t_handoff"] = float(out["t"][idx[0]]) if idx else None
    return out


COMMAND_SCRIPT = [
    #  (hold [s], speed [m/s], heading step [deg])
    (3.0, 0.0, 0.0),      # hold station
    (3.0, 0.5, 0.0),      # drive off
    (3.0, 0.5, 90.0),     # turn at speed
    (3.0, 0.0, 0.0),      # stop
    (3.0, -0.4, 0.0),     # reverse
    (4.0, 0.5, 180.0),    # about-face at speed
]


def rollout_general(name: str = "general_rl", settle_s: float = 1.0,
                    script=None) -> dict:
    """Drive the always-on general policy through a scripted command sequence
    and record the same trace as `rollout`.

    The general policy is not a move: it has no horizon and never hands back,
    so there is nothing to "replay" — what you inspect is how it *tracks a
    command sequence*. Each entry is (hold [s], speed [m/s], heading step
    [deg]) applied as a STEP change, matching how it was trained."""
    params = load_params()
    model = build_model(params, variant="full")
    eq = settle_upright(model)
    data = mujoco.MjData(model)
    data.qpos[:] = eq.qpos
    a = np.deg2rad(0.5)
    data.qpos[3:7] = [np.cos(a / 2), np.sin(a / 2), 0, 0]
    mujoco.mj_forward(model, data)
    c = DriveController(params, model)
    c.reset(model, data)

    aid, sj = c.aid["steer"], c._sj
    # The command columns are what make a general-policy trace readable: the
    # question is not "what did it do" but "did it track what it was told".
    trace: dict[str, list] = {k: [] for k in (
        "t", "steer_cmd", "steer_meas", "yaw", "yaw_cmd", "roll",
        "v_lon", "v_cmd", "mode")}

    def rec(dd):
        s = extract_state(dd, c._ref_pos)
        trace["t"].append(dd.time)
        trace["steer_cmd"].append(float(dd.ctrl[aid]))
        trace["steer_meas"].append(float(dd.qpos[sj]))
        trace["yaw"].append(float(c._psi))
        trace["roll"].append(float(s.roll))
        trace["v_lon"].append(float(s.v_lon))
        trace["mode"].append(c.mode)
        if c.mode == "general":
            psi_c = float(c._gen_psi_cmd)
            vc = np.asarray(c._gen_v_cmd, float)
            trace["yaw_cmd"].append(psi_c)
            # signed speed along the commanded heading
            trace["v_cmd"].append(float(vc[0] * np.cos(psi_c)
                                        + vc[1] * np.sin(psi_c)))
        else:                       # before engaging: no command yet
            trace["yaw_cmd"].append(float(c._psi))
            trace["v_cmd"].append(0.0)

    run(model, data, c, settle_s, on_step=rec)
    t_cmd = data.time
    c.engage_general(data, name=name)
    marks = []
    for hold, speed, dpsi_deg in (script or COMMAND_SCRIPT):
        c.set_command(dpsi=np.deg2rad(dpsi_deg))
        c.set_command_polar(speed)
        marks.append((data.time, speed, dpsi_deg))
        run(model, data, c, hold, on_step=rec)

    out = {k: (np.array(v) if k != "mode" else v) for k, v in trace.items()}
    out.update(name=name, horizon=float(data.time - t_cmd), t_cmd=t_cmd,
               move_mode="general", t_handoff=None, marks=marks)
    return out


def summarize(tr: dict) -> str:
    t, sm = tr["t"], tr["steer_meas"]
    in_move = t >= tr["t_cmd"]
    yaw0 = tr["yaw"][np.argmax(in_move)]
    if tr["move_mode"] == "general":
        head = (f"policy {tr['name']} (always-on): "
                f"{len(tr['marks'])} commands over {tr['horizon']:.1f} s "
                f"from t={tr['t_cmd']:.2f} s")
    else:
        head = (f"move {tr['name']}: horizon {tr['horizon']:.2f} s, "
                f"commanded at t={tr['t_cmd']:.2f} s, "
                + (f"handed back at t={tr['t_handoff']:.2f} s"
                   if tr["t_handoff"] else "no hand-back (ran to the end)"))
    lines = [
        head,
        f"  steer sweep: {sm[in_move].min():+.2f} .. {sm[in_move].max():+.2f} rad "
        f"({np.degrees(sm[in_move].min()):+.0f} .. "
        f"{np.degrees(sm[in_move].max()):+.0f} deg), "
        f"final park {sm[-1]:+.2f} rad = {sm[-1] / np.pi:+.2f} pi",
        f"  yaw change: {np.degrees(tr['yaw'][-1] - yaw0):+.1f} deg, "
        f"max |roll| {np.degrees(np.max(np.abs(tr['roll']))):.1f} deg, "
        f"peak |v_lon| {np.max(np.abs(tr['v_lon'])):.2f} m/s",
    ]
    return "\n".join(lines)


def write_csv(tr: dict, path: Path) -> None:
    # Columns come from the trace itself: a general-policy rollout carries
    # command channels a move rollout does not.
    skip = {"name", "horizon", "t_cmd", "t_handoff", "move_mode", "marks"}
    cols = [k for k in tr if k not in skip]
    with open(path, "w") as f:
        f.write(",".join(cols) + "\n")
        for i in range(len(tr["t"])):
            f.write(",".join(
                (tr[c][i] if c == "mode" else f"{tr[c][i]:.6f}")
                for c in cols) + "\n")


def _no_plot_hint() -> str:
    """Why the PNG is missing, and the fix — naming the interpreter, because
    matplotlib installed into a *different* env is the usual cause and is
    otherwise invisible."""
    import sys
    return ("  no PNG: matplotlib is not installed in this interpreter\n"
            f"    {sys.executable}\n"
            "    fix with:  pip install -e '.[viz]'   (or '.[dev]')")


def _cmd_branch(cmd, actual):
    """Put a heading command on the 2*pi branch it was actually satisfied on.

    A heading command means nothing beyond mod 2*pi — the policy observes it
    as sin/cos(psi_err) — so a 180 deg snap is deliberately NON-directional
    and turning either way is a correct answer. Plotting the raw unwrapped
    command therefore invents a 360 deg error whenever the bike happens to
    turn the other way.

    Each held command is resolved as one piece, against where the bike ended
    up while holding it. Resolving per-sample instead would be correct but
    unreadable: mid-turn the bike is still nearer the old branch, so the line
    jumps a full turn partway through. The CSV keeps the raw values.
    """
    cmd = np.asarray(cmd, float)
    actual = np.asarray(actual, float)
    out = cmd.copy()
    edges = np.flatnonzero(np.abs(np.diff(cmd)) > 1e-9) + 1
    if edges.size > 500:            # a continuously slewed command: per-sample
        err = np.arctan2(np.sin(cmd - actual), np.cos(cmd - actual))
        return actual + err
    starts = np.concatenate(([0], edges))
    ends = np.concatenate((edges, [cmd.size]))
    for s, e in zip(starts, ends):
        k = np.round((actual[e - 1] - cmd[s]) / (2 * np.pi))
        out[s:e] = cmd[s:e] + k * 2 * np.pi
    return out


def plot(tr: dict, path: Path) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    blue, orange, ink, muted = "#2a78d6", "#eb6834", "#1a1a19", "#8a897f"
    t = tr["t"]
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(9, 9))
    fig.suptitle(f"{tr['name']} rollout", color=ink)

    ax = axes[0]
    lo = np.floor(min(tr["steer_meas"].min(), 0) / np.pi)
    hi = np.ceil(max(tr["steer_meas"].max(), 0) / np.pi)
    for k in np.arange(lo, hi + 0.5, 0.5):
        ax.axhline(k * np.pi, lw=0.5, color=muted, alpha=0.3, zorder=0)
    ax.plot(t, tr["steer_meas"], color=blue, lw=1.6, label="measured (qpos)")
    ax.plot(t, tr["steer_cmd"], color=orange, lw=1.2, ls="--", label="commanded")
    ax.set_ylabel("steer [rad]")
    ax.legend(frameon=False, fontsize=8)

    yaw0 = tr["yaw"][0]
    axes[1].plot(t, np.degrees(tr["yaw"] - yaw0), color=blue, lw=1.6,
                 label="actual")
    if "yaw_cmd" in tr:
        axes[1].plot(t, np.degrees(_cmd_branch(tr["yaw_cmd"], tr["yaw"]) - yaw0),
                     color=orange, lw=1.2, ls="--", label="commanded")
        axes[1].legend(frameon=False, fontsize=8)
    axes[1].set_ylabel("yaw [deg]")
    axes[2].plot(t, np.degrees(tr["roll"]), color=blue, lw=1.6)
    axes[2].set_ylabel("roll [deg]")
    axes[3].plot(t, tr["v_lon"], color=blue, lw=1.6, label="actual")
    if "v_cmd" in tr:
        axes[3].plot(t, tr["v_cmd"], color=orange, lw=1.2, ls="--",
                     label="commanded")
        axes[3].legend(frameon=False, fontsize=8)
    axes[3].set_ylabel("v_lon [m/s]")
    axes[3].set_xlabel("time [s]")

    for ax in axes:
        ax.axvline(tr["t_cmd"], lw=0.8, color=muted, ls=":")
        if tr["t_handoff"]:
            ax.axvline(tr["t_handoff"], lw=0.8, color=muted, ls=":")
        for tm, _sp, _dp in tr.get("marks", []):   # command step changes
            ax.axvline(tm, lw=0.6, color=muted, ls="--", alpha=0.5)
        ax.grid(True, lw=0.4, alpha=0.25)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    axes[0].annotate("engaged" if tr["move_mode"] == "general" else "move start",
                     (tr["t_cmd"], axes[0].get_ylim()[1]),
                     fontsize=7, color=muted, ha="left", va="top")
    if tr["t_handoff"]:
        axes[0].annotate("hand-back", (tr["t_handoff"], axes[0].get_ylim()[1]),
                         fontsize=7, color=muted, ha="left", va="top")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return True


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("name", help="move name (moves/<name>.yaml), e.g. flick_rl")
    ap.add_argument("--direction", type=int, default=1, choices=(1, -1))
    ap.add_argument("--mirror", action="store_true", help="ball moves only")
    ap.add_argument("--settle", type=float, default=1.0, dest="settle_s")
    ap.add_argument("--extra", type=float, default=4.0, dest="extra_s",
                    help="seconds to keep recording after the move horizon")
    ap.add_argument("--v-start", type=float, default=0.0,
                    help="pivot moves: glide up to this speed before the move")
    ap.add_argument("--v-end", type=float, default=0.0,
                    help="pivot moves: target exit speed along the line")
    ap.add_argument("--out", type=Path, default=None,
                    help="directory for <name>_trace.csv/.png (default: none)")
    args = ap.parse_args()

    if args.name.startswith("general"):
        # Always-on policy: drive a scripted command sequence instead of
        # replaying a horizon (there isn't one).
        tr = rollout_general(args.name, args.settle_s)
    else:
        tr = rollout(args.name, args.direction, args.mirror,
                     args.settle_s, args.extra_s, args.v_start, args.v_end)
    print(summarize(tr))
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        csv = args.out / f"{args.name}_trace.csv"
        write_csv(tr, csv)
        print(f"  wrote {csv}")
        png = args.out / f"{args.name}_trace.png"
        if plot(tr, png):
            print(f"  wrote {png}")
        else:
            print(_no_plot_hint())


if __name__ == "__main__":
    main()
