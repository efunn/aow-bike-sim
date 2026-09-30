"""What the steer headset carries while the sim bike balances and drives itself.

The headset friction (bike.steering.headset_friction_*, gearbox_friction)
reads the `headset_force` sensor: what the chassis applies to the steer body,
in a frame with z along the steer axis. This logs that sensor at EVERY physics
step while a policy flies the eval grid -- hold, cruise, reverse, turns,
crabs, spins -- and reports the axial load (the thrust face), the side load
(the bushing), and the friction the model puts on the joint from it.

A FALL IS CUT, not averaged in: from the first step an episode's |roll|
passes --cut-roll-deg, the rest of that episode is dropped (the ground
forces of a bike lying down are not the headset's working load).

    python analysis/headset_load.py                     # teleop's policy, truth sensors
    python analysis/headset_load.py --policy general_rl_odo_ahrs --episode-s 15
    python analysis/headset_load.py --plot              # -> analysis/plots/headset_load.png
    python analysis/headset_load.py --lqr-hold 5        # the analytic LQR standing still

WHAT IT FOUND (2026-09-30). Standing still, the analytic LQR loads the
headset with a flat 3.10 +- 0.03 N axial. The RL policy (teleop's) holds the
same mean, 3.22 N, but swings it 0-10 N at every step: ~30% of the variance
near 12 Hz and ~58% above 60 Hz. That is the policy's action dither shaking
the front end, not the plant; over the whole grid it reaches p99 14.6 N and
lifts the axial load negative on 18% of steps. So the RL numbers measure
the policy's chatter (analysis/chatter.py), and the LQR hold is the
headset's working load.

For scale, the bench (steering-design.md): the bike at rest is ~3.2 N axial
and 0.9 N side; the friction fit spans 0.8-3.5 N axial (86-368 g).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from aow_sim.control.balance import extract_state  # noqa: E402
from aow_sim.control.flick import load_move  # noqa: E402
from aow_sim.control.general_env import GeneralEnv, _load_rl_config  # noqa: E402
from aow_sim.control.general_spec import policy_env_overrides  # noqa: E402
from aow_sim.params import load_params  # noqa: E402
from aow_sim.train_general_rl import eval_cmds  # noqa: E402


def fly(policy: str, episode_s: float, cut_roll_deg: float):
    params = load_params()
    cfg = _load_rl_config(ROOT / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    pol = load_move(policy)
    over = policy_env_overrides(pol)
    env = GeneralEnv(params, {**cfg, "env": {**cfg["env"], **over,
                                             "max_episode_s": float(episode_s)}})
    adr = int(env.model.sensor("headset_force").adr[0])
    st = env._gearbox[0]
    scales = np.array([pol.bounds.steer_rate_max, pol.bounds.hub_max,
                       pol.bounds.diff_max, max(pol.bounds.wing_rate_max, 1e-9)])
    rows, eps = [], []
    for k, (v_lon, v_lat, dpsi) in enumerate(eval_cmds(cfg["env"]["v_max"])):
        obs, _ = env.reset(seed=10_000 + k, options={
            "v_cmd": (v_lon, v_lat), "psi_cmd_rel": dpsi, "difficulty": 1.0})
        buf = []
        inner = st.pre_step

        def logged(d, inner=inner, buf=buf):
            inner(d)
            buf.append((d.time, *d.sensordata[adr:adr + 3],
                        float(env.model.dof_frictionloss[st.dof])))
        st.pre_step = logged
        fell_at = None
        done = False
        while not done:
            a = np.asarray(pol.action(obs), float)
            obs, _r, term, trunc, _ = env.step((a / scales[:len(a)])[:env.action_space.shape[0]])
            roll = abs(np.degrees(extract_state(env.data, env._p0).roll))
            if fell_at is None and roll > cut_roll_deg:
                fell_at = env.data.time
            done = term or trunc
        st.pre_step = inner
        b = np.array(buf)
        keep = b[:, 0] < fell_at if fell_at is not None else np.ones(len(b), bool)
        keep &= b[:, 0] > 0.2                  # drop the settle from the spawn
        cmd = (v_lon, v_lat, round(float(np.degrees(dpsi))))
        eps.append({"cmd": cmd, "fell": fell_at is not None,
                    "kept_s": float(keep.sum()) * env.model.opt.timestep})
        rows.append(np.column_stack([b[keep], np.full(keep.sum(), k)]))
    return np.concatenate(rows), eps, params


def lqr_hold(seconds: float) -> None:
    """The analytic LQR standing still: the headset's load without a
    policy's dither. Axial and its spectrum by band."""
    import mujoco

    from aow_sim.build_model import build_model
    from aow_sim.control.balance import run
    from aow_sim.control.drive import DriveController
    from aow_sim.control.linearize import settle_upright
    p = load_params()
    m = build_model(p, variant="full")
    d = settle_upright(m)
    a = np.deg2rad(0.5)
    d.qpos[3:7] = [np.cos(a / 2), np.sin(a / 2), 0, 0]
    mujoco.mj_forward(m, d)
    c = DriveController(p, m)
    c.reset(m, d)
    adr = int(m.sensor("headset_force").adr[0])
    log = []
    run(m, d, c, seconds, on_step=lambda dd: log.append(dd.sensordata[adr:adr + 3].copy()))
    f = np.array(log)[int(0.2 / m.opt.timestep):]
    ax, side = -f[:, 2], np.hypot(f[:, 0], f[:, 1])
    print(f"LQR hold {seconds:g} s: axial {ax.mean():.2f} +- {ax.std():.2f} N "
          f"({ax.min():.2f}..{ax.max():.2f}), side {side.mean():.2f} N")
    _bands(ax, m.opt.timestep)


def _bands(x, dt) -> None:
    X = x - x.mean()
    f = np.fft.rfftfreq(len(X), dt)
    P = np.abs(np.fft.rfft(X)) ** 2
    print("  variance by band: " + "  ".join(
        f"{lo}-{hi} Hz {P[(f >= lo) & (f < hi)].sum() / P.sum() * 100:.0f}%"
        for lo, hi in ((0, 5), (5, 20), (20, 60), (60, 200), (200, 1250))))


def report(data, eps, params) -> None:
    axial = -data[:, 3]                       # + = pushing up the axis (weight on the wheel)
    side = np.hypot(data[:, 1], data[:, 2])
    fric = data[:, 4] * 1000
    fell = [e for e in eps if e["fell"]]
    print(f"{len(eps)} episodes, {len(fell)} fell (cut from the fall on): "
          + ", ".join(str(e["cmd"]) for e in fell))
    print(f"{data.shape[0]} physics steps kept ({sum(e['kept_s'] for e in eps):.0f} s)\n")
    pct = (0.1, 1, 5, 50, 95, 99, 99.9)
    print(f"{'':22s}" + "".join(f"{f'p{p:g}':>8s}" for p in pct) + f"{'min':>8s}{'max':>8s}")
    for name, x in (("axial [N]", axial), ("side [N]", side),
                    ("joint friction [mN m]", fric)):
        q = np.percentile(x, pct)
        print(f"{name:22s}" + "".join(f"{v:8.2f}" for v in q) + f"{x.min():8.2f}{x.max():8.2f}")
    s = params["bike"]["steering"]
    print(f"\nheadset share of the friction = {s['headset_friction_nm'] * 1000:.2f} + "
          f"{s['headset_friction_per_n'] * 1000:.2f} x |axial| mN m "
          f"(the rest is the gearbox's, which follows the motor torque)")
    print(f"axial < 0 (lifting, the skirt carries it): {np.mean(axial < 0) * 100:.2f}% of steps; "
          f"outside the bench's 0.8-3.5 N: {np.mean((axial < 0.8) | (axial > 3.5)) * 100:.1f}%")
    print(f"side / axial, median: {np.median(side / np.maximum(np.abs(axial), 1e-6)):.2f} "
          f"(bench: tan 15 = 0.27)")


def plot(data, eps, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    axial, side = -data[:, 3], np.hypot(data[:, 1], data[:, 2])
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4), gridspec_kw={"width_ratios": [2, 1]})
    for k in np.unique(data[:, 5]).astype(int)[:4]:
        m = data[:, 5] == k
        a1.plot(data[m, 0], axial[m], lw=0.6, label=str(eps[k]["cmd"]))
    a1.axhspan(0.8, 3.5, color="0.9", zorder=0, label="bench fit range")
    a1.set_xlabel("t [s]")
    a1.set_ylabel("axial load at the headset [N]")
    a1.legend(fontsize=7, title="cmd (v_lon, v_lat, dpsi)", title_fontsize=7)
    a1.grid(alpha=0.3)
    a2.hist(axial, bins=120, alpha=0.7, label="axial", density=True)
    a2.hist(side, bins=120, alpha=0.7, label="side", density=True)
    a2.set_xlabel("N (all kept steps)")
    a2.set_yscale("log")
    a2.legend()
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(f"\nplot -> {out.relative_to(ROOT)}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", default=None,
                    help="move name (default: control.general_move, teleop's)")
    ap.add_argument("--episode-s", type=float, default=15.0)
    ap.add_argument("--cut-roll-deg", type=float, default=30.0)
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--lqr-hold", type=float, default=None, metavar="SECONDS",
                    help="instead: the analytic LQR standing still")
    args = ap.parse_args()
    if args.lqr_hold:
        lqr_hold(args.lqr_hold)
        return
    policy = args.policy or load_params()["control"]["general_move"]
    print(f"policy {policy}, truth sensors, randomization off, {args.episode_s:g} s episodes")
    data, eps, params = fly(policy, args.episode_s, args.cut_roll_deg)
    report(data, eps, params)
    hold = data[data[:, 5] == 0]
    print("hold command only:", end="")
    _bands(-hold[:, 3], float(params["sim"]["timestep"]))
    if args.plot:
        plot(data, eps, ROOT / "analysis" / "plots" / "headset_load.png")


if __name__ == "__main__":
    main()
