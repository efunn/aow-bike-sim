"""One onboard run beside the same policy in the sim: time series and spectra.

    python analysis/policy/run_vs_sim.py traces/bike/<stamp>_run
    python analysis/policy/run_vs_sim.py <run> --from 30 --to 78   # pick a window
    python analysis/policy/run_vs_sim.py <run> --zoom 3            # a 3 s detail
    python analysis/policy/run_vs_sim.py <run> --no-plot \
        --set drivetrain_model.servo.command_delay_s=0.012         # one sim variant

`--set PATH=VALUE` (repeatable) overrides one parameter of the SIM's bike,
applied after the move's own `drivetrain_model` block, so it reaches the
detailed drivetrain the policy flies on. It is the sweep tool: which change
makes the sim's spectra look like the bike's.

The hardware side is the run's longest engaged stretch (or --from/--to, in
the record's own clock), minus half a second at each end so the engage and
the hand-off are not in it. The sim side flies the run's own policy standing
on a ZERO command, on its own sensors (the move yaml's AHRS level and
odometry, the detailed drivetrain), the way test_policy_endurance does -- so
it is the reference for a no-input run, and only roughly one for a run you
drove. The steer RATE it observes is the Pi's (differenced counts,
`SimSteerRate`), whatever the policy trained on, because that is what the
bike sends; `--steer-rate joint` gives the exact one instead.

Every signal is the one the controller had: the AHRS gyro and the
odometry's speed estimate, not the truth, in the sim as on the bike. The
steer target is `ctrl[steer]`, the steer position the encoder's. Columns:
the bike, the sim (same y-limits), and both power spectra on one axis.

Writes run_vs_sim.png INTO the run directory (traces/ is gitignored, synced)
unless --out is given, and prints the summary table: std, the RMS left
after a 250 ms and a 50 ms moving average (">2 Hz", ">10 Hz"), the RMS in
one band (`--band`, default 7-12 Hz, where the bike's ~9 Hz shake is), and
the spectrum's peak between 2 and 20 Hz. A peak frequency only names the
biggest bump: the steer's sits at 2-3 Hz on the bike and still carries a
9 Hz one 23x the sim's (run 174349), so read the band column.

Records are 100 Hz, so the spectra stop at 50 Hz and anything faster folds
back below it: a peak near 45-50 Hz may be a faster one aliased.
"""
from __future__ import annotations

import argparse
import warnings
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch

FS = 100.0
# (label, unit) in plot order; each side fills the same keys.
SIGNALS = [
    ("roll", "deg"), ("roll rate (gyro x)", "deg/s"), ("pitch rate (gyro y)", "deg/s"),
    ("yaw rate (gyro z)", "deg/s"), ("steer target", "deg"), ("steer (encoder)", "deg"),
    # NB the two steer rows are plotted about their own mean: the fork is
    # unlimited, so a run sits whole turns away from the sim's zero.
    ("drive A cmd", "rad/s"), ("drive B cmd", "rad/s"), ("speed estimate", "m/s"),
]


def hp_rms(x, n):
    """RMS of what a centred n-sample moving average leaves."""
    k = np.ones(n) / n
    return float(np.sqrt(np.mean((x - np.convolve(x, k, "same"))[n:-n] ** 2)))


def peak_hz(x, lo=2.0, hi=20.0):
    """Frequency of the largest Welch peak between lo and hi Hz."""
    f, p = welch(x - x.mean(), fs=FS, nperseg=min(512, len(x)))
    band = (f >= lo) & (f <= hi)
    return float(f[band][np.argmax(p[band])])


def band_rms(x, lo, hi):
    """RMS of x in the [lo, hi] Hz band, from its Welch spectrum."""
    f, p = welch(x - x.mean(), fs=FS, nperseg=min(512, len(x)))
    m = (f >= lo) & (f <= hi)
    return float(np.sqrt(np.trapezoid(p[m], f[m])))


def hardware(run_dir, t_from, t_to):
    from aow_sim.hw.bench_log import Capture
    c = Capture.load(run_dir)
    t, st = c.bike("t"), c.bike("state").astype(int)
    if t_from is None:
        eng = np.flatnonzero(st == 0)
        if not len(eng):
            raise SystemExit(f"{run_dir}: never engaged")
        segs = np.split(eng, np.flatnonzero(np.diff(eng) > 1) + 1)
        seg = max(segs, key=len)
        t_from, t_to = t[seg[0]] + 0.5, t[seg[-1]] - 0.5
    sel = (t >= t_from) & (t <= t_to) & (st == 0)
    g, ctrl = np.degrees(c.bike("gyro")[sel]), c.bike("ctrl")[sel]
    sig = {
        "roll": np.degrees(c.bike("roll")[sel]),
        "roll rate (gyro x)": g[:, 0], "pitch rate (gyro y)": g[:, 1],
        "yaw rate (gyro z)": g[:, 2],
        "steer target": np.degrees(ctrl[:, 2]),
        "steer (encoder)": np.degrees(c.bike("steer")[sel]),
        "drive A cmd": ctrl[:, 0], "drive B cmd": ctrl[:, 1],
        "speed estimate": np.hypot(*c.bike("v_world")[sel].T),
    }
    cmd = np.abs(c.bike("cmd_v_world")[sel]).max()
    return sig, c.meta, (t_from, t_to), cmd


def _set(params, path, value):
    """params with one dotted PATH replaced; every dict on the way copied."""
    head, _, rest = path.partition(".")
    if head not in params and rest:
        raise SystemExit(f"--set {path}: no key {head!r}")
    # a new LEAF is allowed: an option the model reads with a default
    # (drivetrain_model.detent.hill_nm) has no entry in the config
    out = dict(params)
    out[head] = _set(params[head], rest, value) if rest else value
    return out


def sim(policy, seconds, seed, overrides=(), steer_rate="counts", on_step=None):
    """The run's policy standing on a zero command, sampled at FS.

    `on_step(model, data, ctl, drive)` runs after every physics step (drivetrain_slowmo.py
    renders from it), so a caller sees exactly this loop rather than a copy."""
    warnings.filterwarnings("ignore")
    import mujoco
    import yaml
    from aow_sim import drivetrain_model, gearbox_friction
    from aow_sim.build_model import build_model, load_params
    from aow_sim.control.drive import DriveController
    from aow_sim.control.flick import MOVES_DIR
    from aow_sim.control.linearize import design_all, settle_upright
    from aow_sim.sim_ahrs import TAU_ORIENT_S, SimAhrs
    from contextlib import ExitStack

    import yaml as _yaml
    from aow_sim.sim_odometry import SimOdometry, SimSteerRate

    params = load_params()
    spec = yaml.safe_load((MOVES_DIR / f"{policy}.yaml").read_text()) or {}
    design = None
    if spec.get(drivetrain_model.KEY):
        design = design_all(params, build_model(params))
        params = {**params, drivetrain_model.KEY: spec[drivetrain_model.KEY]}
    if spec.get("wheel_play"):          # the play it trained with; --set still wins
        params = {**params, "wheel_play": spec["wheel_play"]}
    for path, value in overrides:
        params = _set(params, path, _yaml.safe_load(value))
    model = build_model(params)
    dt = model.opt.timestep
    ahrs = odo = None
    if str(spec.get("ahrs_level") or "none") != "none":
        ahrs = SimAhrs(model, params, level=spec["ahrs_level"], seed=seed,
                       tau_orient_s=float(spec.get("ahrs_tau_s", TAU_ORIENT_S)))
    if spec.get("obs_odometry"):
        odo = SimOdometry(model, params, mode="front",
                          encoder=str(spec.get("odometry_encoder") or "ideal"),
                          ahrs=ahrs)
    data = mujoco.MjData(model)
    data.qpos[:] = settle_upright(model).qpos
    mujoco.mj_forward(model, data)
    ctl = DriveController(params, model, design=design)
    ctl.reset(model, data)
    ctl._odometry_active, ctl._ahrs_active = odo is not None, ahrs is not None
    drive = drivetrain_model.DrivetrainSim.attach(model, params)
    ctl._drivetrain_active = drive is not None
    if drive is not None:
        drive.reset(data, np.random.default_rng(seed))
    ctl.engage_general(data, name=policy)
    ctl.set_command_polar(0.0)
    steer_est = SimSteerRate(model, params, encoder=steer_rate)
    # The steer's gearbox and headset friction, as GeneralEnv installs it
    # (a per-substep hook, not part of the model).
    gearbox = gearbox_friction.attach_native(model, params)
    aid = {n: model.actuator(n).id for n in ("drive_a", "drive_b", "steer")}
    every = int(round(1 / (FS * dt)))
    rows = []

    def sample():
        # Inside the estimator's context: what the controller sees.
        q = data.qpos[3:7]
        roll = np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]),
                          1 - 2 * (q[1] ** 2 + q[2] ** 2))
        rows.append((np.degrees(roll), *np.degrees(data.qvel[3:6]),
                     np.degrees(data.ctrl[aid["steer"]]),
                     np.degrees(data.qpos[ctl._sj]),
                     data.ctrl[aid["drive_a"]], data.ctrl[aid["drive_b"]],
                     np.hypot(*data.qvel[:2])))

    for k in range(int(seconds / dt)):
        steer_ctx = ExitStack()
        if steer_rate == "counts":
            # obs[5] reads qvel[steer]; physics keeps the truth (as teleop)
            steer_ctx.enter_context(steer_est.estimated(data, dt))
        steer_ctx.__enter__()
        if odo is not None:
            with odo.estimated(data, dt):
                ctl.step(model, data)
                if k % every == 0:
                    sample()
        elif ahrs is not None:
            ahrs.tick(data, dt)
            with ahrs.estimated(data):
                ctl.step(model, data)
                if k % every == 0:
                    sample()
        else:
            ctl.step(model, data)
            if k % every == 0:
                sample()
        steer_ctx.close()
        if drive is not None:
            drive.pre_step(data)
        for g in gearbox:
            g.pre_step(data)
        mujoco.mj_step(model, data)
        if on_step is not None:
            on_step(model, data, ctl, drive)
    a = np.array(rows)[int(FS):]                 # drop the first second
    return {lab: a[:, i] for i, (lab, _) in enumerate(SIGNALS)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run", type=Path, help="traces/bike/<stamp>_run")
    ap.add_argument("--from", dest="t_from", type=float, default=None)
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    ap.add_argument("--zoom", type=float, default=0.0, metavar="S",
                    help="plot only the first S seconds of each side "
                         "(spectra and table still use the whole window)")
    ap.add_argument("--sim-seconds", type=float, default=0.0,
                    help="sim length (default: the hardware window's, +1 s)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--set", dest="overrides", action="append", default=[],
                    metavar="PATH=VALUE",
                    help="override one sim parameter, e.g. "
                         "drivetrain_model.roller_slop.half_play_deg=12")
    ap.add_argument("--steer-rate", choices=("counts", "joint"),
                    default="counts",
                    help="the steer rate the sim's policy observes (default "
                         "counts, the Pi's)")
    ap.add_argument("--band", default="7-12", metavar="LO-HI",
                    help="the band the table's band-RMS column covers [Hz]")
    ap.add_argument("--no-plot", action="store_true",
                    help="the table only (for sweeps)")
    a = ap.parse_args()

    hw, meta, (t0, t1), cmd = hardware(a.run, a.t_from, a.t_to)
    policy = meta["policy"]
    from aow_sim.params import load_params, plant_digest
    if meta.get("plant_digest") != plant_digest(load_params()):
        print(f"NOTE: the run's plant digest {meta.get('plant_digest')} is not "
              f"today's {plant_digest(load_params())}; the sim is today's plant")
    if cmd > 0:
        print(f"NOTE: the run was commanded (|v| up to {cmd:.2f} m/s); the sim "
              "stands on zero, so it is only a rough reference")
    secs = a.sim_seconds or min(60.0, (t1 - t0) + 1.0)
    print(f"{a.run.name}: {policy}, hardware {t0:.1f}-{t1:.1f} s; sim {secs:.0f} s ...")
    overrides = []
    for o in a.overrides:
        path, eq, value = o.partition("=")
        if not eq:
            ap.error(f"--set wants PATH=VALUE, got {o!r}")
        overrides.append((path, value))
    if overrides:
        print("sim: " + ", ".join(f"{p}={v}" for p, v in overrides))
    sm = sim(policy, secs, a.seed, overrides, a.steer_rate)

    print(f"{'signal':22s} {'unit':6s} {'std bike':>9s} {'sim':>7s} | "
          f"{'>2Hz bike':>9s} {'sim':>7s} | {'>10Hz bike':>10s} {'sim':>7s} | "
          f"{a.band + ' Hz bike':>12s} {'sim':>7s} | "
          f"{'peak Hz bike':>12s} {'sim':>5s}")
    lo, hi = (float(v) for v in a.band.split("-"))
    for lab, unit in SIGNALS:
        h, s = hw[lab], sm[lab]
        print(f"{lab:22s} {unit:6s} {np.std(h):9.3f} {np.std(s):7.3f} | "
              f"{hp_rms(h, 25):9.3f} {hp_rms(s, 25):7.3f} | "
              f"{hp_rms(h, 5):10.3f} {hp_rms(s, 5):7.3f} | "
              f"{band_rms(h, lo, hi):12.3f} {band_rms(s, lo, hi):7.3f} | "
              f"{peak_hz(h):12.1f} {peak_hz(s):5.1f}")
    if a.no_plot:
        return

    n = len(SIGNALS)
    fig, axes = plt.subplots(n, 3, figsize=(16, 2.0 * n), sharex="col",
                             gridspec_kw={"width_ratios": [2.2, 2.2, 1.4]})
    zoom = int(a.zoom * FS) if a.zoom else None
    for r, (lab, unit) in enumerate(SIGNALS):
        h, s = hw[lab], sm[lab]
        if lab.startswith("steer"):
            h, s = h - h.mean(), s - s.mean()
            unit += ", about mean"
        hz, sz = h[:zoom], s[:zoom]
        lo = min(hz.min(), sz.min())
        hi = max(hz.max(), sz.max())
        pad = 0.05 * (hi - lo or 1.0)
        for c, (x, col) in enumerate(((hz, "tab:red"), (sz, "tab:blue"))):
            ax = axes[r, c]
            ax.plot(np.arange(len(x)) / FS, x, lw=0.6, color=col)
            ax.set_ylim(lo - pad, hi + pad)
            ax.grid(alpha=0.3)
        axes[r, 0].set_ylabel(f"{lab}\n[{unit}]", fontsize=8)
        ax = axes[r, 2]
        for x, col, name in ((h, "tab:red", "bike"), (s, "tab:blue", "sim")):
            f, p = welch(x - x.mean(), fs=FS, nperseg=min(256, len(x)))
            ax.semilogy(f, p, lw=0.8, color=col, label=name)
        ax.grid(alpha=0.3, which="both")
        if r == 0:
            ax.legend(fontsize=8)
    axes[0, 0].set_title(f"bike: {a.run.name}, {t0:.1f}-{t1:.1f} s", fontsize=9)
    axes[0, 1].set_title(f"sim: {policy}, zero command, seed {a.seed}", fontsize=9)
    axes[0, 2].set_title("power spectrum (Welch)", fontsize=9)
    axes[-1, 0].set_xlabel("s from window start")
    axes[-1, 1].set_xlabel("s from start (first second dropped)")
    axes[-1, 2].set_xlabel("Hz")
    fig.tight_layout()
    out = a.out or a.run / "run_vs_sim.png"
    fig.savefig(out, dpi=110)
    print(f"-> {out}")


if __name__ == "__main__":
    main()
