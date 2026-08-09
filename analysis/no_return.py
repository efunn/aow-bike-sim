"""Where the point of no return is: the recoverable set in the roll phase plane.

A two-wheeler has NO statically stable roll region -- the contact patch is a
line, so the CoM is over the support at exactly one roll angle. "Point of no
return" is therefore not a tipping angle. It is a *boundary in the (roll, roll
rate) phase plane*, and it moves with forward speed, because a rolling bike can
steer under its own CoM while a standing one can only crawl the rear omni
sideways.

Two measurements, because they answer two different questions.

  COLD (--method cold, or both)
      For a grid of (roll rate, speed, side), bisect the largest initial roll
      angle from which the controller recovers, starting each rollout from the
      settled straight-rolling state with that tilt and rate imposed and
      everything else at equilibrium.
      This is the boundary a FALL DETECTOR can use: it is a curve in the two
      signals the AHRS actually reports. It is also conservative -- a real fall
      arrives at a given (roll, rate) with the steer already deflected and the
      rear already crawling, and that extra state is worth several degrees.

  PULSE (--method pulse, or both)
      Shove the upright bike with a lateral force pulse and bisect the pulse
      DURATION at fixed force. The longest pulse still recovered ends at the
      last savable state; the shortest pulse that is lost ends just past it.
      Nothing is imposed and nothing is frozen, so this is the honest point of
      no return, and the state at pulse-end is a point ON the true boundary
      with realistic controller state.
      It also gives the number the self-righting mechanism is designed around:
      the time from that instant until the bike is flat.

Recovery is judged on roll alone (|roll| < 20 deg for the last second of a 4 s
rollout, having never exceeded 75 deg). Position drift is deliberately NOT a
criterion: catching a big lean costs meters of ground, and a controller that
saves the bike into the next room has still saved the bike.

Left and right are swept separately. The model is mirror-symmetric but a
trained policy is not (see mirror_equivariance.py), so a one-sided number would
be an average of two different bikes.

  python analysis/no_return.py                       # general_rl, 3 speeds
  python analysis/no_return.py --controller lqr
  python analysis/no_return.py --method pulse --speeds 0
  python analysis/no_return.py --csv /tmp/nr.csv --out /tmp/nr.png

Read-only: loads moves/*.npz, writes one PNG (+ optional CSV).
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np

from aow_sim.build_model import build_model, load_params
from aow_sim.control.balance import extract_state, lat_gain
from aow_sim.control.drive import DriveController
from aow_sim.control.linearize import design_all, settle_rolling


FALL_DEG = 75.0        # committed: past here the bike is going down
FLAT_DEG = 90.0        # chassis horizontal (true rest angle needs side geometry)
RECOVER_DEG = 20.0     # tail-window ceiling that counts as "caught it"
ONSET_DEG = 10.0       # last time inside this = start of the terminal divergence
HORIZON_S = 6.0        # rollout length for the recovery test
TAIL_S = 1.0           # trailing window the recovery test looks at
TRACE_DT = 0.005       # trace sample interval [s]
BISECT_DEG = 0.25      # resolution of the cold boundary search
COARSE_DEG = 3.0       # coarse scan step before bisecting
BISECT_MS = 2.0        # resolution of the pulse-duration search
PULSE_MAX_S = 0.6      # longest pulse the search will consider


# --------------------------------------------------------------------------
# worker state: one MuJoCo model + one LQR design per process (~3 s to build)

_W: dict = {}


def _init(params, controller, move, speeds):
    model = build_model(params, variant="full")
    _W["params"] = params
    _W["model"] = model
    _W["design"] = design_all(params, model)
    _W["controller"] = controller
    _W["move"] = move
    # settle_rolling is a half-second sim per speed; cache the snapshots.
    _W["eq"] = {}
    for v in speeds:
        eq = settle_rolling(model, params, float(v))
        _W["eq"][float(v)] = (eq.qpos.copy(), eq.qvel.copy(), eq.ctrl.copy())


def _make_data(v0: float, roll: float, roll_rate: float,
               steer: float | None = None) -> mujoco.MjData:
    """Settled straight-rolling state at speed v0, tilted by `roll` about the
    body +X axis with body-frame roll rate `roll_rate`.

    The tilt is about the chassis origin, which is the rear axle centre, so the
    omni wheel stays tangent to the floor at any roll -- same convention as
    run_balance._tilted_data, which is what makes these angles comparable to
    the tilt-recovery number that harness prints."""
    model = _W["model"]
    qpos, qvel, ctrl = _W["eq"][float(v0)]
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    data.qvel[:] = qvel
    data.ctrl[:] = ctrl
    data.qpos[3:7] = [np.cos(roll / 2), np.sin(roll / 2), 0.0, 0.0]
    data.qvel[3] = roll_rate      # freejoint angular velocity is body-frame
    # Optional steer offset [rad], for asking whether the recoverable set
    # depends on where the steer is parked. The settled snapshot above is a
    # STRAIGHT-rolling equilibrium, so a non-zero steer here is deliberately
    # off-equilibrium — same status as the roll tilt it sits alongside: "the
    # bike arrives in this state, can it recover?" Not an equilibrium sweep.
    if steer is not None:
        data.qpos[model.joint("steer_joint").qposadr[0]] = steer
    mujoco.mj_forward(model, data)
    return data


def _make_controller(data, v0: float):
    p, model = _W["params"], _W["model"]
    ctrl = DriveController(p, model, design=_W["design"])
    ctrl.reset(model, data)
    if _W["controller"] == "general":
        ctrl.engage_general(data, _W["move"])
        # Hold what was being asked for before the disturbance: keep rolling
        # at v0 along the current heading. Commanding a stop instead would
        # measure a different (and easier) problem.
        ctrl.set_command(v_cmd_world=(v0, 0.0), psi_cmd=0.0)
    else:
        ctrl.profile.v_ref = v0
        ctrl.set_speed(v0)
    return ctrl


@dataclass
class Trace:
    t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    roll: np.ndarray = field(default_factory=lambda: np.zeros(0))   # [deg]
    rate: np.ndarray = field(default_factory=lambda: np.zeros(0))   # [rad/s]
    v_lat: np.ndarray = field(default_factory=lambda: np.zeros(0))  # [m/s]

    def at(self, t: float) -> tuple[float, float, float]:
        i = min(int(np.searchsorted(self.t, t)), len(self.t) - 1)
        return float(self.roll[i]), float(self.rate[i]), float(self.v_lat[i])

    def time_to(self, deg: float) -> float:
        hit = np.flatnonzero(np.abs(self.roll) > deg)
        return float(self.t[hit[0]]) if hit.size else float("nan")

    def rate_at_roll(self, deg: float) -> float:
        hit = np.flatnonzero(np.abs(self.roll) > deg)
        return abs(float(self.rate[hit[0]])) if hit.size else float("nan")

    def onset(self) -> float:
        """Start of the terminal divergence: the last instant the trace is
        still inside ONSET_DEG. Losing recoverability and visibly falling are
        different events -- the LQR limps near upright for seconds after it is
        already lost -- so the descent budget has to be measured from here and
        not from the disturbance."""
        inside = np.flatnonzero(np.abs(self.roll) < ONSET_DEG)
        return float(self.t[inside[-1]]) if inside.size else float(self.t[0])

    @property
    def fall_sign(self) -> int:
        """+1 if the trace ends leaning right, -1 left. Traces are truncated at
        the stop angle, so the last sample is the direction of the fall."""
        return int(np.sign(self.roll[-1])) or 1


def _rollout(v0: float, roll: float = 0.0, roll_rate: float = 0.0,
             horizon: float = HORIZON_S, push: tuple[float, float] | None = None,
             stop_deg: float = FALL_DEG, steer: float | None = None):
    """Simulate one disturbance. Returns (recovered, Trace).

    `push` = (force_N, duration_s) applies a lateral force pulse at the chassis
    CoM from t = 0. The rollout ends early once |roll| exceeds `stop_deg`; the
    trace is truncated there, so its sign is always the direction of the fall
    and never a post-tumble artefact."""
    model = _W["model"]
    data = _make_data(v0, roll, roll_rate, steer)
    ctrl = _make_controller(data, v0)
    chassis = model.body("chassis").id
    dt = model.opt.timestep
    n = int(round(horizon / dt))
    n_push = int(round(push[1] / dt)) if push else 0
    every = max(1, int(round(TRACE_DT / dt)))

    n_tr = n // every + 2
    ts, rolls, rates, vlats = (np.empty(n_tr) for _ in range(4))
    k = 0
    fell = False
    tail_max = 0.0
    tail_start = n - int(round(TAIL_S / dt))
    for i in range(n):
        if push:
            data.xfrc_applied[chassis, 1] = push[0] if i < n_push else 0.0
        ctrl.step(model, data)
        mujoco.mj_step(model, data)
        s = extract_state(data, ctrl._ref_pos)
        if not np.isfinite(s.roll):
            fell = True
            break
        # Sample on the grid AND on the last step, so time_to(stop_deg) always
        # has a sample past the threshold to find.
        gone = abs(s.roll) > np.deg2rad(stop_deg)
        if (i % every == 0 or gone) and k < n_tr:
            ts[k], rolls[k] = i * dt, np.degrees(s.roll)
            rates[k], vlats[k] = s.roll_rate, s.v_lat
            k += 1
        if gone:
            fell = True
            break
        if i >= tail_start:
            tail_max = max(tail_max, abs(s.roll))
    recovered = (not fell) and tail_max < np.deg2rad(RECOVER_DEG)
    return recovered, Trace(ts[:k], rolls[:k], rates[:k], vlats[:k])


# --------------------------------------------------------------------------
# method COLD: boundary from imposed initial conditions


@dataclass
class ColdPoint:
    speed: float
    side: int          # +1 = lean right (roll > 0), -1 = lean left
    rate: float        # initial roll rate [rad/s], toward the fall
    roll_crit: float   # largest roll [deg] recovered before the first failure
    roll_max_ok: float # largest roll [deg] recovered anywhere in the scan


def _cold_point(job) -> ColdPoint:
    """Coarse scan upward to the first failure, then bisect.

    Two numbers on purpose. `roll_crit` is the edge of the contiguous
    recoverable region -- the honest "guaranteed" boundary. `roll_max_ok` is
    the largest angle that recovered anywhere in the scan; when it exceeds
    roll_crit there is a recovery island above the boundary (the controller
    happens to catch a steeper lean by a route it cannot find reliably), which
    is worth knowing about and not worth trusting."""
    v0, side, rate = job
    lo, hi = 0.0, None
    roll_max_ok = 0.0
    theta = COARSE_DEG
    ceiling = 75.0
    while theta <= ceiling:
        ok, _ = _rollout(v0, side * np.deg2rad(theta), side * rate)
        if ok:
            roll_max_ok = max(roll_max_ok, theta)
            if hi is None:
                lo = theta
        elif hi is None:
            hi = theta
            ceiling = min(ceiling, theta + 4 * COARSE_DEG)  # island scan only
        theta += COARSE_DEG
    if hi is None:
        return ColdPoint(v0, side, rate, ceiling, roll_max_ok)
    while hi - lo > BISECT_DEG:
        mid = 0.5 * (lo + hi)
        ok, _ = _rollout(v0, side * np.deg2rad(mid), side * rate)
        lo, hi = (mid, hi) if ok else (lo, mid)
    return ColdPoint(v0, side, rate, lo, roll_max_ok)


# --------------------------------------------------------------------------
# method PULSE: point of no return along a real fall


@dataclass
class PulsePoint:
    """One (speed, force) pair. All roll quantities are FALL-SIGNED: positive
    is toward the side the bike actually went, so the tables read the same for
    a left and a right push. `side` records which way that was."""
    speed: float
    force: float       # signed lateral pulse [N]
    side: int = 1      # +1 the bike fell right, -1 left
    t_nr: float = float("nan")     # longest pulse still recovered [s]
    roll_nr: float = float("nan")  # roll at that instant [deg]
    rate_nr: float = float("nan")  # roll rate at that instant [rad/s]
    v_catch: float = float("nan")  # base speed in the CATCH direction [m/s]:
                                   #   positive = the contact is already
                                   #   travelling back under the CoM
    t_lost: float = float("nan")   # shortest pulse that was lost [s]
    latency: float = float("nan")  # t_lost -> the fall becoming visible [s]
    t_flat: float = float("nan")   # onset -> FLAT_DEG [s]
    peak_rate: float = float("nan")  # max |roll rate| during the fall [rad/s]
    descent: dict = field(default_factory=dict)  # roll_deg -> (t_after_onset, rate)
    ok: bool = True    # False if even PULSE_MAX_S never knocked it over


# Roll angles the descent schedule is reported at: a fall detector has to fire
# at one of them, and what it buys is the time left in that row.
DESCENT_DEG = (30.0, 45.0, 60.0, 75.0, 90.0)


def _pulse_point(job) -> PulsePoint:
    """Bisect the pulse DURATION at fixed force: the longest pulse the
    controller still walks away from ends at the last savable state."""
    v0, force = job
    ok_max, _ = _rollout(v0, push=(force, PULSE_MAX_S))
    if ok_max:
        return PulsePoint(v0, force, ok=False)
    lo, hi = 0.0, PULSE_MAX_S
    res = BISECT_MS / 1000.0
    while hi - lo > res:
        mid = 0.5 * (lo + hi)
        ok, _ = _rollout(v0, push=(force, mid))
        lo, hi = (mid, hi) if ok else (lo, mid)
    # Re-run the two bracketing pulses to read the state at pulse-end, and
    # follow the lost one all the way down for the timing budget.
    _, tr_ok = _rollout(v0, push=(force, lo))
    _, tr_lost = _rollout(v0, push=(force, hi), horizon=HORIZON_S + 4.0,
                          stop_deg=FLAT_DEG)
    sd = tr_lost.fall_sign
    roll_nr, rate_nr, vlat_nr = tr_ok.at(lo)
    t0 = tr_lost.onset()
    descent = {d: (tr_lost.time_to(d) - t0, tr_lost.rate_at_roll(d))
               for d in DESCENT_DEG}
    return PulsePoint(
        # roll > 0 leans right, i.e. the top goes toward -Y, so catching a
        # right-hand fall means driving the contact toward -Y. v_catch is the
        # base velocity on that direction, positive = already helping.
        v0, force, sd, lo, sd * roll_nr, sd * rate_nr, -sd * vlat_nr,
        hi, t0 - hi, tr_lost.time_to(FLAT_DEG) - t0,
        float(np.max(np.abs(tr_lost.rate))), descent)


def _crawl_ceiling(params: dict) -> float:
    """Kinematic ceiling on rear-contact lateral speed [m/s]: both drives at
    opposite no-load speed. The real ceiling is lower (torque limits, and any
    common mode eats into it), so this is an upper bound on the only balance
    authority a standing bike has."""
    servo = params["servos"]["xc430_w150"]
    w_max = servo["no_load_rpm"] * 2 * np.pi / 60 * params["drivetrain"]["belt_ratio"]
    return abs(lat_gain(params)) * 2 * w_max


def _fall_trace(job):
    """One representative fall, traced for the phase-plane plot."""
    v0, force, dur = job
    ok, tr = _rollout(v0, push=(force, dur), horizon=3.0, stop_deg=FLAT_DEG)
    return v0, force, ok, tr


# --------------------------------------------------------------------------


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--controller", choices=["general", "lqr"], default="general")
    ap.add_argument("--move", default="general_rl",
                    help="moves/<name> for --controller general")
    ap.add_argument("--method", choices=["cold", "pulse", "both"], default="both")
    ap.add_argument("--speeds", type=float, nargs="+", default=[0.0, 0.4, 0.8])
    ap.add_argument("--rates", type=float, nargs="+",
                    default=[-1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                    help="initial roll rate [rad/s], toward the fall")
    ap.add_argument("--forces", type=float, nargs="+", default=[2.0, 4.0, 8.0],
                    help="lateral pulse magnitudes [N] for the pulse method")
    ap.add_argument("--jobs", type=int, default=0, help="0 = cpu_count")
    ap.add_argument("--csv", type=Path, default=None)
    ap.add_argument("--out", type=Path,
                    default=Path(__file__).parent / "no_return.png")
    args = ap.parse_args()

    params = load_params()
    speeds = [float(v) for v in args.speeds]
    label = args.move if args.controller == "general" else "lqr"
    do_cold = args.method in ("cold", "both")
    do_pulse = args.method in ("pulse", "both")

    cold_jobs = ([(v, side, r) for v in speeds for side in (1, -1)
                  for r in args.rates] if do_cold else [])
    pulse_jobs = ([(v, s * f) for v in speeds for s in (1, -1)
                   for f in args.forces] if do_pulse else [])

    with Pool(args.jobs or None, initializer=_init,
              initargs=(params, args.controller, args.move, speeds)) as pool:
        colds = pool.map(_cold_point, cold_jobs)
        pulses = pool.map(_pulse_point, pulse_jobs)
        # One traced fall per (speed, side): the mid force, pushed well past
        # its no-return duration so the plot shows a committed fall.
        trace_jobs = [(p.speed, p.force, min(PULSE_MAX_S, p.t_lost * 1.5))
                      for p in pulses if p.ok and abs(p.force) == args.forces[
                          len(args.forces) // 2]]
        falls = pool.map(_fall_trace, trace_jobs)

    print(f"controller: {label}   recovery = |roll| < {RECOVER_DEG:.0f} deg over "
          f"the last {TAIL_S:.0f} s of {HORIZON_S:.0f} s\n")

    grids: dict[tuple[float, int], tuple[np.ndarray, np.ndarray]] = {}
    if do_cold:
        print("COLD -- largest initial lean [deg] recovered, by imposed roll rate")
        head = "  ".join(f"{r:>6.1f}" for r in args.rates)
        print(f"{'speed':>6} {'side':>6}  {head}")
        for v in speeds:
            for side in (1, -1):
                row = sorted((p for p in colds if p.speed == v and p.side == side),
                             key=lambda p: p.rate)
                crit = np.array([p.roll_crit for p in row])
                grids[(v, side)] = (np.array([p.rate for p in row]), crit)
                cells = "  ".join(f"{c:>6.1f}" for c in crit)
                print(f"{v:>6.2f} {'right' if side > 0 else 'left':>6}  {cells}")
        islands = [p for p in colds if p.roll_max_ok > p.roll_crit + COARSE_DEG]
        if islands:
            print(f"\n  {len(islands)} recovery islands above the boundary "
                  "(caught a steeper lean, not contiguously):")
            for p in islands[:8]:
                print(f"    v={p.speed:.2f} {'R' if p.side > 0 else 'L'} "
                      f"rate={p.rate:+.1f}: boundary {p.roll_crit:.1f}, "
                      f"also recovered at {p.roll_max_ok:.1f} deg")
        print()

    if do_pulse:
        print("PULSE -- lateral force pulse from upright, duration bisected.")
        print("  'no return' = state at the end of the longest pulse still "
              "recovered;\n  'lost' = end of the shortest that was not. "
              "Roll signs are toward the fall.")
        print(f"{'speed':>6} {'force':>7} {'fell':>5} | {'t_nr':>6} {'roll':>6} "
              f"{'rate':>6} {'v_catch':>8} | {'latency':>8} {'t_flat':>7} "
              f"{'peak':>6}")
        for p in sorted(pulses, key=lambda p: (p.speed, -p.force)):
            if not p.ok:
                print(f"{p.speed:>6.2f} {p.force:>+6.1f}N  never knocked over "
                      f"by a {PULSE_MAX_S * 1000:.0f} ms pulse")
                continue
            print(f"{p.speed:>6.2f} {p.force:>+6.1f}N {'R' if p.side > 0 else 'L':>5} "
                  f"| {p.t_nr:>6.3f} {p.roll_nr:>6.1f} {p.rate_nr:>6.2f} "
                  f"{p.v_catch:>8.2f} | {p.latency:>8.2f} {p.t_flat:>7.2f} "
                  f"{p.peak_rate:>6.1f}")
        got = [p for p in pulses if p.ok]
        if got:
            lat = [p.latency for p in got]
            print(f"\n  latency, no-return -> the fall becoming visible "
                  f"(|roll| > {ONSET_DEG:.0f} deg): "
                  f"{min(lat):.2f}..{max(lat):.2f} s, median {np.median(lat):.2f} s")
            print(f"  no-return state spans roll "
                  f"{min(p.roll_nr for p in got):.1f}..{max(p.roll_nr for p in got):.1f} deg, "
                  f"rate {min(p.rate_nr for p in got):.2f}..{max(p.rate_nr for p in got):.2f} rad/s, "
                  f"v_catch {min(p.v_catch for p in got):.2f}..{max(p.v_catch for p in got):.2f} m/s "
                  f"(crawl ceiling {_crawl_ceiling(params):.2f} m/s)")
            print(f"\n  DESCENT, timed from the onset of the fall (not from "
                  f"the disturbance) --\n  what a detector firing at each roll "
                  f"angle has left, over {len(got)} falls:")
            print(f"    {'roll':>6} {'t since onset [s]':>26} "
                  f"{'roll rate there [rad/s]':>25}")
            print(f"    {'':>6} {'min':>8} {'med':>8} {'max':>8} "
                  f"{'min':>8} {'med':>8} {'max':>8}")
            for d in DESCENT_DEG:
                ts = np.array([p.descent[d][0] for p in got])
                rs = np.array([p.descent[d][1] for p in got])
                ts, rs = ts[np.isfinite(ts)], rs[np.isfinite(rs)]
                if not ts.size:
                    continue
                print(f"    {d:>5.0f}° {ts.min():>8.3f} {np.median(ts):>8.3f} "
                      f"{ts.max():>8.3f} {rs.min():>8.1f} {np.median(rs):>8.1f} "
                      f"{rs.max():>8.1f}")
        print()

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["method", "controller", "speed", "side", "force_N",
                        "rate_or_t_nr", "roll_deg", "rate_rad_s", "v_catch_m_s",
                        "t_flat_s", "roll_max_ok_deg"])
            for p in colds:
                w.writerow(["cold", label, p.speed, p.side, "", p.rate,
                            round(p.roll_crit, 3), "", "", "",
                            round(p.roll_max_ok, 3)])
            for p in pulses:
                w.writerow(["pulse", label, p.speed, p.side, p.force,
                            round(p.t_nr, 4), round(p.roll_nr, 3),
                            round(p.rate_nr, 3), round(p.v_catch, 3),
                            round(p.t_flat, 4), ""])
        print(f"wrote {args.csv}")

    _plot(args.out, label, speeds, grids, pulses, falls)


def _plot(out: Path, label, speeds, grids, pulses, falls) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping the plot")
        return

    fig, axes = plt.subplots(1, len(speeds), figsize=(4.6 * len(speeds), 4.4),
                             sharex=True, sharey=True, squeeze=False)
    for ax, v in zip(axes[0], speeds):
        for side, colour in ((1, "#1f77b4"), (-1, "#d62728")):
            if (v, side) not in grids:
                continue
            rate, crit = grids[(v, side)]
            ax.plot(side * crit, side * rate, "-o", ms=3, color=colour,
                    label=f"cold boundary, {'right' if side > 0 else 'left'}")
            ax.fill_betweenx(side * rate, 0, side * crit, color=colour, alpha=0.10)
        for _v, _f, ok, tr in falls:
            if _v != v:
                continue
            ax.plot(tr.roll, tr.rate, lw=0.9, alpha=0.8,
                    color="#2ca02c" if ok else "#7f7f7f")
        pts = [p for p in pulses if p.speed == v and p.ok]
        if pts:
            ax.plot([p.side * p.roll_nr for p in pts],
                    [p.side * p.rate_nr for p in pts], "k*",
                    ms=10, zorder=5, label="point of no return (pulse)")
        ax.axhline(0, color="k", lw=0.5)
        ax.axvline(0, color="k", lw=0.5)
        ax.set_title(f"v = {v:.2f} m/s")
        ax.set_xlabel("roll [deg]")
        ax.grid(alpha=0.25)
        # Zoomed on the recoverable set; the fall traces run off the panel on
        # their way to the floor, which is the point.
        ax.set_xlim(-45, 45)
        ax.set_ylim(-8, 10)
    axes[0][0].set_ylabel("roll rate [rad/s]")
    axes[0][0].legend(fontsize=7, loc="upper left")
    fig.suptitle(f"recoverable set and falls — {label}   "
                 "(grey trace = committed fall, running off-panel to the floor)",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
