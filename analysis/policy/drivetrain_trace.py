"""Static figure: roll, roller angle and roller slop, old drivetrain vs new.

    python analysis/policy/drivetrain_trace.py \
        --new traces/drivetrain/drivetrain_sawtooth_1024.yaml
    python analysis/policy/drivetrain_trace.py --new <overlay> --start 5 --seconds 2

The still companion to drivetrain_slowmo.py: the same two flights
(run_vs_sim.sim() through its `on_step` hook -- the policy on its own
training drivetrain, then with `--new`'s `detent` and `roller_slop`
swapped in), one column each, three rows:

  roll       chassis roll angle [deg].
  (page only) steer: the steer joint [deg] and its target, the policy's
             steer ctrl, dashed; yaw: the chassis heading from its
             quaternion [deg], unwrapped, about the run's median (sim: the
             free joint, bike: the AHRS quaternion as run_bike had it).
  roller     every roller's spin angle, wrapped to ONE ROLLER REVOLUTION,
             in roller degrees. The black line is where the rollers would
             be with no slop, k_roller x ring-vs-hub (`ring_spin` is
             ring-vs-hub; each slop tendon is roller_i - k_roller ring).
             The solid lines are the detent carried into roller angle, one
             per 7.5 deg of ring = 18 deg of roller, at this run's drawn
             phase. What a line MEANS depends on the detent: a friction
             bump (the training drivetrain) is where the shaft sticks --
             48 % of the floor twin's rests fall within 1 deg of it; a
             sawtooth flip pushes away from it (19 %), and its midway line
             (dashed) is where the push stops, though with friction as big
             as the push the rests barely favour it (26 %; 27 % is even).
             The roller carrying the bike is drawn coloured where it touches.
             Dashed blue: where the drive is COMMANDED to -- in the sim the
             firmware's own position reference (its integral of the velocity
             command); on the bike the same reference recovered sample by
             sample from Present PWM through the firmware law (see
             bike_trace). A shaft
             parked on a line with the command beyond it is held there.
  slop       roller_i - k_roller x ring-vs-hub [roller deg]: how far each
             roller sits inside its play, the loaded one coloured, the
             ends of the play dashed. A loaded roller LAGGING the others
             is this row.

Same seed both columns, NOT the same trajectory. Writes
traces/drivetrain_slowmo/<policy><tag>_trace.png (traces/ is gitignored).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run_vs_sim as R  # noqa: E402

from aow_sim import drivetrain_model as dm  # noqa: E402
from aow_sim.build_model import load_params  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
N = 8
INK, MUTED, GRID = "#1f2328", "#8b929c", "#d9dde3"
LOADED = "#d4572a"          # the roller on the floor
IDEAL = "#1f2328"
TOP, TROUGH = "#b8bec7", "#d5dae0"
CMD = "#2a7bd4"             # where the drive is commanded to


class Tracer:
    """on_step hook: every physics step in [start, start + seconds)."""

    def __init__(self, start, seconds):
        self.t0, self.t1 = start, start + seconds
        self.rows, self.model = [], None

    def __call__(self, model, data, ctl, drive=None):
        if self.model is None:
            self.model, self.drive = model, drive
            self.q_ring = model.joint("ring_spin").qposadr[0]
            self.q_rol = [model.joint(f"roller_spin_{i}").qposadr[0] for i in range(N)]
            body = {int(model.jnt_bodyid[model.joint(f"roller_spin_{i}").id]): i
                    for i in range(N)}
            self.geom_roller = {g: body[int(model.geom_bodyid[g])]
                                for g in range(model.ngeom)
                                if int(model.geom_bodyid[g]) in body}
            self.floor = model.geom("floor").id
            self.q_steer = model.joint("steer_joint").qposadr[0]
            self.a_steer = model.actuator("steer").id
        t = data.time
        if not (self.t0 <= t < self.t1):
            return
        on = -1
        for c in data.contact[:data.ncon]:
            if self.floor in (c.geom1, c.geom2):
                g = c.geom2 if c.geom1 == self.floor else c.geom1
                on = self.geom_roller.get(int(g), on)
        w, x, y, z = data.qpos[3:7]
        roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        # The firmware's position reference: its integral of the velocity
        # command (servo rad, anti-windup included), as input-shaft ring-vs-hub.
        d = self.drive
        ref = (d.belt * (d.ga * d._ref[0] + d.gb * d._ref[1])
               if d is not None and d.servo else np.nan)
        self.rows.append((t, roll, data.qpos[self.q_ring],
                          *(data.qpos[q] for q in self.q_rol), on, ref,
                          data.qpos[self.q_steer], data.ctrl[self.a_steer],
                          math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


def fly(policy, overrides, start, seconds, seed):
    tr = Tracer(start, seconds)
    R.sim(policy, start + seconds + 0.01, seed, overrides, on_step=tr)
    a = np.array(tr.rows)
    return tr, {"t": a[:, 0], "roll": a[:, 1], "ring": a[:, 2],
                "roller": a[:, 3:3 + N], "on": a[:, 3 + N].astype(int),
                "ref": a[:, 4 + N], "steer": np.degrees(a[:, 5 + N]),
                "steer_cmd": np.degrees(a[:, 6 + N]), "yaw": _yaw_about_median(a[:, 7 + N])}


def _breaks(y, jump=180.0):
    """NaN where a wrapped series jumps a revolution, so no line crosses."""
    y = y.copy()
    y[1:][np.abs(np.diff(y)) > jump] = np.nan
    return y


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--new", type=Path, required=True, metavar="OVERLAY")
    ap.add_argument("--parts", nargs="+", default=["detent", "roller_slop"], choices=dm.PARTS)
    ap.add_argument("--policy", default=None)
    ap.add_argument("--set", dest="overrides", action="append", default=[],
                    metavar="PATH=VALUE", help="both columns, as run_vs_sim --set")
    ap.add_argument("--start", type=float, default=5.0)
    ap.add_argument("--seconds", type=float, default=1.5)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--bike-run", type=Path, default=REPO / "traces" / "bike" / "261024-174349_run",
                    help="a floor run for the third column")
    ap.add_argument("--bike-from", type=float, default=None,
                    help="bike window start [s] (default: mid longest engage)")
    ap.add_argument("--bench-creep", default="261024-221037_creep_film",
                    help="the hung creep whose jumps place the detent in the "
                         "encoder frame")
    ap.add_argument("--page", action="store_true",
                    help="instead of the figure: write analysis/policy/plots/"
                         "drivetrain_traces.html, a static page with each --page-runs "
                         "bike run beside its policy's floor sim on the training "
                         "drivetrain and on --new, picked by dropdown")
    ap.add_argument("--page-runs", nargs="+", type=Path,
                    default=[REPO / "traces" / "bike" / r for r in
                             ("261024-174349_run", "261024-191558_run")])
    ap.add_argument("--tag", default="")
    ap.add_argument("--out-dir", type=Path, default=REPO / "traces" / "drivetrain_slowmo")
    a = ap.parse_args()

    import yaml
    params = load_params()
    policy = a.policy or params["control"]["general_move"]
    k = float(params["drivetrain"]["k_roller"])
    shared = [tuple(o.split("=", 1)) for o in a.overrides]
    new = dm.load(a.new)
    swap = [(f"{dm.KEY}.{p}", yaml.safe_dump(new[p])) for p in a.parts]
    cols = [("its training drivetrain", shared),
            (f"{'+'.join(a.parts)} from {a.new.stem}", shared + swap)]
    if a.page:
        build_page(a, k, cols)
        return
    runs = [fly(policy, ov, a.start, a.seconds, a.seed) for _, ov in cols]
    bench = bench_detent(a.bench_creep)
    bike = bike_trace(a.bike_run, bench, a.bike_from, a.seconds)

    fig, axs = plt.subplots(3, 3, figsize=(21, 11), sharex="col",
                            gridspec_kw={"height_ratios": [1, 2.4, 1.3]})
    roll_lim = max([np.abs(r["roll"]).max() for _, r in runs]
                   + [np.abs(bike["roll"]).max()]) * 1.1
    for c, ((title, _), (tr, r)) in enumerate(zip(cols, runs)):
        t = r["t"]
        ax = axs[0, c]
        ax.plot(t, r["roll"], color=INK, lw=1.4)
        ax.set_ylim(-roll_lim, roll_lim)
        ax.set_title(f"sim: {policy}\n{title}", fontsize=11, color=INK, loc="left")

        ax = axs[1, c]
        drv = tr.drive
        if drv is not None and drv.detent:
            # the detent's top (friction peak, sawtooth flip) at ring =
            # phase + period/2 + n period; its trough (the rest) midway
            _detent_lines(ax, math.degrees(drv.phase + 0.5 * drv.period), k,
                          sawtooth=bool(drv.hill and drv.hill_saw))
        ideal = np.degrees(k * r["ring"])
        rol = np.degrees(r["roller"])
        for i in range(N):
            ax.plot(t, _breaks(rol[:, i] % 360), color=MUTED, lw=0.6, alpha=0.7)
        ax.plot(t, _breaks(ideal % 360), color=IDEAL, lw=1.6)
        ax.plot(t, _breaks(np.degrees(k * r["ref"]) % 360), color=CMD, lw=1.3, ls=(0, (4, 2)))
        on = r["on"]
        loaded = np.where(on >= 0, rol[np.arange(len(t)), np.clip(on, 0, N - 1)], np.nan)
        y = loaded % 360
        y[on < 0] = np.nan
        ax.plot(t, _breaks(y), color=LOADED, lw=2.4)
        ax.set_ylim(0, 360)
        ax.set_yticks(np.arange(0, 361, 45))

        ax = axs[2, c]
        slop = rol - ideal[:, None]
        for i in range(N):
            ax.plot(t, slop[:, i], color=MUTED, lw=0.6, alpha=0.7)
        sl = np.where(on >= 0, slop[np.arange(len(t)), np.clip(on, 0, N - 1)], np.nan)
        ax.plot(t, sl, color=LOADED, lw=2.0)
        half = math.degrees(float(tr.model.tendon_range[
            tr.model.tendon("slop_roller_spin_0").id][1])) if _has_slop(tr.model) else None
        if half:
            for s_ in (-half, half):
                ax.axhline(s_, color=MUTED, lw=1, ls="--")
        ax.set_xlabel("sim time [s]")

    # the bike: its own encoders, the detent where the hung creep put it
    ax = axs[0, 2]
    ax.plot(bike["tb"], bike["roll"], color=INK, lw=1.4)
    ax.set_ylim(-roll_lim, roll_lim)
    ax.set_title(f"bike: {a.bike_run.name} ({bike['policy']})\ndetent from the hung creep "
                 f"{a.bench_creep}", fontsize=11, color=INK, loc="left")
    ax = axs[1, 2]
    _detent_lines(ax, bench["flip"], k)
    ax.plot(bike["t"], _breaks((k * bike["phi"]) % 360), color=IDEAL, lw=1.6)
    ax.plot(bike["t"], _breaks((k * bike["cmd"]) % 360), color=CMD, lw=1.3, ls=(0, (4, 2)))
    ax.set_ylim(0, 360)
    ax.set_yticks(np.arange(0, 361, 45))
    ax = axs[2, 2]
    axs[2, 2].remove()
    ax = fig.add_subplot(3, 3, 9)
    notch = 7.5 * k
    bins = np.linspace(0, notch, 25)
    ax.hist((k * bike["rests"]) % notch, bins=bins, color=INK, alpha=0.75,
            label=f"floor rests while engaged ({len(bike['rests'])}, whole run)")
    ax.hist((k * bench["starts_plus"]) % notch, bins=bins, histtype="step", color=LOADED,
            lw=1.8, label="hung creep: + jump starts")
    ax.hist((k * bench["starts_minus"]) % notch, bins=bins, histtype="step", color="#2a7bd4",
            lw=1.8, label="hung creep: - jump starts")
    f = (k * bench["flip"]) % notch
    ax.axvline(f, color=INK, lw=1.4)
    ax.axvline((f + notch / 2) % notch, color=INK, lw=1.4, ls="--")
    ax.set_xlim(0, notch)
    ax.set_xlabel(f"no-slop roller angle mod one notch ({notch:g} roller deg)")
    ax.set_ylabel("count")
    ax.legend(fontsize=8, loc="upper center", framealpha=0.95)
    for s_ in ("top", "right"):
        ax.spines[s_].set_visible(False)
    axs[1, 2].set_xlabel("bike time [s]")
    axs[1, 2].tick_params(labelbottom=True)

    for ax in axs[:2].flat:
        ax.grid(axis="x", color=GRID, lw=0.6)
    for ax in list(axs.flat) + [fig.axes[-1]]:
        ax.tick_params(colors=INK, labelsize=9)
        for s_ in ("top", "right"):
            ax.spines[s_].set_visible(False)
    axs[0, 0].set_ylabel("roll [deg]")
    axs[1, 0].set_ylabel("roller angle, wrapped to one revolution [deg]")
    axs[2, 0].set_ylabel("roller - no-slop angle [roller deg]")
    lim = _common(axs[2, :2])
    for c in (0, 1):
        axs[2, c].set_ylim(*lim)
    handles = [
        plt.Line2D([], [], color=IDEAL, lw=1.6, label=f"no slop: {k:g} x ring-vs-hub"),
        plt.Line2D([], [], color=CMD, lw=1.3, ls=(0, (4, 2)),
                   label="commanded: sim, the firmware's reference; bike, the same "
                         "recovered from its Present PWM"),
        plt.Line2D([], [], color=LOADED, lw=2.4, label="roller on the floor"),
        plt.Line2D([], [], color=MUTED, lw=0.8, label="the other rollers"),
        plt.Line2D([], [], color=TOP, lw=1.6, label="detent line: friction bump, sticks (old) / sawtooth flip, pushes away (new, bike)"),
        plt.Line2D([], [], color=TROUGH, lw=1.2, ls="--", label="midway: where the sawtooth stops pushing"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=3, fontsize=10, frameon=False)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    out = a.out_dir / f"{policy}{a.tag}_trace.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"bench ({a.bench_creep}): + jumps start {bench['plus']:.2f}, - jumps "
          f"{bench['minus']:.2f} deg ring mod 7.5 -> top {bench['flip']:.2f}, trough "
          f"{(bench['flip'] + 3.75) % 7.5:.2f}")
    print(f"bike ({a.bike_run.name}): {len(bike['rests'])} engaged rests, concentration "
          f"R {bike['R']:.2f} (0 = spread evenly over the notch, 1 = all at one angle)")
    for (title, _), (tr, r) in zip(cols, runs):
        # Per step, the loaded roller only: how much of the time it sits
        # within 10 % of a wall of its play, and whether it LAGS (deflected
        # against the way the no-slop angle is moving).
        t, on = r["t"], r["on"]
        ideal = np.degrees(k * r["ring"])
        slop = np.degrees(r["roller"]) - ideal[:, None]
        m = on >= 0
        sl = slop[np.arange(len(on)), np.clip(on, 0, N - 1)]
        v = np.gradient(ideal, t)
        moving = m & (np.abs(v) > 5.0)                  # roller deg/s
        half = (math.degrees(float(tr.model.tendon_range[
            tr.model.tendon("slop_roller_spin_0").id][1])) if _has_slop(tr.model) else np.nan)
        wall = np.mean(np.abs(sl[m]) > 0.9 * half)
        lag = np.mean(sl[moving] * np.sign(v[moving]) < 0)
        oth = np.abs(np.where(np.arange(N)[None, :] != on[:, None], slop, np.nan))[m]
        print(f"{title:50s} roll sd {r['roll'].std():4.2f} deg | loaded roller: at a wall "
              f"{100 * wall:3.0f} % of contact, lagging {100 * lag:3.0f} % while moving | "
              f"others |slop| median {np.nanmedian(oth):4.2f} roller deg")
    print("wrote", out)


def _detent_lines(ax, top_ring_deg, k, sawtooth=True):
    """The detent line (solid) in roller deg over one revolution, and with a
    sawtooth the midway rest (dashed). A friction-only detent has no
    midway: its line is where the shaft STICKS (48 % of the floor twin's
    rests within 1 deg of it, against 27 % for an even spread), not a top."""
    n = np.arange(-400, 400)
    for off, col, ls in ((0.0, TOP, "-"), (3.75, TROUGH, "--"))[:2 if sawtooth else 1]:
        for p in np.unique(np.round((k * (top_ring_deg + off + 7.5 * n)) % 360.0, 6)):
            ax.axhline(p, color=col, lw=0.9, ls=ls, zorder=0)


def _circ(x, p=7.5):
    z = np.exp(2j * np.pi * np.asarray(x) / p).mean()
    return (np.angle(z) / (2 * np.pi) * p) % p, abs(z)


def _phi_enc(cap, ids, signs):
    """ring-vs-hub [deg] in the raw encoder frame, the bench's formula
    BELT (sa P_A - sb P_B) / 2. One servo turn moves it 540 deg = 72
    notches, so a turn count reset at power-up keeps the notch phase."""
    import drivetrain_fit as F
    (A, B), (sa, sb) = ids, signs
    return np.degrees(F.BELT * (sa * F._fill(cap.position_rad(A))
                                - sb * F._fill(cap.position_rad(B))) / 2)


def bench_detent(name):
    """Where the hung creep's jumps start, in ring deg mod 7.5. The shaft
    winds up to the top and jumps: + starts just below it, - just above,
    and the top is midway through the stretch both jump over."""
    sys.path.insert(0, str(HERE.parent / "servo"))
    import drivetrain_fit as F
    cap = F._load(name)
    ids, signs = tuple(cap.ids), tuple(cap.meta["signs"])
    t, ph = F._fill(cap.time_s(ids[0])), _phi_enc(cap, ids, signs)
    out = {}
    for k_, s in enumerate(cap.meta["segments"]):
        inf = s["info"]
        if inf.get("kind") != "creep" or inf["mode"] != "diff":
            continue
        sl = cap.segment_slice(k_)
        tt, sg = t[sl], np.sign(inf["v"])
        q = ph[sl] * sg
        v = abs(np.degrees(F.BELT * inf["v"]))
        vel = np.gradient(np.convolve(q, np.ones(5) / 5, "same"), tt)
        idx = np.flatnonzero(vel > 2.5 * v)
        starts = [ph[sl][max(g[0] - 5, 0)]
                  for g in np.split(idx, np.flatnonzero(np.diff(idx) > 15) + 1)
                  if len(g) >= 3 and q[min(g[-1] + 5, len(q) - 1)] - q[max(g[0] - 5, 0)] > 1]
        out["plus" if sg > 0 else "minus"] = np.array(starts)
    p, _ = _circ(out["plus"] % 7.5)
    m, _ = _circ(out["minus"] % 7.5)
    gap = (m - p) % 7.5                       # + start up to - start: jumped over
    return {"plus": p, "minus": m, "flip": (p + gap / 2) % 7.5,
            "starts_plus": out["plus"], "starts_minus": out["minus"],
            "ids": ids, "signs": signs}


def bike_trace(run_dir, bench, t_from, seconds):
    """The bike's roll and ring-vs-hub over a window, and every engaged rest
    (diff under 20 deg/s for >= 40 ms) of the whole run."""
    from aow_sim.hw.bench_log import Capture
    sys.path.insert(0, str(HERE.parent / "servo"))
    import drivetrain_fit as F
    c = Capture.load(run_dir)
    # ONE row = one loop iteration: the servo reads and the Pi's state of the
    # same tick. Match by row on the Pi's clock. The servo's own tick clock
    # reads 20 ms behind it on every row (a different zero), so interpolating
    # one onto the other paired each position with the command two rows back.
    ph = _phi_enc(c, bench["ids"], bench["signs"])
    tb, st = c.bike("t"), c.bike("state").astype(int)
    t = tb
    eng_b = st == 0
    eng = eng_b
    if t_from is None:
        idx = np.flatnonzero(eng_b)
        segs = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        seg = max(segs, key=len)
        t_from = 0.5 * (tb[seg[0]] + tb[seg[-1]]) - seconds / 2
    w = np.gradient(ph, t)
    still = (np.abs(w) < 20.0) & eng
    idx = np.flatnonzero(still)
    rests = [np.median(ph[g]) for g in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
             if len(g) and t[g[-1]] - t[g[0]] >= 0.04]
    m = (t >= t_from) & (t < t_from + seconds)
    mb = (tb >= t_from) & (tb < t_from + seconds)
    # Where each servo is being pulled, from what it logs, sample by sample:
    # the firmware law duty = Kp (v_ref - w) + Ki (integral(v_ref) - theta)
    # gives the reference error e = (duty - Kp (v_ref - w)) / Ki. Nothing is
    # integrated, so nothing drifts. Kp, Ki are the bench fits per gain unit
    # times this run's gains; v_ref is the logged command (input-shaft rad/s
    # per ROLE) through onboard.yaml's servo_sign and the belt; w is Present
    # Velocity, unfiltered where the firmware filters 12 ms -- on the floor
    # twin this estimate tracks the true reference at r 0.996, median error
    # 0.4 deg of ring-vs-hub, 95th percentile 1.6. Saturated duty (|d| ~ 1)
    # understates e.
    import yaml
    params = load_params()
    srv = params["drivetrain_model"]["servo"]
    onboard = yaml.safe_load((REPO / "config" / "onboard.yaml").read_text())
    sign = dict(zip(("drive_a", "drive_b"), onboard["servo_sign"]))
    role = {int(k): v for k, v in c.meta["roles"].items()}
    belt = float(params["drivetrain"]["belt_ratio"])
    ctrl = c.bike("ctrl")
    ref = {}
    for sid in bench["ids"]:
        g = c.meta["gains_applied"][str(sid)]
        kp = float(srv["kp_per_unit"]) * g["Velocity P Gain"]
        ki = float(srv["ki_per_unit"]) * g["Velocity I Gain"]
        j = ("drive_a", "drive_b").index(role[sid])
        v_ref = sign[role[sid]] * ctrl[:, j] / belt
        duty = F._fill(c.value("Present PWM", sid))
        w_s = F._fill(c.value("Present Velocity", sid))
        ref[sid] = F._fill(c.position_rad(sid)) + (duty - kp * (v_ref - w_s)) / ki
    (A, B), (sa, sb) = bench["ids"], bench["signs"]
    cmd_phi = np.degrees(F.BELT * (sa * ref[A] - sb * ref[B]) / 2)
    return {"t": t[m], "phi": ph[m], "cmd": cmd_phi[m], "tb": tb[mb], "roll": np.degrees(c.bike("roll")[mb]),
            **_steer_about_straight(np.degrees(c.bike("steer")[mb]), np.degrees(ctrl[mb, 2])),
            "yaw": _yaw_about_median(_quat_yaw(c.bike("quat")[mb])),
            "rests": np.array(rests), "R": _circ(np.array(rests) % 7.5)[1],
            "policy": c.meta.get("policy", "?")}


def _quat_yaw(q):
    """Heading [rad] of (w, x, y, z) rows, z-up."""
    w, x, y, z = np.asarray(q).T
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _yaw_about_median(yaw_rad):
    """Unwrapped heading [deg] less its median: the page shows how the bike
    turns over the run, not which way the room faces (22-29 deg p-p on the
    two floor runs' longest engages)."""
    y = np.degrees(np.unwrap(yaw_rad))
    return y - np.median(y)


def _steer_about_straight(steer, target):
    """The bike records the steer joint with the extended-position turn count
    in it (174349's median is 539 deg). Straight repeats every 180 deg (the
    policy sees sin/cos 2 delta), so shift both by the one multiple of 180
    that puts the steer's median within +-90 of 0: the target keeps its
    offset from the shaft, and nothing is wrapped sample by sample."""
    shift = 180.0 * np.round(np.nanmedian(steer) / 180.0)
    return {"steer": steer - shift, "steer_cmd": target - shift}


PAGE_TEMPLATE = HERE / "templates" / "drivetrain_traces.html"
PAGE_OUT = HERE / "plots" / "drivetrain_traces.html"


def _longest_engage(run_dir):
    from aow_sim.hw.bench_log import Capture
    c = Capture.load(run_dir)
    tb, st = c.bike("t"), c.bike("state").astype(int)
    idx = np.flatnonzero(st == 0)
    seg = max(np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1), key=len)
    return c.meta.get("policy", "?"), float(tb[seg[0]]) + 0.2, float(tb[seg[-1]] - tb[seg[0]]) - 0.4


def build_page(a, k, cols):
    """The static page: every --page-runs bike run's longest engage, and its
    policy's floor sim (seed --seed, from t = 1 s, as long as that policy's
    longest engage) on each drivetrain in `cols`, at 100 Hz like the bike.
    Series are unwrapped roller degrees; the page wraps them."""
    import json
    notch = 7.5 * k
    bench = bench_detent(a.bench_creep)
    r2 = lambda x: [None if not np.isfinite(v) else round(float(v), 2) for v in x]
    # time to the ms: the bike's samples are 7-15 ms apart, so 2 decimals
    # merged neighbours into repeated time points (199 on 174349)
    r3 = lambda x: [round(float(v), 3) for v in x]
    bikes, sims, runs = {}, {}, {}
    for run in a.page_runs:
        pol, t0, dur = _longest_engage(run)
        bike = bike_trace(run, bench, t0, dur)
        bikes[run.name] = {
            "label": f"{run.name} ({pol})",
            "meta": f"{pol}, longest engage · drive encoders; commanded from Present PWM; "
                    f"detent placed by the hung creep {a.bench_creep}",
            "top": (k * bench["flip"]) % notch,
            "t": r3(bike["t"]), "roll": r2(bike["roll"]),
            "steer": r2(bike["steer"]), "steer_cmd": r2(bike["steer_cmd"]), "yaw": r2(bike["yaw"]),
            "noslop": r2(k * bike["phi"]), "cmd": r2(k * bike["cmd"])}
        runs[pol] = max(runs.get(pol, 0.0), dur)
    for pol, dur in runs.items():
        for title, ov in cols:
            tr, r = fly(pol, ov, 1.0, dur, a.seed)
            step = max(1, int(round(0.01 / tr.model.opt.timestep)))
            on = r["on"][::step]
            rol = np.degrees(r["roller"][::step])
            loaded = np.where(on >= 0, rol[np.arange(len(on)), np.clip(on, 0, N - 1)], np.nan)
            drv = tr.drive
            saw = bool(drv.hill and drv.hill_saw)
            sims[f"{pol}|{title}"] = {
                "label": f"{pol} · {title}",
                "meta": f"floor twin, seed {a.seed} · detent: "
                        + ("sawtooth flip (pushes away) over flat friction" if saw
                           else "friction bump (the shaft sticks on the line)")
                        + " · commanded = the firmware's own reference",
                "top": (k * math.degrees(drv.phase + 0.5 * drv.period)) % notch,
                "t": r3(r["t"][::step]), "roll": r2(r["roll"][::step]),
                "steer": r2(r["steer"][::step]), "steer_cmd": r2(r["steer_cmd"][::step]),
                "yaw": r2(r["yaw"][::step]),
                "noslop": r2(np.degrees(k * r["ring"][::step])),
                "cmd": r2(np.degrees(k * r["ref"][::step])), "loaded": r2(loaded)}
            print(f"  sim {pol} / {title}: {dur:.0f} s")
    first = a.page_runs[0].name
    pol0 = bikes[first]["label"].split("(")[1].rstrip(")")
    foot = ("Sim: <code>run_vs_sim.sim</code>, the floor twin standing on a zero command; its commanded "
            "line is the servo firmware's position reference (the integral of the velocity command). "
            "Bike: the drive encoders, wrapped the same way; its commanded line is that reference "
            "recovered per sample from Present PWM through the firmware law, "
            "e = (duty − Kp (v_ref − w)) / Ki, which on the sim tracks the true one at r 0.996 "
            "(median 1 roller degree). The bike's detent lines are placed by the hung creep "
            f"<code>{a.bench_creep}</code> in the same encoder frame, not fitted: one servo turn moves "
            "ring-vs-hub 540° = 72 notches, so a power-up turn reset keeps the phase. Sim and bike "
            "share notch geometry, not a time axis. Rendered by "
            "<code>python analysis/policy/drivetrain_trace.py --new "
            f"{a.new.relative_to(REPO) if a.new.is_absolute() else a.new} --page</code>.")
    data = json.dumps({"notch": notch, "k": k, "sims": sims, "bikes": bikes, "foot": foot,
                       "default": {"sim": f"{pol0}|{cols[1][0]}", "bike": first}},
                      separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
    tag = '<script id="data" type="application/json">'
    page = PAGE_TEMPLATE.read_text(encoding="utf-8")
    if page.count(tag + "/*DATA*/</script>") != 1:
        raise SystemExit(f"{PAGE_TEMPLATE.name}: expected exactly one data tag")
    PAGE_OUT.parent.mkdir(parents=True, exist_ok=True)
    PAGE_OUT.write_text(page.replace(tag + "/*DATA*/", tag + data), encoding="utf-8")
    print(f"wrote {PAGE_OUT.relative_to(REPO)} ({PAGE_OUT.stat().st_size / 1e6:.1f} MB)")

def _has_slop(model):
    try:
        model.tendon("slop_roller_spin_0")
        return True
    except KeyError:
        return False


def _common(row):
    lo = min(ax.get_ylim()[0] for ax in row)
    hi = max(ax.get_ylim()[1] for ax in row)
    return lo, hi


if __name__ == "__main__":
    main()
