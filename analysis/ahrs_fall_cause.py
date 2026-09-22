"""Which AHRS error makes the standing bike fall -- and is it one tick or a spell?

`tests/test_policy_endurance.py` registers the fall: the shipped policy,
standing still on its own sensors (velocity estimate + TM151), has an MTBF of
roughly a minute, and only when the attitude error model is on. This script
takes that fall apart. It flies EXACTLY the test's loop, with a `SimAhrs` whose
error components can be switched off after the draw -- the rng stream is
consumed identically whatever the mask, so two arms differ only in what
reaches the controller, never in which errors were drawn.

Complements `no_return.py`, which maps the recoverable set in the roll phase
plane on TRUTH sensors. This one asks which sensor error walks the bike out of
it.

SECTIONS

  study    64 seeds x 60 s on the full error. For every fall: snapshot the whole
           sim (physics, controller, estimator, AHRS) every 0.1 s over the last
           10 s, continue each snapshot on CLEAN sensors, and so find the last
           savable moment; then, from there, which channels are needed and how
           long the error must act. Plus whole-flight channel arms (roll only,
           pitch only, ...) over the same seeds.

  inject   The hypothesis `study` suggested, tested with clean sensors and one
           injected ROLL error: the AHRS reports a lean that is not there, the
           policy leans the real bike the other way, the error fades, and the
           swing back overshoots into a fall on the side the error claimed. A
           pulse (0.1 s rise, hold D, Gauss-Markov-like decay at tau 0.19 s)
           over amplitude x duration x sign x onset, plus a HELD offset that
           never releases -- if the release is what kills it, held must be
           kinder than a pulse of the same size.

  replay   Each fall's own recorded orientation error, played back onto a calm
           clean-sensor flight over its last W seconds: how much history does
           the fall need?

  motion   The test's flight logged at 100 Hz -- rear-wheel position, true
           and estimated velocity, policy commands -- and the excursion
           events in it.

  shaped   The mean roll error around excursion onset, injected on a clean
           bike and on top of the normal error on a noisy one.

  policies The standing flight across moves/personality0..11.

  report   Read the pickles, print the tables, write both figures.

FOUND 2026-09-22 (general_rl_cmd_curriculum2b, TM151, tau 0.19 s):

  * ROLL orientation error is the cause. Whole flights: all channels 36/64
    fall, roll+pitch 32, roll only 24, pitch only 1, everything-but-roll 3.
    The gyro, accel and yaw channels -- the only ones that change sharply
    tick to tick -- do essentially nothing.
  * NOT ONE TICK. On clean sensors the peak roll the bike would reach stays
    flat at ~4 deg from 8 s before the last savable moment until 0.5 s
    before it: no slow erosion. The fall comes 0.32-0.80 s (median 0.46)
    after the last savable snapshot. The lead-up is a roll error excursion
    toward the eventual fall side, ~+1.2 deg averaged, held ~0.5 s (2-3
    tau); toward the fall side in 81% of falls against 50% by chance.
  * HANDED: 36 of 36 falls to NEGATIVE roll (left, balance.py:15).

  ...AND THE SIMPLE HYPOTHESIS FAILED. `inject`, 800 pulses on clean sensors:
  up to 3 deg, held or released, NOTHING falls; it takes 4-8 deg (13/20
  held at 6, 10-11/20 at 8). The lean-away/swing-back KINEMATICS are real
  (true roll moves away from the claimed side by ~the error, then swings
  past upright by +2.8..+7 deg), but the falls that do happen go AWAY from
  the claimed side while the error is still held (41 of 42) -- the opposite
  of the natural falls. `replay`, each fall's OWN recorded roll(+pitch)
  error played onto a calm clean-sensor flight: 0/36 for the last 0.5-2 s,
  1/36 for 4 s, 5/36 for the last 11.9 s, and almost never at the recorded
  time. So a natural fall is neither a tick nor a signature excursion: it
  needs the bike already swaying under the error (roll RMS 2.4 deg vs 1.1
  on clean sensors) when a ~1 deg excursion lands -- the trajectory, not a
  pattern. The handedness survives injection: a phantom lean to the right
  (pushing the bike left) falls 32/400, to the left 10/400.

  python analysis/ahrs_fall_cause.py --section study            # ~17 min, 9 workers
  python analysis/ahrs_fall_cause.py --section inject           # ~1.5 min
  python analysis/ahrs_fall_cause.py --section replay           # ~3 min, needs study
  python analysis/ahrs_fall_cause.py --section motion           # ~1 min
  python analysis/ahrs_fall_cause.py --section shaped           # ~7 min, needs motion
  python analysis/ahrs_fall_cause.py --section policies         # ~20 min
  python analysis/ahrs_fall_cause.py --section report           # + both figures

  THE MOTION (`motion`; figures analysis/plots/ahrs_fall_cause_excursions.png,
  the group means, and ..._excursion_examples.png, three single events each).
  Standing is not still: the rear wheel makes a forward EXCURSION -- ~220 mm
  in ~0.85 s, steer swinging ~30-45 deg each way -- 5.3 times a minute, and
  their sum is the ~2-3 m/min forward creep on a zero command. 32 of the 36
  falls are an excursion that is not caught; before it the rear wheel shows
  nothing unusual (0.5 s net travel median 32-39 mm up to 1 s out, survivors
  38). Excursions go left and right about equally (110 / 98 caught) but ONLY
  LEFT ONES FAIL: 32/142 against 0/98. Nearly every excursion, either side,
  follows ~1 s of AHRS roll error claiming a LEFT lean (median -1.5 deg at
  onset), while the rear wheel drifts right (median ~50 mm over 1.5 s, but
  the 25-75% band spans zero for caught-right); a failed one
  over-steers ~+100 deg left instead of the ~+45 a caught one reaches.

  ...AND THAT SHAPE DOES NOT CAUSE FALLS (`shaped`). The mean roll error
  around excursion onset (a ~1 deg claimed-LEFT lean over ~1.4 s, then a fast
  return), injected at x1/x2/x3: on a CLEAN bike 0/60 fall (peak roll
  3.4-4.2 deg). Added ON TOP of the normal error, 470 paired 5 s windows
  over 64 seeds against the same flight without it: control 7.0%, injected
  7.0 / 7.2 / 8.3% (McNemar p 1.0 / 1.0 / 0.49) -- it reshuffles WHICH
  windows fall (about 2/3 of injected falls are in windows the control
  survived) without raising the rate. Flipped sign: 4.5 / 4.0 / 5.3% (p 0.07
  / 0.02 / 0.27; six comparisons, so weak). Every fall in every arm but 3 is
  to the LEFT. The trigger shape is what excursions look like on average,
  not what makes them fail.

  IT IS THIS POLICY (`policies`, 12 x 64 seeds x 60 s). personality1 is
  byte-identical to curriculum2b and the WORST stander of the twelve:

    policy          fell   MTBF s (95% CI)   L/R    exc/min
    personality0    0/64    inf (>1041)      -       0.4
    personality8    0/64    inf (>1041)      -       0.5
    personality6/7/11  1/64 each  ~3800      L       0.1-1.3
    personality4    2/64   1884              1/1    17.8
    personality3    5/64    731              4/1     0.8
    personality2/5  6/64    ~610         0/6, 6/0   14.1, 3.8
    personality9   15/64    223             10/5    13.0
    personality10  23/64    136              0/23    4.0
    personality1   36/64     75             36/0     5.3

  The side is per policy (1 and 5 fall left, 2 and 10 right), and the
  excursion rate does not predict falls (4: 17.8/min, 2 falls). The good
  standers are the ones seed-sweep-and-personalities.md records as IGNORING
  heading commands (0, 3, 6, 7, 8, 11 reverse on a 180); personality1 is the
  only one that turns as told, and the only one creeping forward on a hold.

Writes traces/ahrs_fall_cause_<section>.pkl (gitignored, Dropbox-synced).
Read-only otherwise. Run from the main checkout: a worktree imports main.
"""

from __future__ import annotations

import argparse
import contextlib
import copy
import io
import multiprocessing as mp
import pickle
import sys
import time
import warnings
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np
import yaml

warnings.filterwarnings("ignore")
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "traces"
T_END = 60.0                               # the endurance test's window
LIM = np.deg2rad(25.0)                     # its MAX_ROLL_DEG
COMPONENTS = ("roll", "pitch", "yaw", "gyro", "accel")
TAU_DECAY = 0.19                           # injected pulse release, = TAU_ORIENT_S
_G = {}
PEAK = [0.0]


# -- the masked sensor ----------------------------------------------------

def _masked_cls():
    from aow_sim import sim_ahrs as SA

    class MaskedAhrs(SA.SimAhrs):
        """SimAhrs with per-component switches and an optional injected roll
        error. `super().sample` always runs, so the rng stream is untouched."""
        on = frozenset(COMPONENTS)
        inject = None                      # t -> extra roll error [rad]

        def sample(self, data, dt):
            out = super().sample(data, dt)
            if self.level == "none" or (self.on == frozenset(COMPONENTS)
                                        and self.inject is None):
                return out
            quat = self._raw(data, "ahrs_quat")
            e = self._orient_err * np.array(
                [c in self.on for c in ("roll", "pitch", "yaw")])
            if self.inject is not None:
                inj = np.atleast_1d(self.inject(float(data.time)))
                e = e + np.resize(np.append(inj, [0.0, 0.0]), 3) if inj.size == 1 \
                    else e + inj
            d = self._yaw_drift if "yaw" in self.on else 0.0
            q = SA._quat_mul(np.array([np.cos(d / 2), 0, 0, np.sin(d / 2)]),
                             SA._quat_mul(SA._small_angle_quat(e), quat))
            self._cache = {
                "quat": q / np.linalg.norm(q),
                "gyro": out["gyro"] if "gyro" in self.on
                else self._raw(data, "ahrs_gyro"),
                "accel": out["accel"] if "accel" in self.on
                else self._raw(data, "ahrs_accel")}
            return self._cache
    return MaskedAhrs


# -- the test's loop, forkable --------------------------------------------

def setup(seed, on=COMPONENTS, name=None):
    from aow_sim.build_model import build_model, load_params
    from aow_sim.control.drive import DriveController
    from aow_sim.control.flick import MOVES_DIR
    from aow_sim.control.linearize import settle_upright
    from aow_sim.sim_odometry import SimOdometry
    if "model" not in _G:
        _G["params"] = load_params()
        _G["model"] = build_model(_G["params"])
        _G["name"] = _G["params"]["control"]["general_move"]
        _G["specs"] = {}
    name = name or _G["name"]
    if name not in _G["specs"]:
        _G["specs"][name] = yaml.safe_load((MOVES_DIR / f"{name}.yaml").read_text())
    params, model, spec = _G["params"], _G["model"], _G["specs"][name]
    ahrs = _masked_cls()(model, params, level=spec["ahrs_level"], seed=seed,
                         tau_orient_s=float(spec["ahrs_tau_s"]))
    ahrs.on = frozenset(on)
    odo = SimOdometry(model, params, mode="front",
                      encoder=spec["odometry_encoder"], ahrs=ahrs)
    data = mujoco.MjData(model)
    data.qpos[:] = settle_upright(model).qpos
    mujoco.mj_forward(model, data)
    with contextlib.redirect_stdout(io.StringIO()):
        ctl = DriveController(params, model)
        ctl.reset(model, data)
        ctl._odometry_active = ctl._ahrs_active = True
        ctl.engage_general(data, name=name)
        ctl.set_command_polar(0.0)
    return {"data": data, "ctl": ctl, "odo": odo, "ahrs": ahrs, "peak": 0.0}


def fork(st):
    m = _G["model"]
    return copy.deepcopy(st, memo={id(m): m})


def _state(data):
    from aow_sim.control.balance import extract_state
    return extract_state(data, np.zeros(3))


def fly(st, t_stop, snap_every=None, keep_s=10.0, log=False):
    """Step as the test does until a fall or t_stop -> (fall time | None,
    snapshots, log rows). Log: t, roll, roll_rate, orient_err xyz, gyro err
    xyz, steer."""
    model = _G["model"]
    dt = model.opt.timestep
    data, ctl, odo, ahrs = st["data"], st["ctl"], st["odo"], st["ahrs"]
    snaps = deque(maxlen=int(keep_s / snap_every) + 1) if snap_every else None
    nsnap = int(round(snap_every / dt)) if snap_every else 0
    rows, k = [], 0
    while data.time < t_stop - 1e-9:
        if nsnap and k % nsnap == 0:
            snaps.append((float(data.time), fork(st)))
        with odo.estimated(data, dt):
            ctl.step(model, data)
        mujoco.mj_step(model, data)
        if k % 25 == 0:
            s = _state(data)
            st["peak"] = max(st["peak"], abs(float(s.roll)))
            if log:
                rows.append((data.time, s.roll, s.roll_rate, *ahrs._orient_err,
                             *(ahrs._cache["gyro"] - ahrs._raw(data, "ahrs_gyro")),
                             float(data.qpos[ctl._sj])))
            if abs(s.roll) > LIM:
                return float(data.time), snaps, np.array(rows)
        k += 1
    return None, snaps, np.array(rows)


def falls_from(snap, on, horizon, err_for=None):
    """Continue a snapshot with only `on` components (after `err_for` seconds
    of the full error, if given). Sets PEAK to the peak |roll| in degrees."""
    st = fork(snap)
    st["peak"] = 0.0
    t0 = st["data"].time
    if err_for:
        st["ahrs"].on = frozenset(COMPONENTS)
        f, _, _ = fly(st, t0 + err_for)
        if f is not None:
            return f
    st["ahrs"].on = frozenset(on)
    f, _, _ = fly(st, t0 + horizon)
    PEAK[0] = np.degrees(st["peak"])
    return f


# -- section: study ---------------------------------------------------------

def study_seed(seed):
    st = setup(seed)
    tf, snaps, _ = fly(st, T_END, snap_every=0.1, keep_s=10.0)
    res = {"seed": seed, "tf": tf}
    if tf is None:
        return res
    _, _, rows = fly(setup(seed), tf + 0.01, log=True)   # deterministic re-run
    res["log"] = rows[rows[:, 0] > tf - 12.0]
    res["fall_dir"] = float(np.sign(rows[-1, 1]))
    snaps = list(snaps)
    clean, margin = [], []
    for t, sn in snaps:
        f = falls_from(sn, (), horizon=5.0)
        clean.append((t, f))
        margin.append((t, PEAK[0] if f is None else 99.0,
                       np.degrees(_state(sn["data"]).roll)))
    res["clean"] = clean
    res["margin"] = np.array(margin)      # t, clean peak |roll| (99 = falls), roll
    saved = [t for t, f in clean if f is None]
    if not saved:
        return res
    t_save = max(saved)
    res["t_save"] = t_save
    res["n_unsaved_before"] = sum(f is not None for t, f in clean if t < t_save)
    sn = dict(snaps)[t_save]
    s = _state(sn["data"])
    res["state_at_save"] = (np.degrees(s.roll), np.degrees(s.roll_rate))
    res["err_for"] = {W: falls_from(sn, (), 5.0, err_for=W)
                      for W in (0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0)}
    res["only"] = {c: falls_from(sn, (c,), 5.0) for c in COMPONENTS}
    res["without"] = {c: falls_from(sn, tuple(x for x in COMPONENTS if x != c), 5.0)
                      for c in COMPONENTS}
    return res


ARMS = (("roll", "pitch"), ("roll",), ("pitch",), ("pitch", "yaw", "gyro", "accel"))


def whole_flight(args):
    seed, on = args
    tf, _, _ = fly(setup(seed, on), T_END)
    return seed, on, tf


# -- section: inject --------------------------------------------------------

AMPS_DEG = (0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0)
HOLDS_S = (0.1, 0.25, 0.5, 1.0, None)      # None = held, never released
ONSETS_S = tuple(5.0 + 1.03 * k for k in range(10))
RISE_S = 0.1


def pulse(t0, amp, hold):
    """Roll error [rad] vs time: linear rise, hold, exponential release."""
    def f(t):
        tau = t - t0
        if tau <= 0.0:
            return 0.0
        if tau < RISE_S:
            return amp * tau / RISE_S
        if hold is None or tau < RISE_S + hold:
            return amp
        return amp * np.exp(-(tau - RISE_S - hold) / TAU_DECAY)
    return f


def inject_onset(args):
    onset, sign = args
    st = setup(0, on=())                   # clean sensors, estimate still on
    fly(st, onset)
    base = fork(st)
    out = []
    for a in AMPS_DEG:
        for hold in HOLDS_S:
            b = fork(base)
            b["ahrs"].inject = pulse(onset, sign * np.deg2rad(a), hold)
            release = onset + RISE_S + (hold if hold is not None else np.inf)
            horizon = onset + RISE_S + (hold or 0.0) + 4.0
            _, _, rows = fly(b, horizon, log=True)
            tf = float(rows[-1, 0]) if abs(rows[-1, 1]) > LIM else None
            t, roll = rows[:, 0], rows[:, 1]
            during = (t > onset) & (t < min(release, horizon))
            after = t >= release
            out.append({
                "onset": onset, "sign": sign, "amp": a, "hold": hold, "tf": tf,
                "fall_side": (float(np.sign(roll[-1])) if tf else 0.0),
                "fell_during_hold": bool(tf is not None and tf < release),
                "t_after_release": (tf - release) if tf and tf >= release else None,
                # signed by the error: + = the side the sensor falsely claimed
                "min_during": np.degrees(np.min(sign * roll[during]))
                if during.any() else np.nan,
                "max_after": np.degrees(np.max(sign * roll[after]))
                if after.any() else np.nan})
    return out


# -- section: motion -------------------------------------------------------

MOTION_COLS = ("t", "x", "y", "yaw", "v_lon", "v_lat", "v_lon_est", "v_lat_est",
               "roll", "roll_rate", "steer", "cmd_steer_rate", "cmd_hub",
               "cmd_diff", "roll_err")


def motion_flight(seed, name=None):
    """The test's flight, logged at 100 Hz: where the REAR WHEEL goes (the
    chassis origin is the rear axle centre), what the bike thinks it is doing,
    and what the policy commands. Columns: MOTION_COLS."""
    st = setup(seed, name=name)
    model = _G["model"]
    dt = model.opt.timestep
    data, ctl, odo, ahrs = st["data"], st["ctl"], st["odo"], st["ahrs"]
    rows, k, tf = [], 0, None
    while data.time < T_END - 1e-9:
        with odo.estimated(data, dt):
            ctl.step(model, data)
        mujoco.mj_step(model, data)
        if k % 25 == 0:
            s = _state(data)
            u = ctl._gen_u or (0.0, 0.0, 0.0, 0.0)
            rows.append((data.time, data.qpos[0], data.qpos[1], s.yaw, s.v_lon,
                         s.v_lat, *odo._last, s.roll, s.roll_rate,
                         float(data.qpos[ctl._sj]), u[0], u[1], u[2],
                         ahrs._orient_err[0]))
            if abs(s.roll) > LIM:
                tf = float(data.time)
                break
        k += 1
    return {"seed": seed, "tf": tf, "log": np.array(rows)}


EXC_DISP_M = 0.150     # net rear-wheel travel over EXC_WIN_S that makes an excursion
EXC_WIN_S = 0.5        # survivors: median 38 mm per 0.5 s, 99th pct 227


def excursions(M):
    """Excursion events from `motion` logs: runs where the rear wheel's NET
    displacement over the last EXC_WIN_S exceeds EXC_DISP_M, merged across
    gaps < EXC_WIN_S. The event starts where the crossing window began."""
    c = {n: i for i, n in enumerate(MOTION_COLS)}
    out = []
    for r in M:
        L = r["log"]
        n, w = len(L), int(round(EXC_WIN_S / 0.01))
        d = np.zeros(n)
        d[w:] = np.hypot(L[w:, c["x"]] - L[:-w, c["x"]], L[w:, c["y"]] - L[:-w, c["y"]])
        on = d > EXC_DISP_M
        i = 0
        while i < n:
            if not on[i]:
                i += 1
                continue
            j = i
            while j < n and on[j:j + w].any():
                j += 1
            s0, s1 = max(0, i - w), min(j, n - 1)
            k = slice(s0, s1 + 1)
            side = float(np.sign(L[k, c["roll"]][np.argmax(np.abs(L[k, c["roll"]]))]))
            yaw = L[s0, c["yaw"]]
            dx, dy = L[s1, c["x"]] - L[s0, c["x"]], L[s1, c["y"]] - L[s0, c["y"]]
            pre = (L[:, 0] >= L[s0, 0] - 1.0) & (L[:, 0] <= L[s0, 0])
            out.append({
                "seed": r["seed"], "i0": s0, "t0": L[s0, 0], "t1": L[s1, 0],
                "fell": r["tf"] is not None and s1 >= n - 1, "side": side,
                "d_lon": np.cos(yaw) * dx + np.sin(yaw) * dy,
                "d_lat": -np.sin(yaw) * dx + np.cos(yaw) * dy,
                "steer_travel": np.degrees(L[k, c["steer"]] - L[s0, c["steer"]]),
                "err_toward": side * np.degrees(L[pre, c["roll_err"]].mean())})
            i = j + 1
    return out


def report_motion(M, plot=True):
    ev = excursions(M)
    expo = sum(r["log"][-1, 0] for r in M) / 60.0
    nf = sum(r["tf"] is not None for r in M)
    F = [e for e in ev if e["fell"]]
    print(f"\nMOTION: {len(ev)} excursions (> {EXC_DISP_M*1e3:.0f} mm net rear travel "
          f"in {EXC_WIN_S} s) over {expo:.1f} min = {len(ev)/expo:.1f}/min; "
          f"{len(F)} of the {nf} falls are one")
    groups = (("caught left", lambda e: not e["fell"] and e["side"] < 0),
              ("caught right", lambda e: not e["fell"] and e["side"] > 0),
              ("failed left", lambda e: e["fell"] and e["side"] < 0),
              ("failed right", lambda e: e["fell"] and e["side"] > 0))
    print("                  n   fwd mm   lat mm   dur s   steer travel min/max deg"
          "   roll err 1 s before, toward side")
    for name, f in groups:
        g = [e for e in ev if f(e)]
        if not g:
            print(f"  {name:13s} {0:4d}")
            continue
        med = lambda h: float(np.median([h(e) for e in g]))
        print(f"  {name:13s} {len(g):4d}  {1e3*med(lambda e: e['d_lon']):+6.0f}  "
              f"{1e3*med(lambda e: e['d_lat']):+6.0f}   {med(lambda e: e['t1']-e['t0']):4.2f}"
              f"    {med(lambda e: e['steer_travel'].min()):+6.1f} / "
              f"{med(lambda e: e['steer_travel'].max()):+6.1f}"
              f"          {med(lambda e: e['err_toward']):+5.2f} "
              f"({np.mean([e['err_toward'] > 0 for e in g]):.0%} toward)")
    left = [e for e in ev if e["side"] < 0]
    right = [e for e in ev if e["side"] > 0]
    print(f"  failure rate: left {sum(e['fell'] for e in left)}/{len(left)}, "
          f"right {sum(e['fell'] for e in right)}/{len(right)}")
    if not plot:
        return
    _plot_excursions(M, ev, groups[:3])


PANELS = (("lon", "rear fwd travel [mm]"), ("lat", "rear lateral travel [mm, +left]"),
          ("roll", "true roll [deg, +right]"), ("err", "AHRS roll error [deg]"),
          ("steer", "steer travel [deg]"))
LAG = np.arange(-150, 151)                          # 10 ms samples, -1.5 .. +1.5 s


def _traces(M, g):
    """Per event, each PANELS quantity on LAG around onset (NaN off the log).
    Travel and steer are relative to their value at onset."""
    c = {n: i for i, n in enumerate(MOTION_COLS)}
    Mi = {r["seed"]: r["log"] for r in M}
    out = {k: [] for k, _ in PANELS}
    for e in g:
        L = Mi[e["seed"]]
        idx = e["i0"] + LAG
        ok = (idx >= 0) & (idx < len(L))
        row = lambda v: np.where(ok, v[np.clip(idx, 0, len(L) - 1)], np.nan)
        yaw = L[e["i0"], c["yaw"]]
        dx = row(L[:, c["x"]]) - L[e["i0"], c["x"]]
        dy = row(L[:, c["y"]]) - L[e["i0"], c["y"]]
        out["lon"].append(1e3 * (np.cos(yaw) * dx + np.sin(yaw) * dy))
        out["lat"].append(1e3 * (-np.sin(yaw) * dx + np.cos(yaw) * dy))
        out["roll"].append(np.degrees(row(L[:, c["roll"]])))
        out["err"].append(np.degrees(row(L[:, c["roll_err"]])))
        out["steer"].append(np.degrees(row(L[:, c["steer"]]) - L[e["i0"], c["steer"]]))
    return {k: np.array(v) for k, v in out.items()}


def _dress(ax, title):
    for a, (_, lab) in zip(ax, PANELS):
        a.set_ylabel(lab, fontsize=8)
        a.axvline(0, color="k", lw=0.5)
        a.axhline(0, color="k", lw=0.5)
        a.grid(alpha=0.3)
    ax[0].set_title(title, fontsize=9)
    ax[-1].set_xlabel("time from excursion onset [s]")


def _plot_excursions(M, ev, groups):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = LAG * 0.01
    cols = ("C0", "C2", "C3")
    tr = [(name, [e for e in ev if f(e)]) for name, f in groups]
    tr = [(name, g, _traces(M, g)) for name, g in tr]

    # 1. THE MEAN, deliberately not the median: a per-step median of noisy
    # traces is itself noisy and reads as one example, which it is not. Band
    # is +-1 SD of the events at that step -- their spread, not an error bar.
    # Falls end their logs, so once fewer than half the group remain the
    # statistic describes a shrinking, biased subset: cut there.
    fig, ax = plt.subplots(len(PANELS), 1, figsize=(7, 12), sharex=True)
    for (name, g, T), col in zip(tr, cols):
        for a, (k, _) in zip(ax, PANELS):
            X = T[k]
            keep = np.isfinite(X).sum(0) >= len(g) / 2
            mu = np.where(keep, np.nanmean(X, 0), np.nan)
            sd = np.where(keep, np.nanstd(X, 0), np.nan)
            a.fill_between(t, mu - sd, mu + sd, color=col, alpha=0.12, lw=0)
            a.plot(t, mu, col, label=f"{name} (n={len(g)})")
    ax[2].set_ylim(-26, 12)
    ax[0].legend(fontsize=8)
    _dress(ax, "Standing excursions, aligned on onset\n"
               "line: MEAN across events at each 10 ms step;"
               " band: +-1 SD (spread of events)")
    fig.tight_layout()
    out = ROOT / "analysis" / "plots" / "ahrs_fall_cause_excursions.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"  wrote {out.relative_to(ROOT)}")

    # 2. EXEMPLARS: three real events per group, one column each. Chosen by
    # rule, not by eye -- the events at the 25th, 50th and 75th percentile of
    # peak steer travel (max |steer - steer at onset| within the event).
    fig, ax = plt.subplots(len(PANELS), len(tr), figsize=(4.2 * len(tr), 12),
                           sharex=True, sharey="row")
    for j, ((name, g, T), col) in enumerate(zip(tr, cols)):
        peak = np.array([np.nanmax(np.abs(e["steer_travel"])) for e in g])
        order = np.argsort(peak)
        for q, ls in zip((25, 50, 75), (":", "-", "--")):
            i = order[min(len(g) - 1, int(round(q / 100 * (len(g) - 1))))]
            e = g[i]
            for a, (k, _) in zip(ax[:, j], PANELS):
                a.plot(t, T[k][i], col, ls=ls, lw=1,
                       label=f"p{q}: seed {e['seed']} t={e['t0']:.1f}s"
                             f"{' FELL' if e['fell'] else ''}")
        _dress(ax[:, j], f"{name} (n={len(g)}): 3 single events")
        ax[0, j].legend(fontsize=7)
    ax[2, 0].set_ylim(-26, 12)
    fig.suptitle("Example excursions, picked at the 25/50/75th percentile of "
                 "peak steer travel within each group", fontsize=10)
    fig.tight_layout()
    out = ROOT / "analysis" / "plots" / "ahrs_fall_cause_excursion_examples.png"
    fig.savefig(out, dpi=100)
    plt.close(fig)
    print(f"  wrote {out.relative_to(ROOT)}")


# -- section: shaped -------------------------------------------------------
#
# The injection `inject` used was a rectangle-ish pulse. This one uses the
# SHAPE the excursions actually follow: the mean AHRS roll error around onset,
# pooled over every excursion `motion` found, -1.5 .. +0.5 s. Played onto a
# CLEAN bike, and ADDED ON TOP of the normal error on a NOISY one, where the
# control is the same deterministic flight without it -- its own fall time.

SHAPE_SCALES = (1.0, 2.0, 3.0)
SHAPE_SIGNS = (1.0, -1.0)           # +1 = as recorded (claims a LEFT lean)
SHAPE_ONSETS_S = tuple(5.0 + 5.0 * k for k in range(10))   # 5 .. 50 s
SHAPE_AFTER_S = 3.0                 # watch this long past the shape's end


def mean_excursion_error():
    """(t [s] relative to injection start, roll error [rad]): the pooled mean,
    shifted to start at 0 and zero outside its 2 s."""
    M = pickle.load(open(OUT / "ahrs_fall_cause_motion.pkl", "rb"))
    ev = excursions(M)
    lag = np.arange(-150, 51)
    c = MOTION_COLS.index("roll_err")
    Mi = {r["seed"]: r["log"] for r in M}
    X = []
    for e in ev:
        idx = e["i0"] + lag
        if idx[0] >= 0 and idx[-1] < len(Mi[e["seed"]]):
            X.append(Mi[e["seed"]][idx, c])
    y = np.mean(X, 0)
    y = y - y[0]
    return (lag - lag[0]) * 0.01, y


def _shape_fn(t0, tt, yy, k):
    return lambda t: (k * float(np.interp(t - t0, tt, yy))
                      if 0.0 <= t - t0 <= tt[-1] else 0.0)


def shaped_clean(args):
    onset, sign = args
    tt, yy = mean_excursion_error()
    st = setup(0, on=())
    fly(st, onset)
    base = fork(st)
    out = []
    for k in SHAPE_SCALES:
        b = fork(base)
        b["ahrs"].inject = _shape_fn(onset, tt, yy, sign * k)
        f, _, rows = fly(b, onset + tt[-1] + SHAPE_AFTER_S, log=True)
        out.append({"onset": onset, "sign": sign, "k": k, "tf": f,
                    "side": float(np.sign(rows[-1, 1])) if f else 0.0,
                    "peak": float(np.degrees(np.abs(rows[:, 1]).max()))})
    return out


def shaped_noisy(seed):
    """Full error + the shape on top, forked off the seed's own flight."""
    tt, yy = mean_excursion_error()
    st = setup(seed)
    out = []
    for onset in SHAPE_ONSETS_S:
        tf, _, _ = fly(st, onset)
        if tf is not None:                 # the flight already fell: done
            break
        base = fork(st)
        end = onset + tt[-1] + SHAPE_AFTER_S
        ctl = fork(base)
        fc, _, _ = fly(ctl, end)           # control: the flight as it was
        row = {"seed": seed, "onset": onset, "control": fc}
        for sign in SHAPE_SIGNS:
            for k in SHAPE_SCALES:
                b = fork(base)
                b["ahrs"].inject = _shape_fn(onset, tt, yy, sign * k)
                f, _, rows = fly(b, end, log=True)
                row[(sign, k)] = (f, float(np.sign(rows[-1, 1])) if f else 0.0)
        out.append(row)
    return out


def report_shaped(R):
    tt, yy = mean_excursion_error()
    print(f"\nSHAPED: the mean pre-excursion roll error (peak "
          f"{np.degrees(yy.min()):+.2f} deg, {tt[-1]:.1f} s), x scale, +-sign")
    C = [r for chunk in R["clean"] for r in chunk]
    print("  CLEAN bike (10 onsets):")
    for s in SHAPE_SIGNS:
        print("    sign " + ("as recorded (claims left)" if s > 0 else "flipped (claims right)") + ": "
              + "  ".join(f"x{k:g} fell {sum(r['tf'] is not None for r in C if r['sign']==s and r['k']==k)}/"
                          f"{sum(1 for r in C if r['sign']==s and r['k']==k)}"
                          f" (peak roll {np.median([r['peak'] for r in C if r['sign']==s and r['k']==k]):.1f})"
                          for k in SHAPE_SCALES))
    N = [r for chunk in R["noisy"] for r in chunk]
    n = len(N)
    base = sum(r["control"] is not None for r in N)
    print(f"  NOISY bike ({n} paired windows of {tt[-1]+SHAPE_AFTER_S:.0f} s over 64 seeds):")
    print(f"    control (no injection): {base}/{n} fell = {base/n:.1%}")
    for s in SHAPE_SIGNS:
        for k in SHAPE_SCALES:
            f = [r[(s, k)] for r in N]
            fell = sum(x[0] is not None for x in f)
            both = sum(x[0] is not None and r["control"] is not None for x, r in zip(f, N))
            left = sum(x[0] is not None and x[1] < 0 for x in f)
            print(f"    {'as recorded' if s > 0 else 'flipped    '} x{k:g}: {fell:3d}/{n} = {fell/n:5.1%}"
                  f"  (control also fell in {both}; to the left {left}/{fell})")


# -- section: policies -----------------------------------------------------
#
# The same standing flight across a family of policies. Only the summary is
# kept per flight (the full logs would be ~0.5 GB).

POLICIES = tuple(f"personality{k}" for k in range(12))


def policy_flight(args):
    name, seed = args
    r = motion_flight(seed, name=name)
    ev = excursions([r])
    return {"policy": name, "seed": seed, "tf": r["tf"],
            "side": float(np.sign(r["log"][-1, MOTION_COLS.index("roll")]))
            if r["tf"] else 0.0,
            "exposure": float(r["log"][-1, 0]),
            "exc": [(e["side"], e["fell"]) for e in ev]}


def report_policies(R):
    from scipy.stats import chi2
    print(f"\nPOLICIES: standing on own sensors, {T_END:.0f} s x "
          f"{len({r['seed'] for r in R})} seeds each")
    print("  policy          fell   MTBF s (95% CI)   L/R falls  test(0-17)"
          "  exc/min  catch fail L / R")
    rows = []
    for name in dict.fromkeys(r["policy"] for r in R):
        g = [r for r in R if r["policy"] == name]
        n = sum(r["tf"] is not None for r in g)
        T = sum(r["exposure"] for r in g)
        lo = T / (chi2.ppf(0.975, 2 * n + 2) / 2)
        hi = T / (chi2.ppf(0.025, 2 * n) / 2) if n else np.inf
        L = sum(r["side"] < 0 for r in g)
        test = sum(r["tf"] is not None for r in g if r["seed"] < ENDURANCE_SEEDS)
        ex = [e for r in g for e in r["exc"]]
        eL = [f for sd, f in ex if sd < 0]
        eR = [f for sd, f in ex if sd > 0]
        rows.append((T / n if n else np.inf, name, n, len(g), lo, hi, L, n - L, test,
                     len(ex) / (T / 60), sum(eL), len(eL), sum(eR), len(eR)))
    for (mtbf, name, n, N, lo, hi, L, Rr, test, epm, fl, nl, fr, nr) in sorted(rows, reverse=True):
        m = f"{mtbf:6.0f}" if np.isfinite(mtbf) else "   inf"
        print(f"  {name:14s} {n:2d}/{N}  {m} ({lo:4.0f}-{hi:5.0f})   {L:2d}/{Rr:<2d}"
              f"      {test:2d}/{ENDURANCE_SEEDS} {'PASS' if test <= ENDURANCE_MAX_FALLS else 'red '}"
              f"   {epm:4.1f}   {fl:2d}/{nl:<3d} / {fr:2d}/{nr}")


ENDURANCE_SEEDS, ENDURANCE_MAX_FALLS = 18, 4      # tests/test_policy_endurance.py


# -- section: replay -------------------------------------------------------

REPLAY_W = (0.5, 1.0, 2.0, 4.0, 8.0, 11.9)


def replay_fall(args):
    """Fly clean sensors, then play back ONE recorded fall's own orientation
    error over the last W seconds before its fall -> fell?"""
    rec, which = args
    L = rec["log"]
    t, er, ep = L[:, 0], L[:, 3], L[:, 4]
    tf = rec["tf"]
    st = setup(0, on=())
    fly(st, tf - max(REPLAY_W) - 0.2)
    base = fork(st)
    out = []
    for W in REPLAY_W:
        b = fork(base)
        t0 = tf - W
        use_p = which == "roll+pitch"

        def inj(tt, t0=t0, use_p=use_p):
            if tt < t0 or tt > t[-1]:
                return np.zeros(3)
            return np.array([np.interp(tt, t, er),
                             np.interp(tt, t, ep) if use_p else 0.0, 0.0])
        b["ahrs"].inject = inj
        f, _, rows = fly(b, tf + 3.0, log=True)
        out.append({"seed": rec["seed"], "which": which, "W": W, "tf": f,
                    "dt": (f - tf) if f else None,
                    "side": float(np.sign(rows[-1, 1])) if f else 0.0,
                    "rec_side": rec["fall_dir"]})
    return out


def report_replay(R):
    rows = [r for chunk in R for r in chunk]
    n = len({r["seed"] for r in rows})
    print(f"\nREPLAY: clean sensors, then the fall's OWN recorded error over its "
          f"last W s. fell / {n}")
    for which in ("roll", "roll+pitch"):
        cells = []
        for W in REPLAY_W:
            c = [r for r in rows if r["which"] == which and r["W"] == W]
            f = [r for r in c if r["tf"] is not None]
            same = sum(r["side"] == r["rec_side"] for r in f)
            near = sum(abs(r["dt"]) < 0.5 for r in f)
            cells.append(f"W {W:4.1f}s {len(f):2d} (same side {same}, within 0.5 s {near})")
        print(f"  {which:10s} " + "\n             ".join(cells))


# -- report ----------------------------------------------------------------

def report_study(D):
    per = D["per"]
    fell = [r for r in per if r["tf"] is not None]
    print(f"STUDY: {len(fell)}/{len(per)} fell in {T_END:.0f} s; MTBF "
          f"{sum(r['tf'] or T_END for r in per) / max(1, len(fell)):.0f} s")
    d = np.array([r["fall_dir"] for r in fell])
    print(f"  fall side: {int((d < 0).sum())} to -roll (left), "
          f"{int((d > 0).sum())} to +roll (right)")
    for arm in ARMS:
        tf = [t for s, a, t in D["whole"] if tuple(a) == arm]
        print(f"  whole flight, only {'+'.join(arm):26s} fell "
              f"{sum(t is not None for t in tf):2d}/{len(tf)}")
    sv = [r for r in fell if r.get("t_save") is not None]
    lead = np.array([r["tf"] - r["t_save"] for r in sv])
    print(f"  last savable -> fall: median {np.median(lead):.2f} s "
          f"({lead.min():.2f}-{lead.max():.2f})")
    print("  clean-sensor peak |roll| by lag to the last savable moment:")
    for lag in (-8.0, -4.0, -2.0, -1.0, -0.5, 0.0):
        v = [p for r in sv for t, p, _ in r["margin"]
             if abs(t - r["t_save"] - lag) < 0.05]
        if v:
            print(f"    {lag:+5.1f} s  median {np.median(np.minimum(v, 99)):4.1f} deg"
                  f"  unsavable {np.mean(np.array(v) >= 99):4.0%}")
    print("  roll error vs true roll, aligned, signed toward the fall side:")
    for lag in (-1.0, -0.75, -0.5, -0.3, -0.1, 0.0, 0.2):
        v = []
        for r in sv:
            L = r["log"]
            i = np.argmin(np.abs(L[:, 0] - r["t_save"] - lag))
            v.append(r["fall_dir"] * np.degrees([L[i, 3], L[i, 1]]))
        v = np.array(v)
        print(f"    {lag:+5.2f} s  roll err {v[:, 0].mean():+5.2f}  "
              f"true roll {v[:, 1].mean():+5.2f} deg")


def report_inject(R):
    rows = [r for chunk in R for r in chunk]
    print("\nINJECT: clean sensors + one roll-error pulse. fell / "
          f"{len(ONSETS_S) * 2} (10 onsets x 2 signs)")
    print("  amp \\ hold  " + "".join(f"{('held' if h is None else f'{h:.2f}s'):>8s}"
                                    for h in HOLDS_S))
    for a in AMPS_DEG:
        cells = []
        for h in HOLDS_S:
            c = [r for r in rows if r["amp"] == a and r["hold"] == h]
            cells.append(f"{sum(r['tf'] is not None for r in c):>8d}")
        print(f"  {a:4.1f} deg    " + "".join(cells))
    f = [r for r in rows if r["tf"] is not None]
    print(f"  falls: {len(f)} of {len(rows)}")
    print("  true roll, signed by the error (+ = the side the sensor claimed), median:")
    print("    amp   during hold (min)   after release (max)")
    for a in AMPS_DEG:
        c = [r for r in rows if r["amp"] == a and r["hold"] is not None]
        print(f"    {a:3.1f}   {np.nanmedian([r['min_during'] for r in c]):+6.2f}"
              f"             {np.nanmedian([r['max_after'] for r in c]):+6.2f}")
    if not f:
        return
    toward = sum(r["fall_side"] == r["sign"] for r in f)
    print(f"  fall on the side the error CLAIMED: {toward}/{len(f)}")
    dur = sum(r["fell_during_hold"] for r in f)
    print(f"  fell while the error was still held: {dur}/{len(f)};  after "
          f"release: {len(f) - dur}")
    ta = [r["t_after_release"] for r in f if r["t_after_release"] is not None]
    if ta:
        print(f"  release -> fall: median {np.median(ta):.2f} s "
              f"({min(ta):.2f}-{max(ta):.2f})")
    for sg, name in ((-1, "- (left)"), (+1, "+ (right)")):
        c = [r for r in rows if r["sign"] == sg]
        print(f"  error sign {name:9s}: {sum(r['tf'] is not None for r in c):3d}/"
              f"{len(c)} fell, of those to -roll "
              f"{sum(r['fall_side'] < 0 for r in c if r['tf']):d}")
    held = [r for r in rows if r["hold"] is not None and r["amp"] >= 1.0]
    print("  released pulses >= 1 deg, true roll (signed by the error):")
    print(f"    during hold: median min {np.nanmedian([r['min_during'] for r in held]):+.2f}"
          f" deg (negative = moved AWAY from the claimed side)")
    print(f"    after release: median max {np.nanmedian([r['max_after'] for r in held]):+.2f}"
          f" deg (positive = swung toward it)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--section", choices=("study", "inject", "replay", "motion",
                                          "shaped", "policies", "report"),
                    required=True)
    ap.add_argument("--seeds", type=int, default=64)
    ap.add_argument("--workers", type=int, default=9)
    a = ap.parse_args()
    ctx = mp.get_context("spawn")
    t0 = time.time()
    if a.section == "study":
        with ProcessPoolExecutor(a.workers, mp_context=ctx) as ex:
            per = list(ex.map(study_seed, range(a.seeds)))
            whole = list(ex.map(whole_flight, [(s, arm) for arm in ARMS
                                               for s in range(a.seeds)]))
        pickle.dump({"per": per, "whole": whole},
                    open(OUT / "ahrs_fall_cause_study.pkl", "wb"))
    elif a.section == "inject":
        with ProcessPoolExecutor(a.workers, mp_context=ctx) as ex:
            res = list(ex.map(inject_onset, [(o, s) for o in ONSETS_S
                                             for s in (-1.0, 1.0)]))
        pickle.dump(res, open(OUT / "ahrs_fall_cause_inject.pkl", "wb"))
    elif a.section == "policies":
        with ProcessPoolExecutor(a.workers, mp_context=ctx) as ex:
            res = list(ex.map(policy_flight, [(p, sd) for p in POLICIES
                                              for sd in range(a.seeds)]))
        pickle.dump(res, open(OUT / "ahrs_fall_cause_policies.pkl", "wb"))
    elif a.section == "shaped":
        with ProcessPoolExecutor(a.workers, mp_context=ctx) as ex:
            clean = list(ex.map(shaped_clean, [(o, sg) for o in ONSETS_S
                                               for sg in SHAPE_SIGNS]))
            noisy = list(ex.map(shaped_noisy, range(a.seeds)))
        pickle.dump({"clean": clean, "noisy": noisy},
                    open(OUT / "ahrs_fall_cause_shaped.pkl", "wb"))
    elif a.section == "motion":
        with ProcessPoolExecutor(a.workers, mp_context=ctx) as ex:
            res = list(ex.map(motion_flight, range(a.seeds)))
        pickle.dump(res, open(OUT / "ahrs_fall_cause_motion.pkl", "wb"))
    elif a.section == "replay":
        per = pickle.load(open(OUT / "ahrs_fall_cause_study.pkl", "rb"))["per"]
        recs = [r for r in per if r["tf"] is not None]
        with ProcessPoolExecutor(a.workers, mp_context=ctx) as ex:
            res = list(ex.map(replay_fall, [(r, w) for r in recs
                                            for w in ("roll", "roll+pitch")]))
        pickle.dump(res, open(OUT / "ahrs_fall_cause_replay.pkl", "wb"))
    else:
        for name, fn in (("study", report_study), ("inject", report_inject),
                         ("replay", report_replay), ("motion", report_motion),
                         ("shaped", report_shaped), ("policies", report_policies)):
            p = OUT / f"ahrs_fall_cause_{name}.pkl"
            if p.exists():
                fn(pickle.load(open(p, "rb")))
            else:
                print(f"(no {p.name}; run --section {name})")
    print(f"[{time.time() - t0:.0f} s]", file=sys.stderr)


if __name__ == "__main__":
    main()
