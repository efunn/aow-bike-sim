"""Pluck test: what on the bike rings, and at what frequency, read by the AHRS.

Why (10-25): the bike responds at 9 Hz 1.5-2.5x harder than the floor twin
(physical-bike-tests.md item 8), the flight records show a ~90 deg phase jump
between 6 and 7 Hz that the rigid twin has not got, and the two places the
bike can flex are the axle and the steerer tube. A part that rings near there
should show as a ring-down in the gyro after a pluck at it.

The AHRS does not need touching: a pluck anywhere rings the whole structure,
and the gyro hears it wherever it is buried. Where you pluck only decides
which modes get the most energy.

Guided: each spot is prompted in turn. Enter starts it, you pluck it --plucks
times (push a few mm, let go quickly; a doorbell press, never enough to move
the bike), ~3 s apart so each ring dies out, and Enter ends it. At the start
prompt `s` skips the spot; at the end prompt `r` redoes it. Hold the bike as
for the bench tests, wheels on the floor, one finger on the rollcage, the
same way at every spot including the baseline (the balancing is below
~2 Hz).

Servos LOCKED by default, as in flight: the drives at Goal Velocity 0 in
velocity mode at the flight gains, the steer held straight in extended
position at P 900 (drivetrain_bench --hold-steer; refused > 45 deg off), and
the wings (righting servo) held where they are in current-based position at
righting.module.gains and onboard.yaml's righting_current (`--no-wings`
leaves them limp; 10-25's first run had them limp, and limp wings can ring).
Each held servo is read back after the settle (torque, goal, shaft, Bus
Watchdog, error) and the run stops to ask if one is > 2 deg off its goal;
torque and error are read again at the end. All restored in `finally`.
`--limp` runs with torque off throughout.

Saves `<out>/tap.json` (spots, each one's start and end on the AHRS clock)
and `<out>/ahrs.npz` (drivetrain_bench.AhrsLog: chassis frame, the bundle's
mount, every sample), then prints per spot and axis the strongest 3-60 Hz
peak of the gyro spectrum against the baseline's.

Pi-safe (hw/ only, never mujoco). After `scripts/bike.sh sync`, on the Pi
from the repo root:

    ~/venv/bin/python analysis/servo/tap_test.py --note "upright on floor, one finger on rollcage"
    ~/venv/bin/python analysis/servo/tap_test.py --limp --tag limp
    python analysis/servo/tap_test.py --report traces/bench/tap_<stamp>   # re-print
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import drivetrain_bench as db                       # noqa: E402
import steer_response as sr                         # noqa: E402
from aow_sim.hw.dynamixel import MODE_CURRENT_POSITION  # noqa: E402

# (name, what to do). The baseline first: every spectrum is read against it.
SPOTS = (
    ("baseline", "hold as for every spot, touch nothing else"),
    ("steerer", "grip the fork at the crown, push it SIDEWAYS (left/right) a few mm, let go"),
    ("front_axle", "push the front tyre SIDEWAYS near its top, let go"),
    ("rear_axle", "push the rear tyre SIDEWAYS near its top, let go"),
    # Pluck size through the rollers' +-7.8 deg play (10-25): backlash rings
    # at a frequency that moves with amplitude, a spring mode mostly does not.
    ("rear_part", "push the rear tyre SIDEWAYS only PART of its free play -- let go "
                  "BEFORE you feel the end"),
    ("rear_end", "push the rear tyre SIDEWAYS until it stops at the END of its play, let go"),
    ("cage_side", "push the top of the rollcage SIDEWAYS with your other hand, let go"),
    ("cage_fore_aft", "push the top of the rollcage FORWARD, let go"),
    ("wing_left", "flick the left wing's tip"),
    ("wing_right", "flick the right wing's tip"),
    ("frame_knock", "knock the frame near the AHRS with a knuckle"),
    ("whole_wheelbase", "keep the wheels grounded while twisting the bike frame"),
)
BAND = (3.0, 60.0)       # Hz: above the balancing, below the sensor's Nyquist
SEG_S = 2.0              # Welch segment: 0.5 Hz bins
AXES = ("roll x", "pitch y", "yaw z")


def walk(spots, plucks, clock, ask=input, marks=None):
    """Prompt each spot; -> [{name, t0, t1}] on `clock`, appended to `marks`
    as each one ends (so a Ctrl-C keeps the finished ones). Skipped spots
    are left out; a redone spot keeps only its last take."""
    marks = [] if marks is None else marks
    k = 0
    while k < len(spots):
        name, how = spots[k]
        n = "" if name == "baseline" else f", {plucks} times ~3 s apart"
        got = ask(f"\n[{k + 1}/{len(spots)}] {name}: {how}{n}.\n"
                  f"  Enter to start (s skips): ").strip().lower()
        if got == "s":
            k += 1
            continue
        t0 = clock()
        got = ask("  ...recording. Enter when done (r redoes): ").strip().lower()
        t1 = clock()
        if got == "r":
            continue
        marks.append({"name": name, "t0": t0, "t1": t1})
        k += 1
    return marks


def _psd(t, x, fs):
    """Welch average of Hann segments, 50 % overlap. -> f, psd (or None)."""
    tu = np.arange(t[0], t[-1], 1 / fs)
    if len(tu) < 8:
        return None
    x = np.interp(tu, t, x)
    n = min(int(SEG_S * fs), len(x))
    w = np.hanning(n)
    segs = [x[i:i + n] for i in range(0, len(x) - n + 1, max(n // 2, 1))]
    P = np.mean([np.abs(np.fft.rfft((s - s.mean()) * w)) ** 2 for s in segs], axis=0)
    return np.fft.rfftfreq(n, 1 / fs), P / (fs * (w ** 2).sum())


def ring_table(t, gyro_deg, marks):
    """-> rows (name, axis, band RMS deg/s, peak Hz, peak over baseline,
    6-10 Hz over baseline). Ratios are NaN without a baseline."""
    fs = 1 / np.median(np.diff(t))
    spec = {}
    for m in marks:
        s = (t >= m["t0"]) & (t <= m["t1"])
        spec[m["name"]] = [_psd(t[s], gyro_deg[s, k], fs) if s.sum() > 8 else None
                           for k in range(3)]
    base = spec.get("baseline")
    rows = []
    for m in marks:
        for k, ax in enumerate(AXES):
            r = spec[m["name"]][k]
            if r is None:
                continue
            f, P = r
            b = (f >= BAND[0]) & (f <= min(BAND[1], fs / 2))
            rms = float(np.sqrt(np.sum(P[b]) * (f[1] - f[0])))
            ratio, mid = np.full(len(f), np.nan), np.nan
            if base is not None and base[k] is not None and m["name"] != "baseline":
                Pb = np.interp(f, *base[k])
                ratio = P / np.maximum(Pb, 1e-12)
                w = (f >= 6) & (f <= 10)
                mid = float(np.sum(P[w]) / max(np.sum(Pb[w]), 1e-12))
            pick = ratio if np.isfinite(ratio[b]).any() else P
            j = np.flatnonzero(b)[np.nanargmax(pick[b])]
            rows.append((m["name"], ax, rms, float(f[j]), float(ratio[j]), mid))
    return rows


def _band(x, fs, lo, hi):
    X = np.fft.rfft(x - x.mean())
    f = np.fft.rfftfreq(len(x), 1 / fs)
    X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, len(x))


def ring_peaks(t, gyro_deg, marks, fs=200.0, lo=2.0, hi=30.0, n_peaks=4):
    """Each pluck found and its ring-down's spectrum. A pluck is the first
    3-60 Hz roll+yaw sample over 6x the spot's median (and 1 deg/s), its
    peak within 0.15 s; the 1.5 s after it (from 30 ms on) is Hann-windowed
    and the spot's plucks' power spectra summed. -> {spot: (n plucks,
    [per axis: [(Hz, power / the largest), ...]])}, the n_peaks largest
    local maxima in [lo, hi] Hz, in frequency order."""
    tu = np.arange(t[0], t[-1], 1 / fs)
    G = np.stack([np.interp(tu, t, gyro_deg[:, k]) for k in range(3)], 1)
    H = np.stack([_band(G[:, k], fs, 3, 60) for k in range(3)], 1)
    L = np.stack([_band(G[:, k], fs, 0.7, 60) for k in range(3)], 1)
    mag = np.hypot(H[:, 0], H[:, 2])
    n = 2048
    f = np.fft.rfftfreq(n, 1 / fs)
    sel = np.flatnonzero((f >= lo) & (f <= hi))
    out = {}
    for m in marks:
        if m["name"] == "baseline":
            continue
        s = np.flatnonzero((tu >= m["t0"]) & (tu <= m["t1"]))
        if len(s) < fs:
            continue
        thr = max(6 * np.median(mag[s]), 1.0)
        ev, j = [], s[0]
        while j <= s[-1]:
            if mag[j] > thr:
                pk = j + int(np.argmax(mag[j:min(j + int(0.15 * fs), s[-1] + 1)]))
                ev.append(pk)
                j = pk + int(1.5 * fs)
            else:
                j += 1
        acc = np.zeros((3, len(f)))
        for pk in ev:
            for k in range(3):
                seg = L[pk + int(0.03 * fs): pk + int(1.5 * fs), k]
                if len(seg) > 16:
                    acc[k] += np.abs(np.fft.rfft(seg * np.hanning(len(seg)), n)) ** 2
        axes = []
        for k in range(3):
            S = acc[k]
            loc = [i for i in sel[1:-1] if S[i] > S[i - 1] and S[i] > S[i + 1]]
            top = S[sel].max() if S[sel].max() > 0 else 1.0
            loc = sorted(sorted(loc, key=lambda i: -S[i])[:n_peaks])
            axes.append([(float(f[i]), float(S[i] / top)) for i in loc])
        out[m["name"]] = (len(ev), axes, [float(tu[k]) for k in ev])
    return out


def pluck_rows(t, gyro_deg, rp, fs=200.0, lo=4.0, hi=30.0):
    """One row per pluck: (spot, time, peak |yaw rate| in the 0.3 s after,
    its yaw ring's strongest frequency in [lo, hi] over 1.5 s). The size
    against the frequency is what tells backlash (moves) from a spring mode
    (stays). Filtered 0.7-60 Hz as `ring_peaks` is: a brick wall at 2 Hz
    rang at 2.1 Hz on every large pluck (10-25, the first draft of this).
    `lo` 4 Hz keeps the 2.5-3.7 Hz rocking out of the pick."""
    tu = np.arange(t[0], t[-1], 1 / fs)
    yaw = _band(np.interp(tu, t, gyro_deg[:, 2]), fs, 0.7, 60)
    f = np.fft.rfftfreq(2048, 1 / fs)
    sel = (f >= lo) & (f <= hi)
    rows = []
    for name, (_n, _axes, times) in rp.items():
        for tp in times:
            k = int(np.searchsorted(tu, tp))
            seg = yaw[k + int(0.03 * fs): k + int(1.5 * fs)]
            if len(seg) < 16:
                continue
            seg = seg - np.polyval(np.polyfit(np.arange(len(seg)), seg, 1), np.arange(len(seg)))
            S = np.abs(np.fft.rfft(seg * np.hanning(len(seg)), 2048)) ** 2
            rows.append((name, tp, float(np.abs(yaw[k: k + int(0.3 * fs)]).max()),
                         float(f[sel][np.argmax(S[sel])])))
    return rows


def report(out: Path) -> None:
    meta = json.loads((out / "tap.json").read_text())
    z = np.load(out / "ahrs.npz")
    rows = ring_table(z["t"], np.degrees(z["gyro"]), meta["marks"])
    print(f"\n{out.name} ({meta.get('note') or 'no note'}; servos "
          f"{'limp' if meta['limp'] else 'locked'})")
    print("gyro, chassis frame. Peak = the 3-60 Hz bin most above the baseline "
          "(the plain peak for the baseline itself);\n'6-10 Hz' = that band's "
          "power over the baseline's.")
    print(f"  {'spot':14s} {'axis':8s} {'RMS deg/s':>9s} {'peak Hz':>8s} "
          f"{'x base':>7s} {'6-10 Hz':>8s}")
    for name, ax, rms, fp, rp, mid in rows:
        print(f"  {name:14s} {ax:8s} {rms:9.2f} {fp:8.1f} {rp:7.1f} {mid:8.1f}")
    empty = [m["name"] for m in meta["marks"] if m["name"] not in {r[0] for r in rows}]
    if empty:
        print(f"  no AHRS samples to speak of in: {', '.join(empty)}")
    rp = ring_peaks(z["t"], np.degrees(z["gyro"]), meta["marks"])
    print("\nring-downs: each spot's plucks' summed 1.5 s spectra, the largest local "
          "peaks 2-30 Hz,\nHz(power over that axis's largest)")
    for name, (n, axes, _) in rp.items():
        for k, ax in enumerate(AXES):
            pk = "  ".join(f"{fq:4.1f}({p:.2f})" for fq, p in axes[k])
            print(f"  {name if k == 0 else '':14s} {f'n={n}' if k == 0 else '':4s} {ax:8s} {pk}")
    print("\nper pluck: yaw rate peak (deg/s) and its ring's strongest frequency, 4-30 Hz")
    for name, tp, amp, fq in pluck_rows(z["t"], np.degrees(z["gyro"]), rp):
        print(f"  {name:14s} t {tp:6.1f}  yaw peak {amp:6.1f}  ring {fq:5.1f} Hz")
    if (out / "servos.npz").exists():
        shaft_rings(out, meta, rp)


class ShaftLog:
    """Present Position of the held servos, polled in a thread while the
    operator plucks (the main thread only waits on Enter then, so the bus is
    this thread's alone). One read per id per pass, as fast as the bus
    answers; each pass stamped on perf_counter, the AHRS's clock. Positions
    are 4-byte two's complement, so they are viewed as int32."""

    def __init__(self, bus, ids):
        self.bus, self.ids, self.rows = bus, tuple(ids), []
        self._stop = threading.Event()
        self._thread = None
        self.errors = 0

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True, name="shaft-log")
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                row = [self.bus.read_raw(i, "Present Position") for i in self.ids]
                self.rows.append((time.perf_counter(), *row))
            except Exception:                    # noqa: BLE001 -- keep logging
                self.errors += 1
            self._stop.wait(0.002)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def save(self, out: Path, t0: float) -> dict:
        a = np.array(self.rows, float).reshape(-1, 1 + len(self.ids))
        pos = (a[:, 1:].astype(np.int64) & 0xFFFFFFFF).astype(np.uint32).view(np.int32)
        t = a[:, 0] - t0
        np.savez(out / "servos.npz", t=t, ids=np.array(self.ids),
                 pos_rad=pos * (2 * np.pi / 4096))
        dt = np.diff(t)
        return {"ids": list(self.ids), "samples": int(len(t)), "read_errors": self.errors,
                "rate_hz": float(1 / np.median(dt)) if len(dt) else float("nan"),
                "file": "servos.npz"}


def shaft_rings(out: Path, meta: dict, rp: dict) -> None:
    """At each pluck found in the gyro: the rollers' ring-vs-hub angle
    (drivetrain_bench's diff, (sa q101 - sb q102)/2, servo degrees), its
    peak-to-peak in the 1.5 s after, and its spectrum's peaks, beside the
    gyro's. Moving at the gyro's ring frequency = the rollers ring with it."""
    z = np.load(out / "servos.npz")
    ids = [int(i) for i in z["ids"]]
    if not {101, 102} <= set(ids) or len(z["t"]) < 50:
        return
    t, q = z["t"], np.degrees(z["pos_rad"])
    sa, sb = 1.0, -1.0                                  # drivetrain_bench --signs 1 -1
    diff = (sa * q[:, ids.index(101)] - sb * q[:, ids.index(102)]) / 2
    steer = q[:, ids.index(db.STEER_ID)] if db.STEER_ID in ids else None
    fs = 1 / np.median(np.diff(t))
    print(f"\nshafts at each pluck ({fs:.0f} Hz): rollers ring-vs-hub and steer, servo deg "
          f"p-p over the 1.5 s after; ring-vs-hub spectrum peaks 2-30 Hz")
    for name, (n, axes, times) in rp.items():
        S, pps, spp = None, [], []
        for tp in times:
            w = (t >= tp + 0.03) & (t <= tp + 1.5)
            if w.sum() < 16:
                continue
            tu = np.arange(t[w][0], t[w][-1], 1 / fs)
            x = np.interp(tu, t[w], diff[w])
            pps.append(np.ptp(x))
            if steer is not None:
                spp.append(np.ptp(steer[w]))
            P = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x)), 2048)) ** 2
            S = P if S is None else S + P
        if S is None:
            continue
        f = np.fft.rfftfreq(2048, 1 / fs)
        sel = np.flatnonzero((f >= 2) & (f <= min(30, fs / 2)))
        loc = [i for i in sel[1:-1] if S[i] > S[i - 1] and S[i] > S[i + 1]]
        loc = sorted(sorted(loc, key=lambda i: -S[i])[:4])
        pk = "  ".join(f"{f[i]:4.1f}({S[i] / S[sel].max():.2f})" for i in loc)
        st = f", steer {np.median(spp):.2f}" if spp else ""
        print(f"  {name:14s} ring-vs-hub p-p {np.median(pps):5.2f}{st} | {pk}")


HELD_OFF_MAX_DEG = 2.0
CHECK_REGS = ("Torque Enable", "Goal Position", "Present Position", "Bus Watchdog",
              "Hardware Error Status")
WING_REGS = ("Operating Mode", "Position P Gain", "Position I Gain", "Position D Gain",
             "Goal Current")


def held_check(bus, i, goal, label, ask=input) -> dict:
    """Did a held servo get to its goal? 10-25's first run's steer ended 94
    counts (8.3 deg) from it at P 900, which a held shaft should not, and
    nothing said so (the operator: it seemed to go limp). Reads it back;
    off by more than HELD_OFF_MAX_DEG, torque off, or the goal not taken
    asks before the spots start (Ctrl-C there restores everything)."""
    got = {k: bus.read_raw(i, k) for k in CHECK_REGS}
    off = (got["Present Position"] - goal) * 360.0 / db.STEER_COUNTS_PER_REV
    got["off_deg"] = off
    print(f"{label} {i}: torque {got['Torque Enable']}, goal {got['Goal Position']} "
          f"(wrote {goal}), at {got['Present Position']} ({off:+.1f} deg), "
          f"watchdog {got['Bus Watchdog']}, error {got['Hardware Error Status']}")
    if abs(off) > HELD_OFF_MAX_DEG or got["Torque Enable"] != 1 or got["Goal Position"] != goal:
        ask(f"!! the {label} is NOT holding its goal ({off:+.1f} deg off). Enter "
            f"carries on anyway, Ctrl-C stops: ")
    return got


def end_state(bus, ids) -> dict:
    """Torque and error per held id just before the restore: did one drop
    out during the spots?"""
    return {i: {k: db._try(bus.read_raw, i, k) for k in
                ("Torque Enable", "Hardware Error Status", "Present Position")}
            for i in ids}


def wings_hold_on(bus, wid, gains, current) -> dict:
    """The righting servo held WHERE IT IS, as run_bike flies it: current-
    based position, righting.module.gains, Goal Current onboard.yaml's
    righting_current. Not moved to stow: on the floor that could shove the
    bike. -> what to restore, and the goal."""
    saved = {k: bus.read_raw(wid, k) for k in WING_REGS}
    bus.torque(False, (wid,))
    if saved["Operating Mode"] != MODE_CURRENT_POSITION:
        bus.write_raw(wid, "Operating Mode", MODE_CURRENT_POSITION)
    for k, v in gains.items():                     # after the mode: it resets them
        bus.write_raw(wid, k, int(v))
    goal = bus.read_raw(wid, "Present Position")
    bus.torque(True, (wid,))
    # Goals AFTER torque on: written with it off they are dropped
    # (control_tables/README.md), as the 10-25 steer hold found.
    bus.write_raw(wid, "Goal Position", goal)
    bus.write_raw(wid, "Goal Current", int(current))
    got = bus.read_raw(wid, "Goal Current")
    if got != int(current):
        raise RuntimeError(f"wings {wid}: wrote Goal Current {current}, read back {got}")
    return {"saved": saved, "goal": goal, "gains": dict(gains), "current": int(current)}


def wings_hold_off(bus, wid, hold) -> None:
    """Goal Current back while torque is still on (a goal written with it off
    may be dropped), then torque off, mode, gains -- every step tried.
    run_bike writes its own Goal Current at every start anyway."""
    saved = hold["saved"]
    steps = [lambda: bus.write_raw(wid, "Goal Current", saved["Goal Current"]),
             lambda: bus.torque(False, (wid,))]
    if saved["Operating Mode"] != MODE_CURRENT_POSITION:
        steps.append(lambda: bus.write_raw(wid, "Operating Mode", saved["Operating Mode"]))
    steps += [lambda k=k: bus.write_raw(wid, k, saved[k]) for k in WING_REGS[1:4]]
    for step in steps:
        if db._try(step) is db._FAILED:
            print(f"!! wing restore step failed (id {wid})")


def save(out, a, spots, marks, steer_hold, ahrs, wing_hold=None, end=None,
         shafts=None) -> None:
    out.mkdir(parents=True, exist_ok=True)
    t0 = marks[0]["t0"] - 1.0
    meta = {"note": a.note, "limp": a.limp, "plucks": a.plucks, "argv": sys.argv,
            "drive_gains": list(a.drive_gains), "steer_hold": steer_hold,
            "wing_hold": wing_hold, "end_state": end,
            "spots": {n: h for n, h in spots},
            "marks": [{**m, "t0": m["t0"] - t0, "t1": m["t1"] - t0} for m in marks]}
    meta["ahrs"] = ahrs.save(out, t0)
    if shafts is not None:
        meta["servos"] = shafts.save(out, t0)
    (out / "tap.json").write_text(json.dumps(meta, indent=1, default=str))
    print(f"\nsaved -> {out}")
    db._try(report, out)


def main(argv=None, ask=input, bus_cls=None, ahrs_log=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", default=db._ONBOARD["dxl_port"])
    ap.add_argument("--ahrs", default=db._ONBOARD["ahrs_port"], metavar="PORT")
    ap.add_argument("--bundle", type=Path, default=db.ROOT / "deploy" / "bundle.npz")
    ap.add_argument("--limp", action="store_true", help="torque off throughout")
    ap.add_argument("--drive-gains", type=lambda x: tuple(int(v) for v in x.split(":")),
                    default=(100, 1920), metavar="P:I")
    ap.add_argument("--no-wings", action="store_true",
                    help="leave the righting servo alone (limp) when locked")
    ap.add_argument("--plucks", type=int, default=3)
    ap.add_argument("--spots", nargs="+", choices=[s[0] for s in SPOTS],
                    help="a subset, in the default order (baseline is always first)")
    ap.add_argument("--note", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--out", type=Path, default=None,
                    help="default traces/bench/tap_<stamp>[_tag]")
    ap.add_argument("--report", type=Path, metavar="DIR", help="re-print a saved test")
    a = ap.parse_args(argv)
    if a.report:
        report(a.report)
        return 0
    spots = [s for s in SPOTS if a.spots is None or s[0] in a.spots or s[0] == "baseline"]
    stamp = time.strftime("%y%m%d-%H%M%S")
    out = a.out or db.ROOT / "traces" / "bench" / f"tap_{stamp}{'_' + a.tag if a.tag else ''}"

    drives = tuple(int(i) for i in db._ONBOARD["servo_ids"][:2])
    sid = db.STEER_ID
    print(f"pluck test: {len(spots)} spots ({', '.join(s[0] for s in spots)}), "
          f"{a.plucks} plucks each; servos {'LIMP (torque off)' if a.limp else 'LOCKED'}")
    wid = None if a.limp or a.no_wings else db._ONBOARD.get("righting_id")
    bus = drive_saved = steer_orig = steer_hold = wing_hold = ahrs = shafts = None
    end = None
    marks = []
    try:
        ahrs = ahrs_log or db.AhrsLog(a.ahrs, a.bundle)
        print(f"AHRS {a.ahrs}: first sample after {ahrs.start() * 1e3:.0f} ms, "
              f"mount {ahrs.mount_source}")
        if not a.limp:
            from aow_sim.params import DEFAULT_PARAMS
            import yaml
            raw = yaml.safe_load(Path(DEFAULT_PARAMS).read_text())
            hold_args = argparse.Namespace(steer_gains=raw["actuators"]["steer_gains"],
                                           steer_zero_deg=float(db._ONBOARD["steer_zero_deg"]))
            bus = (bus_cls or db.DynamixelBus)(a.port, ids=drives).open()
            held = drives + (sid,) + ((int(wid),) if wid else ())
            for i in held[2:]:
                db._attach(bus, i)
            errs = bus.hardware_errors(held)
            if errs:
                raise SystemExit(f"latched hardware error on {sorted(errs)}; clear it first")
            steer_orig = {k: bus.read_raw(sid, k) for k in
                          ("Operating Mode", "Position P Gain", "Position I Gain",
                           "Position D Gain")}
            present = bus.read_raw(sid, "Present Position")
            move = ((db._steer_straight(present, hold_args.steer_zero_deg) - present)
                    * 360.0 / db.STEER_COUNTS_PER_REV)
            if abs(move) > db.STEER_HOLD_MAX_DEG:
                raise SystemExit(f"the fork is {move:+.0f} deg from straight; turn it "
                                 f"nearer by hand first (limit {db.STEER_HOLD_MAX_DEG:g})")
            wings = f", wings {wid} held where they are" if wid else ", wings limp"
            ask(f"\nTorque ON: drives {list(drives)} held at 0, steer {sid} to straight "
                f"({move:+.1f} deg){wings}. Bike held, then Enter (Ctrl-C aborts): ")
            drive_saved = sr.drives_hold_on(bus, drives, a.drive_gains)
            steer_hold = {"id": sid}
            steer_hold.update(db._steer_hold_on(bus, sid, steer_orig, hold_args))
            if wid:
                wing_hold = {"id": int(wid)}
                wing_hold.update(wings_hold_on(bus, int(wid), raw["righting"]["module"]["gains"],
                                               int(db._ONBOARD["righting_current"])))
            time.sleep(0.5)
            steer_hold["check"] = held_check(bus, sid, steer_hold["goal"], "steer", ask)
            if wing_hold:
                wing_hold["check"] = held_check(bus, int(wid), wing_hold["goal"], "wings", ask)
            shafts = ShaftLog(bus, held)
            shafts.start()
        walk(spots, a.plucks, time.perf_counter, ask, marks)
    finally:
        if shafts is not None:
            shafts.stop()
        if bus is not None:
            end = end_state(bus, held)
            print(f"held servos at the end: {end}")
            if wing_hold is not None and "saved" in wing_hold:
                wings_hold_off(bus, int(wid), wing_hold)
            if drive_saved is not None:
                sr.drives_hold_off(bus, drives, drive_saved)
            if steer_hold is not None:
                db._steer_hold_off(bus, sid, steer_orig, steer_hold)
            db._try(bus.close)
        if ahrs is not None:
            ahrs.stop()
            if marks:                          # even after a Ctrl-C: keep what was done
                save(out, a, spots, marks, steer_hold, ahrs, wing_hold, end, shafts)
    return 0


if __name__ == "__main__":
    sys.exit(main())
