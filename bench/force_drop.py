"""Catch wheel drops on the force sensors; per impact: peak, contact, bounce.

Arms after 0.2 s with all four sensors unloaded; the next rise past 0.1 N on
any sensor is a drop. Zero: that 0.2 s. r + enter discards the last drop, q stops.
Heights are labels in order, by hand; with --cam, from the step that released it.

  e_measured   first-impact restitution, no drop height needed: flight time
               gives v_out, impulse gives v_in. Needs the impact mass (--mass-g,
               else the resting force). Synthetic drops: +0.01 to +0.06 high.
  h_measured   the drop height that implies
  e_ratio      second flight / first: no height or mass, second impact
  e_flight, e_impulse   from the set height; only as good as that height
  m_eff_g, a_mps2   the impact mass and the contact's fall acceleration: on
               the rig's hinged arm m_eff = I / r^2, not the resting force / g,
               and the contact falls at F_rest / m_eff (--m-eff-g, else
               config's arm.m_eff_g; by hand, the static mass and a = g)
  clean        a real flight after the first impact, and e_measured within
               0.05 at a 0.2 N edge; otherwise the bounce numbers are blank

Reading clips ~19 N; sensor safe load ~37 N. Raw + summary CSVs to bench/logs/.

    python bench/force_drop.py --wheel front --mass-g 86 --heights 1,2 --repeats 5

THE DROP RIG (--cam): the XL330 turns the snail cam and this script
fires each drop itself, so every drop's height is known from which step
released it. Current-based position mode, one Goal Current for every move,
always FORWARD: per step a setpoint on its top flat (--margin-deg short of the
step: the run-up) and one in the dwell past it (--park-deg), so the cam goes
_|_|_ lift, hold, drop, lift... Never backward: that drives the follower into
a step's undercut face. The port, the servo's ID, the calibration, the
cam's drops and the current are config/drop_rig_bench.yaml's (each flag
overrides one).

The cam sits on the horn pins in one position, so the encoder knows its
angle: Present Position mod 4096. Calibrate once per assembly (torque off):

    python bench/force_drop.py --cam --cam-where
      turn the cam by hand the way it drives; at the moment the FIRST
      of the cam's drops releases the follower, the printed ticks are
      cam.index and the printed sign cam.dir: put both in the yaml

    python bench/force_drop.py --wheel front --mass-g 86 --cam --repeats 3
    python bench/force_drop.py --reanalyse bench/logs/drops_<time>.csv --scale a,b,c,d
      a saved run's summary again from its raw counts, at a new calibration
    python bench/force_drop.py --cam --dry-run --repeats 1
      the cam alone, no force sensor and no files: each drop's edge
      speed, settling and cycle time (with --ahrs: the AHRS and servo
      logs, and a row per drop with its release on the host clock)

Start: wherever the cam is; it drives forward to the next top flat (if that
passes a step, the drop is not recorded) and zeroes there, wheel lifted.
It fires exactly heights x --repeats drops and writes a row for each: one
the sensor missed is outcome no_impact (or impact_lost), never re-fired,
since on the rig that should not happen; its trace is kept too, from
1 s before the cam parked to the 2 s that expired. still_loaded: a sensor
read over 0.1 N in the 0.2 s before the cam began its drop, or was hit
before it: the wheel was not lifted clear (pre_drop_n, every row). fall_ms
(impacts): release to impact, from where the cam's fitted motion crosses
the step tick -- so cam.index, the follower's corner and the clock map's
minimum delay all add one constant; fit t = t0 + sqrt(2h/a) across the
heights rather than reading one drop against free fall. Each drop records the step's
speed past the follower and warns under
sqrt(g * --corner-mm) / r_top (~280 deg/s): slower, the corner lets the
follower down instead of dropping it. q, Ctrl-C or an error: forward to the
next dwell, then torque off. On the Mac, run adjust-ftdi-latency after
plugging in the U2D2 (it refuses a bus slower than 5 ms a read).

THE AHRS (--ahrs --ahrs-at-mm X,Z[,Y]): the TM151 is logged for the whole
run, every frame, beside the drops (bench/ahrs_stream.py). X is the chip's
distance along the bar from the pivot, Z its height above the pivot's axis,
Y how far it sits outboard of the bar (on the fork, say; default 0), mm. Every cam run also logs every servo read of every move. Files, beside
the drops' CSVs:
  <run>_ahrs.csv    host_s, t_us (the TM151's clock), Euler, quaternion, gyro, acc
  <run>_servo.csv   host_s, ticks, goal_ticks
  <run>_run.json    the ports, the station, the arm, the cam, the command line
host_s is time.perf_counter(), one clock across this script's processes; each
drop's summary row has t_zero_host_s, its t_s = 0 on that clock. With
--dry-run (no force sensor) the summary has t_release_host_s instead: the
cam's fitted crossing of the step. Ports are
picked by USB vendor (Teensy 16c0, TM151 0483) and each checked by what it
sends; --port / --ahrs-port override.

    python bench/force_drop.py --wheel front --cam --repeats 3 \
        --hold-s 3 --settle-s 3 --ahrs --ahrs-at-mm 195,22
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))   # this checkout
import threading  # noqa: E402

from aow_sim.hw.dynamixel import (MODE_CURRENT_POSITION,  # noqa: E402
                                  DynamixelBus, describe_hardware_error,
                                  signed)

import ahrs_stream  # noqa: E402
from force_calibrate import Stream, marker  # noqa: E402
from force_sensor import ADC_MAX, CALIBRATED_N_PER_COUNT, CHANNELS, find_port  # noqa: E402

G = 9.80665
ARM_S = 0.2               # arms after this long unloaded; also each drop's zero reference
PRE_S, POST_S = ARM_S, 1.4
NO_IMPACT_S = 2.0         # after the cam's drop, no hit by then is a failed drop
MISS_PRE_S = 1.0          # a failed drop's trace starts this long before the cam parked
STREAM_SAMPLES = 60000    # ~6 s at 10 kHz: a failed drop's whole window is still there
HIT_N = 0.1               # contact edge; the mount rings +-0.05 N
MAX_CONTACT_MS = 30.0     # hand drops 7-9 ms; the rig's hinged arm 14-21 ms (2026-10-03)
MIN_FLIGHT_MS = 3.0
SMALL_PEAK_N = 1.0        # considered a knock
CLIP_COUNTS = ADC_MAX - 8
WARN_N = 19.0             # 0.68 V zero + 1.98 V span runs past the 3.3 V input
SAFE_N = 36.8             # datasheet: 2.5x rated


def analyse(t, f, h0_mm, mass_g, thr=None, m_eff_g=None):
    """Impacts, peak, contact time and the restitution estimates of one trace.

    Two masses. The STATIC one (mass_g, else the resting force / g) is what
    gravity pulls at the contact; the EFFECTIVE one (m_eff_g, else the static)
    is what the impact has to stop. They differ on a hinged arm -- m_eff =
    I_hinge / r^2 -- and then the contact falls and flies at a = F_static /
    m_eff, not g. With m_eff_g unset, a = g: a wheel dropped free."""
    thr = HIT_N if thr is None else thr
    on = f > thr
    edges = np.flatnonzero(np.diff(on.astype(int)))
    starts = [i + 1 for i in edges if not on[i]]
    ends = [i + 1 for i in edges if on[i]]
    if on[0]:
        starts = [0] + starts
    hits = []
    for s in starts:
        e = next((x for x in ends if x > s), len(f))
        hits.append((s, e))
    # gaps under 1 ms are one contact
    merged = []
    for s, e in hits:
        if merged and t[s] - t[merged[-1][1] - 1] < 1e-3:
            merged[-1] = (merged[-1][0], e)
        else:
            merged.append((s, e))
    if not merged:
        return None
    s1, e1 = merged[0]
    out = dict(peak_n=float(f[s1:e1].max()), contact_ms=(t[e1 - 1] - t[s1]) * 1e3,
               bounces=len(merged) - 1)
    rest = f[t > t[-1] - 0.2]
    resting = bool(rest.min() > thr)
    out["rest_n"] = float(rest.mean()) if resting else float("nan")
    # --mass-g, else the resting force (same sensor, so a scale error cancels in J / m)
    f_s = mass_g * 1e-3 * G if mass_g else (out["rest_n"] if resting else float("nan"))
    out["mass_from"] = "given" if mass_g else ("rest" if resting else "none")
    out["mass_g"] = f_s / G * 1e3
    m = m_eff_g * 1e-3 if m_eff_g else f_s / G
    out["m_eff_g"] = m * 1e3
    out["m_eff_from"] = "given" if m_eff_g else "static"
    a = f_s / m                       # the contact's fall and flight, m/s^2
    out["a_mps2"] = a
    h0 = h0_mm * 1e-3
    v_in = np.sqrt(2 * a * h0)
    if len(merged) > 1 and merged[1][0] < len(t):
        flight = t[merged[1][0]] - t[e1 - 1]
        out["flight_ms"] = flight * 1e3
        out["e_flight"] = float(np.sqrt(a * flight ** 2 / 8 / h0))
    else:
        out["flight_ms"] = out["e_flight"] = float("nan")
    if len(merged) > 2:
        flight2 = t[merged[2][0]] - t[merged[1][1] - 1]
        out["e_ratio"] = float(flight2 / (t[merged[1][0]] - t[e1 - 1]))
    else:
        out["e_ratio"] = float("nan")
    impulse = float(np.trapezoid(f[s1:e1], t[s1:e1]))
    out["impulse_mns"] = impulse * 1e3
    # J = m_eff (v_in + v_out) + F_static T
    t_c = t[e1 - 1] - t[s1]
    out["e_impulse"] = (impulse - f_s * t_c) / (m * v_in) - 1 if m == m else float("nan")
    if m == m and out["flight_ms"] == out["flight_ms"]:
        v_out = a * out["flight_ms"] * 1e-3 / 2
        v_in_m = (impulse - f_s * t_c) / m - v_out
        out["h_measured_mm"] = v_in_m ** 2 / (2 * a) * 1e3
        out["e_measured"] = float(v_out / v_in_m)
    else:
        out["h_measured_mm"] = out["e_measured"] = float("nan")
    # a wheel that rocks on the button never unloads: no flight, no bounce numbers
    out["clean"] = bool(out["contact_ms"] < MAX_CONTACT_MS
                        and out["flight_ms"] == out["flight_ms"]
                        and out["flight_ms"] >= MIN_FLIGHT_MS)
    if not out["clean"]:
        for key in ("e_flight", "e_impulse", "e_measured", "h_measured_mm", "e_ratio"):
            out[key] = float("nan")
    return out


def measure(t, counts, h_mm, mass_g, scale, m_eff_g=None):
    """One recorded impact, from its raw counts: t (s, 0 = the impact) and
    counts (n, 4). Zero: the counts before the impact. Returns the summary's
    analysis columns, the lines worth printing, and the force (n, 4); None
    for the columns if no impact survives the analysis. Shared by the live
    run and --reanalyse, so a new calibration re-derives the same columns."""
    zero = counts[t < -0.002].mean(axis=0)
    force = (counts - zero) * np.asarray(scale)
    k = int(force[t >= 0].max(axis=0).argmax())                    # the sensor it hit
    a = analyse(t, force[:, k], h_mm, mass_g, m_eff_g=m_eff_g)
    if a is None:
        return None, [], force
    notes = []
    a2 = analyse(t, force[:, k], h_mm, mass_g, thr=2 * HIT_N, m_eff_g=m_eff_g)
    a["e_measured_2x_edge"] = a2["e_measured"]
    if a["clean"] and not abs(a2["e_measured"] - a["e_measured"]) <= 0.05:
        a["clean"] = False
        notes.append(f"  marginal: e_measured {a['e_measured']:.2f} at a {HIT_N:g} N edge, "
                     f"{a2['e_measured']:.2f} at {2 * HIT_N:g} N (rocking or a slow second contact)")
    if a["peak_n"] < SMALL_PEAK_N:
        notes.append(f"  peak only {a['peak_n']:.2f} N: probably a knock, not a drop (r + enter discards)")
    others = max(force[t >= 0][:, i].max() for i in range(4) if i != k)
    cols = dict(sensor=k + 1, zero_counts=round(zero[k], 1), scale_n_per_count=float(scale[k]),
                clipped=bool(counts[:, k].max() >= CLIP_COUNTS), others_peak_n=round(others, 4),
                **{x: (round(v, 4) if isinstance(v, float) else v) for x, v in a.items()})
    return cols, notes, force


ANALYSIS_COLS = ("sensor", "zero_counts", "scale_n_per_count", "clipped", "others_peak_n",
                 "peak_n", "contact_ms", "bounces", "rest_n", "flight_ms", "e_flight", "e_ratio",
                 "mass_from", "mass_g", "m_eff_from", "m_eff_g", "a_mps2", "impulse_mns", "e_impulse",
                 "h_measured_mm", "e_measured", "clean", "e_measured_2x_edge")


def reanalyse(raw_path, scale, mass_g=None, m_eff_g=None) -> Path:
    """Re-derive a saved run's summary from its raw counts at `scale` (N per
    count, per sensor). Labels, outcomes the sensor never saw (no_impact,
    still_loaded) and the cam's columns are kept as recorded; every impact is
    measured again; m_eff_g, else the run's own (a run from before it: none,
    so a = g as recorded). At the contact's height: the run's h_contact_mm,
    else for a cam run the rig as configured now (contact_drop). Writes <run>_summary_reanalysed.csv beside it."""
    raw_path = Path(raw_path)
    if raw_path.name.endswith("_summary.csv"):
        raw_path = raw_path.with_name(raw_path.name.replace("_summary.csv", ".csv"))
    sum_path = raw_path.with_name(raw_path.stem + "_summary.csv")
    for f in (raw_path, sum_path):
        if not f.exists():
            sys.exit(f"--reanalyse needs both {raw_path.name} and {sum_path.name} in one "
                     f"directory: {f} is missing")
    with open(raw_path) as fh:
        head = fh.readline().strip().split(",")
        if any(f"{ch}_counts" not in head for ch in CHANNELS):
            sys.exit(f"{raw_path.name} has no per-sensor counts ({', '.join(head)}): an older "
                     f"one-channel log, which --reanalyse does not read")
        fh.seek(0)
        traces: dict = {}
        for r in csv.DictReader(fh):
            traces.setdefault(int(r["drop"]), []).append(
                (float(r["t_s"]), [float(r[f"{ch}_counts"]) for ch in CHANNELS]))
    with open(sum_path) as fh:
        rows = list(csv.DictReader(fh))
    out = []
    print(f"re-analysing {raw_path.name} at --scale {','.join(f'{x:g}' for x in scale)}")
    for row in rows:
        d, outcome = int(row["drop"]), row.get("outcome") or "ok"     # hand-mode files predate it
        new = dict(row)                       # the recorded order; analysis values replaced
        if outcome in ("ok", "impact_lost") and d in traces:
            for x in ANALYSIS_COLS:
                new[x] = ""
            t, counts = (np.array(x) for x in zip(*traces[d]))
            m = mass_g or (float(row["mass_g"]) if row.get("mass_from") == "given" else None)
            me = m_eff_g or (float(row["m_eff_g"]) if row.get("m_eff_from") == "given" else None)
            if row.get("h_contact_mm"):
                h_c = float(row["h_contact_mm"])
            else:                   # a cam run from before the column: the rig as configured now
                h_c = (contact_drop if row.get("cam_step") else float)(float(row["height_mm"]))
            cols, notes, _ = measure(t, counts.astype(float), h_c, m, scale, me)
            if row.get("cam_step") or row.get("h_contact_mm"):
                cols = None if cols is None else dict(h_contact_mm=round(h_c, 4), **cols)
            new["outcome"] = "ok" if cols else "impact_lost"
            new.update(cols or {})
            def num(x):
                try:
                    return float(x)
                except (TypeError, ValueError):
                    return float("nan")
            print(f"  drop {d}: {new['outcome']}, peak {num(row.get('peak_n')):.3f} -> "
                  f"{num(new.get('peak_n')):.3f} N, e_measured {num(row.get('e_measured')):.3f} -> "
                  f"{num(new.get('e_measured')):.3f}, mass {num(row.get('mass_g')):.1f} -> "
                  f"{num(new.get('mass_g')):.1f} g, m_eff {num(new.get('m_eff_g')):.1f} g")
        else:
            print(f"  drop {d}: {outcome}, kept as recorded")
        out.append(new)
    dest = raw_path.with_name(raw_path.stem + "_summary_reanalysed.csv")
    cols = list(dict.fromkeys(c for r in out for c in r))
    with open(dest, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        w.writerows(out)
    print(f"wrote {dest}")
    return dest


TICKS = 4096              # per turn
# "Arrived": only ends a wait, never places anything -- the top setpoint has
# ~30 deg of flat behind it and the park sits 12 deg into a 20 deg dwell, so
# >= 4 deg of slack either way. Too tight only risks a false "stuck" if the
# current-limited loop settles short. top_err_deg / park_err_deg in the
# summary are the settled errors: set this from their worst case.
POS_TOL = 45              # ticks (~4 deg)
BUS_TRIES = 3             # per register access (Cam._bus)


BENCH_CFG = Path(__file__).resolve().parents[1] / "config" / "drop_rig_bench.yaml"


def arm_lever(cfg=None) -> float:
    """Contact drop per follower drop: the arm's r_contact / r_follower. The
    cam's heights are at the follower; the wheel drops this many times more."""
    import yaml
    arm = (cfg or yaml.safe_load(BENCH_CFG.read_text())).get("arm") or {}
    return arm["r_contact_mm"] / arm["r_follower_mm"] if "r_follower_mm" in arm else 1.0


def contact_drop(h_mm, cfg=None) -> float:
    """The contact's drop for a cam step labelled h_mm: the steps' differences
    are exact, their absolute heights short by one arm.step_offset_mm, all at
    the follower; then the lever."""
    import yaml
    arm = (cfg or yaml.safe_load(BENCH_CFG.read_text())).get("arm") or {}
    return (h_mm - arm.get("step_offset_mm", 0.0)) * arm_lever(cfg)


def cam_config(args) -> dict:
    """config/drop_rig_bench.yaml, each value overridden by its flag if given."""
    import yaml
    cfg = yaml.safe_load(BENCH_CFG.read_text())
    sv, cm = cfg["servo"], cfg["cam"]
    out = dict(port=args.cam_port or sv["port"], id=args.cam_id or sv["id"],
               index=cm["index"] if args.cam_index is None else args.cam_index,
               dir=args.cam_dir or cm["dir"], ma=args.cam_ma or cm["ma"],
               heights=cm["heights"], gains=dict(sv.get("gains") or {}),
               m_eff_g=(cfg.get("arm") or {}).get("m_eff_g"), cfg=cfg)
    if not out["port"]:
        out["port"] = os.environ.get("AOW_DXL_PORT")
    if not out["port"]:
        import glob
        found = sorted(glob.glob("/dev/cu.usbserial-*") + glob.glob("/dev/ttyUSB*"))
        if len(found) != 1:
            sys.exit(f"U2D2: {len(found)} candidate ports {found}; set servo.port in "
                     f"{BENCH_CFG.name} or pass --cam-port")
        out["port"] = found[0]
    if out["dir"] not in (-1, 1, None):
        sys.exit(f"cam.dir in {BENCH_CFG.name} must be -1 or 1")
    return out


def open_cam_bus(port: str, dxl_id: int):
    return DynamixelBus(port, ids=(dxl_id,)).open()


class Cam:
    """The drop rig's XL330. Step k (any integer) releases heights[k % n] at
    `index + dir * k * TICKS / n`; every move is forward and current-limited."""

    def __init__(self, bus, dxl_id, direction, index, n, park_deg, margin_deg, ma,
                 gains=None):
        self.bus, self.id, self.dir, self.n = bus, dxl_id, direction, n
        deg = TICKS / 360.0
        self.park, self.margin = round(park_deg * deg), round(margin_deg * deg)
        latched = bus.hardware_errors(ids=(dxl_id,))
        if latched:
            raise RuntimeError(
                f"id {dxl_id} has a latched hardware error "
                f"({describe_hardware_error(latched[dxl_id])}): it would accept torque "
                f"and not move. Power-cycle it (or reboot it in Dynamixel Wizard) "
                f"once the cause is clear.")
        bus.prepare(ids=(dxl_id,))    # torque off, return delay 0, profiles 0
        # The mode every run, then the gains: writing Operating Mode resets the
        # position gains to that mode's own values (control_tables/README.md),
        # printed here so the first run records what they are.
        bus.write_raw(dxl_id, "Operating Mode", MODE_CURRENT_POSITION)
        self.gains = {k: int(v) for k, v in (gains or {}).items()}
        reset = {k: bus.read_raw(dxl_id, k) for k in self.gains}
        for k, v in self.gains.items():
            bus.write_raw(dxl_id, k, v)
        bad = {k: bus.read_raw(dxl_id, k) for k in self.gains}
        bad = {k: got for k, got in bad.items() if got != self.gains[k]}
        if bad:
            raise RuntimeError(f"id {dxl_id}: gains did not stick, read back {bad}")
        if self.gains:
            print("gains: the mode write left " + ", ".join(f"{k} {v}" for k, v in reset.items())
                  + "; written " + ", ".join(f"{k} {v}" for k, v in self.gains.items()))
        self.ma_cap = bus.read_raw(dxl_id, "Current Limit")
        self.ma = min(int(ma), self.ma_cap)
        t0 = time.perf_counter()
        for _ in range(20):
            pos = self.pos()
        self.read_ms = (time.perf_counter() - t0) / 20 * 1e3
        if self.read_ms > 5.0:     # 16 ms: ~6 deg a read, one or none near the step
            raise RuntimeError(f"the bus reads at {self.read_ms:.0f} ms, too slow to measure the "
                               f"edge speed: run adjust-ftdi-latency (macOS, after every U2D2 plug-in)")
        phase = (direction * (pos - index)) % TICKS
        self.base = pos - direction * phase           # step 0, at or behind the cam
        self.k = None
        self.log = None     # a list: every read of every move, (host perf_counter, ticks, goal)

    def _bus(self, fn, *a):
        """One register access, retried on a dropped reply: run 20261003-2224
        died at drop 66 of 120 on a single rx timeout (rc -3001), just after
        the laptop idled. Goal writes are idempotent, so a retry is safe;
        each is printed. Three in a row raise."""
        for attempt in range(BUS_TRIES):
            try:
                return fn(self.id, *a)
            except RuntimeError as e:
                if attempt == BUS_TRIES - 1:
                    raise
                print(f"  !! servo bus: {e}; retrying")
                time.sleep(0.01)

    def pos(self) -> int:
        return signed(self._bus(self.bus.read_raw, "Present Position"), 4)

    def step(self, k) -> int:
        return self.base + self.dir * round(k * TICKS / self.n)

    def ahead(self, target) -> int:
        return (target - self.pos()) * self.dir

    def go(self, target, timeout_s=3.0, trace=None, ma=None) -> None:
        """Forward to `target` and wait; `trace` fills with (t_s, ticks), read flat out.
        Goal Current is written with every move (`ma`, default the configured one)."""
        self._bus(self.bus.write_raw, "Goal Current", self.ma if ma is None else min(int(ma), self.ma_cap))
        self._bus(self.bus.write_raw, "Goal Position", target & 0xFFFFFFFF)
        t0 = time.perf_counter()
        if trace is not None:
            self.trace_t0 = t0        # the trace's zero, host perf_counter
        while True:
            ta = time.perf_counter()
            p = self.pos()
            tm = (ta + time.perf_counter()) / 2     # stamped mid-read: the read's latency cancels
            if trace is not None:
                trace.append((tm - t0, p))
            if self.log is not None:
                self.log.append((tm, p, target))
            if abs(p - target) <= POS_TOL:
                return
            if time.perf_counter() - t0 > timeout_s:
                err = self.bus.read_raw(self.id, "Hardware Error Status")
                why = (f"hardware error: {describe_hardware_error(err)}" if err
                       else f"jammed, or {self.ma} mA too little")
                raise RuntimeError(f"cam stuck at {p} -> {target}: {why}")
            if trace is None:
                time.sleep(0.005)

    def home(self) -> bool:
        """Torque on where it stands, then forward to the next top flat.
        True if that passed a step (an unrecorded drop)."""
        pos = self.pos()
        self.bus.write_raw(self.id, "Goal Position", pos & 0xFFFFFFFF)
        self.torque(True)
        seg = TICKS / self.n
        phase = (pos - self.base) * self.dir
        self.k = math.ceil((phase + self.margin - POS_TOL) / seg)
        self.go(self.step(self.k) - self.dir * self.margin, timeout_s=5.0)
        return (self.k - 1) * seg > phase

    def fire(self, hold_s, on_drop=None) -> tuple[int, list]:
        """Lift to step k's top flat, hold, drop into its dwell. Returns k and
        the drop's position trace; `on_drop()` runs as the drop move starts."""
        k = self.k
        top = self.step(k) - self.dir * self.margin
        self.go(top, timeout_s=5.0)
        time.sleep(hold_s)
        self.top_err_deg = self.err_deg(top)
        if on_drop is not None:
            on_drop()
        trace = []
        self.go(self.step(k) + self.dir * self.park, trace=trace)
        self.k = k + 1
        return k, trace

    def err_deg(self, target) -> float:
        """Where it settled against `target`, deg; + is past it (forward)."""
        return -self.ahead(target) * 360.0 / TICKS

    def stop(self) -> None:
        """Forward to the next dwell (a drop, if it is on a top flat), torque off."""
        try:
            if self.k is not None:
                j = self.k - 1
                while self.ahead(self.step(j) + self.dir * self.park) < -POS_TOL:
                    j += 1
                self.go(self.step(j) + self.dir * self.park, timeout_s=5.0)
        finally:
            self.torque(False)

    def torque(self, on: bool) -> None:
        """A confirmed write, not the generic SyncWrite (which returns no
        status): a torque-off that did not land raises."""
        self._bus(self.bus.write_raw, "Torque Enable", int(on))


EDGE_WINDOW_DEG = 5.0      # either side of the step


def edge_fit(trace, at_tick) -> tuple[float, int, float]:
    """The step passing: a least-squares line through every sample within
    EDGE_WINDOW_DEG of `at_tick`. Returns its speed in deg/s, how many samples,
    and when it crosses `at_tick` (the trace's clock). One pair of samples
    would carry a tick's quantisation and two reads' timing jitter; the fit
    averages both. nan with fewer than 4."""
    w = EDGE_WINDOW_DEG * TICKS / 360.0
    pts = [(t, p) for t, p in trace if abs(p - at_tick) <= w]
    if len(pts) < 4:
        return float("nan"), len(pts), float("nan")
    t, p = np.array(pts).T
    slope, icpt = np.polyfit(t - t.mean(), p, 1)
    return abs(slope) * 360.0 / TICKS, len(pts), float(t.mean() + (at_tick - icpt) / slope)


def edge_speed(trace, at_tick) -> tuple[float, int]:
    """deg/s as the step passes, and how many samples were fitted (edge_fit)."""
    return edge_fit(trace, at_tick)[:2]


def cam_report(args, rig, k, trace, h) -> dict:
    """One drop's cam columns, and a line saying whether the release was fast."""
    step_tick = rig.step(k)
    v, n_fit = edge_speed(trace, step_tick)
    need = math.degrees(math.sqrt(G * args.corner_mm * 1e-3) / ((args.r_rest_mm + h) * 1e-3))
    cols = dict(cam_step=k, cam_ma=rig.ma, edge_deg_s=round(v, 1), edge_fit_n=n_fit,
                top_err_deg=round(rig.top_err_deg, 2),
                park_err_deg=round(rig.err_deg(step_tick + rig.dir * rig.park), 2))
    if v != v:
        print(f"  edge speed not measured ({n_fit} reads within "
              f"{EDGE_WINDOW_DEG:g} deg of the step)")
    elif v < need:
        print(f"  !! edge {v:.0f} deg/s, under ~{need:.0f}: a slow release "
              f"(more cam.ma, or more --margin-deg run-up)")
    else:
        print(f"  edge {v:.0f} deg/s ({n_fit} reads fitted); settled "
              f"{cols['top_err_deg']:+.1f} deg at the top, {cols['park_err_deg']:+.1f} parked")
    return cols


def dry_run(args, rig, heights, steps, rows=None) -> None:
    """The cam alone: every drop of the run, back to back, no sensor. `rows`
    (a list) fills with one row per drop: its cam columns, when the drop move
    began and when the step's edge passed the follower (t_release_host_s,
    the fitted crossing), on the host clock."""
    print(f"dry run: {len(steps)} drops, no force sensor.  Ctrl-C to stop\n")
    for i in range(len(steps)):
        t0 = time.perf_counter()
        h = heights[rig.k % len(heights)]
        print(f"drop {i + 1}/{len(steps)}: {h:g} mm, cam step {rig.k % len(heights) + 1}")
        stamp = {}
        k, trace = rig.fire(args.hold_s, on_drop=lambda: stamp.update(t=time.perf_counter()))
        cols = cam_report(args, rig, k, trace, h)
        if rows is not None:
            t_edge = edge_fit(trace, rig.step(k))[2]
            rows.append(dict(drop=i + 1, height_mm=h,
                             h_contact_mm=round(getattr(args, "contact_drop", lambda x: x)(h), 4),
                             t_drop_host_s=round(stamp["t"], 6),
                             t_release_host_s=round(rig.trace_t0 + t_edge, 6), **cols))
        time.sleep(args.settle_s)
        print(f"  cycle {time.perf_counter() - t0:.2f} s")


def cam_where(port, dxl_id) -> None:
    """Torque off; print the shaft angle as it is turned by hand."""
    bus = open_cam_bus(port, dxl_id)
    try:
        bus.write_raw(dxl_id, "Torque Enable", 0)
        print(f"{port}, id {dxl_id}: torque off. Turn the cam the way it drives; "
              f"Ctrl-C to stop.\n"
              f"  cam.index: the ticks at the moment the FIRST drop releases the follower\n"
              f"  cam.dir:   the sign shown while turning it that way")
        last = signed(bus.read_raw(dxl_id, "Present Position"), 4)
        seen = "?"
        while True:
            time.sleep(0.1)
            p = signed(bus.read_raw(dxl_id, "Present Position"), 4)
            sign = "1" if p > last + 2 else "-1" if p < last - 2 else ""
            seen = sign or seen
            print(f"\r  {p % TICKS:4d} ticks  {p % TICKS * 360 / TICKS:6.1f} deg   "
                  f"dir {sign:>2}  ", end="", flush=True)
            last = p
    except KeyboardInterrupt:
        print(f"\n\nin config/{BENCH_CFG.name}, under cam:\n"
              f"  index: <the ticks at the drop>\n  dir: {seen}")
    finally:
        bus.close()


def keys(cmds: list) -> None:
    """r / q typed at any time."""
    for line in sys.stdin:
        cmds.append(line.strip().lower())
    cmds.append("q")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--wheel", help="label, e.g. front / rear-flat-roller (required except "
                                    "--cam-where and --dry-run)")
    ap.add_argument("--heights", default=None,
                    help="mm, a label for grouping drops (default 1,2,3); with --cam, "
                         "the cam's drops in rotation order (default: the yaml's cam.heights)")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--order", choices=("block", "cycle"), default="block",
                    help="by hand: block 1,1,1,2,2,2 or cycle 1,2,1,2 (--cam ignores it)")
    ap.add_argument("--mass-g", type=float, default=None,
                    help="impact mass; use it when anything supports the wheel at rest "
                         "(front wheel + fork halves weigh 86 g)")
    ap.add_argument("--m-eff-g", type=float, default=None,
                    help="effective mass at the contact, g (a hinged arm's I / r^2); with --cam "
                         "the yaml's arm.m_eff_g, otherwise the static mass (a free drop)")
    ap.add_argument("--port", default=None, help="the force sensor's")
    ap.add_argument("--reanalyse", metavar="CSV", default=None,
                    help="re-derive a saved run's summary from its raw counts (drops_<time>.csv), "
                         "at --scale; writes <run>_summary_reanalysed.csv")
    ap.add_argument("--scale", default=None,
                    help="N per count for the four sensors, comma-separated (default: "
                         "force_sensor.CALIBRATED_N_PER_COUNT); with --reanalyse")
    cam = ap.add_argument_group("the drop rig's cam (see the docstring)")
    cam.add_argument("--cam", action="store_true",
                     help=f"drive the cam; the settings below default to config/{BENCH_CFG.name}")
    cam.add_argument("--cam-port", default=None,
                     help="the U2D2 (default: servo.port, else AOW_DXL_PORT, else the one usbserial)")
    cam.add_argument("--dry-run", action="store_true",
                     help="the cam alone: fire the run's drops with no force sensor, record nothing")
    cam.add_argument("--cam-where", action="store_true",
                     help="torque off, print the angle while it is turned by hand: calibration")
    cam.add_argument("--cam-id", type=int, default=None)
    cam.add_argument("--cam-index", type=int, default=None,
                     help="ticks (mod 4096) where the first drop releases (default cam.index)")
    cam.add_argument("--cam-dir", type=int, choices=(-1, 1), default=None,
                     help="the sign of the ticks as the cam turns forward (default cam.dir)")
    cam.add_argument("--cam-ma", type=float, default=None,
                     help="Goal Current for every move, mA (capped at the Current Limit)")
    cam.add_argument("--margin-deg", type=float, default=30.0,
                     help="hold this far short of the step: on its top flat, the run-up")
    cam.add_argument("--park-deg", type=float, default=12.0,
                     help="park this far past a step, in the dwell (0.87 mm clear, cad_drop_rig)")
    cam.add_argument("--hold-s", type=float, default=0.5, help=f"on the top flat; > {ARM_S} s to arm")
    cam.add_argument("--settle-s", type=float, default=0.3, help="after each drop's record")
    cam.add_argument("--r-rest-mm", type=float, default=13.0,
                     help="cam radius under the resting follower (r_dwell + gap); top = this + drop")
    cam.add_argument("--corner-mm", type=float, default=0.45,
                     help="cam + follower corner radii, summed (print rounding)")
    imu = ap.add_argument_group("the AHRS (see the docstring)")
    imu.add_argument("--ahrs", action="store_true", help="log the TM151 for the whole run")
    imu.add_argument("--ahrs-port", default=None,
                     help="default: the one port with USB vendor 0483 (STMicroelectronics)")
    imu.add_argument("--ahrs-at-mm", default=None, metavar="X,Z[,Y]",
                     help="the chip's station: along the bar from the pivot, above the "
                          "pivot's axis, outboard of the bar (default 0), mm (required with --ahrs)")
    args = ap.parse_args()
    if args.reanalyse:
        scale = ([float(x) for x in args.scale.split(",")] if args.scale
                 else list(CALIBRATED_N_PER_COUNT))
        if len(scale) != 4:
            sys.exit("--scale takes four values, one per sensor")
        reanalyse(args.reanalyse, scale, args.mass_g, args.m_eff_g)
        return
    station = None
    if args.ahrs:
        try:
            station = [float(v) for v in (args.ahrs_at_mm or "").split(",")]
            assert len(station) in (2, 3)
            station += [0.0] * (3 - len(station))
        except (ValueError, AssertionError):
            sys.exit("--ahrs needs --ahrs-at-mm X,Z[,Y]: the chip along the bar from the "
                     "pivot, above the pivot's axis, and outboard of the bar, mm")
    cc = cam_config(args) if (args.cam or args.cam_where) else None
    if args.cam_where:
        return cam_where(cc["port"], cc["id"])
    if cc and args.m_eff_g is None:
        args.m_eff_g = cc["m_eff_g"]
    args.contact_drop = (lambda h: contact_drop(h, cc["cfg"])) if cc else (lambda h: h)
    if args.dry_run and not args.cam:
        sys.exit("--dry-run drives the cam: add --cam")
    if not args.wheel and not args.dry_run:
        sys.exit("--wheel is required")
    if cc:
        if cc["index"] is None or cc["dir"] is None:
            sys.exit(f"cam.index / cam.dir are not set in config/{BENCH_CFG.name}: "
                     f"calibrate with --cam --cam-where")
        if args.park_deg <= 0 or args.margin_deg <= 0:
            sys.exit("--park-deg and --margin-deg must be positive")
    scale = np.array(CALIBRATED_N_PER_COUNT)
    if args.heights:
        heights = [float(h) for h in args.heights.split(",")]
    else:
        heights = [float(h) for h in cc["heights"]] if cc else [1.0, 2.0, 3.0]
    if args.cam:
        seg_deg = 360.0 / len(heights)
        if args.park_deg + args.margin_deg >= seg_deg:
            sys.exit(f"--park-deg + --margin-deg must stay under one segment, {seg_deg:g} deg")
    if args.order == "cycle":
        steps = [(h, r) for r in range(1, args.repeats + 1) for h in heights]
    else:
        steps = [(h, r) for h in heights for r in range(1, args.repeats + 1)]
    stem = free_stem(Path(__file__).resolve().parent / "logs" / f"drops_{datetime.now():%Y%m%d-%H%M}")
    if sys.platform == "darwin":        # no idle sleep mid-run; ends with this process
        import subprocess
        subprocess.Popen(["caffeinate", "-i", "-w", str(os.getpid())])
    stem.parent.mkdir(exist_ok=True)
    stream = ahrs = None
    ports, dry_rows = {}, []
    if not args.dry_run:
        ports["force"] = args.port or find_port()
    if args.ahrs:
        ports["ahrs"] = args.ahrs_port or ahrs_stream.find_port()
        if ports["ahrs"] == ports.get("force"):
            sys.exit(f"the force sensor and the TM151 are both on {ports['force']}")
        ahrs = ahrs_stream.AhrsStream(ports["ahrs"])
        print(f"ahrs: {ports['ahrs']}, Combo at {ahrs.wait_ready():.0f} Hz, "
              f"at {station[0]:g} mm from the pivot, {station[1]:g} mm above it")
    if not args.dry_run:
        stream = Stream(ports["force"], maxlen=STREAM_SAMPLES)
        time.sleep(1.0)
    rig = None
    if args.cam:
        rig = Cam(open_cam_bus(cc["port"], cc["id"]), cc["id"], cc["dir"], cc["index"],
                  len(heights), args.park_deg, args.margin_deg, cc["ma"], gains=cc["gains"])
        ports["cam"] = cc["port"]
        if not args.dry_run or args.ahrs:
            rig.log = []
        try:
            if rig.home():
                print("homing passed a step: that drop is not recorded")
        except BaseException:
            rig.stop()
            raise
        print(f"cam: {cc['port']} id {cc['id']}, {rig.ma} mA, on step {rig.k % len(heights) + 1}'s top flat "
              f"({heights[rig.k % len(heights)]:g} mm next)")
        time.sleep(1.0)
    try:
        if args.dry_run:
            dry_run(args, rig, heights, steps, dry_rows if args.ahrs else None)
        else:
            run(args, stream, rig, heights, steps, scale, stem)
    finally:
        if rig is not None:
            rig.stop()
            rig.bus.close()
            print("cam parked, torque off")
        if not args.dry_run or args.ahrs:
            save_logs(args, stem, rig, ahrs, ports, station, cc, heights, dry_rows)


def free_stem(stem: Path) -> Path:
    """`stem`, or stem-2, -3...: the first no file starts with, beside it or in
    its archive/. Two runs in a minute, or a Pi without a clock (it boots on
    its last saved time, and repeated 12:28 on 2026-10-07), would otherwise
    overwrite a run."""
    n, cand = 1, stem
    while any(d.exists() and any(d.glob(cand.name + "*"))
              for d in (cand.parent, cand.parent / "archive")):
        n += 1
        cand = stem.with_name(f"{stem.name}-{n}")
    return cand


def save_logs(args, stem, rig, ahrs, ports, station, cc, heights, dry_rows=()) -> None:
    """The run's AHRS frames and servo reads, whole, on the host clock -- into
    archive/ (gitignored, Dropbox-synced: ~3.5 MB a run) -- and
    <run>_run.json saying what the run was. The drops' rows join them by
    t_zero_host_s; a dry run's (`dry_rows`, written here as its summary) by
    t_release_host_s."""
    big = Path(stem).parent / "archive" / Path(stem).name
    big.parent.mkdir(exist_ok=True)
    files = {}
    if dry_rows:
        cols = list(dict.fromkeys(c for r in dry_rows for c in r))
        with open(f"{stem}_summary.csv", "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols)
            w.writeheader()
            w.writerows(dry_rows)
        files["summary"] = dict(file=f"{Path(stem).name}_summary.csv", drops=len(dry_rows),
                                force_sensor=False)
        print(f"wrote {stem}_summary.csv ({len(dry_rows)} drops, no force sensor)")
    if ahrs is not None:
        ahrs.close()
        n = ahrs.write(f"{big}_ahrs.csv")
        files["ahrs"] = dict(file=f"archive/{big.name}_ahrs.csv", frames=n,
                             crc_bad=ahrs.crc_bad, resync_bytes=ahrs.resync_bytes)
        print(f"wrote {big}_ahrs.csv ({n} frames, {ahrs.crc_bad} CRC failures)")
    if rig is not None and rig.log:
        with open(f"{big}_servo.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(("host_s", "ticks", "goal_ticks"))
            w.writerows((f"{t:.6f}", p, g) for t, p, g in list(rig.log))
        files["servo"] = dict(file=f"archive/{big.name}_servo.csv", reads=len(rig.log))
        print(f"wrote {big}_servo.csv ({len(rig.log)} reads)")
    if not files:
        return
    meta = dict(command=" ".join(sys.argv), written=datetime.now().isoformat(timespec="seconds"),
                ports=ports, files=files, wheel=args.wheel,
                clock="host_s: time.perf_counter() (Linux CLOCK_MONOTONIC, macOS mach time); "
                      "drops: t_zero_host_s per summary row")
    if station is not None:
        meta["ahrs_station_mm"] = dict(along_bar_from_pivot=station[0], above_pivot=station[1],
                                       outboard_of_bar=station[2] if len(station) > 2 else 0.0)
    if cc is not None:
        meta["arm"] = cc["cfg"].get("arm")
        meta["cam"] = dict(id=cc["id"], index=cc["index"], dir=cc["dir"], ma=rig.ma if rig else cc["ma"],
                           heights_mm=heights, ticks_per_turn=TICKS, park_deg=args.park_deg,
                           margin_deg=args.margin_deg, hold_s=args.hold_s, settle_s=args.settle_s)
    with open(f"{stem}_run.json", "w") as fh:
        json.dump(meta, fh, indent=2, default=str)
    print(f"wrote {stem}_run.json")


def run(args, stream, rig, heights, steps, scale, stem) -> None:
    """The capture loop. With `rig`, it fires exactly len(steps) drops, labels
    each by its step, and records every one: a drop the sensor did not see
    is a row with outcome no_impact, impact_lost or still_loaded, never a
    re-fire."""
    with stream.lock:
        t_last = stream.buf[-1][0]
        zero = np.mean([c for t, c in stream.buf if t > t_last - 0.5], axis=0)
    print(f"watching all four sensors, {len(steps)} drops.")
    print("keys, any time:  r + enter = discard the last drop   q + enter = stop and save\n")

    cmds: list = []
    threading.Thread(target=keys, args=(cmds,), daemon=True).start()
    raw, summary = [], []
    seen = t_last
    quiet_since = t_last
    armed, prompted = True, False
    firing = None                 # the cam's drop in flight
    fired, stopping = 0, False    # the cam's drops started; q pressed

    def host(t_sensor):
        """A sensor time on the host clock, the AHRS and servo logs' (6 decimals)."""
        return round(stream.host_time(t_sensor), 6)

    def failed(f, outcome, cam_cols, t_zero="cam_parked", at=None):
        summary.append(dict(drop=f["n"], wheel=args.wheel, height_mm=f["h"], outcome=outcome,
                            t_zero=t_zero, t_zero_host_s=host(f["t_parked"] if at is None else at),
                            **cam_cols))

    def now():
        """This moment on the sensor's clock. Not the newest sample's time:
        that lags by however far behind the samples are arriving."""
        return stream.sensor_time(time.perf_counter())

    def fire(f):
        try:
            f["k"], f["trace"] = rig.fire(args.hold_s, on_drop=lambda: f.update(t_drop=now()))
            f["trace_t0"] = rig.trace_t0
            f["t_parked"] = now()
        except BaseException as e:
            f["error"] = e
        f["done"] = time.time()

    def loaded_n(f):
        """The most any sensor read over the ARM_S before the cam began its
        drop, N past the zero: over HIT_N, the wheel was not lifted clear."""
        with stream.lock:
            c = [c for t, c in stream.buf if f["t_drop"] - ARM_S <= t <= f["t_drop"]]
        return float(((np.asarray(c, dtype=float) - zero) * scale).max()) if c else float("nan")

    def fall_ms(f, hit):
        """Release to impact, ms: the step's edge passing the follower (the
        cam's fitted crossing of the step tick, mapped to the sensor's clock)
        to the first sample over HIT_N."""
        _, _, t_edge = edge_fit(f["trace"], rig.step(f["k"]))
        return (hit - stream.sensor_time(f["trace_t0"] + t_edge)) * 1e3

    def keep(drop, t, counts):
        """The raw record: counts only, so any calibration can be applied
        later (--reanalyse). t_s from the summary's t_zero for that drop."""
        for j in range(len(t)):
            raw.append(dict(drop=drop, t_s=round(t[j], 6),
                            **{f"{ch}_counts": int(counts[j, i]) for i, ch in enumerate(CHANNELS)}))

    try:
        while (len(summary) < len(steps)) if rig is None else (fired < len(steps) or firing):
            while cmds:
                c = cmds.pop(0)
                if c == "q" and rig is not None:
                    stopping = True
                    if firing is None:
                        break
                    print("  stopping after this drop")
                elif c == "q":
                    steps = steps[:len(summary)]
                elif c == "r" and summary:
                    d = summary.pop()["drop"]
                    raw = [x for x in raw if x["drop"] != d]
                    prompted = False
                    print("  discarded the last drop")
            if rig is None and len(summary) >= len(steps):
                break
            if rig is not None and firing is None and (stopping or fired >= len(steps)):
                break
            if rig is None:
                h, r = steps[len(summary)]
                if armed and not prompted:
                    print(f"{args.wheel:>6} {h:4g} mm  drop {r}/{args.repeats}  -- armed, drop when ready")
                    prompted = True
            else:
                if firing is None:
                    k = rig.k
                    h = heights[k % len(heights)]
                    fired += 1
                    r = sum(1 for j in range(k - fired + 1, k + 1) if heights[j % len(heights)] == h)
                    print(f"{args.wheel:>6} {h:4g} mm  drop {r}/{args.repeats}  "
                          f"({fired}/{len(steps)}) -- cam step {k % len(heights) + 1}")
                    firing = dict(h=h, n=fired, done=None, error=None)
                    firing["thread"] = threading.Thread(target=fire, args=(firing,), daemon=True)
                    firing["thread"].start()
                h = firing["h"]
                if firing["error"] is not None:
                    raise firing["error"]
                # timed on the sensor's clock, so the whole window is in the buffer;
                # the host clock only backs it up if the stream stalls
                if firing["done"] is not None and (
                        seen >= firing["t_parked"] + NO_IMPACT_S
                        or time.time() - firing["done"] > NO_IMPACT_S + 1.0):
                    pre = loaded_n(firing)
                    outcome = "still_loaded" if pre > HIT_N else "no_impact"
                    if outcome == "still_loaded":
                        print(f"  !! the sensor read {pre:.2f} N as the cam began the drop: "
                              f"recorded as still_loaded, with its trace")
                    else:
                        print(f"  !! no impact within {NO_IMPACT_S:g} s of the drop: recorded as "
                              f"no_impact, with its trace")
                    t0 = firing["t_parked"]
                    with stream.lock:
                        data = [(t, c) for t, c in stream.buf
                                if t0 - MISS_PRE_S <= t <= t0 + NO_IMPACT_S]
                    if data:            # t_s from the moment the cam parked
                        counts = np.array([x[1] for x in data], dtype=float)
                        keep(firing["n"], np.array([x[0] for x in data]) - t0, counts)
                    failed(firing, outcome, dict(cam_report(args, rig, firing["k"], firing["trace"], h),
                                                 pre_drop_n=round(pre, 4)))
                    firing = None
                    continue
            time.sleep(0.02)
            with stream.lock:
                new = [(t, c) for t, c in stream.buf if t > seen]
            if not new:
                continue
            seen = new[-1][0]
            hit = None
            for t, c in new:
                fn = float(((np.asarray(c) - zero) * scale).max())   # the most loaded sensor
                if fn <= HIT_N:
                    if quiet_since is None:
                        quiet_since = t
                    continue
                if armed and quiet_since is not None and t - quiet_since >= ARM_S:
                    hit = t
                    break
                quiet_since = None
            if quiet_since is not None and not armed and seen - quiet_since >= ARM_S:
                armed, prompted = True, False
            if hit is None:
                continue

            while True:
                with stream.lock:
                    if stream.buf[-1][0] >= hit + POST_S:
                        data = [(t, c) for t, c in stream.buf if hit - PRE_S <= t <= hit + POST_S]
                        break
                time.sleep(0.02)
            seen = data[-1][0]
            armed, quiet_since = False, None
            cam_cols, f_drop = {}, firing
            if firing is not None:
                firing["thread"].join(timeout=5.0)
                if firing["error"] is not None:
                    raise firing["error"]
                pre = loaded_n(firing)
                cam_cols = dict(cam_report(args, rig, firing["k"], firing["trace"], h),
                                pre_drop_n=round(pre, 4))
                firing = None
                if pre > HIT_N or hit < f_drop["t_drop"]:
                    # loaded before the cam dropped: not this drop's impact
                    print(f"  !! loaded before the cam began the drop ({pre:.2f} N over the "
                          f"{ARM_S:g} s before it): recorded as still_loaded, with its trace")
                    # from before the drop, so the load that caused it is in the trace
                    with stream.lock:
                        data = [(t, c) for t, c in stream.buf
                                if min(f_drop["t_drop"] - MISS_PRE_S, hit - PRE_S) <= t <= hit + POST_S]
                    t = np.array([x[0] for x in data]) - f_drop["t_parked"]
                    counts = np.array([x[1] for x in data], dtype=float)
                    keep(f_drop["n"], t, counts)
                    failed(f_drop, "still_loaded", cam_cols)
                    time.sleep(args.settle_s)
                    continue
            if f_drop is not None:
                cam_cols["fall_ms"] = round(fall_ms(f_drop, hit), 2)
            t = np.array([x[0] for x in data]) - hit
            counts = np.array([x[1] for x in data], dtype=float)          # (n, 4)
            h_c = getattr(args, "contact_drop", lambda x: x)(h)    # the cam's heights are the follower's
            cols, notes, _ = measure(t, counts, h_c, args.mass_g, scale, args.m_eff_g)
            if cols is not None:
                cols = dict(h_contact_mm=round(h_c, 4), **cols)
            zero = counts[t < -0.002].mean(axis=0)
            if cols is None and f_drop is not None:
                print("  !! impact lost: recorded as impact_lost, with its trace")
                keep(f_drop["n"], t, counts)
                failed(f_drop, "impact_lost", cam_cols, t_zero="impact", at=hit)
                time.sleep(args.settle_s)
                continue
            if cols is None:
                print("  impact lost, not recorded")
                continue
            for note in notes:
                print(note)
            a = cols
            k = a["sensor"] - 1
            drop = len(summary) + 1 if f_drop is None else f_drop["n"]
            keep(drop, t, counts)
            summary.append(dict(drop=drop, wheel=args.wheel, height_mm=h, outcome="ok",
                                t_zero="impact", t_zero_host_s=host(hit), **cam_cols, **cols))
            if not a["clean"]:
                print("  no clean flight after the first impact (it rocked or rolled on the"
                      " button): peak kept, bounce numbers blank")
            print(f"  sensor {k + 1} {marker(k)}: e_measured {a['e_measured']:.2f} "
                  f"(contact drop measured {a['h_measured_mm']:.2f} mm, set {h_c:.2f}), "
                  f"peak {a['peak_n']:.2f} N, contact {a['contact_ms']:.1f} ms, "
                  f"e_ratio {a['e_ratio']:.2f}, bounces {a['bounces']}, resting {a['rest_n']:.3f} N")
            if "fall_ms" in cam_cols:
                print(f"  release to impact {cam_cols['fall_ms']:.1f} ms "
                      f"(free fall from {h:g} mm: {math.sqrt(2 * h * 1e-3 / G) * 1e3:.1f})")
            if counts[:, k].max() >= CLIP_COUNTS:
                print("  !! ADC SATURATED: the peak is clipped; drop lower")
            elif a["peak_n"] > WARN_N * 0.8:
                print(f"  !! within 20% of the ~{WARN_N:g} N where the reading clips; "
                      f"sensor safe load ~{SAFE_N:.0f} N")
            if rig is None:
                print("  lift the wheel off to re-arm")
            else:
                time.sleep(args.settle_s)
    finally:
        if firing is not None:
            firing["thread"].join(timeout=10.0)
        if summary:
            for path, rows in ((f"{stem}.csv", raw), (f"{stem}_summary.csv", summary)):
                if not rows:
                    continue
                cols = list(dict.fromkeys(c for row in rows for c in row))   # a failed row has fewer
                with open(path, "w", newline="") as fh:
                    w = csv.DictWriter(fh, fieldnames=cols)
                    w.writeheader()
                    w.writerows(rows)
                print(f"wrote {path}")


if __name__ == "__main__":
    main()
