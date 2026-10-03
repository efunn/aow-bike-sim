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

Start: wherever the cam is; it drives forward to the next top flat (if that
passes a step, the drop is not recorded) and zeroes there, wheel lifted.
Each drop records the step's speed past the follower and warns under
sqrt(g * --corner-mm) / r_top (~280 deg/s): slower, the corner lets the
follower down instead of dropping it. q, Ctrl-C or an error: forward to the
next dwell, then torque off. On the Mac, run adjust-ftdi-latency after
plugging in the U2D2 (it warns if the bus reads slower than 5 ms).
"""
from __future__ import annotations

import argparse
import csv
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

from force_calibrate import Stream, marker  # noqa: E402
from force_sensor import ADC_MAX, CALIBRATED_N_PER_COUNT, CHANNELS, find_port  # noqa: E402

G = 9.80665
ARM_S = 0.2               # arms after this long unloaded; also each drop's zero reference
PRE_S, POST_S = ARM_S, 1.4
HIT_N = 0.1               # contact edge; the mount rings +-0.05 N
MAX_CONTACT_MS = 15.0     # front: 7-9 ms measured
MIN_FLIGHT_MS = 3.0
SMALL_PEAK_N = 1.0        # considered a knock
CLIP_COUNTS = ADC_MAX - 8
WARN_N = 19.0             # 0.68 V zero + 1.98 V span runs past the 3.3 V input
SAFE_N = 36.8             # datasheet: 2.5x rated


def analyse(t, f, h0_mm, mass_g, thr=None):
    """Impacts, peak, contact time and the restitution estimates of one trace."""
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
    h0 = h0_mm * 1e-3
    v_in = np.sqrt(2 * G * h0)
    if len(merged) > 1 and merged[1][0] < len(t):
        flight = t[merged[1][0]] - t[e1 - 1]
        out["flight_ms"] = flight * 1e3
        out["e_flight"] = float(np.sqrt(G * flight ** 2 / 8 / h0))
    else:
        out["flight_ms"] = out["e_flight"] = float("nan")
    if len(merged) > 2:
        flight2 = t[merged[2][0]] - t[merged[1][1] - 1]
        out["e_ratio"] = float(flight2 / (t[merged[1][0]] - t[e1 - 1]))
    else:
        out["e_ratio"] = float("nan")
    # --mass-g, else the resting force (same sensor, so a scale error cancels in J / m)
    m = mass_g * 1e-3 if mass_g else (out["rest_n"] / G if resting else float("nan"))
    out["mass_from"] = "given" if mass_g else ("rest" if resting else "none")
    out["mass_g"] = m * 1e3
    impulse = float(np.trapezoid(f[s1:e1], t[s1:e1]))
    out["impulse_mns"] = impulse * 1e3
    # J = m (v_in + v_out) + m g T
    t_c = t[e1 - 1] - t[s1]
    out["e_impulse"] = (impulse - m * G * t_c) / (m * v_in) - 1 if m == m else float("nan")
    if m == m and out["flight_ms"] == out["flight_ms"]:
        v_out = G * out["flight_ms"] * 1e-3 / 2
        v_in_m = (impulse - m * G * t_c) / m - v_out
        out["h_measured_mm"] = v_in_m ** 2 / (2 * G) * 1e3
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


TICKS = 4096              # per turn
# "Arrived": only ends a wait, never places anything -- the top setpoint has
# ~30 deg of flat behind it and the park sits 12 deg into a 20 deg dwell, so
# >= 4 deg of slack either way. Too tight only risks a false "stuck" if the
# current-limited loop settles short. top_err_deg / park_err_deg in the
# summary are the settled errors: set this from their worst case.
POS_TOL = 45              # ticks (~4 deg)


BENCH_CFG = Path(__file__).resolve().parents[1] / "config" / "drop_rig_bench.yaml"


def cam_config(args) -> dict:
    """config/drop_rig_bench.yaml, each value overridden by its flag if given."""
    import yaml
    cfg = yaml.safe_load(BENCH_CFG.read_text())
    sv, cm = cfg["servo"], cfg["cam"]
    out = dict(port=args.cam_port or sv["port"], id=args.cam_id or sv["id"],
               index=cm["index"] if args.cam_index is None else args.cam_index,
               dir=args.cam_dir or cm["dir"], ma=args.cam_ma or cm["ma"],
               heights=cm["heights"], gains=dict(sv.get("gains") or {}))
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
        if self.read_ms > 5.0:
            print(f"!! the bus reads at {self.read_ms:.0f} ms: run adjust-ftdi-latency (macOS, "
                  f"after every U2D2 plug-in), or the edge speed cannot be measured")
        phase = (direction * (pos - index)) % TICKS
        self.base = pos - direction * phase           # step 0, at or behind the cam
        self.k = None

    def pos(self) -> int:
        return signed(self.bus.read_raw(self.id, "Present Position"), 4)

    def step(self, k) -> int:
        return self.base + self.dir * round(k * TICKS / self.n)

    def ahead(self, target) -> int:
        return (target - self.pos()) * self.dir

    def go(self, target, timeout_s=3.0, trace=None, ma=None) -> None:
        """Forward to `target` and wait; `trace` fills with (t_s, ticks), read flat out.
        Goal Current is written with every move (`ma`, default the configured one)."""
        self.bus.write_raw(self.id, "Goal Current", self.ma if ma is None else min(int(ma), self.ma_cap))
        self.bus.write_raw(self.id, "Goal Position", target & 0xFFFFFFFF)
        t0 = time.perf_counter()
        while True:
            ta = time.perf_counter()
            p = self.pos()
            if trace is not None:     # stamped mid-read: the read's latency cancels
                trace.append(((ta + time.perf_counter()) / 2 - t0, p))
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

    def fire(self, hold_s) -> tuple[int, list]:
        """Lift to step k's top flat, hold, drop into its dwell. Returns k and
        the drop's position trace."""
        k = self.k
        top = self.step(k) - self.dir * self.margin
        self.go(top, timeout_s=5.0)
        time.sleep(hold_s)
        self.top_err_deg = self.err_deg(top)
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
        self.bus.write_raw(self.id, "Torque Enable", int(on))


EDGE_WINDOW_DEG = 5.0      # either side of the step


def edge_speed(trace, at_tick) -> tuple[float, int]:
    """deg/s as the step passes: the least-squares slope of every sample within
    EDGE_WINDOW_DEG of `at_tick`, and how many there were. One pair of samples
    would carry a tick's quantisation and two reads' timing jitter; the fit
    averages both. nan with fewer than 4."""
    w = EDGE_WINDOW_DEG * TICKS / 360.0
    pts = [(t, p) for t, p in trace if abs(p - at_tick) <= w]
    if len(pts) < 4:
        return float("nan"), len(pts)
    t, p = np.array(pts).T
    return abs(np.polyfit(t, p, 1)[0]) * 360.0 / TICKS, len(pts)


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
    ap.add_argument("--wheel", help="label, e.g. rear / front (required except --cam-where)")
    ap.add_argument("--phase", default="", help="rear roller phase label, e.g. flat")
    ap.add_argument("--heights", default=None,
                    help="mm, a label for grouping drops (default 1,2,3); with --cam, "
                         "the cam's drops in rotation order (default: the yaml's cam.heights)")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--order", choices=("block", "cycle"), default="block",
                    help="by hand: block 1,1,1,2,2,2 or cycle 1,2,1,2 (--cam ignores it)")
    ap.add_argument("--mass-g", type=float, default=None,
                    help="impact mass; use it when anything supports the wheel at rest "
                         "(front wheel + fork halves weigh 86 g)")
    ap.add_argument("--port", default=None, help="the force sensor's")
    cam = ap.add_argument_group("the drop rig's cam (see the docstring)")
    cam.add_argument("--cam", action="store_true",
                     help=f"drive the cam; the settings below default to config/{BENCH_CFG.name}")
    cam.add_argument("--cam-port", default=None,
                     help="the U2D2 (default: servo.port, else AOW_DXL_PORT, else the one usbserial)")
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
    args = ap.parse_args()
    cc = cam_config(args) if (args.cam or args.cam_where) else None
    if args.cam_where:
        return cam_where(cc["port"], cc["id"])
    if not args.wheel:
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
    stem = Path(__file__).resolve().parent / "logs" / f"drops_{datetime.now():%Y%m%d-%H%M}"
    stream = Stream(args.port or find_port())
    time.sleep(1.0)
    rig = None
    if args.cam:
        rig = Cam(open_cam_bus(cc["port"], cc["id"]), cc["id"], cc["dir"], cc["index"],
                  len(heights), args.park_deg, args.margin_deg, cc["ma"], gains=cc["gains"])
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
        run(args, stream, rig, heights, steps, scale, stem)
    finally:
        if rig is not None:
            rig.stop()
            rig.bus.close()
            print("cam parked, torque off")


def run(args, stream, rig, heights, steps, scale, stem) -> None:
    """The capture loop. With `rig`, it fires each drop and labels it by step."""
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

    def fire(f):
        try:
            f["k"], f["trace"] = rig.fire(args.hold_s)
        except BaseException as e:
            f["error"] = e
        f["done"] = time.time()

    try:
        while len(summary) < len(steps):
            while cmds:
                c = cmds.pop(0)
                if c == "q":
                    steps = steps[:len(summary)]
                elif c == "r" and summary:
                    d = summary.pop()["drop"]
                    raw = [x for x in raw if x["drop"] != d]
                    prompted = False
                    print("  discarded the last drop")
            if len(summary) >= len(steps):
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
                    r = sum(1 for x in summary if x["height_mm"] == h) + 1
                    print(f"{args.wheel:>6} {h:4g} mm  drop {r}/{args.repeats}  -- cam step {k % len(heights) + 1}")
                    firing = dict(h=h, done=None, error=None)
                    firing["thread"] = threading.Thread(target=fire, args=(firing,), daemon=True)
                    firing["thread"].start()
                h = firing["h"]
                if firing["error"] is not None:
                    raise firing["error"]
                if firing["done"] is not None and time.time() - firing["done"] > 2.0:
                    print("  no impact seen after the drop: not recorded")
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
            cam_cols = {}
            if firing is not None:
                firing["thread"].join(timeout=5.0)
                if firing["error"] is not None:
                    raise firing["error"]
                step_tick = rig.step(firing["k"])
                v, n_fit = edge_speed(firing["trace"], step_tick)
                need = math.degrees(math.sqrt(G * args.corner_mm * 1e-3)
                                    / ((args.r_rest_mm + h) * 1e-3))
                cam_cols = dict(cam_step=firing["k"], cam_ma=rig.ma, edge_deg_s=round(v, 1),
                                edge_fit_n=n_fit, top_err_deg=round(rig.top_err_deg, 2),
                                park_err_deg=round(rig.err_deg(step_tick + rig.dir * rig.park), 2))
                if v != v:
                    print(f"  edge speed not measured ({n_fit} reads within "
                          f"{EDGE_WINDOW_DEG:g} deg of the step)")
                elif v < need:
                    print(f"  !! edge {v:.0f} deg/s, under ~{need:.0f}: a slow release "
                          f"(more cam.ma, or more --margin-deg run-up)")
                else:
                    print(f"  edge {v:.0f} deg/s ({n_fit} reads fitted); settled "
                          f"{cam_cols['top_err_deg']:+.1f} deg at the top, "
                          f"{cam_cols['park_err_deg']:+.1f} parked")
                firing = None
            t = np.array([x[0] for x in data]) - hit
            counts = np.array([x[1] for x in data], dtype=float)          # (n, 4)
            zero = counts[t < -0.002].mean(axis=0)
            force = (counts - zero) * scale
            k = int(force[t >= 0].max(axis=0).argmax())                    # the sensor it hit
            a = analyse(t, force[:, k], h, args.mass_g)
            if a is None:
                print("  impact lost, not recorded")
                continue
            a2 = analyse(t, force[:, k], h, args.mass_g, thr=2 * HIT_N)
            a["e_measured_2x_edge"] = a2["e_measured"]
            if a["clean"] and not abs(a2["e_measured"] - a["e_measured"]) <= 0.05:
                a["clean"] = False
                print(f"  marginal: e_measured {a['e_measured']:.2f} at a {HIT_N:g} N edge, "
                      f"{a2['e_measured']:.2f} at {2 * HIT_N:g} N (rocking or a slow second contact)")
            if a["peak_n"] < SMALL_PEAK_N:
                print(f"  peak only {a['peak_n']:.2f} N: probably a knock, not a drop (r + enter discards)")
            drop = len(summary) + 1
            for j in range(len(t)):
                raw.append(dict(drop=drop, t_s=round(t[j], 6),
                                **{f"{ch}_counts": int(counts[j, i]) for i, ch in enumerate(CHANNELS)},
                                **{f"{ch}_n": round(force[j, i], 4) for i, ch in enumerate(CHANNELS)}))
            others = max(force[t >= 0][:, i].max() for i in range(4) if i != k)
            summary.append(dict(drop=drop, wheel=args.wheel, phase=args.phase, height_mm=h,
                                sensor=k + 1, zero_counts=round(zero[k], 1),
                                scale_n_per_count=scale[k],
                                clipped=bool(counts[:, k].max() >= CLIP_COUNTS),
                                others_peak_n=round(others, 4), **cam_cols,
                                **{x: (round(v, 4) if isinstance(v, float) else v)
                                   for x, v in a.items()}))
            if not a["clean"]:
                print("  no clean flight after the first impact (it rocked or rolled on the"
                      " button): peak kept, bounce numbers blank")
            print(f"  sensor {k + 1} {marker(k)}: e_measured {a['e_measured']:.2f} "
                  f"(height measured {a['h_measured_mm']:.2f} mm, set {h:g}), "
                  f"peak {a['peak_n']:.2f} N, contact {a['contact_ms']:.1f} ms, "
                  f"e_ratio {a['e_ratio']:.2f}, bounces {a['bounces']}, resting {a['rest_n']:.3f} N")
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
                with open(path, "w", newline="") as fh:
                    w = csv.DictWriter(fh, fieldnames=list(rows[0]))
                    w.writeheader()
                    w.writerows(rows)
                print(f"wrote {path}")


if __name__ == "__main__":
    main()
