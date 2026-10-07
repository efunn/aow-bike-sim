"""The drop rig's wave cams: the XL330 turns the cam at set speeds while the TM151 logs.

For the AHRS's response to SUSTAINED acceleration, the case neither rig has
measured (docs/plans/drop-release-rig.md, "AHRS mode"). Per speed: a still
hold, then --spin-s at that speed; a last hold at the end. The holds are the
truth reference (the AHRS settles); the spins shake the arm at lobes x rev/s.

On this arm every point shares one angular acceleration alpha, so at the
AHRS the HORIZONTAL acceleration is alpha x its height above the pivot (it
tilts the gravity reading and barely moves |acc|: the untested case) and
the VERTICAL one alpha x its distance along the bar (it moves |acc|, which
the filter now weights down). So mount it high near the pivot for the test,
and low along the bar for the control.

The servo: VELOCITY mode (Profile Velocity does not hold a speed in
current-based position, user 2026-10-07). Refused with a latched hardware
error; torque on, THEN Goal Velocity (a goal written with torque off is
acknowledged and dropped: hw/control_tables/README.md). Turned FORWARD only
(cam.dir), ramped in software over --ramp-s, holds at Goal Velocity 0.
A speed check stops it, torque off, if the cam makes under half the set
speed over STALL_WINDOW_S (Current Limit does nothing in this mode, so
nothing else would). The velocity PI gains are the servo's own (180 / 1600
by default; nothing here writes them), read into the run record. Each
spin's speed is measured from the encoder (a line through its cruise) and
refused at 3% off. force_drop.py writes its own mode every run, so
leaving this one is harmless.

Speeds are checked against the cam (--tones, bench/drop_cam.py): refused
past 90% of the speed where the follower leaves it, and past the servo's
Velocity Limit. Files: the big ones in bench/logs/archive/ (gitignored,
Dropbox-synced), the record in bench/logs/ (tracked):
  archive/wave_<time>_ahrs.csv   every TM151 frame (bench/ahrs_stream.py)
  archive/wave_<time>_servo.csv  host_s, ticks, goal_rev_s: every read
  wave_<time>_segments.csv       hold / spin, set and measured rev/s, start/end
  wave_<time>_run.json           tones, station, speeds, ports, the cam's report

    python bench/wave_run.py --tones 4:2.4 --speeds 0.25,0.5,1,1.5 \\
        --ahrs-at-mm 25,80
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ahrs_stream  # noqa: E402
import drop_cam  # noqa: E402
import force_drop as fd  # noqa: E402

from aow_sim.hw.dynamixel import MODE_VELOCITY  # noqa: E402

RPM_PER_UNIT = 0.229            # Goal Velocity / Velocity Limit (0.02397 rad/s, the table)
SPEED_TOL = 0.03                # measured against set, fraction
SEP_MARGIN = 0.9                # of the follower's separation speed
HOLD_READ_S = 0.05              # servo reads while holding
RAMP_STEP_S = 0.02              # software ramp: one Goal Velocity write per
STALL_WINDOW_S = 0.5            # progress checked over this ...
STALL_FRACTION = 0.5            # ... and refused under this much of the set speed
LOGS = Path(__file__).resolve().parent / "logs"
ARCHIVE = LOGS / "archive"


def stem_pair(name: str) -> tuple[Path, Path]:
    """(record stem in logs/, big-file stem in archive/), one free name in both."""
    n, cand = 1, name
    while any(LOGS.glob(cand + "_*")) or any(ARCHIVE.glob(cand + "_*")):
        n += 1
        cand = f"{name}-{n}"
    return LOGS / cand, ARCHIVE / cand


def plan(speeds, spin_s, hold_s, ramp_s):
    """[(kind, rev_s, seconds)]: hold, spin, hold, spin, ..., hold."""
    out = [("hold", 0.0, hold_s)]
    for v in speeds:
        out += [("spin", v, spin_s), ("hold", 0.0, hold_s)]
    return out


def check_speeds(speeds, tones, fall_ms2, limit_rev_s) -> float:
    """Refuses a speed the cam or the servo cannot do; returns the separation speed."""
    sep = drop_cam.wave_separation_rev_s(tones, fall_ms2)
    bad = [v for v in speeds if v <= 0 or v > SEP_MARGIN * sep or v > limit_rev_s]
    if bad:
        sys.exit(f"speeds {bad} rev/s: each must be > 0, <= {SEP_MARGIN:g} x the follower's "
                 f"separation ({sep:.2f} rev/s for {tones}, falling at {fall_ms2:g} m/s^2) and "
                 f"<= the servo's Velocity Limit ({limit_rev_s:.2f} rev/s)")
    return sep


def cruise_rev_s(reads, t0, t1, ramp_s) -> float:
    """rev/s from a line through the reads between the ramps."""
    pts = [(t, p) for t, p, _ in reads if t0 + ramp_s * 1.5 <= t <= t1 - ramp_s * 1.5]
    if len(pts) < 10:
        return float("nan")
    t, p = np.array(pts).T
    return abs(np.polyfit(t - t.mean(), p, 1)[0]) / fd.TICKS


class WaveServo:
    """The cam's XL330 in velocity mode: forward only, software-ramped, watched."""

    def __init__(self, bus, dxl_id, direction, log):
        self.bus, self.id, self.dir, self.log = bus, dxl_id, direction, log
        latched = bus.hardware_errors(ids=(dxl_id,))
        if latched:
            raise RuntimeError(f"id {dxl_id} has a latched hardware error "
                               f"({fd.describe_hardware_error(latched[dxl_id])}): power-cycle "
                               f"once the cause is clear")
        bus.prepare(ids=(dxl_id,))          # torque off, return delay 0, profiles 0
        bus.write_raw(dxl_id, "Operating Mode", MODE_VELOCITY)
        if bus.read_raw(dxl_id, "Operating Mode") != MODE_VELOCITY:
            raise RuntimeError(f"id {dxl_id}: Operating Mode did not take velocity mode")
        self.limit_rev_s = bus.read_raw(dxl_id, "Velocity Limit") * RPM_PER_UNIT / 60
        self.info = {k: bus.read_raw(dxl_id, k) for k in
                     ("Velocity Limit", "Velocity P Gain", "Velocity I Gain")}
        self.set_rev_s = 0.0
        self.on = False

    def _bus(self, fn, *a):
        for attempt in range(fd.BUS_TRIES):
            try:
                return fn(self.id, *a)
            except RuntimeError as e:
                if attempt == fd.BUS_TRIES - 1:
                    raise
                print(f"  !! servo bus: {e}; retrying")
                time.sleep(0.01)

    def read(self) -> int:
        ta = time.perf_counter()
        p = fd.signed(self._bus(self.bus.read_raw, "Present Position"), 4)
        self.log.append(((ta + time.perf_counter()) / 2, p, self.set_rev_s))
        return p

    def speed(self, rev_s) -> None:
        """Goal Velocity, forward only. Torque must be on (README: dropped otherwise)."""
        assert self.on and rev_s >= 0
        self.set_rev_s = rev_s
        self._bus(self.bus.write_raw, "Goal Velocity",
                  (self.dir * round(rev_s * 60 / RPM_PER_UNIT)) & 0xFFFFFFFF)

    def start(self) -> None:
        """Torque on, THEN Goal Velocity 0: standing still from the start."""
        self._bus(self.bus.write_raw, "Torque Enable", 1)
        self.on = True
        self.speed(0.0)
        self.read()

    def hold(self, seconds) -> None:
        """Goal Velocity 0, read every HOLD_READ_S."""
        self.speed(0.0)
        t_end = time.perf_counter() + seconds
        while time.perf_counter() < t_end:
            time.sleep(HOLD_READ_S)
            self.read()

    def _ramp(self, a, b, ramp_s) -> None:
        n = max(1, round(ramp_s / RAMP_STEP_S))
        for k in range(1, n + 1):
            self.speed(a + (b - a) * k / n)
            t_next = time.perf_counter() + ramp_s / n
            while time.perf_counter() < t_next:
                self.read()

    def spin(self, rev_s, seconds, ramp_s) -> None:
        """Ramp to rev_s, cruise `seconds` reading flat out, ramp to 0. Stops,
        torque off, and raises if the cam makes under STALL_FRACTION of the
        set speed over a STALL_WINDOW_S."""
        self._ramp(0.0, rev_s, ramp_s)
        t_end = time.perf_counter() + seconds
        win = [(time.perf_counter(), self.read())]
        while time.perf_counter() < t_end:
            t, p = time.perf_counter(), self.read()
            win.append((t, p))
            while win and t - win[0][0] > STALL_WINDOW_S:
                t0, p0 = win.pop(0)
                got = self.dir * (p - p0) / fd.TICKS / (t - t0)
                if got < STALL_FRACTION * rev_s:
                    self.stop()
                    raise RuntimeError(f"cam stalled: {got:.2f} rev/s against {rev_s:g} set "
                                       f"over {t - t0:.2f} s; torque off")
        self._ramp(rev_s, 0.0, ramp_s)

    def stop(self) -> None:
        try:
            if self.on:
                self.speed(0.0)
        finally:
            self._bus(self.bus.write_raw, "Torque Enable", 0)
            self.on = False


def run(servo, steps, ramp_s) -> list:
    """Every step of the plan; one row per step, filled as it ends."""
    segs = []
    for i, (kind, v, secs) in enumerate(steps):
        t0 = time.perf_counter()
        n0 = len(servo.log)
        print(f"{i + 1}/{len(steps)}: " + (f"hold {secs:g} s" if kind == "hold"
                                           else f"spin {v:g} rev/s for {secs:g} s"), flush=True)
        if kind == "hold":
            servo.hold(secs)
        else:
            servo.spin(v, secs, ramp_s)
        t1 = time.perf_counter()
        row = dict(step=i + 1, kind=kind, rev_s_set=v, t_start_host_s=round(t0, 6),
                   t_end_host_s=round(t1, 6), rev_s_measured="")
        if kind == "spin":
            got = cruise_rev_s(servo.log[n0:], t0, t1, ramp_s)
            row["rev_s_measured"] = round(got, 4)
            print(f"  measured {got:.3f} rev/s over the cruise")
            if not abs(got - v) <= SPEED_TOL * v:
                segs.append(row)
                raise RuntimeError(f"spun at {got:.3f} rev/s against {v:g} set: the velocity "
                                   f"gains, or a slower speed")
        segs.append(row)
    return segs


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--tones", required=True,
                    help='the cam on the horn, as bench/drop_cam.py: "4:2.4", "8:0.6", ...')
    ap.add_argument("--speeds", default="0.25,0.5,1,1.5", help="rev/s, in order")
    ap.add_argument("--spin-s", type=float, default=15.0,
                    help="cruise per speed (tau ~1 s moving: >= 10)")
    ap.add_argument("--hold-s", type=float, default=5.0, help="still, before and after each spin")
    ap.add_argument("--ramp-s", type=float, default=0.5, help="to speed and back")
    ap.add_argument("--fall-ms2", type=float, default=5.0,
                    help="the follower's fall (the contact's 8.30 m/s^2 x 124/205)")
    ap.add_argument("--ahrs-at-mm", default=None, metavar="X,Z[,Y]",
                    help="the chip: along the bar from the pivot, above the pivot's axis, "
                         "outboard (default 0), mm")
    ap.add_argument("--ahrs-port", default=None)
    ap.add_argument("--no-ahrs", action="store_true", help="the servo alone (a shakedown)")
    ap.add_argument("--note", default="", help="free text for the run record (the mount, ...)")
    cam = ap.add_argument_group("the servo, default config/drop_rig_bench.yaml")
    cam.add_argument("--cam-port", default=None)
    cam.add_argument("--cam-id", type=int, default=None)
    cam.add_argument("--cam-dir", type=int, choices=(-1, 1), default=None)
    args = ap.parse_args()

    tones = drop_cam.parse_tones(args.tones)
    speeds = [float(v) for v in args.speeds.split(",")]
    station = None
    if not args.no_ahrs:
        try:
            station = [float(v) for v in (args.ahrs_at_mm or "").split(",")]
            assert len(station) in (2, 3)
            station += [0.0] * (3 - len(station))
        except (ValueError, AssertionError):
            sys.exit("--ahrs-at-mm X,Z[,Y] (or --no-ahrs): the chip along the bar from the "
                     "pivot, above the pivot's axis, outboard, mm")
    cc = fd.cam_config(SimpleNamespace(cam_port=args.cam_port, cam_id=args.cam_id,
                                       cam_index=0, cam_dir=args.cam_dir, cam_ma=None))
    if cc["dir"] is None:
        sys.exit("cam.dir is not set in config/drop_rig_bench.yaml (or pass --cam-dir)")
    steps = plan(speeds, args.spin_s, args.hold_s, args.ramp_s)
    ARCHIVE.mkdir(parents=True, exist_ok=True)
    stem, big = stem_pair(f"wave_{datetime.now():%Y%m%d-%H%M}")

    bus = fd.open_cam_bus(cc["port"], cc["id"])
    servo = ahrs = None
    log, segs, ports = [], [], {"cam": cc["port"]}
    try:
        servo = WaveServo(bus, cc["id"], cc["dir"], log)
        sep = check_speeds(speeds, tones, args.fall_ms2, servo.limit_rev_s)
        print(f"cam {args.tones}: follower leaves above {sep:.2f} rev/s; servo {servo.info}; "
              f"{len(steps)} steps, ~{sum(s for _, _, s in steps) / 60:.1f} min")
        if not args.no_ahrs:
            ports["ahrs"] = args.ahrs_port or ahrs_stream.find_port()
            ahrs = ahrs_stream.AhrsStream(ports["ahrs"])
            print(f"ahrs: {ports['ahrs']}, Combo at {ahrs.wait_ready():.0f} Hz, at "
                  f"{station[0]:g} mm along, {station[1]:g} up, {station[2]:g} out")
        servo.start()
        segs = run(servo, steps, args.ramp_s)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        if servo is not None:
            servo.stop()
            print("torque off")
        bus.close()
        save(args, stem, big, tones, speeds, station, ports, ahrs, log, segs,
             servo.info if servo else {})


def save(args, stem, big, tones, speeds, station, ports, ahrs, log, segs, servo_info) -> None:
    if not log:
        return
    files = {}
    if ahrs is not None:
        ahrs.close()
        n = ahrs.write(f"{big}_ahrs.csv")
        files["ahrs"] = dict(file=f"archive/{big.name}_ahrs.csv", frames=n, crc_bad=ahrs.crc_bad)
        print(f"wrote {big}_ahrs.csv ({n} frames, {ahrs.crc_bad} CRC failures)")
    with open(f"{big}_servo.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(("host_s", "ticks", "goal_rev_s"))
        w.writerows((f"{t:.6f}", p, g) for t, p, g in log)
    files["servo"] = dict(file=f"archive/{big.name}_servo.csv", reads=len(log))
    print(f"wrote {big}_servo.csv ({len(log)} reads)")
    if segs:
        with open(f"{stem}_segments.csv", "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(segs[0]))
            w.writeheader()
            w.writerows(segs)
        files["segments"] = dict(file=f"{stem.name}_segments.csv", steps=len(segs))
        print(f"wrote {stem}_segments.csv ({len(segs)} steps)")
    report, ok = drop_cam.wave_report(0.0, 0.45, 0.7, tones, args.fall_ms2, 618.0, 124.0, 205.0)
    meta = dict(command=" ".join(sys.argv), written=datetime.now().isoformat(timespec="seconds"),
                tones=args.tones, speeds_rev_s=speeds, spin_s=args.spin_s, hold_s=args.hold_s,
                ramp_s=args.ramp_s, fall_ms2=args.fall_ms2, ports=ports, files=files,
                note=args.note, cam_report=report, servo=servo_info,
                clock="host_s: time.perf_counter(), shared with the AHRS log")
    if station is not None:
        meta["ahrs_station_mm"] = dict(along_bar_from_pivot=station[0], above_pivot=station[1],
                                       outboard_of_bar=station[2])
    with open(f"{stem}_run.json", "w") as fh:
        json.dump(meta, fh, indent=2, default=str)
    print(f"wrote {stem}_run.json")


if __name__ == "__main__":
    main()
