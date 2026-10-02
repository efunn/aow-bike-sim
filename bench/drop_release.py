"""Cycle the drop rig's XL330: lift on the snail cam, hold, drop, settle.

Runs beside force_drop.py. One cam segment per drop, current-based position
mode (multi-turn): --lift-ma up the ramp, the Current Limit for the drop.
Start: torque off, the cam just past its tallest step (arm resting on the sensor).
Hold: the start of the top flat (--margin-deg), a run-up; each drop prints
the edge speed, and warns under sqrt(g * --corner-mm) (~280 deg/s).
Exit: back down to the last park if still on the ramp, else on through the
step; then torque off.

    python bench/drop_release.py --dry-run
    python bench/drop_release.py --port /dev/cu.usbserial-XXXX --drops 12
    python bench/drop_release.py --port $P --direction 1   # cam mounted mirrored
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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))     # this checkout's source

from aow_sim.hw.dynamixel import (MODE_CURRENT_POSITION,  # noqa: E402
                                  MODE_EXTENDED_POSITION, DynamixelBus)

TICKS_PER_REV = 4096
RPM_PER_LSB = 0.229       # Profile Velocity unit on X-series
POS_TOL = 20              # ticks (~1.8 deg): "arrived"


def signed32(raw: int) -> int:
    return raw - (1 << 32) if raw & (1 << 31) else raw


class Cam:
    def __init__(self, bus: DynamixelBus, dxl_id: int, direction: int):
        self.bus, self.id, self.dir = bus, dxl_id, direction

    def pos(self) -> int:
        return signed32(self.bus.read_raw(self.id, "Present Position"))

    def go(self, target: int, rpm: float, timeout_s: float, trace=None, ma=None) -> None:
        """Move and wait. `ma`: Goal Current (current-based mode only).
        `trace`: a list to fill with (t_s, ticks), read flat out."""
        if ma is not None:
            self.bus.write_raw(self.id, "Goal Current", int(ma))
        self.bus.write_raw(self.id, "Profile Velocity", int(rpm / RPM_PER_LSB))  # 0 = max
        self.bus.write_raw(self.id, "Goal Position", target & 0xFFFFFFFF)
        t0 = time.time()
        while True:
            p = self.pos()
            if trace is not None:
                trace.append((time.time() - t0, p))
            if abs(p - target) <= POS_TOL:
                return
            if time.time() - t0 > timeout_s:
                err = self.bus.read_raw(self.id, "Hardware Error Status")
                raise RuntimeError(f"cam stuck at {p} -> {target} "
                                   f"(hardware error {err:#04x}): jammed or overloaded")
            if trace is None:
                time.sleep(0.01)


def edge_speed(trace, at_tick, deg) -> float:
    """deg/s where the trace crosses `at_tick` (the step), from the samples
    either side of it; nan if the trace never crossed it cleanly."""
    for (t0, p0), (t1, p1) in zip(trace, trace[1:]):
        if (p0 - at_tick) * (p1 - at_tick) <= 0 and t1 > t0 and p1 != p0:
            return abs(p1 - p0) / deg / (t1 - t0)
    return float("nan")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default=os.environ.get("AOW_DXL_PORT"))
    ap.add_argument("--baud", type=int, default=3_000_000)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--direction", type=int, choices=(-1, 1), default=-1,
                    help="-1: clockwise looking at the horn (the cam as drawn)")
    ap.add_argument("--drops", type=int, default=12, help="how many to do")
    ap.add_argument("--cam-drops", default="0.5,1,1.5,2",
                    help="the cam's steps in mm, rotation order (as given to drop_cam.py)")
    ap.add_argument("--first", type=int, default=1, help="which of them comes next (1-based)")
    ap.add_argument("--dwell-deg", type=float, default=20.0, help="the cam's, per segment")
    ap.add_argument("--margin-deg", type=float, default=30.0,
                    help="hold this far back from the step: the cam's top flat, the run-up")
    ap.add_argument("--park-deg", type=float, default=12.0,
                    help="how far past a step the start (and every settle) sits; also "
                         "keeps the step clear of where the servo brakes")
    ap.add_argument("--corner-mm", type=float, default=0.45,
                    help="cam + follower corner radii, summed (print rounding)")
    ap.add_argument("--r-top-mm", type=float, default=13.5, help="the cam's smallest top radius")
    ap.add_argument("--lift-rpm", type=float, default=15.0)
    ap.add_argument("--mode", choices=("current", "extended"), default="current",
                    help="current-based position (current-limited) or extended position")
    ap.add_argument("--lift-ma", type=float, default=300.0,
                    help="Goal Current up the ramp, mA (the drop uses the Current Limit)")
    ap.add_argument("--hold-s", type=float, default=1.0)
    ap.add_argument("--settle-s", type=float, default=2.5)
    ap.add_argument("--dry-run", action="store_true", help="print the plan, touch nothing")
    args = ap.parse_args()

    deg = TICKS_PER_REV / 360.0
    cam_drops = [float(x) for x in args.cam_drops.split(",")]
    seg_deg = 360.0 / len(cam_drops)
    if not 0 < args.park_deg < args.dwell_deg:
        sys.exit(f"--park-deg must sit in the cam's dwell, 0-{args.dwell_deg:g} deg past a step")
    # from park (park_deg past a step), forward round the dwell and up the ramp
    # to margin_deg short of the next step
    lift = round((seg_deg - args.park_deg - args.margin_deg) * deg)
    turn = round(seg_deg * deg)
    order = [cam_drops[(args.first - 1 + i) % len(cam_drops)] for i in range(args.drops)]
    need = math.degrees(math.sqrt(9.81 * args.corner_mm * 1e-3) / (args.r_top_mm * 1e-3))
    print(f"per drop: lift {lift / deg:.0f} deg at {args.lift_rpm:g} rpm, hold {args.hold_s:g} s, "
          f"drop {(turn - lift) / deg:.0f} deg at full speed, settle {args.settle_s:g} s; "
          f"direction {args.direction:+d}\n"
          f"{args.drops} drops, nominal mm: {','.join(f'{h:g}' for h in order)}"
          f"  (for force_drop.py: --heights {args.cam_drops} --order cycle)")
    if args.dry_run:
        return
    if not args.port:
        sys.exit("no --port given and AOW_DXL_PORT is unset")

    log = ROOT / "bench" / "logs" / f"drop_release_{datetime.now():%Y%m%d-%H%M}.csv"
    rows = []
    with DynamixelBus(args.port, baud=args.baud, ids=(args.id,)) as bus:
        if bus.read_raw(args.id, "Torque Enable"):
            sys.exit("torque is already on: turn it off with the arm resting on the sensor "
                     "(cam at park), then start again")
        current = args.mode == "current"
        bus.write_raw(args.id, "Operating Mode",
                      MODE_CURRENT_POSITION if current else MODE_EXTENDED_POSITION)
        full_ma = bus.read_raw(args.id, "Current Limit") if current else None
        lift_ma = min(args.lift_ma, full_ma) if current else None
        cam = Cam(bus, args.id, args.direction)
        park = cam.pos()
        bus.write_raw(args.id, "Goal Position", park & 0xFFFFFFFF)
        bus.torque(True, ids=(args.id,))
        print(f"park at {park} ticks; torque on. Ctrl-C parks, then stops.")
        target_park = park
        try:
            for n in range(1, args.drops + 1):
                cam.go(target_park + args.direction * lift, args.lift_rpm,
                       timeout_s=60.0 / args.lift_rpm * seg_deg / 360 * 1.5 + 2, ma=lift_ma)
                time.sleep(args.hold_s)
                target_park += args.direction * turn
                t_drop, trace = time.time(), []
                cam.go(target_park, 0, timeout_s=3.0, trace=trace, ma=full_ma)
                v = edge_speed(trace, target_park - args.direction * round(args.park_deg * deg), deg)
                rows.append(dict(drop=n, nominal_mm=order[n - 1], t_unix=round(t_drop, 3),
                                 edge_deg_s=round(v, 1), samples=len(trace),
                                 temp_c=bus.read(args.id, "Present Temperature")))
                if v != v:
                    speed = f"edge speed not measured ({len(trace)} reads)"
                else:
                    speed = f"edge {v:.0f} deg/s ({len(trace)} reads)"
                    if v < need:
                        speed += f"   !! under ~{need:.0f} deg/s: a slow release"
                print(f"  drop {n}/{args.drops}: nominal {order[n - 1]:g} mm, {speed}")
                time.sleep(args.settle_s)
        except KeyboardInterrupt:
            print("\nstopping: parking first")
        finally:
            # mid-ramp: back down to the park it came from; top flat or mid-drop:
            # on to the next park (a normal drop). Then torque off.
            try:
                ahead = (target_park - cam.pos()) * args.direction
                if ahead > POS_TOL:                       # the drop was under way
                    cam.go(target_park, 0, timeout_s=10.0, ma=full_ma)
                elif ahead < -POS_TOL:                    # lifting or holding
                    if -ahead < lift - round(3 * deg):    # still on the ramp: back down
                        cam.go(target_park, args.lift_rpm, timeout_s=10.0, ma=lift_ma)
                    else:                                 # on the top flat: drop
                        target_park += args.direction * turn
                        cam.go(target_park, 0, timeout_s=10.0, ma=full_ma)
            except Exception as e:
                print(f"!! could not reach park ({e}); torque left on. Lift the arm off "
                      "the cam, then power the servo down.")
                raise
            bus.torque(False, ids=(args.id,))
            print("parked, torque off")
    if rows:
        with open(log, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"wrote {log}")


if __name__ == "__main__":
    main()
