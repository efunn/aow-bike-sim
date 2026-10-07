"""Live readout / CSV log of the four-channel force sensor (force_sensor/force_sensor.ino).

Stream: `micros,x,y,z,rz` raw 13-bit counts per line over USB serial
Each channel is tared over the first --tare-s
Sensors: TE FS2050, 1500 gf (14.71 N) (sheet/tefs20.pdf)
Scale: calibrated 2026-10-02 with 28.5-455 g, +-3% (logs/force_cal_20261002-0044.csv)
Limits: reading clips ~19 N, safe load ~37 N (8 pounds); zero drifts up to 0.07 N in 3 min

    python bench/force_sensor.py
    python bench/force_sensor.py --log bench/logs/x.csv --seconds 5
"""
from __future__ import annotations

import argparse
import sys

import serial

CHANNELS = ("X", "Y", "Z", "Rz")
ADC_MAX = 8192
ADC_VREF = 3.3
FULL_SCALE_N = 14.709975
SPAN_V = 3.0 * 3.3 / 5.0            # FS2050 span at a 3.3 V supply (nominal)
NOMINAL_N_PER_COUNT = FULL_SCALE_N / SPAN_V * ADC_VREF / ADC_MAX
CALIBRATED_N_PER_COUNT = [0.003043, 0.003040, 0.002797, 0.003105]   # 2026-10-02


TEENSY_VID = 0x16C0       # PJRC's USB vendor ID, every Teensy's; not yet read off this unit
BAD_LINES = 200           # in a row, not `micros,x,y,z,rz`: the wrong device


def find_port() -> str:
    """The one serial port whose USB vendor is PJRC. Not the first
    usbmodem / ttyACM by name: the TM151 is one too (STM32, 0483), and
    which sorts first depends on the USB socket (macOS) or plug order (Linux)."""
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    found = [p for p in ports if p.vid == TEENSY_VID]
    if len(found) == 1:
        return found[0].device
    usb = "\n".join(f"  {p.device}  {p.vid:04x}:{p.pid:04x}  {p.manufacturer or ''} "
                    f"{p.product or ''}" for p in ports if p.vid is not None) or "  (none)"
    sys.exit(f"force sensor: {len(found)} ports with USB vendor {TEENSY_VID:04x} (Teensy); "
             f"pass --port. USB serial ports:\n{usb}")


def lines(port: str):
    """Yield (t_s, counts[4]) from the stream, skipping partial lines."""
    with serial.Serial(port, 115200, timeout=1.0) as s:
        s.reset_input_buffer()
        s.readline()                       # drop the partial first line
        t0, bad = None, 0
        while True:
            raw = s.readline()
            if not raw:
                sys.exit("no data from the sensor in 1 s (old firmware? it only sends HID)")
            parts = raw.decode(errors="replace").strip().split(",")
            try:
                if len(parts) != 5:
                    raise ValueError
                us, *c = (int(p) for p in parts)
            except ValueError:
                bad += 1
                if bad >= BAD_LINES:
                    sys.exit(f"{port}: {bad} lines in a row that are not micros,x,y,z,rz: "
                             f"not the force sensor (the TM151 sends binary frames)")
                continue
            bad = 0
            t0 = us if t0 is None else t0
            yield ((us - t0) & 0xFFFFFFFF) * 1e-6, c   # micros() wraps


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default=None)
    ap.add_argument("--tare-s", type=float, default=0.5)
    ap.add_argument("--scale", default=None, help="N per count, four comma-separated")
    ap.add_argument("--log", default=None, help="write t_s, counts and newtons to this CSV")
    ap.add_argument("--seconds", type=float, default=None, help="stop after this long")
    args = ap.parse_args()
    scale = [float(x) for x in args.scale.split(",")] if args.scale else None
    stream = lines(args.port or find_port())

    print(f"taring {args.tare_s} s, keep the sensors unloaded...", flush=True)
    acc, n = [0.0] * 4, 0
    for t, c in stream:
        acc = [a + x for a, x in zip(acc, c)]
        n += 1
        if t >= args.tare_s:
            break
    zero = [a / n for a in acc]
    if scale is None:
        scale = list(CALIBRATED_N_PER_COUNT)
    print("zero [counts]: " + "  ".join(f"{ch} {z:.1f}" for ch, z in zip(CHANNELS, zero))
          + f"   ({n / args.tare_s:.0f} sweeps/s)")
    print("scale [N/count]: " + "  ".join(f"{ch} {k:.5f}" for ch, k in zip(CHANNELS, scale))
          + ("  (--scale)" if args.scale else "  (calibrated 2026-10-02)"))

    log = open(args.log, "w") if args.log else None
    if log:
        log.write("t_s," + ",".join(f"{ch}_counts" for ch in CHANNELS) + ","
                  + ",".join(f"{ch}_N" for ch in CHANNELS) + "\n")
    t_start, shown, peak = None, 0.0, [0.0] * 4
    try:
        for t, c in stream:
            t_start = t if t_start is None else t_start
            f = [(x - z) * k for x, z, k in zip(c, zero, scale)]
            peak = [max(p, v) for p, v in zip(peak, f)]
            if log:
                log.write(f"{t - t_start:.6f}," + ",".join(map(str, c)) + ","
                          + ",".join(f"{v:.4f}" for v in f) + "\n")
            if t - shown >= 0.05:
                shown = t
                sys.stdout.write("\r" + "  ".join(f"{ch} {v:+7.3f} N (pk {p:6.3f})"
                                                  for ch, v, p in zip(CHANNELS, f, peak)))
                sys.stdout.flush()
            if args.seconds is not None and t - t_start >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if log:
            log.close()
            print(f"wrote {args.log}")


if __name__ == "__main__":
    main()
