"""Record real TM151 data over USB, into something the sim can be checked against.

Plug the AHRS into the laptop, run this, get an `.npz`. The protocol lives in
`tm151_serial.py`; this is the part that talks to the port and writes a file.

WHY BOTHER, i.e. what the recording is FOR. `src/aow_sim/sim_ahrs.py` models
this part from its datasheet, and the eval results that came out of it -- a
TM151 attitude error costing `general_rl_odo` survival 1.00 -> 0.90, and
collapsing the truth-trained policies outright -- rest on three numbers. Two
are quoted by the vendor (gyro noise, orientation RMS). THE THIRD, `TAU_ORIENT_S`,
IS A GUESS with no source at all. A few minutes of the unit sitting still on a
desk measures all three. `tm151_check.py` is the script that does the comparing;
this one just gets the samples.

  # is it there, and what is it saying?
  python analysis/tm151_record.py --list
  python analysis/tm151_record.py --monitor

  # the recording that matters: STATIONARY, on a solid surface, undisturbed
  python analysis/tm151_record.py --seconds 300 --tag rest

  # and one being moved by hand, for the dynamic figure
  python analysis/tm151_record.py --seconds 120 --tag handheld

Writes `analysis/recordings/tm151_<tag>.npz`. Gitignored like other loose
analysis output -- these are raw captures, not results.

FIVE MINUTES AT REST IS THE USEFUL LENGTH, and that is not arbitrary. The
orientation error is correlated with a time constant we are trying to measure;
if the guess (2 s) is anywhere near right, 300 s is 150 correlation times,
which is enough for the autocorrelation to be worth fitting. A 10 s capture
would report a confidently wrong number -- see the window-length table in
`sim_ahrs.PP_TO_SIGMA`, where the same effect makes peak-to-peak meaningless
without a stated window.

BAUD. The vendor recommends 460800 at 200 Hz ODR and 921600 or 1 Mbps at 400 Hz
(datasheet, Module Output). The device's own default is often 115200. If the
rate is wrong you get framing garbage rather than silence, so `--baud auto`
tries the plausible ones and keeps whichever actually decodes.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tm151_serial import (CMD, Decoder, find_ports, request)  # noqa: E402

BAUDS = (115200, 230400, 460800, 921600, 1000000)
# Fields worth keeping per packet kind. `combo` carries everything at once and
# is what the unit streams by default on recent firmware.
WANT = ("rpy_deg", "quat", "gyro", "acc", "mag", "temp_c", "rate_hz", "qos")


def _rec_dir() -> Path:
    d = Path(__file__).resolve().parent / "recordings"
    d.mkdir(parents=True, exist_ok=True)
    return d


def open_port(dev: str | None, baud, settle: float = 1.5):
    """Open the port and confirm packets actually decode. Returns (ser, baud).

    A wrong baud rate produces FRAMING GARBAGE, not silence, so "the port
    opened" proves nothing -- this waits for real CRC-passing packets before
    calling it good.
    """
    import serial

    if dev is None:
        found = find_ports()
        if not found:
            raise SystemExit(
                "no likely serial port found.\n"
                "  Plug the TM151 in, then: python analysis/tm151_record.py --list\n"
                "  (on macOS use /dev/cu.*, never /dev/tty.* -- tty blocks on open)")
        dev = found[0][0]
        print(f"  port      {dev}   ({found[0][1]})")

    tries = BAUDS if baud == "auto" else [int(baud)]
    for b in tries:
        ser = serial.Serial(dev, b, timeout=0.1)
        dec = Decoder()
        ser.reset_input_buffer()
        t0 = time.monotonic()
        while time.monotonic() - t0 < settle:
            dec.feed(ser.read(4096))
            if dec.n_ok >= 5:
                print(f"  baud      {b}   ({dec.n_ok} packets in "
                      f"{time.monotonic()-t0:.1f} s)")
                return ser, b
        ser.close()
        if baud != "auto":
            raise SystemExit(
                f"opened {dev} at {b} but decoded nothing in {settle:.0f} s.\n"
                f"  {dec.n_resync} bytes seen and discarded, "
                f"{dec.n_crc_bad} CRC failures.\n"
                "  If bytes ARE arriving, the baud is probably wrong: "
                "try --baud auto.\n"
                "  If none are, the unit may need a request to start "
                "streaming: try --request combo.")
    raise SystemExit(f"no baud rate in {tries} decoded anything on {dev}. "
                     "Is the unit powered and streaming?")


def record(ser, seconds: float, poll: str | None) -> dict:
    """Read for `seconds`. Returns `{kind}__{field}` column arrays.

    KEPT PER PACKET KIND, not merged into one table. The unit does not
    necessarily stream the all-in-one `combo` packet: a TM151 on stock firmware
    was observed streaming `rpy`, `raw_gyro_acc_mag` and `status` as three
    separate 50 Hz streams. Merging those into one row-set gives two packets
    sharing every timestamp (so `median(diff(t))` is 0) and columns that are
    arrays for some rows and absent for others.

    An earlier version did merge, and dropped every ragged column on the way to
    `np.savez` -- it wrote a file containing nothing but timestamps and said
    nothing about it, costing a five-minute capture. Hence per-kind storage and
    the explicit check in `main` that at least one payload column survived.
    """
    dec = Decoder()
    cols: dict[str, dict[str, list]] = {}
    kinds: dict[str, int] = {}
    t0 = time.monotonic()
    last_poll = last_print = 0.0
    last_rpy = None
    cmd_num = {v: k for k, v in CMD.items()}.get(poll) if poll else None

    while True:
        now = time.monotonic() - t0
        if now >= seconds:
            break
        if cmd_num is not None and now - last_poll > 0.05:
            ser.write(request(cmd_num))
            last_poll = now
        for p in dec.feed(ser.read(4096)):
            kinds[p.kind] = kinds.get(p.kind, 0) + 1
            c = cols.setdefault(p.kind, {})
            c.setdefault("t_us", []).append(p.t_us)
            c.setdefault("t_host", []).append(now)
            for k, v in p.fields.items():
                if k == "t_us":
                    continue
                c.setdefault(k, []).append(v)
            if "rpy_deg" in p.fields:
                last_rpy = p.fields["rpy_deg"]
        if now - last_print > 0.5:
            last_print = now
            n = sum(kinds.values())
            # COUNT THE PAYLOAD, NOT JUST THE PACKETS. The failure this
            # replaced looked healthy for five minutes and saved a file with
            # nothing in it but timestamps, because the packet counter cannot
            # tell whether anything is being STORED. This can: it is the number
            # of non-time columns actually accumulating.
            ncol = sum(1 for f in cols.values() for k in f
                       if not k.startswith("t_"))
            msg = (f"\r  {now:6.1f}s  {n:7d} pkt  "
                   f"{n/now if now else 0:6.1f} Hz  "
                   f"[{'+'.join(sorted(kinds))}] {ncol} cols")
            if last_rpy is not None:
                msg += (f"  rpy {last_rpy[0]:+7.2f} {last_rpy[1]:+7.2f} "
                        f"{last_rpy[2]:7.2f}")
            if dec.n_crc_bad:
                msg += f"  crc_bad {dec.n_crc_bad}"
            print(msg, end="", flush=True)
    print()

    out: dict[str, np.ndarray] = {}
    for kind, fields in cols.items():
        for name, vals in fields.items():
            try:
                a = np.array(vals, dtype=float)
            except ValueError:
                continue                       # genuinely ragged: skip, noisily
            if a.size:
                out[f"{kind}__{name}"] = a
    out["_kinds"] = np.array(sorted(kinds.items()), dtype=object)
    out["_crc_bad"] = np.array(dec.n_crc_bad)
    out["_resync_bytes"] = np.array(dec.n_resync)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="list likely ports and exit")
    ap.add_argument("--monitor", action="store_true",
                    help="print live values, record nothing")
    ap.add_argument("--port", default=None, help="e.g. /dev/cu.usbmodem1101")
    ap.add_argument("--baud", default="auto",
                    help=f"one of {BAUDS}, or 'auto' (default)")
    ap.add_argument("--seconds", type=float, default=300.0,
                    help="capture length; 300 s at rest is the useful one")
    ap.add_argument("--tag", default="rest",
                    help="names the output file, e.g. rest / handheld / mast")
    ap.add_argument("--request", default=None, choices=sorted(CMD.values()),
                    help="poll for this packet type instead of listening "
                         "passively (only if the unit does not stream)")
    args = ap.parse_args()

    if args.list:
        found = find_ports()
        if not found:
            print("  nothing likely. Plug it in, then try: ls /dev/cu.*")
            return 1
        for dev, desc in found:
            print(f"  {dev:32} {desc}")
        return 0

    ser, baud = open_port(args.port, args.baud)
    try:
        if args.monitor:
            print("  monitoring, ctrl-c to stop\n")
            record(ser, 1e9, args.request)
            return 0
        print(f"  recording {args.seconds:.0f} s, tag '{args.tag}'")
        print("  KEEP IT STILL if this is a 'rest' capture.\n")
        data = record(ser, args.seconds, args.request)
    except KeyboardInterrupt:
        print("\n  interrupted")
        return 1
    finally:
        ser.close()

    payload = [k for k in data
               if "__" in k and not k.split("__")[1].startswith("t_")]
    if not payload:
        print("  NOTHING USEFUL RECORDED -- no payload columns survived.")
        print(f"  packet kinds seen: {dict(data['_kinds'])}")
        print("  This is a bug in the decoder or an unhandled packet type;"
              " the file is NOT written.")
        return 1

    out = _rec_dir() / f"tm151_{args.tag}.npz"
    data["_baud"] = np.array(baud)
    np.savez_compressed(out, **data)

    print()
    for kind, c in data["_kinds"]:
        key = f"{kind}__t_us"
        if key not in data:
            print(f"  {kind:20} {c:7d} packets   (no timestamped fields)")
            continue
        t = data[key] * 1e-6
        dt = np.diff(t)
        dt = dt[dt > 0]                # same-tick duplicates are not a period
        rate = 1 / np.median(dt) if len(dt) else float("nan")
        have = sorted(k.split("__")[1] for k in data
                      if k.startswith(f"{kind}__") and not k.endswith("t_host"))
        print(f"  {kind:20} {c:7d} packets  {rate:6.1f} Hz  "
              f"{t[-1]-t[0]:6.1f} s   {', '.join(have)}")
    if int(data["_crc_bad"]):
        print(f"  CRC failures {int(data['_crc_bad'])} "
              f"({int(data['_resync_bytes'])} bytes discarded) -- more than a "
              f"handful means the baud or cable is marginal")

    print(f"\n  wrote {out}")
    print(f"  next: python analysis/tm151_check.py --tag {args.tag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
