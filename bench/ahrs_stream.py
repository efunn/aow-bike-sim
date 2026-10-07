"""The TM151 over USB for a bench run: every frame kept, stamped on the host clock.

force_drop.py --ahrs starts one of these beside the force sensor's Stream.
A child process reads the port (like force_calibrate._read, and for the same
reason: in a thread it would share the GIL with the servo's reads) and
decodes with analysis/tm151_serial.Decoder, so every frame survives, not the
newest only as in hw/ahrs.AhrsReader, which serves a control loop.

TWO CLOCKS PER ROW. host_s is time.perf_counter() when the read that carried
the frame returned -- one clock across processes (Linux CLOCK_MONOTONIC,
macOS mach time), shared with the force stream's arrivals and the servo log,
but late by the USB and read delay, and every frame in one read gets the same
stamp. t_us is the TM151's own sample clock: evenly spaced, but its own
crystal. To put a frame on the host clock, fit host_s against t_us along
their lower envelope (the least-delayed frames, as Stream.sensor_time does)
-- with a slope, over a whole run: a crystal 50 ppm off drifts 30 ms in ten
minutes.

PORT: the one serial port whose USB vendor is STMicroelectronics (the TM151
enumerates as "STM32 Virtual ComPort", 0483), then checked by content: no
decoded Combo frames in 2 s and it refuses, naming the port.

    python bench/ahrs_stream.py              # find it, print 2 s of frames
"""
from __future__ import annotations

import csv
import multiprocessing
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
from tm151_serial import Decoder  # noqa: E402

STM32_VID = 0x0483        # STMicroelectronics: the TM151's USB vendor (bike_params' by-id name)
BAUD = 460800             # hw/ahrs.AhrsReader's; a USB virtual port ignores it
READY_FRAMES = 5

# Decoder field -> CSV columns. Combo carries all of them; another kind
# leaves the ones it lacks blank.
FIELDS = (("rpy_deg", ("roll_deg", "pitch_deg", "yaw_deg")),
          ("quat", ("qw", "qx", "qy", "qz")),
          ("gyro", ("gx_rad_s", "gy_rad_s", "gz_rad_s")),
          ("acc", ("ax_g", "ay_g", "az_g")),          # the vendor's g, 9.794 m/s^2
          ("mag", ("mx", "my", "mz")),
          ("temp_c", ("temp_c",)))
COLUMNS = ("host_s", "t_us", "kind", "qos") + tuple(c for _, cs in FIELDS for c in cs)


def find_port() -> str:
    """The one STMicroelectronics serial port; exits listing every USB port otherwise."""
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    found = [p for p in ports if p.vid == STM32_VID]
    if len(found) == 1:
        return found[0].device
    usb = "\n".join(f"  {p.device}  {p.vid:04x}:{p.pid:04x}  {p.manufacturer or ''} "
                    f"{p.product or ''}" for p in ports if p.vid is not None) or "  (none)"
    sys.exit(f"TM151: {len(found)} ports with USB vendor {STM32_VID:04x}; pass --ahrs-port. "
             f"USB serial ports:\n{usb}")


def _row(host: float, p) -> list:
    r = [host, p.t_us, p.kind, p.qos]
    for key, names in FIELDS:
        v = p.fields.get(key)
        if v is None:
            r += [""] * len(names)
        else:
            r += [float(x) for x in (v if len(names) > 1 else [v])]
    return r


def _read(port: str, baud: int, conn) -> None:
    """The child process: read what is there, decode, send rows in ~20 ms
    batches, with the decoder's counters (sent every 0.5 s even with no
    frames, so a port of the wrong device still reports what it saw)."""
    import serial
    dec, batch = Decoder(), []
    try:
        with serial.Serial(port, baud, timeout=0.05) as s:
            s.reset_input_buffer()
            sent = time.perf_counter()
            while True:
                n = s.in_waiting
                data = s.read(n if n else 1)
                host = time.perf_counter()
                batch.extend(_row(host, p) for p in dec.feed(data))
                if (batch and (len(batch) >= 20 or host - sent > 0.02)) or host - sent > 0.5:
                    conn.send((batch, dec.n_crc_bad, dec.n_resync))
                    batch, sent = [], host
    except BaseException as e:
        conn.send((batch, dec.n_crc_bad, dec.n_resync))     # what was read before it
        conn.send(repr(e))


class AhrsStream:
    """Every frame of the run, in `rows` (COLUMNS order)."""

    def __init__(self, port: str, baud: int = BAUD):
        self.port, self.rows, self.lock = port, [], threading.Lock()
        self.crc_bad = self.resync_bytes = 0
        self.error = None
        ours, theirs = multiprocessing.Pipe(duplex=False)
        self.proc = multiprocessing.Process(target=_read, args=(port, baud, theirs), daemon=True)
        self.proc.start()
        threading.Thread(target=self._run, args=(ours,), daemon=True).start()

    def _run(self, conn):
        try:
            while True:
                msg = conn.recv()
                if isinstance(msg, str):
                    self.error = msg
                    return
                batch, self.crc_bad, self.resync_bytes = msg
                with self.lock:
                    self.rows.extend(batch)
        except EOFError:
            self.error = self.error or "the AHRS reader process ended"

    def combo(self) -> int:
        with self.lock:
            return sum(1 for r in self.rows if r[2] == "combo")

    def wait_ready(self, timeout: float = 2.0) -> float:
        """Frames per second once READY_FRAMES Combo frames have decoded; exits
        with what the port did send otherwise (the content check)."""
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < timeout and self.error is None:
            if self.combo() >= READY_FRAMES:
                break
            time.sleep(0.02)
        if self.combo() < READY_FRAMES:
            with self.lock:
                kinds = sorted({r[2] for r in self.rows})
            sys.exit(f"TM151 on {self.port}: {self.combo()} Combo frames in {timeout:g} s "
                     f"(other kinds: {kinds or 'none'}; {self.resync_bytes} bytes discarded, "
                     f"{self.crc_bad} CRC failures; reader: {self.error or 'running'}). "
                     f"Not the TM151, or it is not streaming Combo (hw/ahrs.py, poll mode).")
        time.sleep(0.5)
        with self.lock:
            hs = [r[0] for r in self.rows if r[2] == "combo" and r[0] >= t0 + 0.2]
        return (len(hs) - 1) / (hs[-1] - hs[0]) if len(hs) > 2 else float("nan")

    def close(self) -> None:
        if self.proc.is_alive():
            self.proc.terminate()
            self.proc.join(timeout=1.0)
        time.sleep(0.05)                    # the last batch through the pipe

    def write(self, path) -> int:
        with self.lock:
            rows = list(self.rows)
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(COLUMNS)
            w.writerows([f"{r[0]:.6f}", *r[1:]] for r in rows)
        return len(rows)


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    a = AhrsStream(port)
    hz = a.wait_ready()
    time.sleep(1.5)
    a.close()
    last = dict(zip(COLUMNS, a.rows[-1]))
    print(f"{port}: {len(a.rows)} frames, Combo at {hz:.0f} Hz, {a.crc_bad} CRC failures; "
          f"last: roll {last['roll_deg']:+.2f} pitch {last['pitch_deg']:+.2f} deg, "
          f"acc {last['ax_g']:+.3f} {last['ay_g']:+.3f} {last['az_g']:+.3f} g")
