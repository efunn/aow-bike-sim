"""Live attitude view of a TM151 over USB. Move it, watch it track.

The eyeball test the recordings cannot give you: pick the unit up, roll it,
yaw it, tap it, and see whether the fusion filter follows, lags, or wanders.
Numbers say what the error IS; this says what it FEELS like, which is the thing
that tells you whether a mount is sane before committing to it.

  python analysis/tm151_view.py                 # window: horizon + compass
  python analysis/tm151_view.py --ascii         # terminal only, no GUI
  python analysis/tm151_view.py --seconds 60    # then quit on its own

FOUR PANELS.
  * ARTIFICIAL HORIZON -- roll and pitch, the way an aircraft instrument shows
    them: the horizon rotates against a fixed aircraft mark. Level is level.
  * COMPASS -- heading. This is the magnetometer's channel, and the one to
    watch when you move the unit near a motor or a current-carrying wire: a
    heading that swings while the unit is still is magnetic disturbance, and
    is exactly what mounting near the drive coils would do.
  * TRACES -- the last 20 s of roll/pitch/yaw, for spotting lag and overshoot
    that a static readout hides.
  * GYRO -- body rates, so a fast motion can be compared against the attitude
    response. Attitude visibly trailing the rate is the fusion filter's lag.

WHAT TO ACTUALLY TRY, in rough order of how much it tells you:
  1. Sit still. Roll and pitch should be near-motionless -- measured on a desk,
     0.014 deg RMS, some 35x better than the datasheet's <0.5 deg bound.
  2. Roll 90 deg and hold. It should arrive quickly and STAY, because gravity
     bounds roll and pitch absolutely.
  3. Yaw 90 deg and hold. Also should stay -- but only because a magnetometer
     is bounding it. This is the one to distrust near metal.
  4. Spin it fast about yaw, then stop. Watch for heading that comes back to a
     DIFFERENT value: that is the gyro-only drift the datasheet quotes as
     3 deg per 25 minutes, showing up in seconds under high rate.
  5. Hold it near a running motor. If heading moves and roll/pitch do not, the
     fusion is decoupling the two correctly, which is what a good AHRS does.

Read-only: talks to the port, writes nothing. Use tm151_record.py to capture.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tm151_record import open_port  # noqa: E402
from tm151_serial import CMD, Decoder, request  # noqa: E402

TRACE_S = 20.0


class Stream:
    """Latest attitude/rate from the port, plus a rolling history.

    Holds the LAST value of each field rather than requiring one packet to
    carry them all: the unit may stream `rpy` and `raw_gyro_acc_mag` as
    separate packets (it does on stock firmware), so attitude and rate arrive
    independently and neither should wait for the other.
    """

    def __init__(self, ser, poll: str | None = None):
        self.ser = ser
        self.dec = Decoder()
        self.rpy = np.zeros(3)
        self.gyro = np.zeros(3)
        self.qos = None
        self.t0 = time.monotonic()
        self.n = 0
        self.hist_t: deque = deque()
        self.hist_rpy: deque = deque()
        self.hist_gyro: deque = deque()
        self._cmd = {v: k for k, v in CMD.items()}.get(poll) if poll else None
        self._last_poll = 0.0

    def pump(self) -> None:
        now = time.monotonic() - self.t0
        if self._cmd is not None and now - self._last_poll > 0.05:
            self.ser.write(request(self._cmd))
            self._last_poll = now
        waiting = getattr(self.ser, "in_waiting", 0)
        for p in self.dec.feed(self.ser.read(max(1, min(waiting, 8192)))):
            self.n += 1
            f = p.fields
            if "rpy_deg" in f:
                self.rpy = f["rpy_deg"]
            elif "quat" in f:
                from aow_sim.sim_ahrs import rpy_from_quat
                self.rpy = np.degrees(rpy_from_quat(f["quat"]))
            if "gyro" in f:
                self.gyro = np.degrees(f["gyro"])
            if "qos" in f:
                self.qos = f["qos"]
        self.hist_t.append(now)
        self.hist_rpy.append(self.rpy.copy())
        self.hist_gyro.append(self.gyro.copy())
        while self.hist_t and self.hist_t[0] < now - TRACE_S:
            self.hist_t.popleft(); self.hist_rpy.popleft(); self.hist_gyro.popleft()

    @property
    def hz(self) -> float:
        el = time.monotonic() - self.t0
        return self.n / el if el > 0 else 0.0


def run_ascii(st: Stream, seconds: float) -> int:
    """Terminal view. Works over ssh and needs no GUI backend."""
    print("  roll/pitch/yaw, live. ctrl-c to stop.\n")
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        st.pump()
        r, p, y = st.rpy
        # A 41-wide bar per axis, centred, so level reads as centred.
        def bar(v, span):
            i = int(np.clip((v / span + 1) / 2, 0, 1) * 40)
            return "-" * i + "|" + "-" * (40 - i)
        q = f" qos {st.qos}" if st.qos is not None else ""
        print(f"\r  roll {r:+7.2f} [{bar(r, 90)}]  pitch {p:+7.2f}  "
              f"yaw {y:7.2f}  {st.hz:5.1f} Hz{q}", end="", flush=True)
        time.sleep(0.05)
    print()
    return 0


def run_gui(st: Stream, seconds: float) -> int:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Circle

    fig = plt.figure(figsize=(11, 7.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.15, 1])
    ah = fig.add_subplot(gs[0, 0]); comp = fig.add_subplot(gs[0, 1])
    tr = fig.add_subplot(gs[1, :])

    # --- artificial horizon ------------------------------------------------
    ah.set_xlim(-1.3, 1.3); ah.set_ylim(-1.3, 1.3); ah.set_aspect("equal")
    ah.axis("off"); ah.set_title("attitude", fontsize=10)
    horizon, = ah.plot([-2, 2], [0, 0], color="w", lw=2, zorder=1)
    # The sky/ground wedges are REPLACED each frame (fill_between has no
    # set_data), so the current pair is kept in a one-slot dict rather than in
    # a closure variable -- Python 3 would need `nonlocal` and this reads
    # better next to the clip-path assignment.
    fills: dict[str, object] = {}
    ah.add_patch(Circle((0, 0), 1.25, fill=False, lw=2, color="k", zorder=4))
    # Fixed aircraft reference: the instrument moves, this does not.
    ah.plot([-0.5, -0.15], [0, 0], color="k", lw=3, zorder=5)
    ah.plot([0.15, 0.5], [0, 0], color="k", lw=3, zorder=5)
    ah.plot([0], [0], "ko", ms=5, zorder=5)
    ah_txt = ah.text(0, -1.15, "", ha="center", fontsize=9, zorder=6)
    clip = Circle((0, 0), 1.25, transform=ah.transData)
    horizon.set_clip_path(clip)

    # --- compass -----------------------------------------------------------
    comp.set_xlim(-1.3, 1.3); comp.set_ylim(-1.3, 1.3); comp.set_aspect("equal")
    comp.axis("off"); comp.set_title("heading (the magnetometer's channel)",
                                     fontsize=10)
    comp.add_patch(Circle((0, 0), 1.0, fill=False, lw=2, color="k"))
    for deg, lab in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
        a = np.radians(90 - deg)
        comp.text(1.15 * np.cos(a), 1.15 * np.sin(a), lab, ha="center",
                  va="center", fontsize=11, weight="bold")
    for deg in range(0, 360, 15):
        a = np.radians(90 - deg)
        r0 = 0.9 if deg % 45 else 0.82
        comp.plot([r0 * np.cos(a), np.cos(a)], [r0 * np.sin(a), np.sin(a)],
                  color="k", lw=0.8)
    needle, = comp.plot([0, 0], [0, 0.9], color="crimson", lw=3)
    comp_txt = comp.text(0, -1.15, "", ha="center", fontsize=9)

    # --- traces ------------------------------------------------------------
    tr.set_xlim(-TRACE_S, 0); tr.set_ylim(-95, 95)
    tr.set_xlabel("seconds ago"); tr.set_ylabel("deg  /  deg/s")
    tr.grid(alpha=0.3)
    lines = {n: tr.plot([], [], lw=1.2, label=n)[0]
             for n in ("roll", "pitch", "yaw (rel)")}
    lines["|gyro|"], = tr.plot([], [], lw=0.9, color="gray", alpha=0.8,
                               label="|gyro| deg/s")
    tr.legend(fontsize=8, loc="upper left", ncol=4)

    t_start = time.monotonic()

    def update(_frame):
        st.pump()
        r, p, y = st.rpy
        # Horizon: rotate by -roll, slide by pitch. 1.0 of the dial per 45 deg.
        th = np.radians(-r)
        off = np.clip(p / 45.0, -1.2, 1.2)
        xs = np.array([-2.0, 2.0])
        top = -xs * np.tan(th) + off / max(1e-6, abs(np.cos(th)))
        horizon.set_data(xs, top)
        for old_patch in fills.values():
            old_patch.remove()
        fills["sky"] = ah.fill_between(xs, top, 3, color="#4a90d9", zorder=0)
        fills["gnd"] = ah.fill_between(xs, -3, top, color="#8b6b3d", zorder=0)
        for patch in fills.values():
            patch.set_clip_path(clip)
        ah_txt.set_text(f"roll {r:+.2f}   pitch {p:+.2f}")

        a = np.radians(90 - y)
        needle.set_data([0, 0.9 * np.cos(a)], [0, 0.9 * np.sin(a)])
        q = f"   qos {st.qos}" if st.qos is not None else ""
        comp_txt.set_text(f"yaw {y:.2f} deg   {st.hz:.0f} Hz{q}")

        if st.hist_t:
            tt = np.array(st.hist_t); tt = tt - tt[-1]
            H = np.array(st.hist_rpy); G = np.array(st.hist_gyro)
            lines["roll"].set_data(tt, H[:, 0])
            lines["pitch"].set_data(tt, H[:, 1])
            # Yaw relative to its own start: absolute heading swamps the scale.
            yy = np.unwrap(np.radians(H[:, 2])) * 180 / np.pi
            lines["yaw (rel)"].set_data(tt, yy - yy[0])
            lines["|gyro|"].set_data(tt, np.linalg.norm(G, axis=1))
        if seconds < 1e8 and time.monotonic() - t_start > seconds:
            plt.close(fig)
        return ()

    # cache_frame_data off: this is a live stream, not a fixed sequence.
    # Parked ON THE FIGURE, not in a local: matplotlib only holds a weak
    # reference, so a local would be collected and the animation would stop
    # (silently, showing a frozen first frame).
    fig._tm151_anim = FuncAnimation(fig, update, interval=40, blit=False,
                                    cache_frame_data=False)
    fig.suptitle("TM151 live — move it and watch it track", y=0.98)
    fig.tight_layout()
    plt.show()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", default="auto")
    ap.add_argument("--ascii", action="store_true",
                    help="terminal view; no GUI backend needed")
    ap.add_argument("--seconds", type=float, default=1e9)
    ap.add_argument("--request", default=None, choices=sorted(CMD.values()),
                    help="poll instead of listening passively")
    args = ap.parse_args()

    ser, _baud = open_port(args.port, args.baud)
    st = Stream(ser, args.request)
    try:
        return run_ascii(st, args.seconds) if args.ascii \
            else run_gui(st, args.seconds)
    except KeyboardInterrupt:
        print("\n  stopped")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
