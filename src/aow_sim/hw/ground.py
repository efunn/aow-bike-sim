"""The ground station: keyboard -> UDP command struct -> the bike.

    python -m aow_sim.hw.ground --host aowbike.local
    python -m aow_sim.hw.ground --host 127.0.0.1     # on the Pi, over ssh

WHY THIS EXISTS AT ALL. `run_bike`'s command-age watchdog is armed by the
ground station being there, and before this module nothing in the repo sent a
single datagram to port 9910 -- so the bike could not be run, by anyone, ever.
Teleop is not a substitute: `run_drive.py`'s key handling lives inside MuJoCo's
viewer callback and needs a rendering host and a compiled model, neither of
which exists on a Pi or on a bench.

IT IS A TERMINAL PROGRAM, on purpose. Raw-mode stdin over ssh works from
anywhere, including from the bike itself, which is what lets the first bench
session happen with no radio in the loop at all -- run the station on the Pi
against 127.0.0.1, prove the control loop, and make WiFi a separate experiment
with its own failure modes. It is deliberately NOT a port of teleop's
hold-to-accelerate model: a terminal gets key repeats, not key-down/key-up, so
the honest thing is a step per keypress rather than a simulated hold. Teleop's
`_KeyState` exists precisely because that distinction is hard, and guessing at
it over ssh would be worse than a predictable step.

THE COMMAND IS STREAMED, NOT SENT ON CHANGE. 50 Hz of tiny datagrams whether
or not anything moved, because the datagram IS the heartbeat: silence for
150 ms zeroes the bike's velocity and for 1 s drops its torque. A station that
only speaks when the operator does would look identical to a station that has
crashed.

Keys:
    w / s     forward / reverse, one step of 0.1 m/s
    a / d     heading left / right, 10 deg
    space     stop -- zero velocity, hold heading
    9 / 4     self-righting servo: STEP the target one notch, 9 one way and 4
              the other, through three positions (one side / centre / the
              other). Latched, as teleop does it — no auto-deploy and no auto
              hand-off. Two keys rather than three because the co-rotating
              linkage only needs a direction, and because 6/7/8 are the
              heading snaps
    [ / ]     righting goal current, down / up. RAW counts; see
              ServoBus.set_righting_current for why it is not milliamps
    r         request re-arm after a fall cut (the bike will not come back on
              its own unless it was started with --auto-rearm)
    q         quit -- and quitting stops the heartbeat, which torques the bike
              off a second later. That is the intended way to stop it
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path

V_STEP = 0.1                  # m/s per keypress
PSI_STEP = math.radians(10.0)
CURRENT_STEP = 20             # raw Goal Current counts per keypress
SEND_HZ = 50.0

# Stow at mid-scale: 2048 counts on a 4096-count turn, so +-travel is reachable
# without winding the servo into extended position.
STOW_RAD = math.radians(180.0)
# Where the deployed angle comes from. `_smaller` is the config that WON the
# swing-linkage optimisation (docs/plans/wing-linkage-design-and-optimization.md
# line 501: "_smaller ... _smaller_v2 was tried as an even smaller envelope and
# did not buy enough"), and the linkage's servo drives the CRANK directly --
# gear ratio 1, unlike the geared pair -- so crank degrees ARE servo degrees.
LINKAGE_CONFIG = (Path(__file__).resolve().parents[3] / "config"
                  / "swing_linkage_smaller.yaml")


class OperatorState:
    """The command struct, and what each key does to it. Pure: no I/O.

    Separated from the terminal so the protocol can be tested without a
    keyboard and without a bike -- the bike's half of this contract is
    `BikeRunner._apply_command`, and the two are only in agreement if someone
    checks.
    """

    def __init__(self, v_max: float = 1.2, stow_rad: float = STOW_RAD,
                 travel_rad=None):
        self.v_max = float(v_max)
        self.v = 0.0                      # signed speed along the heading
        self.psi = 0.0
        self.righting = None              # None = never commanded; see set_righting
        self.righting_current = None
        # THREE POSITIONS ON TWO KEYS, and the stow is mid-scale not zero.
        #
        # The swing linkage CO-ROTATES -- one wing swings down and out while the
        # other swings up and in -- so it has to be told which side. Teleop
        # solves that by STEPPING through -1 / 0 / +1 on the same two keys it
        # uses for the mirrored pair: 9 walks one way, 4 the other, clipped at
        # the ends, so double-tapping either crosses the whole range and
        # "swap sides" is one gesture. Copied exactly, because 7 is taken --
        # 6/7/8 are the heading snaps (+90 / -90 / 180) and a third wing key
        # would collide with one.
        #
        # Stowing at 180 deg puts the servo in the middle of its single-turn
        # range with room to swing either way, which is what makes +-travel
        # reachable without winding into extended position.
        self.stow = float(stow_rad)
        self.travel = None if travel_rad is None else float(travel_rad)
        self.pos = 0                      # -1 / 0 / +1, as teleop's wing["pos"]
        self.rearm = False
        self.quit = False

    def key(self, ch: str) -> None:
        if ch == "w":
            self.v = min(self.v_max, self.v + V_STEP)
        elif ch == "s":
            self.v = max(-self.v_max, self.v - V_STEP)
        elif ch == "a":
            self.psi = _wrap(self.psi + PSI_STEP)
        elif ch == "d":
            self.psi = _wrap(self.psi - PSI_STEP)
        elif ch == " ":
            self.v = 0.0
        elif ch in "94" and self.travel is not None:
            self.pos = max(-1, min(1, self.pos + (1 if ch == "9" else -1)))
            self.righting = self.stow + self.pos * self.travel
        elif ch in "[]":
            base = 0 if self.righting_current is None else self.righting_current
            self.righting_current = base + (CURRENT_STEP if ch == "]"
                                            else -CURRENT_STEP)
        elif ch == "r":
            self.rearm = True
        elif ch in ("q", "\x03"):          # q or ctrl-C
            self.quit = True

    def packet(self) -> dict:
        """The datagram. `rearm` is edge-triggered and consumed here, so one
        keypress asks once rather than re-arming forever."""
        # NO `mode` FIELD. The struct used to carry one and the bike never
        # read it: `general_rl` is the only deployable controller (the LQR
        # needs a world anchor it cannot have onboard -- see
        # docs/plans/untethered-setup.md, "Which controller deploys"), so a
        # mode key would be a control the operator believes they have. When a
        # second onboard controller exists it goes into both halves at once,
        # which is what test_every_field_the_station_sends_is_read pins.
        out = {"v_cmd_world": [self.v * math.cos(self.psi),
                               self.v * math.sin(self.psi)],
               "psi_cmd": self.psi}
        if self.righting is not None:
            out["righting_rad"] = self.righting
        if self.righting_current is not None:
            out["righting_current"] = self.righting_current
        if self.rearm:
            out["rearm"] = True
            self.rearm = False
        return out


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def linkage_travel_deg(path=LINKAGE_CONFIG):
    """`stroke.crank_travel_deg` from a swing-linkage config, or None.

    None if the file is not there -- the station then says so and the righting
    keys stay inert, rather than inventing a stroke for a mechanism that does
    not exist yet.
    """
    import yaml
    try:
        with open(path) as f:
            return float((yaml.safe_load(f) or {})["stroke"]["crank_travel_deg"])
    except (FileNotFoundError, KeyError, TypeError):
        return None


def _status(op: OperatorState, telemetry: dict, age: float) -> str:
    t = telemetry
    if not t:
        return f"v {op.v:+.2f}  psi {math.degrees(op.psi):+6.1f}   no telemetry yet"
    stale = "  STALE" if age > 0.5 else ""
    return (f"v {op.v:+.2f} psi {math.degrees(op.psi):+6.1f} | "
            f"{t.get('state', '?'):7s} roll {math.degrees(t.get('roll', 0)):+6.1f} "
            f"vel {t.get('v', [0, 0])} steer {t.get('steer', 0):+.3f} "
            f"{t.get('volts', 0):.1f}V jit {t.get('jitter_ms', 0):.2f}ms "
            f"cuts {t.get('cuts', 0)}{stale}")


def run(host: str, port: int = 9910, v_max: float = 1.2,
        stow_rad: float = STOW_RAD, travel_rad=None) -> None:
    import select
    import termios
    import tty

    op = OperatorState(v_max=v_max, stow_rad=stow_rad, travel_rad=travel_rad)
    print(f"righting: stow {math.degrees(stow_rad):.0f} deg, "
          + (f"swing +-{math.degrees(travel_rad):.1f} deg" if travel_rad
             else "NO TRAVEL — 9/4 inert, pass --travel-deg"))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    dest = (host, port)
    telemetry, t_tel = {}, 0.0

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        print(__doc__.split("Keys:")[1])
        next_tx = time.monotonic()
        while not op.quit:
            timeout = max(0.0, next_tx - time.monotonic())
            ready, _, _ = select.select([fd, sock], [], [], timeout)
            if fd in ready:
                op.key(sys.stdin.read(1))
            if sock in ready:
                try:
                    telemetry = json.loads(sock.recv(4096).decode())
                    t_tel = time.monotonic()
                except (OSError, ValueError):
                    pass
            now = time.monotonic()
            if now >= next_tx:
                next_tx = now + 1.0 / SEND_HZ
                try:
                    sock.sendto(json.dumps(op.packet()).encode(), dest)
                except OSError as e:
                    print(f"\nsend failed: {e}")
                    break
                print("\r" + _status(op, telemetry, now - t_tel)[:150].ljust(150),
                      end="", flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print("\nstation stopped — the bike will torque off in ~1 s")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="aowbike.local",
                    help="the bike. 127.0.0.1 to drive it from the Pi itself")
    ap.add_argument("--port", type=int, default=9910)
    ap.add_argument("--v-max", type=float, default=1.2)
    ap.add_argument("--stow-deg", type=float, default=180.0,
                    help="righting servo angle for stow, at the SERVO SHAFT. "
                         "Mid-scale by default, so the stroke reaches both "
                         "ways without winding into extended position")
    ap.add_argument("--travel-deg", type=float, default=None,
                    help=f"swing either way from stow. Defaults to "
                         f"stroke.crank_travel_deg in {LINKAGE_CONFIG}")
    args = ap.parse_args()
    travel = args.travel_deg
    if travel is None:
        travel = linkage_travel_deg()
    run(args.host, args.port, args.v_max, math.radians(args.stow_deg),
        None if travel is None else math.radians(travel))


if __name__ == "__main__":
    main()
