"""The ground station: keyboard -> UDP command struct -> the bike.

    python -m aow_sim.hw.ground --host aowbike.local
    python -m aow_sim.hw.ground --host 127.0.0.1     # on the Pi, over ssh

WHY THIS EXISTS AT ALL. `run_bike`'s command-age watchdog is armed by the
ground station being there, and before this module nothing in the repo sent a
single datagram to port 9910 -- so the bike could not be run, by anyone, ever.
Teleop is not a substitute: `run_drive.py`'s key handling lives inside MuJoCo's
viewer callback and needs a rendering host and a compiled model, neither of
which exists on a Pi or on a bench.

IT IS A TERMINAL PROGRAM, on purpose, and since the MuJoCo mirror
(`run_drive --mirror`) it is the FALLBACK station, not the main one. Nothing
on the bike imports it; the mirror imports `OperatorState` from here, which is
the one reason the file must stay. What it is still for:

  * a station on any machine with Python -- no MuJoCo, no mjpython, no
    display: `--host aowbike.local`;
  * a keyboard plugged into the Pi itself, or a console with no wifi at all;
  * torque-off or on-a-stand checks run on the Pi over `ssh -t`, against
    127.0.0.1.

NO KEYBOARD IS NEEDED ON THE PI for that last one: `ssh -t` carries the
laptop's keys into this terminal. A gamepad is not read at all.

THE LOCAL MODE DEFEATS THE DEAD-MAN, which is why it is bench-only. The
command is re-sent at 50 Hz whether or not a key arrived, so run on the Pi
the heartbeat is generated ON THE BIKE. If wifi stalls without ssh actually
dropping, the operator's keys stop arriving and the bike keeps receiving
"carry on at the last speed" from localhost -- `run_bike`'s 1 s link watchdog
never trips. (An earlier version of this note sold local mode as "no radio in
the loop"; the keys still come over the radio, the safety net is what goes.)
Run from the laptop instead, the radio carries the heartbeat and a stalled
link stops the bike, which is the property that matters once it can move. It is deliberately NOT a port of teleop's
hold-to-accelerate model: a terminal gets key repeats, not key-down/key-up, so
the honest thing is a step per keypress rather than a simulated hold. Teleop's
`_KeyState` exists precisely because that distinction is hard, and guessing at
it over ssh would be worse than a predictable step.

THE COMMAND IS STREAMED, NOT SENT ON CHANGE. 50 Hz of tiny datagrams whether
or not anything moved, because the datagram IS the heartbeat: silence for
150 ms zeroes the bike's velocity and for 1 s drops its torque. A station that
only speaks when the operator does would look identical to a station that has
crashed.

Keys. THE ARROWS AND `/` ARE THE PRIMARY BINDING, matching run_drive's teleop
so one pair of hands works both; the letters are aliases for a terminal that
eats escape sequences (ssh through something odd, tmux misconfigured).

    up / down     forward / reverse, one step of 0.1 m/s      (also w / s)
    left / right  heading left / right, 10 deg                (also a / d)
    /             stop -- zero velocity AND re-aim the heading command at
                  where the bike actually points, which is what teleop's `/`
                  does. Needs telemetry; without it, it zeroes velocity and
                  holds the commanded heading                 (also space)
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
    q         quit -- and quitting stops the heartbeat: a second later the
              bike cuts (policy servos limp) and WAITS for a station. It no
              longer exits; restart a station and press r to re-arm
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time
from pathlib import Path

from .telemetry import check_version, encode_command, new_messages

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
        # None UNTIL THE BIKE HAS SAID WHERE IT POINTS. It was 0.0, which is
        # the AHRS's own yaw zero -- an arbitrary direction -- so every station
        # connected commanding a turn of 45-66 deg (measured, four sessions
        # 2026-09-18). Absent, the packet carries no heading and the bike keeps
        # its own. See `sync`.
        self.psi = None
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
        # A COUNT, not a flag: one per press of `r`, sent in EVERY packet.
        # The bike acts when it changes. It used to be `rearm: true` in the
        # one packet after the press, so a single dropped datagram silently
        # ate the operator's re-arm; a count survives any number of losses
        # (the next packet carries it) and duplicates are harmless.
        self.rearms = 0
        self.quit = False
        # The bike's own heading, from telemetry. None until the first packet.
        # `/` needs it to do what teleop's `/` does; without it that key
        # degrades to "zero the velocity" rather than lying about the heading.
        self.psi_actual = None

    def sync(self, telemetry: dict) -> None:
        """Adopt what only the BIKE knows. Pure -- takes the decoded dict.

        Two things the station cannot work out for itself:

          * the bike's heading, so `/` can re-aim rather than guess;
          * the Goal Current already on the righting servo, written at startup
            from `control.onboard.righting_current`. Without this the first
            `[` or `]` steps from zero, so the operator's first nudge is a
            300-count drop they did not ask for.

        `righting_current` is adopted ONCE, and only while the operator has not
        touched it -- after that the station's value is the intent and the
        telemetry is an echo of it arriving a tick late.
        """
        if not telemetry:
            return
        psi = telemetry.get("psi")
        if psi is not None:
            self.psi_actual = float(psi)
            # THE COMMAND FOLLOWS THE BIKE WHENEVER THE POLICY IS NOT DRIVING,
            # and on the first packet. So a re-arm -- after a fall, a lost
            # link, a faulted servo -- starts from where the bike points now,
            # not from a heading set before it went down; and a fresh station
            # adopts the bike's heading instead of commanding the AHRS's zero.
            # Reads the bike's `state`, a level it owns: not a second copy of
            # any rule about when it cuts.
            if self.psi is None or telemetry.get("state") != "engaged":
                self.psi = _wrap(self.psi_actual)
                if telemetry.get("state") != "engaged":
                    self.v = 0.0
        if self.righting_current is None:
            got = telemetry.get("righting_current")
            if got is not None:
                self.righting_current = int(got)

    def key(self, ch: str) -> None:
        if ch in ("w", "UP"):
            self.v = min(self.v_max, self.v + V_STEP)
        elif ch in ("s", "DOWN"):
            self.v = max(-self.v_max, self.v - V_STEP)
        elif ch in ("a", "LEFT") and self.psi is not None:
            self.psi = _wrap(self.psi + PSI_STEP)
        elif ch in ("d", "RIGHT") and self.psi is not None:
            self.psi = _wrap(self.psi - PSI_STEP)
        elif ch in (" ", "/"):
            self.v = 0.0
            # teleop's zero_command re-anchors the heading ON THE BIKE so a
            # policy never inherits a stale setpoint. Same here, when telemetry
            # has told us where the bike points.
            if self.psi_actual is not None:
                self.psi = _wrap(self.psi_actual)
        elif ch in "94" and self.travel is not None:
            self.pos = max(-1, min(1, self.pos + (1 if ch == "9" else -1)))
            self.righting = self.stow + self.pos * self.travel
        elif ch in "[]":
            base = 0 if self.righting_current is None else self.righting_current
            self.righting_current = base + (CURRENT_STEP if ch == "]"
                                            else -CURRENT_STEP)
        elif ch == "r":
            self.rearms += 1
        elif ch in ("q", "\x03"):          # q or ctrl-C
            self.quit = True

    def packet(self) -> dict:
        """The datagram. Every field is a LEVEL -- the current state of the
        operator's inputs -- re-sent whole every packet, so a lost one costs
        nothing. `rearm_n` is how the one event (a press of `r`) becomes a
        level: see `self.rearms`."""
        # NO `mode` FIELD. The struct used to carry one and the bike never
        # read it: `general_rl` is the only deployable controller (the LQR
        # needs a world anchor it cannot have onboard -- see
        # docs/plans/untethered-setup.md, "Which controller deploys"), so a
        # mode key would be a control the operator believes they have. When a
        # second onboard controller exists it goes into both halves at once,
        # which is what test_every_field_the_station_sends_is_read pins.
        th = 0.0 if self.psi is None else self.psi
        out = {"v_cmd_world": [self.v * math.cos(th), self.v * math.sin(th)]}
        if self.psi is not None:          # absent: the bike keeps its own
            out["psi_cmd"] = self.psi
        if self.righting is not None:
            out["righting_rad"] = self.righting
        if self.righting_current is not None:
            out["righting_current"] = self.righting_current
        out["rearm_n"] = self.rearms
        return out


# xterm arrow keys, as they arrive in cbreak mode. Three bytes, and the third
# is the only one that differs. `\x1bO` is the "application cursor" variant
# some terminals send instead -- both are decoded, because which one you get
# depends on the terminal's mode rather than on the keyboard.
_ARROWS = {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}


def decode_keys(buf: str):
    """-> (keys, leftover). Splits a raw read into key names.

    Returns the tail unconsumed when an escape sequence is still arriving, so
    a three-byte arrow split across two reads is not seen as ESC + garbage.
    That split is rare locally and common over ssh, which is exactly where the
    station runs.

    A bare ESC (no bracket behind it) is DROPPED rather than passed through.
    It is what a terminal sends for a key this station has no use for, and
    letting it fall into `key()` would make a stray function key look like a
    command.
    """
    keys, i = [], 0
    while i < len(buf):
        ch = buf[i]
        if ch != "\x1b":
            keys.append(ch)
            i += 1
            continue
        if i + 1 >= len(buf):
            return keys, buf[i:]              # ESC alone: might be a prefix
        if buf[i + 1] not in "[O":
            i += 2                            # ESC + something else: drop both
            continue
        if i + 2 >= len(buf):
            return keys, buf[i:]              # still arriving
        name = _ARROWS.get(buf[i + 2])
        if name:
            keys.append(name)
        i += 3
    return keys, ""


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
        return f"v {op.v:+.2f}  psi   --     no telemetry yet"
    stale = "  STALE" if age > 0.5 else ""
    # `v_world`, not `v`: since schema v2 the key `v` is the SCHEMA VERSION.
    # This line read `t.get("v", [0, 0])` and would have printed the integer 2
    # as the velocity vector -- a rename that a .get() default swallows.
    return (f"v {op.v:+.2f} psi {math.degrees(op.psi or 0.0):+6.1f} | "
            f"{t.get('state', '?'):7s} roll {math.degrees(t.get('roll', 0)):+6.1f} "
            f"vel {t.get('v_world', [0, 0])} steer {t.get('steer', 0):+.3f} "
            f"{t.get('volts', 0):.1f}V qos {t.get('qos', '?')} "
            f"jit {t.get('jitter_ms', 0):.2f}ms cuts {t.get('cuts', 0)}{stale}")


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
    pending = ""
    checked = False
    log_seq = 0

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
                # Read what is THERE, not one byte: an arrow key is three
                # bytes and reading them one select() at a time would work
                # only by luck. `pending` carries a sequence split across
                # reads -- see decode_keys.
                pending += os.read(fd, 64).decode("utf-8", "replace")
                keys, pending = decode_keys(pending)
                for k in keys:
                    op.key(k)
            if sock in ready:
                try:
                    telemetry = json.loads(sock.recv(4096).decode())
                    if telemetry and not checked:
                        check_version(telemetry)      # raises on a stale deploy
                        checked = True
                    t_tel = time.monotonic()
                    op.sync(telemetry)
                    msgs, log_seq = new_messages(telemetry, log_seq)
                    for _seq, kind, text in msgs:
                        print(f"\nbike [{kind}]: {text}")
                except (OSError, ValueError):
                    pass
            now = time.monotonic()
            if now >= next_tx:
                next_tx = now + 1.0 / SEND_HZ
                try:
                    sock.sendto(encode_command(op.packet()), dest)
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
