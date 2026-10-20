"""A wired Xbox One pad over raw USB: the analog sticks and triggers, no driver.

    python -m aow_sim.control.gamepad_usb        # live readout, to test a pad

WHY RAW USB. A wired Xbox One controller does not speak USB HID; it speaks
Microsoft's GIP (interface class 0xFF, subclass 0x47, protocol 0xD0). macOS
ships a driver for Microsoft's own pads only, so a third-party one -- the PDP
"Wired Controller for Xbox One" (0e6f:02a7) this was written against,
2026-10-19 -- enumerates on the bus with NO driver attached: no HID device, so
Apple's GameController framework, SDL, pyglet and the browser all see nothing.
The upside of nobody owning it is that libusb can claim it from user space,
send the start-up packets itself, and read the input reports directly.

Matching is by the GIP interface, not by a vendor/product list, so any wired
Xbox One-protocol pad that macOS has not claimed works the same way. A pad the
OS DOES drive (Bluetooth Xbox, DualSense, ...) is not this module's job: it
would come in through a native backend producing the same `PadState`.

THE OUTPUT IS THE STANDARD LAYOUT, named for the Xbox labels every mainstream
API uses (W3C Standard Gamepad, SDL GameController, Apple's
GCExtendedGamepad): `lx ly rx ry` in [-1, 1] with +x RIGHT and +y UP (pushed
away from you), triggers `lt rt` in [0, 1], and named buttons. Nothing about
the bike lives here; `gamepad.from_state` maps this onto the bike's `Pad`, so
a different controller -- or a different backend -- changes nothing
downstream.

THE PROTOCOL, from the Linux `xpad` driver (drivers/input/joystick/xpad.c):

  out  05 20 ss 01 00               power on: no input reports until this
  out  0a 20 ss 03 00 01 14         PDP only: LED on
  out  06 20 ss 02 01 00            PDP only: authentication done
  in   20 00 ss 0e ...              input report (ss is a sequence byte):
         [4] bit 2 MENU(start) 3 VIEW(back) 4 A 5 B 6 X 7 Y
         [5] bit 0 up 1 down 2 left 3 right 4 LB 5 RB 6 LS 7 RS
         [6:8] LT, [8:10] RT     u16 LE, 0..1023
         [10:12] LX, [12:14] LY, [14:16] RX, [16:18] RY   s16 LE, +y UP
  in   07 2x ss 02 bb 5b            the Xbox (guide) button, bit 0 of [4];
                                    if [1] & 0x10 the pad wants an ACK
  in   03 20 ...                    heartbeat; ignored

Measured on the PDP at rest: the sticks sit 236..1181 counts off zero (at most
3.6% of full scale), inside `gamepad.DEADZONE`'s 8%.

Needs `pyusb` and libusb (`brew install libusb`). Both are imported lazily, so
nothing that merely imports this module needs them -- the Pi never will.
"""

from __future__ import annotations

import ctypes.util
import glob
import struct
import threading
import time
from dataclasses import dataclass, field, replace

GIP_CLASS, GIP_SUBCLASS = 0xFF, 0x47
PDP_VENDOR = 0x0E6F

POWER_ON = bytes([0x05, 0x20, 0x00, 0x01, 0x00])
PDP_LED_ON = bytes([0x0A, 0x20, 0x00, 0x03, 0x00, 0x01, 0x14])
PDP_AUTH = bytes([0x06, 0x20, 0x00, 0x02, 0x01, 0x00])

# Bit -> standard button name, per byte of the 0x20 report.
_BYTE4 = {2: "START", 3: "BACK", 4: "A", 5: "B", 6: "X", 7: "Y"}
_BYTE5 = {0: "DPAD_UP", 1: "DPAD_DOWN", 2: "DPAD_LEFT", 3: "DPAD_RIGHT",
          4: "LB", 5: "RB", 6: "LS", 7: "RS"}


@dataclass
class PadState:
    """One controller snapshot in the standard layout. +x right, +y up."""

    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    lt: float = 0.0
    rt: float = 0.0
    buttons: frozenset[str] = field(default_factory=frozenset)


def _axis(raw: int) -> float:
    return max(-1.0, min(1.0, raw / 32767.0))


def parse_input(report: bytes, guide: bool = False) -> PadState | None:
    """A GIP 0x20 input report -> PadState, or None if it is not one.
    `guide` carries the Xbox button, which arrives in its own report."""
    if len(report) < 18 or report[0] != 0x20:
        return None
    b4, b5 = report[4], report[5]
    pressed = {n for bit, n in _BYTE4.items() if b4 >> bit & 1}
    pressed |= {n for bit, n in _BYTE5.items() if b5 >> bit & 1}
    if guide:
        pressed.add("GUIDE")
    lt, rt = struct.unpack_from("<HH", report, 6)
    lx, ly, rx, ry = struct.unpack_from("<hhhh", report, 10)
    return PadState(lx=_axis(lx), ly=_axis(ly), rx=_axis(rx), ry=_axis(ry),
                    lt=min(1.0, lt / 1023.0), rt=min(1.0, rt / 1023.0),
                    buttons=frozenset(pressed))


def _backend():
    import usb.backend.libusb1 as libusb1
    paths = ([ctypes.util.find_library("usb-1.0")]
             + glob.glob("/opt/homebrew/lib/libusb-1.0*.dylib")
             + glob.glob("/usr/local/lib/libusb-1.0*.dylib"))
    for p in filter(None, paths):
        be = libusb1.get_backend(find_library=lambda _n, p=p: p)
        if be is not None:
            return be
    return None


def _is_gip(dev) -> bool:
    try:
        return any(i.bInterfaceClass == GIP_CLASS
                   and i.bInterfaceSubClass == GIP_SUBCLASS
                   for cfg in dev for i in cfg)
    except Exception:              # a device that will not describe itself
        return False


class GipPad:
    """A claimed GIP pad, read on a background thread.

    `state()` is the latest snapshot; `presses()` drains the buttons that went
    DOWN since the last call, so a tap shorter than one frame is not lost.
    `connected` goes False if the pad is unplugged; nothing reconnects.
    """

    def __init__(self, dev):
        import usb.util
        self._usb_util = usb.util
        self.dev = dev
        self.name = f"{dev.manufacturer or ''} {dev.product or ''}".strip() \
            if _safe_strings(dev) else f"{dev.idVendor:04x}:{dev.idProduct:04x}"
        try:
            dev.set_configuration()
        except Exception:
            pass                    # already configured
        usb.util.claim_interface(dev, 0)
        self._seq = 0
        self._lock = threading.Lock()
        self._state = PadState()
        self._guide = False
        self._presses: list[str] = []
        self.connected = True
        self.reports = 0
        self._send(POWER_ON)
        if dev.idVendor == PDP_VENDOR:
            self._send(PDP_LED_ON)
            self._send(PDP_AUTH)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="gamepad-usb")
        self._thread.start()

    def _send(self, pkt: bytes) -> None:
        b = bytearray(pkt)
        b[2] = self._seq & 0xFF
        self._seq += 1
        self.dev.write(0x01, bytes(b), 200)

    def _run(self) -> None:
        import usb.core
        while not self._stop.is_set():
            try:
                r = bytes(self.dev.read(0x81, 64, 100))
            except usb.core.USBTimeoutError:
                continue
            except usb.core.USBError:
                self.connected = False
                return
            if not r:
                continue
            if r[0] == 0x07 and len(r) >= 5:
                if r[1] & 0x10:     # the pad asked for an acknowledgement
                    try:
                        self._send(bytes([0x01, 0x20, 0, 0x09, 0x00, 0x07,
                                          0x20, 0x02, 0, 0, 0, 0, 0]))
                    except usb.core.USBError:
                        pass
                with self._lock:
                    self._guide = bool(r[4] & 1)
                    held = self._state.buttons - {"GUIDE"}
                    if self._guide:
                        self._presses.append("GUIDE")
                        held = held | {"GUIDE"}
                    self._state = replace(self._state, buttons=frozenset(held))
                continue
            with self._lock:
                s = parse_input(r, self._guide)
                if s is None:
                    continue
                self._presses.extend(sorted(s.buttons - self._state.buttons))
                self._state = s
                self.reports += 1

    def state(self) -> PadState:
        with self._lock:
            return self._state

    def presses(self) -> list[str]:
        with self._lock:
            out, self._presses = self._presses, []
        return out

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=0.5)
        try:
            self._usb_util.release_interface(self.dev, 0)
            self._usb_util.dispose_resources(self.dev)
        except Exception:
            pass


def _safe_strings(dev) -> bool:
    try:
        return bool(dev.manufacturer or dev.product)
    except Exception:
        return False


def open_gamepad(verbose: bool = True) -> GipPad | None:
    """The first unclaimed GIP pad on the bus, started, or None -- with the
    reason printed, because "no pad" has three different fixes."""
    try:
        import usb.core
    except ImportError:
        if verbose:
            print("gamepad: pyusb is not installed (pip install pyusb)")
        return None
    be = _backend()
    if be is None:
        if verbose:
            print("gamepad: libusb not found (brew install libusb)")
        return None
    devs = [d for d in usb.core.find(find_all=True, backend=be) if _is_gip(d)]
    if not devs:
        if verbose:
            print("gamepad: no wired Xbox One-protocol pad on USB")
        return None
    try:
        pad = GipPad(devs[0])
    except Exception as e:          # claimed by a driver, or permissions
        if verbose:
            print(f"gamepad: found a GIP pad but could not claim it ({e})")
        return None
    if verbose:
        print(f"gamepad: {pad.name} over raw USB (GIP)")
    return pad


def main() -> None:
    pad = open_gamepad()
    if pad is None:
        raise SystemExit(1)
    print("move the sticks, pull the triggers, press buttons -- Ctrl-C quits")
    try:
        while pad.connected:
            s = pad.state()
            print(f"\rL {s.lx:+.2f} {s.ly:+.2f}  R {s.rx:+.2f} {s.ry:+.2f}  "
                  f"LT {s.lt:.2f} RT {s.rt:.2f}  "
                  f"{' '.join(sorted(s.buttons)):<40}", end="", flush=True)
            time.sleep(0.03)
        print("\npad disconnected")
    except KeyboardInterrupt:
        print()
    finally:
        pad.close()


if __name__ == "__main__":
    main()
