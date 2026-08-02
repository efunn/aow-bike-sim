"""Diagnose whether hold-to-drive can work on this machine.

    python -m aow_sim.check_input          # then hold an arrow key

MuJoCo's viewer reports a key going DOWN and nothing else — no release, no
auto-repeat — so teleop can only offer hold-to-drive if the OS will tell us
real key state some other way. This checks each route and says which (if any)
works here, so a dead hold can be diagnosed as permissions vs. platform vs.
code rather than guessed at.

Run it from the same terminal you launch the sim from: macOS grants Input
Monitoring per-application, so the answer can differ between terminals, and
between `python` and `mjpython`.
"""

from __future__ import annotations

import time

ARROWS = {"left": 123, "right": 124, "down": 125, "up": 126}   # macOS kVK_*


def _probe_quartz(seconds: float = 6.0) -> str:
    """Poll CGEventSourceKeyState and report whether it ever sees a key down.

    Needs macOS Input Monitoring permission. Without it the call does NOT
    fail — it silently reports every key as up forever, which is exactly the
    'holding does nothing' symptom."""
    try:
        from Quartz import (CGEventSourceKeyState,
                            kCGEventSourceStateCombinedSessionState as SESSION,
                            kCGEventSourceStateHIDSystemState as HID)
    except Exception as e:
        print(f"  Quartz unavailable ({type(e).__name__}: {e})")
        print("  -> pip install -e '.[teleop]'   (into THIS interpreter)")
        return "missing"

    print(f"  polling for {seconds:.0f} s — HOLD an arrow key now...")
    sources = {"HID": HID, "session": SESSION}
    seen = {name: set() for name in sources}
    t0 = time.time()
    while time.time() - t0 < seconds:
        for sname, src in sources.items():
            for kname, kc in ARROWS.items():
                try:
                    if CGEventSourceKeyState(src, kc):
                        seen[sname].add(kname)
                except Exception as e:
                    print(f"  {sname}: call failed ({type(e).__name__}: {e})")
                    return "missing"
        time.sleep(0.01)

    ok = False
    for sname in sources:
        if seen[sname]:
            print(f"  {sname:8s} SAW: {sorted(seen[sname])}  -> works")
            ok = True
        else:
            print(f"  {sname:8s} saw nothing")
    if not ok:
        print("\n  Installed, but no key state readable. Either no key was")
        print("  actually held, or this app lacks Input Monitoring. Grant it")
        print("  in System Settings -> Privacy & Security -> Input Monitoring,")
        print("  add the terminal (and/or mjpython), RESTART it and re-run.")
    return "ok" if ok else "denied"


def _probe_nsevent(seconds: float = 6.0) -> bool:
    """Try an NSEvent local monitor, which sees key up/down delivered to our
    OWN process and needs no special permission — but only fires while an
    NSApplication event loop is running (i.e. a window is up), so this
    standalone probe usually reports 'no event loop'."""
    try:
        from AppKit import (NSApplication, NSEvent, NSEventMaskKeyDown,
                            NSEventMaskKeyUp)
    except Exception as e:
        print(f"  AppKit unavailable ({type(e).__name__}: {e})")
        print("  -> pip install -e '.[teleop]'   (into THIS interpreter)")
        return False

    app = NSApplication.sharedApplication()
    got = []

    def handler(event):
        got.append(event.keyCode())
        return event

    NSEvent.addLocalMonitorForEventsMatchingMask_handler_(
        NSEventMaskKeyDown | NSEventMaskKeyUp, handler)
    if not app.isRunning():
        print("  monitor installed, but no NSApplication event loop is running")
        print("  -> can only be judged inside the viewer, not standalone")
        return False
    time.sleep(seconds)
    print(f"  saw {len(got)} key events")
    return bool(got)


def main() -> None:
    import platform
    import sys
    print(f"platform: {platform.system()} {platform.release()}")
    # The interpreter matters more than anything else here: pyobjc installed
    # in a different env is exactly as useless as not installing it, and the
    # sim, mjpython and a bare `python` can easily be three different envs.
    print(f"python:   {sys.executable}")
    print(f"          {platform.python_implementation()} "
          f"{platform.python_version()}")

    if platform.system() != "Darwin":
        print("\nNot macOS: no key-state backend is wired up, so teleop uses "
              "the tap model.\n(Tapping the arrows steps the command; that "
              "path always works.)")
        return

    print("\n[1] Quartz key-state polling (what teleop uses; needs Input "
          "Monitoring)")
    quartz_ok = _probe_quartz()

    print("\n[2] NSEvent local monitor (no permission needed, needs a window)")
    _probe_nsevent(seconds=0.0)

    print("\n" + "=" * 68)
    if quartz_ok == "missing":
        print("pyobjc is NOT installed in this interpreter, so neither route")
        print("exists and teleop can only offer tap-to-step. Install it into")
        print("the SAME env you run the sim from:")
        print("    pip install -e '.[teleop]'")
        print("then re-run this check. (An install in a different conda env is")
        print("exactly as useless as no install at all — compare the `python:`")
        print("line above with the one the sim prints.)")
    elif quartz_ok == "ok":
        print("Quartz works here, so hold-to-drive should work in teleop.")
        print("If it still doesn't, the viewer runs under mjpython — re-run")
        print("as `mjpython -m aow_sim.check_input` and grant Input")
        print("Monitoring to mjpython specifically.")
    else:
        print("pyobjc is installed but Quartz read no keys in THIS process")
        print("(most likely the Input Monitoring permission). Not fatal:")
        print("teleop also installs an NSEvent local monitor, which needs no")
        print("permission and only works once the viewer window exists — so")
        print("it cannot be judged from here. Launch the sim and hold an arrow:")
        print("    mjpython -m aow_sim.run_drive --teleop")
        print("It prints 'hold-to-drive active (via ...)' the moment either")
        print("route reports a real key. If that line never appears, hold is")
        print("unavailable and tapping remains the (fully working) input.")


if __name__ == "__main__":
    main()
