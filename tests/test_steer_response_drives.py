"""`steer_response.py --hold-drives`: the drive servos held still while the
steer sweeps on the ground, against a fake bus.

What it guards: velocity mode BEFORE the gains (a mode write resets them),
Goal Velocity 0 before torque on, and everything put back after -- torque
off first, and every restore step tried even when one fails.
"""

import sys

import pytest

sys.path.insert(0, "analysis/servo")
import steer_response as sr  # noqa: E402

pytestmark = pytest.mark.analysis


class FakeBus:
    def __init__(self, mode=3, fail=None):
        self.regs = {d: {"Operating Mode": mode, "Velocity P Gain": 400,
                         "Velocity I Gain": 1000, "Goal Velocity": 7,
                         "Present Position": 0} for d in (101, 102)}
        self.log, self.fail = [], fail

    def read_raw(self, d, k):
        return self.regs[d][k]

    def write_raw(self, d, k, v):
        if self.fail == (d, k):
            raise IOError("no status packet")
        self.log.append(("w", d, k, v))
        self.regs[d][k] = v

    def torque(self, on, ids):
        self.log.append(("torque", tuple(ids), on))


def test_hold_on_sets_mode_then_gains_then_zero_goal_then_torque():
    bus = FakeBus()
    saved = sr.drives_hold_on(bus, (101, 102), (100, 1920))
    assert bus.log[0] == ("torque", (101, 102), False)
    w101 = [e[2:] for e in bus.log if e[0] == "w" and e[1] == 101]
    assert w101 == [("Operating Mode", sr.MODE_VELOCITY), ("Velocity P Gain", 100),
                    ("Velocity I Gain", 1920), ("Goal Velocity", 0)]
    assert bus.log[-1] == ("torque", (101, 102), True)
    assert saved[101] == {"Operating Mode": 3, "Velocity P Gain": 400, "Velocity I Gain": 1000}


def test_hold_off_restores_everything_even_after_a_failure(capsys):
    bus = FakeBus()
    saved = sr.drives_hold_on(bus, (101, 102), (100, 1920))
    bus.log.clear()
    bus.fail = (101, "Operating Mode")
    sr.drives_hold_off(bus, (101, 102), saved)
    assert bus.log[0] == ("torque", (101, 102), False)
    assert bus.regs[102]["Operating Mode"] == 3
    assert bus.regs[101]["Velocity P Gain"] == 400 and bus.regs[102]["Velocity I Gain"] == 1000
    assert "restore step failed" in capsys.readouterr().out


def test_drive_angles_split_into_ring_vs_hub_and_hub():
    bus = FakeBus()
    bus.regs[101]["Present Position"] = 1024     # +90 deg servo
    bus.regs[102]["Present Position"] = 1024
    rvh, hub = sr.drive_angles(bus, (101, 102), (1.0, -1.0))
    assert rvh == pytest.approx(3 * 90.0) and hub == pytest.approx(0.0)


START = [0.0]


class FakeDxl:
    """Enough of DynamixelBus for `steer_response.main`: an XC330 steer on
    the bus, XC430 drives loadable by `discover`. `apply_map` builds the REAL
    IndirectMap check, so a map over a table without a register fails here
    as it does on the Pi (10-25: Present Current on an XC430)."""

    def __init__(self, port, ids=(), **kw):
        from aow_sim.hw.control_table import table_by_name
        self._xc330, self._xc430 = table_by_name("xc330_t181"), table_by_name("xc430_w150")
        self.ids = tuple(ids)
        self.tables = {i: self._xc330 for i in ids}
        self.regs = {103: {"Operating Mode": 4, "Position P Gain": 640, "Position I Gain": 0,
                           "Position D Gain": 0, "Goal Current": 500},
                     101: {"Operating Mode": 1, "Velocity P Gain": 100, "Velocity I Gain": 1920,
                           "Goal Velocity": 0, "Present Position": 0},
                     102: {"Operating Mode": 1, "Velocity P Gain": 100, "Velocity I Gain": 1920,
                           "Goal Velocity": 0, "Present Position": 0}}
        self.torqued = set()
        self.p = START[0]                    # steer shaft, rad
        self.streams = []                    # every capture's goals, in order
        self.torque_log = []

    def open(self):
        return self

    def discover(self, ids=None):
        t = {i: (self._xc330 if i == 103 else self._xc430) for i in ids}
        self.ids, self.tables = tuple(ids), t
        return dict(t)

    def read_raw(self, d, k):
        return self.regs[d][k]

    def write_raw(self, d, k, v):
        self.regs[d][k] = v

    def prepare(self, ids=None):
        pass

    def hardware_errors(self, ids=None):
        return {}

    def apply_map(self, imap, verify=False):
        for i in self.ids:
            imap.address_bytes(i)            # resolves every register per table
        self._map = imap
        return imap

    def read_frame(self):
        return {103: {"Present Position": self.p}}

    def write_frame(self, values):
        pass

    def torque(self, on, ids=None):
        self.torque_log.append((tuple(ids if ids is not None else self.ids), on))
        for d in (ids if ids is not None else self.ids):
            (self.torqued.add if on else self.torqued.discard)(d)

    def capture(self, seconds, rate_hz, command=None, warn_overrun=True):
        import numpy as np
        rows = []
        for t in np.arange(0, seconds, 1 / rate_hz):
            g = command(t, None)
            rows.append({"t_host": t, "command": g, "servos": {103: {
                "Present Position": g[103], "Present Velocity": 0.0,
                "Present Current": 0.1}}})
            self.p = g[103]                  # an ideal servo, for the bookkeeping
        self.streams.append(np.array([r["command"][103] for r in rows]))
        return rows

    def close(self):
        pass


def test_main_with_held_drives_runs_and_restores(tmp_path, monkeypatch, capsys):
    import aow_sim.hw.dynamixel as dx
    made = {}

    def bus(*a, **kw):
        made["bus"] = FakeDxl(*a, **kw)
        return made["bus"]
    monkeypatch.setattr(dx, "DynamixelBus", bus)
    monkeypatch.setattr(sr.time, "sleep", lambda s: None)
    out = tmp_path / "steer.json"
    monkeypatch.setattr(sys, "argv", ["steer_response.py", "--freqs", "5", "--seconds", "1.2",
                                      "--fade", "0.3", "--hold-drives", "--out", str(out)])
    sr.main()
    b = made["bus"]
    assert out.exists()
    assert b.ids == (103,) and not b.torqued
    assert "drives restored" in capsys.readouterr().out
    assert b.regs[101]["Velocity P Gain"] == 100 and b.regs[103]["Position P Gain"] == 640


@pytest.mark.parametrize("deg, goal", [(3.34, 0.0), (-2.0, 0.0), (183.0, 180.0),
                                       (95.0, 180.0), (363.0, 360.0), (-170.0, -180.0)])
def test_straight_goal_is_the_nearest_straight(deg, goal):
    import numpy as np
    assert np.degrees(sr.straight_goal(np.radians(deg), 180.0)) == pytest.approx(goal)


def _run(monkeypatch, tmp_path, start_deg, *extra):
    import numpy as np
    import aow_sim.hw.dynamixel as dx
    START[0] = np.radians(start_deg)
    made = {}

    def bus(*a, **kw):
        made["bus"] = FakeDxl(*a, **kw)
        return made["bus"]
    monkeypatch.setattr(dx, "DynamixelBus", bus)
    monkeypatch.setattr(sr.time, "sleep", lambda s: None)
    monkeypatch.setattr(sys, "argv", ["steer_response.py", "--freqs", "5", "--seconds", "1.2",
                                      "--fade", "0.3", "--out", str(tmp_path / "s.json"), *extra])
    return made, sr.main


def test_the_steer_is_centred_smoothly_before_the_tones(monkeypatch, tmp_path):
    import json
    import numpy as np
    made, main = _run(monkeypatch, tmp_path, 3.34)
    main()
    b = made["bus"]
    move, tone = b.streams[0], b.streams[1]
    assert np.degrees(move[0]) == pytest.approx(3.34, abs=0.01)     # no step at the start
    assert np.degrees(move[-1]) == pytest.approx(0.0, abs=1e-6)      # ends straight
    assert np.max(np.abs(np.diff(move))) * 250 < 1.0 + 1e-6          # <= 1 rad/s at 250 Hz
    assert abs(np.degrees(tone.mean())) < 0.05                      # the tone is about straight
    meta = json.loads((tmp_path / "s.json").read_text())
    assert meta["centred"]["to_deg"] == pytest.approx(0.0, abs=1e-6)


def test_a_fork_far_off_straight_is_refused_before_torque(monkeypatch, tmp_path):
    made, main = _run(monkeypatch, tmp_path, 60.0)
    with pytest.raises(RuntimeError, match="from straight"):
        main()
    assert not any(on for ids, on in made["bus"].torque_log)


def test_no_center_sweeps_where_the_shaft_is(monkeypatch, tmp_path):
    import numpy as np
    made, main = _run(monkeypatch, tmp_path, 3.34, "--no-center")
    main()
    b = made["bus"]
    assert len(b.streams) == 1
    assert np.degrees(b.streams[0].mean()) == pytest.approx(3.34, abs=0.05)


def test_pitch_vs_steer_reads_a_nose_up_bowl(tmp_path, capsys):
    """A chassis that noses UP by c * steer^2 must print positive, symmetric bins."""
    import argparse
    import numpy as np
    t = np.arange(0, 12, 0.005)
    amp = 30.0
    x = np.radians(amp) * sr.envelope(t, 12, 2) * np.sin(2 * np.pi * 0.25 * t)
    up = np.radians(2e-4 * np.degrees(x) ** 2)            # nose up, rad
    quat = np.column_stack([np.cos(-up / 2), 0 * up, np.sin(-up / 2), 0 * up])   # about +y by -up
    np.savez(tmp_path / "a.npz", t=t + 5.0, quat=quat)
    tone = {"amp_deg": amp, "f": 0.25, "t": t.tolist(), "t_tone": 5.0, "pos": x.tolist()}
    sr.pitch_vs_steer(tmp_path / "a.npz", [tone], argparse.Namespace())
    rows = [l for l in capsys.readouterr().out.splitlines() if "rising" in l or "falling" in l]
    for row in rows:
        vals = [float(v) for v in row.split()[2:] if v != "-"]
        assert vals[0] > 0.1 and vals[-1] > 0.1                 # ~2e-4 * 27.5^2 = 0.15 deg
        assert vals[0] == pytest.approx(vals[-1], rel=0.1)       # symmetric
    assert sr.chassis_pitch_up(quat[:1])[0] == pytest.approx(np.degrees(up[0]))
