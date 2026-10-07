"""bench/wave_run.py against a fake XL330 in velocity mode.

The wave cams spin the drop rig's arm at set speeds for the AHRS. These pin
what must hold with nothing plugged in: velocity mode, torque on before any
Goal Velocity (one written with torque off is dropped, as on the real
servo), forward only, speeds checked against the cam and the servo, each
spin measured and refused when off, a stalled cam stopped with torque off,
and the
big logs in archive/.
"""

import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bench"))
import force_drop as fd  # noqa: E402
import wave_run as wr  # noqa: E402

pytestmark = pytest.mark.pure


class VelocityBus:
    """One servo turning at its Goal Velocity x `slow`, in real time. A Goal
    Velocity written with torque off is acknowledged and dropped (README)."""

    def __init__(self, pos=1000, slow=1.0, latched=0):
        self.reg = {"Torque Enable": 0, "Operating Mode": 5, "Velocity Limit": 445,
                    "Velocity P Gain": 180, "Velocity I Gain": 1600,
                    "Goal Velocity": 0}
        self.p0, self.t0, self.v = pos, time.perf_counter(), 0.0
        self.slow, self.latched, self.writes = slow, latched, []

    def _pos(self):
        return round(self.p0 + self.v * (time.perf_counter() - self.t0))

    def hardware_errors(self, ids=None):
        return {1: self.latched} if self.latched else {}

    def prepare(self, ids=None):
        self.reg["Torque Enable"] = 0

    def read_raw(self, i, name):
        return self._pos() & 0xFFFFFFFF if name == "Present Position" else self.reg[name]

    def write_raw(self, i, name, raw):
        self.writes.append((name, raw, self.reg["Torque Enable"]))
        if name == "Goal Velocity" and not self.reg["Torque Enable"]:
            return                                    # dropped
        self.reg[name] = raw
        if name == "Goal Velocity" or (name == "Torque Enable" and not raw):
            self.p0, self.t0 = self._pos(), time.perf_counter()
            on = self.reg["Torque Enable"]
            self.v = (fd.signed(self.reg["Goal Velocity"], 4) * wr.RPM_PER_UNIT / 60 * fd.TICKS
                      * self.slow) if on else 0.0


def servo(direction=-1, **kw):
    bus = VelocityBus(**kw)
    return wr.WaveServo(bus, 1, direction, []), bus


def test_the_plan_holds_around_every_spin():
    steps = wr.plan([0.5, 1.0], 15, 5, 0.5)
    assert [k for k, _, _ in steps] == ["hold", "spin", "hold", "spin", "hold"]
    assert [v for _, v, _ in steps] == [0.0, 0.5, 0.0, 1.0, 0.0]


def test_speeds_past_the_cam_or_the_servo_are_refused():
    tones = [(4, 2.4, 0.0)]                          # separation 2.57 rev/s at 5 m/s^2
    assert wr.check_speeds([0.5, 1.5], tones, 5.0, 1.7) == pytest.approx(2.57, abs=0.01)
    for bad in ([2.4], [1.8], [0.0]):                # past 0.9 x 2.57; past the limit; zero
        with pytest.raises(SystemExit):
            wr.check_speeds(bad, tones, 5.0, 1.7)


def test_velocity_mode_and_torque_on_before_any_goal_velocity():
    s, bus = servo()
    assert bus.reg["Operating Mode"] == wr.MODE_VELOCITY
    s.start()
    goals = [(raw, on) for name, raw, on in bus.writes if name == "Goal Velocity"]
    assert goals and all(on for _, on in goals)      # none dropped
    assert s.limit_rev_s == pytest.approx(445 * 0.229 / 60)


def test_a_latched_error_is_refused_before_anything_moves():
    with pytest.raises(RuntimeError, match="latched"):
        servo(latched=0x20)


def test_a_run_spins_forward_at_the_set_speed_and_records_each_step():
    s, bus = servo(direction=-1)
    s.start()
    segs = wr.run(s, wr.plan([0.5, 1.0], 0.4, 0.05, 0.04), 0.04)
    assert [r["kind"] for r in segs] == ["hold", "spin", "hold", "spin", "hold"]
    for r in segs:
        if r["kind"] == "spin":
            assert r["rev_s_measured"] == pytest.approx(r["rev_s_set"], rel=wr.SPEED_TOL)
    vel = [fd.signed(raw, 4) for name, raw, _ in bus.writes if name == "Goal Velocity"]
    assert all(v <= 0 for v in vel) and min(vel) < 0           # dir -1: never positive
    assert fd.signed(bus.reg["Goal Velocity"], 4) == 0           # ends still
    s.stop()
    assert bus.reg["Torque Enable"] == 0


def test_a_spin_that_falls_short_of_its_speed_is_refused():
    s, bus = servo(slow=0.8)
    s.start()
    with pytest.raises(RuntimeError, match="spun at"):
        wr.run(s, wr.plan([1.0], 0.4, 0.02, 0.04), 0.04)


def test_a_stalled_cam_stops_it_with_torque_off():
    s, bus = servo(slow=0.0)                         # the cam does not turn
    s.start()
    t0 = time.perf_counter()
    with pytest.raises(RuntimeError, match="stalled"):
        s.spin(1.0, 5.0, 0.04)
    assert time.perf_counter() - t0 < 1.5            # within a stall window, not the spin
    assert bus.reg["Torque Enable"] == 0


def test_the_big_logs_go_to_the_archive_and_never_reuse_a_name(tmp_path, monkeypatch):
    monkeypatch.setattr(wr, "LOGS", tmp_path)
    monkeypatch.setattr(wr, "ARCHIVE", tmp_path / "archive")
    (tmp_path / "archive").mkdir()
    rec, big = wr.stem_pair("wave_x")
    assert rec == tmp_path / "wave_x" and big == tmp_path / "archive" / "wave_x"
    (tmp_path / "archive" / "wave_x_ahrs.csv").touch()           # only the archive has it
    assert wr.stem_pair("wave_x")[0].name == "wave_x-2"
