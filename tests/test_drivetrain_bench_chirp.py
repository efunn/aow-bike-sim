"""`drivetrain_bench.py chirp --steps / --mix`: the stepped sine and the mixed
common + diff command, and the lock-in that reads them back.

What it guards: each step must be whole cycles (so the joins are continuous
and the lock-in sees no partial cycle), the mixed command must decompose back
into diff at --amp and common at RATIO x that, leading by PHASE, and the
lock-in must return amplitude and phase in the sign convention the printout
labels them with.
"""

import argparse
import math
import sys

import numpy as np
import pytest

sys.path.insert(0, "analysis/servo")
import drivetrain_bench as db  # noqa: E402

pytestmark = pytest.mark.analysis


class _Bus:
    ids = (101, 102)


def _ctx(**kw):
    args = argparse.Namespace(signs=(1.0, -1.0), amp=0.4, f0=4.0, f1=14.0, steps=6,
                              dwell=2.0, mix=None, modes=["diff"], gains=None,
                              rate=500.0, seconds=20.0)
    vars(args).update(kw)
    return db.Ctx(bus=_Bus(), args=args, no_load=11.0, v_limit=11.03)


def _split(g, sa=1.0, sb=-1.0):
    return (sa * g[101] - sb * g[102]) / 2, (sa * g[101] + sb * g[102]) / 2


def test_the_mixed_command_decomposes_into_diff_and_common():
    ctx = _ctx()
    wave = db._mixed(ctx, 2.0, 0.3, 35.0)
    for th in np.linspace(0, 2 * np.pi, 9):
        d, c = _split(wave(th))
        assert d == pytest.approx(2.0 * math.sin(th))
        assert c == pytest.approx(0.6 * math.sin(th + math.radians(35.0)))


def test_steps_are_whole_cycles_log_spaced_and_continuous():
    plan = db.plan_chirp(_ctx(mix=(0.3, 35.0)))
    steps = [s for s in plan.segments if s.info["kind"] == "sine"]
    f = np.array([s.info["freq"] for s in steps])
    assert f == pytest.approx(np.geomspace(4.0, 14.0, 6))
    for s in steps:
        assert s.seconds * s.info["freq"] == pytest.approx(s.info["cycles"])
        assert s.seconds >= 2.0
        start, end = s.command(0.0, None), s.command(s.seconds, None)
        assert start[101] == pytest.approx(end[101], abs=1e-9)
        assert start[102] == pytest.approx(end[102], abs=1e-9)
    assert steps[0].info["mode"] == "mix" and steps[0].info["mix_ratio"] == 0.3


def test_a_mix_that_would_pass_the_velocity_limit_is_refused():
    with pytest.raises(SystemExit):
        db.plan_chirp(_ctx(amp=0.8, mix=(0.5, 0.0)))


def test_lockin_reads_amplitude_and_phase():
    t = np.arange(0, 3, 0.002)
    x = 0.7 * np.cos(2 * np.pi * 9 * t - 0.5) + 0.2 + 0.1 * t
    A = db._lockin(t, x, 9.0)
    assert abs(A) == pytest.approx(0.7, rel=1e-6)
    assert np.angle(A) == pytest.approx(-0.5, abs=1e-6)


class _FakeReader:
    """A 200 Hz AHRS whose pitch rate is a 9 Hz sine, sampled on demand."""

    def __init__(self):
        import time
        self._time, self.t0, self._last, self.stopped = time, time.monotonic(), None, False

    def start(self):
        pass

    def wait_ready(self):
        return 0.0

    def stop(self):
        self.stopped = True

    def latest(self, max_age=0.05):
        from aow_sim.hw.ahrs import AhrsSample
        k = int((self._time.monotonic() - self.t0) / 0.005)
        if self._last is None or self._last[0] != k:
            t = self.t0 + k * 0.005
            wy = 0.5 * math.sin(2 * math.pi * 9.0 * (t - self.t0))
            self._last = (k, AhrsSample(np.array([1.0, 0, 0, 0]), np.array([0.0, wy, 0.0]),
                                        np.array([0, 0, 9.8]), t))
        return self._last[1]


def test_ahrs_log_keeps_every_sample_on_the_perf_counter_clock(tmp_path, capsys):
    import time
    log = db.AhrsLog("fake", tmp_path / "unused.npz", reader=_FakeReader())
    t0 = time.perf_counter()
    log.start()
    time.sleep(1.5)
    log.stop()
    meta = log.save(tmp_path, t0)
    z = np.load(tmp_path / "ahrs.npz")
    assert log.reader.stopped
    assert 190 < meta["rate_hz"] < 210
    assert meta["samples"] > 0.9 * 1.5 * 200          # none (or almost none) missed
    assert z["t"][0] > -0.01 and z["t"][-1] < 1.6       # on the capture's timebase
    assert z["gyro"].shape == (meta["samples"], 3)
    db.after_ahrs(tmp_path)
    out = capsys.readouterr().out
    pitch = next(line for line in out.splitlines() if "pitch" in line)
    assert "at   9." in pitch or "at   8." in pitch
