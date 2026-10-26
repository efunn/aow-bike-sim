"""`tap_test.py`: the guided pluck test, against a fake bus and a fake AHRS.

What it guards: each spot is bracketed by its own two Enters (skip and redo
included), a ring at a known frequency is found as that spot's peak against
the baseline, the servos are locked before the first spot and put back after
the last -- also when the test is interrupted -- and an interrupted test
still saves the spots it finished.
"""

import sys

import numpy as np
import pytest

sys.path.insert(0, "analysis/servo")
import tap_test as tt  # noqa: E402
from aow_sim.hw.control_table import table_by_name  # noqa: E402

pytestmark = pytest.mark.analysis

SPOTS = (("baseline", "x"), ("steerer", "y"), ("rear_axle", "z"))


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        self.t += 5.0
        return self.t


def test_each_spot_is_bracketed_by_its_two_enters_with_skip_and_redo():
    answers = iter(["", "",          # baseline
                    "", "r",         # steerer, redone
                    "", "",          # steerer again
                    "s"])            # rear_axle skipped
    marks = tt.walk(SPOTS, 3, Clock(), ask=lambda _p: next(answers))
    assert [m["name"] for m in marks] == ["baseline", "steerer"]
    assert marks[0]["t0"] == 5 and marks[0]["t1"] == 10
    assert marks[1]["t0"] == 25 and marks[1]["t1"] == 30      # the redo's times


def _gyro(fs=200.0, ring_hz=8.0, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(0, 30, 1 / fs)
    g = 0.05 * rng.standard_normal((len(t), 3))
    for t0 in (12.0, 15.0, 18.0):                    # three plucks in 10-20 s
        s = t >= t0
        g[s, 0] += 20 * np.exp(-(t[s] - t0) * 2 * np.pi * ring_hz * 0.05) \
            * np.sin(2 * np.pi * ring_hz * (t[s] - t0))
    return t, g


def test_a_ring_is_found_at_its_frequency_against_the_baseline():
    t, g = _gyro()
    marks = [{"name": "baseline", "t0": 1.0, "t1": 9.0},
             {"name": "steerer", "t0": 11.0, "t1": 21.0}]
    rows = {(r[0], r[1]): r for r in tt.ring_table(t, g, marks)}
    _, _, rms, f, ratio, mid = rows[("steerer", "roll x")]
    assert abs(f - 8.0) <= 0.5
    assert ratio > 100 and mid > 100
    assert rows[("steerer", "yaw z")][5] < 3          # nothing rang there
    assert np.isnan(rows[("baseline", "roll x")][4])


class FakeBus:
    follows = True

    def __init__(self, port, ids=(), **kw):
        self.ids = tuple(ids)
        self.tables = {i: table_by_name("xc430_w150") for i in ids}
        self.regs = {d: {"Operating Mode": 3, "Velocity P Gain": 400, "Velocity I Gain": 0,
                         "Goal Velocity": 0, "Present Position": 2**32 - 10}
                     for d in self.ids}                  # -10 counts, as the bus returns it
        self.regs[103] = {"Operating Mode": 4, "Position P Gain": 640, "Position I Gain": 0,
                          "Position D Gain": 0, "Present Position": 2060,
                          "Torque Enable": 0, "Bus Watchdog": 0, "Hardware Error Status": 0}
        self.regs[104] = {"Operating Mode": 3, "Position P Gain": 800, "Position I Gain": 0,
                          "Position D Gain": 0, "Goal Current": 910, "Present Position": 2100,
                          "Torque Enable": 0, "Bus Watchdog": 0, "Hardware Error Status": 0}
        self.log = []
        self.follows = FakeBus.follows
        FakeBus.last = self

    def open(self):
        return self

    def discover(self, ids=None):
        self.ids, self.tables = tuple(ids), {i: table_by_name("xc330_t181") for i in ids}
        return dict(self.tables)

    def hardware_errors(self, ids=None):
        return {}

    def read_raw(self, d, k):
        return self.regs[d][k]

    def write_raw(self, d, k, v):
        self.log.append(("write", d, k, v))
        if k.startswith("Goal") and d in (103, 104) and not self.regs[d]["Torque Enable"]:
            return                         # acknowledged, dropped (control_tables/README.md)
        self.regs[d][k] = v
        if d == 103 and k == "Goal Position" and self.follows:
            self.regs[d]["Present Position"] = v + 3      # a held shaft, 0.3 deg off

    def torque(self, on, ids=None):
        ids = tuple(ids if ids is not None else self.ids)
        self.log.append(("torque", ids, on))
        for i in {103, 104} & set(ids):
            self.regs[i]["Torque Enable"] = int(on)

    def prepare(self, ids=None, **kw):
        pass

    def close(self):
        self.log.append(("close",))


class FakeAhrs:
    mount_source = "given"

    def start(self):
        return 0.0

    def stop(self):
        self.stopped = True

    def save(self, out, t0):
        t, g = _gyro()
        np.savez(out / "ahrs.npz", t=t, quat=np.tile([1.0, 0, 0, 0], (len(t), 1)),
                 gyro=np.radians(g), accel=np.zeros((len(t), 3)),
                 qos=np.zeros(len(t), np.int64))
        return {"samples": len(t)}


def _run(tmp_path, answers, seen=None):
    it = iter(answers)

    def ask(p):
        if seen is not None and "[1/" in p:
            seen.append(list(FakeBus.last.log))     # the log as the first spot opens
        a = next(it)
        if a is KeyboardInterrupt:
            raise KeyboardInterrupt
        return a
    return tt.main(["--out", str(tmp_path / "tap"), "--spots", "steerer"],
                   ask=ask, bus_cls=FakeBus, ahrs_log=FakeAhrs())


def test_servos_lock_before_the_first_spot_and_come_back_after(tmp_path):
    seen = []
    assert _run(tmp_path, ["", "", "", "", ""], seen) == 0   # torque prompt, 2 spots x 2
    before = seen[0]
    assert ("torque", (102, 101), True) in before and ("torque", (103,), True) in before
    log = FakeBus.last.log
    after = log[len(before):]
    assert ("torque", (102, 101), False) in after and ("torque", (103,), False) in after
    assert log[-1] == ("close",)
    assert FakeBus.last.regs[101]["Operating Mode"] == 3        # mode put back
    assert FakeBus.last.regs[103]["Position P Gain"] == 640
    assert (tmp_path / "tap" / "tap.json").exists()


def test_the_wings_are_held_where_they_are_as_flown_and_put_back(tmp_path):
    seen = []
    _run(tmp_path, ["", "", "", "", ""], seen)
    w = [e for e in seen[0] if e[0] == "write" and e[1] == 104]
    names = [e[2] for e in w]
    assert names[0] == "Operating Mode" and w[0][3] == 5          # mode first
    assert names.index("Position P Gain") > 0
    t_on = seen[0].index(("torque", (104,), True))
    after = [e[2] for e in seen[0][t_on:] if e[0] == "write" and e[1] == 104]
    assert after[:2] == ["Goal Position", "Goal Current"]           # both after torque on
    assert FakeBus.last.regs[104]["Goal Position"] == 2100
    r = FakeBus.last.regs[104]
    assert (r["Operating Mode"], r["Position P Gain"], r["Goal Current"]) == (3, 800, 910)
    import json
    meta = json.loads((tmp_path / "tap" / "tap.json").read_text())
    assert meta["wing_hold"]["check"]["off_deg"] == 0
    assert set(meta["end_state"]) == {"101", "102", "103", "104"}


def test_an_interrupted_test_restores_and_keeps_the_finished_spots(tmp_path):
    with pytest.raises(KeyboardInterrupt):
        _run(tmp_path, ["", "", "", "", KeyboardInterrupt])
    import json
    meta = json.loads((tmp_path / "tap" / "tap.json").read_text())
    assert [m["name"] for m in meta["marks"]] == ["baseline"]
    assert FakeBus.last.log[-1] == ("close",)
    assert FakeBus.last.regs[102]["Operating Mode"] == 3


def test_a_steer_within_the_limit_carries_on_and_its_check_is_saved(tmp_path, monkeypatch):
    monkeypatch.setattr(FakeBus, "follows", False)   # the shaft stays at 2060, goal 2048
    prompts = []

    def run():
        it = iter(["", "", "", "", "", ""])

        def ask(p):
            prompts.append(p)
            return next(it)
        return tt.main(["--out", str(tmp_path / "t"), "--spots", "steerer"], ask=ask,
                       bus_cls=FakeBus, ahrs_log=FakeAhrs())
    run()                                            # 12 counts = 1.05 deg: no question
    assert not any("NOT holding" in p for p in prompts)
    import json
    chk = json.loads((tmp_path / "t" / "tap.json").read_text())["steer_hold"]["check"]
    assert chk["Torque Enable"] == 1 and abs(chk["off_deg"] - 1.05) < 0.01


def test_steer_check_asks_when_the_shaft_is_far_from_its_goal():
    bus = FakeBus("p", ids=(102, 101))
    bus.regs[103].update({"Torque Enable": 1, "Goal Position": 0, "Present Position": 94})
    prompts = []
    got = tt.held_check(bus, 103, 0, "steer", ask=lambda p: prompts.append(p) or "")
    assert abs(got["off_deg"] - 8.26) < 0.01
    assert len(prompts) == 1 and "NOT holding" in prompts[0]


def test_each_pluck_is_found_and_its_ring_is_the_largest_peak():
    t, g = _gyro(ring_hz=6.2)
    marks = [{"name": "baseline", "t0": 1.0, "t1": 9.0},
             {"name": "rear_axle", "t0": 11.0, "t1": 21.0}]
    n, axes, times = tt.ring_peaks(t, g, marks)["rear_axle"]
    assert n == 3 and np.allclose(times, [12, 15, 18], atol=0.2)
    best = max(axes[0], key=lambda p: p[1])
    assert abs(best[0] - 6.2) <= 0.2 and best[1] == 1.0


def test_the_held_shafts_are_logged_signed_on_the_ahrs_clock(tmp_path):
    _run(tmp_path, ["", "", "", "", ""])
    z = np.load(tmp_path / "tap" / "servos.npz")
    assert list(z["ids"]) == [102, 101, 103, 104] and len(z["t"]) > 0
    assert np.allclose(z["pos_rad"][:, 0], -10 * 2 * np.pi / 4096)
    import json
    meta = json.loads((tmp_path / "tap" / "tap.json").read_text())
    assert meta["servos"]["read_errors"] == 0


def test_rollers_ringing_with_the_gyro_show_at_its_frequency(tmp_path, capsys):
    t = np.arange(0, 30, 1 / 150)
    ring = np.zeros_like(t)
    for t0 in (12.0, 15.0, 18.0):
        s = t >= t0
        ring[s] += np.radians(0.7) * np.exp(-(t[s] - t0) * 2) * np.sin(2 * np.pi * 6.2 * (t[s] - t0))
    pos = np.stack([ring, ring, np.zeros_like(t)], 1)    # 102, 101: diff = (q101 + q102)/2
    np.savez(tmp_path / "servos.npz", t=t, ids=np.array([102, 101, 103]), pos_rad=pos)
    rp = {"rear_axle": (3, [[], [], []], [12.0, 15.0, 18.0])}
    tt.shaft_rings(tmp_path, {}, rp)
    line = [x for x in capsys.readouterr().out.splitlines() if "rear_axle" in x][0]
    assert "6.2(1.00)" in line or "6.1(1.00)" in line or "6.3(1.00)" in line
    pp = float(line.split("p-p")[1].split(",")[0])
    assert 1.0 < pp < 1.5                                # ~2 x 0.7 deg


def test_each_pluck_reports_its_size_and_ring_without_a_filter_edge_artifact():
    t, g = _gyro(ring_hz=6.2)
    g[:, 2] = g[:, 0]                                   # the ring in yaw
    g[(t > 12) & (t < 12.4), 2] += 40                   # a large slow lurch on the first
    marks = [{"name": "baseline", "t0": 1.0, "t1": 9.0},
             {"name": "rear_end", "t0": 11.0, "t1": 21.0}]
    rows = tt.pluck_rows(t, g, tt.ring_peaks(t, g, marks))
    assert len(rows) == 3
    assert all(abs(fq - 6.2) <= 0.3 for _n, _t, _a, fq in rows)
    assert all(a > 10 for _n, _t, a, _f in rows)
