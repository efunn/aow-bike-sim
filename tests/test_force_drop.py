"""bench/force_drop.py's cam logic, against a fake bus: no servo, no sensor.

The drop rig's XL330 turns a snail cam that must only ever move FORWARD (a
backward move drives the follower into a step's undercut face), lift onto the
right top flat, and label each drop by the step that released it. These pin
that, and the startup checks, without hardware.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bench"))
import force_drop as fd  # noqa: E402

pytestmark = pytest.mark.pure

SEG = fd.TICKS // 4          # four drops per turn
DEG = fd.TICKS / 360.0


class FakeBus:
    """One servo whose shaft jumps straight to each goal, in multi-turn ticks."""

    def __init__(self, pos, dxl_id=1, latched=0):
        self.id, self.pos, self.latched = dxl_id, pos, latched
        self.reg = {"Current Limit": 1750, "Torque Enable": 0,
                    "Position P Gain": 800, "Position I Gain": 0, "Position D Gain": 0}
        self.goals, self.writes = [], []

    def hardware_errors(self, ids=None):
        return {self.id: self.latched} if self.latched else {}

    def prepare(self, ids=None):
        self.reg["Torque Enable"] = 0

    def read_raw(self, i, name):
        if name == "Present Position":
            return self.pos & 0xFFFFFFFF
        if name == "Hardware Error Status":
            return self.latched
        return self.reg[name]

    def write_raw(self, i, name, raw):
        self.writes.append((name, raw))
        if name == "Goal Position":
            goal = fd.signed(raw, 4)
            self.goals.append(goal)
            if self.reg["Torque Enable"]:
                self.pos = goal
        elif name == "Operating Mode":
            self.reg[name] = raw
            self.reg["Position P Gain"] = 640        # the mode write resets the gains
        else:
            self.reg[name] = raw


def cam(pos, direction=-1, index=1500, **kw):
    bus = FakeBus(pos, **kw)
    return fd.Cam(bus, 1, direction, index, 4, park_deg=12, margin_deg=30, ma=300,
                  gains={"Position P Gain": 400, "Position D Gain": 0}), bus


def phase(c, pos):
    """Forward distance from step 0, within one turn."""
    return ((pos - c.step(0)) * c.dir) % fd.TICKS


def assert_forward(bus, direction):
    d = np.diff(np.array(bus.goals) * direction)
    assert (d >= -fd.POS_TOL).all(), f"a backward goal: {bus.goals}"


@pytest.mark.parametrize("direction", (-1, 1))
def test_homing_from_a_ramp_goes_forward_to_the_next_top_flat(direction):
    """Mid-ramp of segment 2: the next step is 2, and nothing is crossed."""
    start = 1500 + direction * (SEG + 40 * DEG)          # 40 deg into step 1's segment
    c, bus = cam(round(start), direction)
    crossed = c.home()
    assert not crossed and c.k == 2
    assert phase(c, bus.pos) == pytest.approx(2 * SEG - 30 * DEG, abs=1)
    assert bus.reg["Torque Enable"] == 1
    assert_forward(bus, direction)


@pytest.mark.parametrize("direction", (-1, 1))
def test_homing_between_a_top_flat_and_its_step_passes_the_step(direction):
    """Within the run-up of step 1: going forward to a top flat crosses step 1
    (an unrecorded drop) and lands on step 2's."""
    start = 1500 + direction * (SEG - 10 * DEG)
    c, bus = cam(round(start), direction)
    assert c.home() is True and c.k == 2
    assert_forward(bus, direction)


def test_each_fire_lifts_holds_and_drops_one_step_forward():
    c, bus = cam(1500 - 5)                                # just past step 0, parked
    c.home()
    ks = [c.fire(hold_s=0.0)[0] for _ in range(6)]
    assert ks == [1, 2, 3, 4, 5, 6]                       # labels: heights[k % 4]
    assert phase(c, bus.pos) == pytest.approx((6 * SEG + 12 * DEG) % fd.TICKS, abs=1)
    assert_forward(bus, -1)
    # every move carries its Goal Current, written just before its Goal
    # Position; the one exception is home()'s hold-where-it-stands, torque off
    names = [n for n, _ in bus.writes]
    before = [names[i - 1] for i, n in enumerate(names) if n == "Goal Position"]
    assert before[0] != "Goal Current"
    assert before[1:] == ["Goal Current"] * (1 + 2 * 6)


def test_stop_parks_forward_and_turns_torque_off():
    """From a top flat, stop goes ON through the step to the next dwell (never
    back down the ramp), then a confirmed Torque Enable 0."""
    c, bus = cam(1500 + SEG + round(40 * DEG))            # mid-ramp, dir -1
    c.home()
    top = bus.pos
    c.stop()
    assert (bus.pos - top) * -1 > 0                       # moved forward (dir -1)
    assert phase(c, bus.pos) == pytest.approx(c.k * SEG % fd.TICKS + 12 * DEG, abs=1)
    assert bus.writes[-1] == ("Torque Enable", 0)
    assert_forward(bus, -1)


def test_stop_when_parked_does_not_move():
    c, bus = cam(1500 - 5)
    c.home()
    c.fire(hold_s=0.0)
    parked, n_goals = bus.pos, len(bus.goals)
    c.stop()
    assert bus.pos == parked and bus.writes[-1] == ("Torque Enable", 0)
    assert all(g == parked for g in bus.goals[n_goals:])


def test_a_latched_error_refuses_with_its_name():
    with pytest.raises(RuntimeError, match="overload"):
        cam(1500, latched=32)


def test_the_mode_is_written_then_the_gains_and_read_back():
    """Writing Operating Mode resets the gains, so they go AFTER it."""
    _, bus = cam(1500)
    names = [n for n, _ in bus.writes]
    assert names.index("Operating Mode") < names.index("Position P Gain")
    assert bus.reg["Position P Gain"] == 400


def test_a_gain_that_does_not_stick_raises():
    class Stubborn(FakeBus):
        def write_raw(self, i, name, raw):
            if name != "Position P Gain":
                super().write_raw(i, name, raw)
    with pytest.raises(RuntimeError, match="did not stick"):
        fd.Cam(Stubborn(1500), 1, -1, 1500, 4, 12, 30, 300, gains={"Position P Gain": 400})


def test_the_edge_speed_is_a_fit_over_the_window():
    rng = np.random.default_rng(0)
    t = np.cumsum(rng.uniform(0.0005, 0.0025, 200))
    ticks = np.round(400.0 * DEG * t)                     # 400 deg/s
    v, n = fd.edge_speed(list(zip(t, ticks)), at_tick=ticks[100])
    assert v == pytest.approx(400.0, rel=0.02) and n >= 10
    assert math.isnan(fd.edge_speed(list(zip(t[:3], ticks[:3])), ticks[1])[0])


def test_a_dry_run_fires_every_drop_with_no_sensor(capsys):
    from types import SimpleNamespace
    c, bus = cam(1500 - 5)
    c.home()
    args = SimpleNamespace(hold_s=0.0, settle_s=0.0, corner_mm=0.45, r_rest_mm=13.0)
    fd.dry_run(args, c, [0.5, 1.0, 1.5, 2.0], [(h, 1) for h in (0.5, 1.0, 1.5, 2.0)])
    assert c.k == 5                                       # four drops, steps 1-4
    assert capsys.readouterr().out.count("cycle") == 4
    assert_forward(bus, -1)


def test_a_cam_run_fires_exactly_the_count_and_records_every_miss(monkeypatch, tmp_path):
    """No impact on any drop: still heights x repeats fires, each a no_impact
    row with its cam columns and its trace -- a miss is a record, never a
    re-fire."""
    import csv
    import threading
    from types import SimpleNamespace
    monkeypatch.setattr(fd, "keys", lambda cmds: None)
    monkeypatch.setattr(fd, "NO_IMPACT_S", 0.05)
    monkeypatch.setattr(fd, "MISS_PRE_S", 0.05)
    c, bus = cam(1500 - 5)
    c.home()
    k0 = c.k
    import time

    class Stream:                     # the sensor's clock runs; nothing ever hits
        lock = threading.Lock()

        @property
        def buf(self):
            now = time.perf_counter()
            return [(now - 0.002 * i, [0, 0, 0, 0]) for i in range(250, -1, -1)]   # 0.5 s
    stream = Stream()
    args = SimpleNamespace(hold_s=0.0, settle_s=0.0, corner_mm=0.45, r_rest_mm=13.0,
                           wheel="tap", repeats=2)
    heights = [0.5, 1.0, 1.5, 2.0]
    steps = [(h, r) for r in (1, 2) for h in heights]
    fd.run(args, stream, c, heights, steps, np.ones(4), tmp_path / "d")
    assert c.k - k0 == 8
    rows = list(csv.DictReader(open(tmp_path / "d_summary.csv")))
    assert [r["outcome"] for r in rows] == ["no_impact"] * 8
    assert [int(r["drop"]) for r in rows] == list(range(1, 9))
    assert [int(r["cam_step"]) for r in rows] == list(range(k0, k0 + 8))
    assert {r["t_zero"] for r in rows} == {"cam_parked"}       # no impact to time from
    with open(tmp_path / "d.csv") as fh:                   # counts only: any calibration later
        assert fh.readline().strip() == "drop,t_s,X_counts,Y_counts,Z_counts,Rz_counts"
    traced = {int(r["drop"]) for r in csv.DictReader(open(tmp_path / "d.csv"))}
    assert traced == set(range(1, 9))                      # each miss keeps its window
    assert all(float(r["pre_drop_n"]) == 0.0 for r in rows)  # unloaded before each drop


def test_a_load_that_never_lifts_is_still_loaded_not_a_miss(monkeypatch, tmp_path):
    """Loaded from just after the zero onwards -- a wheel the cam never lifts
    clear: every drop is still_loaded, with what the sensor read before it."""
    import csv
    import threading
    import time
    from types import SimpleNamespace
    monkeypatch.setattr(fd, "keys", lambda cmds: None)
    monkeypatch.setattr(fd, "NO_IMPACT_S", 0.05)
    monkeypatch.setattr(fd, "MISS_PRE_S", 0.05)
    t_load = time.perf_counter() + 0.05

    class Stream:
        lock = threading.Lock()

        @property
        def buf(self):
            now = time.perf_counter()
            ts = [now - 0.002 * i for i in range(250, -1, -1)]
            return [(t, [0, 0, 5 if t > t_load else 0, 0]) for t in ts]
    c, bus = cam(1500 - 5)
    c.home()
    args = SimpleNamespace(hold_s=0.3, settle_s=0.0, corner_mm=0.45, r_rest_mm=13.0,
                           wheel="tap", repeats=1)
    heights = [0.5, 1.0, 1.5, 2.0]
    fd.run(args, Stream(), c, heights, [(h, 1) for h in heights], np.ones(4), tmp_path / "d")
    rows = list(csv.DictReader(open(tmp_path / "d_summary.csv")))
    assert [r["outcome"] for r in rows] == ["still_loaded"] * 4
    assert all(float(r["pre_drop_n"]) == pytest.approx(5.0) for r in rows)


def _synthetic_drop(scale, zero=(2020, 1645, 1690, 1670)):
    """One impact on sensor 3 from the raw counts' side: an 8 ms contact,
    a 20 ms flight, a 5 ms second contact, then resting at 0.84 N."""
    t = np.arange(-0.2, 1.4, 1e-4)
    f = np.zeros_like(t)
    for t0, dur, peak in ((0.0, 0.008, 5.0), (0.028, 0.005, 1.5)):
        on = (t >= t0) & (t < t0 + dur)
        f[on] = peak * np.sin(np.pi * (t[on] - t0) / dur)
    f[t >= 0.033] = 0.84
    counts = np.tile(np.array(zero, float), (len(t), 1))
    counts[:, 2] += np.round(f / scale[2])
    return t, counts


def test_reanalyse_reproduces_the_summary_and_follows_a_new_scale(tmp_path):
    import csv
    scale = [0.003043, 0.003040, 0.002797, 0.003105]
    t, counts = _synthetic_drop(scale)
    cols, _, _ = fd.measure(t, counts, 2.0, None, scale)
    assert cols["sensor"] == 3 and cols["peak_n"] == pytest.approx(5.0, abs=0.01)
    stem = tmp_path / "drops_x"
    with open(f"{stem}.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["drop", "t_s"] + [f"{c}_counts" for c in fd.CHANNELS])
        w.writerows([1, round(ti, 6), *map(int, c)] for ti, c in zip(t, counts))
    row = dict(drop=1, wheel="front", height_mm=2.0, outcome="ok", t_zero="impact", cam_step=7, **cols)
    with open(f"{stem}_summary.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(row))
        w.writeheader()
        w.writerow(row)
    same = list(csv.DictReader(open(fd.reanalyse(f"{stem}.csv", scale))))[0]
    assert same == list(csv.DictReader(open(f"{stem}_summary.csv")))[0]
    new = list(csv.DictReader(open(fd.reanalyse(f"{stem}_summary.csv", [s * 1.1 for s in scale]))))[0]
    assert float(new["peak_n"]) == pytest.approx(1.1 * cols["peak_n"], rel=1e-3)
    assert float(new["mass_g"]) == pytest.approx(1.1 * cols["mass_g"], rel=1e-3)
    assert new["cam_step"] == "7" and new["wheel"] == "front"     # labels kept as recorded
