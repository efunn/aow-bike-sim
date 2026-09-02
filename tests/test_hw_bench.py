"""Bus tests that need real servos. Skipped unless AOW_DXL_PORT is set.

THE SPLIT, because it is the question this file exists to answer: everything
else in the suite runs with NO hardware. `test_hw_dynamixel.py` is marked
`pure` and checks the register maps, the unit decoding and the indirect layout
from the vendored control tables alone — `IndirectMap.address_bytes()` IS the
SyncWrite payload, so the bytes that go on the wire are asserted without a wire.

What genuinely cannot be faked, and is therefore here:

  * that a servo answers at all, and reports the Model Number we decode against
  * that the indirect map STICKS — a SyncWrite is a broadcast and returns no
    status packet, so the only proof is reading it back
  * the achievable frame rate, which is a property of the bus, the FTDI latency
    timer and the servos, not of this code

Run:  AOW_DXL_PORT=/dev/cu.usbserial-FTB8HNE3 pytest -m hardware -v

Read-only apart from the indirect map itself: no torque is enabled and no goal
register is written, so this is safe to run on an assembled bike.
"""

import os

import numpy as np
import pytest

from aow_sim.hw.control_table import MODEL_NUMBERS
from aow_sim.hw.dynamixel import DynamixelBus, IndirectMap

pytestmark = pytest.mark.hardware

PORT = os.environ.get("AOW_DXL_PORT")
IDS = tuple(int(i) for i in os.environ.get("AOW_DXL_IDS", "101,102,103,104").split(","))
BAUD = int(os.environ.get("AOW_DXL_BAUD", "3000000"))

pytestmark = [pytest.mark.hardware,
              pytest.mark.skipif(not PORT, reason="set AOW_DXL_PORT to run")]


@pytest.fixture(scope="module")
def bus():
    with DynamixelBus(PORT, baud=BAUD, ids=IDS) as b:
        yield b


@pytest.fixture(scope="module")
def imap(bus):
    m = (IndirectMap(bus.tables)
         .read("Realtime Tick").read("Present Position")
         .read("Present Velocity").read("Present PWM")
         .read({i: ("Present Current" if "Present Current" in bus.tables[i]
                    else "Present Load") for i in bus.ids}, label="torque_sense"))
    bus.apply_map(m)
    return m


def test_every_id_answers_and_is_a_known_model(bus):
    assert set(bus.tables) == set(IDS)
    for i, ct in bus.tables.items():
        assert ct.model_number in MODEL_NUMBERS, f"id {i} is a {ct.model_number}"


def test_torque_is_off_before_anything_else(bus):
    """This file must be safe to run on an assembled bike."""
    for i in bus.ids:
        assert bus.read_raw(i, "Torque Enable") == 0


def test_the_indirect_map_actually_sticks(bus, imap):
    """The SyncWrite is a broadcast with no status packet, so read it back."""
    imap.verify(bus._port, bus._packet)


def test_a_frame_decodes_per_model(bus, imap):
    """The point of per-model tables: slot 126 is amps on one part and a
    fraction of max torque on the other, from the same indirect offset."""
    row = bus.read_frame()
    assert set(row) == set(bus.ids)
    for i, v in row.items():
        assert set(v) == set(imap.read_labels)
        assert 0 <= v["Realtime Tick"] < 32768
        assert abs(v["Present PWM"]) <= 1.0
        assert abs(v["Present Position"]) < 300 * 2 * np.pi
        unit = imap.register(i, "torque_sense").unit_name
        assert unit in ("A", "frac_max_torque")


def test_the_tick_advances_and_paces_the_capture(bus, imap):
    """Timing truth is the servo's own clock, so check it against the request."""
    rate = 200.0
    rows = bus.capture(seconds=1.0, rate_hz=rate, warn_overrun=False)
    assert len(rows) > 0.8 * rate, f"only {len(rows)} frames in 1 s at {rate} Hz"
    tick = np.array([r["servos"][bus.ids[0]]["Realtime Tick"] for r in rows])
    d = np.diff(tick) % 32768
    d = d[(d > 0) & (d < 100)]
    assert len(d) > 0.5 * len(rows), "Realtime Tick is not advancing"
    assert abs(np.median(d) - 1000.0 / rate) < 2.0, (
        f"median tick delta {np.median(d):.1f} ms against a "
        f"{1000.0 / rate:.1f} ms request")


def test_capture_refuses_without_a_time_base(bus):
    """capture() has no usable time base if Realtime Tick is not mapped, and
    says so up front rather than in the analysis."""
    bus.apply_map(IndirectMap(bus.tables).read("Present Position"))
    with pytest.raises(RuntimeError, match="Realtime Tick"):
        bus.capture(seconds=0.1, rate_hz=100.0)



def test_every_servo_clears_the_firmware_floor(bus):
    """`DynamixelBus.discover` refuses a servo below MIN_FIRMWARE, so reaching
    this fixture at all is the assertion. Stated as a test so the reason is
    findable: XC330-T181 firmware 50 echoes back every Indirect Address write
    and then returns an all-zero Indirect Data window, with no error anywhere.
    """
    from aow_sim.hw.dynamixel import MIN_FIRMWARE
    for i, ct in bus.tables.items():
        floor = MIN_FIRMWARE.get(ct.name)
        if floor is not None:
            assert bus.read_raw(i, "Firmware Version") >= floor
