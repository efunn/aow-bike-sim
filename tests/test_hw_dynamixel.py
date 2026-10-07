"""The generic X-series layer (`hw/dynamixel.py`) and the per-model tables:
tick and position arithmetic, register decoding, `IndirectMap` layout.

No hardware and no dynamixel_sdk: `IndirectMap.address_bytes` IS the setup
SyncWrite's payload, so the layout is checked byte for byte without a bus.
The bike's use of it is `test_hw_bike_bus.py`.
"""

import numpy as np
import pytest

from aow_sim.hw.control_table import MODEL_NUMBERS, table_by_name, table_for
from aow_sim.hw.dynamixel import (INDIRECT_DATA_1, N_INDIRECT, POS_WRAP,
                                  TICK_WRAP, VEL_LSB_RAD_S, IndirectMap,
                                  instruction_failed, pos_delta, signed,
                                  tick_delta_ms)

# Register maps, unit conversions and tick math; no bike model.
# See `pytest --markers` for what each one means.
pytestmark = pytest.mark.pure


def test_signed_decoding():
    """Position and velocity are two's complement inside unsigned fields."""
    assert signed(0, 4) == 0
    assert signed(1000, 4) == 1000
    assert signed(0xFFFFFFFF, 4) == -1
    assert signed(0xFFFFFFFF - 999, 4) == -1000
    assert signed(0x7FFFFFFF, 4) == 2 ** 31 - 1
    assert signed(0x80000000, 4) == -2 ** 31
    assert signed(0xFFFF, 2) == -1


def test_velocity_lsb_matches_datasheet():
    """0.229 rev/min per LSB, both models."""
    one_rev_per_s = 2 * np.pi
    assert np.isclose(VEL_LSB_RAD_S, 0.229 * one_rev_per_s / 60.0)
    # 100 raw units -> 22.9 rev/min
    assert np.isclose(100 * VEL_LSB_RAD_S * 60 / one_rev_per_s, 22.9)


def test_tick_delta_handles_the_32768_wrap():
    """Realtime Tick is 0..32767 ms and wraps every ~32.8 s. A naive
    subtraction would hand the estimator a -32.7 s dt once per wrap."""
    assert tick_delta_ms(1100, 1000) == 100
    assert tick_delta_ms(5, TICK_WRAP - 5) == 10        # across the wrap
    assert tick_delta_ms(0, TICK_WRAP - 1) == 1
    assert tick_delta_ms(1000, 1000) == 0
    for prev in (0, 17, 32000, TICK_WRAP - 1):
        for step in (1, 10, 100):
            assert tick_delta_ms((prev + step) % TICK_WRAP, prev) == step


def test_pos_delta_unwraps_the_single_turn_rollover():
    """The hubs run in Velocity Control Mode, where Present Position is
    0..4095 over ONE rotation, so a turning wheel rolls over about once a
    second at speed. Plain subtraction reads that as a full turn backwards."""
    assert pos_delta(10, 5) == 5              # ordinary forward
    assert pos_delta(5, 10) == -5             # ordinary reverse
    assert pos_delta(0, 4095) == 1            # forward THROUGH the rollover
    assert pos_delta(4095, 0) == -1           # reverse through it
    assert pos_delta(100, 4000) == 196
    assert pos_delta(4000, 100) == -196
    # What the bug looked like: unwrapped, one rollover is a whole revolution
    # of phantom motion on a single sample.
    assert 4095 - 0 == 4095 and pos_delta(4095, 0) == -1


def test_pos_delta_is_exact_over_a_full_sweep():
    """Integrating the unwrapped deltas must reproduce the true travel, for
    both directions and across many rollovers."""
    for step in (1, 7, 37, -1, -13):
        pos, total = 0, 0
        for _ in range(600):
            nxt = (pos + step) % POS_WRAP
            total += pos_delta(nxt, pos)
            pos = nxt
        assert total == step * 600, f"step {step}: {total} != {step * 600}"


def test_half_a_turn_is_the_ambiguous_case_and_is_far_from_operation():
    """Exactly POS_WRAP/2 is genuinely undecidable -- +2048 and -2048 are the
    same point -- so the sign there is a convention, not a result. This is the
    limit `assert_alias_margin` keeps the bike 40x away from; the test pins
    that the boundary is handled without raising, not which way it falls."""
    assert abs(pos_delta(2048, 0)) == POS_WRAP // 2
    assert pos_delta(2047, 0) == 2047         # just inside: unambiguous
    assert pos_delta(-2047 % POS_WRAP, 0) == -2047


def test_address_126_diverges_between_the_models():
    """Same address, same width, different meaning and different units."""
    a, b = table_by_name("xc430_w150"), table_by_name("xc330_t181")
    assert a["Present Load"].address == b["Present Current"].address == 126
    assert a["Present Load"].size == b["Present Current"].size == 2
    assert a["Present Load"].unit_name == "frac_max_torque"
    assert b["Present Current"].unit_name == "A"
    # The XC430 genuinely has no current registers at all.
    assert "Present Current" not in a and "Current Limit" not in a
    assert "Current Limit" in b


def test_model_numbers_match_the_hardware_probe():
    """Read off the bus on 2026-09-01, not copied from a datasheet.

    101/102 -> 1070, 103/104 -> 1210, and 150/151 -> 1200 (XL330-M288-T, a
    bench part rather than a bike one: it is the actuator Rhoban/bam identifies,
    so it is where our numbers and theirs can be compared directly).
    """
    assert MODEL_NUMBERS == {1070: "xc430_w150", 1200: "xl330_m288",
                             1210: "xc330_t181"}
    for number, stem in MODEL_NUMBERS.items():
        assert table_for(number).name == stem


def test_unknown_model_number_names_the_fix():
    with pytest.raises(KeyError, match="unknown Model Number"):
        table_for(9999)


def test_signed_registers_decode_negative():
    ct = table_by_name("xc330_t181")
    assert ct.decode("Present Current", 0xFFFF) == pytest.approx(-0.001)
    assert ct["Present Position"].decode(0xFFFFFFFF) < 0


def test_velocity_limit_shares_the_velocity_lsb():
    """It is a velocity register but upstream's [unit info] does not cover it.

    Left as raw counts it decodes to ~460 where the truth is ~11 rad/s, which
    silently defeats any amplitude clamp written against it — analysis/
    servo_reversal.py had exactly that bug before torque was ever enabled.
    """
    for stem in ("xc430_w150", "xc330_t181"):
        ct = table_by_name(stem)
        assert ct["Velocity Limit"].unit == ct["Present Velocity"].unit
        assert ct["Velocity Limit"].unit_name == "rad/s"


def test_trajectory_registers_share_the_goal_lsb():
    """Same gap, second time: Velocity Trajectory decoded as raw counts, and the
    first drivetrain step analysis found no command arrival on any non-zero
    step because it compared counts with rad/s. Signed, like the goals."""
    for stem in ("xc430_w150", "xc330_t181", "xl330_m288"):
        ct = table_by_name(stem)
        for traj, goal in (("Velocity Trajectory", "Goal Velocity"),
                           ("Position Trajectory", "Goal Position")):
            assert ct[traj].unit == pytest.approx(ct[goal].unit), (stem, traj)
            assert ct[traj].signed, (stem, traj)
        assert ct.decode("Velocity Trajectory", (-100) & 0xFFFFFFFF) < 0


def test_goal_current_cannot_be_in_the_shared_block():
    """The reason `set_righting_current` is a separate write and not another
    indirect entry: the XC430 does not have the register at all. 102 is Goal
    Current on the XC330 and does not exist on the XC430, which has Goal
    PWM(100) and Present Load(126) where the XC330 has current."""
    assert "Goal Current" in table_by_name("xc330_t181")
    assert "Goal Current" not in table_by_name("xc430_w150")


def _tables():
    return {101: table_by_name("xc430_w150"), 103: table_by_name("xc330_t181")}


def test_one_slot_can_be_a_different_register_per_model():
    """The reason the map takes a dict: 126 is Load on one and Current on the
    other, so the slot is byte-identical and the DECODE is not."""
    m = (IndirectMap(_tables()).read("Realtime Tick")
         .read({101: "Present Load", 103: "Present Current"}, label="torque"))
    assert m.register(101, "torque").name == "Present Load"
    assert m.register(103, "torque").name == "Present Current"
    # Same offset, same source address, on both servos.
    assert m.address_bytes(101)[4:] == m.address_bytes(103)[4:] == [126, 0, 127, 0]
    assert m.read_offsets["torque"] == 2


def test_mixed_widths_are_refused():
    """A slot whose width differs across servos would put different fields at
    the same offset, and every later decode would be quietly wrong."""
    with pytest.raises(ValueError, match="differ in width"):
        IndirectMap(_tables()).read({101: "Present PWM",
                                     103: "Present Position"})


def test_a_spec_missing_a_servo_is_refused():
    with pytest.raises(KeyError, match="does not name a register"):
        IndirectMap(_tables()).read({101: "Present Load"})


def test_block_budget_is_enforced_and_names_the_overflow():
    """28 bytes for reads and writes together — the XC330 has no second block,
    so a mixed bus cannot spill into 578/634."""
    m = IndirectMap(_tables())
    for name in ("Present Position", "Present Velocity", "Velocity Trajectory",
                 "Position Trajectory", "Goal Position", "Goal Velocity",
                 "Profile Velocity", "Profile Acceleration"):
        m.read(name)
    assert m.n_bytes > N_INDIRECT
    with pytest.raises(RuntimeError, match="second block"):
        m.apply(None, None)


def test_reads_then_writes_are_each_one_contiguous_span():
    m = (IndirectMap(_tables()).read("Realtime Tick").read("Present Position")
         .write({101: "Goal Velocity", 103: "Goal Position"}, label="goal"))
    assert m.read_addr == INDIRECT_DATA_1
    assert m.read_len == 6 and m.write_len == 4
    assert m.write_addr == INDIRECT_DATA_1 + m.read_len
    assert m.n_bytes == 10


def test_the_alert_bit_is_not_a_failed_instruction():
    """err=128 is 'this servo has a latched hardware error', on every reply,
    and the instruction WAS carried out."""
    assert not instruction_failed(0, 0x80)          # alert only: done
    assert instruction_failed(0, 0x80 | 0x04)       # alert AND a real error
    assert instruction_failed(0, 0x07)              # access error, no alert
    assert instruction_failed(-1000, 0)             # port busy
    assert not instruction_failed(0, 0)


def test_the_latency_check_follows_a_by_id_symlink(tmp_path, monkeypatch):
    """bike_params names the U2D2 by /dev/serial/by-id, a symlink to ttyUSB0:
    the check must read ttyUSB0's timer, not pass for want of a by-id one."""
    import pytest

    from aow_sim.hw import dynamixel as dx
    sysfs = tmp_path / "sys"
    (sysfs / "ttyUSB0").mkdir(parents=True)
    (sysfs / "ttyUSB0" / "latency_timer").write_text("16\n")
    (tmp_path / "ttyUSB0").touch()
    by_id = tmp_path / "usb-FTDI_USB__-__Serial_Converter_FTB8HNE3-if00-port0"
    by_id.symlink_to(tmp_path / "ttyUSB0")
    monkeypatch.setattr(dx, "SYSFS_USB_SERIAL", sysfs)
    with pytest.raises(RuntimeError, match="16 ms"):
        dx.assert_low_latency(str(by_id))
    (sysfs / "ttyUSB0" / "latency_timer").write_text("1\n")
    dx.assert_low_latency(str(by_id))
