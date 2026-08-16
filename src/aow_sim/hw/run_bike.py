"""The onboard process: three threads, four failsafes, one control loop.

    python -m aow_sim.hw.run_bike --bundle deploy/bundle.npz

Structure (docs/plans/untethered-setup.md):
  control thread  SCHED_FIFO, 100 Hz — SyncRead -> DriveController -> BulkWrite
  ahrs thread     200 Hz             — UART -> latest-value slot
  link thread      50 Hz             — UDP command in, telemetry out

100 Hz, not the simulator's 200: `DriveController._gen_every` already retimes
the 50 Hz general policy against whatever the controller rate is, so 100 Hz
ticks driving a 50 Hz policy is the designed path and needs no code change.
Raise it only once measured jitter says you can.

WHY THE COMMAND LINK IS WiFi: the operator sends a whole command struct —
velocity vector, heading, controller mode, move triggers, re-zero — not two
analog axes. That is the surface run_drive.py's teleop already exposes, and it
is why an RC receiver was rejected. The cost is that WiFi has no failsafe of
its own, so all four below are mandatory, not optional.

FAILSAFES
  1. command age  >150 ms -> zero the velocity command (the policy keeps
                   balancing, which IS the safe state); >1 s -> torque off.
  2. pack voltage <10.2 V (3.4 V/cell) -> torque off. Read from the servos'
                   own Present Input Voltage register; no extra hardware.
  3. |roll|       >60 deg -> torque off, so it does not thrash on its side.
                   Matches rl_general.yaml's fall_roll_deg.
  4. AHRS stale   -> torque off (raised by AhrsReader.latest).
A physical switch cutting servo power independently of the Pi is the fifth,
and the only one that still works if this process is the thing that failed.
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time

import numpy as np

from ..params import load_params
from ..control.drive import DriveController
from .ahrs import AhrsReader, MountCalibration
from .dynamixel import ServoBus
from .odometry import VelocityEstimator, body_to_world
from .state import HardwareData, load_bundle

CONTROL_HZ = 100.0
CMD_STALE_S = 0.15        # -> zero the command. PROVISIONAL, see note below
CMD_DEAD_S = 1.0          # -> torque off
VOLTAGE_MIN = 10.2        # 3.4 V/cell on 3S. Pack total, not per-cell — see note
FALL_ROLL_RAD = np.deg2rad(60.0)

# CMD_STALE_S is a round number, not a measurement. Two things can put the real
# command-age distribution's tail past it:
#
#   * `brcmfmac` enables WiFi power save by DEFAULT, which spikes latency by
#     tens to hundreds of ms — squarely inside this window. The bike then zeroes
#     its velocity command with no external cause, i.e. the failsafe firing
#     correctly on a fault that does not exist. Fix it at the OS level
#     (`iw dev wlan0 set power_save off`, made persistent) and assert it at
#     startup the same way ServoBus asserts latency_timer.
#   * Client reconnect after an AP hiccup takes seconds, which is past
#     CMD_DEAD_S as well — that surfaces as an unexplained torque-off.
#
# Set this from a measured p99 before the first untethered run. See
# docs/plans/untethered-setup.md, "The radio, and what actually goes wrong with
# it" and Verification step 2b.
#
# VOLTAGE_MIN reads the pack total via the servos' address 144, which cannot see
# individual cells. A pack with one weak cell can sit at 3.6/3.6/3.0 V and still
# report 10.2 V, so the weak cell is over-discharged with nothing onboard to
# notice. That is an accepted limitation — the free voltage read is still the
# right call — but it assumes regular balance charging keeps the cells matched
# enough for the average to mean something.


class CommandLink:
    """UDP command receive + telemetry transmit.

    The command is a JSON object rather than a packed struct: it is 50 Hz of
    tiny datagrams on a private link, the readability is worth more than the
    bytes, and it keeps the ground station trivial to write and extend.
    """

    def __init__(self, listen=("0.0.0.0", 9910), telemetry_hz=50.0):
        self.addr = listen
        self.telemetry_hz = telemetry_hz
        self.cmd = {"v_cmd_world": [0.0, 0.0], "psi_cmd": 0.0, "mode": "general"}
        self.cmd_t = 0.0
        self.peer = None
        self._stop = threading.Event()
        self._thread = None
        self.telemetry = {}

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(self.addr)
        self._sock.settimeout(0.1)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="link")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self):
        last_tx = 0.0
        while not self._stop.is_set():
            try:
                data, peer = self._sock.recvfrom(4096)
                self.cmd = json.loads(data.decode())
                self.cmd_t = time.monotonic()
                self.peer = peer
            except socket.timeout:
                pass
            except Exception:
                pass          # a malformed datagram must never stop the link
            now = time.monotonic()
            if self.peer and now - last_tx > 1.0 / self.telemetry_hz:
                try:
                    self._sock.sendto(json.dumps(self.telemetry).encode(), self.peer)
                except OSError:
                    pass
                last_tx = now

    def age(self) -> float:
        return time.monotonic() - self.cmd_t if self.cmd_t else float("inf")


class BikeRunner:
    def __init__(self, bundle_path: str, port: str, ahrs_port: str,
                 control_hz: float = CONTROL_HZ):
        self.params = load_params()
        self.design, self.model = load_bundle(bundle_path, self.params)
        self.dt = 1.0 / control_hz
        self.ctl = DriveController(self.params, self.model, self.design)
        self.aid = self.ctl.aid
        self.data = HardwareData(self.model.nq, self.model.nv, self.model.nu)
        self.bus = ServoBus(self.params, port=port)
        self.ahrs = AhrsReader(ahrs_port, calibration=MountCalibration())
        self.est = VelocityEstimator(self.params)
        self.link = CommandLink()
        self.jitter = []

    # -- one tick ----------------------------------------------------------

    def _sense(self) -> None:
        """Servos + AHRS -> HardwareData. Everything the controllers read.

        Time advances by the servos' OWN Realtime Tick delta, not by the rate
        we asked for. The tick is stamped when the servo sampled its encoder,
        so integrating over it keeps the estimator and the controller's
        zero-order hold honest when the loop jitters or drops a tick. Falls
        back to the nominal period on the first tick (no previous stamp) or if
        the delta looks implausible.
        """
        s = self.bus.to_controller_units(self.bus.read_state())
        a = self.ahrs.latest()

        dt = s["dt"] if s["dt"] and 0.2 * self.dt < s["dt"] < 5 * self.dt else self.dt
        self._dt_meas = dt
        self.data.time += dt
        self.data.set_orientation(a.quat, a.gyro)
        self.data.qpos[self.ctl._sj] = s["steer_pos"]
        self.data.qvel[self.ctl._sd] = s["steer_vel"]

        roll, pitch, yaw = _rpy(a.quat)
        v_lon, v_lat = self.est.update(
            dt, s["w_servo_a"], s["w_servo_b"],
            steer_joint=s["steer_pos"], yaw_rate=a.gyro[2],
            accel_body=a.accel, roll=roll, pitch=pitch)
        self.data.set_velocity(body_to_world(v_lon, v_lat, yaw))
        self.data.integrate_position(dt)
        self._roll = roll

    def _apply_command(self) -> None:
        age = self.link.age()
        if age > CMD_STALE_S:
            # Hold heading, zero velocity: keep balancing, stop travelling.
            self.ctl.set_command(v_cmd_world=[0.0, 0.0])
            return
        c = self.link.cmd
        self.ctl.set_command(v_cmd_world=c.get("v_cmd_world", [0.0, 0.0]),
                             psi_cmd=c.get("psi_cmd"))

    def _check_failsafes(self, voltage: float) -> str | None:
        if self.link.age() > CMD_DEAD_S:
            return f"command link dead ({self.link.age():.1f} s)"
        if voltage < VOLTAGE_MIN:
            return f"pack at {voltage:.1f} V (min {VOLTAGE_MIN})"
        if abs(self._roll) > FALL_ROLL_RAD:
            return f"roll {np.degrees(self._roll):.0f} deg — fallen"
        return None

    def run(self) -> None:
        self.bus.open()
        self.ahrs.start()
        self.link.start()
        _try_realtime()

        voltage = self.bus.pack_voltage()
        print(f"pack {voltage:.1f} V — engaging general policy at "
              f"{1/self.dt:.0f} Hz")

        self.data.time = 0.0
        self._sense()
        self.ctl.reset(self.model, self.data)
        self.ctl.engage_general(self.data)

        t0 = time.monotonic()
        next_tick = t0
        k = 0
        try:
            while True:
                next_tick += self.dt
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick - now)
                self.jitter.append(time.monotonic() - next_tick)

                self._sense()
                self._apply_command()

                k += 1
                if k % 100 == 0:            # 1 Hz; a bus read is not free
                    voltage = self.bus.pack_voltage()
                reason = self._check_failsafes(voltage)
                if reason is not None:
                    print(f"FAILSAFE: {reason}")
                    break

                self.ctl.step(self.model, self.data)
                self.bus.write_commands(self.data.ctrl, self.aid)

                self.link.telemetry = {
                    "t": round(self.data.time, 3),
                    "roll": round(float(self._roll), 4),
                    "v": [round(float(v), 3) for v in self.data.qvel[:2]],
                    "steer": round(float(self.data.qpos[self.ctl._sj]), 4),
                    "volts": round(voltage, 1),
                    # <1 means the front wheel is near perpendicular and v_lat
                    # is coasting on the accelerometer -- worth seeing live.
                    "vlat_conf": round(float(self.est.confidence), 2),
                    "jitter_ms": round(float(np.percentile(self.jitter[-500:], 99)) * 1e3, 2),
                    "dt_ms": round(self._dt_meas * 1e3, 2),
                }
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        try:
            self.bus.close()
        finally:
            self.ahrs.stop()
            self.link.stop()
            if self.jitter:
                j = np.array(self.jitter) * 1e3
                print(f"tick jitter: mean {j.mean():.2f} ms  "
                      f"p99 {np.percentile(j, 99):.2f} ms  max {j.max():.2f} ms")


def _rpy(quat) -> tuple[float, float, float]:
    """(w,x,y,z) -> (roll, pitch, yaw), the ZYX convention extract_state uses."""
    from .ahrs import quat_to_mat
    R = quat_to_mat(quat)
    return (float(np.arctan2(R[2, 1], R[2, 2])),
            float(-np.arcsin(np.clip(R[2, 0], -1, 1))),
            float(np.arctan2(R[1, 0], R[0, 0])))


def _try_realtime() -> None:
    """SCHED_FIFO for the control thread. Best effort — without CAP_SYS_NICE
    this fails, and a warning beats refusing to run."""
    try:
        import os
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(80))
    except (AttributeError, PermissionError, OSError) as e:
        print(f"warning: no SCHED_FIFO ({e}); expect worse tick jitter")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bundle", default="deploy/bundle.npz")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="U2D2 serial port")
    ap.add_argument("--ahrs-port", default="/dev/serial0", help="TM151 UART")
    ap.add_argument("--rate", type=float, default=CONTROL_HZ)
    args = ap.parse_args()
    BikeRunner(args.bundle, args.port, args.ahrs_port, args.rate).run()


if __name__ == "__main__":
    main()
