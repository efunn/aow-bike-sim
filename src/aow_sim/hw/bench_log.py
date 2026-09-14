"""Bench capture: a command schedule run against a `DynamixelBus`, logged whole.

`DynamixelBus.capture` returns rows in memory and suits a script that analyses
on the spot. A bench session wants the opposite: record EVERYTHING the read
block carries, save it with the configuration it was taken under, and decide
what to compute afterwards -- because a hand-held rig makes a re-run expensive,
and the question asked of the data changes after the first look at it.

A CAPTURE IS A DIRECTORY:

    capture.npz   one row per frame
        t_host      [s]   host clock at the START of the read, since capture start
        read_s      [s]   duration of the FastSyncRead call
        write_s     [s]   duration of the SyncWrite call (nan: nothing sent)
        ok          bool  False = the read failed; that frame's raw rows are 0
        segment     int   index into meta["segments"]
        ids         (S,)  servo ids, the column order of every (N, S) array
        raw_<label> (N, S) int64  register fields EXACTLY as read: unsigned,
                          before sign or unit
        cmd         (N, S) float  command in the write register's physical
                          units (nan: not sent to that servo on that frame)
    meta.json     how to decode it without this repo's current state: each
                  servo's register address/size/unit/sign per label, the
                  segment table, and whatever the caller adds (config
                  snapshots, git revision, rig note)

RAW ON DISK, DECODED ON LOAD. A unit mistake in `control_table.py` found next
month is then a re-decode of existing captures rather than a re-run with the rig
held up again. `Capture.value()` decodes from the units stored in meta.json, not
from the table as it is on the day of analysis.

TIMING. Each servo's Realtime Tick is the truth (see `DynamixelBus.capture`).
`t_host` and the call durations exist to diagnose the host -- a GC pause, a slow
write -- and to place commands, which the HOST times. The tick has 1 ms
resolution, so at 500 Hz a single-frame dt is 2 +- 1 ms: difference over spans,
not over consecutive frames, when that matters (`Capture.diff_velocity`).

A DROPPED FRAME IS RECORDED, NOT FATAL. `read_frame` raises on a bad read; one
corrupted packet should not cost a hand-held capture, so the frame is kept with
`ok=False` and the schedule carries on. Ctrl-C and any other exception also
stop the schedule and still return what was captured, with
`meta["complete"] = False` and the reason in `meta["stopped_by"]`.
"""

from __future__ import annotations

import dataclasses
import datetime as _dt
import json
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np

from .control_table import Register

FORMAT = "aow-bench-capture"
FORMAT_VERSION = 1
TICK_WRAP = 32768        # Realtime Tick is 0..32767 ms
POS_WRAP = 4096          # single-turn Present Position, velocity and PWM modes


def raw_key(label: str) -> str:
    """npz key for a read label: ``"Present Position"`` -> ``raw_present_position``."""
    return "raw_" + "".join(c if c.isalnum() else "_" for c in label.lower())


@dataclass
class Segment:
    """One stretch of a schedule.

    ``command(t, state)`` is called every frame with ``t`` seconds since the
    segment began and that frame's decoded ``{id: {label: value}}`` (None when
    the read failed). It returns ``{id: value}`` in the write register's units,
    or None / ``{}`` to send nothing.

    ``enter(bus)`` runs once before the segment's first frame -- torque, gains.
    Its bus traffic stalls the loop for as long as it takes, and the tick record
    shows exactly how long, so that is visible rather than hidden.

    ``info`` goes into meta.json verbatim: whatever an analysis needs to know
    about the segment without re-deriving it from the label.
    """

    label: str
    seconds: float
    command: Callable | None = None
    enter: Callable | None = None
    info: dict = field(default_factory=dict)


def record(bus, imap, segments, rate_hz: float, log=print,
           finish=None) -> "Capture":
    """Run ``segments`` back to back at ``rate_hz`` (0 = unpaced).

    ``bus`` needs only ``read_frame(decode=False)`` and ``write_frame(values)``;
    ``imap`` is the installed :class:`IndirectMap`, which names and decodes the
    read block.

    ``finish(bus)`` runs the moment the schedule ends -- normally, on Ctrl-C or
    on an error -- and BEFORE the frames are converted to arrays. That
    conversion is a pause in bus traffic that grows with the capture, and a
    caller holding a Bus Watchdog armed must stop traffic cleanly first: on
    2026-09-13 two 89 s captures read the watchdog clear on every one of their
    frames and TRIPPED at shutdown. A failure inside ``finish`` is recorded in
    ``meta["finish_error"]``, never raised, so it cannot cost the capture.
    """
    ids = tuple(imap.ids)
    labels = tuple(imap.read_labels)
    regs = {(i, lbl): imap.register(i, lbl) for i in ids for lbl in labels}
    n_servo = len(ids)
    nan_row = (np.nan,) * n_servo
    zero_row = (0,) * n_servo

    t_host, read_s, write_s, ok, seg, cmd = [], [], [], [], [], []
    cols = {lbl: [] for lbl in labels}
    table, overruns, stopped_by = [], 0, None
    dt = 1.0 / rate_hz if rate_hz > 0 else 0.0
    clock = time.perf_counter
    t0 = clock()
    next_t = 0.0
    try:
        for k, s in enumerate(segments):
            if log:
                log(f"[{k + 1}/{len(segments)}] {s.label}  ({s.seconds:g} s)")
            if s.enter is not None:
                s.enter(bus)
            t_seg = clock() - t0
            next_t = max(next_t, t_seg)      # do not sprint to repay enter()'s I/O
            table.append({"label": s.label, "seconds": s.seconds, "info": s.info,
                          "start": len(t_host), "stop": len(t_host),
                          "t_host": t_seg})
            while True:
                ta = clock() - t0
                if ta - t_seg >= s.seconds:
                    break
                try:
                    state, good = bus.read_frame(decode=False), True
                except RuntimeError:
                    state, good = None, False
                tb = clock() - t0
                decoded = ({i: {lbl: regs[i, lbl].decode(state[i][lbl])
                                for lbl in labels} for i in ids}
                           if good else None)
                out = s.command(ta - t_seg, decoded) if s.command else None
                w, row = np.nan, nan_row
                if out:
                    bus.write_frame(out)
                    w = clock() - t0 - tb
                    row = tuple(float(out[i]) if i in out else np.nan for i in ids)
                # All appends together, so an interrupt can misalign at most
                # the final frame -- trimmed below.
                t_host.append(ta)
                read_s.append(tb - ta)
                write_s.append(w)
                ok.append(good)
                seg.append(k)
                cmd.append(row)
                for lbl in labels:
                    cols[lbl].append(tuple(state[i][lbl] for i in ids)
                                     if good else zero_row)
                table[-1]["stop"] = len(t_host)
                if dt:
                    next_t += dt
                    slack = next_t - (clock() - t0)
                    if slack > 0:
                        time.sleep(min(slack, dt))
                    else:
                        overruns += 1
                        next_t = clock() - t0
    except KeyboardInterrupt:
        stopped_by = "KeyboardInterrupt"
        if log:
            log("interrupted -- keeping the frames captured so far")
    except Exception as e:                   # noqa: BLE001 -- saved, then reported
        stopped_by = f"{type(e).__name__}: {e}"
        if log:
            log(f"stopped by {stopped_by} -- keeping the frames captured so far")

    finish_error = None
    if finish is not None:
        try:
            finish(bus)
        except Exception as e:               # noqa: BLE001 -- recorded, not raised
            finish_error = f"{type(e).__name__}: {e}"
            if log:
                log(f"finish hook failed: {finish_error}")

    n = min(len(t_host), len(read_s), len(write_s), len(ok), len(seg), len(cmd),
            *(len(c) for c in cols.values()))
    for entry in table:
        entry["stop"] = min(entry["stop"], n)
        entry["start"] = min(entry["start"], n)

    arrays = {
        "t_host": np.asarray(t_host[:n], float),
        "read_s": np.asarray(read_s[:n], float),
        "write_s": np.asarray(write_s[:n], float),
        "ok": np.asarray(ok[:n], bool),
        "segment": np.asarray(seg[:n], np.int32),
        "ids": np.asarray(ids, np.int64),
        "cmd": np.asarray(cmd[:n], float).reshape(n, n_servo),
    }
    for lbl in labels:
        arrays[raw_key(lbl)] = np.asarray(cols[lbl][:n], np.int64).reshape(n, n_servo)

    def reg_meta(per_label):
        return {str(i): dataclasses.asdict(per_label(i)) for i in ids}

    meta = {
        "format": FORMAT,
        "version": FORMAT_VERSION,
        "created": _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "rate_hz": rate_hz,
        "ids": list(ids),
        "read": [{"label": lbl, "key": raw_key(lbl),
                  "registers": reg_meta(lambda i, lbl=lbl: regs[i, lbl])}
                 for lbl in labels],
        "write": [{"label": lbl,
                   "registers": reg_meta(lambda i, lbl=lbl: imap.register(i, lbl))}
                  for lbl in imap.write_labels],
        "segments": table,
        "complete": stopped_by is None,
        "stopped_by": stopped_by,
        "finish_error": finish_error,
        "stats": {"frames": n, "dropped": int(n - arrays["ok"].sum()),
                  "overruns": overruns},
    }
    return Capture(arrays, meta)


def _decode(reg: Register, raw: np.ndarray) -> np.ndarray:
    v = raw.astype(np.int64)
    if reg.signed:
        bits = 8 * reg.size
        v = np.where(v >= (1 << (bits - 1)), v - (1 << bits), v)
    v = v.astype(float)
    return v * reg.unit if reg.unit is not None else v


class Capture:
    """A loaded (or just recorded) capture. Decodes from its own meta."""

    def __init__(self, arrays: dict, meta: dict):
        self.arrays, self.meta = arrays, meta
        self.ids = tuple(int(i) for i in arrays["ids"])
        self._col = {i: c for c, i in enumerate(self.ids)}
        self._reg = {e["label"]: {int(i): Register(**d)
                                  for i, d in e["registers"].items()}
                     for e in meta["read"]}
        self._key = {e["label"]: e["key"] for e in meta["read"]}

    # -- io ----------------------------------------------------------------

    def save(self, directory) -> Path:
        d = Path(directory)
        d.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(d / "capture.npz", **self.arrays)
        (d / "meta.json").write_text(json.dumps(self.meta, indent=1,
                                                default=_jsonable) + "\n")
        return d

    @classmethod
    def load(cls, directory) -> "Capture":
        d = Path(directory)
        meta = json.loads((d / "meta.json").read_text())
        if meta.get("format") != FORMAT:
            raise ValueError(f"{d} is not a {FORMAT} (format={meta.get('format')!r})")
        with np.load(d / "capture.npz", allow_pickle=False) as z:
            arrays = {k: z[k] for k in z.files}
        return cls(arrays, meta)

    # -- access ------------------------------------------------------------

    def __len__(self) -> int:
        return len(self.arrays["t_host"])

    @property
    def labels(self) -> tuple:
        return tuple(self._reg)

    @property
    def ok(self) -> np.ndarray:
        return self.arrays["ok"]

    @property
    def segments(self) -> list:
        return self.meta["segments"]

    def segment_slice(self, key) -> slice:
        """By index, or by label (first match)."""
        if isinstance(key, str):
            key = next(k for k, s in enumerate(self.segments) if s["label"] == key)
        s = self.segments[key]
        return slice(s["start"], s["stop"])

    def register(self, label: str, dxl_id: int) -> Register:
        return self._reg[label][dxl_id]

    def raw(self, label: str, dxl_id: int) -> np.ndarray:
        return self.arrays[self._key[label]][:, self._col[dxl_id]]

    def value(self, label: str, dxl_id: int) -> np.ndarray:
        """Physical units, from the units recorded in meta. nan where not ok."""
        v = _decode(self.register(label, dxl_id), self.raw(label, dxl_id))
        v[~self.ok] = np.nan
        return v

    def command(self, dxl_id: int) -> np.ndarray:
        return self.arrays["cmd"][:, self._col[dxl_id]]

    # -- derived -----------------------------------------------------------

    def time_s(self, dxl_id: int) -> np.ndarray:
        """The servo's own clock, unwrapped, seconds since its first good frame.

        Undoes the 32768 ms wrap by assuming consecutive good frames are less
        than 32.8 s apart, which a running capture always satisfies.
        """
        out = np.full(len(self), np.nan)
        good = np.flatnonzero(self.ok)
        if len(good):
            tick = self.raw("Realtime Tick", dxl_id)[good]
            d = np.diff(tick) % TICK_WRAP
            out[good] = np.concatenate([[0], np.cumsum(d)]) * 1e-3
        return out

    def position_rad(self, dxl_id: int, label: str = "Present Position") -> np.ndarray:
        """Continuous shaft angle [rad], unwrapped along the shortest path.

        Velocity and PWM modes report position over ONE turn, so a spinning
        shaft rolls 4095 -> 0; see `hw.dynamixel._pos_delta`. For a multi-turn
        mode the shortest path gives the plain difference, so this is correct
        either way while the shaft turns under half a rev per frame.
        """
        reg = self.register(label, dxl_id)
        out = np.full(len(self), np.nan)
        good = np.flatnonzero(self.ok)
        if len(good):
            counts = _decode(dataclasses.replace(reg, unit=None),
                             self.raw(label, dxl_id)[good])
            d = ((np.diff(counts) + POS_WRAP // 2) % POS_WRAP) - POS_WRAP // 2
            out[good] = (counts[0] + np.concatenate([[0], np.cumsum(d)])) * reg.unit
        return out

    def diff_velocity(self, dxl_id: int, half_span: int = 5) -> np.ndarray:
        """Central difference of position over +-``half_span`` good frames [rad/s].

        Against the servo tick, not the host clock. One encoder count over the
        span is the noise floor: at 500 Hz, half_span 5 spans ~20 ms, i.e.
        ~0.08 rad/s per count.
        """
        out = np.full(len(self), np.nan)
        good = np.flatnonzero(self.ok)
        h = int(half_span)
        if len(good) > 2 * h:
            p = self.position_rad(dxl_id)[good]
            t = self.time_s(dxl_id)[good]
            dp, dtt = p[2 * h:] - p[:-2 * h], t[2 * h:] - t[:-2 * h]
            v = np.full(len(good), np.nan)
            with np.errstate(divide="ignore", invalid="ignore"):
                v[h:-h] = np.where(dtt > 0, dp / dtt, np.nan)
            out[good] = v
        return out


def new_capture_dir(root, test: str, tag: str | None = None) -> Path:
    """``root/YYMMDD-HHMMSS_<test>[_<tag>]``, never an existing directory."""
    stamp = _dt.datetime.now().strftime("%y%m%d-%H%M%S")
    base = Path(root) / "_".join(p for p in (stamp, test, tag) if p)
    d, n = base, 2
    while d.exists():
        d, n = base.with_name(f"{base.name}-{n}"), n + 1
    d.mkdir(parents=True)
    return d


def git_state(repo: Path | None = None) -> dict:
    """Commit and dirty paths of the checkout that ran the capture."""
    repo = repo or Path(__file__).resolve().parents[3]

    def git(*a):
        return subprocess.run(["git", *a], cwd=repo, capture_output=True,
                              text=True, timeout=5).stdout.strip()
    try:
        return {"commit": git("rev-parse", "HEAD") or None,
                "dirty": git("status", "--porcelain").splitlines()}
    except (OSError, subprocess.SubprocessError):
        return {"commit": None, "dirty": None}


def _jsonable(o):
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    raise TypeError(f"not JSON serialisable: {type(o).__name__}")
