"""One set of MJWarp scratch buffers, shared by every MJWarp call in a step.

WHY. `mjw.step` allocates its working memory on every call (solver and
collision contexts, the implicit integrator's matrices, sensor temporaries)
and drops it on return. Warp's memory pool hands the same bytes back on the
next call, so an ordinary `ScopedCapture` costs one call's worth. The APIC
capture that `export_graph` saves for PufferLib does NOT reuse them: every
call's allocations get memory of their own, for the life of the graph.
Measured 2026-10-16 on an A40, 16384 worlds, one step:

    2341 allocations inside the step, 10581 MiB, 661 KiB a world
    ScopedCapture: 1.25 GB    ScopedCapture(apic=True): 11.8 GB
    the saved graph reloaded (PufferLib's path): 12.3 GB

-- 0.72 MB a world, so a 24 GB card stopped near 30k worlds and the A40's
64k graph failed to load (analysis/gpu/phase3/results_a40_bench/).

WHAT. `MjwScratch` is a Warp allocator (`wp.ScopedAllocator`). Inside an
MJWarp call (`with scratch.call():`), the i-th allocation of the call is
served from slot i, the same bytes on every call; outside one, everything
passes through to the device's own allocator untouched. A warm-up step
OUTSIDE the capture creates the slots; `freeze()` then makes any allocation
the slots cannot serve an error rather than a silent graph allocation.

WHY IT IS SAFE: the substeps run in order on one stream, so call k+1 cannot
start before call k's kernels finish -- the pool's own reuse rests on the
same fact. It would NOT be safe for an allocation that outlives its call
and is read by a later one; a slot is rewritten by the next call. Nothing
in MJWarp's step was seen doing that, and check_scratch.py compares whole
trajectories with and without to catch it.
"""
from __future__ import annotations

from contextlib import contextmanager

import warp as wp


class MjwScratch:
    deallocate_requires_context_guard = True

    def __init__(self, device):
        self.device = wp.get_device(device)
        self.base = self.device.current_allocator         # the built-in one
        self.memory_kind = getattr(self.base, "memory_kind", None)
        self.slots: list[list[int]] = []                  # [ptr, capacity]
        self.owned: set[int] = set()
        self.i: int | None = None                         # None: pass through
        self.frozen = False

    @property
    def nbytes(self) -> int:
        return sum(c for _, c in self.slots)

    def allocate(self, size_in_bytes: int) -> int:
        if self.i is None:
            return self.base.allocate(size_in_bytes)
        i, self.i = self.i, self.i + 1
        if i < len(self.slots) and self.slots[i][1] >= size_in_bytes:
            return self.slots[i][0]
        if self.frozen:
            raise RuntimeError(
                f"MjwScratch: allocation {i} of an MJWarp call ({size_in_bytes} B) has no "
                f"slot after freeze() -- the captured step differs from the warm-up step")
        ptr = self.base.allocate(size_in_bytes)       # kept for good: never freed
        self.owned.add(ptr)
        if i < len(self.slots):
            self.slots[i] = [ptr, size_in_bytes]      # outgrown; the old one stays owned
        else:
            self.slots.append([ptr, size_in_bytes])
        return ptr

    def deallocate(self, ptr: int, size_in_bytes: int) -> None:
        if ptr in self.owned:
            return
        self.base.deallocate(ptr, size_in_bytes)

    def freeze(self) -> None:
        self.frozen = True

    @contextmanager
    def call(self):
        """Around ONE MJWarp entry point: its allocations number from 0."""
        self.i = 0
        try:
            yield
        finally:
            self.i = None
