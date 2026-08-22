"""What the Onshape API quota has been spent on, and whether the rate is fine.

    python analysis/onshape_usage.py                 # summary
    python analysis/onshape_usage.py --plot          # -> analysis/plots/onshape_usage.png
    python analysis/onshape_usage.py --plot --tag alt

Reads `~/.local/state/aow/onshape_calls.jsonl` and changes nothing — same rule
as every other script in this directory.

WHY THIS EXISTS. The quota is **2500 billable calls per YEAR**, and Onshape
shows you a total and never a breakdown. "38 of 2500" does not tell you whether
those went on useful pushes or on a loop somebody left running, and by the time
the number is alarming the evidence is gone. The log answers that; this
summarises it.

WHY IT SEARCHES FOR OTHER LOGS. On 2026-08-21 four real calls were made through
the standalone `onshape-api` skill script with `ONSHAPE_LOG` pointed at /tmp.
They counted against the account and not against the local tally, which
therefore under-reported by four and looked perfectly healthy. Onshape bills the
ACCOUNT; a log file is only a ledger, and two ledgers is a silent undercount. So
this looks for strays rather than trusting one path.

The counter is advisory in any case. The usage page at cad.onshape.com is the
authority — reconcile against it occasionally rather than trusting this.
"""

from __future__ import annotations

import argparse
import json
import os
from collections import Counter, defaultdict
from datetime import date, datetime, timedelta
from pathlib import Path

from aow_sim.onshape import BUDGET, LOG, cycle_start, read_log

# Where a stray ledger plausibly ends up. Not exhaustive — a deliberate
# ONSHAPE_LOG somewhere exotic will not be found, which is why the summary
# prints the paths it actually read.
STRAY_GLOBS = [
    (Path("/tmp"), "*onshape*.jsonl"),
    (Path.home() / ".local/state", "onshape/*.jsonl"),
    (Path.home(), "onshape_calls.jsonl"),
]


def _plots_dir() -> Path:
    d = Path(__file__).resolve().parent / "plots"
    d.mkdir(exist_ok=True)
    return d


def find_logs() -> list[Path]:
    """Every ledger we can see, main one first, deduplicated by real path."""
    found, seen = [], set()
    for p in [LOG, Path(os.environ["ONSHAPE_LOG"])
              if os.environ.get("ONSHAPE_LOG") else None]:
        if p and p.exists() and p.resolve() not in seen:
            seen.add(p.resolve())
            found.append(p)
    for root, pat in STRAY_GLOBS:
        if not root.exists():
            continue
        for p in sorted(root.glob(pat)):
            if p.resolve() not in seen:
                seen.add(p.resolve())
                found.append(p)
    return found


def load(path: Path) -> list[dict]:
    if path.resolve() == LOG.resolve():
        return read_log()                     # one parser, one place
    out = []
    try:
        for ln in path.read_text().splitlines():
            try:
                out.append(json.loads(ln))
            except ValueError:
                pass
    except OSError:
        pass
    return out


def kind(rec: dict) -> str:
    """Bucket a call by what it was for, from the human `what` string.

    Bucketing on `what` rather than on the endpoint because the endpoint is not
    the interesting axis — two GETs against /features can be one deliberate
    read or the first two iterations of a loop, and the sentence says which.
    """
    w = (rec.get("what") or "").lower().removeprefix("[skill test] ")
    for key in ("push", "render", "check", "tabs", "documents", "feature tree",
                "read feature studio", "list"):
        if key in w:
            return {"feature tree": "read state",
                    "read feature studio": "read studio",
                    "documents": "list docs", "list": "list docs"}.get(key, key)
    return "other"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--plot", action="store_true",
                    help="also write a cumulative-spend figure")
    ap.add_argument("--tag", default="",
                    help="suffix for the figure, so a variant gets its own "
                         "name instead of overwriting the tracked one")
    args = ap.parse_args()

    logs = find_logs()
    if not logs:
        raise SystemExit(f"no call log found. Expected {LOG}.\n"
                         "  Nothing has used the API from this machine yet, or "
                         "ONSHAPE_LOG points somewhere not searched.")

    # DEDUPLICATE ACROSS LEDGERS. A stray log's entries may already have been
    # merged into the main one, in which case naive summing double-counts them
    # and reports MORE usage than the account has — the opposite of the
    # undercount this script exists to catch, and just as wrong. The identity
    # of a call is when it happened plus what it hit; `what` is excluded from
    # the key because a merge may have annotated it ("[skill test] ...").
    rows, per_file, seen = [], {}, set()
    for p in logs:
        r = load(p)
        per_file[p] = r
        # Dedup only ACROSS files, never within one. A ledger is append-only,
        # so two of its own records are two real calls even when they look
        # identical — and they legitimately can: three backfilled entries
        # written in the same second with no endpoint recorded collapse to one
        # under a naive key, quietly under-reporting the thing this script
        # exists to report. `what` is normalised into the key because a merge
        # may have annotated it ("[skill test] ...").
        for rec in r:
            key = (rec.get("t"), rec.get("method"), rec.get("endpoint"),
                   rec.get("status"),
                   (rec.get("what") or "").removeprefix("[skill test] "))
            if key in seen:
                continue
            rows.append(rec)
        seen.update(
            (rec.get("t"), rec.get("method"), rec.get("endpoint"),
             rec.get("status"),
             (rec.get("what") or "").removeprefix("[skill test] "))
            for rec in r)

    print("LEDGERS READ")
    for p, r in per_file.items():
        bill = sum(1 for x in r if x.get("billable"))
        main = "  <- the main one" if p.resolve() == LOG.resolve() else ""
        print(f"  {bill:4} billable  {len(r):4} total   {p}{main}")
    if len(per_file) > 1:
        dupes = sum(len(r) for r in per_file.values()) - len(rows)
        print(f"  {len(rows)} distinct calls after removing {dupes} that "
              f"appear in more than one ledger.")
        if dupes < sum(len(r) for r in per_file.values()) - max(
                len(r) for r in per_file.values()):
            print("  NOTE: a ledger holds calls the main one does not. Onshape "
                  "bills the\n        ACCOUNT, so reading one file alone would "
                  "under-report. Merge them,\n        or point ONSHAPE_LOG at "
                  "the main one.")

    start = cycle_start()
    since = start.isoformat()
    live = [r for r in rows if r.get("t", "") >= since]
    billable = [r for r in live if r.get("billable")]
    failed = [r for r in live if not r.get("billable")]
    spent = len(billable)
    reset = date(start.year + 1, start.month, start.day)
    left_days = (reset - date.today()).days

    print(f"\nCYCLE  {start:%d %b %Y} -> {reset:%d %b %Y}")
    print(f"  spent      {spent}/{BUDGET}  ({100 * spent / BUDGET:.1f}%)")
    print(f"  remaining  {BUDGET - spent} calls, {left_days} days "
          f"({(BUDGET - spent) / max(left_days, 1):.0f}/day available)")
    print(f"  free       {len(failed)} failed calls (4xx/5xx are not billable)")

    print("\nWHAT IT WENT ON")
    for k, n in Counter(kind(r) for r in billable).most_common():
        bar = "#" * min(40, round(40 * n / max(spent, 1)))
        print(f"  {k:12} {n:4}  {bar}")

    by_day = defaultdict(int)
    for r in billable:
        by_day[r["t"][:10]] += 1
    if by_day:
        print("\nBY DAY")
        for d in sorted(by_day)[-10:]:
            print(f"  {d}  {by_day[d]:4}  {'#' * min(40, by_day[d])}")
        days_used = len(by_day)
        rate = spent / max(days_used, 1)
        print(f"\n  {rate:.1f} calls on an average ACTIVE day ({days_used} of "
              f"them so far).")
        if days_used < 3:
            print("  Too few active days to project from — one busy setup day "
                  "extrapolates\n  to a catastrophe that will not happen. "
                  "Come back after a week of use.")
            rate = None
        # Projecting on active days rather than elapsed days: the elapsed-day
        # rate is dominated by the long stretches where nobody touched CAD, and
        # would tell you everything is fine right up until it is not.
        if rate is None:
            pass                              # already said why, above
        elif rate * left_days > BUDGET - spent:
            print(f"  AT THIS RATE the cycle runs out before it resets "
                  f"({rate * left_days:.0f} projected vs "
                  f"{BUDGET - spent} left). Something is probably polling.")
        else:
            print(f"  At this rate: ~{rate * left_days:.0f} more against "
                  f"{BUDGET - spent} left. Fine.")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        ts = sorted(datetime.fromisoformat(r["t"]) for r in billable)
        fig, ax = plt.subplots(figsize=(9, 4.5))
        ax.plot(ts, range(1, len(ts) + 1), drawstyle="steps-post", lw=1.8)
        ax.axhline(BUDGET, color="tab:red", ls="--", lw=1,
                   label=f"budget {BUDGET}")
        # Draw the projection only when the TEXT was willing to make one.
        # Recomputing it here produced a figure that projected 2050 calls off a
        # single day while the summary above it said not to project at all —
        # and the picture is what people remember.
        if ts and rate:
            ax.plot([ts[-1], datetime.combine(reset, datetime.min.time())],
                    [spent, spent + rate * left_days], color="tab:orange",
                    ls=":", lw=1.5, label="at current active-day rate")
        elif ts:
            ax.text(0.5, 0.5, f"only {len(by_day)} active day(s)\n"
                              "— too little to project from",
                    transform=ax.transAxes, ha="center", va="center",
                    fontsize=11, color="0.45")
        ax.set_ylabel("cumulative billable calls")
        ax.set_title(f"Onshape API usage — {spent}/{BUDGET}, "
                     f"{left_days} days to reset")
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(alpha=0.3)
        fig.autofmt_xdate()
        fig.tight_layout()
        out = _plots_dir() / f"onshape_usage{'_' + args.tag if args.tag else ''}.png"
        fig.savefig(out, dpi=140)
        print(f"\nwrote {out}")

    print("\n  Advisory only. cad.onshape.com -> account settings is the "
          "authority;\n  reconcile against it rather than trusting this.")


if __name__ == "__main__":
    main()
