"""Read a running TensorBoard's scalars over its own HTTP API.

TensorBoard already serves every scalar as JSON; `rl.sh board` already starts it
with --bind_all. So watching a remote run needs no data sync, no extra server
and no change to how the board is launched -- just a client:

    python scripts/tb_summary.py --host 192.168.1.101
    python scripts/tb_summary.py --host 192.168.1.101 --eval
    python scripts/tb_summary.py --host 192.168.1.101 --tag rollout/ep_rew_mean

Default view answers "is it still improving, or is it done?" by binning the
reward curve against the curriculum -- a reward DROP is usually the curriculum
getting harder, not the policy getting worse, and the two are indistinguishable
without plotting them together. `--eval` prints the eval metrics as a matrix,
which is where the slow regressions show up that `score` alone hides.

Endpoints used (all read-only, all built in):
    /data/runs
    /data/plugin/scalars/tags
    /data/plugin/scalars/scalars?run=<run>&tag=<tag>   -> [[wall, step, value]]
"""

from __future__ import annotations

import argparse
import json
import statistics as st
import urllib.error
import urllib.parse
import urllib.request


def fetch(host: str, path: str, timeout: float = 10.0):
    url = f"http://{host}{path}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.load(r)
    except urllib.error.URLError as e:
        raise SystemExit(
            f"cannot reach TensorBoard at {url}\n"
            f"  {e}\n"
            "  is it up? `./scripts/rl.sh board <move>` on the training box"
        ) from e


def scalars(host: str, run: str, tag: str):
    q = urllib.parse.urlencode({"run": run, "tag": tag})
    return fetch(host, f"/data/plugin/scalars/scalars?{q}")


def pick_run(host: str, want: str | None) -> str:
    runs = fetch(host, "/data/runs")
    if not runs:
        raise SystemExit("TensorBoard is up but has no runs yet")
    if want and want not in runs:
        raise SystemExit(f"no run {want!r}; available: {', '.join(runs)}")
    return want or runs[-1]


def cmd_progress(host: str, run: str, tags: set[str], bins: int) -> None:
    rew = scalars(host, run, "rollout/ep_rew_mean")
    last_step = rew[-1][1]
    print(f"run {run}: {len(rew)} points, {last_step:,} steps\n")

    # The curriculum is what makes a falling reward curve ambiguous, so bin the
    # two together. Absent (not every trainer has one) -> just the reward.
    diff = scalars(host, run, "curriculum/difficulty") \
        if "curriculum/difficulty" in tags else []
    full = next((r[1] for r in diff if r[2] >= 0.999), None)

    n = len(rew)
    head = f"{'steps':>23} {'ep_rew_mean':>12}" + ("  difficulty" if diff else "")
    print(head)
    for i in range(bins):
        lo, hi = i * n // bins, (i + 1) * n // bins
        c = rew[lo:hi]
        if not c:
            continue
        line = (f"{c[0][1]:>10,}-{c[-1][1]:>11,} "
                f"{st.mean(x[2] for x in c):>12.1f}")
        if diff:
            d = [x[2] for x in diff if c[0][1] <= x[1] <= c[-1][1]]
            line += f"  {st.mean(d) if d else float('nan'):>10.2f}"
        print(line)

    if full is not None:
        post = [x[2] for x in rew if x[1] >= full]
        half = len(post) // 2
        if half:
            a, b = st.mean(post[:half]), st.mean(post[half:])
            print(f"\ncurriculum topped out at {full:,} "
                  f"({100 * full / last_step:.0f}% through)")
            print(f"since then: {len(post)} points, "
                  f"{a:.1f} -> {b:.1f} ({100 * (b - a) / abs(a):+.1f}%)")
            print("  -> " + ("still climbing" if b > a * 1.05 else
                             "PLATEAUED; further steps are buying little"))


def cmd_eval(host: str, run: str, tags: set[str]) -> None:
    """Eval metrics as a matrix. Read the ROWS: score can sit flat while a
    secondary metric drifts steadily the wrong way, and that is the failure
    this view exists to catch."""
    names = sorted(t for t in tags if t.startswith("eval/"))
    if not names:
        raise SystemExit(f"run {run!r} logs no eval/* scalars")
    data = {t: scalars(host, run, t) for t in names}
    steps = [r[1] for r in data[names[0]]]
    w = max(len(t) for t in names)
    print(f"{'eval tag':<{w}} " + " ".join(f"{s / 1e6:>8.2f}M" for s in steps))
    for t in names:
        print(f"{t:<{w}} " + " ".join(f"{r[2]:>9.3f}" for r in data[t]))


def cmd_tag(host: str, run: str, tag: str) -> None:
    s = scalars(host, run, tag)
    vals = [r[2] for r in s]
    print(f"{tag}: {len(s)} points, last step {s[-1][1]:,}")
    print(f"  first {vals[0]:.4f}   last {vals[-1]:.4f}   "
          f"min {min(vals):.4f}   max {max(vals):.4f}")
    if len(vals) >= 10:
        print(f"  last 10 mean {st.mean(vals[-10:]):.4f}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.1.101:6006",
                    help="host:port of the running board (default %(default)s)")
    ap.add_argument("--run", default=None, help="which run (default: the last)")
    ap.add_argument("--eval", action="store_true", help="eval metrics matrix")
    ap.add_argument("--tag", default=None, help="dump one tag instead")
    ap.add_argument("--bins", type=int, default=10)
    a = ap.parse_args()

    host = a.host if ":" in a.host else f"{a.host}:6006"
    run = pick_run(host, a.run)
    tags = set(fetch(host, "/data/plugin/scalars/tags").get(run, []))

    if a.tag:
        cmd_tag(host, run, a.tag)
    elif a.eval:
        cmd_eval(host, run, tags)
    else:
        cmd_progress(host, run, tags, a.bins)


if __name__ == "__main__":
    main()
