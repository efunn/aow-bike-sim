"""Push the generated Feature Studio to Onshape, and pull a render back.

    python -m aow_sim.cad_layout --format featurescript \
        --push https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>

WHY THIS IS SMALL: the whole workflow is two calls. `POST /featurestudios/...`
replaces a studio's text; `GET /partstudios/.../shadedviews` returns a PNG of
whatever that text built. Everything else -- feature trees, mass properties,
translations -- is available and is exactly the kind of thing that burns the
quota, so it is deliberately not here. See docs/plans/cad-onshape-workflow.md.

THE QUOTA IS THE DESIGN CONSTRAINT. Free and Standard plans get **2500 API
calls per year**, per user. Not per day. Only 2xx/3xx responses count, so a
push that fails on a compile error is free. Every call is appended to
`~/.local/state/aow/onshape_calls.jsonl` with what it did, because Onshape
shows you a total and never a breakdown. It is advisory, not authoritative --
`python -m aow_sim.onshape --log` prints it, and the running total is meant to
be checked against cad.onshape.com occasionally, not trusted blindly. It lives
outside the repo because it is per-machine state, not project data.

AUTH is HTTP Basic with `accessKey:secretKey`. Onshape calls that "local
testing only" and prefers an HMAC-SHA256 request signature; for one script on
one machine that is forty lines to defend against an attacker who already has
your Keychain. Keys come from the macOS Keychain first:

    security add-generic-password -a onshape -s aow-onshape-access -w
    security add-generic-password -a onshape -s aow-onshape-secret -w

then from ONSHAPE_ACCESS_KEY / ONSHAPE_SECRET_KEY. There is deliberately no
file fallback: this checkout lives under Dropbox/CloudStorage, where a
gitignored `.env` is still synced to someone else's computer.

stdlib only (urllib, not requests) -- `requests` is not in `dependencies` and
this is a dev convenience, not something the Pi or the sim should ever pull in.
"""

from __future__ import annotations

import base64
import json
import os
import re
import subprocess
import urllib.error
import urllib.request
from datetime import date, datetime
from pathlib import Path

# Unversioned, which Onshape maps to the current API version. A `/v10` style
# prefix pins it, and is what to reach for if a response shape ever changes
# underneath this -- but pinning to a version that gets retired 404s instead.
API = "https://cad.onshape.com/api"

# Onshape's own documented Accept header. The `qs=0.09` is a content-negotiation
# quality hint their gateway wants; plain application/json usually works and
# occasionally does not.
ACCEPT = "application/json;charset=UTF-8; qs=0.09"

KEYCHAIN = ("aow-onshape-access", "aow-onshape-secret")
ENVVARS = ("ONSHAPE_ACCESS_KEY", "ONSHAPE_SECRET_KEY")
LOG = Path.home() / ".local/state/aow/onshape_calls.jsonl"
BUDGET = 2500          # free/standard plan; professional is 5000
# The annual window is anchored to the account, NOT to the calendar year, and
# NOT to the "Tracking start date" the usage page shows — that field said
# 19 Feb 2026 while the same page said 312/365 days elapsed, which puts the
# real anchor at 13 Oct 2025. Derived from the elapsed-days figure on
# 2026-08-21, so re-read it off cad.onshape.com if the countdown ever looks
# wrong; the day-count is the trustworthy field and the date is not.
CYCLE_ANCHOR = (10, 13)

# https://cad.onshape.com/documents/{did}/{wvm}/{wvmid}/e/{eid}
URL_RE = re.compile(r"/documents/([0-9a-f]{24})/([wvm])/([0-9a-f]{24})/e/([0-9a-f]{24})")


class OnshapeError(RuntimeError):
    pass


DOC_CFG = "config/onshape.yaml"


def tab_url(tab: str = "feature_studio", cfg: str = DOC_CFG) -> str:
    """Browser URL for a named tab in config/onshape.yaml.

    Indirection through a name rather than an id because the two Part Studios
    and the Feature Studio are three 24-hex strings that differ in no visible
    way, and passing the wrong one is a 404 with nothing in it to read.
    """
    import yaml
    d = yaml.safe_load(Path(cfg).read_text())
    if tab not in d["tabs"]:
        raise OnshapeError(f"no tab {tab!r} in {cfg} — have "
                           f"{', '.join(d['tabs'])}")
    return (f"https://cad.onshape.com/documents/{d['document']}"
            f"/w/{d['workspace']}/e/{d['tabs'][tab]}")


def resolve(target: str | None, default_tab: str) -> str:
    """A full URL, a tab NAME from the config, or None for the default tab."""
    if target and "/" in target:
        return target
    return tab_url(target or default_tab)


def parse_url(url: str) -> tuple[str, str, str, str]:
    """(did, wvm, wvmid, eid) from a browser URL — paste the tab, not the ids.

    Every id is a 24-char hex, so getting two of them the wrong way round is
    undetectable at the call site and shows up as a 404 with no clue in it.
    Parsing the URL removes the chance entirely.
    """
    m = URL_RE.search(url)
    if not m:
        raise OnshapeError(
            f"not an Onshape element URL: {url!r}\n"
            "  expected .../documents/<did>/w/<wid>/e/<eid> — open the TAB you "
            "want and copy the address bar")
    return m.groups()


def _keychain(service: str) -> str | None:
    try:
        r = subprocess.run(
            ["security", "find-generic-password", "-a", "onshape", "-s", service, "-w"],
            capture_output=True, text=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None
    return r.stdout.strip() or None


def _auth() -> str:
    keys = [_keychain(s) or os.environ.get(e) for s, e in zip(KEYCHAIN, ENVVARS)]
    if not all(keys):
        raise OnshapeError(
            "no Onshape API keys. Either\n"
            f"  security add-generic-password -a onshape -s {KEYCHAIN[0]} -w\n"
            f"  security add-generic-password -a onshape -s {KEYCHAIN[1]} -w\n"
            f"or export {ENVVARS[0]} / {ENVVARS[1]}.\n"
            "Make them at cad.onshape.com → account settings → API keys "
            "(individual accounts are capped at two, and the secret is shown once).")
    return "Basic " + base64.b64encode(":".join(keys).encode()).decode()


def cycle_start() -> date:
    """First day of the current billing window — the most recent CYCLE_ANCHOR."""
    today = date.today()
    start = date(today.year, *CYCLE_ANCHOR)
    return start if start <= today else date(today.year - 1, *CYCLE_ANCHOR)


def read_log() -> list[dict]:
    try:
        lines = LOG.read_text().splitlines()
    except OSError:
        return []
    out = []
    for ln in lines:
        try:
            out.append(json.loads(ln))
        except ValueError:
            pass                              # a torn line must not hide the rest
    return out


def _log(**rec) -> int:
    """Append one call to the log; return billable calls this cycle.

    JSONL and append-only, so a crash mid-write loses one line rather than the
    history, and two processes cannot clobber each other. EVERY call is
    recorded, billable or not — a 404 costs nothing but is exactly what you
    want to see when wondering why a push did nothing.
    """
    rec = {"t": datetime.now().isoformat(timespec="seconds"), **rec}
    try:
        LOG.parent.mkdir(parents=True, exist_ok=True)
        with LOG.open("a") as f:
            f.write(json.dumps(rec) + "\n")
    except OSError:
        pass                                  # a log is not worth failing over
    return billed()


def billed() -> int:
    """Billable calls since the cycle anchor."""
    since = cycle_start().isoformat()
    return sum(1 for r in read_log() if r.get("billable") and r.get("t", "") >= since)


def _call(method: str, path: str, body: dict | None = None,
          accept: str = ACCEPT, what: str = "", **extra) -> tuple[bytes, str]:
    """One HTTP call, always logged. `what` is the human sentence in the log."""
    req = urllib.request.Request(
        f"{API}{path}", method=method,
        data=None if body is None else json.dumps(body).encode(),
        headers={"Authorization": _auth(), "Accept": accept,
                 **({"Content-Type": "application/json"} if body is not None else {})})
    ep = path.split("?")[0]
    try:
        with urllib.request.urlopen(req) as r:
            data = r.read()
            _log(what=what or ep, method=method, endpoint=ep, status=r.status,
                 billable=True, bytes=len(data), **extra)
            return data, r.headers.get_content_type()
    except urllib.error.HTTPError as e:
        detail = e.read().decode("utf-8", "replace")[:600]
        _log(what=what or ep, method=method, endpoint=ep, status=e.code,
             billable=False, error=e.reason, **extra)
        # 4xx/5xx do NOT count against the annual quota — worth saying, because
        # the natural reaction to a failed push is to stop pushing.
        raise OnshapeError(
            f"{method} {path} -> {e.code} {e.reason}\n  {detail}\n"
            "  (failed calls do not count against the API quota)") from None


def push_feature_studio(text: str, url: str) -> int:
    """Replace a Feature Studio's entire contents. One billable call.

    `sourceMicroversion` + `rejectMicroversionSkew` would make this fail rather
    than clobber a concurrent edit. Omitted on purpose: the studio is generated
    and must never be hand-edited, so overwriting is the intended behaviour and
    a skew rejection would only ever be noise.
    """
    did, wvm, wid, eid = parse_url(url)
    if wvm != "w":
        raise OnshapeError("push needs a WORKSPACE url (/w/), not a version or "
                           "microversion — those are read-only")
    _call("POST", f"/featurestudios/d/{did}/w/{wid}/e/{eid}", {"contents": text},
          what=f"push {len(text)} chars of FeatureScript", doc=did, elem=eid)
    return billed()


def create_feature_studio(name: str, cfg: str = DOC_CFG) -> tuple[str, str]:
    """Make a new Feature Studio tab; return (element id, browser URL). One call.

    Both generated studios that already exist were created by a POST typed at a
    prompt and recorded only in a config/onshape.yaml comment. Same call, kept
    where the next person needing one will find it.

    The new tab arrives with a `FeatureScript <n>;` header of its own, and that
    version comes from the DOCUMENT rather than from the current std -- so a
    generator emitting a number it picked can disagree with the tab it pushes
    into. Everything generated for this document is on 3044, which is what the
    two existing studios compile at.
    """
    import yaml
    d = yaml.safe_load(Path(cfg).read_text())
    raw, _ = _call("POST", f"/featurestudios/d/{d['document']}/w/{d['workspace']}",
                   {"name": name}, what=f"create Feature Studio tab {name!r}",
                   doc=d["document"])
    got = json.loads(raw)
    eid = got.get("id") or got.get("elementId")
    if not eid:
        raise OnshapeError(f"no element id in the reply: {sorted(got)}")
    return eid, (f"https://cad.onshape.com/documents/{d['document']}"
                 f"/w/{d['workspace']}/e/{eid}")


def shaded_view(url: str, out: Path, width: int = 1600, height: int = 1200,
                view: str = "isometric", edges: bool = True,
                bg: str | None = "white") -> tuple[Path, int]:
    """Render a Part Studio to PNG. One billable call.

    `pixelSize=0` means "fit the model to the frame" — with any other value the
    view matrix sets direction and pan only, and the zoom is yours to get wrong.
    `view` is a named view (front/top/right/isometric...) or twelve
    comma-separated numbers, a row-major 3x4 view matrix.

    NOTE this reads whatever the workspace currently regenerates to, and a push
    lands as a new microversion the Part Studio picks up on read — so
    push-then-shot is coherent, but a studio that failed to compile renders the
    error, not the old geometry.
    """
    did, wvm, wvmid, eid = parse_url(url)
    q = (f"?viewMatrix={view}&outputWidth={width}&outputHeight={height}"
         f"&pixelSize=0&edges={str(edges).lower()}&useAntiAliasing=true")
    raw, ctype = _call("GET", f"/partstudios/d/{did}/{wvm}/{wvmid}/e/{eid}/shadedviews{q}",
                       what=f"render {view} {width}x{height} -> {out.name}",
                       doc=did, elem=eid)
    if ctype.startswith("image/"):
        png = raw                             # some deployments return the image itself
    else:
        images = json.loads(raw).get("images") or []
        if not images:
            raise OnshapeError("shadedviews returned no image — is the element a "
                               "Part Studio, and does it contain any geometry?")
        png = base64.b64decode(images[0])
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(_flatten(png, bg))
    return out, billed()


def _flatten(png: bytes, bg):
    """Composite the render onto an opaque background, if one is wanted.

    `shadedviews` returns RGBA with a FULLY TRANSPARENT background — corner
    pixel (0, 0, 0, 0). That is not a setting and not a preference: every
    viewer picks its own backdrop, so the same file reads white in one and
    black in another. For a figure tracked in the repo that is a bug, because
    two people looking at `docs/measurements/cad_layout.png` disagree about
    what it shows.

    Pillow is imported lazily and its absence is not fatal — it arrives with
    matplotlib in the `dev`/`viz` extras and this module is otherwise stdlib
    only, which is deliberate (nothing here should ever be a reason the Pi
    needs a dependency).
    """
    if not bg:
        return png                      # bg=None keeps the alpha channel
    try:
        import io

        from PIL import Image
    except ImportError:
        print("  (Pillow not installed — keeping the transparent background; "
              "it will look different in different viewers)")
        return png
    im = Image.open(io.BytesIO(png))
    if im.mode not in ("RGBA", "LA"):
        return png
    flat = Image.new("RGB", im.size, bg)
    flat.paste(im, mask=im.getchannel("A"))
    buf = io.BytesIO()
    flat.save(buf, "PNG")
    return buf.getvalue()


def eval_featurescript(script: str, url: str) -> dict:
    """Compile AND RUN FeatureScript against a Part Studio. One billable call.

    Onshape derives a THROWAWAY copy of the studio's context, executes the
    script against it -- opExtrude, opBoolean, setProperty all really run --
    returns what you measured, and discards the whole thing. Verified 2026-08-24
    by building a body, counting it, and counting again in a second call: 0.

    This is the only way to find out whether generated FeatureScript is sound
    without pushing it and asking a human to look. A push CANNOT tell you: the
    contents endpoint takes any text at all, so a broken export pushes happily
    and then renders EMPTY, with no error anywhere.

    Two asymmetries worth remembering:

    - The script must be a BARE FUNCTION EXPRESSION. A `FeatureScript 3044;` +
      `import(...)` preamble is a parse error; std is implicit. See
      `cad_layout._eval_wrapper`, which does the rewrite for a whole studio.
    - A script that fails to compile still returns 200, so unlike a failed push
      it COSTS A CALL. Batch every question into one script.

    It also compiles at the CURRENT library version rather than the target
    studio's, so it proves the code is sound, not that it is sound in a tab
    pinned to something older. `libraryVersion` in the reply says which.
    """
    did, wvm, wid, eid = parse_url(url)
    raw, _ = _call("POST", f"/partstudios/d/{did}/{wvm}/{wid}/e/{eid}/featurescript",
                   {"script": script, "queries": []},
                   what=f"eval {len(script)} chars of FeatureScript",
                   doc=did, elem=eid)
    return json.loads(raw)


def notice_lines(reply: dict) -> list[str]:
    """Human-readable notices from an eval reply, worst first.

    PARSE errors carry a real line and column. SEMANTIC ones report line 0 but
    name the missing function in the message, so print the message rather than
    sending anyone to look at line 0.
    """
    out = []
    rank = {"ERROR": 0, "WARNING": 1, "INFO": 2}
    for n in sorted(reply.get("notices", []),
                    key=lambda n: rank.get(n["message"]["level"], 3)):
        m = n["message"]
        loc = (m.get("stackTrace") or [{}])[0].get("message", {})
        where = (f" line {loc['line']} col {loc['column']}"
                 if loc.get("line") else "")
        out.append(f"[{m['level']}/{m['type']}]{where}: {m['message']}")
    return out


def budget_line(total: int | None = None) -> str:
    total = billed() if total is None else total
    start = cycle_start()
    left = (date(start.year + 1, *CYCLE_ANCHOR) - date.today()).days
    # Unused calls do not roll over, so the useful framing is the daily budget
    # remaining, not the fraction spent.
    rate = (BUDGET - total) / max(left, 1)
    return (f"  {total}/{BUDGET} calls since {start:%d %b %Y} — {left} days "
            f"left, {rate:.0f}/day available (failed calls are free)")


def show_log(n: int = 40) -> str:
    """The last n calls, newest last. Failures included and marked."""
    rows = read_log()
    if not rows:
        return "no calls logged yet"
    out = [f"{'when':<17} {'st':>3} {'$':1} what"]
    for r in rows[-n:]:
        out.append(f"{r.get('t', '?')[5:].replace('T', ' '):<17} "
                   f"{r.get('status', '?'):>3} "
                   f"{'*' if r.get('billable') else ' '} {r.get('what', '?')}")
    out.append(f"({len(rows)} logged, * = billable)")
    out.append(budget_line())
    return "\n".join(out)


def check(url: str) -> str:
    """Confirm the keys and the READ scope against a document you own.

    Costs one call on success and nothing on failure, which is the right way
    round: 4xx is not billable, and a success was a call you were about to
    spend on a push anyway.

    There is no free version. Onshape 404s on a nonexistent id whether or not
    you are authenticated, and 403s on a real document identically for bogus
    credentials and for none at all — measured 2026-08-21, both directions. So
    no probe of an absent document can tell a good key from a bad one.

    WRITE scope stays unproven: a read-only key passes this and then 403s on
    the push. The first push is the only test of that.
    """
    did, _, _, _ = parse_url(url)
    try:
        _call("GET", f"/documents/{did}", what="check keys + read scope", doc=did)
    except OnshapeError as e:
        if "403" in str(e):
            raise OnshapeError(
                "403 — keys rejected, or they lack 'read your documents', or "
                "this document is not yours. Nothing was billed.") from None
        raise
    return f"keys good, read scope good. {budget_line(billed())}"


if __name__ == "__main__":
    import sys
    arg = sys.argv[1] if len(sys.argv) == 2 else None
    if arg == "--log":
        print(show_log())
    elif arg:
        try:
            print(check(arg))
        except OnshapeError as e:
            raise SystemExit(str(e))
    else:
        raise SystemExit("usage: python -m aow_sim.onshape <document url>  "
                         "# check keys: 1 call on success, 0 on failure\n"
                         "       python -m aow_sim.onshape --log           "
                         "# every call made, free")
