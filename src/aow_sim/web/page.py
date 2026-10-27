"""Static pages built as a template plus one JSON data blob.

The analysis pages (`analysis/mechanism/swing_stepthrough.py`,
`analysis/policy/drivetrain_trace.py`) compute their data and pour it into an
HTML template at `<script id="data" type="application/json">/*DATA*/</script>`.
Every word a reader sees lives in the template; the data is numbers and ids.

So a wording change needs no recompute: `cached()` reads the data back out of
the page last written, and `fill()` pours it into the edited template. That is
what each generator's `--from-cache` does, and what `scripts/site.sh build`
runs.

Ids (policy exports, bike runs, designs) are shown under the names in
`site/labels.yaml`; `label()` falls back to the id itself, and warns once, so
a new run shows up unnamed rather than missing.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[3]
LABELS = REPO / "site" / "labels.yaml"
TAG = '<script id="data" type="application/json">'
SLOT = TAG + "/*DATA*/</script>"

_labels: dict | None = None
_warned: set = set()


def label(kind: str, key: str) -> str:
    """The display name for `key` under `kind` in site/labels.yaml."""
    global _labels
    if _labels is None:
        _labels = yaml.safe_load(LABELS.read_text()) if LABELS.exists() else {}
    name = (_labels.get(kind) or {}).get(key)
    if name is None:
        if (kind, key) not in _warned:
            print(f"site/labels.yaml: no {kind} label for {key!r}; showing the id", file=sys.stderr)
            _warned.add((kind, key))
        return key
    return name


def fill(template: Path, data: dict, out: Path) -> None:
    page = template.read_text(encoding="utf-8")
    if page.count(SLOT) != 1:
        raise SystemExit(f"{template.name}: expected exactly one data tag")
    blob = json.dumps(data, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(page.replace(TAG + "/*DATA*/", TAG + blob), encoding="utf-8")
    try:
        shown = out.relative_to(REPO)
    except ValueError:
        shown = out
    print(f"wrote {shown} ({out.stat().st_size / 1e6:.1f} MB)")


def cached(page: Path) -> dict:
    """The data blob of a page `fill` wrote."""
    if not page.exists():
        raise SystemExit(f"{page}: no page to read the data from; render it once without --from-cache")
    s = page.read_text(encoding="utf-8")
    i = s.index(TAG) + len(TAG)
    return json.loads(s[i:s.index("</script>", i)])
