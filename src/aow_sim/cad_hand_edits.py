"""Read back the hand edits in a generated module's Part Studio, so they can
be folded into its generator. TWO billable calls, whatever the tree's size.

    python -m aow_sim.cad_hand_edits steering      # an onshape.yaml tab name

1. GET .../features: the feature tree. Each feature after the generated one
   is a hand edit; its dialog values (widths, radii, depths, end types) and
   its sketches' geometry come back here, but its picked edges and faces
   only as Onshape ids, which say nothing about WHERE.
2. One eval in that Part Studio: for every hand feature, the faces it
   created, with the part that owns them, the area and the bounding box,
   in the module frame. That is what pins each edit to a part and an edge
   of the generated geometry.

Writes traces/hand_edits/<tab>_features.json (the raw tree) and
<tab>_hand_edits.txt (one line per feature: type, name, values, then the
faces it made), and prints the summary. Reading it back changes nothing.

The workflow this serves is in docs/plans/cad-onshape-workflow.md, "Folding
hand edits into a generator".
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

from . import onshape

OUT = Path("traces/hand_edits")
GENERATED = ("aow", "Aow")          # featureType prefix of the repo's custom features
SKIP = {"Origin", "Top", "Front", "Right"}

EVAL = '''function(context is Context, queries)
{
    const mm = millimeter;
    const r1 = function(v) returns string { return toString(roundToPrecision(v / mm, 2)); };
    for (var f in %L%)
    {
        for (var fc in evaluateQuery(context, qCreatedBy(makeId(f[0]), EntityType.FACE)))
        {
            const n = getProperty(context, { "entity" : qOwnerBody(fc), "propertyType" : PropertyType.NAME });
            const bb = evBox3d(context, { "topology" : fc, "tight" : true });
            println("F|" ~ f[1] ~ "|" ~ n ~ "|" ~ toString(roundToPrecision(evArea(context, { "entities" : fc }) / (mm * mm), 1))
                ~ "|" ~ r1(bb.minCorner[0]) ~ "," ~ r1(bb.minCorner[1]) ~ "," ~ r1(bb.minCorner[2])
                ~ "|" ~ r1(bb.maxCorner[0]) ~ "," ~ r1(bb.maxCorner[1]) ~ "," ~ r1(bb.maxCorner[2]));
        }
    }
}'''


def _values(m: dict) -> dict:
    """The dialog values worth reading: expressions, enums, booleans, pick counts."""
    out = {}
    for p in m.get("parameters", []):
        pm = p["message"]
        pid = pm.get("parameterId")
        if "expression" in pm:
            out[pid] = pm["expression"]
        elif "queries" in pm:
            out[pid] = f"{len(pm['queries'])} picks"
        elif "value" in pm and not isinstance(pm["value"], (dict, list)):
            out[pid] = pm["value"]
    return out


def _sketch(m: dict) -> list[str]:
    rows = []
    for e in m.get("entities", []):
        em = e["message"]
        g = em.get("geometry", {}).get("message", {})
        rows.append(f"    {e['typeName']} " + ", ".join(
            f"{k} {v * 1000:.2f}" for k, v in g.items() if isinstance(v, float)
            and k not in ("xDir", "yDir", "dirX", "dirY")))
    for c in m.get("constraints", []):
        cm = c["message"]
        if cm.get("constraintType") in ("DISTANCE", "DIAMETER", "RADIUS", "LENGTH", "ANGLE"):
            vals = [p["message"].get("expression") for p in cm.get("parameters", [])
                    if p["message"].get("expression")]
            rows.append(f"    {cm['constraintType']} {', '.join(vals)}")
    return rows


def read(tab: str) -> str:
    url = onshape.resolve(tab, tab)
    did, _, wid, eid = onshape.parse_url(url)
    raw, _ = onshape._call("GET", f"/partstudios/d/{did}/w/{wid}/e/{eid}/features",
                           what=f"read {tab} feature tree (hand edits)", doc=did, elem=eid)
    tree = json.loads(raw)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{tab}_features.json").write_text(json.dumps(tree, indent=1))
    states = onshape.feature_states(tree)
    feats = [f["message"] for f in tree["features"]]
    gen = [i for i, m in enumerate(feats) if str(m.get("featureType", "")).startswith(GENERATED)]
    hand = feats[(gen[-1] + 1) if gen else 0:]
    hand = [m for m in hand if m.get("name") not in SKIP]
    lines = [f"{tab}: {len(hand)} hand features after "
             f"{feats[gen[-1]]['name'] if gen else '(no generated feature found)'}"]
    if not hand:
        return "\n".join(lines)
    lst = "[" + ", ".join(f'["{m["featureId"]}", {json.dumps(m["name"])}]' for m in hand) + "]"
    rep = onshape.eval_featurescript(EVAL.replace("%L%", lst), url)
    for n in onshape.notice_lines(rep):
        lines.append(f"  {n}")
    faces = defaultdict(list)
    for l in (rep.get("console") or "").splitlines():
        p = l.split("|")
        if p[0] == "F":
            faces[p[1]].append(p[2:])
    for m in hand:
        v = _values(m)
        keep = {k: x for k, x in v.items() if k in (
            "entities", "width", "radius", "depth", "endBound", "operationType",
            "oppositeDirection", "angle", "chamferType", "sketchPlane", "booleanScope")}
        lines.append(f"{m['featureType']:10} {m['name']:14} {states.get(m['featureId'], '?'):7} {keep}")
        if m["featureType"] == "newSketch":
            lines += _sketch(m)
        by_part = defaultdict(list)
        for part, area, lo, hi in faces.get(m["name"], []):
            by_part[part].append((area, lo, hi))
        for part, fs in by_part.items():
            lines.append(f"    -> {part}: {len(fs)} faces, e.g. {fs[0][1]} .. {fs[0][2]}")
    text = "\n".join(lines)
    (OUT / f"{tab}_hand_edits.txt").write_text(text + "\n")
    return text


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tab", help="the module's Part Studio, as named in config/onshape.yaml tabs:")
    args = ap.parse_args()
    print(read(args.tab))
    print(f"-> {OUT}/{args.tab}_hand_edits.txt")
    print(onshape.budget_line())


if __name__ == "__main__":
    main()
