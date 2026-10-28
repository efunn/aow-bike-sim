"""Read back the hand edits in a generated module's Part Studio, so they can
be folded into its generator. TWO billable calls, whatever the tree's size.

    python -m aow_sim.cad.hand_edits steering      # an onshape.yaml tab name

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

    python -m aow_sim.cad.hand_edits bike_v2 --picks

`--picks` reads where every hand feature's picks ARE, exactly: each edge's
midpoint, ends and the normals of the faces it joins, each face's point and
normal, each body's interior point, each sketch plane. A pick is read with
the rollback bar as near before its own feature as the rounds get: round
one rolls back to the first hand feature, and a pick that does not exist
there yet (an edge a chamfer before it made) waits for a later round, its
bar before the first feature still unread. One billable call per round (bike_v2, 77 features: a handful).
Writes traces/hand_edits/<tab>_picks.json, which `cad.finish` turns into the
generator's replay data.
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
            lines.append(f"    -> {part}: {len(fs)} faces")
            # every face: an edit that picks several edges is only pinned
            # down by all of them (one example left the drive's second
            # chamfer edge ambiguous, 2026-10-11)
            lines += [f"         {area:>8} mm2  {lo} .. {hi}" for area, lo, hi in fs]
    text = "\n".join(lines)
    (OUT / f"{tab}_hand_edits.txt").write_text(text + "\n")
    return text


PICKS = '''function(context is Context, queries)
{
    const mm = millimeter;
    const s = function(v) returns string
    {
        return toString(roundToPrecision(v[0] / mm, 5)) ~ "," ~ toString(roundToPrecision(v[1] / mm, 5))
               ~ "," ~ toString(roundToPrecision(v[2] / mm, 5));
    };
    const u = function(v) returns string
    {
        return toString(roundToPrecision(v[0], 6)) ~ "," ~ toString(roundToPrecision(v[1], 6))
               ~ "," ~ toString(roundToPrecision(v[2], 6));
    };
    const nm = function(b) returns string
    {
        const n = getProperty(context, { "entity" : b, "propertyType" : PropertyType.NAME });
        return n == undefined ? "UNNAMED" : n;
    };
    const onFace = function(f, p) returns map
    {
        const d = evDistance(context, { "side0" : p, "side1" : f });
        const t = evFaceTangentPlane(context, { "face" : f, "parameter" : d.sides[1].parameter });
        return { "p" : d.sides[1].point, "n" : t.normal };
    };
    for (var k in keys(queries))
    {
        const es = evaluateQuery(context, queries[k]);
        println("N|" ~ k ~ "|" ~ size(es));
        for (var e in es)
        {
            if (size(evaluateQuery(context, qEntityFilter(e, EntityType.EDGE))) > 0)
            {
                const l = evEdgeTangentLine(context, { "edge" : e, "parameter" : 0.5 });
                const a = evEdgeTangentLine(context, { "edge" : e, "parameter" : 0 }).origin;
                const b = evEdgeTangentLine(context, { "edge" : e, "parameter" : 1 }).origin;
                var n = vector(0, 0, 0);
                for (var f in evaluateQuery(context, qAdjacent(e, AdjacencyType.EDGE, EntityType.FACE)))
                    n = n + onFace(f, l.origin).n;
                println("E|" ~ k ~ "|EDGE|" ~ nm(qOwnerBody(e)) ~ "|" ~ s(l.origin) ~ "|" ~ u(n) ~ "|" ~ s(a)
                        ~ "|" ~ s(b) ~ "|" ~ u(l.direction));
            }
            else if (size(evaluateQuery(context, qEntityFilter(e, EntityType.FACE))) > 0)
            {
                const pl = try silent(evPlane(context, { "face" : e }));
                const c = evApproximateCentroid(context, { "entities" : e });
                const q = onFace(e, c);
                var tail = "|-|-|-";
                if (pl != undefined)
                    tail = "|" ~ s(pl.origin) ~ "|" ~ u(pl.normal) ~ "|" ~ u(pl.x);
                println("E|" ~ k ~ "|FACE|" ~ nm(qOwnerBody(e)) ~ "|" ~ s(q.p) ~ "|" ~ u(q.n) ~ tail);
            }
            else if (size(evaluateQuery(context, qEntityFilter(e, EntityType.BODY))) > 0)
            {
                var best = undefined;
                var area = 0 * mm * mm;
                for (var f in evaluateQuery(context, qOwnedByBody(e, EntityType.FACE)))
                {
                    const ar = evArea(context, { "entities" : f });
                    if (ar > area)
                    {
                        area = ar;
                        best = f;
                    }
                }
                const q = onFace(best, evApproximateCentroid(context, { "entities" : best }));
                println("E|" ~ k ~ "|BODY|" ~ nm(e) ~ "|" ~ s(q.p - 0.05 * mm * q.n) ~ "|" ~ u(q.n));
            }
            else
                println("E|" ~ k ~ "|VERTEX|" ~ nm(qOwnerBody(e)) ~ "|" ~ s(evVertexPoint(context, { "vertex" : e })));
        }
    }
}'''


def _pick_keys(tree: dict) -> tuple[list[dict], dict[str, list[str]], dict[str, int]]:
    """Every hand feature, and every pick in it as key -> geometry ids, with
    the tree index of the feature it belongs to."""
    feats = [f["message"] for f in tree["features"]]
    gen = [i for i, m in enumerate(feats) if str(m.get("featureType", "")).startswith(GENERATED)]
    first = (gen[-1] + 1) if gen else 0
    hand, keys, at = [], {}, {}
    for i in range(first, len(feats)):
        m = feats[i]
        if m.get("name") in SKIP:
            continue
        hand.append({"idx": i, "id": m["featureId"], "name": m["name"], "type": m["featureType"]})
        for p in m.get("parameters", []):
            pm = p["message"]
            for qi, q in enumerate(pm.get("queries", [])):
                ids = q["message"].get("geometryIds") or []
                if ids:
                    k = f"f{i}__{pm['parameterId']}__{qi}"
                    keys[k], at[k] = ids, i
    return hand, keys, at


def _parse_pick(row: list[str]) -> dict:
    v = lambda t: [float(x) for x in t.split(",")]
    kind = row[2]
    out = {"kind": kind, "body": row[3], "pt": v(row[4])}
    if kind == "EDGE":
        out.update(n=v(row[5]), a=v(row[6]), b=v(row[7]), dir=v(row[8]))
    elif kind == "FACE":
        out["n"] = v(row[5])
        if row[6] != "-":
            out["plane"] = {"origin": v(row[6]), "normal": v(row[7]), "x": v(row[8])}
    elif kind == "BODY":
        out["n"] = v(row[5])
    return out


def picks(tab: str, max_rounds: int = 40, resume: bool = False) -> dict:
    """Read every hand feature's picks at a rollback bar as close before
    the feature as a few rounds get. One billable call per round."""
    url = onshape.resolve(tab, tab)
    did, wvm, wid, eid = onshape.parse_url(url)
    raw, _ = onshape._call("GET", f"/partstudios/d/{did}/w/{wid}/e/{eid}/features",
                           what=f"read {tab} feature tree (picks)", doc=did, elem=eid)
    tree = json.loads(raw)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{tab}_features.json").write_text(json.dumps(tree, indent=1))
    hand, keys, at = _pick_keys(tree)
    got: dict[str, dict] = {}
    rb = hand[0]["idx"] if hand else 0
    prev = OUT / f"{tab}_picks.json"
    if resume and prev.exists():
        # carry on from the last run's rounds: only what is still unread
        for f in json.loads(prev.read_text())["features"]:
            for k, v in f["picks"].items():
                if v is not None and f"f{f['idx']}__{k}" in keys:
                    got[f"f{f['idx']}__{k}"] = v
        rb = min((at[k] for k in keys if k not in got), default=rb)
    for rnd in range(max_rounds):
        # a resumed run re-reads only what is unread: what it has came from a
        # bar at least as close as this one
        todo = {k: ids for k, ids in keys.items() if at[k] >= rb and not (resume and k in got)}
        body = {"script": PICKS, "queries": [{"key": k, "value": ids} for k, ids in todo.items()]}
        raw, _ = onshape._call("POST", f"/partstudios/d/{did}/{wvm}/{wid}/e/{eid}/featurescript?rollbackBarIndex={rb}",
                               body, what=f"read {tab} picks, rollback {rb} ({len(todo)} picks)", doc=did, elem=eid)
        rep = json.loads(raw)
        for n in onshape.notice_lines(rep):
            print(f"  {n}")
        rows: dict[str, list] = defaultdict(list)
        for line in (rep.get("console") or "").splitlines():
            p = line.split("|")
            if p[0] == "E":
                rows[p[1]].append(_parse_pick(p))
        for k, r in rows.items():
            # nearest rollback before its own feature wins: later rounds overwrite
            got[k] = {"rb": rb, "entities": r}
        missing = sorted({at[k] for k in keys if k not in got})
        print(f"  round {rnd + 1}: rollback {rb}, {len(rows)} of {len(todo)} read; "
              f"{sum(1 for k in keys if k not in got)} picks in {len(missing)} features still unread")
        nxt = [i for i in missing if i > rb]
        if not nxt:
            break
        rb = nxt[0]
    out = {"tab": tab, "features": [{**h, "picks": {k.split("__", 1)[1]: got.get(k) for k in keys if at[k] == h["idx"]}}
                                    for h in hand]}
    (OUT / f"{tab}_picks.json").write_text(json.dumps(out, indent=1))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tab", help="the module's Part Studio, as named in config/cad/onshape.yaml tabs:")
    ap.add_argument("--picks", action="store_true",
                    help="read every pick's location at a rollback before its feature (a call per round)")
    ap.add_argument("--resume", action="store_true",
                    help="with --picks: keep the last run's reads, read only what is still unread")
    args = ap.parse_args()
    if args.picks:
        picks(args.tab, resume=args.resume)
        print(f"-> {OUT}/{args.tab}_picks.json")
    else:
        print(read(args.tab))
        print(f"-> {OUT}/{args.tab}_hand_edits.txt")
    print(onshape.budget_line())


if __name__ == "__main__":
    main()
