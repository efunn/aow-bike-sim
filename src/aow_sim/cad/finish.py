"""The finishing pass: hand features modelled in a Part Studio, replayed by
the generator. Workflow in docs/plans/cad-onshape-workflow.md, "Folding hand
edits into a generator".

    python -m aow_sim.cad.hand_edits bike_v2 --picks     # read them (a call per round)
    python -m aow_sim.cad.finish bike_v2                 # -> config/cad/finish/bike_v2.yaml

Each hand feature (fillet, chamfer, extrude, move face, revolve, and the
sketches they use) becomes one OP, replayed by Onshape's OWN feature of that
type with the dialog values it had. Only the picks are new: each edge, face
or body is found again by a POINT on it (and the body by a point inside it),
read with the rollback bar before its feature. The replay is therefore
exactly the hand model as long as the geometry under the points does not
move -- a dimension change that moves an edge drops the pick, and the
check says which (`FIN|miss`).

So this is NOT the "points computed in layout()" form the workflow asks of
a module's own finishing: the points are the hand model's, frozen. That
was the trade for 77 features and ~400 picks in one round (2026-10-28):
it encodes them exactly and at once, and every pick that stops resolving
is named rather than silently lost. A feature worth keeping through
another design round is worth moving into its generator properly.

Copies. A part that is one print with another gets the other's edits too:
the rollcage front's are copied to the rear (the same part turned 180 deg
about Z through mid-wheelbase) and, with `mirror`, each one-sided pick on a
left/right pair is offered on its twin, mirrored in X. A mirrored pick is
kept only if the check finds it on the twin (`FIN|mir`), so
`mirror_keep` in the yaml is what was found, written back by --bake.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import yaml

OUT = Path("config/cad/finish")
TRACES = Path("traces/hand_edits")
FEATURES = ("fillet", "chamfer", "extrude", "moveFace", "revolve")
# the parameters of each kind that are never worth carrying: arrays the
# dialog leaves empty
DROP = {"pointOnEdgeSettings", "vertexSettings"}
NUDGE = 0.05        # mm inside a body from a point on its boundary


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v] if n > 1e-12 else [0.0, 0.0, 0.0]


def _quantity(expr: str):
    m = re.fullmatch(r"\s*\(?([-\d.]+)\)?\s*(mm|deg|in)?\s*", expr)
    if not m:
        raise ValueError(f"cannot read the expression {expr!r}")
    return {"v": float(m.group(1)), "u": m.group(2) or ""}


def _sketch(m: dict, plane: dict) -> dict:
    """A sketch's curves in its own (x, y), mm. Onshape's sketch frame is the
    picked plane's x and normal with its origin at the WORLD origin projected
    onto the plane (checked against two sketches' faces, 2026-10-28), so the
    plane's origin is replaced by that projection."""
    n, o = plane["normal"], plane["origin"]
    d = sum(a * b for a, b in zip(n, o))
    out = {"origin": [d * c for c in n], "normal": n, "x": plane["x"], "lines": [], "circles": [], "arcs": []}
    for e in m["entities"]:
        em = e["message"]
        g = em.get("geometry", {}).get("message", {})
        con = bool(em.get("isConstruction"))
        if "pntX" in g:
            a = [(g["pntX"] + g["dirX"] * em["startParam"]) * 1000, (g["pntY"] + g["dirY"] * em["startParam"]) * 1000]
            b = [(g["pntX"] + g["dirX"] * em["endParam"]) * 1000, (g["pntY"] + g["dirY"] * em["endParam"]) * 1000]
            out["lines"].append({"a": a, "b": b, "c": con})
        elif "radius" in g:
            c, r = [g["xCenter"] * 1000, g["yCenter"] * 1000], g["radius"] * 1000
            if "startParam" not in em:
                out["circles"].append({"c": c, "r": r, "k": con})
            else:
                # param t from xDir, counter-clockwise unless `clockwise`
                xd = [g["xDir"], g["yDir"]]
                yd = [-xd[1], xd[0]] if not g.get("clockwise") else [xd[1], -xd[0]]
                at = lambda t: [c[0] + r * (math.cos(t) * xd[0] + math.sin(t) * yd[0]),
                                c[1] + r * (math.cos(t) * xd[1] + math.sin(t) * yd[1])]
                t0, t1 = em["startParam"], em["endParam"]
                out["arcs"].append({"a": at(t0), "m": at((t0 + t1) / 2), "b": at(t1), "k": con})
        elif e["typeName"] != "BTMSketchPoint":
            raise ValueError(f"{m['name']}: a sketch entity this does not draw: {e['typeName']}")
    return out


def _to_local(sk: dict, p) -> list[float]:
    x, n = sk["x"], sk["normal"]
    y = [n[1] * x[2] - n[2] * x[1], n[2] * x[0] - n[0] * x[2], n[0] * x[1] - n[1] * x[0]]
    d = [a - b for a, b in zip(p, sk["origin"])]
    return [sum(a * b for a, b in zip(d, x)), sum(a * b for a, b in zip(d, y))]


def _region_point(sk: dict, p) -> list[float]:
    """A region pick's point, moved off the sketch's curves: the readback's
    point is the region's closest point to its centroid, which for an
    annulus is ON its inner circle (between two regions). Moved NUDGE away
    from that circle's centre, it is inside the annulus."""
    u = _to_local(sk, p)
    for c in (c for c in sk["circles"] if not c["k"]):
        dx, dy = u[0] - c["c"][0], u[1] - c["c"][1]
        r = math.hypot(dx, dy)
        if abs(r - c["r"]) < 1e-3 and r > 1e-9:
            k = NUDGE / r
            q = [p[i] + k * (p[i] - _from_local(sk, c["c"])[i]) for i in range(3)]
            return q
    for ln in (ln for ln in sk["lines"] if not ln["c"]):
        (ax, ay), (bx, by) = ln["a"], ln["b"]
        L = math.hypot(bx - ax, by - ay)
        t = ((u[0] - ax) * (bx - ax) + (u[1] - ay) * (by - ay)) / (L * L)
        if 0 <= t <= 1 and abs((u[0] - ax) * (by - ay) - (u[1] - ay) * (bx - ax)) / L < 1e-3:
            raise ValueError(f"a region pick sits on a line of its sketch at {u}: move it by hand")
    return list(p)


def _from_local(sk: dict, u) -> list[float]:
    x, n = sk["x"], sk["normal"]
    y = [n[1] * x[2] - n[2] * x[1], n[2] * x[0] - n[0] * x[2], n[0] * x[1] - n[1] * x[0]]
    return [sk["origin"][i] + u[0] * x[i] + u[1] * y[i] for i in range(3)]


def _pick(e: dict) -> dict:
    """One readback entity -> what the replay finds it by."""
    k = e["kind"]
    if k == "EDGE":
        # the body: a point just inside both faces at the edge's midpoint
        b = [p - NUDGE * n for p, n in zip(e["pt"], _unit(e["n"]))] if any(e["n"]) else None
        return {"k": "E", "p": e["pt"], "b": b, "nm": e["body"]}
    if k == "FACE":
        return {"k": "F", "p": e["pt"], "n": e["n"], "b": [p - NUDGE * n for p, n in zip(e["pt"], e["n"])],
                "nm": e["body"]}
    if k == "BODY":
        return {"k": "B", "p": e["pt"], "nm": e["body"]}
    raise ValueError(f"a pick of kind {k} has no replay")


def ops_from_readback(tab: str) -> list[dict]:
    """Every hand feature of a tab as an op, in tree order."""
    tree = json.loads((TRACES / f"{tab}_features.json").read_text())
    picks = {f["id"]: f for f in json.loads((TRACES / f"{tab}_picks.json").read_text())["features"]}
    msgs = {f["message"]["featureId"]: f["message"] for f in tree["features"]}
    ops, sketches = [], {}
    for fid, f in picks.items():
        m = msgs[fid]
        if f["type"] == "newSketch":
            pl = f["picks"]["sketchPlane__0"]["entities"][0]["plane"]
            sketches[fid] = len(ops)
            ops.append({"op": "sketch", "name": f["name"], **_sketch(m, pl)})
            continue
        if f["type"] not in FEATURES:
            raise ValueError(f"{f['name']}: no replay for a {f['type']}")
        # a region pick belongs to the latest sketch whose plane holds it
        prior = [ops[i] for i in sketches.values()]
        params: dict = {}
        for p in m["parameters"]:
            pm, tn = p["message"], p["typeName"]
            pid = pm["parameterId"]
            if pid in DROP:
                if pm.get("items"):
                    raise ValueError(f"{f['name']}: {pid} is not empty")
                continue
            if tn == "BTMParameterEnum":
                params[pid] = {"e": f"{pm['enumName']}.{pm['value']}"}
            elif tn == "BTMParameterQuantity":
                params[pid] = _quantity(pm["expression"])
            elif tn == "BTMParameterBoolean":
                params[pid] = bool(pm["value"])
            elif tn == "BTMParameterQueryList":
                q = []
                for qi in range(len(pm["queries"])):
                    got = f["picks"].get(f"{pid}__{qi}")
                    if got is None:
                        raise ValueError(f"{f['name']}: pick {pid}[{qi}] was never read")
                    for e in got["entities"]:
                        if e["body"] == "UNNAMED" and e["kind"] == "FACE":
                            sk = next((s for s in reversed(prior) if _on_plane(s, e["pt"])), None)
                            if sk is None:
                                raise ValueError(f"{f['name']}: a region pick on no sketch")
                            q.append({"k": "R", "s": sk["name"], "p": _region_point(sk, e["pt"])})
                        else:
                            q.append(_pick(e))
                params[pid] = {"q": q}
            else:
                raise ValueError(f"{f['name']}: parameter {pid} is a {tn}")
        ops.append({"op": f["type"], "name": f["name"], "params": params})
    return ops


def _on_plane(sk: dict, p) -> bool:
    return abs(sum((a - b) * n for a, b, n in zip(p, sk["origin"], sk["normal"]))) < 1e-3


# --------------------------------------------------------------------------
# copies: a part that is another part's print gets its edits
# --------------------------------------------------------------------------

def _map_pick(q: dict, f) -> dict:
    """A pick through a rigid map f(point, is_direction)."""
    out = dict(q)
    for k in ("p", "b"):
        if q.get(k) is not None:
            out[k] = f(q[k], False)
    if "n" in q:
        out["n"] = f(q["n"], True)
    return out


def _map_op(op: dict, f, name: str) -> dict:
    if op["op"] == "sketch":
        return {**op, "name": name, "origin": f(op["origin"], False), "normal": f(op["normal"], True),
                "x": f(op["x"], True)}
    params = {}
    for k, v in op["params"].items():
        if isinstance(v, dict) and "q" in v:
            params[k] = {"q": [{**_map_pick(q, f), **({"s": q["s"] + " (copy)"} if q["k"] == "R" else {})}
                               for q in v["q"]]}
        else:
            params[k] = v
    return {**op, "name": name, "params": params}


def turned(wb: float):
    """The rollcage's map: 180 deg about Z through mid-wheelbase."""
    def f(p, direction):
        return [-p[0], -p[1], p[2]] if direction else [-p[0], wb - p[1], p[2]]
    return f


def mirrored_x(p, direction=False):
    """X -> -X: a point's or a direction's mirror alike."""
    return [-p[0], p[1], p[2]]


def copy_ops(ops: list[dict], select, f, suffix: str) -> list[dict]:
    """Each op whose picks all satisfy `select` (and the sketches its region
    picks draw on) repeated through f right after the original."""
    out, done = [], {}
    for op in ops:
        out.append(op)
        if op["op"] == "sketch":
            continue
        qs = [q for v in op["params"].values() if isinstance(v, dict) and "q" in v for q in v["q"]]
        if not qs or not all(select(q) for q in qs):
            continue
        for q in qs:
            if q["k"] == "R" and q["s"] not in done:
                sk = next(o for o in ops if o["op"] == "sketch" and o["name"] == q["s"])
                done[q["s"]] = True
                out.append(_map_op(sk, f, q["s"] + " (copy)"))
        out.append(_map_op(op, f, op["name"] + suffix))
    return out


def mirror_candidates(ops: list[dict], tol: float = 0.05, skip=()) -> list[dict]:
    """Each edge or face pick of a fillet or chamfer whose X-mirror is not
    already picked in the same op, offered mirrored (marked `m`). A pick on
    a body named with a word in `skip` is not offered."""
    out = []
    for op in ops:
        if op["op"] not in ("fillet", "chamfer"):
            out.append(op)
            continue
        params = dict(op["params"])
        q = list(params["entities"]["q"])
        have = [x["p"] for x in q]
        extra = []
        for x in q:
            if x["k"] not in ("E", "F") or any(w in x.get("nm", "") for w in skip):
                continue
            mp = mirrored_x(x["p"])
            if abs(x["p"][0]) < tol or any(math.dist(mp, h) < tol for h in have):
                continue
            extra.append({**_map_pick(x, mirrored_x), "m": True})
        params["entities"] = {"q": q + extra}
        out.append({**op, "params": params})
    return out


def touched(ops: list[dict]) -> set[str]:
    """The bodies some op picks on, by the name they had when read."""
    return {q["nm"] for op in ops if op["op"] != "sketch" for v in op["params"].values()
            if isinstance(v, dict) and "q" in v for q in v["q"] if q.get("nm")}


def enums(ops: list[dict]) -> list[str]:
    return sorted({v["e"] for op in ops if op["op"] != "sketch"
                   for v in op["params"].values() if isinstance(v, dict) and "e" in v})


# --------------------------------------------------------------------------
# FeatureScript: the interpreter, shared (it lives in the righting's layer,
# which every bike merges)
# --------------------------------------------------------------------------

FS = r'''
/**
 * The finishing pass (cad.finish): hand features replayed as Onshape's own
 * features with their dialog values, every pick found again by a point on
 * it, scoped to the body holding a point inside it. `S` is the solids it may
 * touch; `en` maps enum names to values; `nameOf` (the check's, a function
 * of a body; undefined in a studio, where a name cannot be read) prints
 * where each mirrored pick landed. Returns the picks that found
 * nothing and the ops that failed, as text: a dimension that moved an edge
 * drops its pick and is named here, never silently.
 */
export function finishOps(context is Context, id is Id, S is Query, ops is array, en is map, nameOf) returns array
{
    const mm = millimeter;
    const V = function(a) returns Vector { return vector(a[0], a[1], a[2]) * mm; };
    const U = function(a) returns Vector { return vector(a[0], a[1], a[2]); };
    const check = nameOf != undefined;
    const TOL = 0.01 * mm;
    // the one of several candidates nearest p; of faces (n given), the one
    // whose normal there is n's first -- a face's point can sit on an edge
    // it shares with another
    const nearest = function(cands is Query, p is Vector, n) returns Query
    {
        const es = evaluateQuery(context, cands);
        if (size(es) < 2)
            return cands;
        var best = es[0];
        var score = inf;
        for (var e in es)
        {
            const d = evDistance(context, { "side0" : p, "side1" : e });
            var sc = d.distance / mm;
            if (n != undefined)
            {
                const t = evFaceTangentPlane(context, { "face" : e, "parameter" : d.sides[1].parameter });
                sc = sc + 10 * (1 - dot(t.normal, n));
            }
            if (sc < score)
            {
                score = sc;
                best = e;
            }
        }
        return best;
    };
    var bad = [];
    var sk = {};
    var sketches = [];
    for (var i = 0; i < size(ops); i += 1)
    {
        const op = ops[i];
        const oid = id + ("h" ~ i);
        if (op.op == "sketch")
        {
            const s = newSketchOnPlane(context, oid, { "sketchPlane" : plane(V(op.origin), U(op.normal), U(op.x)) });
            const p2 = function(a) returns Vector { return vector(a[0], a[1]) * mm; };
            for (var k = 0; k < size(op.lines); k += 1)
                skLineSegment(s, "l" ~ k, { "start" : p2(op.lines[k].a), "end" : p2(op.lines[k].b), "construction" : op.lines[k].c });
            for (var k = 0; k < size(op.circles); k += 1)
                skCircle(s, "c" ~ k, { "center" : p2(op.circles[k].c), "radius" : op.circles[k].r * mm, "construction" : op.circles[k].k });
            for (var k = 0; k < size(op.arcs); k += 1)
                skArc(s, "a" ~ k, { "start" : p2(op.arcs[k].a), "mid" : p2(op.arcs[k].m), "end" : p2(op.arcs[k].b), "construction" : op.arcs[k].k });
            skSolve(s);
            sk[op.name] = oid;
            sketches = append(sketches, oid);
            continue;
        }
        var def = {};
        for (var k in keys(op.params))
        {
            const v = op.params[k];
            if (v is boolean)
                def[k] = v;
            else if (v.e != undefined)
                def[k] = en[v.e];
            else if (v.u != undefined)
            {
                // (a chained ?: does not nest right-to-left here)
                var x = v.v;
                if (v.u == "mm")
                    x = v.v * mm;
                else if (v.u == "deg")
                    x = v.v * degree;
                else if (v.u == "in")
                    x = v.v * inch;
                def[k] = x;
            }
            else
            {
                var qs = [];
                for (var j = 0; j < size(v.q); j += 1)
                {
                    const q = v.q[j];
                    // within TOL of the point, not ON it: the points were read to 1e-5
                    // mm, and a sketch's plane rebuilt from rounded numbers is 1e-4 off
                    // its region points (qContainsPoint found none of those, 2026-10-28)
                    var r = qNothing();
                    if (q.k == "R")
                        r = nearest(qWithinRadius(qSketchRegion(sk[q.s]), V(q.p), TOL), V(q.p), undefined);
                    else if (q.k == "B")
                        r = qContainsPoint(S, V(q.p));
                    else
                    {
                        var scope = S;
                        if (q.b != undefined)
                        {
                            const bq = qContainsPoint(S, V(q.b));
                            if (size(evaluateQuery(context, bq)) == 1)
                                scope = bq;
                        }
                        const t = q.k == "E" ? EntityType.EDGE : EntityType.FACE;
                        r = nearest(qWithinRadius(qOwnedByBody(scope, t), V(q.p), TOL), V(q.p),
                                    q.k == "F" ? U(q.n) : undefined);
                    }
                    const n = size(evaluateQuery(context, r));
                    if (check && q.m == true)
                        println("FIN|mir|" ~ i ~ "|" ~ k ~ "|" ~ j ~ "|" ~ n ~ "|"
                                ~ (n == 1 ? nameOf(qOwnerBody(r)) : "-"));
                    if (n == 0)
                    {
                        if (q.m != true)
                            bad = append(bad, op.name ~ " " ~ k ~ "[" ~ j ~ "]");
                        continue;
                    }
                    qs = append(qs, r);
                }
                def[k] = qUnion(qs);
            }
        }
        var ok = true;
        try
        {
            if (op.op == "fillet")
                fillet(context, oid, def);
            else if (op.op == "chamfer")
                chamfer(context, oid, def);
            else if (op.op == "extrude")
                extrude(context, oid, def);
            else if (op.op == "moveFace")
                moveFace(context, oid, def);
            else if (op.op == "revolve")
                revolve(context, oid, def);
        }
        catch (e)
        {
            ok = false;
            bad = append(bad, op.name ~ " FAILED: " ~ toString(e));
        }
        if (check)
            println("FIN|op|" ~ i ~ "|" ~ op.name ~ "|" ~ (ok ? "ok" : "FAILED"));
    }
    // the sketches, drawn for the regions only: their own ids, after
    // everything (an id under a sketch's would split its history)
    for (var k = 0; k < size(sketches); k += 1)
        opDeleteBodies(context, id + ("sd" ~ k), { "entities" : qCreatedBy(sketches[k], EntityType.BODY) });
    if (check)
        for (var b in bad)
            println("FIN|miss|" ~ b);
    return bad;
}
'''


def fs_data(name: str, ops: list[dict]) -> str:
    """The ops and the enum map, as `export const` declarations."""
    from .righting import _fs
    en = "{ " + ", ".join(f'"{e}" : {e}' for e in enums(ops)) + " }"
    return f"export const {name} = {_fs(ops)};\n\nexport const {name}_EN = {en};\n"


# --------------------------------------------------------------------------
# baking: readback -> config/cad/finish/<tab>.yaml
# --------------------------------------------------------------------------

def twin(name: str) -> str:
    """The body that is `name` mirrored in X: a left part's right one (the
    wing stubs swap knuckle and rocker with the side: R knuckle's mirror is
    L rocker), or itself for a part symmetric about the centreline."""
    swap = {"L": "R", "R": "L", "[print X+]": "[print X-]", "[print X-]": "[print X+]"}
    if "stub" in name:
        swap |= {"(knuckle)": "(rocker)", "(rocker)": "(knuckle)"}
    pat = r"\b[LR]\b|" + "|".join(re.escape(k) for k in swap if len(k) > 1)
    return re.sub(pat, lambda m: swap[m.group(0)], name)


def bake(tab: str, wb: float | None = None, mirror: bool = False, no_mirror=("rollcage", "power holder")) -> dict:
    """The tab's ops: the readback, the rollcage front's copied to the rear
    (wb: the wheelbase, mm), and mirror candidates on every other part."""
    ops = ops_from_readback(tab)
    if wb is not None:
        ops = copy_ops(ops, lambda q: q["k"] == "R" or q.get("nm") == "rollcage front", turned(wb), " (rear)")
    if mirror:
        ops = mirror_candidates(ops, skip=no_mirror)
    # a sketch nothing extrudes or revolves (one was left drawn on the fork)
    used = {q["s"] for o in ops if o["op"] != "sketch" for v in o["params"].values()
            if isinstance(v, dict) and "q" in v for q in v["q"] if q["k"] == "R"}
    ops = [o for o in ops if o["op"] != "sketch" or o["name"] in used]
    return {"tab": tab, "ops": ops}


def keep_mirrors(ops: list[dict], found: dict[tuple, str]) -> tuple[list[dict], list[str]]:
    """Drop each mirror candidate the check did not find on its twin
    (found: (op index, param, pick index) -> the body it landed in)."""
    out, kept = [], []
    for i, op in enumerate(ops):
        if op["op"] not in ("fillet", "chamfer"):
            out.append(op)
            continue
        q = []
        for j, x in enumerate(op["params"]["entities"]["q"]):
            if not x.get("m"):
                q.append(x)
                continue
            body = found.get((i, "entities", j))
            if body is not None and body == twin(x["nm"]):
                q.append({k: v for k, v in x.items() if k != "m"} | {"nm": body, "mirror": True})
                kept.append(f"{op['name']}: {body} at {[round(v, 2) for v in x['p']]}")
        out.append({**op, "params": {**op["params"], "entities": {"q": q}}})
    return out, kept


def verdict(rows: list[list[str]], tab: str, strip: str = "") -> tuple[bool, str]:
    """The check's word on the finishing pass: every pick that found
    nothing (FAIL), each mirror candidate's landing, and each body's volume
    against the hand model's (traces/hand_edits/<tab>_final_bodies.txt, read
    from the tab with its hand features on) -- the replay IS that model, so
    a body that differs is one an op did not reproduce, or one a copy or a
    mirror added to. `strip`: a prefix the check puts on body names."""
    out, ok = [], True
    miss = [r[2] for r in rows if r[:2] == ["FIN", "miss"]]
    if miss:
        ok = False
        out.append(f"  finishing: {len(miss)} picks or ops failed  FAIL")
        out += [f"    {m}" for m in miss]
    mir = [r for r in rows if r[:2] == ["FIN", "mir"]]
    if mir:
        hit = sum(1 for r in mir if r[5] == "1")
        out.append(f"  finishing: {hit} of {len(mir)} mirror candidates found an edge (re-bake with --keep to fix them)")
    ref = TRACES / f"{tab}_final_bodies.txt"
    if ref.exists():
        want: dict[str, list[float]] = {}
        for l in ref.read_text().splitlines():
            r = l.split("|")
            if r[0] == "BODY":
                want.setdefault(r[1], []).append(float(r[8]))
        got: dict[str, list[float]] = {}
        for r in rows:
            if r[0] == "BODY":
                n = r[1].split(": ", 1)[1] if strip and r[1].startswith(tuple(strip)) and ": " in r[1] else r[1]
                got.setdefault(n, []).append(float(r[8]))
        diff = []
        for n in sorted(set(want) | set(got)):
            a, b = sorted(want.get(n, [])), sorted(got.get(n, []))
            if len(a) != len(b) or any(abs(x - y) > max(0.5, 1e-4 * x) for x, y in zip(a, b)):
                diff.append(f"    {n:28} hand model {', '.join(f'{x:.1f}' for x in a) or '-':>22}   "
                            f"replay {', '.join(f'{y:.1f}' for y in b) or '-'}")
        out.append(f"  finishing: {len(diff)} bodies' volumes differ from the hand model (mm3)"
                   + ("" if diff else "  ok"))
        out += diff
    return ok, "\n".join(out)


def mirror_hits(rows: list[list[str]]) -> dict[tuple, str]:
    """(op index, param, pick index) -> the body a mirror candidate landed in."""
    return {(int(r[2]), r[3], int(r[4])): r[6] for r in rows
            if r[:2] == ["FIN", "mir"] and r[5] == "1"}


def dump(tab: str, data: dict) -> Path:
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"{tab}.yaml"

    def rnd(v):
        if isinstance(v, float):
            return round(v, 5)
        if isinstance(v, list):
            return [rnd(x) for x in v]
        if isinstance(v, dict):
            return {k: rnd(x) for k, x in v.items()}
        return v
    head = (f"# GENERATED by python -m aow_sim.cad.finish {tab}, from the hand features read back by\n"
            f"# python -m aow_sim.cad.hand_edits {tab} --picks. Points in mm, the studio's frame.\n"
            "# Each op is replayed by Onshape's own feature of that kind (cad.finish.FS).\n")
    path.write_text(head + yaml.safe_dump(rnd(data), sort_keys=False, default_flow_style=None, width=110))
    return path


def load(tab: str) -> list[dict]:
    path = OUT / f"{tab}.yaml"
    return yaml.safe_load(path.read_text())["ops"] if path.exists() else []


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tab", help="the Part Studio the hand features were read from (cad.hand_edits --picks)")
    ap.add_argument("--wheelbase", type=float, help="copy the rollcage front's ops to the rear (mm)")
    ap.add_argument("--mirror", action="store_true", help="offer each one-sided pick on its twin")
    ap.add_argument("--keep", metavar="CHECK_CONSOLE",
                    help="keep only the mirror candidates this check console found on their twin")
    args = ap.parse_args()
    data = bake(args.tab, args.wheelbase, args.mirror)
    if args.keep:
        rows = [l.split("|") for l in Path(args.keep).read_text().splitlines()]
        data["ops"], kept = keep_mirrors(data["ops"], mirror_hits(rows))
        print(f"kept {len(kept)} mirrored picks:")
        for k in kept:
            print(f"  {k}")
    n = sum(1 for o in data["ops"] if o["op"] != "sketch")
    m = sum(1 for o in data["ops"] if o["op"] != "sketch" for v in o["params"].values()
            if isinstance(v, dict) and "q" in v for q in v["q"] if q.get("m"))
    print(f"{n} ops, {m} mirror candidates -> {dump(args.tab, data)}")


if __name__ == "__main__":
    main()
