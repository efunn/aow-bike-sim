# Text to CAD: the Onshape / FeatureScript workflow

How `aow_sim.cad_layout` gets the simulator's geometry into a drawn bike, what
the platform will and will not let us do, and the things that cost a day to
find out. Started 2026-08-18; this section of the record covers up to
2026-08-19.

The export itself is `docs/measurements/cad_layout.fs`, generated. Never edit
it — the regeneration command is in its header.

---

## The shape of it

One **Feature Studio** (a code tab) holding **several features**, inserted into
a Part Studio as separate nodes:

| feature | what it does |
|---|---|
| `AOW layout variables` | `setVariable` for every coordinate, so a sketch can reference `#aow_servo_steer_z`. Draws nothing, cannot fail. Insert first, once. |
| `AOW drivetrain` / `steering` / `servos` / `mount` / `electronics` / `righting` | envelopes, origin points and axis planes for **one group** |
| `AOW planes` | one print plane (the fork's) and four belt-clearance planes, two per side. No tickboxes — everything in it is a plane already |
| `AOW four-bar sketch` | the righting linkage as construction geometry |
| `AOW bike layout` | the superseded all-in-one node, kept so documents that already have it keep their geometry |

The per-group split is not cosmetic. A tree node can be **renamed**,
**suppressed**, and **reordered**; a checkbox inside one feature can do none of
those. It also shrinks the blast radius: a runtime error aborts the whole
feature it is in, and the first version of this lost the published variables to
a bug in the envelope code because both lived in one function.

Groups are derived from the data — `groups = dict.fromkeys(it["group"] ...)` —
so adding a group to `build()` adds a feature. There is no per-group code.

## What can be named, and what cannot

- **Bodies: yes**, via `setProperty(..., PropertyType.NAME, ...)`. Every
  envelope and origin point carries its component name.
- **Planes and mate connectors: no, and there is no workaround.** They carry no
  metadata. The UI derives their names from the feature that made them, and
  that derivation is hardcoded to the feature type literally called `cPlane` —
  rename that feature and even Onshape's own naming stops. Both the filtered
  query (silently names nothing) and the unfiltered one (throws, taking the
  plane with it) were dead ends for the same reason.
- A property **the user has set by hand can never afterwards be overwritten**
  from FeatureScript. Rename a generated part in the UI and later
  regenerations silently stop renaming it. Reset under part → properties.

So a plane is always "Plane N" under whatever feature drew it, and the two
things that work are drawing fewer of them per feature and renaming the
feature.

## Query variables

`setQueryVariable(context, name, query)` — release 1.203 and later — publishes
a **named selection** that downstream features consume in a selection or plane
field. This is the mechanism that actually solves the naming problem: you
cannot name the plane, but you can name the *reference* to it, and that
reference survives the geometry being regenerated underneath it.

What the export publishes:

| name | what |
|---|---|
| `aow_q_<component>` | the envelope body |
| `aow_q_<component>_point` | the origin point |
| `aow_q_<component>_plane` | the axis or print plane |
| `aow_q_<component>_case_holes_<face>` | a 4-point case hole pattern |
| `aow_q_fourbar` | the four-bar sketch edges |

The `plane_*` items therefore come out as `#aow_q_plane_fork_print_plane` and
so on — the doubled word is the item's own `plane_` prefix meeting the `_plane`
suffix the helper appends. Ugly, stable, left alone.

**They do not appear in the Variable table.** That table lists variables owned
by Variable features; `setVariable` from a custom feature lands there, and
`setQueryVariable` does not. The queries surface only where they are
consumable. There is no browser, so the generated `.fs` is the index —
`grep setQueryVariable docs/measurements/cad_layout.fs`.

`--no-query-vars` regenerates without them. Worth knowing why the flag exists:
an unknown function in FeatureScript is a **compile** error, which takes down
the entire Feature Studio rather than one feature, so a signature we could not
verify from the docs needed an escape hatch. It compiled first time.

## Reference stability across a regeneration

Onshape does not do SolidWorks-style topological name matching. **Entity ids
are a deterministic function of the id of the operation that created them.**
Consequences, in the order they matter:

- **Order of operations inside the script is irrelevant**, because sub-ids are
  keyed by component *name* — `id + ("solid_" ~ name)` — not by a loop counter.
  This is the single most important thing to preserve. With `id + i`, inserting
  a component renumbers everything after it and silently rebinds downstream
  references to different geometry.
- **Changing a number is free.** New position, new radius: same ids, geometry
  moves, downstream features follow. This is why pasting a regenerated studio
  over the old one has been safe every time.
- **Changing a key breaks loudly.** Rename a component, drop one, or flip its
  `shape`, and the ids it produced cease to exist. Downstream features go red
  with a missing reference. That is the good failure.
- Same for variables: `#aow_payload_pi_z` breaks visibly on a rename.
- The one exception to "keyed by name" is a hole *pattern*, whose four corners
  are indexed. Safe, because the four corners of a rectangle cannot be
  reordered without the pattern itself changing.

The genuinely fragile part is that there is **no version pinning**: the Feature
Studio lives in the same workspace as the Part Studio, so every paste
propagates instantly with no rollback but undo. Putting the generated studio in
its own document and importing it as a versioned linked document would fix
that, at the cost of a version per regeneration. Not done; noted.

## Getting the script into Onshape

Copy-paste today. The automated path, if it is ever worth it:

```
POST /api/featurestudios/d/{did}/w/{wid}/e/{eid}
{"contents": "<the whole .fs text>"}
```

HTTP Basic with `accessKey:secretKey` from an API key (Onshape calls that
"local testing only" and prefers HMAC-SHA256; for a script on one machine it is
five lines instead of forty). `sourceMicroversion` + `rejectMicroversionSkew`
are optional concurrency guards — for a generated file that is never
hand-edited, overwriting is what you want.

**The quota is the reason not to get clever.** Free and Standard plans get
**2500 API calls per year**; failed calls and anything done in the browser do
not count. A `--push` that POSTs once per regeneration is fine. A
watcher that syncs on save, or anything that walks the feature tree through
`getPartStudioFeatures`, is how you burn a year's allowance in a week.

There is no official local↔Onshape sync, no LSP, and no first-party editor
integration; the community VS Code extension is syntax highlighting only.

## Imports

`import(path : "<element id>", version : "<version>")`, optionally namespaced.
Same workspace → always current. Another document → pinned to a version, with a
link icon that offers the update when you want it. You can import Feature
Studios (their exported symbols), Part Studios (a `build` function that
instantiates their geometry — this is the Derived mechanism), and data blobs
(JSON/CSV via `BLOB_DATA`).

That last one is the interesting one for us: the layout *data* could be a JSON
blob tab consumed by a hand-written, stable Feature Studio, so the pushed
artifact is data and the code stops churning. Not done.

For several Part Studios — one per real part — the rule is **don't reference
geometry across them if you can avoid it**. Every studio imports the same
constants and positions itself from the same numbers the sim uses. Nothing to
break.

## Folders

Onshape has feature-tree folders: select features, group, name, nest, suppress
as a unit. But they organise the tree that *contains* a feature, and the planes
a custom feature draws are inside the node rather than siblings of it — so no
folder can be wrapped around them. FeatureScript cannot create folders or put
anything into one, and the same goes for the Parts list's folders.

Where it does pay off: the per-group features can be selected and folded into
one collapsed line. Which is an argument for inserting them.

## When a plane is worth generating

**Only when it does not coincide with geometry the model already has.** A print
or sketch plane picked off an existing face costs nothing to select and cannot
go stale; a generated one is a thing to keep in sync. So the servo mounting
plane never needed exporting — it is the plate's own face — while the fork's
plane did, because it holds the axle direction and the raked steering axis at
once and no face in the model is parallel to it.

Two traps found by using them:

- **The linkage's axis planes are not datums.** Each is normal to its own link,
  so they sit at 61.7 deg, 57.2 deg and so on to lateral — they look like flat
  references in the viewport and measuring against one produced a 47.076 deg
  that had no meaning. Only `AOW planes` and the `front_wheel` / `fork` axis
  planes are intended as datums.
- **The two sides are not mirror images.** The servos straddle 45 deg rather
  than sharing it, so each belt spans its own pair of tangents — left 24.52 to
  50.22, right 39.78 to 65.48. The bands OVERLAP, so their union is contiguous
  and there is no corridor between them: anything crossing both belt planes at
  one angular station has to pass below 24.52 or above 65.48.
- **A plane parallel to a belt run is clearance, not a print orientation.** A
  rear dropout leaves the axle outboard of its own belt and arrives at the
  servo mount inboard of it, so it crosses the belt plane, and it can only do
  that outside the belt-and-pulley hull — the way a chainstay threads past the
  chain. The crossing dictates the shape, and no single flat build plane then
  aligns with both the sleeve and the arm.

## What the export carries

Shapes: `box`, `cylinder`, `capsule`, `point`, `holes` (a pattern of positions
travelling as one entry, so eight of them do not bury the layout) and `plane`
(position plus normal). Frame conversion is done once, in code — model frame is
+X forward / +Y left / +Z up in metres, CAD is +X right / +Y forward / +Z up in
millimetres, and hand-converting per component is exactly the sign error that
survives review.

## What the ROBOTIS drawings taught us

Three separate times, the servo's real envelope exceeded what `box_size`
carried, and each time the mechanism was the same: **the depth ROBOTIS quotes
is not consistently the same thing.**

- **XC330**: quoted 26 mm = 23 case + 3 horn, dimensioned as two numbers on its
  own drawing. The horn was already inside the box.
- **XC430**: quoted 34 mm is the case *alone*, measured from the horn-side case
  face. The Ø20.5 horn stands 2 mm proud of it and a Ø7.9 boss another 1.9
  beyond that. Real overall depth 36, not 34.
- Neither is documented as such anywhere except the three-view.

The fix was to stop treating a datasheet number as a face. `box_size` D is now
the **case alone** for both, the horn is its own primitive, and the shaft point
is the **mounting datum** — the outer face of the horn, the surface a pulley or
bracket actually bolts to — which makes `shaft_from_horn_face` negative and
equal to `-horn_thickness`.

Also worth having written down:

- The **P.C.D 16 pattern on the horn side is on the horn** (bolt circle 16 sits
  inside the Ø20.5 disc), so it rotates and is not a mount. There is no idler
  on the back by default. The only static pattern common to both faces is the
  **22 × 40** corner set, which is where the case's own M2.5 FHS assembly
  screws live — longer replacements capture a plate.
- The `8 | 8` chain beside the horn is the P.C.D 16 written as two radii. The
  same circle twice, not a third pattern.
- Reading a dimension off the wrong end of a symmetric drawing is easy: the
  XC430's horn side reads `3.9 | 2` and its idler side `2 | 3`.

## Open

- No version pinning between the Feature Studio and the Part Studio.
- `--push` not written; the quota says one call per regeneration is affordable.
- The layout data could travel as a JSON blob rather than as generated
  FeatureScript source.
- The drive-servo mount is a proposal, not a decision — see
  `analysis/servo_mount.py`, four tags: `cage`, `spine`, `sleeve`, `plate`.
