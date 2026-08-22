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
| `AOW belts` | the four straight belt runs as solids, opaque near-black |
| `AOW belts mirror` | each run copied to the OTHER side, translucent magenta. Suppress this node to drop the symmetry aid without touching the real belts |
| `AOW four-bar sketch` | the righting linkage as construction geometry |
| `AOW bike layout` | the superseded all-in-one node, kept so documents that already have it keep their geometry. **Now carries a "Draw <group>" tickbox per group** — see below |

The per-group split is not cosmetic. A tree node can be **renamed**,
**suppressed**, and **reordered**; a checkbox inside one feature can do none of
those. It also shrinks the blast radius: a runtime error aborts the whole
feature it is in, and the first version of this lost the published variables to
a bug in the envelope code because both lived in one function.

Groups are derived from the data — `groups = dict.fromkeys(it["group"] ...)` —
so adding a group to `build()` adds a feature. There is no per-group code.

### The legacy node got per-group tickboxes too

The argument above is still right: a tree node can be renamed, reordered,
suppressed and folded, and a checkbox can do none of those. But a document
already built on the single `AOW bike layout` node cannot have any of it
without inserting eleven features and losing every entity id it already
references — which is a real cost for the one thing people actually want,
"hide all the electronics". So the same group list also generates a
`Draw <group>` boolean on the legacy node, and the draw loop skips a component
whose group is switched off.

They live in a **named parameter section** (`"Group Name" : "Which groups to
draw"`), which is not cosmetic. Appended bare they landed underneath the
`Plane size` numeric field at the bottom of a fifteen-field dialog, and the
first person to go looking for one did not find it. A generated control nobody
can locate is not a feature.

**The test is `== false`, deliberately, not `!= true`.** These parameters are
new on a feature that is already inserted in live documents. If Onshape does
not backfill an annotation default into an existing instance, every one of them
reads `undefined` — and under `!= true` the whole model would vanish on the
next regeneration. Under `== false` an unset parameter draws, so the worst case
is a checkbox that does nothing until the feature is edited once.
Wrong-but-visible beats wrong-and-empty.

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

Copy-paste, or `--push`. Wired 2026-08-21 as `aow_sim.onshape`, two calls wide:

```
POST /api/featurestudios/d/{did}/w/{wid}/e/{eid}   {"contents": "<the .fs text>"}
GET  /api/partstudios/d/{did}/w/{wid}/e/{eid}/shadedviews?pixelSize=0&...
```

Copy-paste is still the default and still the fallback: browser calls are
exempt from the quota, so exhausting the API changes nothing about pasting.

`config/onshape.yaml` holds the document, workspace and three tab ids under
names, so `--push` and `--shot` take no arguments. It is checked in and carries
no secret — an id grants nobody anything. Naming the tabs is not tidiness: the
Feature Studio and the two Part Studios are three 24-hex strings with no
visible difference, and passing the wrong one 404s with nothing to read.

- **`feature_studio`** — generated, overwritten wholesale on every push
- **`layout`** (the Part Studio named `aow-bike-import`) — where the generated
  features are inserted, and what `--shot` renders by default
- **`bike`** — the real drawing, with imported wheel geometry. A `--shot`
  target only; nothing generated is ever written there

HTTP Basic with `accessKey:secretKey` from an API key (Onshape calls that
"local testing only" and prefers HMAC-SHA256; for a script on one machine it is
five lines instead of forty). The key needs **read + write documents and
nothing else** — replacing a studio's contents is a write, not a delete, and
`delete` / `share` / `purchases` are the three scopes that turn a bad script
from recoverable into not.

**You cannot test a key for free.** Measured 2026-08-21: Onshape answers 404 to
a nonexistent document id whether or not you are authenticated, and 403 to a
real one identically for bogus credentials and for no `Authorization` header at
all. So no probe of an absent document distinguishes a good key from a bad one,
and `python -m aow_sim.onshape <url>` reads a document you own instead — one
call on success, zero on failure, because 4xx is not billable. It proves read
scope only; the first push is the only test of write. `sourceMicroversion` + `rejectMicroversionSkew`
are optional concurrency guards — for a generated file that is never
hand-edited, overwriting is what you want.

**The quota is the reason not to get clever.** Free and Standard plans get
**2500 API calls per year** (Professional 5000); failed calls and anything done
in the browser do not count. A `--push` that POSTs once per regeneration is
fine. A watcher that syncs on save, or anything that walks the feature tree
through `getPartStudioFeatures`, is how you burn a year's allowance in a week.
Onshape shows a total and never a breakdown, so every call is appended to
`~/.local/state/aow/onshape_calls.jsonl` with what it did, billable or not
(`python -m aow_sim.onshape --log`). It lives outside the repo because it is
per-machine state, and it is advisory — check it against the usage page rather
than trusting it.

**The billing cycle is not the calendar year, and it is not the date the usage
page calls "Tracking start date" either.** That field read 19 Feb 2026 while
the same page said 312/365 days elapsed, which puts the real anchor at
13 Oct 2025. The elapsed-day count is the field to trust. Unused calls do not
roll over, so the number that matters is calls-per-day remaining, which is what
`budget_line` prints.

**`pixelSize=0` is what makes the render usable** — it fits the model to the
frame. With any other value the view matrix sets direction and pan only, and
the zoom is yours to get wrong. `shadedviews` reads the Part Studio, not the
Feature Studio, so it needs the other tab's element id; a studio that failed to
compile renders the error rather than the previous geometry.

There is no official local↔Onshape sync, no LSP, and no first-party editor
integration; the community VS Code extension is syntax highlighting only.

## The belts, and what "symmetric" costs

The belts were four clearance PLANES and are now also eight SOLIDS. A plane has
no thickness and no ends, so it forbids a whole sheet the belt does not occupy
and permits the two regions past the pulleys that it does; a chainstay can only
be checked against a solid.

Each run is a prism from tangent point to tangent point, 104.66 x 9 x 3.6 mm,
with its INNER face on the tangent line — `p1`/`p2` sit on the flange
envelopes, so the line is the belt's inner surface, not its centre. Only the
straight run is drawn; the wrap is the pulley envelope, already there.

**Each side also carries the OTHER side's belt, mirrored** (`belts_mirror`,
translucent magenta, its own feature node so it can be suppressed in one
click). The servos straddle 45 deg rather than sharing it — left at 37.372,
right at 52.628 — so the two sides' keep-outs differ by the 15.256 deg of
straddle. A chainstay that is the same part on both sides has to clear both,
and no single side's geometry shows that. Mirroring draws the union instead of
asserting it.

### The corridor, in the frame that matters

Two coordinate systems get mixed up here, and they differ by 90 deg.

- **Run angles**: left 24.522 / 50.222, right 39.778 / 65.478. These OVERLAP —
  right-lower sits below left-upper — so the union is contiguous, 24.522 to
  65.478, and there is no threading between the two belts. A guess that the gap
  lies "between the lower drive's upper belt and the upper drive's lower belt"
  is inverted: that window is negative by 10.444 deg.
- **Angular station about the rear axle**, which is the frame a chainstay
  leaving the axle actually lives in. Each tangent line touches the input
  pulley at run_angle -/+ 90, so the hull occupies 205.700 deg per side and the
  symmetric free window is **155.478 to 294.522 deg** — 139.044 wide, centred
  on exactly 225 = `drive_servo_angle_deg` + 180. Ray-sampled against the true
  two-circle hull to confirm.

Recorded rather than drawn. A drawn sector would wrongly exclude a chainstay
that ducks under both belts entirely, which is a legitimate route.

## Two traps in the generator

**The framed-box ordering.** `render` and `render_featurescript` feed `fCuboid`
the extents as `(box[1], box[0], box[2])` — the model-to-CAD X/Y swap — but
pass the three frame axes through `to_cad_dir` **without reordering the tuple**.
So `box[0]` lands along `frame[1]`, `box[1]` along `frame[0]`, and only
`box[2]`/`frame[2]` pair up the way they read. The servos are correct because
`box_size[0]` is the lateral width and `frame[1]` is the lateral axis, so they
happen to agree — which is why this never surfaced. Authoring the belts the
obvious way produced a run 9 mm long and 104.66 mm wide. Matched, not fixed:
the servos depend on it. Normalising it means reordering the frame tuple
alongside the extents and re-verifying the servos and the payload pack.

**Colour is a one-way door, and this document has already walked through it.**
`setProperty` with `PropertyType.APPEARANCE` and `color(r, g, b, a)` on
`qCreatedBy(subId, EntityType.BODY)` works; the 4th alpha argument is real and
alpha is honoured on a part with no `allowFaces`. But the rule that governs
names governs appearance: a colour the USER has set by hand can never
afterwards be overwritten from FeatureScript.

**Observed 2026-08-21, not merely feared.** A per-group palette was pushed with
correct `rgba` on all 49 bodies — verified in the generated `.fs`, amber servos,
green electronics, violet linkage — and in the viewport ONLY the eight belts
changed colour. The belts were brand-new bodies; every other body in this
document already carried a hand-set appearance (which is why the model was
uniformly pale translucent blue before any of this), and each one is
permanently immune.

Nothing in the generator can recover them. **What actually worked was a fresh
Part Studio tab**: new bodies have no hand-set appearance, so they take the
palette immediately. The old tab is kept as `rip` in `config/onshape.yaml`
rather than deleted, because the lock-out is worth being able to poke at.
Resetting appearance per part is the in-place alternative, if you can find the
control.

Until one or the other happens the palette is real in the export and invisible
in the document — worth knowing before concluding the generator is broken. It
took a check of the pushed `.fs`, where the correct `rgba` was sitting on all
49 bodies, to tell the two apart.

Two live consequences: **alpha is scene-wide.** A first attempt at 0.45 across
every group washed all hues toward the background AND made `shadedviews` return
a BLACK background instead of white. 0.85 restored both. And the see-through
should be spent only where it earns its keep — the mirrored belts, and the
frame's inertia primitives, which are not parts at all.

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

### The two prospective build planes

Both added 2026-08-21, both alongside the datum they derive from rather than
replacing it — sketching on one must not silently redefine the other.

- **`plane_fork_print_offset`** — `plane_fork_print` moved 8 mm along its own
  normal (forward, tilted up by the 15 deg of rake). The ORIENTATION is the
  derived half and stays authoritative; the offset is `bike.fork_print_offset`,
  signed, and is meant to be edited once the fork has a real thickness.
- **`plane_drive_mount_print`** — the rear motor mount and dropout as one part.
  Normal is the mount's TANGENTIAL axis, 135 deg, so the build direction is up
  and rearward and the first layer is the bottom-front face. It lies in the
  LOWER (left) servo's long 34 x 46.5 outer face, at 28.500 mm tangential off
  the 45 deg centre line — a face that already exists in the assembly, rather
  than a datum nobody can point at. That 28.500 is `C sin(dtheta)` = 14.250
  plus half a case = 14.250, equal only because `drive_servo_gap` is 0 and the
  cases touch; open that gap and the plane moves with it. Per
  `drive_mount_open_wall`, a tangential build axis makes a SIDE wall the
  ceiling.

  It is a build plane for the MOUNT end, not a promise the whole part lies
  flat: the dropout still crosses the belt plane between here and the axle.

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
