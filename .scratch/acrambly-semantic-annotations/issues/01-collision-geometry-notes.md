# Move the collision-geometry notes

Status: resolved
Type: task
Blocked by:

Move `meshes/plane/collision_geometry_design.md` to
`.scratch/aircraft-collision-geometry/spec.md` and
`meshes/plane/collision_mesh_report.md` beside it. The copy in
`assembly_models/battat/plane/` is an older version (it has a stray `$`) and is
deleted in ticket 02, not merged.

Fix the links in the moved plan:

- the source directory `assembly_models/battat/plane` becomes
  `semantic_digital_twin_acrambly/resources/meshes/aircraft`;
- `../convention` becomes `acrambly/assets/battat/convention`;
- the glossary link, relative to the new location;
- the sentence claiming the glossary distinguishes visual from collision
  geometry: the glossary defines neither term, so drop the claim.

Content otherwise unchanged; the plan itself stays unexecuted.

Done when both notes are under `.scratch/aircraft-collision-geometry/` and
every link in them resolves.

## Comments

## Answer

Both notes moved with `git mv` to `.scratch/aircraft-collision-geometry/`
(`spec.md`, `collision_mesh_report.md`) in `cd1296899f`. The source directory
and the report's scope line now name
`semantic_digital_twin_acrambly/resources/meshes/aircraft`, the convention link
points at `acrambly/assets/battat/convention`, and the code links are relative
to the new location. The glossary sentence was dropped with its claim, which
took the glossary link with it. Every relative link resolves.
