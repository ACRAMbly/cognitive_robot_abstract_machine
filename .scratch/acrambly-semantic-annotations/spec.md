# ACRAMbly semantic annotations

Status: ready-for-agent

Released for implementation by the developer on 2026-11-26.

## Goal

Give the aircraft, its aircraft parts and the tetris blocks semantic
annotations, so plans and queries can ask the world for `a(TopWing)` or
`a(CornerTetrisBlock)`. Give their meshes one home next to those annotations,
and let perception label its object hypotheses with the same classes instead
of a second list of part names.

Decision record: [ADR 0002](../../docs/adr/0002-acrambly-semantic-annotations-in-own-distribution.md).
Vocabulary: **Aircraft**, **Aircraft part**, **Tetris block** in
[the glossary](../../GLOSSARY.md).

## Scope

In:

- A new workspace distribution `semantic_digital_twin_acrambly`, depending on
  `semantic_digital_twin`, registered in the root `pyproject.toml`
  (`dependencies` and `[tool.uv.sources]`).
- Its meshes: `meshes/plane` moves to
  `semantic_digital_twin_acrambly/resources/meshes/aircraft/`, `meshes/cube` to
  `semantic_digital_twin_acrambly/resources/meshes/tetris_block/`, file names
  unchanged.
- Deleting the top-level `meshes/` and the obsolete `assembly_models/` (an
  older copy of the mesh report).
- Moving the two aircraft collision-geometry notes to
  `.scratch/aircraft-collision-geometry/`.
- The part annotations, the `Aircraft` annotation, and pytest coverage under
  `test/semantic_digital_twin_acrambly_test/`.
- Replacing the `StrEnum` labels of `robokudo_acrambly` with the annotation
  classes.

Out:

- Anything under `acrambly/`, including its duplicate meshes in
  `acrambly/assets`: left exactly as it is.
- An ORM interface (`scripts/generate_orm.py`) for the new distribution.
- Bolts, nuts and part holders.
- Turning a hypothesis into a mesh body in the world, and rescaling the tetris
  meshes from millimetres to metres.
- Executing the collision-geometry plan.

## Annotations

All in `semantic_digital_twin_acrambly`, following the conventions of
`semantic_digital_twin/semantic_annotations/semantic_annotations.py`.

- `AircraftPart`: abstract, built on `IsGraspable`. One concrete subclass per
  aircraft part mesh: `BottomWing`, `Chassis`, `FrontWheel`, `MotorGrill`,
  `Propeller`, `LeftRearWheel` (mesh `RearWheelLeft`), `RightRearWheel` (mesh
  `RearWheelRight`), `RearWing`, `TopWing`, `UnderBody`, `UpperBody`, `Window`.
- `TetrisBlock`: abstract, built on `IsGraspable`, with
  `StraightTetrisBlock` (mesh `child_cube_0_colored`, a bar of four cubes) and
  `CornerTetrisBlock` (mesh `child_cube_2_colored`, a row of three cubes with
  one cube sideways and one upwards from the same end).
- `Aircraft`: rooted at the body of its `Chassis`, with one field per other
  aircraft part, typed as that part's class or `None`, defaulting to `None`
  until the part is mounted, and marked with `IsPartWholeRelationship`.

## Perception labels

`robokudo_acrambly/synthetic_observations/parts.py` and its `TetrisBlock` and
`AircraftPart` `StrEnum`s are removed. A scenario part's label is the name of
its annotation class (`TopWing`, `StraightTetrisBlock`), and the parts a
scenario contains come from the annotation class hierarchy, never from a
second hand-written list. `robokudo_acrambly` gains a dependency on
`semantic_digital_twin_acrambly`.

## Tickets

1. [Move the collision-geometry notes](issues/01-collision-geometry-notes.md)
2. [Distribution and meshes](issues/02-distribution-and-meshes.md)
3. [Part annotations](issues/03-part-annotations.md)
4. [Aircraft annotation](issues/04-aircraft-annotation.md)
5. [Perception labels from annotation classes](issues/05-perception-labels.md)

## Comments
