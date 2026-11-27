# Part annotations

Status: claimed
Type: task
Blocked by: 02

Add, in `semantic_digital_twin_acrambly`, the abstract `AircraftPart` and
`TetrisBlock` annotations built on `IsGraspable`, and their concrete
subclasses as listed in the spec: twelve aircraft parts (`LeftRearWheel` and
`RightRearWheel` renamed from their meshes) and `StraightTetrisBlock`,
`CornerTetrisBlock`. Follow AGENTS.md: dataclasses, `# %%` sections,
docstrings on every class.

Tests under `test/semantic_digital_twin_acrambly_test/`, test-first:

- each concrete part annotation attaches to a body in a world and is found by
  a query for its class and for its abstract base;
- the concrete subclasses of `AircraftPart` and of `TetrisBlock` are exactly
  the ones the spec lists, asserted against the classes themselves.

Done when the tests pass and `krrood check` is clean on the new code.

## Comments
