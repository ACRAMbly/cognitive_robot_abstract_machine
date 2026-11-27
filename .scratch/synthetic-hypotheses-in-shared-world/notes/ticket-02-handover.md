# Ticket 02 handover

Branch `ticket/shared-world-02-shared-world`, worktree
`/home/nasim/Desktop/cognitive_robot_abstract_machine/.claude/worktrees/agent-a2136b0a9ff76a585`,
based on integration tip 76b2c3b500. WIP commit d2f3d9b74c is empty; nothing pushed.

## Done

- I read the ticket and its decisions, the spec, ADRs 0001/0003, the exploration notes and
  every source and test file the ticket touches.
- I copied the generated ORM interfaces into the worktree. They are git-ignored.
- I stopped the baseline test run before it finished. No test results yet.

## Left (all of the ticket)

No code or tests are written yet. The plan, test-first:

1. Mesh seam (`test/semantic_digital_twin_acrambly_test/test_meshes.py`): every concrete
   part/block class has a `mesh_file` classproperty, the mesh exists, and its extent is in metres.
   Then:
   - Add the new module `semantic_digital_twin_acrambly/mesh_files.py` with
     `MeshFile(PathEnumeration)`, `Path(__file__).resolve().parents[2]/"resources"/"meshes"/...`.
   - `AircraftPart` and `TetrisBlock` each get `IsPerceivable` and a non-abstract
     `mesh_file` classproperty. On the base it raises a new `DataclassException` in a new
     `semantic_digital_twin_acrambly/exceptions.py`.
   - Rewrite the two tetris STLs in metres, re-centred on the bounding-box centre.
   - Update the README ("in millimetres").
2. Interface tests: rewrite the two "no world pose / nothing added" tests. Then:
   - Add `Scenario.world_T_camera` (default identity).
   - Remove `adds_missing_camera` from the interface.
   - In `set_data`, call `bind_world_T_camera` before `store_camera_observation`.
3. Engine test: rewrite it to reader -> `MeshShapeAnnotator` -> `WorldUpdatingAnnotator`. Then:
   - Add `clock: Clock = field(default_factory=SystemClock)` to the engine.
   - Update the engine docstring.
4. Main test in `test/robokudo_acrambly_test`, parametrized over both engines. Use the
   `shared_world` and `node` fixtures (add `other_process_node, shared_world` to the
   conftest import). The exploration notes have a sketch.
   - Use a non-identity camera pose (`dataclasses.replace` on the scenario).
   - Wait for the first-pass meshes (about 9 MB) with a timeout longer than 5 s.
   - After a later pass, assert `ObjectMatcher.visual_center`, not the body origin.

Risk to check early: a classproperty that raises on the base may break any code that reads
every class attribute. For example, `inspect.getmembers` only catches `AttributeError`. A tetris
label resolves to `TetrisBlock` itself, so the main tetris test will show whether this is a
problem.

## Defaults chosen (not yet implemented)

- **D (world without a root):** the camera hangs below `str(world.root.name)`. With no root,
  the reference frame is the default of `StaticCameraTransformComponent.static_world_frame`
  ("map"), with no new literal.
- **E (engine needs each part's class):** `ScenarioPart.label` becomes a property
  `part_type.__name__` over a new field `part_type: type[AircraftPart | TetrisBlock]`, set by
  `PlaneLayout.arrange`. The engine builds `ObjectClassMesh(class_name=part.label,
  shape=Mesh.from_file(str(part.part_type.mesh_file)))`. The hypotheses still carry only the
  label.

## Tests

None run to completion on this branch. The README notes give the integration baseline:
`test/robokudo_acrambly_test test/semantic_digital_twin_acrambly_test` -> 64 passed.
