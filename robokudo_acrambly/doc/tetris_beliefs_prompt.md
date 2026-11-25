# Prompt: typed beliefs for the scripted Tetris blocks

Hand this document to a coding agent at the start of the Tetris session. It is
the brief for one pull request. Read it in full before doing anything else.

## Your task

Give the scripted Tetris source the same belief pipeline that PR 13
([ACRAMbly PR 13](https://github.com/ACRAMbly/cognitive_robot_abstract_machine/pull/13),
"Create typed aircraft beliefs and preserve local world updates") gave the
scripted aircraft source. That includes semantic types and semantic
annotations for the blocks, a model catalog, a camera-to-world calibration, and
acceptance of the observed blocks into RoboKudo's local belief world through
`SemanticDigitalTwinConnector`. Each block then has one persistent belief, body,
UUID, semantic annotation, mesh geometry and accepted pose, and these survive
movement, absence and reappearance.

Do not write code first. Follow this order:

1. Read `AGENTS.md`, `GLOSSARY.md`, `docs/adr/` and the files under
   [Where to look](#where-to-look).
2. Run `/grill-with-docs` on this document and settle every question in
   [Open questions](#open-questions) with the user. Update `GLOSSARY.md` and
   `docs/adr/` as decisions are made.
3. Write the spec and tickets under `.scratch/tetris-beliefs/` (`spec.md`,
   `map.md`, `issues/NN-<slug>.md`), following `docs/agents/issue-tracker.md`
   and using `.scratch/local-world-belief-updates/` (PR 13's plan) as the
   model.
4. Implement ticket by ticket, test first, only after the user grants
   implementation permission.

## Where things stand

- PR 13 is merged into `main` of the ACRAMbly fork before this work starts.
  Branch from that `main`. Confirm with `git log` that PR 13's commits are
  there, and with the full `git remote -v` output that `origin` is the fork.
- The pull requests follow in this order: PR 13 (aircraft beliefs), then
  **this PR** (Tetris beliefs), then the Coraplex belief consumption PR
  ([coraplex_belief_consumption.md](coraplex_belief_consumption.md)).

## What the user has decided

- Treat the scripted Tetris source exactly like the aircraft source, including
  its integration with semantic labels and semantic annotations.
- `robokudo_acrambly/src/robokudo_acrambly/descriptors/analysis_engines/stacking_robokudo.py`
  (the query-driven Orbbec cube engine) is out of scope. Leave it unchanged.
- The user plans to test Tetris on the real robot later. This PR stays with
  the scripted source.

## Out of scope

These belong to the Coraplex PR that comes after this one. Do not build them
here:

- the shared robot world: fetching Giskard's world, `WorldSynchronizer`,
  held-back updates in Coraplex
- the capture time of accepted observations in the shared world
- the stand-in world server in `acrambly_temp/`
- anything in `coraplex/`, `acrambly/` or `robokudo/`

Do not make choices that block the Coraplex PR. It will:

- run both the aircraft and the Tetris engines against a world fetched from a
  world server, with no local belief world for either (`docs/adr/0001-…`);
- take the world frame from the root of the fetched world, not from a
  configured name;
- give every perceived part or block a capture-time field from a mixin in
  `robokudo_acrambly`, which the Tetris semantic types must be able to take;
- remove the two rear wheels from the aircraft catalog. If this PR refactors
  the aircraft catalog, tell the user so the removal can move here.

## How Tetris differs from the aircraft today

| | Aircraft (after PR 13) | Scripted Tetris |
|---|---|---|
| Semantic types | `AircraftPart` and one subclass per kind (`aircraft/types.py`) | none; kinds are labels `child_cube_0` and `child_cube_2` (`scripted_observations/models.py`) |
| Model catalog | `AircraftModel`: label → semantic type → STL (`aircraft/models.py`) | `TetrisModel`: label → PLY, `metres_per_mesh_unit = 0.001` |
| Frames | `aircraft_frames.json`: world frame, camera frame, camera-to-world calibration | camera optical frame only (`scripted_camera_optical_frame`); poses are camera-relative and there is no world frame |
| Hypotheses | classification and pose (`annotators/aircraft_observation.py`) | classification, projected region and mesh-derived annotations (`annotators/scripted_observation.py`) |
| Belief acceptance | `SemanticDigitalTwinConnector` with `semantic_types` and model geometry (`descriptors/analysis_engines/scripted_aircraft.py`) | none; the pipeline holds only `ScriptedObservationAnnotator` (`descriptors/analysis_engines/scripted_tetris.py`) |
| Mesh location | packaged resources | `MeshEnvironment.DIRECTORY` environment variable or the working directory |

The two scripted sources already repeat the same work: scheduling captures,
writing a classification for each visible kind, interpolating keyframe poses
and handling absence intervals.

## Open questions

Settle these with the user before writing the spec.

1. **Shared code, or a Tetris copy of the aircraft code?** `AGENTS.md`
   forbids duplicated code. Options:
   - (a) one generic path with two catalogs. A catalog base maps a kind label
     to a semantic type, a mesh resource and metres per mesh unit, and one
     scripted observation source and one engine setup are configured by a
     catalog and a scenario. PR 13's aircraft code is refactored onto it, and
     its tests must stay green.
   - (b) a Tetris copy of the aircraft structure.

   The earlier session recommended (a).
2. **What the blocks are and what their types are called.** The CAD names
   (`child_cube_0`, `child_cube_2`, `FIRST_BLOCK`, `SECOND_BLOCK`) say nothing
   about the blocks, and the real Tetris task
   (`acrambly/sub_parts/real/task_tetris.py`) talks about red, yellow and blue
   blocks. Ask the user how many physical blocks there are, and the shape and
   colour of each CAD model. Then choose names by shape, by colour, or as one
   kind with several copies. If two blocks are physically identical, this is the
   same problem as the aircraft's two rear wheels: PR 13 never lets one kind
   match a belief of another kind.
3. **Where the Tetris camera sits in the world.** Either Tetris gets its own
   scripted camera-to-world calibration in a frames file like
   `aircraft_frames.json` (earlier recommendation), or the two scenes share one
   scripted camera.
4. **Which annotations the Tetris hypotheses carry for acceptance.** Do the
   projected region and the mesh-derived annotations stay next to the pose
   the connector needs, and which of them does the object comparator use for
   association?
5. **Where the meshes come from.** The repository root has `meshes/plane/`
   (the aircraft STLs) and `meshes/cube/` (`child_cube_0_colored.stl`,
   `child_cube_2_colored.stl`). The Tetris code still reads the PLY files in
   `robokudo_acrambly/src/robokudo_acrambly/data/` through the
   `MeshEnvironment.DIRECTORY` variable. Decide which files are the block
   models and how the engine locates them, and check that their scale reaches
   the connector's model geometry correctly.
6. **Test scenario.** The aircraft acceptance test runs all kinds through a
   six-stage sequence (creation, translation, rotation, partial absence, total
   absence, reappearance). The Tetris scenario already moves, rotates, hides
   and brings back its blocks. Decide whether the same six stages apply.
7. **Glossary.** The glossary speaks of "blocks" in *Scripted observation*,
   *Motion pause* and *Foreground mask*, but has no term for a Tetris block as
   a kind. Decide whether one is needed, next to *Aircraft part*.

## Where to look

- PR 13's plan, as the model for this one: `.scratch/local-world-belief-updates/`
  (`spec.md`, `map.md`, `issues/01`–`06`).
- PR 13's brief and documentation: `robokudo_acrambly/doc/semantic_digital_twin_connector.md`,
  `aircraft_models.md`, `scripted_aircraft.md`, `aircraft_belief_verification.md`.
- Aircraft code: `robokudo_acrambly/src/robokudo_acrambly/aircraft/`,
  `aircraft_observations/`, `annotators/aircraft_observation.py`,
  `descriptors/analysis_engines/scripted_aircraft.py`.
- Tetris code: `robokudo_acrambly/src/robokudo_acrambly/scripted_observations/`,
  `annotators/scripted_observation.py`,
  `descriptors/analysis_engines/scripted_tetris.py`, the PLY meshes in
  `robokudo_acrambly/src/robokudo_acrambly/data/`.
- Tetris documentation: `robokudo_acrambly/doc/scripted_tetris.md`,
  `scripted_tetris_masks.md`, `scripted_tetris_visualization.md`.
- Generic connector: `robokudo/src/robokudo/annotators/semantic_world_connector.py`,
  `robokudo/src/robokudo/world.py`.
- Tests: `test/robokudo_acrambly_test/test_aircraft_*.py`,
  `test_scripted_aircraft_*.py`, `test_scripted_tetris_*.py`, `conftest.py`,
  `dataset/`.
- The next PR's design, to stay compatible with it:
  `robokudo_acrambly/doc/coraplex_belief_consumption.md`, `docs/adr/`.

## Rules

- `AGENTS.md` applies in full. In particular, follow its rules for tests:
  pytest, test first, `--orm-build never`, never the whole suite at once, check
  free memory and cap each run's memory.
- Use the project's configured Python interpreter on this machine. Paths in
  this document are relative to the repository root.
- Ask the user before changing anything outside `robokudo_acrambly/` and
  `test/robokudo_acrambly_test/`.
