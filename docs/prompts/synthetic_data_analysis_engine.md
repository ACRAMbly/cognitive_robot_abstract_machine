# Prompt: synthetic Tetris observations and analysis engine

Work on a new branch created from `main`, suggested name
`feature/synthetic-tetris-observations`. If the user has already created that branch
from `main`, use it. Prepare a reviewable change for a PR titled
`Add synthetic Tetris observations and an observation-only analysis engine`.

Push the implementation branch to the explicitly authorized
[ACRAMbly fork](https://github.com/ACRAMbly/cognitive_robot_abstract_machine.git)
and open the PR in `ACRAMbly/cognitive_robot_abstract_machine`, targeting that
fork's `main` branch. Both the head and base branches must belong to that fork.

## Goal

Provide deterministic synthetic perception data for the two supplied Tetris block
models, exposed through a runnable RoboKudo analysis engine. The output is CAS
object hypotheses that a later annotator can consume.

The boundary of this change is:

`Scripted observations → CAS object hypotheses`

## Establish the starting point

- Read the applicable `AGENTS.md` instructions. Read `GLOSSARY.md` and
  `docs/agents/` guidance if present; these files are absent from the inspected
  `main` and are not prerequisites supplied by this prompt. If using a skill,
  follow its instructions and any available repository prerequisites.
- Inspect the branch and working tree and confirm the branch starts from `main`.
  Preserve unrelated work and use an isolated worktree when needed.
- Implement this feature on top of `main`. Assume the scripted-observation files
  below do not exist. Do not depend on or merge another feature branch, or require
  its glossary, specification, source modules, tests, or documentation.

## Existing reference files on main

Use these files to understand the existing interfaces and conventions:

- `robokudo/src/robokudo/analysis_engine.py`
- `robokudo/src/robokudo/pipeline.py`
- `robokudo/src/robokudo/cas.py`
- `robokudo/src/robokudo/annotators/core.py`
- `robokudo/src/robokudo/annotators/testing.py`
- `robokudo/src/robokudo/types/scene.py`
- `robokudo/src/robokudo/types/annotation.py`
- `robokudo/src/robokudo/descriptors/analysis_engines/stacking_robokudo.py`
- `test/robokudo_test/conftest.py` and `test/conftest.py`
- `test/robokudo_test/test_pipeline.py`

## Proposed new files to create

The following are suggested destinations for this PR's implementation, not
existing code to read or import. Create them as needed, following the established
interfaces on `main`:

- `robokudo/src/robokudo/io/scripted_observations.py`
- `robokudo/src/robokudo/annotators/scripted_observations.py`
- `robokudo/src/robokudo/descriptors/scripted_tetris_observations.py`
- `robokudo/src/robokudo/descriptors/analysis_engines/scripted_tetris.py`
- `robokudo/src/robokudo/vis/scripted_observations.py`
- `test/robokudo_test/test_scripted_observations.py`
- `robokudo/scripted_tetris.md`

Create any additional test data under the existing test directory conventions.
Do not assume names such as `ScriptedScenario`, `TetrisBlock`, or
`ScriptedObservationAnnotator` are already defined; introduce the types needed by
this implementation.

## Required behavior

1. Use the supplied `child_cube_0_colored.ply` and
   `child_cube_2_colored.ply` meshes. Keep their model identifiers and resource
   paths in structured model definitions introduced by this PR. Accept a configured
   mesh directory; do not assume a developer's local asset directory is tracked on
   `main`. Document how to supply the two meshes and report missing resources
   clearly. Require no camera, inference model, worker process, or credentials.
2. Define an editable scenario containing six stages: both blocks visible,
   translation, rotation, the first block absent, both blocks absent, and both
   blocks reappearing. Keep scenario values in a descriptor rather than scattering
   them through the annotator or tests. Describe these poses as illustrative data.
3. Emit a fresh CAS for every completed observation, including an empty scene.
   Preserve capture time, camera frame, camera intrinsics, and image dimensions.
   Absent blocks must not leave detections from the preceding observation.
4. Emit one `ObjectHypothesis` per visible block, with `Classification`, a plain
   camera-relative `PoseAnnotation`, `BoundingBox3DAnnotation`, an image region,
   and a cropped binary foreground mask. Clearly document rectangular diagnostic
   masks if those are used; do not imply rendered silhouettes or sensor realism.
5. Keep translations in metres and quaternion components in `xyzw` order.
   Convert mesh units once, preserve the original CAD origin, and derive box
   dimensions and center offsets from the supplied mesh. Rotation must move an
   offset box center correctly. Reuse an existing shape annotation to expose CAD
   geometry when available; do not introduce a parallel annotation format.
6. Support deterministic finite playback and optional repeating playback, with
   configurable timing and an injectable clock. Waiting ticks must not emit a
   second completed observation. Capture times must advance across repeat cycles.
   Document completion and waiting statuses using the existing runner contract.
7. Provide configurable intermediate motion between visible keyframes using
   linear translation and shortest-path quaternion interpolation. Preserve
   discrete absence and reappearance stages, configurable pauses, and a smooth
   visible return to the initial poses when repeating. Playback scheduling and
   capture timestamps must describe the same timeline.
8. Keep the `scripted_tetris` engine observation-only. Provide exact run commands
   and a concise description of its output and configuration in the documentation.

## Scope boundaries

Do not create or update object beliefs, initialize a belief world, modify
`SemanticDigitalTwinConnector`, add `WorldSynchronizer`, launch a receiving world,
or add RViz synchronization infrastructure. Those belong to separate changes.
Do not add Noctis, FoundationPose, live camera processing, planning, or control.

## Tests and acceptance criteria

Follow test-driven development: for each missing behavior, add a meaningful failing
pytest test before implementing it. Reuse existing fixtures and a controllable
clock; avoid real-time sleeps. Do not weaken or rewrite existing failing tests.

Cover both model classifications; exact scenario poses; translation and rotation
samples and endpoints; finite completion; repeat continuity; fresh CAS instances;
empty scenes; absence and reappearance; monotonic capture times; camera metadata;
mesh-derived metric bounds; and the offset CAD origin under rotation. Compare
against scenario definitions, model definitions, and actual mesh geometry rather
than duplicating constants. Test the public analysis-engine output, including that
the engine ends at hypotheses without creating beliefs.

All added tests must be discovered by the existing RoboKudo CI suite. Cover the
core behavior with checked-in test resources or deterministic local mesh fixtures,
without requiring a developer's untracked CAD directory, live services, or
credentials. Keep any additional real-model checks conditional on asset
availability and pair them with equivalent fixture-based coverage that runs in CI.

Use the configured project interpreter and determine required ROS overlays from
the environment and existing setup instructions. Do not assume a feature branch's
virtual environment or run documentation exists. Run the new scripted-observation
pytest suite and relevant existing pipeline checks. Run
`scripts/format_docstrings.py` on modified Python files.

## Deliverable

Provide the branch, focused implementation, tests, documentation, and a PR-ready
title and description with exact validation results. If the selected base already
satisfies a requirement, retain it and report the evidence. Do not manufacture
changes to already complete behavior. Do not publish, comment on, or modify an
upstream `cram2/cognitive_robot_abstract_machine` PR. Open a PR only in an explicitly
authorized fork: `ACRAMbly/cognitive_robot_abstract_machine`.
