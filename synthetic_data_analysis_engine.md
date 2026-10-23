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
object hypotheses that a later annotator can consume. Generate one finite
observation sequence per run.

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

## Package layout

The implementation lives in its own `robokudo_acrambly` workspace package,
extending the existing RoboKudo interfaces:

- `robokudo_acrambly/src/robokudo_acrambly/annotators/scripted_observation.py`
- `robokudo_acrambly/src/robokudo_acrambly/descriptors/analysis_engines/scripted_tetris.py`
- `robokudo_acrambly/src/robokudo_acrambly/scripted_observations/`
- `robokudo_acrambly/src/robokudo_acrambly/data/`
- `test/robokudo_acrambly_test/`
- `robokudo_acrambly/doc/scripted_tetris.md`

Create any additional test data under the existing test directory conventions.
Do not assume names such as `ScriptedScenario`, `TetrisBlock`, or
`ScriptedObservationAnnotator` are already defined; introduce the types needed by
this implementation.

## Required behavior

1. Use the supplied `child_cube_0_colored.ply` and
   `child_cube_2_colored.ply` meshes in the new package's data directory. Keep their
   model identifiers and resource paths in structured model definitions introduced
   by this PR. Accept a configured
   mesh directory; do not assume a developer's local asset directory is tracked on
   `main`. Document how to supply the two meshes and report missing resources
   clearly. Require no camera, inference model, worker process, or credentials.
   Declare the supplied meshes as millimetre-based, with an explicit scale of
   `0.001` metres per mesh unit in each model definition; do not infer units from
   mesh dimensions.
2. Define an editable scenario containing six stages: both blocks visible,
   translation, rotation, the first block absent, both blocks absent, and both
   blocks reappearing. Keep scenario values in a descriptor rather than scattering
   them through the annotator or tests. Describe these poses as illustrative data.
   Keep the supplied scenario's blocks fully in frame and non-overlapping.
3. Emit a fresh CAS for every completed observation, including an empty scene.
   Preserve capture time, camera frame, camera intrinsics, and image dimensions.
   Absent blocks must not leave detections from the preceding observation.
4. Emit one `ObjectHypothesis` per visible block, with `Classification`, a plain
   camera-relative `PoseAnnotation`, `BoundingBox3DAnnotation`, an image region,
   and a cropped binary foreground mask following the actual block outline
   projected from its CAD mesh. The image region may be rectangular, but the mask
   must exclude background gaps within that rectangle. A filled rectangular mask
   is insufficient. These are synthetic observations; do not imply sensor realism.
   Assign a deterministic detection identifier unique to each detection in each
   observation. Keep model labels `child_cube_0` and
   `child_cube_2` distinct from detection identifiers. A later connector owns
   association with persistent object identities.
5. Keep translations in metres and quaternion components in `xyzw` order.
   Convert mesh units once, preserve the original CAD origin, and derive box
   dimensions and center offsets from the supplied mesh. Rotation must move an
   offset box center correctly. Expose the CAD mesh through an existing shape
   annotation, alongside the mesh-derived bounding box. These provide separate
   inputs for a later connector's visual mesh and collision-box geometry; the
   foreground mask does not supply either 3D shape. Do not introduce a parallel
   annotation format or create world geometry in this engine.
6. Support one deterministic finite sequence, with configurable timing and an
   injectable clock. Waiting ticks must not emit a second completed observation.
   Use the clock's current Unix time at sequence start for the first capture time,
   then advance capture times according to the sample schedule. Tests control the
   clock; no additional recording or production timestamp-setting option is needed.
   Keep capture time distinct from CAS creation time.
   Preserve every scheduled observation and use its scheduled capture time,
   independent of when the runner emits it. If the runner falls behind, emit
   overdue observations in chronological order without dropping samples.
   Document completion and waiting statuses using the existing runner contract.
   Finite headless playback must exit after its final observation. An optional
   viewer remains open on the final scene until closed.
7. Provide configurable intermediate motion between visible keyframes using
   linear translation and shortest-path quaternion interpolation. Preserve
   discrete absence and reappearance stages and configurable pauses. Playback
   scheduling and capture timestamps must describe the same timeline.
   During motion pauses, continue emitting scheduled observations with unchanged
   poses, fresh detection identifiers, and advancing capture times.
8. Keep the `scripted_tetris` engine observation-only. Provide exact run commands
   and a concise description of its output and configuration in the documentation.
   Make the documented default command headless; visualization is opt-in.

## Illustrative sequence defaults

Use a fixed optical camera frame with X right, Y down, and Z forward. Use a
640-by-480 image and a 60-degree horizontal field of view. Derive camera intrinsics
with the existing camera-parameter calculation.

All translations below locate the original CAD origin in metres. Store rotations
as `xyzw` quaternions. Only the first block moves; the second remains stationary.
Keep these values editable in the descriptor.

| Pose | `child_cube_0` translation | `child_cube_0` orientation | `child_cube_2` translation and orientation |
| --- | --- | --- | --- |
| Initial | `(-0.30, -0.15, 1.00)` | Identity | `(0.05, -0.15, 1.00)`, identity |
| Translated | `(-0.20, -0.15, 1.00)` | Identity | Unchanged |
| Rotated | `(-0.20, -0.15, 1.00)` | Positive 90 degrees about camera Z | Unchanged |

Emit 10 observations per second over one 30-second sequence. Each motion lasts six
seconds; each stationary, single-block, or empty-scene interval lasts three
seconds. Keep timing editable in the descriptor and align interval durations to
the sample period.

| Elapsed time | Scene |
| --- | --- |
| 0–3 seconds | Both blocks at initial poses |
| 3–9 seconds | First block translates; second remains stationary |
| 9–12 seconds | Both blocks hold the translated state |
| 12–18 seconds | First block rotates; second remains stationary |
| 18–21 seconds | Both blocks hold the rotated state |
| 21–24 seconds | First block absent; second remains stationary |
| 24–27 seconds | Both blocks absent |
| 27–30 seconds | Both reappear at their last visible poses and remain stationary |

At an interval boundary, emit the new interval's state once. Include the initial
observation at zero seconds and the final observation at thirty seconds, yielding
301 observations at these defaults. Finish after the final observation. Do not
interpolate appearance and disappearance.

## Scope boundaries

Do not create or update object beliefs, initialize a belief world, modify
`SemanticDigitalTwinConnector`, add `WorldSynchronizer`, launch a receiving world,
or add RViz synchronization infrastructure. Those belong to separate changes.
Do not add Noctis, FoundationPose, live camera processing, planning, or control.
Keep existing shared-world startup behavior outside this change; do not refactor
the launcher or its imports to eliminate existing empty-world allocation. The
synthetic engine itself creates no beliefs and makes no world changes.
Inter-object occlusion and partially hidden blocks are outside this change.
Repeating playback and an additional return to the initial poses are outside scope.

## Tests and acceptance criteria

Follow test-driven development: for each missing behavior, add a meaningful failing
pytest test before implementing it. Reuse existing fixtures and a controllable
clock; avoid real-time sleeps. Do not weaken or rewrite existing failing tests.

Cover both model classifications; exact scenario poses; translation and rotation
samples and endpoints; finite completion; fresh CAS instances;
empty scenes; absence and reappearance; monotonic capture times; camera metadata;
mesh-derived metric bounds; foreground masks following actual mesh outlines;
unique deterministic detection identifiers; observations during motion pauses;
late ticks preserving scheduled samples; and the offset CAD origin under rotation. Compare
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
also pr must be created using pr skill in matt skills

Provide the branch, focused implementation, tests, documentation, and a PR-ready
title and description with exact validation results. If the selected base already
satisfies a requirement, retain it and report the evidence. Do not manufacture
changes to already complete behavior. Do not publish, comment on, or modify an
upstream `cram2/cognitive_robot_abstract_machine` PR. Open a PR only in an explicitly
authorized fork: `ACRAMbly/cognitive_robot_abstract_machine`.
