# Synthetic Tetris observations and an observation-only analysis engine

Status: resolved

## Problem Statement

Developers need predictable perception observations to exercise later RoboKudo
annotators without operating a camera, running an inference model, or preparing a
belief world. They need to observe the two supplied Tetris blocks translating,
rotating, disappearing, and reappearing while retaining reliable camera metadata,
metric geometry, and capture times.

The immediate requirement is one finite, thirty-second observation sequence. Its
output must be the same CAS object hypotheses a later annotator will consume.
Creating and updating persistent beliefs belongs to the separate connector work.

## Solution

Provide a runnable `scripted_tetris` analysis engine that turns an editable
scenario into fresh CAS observations. The default sequence emits ten observations
per second from zero through thirty seconds inclusive, producing 301 observations.
It includes both blocks, translation and rotation of the first block, the first
block's absence, an empty scene, and both blocks reappearing.

Use the supplied `child_cube_0_colored.ply` and `child_cube_2_colored.ply` meshes
from a configured mesh directory. Each detected block carries its model
classification, camera-relative CAD-origin pose, mesh-derived bounding box, CAD
mesh geometry, image region, and cropped foreground mask following the actual
projected block outline. These are illustrative synthetic observations.

Run headlessly by default. An optional viewer can show the sequence and retain
the final scene after playback finishes. The engine stops at object hypotheses;
the later connector will associate those observations with persistent world
objects and create their visual and collision geometry.

## User Stories

1. As a perception developer, I want to run a synthetic analysis engine without a camera, so that I can work without physical sensing hardware.
2. As a perception developer, I want observations without inference models, worker processes, or credentials, so that setup remains local and predictable.
3. As a perception developer, I want to configure the mesh directory, so that I can use the supplied assets without relying on another developer's filesystem.
4. As a perception developer, I want missing meshes reported clearly, so that I can correct the resource configuration.
5. As a perception developer, I want both supplied block models identified consistently, so that later annotators can recognize their object kinds.
6. As a perception developer, I want an editable scenario descriptor, so that I can change poses, camera settings, timing, and motion pauses in one place.
7. As a perception developer, I want one thirty-second sequence, so that I can exercise the required behavior in a single finite run.
8. As a perception developer, I want ten observations per second by default, so that motion has intermediate samples.
9. As a perception developer, I want both initial and final observations included, so that the complete configured sequence is available.
10. As a perception developer, I want both blocks visible at the beginning, so that downstream processing receives an initial scene containing both object kinds.
11. As a perception developer, I want the first block to translate while the second stays stationary, so that I can distinguish object motion from a stable reference.
12. As a perception developer, I want the first block to rotate, so that I can exercise orientation changes and offset geometry.
13. As a perception developer, I want configurable intermediate translation and rotation samples, so that motion is continuous between visible keyframes.
14. As a perception developer, I want observations to continue while blocks remain stationary, so that motion pauses still show that the objects are present.
15. As a perception developer, I want the first block to disappear independently, so that later annotators can receive a scene with only the second block.
16. As a perception developer, I want an observation with no detected blocks, so that an empty scene is represented explicitly.
17. As a perception developer, I want both blocks to reappear at their last visible poses, so that later processing can exercise reassociation.
18. As a perception developer, I want disappearance and reappearance to happen at defined boundaries, so that absence is not represented as invented motion.
19. As a perception developer, I want a fresh CAS for every completed observation, so that absent blocks do not leave stale detections behind.
20. As a perception developer, I want waiting ticks to produce no additional completed observation, so that the sequence does not contain duplicate samples.
21. As a perception developer, I want every scheduled observation preserved when execution falls behind, so that runner speed does not change the observation sequence.
22. As a perception developer, I want capture times to follow the scheduled timeline, so that delivery delays do not change the apparent motion.
23. As a perception developer, I want camera frame, intrinsics, image dimensions, and capture time preserved, so that later annotators can interpret each observation correctly.
24. As a perception developer, I want translations in metres and quaternions in xyzw order, so that geometry follows the existing spatial conventions.
25. As a perception developer, I want the mesh scale declared explicitly, so that block dimensions are physically consistent without guessing file units.
26. As a perception developer, I want the original CAD origin preserved, so that a model pose has a stable geometric meaning.
27. As a perception developer, I want bounding-box dimensions and center offsets derived from the mesh, so that the box encloses the actual model.
28. As a perception developer, I want rotation to move an offset box center correctly, so that the bounding box follows the CAD geometry.
29. As a perception developer, I want the CAD mesh carried through existing annotations, so that later visual geometry can use the supplied model.
30. As a perception developer, I want mesh and bounding-box geometry kept separate from foreground masks, so that later world geometry does not depend on reconstructing a 3D shape from image pixels.
31. As a perception developer, I want foreground masks to follow the actual block outline, so that background gaps do not become foreground evidence during association.
32. As a perception developer, I want each foreground mask cropped to its image region, so that it fits the existing RoboKudo annotation contract.
33. As a perception developer, I want the example blocks fully in frame and separated, so that this feature does not require partial-visibility or occlusion handling.
34. As a perception developer, I want new deterministic detection identifiers in each observation, so that detections remain distinct without assigning persistent physical identity.
35. As a connector developer, I want the source to leave persistent identity and belief updates to the connector, so that observation generation remains independently usable.
36. As a perception developer, I want the headless command to exit after the final observation, so that a finite run completes on its own.
37. As a perception developer, I want visualization to be optional, so that a display is unnecessary for normal execution.
38. As a perception developer, I want an enabled viewer to retain the final scene until closed, so that I can inspect the result after playback.
39. As a test author, I want to control the clock and inspect the engine's completed CAS observations, so that tests can verify timing without waiting thirty seconds.
40. As a maintainer, I want deterministic local mesh fixtures in CI, so that validation does not depend on untracked assets or live services.
41. As a maintainer, I want tests to compare output with scenario definitions and mesh geometry, so that duplicated constants do not conceal errors.
42. As a developer running the engine, I want exact setup and run commands with configuration and output descriptions, so that I can reproduce the intended use.

## Implementation Decisions

- Preserve the boundary from scripted observations to CAS object hypotheses.
  Introduce structured model and scenario definitions, a finite playback source,
  an observation annotator, the analysis-engine descriptor, and visualization
  support as needed. Reuse existing RoboKudo interfaces and geometry support.
- Keep scenario and camera values in an editable descriptor. Keep model
  identifiers, mesh resources, and scale in structured model definitions. Accept
  a configured mesh directory and report missing resources clearly.
- Preserve the model identifiers `child_cube_0` and `child_cube_2` in
  classifications. Generate a deterministic detection identifier unique to each
  detection in each observation. A model identifier describes an object kind;
  neither that label nor a detection identifier is a persistent object identity.
- Declare both supplied meshes as millimetre-based, with `0.001` metres per mesh
  unit. Convert units once and preserve the original CAD origin. Mesh dimensions
  and center offsets come from the geometry rather than duplicated constants.
- Use translations in metres and quaternion components in `xyzw` order. The
  plain camera-relative `PoseAnnotation` locates the original CAD origin. The
  CAS supplies its camera frame and capture metadata.
- Each visible block produces one `ObjectHypothesis` containing `Classification`,
  a plain `PoseAnnotation`, `BoundingBox3DAnnotation`, an image region, and a
  cropped binary foreground mask. Carry CAD geometry through the existing
  `Shape` annotation with existing mesh geometry support.
- Keep the CAD mesh in its model frame. Derive box lengths and the model-local
  box center from mesh bounds. The camera-relative box pose transforms that
  center with the object's translation and rotation, including a nonzero center
  offset. The mesh, object pose, and box must describe the same transformed shape
  without applying scale or pose twice.
- Derive foreground masks from the projected CAD outline. Preserve background
  gaps within a rectangular image region; a filled rectangular mask is
  insufficient. Crop masks to their image regions and follow the existing
  binary-mask representation. Keep the example blocks fully in frame and
  non-overlapping. CAD visual geometry, enclosing collision geometry, and a 2D
  foreground mask are distinct inputs to later processing.
- Use a fixed optical camera frame with X right, Y down, and Z forward. Default
  image dimensions are 640 by 480 pixels and horizontal field of view is sixty
  degrees. Derive intrinsics with the existing camera-parameter calculation.
- Use the following illustrative CAD-origin poses, expressed in metres. Only
  the first block moves; the second remains a stationary reference. Keep all
  pose values editable in the descriptor.

  | Pose | `child_cube_0` translation | `child_cube_0` orientation | `child_cube_2` translation and orientation |
  | --- | --- | --- | --- |
  | Initial | `(-0.30, -0.15, 1.00)` | Identity | `(0.05, -0.15, 1.00)`, identity |
  | Translated | `(-0.20, -0.15, 1.00)` | Identity | Unchanged |
  | Rotated | `(-0.20, -0.15, 1.00)` | Positive ninety degrees about camera Z | Unchanged |

- Provide intermediate motion using linear translation and shortest-path
  quaternion interpolation. Motion pauses continue to produce observations
  with unchanged poses, new detection identifiers, and advancing capture times.
  Appearance and disappearance are discrete changes.
- Default to one thirty-second sequence at ten observations per second. Each
  movement lasts six seconds; each stationary or absence interval lasts three
  seconds. Keep timing editable and align interval durations to the sample
  period. The six conceptual stages include holds around translation and rotation.

  | Elapsed time | Observation state |
  | --- | --- |
  | 0–3 seconds | Both blocks at initial poses |
  | 3–9 seconds | First block translates; second stays stationary |
  | 9–12 seconds | Both blocks hold the translated state |
  | 12–18 seconds | First block rotates; second stays stationary |
  | 18–21 seconds | Both blocks hold the rotated state |
  | 21–24 seconds | First block absent; second stays stationary |
  | 24–27 seconds | Both blocks absent |
  | 27–30 seconds | Both reappear at their last visible poses and remain stationary |

- At each interval boundary, emit the new interval's state once. Include both
  the initial observation at zero seconds and the final observation at thirty
  seconds, yielding 301 observations at the defaults. Complete the sequence after
  that final observation.
- Use an injectable clock. Capture time starts at the clock's current Unix time
  when the sequence begins and advances by scheduled sample offsets. Preserve
  capture time in `CAS.data_timestamp` as integer nanoseconds since the Unix epoch
  and keep camera metadata consistent with it. Keep it distinct from the CAS
  construction time in `CAS.timestamp`. Tests control the clock; no production
  fixed-timestamp setting is required.
- Preserve every scheduled observation. Late ticks emit overdue samples in
  chronological order without dropping them or replacing their capture times
  with delivery times. Waiting ticks do not create a second completed observation.
- Replace the pipeline's CAS for each completed observation, including an empty
  scene. Preserve camera metadata and expose the result through the normal
  pipeline CAS access. Keep playback progress across successful iterations rather
  than restarting the scenario after each observation.
- Follow the existing runner contract: a completed observation succeeds, waiting
  remains running, and exhausted playback signals completion using the runner's
  existing failure-based termination behavior. Arrange the engine so the current
  headless runner observes exhaustion. The final CAS remains the last completed
  observation rather than becoming another sample.
- Make the documented default invocation headless and set an adequate runner
  tick rate for the configured sample rate. The default sequence describes thirty
  seconds of capture time; a runner that falls behind may take longer to deliver
  all observations. An optional viewer remains open on the final scene until
  closed.
- The synthetic engine creates no beliefs and makes no world changes. Existing
  empty shared-world allocation during launcher imports is outside this feature;
  it does not require startup or global-state refactoring.
- Follow repository rules for dataclasses, structured configuration, documentation,
  imports, and testing. Prefer existing dependencies; ask before adding one.

## Testing Decisions

- The user confirmed one primary test boundary: run the public `scripted_tetris`
  analysis engine and inspect its completed CAS observations, which are the same
  data a later annotator receives. Tests cover the assembled source, scenario,
  annotator, geometry conversion, and engine wiring through that boundary.
  Connector belief acceptance remains outside these tests.
- Good tests check observable behavior and fail for the behavior they name.
  Assert exact expected values where defined. Derive expectations from the
  scenario, model definitions, and consumed mesh fixtures. Do not mirror private
  implementation steps or duplicate production constants in assertions.
- Prior art includes existing Pipeline tests for CAS access and annotator scope,
  full-analysis-engine execution tests, and support for collecting distinct
  successful CAS observations. Reuse the existing ROS node, blackboard isolation,
  and annotator-output fixtures. Follow current pipeline CAS ownership rather
  than copying an outdated testing reader's separate blackboard output.
- Inject a controllable clock and advance it without real-time sleeps. Drive the
  public engine at regular, early, and late tick times. Check that the default
  sequence has exactly 301 distinct completed observations spanning thirty
  seconds, with strictly increasing scheduled capture times. Check that waiting
  ticks produce no completed sample and that late ticks lose no samples.
- Verify both model classifications, deterministic unique detection identifiers,
  exact scenario keyframes, intermediate translation and rotation samples, and
  their endpoints. Verify shortest-path rotation, including equivalent quaternion
  signs, through configured scenarios rather than private interpolation methods.
- Verify motion pauses continue to produce fresh observations at stationary
  poses. Verify the first block's disappearance, the empty interval, and both
  blocks' reappearance at the defined boundaries. Verify absent detections do not
  remain from preceding CAS instances.
- Verify each completed observation has a fresh CAS instance and the expected
  camera frame, capture time, image dimensions, and intrinsics. Include an empty
  scene in those metadata checks. Verify the final observation is emitted once
  and subsequent ticks do not restart playback or produce more observations.
- Use deterministic local mesh fixtures to verify metric bounds, scale applied
  once, CAD-origin preservation, and rotation of a nonzero box-center offset.
  Inspect the existing shape and box annotations on the emitted hypotheses and
  compare their geometry with the fixture the engine consumed.
- Use a mesh fixture with a known projected outline containing background inside
  its surrounding image rectangle. Check foreground and background pixels, mask
  dimensions, and alignment with the image region. Cover translation and rotation
  so a rectangular mask cannot satisfy the assertions. Verify the supplied
  scenario stays fully in frame and non-overlapping when real assets are present.
- Verify the engine leaves persistent beliefs and world contents unchanged from
  an isolated baseline. Existing launcher import-time empty-world allocation is
  not a failure of the observation-only contract. Do not exercise connector
  creation, matching, or world-update behavior in this feature's tests.
- Check compatibility with finite headless runner termination through the public
  engine's observable statuses. Optional visualization checks must not require a
  display in CI. Keep documentation consistent with the resulting waiting,
  completed-observation, and exhausted statuses.
- All new tests belong to the existing RoboKudo CI suite. Core coverage uses
  checked-in resources or deterministic local mesh fixtures, with no credentials,
  cameras, live APIs, external model downloads, or developer-specific asset paths.
  Extra supplied-mesh checks may be conditional on those assets, paired with
  equivalent fixture-based coverage that always runs in CI.
- Implement test-first with pytest and the project's configured interpreter.
  Reuse existing fixtures and do not weaken existing failing tests. Run the new
  focused suite and relevant pipeline checks in bounded package-level runs,
  checking available memory and capping memory before each run. Use the required
  no-automatic-ORM-build option and regenerate interfaces explicitly only if a
  change requires it. Format modified Python files with the repository's
  configured docstring formatter.

## Out of Scope

- Creating or updating object beliefs, assigning persistent world identity, or
  changing `SemanticDigitalTwinConnector`.
- Initializing a belief world, adding bodies to the world, or creating the later
  connector's visual mesh and collision shapes. This feature supplies the
  observation geometry needed for that separate work.
- Shared-world API changes, import cleanup, and launcher refactoring to eliminate
  existing empty-world allocation.
- `WorldSynchronizer`, receiving processes or worlds, synchronization transport,
  and RViz synchronization infrastructure.
- Noctis, FoundationPose, live camera processing, inference workers, planning,
  control, and sensor-realistic rendering.
- Inter-object occlusion, partially hidden blocks, and off-screen example poses.
- Repeating playback and an extra motion returning blocks to the initial poses.
- Additional recording workflows or production controls for a fixed timestamp.
- Dependence on the separate connector feature branch or its proposed modules.

## Further Notes

- The agreed input and operational requirements are captured in the
  [implementation prompt](../../synthetic_data_analysis_engine.md). Use the
  [domain glossary](../../GLOSSARY.md) for canonical terms.
- Respect [ADR 0001](../../docs/adr/0001-preserve-scheduled-observations.md) on
  preserving scheduled observations and
  [ADR 0002](../../docs/adr/0002-use-block-outlines-for-foreground-masks.md) on
  foreground masks and separate 3D geometry.
- Implement independently on a new branch from `main`, preserving unrelated local
  work. The suggested branch is `feature/synthetic-tetris-observations`. Do not
  merge or require the separate connector branch.
- The intended implementation PR title is “Add synthetic Tetris observations and
  an observation-only analysis engine.” Prepare its description with the
  `mattpocock-skills:pr` skill and include exact validation results. Retain and
  report any existing behavior that already satisfies a requirement.
- Future implementation publication is authorized only in the
  [ACRAMbly fork](https://github.com/ACRAMbly/cognitive_robot_abstract_machine),
  with both the head branch and base `main` belonging to that fork. Follow the
  repository remote-verification rules before any push. Do not publish, comment
  on, or modify upstream PRs.
- The specification and implementation tickets use the configured local Markdown
  tracker. Completion evidence is recorded below and in [review.md](review.md).

## Comments

- The user approved testing through the observations produced by the public
  analysis engine, including poses, meshes, bounding boxes, masks, timestamps,
  disappearance, reappearance, and completion with 301 observations. Connector
  world-update tests remain separate.

## Answer

All six implementation tickets are resolved on
`feature/synthetic-tetris-observations`. The public engine supplies the complete
301-observation sequence, metric CAD-origin geometry, projected foreground masks,
camera metadata, finite headless completion, and optional native visualization.
Existing runner and visualizer lifecycle behavior supplies headless exhaustion
and responsive inspection of the final scene. Beliefs and world contents remain
unchanged.

Final integrated validation used the configured workspace Python 3.12.3
interpreter with Open3D 0.20.0, serialized execution, a free-memory check, an
8 GiB cap, and `--orm-build never`:

```bash
python -m pytest \
  test/robokudo_test/test_scripted_tetris*.py \
  test/robokudo_test/test_pipeline.py \
  test/robokudo_test/test_cas.py \
  test/robokudo_test/test_garden.py \
  test/robokudo_test/test_gui.py \
  test/robokudo_test/test_utils_type_conversion.py \
  --orm-build never -q --disable-warnings
```

Result: **91 passed, 19 existing dependency warnings in 18.05 seconds**.
The real default headless command also completed with the supplied PLY meshes:

```bash
TETRIS_MESH_DIRECTORY="$PWD" \
  python -m robokudo.scripts.main _ae scripted_tetris _headless _tickrate 20
```

It exited with status **0** in **37.63 seconds including startup**, with no
Python exception in the execution log. Native visualization coverage uses the
real manager and backend selection with desktop window boundaries mocked.

All 17 changed Python files pass `scripts/format_docstrings.py` unchanged, and
`git diff --check` passes. The independent review found zero specification
mismatches; all eight documented standards groups were fixed, and the two
heuristics were considered separately. See [review.md](review.md), the
[run guide](../../robokudo/doc/scripted_tetris.md), and the
[prepared PR description](pull-request.md).
