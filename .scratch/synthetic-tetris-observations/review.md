# Synthetic Tetris observations review

Reviewed integration base: `4607593b890d0c9f40893971247d1e326003ec6c`.

The independent review reported **8 documented standards findings**, **2 heuristic
judgment calls**, and **0 specification findings**. All eight documented groups
are resolved below. The specification and its six implementation tickets remain
unchanged; final integration validation and tracker completion belong to the
integrating agent.

## Standards axis: original report

Hard violations (paths below are relative to `robokudo/src/robokudo/` or `test/robokudo_test/`; rules are from root `AGENTS.md`):

1. `annotators/scripted_observation.py:100–149`: `update` mixes scheduling/orchestration with detailed hypothesis construction. Breaches Methods: “every statement … on the same level of abstraction.”
2. `scripted_observations/scenario.py:125–148`: repeats the exact cadence-alignment predicate across both validators. `test_scripted_tetris_masks.py:184–198` and `test_scripted_tetris_motion.py:351–362` repeat occupancy checks; static/sequence tests duplicate calibration assertions and playback/motion/sequence tests duplicate capture loops. Breaches Code Style: “Never duplicate code.”
3. `test_scripted_tetris_sequence.py:41–48`: duplicates production `AppearanceBoundary`; `:130–132` retypes timing defaults. Breaches Assertions: “Assert against the definition, not a retyped copy.” `test_scripted_tetris_visualization.py:90` additionally asserts incidental native-renderer color in a mask/region test, contrary to “each test checks the one behaviour it names.”
4. `scripted_observations/scenario.py:201–232`: meaningful default translations and quarter-turn are literals inside a factory instead of configurable fields/classproperties/enums. Tests also scatter mesh dimensions, worked projection coordinates, timing values and border widths. Breaches Structured Data: magic numbers require those structured homes.
5. `test_scripted_tetris_masks.py:168–173`: `(translation_offset, rotation_angle)` examples are positional records. Breaches Structured Data: meaningful tuple positions become dataclasses.
6. Imports in `test_scripted_tetris_masks.py`, `test_scripted_tetris_playback.py`, `test_scripted_tetris_motion.py`, `test_scripted_tetris_sequence.py`, `test_scripted_tetris_visualization.py`: `LocalMeshes`, plus motion’s `CAS`, sequence’s `CapturedSequence`, visualization’s `Iterator`, are type-only but unguarded. Breaches Imports: guard type-only imports.
7. New source/tests use abbreviated aliases `np`, `o3d`, `npt`; `models.py:17–43` uses subjectless `FIRST`/`SECOND`; `CameraSettings.frame`, `ObservationCamera.settings`, and annotator `models` obscure their subjects. Breaches Naming: no abbreviations; names reveal their subject independently.
8. `scripted_observations/geometry.py:57` and `projection.py:57–58`: spatial points use generic `box_center`/`vertices` and three coordinates. `semantic_digital_twin/doc/style_guide.md` requires frame-qualified `root_P_tip` homogeneous points.

Judgment calls:

- Possible Message Chains: visualization `“hypothesis.roi.roi.get_corner_points()”` and tests `“annotator_output_buffer.map[pipeline.name].outputs[source.name]”` expose nested storage.
- Possible Feature Envy: annotator initialization directly validates `“timing.duration_nanoseconds % timing.sample_period_nanoseconds”`; `PlaybackTiming` could own schedule validity.

No tests run; formatter-enforced concerns omitted. API renames require user approval.

## Standards axis: resolutions

1. **Abstraction levels:** `ScriptedObservationAnnotator.update` now orchestrates
   scheduling, a fresh CAS, camera metadata, hypotheses, and native output.
   `schedule_capture` owns clock/cadence decisions; `write_hypotheses` owns the
   detailed hypothesis construction. `ScheduledCapture` groups the elapsed
   offset and Unix capture timestamp. Existing method names remain intact.
2. **Duplication:** `PlaybackTiming.contains_unsampled_offsets` owns the shared
   cadence predicate. The test-owned `CameraMetadata` and `ImageOccupancy`
   dataclasses retain all exact calibration, bounds, dimensions, mask, and
   disjointness checks. `CapturedSequence.capture` collects public pipeline CAS
   observations for regular, delayed, and headless-tree delivery. Existing
   fixtures still provide local meshes, clock control, ROS nodes, and output
   isolation. The motion occupancy check additionally verifies binary pixels.
3. **Definitions and focused assertions:** sequence tests import the production
   `AppearanceBoundary`. One `SequenceAgreement` defines the independent approved
   defaults, including 301 observations and the 21/24/27-second boundaries; the
   complete-sequence test checks those defaults once. Other checks consume the
   production definitions or fixture/scenario values. The image-region test
   checks exact native rectangle coordinates for every emitted region instead
   of requiring an incidental native rectangle color. Exact foreground and
   background comparisons remain.
4. **Structured configuration:** `ScenarioPoses` supplies configurable initial,
   translated, and reference poses plus the turn angle; each default pose has
   one source. `TetrisScenario.create_blocks` accepts that dataclass without
   breaking its existing no-argument call. Fixture dimensions, worked image
   cases, short crossing rotations, image margins, and compact playback timing
   now live in dataclass defaults or enums. Worked pixel support remains an
   independent expected result; no production rasterizer computes the oracle.
   Image colors also use named `ImageChannels`, with positional conversion only
   at the NumPy image boundary.
5. **Scenario records:** `ProjectedMotion` replaces positional translation/angle
   records, and `MotionExample` names the selected stages. Callers read the
   explicit translation and rotation attributes.
6. **Type-only imports:** `LocalMeshes`, `CAS`, `Iterator`, `Pipeline`, `Callable`,
   and the model type in the geometry loader are guarded where only annotations
   consume them. `CapturedSequence` is now intentionally a runtime import in
   sequence/playback tests because its capture classmethod is called there.
7. **Names:** touched feature code uses `numpy`, `numpy.typing`, and `open3d`
   without abbreviated aliases. CAD enums identify `FIRST_BLOCK` and
   `SECOND_BLOCK`. The formerly vague fields are `camera_frame`,
   `camera_settings`, and `model_geometries`. All feature readers are updated;
   no method was renamed.
8. **Spatial conventions:** metric geometry uses `model_P_box`,
   `camera_T_model`, and `camera_P_box`; projection uses `model_P_vertices` and
   `camera_P_vertices`. Points have homogeneous fourth coordinates, transforms
   are 4 by 4 matrices from the existing transform helper, and only the external
   projection boundary receives three-dimensional coordinates.

### Heuristic decisions

- **Message chains:** retained the existing RoboKudo region and output-buffer
  contracts. They describe stable external data ownership, and adding forwarding
  accessors solely to shorten them would not improve the feature abstraction.
  Shared metadata/occupancy checks remove the duplicated traversal where it
  matters without changing those APIs.
- **Feature envy:** resolved by moving finite cadence validity into
  `PlaybackTiming.validate`, alongside its shared alignment predicate. The
  annotator requests timing validation rather than inspecting timing fields
  itself during initialization.

## Specification axis: original report

No spec findings. The implementation matches the six tickets and approved specification: 301 scheduled captures with endpoint and interval-boundary behavior; overdue delivery without dropped samples; fresh CAS metadata and detection identifiers; editable motion and visibility; metric CAD-origin geometry and rotated box offsets; projected foreground masks; optional native visualization with final-scene retention; and unchanged beliefs/world contents. The setup and run guides describe the resulting behavior.

No missing or partial requirement, scope creep, or apparently implemented requirement with a demonstrably incorrect implementation was identified in the reviewed implementation diff.

Review evidence: production code, public-engine tests, existing runner/visualizer contracts, glossary, and ADRs. No tests were run by this reviewer. Native GUI coverage intercepts desktop boundaries; a manual desktop inspection was not performed.

## Validation

All 63 pre-existing scripted feature cases pass after the review refactor:

```sh
/tmp/tetris-pytest-configured \
  test/robokudo_test/test_scripted_tetris.py \
  test/robokudo_test/test_scripted_tetris_masks.py \
  test/robokudo_test/test_scripted_tetris_motion.py \
  test/robokudo_test/test_scripted_tetris_playback.py \
  test/robokudo_test/test_scripted_tetris_sequence.py \
  test/robokudo_test/test_scripted_tetris_visualization.py \
  --disable-warnings -q
```

Result: **63 passed, 19 warnings in 17.28 seconds**. The command ran from the
isolated review worktree using the configured interpreter, serialized execution,
a pre-run memory check, an 8 GiB virtual-memory cap, and `--orm-build never`.
The log is `/tmp/tetris-review-focused.log`. No existing failing test was edited;
all 63 cases were passing at the reviewed base and remain passing. All modified
Python files were processed with `scripts/format_docstrings.py`, and
`git diff --check` passes. Final integrated selection and real headless launcher
validation will be recorded by the integrating agent.
