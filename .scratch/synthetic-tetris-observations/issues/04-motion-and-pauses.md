# 04: Play translation, rotation, and motion pauses

**What to build:** A developer can edit visible block keyframes and inspect smooth
translation, rotation, and stationary pauses through the public engine's completed
observations. The first block moves while the second provides a stationary
reference, and geometry and capture times describe the same configured timeline.

**Blocked by:** 03: Run finite headless playback without losing observations.

**Status:** resolved

Implementation requires separate explicit user permission. Follow the approved
synthetic Tetris observation specification and domain glossary. Use existing
geometry and interpolation support and ask before adding dependencies.

- [x] The editable descriptor defines poses, motion durations, pauses, and sample
  timing. Use the agreed initial, translated, and rotated CAD-origin poses;
  translation moves the first block ten centimetres and rotation turns it positive
  ninety degrees about camera Z. The second block remains stationary.
- [x] Configured visible keyframes produce linear translation and shortest-path
  quaternion interpolation in xyzw order. Equivalent quaternion signs do not
  cause a longer rotation or discontinuity. Keyframe endpoints appear exactly
  on their scheduled captures.
- [x] Motion pauses continue producing scheduled observations with unchanged
  poses, fresh CAS instances and detection identifiers, and advancing capture
  times. Pauses do not stop sampling or restart playback.
- [x] Every emitted mesh, CAD-origin pose, and bounding box describes the same
  transformed object. A nonzero model-local box center rotates with the object
  rather than remaining at an unrotated offset.
- [x] Public-engine tests cover intermediate translations and rotations,
  endpoints, equivalent quaternion signs, motion pauses, and stationary reference
  poses. Expectations come from the configured scenario and consumed mesh
  fixtures, without calling private interpolation methods or copying production
  constants into assertions.
- [x] The agreed motion keeps the supplied blocks fully in frame and
  non-overlapping. Verify intermediate poses as well as endpoints, using
  deterministic fixture coverage in CI and conditional supplied-model coverage
  when those assets are available.
- [x] Documentation describes editing keyframes, movement durations, and pauses,
  including their relationship to the configured sample period and capture times.
- [x] Implement test-first with pytest and existing fixtures. Run focused checks
  with the configured interpreter and `--orm-build never`, checking and capping
  memory before each pytest run. Format modified Python docstrings using the
  repository formatter.


## Answer

Added editable `PoseKeyframe` descriptors to each `BlockState` and the public
`interpolate_pose` operation. The default first block holds through three seconds,
translates to its configured endpoint at nine seconds, holds through twelve
seconds, and rotates positive ninety degrees by eighteen seconds. The second
block stays stationary. Each initial, translated, and rotated pose is defined
once; hold keyframes reuse those definitions.

Translation is linear and orientation uses existing SciPy shortest-path quaternion
interpolation. Exact endpoints retain their configured representations. Both mesh
projection and offset bounding boxes receive the same scheduled pose. Invalid
keyframe order and unsampled endpoints raise `InvalidPoseKeyframes`; unused future
endpoints remain compatible with short playback schedules and exact nanosecond
sample periods. The editing guide documents keyframes, holds, and timing.

Validation through completed public-engine CAS observations covers midpoint
translations and rotations, exact endpoints, equivalent quaternion signs and the
170 to minus-170 degree short path, all three motion pauses, stationary reference
poses, rotating nonzero box-center offsets, and overdue delivery. Every default
capture stays fully in frame and non-overlapping with both deterministic fixture
and supplied meshes. Tests remain compatible with the later absence intervals.

Test-first slices proved missing motion and invalid-keyframe behavior before
implementation. With normal fixtures, `--orm-build never`, and an 8 GiB memory cap,
the combined static, mask, playback, motion, and pipeline run passed **46 tests**.
The final motion-only run passed **17 tests**; both runs reported 19 existing
dependency warnings. All modified Python files were formatted with
`scripts/format_docstrings.py`.
