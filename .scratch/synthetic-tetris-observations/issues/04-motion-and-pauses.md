# 04: Play translation, rotation, and motion pauses

**What to build:** A developer can edit visible block keyframes and inspect smooth
translation, rotation, and stationary pauses through the public engine's completed
observations. The first block moves while the second provides a stationary
reference, and geometry and capture times describe the same configured timeline.

**Blocked by:** 03: Run finite headless playback without losing observations.

**Status:** ready-for-agent

Implementation requires separate explicit user permission. Follow the approved
synthetic Tetris observation specification and domain glossary. Use existing
geometry and interpolation support and ask before adding dependencies.

- [ ] The editable descriptor defines poses, motion durations, pauses, and sample
  timing. Use the agreed initial, translated, and rotated CAD-origin poses;
  translation moves the first block ten centimetres and rotation turns it positive
  ninety degrees about camera Z. The second block remains stationary.
- [ ] Configured visible keyframes produce linear translation and shortest-path
  quaternion interpolation in xyzw order. Equivalent quaternion signs do not
  cause a longer rotation or discontinuity. Keyframe endpoints appear exactly
  on their scheduled captures.
- [ ] Motion pauses continue producing scheduled observations with unchanged
  poses, fresh CAS instances and detection identifiers, and advancing capture
  times. Pauses do not stop sampling or restart playback.
- [ ] Every emitted mesh, CAD-origin pose, and bounding box describes the same
  transformed object. A nonzero model-local box center rotates with the object
  rather than remaining at an unrotated offset.
- [ ] Public-engine tests cover intermediate translations and rotations,
  endpoints, equivalent quaternion signs, motion pauses, and stationary reference
  poses. Expectations come from the configured scenario and consumed mesh
  fixtures, without calling private interpolation methods or copying production
  constants into assertions.
- [ ] The agreed motion keeps the supplied blocks fully in frame and
  non-overlapping. Verify intermediate poses as well as endpoints, using
  deterministic fixture coverage in CI and conditional supplied-model coverage
  when those assets are available.
- [ ] Documentation describes editing keyframes, movement durations, and pauses,
  including their relationship to the configured sample period and capture times.
- [ ] Implement test-first with pytest and existing fixtures. Run focused checks
  with the configured interpreter and `--orm-build never`, checking and capping
  memory before each pytest run. Format modified Python docstrings using the
  repository formatter.
