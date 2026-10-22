# 05: Complete the default disappearance and reappearance sequence

**What to build:** One headless invocation produces the complete agreed
thirty-second Tetris observation sequence: both blocks appear, the first
translates and rotates, blocks disappear, an empty interval is observed, and both
reappear at their last visible poses. The finite run emits exactly 301 completed
observations and then exits.

**Blocked by:** 04: Play translation, rotation, and motion pauses.

**Status:** resolved

Implementation requires separate explicit user permission. Follow the approved
synthetic Tetris observation specification and domain glossary. Visibility
playback can proceed independently of foreground-mask and native-visualization
work; this ticket retains their existing observation interfaces.

- [x] The default descriptor uses ten observations per second from zero through
  thirty seconds inclusive. Timing stays editable and interval durations align
  with the sample period.
- [x] The default sequence uses the agreed intervals: both initial poses from
  zero to three seconds; translation from three to nine; a translated-state pause
  from nine to twelve; rotation from twelve to eighteen; a rotated-state pause
  from eighteen to twenty-one; only the second block from twenty-one to
  twenty-four; no blocks from twenty-four to twenty-seven; and both blocks
  reappearing at their last visible poses from twenty-seven through thirty.
- [x] Each boundary emits the new interval's state once. Appearance and
  disappearance are discrete, and reappearance introduces no invented motion,
  extra return movement, or looping.
- [x] Single-block and empty observations receive fresh CAS instances with the
  expected capture time, frame, image dimensions, and camera intrinsics. Absent
  blocks leave no stale hypotheses, and reappearing detections receive fresh
  identifiers without assigning persistent physical identity.
- [x] Public-engine pytest coverage runs the complete default sequence without
  real-time sleeps and verifies exactly 301 distinct completed observations,
  strictly increasing scheduled capture times spanning thirty seconds, interval
  membership, motion and pause states, absence, and reappearance. Derive expected
  poses and timing from the scenario definitions that the engine consumed.
- [x] Late-tick coverage preserves the complete scheduled sequence. Early and
  exhausted ticks add no observations; the final observation appears once and
  remains accessible after exhaustion. Headless termination follows the runner
  contract already established by the playback ticket.
- [x] The complete run leaves persistent beliefs and world contents unchanged
  from an isolated baseline. Tests stop at emitted hypotheses; connector
  association, world updates, and synchronization remain outside scope.
- [x] Documentation describes the complete default timeline, illustrative poses,
  expected observation count, output access, and finite headless completion.
- [x] Implement test-first with existing fixtures and deterministic local meshes.
  All core checks run in RoboKudo CI without untracked assets, services, or
  credentials. Run focused checks and relevant pipeline tests with the configured
  interpreter and `--orm-build never`, checking and capping memory before each
  pytest run. Format modified Python docstrings using the repository formatter.

## Answer

Added editable `AbsenceInterval` descriptors to each `BlockState`. The default
first block is absent during `[21 s, 27 s)` and the second during `[24 s, 27 s)`.
Visibility uses scheduled capture offsets independently of pose keyframes, so
both blocks reappear at their last visible poses without extra motion. Invalid
negative, reversed, empty, or unsampled intervals raise `InvalidAbsenceInterval`;
future endpoints remain compatible with shortened exact-nanosecond schedules.

Every completed capture receives a fresh CAS and camera metadata before absent
blocks are excluded from geometry and image evidence. The complete run produces
301 observations: 241 with both blocks, 30 with only the reference, and 30 empty.
Native visualization publication remains after the block loop, including empty
captures. The guide documents the complete timeline and editing visibility.

Test-first slices proved the missing first disappearance, missing empty interval,
and invalid-interval acceptance before implementing each behavior. The new public
engine sequence suite passes **13 tests**, covering all default captures,
half-open boundaries, exact reappearance poses, distinct CAS objects and IDs,
unique deterministic detection identifiers, capture and construction times,
camera calibration in empty scenes, regular and late delivery, early waiting,
stable exhaustion, editable intervals, and unchanged world/belief baselines.

After merging ticket 06, all six scripted-observation suites pass **63 tests**.
Existing pipeline, CAS, garden, and type-conversion checks pass **23 tests** in a
separate run. Runs used the configured interpreter, `--orm-build never`, the
serialized memory-checking helper, and an 8 GiB memory cap; each reported the same
19 existing dependency warnings. All modified Python files have a stable pass of
`scripts/format_docstrings.py`.
