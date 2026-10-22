# 05: Complete the default disappearance and reappearance sequence

**What to build:** One headless invocation produces the complete agreed
thirty-second Tetris observation sequence: both blocks appear, the first
translates and rotates, blocks disappear, an empty interval is observed, and both
reappear at their last visible poses. The finite run emits exactly 301 completed
observations and then exits.

**Blocked by:** 04: Play translation, rotation, and motion pauses.

**Status:** claimed

Implementation requires separate explicit user permission. Follow the approved
synthetic Tetris observation specification and domain glossary. Visibility
playback can proceed independently of foreground-mask and native-visualization
work; this ticket retains their existing observation interfaces.

- [ ] The default descriptor uses ten observations per second from zero through
  thirty seconds inclusive. Timing stays editable and interval durations align
  with the sample period.
- [ ] The default sequence uses the agreed intervals: both initial poses from
  zero to three seconds; translation from three to nine; a translated-state pause
  from nine to twelve; rotation from twelve to eighteen; a rotated-state pause
  from eighteen to twenty-one; only the second block from twenty-one to
  twenty-four; no blocks from twenty-four to twenty-seven; and both blocks
  reappearing at their last visible poses from twenty-seven through thirty.
- [ ] Each boundary emits the new interval's state once. Appearance and
  disappearance are discrete, and reappearance introduces no invented motion,
  extra return movement, or looping.
- [ ] Single-block and empty observations receive fresh CAS instances with the
  expected capture time, frame, image dimensions, and camera intrinsics. Absent
  blocks leave no stale hypotheses, and reappearing detections receive fresh
  identifiers without assigning persistent physical identity.
- [ ] Public-engine pytest coverage runs the complete default sequence without
  real-time sleeps and verifies exactly 301 distinct completed observations,
  strictly increasing scheduled capture times spanning thirty seconds, interval
  membership, motion and pause states, absence, and reappearance. Derive expected
  poses and timing from the scenario definitions that the engine consumed.
- [ ] Late-tick coverage preserves the complete scheduled sequence. Early and
  exhausted ticks add no observations; the final observation appears once and
  remains accessible after exhaustion. Headless termination follows the runner
  contract already established by the playback ticket.
- [ ] The complete run leaves persistent beliefs and world contents unchanged
  from an isolated baseline. Tests stop at emitted hypotheses; connector
  association, world updates, and synchronization remain outside scope.
- [ ] Documentation describes the complete default timeline, illustrative poses,
  expected observation count, output access, and finite headless completion.
- [ ] Implement test-first with existing fixtures and deterministic local meshes.
  All core checks run in RoboKudo CI without untracked assets, services, or
  credentials. Run focused checks and relevant pipeline tests with the configured
  interpreter and `--orm-build never`, checking and capping memory before each
  pytest run. Format modified Python docstrings using the repository formatter.
