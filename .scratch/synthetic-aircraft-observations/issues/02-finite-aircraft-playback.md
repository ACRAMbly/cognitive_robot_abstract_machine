# 02: Run the finite aircraft observation sequence

Status: resolved
Type: task
Blocked by: 01: Minimal aircraft CAS source.

Implementation permission: granted by the subsequent `/implement-spec` request.
The user also approved extracting shared playback helpers while preserving
Tetris behavior and its existing tests.

## Outcome

The public aircraft engine delivers the complete movement, absence, and
reappearance sequence at the established capture cadence without dropping samples.

## Acceptance criteria

- [x] Implement the established clock, scheduling, keyframe, and absence behavior
  in the separate aircraft source, reusing existing generic facilities where
  applicable. Leave the Tetris source and configuration unchanged.
- [x] Provide an editable default scenario for all aircraft kinds using the
  specification's thirty-second timeline and ten-Hz cadence, with 301 captures.
- [x] Keep aircraft layout, pose endpoints, and deterministic partial-absence
  membership in scenario definitions; no assembled-aircraft relationship is
  introduced.
- [x] Preserve linear translation, shortest-path rotation, exact endpoints,
  continuing observations during holds, and half-open absence boundaries.
- [x] Each completed capture replaces the CAS and writes all required metadata.
  Empty captures contain no stale hypotheses; reappearance retains last visible
  poses and creates fresh detection IDs.
- [x] Early ticks retain the last CAS; late ticks preserve every scheduled
  capture in order with its scheduled timestamp.
- [x] Emit stable waiting/completed/exhausted statuses through the public engine.
  Retain the final CAS without looping or creating an extra capture.
- [x] Test the complete default sequence, selected intermediate poses, holds,
  visibility boundaries, late delivery, and exhaustion using an injected clock
  without real-time sleeping. Preserve existing timing validation behavior.

## Verification

After permission, derive expected times, labels, subset membership, and poses
from consumed definitions. Assert fresh CAS instances and detection IDs even
during holds. Include zero-duration and configured shorter sequences through the
public engine. Run focused aircraft playback and existing Tetris motion/playback
regressions under the specification's verification requirements.

## Completion record

Implemented and verified through the public aircraft engine. Tetris's renderer,
models, assets, configuration, and tests are unchanged; only the user-approved
generic helper extraction touches its source modules.

## Answer

The default aircraft scenario now emits 301 scheduled captures at the approved
cadence. Every part has editable initial poses, keyframes, and half-open absence
intervals. `AircraftLayout` exposes translation distance, rotation angle, and a
deterministic editable partial-absence subset. The source retains completed CAS
objects while waiting and after exhaustion, preserves all overdue captures, and
rewrites explicit frame metadata even for empty captures.

Shared clocks, finite capture scheduling, motion interpolation, validation,
keyframes, absence intervals, and their exceptions live in
`robokudo_acrambly.playback`. Tetris's established type import paths re-export
these shared definitions. `BlockState` retains its constructor and inherits the
same algorithms as aircraft states through an explicit superclass; neither
observation source depends on the other's scenario or models.

Verification used `/home/nasimpcm/.virtualenvs/cram-env/bin/python` after
`source /tmp/aircraft-env.sh`. That script loads `/opt/ros/jazzy/setup.bash`, the
current worktree source directories, the preexisting RoboKudo environment's
dependencies, and the user-approved prerequisites in `/tmp/aircraft-prerequisites`.
Each pytest batch checked `free -m` (at least 11,860 MiB available), checked for
competing pytest processes, ran sequentially with `ulimit -v 6291456`, and passed
`--orm-build never`. No ORM regeneration was needed.

- Red: the new early-tick public-engine test failed because the source returned
  `SUCCESS` instead of `RUNNING` (`/tmp/aircraft-02-red-waiting.log`). After shared
  scheduling was connected, aircraft tests passed: 9 passed.
- Red: public full-sequence visibility coverage failed because aircraft states
  had no absence intervals (`/tmp/aircraft-02-red-visibility.log`: 1 failed,
  2 passed). After shared motion and configured visibility were connected,
  aircraft tests passed: 11 passed.
- Final: `python -m pytest --orm-build never
  test/robokudo_acrambly_test/test_scripted_aircraft_playback.py
  test/robokudo_acrambly_test/test_scripted_aircraft.py -q`: **39 passed**
  (`/tmp/aircraft-02-final-aircraft.log`). Coverage includes the complete timeline,
  exact keyframes, independently calculated translation and rotation midpoints,
  both quaternion signs across the shortest turn, fresh identifiers during holds
  and reappearance, motion during invisibility, every capture's metadata,
  nanosecond waiting boundaries, overdue captures, zero duration, stable
  exhaustion, and invalid timing/keyframe/absence configuration.
- Regression: `python -m pytest --orm-build never
  test/robokudo_acrambly_test/test_scripted_tetris_motion.py
  test/robokudo_acrambly_test/test_scripted_tetris_playback.py
  test/robokudo_acrambly_test/test_scripted_tetris_sequence.py -q`:
  **14 passed, 12 failed, 13 errors**. Every failure/error is the existing
  `KeyError: CASViews.COLOR_TO_DEPTH_RATIO` in the unchanged image projection
  path (`/tmp/aircraft-02-tetris-regressions.log`). Ticket 01 already observed
  this failure before extraction. Rendering assertions therefore remain
  unverified in this environment; no existing tests were modified.
- `scripts/format_docstrings.py` ran on all 13 modified/new Python files;
  its final repeat was stable. `git diff --check` passed.

No native display, network calls, credentials, or new application dependencies
are required by the added tests. Native preview is handled by ticket 03.
