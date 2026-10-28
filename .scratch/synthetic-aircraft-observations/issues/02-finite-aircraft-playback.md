# 02: Run the finite aircraft observation sequence

Status: ready-for-agent
Type: task
Blocked by: 01: Minimal aircraft CAS source.

Implementation permission: not granted. Follow the approved
[specification](../spec.md), especially its scenario and playback contract.

## Outcome

The public aircraft engine delivers the complete movement, absence, and
reappearance sequence at the established capture cadence without dropping samples.

## Acceptance criteria

- [ ] Implement the established clock, scheduling, keyframe, and absence behavior
  in the separate aircraft source, reusing existing generic facilities where
  applicable. Leave the Tetris source and configuration unchanged.
- [ ] Provide an editable default scenario for all aircraft kinds using the
  specification's thirty-second timeline and ten-Hz cadence, with 301 captures.
- [ ] Keep aircraft layout, pose endpoints, and deterministic partial-absence
  membership in scenario definitions; no assembled-aircraft relationship is
  introduced.
- [ ] Preserve linear translation, shortest-path rotation, exact endpoints,
  continuing observations during holds, and half-open absence boundaries.
- [ ] Each completed capture replaces the CAS and writes all required metadata.
  Empty captures contain no stale hypotheses; reappearance retains last visible
  poses and creates fresh detection IDs.
- [ ] Early ticks retain the last CAS; late ticks preserve every scheduled
  capture in order with its scheduled timestamp.
- [ ] Emit stable waiting/completed/exhausted statuses through the public engine.
  Retain the final CAS without looping or creating an extra capture.
- [ ] Test the complete default sequence, selected intermediate poses, holds,
  visibility boundaries, late delivery, and exhaustion using an injected clock
  without real-time sleeping. Preserve existing timing validation behavior.

## Verification

After permission, derive expected times, labels, subset membership, and poses
from consumed definitions. Assert fresh CAS instances and detection IDs even
during holds. Include zero-duration and configured shorter sequences through the
public engine. Run focused aircraft playback and existing Tetris motion/playback
regressions under the specification's verification requirements.

## Completion record

Pending explicit implementation permission. Record red/green evidence, focused
regression results, and any limitations after implementation.
