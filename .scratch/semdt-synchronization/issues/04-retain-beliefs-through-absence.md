# 04: Retain beliefs through absence and reappearance

**What to build:** The demonstration makes the first block absent, then both blocks
absent, and finally observes both again. Perception reports the missing detections,
while the receiving world retains the last accepted poses. Reappearance updates the
same object beliefs and UUIDs without duplicate bodies.

**Blocked by:** 02 — Synchronize accepted position and orientation changes.

**Status:** ready-for-agent

- [ ] The runnable scenario exercises first-block absence, both-block absence, and
  reappearance through the complete observation, connector, and synchronization
  path. This works with discrete motion independently of smooth playback.
- [ ] Every completed absent observation has a fresh CAS and increasing capture
  time. Its detections exclude the absent blocks without retaining annotations
  from the previous observation.
- [ ] Source and receiving worlds retain each absent block's UUID, visual mesh,
  collision geometry, and last accepted pose; absence alone causes no removal.
- [ ] Reappearance associates each detection with its previous object belief,
  applies its accepted pose, and creates no duplicate belief or receiving-world body.
- [ ] Scene reporting identifies the visibility stage and provides stable identity
  information so the user can distinguish missing detections from retained beliefs.
  The CAS preview reflects detections; RViz reflects the receiving world's beliefs.
- [ ] Failing pytest tests precede implementation and verify the complete visibility
  cycle, unchanged absent poses, persistent per-block identities, and no duplicates.
  Comparisons wait for the relevant source stream position to be applied.
- [ ] Coverage includes repeated visibility cycles and composition with smooth
  playback when enabled, without making ticket 03 a prerequisite for the discrete
  visibility behavior.
- [ ] Tests reuse existing fixtures, isolate ROS communication, require no inference
  services or credentials, run in CI, and leave relevant existing regressions passing.
- [ ] Verification instructions describe what remains visible in RViz during
  absence, how scene reporting confirms missing detections, and how to check
  identity reuse after reappearance. The complete cycle is exercised across the
  separate source and receiving processes.
- [ ] Modified Python files pass the required docstring formatter, and automated
  evidence is distinguished from any manual GUI inspection.
