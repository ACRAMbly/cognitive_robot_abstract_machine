# 03: Play smooth repeating motion in RViz

**What to build:** The scripted demonstration sends intermediate accepted poses
through the connector and native synchronization so both CAD blocks move smoothly
in the receiving world's RViz display. Playback pauses at scene endpoints and
returns smoothly to the starting poses before repeating.

**Blocked by:** 02 — Synchronize accepted position and orientation changes.

**Status:** ready-for-agent

- [ ] Translation follows a straight path between visible poses of the same block;
  orientation follows the shortest quaternion rotation. Keyframe endpoints are
  reached without changing object identity or CAD geometry.
- [ ] Playback configuration exposes observation interval, motion duration, and
  pause duration with a defined sampling policy and validation of invalid values.
  The default cadence visibly supports smooth motion in RViz.
- [ ] Scheduling and capture timestamps describe the same configured timeline.
  Timestamps continue increasing through repeats, and waiting ticks do not publish
  a fresh observation or reprocess the preceding observation as new.
- [ ] Endpoint pauses are observable, and the visible return path reaches the
  initial poses before the next cycle without a pose jump at the repeat boundary.
- [ ] Interpolation only connects compatible visible detections. It does not invent
  detections during absence or interpolate an invisible block into reappearance.
- [ ] A finite playback option completes predictably and the repeating option
  continues until application shutdown.
- [ ] Failing pytest tests precede implementation and verify intermediate
  translations and rotations, endpoint timing, monotonic capture timestamps,
  repeat continuity, and matching sampled poses in an independent receiving world
  after synchronization has applied them.
- [ ] Tests use a controllable clock and existing fixtures, require no inference
  services or credentials, and run in CI. Existing observation and connector
  regressions remain passing.
- [ ] Existing uncommitted interpolation changes are reviewed against these
  criteria, and modified Python files pass the required docstring formatter.
- [ ] Documentation gives timing configuration and a repeat-cycle visual check.
  Separate-process motion is exercised locally, with GUI inspection reported
  separately from automated synchronization checks.
