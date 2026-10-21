# 02: Synchronize accepted position and orientation changes

**What to build:** Subsequent scripted observations move and rotate both existing
object beliefs through SemanticDigitalTwinConnector. Native synchronization applies
the accepted poses to the same bodies in the receiving world, and RViz shows the
changed poses while preserving CAD geometry and persistent identity.

**Blocked by:** 01 — Display synchronized CAD blocks in RViz.

**Status:** ready-for-agent

- [ ] A runnable finite sequence includes the initial, translated, and rotated
  observations. Every fresh CAS retains the world frame and fixed camera transform.
- [ ] Accepted plain poses are transformed from camera to world exactly once. A
  nonidentity camera transform, including rotation, is covered by tests.
- [ ] The body frame remains the CAD frame. Rotation does not replace it with the
  enclosing box center or shift the mesh relative to its collision box.
- [ ] Association reuses each existing object belief and body UUID across position
  and orientation changes, without creating duplicate beliefs.
- [ ] Pose changes preserve the existing visual meshes and collision geometry and
  are delivered as state updates without recreating bodies or replacing geometry.
- [ ] Native synchronization produces matching source and receiving-world poses
  after the receiver applies the required stream position. Tests also verify that
  mesh dimensions and collision-box offsets remain unchanged.
- [ ] Observations without supplied CAD geometry retain the existing bounding-box
  behavior, with relevant connector regressions passing.
- [ ] Meaningful failing pytest tests precede each behavior change, use existing
  fixtures and isolated communication, require no inference services or credentials,
  and are included in CI.
- [ ] Launch and verification instructions explain the expected discrete pose
  changes in RViz. Separate-process updates are checked locally and any GUI
  inspection is distinguished from automated evidence.
- [ ] Existing uncommitted pose-update changes are reviewed against these criteria,
  and every modified Python file passes the required docstring formatter.
