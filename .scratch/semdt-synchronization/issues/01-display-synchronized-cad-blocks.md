# 01: Display synchronized CAD blocks in RViz

**What to build:** A runnable static demonstration in which one scripted observation
of both Tetris blocks passes through SemanticDigitalTwinConnector into the perception
world, then through native WorldSynchronizer into a separate receiving process.
RViz displays the receiving world's CAD meshes and transforms.

**Blocked by:** None (can start immediately).

**Status:** ready-for-agent

- [x] Both supplied object models appear at their initial accepted poses as metric
  CAD meshes with enclosing collision boxes. The CAD origins and collision-box
  center offsets are preserved.
- [x] The source initializes its world root and fixed camera frame, attaches native
  synchronization before publishing structural changes, and provides the world
  frame and camera-to-world transform on the fresh CAS.
- [x] Body-creation changes include visual and collision geometry when recorded;
  the receiving world reconstructs geometry without accessing source-world objects.
- [x] The receiver is a separate process with an independent SDT World and receives
  accepted changes through WorldSynchronizer. RViz geometry and TF are published
  by the receiving process without competing source publishers for those frames.
- [x] The approved launch contract requires the receiver before the source. The
  source verifies receiver readiness before publishing initial changes and reports
  a clear startup failure if the required receiver is unavailable. Its own
  synchronization subscription does not count as receiver readiness.
- [x] A supplied RViz configuration selects the receiving marker topic, compatible
  durability, and the correct Fixed Frame. Launch instructions cover ROS overlays,
  the current project interpreter, startup order, and the static visual check.
- [x] Startup and shutdown explicitly manage application-owned synchronizers,
  visualization callbacks, publishers, subscriptions, executors, and ROS nodes.
- [x] Failing pytest tests precede implementation and verify initial geometry and
  poses in an independent receiving world, waiting for the relevant source stream
  position before comparison. Startup failure and shutdown behavior are covered.
- [x] Tests reuse existing fixtures, isolate ROS communication, require no inference
  services or credentials, and run in the existing CI suite. Separate-process
  replication is checked locally; GUI inspection is reported separately.
- [x] Relevant existing observation and connector tests pass, the observation-only
  analysis engine remains available, and modified Python files pass the required
  docstring formatter.
- [x] Existing uncommitted CAD and belief-creation changes are reviewed as candidate
  implementation; the unfinished integration test is reconciled with the delivered
  public interface so this ticket leaves the relevant suite passing.

## Comments

The user approved this ticket breakdown and receiver-first startup. Late joins and
receiver restarts are outside the initial contract; instructions must explain how
to restart the demonstration from a consistent initial world.

### Implementation and validation — 2026-10-21

Implemented the `scripted_tetris_sync` analysis engine and dedicated
`python -m robokudo.scripts.scripted_tetris_sync receiver|source` lifecycle entry
point. The command accepts only the initial observation and keeps both applications
running until interrupted. The observation-only engine remains available. Existing
CAD-belief changes are retained, and the unfinished integration test now exercises
the implemented engine with valid, isolated ROS topic names.

The source requires a peer native publisher and a matched subscription on the
dedicated synchronization stream before creating world structure. Its own native
subscription and a standalone diagnostic listener each fail the readiness check.

Validation using the current `.venv-foundation-pose/bin/python` interpreter and
both ROS overlays:

- 32 pytest tests passed across `test_scripted_twin.py`,
  `test_scripted_tetris_processes.py`, `test_scripted_observations.py`, and
  `test_semantic_world_connector.py`.
- Independent-world checks wait for the relevant source stream position and verify
  metric CAD bounds, collision-box dimensions and center offsets, accepted poses,
  nonidentity camera conversion, and identity retention through absence.
- The subprocess test launches separate source and receiver applications, receives
  CAD mesh markers, collision boxes, and final accepted transforms, verifies that
  TF comes only from the receiver, and requires orderly exit from both processes.
- Receiver callback and ROS-entity cleanup, unavailable-receiver startup cleanup,
  and borrowed-node preservation are covered.
- All eight modified Python files were processed with
  `scripts/format_docstrings.py`.

GUI verification was performed separately: launched the two applications and RViz
with `robokudo/scripted_tetris.rviz`, inspected both CAD meshes, enclosing
translucent boxes, and coordinate frames, and confirmed RViz's global status was
`Ok`. The inspected window capture is `/tmp/scripted-tetris-rviz.png`. All three
applications were closed after inspection.

Launch and restart instructions are in `robokudo/scripted_tetris.md`. Late joins
and receiver restart recovery remain outside ticket 01's approved contract.
