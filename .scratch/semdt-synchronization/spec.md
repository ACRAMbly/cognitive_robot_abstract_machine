# Scripted observations to a synchronized RViz world

Status: needs-triage

## Outcome

Provide a runnable synthetic demonstration of the path in `z.md`:

Scripted observations → CAS object hypotheses → SemanticDigitalTwinConnector →
RoboKudo belief world → native WorldSynchronizer → separate receiving world → RViz.

Use the integration guidance in `x.md` for coordinate frames, geometry preservation,
persistent identity, and synchronization. External Noctis/FoundationPose workers,
Coraplex plans, and Giskard control are outside this demonstration.

## Confirmed behavior

- Both supplied Tetris models appear as CAD meshes with enclosing collision boxes.
- Translation and rotation include intermediate poses, with pauses at scene changes.
- Repeating the sequence includes a smooth return to the initial poses.
- A missing detection retains the last accepted object belief and its UUID.
- Reappearance reuses the existing belief rather than creating a duplicate body.
- RViz displays the receiving world's geometry and transforms in a separate process.
- Meaningful pytest coverage precedes each implementation change.

## Implementation plan

### 1. Define the application lifecycle and launch contract

- Keep the existing observation-only analysis engine available.
- Add a synchronization demonstration engine that uses the scripted source followed
  by the existing SemanticDigitalTwinConnector.
- Establish one source world, its root frame, the fixed camera transform, and one
  native WorldSynchronizer before publishing world changes.
- Make synchronization resources application-owned and explicitly closed on shutdown.
- Publish RViz geometry and TF from the receiving process, avoiding competing source
  and receiver TF publishers for the same frames.
- Settle receiver startup before building it. Proposed initial contract: start the
  receiver before the source, and fail clearly if the required receiver is missing.
  Supporting late joins or receiver restarts requires a consistent snapshot and
  stream-position handoff; the existing fetch helper alone does not establish that.

Deliverable: documented entry points, ownership, frames, and startup order.

### 2. Produce smooth observations

- Preserve the existing six scene keyframes and visibility semantics.
- Interpolate translation linearly and orientation along the shortest quaternion
  rotation between visible poses of the same block.
- Configure motion duration, pause duration, and observation interval together so
  playback speed and capture timestamps describe the same timeline.
- Keep disappearance and reappearance discrete; interpolate the visible return path
  before repeating.

Tests: intermediate translation and rotation, endpoint poses, absence pauses,
monotonic capture times, and continuity at the repeat boundary.

### 3. Preserve CAD geometry through accepted belief updates

- Reuse existing shape annotations to carry metric CAD geometry where practical.
- Initialize visual mesh and collision box before adding each perceived body to
  the source world, so recorded structural changes contain its geometry.
- Treat the body frame as the supplied CAD frame; keep the collision box's center
  offset relative to that frame.
- Apply camera-to-world conversion exactly once to the accepted plain pose.
- Preserve visual and collision geometry during pose-only updates.
- Preserve the existing bounding-box behavior for observations without CAD geometry.

Tests: creation with mesh and collision box, nonidentity camera transform, rotation
about the CAD origin, geometry preservation, and existing connector regressions.

### 4. Connect the pipeline and prove native replication

- Supply world frame and fixed camera-to-world transform on every fresh CAS.
- Configure the existing association machinery for these two distinct models.
- Exercise motion, first-block absence, both-block absence, and reappearance through
  the complete pipeline.
- Receive native structural and state updates into an independent SDT World.
- Wait for the receiver to apply the source stream position before comparing worlds.

Tests: matching poses, mesh dimensions, collision geometry, persistent UUIDs, no
duplicate beliefs, and unchanged poses during absent observations. Use isolated ROS
topics and existing fixtures; require no camera, inference services, or credentials.

### 5. Provide the receiving process and RViz configuration

- Add the independent receiver entry point using native WorldSynchronizer.
- Start SDT visualization and TF publication once the receiving root exists.
- Provide an RViz configuration with the receiving marker topic, appropriate QoS,
  and the agreed root as Fixed Frame.
- Report scene progression and enough identity information to verify retained
  beliefs during absent detections.
- Close ROS publishers, subscriptions, callbacks, executors, and nodes in ownership
  order on shutdown.

Deliverable: commands for both processes and RViz, with a short verification checklist.

### 6. Validate and document

- Run targeted pytest checks with the current project interpreter and ROS overlays.
- Run existing scripted-observation and connector tests to catch regressions.
- Run `scripts/format_docstrings.py` on every modified Python file.
- Exercise source and receiver as separate processes and confirm that updates arrive.
- Check RViz when a GUI is available; distinguish automated synchronization evidence
  from visual inspection in the final report.
- Document startup/restart limits, configuration, expected absence behavior, and
  exact launch commands in `robokudo/scripted_tetris.md`.

## Current partial work

Implementation began before this written plan was prepared. The uncommitted changes
currently include interpolation, mesh shape annotations, CAD-preserving belief
creation and pose updates, glossary terms, and a new test module.

The last completed test run passed 25 tests across the scripted observations,
connector, and the first two new tests. A subsequent integration test was written
against a proposed synchronization engine that does not exist yet; that test has not
been validated. The interrupted run does not establish an end-to-end result.

No synchronization engine, receiver application, RViz configuration, or launch
documentation has been implemented. Existing partial code must be reviewed against
this plan before further implementation.
