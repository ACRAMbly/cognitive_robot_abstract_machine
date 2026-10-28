# 01: Emit minimal aircraft CAS observations

Status: ready-for-agent
Type: task
Blocked by: None; the implementation base must contain PR #13's interfaces.

Implementation permission: not granted. Follow the approved
[specification](../spec.md); this ticket authorizes no implementation by itself.

## Outcome

A public `scripted_aircraft` engine emits a configured static observation of the
twelve aircraft-part kinds, with the minimal payload and capture/frame metadata
required by PR #13. Existing `scripted_tetris` behavior remains available.

## Acceptance criteria

- [ ] Use the existing aircraft inventory and structured editable scene/frame
  configuration; retain existing model labels, units, and mesh-origin meaning.
- [ ] Emit one `ObjectHypothesis` per visible part containing its classification,
  plain camera-relative pose, and deterministic detection ID.
- [ ] Write scheduled integer capture time, distinct world/camera frame IDs, and
  an explicit valid camera-to-world transform into each new CAS, including an
  empty scene. Keep the reusable configuration outside the replaced CAS.
- [ ] Require explicit frame/transform configuration rather than silently
  manufacturing identity calibration. Caller-owned frame initialization remains
  separate from source construction and capture.
- [ ] Emit the specification's minimal payload without populated masks/regions,
  shape or box annotations, camera calibration, images, depth, or point clouds.
- [ ] Source construction and capture leave persistent world state unchanged.
  No connector is included in the source-only engine.
- [ ] Keep the aircraft engine, scenario, observation source, and output contract
  separate from Tetris. Reuse existing generic helpers where applicable, without
  importing Tetris-specific source components or modifying its implementation.
- [ ] Add failing public-engine pytest coverage before implementation, asserting
  configured labels, poses, exact metadata, excluded payload, and world isolation.

## Verification

After permission, use existing application ROS, blackboard, CAS, and world
fixtures. Validate a single-capture configuration and an empty configuration
without a display, mesh-payload construction, or external services. Run focused
application and existing static Tetris/pipeline regressions under the
specification's interpreter, memory, ORM, and formatter requirements.

## Completion record

Pending explicit implementation permission. Record the public boundary, initial
failing test, final checks, changed files, and limitations after implementation.
