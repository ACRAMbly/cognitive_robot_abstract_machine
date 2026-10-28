# 04: Prove compatibility with PR #13's aircraft input

Status: ready-for-agent
Type: task
Blocked by: 01: Minimal aircraft CAS source; 02: Finite aircraft playback.

Implementation permission: not granted. Follow the approved
[specification](../spec.md) and PR #13's linked observation contract.

## Outcome

Actual CAS observations emitted by the public aircraft engine satisfy existing
semantic acceptance without masks, regions, boxes, or per-detection meshes.

## Acceptance criteria

- [ ] Generate observations through the public aircraft analysis engine rather
  than substituting the existing connector-test sequence for the new source.
- [ ] Reuse the public PR #13 acceptance interfaces and existing application
  fixture configuration for all twelve model kinds.
- [ ] Preinitialize fixture world/camera frames before the isolation baseline.
  Configure a camera transform containing translation and rotation.
- [ ] Verify emitted labels and camera-relative mesh-origin poses resolve to the
  expected semantic types and independently calculated world poses, retaining
  exact integer capture timestamps.
- [ ] Cover completed captures across movement, holds, partial absence, total
  absence, and reappearance. Empty CAS observations retain valid frame/capture
  metadata without requiring hypotheses to trigger validation.
- [ ] Run compatibility checks with the minimal payload and no preview data.
  Verify the caller-supplied frame configuration is present after CAS replacement.
- [ ] Source execution and compatibility preparation leave persistent beliefs,
  world bodies/connections, and semantic annotations unchanged from the baseline.
- [ ] Keep persistent belief creation, association, UUID retention, and geometry
  update tests in PR #13's existing suite; this ticket does not add a connector
  stage or duplicate those acceptance tests.
- [ ] Establish any missing source behavior with a failing test before fixing
  production code. Preserve existing failing tests and generic acceptance rules.

## Verification

After permission, keep aircraft-specific imports within application tests and
use existing ROS/world/CAS fixtures. Run the new source-compatibility tests and
existing aircraft/semantic acceptance regressions in bounded, serialized batches.
Follow the specification's interpreter, memory, ORM, and formatting requirements.

## Completion record

Pending explicit implementation permission. Record exact compatibility and
regression results and any preexisting environment limitations after execution.
