# Synthetic aircraft observations: feature map

Status: specified

Implementation permission: not granted. The user authorized specifications only;
explicit permission is required before implementing any ticket.

## Approved design

The [specification](spec.md) is the authoritative requirement record. It narrows
the aircraft source to PR #13's required model/pose and capture/frame contract,
with optional native 3D mesh verification. Existing Tetris behavior is retained.

The design interview settled:

- Individual aircraft parts, using the existing twelve-kind aircraft inventory.
- CAS observations only; belief behavior remains in PR #13.
- Separate `scripted_aircraft` and `scripted_tetris` engines, scenarios, sources,
  output contracts, and previews; existing Tetris implementation stays unchanged.
- Existing finite playback and runner lifecycle.
- No aircraft 3D bounding boxes.
- The final PR #13 payload replaces the earlier masks/regions/mesh payload;
  catalog meshes are used for optional visual verification.
- Specifications are authorized; implementation is not.

## Design tree

- Aircraft semantics: reuse PR #13.
  - Existing labels, resources, units, mesh-origin poses, and twelve part kinds.
  - Existing six-stage acceptance semantics; no assembled layout or persistent
    identity in source output.
- Production source: application-owned, observation-only.
  - Fresh CAS, camera-relative poses, capture time, frames, explicit transform.
  - Frame setup belongs to the caller; source writes metadata each capture.
  - Finite, editable playback follows the established scheduling and keyframe
    behavior in the separate aircraft source.
- Presentation: optional catalog-based native 3D preview.
  - No image evidence, box annotations, or per-detection mesh requirement.
  - Final scene retained in the viewer; headless completion exits.
- Source separation: aircraft runs independently; Tetris stays unchanged.
  - Generic helpers may be reused; Tetris-specific source code is not a dependency
    of aircraft generation, and there is no combined mode or Tetris refactor.
- PR #13 input compatibility: consume its existing observation interfaces.
  - Verify source observations through public acceptance without adding a belief
    stage to the source-only engine.

## Implementation tickets

| Ticket | Dependencies | Status |
| --- | --- | --- |
| [01: Minimal aircraft CAS source](issues/01-minimal-aircraft-cas-source.md) | None; requires PR #13 interfaces | ready-for-agent |
| [02: Finite aircraft playback](issues/02-finite-aircraft-playback.md) | 01 | ready-for-agent |
| [03: Native 3D mesh verification](issues/03-native-mesh-verification.md) | 01, 02 | ready-for-agent |
| [04: PR #13 input compatibility](issues/04-pr13-input-compatibility.md) | 01, 02 | ready-for-agent |
| [05: Usage documentation and regressions](issues/05-documentation-and-regressions.md) | 01, 02, 03, 04 | ready-for-agent |

Status describes specification readiness, not permission to execute a ticket.

## Baseline and remaining work

The inspected checkout is PR #13 head
`3b86514b78521115fc1ed427dcbb77b23c43d023` on
`feature/local-world-belief-updates`. The existing Tetris source supplies capture
and camera metadata but omits the world frame and camera-to-world transform;
its viewer assumes masks and boxes. These are implementation differences for the
new aircraft source, not missing aircraft semantics in PR #13. They do not call
for changes to the Tetris source or its viewer.

No requirement question remains open. Aircraft pose values, layout, and resource
configuration wiring are implementation details constrained by the specification.
All production changes, tests, fixture changes, and implementation verification
remain pending explicit permission. No application behavior has been changed or
tested by creating these documents.
