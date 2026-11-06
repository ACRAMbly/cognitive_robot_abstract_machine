# Synthetic aircraft observations: feature map

Status: resolved

Implementation permission: granted by the user's explicit `/implement-spec`
request and subsequent `proceed` instruction. The specification's earlier
permission records describe the design phase.

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
- Implementation is authorized by the subsequent `/implement-spec` request.

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
| [01: Minimal aircraft CAS source](issues/01-minimal-aircraft-cas-source.md) | None; requires PR #13 interfaces | resolved |
| [02: Finite aircraft playback](issues/02-finite-aircraft-playback.md) | 01 | resolved |
| [03: Native 3D mesh verification](issues/03-native-mesh-verification.md) | 01, 02 | resolved |
| [04: PR #13 input compatibility](issues/04-pr13-input-compatibility.md) | 01, 02 | resolved |
| [05: Usage documentation and regressions](issues/05-documentation-and-regressions.md) | 01, 02, 03, 04 | resolved |

Status records the local tracker's claim and resolution state.

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
Implementation is now authorized. Verification results are recorded in each
ticket as work completes.

## Notes

The issue tracker is local Markdown, following `docs/agents/issue-tracker.md`.
Tickets are claimed and resolved here without opening a pull request.

## Decisions-so-far

- Use PR #13's existing `feature/local-world-belief-updates` branch as the
  integration branch, as clarified by the user. The implementation baseline
  is `812ded39bcdca29edec8c89c2082236432bd9b87`.
- The user approved installing missing verification prerequisites under `/tmp`.
  The configured workspace interpreter and existing fixtures remain in use.
- The user approved extracting generic playback helpers from Tetris modules,
  preserving Tetris behavior and existing tests, to avoid duplicating scheduling
  and interpolation in the aircraft source. This supersedes the specification's
  restriction against changing Tetris files for this extraction only.
- [Ticket 01](issues/01-minimal-aircraft-cas-source.md) supplies the minimal
  aircraft engine and explicit frame configuration. Eight new tests pass;
  seven existing Tetris tests fail in the unchanged color/depth metadata path.
- [Ticket 02](issues/02-finite-aircraft-playback.md) supplies shared finite
  scheduling and motion helpers and the complete aircraft timeline. All 39
  aircraft tests pass; Tetris rendering checks remain blocked by the same
  existing metadata failure.
- [Ticket 03](issues/03-native-mesh-verification.md) adds optional catalog mesh
  preview, including native final-scene controls. All 51 aircraft tests pass,
  including 12 preview checks; existing Tetris visualization failures retain
  the same metadata cause. No manual desktop inspection was performed.
- [Ticket 04](issues/04-pr13-input-compatibility.md) proves semantic acceptance,
  independently calculated world poses, exact capture metadata, and world
  isolation across the complete sequence. Four new tests and 76 existing
  semantic regressions pass; aircraft belief regressions pass after restoring
  the existing `vhacdx` prerequisite under `/tmp`.

- [Ticket 05](issues/05-documentation-and-regressions.md) documents aircraft-first
  usage and wires explicit launcher calibration and optional catalog preview.
  All 111 aircraft and 107 selected generic regressions pass; the documented
  finite headless run exits successfully. Existing Tetris metadata, package
  import, and query-teardown failures remain explicit in the completion record.
  Static typechecking and native desktop inspection were unavailable.

## Fog

No unresolved design question.

## Whole-feature review

The [review since `812ded39b`](review.md) found five standards issues, all
resolved, and no spec issues. The final aircraft batch passes 111 tests with
96.52% feature-module coverage; 107 selected generic regressions pass. Existing
Tetris metadata failures and unavailable typechecking remain explicit. The clean
temporary ticket 05 worktree and its merged task branch were removed.
