# 05: Prove the six-stage aircraft-part acceptance sequence

Status: ready-for-agent
Type: task
Blocked by: 01, 02, 03, 04

Implementation permission: not granted. Do not claim or execute this ticket
without explicit user permission to start implementation.

## Outcome

A deterministic local pipeline proves acceptance of all 12 configured aircraft
part kinds through movement, absence, and reappearance.

Follow the [specification's acceptance matrix](../spec.md#acceptance-matrix).

## Acceptance criteria

- [ ] Build fresh CAS observations through public types with fixture-owned
  capture times, frame metadata, poses, and changing detection identifiers.
- [ ] Use the existing connector in a local pipeline with initialized world and
  camera frames and a nonidentity camera transform containing rotation and
  translation.
- [ ] Exercise all present, translation, rotation, partial absence, total absence,
  and reappearance, using all 12 configured model kinds.
- [ ] Assert exact observation and belief counts from the acceptance matrix,
  retained UUIDs and semantic annotations, accepted poses, and geometry in both
  collections. Derive expected geometry from configured resources.
- [ ] Compare absent-object state, including last accepted capture time, against
  its prior accepted state; reappearance updates existing beliefs without
  duplicates or accumulated geometry.
- [ ] Keep fixtures deterministic and isolated using existing ROS, world, CAS,
  and blackboard fixtures. Include actual configured plane-model coverage in CI.
- [ ] Do not add a production observation source, detector, assembly layout,
  synchronization process, or the declined duplicate-copy test case.

## Verification

Keep aircraft-specific imports in `test/robokudo_acrambly_test`. Implement
test-first; run focused application tests with memory controls and
`--orm-build never`. Format modified Python files and record results.

## Comments

- 2026-10-28: Specification ticket only; implementation is not authorized.
