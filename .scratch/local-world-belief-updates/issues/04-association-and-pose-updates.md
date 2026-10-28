# 04: Associate compatible models and preserve accepted geometry

Status: ready-for-agent
Type: task
Blocked by: 02, 03

Implementation permission: not granted. Do not claim or execute this ticket
without explicit user permission to start implementation.

## Outcome

Compatible observations update the existing physical-part belief while retaining
its body, identity, geometry, and semantic annotation.

Follow the [specification](../spec.md). Reuse the existing comparators, assignment,
and world-update machinery.

## Acceptance criteria

- [ ] Prevent known different model kinds from matching, even when other
  association evidence would favor the wrong belief.
- [ ] Keep association similarity distinct from classification confidence and
  introduce no new acceptance thresholds or uncertainty model.
- [ ] Preserve existing unmatched and low-similarity creation behavior, with
  existing unmatched beliefs retained.
- [ ] Accepted pose-only updates translate and rotate the existing body without
  replacing meshes or accumulating shapes or semantic annotations.
- [ ] Retain belief and body UUID, semantic annotation identity, class label, and
  geometry on successful matches. Detection IDs may change between observations.
- [ ] Absent objects retain their last accepted pose, capture time, and all other
  accepted object state. Empty observations make no object-state changes.
- [ ] Add test-first regressions for incompatible models, geometry-preserving
  pose updates, unmatched/low-similarity behavior, and absence.

## Verification

Run focused generic association and world-update tests, then relevant existing
connector tests, in separate pytest batches with memory controls and
`--orm-build never`. Format modified Python files and record results.

## Comments

- 2026-10-28: Specification ticket only; implementation is not authorized.
