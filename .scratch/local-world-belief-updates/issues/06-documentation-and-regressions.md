# 06: Document configuration and complete focused regressions

Status: ready-for-agent
Type: task
Blocked by: 01, 02, 03, 04, 05

Implementation permission: not granted. Do not claim or execute this ticket
without explicit user permission to start implementation.

## Outcome

Maintainers can configure aircraft acceptance, understand its local-world
boundary, replace mesh resources later, and review exact validation evidence.

Follow the [specification](../spec.md) and the fork-only workflow in the connector
brief after implementation is explicitly authorized.

## Acceptance criteria

- [ ] Document the actual implemented configuration interface, model mapping,
  mesh-directory setup, metadata requirements, and test/pipeline usage.
- [ ] Explain mesh-origin poses, camera/world frames, capture times, invalid
  observation rejection, persistent identity, and absence behavior.
- [ ] Explain using original meshes for both geometry roles and later replacing
  configured resources with simplified models while preserving labels.
- [ ] Run existing connector tests, focused new generic tests, aircraft
  acceptance tests, and relevant full-engine regressions in separate batches.
- [ ] Use the configured interpreter and ROS setup; check free memory, account
  for concurrent test runs, enforce a per-run cap, and pass `--orm-build never`.
- [ ] Format modified Python files with `scripts/format_docstrings.py`. Record
  exact test counts, failures, skips, and environment limitations; do not weaken
  existing tests or claim unexecuted checks passed.
- [ ] Confirm new tests belong to the existing generic or application CI suites
  and require no live APIs or credentials.
- [ ] Prepare a PR description around the final implementation and verification;
  use only the authorized ACRAMbly fork under repository workflow rules.

## Comments

- 2026-10-28: Specification ticket only; implementation is not authorized.
