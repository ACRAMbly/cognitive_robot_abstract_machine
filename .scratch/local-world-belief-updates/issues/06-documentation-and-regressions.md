# 06: Document configuration and complete focused regressions

Status: resolved
Type: task
Blocked by: 01, 02, 03, 04, 05

Implementation permission: granted by the user’s explicit request to implement
tickets 04, 05, and 06 on 2026-10-28.

## Outcome

Maintainers can configure aircraft acceptance, understand its local-world
boundary, replace mesh resources later, and review exact validation evidence.

Follow the [specification](../spec.md) and the fork-only workflow in the connector
brief after implementation is explicitly authorized.

## Acceptance criteria

- [x] Document the actual implemented configuration interface, model mapping,
  mesh-directory setup, metadata requirements, and test/pipeline usage.
- [x] Explain mesh-origin poses, camera/world frames, capture times, invalid
  observation rejection, persistent identity, and absence behavior.
- [x] Explain using original meshes for both geometry roles and later replacing
  configured resources with simplified models while preserving labels.
- [x] Run existing connector tests, focused new generic tests, aircraft
  acceptance tests, and relevant full-engine regressions in separate batches.
- [x] Use the configured interpreter and ROS setup; check free memory, account
  for concurrent test runs, enforce a per-run cap, and pass `--orm-build never`.
- [x] Format modified Python files with `scripts/format_docstrings.py`. Record
  exact test counts, failures, skips, and environment limitations; do not weaken
  existing tests or claim unexecuted checks passed.
- [x] Confirm new tests belong to the existing generic or application CI suites
  and require no live APIs or credentials.
- [x] Prepare a PR description around the final implementation and verification;
  use only the authorized ACRAMbly fork under repository workflow rules.

## Comments

- 2026-10-28: Specification ticket only; implementation is not authorized.

- 2026-10-28: User authorized implementation of tickets 04–06.

## Answer

Updated application configuration, generic observation documentation, and the
connector brief to describe the implemented model catalog, metadata contract,
model-origin poses, compatible association, retained geometry and identity,
absence, and replacement resources. The prepared PR description is local;
nothing was pushed or published.

Exact commands, environment and limitations are recorded in
[verification](../../../robokudo_acrambly/doc/aircraft_belief_verification.md).
The [prepared PR description](../../../robokudo_acrambly/doc/aircraft_belief_pr.md)
is ready for review.

Final focused batches: 149 generic cases and 43 aircraft cases passed. Following
review refinements, the affected 69 generic cases and all 43 aircraft cases
passed again. Full-engine regressions: 2 raytracer cases passed, 2 file-reader
cases failed because the existing dataset host could not resolve. Broader
package collection stopped at two missing get_scale_coordinates imports in
RoboKudo (864 items collected) and a missing CrDescriptorFactory import in the
application (116 items collected). No existing failing tests were modified.

Tests ran sequentially with the configured interpreter and ROS setup, free-memory
checks, an 8 GiB virtual-memory cap, and --orm-build never. No dependencies or
ORM regeneration were needed. Modified Python files were formatted with
scripts/format_docstrings.py using existing tooling. Both final review axes
reported zero remaining findings. Temporary implementer worktrees are removed
after integration. Delivery branch: feature/local-world-belief-updates.
