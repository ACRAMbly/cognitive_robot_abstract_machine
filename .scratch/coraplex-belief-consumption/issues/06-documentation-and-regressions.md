# 06: Documentation and focused regressions

Status: ready-for-agent
Type: task
Blocked by: 01, 02, 03, 04, 05

Implementation permission: not granted.

## Outcome

The documentation describes the shared-world aircraft engine as built, and the
focused test runs are recorded. Follow the [specification](../spec.md).

## Acceptance criteria

- [ ] `scripted_aircraft.md` explains starting the stand-in before the engine or
  the inspection tool, the world frame from the fetched root, and the accepted
  capture topic.
- [ ] `coraplex_belief_consumption.md` marks part 1's steps as done and corrects
  any file and line references that moved.
- [ ] The glossary and ADRs 0001–0004 match what was built.
- [ ] `scripts/format_docstrings.py` ran on every modified Python file.
- [ ] Focused runs of `test/robokudo_acrambly_test` and the affected
  `test/robokudo_test` and `test/semantic_digital_twin_test/test_ros` tests pass,
  one package at a time, with `--orm-build never` and a memory cap. Results are
  recorded in this ticket's answer.

## Verification

Record the exact commands, interpreter and results in an `## Answer` section.

## Comments

- 2026-11-21: Specified in the planning session; no code written.
