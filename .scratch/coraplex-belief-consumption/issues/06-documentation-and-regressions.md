# 06: Documentation and focused regressions

Status: done
Type: task
Blocked by: 01, 02, 03, 04, 05

Implementation permission: granted 2026-11-25.

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

## Answer

Implemented on PR 14's branch (`feature/coraplex-belief-consumption`), stacked on
PR 13, on 2026-11-25. Two upstream Semantic Digital Twin fixes were backported
with the user's approval, because part 1 failed without them: positions set in
a modification are published after its model change (upstream PR 1018), and
`WorldSynchronizer` keeps 1000 instead of 10 messages for a busy receiver
(upstream `a0f9688646`).

Interpreter `/home/nasim/.virtualenvs/cram-env/bin/python` (Python 3.12) with
`/home/nasim/ros_ws/install/setup.bash`, every workspace `src/` directory and the
repository root on `PYTHONPATH`, `CRAM_ORM_BUILD=never`, `OPENBLAS_NUM_THREADS=1`,
`OMP_NUM_THREADS=1`, `QT_QPA_PLATFORM=offscreen`, and a 16 GiB virtual memory
cap (`ulimit -v 16777216`; 8 GiB stops ROS from creating threads). Free memory
was checked before each run.

```bash
python -m pytest --orm-build never test/robokudo_acrambly_test \
  --ignore=test/robokudo_acrambly_test/test_package_loading.py \
  --ignore=test/robokudo_acrambly_test/test_robokudo_cram_integration.py \
  --ignore=test/robokudo_acrambly_test/test_robokudo_cram_integration_spin.py
```

All aircraft, synchronization, accepted capture and stand-in tests pass (130).
Two `test_inspection_stops_when_a_standard_visualizer_requests_exit` cases fail
with `StopIteration` from the mocked key events, as before this work. The
Tetris tests give 33 failed, 17 passed, 13 errors, identical to the pre-PR
commit `49bd584178` run in a separate worktree. The ignored files fail before
this work: `test_package_loading.py` cannot import `CrDescriptorFactory`, and
`test_robokudo_cram_integration.py` leaves a broken query server on the
blackboard that makes every later test error in teardown.

```bash
python -m pytest --orm-build never test/robokudo_test/test_belief_updates.py \
  test/robokudo_test/test_belief_creation.py test/robokudo_test/test_semantic_observations.py \
  test/robokudo_test/test_semantic_world_connector.py test/robokudo_test/test_utils_comparators.py
```

Result: 149 passed.

```bash
python -m pytest --orm-build never test/semantic_digital_twin_test/test_ros
```

Result: 302 passed, 1 skipped. `test/semantic_digital_twin_test/test_worlds`
also passes with the backport (403 passed with the synchronizer tests).

By hand: `python acrambly_temp/stand_in_world_server.py` in one process, the
headless `robokudo.scripts.main ... _ae scripted_aircraft _headless` runner in a
second (exit 0 after 301 captures), and a third process fetching afterwards got
root `map`, 11 bodies and 10 aircraft parts. RViz was not checked; that still
needs a desktop session.
