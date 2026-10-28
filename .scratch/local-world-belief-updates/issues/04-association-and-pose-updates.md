# 04: Associate compatible models and preserve accepted geometry

Status: resolved
Type: task
Blocked by: 02, 03

Implementation permission: granted by the user’s explicit request to implement
tickets 04, 05, and 06 on 2026-10-28.

## Outcome

Compatible observations update the existing physical-part belief while retaining
its body, identity, geometry, and semantic annotation.

Follow the [specification](../spec.md). Reuse the existing comparators, assignment,
and world-update machinery.

## Acceptance criteria

- [x] Prevent known different model kinds from matching, even when other
  association evidence would favor the wrong belief.
- [x] Keep association similarity distinct from classification confidence and
  introduce no new acceptance thresholds or uncertainty model.
- [x] Preserve existing unmatched and low-similarity creation behavior, with
  existing unmatched beliefs retained.
- [x] Accepted pose-only updates translate and rotate the existing body without
  replacing meshes or accumulating shapes or semantic annotations.
- [x] Retain belief and body UUID, semantic annotation identity, class label, and
  geometry on successful matches. Detection IDs may change between observations.
- [x] Absent objects retain their last accepted pose, capture time, and all other
  accepted object state. Empty observations make no object-state changes.
- [x] Add test-first regressions for incompatible models, geometry-preserving
  pose updates, unmatched/low-similarity behavior, and absence.

## Verification

Run focused generic association and world-update tests, then relevant existing
connector tests, in separate pytest batches with memory controls and
`--orm-build never`. Format modified Python files and record results.

## Comments

- 2026-10-28: Specification ticket only; implementation is not authorized.

- 2026-10-28: User authorized implementation of tickets 04–06.

## Answer


Worktree `.scratch/worktrees/ticket-04`, branch `implement/belief-ticket-04`, base `dba7bdcd3` on integration branch. Public seams: existing connector association and local world state.

Red/green:
- New incompatible model test first failed because the Panel observation updated the existing Block belief despite identical image/pose evidence. Positive forbidden association costs now exclude configured incompatible model labels; negative similarity thresholds cannot override this.
- New geometry-retaining pose test first failed with missing image masks, then with the body retaining its old pose. Available evidence now skips absent ROI masks and accepted configured model poses update the existing connection without replacing geometry.
- Last accepted capture state, partial/empty absence, UUIDs, annotations and local-offset rotation are asserted. Existing creation tests cover unmatched/low-similarity creation and existing connector tests cover generic bounding boxes.

Commands used configured `/home/nasim/.virtualenvs/cram-env/bin/python`, sourced `/home/nasim/ros_ws/install/setup.bash`, and exported worktree `robokudo/src` first in PYTHONPATH. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ROS_LOG_DIR=/tmp/belief-ros-logs MPLCONFIGDIR=/tmp/belief-matplotlib XDG_CACHE_HOME=/tmp/belief-cache QT_QPA_PLATFORM=offscreen. Every pytest batch checked free memory (over 27GB available), ran sequentially with no concurrent pytest, and capped virtual memory using `ulimit -v 8388608`.

- `python -m pytest --orm-build never test/robokudo_test/test_belief_updates.py`: final 4 tests pass (included below).
- `python -m pytest --orm-build never test/robokudo_test/test_belief_updates.py test/robokudo_test/test_belief_creation.py test/robokudo_test/test_semantic_observations.py -q`: 83 passed (three update tests before negative-threshold case was added).
- `python -m pytest --orm-build never test/robokudo_test/test_belief_updates.py test/robokudo_test/test_semantic_world_connector.py test/robokudo_test/test_utils_comparators.py -q`: 69 passed (four final update tests, five connector cases, 60 comparators).
- All runs reported one existing Matplotlib Axes3D import warning.
- `scripts/format_docstrings.py` run on all four changed Python files, using existing `/tmp/aircraft-tools/bin/docformatter`. `git diff --check` passed.

No dependencies added, no ORM regeneration, no existing tests modified, no pushes.

Post-review: named incompatible assignment costs, shared label extraction, and
consolidated application fixture state. The affected generic batch passed 69
cases; the complete aircraft focused batch passed 43 cases. Final Standards
and Spec reviews report zero remaining findings.
