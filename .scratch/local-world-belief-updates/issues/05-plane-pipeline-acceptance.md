# 05: Prove the six-stage aircraft-part acceptance sequence

Status: resolved
Type: task
Blocked by: 01, 02, 03, 04

Implementation permission: granted by the user’s explicit request to implement
tickets 04, 05, and 06 on 2026-10-28.

## Outcome

A deterministic local pipeline proves acceptance of all 12 configured aircraft
part kinds through movement, absence, and reappearance.

Follow the [specification's acceptance matrix](../spec.md#acceptance-matrix).

## Acceptance criteria

- [x] Build fresh CAS observations through public types with fixture-owned
  capture times, frame metadata, poses, and changing detection identifiers.
- [x] Use the existing connector in a local pipeline with initialized world and
  camera frames and a nonidentity camera transform containing rotation and
  translation.
- [x] Exercise all present, translation, rotation, partial absence, total absence,
  and reappearance, using all 12 configured model kinds.
- [x] Assert exact observation and belief counts from the acceptance matrix,
  retained UUIDs and semantic annotations, accepted poses, and geometry in both
  collections. Derive expected geometry from configured resources.
- [x] Compare absent-object state, including last accepted capture time, against
  its prior accepted state; reappearance updates existing beliefs without
  duplicates or accumulated geometry.
- [x] Keep fixtures deterministic and isolated using existing ROS, world, CAS,
  and blackboard fixtures. Include actual configured plane-model coverage in CI.
- [x] Do not add a production observation source, detector, assembly layout,
  synchronization process, or the declined duplicate-copy test case.

## Verification

Keep aircraft-specific imports in `test/robokudo_acrambly_test`. Implement
test-first; run focused application tests with memory controls and
`--orm-build never`. Format modified Python files and record results.

## Comments

- 2026-10-28: Specification ticket only; implementation is not authorized.

- 2026-10-28: User authorized implementation of tickets 04–06.

## Answer


Worktree: `.scratch/worktrees/ticket-05`, initially integration branch commit `dba7bdcd3`.
Public seam: real `SemanticDigitalTwinConnector.compute` attached to a local `Pipeline`, public CAS annotations and world queries. Every stage creates a fresh CAS, receives deterministic metadata and poses, and runs the actual connector synchronously without waits or hardware.

New deterministic fixture covers all `AircraftModel` definitions with a camera translated and rotated by a quarter turn, changing detection identifiers, integer capture seconds and retained nanoseconds. Six stages cover initial creation, translation, rotation, partial absence, total absence, and reappearance. Expected world matrices use the independently worked camera conversion; expected mesh bounds and centroid offsets derive from checked-in source resources. Checks retain belief/body/UUID, semantic annotation, shape instances/counts, last accepted hypothesis/capture metadata during absence, and world poses.

Configured interpreter `/home/nasim/.virtualenvs/cram-env/bin/python`; source `/home/nasim/ros_ws/install/setup.bash`. Worktree `robokudo/src` and `robokudo_acrambly/src` prepend PYTHONPATH. Existing formatting/typecheck tools in `/tmp/aircraft-tools` are reused without installing dependencies. Env: OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ROS_LOG_DIR=/tmp/belief-ros-logs MPLCONFIGDIR=/tmp/belief-matplotlib XDG_CACHE_HOME=/tmp/belief-cache QT_QPA_PLATFORM=offscreen.

Baseline red run:
`python -m pytest --orm-build never test/robokudo_acrambly_test/test_aircraft_observation_sequence.py -q`
1 failed, 1 warning, 2.58 seconds. Creation and all12 initial assertions succeeded; second capture failed in existing ImageROIComparator because optional mask is None. No test changes were required. Raw log `/tmp/ticket05-red.log`.
Checked available memory 27160 MiB and no competing pytest processes, enforced `ulimit -v 8388608`.

`scripts/format_docstrings.py` completed both new files (black + docformatter) with configured Python and existing tools.
`python -m mypy --follow-imports=silent --ignore-missing-imports` on the two new files: success.
`git diff --check`: success.

Merged integration ticket04 tip `f65d1b1f2` into ticket05 before green verification (fast-forward). No test changes after the red run.
Green batch:
`python -m pytest --orm-build never test/robokudo_acrambly_test/test_aircraft_observation_sequence.py test/robokudo_acrambly_test/test_aircraft_belief_creation.py test/robokudo_acrambly_test/test_aircraft_models.py -q`
43 passed, 1 existing matplotlib warning, 6.49 seconds. Available memory 27210 MiB; enforced 8 GiB virtual memory cap. Log `/tmp/ticket05-green.log`. No new runtime changes required beyond ticket04.
