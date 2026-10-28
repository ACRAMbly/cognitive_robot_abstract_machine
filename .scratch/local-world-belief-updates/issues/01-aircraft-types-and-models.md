# 01: Define aircraft semantic types and model resources

Status: resolved
Type: task
Blocked by: None

Implementation permission: granted by the user on 2026-10-28 for ticket 01.
Other tickets remain outside this authorization.

## Outcome

`robokudo_acrambly` owns the 12 configured aircraft-part kinds under
`AircraftPart`, with labels and model resources ready for generic acceptance.
Follow the [specification](../spec.md) and [ADR 0003](../../../docs/adr/0003-keep-aircraft-semantics-in-acrambly.md).

## Acceptance criteria

- [x] Add documented dataclass semantic types using existing body-root and
  perception interfaces, retaining the accepted classification label.
- [x] Define each model's label, concrete semantic type, resource, and metric
  scale once, using the inventory in the connector brief.
- [x] Accept a configured mesh directory and report unavailable resources with
  an actionable existing or custom dataclass exception.
- [x] Load each configured model once and permit its resource to be replaced
  later without changing observation labels.
- [x] Supply the same model geometry for both visual and collision collections,
  preserving mesh origin and local offsets with independent per-body state.
- [x] Add test-first application coverage for all 12 mappings, semantic subtype
  queries, resource loading, and missing resources. Use checked-in assets where
  the behavior depends on those models.
- [x] Keep application types out of generic RoboKudo and semantic digital twin
  imports. Grasp definitions and assembly relationships remain outside scope.

## Verification

Run the focused aircraft model/type tests in `test/robokudo_acrambly_test` with
the repository's interpreter, memory controls, and `--orm-build never`. Format
modified Python files and record results when implementation is authorized.

## Comments

- 2026-10-28: Specification ticket only; implementation is not authorized.
- 2026-10-28: User explicitly requested implementation of ticket 01 and confirmed
  semantic subtype/label queries and model catalog/geometry as the test seams.
  This supersedes the earlier permission note for ticket 01 only.


## Answer

Implemented application-owned `aircraft.types`, `aircraft.models`, and
`aircraft.exceptions`, with all 12 mappings and configurable metric resources.
The catalog eagerly loads templates once and creates independent visual/collision
shapes at the original mesh origin. Resource replacement retains observation
labels. Missing, unreadable, and non-file paths raise `MissingAircraftResource`.
See [configuration documentation](../../../robokudo_acrambly/doc/aircraft_models.md).

Verification used `/home/nasim/.virtualenvs/cram-env/bin/python`, the ROS overlay
`/home/nasim/ros_ws/install/setup.bash`, `OPENBLAS_NUM_THREADS=1`,
`OMP_NUM_THREADS=1`, `ROS_LOG_DIR=/tmp/aircraft-ros-logs`, and `ulimit -v 8388608`.
Available memory exceeded 27 GiB before each sequential test batch. All pytest
invocations included `--orm-build never`; no ORM changes were needed.

- `python -m pytest --orm-build never test/robokudo_acrambly_test/test_aircraft_models.py -q`:
  41 passed. Red runs proved missing types/catalog, missing-resource handling,
  and raw permission errors before implementation. The user approved correction
  of a disconnected-body mistake in the initial test setup.
- `python -m mypy --follow-imports=silent --ignore-missing-imports robokudo_acrambly/src/robokudo_acrambly/aircraft test/robokudo_acrambly_test/test_aircraft_models.py`:
  success for all five files. Imported workspace packages were not typechecked.
- `scripts/format_docstrings.py`: run on all five Python files with black and
  docformatter; `git diff --check` passed.
- Complete `test/robokudo_acrambly_test`: collection blocked by the existing
  `CrDescriptorFactory` import in `test_package_loading.py`.
- Same package excluding that file: 40 passed, 1 failed, 73 errors. The existing
  integration test expects SUCCESS but receives RUNNING; blackboard cleanup
  then calls `destroy()` on None, preventing subsequent fixtures.
- Same package excluding package-loading and both integration test files:
  57 passed, 33 failed, 13 errors. Existing Tetris failures include missing
  `CASViews.COLOR_TO_DEPTH_RATIO` metadata. These files were left unchanged.
- `test/robokudo_test/test_semantic_world_connector.py` and
  `test/robokudo_test/test_full_ae_execution.py`: 6 passed, 3 failed. The failures
  hit read-only cache locations under `~/.cache/robokudo_test_data` and
  `~/.cache/semantic_digital_twin/convex_decompositions`.

The missing local `random_events` C++ extension was rebuilt in place using the
configured interpreter; no source changes were required. Already-declared mypy
and docformatter tools were installed under `/tmp/aircraft-tools` for verification.
The full monorepo suite was not run, following the repository's package-batch
rule. Broader regression results above are not a clean-suite claim.

Two-axis code review against `e4be120fd`: Standards reported three issues
(abbreviated import, type-only imports, fixture filenames), all resolved;
Spec reported one issue (unreadable resources), resolved with a failing test and
readability guard. Reviewers confirmed closure. The permission-bit test skips
only when the execution identity can still read a file with mode zero.
