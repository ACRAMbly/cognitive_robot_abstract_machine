# 02: Validate configured observations and resolve world poses

Status: resolved
Type: task
Blocked by: None

Implementation permission: granted by the user on 2026-10-28 for ticket 02.
Other tickets remain outside this authorization.

## Outcome

The generic connector accepts application-supplied semantic configuration and
prepares valid observations before changing persistent beliefs. Generic tests
use local test semantic types rather than aircraft imports.

Follow the [specification](../spec.md), especially its observation and validation
contracts. Coordinate this extension point with ticket 01's application models.

## Acceptance criteria

- [x] Add a configurable label-to-semantic-type acceptance interface without
  introducing application-package imports into generic RoboKudo.
- [x] Validate every configured hypothesis's model label, supported pose, frame,
  capture metadata, and required transform before persistent belief changes.
- [x] Reject missing, unknown, and conflicting model labels with meaningful
  exceptions; distinguish validation from association confidence.
- [x] Interpret plain poses in the camera frame and stamped poses in the declared
  camera or world frame, placing the mesh origin correctly.
- [x] Convert camera poses once; accept a world-frame stamped pose without an
  otherwise unnecessary camera transform. Preserve capture time and frame.
- [x] Reprocessing annotations does not compound transforms or replace capture
  metadata with processing time.
- [x] Test an invalid hypothesis after a valid one and compare persistent state
  exactly before and after, for both empty and populated belief worlds.
- [x] Preserve supported generic behavior when semantic configuration is absent.

## Verification

Add meaningful failing tests before implementation. Run focused generic connector
tests with a nonidentity camera transform, existing fixtures, memory controls,
and `--orm-build never`. Format modified Python files and record results.

## Comments

- 2026-10-28: Specification ticket only; implementation is not authorized.
- 2026-10-28: User requested implementation of ticket 02 and confirmed connector
  association and world-pose annotation preparation as the test boundaries.

## Answer

Implemented optional `Descriptor.Parameters.semantic_types` configuration and
generic `SemanticObservationAcceptance.prepare`, returning the original
hypothesis, resolved semantic type, and captured world pose. All configured
hypotheses are prepared before derived annotations or persistent beliefs change.
Generic acceptance remains available with `semantic_types=None`.

Validation reports missing, unknown, and conflicting labels; missing or malformed
poses; unsupported or ambiguous frames; inconsistent capture timestamps; and
missing, reversed, nonfinite, or nonrigid required camera transforms through
dataclass exceptions. Classification confidence remains independent of acceptance
and association similarity.

Plain and stamped camera poses convert at the mesh origin. World stamped poses
retain their frame and timestamp without requiring camera metadata solely for
their acceptance. Capture precision comes from integer CAS nanoseconds or ROS
camera-info timestamps. Reprocessing preserves original annotations and avoids
duplicate derived world poses. The
[configuration documentation](../../../robokudo/docs/semantic_observations.md)
shows how ticket 01's catalog supplies the mapping. Typed geometry creation and
body updates remain in tickets 03 and 04.

Verification used `/home/nasim/.virtualenvs/cram-env/bin/python`, sourced
`/home/nasim/ros_ws/install/setup.bash`, set `OPENBLAS_NUM_THREADS=1`,
`OMP_NUM_THREADS=1`, `ROS_LOG_DIR=/tmp/observation-ros-logs`, and
`MPLCONFIGDIR=/tmp/observation-matplotlib`. Each sequential pytest run checked
available memory (over 26 GiB available), used `ulimit -v 8388608`, and passed
`--orm-build never`. No ORM regeneration or new dependencies were required.

- Failing tests preceded the label, capture, stamped-pose, pose-validation, and
  metadata implementations. Runtime red runs proved missing validation and raw
  numerical errors for 12 malformed-pose cases and all 24 capture cases. Review
  added four failing cases for missing translation/quaternion components before
  fixing their raw `TypeError`. Missing stamped timestamps were also reproduced
  and corrected. Invalid-hypothesis tests compare exact before/after snapshots
  in empty and populated worlds, including retained observations, world poses,
  geometry identities, UUIDs, world state, and recorded changes.
- `python -m pytest --orm-build never test/robokudo_test/test_semantic_observations.py test/robokudo_test/test_semantic_world_connector.py -q`:
  76 passed (71 new cases and five existing connector regressions).
- Final generic regression batch added `test_utils_annotator_helper.py`,
  `test_utils_comparators.py`, and `test_full_ae_execution.py` to those files:
  152 passed, 19 failed. All connector cases and all 60 comparator cases passed.
  Sixteen helper failures reference the existing removed
  `CASViews.COLOR2DEPTH_RATIO`; three full-engine failures hit read-only cache
  paths under `~/.cache/robokudo_test_data` and
  `~/.cache/semantic_digital_twin/convex_decompositions`. The fourth full-engine
  case passed. Existing failing tests were left unchanged.
- `python -m pytest --orm-build never test/robokudo_acrambly_test/test_aircraft_models.py -q`:
  41 passed.
- Complete `test/robokudo_test` package attempt: collection stopped with two
  existing imports of the missing `get_scale_coordinates` function, in
  `test_exceptions.py` and `test_utils_cv_helper.py` (847 items collected at that
  point). That attempt also used `XDG_CACHE_HOME=/tmp/observation-cache` and
  `QT_QPA_PLATFORM=offscreen`. The monorepo suite was not run in one invocation,
  following repository rules.
- `python -m mypy --follow-imports=silent --ignore-missing-imports` on the three
  changed production modules, new tests, and test dataset: success for all five
  files. Imported workspace packages were not typechecked.
- `scripts/format_docstrings.py` ran on all six changed Python files;
  `git diff --check` passed.

Two-axis code review compared this work with starting commit
`f688e04d60a971bcab9f66f55e3389ebd5212c72`. Standards found five issues, all
corrected: duplicate setup/selection, type-only imports, matrix-dimension naming,
snapshot field names, and structured test parameters. Spec found the missing
pose-component guard, corrected with failing regressions. Both reviewers
confirmed zero remaining findings. Work is committed on the existing
`plane-semdt-syn` branch; nothing was pushed.
