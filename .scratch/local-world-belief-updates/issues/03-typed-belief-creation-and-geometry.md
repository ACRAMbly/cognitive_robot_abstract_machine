# 03: Create typed beliefs with complete mesh geometry

Status: resolved
Type: task
Blocked by: 01, 02

Implementation permission: granted by the user's explicit ticket 03 implementation
request on 2026-10-28. Other tickets remain outside this authorization.

## Outcome

A validated unmatched hypothesis creates one persistent belief, mesh-bearing
body, and semantic annotation through existing world APIs.

Follow the [specification](../spec.md) and use the application configuration from
ticket 01 through the generic extension point from ticket 02.

## Acceptance criteria

- [x] Initialize visual and collision geometry from the configured model before
  adding its body to the local world.
- [x] Keep the body frame at the mesh origin and retain model-local offsets and
  explicit metric scale in both collections.
- [x] Register the body, belief, and correctly typed semantic annotation once,
  retaining the classification label and persistent body UUID.
- [x] Accept hypotheses containing a model label and supported pose without
  requiring a per-observation mesh or bounding box.
- [x] Prepare required geometry before persistent mutation so missing model
  resources do not leave partially created beliefs.
- [x] Preserve existing generic bounding-box creation when no model geometry is
  configured.
- [x] Add failing generic creation/geometry tests first, then application tests
  connecting actual aircraft model definitions to the acceptance interface.

## Verification

Use existing world/CAS fixtures and deterministic local geometry for focused
checks. Verify the geometry present at insertion as well as final state. Run
generic and application tests in separate memory-capped pytest batches with
`--orm-build never`; format modified Python files and record results.

## Comments

- 2026-10-28: Specification ticket only; implementation is not authorized.
- 2026-10-28: User explicitly requested ticket 03 implementation. This supersedes
  the earlier specification-only permission note for this ticket. Generic tests
  exercise connector creation/geometry; application coverage connects the actual
  aircraft catalog through the same acceptance interface.
- 2026-10-28: User approved correcting the new registration test to use the
  existing UUID lookup and adding existing ROI/mask evidence to two new
  association-route regression tests. Existing failing tests were not changed.

## Answer

Added generic `ModelGeometry` configuration through
`Descriptor.Parameters.model_geometry`. The aircraft catalog implements that
interface. Configured observations prepare detached bodies, loaded visual and
collision meshes, and typed semantic annotations before persistent creation.
All creation routes use the prepared body and captured world pose. Registration
uses existing world connection and semantic annotation APIs, retaining the
body UUID and accepted classification label. Source origins, local offsets,
and metric scales are preserved. Missing geometry mappings raise the dataclass
`MissingModelGeometry`; unavailable model resources cannot partially change an
empty or populated belief world. Generic bounding-box creation remains available
with no model geometry configured.

Configuration examples are in
[generic observation documentation](../../../robokudo/docs/semantic_observations.md)
and [aircraft model documentation](../../../robokudo_acrambly/doc/aircraft_models.md).
Association constraints, geometry-preserving updates, and the full observation
sequence remain in tickets 04 and 05.

Verification used `/home/nasim/.virtualenvs/cram-env/bin/python`, sourced
`/home/nasim/ros_ws/install/setup.bash`, and set `OPENBLAS_NUM_THREADS=1`,
`OMP_NUM_THREADS=1`, `ROS_LOG_DIR=/tmp/belief-ros-logs`, and
`MPLCONFIGDIR=/tmp/belief-matplotlib`. Final regression runs also used
`XDG_CACHE_HOME=/tmp/belief-cache` and `QT_QPA_PLATFORM=offscreen`. Each pytest
batch checked free memory (over 26 GiB available), capped virtual memory with
`ulimit -v 8388608`, ran sequentially, and passed `--orm-build never`. No ORM
regeneration or dependency additions were required.

- Red runs reproduced empty model geometry, missing typed annotations, and a
  raw `KeyError` for a missing model mapping before their fixes. Application
  typechecking exposed the missing generic interface implementation before its
  adaptation. Generic coverage precedes application integration coverage.
- `python -m pytest --orm-build never test/robokudo_test/test_belief_creation.py
  test/robokudo_test/test_semantic_observations.py
  test/robokudo_test/test_semantic_world_connector.py
  test/robokudo_test/test_utils_comparators.py
  test/robokudo_test/test_full_ae_execution.py -q`: 147 passed, 2 failed.
  All 85 creation, validation, and connector cases and all 60 comparator cases
  passed. Two raytracer full-engine cases passed. The other two engine cases
  could not download the existing RoboKudo test dataset because the GitLab
  hostname could not resolve in this environment.
- `python -m pytest --orm-build never
  test/robokudo_acrambly_test/test_aircraft_belief_creation.py
  test/robokudo_acrambly_test/test_aircraft_models.py -q`: 42 passed, including
  creation of all twelve actual aircraft model kinds in one observation.
  The initial integration attempt hit a read-only home cache; redirecting the
  cache to `/tmp` enabled the real collision-engine run.
- `python -m pytest --orm-build never test/robokudo_test -q`: collection stopped
  at two existing imports of missing `get_scale_coordinates`, in
  `test_exceptions.py` and `test_utils_cv_helper.py` (860 items collected).
- `python -m pytest --orm-build never test/robokudo_acrambly_test -q`: collection
  stopped at the existing missing `CrDescriptorFactory` import in
  `test_package_loading.py` (115 items collected).
- `python -m mypy --follow-imports=silent --ignore-missing-imports` on the new
  creation and exception modules, connector, aircraft models, new tests, and
  test dataset: success for seven files. Including `robokudo/world.py` reports
  five existing dynamic module attribute errors; typechecking the original
  starting-commit version reproduced the same five errors.
- `scripts/format_docstrings.py` ran on all eight modified Python files with
  black and docformatter; `git diff --check` passed.

The monorepo suite was not run in one invocation, following repository rules.
Complete-package and remote-dataset limitations above are not a clean-suite claim.

Two-axis code review used starting commit
`5a8674264adb07f0369a14457ea16139348b33af`. Standards found three issues
(broad exception assertion, duplicate registration logic, and an unnecessary
override docstring) and one design concern (mixed batch and individual preparation).
All were corrected. Spec found no ticket 03 gaps. Both reviewers confirmed zero
remaining findings after the fixes.

Post-review verification reran the creation, observation-validation, and existing
connector files: 85 passed. The two aircraft files passed again: 42 passed.
Focused mypy passed again for all seven files, and formatting and whitespace
checks passed. Existing-body association without image evidence still encounters
the image comparator's missing-mask behavior; it is explicitly deferred to
ticket 04 and documented in the generic configuration guide. No association or
existing-body update changes were introduced here.

Committed on the existing `plane-semdt-syn` branch using the configured human
Git identity. Nothing was pushed.
