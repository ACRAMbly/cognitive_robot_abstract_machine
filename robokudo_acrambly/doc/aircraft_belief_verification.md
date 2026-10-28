# Aircraft belief verification

The acceptance boundary is the local RoboKudo belief world. Tests exercise the
public connector and world interfaces; aircraft tests configure the actual
application model catalog and checked-in `plane/*.stl` resources.

## Focused batches

Run from the checkout root using the configured project interpreter and sourced
ROS installation. Pass `--orm-build never`; regenerate interfaces explicitly
with `scripts/regenerate_all_orm.py` only when needed. Before each batch, inspect
available memory and other running tests, then enforce a suitable memory cap.
Run these batches sequentially.

The local verification environment uses
`/home/nasim/.virtualenvs/cram-env/bin/python` and
`/home/nasim/ros_ws/install/setup.bash`, with `OPENBLAS_NUM_THREADS=1`,
`OMP_NUM_THREADS=1`, `QT_QPA_PLATFORM=offscreen`, `ROS_LOG_DIR`, `MPLCONFIGDIR`,
and `XDG_CACHE_HOME` redirected to writable temporary directories. Each batch is
limited to 8 GiB of virtual memory (`ulimit -v 8388608`).

## CI and boundaries

Generic configuration, validation, geometry, and association tests live in
`test/robokudo_test` and use local semantic mimics. Application tests live in
`test/robokudo_acrambly_test`; the existing CI matrix includes both packages.
The application CI job already regenerates ORM interfaces explicitly and runs
pytest with `--orm-build never` under a memory cap. No new live services,
credentials, hardware, synchronization process, or detector are required.

Full-engine regression cases also include existing file-reader scenarios that
may download the existing RoboKudo dataset. A failure to fetch that dataset is
reported separately from local connector and aircraft acceptance results.

## Recorded results, 2026-10-28

Verification used integration commit `96ccc8c00` (tickets 01–05). Worktree package
sources were placed first in `PYTHONPATH` so editable installations could not
silently select a different checkout. Over 26 GiB of memory was available before
each sequential test batch. No dependencies were added or ORM files regenerated.

The new tests first exposed cross-model identity matching, missing image-mask
handling, and a body retaining its old pose. The six-stage application sequence
failed at the translation observation on the pre-ticket-04 baseline; after the
association and pose update changes it passes through reappearance.

```bash
python -m pytest --orm-build never \
  test/robokudo_test/test_belief_updates.py \
  test/robokudo_test/test_belief_creation.py \
  test/robokudo_test/test_semantic_observations.py \
  test/robokudo_test/test_semantic_world_connector.py \
  test/robokudo_test/test_utils_comparators.py -q
```

Result: **149 passed**, one existing Matplotlib `Axes3D` import warning. This
covers model compatibility, geometry-preserving updates, absence, configured
creation and validation, existing connector behavior, and comparators.

The aircraft pipeline test is
`test/robokudo_acrambly_test/test_aircraft_observation_sequence.py`. It creates a
local `Pipeline` with the real connector and fresh CAS instances. All twelve
configured kinds retain beliefs, bodies, UUIDs, annotations, and mesh shapes
through initial creation, translation, rotation, partial absence, total absence,
and reappearance. Visible parts accept the fixture's pose and timestamp; absent
parts retain exact prior snapshots. Geometry bounds and rotated local centers
are checked against the configured source resources.

```bash
python -m pytest --orm-build never \
  test/robokudo_acrambly_test/test_aircraft_models.py \
  test/robokudo_acrambly_test/test_aircraft_belief_creation.py \
  test/robokudo_acrambly_test/test_aircraft_observation_sequence.py -q
```

Result: **43 passed**, one existing Matplotlib `Axes3D` import warning. All
configured source models and the six-stage sequence were exercised locally.

```bash
python -m pytest --orm-build never test/robokudo_test/test_full_ae_execution.py -q
```

Result: **2 passed, 2 failed**, five warnings. Both raytracer cases passed.
`test_run_simple_ae_successfully` and
`test_run_file_reader_ae_synchronizes_semantic_digital_twin_belief_state` failed
because the existing dataset downloader could not resolve
`gitlab.informatik.uni-bremen.de`. The process also emitted the existing NumPy
binary compatibility diagnostic at startup. These results do not establish
passing file-reader regressions.

```bash
python -m pytest --orm-build never test/robokudo_test -q
```

Result: **864 items collected, 2 collection errors**. Existing
`test_exceptions.py` and `test_utils_cv_helper.py` import the unavailable
`get_scale_coordinates`. Collection stopped before package tests executed.

```bash
python -m pytest --orm-build never test/robokudo_acrambly_test -q
```

Result: **116 items collected, 1 collection error**. Existing
`test_package_loading.py` imports the unavailable `CrDescriptorFactory` from
`robokudo.descriptors`. Collection stopped before package tests executed.

No tests were skipped in the focused or full-engine batches. Existing failing
tests were left unchanged. Full-package collection failures and the unavailable
dataset remain verification limitations; the full monorepo suite was not run.

Modified Python files for tickets 04 and 05 were formatted with
`scripts/format_docstrings.py`. Ticket 06 changes Markdown only. Whitespace
validation uses `git diff --check`.

## Post-review verification

Review commit `e509985a3` (merged as `ab57c5c2c`) names the incompatible-assignment
cost, shares model-label extraction, and groups accepted aircraft state in a
dataclass. The same capped, sequential environment reran the affected batches:

```bash
python -m pytest --orm-build never \
  test/robokudo_test/test_belief_updates.py \
  test/robokudo_test/test_semantic_world_connector.py \
  test/robokudo_test/test_utils_comparators.py -q
```

Result: **69 passed**, one existing Matplotlib warning.

```bash
python -m pytest --orm-build never \
  test/robokudo_acrambly_test/test_aircraft_observation_sequence.py \
  test/robokudo_acrambly_test/test_aircraft_belief_creation.py \
  test/robokudo_acrambly_test/test_aircraft_models.py -q
```

Result: **43 passed**, one existing Matplotlib warning. Formatting ran on all
three Python files changed during review, and `git diff --check` passed. Earlier
full-engine dataset and package-collection limitations remain unresolved.
