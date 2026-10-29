# 03: Verify aircraft observations with native 3D meshes

Status: resolved
Type: task
Blocked by: 01: Minimal aircraft CAS source; 02: Finite aircraft playback.

Implementation permission: granted by the explicit `/implement-spec` request.
Follow the approved
[specification](../spec.md), especially its optional 3D verification contract.

## Outcome

A developer can inspect catalog meshes moving at the poses emitted by the
aircraft source without adding mesh or image evidence to the CAS contract.

## Acceptance criteria

- [x] Use existing native annotator geometry output and `AircraftModelCatalog`
  with an explicit mesh directory; reuse existing catalog resource errors.
- [x] Load configured preview meshes once per preview configuration. Preserve
  original metric units, source origins, and component offsets.
- [x] Place independent display mesh copies using the emitted camera-relative
  poses. Preserve shared catalog geometry, CAS poses, and world state.
- [x] Render without requiring CAS shape/box annotations, masks, 2D regions,
  calibration, or a fabricated camera image. Leave Tetris visualization intact.
- [x] Replace displayed part geometry on completed captures, clear absent parts,
  and retain prior output on waiting ticks.
- [x] Retain the final scene and normal native exit controls after exhaustion;
  keep finite headless completion independent of display processing.
- [x] Verify mesh bounds/offsets and pose placement using deterministic local
  resources, deriving expectations from the consumed catalog and scenario.
- [x] Add display-independent pytest coverage for movement, empty output,
  geometry isolation, final-scene retention, and exit behavior before implementing
  the corresponding behavior. Mock desktop boundaries only as required.

## Verification

After permission, reuse the existing native-output/visualization fixtures. Core
checks require no display or live services. Actual aircraft-resource coverage
may supplement always-running local fixture checks. Run relevant existing Tetris
visualization and native lifecycle regressions as separate bounded batches.

## Completion record

Implemented through the public engine and native output boundaries. See the
answer below for executed verification and limitations.

## Answer

`AircraftPreview` owns an explicitly loaded `AircraftModelCatalog`. The public
engine accepts an optional `preview` configuration; absent that configuration,
observation generation loads no preview resources. Completed observations supply
native mesh copies transformed at their emitted camera-relative mesh origins.
Catalog metric coordinates and source offsets are retained. The existing mesh
converter and pose conversion are reused; no shapes, boxes, images, calibration,
or depth evidence enter the CAS.

The source updates native geometry only after a completed capture. Empty captures
publish an empty geometry list, while waiting and exhausted ticks retain the
previous output. The existing visualization manager, native backends, and runner
already provide final-scene interaction and termination, so no generic viewer,
runner, Tetris source, or existing test was changed. Launcher environment wiring
and usage instructions belong to ticket 05.

Test-first evidence and verification:

- Initial red: the public-engine preview test failed during collection with the
  missing `robokudo_acrambly.aircraft_observations.preview` module before any
  preview production implementation (`/tmp/aircraft-03-red-preview.log`).
- First green: the offset, explicitly scaled local-resource test passed after
  implementing catalog display copies and the optional source hook
  (`/tmp/aircraft-03-green-preview.log`: **1 passed**).
- Local output/isolation checks passed: **7 passed**, covering exact placement,
  independent copies, resource reuse after removal, moving/absent/reappearing
  geometry, waiting/exhaustion retention, complete world snapshots, and the
  minimal CAS contract (`/tmp/aircraft-03-geometry-lifecycle.log`).
- Existing native lifecycle behavior required no further production changes:
  **4 passed** with the real visualization manager and backends, intercepting
  desktop operations only. Checks cover final-scene event processing, annotator
  navigation, Escape through native key polling, window closure, and headless
  completion (`/tmp/aircraft-03-native-lifecycle.log`).
- Final aircraft regression command:
  `/tmp/aircraft-pytest test/robokudo_acrambly_test/test_scripted_aircraft.py test/robokudo_acrambly_test/test_scripted_aircraft_playback.py test/robokudo_acrambly_test/test_scripted_aircraft_preview.py test/robokudo_acrambly_test/test_scripted_aircraft_native_preview.py -q --tb=short`:
  **51 passed** (`/tmp/aircraft-03-final-aircraft.log`). This includes all twelve
  actual checkout `plane/` resources. That supplemental check skips when local
  aircraft resources are absent; deterministic local-resource checks always run.
- Separate retained Tetris visualization command:
  `/tmp/aircraft-pytest test/robokudo_acrambly_test/test_scripted_tetris_visualization.py -q --tb=short`:
  **1 passed, 9 failed**, all with the preexisting
  `KeyError: CASViews.COLOR_TO_DEPTH_RATIO` in the unchanged projection path
  (`/tmp/aircraft-03-tetris-visualization.log`). Tetris visual assertions remain
  blocked by that baseline failure; no failing existing test was modified.
- `scripts/format_docstrings.py` ran on the five added/modified Python files;
  its final repeated run returned zero with no changes. `git diff --check` passed.

All pytest batches ran from `/tmp/aircraft-ticket-03` through the shared
`/tmp/aircraft-pytest` lock. It loads `/tmp/aircraft-env.sh`, uses the configured
`/home/nasimpcm/.virtualenvs/cram-env/bin/python`, sources ROS Jazzy and the
available temporary overlay, checks free memory and Python processes, caps
virtual memory at 6 GiB, and passes `--orm-build never`. Recorded available
memory exceeded 11 GiB. The shared lock serialized these tests with other
implementation batches. No generated ORM interface was read or changed, and no
new application dependency was added.

No physical desktop inspection was performed. Native behavior was verified at
mocked desktop boundaries; tests needed no hardware, credentials, or live network.
