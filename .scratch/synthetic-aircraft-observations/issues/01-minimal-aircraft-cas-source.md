# 01: Emit minimal aircraft CAS observations

Status: resolved
Type: task
Blocked by: None; the implementation base must contain PR #13's interfaces.

Implementation permission: granted by the user invoking `/implement-spec` and
subsequently directing implementation to proceed.

## Outcome

A public `scripted_aircraft` engine emits a configured static observation of the
twelve aircraft-part kinds, with the minimal payload and capture/frame metadata
required by PR #13. Existing `scripted_tetris` behavior remains available.

## Acceptance criteria

- [x] Use the existing aircraft inventory and structured editable scene/frame
  configuration; retain existing model labels, units, and mesh-origin meaning.
- [x] Emit one `ObjectHypothesis` per visible part containing its classification,
  plain camera-relative pose, and deterministic detection ID.
- [x] Write scheduled integer capture time, distinct world/camera frame IDs, and
  an explicit valid camera-to-world transform into each new CAS, including an
  empty scene. Keep the reusable configuration outside the replaced CAS.
- [x] Require explicit frame/transform configuration rather than silently
  manufacturing identity calibration. Caller-owned frame initialization remains
  separate from source construction and capture.
- [x] Emit the specification's minimal payload without populated masks/regions,
  shape or box annotations, camera calibration, images, depth, or point clouds.
- [x] Source construction and capture leave persistent world state unchanged.
  No connector is included in the source-only engine.
- [x] Keep the aircraft engine, scenario, observation source, and output contract
  separate from Tetris. Reuse existing generic helpers where applicable, without
  importing Tetris-specific source components or modifying its implementation.
- [x] Add failing public-engine pytest coverage before implementation, asserting
  configured labels, poses, exact metadata, excluded payload, and world isolation.

## Verification

After permission, use existing application ROS, blackboard, CAS, and world
fixtures. Validate a single-capture configuration and an empty configuration
without a display, mesh-payload construction, or external services. Run focused
application and existing static Tetris/pipeline regressions under the
specification's interpreter, memory, ORM, and formatter requirements.

## Completion record

Implemented through the public analysis engine and its completed CAS observations.
See the answer below for verification and limitations.


## Answer

Added the independent public `scripted_aircraft` engine, an aircraft observation
annotator, and structured `AircraftFrames`, `AircraftScenario`, `AircraftLayout`,
and `AircraftPartState` descriptors under `aircraft_observations`. The source
uses existing `AircraftModel` definitions without loading meshes. It emits only
classification and plain mesh-origin pose annotations, deterministic detection
identifiers, and explicit capture/frame metadata. Existing public observation
validation checks frame calibration and pose validity without initializing world
frames or changing persistent beliefs. Playback is the dependent ticket 02.

The initial public-engine test failed with the missing `scripted_aircraft`
module before production implementation. Root conftest initially could not load
because the local environment lacked Bullet bindings; a temporary
`--noconftest` collection run established that missing-engine failure without
building ORM interfaces. Subsequent standard runs used `--orm-build never`.
Frame rejection then failed with `DID NOT RAISE InvalidObservationFrame` (five
other tests passed), and invalid quaternion cases failed with `DID NOT RAISE
InvalidObservationPose` (six other tests passed), before implementing each
validation behavior using the existing generic acceptance interfaces.

Commands were run from `/tmp/aircraft-ticket-01`, using
`source /tmp/aircraft-env.sh`, the configured
`/home/nasimpcm/.virtualenvs/cram-env/bin/python`, ROS Jazzy,
`OPENBLAS_NUM_THREADS=1`, `OMP_NUM_THREADS=1`, temporary prerequisite paths,
per-run free-memory checks (over 11 GiB available), and
`ulimit -v 6291456`. Test runs were serialized.

- `python -m pytest --orm-build never test/robokudo_acrambly_test/test_scripted_aircraft.py -q --tb=short`: **8 passed**.
- `python -m pytest --orm-build never test/robokudo_acrambly_test/test_scripted_aircraft.py test/robokudo_acrambly_test/test_scripted_tetris.py -q --tb=short`: **10 passed, 7 failed**. The seven existing Tetris tests fail with `KeyError: CASViews.COLOR_TO_DEPTH_RATIO` in the existing projection path. No Tetris source or test file was modified by this ticket.
- `python scripts/format_docstrings.py` ran on all five added Python files; its first pass reformatted files. A final stable pass succeeds.
- `git diff --check`: passed.

ROS DDS printed sandbox socket errors, but the existing ROS fixtures and all
new engine tests completed successfully. Tests used no external services,
display, credentials, or mesh resources. Generated ORM interfaces were not read,
modified, regenerated, or tracked.
