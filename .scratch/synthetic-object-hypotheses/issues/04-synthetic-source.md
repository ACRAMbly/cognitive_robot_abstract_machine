# Synthetic observation source

Status: resolved
Type: task
Blocked by: 02, 03

A `CameraInterface` subclass read by `CollectionReaderAnnotator`. Each due pass
writes the `CameraObservation` (standalone synthetic camera, scheduled time, no
world pose) and one `ObjectHypothesis` per part with `Classification` and
`PoseAnnotation`. Adds nothing to the world.

Done when tests on a reader pipeline check those CAS contents for chosen passes.

## Answer

`synthetic_observations/camera_interface.py` (`SyntheticObservationInterface`, `SyntheticCameraConfig`). Covered by `test_synthetic_observation_interface.py`, including a mutation check that the no-world-change test fails when the camera is added.

## Comments
