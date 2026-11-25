# Synthetic object hypotheses

Status: resolved

## Goal

Feed RoboKudo pipelines with scripted observations: the object hypotheses a
real NOCTIS segmentation followed by FoundationPose would leave in the CAS,
specified by a scenario instead of perceived. This lets later work on the shared
world be tested without a camera, NOCTIS or FoundationPose.

## Scope

In:

- A new `robokudo_acrambly` package, written from scratch (the earlier package
  and its tests are deleted, not restored).
- One synthetic observation source, run by RoboKudo's stock
  `CollectionReaderAnnotator` through a `CameraInterface` subclass, so CAS
  creation, query keeping and the camera observation are the real ones.
- Two scenarios, tetris and aircraft, run by that same source.
- One analysis-engine descriptor per scenario, containing only the reader.
- Pytest coverage of the CAS each pass produces.

Out, for later PRs:

- `WorldUpdatingAnnotator` turning a `PoseAnnotation` and a class label into a
  mesh body (today it ignores `PoseAnnotation`, so nothing from this PR reaches
  the world; the real NOCTIS → FoundationPose pipeline has the same gap).
- Meshes, including rewriting the tetris STLs from millimetres to metres.
- Semantic part classes (`TopWing`, …); until then beliefs would be
  `UnknownObject` with a `class_label`.
- A stand-in world server, `WorldUpdatingAnnotator` in the descriptors, RViz.
- A timestamp or pass link on world objects; starting a pass on demand.
- No changes outside `robokudo_acrambly/` and `test/robokudo_acrambly_test/`.

## What one pass puts into the CAS

- `cas.camera_observation`: a `CameraObservation` with
  - a synthetic `Camera` on a standalone body registered in no world, built each
    pass by RoboKudo's own `CameraInterface.store_camera_observation`, as for any
    camera the world lacks;
  - a 640 × 480 pinhole calibration;
  - `world_T_camera = None`;
  - the pass's *scheduled* capture time.
- One `ObjectHypothesis` per scenario part, carrying exactly
  - a `Classification`: `classname` = the part's label (the mesh file stem,
    e.g. `TopWing`, `child_cube_0_colored`), confidence 1.0;
  - a `PoseAnnotation`: the part's scripted pose in the camera optical frame
    (translation x, y, z; quaternion x, y, z, w).

Both annotations carry the source's name in `source`. No images, point clouds,
regions of interest or masks.

## Scenarios

- Tetris: one instance each of `child_cube_0_colored` and `child_cube_2_colored`.
- Aircraft: one instance each of the twelve parts in `meshes/plane`, unassembled.
- Parts rest on a horizontal plane in front of the camera, each moving on its
  own smooth, deterministic path and turning about the vertical axis.
- Limits, so that `ObjectMatcher` keeps each part's identity once the world path
  exists: speed at most 5 cm/s, turn rate at most 15 °/s, part centres at least
  0.3 m apart, every part centre inside the image.
- 30 s at 10 Hz: passes 0 … 299, pass *n* scheduled at start + *n* × 100 ms.

## Playback

- The sequence starts at the first time the reader asks for data.
- A pass is available once its scheduled time has come. Every pass is emitted in
  order with its scheduled time, even when the reader ticks late (ADR-0001).
- After pass 299 no more data is available; the reader waits as it does for a
  silent camera.

## Acceptance

- Each scenario's descriptor builds a pipeline whose reader uses the synthetic
  source.
- For chosen passes, the CAS holds exactly one hypothesis per part with its
  label and scripted camera-frame pose, and the scheduled capture time.
- A late reader still gets passes 0 … 299 in order; none after.
- Nothing is added to the pipeline's world.
- Both scenarios respect the speed, turn-rate, spacing and visibility limits at
  every pass.
